"""Fail-closed command gate for the SLAM navigation launch only."""
from math import atan2, cos, hypot, isfinite, sin
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

from .online_grid import trajectory_clear
from .online_slam import grid_from_message, _run


class OnlineSafety(Node):
    def __init__(self):
        super().__init__('online_safety')
        for name, value in [('robot_radius', .18), ('map_timeout', 4.),
                            ('sensor_timeout', .6), ('command_timeout', .3),
                            ('prediction_horizon', .8), ('base_frame', 'base_footprint')]:
            self.declare_parameter(name, value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(TwistStamped, 'cmd_vel_unchecked', self.on_command, 10)
        self.create_subscription(Odometry, 'slam_pose', self.on_pose, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, 'navigation_active', self.on_active, 10)
        self.create_subscription(OccupancyGrid, 'global_costmap', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.command = self.pose = self.grid = None
        self.points = []
        self.active = False
        self.times = dict.fromkeys(['command', 'pose', 'scan', 'map', 'active'], 0.)
        self.create_timer(.05, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def on_command(self, msg):
        self.command = msg
        self.times['command'] = monotonic()

    def on_active(self, msg):
        self.active = msg.data
        self.times['active'] = monotonic()

    def stamp_fresh(self, stamp, limit):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return -.1 <= age <= limit

    def on_pose(self, msg):
        if msg.header.frame_id != 'map' or not self.stamp_fresh(msg.header.stamp, self.param('sensor_timeout')):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if not all(isfinite(v) for v in (p.x, p.y, yaw)):
            return
        self.pose = p.x, p.y, yaw
        self.times['pose'] = monotonic()

    def on_map(self, msg):
        if msg.header.frame_id != 'map' or not self.stamp_fresh(msg.header.stamp, self.param('map_timeout')):
            return
        try:
            self.grid = grid_from_message(msg)
        except ValueError:
            self.grid = None
            return
        self.times['map'] = monotonic()

    def on_scan(self, msg):
        if not self.stamp_fresh(msg.header.stamp, self.param('sensor_timeout')):
            return
        try:
            tf = self.buffer.lookup_transform(self.param('base_frame'), msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        q, tr = tf.transform.rotation, tf.transform.translation
        yaw = atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        points, valid = [], 0
        for i, distance in enumerate(msg.ranges):
            if distance == float('inf'):
                valid += 1
            elif isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment + yaw
                points.append((tr.x + distance*cos(angle), tr.y + distance*sin(angle)))
                valid += 1
        if not valid:
            return
        self.points = points
        self.times['scan'] = monotonic()

    def allowed(self):
        now = monotonic()
        if not self.active or self.command is None or self.pose is None or self.grid is None:
            return False
        limits = {'active': 1.5, 'command': self.param('command_timeout'),
                  'map': self.param('map_timeout'), 'pose': self.param('sensor_timeout'),
                  'scan': self.param('sensor_timeout')}
        if any(now - self.times[name] > limit for name, limit in limits.items()):
            return False
        v, w = self.command.twist.linear.x, self.command.twist.angular.z
        if not isfinite(v) or not isfinite(w) or abs(v) > .2 or abs(w) > 2.:
            return False
        horizon = self.param('prediction_horizon')
        if not trajectory_clear(self.grid, self.pose, v, w, horizon):
            return False
        # Scan endpoints are expressed in the current robot frame. Include a
        # margin for scan age and discretisation; test the whole command arc.
        radius = self.param('robot_radius') + .04 + abs(v) * (now - self.times['scan'])
        for k in range(17):
            t = horizon * k / 16
            if abs(w) < 1e-8:
                x, y = v*t, 0.
            else:
                x, y = v/w*sin(w*t), v/w*(1-cos(w*t))
            if any(hypot(px-x, py-y) <= radius for px, py in self.points):
                return False
        return True

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.param('base_frame')
        if self.allowed():
            msg.twist = self.command.twist
        self.publisher.publish(msg)


def main(args=None):
    _run(OnlineSafety, args)
