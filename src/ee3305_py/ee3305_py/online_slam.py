"""Live SLAM costmap, map-frame pose adapter, and goal-directed exploration."""
from copy import deepcopy
from math import hypot, isfinite
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from .online_grid import Grid, inflate, reachable, select_target


def grid_from_message(msg):
    q = msg.info.origin.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
        raise ValueError('Only axis-aligned occupancy maps are supported')
    return Grid(msg.info.width, msg.info.height, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y, list(msg.data))


class SlamPose(Node):
    """Adapt Cartographer's map→base TF to the existing Odometry interface.

    This does not publish TF or feed the derived pose back to Cartographer.
    Its input remains the simulator's original odometry and scan streams.
    """
    def __init__(self):
        super().__init__('slam_pose')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('transform_timeout', 1.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Odometry, 'slam_pose', 10)
        self.create_timer(.05, self.update)

    def update(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', self.get_parameter('base_frame').value, Time())
        except TransformException:
            return
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age < -.1 or age > self.get_parameter('transform_timeout').value:
            return
        msg = Odometry()
        msg.header = tf.header
        msg.child_frame_id = self.get_parameter('base_frame').value
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        self.publisher.publish(msg)


class OnlineNavigator(Node):
    def __init__(self):
        super().__init__('online_navigator')
        for name, value in [('robot_radius', .18), ('inflation_radius', .4),
                            ('map_timeout', 4.), ('pose_timeout', .5),
                            ('stall_timeout', 20.), ('goal_tolerance', .15),
                            ('frontier_stand_off', .45), ('goal_weight', 1.)]:
            self.declare_parameter(name, value)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Volatile subscription receives periodic maps from either volatile or
        # transient-local SLAM publishers, without a durability mismatch.
        self.create_subscription(OccupancyGrid, 'map', self.on_map, 1)
        self.create_subscription(Odometry, 'slam_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.create_subscription(Path, 'path', self.on_path, 10)
        self.cost_pub = self.create_publisher(OccupancyGrid, 'global_costmap', latch)
        self.request_pub = self.create_publisher(Path, 'path_request', 10)
        self.active_pub = self.create_publisher(Bool, 'navigation_active', 10)
        self.status_pub = self.create_publisher(String, 'exploration_status', latch)
        self.frontier_pub = self.create_publisher(PoseStamped, 'exploration_target', 10)
        self.map_msg = self.pose = self.goal = self.target = None
        self.raw = self.cost = None
        self.map_received = self.pose_received = 0.
        self.map_dirty = False
        self.excluded = []
        self.progress_pose = None
        self.progress_time = monotonic()
        self.awaiting_path = False
        self.request_time = 0.
        self.failed_target = False
        self.last_request_stamp = None
        self.last_status = None
        self.create_timer(.5, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def status(self, text):
        if text != self.last_status:
            self.get_logger().info(text)
            self.status_pub.publish(String(data=text))
            self.last_status = text

    def on_map(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().error('SLAM map must use the map frame')
            return
        self.map_msg = msg
        self.map_received = monotonic()
        self.map_dirty = True

    def on_pose(self, msg):
        if msg.header.frame_id != 'map':
            return
        p = msg.pose.pose.position
        if not isfinite(p.x) or not isfinite(p.y):
            return
        self.pose = p.x, p.y
        self.pose_received = monotonic()

    def stop(self, reason):
        self.active_pub.publish(Bool(data=False))
        cancel = Path()
        cancel.header.frame_id = 'map'
        self.request_pub.publish(cancel)
        self.awaiting_path = False
        self.status(reason)

    def on_goal(self, msg):
        p = msg.pose.position
        if msg.header.frame_id != 'map' or not isfinite(p.x) or not isfinite(p.y):
            self.get_logger().warn('Goal must contain finite coordinates in the map frame')
            return
        self.stop('New goal: waiting for a fresh plan')
        self.goal = p.x, p.y
        self.target = None
        self.excluded = []
        self.failed_target = False
        self.progress_pose = self.pose
        self.progress_time = monotonic()

    def on_path(self, msg):
        if self.last_request_stamp is None or msg.header.stamp != self.last_request_stamp:
            return
        self.awaiting_path = False
        if not msg.poses:
            self.failed_target = True

    def tick(self):
        now = monotonic()
        if (self.map_msg is None or self.pose is None
                or now - self.map_received > self.param('map_timeout')
                or now - self.pose_received > self.param('pose_timeout')):
            self.stop('Waiting for fresh SLAM map and map-frame pose')
            return
        if self.map_dirty:
            try:
                self.raw = grid_from_message(self.map_msg)
                self.cost = inflate(self.raw, self.param('robot_radius'),
                                    self.param('inflation_radius'))
            except ValueError as exc:
                self.stop(str(exc))
                return
            msg = deepcopy(self.map_msg)
            msg.data = self.cost.data
            self.cost_pub.publish(msg)
            self.map_dirty = False
        if self.goal is None:
            self.stop('Ready: select a goal in RViz or publish goal_pose in map coordinates')
            return
        if hypot(self.pose[0] - self.goal[0], self.pose[1] - self.goal[1]) <= self.param('goal_tolerance'):
            self.goal = self.target = None
            self.stop('Goal reached')
            return
        if self.progress_pose is None or hypot(self.pose[0] - self.progress_pose[0],
                                               self.pose[1] - self.progress_pose[1]) > .12:
            self.progress_pose, self.progress_time = self.pose, now
        self.excluded = [(xy, until) for xy, until in self.excluded if until > now]
        if self.target and (self.failed_target or now - self.progress_time > self.param('stall_timeout')
                            or hypot(self.pose[0] - self.target[0], self.pose[1] - self.target[1]) < .16):
            self.excluded.append((self.target, now + 45.))
            self.target = None
            self.failed_target = False
            self.progress_time = now
        cells = reachable(self.cost, self.cost.cell(*self.pose))
        if not cells:
            self.stop('Robot footprint is not inside known traversable space; waiting for SLAM')
            return
        direct = self.cost.cell(*self.goal) in cells
        if direct:
            self.target = self.goal
        elif self.target is None or self.cost.cell(*self.target) not in cells:
            chosen = select_target(self.raw, self.cost, self.pose, self.goal,
                                   [xy for xy, _ in self.excluded],
                                   self.param('goal_weight'),
                                   stand_off=self.param('frontier_stand_off'))
            self.target = chosen[0] if chosen else None
            self.progress_time = now
        if self.target is None:
            self.stop('No reachable frontier: goal not yet reachable (or exploration exhausted)')
            return
        # Correlate planner responses; don't flood an in-progress search.
        if self.awaiting_path:
            if now - self.request_time < 2.:
                self.active_pub.publish(Bool(data=True))
                return
            self.awaiting_path = False
        req = Path()
        req.header.frame_id = 'map'
        req.header.stamp = self.get_clock().now().to_msg()
        self.last_request_stamp = req.header.stamp
        for x, y in [self.pose, self.target]:
            p = PoseStamped()
            p.header = req.header
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.w = 1.
            req.poses.append(p)
        self.request_pub.publish(req)
        self.frontier_pub.publish(req.poses[-1])
        self.awaiting_path = True
        self.request_time = now
        self.active_pub.publish(Bool(data=True))
        self.status('Navigating to final goal' if direct else 'Exploring a reachable frontier toward the goal')


def _run(node_type, args):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def pose_main(args=None):
    _run(SlamPose, args)


def main(args=None):
    _run(OnlineNavigator, args)
