"""Opt-in adapters around the unchanged course planner and controller.

Only the online SLAM launch uses these entry points. The original executables,
algorithms, launch files, parameters, maps, and worlds are not modified.
"""
from math import atan2, cos, hypot, isfinite, sin
from time import monotonic

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

from .planner import Planner
from .controller import Controller
from .online_slam import grid_from_message, _run


class _PathPublisher:
    """Preserve request identity without changing the existing search code."""
    def __init__(self, owner, publisher):
        self.owner, self.publisher = owner, publisher

    def publish(self, msg):
        self.owner.path_published = True
        if self.owner.request_stamp is not None:
            msg.header.stamp = self.owner.request_stamp
        self.publisher.publish(msg)


class OnlinePlanner(Planner):
    def __init__(self):
        super().__init__()
        self.max_access_cost_ = min(99, max(0, self.max_access_cost_))
        self.has_request = False
        self.request_stamp = None
        self.path_published = False
        self.output_publisher = self.pub_path_
        self.pub_path_ = _PathPublisher(self, self.output_publisher)

    def empty_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        self.pub_path_.publish(msg)

    def callbackSubPathRequest_(self, msg):
        self.request_stamp = msg.header.stamp
        valid = (len(msg.poses) == 2 and msg.header.frame_id == 'map'
                 and all(isfinite(value) for value in (
                     msg.poses[0].pose.position.x, msg.poses[0].pose.position.y,
                     msg.poses[1].pose.position.x, msg.poses[1].pose.position.y)))
        if not valid:
            self.has_request = self.has_new_request_ = False
            self.empty_path()
            return
        self.has_request = True
        super().callbackSubPathRequest_(msg)

    def callbackSubGlobalCostmap_(self, msg):
        try:
            if msg.header.frame_id != 'map':
                raise ValueError('Expected map-frame costmap')
            grid_from_message(msg)
        except ValueError:
            self.received_map_ = False
            self.empty_path()
            return
        super().callbackSubGlobalCostmap_(msg)
        if self.has_request:
            self.has_new_request_ = True

    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
        self.path_published = False
        # All route search, costs, corner checks and path reconstruction remain
        # in the team's existing implementation (including its A* option).
        super().dijkstra_(start_x, start_y, goal_x, goal_y)
        if not self.path_published:
            self.empty_path()


class OnlineController(Controller):
    def __init__(self):
        super().__init__()
        self.declare_parameter('data_timeout', 2.)
        self.declare_parameter('heading_align_threshold', .7)
        self.path_received_at = self.odom_received_at = 0.

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd_vel_.publish(msg)

    def callbackSubPath_(self, msg):
        if not msg.poses or msg.header.frame_id != 'map' or not all(
                isfinite(v) for p in msg.poses for v in (p.pose.position.x, p.pose.position.y)):
            self.received_path_ = False
            self.path_poses_ = []
            self.stop()
            return
        super().callbackSubPath_(msg)
        self.path_received_at = monotonic()

    def callbackSubOdom_(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.header.frame_id != 'map' or not all(
                isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.received_odom_ = False
            self.stop()
            return
        super().callbackSubOdom_(msg)
        self.odom_received_at = monotonic()

    def callbackTimer_(self):
        now = monotonic()
        if (not self.received_odom_ or not self.received_path_
                or now - self.path_received_at > self.get_parameter('data_timeout').value
                or now - self.odom_received_at > self.get_parameter('data_timeout').value):
            self.stop()
            return
        goal = self.path_poses_[-1].pose.position
        if hypot(goal.x-self.rbt_x_, goal.y-self.rbt_y_) > self.stop_thres_:
            p = self.getLookaheadPoint_().pose.position
            dx, dy = p.x-self.rbt_x_, p.y-self.rbt_y_
            heading = atan2(-sin(self.rbt_yaw_)*dx + cos(self.rbt_yaw_)*dy,
                            cos(self.rbt_yaw_)*dx + sin(self.rbt_yaw_)*dy)
            if abs(heading) > self.get_parameter('heading_align_threshold').value:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.angular.z = max(-self.max_ang_vel_, min(self.max_ang_vel_, heading))
                self.pub_cmd_vel_.publish(msg)
                return
        # The original lookahead, curvature, velocity scaling and stopping law.
        super().callbackTimer_()


def planner_main(args=None):
    _run(OnlinePlanner, args)


def controller_main(args=None):
    _run(OnlineController, args)
