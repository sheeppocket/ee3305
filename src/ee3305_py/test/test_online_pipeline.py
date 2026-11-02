"""Exercise the real planner/controller callbacks without requiring ROS binaries.

These tests verify algorithm and state transitions. They do not validate DDS,
Gazebo sensors, Cartographer, or the installed ROS launch API.
"""
import sys
import unittest
from math import cos, sin
from pathlib import Path as FilePath
from time import monotonic
from unittest.mock import patch

sys.path.insert(0, str(FilePath(__file__).resolve().parents[1]))
from ee3305_py.online_grid import Grid, inflate, select_target, trajectory_clear
from ros_fakes import load_modules, map_message, request, Path, Odometry, TwistStamped, Stamp

planner_module, controller_module, slam_module, safety_module = load_modules()


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.planner = planner_module.Planner()
        self.controller = controller_module.Controller()

    def plan(self, grid, start, goal):
        self.planner.callbackSubGlobalCostmap_(map_message(grid))
        self.planner.callbackSubPathRequest_(request(start, goal))
        self.planner.callbackTimer_()
        return self.planner.output_publisher.messages[-1]

    def test_both_existing_algorithms_use_updated_origin_and_dimensions(self):
        for algorithm in ['dijkstra', 'astar']:
            self.planner.algorithm_ = algorithm
            for size, origin in [(15, -1.), (25, -2.)]:
                g = Grid(size, size, .2, origin, origin, [0] * size**2)
                path = self.plan(g, (0., 0.), (.6, .6))
                self.assertTrue(path.poses)
                self.assertAlmostEqual(path.poses[-1].pose.position.x, .7)

    def test_new_obstacle_cancels_old_path_and_controller_stops(self):
        g = Grid(10, 10, .1, 0., 0., [0] * 100)
        path = self.plan(g, (.25, .25), (.75, .25))
        self.controller.callbackSubPath_(path)
        self.assertTrue(self.controller.received_path_)
        for r in range(10):
            g.data[g.index(5, r)] = 100
        self.planner.callbackSubGlobalCostmap_(map_message(g))
        self.planner.callbackTimer_()  # automatically replans on a map update
        self.controller.callbackSubPath_(self.planner.output_publisher.messages[-1])
        self.assertFalse(self.controller.received_path_)
        self.assertEqual(self.controller.pub_cmd_vel_.messages[-1].twist.linear.x, 0.)

    def test_explicit_cancel_prevents_map_update_resurrecting_old_goal(self):
        g = Grid(10, 10, .1, 0., 0., [0] * 100)
        self.plan(g, (.25, .25), (.75, .25))
        self.planner.callbackSubPathRequest_(Path())
        self.planner.callbackSubGlobalCostmap_(map_message(g))
        self.planner.callbackTimer_()
        self.assertFalse(self.planner.output_publisher.messages[-1].poses)
        self.assertFalse(self.planner.has_request)

    def test_controller_rotates_toward_rear_target_and_rejects_stale_pose(self):
        c = self.controller
        c.parameters['heading_align_threshold'] = .7
        c.parameters['data_timeout'] = 2.
        path = Path()
        path.header.frame_id = 'map'
        path.poses = request((0., 0.), (-1., 0.)).poses
        c.callbackSubPath_(path)
        odom = Odometry(); odom.header.frame_id = 'map'
        c.callbackSubOdom_(odom)
        c.callbackTimer_()
        cmd = c.pub_cmd_vel_.messages[-1].twist
        self.assertEqual(cmd.linear.x, 0.)
        self.assertNotEqual(cmd.angular.z, 0.)
        c.odom_received_at = monotonic() - 3.
        c.callbackTimer_()
        self.assertEqual(c.pub_cmd_vel_.messages[-1].twist.angular.z, 0.)

    def test_goal_outside_initial_map_produces_frontier_request(self):
        n = slam_module.OnlineNavigator()
        raw = Grid(40, 30, .1, -2., -1.5, [-1] * 1200)
        for r in range(4, 26):
            for c in range(4, 23):
                raw.data[raw.index(c, r)] = 0
        n.on_map(map_message(raw))
        pose = Odometry(); pose.header.frame_id = 'map'
        pose.pose.pose.position.x = -.8
        n.on_pose(pose)
        goal = request((0., 0.), (3., 0.)).poses[-1]
        goal.header.frame_id = 'map'
        n.on_goal(goal)
        n.tick()
        req = n.request_pub.messages[-1]
        self.assertEqual(len(req.poses), 2)
        self.assertLess(req.poses[-1].pose.position.x, 1.)
        self.assertTrue(n.active_pub.messages[-1].data)
        n.map_received = monotonic()-10
        n.tick()
        self.assertFalse(n.active_pub.messages[-1].data)
        self.assertFalse(n.request_pub.messages[-1].poses)

    def test_safety_rejects_stale_scan_and_predicted_obstacle(self):
        guard = safety_module.OnlineSafety()
        guard.active = True
        guard.pose = (0., 0., 0.)
        guard.grid = Grid(30, 30, .05, -.5, -.5, [0]*900)
        guard.command = TwistStamped()
        guard.command.twist.linear.x = .1
        guard.times = {name: monotonic() for name in guard.times}
        self.assertTrue(guard.allowed())
        guard.times['scan'] -= 2.
        self.assertFalse(guard.allowed())
        guard.times['scan'] = monotonic()
        guard.points = [(.25, 0.)]
        self.assertFalse(guard.allowed())

    def test_incremental_mapping_reaches_goal_with_existing_planner_and_pursuit(self):
        # Synthetic sensing/kinematics, not SLAM validation: initially unknown
        # map is revealed locally as the real Pure Pursuit controller moves.
        raw = Grid(80, 40, .1, -2., -2., [-1]*3200)
        robot, goal, yaw = [-.5, 0.], (4., 0.), 0.
        c = self.controller
        c.parameters['heading_align_threshold'] = .7
        c.lookahead_distance_ = .2
        frontier_count = 0
        for step in range(1100):
            if step % 10 == 0:
                for r in range(raw.height):
                    for col in range(raw.width):
                        x, y = raw.xy(col, r)
                        if (x-robot[0])**2+(y-robot[1])**2 < 1.2**2:
                            raw.data[raw.index(col, r)] = 0
                cost = inflate(raw)
                target = select_target(raw, cost, robot, goal, stand_off=.45)
                self.assertIsNotNone(target)
                frontier_count += not target[1]
                c.callbackSubPath_(self.plan(cost, robot, target[0]))
            odom = Odometry()
            odom.header.frame_id = 'map'
            odom.pose.pose.position.x, odom.pose.pose.position.y = robot
            odom.pose.pose.orientation.z = sin(yaw/2)
            odom.pose.pose.orientation.w = cos(yaw/2)
            c.callbackSubOdom_(odom)
            c.callbackTimer_()
            cmd = c.pub_cmd_vel_.messages[-1].twist
            robot[0] += cmd.linear.x*cos(yaw)*.05
            robot[1] += cmd.linear.x*sin(yaw)*.05
            yaw += cmd.angular.z*.05
            if ((robot[0]-goal[0])**2+(robot[1]-goal[1])**2)**.5 < .15:
                break
        else:
            self.fail(f'Incremental navigation stalled at {robot}')
        self.assertGreater(frontier_count, 0)

    def test_unknown_wall_requires_discovering_an_offset_doorway(self):
        raw = Grid(70, 50, .1, -2., -2.5, [-1]*3500)
        robot, goal, yaw = [-.5, 0.], (3.5, 0.), 0.
        controller = self.controller
        controller.lookahead_distance_ = .2
        controller.parameters['heading_align_threshold'] = .7
        simulation_time = [10.]
        time_patch = patch.object(slam_module, 'monotonic', lambda: simulation_time[0])
        time_patch.start()
        self.addCleanup(time_patch.stop)
        navigator = slam_module.OnlineNavigator()
        goal_msg = request(robot, goal).poses[-1]
        goal_msg.header.frame_id = 'map'
        navigator.on_goal(goal_msg)
        cost = None
        maximum_y = 0.
        for step in range(4000):
            simulation_time[0] = 10 + step*.05
            if step % 10 == 0:
                # Ray observations reveal free cells only up to the first wall.
                for ray in range(180):
                    angle = ray * 6.283185307179586 / 180
                    for distance_step in range(33):
                        distance = distance_step*.05
                        x, y = robot[0]+distance*cos(angle), robot[1]+distance*sin(angle)
                        col, row = raw.cell(x, y)
                        if not raw.inside(col, row):
                            break
                        blocked = col == 35 and not 31 <= row <= 40
                        raw.data[raw.index(col, row)] = 100 if blocked else 0
                        if blocked:
                            break
            odom = Odometry()
            odom.header.frame_id = 'map'
            odom.pose.pose.position.x, odom.pose.pose.position.y = robot
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = sin(yaw/2), cos(yaw/2)
            controller.callbackSubOdom_(odom)
            navigator.on_pose(odom)
            if step % 10 == 0:
                navigator.on_map(map_message(raw))
                navigator.tick()
                cost = navigator.cost
                self.planner.callbackSubGlobalCostmap_(map_message(cost))
                self.planner.callbackSubPathRequest_(navigator.request_pub.messages[-1])
                self.planner.callbackTimer_()
                path = self.planner.output_publisher.messages[-1]
                navigator.on_path(path)
                controller.callbackSubPath_(path)
            controller.callbackTimer_()
            command = controller.pub_cmd_vel_.messages[-1].twist
            v, w = command.linear.x, command.angular.z
            if not trajectory_clear(cost, (*robot, yaw), v, w):
                v = w = 0.
            robot[0] += v*cos(yaw)*.05
            robot[1] += v*sin(yaw)*.05
            yaw += w*.05
            maximum_y = max(maximum_y, robot[1])
            if ((robot[0]-goal[0])**2+(robot[1]-goal[1])**2)**.5 < .15:
                break
        else:
            self.fail(f'Doorway navigation stalled at {robot}; target={navigator.target}; status={navigator.last_status}; yaw={yaw}')
        self.assertGreater(maximum_y, .6)


if __name__ == '__main__':
    unittest.main()
