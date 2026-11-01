from heapq import heappush, heappop
from math import hypot, floor, inf
from time import perf_counter

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        return self.g < other.g


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Select the planning algorithm: "dijkstra" or "astar".
        self.declare_parameter("algorithm", "dijkstra")
        self.algorithm_ = self.get_parameter("algorithm").value.lower()

        if self.algorithm_ not in ("dijkstra", "astar"):
            self.get_logger().warn(
                f"Unknown algorithm '{self.algorithm_}'. Using Dijkstra."
            )
            self.algorithm_ = "dijkstra"

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # !TODO: Path request subscriber ##DONE
        self.sub_path_request_ = self.create_subscription(
            Path,
            "path_request",
            self.callbackSubPathRequest_,
            10,
        )

        # Handles: Publishers
        # !TODO: Path publisher ##DONE
        self.pub_path_ = self.create_publisher(
            Path,
            "path",
            10,
        )

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):   
        # !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_  ##DONE
        # A valid request must contain both start and goal poses.
        if len(msg.poses) < 2:
            self.get_logger().warn(
                "Invalid path request: expected start and goal poses."
            )
            return

        # The first pose contains the robot's current position.
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y

        # The second pose contains the destination.
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y

        # Inform the timer that a new planning request is available.
        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        
        # !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_ ##DONE
        self.costmap_ = msg.data
        self.costmap_resolution_ = msg.info.resolution
        
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        c = floor((x - self.costmap_origin_x_) / self.costmap_resolution_)
        r = floor((y - self.costmap_origin_y_) / self.costmap_resolution_)

        return c, r

    # Converts cell column and cell row to world coordinates.
    def CRToXY_(self, c, r):
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_

        return x, y

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        return int(r * self.costmap_cols_ + c)

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        return (c < 0 or 
                c >= self.costmap_cols_ or 
                r < 0 or 
                r >= self.costmap_rows_)

    def heuristic_(self, c, r, goal_c, goal_r):
        """Octile distance for an 8-connected grid."""
        dx = abs(goal_c - c)
        dy = abs(goal_r - r)

        return max(dx, dy) + (2**0.5 - 1) * min(dx, dy)

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
        # Initializations ---------------------------------
        # Initialize nodes
        nodes = [DijkstraNode(c, r)
        for r in range(self.costmap_rows_)
        for c in range(self.costmap_cols_)]

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)  # replace this ##DONE

        if self.outOfMap_(rbt_c, rbt_r) or self.outOfMap_(goal_c, goal_r):
            self.get_logger().warn("Start or goal is outside the costmap!")
            return

        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        goal_idx = self.CRToIndex_(goal_c, goal_r)

        if (self.costmap_[rbt_idx] < 0 or 
            self.costmap_[rbt_idx] > self.max_access_cost_ or 
            self.costmap_[goal_idx] < 0 or 
            self.costmap_[goal_idx] > self.max_access_cost_):

            self.get_logger().warn("Start or goal is inaccessible!")
            return
        
        start_node = nodes[rbt_idx]
        start_node.g = 0.0

        # Dijkstra uses h = 0; A* uses an octile-distance heuristic.
        use_astar = self.algorithm_ == "astar"

        if use_astar:
            start_node.h = self.heuristic_(
                rbt_c, rbt_r, goal_c, goal_r
            )
        else:
            start_node.h = 0.0

        start_node.f = start_node.g + start_node.h

        open_list = []
        push_count = 0

        # Priority queue is ordered by f = g + h.
        heappush(open_list, (start_node.f, push_count, start_node))

        #timer for performance checking
        start_time = perf_counter()
        expanded_count = 0

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            queued_f, _, node = heappop(open_list)

            # Skip if visited
            if node.expanded or queued_f > node.f:
                continue

            node.expanded = True
            expanded_count += 1

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                current_node = node

                # obtain the path from the goal to the start.
                while current_node is not None:
                    # Convert grid cell into world coordinates.
                    x, y = self.CRToXY_(current_node.c, current_node.r)

                    # Create a waypoint for the controller.
                    pose = PoseStamped()
                    pose.header = msg_path.header
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.orientation.w = 1.0

                    msg_path.poses.append(pose)

                    # Move to the previous node in the path.
                    current_node = current_node.parent

                # Reverse to obtain start -> goal.
                msg_path.poses.reverse()

                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )

                elapsed_ms = (perf_counter() - start_time) * 1000
                self.get_logger().info(
                    f"{self.algorithm_.upper()}: "
                    f"expanded={expanded_count}, "
                    f"time={elapsed_ms:.3f} ms, "
                    f"path_cost={node.g:.3f}, "
                    f"waypoints={len(msg_path.poses)}"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = node.c + dc
                nb_r = node.r + dr

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Prevent diagonal corner-cutting between inaccessible cells.
                if dc != 0 and dr != 0:
                    side1_idx = self.CRToIndex_(node.c + dc, node.r)
                    side2_idx = self.CRToIndex_(node.c, node.r + dr)

                    side1_cost = self.costmap_[side1_idx]
                    side2_cost = self.costmap_[side2_idx]

                    if (side1_cost < 0 or side1_cost > self.max_access_cost_
                            or side2_cost < 0 or side2_cost > self.max_access_cost_):
                        continue
                    
                # Get the neighbor node
                nb_idx = self.CRToIndex_(nb_c, nb_r)
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_node.expanded:
                    continue

                # Retrieve the neighbor's costmap value.
                cell_cost = self.costmap_[nb_idx]
                
                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                if cell_cost < 0 or cell_cost > self.max_access_cost_:
                    continue

                distance = hypot(dc, dr)
                new_g = node.g + distance * (cell_cost + 1)

                # Get the relative g-cost and push to open-list
                if new_g < nb_node.g:
                    nb_node.g = new_g
                    # Remember which node led to this neighbor.
                    nb_node.parent = node

                    if use_astar:
                        nb_node.h = self.heuristic_(
                            nb_c, nb_r, goal_c, goal_r
                        )
                    else:
                        nb_node.h = 0.0

                    nb_node.f = nb_node.g + nb_node.h

                    # Add neighbor to the priority queue.
                    push_count += 1
                    heappush(open_list, (nb_node.f, push_count, nb_node))

        #self.get_logger().warn("No Path Found!")
        elapsed_ms = (perf_counter() - start_time) * 1000
        self.get_logger().warn(
            f"{self.algorithm_.upper()}: No Path Found! "
            f"expanded={expanded_count}, "
            f"time={elapsed_ms:.3f} ms"
        )


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
