# Online SLAM navigation

This optional Python mode starts Cartographer without a saved map. The robot
waits for an initial scan-derived map and pose, then explores toward a selected
goal while updating the map. Unobserved space remains **unknown**, not free.
No saved occupancy map or Gazebo ground-truth pose is supplied to navigation.
Gazebo still simulates the environment and provides scan/odometry inputs to SLAM.

The existing `run.launch.py` remains the entry point for the saved-map project.
Do not run it, teleoperation, or another velocity-command publisher alongside
the online mode.

## Build and launch on Ubuntu 24.04 with ROS2 Jazzy

From the `ee3305` workspace containing `src`:

```bash
bash scripts/run_online_slam.sh
```

The script sources ROS2 Jazzy, builds the Python and bringup packages, and
starts only the new launch mode. It can be called from any working directory
using its absolute path. It does not rewrite existing simulation files or maps.

```bash
bash scripts/run_online_slam.sh --astar --headless
bash scripts/run_online_slam.sh --no-build --virtualbox
bash scripts/run_online_slam.sh --build-only
bash scripts/run_online_slam.sh --help
```

To install dependencies once and build/launch manually:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-cartographer
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ee3305_py ee3305_bringup
source install/setup.bash
ros2 launch ee3305_bringup run_online_slam.launch.py
```

For VirtualBox, add `libgl:=True`. For a simulation without GUIs, add
`headless:=True rviz:=False`. Switch the existing planner to A* using
`algorithm:=astar`; the default is the team's Dijkstra implementation.
The defaults target TurtleBot3 Burger. Match both `robot_radius` parameters to
the footprint before using a different robot model.

Wait for `/map` and `/slam_pose` to become available. Then choose **2D Goal
Pose** in RViz, including in the unknown area, or publish a destination:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The example is a coordinate, not a promise that the corresponding point is
reachable in every world. Coordinates are in **Cartographer's map frame**;
they are not necessarily the same as Gazebo world coordinates or coordinates
from an earlier mapping session. A GPS goal would require a separate calibrated
geographic-to-map transformation; GPS integration is not implemented here.

No autonomous motion occurs until a goal is supplied. The map begins absent,
then immediately shows the first observed local region; it does not remain a
zero-sized grid until the robot starts moving. This is normal for live SLAM.

## Architecture

```text
Gazebo scan + original odometry + TF → Cartographer → /map + map→odom TF
                                                         │
map→base_footprint TF → slam_pose adapter → /slam_pose     │
                                              │          ▼
/goal_pose → online_navigator ← live map → inflated /global_costmap
                  │                                      │
           /path_request ────────→ existing planner ←──────┘
                                         │
                                       /path
                                         ▼
                    /slam_pose → existing Pure Pursuit controller
                                         │
                               /cmd_vel_unchecked
                                         ▼
                   map + pose + scan → online_safety → /cmd_vel
```

Cartographer owns `map→odom`. No static map-to-odom publisher or AMCL is started.
The pose adapter publishes a map-frame `Odometry` message on `/slam_pose` solely
to reuse the controller's existing interface. It neither replaces `/odom` nor
feeds its output back into Cartographer. The original behaviour node is replaced
by the online navigator in this launch; Nav2 is not used for planning or control.

## Exploration and map updates

1. Build a footprint-inflated costmap from each new SLAM occupancy grid. Preserve
   unknown cells and treat uncertain occupancy conservatively. Reload the map's
   dimensions and origin on every update.
2. Find cells reachable through observed traversable space, disallowing diagonal
   corner cutting. If the final goal is reachable, request a path directly to it.
3. Otherwise cluster observed/unknown boundaries into frontiers. Select a reachable
   waypoint near a frontier, with enough stand-off for the footprint. Rank options
   using travel distance plus distance to the final goal.
4. Send that temporary destination to the team's planner. The team's Pure Pursuit
   implementation follows the resulting path; an optional alignment step rotates
   toward routes that begin behind the robot.
5. Repeat as the map grows. Retain a target until it is reached, becomes blocked,
   fails planning, or stalls. Temporarily exclude unsuccessful/reached frontier
   targets to permit exploration of alternatives. Switch to the final goal once
   it is reachable through known space.

The reachability search supports frontier selection; it does not publish paths
or replace the existing Dijkstra/A* planner. Existing search metrics remain in
the planner logs.

Unknown space is kept out of the robot footprint as well as out of planned path
centres. Outside the current map bounds is also unknown. This is conservative
and may stop exploration in a very narrow or poorly observed area.

## Stopping and command checks

The separate `online_controller` adapter clears an empty path instead of
retaining the last route. The separate `online_planner` adapter publishes an
empty path on failure or cancellation and replans when its costmap changes.
The online controller configuration also expires old pose and path data.
Both adapters subclass the original nodes and reuse their existing algorithms.
The original `planner` and `controller` executables are unchanged, as are all
existing launch files, parameter files, worlds, and the maps at the pulled HEAD.

Only `online_safety` publishes final commands in this launch. It requires a
recent active-navigation heartbeat, command, map, SLAM pose, and usable scan.
It checks the predicted command arc against known traversable costmap cells
and checks scan endpoints against the swept circular footprint. Missing
transforms, stale data, obstacles, or unknown trajectory cells result in a zero
command. Scan endpoints are transformed from the laser frame to the base frame.

These checks are intended for this low-speed simulation; they are not a
hardware safety certification. The guard cannot guarantee that a newly moving
obstacle will be detected before impact. Loop-closure pose corrections may
also require the controller to stop and replan.

## Observe and save

```bash
ros2 topic echo /exploration_status
ros2 topic echo /exploration_target
ros2 topic hz /map
ros2 topic hz /slam_pose
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map base_footprint
```

`/cmd_vel` should have one publisher: `online_safety`. The controller publishes
`/cmd_vel_unchecked`. Add `/exploration_target` as a Pose display in RViz to see
the temporary destination. The existing RViz configuration already shows
`/map`, `/global_costmap`, `/path`, and `/lookahead`.

To save the observed occupancy map (requires `ros-jazzy-nav2-map-server`):

```bash
ros2 run nav2_map_server map_saver_cli -f online_map
```

This saves an occupancy map, not Cartographer's complete resumable SLAM state.

## Parameters

Edit `src/ee3305_bringup/params/online_slam.yaml`, or pass an alternative file
with `params_file:=/absolute/path/to/online_slam.yaml`.

| Parameter | Default | Effect |
|---|---:|---|
| Navigator and guard robot radius | 0.18 m | Circular footprint clearance; keep both consistent |
| Inflation radius | 0.40 m | Soft obstacle-clearance preference beyond the footprint |
| Frontier stand-off | 0.45 m | Search radius from raw frontiers to reachable waypoint cells |
| Goal weight | 1.0 | Preference for frontier waypoints closer to the final goal |
| Stall timeout | 20 s | Lack of position progress before trying an alternative frontier |
| Goal tolerance | 0.15 m | Final goal completion tolerance |
| Controller data timeout | 2 s | Maximum age of received pose/path before stopping |
| Map timeout | 4 s | Stop when live map updates are missing/stale |
| Guard sensor timeout | 0.6 s | Stop when pose or scan data are missing/stale |

The navigator ticks at 2 Hz; Cartographer is requested to publish occupancy
grids every 0.5 s; control and command checks run at 20 Hz. Full-grid inflation
and frontier selection run in Python, so large maps may require a lower update
rate or an optimised implementation. Slow computation causes command gating,
rather than permitting indefinitely stale motion.

## Verification and remaining integration checks

Run the automated tests without ROS from the workspace root:

```bash
python3 -m unittest discover -s src/ee3305_py/test -v
```

Tests cover unknown/uncertain space, footprint inflation, map growth/origin
changes, diagonal corner cutting, frontier selection, cancellation, stale data,
command collision prediction, and the existing planner/controller callbacks.
Synthetic mapping/kinematics scenarios include an initially out-of-map goal
and an unseen wall with an offset doorway. ROS stand-ins are used for callback
tests; these do not validate DDS, Gazebo, TF timing, or Cartographer accuracy.

Before describing this as a demonstrated project improvement, run these actual
Gazebo trials on Ubuntu:

1. Start without a saved map and verify no motion until a goal is supplied.
2. Select a reachable goal in initially unknown space. Confirm map expansion,
   frontier changes, and final goal completion.
3. Place/select a goal behind a wall with a connecting doorway. Confirm the
   robot explores alternatives instead of planning through unknown cells.
4. Select an enclosed or unreachable goal. Confirm it stops when no reachable
   frontier exists; this heuristic does not guarantee complete exploration or
   prove a goal unreachable.
5. Interrupt Cartographer, scan publication, or the planner and verify a zero
   velocity command within the configured timeouts.
6. Repeat the saved-map baseline and online mode for identical physical goals,
   aligning their map frames first. Record success, travel distance, time,
   number of replans, and map coverage.

The current development host cannot start its WSL Ubuntu VM, so no real ROS2
build, Cartographer run, or Gazebo performance result is claimed here.

Implementation follows the upstream [TurtleBot3 Cartographer launch interface](https://github.com/ROBOTIS-GIT/turtlebot3/blob/jazzy/turtlebot3_cartographer/launch/cartographer.launch.py)
and the [ROS2 Jazzy TF listener API](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html).
