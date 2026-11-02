#!/usr/bin/env bash
# Build and start the opt-in mode. No existing config/map/world is rewritten.
set -e -o pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build=true
build_only=false
launch_args=()

usage() {
    cat <<'EOF'
Usage: bash scripts/run_online_slam.sh [options] [ROS launch arguments]

Options:
  --no-build     Use an existing build
  --build-only   Build the new mode without starting a simulation
  --headless     Run Gazebo without its GUI
  --no-rviz      Do not open RViz
  --astar        Use the team's A* option instead of Dijkstra
  --virtualbox   Enable the existing VirtualBox rendering option
  -h, --help     Show this help

Examples:
  bash scripts/run_online_slam.sh
  bash scripts/run_online_slam.sh --astar --headless
  bash scripts/run_online_slam.sh --no-build world:=turtlebot3_world

Requires Ubuntu and ROS2 Jazzy. Select a map-frame goal in RViz after startup.
This starts a separate mode; do not run another robot simulation at the same time.
EOF
}

while (($#)); do
    case "$1" in
        --no-build) build=false ;;
        --build-only) build_only=true ;;
        --headless) launch_args+=(headless:=True) ;;
        --no-rviz) launch_args+=(rviz:=False) ;;
        --astar) launch_args+=(algorithm:=astar) ;;
        --virtualbox) launch_args+=(libgl:=True) ;;
        -h|--help) usage; exit 0 ;;
        *:=*) launch_args+=("$1") ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

ros_setup="/opt/ros/jazzy/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
    printf 'ROS2 Jazzy setup not found: %s\nRun this script on your Ubuntu ROS2 machine.\n' "$ros_setup" >&2
    exit 1
fi
# ROS environment scripts may reference unset variables, so nounset is not used.
source "$ros_setup"
cd "$workspace"

if "$build"; then
    if ! command -v colcon >/dev/null 2>&1; then
        printf 'colcon is not installed. Install python3-colcon-common-extensions first.\n' >&2
        exit 1
    fi
    colcon build --symlink-install --packages-select ee3305_py ee3305_bringup
fi
if [[ ! -f "$workspace/install/setup.bash" ]]; then
    printf 'Workspace has no install/setup.bash. Run again without --no-build.\n' >&2
    exit 1
fi
source "$workspace/install/setup.bash"
if "$build_only"; then
    printf 'Online SLAM mode built. Existing launch modes and maps were not changed.\n'
    exit 0
fi
exec ros2 launch ee3305_bringup run_online_slam.launch.py "${launch_args[@]}"
