#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=========================================="
echo " Starting ROS 2 Humble & Gazebo Fortress"
echo "=========================================="

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    echo "Warning: install/setup.bash not found. Did you run ./scripts/build.sh?"
fi

# Set Gazebo Fortress (Ignition) resource paths
export IGN_GAZEBO_RESOURCE_PATH="$WORKSPACE_ROOT/src/sim_env/models:$WORKSPACE_ROOT/src/sim_env/worlds:$IGN_GAZEBO_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH"

echo "Launching sim_env main.launch.py..."
ros2 launch sim_env main.launch.py "$@"
