#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=========================================="
echo " Building for ROS 2 Humble & Gazebo Fortress"
echo " Workspace: $WORKSPACE_ROOT"
echo "=========================================="

cd "$WORKSPACE_ROOT"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

echo "Running colcon build for motion planning packages..."
colcon build --symlink-install --packages-select common system_config path_planner sim_env

echo "=========================================="
echo " Build successful!"
echo " Please run: source install/setup.bash"
echo " To start simulation: ./scripts/main.sh"
echo "=========================================="
