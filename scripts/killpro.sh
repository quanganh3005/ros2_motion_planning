#!/bin/bash
echo "Stopping ROS 2 and Gazebo Fortress (Ignition) processes..."
killall -9 ign 2>/dev/null || true
killall -9 ruby 2>/dev/null || true
killall -9 parameter_bridge 2>/dev/null || true
killall -9 robot_state_publisher 2>/dev/null || true
killall -9 rviz2 2>/dev/null || true
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
killall -9 ros2 2>/dev/null || true
echo "All processes stopped."
