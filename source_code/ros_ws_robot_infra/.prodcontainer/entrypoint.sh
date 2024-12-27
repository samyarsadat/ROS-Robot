#!/bin/bash
set -e

cd "$HOME/ros_ws" || exit 1
source /opt/vulcanexus/humble/setup.bash
colcon build
source install/local_setup.sh
ros2 launch ros_robot_driver_wrapper driver_launch.py