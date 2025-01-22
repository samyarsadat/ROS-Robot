#!/bin/bash
set -e

SOURCE_CODE_PATH="$HOME/ros_robot/source_code"
source /opt/vulcanexus/$ROS_DISTRO/setup.bash
source "$SOURCE_CODE_PATH/pico_ws/libmicroros/install/local_setup.sh"
source "$SOURCE_CODE_PATH/ros_ws_robot_infra/install/local_setup.sh"

ros2 launch ros_robot_driver_wrapper driver_launch.py