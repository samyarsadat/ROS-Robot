#!/bin/bash
ROS_DISTRO="jazzy"
set -e

SOURCE_CODE_PATH="$HOME/ros_robot/source_code"
source /opt/vulcanexus/$ROS_DISTRO/setup.bash
source "$SOURCE_CODE_PATH/pico_ws/libmicroros/install/local_setup.sh"
source "$SOURCE_CODE_PATH/ros_ws_robot_infra/install/local_setup.sh"
source "$SOURCE_CODE_PATH/ros_camera_ws/install/local_setup.sh"
source "$HOME"/roslaunch/install/setup.bash

ros2 launch ros_robot_bringup infra.launch.py namespace:="/$ROBOT_NAMESPACE"