#!/bin/bash
set -e

cd "$(git rev-parse --show-toplevel)/source_code/ros_camera_ws" || exit 1
sudo apt -y install python3-colcon-meson
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera