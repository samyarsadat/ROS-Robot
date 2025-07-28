#!/bin/bash
set -e

cd "$(git rev-parse --show-toplevel)/source_code/ros_camera_ws" || exit 1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --event-handlers=console_direct+