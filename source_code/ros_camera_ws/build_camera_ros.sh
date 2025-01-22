#!/bin/bash
set -e

cd "$(git rev-parse --show-toplevel)/source_code/ros_camera_ws" || exit 1
colcon build --event-handlers=console_direct+