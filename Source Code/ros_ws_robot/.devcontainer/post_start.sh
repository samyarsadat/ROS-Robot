#!/usr/bin/env bash
# Devcontainer post-start script.
# This script runs every time the devcontainer is started.

set -e
echo "--> post_start.sh started!"
echo "-> Setting up environment..."

echo "-> Sourcing ROS..."
source /opt/ros/"$ROS_DISTRO"/setup.bash

echo "-> Colcon build..."
cd $HOME/ros_ws && colcon build
source $HOME/ros_ws/install/local_setup.bash
cd $HOME/ros_ws

echo "--> post_start.sh done!"