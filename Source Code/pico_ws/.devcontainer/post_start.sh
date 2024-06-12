#!/usr/bin/env bash
# Devcontainer post-start script.
# This script runs every time the devcontainer is started.

set -e
echo "--> post_start.sh started!"
echo "-> Setting up environment..."

echo "-> Sourcing ROS..."
source /opt/ros/"$ROS_DISTRO"/setup.bash

echo "-> Colcon build..."
cd $HOME/pico_ws/libmicroros/src/micro_ros_setup && colcon build
source $HOME/pico_ws/libmicroros/src/micro_ros_setup/install/local_setup.bash
cd $HOME/pico_ws

echo "--> post_start.sh done!"