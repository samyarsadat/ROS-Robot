#!/usr/bin/env bash
# This is a temporary fix!
# There is currently a bug in ROS 2 Launch whcih affects the robot bringup.
# PR #893 (https://github.com/ros2/launch/pull/893) adresses this issue for Jazzy,
# however, it has not yet been merged. Until it is merged, this script will allow us
# to use ROS Launch from the PR branch.

mkdir $HOME/roslaunch
cd $HOME/roslaunch

PR_NUMBER=893
git clone https://github.com/ros2/launch.git .
git fetch origin pull/$PR_NUMBER/head:pr-$PR_NUMBER
git checkout pr-$PR_NUMBER

rosdep update
sudo apt-get update
rosdep install --from-paths . --ignore-src -r -y

source /opt/ros/"$ROS_DISTRO"/setup.bash
colcon build
source "$HOME"/roslaunch/install/setup.bash

if [[ "$(ros2 pkg prefix launch)" != "$HOME/roslaunch/install/launch" ]]; then
    echo "ROS Launch fix installation failed!"
    exit 1
fi