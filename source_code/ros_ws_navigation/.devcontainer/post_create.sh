#!/usr/bin/env bash
# Devcontainer post-create script.
# This script is run after the devcontainer is created.

set -e
echo "--> post_create.sh started!"
echo "-> First container run, running setup..."

echo "-> Setting folder permissions and copying files..."
sudo chown -R nonroot: $HOME/ros_ws/

echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> $HOME/.bashrc
echo "source '$HOME/ros_ws/install/local_setup.bash'" >> $HOME/.bashrc

# Temporary ROS Launch fix.
#bash /roslaunch_fix.sh

echo "--> post_create.sh done!"