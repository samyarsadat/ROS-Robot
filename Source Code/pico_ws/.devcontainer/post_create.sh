#!/usr/bin/env bash
# Devcontainer post-create script.
# This script is run after the devcontainer is created.

set -e
echo "--> post_create.sh started!"
echo "-> First container run, running MicroROS tools setup..."

echo "-> Setting folder permissions and copying files..."
sudo chown -R urosdev: $HOME/pico_ws/
cp "/pico/pico-sdk/external/pico_sdk_import.cmake" $HOME/pico_ws/pico_sdk_import.cmake
cp "$HOME/pico_ws/libfreertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake" $HOME/pico_ws/FreeRTOS_Kernel_import.cmake
    
echo "-> Installing MicroROS tools..."
sudo apt-get update \
&& rosdep update \
&& cd $HOME/pico_ws/libmicroros \
&& sudo rosdep install --from-paths src --ignore-src -y \
&& sudo apt-get autoremove && sudo apt-get autoclean \
&& echo "-> Tools installed!"

# These were moved here because they only need to be run once!
echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> $HOME/.bashrc
echo "source '$HOME/pico_ws/libmicroros/src/micro_ros_setup/install/local_setup.bash'" >> $HOME/.bashrc

echo "--> post_create.sh done!"