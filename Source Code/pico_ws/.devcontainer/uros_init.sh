#!/usr/bin/env bash
# MUST BE RUN AS THE NON-ROOT USER!

set -e
echo "-> uros_init.sh"

if ! test -f /not_first_run; then
    echo "-> First container run, running MicroROS tools setup..."

    sudo chown -R urosdev: ~/pico_ws/
    cp "/pico/pico-sdk/external/pico_sdk_import.cmake" ~/pico_ws/pico_sdk_import.cmake
    cp "~/pico_ws/libfreertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake" ~/pico_ws/FreeRTOS_Kernel_import.cmake
    
    sudo apt-get update \
    && rosdep update \
    && cd ~/pico_ws/libmicroros \
    && sudo rosdep install --from-paths src --ignore-src -y \
    && sudo apt-get autoremove && sudo apt-get autoclean \
    && echo "-> Tools installed!"

    sudo touch /not_first_run
fi

cd ~/pico_ws/libmicroros/src/micro_ros_setup && colcon build
cd ~/pico_ws

source ~/pico_ws/libmicroros/src/micro_ros_setup/install/local_setup.bash
echo "source '~/pico_ws/libmicroros/src/micro_ros_setup/install/local_setup.bash'" >> ~/.bashrc

sleep infinity