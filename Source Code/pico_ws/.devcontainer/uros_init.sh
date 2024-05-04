#!/usr/bin/env bash

set -e
echo "-> uros_init.sh"

if ! test -f /not_first_run; then
    echo "-> First container run, running MicroROS tools setup..."

    sudo chown -R urosdev: /home/urosdev/pico_ws/
    cp "/pico/pico-sdk/external/pico_sdk_import.cmake" /home/urosdev/pico_ws/pico_sdk_import.cmake
    cp "/home/urosdev/pico_ws/freertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake" /home/urosdev/pico_ws/FreeRTOS_Kernel_import.cmake
    
    sudo apt-get update \
    && rosdep update \
    && cd /home/urosdev/pico_ws/uros_ws \
    && sudo rosdep install --from-paths src --ignore-src -y \
    && sudo apt-get autoremove && sudo apt-get autoclean \
    && echo "-> Tools installed!"

    sudo touch /not_first_run
fi

cd /home/urosdev/pico_ws/uros_ws/src/micro_ros_setup && colcon build
cd /home/urosdev/pico_ws

source /home/urosdev/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash
echo "source '/home/urosdev/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash'" >> ~/.bashrc

sleep infinity