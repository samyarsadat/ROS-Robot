#!/usr/bin/env bash

set -e
echo "-> uros_init.sh"

if ! test -f /not_first_run; then
    echo "-> First container run, running MicroROS tools setup..."

    sudo chown urosdev: /home/urosdev/pico_ws/
    cp "/pico/pico-sdk/external/pico_sdk_import.cmake" /home/urosdev/pico_ws/pico_sdk_import.cmake
    
    mkdir -p /home/urosdev/pico_ws/uros_ws/src && cd /home/urosdev/pico_ws/uros_ws/src \
    && rm -rf micro_ros_setup \
    && mkdir micro_ros_setup \
    && cp -a /home/urosdev/uros_tools_temp/. ./micro_ros_setup/ \
    && rm -rf /home/urosdev/uros_tools_temp \
    && echo "-> File prep complete."
    
    sudo apt-get update \
    && rosdep update \
    && cd /home/urosdev/pico_ws/uros_ws \
    && sudo rosdep install --from-paths src --ignore-src -y \
    && sudo rm -rf /var/lib/apt/lists/* \
    && sudo apt-get autoremove && sudo apt-get autoclean \
    && echo "-> Tools installed!"

    sudo touch /not_first_run
fi

cd /home/urosdev/pico_ws/uros_ws/src/micro_ros_setup && colcon build
cd /home/urosdev/pico_ws

source /home/urosdev/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash
echo "source '/home/urosdev/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash'" >> ~/.bashrc

sleep infinity