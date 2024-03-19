#!/usr/bin/env bash

set -e
echo "-> uros_init.sh"

cd /home/urosdev/pico_ws/uros_ws/src/micro_ros_setup && colcon build
cd /home/urosdev/pico_ws

source /home/urosdev/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash
echo "source '/home/urosdev/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash'" >> ~/.bashrc

sleep infinity