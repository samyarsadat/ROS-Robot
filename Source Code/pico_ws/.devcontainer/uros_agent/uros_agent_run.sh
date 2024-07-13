#!/usr/bin/env bash
# shellcheck disable=SC1091

# Temporary script for running the MicroROS agent.
# DO NOT RUN AS ROOT, RUN AS NON-ROOT USER!

cd ~/pico_ws/uros_agent \
&& source install/local_setup.bash \
&& ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v 4
