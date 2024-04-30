#!/usr/bin/env bash
# Temporary script for running the MicroROS agent.
# DO NOT RUN AS ROOT, RUN AS UROSDEV!

cd ~/pico_ws/uros_agent \
&& source install/local_setup.bash \
&& ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v 5