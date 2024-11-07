#!/usr/bin/env bash
# Temporary script for building the MicroROS agent.
# DO NOT RUN AS ROOT, RUN AS NON-ROOT USER!

mkdir ~/pico_ws/uros_agent
cd ~/pico_ws/uros_agent \
&& ros2 run micro_ros_setup create_agent_ws.sh \
&& ros2 run micro_ros_setup build_agent.sh
