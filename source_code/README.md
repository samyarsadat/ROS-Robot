## Introduction
This directory contains the source code and related files (such as Dockerfiles for deployment / devconainers for development) for Raspberry Pi Pico firmware and all custom ROS packages that I have written for this project.<br><br>

### Operating Systems & Software Versions
**Raspberry Pi OS:** Ubuntu 23.10 (until 24.04 is released), ROS2 running in an Ubuntu 22.04.4 Docker Container.<br>
**Workstation OS:** Ubuntu 22.04.4, ROS2 running in an Ubuntu 22.04.4 Docker Container.<br>
**ROS2 Distribution:** ROS2 Humble Hawksbill.<br><br>

### Directories
 - **pico_ws:** Raspberry Pi Pico workspace. Contains the source code for Raspberry Pi Pico firmware, the source code for the ROS package that contains the custom message definitions that are used by the Picos to communicate with the main Raspberry Pi computer, and a devcontainer for building the firmware and the ROS package. It also contains pre-built binaries for the Picos.
 - **ros_ws_robot:** Robot ROS workspace. Contains the source code for all of the ROS packages that run on the robot, a devcontainer for developing and building the packages, and a Dockerfile for deploying the packages to the robot.
 - **ros_ws_workstation:** Workstation ROS workspace. Contains the source code for all of the ROS packages that run on the workstation (ROS base station), a devcontainer for developing and building the packages, and a Dockerfile for running the packages on the workstation.<br><br>

***W.I.P.***