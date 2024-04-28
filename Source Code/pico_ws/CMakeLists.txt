#  The ROS robot project - Top Level CMake config
#  Copyright 2022-2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2022-2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.


# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set project name and Pico A/B name suffixes.
set(PROJ_NAME ROS_Robot)
set(PICO_A_NAME ${PROJ_NAME}_Pico_A)
set(PICO_B_NAME ${PROJ_NAME}_Pico_B)

# Include build functions from the Pico SDK
include(pico_sdk_import.cmake)

# Set project name and C/C++ standards
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Include MicroROS
set(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/uros_ws/firmware/build" CACHE STRING "MicroROS Library Path")
include(micro_ros.cmake)

# Add our source directory
add_subdirectory(src)

message("Configuration completed! (${PROJ_NAME} - v${PROJECT_VERSION})")