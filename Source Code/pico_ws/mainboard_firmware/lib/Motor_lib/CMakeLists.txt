#  The ROS robot project (Motor Control Library) - Library CMake config
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

# Include build functions from the Pico SDK
include(../../../pico_sdk_import.cmake)

# Set project name and C/C++ standards
project(motor_control_lib VERSION 1.0.0 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()


# Add source files
add_library(${PROJECT_NAME} Motor.c++ Motor_Driver.c++ Motor_Encoder.c++ Motor_Safety.c++)

# Link to libraries
target_link_libraries(${PROJECT_NAME} pico_stdlib)
target_link_libraries(${PROJECT_NAME} hardware_pwm)
target_link_libraries(${PROJECT_NAME} hardware_gpio)
target_link_libraries(${PROJECT_NAME} helpers_lib)
target_link_libraries(${PROJECT_NAME} pid_v1_lib)


# CMake flags
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
add_compile_options(-Wall -Wextra -Wpedantic)

message("Configured Motor Control Library build. (${PROJECT_NAME} - v${CMAKE_PROJECT_VERSION})")