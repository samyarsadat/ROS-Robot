#  The ROS robot project - Library CMake config
#  Helper functions for using MicroROS with FreeRTOS.  
#
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
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

# Add source files
add_library(${PROJECT_NAME} FreeRTOS_Helpers.c++
                            RTOS_Agent.c++
                            utils/pico_uart_transport.c
                            utils/uros_allocators.c
                            uROS_Bridge_Agent.c++
                            uROS_Publishing_Handler.c++)

# Include header files
target_include_directories(${PROJECT_NAME} PUBLIC ../include)

# Link to libraries
target_link_libraries(${PROJECT_NAME} pico_stdlib
                                      pico_stdio_usb
                                      micro_ros
                                      FreeRTOS-Kernel
                                      FreeRTOS-Kernel-Heap4
                                      local_helpers_lib)