#  The ROS robot project - CMake config
#  Copyright 2022 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2022.
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
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set project name and C/C++ standards
project(ROS_Robot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()


# ---- Pico A ----

# Add source files
add_executable(${PROJECT_NAME}_PICO_A Pico_A.cpp)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_PICO_A)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_PICO_A pico_stdlib)
target_link_libraries(${PROJECT_NAME}_PICO_A pico_multicore)

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_PICO_A 1)
pico_enable_stdio_uart(${PROJECT_NAME}_PICO_A 0)


# ---- Pico B ----

# Add source files
add_executable(${PROJECT_NAME}_PICO_B Pico_B.cpp)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_PICO_B)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_PICO_B pico_stdlib)
target_link_libraries(${PROJECT_NAME}_PICO_B pico_multicore)

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_PICO_B 1)
pico_enable_stdio_uart(${PROJECT_NAME}_PICO_B 0)