#!/bin/bash
#  The ROS Robot Project - MicroROS build script for CodeQL analysis.
#  This is a modified version of the scripts from the MicroROS Build Action.
#  This workaround is necessary because CodeQL needs run in the same environment as the build script.
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

set -e
export ROS_DISTRO="humble"

# Get arguments
MICROROS_DIR_PATH=$1
MICROROS_SRC_PATH="src"
MICROROS_SETUP_PATH="micro_ros_setup"
COLCON_META_NAME="my_colcon.meta"
TOOLCHAIN_CMAKE_NAME="my_toolchain.cmake"
EXTRA_BUILD_PACKAGES="rrp_pico_coms"

# Split package names into array
IFS=" " read -r -a EXTRA_BUILD_PACKAGES <<< "$EXTRA_BUILD_PACKAGES"

# Make paths absolute
RELATIVE_MICROROS_DIR_PATH="$MICROROS_DIR_PATH"
MICROROS_DIR_PATH="$GITHUB_WORKSPACE/$MICROROS_DIR_PATH"
MICROROS_SRC_PATH="$MICROROS_DIR_PATH/$MICROROS_SRC_PATH"
MICROROS_SETUP_PATH="$MICROROS_SRC_PATH/$MICROROS_SETUP_PATH"

# Directory existance check function
check_directory_exists() {
    local dir_path=$1
    
    if [ ! -d "$dir_path" ]; then
        echo "ERROR: $dir_path does not exist."
        exit 1
    fi
}

# File existance check function
check_file_exists() {
    local file_path=$1
    
    if [ ! -f "$file_path" ]; then
        echo "ERROR: $file_path does not exist."
        exit 1
    fi
}

# Check if directories exist
check_directory_exists "$MICROROS_DIR_PATH"
check_directory_exists "$MICROROS_SRC_PATH"

for package in "${EXTRA_BUILD_PACKAGES[@]}"; do
    check_directory_exists "$MICROROS_SRC_PATH/$package"
done

# Check if the toolchain, colcon meta, and environment setup files exist
check_file_exists "$MICROROS_DIR_PATH/$TOOLCHAIN_CMAKE_NAME"
check_file_exists "$MICROROS_DIR_PATH/$COLCON_META_NAME"

# Environment setup
apt-get update \
&& apt-get install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib build-essential \
&& apt-get install -y git python3 rsync

mkdir /pico && cd /pico \
&& git clone https://github.com/raspberrypi/pico-sdk.git --branch "1.5.1" \
&& cd pico-sdk \
&& git submodule update --init

export PICO_SDK_PATH="/pico/pico-sdk"
export PICO_COMPILER="pico_arm_gcc"
cd "$GITHUB_WORKSPACE"

# Echo arguments
echo "Configuration:"
echo "MICROROS_DIR_PATH=$MICROROS_DIR_PATH"
echo "MICROROS_SRC_PATH=$MICROROS_SRC_PATH"
echo "MICROROS_SETUP_PATH=$MICROROS_SETUP_PATH"
echo "COLCON_META_NAME=$COLCON_META_NAME"
echo "TOOLCHAIN_CMAKE_NAME=$TOOLCHAIN_CMAKE_NAME"
echo "EXTRA_BUILD_PACKAGES=${EXTRA_BUILD_PACKAGES[@]}"
echo "ROS_DISTRO=$ROS_DISTRO"

# Install MicroROS tools
echo "Running tools setup script..."

# Install MicroROS tools
echo "Installing MicroROS tools..."
rosdep update
cd "$MICROROS_DIR_PATH"
sudo rosdep install --rosdistro $ROS_DISTRO --from-paths "$MICROROS_SRC_PATH" --ignore-src -y

#rm -rf "$MICROROS_SETUP_PATH"
#git clone --recurse-submodules --branch $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git "$MICROROS_SETUP_PATH"

echo "Building MicroROS setup..."
source /opt/ros/$ROS_DISTRO/setup.bash
cd "$MICROROS_SETUP_PATH" && colcon build

# Build MicroROS and extra packages
cd "$MICROROS_DIR_PATH"
echo "Removing directories..."
rm -rf ./build && rm -rf ./firmware && rm -rf ./install && rm -rf ./log

for package in "${EXTRA_BUILD_PACKAGES[@]}"; do
    echo "Building extra package: $package..."
    colcon build --packages-select "$package"
done

source install/local_setup.bash
source "$MICROROS_SETUP_PATH/install/local_setup.bash"

ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

for package in "${EXTRA_BUILD_PACKAGES[@]}"; do
    echo "Running workspace creation script for package: $package..."
    ros2 run "$package" create_firmware_ws.sh
done

ros2 run micro_ros_setup build_firmware.sh "$MICROROS_DIR_PATH/$TOOLCHAIN_CMAKE_NAME" "$MICROROS_DIR_PATH/$COLCON_META_NAME"
echo "Build completed successfully (probably)!"

# Add output directory path to GITHUB_OUTPUT
echo "library_build_dir=$RELATIVE_MICROROS_DIR_PATH/firmware/build/" >> $GITHUB_OUTPUT
