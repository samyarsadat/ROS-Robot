#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091,SC2162

# A short script for building the MicroROS library.
# Written by Samyar Sadat Akhavi, 2024.
# NOTE: DO NOT RUN AS ROOT, RUN AS "urosdev"!

set -e
echo "MicroROS Library Build Script."
echo "NOTE: DO NOT RUN AS ROOT, RUN AS 'urosdev'!"
echo ""

read -p "This will delete the previous build and start from scratch. Continue? (y/n) " confirm

case $confirm in 
    [yY]) 
        cd ~/pico_ws/uros_ws \
        && echo "Removing directories..." && rm -rf ./build && rm -rf ./firmware && rm -rf ./install && rm -rf ./log \
        && echo "Building..." && colcon build --packages-select rrp_pico_coms \
        && source install/local_setup.bash \
        && sudo apt update \
        && ros2 run micro_ros_setup create_firmware_ws.sh generate_lib \
        && ros2 run rrp_pico_coms create_fwws.sh \
        && ros2 run micro_ros_setup build_firmware.sh "$(pwd)/my_toolchain.cmake" "$(pwd)/my_colcon.meta"
        echo "Build completed successfully (probably)!"
        ;;
    [nN]) 
        echo "Cancelled."
        ;;
    *)
        echo "Invalid input!"
        echo "Cancelled."
        ;;
esac