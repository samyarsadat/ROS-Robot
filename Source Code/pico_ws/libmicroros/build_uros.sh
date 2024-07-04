#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091,SC2162

# A short script for building the MicroROS library.
# Written by Samyar Sadat Akhavi, 2024.
# NOTE: DO NOT RUN AS ROOT, RUN AS NON-ROOT USER!

set -e
echo "MicroROS Library Build Script."
echo "NOTE: DO NOT RUN AS ROOT, RUN AS 'nonroot'!"
echo ""

while getopts bfr flag
do
    case "${flag}" in
        b) FULL_REBUILD="false";;
        f) FULL_REBUILD="true";;
        r) REINSTALL_PACKAGES="true";;
        *) echo "Invalid flags! (-b: firmware build only, -f: full re-build)" && exit 1;;
    esac
done

if [ "$FULL_REBUILD" == "true" ]; then
    read -p "This will delete the previous build and start from scratch. Continue? (y/n) " confirm

    case $confirm in 
        [yY]) 
            cd ~/pico_ws/libmicroros \
            && echo "Removing directories..." && rm -rf ./build && rm -rf ./firmware && rm -rf ./install && rm -rf ./log \
            && echo "Building..." && colcon build --packages-select rrp_pico_coms \
            && source install/local_setup.bash \
            && sudo apt update \
            && ros2 run micro_ros_setup create_firmware_ws.sh generate_lib \
            && ros2 run rrp_pico_coms create_firmware_ws.sh \
            && ros2 run micro_ros_setup build_firmware.sh "$(pwd)/my_toolchain.cmake" "$(pwd)/my_colcon.meta"
            echo "Build completed successfully (probably)!"
            ;;
        [nN]) 
            echo "Cancelled."
            ;;
        *)
            echo "Invalid input!"
            echo "Cancelled."
            exit 1
            ;;
    esac
fi

if [ "$FULL_REBUILD" == "false" ]; then
    echo "Re-building firmware only..."
    cd ~/pico_ws/libmicroros \
    && ros2 run micro_ros_setup build_firmware.sh "$(pwd)/my_toolchain.cmake" "$(pwd)/my_colcon.meta"
    echo "Build completed successfully (probably)!"
fi

if [ "$REINSTALL_PACKAGES" == "true" ]; then
    echo "Re-building and re-installing packages..."
    echo "Removing directories..." && rm -rf ./build && rm -rf ./install && rm -rf ./log
    cd ~/pico_ws/libmicroros/src && colcon build
    source ~/pico_ws/libmicroros/src/install/local_setup.bash
    source ~/.bashrc
    echo "Packages re-installed successfully!"
fi