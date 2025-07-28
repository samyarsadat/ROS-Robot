#!/bin/bash
ROS_DISTRO="jazzy"
set -e

while getopts rb flag
do
    case "${flag}" in
        b) FORCE_REBUILD="true";;
        r) FORCE_RESET="true";;
        *) echo "Invalid flags! (-r: reset repository, -b: run colcon build regardless of up-to-dateness)" && exit 1;;
    esac
done

SOURCE_CODE_PATH="$HOME/ros_robot/source_code"
cd "$HOME/ros_robot" || exit 1
git fetch origin
GIT_BRANCH=$(git branch --show-current)
IS_UPTODATE=$(git diff "origin/$GIT_BRANCH")

if [ "$IS_UPTODATE" != "" ] || [ "$FORCE_RESET" == "true" ]; then
    git clean -dfx
    git reset --recurse-submodules --hard
    git pull origin "$GIT_BRANCH"
    git submodule update --recursive
fi

if [ "$IS_UPTODATE" != "" ] || [ "$FORCE_REBUILD" == "true" ]; then
    source /opt/vulcanexus/$ROS_DISTRO/setup.bash
    
    cd "$SOURCE_CODE_PATH/pico_ws/libmicroros" || exit 1
    colcon build --packages-select rrp_pico_coms
    
    cd "$SOURCE_CODE_PATH/ros_ws_robot_infra" || exit 1
    colcon build
fi

echo "All up to date with $GIT_BRANCH."