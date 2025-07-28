#!/bin/bash
ROS_DISTRO="jazzy"
set -e

sudo apt-get install software-properties-common
sudo add-apt-repository universe -y
sudo apt-get update && sudo apt-get install ca-certificates curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null

sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
     $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin vulcanexus-$ROS_DISTRO-base python3-pip -y
source /opt/vulcanexus/$ROS_DISTRO/setup.bash
sudo systemctl status docker -n 0

sudo groupadd -f docker
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER

sudo rosdep init
rosdep update

cd "$HOME" || exit 1
git clone https://github.com/samyarsadat/ROS-Robot ./ros_robot --recurse-submodules
SOURCE_CODE_PATH="$HOME/ros_robot/source_code"
sudo chmod +s "$SOURCE_CODE_PATH/ros_ws_robot_infra/deployment/run_infra.bash"

cd "$SOURCE_CODE_PATH/ros_camera_ws" || exit 1
ROS_DISTRO=$ROS_DISTRO bash ./build_setup_camera_ros.sh
ROS_DISTRO=$ROS_DISTRO bash ./build_camera_ros.sh

cd "$SOURCE_CODE_PATH/pico_ws/libmicroros" || exit 1
colcon build --packages-select rrp_pico_coms
source "./install/local_setup.sh"

cd "$SOURCE_CODE_PATH/ros_ws_robot_infra" || exit 1
colcon build --packages-select ros_robot_msgs
source "./install/local_setup.sh"

export PIP_BREAK_SYSTEM_PACKAGES=1
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-skip ros_robot_msgs

# ------ TEMPORARY ROS LAUNCH FIX ------
mkdir $HOME/roslaunch
cd $HOME/roslaunch

PR_NUMBER=893
git clone https://github.com/ros2/launch.git .
git fetch origin pull/$PR_NUMBER/head:pr-$PR_NUMBER
git checkout pr-$PR_NUMBER

rosdep install --from-paths . --ignore-src --skip-keys="camera_ros" -r -y
source /opt/ros/"$ROS_DISTRO"/setup.bash
colcon build
source "$HOME"/roslaunch/install/setup.bash

if [[ "$(ros2 pkg prefix launch)" != "$HOME/roslaunch/install/launch" ]]; then
    echo "ROS Launch fix installation failed!"
    exit 1
fi
# ------ TEMPORARY ROS LAUNCH FIX ------

sudo apt-get autoremove -y
sudo apt-get autoclean -y
newgrp docker