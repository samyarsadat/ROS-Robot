#!/bin/bash
set -e

sudo apt-get update && sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
     $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl status docker -n 0

sudo groupadd -f docker
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER

cd "$HOME" || exit 1
git clone https://github.com/samyarsadat/ROS-Robot ./ros_robot --recurse-submodules
cd ./ros_robot || exit 1
sudo chmod +s "./source_code/ros_ws_robot_infra/deployment/run_driver.bash"

newgrp docker