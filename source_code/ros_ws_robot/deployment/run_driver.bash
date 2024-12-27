#!/bin/bash
set -e
cd "$HOME/ros_robot/source_code/ros_ws_robot/.prodcontainer" || exit 1
docker compose up -d