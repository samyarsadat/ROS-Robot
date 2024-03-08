#!/bin/bash
# shellcheck disable=SC1090,SC1091

set -e

source /opt/ros/"$ROS_DISTRO"/setup.bash --
echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> ~/.bashrc

exec "$@"