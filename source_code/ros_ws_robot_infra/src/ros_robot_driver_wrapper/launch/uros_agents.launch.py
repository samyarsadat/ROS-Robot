#  The ROS robot project (Robot Driver ROS Wrapper Package)
#  MicroROS agent(s) launch description
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

import launch
from ament_index_python import has_resource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_agent_node_description(name: str) -> Node:
    return Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name=f"{name}_uros_agent",
        arguments=["serial", "--dev", LaunchConfiguration(f"{name}_dev")]
    )


def generate_launch_description():
    if not has_resource("packages", "micro_ros_agent"):
        raise RuntimeError("The `micro_ros_agent` package must be installed!")

    pico_dev_args = [
        DeclareLaunchArgument("pico_a_dev", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("pico_b_dev", default_value="/dev/ttyACM1")
    ]

    return launch.LaunchDescription([
        generate_agent_node_description("pico_a"),
        generate_agent_node_description("pico_b")
    ] + pico_dev_args)