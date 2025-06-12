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
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_agent_node_description(context: LaunchContext, name: str) -> Node:
    return Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name=f"{LaunchConfiguration("agent_name_prefix").perform(context)}{name}_uros_agent",
        namespace=LaunchConfiguration("namespace"),
        arguments=["serial", "--dev", LaunchConfiguration(f"{name}_dev")]
    )


def generate_launch_description():
    if not has_resource("packages", "micro_ros_agent"):
        raise RuntimeError("The `micro_ros_agent` package must be installed!")

    namespace_arg = DeclareLaunchArgument("namespace", default_value="ros_robot")
    agent_name_prefix_arg = DeclareLaunchArgument("agent_name_prefix", default_value="")

    pico_dev_args = [
        DeclareLaunchArgument("pico_a_dev", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("pico_b_dev", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("pico_c_dev", default_value="/dev/ttyACM2")
    ]

    agent_node_ofs = [
        OpaqueFunction(function=lambda context: [generate_agent_node_description(context, "pico_a")]),
        OpaqueFunction(function=lambda context: [generate_agent_node_description(context, "pico_b")])
    ]

    return launch.LaunchDescription([
        namespace_arg, agent_name_prefix_arg
    ] + pico_dev_args + agent_node_ofs)