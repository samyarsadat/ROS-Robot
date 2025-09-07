#  The ROS robot project (command velocity multiplexer launch)
#  MicroROS agent(s) launch description
#  Copyright 2025 Samyar Sadat Akhavi.
#  Written by Samyar Sadat Akhavi, 2025.
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
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_utils.remap_utils import load_remappings_tuple
package_name = "ros_robot_bringup"


def generate_launch_description():
    twist_mux_config = "twist_mux_conf.yaml"
    twist_mux_remaps = "twist_mux_remaps.yaml"

    twist_mux_node_name = "twist_mux"
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name=twist_mux_node_name,
        parameters=[
            PathJoinSubstitution([FindPackageShare(package_name), "config", twist_mux_config])
        ],
        remappings=load_remappings_tuple(package_name, twist_mux_remaps, twist_mux_node_name),
        output="screen"
    )

    return LaunchDescription([
        twist_mux_node
    ])