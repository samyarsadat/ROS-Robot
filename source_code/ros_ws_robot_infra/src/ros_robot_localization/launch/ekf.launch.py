#  The ROS robot project (robot localization)
#  Localization Extended Kalman Filter launch file.
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
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_utils.remap_utils import load_remappings_tuple
package_name = "ros_robot_localization"


def generate_launch_description():
    ekf_config = PathJoinSubstitution([FindPackageShare(package_name), "config", "ekf_conf.yaml"])

    node_name = "ekf_node"
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name=node_name,
        parameters=[ekf_config],
        remappings=load_remappings_tuple(package_name, "ekf_remaps.yaml", node_name=node_name)
    )

    return LaunchDescription([
        robot_localization_node
    ])