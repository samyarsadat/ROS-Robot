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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
package_name = "ros_robot_localization"


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    ekf_config = PathJoinSubstitution([FindPackageShare(package_name), "config", "ekf_conf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[ekf_config],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/tf_static", "tf_static"),
            ("/tf", "tf")
        ],
        namespace=LaunchConfiguration("namespace")
    )

    return LaunchDescription([
        namespace_arg,
        robot_localization_node
    ])