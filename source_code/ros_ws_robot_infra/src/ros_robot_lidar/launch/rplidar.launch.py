#  The ROS robot project (LiDAR package)
#  Physical RPLiDAR A1 data publisher launch file.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_utils.remap_utils import load_remappings
package_name = "ros_robot_lidar"


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0")

    rp_config_file = "rplidar_config.yaml"
    rmp_config_file = "rplidar_remap.yaml"

    node_name = "rplidar_node"
    laser_filter_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name=node_name,
        parameters=[
            {"serial_port": LaunchConfiguration("serial_port")},
            PathJoinSubstitution([FindPackageShare(package_name), "config", rp_config_file]),
        ],
        remappings=load_remappings(package_name, rmp_config_file, node_name),
        output="screen",
        namespace=LaunchConfiguration("namespace"),
    )

    return LaunchDescription([
        namespace_arg,
        serial_port_arg,
        laser_filter_node
    ])