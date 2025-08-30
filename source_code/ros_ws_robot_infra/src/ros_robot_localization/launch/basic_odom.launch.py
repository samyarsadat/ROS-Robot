#  The ROS robot project (robot localization)
#  Basic odom -> base_link broadcaster.
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
package_name = "ros_robot_localization"


def generate_launch_description():
    odom_tf_config = PathJoinSubstitution([FindPackageShare(package_name), "config", "odom_tf_conf.yaml"])

    tf_broadcaster_node = Node(
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        name="odom_tf_node",
        parameters=[odom_tf_config]
    )

    return LaunchDescription([
        tf_broadcaster_node
    ])
