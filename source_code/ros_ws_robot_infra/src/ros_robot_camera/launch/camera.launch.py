#  The ROS robot project (camera_ros launch)
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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ros_robot_camera.launch_utils import get_camera_launch_arguments
from ament_index_python.resources import has_resource


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="ros_robot")

    if not has_resource("packages", "camera_ros"):
        raise RuntimeError("The `camera_ros` package must be installed!")

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("config_file"), {
            "camera": LaunchConfiguration("camera"),
            "camera_info_url": LaunchConfiguration("camera_info_url")
        }],
    )

    return launch.LaunchDescription(
        get_camera_launch_arguments() + [
        namespace_arg,
        camera_node
    ])