#  The ROS robot project (camera_ros launch)
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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ros_robot_bringup.cam_launch_utils import get_camera_launch_arguments
from ament_index_python.resources import has_resource


def generate_launch_description():
    if not has_resource("packages", "camera_ros"):
        raise RuntimeError("The `camera_ros` package must be installed!")

    namespace_arg_name = "namespace"
    namespace_arg = DeclareLaunchArgument(namespace_arg_name, default_value="ros_robot")

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration(namespace_arg_name),
        parameters=[LaunchConfiguration("config_file"), {
            "camera": LaunchConfiguration("camera"),
            "camera_info_url": LaunchConfiguration("camera_info_url")
        }],
    )

    return launch.LaunchDescription(get_camera_launch_arguments() + [
        namespace_arg,
        camera_node
    ])