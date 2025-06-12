#  The ROS robot project (camera_ros launch utils)
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

from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
package_name = "ros_robot_bringup"


def get_camera_launch_arguments() -> list[DeclareLaunchArgument]:
    ros_robot_bringup_pkg = FindPackageShare(package_name)
    args = []

    camera_arg_name = "camera"
    args.append(DeclareLaunchArgument(camera_arg_name, default_value="0"))

    config_file_arg_name = "config_file"
    config_file_default = PathJoinSubstitution([ros_robot_bringup_pkg, "config", "camera_config.yaml"])
    args.append(DeclareLaunchArgument(config_file_arg_name, default_value=config_file_default))

    ci_url_arg_name = "camera_info_url"
    ci_url_default = PathJoinSubstitution([ros_robot_bringup_pkg, "config", "camera_calib.yaml"])
    args.append(DeclareLaunchArgument(ci_url_arg_name, default_value=ci_url_default))

    node_name_arg_name = "node_name"
    args.append(DeclareLaunchArgument(node_name_arg_name, default_value="camera_node"))

    return args


def get_composable_camera_node() -> ComposableNode:
    return ComposableNode(
        package="camera_ros",
        plugin="camera::CameraNode",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace", default="ros_robot"),
        parameters=[LaunchConfiguration("config_file"), {
            "camera": LaunchConfiguration("camera"),
            "camera_info_url": LaunchConfiguration("camera_info_url")
        }],
        extra_arguments=[{"use_intra_process_comms": True}]
    )