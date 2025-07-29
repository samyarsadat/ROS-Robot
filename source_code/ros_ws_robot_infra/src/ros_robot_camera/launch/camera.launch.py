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
from ros_robot_camera.launch_utils import get_camera_config_path


def generate_launch_description():
    camera_arg = DeclareLaunchArgument("camera", default_value="0")
    camera_config_file_arg = DeclareLaunchArgument("camera_config_file", default_value="camera_config.yaml")
    camera_info_url_arg = DeclareLaunchArgument("camera_calib_file", default_value="camera_calib.yaml")

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        parameters=[get_camera_config_path(LaunchConfiguration("camera_config_file")), {
            "camera_info_url": get_camera_config_path(LaunchConfiguration("camera_calib_file"), True),
            "camera": LaunchConfiguration("camera")
        }],
    )

    return launch.LaunchDescription([
        camera_arg,
        camera_config_file_arg,
        camera_info_url_arg,
        camera_node
    ])