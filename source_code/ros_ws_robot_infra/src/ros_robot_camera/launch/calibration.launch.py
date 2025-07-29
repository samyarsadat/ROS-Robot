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


def generate_launch_description():
    camera_node_name_arg = DeclareLaunchArgument("camera_node_name", default_value="camera")

    camera_node = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        arguments=[
            '--size', '8x6',
            '--square', '0.025',
            '--size', '6x4',
            '--square', '0.041'
        ],
        remappings=[
            ("image", [LaunchConfiguration("camera_node_name"), "/image_raw"]),
            ("camera", [LaunchConfiguration("camera_node_name")]),
            ("camera/set_camera_info", [LaunchConfiguration("camera_node_name"), "/set_camera_info"]),
        ]
    )

    return launch.LaunchDescription([
        camera_node_name_arg,
        camera_node
    ])