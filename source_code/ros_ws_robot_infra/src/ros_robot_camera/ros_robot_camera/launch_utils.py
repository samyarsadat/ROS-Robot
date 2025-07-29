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

from launch import Substitution
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
package_name = "ros_robot_camera"


def get_camera_config_path(filename, url_form: bool=False) -> list[Substitution]:
    return ([TextSubstitution(text="file://")] if url_form else []) + [
        PathJoinSubstitution([FindPackageShare(package_name), "config", filename])
    ]

def get_composable_camera_node(config_file, calib_file, name="camera_node", camera="0") -> ComposableNode:
    return ComposableNode(
        package="camera_ros",
        plugin="camera::CameraNode",
        name=name,
        parameters=[get_camera_config_path(config_file), {
            "camera": camera,
            "camera_info_url": get_camera_config_path(calib_file, True),
        }],
        extra_arguments=[{
            "use_intra_process_comms": True
        }]
    )