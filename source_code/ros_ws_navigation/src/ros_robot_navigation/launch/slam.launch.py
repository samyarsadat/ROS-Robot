#  The ROS Robot Project (robot mapping & navigation)
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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString
package_name = "ros_robot_navigation"


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument("params_file")
    scan_topic_arg = DeclareLaunchArgument("scan_topic", default_value="scan")
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False")

    params_file_lc = LaunchConfiguration("params_file")
    scan_topic_lc = LaunchConfiguration("scan_topic")
    use_sim_lc = LaunchConfiguration("use_sim")

    slam_params_file = ReplaceString(
        source_file=PathJoinSubstitution([
            FindPackageShare(package_name), "config", "slam_config.yaml"
        ]),
        replacements={
            "<scan_topic>": scan_topic_lc
        }
    )

    slam_config_params = ParameterFile(slam_params_file, allow_substs=True)
    config_params = ParameterFile(params_file_lc, allow_substs=True)
    lifecycle_nodes = [
        "map_saver"
    ]

    bringup_cmd_group = GroupAction(
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                output="screen",
                parameters=[slam_config_params, config_params]
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                output="screen",
                parameters=[{
                    "node_names": lifecycle_nodes
                }]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"), "launch", "online_sync_launch.py"
                ])),
                launch_arguments={
                    "use_sim_time": use_sim_lc, 
                    "slam_params_file": slam_params_file
                }.items()
            )
        ]
    )

    return LaunchDescription([
        params_file_arg,
        scan_topic_arg,
        use_sim_arg,
        bringup_cmd_group
    ])
