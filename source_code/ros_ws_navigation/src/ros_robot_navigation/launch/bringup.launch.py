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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_utils.remap_utils import load_remappings
package_name = "ros_robot_navigation"


def generate_launch_description():
    this_package_share = FindPackageShare(package_name)
    pkg_launch_dir = PathJoinSubstitution([this_package_share, "launch"])

    scan_filtered_arg = DeclareLaunchArgument("scan_filtered", default_value="False")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="True")
    map_arg = DeclareLaunchArgument("map", default_value="/maps/map.yaml")
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    slam_arg = DeclareLaunchArgument("slam", default_value="False")
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False")

    scan_filtered_lc = LaunchConfiguration("scan_filtered")
    autostart_lc = LaunchConfiguration("autostart")
    map_lc = LaunchConfiguration("map")
    namespace_lc = LaunchConfiguration("namespace")
    slam_lc = LaunchConfiguration("slam")
    use_sim_lc = LaunchConfiguration("use_sim")

    param_substitutions = {
        "yaml_filename": map_lc
    }

    scan_topic = PythonExpression([
        "'lidar/scan_filtered' if ", scan_filtered_lc, " else 'lidar/scan'"
    ])

    params_file = ReplaceString(
        source_file=PathJoinSubstitution([
            this_package_share, "config", "nav2_config.yaml"
        ]),
        replacements={
            "<scan_topic>": scan_topic,
            "<namespace>": namespace_lc
        }
    )

    params_file = RewrittenYaml(
        source_file=params_file,
        root_key=namespace_lc,
        param_rewrites=param_substitutions,
        convert_types=True,
    ),

    config_params = ParameterFile(params_file, allow_substs=True)
    container_name = "nav2_container"
    container_name_full = (namespace_lc, "/", container_name)

    bringup_cmd_group = GroupAction(
        load_remappings(package_name, "nav_remappings.yaml") + 
        [
        SetParameter("use_sim_time", use_sim_lc),
        SetParameter("autostart", autostart_lc),
        PushRosNamespace(namespace_lc),
        Node(
            name=container_name,
            package="rclcpp_components",
            executable="component_container_isolated",
            parameters=[config_params, {"autostart": autostart_lc}],
            output="screen"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_launch_dir, "slam.launch.py"])),
            condition=IfCondition(slam_lc),
            launch_arguments={
                "params_file": params_file,
                "scan_topic": scan_topic,
                "use_sim": use_sim_lc
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_launch_dir, "localization.launch.py"])),
            condition=UnlessCondition(slam_lc),
            launch_arguments={
                "container_name": container_name_full,
                "params_file": params_file,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_launch_dir, "navigation.launch.py"])),
            launch_arguments={
                "params_file": params_file,
                "container_name": container_name_full,
            }.items()
        )
    ])

    return LaunchDescription([
        scan_filtered_arg,
        autostart_arg,
        map_arg,
        namespace_arg,
        slam_arg,
        use_sim_arg,
        bringup_cmd_group
    ])
