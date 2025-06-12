#  The ROS robot project (general robot infrastructure bring-up)
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

import os
import launch
from ament_index_python import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
package_name = "ros_robot_bringup"


def get_launch_path(pkg_name: str, launch_file: str) -> str:
    package_share_directory = get_package_share_directory(pkg_name)
    return os.path.join(package_share_directory, "launch", launch_file)


def get_agent_launch(context: LaunchContext) -> IncludeLaunchDescription:
    robot_name = LaunchConfiguration("robot_name").perform(context)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_path(package_name, "uros_agents_launch.py")),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "agent_name_prefix": f"{robot_name}_" if robot_name else "",
        }.items()
    )


def get_driver_launch(context: LaunchContext) -> IncludeLaunchDescription:
    robot_name = LaunchConfiguration("robot_name").perform(context)
    launch_args = {
        "namespace": LaunchConfiguration("namespace"),
    }

    if robot_name:
        launch_args["node_name"] = f"{robot_name}_driver"

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_path(package_name, "driver_node_launch.py")),
        launch_arguments=launch_args.items()
    )


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="ros_robot")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="")
    

    agents_launch = OpaqueFunction(function=lambda context: [get_agent_launch(context)])
    driver_launch = OpaqueFunction(function=lambda context: [get_driver_launch(context)])

    driver_launch_timer = TimerAction(
        period=3.5,
        actions=[
            LogInfo(msg="Starting driver node(s)..."),
            driver_launch
        ]
    )

    return launch.LaunchDescription([
        namespace_arg, robot_name_arg,
        agents_launch, driver_launch_timer
    ])