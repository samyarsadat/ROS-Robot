#  The ROS robot project (Robot Driver ROS Wrapper Package)
#  Full robot driver launch description
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
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
package_name = "ros_robot_driver_wrapper"


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    namespace_lc = LaunchConfiguration("namespace")

    agents_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_name, "launch", "uros_agents.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace_lc
        }.items()
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_name, "launch", "driver_node.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace_lc
        }.items()
    )

    driver_launch_timer = TimerAction(
        period=3.5,
        actions=[
            LogInfo(msg="Starting driver node(s)..."),
            driver_launch
        ]
    )

    return launch.LaunchDescription([
        namespace_arg,
        agents_launch,
        driver_launch_timer
    ])