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
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_utils.log_styles import Ansi
package_name = "ros_robot_driver_wrapper"


def generate_launch_description():
    this_package_share = FindPackageShare(package_name)

    agents_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([this_package_share, "launch", "uros_agents.launch.py"])
        )
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([this_package_share, "launch", "driver_node.launch.py"])
        )
    )

    driver_launch_timer = TimerAction(
        period=3.5,
        actions=[
            LogInfo(msg=f"{Ansi.BLUE}Starting driver node(s)...{Ansi.RESET}"),
            driver_launch
        ]
    )

    return launch.LaunchDescription([
        agents_launch,
        driver_launch_timer
    ])