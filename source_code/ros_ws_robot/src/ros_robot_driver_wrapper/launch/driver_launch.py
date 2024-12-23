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

import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_launch_path(package_name: str, launch_file: str) -> str:
    package_share_directory = get_package_share_directory(package_name)
    return os.path.join(package_share_directory, "launch", launch_file)


def generate_launch_description():
    agents_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_path("ros_robot_driver_wrapper", "uros_agents_launch.py"))
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_path("ros_robot_driver_wrapper", "driver_node_launch.py"))
    )

    return launch.LaunchDescription([
        agents_launch, driver_launch
    ])