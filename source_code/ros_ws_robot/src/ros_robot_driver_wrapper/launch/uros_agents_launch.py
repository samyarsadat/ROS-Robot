#  The ROS robot project (Robot Driver ROS Wrapper Package)
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

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pico_a_dev_arg = DeclareLaunchArgument("pico_a_dev", default_value="/dev/ttyACM0")
    pico_b_dev_arg = DeclareLaunchArgument("pico_b_dev", default_value="/dev/ttyACM1")
    pico_c_dev_arg = DeclareLaunchArgument("pico_c_dev", default_value="/dev/ttyACM2")

    launch_agent_a = launch_ros.actions.Node(package="micro_ros_agent", executable="micro_ros_agent", name="pico_a_uros_agent",
                                             arguments=["serial", "--dev", LaunchConfiguration("pico_a_dev")])
    launch_agent_b = launch_ros.actions.Node(package="micro_ros_agent", executable="micro_ros_agent", name="pico_b_uros_agent",
                                             arguments=["serial", "--dev", LaunchConfiguration("pico_b_dev")])
    launch_agent_c = launch_ros.actions.Node(package="micro_ros_agent", executable="micro_ros_agent", name="pico_c_uros_agent",
                                             arguments=["serial", "--dev", LaunchConfiguration("pico_c_dev")])

    return launch.LaunchDescription([
        pico_a_dev_arg, pico_b_dev_arg, pico_c_dev_arg,
        launch_agent_a, launch_agent_b
    ])