#  The ROS robot project (Robot Driver ROS Wrapper Package)
#  Robot driver node launch description
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
from launch.actions import RegisterEventHandler, LogInfo, DeclareLaunchArgument, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
package_name = "ros_robot_driver_wrapper"


def generate_launch_description():
    pico_domain_id_arg = DeclareLaunchArgument("pico_domain_id", default_value="95")
    robot_namespace_arg = DeclareLaunchArgument("namespace", default_value="ros_robot")
    node_name_arg = DeclareLaunchArgument("node_name", default_value="ros_robot_driver")

    launch_driver = Node(
        package=package_name,
        executable="ros_robot_driver",
        namespace=LaunchConfiguration("namespace"),
        name=LaunchConfiguration("node_name"),
        parameters=[{
            "pico_domain_id": LaunchConfiguration("pico_domain_id")
        }],
        remappings = [
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )

    return launch.LaunchDescription([
        pico_domain_id_arg, robot_namespace_arg, node_name_arg, launch_driver,
        RegisterEventHandler(
            OnProcessExit(
                target_action=launch_driver,
                on_exit=[
                    LogInfo(msg="Driver exited, shutting down."),
                    EmitEvent(event=Shutdown(reason="Driver node exited."))
                ]
            )
        ),
    ])