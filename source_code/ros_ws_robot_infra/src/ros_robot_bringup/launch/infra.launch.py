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

import launch
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, PushROSNamespace
from launch_ros.substitutions import FindPackageShare
from launch_utils.log_styles import Ansi
from launch_utils.remap_utils import load_remappings
package_name = "ros_robot_bringup"


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    launch_camera_arg = DeclareLaunchArgument("launch_camera", default_value="True")
    launch_lidar_filter_arg = DeclareLaunchArgument("launch_lidar_filter", default_value="False")
    namespace_lc = LaunchConfiguration("namespace")

    rrp_camera_pkg = FindPackageShare("ros_robot_camera")
    rrp_description_pkg = FindPackageShare("ros_robot_description")
    rrp_driver_pkg = FindPackageShare("ros_robot_driver_wrapper")
    rrp_lidar_pkg = FindPackageShare("ros_robot_lidar")
    rrp_localization_pkg = FindPackageShare("ros_robot_localization")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rrp_camera_pkg, "launch", "camera.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("launch_camera")),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rrp_description_pkg, "launch", "robot_state.launch.py"])
        ),
        launch_arguments={
            "mock_joints": "False",
            "namespace": namespace_lc,
            "use_sim": "False"
        }.items()
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rrp_driver_pkg, "launch", "driver.launch.py"])
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rrp_lidar_pkg, "launch", "rplidar.launch.py"])
        )
    )

    lidar_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rrp_lidar_pkg, "launch", "rplidar_filter.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("launch_lidar_filter"))
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rrp_localization_pkg, "launch", "ekf.launch.py"])
        )
    )

    remappings = load_remappings(package_name, "global_remaps.yaml")
    parameters = [
        SetParameter(name="use_sim", value="False"),
        SetParameter(name="use_sim_time", value="False")
    ]

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            on_exit=lambda event, context: [
                LogInfo(msg=f"{Ansi.RED}A process exited with error(s).{Ansi.RESET}"),
                Shutdown(reason="A process exited with error(s).")
            ] if event.returncode != 0 else []
        )
    )

    status_ok = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=f"{Ansi.GREEN}ROS Robot infrastructure is running!{Ansi.RESET}"),
        ]
    )

    return launch.LaunchDescription(
        parameters + remappings + [
        namespace_arg,
        launch_camera_arg,
        launch_lidar_filter_arg,
        PushROSNamespace(namespace_lc),
        shutdown_on_exit,
        camera_launch,
        description_launch,
        driver_launch,
        lidar_launch,
        lidar_filter_launch,
        localization_launch,
        status_ok
    ])