#  The ROS robot project (robot description)
#  RViz launch file.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
package_name="ros_robot_description"


def generate_launch_description():
    rviz_config_file = PathJoinSubstitution([FindPackageShare("rosbot_description"), "rviz", "ros_robot.rviz"])

    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False",  choices=["True", "False"])
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_config_file)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("namespace"),
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_arg,
        rviz_config_arg,
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim")),
        rviz_node
    ])
