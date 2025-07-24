#  The ROS robot project (robot description)
#  Robot state publisher launch file.
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
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
package_name="ros_robot_description"


def generate_launch_description():
    urdf_file = "ros_robot.urdf.xacro"

    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False", choices=["True", "False"])
    mock_joints_arg = DeclareLaunchArgument("mock_joints", default_value="True", choices=["True", "False"])

    use_sim_lc = LaunchConfiguration("use_sim")
    namespace_lc = LaunchConfiguration("namespace")

    robot_desc_config = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(package_name), "urdf", urdf_file]),
        " namespace:=", namespace_lc,
        " use_sim:=", use_sim_lc
    ])

    robot_description = {"robot_description": robot_desc_config}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        namespace=namespace_lc,
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(
            LaunchConfiguration("mock_joints", default="True")
        ),
        namespace=namespace_lc
    )

    return LaunchDescription([
        use_sim_arg,
        namespace_arg,
        mock_joints_arg,
        SetParameter(name="use_sim_time", value=use_sim_lc),
        robot_state_pub_node,
        joint_state_publisher_node,
    ])
