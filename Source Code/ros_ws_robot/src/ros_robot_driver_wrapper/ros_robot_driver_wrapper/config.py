#  The ROS robot project (Robot Driver ROS Wrapper Package)
#  Empty package init file
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

PROGRAM_VERSION = "2024.8.25"
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy
import sys


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2024/08/25 @ 09:05PM UTC"


class ProgramConfig:
    THREADS_LIVELINESS_CHECK_INTERVAL_S = 2


# ---- ROS Config ----
class RosConfig:
    NODE_NAME = "robot_driver_wrapper_node"
    NODE_NAMESPACE = sys.argv[1] if len(sys.argv) > 1 else "ros_robot"
    EXECUTOR_DOMAIN_ID = None
    EXECUTOR_TIMEOUT = 0.05         # 50ms
    EXECUTOR_SHUTDOWN_TIMEOUT = 5   # 5s
    QOS_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    THREAD_NAME = "robot_ros_thread"


class RosFrameIds:
    IMU_FRAME_ID = "imu_sens"
    TEMP_SENS_FRAME_ID = "temp_sens"
    FRONT_CLIFF_SENS_BASE_FRAME_ID = "cliff_sens_front_{}"
    BACK_CLIFF_SENS_BASE_FRAME_ID = "cliff_sens_back_{}"
    ULTRASONIC_SENS_BASE_FRAME_ID = "ultrasonic_sens_{}"
    MICRO_SW_SENS_BASE_FRAME_ID = "micro_switch_{}"
