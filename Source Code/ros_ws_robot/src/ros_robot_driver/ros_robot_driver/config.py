#  The ROS robot project (Robot Driver Package)
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

PROGRAM_VERSION = "2024.10.4"
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2024/10/04 @ 10:00 UTC"


# ---- ROS Config ----
class RosConfig:
    NODE_NAME = "robot_driver_pico_node"
    NODE_NAMESPACE = "io"
    EXECUTOR_DOMAIN_ID = 95
    EXECUTOR_TIMEOUT = 0.05         # 50ms
    EXECUTOR_SHUTDOWN_TIMEOUT = 5   # 5s
    SERVICE_CALL_TIMEOUT = 18       # 18s
    QOS_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    THREAD_NAME = "pico_ros_thread"


class RosNames:
    # Generic
    EMERGENCY_STOP_TOPIC = "/e_stop"
    DIAGNOSTICS_TOPIC = "/diagnostics"

    # Pico A
    COMMAND_VEL_TOPIC = "/cmd_vel"
    PICO_A_MISC_SENS_TOPIC = "sensors_raw/misc_a"
    ULTRASONIC_SENS_TOPIC = "sensors_raw/ultrasonics"
    CLIFF_SENS_TOPIC = "sensors_raw/falloff"
    MOTOR_CTRL_STATE_R_TOPIC = "sensors_raw/mtr_ctrl_right"
    MOTOR_CTRL_STATE_L_TOPIC = "sensors_raw/mtr_ctrl_left"
    ENC_ODOMETRY_TOPIC = "sensors/enc_odom"
    EN_MOTOR_CTRL_SRV = "enable_disable/motor_ctrl"
    EN_EMITTERS_SRV = "enable_disable/emitters"
    EN_RELAY_SRV = "enable_disable/pico_a_relay"
    SET_MTR_PID_TUNINGS_SRV = "config/set_motor_pid_tunings"
    PICO_A_RUN_SELFTEST_SRV = "self_test/pico_a"
    PICO_A_RUN_CALIB_SRV = "calibrate/pico_a"
    PICO_A_GET_CONFIG_SRV = "config/get_pico_a"

    # Pico B
    PICO_B_MISC_SENS_TOPIC = "sensors_raw/misc_b"
    MICRO_SW_SENS_TOPIC = "sensors_raw/microswitches"
    EN_CAMERA_LEDS_SRV = "enable_disable/camera_leds"
    PICO_B_RUN_SELFTEST_SRV = "self_test/pico_b"


# ---- Robot Config ----
class RobotConfig:
    # Sensor info
    MICRO_SW_TRIGGER_DISTANCE_CM = float(1.0)

    # Battery
    BATTERY_CAPACITY = 1.35   # In Ah, 1350mAh
    BATTERY_TECH = 3          # LiPo

    # Failure/success messages
    SERVICE_CALL_FAILURE = "Service call failed!"
    SERVICE_CALL_REDUNDANT = "Value already set, redundant call."

