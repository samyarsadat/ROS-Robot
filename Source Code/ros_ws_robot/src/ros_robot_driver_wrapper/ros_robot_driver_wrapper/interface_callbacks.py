#  The ROS robot project (Robot Driver ROS Wrapper Package)
#  Robot interface callback functions
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

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from ros_robot_driver.driver_impl import ros_robot_interface
from ros_robot_driver.interface_data_structs.diagnostics_report import DiagnosticsReport
from ros_robot_driver_wrapper.config import RosFrameIds
from ros_robot_driver_wrapper.ros_main import get_ros_node
from ros_robot_msgs.msg import FPSwitches, MotorCtrlState
from sensor_msgs.msg import BatteryState, Imu, Temperature, RelativeHumidity, Range
from ros_robot_driver_wrapper.utils import euler_to_quaternion


# ---- Callbacks ----

def diag_recv_callback(report: DiagnosticsReport) -> None:
    get_ros_node().diagnostics_pub.publish(report.to_ros_message())


def battery_info_callback() -> None:
    msg = BatteryState()
    msg.present = True
    msg.current = ros_robot_interface.battery.get_power_current_ma() / 1000
    msg.voltage = ros_robot_interface.battery.get_power_voltage()
    msg.design_capacity = ros_robot_interface.battery.get_battery_capacity_ah()
    msg.power_supply_technology = ros_robot_interface.battery.get_battery_technology().value
    msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
    msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.battery.get_last_timestamp()
    get_ros_node().battery_info_pub.publish(msg)


def encoder_odometry_callback() -> None:
    msg = Odometry()
    twist = Twist()
    pose = Pose()
    twist.linear.x, twist.linear.y, twist.linear.z = ros_robot_interface.encoder_odometry.get_linear_accel()
    twist.angular.x, twist.angular.y, twist.angular.z = ros_robot_interface.encoder_odometry.get_angular_accel()
    pose.position.x, pose.position.y, pose.position.z = ros_robot_interface.encoder_odometry.get_position()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ros_robot_interface.encoder_odometry.get_orientation()
    msg.twist.twist = twist
    msg.pose.pose = pose
    msg.child_frame_id = ros_robot_interface.encoder_odometry.get_child_frame_id()
    msg.header.frame_id = ros_robot_interface.encoder_odometry.get_header_frame_id()
    msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.encoder_odometry.get_last_timestamp()
    get_ros_node().encoder_odom_pub.publish(msg)


def fp_switches_callback() -> None:
    msg = FPSwitches()
    msg.switch_1 = ros_robot_interface.fp_switches.get_switch_1_state()
    msg.switch_2 = ros_robot_interface.fp_switches.get_switch_2_state()
    msg.time.sec, msg.time.nanosec = ros_robot_interface.fp_switches.get_last_timestamp()
    get_ros_node().front_panel_switches_pub.publish(msg)


def imu_sens_callback() -> None:
    msg = Imu()
    orient_euler_x, orient_euler_y, orient_euler_z = ros_robot_interface.imu_compass.get_compass()
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = euler_to_quaternion(orient_euler_x, orient_euler_y, orient_euler_z)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ros_robot_interface.imu_compass.get_accel()
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = ros_robot_interface.imu_compass.get_gyro()
    msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.imu_compass.get_last_timestamp()
    msg.header.frame_id = RosFrameIds.IMU_FRAME_ID
    get_ros_node().imu_sens_pub.publish(msg)


def left_mtr_ctrl_callback() -> None:
    msg = MotorCtrlState()
    msg.target_dir = ros_robot_interface.left_motor_controller.get_target_dir()
    msg.target_rpm = ros_robot_interface.left_motor_controller.get_target_rpm()
    msg.measured_dirs = ros_robot_interface.left_motor_controller.get_measured_dirs()
    msg.measured_rpms = ros_robot_interface.left_motor_controller.get_measured_rpms()
    msg.total_enc_counts = ros_robot_interface.left_motor_controller.get_enc_pulse_counts()
    msg.pid_output = ros_robot_interface.left_motor_controller.get_current_pid_out()
    msg.pid_tunings = ros_robot_interface.left_motor_controller.get_pid_tunings()
    msg.controller_enabled = ros_robot_interface.left_motor_controller.is_enabled()
    msg.total_current = ros_robot_interface.left_motor_controller.get_power_current_ma()
    msg.driver_out_voltage = ros_robot_interface.left_motor_controller.get_power_output_voltage()
    msg.time.sec, msg.time.nanosec = ros_robot_interface.left_motor_controller.get_last_timestamp()
    get_ros_node().left_mtr_ctrl_pub.publish(msg)


def right_mtr_ctrl_callback() -> None:
    msg = MotorCtrlState()
    msg.target_dir = ros_robot_interface.right_motor_controller.get_target_dir()
    msg.target_rpm = ros_robot_interface.right_motor_controller.get_target_rpm()
    msg.measured_dirs = ros_robot_interface.right_motor_controller.get_measured_dirs()
    msg.measured_rpms = ros_robot_interface.right_motor_controller.get_measured_rpms()
    msg.total_enc_counts = ros_robot_interface.right_motor_controller.get_enc_pulse_counts()
    msg.pid_output = ros_robot_interface.right_motor_controller.get_current_pid_out()
    msg.pid_tunings = ros_robot_interface.right_motor_controller.get_pid_tunings()
    msg.controller_enabled = ros_robot_interface.right_motor_controller.is_enabled()
    msg.total_current = ros_robot_interface.right_motor_controller.get_power_current_ma()
    msg.driver_out_voltage = ros_robot_interface.right_motor_controller.get_power_output_voltage()
    msg.time.sec, msg.time.nanosec = ros_robot_interface.right_motor_controller.get_last_timestamp()
    get_ros_node().right_mtr_ctrl_pub.publish(msg)


def temperature_sensor_callback() -> None:
    msg_temp = Temperature()
    msg_humidity = RelativeHumidity()
    msg_temp.temperature = ros_robot_interface.temperature_sensor.get_temp()
    msg_temp.header.frame_id = RosFrameIds.TEMP_SENS_FRAME_ID
    msg_temp.header.stamp.sec, msg_temp.header.stamp.nanosec = ros_robot_interface.temperature_sensor.get_last_timestamp()
    msg_humidity.relative_humidity = ros_robot_interface.temperature_sensor.get_humidity_percent()
    msg_humidity.header.frame_id = RosFrameIds.TEMP_SENS_FRAME_ID
    msg_humidity.header.stamp.sec, msg_humidity.header.stamp.nanosec = ros_robot_interface.temperature_sensor.get_last_timestamp()
    get_ros_node().env_temp_sens_pub.publish(msg_temp)
    get_ros_node().env_humidity_sens_pub.publish(msg_humidity)


def micro_switches_callback() -> None:
    for i in range(0, 4):
        msg = Range()
        msg.max_range = ros_robot_interface.micro_switches[i].get_trigger_distance()
        msg.min_range = ros_robot_interface.micro_switches[i].get_trigger_distance()
        msg.radiation_type = Range.INFRARED   # Not actually infrared, just a normal micro switches.
        msg.field_of_view = 90.0
        msg.range = (float("inf") if ros_robot_interface.micro_switches[i].get_latest_state() else float("-inf"))
        msg.header.frame_id = RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(get_ros_node().micro_switch_pubs[i].topic.split("/")[2])
        msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.micro_switches[i].get_last_timestamp()
        get_ros_node().micro_switch_pubs[i].publish(msg)


def ultrasonic_sens_callback() -> None:
    for i in range(0, 4):
        msg = Range()
        msg.max_range = ros_robot_interface.ultrasonic_sensors[i].get_max_distance()
        msg.min_range = ros_robot_interface.ultrasonic_sensors[i].get_min_distance()
        msg.field_of_view = float(ros_robot_interface.ultrasonic_sensors[i].get_field_of_view())
        msg.radiation_type = Range.ULTRASOUND
        msg.range = ros_robot_interface.ultrasonic_sensors[i].get_distance()
        msg.header.frame_id = RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(get_ros_node().ultrasonic_sens_pubs[i].topic.split("/")[2])
        msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.ultrasonic_sensors[i].get_last_timestamp()
        get_ros_node().ultrasonic_sens_pubs[i].publish(msg)


def cliff_sens_callback() -> None:
    for i in range(0, 4):
        msg = Range()
        msg.max_range = ros_robot_interface.front_cliff_sensors[i].get_trigger_distance()
        msg.min_range = ros_robot_interface.front_cliff_sensors[i].get_trigger_distance()
        msg.field_of_view = float(ros_robot_interface.front_cliff_sensors[i].get_field_of_view())
        msg.radiation_type = Range.INFRARED
        msg.range = (float("inf") if ros_robot_interface.front_cliff_sensors[i].get_latest_state() else float("-inf"))
        msg.header.frame_id = RosFrameIds.FRONT_CLIFF_SENS_BASE_FRAME_ID.format(i + 1)
        msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.front_cliff_sensors[i].get_last_timestamp()
        get_ros_node().cliff_sens_pubs[i].publish(msg)

    for i in range(0, 4):
        msg = Range()
        msg.max_range = ros_robot_interface.back_cliff_sensors[i].get_trigger_distance()
        msg.min_range = ros_robot_interface.back_cliff_sensors[i].get_trigger_distance()
        msg.field_of_view = float(ros_robot_interface.back_cliff_sensors[i].get_field_of_view())
        msg.radiation_type = Range.INFRARED
        msg.range = (float("inf") if ros_robot_interface.back_cliff_sensors[i].get_latest_state() else float("-inf"))
        msg.header.frame_id = RosFrameIds.BACK_CLIFF_SENS_BASE_FRAME_ID.format(i + 1)
        msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.back_cliff_sensors[i].get_last_timestamp()
        get_ros_node().cliff_sens_pubs[i + 4].publish(msg)


def pico_a_temp_callback() -> None:
    msg = Temperature()
    msg.temperature = ros_robot_interface.pico_a_temp.get_temp()
    msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.pico_b_temp.get_last_timestamp()
    get_ros_node().pico_a_cpu_temp_pub.publish(msg)


def pico_b_temp_callback() -> None:
    msg = Temperature()
    msg.temperature = ros_robot_interface.pico_b_temp.get_temp()
    msg.header.stamp.sec, msg.header.stamp.nanosec = ros_robot_interface.pico_b_temp.get_last_timestamp()
    get_ros_node().pico_b_cpu_temp_pub.publish(msg)


def init_callbacks() -> None:
    get_ros_node().get_logger().info("Initializing driver interface update callbacks...")

    ros_robot_interface.set_diag_recv_callback(diag_recv_callback)
    ros_robot_interface.battery.set_on_update_callback_func(battery_info_callback)
    ros_robot_interface.encoder_odometry.set_on_update_callback_func(encoder_odometry_callback)
    ros_robot_interface.fp_switches.set_on_update_callback_func(fp_switches_callback)
    ros_robot_interface.imu_compass.set_on_update_callback_func(imu_sens_callback)
    ros_robot_interface.left_motor_controller.set_on_update_callback_func(left_mtr_ctrl_callback)
    ros_robot_interface.right_motor_controller.set_on_update_callback_func(right_mtr_ctrl_callback)
    ros_robot_interface.temperature_sensor.set_on_update_callback_func(temperature_sensor_callback)
    ros_robot_interface.pico_a_temp.set_on_update_callback_func(pico_a_temp_callback)
    ros_robot_interface.pico_b_temp.set_on_update_callback_func(pico_b_temp_callback)

    # All four micro switches are updated at once, so no need to set callbacks for all.
    ros_robot_interface.micro_switches[0].set_on_update_callback_func(micro_switches_callback)

    # All four ultrasonics are updated at once, so no need to set callbacks for all.
    ros_robot_interface.ultrasonic_sensors[0].set_on_update_callback_func(ultrasonic_sens_callback)

    # All eight cliff sensors are updated at once, so no need to set callbacks for all.
    ros_robot_interface.front_cliff_sensors[0].set_on_update_callback_func(cliff_sens_callback)
