#  The ROS robot project (Robot Driver Package)
#  ROS setup, node, and executor
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

import array
import sys
import threading
import rclpy
from asyncio import Future
from typing import Union
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.client import SrvTypeRequest, Client, SrvTypeResponse
from ros_robot_driver.driver_impl import ros_robot_interface
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_robot_driver.config import RosConfig, RosNames
from rrp_pico_coms.msg import MiscSensorsA, UltrasonicSensors, FalloffSensors, MotorCtrlState, FastOdometry, MiscSensorsB, MicroSwSensors
from geometry_msgs.msg import Twist
from rrp_pico_coms.srv import SetPidTunings, SetCameraLeds, RunCalibrationsA, GetConfigA
from std_srvs.srv import SetBool
from std_msgs.msg import Empty
from diagnostic_msgs.srv import SelfTest
from diagnostic_msgs.msg import DiagnosticStatus
from copy import deepcopy


# ---- Global variables ----
_stop_ros_thread = False
_ros_thread = None


# ---- ROS Node ----
class RosNode(Node):
    # Client.call() implementation with timeout.
    def srv_call_with_timeout(self, client: Client, request: SrvTypeRequest, timeout_s: int) -> Union[SrvTypeResponse, None]:
        event = threading.Event()

        def unblock(ftr):
            event.set()

        future = client.call_async(request)
        future.add_done_callback(unblock)

        if not future.done():
            if not event.wait(float(timeout_s)):
                self.get_logger().error(f"Service call failure [{client.srv_name}]: timed out! Cancelling.")
                future.cancel()
                return None

        if future.exception() is not None:
            raise future.exception()

        return future.result()

    def __init__(self, context: rclpy.Context):
        super().__init__(context=context, node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE)
        self.get_logger().info("Creating publishers, subscribers, and services servers...")

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._selftest_calib_cb_group = MutuallyExclusiveCallbackGroup()
        self._emer_stop_cb_group = MutuallyExclusiveCallbackGroup()

        self._emergency_stop_pub = self.create_publisher(Empty, RosNames.EMERGENCY_STOP_TOPIC, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._emer_stop_cb_group)
        self._command_vel_pub = self.create_publisher(Twist, RosNames.COMMAND_VEL_TOPIC, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)

        self._diagnostics_sub = self.create_subscription(DiagnosticStatus, RosNames.DIAGNOSTICS_TOPIC, self.diagnostics_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._pico_a_misc_sens_sub = self.create_subscription(MiscSensorsA, RosNames.PICO_A_MISC_SENS_TOPIC, self.pico_a_misc_sens_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._ultrasonic_sens_sub = self.create_subscription(UltrasonicSensors, RosNames.ULTRASONIC_SENS_TOPIC, self.ultrasonic_sens_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._cliff_sens_sub = self.create_subscription(FalloffSensors, RosNames.CLIFF_SENS_TOPIC, self.cliff_sens_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._motor_ctrl_state_r_sub = self.create_subscription(MotorCtrlState, RosNames.MOTOR_CTRL_STATE_R_TOPIC, self.motor_ctrl_state_r_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._motor_ctrl_state_l_sub = self.create_subscription(MotorCtrlState, RosNames.MOTOR_CTRL_STATE_L_TOPIC, self.motor_ctrl_state_l_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._encoder_odom_sub = self.create_subscription(FastOdometry, RosNames.ENC_ODOMETRY_TOPIC, self.encoder_odom_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._pico_b_misc_sens_sub = self.create_subscription(MiscSensorsB, RosNames.PICO_B_MISC_SENS_TOPIC, self.pico_b_misc_sens_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._micro_sw_sens_sub = self.create_subscription(MicroSwSensors, RosNames.MICRO_SW_SENS_TOPIC, self.micro_sw_sens_call, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)

        self._en_motor_ctrl_srvcl = self.create_client(SetBool, RosNames.EN_MOTOR_CTRL_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self._en_emitters_srvcl = self.create_client(SetBool, RosNames.EN_EMITTERS_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self._en_relay_srvcl = self.create_client(SetBool, RosNames.EN_RELAY_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self._set_mtr_pid_tunings_srvcl = self.create_client(SetPidTunings, RosNames.SET_MTR_PID_TUNINGS_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self._pico_a_run_selftest_srvcl = self.create_client(SelfTest, RosNames.PICO_A_RUN_SELFTEST_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self._pico_a_run_calib_srvcl = self.create_client(RunCalibrationsA, RosNames.PICO_A_RUN_CALIB_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self._pico_a_get_config_srvcl = self.create_client(GetConfigA, RosNames.PICO_A_GET_CONFIG_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self._en_camera_leds_srvcl = self.create_client(SetCameraLeds, RosNames.EN_CAMERA_LEDS_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self._pico_b_run_selftest_srvcl = self.create_client(SelfTest, RosNames.PICO_B_RUN_SELFTEST_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)

        self._pico_a_get_config_future = None

        self._en_motor_ctrl_req = SetBool.Request()
        self._en_emitters_req = SetBool.Request()
        self._en_relay_req = SetBool.Request()
        self._set_mtr_pid_tunings_req = SetPidTunings.Request()
        self._pico_a_run_selftest_req = SelfTest.Request()
        self._pico_a_run_calib_req = RunCalibrationsA.Request()
        self._pico_a_get_config_req = GetConfigA.Request()
        self._en_camera_leds_req = SetCameraLeds.Request()
        self._pico_b_run_selftest_req = SelfTest.Request()

    def publish_command_vel(self, linear_vel: float, angular_vel: float) -> None:
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self._command_vel_pub.publish(msg)

    def publish_emergency_stop(self) -> None:
        self.get_logger().warn("Emergency stop received!")
        self._emergency_stop_pub.publish(Empty())

    def en_motor_ctrl(self, enable: bool) -> Union[SetBool.Response, None]:
        if self._en_motor_ctrl_srvcl.service_is_ready():
            self._en_motor_ctrl_req.data = enable
            return self.srv_call_with_timeout(self._en_motor_ctrl_srvcl, self._en_motor_ctrl_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._en_motor_ctrl_srvcl.srv_name}]: service not ready!")
        return None

    def en_emitters(self, enable: bool) -> Union[SetBool.Response, None]:
        if self._en_emitters_srvcl.service_is_ready():
            self._en_emitters_req.data = enable
            return self.srv_call_with_timeout(self._en_emitters_srvcl, self._en_emitters_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._en_emitters_srvcl.srv_name}]: service not ready!")
        return None

    def en_relay(self, enable: bool) -> Union[SetBool.Response, None]:
        if self._en_relay_srvcl.service_is_ready():
            self._en_relay_req.data = enable
            return self.srv_call_with_timeout(self._en_relay_srvcl, self._en_relay_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._en_relay_srvcl.srv_name}]: service not ready!")
        return None

    def set_mtr_pid_tunings(self, kp: float, ki: float, kd: float) -> Union[SetPidTunings.Response, None]:
        if self._set_mtr_pid_tunings_srvcl.service_is_ready():
            self._set_mtr_pid_tunings_req.pid_kp = kp
            self._set_mtr_pid_tunings_req.pid_ki = ki
            self._set_mtr_pid_tunings_req.pid_kd = kd
            return self.srv_call_with_timeout(self._set_mtr_pid_tunings_srvcl, self._set_mtr_pid_tunings_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._set_mtr_pid_tunings_srvcl.srv_name}]: service not ready!")
        return None

    def run_pico_a_selftest(self) -> Union[SelfTest.Response, None]:
        if self._pico_a_run_selftest_srvcl.service_is_ready():
            return self.srv_call_with_timeout(self._pico_a_run_selftest_srvcl, self._pico_a_run_selftest_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._pico_a_run_selftest_srvcl.srv_name}]: service not ready!")
        return None

    def run_pico_a_calib(self, calib_ir_cliff: bool, calib_imu: bool, autotune_pid_left: bool, autotune_pid_right: bool) -> Union[RunCalibrationsA.Response, None]:
        if self._pico_a_run_calib_srvcl.service_is_ready():
            self._pico_a_run_calib_req.calib_ir_edge = calib_ir_cliff
            self._pico_a_run_calib_req.calib_imu = calib_imu
            self._pico_a_run_calib_req.calib_pid_left = autotune_pid_left
            self._pico_a_run_calib_req.calib_pid_right = autotune_pid_right
            return self.srv_call_with_timeout(self._pico_a_run_calib_srvcl, self._pico_a_run_calib_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._pico_a_run_calib_srvcl.srv_name}]: service not ready!")
        return None

    def set_camera_leds(self, led_outputs: array.array) -> Union[SetCameraLeds.Response, None]:
        if self._en_camera_leds_srvcl.service_is_ready():
            self._en_camera_leds_req.led_outputs = led_outputs
            return self.srv_call_with_timeout(self._en_camera_leds_srvcl, self._en_camera_leds_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._en_camera_leds_srvcl.srv_name}]: service not ready!")
        return None

    def run_pico_b_selftest(self) -> Union[SelfTest.Response, None]:
        if self._pico_b_run_selftest_srvcl.service_is_ready():
            return self.srv_call_with_timeout(self._pico_b_run_selftest_srvcl, self._pico_b_run_selftest_req, RosConfig.SERVICE_CALL_TIMEOUT)
        self.get_logger().error(f"Service call failure [{self._pico_b_run_selftest_srvcl.srv_name}]: service not ready!")
        return None

    def init_config_vars(self) -> bool:
        if self._pico_a_get_config_srvcl.service_is_ready():
            self._pico_a_get_config_future = self._pico_a_get_config_srvcl.call_async(self._pico_a_get_config_req)
            self._pico_a_get_config_future.add_done_callback(self._init_config_vars_future_call)
            return True
        self.get_logger().error(f"Service call failure [{self._pico_a_get_config_srvcl.srv_name}]: service not ready!")
        return False

    def _init_config_vars_future_call(self, future: Future):
        response = future.result()

        if response:
            ros_robot_interface.motor_wheel_info._encoder_ppr = response.encoder_pulses_per_rotation
            ros_robot_interface.motor_wheel_info._wheelbase_mm = response.wheelbase_mm
            ros_robot_interface.motor_wheel_info._wheel_diameter_mm = response.wheel_diameter_mm
            ros_robot_interface.motor_wheel_info._motor_gear_ratio = response.motor_gear_ratio_fl
            ros_robot_interface.right_motor_controller._pid_tunings = array.array("f", response.pid_cals_right)
            ros_robot_interface.left_motor_controller._pid_tunings = array.array("f", response.pid_cals_left)

            for i in range(0, 4):
                ros_robot_interface.ultrasonic_sensors[i]._field_of_view = response.ultrasonic_fov
                ros_robot_interface.ultrasonic_sensors[i]._min_distance = float(response.ultrasonic_min_dist)
                ros_robot_interface.ultrasonic_sensors[i]._max_distance = float(response.ultrasonic_max_dist)
                ros_robot_interface.front_cliff_sensors[i]._field_of_view = response.ir_edge_sens_fov
                ros_robot_interface.front_cliff_sensors[i]._trigger_distance = float(response.ir_edge_sens_range)
                ros_robot_interface.back_cliff_sensors[i]._field_of_view = response.ir_edge_sens_fov
                ros_robot_interface.back_cliff_sensors[i]._trigger_distance = float(response.ir_edge_sens_range)

            return
        self.get_logger().error("Init config vars future result empty!")

    @staticmethod
    def diagnostics_call(msg: DiagnosticStatus) -> None:
        ros_robot_interface._call_diag_recv_safe_ros(msg)

    @staticmethod
    def pico_a_misc_sens_call(msg: MiscSensorsA) -> None:
        timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])
        ros_robot_interface.pico_a_temp._temperature = msg.cpu_temp
        ros_robot_interface.pico_a_temp._last_timestamp = timestamp
        ros_robot_interface.imu_compass._acceleration = array.array("f", [msg.imu_accel.x, msg.imu_accel.y, msg.imu_accel.z])
        ros_robot_interface.imu_compass._gyroscope = array.array("f", [msg.imu_gyro.x, msg.imu_gyro.y, msg.imu_gyro.z])
        ros_robot_interface.imu_compass._compass = array.array("f", [msg.imu_compass.x, msg.imu_compass.y, msg.imu_compass.z])
        ros_robot_interface.imu_compass._temperature = msg.imu_temp
        ros_robot_interface.imu_compass._is_free_fall_triggered = msg.imu_freefall_int
        ros_robot_interface.imu_compass._last_timestamp = timestamp
        ros_robot_interface.imu_compass._call_update_callback_safe()
        ros_robot_interface.pico_a_temp._call_update_callback_safe()

    @staticmethod
    def ultrasonic_sens_call(msg: UltrasonicSensors) -> None:
        timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])
        ros_robot_interface.ultrasonic_sensors[0]._reading = msg.ultrasonic_f_reading
        ros_robot_interface.ultrasonic_sensors[1]._reading = msg.ultrasonic_b_reading
        ros_robot_interface.ultrasonic_sensors[2]._reading = msg.ultrasonic_r_reading
        ros_robot_interface.ultrasonic_sensors[3]._reading = msg.ultrasonic_l_reading

        for i in range(0, 4):
            ros_robot_interface.ultrasonic_sensors[i]._last_timestamp = timestamp
            ros_robot_interface.ultrasonic_sensors[i]._call_update_callback_safe()

    @staticmethod
    def cliff_sens_call(msg: FalloffSensors) -> None:
        timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])

        for i in range(0, 4):
            ros_robot_interface.front_cliff_sensors[i]._is_triggered = msg.ir_edge_sens_front_trig[i]
            ros_robot_interface.front_cliff_sensors[i]._last_timestamp = timestamp
            ros_robot_interface.back_cliff_sensors[i]._is_triggered = msg.ir_edge_sens_back_trig[i]
            ros_robot_interface.back_cliff_sensors[i]._last_timestamp = timestamp
            ros_robot_interface.front_cliff_sensors[i]._call_update_callback_safe()
            ros_robot_interface.back_cliff_sensors[i]._call_update_callback_safe()

    @staticmethod
    def motor_ctrl_state_r_call(msg: MotorCtrlState) -> None:
        ros_robot_interface.right_motor_controller._controller_enabled = msg.controller_enabled
        ros_robot_interface.right_motor_controller._current_pid_out = msg.pid_output
        ros_robot_interface.right_motor_controller._enc_pulse_counts = array.array("l", msg.total_enc_counts)
        ros_robot_interface.right_motor_controller._measured_rpms = array.array("f", msg.measured_rpms)
        ros_robot_interface.right_motor_controller._measured_dirs = list(msg.measured_dirs)
        ros_robot_interface.right_motor_controller._target_rpm = msg.target_rpm
        ros_robot_interface.right_motor_controller._target_dir = msg.target_dir
        ros_robot_interface.right_motor_controller._power_output_voltage = msg.driver_out_voltage
        ros_robot_interface.right_motor_controller._power_current_ma = msg.total_current
        ros_robot_interface.right_motor_controller._last_timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])
        ros_robot_interface.right_motor_controller._call_update_callback_safe()

    @staticmethod
    def motor_ctrl_state_l_call(msg: MotorCtrlState) -> None:
        ros_robot_interface.left_motor_controller._controller_enabled = msg.controller_enabled
        ros_robot_interface.left_motor_controller._current_pid_out = msg.pid_output
        ros_robot_interface.left_motor_controller._enc_pulse_counts = array.array("l", msg.total_enc_counts)
        ros_robot_interface.left_motor_controller._measured_rpms = array.array("f", msg.measured_rpms)
        ros_robot_interface.left_motor_controller._measured_dirs = deepcopy(list(msg.measured_dirs))
        ros_robot_interface.left_motor_controller._target_rpm = msg.target_rpm
        ros_robot_interface.left_motor_controller._target_dir = msg.target_dir
        ros_robot_interface.left_motor_controller._power_output_voltage = msg.driver_out_voltage
        ros_robot_interface.left_motor_controller._power_current_ma = msg.total_current
        ros_robot_interface.left_motor_controller._last_timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])
        ros_robot_interface.left_motor_controller._call_update_callback_safe()

    @staticmethod
    def encoder_odom_call(msg: FastOdometry) -> None:
        ros_robot_interface.encoder_odometry._position = array.array("f", [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        ros_robot_interface.encoder_odometry._linear_accel = array.array("f", [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        ros_robot_interface.encoder_odometry._angular_accel = array.array("f", [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        ros_robot_interface.encoder_odometry._orientation = array.array("f", [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        ros_robot_interface.encoder_odometry._header_frame_id = msg.header.frame_id
        ros_robot_interface.encoder_odometry._child_frame_id = msg.child_frame_id
        ros_robot_interface.encoder_odometry._last_timestamp = array.array("L", [msg.header.stamp.sec, msg.header.stamp.nanosec])
        ros_robot_interface.encoder_odometry._call_update_callback_safe()

    @staticmethod
    def pico_b_misc_sens_call(msg: MiscSensorsB) -> None:
        timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])
        ros_robot_interface.pico_b_temp._temperature = msg.cpu_temp
        ros_robot_interface.pico_b_temp._last_timestamp = timestamp
        ros_robot_interface.fp_switches._switch_1_state = msg.control_mode_switch
        ros_robot_interface.fp_switches._switch_2_state = msg.speed_sel_switch
        ros_robot_interface.fp_switches._last_timestamp = timestamp
        ros_robot_interface.temperature_sensor._temperature = msg.env_temp
        ros_robot_interface.temperature_sensor._humidity_percent = msg.env_humidity
        ros_robot_interface.temperature_sensor._last_timestamp = timestamp
        ros_robot_interface.camera_leds._set_pwm_vals = array.array("i", msg.camera_led_vals)
        ros_robot_interface.camera_leds._last_timestamp = timestamp
        ros_robot_interface.battery._power_current_milliamps = msg.battery_current
        ros_robot_interface.battery._power_voltage = msg.battery_voltage
        ros_robot_interface.battery._last_timestamp = timestamp
        ros_robot_interface.battery._call_update_callback_safe()
        ros_robot_interface.camera_leds._call_update_callback_safe()
        ros_robot_interface.temperature_sensor._call_update_callback_safe()
        ros_robot_interface.fp_switches._call_update_callback_safe()
        ros_robot_interface.pico_b_temp._call_update_callback_safe()

    @staticmethod
    def micro_sw_sens_call(msg: MicroSwSensors) -> None:
        timestamp = array.array("L", [msg.time.sec, msg.time.nanosec])
        ros_robot_interface.micro_switches[0]._is_triggered = msg.micro_fr_trig
        ros_robot_interface.micro_switches[1]._is_triggered = msg.micro_fl_trig
        ros_robot_interface.micro_switches[2]._is_triggered = msg.micro_br_trig
        ros_robot_interface.micro_switches[3]._is_triggered = msg.micro_bl_trig

        for i in range(0, 4):
            ros_robot_interface.micro_switches[i]._last_timestamp = timestamp
            ros_robot_interface.micro_switches[i]._call_update_callback_safe()


# ---- Node object ----
pico_ros_node: RosNode


# ---- Execution ----
def is_ros_node_initialized() -> bool:
    global pico_ros_node
    return isinstance(pico_ros_node, RosNode)

def get_ros_node() -> RosNode:
    global pico_ros_node
    return pico_ros_node

def _ros_executor_thread(stop_thread) -> None:
    try:
        internal_context = rclpy.Context()
        rclpy.init(context=internal_context, domain_id=RosConfig.EXECUTOR_DOMAIN_ID)

        global pico_ros_node
        pico_ros_node = RosNode(internal_context)
        pico_ros_node.get_logger().info("Driver node initialized!")

        executor = MultiThreadedExecutor(context=internal_context)
        executor.add_node(pico_ros_node)
        pico_ros_node.get_logger().info("Driver executor initialized!")

        pico_ros_node.get_logger().info("Initializing static config info variables...")
        if not pico_ros_node.init_config_vars():
            pico_ros_node.get_logger().fatal("Static config info variable init failed! Exiting...")
            sys.exit(1)

        pico_ros_node.get_logger().info("Starting the executor...")

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        pico_ros_node.get_logger().warn("Driver thread stopping. Shutting the executor down.")
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)
        rclpy.shutdown(context=internal_context)

    except ExternalShutdownException:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("External shutdown exception!")


# ROS thread control.
class RosRobotDriverThread:
    @staticmethod
    def start_thread() -> None:
        global _stop_ros_thread
        global _ros_thread

        if _ros_thread is None:
            _ros_thread = threading.Thread(target=_ros_executor_thread, args=(lambda: _stop_ros_thread, ), name=RosConfig.THREAD_NAME)
            _ros_thread.start()

    @staticmethod
    def stop_thread(await_thread_termination: bool) -> None:
        global _stop_ros_thread
        global _ros_thread

        if not _ros_thread is None:
            if is_ros_node_initialized():
                get_ros_node().get_logger().warn("Driver thread stop request received!")

            _stop_ros_thread = True
            if await_thread_termination: _ros_thread.join()

    @staticmethod
    def is_alive() -> bool:
        global _ros_thread

        if _ros_thread:
            return _ros_thread.is_alive()
        return False
