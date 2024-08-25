#  The ROS robot project (Robot Driver ROS Wrapper Package)
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

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.srv import SelfTest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_robot_driver_wrapper.config import RosConfig
from ros_robot_driver.driver_impl import ros_robot_interface
from sensor_msgs.msg import BatteryState, Imu, Temperature, RelativeHumidity, Range
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from ros_robot_msgs.srv import SetCameraLeds, GetCameraLeds, SetPidTunings, RunCalibrationsA, GetBool
from ros_robot_msgs.msg import FPSwitches, MotorCtrlState


# ---- ROS Node ----
class RosNode(Node):
    def __init__(self, context: rclpy.Context):
        super().__init__(context=context, node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE)
        self.get_logger().info("Creating publishers, subscribers, and services servers...")

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._selftest_calib_cb_group = MutuallyExclusiveCallbackGroup()
        self._emer_stop_cb_group = MutuallyExclusiveCallbackGroup()

        self.diagnostics_pub = self.create_publisher(DiagnosticStatus, "diagnostics", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.enable_relay_srv = self.create_service(SetBool, "enable/set_relay", self._enable_relay_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.battery_info_pub = self.create_publisher(BatteryState, "electrical/battery_state", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.encoder_odom_pub = self.create_publisher(Odometry, "pos_data/encoder_odom", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.set_camera_leds_srv = self.create_service(SetCameraLeds, "lights/camera/set", self._set_camera_leds_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.get_camera_leds_srv = self.create_service(GetCameraLeds, "lights/camera/get", self._get_camera_leds_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.front_panel_switches_pub = self.create_publisher(FPSwitches, "front_panel/switch_states", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.imu_sens_pub = self.create_publisher(Imu, "pos_data/imu", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.env_temp_sens_pub = self.create_publisher(Temperature, "env_sensors/temp", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.env_humidity_sens_pub = self.create_publisher(RelativeHumidity, "env_sensors/humidity", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._cmd_vel_sub_call, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.emergency_stop_sub = self.create_subscription(Empty, "emergency_stop", self._emergency_stop_sub_call, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._emer_stop_cb_group)
        self.pico_a_cpu_temp_pub = self.create_publisher(Temperature, "sys_temps/processor/pico_a", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.pico_b_cpu_temp_pub = self.create_publisher(Temperature, "sys_temps/processor/pico_b", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.pico_a_selftest_srv = self.create_service(SelfTest, "self_test/run_pico_a_routines", self._pico_a_selftest_srv_call, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self.pico_b_selftest_srv = self.create_service(SelfTest, "self_test/run_pico_b_routines", self._pico_b_selftest_srv_call, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self.pico_a_calibrate_srv = self.create_service(RunCalibrationsA, "calibrate/run_pico_a_routines", self._pico_a_calibrate_srv_call, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self.enable_emitters_srv = self.create_service(SetBool, "enable/set_emitters", self._enable_emitters_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.get_emitters_enabled_srv = self.create_service(GetBool, "enable/get_emitters", self._get_emitters_enabled_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.left_mtr_ctrl_pub = self.create_publisher(MotorCtrlState, "motors/controllers/left_state", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.right_mtr_ctrl_pub = self.create_publisher(MotorCtrlState, "motors/controllers/right_state", qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.mtr_ctrl_enable_srv = self.create_service(SetBool, "motors/controllers/enable", self._mtr_ctrl_enable_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.mtr_ctrl_set_pid_srv = self.create_service(SetPidTunings, "motors/controllers/set_pid", self._mtr_ctrl_set_pid_srv_call, qos_profile=RosConfig.QOS_RELIABLE)
        self.micro_switch_pubs = []
        self.ultrasonic_sens_pubs = []
        self.cliff_sens_pubs = []

        for i in range(0, 4):
            name = ros_robot_interface.micro_switches[i].get_id().split("_")[:2]
            self.micro_switch_pubs.append(self.create_publisher(Range, f"range_sens/micro_switch/{'_'.join(name)}", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

        for i in range(0, 4):
            name = ros_robot_interface.ultrasonic_sensors[i].get_id().split("_")[1]
            self.ultrasonic_sens_pubs.append(self.create_publisher(Range, f"range_sens/ultrasonic/{name}", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

        for i in range(0, 4):
            name = ros_robot_interface.front_cliff_sensors[i].get_id().split("_")[2:5]
            self.cliff_sens_pubs.append(self.create_publisher(Range, f"range_sens/cliff/{'_'.join(name)}", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

        for i in range(0, 4):
            name = ros_robot_interface.back_cliff_sensors[i].get_id().split("_")[2:5]
            self.cliff_sens_pubs.append(self.create_publisher(Range, f"range_sens/cliff/{'_'.join(name)}", qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

    @staticmethod
    def _enable_relay_srv_call(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data:
            res.success, res.message = ros_robot_interface.relay.enable_relay()
            return res
        res.success, res.message = ros_robot_interface.relay.disable_relay()
        return res

    @staticmethod
    def _set_camera_leds_srv_call(req: SetCameraLeds.Request, res: SetCameraLeds.Response) -> SetCameraLeds.Response:
        current_outputs = ros_robot_interface.camera_leds.get_set_pwm_vals()

        for i in range(0, 4):
            if req.set_output_mask[i]:
                current_outputs[i] = req.led_outputs[i]

        res.success = ros_robot_interface.camera_leds.set_all_outputs(current_outputs)
        return res

    @staticmethod
    def _get_camera_leds_srv_call(req: GetCameraLeds.Request, res: GetCameraLeds.Response) -> GetCameraLeds.Response:
        res.led_outputs = ros_robot_interface.camera_leds.get_set_pwm_vals()
        return res

    @staticmethod
    def _cmd_vel_sub_call(msg: Twist):
        ros_robot_interface.set_command_vel(msg.linear.x, msg.angular.z)

    @staticmethod
    def _emergency_stop_sub_call(msg: Empty):
        ros_robot_interface.robot_emergency_stop()

    @staticmethod
    def _pico_a_selftest_srv_call(req: SelfTest.Request, res: SelfTest.Response) -> SelfTest.Response:
        result = ros_robot_interface.run_pico_a_selftest()

        if result:
            res = result.to_ros_result()
        return res

    @staticmethod
    def _pico_b_selftest_srv_call(req: SelfTest.Request, res: SelfTest.Response) -> SelfTest.Response:
        result = ros_robot_interface.run_pico_b_selftest()

        if result:
            res = result.to_ros_result()
        return res

    @staticmethod
    def _pico_a_calibrate_srv_call(req: RunCalibrationsA.Request, res: RunCalibrationsA.Response) -> RunCalibrationsA.Response:
        res.success, res.message = ros_robot_interface.run_pico_a_calib(req.calib_ir_edge, req.calib_imu, req.calib_pid_left, req.calib_pid_right)
        return res

    @staticmethod
    def _enable_emitters_srv_call(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        res.success, res.message = ros_robot_interface.enable_emitters(req.data)
        return res

    @staticmethod
    def _get_emitters_enabled_srv_call(req: GetBool.Request, res: GetBool.Response) -> GetBool.Response:
        res.data = ros_robot_interface.get_emitters_enabled()
        return res

    @staticmethod
    def _mtr_ctrl_enable_srv_call(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        # IMPORTANT NOTE: Currently, the left and right controller cannot be enabled/disabled separately,
        # so only enabling/disabling one of them will affect both.

        if req.data:
            res.success, res.message = ros_robot_interface.right_motor_controller.enable_controller()
            return res
        res.success, res.message = ros_robot_interface.right_motor_controller.disable_controller()
        return res

    @staticmethod
    def _mtr_ctrl_set_pid_srv_call(req: SetPidTunings.Request, res: SetPidTunings.Response) -> SetPidTunings.Response:
        # IMPORTANT NOTE: Currently, the left and right controller's PID tunings cannot be set separately,
        # so only setting one of them will affect both.

        res.success = ros_robot_interface.right_motor_controller.set_pid_tunings(req.pid_kp, req.pid_ki, req.pid_kd)
        return res


# ---- Node object ----
robot_ros_node = RosNode


# ---- Execution ----
def is_ros_node_initialized() -> bool:
    global robot_ros_node
    return isinstance(robot_ros_node, RosNode)

def get_ros_node() -> RosNode:
    global robot_ros_node
    return robot_ros_node

def ros_executor_thread(stop_thread) -> None:
    try:
        internal_context = rclpy.Context()
        rclpy.init(context=internal_context, domain_id=RosConfig.EXECUTOR_DOMAIN_ID)

        global robot_ros_node
        robot_ros_node = RosNode(internal_context)
        robot_ros_node.get_logger().info("Driver wrapper node initialized!")

        executor = MultiThreadedExecutor(context=internal_context)
        executor.add_node(robot_ros_node)
        robot_ros_node.get_logger().info("Driver wrapper executor initialized!")

        # Initialize publication callbacks.
        from ros_robot_driver_wrapper.interface_callbacks import init_callbacks
        init_callbacks()

        robot_ros_node.get_logger().info("Starting the executor...")

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        robot_ros_node.get_logger().warn("Driver wrapper thread stopping. Shutting the executor down.")
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)
        rclpy.shutdown(context=internal_context)

    except ExternalShutdownException:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("External shutdown exception!")
