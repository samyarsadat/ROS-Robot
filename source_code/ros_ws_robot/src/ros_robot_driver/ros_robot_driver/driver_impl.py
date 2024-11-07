#  The ROS robot project (Robot Driver Package)
#  Complete robot hardware interface model
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

import ros_robot_driver.ros_main
from typing import Union, Callable
from diagnostic_msgs.msg import DiagnosticStatus
from ros_robot_driver.interface_models.relay import Relay
from ros_robot_driver.interface_models.battery import BatteryInfo
from ros_robot_driver.interface_models.odometry import Odometry
from ros_robot_driver.interface_models.camera_leds import CameraLeds
from ros_robot_driver.interface_models.front_panel import FrontPanelSwitches
from ros_robot_driver.interface_models.imu_compass_sensor import ImuCompassSensor
from ros_robot_driver.interface_models.micro_sw_sensor import MicroSwSensor
from ros_robot_driver.interface_models.motor_controller import MotorController
from ros_robot_driver.interface_models.ultrasonic_sensor import UltrasonicSensor
from ros_robot_driver.interface_models.ir_cliff_sensor import CliffSensor
from ros_robot_driver.interface_models.temp_sensor import TempSensor
from ros_robot_driver.interface_models.processor_temp import ProcessorTempSensor
from ros_robot_driver.interface_data_structs.self_test_result import SelftestResult
from ros_robot_driver.interface_data_structs.diagnostics_report import DiagnosticsReport
from ros_robot_driver.interface_data_structs.motor_wheel_info import MotorWheelInfo
from ros_robot_driver.config import RobotConfig


class RosRobot:
    # Component interfaces
    relay: Relay
    battery: BatteryInfo
    encoder_odometry: Odometry
    camera_leds: CameraLeds
    fp_switched: FrontPanelSwitches
    imu_compass: ImuCompassSensor
    pico_a_temp: ProcessorTempSensor
    pico_b_temp: ProcessorTempSensor
    micro_switches: list[MicroSwSensor]   # [front-right, front-left, back-right, back-left]
    left_motor_controller: MotorController
    right_motor_controller: MotorController
    ultrasonic_sensors: list[UltrasonicSensor]   # [front, back, right, left]
    front_cliff_sensors: list[CliffSensor]
    back_cliff_sensors: list[CliffSensor]
    temperature_sensor: TempSensor
    motor_wheel_info: MotorWheelInfo

    _emitters_enabled: bool
    _diagnostics_recv_callback: Callable[[DiagnosticsReport], None]

    def __init__(self):
        self._emitters_enabled = True
        self._diagnostics_recv_callback = None

        self.relay = Relay("relay")
        self.battery = BatteryInfo("battery_monitoring", BatteryInfo.BatteryTechnology(RobotConfig.BATTERY_TECH), RobotConfig.BATTERY_CAPACITY)
        self.encoder_odometry = Odometry("encoder_odometry")
        self.camera_leds = CameraLeds("camera_leds")
        self.fp_switches = FrontPanelSwitches("front_panel_switches")
        self.imu_compass = ImuCompassSensor("imu_compass_sensors")
        self.temperature_sensor = TempSensor("env_dht11_sensor")
        self.motor_wheel_info = MotorWheelInfo()

        # Pico temperature
        self.pico_a_temp = ProcessorTempSensor("pico_a_temp")
        self.pico_b_temp = ProcessorTempSensor("pico_b_temp")

        # Micro switches
        self.micro_switches = []
        self.micro_switches.append(MicroSwSensor("front_right_micro_switch", RobotConfig.MICRO_SW_TRIGGER_DISTANCE_CM))
        self.micro_switches.append(MicroSwSensor("front_left_micro_switch", RobotConfig.MICRO_SW_TRIGGER_DISTANCE_CM))
        self.micro_switches.append(MicroSwSensor("back_right_micro_switch", RobotConfig.MICRO_SW_TRIGGER_DISTANCE_CM))
        self.micro_switches.append(MicroSwSensor("back_left_micro_switch", RobotConfig.MICRO_SW_TRIGGER_DISTANCE_CM))

        # Motor controllers
        self.left_motor_controller = MotorController("motor_controller_left")
        self.right_motor_controller = MotorController("motor_controller_right")

        # Ultrasonic sensors
        self.ultrasonic_sensors = []
        self.ultrasonic_sensors.append(UltrasonicSensor("ultrasonic_front", 0, 0.0, 0.0))
        self.ultrasonic_sensors.append(UltrasonicSensor("ultrasonic_back", 0, 0.0, 0.0))
        self.ultrasonic_sensors.append(UltrasonicSensor("ultrasonic_right", 0, 0.0, 0.0))
        self.ultrasonic_sensors.append(UltrasonicSensor("ultrasonic_left", 0, 0.0, 0.0))

        # Cliff sensor
        self.front_cliff_sensors = []
        self.back_cliff_sensors = []
        self.front_cliff_sensors.append(CliffSensor("ir_cliff_front_1_sensor", 0, 0.0))
        self.front_cliff_sensors.append(CliffSensor("ir_cliff_front_2_sensor", 0, 0.0))
        self.front_cliff_sensors.append(CliffSensor("ir_cliff_front_3_sensor", 0, 0.0))
        self.front_cliff_sensors.append(CliffSensor("ir_cliff_front_4_sensor", 0, 0.0))
        self.back_cliff_sensors.append(CliffSensor("ir_cliff_back_1_sensor", 0, 0.0))
        self.back_cliff_sensors.append(CliffSensor("ir_cliff_back_2_sensor", 0, 0.0))
        self.back_cliff_sensors.append(CliffSensor("ir_cliff_back_3_sensor", 0, 0.0))
        self.back_cliff_sensors.append(CliffSensor("ir_cliff_back_4_sensor", 0, 0.0))

    # Generic robot methods
    def set_command_vel(self, linear_vel: float, angular_vel: float):
        ros_robot_driver.ros_main.get_ros_node().publish_command_vel(linear_vel, angular_vel)

    def robot_emergency_stop(self):
        ros_robot_driver.ros_main.get_ros_node().publish_emergency_stop()

    def run_pico_a_selftest(self) -> Union[SelftestResult, None]:
        response = ros_robot_driver.ros_main.get_ros_node().run_pico_a_selftest()

        if response:
            result = SelftestResult()
            result.from_ros_result(response)
            return result
        return None

    def run_pico_a_calib(self, calib_ir_cliff: bool, calib_imu: bool, pid_autotune_left: bool, pid_autotune_right: bool) -> tuple[bool, str]:
        resp = ros_robot_driver.ros_main.get_ros_node().run_pico_a_calib(calib_ir_cliff, calib_imu, pid_autotune_left, pid_autotune_right)

        if resp:
            return resp.success, resp.message
        return False, RobotConfig.SERVICE_CALL_FAILURE

    def run_pico_b_selftest(self) -> Union[SelftestResult, None]:
        response = ros_robot_driver.ros_main.get_ros_node().run_pico_b_selftest()

        if response:
            result = SelftestResult()
            result.from_ros_result(response)
            return result
        return None

    def enable_emitters(self, enable: bool) -> tuple[bool, str]:
        if (not self._emitters_enabled and enable) or (self._emitters_enabled and not enable):
            resp = ros_robot_driver.ros_main.get_ros_node().en_emitters(enable)

            if resp:
                self._emitters_enabled = enable
                return resp.success, resp.message
            return False, RobotConfig.SERVICE_CALL_FAILURE
        return True, RobotConfig.SERVICE_CALL_REDUNDANT

    def get_emitters_enabled(self) -> bool:
        return self._emitters_enabled

    def set_diag_recv_callback(self, func: Callable[[DiagnosticsReport], None]):
        self._diagnostics_recv_callback = func

    @staticmethod
    def ping_driver() -> bool:
        return True

    def _call_diag_recv_safe_ros(self, status: DiagnosticStatus):
        if not self._diagnostics_recv_callback is None:
            report = DiagnosticsReport()
            report.from_ros_message(status)
            self._diagnostics_recv_callback(report)


# ---- Robot instance ----
ros_robot_interface = RosRobot()
