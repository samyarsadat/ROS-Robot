#  The ROS robot project (Robot Driver Package)
#  Motor controller interface
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

from ros_robot_driver.interface_models.base_impl import BaseInterface
from ros_robot_driver.config import RobotConfig
import array


class MotorController(BaseInterface):
    _measured_rpms: array.array
    _measured_dirs: list[bool]   # true: forwards, false: backwards
    _enc_pulse_counts: array.array
    _target_rpm: float
    _target_dir: bool   # true: forwards, false: backwards
    _current_pid_out: int
    _pid_tunings: array.array   # [Kp, Ki, Kd]
    _power_current_ma: int
    _power_output_voltage: float
    _controller_enabled: bool

    def __init__(self, system_id: str):
        super().__init__(system_id)
        self._measured_rpms = array.array("I", [0, 0])
        self._measured_dirs = [False, False]
        self._enc_pulse_counts = array.array("i", [0, 0])
        self._target_rpm = 0.0
        self._target_dir = False
        self._current_pid_out = 0
        self._pid_tunings = array.array("f", [0, 0, 0])
        self._power_current_ma = 0
        self._power_output_voltage = 0.0
        self._controller_enabled = False

    # Setters
    def set_pid_tunings(self, kp: float, ki: float, kd: float) -> bool:
        from ros_robot_driver.ros_main import get_ros_node
        resp = get_ros_node().set_mtr_pid_tunings(kp, ki, kd)

        if resp:
            if resp.success:
                get_ros_node().init_config_vars()
            return resp.success
        return False

    def disable_controller(self) -> tuple[bool, str]:
        from ros_robot_driver.ros_main import get_ros_node

        if self._controller_enabled:
            resp = get_ros_node().en_motor_ctrl(False)

            if resp:
                if resp.success:
                    self._controller_enabled = False
                return resp.success, resp.message
            return False, RobotConfig.SERVICE_CALL_FAILURE
        return True, RobotConfig.SERVICE_CALL_REDUNDANT

    def enable_controller(self) -> tuple[bool, str]:
        from ros_robot_driver.ros_main import get_ros_node

        if not self._controller_enabled:
            resp = get_ros_node().en_motor_ctrl(True)

            if resp:
                if resp.success:
                    self._controller_enabled = True
                return resp.success, resp.message
            return False, RobotConfig.SERVICE_CALL_FAILURE
        return True, RobotConfig.SERVICE_CALL_REDUNDANT

    # Getters
    def get_measured_rpms(self) -> array.array:
        return self._measured_rpms

    def get_measured_dirs(self) -> list[bool]:
        return self._measured_dirs

    def get_enc_pulse_counts(self) -> array.array:
        return self._enc_pulse_counts

    def get_target_rpm(self) -> float:
        return self._target_rpm

    def get_target_dir(self) -> bool:
        return self._target_dir

    def get_current_pid_out(self) -> int:
        return self._current_pid_out

    def get_pid_tunings(self) -> array.array:
        return self._pid_tunings

    def get_power_current_ma(self) -> int:
        return self._power_current_ma

    def get_power_output_voltage(self) -> float:
        return self._power_output_voltage

    def is_enabled(self) -> bool:
        return self._controller_enabled
