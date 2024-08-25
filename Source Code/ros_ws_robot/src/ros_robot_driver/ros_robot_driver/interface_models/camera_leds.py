#  The ROS robot project (Robot Driver Package)
#  Camera LED interface
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
from copy import deepcopy
import array


class CameraLeds(BaseInterface):
    _set_pwm_vals: array.array

    def __init__(self, system_id: str):
        super().__init__(system_id)
        self._set_pwm_vals = array.array("I", [0, 0, 0, 0])

    # Setters
    def set_led_output(self, index: int, pwm_out: int) -> bool:
        from ros_robot_driver.ros_main import get_ros_node

        outputs = deepcopy(self._set_pwm_vals)
        outputs[index] = pwm_out   # This is unsafe.
        resp = get_ros_node().set_camera_leds(outputs)

        if resp:
            return resp.success
        return False

    def set_all_outputs(self, pwm_outs: array.array) -> bool:
        from ros_robot_driver.ros_main import get_ros_node
        resp = get_ros_node().set_camera_leds(pwm_outs)

        if resp:
            return resp.success
        return False

    def set_all_full(self) -> None:
        self.set_all_outputs(array.array("I", [65535, 65535, 65535, 65535]))

    def set_all_off(self) -> None:
        self.set_all_outputs(array.array("I", [0, 0, 0, 0]))

    # Getters
    def get_set_pwm_vals(self) -> array.array:
        return self._set_pwm_vals
