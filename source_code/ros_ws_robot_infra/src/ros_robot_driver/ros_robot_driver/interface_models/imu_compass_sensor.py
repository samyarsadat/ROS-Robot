#  The ROS robot project (Robot Driver Package)
#  IMU and compass sensor interface
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
import array


class ImuCompassSensor(BaseInterface):
    _acceleration: array.array   # [x, y, z]
    _gyroscope: array.array      # [x, y, z]
    _compass: array.array        # [x, y, z]
    _temperature: float
    _is_free_fall_triggered: bool

    def __init__(self, sensor_id: str):
        super().__init__(sensor_id)
        self._acceleration = array.array("f", [0, 0, 0])
        self._gyroscope = array.array("f", [0, 0, 0])
        self._compass = array.array("f", [0, 0, 0])
        self._temperature = 0
        self._is_free_fall_triggered = False

    def get_accel(self) -> array.array:
        return self._acceleration

    def get_gyro(self) -> array.array:
        return self._gyroscope

    def get_compass(self) -> array.array:
        return self._compass

    def get_temperature(self) -> float:
        return self._temperature

    def is_in_freefall(self) -> bool:
        return self._is_free_fall_triggered
