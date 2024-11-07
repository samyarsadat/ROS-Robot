#  The ROS robot project (Robot Driver Package)
#  Ultrasonic sensor interface
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


class UltrasonicSensor(BaseInterface):
    _field_of_view: int
    _min_distance: float
    _max_distance: float
    _reading: float

    def __init__(self, sensor_id: str, field_of_view: int, min_distance: float, max_distance: float):
        super().__init__(sensor_id)
        self._field_of_view = field_of_view
        self._min_distance = min_distance
        self._max_distance = max_distance
        self._reading = 0.0

    def get_distance(self) -> float:
        return self._reading

    def get_field_of_view(self) -> int:
        return self._field_of_view

    def get_min_distance(self) -> float:
        return self._min_distance

    def get_max_distance(self) -> float:
        return self._max_distance
