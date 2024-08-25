#  The ROS robot project (Robot Driver Package)
#  Temperature and humidity sensor interface
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


class TempSensor(BaseInterface):
    _temperature: float
    _humidity_percent: float

    def __init__(self, sensor_id: str):
        super().__init__(sensor_id)
        self._temperature = 0.0
        self._humidity_percent = 0.0

    def get_temp(self) -> float:
        return self._temperature

    def get_humidity_percent(self) -> float:
        return self._humidity_percent
