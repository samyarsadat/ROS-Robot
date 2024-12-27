#  The ROS robot project (Robot Driver Package)
#  Battery information interface
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
from enum import Enum


class BatteryInfo(BaseInterface):
    class BatteryTechnology(Enum):
        TECHNOLOGY_UNKNOWN = 0
        TECHNOLOGY_NIMH = 1
        TECHNOLOGY_LION = 2
        TECHNOLOGY_LIPO = 3
        TECHNOLOGY_LIFE = 4
        TECHNOLOGY_NICD = 5
        TECHNOLOGY_LIMN = 6

    _power_voltage: float
    _power_current_milliamps: int
    _battery_tech: BatteryTechnology
    _battery_capacity: float   # Amp-hours

    def __init__(self, system_id: str, battery_tech: BatteryTechnology, battery_capacity: float):
        super().__init__(system_id)
        self._battery_tech = battery_tech
        self._battery_capacity = battery_capacity
        self._power_voltage = 0.0
        self._power_current_milliamps = 0

    def get_power_voltage(self) -> float:
        return self._power_voltage

    def get_power_current_ma(self) -> int:
        return self._power_current_milliamps

    def get_battery_technology(self) -> BatteryTechnology:
        return self._battery_tech

    def get_battery_capacity_ah(self) -> float:
        return self._battery_capacity
