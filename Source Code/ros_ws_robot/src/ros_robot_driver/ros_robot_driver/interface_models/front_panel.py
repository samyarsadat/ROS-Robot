#  The ROS robot project (Robot Driver Package)
#  Robot "front panel" switch states interface
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


class FrontPanelSwitches(BaseInterface):
    _switch_1_state: bool
    _switch_2_state: int   # 0: middle position, 1: first position, 2: second position

    def __init__(self, system_id: str):
        super().__init__(system_id)
        self._switch_1_state = False
        self._switch_2_state = 0

    def get_switch_1_state(self) -> bool:
        return self._switch_1_state

    def get_switch_2_state(self) -> int:
        return self._switch_2_state
