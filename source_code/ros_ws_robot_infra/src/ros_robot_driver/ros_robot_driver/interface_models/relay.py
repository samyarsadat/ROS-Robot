#  The ROS robot project (Robot Driver Package)
#  Relay interface
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


class Relay(BaseInterface):
    _relay_enabled: bool

    def __init__(self, system_id: str):
        super().__init__(system_id)
        self._relay_enabled = False

    # Setters
    def enable_relay(self) -> tuple[bool, str]:
        if not self._relay_enabled:
            from ros_robot_driver.ros_main import get_ros_node
            resp = get_ros_node().en_relay(True)

            if resp:
                if resp.success:
                    self._relay_enabled = True
                    self._call_update_callback_safe()
                return resp.success, resp.message
            return False, RobotConfig.SERVICE_CALL_FAILURE
        return True, RobotConfig.SERVICE_CALL_REDUNDANT

    def disable_relay(self) -> tuple[bool, str]:
        if self._relay_enabled:
            from ros_robot_driver.ros_main import get_ros_node
            resp = get_ros_node().en_relay(False)

            if resp:
                if resp.success:
                    self._relay_enabled = False
                    self._call_update_callback_safe()
                return resp.success, resp.message
            return False, RobotConfig.SERVICE_CALL_FAILURE
        return True, RobotConfig.SERVICE_CALL_REDUNDANT

    # Getters
    def is_enabled(self) -> bool:
        return self._relay_enabled
