#  The ROS robot project (Robot Driver Package)
#  Odometry interface
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


class Odometry(BaseInterface):
    _position: array.array        # [X, Y, Z]
    _orientation: array.array     # [X, Y, Z, W] (quaternion)
    _linear_accel: array.array    # [X, Y, Z]
    _angular_accel: array.array   # [X, Y, Z]
    _header_frame_id: str
    _child_frame_id: str

    def __init__(self, system_id: str):
        super().__init__(system_id)
        self._position = array.array("f", [0, 0, 0])
        self._orientation = array.array("f", [0, 0, 0, 0])
        self._linear_accel = array.array("f", [0, 0, 0])
        self._angular_accel = array.array("f", [0, 0, 0])
        self._header_frame_id = ""
        self._child_frame_id = ""

    def get_position(self) -> array.array:
        return self._position

    def get_orientation(self) -> array.array:
        return self._orientation

    def get_linear_accel(self) -> array.array:
        return self._linear_accel

    def get_angular_accel(self) -> array.array:
        return self._angular_accel

    def get_header_frame_id(self) -> str:
        return self._header_frame_id

    def get_child_frame_id(self) -> str:
        return self._child_frame_id
