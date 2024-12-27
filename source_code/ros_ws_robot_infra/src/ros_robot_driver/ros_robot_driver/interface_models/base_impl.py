#  The ROS robot project (Robot Driver Package)
#  Base sensor interface implementation
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

import array
from typing import Callable


# Base interface implementation
class BaseInterface:
    _last_timestamp: array.array  # [seconds, nanoseconds]
    _on_update_callback: Callable[[], None]
    _id: str

    def __init__(self, system_id: str):
        self._id = system_id
        self._on_update_callback = None
        self._last_timestamp = array.array("i", [0, 0])

    def set_on_update_callback_func(self, func: Callable[[], None]) -> None:
        self._on_update_callback = func

    def get_last_timestamp(self) -> array.array:
        return self._last_timestamp

    def get_id(self) -> str:
        return self._id

    def _call_update_callback_safe(self) -> None:
        if not self._on_update_callback is None:
            self._on_update_callback()
