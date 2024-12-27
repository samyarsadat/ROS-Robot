#  The ROS robot project (Robot Driver Package)
#  Motor and wheel information data structure
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


class MotorWheelInfo:
    _encoder_ppr: int  # Pulses per rotation
    _wheel_diameter_mm: int
    _motor_gear_ratio: float
    _wheelbase_mm: int

    def __init__(self):
        self._encoder_ppr = 0
        self._wheel_diameter_mm = 0
        self._motor_gear_ratio = 0
        self._wheelbase_mm = 0

    def get_encoder_ppr(self) -> int:
        return self._encoder_ppr

    def get_wheel_diameter_mm(self) -> int:
        return self._wheel_diameter_mm

    def get_motor_gear_ratio(self) -> float:
        return self._motor_gear_ratio

    def get_wheelbase_mm(self) -> int:
        return self._wheelbase_mm
