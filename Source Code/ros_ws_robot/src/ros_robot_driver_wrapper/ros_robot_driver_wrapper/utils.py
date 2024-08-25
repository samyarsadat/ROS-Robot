#  The ROS robot project (Robot Driver ROS Wrapper Package)
#  General utilities
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
from math import cos, sin


# Converts Euler angles to a quaternion
# Output: [x, y, z, w]
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> array.array:
    yaw_half = yaw * 0.5
    pitch_half = pitch * 0.5
    roll_half = roll * 0.5

    cy = cos(yaw_half)
    sy = sin(yaw_half)
    cp = cos(pitch_half)
    sp = sin(pitch_half)
    cr = cos(roll_half)
    sr = sin(roll_half)

    quaternion = array.array("f")
    quaternion.append(sr * cp * cy - cr * sp * sy)  # x
    quaternion.append(cr * sp * cy + sr * cp * sy)  # y
    quaternion.append(cr * cp * sy - sr * sp * cy)  # z
    quaternion.append(cr * cp * cy + sr * sp * sy)  # w

    return quaternion
