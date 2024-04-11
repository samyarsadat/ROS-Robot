#  The ROS robot project - Raspberry Pi Pico Communication Messages Package
#  Ultrasonic Distance Sensor States Message (Pico A)
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2022-2024.
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

builtin_interfaces/Time time


# ---- Ultrasonics ----
uint8 ultrasonic_fov          # Field of View
uint16 ultrasonic_min_dist    # Minimum measurement distance
uint16 ultrasonic_max_dist    # Maximum measurement distance
float32 ultrasonic_f_reading
float32 ultrasonic_b_reading
float32 ultrasonic_r_reading
float32 ultrasonic_l_reading