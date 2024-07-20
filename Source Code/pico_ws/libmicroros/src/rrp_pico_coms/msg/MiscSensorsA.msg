#  The ROS robot project - Raspberry Pi Pico Communication Messages Package
#  Miscellaneous Sensor(s) State Message (Pico A)
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

builtin_interfaces/Time time


# ---- Inertial Measurement Unit ----
geometry_msgs/Vector3 imu_accel    # Linear Acceleration
geometry_msgs/Vector3 imu_gyro     # Angular Velocity
geometry_msgs/Vector3 imu_compass  # Orientation (from extra HMC5883L sensor)
float32 imu_temp                   # Temperature From MPU6050
bool imu_freefall_int              # MPU6050 Freefall Detection Interrupt

# ---- Misc. ----
float32 cpu_temp     # RP2040 temperature, Celsius