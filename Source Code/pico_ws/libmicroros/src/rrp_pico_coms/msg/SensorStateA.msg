#  The ROS robot project - Raspberry Pi Pico Communication Messages Package
#  Sensor(s) State Message (Pico A)
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

std_msgs/Header header


# ---- Ultrasonics ----
uint8 ultra_fov          # Field of View
uint16 ultra_min_dist    # Minimum measurement distance
uint16 ultra_max_dist    # Maximum measurement distance
float32 ultra_f_reading
float32 ultra_b_reading
float32 ultra_r_reading
float32 ultra_l_reading
string ultra_f_frame_id
string ultra_l_frame_id
string ultra_r_frame_id
string ultra_b_frame_id

# ---- IR Edge Sensors ----
uint8 ir_edge_fov                # Field of View
uint8 ir_edge_range              # IR detection range
bool[4] ir_edge_front_trig
bool[4] ir_edge_back_trig
string[4] ir_edge_front_frame_id
string[4] ir_edge_back_frame_id

# ---- Inertial Measurement Unit ----
geometry_msgs/Vector3 imu_accel  # Linear Acceleration
geometry_msgs/Vector3 imu_gyro   # Angular Velocity
string imu_frame_id

# ---- Battery Power ----
float32 battery_voltage  # Volts
int16 battery_current    # Milliamps

# ---- Environmental Sensors ----
float32 env_temp      # Celsius
float32 env_humidity  # Relative Humidity %

# ---- Misc. ----
float32 cpu_temp  # RP2040 temperature, Celsius