#  The ROS robot project - Raspberry Pi Pico Communication Messages Package
#  Motor Controller State Message (Pico A)
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


# ---- Encoder Measurements ----
float32[] measured_rpms
bool[] measured_dirs       # true:forwards, false:backwards
int32[] total_enc_counts   # Total number of counted encoder pulses; TODO: Better explanation.

# ---- Encoder Specs ----
int16 encoder_pulses_per_rotation  # Number of encoder pulses per rotation of the motor shaft, NOT THE WHEEL!
int16 wheel_diameter_mm            # Wheel diameter in millimeters.
float32 motor_gear_ratio_fl        # Motor gearbox ratio (ex. 80/1).

# ---- Set Targets ----
float32 target_rpm
bool target_dir  # true:forwards, false:backwards

# ---- PID ----
uint16 pid_output
float32[3] pid_cals  # [Pk, Pi, Pd]

# ---- Power ----
uint16 total_current        # In milliamps
float32 driver_out_voltage  # In volts