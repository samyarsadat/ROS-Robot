#  The ROS robot project - Raspberry Pi Pico Communication Messages Package
#  Miscellaneous Sensor(s) State Message (Pico B)
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


# ---- UI Switches ----
bool control_mode_switch
uint8 speed_sel_switch  # 0: first position, 1: second position, 2: third position

# ---- Battery Power ----
float32 battery_voltage  # Volts
int16 battery_current    # Milliamps

# ---- Misc. ----
bool[4] camera_led_stats  # true:on, false:off
float32 cpu_temp          # RP2040 temperature, Celsius