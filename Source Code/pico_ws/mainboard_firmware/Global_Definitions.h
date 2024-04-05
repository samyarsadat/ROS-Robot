/*
    The ROS robot project - Global Program Definitions
    These are used by the Picos as well as the Raspberry Pi computer and
    the workstation programs. These are mostly physical parameters and configuration info.

    Copyright 2022-2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2022-2024.
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https: www.gnu.org/licenses/>.
*/

#pragma once
#include "Frameid_Definitions.h"


// ---- Motor & Encoder Params ----
#define enc_pulses_per_rotation         2
#define motor_gear_ratio                80/1
#define wheel_diameter                  100.0f                  // In millimeters
#define wheelbase                       140.0f                  // In millimeters
#define wheel_circumference             (PI * wheel_diameter)   // In millimeters
#define enc_pulses_per_meter_of_travel  (1000 / wheel_circumference) * (motor_gear_ratio * enc_pulses_per_rotation)


// ---- Frame IDs ----