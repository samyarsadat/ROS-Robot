/*
    The ROS robot project - IO Helper Module - Ultrasonic Sensor
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


// ------- Libraries & Modules -------
#include "pico/stdlib.h"


// ------- Functions ------- 

// ---- Dual-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_mux(uint trig_io, uint echo_io, char *ultra_hwid);

// ---- Single-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_single(uint ultra_pin, char *ultra_hwid);