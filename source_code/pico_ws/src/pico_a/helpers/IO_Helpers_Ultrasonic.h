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
#include <string>
#include <functional>


// ------- Definitions -------
enum ULTRA_TEST_RESULT_STATUS
{
    ULTRA_TEST_PASS,
    ULTRA_TEST_FAIL_OUT_OF_TEST_RANGE,   // Test range is 18 to 22 centimeters
    ULTRA_TEST_FAIL_BELOW_ABSOLUTE_MINIMUM,
    ULTRA_TEST_FAIL_ABOVE_ABSOLUTE_MAXIMUM
};

struct ULTRA_TEST_RESULT
{
    ULTRA_TEST_RESULT_STATUS status;
    float measured_distance;
};

typedef struct ULTRA_TEST_RESULT ULTRA_TEST_RESULT_t;


// ------- Functions ------- 

// ---- Dual-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_mux(uint trig_io, uint echo_io, std::string ultra_hwid);

// ---- Single-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_single(uint ultra_pin, std::string ultra_hwid);

// ---- Ultrasonic sensor self-test ----
ULTRA_TEST_RESULT_t ultra_self_test(std::function<float()> sensor_get_dist_func);