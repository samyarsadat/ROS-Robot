/*
    The ROS robot project - Diagnostics Report Messages - Pico A
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


// ---- STATUS OK ----
#define DIAG_OK_MSG_IR_EDGE_TEST_PASS    "IR edge sensor self-test passed! [SENSOR: %d]"
#define DIAG_OK_MSG_ULTRA_TEST_PASS      "Ultrasonic sensor self-test passed! [CODE: %d]"
#define DIAG_OK_MSG_MPU6050_TEST_PASSED  "MPU6050 self-test passed! "

// ---- STATUS WARN ----
#define DIAG_WARN_MSG_ULTRA_MIN_LIM_EXCEED  "Ultrasonic sensor distance reading exceeded ultra_min_dist"
#define DIAG_WARN_MSG_ULTRA_MAX_LIM_EXCEED  "Ultrasonic sensor distance reading exceeded ultra_max_dist"

// ---- STATUS ERROR ----
#define DIAG_ERR_MSG_MOTOR_SAFETY       "A MotorSafety condition was triggered"
#define DIAG_ERR_MSG_IR_EDGE_TEST_FAIL  "IR edge sensor self-test failed! [SENSOR: %d]"
#define DIAG_ERR_MSG_ULTRA_TEST_FAIL    "Ultrasonic sensor self-test failed! [CODE: %d]"
#define DIAG_ERR_MSG_MPU6050_TEST_FAIL  "MPU6050 self-test failed! "

// ---- STATUS STALE ----