/*
    The ROS robot project - Program Definitions - Pico B
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
#include "../Common_Definitions.h"
#include "../Diagnostics_Definitons.h"
#include "Diag_Msgs.h"


// ------- Pin definitions -------

// ---- Misc. ----
#define ready_sig   2
#define dht11_sens  14
#define batt_adc    28   // Battery voltage divider

// ---- Settings switches ----
#define speed_sw_1  16
#define speed_sw_2  17
#define mode_sw     22

// ---- Raspberry Pi camera LEDs ----
#define cam_led_1  18
#define cam_led_2  19
#define cam_led_3  20
#define cam_led_4  21

// ---- Micro switches ----
#define ms_front_l  13
#define ms_front_r  12
#define ms_back_l   11
#define ms_back_r   10

// ---- I2C ----
#define i2c_sda  0
#define i2c_scl  1


// ------- Other definitions -------

// ---- Battery ----
#define batt_voltage_divider_r1  10 * 1000    // In ohms (10k ohms)
#define batt_voltage_divider_r2  2.7 * 1000   // In ohms (2.7k ohms)

// ---- MicroROS node config ----
#define UROS_NODE_NAME       "pico_b"
#define UROS_NODE_NAMESPACE  "io"

// ---- Repeating timer intervals ----
#define microsw_pub_rt_interval   100   // In milliseconds
#define sensors_pub_rt_interval   80    // In milliseconds