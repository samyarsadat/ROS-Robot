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
#include "../../Diagnostics_Names.h"
#include "local_helpers_lib/Common_Definitions.h"
#include "hardware/i2c.h"
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
#define cam_led_1  20
#define cam_led_2  18
#define cam_led_3  19
#define cam_led_4  21

// ---- Micro switches ----
#define ms_front_l  11
#define ms_front_r  10
#define ms_back_l   12
#define ms_back_r   13

// ---- I2C ----
#define i2c_inst  i2c0   // I2C instance (0 & 1 are on I2C0)
#define i2c_sda   0
#define i2c_scl   1


// ------- Other definitions -------

// ---- Battery ----
#define batt_voltage_divider_r1  10 * 1000     // In ohms (10k ohms)
#define batt_voltage_divider_r2  2.67 * 1000   // In ohms (2.7k ohms)

// ---- MicroROS node config ----
#define UROS_NODE_NAME                     "pico_b"
#define UROS_NODE_NAMESPACE                "io"
#define UROS_NODE_DOMAIN_ID                95
#define AGENT_WAITING_LED_TOGGLE_DELAY_MS  500   // In milliseconds
#define AGENT_AVAIL_LED_TOGGLE_DELAY_MS    250   // In milliseconds

// ---- Repeating timer intervals ----
#define microsw_pub_rt_interval      200   // In milliseconds
#define sensors_pub_rt_interval      80    // In milliseconds
#define dht_measurement_rt_interval  1500  // In milliseconds

// ---- Misc. ----
#define SETUP_TASK_STACK_DEPTH       1024       // In FreeRTOS words
#define TIMER_TASK_STACK_DEPTH       1024       // In FreeRTOS words
#define GENERIC_TASK_STACK_DEPTH     1024       // In FreeRTOS words
#define STARTUP_WAIT_TIME_S          3          // In seconds
#define microsw_led_test_timeout_ms  15 * 1000  // 15 seconds