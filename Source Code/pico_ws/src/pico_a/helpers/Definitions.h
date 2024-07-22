/*
    The ROS robot project - Program Definitions - Pico A
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
#include "local_helpers_lib/Common_Definitions.h"
#include "hardware/i2c.h"
#include "Diag_Msgs.h"


// ------- Pin definitions -------

// ---- Misc. ----
#define power_led       1
#define ready_sig       28
#define pi_power_relay  6
#define edge_sens_en    0

// ---- Ultrasonic sensor ----
#define front_ultra_io        13
#define back_ultra_trig_mux   0
#define back_ultra_echo_mux   1
#define right_ultra_trig_mux  2
#define right_ultra_echo_mux  3
#define left_ultra_trig_mux   4
#define left_ultra_echo_mux   5
#define extra_ultra_trig_mux  6
#define extra_ultra_echo_mux  7

// ---- Analog multiplexer ----
#define analog_mux_s0  3
#define analog_mux_s1  2
#define analog_mux_s2  4
#define analog_mux_s3  5
#define analog_mux_io  26

// ---- Motor encoders & motor driver ----
#define l_motor_1_enc_b  12   // Supposed to be 27, temporarily swapped.
#define l_motor_1_enc_a  27   // Supposed to be 12, temporarily swapped.
#define l_motor_2_enc_b  10   // Supposed to be 9, temporarily swapped.
#define l_motor_2_enc_a  9    // Supposed to be 10, temporarily swapped.
#define r_motor_1_enc_b  8
#define r_motor_1_enc_a  7
#define r_motor_2_enc_b  11
#define r_motor_2_enc_a  22
#define l_motor_drive_1  18
#define l_motor_drive_2  19
#define r_motor_drive_1  20
#define r_motor_drive_2  21

// ---- I2C ----
#define i2c_inst  i2c1   // I2C instance (14 & 15 are on I2C1)
#define i2c_sda   14
#define i2c_scl   15


// ------- Other definitions -------

// ---- MicroROS node config ----
#define UROS_NODE_NAME                     "pico_a"
#define UROS_NODE_NAMESPACE                "io"
#define AGENT_WAITING_LED_TOGGLE_DELAY_MS  500   // In milliseconds
#define AGENT_AVAIL_LED_TOGGLE_DELAY_MS    250   // In milliseconds

// ---- Repeating timer intervals ----
#define motor_odom_rt_interval          100   // In milliseconds
#define ultra_pub_rt_interval           200   // In milliseconds
#define edge_ir_pub_rt_interval         100   // In milliseconds
#define sensors_pub_rt_interval         100   // In milliseconds
#define motor_enc_method_2_rt_interval  150   // In milliseconds

// ---- Ultrasonic sensor specs ----
#define ultra_fov                    30
#define ultra_min_dist               1         // In cm
#define ultra_max_dist               400       // In cm
#define ultrasonic_signal_timout_us  24*1000   // 24 milliseconds
#define ultra_selftest_measurements  5
#define ultra_selftest_range_min_cm  18        // In cm
#define ultra_selftest_range_max_cm  22        // In cm

// ---- IR edge sensors ----
#define ir_edge_detection_range          3   // In cm
#define ir_edge_fov                      20
#define num_ir_sensors                   8
#define ir_trigger_limit                 2000
#define ir_self_test_z_score_threshhold  2.5f

// ---- Motor controller & safety ----
#define motor_safety_trigger_timeout        3500     // In milliseconds
#define motor_actual_vs_set_extra_timeout   4000     // In milliseconds
#define motor_single_side_max_difference    10       // In RPM
#define motor_set_vs_actual_max_difference  60       // In RPM
#define motor_rpm_method_1_cutoff           18       // In RPM
#define right_motor_controller_id           0
#define left_motor_controller_id            1
#define motor_auto_reverse_cooldown_ms      4*1000   // 4 seconds

// ---- Motor specs ----
#define enc_pulses_per_rot              2
#define gear_ratio_motor                80/1
#define wheel_diameter                  100.0f                  // In millimeters
#define wheelbase                       140.0f                  // In millimeters
#define wheel_circumference             (PI * wheel_diameter)   // In millimeters
#define enc_pulses_per_meter_of_travel  (1000 / wheel_circumference) * (gear_ratio_motor * enc_pulses_per_rot)

// ---- Frame IDs ----
#define odom_frame_id       (char *) "odom"
#define base_link_frame_id  (char *) "base_link"

// ---- Misc. ----
#define SETUP_TASK_STACK_DEPTH    1024   // In FreeRTOS words
#define TIMER_TASK_STACK_DEPTH    1024   // In FreeRTOS words
#define GENERIC_TASK_STACK_DEPTH  1024   // In FreeRTOS words
#define STARTUP_WAIT_TIME_S       3      // In seconds