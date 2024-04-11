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
#include "../Common_Definitions.h"
#include "../Diagnostics_Definitons.h"
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
#define l_motor_1_enc_b  7
#define l_motor_1_enc_a  9
#define l_motor_2_enc_b  8
#define l_motor_2_enc_a  10
#define r_motor_1_enc_b  22
#define r_motor_1_enc_a  11
#define r_motor_2_enc_b  27
#define r_motor_2_enc_a  12
#define l_motor_drive_1  21
#define l_motor_drive_2  20
#define r_motor_drive_1  19
#define r_motor_drive_2  18

// ---- I2C ----
#define i2c_sda  14
#define i2c_scl  15


// ------- Other definitions -------

// ---- Misc. ----
#define loop_time_max    60*1000    // 60 milliseconds
#define loop_1_time_max  200*1000   // 200 milliseconds

// ---- MicroROS node config ----
#define UROS_NODE_NAME       "pico_a"
#define UROS_NODE_NAMESPACE  "io"

// ---- Repeating timer intervals ----
#define motor_odom_rt_interval    80    // In milliseconds
#define ultra_pub_rt_interval     100   // In milliseconds
#define edge_ir_pub_rt_interval   100   // In milliseconds
#define sensors_pub_rt_interval   100   // In milliseconds

// ---- Ultrasonic sensor specs ----
#define ultra_fov                    30
#define ultra_min_dist               1         // In cm
#define ultra_max_dist               400       // In cm
#define ultrasonic_signal_timout_us  32*1000   // 32 milliseconds

// ---- IR edge sensors ----
#define ir_edge_detection_range          3   // In cm
#define ir_edge_fov                      20
#define num_ir_sensors                   8
#define ir_trigger_limit                 6500
#define ir_self_test_z_score_threshhold  200.0f

// ---- Motor controller & safety ----
#define motor_safety_trigger_timeout        3500   // In milliseconds
#define motor_actual_vs_set_extra_timeout   4000   // In milliseconds
#define motor_single_side_max_difference    10     // In RPM
#define motor_set_vs_actual_max_difference  60     // In RPM
#define right_motor_controller_id           0
#define left_motor_controller_id            1

// ---- Motor specs ----
#define enc_pulses_per_rotation         2
#define motor_gear_ratio                80/1
#define wheel_diameter                  100.0f                  // In millimeters
#define wheelbase                       140.0f                  // In millimeters
#define wheel_circumference             (PI * wheel_diameter)   // In millimeters
#define enc_pulses_per_meter_of_travel  (1000 / wheel_circumference) * (motor_gear_ratio * enc_pulses_per_rotation)