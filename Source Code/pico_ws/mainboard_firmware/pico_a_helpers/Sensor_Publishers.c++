/*
    The ROS robot project - Sensor Data MicroROS Publishers - Pico A
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


// ------- Libraries & Modules -------
#include "pico/stdlib.h"
#include "Definitions.h"
#include "uROS_Init.h"
#include "IO_Helpers_Ultrasonic.h"
#include "IO_Helpers_Edge.h"
#include "lib/Helper_lib/Helpers.h"
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <limits>



// ------- Functions ------- 

// ---- RCL return checker prototype ----
extern bool check_rc(rcl_ret_t rctc, uint mode);


// ---- Publish ultrasonic sensor data ----
bool publish_ultra(struct repeating_timer *rt)
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

    ultrasonic_sensor_msg.time.sec = timestamp_sec;
    ultrasonic_sensor_msg.time.nanosec = timestamp_nanosec;
    ultrasonic_sensor_msg.ultrasonic_fov = ultra_fov;
    ultrasonic_sensor_msg.ultrasonic_min_dist = ultra_min_dist;
    ultrasonic_sensor_msg.ultrasonic_max_dist = ultra_max_dist;

    ultrasonic_sensor_msg.ultrasonic_f_reading = get_ultra_dist_single(front_ultra_io, DIAG_HWID_ULTRASONIC_F);
    ultrasonic_sensor_msg.ultrasonic_b_reading = get_ultra_dist_mux(back_ultra_trig_mux, back_ultra_echo_mux, DIAG_HWID_ULTRASONIC_B);
    ultrasonic_sensor_msg.ultrasonic_r_reading = get_ultra_dist_mux(right_ultra_trig_mux, right_ultra_echo_mux, DIAG_HWID_ULTRASONIC_R);
    ultrasonic_sensor_msg.ultrasonic_l_reading = get_ultra_dist_mux(left_ultra_trig_mux, left_ultra_echo_mux, DIAG_HWID_ULTRASONIC_L);

    check_rc(rcl_publish(&ultrasonic_sensor_pub, &ultrasonic_sensor_msg, NULL), RCL_SOFT_CHECK);
    return true;
}


// ---- Publish edge sensor data ----
bool publish_edge_ir(struct repeating_timer *rt)
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;
    
    calibrate_ir_offset();
    bool* readings = get_ir_status();

    falloff_sensor_msg.time.sec = timestamp_sec;
    falloff_sensor_msg.time.nanosec = timestamp_nanosec;
    falloff_sensor_msg.ir_edge_sens_fov = ir_edge_fov;
    falloff_sensor_msg.ir_edge_sens_range = ir_edge_detection_range;

    // FRONT SENSORS
    falloff_sensor_msg.ir_edge_sens_front_trig[0] = readings[0];
    falloff_sensor_msg.ir_edge_sens_front_trig[1] = readings[1];
    falloff_sensor_msg.ir_edge_sens_front_trig[2] = readings[2];
    falloff_sensor_msg.ir_edge_sens_front_trig[3] = readings[3];

    // BACK SENSORS
    falloff_sensor_msg.ir_edge_sens_back_trig[0] = readings[4];
    falloff_sensor_msg.ir_edge_sens_back_trig[1] = readings[5];
    falloff_sensor_msg.ir_edge_sens_back_trig[2] = readings[6];
    falloff_sensor_msg.ir_edge_sens_back_trig[3] = readings[7];

    check_rc(rcl_publish(&falloff_sensor_pub, &falloff_sensor_msg, NULL), RCL_SOFT_CHECK);
    return true;
}


// ---- Other sensor data ----
bool publish_misc_sens(struct repeating_timer *rt)
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

    misc_sensor_msg.time.sec = timestamp_sec;
    misc_sensor_msg.time.nanosec = timestamp_nanosec;
    misc_sensor_msg.wheelbase_mm = wheelbase;
    
    misc_sensor_msg.cpu_temp = get_rp2040_temp();
    misc_sensor_msg;

    return true;
}