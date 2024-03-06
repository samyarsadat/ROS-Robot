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

    front_ultra_msg.range = get_ultra_dist_single(front_ultra_io, DIAG_HWID_ULTRASONIC_F);
    front_ultra_msg.field_of_view = ultra_fov;
    front_ultra_msg.max_range = ultra_max_dist;
    front_ultra_msg.min_range = ultra_min_dist;
    front_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    front_ultra_msg.header.stamp.sec = timestamp_sec;
    front_ultra_msg.header.stamp.nanosec = timestamp_nanosec;

    back_ultra_msg.range = get_ultra_dist_mux(back_ultra_trig_mux, back_ultra_echo_mux, DIAG_HWID_ULTRASONIC_B);
    back_ultra_msg.field_of_view = ultra_fov;
    back_ultra_msg.max_range = ultra_max_dist;
    back_ultra_msg.min_range = ultra_min_dist;
    back_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    back_ultra_msg.header.stamp.sec = timestamp_sec;
    back_ultra_msg.header.stamp.nanosec = timestamp_nanosec;

    right_ultra_msg.range = get_ultra_dist_mux(right_ultra_trig_mux, right_ultra_echo_mux, DIAG_HWID_ULTRASONIC_R);
    right_ultra_msg.field_of_view = ultra_fov;
    right_ultra_msg.max_range = ultra_max_dist;
    right_ultra_msg.min_range = ultra_min_dist;
    right_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    right_ultra_msg.header.stamp.sec = timestamp_sec;
    right_ultra_msg.header.stamp.nanosec = timestamp_nanosec;

    left_ultra_msg.range = get_ultra_dist_mux(left_ultra_trig_mux, left_ultra_echo_mux, DIAG_HWID_ULTRASONIC_L);
    left_ultra_msg.field_of_view = ultra_fov;
    left_ultra_msg.max_range = ultra_max_dist;
    left_ultra_msg.min_range = ultra_min_dist;
    left_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    left_ultra_msg.header.stamp.sec = timestamp_sec;
    left_ultra_msg.header.stamp.nanosec = timestamp_nanosec;

    // Publish
    check_rc(rcl_publish(&front_ultra_pub, &front_ultra_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&back_ultra_pub, &back_ultra_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&right_ultra_pub, &right_ultra_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&left_ultra_pub, &left_ultra_msg, NULL), RCL_SOFT_CHECK);

    return true;
}


// ---- Publish edge sensor data ----
bool publish_edge_ir(struct repeating_timer *rt)
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;
    
    calibrate_ir_offset();
    bool* readings = get_ir_status();

    // FRONT SENSORS
    front_1_edge_msg.range = readings[0] ? N_INF : INF;
    front_1_edge_msg.field_of_view = ir_edge_fov;
    front_1_edge_msg.max_range = ir_edge_detection_range;
    front_1_edge_msg.min_range = ir_edge_detection_range;
    front_1_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    front_1_edge_msg.header.stamp.sec = timestamp_sec;
    front_1_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    front_2_edge_msg.range = readings[1] ? N_INF : INF;
    front_2_edge_msg.field_of_view = ir_edge_fov;
    front_2_edge_msg.max_range = ir_edge_detection_range;
    front_2_edge_msg.min_range = ir_edge_detection_range;
    front_2_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    front_2_edge_msg.header.stamp.sec = timestamp_sec;
    front_2_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    front_3_edge_msg.range = readings[2] ? N_INF : INF;
    front_3_edge_msg.field_of_view = ir_edge_fov;
    front_3_edge_msg.max_range = ir_edge_detection_range;
    front_3_edge_msg.min_range = ir_edge_detection_range;
    front_3_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    front_3_edge_msg.header.stamp.sec = timestamp_sec;
    front_3_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    front_4_edge_msg.range = readings[3] ? N_INF : INF;
    front_4_edge_msg.field_of_view = ir_edge_fov;
    front_4_edge_msg.max_range = ir_edge_detection_range;
    front_4_edge_msg.min_range = ir_edge_detection_range;
    front_4_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    front_4_edge_msg.header.stamp.sec = timestamp_sec;
    front_4_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    // BACK SENSORS
    back_1_edge_msg.range = readings[4] ? N_INF : INF;
    back_1_edge_msg.field_of_view = ir_edge_fov;
    back_1_edge_msg.max_range = ir_edge_detection_range;
    back_1_edge_msg.min_range = ir_edge_detection_range;
    back_1_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    back_1_edge_msg.header.stamp.sec = timestamp_sec;
    back_1_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    back_2_edge_msg.range = readings[5] ? N_INF : INF;
    back_2_edge_msg.field_of_view = ir_edge_fov;
    back_2_edge_msg.max_range = ir_edge_detection_range;
    back_2_edge_msg.min_range = ir_edge_detection_range;
    back_2_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    back_2_edge_msg.header.stamp.sec = timestamp_sec;
    back_2_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    back_3_edge_msg.range = readings[6] ? N_INF : INF;
    back_3_edge_msg.field_of_view = ir_edge_fov;
    back_3_edge_msg.max_range = ir_edge_detection_range;
    back_3_edge_msg.min_range = ir_edge_detection_range;
    back_3_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    back_3_edge_msg.header.stamp.sec = timestamp_sec;
    back_3_edge_msg.header.stamp.nanosec = timestamp_nanosec;

    back_4_edge_msg.range = readings[7] ? N_INF : INF;
    back_4_edge_msg.field_of_view = ir_edge_fov;
    back_4_edge_msg.max_range = ir_edge_detection_range;
    back_4_edge_msg.min_range = ir_edge_detection_range;
    back_4_edge_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    back_4_edge_msg.header.stamp.sec = timestamp_sec;
    back_4_edge_msg.header.stamp.nanosec = timestamp_nanosec;
    
    // Publish
    check_rc(rcl_publish(&front_1_edge_pub, &front_1_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&front_2_edge_pub, &front_2_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&front_3_edge_pub, &front_3_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&front_4_edge_pub, &front_4_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&back_1_edge_pub, &back_1_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&back_2_edge_pub, &back_2_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&back_3_edge_pub, &back_3_edge_msg, NULL), RCL_SOFT_CHECK);
    check_rc(rcl_publish(&back_4_edge_pub, &back_4_edge_msg, NULL), RCL_SOFT_CHECK);

    return true;
}