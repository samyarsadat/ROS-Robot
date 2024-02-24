/*
    The ROS robot project - MicroROS Init - Pico A
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
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/empty.h>
#include <geometry_msgs/msg/twist.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_srvs/srv/set_bool.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>



// ------- Variables -------

// ---- General ----
extern rcl_allocator_t rc_alloc;
extern rclc_support_t rc_supp;
extern rcl_node_t rc_node;
extern rclc_executor_t rc_executor;


// ---- Subscribers ----

// Misc.
extern rcl_subscription_t e_stop_sub;
extern std_msgs__msg__Empty e_stop_msg;

// Command & Control
extern rcl_subscription_t cmd_vel_sub;
extern geometry_msgs__msg__Twist cmd_vel_msg;


// ---- Publishers ----

// Misc.
extern rcl_publisher_t diagnostics_pub;
extern diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;

// Ultrasonic Sensors
extern rcl_publisher_t front_ultra_pub, left_ultra_pub, right_ultra_pub, back_ultra_pub;
extern sensor_msgs__msg__Range front_ultra_msg, back_ultra_msg, right_ultra_msg, left_ultra_msg;

// IR Edge Sensors
extern rcl_publisher_t front_1_edge_pub, front_2_edge_pub, front_3_edge_pub, front_4_edge_pub;
extern rcl_publisher_t back_1_edge_pub, back_2_edge_pub, back_3_edge_pub, back_4_edge_pub;
extern sensor_msgs__msg__Range front_1_edge_msg, front_2_edge_msg, front_3_edge_msg, front_4_edge_msg;
extern sensor_msgs__msg__Range back_1_edge_msg, back_2_edge_msg, back_3_edge_msg, back_4_edge_msg;

// Odometry
extern rcl_publisher_t enc_odom_pub;
extern nav_msgs__msg__Odometry enc_odom_msg;


// ---- Services ----

// Motor controller enable/disable
extern rcl_service_t en_motor_ctrl_serv;
extern std_srvs__srv__SetBool_Request en_motor_ctrl_req;
extern std_srvs__srv__SetBool_Response en_motor_ctrl_res;

// Emitters (ultrasonic, IR edge) enable/disable 
extern rcl_service_t en_emitters_serv;
extern std_srvs__srv__SetBool_Request en_emitters_req;
extern std_srvs__srv__SetBool_Response en_emitters_res;



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs();

// ---- Executor init ----
void exec_init();

// ---- Node init ----
void uros_init(const char * node_name, const char * name_space);