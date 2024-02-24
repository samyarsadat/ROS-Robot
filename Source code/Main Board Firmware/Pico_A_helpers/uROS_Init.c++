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



// ------- Libraries & Modules -------
#include "uROS_Init.h"
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/empty.h>
#include <geometry_msgs/msg/twist.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_srvs/srv/set_bool.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "Definitions.h"



// ------- Variables -------

// ---- General ----
rcl_allocator_t rc_alloc;
rclc_support_t rc_supp;
rcl_node_t rc_node;
rclc_executor_t rc_executor;


// ---- Subscribers ----

// Misc.
rcl_subscription_t e_stop_sub;
std_msgs__msg__Empty e_stop_msg;

// Command & Control
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;


// ---- Publishers ----

// Misc.
rcl_publisher_t diagnostics_pub;
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;

// Ultrasonic Sensors
rcl_publisher_t front_ultra_pub, left_ultra_pub, right_ultra_pub, back_ultra_pub;
sensor_msgs__msg__Range front_ultra_msg, back_ultra_msg, right_ultra_msg, left_ultra_msg;

// IR Edge Sensors
rcl_publisher_t front_1_edge_pub, front_2_edge_pub, front_3_edge_pub, front_4_edge_pub;
rcl_publisher_t back_1_edge_pub, back_2_edge_pub, back_3_edge_pub, back_4_edge_pub;
sensor_msgs__msg__Range front_1_edge_msg, front_2_edge_msg, front_3_edge_msg, front_4_edge_msg;
sensor_msgs__msg__Range back_1_edge_msg, back_2_edge_msg, back_3_edge_msg, back_4_edge_msg;

// Odometry
rcl_publisher_t enc_odom_pub;
nav_msgs__msg__Odometry enc_odom_msg;


// ---- Services ----

// Motor controller enable/disable
rcl_service_t en_motor_ctrl_serv;
std_srvs__srv__SetBool_Request en_motor_ctrl_req;
std_srvs__srv__SetBool_Response en_motor_ctrl_res;

// Emitters (ultrasonic, IR edge) enable/disable 
rcl_service_t en_emitters_serv;
std_srvs__srv__SetBool_Request en_emitters_req;
std_srvs__srv__SetBool_Response en_emitters_res;



// ------- Subscriber & service callback prototypes -------
extern void rpi_ready_call(const void *msgin);
extern void cmd_vel_call(const void *msgin);
extern void clean_shutdown(const void *msgin);
extern void en_motor_ctrl_callback(const void *req, void *res);
extern void en_emitters_callback(const void *req, void *res);



// ------- Functions -------

// ---- RCL return checker prototype ----
extern bool check_rc(rcl_ret_t rctc, uint mode);


// ---- Setup subscribers and publishers ----
void init_subs_pubs()
{
    // const rosidl_message_type_support_t *string_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    const rosidl_message_type_support_t *twist_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    const rosidl_message_type_support_t *range_sens_type = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range);
    const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    const rosidl_message_type_support_t *empty_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    const rosidl_message_type_support_t *odom_type = ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
    const rosidl_service_type_support_t *set_bool_type = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);

    // ---- Enable/disable services ----
    check_rc(rclc_service_init_default(&en_motor_ctrl_serv, &rc_node, set_bool_type, "/enable_disable/motor_ctrl"), RCL_HARD_CHECK);
    check_rc(rclc_service_init_default(&en_emitters_serv, &rc_node, set_bool_type, "/enable_disable/emitters"), RCL_HARD_CHECK);

    // ---- Command velocity topic ----
    check_rc(rclc_subscription_init_default(&cmd_vel_sub, &rc_node, twist_type, "/cmd_vel"), RCL_HARD_CHECK);
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    // ---- E-stop topic ----
    check_rc(rclc_subscription_init_default(&e_stop_sub, &rc_node, empty_type, "/e_stop"), RCL_HARD_CHECK);
    std_msgs__msg__Empty__init(&e_stop_msg);

    // ---- Odometry topic ----
    check_rc(rclc_publisher_init_default(&enc_odom_pub, &rc_node, diag_status_type, "/sensors/enc_odom"), RCL_HARD_CHECK);
    nav_msgs__msg__Odometry__init(&enc_odom_msg);

    // ---- Diagnostics ----
    check_rc(rclc_publisher_init_default(&diagnostics_pub, &rc_node, diag_status_type, "/diagnostics"), RCL_HARD_CHECK);
    diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg);

    // ---- Ultrasonic sensors ----
    check_rc(rclc_publisher_init_default(&front_ultra_pub, &rc_node, range_sens_type, "/sensors/ultrasonic/front"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&left_ultra_pub, &rc_node, range_sens_type, "/sensors/ultrasonic/left"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&right_ultra_pub, &rc_node, range_sens_type, "/sensors/ultrasonic/right"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&back_ultra_pub, &rc_node, range_sens_type, "/sensors/ultrasonic/back"), RCL_HARD_CHECK);
    sensor_msgs__msg__Range__init(&front_ultra_msg);
    sensor_msgs__msg__Range__init(&back_ultra_msg);
    sensor_msgs__msg__Range__init(&right_ultra_msg);
    sensor_msgs__msg__Range__init(&left_ultra_msg);

    // ---- IR edge sensors ----
    check_rc(rclc_publisher_init_default(&front_1_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/front_1"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&front_2_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/front_2"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&front_3_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/front_3"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&front_4_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/front_4"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&back_1_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/back_1"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&back_2_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/back_2"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&back_3_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/back_3"), RCL_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&back_4_edge_pub, &rc_node, range_sens_type, "/sensors/ir_edge/back_4"), RCL_HARD_CHECK);
    sensor_msgs__msg__Range__init(&front_1_edge_msg);
    sensor_msgs__msg__Range__init(&front_2_edge_msg);
    sensor_msgs__msg__Range__init(&front_3_edge_msg);
    sensor_msgs__msg__Range__init(&front_4_edge_msg);
    sensor_msgs__msg__Range__init(&back_1_edge_msg);
    sensor_msgs__msg__Range__init(&back_2_edge_msg);
    sensor_msgs__msg__Range__init(&back_3_edge_msg);
    sensor_msgs__msg__Range__init(&back_4_edge_msg);
}


// ---- Executor init ----
void exec_init()
{
    rc_executor = rclc_executor_get_zero_initialized_executor();
    uint num_handles = 4;

    check_rc(rclc_executor_init(&rc_executor, &rc_supp.context, num_handles, &rc_alloc), RCL_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_call, ON_NEW_DATA), RCL_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &e_stop_sub, &e_stop_msg, &clean_shutdown, ON_NEW_DATA), RCL_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_motor_ctrl_serv, &en_motor_ctrl_req, &en_motor_ctrl_res, en_motor_ctrl_callback), RCL_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_emitters_serv, &en_emitters_req, &en_emitters_res, en_emitters_callback), RCL_HARD_CHECK);
}


// ---- Node init ----
void uros_init(const char *node_name, const char *name_space)
{
    rc_alloc = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp, 0, NULL, &rc_alloc), RCL_HARD_CHECK);
    check_rc(rclc_node_init_default(&rc_node, node_name, name_space, &rc_supp), RCL_HARD_CHECK);
}