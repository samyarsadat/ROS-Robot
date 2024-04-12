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
#include <geometry_msgs/msg/transform_stamped.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_srvs/srv/set_bool.h>
#include <nav_msgs/msg/odometry.h>
#include <rrp_pico_coms/msg/misc_sensors_a.h>
#include <rrp_pico_coms/msg/ultrasonic_sensors.h>
#include <rrp_pico_coms/msg/falloff_sensors.h>
#include <rrp_pico_coms/msg/motor_ctrl_state.h>
#include <rrp_pico_coms/srv/set_pid_tunings.h>
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

// Diagnostics
rcl_publisher_t diagnostics_pub;
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;

// Sensor States
rcl_publisher_t misc_sensor_pub, ultrasonic_sensor_pub, falloff_sensor_pub;
rrp_pico_coms__msg__MiscSensorsA misc_sensor_msg;
rrp_pico_coms__msg__UltrasonicSensors ultrasonic_sensor_msg;
rrp_pico_coms__msg__FalloffSensors falloff_sensor_msg;

// Motor Controller States
rcl_publisher_t mtr_ctrl_r_state_pub, mtr_ctrl_l_state_pub;
rrp_pico_coms__msg__MotorCtrlState mtr_ctrl_r_state_msg, mtr_ctrl_l_state_msg;

// Odometry
rcl_publisher_t enc_odom_pub;
nav_msgs__msg__Odometry enc_odom_msg;

// Odometry -> Base Link Transform
rcl_publisher_t odom_baselink_tf_pub;
geometry_msgs__msg__TransformStamped odom_baselink_tf_msg;


// ---- Services ----

// Motor Controller Enable/Disable
rcl_service_t en_motor_ctrl_srv;
std_srvs__srv__SetBool_Request en_motor_ctrl_req;
std_srvs__srv__SetBool_Response en_motor_ctrl_res;

// Emitters (Ultrasonic, IR Edge) Enable/Disable 
rcl_service_t en_emitters_srv;
std_srvs__srv__SetBool_Request en_emitters_req;
std_srvs__srv__SetBool_Response en_emitters_res;

// Power Relay Enable/Disable  
rcl_service_t en_relay_srv;
std_srvs__srv__SetBool_Request en_relay_req;
std_srvs__srv__SetBool_Response en_relay_res;

// Motor Controller PID Tunings
rcl_service_t set_mtr_pid_tunings_srv;
rrp_pico_coms__srv__SetPidTunings_Request set_mtr_pid_tunings_req;
rrp_pico_coms__srv__SetPidTunings_Response set_mtr_pid_tunings_res;



// ------- Subscriber & service callback prototypes -------
extern void cmd_vel_call(const void *msgin);
extern void clean_shutdown(const void *msgin);
extern void en_motor_ctrl_callback(const void *req, void *res);
extern void en_emitters_callback(const void *req, void *res);
extern void en_relay_callback(const void *req, void *res);
extern void set_mtr_pid_tunings_callback(const void *req, void *res);



// ------- Functions -------

// ---- RCL return checker prototype ----
extern bool check_rc(rcl_ret_t rctc, uint mode);


// ---- Setup subscribers and publishers ----
void init_subs_pubs()
{
    // const rosidl_message_type_support_t *string_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    const rosidl_message_type_support_t *twist_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    const rosidl_message_type_support_t *transform_s_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped);
    const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    const rosidl_message_type_support_t *empty_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    const rosidl_message_type_support_t *odom_type = ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
    const rosidl_message_type_support_t *mtr_ctrl_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MotorCtrlState);
    const rosidl_message_type_support_t *misc_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MiscSensorsA);
    const rosidl_message_type_support_t *ultrasonics_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, UltrasonicSensors);
    const rosidl_message_type_support_t *falloff_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, FalloffSensors);
    const rosidl_service_type_support_t *set_bool_type = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
    const rosidl_service_type_support_t *set_pid_tunings_type = ROSIDL_GET_SRV_TYPE_SUPPORT(rrp_pico_coms, srv, SetPidTunings);

    // ---- Services ----
    check_rc(rclc_service_init_default(&en_motor_ctrl_srv, &rc_node, set_bool_type, "/enable_disable/motor_ctrl"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&en_emitters_srv, &rc_node, set_bool_type, "/enable_disable/emitters"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&en_relay_srv, &rc_node, set_bool_type, "/enable_disable/pico_a_relay"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&set_mtr_pid_tunings_srv, &rc_node, set_pid_tunings_type, "/config/set_motor_pid_tunings"), RT_HARD_CHECK);

    // ---- Command velocity topic ----
    check_rc(rclc_subscription_init_default(&cmd_vel_sub, &rc_node, twist_type, "/cmd_vel"), RT_HARD_CHECK);
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    // ---- E-stop topic ----
    check_rc(rclc_subscription_init_default(&e_stop_sub, &rc_node, empty_type, "/e_stop"), RT_HARD_CHECK);
    std_msgs__msg__Empty__init(&e_stop_msg);

    // ---- Odometry topic ----
    check_rc(rclc_publisher_init_default(&enc_odom_pub, &rc_node, odom_type, "/sensors/enc_odom"), RT_HARD_CHECK);
    nav_msgs__msg__Odometry__init(&enc_odom_msg);

    // ---- Diagnostics ----
    check_rc(rclc_publisher_init_default(&diagnostics_pub, &rc_node, diag_status_type, "/diagnostics"), RT_HARD_CHECK);
    diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg);

    // ---- Sensor state topics ----
    check_rc(rclc_publisher_init_default(&misc_sensor_pub, &rc_node, misc_sensors_type, "/sensors_raw/misc_a"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&ultrasonic_sensor_pub, &rc_node, ultrasonics_sensors_type, "/sensors_raw/ultrasonics"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&falloff_sensor_pub, &rc_node, falloff_sensors_type, "/sensors_raw/falloff"), RT_HARD_CHECK);
    rrp_pico_coms__msg__MiscSensorsA__init(&misc_sensor_msg);
    rrp_pico_coms__msg__UltrasonicSensors__init(&ultrasonic_sensor_msg);
    rrp_pico_coms__msg__FalloffSensors__init(&falloff_sensor_msg);

    // ---- Motor controller states topic ----
    check_rc(rclc_publisher_init_default(&mtr_ctrl_r_state_pub, &rc_node, mtr_ctrl_state_type, "/sensors_raw/mtr_ctrl_right"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&mtr_ctrl_l_state_pub, &rc_node, mtr_ctrl_state_type, "/sensors_raw/mtr_ctrl_left"), RT_HARD_CHECK);
    rrp_pico_coms__msg__MotorCtrlState__init(&mtr_ctrl_r_state_msg);
    rrp_pico_coms__msg__MotorCtrlState__init(&mtr_ctrl_l_state_msg);

    // ---- Odometry -> base link transform topic ----
    check_rc(rclc_publisher_init_default(&odom_baselink_tf_pub, &rc_node, transform_s_type, "/tf/pico_odom_base"), RT_HARD_CHECK);
    geometry_msgs__msg__TransformStamped__init(&odom_baselink_tf_msg);
}


// ---- Executor init ----
void exec_init()
{
    rc_executor = rclc_executor_get_zero_initialized_executor();
    const uint num_handles = 6;

    check_rc(rclc_executor_init(&rc_executor, &rc_supp.context, num_handles, &rc_alloc), RT_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_call, ON_NEW_DATA), RT_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &e_stop_sub, &e_stop_msg, &clean_shutdown, ON_NEW_DATA), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_motor_ctrl_srv, &en_motor_ctrl_req, &en_motor_ctrl_res, en_motor_ctrl_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_emitters_srv, &en_emitters_req, &en_emitters_res, en_emitters_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_relay_srv, &en_relay_req, &en_relay_res, en_relay_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &set_mtr_pid_tunings_srv, &set_mtr_pid_tunings_req, &set_mtr_pid_tunings_res, set_mtr_pid_tunings_callback), RT_HARD_CHECK);
}


// ---- Node init ----
void uros_init(const char *node_name, const char *name_space)
{
    rc_alloc = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp, 0, NULL, &rc_alloc), RT_HARD_CHECK);
    check_rc(rclc_node_init_default(&rc_node, node_name, name_space, &rc_supp), RT_HARD_CHECK);
}