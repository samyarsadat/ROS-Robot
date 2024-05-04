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
#include <diagnostic_msgs/srv/self_test.h>
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
#include "local_helpers_lib/Local_Helpers.h"



// ------- Variables -------

// ---- General ----
rcl_allocator_t rc_alloc, rc_alloc_1;
rclc_support_t rc_supp, rc_supp_1;
rcl_node_t rc_node, rc_node_1;
rclc_executor_t rc_executor, rc_executor_1;


// ---- Executor Timers ----
rcl_timer_t motor_odom_timer, ultrasonic_publish_timer;
rcl_timer_t edge_ir_publish_timer, other_sensors_publish_timer;


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

// Initiate the self-test function
rcl_service_t run_self_test_srv;
diagnostic_msgs__srv__SelfTest_Request run_self_test_req;
diagnostic_msgs__srv__SelfTest_Response run_self_test_res;



// ------- Subscriber & service callback prototypes -------
extern void cmd_vel_call(const void *msgin);
extern void en_motor_ctrl_callback(const void *req, void *res);
extern void en_emitters_callback(const void *req, void *res);
extern void en_relay_callback(const void *req, void *res);
extern void set_mtr_pid_tunings_callback(const void *req, void *res);
extern void run_self_test_callback(const void *req, void *res);
extern void clean_shutdown();
void clean_shutdown_callback(const void *msgin) { clean_shutdown(); }

// ---- Timer callbacks ----
/*extern void publish_ultra(rcl_timer_t *timer, int64_t last_call_time);
extern void publish_edge_ir(rcl_timer_t *timer, int64_t last_call_time);
extern void publish_misc_sens(rcl_timer_t *timer, int64_t last_call_time);
extern void motor_ctrl_odom_timer_call(rcl_timer_t *timer, int64_t last_call_time);*/



// ------- Functions -------

// ---- Setup subscribers, publishers, services, and timers ----
void init_subs_pubs()
{
    write_log("init_subs_pubs", "Initializing publishers, subscribers, services, and timers...", LOG_LVL_INFO);

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
    const rosidl_service_type_support_t *run_self_test_type = ROSIDL_GET_SRV_TYPE_SUPPORT(diagnostic_msgs, srv, SelfTest);


    // ---- Timers ----
    write_log("init_subs_pubs", "Initializing timers...", LOG_LVL_INFO);
    /*check_rc(rclc_timer_init_default(&motor_odom_timer, &rc_supp, RCL_MS_TO_NS(ultra_pub_rt_interval), publish_ultra), RT_HARD_CHECK);
    check_rc(rclc_timer_init_default(&ultrasonic_publish_timer, &rc_supp_1, RCL_MS_TO_NS(edge_ir_pub_rt_interval), publish_edge_ir), RT_HARD_CHECK);
    check_rc(rclc_timer_init_default(&edge_ir_publish_timer, &rc_supp_1, RCL_MS_TO_NS(sensors_pub_rt_interval), publish_misc_sens), RT_HARD_CHECK);
    check_rc(rclc_timer_init_default(&other_sensors_publish_rt, &rc_supp_1, RCL_MS_TO_NS(motor_odom_rt_interval), motor_ctrl_odom_timer_call), RT_HARD_CHECK);*/


    // ---- Services ----
    write_log("init_subs_pubs", "Initializing services...", LOG_LVL_INFO);
    check_rc(rclc_service_init_default(&en_motor_ctrl_srv, &rc_node, set_bool_type, "/enable_disable/motor_ctrl"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&en_emitters_srv, &rc_node, set_bool_type, "/enable_disable/emitters"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&en_relay_srv, &rc_node, set_bool_type, "/enable_disable/pico_a_relay"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&set_mtr_pid_tunings_srv, &rc_node, set_pid_tunings_type, "/config/set_motor_pid_tunings"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&run_self_test_srv, &rc_node, run_self_test_type, "/self_test/pico_a"), RT_HARD_CHECK);


    // ---- Subscribers ----
    write_log("init_subs_pubs", "Initializing subscribers...", LOG_LVL_INFO);

    // Command velocity topic
    check_rc(rclc_subscription_init_default(&cmd_vel_sub, &rc_node, twist_type, "/cmd_vel"), RT_HARD_CHECK);
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    // E-stop topic
    check_rc(rclc_subscription_init_default(&e_stop_sub, &rc_node, empty_type, "/e_stop"), RT_HARD_CHECK);
    std_msgs__msg__Empty__init(&e_stop_msg);


    // ---- Publishers ----
    write_log("init_subs_pubs", "Initializing publishers...", LOG_LVL_INFO);

    // Odometry topic
    check_rc(rclc_publisher_init_default(&enc_odom_pub, &rc_node, odom_type, "/sensors/enc_odom"), RT_HARD_CHECK);
    nav_msgs__msg__Odometry__init(&enc_odom_msg);

    // Diagnostics
    check_rc(rclc_publisher_init_default(&diagnostics_pub, &rc_node, diag_status_type, "/diagnostics"), RT_HARD_CHECK);
    diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg);

    // Sensor state topics
    check_rc(rclc_publisher_init_default(&misc_sensor_pub, &rc_node, misc_sensors_type, "/sensors_raw/misc_a"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&ultrasonic_sensor_pub, &rc_node, ultrasonics_sensors_type, "/sensors_raw/ultrasonics"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&falloff_sensor_pub, &rc_node, falloff_sensors_type, "/sensors_raw/falloff"), RT_HARD_CHECK);
    rrp_pico_coms__msg__MiscSensorsA__init(&misc_sensor_msg);
    rrp_pico_coms__msg__UltrasonicSensors__init(&ultrasonic_sensor_msg);
    rrp_pico_coms__msg__FalloffSensors__init(&falloff_sensor_msg);

    // Motor controller state topics
    check_rc(rclc_publisher_init_default(&mtr_ctrl_r_state_pub, &rc_node, mtr_ctrl_state_type, "/sensors_raw/mtr_ctrl_right"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&mtr_ctrl_l_state_pub, &rc_node, mtr_ctrl_state_type, "/sensors_raw/mtr_ctrl_left"), RT_HARD_CHECK);
    rrp_pico_coms__msg__MotorCtrlState__init(&mtr_ctrl_r_state_msg);
    rrp_pico_coms__msg__MotorCtrlState__init(&mtr_ctrl_l_state_msg);

    // Odometry -> base link transform topic 
    // TODO: This should be broadcast.
    check_rc(rclc_publisher_init_default(&odom_baselink_tf_pub, &rc_node, transform_s_type, "/tf/pico_odom_base"), RT_HARD_CHECK);
    geometry_msgs__msg__TransformStamped__init(&odom_baselink_tf_msg);


    write_log("init_subs_pubs", "Init. completed.", LOG_LVL_INFO);
}


// ---- Executor init ----
void exec_init()
{
    write_log("exec_init", "Initializing MicroROS executors...", LOG_LVL_INFO);

    rc_executor = rclc_executor_get_zero_initialized_executor();
    rc_executor_1 = rclc_executor_get_zero_initialized_executor();
    const uint num_handles_core0 = 5;
    const uint num_handles_core1 = 2;

    // Core 0 executor
    write_log("exec_init", "Core 0 executor...", LOG_LVL_INFO);
    check_rc(rclc_executor_init(&rc_executor, &rc_supp.context, num_handles_core0, &rc_alloc), RT_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_call, ON_NEW_DATA), RT_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &e_stop_sub, &e_stop_msg, &clean_shutdown_callback, ON_NEW_DATA), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_motor_ctrl_srv, &en_motor_ctrl_req, &en_motor_ctrl_res, &en_motor_ctrl_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &set_mtr_pid_tunings_srv, &set_mtr_pid_tunings_req, &set_mtr_pid_tunings_res, &set_mtr_pid_tunings_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &run_self_test_srv, &run_self_test_req, &run_self_test_res, &run_self_test_callback), RT_HARD_CHECK);

    // Core 1 executor
    write_log("exec_init", "Core 1 executor...", LOG_LVL_INFO);
    check_rc(rclc_executor_init(&rc_executor_1, &rc_supp_1.context, num_handles_core1, &rc_alloc_1), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_emitters_srv, &en_emitters_req, &en_emitters_res, &en_emitters_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_relay_srv, &en_relay_req, &en_relay_res, &en_relay_callback), RT_HARD_CHECK);

    write_log("exec_init", "Init. completed.", LOG_LVL_INFO);
}


// ---- Node init ----
void uros_init(const char *node_name, const char *name_space)
{
    std::string core0_node_name = std::string(node_name) + "_core0";
    std::string core1_node_name = std::string(node_name) + "_core1";

    write_log("uros_init", "Initializing the MicroROS node (core 0 node)...", LOG_LVL_INFO);
    rc_alloc = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp, 0, NULL, &rc_alloc), RT_HARD_CHECK);
    check_rc(rclc_node_init_default(&rc_node, core0_node_name.c_str(), name_space, &rc_supp), RT_HARD_CHECK);

    write_log("uros_init", "Initializing the MicroROS node (core 1 node)...", LOG_LVL_INFO);
    rc_alloc_1 = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp_1, 0, NULL, &rc_alloc_1), RT_HARD_CHECK);
    check_rc(rclc_node_init_default(&rc_node_1, core1_node_name.c_str(), name_space, &rc_supp_1), RT_HARD_CHECK);
    
    write_log("uros_init", "Init. completed.", LOG_LVL_INFO);
}