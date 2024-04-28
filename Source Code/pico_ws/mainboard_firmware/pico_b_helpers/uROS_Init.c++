/*
    The ROS robot project - MicroROS Init - Pico B
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
#include <rrp_pico_coms/msg/misc_sensors_b.h>
#include <rrp_pico_coms/msg/micro_sw_sensors.h>
#include <rrp_pico_coms/srv/set_camera_leds.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <diagnostic_msgs/srv/self_test.h>
#include <rclc/executor.h>
#include "Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"



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


// ---- Publishers ----

// Diagnostics
rcl_publisher_t diagnostics_pub;
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;

// Sensor States
rcl_publisher_t misc_sensor_pub, microsw_sensor_pub;
rrp_pico_coms__msg__MiscSensorsB misc_sensor_msg;
rrp_pico_coms__msg__MicroSwSensors microsw_sensor_msg;


// ---- Services ----

// Motor Controller Enable/Disable
rcl_service_t en_camera_leds_srv;
rrp_pico_coms__srv__SetCameraLeds_Request en_camera_leds_req;
rrp_pico_coms__srv__SetCameraLeds_Response en_camera_leds_res;

// Initiate the self-test function
rcl_service_t run_self_test_srv;
diagnostic_msgs__srv__SelfTest_Request run_self_test_req;
diagnostic_msgs__srv__SelfTest_Response run_self_test_res;



// ------- Subscriber & service callback prototypes -------
extern void en_camera_leds_callback(const void *req, void *res);
extern void run_self_test_callback(const void *req, void *res);
extern void clean_shutdown();
void clean_shutdown_callback(const void *msgin) { clean_shutdown(); }



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs()
{
    const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    const rosidl_message_type_support_t *empty_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    const rosidl_message_type_support_t *misc_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MiscSensorsB);
    const rosidl_message_type_support_t *microsw_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MicroSwSensors);
    const rosidl_service_type_support_t *set_camera_leds_type = ROSIDL_GET_SRV_TYPE_SUPPORT(rrp_pico_coms, srv, SetCameraLeds);
    const rosidl_service_type_support_t *run_self_test_type = ROSIDL_GET_SRV_TYPE_SUPPORT(diagnostic_msgs, srv, SelfTest);

    // ---- Services ----
    check_rc(rclc_service_init_default(&en_camera_leds_srv, &rc_node, set_camera_leds_type, "/enable_disable/camera_leds"), RT_HARD_CHECK);
    check_rc(rclc_service_init_default(&run_self_test_srv, &rc_node, run_self_test_type, "/self-test/pico_b"), RT_HARD_CHECK);

    // ---- E-stop topic ----
    check_rc(rclc_subscription_init_default(&e_stop_sub, &rc_node, empty_type, "/e_stop"), RT_HARD_CHECK);
    std_msgs__msg__Empty__init(&e_stop_msg);

    // ---- Diagnostics ----
    check_rc(rclc_publisher_init_default(&diagnostics_pub, &rc_node, diag_status_type, "/diagnostics"), RT_HARD_CHECK);
    diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg);

    // ---- Sensor state topics ----
    check_rc(rclc_publisher_init_default(&misc_sensor_pub, &rc_node, misc_sensors_type, "/sensors_raw/misc_b"), RT_HARD_CHECK);
    check_rc(rclc_publisher_init_default(&microsw_sensor_pub, &rc_node, microsw_sensors_type, "/sensors_raw/microswitches"), RT_HARD_CHECK);
    rrp_pico_coms__msg__MiscSensorsB__init(&misc_sensor_msg);
    rrp_pico_coms__msg__MicroSwSensors__init(&microsw_sensor_msg);
}


// ---- Executor init ----
void exec_init()
{
    rc_executor = rclc_executor_get_zero_initialized_executor();
    const uint num_handles = 3;

    check_rc(rclc_executor_init(&rc_executor, &rc_supp.context, num_handles, &rc_alloc), RT_HARD_CHECK);
    check_rc(rclc_executor_add_subscription(&rc_executor, &e_stop_sub, &e_stop_msg, &clean_shutdown_callback, ON_NEW_DATA), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &en_camera_leds_srv, &en_camera_leds_req, &en_camera_leds_res, en_camera_leds_callback), RT_HARD_CHECK);
    check_rc(rclc_executor_add_service(&rc_executor, &run_self_test_srv, &run_self_test_req, &run_self_test_res, &run_self_test_callback), RT_HARD_CHECK);
}


// ---- Node init ----
void uros_init(const char *node_name, const char *name_space)
{
    rc_alloc = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp, 0, NULL, &rc_alloc), RT_HARD_CHECK);
    check_rc(rclc_node_init_default(&rc_node, node_name, name_space, &rc_supp), RT_HARD_CHECK);
}