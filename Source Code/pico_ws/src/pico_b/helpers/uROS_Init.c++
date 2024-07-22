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
#include "Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "uros_freertos_helpers_lib/uROS_Bridge_Agent.h"



// ------- Variables -------

// ---- General ----
extern uRosBridgeAgent *bridge;   // From the main file.


// ---- Subscribers ----

// Misc.
rcl_subscription_t e_stop_sub;
std_msgs__msg__Empty e_stop_msg;


// ---- Publishers ----

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
extern void start_timers();
void clean_shutdown_callback(const void *msgin) { clean_shutdown(); }



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs()
{
    write_log("Initializing publishers, subscribers, and services...", LOG_LVL_INFO, FUNCNAME_ONLY);

    const rosidl_message_type_support_t *empty_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    const rosidl_message_type_support_t *misc_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MiscSensorsB);
    const rosidl_message_type_support_t *microsw_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MicroSwSensors);
    const rosidl_service_type_support_t *set_camera_leds_type = ROSIDL_GET_SRV_TYPE_SUPPORT(rrp_pico_coms, srv, SetCameraLeds);
    const rosidl_service_type_support_t *run_self_test_type = ROSIDL_GET_SRV_TYPE_SUPPORT(diagnostic_msgs, srv, SelfTest);

    // ---- Services ----
    write_log("Initializing services...", LOG_LVL_INFO, FUNCNAME_ONLY);
    bridge->init_service(&en_camera_leds_srv, set_camera_leds_type, "enable_disable/camera_leds");
    bridge->init_service(&run_self_test_srv, run_self_test_type, "self_test/pico_b");


    // ---- Subscribers ----
    write_log("Initializing subscribers...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // E-stop topic
    bridge->init_subscriber(&e_stop_sub, empty_type, "/e_stop");
    std_msgs__msg__Empty__init(&e_stop_msg);


    // ---- Publishers ----
    write_log("Initializing publishers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    
    // Diagnostics
    diag_uros_init();

    // Sensor state topics
    bridge->init_publisher(&misc_sensor_pub, misc_sensors_type, "sensors_raw/misc_b");
    bridge->init_publisher(&microsw_sensor_pub, microsw_sensors_type, "sensors_raw/microswitches");
    rrp_pico_coms__msg__MiscSensorsB__init(&misc_sensor_msg);
    rrp_pico_coms__msg__MicroSwSensors__init(&microsw_sensor_msg);


    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}


// ---- Executor init ----
void exec_init()
{
    write_log("Initializing MicroROS executor...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge->uros_init_executor();
    bridge->add_service(&en_camera_leds_srv, &en_camera_leds_req, &en_camera_leds_res, en_camera_leds_callback);
    bridge->add_service(&run_self_test_srv, &run_self_test_req, &run_self_test_res, run_self_test_callback);
    bridge->add_subscriber(&e_stop_sub, &e_stop_msg, clean_shutdown_callback);

    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}


// ---- Node init ----
bool uros_init()
{
    write_log("MicroROS init...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge->uros_init_node(UROS_NODE_NAME, UROS_NODE_NAMESPACE);
    init_subs_pubs();
    exec_init();
    start_timers();
 
    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
    return true;   
}