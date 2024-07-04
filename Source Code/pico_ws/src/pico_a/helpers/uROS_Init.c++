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
#include "Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "freertos_helpers_lib/uROS_Bridge_Agent.h"



// ------- Variables -------

// ---- General ----
extern uRosBridgeAgent *bridge;   // From the main file.


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
rrp_pico_coms__msg__FastOdometry enc_odom_msg;


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

// Calibration routines service
rcl_service_t run_calib_srv;
rrp_pico_coms__srv__RunCalibrationsA_Request run_calib_req;
rrp_pico_coms__srv__RunCalibrationsA_Response run_calib_res;

// Get configuration information service
rcl_service_t get_config_srv;
rrp_pico_coms__srv__GetConfigA_Request get_config_req;
rrp_pico_coms__srv__GetConfigA_Response get_config_res;



// ------- Subscriber & service callback prototypes -------
extern void cmd_vel_call(const void *msgin);
extern void en_motor_ctrl_callback(const void *req, void *res);
extern void en_emitters_callback(const void *req, void *res);
extern void en_relay_callback(const void *req, void *res);
extern void set_mtr_pid_tunings_callback(const void *req, void *res);
extern void run_self_test_callback(const void *req, void *res);
extern void run_calib_callback(const void *req, void *res);
extern void get_config_callback(const void *req, void *res);
extern void clean_shutdown();
extern void start_timers();
void clean_shutdown_callback(const void *msgin) { clean_shutdown(); }



// ------- Functions -------

// ---- Setup subscribers, publishers, and services ----
void init_subs_pubs()
{
    write_log("Initializing publishers, subscribers, and services...", LOG_LVL_INFO, FUNCNAME_ONLY);

    const rosidl_message_type_support_t *twist_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    const rosidl_message_type_support_t *empty_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    const rosidl_message_type_support_t *odom_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, FastOdometry);
    const rosidl_message_type_support_t *mtr_ctrl_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MotorCtrlState);
    const rosidl_message_type_support_t *misc_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, MiscSensorsA);
    const rosidl_message_type_support_t *ultrasonics_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, UltrasonicSensors);
    const rosidl_message_type_support_t *falloff_sensors_type = ROSIDL_GET_MSG_TYPE_SUPPORT(rrp_pico_coms, msg, FalloffSensors);
    const rosidl_service_type_support_t *set_bool_type = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
    const rosidl_service_type_support_t *set_pid_tunings_type = ROSIDL_GET_SRV_TYPE_SUPPORT(rrp_pico_coms, srv, SetPidTunings);
    const rosidl_service_type_support_t *run_self_test_type = ROSIDL_GET_SRV_TYPE_SUPPORT(diagnostic_msgs, srv, SelfTest);
    const rosidl_service_type_support_t *run_calib_type = ROSIDL_GET_SRV_TYPE_SUPPORT(rrp_pico_coms, srv, RunCalibrationsA);
    const rosidl_service_type_support_t *get_config_type = ROSIDL_GET_SRV_TYPE_SUPPORT(rrp_pico_coms, srv, GetConfigA);


    // ---- Services ----
    write_log("Initializing services...", LOG_LVL_INFO, FUNCNAME_ONLY);
    bridge->init_service(&en_motor_ctrl_srv, set_bool_type, "enable_disable/motor_ctrl");
    bridge->init_service(&en_emitters_srv, set_bool_type, "enable_disable/emitters");
    bridge->init_service(&en_relay_srv, set_bool_type, "enable_disable/pico_a_relay");
    bridge->init_service(&set_mtr_pid_tunings_srv, set_pid_tunings_type, "config/set_motor_pid_tunings");
    bridge->init_service(&run_self_test_srv, run_self_test_type, "self_test/pico_a");
    bridge->init_service(&run_calib_srv, run_calib_type, "calibrate/pico_a");
    bridge->init_service(&get_config_srv, get_config_type, "config/get_pico_a");


    // ---- Subscribers ----
    write_log("Initializing subscribers...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Command velocity topic
    bridge->init_subscriber(&cmd_vel_sub, twist_type, "/cmd_vel");
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    // E-stop topic
    bridge->init_subscriber(&e_stop_sub, empty_type, "/e_stop");
    std_msgs__msg__Empty__init(&e_stop_msg);


    // ---- Publishers ----
    write_log("Initializing publishers...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Diagnostics
    bridge->init_publisher(&diagnostics_pub, diag_status_type, "/diagnostics");
    diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg);

    // Sensor state topics
    bridge->init_publisher(&misc_sensor_pub, misc_sensors_type, "sensors_raw/misc_a");
    bridge->init_publisher(&ultrasonic_sensor_pub, ultrasonics_sensors_type, "sensors_raw/ultrasonics");
    bridge->init_publisher(&falloff_sensor_pub, falloff_sensors_type, "sensors_raw/falloff");
    rrp_pico_coms__msg__MiscSensorsA__init(&misc_sensor_msg);
    rrp_pico_coms__msg__UltrasonicSensors__init(&ultrasonic_sensor_msg);
    rrp_pico_coms__msg__FalloffSensors__init(&falloff_sensor_msg);

    // Motor controller state topics
    bridge->init_publisher(&mtr_ctrl_r_state_pub, mtr_ctrl_state_type, "sensors_raw/mtr_ctrl_right");
    bridge->init_publisher(&mtr_ctrl_l_state_pub, mtr_ctrl_state_type, "sensors_raw/mtr_ctrl_left");
    rrp_pico_coms__msg__MotorCtrlState__init(&mtr_ctrl_r_state_msg);
    rrp_pico_coms__msg__MotorCtrlState__init(&mtr_ctrl_l_state_msg);

    // Odometry topic
    bridge->init_publisher(&enc_odom_pub, odom_type, "sensors/enc_odom");
    rrp_pico_coms__msg__FastOdometry__init(&enc_odom_msg);


    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}


// ---- Executor init ----
void exec_init()
{
    write_log("Initializing MicroROS executor...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge->uros_init_executor();
    bridge->add_subscriber(&cmd_vel_sub, &cmd_vel_msg, &cmd_vel_call);
    bridge->add_subscriber(&e_stop_sub, &e_stop_msg, &clean_shutdown_callback);
    bridge->add_service(&en_motor_ctrl_srv, &en_motor_ctrl_req, &en_motor_ctrl_res, &en_motor_ctrl_callback);
    bridge->add_service(&set_mtr_pid_tunings_srv, &set_mtr_pid_tunings_req, &set_mtr_pid_tunings_res, &set_mtr_pid_tunings_callback);
    bridge->add_service(&run_self_test_srv, &run_self_test_req, &run_self_test_res, &run_self_test_callback);
    bridge->add_service(&en_emitters_srv, &en_emitters_req, &en_emitters_res, &en_emitters_callback);
    bridge->add_service(&en_relay_srv, &en_relay_req, &en_relay_res, &en_relay_callback);
    bridge->add_service(&run_calib_srv, &run_calib_req, &run_calib_res, &run_calib_callback);
    bridge->add_service(&get_config_srv, &get_config_req, &get_config_res, &get_config_callback);

    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}


// ---- Bridge init function ----
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