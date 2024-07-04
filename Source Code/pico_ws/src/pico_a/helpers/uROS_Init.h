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
#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/empty.h>
#include <geometry_msgs/msg/twist.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_srvs/srv/set_bool.h>
#include <rrp_pico_coms/msg/misc_sensors_a.h>
#include <rrp_pico_coms/msg/ultrasonic_sensors.h>
#include <rrp_pico_coms/msg/falloff_sensors.h>
#include <rrp_pico_coms/msg/motor_ctrl_state.h>
#include <rrp_pico_coms/msg/fast_odometry.h>
#include <rrp_pico_coms/srv/set_pid_tunings.h>
#include <rrp_pico_coms/srv/run_calibrations_a.h>
#include <rrp_pico_coms/srv/get_config_a.h>
#include <diagnostic_msgs/srv/self_test.h>



// ------- Variables -------

// ---- Subscribers ----

// Misc.
extern rcl_subscription_t e_stop_sub;
extern std_msgs__msg__Empty e_stop_msg;

// Command & Control
extern rcl_subscription_t cmd_vel_sub;
extern geometry_msgs__msg__Twist cmd_vel_msg;


// ---- Publishers ----

// Diagnostics
extern rcl_publisher_t diagnostics_pub;
extern diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;

// Sensor States
extern rcl_publisher_t misc_sensor_pub, ultrasonic_sensor_pub, falloff_sensor_pub;
extern rrp_pico_coms__msg__MiscSensorsA misc_sensor_msg;
extern rrp_pico_coms__msg__UltrasonicSensors ultrasonic_sensor_msg;
extern rrp_pico_coms__msg__FalloffSensors falloff_sensor_msg;

// Motor Controller States
extern rcl_publisher_t mtr_ctrl_r_state_pub, mtr_ctrl_l_state_pub;
extern rrp_pico_coms__msg__MotorCtrlState mtr_ctrl_r_state_msg, mtr_ctrl_l_state_msg;

// Odometry
extern rcl_publisher_t enc_odom_pub;
extern rrp_pico_coms__msg__FastOdometry enc_odom_msg;


// ---- Services ----

// Motor Controller Enable/Disable
extern rcl_service_t en_motor_ctrl_srv;
extern std_srvs__srv__SetBool_Request en_motor_ctrl_req;
extern std_srvs__srv__SetBool_Response en_motor_ctrl_res;

// Emitters (Ultrasonic, IR Edge) Enable/Disable 
extern rcl_service_t en_emitters_srv;
extern std_srvs__srv__SetBool_Request en_emitters_req;
extern std_srvs__srv__SetBool_Response en_emitters_res;

// Power Relay Enable/Disable  
extern rcl_service_t en_relay_srv;
extern std_srvs__srv__SetBool_Request en_relay_req;
extern std_srvs__srv__SetBool_Response en_relay_res;

// Motor Controller PID Tunings
extern rcl_service_t set_mtr_pid_tunings_srv;
extern rrp_pico_coms__srv__SetPidTunings_Request set_mtr_pid_tunings_req;
extern rrp_pico_coms__srv__SetPidTunings_Response set_mtr_pid_tunings_res;

// Initiate the self-test function
extern rcl_service_t run_self_test_srv;
extern diagnostic_msgs__srv__SelfTest_Request run_self_test_req;
extern diagnostic_msgs__srv__SelfTest_Response run_self_test_res;

// Calibration routines service
extern rcl_service_t run_calib_srv;
extern rrp_pico_coms__srv__RunCalibrationsA_Request run_calib_req;
extern rrp_pico_coms__srv__RunCalibrationsA_Response run_calib_res;

// Get configuration information service
extern rcl_service_t get_config_srv;
extern rrp_pico_coms__srv__GetConfigA_Request get_config_req;
extern rrp_pico_coms__srv__GetConfigA_Response get_config_res;



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs();

// ---- Executor init ----
void exec_init();

// ---- Bridge init function ----
bool uros_init();