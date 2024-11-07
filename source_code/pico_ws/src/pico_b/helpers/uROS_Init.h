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

#pragma once


// ------- Libraries & Modules -------
#include <rcl/rcl.h>
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
#include <diagnostic_msgs/srv/self_test.h>



// ------- Variables -------

// ---- Subscribers ----

// Misc.
extern rcl_subscription_t e_stop_sub;
extern std_msgs__msg__Empty e_stop_msg;


// ---- Publishers ----

// Sensor States
extern rcl_publisher_t misc_sensor_pub, microsw_sensor_pub;
extern rrp_pico_coms__msg__MiscSensorsB misc_sensor_msg;
extern rrp_pico_coms__msg__MicroSwSensors microsw_sensor_msg;


// ---- Services ----

// Motor Controller Enable/Disable
extern rcl_service_t en_camera_leds_srv;
extern rrp_pico_coms__srv__SetCameraLeds_Request en_camera_leds_req;
extern rrp_pico_coms__srv__SetCameraLeds_Response en_camera_leds_res;

// Initiate the self-test function
extern rcl_service_t run_self_test_srv;
extern diagnostic_msgs__srv__SelfTest_Request run_self_test_req;
extern diagnostic_msgs__srv__SelfTest_Response run_self_test_res;



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs();

// ---- Executor init ----
void exec_init();

// ---- Bridge init function ----
bool uros_init();