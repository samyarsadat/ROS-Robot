/*
    The ROS robot project - Sensor Data MicroROS Publishers - Pico B
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
#include "helpers_lib/Helpers.h"
#include "IO_Helpers_General.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <limits>
#include "local_helpers_lib/Local_Helpers.h"



// ------- Variables ------- 

// ---- Timer execution times storage (milliseconds) ----
extern uint32_t last_microsw_publish_time, last_other_sensors_publish_time;



// ------- Functions ------- 

// ---- Microswitch data ----
bool publish_microsw_sens(struct repeating_timer *rt)
{
    // Check execution time
    uint16_t exec_time_ms = (time_us_32() / 1000) - last_microsw_publish_time;   // TODO: Log this.
    last_microsw_publish_time = time_us_32() / 1000;
    
    if (exec_time_ms > (sensors_pub_rt_interval + 10)) 
    { 
        publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_B, DIAG_WARN_MSG_TIMER_EXEC_TIME_OVER, DIAG_KV_EMPTY());
    }

    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

    microsw_sensor_msg.time.sec = timestamp_sec;
    microsw_sensor_msg.time.nanosec = timestamp_nanosec;

    microsw_sensor_msg.micro_fr_trig = gpio_get(ms_front_r);
    microsw_sensor_msg.micro_fl_trig = gpio_get(ms_front_l);
    microsw_sensor_msg.micro_br_trig = gpio_get(ms_back_r);
    microsw_sensor_msg.micro_bl_trig = gpio_get(ms_back_l);

    check_rc(rcl_publish(&microsw_sensor_pub, &microsw_sensor_msg, NULL), RT_SOFT_CHECK);
    return true;
}


// ---- Other sensor data ----
bool publish_misc_sens(struct repeating_timer *rt)
{
    // Check execution time
    uint16_t exec_time_ms = (time_us_32() / 1000) - last_other_sensors_publish_time;   // TODO: Log this.
    last_other_sensors_publish_time = time_us_32() / 1000;
    
    if (exec_time_ms > (sensors_pub_rt_interval + 10)) 
    { 
        publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_B, DIAG_WARN_MSG_TIMER_EXEC_TIME_OVER, DIAG_KV_EMPTY());
    }

    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

    misc_sensor_msg.time.sec = timestamp_sec;
    misc_sensor_msg.time.nanosec = timestamp_nanosec;

    misc_sensor_msg.speed_sel_switch = get_speed_sel_sw_pos();
    misc_sensor_msg.control_mode_switch = gpio_get(mode_sw);
    misc_sensor_msg.cpu_temp = get_rp2040_temp();
    misc_sensor_msg.battery_voltage = get_battery_voltage();
    
    misc_sensor_msg.battery_current = 0;   // TODO: SENSOR TO BE ADDED
    misc_sensor_msg.env_temp = 0;          // TODO: SENSOR TO BE ADDED
    misc_sensor_msg.env_humidity = 0;      // TODO: SENSOR TO BE ADDED

    check_rc(rcl_publish(&misc_sensor_pub, &misc_sensor_msg, NULL), RT_SOFT_CHECK);
    return true;
}