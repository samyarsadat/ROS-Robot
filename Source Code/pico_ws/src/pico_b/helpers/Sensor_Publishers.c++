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
#include "Definitions.h"
#include "uROS_Init.h"
#include "helpers_lib/Helpers.h"
#include "IO_Helpers_General.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "freertos_helpers_lib/uROS_Publishing_Handler.h"



// ------- Global variables ------- 

// ---- Publishing handler instance ----
extern uRosPublishingHandler *pub_handler;

// ---- Timer execution times storage (milliseconds) ----
extern uint32_t last_microsw_publish_time, last_other_sensors_publish_time;



// ------- Functions ------- 

// ---- Microswitch data ----
void publish_microsw_sens(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_microsw_publish_time, (microsw_pub_rt_interval + 50), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        microsw_sensor_msg.time.sec = timestamp_sec;
        microsw_sensor_msg.time.nanosec = timestamp_nanosec;

        microsw_sensor_msg.micro_fr_trig = !gpio_get(ms_front_r);
        microsw_sensor_msg.micro_fl_trig = !gpio_get(ms_front_l);
        microsw_sensor_msg.micro_br_trig = !gpio_get(ms_back_r);
        microsw_sensor_msg.micro_bl_trig = !gpio_get(ms_back_l);

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &microsw_sensor_pub;
        pub_item.message = &microsw_sensor_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}


// ---- Other sensor data ----
void publish_misc_sens(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_other_sensors_publish_time, (sensors_pub_rt_interval + 50), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        misc_sensor_msg.time.sec = timestamp_sec;
        misc_sensor_msg.time.nanosec = timestamp_nanosec;

        misc_sensor_msg.speed_sel_switch = get_speed_sel_sw_pos();
        misc_sensor_msg.control_mode_switch = gpio_get(mode_sw);
        misc_sensor_msg.battery_voltage = get_battery_voltage();
        misc_sensor_msg.cpu_temp = -1.0f;
        
        // The get_rp2040_temp function uses the ADC, so we must take the mutex.
        if (check_bool(adc_take_mutex(), RT_SOFT_CHECK))
        {
            misc_sensor_msg.cpu_temp = get_rp2040_temp();
            adc_release_mutex();
        }
        
        misc_sensor_msg.battery_current = 0;   // TODO: SENSOR TO BE ADDED
        misc_sensor_msg.env_temp = 0;          // TODO: SENSOR TO BE ADDED
        misc_sensor_msg.env_humidity = 0;      // TODO: SENSOR TO BE ADDED

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &misc_sensor_pub;
        pub_item.message = &misc_sensor_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}