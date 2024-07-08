/*
    The ROS robot project - Sensor Data MicroROS Publishers - Pico A
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
#include "IO_Helpers_Ultrasonic.h"
#include "IO_Helpers_Edge.h"
#include "haw/MPU6050.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include "freertos_helpers_lib/uROS_Publishing_Handler.h"
#include "Definitions.h"
#include "timers.h"



// ------- Variables ------- 

// ---- Publishing handler instance ----
extern uRosPublishingHandler *pub_handler;

// ---- MPU6050 object ----
extern mpu6050_t mpu6050;

// ---- Timer execution times storage (milliseconds) ----
extern uint32_t last_ultrasonic_publish_time, last_edge_ir_publish_time, last_other_sensors_publish_time;



// ------- Functions ------- 

// ---- Publish ultrasonic sensor data ----
void publish_ultra(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_ultrasonic_publish_time, (ultra_pub_rt_interval + 50), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        ultrasonic_sensor_msg.time.sec = timestamp_sec;
        ultrasonic_sensor_msg.time.nanosec = timestamp_nanosec;

        ultrasonic_sensor_msg.ultrasonic_f_reading = get_ultra_dist_single(front_ultra_io, DIAG_ID_ULTRASONIC_F);
        ultrasonic_sensor_msg.ultrasonic_b_reading = get_ultra_dist_mux(back_ultra_trig_mux, back_ultra_echo_mux, DIAG_ID_ULTRASONIC_B);
        ultrasonic_sensor_msg.ultrasonic_r_reading = get_ultra_dist_mux(right_ultra_trig_mux, right_ultra_echo_mux, DIAG_ID_ULTRASONIC_R);
        //ultrasonic_sensor_msg.ultrasonic_l_reading = get_ultra_dist_mux(left_ultra_trig_mux, left_ultra_echo_mux, DIAG_ID_ULTRASONIC_L);

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &ultrasonic_sensor_pub;
        pub_item.message = &ultrasonic_sensor_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}


// ---- Publish edge sensor data ----
void publish_edge_ir(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_edge_ir_publish_time, (edge_ir_pub_rt_interval + 15), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;
        
        bool* readings = get_ir_status();

        if (check_bool((readings != NULL), RT_SOFT_CHECK))
        {
            falloff_sensor_msg.time.sec = timestamp_sec;
            falloff_sensor_msg.time.nanosec = timestamp_nanosec;

            // FRONT SENSORS
            falloff_sensor_msg.ir_edge_sens_back_trig[0] = readings[0];
            falloff_sensor_msg.ir_edge_sens_back_trig[1] = readings[1];
            falloff_sensor_msg.ir_edge_sens_back_trig[2] = readings[2];
            falloff_sensor_msg.ir_edge_sens_back_trig[3] = readings[3];

            // BACK SENSORS
            falloff_sensor_msg.ir_edge_sens_front_trig[0] = readings[4];
            falloff_sensor_msg.ir_edge_sens_front_trig[1] = readings[5];
            falloff_sensor_msg.ir_edge_sens_front_trig[2] = readings[6];
            falloff_sensor_msg.ir_edge_sens_front_trig[3] = false;
            // FIXME: This sensor is currently broken. This will be fixed.
            //falloff_sensor_msg.ir_edge_sens_front_trig[3] = readings[7];

            uRosPublishingHandler::PublishItem_t pub_item;
            pub_item.publisher = &falloff_sensor_pub;
            pub_item.message = &falloff_sensor_msg;
            xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
        }
    }
}


// ---- Other sensor data ----
void publish_misc_sens(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_other_sensors_publish_time, (sensors_pub_rt_interval + 15), "Publish interval exceeded limits!", true);
        
        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        misc_sensor_msg.time.sec = timestamp_sec;
        misc_sensor_msg.time.nanosec = timestamp_nanosec;
        
        // The get_rp2040_temp function uses the ADC, so we must take the mutex.
        check_bool(adc_take_mutex(), RT_HARD_CHECK);
        misc_sensor_msg.cpu_temp = get_rp2040_temp();
        adc_release_mutex();

        // MPU6050 IMU
        // FIXME: IMU is currently broken.
        //if (check_bool(mpu6050_event(&mpu6050) == 1, RT_SOFT_CHECK))
        if (false)
        {
            mpu6050_vectorf_t *mpu_accel = mpu6050_get_accelerometer(&mpu6050);
            mpu6050_vectorf_t *mpu_gyro = mpu6050_get_gyroscope(&mpu6050);
            mpu6050_activity_t *interrupts = mpu6050_read_activities(&mpu6050);
            
            misc_sensor_msg.imu_accel.x = mpu_accel->x;
            misc_sensor_msg.imu_accel.y = mpu_accel->y;
            misc_sensor_msg.imu_accel.z = mpu_accel->z;
            misc_sensor_msg.imu_gyro.x = mpu_gyro->x;
            misc_sensor_msg.imu_gyro.y = mpu_gyro->y;
            misc_sensor_msg.imu_gyro.z = mpu_gyro->z;

            misc_sensor_msg.imu_freefall_int = (interrupts->isFreefall == 1);
            misc_sensor_msg.imu_temp = mpu6050_get_temperature_c(&mpu6050);
        }

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &misc_sensor_pub;
        pub_item.message = &misc_sensor_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}