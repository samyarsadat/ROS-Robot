/*
    The ROS robot project - Self Test Module - Pico A
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
#include "helpers_lib/Helpers.h"
#include "uros_freertos_helpers_lib/uROS_Publishing_Handler.h"
#include "IO_Helpers_Mux.h"
#include "IO_Helpers_Edge.h"
#include "IO_Helpers_Ultrasonic.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "Definitions.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/srv/self_test.h>
#include "motor_control_lib/Motor.h"
#include "motor_control_lib/Motor_Safety.h"
#include <functional>
#include "FreeRTOS.h"
#include "semphr.h"
#include "haw/MPU6050.h"



// ------- Global variables -------
std::vector<int> self_test_diag_data_slot_nums;
std::vector<diagnostic_msgs__msg__DiagnosticStatus> self_test_diag_status_reports;
SemaphoreHandle_t run_self_test_mutex = NULL;
extern mpu6050_t mpu6050;



// ------- Functions -------

// ---- Run self-test functions service callback ----
void run_self_test_callback(const void *req, void *res)
{
    // Tests the:
    // - IR edge sensors
    // - Ultrasonic sensors
    // - IMU

    diagnostic_msgs__srv__SelfTest_Request *req_in = (diagnostic_msgs__srv__SelfTest_Request *) req;
    diagnostic_msgs__srv__SelfTest_Response *res_in = (diagnostic_msgs__srv__SelfTest_Response *) res;
    uint8_t loop_index = 0;

    write_log("Received self-test request.", LOG_LVL_INFO, FUNCNAME_ONLY);
    res_in->passed = true;
    disable_diag_pub();

    
    // Perform IR edge sensor self-test
    write_log("Performing IR edge sensor self-test...", LOG_LVL_INFO, FUNCNAME_ONLY);
    std::vector<bool> ir_test_results = ir_self_test();
    loop_index = 0;

    for (auto result : ir_test_results)
    {
        char buffer[60];
        char HW_ID[30];

        if (loop_index > 4) { strncpy(HW_ID, DIAG_ID_IR_EDGE_B, sizeof(HW_ID)); }
        else { strncpy(HW_ID, DIAG_ID_IR_EDGE_F, sizeof(HW_ID)); }

        diag_publish_item_t pub_item;
        std::vector<diag_kv_pair_item_t> kv_pairs;
        std::string loop_index_str = std::to_string(loop_index);
        kv_pairs.push_back(diag_kv_pair_item_t{"sensor_num", loop_index_str});

        if (result)
        {
            snprintf(buffer, sizeof(buffer), DIAG_ERR_MSG_IR_EDGE_TEST_FAIL, loop_index);
            pub_item = prepare_diag_publish_item(DIAG_LVL_ERROR, DIAG_NAME_IR_EDGE, HW_ID, buffer, &kv_pairs);
            res_in->passed = false;
        }

        else
        {
            snprintf(buffer, sizeof(buffer), DIAG_OK_MSG_IR_EDGE_TEST_PASS, loop_index);
            pub_item = prepare_diag_publish_item(DIAG_LVL_OK, DIAG_NAME_IR_EDGE, HW_ID, buffer, &kv_pairs);
        }

        if (pub_item.allocated_slot >= 0)
        {
            self_test_diag_status_reports.push_back(*pub_item.diag_msg);
            self_test_diag_data_slot_nums.push_back(pub_item.allocated_slot);
        }

        else
        {
            write_log("Failed to allocate diagnostic slot!", LOG_LVL_ERROR, FUNCNAME_LINE_ONLY);
        }
        
        loop_index ++;
    }


    // Perform ultrasonic sensor self-test
    write_log("Performing ultrasonic sensor self-test...", LOG_LVL_INFO, FUNCNAME_ONLY);
    ULTRA_TEST_RESULT_t ultra_test_results[4];   // [front, back, right, left]
    loop_index = 0;

    ultra_test_results[0] = ultra_self_test([=]() -> float { return get_ultra_dist_single(front_ultra_io, DIAG_ID_ULTRASONIC_F); });
    ultra_test_results[1] = ultra_self_test([=]() -> float { return get_ultra_dist_mux(back_ultra_trig_mux, back_ultra_echo_mux, DIAG_ID_ULTRASONIC_B); });
    ultra_test_results[2] = ultra_self_test([=]() -> float { return get_ultra_dist_mux(right_ultra_trig_mux, right_ultra_echo_mux, DIAG_ID_ULTRASONIC_R); });
    ultra_test_results[3] = ultra_self_test([=]() -> float { return get_ultra_dist_mux(left_ultra_trig_mux, left_ultra_echo_mux, DIAG_ID_ULTRASONIC_L); });

    for (int i = 0; i < 4; i++)
    {
        char buffer[60];
        char HW_ID[30];

        if (loop_index == 0) { strncpy(HW_ID, DIAG_ID_ULTRASONIC_F, sizeof(HW_ID)); }
        else if (loop_index == 1) { strncpy(HW_ID, DIAG_ID_ULTRASONIC_B, sizeof(HW_ID)); }
        else if (loop_index == 2) { strncpy(HW_ID, DIAG_ID_ULTRASONIC_R, sizeof(HW_ID)); }
        else if (loop_index == 3) { strncpy(HW_ID, DIAG_ID_ULTRASONIC_L, sizeof(HW_ID)); }

        diag_publish_item_t pub_item;
        std::vector<diag_kv_pair_item_t> kv_pairs;
        std::string measured_dist_str = std::to_string(ultra_test_results[i].measured_distance);
        std::string status_code_str = std::to_string(ultra_test_results[i].status);
        kv_pairs.push_back(diag_kv_pair_item_t{"measured_distance", measured_dist_str});
        kv_pairs.push_back(diag_kv_pair_item_t{"status_code", status_code_str});
        
        if (ultra_test_results[i].status == ULTRA_TEST_PASS)
        {
            snprintf(buffer, sizeof(buffer), DIAG_OK_MSG_ULTRA_TEST_PASS, (int) ultra_test_results[i].status);
            pub_item = prepare_diag_publish_item(DIAG_LVL_OK, DIAG_NAME_ULTRASONICS, HW_ID, buffer, &kv_pairs);
        }

        else
        {
            snprintf(buffer, sizeof(buffer), DIAG_ERR_MSG_ULTRA_TEST_FAIL, (int) ultra_test_results[i].status);
            pub_item = prepare_diag_publish_item(DIAG_LVL_ERROR, DIAG_NAME_ULTRASONICS, HW_ID, buffer, &kv_pairs);
            res_in->passed = false;
        }

        if (pub_item.allocated_slot >= 0)
        {
            self_test_diag_status_reports.push_back(*pub_item.diag_msg);
            self_test_diag_data_slot_nums.push_back(pub_item.allocated_slot);
        }

        else
        {
            write_log("Failed to allocate diagnostic slot!", LOG_LVL_ERROR, FUNCNAME_LINE_ONLY);
        }

        loop_index ++;
    }


    // Perform IMU self-test
    write_log("Performing MPU6050 sensor self-test...", LOG_LVL_INFO, FUNCNAME_ONLY);
    {
        float test_results[6] = {0, 0, 0, 0, 0, 0};
        mpu6050_self_test(&mpu6050, test_results);

        diag_publish_item_t pub_item;

        std::vector<diag_kv_pair_item_t> kv_pairs;
        std::string result_keys[6];
        std::string test_result_str[6];
        std::string result_msg;
        bool test_passed = true;

        for (int i = 0; i < 6; i++)
        {
            result_keys[i] = std::string("result_index_" + std::to_string(i));
            test_result_str[i] = std::to_string(test_results[i]);
            kv_pairs.push_back(diag_kv_pair_item_t{result_keys[i], test_result_str[i]});

            if ((test_results[i] > mpu6050_selftest_trim_dev_lim) && (test_results[i] < (mpu6050_selftest_trim_dev_lim * -1)))
            {
                result_msg = result_msg + "No. " + std::to_string(i) + " limits exceeded";
                test_passed = false;
            }

            else
            {
                result_msg = result_msg + "No. " + std::to_string(i) + " passed";
            }

            if (i < 5)
            {
                result_msg = result_msg + ", ";
            }
        }

        if (test_passed)
        {
            std::string diag_msg = DIAG_OK_MSG_MPU6050_TEST_PASSED + result_msg;
            pub_item = prepare_diag_publish_item(DIAG_LVL_OK, DIAG_NAME_ENV_SENSORS, DIAG_ID_ENV_IMU, diag_msg, &kv_pairs);
        }

        else
        {
            std::string diag_msg = DIAG_ERR_MSG_MPU6050_TEST_FAIL + result_msg;
            pub_item = prepare_diag_publish_item(DIAG_LVL_ERROR, DIAG_NAME_ENV_SENSORS, DIAG_ID_ENV_IMU, diag_msg, &kv_pairs);
            res_in->passed = false;
        }

        if (pub_item.allocated_slot >= 0)
        {
            self_test_diag_status_reports.push_back(*pub_item.diag_msg);
            self_test_diag_data_slot_nums.push_back(pub_item.allocated_slot);
        }

        else
        {
            write_log("Failed to allocate diagnostic slot!", LOG_LVL_ERROR, FUNCNAME_LINE_ONLY);
        }
    }


    res_in->id.data = (char *) "Pico A";
    res_in->id.size = strlen(res_in->id.data);
    res_in->status.data = self_test_diag_status_reports.data();
    res_in->status.size = self_test_diag_status_reports.size();

    // Omitting motor test as the Motor Safety module practically acts as a constantly running "self-test" for the motors.

    write_log("Self-test completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}