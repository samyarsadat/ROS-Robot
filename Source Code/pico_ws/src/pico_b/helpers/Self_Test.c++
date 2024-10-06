/*
    The ROS robot project - Self Test Module - Pico B
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
#include "helpers_lib/Helpers.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "Definitions.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/srv/self_test.h>
#include "FreeRTOS.h"
#include "IO_Helpers_General.h"
#include "semphr.h"



// ------- Global variables -------
std::vector<int> self_test_diag_data_slot_nums;
std::vector<diagnostic_msgs__msg__DiagnosticStatus> self_test_diag_status_reports;
SemaphoreHandle_t run_self_test_mutex = NULL;



// ------- Functions -------

// ---- Run self-test functions service callback ----
void run_self_test_callback(const void *req, void *res)
{
    // Tests the:
    // - Camera LEDs
    // - Microswitches

    diagnostic_msgs__srv__SelfTest_Request *req_in = (diagnostic_msgs__srv__SelfTest_Request *) req;
    diagnostic_msgs__srv__SelfTest_Response *res_in = (diagnostic_msgs__srv__SelfTest_Response *) res;
    res_in->passed = false;

    write_log("Received self-test request.", LOG_LVL_INFO, FUNCNAME_ONLY);
    disable_diag_pub();
    

    // Perform camera LED and microswitch self-test
    write_log("Performing camera LED and microswitch self-test...", LOG_LVL_INFO, FUNCNAME_ONLY);
    uint32_t start_time = time_us_32();
    bool triggered_microsws[4] = {false};   // [front_right, front_left, back_right, back_left]
    bool microsws_passed = false;
    
    taskENTER_CRITICAL();
    set_camera_leds(0, 0, 0, 0);

    while ((time_us_32() - start_time) < (microsw_led_test_timeout_ms * 1000))
    {
        if (!gpio_get(ms_front_r) && !triggered_microsws[0])
        {
            triggered_microsws[0] = true;
            gpio_put_pwm(cam_led_1, 65535);
        }

        if (!gpio_get(ms_front_l) && !triggered_microsws[1])
        {
            triggered_microsws[1] = true;
            gpio_put_pwm(cam_led_2, 65535);
        }

        if (!gpio_get(ms_back_r) && !triggered_microsws[2])
        {
            triggered_microsws[2] = true;
            gpio_put_pwm(cam_led_3, 65535);
        }

        if (!gpio_get(ms_back_l) && !triggered_microsws[3])
        {
            triggered_microsws[3] = true;
            gpio_put_pwm(cam_led_4, 65535);
        }

        if (triggered_microsws[0] && triggered_microsws[1] && triggered_microsws[2] && triggered_microsws[3])
        {
            taskEXIT_CRITICAL();
            res_in->passed = true;
            microsws_passed = true;
            
            diag_publish_item_t pub_item = prepare_diag_publish_item(DIAG_LVL_OK, DIAG_NAME_SYSTEM, DIAG_ID_SYS_GENERAL, DIAG_OK_MSG_MICROSW_LED_TEST_PASS, NULL);
            self_test_diag_status_reports.push_back(*pub_item.diag_msg);
            self_test_diag_data_slot_nums.push_back(pub_item.allocated_slot);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
    }

    set_camera_leds(0, 0, 0, 0);

    if (!microsws_passed)
    {
        taskEXIT_CRITICAL();

        std::vector<diag_kv_pair_item_t> kv_pairs;
        std::string pass_states = bool_array_as_str(triggered_microsws, 4);
        kv_pairs.push_back(diag_kv_pair_item_t{"sensor_pass_states", pass_states});

        diag_publish_item_t pub_item = prepare_diag_publish_item(DIAG_LVL_ERROR, DIAG_NAME_SYSTEM, DIAG_ID_SYS_GENERAL, DIAG_ERR_MSG_MICROSW_LED_TEST_FAIL, &kv_pairs);
        self_test_diag_status_reports.push_back(*pub_item.diag_msg);
        self_test_diag_data_slot_nums.push_back(pub_item.allocated_slot);
    }


    res_in->id.data = (char *) "Pico B";
    res_in->id.size = strlen(res_in->id.data);
    res_in->status.data = self_test_diag_status_reports.data();
    res_in->status.size = self_test_diag_status_reports.size();

    write_log("Self-test completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}