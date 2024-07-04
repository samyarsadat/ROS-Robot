/*
    The ROS robot project - Motor safety handler module - Pico A
    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
 
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
#include "motor_control_lib/Motor_Safety.h"
#include "local_helpers_lib/Local_Helpers.h"



// ------- Definitions -------
#define SAFETY_QUEUE_SIZE 10

struct MotorSafetyQueueItem
{
    MotorSafety::safety_trigger_conditions condition;
    uint8_t id;
};

typedef struct MotorSafetyQueueItem MotorSafetyQueueItem_t;



// ------- Global variables -------
QueueHandle_t motor_safety_queue = NULL;
extern bool self_test_mode;
extern Motor r_motors, l_motors;
extern void clean_shutdown();



// ------- Functions -------

// ---- Initialize motor safety queue ----
void init_motor_safety_queue()
{
    motor_safety_queue = xQueueCreate(SAFETY_QUEUE_SIZE, sizeof(MotorSafetyQueueItem_t));
    
    if (motor_safety_queue == NULL)
    {
        write_log("Failed to create motor safety queue!", LOG_LVL_FATAL, FUNCNAME_LINE_ONLY);
        clean_shutdown();
        return;
    }
}


// ---- Motor safety ISR callback ----
void motor_safety_isr_callback(MotorSafety::safety_trigger_conditions condition, uint8_t id)
{
    MotorSafetyQueueItem_t item;
    item.condition = condition;
    item.id = id;

    BaseType_t high_prio_woken = pdFALSE;
    check_bool((xQueueSendToBackFromISR(motor_safety_queue, (void *) &item, &high_prio_woken) == pdPASS), RT_HARD_CHECK);
    portYIELD_FROM_ISR(high_prio_woken);
}


// ---- MotorSafety trigger handler ----
void motor_safety_handler_task(void *parameters)
{
    MotorSafetyQueueItem_t queue_item;
    char buffer[60];

    while (true)
    {
        // Blocks indefinitely until an item is available.
        if (xQueueReceive(motor_safety_queue, &queue_item, portMAX_DELAY) == pdTRUE)
        {
            snprintf(buffer, 60, "Motor Safety triggered: [code: %d, id: %d]", static_cast<int>(queue_item.condition), queue_item.id);
            write_log(buffer, LOG_LVL_INFO, FUNCNAME_LINE_ONLY);

            if (!self_test_mode)
            {
                // FIXME: This causes the motors to jitter back and forth, a cooldown period must be added!
                /*if (queue_item.condition == MotorSafety::safety_trigger_conditions::SET_VS_ACTUAL_DIR_DIFF)
                {
                    if (queue_item.id == right_motor_controller_id)
                    {
                        write_log("SET_VS_ACTUAL_DIR_DIFF triggered. Reversing control direction. [Right]", LOG_LVL_WARN, FUNCNAME_LINE_ONLY);
                        r_motors.set_direction_reversed(!r_motors.get_dir_reversed());
                    }

                    else
                    {
                        write_log("SET_VS_ACTUAL_DIR_DIFF triggered. Reversing control direction. [Left]", LOG_LVL_WARN, FUNCNAME_LINE_ONLY);
                        l_motors.set_direction_reversed(!l_motors.get_dir_reversed());
                    }
                }

                else*/
                {
                    if (queue_item.id == right_motor_controller_id)
                    {
                        write_log("Critical Motor Safety condition triggered! [Right]", LOG_LVL_FATAL, FUNCNAME_LINE_ONLY);
                        publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_DRV_R, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                        publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_ENC_R1, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                        publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_ENC_R2, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                    }
                    
                    else
                    {
                        write_log("Critical Motor Safety condition triggered! [Left]", LOG_LVL_FATAL, FUNCNAME_LINE_ONLY);
                        publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_DRV_L, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                        publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_ENC_L1, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                        publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_ENC_L2, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                    }
                }
            }
        }
    }
}