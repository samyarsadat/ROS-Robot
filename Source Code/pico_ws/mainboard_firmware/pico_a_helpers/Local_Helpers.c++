/*
    The ROS robot project - Pico A local helper functions
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
#include "Local_Helpers.h"
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include "Definitions.h"
#include "pico/multicore.h"
#include "uROS_Init.h"



// ------- Functions ------- 

// ----- RETURN CHECKERS -----
// Note: clean_shutdown() must be defines elsewhere!
// Note: publish_diag_report() must be defines elsewhere!

extern void clean_shutdown();
void publish_diag_report(uint8_t level, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values);

// --- RCL return checker ---
bool check_rc(rcl_ret_t rctc, uint mode)
{
    if (rctc != RCL_RET_OK)
    {
        switch (mode)
        {
            case RT_HARD_CHECK:
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_UROS, DIAG_HWID_UROS, DIAG_ERR_MSG_UROS_RC_CHECK_FAIL, NULL);
                clean_shutdown();
                break;

            case RT_SOFT_CHECK:
                publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UROS, DIAG_HWID_UROS, DIAG_WARN_MSG_UROS_RC_CHECK_FAIL, NULL);
                return false;
                break;
            
            case RT_LOG_ONLY_CHECK:
                // TODO: LOGGING
                return false;
                break;

            default:
                break;
        }
    }

    return true;
}


// --- Return checker, except for functions that return a boolean ---
bool check_bool(bool function, uint mode)
{
    if (!function)
    {
        switch (mode)
        {
            case RT_HARD_CHECK:
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_A, DIAG_ERR_MSG_BOOL_RT_CHECK_FAIL, NULL);
                clean_shutdown();
                break;

            case RT_SOFT_CHECK:
                publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_A, DIAG_WARN_MSG_BOOL_RT_CHECK_FAIL, NULL);
                break;
            
            case RT_LOG_ONLY_CHECK:
                // TODO: LOGGING
                break;

            default:
                break;
        }
    }

    return function;
}


// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values)
{
    diagnostics_msg.name.data = hw_name;
    diagnostics_msg.name.size = strlen(diagnostics_msg.name.data);
    diagnostics_msg.hardware_id.data = hw_id;
    diagnostics_msg.hardware_id.size = strlen(diagnostics_msg.hardware_id.data);
    diagnostics_msg.message.data = msg;
    diagnostics_msg.message.size = strlen(diagnostics_msg.message.data);
    diagnostics_msg.values.data = NULL;
    diagnostics_msg.values.size = 0;
    diagnostics_msg.level = level;
    
    if (key_values != NULL)
    {
        diagnostics_msg.values.data = key_values;
        diagnostics_msg.values.size = sizeof(key_values) / sizeof(key_values[0]);
    }

    /* 
        The return value of rcl_publish() is intentionally not checked here to prevent infinite recursion of publish_diag_report().
        This edge case may occur if MicroROS is not initialized properly when check_rc() is called 
        (this could happen if check_rc() fails for rclc_node_init_default(), for example).
        In this case, this publish function would also not return RCL_RET_OK, resulting in its
        check_rc() calling publish_diag_report() and then the same thing happening over and over again.
    */
    rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
}