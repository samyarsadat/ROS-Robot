/*
    The ROS robot project - Local Helper/commonly used functions
    These are specific MicroROS/IO/etc. functions/definitions that are not 
    designed to be used in any other programs.
    They are program-specific.
    
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
#include "local_helpers_lib/Local_Helpers.h"
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <string>
#include <rmw_microros/rmw_microros.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include "pico/stdio_uart.h"
#include "pico/stdio/driver.h"
#include "pico/stdio.h"



// ------- Functions ------- 

// ----- RETURN CHECKERS -----
// Note: clean_shutdown() must be defined elsewhere!
// Note: diagnostics_msg (type: diagnostic_msgs__msg__DiagnosticStatus) must be defined eslewhere!
// Note: diagnostics_pub (type: rcl_publisher_t) must be defined eslewhere!

extern diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;
extern rcl_publisher_t diagnostics_pub;
extern void clean_shutdown();
void publish_diag_report(uint8_t level, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values);
void write_log(std::string src, std::string msg, LOG_LEVEL lvl);


// --- RCL return checker ---
bool check_rc(rcl_ret_t rctc, RT_CHECK_MODE mode)
{
    if (rctc != RCL_RET_OK)
    {
        char buffer[60];

        switch (mode)
        {
            case RT_HARD_CHECK:
                sprintf(buffer, "RCL Return check failed: [code: %d, RT_HARD_CHECK]", rctc);
                write_log("check_rc", buffer, LOG_LVL_FATAL);
                
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_UROS, DIAG_HWID_UROS, DIAG_ERR_MSG_UROS_RC_CHECK_FAIL, NULL);
                clean_shutdown();
                break;

            case RT_SOFT_CHECK:
                sprintf(buffer, "RCL Return check failed: [code: %d, RT_SOFT_CHECK]", rctc);
                write_log("check_rc", buffer, LOG_LVL_ERROR);
                
                publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UROS, DIAG_HWID_UROS, DIAG_WARN_MSG_UROS_RC_CHECK_FAIL, NULL);
                return false;
                break;
            
            case RT_LOG_ONLY_CHECK:
                sprintf(buffer, "RCL Return check failed: [code: %d, RT_LOG_ONLY_CHECK]", rctc);
                write_log("check_rc", buffer, LOG_LVL_WARN);

                return false;
                break;

            default:
                break;
        }
    }

    return true;
}


// --- Return checker, except for functions that return a boolean ---
bool check_bool(bool function, RT_CHECK_MODE mode)
{
    if (!function)
    {
        switch (mode)
        {
            case RT_HARD_CHECK:
                write_log("check_bool", "BOOL Return check failed: [RT_HARD_CHECK]", LOG_LVL_FATAL);
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_A, DIAG_ERR_MSG_BOOL_RT_CHECK_FAIL, NULL);
                clean_shutdown();
                break;

            case RT_SOFT_CHECK:
                write_log("check_bool", "BOOL Return check failed: [RT_SOFT_CHECK]", LOG_LVL_ERROR);
                publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_A, DIAG_WARN_MSG_BOOL_RT_CHECK_FAIL, NULL);
                break;
            
            case RT_LOG_ONLY_CHECK:
                write_log("check_bool", "BOOL Return check failed: [RT_LOG_ONLY_CHECK]", LOG_LVL_WARN);
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

    // TODO: LOGGING
    /*char buffer[60];
    sprintf(buffer, "Publishing diagnostics report: [hwname: %d, hwid: %s, lvl: %u]", hw_name);
    write_log("publish_diag_report", buffer, LOG_LVL_FATAL);*/
    
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


// ---- Logging functions ----
void write_log(std::string src, std::string msg, LOG_LEVEL lvl)
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

    std::string level;
    if (lvl == LOG_LVL_INFO) { level = "INFO"; }
    else if (lvl == LOG_LVL_WARN) { level = "WARNING"; }
    else if (lvl == LOG_LVL_ERROR) { level = "ERROR"; }
    else if (lvl == LOG_LVL_FATAL) { level = "FATAL"; }

    // This is quite ugly, but it works.
    std::string log_msg = "[" + std::to_string(timestamp_sec) + "." + std::to_string(timestamp_nanosec) + "] [" + level + "] [" + src + "]: " + msg + "\r\n";
    stdio_uart.out_chars(log_msg.c_str(), log_msg.length());
}


// ---- Pings the MicroROS agent ----
bool ping_agent()
{
    bool success = (rmw_uros_ping_agent(uros_agent_find_timeout_ms, uros_agent_find_attempts) == RMW_RET_OK);
    return success;
}