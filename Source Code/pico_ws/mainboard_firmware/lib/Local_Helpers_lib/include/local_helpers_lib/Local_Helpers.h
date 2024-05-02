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

#pragma once


// ------- Libraries & Modules -------
#include "pico/stdlib.h"
#include "local_helpers_lib/Diagnostics_Definitons.h"
#include "local_helpers_lib/Common_Definitions.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <rcl/rcl.h>
#include <string>



// ------- Enums -------
enum RT_CHECK_MODE {RT_HARD_CHECK, RT_SOFT_CHECK, RT_LOG_ONLY_CHECK};
enum LOG_LEVEL {LOG_LVL_INFO, LOG_LVL_WARN, LOG_LVL_ERROR, LOG_LVL_FATAL};



// ------- Functions ------- 

// ----- RETURN CHECKERS -----
// Note: clean_shutdown() must be defined elsewhere!
// Note: diagnostics_msg (type: diagnostic_msgs__msg__DiagnosticStatus) must be defined eslewhere!

// --- RCL return checker ---
bool check_rc(rcl_ret_t rctc, RT_CHECK_MODE mode);

// --- Return checker, except for functions that return a boolean ---
bool check_bool(bool function, RT_CHECK_MODE mode);

// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, std::string hw_name, std::string hw_id, std::string msg, diagnostic_msgs__msg__KeyValue *key_values);

// ---- Logging functions ----
void write_log(std::string src, std::string msg, LOG_LEVEL lvl);

// ---- Pings the MicroROS agent ----
bool ping_agent();

// ---- Execution interval checker ----
// ---- Checks the amount of time passed since the last time it was called (with the specific time storage varialble provided) ----
// ---- Returns false if the execution time has exceeded the specified limit ----
bool check_exec_interval(uint32_t &last_call_time_ms, uint16_t max_exec_time_ms, std::string log_msg);