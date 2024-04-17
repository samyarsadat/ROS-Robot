/*
    The ROS robot project - Pico B local helper functions
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
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include "Definitions.h"
#include "pico/multicore.h"
#include "uROS_Init.h"
#include <string>



// ------- Enums -------
enum RT_CHECK_MODE {RT_HARD_CHECK, RT_SOFT_CHECK, RT_LOG_ONLY_CHECK};
enum LOG_LEVEL {LOG_LVL_INFO, LOG_LVL_WARN, LOG_LVL_ERROR, LOG_LVL_FATAL};



// ------- Functions ------- 

// ----- RETURN CHECKERS -----
// Note: clean_shutdown() must be defines elsewhere!
// Note: publish_diag_report() must be defines elsewhere!

// --- RCL return checker ---
bool check_rc(rcl_ret_t rctc, RT_CHECK_MODE mode);

// --- Return checker, except for functions that return a boolean ---
bool check_bool(bool function, RT_CHECK_MODE mode);

// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values);

// ---- Logging functions ----
void write_log(std::string src, std::string msg, LOG_LEVEL lvl);