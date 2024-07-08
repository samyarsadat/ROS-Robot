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
#include "local_helpers_lib/Common_Definitions.h"
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <string>
#include "FreeRTOS.h"
#include "queue.h"



// ------- Enums -------
enum RT_CHECK_MODE {RT_HARD_CHECK, RT_SOFT_CHECK, RT_LOG_ONLY_CHECK};
enum LOG_LEVEL {LOG_LVL_INFO, LOG_LVL_WARN, LOG_LVL_ERROR, LOG_LVL_FATAL};
enum LOG_SOURCE_VERBOSITY {FUNCNAME_ONLY, FILENAME_LINE_ONLY, FUNCNAME_LINE_ONLY, FILENAME_LINE_FUNCNAME};


// ------- Functions ------- 

// ----- RETURN CHECKERS -----
// Note: clean_shutdown() must be defined elsewhere!
// Note: diagnostics_msg (type: diagnostic_msgs__msg__DiagnosticStatus) must be defined eslewhere!
// Note: diagnostics_pub (type: rcl_publisher_t) must be defined eslewhere!

// --- RCL return checker ---
bool check_rc(rcl_ret_t rctc, RT_CHECK_MODE mode, const char *func=__builtin_FUNCTION(), uint16_t line=__builtin_LINE());

// --- Return checker, except for functions that return a boolean ---
bool check_bool(bool function, RT_CHECK_MODE mode, const char *func=__builtin_FUNCTION(), uint16_t line=__builtin_LINE());

// ---- Set MicroROS publishing queue ----
void set_diag_pub_queue(QueueHandle_t queue);

// ---- Create a diagnostics key-value pair ----
diagnostic_msgs__msg__KeyValue create_diag_kv_pair(std::string key, std::string value);

// ---- Create a diagnostic status message ----
diagnostic_msgs__msg__DiagnosticStatus create_diag_msg(uint8_t level, std::string hw_name, std::string hw_id, std::string msg, std::vector<diagnostic_msgs__msg__KeyValue> key_values);

// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, std::string hw_name, std::string hw_id, std::string msg, std::vector<diagnostic_msgs__msg__KeyValue> key_values);

// ---- Initialize ADC mutex ----
void adc_init_mutex();

// ---- Take the ADC mutex ----
bool adc_take_mutex();

// ---- Release the ADC mutex ----
void adc_release_mutex();

// ---- Change ADC mux channel with mutex ----
bool adc_select_input_with_mutex(uint8_t channel);

// ---- Initialize print_uart mutex ----
void init_print_uart_mutex();

// ---- Print to STDIO UART function ----
bool print_uart(std::string msg);

// ---- Logging functions ----
void write_log(std::string msg, LOG_LEVEL lvl, LOG_SOURCE_VERBOSITY src_verb, const char *func=__builtin_FUNCTION(), const char *file=__builtin_FILE(), uint16_t line=__builtin_LINE());

// ---- Pings the MicroROS agent ----
bool ping_agent();

// ---- Execution interval checker ----
// ---- Checks the amount of time passed since the last time it was called (with the specific time storage varialble provided) ----
// ---- Returns false if the execution time has exceeded the specified limit ----
bool check_exec_interval(uint32_t &last_call_time_ms, uint16_t max_exec_time_ms, std::string log_msg, bool publish_diag = false, const char *func=__builtin_FUNCTION());