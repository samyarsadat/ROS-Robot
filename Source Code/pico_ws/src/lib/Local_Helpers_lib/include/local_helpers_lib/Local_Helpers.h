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



// ------- Structs -------

// Diagnostic key-value pair data item for external use.
struct diag_kv_pair_item
{
    std::string_view key;
    std::string_view value;
};

typedef struct diag_kv_pair_item diag_kv_pair_item_t;

// Ready to publish diagnostics message item struct.
struct diag_publish_item
{
    diagnostic_msgs__msg__DiagnosticStatus *diag_msg;
    int allocated_slot;
};

typedef struct diag_publish_item diag_publish_item_t;



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

// ---- Initialize diagnostics mutexes ----
void diag_mutex_init();

// ---- Initialize diagnostics publisher & message ----
void diag_uros_init();

// ---- Allocate array slots for a message ----
int allocate_slots();

// ---- Deallocate array slots ----
void deallocate_slots(int allocated_slot);

// ---- Populate a diagnostics key-value pair vector ----
void populate_diag_kv_pair(std::vector<diag_kv_pair_item_t> *kv_pairs, int allocated_slot);

// ---- Populate a diagnostics key-value pair reference vector ----
void populate_diag_kv_pair_refs(int allocated_slot);

// ---- Populate the diagnostic status message of the given slot ----
void populate_diag_msg_object(uint8_t level, std::string_view hw_name, std::string_view hw_id, std::string_view msg, int allocated_slot);

// ---- Destroy a diagnostic status message ----
void destroy_diag_msg_object(int allocated_slot);

// ---- Destroy MicroROS DiagnosticStatus message ----
void destroy_uros_diag_status_msg(int allocated_slot);

// ---- Destroy a diagnostics key-value pair vector ----
void destroy_diag_kv_pair(int allocated_slot);

// ---- Destroy a diagnostics key-value pair reference vector ----
void destroy_diag_kv_pair_refs(int allocated_slot);

// ---- Prepares a diag_publish_item_t struct for diagnostics message publishing based on inputs ----
diag_publish_item_t prepare_diag_publish_item(uint8_t level, std::string_view hw_name, std::string_view hw_id, std::string_view msg, std::vector<diag_kv_pair_item_t> *kv_pairs);

// ---- Returns a DiagnosticStatus message from the provided global data array slot ----
diagnostic_msgs__msg__DiagnosticStatus get_diag_msg(int allocated_slot);

// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, std::string_view hw_name, std::string_view hw_id, std::string_view msg, std::vector<diag_kv_pair_item_t> *kv_pairs);

// ---- Temporarily disable publish_diag_report() ----
void disable_diag_pub();

// ---- Re-enable publish_diag_report() ----
void enable_diag_pub();

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