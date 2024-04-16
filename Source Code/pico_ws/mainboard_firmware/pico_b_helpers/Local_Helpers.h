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



// ------- Functions ------- 

// ----- RETURN CHECKERS -----
// Note: clean_shutdown() must be defines elsewhere!
// Note: publish_diag_report() must be defines elsewhere!

// --- RCL return checker ---
bool check_rc(rcl_ret_t rctc, uint mode);

// --- Return checker, except for functions that return a boolean ---
bool check_bool(bool function, uint mode);

// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values);