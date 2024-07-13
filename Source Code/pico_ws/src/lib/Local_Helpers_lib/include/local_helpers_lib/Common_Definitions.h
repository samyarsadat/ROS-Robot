/*
    The ROS robot project - Local Helper/commonly used functions
    Common program definitions
    
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
#include "local_helpers_lib/Common_Diag_Msgs.h"
#include "local_helpers_lib/Diagnostics_Definitons.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <limits>
#include <vector>


// ------- Pin definitions -------

// ---- Misc. ----
#define onboard_led  25


// ------- Other definitions -------

// ---- Misc. ----
#define PI     3.141f
#define TAU    (2 * PI)
#define INF    std::numeric_limits<float>::infinity()
#define N_INF  (INF * -1)

// ---- Diagnostics message levels ----
#define DIAG_LVL_OK     diagnostic_msgs__msg__DiagnosticStatus__OK
#define DIAG_LVL_WARN   diagnostic_msgs__msg__DiagnosticStatus__WARN
#define DIAG_LVL_ERROR  diagnostic_msgs__msg__DiagnosticStatus__ERROR
#define DIAG_LVL_STALE  diagnostic_msgs__msg__DiagnosticStatus__STALE

// ---- MicroROS agent detection ----
#define uros_agent_find_timeout_ms  100
#define uros_agent_find_attempts    10