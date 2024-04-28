/*
    The ROS robot project - Local Helper/commonly used functions
    Common diagnostics message definitions
    
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


// ---- STATUS OK ----

// ---- STATUS WARM ----
#define DIAG_WARN_MSG_TIMER_EXEC_TIME_OVER  (char*) "One of the timers' execution intervals exceeded limits"
#define DIAG_WARN_MSG_UROS_RC_CHECK_FAIL    (char*) "MicroROS RCL return checker fail (Soft Fail)"
#define DIAG_WARN_MSG_BOOL_RT_CHECK_FAIL    (char*) "Boolean function return checker fail (Soft Fail)"

// ---- STATUS ERROR ----
#define DIAG_ERR_MSG_UROS_RC_CHECK_FAIL  (char*) "MicroROS RCL return checker fail (Hard Fail)"
#define DIAG_ERR_MSG_INIT_FAILED         (char*) "Init. failed"
#define DIAG_ERR_MSG_BOOL_RT_CHECK_FAIL  (char*) "Boolean function return checker fail (Hard Fail)"

// ---- STATUS STALE ----