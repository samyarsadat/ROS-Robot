/*
    The ROS robot project - Common Program Definitions
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
#include <limits>


// ------- Pin definitions -------

// ---- Misc. ----
#define onboard_led  25


// ------- Other definitions -------

// ---- Misc. ----
#define PI          3.141f
#define CHECK_FLAG  1687
#define INF         std::numeric_limits<float>::infinity()
#define N_INF       INF * -1

// ---- RCL return checker modes ----
#define RCL_HARD_CHECK      0
#define RCL_SOFT_CHECK      1
#define RCL_LOG_ONLY_CHECK  2