/*
    The ROS robot project - IO Helper Module - IR Edge Sensors INCOMPLETE
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


// ------- Global variables -------
extern uint ir_en_pin;
extern uint16_t ambient_reading;


// ------- Functions ------- 

// ---- Sets the IR enable pin ----
void set_ir_en_pin(uint en_pin);

// TODO: Self-check function.
char* ir_self_test();

// ---- Take ambient IR reading with emitters off and set reading offset ----
void calibrate_ir_offset();

// ---- Return the status of each sensor as true or false (triggered or not), as an array ----
bool* get_ir_status();