/*
    The ROS robot project - IO Helper Module - Multiplexer
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
#include "helpers_lib/Helpers.h"


// ------- Functions ------- 

// ---- Sets the mux pins ----
void set_mux_pins(uint8_t addr_a, uint8_t addr_b, uint8_t addr_c, uint8_t addr_d, uint8_t io);

// ---- Set mux address pins according to mux IO pin number ----
void set_mux_addr(uint pin);

// ---- Set mux IO mode ----
void set_mux_io_mode(PIN_CONFIG_MODE mode);

// ---- Take the multiplexer mutex ----
bool take_mux_mutex();

// ---- Release the multiplexer mutex ----
void release_mux_mutex();