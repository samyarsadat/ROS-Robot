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


// ------- Libraries & Modules -------
#include "pico/stdlib.h"
#include "../lib/Helper_lib/Helpers.h"
#include "hardware/pwm.h"
#include "IO_Helpers_Mux.c++"



// ------- Global variables -------
uint ir_en_pin;
bool ir_enabled;
float ambient_reading;



// ------- Definitions -------
// INCOMPLETE


// ------- Functions ------- 

// ---- Sets the IR enable pin ----
void set_ir_en_pin(uint en_pin)
{
    ir_en_pin = en_pin;
    init_pin(ir_en_pin, OUTPUT);
}


// ---- Take ambient IR reading with emitters off and set reading offset ----
void calibrate_ir_offset()
{
    gpio_put(ir_en_pin, LOW);
    set_mux_io_mode(INPUT_ADC);
    sleep_us(5);
    // INCOMPLETE
}