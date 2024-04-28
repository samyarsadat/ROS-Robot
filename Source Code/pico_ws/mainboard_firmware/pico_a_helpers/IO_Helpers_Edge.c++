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
#include "IO_Helpers_Edge.h"
#include "pico/stdlib.h"
#include "helpers_lib/Helpers.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "IO_Helpers_Mux.h"
#include "Definitions.h"
#include <memory>



// ------- Global variables -------
uint ir_en_pin;
uint16_t ambient_reading;



// ------- Functions ------- 

// ---- Sets the IR enable pin ----
void set_ir_en_pin(uint en_pin)
{
    ir_en_pin = en_pin;
    init_pin(ir_en_pin, OUTPUT);
    adc_init();
}


// TODO: Self-check function.
char* ir_self_test()
{
    float readings[num_ir_sensors];
    gpio_put(ir_en_pin, LOW);
    set_mux_io_mode(INPUT_ADC);
    sleep_us(10);

    for (int i = 0; i < num_ir_sensors; i++)
    {
        set_mux_addr(i + 8);
        readings[i] = (float) adc_read();
    }

    std::unique_ptr<bool[]> results = standard_score_check(readings, num_ir_sensors, ir_self_test_z_score_threshhold);

    for (int i = 0; i < num_ir_sensors; i++)
    {
        
    }

    return NULL;
}


// ---- Take ambient IR reading with emitters off and set reading offset ----
void calibrate_ir_offset()
{
    uint32_t readings_total;
    gpio_put(ir_en_pin, LOW);
    set_mux_io_mode(INPUT_ADC);
    sleep_us(10);

    for (int i = 0; i < num_ir_sensors; i++)
    {
        set_mux_addr(i + 8);
        readings_total = readings_total + adc_read();
    }

    ambient_reading = readings_total / num_ir_sensors;
}


// ---- Return the status of each sensor as true or false (triggered or not), as an array ----
bool* get_ir_status()
{
    static bool readings[num_ir_sensors];
    gpio_put(ir_en_pin, HIGH);
    set_mux_io_mode(INPUT_ADC);
    sleep_us(10);

    for (int i = 0; i < num_ir_sensors; i++)
    {
        set_mux_addr(i + 8);
        readings[i] = !((adc_read() - ambient_reading) > ir_trigger_limit);
    }

    return readings;
}