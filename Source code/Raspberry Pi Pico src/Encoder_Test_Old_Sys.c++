/*
   The ROS robot project - Encoder system test program (Method 1 - No 0 RPM det., stable RPM readings)
   Copyright 2022 Samyar Sadat Akhavi
   Written by Samyar Sadat Akhavi, 2022.
 
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
#include <stdio.h>
#include "pico/stdlib.h"
#include "lib/helpers.h"
#include "lib/PID_lib/PID_v1.h"
#include "hardware/pwm.h"


// ------- Global variables -------
uint slice_num;
uint32_t enc_time, enc_time_old, enc_time_diff = 5000000;
bool enc_other_pulse = true;
char msg[50];


// ------- Defines -------
#define motor_1                  16
#define motor_2                  17
#define motor_enc_a              18
#define enc_pulses_per_rotation  2
#define motor_gear_ratio         80/1


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    if (enc_other_pulse)
    {
        enc_time_old = enc_time;
        enc_time = time_us_32();
        enc_time_diff = enc_time - enc_time_old;

        enc_other_pulse = false;
    }

    else
    {
        enc_other_pulse = true;
    }
}


// ---- Pin init ----
void init_pins()
{
    // ---- Inputs ----
    init_pin(motor_enc_a, INPUT);

    // ---- Outputs ----
    init_pin(motor_1, OUTPUT_PWM);
    init_pin(motor_2, OUTPUT_PWM);

    // ---- PWM ----
    pwm_set_wrap(slice_num, 65535);
    slice_num = pwm_gpio_to_slice_num(motor_1);

    // ---- Interrupts ----
    gpio_set_irq_enabled_with_callback(motor_enc_a, GPIO_IRQ_EDGE_RISE, true, &irq_call);
}


// ------- Main program -------
void setup()
{
    // --- Set motor to max ---
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 65535);
    pwm_set_enabled(slice_num, true);
    
    stdio_init_all();
}


void loop()
{
    float tor_ms = (motor_gear_ratio * (enc_time_diff * (enc_pulses_per_rotation / 2))) / 1000;
    float rpm = 60000 / tor_ms;

    sprintf(msg, "Motor RPM: %f\nEncoder time diff: %i\n", rpm, enc_time_diff);
    printf(msg);
}


int main() 
{
    init_pins();
    setup();

    while (true)
    {
        loop();
    }
}