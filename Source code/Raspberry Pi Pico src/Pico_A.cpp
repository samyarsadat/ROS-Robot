/*
   The ROS robot project - Raspberry Pi Pico (A) program
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
#include <charconv>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "lib/helpers.h"


// ------- Pin defines -------

// ---- Misc. ----
#define power_led         1
#define onboard_led       25
#define pico_b_ready_sig  28
#define pi_power_relay    6

// ---- Ultrasonic sensor ----
#define front_ultra  2
#define back_ultra   3
#define right_ultra  4
#define left_ultra   5

// ---- Edge detection sensor multiplexer ----
#define edge_sens_multiplex_s2   13
#define edge_sens_multiplex_s1   14
#define edge_sens_multiplex_s0   15
#define edge_sens_multiplex_sig  26

// ---- Motor encoders & motor driver ----
#define l_motor_1_enc_b  7
#define l_motor_1_enc_a  9
#define l_motor_2_enc_b  8
#define l_motor_2_enc_a  10
#define r_motor_1_enc_b  22
#define r_motor_1_enc_a  11
#define r_motor_2_enc_b  27
#define r_motor_2_enc_a  12
#define l_motor_drive_1  21
#define l_motor_drive_2  20
#define r_motor_drive_1  19
#define r_motor_drive_2  18

// ---- I2C ----
#define i2c_sda  16
#define i2c_scl  17


// ------- Other defines -------

// ---- Multicore check flag ----
#define FLAG 1687


// ------- Functions -------

// ---- Pin init ----
void init_pins()
{
    // ---- Inputs ----
    init_pin(pico_b_ready_sig, INPUT);
    init_pin(edge_sens_multiplex_sig, INPUT);
    init_pin(l_motor_1_enc_b, INPUT);
    init_pin(l_motor_1_enc_a, INPUT);
    init_pin(l_motor_2_enc_b, INPUT);
    init_pin(l_motor_2_enc_a, INPUT);
    init_pin(r_motor_1_enc_b, INPUT);
    init_pin(r_motor_1_enc_a, INPUT);
    init_pin(r_motor_2_enc_b, INPUT);
    init_pin(r_motor_2_enc_a, INPUT);

    // ---- Outputs ----
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);
    init_pin(pi_power_relay, OUTPUT);
    init_pin(l_motor_drive_1, OUTPUT);
    init_pin(l_motor_drive_2, OUTPUT);
    init_pin(r_motor_drive_1, OUTPUT);
    init_pin(r_motor_drive_2, OUTPUT);
    init_pin(edge_sens_multiplex_s2, OUTPUT);
    init_pin(edge_sens_multiplex_s1, OUTPUT);
    init_pin(edge_sens_multiplex_s0, OUTPUT);

    printf("Pin init complete\r\n");
}


// ---- Setup loop (Runs once) ----
void setup()
{
    init_pins();
    stdio_init_all();
}


// ---- Main loop (Runs forever on core 0) ----
void loop()
{
    printf("Blinking!\r\n");
    gpio_put(onboard_led, true);
    sleep_ms(1000);
    gpio_put(onboard_led, false);
    sleep_ms(1000);
}


// ---- Main loop (Runs forever on core 1) ----
void loop1()
{
    printf("Loop1!\r\n");
    sleep_ms(1000);
}


// ------- Init -------

// ---- Core 1 loop init ----
void init_core_1()
{
    multicore_fifo_push_blocking(FLAG);
    uint32_t flag_r = multicore_fifo_pop_blocking();
    
    if (flag_r != FLAG)
    {
        printf("Core 1 fail!\r\n");
    }

    else
    {
        printf("Core 1 starting...\r\n");

        while(true)
        {
            loop1();
        }
    }
}


// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    setup();

    multicore_reset_core1();
    multicore_launch_core1(init_core_1);

    multicore_fifo_push_blocking(FLAG);
    uint32_t flag_r = multicore_fifo_pop_blocking();
    
    if (flag_r != FLAG)
    {
        printf("Core 0 fail!\r\n");
    }

    else
    {
        printf("Core 0 starting...\r\n");

        while(true)
        {
            loop();
        }
    }
}