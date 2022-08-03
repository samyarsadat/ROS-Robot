/*
   The ROS robot project - Raspberry Pi Pico (B) program
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
#define onboard_led  25
#define ready_sig    2

// ---- Settings switches ----
#define speed_sw_1  22
#define speed_sw_2  17
#define mode_sw     16

// ---- Raspberry Pi camera LEDs ----
#define cam_led_1  18
#define cam_led_2  19
#define cam_led_3  20
#define cam_led_4  21

// ---- Micro switches ----
#define ms_front_l  13
#define ms_front_r  12
#define ms_back_l   11
#define ms_back_r   10


// ------- Other defines -------

// ---- Multicore check flag ----
#define FLAG 1687


// ------- Functions -------

// ---- Pin init ----
void init_pins()
{
    // ---- Inputs ----
    init_pin(speed_sw_1, INPUT_PULLUP);
    init_pin(speed_sw_2, INPUT_PULLUP);
    init_pin(mode_sw, INPUT_PULLUP);
    init_pin(ms_front_l, INPUT_PULLUP);
    init_pin(ms_front_r, INPUT_PULLUP);
    init_pin(ms_back_l, INPUT_PULLUP);
    init_pin(ms_back_r, INPUT_PULLUP);

    // ---- Outputs ----
    init_pin(onboard_led, OUTPUT);
    init_pin(ready_sig, OUTPUT);
    init_pin(cam_led_1, OUTPUT);
    init_pin(cam_led_2, OUTPUT);
    init_pin(cam_led_3, OUTPUT);
    init_pin(cam_led_4, OUTPUT);

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

}


// ---- Main loop (Runs forever on core 1) ----
void loop1()
{

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