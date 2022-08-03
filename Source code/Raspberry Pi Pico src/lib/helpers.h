/*
   The ROS robot project - Arduino functions library
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


// ------- Defines -------

// ---- Pin direction ----
#define OUTPUT          1
#define INPUT           2
#define INPUT_PULLUP    3
#define INPUT_PULLDOWN  4


// ------- Functions -------

// ---- Arduino pinMode-like function ----
void init_pin(int pin, int mode)
{
    switch(mode)
    {
        case 1:
            gpio_init(pin);
            gpio_set_dir(pin, true);
            break;

        case 2:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            break;

        case 3:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            gpio_pull_up(pin);
            break;

        case 4:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            gpio_pull_down(pin);
            break;
    }
}