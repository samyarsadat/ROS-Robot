/*
   The ROS robot project - Helper/commonly used functions
   Copyright 2022-2023 Samyar Sadat Akhavi
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
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <math.h>
#include "Helpers.h"


// ------- Functions -------
// TODO: Better usage information
// TODO: Use enums instead of definitions for init_pin()

// ---- Arduino pinMode-like function ----
void init_pin(uint pin, int mode)
{
    switch(mode)
    {
        case 1:
            gpio_init(pin);
            gpio_set_dir(pin, true);
            break;

        case 2:
            gpio_init(pin);
            gpio_set_dir(pin, true);
            gpio_set_function(pin, GPIO_FUNC_PWM);
            pwm_set_enabled(pwm_gpio_to_slice_num(pin), true);
            break;

        case 3:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            break;

        case 4:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            gpio_pull_up(pin);
            break;

        case 5:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            gpio_pull_down(pin);
            break;

        case 6:
            adc_gpio_init(pin);
            gpio_set_dir(pin, false);
            break;
    }
}


// ---- gpio_put function but for PWM-enbaled pins ----
void gpio_put_pwm(uint pin, uint16_t level)
{
    // All A channel PWM pins' pin numbers are even so we can easily check to see whether
    // the pin we are setting is on channel A or channel B.
    if (pin % 2 == 0)
    {
        pwm_set_chan_level(pwm_gpio_to_slice_num(pin), PWM_CHAN_A, level);
    }

    else
    {
        pwm_set_chan_level(pwm_gpio_to_slice_num(pin), PWM_CHAN_B, level);
    }
}


// ---- Arduino map-like function ----
float map(float input, int in_min, int in_max, int out_min, int out_max)
{
    return (float) (((input - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}


// ---- Adjustable truncate function ----
float truncate_adj(float input, int trunc_amount)
{
    return round(input * pow(10, trunc_amount)) / pow(10, trunc_amount);
}