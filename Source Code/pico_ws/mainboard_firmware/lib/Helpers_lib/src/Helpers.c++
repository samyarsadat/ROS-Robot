/*
    The ROS robot project - Helper/commonly used functions
    These are general IO/math functions that can be used in any other program.
    They are not program-specific.
    
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
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <math.h>
#include <cmath>
#include <memory>
#include "helpers_lib/Helpers.h"
using namespace std;


// ------- Functions -------
// TODO: Better usage information

// ---- Arduino pinMode-like function ----
void init_pin(uint pin, PIN_CONFIG_MODE mode)
{
    switch(mode)
    {
        case OUTPUT:
            gpio_init(pin);
            gpio_set_dir(pin, true);
            break;

        case OUTPUT_PWM:
            gpio_init(pin);
            gpio_set_dir(pin, true);
            gpio_set_function(pin, GPIO_FUNC_PWM);
            pwm_set_enabled(pwm_gpio_to_slice_num(pin), true);
            break;

        case INPUT:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            break;

        case INPUT_PULLUP:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            gpio_pull_up(pin);
            break;

        case INPUT_PULLDOWN:
            gpio_init(pin);
            gpio_set_dir(pin, false);
            gpio_pull_down(pin);
            break;

        case INPUT_ADC:
            adc_gpio_init(pin);
            gpio_set_dir(pin, false);
            break;

        case PROT_I2C:
            gpio_init(pin);
            gpio_set_function(pin, GPIO_FUNC_I2C);
            break;
        
        case PROT_UART:
            gpio_init(pin);
            gpio_set_function(pin, GPIO_FUNC_UART);
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


// ---- Calculates the mean (average) of the numbers in a float array ----
float calculate_mean(float numbers[], int array_length)
{
    float total;

    for (int i = 0; i < array_length; i++)
    {
        total += numbers[i];
    }

    return total / array_length;
}


// ---- Calculates the standard deviation of the numbers in a float array ----
float calculate_standard_deviation(float numbers[], int array_length, float numbers_mean)
{
    float deviation_total;

    for (int i = 0; i < array_length; i++)
    {
        deviation_total += pow(numbers[i] - numbers_mean, 2);
    }

    return sqrt(deviation_total / array_length);
}


// ---- Finds "outliers" in-between the numbers in a float array using the Z-Score (Standard Score) method ----
// ---- It returns a boolean array (with the same size as the input array) that indicates the "outliers" by returning their array slots as true ----
unique_ptr<bool[]> standard_score_check(float numbers[], int array_length, float z_score_threshhold)
{
    float mean = calculate_mean(numbers, array_length);
    float standard_deviation = calculate_standard_deviation(numbers, array_length, mean);
    unique_ptr<bool[]> outliers(new bool[array_length]);
    
    for (int i = 0; i < array_length; i++)
    {
        outliers[i] = false;

        if (abs((numbers[i] - mean) / standard_deviation) > z_score_threshhold)
        {
            outliers[i] = true;
        }
    }

    return outliers;
}


// ---- Returns the temperature measured by the RP2040's internal sensor in Celsius ----
// ---- NOTE: The ADC must be initialized and the temperature sensor must be enabled! ----
float get_rp2040_temp()
{
    adc_select_input(4);
    double reading_volts = adc_read() * adc_conversion_factor;
    float reading_celsius = 27 - (reading_volts - 0.706) / 0.001721;  // Formula taken from the RP2040 datasheet.

    return reading_celsius;
}


// ---- Returns the ADC channel of a given GPIO pin ----
int get_gpio_adc_channel(uint gpio)
{
    if (gpio == 26) { return 0; }
    else if (gpio == 27) { return 1; }
    else if (gpio == 28) { return 2; }
    return 0;   // Just in case there's user error and a non-ADC pin is provided.
}