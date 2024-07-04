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


#pragma once
#include "pico/stdlib.h"
#include <vector>
using namespace std;


// ------- Definitions -------

// ---- Enums ----
enum PIN_CONFIG_MODE {OUTPUT, OUTPUT_PWM, INPUT, INPUT_PULLUP, INPUT_PULLDOWN, INPUT_ADC, PROT_I2C, PROT_UART};
enum PIN_STATE {LOW, HIGH};

// ---- Misc. ----
#define temp_sens_vref         3.25f   // Volts
#define adc_conversion_factor  (temp_sens_vref / (1 << 12))


// ------- Functions -------

// ---- Arduino pinMode-like function ----
void init_pin(uint pin, PIN_CONFIG_MODE mode);

// ---- gpio_put function but for PWM-enbaled pins ----
void gpio_put_pwm(uint pin, uint16_t level);

// ---- Arduino map-like function ----
float map(float input, int in_min, int in_max, int out_min, int out_max);

// ---- Adjustable truncate function ----
float truncate_adj(float input, int trunc_amount);

// ---- Calculates the mean (average) of the numbers in a float vector ----
float calculate_mean(vector<float> &numbers);

// ---- Calculates the standard deviation of the numbers in a float vector ----
float calculate_standard_deviation(vector<float> &numbers, float numbers_mean);

// ---- Finds "outliers" in-between the numbers in a float vector using the Z-Score (Standard Score) method ----
// ---- It returns a boolean vector (with the same size as the input vector) that indicates the "outliers" by returning their slots as true ----
vector<bool> standard_score_check(vector<float> &numbers, float z_score_threshhold);

// ---- Returns the temperature measured by the RP2040's internal sensor in Celsius ----
// ---- NOTE: The ADC must be initialized and the temperature sensor must be enabled! ----
float get_rp2040_temp();

// ---- Returns the ADC channel of a given GPIO pin ----
int get_gpio_adc_channel(uint gpio);

// ---- Converts Euler angles to a quaternion ----
// ---- Output: [x, y, z, w] ----
vector<float> euler_to_quaternion(float roll, float pitch, float yaw);