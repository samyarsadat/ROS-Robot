/*
   The ROS robot project - Helper/commonly used functions
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
#include <memory>


// ------- Definitions -------

// TODO: Make these Enums
// ---- Pin direction/type ----
#define OUTPUT          1
#define OUTPUT_PWM      2
#define INPUT           3
#define INPUT_PULLUP    4
#define INPUT_PULLDOWN  5
#define INPUT_ADC       6

// ---- Pin state ----
#define HIGH  1
#define LOW   0


// ------- Functions -------

// ---- Arduino pinMode-like function ----
void init_pin(uint pin, int mode);

// ---- gpio_put function but for PWM-enbaled pins ----
void gpio_put_pwm(uint pin, uint16_t level);

// ---- Arduino map-like function ----
float map(float input, int in_min, int in_max, int out_min, int out_max);

// ---- Adjustable truncate function ----
float truncate_adj(float input, int trunc_amount);

// ---- Calculates the mean (average) of the numbers in a float array ----
float calculate_mean(float numbers[], int array_length);

// ---- Calculates the standard deviation of the numbers in a float array ----
float calculate_standard_deviation(float numbers[], int array_length);

// ---- Finds "outliers" in-between the numbers in a float array using the Z-Score (Standard Score) method ----
// ---- It returns a boolean array (with the same size as the input array) that indicates the "outliers" by returning their array slots as true ----
std::unique_ptr<bool[]> standard_score_check(float numbers[], int array_length, float z_score_threshhold);