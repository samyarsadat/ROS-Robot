/*
    The ROS robot project - IO Helper Module - General
    (Camera LEDs, battery voltage measurement)
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
#include "Definitions.h"
#include "IO_Helpers_General.h"
#include "helpers_lib/Helpers.h"
#include "hardware/adc.h"



// ------- Functions ------- 

// ---- Sets camera LED outputs ----
void set_camera_leds(uint16_t led_1, uint16_t led_2, uint16_t led_3, uint16_t led_4)
{
    gpio_put_pwm(cam_led_1, led_1);
    gpio_put_pwm(cam_led_2, led_2);
    gpio_put_pwm(cam_led_3, led_3);
    gpio_put_pwm(cam_led_4, led_4);
}


// ---- Get measured battery voltage ----
float get_battery_voltage()
{
    adc_select_input(get_gpio_adc_channel(batt_adc));
    float adc_voltage = adc_read() * adc_conversion_factor;
    return adc_voltage / (batt_voltage_divider_r2 / (batt_voltage_divider_r1 + batt_voltage_divider_r2));
}


// ---- Get the position of the speed select switch ----
// ---- (0: first position, 1: second position, 2: third position) ----
uint8_t get_speed_sel_sw_pos()
{
    if (!gpio_get(speed_sw_1) && !gpio_get(speed_sw_2)) { return 0; }
    else if (gpio_get(speed_sw_1)) { return 1; }
    else if (gpio_get(speed_sw_2)) { return 2; }
    return 0;   // This will never be reached, but it's here just to be safe.
}