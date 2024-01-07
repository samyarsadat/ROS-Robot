/*
    The ROS robot project - IO Helper Module - Ultrasonic Sensor
    Copyright 2022-2023 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2022-2023.
 
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
#include "../lib/Helper_lib/Helpers.h"
#include "hardware/pwm.h"
#include "IO_Helpers_Mux.c++"



// ------- Definitions -------
#define ultrasonic_signal_timout_us  32*1000   // 32 milliseconds



// ------- Functions ------- 

// ---- Dual-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_mux(uint echo_io, uint trig_io)
{
    set_mux_io_mode(OUTPUT);
    set_mux_addr(trig_io);
    
    gpio_put(mux_io, HIGH);
    sleep_us(10);
    gpio_put(mux_io, LOW);
    sleep_us(5);

    uint32_t start_time = time_us_32();

    set_mux_io_mode(INPUT);
    set_mux_addr(echo_io);

    while (!gpio_get(mux_io) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
    uint32_t pulse_start = time_us_32();

    while (gpio_get(mux_io) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
    uint32_t pulse_end = time_us_32();

    uint16_t time_diff = pulse_end - pulse_start;
    return (time_diff * 34.3) / 2;
}


// ---- Single-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_single(uint ultra_pin)
{
    gpio_deinit(ultra_pin);
    init_pin(ultra_pin, OUTPUT);
    
    gpio_put(ultra_pin, HIGH);
    sleep_us(10);
    gpio_put(ultra_pin, LOW);
    sleep_us(5);

    uint32_t start_time = time_us_32();

    gpio_deinit(ultra_pin);
    init_pin(ultra_pin, INPUT);

    while (!gpio_get(ultra_pin) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
    uint32_t pulse_start = time_us_32();

    while (gpio_get(ultra_pin) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
    uint32_t pulse_end = time_us_32();

    uint16_t time_diff = pulse_end - pulse_start;
    return (time_diff * 34.3) / 2;
}