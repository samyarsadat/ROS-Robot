/*
    The ROS robot project - IO Helper Module - Ultrasonic Sensor
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
#include "../lib/Helper_lib/Helpers.h"
#include "hardware/pwm.h"
#include "IO_Helpers_Mux.h"
#include "Definitions.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>



// ------- Functions ------- 

// ---- Diagnostics reporting function prototype ----
extern void publish_diag_report(uint8_t level, char *hw_source, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values);


// ---- (INTERNAL) Checks whether the measured distance is within limits ----
float check_return_distance(float dist, char *ultra_hwid)
{
    if (dist < ultra_min_dist)
    {
        publish_diag_report(DIAG_LVL_WARN, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_ULTRASONICS, ultra_hwid, DIAG_WARN_MSG_ULTRA_MIN_LIM_EXCEED, NULL);
        return N_INF;
    }

    else if (dist > ultra_max_dist)
    {
        publish_diag_report(DIAG_LVL_WARN, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_ULTRASONICS, ultra_hwid, DIAG_WARN_MSG_ULTRA_MAX_LIM_EXCEED, NULL);
        return INF;
    }

    else
    {
        return dist;
    }
}


// ---- Dual-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_mux(uint trig_io, uint echo_io, char *ultra_hwid)
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
    float dist = (time_diff * 34.3) / 2;

    return check_return_distance(dist, ultra_hwid);
}


// ---- Single-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_single(uint ultra_pin, char *ultra_hwid)
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
    float dist = (time_diff * 34.3) / 2;

    return check_return_distance(dist, ultra_hwid);
}