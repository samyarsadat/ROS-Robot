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
#include "IO_Helpers_Ultrasonic.h"
#include "IO_Helpers_Mux.h"
#include "Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"



// ------- Functions ------- 

// ---- (INTERNAL) Checks whether the measured distance is within limits ----
float check_return_distance(float dist, std::string_view ultra_hwid)
{
    std::string log_distance = "[dist: " + std::to_string(dist) + ", " + std::string{ultra_hwid} + "]";

    if (dist < ultra_min_dist)
    {
        publish_diag_report(DIAG_LVL_WARN, DIAG_NAME_ULTRASONICS, std::string{ultra_hwid}, DIAG_WARN_MSG_ULTRA_MIN_LIM_EXCEED, DIAG_KV_PAIR_VEC());
        write_log("Ultrasonic sensor distance below minimum limit! " + log_distance, LOG_LVL_WARN, FUNCNAME_LINE_ONLY);
        return N_INF;
    }

    else if (dist > ultra_max_dist)
    {
        // NOTE: This could become a problem if the robot is placed in an open space where the sensors might not get their signals back.
        publish_diag_report(DIAG_LVL_WARN, DIAG_NAME_ULTRASONICS, std::string{ultra_hwid}, DIAG_WARN_MSG_ULTRA_MAX_LIM_EXCEED, DIAG_KV_PAIR_VEC());
        write_log("Ultrasonic sensor distance above maximum limit! " + log_distance, LOG_LVL_WARN, FUNCNAME_LINE_ONLY);
        return INF;
    }

    else
    {
        return dist;
    }
}


// ---- Dual-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_mux(uint trig_io, uint echo_io, std::string ultra_hwid)
{
    if (check_bool(take_mux_mutex(), RT_LOG_ONLY_CHECK))
    {
        set_mux_io_mode(OUTPUT);
        set_mux_addr(trig_io);
        
        taskENTER_CRITICAL();

        gpio_put(analog_mux_io, HIGH);
        sleep_us(10);
        gpio_put(analog_mux_io, LOW);

        uint32_t start_time = time_us_32();

        set_mux_io_mode(INPUT);
        set_mux_addr(echo_io);

        while (!gpio_get(analog_mux_io) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
        uint32_t pulse_start = time_us_32();

        while (gpio_get(analog_mux_io) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
        uint32_t pulse_end = time_us_32();

        release_mux_mutex();
        taskEXIT_CRITICAL();

        uint16_t time_diff = pulse_end - pulse_start;
        float dist = (time_diff * 0.0343) / 2;

        return check_return_distance(dist, ultra_hwid);
    }

    return N_INF;
}


// ---- Single-pin ultrasonic sensor distance measurement using mux ----
float get_ultra_dist_single(uint ultra_pin, std::string ultra_hwid)
{
    gpio_deinit(ultra_pin);
    init_pin(ultra_pin, OUTPUT);

    taskENTER_CRITICAL();

    gpio_put(ultra_pin, HIGH);
    sleep_us(10);
    gpio_put(ultra_pin, LOW);

    uint32_t start_time = time_us_32();

    gpio_deinit(ultra_pin);
    init_pin(ultra_pin, INPUT);

    while (!gpio_get(ultra_pin) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
    uint32_t pulse_start = time_us_32();

    while (gpio_get(ultra_pin) && ((time_us_32() - start_time) < ultrasonic_signal_timout_us));
    uint32_t pulse_end = time_us_32();

    taskEXIT_CRITICAL();

    uint16_t time_diff = pulse_end - pulse_start;
    float dist = (time_diff * 0.0343) / 2;

    return check_return_distance(dist, ultra_hwid);
}


// ---- Ultrasonic sensor self-test ----
ULTRA_TEST_RESULT_t ultra_self_test(std::function<float()> sensor_get_dist_func)
{
    ULTRA_TEST_RESULT_t test_result;
    float avg_dist;

    for (int i = 0; i < ultra_selftest_measurements; i++)
    {
        float current_reading = sensor_get_dist_func();
        test_result.measured_distance = current_reading;

        if (current_reading == INF || current_reading < ultra_min_dist)
        {
            test_result.status = ULTRA_TEST_FAIL_BELOW_ABSOLUTE_MINIMUM;
            return test_result;
        }

        else if (current_reading == N_INF || current_reading > ultra_max_dist)
        {
            test_result.status = ULTRA_TEST_FAIL_ABOVE_ABSOLUTE_MAXIMUM;
            return test_result;
        }

        avg_dist += sensor_get_dist_func();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    avg_dist /= ultra_selftest_measurements;
    test_result.measured_distance = avg_dist;

    if (avg_dist < ultra_selftest_range_min_cm || avg_dist > ultra_selftest_range_max_cm)
    {
        test_result.status = ULTRA_TEST_FAIL_OUT_OF_TEST_RANGE;
        return test_result;
    }

    test_result.status = ULTRA_TEST_PASS;
    return test_result;
}