/*
    Motor control library (motor encoder interface) - Written for the ROS Robot Project
    This library handles encoder inputs from motors, and controls them with a PID controller.

    Copyright 2023-2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2023-2024.
 
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


// ---- Static config ----
#define rpm_zero_threshold  1


// ---- Main Encoder object ----
class MotorEncoder
{
    public:
        enum enc_direction
        {
            FORWARD = 1,
            BACKWARD = 0
        };

        // Single-channel encoder constructor
        MotorEncoder(uint8_t encoder_pin, float mtr_gear_ratio, uint16_t enc_pulses_per_rotation, gpio_irq_callback_t gpio_irq_callback_func);

        // Dual-channel encoder constructor
        MotorEncoder(uint8_t encoder_chan_a_pin, uint8_t encoder_chan_b_pin, float mtr_gear_ratio, uint16_t enc_pulses_per_rotation, gpio_irq_callback_t gpio_irq_callback_func);


        // ---- Functions ----
        
        // Sets motor gear ratio config variable.
        void set_gear_ratio(float gear_ratio);

        // Sets method 1 RPM cutoff config variable.
        void set_method_1_cutoff(uint16_t rpm);

        // Sets encoder pulses per pre-gearbox shaft rotation config variable.
        void set_enc_pulses_per_rotation(uint16_t pulses);

        // Configures whether the motor encoder measured direction should be reversed.
        void set_enc_direction_reversed(bool is_reversed);

        // This function should be called in the GPIO irq callback function.
        void enc_hardware_irq_trigger(uint8_t pin);

        // This function should be called in a timer interrupt callback function.
        // How often this function is called determines the sample time of method 2 RPM measurements.
        void enc_timer_irq_trigger();

        // Calculates and returns the motor's measured RPM using method 1.
        float get_m1_rpm();

        // Calculates and returns the motor's measured RPM using method 2.
        float get_m2_rpm();

        // Calculates and returns the motor's measured RPM using both methods.
        float get_rpm();

        // Returns the encoder position in terms of number of pulses.
        // This value increases/decreases based on the measured direction.
        int32_t get_pulse_counter();

        // Returns measured motor rotation direction.
        enc_direction get_direction();


    private:
        uint8_t channel_a_pin;
        uint8_t channel_b_pin;
        bool is_dual_channel_encoder;

        uint16_t encoder_pulses_per_rotation;
        uint16_t method_1_cutoff_rpm;
        float motor_gear_ratio;

        uint16_t enc_pulses;
        uint32_t enc_pulse_time;
        uint32_t enc_last_pulse_rst;
        uint32_t enc_time_per_rot;
        uint32_t enc_current_pulse_time;
        uint32_t enc_last_pulse_time;

        float measured_speed;
        enc_direction measured_direction;
        bool direction_reversed;
        int32_t enc_pulse_counter;
};