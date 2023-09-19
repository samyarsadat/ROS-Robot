/*
    Motor control library (motor encoder interface) - Written for the ROS Robot Project
    This library handles encoder inputs from motors, and controls them with a PID controller.
    More about how this implementation works in ../../README.md

    Copyright 2023 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2023.
 
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


#include "pico/stdlib.h"
#include "../Helper_lib/Helpers.h"
#include "Motor_Encoder.h"


/*  Constructor (dual-channel encoders)
 *  
 *  Arguments:
 *    uint encoder_chan_a_pin: encoder's A channel pin
 *    uint encoder_chan_b_pin: encoder's B channel pin
 *    float mtr_gear_ratio: the motor's gear ratio (e.g. 80/1 = 80:1)
 *    uint enc_pulses_per_rotation: number of pulses sent by the encoder per pre-gearbox shaft rotation
 *    gpio_irq_callback_t gpio_irq_callback_func: GPIO irq callback function
 */
MotorEncoder::MotorEncoder(uint encoder_chan_a_pin, uint encoder_chan_b_pin, float mtr_gear_ratio, uint enc_pulses_per_rotation, gpio_irq_callback_t gpio_irq_callback_func)
{
    is_dual_channel_encoder = true;
    channel_a_pin = encoder_chan_a_pin; 
    channel_b_pin = encoder_chan_b_pin;
    MotorEncoder::set_gear_ratio(mtr_gear_ratio);
    MotorEncoder::set_enc_pulses_per_rotation(enc_pulses_per_rotation);
    MotorEncoder::set_enc_direction_reversed(false);
    MotorEncoder::set_method_1_cutoff(12);

    init_pin(channel_a_pin, INPUT);
    init_pin(channel_b_pin, INPUT);
    
    gpio_set_irq_enabled_with_callback(channel_a_pin, GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback_func);
}


/*  Constructor (single-channel encoders)
 *  
 *  Arguments:
 *    uint encoder_pin: encoder's signal pin
 *    float mtr_gear_ratio: the motor's gear ratio (e.g. 80/1 = 80:1)
 *    uint enc_pulses_per_rotation: number of pulses sent by the encoder per pre-gearbox shaft rotation
 *    gpio_irq_callback_t gpio_irq_callback_func: GPIO irq callback function
 */
MotorEncoder::MotorEncoder(uint encoder_pin, float mtr_gear_ratio, uint enc_pulses_per_rotation, gpio_irq_callback_t gpio_irq_callback_func)
{
    is_dual_channel_encoder = false;
    channel_a_pin = encoder_pin; 
    MotorEncoder::set_gear_ratio(mtr_gear_ratio);
    MotorEncoder::set_enc_pulses_per_rotation(enc_pulses_per_rotation);
    MotorEncoder::set_enc_direction_reversed(false);
    MotorEncoder::set_method_1_cutoff(12);

    init_pin(channel_a_pin, INPUT);
    
    gpio_set_irq_enabled_with_callback(channel_a_pin, GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback_func);
}


// --------- Public functions ---------

/*  Sets motor gear ratio config variable.
 *  
 *  Arguments:
 *    float gear_ratio: motor gear ratio (e.g. 80/1 = 80:1)
 * 
 *  Returns:
 *    void
 */
void MotorEncoder::set_gear_ratio(float gear_ratio)
{
    motor_gear_ratio = gear_ratio;
}


/*  Sets method 1 RPM cutoff config variable.
 *  
 *  Arguments:
 *    int RPM: cutoff RPM
 * 
 *  Returns:
 *    void
 */
void MotorEncoder::set_method_1_cutoff(int rpm)
{
    method_1_cutoff_rpm = rpm;
}


/*  Sets encoder pulses per pre-gearbox shaft rotation config variable.
 *  
 *  Arguments:
 *    int pulses: number of pulses
 * 
 *  Returns:
 *    void
 */
void MotorEncoder::set_enc_pulses_per_rotation(int pulses)
{
    encoder_pulses_per_rotation = pulses;
}


/*  Configures whether the motor encoder measured direction should be reversed.
 *  
 *  Arguments:
 *    bool is_reversed: true to reverse, false to not reverse
 * 
 *  Returns:
 *    void
 */
void MotorEncoder::set_enc_direction_reversed(bool is_reversed)
{
    direction_reversed = is_reversed;
}


/*  This function should be called in the GPIO irq callback function.
 *  
 *  Arguments:
 *    uint pin: the pin that triggered the interrupt event
 * 
 *  Returns:
 *    void
 */
void MotorEncoder::enc_hardware_irq_trigger(uint pin)
{
    if (pin == channel_a_pin)
    {
        enc_last_pulse_time = enc_current_pulse_time;
        enc_current_pulse_time = time_us_32();;
        enc_pulses ++;

        // Set the measured direction to BACKWARD IF channel_b_pin is logic HIGH OR IF channel_b_pin is logic LOW but encoder direction is REVERSED.
        if (is_dual_channel_encoder && (gpio_get(channel_b_pin) || (!gpio_get(channel_b_pin) && direction_reversed)))
        {
            measured_direction = enc_direction::BACKWARD;
        }

        else
        {
            measured_direction = enc_direction::FORWARD;
        }
    }
}


/*  This function should be called in a timer interrupt callback function.
 *  How often this function is called determines the sample time of method 2 RPM measurements.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorEncoder::enc_timer_irq_trigger()
{
    enc_pulse_time = time_us_32() - enc_last_pulse_rst;
    enc_time_per_rot = (enc_pulse_time / enc_pulses) * encoder_pulses_per_rotation;

    if (enc_pulses > 0)
    {
        enc_pulses = 0;
        enc_last_pulse_rst = time_us_32();
    }
}


/*  Calculates and returns the motor's measured RPM using method 1.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    float: measured RPM
 */
float MotorEncoder::get_m1_rpm()
{
    double tor_ms = (motor_gear_ratio * ((enc_current_pulse_time - enc_last_pulse_time) * (encoder_pulses_per_rotation))) / 1000;
    float rpm = 60000 / tor_ms;

    return truncate_adj(rpm, 2);  // Reduce number of decimal points using truncate_adj.
}


/*  Calculates and returns the motor's measured RPM using method 2.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    float: measured RPM
 */
float MotorEncoder::get_m2_rpm()
{
    double tor_ms = (motor_gear_ratio * enc_time_per_rot) / 1000;
    float rpm = 60000 / tor_ms;

    return truncate_adj(rpm, 2);  // Reduce number of decimal points using truncate_adj.
}


/*  Calculates and returns the motor's measured RPM using both methods.
 *  More about how this implementation works in ../../README.md
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    float: measured RPM
 */
float MotorEncoder::get_rpm()
{
    float rpm_m2 = MotorEncoder::get_m2_rpm();
    float final_reading = MotorEncoder::get_m1_rpm();

    if (rpm_m2 < method_1_cutoff_rpm)
    {
        final_reading = rpm_m2;
    }

    if (final_reading < rpm_zero_threshold)
    {
        final_reading = 0.0f;
    }

    return final_reading;
}


/*  Returns measured motor rotation direction.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    MotorEncoder::enc_direction: measured direction
 */
MotorEncoder::enc_direction MotorEncoder::get_direction()
{
    return measured_direction;
}