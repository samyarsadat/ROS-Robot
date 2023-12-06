/*
    Motor control library (safety monitoring module) - Written for the ROS Robot Project
    This is an external module of this library that handles safety checks.

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


#include "Motor_Safety.h"
#include "Motor_Encoder.h"   // Motor encoder interface
#include "Motor.h"           // Motor controller
#include "pico/stdlib.h"
#include <cstdlib>


// Constructor
MotorSafety::MotorSafety(Motor* ctrl, MotorEncoder* encs[], int number_of_encoders, int safety_object_id)
{
    controller = ctrl;
    number_of_encoders_defined = number_of_encoders;

    // Copies all of encs' items over to encoders.
    for (int i = 0; i < number_of_encoders; i++)
    {
        encoders[i] = encs[i];
    }
}


// --------- Public Functions ---------
// TODO: Comments.
void MotorSafety::configure_safety(bool enable_enc_devi_check, int enc_max_deviation, bool enable_set_vs_actual_devi_check, int set_vs_actual_max_deviation, int check_fail_trig_timeout, safety_trigger_callback trig_callback)
{
    safety_configured = true;
    enc_diff_check_enabled = enable_enc_devi_check;
    enc_diff_trigger_tolerance = enc_max_deviation;
    set_actual_spd_diff_check_enabled = enable_set_vs_actual_devi_check;
    set_actual_spd_diff_tolerance = set_vs_actual_max_deviation;
    checks_fail_trigger_timout_ms = check_fail_trig_timeout;
    trigger_callback = trig_callback;
}

void MotorSafety::enable_safety()
{
    if (safety_configured)
    {
        safety_enabled = true;
    }
}

void MotorSafety::disable_safety()
{
    safety_enabled = false;
}

void MotorSafety::encoder_diff_check_enabled(bool is_enabled)
{
    enc_diff_check_enabled = is_enabled;
}

void MotorSafety::set_vs_actual_spd_diff_check_enabled(bool is_enabled)
{
    enc_diff_trigger_tolerance = is_enabled;
}

void MotorSafety::set_set_vs_actual_spd_tolerance(int tolerance)
{
    set_actual_spd_diff_tolerance = tolerance;
}

void MotorSafety::set_enc_diff_tolerance(int tolerance)
{
    enc_diff_trigger_tolerance = tolerance;
}

void MotorSafety::set_fail_trigger_timeout(int timeout)
{
    checks_fail_trigger_timout_ms = timeout;
}

void MotorSafety::safety_check_timer_callback()
{
    if (safety_enabled)
    {
        if (!check_encoder_difference() && !check_set_vs_actual_speed_difference())
        {
            trigger_callback(safety_trigger_conditions::ENC_DIFF_EXCEED, id);
            trigger_callback(safety_trigger_conditions::SET_VS_ACTUAL_SPD_EXCEED, id);
        }

        else if (!check_encoder_difference())
        {
            trigger_callback(safety_trigger_conditions::ENC_DIFF_EXCEED, id);
        }

        else if (!check_set_vs_actual_speed_difference())
        {
            trigger_callback(safety_trigger_conditions::SET_VS_ACTUAL_SPD_EXCEED, id);
        }
    }
}


// --------- Private Functions ---------
bool MotorSafety::check_encoder_difference()
{
    if (number_of_encoders_defined > 1)
    {
        float enc_val_max = encoders[0]->get_rpm();
        float enc_val_min = encoders[0]->get_rpm();

        for (int i = 1; i < number_of_encoders_defined; i++)
        {
            float rpm = encoders[i]->get_rpm();
            
            if (rpm < enc_val_min)
            {
                enc_val_min = rpm;
            }

            else if (rpm > enc_val_max)
            {
                enc_val_max = rpm;
            }
        }

        if ((int) (enc_val_max - enc_val_min) > enc_diff_trigger_tolerance)
        {
            if (last_enc_diff_trigger == 0)
            {
                last_enc_diff_trigger = time_us_32();
            }

            else
            {
                if (((time_us_32() - last_enc_diff_trigger) / 1000) > checks_fail_trigger_timout_ms)
                {
                    return false;
                }
            }
        }

        else
        {
            if (last_enc_diff_trigger != 0)
            {
                last_enc_diff_trigger = 0;
            }
        }
    }

    return true;
}

bool MotorSafety::check_set_vs_actual_speed_difference()
{
    if (controller->get_control_mode() == Motor::control_mode::PID)
    {
        if (abs((int) (controller->get_pid_ctrl_speed() - controller->get_avg_rpm())) > set_actual_spd_diff_tolerance)
        {
            if (last_set_speed_trigger == 0)
            {
                last_set_speed_trigger = time_us_32();
            }

            else
            {
                if (((time_us_32() - last_set_speed_trigger) / 1000) > checks_fail_trigger_timout_ms)
                {
                    return false;
                }
            }
        }

        else
        {
            if (last_set_speed_trigger != 0)
            {
                last_set_speed_trigger = 0;
            }
        }
    }

    return true;
}