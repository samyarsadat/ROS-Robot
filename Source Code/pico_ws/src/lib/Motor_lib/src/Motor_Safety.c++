/*
    Motor control library (safety monitoring module) - Written for the ROS Robot Project
    This is an external module of this library that handles safety checks.

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


#include "motor_control_lib/Motor_Safety.h"
#include "motor_control_lib/Motor_Encoder.h"   // Motor encoder interface
#include "motor_control_lib/Motor.h"           // Motor controller
#include <cstdlib>


/*  Constructor
 *  
 *  Arguments:
 *    Motor* ctrl: the motor controller object that is to be monitored.
 *    int safety_object_id: the ID of this monitoring object. This is passed to the trigger_callback so that the callback can 
 *                          identify which object is triggered if the same callback is used for multiple monitoring objects.
 */
MotorSafety::MotorSafety(Motor* ctrl, uint8_t safety_object_id)
{
    controller = ctrl;
    number_of_encoders_defined = ctrl->get_num_defined_encs();

    // Copies all of the items in the returned array from get_encs_array() to encoders.
    for (int i = 0; i < number_of_encoders_defined; i++)
    {
        encoders[i] = ctrl->get_encs_array()[i];
    }

    MotorSafety::disable_safety();
}


// --------- Public Functions ---------

/*  Sets initial configuration for the module.
 *  
 *  Arguments:
 *    int enc_max_deviation: the maximum allowed RPM deviation between all of the encoders.
 *    int set_vs_actual_max_deviation: the maximum allowed deviation between the set PID target RPM and the measured RPM (rpm_avg).
 *    int check_fail_trig_timeout: the amount of time (in milliseconds) for which a test has to fail for the callback to be called.
 *    safety_trigger_callback trig_callback: the safety trigger callback function.
 * 
 *  Returns:
 *    void
 */
void MotorSafety::configure_safety(uint16_t enc_max_deviation, uint16_t set_vs_actual_max_deviation, uint16_t check_fail_trig_timeout, safety_trigger_callback trig_callback)
{
    safety_configured = true;
    trigger_callback = trig_callback;
    MotorSafety::set_enc_diff_tolerance(enc_max_deviation);
    MotorSafety::set_set_vs_actual_spd_tolerance(set_vs_actual_max_deviation);
    MotorSafety::set_fail_trigger_timeout(check_fail_trig_timeout);
    MotorSafety::set_set_vs_actual_spd_time_tolerance(0);
    MotorSafety::encoder_dir_diff_check_enabled(true);
    MotorSafety::encoder_diff_check_enabled(true);
    MotorSafety::set_vs_actual_spd_diff_check_enabled(true);
    MotorSafety::set_vs_actual_dir_check_enabled(true);
}


/*  Enables safety monitoring.
 *  Only enables safety if the module is configured. (call configure_safety())
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorSafety::enable_safety()
{
    if (safety_configured)
    {
        safety_enabled = true;
    }
}


/*  Disables safety monitoring.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorSafety::disable_safety()
{
    safety_enabled = false;
}


/*  Configures whether the encoder speed difference check
 *  should be enabled or not.
 *  
 *  Arguments:
 *    bool enabled: true to enable, false to disable
 * 
 *  Returns:
 *    void
 */
void MotorSafety::encoder_diff_check_enabled(bool is_enabled)
{
    enc_diff_check_enabled = is_enabled;
}


/*  Configures whether the set vs. measured RPM difference 
 *  check should be enabled or not.
 *  
 *  Arguments:
 *    bool enabled: true to enable, false to disable
 * 
 *  Returns:
 *    void
 */
void MotorSafety::set_vs_actual_spd_diff_check_enabled(bool is_enabled)
{
    set_actual_spd_diff_check_enabled = is_enabled;
}


/*  Configures whether the encoder measured direction difference
 *  check should be enabled or not.
 *  
 *  Arguments:
 *    bool enabled: true to enable, false to disable
 * 
 *  Returns:
 *    void
 */
void MotorSafety::encoder_dir_diff_check_enabled(bool is_enabled)
{
    enc_dir_diff_check_enabled = is_enabled;
}


/*  Configure whether actual vs. set direction
 *  check should be enabled or not.
 *  
 *  Arguments:
 *    bool enabled: true to enable, false to disable
 * 
 *  Returns:
 *    void
 */
void MotorSafety::set_vs_actual_dir_check_enabled(bool is_enabled)
{
    set_actual_direction_check_enabled = is_enabled;
}


/*  Sets the maximum allowed RPM difference between the set
 *  and measured RPMs.
 *  
 *  Arguments:
 *    int tolerance: maximum allowed RPM difference
 * 
 *  Returns:
 *    void
 */
void MotorSafety::set_set_vs_actual_spd_tolerance(uint16_t tolerance)
{
    set_actual_spd_diff_tolerance = tolerance;
}


/*  Sets how much extra time tolerance there should be for the
 *  set vs. measured speed check.
 *
 *  The set vs. measured speed check has to fail for
 *  checks_fail_trigger_timout_ms + the amount set by this function 
 *  before the trigger callback is called.
 * 
 *  This extra tolerance only exists for this check because
 *  it can take the PID controller longer than checks_fail_trigger_timout_ms
 *  to fully speed up the motors; depending on the tunings, of course.
 *  
 *  Arguments:
 *    int milliseconds: extra time tolerance in milliseconds
 * 
 *  Returns:
 *    void
 */
void MotorSafety::set_set_vs_actual_spd_time_tolerance(uint16_t milliseconds)
{
    set_vs_actual_spd_check_timeout_extra_ms = milliseconds;
}


/*  Sets the maximum allowed RPM difference between all of
 *  the encoders' measured RPMs.
 *  
 *  Arguments:
 *    int tolerance: maximum allowed RPM difference
 * 
 *  Returns:
 *    void
 */
void MotorSafety::set_enc_diff_tolerance(uint16_t tolerance)
{
    enc_diff_trigger_tolerance = tolerance;
}


/*  Sets the amount of time (in milliseconds) for which
 *  a check has to fail before the trigger callback is called.
 *  
 *  Arguments:
 *    int timeout: timeout in milliseconds 
 * 
 *  Returns:
 *    void
 */
void MotorSafety::set_fail_trigger_timeout(uint16_t timeout)
{
    checks_fail_trigger_timout_ms = timeout;
}


/*  When called, this function calls all of the
 *  check functions and calls the trigger callback
 *  if one or more of the checks fail.
 * 
 *  How often this function is called determines
 *  how often the safety conditions are checked.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorSafety::safety_check_timer_callback()
{
    if (safety_enabled)
    {
        if (!check_encoder_difference())
        {
            trigger_callback(safety_trigger_conditions::ENC_DIFF_EXCEED, id);
        }

        if (!check_set_vs_actual_speed_difference())
        {
            trigger_callback(safety_trigger_conditions::SET_VS_ACTUAL_SPD_EXCEED, id);
        }

        if (!check_encoder_dir_difference())
        {
            trigger_callback(safety_trigger_conditions::ENC_DIR_DIFF_DET, id);
        }

        if (!check_set_vs_actual_direction())
        {
            trigger_callback(safety_trigger_conditions::SET_VS_ACTUAL_DIR_DIFF, id);
        }
    }
}


// --------- Private Functions ---------
// TODO: ADD ENC_PULSE_COUNTER DIFF. MONITORING

/*  Checks the measured speed difference between all of the defined
 *  encoders and returns false if the difference between the largest
 *  and smallest reading exceeds the configured tolerance.
 * 
 *  TODO: CHANGE FROM RPM TRIGGER TO PERCENTAGE TRIGGER
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    bool: true if the check passed, false if it failed
 */
bool MotorSafety::check_encoder_difference()
{
    if (number_of_encoders_defined > 1 && enc_diff_check_enabled)
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
                    last_enc_diff_trigger = time_us_32();
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


/*  Checks the set PID target speed vs. the measured speed
 *  and returns false if the difference exceeds
 *  the configured tolerance.
 * 
 *  TODO: CHANGE FROM RPM TRIGGER TO PERCENTAGE TRIGGER
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    bool: true if the check passed, false if it failed
 */
bool MotorSafety::check_set_vs_actual_speed_difference()
{
    if (controller->get_control_mode() == Motor::control_mode::PID && set_actual_spd_diff_check_enabled)
    {
        if (abs((int) (controller->get_pid_ctrl_speed() - controller->get_avg_rpm())) > set_actual_spd_diff_tolerance)
        {
            if (last_set_speed_trigger == 0)
            {
                last_set_speed_trigger = time_us_32();
            }

            else
            {
                if (((time_us_32() - last_set_speed_trigger) / 1000) > checks_fail_trigger_timout_ms + set_vs_actual_spd_check_timeout_extra_ms)
                {
                    last_set_speed_trigger = time_us_32();
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


/*  Checks the measured direction of all of the defined encoders
 *  and returns false if any of the encoders are spinning in the
 *  opposite direction compared to the other ones.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    bool: true if the check passed, false if it failed
 */
bool MotorSafety::check_encoder_dir_difference()
{
    if (number_of_encoders_defined > 1 && controller->get_avg_rpm() > 0 && enc_dir_diff_check_enabled)
    {
        MotorEncoder::enc_direction last_direction = encoders[0]->get_direction();

        for (int i = 1; i < number_of_encoders_defined; i++)
        {
            if (encoders[i]->get_direction() != last_direction)
            {
                if (last_enc_dir_diff_trigger == 0)
                {
                    last_enc_dir_diff_trigger = time_us_32();
                }

                else
                {
                    if (((time_us_32() - last_enc_dir_diff_trigger) / 1000) > checks_fail_trigger_timout_ms + set_vs_actual_spd_check_timeout_extra_ms)
                    {
                        last_enc_dir_diff_trigger = time_us_32();
                        return false;
                    }
                }
            }

            else
            {
                if (last_enc_dir_diff_trigger != 0)
                {
                    last_enc_dir_diff_trigger = 0;
                }
            }

            last_direction = encoders[i]->get_direction();
        }
    }

    return true;
}


/*  Checks the set target direction vs. the measured
 *  direction and returns false if they are different.
 * 
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    bool: true if the check passed, false if it failed
 */
bool MotorSafety::check_set_vs_actual_direction()
{
    volatile float avrrpm = controller->get_avg_rpm();

    if (number_of_encoders_defined > 1 && controller->get_avg_rpm() > 0 && set_actual_direction_check_enabled)
    {
        if (controller->get_set_motor_direction() != encoders[0]->get_direction())
        {
            if (last_set_direction_trigger == 0)
            {
                last_set_direction_trigger = time_us_32();
            }

            else
            {
                if (((time_us_32() - last_set_direction_trigger) / 1000) > checks_fail_trigger_timout_ms)
                {
                    last_set_direction_trigger = time_us_32();
                    return false;
                }
            }
        }

        else
        {
            if (last_set_direction_trigger != 0)
            {
                last_set_direction_trigger = 0;
            }
        }
    }

    return true;
}