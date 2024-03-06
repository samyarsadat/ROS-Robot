/*
    Motor control library - Written for the ROS Robot Project
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


#include "Motor_Driver.h"    // Motor driver interface
#include "Motor_Encoder.h"   // Motor encoder interface
#include "pico/stdlib.h"
#include <memory>
#include "../PID_lib/PID_v1.h"
#include "../Helper_lib/Helpers.h"
#include "Motor.h"


/*  Constructor
 *  
 *  Arguments:
 *    MotorDriver* drv: a pointer to the motor driver object that is to be used by this controller.
 *    MotorEncoder* encs[]: an array containing pointers to all of the MotorEncoder objects that are to be used by this controller.
 *    int number_of_encoders: the number of MotorEncoder object pointers that have been passed in the encs array.
 */
Motor::Motor(MotorDriver* drv, MotorEncoder* encs[], int number_of_encoders)
{
    Motor::set_control_mode(control_mode::BLIND);
    Motor::set_motor_direction(motor_direction::FORWARD);

    if (number_of_encoders <= max_number_of_encoders)
    {
        driver = drv;
        number_of_encoders_defined = number_of_encoders;

        // Copies all of encs' items over to encoders.
        for (int i = 0; i < number_of_encoders; i++)
        {
            encoders[i] = encs[i];
        }

        pid = std::make_unique<PID_v1_h::PID>(&average_rpm, &pid_output, &set_speed_pid, default_pid_Kp, default_pid_Ki, default_pid_Kd, DIRECT);
        pid->SetOutputLimits(driver->get_input_limit_min(), driver->get_input_limit_max());
        pid->SetSampleTime(default_pid_sample_time_ms);
        pid->SetMode(AUTOMATIC);

        // Disabled by default to prevent erroneous motor control output until
        // the program is ready and explicitly enables the controller.
        Motor::disable_controller();
    }
}


// --------- Public functions ---------

/*  Sets the control mode of the controller.
 *  
 *  Arguments:
 *    control_mode mode: the control mode that should be used. (PID/Blind PWM)
 * 
 *  Returns:
 *    void
 */
void Motor::set_control_mode(control_mode mode)
{
    ctrl_mode = mode;
}


/*  Sets the target speed for the PID controller.
 *  Only used when the controller is in PID mode.
 *  
 *  Arguments:
 *    float rpm: target RPM.
 * 
 *  Returns:
 *    void
 */
void Motor::set_pid_ctrl_speed(float rpm)
{
    set_speed_pid = rpm;
}


/*  Sets the PWM output for when the controller is in PWM (blind) mode.
 *  The requested PWM value must be within the driver's input_limits.
 *  
 *  Arguments:
 *    int pwm_out: requested PWM output. (Must be within driver's input_limits)
 * 
 *  Returns:
 *    void
 */
void Motor::set_pwm_ctrl_speed(int pwm_out)
{
    set_speed = pwm_out;
}


/*  Sets the motor's rotation direction.
 *  This is not verified (open-loop control), neither in PID or PWM mode.
 *  
 *  Arguments:
 *    motor_direction dir: requested direction.
 * 
 *  Returns:
 *    void
 */
void Motor::set_motor_direction(motor_direction dir)
{
    dir_change_required = true;
    motor_dir = dir;
}


/*  Configures whether the rotation direction of the motor should be reversed.
 *  This is useful for when the wiring of the driver is backwards, for example.
 *  
 *  Arguments:
 *    bool reversed: true to reverse, false to not reverse.
 * 
 *  Returns:
 *    void
 */
void Motor::set_direction_reversed(bool reversed)
{
    dir_change_required = true;
    dir_reversed = reversed;
}


/*  Enables the controller and the driver.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void Motor::enable_controller()
{
    controller_enabled = true;
    driver->enable();
}


/*  Disables the controller, disables the driver, and sets both PWM and PID target speeds to 0.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void Motor::disable_controller()
{
    controller_enabled = false;
    driver->disable();
    Motor::set_pid_ctrl_speed(0);
    Motor::set_pwm_ctrl_speed(0);
}


/*  Returns the average RPM of all of the defined encoders.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    float: average RPM
 */
float Motor::get_avg_rpm()
{
    float measured_speeds[number_of_encoders_defined];

    for (int i = 0; i < number_of_encoders_defined; i++)
    {
        measured_speeds[i] = encoders[i]->get_rpm();
    }

    return calculate_mean(measured_speeds, number_of_encoders_defined);
}


/*  Returns the averaged encoder pulse count of all of the defined encoders.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    int32_t: average encoder pulse count
 */
int32_t Motor::get_avg_enc_pulse_count()
{
    int64_t pulse_counters_total;

    for (int i = 0; i < number_of_encoders_defined; i++)
    {
        pulse_counters_total += encoders[i]->get_pulse_counter();
    }

    return pulse_counters_total / number_of_encoders_defined;
}


/*  Returns the set RPM target for PID control.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    float: set PID target RPM
 */
float Motor::get_pid_ctrl_speed()
{
    return set_speed_pid;
}


/*  Returns the set PWM output for blind control.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    int: set PWM output
 */
int Motor::get_pwm_ctrl_speed()
{
    return set_speed;
}


/*  Returns the set motor rotation direction.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    motor_direction: set rotation direction
 */
Motor::motor_direction Motor::get_set_motor_direction()
{
    return motor_dir;
}


/*  Returns the set control mode of the controller.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    control_mode: set control mode
 */
Motor::control_mode Motor::get_control_mode()
{
    return ctrl_mode;
}


/*  Returns whether motor control direction
 *  is reversed or not.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    bool: true for reversed, false for not reversed
 */
bool Motor::get_dir_reversed()
{
    return dir_reversed;
}


/*  Calculates and sets motor outputs depending on the control mode and set direction.
 *  How often this function is called determines how often the driver outputs are set,
 *  and how often the PID Compute() function is called.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void Motor::compute_outputs()
{
    if (controller_enabled)
    {
        // Motor direction
        if (dir_change_required)
        {
            if ((motor_dir == motor_direction::FORWARD && !dir_reversed) || (motor_dir == motor_direction::BACKWARD && dir_reversed))
            {
                driver->set_direction(MotorDriver::direction::FORWARD);
            }

            else
            {
                driver->set_direction(MotorDriver::direction::BACKWARD);
            }

            dir_change_required = false;
        }

        // Motor speed
        if (ctrl_mode == control_mode::PID)
        {
            if (set_speed_pid != 0)
            {
                average_rpm = Motor::get_avg_rpm();
                pid->Compute();
                driver->set_speed((int) pid_output);
            }

            else
            {
                driver->set_speed(0);
            }
        }

        else if (ctrl_mode == control_mode::BLIND)
        {
            driver->set_speed(set_speed);
        }
    }
}


/*  Returns the encoder object pointers array.
 *  This is used by the MotorSafety module.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    MotorEncoder**: encoder object pointers array
 */
MotorEncoder** Motor::get_encs_array()
{
    return encoders;
}


/*  Returns the number of defined encoders. 
 *  (i.e. number of encoder object pointers in the encoders array)
 *  This is used by the MotorSafety module.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    int: number of defined encoders
 */
int Motor::get_num_defined_encs()
{
    return number_of_encoders_defined;
}