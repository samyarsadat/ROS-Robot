/*
    Motor control library - Written for the ROS Robot Project
    This library handles encoder inputs from motors, and controls them with a PID controller.

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


#include "Motor_Driver.h"    // Motor driver interface
#include "Motor_Encoder.h"   // Motor encoder interface
#include "pico/stdlib.h"
#include <memory>
#include "../PID_lib/PID_v1.h"
#include "Motor.h"


// Constructor
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

        // Disabled by default to prevent erroneous motor control output until
        // the program is ready and explicitly enables the controller.
        Motor::disable_controller();
    }
}


// --------- Public functions ---------
// TODO: Comments.
void Motor::set_control_mode(control_mode mode)
{
    ctrl_mode = mode;
}

void Motor::set_pid_ctrl_speed(float rpm)
{
    set_speed_pid = rpm;
}

void Motor::set_pwm_ctrl_speed(int pwm_out)
{
    set_speed = pwm_out;
}

void Motor::set_motor_direction(motor_direction dir)
{
    motor_dir = dir;
}

void Motor::motor_emergency_brake()
{
    // TODO: Implement emergency break.
}

void Motor::enable_controller()
{
    controller_enabled = true;
    driver->enable();
}

void Motor::disable_controller()
{
    controller_enabled = false;
    driver->disable();
    Motor::set_pid_ctrl_speed(0);
    Motor::set_pwm_ctrl_speed(0);
}

float Motor::get_avg_rpm()
{
    float speed_total;

    for (int i = 0; i < number_of_encoders_defined; i++)
    {
        speed_total += encoders[i]->get_rpm();
    }

    return speed_total / number_of_encoders_defined;
}

float Motor::get_pid_ctrl_speed()
{
    return set_speed_pid;
}

int Motor::get_pwm_ctrl_speed()
{
    return set_speed;
}

Motor::control_mode Motor::get_control_mode()
{
    return ctrl_mode;
}

void Motor::compute_outputs()
{
    if (controller_enabled)
    {
        if (ctrl_mode == control_mode::PID)
        {
            if (set_speed_pid != 0)
            {
                if (motor_dir == motor_direction::FORWARD || (motor_dir == motor_direction::BACKWARD && dir_reversed))
                {
                    driver->set_direction(MotorDriver::direction::FORWARD);
                }

                else if (motor_dir == motor_direction::BACKWARD || (motor_dir == motor_direction::FORWARD && dir_reversed))
                {
                    driver->set_direction(MotorDriver::direction::BACKWARD);
                }

                average_rpm = Motor::get_avg_rpm();
                pid->Compute();
                driver->set_speed((int) pid_output);

                if (!(motor_dir == motor_direction::FORWARD && encoders[0]->get_direction() == MotorEncoder::enc_direction::FORWARD))
                {
                    dir_reversed_loop_count ++;
                }

                else if (!(motor_dir == motor_direction::BACKWARD && encoders[0]->get_direction() == MotorEncoder::enc_direction::BACKWARD))
                {
                    dir_reversed_loop_count ++;
                }

                else 
                {
                    dir_reversed_loop_count = 0;
                }

                if (dir_reversed_loop_count > max_dir_reversed_loop_count)
                {
                    if (dir_reversed)
                    {
                        dir_reversed = false;
                    }

                    else 
                    {
                        dir_reversed = true;
                    }
                }
            }

            else
            {
                driver->set_speed(0);
            }
        }

        else if (ctrl_mode == control_mode::BLIND)
        {
            if (motor_dir == motor_direction::FORWARD)
            {
                driver->set_direction(MotorDriver::direction::FORWARD);
            }

            else if (motor_dir == motor_direction::BACKWARD)
            {
                driver->set_direction(MotorDriver::direction::BACKWARD);
            }

            driver->set_speed(set_speed);
        }
    }
}