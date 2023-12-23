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


#pragma once

#include "Motor_Driver.h"    // Motor driver interface
#include "Motor_Encoder.h"   // Motor encoder interface
#include "../PID_lib/PID_v1.h"
#include "pico/stdlib.h"
#include <memory>


// ---- Static config ----
#define max_number_of_encoders       2

// Default tunings for Namiki 22CL-3501PG
#define default_pid_Kp              21.2f
#define default_pid_Ki              120.0f
#define default_pid_Kd              0.06f
#define default_pid_sample_time_ms  50


// ---- Main Motor object ----
class Motor
{
    public:
        enum motor_direction
        {
            FORWARD = 1,
            BACKWARD = 0
        };

        // Motor control mode (PID controlled, blind PWM control)
        enum control_mode
        {
            PID = 1,
            BLIND = 0
        };

        // PID controller. Needs to be public so that PID parameters can be adjusted externally.
        std::unique_ptr<PID_v1_h::PID> pid;

        // Constructor
        Motor(MotorDriver* drv, MotorEncoder* encs[], int number_of_encoders);


        // ---- Functions ----
        
        // Set control mode (PID / blind PWM).
        void set_control_mode(control_mode mode);

        // Set target RPM for PID control.
        void set_pid_ctrl_speed(float rpm);

        // Set PWM output for blind PWM control.
        void set_pwm_ctrl_speed(int pwm_out);

        // Set motor direction.
        // The actual direction in which the motor is 
        // spinning is only checked when in PID mode.
        void set_motor_direction(motor_direction dir);

        // Configures whether the motor control direction
        // should be reversed. Useful for when motor
        // wiring is reversed.
        void set_direction_reversed(bool reversed);

        // Enable the controller.
        // This function also calls the enable() function
        // of the motor driver.
        void enable_controller();

        // Disable the controller.
        // This function also calls the disable() function
        // of the motor driver, which stops the motors.
        void disable_controller();

        // Returns the averaged RPM of all of the defined encoders.
        float get_avg_rpm();

        // Returns the currently set PID RPM target.
        float get_pid_ctrl_speed();

        // Returns the currently set blind PWM output.
        int get_pwm_ctrl_speed();

        // Returns the currently set motor direction.
        motor_direction get_set_motor_direction();

        // Returns the current control mode.
        control_mode get_control_mode();

        // Computes and sets motor outputs based on the
        // currently set control mode.
        void compute_outputs();

        // Returns the encoder object pointers array.
        // This is used by the MotorSafety module.
        MotorEncoder** get_encs_array();

        // Returns the number of defined encoders. 
        // (i.e. number of encoder object pointers in the encoders array)
        // This is used by the MotorSafety module.
        int get_num_defined_encs();
        

    private:
        MotorDriver* driver;
        MotorEncoder* encoders[max_number_of_encoders];
        int number_of_encoders_defined;
        bool controller_enabled;

        float average_rpm;
        float set_speed_pid;
        float set_speed;
        motor_direction motor_dir;
        bool dir_change_required;
        control_mode ctrl_mode;
        float pid_output;
        bool dir_reversed;             // Whether the motor outputs should be reversed for the them to turn in the correct direction.
};