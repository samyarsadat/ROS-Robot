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
#define max_dir_reversed_loop_count  10

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

        // TODO: Comments
        void set_control_mode(control_mode mode);
        void set_pid_ctrl_speed(float rpm);
        void set_pwm_ctrl_speed(int speed);
        void set_motor_direction(motor_direction dir);
        void motor_emergency_brake();
        void enable_controller();
        void disable_controller();
        float get_avg_rpm();
        float get_pid_ctrl_speed();
        int get_pwm_ctrl_speed();
        control_mode get_control_mode();
        void compute_outputs();
        

    private:
        MotorDriver* driver;
        MotorEncoder* encoders[max_number_of_encoders];
        int number_of_encoders_defined;
        bool controller_enabled;

        float average_rpm;
        float set_speed_pid;
        float set_speed;
        motor_direction motor_dir;
        control_mode ctrl_mode;
        int pid_output;
        int dir_reversed_loop_count;
        bool dir_reversed;

        struct repeating_timer rpm_method_2_tpr_calc_rt;
        uint rpm_method_2_sample_time;
};