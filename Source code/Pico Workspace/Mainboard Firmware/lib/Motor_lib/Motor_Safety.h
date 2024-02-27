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


#pragma once

#include "Motor_Encoder.h"   // Motor encoder interface
#include "Motor.h"           // Motor controller
#include "pico/stdlib.h"


// ---- Main Motor Safety object ----
class MotorSafety
{
    public:
        // Constructor
        MotorSafety(Motor* controller, int safety_object_id);

        // Safety trigger conditions that will be passed to the trigger callback function.
        enum safety_trigger_conditions
        {
            ENC_DIFF_EXCEED = 0,
            SET_VS_ACTUAL_SPD_EXCEED = 1,
            ENC_DIR_DIFF_DET = 2,
            SET_VS_ACTUAL_DIR_DIFF = 3
        };

        // Safety trigger function callback type definition.
        typedef void (*safety_trigger_callback)(safety_trigger_conditions condition, int id);


        // ---- Functions ----
        
        // Set initial configuration for the module.
        void configure_safety(int enc_max_deviation, int set_vs_actual_max_deviation, int check_fail_trig_timeout, safety_trigger_callback trig_callback);
        
        // Enable safety monitoring.
        void enable_safety();

        // Disable safety monitoring.
        void disable_safety();

        // Configure whether encoder speed difference monitoring
        // should be enabled.
        void encoder_diff_check_enabled(bool is_enabled);

        // Configure whether actual vs. set speed
        // monitoring should be enabled.
        void set_vs_actual_spd_diff_check_enabled(bool is_enabled);

        // Configure whether encoder direction difference
        // monitoring should be enabled.
        void encoder_dir_diff_check_enabled(bool is_enabled);

        // Configure whether actual vs. set direction
        // monitoring should be enabled.
        void set_vs_actual_dir_check_enabled(bool is_enabled);

        // Set actual vs. set speed monitoring tolerance.
        void set_set_vs_actual_spd_tolerance(int tolerance);

        // Set actual vs. set speed monitoring extra timeout tolerance.
        void set_set_vs_actual_spd_time_tolerance(int milliseconds);

        // Set encoder difference speed monitoring tolerance.
        void set_enc_diff_tolerance(int tolerance);

        // Set the check fail trigger timeout.
        // (The amount of time for which a check has to keep failing until
        // the trigger function is called)
        void set_fail_trigger_timeout(int timeout);

        // Run the safety monitoring checks.
        // The safety monitoring conditions are only checked
        // when this function is called.
        void safety_check_timer_callback();
        

    private:
        Motor* controller;
        MotorEncoder* encoders[max_number_of_encoders];
        int number_of_encoders_defined;

        int id;   // The ID of this monitoring object. This is passed to the trigger_callback so that the callback can 
                  // identify which object is triggered if the same callback is used for multiple monitoring objects.
        bool safety_configured;
        bool safety_enabled;
        bool enc_diff_check_enabled;
        bool set_actual_spd_diff_check_enabled;
        bool set_actual_direction_check_enabled;
        bool enc_dir_diff_check_enabled;
        safety_trigger_callback trigger_callback;
        int enc_diff_trigger_tolerance;
        int set_actual_spd_diff_tolerance;
        int checks_fail_trigger_timout_ms;
        uint32_t last_enc_diff_trigger;
        uint32_t last_set_speed_trigger;
        uint32_t last_enc_dir_diff_trigger;
        uint32_t last_set_direction_trigger;
        int set_vs_actual_spd_check_timeout_extra_ms;   // How many milliseconds should be added to checks_fail_trigger_timout_ms when checking for set vs. actual speed difference. 
                                                        // Essentially how much extra tolerance should be allowed for this check.


        // ---- Internal Functions ----

        // Checks the measured speed difference between all of the defined
        // encoders and returns false if the difference between the largest
        // and smallest reading exceeds the configured tolerance.
        bool check_encoder_difference();

        // Checks the set PID target speed vs. the measured speed
        // and returns false if the difference exceeds
        // the configured tolerance.
        bool check_set_vs_actual_speed_difference();

        // Checks the measured direction of all of the defined encoders
        // and returns false if any of the encoders are spinning in the
        // opposite direction compared to the other ones.
        bool check_encoder_dir_difference();

        // Checks the set target direction vs. the measured
        // direction and returns false if they are different.
        bool check_set_vs_actual_direction();
};