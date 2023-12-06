/*
    Motor control library (motor driver interface) - Written for the ROS Robot Project
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

#include "pico/stdlib.h"


// ---- Static config ----
#define max_number_of_driver_pins  2
#define input_limit_max            5200
#define input_limit_min            0

// Driver pin counts
#define GENERIC_PWM_NUM_PINS  2


// ---- Main Driver object ----
class MotorDriver
{
    public:
        enum driver_type
        {
            GENERIC_PWM = 0   // Generic, 2-pin PWM motor driver (e.g. L298, L293)
        };

        enum direction
        {
            FORWARD = 1,
            BACKWARD = 0
        };

        // Constructor
        MotorDriver(uint drv_pins[], int number_of_pins, driver_type drv_type);


        // ---- Functions ----

        // Set driver output/speed (input must be within input_limits).
        void set_speed(int speed);

        // Set driver direction.
        void set_direction(direction dir);

        // Enable driver.
        // Sets driver enable pin to 1 on drivers that support it, and
        // sets driver output to input.
        void enable();

        // Disable driver.
        // Sets driver enable pin to 0 on drivers that support it, and
        // prevents driver output/speed from increasing (keeps them at 0) 
        // until the driver is enabled.
        void disable();

        // Returns current set driver output/speed.
        int get_speed();

        // Returns current driver direction.
        direction get_direction();

        // Returns true if the driver is enabled,
        // false if it is not.
        bool is_enabled();

        // Returns minimum set speed input limit.
        int get_input_limit_min();

        // Returns maximum set speed input limit.
        int get_input_limit_max();


    private:
        uint driver_pins[max_number_of_driver_pins];
        driver_type drv_type;
        
        int input_limits[2];
        int input;                   // Currently set driver output/speed
        direction input_direction;   // Currently set driver direction
        bool driver_enabled;


        // ---- Internal Functions ----

        // Initializes the driver.
        // This function handles driver-specific initialization requirements.
        void init_driver();

        // Sets driver outputs according to speed and direction variables.
        // This function handles driver-specific output requirements.
        void set_driver_outputs();

        // Sets input limits variable.
        void set_input_limits(int min, int max);

        // TODO: Implement current sesning and fault pins for drivers that support them.
};