/*
    Motor control library (motor driver interface) - Written for the ROS Robot Project
    This submodule provides a standard interface for using motor drivers and handles any
    driver-specific output requirements.

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


#include "pico/stdlib.h"
#include "../Helper_lib/Helpers.h"
#include "Motor_Driver.h"


/*  Constructor
 *  
 *  Arguments:
 *    uint *driver_pins: an unsigned integer array containing all necessary driver pins. (DRIVER-SPECIFIC)
 *    int number_of_pins: the number of pins that have been passed in the array.
 *    driver_type drv_type: type of driver used (e.g. GENERIC_PWM for L298, L293, etc.)
 */
MotorDriver::MotorDriver(uint drv_pins[], int number_of_pins, driver_type drv_type)
{
    MotorDriver::disable();
    MotorDriver::set_input_limits(input_limit_min, input_limit_max);
    MotorDriver::set_direction(direction::FORWARD);

    if (number_of_pins <= max_number_of_driver_pins)
    {
        // Copies all of drv_pins's items over to driver_pins.
        for (int i = 0; i < number_of_pins; i++)
        {
            driver_pins[i] = drv_pins[i];
        }

        if (drv_type == driver_type::GENERIC_PWM && number_of_pins == GENERIC_PWM_NUM_PINS)
        {
            MotorDriver::init_driver();
            MotorDriver::enable();
        }
    }
}


// --------- Private functions ---------

/*  Initializes the driver.
 *  This function handles driver-specific initialization requirements.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorDriver::init_driver()
{
    switch (drv_type)
    {
        /*  Initialize Generic PWM driver (e.g. L298, L293, etc.)
         *  
         *  Pins:
         *    driver_pins[0]: driver's A channel
         *    driver_pins[1]: driver's B channel
         * 
         *  Supported features:
         *    PWM control         : Yes
         *    Digital control     : No
         *    Current sensing     : No
         *    Enable pin          : No
         *    Fault reporting pin : No
         */
        case driver_type::GENERIC_PWM:
            init_pin(driver_pins[0], OUTPUT_PWM);
            init_pin(driver_pins[1], OUTPUT_PWM);
            gpio_put_pwm(driver_pins[0], 0);
            gpio_put_pwm(driver_pins[1], 0);
            break;
    }
}


/*  Sets driver outputs according to speed, direction, and driver_enabled variables.
 *  This function handles driver-specific output requirements.
 *
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorDriver::set_driver_outputs()
{
    switch (drv_type)
    {
        // Sets Generic PWM driver outputs
        case driver_type::GENERIC_PWM:
            int mapped_input = 0;

            if (driver_enabled)
            {
                mapped_input = map(input, input_limits[0], input_limits[1], 0, 65535);
            }

            if (input_direction == direction::FORWARD)
            {
                gpio_put_pwm(driver_pins[0], mapped_input);
                gpio_put_pwm(driver_pins[1], 0);
            }

            else
            {
                gpio_put_pwm(driver_pins[0], 0);
                gpio_put_pwm(driver_pins[1], mapped_input);
            }

            break;
    }
}


/*  Sets input limits variable.
 *
 *  Arguments:
 *    int min: minimum input limit to be set
 *    int max: maximum input limit to be set
 * 
 *  Returns:
 *    void
 */
void MotorDriver::set_input_limits(int min, int max)
{
    input_limits[0] = min;
    input_limits[1] = max;
}


// --------- Public functions ---------

/*  Set driver speed/output.
 *  
 *  Arguments:
 *    int speed: driver speed/output (must be within input_limits)
 * 
 *  Returns:
 *    void
 */
void MotorDriver::set_speed(int speed)
{
    input = speed;
    MotorDriver::set_driver_outputs();
}


/*  Set driver direction.
 *  
 *  Arguments:
 *    MotorDriver::direction dir: driver direction (FORWARD/BACKWARD)
 * 
 *  Returns:
 *    void
 */
void MotorDriver::set_direction(direction dir)
{
    input_direction = dir;
    MotorDriver::set_driver_outputs();
}


/*  Enable driver.
 *  Sets driver enable pin to 1 on drivers that support it, and
 *  sets driver output to input.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorDriver::enable()
{
    driver_enabled = true;
    MotorDriver::set_driver_outputs();
}


/*  Disable driver.
 *  Sets driver enable pin to 0 on drivers that support it, and
 *  prevents driver output/speed from increasing (keeps them at 0) 
 *  until the driver is enabled.
 *  
 *  Arguments:
 *    None
 * 
 *  Returns:
 *    void
 */
void MotorDriver::disable()
{
    driver_enabled = false;
    MotorDriver::set_driver_outputs();
}


// ------ Public status functions ------

int MotorDriver::get_speed()                         { return input; }            // Returns current set driver output/speed.
MotorDriver::direction MotorDriver::get_direction()  { return input_direction; }  // Returns current driver direction
bool MotorDriver::is_enabled()                       { return driver_enabled; }   // Returns true if the driver is enabled, false if it is not.
int MotorDriver::get_input_limit_min()               { return input_limits[0]; }  // Returns minimum set speed input limit.
int MotorDriver::get_input_limit_max()               { return input_limits[1]; }  // Returns maximum set speed input limit.