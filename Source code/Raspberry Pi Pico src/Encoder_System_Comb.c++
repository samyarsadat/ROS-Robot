/*
   The ROS robot project - Encoder system test program (Method 3 (Comb. M1, M2) - 0 RPM det., stable RPM readings)
   Copyright 2022 Samyar Sadat Akhavi
   Written by Samyar Sadat Akhavi, 2022.
 
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


// ------- Libraries & Modules -------
#include <stdio.h>
#include "pico/stdlib.h"
#include "lib/helpers.h"


// ------- Global variables -------
char msg[50];
struct repeating_timer m2_calc_rt;

volatile int pulses = 0;
volatile uint32_t pulses_time, last_pulses_reset, time_per_rotation;

volatile uint32_t enc_time = 10, enc_time_old = 10;
float final_reading, rpm_m2, tor_ms_m2, rpm_m1, tor_ms_m1;


// ------- Defines -------
#define motor_enc_a              18
#define enc_pulses_per_rotation  2
#define motor_gear_ratio         80/1
#define sample_time              50


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    enc_time_old = enc_time;
    enc_time = time_us_32();
	pulses ++;
}


// ---- Method 2 RPM calc function ----
bool calc_rpm_m2(struct repeating_timer *rt)
{
	pulses_time = time_us_32() - last_pulses_reset;
    time_per_rotation = (pulses_time / pulses) * enc_pulses_per_rotation;

    if (pulses > 0)
    {
        pulses = 0;
        last_pulses_reset = time_us_32();
    }

	return true;
}


// ------- Main program -------
void setup()
{
    init_pin(motor_enc_a, INPUT);
    gpio_set_irq_enabled_with_callback(motor_enc_a, GPIO_IRQ_EDGE_RISE, true, &irq_call);

    stdio_init_all();
    //
    add_repeating_timer_ms(sample_time, calc_rpm_m2, NULL, &m2_calc_rt);
}


void loop()
{
	// Method 1 RPM calc
	tor_ms_m1 = (motor_gear_ratio * ((enc_time - enc_time_old) * (enc_pulses_per_rotation))) / 1000;
    rpm_m1 = 60000 / tor_ms_m1;

    // Method 2 RPM calc
    tor_ms_m2 = (motor_gear_ratio * time_per_rotation) / 1000;
    rpm_m2 = 60000 / tor_ms_m2;

	// Reduce accuracy
	rpm_m1 = truncate_adj(rpm_m1, 2);
	rpm_m2 = truncate_adj(rpm_m2, 2);

	// Use readings from both methods to determine RPM
    final_reading = rpm_m1;

	if (rpm_m1 < 18)
	{
		final_reading = rpm_m2;
	}

	// Display measurements
    sprintf(msg, "Motor RPM M1: %.2f - ", rpm_m1);
    printf(msg);
    
    sprintf(msg, "Motor RPM M2: %.2f - ", rpm_m2);
    printf(msg);
    
    sprintf(msg, "Final Reading: %.2f\n", final_reading);
    printf(msg);

	sleep_ms(80);
}


int main() 
{
    setup();

    while (true)
    {
        loop();
    }
}