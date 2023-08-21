/*
   The ROS robot project - Encoder system test program (Implementation A (Comb. M1, M2) - 0 RPM det., stable RPM readings, PID control)
   Copyright 2022-2023 Samyar Sadat Akhavi
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
#include "hardware/pwm.h"
#include "lib/helpers.h"
#include "lib/PID_lib/PID_v1.h"


// ------- Global variables -------
char msg[50];
struct repeating_timer m2_calc_rt;

volatile int pulses = 0;
volatile uint32_t pulses_time, last_pulses_reset, time_per_rotation;

volatile uint32_t enc_time = 10, enc_time_old = 10;
float final_reading, rpm_m2, tor_ms_m2, rpm_m1, tor_ms_m1;


// ------- Defines -------
#define motor_enc_a              12
#define motor_drive_ra           19
#define motor_drive_rb           18
#define motor_drive_la           21
#define motor_drive_lb           20
#define enc_pulses_per_rotation  2
#define motor_gear_ratio         80/1
#define sample_time              50

#define l_motor_1_enc_b  7
#define l_motor_1_enc_a  9
#define l_motor_2_enc_b  8
#define l_motor_2_enc_a  11
#define r_motor_1_enc_b  22
#define r_motor_1_enc_a  10
#define r_motor_2_enc_b  27

// ---- Right motor PID ----
float motor_r_spd = 80, motor_r_set;
float rm_kp=20, rm_ki=0, rm_kd=0;

// ---- Motor PWM slice ----
uint r_motor_slice_num;


// ------- PID init -------
PID r_motors(&final_reading, &motor_r_set, &motor_r_spd, rm_kp, rm_ki, rm_kd, DIRECT);


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
	init_pin(l_motor_1_enc_b, INPUT);
	init_pin(l_motor_1_enc_a, INPUT);
	init_pin(l_motor_2_enc_b, INPUT);
	init_pin(l_motor_2_enc_a, INPUT);
	init_pin(r_motor_1_enc_b, INPUT);
	init_pin(r_motor_1_enc_a, INPUT);
	init_pin(r_motor_2_enc_b, INPUT);

	init_pin(motor_enc_a, INPUT);
	gpio_set_irq_enabled_with_callback(motor_enc_a, GPIO_IRQ_EDGE_RISE, true, &irq_call);

	// Activate motor at full speed
	init_pin(motor_drive_ra, OUTPUT_PWM);
	init_pin(motor_drive_rb, OUTPUT_PWM);
	init_pin(motor_drive_la, OUTPUT_PWM);
	init_pin(motor_drive_lb, OUTPUT_PWM);

	r_motor_slice_num = pwm_gpio_to_slice_num(motor_drive_la);
	pwm_set_enabled(r_motor_slice_num, true);

	stdio_init_all();
	add_repeating_timer_ms(sample_time, calc_rpm_m2, NULL, &m2_calc_rt);
	
	r_motors.SetMode(AUTOMATIC);
	r_motors.SetSampleTime(80);
}

// ---- Set right motor PWM output ----
void set_r_motor_output(int spd)
{
	if (spd >= 0)
	{
		pwm_set_chan_level(r_motor_slice_num, PWM_CHAN_B, spd);
		pwm_set_chan_level(r_motor_slice_num, PWM_CHAN_A, 0);
	}

	else
	{
		pwm_set_chan_level(r_motor_slice_num, PWM_CHAN_B, 0);
		pwm_set_chan_level(r_motor_slice_num, PWM_CHAN_A, spd);  
	}
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

	if (rpm_m1 < 20)
	{
		final_reading = rpm_m2;
	}

	if (final_reading < 0.5)
	{
		final_reading = 0;
	}

	r_motors.Compute();
	set_r_motor_output(motor_r_set);

	// Display measurements
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