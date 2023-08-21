/*
   The ROS robot project - Encoder system test program (Implementation A Motor (Comb. Met_1, Met_2) - 0 RPM det., stable RPM readings, all four motors, motor control)
   Copyright 2022-2023 Samyar Sadat Akhavi
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


// ------- Libraries & Modules -------
#include <stdio.h>
#include "pico/stdlib.h"
#include "lib/helpers.h"
#include "hardware/pwm.h"


// ------- Global variables -------
struct repeating_timer method_2_tpr_calc_rt;
uint r_motor_pwm_slice, l_motor_pwm_slice;

int pulses_l1, pulses_l2, pulses_r1, pulses_r2;
uint32_t pulse_l1_time, pulse_l2_time, pulse_r1_time, pulse_r2_time;
uint32_t last_pulse_l1_rst, last_pulse_l2_rst, last_pulse_r1_rst, last_pulse_r2_rst;
uint32_t time_per_rot_l1, time_per_rot_l2, time_per_rot_r1, time_per_rot_r2;

uint32_t enc_time_l1, enc_time_l2, enc_time_r1, enc_time_r2; 
uint32_t enc_time_l1_old, enc_time_l2_old, enc_time_r1_old, enc_time_r2_old;


// ------- Definitions -------
#define RIGHT_MOTOR  1
#define LEFT_MOTOR   2
#define BOTH_MOTORS  3

#define enc_pulses_per_rotation     2
#define motor_gear_ratio            80/1
#define sample_time                 100
#define method_1_cutoff_rpm         16
#define emer_brake_rev_duration_ms  150  

#define l_motor_1_enc_a  7
#define l_motor_2_enc_a  9
#define r_motor_1_enc_a  22
#define r_motor_2_enc_a  27

#define l_motor_1_drive  19
#define l_motor_2_drive  18
#define r_motor_1_drive  21
#define r_motor_2_drive  20


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    uint32_t current_time = time_us_32();

    switch (pin)
    {
        case l_motor_1_enc_a:
            enc_time_l1_old = enc_time_l1;
            enc_time_l1 = current_time;
            pulses_l1 ++;
            break;

        case l_motor_2_enc_a:
            enc_time_l2_old = enc_time_l2;
            enc_time_l2 = current_time;
            pulses_l2 ++;
            break;

        case r_motor_1_enc_a:
            enc_time_r1_old = enc_time_r1;
            enc_time_r1 = current_time;
            pulses_r1 ++;
            break;

        case r_motor_2_enc_a:
            enc_time_r2_old = enc_time_r2;
            enc_time_r2 = current_time;
            pulses_r2 ++;
            break;
    }
}


// ---- Method 2 RPM calc function ----
void method_2_tpr_calc(uint32_t* pulse_time, uint32_t* last_pulse_rst, uint32_t* time_per_rot, int* pulses)
{
    uint32_t current_time = time_us_32();
    
    *pulse_time = current_time - *last_pulse_rst;
    *time_per_rot = (*pulse_time / *pulses) * enc_pulses_per_rotation;

    if (*pulses > 0)
    {
        *pulses = 0;
        *last_pulse_rst = current_time;
    }
}


// ---- Calculate all motor RPMs using the method 2 calc function ----
bool method_2_all_tpr_calc(struct repeating_timer *rt)
{
    method_2_tpr_calc(&pulse_l1_time, &last_pulse_l1_rst, &time_per_rot_l1, &pulses_l1);
    method_2_tpr_calc(&pulse_l2_time, &last_pulse_l2_rst, &time_per_rot_l2, &pulses_l2);
    method_2_tpr_calc(&pulse_r1_time, &last_pulse_r1_rst, &time_per_rot_r1, &pulses_r1);
    method_2_tpr_calc(&pulse_r2_time, &last_pulse_r2_rst, &time_per_rot_r2, &pulses_r2);

    return true;
}


// ---- Set motor pwm output using pwm slice ----
void set_motor_output_slice(uint slice, int spd)
{
    if (spd >= 0)
    {
        pwm_set_chan_level(slice, PWM_CHAN_B, spd);
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    }

    else
    {
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
        pwm_set_chan_level(slice, PWM_CHAN_A, abs(spd));  
    }
}


// ---- Set motor PWM outputs ----
void set_motor_outputs(char side, int spd)
{
    switch (side)
    {
        case 1:
            set_motor_output_slice(r_motor_pwm_slice, spd);
            break;

        case 2:
            set_motor_output_slice(l_motor_pwm_slice, spd);
            break;

        case 3:
            set_motor_output_slice(r_motor_pwm_slice, spd);
            set_motor_output_slice(l_motor_pwm_slice, spd);
            break;
    }
}


// ---- Get motor speeds from user and set the motor speeds accordingly ----
void get_set_motor_speeds()
{
    int spd_left;
    int spd_right;

    scanf("%d", &spd_left);
    scanf("%d", &spd_right);

    char msg[50];
    sprintf(msg, "Right Motor: %d - Left Motor: %d\n", spd_right, spd_left);
    printf(msg);

    set_motor_outputs(LEFT_MOTOR, spd_left);
    set_motor_outputs(RIGHT_MOTOR, spd_right);
}


// ------- Main program -------
void setup()
{
    // Encoder inputs
    init_pin(l_motor_1_enc_a, INPUT);
    init_pin(l_motor_2_enc_a, INPUT);
    init_pin(r_motor_1_enc_a, INPUT);
    init_pin(r_motor_2_enc_a, INPUT);

    gpio_set_irq_enabled_with_callback(l_motor_1_enc_a, GPIO_IRQ_EDGE_FALL, true, &irq_call);
    gpio_set_irq_enabled(l_motor_2_enc_a, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(r_motor_1_enc_a, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(r_motor_2_enc_a, GPIO_IRQ_EDGE_FALL, true);

    // Motor outputs
    init_pin(r_motor_1_drive, OUTPUT_PWM);
    init_pin(r_motor_2_drive, OUTPUT_PWM);
    init_pin(l_motor_1_drive, OUTPUT_PWM);
    init_pin(l_motor_2_drive, OUTPUT_PWM);

    r_motor_pwm_slice = pwm_gpio_to_slice_num(r_motor_1_drive);
    l_motor_pwm_slice = pwm_gpio_to_slice_num(l_motor_1_drive);
    pwm_set_enabled(r_motor_pwm_slice, true);
    pwm_set_enabled(l_motor_pwm_slice, true);

    stdio_init_all();
    add_repeating_timer_ms(sample_time, method_2_all_tpr_calc, NULL, &method_2_tpr_calc_rt);

    set_motor_outputs(BOTH_MOTORS, 0);
    get_set_motor_speeds();
}


// ---- Final motor RPM calc ----
float motor_rpm_calc(uint32_t m1_enc_time, uint32_t m1_enc_time_old, uint32_t m2_time_per_rot)
{
    // Method 1 RPM calc
    double tor_ms_m1 = (motor_gear_ratio * ((m1_enc_time - m1_enc_time_old) * (enc_pulses_per_rotation))) / 1000;
    float rpm_m1 = 60000 / tor_ms_m1;

    // Method 2 RPM calc
    double tor_ms_m2 = (motor_gear_ratio * m2_time_per_rot) / 1000;
    float rpm_m2 = 60000 / tor_ms_m2;

    // Reduce accuracy
    rpm_m1 = truncate_adj(rpm_m1, 2);
    rpm_m2 = truncate_adj(rpm_m2, 2);

    // Use readings from both methods to determine RPM
    float final_reading = rpm_m1;

    if (rpm_m2 < method_1_cutoff_rpm)
    {
        final_reading = rpm_m2;
    } 
    

    if (final_reading < 0.5) 
    { 
        final_reading = 0;
    }

    return final_reading;
}


void loop()
{
    // Calculate RPMs
    float l1_rpm = motor_rpm_calc(enc_time_l1, enc_time_l1_old, time_per_rot_l1);
    float l2_rpm = motor_rpm_calc(enc_time_l2, enc_time_l2_old, time_per_rot_l2);
    float r1_rpm = motor_rpm_calc(enc_time_r1, enc_time_r1_old, time_per_rot_r1);
    float r2_rpm = motor_rpm_calc(enc_time_r2, enc_time_r2_old, time_per_rot_r2);

    // Display calculations
    char msg[50];
    sprintf(msg, "L1M: %.2f - L2M: %.2f - R1M: %.2f - R2M: %.2f\n", l1_rpm, l2_rpm, r1_rpm, r2_rpm);
    printf(msg);

    sleep_ms(100);
}


int main()
{
    setup();

    while (true)
    {
        loop();
    }
}