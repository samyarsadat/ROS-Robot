/*
   The ROS robot project - Encoder system test program (Implementation A Motor (Comb. Met_1, Met_2) - 0 RPM det., stable RPM readings, all four motors, motor control)
   Copyright 2022-2024 Samyar Sadat Akhavi
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
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "lib/PID_lib/PID_v1.h"
#include "lib/Helper_lib/Helpers.h"


// ------- Global variables -------
struct repeating_timer method_2_tpr_calc_rt;
uint r_motor_pwm_slice, l_motor_pwm_slice;

int pulses_l1, pulses_l2, pulses_r1, pulses_r2;
uint32_t pulse_l1_time, pulse_l2_time, pulse_r1_time, pulse_r2_time;
uint32_t last_pulse_l1_rst, last_pulse_l2_rst, last_pulse_r1_rst, last_pulse_r2_rst;
uint32_t time_per_rot_l1, time_per_rot_l2, time_per_rot_r1, time_per_rot_r2;

uint32_t enc_time_l1, enc_time_l2, enc_time_r1, enc_time_r2; 
uint32_t enc_time_l1_old, enc_time_l2_old, enc_time_r1_old, enc_time_r2_old;

float r_motors_rpm, l_motors_rpm;

float r_motor_pid_kp = 21.2f, r_motor_pid_ki = 120, r_motor_pid_kd = 0.06f;
float l_motor_pid_kp = 21.2f, l_motor_pid_ki = 120, l_motor_pid_kd = 0.06f;
float r_motor_set_rpm, r_motor_pid_out, l_motor_set_rpm, l_motor_pid_out;


// ------- Definitions -------
#define RIGHT_MOTOR  1
#define LEFT_MOTOR   2
#define BOTH_MOTORS  3

#define enc_pulses_per_rotation     2
#define motor_gear_ratio            80/1
#define sample_time                 100
#define method_1_cutoff_rpm         10
#define emer_brake_rev_duration_ms  150  

#define pid_sample_time_ms  35
#define pid_mode            AUTOMATIC

#define l_motor_1_enc_a  7
#define l_motor_2_enc_a  9
#define r_motor_1_enc_a  22
#define r_motor_2_enc_a  27

#define l_motor_1_drive  19
#define l_motor_2_drive  18
#define r_motor_1_drive  21
#define r_motor_2_drive  20

#define mux_add_0  3
#define mux_add_1  2
#define mux_add_2  4
#define mux_add_3  5
#define mux_io     26


// ---- PID init ----
PID r_motors_pid(&r_motors_rpm, &r_motor_pid_out, &r_motor_set_rpm, r_motor_pid_kp, r_motor_pid_ki, r_motor_pid_kd, DIRECT);
PID l_motors_pid(&l_motors_rpm, &l_motor_pid_out, &l_motor_set_rpm, l_motor_pid_kp, l_motor_pid_ki, l_motor_pid_kd, DIRECT);


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    switch (pin)
    {
        case l_motor_1_enc_a:
            enc_time_l1_old = enc_time_l1;
            enc_time_l1 = time_us_32();;
            pulses_l1 ++;
            break;

        case l_motor_2_enc_a:
            enc_time_l2_old = enc_time_l2;
            enc_time_l2 = time_us_32();;
            pulses_l2 ++;
            break;

        case r_motor_1_enc_a:
            enc_time_r1_old = enc_time_r1;
            enc_time_r1 = time_us_32();;
            pulses_r1 ++;
            break;

        case r_motor_2_enc_a:
            enc_time_r2_old = enc_time_r2;
            enc_time_r2 = time_us_32();;
            pulses_r2 ++;
            break;
    }
}


// ---- Method 2 RPM calc function ----
void method_2_tpr_calc(uint32_t* pulse_time, uint32_t* last_pulse_rst, uint32_t* time_per_rot, int* pulses)
{
    *pulse_time = time_us_32() - *last_pulse_rst;
    *time_per_rot = (*pulse_time / *pulses) * enc_pulses_per_rotation;

    if (*pulses > 0)
    {
        *pulses = 0;
        *last_pulse_rst = time_us_32();
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


// ---- Set multiplexer address pins according to multiplexer IO pin number ----
void set_mux_addr(uint pin)
{
    bool add_0, add_1, add_2, add_3;

    add_0 = (pin / 1) % 2;
    add_1 = (pin / 2) % 2;
    add_2 = (pin / 4) % 2;
    add_3 = (pin / 8) % 2;

    gpio_put(mux_add_0, add_0);
    gpio_put(mux_add_1, add_1);
    gpio_put(mux_add_2, add_2);
    gpio_put(mux_add_3, add_3);

    sleep_us(10);
}


// ---- Get motor speeds from user and set the pid target speed accordingly ----
void get_set_rpm_targets()
{
    scanf("%f", &r_motor_set_rpm);
    scanf("%f", &l_motor_set_rpm);

    char msg[50];
    sprintf(msg, "Right Motor: %f - Left Motor: %f\n", r_motor_set_rpm, l_motor_set_rpm);
    printf(msg);
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

    // Multiplexer pins
    init_pin(mux_add_0, OUTPUT);
    init_pin(mux_add_1, OUTPUT);
    init_pin(mux_add_2, OUTPUT);
    init_pin(mux_add_3, OUTPUT);

    adc_init();
    init_pin(mux_io, INPUT_ADC);

    stdio_init_all();
    add_repeating_timer_ms(sample_time, method_2_all_tpr_calc, NULL, &method_2_tpr_calc_rt);

    // PID setup
    r_motors_pid.SetMode(pid_mode);
    r_motors_pid.SetSampleTime(pid_sample_time_ms);
    r_motors_pid.SetOutputLimits(0, 5200);
    l_motors_pid.SetMode(pid_mode);
    l_motors_pid.SetSampleTime(pid_sample_time_ms);
    l_motors_pid.SetOutputLimits(0, 5200);

    set_motor_outputs(BOTH_MOTORS, 0);
    get_set_rpm_targets();
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


/*// ---- Get and set PID Kp, Ki, Kd, and set target values from multiplexer inputs 0-3 ----
void get_set_pid_vals()
{
    gpio_deinit(mux_io);
    init_pin(mux_io, INPUT_ADC);
    adc_select_input(0);
    
    set_mux_addr(0);
    l_motor_set_rpm = (int) map(adc_read(), 0, 4096, 0, 118);

    set_mux_addr(1);
    r_motor_set_rpm = (int) map(adc_read(), 0, 4096, 0, 118);

    set_mux_addr(3);
    float Kp = truncate_adj(map(adc_read(), 0, 4096, 0, 90), 1);

    set_mux_addr(2);
    float Ki = truncate_adj(map(adc_read(), 0, 4096, 0, 120), 1);

    set_mux_addr(1);
    float Kd = truncate_adj(map(adc_read(), 0, 4096, 0, 8), 2);

    set_mux_addr(7);
    gpio_deinit(mux_io);
    init_pin(mux_io, INPUT);

    if (gpio_get(mux_io))
    {
        r_motors_pid.SetMode(AUTOMATIC);
        l_motors_pid.SetMode(AUTOMATIC);
    } 
    
    else 
    {
        r_motors_pid.SetMode(MANUAL);
        l_motors_pid.SetMode(MANUAL);
    }

    r_motors_pid.SetTunings(Kp, Ki, Kd);
    l_motors_pid.SetTunings(Kp, Ki, Kd);
}*/


void loop()
{
    //get_set_pid_vals();

    // Calculate RPMs
    float l1_rpm = motor_rpm_calc(enc_time_l1, enc_time_l1_old, time_per_rot_l1);
    float l2_rpm = motor_rpm_calc(enc_time_l2, enc_time_l2_old, time_per_rot_l2);
    l_motors_rpm = (l1_rpm + l2_rpm) / 2;

    float r1_rpm = motor_rpm_calc(enc_time_r1, enc_time_r1_old, time_per_rot_r1);
    float r2_rpm = motor_rpm_calc(enc_time_r2, enc_time_r2_old, time_per_rot_r2);
    r_motors_rpm = (r1_rpm + r2_rpm) / 2;

    // Compute motor outputs with PID
    r_motors_pid.Compute();
    l_motors_pid.Compute();

    set_motor_outputs(RIGHT_MOTOR, (int) map(r_motor_pid_out, 0, 5200, 0, 65535));
    set_motor_outputs(LEFT_MOTOR, (int) map(l_motor_pid_out, 0, 5200, 0, 65535));

    // Display calculations
    char msg[160];
    sprintf(msg, "R1: %.2f - R2: %.2f - L1: %.2f - L2: %.2f - R Out: %.2f - L Out: %.2f - Target: %.2f - Kp: %.2f - Ki: %.2f - Kd: %.2f - Mode (0: MAN, 1: AUTO): %d\n", r1_rpm, r2_rpm, l1_rpm, l2_rpm, r_motor_pid_out, l_motor_pid_out, r_motor_set_rpm, r_motors_pid.GetKp(), r_motors_pid.GetKi(), r_motors_pid.GetKd(), r_motors_pid.GetMode());
    //sprintf(msg, "Left_Avg_RPM:%.2f,Left_Setpoint:%.2f,Left_Output:%.2f\n", l_motors_rpm, l_motor_set_rpm, map(l_motor_pid_out, 0, 5200, 0, 600));
    printf(msg);

    sleep_ms(65);
}


int main()
{
    setup();

    while (true)
    {
        loop();
    }

    return 0;
}