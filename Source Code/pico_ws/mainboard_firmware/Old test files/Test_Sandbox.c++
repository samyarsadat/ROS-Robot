// Testing sandbox
// Written by Samyar Sadat Akhavi, 2023-2024
// Motor_lib testing


// ------- Libraries & Modules -------
#include <stdio.h>
#include "hardware/adc.h"
#include "lib/Helper_lib/Helpers.h"
#include "pico/stdlib.h"
#include "lib/Helper_lib/Helpers.h"
#include "lib/Motor_lib/Motor.h"
#include "lib/Motor_lib/Motor_Safety.h"


// ------- Global variables -------
struct repeating_timer timer_callback_rt;
struct repeating_timer fast_timer_callback_rt;


// ------- Definitions -------
#define enc_pulses_per_rotation     2
#define motor_gear_ratio            80/1
#define sample_time                 120
#define fast_sample_time            80
#define speed_update_time_ms        100

#define l_motor_1_enc_b  8
#define l_motor_2_enc_b  7
#define l_motor_1_enc_a  10
#define l_motor_2_enc_a  9

#define l_motor_1_drive  19
#define l_motor_2_drive  18

#define mux_add_0  3
#define mux_add_1  2
#define mux_add_2  4
#define mux_add_3  5
#define mux_io     26


// ---- PID init ----
uint pins[2] = {l_motor_1_drive, l_motor_2_drive};
MotorDriver driver(pins, 2, MotorDriver::driver_type::GENERIC_PWM);

void irq_call(uint pin, uint32_t events);

MotorEncoder enca(l_motor_1_enc_a, l_motor_2_enc_a, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder encb(l_motor_1_enc_b, l_motor_2_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder* encs[] = {&enca, &encb};

Motor motor(&driver, encs, 2);
MotorSafety motor_safety(&motor, 0);


void irq_call(uint pin, uint32_t events)
{
    enca.enc_hardware_irq_trigger(pin);
    encb.enc_hardware_irq_trigger(pin);
}


bool timer_callback(struct repeating_timer *rt)
{
    enca.enc_timer_irq_trigger();
    encb.enc_timer_irq_trigger();
    motor_safety.safety_check_timer_callback();
    return true;
}


void log_data();


bool fast_timer_callback(struct repeating_timer *rt)
{
    log_data();
    motor.compute_outputs();
    return true;
}


void ramp_test(float max_speed, Motor::motor_direction direction, int duration_ms);


void get_run_test_request()
{
    int test = 0;
    printf("time,rpm_a,rpm_b,dir_a,dir_b,rpm_avg,pid_out,target_rpm,target_dir\n");

    add_repeating_timer_ms(sample_time, timer_callback, NULL, &timer_callback_rt);
    add_repeating_timer_ms(fast_sample_time, fast_timer_callback, NULL, &fast_timer_callback_rt);

    switch (test)
    {
        // Speed ramp test
        case 0:
            for (int i = 0; i < 50; i++)
            {
                Motor::motor_direction dir;

                if (i % 2) {
                    dir = Motor::motor_direction::FORWARD;
                } else {
                    dir = Motor::motor_direction::BACKWARD;
                }

                ramp_test(120, dir, 15000);
            }

            break;
        
        default:
            printf("UI:ERROR-TEST-NOT-FOUND\n");
            break;
    }

    cancel_repeating_timer(&timer_callback_rt);
    cancel_repeating_timer(&fast_timer_callback_rt);
}


void safety_call_func(MotorSafety::safety_trigger_conditions condition, int id)
{
    switch (condition)
    {
        case MotorSafety::safety_trigger_conditions::ENC_DIFF_EXCEED:
            printf("SAFETY_TRIGGER:ENC_DIFF_EXCEED\n");
            break;

        case MotorSafety::safety_trigger_conditions::ENC_DIR_DIFF_DET:
            printf("SAFETY_TRIGGER:ENC_DIR_DIFF_DET\n");
            break;

        case MotorSafety::safety_trigger_conditions::SET_VS_ACTUAL_SPD_EXCEED:
            printf("SAFETY_TRIGGER:SET_VS_ACTUAL_SPD_EXCEED\n");
            break;
        
        default:
            printf("SAFETY_TRIGGER:NO_CONDITION\n");
            break;
    }
}


void log_data()
{
    bool dira, dirb, dirt;

    if (enca.get_direction() == MotorEncoder::enc_direction::FORWARD) {
        dira = true;
    } else {
        dira = false;
    }

    if (encb.get_direction() == MotorEncoder::enc_direction::FORWARD) {
        dirb = true;
    } else {
        dirb = false;
    }

    if (motor.get_set_motor_direction() == Motor::motor_direction::FORWARD) {
        dirt = true;
    } else {
        dirt = false;
    }

    char msg[250];
    sprintf(msg, "%f,%f,%s,%s,%f,%d,%f,%s\n", enca.get_rpm(), encb.get_rpm(), dira ? "50":"0", dirb ? "50":"0", motor.get_avg_rpm(), driver.get_speed(), motor.get_pid_ctrl_speed(), dirt ? "50":"0");
    printf(msg);
}


// TESTS
void ramp_test(float max_speed, Motor::motor_direction direction, int duration_ms)
{
    int loop_count = duration_ms / speed_update_time_ms;
    float speed_divided = max_speed / loop_count;
    motor.set_motor_direction(direction);

    for (int i = 0; i <= loop_count; i++)
    {
        motor.set_pid_ctrl_speed(speed_divided * i);
        sleep_ms(speed_update_time_ms);
    }

    sleep_ms(2000);

    for (int i = loop_count; i >= 0; i--)
    {
        motor.set_pid_ctrl_speed(speed_divided * i);
        sleep_ms(speed_update_time_ms);
    }

    sleep_ms(2000);
}


void setup()
{
    init_pin(mux_add_0, OUTPUT);
    init_pin(mux_add_1, OUTPUT);
    init_pin(mux_add_2, OUTPUT);
    init_pin(mux_add_3, OUTPUT);

    adc_init();
    init_pin(mux_io, INPUT_ADC);

    stdio_init_all();

    enca.set_enc_direction_reversed(true);
    encb.set_enc_direction_reversed(true);
    motor.set_control_mode(Motor::control_mode::PID);
    motor.enable_controller();
    motor.set_direction_reversed(true);
    motor.set_motor_direction(Motor::motor_direction::BACKWARD);
    motor_safety.configure_safety(20, 70, 4000, &safety_call_func);
    motor_safety.enable_safety();

    while (!stdio_usb_connected());
    sleep_ms(1000);

    get_run_test_request();
}


void loop()
{
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