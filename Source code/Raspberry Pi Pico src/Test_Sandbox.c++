// Testing sandbox (UNTESTED)
// Written by Samyar Sadat Akhavi, 2023
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


// ------- Definitions -------
#define enc_pulses_per_rotation     2
#define motor_gear_ratio            80/1
#define sample_time                 100

#define l_motor_1_enc_a  22
#define l_motor_2_enc_a  27
#define l_motor_1_enc_b  11
#define l_motor_2_enc_b  12

#define l_motor_1_drive  21
#define l_motor_2_drive  20

#define mux_add_0  3
#define mux_add_1  2
#define mux_add_2  4
#define mux_add_3  5
#define mux_io     26


// ---- PID init ----
uint pins[2] = {l_motor_1_drive, l_motor_2_drive};
MotorDriver driver(pins, 2, MotorDriver::driver_type::GENERIC_PWM);

void irq_call(uint pin, uint32_t events);

MotorEncoder enca(l_motor_1_enc_a, l_motor_1_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder encb(l_motor_2_enc_a, l_motor_2_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder* encs[] = {&enca, &encb};

Motor motor(&driver, encs, 2);
MotorSafety motor_safety(&motor, encs, 2, 0);


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


void get_set_rpm_targets()
{
    char msg[50];
    float target_rpm;

    scanf("%f", &target_rpm);
    sprintf(msg, "Target RPM: %f\n", target_rpm);
    printf(msg);

    motor.set_pid_ctrl_speed(target_rpm);
    motor.set_pwm_ctrl_speed(3000);
}


void safety_call_func(MotorSafety::safety_trigger_conditions condition, int id)
{
    switch (condition)
    {
        case MotorSafety::safety_trigger_conditions::ENC_DIFF_EXCEED:
            printf("ENC_DIFF_EXCEED\n");
            break;

        case MotorSafety::safety_trigger_conditions::ENC_DIR_DIFF_DET:
            printf("ENC_DIR_DIFF_DET\n");
            break;

        case MotorSafety::safety_trigger_conditions::SET_VS_ACTUAL_SPD_EXCEED:
            printf("SET_VS_ACTUAL_SPD_EXCEED\n");
            break;
        
        default:
            printf("NO_CONDITION\n");
            break;
    }
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
    add_repeating_timer_ms(sample_time, timer_callback, NULL, &timer_callback_rt);

    motor.set_control_mode(Motor::control_mode::PID);
    motor.enable_controller();
    motor.set_direction_reversed(true);
    motor.set_motor_direction(Motor::motor_direction::BACKWARD);
    get_set_rpm_targets();
    motor_safety.configure_safety(20, 70, 4000, &safety_call_func);
    motor_safety.enable_safety();
}


void loop()
{
    bool dira, dirb;

    if (enca.get_direction() == MotorEncoder::enc_direction::FORWARD)
    {
        dira = true;
    }

    else
    {
        dira = false;
    }

    if (encb.get_direction() == MotorEncoder::enc_direction::FORWARD)
    {
        dirb = true;
    }

    else
    {
        dirb = false;
    }

    char msg[120];
    sprintf(msg, "RPM 1: %f, RPM 2: %f, DIR 1: %s, DIR 2: %s, RPM AVG: %f\n", enca.get_rpm(), encb.get_rpm(), dira ? "FW":"BW", dirb ? "FW":"BW", motor.get_avg_rpm());
    printf(msg);

    motor.compute_outputs();
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