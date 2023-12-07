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

#define l_motor_1_enc_a  7
#define l_motor_2_enc_a  9
#define l_motor_1_enc_b  8
#define l_motor_2_enc_b  10

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
    enca.enc_timer_irq_trigger();
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

    get_set_rpm_targets();
    motor.set_control_mode(Motor::control_mode::PID);
    motor.enable_controller();
}


void loop()
{
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