/*
    The ROS robot project - Raspberry Pi Pico (A) program
    Copyright 2022-2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2022-2024.
 
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
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rrp_pico_coms/srv/set_camera_leds.h>
#include "pico_b_helpers/Self_Test.c++"
#include "pico_b_helpers/Sensor_Publishers.c++"
#include "pico_b_helpers/uROS_Init.h"
#include "pico_b_helpers/Definitions.h"
#include "pico_b_helpers/Local_Helpers.h"
#include "pico_b_helpers/IO_Helpers_General.h"
#include "uart_transport/pico_uart_transports.h"
#include <rmw_microros/rmw_microros.h>
#include <iterator>



// ------- Global variables -------

// ---- Misc. ----
bool halt_core_0 = false;
bool self_test_mode = false;
alarm_pool_t *core_1_alarm_pool;

// ---- Timer execution times storage (milliseconds) ----
uint32_t last_microsw_publish_time, last_other_sensors_publish_time;

// ---- Timers ----
struct repeating_timer microsw_publish_rt, other_sensors_publish_rt;



// ------- Functions ------- 

// ---- Graceful shutdown ----
void clean_shutdown()
{
    // Stop all repeating timers
    cancel_repeating_timer(&microsw_publish_rt);
    cancel_repeating_timer(&other_sensors_publish_rt);


    // IO cleanup
    set_camera_leds(0, 0, 0, 0);

    // MicroROS cleanup
    // TODO: ADD ALL PUBS AND SUBS.
    check_rc(rcl_publisher_fini(&diagnostics_pub, &rc_node), RT_LOG_ONLY_CHECK);
    check_rc(rclc_executor_fini(&rc_executor), RT_LOG_ONLY_CHECK);
    check_rc(rcl_node_fini(&rc_node), RT_LOG_ONLY_CHECK);


    // Stop core 0 (only effective when this function is called from core 1)
    halt_core_0 = true;

    // Stop core 1 if this function is being called from core 0.
    if (get_core_num() == 0)
    {
        multicore_reset_core1();
    }

    while (true)
    {
        gpio_put_pwm(cam_led_1, 0);
        gpio_put(onboard_led, LOW);
        sleep_ms(100);
        gpio_put_pwm(cam_led_1, 65535);
        gpio_put(onboard_led, HIGH);
        sleep_ms(100);
    }
}


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    if (pin == ms_front_r || pin == ms_front_l || pin == ms_back_r || pin == ms_back_l)
    {
        // Publish microswitch states if they are triggered, regardless of the timer.
        publish_microsw_sens(NULL);   
    }
}



// ------- MicroROS subscriber & service callbacks ------- 

// ---- Set camera LED outputs ----
void en_camera_leds_callback(const void *req, void *res) 
{
    rrp_pico_coms__srv__SetCameraLeds_Request *req_in = (rrp_pico_coms__srv__SetCameraLeds_Request *) req;
    rrp_pico_coms__srv__SetCameraLeds_Response *res_in = (rrp_pico_coms__srv__SetCameraLeds_Response *) res;

    set_camera_leds(req_in->led_outputs[0], req_in->led_outputs[1], req_in->led_outputs[2], req_in->led_outputs[3]);
    res_in->success = true;
}



// ------- Main program -------

// ---- Setup function (Runs once on core 0) ----
void setup()
{
    // ---- Pin init ----
    init_pin(ready_sig, INPUT);
    init_pin(batt_adc, INPUT_ADC);
    init_pin(ms_front_r, INPUT_PULLUP);
    init_pin(ms_front_l, INPUT_PULLUP);
    init_pin(ms_back_r, INPUT_PULLUP);
    init_pin(ms_back_l, INPUT_PULLUP);
    init_pin(speed_sw_1, INPUT_PULLUP);
    init_pin(speed_sw_2, INPUT_PULLUP);
    init_pin(mode_sw, INPUT_PULLUP);
    init_pin(onboard_led, OUTPUT);
    init_pin(cam_led_1, OUTPUT_PWM);
    init_pin(cam_led_2, OUTPUT_PWM);
    init_pin(cam_led_3, OUTPUT_PWM);
    init_pin(cam_led_4, OUTPUT_PWM);

    // Microswitch interrupts
    gpio_set_irq_enabled_with_callback(ms_front_r, GPIO_IRQ_EDGE_FALL, true, irq_call);
    gpio_set_irq_enabled(ms_front_l, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ms_back_r, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ms_back_l, GPIO_IRQ_EDGE_FALL, true);

    // Misc. init
    adc_init();
    adc_set_temp_sensor_enabled(true);

    // MicroROS init
    init_subs_pubs();
    exec_init();
    uros_init(UROS_NODE_NAME, UROS_NODE_NAMESPACE);
    rclc_executor_spin(&rc_executor);

    // Repeating timers
    add_repeating_timer_ms(microsw_pub_rt_interval, publish_microsw_sens, NULL, &microsw_publish_rt);
}


// ---- Setup function (Runs once on core 1) ----
void setup1()
{
    // Create alarm pool for core 1 timers
    core_1_alarm_pool = alarm_pool_create(4, 3);

    // Repeating timers
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sensors_pub_rt_interval, publish_misc_sens, NULL, &other_sensors_publish_rt);
}



// ******** END OF MAIN PROGRAM *********
// *********** STARTUP & INIT ***********

// ---- Core 1 main function ----
void main_core_1()
{
    setup1();

    // Core 1 loop
    while (true);   // All core 1 functions run on timers. Nothing needs to be run in a loop.
}


// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    multicore_reset_core1();

    // Wait for 5 seconds
    // TODO: Should wait for signal from Raspberry Pi computer.
    for (int i = 0; i < 5; i++)
    {
        gpio_put_pwm(cam_led_1, 65535);
        gpio_put(onboard_led, HIGH);
        sleep_ms(500);
        gpio_put_pwm(cam_led_1, 0);
        gpio_put(onboard_led, LOW);
        sleep_ms(500);
    }

    setup();
    multicore_launch_core1(main_core_1);

    //gpio_put(power_led, HIGH);

    // Core 0 loop
    while (!halt_core_0);   // All core 0 functions run on timers. Nothing needs to be run in a loop.

    return 0;
}