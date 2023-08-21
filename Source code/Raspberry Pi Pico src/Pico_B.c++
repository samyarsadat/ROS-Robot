/*
   The ROS robot project - Raspberry Pi Pico (B) program
   Copyright 2022-2023 Samyar Sadat Akhavi
   Written by Samyar Sadat Akhavi, 2022-2023.
 
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
#include <std_msgs/msg/string.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "lib/helpers.h"
#include "hardware/pwm.h"


// ------- Global variables -------

// ---- Micro ROS ----
rcl_allocator_t rc_alloc;
rclc_support_t rc_supp;
rcl_node_t rc_node;
rclc_executor_t executor;

// ---- Micro ROS subscribers and publishers ----
rcl_publisher_t e_stop_pub;
std_msgs__msg__String e_stop_msg;

// ---- Misc. ----
bool core_1_continue = false;
bool core_1_setup_start = false;
bool halt_core_0 = false;

// ---- Loop time ----
uint32_t loop_time, loop_1_time;


// ------- Pin defines -------

// ---- Misc. ----
#define onboard_led  25
#define ready_sig    2

// ---- Settings switches ----
#define speed_sw_1  16
#define speed_sw_2  17
#define mode_sw     22

// ---- Raspberry Pi camera LEDs ----
#define cam_led_1  18
#define cam_led_2  19
#define cam_led_3  20
#define cam_led_4  21

// ---- Micro switches ----
#define ms_front_l  13
#define ms_front_r  12
#define ms_back_l   11
#define ms_back_r   10


// ------- Other defines -------

// ---- Misc. ----
#define loop_timeout    60000    // In microseconds
#define loop_1_timeout  200000   // In microseconds


// ------- Functions -------

// ---- RCL return checker prototype ----
void check_rc(rcl_ret_t rctc);


// ---- Error handler ----
void handle_error(int core, const char * err_msg)
{
    if (core_1_continue)
    {
        sprintf(e_stop_msg.data.data, "E_STOP (PICO_B): %s", err_msg);
        e_stop_msg.data.size = strlen(e_stop_msg.data.data);
        check_rc(rcl_publish(&e_stop_pub, &e_stop_msg, NULL));
    }

    if (core == 0)
    {
        multicore_reset_core1();

        gpio_put(onboard_led, LOW);
        gpio_put(ready_sig, LOW);
        gpio_put(cam_led_1, LOW);
        gpio_put(cam_led_2, LOW);
        gpio_put(cam_led_3, LOW);
        gpio_put(cam_led_4, LOW);

        while (true)
        {
            gpio_put(onboard_led, LOW);
            sleep_ms(100);
            gpio_put(onboard_led, HIGH);
            sleep_ms(100);
        }
    }

    else
    {
        halt_core_0 = true;

        gpio_put(onboard_led, LOW);
        gpio_put(ready_sig, LOW);
        gpio_put(cam_led_1, LOW);
        gpio_put(cam_led_2, LOW);
        gpio_put(cam_led_3, LOW);
        gpio_put(cam_led_4, LOW);

        while (true)
        {
            gpio_put(onboard_led, LOW);
            sleep_ms(100);
            gpio_put(onboard_led, HIGH);
            sleep_ms(100);
        }
    }
}


// ---- Pin init ----
void init_pins()
{
    // ---- Inputs ----
    init_pin(speed_sw_1, INPUT_PULLUP);
    init_pin(speed_sw_2, INPUT_PULLUP);
    init_pin(mode_sw, INPUT_PULLUP);
    init_pin(ms_front_l, INPUT_PULLUP);
    init_pin(ms_front_r, INPUT_PULLUP);
    init_pin(ms_back_l, INPUT_PULLUP);
    init_pin(ms_back_r, INPUT_PULLUP);

    // ---- Outputs ----
    init_pin(onboard_led, OUTPUT);
    init_pin(ready_sig, OUTPUT);
    init_pin(cam_led_1, OUTPUT);
    init_pin(cam_led_2, OUTPUT);
    init_pin(cam_led_3, OUTPUT);
    init_pin(cam_led_4, OUTPUT);
}


// ---- RCL return checker ----
void check_rc(rcl_ret_t rctc)
{
    if (rctc != RCL_RET_OK)
    {
        handle_error(0, "RC Check failed");
    }
}


// ---- Setup ROS subscribers and publishers ----
void init_subs_pubs()
{
    const rosidl_message_type_support_t * string_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

    // ---- E-stop topic ----
    check_rc(rclc_publisher_init_default(&e_stop_pub, &rc_node, string_type, "/e_stop"));
    std_msgs__msg__String__init(&e_stop_msg);
}


// ---- Micro ROS executor init ----
void exec_init()
{
    executor = rclc_executor_get_zero_initialized_executor();
    unsigned int num_handles = 0;

    check_rc(rclc_executor_init(&executor, &rc_supp.context, num_handles, &rc_alloc));
}


// ------- Main program -------

// ---- Setup function (Runs once on core 0) ----
void setup()
{
    init_subs_pubs();
    exec_init();
}


// ---- Setup function (Runs once on core 1) ----
void setup1()
{

}


// ---- Main loop (Runs forever on core 0) ----
void loop()
{
    loop_time = time_us_32();

    gpio_put(onboard_led, HIGH);
    sleep_ms(1000);
    gpio_put(onboard_led, LOW);
    sleep_ms(1000);

    if (time_us_32() - loop_time > loop_timeout)
    {
        handle_error(0, "Loop timeout exceeded");
    }
}


// ---- Main loop (Runs forever on core 1) ----
void loop1()
{
    loop_1_time = time_us_32();

    gpio_put(ready_sig, HIGH);
    sleep_ms(500);
    gpio_put(ready_sig, LOW);
    sleep_ms(500);

    if (time_us_32() - loop_1_time > loop_1_timeout)
    {
        handle_error(1, "Loop 1 timeout exceeded");
    }
}


// ------- Init -------

// ---- Core 1 loop init ----
void init_core_1()
{
    multicore_fifo_push_blocking(FLAG);
    uint32_t flag_r = multicore_fifo_pop_blocking();
    
    if (flag_r != FLAG)
    {
        init_pins();
        handle_error(1, "Core 1 FIFO receive failure");
    }

    else
    {
        // Wait for core_1_setup_start to be set to true
        while (!core_1_setup_start);

        setup1();

        // Wait for core_1_continue to be set to true
        while (!core_1_continue);

        while (true)
        {
            loop1();
        }
    }
}


// ---- Micro ROS node init ----
void uros_init(const char * node_name, const char * name_space)
{
    rc_alloc = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp, 0, NULL, &rc_alloc));
    check_rc(rclc_node_init_default(&rc_node, node_name, name_space, &rc_supp));
}


// ---- Startup function ----
void startup()
{
    init_pins();

    uros_init("pico_b", "io");

    setup();
    core_1_setup_start = true;

    // Send ready signal to Pico (A)
    gpio_put(ready_sig, HIGH);
    sleep_ms(2000);
    gpio_put(ready_sig, LOW);
    init_pin(ready_sig, INPUT);

    // Wait for Pico (A) ready signal
    while (!gpio_get(ready_sig))
    {
        gpio_put(onboard_led, HIGH);
        sleep_ms(400);
        gpio_put(onboard_led, LOW);
        sleep_ms(400);
    }

    // Start the Micro ROS executor
    rclc_executor_spin(&executor);

    // Micro ROS cleanup
    check_rc(rcl_publisher_fini(&e_stop_pub, &rc_node));
    check_rc(rcl_node_fini(&rc_node));

    core_1_continue = true;
}


// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    multicore_reset_core1();
    multicore_launch_core1(init_core_1);

    multicore_fifo_push_blocking(FLAG);
    uint32_t flag_r = multicore_fifo_pop_blocking();
    
    if (flag_r != FLAG)
    {
        init_pins();
        handle_error(0, "Core 0 FIFO receive failure");
    }

    else
    {
        startup();

        // Start core 0 loop
        while (!halt_core_0)
        {
            loop();
        }
    }
}