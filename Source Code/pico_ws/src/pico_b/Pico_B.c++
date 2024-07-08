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
#include "helpers/Self_Test.c++"
#include "helpers/Sensor_Publishers.c++"
#include "freertos_helpers_lib/uROS_Bridge_Agent.h"
#include "freertos_helpers_lib/uROS_Publishing_Handler.h"
#include "helpers/uROS_Init.h"
#include "helpers/Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "helpers/IO_Helpers_General.h"
#include <rmw_microros/rmw_microros.h>
#include <iterator>
#include <task.h>



// ------- Global variables -------

// ---- Misc. ----
alarm_pool_t *core_1_alarm_pool;

// ---- Timer execution times storage (milliseconds) ----
uint32_t last_microsw_publish_time, last_other_sensors_publish_time;

// ---- Timers ----
struct repeating_timer microsw_publish_rt, other_sensors_publish_rt;
TaskHandle_t microsw_publish_th, other_sensors_publish_th;



// ------- Library object inits -------

// ---- MicroROS ----
uRosBridgeAgent *bridge;
uRosPublishingHandler *pub_handler;



// ------- Functions ------- 

// ---- Graceful shutdown ----
void clean_shutdown()
{
    write_log("A clean shutdown has been triggered. The program will now shut down.", LOG_LVL_FATAL, FUNCNAME_ONLY);

    // Stop all repeating timers
    cancel_repeating_timer(&microsw_publish_rt);
    cancel_repeating_timer(&other_sensors_publish_rt);

    // IO cleanup
    set_camera_leds(0, 0, 0, 0);
    init_pin(onboard_led, OUTPUT);
    gpio_put(onboard_led, LOW);

    // MicroROS cleanup
    bridge->uros_fini();

    // Suspend the FreeRTOS scheduler
    // No FreeRTOS API calls beyond this point!
    vTaskSuspendAll();

    while (true)
    {
        gpio_put(onboard_led, LOW);
        sleep_ms(100);
        gpio_put(onboard_led, HIGH);
        sleep_ms(100);
    }
}


// ---- FreeRTOS task stack overflow hook ----
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    write_log("Stack overflow! Task: " + std::string(pcTaskName, strlen(pcTaskName)), LOG_LVL_FATAL, FUNCNAME_ONLY);
    clean_shutdown();
}


// ---- FreeRTOS memory allocation failure hook ----
void vApplicationMallocFailedHook()
{
    // Not calling clean_shutdown() because it calls write_log() which uses malloc.
    // This shouldn't ever happen anyway, so it doesn't matter.
    panic("Memory allocation failed!");
}


// ---- Timer callbacks for task notification ----
bool publish_microsw_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(microsw_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_misc_sens_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(other_sensors_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}


// ---- IRQ callback ----
void irq_call(uint pin, uint32_t events)
{
    if (pin == ms_front_r || pin == ms_front_l || pin == ms_back_r || pin == ms_back_l)
    {
        // Publish microswitch states if they are triggered, regardless of the timer.
        publish_microsw_notify(NULL);   
    }
}



// ------- MicroROS subscriber & service callbacks ------- 

// ---- Set camera LED outputs ----
void en_camera_leds_callback(const void *req, void *res) 
{
    rrp_pico_coms__srv__SetCameraLeds_Request *req_in = (rrp_pico_coms__srv__SetCameraLeds_Request *) req;
    rrp_pico_coms__srv__SetCameraLeds_Response *res_in = (rrp_pico_coms__srv__SetCameraLeds_Response *) res;

    char buffer[75];
    snprintf(buffer, sizeof(buffer), "Received set_camera_leds: [c1: %u, c2: %u, c3: %u, c4: %u]", req_in->led_outputs[0], req_in->led_outputs[1], req_in->led_outputs[2], req_in->led_outputs[3]);
    write_log(buffer, LOG_LVL_INFO, FUNCNAME_ONLY);

    set_camera_leds(req_in->led_outputs[0], req_in->led_outputs[1], req_in->led_outputs[2], req_in->led_outputs[3]);
    res_in->success = true;
}



// ------- Main program -------

// ---- Setup repeating timers ----
void start_timers()
{
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, microsw_pub_rt_interval, publish_microsw_notify, NULL, &microsw_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sensors_pub_rt_interval, publish_misc_sens_notify, NULL, &other_sensors_publish_rt);
}


// ---- Setup function (Runs once on core 0) ----
void setup(void *parameters)
{
    init_print_uart_mutex();
    write_log("Core 0 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Pin init
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
    adc_init_mutex();
    adc_set_temp_sensor_enabled(true);

    // Create timer tasks
    write_log("Creating timer tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    xTaskCreate(publish_microsw_sens, "microsw_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 3, &microsw_publish_th);
    xTaskCreate(publish_misc_sens, "other_sensors_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &other_sensors_publish_th);
    vTaskCoreAffinitySet(microsw_publish_th, (1 << 1));
    vTaskCoreAffinitySet(other_sensors_publish_th, (1 << 1));

    // Start MicroROS tasks
    write_log("Starting MicroROS tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    check_bool(bridge->start(configMAX_PRIORITIES - 1, (1 << 0), true), RT_HARD_CHECK);
    check_bool(pub_handler->start(configMAX_PRIORITIES - 2, (1 << 0), true), RT_HARD_CHECK);

    // Indicate successful startup
    gpio_put(onboard_led, HIGH);

    // Delete setup task
    vTaskDelete(NULL);
}


// ---- Setup function (Runs once on core 1) ----
void setup1(void *parameters)
{
    write_log("Core 1 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Create alarm pool for core 1 timers
    write_log("Creating core 1 alarm pool...", LOG_LVL_INFO, FUNCNAME_ONLY);
    core_1_alarm_pool = alarm_pool_create(2, 8);

    // Delete setup task
    vTaskDelete(NULL);
}



// ******** END OF MAIN PROGRAM *********
// *********** STARTUP & INIT ***********

// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    // UART & USB STDIO outputs
    stdio_init_all();
    stdio_filter_driver(&stdio_usb);   // Filter the output of STDIO to USB.
    write_log("STDIO init, program starting...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);

    // Wait for 3 seconds
    init_pin(onboard_led, OUTPUT);

    for (int i = 0; i < STARTUP_WAIT_TIME_S; i++)
    {
        write_log("Startup wait " + std::to_string(i + 1) + "...", LOG_LVL_INFO, FUNCNAME_ONLY);
        gpio_put(onboard_led, HIGH);
        sleep_ms(500);
        gpio_put(onboard_led, LOW);
        sleep_ms(500);
    }

    // MicroROS pre-init
    write_log("MicroROS pre-init...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    bridge = uRosBridgeAgent::get_instance();
    pub_handler = uRosPublishingHandler::get_instance();
    bridge->pre_init(uros_init, clean_shutdown);
    pub_handler->pre_init(bridge);
    set_diag_pub_queue(pub_handler->get_queue_handle());

    // Stetup function tasks
    write_log("Creating setup tasks...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    xTaskCreateAffinitySet(setup, "setup_core_0", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 0), NULL);
    xTaskCreateAffinitySet(setup1, "setup_core_1", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 1), NULL);

    // Start FreeRTOS scheduler
    write_log("Starting FreeRTOS scheduler...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    vTaskStartScheduler();

    // We should never get to this point!
    write_log("Program exit. Scheduler start failed!", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    
    return 0;
}