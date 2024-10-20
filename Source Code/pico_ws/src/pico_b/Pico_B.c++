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
#include "hardware/adc.h"
#include "helpers/Self_Test.c++"
#include "helpers/Sensor_Publishers.c++"
#include "helpers/uROS_Init.h"
#include "timers.h"



// ------- Global variables -------

// ---- Misc. ----
alarm_pool_t *core_1_alarm_pool;

// ---- Timer execution times storage (milliseconds) ----
uint32_t last_microsw_publish_time, last_other_sensors_publish_time, last_dht_measurement_time;

// ---- Timers ----
struct repeating_timer microsw_publish_rt, other_sensors_publish_rt, take_dht_measurement_rt;
TaskHandle_t microsw_publish_th, other_sensors_publish_th, take_dht_measurement_th;
TimerHandle_t waiting_for_agent_timer;

// ---- DHT11 measurements ----
float latest_dht_temp_c, latest_dht_humidity;



// ------- Library object inits -------

// ---- MicroROS ----
uRosBridgeAgent *bridge;
uRosPublishingHandler *pub_handler;

// ---- DHT11 sensor ----
dht_t dht_env_sens;



// ------- Functions ------- 

// ---- Graceful shutdown ----
void clean_shutdown()
{
    write_log("A clean shutdown has been triggered. The program will now shut down.", LOG_LVL_FATAL, FUNCNAME_ONLY);

    // Stop all repeating timers
    cancel_repeating_timer(&take_dht_measurement_rt);
    cancel_repeating_timer(&microsw_publish_rt);
    cancel_repeating_timer(&other_sensors_publish_rt);

    // IO cleanup
    set_camera_leds(0, 0, 0, 0);
    init_pin(onboard_led, OUTPUT);
    gpio_put(onboard_led, LOW);

    // MicroROS cleanup
    pub_handler->stop();
    bridge->uros_fini();

    write_log("MicroROS cleanup completed. Suspending the scheduler...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Suspend the FreeRTOS scheduler
    // No FreeRTOS API calls beyond this point!
    vTaskSuspendAll();
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

bool take_dht_measurement_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(take_dht_measurement_th, &higher_prio_woken);
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

// ---- Take DHT11 measurements task ----
void take_dht_measurement(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_dht_measurement_time, (dht_measurement_rt_interval + 100), "Publish interval exceeded limits!", true);

        // Take measurement and update global vars
        dht_start_measurement(&dht_env_sens);
        dht_result_t result = dht_finish_measurement_blocking(&dht_env_sens, &latest_dht_humidity, &latest_dht_temp_c);

        switch (result)
        {
            case DHT_RESULT_OK:
                // NOP
                break;

            case DHT_RESULT_TIMEOUT:
                write_log("DHT11 measurement timed out!", LOG_LVL_WARN, FUNCNAME_ONLY);
                publish_diag_report(DIAG_LVL_WARN, DIAG_NAME_ENV_SENSORS, DIAG_ID_ENV_DHT11, DIAG_WARN_MSG_DHT11_MEASUREMENT_TIMEOUT, NULL);
                break;

            case DHT_RESULT_BAD_CHECKSUM:
                write_log("DHT11 measurement bad checksum!", LOG_LVL_ERROR, FUNCNAME_ONLY);
                publish_diag_report(DIAG_LVL_ERROR, DIAG_NAME_ENV_SENSORS, DIAG_ID_ENV_DHT11, DIAG_ERR_MSG_DHT11_BAD_CHECKSUM, NULL);
                break;
            
            default:
                write_log("DHT11 unspecified failure.", LOG_LVL_ERROR, FUNCNAME_ONLY);
                publish_diag_report(DIAG_LVL_ERROR, DIAG_NAME_ENV_SENSORS, DIAG_ID_ENV_DHT11, DIAG_ERR_MSG_DHT11_UNSPEC_FAIL, NULL);
                break;
        }
    }
}


// ---- MicroROS executor post-execution function ----
void uros_post_exec_call()
{
    // Cleanup for self-test diagnostics messages.
    if (!self_test_diag_data_slot_nums.empty())
    {
        for (auto slot_num : self_test_diag_data_slot_nums)
        {
            destroy_uros_diag_status_msg(slot_num);
            destroy_diag_kv_pair(slot_num);
            destroy_diag_kv_pair_refs(slot_num);
            destroy_diag_msg_object(slot_num);
            deallocate_slots(slot_num);
        }

        self_test_diag_status_reports.clear();
        self_test_diag_status_reports.shrink_to_fit();
        self_test_diag_data_slot_nums.clear();
        self_test_diag_data_slot_nums.shrink_to_fit();

        enable_diag_pub();
    }
}


// ---- Setup repeating timers ----
void start_timers()
{
    write_log("Starting hardware timers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    add_repeating_timer_ms(dht_measurement_rt_interval, take_dht_measurement_notify, NULL, &take_dht_measurement_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, microsw_pub_rt_interval, publish_microsw_notify, NULL, &microsw_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sensors_pub_rt_interval, publish_misc_sens_notify, NULL, &other_sensors_publish_rt);
}


// ---- Waiting for agent LED flash timer callback ----
void waiting_for_agent_timer_call(TimerHandle_t timer)
{
    if (bridge->get_agent_state() == uRosBridgeAgent::WAITING_FOR_AGENT)
    {
        gpio_put(onboard_led, !gpio_get_out_level(onboard_led));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_AVAILABLE)
    {
        if (xTimerGetPeriod(timer) != pdMS_TO_TICKS(AGENT_AVAIL_LED_TOGGLE_DELAY_MS))
        {
            xTimerChangePeriod(timer, AGENT_AVAIL_LED_TOGGLE_DELAY_MS, 0);
        }

        gpio_put(onboard_led, !gpio_get_out_level(onboard_led));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_CONNECTED)
    {
        gpio_put(onboard_led, HIGH);
    }

    else
    {
        gpio_put(onboard_led, LOW);
    }

    xTimerDelete(timer, 0);
}


// ---- Setup function (Runs once on core 0) ----
void setup(void *parameters)
{
    init_print_uart_mutex();
    write_log("Core 0 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Initialize some mutexes
    diag_mutex_init();

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

    // DHT11 sensor
    dht_init(&dht_env_sens, DHT11, pio0, dht11_sens, false);

    // Misc. init
    adc_init();
    adc_init_mutex();
    adc_set_temp_sensor_enabled(true);

    // Create timer tasks
    write_log("Creating timer tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    xTaskCreate(publish_microsw_sens, "microsw_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 3, &microsw_publish_th);
    xTaskCreate(publish_misc_sens, "other_sensors_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &other_sensors_publish_th);
    xTaskCreate(take_dht_measurement, "dht_measurement_updater", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 5, &take_dht_measurement_th);
    vTaskCoreAffinitySet(microsw_publish_th, (1 << 1));
    vTaskCoreAffinitySet(other_sensors_publish_th, (1 << 1));
    vTaskCoreAffinitySet(take_dht_measurement_th, (1 << 1));

    // Create FreeRTOS timers
    write_log("Creating FreeRTOS software timers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    waiting_for_agent_timer = xTimerCreate("waiting_for_agent_timer", pdMS_TO_TICKS(AGENT_WAITING_LED_TOGGLE_DELAY_MS), pdTRUE, NULL, waiting_for_agent_timer_call);

    // Start MicroROS tasks
    write_log("Starting MicroROS tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    check_bool(bridge->start(configMAX_PRIORITIES - 1, (1 << 0), true), RT_HARD_CHECK);
    check_bool(pub_handler->start(configMAX_PRIORITIES - 2, (1 << 0), true), RT_HARD_CHECK);

    // Start the waiting for MicroROS agent LED blink timer
    xTimerStart(waiting_for_agent_timer, 0);

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
    bridge->pre_init(uros_init, clean_shutdown, uros_post_exec_call);
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