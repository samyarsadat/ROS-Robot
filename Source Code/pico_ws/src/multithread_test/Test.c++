#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <FreeRTOS_POSIX.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "helpers_lib/Helpers.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "freertos_helpers_lib/uROS_Bridge_Agent.h"
#include "freertos_helpers_lib/RTOS_Agent.h"
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <iterator>
#include <vector>
#include <cmath>
#include <task.h>


uRosBridgeAgent *bridge_instance;
rcl_publisher_t pub0, pub1, diagnostics_pub;
rcl_subscription_t sub;
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;
std_msgs__msg__Int32 pub0_msg, pub1_msg, sub_msg;
const rosidl_message_type_support_t *int32_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
struct repeating_timer rt_0;
int32_t pub0_data = 0, pub1_data = 0;
TimerHandle_t rtos_timer;


void clean_shutdown()
{
    write_log(__FUNCTION__, "Shutting down...", LOG_LVL_INFO);
    bridge_instance->uros_fini();
    vTaskEndScheduler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    std::string task_name_str(pcTaskName, strlen(pcTaskName));
    write_log(__FUNCTION__, "PANIC: Stack overflow. Task: " + task_name_str, LOG_LVL_FATAL);
    panic("Stack overflow. Task: %s\n", pcTaskName);
}

void vApplicationMallocFailedHook()
{
    write_log(__FUNCTION__, "PANIC: malloc failed", LOG_LVL_FATAL);
    panic("malloc failed");
}

void sub_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    write_log(__FUNCTION__, "Received: " + std::to_string(msg->data), LOG_LVL_INFO);
}

void task0(void *pvParameters)
{
    while (1)
    {
        write_log(__FUNCTION__, "Task 0 executing!", LOG_LVL_INFO);
        pub0_msg.data = pub0_data;
        check_rc(rcl_publish(&pub0, &pub0_msg, NULL), RT_SOFT_CHECK);
        pub0_data++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool timer0(struct repeating_timer *rt)
{
    BaseType_t task_woken = taskENTER_CRITICAL_FROM_ISR();
    write_log(__FUNCTION__, "Timer 0 (Hardware) executing!", LOG_LVL_INFO);
    pub0_msg.data = pub0_data;
    check_rc(rcl_publish(&pub0, &pub0_msg, NULL), RT_SOFT_CHECK);
    pub0_data++;
    taskEXIT_CRITICAL_FROM_ISR(task_woken);
    portYIELD_FROM_ISR(task_woken);
    return true;
}

void timer1(TimerHandle_t xTimer)
{
    write_log(__FUNCTION__, "Timer 1 (FreeRTOS) executing!", LOG_LVL_INFO);
    pub1_msg.data = pub1_data;
    check_rc(rcl_publish(&pub1, &pub1_msg, NULL), RT_SOFT_CHECK);
    pub1_data++;
}

bool uros_init()
{
    write_log(__FUNCTION__, "MicroROS init...", LOG_LVL_INFO);

    bridge_instance->uros_init_node("uros_bridge_agent", "io");
    bridge_instance->init_publisher(&pub0, int32_type, "pub0");
    bridge_instance->init_publisher(&pub1, int32_type, "pub1");
    bridge_instance->init_publisher(&diagnostics_pub, diag_status_type, "diagnostics");
    bridge_instance->init_subscriber(&sub, int32_type, "sub");
    bridge_instance->uros_init_executor();
    bridge_instance->add_subscriber(&sub, &sub_msg, &sub_callback);

    /*if (xTimerStart(rtos_timer, 0) != pdPASS)
    {
        write_log(__FUNCTION__, "Failed to start the FreeRTOS timer.", LOG_LVL_ERROR);
    }*/

    //add_repeating_timer_ms(1000, timer0, NULL, &rt_0);

    write_log(__FUNCTION__, "MicroROS init done.", LOG_LVL_INFO);
    return true;
}

int main()
{
    stdio_init_all();
    stdio_filter_driver(&stdio_usb);   // Filter the output of STDIO to USB.
    write_log(__FUNCTION__, "STDIO init, program starting...", LOG_LVL_INFO);

    bridge_instance = uRosBridgeAgent::get_instance();
    bridge_instance->pre_init(uros_init);
    
    write_log(__FUNCTION__, "xTimerCreate...", LOG_LVL_INFO);
    rtos_timer = xTimerCreate("pub1_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *) 0, timer1);
    
    check_bool(bridge_instance->start(configMAX_PRIORITIES - 3), RT_HARD_CHECK);
    //xTaskCreate(task0, "task0", 1024, NULL, configMAX_PRIORITIES - 2, NULL);

    write_log(__FUNCTION__, "vTaskStartScheduler...", LOG_LVL_INFO);
    vTaskStartScheduler();
    write_log(__FUNCTION__, "Failed to start the FreeRTOS scheduler.", LOG_LVL_FATAL);
    return 0;
}