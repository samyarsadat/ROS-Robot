#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "helpers_lib/Helpers.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "freertos_helpers_lib/uROS_Bridge_Agent.h"
#include "freertos_helpers_lib/RTOS_Agent.h"
#include "freertos_helpers_lib/uROS_Publishing_Handler.h"
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <iterator>
#include <vector>
#include <cmath>
#include "FreeRTOS.h"
#include <task.h>
#include <timers.h>


uRosBridgeAgent *bridge_instance;
uRosPublishingHandler *publishing_handler;
rcl_timer_t rc_timer;
rcl_publisher_t pub0, pub1, diagnostics_pub;
rcl_subscription_t sub;
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;
std_msgs__msg__Int32 pub0_msg, pub1_msg, sub_msg;
const rosidl_message_type_support_t *int32_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
TimerHandle_t rtos_timer, top_timer;


void clean_shutdown()
{
    write_log("Shutting down...", LOG_LVL_INFO, FUNCNAME_ONLY);
    bridge_instance->uros_fini();
    vTaskEndScheduler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    std::string task_name_str(pcTaskName, strlen(pcTaskName));
    write_log("PANIC: Stack overflow. Task: " + task_name_str, LOG_LVL_FATAL, FUNCNAME_ONLY);
    panic("Stack overflow. Task: %s\n", pcTaskName);
}

void vApplicationMallocFailedHook()
{
    write_log("PANIC: malloc failed", LOG_LVL_FATAL, FUNCNAME_ONLY);
    panic("malloc failed");
}

void sub_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    write_log("Received: " + std::to_string(msg->data), LOG_LVL_INFO, FUNCNAME_ONLY);
}

void timer0(rcl_timer_t *timer, int64_t last_call_time)
{   
    if (timer != NULL)
    {
        write_log("Timer 0 (MicroROS) executing!", LOG_LVL_INFO, FUNCNAME_ONLY);

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &pub0;
        pub_item.message = &pub0_msg;
        pub_item.failed_callback = NULL;

        xQueueSendToBack(publishing_handler->get_queue_handle(), (void *) &pub_item, 0);
        pub0_msg.data++;
    }
}

void timer1(TimerHandle_t xTimer)
{
    write_log("Timer 1 (FreeRTOS) executing!", LOG_LVL_INFO, FUNCNAME_ONLY);
    
    uRosPublishingHandler::PublishItem_t pub_item;
    pub_item.publisher = &pub1;
    pub_item.message = &pub1_msg;
    pub_item.failed_callback = NULL;

    xQueueSendToBack(publishing_handler->get_queue_handle(), (void *) &pub_item, 0);
    pub1_msg.data++;
}

bool uros_init()
{
    write_log("MicroROS init...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge_instance->uros_init_node("uros_bridge_agent", "io");
    
    rclc_timer_init_default(&rc_timer, bridge_instance->get_support(), RCL_MS_TO_NS(1000), timer0);
    bridge_instance->init_publisher(&pub0, int32_type, "pub0");
    bridge_instance->init_publisher(&pub1, int32_type, "pub1");
    bridge_instance->init_publisher(&diagnostics_pub, diag_status_type, "diagnostics");
    bridge_instance->init_subscriber(&sub, int32_type, "sub");
    
    bridge_instance->add_executor_handles(1);   // For rc_timer!
    bridge_instance->uros_init_executor();
    bridge_instance->add_subscriber(&sub, &sub_msg, &sub_callback);

    write_log("Starting timers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    rclc_executor_add_timer(bridge_instance->get_executor(), &rc_timer);

    if (xTimerStart(rtos_timer, 0) != pdPASS)
    {
        write_log("Failed to start the FreeRTOS timer.", LOG_LVL_ERROR, FUNCNAME_ONLY);
    }

    if (xTimerStart(top_timer, 0) != pdPASS)
    {
        write_log("Failed to start the FreeRTOS TOP timer.", LOG_LVL_ERROR, FUNCNAME_ONLY);
    }

    write_log("MicroROS init done.", LOG_LVL_INFO, FUNCNAME_ONLY);
    return true;
}

void uros_fini()
{
    write_log("MicroROS fini...", LOG_LVL_WARN, FUNCNAME_ONLY);
    rclc_executor_remove_timer(bridge_instance->get_executor(), &rc_timer);
    xTimerStop(rtos_timer, 0);
    bridge_instance->uros_fini();
    write_log("MicroROS fini done.", LOG_LVL_WARN, FUNCNAME_ONLY);
}

void print_task_info(TimerHandle_t xTimer)
{
    TaskStatus_t *task_status_array;
    UBaseType_t task_count;
    task_count = uxTaskGetNumberOfTasks();
    task_status_array = (TaskStatus_t *)pvPortMalloc(task_count * sizeof(TaskStatus_t));

    if (task_status_array != NULL)
    {
        print_uart("\e[1;1H\e[2J");
        task_count = uxTaskGetSystemState(task_status_array, task_count, NULL);
        
        for (UBaseType_t i = 0; i < task_count; i++)
        {
            TaskStatus_t *task_status = &task_status_array[i];
            char buffer[256];

            snprintf(buffer, 256, "Task Name: %s\r\n", task_status->pcTaskName);
            print_uart(buffer);
            snprintf(buffer, 256, "Task Priority: %lu\r\n", task_status->uxCurrentPriority);
            print_uart(buffer);
            snprintf(buffer, 256, "Task State: %lu\r\n", task_status->eCurrentState);
            print_uart(buffer);
            snprintf(buffer, 256, "Task Stack High Water Mark: %lu\r\n", task_status->usStackHighWaterMark);
            print_uart(buffer);
            snprintf(buffer, 256, "Task Stack Size: %lu\r\n", task_status->usStackHighWaterMark + task_status->usStackHighWaterMark);
            print_uart(buffer);
            snprintf(buffer, 256, "Task Run Time: %lu\r\n", task_status->ulRunTimeCounter);
            print_uart(buffer);
            print_uart("------------------------\r\n");
        }
        
        vPortFree(task_status_array);
    }
    
    else
    {
        write_log("Failed to allocate memory for task status array.", LOG_LVL_ERROR, FUNCNAME_ONLY);
    }
}

int main()
{
    stdio_init_all();
    stdio_filter_driver(&stdio_usb);
    write_log("STDIO init, program starting...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge_instance = uRosBridgeAgent::get_instance();
    bridge_instance->pre_init(uros_init, uros_fini);
    publishing_handler = uRosPublishingHandler::get_instance();
    publishing_handler->pre_init(bridge_instance);
    
    write_log("xTimerCreate...", LOG_LVL_INFO, FUNCNAME_ONLY);
    rtos_timer = xTimerCreate("pub1_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *) 0, timer1);
    top_timer = xTimerCreate("top_timer", pdMS_TO_TICKS(2000), pdTRUE, (void *) 0, print_task_info);
    
    check_bool(bridge_instance->start(configMAX_PRIORITIES - 2, (1 << 0), true), RT_HARD_CHECK);
    check_bool(publishing_handler->start(configMAX_PRIORITIES - 2, (1 << 0), true), RT_HARD_CHECK);

    write_log("vTaskStartScheduler...", LOG_LVL_INFO, FUNCNAME_ONLY);
    vTaskStartScheduler();
    write_log("Failed to start the FreeRTOS scheduler.", LOG_LVL_FATAL, FUNCNAME_ONLY);
    return 0;
}