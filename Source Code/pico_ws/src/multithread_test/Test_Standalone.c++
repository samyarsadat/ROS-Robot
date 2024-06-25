#include "pico/stdlib.h"
#include "pico/stdio_uart.h"
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include "utils/pico_uart_transports.h"
#include "utils/uros_allocators.h"
#include <string>
#include "FreeRTOS.h"
#include <task.h>
#include <timers.h>


rcl_allocator_t rcl_allocator;
rcl_node_t rc_node;
rclc_support_t rc_support;
rclc_executor_t rc_executor;
rcl_timer_t rc_timer;
rcl_publisher_t pub0, pub1, diagnostics_pub;
rcl_subscription_t sub;
std_msgs__msg__Int32 pub0_msg, pub1_msg, sub_msg;
const rosidl_message_type_support_t *int32_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

enum UROS_STATE {WAITING_FOR_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED};
enum LOG_LEVEL {LOG_LVL_INFO, LOG_LVL_WARN, LOG_LVL_ERROR, LOG_LVL_FATAL};
TimerHandle_t rtos_timer;


/* SYSTEM FUNCTIONS */
void write_log(const char *function, std::string msg, LOG_LEVEL lvl)
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint16_t timestamp_millisec = to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000);

    std::string level;
    if (lvl == LOG_LVL_INFO) { level = "INFO"; }
    else if (lvl == LOG_LVL_WARN) { level = "WARNING"; }
    else if (lvl == LOG_LVL_ERROR) { level = "ERROR"; }
    else if (lvl == LOG_LVL_FATAL) { level = "FATAL"; }

    // This is quite ugly, but it works.
    msg = "[" + std::to_string(timestamp_sec) + "." + std::to_string(timestamp_millisec) + "] [" + level + "] [" + function + "]: " + msg + "\r\n";
    stdio_uart.out_chars(msg.c_str(), msg.length());
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    std::string task_name_str(pcTaskName, strlen(pcTaskName));
    write_log(__FUNCTION__, "PANIC: Stack overflow! Task: " + task_name_str, LOG_LVL_FATAL);
    panic("Stack overflow. Task: %s\n", pcTaskName);
}

void vApplicationMallocFailedHook()
{
    write_log(__FUNCTION__, "PANIC: malloc failed", LOG_LVL_FATAL);
    panic("malloc failed");
}

bool ping_agent()
{
    // Try for 10 seconds max.
    write_log(__FUNCTION__, "Pinging agent...", LOG_LVL_INFO);
    bool success = (rmw_uros_ping_agent(1000, 10) == RMW_RET_OK);
    return success;
}


/* CALLBACKS */
void sub_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    write_log(__FUNCTION__, "Received: " + std::to_string(msg->data), LOG_LVL_INFO);
}


/* TIMERS */
void timer0(rcl_timer_t *timer, int64_t last_call_time)
{
    write_log(__FUNCTION__, "Timer 0 (MicroROS) executing!", LOG_LVL_INFO);
    
    if (timer != NULL)
    {
        rcl_publish(&pub0, &pub0_msg, NULL);
        pub0_msg.data++;
    }
}

void timer1(TimerHandle_t xTimer)
{
    write_log(__FUNCTION__, "Timer 1 (FreeRTOS) executing!", LOG_LVL_INFO);

    rcl_publish(&pub1, &pub1_msg, NULL);
    pub1_msg.data++;
}


/* INIT FUNCTIONS */
void uros_preinit()
{
    // Set MicroROS default allocators
    rcl_allocator_t rtos_allocators = rcutils_get_zero_initialized_allocator();
	rtos_allocators.allocate = uros_rtos_allocate;
	rtos_allocators.deallocate = uros_rtos_deallocate;
	rtos_allocators.reallocate = uros_rtos_reallocate;
	rtos_allocators.zero_allocate = uros_rtos_zero_allocate;
	rcutils_set_default_allocator(&rtos_allocators);

    // Set MicroROS transport
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
}

void uros_create()
{
    write_log(__FUNCTION__, "MicroROS create...", LOG_LVL_INFO);

    // Initialize the MicroROS node
    rcl_allocator = rcl_get_default_allocator();
    rclc_support_init(&rc_support, 0, NULL, &rcl_allocator);
    rclc_node_init_default(&rc_node, "pico_freertos_test", "", &rc_support);

    // Initialize publishers
    rclc_publisher_init_default(&pub0, &rc_node, int32_type, "pub0");
    rclc_publisher_init_default(&pub1, &rc_node, int32_type, "pub1");

    // Initialize subscribers
    rclc_subscription_init_default(&sub, &rc_node, int32_type, "sub");
    
    // Initialize timers
    rclc_timer_init_default(&rc_timer, &rc_support, RCL_MS_TO_NS(1000), timer0);

    // Initialize the executor
    rc_executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&rc_executor, &rc_support.context, 2, &rcl_allocator);
    rclc_executor_add_subscription(&rc_executor, &sub, &sub_msg, &sub_callback, ON_NEW_DATA);

    write_log(__FUNCTION__, "MicroROS create done.", LOG_LVL_INFO);
}

void timers_start()
{
    write_log(__FUNCTION__, "Starting timers...", LOG_LVL_INFO);
    rclc_executor_add_timer(&rc_executor, &rc_timer);

    if (xTimerStart(rtos_timer, 0) != pdPASS)
    {
        write_log(__FUNCTION__, "Failed to start the FreeRTOS timer.", LOG_LVL_ERROR);
    }
}


/* TASKS */
void uros_task(void *pvParameters)
{
    UROS_STATE current_uros_state = WAITING_FOR_AGENT;

    while (true)
    {
        switch (current_uros_state) 
        {
            case WAITING_FOR_AGENT:
                write_log(__FUNCTION__, "Waiting for agent...", LOG_LVL_INFO);
                current_uros_state = ping_agent() ? AGENT_AVAILABLE:WAITING_FOR_AGENT;
                break;
            
            case AGENT_AVAILABLE:
                write_log(__FUNCTION__, "Agent available!", LOG_LVL_INFO);
                uros_create();
                timers_start();
                current_uros_state = AGENT_CONNECTED;
                break;
            
            case AGENT_CONNECTED:
                current_uros_state = ping_agent() ? AGENT_CONNECTED:AGENT_DISCONNECTED;
                
                if (current_uros_state == AGENT_CONNECTED) 
                {
                    rclc_executor_spin_some(&rc_executor, RCL_MS_TO_NS(100));
                    vTaskDelay(75 / portTICK_PERIOD_MS);
                }
                
                break;
            
            case AGENT_DISCONNECTED:
                write_log(__FUNCTION__, "Agent disconnected!", LOG_LVL_INFO);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
        }
    }
}


/* MAIN */
int main()
{
    stdio_init_all();
    stdio_filter_driver(&stdio_usb);   // Filter the output of STDIO to USB.
    write_log(__FUNCTION__, "STDIO init, program starting...", LOG_LVL_INFO);

    uros_preinit();

    TaskHandle_t uros_task_handle;
    xTaskCreate(uros_task, "uros_task", 1024, NULL, configMAX_PRIORITIES - 1, &(uros_task_handle));
    vTaskCoreAffinitySet(uros_task_handle, (1 << 1));

    write_log(__FUNCTION__, "xTimerCreate...", LOG_LVL_INFO);
    rtos_timer = xTimerCreate("pub1_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *) 0, timer1);

    write_log(__FUNCTION__, "vTaskStartScheduler...", LOG_LVL_INFO);
    vTaskStartScheduler();
    
    // We should never get here...
    write_log(__FUNCTION__, "Failed to start the FreeRTOS scheduler.", LOG_LVL_FATAL);
    return 0;
}