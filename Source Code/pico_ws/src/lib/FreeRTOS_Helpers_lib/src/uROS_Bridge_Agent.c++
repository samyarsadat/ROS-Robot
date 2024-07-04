/*
    The ROS robot project
    uROS Bridge singleton object for managing MicroROS and MicroROS comms.
    This object handles the MicroROS executor and the MicroROS node.
    It also provides methods for initializing publishers, subscribers, and services.
    
    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
    Inspired by: https://github.com/jondurrant/RPIPicoFreeRTOSuROSPubSub
 
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


#include "freertos_helpers_lib/uROS_Bridge_Agent.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "pico_uart_transports.h"
#include "uros_allocators.h"
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>


// Constructor
uRosBridgeAgent::uRosBridgeAgent() : Agent(BRIDGE_AGENT_NAME, BRIDGE_AGENT_MEMORY_WORDS)
{}


// Destructor
uRosBridgeAgent::~uRosBridgeAgent()
{
    if (instance != NULL)
    {
        delete instance;
    }
}


// ---- Functions ----

// Get the singleton instance
uRosBridgeAgent *uRosBridgeAgent::instance = NULL;
uRosBridgeAgent *uRosBridgeAgent::get_instance()
{
    if (instance == NULL)
    {
        instance = new uRosBridgeAgent();
    }

    return instance;
}


// Pre-init configuration
void uRosBridgeAgent::pre_init(uros_init_function init_function, uros_fini_function fini_function)
{
    uRosBridgeAgent::get_instance()->init_func = init_function;
    uRosBridgeAgent::get_instance()->fini_func = fini_function;

    // Set MicroROS default allocators
    rcl_allocator_t rtos_allocators = rcutils_get_zero_initialized_allocator();
	rtos_allocators.allocate = uros_rtos_allocate;
	rtos_allocators.deallocate = uros_rtos_deallocate;
	rtos_allocators.reallocate = uros_rtos_reallocate;
	rtos_allocators.zero_allocate = uros_rtos_zero_allocate;
	check_bool(rcutils_set_default_allocator(&rtos_allocators), RT_HARD_CHECK);

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


// Initialize MicroROS node and executor.
// This function should be called before any other uROS-related functions.
// This function is NOT thread-safe.
void uRosBridgeAgent::uros_init_node(const char *node_name, const char *name_space)
{
    if (!node_initialized)
    {
        // Initialize the MicroROS allocator
        rcl_allocator = rcl_get_default_allocator();

        // Initialize the MicroROS node
        check_rc(rclc_support_init(&rc_support, 0, NULL, &rcl_allocator), RT_HARD_CHECK);
        check_rc(rclc_node_init_default(&rc_node, node_name, name_space, &rc_support), RT_HARD_CHECK);
        
        node_initialized = true;
    }
}


// Initialize MicroROS executor.
// This function should be called after uros_init_node().
// This function is NOT thread-safe.
void uRosBridgeAgent::uros_init_executor()
{
    if (!executor_initialized)
    {
        // Initialize the MicroROS executor
        rc_executor = rclc_executor_get_zero_initialized_executor();
        check_rc(rclc_executor_init(&rc_executor, &rc_support.context, executor_handles, &rcl_allocator), RT_HARD_CHECK);
        
        executor_initialized = true;
    }
}


// Finalize MicroROS node, executor, services, subscriptions,
// publishers and timers, and stop the agent.
// This function is NOT thread-safe.
void uRosBridgeAgent::uros_fini()
{
    current_uros_state = AGENT_DISCONNECTED;

    for (auto sub : subscribers)
    {
        rcl_subscription_fini(sub, &rc_node);
    }

    for (auto srv : services)
    {
        rcl_service_fini(srv, &rc_node);
    }

    for (auto pub : publishers)
    {
        rcl_publisher_fini(pub, &rc_node);
    }

    rclc_executor_fini(&rc_executor);
    rcl_node_fini(&rc_node);
    rclc_support_fini(&rc_support);
}


// Initialize a publisher.
// Call this before uros_init_executor().
// This function is NOT thread-safe.
bool uRosBridgeAgent::init_publisher(rcl_publisher_t *publisher, const rosidl_message_type_support_t *type_support, const char *topic_name, QOS_MODE qos_mode)
{
    if (publishers.size() < MAX_PUBLISHERS)
    {
        if (qos_mode == QOS_RELIABLE)
        {
            check_rc(rclc_publisher_init_default(publisher, &rc_node, type_support, topic_name), RT_HARD_CHECK);
        }
        
        else
        {
            check_rc(rclc_publisher_init_best_effort(publisher, &rc_node, type_support, topic_name), RT_HARD_CHECK);
        }

        publishers.push_back(publisher);
        return true;
    }

    return false;
}


// Initialize a subscriber.
// Call this before uros_init_executor().
// This function is NOT thread-safe.
bool uRosBridgeAgent::init_subscriber(rcl_subscription_t *subscriber, const rosidl_message_type_support_t *type_support, const char *topic_name, QOS_MODE qos_mode)
{
    if (subscribers.size() < MAX_SUBSCRIBERS)
    {
        if (qos_mode == QOS_RELIABLE)
        {
            check_rc(rclc_subscription_init_default(subscriber, &rc_node, type_support, topic_name), RT_HARD_CHECK);
        }
        
        else
        {
            check_rc(rclc_subscription_init_best_effort(subscriber, &rc_node, type_support, topic_name), RT_HARD_CHECK);
        }

        subscribers.push_back(subscriber);
        executor_handles += 1;
        return true;
    }

    return false;
}


// Initialize a service.
// Call this before uros_init_executor().
// This function is NOT thread-safe.
bool uRosBridgeAgent::init_service(rcl_service_t *service, const rosidl_service_type_support_t *type_support, const char *service_name, QOS_MODE qos_mode)
{
    if (services.size() < MAX_SERVICES)
    {
        if (qos_mode == QOS_RELIABLE)
        {
            check_rc(rclc_service_init_default(service, &rc_node, type_support, service_name), RT_HARD_CHECK);
        }
        
        else
        {
            check_rc(rclc_service_init_best_effort(service, &rc_node, type_support, service_name), RT_HARD_CHECK);
        }
        
        services.push_back(service);
        executor_handles += 1;
        return true;
    }

    return false;
}


// Add a subscriber to the executor.
// Call this after uros_init_executor().
// This function is NOT thread-safe.
bool uRosBridgeAgent::add_subscriber(rcl_subscription_t *subscriber, void *msg, rclc_subscription_callback_t callback, rclc_executor_handle_invocation_t invocation)
{
    for (auto sub : subscribers)
    {
        if (sub == subscriber)
        {
            check_rc(rclc_executor_add_subscription(&rc_executor, subscriber, msg, callback, invocation), RT_HARD_CHECK);
            return true;
        }
    }

    return false;
}


// Add a service to the executor.
// Call this after uros_init_executor().
// This function is NOT thread-safe.
bool uRosBridgeAgent::add_service(rcl_service_t *service, void *request, void *response, rclc_service_callback_t callback)
{
    for (auto srv : services)
    {
        if (srv == service)
        {
            check_rc(rclc_executor_add_service(&rc_executor, service, request, response, callback), RT_HARD_CHECK);
            return true;
        }
    }

    return false;
}


// Add n amount of handles to the executor.
// This function is only effective before the executor is started.
void uRosBridgeAgent::add_executor_handles(int num_handles)
{
    if (!executor_initialized)
    {
        executor_handles += num_handles;
    }
}


// Get the MicroROS allocator.
rcl_allocator_t* uRosBridgeAgent::get_allocator()
{
    return &rcl_allocator;
}


// Get the MicroROS node.
rcl_node_t* uRosBridgeAgent::get_node()
{
    return &rc_node;
}


// Get the MicroROS support.
rclc_support_t* uRosBridgeAgent::get_support()
{
    return &rc_support;
}


// Get the MicroROS executor.
rclc_executor_t* uRosBridgeAgent::get_executor()
{
    return &rc_executor;
}


// Get the MicroROS agent state.
uRosBridgeAgent::UROS_STATE uRosBridgeAgent::get_agent_state()
{
    return current_uros_state;
}


// Execution function.
// Call this after you've initialized everything.
void uRosBridgeAgent::execute()
{
    uint32_t last_exec_time;
    current_uros_state = WAITING_FOR_AGENT;
    write_log("Waiting for agent...", LOG_LVL_INFO, FUNCNAME_ONLY);

    while (true)
    {
        switch (current_uros_state) 
        {
            case WAITING_FOR_AGENT:
                current_uros_state = ping_agent() ? AGENT_AVAILABLE:WAITING_FOR_AGENT;
                break;
            
            case AGENT_AVAILABLE:
                write_log("Agent available!", LOG_LVL_INFO, FUNCNAME_ONLY);
                check_bool(init_func(), RT_HARD_CHECK);
                current_uros_state = AGENT_CONNECTED;
                last_exec_time = time_us_32() / 1000;
                break;
            
            case AGENT_CONNECTED:
                current_uros_state = ping_agent() ? AGENT_CONNECTED:AGENT_DISCONNECTED;
                
                if (current_uros_state == AGENT_CONNECTED) 
                {
                    check_bool(check_exec_interval(last_exec_time, MAX_EXEC_TIME + EXECUTE_DELAY_MS, "Executor execution time exceeded limits!"), RT_SOFT_CHECK);
                    check_rc(rclc_executor_spin_some(&rc_executor, RCL_MS_TO_NS(EXECUTOR_TIMEOUT_MS)), RT_LOG_ONLY_CHECK);
                    vTaskDelay(EXECUTE_DELAY_MS / portTICK_PERIOD_MS);
                }
                
                break;
            
            case AGENT_DISCONNECTED:
                write_log("Agent disconnected!", LOG_LVL_INFO, FUNCNAME_ONLY);
                fini_func();
                stop();
                break;
        }
    }
}