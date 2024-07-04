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


#pragma once
#include "freertos_helpers_lib/RTOS_Agent.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <vector>


// Absolute maximums
#define MAX_PUBLISHERS   8
#define MAX_SUBSCRIBERS  2
#define MAX_SERVICES     7
#define MAX_TIMERS       10
#define MAX_EXEC_TIME    60

// Misc.
#define EXECUTE_DELAY_MS           80
#define EXECUTOR_TIMEOUT_MS        25
#define BRIDGE_AGENT_MEMORY_WORDS  2048
#define BRIDGE_AGENT_NAME          "uROS_Bridge_Agent"


// uROS Bridge Agent class
class uRosBridgeAgent : public Agent
{
    public:
        // MicroROS init & fini (pubs, subs, services, timers, executor, node, etc.) function typedefs
        typedef bool (*uros_init_function)(void);
        typedef void (*uros_fini_function)(void);

        // MicroROS agent state enum
        enum UROS_STATE {WAITING_FOR_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED};

        // MicroROS object QoS mode
        enum QOS_MODE {QOS_BEST_EFFORT, QOS_RELIABLE};

        // Get the singleton instance
        static uRosBridgeAgent* get_instance();

        // Pre-init configuration
        void pre_init(uros_init_function init_function, uros_fini_function fini_function);

        // Initialize MicroROS node.
        // This function should be called before any other uROS-related functions.
        // This function is NOT thread-safe.
        void uros_init_node(const char *node_name, const char *name_space);

        // Initialize MicroROS executor.
        // This function should be called after uros_init_node().
        // This function is NOT thread-safe.
        void uros_init_executor();

        // Finalize MicroROS node, executor, services, subscriptions, 
        // publishers and timers,and stop the agent.
        // This function is NOT thread-safe.
        void uros_fini();

        // Initialize a publisher.
        // This function is NOT thread-safe.
        bool init_publisher(rcl_publisher_t *publisher, const rosidl_message_type_support_t *type_support, const char *topic_name, QOS_MODE qos_mode = QOS_BEST_EFFORT);

        // Initialize a subscriber.
        // This function is NOT thread-safe.
        bool init_subscriber(rcl_subscription_t *subscriber, const rosidl_message_type_support_t *type_support, const char *topic_name, QOS_MODE qos_mode = QOS_RELIABLE);

        // Initialize a service.
        // This function is NOT thread-safe.
        bool init_service(rcl_service_t *service, const rosidl_service_type_support_t *type_support, const char *service_name, QOS_MODE qos_mode = QOS_RELIABLE);

        // Add a subscriber to the executor.
        // This function is NOT thread-safe.
        bool add_subscriber(rcl_subscription_t *subscriber, void *msg, rclc_subscription_callback_t callback, rclc_executor_handle_invocation_t invocation = ON_NEW_DATA);

        // Add a service to the executor.
        // This function is NOT thread-safe.
        bool add_service(rcl_service_t *service, void *request, void *response, rclc_service_callback_t callback);

        // TODO: Implement timers
        //bool init_timer(rcl_timer_t *timer, uint64_t period, rcl_timer_callback_t callback);

        // Add n amount of handles to the executor.
        // This function is only effective before the executor is started.
        void add_executor_handles(int num_handles);
        
        // Get the MicroROS allocator.
        rcl_allocator_t* get_allocator();

        // Get the MicroROS node.
        rcl_node_t* get_node();

        // Get the MicroROS support.
        rclc_support_t* get_support();

        // Get the MicroROS executor.
        rclc_executor_t* get_executor();

        // Get the MicroROS agent state.
        uRosBridgeAgent::UROS_STATE get_agent_state();


    private:
        // Constructor & Destructor
        uRosBridgeAgent();
        virtual ~uRosBridgeAgent();

        // Initialize function
        uros_init_function init_func;
        uros_fini_function fini_func;

        // MicroROS agent state
        UROS_STATE current_uros_state;

        static uRosBridgeAgent *instance;
        rcl_allocator_t rcl_allocator;
        rcl_node_t rc_node;
        rclc_support_t rc_support;
        rclc_executor_t rc_executor;

        bool node_initialized = false;
        bool executor_initialized = false;

        int executor_handles = 0;
        std::vector<rcl_publisher_t *> publishers;
        std::vector<rcl_subscription_t *> subscribers;
        std::vector<rcl_service_t *> services;
        std::vector<rcl_timer_t *> timers;


    protected:
        // Execution function
        virtual void execute();
};