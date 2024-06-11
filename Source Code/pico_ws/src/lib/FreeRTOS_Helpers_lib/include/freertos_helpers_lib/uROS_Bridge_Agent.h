/*
    The ROS robot project
    uROS Bridge singleton object for managing uROS and uROS comms.
    This object handles the MicroROS executor and the MicroROS node.
    It also provides a method for adding messaged to its publishing queue.
    
    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
    Heavily inspired by: https://github.com/jondurrant/RPIPicoFreeRTOSuROSPubSub
 
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
#include "pico/stdlib.h"
#include "freertos_helpers_lib/RTOS_Agent.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>


// uROS Bridge Agent class
class uRosBridgeAgent : public Agent
{
    public:
        // Get the singleton instance
        static uRosBridgeAgent *get_instance();

        // Initialize MicroROS node and executor.
        // This function should be called before any other uROS-related functions.
        // This function is NOT thread-safe.
        void uros_init();

        // Finalize MicroROS node and executor, and stop the agent.
        // This function is NOT thread-safe.
        void uros_fini();


    private:
        // Constructor & Destructor
        uRosBridgeAgent();
        ~uRosBridgeAgent();

        static uRosBridgeAgent *instance;
        rcl_allocator_t rcl_allocator;
        rcl_node_t rc_node;
        rclc_support_t rc_support;
        rclc_executor_t rc_executor;


    protected:
        // Execution function
        virtual void execute();
};