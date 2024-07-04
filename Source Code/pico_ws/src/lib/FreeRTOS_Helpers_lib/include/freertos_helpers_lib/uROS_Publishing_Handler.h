/*
    The ROS robot project
    uROS Publishing Handler for managing MicroROS message publication.
    TODO: Explain why this is needed.
    
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
#include "freertos_helpers_lib/uROS_Bridge_Agent.h"
#include "local_helpers_lib/Local_Helpers.h"


// Absolute maximums
#define MAX_PUBLISH_QUEUE_ITEMS  20

// Misc.
#define PUB_AGENT_MEMORY_WORDS  2048
#define PUB_AGENT_NAME          "uROS_Publishing_Handler"


// uROS Publishing Handler Agent class
class uRosPublishingHandler : public Agent
{
    public:
        // Publishing failed callback typedef
        typedef void (*publish_failed_callback)(rcl_publisher_t *publisher, void *message);

        // Publishing queue item struct
        struct PublishItem 
        {
            rcl_publisher_t *publisher;
            void *message;
            publish_failed_callback failed_callback = NULL;
            RT_CHECK_MODE rt_check_mode = RT_SOFT_CHECK;
        };

        // Publishing queue item typedef
        typedef struct PublishItem PublishItem_t;

        // Get the singleton instance
        static uRosPublishingHandler *get_instance();

        // Pre-init configuration
        void pre_init(uRosBridgeAgent *bridge_instance);

        // Get the queue handle
        QueueHandle_t get_queue_handle();


    private:
        // Constructor & Destructor
        uRosPublishingHandler();
        virtual ~uRosPublishingHandler();

        static uRosPublishingHandler *instance;
        QueueHandle_t publishing_queue = NULL;
        uRosBridgeAgent *bridge;


    protected:
        // Execution function
        virtual void execute();
};