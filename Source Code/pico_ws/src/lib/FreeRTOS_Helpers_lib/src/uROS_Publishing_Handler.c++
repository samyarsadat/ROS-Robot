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


#include "freertos_helpers_lib/uROS_Publishing_Handler.h"


// Constructor
uRosPublishingHandler::uRosPublishingHandler() : Agent(PUB_AGENT_NAME, PUB_AGENT_MEMORY_WORDS)
{}


// Destructor
uRosPublishingHandler::~uRosPublishingHandler()
{
    if (publishing_queue != NULL)
    {
        vQueueDelete(publishing_queue);
    }

    if (instance != NULL)
    {
        delete instance;
    }
}


// ---- Functions ----

// Get the singleton instance
uRosPublishingHandler *uRosPublishingHandler::instance = NULL;
uRosPublishingHandler *uRosPublishingHandler::get_instance()
{
    if (instance == NULL)
    {
        instance = new uRosPublishingHandler();
    }

    return instance;
}


// Pre-init configuration
void uRosPublishingHandler::pre_init(uRosBridgeAgent *bridge_instance)
{
    // Set the bridge instance
    bridge = bridge_instance;

    // Create the publishing queue
    publishing_queue = xQueueCreate(MAX_PUBLISH_QUEUE_ITEMS, sizeof(PublishItem_t));

    if (publishing_queue == NULL)
    {
        write_log("Error creating publishing queue.", LOG_LVL_FATAL, FUNCNAME_ONLY);
    }
}


// Get the queue handle
QueueHandle_t uRosPublishingHandler::get_queue_handle()
{
    return publishing_queue;
}


// Main task execution function
void uRosPublishingHandler::execute()
{
    PublishItem_t queue_item;

    while (true)
    {
        // Blocks indefinitely until an item is available.
        BaseType_t avail = xQueueReceive(publishing_queue, &queue_item, portMAX_DELAY);

        if (avail == pdTRUE && bridge->get_agent_state() == uRosBridgeAgent::AGENT_CONNECTED && queue_item.publisher != NULL && queue_item.message != NULL)
        {
            if (!check_rc(rcl_publish(queue_item.publisher, queue_item.message, NULL), queue_item.rt_check_mode) && queue_item.failed_callback != NULL)
            {
                // Call the failed publish callback, if available.
                queue_item.failed_callback(queue_item.publisher, queue_item.message);
            }
        }
    }
}