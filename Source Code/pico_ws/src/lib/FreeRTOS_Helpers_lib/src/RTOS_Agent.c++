/*
    The ROS robot project
    Agent object that abstracts FreeRTOS tasks.

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


#include "freertos_helpers_lib/RTOS_Agent.h"
#include <string.h>


// Constructor
// (name is intentionally not a std::string to avoid dynamic memory alloc.)
Agent::Agent(const char *name, configSTACK_DEPTH_TYPE task_stack_depth)
{
    stack_depth = task_stack_depth;

    // Make sure the provided name is not longer than the allowed maximum.
    if (strlen(name) >= MAX_NAME_LENGTH)
    {
		memcpy(agent_name, name, MAX_NAME_LENGTH);
		agent_name[MAX_NAME_LENGTH - 1] = 0;
	}
    
    else 
    {
		strcpy(agent_name, name);
	}
}


// Destructor
Agent::~Agent()
{
    stop();
}


// ---- Functions ----

// Start the agent (FreeRTOS task)
// Pass false to set_core_affinity to let FreeRTOS decide.
bool Agent::start(UBaseType_t priority = tskIDLE_PRIORITY, UBaseType_t core_affinity_mask = (1 << 0), bool set_core_affinity = false)
{
    BaseType_t res = xTaskCreate(Agent::vTask, agent_name, stack_depth, (void *) this, priority, &task_handle);

    if (set_core_affinity && res == pdPASS)
    {
        vTaskCoreAffinitySet(task_handle, core_affinity_mask);
    }

    return (res == pdPASS);
}


// Stop the agent (FreeRTOS task)
void Agent::stop()
{
    if (task_handle != NULL)
    {
        vTaskDelete(task_handle);
        task_handle = NULL;
    }
}


// Get the task High Water Mark.
// Close to 0 is an overflow risk.
UBaseType_t Agent::get_high_water_mark()
{
    if (task_handle != NULL)
    {
        return uxTaskGetStackHighWaterMark(task_handle);
    }

    return 0;
}


// Get the FreeRTOS task that is being used by the agent.
TaskHandle_t Agent::get_rtos_task()
{
    return task_handle;
}



// ---- Functions ----

// Static internal function used by FreeRTOS to start
// the agent task.
void Agent::vTask(void *parameters)
{
    Agent *agent = (Agent *) parameters;

    if (agent != NULL)
    {
        agent->execute();
    }
}