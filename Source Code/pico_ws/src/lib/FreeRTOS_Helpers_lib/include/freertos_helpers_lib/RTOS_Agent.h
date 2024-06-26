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


#pragma once
#include "FreeRTOS.h"
#include "task.h"


// Max agent name length
#define MAX_NAME_LENGTH  25


// Agent class
class Agent
{
    public:
        // Constructor
        // (name is intentionally not a std::string to avoid dynamic memory alloc.)
        Agent(const char *name, configSTACK_DEPTH_TYPE task_stack_depth);

        // Destructor
        virtual ~Agent();


        // ---- Functions ----

        // Start the agent (FreeRTOS task)
        virtual bool start(UBaseType_t priority, UBaseType_t core_affinity_mask, bool set_core_affinity);

        // Stop the agent (FreeRTOS task)
        virtual void stop();

        // Get the task High Water Mark.
        // Close to 0 is an overflow risk.
        UBaseType_t get_high_water_mark();

        // Get the FreeRTOS task that is being used by the agent.
        TaskHandle_t get_rtos_task();


    protected:
        // ---- Functions ----

        // Static internal function used by FreeRTOS to start
        // the agent task.
        static void vTask(void *pvParameters);

        // Main task execute function.
        // Code that is to be executed by the task goes here.
        virtual void execute() = 0;  // Declare as pure virtual function.


        // ---- Variables ----

        // The FreeRTOS task handle
        TaskHandle_t task_handle = NULL;

        // The agent (task) name.
        char agent_name[MAX_NAME_LENGTH];

        // The static stack depth required by the agent.
        configSTACK_DEPTH_TYPE stack_depth;
};