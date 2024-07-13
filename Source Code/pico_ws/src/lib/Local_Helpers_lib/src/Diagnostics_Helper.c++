/*
    The ROS robot project - Local Helper/commonly used functions
    This file is for the diagnostics helper functions.
    They are program-specific.
    
    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
 
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


// ------- Libraries & Modules -------
#include "local_helpers_lib/Local_Helpers.h"
#include "freertos_helpers_lib/uROS_Bridge_Agent.h"
#include "freertos_helpers_lib/uROS_Publishing_Handler.h"
#include "semphr.h"



// ------- Definitions -------
#define DIAG_MSG_GLOBAL_DATA_ARRAY_SIZE  18
#define DIAG_PUBLISHER_TOPIC_NAME        "/diagnostics"



// ------- Structs -------

// Diagnostics message global message data array item struct.
struct diag_msg_global_item
{
    std::string name;
    std::string hardware_id;
    std::string message;
    uint8_t level;
};

typedef struct diag_msg_global_item diag_msg_global_item_t;

// Diagnostics key-value pair data array item struct.
struct diag_kv_pair_global_item
{
    std::string key;
    std::string value;
};

typedef struct diag_kv_pair_global_item diag_kv_pair_global_item_t;

// Global data storage array slot numbers storage array item struct.
struct diag_data_slot_nums_item
{
    int diag_msg_slot;
    int diag_status_msg_slot;
    std::vector<int> kv_pair_slots;
    int kv_vector_slot_num;
    int self_slot_num;
};

typedef struct diag_data_slot_nums_item diag_data_slot_nums_item_t;



// ------- Global variables -------

bool diag_pub_enabled = true;

// External variables
extern uRosBridgeAgent *bridge;
QueueHandle_t pub_queue;

// Mutexes
SemaphoreHandle_t array_data_access_mutex = NULL;

// MicroROS
rcl_publisher_t diagnostics_pub;
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msgs_array[DIAG_MSG_GLOBAL_DATA_ARRAY_SIZE] = {diagnostic_msgs__msg__DiagnosticStatus()};

// Global key-value pair vector array.
std::vector<diagnostic_msgs__msg__KeyValue> global_kv_pairs_vector_array[DIAG_MSG_GLOBAL_DATA_ARRAY_SIZE] = {std::vector<diagnostic_msgs__msg__KeyValue>()};

// Global diagnostics message data array.
diag_msg_global_item_t global_diag_data_buffer[DIAG_MSG_GLOBAL_DATA_ARRAY_SIZE] = {diag_msg_global_item_t()};

// Global diagnostics key-value pair vectors data array.
std::vector<diag_kv_pair_global_item_t> global_kv_pair_data_buffer[DIAG_MSG_GLOBAL_DATA_ARRAY_SIZE] = {std::vector<diag_kv_pair_global_item_t>()};

// Array slot availability indication array (true: full, false: free).
bool slot_availability[DIAG_MSG_GLOBAL_DATA_ARRAY_SIZE] = {false};



// ------- Functions ------- 

// Note: clean_shutdown() must be defined elsewhere!
extern void clean_shutdown();


// ---- Set Publishing Handler queue handle ----
void set_diag_pub_queue(QueueHandle_t queue)
{
    pub_queue = queue;
}


// ---- Initialize diagnostics publisher & message ----
void diag_uros_init()
{
    const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    bridge->init_publisher(&diagnostics_pub, diag_status_type, DIAG_PUBLISHER_TOPIC_NAME);
}


// ---- Initialize data access mutex ----
void diag_mutex_init()
{
    if (array_data_access_mutex == NULL)
    {
        array_data_access_mutex = xSemaphoreCreateMutex();

        if (array_data_access_mutex == NULL)
        {
            write_log("Array data access mutex creation failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
            clean_shutdown();
        }
    }
}


// ---- Allocate array slots for a message ----
int allocate_slots()
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (int i = 0; i < (sizeof(slot_availability) / sizeof(slot_availability[0])); i++)
        {
            if (slot_availability[i] == false)
            {
                slot_availability[i] = true;
                xSemaphoreGive(array_data_access_mutex);
                return i;
            }
        }

        xSemaphoreGive(array_data_access_mutex);
    }

    write_log("Diagnostics message slot allocation failed!", LOG_LVL_ERROR, FUNCNAME_ONLY);
    return -1;
}


// ---- Deallocate array slots ----
void deallocate_slots(int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        slot_availability[allocated_slot] = false;
        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Populate a diagnostics key-value pair vector ----
void populate_diag_kv_pair(std::vector<diag_kv_pair_item_t> *kv_pairs, int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (kv_pairs != NULL)
        {
            for (auto &pair : *kv_pairs)
            {
                global_kv_pair_data_buffer[allocated_slot].push_back(diag_kv_pair_global_item_t());
                global_kv_pair_data_buffer[allocated_slot].back().key = pair.key;
                global_kv_pair_data_buffer[allocated_slot].back().value = pair.value;
            }
        }

        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Destroy a diagnostics key-value pair vector ----
void destroy_diag_kv_pair(int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (auto &pair : global_kv_pair_data_buffer[allocated_slot])
        {
            // This is probably completely unnecessary...
            pair.key.resize(0);
            pair.value.resize(0);
            pair.key.shrink_to_fit();
            pair.value.shrink_to_fit();
        }

        global_kv_pair_data_buffer[allocated_slot].resize(0);
        global_kv_pair_data_buffer[allocated_slot].shrink_to_fit();
        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Populate a diagnostics key-value pair reference vector ----
void populate_diag_kv_pair_refs(int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (auto &pair : global_kv_pair_data_buffer[allocated_slot])
        {
            diagnostic_msgs__msg__KeyValue kv_pair;

            kv_pair.key.capacity = 0;
            kv_pair.value.capacity = 0;

            kv_pair.key.data = pair.key.data();
            kv_pair.key.size = pair.key.size();
            kv_pair.value.data = pair.value.data();
            kv_pair.value.size = pair.value.size();

            global_kv_pairs_vector_array[allocated_slot].push_back(kv_pair);
        }

        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Destroy a diagnostics key-value pair reference vector ----
void destroy_diag_kv_pair_refs(int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        global_kv_pairs_vector_array[allocated_slot].resize(0);
        global_kv_pairs_vector_array[allocated_slot].shrink_to_fit();
        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Populate the diagnostic status message of the given slot ----
void populate_diag_msg_object(uint8_t level, std::string_view hw_name, std::string_view hw_id, std::string_view msg, int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        global_diag_data_buffer[allocated_slot].level = level;
        global_diag_data_buffer[allocated_slot].name = hw_name;
        global_diag_data_buffer[allocated_slot].hardware_id = hw_id;
        global_diag_data_buffer[allocated_slot].message = msg;
        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Destroy a diagnostic status message ----
void destroy_diag_msg_object(int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        global_diag_data_buffer[allocated_slot].name.resize(0);
        global_diag_data_buffer[allocated_slot].hardware_id.resize(0);
        global_diag_data_buffer[allocated_slot].message.resize(0);
        global_diag_data_buffer[allocated_slot].name.shrink_to_fit();
        global_diag_data_buffer[allocated_slot].hardware_id.shrink_to_fit();
        global_diag_data_buffer[allocated_slot].message.shrink_to_fit();
        global_diag_data_buffer[allocated_slot].level = 0;
        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Returns a DiagnosticStatus message from the provided global data array slot ----
diagnostic_msgs__msg__DiagnosticStatus get_diag_msg(int allocated_slot)
{
    diagnostic_msgs__msg__DiagnosticStatus diag_msg;

    diag_msg.name.capacity = 0;
    diag_msg.hardware_id.capacity = 0;
    diag_msg.message.capacity = 0;
    diag_msg.values.capacity = 0;
    diag_msg.values.size = 0;
    diag_msg.values.data = NULL;

    diag_msg.name.data = global_diag_data_buffer[allocated_slot].name.data();
    diag_msg.name.size = global_diag_data_buffer[allocated_slot].name.size();
    diag_msg.hardware_id.data = global_diag_data_buffer[allocated_slot].hardware_id.data();
    diag_msg.hardware_id.size = global_diag_data_buffer[allocated_slot].hardware_id.size();
    diag_msg.message.data = global_diag_data_buffer[allocated_slot].message.data();
    diag_msg.message.size = global_diag_data_buffer[allocated_slot].message.size();
    diag_msg.level = global_diag_data_buffer[allocated_slot].level;

    if (!global_kv_pair_data_buffer[allocated_slot].empty())
    {
        diag_msg.values.data = global_kv_pairs_vector_array[allocated_slot].data();
        diag_msg.values.size = global_kv_pairs_vector_array[allocated_slot].size();
    }

    return diag_msg;
}


// ---- Destroy MicroROS DiagnosticStatus message ----
void destroy_uros_diag_status_msg(int allocated_slot)
{
    if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        diagnostics_msgs_array[allocated_slot].name.data = NULL;
        diagnostics_msgs_array[allocated_slot].name.size = 0;
        diagnostics_msgs_array[allocated_slot].name.capacity = 0;
        diagnostics_msgs_array[allocated_slot].hardware_id.data = NULL;
        diagnostics_msgs_array[allocated_slot].hardware_id.size = 0;
        diagnostics_msgs_array[allocated_slot].hardware_id.capacity = 0;
        diagnostics_msgs_array[allocated_slot].message.data = NULL;
        diagnostics_msgs_array[allocated_slot].message.size = 0;
        diagnostics_msgs_array[allocated_slot].message.capacity = 0;
        diagnostics_msgs_array[allocated_slot].values.data = NULL;
        diagnostics_msgs_array[allocated_slot].values.size = 0;
        diagnostics_msgs_array[allocated_slot].values.capacity = 0;
        diagnostics_msgs_array[allocated_slot].level = 0;
        xSemaphoreGive(array_data_access_mutex);
        return;
    }

    write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
}


// ---- Prepares a diag_publish_item_t struct for diagnostics message publishing based on inputs ----
diag_publish_item_t prepare_diag_publish_item(uint8_t level, std::string_view hw_name, std::string_view hw_id, std::string_view msg, std::vector<diag_kv_pair_item_t> *kv_pairs)
{
    int allocated_slot = allocate_slots();
    diag_publish_item_t pub_item;

    if (allocated_slot > -1)
    {
        populate_diag_msg_object(level, hw_name, hw_id, msg, allocated_slot);
        populate_diag_kv_pair(kv_pairs, allocated_slot);
        populate_diag_kv_pair_refs(allocated_slot);

        if (xSemaphoreTake(array_data_access_mutex, portMAX_DELAY) == pdTRUE)
        {
            diagnostics_msgs_array[allocated_slot] = get_diag_msg(allocated_slot);
            xSemaphoreGive(array_data_access_mutex);
        }

        else
        {
            write_log("Array data access mutex acquisition failed!", LOG_LVL_FATAL, FUNCNAME_ONLY);
        }
    }

    pub_item.diag_msg = &diagnostics_msgs_array[allocated_slot];
    pub_item.allocated_slot = allocated_slot;
    return pub_item;
}


// ---- INTERNAL: Post-publish message destruction callback ----
void post_publish_diag_msg_destroy(rcl_publisher_t *publisher, void *message, void *param)
{
    uint32_t allocated_slot = (uint32_t) param;
    destroy_uros_diag_status_msg(allocated_slot);
    destroy_diag_kv_pair(allocated_slot);
    destroy_diag_kv_pair_refs(allocated_slot);
    destroy_diag_msg_object(allocated_slot);
    deallocate_slots(allocated_slot);
}


// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, std::string_view hw_name, std::string_view hw_id, std::string_view msg, std::vector<diag_kv_pair_item_t> *kv_pairs)
{
    if (diag_pub_enabled)
    {
        char buffer[120];
        snprintf(buffer, sizeof(buffer), "Diagnostic report! [hwname: %s, hwid: %s, lvl: %d]", hw_name.data(), hw_id.data(), level);
        write_log(buffer, LOG_LVL_WARN, FUNCNAME_ONLY);

        /* 
            rt_check_mode is intentionally set to log-only to prevent infinite recursion of publish_diag_report().
            This edge case may occur if MicroROS is not initialized properly when check_rc() is called 
            (this could happen if check_rc() fails for rclc_node_init_default(), for example).
            In this case, this publish function would also not return RCL_RET_OK, resulting in its
            check_rc() calling publish_diag_report() and then the same thing happening over and over again.
        */
        diag_publish_item_t pub_item_diag = prepare_diag_publish_item(level, hw_name, hw_id, msg, kv_pairs);
        uRosPublishingHandler::PublishItem_t pub_item;
        
        pub_item.message = pub_item_diag.diag_msg;
        pub_item.publisher = &diagnostics_pub;
        pub_item.post_pub_user_param = (void *) pub_item_diag.allocated_slot;
        pub_item.post_pub_callback = post_publish_diag_msg_destroy;
        pub_item.rt_check_mode = RT_LOG_ONLY_CHECK;
        
        xQueueSendToBack(pub_queue, (void *) &pub_item, 0);
    }
}


// ---- Temporarily disable publish_diag_report() ----
void disable_diag_pub()
{
    diag_pub_enabled = false;
}


// ---- Re-enable publish_diag_report() ----
void enable_diag_pub()
{
    diag_pub_enabled = true;
}