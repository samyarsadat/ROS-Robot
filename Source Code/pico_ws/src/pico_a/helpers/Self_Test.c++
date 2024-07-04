/*
    The ROS robot project - Self Test Module - Pico A
    Copyright 2022-2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2022-2024.
 
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

// TODO: THIS MODULE IS TODO AND NOT HIGH-PRIORITY!

// ------- Libraries & Modules -------
#include "pico/stdlib.h"
#include "helpers_lib/Helpers.h"
#include "IO_Helpers_Mux.h"
#include "IO_Helpers_Edge.h"
#include "local_helpers_lib/Local_Helpers.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/srv/self_test.h>


// ------- Global variables -------
extern bool self_test_mode;


// ---- Run self-test functions service callback ----
void run_self_test_callback(const void *req, void *res)
{
    diagnostic_msgs__srv__SelfTest_Request *req_in = (diagnostic_msgs__srv__SelfTest_Request *) req;
    diagnostic_msgs__srv__SelfTest_Response *res_in = (diagnostic_msgs__srv__SelfTest_Response *) res;
    std::vector<diagnostic_msgs__msg__DiagnosticStatus> diag_status_reports;

    write_log("Received self-test request.", LOG_LVL_INFO, FUNCNAME_ONLY);
    self_test_mode = true;

    // Perform IR edge sensor self-test
    write_log("Performing IR edge sensor self-test...", LOG_LVL_INFO, FUNCNAME_ONLY);
    std::vector<bool> ir_test_results = ir_self_test();

    for (auto result : ir_test_results)
    {
        if (result)
        {
            //std::string message = "IR edge sensor self-test passed. Sensor ID: " + std::to_string(result);
            //diag_status_reports.push_back(create_diag_msg(DIAG_LVL_OK, DIAG_HWNAME_IR_EDGE_FRONT, DIAG_HWID_IR_EDGE_B1, ))
        }
    }
}