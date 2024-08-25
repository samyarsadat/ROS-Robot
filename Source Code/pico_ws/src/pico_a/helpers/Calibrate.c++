/*
    The ROS robot project - Calibration routines module - Pico A
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
#include <rrp_pico_coms/srv/run_calibrations_a.h>
#include "IO_Helpers_Edge.h"



// ------- Global variables -------
extern mpu6050_t mpu6050;



// ---- Run calibration functions service callback ----
void run_calib_callback(const void *req, void *res)
{
    rrp_pico_coms__srv__RunCalibrationsA_Request *req_in = (rrp_pico_coms__srv__RunCalibrationsA_Request *) req;
    rrp_pico_coms__srv__RunCalibrationsA_Response *res_in = (rrp_pico_coms__srv__RunCalibrationsA_Response *) res;

    write_log("Received calibration request.", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Perform IMU calibration
    if (req_in->calib_imu)
    {
        write_log("Calibrating IMU gyroscope...", LOG_LVL_INFO, FUNCNAME_ONLY);
        mpu6050_calibrate_gyro(&mpu6050, mpu6050_gyro_calib_cycles);
    }

    // Perform IR edge sensor ambient IR level calibration
    if (req_in->calib_ir_edge)
    {
        write_log("Calibrating IR edge sensor offset...", LOG_LVL_INFO, FUNCNAME_ONLY);

        if (!calibrate_ir_offset())
        {
            write_log("IR edge sensor calibration failed!", LOG_LVL_ERROR, FUNCNAME_ONLY);
            res_in->message.data = (char *) "IR edge sensor calibration failed!";
            res_in->message.size = strlen(res_in->message.data);
            res_in->success = false;
            return;
        }
    }

    // Perform left motor PID auto-tuning (TODO)
    if (req_in->calib_pid_left)
    {
        write_log("Tuning left motor PID...", LOG_LVL_INFO, FUNCNAME_ONLY);

        //if (!l_motors.pid_autotune())
        if (false)
        {
            write_log("Left motor PID auto-tuning failed!", LOG_LVL_ERROR, FUNCNAME_ONLY);
            res_in->message.data = (char *) "Left motor PID auto-tuning failed!";
            res_in->message.size = strlen(res_in->message.data);
            res_in->success = false;
            return;
        }
    }

    // Perform right motor PID auto-tuning (TODO)
    if (req_in->calib_pid_right)
    {
        write_log("Tuning right motor PID...", LOG_LVL_INFO, FUNCNAME_ONLY);

        //if (!r_motors.pid_autotune())
        if (false)
        {
            write_log("Right motor PID auto-tuning failed!", LOG_LVL_ERROR, FUNCNAME_ONLY);
            res_in->message.data = (char *) "Right motor PID auto-tuning failed!";
            res_in->message.size = strlen(res_in->message.data);
            res_in->success = false;
            return;
        }
    }

    res_in->success = true;
}