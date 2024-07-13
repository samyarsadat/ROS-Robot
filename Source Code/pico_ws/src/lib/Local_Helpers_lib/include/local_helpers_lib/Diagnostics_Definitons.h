/*
    The ROS robot project - Local Helper/commonly used functions
    Diagnostics Hardware Name/ID definitions
    
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

#pragma once


// ------- Diagnostics Hardware Names -------
#define DIAG_NAME_MOTOR          "motors"
#define DIAG_NAME_ULTRASONICS    "ultrasonic_sensors"
#define DIAG_NAME_ENV_SENSORS    "env_sensors"
#define DIAG_NAME_IR_EDGE        "ir_edge_sensors"
#define DIAG_NAME_BATT_MON       "battery_monitoring"
#define DIAG_NAME_SYS_PWR_MGMT   "power_management"
#define DIAG_NAME_SYSTEM         "system"   // Firmware-related diagnostics
#define DIAG_NAME_MCU            "mcus"


// ------- Diagnostics Hardware IDs -------

// ---- Motors ----
#define DIAG_ID_MOTOR_DRV_L     "motor_driver_left"
#define DIAG_ID_MOTOR_DRV_R     "motor_driver_right"
#define DIAG_ID_MOTOR_CTRL_L    "motor_controller_left"
#define DIAG_ID_MOTOR_CTRL_R    "motor_controller_right"

// ---- Ultrasonic Sensors ----
#define DIAG_ID_ULTRASONIC_F    "ultrasonic_front"
#define DIAG_ID_ULTRASONIC_B    "ultrasonic_back"
#define DIAG_ID_ULTRASONIC_R    "ultrasonic_right"
#define DIAG_ID_ULTRASONIC_L    "ultrasonic_left"

// ---- Environmental Sensors ----
#define DIAG_ID_ENV_DHT11       "env_sens_dht11"
#define DIAG_ID_ENV_IMU         "env_sens_imu"

// ---- IR Edge Sensors ----
#define DIAG_ID_IR_EDGE_F       "ir_edge_front"
#define DIAG_ID_IR_EDGE_B       "ir_edge_back"
#define DIAG_ID_IR_EDGE_FADC    "ir_edge_front_adc"
#define DIAG_ID_IR_EDGE_BADC    "ir_edge_back_adc"

// ---- Battery Monitoring ----
#define DIAG_ID_BATT_PWR_MON    "batt_power_monitor"
#define DIAG_ID_BATT_TEMP_MON   "batt_temp_monitor"

// ---- Power Management ----
#define DIAG_ID_SYSPWR_12V_MON  "sys_power_12v_bus_monitor"
#define DIAG_ID_SYSPWR_5V_MON   "sys_power_5v_bus_monitor"
#define DIAG_ID_SYSPWR_3v3_MON  "sys_power_3v3_bus_monitor"
#define DIAG_ID_SYSPWR_12V_REG  "sys_power_12v_regulator"
#define DIAG_ID_SYSPWR_5V_REG   "sys_power_5v_regulator"
#define DIAG_ID_SYSPWR_3v3_REG  "sys_power_3v3_regulator"

// ---- System ----
#define DIAG_ID_SYS_GENERAL     "system_general"
#define DIAG_ID_SYS_TIMERS      "system_timers"
#define DIAG_ID_SYS_UROS        "microros"

// ---- MCUs ----
#define DIAG_ID_MCU_MABO_A      "mcu_mainboard_rp2040_a"
#define DIAG_ID_MCU_MABO_B      "mcu_mainboard_rp2040_b"
#define DIAG_ID_MCU_POBO        "mcu_powerboard_rp2040"
#define DIAG_ID_MCU_MOTBO       "mcu_motorboard_rp2040"