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
#define DIAG_HWNAME_MOTOR_CTRL_L   "motor_control_left"
#define DIAG_HWNAME_MOTOR_CTRL_R   "motor_control_right"
#define DIAG_HWNAME_ULTRASONICS    "ultrasonic_sensors"
#define DIAG_HWNAME_ENV_SENSORS    "env_sensors"
#define DIAG_HWNAME_IR_EDGE_FRONT  "ir_edge_sensors_front"
#define DIAG_HWNAME_IR_EDGE_BACK   "ir_edge_sensors_back"
#define DIAG_HWNAME_BATT_MON       "battery_monitoring"
#define DIAG_HWNAME_SYS_PWR_MGMT   "system_power_management"
#define DIAG_HWNAME_USER_INPUTS    "user_inputs"
#define DIAG_HWNAME_UCONTROLLERS   "microcontrollers"
#define DIAG_HWNAME_UROS           "microros"


// ------- Diagnostics Hardware IDs -------
#define DIAG_HWID_MOTOR_ENC_L1    "motor_encoder_left_1"
#define DIAG_HWID_MOTOR_ENC_L2    "motor_encoder_left_2"
#define DIAG_HWID_MOTOR_ENC_R1    "motor_encoder_right_1"
#define DIAG_HWID_MOTOR_ENC_R2    "motor_encoder_right_2"
#define DIAG_HWID_MOTOR_DRV_L     "motor_driver_left"
#define DIAG_HWID_MOTOR_DRV_R     "motor_driver_right"
#define DIAG_HWID_ULTRASONIC_F    "ultrasonic_front"
#define DIAG_HWID_ULTRASONIC_B    "ultrasonic_back"
#define DIAG_HWID_ULTRASONIC_R    "ultrasonic_right"
#define DIAG_HWID_ULTRASONIC_L    "ultrasonic_left"
#define DIAG_HWID_ENV_DHT11       "env_sens_dht11"
#define DIAG_HWID_ENV_IMU         "env_sens_imu"
#define DIAG_HWID_IR_EDGE_F1      "ir_edge_front_sens_1"
#define DIAG_HWID_IR_EDGE_F2      "ir_edge_front_sens_2"
#define DIAG_HWID_IR_EDGE_F3      "ir_edge_front_sens_3"
#define DIAG_HWID_IR_EDGE_F4      "ir_edge_front_sens_4"
#define DIAG_HWID_IR_EDGE_B1      "ir_edge_back_sens_1"
#define DIAG_HWID_IR_EDGE_B2      "ir_edge_back_sens_2"
#define DIAG_HWID_IR_EDGE_B3      "ir_edge_back_sens_3"
#define DIAG_HWID_IR_EDGE_B4      "ir_edge_back_sens_4"
#define DIAG_HWID_IR_EDGE_FADC    "ir_edge_front_adc"
#define DIAG_HWID_IR_EDGE_BADC    "ir_edge_back_adc"
#define DIAG_HWID_BATT_PWR_MON    "batt_power_monitor"
#define DIAG_HWID_BATT_TEMP_MON   "batt_temp_monitor"
#define DIAG_HWID_SYSPWR_12V_MON  "sys_power_12v_bus_monitor"
#define DIAG_HWID_SYSPWR_5V_MON   "sys_power_5v_bus_monitor"
#define DIAG_HWID_SYSPWR_3v3_MON  "sys_power_3v3_bus_monitor"
#define DIAG_HWID_SYSPWR_12V_REG  "sys_power_12v_regulator"
#define DIAG_HWID_SYSPWR_5V_REG   "sys_power_5v_regulator"
#define DIAG_HWID_SYSPWR_3v3_REG  "sys_power_3v3_regulator"
#define DIAG_HWID_UI_NEXTION      "user_interface_nextion"
#define DIAG_HWID_MCU_MABO_A      "mcu_mainboard_rp2040_a"
#define DIAG_HWID_MCU_MABO_B      "mcu_mainboard_rp2040_b"
#define DIAG_HWID_MCU_POBO        "mcu_powerboard_rp2040"
#define DIAG_HWID_MCU_MOTBO       "mcu_motorboard_rp2040"
#define DIAG_HWID_UROS            "microros"