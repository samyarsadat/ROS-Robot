/*
    The ROS robot project - Common Program Definitions
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

#pragma once


// ------- Diagnostics Sources -------
#define DIAG_SOURCE_MAIN_BOARD   (char*) "main_board"
#define DIAG_SOURCE_MOTOR_BOARD  (char*) "motor_board"
#define DIAG_SOURCE_POWER_BOARD  (char*) "power_board"


// ------- Diagnostics Hardware Names -------
#define DIAG_HWNAME_MOTOR_CTRL_L   (char*) "motor_control_left"
#define DIAG_HWNAME_MOTOR_CTRL_R   (char*) "motor_control_right"
#define DIAG_HWNAME_ULTRASONICS    (char*) "ultrasonic_sensors"
#define DIAG_HWNAME_ENV_SENSORS    (char*) "env_sensors"
#define DIAG_HWNAME_IR_EDGE_FRONT  (char*) "ir_edge_sensors_front"
#define DIAG_HWNAME_IR_EDGE_BACK   (char*) "ir_edge_sensors_back"
#define DIAG_HWNAME_BATT_MON       (char*) "battery_monitoring"
#define DIAG_HWNAME_SYS_PWR_MGMT   (char*) "system_power_management"
#define DIAG_HWNAME_USER_INPUTS    (char*) "user_inputs"
#define DIAG_HWNAME_UCONTROLLERS   (char*) "microcontrollers"
#define DIAG_HWNAME_UROS           (char*) "microros"


// ------- Diagnostics Hardware IDs -------
#define DIAG_HWID_MOTOR_ENC_L1    (char*) "motor_encoder_left_1"
#define DIAG_HWID_MOTOR_ENC_L2    (char*) "motor_encoder_left_2"
#define DIAG_HWID_MOTOR_ENC_R1    (char*) "motor_encoder_right_1"
#define DIAG_HWID_MOTOR_ENC_R2    (char*) "motor_encoder_right_2"
#define DIAG_HWID_MOTOR_DRV_L     (char*) "motor_driver_left"
#define DIAG_HWID_MOTOR_DRV_R     (char*) "motor_driver_right"
#define DIAG_HWID_ULTRASONIC_F    (char*) "ultrasonic_front"
#define DIAG_HWID_ULTRASONIC_B    (char*) "ultrasonic_back"
#define DIAG_HWID_ULTRASONIC_R    (char*) "ultrasonic_right"
#define DIAG_HWID_ULTRASONIC_L    (char*) "ultrasonic_left"
#define DIAG_HWID_ENV_DHT11       (char*) "env_sens_dht11"
#define DIAG_HWID_ENV_IMU         (char*) "env_sens_imu"
#define DIAG_HWID_IR_EDGE_F1      (char*) "ir_edge_front_sens_1"
#define DIAG_HWID_IR_EDGE_F2      (char*) "ir_edge_front_sens_2"
#define DIAG_HWID_IR_EDGE_F3      (char*) "ir_edge_front_sens_3"
#define DIAG_HWID_IR_EDGE_F4      (char*) "ir_edge_front_sens_4"
#define DIAG_HWID_IR_EDGE_B1      (char*) "ir_edge_back_sens_1"
#define DIAG_HWID_IR_EDGE_B2      (char*) "ir_edge_back_sens_2"
#define DIAG_HWID_IR_EDGE_B3      (char*) "ir_edge_back_sens_3"
#define DIAG_HWID_IR_EDGE_B4      (char*) "ir_edge_back_sens_4"
#define DIAG_HWID_IR_EDGE_FADC    (char*) "ir_edge_front_adc"
#define DIAG_HWID_IR_EDGE_BADC    (char*) "ir_edge_back_adc"
#define DIAG_HWID_BATT_PWR_MON    (char*) "batt_power_monitor"
#define DIAG_HWID_BATT_TEMP_MON   (char*) "batt_temp_monitor"
#define DIAG_HWID_SYSPWR_12V_MON  (char*) "sys_power_12v_bus_monitor"
#define DIAG_HWID_SYSPWR_5V_MON   (char*) "sys_power_5v_bus_monitor"
#define DIAG_HWID_SYSPWR_3v3_MON  (char*) "sys_power_3v3_bus_monitor"
#define DIAG_HWID_SYSPWR_12V_REG  (char*) "sys_power_12v_regulator"
#define DIAG_HWID_SYSPWR_5V_REG   (char*) "sys_power_5v_regulator"
#define DIAG_HWID_SYSPWR_3v3_REG  (char*) "sys_power_3v3_regulator"
#define DIAG_HWID_UI_NEXTION      (char*) "user_interface_nextion"
#define DIAG_HWID_MCU_MABO_A      (char*) "mcu_mainboard_rp2040_a"
#define DIAG_HWID_MCU_MABO_B      (char*) "mcu_mainboard_rp2040_b"
#define DIAG_HWID_MCU_POBO        (char*) "mcu_powerboard_rp2040"
#define DIAG_HWID_MCU_MOTBO       (char*) "mcu_motorboard_rp2040"
#define DIAG_HWID_UROS            (char*) "microros"