/*
    The ROS robot project - Raspberry Pi Pico (A) program
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


// ------- Libraries & Modules -------
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <sensor_msgs/msg/range.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "lib/PID_lib/PID_v1.h"
#include "lib/Helper_lib/Helpers.h"
#include "lib/Motor_lib/Motor.h"
#include "lib/Motor_lib/Motor_Safety.h"
#include "Pico_A_helpers/IO_Helpers_Mux.h"
#include "Pico_A_helpers/IO_Helpers_Ultrasonic.h"
#include "Pico_A_helpers/IO_Helpers_Edge.h"
#include "Pico_A_helpers/Self_Test.c++"
#include "Pico_A_helpers/Sensor_Publishers.c++"
#include "Pico_A_helpers/uROS_Init.h"
#include "Pico_A_helpers/Definitions.h"
#include "pico_uart_transports.h"
#include <rmw_microros/rmw_microros.h>



// ------- Global variables -------

// ---- Misc. ----
bool halt_core_0 = false;
bool self_test_mode = false;
alarm_pool_t *core_1_alarm_pool;

// ---- Loop time ----
uint32_t loop_time, loop_1_time;

// ---- Timers ----
bool ultrasonic_ir_edge_rt_active = false;
struct repeating_timer motor_odom_rt, ultrasonic_publish_rt, edge_ir_publish_rt, other_sensors_publish_rt;



// ------- Library object inits -------

// ---- Motor Controller ----
uint r_motors_drv_pins[2] = {r_motor_drive_1, r_motor_drive_2};
uint l_motors_drv_pins[2] = {l_motor_drive_1, l_motor_drive_2};
MotorDriver r_motors_driver(r_motors_drv_pins, 2, MotorDriver::driver_type::GENERIC_PWM);
MotorDriver l_motors_driver(l_motors_drv_pins, 2, MotorDriver::driver_type::GENERIC_PWM);

// IRQ callback prototype
void irq_call(uint pin, uint32_t events);

MotorEncoder r_motor_1_enc(r_motor_1_enc_a, r_motor_1_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder r_motor_2_enc(r_motor_2_enc_a, r_motor_2_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder l_motor_1_enc(l_motor_1_enc_a, l_motor_1_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder l_motor_2_enc(l_motor_2_enc_a, l_motor_2_enc_b, motor_gear_ratio, enc_pulses_per_rotation, &irq_call);
MotorEncoder* r_motor_encs[2] = {&r_motor_1_enc, &r_motor_2_enc};
MotorEncoder* l_motor_encs[2] = {&l_motor_1_enc, &l_motor_2_enc};

Motor r_motors(&r_motors_driver, r_motor_encs, 2);
Motor l_motors(&l_motors_driver, l_motor_encs, 2);
MotorSafety r_motors_safety(&r_motors, right_motor_controller_id);
MotorSafety l_motors_safety(&l_motors, left_motor_controller_id);



// ------- Functions ------- 

// ---- RCL return checker prototype ----
bool check_rc(rcl_ret_t rctc, uint mode);


// ---- Diagnostics error reporting ----
void publish_diag_report(uint8_t level, char *hw_source, char *hw_name, char *hw_id, char *msg, diagnostic_msgs__msg__KeyValue *key_values)
{
    sprintf(diagnostics_msg.name.data, "%s/%s", hw_source, hw_name);
    diagnostics_msg.name.size = strlen(diagnostics_msg.name.data);
    diagnostics_msg.hardware_id.data = hw_id;
    diagnostics_msg.hardware_id.size = strlen(diagnostics_msg.hardware_id.data);
    diagnostics_msg.message.data = msg;
    diagnostics_msg.message.size = strlen(diagnostics_msg.message.data);
    diagnostics_msg.values.data = NULL;
    diagnostics_msg.values.size = 0;
    diagnostics_msg.level = level;
    
    if (key_values != NULL)
    {
        diagnostics_msg.values.data = key_values;
        diagnostics_msg.values.size = sizeof(key_values) / sizeof(key_values[0]);
    }

    check_rc(rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL), RCL_HARD_CHECK);
}


// ---- Graceful shutdown ----
void clean_shutdown(const void *msgin)
{
    // Stop all repeating timers
    cancel_repeating_timer(&motor_odom_rt);
    cancel_repeating_timer(&ultrasonic_publish_rt);
    cancel_repeating_timer(&edge_ir_publish_rt);
    cancel_repeating_timer(&other_sensors_publish_rt);


    // IO cleanup
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);
    gpio_put(power_led, LOW);
    gpio_put(onboard_led, LOW);
        
    l_motors.disable_controller();
    r_motors.disable_controller();
        
    set_mux_addr(0);
    set_mux_io_mode(OUTPUT);
    gpio_put(analog_mux_io, LOW);
    init_pin(edge_sens_en, OUTPUT);
    gpio_put(edge_sens_en, LOW);

    // MicroROS cleanup
    // TODO: ADD ALL PUBS AND SUBS.
    check_rc(rcl_subscription_fini(&cmd_vel_sub, &rc_node), RCL_LOG_ONLY_CHECK);
    check_rc(rcl_publisher_fini(&diagnostics_pub, &rc_node), RCL_LOG_ONLY_CHECK);
    check_rc(rclc_executor_fini(&rc_executor), RCL_LOG_ONLY_CHECK);
    check_rc(rcl_node_fini(&rc_node), RCL_LOG_ONLY_CHECK);


    // Stop core 0 (only effective when this function is called from core 1)
    halt_core_0 = true;

    // Stop core 1 if this function is being called from core 0.
    if (get_core_num() == 0)
    {
        multicore_reset_core1();
    }

    while (true)
    {
        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(100);
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(100);
    }
}


// ---- RCL return checker ----
bool check_rc(rcl_ret_t rctc, uint mode)
{
    if (rctc != RCL_RET_OK)
    {
        switch (mode)
        {
            case RCL_HARD_CHECK:
                clean_shutdown(NULL);
                break;

            case RCL_SOFT_CHECK:
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_UROS, DIAG_HWID_UROS, DIAG_ERR_MSG_UROS_RC_CHECK_FAIL, NULL);
                return false;
                break;
            
            case RCL_LOG_ONLY_CHECK:
                // TODO: LOGGING
                return false;
                break;

            default:
                break;
        }
    }

    return true;
}


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    r_motor_1_enc.enc_hardware_irq_trigger(pin);
    r_motor_2_enc.enc_hardware_irq_trigger(pin);
    l_motor_1_enc.enc_hardware_irq_trigger(pin);
    l_motor_2_enc.enc_hardware_irq_trigger(pin);
}


// ---- MotorSafety trigger callback ----
void motor_safety_callback(MotorSafety::safety_trigger_conditions condition, int id)
{
    if (!self_test_mode)
    {
        if (condition == MotorSafety::safety_trigger_conditions::SET_VS_ACTUAL_DIR_DIFF)
        {
            if (id == right_motor_controller_id)
            {
                r_motors.set_direction_reversed(!r_motors.get_dir_reversed());
            }

            else if (id == left_motor_controller_id)
            {
                l_motors.set_direction_reversed(!l_motors.get_dir_reversed());
            }
        }

        else
        {
            if (id == right_motor_controller_id)
            {
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_DRV_R, DIAG_ERR_MSG_MOTOR_SAFETY, NULL);
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_ENC_R1, DIAG_ERR_MSG_MOTOR_SAFETY, NULL);
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_ENC_R2, DIAG_ERR_MSG_MOTOR_SAFETY, NULL);
            }
            
            else
            {
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_DRV_L, DIAG_ERR_MSG_MOTOR_SAFETY, NULL);
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_ENC_L1, DIAG_ERR_MSG_MOTOR_SAFETY, NULL);
                publish_diag_report(diagnostic_msgs__msg__DiagnosticStatus__ERROR, DIAG_SOURCE_MAIN_BOARD, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_ENC_L2, DIAG_ERR_MSG_MOTOR_SAFETY, NULL);
            }

            clean_shutdown(NULL);
        }
    }
}



// ------- MicroROS subscriber & service callbacks ------- 

// ---- Enable/disable motor controller service ----
void en_motor_ctrl_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    if ((bool) req_in->data)
    {
        r_motors.enable_controller();
        l_motors.enable_controller();
    }

    else 
    {
        r_motors.disable_controller();
        l_motors.disable_controller();
    }

    res_in->success = true;
}


// ---- Enable/disable emitters (ultrasonic, IR edge) service ----
void en_emitters_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    if ((bool) req_in->data && !ultrasonic_ir_edge_rt_active)
    {
        ultrasonic_ir_edge_rt_active = true;
        alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, edge_ir_pub_rt_interval, publish_edge_ir, NULL, &edge_ir_publish_rt);
        alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, ultra_pub_rt_interval, publish_ultra, NULL, &ultrasonic_publish_rt);
    }

    else 
    {
        ultrasonic_ir_edge_rt_active = false;
        cancel_repeating_timer(&ultrasonic_publish_rt);
        cancel_repeating_timer(&edge_ir_publish_rt);
        gpio_put(ir_en_pin, LOW);
    }

    res_in->success = true;
}


// ---- Command velocity topic callback ----
void cmd_vel_call(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;
    
    float angular = msg -> angular.z;     // rad/s
    float linear = msg -> linear.x;       // m/s

    float motor_l_speed_ms = linear - (angular * ((wheelbase / 1000) / 2));               // Calculate motor speeds in m/s
    float motor_r_speed_ms = linear + (angular * ((wheelbase / 1000) / 2));

    r_motors.set_pid_ctrl_speed((motor_r_speed_ms / (wheel_circumference / 1000)) * 60);  // Convert motor speeds from m/s to RPM
    l_motors.set_pid_ctrl_speed((motor_l_speed_ms / (wheel_circumference / 1000)) * 60);           
}



// ------- Main program -------

// ----- Timer tasks -----
bool motor_control_and_odom_timer_callback(struct repeating_timer *rt)
{
    // Calculate and set motor outputs
    r_motors.compute_outputs();
    l_motors.compute_outputs();

    // Calculate and publish odometry data
    enc_odom_msg.twist.twist.linear.x;

    return true;
}

bool publish_other_sensors_timer_callback(struct repeating_timer *rt)
{
    return true;
}


// ---- Setup function (Runs once on core 0) ----
void setup()
{
    // ---- Pin init ----
    init_pin(ready_sig, INPUT);
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);
    init_pin(pi_power_relay, OUTPUT);
    init_pin(edge_sens_en, OUTPUT);

    // Mux init
    set_mux_pins(analog_mux_s0, analog_mux_s1, analog_mux_s2, analog_mux_s3, analog_mux_io);

    // MotorSafety init
    r_motors_safety.configure_safety(motor_single_side_max_difference, motor_set_vs_actual_max_difference, motor_safety_trigger_timeout, &motor_safety_callback);
    l_motors_safety.configure_safety(motor_single_side_max_difference, motor_set_vs_actual_max_difference, motor_safety_trigger_timeout, &motor_safety_callback);
    r_motors_safety.set_set_vs_actual_spd_time_tolerance(motor_actual_vs_set_extra_timeout);
    l_motors_safety.set_set_vs_actual_spd_time_tolerance(motor_actual_vs_set_extra_timeout);

    // MicroROS init
    init_subs_pubs();
    exec_init();
    uros_init(UROS_NODE_NAME, UROS_NODE_NAMESPACE);
    rclc_executor_spin(&rc_executor);

    // Repeating timers
    add_repeating_timer_ms(motor_odom_rt_interval, motor_control_and_odom_timer_callback, NULL, &motor_odom_rt);
}


// ---- Setup function (Runs once on core 1) ----
void setup1()
{
    // Create alarm pool for core 1 timers
    core_1_alarm_pool = alarm_pool_create(4, 16);

    // Repeating timers
    ultrasonic_ir_edge_rt_active = true;
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, edge_ir_pub_rt_interval, publish_edge_ir, NULL, &edge_ir_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, ultra_pub_rt_interval, publish_ultra, NULL, &ultrasonic_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sensors_pub_rt_interval, publish_other_sensors_timer_callback, NULL, &other_sensors_publish_rt);
}


// ---- Main loop (Runs forever on core 0) ----
void loop()
{
    loop_time = time_us_32();

    if (time_us_32() - loop_time > loop_time_max)
    {
        clean_shutdown(NULL);
    }
}


// ---- Main loop (Runs forever on core 1) ----
void loop1()
{
    loop_1_time = time_us_32();

    if (time_us_32() - loop_1_time > loop_1_time_max)
    {
        clean_shutdown(NULL);
    }
}



// ******** END OF MAIN PROGRAM *********
// *********** STARTUP & INIT ***********

// ---- Core 1 main function ----
void main_core_1()
{
    setup1();

    // Start core 1 loop
    while (true)
    {
        loop1();
    }
}


// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    multicore_reset_core1();

    // Wait for 5 seconds
    // TODO: Should wait for signal from Raspberry Pi computer.
    for (int i = 0; i < 5; i++)
    {
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(500);
        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(500);
    }

    setup();
    multicore_launch_core1(main_core_1);

    gpio_put(power_led, HIGH);

    // Start core 0 loop
    while (!halt_core_0)
    {
        loop();
    }

    return 0;
}