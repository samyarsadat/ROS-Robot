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
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <FreeRTOS_POSIX.h>
#include <geometry_msgs/msg/twist.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <sensor_msgs/msg/range.h>
#include <rrp_pico_coms/msg/motor_ctrl_state.h>
#include <rrp_pico_coms/srv/set_pid_tunings.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "helpers_lib/Helpers.h"
#include "motor_control_lib/Motor.h"
#include "motor_control_lib/Motor_Safety.h"
#include "haw/MPU6050.h"
#include "helpers/IO_Helpers_Mux.h"
#include "helpers/IO_Helpers_Ultrasonic.h"
#include "helpers/IO_Helpers_Edge.h"
#include "helpers/Self_Test.c++"
#include "helpers/Sensor_Publishers.c++"
#include "helpers/uROS_Init.h"
#include "helpers/Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "../uart_transport/pico_uart_transports.h"
#include <rmw_microros/rmw_microros.h>
#include <iterator>
#include <vector>
#include <cmath>
#include <task.h>


void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    panic("Stack overflow. Task: %s\n", pcTaskName);
}

void vApplicationMallocFailedHook()
{
    panic("malloc failed");
}




// ------- Global variables -------

// ---- Misc. ----
bool halt_core_0 = false;
bool self_test_mode = false;
alarm_pool_t *core_1_alarm_pool;

// ---- MicroROS state ----
enum UROS_STATE {WAITING_FOR_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED};
UROS_STATE current_uros_state = WAITING_FOR_AGENT;

// ---- Timer execution times storage (milliseconds) ----
uint32_t last_motor_odom_time, last_ultrasonic_publish_time, last_edge_ir_publish_time, last_other_sensors_publish_time;
uint32_t last_uros_exec_time;

// ---- Motor encoder counter storage ----
int32_t enc_r_count_old, enc_l_count_old, total_enc_avg_travel;
int32_t enc_odom_x_pos, enc_odom_y_pos;
float theta;

// ---- Timers ----
bool ultrasonic_ir_edge_rt_active = false;
struct repeating_timer motor_odom_rt, ultrasonic_publish_rt, edge_ir_publish_rt, other_sensors_publish_rt;



// ------- Library object inits -------

// ---- MPU6050 ----
mpu6050_t mpu6050 = mpu6050_init(i2c_inst, MPU6050_ADDRESS_A0_VCC);

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

// ---- Graceful shutdown ----
void clean_shutdown()
{
    write_log("clean_shutdown", "A clean shutdown has been triggered. The program will now shut down.", LOG_LVL_FATAL);

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
    rcl_service_fini(&en_motor_ctrl_srv, &rc_node);
    rcl_service_fini(&en_emitters_srv, &rc_node);
    rcl_service_fini(&en_relay_srv, &rc_node);
    rcl_service_fini(&set_mtr_pid_tunings_srv, &rc_node);
    rcl_service_fini(&run_self_test_srv, &rc_node);
    rcl_subscription_fini(&cmd_vel_sub, &rc_node);
    rcl_subscription_fini(&e_stop_sub, &rc_node);
    rcl_publisher_fini(&enc_odom_pub, &rc_node);
    rcl_publisher_fini(&diagnostics_pub, &rc_node);
    rcl_publisher_fini(&misc_sensor_pub, &rc_node);
    rcl_publisher_fini(&ultrasonic_sensor_pub, &rc_node);
    rcl_publisher_fini(&falloff_sensor_pub, &rc_node);
    rcl_publisher_fini(&mtr_ctrl_r_state_pub, &rc_node);
    rcl_publisher_fini(&mtr_ctrl_l_state_pub, &rc_node);
    rcl_publisher_fini(&odom_baselink_tf_pub, &rc_node);
    rclc_executor_fini(&rc_executor);
    rcl_node_fini(&rc_node);
    rclc_support_fini(&rc_supp);


    // Stop core 0 (only effective when this function is called from core 1)
    halt_core_0 = true;

    // Stop core 1 if this function is being called from core 0.
    if (get_core_num() == 0)
    {
        multicore_reset_core1();
    }

    while (true)
    {
        write_log("clean_shutdown", "Program shutdown hang loop.", LOG_LVL_INFO);

        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(100);
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(100);
    }
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
    char buffer[50];
    sprintf(buffer, "Motor Safety triggered: [code: %d, id: %d]", static_cast<int>(condition), id);
    write_log("motor_safety_callback", buffer, LOG_LVL_INFO);

    if (!self_test_mode)
    {
        // TODO: This could cause the motors to jitter back and forth if a cooldown period is not added!
        if (condition == MotorSafety::safety_trigger_conditions::SET_VS_ACTUAL_DIR_DIFF)
        {
            if (id == right_motor_controller_id)
            {
                write_log("motor_safety_callback", "SET_VS_ACTUAL_DIR_DIFF triggered. Reversing control direction. [Right]", LOG_LVL_WARN);
                r_motors.set_direction_reversed(!r_motors.get_dir_reversed());
            }

            else
            {
                write_log("motor_safety_callback", "SET_VS_ACTUAL_DIR_DIFF triggered. Reversing control direction. [Left]", LOG_LVL_WARN);
                l_motors.set_direction_reversed(!l_motors.get_dir_reversed());
            }
        }

        else
        {
            if (id == right_motor_controller_id)
            {
                write_log("motor_safety_callback", "Critical Motor Safety condition triggered! Shutting down. [Right]", LOG_LVL_FATAL);
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_DRV_R, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_ENC_R1, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_R, DIAG_HWID_MOTOR_ENC_R2, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
            }
            
            else
            {
                write_log("motor_safety_callback", "Critical Motor Safety condition triggered! Shutting down. [Left]", LOG_LVL_FATAL);
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_DRV_L, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_ENC_L1, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
                publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_MOTOR_CTRL_L, DIAG_HWID_MOTOR_ENC_L2, DIAG_ERR_MSG_MOTOR_SAFETY, DIAG_KV_EMPTY());
            }

            clean_shutdown();  // This is a critical failure, so we'll call the shutdown function without waiting for an e-stop signal.
        }
    }
}



// ------- MicroROS subscriber & service callbacks ------- 

// ---- Enable/disable motor controller service ----
void en_motor_ctrl_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    // TODO: Add logging.

    if (req_in->data)
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

    // TODO: Add logging.

    if (req_in->data && !ultrasonic_ir_edge_rt_active)
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


// ---- Enable/disable relay service ----
void en_relay_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    // TODO: Add logging.

    if (req_in->data)
    {
        gpio_put(pi_power_relay, HIGH);
    }

    else 
    {
        gpio_put(pi_power_relay, LOW);
    }

    res_in->success = true;
}


// ---- Set motor controller PID tunings service ----
void set_mtr_pid_tunings_callback(const void *req, void *res)
{
    rrp_pico_coms__srv__SetPidTunings_Request *req_in = (rrp_pico_coms__srv__SetPidTunings_Request *) req;
    rrp_pico_coms__srv__SetPidTunings_Response *res_in = (rrp_pico_coms__srv__SetPidTunings_Response *) res;

    char buffer[65];
    sprintf(buffer, "Received set_pid_tunings: [Kp: %.3f, Ki: %.3f, Kd: %.3f]", req_in->pid_kp, req_in->pid_ki, req_in->pid_kd);
    write_log("set_mtr_pid_tunings_callback", buffer, LOG_LVL_INFO);

    r_motors.pid->SetTunings(req_in->pid_kp, req_in->pid_ki, req_in->pid_kd);
    l_motors.pid->SetTunings(req_in->pid_kp, req_in->pid_ki, req_in->pid_kd);

    res_in->success = true;
}


// ---- Run self-test functions service ----
void run_self_test_callback(const void *req, void *res)
{

}


// ---- Command velocity topic callback ----
void cmd_vel_call(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

    char buffer[60];
    sprintf(buffer, "Received cmd_vel: [lin: %.2fm/s, ang: %.2frad/s]", msg->linear.x, msg->angular.z);
    write_log("cmd_vel_call", buffer, LOG_LVL_INFO);
    
    float angular = msg->angular.z;     // rad/s
    float linear = msg->linear.x;       // m/s

    float motor_l_speed_ms = linear - (angular * ((wheelbase / 1000) / 2));             // Calculate motor speeds in m/s
    float motor_r_speed_ms = linear + (angular * ((wheelbase / 1000) / 2));

    float motor_l_speed_rpm = (motor_l_speed_ms / (wheel_circumference / 1000)) * 60;   // Convert motor speeds from m/s to RPM
    float motor_r_speed_rpm = (motor_r_speed_ms / (wheel_circumference / 1000)) * 60;

    r_motors.set_pid_ctrl_speed(abs(motor_r_speed_rpm));
    l_motors.set_pid_ctrl_speed(abs(motor_l_speed_rpm)); 
    r_motors.set_motor_direction((motor_r_speed_rpm > 0) ? Motor::FORWARD : Motor::BACKWARD);
    l_motors.set_motor_direction((motor_l_speed_rpm > 0) ? Motor::FORWARD : Motor::BACKWARD);         
}



// ------- Main program -------

// ---- Publish motor controller data ----
void publish_motor_ctrl_data()
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;
    bool temp_bool_array[2];
    float temp_float_array[2];
    int32_t temp_int_array[2];
    
    mtr_ctrl_r_state_msg.time.sec = timestamp_sec;
    mtr_ctrl_r_state_msg.time.nanosec = timestamp_nanosec;
    mtr_ctrl_l_state_msg.time.sec = timestamp_sec;
    mtr_ctrl_l_state_msg.time.nanosec = timestamp_nanosec;

    // RIGHT
    temp_bool_array[0] = (r_motor_1_enc.get_direction() == MotorEncoder::FORWARD);
    temp_bool_array[1] = (r_motor_2_enc.get_direction() == MotorEncoder::FORWARD);
    mtr_ctrl_r_state_msg.measured_dirs.data = temp_bool_array;
    mtr_ctrl_r_state_msg.measured_dirs.size = sizeof(mtr_ctrl_r_state_msg.measured_dirs.data);

    temp_float_array[0] = r_motor_1_enc.get_rpm();
    temp_float_array[1] = r_motor_2_enc.get_rpm();
    mtr_ctrl_r_state_msg.measured_rpms.data = temp_float_array;
    mtr_ctrl_r_state_msg.measured_rpms.size = sizeof(mtr_ctrl_r_state_msg.measured_rpms.data);

    temp_int_array[0] = r_motor_1_enc.get_pulse_counter();
    temp_int_array[1] = r_motor_2_enc.get_pulse_counter();
    mtr_ctrl_r_state_msg.total_enc_counts.data = temp_int_array;
    mtr_ctrl_r_state_msg.total_enc_counts.size = sizeof(mtr_ctrl_r_state_msg.total_enc_counts.data);

    mtr_ctrl_r_state_msg.motor_gear_ratio_fl = motor_gear_ratio;
    mtr_ctrl_r_state_msg.pid_output = (int) r_motors.get_pid_output();
    mtr_ctrl_r_state_msg.target_dir = (r_motors.get_set_motor_direction() == Motor::FORWARD);
    mtr_ctrl_r_state_msg.target_rpm = r_motors.get_pid_ctrl_speed();
    
    mtr_ctrl_r_state_msg.pid_cals[0] = r_motors.pid->GetKp();
    mtr_ctrl_r_state_msg.pid_cals[1] = r_motors.pid->GetKi();
    mtr_ctrl_r_state_msg.pid_cals[2] = r_motors.pid->GetKd();
    mtr_ctrl_r_state_msg.encoder_pulses_per_rotation = enc_pulses_per_rotation;
    mtr_ctrl_r_state_msg.wheel_diameter_mm = wheel_diameter;

    mtr_ctrl_r_state_msg.total_current = 0;            // TODO: Sensor not installed (for V2)
    mtr_ctrl_r_state_msg.driver_out_voltage = -1.0f;   // TODO: Sensor not installed (for V2)

    // LEFT
    temp_bool_array[0] = (l_motor_1_enc.get_direction() == MotorEncoder::FORWARD);
    temp_bool_array[1] = (l_motor_2_enc.get_direction() == MotorEncoder::FORWARD);
    mtr_ctrl_l_state_msg.measured_dirs.data = temp_bool_array;
    mtr_ctrl_l_state_msg.measured_dirs.size = sizeof(mtr_ctrl_l_state_msg.measured_dirs.data);

    temp_float_array[0] = l_motor_1_enc.get_rpm();
    temp_float_array[1] = l_motor_2_enc.get_rpm();
    mtr_ctrl_l_state_msg.measured_rpms.data = temp_float_array;
    mtr_ctrl_l_state_msg.measured_rpms.size = sizeof(mtr_ctrl_l_state_msg.measured_rpms.data);

    temp_int_array[0] = l_motor_1_enc.get_pulse_counter();
    temp_int_array[1] = l_motor_2_enc.get_pulse_counter();
    mtr_ctrl_l_state_msg.total_enc_counts.data = temp_int_array;
    mtr_ctrl_l_state_msg.total_enc_counts.size = sizeof(mtr_ctrl_l_state_msg.total_enc_counts.data);

    mtr_ctrl_l_state_msg.motor_gear_ratio_fl = motor_gear_ratio;
    mtr_ctrl_l_state_msg.pid_output = (int) l_motors.get_pid_output();
    mtr_ctrl_l_state_msg.target_dir = (l_motors.get_set_motor_direction() == Motor::FORWARD);
    mtr_ctrl_l_state_msg.target_rpm = l_motors.get_pid_ctrl_speed();
    
    mtr_ctrl_l_state_msg.pid_cals[0] = l_motors.pid->GetKp();
    mtr_ctrl_l_state_msg.pid_cals[1] = l_motors.pid->GetKi();
    mtr_ctrl_l_state_msg.pid_cals[2] = l_motors.pid->GetKd();
    mtr_ctrl_l_state_msg.encoder_pulses_per_rotation = enc_pulses_per_rotation;
    mtr_ctrl_l_state_msg.wheel_diameter_mm = wheel_diameter;

    mtr_ctrl_l_state_msg.total_current = 0;            // TODO: Sensor not installed (for V2)
    mtr_ctrl_l_state_msg.driver_out_voltage = -1.0f;   // TODO: Sensor not installed (for V2)

    // FIXME: rcl_publish is extremely slow and fails here.
    //check_rc(rcl_publish(&mtr_ctrl_l_state_pub, &mtr_ctrl_l_state_msg, NULL), RT_SOFT_CHECK);
}


// ---- Publish odometry and transform ----
void publish_odom_tf()
{
    uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
    uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

    int32_t r_enc_counts = r_motors.get_avg_enc_pulse_count();
    int32_t l_enc_counts = l_motors.get_avg_enc_pulse_count();

    int16_t r_enc_diff = r_enc_counts - enc_r_count_old;
    int16_t l_enc_diff = l_enc_counts - enc_l_count_old;
    enc_r_count_old = r_enc_counts;
    enc_l_count_old = l_enc_counts;

    // Convert to millimeters
    int32_t r_enc_mm = (int) (r_enc_counts / (enc_pulses_per_rotation * motor_gear_ratio)) * wheel_circumference;
    int32_t l_enc_mm = (int) (l_enc_counts / (enc_pulses_per_rotation * motor_gear_ratio)) * wheel_circumference;
    float r_enc_diff_mm = ((r_enc_counts - enc_r_count_old) / (enc_pulses_per_rotation * motor_gear_ratio)) * wheel_circumference;
    float l_enc_diff_mm = ((l_enc_counts - enc_l_count_old) / (enc_pulses_per_rotation * motor_gear_ratio)) * wheel_circumference;

    // Calculate the total distance travelled by taking the average of both sides (in millimeters)
    int32_t enc_avg_travel_diff = (int) (r_enc_diff_mm + l_enc_diff_mm) / 2;
    total_enc_avg_travel += enc_avg_travel_diff;

    // Calculate rotation
    theta += (r_enc_diff_mm - l_enc_diff_mm) / 360;
    if (theta > PI) { theta -= TAU; }
    else if (theta < (-PI)) { theta += TAU; }
    
    // Calculate X and Y (in millimeters)
    enc_odom_x_pos += (int) enc_avg_travel_diff * cos(theta);
    enc_odom_y_pos += (int) enc_avg_travel_diff * sin(theta);

    // Odometry
    enc_odom_msg.header.stamp.sec = timestamp_sec;
    enc_odom_msg.header.stamp.nanosec = timestamp_nanosec;
    enc_odom_msg.header.frame_id.data = odom_frame_id;
    enc_odom_msg.header.frame_id.size = sizeof(enc_odom_msg.header.frame_id.data);
    enc_odom_msg.child_frame_id.data = base_link_frame_id;
    enc_odom_msg.child_frame_id.size = sizeof(enc_odom_msg.child_frame_id.data);

    enc_odom_msg.pose.pose.position.x = enc_odom_x_pos / 1000;   // Convert to meters
    enc_odom_msg.pose.pose.position.y = enc_odom_y_pos / 1000;   // Convert to meters
    enc_odom_msg.pose.pose.position.z = 0;
    enc_odom_msg.pose.pose.orientation.z = theta;
    enc_odom_msg.twist.twist.linear.x = (r_motors.get_avg_rpm() + l_motors.get_avg_rpm()) / 2;   // Linear velocity
    enc_odom_msg.twist.twist.linear.y = 0;
    enc_odom_msg.twist.twist.angular.z = ((r_enc_diff_mm - l_enc_diff_mm) / 360) * 100;          // Anglular velocity

    // Odom -> base_link transform
    // TODO: TRANSFORMS SHOULD BE BROADCAST, NOT PUBLISHED! (will require geometry2/tf2)
    odom_baselink_tf_msg.header.stamp.sec = timestamp_sec;
    odom_baselink_tf_msg.header.stamp.nanosec = timestamp_nanosec;
    odom_baselink_tf_msg.header.frame_id.data = odom_frame_id;
    odom_baselink_tf_msg.header.frame_id.size = sizeof(odom_baselink_tf_msg.header.frame_id.data);
    odom_baselink_tf_msg.child_frame_id.data = base_link_frame_id;
    odom_baselink_tf_msg.child_frame_id.size = sizeof(odom_baselink_tf_msg.child_frame_id.data);

    odom_baselink_tf_msg.transform.translation.x = enc_odom_x_pos;
    odom_baselink_tf_msg.transform.translation.y = enc_odom_y_pos;
    odom_baselink_tf_msg.transform.translation.z = 0;
    odom_baselink_tf_msg.transform.rotation.z = theta;

    // Publish
    // FIXME: rcl_publish is extremely slow and fails here.
    //check_rc(rcl_publish(&enc_odom_pub, &enc_odom_msg, NULL), RT_SOFT_CHECK);
    //check_rc(rcl_publish(&odom_baselink_tf_pub, &odom_baselink_tf_msg, NULL), RT_SOFT_CHECK);
}


// ----- Compute motor PID outputs and publish odometry data (timer callback) -----
bool motor_ctrl_odom_timer_call(struct repeating_timer *rt)
{
    // Check execution time
    uint16_t exec_time_ms = (time_us_32() / 1000) - last_motor_odom_time;
    last_motor_odom_time = time_us_32() / 1000;
    
    if (exec_time_ms > (motor_odom_rt_interval + 10)) 
    {
        char buffer[80];
        sprintf(buffer, "Timer function execution time exceeded limits! [act: %ums, lim: %dms]", exec_time_ms, (motor_odom_rt_interval + 10));
        write_log("motor_ctrl_odom_timer_call", buffer, LOG_LVL_WARN);

        publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_A, DIAG_WARN_MSG_TIMER_EXEC_TIME_OVER, DIAG_KV_EMPTY());
    }

    // Call the encoder timer callbacks
    r_motor_1_enc.enc_timer_irq_trigger();
    r_motor_2_enc.enc_timer_irq_trigger();
    l_motor_1_enc.enc_timer_irq_trigger();
    l_motor_2_enc.enc_timer_irq_trigger();

    // Calculate and set motor outputs
    r_motors.compute_outputs();
    l_motors.compute_outputs();

    publish_motor_ctrl_data();   // Publish motor controller data
    publish_odom_tf();           // Calculate and publish odometry data

    /*char buffer[100];
    sprintf(buffer, "L1: %.2f, L2: %.2f, R1: %.2f, R2: %.2f", l_motor_1_enc.get_m2_rpm(), l_motor_2_enc.get_m2_rpm(), r_motor_1_enc.get_m2_rpm(), r_motor_2_enc.get_m2_rpm());
    write_log("mtr_debug", buffer, LOG_LVL_INFO);*/

    /*char buffer[150];
    sprintf(buffer, "L1: %d, L2: %d, R1: %d, R2: %d", l_motor_1_enc.get_pulse_counter(), l_motor_2_enc.get_pulse_counter(), r_motor_1_enc.get_pulse_counter(), r_motor_2_enc.get_pulse_counter());
    write_log("mtr_debug", buffer, LOG_LVL_INFO);*/

    return true;
}


// ---- MPU6050 init ----
bool init_mpu6050()
{
    write_log("init_mpu6050", "MPU6050 disabled!", LOG_LVL_INFO);

    // Init pins
    init_pin(i2c_sda, PROT_I2C);
    init_pin(i2c_scl, PROT_I2C);
    gpio_pull_up(i2c_sda);
    gpio_pull_up(i2c_scl);
    bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C));   // For Picotool

    write_log("init_mpu6050", "MPU6050 WHO_AM_I ID: " + std::to_string(mpu6050_who_am_i(&mpu6050)), LOG_LVL_INFO);

    // Commented out as my MPU6050 is broken, waiting for the replacement.
    /*if (mpu6050_begin(&mpu6050))
    {
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

        mpu6050_set_temperature_measuring(&mpu6050, true);
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);
        mpu6050_set_int_free_fall(&mpu6050, true);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        write_log("init_mpu6050", "MPU init successful.", LOG_LVL_INFO);
        return true;
    }

    write_log("init_mpu6050", "MPU init failed.", LOG_LVL_WARN);
    publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_ENV_SENSORS, DIAG_HWID_ENV_IMU, DIAG_ERR_MSG_INIT_FAILED, NULL);
    return false;*/

    return true;
}


// ---- Setup repeating timers ----
void start_repeating_timers()
{
    add_repeating_timer_ms(motor_odom_rt_interval, motor_ctrl_odom_timer_call, NULL, &motor_odom_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, edge_ir_pub_rt_interval, publish_edge_ir, NULL, &edge_ir_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, ultra_pub_rt_interval, publish_ultra, NULL, &ultrasonic_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sensors_pub_rt_interval, publish_misc_sens, NULL, &other_sensors_publish_rt);
    ultrasonic_ir_edge_rt_active = true;
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

    // Misc. init
    adc_init();
    adc_set_temp_sensor_enabled(true);
    set_mux_pins(analog_mux_s0, analog_mux_s1, analog_mux_s2, analog_mux_s3, analog_mux_io);
    set_ir_en_pin(edge_sens_en);

    // MotorSafety init
    r_motors_safety.configure_safety(motor_single_side_max_difference, motor_set_vs_actual_max_difference, motor_safety_trigger_timeout, &motor_safety_callback);
    l_motors_safety.configure_safety(motor_single_side_max_difference, motor_set_vs_actual_max_difference, motor_safety_trigger_timeout, &motor_safety_callback);
    r_motors_safety.set_set_vs_actual_spd_time_tolerance(motor_actual_vs_set_extra_timeout);
    l_motors_safety.set_set_vs_actual_spd_time_tolerance(motor_actual_vs_set_extra_timeout);

    // Motor Controller init
    r_motors.set_control_mode(Motor::PID);
    l_motors.set_control_mode(Motor::PID);
    r_motor_1_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    r_motor_2_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    l_motor_1_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    l_motor_2_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);

    // UART & USB STDIO outputs
    //stdio_init_all();
    //stdio_filter_driver(&stdio_usb);   // Filter the output of STDIO to USB.
    //write_log("main", "STDIO init!", LOG_LVL_INFO);

    // MicroROS transport init
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // MicroROS and timer inits are performed in the core 0 loop.
}


// ---- Setup function (Runs once on core 1) ----
void setup1()
{
    // MPU init
    write_log("setup1", "Begin MPU6050 init.", LOG_LVL_INFO);
    check_bool(init_mpu6050(), RT_HARD_CHECK);

    // Create alarm pool for core 1 timers
    // FIXME: For some reason, alarm_pool_create() only gets called when this write_log() call is here.
    write_log("setup1", "We are at alarm_pool_create()!", LOG_LVL_INFO);
    core_1_alarm_pool = alarm_pool_create(1, 16);
}


// ---- Core 0 loop ----
void loop()
{
    switch (current_uros_state) 
    {
        case WAITING_FOR_AGENT:
            current_uros_state = ping_agent() ? AGENT_AVAILABLE:WAITING_FOR_AGENT;
            break;
        
        case AGENT_AVAILABLE:
            write_log("main", "MicroROS agent available... initializing MicroROS.", LOG_LVL_INFO);

            // MicroROS init
            uros_init(UROS_NODE_NAME, UROS_NODE_NAMESPACE);
            init_subs_pubs();
            exec_init();

            // Repeating timers
            start_repeating_timers();

            // Enable the motor controllers.
            r_motors.enable_controller();
            l_motors.enable_controller();

            write_log("main", "Initialization completed! State changed to AGENT_CONNECTED.", LOG_LVL_INFO);
            current_uros_state = AGENT_CONNECTED;
            break;
        
        case AGENT_CONNECTED:
            current_uros_state = ping_agent() ? AGENT_CONNECTED:AGENT_DISCONNECTED;
            
            if (current_uros_state == AGENT_CONNECTED) 
            {
                // Check execution time
                uint16_t exec_time_ms = (time_us_32() / 1000) - last_uros_exec_time;
                last_uros_exec_time = time_us_32() / 1000;
                
                if (exec_time_ms > uros_executor_exec_timeout) 
                {
                    char buffer[80];
                    sprintf(buffer, "MicroROS executor execution time exceeded limits! [act: %ums, lim: %dms]", exec_time_ms, uros_executor_exec_timeout);
                    write_log("main", buffer, LOG_LVL_WARN);

                    publish_diag_report(DIAG_LVL_WARN, DIAG_HWNAME_UCONTROLLERS, DIAG_HWID_MCU_MABO_A, DIAG_WARN_MSG_TIMER_EXEC_TIME_OVER, DIAG_KV_EMPTY());
                }

                rclc_executor_spin_some(&rc_executor, RCL_MS_TO_NS(UROS_EXEC_TIMEOUT_MS));
            }
            
            break;
        
        case AGENT_DISCONNECTED:
            // TODO: Maybe wait for the agent to connect again?
            write_log("main", "MicroROS agent disconnected! Shutting down...", LOG_LVL_INFO);
            clean_shutdown();
            break;
    }
}


// ---- Core 1 loop ----
void loop_1()
{

}



// ******** END OF MAIN PROGRAM *********
// *********** STARTUP & INIT ***********

// ---- Core 1 main function ----
void main_core_1()
{
    setup1();

    // Core 1 loop
    while (true) { loop_1(); };

    write_log("main_core_1", "Core 1 exit!", LOG_LVL_INFO);
}


// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    multicore_reset_core1();

    // UART & USB STDIO outputs
    stdio_init_all();
    stdio_filter_driver(&stdio_usb);   // Filter the output of STDIO to USB.
    write_log("main", "STDIO init, program starting...", LOG_LVL_INFO);

    // Wait for 3 seconds
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);

    for (int i = 0; i < 3; i++)
    {
        write_log("main", "Waiting...", LOG_LVL_INFO);
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(500);
        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(500);
    }

    setup();
    multicore_launch_core1(main_core_1);

    gpio_put(onboard_led, HIGH);
    gpio_put(power_led, HIGH);

    // Core 0 loop
    while (!halt_core_0) { loop(); }

    write_log("main", "Program exit.", LOG_LVL_INFO);
    return 0;
}