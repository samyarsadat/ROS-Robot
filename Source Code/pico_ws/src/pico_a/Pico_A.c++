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
#include "helpers/Self_Test.c++"
#include "helpers/Calibrate.c++"
#include "helpers/Sensor_Publishers.c++"
#include "helpers/Motor_Safety.c++"
#include "motor_control_lib/Motor.h"
#include "motor_control_lib/Motor_Safety.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include <cmath>



// ------- Global variables -------

// ---- Misc. ----
alarm_pool_t *core_1_alarm_pool;

// ---- Timer execution times storage (milliseconds) ----
uint32_t last_motor_odom_time, last_ultrasonic_publish_time, last_edge_ir_publish_time, last_other_sensors_publish_time;
uint32_t last_uros_exec_time;

// ---- Motor encoder counter storage ----
int32_t enc_r_count_old, enc_l_count_old, total_enc_avg_travel;
int32_t enc_odom_x_pos, enc_odom_y_pos;
float theta;

// ---- Timers ----
bool ultrasonic_ir_edge_rt_active = false;
struct repeating_timer motor_odom_rt, ultrasonic_publish_rt, edge_ir_publish_rt, other_sensors_publish_rt, motor_enc_rt;
TaskHandle_t motor_odom_th, ultrasonic_publish_th, edge_ir_publish_th, other_sensors_publish_th, motor_safety_task_th;
TimerHandle_t waiting_for_agent_timer;



// ------- Library object inits -------

// ---- MicroROS ----
uRosBridgeAgent *bridge;
uRosPublishingHandler *pub_handler;

// ---- MPU6050 ----
mpu6050_t mpu6050 = mpu6050_init(i2c_inst, MPU6050_ADDRESS_A0_VCC);

// ---- Motor Controller ----
uint8_t r_motors_drv_pins[2] = {r_motor_drive_1, r_motor_drive_2};
uint8_t l_motors_drv_pins[2] = {l_motor_drive_1, l_motor_drive_2};
MotorDriver r_motors_driver(r_motors_drv_pins, (sizeof(r_motors_drv_pins) / sizeof(r_motors_drv_pins[0])), MotorDriver::driver_type::GENERIC_PWM);
MotorDriver l_motors_driver(l_motors_drv_pins, (sizeof(l_motors_drv_pins) / sizeof(l_motors_drv_pins[0])), MotorDriver::driver_type::GENERIC_PWM);

// IRQ callback prototype
void irq_call(uint pin, uint32_t events);

MotorEncoder r_motor_1_enc(r_motor_1_enc_a, r_motor_1_enc_b, gear_ratio_motor, enc_pulses_per_rot, &irq_call);
MotorEncoder r_motor_2_enc(r_motor_2_enc_a, r_motor_2_enc_b, gear_ratio_motor, enc_pulses_per_rot, &irq_call);
MotorEncoder l_motor_1_enc(l_motor_1_enc_a, l_motor_1_enc_b, gear_ratio_motor, enc_pulses_per_rot, &irq_call);
MotorEncoder l_motor_2_enc(l_motor_2_enc_a, l_motor_2_enc_b, gear_ratio_motor, enc_pulses_per_rot, &irq_call);
MotorEncoder* r_motor_encs[2] = {&r_motor_1_enc, &r_motor_2_enc};
MotorEncoder* l_motor_encs[2] = {&l_motor_1_enc, &l_motor_2_enc};

Motor r_motors(&r_motors_driver, r_motor_encs, (sizeof(r_motor_encs) / sizeof(r_motor_encs[0])));
Motor l_motors(&l_motors_driver, l_motor_encs, (sizeof(l_motor_encs) / sizeof(l_motor_encs[0])));
MotorSafety r_motors_safety(&r_motors, right_motor_controller_id);
MotorSafety l_motors_safety(&l_motors, left_motor_controller_id);



// ------- Functions ------- 

// ---- Graceful shutdown ----
void clean_shutdown()
{
    write_log("A clean shutdown has been triggered. The program will now shut down.", LOG_LVL_FATAL, FUNCNAME_ONLY);

    // Stop all timers
    cancel_repeating_timer(&motor_odom_rt);
    cancel_repeating_timer(&ultrasonic_publish_rt);
    cancel_repeating_timer(&edge_ir_publish_rt);
    cancel_repeating_timer(&other_sensors_publish_rt);
    cancel_repeating_timer(&motor_enc_rt);

    // IO cleanup
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);
    gpio_put(power_led, LOW);
    gpio_put(onboard_led, LOW);
        
    l_motors.disable_controller();
    r_motors.disable_controller();
        
    take_mux_mutex();   // No need to release the mutex.
    set_mux_addr(0);
    set_mux_io_mode(OUTPUT);
    gpio_put(analog_mux_io, LOW);
    init_pin(edge_sens_en, OUTPUT);
    gpio_put(edge_sens_en, LOW);

    // MicroROS cleanup
    pub_handler->stop();
    bridge->uros_fini();

    write_log("MicroROS cleanup completed. Suspending the scheduler...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Suspend the FreeRTOS scheduler
    // No FreeRTOS API calls beyond this point!
    vTaskSuspendAll();
}


// ---- FreeRTOS task stack overflow hook ----
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    write_log("Stack overflow! Task: " + std::string(pcTaskName, strlen(pcTaskName)), LOG_LVL_FATAL, FUNCNAME_ONLY);
    clean_shutdown();
}


// ---- FreeRTOS memory allocation failure hook ----
void vApplicationMallocFailedHook()
{
    // Not calling clean_shutdown() because it calls write_log() which uses malloc.
    // This shouldn't ever happen anyway, so it doesn't matter.
    panic("Memory allocation failed!");
}


// ---- IRQ callback ----
void irq_call(uint pin, uint32_t events)
{
    r_motor_1_enc.enc_hardware_irq_trigger(pin);
    r_motor_2_enc.enc_hardware_irq_trigger(pin);
    l_motor_1_enc.enc_hardware_irq_trigger(pin);
    l_motor_2_enc.enc_hardware_irq_trigger(pin);
    portYIELD_FROM_ISR(pdTRUE);
}


// ---- Timer callbacks for task notification ----
bool motor_odom_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(motor_odom_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_ultra_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(ultrasonic_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_edge_ir_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(edge_ir_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_misc_sens_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(other_sensors_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}



// ------- MicroROS subscriber & service callbacks ------- 

// ---- Motor encoder timer callback prototype ----
bool motor_enc_timer_call(struct repeating_timer *rt);


// ---- Enable/disable motor controller service ----
void en_motor_ctrl_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    if (req_in->data && !r_motors.is_enabled() && !l_motors.is_enabled())
    {
        write_log("Enabling motor controllers.", LOG_LVL_INFO, FUNCNAME_ONLY);
        r_motors.enable_controller();
        l_motors.enable_controller();
        alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, motor_odom_rt_interval, motor_odom_notify, NULL, &motor_odom_rt);
        add_repeating_timer_ms(motor_enc_method_2_rt_interval, motor_enc_timer_call, NULL, &motor_enc_rt);
    }

    else if (!req_in->data) 
    {
        write_log("Disabling motor controllers.", LOG_LVL_INFO, FUNCNAME_ONLY);
        r_motors.disable_controller();
        l_motors.disable_controller();
        cancel_repeating_timer(&motor_odom_rt);
        cancel_repeating_timer(&motor_enc_rt);
    }

    else 
    {
        res_in->success = false;
        return;
    }

    res_in->success = true;
}


// ---- Enable/disable emitters (ultrasonic, IR edge) service ----
void en_emitters_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    if (req_in->data && !ultrasonic_ir_edge_rt_active)
    {
        write_log("Enabling ultrasonic and IR edge emitters.", LOG_LVL_INFO, FUNCNAME_ONLY);
        ultrasonic_ir_edge_rt_active = true;
        alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, ultra_pub_rt_interval, publish_ultra_notify, NULL, &ultrasonic_publish_rt);
        alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, edge_ir_pub_rt_interval, publish_edge_ir_notify, NULL, &edge_ir_publish_rt);
    }

    else if (!req_in->data && ultrasonic_ir_edge_rt_active)
    {
        write_log("Disabling ultrasonic and IR edge emitters.", LOG_LVL_INFO, FUNCNAME_ONLY);
        ultrasonic_ir_edge_rt_active = false;
        cancel_repeating_timer(&ultrasonic_publish_rt);
        cancel_repeating_timer(&edge_ir_publish_rt);
        gpio_put(edge_sens_en, LOW);
    }

    else 
    {
        res_in->success = false;
        return;
    }

    res_in->success = true;
}


// ---- Enable/disable relay service ----
void en_relay_callback(const void *req, void *res) 
{
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) res;

    if (req_in->data)
    {
        write_log("Enabling relay.", LOG_LVL_INFO, FUNCNAME_ONLY);
        gpio_put(pi_power_relay, HIGH);
    }

    else 
    {
        write_log("Disabling relay.", LOG_LVL_INFO, FUNCNAME_ONLY);
        gpio_put(pi_power_relay, LOW);
    }

    res_in->success = true;
}


// ---- Set motor controller PID tunings service ----
void set_mtr_pid_tunings_callback(const void *req, void *res)
{
    rrp_pico_coms__srv__SetPidTunings_Request *req_in = (rrp_pico_coms__srv__SetPidTunings_Request *) req;
    rrp_pico_coms__srv__SetPidTunings_Response *res_in = (rrp_pico_coms__srv__SetPidTunings_Response *) res;

    char buffer[70];
    snprintf(buffer, sizeof(buffer), "Received set_pid_tunings: [Kp: %.3f, Ki: %.3f, Kd: %.3f]", req_in->pid_kp, req_in->pid_ki, req_in->pid_kd);
    write_log(buffer, LOG_LVL_INFO, FUNCNAME_ONLY);

    r_motors.pid->SetTunings(req_in->pid_kp, req_in->pid_ki, req_in->pid_kd);
    l_motors.pid->SetTunings(req_in->pid_kp, req_in->pid_ki, req_in->pid_kd);

    res_in->success = true;
}


// ---- Get configuration information service ----
void get_config_callback(const void *req, void *res)
{
    rrp_pico_coms__srv__GetConfigA_Request *req_in = (rrp_pico_coms__srv__GetConfigA_Request *) req;   // Unused because it is empty.
    rrp_pico_coms__srv__GetConfigA_Response *res_in = (rrp_pico_coms__srv__GetConfigA_Response *) res;

    write_log("Received get_config request.", LOG_LVL_INFO, FUNCNAME_ONLY);

    res_in->wheelbase_mm = wheelbase;
    res_in->wheel_diameter_mm = wheel_diameter;
    res_in->encoder_pulses_per_rotation = enc_pulses_per_rot;
    res_in->motor_gear_ratio_fl = gear_ratio_motor;
    res_in->ir_edge_sens_fov = ir_edge_fov;
    res_in->ir_edge_sens_range = ir_edge_detection_range;
    res_in->ultrasonic_fov = ultra_fov;
    res_in->ultrasonic_max_dist = ultra_max_dist;
    res_in->ultrasonic_min_dist = ultra_min_dist;
    res_in->pid_cals_right[0] = r_motors.pid->GetKp();
    res_in->pid_cals_right[1] = r_motors.pid->GetKi();
    res_in->pid_cals_right[2] = r_motors.pid->GetKd();
    res_in->pid_cals_left[0] = l_motors.pid->GetKp();
    res_in->pid_cals_left[1] = l_motors.pid->GetKi();
    res_in->pid_cals_left[2] = l_motors.pid->GetKd();
}


// ---- Command velocity topic callback ----
void cmd_vel_call(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

    char buffer[60];
    snprintf(buffer, sizeof(buffer), "Received cmd_vel: [lin: %.2fm/s, ang: %.2frad/s]", msg->linear.x, msg->angular.z);
    write_log(buffer, LOG_LVL_INFO, FUNCNAME_ONLY);
    
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
    
    mtr_ctrl_r_state_msg.time.sec = timestamp_sec;
    mtr_ctrl_r_state_msg.time.nanosec = timestamp_nanosec;
    mtr_ctrl_l_state_msg.time.sec = timestamp_sec;
    mtr_ctrl_l_state_msg.time.nanosec = timestamp_nanosec;

    // RIGHT
    mtr_ctrl_r_state_msg.measured_dirs[0] = (r_motor_1_enc.get_direction() == MotorEncoder::FORWARD);
    mtr_ctrl_r_state_msg.measured_dirs[1] = (r_motor_2_enc.get_direction() == MotorEncoder::FORWARD);

    mtr_ctrl_r_state_msg.measured_rpms[0] = r_motor_1_enc.get_rpm();
    mtr_ctrl_r_state_msg.measured_rpms[1] = r_motor_2_enc.get_rpm();

    mtr_ctrl_r_state_msg.total_enc_counts[0] = r_motor_1_enc.get_pulse_counter(); 
    mtr_ctrl_r_state_msg.total_enc_counts[1] = r_motor_2_enc.get_pulse_counter(); 

    mtr_ctrl_r_state_msg.pid_output = (int) r_motors.get_pid_output();
    mtr_ctrl_r_state_msg.target_dir = (r_motors.get_set_motor_direction() == Motor::FORWARD);
    mtr_ctrl_r_state_msg.target_rpm = r_motors.get_pid_ctrl_speed();

    mtr_ctrl_r_state_msg.total_current = 0;            // TODO: Sensor not installed (for V2)
    mtr_ctrl_r_state_msg.driver_out_voltage = -1.0f;   // TODO: Sensor not installed (for V2)

    mtr_ctrl_r_state_msg.controller_enabled = r_motors.is_enabled();

    // LEFT
    mtr_ctrl_l_state_msg.measured_dirs[0] = (l_motor_1_enc.get_direction() == MotorEncoder::FORWARD);
    mtr_ctrl_l_state_msg.measured_dirs[1] = (l_motor_2_enc.get_direction() == MotorEncoder::FORWARD);

    mtr_ctrl_l_state_msg.measured_rpms[0] = l_motor_1_enc.get_rpm();
    mtr_ctrl_l_state_msg.measured_rpms[1] = l_motor_2_enc.get_rpm();

    mtr_ctrl_l_state_msg.total_enc_counts[0] = l_motor_1_enc.get_pulse_counter(); 
    mtr_ctrl_l_state_msg.total_enc_counts[1] = l_motor_2_enc.get_pulse_counter(); 

    mtr_ctrl_l_state_msg.pid_output = (int) l_motors.get_pid_output();
    mtr_ctrl_l_state_msg.target_dir = (l_motors.get_set_motor_direction() == Motor::FORWARD);
    mtr_ctrl_l_state_msg.target_rpm = l_motors.get_pid_ctrl_speed();

    mtr_ctrl_l_state_msg.total_current = 0;            // TODO: Sensor not installed (for V2)
    mtr_ctrl_l_state_msg.driver_out_voltage = -1.0f;   // TODO: Sensor not installed (for V2)

    mtr_ctrl_l_state_msg.controller_enabled = l_motors.is_enabled();

    // PUBLISH
    uRosPublishingHandler::PublishItem_t mtr_left, mtr_right;
    
    mtr_left.publisher = &mtr_ctrl_l_state_pub;
    mtr_left.message = &mtr_ctrl_l_state_msg;
    xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &mtr_left, 0);

    mtr_right.publisher = &mtr_ctrl_r_state_pub;
    mtr_right.message = &mtr_ctrl_r_state_msg;
    xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &mtr_right, 0);
}


// ---- Publish odometry ----
void publish_odom()
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
    float r_enc_mm = (((float) r_enc_counts / (enc_pulses_per_rot * gear_ratio_motor)) * wheel_circumference);
    float l_enc_mm = (((float) l_enc_counts / (enc_pulses_per_rot * gear_ratio_motor)) * wheel_circumference);
    float r_enc_diff_mm = (((float) r_enc_diff / (enc_pulses_per_rot * gear_ratio_motor)) * wheel_circumference);
    float l_enc_diff_mm = (((float) l_enc_diff / (enc_pulses_per_rot * gear_ratio_motor)) * wheel_circumference);

    // Calculate the total distance travelled by taking the average of both sides (in millimeters)
    float enc_avg_travel_diff = ((r_enc_diff_mm + l_enc_diff_mm) / 2.0f);
    total_enc_avg_travel += enc_avg_travel_diff;

    // Calculate rotation
    theta += (float) (r_enc_diff_mm - l_enc_diff_mm) / 360;
    if (theta > PI) { theta -= TAU; }
    else if (theta < (-PI)) { theta += TAU; }

    // Get rotation in quaternion
    std::vector<float> rot_quat = euler_to_quaternion(0, 0, theta);
    geometry_msgs__msg__Quaternion quat_msg;
    quat_msg.x = rot_quat[0];
    quat_msg.y = rot_quat[1];
    quat_msg.z = rot_quat[2];
    quat_msg.w = rot_quat[3];
    
    // Calculate X and Y (in millimeters)
    enc_odom_x_pos += (int) enc_avg_travel_diff * cos(theta);
    enc_odom_y_pos += (int) enc_avg_travel_diff * sin(theta);

    // Odometry
    enc_odom_msg.header.stamp.sec = timestamp_sec;
    enc_odom_msg.header.stamp.nanosec = timestamp_nanosec;
    enc_odom_msg.header.frame_id.data = odom_frame_id;
    enc_odom_msg.header.frame_id.size = strlen(enc_odom_msg.header.frame_id.data);
    enc_odom_msg.child_frame_id.data = base_link_frame_id;
    enc_odom_msg.child_frame_id.size = strlen(enc_odom_msg.child_frame_id.data);

    enc_odom_msg.pose.position.x = (float) enc_odom_x_pos / 1000;   // Convert to meters
    enc_odom_msg.pose.position.y = (float) enc_odom_y_pos / 1000;   // Convert to meters
    enc_odom_msg.pose.position.z = 0;
    enc_odom_msg.pose.orientation = quat_msg;
    enc_odom_msg.twist.linear.x = (float) (r_enc_diff_mm + l_enc_diff_mm) / 2;              // Linear velocity
    enc_odom_msg.twist.linear.y = 0;
    enc_odom_msg.twist.angular.z = (float) ((r_enc_diff_mm - l_enc_diff_mm) / 360) * 100;   // Anglular velocity

    // Publish
    uRosPublishingHandler::PublishItem_t enc_odom;
    enc_odom.publisher = &enc_odom_pub;
    enc_odom.message = &enc_odom_msg;
    xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &enc_odom, 0);
}


// ----- Compute motor PID outputs and publish odometry data (timer callback) -----
void motor_ctrl_odom_task(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely
        
        // Check execution time
        check_exec_interval(last_motor_odom_time, (motor_odom_rt_interval + 50), "Execution interval exceeded limits!", true);

        // Calculate and set motor outputs
        r_motors.compute_outputs();
        l_motors.compute_outputs();

        publish_motor_ctrl_data();   // Publish motor controller data
        publish_odom();           // Calculate and publish odometry data
    }
}


// ---- Motor encoder method 2 timer callback ----
bool motor_enc_timer_call(struct repeating_timer *rt)
{
    // Call the encoder timer callbacks
    r_motor_1_enc.enc_timer_irq_trigger();
    r_motor_2_enc.enc_timer_irq_trigger();
    l_motor_1_enc.enc_timer_irq_trigger();
    l_motor_2_enc.enc_timer_irq_trigger();

    // Motor safety timer callbacks
    r_motors_safety.safety_check_timer_callback();
    l_motors_safety.safety_check_timer_callback();
    
    portYIELD_FROM_ISR(pdTRUE);
    return true;
}


// ---- MicroROS executor post-execution function ----
void uros_post_exec_call()
{
    // Cleanup for self-test diagnostics messages.
    if (!self_test_diag_data_slot_nums.empty())
    {
        for (auto slot_num : self_test_diag_data_slot_nums)
        {
            destroy_uros_diag_status_msg(slot_num);
            destroy_diag_kv_pair(slot_num);
            destroy_diag_kv_pair_refs(slot_num);
            destroy_diag_msg_object(slot_num);
            deallocate_slots(slot_num);
        }

        self_test_diag_status_reports.clear();
        self_test_diag_status_reports.shrink_to_fit();
        self_test_diag_data_slot_nums.clear();
        self_test_diag_data_slot_nums.shrink_to_fit();

        enable_diag_pub();
    }
}


// ---- MPU6050 init ----
bool init_mpu6050()
{
    write_log("MPU6050 disabled!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Init pins
    init_pin(i2c_sda, PROT_I2C);
    init_pin(i2c_scl, PROT_I2C);
    gpio_pull_up(i2c_sda);
    gpio_pull_up(i2c_scl);
    bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C));   // For Picotool

    write_log("MPU6050 WHO_AM_I ID: " + std::to_string(mpu6050_who_am_i(&mpu6050)), LOG_LVL_INFO, FUNCNAME_ONLY);

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

        write_log("MPU init successful.", LOG_LVL_INFO, FUNCNAME_ONLY);
        return true;
    }

    write_log("MPU init failed.", LOG_LVL_WARN, FUNCNAME_ONLY);
    publish_diag_report(DIAG_LVL_ERROR, DIAG_HWNAME_ENV_SENSORS, DIAG_HWID_ENV_IMU, DIAG_ERR_MSG_INIT_FAILED, DIAG_KV_PAIR_EMPTY());
    return false;*/

    return true;
}


// ---- Setup repeating timers ----
void start_timers()
{
    write_log("Starting hardware timers...", LOG_LVL_INFO, FUNCNAME_ONLY);

    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, motor_odom_rt_interval, motor_odom_notify, NULL, &motor_odom_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, ultra_pub_rt_interval, publish_ultra_notify, NULL, &ultrasonic_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, edge_ir_pub_rt_interval, publish_edge_ir_notify, NULL, &edge_ir_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sensors_pub_rt_interval, publish_misc_sens_notify, NULL, &other_sensors_publish_rt);
    add_repeating_timer_ms(motor_enc_method_2_rt_interval, motor_enc_timer_call, NULL, &motor_enc_rt);
    
    ultrasonic_ir_edge_rt_active = true;
    r_motors.enable_controller();
    l_motors.enable_controller();
    r_motors_safety.enable_safety();
    l_motors_safety.enable_safety();
}


// ---- Waiting for agent LED flash timer callback ----
void waiting_for_agent_timer_call(TimerHandle_t timer)
{
    if (bridge->get_agent_state() == uRosBridgeAgent::WAITING_FOR_AGENT)
    {
        gpio_put(onboard_led, !gpio_get_out_level(onboard_led));
        gpio_put(power_led, !gpio_get_out_level(power_led));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_AVAILABLE)
    {
        if (xTimerGetPeriod(timer) != pdMS_TO_TICKS(AGENT_AVAIL_LED_TOGGLE_DELAY_MS))
        {
            xTimerChangePeriod(timer, AGENT_AVAIL_LED_TOGGLE_DELAY_MS, 0);
        }

        gpio_put(onboard_led, !gpio_get_out_level(onboard_led));
        gpio_put(power_led, !gpio_get_out_level(power_led));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_CONNECTED)
    {
        gpio_put(onboard_led, HIGH);
        gpio_put(power_led, HIGH);
    }

    else
    {
        gpio_put(onboard_led, LOW);
        gpio_put(power_led, LOW);
    }

    xTimerDelete(timer, 0);
}


// ---- Setup function (Runs once on core 0) ----
void setup(void *parameters)
{
    init_print_uart_mutex();
    write_log("Core 0 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Initialize mutexes
    diag_mutex_init();

    // Pin init
    init_pin(ready_sig, INPUT);
    init_pin(pi_power_relay, OUTPUT);
    init_pin(edge_sens_en, OUTPUT);

    // Misc. init
    adc_init();
    adc_init_mutex();
    adc_set_temp_sensor_enabled(true);
    set_mux_pins(analog_mux_s0, analog_mux_s1, analog_mux_s2, analog_mux_s3, analog_mux_io);
    set_ir_en_pin(edge_sens_en);

    // MotorSafety init
    r_motors_safety.configure_safety(motor_single_side_max_difference, motor_set_vs_actual_max_difference, motor_safety_trigger_timeout, &motor_safety_isr_callback);
    l_motors_safety.configure_safety(motor_single_side_max_difference, motor_set_vs_actual_max_difference, motor_safety_trigger_timeout, &motor_safety_isr_callback);
    r_motors_safety.set_set_vs_actual_spd_time_tolerance(motor_actual_vs_set_extra_timeout);
    l_motors_safety.set_set_vs_actual_spd_time_tolerance(motor_actual_vs_set_extra_timeout);
    init_motor_safety_queue();

    // Motor Controller init
    r_motors.set_control_mode(Motor::PID);
    l_motors.set_control_mode(Motor::PID);
    r_motor_1_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    r_motor_2_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    l_motor_1_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    l_motor_2_enc.set_method_1_cutoff(motor_rpm_method_1_cutoff);
    l_motor_1_enc.set_enc_direction_reversed(true);   // This is temporary, the encoder pins are currently swapped!
    l_motor_2_enc.set_enc_direction_reversed(true);   // This is temporary, the encoder pins are currently swapped!

    // Create timer tasks
    write_log("Creating timer tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    xTaskCreate(motor_ctrl_odom_task, "motor_odom", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &motor_odom_th);
    xTaskCreate(publish_edge_ir, "publish_edge_ir", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 5, &edge_ir_publish_th);
    xTaskCreate(publish_ultra, "publish_ultra", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 5, &ultrasonic_publish_th);
    xTaskCreate(publish_misc_sens, "publish_misc_sens", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 5, &other_sensors_publish_th);
    vTaskCoreAffinitySet(ultrasonic_publish_th, (1 << 1));      // Lock task to core 1
    vTaskCoreAffinitySet(edge_ir_publish_th, (1 << 1));         // Lock task to core 1
    vTaskCoreAffinitySet(other_sensors_publish_th, (1 << 1));   // Lock task to core 1

    // Other tasks
    write_log("Creating other tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    xTaskCreate(motor_safety_handler_task, "motor_safety_handler", GENERIC_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 3, &motor_safety_task_th);
    vTaskCoreAffinitySet(motor_safety_task_th, (1 << 1));   // Lock task to core 1

    // Create FreeRTOS timers
    write_log("Creating FreeRTOS software timers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    waiting_for_agent_timer = xTimerCreate("waiting_for_agent_timer", pdMS_TO_TICKS(AGENT_WAITING_LED_TOGGLE_DELAY_MS), pdTRUE, NULL, waiting_for_agent_timer_call);

    // Start MicroROS tasks
    write_log("Starting MicroROS tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    check_bool(bridge->start(configMAX_PRIORITIES - 1, (1 << 0), true), RT_HARD_CHECK);
    check_bool(pub_handler->start(configMAX_PRIORITIES - 2, (1 << 0), true), RT_HARD_CHECK);

    // Start the waiting for MicroROS agent LED blink timer
    xTimerStart(waiting_for_agent_timer, 0);

    // Delete setup task
    vTaskDelete(NULL);
}


// ---- Setup function (Runs once on core 1) ----
void setup1(void *parameters)
{
    write_log("Core 1 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Create alarm pool for core 1 timers
    write_log("Creating core 1 alarm pool...", LOG_LVL_INFO, FUNCNAME_ONLY);
    core_1_alarm_pool = alarm_pool_create(2, 8);

    // MPU6050 init
    check_bool(init_mpu6050(), RT_HARD_CHECK);

    // Delete setup task
    vTaskDelete(NULL);
}



// ******** END OF MAIN PROGRAM *********
// *********** STARTUP & INIT ***********

// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    // UART & USB STDIO outputs
    stdio_init_all();
    stdio_filter_driver(&stdio_usb);   // Filter the output of STDIO to USB.
    write_log("STDIO init, program starting...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);

    // Wait for 3 seconds
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);

    for (int i = 0; i < STARTUP_WAIT_TIME_S; i++)
    {
        write_log("Startup wait " + std::to_string(i + 1) + "...", LOG_LVL_INFO, FUNCNAME_ONLY);
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(500);
        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(500);
    }

    // MicroROS pre-init
    write_log("MicroROS pre-init...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    bridge = uRosBridgeAgent::get_instance();
    pub_handler = uRosPublishingHandler::get_instance();
    bridge->pre_init(uros_init, clean_shutdown, uros_post_exec_call);
    pub_handler->pre_init(bridge);
    set_diag_pub_queue(pub_handler->get_queue_handle());

    // Stetup function tasks
    write_log("Creating setup tasks...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    xTaskCreateAffinitySet(setup, "setup_core_0", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 0), NULL);
    xTaskCreateAffinitySet(setup1, "setup_core_1", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 1), NULL);

    // Start FreeRTOS scheduler
    write_log("Starting FreeRTOS scheduler...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    vTaskStartScheduler();

    // We should never get to this point!
    write_log("Program exit. Scheduler start failed!", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    return 0;
}