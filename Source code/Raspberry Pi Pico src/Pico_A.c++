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
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "lib/Helper_lib/Helpers.h"
#include "lib/PID_lib/PID_v1.h"
#include "hardware/pwm.h"
#include "lib/Motor_lib/Motor.h"
#include "lib/Motor_lib/Motor_Safety.h"



// ------- Global variables -------

// ---- Micro ROS ----
rcl_allocator_t rc_alloc;
rclc_support_t rc_supp;
rcl_node_t rc_node;
rclc_executor_t executor;

// ---- Micro ROS subscribers and publishers ----
rcl_subscription_t rpi_ready_sub;
std_msgs__msg__String rpi_ready_msg;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t e_stop_pub;
std_msgs__msg__String e_stop_msg;

rcl_publisher_t front_ultra_pub;
sensor_msgs__msg__Range front_ultra_msg;

rcl_publisher_t back_ultra_pub;
sensor_msgs__msg__Range back_ultra_msg;

rcl_publisher_t right_ultra_pub;
sensor_msgs__msg__Range right_ultra_msg;

rcl_publisher_t left_ultra_pub;
sensor_msgs__msg__Range left_ultra_msg;

// ---- Misc. ----
bool rpi_ready = false;
bool core_1_continue = false;
bool core_1_setup_start = false;
bool halt_core_0 = false;

// ---- Loop time ----
uint32_t loop_time, loop_1_time;



// ------- Pin definitions -------

// ---- Misc. ----
#define power_led       1
#define onboard_led     25
#define ready_sig       28
#define pi_power_relay  6

// ---- Ultrasonic sensor ----
#define front_ultra  2
#define back_ultra   3
#define right_ultra  4
#define left_ultra   5

// ---- Edge detection sensors ----
#define edge_sens_mux_s2   13
#define edge_sens_mux_s1   14
#define edge_sens_mux_s0   15
#define edge_sens_mux_sig  26
#define edge_sens_en       0

// ---- Motor encoders & motor driver ----
#define l_motor_1_enc_b  7
#define l_motor_1_enc_a  9
#define l_motor_2_enc_b  8
#define l_motor_2_enc_a  10
#define r_motor_1_enc_b  22
#define r_motor_1_enc_a  11
#define r_motor_2_enc_b  27
#define r_motor_2_enc_a  12
#define l_motor_drive_1  21
#define l_motor_drive_2  20
#define r_motor_drive_1  19
#define r_motor_drive_2  18

// ---- I2C ----
#define i2c_sda  16
#define i2c_scl  17


// ------- Other definitions -------

// ---- Misc. ----
#define loop_time_max    60000    // In microseconds
#define loop_1_time_max  200000   // In microseconds
#define PI               3.1415
#define CHECK_FLAG       1687

// ---- Ultrasonic sensor specs ----
#define ultra_fov        30
#define ultra_min_dist   1     // In cm
#define ultra_max_dist   400   // In cm

// ---- Motor specs ----
#define enc_pulses_per_rotation  2
#define motor_no_pulse_timeout   800000   // In microseconds
#define motor_gear_ratio         80/1
#define wheel_diameter           100.0    // In millimeters
#define wheelbase                140.0    // In millimeters

// ---- Wheel circumference calculation ----
#define wheel_circumference  (PI * wheel_diameter) / 100



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
MotorEncoder* r_motor_encs[] = {&r_motor_1_enc, &r_motor_2_enc};
MotorEncoder* l_motor_encs[] = {&l_motor_1_enc, &l_motor_2_enc};

Motor r_motors(&r_motors_driver, r_motor_encs, 2);
Motor l_motors(&l_motors_driver, l_motor_encs, 2);
MotorSafety r_motors_safety(&r_motors, 0);
MotorSafety l_motors_safety(&l_motors, 1);



// ------- Functions ------- 

// ---- RCL return checker prototype ----
void check_rc(rcl_ret_t rctc);


// ---- Error handler ----
void handle_error(int core, const char * err_msg)
{
    // Publish E_STOP message if the Raspberry Pi computer is active.
    if (rpi_ready)
    {
        sprintf(e_stop_msg.data.data, "E_STOP (PICO_A): %s", err_msg);
        check_rc(rcl_publish(&e_stop_pub, &e_stop_msg, NULL));
    }


    // IO cleanup
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);
    gpio_put(power_led, LOW);
    gpio_put(onboard_led, LOW);
        
    l_motors.disable_controller();
    r_motors.disable_controller();
        
    gpio_put(edge_sens_mux_s2, LOW);
    gpio_put(edge_sens_mux_s1, LOW);
    gpio_put(edge_sens_mux_s0, LOW);
    gpio_put(edge_sens_en, LOW);

    // Micro ROS cleanup
    check_rc(rcl_subscription_fini(&rpi_ready_sub, &rc_node));
    check_rc(rcl_subscription_fini(&cmd_vel_sub, &rc_node));
    check_rc(rcl_publisher_fini(&e_stop_pub, &rc_node));
    check_rc(rcl_node_fini(&rc_node));


    // Stop core 0 (only effective when this function is called from core 1)
    halt_core_0 = true;

    // Stop core 1 if this function is being called from core 0.
    if (core == 0)
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


// ---- Irq callback ----
void irq_call(uint pin, uint32_t events)
{
    r_motor_1_enc.enc_hardware_irq_trigger(pin);
    r_motor_2_enc.enc_hardware_irq_trigger(pin);
    l_motor_1_enc.enc_hardware_irq_trigger(pin);
    l_motor_2_enc.enc_hardware_irq_trigger(pin);
}


// ---- Ultrasonic sensor distance calculator (In cm) ----
float get_ultrasonic_dist(int pin)
{
    init_pin(pin, OUTPUT);
    
    gpio_put(pin, HIGH);
    sleep_us(10);
    gpio_put(pin, LOW);
    sleep_us(10);

    init_pin(pin, INPUT);

    while (!gpio_get(pin));
    uint32_t pulse_start = time_us_32();

    while (gpio_get(pin));
    uint32_t pulse_end = time_us_32();

    uint16_t time_diff = pulse_end - pulse_start;

    return (time_diff * 34.3) / 2;
}


// ---- RCL return checker ----
void check_rc(rcl_ret_t rctc)
{
    if (rctc != RCL_RET_OK)
    {
        handle_error(0, "RC Check failed");
    }
}


// ---- Raspberry Pi 4 status topic callback ----
void rpi_ready_call(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *) msgin;

    if (msg -> data.data == "READY")
    {
        rpi_ready = true;
    }
}


// ---- Command velocity topic callback ----
void cmd_vel_call(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
    
    float angular = msg -> angular.z;     // rad/s
    float linear = msg -> linear.x;       // m/s

    float motor_l_speed_ms = linear - (angular * ((wheelbase / 1000) / 2));      // Calculate motor speeds in m/s
    float motor_r_speed_ms = linear + (angular * ((wheelbase / 1000) / 2));

    r_motors.set_pid_ctrl_speed((motor_r_speed_ms / wheel_circumference) * 60);  // Convert motor speeds from m/s to RPM
    l_motors.set_pid_ctrl_speed((motor_l_speed_ms / wheel_circumference) * 60);           
}


// ---- Setup ROS subscribers and publishers ----
void init_subs_pubs()
{
    const rosidl_message_type_support_t * string_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    const rosidl_message_type_support_t * twist_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    const rosidl_message_type_support_t * range_sens_type = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range);

    // ---- Raspberry Pi ready topic ----
    check_rc(rclc_subscription_init_default(&rpi_ready_sub, &rc_node, string_type, "/rpi_ready"));
    std_msgs__msg__String__init(&rpi_ready_msg);

    // ---- Command velocity topic ----
    check_rc(rclc_subscription_init_default(&cmd_vel_sub, &rc_node, twist_type, "/cmd_vel"));
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    // ---- E-stop topic ----
    check_rc(rclc_publisher_init_default(&e_stop_pub, &rc_node, string_type, "/e_stop"));
    std_msgs__msg__String__init(&e_stop_msg);

    // ---- Front ultrasonic sensor topic ----
    check_rc(rclc_publisher_init_default(&front_ultra_pub, &rc_node, range_sens_type, "/ultrasonic/front"));
    sensor_msgs__msg__Range__init(&front_ultra_msg);

    // ---- Back ultrasonic sensor topic ----
    check_rc(rclc_publisher_init_default(&back_ultra_pub, &rc_node, range_sens_type, "/ultrasonic/back"));
    sensor_msgs__msg__Range__init(&back_ultra_msg);

    // ---- Right ultrasonic sensor topic ----
    check_rc(rclc_publisher_init_default(&right_ultra_pub, &rc_node, range_sens_type, "/ultrasonic/right"));
    sensor_msgs__msg__Range__init(&right_ultra_msg);

    // ---- Left ultrasonic sensor topic ----
    check_rc(rclc_publisher_init_default(&left_ultra_pub, &rc_node, range_sens_type, "/ultrasonic/left"));
    sensor_msgs__msg__Range__init(&left_ultra_msg);
}


// ---- Micro ROS executor init ----
void exec_init()
{
    executor = rclc_executor_get_zero_initialized_executor();
    unsigned int num_handles = 1;

    check_rc(rclc_executor_init(&executor, &rc_supp.context, num_handles, &rc_alloc));
    
    check_rc(rclc_executor_add_subscription(&executor, &rpi_ready_sub, &rpi_ready_msg, &rpi_ready_call, ON_NEW_DATA));
    check_rc(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_call, ON_NEW_DATA));
}


// ---- Publish ultrasonic sensor data ----
void publish_ultra()
{
    front_ultra_msg.range = get_ultrasonic_dist(front_ultra);
    front_ultra_msg.field_of_view = ultra_fov;
    front_ultra_msg.max_range = ultra_max_dist;
    front_ultra_msg.min_range = ultra_min_dist;
    front_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    front_ultra_msg.header.stamp.sec = to_ms_since_boot(get_absolute_time()) / 1000;
    check_rc(rcl_publish(&front_ultra_pub, &front_ultra_msg, NULL));

    back_ultra_msg.range = get_ultrasonic_dist(front_ultra);
    back_ultra_msg.field_of_view = ultra_fov;
    back_ultra_msg.max_range = ultra_max_dist;
    back_ultra_msg.min_range = ultra_min_dist;
    back_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    back_ultra_msg.header.stamp.sec = to_ms_since_boot(get_absolute_time()) / 1000;
    check_rc(rcl_publish(&front_ultra_pub, &back_ultra_msg, NULL));

    right_ultra_msg.range = get_ultrasonic_dist(front_ultra);
    right_ultra_msg.field_of_view = ultra_fov;
    right_ultra_msg.max_range = ultra_max_dist;
    right_ultra_msg.min_range = ultra_min_dist;
    right_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    right_ultra_msg.header.stamp.sec = to_ms_since_boot(get_absolute_time()) / 1000;
    check_rc(rcl_publish(&front_ultra_pub, &right_ultra_msg, NULL));

    left_ultra_msg.range = get_ultrasonic_dist(front_ultra);
    left_ultra_msg.field_of_view = ultra_fov;
    left_ultra_msg.max_range = ultra_max_dist;
    left_ultra_msg.min_range = ultra_min_dist;
    left_ultra_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    left_ultra_msg.header.stamp.sec = to_ms_since_boot(get_absolute_time()) / 1000;
    check_rc(rcl_publish(&front_ultra_pub, &left_ultra_msg, NULL));
}


// ------- Main program -------

// ---- Setup function (Runs once on core 0) ----
void setup()
{
    // ---- Inputs ----
    init_pin(ready_sig, INPUT);
    init_pin(edge_sens_mux_sig, INPUT);

    // ---- Outputs ----
    init_pin(power_led, OUTPUT);
    init_pin(onboard_led, OUTPUT);
    init_pin(pi_power_relay, OUTPUT);
    init_pin(edge_sens_mux_s2, OUTPUT);
    init_pin(edge_sens_mux_s1, OUTPUT);
    init_pin(edge_sens_mux_s0, OUTPUT);
    init_pin(edge_sens_en, OUTPUT);

    init_subs_pubs();
    exec_init();
}


// ---- Setup function (Runs once on core 1) ----
void setup1()
{

}


// ---- Main loop (Runs forever on core 0) ----
void loop()
{
    loop_time = time_us_32();

    // ---- Calculate motor outputs ----
    right_motor_con();
    left_motor_con();

    uint32_t current_time = time_us_32();

    if (motor_r_spd > 10 && (current_time - r_enc_1_time > motor_no_pulse_timeout || current_time - r_enc_2_time > motor_no_pulse_timeout))
    {
        handle_error(0, "Right motor encoder no pulse timout exceeded");
    }

    if (motor_l_spd > 10 && (current_time - l_enc_1_time > motor_no_pulse_timeout || current_time - l_enc_2_time > motor_no_pulse_timeout))
    {
        handle_error(0, "Left motor encoder no pulse timout exceeded");
    }

    if (time_us_32() - loop_time > loop_time_max)
    {
        handle_error(0, "(Loop) max loop time exceeded");
    }
}


// ---- Main loop (Runs forever on core 1) ----
void loop1()
{
    loop_1_time = time_us_32();

    // ---- Publish ultrasonic sensor data ----
    publish_ultra();

    if (time_us_32() - loop_1_time > loop_1_time_max)
    {
        handle_error(1, "(Loop 1) max loop time exceeded");
    }
}


// **************** Init ****************
// ********* END OF MAIN PROGRAM ********

// ---- Core 1 loop init ----
void init_core_1()
{
    multicore_fifo_push_blocking(CHECK_FLAG);
    uint32_t flag_r = multicore_fifo_pop_blocking();
    
    if (flag_r != CHECK_FLAG)
    {
        handle_error(1, "Core 1 FIFO failure");
    }

    else
    {
        // Wait for core_1_setup_start to be set to true
        while (!core_1_setup_start);

        setup1();

        // Wait for core_1_continue to be set to true
        while (!core_1_continue);

        while (true)
        {
            loop1();
        }
    }
}


// ---- Micro ROS node init ----
void uros_init(const char * node_name, const char * name_space)
{
    rc_alloc = rcl_get_default_allocator();
    check_rc(rclc_support_init(&rc_supp, 0, NULL, &rc_alloc));
    check_rc(rclc_node_init_default(&rc_node, node_name, name_space, &rc_supp));
}


// ---- Startup function ----
void startup()
{
    init_pins();

    uros_init("pico_a", "io");

    setup();
    core_1_setup_start = true;

    // Wait for Pico (B) ready signal
    while (!gpio_get(ready_sig))
    {
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(800);
        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(800);
    }

    // Activate the Raspberry Pi 4
    gpio_put(pi_power_relay, HIGH);

    // Start the Micro ROS executor
    rclc_executor_spin(&executor);

    // Wait for Raspberry Pi 4 ready signal
    while (!rpi_ready)
    {
        gpio_put(power_led, HIGH);
        gpio_put(onboard_led, HIGH);
        sleep_ms(380);
        gpio_put(power_led, LOW);
        gpio_put(onboard_led, LOW);
        sleep_ms(380);
    }

    // Send ready signal to Pico (B)
    init_pin(ready_sig, OUTPUT);
    gpio_put(ready_sig, HIGH);

    core_1_continue = true;
}


// ---- Main function (Runs setup, loop, and loop1) ----
int main() 
{
    multicore_reset_core1();
    multicore_launch_core1(init_core_1);

    multicore_fifo_push_blocking(CHECK_FLAG);
    uint32_t flag_r = multicore_fifo_pop_blocking();
    
    if (flag_r != CHECK_FLAG)
    {
        init_pins();
        handle_error(0, "Core 0 FIFO failure");
    }

    else
    {
        startup();
        gpio_put(power_led, HIGH);

        // Start core 0 loop
        while (!halt_core_0)
        {
            loop();
        }
    }

    return 0;
}