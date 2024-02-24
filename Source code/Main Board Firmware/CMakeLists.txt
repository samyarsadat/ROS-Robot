#  The ROS robot project - CMake config
#  Copyright 2022-2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2022-2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.


# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set project name and C/C++ standards
project(ROS_Robot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Micro-ROS
link_directories($ENV{MICROROS_PATH}/libmicroros)


# Custom library subdirectories
add_subdirectory(lib/Motor_lib)
add_subdirectory(lib/Helper_lib)
add_subdirectory(lib/PID_lib)


# ---- Pico A ----

# Add source files
add_executable(${PROJECT_NAME}_PICO_A Pico_A.c++
               Pico_A_helpers/IO_Helpers_Mux.c++
               Pico_A_helpers/IO_Helpers_Edge.c++
               Pico_A_helpers/IO_Helpers_Ultrasonic.c++
               Pico_A_helpers/uROS_Init.c++
               $ENV{MICROROS_PATH}/pico_uart_transport.c)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_PICO_A)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_PICO_A microros)
target_link_libraries(${PROJECT_NAME}_PICO_A pico_stdlib)
target_link_libraries(${PROJECT_NAME}_PICO_A pico_multicore)
target_link_libraries(${PROJECT_NAME}_PICO_A hardware_pwm)
target_link_libraries(${PROJECT_NAME}_PICO_A hardware_adc)
target_link_libraries(${PROJECT_NAME}_PICO_A hardware_gpio)
target_link_libraries(${PROJECT_NAME}_PICO_A helpers_lib)
target_link_libraries(${PROJECT_NAME}_PICO_A pid_v1_lib)
target_link_libraries(${PROJECT_NAME}_PICO_A motor_control_lib)

# Link to include directories
target_include_directories(${PROJECT_NAME}_PICO_A PUBLIC $ENV{MICROROS_PATH}/libmicroros/include)
target_include_directories(${PROJECT_NAME}_PICO_A PUBLIC $ENV{MICROROS_PATH})

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_PICO_A 1)
pico_enable_stdio_uart(${PROJECT_NAME}_PICO_A 0)


# ---- Pico B ----

# Add source files
add_executable(${PROJECT_NAME}_PICO_B Pico_B.c++ $ENV{MICROROS_PATH}/pico_uart_transport.c)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_PICO_B)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_PICO_B microros)
target_link_libraries(${PROJECT_NAME}_PICO_B pico_stdlib)
target_link_libraries(${PROJECT_NAME}_PICO_B pico_multicore)
target_link_libraries(${PROJECT_NAME}_PICO_B hardware_pwm)
target_link_libraries(${PROJECT_NAME}_PICO_B hardware_adc)
target_link_libraries(${PROJECT_NAME}_PICO_B hardware_gpio)
target_link_libraries(${PROJECT_NAME}_PICO_B helpers_lib)

# Link to include directories
target_include_directories(${PROJECT_NAME}_PICO_B PUBLIC $ENV{MICROROS_PATH}/libmicroros/include)
target_include_directories(${PROJECT_NAME}_PICO_B PUBLIC $ENV{MICROROS_PATH})

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_PICO_B 1)
pico_enable_stdio_uart(${PROJECT_NAME}_PICO_B 0)



# ---- ENCODER TEST ----

# Add source files
add_executable(${PROJECT_NAME}_ENC_TEST Enc_Systest.c++)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_ENC_TEST)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_ENC_TEST pico_stdlib)
target_link_libraries(${PROJECT_NAME}_ENC_TEST helpers_lib)

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_ENC_TEST 1)
pico_enable_stdio_uart(${PROJECT_NAME}_ENC_TEST 0)



# ---- ENCODER TEST WITH MOTOR AND PID ----

# Add source files
add_executable(${PROJECT_NAME}_ENC_TEST_MOTOR_PID Enc_Systest_Motor_PID.c++)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_ENC_TEST_MOTOR_PID)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_ENC_TEST_MOTOR_PID pico_stdlib)
target_link_libraries(${PROJECT_NAME}_ENC_TEST_MOTOR_PID hardware_pwm)
target_link_libraries(${PROJECT_NAME}_ENC_TEST_MOTOR_PID hardware_adc)
target_link_libraries(${PROJECT_NAME}_ENC_TEST_MOTOR_PID hardware_gpio)
target_link_libraries(${PROJECT_NAME}_ENC_TEST_MOTOR_PID helpers_lib)
target_link_libraries(${PROJECT_NAME}_ENC_TEST_MOTOR_PID pid_v1_lib)

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_ENC_TEST_MOTOR_PID 1)
pico_enable_stdio_uart(${PROJECT_NAME}_ENC_TEST_MOTOR_PID 0)



# ---- TESTING SANDBOX ----

# Add source files
add_executable(${PROJECT_NAME}_TEST_SANDBOX Test_Sandbox.c++)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME}_TEST_SANDBOX)

# Link to libraries
target_link_libraries(${PROJECT_NAME}_TEST_SANDBOX pico_stdlib)
target_link_libraries(${PROJECT_NAME}_TEST_SANDBOX helpers_lib)
target_link_libraries(${PROJECT_NAME}_TEST_SANDBOX motor_control_lib)

# Enable USB, disable UART
pico_enable_stdio_usb(${PROJECT_NAME}_TEST_SANDBOX 1)
pico_enable_stdio_uart(${PROJECT_NAME}_TEST_SANDBOX 0)

target_compile_definitions(${PROJECT_NAME}_TEST_SANDBOX PRIVATE
  PICO_DEFAULT_UART=0
  PICO_DEFAULT_UART_TX_PIN=16
  PICO_DEFAULT_UART_RX_PIN=17
)


# CMake flags
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Compile definitions & options
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_options(-Wall -Wextra -Wpedantic)