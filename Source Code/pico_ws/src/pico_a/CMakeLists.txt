#  The ROS robot project - Firmware CMake config, Pico A (NOTE: THIS IS NOT THE TOP-LEVEL CMAKE FILE!)
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



# ---- Pico A ----
message("Configuring Pico A firmware build... (${PICO_A_NAME})")

# Add source files
add_executable(${PICO_A_NAME} Pico_A.c++
                              helpers/IO_Helpers_Mux.c++
                              helpers/IO_Helpers_Edge.c++
                              helpers/IO_Helpers_Ultrasonic.c++
                              helpers/uROS_Init.c++
                              ../posix/functions.c)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PICO_A_NAME})

# Link to libraries
target_link_libraries(${PICO_A_NAME} micro_ros
                                     haw_mpu6050
                                     pico_stdlib
                                     hardware_adc
                                     helpers_lib
                                     local_helpers_lib
                                     pid_v1_lib
                                     motor_control_lib
                                     freertos_helpers_lib
                                     FreeRTOS-Kernel
                                     FreeRTOS-Kernel-Heap4)

# Link to include directories
target_include_directories(${PICO_A_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# UART Config
target_compile_definitions(${PICO_A_NAME} PRIVATE
    PICO_RP2040_B0_SUPPORTED=0
    PICO_RP2040_B1_SUPPORTED=0
    PICO_STACK_SIZE=0x1000
    PICO_USE_STACK_GUARDS=1
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
)

# PicoTool binary information
pico_set_program_name(${PICO_A_NAME} ${PICO_A_NAME})
pico_set_program_version(${PICO_A_NAME} ${PROJECT_VERSION})
pico_set_program_description(${PICO_A_NAME} "The ROS Robot Project - Main Board Pico A Firmware")

# Enable STDIO over USB and UART
pico_enable_stdio_usb(${PICO_A_NAME} 1)
pico_enable_stdio_uart(${PICO_A_NAME} 1)

# CMake flags
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Compile definitions & options
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_options(-Wall -Wextra -Wpedantic)