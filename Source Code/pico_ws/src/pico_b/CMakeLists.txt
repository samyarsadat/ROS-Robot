#  The ROS robot project - Firmware CMake config, Pico B (NOTE: THIS IS NOT THE TOP-LEVEL CMAKE FILE!)
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



# ---- Pico B ----
message("Configuring Pico B firmware build... (${PICO_B_NAME})")

# Add source files
add_executable(${PICO_B_NAME}
               ${FREERTOS_POSIX_SRC_PREFIX}_clock.c
               #${FREERTOS_POSIX_SRC_PREFIX}_mqueue.c
               #${FREERTOS_POSIX_SRC_PREFIX}_pthread_barrier.c
               #${FREERTOS_POSIX_SRC_PREFIX}_pthread_cond.c
               #${FREERTOS_POSIX_SRC_PREFIX}_pthread_mutex.c
               #${FREERTOS_POSIX_SRC_PREFIX}_pthread.c
               ${FREERTOS_POSIX_SRC_PREFIX}_sched.c
               #${FREERTOS_POSIX_SRC_PREFIX}_semaphore.c
               #${FREERTOS_POSIX_SRC_PREFIX}_timer.c
               ${FREERTOS_POSIX_SRC_PREFIX}_unistd.c
               ${FREERTOS_POSIX_SRC_PREFIX}_utils.c
               ../uros_allocators/uros_allocators.c
               ../uart_transport/pico_uart_transport.c
               Pico_B.c++ 
               helpers/uROS_Init.c++
               helpers/IO_Helpers_General.c++
               )

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PICO_B_NAME})

# Link to libraries
target_link_libraries(${PICO_B_NAME} micro_ros
                                     pico_stdlib
                                     pico_multicore
                                     hardware_pwm
                                     hardware_adc
                                     hardware_gpio
                                     helpers_lib
                                     local_helpers_lib
                                     FreeRTOS-Kernel
                                     FreeRTOS-Kernel-Heap4
)

# Link to include directories
target_include_directories(${PICO_B_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# UART Config
target_compile_definitions(${PICO_B_NAME} PRIVATE
    PICO_DEOPTIMIZED_DEBUG=1
    PICO_RP2040_B0_SUPPORTED=0
    PICO_RP2040_B1_SUPPORTED=0
    PICO_STACK_SIZE=0x1000
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=8
    PICO_DEFAULT_UART_RX_PIN=9
)

# PicoTool binary information
pico_set_program_name(${PICO_B_NAME} ${PICO_B_NAME})
pico_set_program_version(${PICO_B_NAME} ${PROJECT_VERSION})
pico_set_program_description(${PICO_B_NAME} "The ROS Robot Project - Main Board Pico B Firmware")

# Enable STDIO over USB and UART
pico_enable_stdio_usb(${PICO_B_NAME} 1)
pico_enable_stdio_uart(${PICO_B_NAME} 1)

# CMake flags
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Compile definitions & options
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_options(-Wall -Wextra -Wpedantic)