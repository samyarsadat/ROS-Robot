#  The ROS robot project - Firmware CMake config (NOTE: THIS IS NOT THE TOP-LEVEL CMAKE FILE!)
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


# Custom library subdirectories
add_subdirectory(lib/Motor_lib)
add_subdirectory(lib/Helper_lib)
add_subdirectory(lib/PID_lib)



# ---- Pico A ----
message("Configuring Pico A firmware build... (${PICO_A_NAME})")

# Add source files
add_executable(${PICO_A_NAME} Pico_A.c++
               pico_a_helpers/IO_Helpers_Mux.c++
               pico_a_helpers/IO_Helpers_Edge.c++
               pico_a_helpers/IO_Helpers_Ultrasonic.c++
               pico_a_helpers/uROS_Init.c++
               uart_transport/pico_uart_transport.c)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PICO_A_NAME})

# Link to libraries
target_link_libraries(${PICO_A_NAME} micro_ros)
target_link_libraries(${PICO_A_NAME} pico_stdlib)
target_link_libraries(${PICO_A_NAME} pico_multicore)
target_link_libraries(${PICO_A_NAME} hardware_pwm)
target_link_libraries(${PICO_A_NAME} hardware_adc)
target_link_libraries(${PICO_A_NAME} hardware_gpio)
target_link_libraries(${PICO_A_NAME} helpers_lib)
target_link_libraries(${PICO_A_NAME} pid_v1_lib)
target_link_libraries(${PICO_A_NAME} motor_control_lib)

# Link to include directories
target_include_directories(${PICO_A_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Enable USB, disable UART
pico_enable_stdio_usb(${PICO_A_NAME} 1)
pico_enable_stdio_uart(${PICO_A_NAME} 0)

# UART Config
target_compile_definitions(${PICO_A_NAME} PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
)



# ---- Pico B ----
message("Configuring Pico B firmware build... (${PICO_B_NAME})")

# Add source files
add_executable(${PICO_B_NAME} Pico_B.c++ 
               uart_transport/pico_uart_transport.c)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PICO_B_NAME})

# Link to libraries
target_link_libraries(${PICO_B_NAME} micro_ros)
target_link_libraries(${PICO_B_NAME} pico_stdlib)
target_link_libraries(${PICO_B_NAME} pico_multicore)
target_link_libraries(${PICO_B_NAME} hardware_pwm)
target_link_libraries(${PICO_B_NAME} hardware_adc)
target_link_libraries(${PICO_B_NAME} hardware_gpio)
target_link_libraries(${PICO_B_NAME} helpers_lib)

# Link to include directories
target_include_directories(${PICO_B_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Enable USB, disable UART
pico_enable_stdio_usb(${PICO_B_NAME} 1)
pico_enable_stdio_uart(${PICO_B_NAME} 0)

# UART Config
target_compile_definitions(${PICO_A_NAME} PRIVATE
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=8
    PICO_DEFAULT_UART_RX_PIN=9
)



# CMake flags
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Compile definitions & options
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_options(-Wall -Wextra -Wpedantic)