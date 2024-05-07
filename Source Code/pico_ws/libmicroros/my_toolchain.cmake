include($ENV{PICO_SDK_PATH}/cmake/preload/toolchains/find_compiler.cmake)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if (NOT PICO_GCC_TRIPLE)
    if (DEFINED ENV{PICO_GCC_TRIPLE})
        set(PICO_GCC_TRIPLE $ENV{PICO_GCC_TRIPLE})
        message("PICO_GCC_TRIPLE set from environment: $ENV{PICO_GCC_TRIPLE}")
    else()
        set(PICO_GCC_TRIPLE arm-none-eabi)
        message("PICO_GCC_TRIPLE defaulted to arm-none-eabi")
    endif()
endif()

pico_find_compiler(PICO_COMPILER_CC ${PICO_GCC_TRIPLE}-gcc)
pico_find_compiler(PICO_COMPILER_CXX ${PICO_GCC_TRIPLE}-g++)
set(CMAKE_C_COMPILER ${PICO_COMPILER_CC} CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER ${PICO_COMPILER_CXX} CACHE FILEPATH "C++ compiler")

SET(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
SET(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

set(FLAGS "-O2 -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -fno-exceptions -nostdlib -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

# FreeRTOS + POSIX
#set(FREERTOS_POSIX_PATH "${CMAKE_CURRENT_LIST_DIR}/../freertos/FreeRTOS-POSIX")
#include_directories("${FREERTOS_POSIX_PATH}/include")
#include_directories("${FREERTOS_POSIX_PATH}/include/FreeRTOS_POSIX")
#include_directories("${FREERTOS_POSIX_PATH}/include/private")
#include_directories("${FREERTOS_POSIX_PATH}/FreeRTOS-Plus-POSIX/include")
#include_directories("${FREERTOS_POSIX_PATH}/FreeRTOS-Plus-POSIX/include/portable")

# FreeRTOS Kernel
add_compile_definitions(PLATFORM_NAME_FREERTOS)
set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/../freertos/FreeRTOS-Kernel")
set(FREERTOS_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../freertos")
include_directories("${FREERTOS_CONFIG_DIR}") 
include_directories("${FREERTOS_KERNEL_PATH}/include") 
include_directories("${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include")

# Raspberry Pi Pico SDK headers
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include_directories("${PICO_SDK_PATH}/src/common/pico_base/include")
include_directories("${PICO_SDK_PATH}/src/rp2_common/pico_platform/include")
include_directories("${PICO_SDK_PATH}/src/rp2040/hardware_regs/include")
include_directories("${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include")
include_directories("${PICO_SDK_PATH}/src/rp2_common/hardware_base/include")
include_directories("${CMAKE_CURRENT_LIST_DIR}/../build/generated/pico_base")   # Auto-generated headers