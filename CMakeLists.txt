# The following CMake or environment variables should be defined:

cmake_minimum_required(VERSION 3.14)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(plc-cleaner C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# FREERTOS: include FreeRTOS Kernel libraries
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

add_executable(${PROJECT_NAME} ./src/main.c)
# FREERTOS: FreeRTOSConfig.h needs to be in the include path
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
# FREERTOS: Note, you should pick the FreeRTOS library that suits you best:
#
# FreeRTOS-Kernel-Heap1 thru FreeRTOS-Kernel_Heap4
# or
# FreeRTOS-Kernel-Static

add_subdirectory(extern/hx711-pico-c)
add_subdirectory(extern/onewire_library)
target_link_libraries(${PROJECT_NAME}
  pico_stdlib
  hardware_i2c
  FreeRTOS-Kernel-Heap4
  hardware_pio
  hx711-pico-c
  onewire_library
)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})