# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(PICO_SDK_PATH "C:/Users/j_gab/Documents/Pico/pico-sdk")

set(FREERTOS_KERNEL_PATH "C:/Users/j_gab/Documents/Pico/FreeRTOS-Kernel")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

include(FreeRTOS_Kernel_import.cmake)

project(neptuneProject C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
if(TARGET tinyusb_device)
    add_executable(neptuneProject neptuneProject.c mpu9250.c)

    add_subdirectory(lib/pico-nrf24/lib/nrf24l01)

    target_include_directories(neptuneProject PUBLIC ./)

    pico_set_program_name(neptuneProject "neptuneProject")
    pico_set_program_version(neptuneProject "0.1")

    pico_enable_stdio_uart(neptuneProject 0)
    pico_enable_stdio_usb(neptuneProject 1)

    # Add the standard library to the build
    target_link_libraries(neptuneProject
        pico_stdlib
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        hardware_spi
        hardware_adc
        hardware_timer
        nrf24_driver
    )

    pico_add_extra_outputs(neptuneProject)

    elseif(PICO_ON_DEVICE)
        message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
    endif()

