# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoBopItTesting C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Adds a CMakeLists.txt file from a subdirectory
add_subdirectory(vendors)

# Add executable. Default name is the project name, version 0.1

add_executable(PicoBopItTesting
    PicoBopItTesting.c
    )

# pull in common dependencies
target_link_libraries(PicoBopItTesting
    pico_stdlib
    hardware_pwm
    hardware_adc
    pico_multicore
    hardware_irq
    hardware_sync
    hardware_clocks
    pico_rand
    haw_mpu6050
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(PicoBopItTesting pico_cyw43_arch_none)
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(PicoBopItTesting)

# add url via pico_set_program_url


pico_enable_stdio_usb(PicoBopItTesting 1)