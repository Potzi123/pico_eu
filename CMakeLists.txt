# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_30_10_2024 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_subdirectory(libs/eInk/EPD_1in54_V2)
add_subdirectory(libs/eInk/Fonts)
add_subdirectory(libs/eInk/GUI)


add_library(BME68x_LIB STATIC libs/bme688/api/bme68x_SensorAPI/bme68x.c)


add_executable(pico_30_10_2024 
    pico_30_10_2024.cpp 
    libs/bme688/bme688.cpp
    libs/hm3301/hm3301.cpp
    libs/pas_co2/pas_co2.cpp
    libs/adc/adc.cpp
    libs/wifi/wifi.cpp
    libs/eInk/EPD_1in54_V2/EPD_1in54_V2.c    
    libs/eInk/GUI/GUI_Paint.c
    libs/eInk/Fonts/font8.c
    libs/eInk/Fonts/font12.c
    libs/eInk/Fonts/font16.c
    libs/eInk/Fonts/font20.c
    libs/eInk/Fonts/font24.c
    libs/flash/pico_flash.c
    libs/flash/pico_flash.h
)

pico_set_program_name(pico_30_10_2024 "pico_30_10_2024")
pico_set_program_version(pico_30_10_2024 "0.1")

# Generate PIO header
pico_generate_pio_header(pico_30_10_2024 ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_30_10_2024 0)
pico_enable_stdio_usb(pico_30_10_2024 1)

# Add the standard library to the build
target_link_libraries(pico_30_10_2024 
    pico_stdlib
    hardware_adc
    hardware_spi
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_interp
    hardware_timer
    hardware_watchdog
    hardware_flash
    hardware_sync
    pico_stdio_usb
    pico_cyw43_arch_lwip_threadsafe_background
    BME68x_LIB
    epd_1in54_v2 
    epd_gui_paint 
    epd_fonts 
)

# Add the standard include files to the build
target_include_directories(pico_30_10_2024 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/libs/bme688
    ${CMAKE_CURRENT_LIST_DIR}/libs/bme688/api
    ${CMAKE_CURRENT_LIST_DIR}/libs/hm3301
    ${CMAKE_CURRENT_LIST_DIR}/libs/pas_co2
    ${CMAKE_CURRENT_LIST_DIR}/libs/adc
    ${CMAKE_CURRENT_LIST_DIR}/libs/wifi
    ${CMAKE_CURRENT_LIST_DIR}/libs/eInk
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(pico_30_10_2024)