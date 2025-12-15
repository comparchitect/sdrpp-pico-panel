cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_panel C CXX ASM)

pico_sdk_init()

add_executable(pico_panel
    pico_panel.cpp
)

target_link_libraries(pico_panel pico_stdlib hardware_adc hardware_i2c)

pico_enable_stdio_usb(pico_panel 1)
pico_enable_stdio_uart(pico_panel 0)

pico_add_extra_outputs(pico_panel)
