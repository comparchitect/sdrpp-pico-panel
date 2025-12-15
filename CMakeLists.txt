cmake_minimum_required(VERSION 3.13)
project(sdrpp_pico_panel)

# You must point this to the SDR++ Brown build dir when configuring:
#   cmake .. -DSDRPP_MODULE_CMAKE=/home/dimeco/SDRPlusPlusBrown/build/sdrpp_module.cmake
if (NOT SDRPP_MODULE_CMAKE)
    message(FATAL_ERROR "Set -DSDRPP_MODULE_CMAKE=/home/dimeco/SDRPlusPlusBrown/build/sdrpp_module.cmake")
endif()

if (NOT SDRPP_CORE_ROOT)
    get_filename_component(SDRPP_MODULE_DIR "${SDRPP_MODULE_CMAKE}" DIRECTORY)
    get_filename_component(SDRPP_SOURCE_ROOT "${SDRPP_MODULE_DIR}" DIRECTORY)
    set(SDRPP_CORE_ROOT "${SDRPP_SOURCE_ROOT}/core")
endif()

if (NOT SDRPP_SOURCE_ROOT)
    get_filename_component(SDRPP_SOURCE_ROOT "${SDRPP_CORE_ROOT}/.." ABSOLUTE)
endif()

set(SRC
    src/pico_panel.cpp
)

include(${SDRPP_MODULE_CMAKE})

target_include_directories(${PROJECT_NAME} PRIVATE
    "${SDRPP_CORE_ROOT}/src/imgui"
    "${SDRPP_SOURCE_ROOT}/decoder_modules"
    "${SDRPP_SOURCE_ROOT}/decoder_modules/radio/src"
)
