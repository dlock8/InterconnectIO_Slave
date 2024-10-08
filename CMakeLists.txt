cmake_minimum_required(VERSION 3.18)

include(pico_sdk_import.cmake)

project(PICO_I2C_SLAVE C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICOTOOL_FETCH_FROM_GIT_PATH "~/pico")  # the directory where you want picotool to be fetched

pico_sdk_init()

add_subdirectory(firmware)



