cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(buddy3_barcode_bringup C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

set(BUDDY3_LOGIC
    bar_capture.c
    bar_ring.c
    barcode_decode.c
    line_logic.c
)

add_executable(barcode_bringup
    barcode_bringup.c
    ${BUDDY3_LOGIC}
)

target_include_directories(barcode_bringup PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(barcode_bringup pico_stdlib)

pico_enable_stdio_usb(barcode_bringup 1)
pico_enable_stdio_uart(barcode_bringup 0)

pico_add_extra_outputs(barcode_bringup)
