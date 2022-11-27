cmake_minimum_required(VERSION 3.20.0)

add_compile_options(-fdiagnostics-color=always)
add_compile_options(-Wall)
add_compile_options(-Werror)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(radio_rx)

target_sources(app PRIVATE src/main.c src/rx_uart.c src/rx_radio.c src/common/radio.c)