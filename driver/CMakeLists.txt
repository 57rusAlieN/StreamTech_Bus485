cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bus485)

zephyr_include_directories(app PRIVATE
    inc
)
target_sources(app PRIVATE
    src/bus485.c
)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/dts)