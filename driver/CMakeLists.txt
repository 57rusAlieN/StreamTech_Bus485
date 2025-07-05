cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_SKIP_BUILD_RPATH TRUE)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/dts)
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/bus485.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# project(bus485_d LANGUAGES C)
# target_sources(app PRIVATE build.debug.main.c)

zephyr_library()
zephyr_include_directories(inc INTERFACE)
zephyr_library_sources(
    src/bus485.c
    inc/bus485.h
)
