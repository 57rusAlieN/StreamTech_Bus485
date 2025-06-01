cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bus485)

zephyr_library()
zephyr_library_sources(
    src/bus485.c
    src/manager.c
    # src/spi_context_ext.c
)
zephyr_library_include_directories(include)