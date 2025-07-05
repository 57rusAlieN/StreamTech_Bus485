cmake_minimum_required(VERSION 3.20.0)

# Указываем абсолютный путь к overlay и dts
get_filename_component(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../driver ABSOLUTE)

# Подключаем модуль драйвера
list(APPEND ZEPHYR_EXTRA_MODULES ${DRIVER_DIR})

# Добавляем DTS корни для поиска биндингов
list(APPEND DTS_ROOT ${DRIVER_DIR}/dts)
list(APPEND DTS_ROOT $ENV{ZEPHYR_BASE}/dts)

# Подключаем DTS драйвера
list(APPEND EXTRA_DTC_OVERLAY_FILE ${DRIVER_DIR}/bus485.overlay)

message(STATUS "DTS_ROOT: ${DTS_ROOT}")
message(STATUS "Overlay: ${EXTRA_DTC_OVERLAY_FILE}")

# Подключаем Zephyr RTOS
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(basic_sample)

# Добавляем исходники приложения
target_sources(app PRIVATE src/main.c)