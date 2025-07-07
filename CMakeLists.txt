cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/drivers/zephyr)

list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(MiracZ7)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
target_sources(app PRIVATE src/main.c)
