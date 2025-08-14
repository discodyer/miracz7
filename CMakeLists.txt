cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(MiracZ7)

if(CONFIG_MICROXRCEDDSCLIENT)
  target_link_libraries(app PRIVATE microxrceddsclient)
endif()

target_sources(app PRIVATE src/main.c src/HelloWorld.c src/microxrce_transports.c)

target_include_directories(app PRIVATE 
  ${ZEPHYR_MICRO_CDR_MODULE_DIR}/include
  ${ZEPHYR_MICRO_XRCE_DDS_CLIENT_MODULE_DIR}/include
)