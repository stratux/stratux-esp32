cmake_minimum_required(VERSION 3.16)

# --- Build-time radio selection -------------------------------------------
# The WiFi build is the primary GDL90/UDP output path and the default. The
# original ESP32 cannot host BT Classic + Bluedroid + WiFi + lwIP in one image
# (see AGENTS.md), so an optional Connext/BT variant (M6) ships as a separate
# binary, selected here:
#
#   . $IDF_PATH/export.sh
#   idf.py -B build-wifi -D RADIO=wifi build
#   idf.py -B build-wifi -p <PORT> -b 115200 flash monitor   # note 115200
#   idf.py -B build-bt   -D RADIO=bt   build                 # M6, optional
#
# RADIO drives both the sdkconfig fragment layered below and (eventually) the
# source/component gating in main/CMakeLists.txt.
if(NOT DEFINED RADIO)
    set(RADIO "wifi" CACHE STRING "Radio configuration: wifi | bt")
endif()
if(NOT RADIO STREQUAL "wifi" AND NOT RADIO STREQUAL "bt")
    message(FATAL_ERROR "RADIO must be 'wifi' or 'bt', got '${RADIO}'")
endif()
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.${RADIO}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(stratux_esp32)
