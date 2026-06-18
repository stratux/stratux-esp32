cmake_minimum_required(VERSION 3.16)

# --- Build-time board/radio selection -------------------------------------
# BOARD selects the hardware pin map. RADIO selects the output variant.
#
#   idf.py -B build-devkitc -D BOARD=devkitc_v4 -D RADIO=wifi build
#
if(NOT DEFINED BOARD)
    set(BOARD "ttgo_t8" CACHE STRING "Board configuration: ttgo_t8 | devkitc_v4")
endif()
if(NOT BOARD STREQUAL "ttgo_t8" AND NOT BOARD STREQUAL "devkitc_v4")
    message(FATAL_ERROR "BOARD must be 'ttgo_t8' or 'devkitc_v4', got '${BOARD}'")
endif()

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
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.${RADIO};sdkconfig.defaults.${BOARD}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(stratux_esp32)
