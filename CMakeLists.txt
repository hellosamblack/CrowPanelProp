# Cassette-futurism communicator/scanner prop firmware
# See CLAUDE.md. Originally forked from the Elecrow example
# Lesson09-LVGL_Lighting_Control (now in the reference/ submodule).
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS "peripheral")
project(communicator)
