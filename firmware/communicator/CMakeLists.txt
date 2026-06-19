# Cassette-futurism communicator/scanner prop firmware
# See plan + readme.md. Forked from example/V1.0/idf-code/Lesson09-LVGL_Lighting_Control.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS "peripheral")
project(communicator)
