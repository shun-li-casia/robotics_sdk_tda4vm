cmake_minimum_required(VERSION 3.0.2)
project(ti_viz_color_conversion)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

# Visualization nodes only built on PC
if(${TARGET_PLATFORM} STREQUAL "PC")

    # yuv420 to RGB conversion
    set(VIZ_COL_CONV_SRCS src/ros_app_color_conversion_node.cpp)
    build_node(viz_color_conversion VIZ_COL_CONV_SRCS)

endif()

