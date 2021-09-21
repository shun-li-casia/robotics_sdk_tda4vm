cmake_minimum_required(VERSION 3.5)
project(ti_sde)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/ros2_common.cmake)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    # SDE example
    set(APP_SDE_SRCS
        src/sde_main.cpp
        src/sde_node.cpp)

    # Build nodes
    build_node(sde APP_SDE_SRCS)

    ament_package()
endif()
