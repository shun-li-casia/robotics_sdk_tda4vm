cmake_minimum_required(VERSION 3.0.2)
project(ti_sde)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

# LDC+SDE example
set(APP_SDE_SRCS
    src/sde_main.cpp
    src/sde_node.cpp
    src/sde.cpp
    src/sde_manage_graph.cpp)

# Build nodes
build_node(sde APP_SDE_SRCS)
