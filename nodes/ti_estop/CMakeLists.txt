cmake_minimum_required(VERSION 3.0.2)
project(ti_estop)

# Build only on J7
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

    include_directories(../common)

    # AMR E-Stop example
    set(APP_ESTOP_SRCS
        ../common/cm_common.cpp
        ../common/cm_profile.cpp
        src/estop_main.cpp
        src/estop_node.cpp
        src/estop.cpp
        src/estop_manage_graph.cpp)

    # Build nodes
    build_node(estop APP_ESTOP_SRCS)
endif()