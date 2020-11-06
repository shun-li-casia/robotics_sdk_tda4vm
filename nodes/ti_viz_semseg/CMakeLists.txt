cmake_minimum_required(VERSION 3.0.2)
project(ti_viz_semseg)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

# Visualization nodes only built on PC
if(${TARGET_PLATFORM} STREQUAL "PC")
    set(VIZ_SEMSEG_SRCS src/viz_semseg_node.cpp)
    build_node(viz_semseg VIZ_SEMSEG_SRCS)
    add_dependencies(viz_semseg ${catkin_EXPORTED_TARGETS})
endif()



