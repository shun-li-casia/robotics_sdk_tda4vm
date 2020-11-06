cmake_minimum_required(VERSION 3.0.2)
project(ti_viz_disparity)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

# Visualization nodes only built on PC
if(${TARGET_PLATFORM} STREQUAL "PC")
    set(VIZ_DISP_SRCS src/viz_disparity_node.cpp)
    build_node(viz_disparity VIZ_DISP_SRCS)
    add_dependencies(viz_disparity ${catkin_EXPORTED_TARGETS})
endif()




