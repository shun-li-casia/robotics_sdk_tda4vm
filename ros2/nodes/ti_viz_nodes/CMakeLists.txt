cmake_minimum_required(VERSION 3.5)
project(ti_viz_nodes)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/ros2_common.cmake)

# Visualization nodes only built on PC
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    # viz_disparity
    set(VIZ_DISP_SRCS src/viz_disparity_node.cpp)
    build_node(viz_disparity VIZ_DISP_SRCS)

    # YUV to RGB conversion
    set(VIZ_COL_CONV_SRCS src/ros_app_color_conversion_node.cpp)
    build_node(viz_color_conv_yuv2rgb VIZ_COL_CONV_SRCS)

    # viz_semseg
    set(VIZ_SEMSEG_SRCS src/viz_semseg_node.cpp)
    build_node(viz_semseg VIZ_SEMSEG_SRCS)

    # viz_objdet
    set(VIZ_OBJDET_SRCS src/viz_objdet_node.cpp)
    build_node(viz_objdet VIZ_OBJDET_SRCS)

    # viz_estop
    set(VIZ_ESTOP_SRCS src/viz_estop_node.cpp)
    build_node(viz_estop VIZ_ESTOP_SRCS)

    ament_package()
endif()
