cmake_minimum_required(VERSION 3.0.2)
project(ti_viz_nodes)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs std_msgs message_generation common_msgs)

catkin_package(CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs message_runtime)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

# Function for building a node:
# ARG0: app name
# ARG1: source list
function(build_node)
    set(app ${ARGV0})
    set(src ${ARGV1})
    add_executable(${app} ${${src}})
    target_link_libraries(${app} ${catkin_LIBRARIES})
    add_dependencies(${app} ${common_msgs_EXPORTED_TARGETS})
endfunction()

# Visualization nodes only built on PC
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    # viz_disparity
    set(VIZ_DISP_SRCS src/viz_disparity_node.cpp)
    build_node(viz_disparity VIZ_DISP_SRCS)
    add_dependencies(viz_disparity ${catkin_EXPORTED_TARGETS})

    # YUV to RGB conversion
    set(VIZ_COL_CONV_SRCS src/ros_app_color_conversion_node.cpp)
    build_node(viz_color_conversion VIZ_COL_CONV_SRCS)

    # viz_semseg
    set(VIZ_SEMSEG_SRCS src/viz_semseg_node.cpp)
    build_node(viz_semseg VIZ_SEMSEG_SRCS)
    add_dependencies(viz_semseg ${catkin_EXPORTED_TARGETS})

    # viz_estop
    set(VIZ_ESTOP_SRCS src/viz_estop_node.cpp)
    build_node(viz_estop VIZ_ESTOP_SRCS)
    add_dependencies(viz_estop ${catkin_EXPORTED_TARGETS})
endif()




