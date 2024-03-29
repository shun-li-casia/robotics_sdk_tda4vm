cmake_minimum_required(VERSION 3.5)

project(gscam2)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(BUILD_CORE_NODES OFF)
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(BUILD_CORE_NODES ON CACHE BOOL "Build core nodes")
else()
  message(FATAL_ERROR "Unknown processor:" ${CMAKE_SYSTEM_PROCESSOR})
endif()

if(${BUILD_CORE_NODES})

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Gstreamer doesn't provide CMake files
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# GSCam node
#=============

set(GSCAM_NODE_SOURCES
  src/gscam_node.cpp)

set(GSCAM_NODE_DEPS
  camera_calibration_parsers
  camera_info_manager
  class_loader
  rclcpp
  rclcpp_components
  ros2_shared
  sensor_msgs)

add_library(gscam_node SHARED
  ${GSCAM_NODE_SOURCES})

target_compile_definitions(gscam_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(gscam_node
  ${GSCAM_NODE_DEPS})

target_link_libraries(gscam_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES})

rclcpp_components_register_nodes(gscam_node "gscam2::GSCamNode")
set(node_plugins "${node_plugins}gscam2::GSCamNode;$<TARGET_FILE:gscam_node>\n")

#=============
# Test subscriber node
#=============

set(SUBSCRIBER_NODE_SOURCES
  src/subscriber_node.cpp)

set(SUBSCRIBER_NODE_DEPS
  class_loader
  rclcpp
  rclcpp_components
  sensor_msgs)

add_library(subscriber_node SHARED
  ${SUBSCRIBER_NODE_SOURCES})

target_compile_definitions(subscriber_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(subscriber_node
  ${SUBSCRIBER_NODE_DEPS})

rclcpp_components_register_nodes(subscriber_node "gscam2::ImageSubscriberNode")
set(node_plugins "${node_plugins}gscam2::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

#=============
# GSCam main, statically linked to make IDE debugging easier
#=============

add_executable(gscam_main
  src/gscam_main.cpp
  ${GSCAM_NODE_SOURCES})

ament_target_dependencies(gscam_main
  ${GSCAM_NODE_DEPS})

target_link_libraries(gscam_main
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES})

#=============
# Manual composition of camera and subscriber nodes, IPC=true
#=============

add_executable(
  ipc_test_main
  src/ipc_test_main.cpp
)
target_link_libraries(
  ipc_test_main
  gscam_node
  subscriber_node
)
ament_target_dependencies(
  ipc_test_main
  rclcpp
)

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_gscam_node export_subscriber_node)

ament_export_libraries(gscam_node subscriber_node)

ament_package()

#=============
# Install
#=============

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS gscam_node
  EXPORT export_gscam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS subscriber_node
  EXPORT export_subscriber_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS gscam_main ipc_test_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY cfg config launch
  DESTINATION share/${PROJECT_NAME}
)

endif ()