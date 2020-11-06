cmake_minimum_required(VERSION 3.0.2)
project(ti_semseg_cnn)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

# CNN based Semantic Segmentation example
set(APP_SEMSEG_CNN_SRCS
    src/semseg_cnn_main.cpp
    src/semseg_cnn_node.cpp
    src/semseg_cnn.cpp)

# Build nodes
build_node(semseg_cnn APP_SEMSEG_CNN_SRCS)
