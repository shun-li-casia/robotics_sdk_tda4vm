cmake_minimum_required(VERSION 3.0.2)
project(ti_semseg_cnn)

# Build only on J7
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64") 
    include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/common.cmake)

    # CNN based Semantic Segmentation example
    set(APP_SEMSEG_CNN_SRCS
        src/semseg_cnn_main.cpp
        src/semseg_cnn_node.cpp
        src/semseg_cnn.cpp)

    # Build nodes
    build_node(semseg_cnn APP_SEMSEG_CNN_SRCS)
endif()