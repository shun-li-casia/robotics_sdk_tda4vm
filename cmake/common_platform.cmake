# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

add_compile_options(-std=c++17)

SET(CMAKE_FIND_LIBRARY_PREFIXES "" "lib")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ".lib" ".so")

# Get the path of this file
get_filename_component(SELF_DIR        "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(TI_ROS_ROOT_DIR "${SELF_DIR}/.." ABSOLUTE)

# From the self path, derive and set the ti_core directory
get_filename_component(TI_CORE_ROOT_DIR   "${TI_ROS_ROOT_DIR}/ti_core" ABSOLUTE)
get_filename_component(TI_EDGEAI_ROOT_DIR "/opt/edge_ai_apps/apps_cpp" ABSOLUTE)
get_filename_component(INSTALL_DIR        ${CMAKE_INSTALL_PREFIX}/.. ABSOLUTE)

# PSDKRA base folder location
set(PSDK_DIR $ENV{PSDK_BASE_PATH})

# Set PROFLE based on CMAKE_BUILD_TYPE which is passed from command line,
# e.g., -DCMAKE_BUILD_TYPE=Debug
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if ((${CMAKE_BUILD_TYPE} STREQUAL Debug) OR
    (${CMAKE_BUILD_TYPE} STREQUAL RelWithDebInfo))
    set(PROFILE debug)
else()
    set(PROFILE release)
endif()

message(STATUS "PROFILE = ${PROFILE}")
unset(CMAKE_BUILD_TYPE CACHE)

set(TI_EXTERNAL_INCLUDE_DIRS
    ${TI_CORE_ROOT_DIR}/common/include
    ${TI_CORE_ROOT_DIR}/utils/include
    ${TI_CORE_ROOT_DIR}/ti_vision_cnn/include
    ${TI_CORE_ROOT_DIR}/ti_sde/include
    ${TI_CORE_ROOT_DIR}/ti_estop/include
    ${TI_CORE_ROOT_DIR}/ti_vl/include
    ${TI_EDGEAI_ROOT_DIR}
   )

if (NOT "$ENV{MMALIB_PACKAGE_ROOT}" STREQUAL "")
    set(MMALIB_PACKAGE_ROOT "$ENV{MMALIB_PACKAGE_ROOT}"
        CACHE INTERNAL
        "Copied MMALIB_PACKAGE_ROOT from environment variable")
else()
    set(MMALIB_PACKAGE_ROOT mmalib_02_01_00_07)
endif()

if (NOT "$ENV{TIDL_PACKAGE_ROOT}" STREQUAL "")
    set(TIDL_PACKAGE_ROOT "$ENV{TIDL_PACKAGE_ROOT}/ti_dl"
        CACHE INTERNAL
        "Copied TIDL_PACKAGE_ROOT from environment variable")
else()
    # tidl_j7 -> tidl_j7_<version> should be extablished in the root filesystem
    set(TIDL_PACKAGE_ROOT tidl_j7/ti_dl)
endif()

set(CGT7X_ROOT          ti-cgt-c7000_1.4.2.LTS)
set(PDK_PACKAGE_ROOT    pdk/packages/ti)
set(TIADALG_PATH        tiadalg)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    set(BUILD_CORE_NODES          OFF)
    set(BUILD_VISUALIZATION_NODES ON CACHE BOOL "Build Visualization nodes")
    set(TARGET_PLATFORM           PC)
    set(TARGET_CPU                x86_64)
    set(BUILD_EMULATION_MODE      yes)
    set(TIVISION_APPS_TYPE        static)
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(BUILD_CORE_NODES          ON CACHE BOOL "Build core nodes")
    set(BUILD_VISUALIZATION_NODES OFF CACHE BOOL "Do not build Visualization nodes")
    set(TARGET_PLATFORM           J7)
    set(TARGET_CPU                A72)
    set(BUILD_EMULATION_MODE      no)
    set(TIVISION_APPS_TYPE        shared)
else()
    message(FATAL_ERROR "Unknown processor:" ${CMAKE_SYSTEM_PROCESSOR})
endif()

if (${BUILD_CORE_NODES})
    set(TI_EXTERNAL_LIBS
        ti_core
        ti_dl_inferer
        edgeai_utils
        ncurses
        dlr
        tensorflow-lite
        onnxruntime
        dl
        yaml-cpp
       )
else()
    set(TI_EXTERNAL_LIBS "")
endif()

set(TARGET_OS LINUX)

# pass the macros
add_definitions(
    -DTARGET_CPU=${TARGET_CPU}
    -DTARGET_OS=${TARGET_OS}
    -DPROFILE=${PROFILE}
    -DBUILD_EMULATION_MODE=${BUILD_EMULATION_MODE}
    -DSOC_J721E
)

# TIOVX, VISION_APPS, PTK_DEMOS: include folders
set(TIOVX_INCLUDE_DIRS
    ${PSDK_DIR}/tiovx/include
    ${PSDK_DIR}/tiovx/kernels_j7/include
    ${PSDK_DIR}/tiovx/tiovx_dev/kernels_j7/include
    ${PSDK_DIR}/tiovx/utils/include/
    ${PSDK_DIR}/tiovx/kernels/include/
)

set(PTK_INCLUDE_DIRS
    ${PSDK_DIR}/perception/include
)

set(MMALIB_INCLUDE_DIRS
    ${PSDK_DIR}/${MMALIB_PACKAGE_ROOT}/include
)

set(TIDL_INCLUDE_DIRS
    ${PSDK_DIR}/${TIDL_PACKAGE_ROOT}/inc
)

set(TIADALG_INCLUDE_DIRS
    ${PSDK_DIR}/${TIADALG_PATH}/include
)

set(VISION_APP_INCLUDE_DIRS
    ${PSDK_DIR}/vision_apps
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/app_common
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/sde_ldc_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/sde_multilayer_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/sde_singlelayer_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/sde_triangulate_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/sde_obstacle_detection_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/ss_sde_detection_applib
    ${PSDK_DIR}/vision_apps/kernels/img_proc/include
    ${PSDK_DIR}/vision_apps/kernels/img_proc/host
    ${PSDK_DIR}/vision_apps/kernels/stereo/include
    src
)

# should add more directories in ${PSDK_DIR}/vision_apps/kernels
set(VISION_APP_KERNEL_INCLUDE_DIRS
    ${PSDK_DIR}/vision_apps/kernels/img_proc/include
    ${PSDK_DIR}/vision_apps/kernels/stereo/include
)

set(OTHER_INCLUDE_DIRS
    ${PSDK_DIR}/${CGT7X_ROOT}/host_emulation/include/C7100
    ${PSDK_DIR}/j7_c_models/include
    ${PSDK_DIR}/ivision
)

include_directories(
    ${TI_EXTERNAL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${TIOVX_INCLUDE_DIRS}
    ${PTK_INCLUDE_DIRS}
    ${VISION_APP_INCLUDE_DIRS}
    ${VISION_APP_KERNEL_INCLUDE_DIRS}
    ${MMALIB_INCLUDE_DIRS}
    ${TIDL_INCLUDE_DIRS}
    ${TIADALG_INCLUDE_DIRS}
    ${OTHER_INCLUDE_DIRS}
    /usr/include/opencv4
)

set(COMMON_LINK_LIBS
    ${TI_EXTERNAL_LIBS}
    )

set(TI_EXTERNAL_LIB_DIRS
    ${INSTALL_DIR}/ti_external/lib
   )

# BUILD and LINK
if (${TARGET_PLATFORM} STREQUAL J7)
    set(TARGET_LINK_DIRECTORIES
        ${TI_EXTERNAL_LIB_DIRS}
        /usr/local/dlr
        /usr/lib
        )

    set(TARGET_LINK_LIBS
        tivision_apps dlr)
endif()

link_directories(${TARGET_LINK_DIRECTORIES})

function(print_vars)
    message("CMAKE_SYSTEM_PROCESSOR   =" ${CMAKE_SYSTEM_PROCESSOR})
    message("PROJECT_SOURCE_DIR       =" ${PROJECT_SOURCE_DIR})
    message("CMAKE_SOURCE_DIR         =" ${CMAKE_SOURCE_DIR})
    message("CMAKE_CURRENT_SOURCE_DIR =" ${CMAKE_CURRENT_SOURCE_DIR})
    message("CMAKE_INSTALL_PREFIX     =" ${CMAKE_INSTALL_PREFIX})
    message("CMAKE_CURRENT_BINARY_DIR =" ${CMAKE_CURRENT_BINARY_DIR})
    message("INSTALL_DIR              =" ${INSTALL_DIR})
    message("MMALIB_PACKAGE_ROOT      =" ${MMALIB_PACKAGE_ROOT})
    message("TIDL_PACKAGE_ROOT        =" ${TIDL_PACKAGE_ROOT})
    message("TI_CORE_ROOT_DIR         =" ${TI_CORE_ROOT_DIR})
    message("TI_EDGEAI_ROOT_DIR       =" ${TI_EDGEAI_ROOT_DIR})

endfunction(print_vars)

