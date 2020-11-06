add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp cv_bridge image_transport sensor_msgs std_msgs message_generation common_msgs)

# for custom ROS messages
if (DEPENDENT_PKG_LIST)
    generate_messages(DEPENDENCIES ${DEPENDENT_PKG_LIST})
endif()

catkin_package(CATKIN_DEPENDS nodelet roscpp cv_bridge image_transport sensor_msgs message_runtime)

find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# PSDKRA base folder location
set(PSDK_DIR $ENV{PSDK_BASE_PATH})

set(CGT7X_ROOT          ti-cgt-c7000_1.4.0.LTS)
set(PDK_PACKAGE_ROOT    pdk/packages/ti)
set(TIDL_PACKAGE_ROOT   tidl_j7_01_03_00_07/ti_dl)
set(MMALIB_PACKAGE_ROOT mmalib_01_03_00_06)
set(TIADALG_PATH        tiadalg)

SET(CMAKE_FIND_LIBRARY_PREFIXES "" "lib")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ".lib" ".so")

# tivision_apps library: build agaist static or shared library
# tiovx_apps shared lib currently available only on J7
## TI OpenVX
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    set(TARGET_PLATFORM      PC)
    set(TARGET_CPU           x86_64)
    set(BUILD_EMULATION_MODE yes)
    set(TIVISION_APPS_TYPE   static)
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(TARGET_PLATFORM      J7)
    set(TARGET_CPU           A72)
    set(BUILD_EMULATION_MODE no)
    set(TIVISION_APPS_TYPE   shared)
else()
    message(FATAL_ERROR "Unknown processor:" ${CMAKE_SYSTEM_PROCESSOR})
endif()

set(TARGET_OS LINUX)

# Set PROFLE based on CMAKE_BUILD_TYPE which is passed from command line,
# e.g., catkin_make -DCMAKE_BUILD_TYPE=Debug
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if ((${CMAKE_BUILD_TYPE} STREQUAL Debug) OR (${CMAKE_BUILD_TYPE} STREQUAL RelWithDebInfo))
    set(PROFILE debug)
else()
    set(PROFILE release)
endif()

message(STATUS "PROFILE = ${PROFILE}")
unset(CMAKE_BUILD_TYPE CACHE)

# pass the macros
if(${TARGET_PLATFORM} STREQUAL "PC")
    add_definitions(-DTARGET_CPU=${TARGET_CPU}
                    -DTARGET_OS=${TARGET_OS}
                    -DPROFILE=${PROFILE}
                    -DBUILD_EMULATION_MODE=${BUILD_EMULATION_MODE}
                    -DPC
                    # -DTARGET_CPU_PC=true
    )
else()  # J7
    add_definitions(-DTARGET_CPU=${TARGET_CPU}
                    -DTARGET_OS=${TARGET_OS}
                    -DPROFILE=${PROFILE}
                    -DBUILD_EMULATION_MODE=${BUILD_EMULATION_MODE}
    )
endif()

# TIOVX, VISION_APPS, PTK_DEMOS: include folders
set(TIOVX_INCLUDE_DIRS
    ${PSDK_DIR}/tiovx/include
    ${PSDK_DIR}/tiovx/tiovx_dev/kernels_j7/include
    ${PSDK_DIR}/tiovx/utils/include/
    ${PSDK_DIR}/tiovx/kernels/include/
)

# set(PDK_INCLUDE_DIRS
#     ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/drv/udma
# )

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
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/sde_obstacle_detection_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/semseg_cnn_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/ss_sde_detection_applib
    ${PSDK_DIR}/vision_apps/apps/ptk_demos/applibs/applib_common
    ${PSDK_DIR}/vision_apps/kernels/img_proc/include
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

include_directories(${TIOVX_INCLUDE_DIRS}
                    ${PTK_INCLUDE_DIRS}
                    ${VISION_APP_INCLUDE_DIRS}
                    ${VISION_APP_KERNEL_INCLUDE_DIRS}
                    ${MMALIB_INCLUDE_DIRS}
                    ${TIDL_INCLUDE_DIRS}
                    ${TIADALG_INCLUDE_DIRS}
                    ${OTHER_INCLUDE_DIRS}
)

# TIOVX, VISION_APPS, PTK_DEMOS: define lib variables
if(${TARGET_PLATFORM} STREQUAL "PC")
    include(${CMAKE_CURRENT_LIST_DIR}/platform_${TARGET_PLATFORM}.cmake)
endif()

# Function for building a node:
# ARG0: app name
# ARG1: source list
function(build_node)
    set(app ${ARGV0})
    set(src ${ARGV1})
    add_executable(${app} ${${src}})
    target_link_libraries(${app} ${TARGET_LINK_LIBS})
    add_dependencies(${app} ${common_msgs_EXPORTED_TARGETS})
endfunction()

# Function for building a nodelet:
# ARG0: lib name
# ARG1: source list
function(build_nodelet)
    set(lib ${ARGV0})
    set(src ${ARGV1})
    add_library(${lib} ${${src}})
    target_link_libraries(${lib} ${TARGET_LINK_LIBS})
endfunction()

# BUILD and LINK
if ((${TIVISION_APPS_TYPE} STREQUAL shared) AND (${TARGET_PLATFORM} STREQUAL J7))
    set(TARGET_LINK_LIBS
        ${catkin_LIBRARIES}
        tivision_apps gbm)
else() # static
    set(TARGET_LINK_LIBS
        ${catkin_LIBRARIES}
        -Wl,--start-group
        ${VISION_APPS_LIBS}
        ${PDK_LIBS}
        ${TIOVX_LIBS}
        ${PTK_LIBS}
        ${J7_CMODEL_LIBS}
        ${IMAGING_LIBS}
        ${CGT7X_LIBS}
        ${TIDL_LIBS}
        ${MMA_LIBS}
        ${TIADALG_LIBS}
        ${PTHREAD_LIB}
        ${ION_LIB}         # only for J7
        ${CMAKE_DL_LIBS}
        -Wl,--end-group)

    link_directories(${TIOVX_LIBS_DIR}
                     ${VISION_APPS_LIBS_DIR}
                     ${PDK_LIBS_DIR}
                     ${PTK_LIBS_DIR}
                     ${IMAGING_LIBS_DIR}
                     ${CGT7X_LIBS_DIR}
                     ${TIDL_LIBS_DIR}
                     ${MMA_LIBS_DIR}
                     ${TIADALG_LIBS_DIR}
                     ${J7_CMODEL_LIBS_DIR}
                     ${PTHREAD_LIB_DIR}
                     ${ION_LIB_DIR}  # only for J7
                    )
endif()

