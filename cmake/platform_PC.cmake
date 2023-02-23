## TI OpenVX
# ${PSDK_DIR}/tiovx/lib/PC/x86_64/LINUX/release
set(TIOVX_LIBS_DIR
    ${PSDK_DIR}/tiovx/lib/${TARGET_PLATFORM}/${TARGET_CPU}/${TARGET_OS}/${PROFILE}
)
set(TIOVX_LIBS
    algframework_x86_64
    c6xsim_x86_64_C66
    dmautils_x86_64
    vx_conformance_engine
    vx_conformance_tests
    vx_conformance_tests_testmodule
    vx_framework
    vx_kernels_host_utils
    vx_kernels_hwa
    vx_kernels_hwa_tests
    vx_kernels_openvx_core
    vx_kernels_target_utils
    vx_kernels_test_kernels
    vx_kernels_test_kernels_tests
    vx_kernels_tidl
    vxlib_bamplugin_x86_64
    vxlib_x86_64
    vx_platform_pc
    vx_sample_usecases
    vx_target_kernels_c66
    vx_target_kernels_dmpac_dof
    vx_target_kernels_dmpac_sde
    vx_target_kernels_ivision_common
    vx_target_kernels_j7_arm
    vx_target_kernels_openvx_core
    vx_target_kernels_source_sink
    vx_target_kernels_tidl
    vx_target_kernels_tutorial
    vx_target_kernels_vpac_ldc
    vx_target_kernels_vpac_msc
    vx_target_kernels_vpac_nf
    vx_target_kernels_vpac_viss
    vx_tiovx_tests
    vx_tiovx_tidl_tests
    vx_tutorial
    vx_utils
    vx_vxu
)

## pthread lib
set(PTHREAD_LIB_DIR "/lib/x86_64-linux-gnu") # PC
set(PTHREAD_LIB pthread)

## TI vision_apps
# ${PSDK_DIR}/vision_apps/lib/PC/x86_64/LINUX/release
set(VISION_APPS_LIBS_DIR
    ${PSDK_DIR}/vision_apps/lib/${TARGET_PLATFORM}/${TARGET_CPU}/${TARGET_OS}/${PROFILE}
)
set(VISION_APPS_LIBS
    app_utils_console_io
    app_utils_draw2d
    app_utils_grpx
    app_utils_iss
    app_utils_mem
    app_utils_opengl
    app_utils_perf_stats
    vx_app_c7x_target_kernel
    vx_applib_fused_ogmap
    vx_applib_lidar_ogmap
    vx_applib_ps_mapping
    vx_applib_radar_gtrack
    vx_applib_sde_ldc
    vx_applib_sde_obstacle_detection
    vx_applib_sde_singlelayer
    vx_applib_sde_multilayer
    vx_applib_sde_triangulate
    vx_applib_srv_bowl_lut_gen
    vx_applib_srv_calibration
    vx_applib_surround_radar_ogmap
    vx_applib_surround_sfm_ogmap
    vx_applib_ss_sde_detection
    vx_applib_tests
    vx_kernels_common
    vx_kernels_fileio
    vx_kernels_img_proc
    vx_target_kernels_img_proc_a72
    vx_kernels_lidar
    vx_kernels_park_assist
    vx_kernels_sample
    vx_kernels_srv
    vx_kernels_srv_tests
    vx_kernels_stereo
    vx_kernels_stereo_tests
    vx_srv_render_utils
    vx_srv_render_utils_tools
    vx_target_kernels_fileio
    vx_target_kernels_img_proc_c66
    vx_target_kernels_lidar_arm
    vx_target_kernels_park_assist
    vx_target_kernels_sample_a72
    vx_target_kernels_srv_c66
    vx_target_kernels_srv_gpu
    vx_target_kernels_stereo
)

## PDK_LIB
set(PDK_LIBS_DIR
    ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/drv/udma/lib/j721e_hostemu/c7x-hostemu/${PROFILE}
)

set(PDK_LIBS
    ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/drv/udma/lib/j721e_hostemu/c7x-hostemu/${PROFILE}/dmautils.lib
    ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/drv/udma/lib/j721e_hostemu/c7x-hostemu/${PROFILE}/udma.lib
    ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/drv/sciclient/lib/j721e_hostemu/c7x-hostemu/${PROFILE}/sciclient.lib
    ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/csl/lib/j721e/c7x-hostemu/${PROFILE}/ti.csl.lib
    ${PSDK_DIR}/${PDK_PACKAGE_ROOT}/osal/lib/nonos/j721e/c7x-hostemu/${PROFILE}/ti.osal.lib
)

## PTK_LIB
# ${PSDK_DIR}/perception/lib/PC/x86_64/LINUX/release
set(PTK_LIBS_DIR
    ${PSDK_DIR}/perception/lib/${TARGET_PLATFORM}/${TARGET_CPU}/${TARGET_OS}/${PROFILE}
)
set(PTK_LIBS
    ptk_algos
    ptk_base
    ptk_dbtools
    ptk_drv
    ptk_gui
    ptk_net
    ptk_utils
)

## Imaging LIB
set(IMAGING_LIBS_DIR
    ${PSDK_DIR}/imaging/lib/${TARGET_PLATFORM}/${TARGET_CPU}/${TARGET_OS}/${PROFILE}
)
set(IMAGING_LIBS
    ti_imaging_aealg
    ti_imaging_awbalg
    ti_imaging_dcc
    vx_kernels_imaging
    vx_target_kernels_imaging_aewb
)

## CGT7X
set(CGT7X_LIBS_DIR
    ${PSDK_DIR}/${CGT7X_ROOT}//host_emulation
)

set(CGT7X_LIBS
    C7100-host-emulation
)

## TIDL 
set(TIDL_LIBS_DIR
    ${PSDK_DIR}/${TIDL_PACKAGE_ROOT}/lib/PC/dsp/algo/release
)

set(TIDL_LIBS
    tidl_algo
    tidl_priv_algo
    tidl_obj_algo
    tidl_custom
)

## MMALIB
set(MMA_LIBS_DIR
    ${PSDK_DIR}/${MMALIB_PACKAGE_ROOT}/lib/${PROFILE}
)

set(MMA_LIBS
    common_x86_64
    mmalib_cn_x86_64
    mmalib_x86_64
)

## TIADALG
set(TIADALG_LIBS_DIR
    ${PSDK_DIR}/${TIADALG_PATH}/lib/${TARGET_CPU}/${PROFILE}
)

set(TIADALG_LIBS
    tiadalg_dof_plane_seperation
    tiadalg_fisheye_transformation
    tiadalg_image_color_blending
    tiadalg_image_preprocessing
    tiadalg_select_top_feature
    tiadalg_solve_pnp
    tiadalg_sparse_upsampling
    tiadalg_visual_localization
    tiadalg_image_recursive_nms
    c6xsim
)

## J7 C Model LIB
#==> When ${PROFILE} = Debug: glbce is missing => compile error
# Temporary solution: link "release" lib always
set(J7_CMODEL_LIBS_DIR
    # ${PSDK_DIR}/j7_c_models/lib/${TARGET_PLATFORM}/${TARGET_CPU}/${TARGET_OS}/${PROFILE}
    ${PSDK_DIR}/j7_c_models/lib/${TARGET_PLATFORM}/${TARGET_CPU}/${TARGET_OS}/release
)
set(J7_CMODEL_LIBS
    bl_filter_lib
    #libDOF.so  # libDOF.so causes crash in cv_bridge functions
    ee
    flexcc
    flexcfa
    glbce.a
    glbce.so
    h3a
    ldc
    nsf4
    rawfe
    scalar
    sde_hw
    utils
)

set(TARGET_LINK_LIBS
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
    ${ION_LIB}
)

set(TARGET_LINK_DIRECTORIES
    ${TIOVX_LIBS_DIR}
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

add_definitions(
    -DPC
)
message(STATUS "=====> platform_PC.txt finished")
