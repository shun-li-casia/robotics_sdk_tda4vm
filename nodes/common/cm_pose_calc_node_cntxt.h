/*
 *
 * Copyright (c) 2021 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _CM_POSE_CALC_CNTXT_H
#define _CM_POSE_CALC_CNTXT_H


//#include "tiadalg_interface.h"
#include "../common/tiadalg_common_data_struct.h"
#include "../tiadalg_visual_localization/inc/tiadalg_visual_localization.h"
#include "itidl_ti.h"
#include <TI/tivx_img_proc.h>
#include <cm_common.h>
#include <cm_dlr_node_cntxt.h>



/**
 * \defgroup group_applib_common_posecalc  Common Pose Calculation node
 *                                         set up code
 * \ingroup  group_applib_common
 *
 */

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief Constant for representing the Pose Calc node context invalid state.
 * \ingroup group_applib_common_posecalc
 */
#define CM_POSECALC_NODE_CNTXT_STATE_INVALID     (0U)

/**
 * \brief Constant for representing the Pose Calc node context initialization
 *        state. 
 * \ingroup group_applib_common_posecalc
 */
#define CM_POSECALC_NODE_CNTXT_STATE_INIT        (1U)

/**
 * \brief Constant for representing the Pose Calc node context setup state.
 * \ingroup group_applib_common_posecalc
 */
#define CM_POSECALC_NODE_CNTXT_STATE_SETUP       (2U)


/**
 * \brief Pose cale node create time parameters.
 * \ingroup group_applib_common_posecalc
 */
typedef struct
{
    /** Pipeline depth. */
    uint8_t                 pipelineDepth;

    /** Image width in pixels. */
    uint32_t                outWidth;

    /** Image height in pixels. */
    uint32_t                outHeight;

    /** Input width to CNN */
    int32_t                 dlWidth;

    /** Input height to CNN */
    int32_t                 dlHeight;

    /** Number of voxels */
    int32_t                 numVoxels;

    /** Number of map featurs */
    int32_t                 numMapFeat;

    /** Max number of features accounted in given frame of estimation */
    int32_t                 maxMapFeat;

    /** Max number of features possible in a frame*/
    int32_t                 maxFrameFeat;

    /** NMS score threshold for feature point filtering */
    int32_t                 scoreTh;

    /** Upsampling filter scale in power of 2 */
    int32_t                 filterScalePw2;

    /** Output descriptor (at original resolution) scale in power of 2. 
     *  Has to be sync with MAP descriptor scale 
     */
    int32_t                 hiResDescScalePw2;

    /** pose calculation skip flag */
    int32_t                 skipFlag;

 
    /** Initial estimate for location used in sampling the map data */
    float                   initPoseEst[3];


    /** Input voxel info path */
    char                    inputVoxelInfoPath[CM_MAX_FILE_LEN];

    /** Input map feature point info path */
    char                    inputMapFeatPtPath[CM_MAX_FILE_LEN];

    /** Input map feature descriptor info path */
    char                    inputMapFeatDescPath[CM_MAX_FILE_LEN];

    /** Input lens distortion table path */
    char                    inputLensDistTablePath[CM_MAX_FILE_LEN];

    /** Input up-sampling weight info path */
    char                    inputUpsampleWtPath[CM_MAX_FILE_LEN];

    /** Input up-sampling bias info path */
    char                    inputUpsampleBiasPath[CM_MAX_FILE_LEN];

} CM_PoseCalcCreateParams;

/**
 * \brief   Pose Calculation node context
 * \ingroup group_applib_common_posecalc
 */
typedef struct 
{
    /** State variable. */
    uint32_t                     state;

    /** Pipeline depth. */
    uint8_t                      pipelineDepth;

    vx_node                      node;
    vx_array                     config;
   
    // Output
    vx_matrix                  * poseMatrix;

    // Input
    vx_tensor                    voxelInfo;
    vx_tensor                    mapFeat;
    vx_tensor                    mapDesc;
    vx_tensor                    wtTable;
    vx_tensor                    lensTable;
    vx_user_data_object          outArgs;
    
    vx_int32                     graph_parameter_index;

    const char                  *inputUpsampleWtPath;
    const char                  *inputUpsampleBiasPath;
    const char                  *inputVoxelInfoPath;
    const char                  *inputMapFeatPtPath;
    const char                  *inputMapFeatDescPath;
    const char                  *inputLensDistTablePath;

    tivxVisualLocalizationParams vlParams;

} CM_PoseCalcNodeCntxt;

 
/**
 * \brief Function to initialize the Pose Calculation node.
 *
 * \param [in,out] poseCalcObj Pose Calculation node context.
 * 
 * \param [in] dlrObj DLR node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 * 
 * \param [in] createParams Pose Calculation node context create parameters.
 * 
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_posecalc
 */
vx_status CM_poseCalcNodeCntxtInit(CM_PoseCalcNodeCntxt           *poseCalcObj,
                                   CM_DLRNodeCntxt                *dlrObj,
                                   vx_context                      context,
                                   const CM_PoseCalcCreateParams  *createParams);

/**
 * \brief Function to create and setup the Pose Calculation node.
 *
 * \param [in,out] poseCalcObj Pose Calculation node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 * 
 * \param [in] graph The handle to the graph this node belongs to.
 * 
 * \param [in] curFeatTensor Feature score tensor.
 * 
 * \param [in] curDescTensor Feature decsriptor tensor.
 * 
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_posecalc
 */
vx_status CM_poseCalcNodeCntxtSetup(CM_PoseCalcNodeCntxt     *poseCalcObj,
                                    vx_context                context,
                                    vx_graph                  graph,
                                    vx_tensor                 curFeatTensor,
                                    vx_tensor                 curDescTensor);

/**
 * \brief Function to de-inmitialize the Pose Calculation node.
 *
 * \param [in,out] poseCalcObj Pose Calculation node context.
 * 
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_posecalc
 */
vx_status CM_poseCalcNodeCntxtDeInit(CM_PoseCalcNodeCntxt *poseCalcObj);

#ifdef __cplusplus
}
#endif

#endif  //_CM_POSE_CALC_CNTXT_H
