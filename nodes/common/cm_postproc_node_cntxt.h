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
 * Redistributions must postserve existing copyright notices and reproduce this license
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
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPOSTSS
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
#if !defined(_CM_POSTPROC_NODE_CNTXT_H_)
#define _CM_POSTPROC_NODE_CNTXT_H_

#include <TI/tivx_img_proc.h>
#include <cm_common.h>

/**
 * \defgroup group_applib_common_postproc Common Post-processing node
 *                                        setup code.
 * \ingroup group_applib_common
 *
 */

typedef struct
{
    uint8_t y;
    uint8_t u;
    uint8_t v;

} SemSegColor;

SemSegColor yuvColMap[] =
{
    //       Y    U    V         R    G    B
    [ 0] = { 94, 147, 152}, // {128,  64, 128}
    [ 1] = {119, 183, 206}, // {244,  35, 232}
    [ 2] = { 76, 128, 128}, // { 70,  70,  70}
    [ 3] = {109, 152, 124}, // {102, 102, 156}
    [ 4] = {157, 123, 144}, // {190, 153, 153}
    [ 5] = {147, 128, 128}, // {153, 153, 153}
    [ 6] = {169,  55, 173}, // {250, 170,  30}
    [ 7] = {184,  32, 143}, // {220, 220,   0}
    [ 8] = {119,  86, 120}, // {107, 142,  35}
    [ 9] = {197,  99,  92}, // {152, 251, 152}
    [10] = {117, 159,  98}, // { 70, 130, 180}
    [11] = { 89, 116, 213}, // {220,  20,  60}
    [12] = { 82,  90, 240}, // {255,   0,   0}
    [13] = { 30, 190, 118}, // {  0,   0, 142}
    [14] = { 23, 159, 123}, // {  0,   0,  70}
    [15] = { 56, 154,  99}, // {  0,  60, 100}
    [16] = { 66, 149,  92}, // {  0,  80, 100}
    [17] = { 38, 229, 112}, // {  0,   0, 230}
    [18] = { 55, 121, 174}, // {119,  11,  32}
    [19] = {126, 128, 128}  // {128, 128, 128}
}; // yuvColMap


#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Constant for repostsenting the postproc node context invalid state.
 * \ingroup group_applib_common_postproc
 */
#define CM_POSTPROC_NODE_CNTXT_STATE_INVALID    (0U)

/**
 * \brief Constant for repostsenting the postproc node context initialization
 *        state.
 * \ingroup group_applib_common_postproc
 */
#define CM_POSTPROC_NODE_CNTXT_STATE_INIT       (1U)

/**
 * \brief Constant for repostsenting the postproc node context setup state.
 * \ingroup group_applib_common_postproc
 */
#define CM_POSTPROC_NODE_CNTXT_STATE_SETUP      (2U)

/**
 * \brief Post-processing node create time parameters.
 *
 * \ingroup group_applib_common_postproc
 */
typedef struct
{
    /** Output Image width in pixels. */
    uint32_t                outWidth;

    /** Output Image height in pixels. */
    uint32_t                outHeight;

    /** Number of detection classes. */
    uint8_t                 numClasses;

    /** Pipeline depth. */
    uint8_t                 pipelineDepth;

    /** Handle to the user object with TIDL configuration data. */
    vx_user_data_object     tidlCfg;

} CM_PostProcCreateParams;

/**
 * \brief Post-processing node context.
 *
 * \ingroup group_applib_common_postproc
 */
typedef struct
{
    /** State variable. */
    uint32_t                state;

    /** Handle to the pre-processing node. */
    vx_node                 vxNode;

    /** Handle to the post-processing kernel. */
    vx_kernel               vxVizKernel;

    /** Handle to the vx object for holding the post-processing configuration
     *  parameters.
     */
    vx_user_data_object     vxVizConfig;

    /** Post-processing configuration parameters. */
    tivxPixelVizParams      vxVizParams;

    /** Output Image width in pixels. */
    vx_uint32               outWidth;

    /** Output Image height in pixels. */
    vx_uint32               outHeight;

    /** Number of output tensors. */
    uint8_t                 numOutTensors;

    /** Pipeline depth. */
    uint8_t                 pipelineDepth;

    /** Start graph parameter index. */
    uint8_t                 outImageBaseParamIdx;

    /** Size of the memory allocated for holding a 2d-array of output images. 
     *  This is saved since the size is needed when releasing the memory
     *  during the de-init time.
     */
    uint32_t                vxOutImageMemSize;

    /** Size of the memory allocated for holding a 2d-array of output images. 
     *  This is saved since the size is needed when releasing the memory
     *  during the de-init time.
     */
    uint32_t                vxOutImageRefMemSize;

    /** 
     * A 2d-array of output images.
     * vxOutputImage[pipelineDepth][numOutTensors];
     */
    vx_image              **vxOutputImage;

    /** 
     * A 2d-array of output images.
     * vxOutputImageRefs[numOutTensors][pipelineDepth];
     *
     * Need to give the list of buffers for each output so need this structure.
     * This array just points to the objects in vxOutputImage.
     */
    vx_image              **vxOutputImageRefs;

} CM_PostProcNodeCntxt;

/**
 * \brief Function to initialize the post-processing node context.
 *
 * \param [in,out] postProcObj Post-processing node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] createParams Post-processing node context create parameters.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_postproc
 */
vx_status CM_postProcNodeCntxtInit(
        CM_PostProcNodeCntxt           *postProcObj,
        vx_context                      context,
        const CM_PostProcCreateParams  *createParams);

/**
 * \brief Function to create and setup the post-processing node.
 *        
 *        The node should have been initialized by calling
 *        CM_postProcNodeCntxtInit() API, prior to invoking this API.
 *
 * \param [in,out] postProcObj Post-processing node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] graph The handle to the graph this node belongs to.
 *
 * \param [in] tidlOutArgs Output arguments from the TIDL node.
 *
 * \param [in] inputImage Input image parameter to the node.
 *
 * \param [in] detection Array of tensors. This is the output from the TIDL
 *                       node.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_postproc
 */
vx_status CM_postProcNodeCntxtSetup(
        CM_PostProcNodeCntxt   *postProcObj,
        vx_context              context,
        vx_graph                graph,
        vx_user_data_object     tidlOutArgs,
        vx_image                inputImage,
        vx_tensor              *detection);

/**
 * \brief Function to de-initialize the post-processing node context.
 *
 * \param [in,out] postProcObj Post-processing node context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_postproc
 */
vx_status CM_postProcNodeCntxtDeInit(
        CM_PostProcNodeCntxt   *postProcObj);

#ifdef __cplusplus
}
#endif

#endif /* _CM_POSTPROC_NODE_CNTXT_H_ */

