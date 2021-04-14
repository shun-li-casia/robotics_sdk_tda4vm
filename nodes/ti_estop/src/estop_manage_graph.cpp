/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
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
 * licensed and provided to you in appCntxtect code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any appCntxtect code compiled from the source code
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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>


#include <app_ptk_demo_common.h>
#include <cm_common.h>
#include <cm_postproc_node_cntxt.h>

#include "estop.h"

#define ESTOP_APP_MAX_OUT_TENSOR_DIMS   (3U)
#define ESTOP_APP_NUM_GRAPH_PARAMS      (7U)


vx_status ESTOP_APP_init_LDC(ESTOP_APP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams;
    int32_t                     i;
    vx_status                   vxStatus = VX_SUCCESS;

    createParams            = &appCntxt->sdeLdcCreateParams;
    createParams->vxContext = appCntxt->vxContext;
    createParams->vxGraph   = appCntxt->vxGraph;

    /*
     * Create input image objects 
     */
    createParams->createInputFlag      = 0;
    createParams->inputPipelineDepth   = appCntxt->pipelineDepth;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // input left image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxInputLeftImage[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxInputLeftImage[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_UYVY);
        }

        if (appCntxt->vxInputLeftImage[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxInputLeftImage[i], "InputLeftImage");

        // input right image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxInputRightImage[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxInputRightImage[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_UYVY);
        }

        if (appCntxt->vxInputRightImage[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxInputRightImage[i], "InputRightImage");

        // pass to LDC Applib createParams
        createParams->vxInputLeftImage[i]  = appCntxt->vxInputLeftImage[i];
        createParams->vxInputRightImage[i] = appCntxt->vxInputRightImage[i];
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        /*
         * Create output image objects 
         */
        createParams->createOutputFlag      = 0;
        createParams->outputPipelineDepth   = 1;

        // output left image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftRectImage = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxLeftRectImage = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
        }

        if (appCntxt->vxLeftRectImage == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxLeftRectImage, "LeftRectifiedImage");

        // pass to LDC Applib createParams
        createParams->vxOutputLeftImage[0]  = appCntxt->vxLeftRectImage;

        // output right image
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
            {
                appCntxt->vxRightRectImage[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
            } else 
            {
                appCntxt->vxRightRectImage[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
            }

            if (appCntxt->vxRightRectImage[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);
                vxStatus = VX_FAILURE;
                break;
            }
            vxSetReferenceName((vx_reference)appCntxt->vxRightRectImage[i], "RightRectifiedImage");

            // pass to LDC Applib createParams
            createParams->vxOutputRightImage[i] = appCntxt->vxRightRectImage[i];
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->sdeLdcHdl = SDELDCAPPLIB_create(createParams);
        if (appCntxt->sdeLdcHdl == NULL)
        {
            PTK_printf("[%s:%d] SDELDCAPPLIB_create() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;;
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_init_SDE(ESTOP_APP_Context *appCntxt)
{
    int32_t i;
    vx_status vxStatus = VX_SUCCESS;

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_createParams   * slSdeCreateParams;

        slSdeCreateParams            = &appCntxt->slSdeCreateParams;
        slSdeCreateParams->vxContext = appCntxt->vxContext;
        slSdeCreateParams->vxGraph   = appCntxt->vxGraph;

        /* 
         * Create input image objects for SL SDE
         */
        slSdeCreateParams->createInputFlag     = 0;
        slSdeCreateParams->inputPipelineDepth  = 1;
        slSdeCreateParams->vxLeftRectImage[0]  = appCntxt->vxLeftRectImage;
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            slSdeCreateParams->vxRightRectImage[i] = appCntxt->vxRightRectImage[i];
        }

        /* 
         * Create output image objects for SL SDE
         */
        slSdeCreateParams->createOutputFlag    = 0;
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxSde16BitOutput[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
            if (appCntxt->vxSde16BitOutput[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);
                vxStatus = VX_FAILURE;
                break;
            }
            vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutput[i], "RawDisparityMap");

            slSdeCreateParams->vxSde16BitOutput[i]  = appCntxt->vxSde16BitOutput[i];
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appCntxt->slSdeHdl = SL_SDEAPPLIB_create(slSdeCreateParams);
            if (appCntxt->slSdeHdl == NULL)
            {
                PTK_printf("[%s:%d] SL_SDEAPPLIB_create() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;;
            }
        }
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_createParams   * mlSdeCreateParams;

        mlSdeCreateParams            = &appCntxt->mlSdeCreateParams;
        mlSdeCreateParams->vxContext = appCntxt->vxContext;
        mlSdeCreateParams->vxGraph   = appCntxt->vxGraph;

        /* 
         * Create input image objects for SL SDE
         */
        mlSdeCreateParams->createInputFlag    = 0;
        mlSdeCreateParams->inputPipelineDepth = 1;
        mlSdeCreateParams->vxLeftRectImageL0[0]  = appCntxt->vxLeftRectImage;
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            mlSdeCreateParams->vxRightRectImageL0[i] = appCntxt->vxRightRectImage[i];
        }

        /* 
         * Create output image objects for ML SDE
         */
        mlSdeCreateParams->createOutputFlag  = 0;
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                appCntxt->vxMedianFilteredDisparity[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
                if (appCntxt->vxMedianFilteredDisparity[i] == NULL)
                {
                    PTK_printf("[%s:%d] vxCreateImage() failed\n",
                                __FUNCTION__, __LINE__);
                    vxStatus = VX_FAILURE;
                    break;
                }
                mlSdeCreateParams->vxMedianFilteredDisparity[i]  = appCntxt->vxMedianFilteredDisparity[i];
            }
        }

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxMergeDisparityL0[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
            if (appCntxt->vxMergeDisparityL0[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);
                vxStatus = VX_FAILURE;;
                break;
            }
            mlSdeCreateParams->vxMergeDisparityL0[i]  = appCntxt->vxMergeDisparityL0[i];
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appCntxt->mlSdeHdl = ML_SDEAPPLIB_create(mlSdeCreateParams);
            if (appCntxt->mlSdeHdl == NULL)
            {
                PTK_printf("[%s:%d] ML_SDEAPPLIB_create() failed\n",
                            __FUNCTION__, __LINE__);
                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_init_SS(ESTOP_APP_Context *appCntxt)
{
    int32_t               i;
    int32_t               vxStatus = VX_SUCCESS;
    vx_image              scalerInput;

    CM_ScalerNodeCntxt   *scalerObj;
    CM_DLRNodeCntxt      *dlrObj;

    scalerObj           = &appCntxt->scalerObj;
    dlrObj              = &appCntxt->dlrObj;

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_size tensorDims[ESTOP_APP_MAX_OUT_TENSOR_DIMS];

        /* DIM 0 - Width. */
        tensorDims[0] = appCntxt->tensor_width;

        /* DIM 1 - Height. */
        tensorDims[1] = appCntxt->tensor_height;

        /* DIM 2 - Number of channels. */
        tensorDims[2] = 1;

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxOutTensor[i] =
                vxCreateTensor(appCntxt->vxContext,
                               ESTOP_APP_MAX_OUT_TENSOR_DIMS,
                               tensorDims,
                               VX_TYPE_UINT8,
                               0);

            if (appCntxt->vxOutTensor[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateTensor() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        int32_t numDims;

#if 0
        CM_DLRCreateParams  params;
        int32_t             status;

        params.modelPath = appCntxt->dlrModelPath;
        params.devType   = DLR_DEVTYPE_CPU;
        params.devId     = 0;

        status = CM_dlrNodeCntxtInit(dlrObj, &params);

        if (status < 0)
        {
            PTK_printf("[%s:%d] CM_dlrNodeCntxtInit() failed.\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
#endif

        numDims = dlrObj->input.info[0].dim - 1;
        if (dlrObj->input.numInfo != CM_PREPROC_NUM_OUTPUT_TENSORS)
        {
            PTK_printf("[%s:%d] Number of DLR inputs [%d] does not match the "
                       "number of tensors [%d] output by the pre-processing "
                       "node.\n",
                        __FUNCTION__,
                        __LINE__,
                        dlrObj->input.numInfo,
                        CM_PREPROC_NUM_OUTPUT_TENSORS);

            vxStatus = VX_FAILURE;
        }
        else if (numDims != CM_PREPROC_OUTPUT_TENSORS_DIMS)
        {
            PTK_printf("[%s:%d] Number of DLR input dims [%d] does not match the "
                       "number of tensor dims [%d] output by the pre-processing "
                       "node.\n",
                        __FUNCTION__,
                        __LINE__,
                        numDims,
                        CM_PREPROC_OUTPUT_TENSORS_DIMS);

            vxStatus = VX_FAILURE;
        }
        else
        {
            CM_DLRIfInfo   *info;

            /* Allocate DLR input buffers. Currenly, the input buffers are held
             * for each pipelined outputs so we need pipeline number of input
             * buffers.
             */
            info = &dlrObj->input.info[0];
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                /* Allocate the input buffer. */
                appCntxt->dlrInputBuff[i] = (float *)
                    tivxMemAlloc(info->size * sizeof(float), TIVX_MEM_EXTERNAL);

                if (appCntxt->dlrInputBuff[i] == NULL)
                {
                    PTK_printf("[%s:%d] Failed to allocate input (%d) "
                               "of size (%d) bytes.\n",
                                __FUNCTION__, __LINE__, i, info->size);
                    vxStatus = VX_FAILURE;
                    break;
                }
            }

            /* Allocate DLR input buffer. Curently, the output buffer is not
             * held across frames so just one istance is sufficient.
             */
            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Allocate the output buffer. */
                info = &dlrObj->output.info[0];
                for (i = 0; i < appCntxt->pipelineDepth; i++)
                {
                    /* Allocate the input buffer. */
                    appCntxt->dlrOutputBuff[i] = (int32_t *)
                        tivxMemAlloc(info->size * sizeof(int32_t), TIVX_MEM_EXTERNAL);

                    if (appCntxt->dlrOutputBuff[i] == NULL)
                    {
                        PTK_printf("[%s:%d] Failed to allocate output (%d) "
                                   "of size (%d) bytes.\n",
                                    __FUNCTION__, __LINE__, i, info->size);
                        vxStatus = VX_FAILURE;
                        break;
                    }
                }
            }
        }
    }

    /* Input image will be the scaler default input. */
    scalerInput = appCntxt->vxRightRectImage[0];

    /************************/
    /*    Scaler node       */
    /************************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_ScalerCreateParams   params;

        params.width         = appCntxt->tensor_width;
        params.height        = appCntxt->tensor_height;
        params.imageType     = VX_DF_IMAGE_NV12;
        params.interpolation = VX_INTERPOLATION_BILINEAR;
        params.pipelineDepth = appCntxt->pipelineDepth;

        vxStatus = CM_scalerNodeCntxtInit(&appCntxt->scalerObj,
                                          appCntxt->vxContext,
                                          &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] CM_scalerNodeCntxtInit() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            vxStatus = CM_scalerNodeCntxtSetup(scalerObj,
                                               appCntxt->vxContext,
                                               appCntxt->vxGraph,
                                               scalerInput);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] CM_scalerNodeCntxtSetup() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_deinit_SS(ESTOP_APP_Context *appCntxt)
{
    CM_DLRNodeCntxt   *dlrObj;
    CM_DLRIfInfo      *info;
    int32_t            i;
    vx_status          vxStatus = VX_SUCCESS;

    dlrObj           = &appCntxt->dlrObj;

    /* Scaler node.  */
    CM_scalerNodeCntxtDeInit(&appCntxt->scalerObj);

    // vxOutTensor is always created
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->vxOutTensor[i] != NULL)
            {
                vxReleaseTensor(&appCntxt->vxOutTensor[i]);
            }
        }
    }

    /* De-allocate the input buffer. */
    info = &dlrObj->input.info[0];
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        if (appCntxt->dlrInputBuff[i] != NULL)
        {
            tivxMemFree(appCntxt->dlrInputBuff[i],
                        info->size * sizeof(float),
                        TIVX_MEM_EXTERNAL);
        }
    }

    /* De-allocate the DLR output buffer. */
    info = &dlrObj->output.info[0];
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        if (appCntxt->dlrOutputBuff[i] != NULL)
        {
            tivxMemFree(appCntxt->dlrOutputBuff[i],
                        info->size * sizeof(int32_t),
                        TIVX_MEM_EXTERNAL);
        }
    }

    return vxStatus;
}


vx_status ESTOP_APP_init_SS_Detection(ESTOP_APP_Context *appCntxt)
{
    SS_DETECT_APPLIB_createParams  *ssDetectCreateParams;
    vx_tensor                      *vxOutTensor;
    int32_t                         vxStatus = VX_SUCCESS;
    int32_t                         i;

    ssDetectCreateParams                = &appCntxt->ssDetectCreateParams;
    ssDetectCreateParams->vxContext     = appCntxt->vxContext;
    ssDetectCreateParams->vxGraph       = appCntxt->vxGraph;

    ssDetectCreateParams->width         = appCntxt->width;
    ssDetectCreateParams->height        = appCntxt->height;
    ssDetectCreateParams->tensorWidth   = appCntxt->tensor_width;
    ssDetectCreateParams->tensorHeight  = appCntxt->tensor_height;
    ssDetectCreateParams->pipelineDepth = appCntxt->pipelineDepth;
    ssDetectCreateParams->inputFormat   = appCntxt->inputFormat;

    /* 
     * Create input image objects for SL SDE
     */
    ssDetectCreateParams->createInputFlag = 0;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // right rectified image
        ssDetectCreateParams->vxRightRectImage[i] = appCntxt->vxRightRectImage[i];

        // raw disparity image
        if (appCntxt->sdeAlgoType == 0)
        {
            ssDetectCreateParams->vxSde16BitOutput[i] = appCntxt->vxSde16BitOutput[i];
        } else
        {
            if (appCntxt->ppMedianFilterEnable)
            {
                ssDetectCreateParams->vxSde16BitOutput[i] = appCntxt->vxMedianFilteredDisparity[i];
            } else
            {
                ssDetectCreateParams->vxSde16BitOutput[i] = appCntxt->vxMergeDisparityL0[i];
            }
        }
    }

    vxOutTensor = appCntxt->vxOutTensor;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        ssDetectCreateParams->vxSSMapTensor[i] = vxOutTensor[i];
    }

    // output objects are created in applib
    ssDetectCreateParams->createOutputFlag = 0;

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->ssDetectHdl = SS_DETECT_APPLIB_create(ssDetectCreateParams);
        if (appCntxt->ssDetectHdl == NULL)
        {
            PTK_printf("[%s:%d] SS_DETECT_APPLIB_create() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
        }
    }

    // get the output object from ss_detect applib
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        appCntxt->vx3DBoundBox[i] = SS_DETECT_APPLIB_get3DBBObject(appCntxt->ssDetectHdl, i);
    }

    return vxStatus;
}



vx_status  ESTOP_APP_setupPipeline(ESTOP_APP_Context * appCntxt)
{
    uint32_t   appValue;
    vx_status  vxStatus = VX_SUCCESS;

    if (appCntxt->sdeAlgoType == 0)
    {
        vxStatus = ESTOP_APP_setupPipeline_SL(appCntxt);
    } else
    {
        vxStatus = ESTOP_APP_setupPipeline_ML(appCntxt);
    }

    if (vxStatus == VX_SUCCESS)
    {
        appValue = appCntxt->vxEvtAppValBase + ESTOP_APP_GRAPH_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)appCntxt->vxGraph,
                                   VX_EVENT_GRAPH_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxRegisterEvent() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }


    if (vxStatus == VX_SUCCESS)
    {
        CM_ScalerNodeCntxt * scalerObj = &appCntxt->scalerObj;

        appValue = appCntxt->vxEvtAppValBase + ESTOP_APP_SCALER_NODE_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)scalerObj->vxNode,
                                   VX_EVENT_NODE_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxRegisterEvent() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }    


    return vxStatus;
}

vx_status ESTOP_APP_setupPipeline_SL(ESTOP_APP_Context * appCntxt)
{
    ESTOP_APP_graphParams             * paramDesc;
    vx_graph_parameter_queue_params_t   q[ESTOP_APP_NUM_GRAPH_PARAMS];
    uint32_t                            i;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    vx_node leftLdcNode  = SDELCDAPPLIB_getLeftLDCNode(appCntxt->sdeLdcHdl);
    vx_node rightLdcNode = SDELCDAPPLIB_getRightLDCNode(appCntxt->sdeLdcHdl);
    vx_node sdeNode      = SL_SDEAPPLIB_getSDENode(appCntxt->slSdeHdl);
    vx_node pcNode       = SS_DETECT_APPLIB_getPCNode(appCntxt->ssDetectHdl);
    vx_node ogNode       = SS_DETECT_APPLIB_getOGNode(appCntxt->ssDetectHdl);

    CM_ScalerNodeCntxt   * scalerObj   = &appCntxt->scalerObj;
    vx_tensor            * vxOutTensor = appCntxt->vxOutTensor;

    float             ** dlrInputBuff  = appCntxt->dlrInputBuff;
    int32_t           ** dlrOutputBuff = appCntxt->dlrOutputBuff;

    appCntxt->numGraphParams = 5;

    /* LDC left node Param 6 (leftInputImg) ==> graph param 0. */
    vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                           leftLdcNode,
                                           6);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                    __FUNCTION__, __LINE__);
    }
    else
    {
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputLeftImage;
    }

    /* LDC right node Param 6 (rightInputImg) ==> graph param 1. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               rightLdcNode,
                                               6);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputRightImage;
        }
    }

    /* LDC right node Param 7 (rightOutputImg)==> graph param 2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               rightLdcNode,
                                               7);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxRightRectImage;
        }
    }

    /* vxDmpacSdeNode Param 3 (vxSde16BitOutput) ==> graph param 3 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               sdeNode,
                                               3);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxSde16BitOutput;
        }
    }

    /* vxOGNode Param 2 (vx3DBoundBox) ==> graph param 4. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                    ogNode,
                                    2);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vx3DBoundBox;
        }
    } 


    /*  scalerObj->vxNode Param 1 (outImage)==> graph param 5 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               scalerObj->vxNode,
                                               1);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)scalerObj->outImage;
        }
    }

    /*  pcNode Param 3 (vxSSMapTensor)==> graph param 6 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               pcNode,
                                               3);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)vxOutTensor;
        }
    }


    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < cnt; i++)
        {
            q[i].graph_parameter_index = i;
            q[i].refs_list_size        = appCntxt->pipelineDepth;
        }

        // allocate free Q
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            paramDesc                      = &appCntxt->paramDesc[i];
            paramDesc->vxInputLeftImage    = appCntxt->vxInputLeftImage[i];
            paramDesc->vxInputRightImage   = appCntxt->vxInputRightImage[i];
            paramDesc->vxRightRectImage    = appCntxt->vxRightRectImage[i];
            paramDesc->vxSde16BitOutput    = appCntxt->vxSde16BitOutput[i];
            paramDesc->vx3DBoundBox        = appCntxt->vx3DBoundBox[i];

            paramDesc->vxScalerOut         = scalerObj->outImage[i];
            paramDesc->vxOutTensor         = vxOutTensor[i];

            paramDesc->dlrInputBuff        = dlrInputBuff[i];
            paramDesc->dlrOutputBuff       = dlrOutputBuff[i];

            paramDesc->timestamp           = &appCntxt->timestamp[i];

            ESTOP_APP_enqueInputDesc(appCntxt, paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);
    }

    if (vxStatus == VX_SUCCESS)
    {
        /* explicitly set graph pipeline depth */
        vxStatus = tivxSetGraphPipelineDepth(appCntxt->vxGraph,
                                             appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxSetGraphPipelineDepth() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }
    else
    {
        PTK_printf("[%s:%d] vxSetGraphScheduleConfig() failed\n",
                   __FUNCTION__, __LINE__);
    }

    if (vxStatus == VX_SUCCESS)
    {
        /*  vxLeftRectImage */
        vxStatus = tivxSetNodeParameterNumBufByIndex(leftLdcNode,
                                                     7,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);

        // vxPointCloud */
        vxStatus =
            tivxSetNodeParameterNumBufByIndex(pcNode,
                                              4, 
                                              appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);
    }

    return vxStatus;
}


vx_status ESTOP_APP_setupPipeline_ML(ESTOP_APP_Context * appCntxt)
{
    ESTOP_APP_graphParams             * paramDesc;
    vx_graph_parameter_queue_params_t   q[ESTOP_APP_NUM_GRAPH_PARAMS];
    uint32_t                            i;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    vx_node leftLdcNode    = SDELCDAPPLIB_getLeftLDCNode(appCntxt->sdeLdcHdl);
    vx_node rightLdcNode   = SDELCDAPPLIB_getRightLDCNode(appCntxt->sdeLdcHdl);

    vx_node sdeNodeL0      = ML_SDEAPPLIB_getSDENodeL0(appCntxt->mlSdeHdl);
    vx_node sdeNodeL1      = ML_SDEAPPLIB_getSDENodeL1(appCntxt->mlSdeHdl);
    vx_node sdeNodeL2      = ML_SDEAPPLIB_getSDENodeL2(appCntxt->mlSdeHdl);
    vx_node medFilterNode  = ML_SDEAPPLIB_getMedFilterNode(appCntxt->mlSdeHdl);
    vx_node mergeNodeL1    = ML_SDEAPPLIB_getMergeNodeL1(appCntxt->mlSdeHdl);
    vx_node mergeNodeL2    = ML_SDEAPPLIB_getMergeNodeL2(appCntxt->mlSdeHdl);
    vx_node leftMscNodeL1  = ML_SDEAPPLIB_getLeftMSCNodeL1(appCntxt->mlSdeHdl);
    vx_node rightMscNodeL1 = ML_SDEAPPLIB_getRightMSCNodeL1(appCntxt->mlSdeHdl);
    vx_node leftMscNodeL2  = ML_SDEAPPLIB_getLeftMSCNodeL2(appCntxt->mlSdeHdl);
    vx_node rightMscNodeL2 = ML_SDEAPPLIB_getRightMSCNodeL2(appCntxt->mlSdeHdl);

    vx_node pcNode         = SS_DETECT_APPLIB_getPCNode(appCntxt->ssDetectHdl);
    vx_node ogNode         = SS_DETECT_APPLIB_getOGNode(appCntxt->ssDetectHdl);

    CM_ScalerNodeCntxt   * scalerObj   = &appCntxt->scalerObj; 
    vx_tensor            * vxOutTensor = appCntxt->vxOutTensor;

    float             ** dlrInputBuff  = appCntxt->dlrInputBuff;
    int32_t           ** dlrOutputBuff = appCntxt->dlrOutputBuff;

    appCntxt->numGraphParams = 5;

    /* LDC left node Param 6 (leftInputImg) ==> graph param 0. */
    vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                           leftLdcNode,
                                           6);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                    __FUNCTION__, __LINE__);
    }
    else
    {
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputLeftImage;
    }

    /* LDC rigth node Param 6 (rightInputImg) ==> graph param 1. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                              rightLdcNode,
                                              6);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputRightImage;
        }
    }

    /* LDC right node Param 7  (rightOutputImg) ==> graph param 2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               rightLdcNode,
                                               7);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxRightRectImage;
        }
    }

    /* vxDisparityMergeNodeL1 Param 3 (vxMergeDisparityL0) => graph param 3. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               mergeNodeL1,
                                               3);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxMergeDisparityL0;
        }
    }

    // Always ppMedianFilterEnable = 0 
    if (vxStatus == (vx_status)VX_SUCCESS && appCntxt->ppMedianFilterEnable)
    {
        appCntxt->numGraphParams += 1;

        /* vxMedianFilterNode Param 2 (vxMedianFilteredDisparity) => graph param 3. */
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               medFilterNode,
                                               2);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxMedianFilteredDisparity;
        }
    }

    /* vxOGNode Param 2 ==> graph param 4. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               ogNode,
                                               2);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vx3DBoundBox;
        }
    }    


    /*  scalerObj->vxNode Param 1 (outImage)==> graph param 5 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               scalerObj->vxNode,
                                               1);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)scalerObj->outImage;
        }
    }

    /*  pcNode Param 3 (vxSSMapTensor)==> graph param 6 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               pcNode,
                                               3);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)vxOutTensor;
        }
    } 


    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < cnt; i++)
        {
            q[i].graph_parameter_index = i;
            q[i].refs_list_size        = appCntxt->pipelineDepth;
        }

        // allocate free Q
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            paramDesc                      = &appCntxt->paramDesc[i];
            paramDesc->vxInputLeftImage    = appCntxt->vxInputLeftImage[i];
            paramDesc->vxInputRightImage   = appCntxt->vxInputRightImage[i];
            paramDesc->vxRightRectImage    = appCntxt->vxRightRectImage[i];
            paramDesc->vxMergeDisparityL0  = appCntxt->vxMergeDisparityL0[i];

            if (appCntxt->ppMedianFilterEnable)
            {
                paramDesc->vxMedianFilteredDisparity = appCntxt->vxMedianFilteredDisparity[i];
            }

            paramDesc->vx3DBoundBox        = appCntxt->vx3DBoundBox[i];

            paramDesc->vxScalerOut         = scalerObj->outImage[i];
            paramDesc->vxOutTensor         = vxOutTensor[i];

            paramDesc->dlrInputBuff        = dlrInputBuff[i];
            paramDesc->dlrOutputBuff       = dlrOutputBuff[i];

            paramDesc->timestamp           = &appCntxt->timestamp[i];

            ESTOP_APP_enqueInputDesc(appCntxt, paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);
    }

    if (vxStatus == VX_SUCCESS)
    {
        /* explicitly set graph pipeline depth */
        vxStatus = tivxSetGraphPipelineDepth(appCntxt->vxGraph,
                                             appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxSetGraphPipelineDepth() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }
    else
    {
        PTK_printf("[%s:%d] vxSetGraphScheduleConfig() failed\n",
                   __FUNCTION__, __LINE__);
    }

    if (vxStatus == VX_SUCCESS)
    {
        /*  vxLeftRectImage */
        vxStatus = tivxSetNodeParameterNumBufByIndex(leftLdcNode,
                                                     7,
                                                     appCntxt->pipelineDepth);

        /* vxSde16BitOutputL0 */
        vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL0,
                                                     3,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);

        if (appCntxt->numLayers > 1)
        {
            /* vxLeftImageL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(leftMscNodeL1,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxRightImageL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(rightMscNodeL1,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxSde16BitOutputL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL1,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);
        }

        if (appCntxt->numLayers > 2)
        {
            /* vxLeftImageL2 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(leftMscNodeL2,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxRightImageL2 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(rightMscNodeL2,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxSde16BitOutputL2 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL2,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxMergeDisparityL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(mergeNodeL2,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);
        }

        // vxPointCloud */
        vxStatus =
            tivxSetNodeParameterNumBufByIndex(pcNode,
                                              4, 
                                              appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);
    }

    return vxStatus;
}

void ESTOP_APP_printStats(ESTOP_APP_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);
    appPerfPointPrint(&appCntxt->estopPerf);
    PTK_printf("\n");
    appPerfPointPrintFPS(&appCntxt->estopPerf);
    PTK_printf("\n");
}

vx_status ESTOP_APP_exportStats(ESTOP_APP_Context * appCntxt, FILE *fp, bool exportAll)
{
    vx_status vxStatus = tivx_utils_graph_perf_export(fp, appCntxt->vxGraph);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivx_utils_graph_perf_export() failed\n",
                    __FUNCTION__, __LINE__);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (exportAll == true)
        {
            app_perf_point_t *perfArr[1] = {&appCntxt->estopPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_waitGraph(ESTOP_APP_Context * appCntxt)
{
    vx_status vxStatus = VX_SUCCESS;

    vxWaitGraph(appCntxt->vxGraph);

    /* Wait for the output queue to get flushed. */
    while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return vxStatus;
}

vx_status ESTOP_APP_popFreeInputDesc(ESTOP_APP_Context       *appCntxt,
                                     ESTOP_APP_graphParams  **gpDesc)
{
    vx_status vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->freeQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status ESTOP_APP_popPreprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                        ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->preProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status ESTOP_APP_popDLRInputDesc(ESTOP_APP_Context       *appCntxt,
                                    ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->dlrQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status ESTOP_APP_popPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->postProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status ESTOP_APP_getPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc)
{
    ESTOP_APP_graphParams  *desc;
    vx_status               vxStatus = VX_SUCCESS;

    desc = appCntxt->postProcQ.peek();

    if (desc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }
    else
    {
        *gpDesc = desc;
    }


    return vxStatus;
}


vx_status ESTOP_APP_getOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams   *gpDesc)
{
    ESTOP_APP_graphParams  *desc;
    vx_status               vxStatus = VX_SUCCESS;

    desc = appCntxt->outputQ.peek();

    if (desc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }
    else
    {
        *gpDesc = *desc;
    }

    return vxStatus;
}


vx_status ESTOP_APP_popOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->outputQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


void ESTOP_APP_enqueInputDesc(ESTOP_APP_Context      *appCntxt,
                              ESTOP_APP_graphParams  *desc)
{
    appCntxt->freeQ.push(desc);
}


void ESTOP_APP_enquePreprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                     ESTOP_APP_graphParams  *desc)
{
    appCntxt->preProcQ.push(desc);
}

void ESTOP_APP_enqueDLRInputDesc(ESTOP_APP_Context      *appCntxt,
                                 ESTOP_APP_graphParams  *desc)
{
    appCntxt->dlrQ.push(desc);
}

void ESTOP_APP_enquePostprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                      ESTOP_APP_graphParams  *desc)
{
    appCntxt->postProcQ.push(desc);
}

void ESTOP_APP_enqueOutputDesc(ESTOP_APP_Context      *appCntxt,
                               ESTOP_APP_graphParams  *desc)
{
    appCntxt->outputQ.push(desc);
}




vx_status  ESTOP_APP_process(ESTOP_APP_Context * appCntxt, ESTOP_APP_graphParams * gpDesc)
{
    vx_status vxStatus = VX_SUCCESS;
    uint16_t   i;
    uint8_t   cnt = 0;

    vx_reference  obj[ESTOP_APP_NUM_GRAPH_PARAMS];

    obj[cnt++] = (vx_reference)gpDesc->vxInputLeftImage;
    obj[cnt++] = (vx_reference)gpDesc->vxInputRightImage;
    obj[cnt++] = (vx_reference)gpDesc->vxRightRectImage;

    if (appCntxt->sdeAlgoType == 0)
    {
        obj[cnt++] = (vx_reference)gpDesc->vxSde16BitOutput;
    } 
    else
    {
        /* code */
        obj[cnt++] = (vx_reference)gpDesc->vxMergeDisparityL0;

        // ppMedianFilterEnable is always 0 
        if (appCntxt->ppMedianFilterEnable)
        {
            obj[cnt++] = (vx_reference)gpDesc->vxMedianFilteredDisparity;
        }
    }

    obj[cnt++] = (vx_reference)gpDesc->vx3DBoundBox;

    obj[cnt++] = (vx_reference)gpDesc->vxScalerOut;

    // Enqueue buffers
    // Enqueue vxOutTensor after DLR is completed
    for (i = 0; i < cnt; i++)
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                   i,
                                                   &obj[i],
                                                   1);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxGraphParameterEnqueueReadyRef(%d) "
                       "failed\n", __FUNCTION__, __LINE__, i);
            break;
        }
    }


    /* Push the descriptor to the DLR input queue. */
    ESTOP_APP_enquePreprocInputDesc(appCntxt, gpDesc);

    return vxStatus;
}


vx_status ESTOP_APP_CNN_preProcess(ESTOP_APP_Context     * appCntxt, 
                                   vx_image                vxScalerOut,
                                   float                 * dlrInputBuff)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    uint8_t                    *srcPtr[2];
    vx_map_id                   mapId[2];
    bool                        mapped[2] = {false, false};
    vx_status                   vxStatus;

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = appCntxt->tensor_width;
    rect.end_y   = appCntxt->tensor_height;

    /* Map the image planes. */
    for (uint32_t i = 0; i < 2; i++)
    {
        /* Get the pointer to the YUV data plans. */
        vxStatus = vxMapImagePatch(vxScalerOut,
                                   &rect,
                                   i,
                                   &mapId[i],
                                   &imgAddr,
                                   (void **)&srcPtr[i],
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxMapImagePatch() failed.",
                       __FUNCTION__, __LINE__);
        }
        else
        {
            mapped[i] = true;
        }

        /* UV plane has half the height. */
        rect.end_y /= 2;
    }

    /* Take thr output image from the scaler which is in NV12 format and do
     * the following:
     * - Color convert the image from YUV420 to BGR format
     * - Convert to float and write to the DLR input buffer
     */

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus =
            CM_SEMSEG_CNN_convertYUV2RGB(dlrInputBuff,
                                         (const uint8_t**)srcPtr,
                                         appCntxt->preProcMean,
                                         appCntxt->preProcScale,
                                         appCntxt->tensor_width,
                                         appCntxt->tensor_height);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] CM_SEMSEG_CNN_convertYUV2RGB() failed.",
                       __FUNCTION__, __LINE__);
        }
    }

    /* Unmap the image planes. */
    for (uint32_t i = 0; i < 2; i++)
    {
        if (mapped[i] == true)
        {
            vxUnmapImagePatch(vxScalerOut, mapId[i]);
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_CNN_postProcess(ESTOP_APP_Context     * appCntxt, 
                                    vx_image                vxScalerOut,
                                    int32_t               * dlrOutputBuff)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    vx_map_id                   mapId;
    uint8_t                    *dPtr;
    vx_status                   vxStatus;

    /* Get the pointer to the UV plane in the output image. */
    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = appCntxt->tensor_width;
    rect.end_y   = appCntxt->tensor_height/2; // Chroma is UV interleaved

    /* Get the pointer to the UV data plans. */
    vxStatus = vxMapImagePatch(vxScalerOut,
                               &rect,
                               1, // UV plane
                               &mapId,
                               &imgAddr,
                               (void **)&dPtr,
                               VX_WRITE_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxMapImagePatch() failed.",
                   __FUNCTION__, __LINE__);     
    }
    else
    {
        int32_t    *classId;
        uint8_t    *p;
        float       alpha;
        float       alphaC;
        int32_t     widthInByte;

        alpha  = 0.5;
        alphaC = 1.0f - alpha;

        classId     = dlrOutputBuff;
        widthInByte = imgAddr.dim_x/2;

        /* Blend the input image data with the DLR output tensor and generate
         * the output image.
         */
        for (uint32_t i = 0; i < imgAddr.dim_y; i++)
        {
            p = dPtr;

            for (int32_t j = 0; j < widthInByte; j++)
            {
                uint8_t c = (uint8_t)*classId;

                p[0] = (uint8_t)((p[0] * alpha) + yuvColMap[c].u * alphaC);
                p[1] = (uint8_t)((p[1] * alpha) + yuvColMap[c].v * alphaC);

                /* Move to the next pain of UV pixels. */
                p += 2;

                /* We use one class Id for a pair of UV pixels, so skip one
                 * class value.
                 */
                classId += 2;
            }

            /* Move to the next line in the image. */
            dPtr += imgAddr.stride_y;

            /* Skip the next line in the tensor data. */
            classId += appCntxt->tensor_width;
        }

        vxUnmapImagePatch(vxScalerOut, mapId);
    }

    return vxStatus;
    
}



vx_status ESTOP_APP_createOutTensor(ESTOP_APP_Context     * appCntxt,
                                    vx_tensor                vxOutTensor,
                                    int32_t                * dlrOutputBuff)
{
    uint8_t    *dPtr;
    vx_size     start[ESTOP_APP_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[ESTOP_APP_MAX_OUT_TENSOR_DIMS];
    vx_size     tensorDims[ESTOP_APP_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    start[0] = 0;
    start[1] = 0;
    start[2] = 0;

    /* DIM 0 - Width. */
    tensorDims[0] = appCntxt->tensor_width;

    /* DIM 1 - Height. */
    tensorDims[1] = appCntxt->tensor_height;

    /* DIM 2 - Number of channels. */
    tensorDims[2] = 1;

    /* Strides */
    strides[0] = 1;
    strides[1] = tensorDims[0];
    strides[2] = strides[1]*tensorDims[1];

    vxStatus = tivxMapTensorPatch(vxOutTensor,
                                  ESTOP_APP_MAX_OUT_TENSOR_DIMS,
                                  start,
                                  tensorDims,
                                  &mapId,
                                  strides,
                                  (void **)&dPtr,
                                  VX_READ_AND_WRITE,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivxMapTensorPatch() failed.",
                   __FUNCTION__, __LINE__);
    }
    else
    {
        int32_t    *sPtr;
        uint32_t    size;

        sPtr = dlrOutputBuff;
        size = tensorDims[0] * tensorDims[1];

        for (uint32_t i = 0; i < size; i++)
        {
            *dPtr++ = (uint8_t)*sPtr++;
        }

        vxStatus = tivxUnmapTensorPatch(vxOutTensor, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxUnmapTensorPatch() failed.",
                       __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_processEvent(ESTOP_APP_Context * appCntxt, vx_event_t * event)
{
    vx_reference            ref;
    uint8_t                 i;
    uint32_t                numRefs;
    uint32_t                index;
    vx_status               vxStatus;

    // For profiling
    float                   diff;

    ref      = NULL;
    vxStatus = (vx_status)VX_SUCCESS;

    if(event->type == VX_EVENT_NODE_COMPLETED)
    {
        uint32_t appValue = appCntxt->vxEvtAppValBase + 
                            ESTOP_APP_SCALER_NODE_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            PTK_printf("[%s:%d] Unknown App Value [%d].\n",
                       __FUNCTION__, __LINE__, event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Wakeup the DLR thread. The DLR thread will process the
             * descriptor at the head of the queue.
             */
            appCntxt->preProcSem->notify();
        }

    } else
    if(event->type == VX_EVENT_GRAPH_COMPLETED)
    {
        uint32_t appValue = appCntxt->vxEvtAppValBase + ESTOP_APP_GRAPH_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            PTK_printf("[%s:%d] Unknown App Value [%d].\n",
                       __FUNCTION__, __LINE__, event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->startPerfCapt == false)
            {
                appCntxt->startPerfCapt = true;
            }
            else
            {
                appPerfPointEnd(&appCntxt->estopPerf);

                appCntxt->profileEnd = GET_TIME();
                diff  = GET_DIFF(appCntxt->profileStart, appCntxt->profileEnd);
                CM_reportProctime("Inter_frame_interval", diff);
            }

            appPerfPointBegin(&appCntxt->estopPerf);
            appCntxt->profileStart = GET_TIME();

            /* Node execution is complete. Deque all the parameters
             * for this node.
             */
            for (i = 0; i < appCntxt->numGraphParams; i++)
            {
                vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
                                                          i,
                                                          &ref,
                                                          1,
                                                          &numRefs);
                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxGraphParameterDequeueDoneRef() failed\n",
                               __FUNCTION__, __LINE__);

                    break;
                }
            }
        }

        ESTOP_APP_graphParams * desc;
        vxStatus = ESTOP_APP_popPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            ESTOP_APP_enqueOutputDesc(appCntxt, desc);

            /* Wakeup the display thread. */
            if (appCntxt->outputCtrlSem)
            {
                appCntxt->outputCtrlSem->notify();
            }
        }
    }

    return vxStatus;
}


vx_status ESTOP_APP_getOutBuff(ESTOP_APP_Context *appCntxt, 
                               vx_image *rightRectImage, 
                               vx_image *ssOutput, 
                               vx_tensor *outTensor, 
                               vx_user_data_object *obsBB, 
                               vx_image *disparity16,
                               vx_uint64 *timestamp)
{
    vx_status                        vxStatus = VX_SUCCESS;
    vx_map_id                        map_id;

    ESTOP_APP_graphParams            desc;
    tivx_ss_sde_obs_3d_bound_box_t * boundingBox;

    vxStatus = ESTOP_APP_getOutputDesc(appCntxt, &desc);

    /* Get the descriptor. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *rightRectImage = desc.vxRightRectImage;
        *ssOutput       = desc.vxScalerOut;
        *outTensor      = desc.vxOutTensor;

        if (appCntxt->sdeAlgoType == 0)
        {
            *disparity16 = desc.vxSde16BitOutput;
        }
        else if (appCntxt->ppMedianFilterEnable)
        {
            *disparity16 = desc.vxMedianFilteredDisparity;
        }
        else 
        {
            *disparity16 = desc.vxMergeDisparityL0;
        }

        *obsBB = desc.vx3DBoundBox;
        *timestamp = *desc.timestamp;
    }


    return vxStatus;
}


vx_status ESTOP_APP_releaseOutBuff(ESTOP_APP_Context * appCntxt)
{
    vx_status                       vxStatus;
    ESTOP_APP_graphParams         * desc;

    /* Pop the output descriptor. */
    vxStatus = ESTOP_APP_popOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Enqueue the descriptor to the free descriptor queue. */
        ESTOP_APP_enqueInputDesc(appCntxt, desc);
    }

    return vxStatus;
}
