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

#include "semseg_cnn.h"

#define SEMSEG_CNN_MAX_OUT_TENSOR_DIMS   (3U)
#define SEMSEG_CNN_NUM_GRAPH_PARAMS      (3U)



vx_status SEMSEG_CNN_init_SS(SEMSEG_CNN_Context *appCntxt)
{
    int32_t               i;
    int32_t               vxStatus = VX_SUCCESS;
    vx_image              scalerInput;
    CM_LdcNodeCntxt      *ldcObj;
    CM_ScalerNodeCntxt   *scalerObj;
    CM_DLRNodeCntxt      *dlrObj;

    ldcObj              = &appCntxt->ldcObj;
    scalerObj           = &appCntxt->scalerObj;
    dlrObj              = &appCntxt->dlrObj;


    // Create input image object
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            // create objects
            appCntxt->vxInputImage[i] = vxCreateImage(appCntxt->vxContext,
                                                      appCntxt->inputImageWidth,
                                                      appCntxt->inputImageHeight,
                                                      VX_DF_IMAGE_UYVY);

            if (appCntxt->vxInputImage[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxInputImage[i],
                               "InputImage");
        }
    }

    // Create rectified image object
    if ((vxStatus == (vx_status)VX_SUCCESS) && appCntxt->enableLdcNode)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxRectifiedImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->inputImageWidth,
                              appCntxt->inputImageHeight,
                              VX_DF_IMAGE_NV12);

            if (appCntxt->vxRectifiedImage[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxRectifiedImage[i],
                               "RectifiedImage_NV12");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_size tensorDims[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];

        /* DIM 0 - Width. */
        tensorDims[0] = appCntxt->tidlImageWidth;

        /* DIM 1 - Height. */
        tensorDims[1] = appCntxt->tidlImageHeight;

        /* DIM 2 - Number of channels. */
        tensorDims[2] = 1;

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxOutTensor[i] =
                vxCreateTensor(appCntxt->vxContext,
                               SEMSEG_CNN_MAX_OUT_TENSOR_DIMS,
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
    scalerInput = appCntxt->vxInputImage[0];

    /************************/
    /*     LDC node         */
    /************************/
    if ((appCntxt->enableLdcNode) && (vxStatus == (vx_status)VX_SUCCESS))
    {
        CM_LdcCreateParams  params;

        params.width       = appCntxt->inputImageWidth;
        params.height      = appCntxt->inputImageHeight;
        params.ssFactor    = appCntxt->ldcSsFactor;
        params.blockWidth  = appCntxt->ldcBlockWidth;
        params.blockHeight = appCntxt->ldcBlockHeight;
        params.pixelPad    = appCntxt->ldcPixelPad;
        params.lutFilePath = appCntxt->ldcLutFilePath;

        vxStatus = CM_ldcNodeCntxtInit(&appCntxt->ldcObj,
                                       appCntxt->vxContext,
                                       &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] CM_ldcNodeCntxtInit() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            vxStatus = CM_ldcNodeCntxtSetup(ldcObj,
                                            appCntxt->vxContext,
                                            appCntxt->vxGraph,
                                            appCntxt->vxInputImage[0],
                                            appCntxt->vxRectifiedImage[0]);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] CM_ldcNodeCntxtSetup() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                /* LDC output will be the scaler input. */
                scalerInput = appCntxt->vxRectifiedImage[0];
            }
        }
    }

    /************************/
    /*    Scaler node       */
    /************************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_ScalerCreateParams   params;

        params.width         = appCntxt->tidlImageWidth;
        params.height        = appCntxt->tidlImageHeight;
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

vx_status SEMSEG_CNN_deinit_SS(SEMSEG_CNN_Context *appCntxt)
{
    CM_DLRNodeCntxt   *dlrObj;
    CM_DLRIfInfo      *info;
    int32_t            i;
    vx_status          vxStatus = VX_SUCCESS;

    dlrObj           = &appCntxt->dlrObj;

    
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        if (appCntxt->vxInputImage[i] != NULL)
        {
            vxReleaseImage(&appCntxt->vxInputImage[i]);
        }
    }

    if (appCntxt->enableLdcNode)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->vxRectifiedImage[i] != NULL)
            {
                vxReleaseImage(&appCntxt->vxRectifiedImage[i]);
            }
        }

        CM_ldcNodeCntxtDeInit(&appCntxt->ldcObj);
    }

    
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

vx_status  SEMSEG_CNN_setupPipeline(SEMSEG_CNN_Context * appCntxt)
{
    CM_LdcNodeCntxt                    *ldcObj;
    CM_ScalerNodeCntxt                 *scalerObj;
    SEMSEG_CNN_graphParams             *paramDesc;
    vx_graph_parameter_queue_params_t   q[SEMSEG_CNN_NUM_GRAPH_PARAMS];

    vx_status             vxStatus;
    uint32_t              appValue;
    uint32_t              i;
    uint32_t              cnt;

    ldcObj     = &appCntxt->ldcObj;
    scalerObj  = &appCntxt->scalerObj;
    cnt        = 0;

    appCntxt->numGraphParams = 3;

    /* LDC node Param 6 ==> graph param 0. */
    vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                           ldcObj->vxNode,
                                           6);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                    __FUNCTION__, __LINE__);
    }
    else
    {
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputImage;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* LDC node Param 7 ==> graph param 1. */
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               ldcObj->vxNode,
                                               7);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxRectifiedImage;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Scaler node Param 1 ==> graph param 1/2. */
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
            paramDesc                   = &appCntxt->paramDesc[i];
            paramDesc->vxInputImage     = appCntxt->vxInputImage[i];
            paramDesc->vxOutTensor      = appCntxt->vxOutTensor[i];
            paramDesc->timestamp        = &appCntxt->timestamp[i];
            paramDesc->dlrInputBuff     = appCntxt->dlrInputBuff[i];
            paramDesc->dlrOutputBuff    = appCntxt->dlrOutputBuff[i];
            paramDesc->vxScalerOut      = scalerObj->outImage[i];
            paramDesc->vxRectifiedImage = appCntxt->vxRectifiedImage[i];

            SEMSEG_CNN_enqueInputDesc(appCntxt, paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxSetGraphScheduleConfig() failed\n",
                       __FUNCTION__, __LINE__);
        }
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

    if (vxStatus == VX_SUCCESS)
    {
        appValue = appCntxt->vxEvtAppValBase + SEMSEG_CNN_SCALER_NODE_COMPLETE_EVENT;

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


void SEMSEG_CNN_printStats(SEMSEG_CNN_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);

    appPerfPointPrint(&appCntxt->semsegPerf);
    PTK_printf("\n");
    appPerfPointPrintFPS(&appCntxt->semsegPerf);
    PTK_printf("\n");
}


vx_status SEMSEG_CNN_exportStats(SEMSEG_CNN_Context * appCntxt, FILE *fp, bool exportAll)
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
            app_perf_point_t *perfArr[1] = {&appCntxt->semsegPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}


vx_status SEMSEG_CNN_waitGraph(SEMSEG_CNN_Context * appCntxt)
{
    vx_status vxStatus;

    vxStatus = vxWaitGraph(appCntxt->vxGraph);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxWaitGraph() failed\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Wait for the output queue to get flushed. */
        while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return vxStatus;
}


vx_status  SEMSEG_CNN_process(SEMSEG_CNN_Context     * appCntxt, 
                              SEMSEG_CNN_graphParams * gpDesc,
                              uint64_t                 timestamp)
{
    vx_status vxStatus = VX_SUCCESS;
    uint16_t   i;
    uint8_t    cnt = 0;

    vx_reference  obj[SEMSEG_CNN_NUM_GRAPH_PARAMS];

    // set timestamp of input image
    *gpDesc->timestamp = timestamp;

    obj[cnt++] = (vx_reference)gpDesc->vxInputImage;
    if (appCntxt->enableLdcNode)
    {
        obj[cnt++] = (vx_reference)gpDesc->vxRectifiedImage;
    }
    obj[cnt++] = (vx_reference)gpDesc->vxScalerOut;

    // Enqueue buffers
    // Enqueue vxOutTensor after DLR is completed
    for (i = 0; i < appCntxt->numGraphParams; i++)
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
    SEMSEG_CNN_enquePreprocInputDesc(appCntxt, gpDesc);

    return vxStatus;
}


vx_status SEMSEG_CNN_preProcess(SEMSEG_CNN_Context     * appCntxt, 
                                vx_image                 vxScalerOut,
                                float                  * dlrInputBuff)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    uint8_t                    *srcPtr[2];
    vx_map_id                   mapId[2];
    bool                        mapped[2] = {false, false};
    vx_status                   vxStatus;

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = appCntxt->tidlImageWidth;
    rect.end_y   = appCntxt->tidlImageHeight;

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
                                         appCntxt->tidlImageWidth,
                                         appCntxt->tidlImageHeight);

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

vx_status SEMSEG_CNN_postProcess(SEMSEG_CNN_Context     * appCntxt, 
                                 vx_image                 vxScalerOut,
                                 int32_t                * dlrOutputBuff)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    vx_map_id                   mapId;
    uint8_t                    *dPtr;
    vx_status                   vxStatus;

    /* Get the pointer to the UV plane in the output image. */
    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = appCntxt->tidlImageWidth;
    rect.end_y   = appCntxt->tidlImageHeight/2; // Chroma is UV interleaved

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
            classId += appCntxt->tidlImageWidth;
        }

        vxUnmapImagePatch(vxScalerOut, mapId);
    }

    return vxStatus;
    
}

vx_status SEMSEG_CNN_createOutTensor(SEMSEG_CNN_Context     * appCntxt,
                                     vx_tensor                vxOutTensor,
                                     int32_t                * dlrOutputBuff)
{
    uint8_t    *dPtr;
    vx_size     start[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];
    vx_size     tensorDims[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    start[0] = 0;
    start[1] = 0;
    start[2] = 0;

    /* DIM 0 - Width. */
    tensorDims[0] = appCntxt->tidlImageWidth;

    /* DIM 1 - Height. */
    tensorDims[1] = appCntxt->tidlImageHeight;

    /* DIM 2 - Number of channels. */
    tensorDims[2] = 1;

    /* Strides */
    strides[0] = 1;
    strides[1] = tensorDims[0];
    strides[2] = strides[1]*tensorDims[1];

    vxStatus = tivxMapTensorPatch(vxOutTensor,
                                  SEMSEG_CNN_MAX_OUT_TENSOR_DIMS,
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



vx_status SEMSEG_CNN_popFreeInputDesc(SEMSEG_CNN_Context        *appCntxt,
                                      SEMSEG_CNN_graphParams   **gpDesc)
{
    vx_status  vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->freeQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status SEMSEG_CNN_popPreprocInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                         SEMSEG_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->preProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status SEMSEG_CNN_popDLRInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                     SEMSEG_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->dlrQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_popPostprocInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                          SEMSEG_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->postProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_getOutputDesc(SEMSEG_CNN_Context       *appCntxt,
                                   SEMSEG_CNN_graphParams   *gpDesc)
{
    SEMSEG_CNN_graphParams  *desc;
    vx_status                vxStatus = VX_SUCCESS;

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

vx_status SEMSEG_CNN_popOutputDesc(SEMSEG_CNN_Context       *appCntxt,
                                   SEMSEG_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->outputQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


void SEMSEG_CNN_enqueInputDesc(SEMSEG_CNN_Context      *appCntxt,
                               SEMSEG_CNN_graphParams  *desc)
{
    appCntxt->freeQ.push(desc);
}


void SEMSEG_CNN_enquePreprocInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                      SEMSEG_CNN_graphParams  *desc)
{
    appCntxt->preProcQ.push(desc);
}

void SEMSEG_CNN_enqueDLRInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                  SEMSEG_CNN_graphParams  *desc)
{
    appCntxt->dlrQ.push(desc);
}

void SEMSEG_CNN_enquePostprocInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                       SEMSEG_CNN_graphParams  *desc)
{
    appCntxt->postProcQ.push(desc);
}


void SEMSEG_CNN_enqueOutputDesc(SEMSEG_CNN_Context      *appCntxt,
                                SEMSEG_CNN_graphParams  *desc)
{
    appCntxt->outputQ.push(desc);
}


vx_status SEMSEG_CNN_processEvent(SEMSEG_CNN_Context * appCntxt, vx_event_t * event)
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
                            SEMSEG_CNN_SCALER_NODE_COMPLETE_EVENT;

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
                appPerfPointEnd(&appCntxt->semsegPerf);

                appCntxt->profileEnd = GET_TIME();
                diff  = GET_DIFF(appCntxt->profileStart, appCntxt->profileEnd);
                CM_reportProctime("Inter_frame_interval", diff);
            }

            appPerfPointBegin(&appCntxt->semsegPerf);
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

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Wakeup the pre-process thread. The pre-process thread will
             * process the descriptor at the head of the queue.
             */
            appCntxt->preProcSem->notify();
        }
    }



    return vxStatus;
}


vx_status SEMSEG_CNN_getOutBuff(SEMSEG_CNN_Context   * appCntxt,
                                vx_image             * inputImage,
                                vx_reference         * output,
                                vx_uint64            * timestamp)
{
    vx_status                vxStatus;
    SEMSEG_CNN_graphParams   desc;

    vxStatus = SEMSEG_CNN_getOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->enableLdcNode)
            {
                *inputImage = desc.vxRectifiedImage;
            }
            else
            {
                *inputImage = desc.vxInputImage;
            }

            if (appCntxt->enablePostProcNode)
            {
                *output = (vx_reference)desc.vxScalerOut;
            }
            else
            {
                *output = (vx_reference)desc.vxOutTensor;
            }

            *timestamp = *desc.timestamp;
        }
        else
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_getOutputDesc() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;

}

vx_status SEMSEG_CNN_releaseOutBuff(SEMSEG_CNN_Context * appCntxt)
{
    vx_status                       vxStatus;
    SEMSEG_CNN_graphParams        * desc;

    /* Pop the output descriptor. */
    vxStatus = SEMSEG_CNN_popOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Enqueue the descriptor to the free descriptor queue. */
        SEMSEG_CNN_enqueInputDesc(appCntxt, desc);
    }

    return vxStatus;
}
