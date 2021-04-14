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
 * *       any redistribution and use ar./apps/ptk_demos/app_dof_sfm_fisheye/config/app.cfge licensed by TI for use only with TI Devices.
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

#include "estop.h"

#include <app_ptk_demo_common.h>
#include <app_ptk_demo_disparity.h>


void ESTOP_APP_setPcParams(ESTOP_APP_Context *appCntxt);
void ESTOP_APP_setOgParams(ESTOP_APP_Context *appCntxt);


static char menu[] = {
    "\n"
    "\n ======================================================"
    "\n Demo : AMR E-STOP                                     "
    "\n ======================================================"
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};


void ESTOP_APP_setLDCCreateParams(ESTOP_APP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}

void ESTOP_APP_setSLSdeCreateParams(ESTOP_APP_Context *appCntxt)
{
    SL_SDEAPPLIB_createParams * createParams = &appCntxt->slSdeCreateParams;
    createParams->sdeCfg = appCntxt->sde_params;

     if (appCntxt->sde_params.disparity_min == 0)
    {
        createParams->minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        createParams->minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        createParams->maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        createParams->maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        createParams->maxDisparity = 191;
    }

    createParams->width              = appCntxt->width;
    createParams->height             = appCntxt->height;
    createParams->inputFormat        = appCntxt->inputFormat;
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void ESTOP_APP_setMLSdeCreateParams(ESTOP_APP_Context *appCntxt)
{
    ML_SDEAPPLIB_createParams * createParams = &appCntxt->mlSdeCreateParams;
    createParams->sdeCfg = appCntxt->sde_params;

    if (appCntxt->sde_params.disparity_min == 0)
    {
        createParams->minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        createParams->minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        createParams->maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        createParams->maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        createParams->maxDisparity = 191;
    }

    createParams->inputFormat        = appCntxt->inputFormat;
    createParams->numLayers          = appCntxt->numLayers;
    createParams->enableMedianFilter = appCntxt->ppMedianFilterEnable;
    createParams->width              = appCntxt->width;
    createParams->height             = appCntxt->height;
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void ESTOP_APP_setSSDetectCreateParams(ESTOP_APP_Context *appCntxt)
{
    SS_DETECT_APPLIB_createParams * createParams = &appCntxt->ssDetectCreateParams;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->tensorWidth      = appCntxt->tensor_width;
    createParams->tensorHeight     = appCntxt->tensor_height;

    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->exportGraph      = appCntxt->exportGraph;
    createParams->rtLogEnable      = appCntxt->rtLogEnable;
    createParams->vxEvtAppValBase  = 0;

    ESTOP_APP_setPcParams(appCntxt);
    ESTOP_APP_setOgParams(appCntxt);
}


void ESTOP_APP_setPcParams(ESTOP_APP_Context *appCntxt)
{
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.width        = appCntxt->width;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.height       = appCntxt->height;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.tensorWidth  = appCntxt->tensor_width;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.tensorHeight = appCntxt->tensor_height;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsFactor     = 4;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.confidenceTh = appCntxt->confidence_threshold;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsWidth      =
        appCntxt->ssDetectCreateParams.pcCfg.cfgParams.width / appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsFactor;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsHeight     =
        appCntxt->ssDetectCreateParams.pcCfg.cfgParams.height / appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsFactor;

    appCntxt->ssDetectCreateParams.pcCfg.camParams.camHeight    = appCntxt->camHeight;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.camRoll      = appCntxt->camRoll;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.camPitch     = appCntxt->camPitch;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.camYaw       = appCntxt->camYaw;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.sinPitch     = sin(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.pcCfg.camParams.cosPitch     = cos(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.pcCfg.camParams.baseline     = appCntxt->baseline;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.dcx          = appCntxt->distCenterX;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.dcy          = appCntxt->distCenterY;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.f            = appCntxt->focalLength;
}


void ESTOP_APP_setOgParams(ESTOP_APP_Context *appCntxt)
{
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.width                      = appCntxt->width;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.height                     = appCntxt->height;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.enableSpatialObjMerge      = appCntxt->enableSpatialObjMerge;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.enableTemporalObjMerge     = appCntxt->enableTemporalObjMerge;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.enableTemporalObjSmoothing = appCntxt->enableTemporalObjSmoothing;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.objectDistanceMode         = appCntxt->objectDistanceMode;

    appCntxt->ssDetectCreateParams.ogCfg.camParams.camHeight    = appCntxt->camHeight;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.camRoll      = appCntxt->camRoll;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.camPitch     = appCntxt->camPitch;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.camYaw       = appCntxt->camYaw;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.sinPitch     = sin(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.ogCfg.camParams.cosPitch     = cos(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.ogCfg.camParams.baseline     = appCntxt->baseline;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.dcx          = appCntxt->distCenterX;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.dcy          = appCntxt->distCenterY;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.f            = appCntxt->focalLength;

    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xGridSize     = appCntxt->xGridSize;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yGridSize     = appCntxt->yGridSize;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xMinRange     = appCntxt->xMinRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xMaxRange     = appCntxt->xMaxRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yMinRange     = appCntxt->yMinRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yMaxRange     = appCntxt->yMaxRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xGridNum      = appCntxt->xGridNum;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yGridNum      = appCntxt->yGridNum;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.thCnt         = appCntxt->thCnt;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.thObjCnt      = appCntxt->thObjCnt;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.maxNumObject  = appCntxt->maxNumObject;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.cNeighNum     = appCntxt->cNeighNum;
}

void ESTOP_APP_setAllParams(ESTOP_APP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    /* LDC params */
    ESTOP_APP_setLDCCreateParams(appCntxt);

    /* SDE params */
    if (appCntxt->sdeAlgoType == 0)
    {
        ESTOP_APP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ESTOP_APP_setMLSdeCreateParams(appCntxt);
    }

    ESTOP_APP_setSSDetectCreateParams(appCntxt);
}


vx_status ESTOP_APP_init(ESTOP_APP_Context *appCntxt)
{
    int32_t   status;
    vx_status vxStatus = VX_SUCCESS;

    // Create the DLR Model handle
    CM_DLRCreateParams  params;

    params.modelPath = appCntxt->dlrModelPath;
    params.devType   = DLR_DEVTYPE_CPU;
    params.devId     = 0;

    status = CM_dlrNodeCntxtInit(&appCntxt->dlrObj, &params);
    if (status < 0)
    {
        PTK_printf("[%s:%d] CM_dlrNodeCntxtInit() failed.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

#if 0
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        status = appInit();
        if (status < 0)
        {
            PTK_printf("[%s:%d] appInit() failed.\n",
                       __FUNCTION__, __LINE__);
    
            vxStatus = VX_FAILURE;
        }
    }
#endif

    // OpenVX initialization
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->vxContext = vxCreateContext();
        if (appCntxt->vxContext == NULL)
        {
            PTK_printf("[%s:%d] vxCreateContext() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    // create graph 
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
        if (appCntxt->vxGraph == NULL)
        {
            PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
        } else 
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph, "AMR E-Stop Graph");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* load TILDL kernels */
        //tivxTIDLLoadKernels(appCntxt->vxContext);

        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* Load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Load stereo kernels */
        tivxStereoLoadKernels(appCntxt->vxContext);
    }

    /*
     * 1 Setup Stereo LDC nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_SDE(appCntxt);
    }

    /*
     * 3 Setup Semantic Segmentation nodes 
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_SS(appCntxt);
    }

    /*
     * 4 Setup OG map based detection nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_SS_Detection(appCntxt);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->estopPerf , "Emergency Stop GRAPH");

        /*
         * set up the pipeline. 
         */
        vxStatus = ESTOP_APP_setupPipeline(appCntxt);

        /* Verify graph */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxStatus = vxVerifyGraph(appCntxt->vxGraph);
            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxVerifyGraph() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        /* Set the MSC coefficients. */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            CM_ScalerNodeCntxt  *scalerObj = &appCntxt->scalerObj;
            vxStatus = CM_scalerNodeCntxtSetCoeff(scalerObj);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_setCoeff() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }
    } else
    {
        vxStatus == (vx_status)VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // init sacler for ML SDE
        if (appCntxt->sdeAlgoType == 1)
        {
            ML_SDEAPPLIB_initScaler(appCntxt->mlSdeHdl);
        }

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_estop");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }


        appCntxt->exitOutputThread     = false;
        appCntxt->outputCtrlSem        = new UTILS::Semaphore(0);

        appCntxt->preProcSem           = new UTILS::Semaphore(0);
        appCntxt->exitPreprocThread    = false;

        appCntxt->dlrDataReadySem      = new UTILS::Semaphore(0);
        appCntxt->exitDlrThread        = false;

        appCntxt->postProcSem          = new UTILS::Semaphore(0);
        appCntxt->exitPostprocThread   = false;

        appCntxt->exitInputDataProcess = false;
        appCntxt->state                = ESTOP_APP_STATE_INIT;

        ESTOP_APP_reset(appCntxt);
    }

    return vxStatus;
}

vx_status ESTOP_APP_init_camInfo(ESTOP_APP_Context *appCntxt, 
                                 uint32_t width,
                                 uint32_t height,
                                 double   f,
                                 double   dx,
                                 double   dy)
{
    appCntxt->width       = (uint16_t) width;
    appCntxt->height      = (uint16_t) height;
    appCntxt->focalLength = (float)f;
    appCntxt->distCenterX = (float)dx;
    appCntxt->distCenterY = (float)dy;

    return (vx_status)VX_SUCCESS;
}

vx_status ESTOP_APP_run(ESTOP_APP_Context *appCntxt, 
                        const vx_uint8 * inputLeftImage, const vx_uint8 * inputRightImage, 
                        vx_uint64 timestamp)
{
    uint32_t  i;
    vx_status vxStatus;

    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    ESTOP_APP_graphParams* gpDesc;

    vxStatus = ESTOP_APP_popFreeInputDesc(appCntxt, &gpDesc);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] ESTOP_APP_popFreeInputDesc() failed\n",
                    __FUNCTION__, __LINE__);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        start = GET_TIME();
        vxStatus = ptkdemo_copy_data_to_image(inputLeftImage, gpDesc->vxInputLeftImage);

        end   = GET_TIME();
        diff  = GET_DIFF(start, end);
        CM_reportProctime("input_image_load", diff);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_copy_data_to_image() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_copy_data_to_image(inputRightImage, gpDesc->vxInputRightImage);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_copy_data_to_image() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    // then run AMR applib
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // set time stamp
        *(gpDesc->timestamp) = timestamp;

        vxStatus = ESTOP_APP_process(appCntxt, gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ESTOP_APP_process() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

void ESTOP_APP_deInit(ESTOP_APP_Context *appCntxt)
{
    int32_t status;
    uint8_t i;

    // release input image object
    vxReleaseImage(&appCntxt->vxLeftRectImage);
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        vxReleaseImage(&appCntxt->vxInputLeftImage[i]);
        vxReleaseImage(&appCntxt->vxInputRightImage[i]);
        vxReleaseImage(&appCntxt->vxRightRectImage[i]);
    }

    // release dispairty object
    if (appCntxt->sdeAlgoType == 0)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxSde16BitOutput[i]);
        }
    } else
    {
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                vxReleaseImage(&appCntxt->vxMedianFilteredDisparity[i]);
            }
        }

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxMergeDisparityL0[i]);
        }
    }

    // release graph
    vxReleaseGraph(&appCntxt->vxGraph);

    // Unload HWA kernels
    tivxStereoUnLoadKernels(appCntxt->vxContext);
    tivxImgProcUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    /* Delete the DLR Model handle. */
    status = CM_dlrNodeCntxtDeInit(&appCntxt->dlrObj);

    if (status < 0)
    {
        PTK_printf("[%s:%d] CM_dlrNodeCntxtDeInit() failed.\n",
                    __FUNCTION__, __LINE__);
    }

#if 0
    status = appDeInit();
    PTK_assert(status == 0);
#endif
}

static void ESTOP_APP_exitProcThreads(ESTOP_APP_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               ESTOP_APP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n");
    }

    /* Set the exit flag for the pre-process thread. */
    appCntxt->exitPreprocThread = true;

    /* Wake-up the pre-process thread. */
    if (appCntxt->preProcSem)
    {
        appCntxt->preProcSem->notify();
    }

    /* Set the exit flag for the DLR thread. */
    appCntxt->exitDlrThread = true;

    /* Wake-up the DLR thread. */
    if (appCntxt->dlrDataReadySem)
    {
        appCntxt->dlrDataReadySem->notify();
    }

    /* Set the exit flag for the post-process thread. */
    appCntxt->exitPostprocThread = true;

    /* Wake-up the post-process thread. */
    if (appCntxt->postProcSem)
    {
        appCntxt->postProcSem->notify();
    }

    /* Let the display thread exit. */
    appCntxt->exitOutputThread = true;

    if (appCntxt->outputCtrlSem)
    {
        appCntxt->outputCtrlSem->notify();
    }


    /* Wait for the thread exit */
    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }

    if (appCntxt->preProcThread.joinable())
    {
        appCntxt->preProcThread.join();
    }

    if (appCntxt->dlrThread.joinable())
    {
        appCntxt->dlrThread.join();
    }

    if (appCntxt->postProcThread.joinable())
    {
        appCntxt->postProcThread.join();
    }


    /* Delete the semaphores. */
    if (appCntxt->outputCtrlSem)
    {
        delete appCntxt->outputCtrlSem;
    }

    if (appCntxt->preProcSem)
    {
        delete appCntxt->preProcSem;
    }

    if (appCntxt->dlrDataReadySem)
    {
        delete appCntxt->dlrDataReadySem;
    }

    if (appCntxt->postProcSem)
    {
        delete appCntxt->postProcSem;
    }

}

void ESTOP_APP_cleanupHdlr(ESTOP_APP_Context *appCntxt)
{
    if (appCntxt->state == ESTOP_APP_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = ESTOP_APP_STATE_INVALID;

    /* Wait for the threads to exit. */
    ESTOP_APP_exitProcThreads(appCntxt);

    PTK_printf("\nPress ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    ESTOP_APP_printStats(appCntxt);

    CM_printProctime();
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", ESTOP_APP_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the objects. */
    SDELDCAPPLIB_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_delete(&appCntxt->mlSdeHdl);
    }

    SS_DETECT_APPLIB_delete(&appCntxt->ssDetectHdl);

    ESTOP_APP_deinit_SS(appCntxt);

    ESTOP_APP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

void ESTOP_APP_reset(ESTOP_APP_Context * appCntxt)
{
    vx_status vxStatus;

    vx_node ogNode = SS_DETECT_APPLIB_getOGNode(appCntxt->ssDetectHdl);
    vxStatus = tivxNodeSendCommand(ogNode,
                                   0,
                                   TIVX_KERNEL_OCCUPANCY_GRID_DETECTION_RESET,
                                   NULL,
                                   0);
    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivxNodeSendCommand() failed.\n",
                   __FUNCTION__,
                   __LINE__);

    }

    appCntxt->startPerfCapt = false;
}

static void ESTOP_APP_evtHdlrThread(ESTOP_APP_Context *appCntxt)
{
    vx_event_t evt;
    vx_status vxStatus = VX_SUCCESS;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_false_e);

        if (vxStatus == VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == ESTOP_APP_USER_EVT_EXIT)
                {
                    break;
                }
                else if (evt.app_value == ESTOP_APP_CNN_OUT_AVAIL_EVENT)
                {
                    ESTOP_APP_graphParams * desc;
                    vxStatus = ESTOP_APP_getPostprocInputDesc(appCntxt, &desc);

                    // enqueue vxOutTensor
                    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                               appCntxt->numGraphParams - 1,
                                                               (vx_reference*)&desc->vxOutTensor,
                                                               1);
                    if (vxStatus != (vx_status)VX_SUCCESS)
                    {
                        PTK_printf("[%s:%d] vxGraphParameterEnqueueReadyRef(%d) "
                                   "failed\n", __FUNCTION__, __LINE__, appCntxt->numGraphParams - 1);
                     
                    }
                }
            }

            if (evt.type == VX_EVENT_GRAPH_COMPLETED || evt.type == VX_EVENT_NODE_COMPLETED)
            {
                ESTOP_APP_processEvent(appCntxt, &evt);
            }
        }

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}


static int32_t ESTOP_APP_userControlThread(ESTOP_APP_Context *appCntxt)
{
    int32_t             status;
    vx_status           vxStatus = VX_SUCCESS;

    uint32_t done = 0;

    appPerfStatsResetAll();

    while (!done)
    {
        char ch;

        PTK_printf(menu);
        ch = getchar();
        PTK_printf("\n");

        switch (ch)
        {
            case 'p':
                appPerfStatsPrintAll();
                ESTOP_APP_printStats(appCntxt);

                CM_printProctime();
                CM_resetProctime();
                break;

            case 'e':
            {
                FILE *fp;
                const char *name = ESTOP_APP_PERF_OUT_FILE;

                fp = appPerfStatsExportOpenFile(".", (char *)name);

                if (fp != NULL)
                {
                    ESTOP_APP_exportStats(appCntxt, fp, true);

                    appPerfStatsExportCloseFile(fp);
                }
                else
                {
                    PTK_printf("Could not open [%s] for exporting "
                               "performance data\n", name);
                }
            }
            break;

            case 'x':
                done = 1;
                break;
        } // switch(ch)

        /* Consume the newline character. */
        if (ch != '\n')
        {
            getchar();
        }
    } // while (!done)

    appCntxt->state = ESTOP_APP_STATE_SHUTDOWN;
    PTK_printf("[%s:%d] Waiting for the graph to finish.\n",
                __FUNCTION__, __LINE__);

    vxStatus = ESTOP_APP_waitGraph(appCntxt);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] ESTOP_APP_waitGraph() failed\n",
                    __FUNCTION__, __LINE__);
    }

    ESTOP_APP_cleanupHdlr(appCntxt);

    PTK_printf("\nDEMO FINISHED!\n");

    return vxStatus;
}

static void ESTOP_APP_preProcThread(ESTOP_APP_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        ESTOP_APP_graphParams  *desc;
        vx_status               vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->preProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPreprocThread == true)
        {
            break;
        }

        vxStatus = ESTOP_APP_popPreprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus =
                ESTOP_APP_CNN_preProcess(appCntxt,
                                        desc->vxScalerOut,
                                        desc->dlrInputBuff);

            end = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Preprocessing", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] ESTOP_APP_CNN_preProcess() "
                           "failed.\n",
                            __FUNCTION__, __LINE__);
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                ESTOP_APP_enqueDLRInputDesc(appCntxt, desc);

                /* Wakeup the DLR thread. The DLR thread will process the
                 * descriptor at the head of the queue.
                 */
                appCntxt->dlrDataReadySem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}


static void ESTOP_APP_dlrThread(ESTOP_APP_Context *appCntxt)
{
    CM_DLRNodeCntxt         *dlrObj;
    CM_DLRNodeInputInfo     *dlrInput;
    CM_DLRNodeOutputInfo    *dlrOutput;

    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    dlrObj       = &appCntxt->dlrObj;
    dlrInput     = &dlrObj->input;
    dlrOutput    = &dlrObj->output;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        ESTOP_APP_graphParams  *desc;
        vx_status               vxStatus;

        /* Wait fot the input buffer. */
        appCntxt->dlrDataReadySem->wait();

        if (appCntxt->exitDlrThread == true)
        {
            break;
        }

        vxStatus = ESTOP_APP_popDLRInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            int32_t status;

            dlrInput->info[0].data  = desc->dlrInputBuff;
            dlrOutput->info[0].data = desc->dlrOutputBuff;

            start = GET_TIME();
            status = CM_dlrNodeCntxtProcess(dlrObj, dlrInput, dlrOutput);

            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("DLR-node", diff);

            if (status < 0)
            {
                PTK_printf("[%s:%d] CM_dlrNodeCntxtProcess() failed.\n",
                            __FUNCTION__, __LINE__);
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                ESTOP_APP_enquePostprocInputDesc(appCntxt, desc);

                /* Wakeup the post-process thread. The post-process thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->postProcSem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);

}

static void ESTOP_APP_postProcThread(ESTOP_APP_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        ESTOP_APP_graphParams  *desc;
        vx_status               vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->postProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPostprocThread == true)
        {
            break;
        }

        vxStatus = ESTOP_APP_getPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->enablePostProcNode)
            {
                /* Create the output object for display. */
                start = GET_TIME();
                vxStatus =
                    ESTOP_APP_CNN_postProcess(appCntxt,
                                              desc->vxScalerOut,
                                              desc->dlrOutputBuff);

                end   = GET_TIME();
                diff  = GET_DIFF(start, end);
                CM_reportProctime("Postprocess", diff);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] SEMSEG_CNN_postProcess() "
                               "failed.\n",
                                __FUNCTION__, __LINE__);
                }
            }

            // we should create output tensor always
            {
                /* Create an output tensor. */
                start = GET_TIME();
                vxStatus =
                    ESTOP_APP_createOutTensor(appCntxt, 
                                              desc->vxOutTensor,
                                              desc->dlrOutputBuff);

                end   = GET_TIME();
                diff  = GET_DIFF(start, end);
                CM_reportProctime("Output_tensor_creation", diff);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] SEMSEG_CNN_createOutTensor() "
                               "failed.\n",
                                __FUNCTION__, __LINE__);
                }
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Let the usr know that output is ready. */
                vxStatus =
                    vxSendUserEvent(appCntxt->vxContext,
                                    ESTOP_APP_CNN_OUT_AVAIL_EVENT,
                                    NULL);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxSendUserEvent() failed.\n",
                                __FUNCTION__, __LINE__);
                }
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}


void ESTOP_APP_launchProcThreads(ESTOP_APP_Context *appCntxt)
{
    /* Launch the input data thread. */
    if (appCntxt->is_interactive)
    {
        appCntxt->userCtrlThread = 
            std::thread(ESTOP_APP_userControlThread, appCntxt);

        appCntxt->userCtrlThread.detach();
    }

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread = 
        std::thread(ESTOP_APP_evtHdlrThread, appCntxt);

    /* Launch CNN pre-processing thread */
    appCntxt->preProcThread =
        std::thread(ESTOP_APP_preProcThread, appCntxt);

    /* Launch the dlr thread */
    appCntxt->dlrThread = 
        std::thread(ESTOP_APP_dlrThread, appCntxt);

    /* launch post-processing thread. */
    appCntxt->postProcThread =
        std::thread(ESTOP_APP_postProcThread, appCntxt);
}

void ESTOP_APP_intSigHandler(ESTOP_APP_Context *appCntxt)
{
    if (appCntxt->state != ESTOP_APP_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = ESTOP_APP_STATE_SHUTDOWN;
        PTK_printf("[%s:%d] Waiting for the graph to finish.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = ESTOP_APP_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ESTOP_APP_waitGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }

        ESTOP_APP_cleanupHdlr(appCntxt);
        PTK_printf("\nDEMO FINISHED!\n");
    }

    exit(0);
}
