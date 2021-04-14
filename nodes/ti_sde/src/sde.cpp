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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "sde.h"

#include <app_ptk_demo_common.h>
#include <app_ptk_demo_disparity.h>


static char menu[] = {
    "\n"
    "\n ======================================================"
    "\n Demo : SDE & Point Cloud                                "
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


void SDEAPP_setLDCCreateParams(SDEAPP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}

void SDEAPP_setSLSdeCreateParams(SDEAPP_Context *appCntxt)
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

void SDEAPP_setMLSdeCreateParams(SDEAPP_Context *appCntxt)
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


void SDEAPP_setSdeTriangCreateParams(SDEAPP_Context *appCntxt)
{
    SDE_TRIANG_APPLIB_createParams * createParams = &appCntxt->sdeTriangCreateParams;

    createParams->width                      = appCntxt->width;
    createParams->height                     = appCntxt->height;
    createParams->pipelineDepth              = appCntxt->pipelineDepth;

    createParams->stereoPcCfg.usePCConfig    = appCntxt->usePCConfig;
    createParams->stereoPcCfg.subSampleRatio = appCntxt->pcSubsampleRatio;
    createParams->stereoPcCfg.thConfidence   = appCntxt->dispConfidence;
    createParams->stereoPcCfg.lowLimitX      = appCntxt->lowPtX;
    createParams->stereoPcCfg.highLimitX     = appCntxt->highPtX;
    createParams->stereoPcCfg.lowLimitY      = appCntxt->lowPtY;
    createParams->stereoPcCfg.highLimitY     = appCntxt->highPtY;
    createParams->stereoPcCfg.lowLimitZ      = appCntxt->lowPtZ;
    createParams->stereoPcCfg.highLimitZ     = appCntxt->highPtZ;

    createParams->stereoCamCfg.baseline      = appCntxt->baseline;
    createParams->stereoCamCfg.scale_x       = 1;
    createParams->stereoCamCfg.scale_y       = 1;
    createParams->stereoCamCfg.focal_length  = appCntxt->focalLength;
    createParams->stereoCamCfg.dist_center_x = appCntxt->distCenterX;
    createParams->stereoCamCfg.dist_center_y = appCntxt->distCenterY;
}

void SDEAPP_setAllParams(SDEAPP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    /* LDC params */
    SDEAPP_setLDCCreateParams(appCntxt);

    /* SDE params */
    if (appCntxt->sdeAlgoType == 0)
    {
        SDEAPP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        SDEAPP_setMLSdeCreateParams(appCntxt);
    }

    /* SDE Triangulation params */
    if (appCntxt->enablePC)
    {
        SDEAPP_setSdeTriangCreateParams(appCntxt);
    }
}


vx_status SDEAPP_init(SDEAPP_Context *appCntxt)
{
    int32_t   status;
    vx_status vxStatus = VX_SUCCESS;

    status = appInit();
    if (status < 0)
    {
        PTK_printf("[%s:%d] appInit() failed.\n",
                   __FUNCTION__, __LINE__);
        vxStatus = VX_FAILURE;
    }

    // OpenVX initialization
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appCntxt->vxContext = vxCreateContext();
        if (appCntxt->vxContext == NULL)
        {
            PTK_printf("[%s:%d] vxCreateContext() failed.\n",
                       __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
        }
    }

    // Create graph 
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
        if (appCntxt->vxGraph == NULL)
        {
            PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                        __FUNCTION__, __LINE__);
            vxStatus = VX_FAILURE;
        } else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph, "SDE Graph");
        }
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
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
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        vxStatus = SDEAPP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        vxStatus = SDEAPP_init_SDE(appCntxt); 
    }

    /*
     * 3 Setup SDE Triangulation nodes 
     */
    if (appCntxt->enablePC)
    {
        if (vxStatus == (vx_status) VX_SUCCESS)
        {
            vxStatus = SDEAPP_init_SDE_Triang(appCntxt);
        }
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->sdePclPerf, "Stereo GRAPH");

        /*
         * set up the pipeline. 
         */
        vxStatus = SDEAPP_setupPipeline(appCntxt);

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
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_sde_pcl");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }

        appCntxt->exitInputDataProcess = false;
        appCntxt->outputCtrlSem = new Semaphore(0);
        appCntxt->state = SDEAPP_STATE_INIT;
    }

    return vxStatus;
}

vx_status SDEAPP_init_camInfo(SDEAPP_Context *appCntxt, 
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


vx_status SDEAPP_run(SDEAPP_Context *appCntxt, 
                     const vx_uint8 * inputLeftImage, 
                     const vx_uint8 * inputRightImage, 
                     vx_uint64 timestamp)
{
    vx_status vxStatus;

    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    SDEAPP_graphParams* gpDesc;

    vxStatus = SDEAPP_getFreeParamRsrc(appCntxt, &gpDesc);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] SDEAPP_getFreeParamRsrc() failed\n",
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

    // then run app
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // set time stamp
       *(gpDesc->timestamp) = timestamp;

        vxStatus = SDEAPP_process(appCntxt, gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SDEAPP_process() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

void SDEAPP_deInit(SDEAPP_Context *appCntxt)
{
    int32_t status;
    uint8_t i;


    if (appCntxt->outputCtrlSem)
    {
        delete appCntxt->outputCtrlSem;
    }

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

    // release output point cloud object
    if (appCntxt->enablePC)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseUserDataObject(&appCntxt->vxOutputTriangPC[i]);
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

    status = appDeInit();
    PTK_assert(status == 0);
}

static void SDEAPP_exitProcThreads(SDEAPP_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               SDEAPP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n");
    }

    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }

    /* Let the display thread exit. */
    appCntxt->exitOutputThread = true;

    if (appCntxt->outputCtrlSem)
    {
        appCntxt->outputCtrlSem->notify();
    }

}

void SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt)
{
    if (appCntxt->state == SDEAPP_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = SDEAPP_STATE_INVALID;

    /* Wait for the threads to exit. */
    SDEAPP_exitProcThreads(appCntxt);

    PTK_printf("\nPress ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    SDEAPP_printStats(appCntxt);

    CM_printProctime();
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");    

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", SDEAPP_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the Application context. */
    SDELDCAPPLIB_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_delete(&appCntxt->mlSdeHdl);
    }

    SDE_TRIANG_APPLIB_delete(&appCntxt->sdeTriangHdl);


    SDEAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

static void SDEAPP_reset(SDEAPP_Context * appCntxt)
{
    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false;
}

static void SDEAPP_evtHdlrThread(SDEAPP_Context *appCntxt)
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
                if (evt.app_value == SDEAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                SDEAPP_processEvent(appCntxt, &evt);

                /* Wakeup the display thread. */
                if (appCntxt->outputCtrlSem)
                {
                    appCntxt->outputCtrlSem->notify();
                }
            }
        }

    } // while (true)
}

static int32_t SDEAPP_userControlThread(SDEAPP_Context *appCntxt)
{
    int32_t             status;
    vx_status           vxStatus = VX_SUCCESS;

    if (appCntxt->is_interactive)
    {
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
                    SDEAPP_printStats(appCntxt);

                    CM_printProctime();
                    CM_resetProctime();
                    break;

                case 'e':
                {
                    FILE *fp;
                    const char *name = SDEAPP_PERF_OUT_FILE;

                    fp = appPerfStatsExportOpenFile(".", (char *)name);

                    if (fp != NULL)
                    {
                        SDEAPP_exportStats(appCntxt, fp, true);

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

        appCntxt->state = SDEAPP_STATE_SHUTDOWN;
        PTK_printf("[%s:%d] Waiting for the graph to finish.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = SDEAPP_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SDEAPP_waitGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }

        SDEAPP_cleanupHdlr(appCntxt);

        PTK_printf("\nDEMO FINISHED!\n");
    } // if (appCntxt->is_interactive)

    return vxStatus;
}


void SDEAPP_launchProcThreads(SDEAPP_Context *appCntxt)
{
    /* Launch the input data thread. */
    appCntxt->userCtrlThread = std::thread(SDEAPP_userControlThread, appCntxt);

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread = std::thread(SDEAPP_evtHdlrThread, appCntxt);

    appCntxt->userCtrlThread.detach();
}

void SDEAPP_intSigHandler(SDEAPP_Context *appCntxt)
{
    if (appCntxt->state != SDEAPP_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = SDEAPP_STATE_SHUTDOWN;
        PTK_printf("[%s:%d] Waiting for the graph to finish.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = SDEAPP_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SDEAPP_waitGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }

        SDEAPP_cleanupHdlr(appCntxt);
        PTK_printf("\nDEMO FINISHED!\n");
    }

    exit(0);
}
