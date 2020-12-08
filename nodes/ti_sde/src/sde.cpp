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

#include "perception/utils/sde_rd_wr.h"

#include <app_ptk_demo_common.h>
#include <app_ptk_demo_disparity.h>

#include "sde.h"

#define APP_SDE_NAME    "sde"


void SDEAPP_setLDCCreateParams(SDEAPP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width[0];
    createParams->height           = appCntxt->height[0];
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

    createParams->width            = appCntxt->width[0];
    createParams->height           = appCntxt->height[0];
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->vis_confidence   = appCntxt->confidence_threshold;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
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
    createParams->width              = appCntxt->width[0];
    createParams->height             = appCntxt->height[0];

    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void SDEAPP_setAllParams(SDEAPP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    SDEAPP_setLDCCreateParams(appCntxt);

    /* sinle-layer SDE */
    if (appCntxt->sdeAlgoType == 0)
    {
        SDEAPP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        SDEAPP_setMLSdeCreateParams(appCntxt);
    }
} /* SDEAPP_setAllParams */


int32_t SDEAPP_init(SDEAPP_Context *appCntxt)
{
    int32_t                     status;
    vx_status                   vxStatus;

    status = appInit();
    PTK_assert(status == 0);

    // OpenVX initialization
    appCntxt->vxContext = vxCreateContext();
    PTK_assert(appCntxt->vxContext);

    // Create graph
    appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
    if (appCntxt->vxGraph == NULL)
    {
        PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                    __FUNCTION__, __LINE__);
        status = -1;
    }

    if (status >= 0)
    {
        vxSetReferenceName((vx_reference)appCntxt->vxGraph, "SDE Graph");
    }

    if (status >= 0)
    {
        tivxStereoLoadKernels(appCntxt->vxContext);
        tivxHwaLoadKernels(appCntxt->vxContext);

        /*
         * 1 Setup Stereo LDC nodes
         */
        status = SDEAPP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (status >= 0)
    {
        status = SDEAPP_init_SDE(appCntxt);
    }

    if (status >= 0)
    {
        appPerfPointSetName(&appCntxt->sdePerf , "Stereo GRAPH");

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
                status = -1;
            }
        }
    }

    if (status >= 0)
    {
        // init scaler for ML SDE
        if (appCntxt->sdeAlgoType == 1)
        {
            ML_SDEAPPLIB_initScaler(appCntxt->mlSdeHdl);
        }

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_sde");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }

        SDEAPP_reset(appCntxt);
        appPerfStatsResetAll();
    }

    return status;
}


void SDEAPP_run(SDEAPP_Context *appCntxt, vx_uint8 * inputLeftImage, vx_uint8 * inputRightImage)
{
    int32_t status = 0;
    char     temp[SDEAPP_MAX_LINE_LEN];

    SDEAPP_graphParams* gpDesc;

    status = SDEAPP_getFreeParamRsrc(appCntxt, &gpDesc);
    if (status < 0)
    {
        return;
    }

    ptkdemo_copy_data_to_image(inputLeftImage, gpDesc->vxInputLeftImage);
    ptkdemo_copy_data_to_image(inputRightImage, gpDesc->vxInputRightImage);

    // run the app
    status = SDEAPP_process(appCntxt, gpDesc);
    if (status < 0)
    {
        PTK_printf("[%s:%d] SDEAPP_process() failed\n",
                    __FUNCTION__, __LINE__);
    }
}

void SDEAPP_deInit(SDEAPP_Context *appCntxt)
{
    int32_t  i, status;

    // release input image object
    vxReleaseImage(&appCntxt->vxLeftRectImage[0]);
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

    if (appCntxt->rtLogEnable == 1)
    {
        tivxLogRtTraceDisable(appCntxt->vxGraph);
    }

    vxReleaseGraph(&appCntxt->vxGraph);
    tivxStereoUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    status = appDeInit();
    PTK_assert(status == 0);
}


static void SDEAPP_exitProcThreads(SDEAPP_Context *appCntxt,
                                   bool            detach)
{
    vx_status vxStatus;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               SDEAPP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n", __FUNCTION__,  __LINE__);
    }

    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }
}

void SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt, bool detach)
{
    /* Wait for the threads to exit. */
    SDEAPP_exitProcThreads(appCntxt, detach);

    PTK_printf("\nPress ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    SDEAPP_printStats(appCntxt);
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", APP_SDE_NAME);
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

    SDEAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

void SDEAPP_reset(SDEAPP_Context * appCntxt)
{
    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false;
}

void SDEAPP_intSigHandler(SDEAPP_Context *appCntxt, int sig)
{
    SDEAPP_waitGraph(appCntxt);

    SDEAPP_cleanupHdlr(appCntxt, true);
    exit(0);
}

