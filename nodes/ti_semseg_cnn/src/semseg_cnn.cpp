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
#include <semseg_cnn.h>

static char menu[] = {
    "\n"
    "\n =========================================="
    "\n Demo : Sematic Segmentation  Network      "
    "\n =========================================="
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};

#if !defined(PC)
void SEMSEG_CNN_drawGraphics(
        Draw2D_Handle *handle,
        Draw2D_BufInfo *draw2dBufInfo,
        uint32_t update_type)
{
    appGrpxDrawDefault(handle, draw2dBufInfo, update_type);

    return;
}
#endif

vx_status SEMSEG_CNN_init(SEMSEG_CNN_Context *appCntxt)
{
    SEMSEG_CNN_APPLIB_createParams *createParams;
    int32_t                         status;
    vx_status                       vxStatus;

    vxStatus     = VX_SUCCESS;
    createParams = &appCntxt->createParams;

    status = appInit();

    if (status < 0)
    {
        PTK_printf("[%s:%d] appInit() failed.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

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

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // create objects
        appCntxt->vxInputImage = vxCreateImage(appCntxt->vxContext,
                                               createParams->inputImageWidth,
                                               createParams->inputImageHeight,
                                               VX_DF_IMAGE_UYVY);

        if (appCntxt->vxInputImage == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxInputImage,
                               "InputImage");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* load TILDL kernels */
        tivxTIDLLoadKernels(appCntxt->vxContext);

        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* create SDE CNNPP Applib */
        createParams->vxContext = appCntxt->vxContext;
        createParams->vxGraph   = NULL;

        appCntxt->sscnnHdl = SEMSEG_CNN_APPLIB_create(createParams);

        if (appCntxt->sscnnHdl == NULL)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_create() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appCntxt->outputCtrlSem = new UTILS::Semaphore(0);

            appCntxt->exitInputDataProcess = 0;
            appCntxt->state = SEMSEG_CNN_STATE_INIT;
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_processImage(SEMSEG_CNN_Context   *appCntxt,
                                  const unsigned char  *inputImage)
{
    uint32_t    width;
    uint32_t    height;
    vx_status   vxStatus;

    width  = appCntxt->createParams.inputImageWidth;
    height = appCntxt->createParams.inputImageHeight;

    vxStatus = ptkdemo_copy_data_to_image(inputImage, appCntxt->vxInputImage);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] ptkdemo_copy_data_to_image() failed\n",
                    __FUNCTION__, __LINE__);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = SEMSEG_CNN_APPLIB_process(appCntxt->sscnnHdl,
                                             appCntxt->vxInputImage);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_process() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

void SEMSEG_CNN_deInit(SEMSEG_CNN_Context *appCntxt)
{
    if (appCntxt->outputCtrlSem)
    {
        delete appCntxt->outputCtrlSem;
    }

    if (appCntxt->vxInputImage != NULL)
    {
        vxReleaseImage(&appCntxt->vxInputImage);
    }

    tivxTIDLUnLoadKernels(appCntxt->vxContext);
    tivxImgProcUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    appDeInit();
}

static void SEMSEG_CNN_exitProcThreads(SEMSEG_CNN_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               SEMSEG_CNN_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != (vx_status)VX_SUCCESS)
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

void SEMSEG_CNN_cleanupHdlr(SEMSEG_CNN_Context *appCntxt)
{
    if (appCntxt->state == SEMSEG_CNN_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = SEMSEG_CNN_STATE_INVALID;

    /* Wait for the threads to exit. */
    SEMSEG_CNN_exitProcThreads(appCntxt);

    PTK_printf("\nPress ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", SEMSEG_CNN_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the Application context. */
    SEMSEG_CNN_APPLIB_delete(&appCntxt->sscnnHdl);

    SEMSEG_CNN_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

void SEMSEG_CNN_reset(SEMSEG_CNN_Context * appCntxt)
{
    int32_t status;

    status = SEMSEG_CNN_APPLIB_reset(appCntxt->sscnnHdl);

    if (status < 0)
    {
        PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_reset() failed.\n",
                   __FUNCTION__,
                   __LINE__);
    }
}

static void SEMSEG_CNN_evtHdlrThread(SEMSEG_CNN_Context *appCntxt)
{
    vx_event_t  evt;
    vx_status   vxStatus;

    vxStatus = VX_SUCCESS;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_false_e);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == SEMSEG_CNN_USER_EVT_EXIT)
                {
                    break;
                }
            }

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                vxStatus =
                    SEMSEG_CNN_APPLIB_processEvent(appCntxt->sscnnHdl, &evt);

                /* Wakeup the display thread. */
                if (appCntxt->outputCtrlSem)
                {
                    appCntxt->outputCtrlSem->notify();
                }

            } // if (evt.type == VX_EVENT_GRAPH_COMPLETED)

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

static int32_t SEMSEG_CNN_userControlThread(SEMSEG_CNN_Context *appCntxt)
{
    int32_t     status;
    vx_status   vxStatus = VX_SUCCESS;
    uint32_t    done = 0;

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
                SEMSEG_CNN_APPLIB_printStats(appCntxt->sscnnHdl);
                break;

                case 'e':
                {
                    FILE *fp;
                    const char *name = SEMSEG_CNN_PERF_OUT_FILE;

                    fp = appPerfStatsExportOpenFile(".", (char *)name);

                    if (fp != NULL)
                    {
                        SEMSEG_CNN_APPLIB_exportStats(appCntxt->sscnnHdl,
                                                      fp,
                                                      true);

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

    appCntxt->state = SEMSEG_CNN_STATE_SHUTDOWN;
    PTK_printf("[%s:%d] Waiting for the graph to finish.\n",
                __FUNCTION__, __LINE__);

    vxStatus = SEMSEG_CNN_APPLIB_waitGraph(appCntxt->sscnnHdl);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_waitGraph() failed\n",
                    __FUNCTION__, __LINE__);
    }

    SEMSEG_CNN_cleanupHdlr(appCntxt);

    PTK_printf("\nDEMO FINISHED!\n");

    return vxStatus;
}

void SEMSEG_CNN_launchProcThreads(SEMSEG_CNN_Context *appCntxt)
{
    /* Launch the graph processing thread. */
    if (appCntxt->is_interactive)
    {
        appCntxt->userCtrlThread =
            std::thread(SEMSEG_CNN_userControlThread, appCntxt);

        appCntxt->userCtrlThread.detach();
    }

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread =
        std::thread(SEMSEG_CNN_evtHdlrThread, appCntxt);
}

void SEMSEG_CNN_intSigHandler(SEMSEG_CNN_Context *appCntxt)
{
    if (appCntxt->state != SEMSEG_CNN_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        SEMSEG_CNN_exitProcThreads(appCntxt);
    }

    exit(0);
}

