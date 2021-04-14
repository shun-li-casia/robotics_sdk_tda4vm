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

#include <semseg_cnn.h>

static char menu[] = {
    "\n"
    "\n =========================================="
    "\n Demo : Sematic Segmentation  Network      "
    "\n =========================================="
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};


vx_status SEMSEG_CNN_init(SEMSEG_CNN_Context *appCntxt)
{
    CM_DLRCreateParams params;
    int32_t            status;
    vx_status          vxStatus;

    vxStatus         = VX_SUCCESS;

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
    status = appInit();

    if (status < 0)
    {
        PTK_printf("[%s:%d] appInit() failed.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
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
            vxSetReferenceName((vx_reference)appCntxt->vxGraph, 
                               "CNN Semantic Segmentation Graph");
        }
    }


    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* load TILDL kernels */
        //tivxTIDLLoadKernels(appCntxt->vxContext);

        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Create CNN semantic semgentation nodes */
        vxStatus = SEMSEG_CNN_init_SS(appCntxt);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->semsegPerf,
                            "Semantic Segmentation GRAPH");

        vxStatus = SEMSEG_CNN_setupPipeline(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_setupPipeline() failed\n",
                        __FUNCTION__, __LINE__);
        }            
    }

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
            PTK_printf("[%s:%d] CM_scalerNodeCntxtSetCoeff() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {    

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_semseg_cnn");
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

        appCntxt->exitInputDataProcess = 0;
        appCntxt->state                = SEMSEG_CNN_STATE_INIT;

        SEMSEG_CNN_reset(appCntxt);
        appPerfStatsResetAll();
    }


    return vxStatus;
}

vx_status SEMSEG_CNN_run(SEMSEG_CNN_Context   *appCntxt,
                         const unsigned char  *inputImage,
                         uint64_t              timestamp)
{
    vx_status   vxStatus;

    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    SEMSEG_CNN_graphParams* gpDesc;

    vxStatus = SEMSEG_CNN_popFreeInputDesc(appCntxt, &gpDesc);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] SEMSEG_CNN_popFreeInputDesc() failed\n",
                    __FUNCTION__, __LINE__);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        start = GET_TIME();
        vxStatus = ptkdemo_copy_data_to_image(inputImage, gpDesc->vxInputImage);

        end   = GET_TIME();
        diff  = GET_DIFF(start, end);
        CM_reportProctime("input_image_load", diff);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            /* Release the descriptor back to the free descriptor queue. */
            SEMSEG_CNN_enqueInputDesc(appCntxt, gpDesc);

            PTK_printf("[%s:%d] ptkdemo_copy_data_to_image() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = SEMSEG_CNN_process(appCntxt,
                                      gpDesc,
                                      timestamp);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_process() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

void SEMSEG_CNN_deInit(SEMSEG_CNN_Context *appCntxt)
{
    int32_t status;

    // release graph
    vxReleaseGraph(&appCntxt->vxGraph);

    //tivxTIDLUnLoadKernels(appCntxt->vxContext);
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

    if (status < 0)
    {
        PTK_printf("[%s:%d] appDeInit() failed\n", __FUNCTION__, __LINE__);
    }
#endif

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

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    SEMSEG_CNN_printStats(appCntxt);

    CM_printProctime();
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", SEMSEG_CNN_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    /* Deinit semantic segmentation nodes */
    SEMSEG_CNN_deinit_SS(appCntxt);

    SEMSEG_CNN_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}


void SEMSEG_CNN_reset(SEMSEG_CNN_Context * appCntxt)
{
    appCntxt->startPerfCapt = false;
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
                else if (evt.app_value == SEMSEG_CNN_OUT_AVAIL_EVT)
                {
                    /* Wakeup the display thread. */
                    if (appCntxt->outputCtrlSem)
                    {
                        appCntxt->outputCtrlSem->notify();
                    }
                }
            }

            if (evt.type == VX_EVENT_NODE_COMPLETED)
            {
                vxStatus =
                    SEMSEG_CNN_processEvent(appCntxt, &evt);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] SEMSEG_CNN_processEvent() failed\n",
                                __FUNCTION__, __LINE__);
                }

            } // if (evt.type == VX_EVENT_GRAPH_COMPLETED)

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

static int32_t SEMSEG_CNN_userControlThread(SEMSEG_CNN_Context *appCntxt)
{
    vx_status   vxStatus = VX_SUCCESS;
    uint32_t    done = 0;

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
                SEMSEG_CNN_printStats(appCntxt);

                CM_printProctime();
                CM_resetProctime();
                break;

                case 'e':
                {
                    FILE *fp;
                    const char *name = SEMSEG_CNN_PERF_OUT_FILE;

                    fp = appPerfStatsExportOpenFile(".", (char *)name);

                    if (fp != NULL)
                    {
                        SEMSEG_CNN_exportStats(appCntxt,
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

    vxStatus = SEMSEG_CNN_waitGraph(appCntxt);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] SEMSEG_CNN_waitGraph() failed\n",
                    __FUNCTION__, __LINE__);
    }

    SEMSEG_CNN_cleanupHdlr(appCntxt);

    PTK_printf("\nDEMO FINISHED!\n");

    return vxStatus;
}

static void SEMSEG_CNN_preProcThread(SEMSEG_CNN_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        SEMSEG_CNN_graphParams  *desc;
        vx_status                vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->preProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPreprocThread == true)
        {
            break;
        }

        vxStatus = SEMSEG_CNN_popPreprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus =
                SEMSEG_CNN_preProcess(appCntxt,
                                      desc->vxScalerOut,
                                      desc->dlrInputBuff);

            end = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Preprocessing", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] SEMSEG_CNN_preProcess() "
                           "failed.\n",
                            __FUNCTION__, __LINE__);
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                SEMSEG_CNN_enqueDLRInputDesc(appCntxt, desc);

                /* Wakeup the DLR thread. The DLR thread will process the
                 * descriptor at the head of the queue.
                 */
                appCntxt->dlrDataReadySem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

static void SEMSEG_CNN_dlrThread(SEMSEG_CNN_Context  *appCntxt)
{
    CM_DLRNodeCntxt        *dlrObj;
    CM_DLRNodeInputInfo    *dlrInput;
    CM_DLRNodeOutputInfo   *dlrOutput;

    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    dlrObj       = &appCntxt->dlrObj;
    dlrInput     = &dlrObj->input;
    dlrOutput    = &dlrObj->output;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        SEMSEG_CNN_graphParams  *desc;
        vx_status                vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->dlrDataReadySem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitDlrThread == true)
        {
            break;
        }

        vxStatus = SEMSEG_CNN_popDLRInputDesc(appCntxt, &desc);

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
                SEMSEG_CNN_enquePostprocInputDesc(appCntxt, desc);

                /* Wakeup the post-process thread. The post-process thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->postProcSem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

static void SEMSEG_CNN_postProcThread(SEMSEG_CNN_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        SEMSEG_CNN_graphParams  *desc;
        vx_status                vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->postProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPostprocThread == true)
        {
            break;
        }

        vxStatus = SEMSEG_CNN_popPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->enablePostProcNode)
            {
                /* Create the output object for display. */
                start = GET_TIME();
                vxStatus =
                    SEMSEG_CNN_postProcess(appCntxt,
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
            
            // create vxOutTensor always
            {
                /* Create an output tensor. */
                start = GET_TIME();
                vxStatus =
                    SEMSEG_CNN_createOutTensor(appCntxt, 
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
                /* Push the descriptor to the output queue. */
                SEMSEG_CNN_enqueOutputDesc(appCntxt, desc);
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Let the usr know that output is ready. */
                vxStatus =
                    vxSendUserEvent(appCntxt->vxContext,
                                    SEMSEG_CNN_OUT_AVAIL_EVT,
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

    /* Launch CNN pre-processing thread */
    appCntxt->preProcThread =
        std::thread(SEMSEG_CNN_preProcThread, appCntxt);

    /* Launch DLR processing thread */
    appCntxt->dlrThread =
        std::thread(SEMSEG_CNN_dlrThread, appCntxt);

    /* launch post-processing thread. */
    appCntxt->postProcThread =
            std::thread(SEMSEG_CNN_postProcThread, appCntxt);
}

void SEMSEG_CNN_intSigHandler(SEMSEG_CNN_Context *appCntxt)
{
    if (appCntxt->state != SEMSEG_CNN_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = SEMSEG_CNN_STATE_SHUTDOWN;
        PTK_printf("[%s:%d] Waiting for the graph to finish.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = SEMSEG_CNN_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_waitGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }

        SEMSEG_CNN_cleanupHdlr(appCntxt);
        PTK_printf("\nDEMO FINISHED!\n");
    }

    exit(0);
}

