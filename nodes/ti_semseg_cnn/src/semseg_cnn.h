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
#ifndef _APP_SEMSEG_CNN_H_
#define _APP_SEMSEG_CNN_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <thread>
#include <queue>
#include <vector>

#include <itidl_ti.h>
#include <TI/j7_tidl.h>
#include <TI/tivx_img_proc.h>

#include <perception/perception.h>
#include <perception/utils/ptk_semaphore.h>

#include <app_ptk_demo_common.h>

#include <cm_scaler_node_cntxt.h>
#include <cm_preproc_node_cntxt.h>
#include <cm_dlr_node_cntxt.h>
#include <cm_ldc_node_cntxt.h>
#include <cm_profile.h>


#ifdef __cplusplus
extern "C" {
#endif

#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_graph_perf.h>

#define SEMSEG_CNN_DEFAULT_IMAGE_WIDTH  (1280)
#define SEMSEG_CNN_DEFAULT_IMAGE_HEIGHT (720)
#define SEMSEG_LDC_DS_FACTOR            (2)
#define SEMSEG_LDC_BLOCK_WIDTH          (64)
#define SEMSEG_LDC_BLOCK_HEIGHT         (16)
#define SEMSEG_LDC_PIXEL_PAD            (1)

#define SEMSEG_CNN_PERF_OUT_FILE        "apps_semseg_cnn"

#define SEMSEG_CNN_MAX_FILE_PATH        (1024U)

#define SEMSEG_CNN_MAX_LINE_LEN         (1024U)
#define SEMSEG_CNN_NUM_BUFF_DESC        (1U)

#define SEMSEG_CNN_STATE_INVALID        (0U)
#define SEMSEG_CNN_STATE_INIT           (1U)
#define SEMSEG_CNN_STATE_SHUTDOWN       (2U)

#define SEMSEG_CNN_SCALER_NODE_COMPLETE_EVENT (0U)
#define SEMSEG_CNN_OUT_AVAIL_EVT              (SEMSEG_CNN_SCALER_NODE_COMPLETE_EVENT + 1)
#define SEMSEG_CNN_USER_EVT_EXIT              (SEMSEG_CNN_OUT_AVAIL_EVT + 1)



struct SEMSEG_CNN_graphParams
{
    /* Graph parameter 0 */
    vx_image                vxInputImage;

    /* Graph parameter 1 */
    vx_image                vxRectifiedImage;

    /* Graph parameter 2 */
    vx_image                vxScalerOut;

    /* Output Tensor after post-processing. */
    vx_tensor               vxOutTensor;

    /* Input to DLR node. */
    float                  *dlrInputBuff;

    /* Output from the DLR node. */
    int32_t                *dlrOutputBuff;

    /* timestamp - Not a Graph param */
    vx_uint64              *timestamp;
};

using SEMSEG_CNN_graphParamQ =
      std::queue<SEMSEG_CNN_graphParams*>;

struct SEMSEG_CNN_Queue
{
    /** Resource lock. Used get/put from/to freeQ. */
    std::mutex                      m_mutex;

    /** A queue for holding free descriptors. */
    SEMSEG_CNN_graphParamQ   m_q;

    SEMSEG_CNN_graphParams *peek()
    {
        std::unique_lock<std::mutex>    lock(m_mutex);
        SEMSEG_CNN_graphParams  *desc = nullptr;

        /* Check if we have descriptors available. */
        if (!m_q.empty())
        {
            desc = m_q.front();
        }

        return desc;
    }

    SEMSEG_CNN_graphParams *pop()
    {
        std::unique_lock<std::mutex>    lock(m_mutex);
        SEMSEG_CNN_graphParams  *desc = nullptr;

        /* Check if we have descriptors available. */
        if (!m_q.empty())
        {
            desc = m_q.front();
            m_q.pop();
        }

        return desc;
    }

    void push(SEMSEG_CNN_graphParams  *desc)
    {
        std::unique_lock<std::mutex>   lock(m_mutex);

        m_q.push(desc);
    }

    int32_t size()
    {
        return m_q.size();
    }
};

struct SEMSEG_CNN_Context
{
    /** Application state */
    uint32_t                state;

    /** openVX context handle. */
    vx_context              vxContext;

    /** Graph handle. */
    vx_graph                vxGraph{};

    /** LDC node context object. */
    CM_LdcNodeCntxt         ldcObj{};

    /** Scaler node context object. */
    CM_ScalerNodeCntxt      scalerObj{};

    /** DLR node context object. */
    CM_DLRNodeCntxt         dlrObj;

    /** Input image object  */
    vx_image                vxInputImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Rectified image object  */
    vx_image                vxRectifiedImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* Output Tensor after post-processing. */
    vx_tensor               vxOutTensor[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* Input timestamp */
    vx_uint64               timestamp[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* DLR input buffers. */
    float                  *dlrInputBuff[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* DLR output buffer. */
    int32_t                *dlrOutputBuff[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** */
    vx_image                vxInputDisplayImage;

    /** */
    vx_image                vxOutputImage;

    /** */
    vx_user_data_object     vxNNConfig;

    /** */
    vx_tensor               vxOutputTensor;

    /** */
    char                    ldcLutFilePath[SEMSEG_CNN_MAX_FILE_PATH];

    /** */
    char                    dlrModelPath[SEMSEG_CNN_MAX_FILE_PATH];

    /** */
    char                    input_file_list[SEMSEG_CNN_MAX_LINE_LEN];

    /** */
    char                    input_file_path[SEMSEG_CNN_MAX_LINE_LEN];

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                 is_interactive;

    /** output tensor write flag */
    uint8_t                 enOutTensorWrite;

    /** Total frames processed */
    uint32_t                frameCnt;

    /** Input image width in pixels. */
    int32_t                 inputImageWidth;

    /** Input image height in pixels. */
    int32_t                 inputImageHeight;

    /** LDC sub-sampling factor. Ignored if enableLdcNode is false. */
    uint32_t                ldcSsFactor;

    /** LDC block width. Ignored if enableLdcNode is false. */
    uint32_t                ldcBlockWidth;

    /** LDC block height. Ignored if enableLdcNode is false. */
    uint32_t                ldcBlockHeight;

    /** Pixed padding. Ignored if enableLdcNode is false. */
    uint32_t                ldcPixelPad;

    /** DL image width in pixels. */
    int32_t                 tidlImageWidth;

    /** DL image height in pixels. */
    int32_t                 tidlImageHeight;

    /** Output image width in pixels. */
    int32_t                 outImageWidth;

    /** Output image height in pixels. */
    int32_t                 outImageHeight;

    /** Number of classed that will be color coded in a post processing node */
    int16_t                 numClasses;

    /** Mean values to be used in pre-processing stage. */
    float                   preProcMean[CM_PREPROC_MAX_IMG_CHANS];

    /** Scaling values to be used in pre-processing stage. */
    float                   preProcScale[CM_PREPROC_MAX_IMG_CHANS];


    /** Flag to indicate if padding of the input image will be done inside the
     *  TIDL.
     *  0 - Padding will not be handled inside TIDL
     *  1 - Padding will be handled inside TIDL
     */
    uint8_t                 padInTidl;

    /** Flag to indicate if LDC node will need to be created. If true, then
     *  the LDC node will be the head node, otherwise the scaler node will
     *  be the head node.
     */
    uint8_t                 enableLdcNode;

    /** Flag to indicate if PostProc node will need to be created. If true, then
     *  the TIDL node will be the tail node, otherwise the PostProc node will
     *  be the tail node.
     */
    uint8_t                 enablePostProcNode;

    /** Input image format */
    uint8_t                 inputFormat;

    /** OpenVX pipeline depth */
    uint8_t                 pipelineDepth;

    /* output time stamp */
    vx_uint64               outTimestamp;

    /** Number of graph params */
    uint8_t                 numGraphParams;

    /* graph parameter tracking */
    SEMSEG_CNN_graphParams  paramDesc[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    SEMSEG_CNN_Queue        freeQ;

    /** Pre-process task queue. */
    SEMSEG_CNN_Queue         preProcQ;

    /** DLR task queue. */
    SEMSEG_CNN_Queue         dlrQ;

    /** Post-process task queue. */
    SEMSEG_CNN_Queue         postProcQ;

    /** Queue for output processing. */
    SEMSEG_CNN_Queue         outputQ;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                    exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                    exitOutputThread;

    /** Flag to control DLR process exit. */
    bool                    exitPreprocThread;

    /** Flag to control DLR process exit. */
    bool                    exitDlrThread;

    /** Flag to control DLR process exit. */
    bool                    exitPostprocThread;

    /** User interactive thread. */
    std::thread             userCtrlThread;

    /** Event handler thread. */
    std::thread             evtHdlrThread;

    /** pre-processing thread. */
    std::thread             preProcThread;

    /** DLR processing thread. */
    std::thread             dlrThread;

    /** post-processing thread. */
    std::thread             postProcThread;

    /** Semaphore for controlling the pre-processing thread. */
    UTILS::Semaphore       *preProcSem;

    /** Semaphore to synchronize the DLR iput data and DLR threads. */
    UTILS::Semaphore       *dlrDataReadySem;

    /** Semaphore for controlling the post-processing thread. */
    UTILS::Semaphore       *postProcSem;

    /** Semaphore to synchronize the processing and output threads. */
    UTILS::Semaphore       *outputCtrlSem;

    /** For graph profiling: graph start time */
    chrono::time_point<chrono::system_clock> profileStart;

    /** For OpenVX graph profiling: graph end time */
    chrono::time_point<chrono::system_clock> profileEnd;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                 exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                 rtLogEnable;

    /** Base value to be used for any programmed VX events. */
    uint32_t                vxEvtAppValBase;

    /** Performance monitoring. */
    app_perf_point_t        semsegPerf;


    /** Flag to track if the performance counter has been initialized. */
    bool                    startPerfCapt;
};

/**
 * \brief Initialize app, e.g. create graph, load kernels, 
 *        initialize SemSeg nodes, setup pipelines, etc
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SEMSEG_CNN_init(SEMSEG_CNN_Context *appCntxt);


/**
 * \brief Reset app's parameters
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
void      SEMSEG_CNN_reset(SEMSEG_CNN_Context * appCntxt);

/**
 * \brief Function to initialize Semantic Segmentation nodes 
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SEMSEG_CNN_init_SS(SEMSEG_CNN_Context *appCntxt);

/**
 * \brief Function to de-initialize Semantic Segmentation nodes 
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SEMSEG_CNN_deinit_SS(SEMSEG_CNN_Context *appCntxt);

/**
 * \brief Function to set up a graph pipeline
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status SEMSEG_CNN_setupPipeline(SEMSEG_CNN_Context * appCntxt);


/**
 * \brief Run a graph for the given input
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] inputImage input image
 * 
 * \param [in] timestamp input images' time stamp
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SEMSEG_CNN_run(SEMSEG_CNN_Context   *appCntxt,
                         const unsigned char  *inputImage,
                         uint64_t              timestamp);


/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SEMSEG_CNN_printStats(SEMSEG_CNN_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 */
vx_status SEMSEG_CNN_exportStats(SEMSEG_CNN_Context * appCntxt, FILE *fp, bool exportAll);


/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 */
vx_status SEMSEG_CNN_waitGraph(SEMSEG_CNN_Context * appCntxt);

/**
 * \brief Function to process the events programmed to be handled by the
 *        APPLIB. Currently VX_EVENT_NODE_COMPLETED event is handled.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] event pointer to event
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_processEvent(SEMSEG_CNN_Context * appCntxt, vx_event_t * event);



/**
 * \brief Function to process input images. This function is non-blocking
 *        and returns success if the pipeline is not full.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_process(SEMSEG_CNN_Context     * appCntxt, 
                             SEMSEG_CNN_graphParams * gpDesc,
                             uint64_t                 timestamp);

/**
 * \brief Function to returns references to the {input, output} images at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        SEMSEG_CNN_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SEMSEG_CNN_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in]  appCntxt APP context
 *
 * \param [out] inputImg Input image passed to the graph.
 *
 * \param [out] output Reference to the output object from the graph
 *              corresponding to the 'inputImage'. The reference could
 *              be either a tensor or an image. See following.
 *              - output refere to a tensor if enablePostProcNode is false
 *              - output refere to an image if enablePostProcNode is true
 * 
 * \param [out] timestamp Timestamp of inputImg
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_getOutBuff(SEMSEG_CNN_Context   * appCntxt,
                                vx_image             * inputImage,
                                vx_reference         * output,
                                vx_uint64            * timestamp);

/**
 * \brief Function to release buffer in the output queue by moving 
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 * 
 */
vx_status SEMSEG_CNN_releaseOutBuff(SEMSEG_CNN_Context * appCntxt);

/**
 * \brief Function to convert the scaler output to RGB
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] vxScalerOut Scaler (MSC) output image
 *
 * \param [out] dlrInputBuff RGB data from the scaler output image
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_preProcess(SEMSEG_CNN_Context     * appCntxt, 
                                vx_image                 vxScalerOut,
                                float                  * dlrInputBuff);

/**
 * \brief Function to create color-coded semantic segmantaion map
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in, out] vxScalerOut Scaler (MSC) output image
 *
 * \param [in] dlrOutputBuff DLR modle output that includes sematic segmentation classes
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_postProcess(SEMSEG_CNN_Context     * appCntxt, 
                                 vx_image                 vxScalerOut,
                                 int32_t                * dlrOutputBuff);

/**
 * \brief Function to convert semantic sementation output to tensor
 *
 * \param [in] appCntxt APP context
 * 
 * \param [out] vxOutTensor tensor object created from dlrOutputBuff
 *
 * \param [in] dlrOutputBuff DLR modle output that includes sematic segmentation classes
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_createOutTensor(SEMSEG_CNN_Context     * appCntxt,
                                     vx_tensor                vxOutTensor,
                                     int32_t                * dlrOutputBuff);


/**
 * \brief Function to get and pop free resource from the free input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_popFreeInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                      SEMSEG_CNN_graphParams  **gpDesc);


/**
 * \brief Function to get and pop free resource from the pre-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_popPreprocInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                         SEMSEG_CNN_graphParams  **gpDesc);


/**
 * \brief Function to get and pop free resource from the DLR input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_popDLRInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                     SEMSEG_CNN_graphParams  **gpDesc);


/**
 * \brief Function to get and pop free resource from the post-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_popPostprocInputDesc(SEMSEG_CNN_Context       *appCntxt,
                                          SEMSEG_CNN_graphParams  **gpDesc);


/**
 * \brief Function to get the buffer from the output queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_getOutputDesc(SEMSEG_CNN_Context       *appCntxt,
                                   SEMSEG_CNN_graphParams   *gpDesc);


/**
 * \brief Function to release the buffer from the output queue. 
 *        The release output buffer is moved to the input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SEMSEG_CNN_popOutputDesc(SEMSEG_CNN_Context       *appCntxt,
                                   SEMSEG_CNN_graphParams  **gpDesc);

/**
 * \brief Function to add the buffer to the free input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void      SEMSEG_CNN_enqueInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                    SEMSEG_CNN_graphParams  *desc);

/**
 * \brief Function to add the buffer to the pre-processing input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void      SEMSEG_CNN_enquePreprocInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                           SEMSEG_CNN_graphParams  *desc);


/**
 * \brief Function to add the buffer to the DLR input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void      SEMSEG_CNN_enqueDLRInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                       SEMSEG_CNN_graphParams  *desc);

/**
 * \brief Function to add the buffer to the post-proce input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void      SEMSEG_CNN_enquePostprocInputDesc(SEMSEG_CNN_Context      *appCntxt,
                                            SEMSEG_CNN_graphParams  *desc);
                                           

/**
 * \brief Function to add the buffer to the output queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void      SEMSEG_CNN_enqueOutputDesc(SEMSEG_CNN_Context      *appCntxt,
                                     SEMSEG_CNN_graphParams  *desc);



/**
 * \brief Launch input data thread, event handler thread and DLR thread
 *
 * \param [in] appCntxt APP context
 * 
 */
void SEMSEG_CNN_launchProcThreads(SEMSEG_CNN_Context *appCntxt);

/**
 * \brief Handle intercept signal (Ctrl+C) to exit
 *
 * \param [in] appCntxt APP context
 * 
 */
void SEMSEG_CNN_intSigHandler(SEMSEG_CNN_Context *appCntxt);

/**
 * \brief Clean up all the resources before exiting 
 *
 * \param [in] appCntxt APP context
 * 
 */
void SEMSEG_CNN_cleanupHdlr(SEMSEG_CNN_Context *appCntxt);


#ifdef __cplusplus
}
#endif

#endif /* _APP_SEMSEG_CNN_H_ */
