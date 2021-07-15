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
#ifndef _APP_ESTOP_H_
#define _APP_ESTOP_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/j7.h>
#include <TI/tivx_stereo.h>

#include <perception/perception.h>
#include <perception/utils/ipc_chan.h>
#include <perception/utils/ptk_semaphore.h>

#include <sde_ldc_applib.h>
#include <sde_singlelayer_applib.h>
#include <sde_multilayer_applib.h>
#include <ss_sde_detection_applib.h>

#include <app_ptk_demo_common.h>

#include <cm_scaler_node_cntxt.h>
#include <cm_preproc_node_cntxt.h>
#include <cm_dlr_node_cntxt.h>
#include <cm_profile.h>


using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_graph_perf.h>

#define APP_ASSERT(x)               PTK_assert((x))
#define APP_ASSERT_VALID_REF(ref)   APP_ASSERT(vxGetStatus((vx_reference)(ref))==VX_SUCCESS)

#define ESTOP_APP_PERF_OUT_FILE     "app_estop"

#define ESTOP_APP_MAX_LINE_LEN         (1024U)
#define ESTOP_APP_NUM_BUFF_DESC        (1U)

#define ESTOP_APP_DEFAULT_IMAGE_WIDTH  (1280)
#define ESTOP_APP_DEFAULT_IMAGE_HEIGHT (720)

#define ESTOP_APP_GRAPH_COMPLETE_EVENT        (0U)
#define ESTOP_APP_SCALER_NODE_COMPLETE_EVENT  (ESTOP_APP_GRAPH_COMPLETE_EVENT + 1)
#define ESTOP_APP_CNN_OUT_AVAIL_EVENT         (ESTOP_APP_SCALER_NODE_COMPLETE_EVENT + 1)
#define ESTOP_APP_USER_EVT_EXIT               (ESTOP_APP_CNN_OUT_AVAIL_EVENT + 1)

#define ESTOP_APP_STATE_INVALID    (0U)
#define ESTOP_APP_STATE_CAM_INIT   (1U)
#define ESTOP_APP_STATE_INIT       (2U)
#define ESTOP_APP_STATE_SHUTDOWN   (3U)

#define FOV_GRID_VALUE             (-1)
#define NON_FOV_GRID_VALUE          (0)
#define ESTOP_AREA_GRID_VALUE      (98)

typedef struct
{
    /* Graph parameter 0: left input image */
    vx_image                vxInputLeftImage;

    /* Graph parameter 1: right input image */
    vx_image                vxInputRightImage;

    /* Graph parameter 2: rectified right (base) image */
    vx_image                vxRightRectImage;

    /* Graph parameter 3: 16-bit raw SDE output */
    vx_image                vxSde16BitOutput;

    /* Graph parameter 3: output disparity in multi-layer SDE */
    vx_image                vxMergeDisparityL0;

    /* Graph parameter 3: median filtered output disparity in multi-layer SDE */
    vx_image                vxMedianFilteredDisparity;

    /* Graph parameter 4 */
    vx_user_data_object     vx3DBoundBox;

    /* Graph parameter 5 */
    vx_image                vxScalerOut;

    /* Graph Parameter 6 */
    vx_tensor               vxOutTensor;

    /* Input to DLR node. */
    float                 * dlrInputBuff;

    /* Output from the DLR node. */
    int32_t               * dlrOutputBuff;

    /* Timestamp - Not a Graph param */
    vx_uint64             * timestamp;

} ESTOP_APP_graphParams;

using ESTOP_APP_graphParamQ = std::queue<ESTOP_APP_graphParams*>;

struct ESTOP_APP_Queue
{
    /** Resource lock. Used get/put from/to freeQ. */
    std::mutex                      m_mutex;

    /** A queue for holding free descriptors. */
    ESTOP_APP_graphParamQ   m_q;

    ESTOP_APP_graphParams *peek()
    {
        std::unique_lock<std::mutex>   lock(m_mutex);
        ESTOP_APP_graphParams  *desc = nullptr;

        /* Check if we have descriptors available. */
        if (!m_q.empty())
        {
            desc = m_q.front();
        }

        return desc;
    }

    ESTOP_APP_graphParams *pop()
    {
        std::unique_lock<std::mutex>   lock(m_mutex);
        ESTOP_APP_graphParams  *desc = nullptr;

        /* Check if we have descriptors available. */
        if (!m_q.empty())
        {
            desc = m_q.front();
            m_q.pop();
        }

        return desc;
    }

    void push(ESTOP_APP_graphParams  *desc)
    {
        std::unique_lock<std::mutex>   lock(m_mutex);

        m_q.push(desc);
    }

    int32_t size()
    {
        return m_q.size();
    }
};


struct ESTOP_APP_Context
{
    /** Application state */
    uint32_t                               state;

    /** OpenVX references */
    vx_context                             vxContext;

    /** OpenVX graph */
    vx_graph                               vxGraph;

    /** Scaler node context object. */
    CM_ScalerNodeCntxt                     scalerObj{};

    /** DLR node context object. */
    CM_DLRNodeCntxt                        dlrObj;

    /** Input left image object */
    vx_image                               vxInputLeftImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input right image object */
    vx_image                               vxInputRightImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input rectified left image object */
    vx_image                               vxLeftRectImage;

    /** Input rectified right image object */
    vx_image                               vxRightRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* Output Tensor after post-processing. */
    vx_tensor                              vxOutTensor[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                               vxSde16BitOutput[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Merged disparity map object at base layer */
    vx_image                               vxMergeDisparityL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Median filtered disparity map object at base layer */
    vx_image                               vxMedianFilteredDisparity[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Ouput point cloud object */
    vx_user_data_object                    vx3DBoundBox[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* Input timestamp */
    vx_uint64                              timestamp[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* DLR input buffers. */
    float                                 *dlrInputBuff[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* DLR output buffer. */ 
    int32_t                               *dlrOutputBuff[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Object that right image is passed to after processing graph */
    vx_image                               vxDispRightImage;

    /** Object that output semantic segmentation image is passed to after processing graph */
    vx_image                               vxDispSSImage;

    /** Object that output semantic segmentation tensor is passed to after processing graph */
    vx_tensor                              vxOutputTensor;

    /** Object that output 3D bounding box is passed to after processing graph */
    vx_user_data_object                    vxDisp3DBoundBox;

    /** Object that output disparity map is passed to after processing graph */
    vx_image                               vxDisparity16;

    /* output time stamp */
    vx_uint64                              outTimestamp;

    /** 3D bounding box memory size */
    int32_t                                bbSize;

    /** Input image width */
    int16_t                                width;

    /** Input image height */
    int16_t                                height;

    /** Tensor width */
    int16_t                                tensor_width;

    /** Tensor height */
    int16_t                                tensor_height;

    /** Number of classed that will be color coded in a post processing node */
    int16_t                                numClasses;

    /** Mean values to be used in pre-processing stage. */
    float                                  preProcMean[CM_PREPROC_MAX_IMG_CHANS];

    /** Scaling values to be used in pre-processing stage. */
    float                                  preProcScale[CM_PREPROC_MAX_IMG_CHANS];

    /** Flag to indicate if PostProc node will need to be created. If true, then
     *  the TIDL node will be the tail node, otherwise the PostProc node will
     *  be the tail node.
     */
    uint8_t                                enablePostProcNode;

    /** Input image format */
    uint8_t                                inputFormat;

    /** OpenVX pipeline depth */
    uint8_t                                pipelineDepth;

    /** SDE algo type (0: Single-layer SDE, 1: Multi-layer SDE) */
    uint8_t                                sdeAlgoType;

    /** the number of layers (2, 3) for multi-layer SDE */
    uint8_t                                numLayers;

    /** whether enable median filter in multi-layer SDE */
    uint8_t                                ppMedianFilterEnable;

    /** disparity confidence threshold */ 
    uint8_t                                confidence_threshold;

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t                sde_params;

    /** camera parameters 
     * horizontal distortion center
     */
    float                                  distCenterX;

    /** vetical distortion center */
    float                                  distCenterY;

    /** camera focal length */
    float                                  focalLength;

    /** camera roll angle */
    float                                  camRoll;

    /** camera pitch angle */
    float                                  camPitch;

    /** camera yaw angle */
    float                                  camYaw;

    /** camera mounting height */
    float                                  camHeight;

    /** baseline between left and right cameras */
    float                                  baseline;

    /** OG map parameters 
     * horizontal grid size 
     */
    int32_t                                xGridSize;

    /** vertical grid size */
    int32_t                                yGridSize;

    /** Min X range in mm */
    int32_t                                xMinRange;

    /** Min X range in mm */
    int32_t                                xMaxRange;

    /** Min Y range in mm */
    int32_t                                yMinRange;

    /** Min Y range in mm */    
    int32_t                                yMaxRange;

    /** Number of grid in X dimension */
    int32_t                                xGridNum;

    /** Number of grid in Y dimension */
    int32_t                                yGridNum;

    /** Pixel count threshold of grid for occupied/non-occupied decision */
    int16_t                                thCnt;

    /** Pixel count threshold of object for occupied/non-occupied decision */
    int16_t                                thObjCnt;

    /** Maximum number of objects to be detected */
    int16_t                                maxNumObject;

    /** Number of neighboring grids to check for connected component analysis */
    int16_t                                cNeighNum;

    /** Enable flag of spatial object merge */
    uint8_t                                enableSpatialObjMerge;

    /** Enable flag of temporal object merge */
    uint8_t                                enableTemporalObjMerge;

    /** Enable flag of temporal object smoothing */
    uint8_t                                enableTemporalObjSmoothing;

    /** Method to compute distance between objects
     *  0: distance between centers
     *  1: distacne between corners
     */
    uint8_t                                objectDistanceMode;

    /* graph parameter tracking */
    ESTOP_APP_graphParams                  paramDesc[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    ESTOP_APP_Queue                        freeQ;

    /** Pre-process task queue. */
    ESTOP_APP_Queue                        preProcQ;

    /** DLR task queue. */
    ESTOP_APP_Queue                        dlrQ;

    /** Post-process task queue. */
    ESTOP_APP_Queue                        postProcQ;

    /** Queue for output processing. */
    ESTOP_APP_Queue                        outputQ;

    /** TIDL network file name */
    char                                   dlrModelPath[ESTOP_APP_MAX_LINE_LEN];

    /** Left rectification LUT file name */
    char                                   left_LUT_file_name[ESTOP_APP_MAX_LINE_LEN];

    /** Right rectification LUT file name */
    char                                   right_LUT_file_name[ESTOP_APP_MAX_LINE_LEN];

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                                is_interactive;

    /** LDC applib create params */
    SDELDCAPPLIB_createParams              sdeLdcCreateParams;

    /** LDC applib handler */
    SDELDCAPPLIB_Handle                    sdeLdcHdl;

    /** Single-layer SDE applib create params */
    SL_SDEAPPLIB_createParams              slSdeCreateParams;

    /** Single-layer SDE applib handler */
    SL_SDEAPPLIB_Handle                    slSdeHdl;

    /** Multi-layer SDE applib create params */
    ML_SDEAPPLIB_createParams              mlSdeCreateParams;

    /** Multi-layer SDE applib handler */
    ML_SDEAPPLIB_Handle                    mlSdeHdl;

    /** OG Map based detector applib create params */
    SS_DETECT_APPLIB_createParams          ssDetectCreateParams;

    /** OG Map based detector applib handler */
    SS_DETECT_APPLIB_Handle                ssDetectHdl;

    /** Number of graph params */
    uint8_t                                numGraphParams;

    /** Number of output tensor */
    uint8_t                                numOutTensors;

    /** Render periodicity in milli-sec. */
    uint64_t                               renderPeriod;


    /** Flag to indicate that the input data processing thread should exit. */
    bool                                   exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                                   exitOutputThread;

    /** Flag to control DLR process exit. */
    bool                                   exitPreprocThread;

    /** Flag to control DLR process exit. */
    bool                                   exitDlrThread;

    /** Flag to control DLR process exit. */
    bool                                   exitPostprocThread;

    /** Input data thread. */
    std::thread                            userCtrlThread;

    /** Event handler thread. */
    std::thread                            evtHdlrThread;

    /** pre-processing thread. */
    std::thread                            preProcThread;

    /** DLR thread */
    std::thread                            dlrThread;

    /** post-processing thread. */
    std::thread                            postProcThread;

    /** Semaphore for controlling the pre-processing thread. */
    UTILS::Semaphore                     * preProcSem;

    /** Semaphore to synchronize the DLR iput data and DLR threads. */
    UTILS::Semaphore                     * dlrDataReadySem;

    /** Semaphore for controlling the post-processing thread. */
    UTILS::Semaphore                     * postProcSem;

    /** Semaphore to synchronize the output and display threads. */
    UTILS::Semaphore                     * outputCtrlSem;

    /** For graph profiling: graph start time */
    chrono::time_point<chrono::system_clock> profileStart;

    /** For OpenVX graph profiling: graph end time */
    chrono::time_point<chrono::system_clock> profileEnd;


    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                rtLogEnable;

    /** Base value to be used for any programmed VX events. */
    uint32_t                               vxEvtAppValBase;

    /** Performance monitoring. */
    app_perf_point_t                       estopPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                                   startPerfCapt;

};

/**
 * \brief Set LDC, SDE, SemSeg and detection create parameters
 *
 * \param [in] appCntxt APP context
 * 
 */
void      ESTOP_APP_setAllParams(ESTOP_APP_Context *appCntxt);

/**
 * \brief Initialize app, e.g. create graph, load kernels, 
 *        initialize LDC node, SDE node, SemSeg node, and detection node, 
 *        setup pipeline, etc.
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_init(ESTOP_APP_Context *appCntxt);

/**
 * \brief Reset app's parameters
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
void      ESTOP_APP_reset(ESTOP_APP_Context * appCntxt);

/**
 * \brief Launch input data thread and event handler thread. 
 *
 * \param [in] appCntxt APP context
 * 
 */
void      ESTOP_APP_launchProcThreads(ESTOP_APP_Context *appCntxt);

/**
 * \brief Handle intercept signal (Ctrl+C) to exit
 *
 * \param [in] appCntxt APP context
 * 
 */
void      ESTOP_APP_intSigHandler(ESTOP_APP_Context *appCntxt);

/**
 * \brief Clean up all the resources before exiting 
 *
 * \param [in] appCntxt APP context
 * 
 */
void      ESTOP_APP_cleanupHdlr(ESTOP_APP_Context *appCntxt);


/**
 * \brief Initialize camera info
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] width image width
 
 * \param [in] width image height
 * 
 * \param [in] f camera focal length
 
 * \param [in] dx camera horizontal distortion center
 * 
 * \param [in] dy camera vertical distortion center
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_init_camInfo(ESTOP_APP_Context *appCntxt, 
                                 uint32_t width,
                                 uint32_t height,
                                 double   f,
                                 double   dx,
                                 double   dy);

/**
 * \brief Run a graph for the given input
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] inputLeftImage input left image
 * 
 * \param [in] inputRightImage input right image
 * 
 * \param [in] timestamp input images' time stamp
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_run(ESTOP_APP_Context *appCntxt, 
                        const vx_uint8 * inputLeftImage, const vx_uint8 * inputRightImage, 
                        vx_uint64 timestamp);

/**
 * \brief Function to initialize LDC nodes and graphs
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status   ESTOP_APP_init_LDC(ESTOP_APP_Context *appCntxt);

/**
 * \brief Function to initialize SDE nodes and graphs
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status   ESTOP_APP_init_SDE(ESTOP_APP_Context *appCntxt);

/**
 * \brief Function to initialize Semantic Segmentation nodes
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_init_SS(ESTOP_APP_Context *appCntxt);


/**
 * \brief Function to de-initialize Semantic Segmentation nodes
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_deinit_SS(ESTOP_APP_Context *appCntxt);


/**
 * \brief Function to initialize Detection nodes and graphs
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_init_SS_Detection(ESTOP_APP_Context *appCntxt);

/**
 * \brief Main function to set up a whole graph pipeline, which calls 
 *        either ESTOP_APP_setupPipeline_SL or ESTOP_APP_setupPipeline_ML
 *        depending on SDE type
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status ESTOP_APP_setupPipeline(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to set up a whole graph pipeline when a single-layer SDE is used
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status ESTOP_APP_setupPipeline_SL(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to set up a whole graph pipeline when a multi-layer SDE is used
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status ESTOP_APP_setupPipeline_ML(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 */
void      ESTOP_APP_printStats(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 */
vx_status ESTOP_APP_exportStats(ESTOP_APP_Context * appCntxt, FILE *fp, bool exportAll);

/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 */
vx_status ESTOP_APP_waitGraph(ESTOP_APP_Context * appCntxt);




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
vx_status ESTOP_APP_process(ESTOP_APP_Context * appCntxt, ESTOP_APP_graphParams * gpDesc);



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
vx_status ESTOP_APP_CNN_preProcess(ESTOP_APP_Context     * appCntxt, 
                                   vx_image                vxScalerOut,
                                   float                 * dlrInputBuff);


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
vx_status ESTOP_APP_CNN_postProcess(ESTOP_APP_Context     * appCntxt, 
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
vx_status ESTOP_APP_createOutTensor(ESTOP_APP_Context     * appCntxt,
                                    vx_tensor                vxOutTensor,
                                    int32_t                * dlrOutputBuff);


/**
 * \brief Function to process the events programmed to be handled by the
 *        APPLIB. Currently only VX_EVENT_GRAPH_COMPLETED event is handled.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] event pointer to event
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status ESTOP_APP_processEvent(ESTOP_APP_Context * appCntxt, vx_event_t * event);


/**
 * \brief Function to returns references to the input and output at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        ESTOP_APP_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to ESTOP_APP_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] appCntxt APP context
 *
 * \param [out] rightRectImage right rectified image passed from a graph
 * 
 * \param [out] ssOutput output color-coded semantic segmentation map passed from a graph
 *
 * \param [out] outTensor output semantic segmentation tensor passed from a graph
 * 
 * \param [out] obsBB output obstacle bounding boxes passed from a graph
*
 * \param [out] disparity16 output raw disparity map passed from a graph
 *
 * \param [out] timestamp time stamp passed from a graph, which should be identical
 *                        to input time stamp
 *
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_getOutBuff(ESTOP_APP_Context *appCntxt, 
                               vx_image *rightRectImage, 
                               vx_image *ssOutput, 
                               vx_tensor *outTensor, 
                               vx_user_data_object *obsBB, 
                               vx_image *disparity16,
                               vx_uint64 *timestamp);

/**
 * \brief Function to release buffer in the output queue by moving 
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 * 
 */
vx_status ESTOP_APP_releaseOutBuff(ESTOP_APP_Context * appCntxt);


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
vx_status ESTOP_APP_popFreeInputDesc(ESTOP_APP_Context       *appCntxt,
                                     ESTOP_APP_graphParams  **gpDesc);

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
vx_status ESTOP_APP_popPreprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                        ESTOP_APP_graphParams  **gpDesc);

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
vx_status ESTOP_APP_popDLRInputDesc(ESTOP_APP_Context       *appCntxt,
                                    ESTOP_APP_graphParams  **gpDesc);

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
vx_status ESTOP_APP_popPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc);
                                         /**
 * \brief Function to get free resource from the post-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status ESTOP_APP_getPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc);


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
vx_status ESTOP_APP_getOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams   *gpDesc);

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
vx_status ESTOP_APP_popOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams  **gpDesc);

/**
 * \brief Function to add the buffer to the free input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void ESTOP_APP_enqueInputDesc(ESTOP_APP_Context      *appCntxt,
                              ESTOP_APP_graphParams  *desc);

/**
 * \brief Function to add the buffer to the pre-processing input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void ESTOP_APP_enquePreprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                     ESTOP_APP_graphParams  *desc);

/**
 * \brief Function to add the buffer to the DLR input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void ESTOP_APP_enqueDLRInputDesc(ESTOP_APP_Context      *appCntxt,
                                 ESTOP_APP_graphParams  *desc);

/**
 * \brief Function to add the buffer to the post-proce input queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void ESTOP_APP_enquePostprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                      ESTOP_APP_graphParams  *desc);

/**
 * \brief Function to add the buffer to the output queue. 
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 */
void ESTOP_APP_enqueOutputDesc(ESTOP_APP_Context      *appCntxt,
                               ESTOP_APP_graphParams  *desc);


#ifdef __cplusplus
}
#endif

#endif /* _APP_ESTOP_H_ */
