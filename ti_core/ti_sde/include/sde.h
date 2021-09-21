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
#ifndef _APP_SDE_H_
#define _APP_SDE_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/j7.h>
#include <TI/tivx_stereo.h>
#include <TI/tivx_img_proc.h>

#include <perception/perception.h>
#include <perception/utils/ipc_chan.h>
#include <perception/utils/ptk_semaphore.h>

#include <cm_profile.h>
#include <cm_common.h>
#include <cm_remote_service.h>

#include <sde_ldc_applib.h>
#include <sde_singlelayer_applib.h>
#include <sde_multilayer_applib.h>
#include <sde_triangulate_applib.h>


using namespace std;
using namespace ti_core_common;

#include <utils/perf_stats/include/app_perf_stats.h>

#define SDEAPP_PERF_OUT_FILE        "app_sde"

#define SDEAPP_MAX_LINE_LEN         (1024U)
#define SDEAPP_NUM_BUFF_DESC        (1U)

#define SDEAPP_USER_EVT_EXIT        (1U)

#define SDEAPP_MAX_IMAGE_WIDTH      (2048)
#define SDEAPP_MAX_IMAGE_HEIGHT     (1024)
#define SDEAPP_DEFAULT_IMAGE_WIDTH  (1280)
#define SDEAPP_DEFAULT_IMAGE_HEIGHT (720)

#define SDEAPP_MAX_PIPELINE_DEPTH   (8)
#define SDEAPP_NUM_GRAPH_PARAMS     (5U)
#define SDEAPP_GRAPH_COMPLETE_EVENT (0U)

#define SDEAPP_STATE_INVALID        (0U)
#define SDEAPP_STATE_INIT           (1U)
#define SDEAPP_STATE_SHUTDOWN       (2U)

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

    /* Graph parameter 4: output point cloud */
    vx_user_data_object     vxOutputTriangPC;

    /* Timestamp - Not a Graph param */
    vx_uint64             * timestamp;

} SDEAPP_graphParams;

using SDEAPP_graphParamQ = std::queue<SDEAPP_graphParams*>;


struct SDEAPP_Context
{
    /** Application state */
    uint32_t                               state;

    /** OpenVX references */
    vx_context                             vxContext;

    /** OpenVX graph */
    vx_graph                               vxGraph;

    /** Input left image object */
    vx_image                               vxInputLeftImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input right image object */
    vx_image                               vxInputRightImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input rectified left image object */
    vx_image                               vxLeftRectImage;

    /** Input rectified right image object */
    vx_image                               vxRightRectImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                               vxSde16BitOutput[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Merged disparity map object at base layer */
    vx_image                               vxMergeDisparityL0[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Median filtered disparity map object at base layer */
    vx_image                               vxMedianFilteredDisparity[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Ouput point cloud object */
    vx_user_data_object                    vxOutputTriangPC[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input timestamp */
    vx_uint64                              timestamp[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Object that right image is passed to  after processing graph */
    vx_image                               vxDispRightImage;

    /** Object that output disparity map is passed to after processing graph */
    vx_image                               vxDisparity16;

    /** Object that output point cloud is passed to after processing graph */
    vx_user_data_object                    vxDispTriangPC;

    /** output timestamp */
    vx_uint64                              outTimestamp;

    /** Input image width */
    int16_t                                width;

    /** Input image height */
    int16_t                                height;

    /** Horizontal pixel offset from which the image is processed */
    int16_t                                hImgOfst;

    /** Vertical pixel offset from which the image is processed */
    int16_t                                vImgOfst;

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

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t                sde_params;

    /** camera parameters:  
     * horizontal distortion center
     */
    float                                  distCenterX;

    /** vetical distortion center */
    float                                  distCenterY;

    /** camera focal length */
    float                                  focalLength;

    /** baseline between left and right cameras */
    float                                  baseline;

    /** Flag to create point cloud */
    uint8_t                                enablePC;

    /** Flag to use PC configuration */
    uint8_t                                usePCConfig;

    /** Sub-sampling ratio */
    uint8_t                                pcSubsampleRatio;

    /** disparity confidence threshold */ 
    uint8_t                                dispConfidence;

    /** 3D point cloud configuration:
     * Min X position for a point to be rendered
     */
    float                                  lowPtX;

    /** Max X position for a point to be rendered */
    float                                  highPtX;

    /** Min Y position for a point to be rendered */
    float                                  lowPtY;

    /** Max Y position for a point to be rendered */
    float                                  highPtY;

    /** Min Z position for a point to be rendered */
    float                                  lowPtZ;

    /** Max Z position for a point to be rendered */
    float                                  highPtZ;

    /** graph parameter tracking */
    SDEAPP_graphParams                     paramDesc[SDEAPP_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    SDEAPP_graphParamQ                     freeQ;

    /** Queue for output processing. */
    SDEAPP_graphParamQ                     outputQ;

    /** Left rectification LUT file name */
    char                                   left_LUT_file_name[SDEAPP_MAX_LINE_LEN];

    /** Right rectification LUT file name */
    char                                   right_LUT_file_name[SDEAPP_MAX_LINE_LEN];

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

    /** Semantic segmentation applib create params */
    SDE_TRIANG_APPLIB_createParams         sdeTriangCreateParams;

    /** Semantic segmentation applib handler */
    SDE_TRIANG_APPLIB_Handle               sdeTriangHdl;

    /** Number of graph params */
    uint8_t                                numGraphParams;

    /** Render periodicity in milli-sec. */
    uint64_t                               renderPeriod;


    /** Flag to indicate that the input data processing thread should exit. */
    bool                                   exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                                   exitOutputThread;

    /** Flag to indicate that the input data processing has finished. */
    bool                                   processFinished;

    /** Input data thread. */
    std::thread                            userCtrlThread;

    /** Event handler thread. */
    std::thread                            evtHdlrThread;

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
    app_perf_point_t                       sdePclPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                                   startPerfCapt;

    /** Resource lock. */
    std::mutex                             paramRsrcMutex;

};


/**
 * \brief Set LDC, SDE and Point Cloud create parameters
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_setAllParams(SDEAPP_Context *appCntxt);

/**
 * \brief Initialize app, e.g. create graph, load kernels, 
 *        initialize LDC node, SDE node, Triangulation (Point Cloud) node, 
 *        setup pipeline, etc.
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_init(SDEAPP_Context *appCntxt);

/**
 * \brief Launch input data thread and event handler thread. 
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_launchProcThreads(SDEAPP_Context *appCntxt);

/**
 * \brief Handle intercept signal (Ctrl+C) to exit
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_intSigHandler(SDEAPP_Context *appCntxt);


/**
 * \brief Clean up all the resources before exiting 
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt);


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
vx_status SDEAPP_init_camInfo(SDEAPP_Context *appCntxt, 
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
vx_status SDEAPP_run(SDEAPP_Context *appCntxt,
                     const vx_uint8 * inputLeftImage,
                     const vx_uint8 * inputRightImage,
                     vx_uint64 timestamp);

/**
 * \brief Function to initialize LDC nodes and graphs
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_init_LDC(SDEAPP_Context *appCntxt);


/**
 * \brief Function to initialize SDE nodes and graphs
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_init_SDE(SDEAPP_Context *appCntxt);


/**
 * \brief Function to initialize Triangulation (Point Cloud)  nodes and graphs
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status SDEAPP_init_SDE_Triang(SDEAPP_Context *appCntxt);


/**
 * \brief Main function to set up a whole graph pipeline, which calls 
 *        either SDEAPP_setupPipeline_SL or SDEAPP_setupPipeline_ML
 *        depending on SDE type
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status SDEAPP_setupPipeline(SDEAPP_Context * appCntxt);


/**
 * \brief Function to set up a whole graph pipeline when a single-layer SDE is used
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status SDEAPP_setupPipeline_SL(SDEAPP_Context * appCntxt);

/**
 * \brief Function to set up a whole graph pipeline when a multi-layer SDE is used
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 *
 */ 
vx_status SDEAPP_setupPipeline_ML(SDEAPP_Context * appCntxt);


/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SDEAPP_printStats(SDEAPP_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 */
vx_status SDEAPP_exportStats(SDEAPP_Context * appCntxt, FILE *fp, bool exportAll);

/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 */
vx_status SDEAPP_waitGraph(SDEAPP_Context * appCntxt);

/**
 * \brief Function to get free resource for the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SDEAPP_getFreeParamRsrc(SDEAPP_Context       *appCntxt,
                                  SDEAPP_graphParams   **gpDesc);

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
vx_status SDEAPP_process(SDEAPP_Context * appCntxt, SDEAPP_graphParams * gpDesc);

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
vx_status SDEAPP_processEvent(SDEAPP_Context * appCntxt, vx_event_t * event);

/**
 * \brief Function to mark the dequeued resource as free by moving dequeued 
 *        resource to the output queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] rsrcIndex index of paramDesc that will be moved to the output queue
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SDEAPP_releaseParamRsrc(SDEAPP_Context  *appCntxt, uint32_t rsrcIndex);


/**
 * \brief Function to returns references to the input and output at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        SDEAPP_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SDEAPP_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] appCntxt APP context
 *
 * \param [out] rightRectImage right rectified image passed from a graph
 * 
 * \param [out] disparity16 output raw disparity map passed from a graph
 *
 * \param [out] pointcloud output point cloud passed from a graph
 * 
 * \param [out] timestamp time stamp passed from a graph, which should be identical
 *                        to input time stamp
 *
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_getOutBuff(SDEAPP_Context      *appCntxt,
                            vx_image            *rightRectImage,
                            vx_image            *disparity16,
                            vx_user_data_object *pointcloud,
                            vx_uint64           *timestamp);

/**
 * \brief Function to release buffer in the output queue by moving 
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_releaseOutBuff(SDEAPP_Context * appCntxt);

#endif /* _APP_SDE_H_ */
