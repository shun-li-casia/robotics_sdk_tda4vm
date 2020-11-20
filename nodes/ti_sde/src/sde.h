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
#ifndef _SDE_H_
#define _SDE_H_

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
#include <sde_multilayer_applib.h>
#include <sde_singlelayer_applib.h>


#ifdef __cplusplus
extern "C" {
#endif


#define SDEAPP_DEFAULT_IMAGE_WIDTH  (1280)
#define SDEAPP_DEFAULT_IMAGE_HEIGHT (720)
#define SDEAPP_MAX_PIPELINE_DEPTH   (4)

#define SDEAPP_MAX_NUM_LAYERS       (3U)

#define SDEAPP_MAX_LINE_LEN         (1024U)
#define SDEAPP_NUM_BUFF_DESC        (1U)

#define SDEAPP_NUM_GRAPH_PARAMS     (6U)
#define SDEAPP_GRAPH_COMPLETE_EVENT (0U)

#define SDEAPP_USER_EVT_EXIT        (1U)


using namespace std;


typedef struct
{
    /* Graph parameter 0 */
    vx_image                vxInputLeftImage;

    /* Graph parameter 1 */
    vx_image                vxInputRightImage;

    /* Graph parameter 2 */
    //vx_image                vxLeftRectImage;

    /* Graph parameter 3 */
    vx_image                vxRightRectImage;

    /* Graph parameter 4 */
    vx_image                vxSde16BitOutput;

    /* Graph parameter 4 */
    vx_image                vxMergeDisparityL0;

    /* Graph parameter 5 */
    vx_image                vxMedianFilteredDisparity;

} SDEAPP_graphParams;

using SDEAPP_graphParamQ = std::queue<SDEAPP_graphParams*>;


typedef struct
{
    vx_context                             vxContext;
    vx_graph                               vxGraph;

    vx_node                                node_disparity_display;
    vx_user_data_object                    disparity_display_config;
    tivx_display_params_t                  disparity_display_params;
    vx_node                                node_image_display;
    vx_user_data_object                    image_display_config;
    tivx_display_params_t                  image_display_params;

    /* graph parameter tracking */
    SDEAPP_graphParams                     paramDesc[SDEAPP_MAX_PIPELINE_DEPTH];

        /** A queue for holding free descriptors. */
    SDEAPP_graphParamQ                     freeQ;

    /** Queue for output processing. */
    SDEAPP_graphParamQ                     outputQ;

    vx_image                               vxInputLeftImage[SDEAPP_MAX_PIPELINE_DEPTH];
    vx_image                               vxInputRightImage[SDEAPP_MAX_PIPELINE_DEPTH];
    vx_image                               vxLeftRectImage[SDEAPP_MAX_PIPELINE_DEPTH];
    vx_image                               vxRightRectImage[SDEAPP_MAX_PIPELINE_DEPTH];
    vx_image                               vxSde16BitOutput[SDEAPP_MAX_PIPELINE_DEPTH];
    vx_image                               vxMergeDisparityL0[SDEAPP_MAX_PIPELINE_DEPTH];
    vx_image                               vxMedianFilteredDisparity[SDEAPP_MAX_PIPELINE_DEPTH];

    vx_image                               vxInputImage;
    vx_image                               vxInputDisplayImage;
    vx_image                               vxDisparityCC;
    vx_image                               vxDisparity16;

    tivx_dmpac_sde_params_t                sde_params;

    uint16_t                               width[SDEAPP_MAX_NUM_LAYERS];
    uint16_t                               height[SDEAPP_MAX_NUM_LAYERS];
    uint8_t                                sdeAlgoType;
    uint8_t                                numLayers;
    uint8_t                                ppMedianFilterEnable;

    uint8_t                                confidence_threshold;
    uint8_t                                pipelineDepth;

    char                                   left_img_file_path[SDEAPP_MAX_LINE_LEN];
    char                                   right_img_file_path[SDEAPP_MAX_LINE_LEN];
    char                                   output_file_path[SDEAPP_MAX_LINE_LEN];

    char                                   left_img_file_name[SDEAPP_MAX_LINE_LEN];
    char                                   right_img_file_name[SDEAPP_MAX_LINE_LEN];

    char                                   left_LUT_file_name[SDEAPP_MAX_LINE_LEN];
    char                                   right_LUT_file_name[SDEAPP_MAX_LINE_LEN];

    uint32_t                               start_fileno;
    uint32_t                               end_fileno;

    uint8_t                                inputFormat;

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                                is_interactive;

    SDELDCAPPLIB_createParams              sdeLdcCreateParams;
    SDELDCAPPLIB_Handle                    sdeLdcHdl;

    ML_SDEAPPLIB_createParams              mlSdeCreateParams;
    ML_SDEAPPLIB_Handle                    mlSdeHdl;

    SL_SDEAPPLIB_createParams              slSdeCreateParams;
    SL_SDEAPPLIB_Handle                    slSdeHdl;

    uint8_t                                numGraphParams;

    /** Render periodicity in milli-sec. */
    uint64_t                               renderPeriod;

    /** Flag to indicate that the graph processing thread has finished */
    bool                                   processFinished;

    /** Event handler thread. */
    std::thread                            evtHdlrThread;

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
    app_perf_point_t                       sdePerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                                   startPerfCapt;

    /** Resource lock. */
    std::mutex                             paramRsrcMutex;

} SDEAPP_Context;


void      SDEAPP_setAllParams(SDEAPP_Context *appCntxt);

int32_t   SDEAPP_init(SDEAPP_Context *appCntxt);

void      SDEAPP_launchProcThreads(SDEAPP_Context *appCntxt);

void      SDEAPP_intSigHandler(SDEAPP_Context *appCntxt, int sig);

void      SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt);

void      SDEAPP_run(SDEAPP_Context *appCntxt, vx_uint8 * inputLeftImage, vx_uint8 * inputRightImage);


int32_t   SDEAPP_init_LDC(SDEAPP_Context *appCntxt);

int32_t   SDEAPP_init_SDE(SDEAPP_Context *appCntxt);

vx_status SDEAPP_setupPipeline(SDEAPP_Context * appCntxt);

vx_status SDEAPP_setupPipeline_SL(SDEAPP_Context * appCntxt);

vx_status SDEAPP_setupPipeline_ML(SDEAPP_Context * appCntxt);

void      SDEAPP_printStats(SDEAPP_Context * appCntxt);

void      SDEAPP_exportStats(SDEAPP_Context * appCntxt);

void      SDEAPP_waitGraph(SDEAPP_Context * appCntxt);

int32_t   SDEAPP_getFreeParamRsrc(SDEAPP_Context       *appCntxt,
                                  SDEAPP_graphParams   **gpDesc);

int32_t   SDEAPP_process(SDEAPP_Context * appCntxt,  SDEAPP_graphParams * gpDesc);

int32_t   SDEAPP_processEvent(SDEAPP_Context * appCntxt, vx_event_t * event);

int32_t   SDEAPP_releaseParamRsrc(SDEAPP_Context  *appCntxt, uint32_t rsrcIndex);

int32_t   SDEAPP_getOutBuff(SDEAPP_Context *appCntxt, vx_image *rightRectImage, vx_image *disparity16);

void      SDEAPP_releaseOutBuff(SDEAPP_Context * appCntxt);

#ifdef __cplusplus
}
#endif


#endif /* _SDE_H_ */

