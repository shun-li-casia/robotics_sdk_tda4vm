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
#include <vector>

#include <itidl_ti.h>
#include <TI/j7_tidl.h>
#include <TI/tivx_img_proc.h>
#include <TI/tivx_stereo.h>

#include <perception/perception.h>
#include <perception/utils/ptk_semaphore.h>

#include <semseg_cnn_applib.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <utils/draw2d/include/draw2d.h>
#include <utils/grpx/include/app_grpx.h>
#include <utils/perf_stats/include/app_perf_stats.h>

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

#define OUTPUT_DISPLAY_WIDTH            (960U)
#define OUTPUT_DISPLAY_HEIGHT           (480U)

#define INPUT_DISPLAY_WIDTH             (960U)
#define INPUT_DISPLAY_HEIGHT            (480U)

#define INPUT_START_X_OFFSET            (0U)
#define INPUT_START_Y_OFFSET            (300U)

#define SEMSEG_CNN_MAX_TENSOR_DIMS      (4u)
#define SEMSEG_CNN_USER_EVT_EXIT        (1U)

#define SEMSEG_CNN_STATE_INVALID        (0U)
#define SEMSEG_CNN_STATE_INIT           (1U)
#define SEMSEG_CNN_STATE_SHUTDOWN       (2U)

struct SEMSEG_CNN_Context
{
    /** */
    uint32_t                state;

    /** */
    vx_context              vxContext;

    /** */
    vx_image                vxInputImage;

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
    char                    tidlCfgFilePath[SEMSEG_CNN_MAX_FILE_PATH];

    /** */
    char                    tidlNwFilePath[SEMSEG_CNN_MAX_FILE_PATH];

    /** */
    char                    input_file_list[SEMSEG_CNN_MAX_LINE_LEN];

    /** */
    char                    input_file_path[SEMSEG_CNN_MAX_LINE_LEN];

    /** */
    char                    output_file_path[SEMSEG_CNN_MAX_LINE_LEN];

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                 is_interactive;

    /** output tensor write flag */
    uint8_t                 enOutTensorWrite;

    /** Total frames processed */
    uint32_t                frameCnt;

    /** */
    vx_int32                inDispWidth;

    /** */
    vx_int32                inDispHeight;

    /** */
    vx_int32                outDispWidth;

    /** */
    vx_int32                outDispHeight;

    /** */
    SEMSEG_CNN_APPLIB_createParams  createParams;

    /** */
    SEMSEG_CNN_APPLIB_Handle sscnnHdl;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                    exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                    exitOutputThread;

    /** User interactive thread. */
    std::thread             userCtrlThread;

    /** Event handler thread. */
    std::thread             evtHdlrThread;

    /** Semaphore to synchronize the processing and output threads. */
    UTILS::Semaphore       *outputCtrlSem;

    /** Delay in milli-sec to introduce between successive input frames.
     * Valid only for file based input.
     */
    uint32_t                interFrameDelay;

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
};

vx_status SEMSEG_CNN_init(SEMSEG_CNN_Context *appCntxt);

vx_status SEMSEG_CNN_processImage(SEMSEG_CNN_Context   *appCntxt,
                                  const unsigned char  *inputImage);

void SEMSEG_CNN_launchProcThreads(SEMSEG_CNN_Context *appCntxt);

void SEMSEG_CNN_intSigHandler(SEMSEG_CNN_Context *appCntxt);

void SEMSEG_CNN_cleanupHdlr(SEMSEG_CNN_Context *appCntxt);

#ifdef __cplusplus
}
#endif

#endif /* _APP_SEMSEG_CNN_H_ */
