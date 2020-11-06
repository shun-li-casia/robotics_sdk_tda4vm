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
#ifndef _SDE_NODE_H_
#define _SDE_NODE_H_

#include <stdio.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <sde.h>
#include <signal.h>

#include <common_msgs/Disparity.h>


#ifdef __cplusplus
extern "C" {
#endif

class ROSAppSDE
{
public:
    ROSAppSDE(ros::NodeHandle &privNodeHdl,  std::string disparity_topic_name);

    ~ROSAppSDE();

    /** Event handler thread */
    void rosAppSDEEvtHdlrThread(SDEAPP_Context *appCntxt);

    /** Intc signal handler thread */
    void rosAppSDEIntcHdlrThread(SDEAPP_Context *appCntxt);

    /** Processing thread */
    void rosAppSdelaunchProcThreads(SDEAPP_Context *appCntxt);

    /* Initialize application */
    int  rosAppSDEInit();

    /* Process input stereo data */
    void rosAppSDEProcessData(unsigned char* leftImg, unsigned char* rightImg);

    /** Deinitialize application */
    void rosAppSDEDeInit();

    /** Publish raw disparity map output */
    void rosAppSDEPublishOutput(int32_t width, int32_t height, int16_t minSdeDisp, int16_t maxSdeDisp, vx_image disparityImage);

    /** Get application context */
    SDEAPP_Context * getSDEAPPContext() { return appCntxt; }

private:
    /** Read yaml algorithm paramters file */
    void readParams();

    /** Node handler to read alogrithm parameters */
    ros::NodeHandle                 &m_privNodeHdl;

    /** Event handler thread. */
    std::thread                     intcHdlrThread;

    /** Application context */
    SDEAPP_Context                * appCntxt;
};

#ifdef __cplusplus
}
#endif

#endif /* _SDE_NODE_H_ */







