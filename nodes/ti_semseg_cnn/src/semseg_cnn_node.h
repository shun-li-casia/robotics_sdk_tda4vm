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
#ifndef _APP_SEMSEG_CNN_NODE_H_
#define _APP_SEMSEG_CNN_NODE_H_

#include <stdio.h>
#include <thread>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <semseg_cnn.h>

using namespace sensor_msgs;
using namespace message_filters;

class CnnSemSegNode
{
    using ImgSub   = message_filters::Subscriber<Image>;
    using ImgPub   = image_transport::Publisher;
    using ImgTrans = image_transport::ImageTransport;
    using SubCon   = message_filters::Connection;

    public:
        CnnSemSegNode(ros::NodeHandle &nodeHdl,
                      ros::NodeHandle &privNodeHdl);
        ~CnnSemSegNode();
        void sigHandler(int32_t  sig);

    private:
        vx_status init();
        void readParams();
        void imgCb(const ImageConstPtr& imgPtr);
        void subscriberThread();
        void publisherThread();
        void processCompleteEvtHdlr();
        vx_status extractImageData(std::vector<uint8_t>    &v,
                                   const vx_image           vxImage,
                                   uint32_t                 width,
                                   uint32_t                 height);

    private:
        SEMSEG_CNN_Context     *m_cntxt{};
        ImgSub                 *m_sub{};
        ImgTrans               *m_imgTrans{};
        uint8_t                *m_rectImageData{};
        uint8_t                *m_outImageData{};
        uint8_t                *m_ssTensorData{};
        ros::NodeHandle        &m_nodeHdl;
        ros::NodeHandle        &m_privNodeHdl;
        SubCon                  m_conObj;
        ImgPub                  m_rectImgPub;
        ImgPub                  m_outImgPub;
        ImgPub                  m_ssTensorPub;
        std::thread             m_subThread;
        std::thread             m_pubThread;

        uint32_t                m_inputImgWidth;
        uint32_t                m_inputImgHeight;
        uint32_t                m_outImgWidth;
        uint32_t                m_outImgHeight;
        std::vector<uint8_t>    m_rectImagePubData;
        std::vector<uint8_t>    m_outPubData;
        std::vector<uint8_t>    m_ssTensorPubData;
        uint32_t                m_rectImageSize;
        uint32_t                m_outImageSize;
        uint32_t                m_ssTensorSize;
        bool                    m_outputRgb{true};
};

#endif /* _APP_SEMSEG_CNN_NODE_H_ */

