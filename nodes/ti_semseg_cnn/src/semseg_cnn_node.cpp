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
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <semseg_cnn_node.h>
#include <semseg_cnn.h>

#include <cm_common.h>
#include <app_ptk_demo_common.h>

static void SEMSEG_CNN_appShowUsage(char *argv[])
{
    PTK_printf("\n");
    PTK_printf(" Semantic Segmentation Network Demo - (c) Texas Instruments 2020\n");
    PTK_printf(" =====================================================================\n");
    PTK_printf("\n");
    PTK_printf("Please refer to demo guide for prerequisites before running this demo\n");
    PTK_printf("\n");
    PTK_printf(" Usage,\n");
    PTK_printf("  %s --cfg <config file>\n", argv[0]);
    PTK_printf("\n");

    return;
} 

CnnSemSegNode::CnnSemSegNode(ros::NodeHandle &nodeHdl,
                             ros::NodeHandle &privNodeHdl):
    m_nodeHdl(nodeHdl),
    m_privNodeHdl(privNodeHdl),
    m_imgTrans(new ImgTrans(nodeHdl))
{
    bool        status;
    vx_status   vxStatus;

    // Initialize the CNN context
    vxStatus = this->init();

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        ROS_INFO("Application Initialization failed.");
        exit(-1);
    }

    // Cache the input and output image sizes
    m_inputImgWidth  = m_cntxt->inputImageWidth;
    m_inputImgHeight = m_cntxt->inputImageHeight;
    m_outImgWidth    = m_cntxt->outImageWidth;
    m_outImgHeight   = m_cntxt->outImageHeight;

    // launch the subscriber thread
    m_subThread = std::thread([=]{subscriberThread();});
    m_subThread.detach();

    // launch the publisher threads
    m_pubThread = std::thread([=]{publisherThread();});
    m_pubThread.detach();
}

void CnnSemSegNode::subscriberThread()
{
    std::string topicName;
    bool        status;

    // Query the topicname to subscribe to
    status = m_privNodeHdl.getParam("input_topic_name", topicName);

    if (status == false)
    {
        ROS_INFO("Config parameter 'input_topic_name' not found.");
        exit(-1);
    }

    m_sub = new ImgSub(m_nodeHdl, topicName, 1);

    if (m_sub == nullptr)
    {
        ROS_ERROR("new ImgSub() failed.");
        exit(-1);
    }

    m_conObj = m_sub->registerCallback(&CnnSemSegNode::imgCb, this);

    ROS_INFO_STREAM("Subscribed to the topic: " << topicName);
}

void CnnSemSegNode::publisherThread()
{
    std::string rectImgTopic;
    std::string outImgTopic;
    std::string ssTensorTopic;
    bool        status;
    bool        latch = true;

    if (m_cntxt->enableLdcNode)
    {
        status = m_privNodeHdl.getParam("rectified_image_topic", rectImgTopic);
        if (status == false)
        {
            ROS_ERROR("Config parameter 'rectified_image_topic' not found.");
            exit(-1);
        }

        m_rectImageSize = m_inputImgWidth * m_inputImgHeight;
        m_rectImagePubData.reserve(m_rectImageSize);
        m_rectImageData = new uint8_t[m_rectImageSize];

        // Create the publisher for the rectified image
        m_rectImgPub = m_imgTrans->advertise(rectImgTopic, latch);
        ROS_INFO_STREAM("Created Publisher for topic: " << rectImgTopic);
    }

    if (m_cntxt->enablePostProcNode)
    {
        m_privNodeHdl.param("output_rgb", m_outputRgb, true);

        // Query the topic name to publish the output image.
        status = m_privNodeHdl.getParam("semseg_cnn_out_image", outImgTopic);
        if (status == false)
        {
            ROS_ERROR("Config parameter 'semseg_cnn_out_image' not found.");
            exit(-1);
        }

        // Pre-allocate vectors used for publishing out image data
        m_outImageSize = 3 * m_outImgWidth * m_outImgHeight;

        if (m_outputRgb == false)
        {
            m_outImageSize /= 2;
        }

        m_outPubData.reserve(m_outImageSize);
        m_outImageData = new uint8_t[m_outImageSize];

        // Create the publisher for the output image
        m_outImgPub = m_imgTrans->advertise(outImgTopic, latch);
        ROS_INFO_STREAM("Created Publisher for topic: " << outImgTopic);
    } else
    {
        status = m_privNodeHdl.getParam("semseg_cnn_tensor_topic", ssTensorTopic);
        if (status == false)
        {
            ROS_ERROR("Config parameter 'semseg_cnn_tensor_topic' not found.");
            exit(-1);
        }

        m_ssTensorSize = m_outImgWidth * m_outImgHeight;
        m_ssTensorPubData.reserve(m_ssTensorSize);
        m_ssTensorData = new uint8_t[m_ssTensorSize];

        // Create the publisher for the SS map output image
        m_ssTensorPub = m_imgTrans->advertise(ssTensorTopic, latch);
        ROS_INFO_STREAM("Created Publisher for topic: " << ssTensorTopic);
    }

    processCompleteEvtHdlr();

    // Shutdown the publishers
    if (m_cntxt->enableLdcNode)
    {
        m_rectImgPub.shutdown();
    }

    if (m_cntxt->enablePostProcNode)
    {
        m_outImgPub.shutdown();
    } 
    else
    {
        m_ssTensorPub.shutdown();
    }
}

void CnnSemSegNode::imgCb(const ImageConstPtr& imgPtr)
{
    if (m_cntxt->state == SEMSEG_CNN_STATE_INIT)
    {
        SEMSEG_CNN_run(m_cntxt, imgPtr->data.data(), imgPtr->header.stamp.toNSec());
    }
    else
    {
        m_conObj.disconnect();
    }
}

void CnnSemSegNode::processCompleteEvtHdlr()
{
    Image       rectImg;
    Image       outImg;
    Image       ssTensor;
    vx_status   vxStatus = VX_SUCCESS;

    ROS_INFO("%s Launched.", __FUNCTION__);

    if (m_cntxt->enableLdcNode)
    {
        rectImg.width    = m_inputImgWidth;
        rectImg.height   = m_inputImgHeight;
        rectImg.step     = m_inputImgWidth;
        rectImg.encoding = "mono8";
    }

    if (m_cntxt->enablePostProcNode)
    {
        outImg.width  = m_outImgWidth;
        outImg.height = m_outImgHeight;

        if (m_outputRgb == true)
        {
            outImg.step     = m_outImgWidth * 3;
            outImg.encoding = "rgb8";
        }
        else
        {
            outImg.step     = m_outImgWidth;
            outImg.encoding = "yuv420";
        }
    }
    else
    {
        // SS Tensor
        ssTensor.width    = m_outImgWidth;
        ssTensor.height   = m_outImgHeight;

        ssTensor.step     = m_outImgWidth;
        ssTensor.encoding = "mono8";
    }

    while (m_cntxt->exitOutputThread == false)
    {
        vx_reference    outRef{};
        vx_status       vxStatus;

        /* Wait for the data ready semaphore. */
        if (m_cntxt->outputCtrlSem)
        {
            m_cntxt->outputCtrlSem->wait();
        }

        if (m_cntxt->exitOutputThread == true)
        {
            break;
        }

        SEMSEG_CNN_getOutBuff(m_cntxt,
                             &m_cntxt->vxInputDisplayImage,
                             &outRef,
                             &m_cntxt->outTimestamp);

        if (m_cntxt->enableLdcNode)
        {
            // Extract rectified right image data (Luma only)
            vxStatus = CM_extractImageData(m_rectImageData,
                                           m_cntxt->vxInputDisplayImage,
                                           m_inputImgWidth,
                                           m_inputImgHeight,
                                           1,
                                           0);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                ROS_ERROR("CM_extractImageData() failed.");
            }
        }

        if (m_cntxt->enablePostProcNode)
        {
            m_cntxt->vxOutputImage = (vx_image)outRef;

            // Extract output image data
            vxStatus = CM_extractImageData(m_outImageData,
                                           m_cntxt->vxOutputImage,
                                           m_outImgWidth,
                                           m_outImgHeight,
                                           2,
                                           m_outputRgb);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                ROS_ERROR("CM_extractImageData() failed.");
            }
        } 
        else
        {
            m_cntxt->vxOutputTensor = (vx_tensor)outRef;

            // Extract raw tensor data
            vxStatus = CM_extractTensorData(m_ssTensorData,
                                            m_cntxt->vxOutputTensor,
                                            m_outImgWidth,
                                            m_outImgHeight);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                ROS_ERROR("CM_extractTensorData() failed.");
            }
        }

        // Exclusively for debugging during initial development
#if 0
        {
            static uint32_t    cnt = 0;

            cnt++;
            std::string i = "ros_input_image_" + std::to_string(cnt) + ".yuv";
            vxStatus = CM_saveImage((char*)i.c_str(), m_cntxt->vxInputDisplayImage);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                ROS_ERROR("CM_saveImage() failed.");
            }

            std::string o = "ros_output_image_" + std::to_string(cnt) + ".yuv";
            vxStatus = CM_saveImage((char*)o.c_str(), m_cntxt->vxOutputImage);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                ROS_ERROR("CM_saveImage() failed.");
            }
        }
#endif

        // Release the output buffers
        SEMSEG_CNN_releaseOutBuff(m_cntxt);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            ros::Time time;
            time.fromNSec(m_cntxt->outTimestamp);

            if (m_cntxt->enableLdcNode)
            {
                // Publish Rectified (Right) image
                m_rectImagePubData.assign(m_rectImageData,
                                          &m_rectImageData[m_rectImageSize]);

                rectImg.data         = m_rectImagePubData;
                rectImg.header.stamp = time;

                m_rectImgPub.publish(rectImg);
            }

            if (m_cntxt->enablePostProcNode)
            {
                // Publish output image
                m_outPubData.assign(m_outImageData,
                                    &m_outImageData[m_outImageSize]);

                outImg.data         = m_outPubData;
                outImg.header.stamp = time;

                m_outImgPub.publish(outImg);
            }
            else
            {
                // Publish output tensor
                m_ssTensorPubData.assign(m_ssTensorData,
                                         &m_ssTensorData[m_ssTensorSize]);

                ssTensor.data         = m_ssTensorPubData;
                ssTensor.header.stamp = time;

                m_ssTensorPub.publish(ssTensor);
            }
        }
    }

    ROS_INFO("%s Exiting.", __FUNCTION__);
}

CnnSemSegNode::~CnnSemSegNode()
{
    if (m_rectImageData)
    {
        delete [] m_rectImageData;
    }

    if (m_outImageData)
    {
        delete [] m_outImageData;
    }

    if (m_ssTensorData)
    {
        delete [] m_ssTensorData;
    }

    if (m_imgTrans)
    {
        delete m_imgTrans;
    }

    if (m_sub)
    {
        delete m_sub;
    }

    if (m_cntxt)
    {
        delete m_cntxt;
    }
}

void CnnSemSegNode::readParams()
{
    std::string                     str;
    bool                            status;
    int32_t                         tmp;

    // set default parameters
    m_cntxt->ldcSsFactor        = SEMSEG_LDC_DS_FACTOR;
    m_cntxt->ldcBlockWidth      = SEMSEG_LDC_BLOCK_WIDTH;
    m_cntxt->ldcBlockHeight     = SEMSEG_LDC_BLOCK_HEIGHT;
    m_cntxt->ldcPixelPad        = SEMSEG_LDC_PIXEL_PAD;
    m_cntxt->enableLdcNode      = 1;
    m_cntxt->padInTidl          = 0;
    m_cntxt->vxEvtAppValBase    = 0;

    /* Get LUT file path information. */
    status = m_privNodeHdl.getParam("lut_file_path", str);

    if (status == false)
    {
        ROS_INFO("Config parameter 'lut_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->ldcLutFilePath, SEMSEG_CNN_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get TIDL network path information. */
    status = m_privNodeHdl.getParam("dlr_model_file_path", str);

    if (status == false)
    {
        ROS_INFO("Config parameter 'dlr_model_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->dlrModelPath, SEMSEG_CNN_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get input image width information. */
    m_privNodeHdl.param("width", m_cntxt->inputImageWidth, SEMSEG_CNN_DEFAULT_IMAGE_WIDTH);

    /* Get input image height information. */
    m_privNodeHdl.param("height", m_cntxt->inputImageHeight, SEMSEG_CNN_DEFAULT_IMAGE_HEIGHT);

    /* Get input TIDL input image width information. */
    m_privNodeHdl.param("dl_width", m_cntxt->tidlImageWidth, SEMSEG_CNN_DEFAULT_IMAGE_WIDTH);

    /* Get input TIDL input image height information. */
    m_privNodeHdl.param("dl_height", m_cntxt->tidlImageHeight, SEMSEG_CNN_DEFAULT_IMAGE_HEIGHT);

    /* Get input TIDL output image width information. */
    m_privNodeHdl.param("out_width", m_cntxt->outImageWidth, SEMSEG_CNN_DEFAULT_IMAGE_WIDTH);

    /* Get input TIDL output image height information. */
    m_privNodeHdl.param("out_height", m_cntxt->outImageHeight, SEMSEG_CNN_DEFAULT_IMAGE_HEIGHT);

    /* Get class count information. */
    status = m_privNodeHdl.getParam("num_classes", tmp);

    if (status == false)
    {
        ROS_INFO("Config parameter 'num_classes' not found.");
        exit(-1);
    }

    m_cntxt->numClasses = (int16_t)tmp;

    /* Get pre-process mean information. */
    std::vector<int32_t>   mean = {128, 128, 128};
    mean = m_privNodeHdl.param("pre_proc_mean", mean);

    m_cntxt->preProcMean[0] = mean[0];
    m_cntxt->preProcMean[1] = mean[1];
    m_cntxt->preProcMean[2] = mean[2];

    /* Get pre-process mean information. */
    std::vector<float>   scale = {0.015625, 0.015625, 0.015625};
    scale = m_privNodeHdl.param("pre_proc_scale", scale);

    m_cntxt->preProcScale[0] = scale[0];
    m_cntxt->preProcScale[1] = scale[1];
    m_cntxt->preProcScale[2] = scale[2];

    /* Get post processing node enable information. */
    m_privNodeHdl.param("enable_post_proc", tmp, 1);
    m_cntxt->enablePostProcNode = (uint8_t)tmp;

    /* Get interactive mode flag information. */
    m_privNodeHdl.param("is_interactive", tmp, 0);
    m_cntxt->is_interactive = (uint8_t)tmp;

    /* Get pipeline depth information. */
    m_privNodeHdl.param("pipeline_depth", tmp, 1);
    m_cntxt->pipelineDepth = (uint8_t)tmp;

    /* Get graph export flag information. */
    m_privNodeHdl.param("exportGraph", tmp, 0);
    m_cntxt->exportGraph = (uint8_t)tmp;

    /* Get real-time logging enable information. */
    m_privNodeHdl.param("rtLogEnable", tmp, 0);
    m_cntxt->rtLogEnable = (uint8_t)tmp;

    return;
}

vx_status CnnSemSegNode::init()
{
    PTK_CRT             ptkConfig;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    //tivx_set_debug_zone(VX_ZONE_INFO);

    PTK_init(&ptkConfig);

    m_cntxt = new SEMSEG_CNN_Context();

    if (m_cntxt == nullptr)
    {
        ROS_ERROR("new SEMSEG_CNN_Context() failed.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Read the node parameters. */
        readParams();

        /* Initialize the Application context */
        vxStatus = SEMSEG_CNN_init(m_cntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            ROS_ERROR("SEMSEG_CNN_init() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Launch processing threads. */
        SEMSEG_CNN_launchProcThreads(m_cntxt);
    }

    return vxStatus;
}

void CnnSemSegNode::sigHandler(int32_t  sig)
{
    (void)sig;

    if (m_cntxt)
    {
        SEMSEG_CNN_intSigHandler(m_cntxt);
    }
}
