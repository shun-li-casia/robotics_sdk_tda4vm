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
#include <sde_node.h>
#include <sde.h>

using namespace common_msgs;

ros::Publisher  disp_pub;
Disparity       disp_msg;

static char menu[] = {
    "\n"
    "\n =========================================="
    "\n Demo : ROS Stereo                         "
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

ROSAppSDE::ROSAppSDE(ros::NodeHandle &privNodeHdl, std::string disparity_topic_name):
    m_privNodeHdl(privNodeHdl)
{
    ros::NodeHandle nh;

    disp_pub = nh.advertise<Disparity>(disparity_topic_name, 1);

    // OpenVX Graph init
    rosAppSDEInit();
}

ROSAppSDE::~ROSAppSDE()
{
    // OpenVX Graph deinit
    rosAppSDEDeInit();
}


void ROSAppSDE::rosAppSDEEvtHdlrThread(SDEAPP_Context *appCntxt)
{
    vx_event_t evt;
    vx_status vxStatus;

    vxStatus = VX_SUCCESS;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_false_e);

        if (vxStatus == VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == SDEAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            SDEAPP_processEvent(appCntxt, &evt);
            
            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                SDEAPP_getOutBuff(appCntxt, &appCntxt->vxInputImage, &appCntxt->vxDisparity16);

                /****************************/
                /* publish output to topics */
                /****************************/
                rosAppSDEPublishOutput(appCntxt->width[0], appCntxt->height[0],
                                       appCntxt->sde_params.disparity_min, appCntxt->sde_params.disparity_max,
                                       appCntxt->vxDisparity16);

                SDEAPP_releaseOutBuff(appCntxt);
            }
        }
    } // while (true)
}


void ROSAppSDE::rosAppSDEIntcHdlrThread(SDEAPP_Context *appCntxt)
{

    uint32_t done  = 0;

    //appPerfStatsResetAll();

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
            SDEAPP_printStats(appCntxt);
            break;

        case 'e':
            SDEAPP_exportStats(appCntxt);
            break;

        case 'x':
            done = 1;
            appCntxt->processFinished = true;
            /* Consume the newline character. */
            getchar();
            break;
        }
    } // if (appCntxt->is_interactive)

    /* Wait for the graph to consume all input. */
    SDEAPP_waitGraph(appCntxt);

    SDEAPP_cleanupHdlr(appCntxt, false);

    PTK_printf("\nDEMO FINISHED!\n");
}


void ROSAppSDE::rosAppSdelaunchProcThreads(SDEAPP_Context *appCntxt)
{
    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread   = std::thread(&ROSAppSDE::rosAppSDEEvtHdlrThread, this, appCntxt);
    if (appCntxt->is_interactive)
    {
        intcHdlrThread = std::thread(&ROSAppSDE::rosAppSDEIntcHdlrThread, this, appCntxt);
    }
}


void ROSAppSDE::readParams()
{
    std::string                     str;
    bool                            status;
    int32_t                         tmp;

    // Set default SDE parameters
    appCntxt->sde_params.confidence_score_map[0] = 0;
    appCntxt->sde_params.confidence_score_map[1] = 4;
    appCntxt->sde_params.confidence_score_map[2] = 9;
    appCntxt->sde_params.confidence_score_map[3] = 18;
    appCntxt->sde_params.confidence_score_map[4] = 28;
    appCntxt->sde_params.confidence_score_map[5] = 43;
    appCntxt->sde_params.confidence_score_map[6] = 109;
    appCntxt->sde_params.confidence_score_map[7] = 127;

    /* Get left LUT file path information. */
    status = m_privNodeHdl.getParam("left_lut_file_path", str);
    if (status == false)
    {
        ROS_INFO("Config parameter 'left_lut_file_path' not found.");
        exit(-1);
    }

    snprintf(appCntxt->left_LUT_file_name, SDEAPP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get right LUT file path information. */
    status = m_privNodeHdl.getParam("right_lut_file_path", str);
    if (status == false)
    {
        ROS_INFO("Config parameter 'left_lut_file_path' not found.");
        exit(-1);
    }

    snprintf(appCntxt->right_LUT_file_name, SDEAPP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get input image format information */
    m_privNodeHdl.param("input_format", tmp, 1);
    appCntxt->inputFormat = (uint8_t)tmp;

    /* Get input image width information. */
    m_privNodeHdl.param("width", tmp, SDEAPP_DEFAULT_IMAGE_WIDTH);
    appCntxt->width[0] = (uint16_t)tmp;

    /* Get input image height information. */
    m_privNodeHdl.param("height", tmp, SDEAPP_DEFAULT_IMAGE_HEIGHT);
    appCntxt->height[0] = (uint16_t)tmp;

    /* Get stereo algorithm type information */
    m_privNodeHdl.param("sde_algo_type", tmp, 0);
    appCntxt->sdeAlgoType = (uint8_t)tmp;

    /* Get the number of layers information when sde_algo_typ = 1*/
    m_privNodeHdl.param("num_layers", tmp, 2);
    appCntxt->numLayers = (uint8_t)tmp;

    /* Get the median filtering flag information when sde_algo_typ = 1*/
    m_privNodeHdl.param("pp_median_filter_enable", tmp, 0);
    appCntxt->ppMedianFilterEnable = (uint8_t)tmp;

    /* Get the disparity confidence threshold information */
    m_privNodeHdl.param("sde_confidence_threshold", tmp, 0);
    appCntxt->confidence_threshold = (uint8_t)tmp;

    /* SDE Parameters */ 
    /* Get the SDE median filter enable flag information */
    m_privNodeHdl.param("median_filter_enable", tmp, 1);
    appCntxt->sde_params.median_filter_enable = (uint16_t)tmp;

    /* Get the SDE reduced range search enable flag information */
    m_privNodeHdl.param("reduced_range_search_enable", tmp, 0);
    appCntxt->sde_params.reduced_range_search_enable = (uint16_t)tmp;

    /* Get the SDE minimum disparity information */
    m_privNodeHdl.param("disparity_min", tmp, 0);
    appCntxt->sde_params.disparity_min = (uint16_t)tmp;

    /* Get the SDE maximum disparity information */
    m_privNodeHdl.param("disparity_max", tmp, 1);
    appCntxt->sde_params.disparity_max = (uint16_t)tmp;

    /* Get the SDE left-right consistency check threshold information */
    m_privNodeHdl.param("threshold_left_right", tmp, 3);
    appCntxt->sde_params.threshold_left_right = (uint16_t)tmp;

    /* Get the SDE texture based filtering enable flag information */
    m_privNodeHdl.param("texture_filter_enable", tmp, 0);
    appCntxt->sde_params.texture_filter_enable = (uint16_t)tmp;

    /* Get the SDE texture filtering threshold information */
    m_privNodeHdl.param("threshold_texture", tmp, 0);
    appCntxt->sde_params.threshold_texture = (uint16_t)tmp;

    /* Get the SDE aggregation penalty p1 information */
    m_privNodeHdl.param("aggregation_penalty_p1", tmp, 32);
    appCntxt->sde_params.aggregation_penalty_p1 = (uint16_t)tmp;

    /* Get the SDE aggregation penalty p2 information */
    m_privNodeHdl.param("aggregation_penalty_p2", tmp, 197);
    appCntxt->sde_params.aggregation_penalty_p2 = (uint16_t)tmp;

    /* Get pipeline depth information. */
    m_privNodeHdl.param("pipeline_depth", tmp, SDEAPP_MAX_PIPELINE_DEPTH);
    appCntxt->pipelineDepth = (uint8_t)tmp;

    /* Get interactive mode flag information. */
    m_privNodeHdl.param("is_interactive", tmp, 0);
    appCntxt->is_interactive = (uint8_t)tmp;

    /* Get graph export flag information. */
    m_privNodeHdl.param("exportGraph", tmp, 0);
    appCntxt->exportGraph = (uint8_t)tmp;

    /* Get real-time logging enable information. */
    m_privNodeHdl.param("rtLogEnable", tmp, 0);
    appCntxt->rtLogEnable = (uint8_t)tmp;

    if (appCntxt->sdeAlgoType == 1)
    {
        /* Get disparity merge node core information. */
        status = m_privNodeHdl.getParam("disp_merge_deploy_core", str);
        if (status == false)
        {
            ROS_INFO("Config parameter 'disp_merge_deploy_core' not found.");
            exit(-1);
        }

        appCntxt->mlSdeCreateParams.dispMergeNodeCore =  app_common_get_coreName(str.c_str());

        /* Get hole fillinge node core information. */
        status = m_privNodeHdl.getParam("hole_filling_deploy_core", str);
        if (status == false)
        {
            ROS_INFO("Config parameter 'hole_filling_deploy_core' not found.");
            exit(-1);
        }

        appCntxt->mlSdeCreateParams.holeFillingNodeCore = app_common_get_coreName(str.c_str());
    }

    // when multi-layer SDE is used
    if (appCntxt->sdeAlgoType == 1)
    {
        if (appCntxt->numLayers < 2 || appCntxt->numLayers > 3)
        {
            ROS_INFO("The number of layer should be 2 or 3. Set the number of layers to 2...\n");
            appCntxt->numLayers = 2;
        }

        int8_t  i;
        int8_t  factor = (appCntxt->numLayers - 1) * 2;

        if (appCntxt->height[0] % factor != 0 || appCntxt->width[0] % factor != 0)
        {
            ROS_INFO("Improper stereo image resolution...\n");
            exit(-1);
        }

        for (i = 1; i < appCntxt->numLayers; i++)
        {
            appCntxt->width[i]  = appCntxt->width[i-1]/2;
            appCntxt->height[i] = appCntxt->height[i-1]/2;

            if (appCntxt->width[i] % 16 != 0)
            {
                ROS_INFO("Improper image width is not multiple of 16...\n");
                exit(-1);
            }
        }
    }

    appCntxt->renderPeriod = 0;

    return;
}


int ROSAppSDE::rosAppSDEInit()
{
    PTK_CRT    ptkConfig;
    vx_status  vxStatus = VX_SUCCESS;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    /* create appCntxt */
    appCntxt = new SDEAPP_Context();

    if (appCntxt == nullptr)
    {
        ROS_ERROR("new SDEAPP_Context() failed.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Read parameters */
        readParams();

        /* init processFinished */
        appCntxt->processFinished = false;

        /* set defalut params */
        SDEAPP_setAllParams(appCntxt);

        /* Initialize the Application context */
        SDEAPP_init(appCntxt);

        /* Launch processing threads. */
        rosAppSdelaunchProcThreads(appCntxt);
    }

    return 0;
}

void ROSAppSDE::rosAppSDEProcessData(unsigned char* leftImg, unsigned char* rightImg)
{
    if(!appCntxt->processFinished)
    {
        SDEAPP_run(appCntxt, leftImg, rightImg);
    }
}

void ROSAppSDE::rosAppSDEDeInit()
{
    delete appCntxt;
}


void ROSAppSDE::rosAppSDEPublishOutput(int32_t width, int32_t height, int16_t minSdeDisp, int16_t maxSdeDisp, vx_image disparity)
{
    vx_status                  vxStatus;
    vx_map_id                  map_id;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    uint16_t                 * data_ptr;
    int32_t                    i, j;

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = width;
    rect.end_y   = height;

    // raw disparity data
    vxStatus = vxMapImagePatch(disparity,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               (void **)&data_ptr,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);
    PTK_assert(VX_SUCCESS == vxStatus);
    vxUnmapImagePatch(disparity, map_id);


    // Publish disparity data
    ros::Time time = ros::Time::now();

    // Does not work if stride_y is not the same as stride_x*width
    std::vector<uint16_t> vec(data_ptr, data_ptr + width*height);
    disp_msg.width        = width;
    disp_msg.height       = height;
    disp_msg.minDisp      = 0;
    if (maxSdeDisp == 0)
    {
        disp_msg.maxDisp  = 63;
    }
    else if (maxSdeDisp == 1)
    {
        disp_msg.maxDisp  = 127;
    }
    else if (maxSdeDisp == 2)
    {
       disp_msg.maxDisp   = 191;
    }
    disp_msg.step         = width*2;
    disp_msg.data         = vec;
    disp_msg.header.stamp = time;

    disp_pub.publish(disp_msg);
}
