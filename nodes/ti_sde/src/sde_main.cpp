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
#include <stdio.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


#include <sde_node.h>


using namespace sensor_msgs;
using namespace message_filters;


namespace ros_app_sde
{
    /**
     * @brief  SDE ROS warpper class
     */

    class StereoROS
    {
    public:
        /**
         * @brief { function_description }
         *
         * @param[in]  resolution  The resolution
         * @param[in]  frame_rate  The frame rate
         */

        StereoROS() : it(nh)
        {
            ros::NodeHandle private_nh("~");

            private_nh.param("left_input_topic", left_input_topic_, std::string(""));
            private_nh.param("right_input_topic", right_input_topic_, std::string(""));
            private_nh.param("disparity_topic", disparity_topic_, std::string(""));

            /*****************************************/
            /* OpenVX Graph Init                     */
            /*****************************************/
            rosAppSde = new ROSAppSDE(private_nh, disparity_topic_);
        }

        ~StereoROS()
        {
            /*****************************************/
            /*  OpenVX Graph Deinit                  */
            /*****************************************/
            delete rosAppSde;
        }

        void run()
        {
            // Read streo images from rosbag file
            message_filters::Subscriber<Image> left_imag_sub(nh, left_input_topic_, 1);
            message_filters::Subscriber<Image> right_imag_sub(nh, right_input_topic_, 1);

            TimeSynchronizer<Image, Image> sync(left_imag_sub, right_imag_sub, 10);
            sync.registerCallback(boost::bind(&StereoROS::callback_stereo, this, _1, _2));
            
            // question: how to do rate control?, is callback blocking function?
            ros::spin();
        }

        void callback_stereo(const ImageConstPtr& left_image_ptr, const ImageConstPtr& right_image_ptr)
        {
            sensor_msgs::Image left_image  = *left_image_ptr;
            sensor_msgs::Image right_image = *right_image_ptr;


            // Conver to monochrome image to run OpenVX graph
            // ZED captured images should be monochrome by itself
            cv_bridge::CvImagePtr cv_ptr_l, cv_ptr_r;

            // left image
            cv_ptr_l = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::YUV422);

            // right image
            cv_ptr_r = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::YUV422);

            /*****************************************/
            /* Process OpenVX Graph                  */
            /*****************************************/
            rosAppSde->rosAppSDEProcessData(cv_ptr_l->image.data, cv_ptr_r->image.data);
        }

        ROSAppSDE * getROSAppSDE()
        {
            return rosAppSde;
        }


    private:

        std::string left_input_topic_;
        std::string right_input_topic_;
        std::string disparity_topic_;

        ROSAppSDE  *rosAppSde;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
    };
}

ros_app_sde::StereoROS * sdeRos;

void sde_intSigHandler(int32_t sig)
{
    if (sdeRos != nullptr)
    {
        ROSAppSDE * rosAppSde = sdeRos->getROSAppSDE();
        if (rosAppSde != nullptr)
        {
            SDEAPP_Context * appCntxt  = rosAppSde->getSDEAPPContext();
            SDEAPP_intSigHandler(appCntxt, sig);
        }
    }

    ros::shutdown();
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "ros_app_sde", ros::init_options::NoSigintHandler);
        signal(SIGINT, sde_intSigHandler);

        // To make intSigHandler work, we create a StereoROS object first,
        // then call run() to read input data from topics using ros::spin()
        // If we call ros::spin() in StereoROS constructor,
        // sdeRos is nullptr always in intSigHandler
        sdeRos = new ros_app_sde::StereoROS();
        sdeRos->run();

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }

}


