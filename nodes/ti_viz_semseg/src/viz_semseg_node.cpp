#include <stdio.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


using namespace sensor_msgs;
using namespace message_filters;


static const uint8_t color_map[20][3] =
{{128,  64, 128},{244,  35, 232},{ 70,  70,  70},{102, 102, 156},{190, 153, 153},
 {153, 153, 153},{250, 170,  30},{220, 220,   0},{107, 142,  35},{152, 251, 152},
 { 70, 130, 180},{220,  20,  60},{255,   0,   0},{  0,   0, 142},{  0,   0,  70},
 {  0,  60, 100},{  0,  80, 100},{  0,   0, 230},{119,  11,  32},{128, 128, 128}};

namespace ros_app_viz
{
    /**
     * @brief  SDE ROS warpper class
     */

    class VizSemSeg
    {
    public:
        /**
         * @brief { function_description }
         *
         * @param[in]  resolution  The resolution
         * @param[in]  frame_rate  The frame rate
         */

        VizSemSeg() : m_it(m_nh)
        {
            ros::NodeHandle private_nh("~");

            std::string rectImgTopic;
            std::string ssTensorTopic;
            std::string ssMapImgTopic;

            // input and tensor sizes
            private_nh.param("width",            m_width,        1280);
            private_nh.param("height",           m_height,       720);
            private_nh.param("tensor_width",     m_tensorWidth,  768);
            private_nh.param("tensor_height",    m_tensorHeight, 432);

            // input topics
            private_nh.param("rectified_image_topic",    rectImgTopic,  std::string(""));
            private_nh.param("semseg_cnn_tensor_topic",  ssTensorTopic, std::string(""));

            // output topics
            private_nh.param("semseg_cnn_image_topic",   ssMapImgTopic, std::string(""));

            m_ssMapImgPub = m_it.advertise(ssMapImgTopic, 1);

            message_filters::Subscriber<Image> ssTensorSub(m_nh, ssTensorTopic, 1);
            message_filters::Subscriber<Image> rectImgSub(m_nh, rectImgTopic, 1);

            TimeSynchronizer<Image, Image> sync(ssTensorSub, rectImgSub, 10);
            sync.registerCallback(boost::bind(&VizSemSeg::callback_vizSemSeg, this, _1, _2));

            ros::spin();
        }

        ~VizSemSeg()
        {

        }

        void callback_vizSemSeg(const ImageConstPtr& tensorPtr,
                                const ImageConstPtr& imagePtr)
        {
            // position in original image
            int32_t i, j;
            // position in tensor
            int32_t y, x;
            // Semantic class
            uint8_t classId;

            int32_t stride = m_width * 3;

            float   horScale = m_tensorWidth *1.0/m_width;
            float   verScale = m_tensorHeight*1.0/m_height;

            cv_bridge::CvImagePtr cv_ssPtr;
            cv_ssPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::BGR8);

            for (j = 0; j < m_height; j++)
            {
                // y pos in tensor
                y = (j * verScale + 0.5);
                for (i = 0; i < m_width; i++)
                {
                    // x pos in tensor
                    x = (i * horScale + 0.5);
                    classId =  tensorPtr->data[y*m_tensorWidth + x];

                    cv_ssPtr->image.data[j*stride + i*3]     =
                        0.5*cv_ssPtr->image.data[j*stride + i*3]     + 0.5*color_map[classId][0];
                    cv_ssPtr->image.data[j*stride + i*3 + 1] =
                        0.5*cv_ssPtr->image.data[j*stride + i*3 + 1] + 0.5*color_map[classId][1];
                    cv_ssPtr->image.data[j*stride + i*3 + 2] =
                        0.5*cv_ssPtr->image.data[j*stride + i*3 + 2] + 0.5*color_map[classId][2];
                }
            }

            m_ssMapImgPub.publish(cv_ssPtr->toImageMsg());
        }

    private:
        int32_t                         m_width;
        int32_t                         m_height;
        int32_t                         m_tensorWidth;
        int32_t                         m_tensorHeight;

        ros::NodeHandle                 m_nh;
        image_transport::ImageTransport m_it;

        image_transport::Publisher      m_ssMapImgPub;
    };
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "app_viz_semseg");
        ros_app_viz::VizSemSeg semSegViz;

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }

}


