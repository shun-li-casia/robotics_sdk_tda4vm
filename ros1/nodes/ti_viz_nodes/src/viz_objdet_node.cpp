#include <stdio.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <common_msgs/Detection2D.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "viz_objdet.h"

using namespace sensor_msgs;
using namespace common_msgs;
using namespace message_filters;
using namespace cv;

static cv::Mat overlayBoundingBox(cv::Mat &img,
                                  const int32_t *box,
                                  const int32_t label_id)
{
    int32_t label_id_mod = label_id % 20;
    Scalar box_color = Scalar(color_map[label_id_mod][0],
                              color_map[label_id_mod][1],
                              color_map[label_id_mod][2]);
    Scalar text_color    = Scalar(244,  35, 232);
    Scalar text_bg_color = Scalar(120, 120, 120);
    std::string label = classnames_coco[label_id];

    // Draw bounding box for the detected object
    Point topleft     = Point(box[0], box[1]);
    Point bottomright = Point(box[2], box[3]);
    rectangle(img, topleft, bottomright, box_color, 3);

    // Draw text with detected class with a background box
    Point center        = Point((box[0] + box[2])/2, (box[1] + box[3])/2);
    Point t_topleft     = center + Point(-5, -10);
    Point t_bottomright = center + Point( 80, 10);
    rectangle(img, t_topleft, t_bottomright, text_bg_color, -1);
    putText(img, label, t_topleft + Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.6, text_color);

    return img;
}

namespace ros_app_viz
{
    /**
     * @brief  VizObjDet ROS warpper class
     */

    class VizObjDet
    {
    public:
        /**
         * @brief { Overlays the bounding boxes on the image }
         *
         */

        VizObjDet(ros::NodeHandle *nh, ros::NodeHandle *private_nh)
        {
            std::string rectImgTopic;
            std::string tensorTopic;
            std::string odMapImgTopic;

            // input topics
            private_nh->param("rectified_image_topic", rectImgTopic, std::string(""));
            private_nh->param("vision_cnn_tensor_topic", tensorTopic, std::string(""));

            // output topics
            private_nh->param("vision_cnn_image_topic", odMapImgTopic, std::string(""));

            m_odMapImgPub = nh->advertise<Image>(odMapImgTopic, 1);

            message_filters::Subscriber<Detection2D> tensorSub(*nh, tensorTopic, 1);
            message_filters::Subscriber<Image> rectImgSub(*nh, rectImgTopic, 1);

            TimeSynchronizer<Detection2D, Image> sync(tensorSub, rectImgSub, 10);
            sync.registerCallback(boost::bind(&VizObjDet::callback_vizObjDet, this, _1, _2));

            ros::spin();
        }

        ~VizObjDet()
        {

        }

        void callback_vizObjDet(const Detection2D::ConstPtr& detMsg,
                                const ImageConstPtr& imagePtr)
        {

            cv_bridge::CvImagePtr cv_outImgPtr;
            cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);

            int32_t box[4];
            int32_t class_id;
            int32_t num_objects = detMsg->num_objects;

            for (int32_t i = 0; i < num_objects; i++)
            {
                box[0] = detMsg->bounding_boxes[i].xmin;
                box[1] = detMsg->bounding_boxes[i].ymin;
                box[2] = detMsg->bounding_boxes[i].xmax;
                box[3] = detMsg->bounding_boxes[i].ymax;
                class_id = detMsg->bounding_boxes[i].label_id;

                overlayBoundingBox(cv_outImgPtr->image, box, class_id);
            }

            m_odMapImgPub.publish(cv_outImgPtr->toImageMsg());
        }

    private:
        ros::Publisher m_odMapImgPub;
    };
}

/**
 * Main
 */
int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "app_viz_objdet");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        ros_app_viz::VizObjDet objDetViz(&nh, &private_nh);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }
}
