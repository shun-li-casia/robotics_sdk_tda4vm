#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include <common_msgs/msg/detection2_d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "viz_objdet.h"

using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace message_filters;
using namespace cv;

using std::placeholders::_1;
using std::placeholders::_2;

static void sigHandler(int32_t sig)
{
    (void) sig;
    rclcpp::shutdown();
}

static cv::Mat overlayBoundingBox(cv::Mat &img,
                                  int32_t *box,
                                  int32_t label_id)
{
    // Mat img = Mat(outDataHeight, outDataWidth, CV_8UC3, frame);
    int32_t label_id_mod = label_id % 20;
    Scalar box_color     = Scalar(color_map[label_id_mod][0],
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

namespace ti_ros2
{
    /**
     * @brief  VizObjDet ROS warpper class
     */

    class VizObjDet: public rclcpp::Node
    {
        public:
            /**
             * @brief { Overlays the bounding boxes on the image }
             *
             */

            VizObjDet(const std::string &name,
                      const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string tensorTopic;
                std::string odMapImgTopic;

                // input topics
                get_parameter_or("rectified_image_topic",   rectImgTopic, std::string(""));
                get_parameter_or("vision_cnn_tensor_topic", tensorTopic,  std::string(""));

                // output topics
                get_parameter_or("vision_cnn_image_topic", odMapImgTopic, std::string(""));

                m_odMapImgPub = this->create_publisher<Image>(odMapImgTopic, 10);

                message_filters::Subscriber<Detection2D> tensorSub(this, tensorTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);

                TimeSynchronizer<Detection2D, Image> sync(tensorSub, rectImgSub, 10);
                sync.registerCallback(std::bind(&VizObjDet::callback_vizObjDet, this, _1, _2));

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizObjDet()
            {
            }

            void callback_vizObjDet(const Detection2D::ConstSharedPtr& detMsg,
                                    const Image::ConstSharedPtr& imagePtr)
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

		        auto imgPtr = cv_outImgPtr->toImageMsg();
		        auto hdr = &imgPtr->header;

		        hdr->frame_id = "map";

                m_odMapImgPub->publish(*imgPtr);
            }

        private:
            rclcpp::Publisher<Image>::SharedPtr m_odMapImgPub;
    };
}

static ti_ros2::VizObjDet  *objDetViz = nullptr;

/**
 * Main
 */
int main(int argc, char **argv)
{
    try
    {
        rclcpp::InitOptions initOptions{};
        rclcpp::NodeOptions nodeOptions{};

        /* Prevent the RCLCPP signal handler binding. */
        initOptions.shutdown_on_sigint = false;

        rclcpp::init(argc, argv, initOptions);

        signal(SIGINT, sigHandler);

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);
        nodeOptions.use_intra_process_comms(false);

        objDetViz = new ti_ros2::VizObjDet("viz_objdet", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

