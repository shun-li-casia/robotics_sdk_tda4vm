#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.h>
#include <common_msgs/msg/disparity.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs::msg;
using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;

// OpenCV uses BGR format
static const uint8_t color_map[20][3] =
{{128,  64, 128},{244,  35, 232},{ 70,  70,  70},{102, 102, 156},{190, 153, 153},
 {153, 153, 153},{250, 170,  30},{220, 220,   0},{107, 142,  35},{152, 251, 152},
 { 70, 130, 180},{220,  20,  60},{255,   0,   0},{  0,   0, 142},{  0,   0,  70},
 {  0,  60, 100},{  0,  80, 100},{  0,   0, 230},{119,  11,  32},{128, 128, 128}};

static void sigHandler(int32_t sig)
{
    (void) sig;
    std::exit(EXIT_SUCCESS);
}

namespace ti_ros2
{
    using ImgTrans = image_transport::ImageTransport;
    using ImgPub   = image_transport::Publisher;

    /**
     * @brief  VizSemSeg ROS warpper class
     */

    class VizSemSeg: public rclcpp::Node
    {
        public:
            VizSemSeg(const std::string&            name,
                      const rclcpp::NodeOptions&    options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string ssTensorTopic;
                std::string ssMapImgTopic;

                m_it = new ImgTrans(static_cast<rclcpp::Node::SharedPtr>(this));

                // input topics
                this->get_parameter_or("rectified_image_topic",   rectImgTopic,  std::string(""));
                this->get_parameter_or("vision_cnn_tensor_topic", ssTensorTopic, std::string(""));

                // output topics
                this->get_parameter_or("vision_cnn_image_topic", ssMapImgTopic, std::string(""));

                m_ssMapImgPub = m_it->advertise(ssMapImgTopic, 1);

                message_filters::Subscriber<Image> ssTensorSub(this, ssTensorTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);

                TimeSynchronizer<Image, Image> sync(ssTensorSub, rectImgSub, 10);
                sync.registerCallback(std::bind(&VizSemSeg::callback_vizSemSeg, this, _1, _2));

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizSemSeg()
            {
            }

            void callback_vizSemSeg(const Image::ConstSharedPtr& tensorPtr,
                                    const Image::ConstSharedPtr& imagePtr)
            {
                // position in original image
                int32_t i, j;
                // position in tensor
                int32_t y, x;
                // Semantic class
                uint8_t classId;

                // input image size
                int32_t width  = imagePtr->width;
                int32_t height = imagePtr->height;

                // tensor size
                int32_t tensorWidth  = tensorPtr->width;
                int32_t tensorHeight = tensorPtr->height;

                // blending factor
                float bf = 0.5;

                int32_t stride = width * 3;

                float   horScale = tensorWidth *1.0/width;
                float   verScale = tensorHeight*1.0/height;

                cv_bridge::CvImagePtr cv_ssPtr;
                cv_ssPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);

                for (j = 0; j < height; j++)
                {
                    // y pos in tensor
                    y = (j * verScale + 0.5);
                    for (i = 0; i < width; i++)
                    {
                        // x pos in tensor
                        x = (i * horScale + 0.5);
                        classId =  tensorPtr->data[y*tensorWidth + x];

                        cv_ssPtr->image.data[j*stride + i*3]     =
                            (1-bf)*cv_ssPtr->image.data[j*stride + i*3]     + bf*color_map[classId][0];
                        cv_ssPtr->image.data[j*stride + i*3 + 1] =
                            (1-bf)*cv_ssPtr->image.data[j*stride + i*3 + 1] + bf*color_map[classId][1];
                        cv_ssPtr->image.data[j*stride + i*3 + 2] =
                            (1-bf)*cv_ssPtr->image.data[j*stride + i*3 + 2] + bf*color_map[classId][2];
                    }
                }

		auto imgPtr = cv_ssPtr->toImageMsg();
		auto hdr = &imgPtr->header;

		hdr->frame_id = "map";

                m_ssMapImgPub.publish(imgPtr);
            }

        private:
            ImgTrans   *m_it;
            ImgPub      m_ssMapImgPub;
    };
}

static ti_ros2::VizSemSeg  *semSegViz = nullptr;

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

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);

        signal(SIGINT, sigHandler);

        semSegViz = new ti_ros2::VizSemSeg("viz_semseg", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

