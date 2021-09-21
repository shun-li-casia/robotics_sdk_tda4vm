#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>

#include <common_msgs/msg/detection2_d.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace message_filters;
using namespace cv;

using std::placeholders::_1;
using std::placeholders::_2;

// OpenCV uses BGR format
static const uint8_t color_map[20][3] =
{
    {128,  64, 128},{244,  35, 232},{ 70,  70,  70},{102, 102, 156},{190, 153, 153},
    {153, 153, 153},{250, 170,  30},{220, 220,   0},{107, 142,  35},{152, 251, 152},
    { 70, 130, 180},{220,  20,  60},{255,   0,   0},{  0,   0, 142},{  0,   0,  70},
    {  0,  60, 100},{  0,  80, 100},{  0,   0, 230},{119,  11,  32},{128, 128, 128}
};

static const std::string classnames_coco[] =
{
    [0] = "None",
    [1] = "person",
    [2] = "bicycle",
    [3] = "car",
    [4] = "motorcycle",
    [5] = "airplane",
    [6] = "bus",
    [7] = "train",
    [8] = "truck",
    [9] = "boat",
    [10] = "traffic light",
    [11] = "fire hydrant",
    [12] = "street sign",
    [13] = "stop sign",
    [14] = "parking meter",
    [15] = "bench",
    [16] = "bird",
    [17] = "cat",
    [18] = "dog",
    [19] = "horse",
    [20] = "sheep",
    [21] = "cow",
    [22] = "elephant",
    [23] = "bear",
    [24] = "zebra",
    [25] = "giraffe",
    [26] = "hat",
    [27] = "backpack",
    [28] = "umbrella",
    [29] = "shoe",
    [30] = "eye glasses",
    [31] = "handbag",
    [32] = "tie",
    [33] = "suitcase",
    [34] = "frisbee",
    [35] = "skis",
    [36] = "snowboard",
    [37] = "sports ball",
    [38] = "kite",
    [39] = "baseball bat",
    [40] = "baseball glove",
    [41] = "skateboard",
    [42] = "surfboard",
    [43] = "tennis racket",
    [44] = "bottle",
    [45] = "plate",
    [46] = "wine glass",
    [47] = "cup",
    [48] = "fork",
    [49] = "knife",
    [50] = "spoon",
    [51] = "bowl",
    [52] = "banana",
    [53] = "apple",
    [54] = "sandwich",
    [55] = "orange",
    [56] = "broccoli",
    [57] = "carrot",
    [58] = "hot dog",
    [59] = "pizza",
    [60] = "donut",
    [61] = "cake",
    [62] = "chair",
    [63] = "couch",
    [64] = "potted plant",
    [65] = "bed",
    [66] = "mirror",
    [67] = "dining table",
    [68] = "window",
    [69] = "desk",
    [70] = "toilet",
    [71] = "door",
    [72] = "tv",
    [73] = "laptop",
    [74] = "mouse",
    [75] = "remote",
    [76] = "keyboard",
    [77] = "cell phone",
    [78] = "microwave",
    [79] = "oven",
    [80] = "toaster",
    [81] = "sink",
    [82] = "refrigerator",
    [83] = "blender",
    [84] = "book",
    [85] = "clock",
    [86] = "vase",
    [87] = "scissors",
    [88] = "teddy bear",
    [89] = "hair drier",
    [90] = "toothbrush",
    [91] = "hair brush"
};

static cv::Mat overlayBoundingBox(cv::Mat    &img,
                            int32_t          *box,
                            int32_t          label_id)
{
    // Mat img = Mat(outDataHeight, outDataWidth, CV_8UC3, frame);
    int32_t label_id_mod = label_id % 20;
    Scalar box_color     = (color_map[label_id_mod][0], color_map[label_id_mod][1], color_map[label_id_mod][2]);
    Scalar text_color    = (240, 240, 240);
    Scalar text_bg_color = (128, 128, 128);
    // Scalar text_color = box_color;

    std::string label = classnames_coco[label_id];

    // Draw bounding box for the detected object
    Point topleft     = Point(box[0], box[1]);
    Point bottomright = Point(box[2], box[3]);
    rectangle(img, topleft, bottomright, box_color, 3);

    // Draw text with detected class with a background box
    Point t_topleft     = Point((box[0] + box[2])/2 - 5, (box[1] + box[3])/2 + 5);
    Point t_bottomright = Point((box[0] + box[2])/2 + 80, (box[1] + box[3])/2 - 15);
    rectangle(img, t_topleft, t_bottomright, text_bg_color, -1);

    // putText(img, objectname, t_topleft, FONT_HERSHEY_SIMPLEX, 0.5, text_color);
    putText(img, label, t_topleft, FONT_HERSHEY_SIMPLEX, 0.6, text_color);

    return img;
}

namespace ti_ros2
{
    using ImgTrans = image_transport::ImageTransport;

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

            VizObjDet(const std::string&            name,
                      const rclcpp::NodeOptions&    options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string tensorTopic;
                std::string odMapImgTopic;

                m_it = new ImgTrans(static_cast<rclcpp::Node::SharedPtr>(this));

                // input topics
                this->get_parameter_or("rectified_image_topic",   rectImgTopic, std::string(""));
                this->get_parameter_or("vision_cnn_tensor_topic", tensorTopic,  std::string(""));

                // output topics
                this->get_parameter_or("vision_cnn_image_topic", odMapImgTopic, std::string(""));

                m_odMapImgPub = m_it->advertise(odMapImgTopic, 1);

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

                m_odMapImgPub.publish(imgPtr);
            }

        private:
            ImgTrans                       *m_it;
            image_transport::Publisher      m_odMapImgPub;
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
        rclcpp::NodeOptions options;

        rclcpp::init(argc, argv);

        options.allow_undeclared_parameters(true);
        options.automatically_declare_parameters_from_overrides(true);

        objDetViz = new ti_ros2::VizObjDet("viz_objdet", options);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

