#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.h>
#include <common_msgs/msg/disparity.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs::msg;
using namespace message_filters;

#define clip3(x, min, max) ( x > max? max : (x < min ? min : x))

namespace ti_ros2
{
    using ImgTrans = image_transport::ImageTransport;
    using ImgSub   = message_filters::Subscriber<Image>;
    using ImgPub   = image_transport::Publisher;
    using SubCon   = message_filters::Connection;

    /**
     * @brief Yuv2Rgb class
     */
    class Yuv2Rgb: public rclcpp::Node
    {
        public:
            /**
             * @brief { function_description }
             *
             * @param[in]  resolution  The resolution
             * @param[in]  frame_rate  The frame rate
             */

            Yuv2Rgb(const std::string&          name,
                    const rclcpp::NodeOptions&  options):
                Node(name, options), logger(rclcpp::get_logger("Yuv2Rgb"))
            {
                it = new ImgTrans(static_cast<rclcpp::Node::SharedPtr>(this));

                // get ROS params
                this->get_parameter_or("width", width, 1280);

                this->get_parameter_or("height", height, 720);

                this->get_parameter_or("input_yuv_topic",
                                       input_yuv_topic_,
                                       std::string("camera/right/image_raw"));

                this->get_parameter_or("output_rgb_topic",
                                       output_rgb_topic_,
                                       std::string("camera/right/image_raw_rgb"));

                this->get_parameter_or("yuv_format",
                                       yuv_format_,  std::string("YUV420"));

                this->get_parameter_or("yuv420_luma_only",
                                       yuv420_luma_only_,  false);

                if ((yuv_format_.compare("YUV420") != 0) &&
                    (yuv_format_.compare("YUV422") != 0))
                {
                    RCLCPP_ERROR(logger, "yuv_format should be either YUV420 or YUV422 (UYVY).");
                    exit(-1);
                }

                // allocation for output RGB images
                outRgbIm = (uint8_t *)malloc(width * height * 3);

                out_pub  = it->advertise(output_rgb_topic_, 1);
                in_sub   = new ImgSub(this, input_yuv_topic_);
                conObj   = in_sub->registerCallback(&Yuv2Rgb::callback_yuv2rgb, this);

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~Yuv2Rgb()
            {
                conObj.disconnect();

                // free outRgbIm
                if (outRgbIm != NULL)
                {
                    free(outRgbIm);
                }
            }

            void callback_yuv2rgb(const Image::ConstSharedPtr& yuv_image_ptr)
            {
                Image yuv_image  = *yuv_image_ptr;

                // Sometimes, output image can be smaller than input raw image, e.g.
                // input yuv image is 1920x1080, but output rgb image is 1920x1024, which is disparity map size.
                // But output image cannot be larger than input raw image.
                if (yuv_image.width < width || yuv_image.height < height)
                {
                    RCLCPP_INFO(logger, "Output image size is less than source image size.");
                    exit(-1);
                }

                // Color conversion from YUV to RGB
                if (yuv_format_.compare("YUV420") == 0)
                {
                    if (yuv_image.encoding != "yuv420")
                    {
                        RCLCPP_ERROR(logger, "Image encoding does not match: %s", yuv_image.encoding.c_str());
                        exit(-1);
                    }

                    convertYUV420RGB(outRgbIm,
                                    (uint8_t *) yuv_image.data.data(),
                                    width,
                                    height,
                                    yuv_image.width,
                                    yuv_image.height,
                                    yuv420_luma_only_);
                }
                else
                {
                    if (yuv_image.encoding != "yuv422")
                    {
                        RCLCPP_ERROR(logger, "Image encoding does not match: %s", yuv_image.encoding.c_str());
                        exit(-1);
                    }

                    convertYUV422RGB(outRgbIm,
                                    (uint8_t *) yuv_image.data.data(),
                                    width,
                                    height,
                                    yuv_image.width,
                                    yuv_image.height);
                }

                // Publish RGB image
                std::vector<uint8_t> vec_rgb(outRgbIm, outRgbIm + width*height*3);
                out_msg.width           = width;
                out_msg.height          = height;
                out_msg.encoding        = "rgb8";
                out_msg.step            = width*3;
                out_msg.data            = vec_rgb;
                out_msg.header.stamp    = yuv_image.header.stamp;
                out_msg.header.frame_id = "map";
                out_pub.publish(out_msg);
            }

            /*
            * Convert YUV420 (NV12) to planar RGB
            * If yuv420_luma_only = true, uses Luma channel only and outputs gray RGB image (R=G=B)
            */
        void convertYUV420RGB(uint8_t * dstImage,
                              uint8_t * srcImage,
                              int32_t   dstWidth,
                              int32_t   dstHeight,
                              int32_t   srcWidth,
                              int32_t   srcHeight,
                              bool      yuv420_luma_only)
            {
                int32_t i, j;
                int32_t y, cb, cr;
                int32_t r, g,  b;
                int32_t hOfst, vOfst;
                uint8_t * src_ptr_y, *src_ptr_uv;
                uint8_t * dst_ptr_r, *dst_ptr_g, *dst_ptr_b;

                hOfst = (srcWidth  - dstWidth)  >> 1;
                vOfst = (srcHeight - dstHeight) >> 1;

                src_ptr_y  = srcImage;
                src_ptr_uv = srcImage + width*height;

                dst_ptr_r  = dstImage;
                dst_ptr_g  = dstImage + 1;
                dst_ptr_b  = dstImage + 2;

                if (yuv420_luma_only==false)
                {
                    for (j = vOfst; j < vOfst + height; j++)
                    {
                        for (i = hOfst; i < hOfst + width; i++)
                        {
                            y  = src_ptr_y[j * srcWidth + i];
                            cb = src_ptr_uv[(j >> 1)*srcWidth + (i>>1)*2];
                            cr = src_ptr_uv[(j >> 1)*srcWidth + (i>>1)*2 + 1];

                            y  = y  - 16;
                            cb = cb - 128;
                            cr = cr - 128;

                            r = clip3((298*y + 409*cr + 128) >> 8, 0, 255);
                            g = clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
                            b = clip3((298*y + 516*cb + 128) >> 8, 0, 255);

                            *dst_ptr_r = (uint8_t)r; dst_ptr_r += 3;
                            *dst_ptr_g = (uint8_t)g; dst_ptr_g += 3;
                            *dst_ptr_b = (uint8_t)b; dst_ptr_b += 3;

                        }

                    }

                }
                else // yuv420_luma_only==true
                {
                    for (j = vOfst; j < vOfst + height; j++)
                    {
                        for (i = hOfst; i < hOfst + width; i++)
                        {
                            y  = src_ptr_y[j * srcWidth + i];
                            y  = y  - 16;

                            r = clip3((298*y + 128) >> 8, 0, 255);
                            g = r;
                            b = r;

                            *dst_ptr_r = (uint8_t)r; dst_ptr_r += 3;
                            *dst_ptr_g = (uint8_t)g; dst_ptr_g += 3;
                            *dst_ptr_b = (uint8_t)b; dst_ptr_b += 3;

                        } // for (i = hOfst; i < hOfst + width; i++)

                    } // for (j = vOfst; j < vOfst + height; j++)

                } // if (yuv420_luma_only==false) else

            } // convertYUV420RGB()

            /*
            * Convert YUV422 (UYVY) to planar RGB
            */
            void convertYUV422RGB(uint8_t * dstImage,
                                uint8_t * srcImage,
                                int32_t   dstWidth,
                                int32_t   dstHeight,
                                int32_t   srcWidth,
                                int32_t   srcHeight)
            {
                int32_t i, j, n;
                int32_t y, cb, cr;
                int32_t r, g,  b;
                int32_t hOfst, vOfst;
                int32_t srcStride;
                uint8_t * src_ptr;
                uint8_t * dst_ptr_r, *dst_ptr_g, *dst_ptr_b;

                hOfst = (srcWidth  - dstWidth)  >> 1;
                vOfst = (srcHeight - dstHeight) >> 1;

                src_ptr   = srcImage;

                dst_ptr_r = dstImage;
                dst_ptr_g = dstImage + 1;
                dst_ptr_b = dstImage + 2;

                srcStride = srcWidth * 2;

                for (j = vOfst; j < vOfst + height; j++)
                {
                    for (i = hOfst; i < hOfst + width; i+=2)
                    {
                        for (n = 0; n < 2; n++)
                        {
                            // U (Cb)
                            cb = src_ptr[j*srcStride + (i<<1)];
                            // V (Cr)
                            cr = src_ptr[j*srcStride + (i<<1)+2];

                            // Y
                            y  = src_ptr[j*srcStride + (i<<1)+ (n << 1) + 1];

                            // create RGB
                            y  = y  - 16;
                            cb = cb - 128;
                            cr = cr - 128;

                            r = clip3((298*y + 409*cr + 128) >> 8, 0, 255);
                            g = clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
                            b = clip3((298*y + 516*cb + 128) >> 8, 0, 255);

                            *dst_ptr_r = (uint8_t)r; dst_ptr_r += 3;
                            *dst_ptr_g = (uint8_t)g; dst_ptr_g += 3;
                            *dst_ptr_b = (uint8_t)b; dst_ptr_b += 3;
                        } // for (n = 0; n < 2; n++)

                    } // for (i = hOfst; i < hOfst + width; i+=2)

                } // for (j = vOfst; j < vOfst + height; j++)

            } // convertYUV422RGB()

        private:
            ImgSub         *in_sub;
            ImgPub          out_pub;
            SubCon          conObj;
            ImgTrans       *it;
            Image           out_msg;
            uint8_t        *outRgbIm;
            std::string     input_yuv_topic_;
            std::string     output_rgb_topic_;
            std::string     yuv_format_;
            bool            yuv420_luma_only_{false};
            int32_t         width;
            int32_t         height;
            rclcpp::Logger  logger;
    };
}

//static std::shared_ptr<ti_ros2::Yuv2Rgb>  colorConv = nullptr;
static ti_ros2::Yuv2Rgb *colorConv = nullptr;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    try
    {
        rclcpp::NodeOptions options;

        rclcpp::init(argc, argv);

        options.allow_undeclared_parameters(true);
        options.automatically_declare_parameters_from_overrides(true);

        colorConv = new ti_ros2::Yuv2Rgb("viz_color_conv_yuv2rgb", options);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

}


