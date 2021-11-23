#include <stdio.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace message_filters;

#define clip3(x, min, max) ( x > max? max : (x < min ? min : x))

namespace ros_app_yuv2rgb
{
    /**
     * @brief Yuv2Rgb class
     */

    class Yuv2Rgb
    {
    public:
        /**
         * @brief { function_description }
         *
         * @param[in]  resolution  The resolution
         * @param[in]  frame_rate  The frame rate
         */

        Yuv2Rgb(ros::NodeHandle *nh, ros::NodeHandle *private_nh)
        {
            // get ROS params
            private_nh->param("width", width, 1280);
            private_nh->param("height", height, 720);
            private_nh->param("input_yuv_topic", input_yuv_topic_, std::string("camera/right/image_raw"));
            private_nh->param("output_rgb_topic", output_rgb_topic_, std::string("camera/right/image_raw_rgb"));
            private_nh->param("yuv_format", yuv_format_,  std::string("YUV420"));
            private_nh->param("yuv420_luma_only", yuv420_luma_only_, false);

            if (yuv_format_.compare("YUV420") != 0 && yuv_format_.compare("YUV422") != 0)
            {
                ROS_INFO("yuv_format should be either YUV420 or YUV422 (UYVY).");
                exit(-1);
            }

            // allocation for output RGB images
            outRgbIm = (uint8_t *) malloc(width * height * 3);

            out_pub  = nh->advertise<Image>(output_rgb_topic_, 10);
            in_sub   = nh->subscribe(input_yuv_topic_, 1, &Yuv2Rgb::callback_yuv2rgb, this);

            ros::spin();
        }

        ~Yuv2Rgb()
        {
            // free outRgbIm
            if (outRgbIm != NULL)
            {
                free(outRgbIm);
            }
        }

        void callback_yuv2rgb(const ImageConstPtr& yuv_image_ptr)
        {
            sensor_msgs::Image yuv_image  = *yuv_image_ptr;

            // Sometimes, output image can be smaller than input raw image, e.g.
            // input yuv image is 1920x1080, but output rgb image is 1920x1024, which is disparity map size.
            // But output image cannot be larger than input raw image.
            if (yuv_image.width < width || yuv_image.height < height)
            {
                ROS_INFO("Output image size is less than source image size.");
                exit(-1);
            }

            // Color conversion from YUV to RGB
            if (yuv_format_.compare("YUV420") == 0)
            {
                if (yuv_image.encoding != "yuv420")
                {
                    ROS_ERROR("Image encoding does not match: %s", yuv_image.encoding.c_str());
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
            else // YUV422
            {
                if (yuv_image.encoding != "yuv422")
                {
                    ROS_ERROR("Image encoding does not match: %s", yuv_image.encoding.c_str());
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
            out_msg.width        = width;
            out_msg.height       = height;
            out_msg.encoding     = "rgb8";
            out_msg.step         = width*3;
            out_msg.data         = vec_rgb;
            out_msg.header.stamp = yuv_image.header.stamp;
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

                    } // for (i = hOfst; i < hOfst + width; i++)

                } // for (j = vOfst; j < vOfst + height; j++)

            } // f (yuv420_luma_only==false)
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
        ros::Subscriber                 in_sub;
        ros::Publisher                  out_pub;
        sensor_msgs::Image              out_msg;
        uint8_t                       * outRgbIm;
        std::string                     input_yuv_topic_;
        std::string                     output_rgb_topic_;
        std::string                     yuv_format_;
        bool                            yuv420_luma_only_;
        int32_t                         width;
        int32_t                         height;
    };
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "ros_app_yuv2rgb");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        ros_app_yuv2rgb::Yuv2Rgb colorConv(&nh, &private_nh);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }

}


