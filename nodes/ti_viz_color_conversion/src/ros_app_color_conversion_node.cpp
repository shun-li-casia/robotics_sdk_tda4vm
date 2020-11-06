#include <stdio.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>



using namespace sensor_msgs;
using namespace message_filters;


#define clip3(x, min, max) ( x > max? max : (x < min ? min : x))

namespace ros_app_yuv2rgb
{
    /**
     * @brief  SDE ROS warpper class
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

        Yuv2Rgb() : it(nh)
        {
            ros::NodeHandle private_nh("~");

            // get ROS params
            private_nh.param("width", width, 1280);
            private_nh.param("height", height, 720);
            private_nh.param("input_yuv_topic", input_yuv_topic_, std::string("camera/right/image_raw"));
            private_nh.param("output_rgb_topic", output_rgb_topic_, std::string("camera/right/image_raw_rgb"));
            private_nh.param("yuv_format", yuv_format_,  std::string("YUV420"));

            if (yuv_format_.compare("YUV420") != 0 && yuv_format_.compare("YUV422") != 0)
            {
                ROS_INFO("yuv_format should be either YUV420 or YUV422 (UYVY).");
                exit(-1);
            }

            // allocation for output RGB images
            outRgbIm = (uint8_t *)malloc(width * height * 3);

            out_pub  = it.advertise(output_rgb_topic_, 1);
            in_sub   = nh.subscribe(input_yuv_topic_, 1, &Yuv2Rgb::callback_yuv2rgb, this);

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

            // Color conversion from YUV to RGB
            if (yuv_format_.compare("YUV420") == 0)
            {
                convertYUV420RGB(outRgbIm,  (uint8_t *) yuv_image.data.data(),  yuv_image.width,  yuv_image.height);
            }
            else 
            {
                convertYUV422RGB(outRgbIm,  (uint8_t *) yuv_image.data.data(),  yuv_image.width,  yuv_image.height);
            }

            // Publish RGB image
            std::vector<uint8_t> vec_rgb(outRgbIm, outRgbIm + width*height*3);
            out_msg.width        = yuv_image.width;
            out_msg.height       = yuv_image.height;
            out_msg.encoding     = "rgb8";
            out_msg.step         = yuv_image.width*3;
            out_msg.data         = vec_rgb;
            out_msg.header.stamp = yuv_image.header.stamp;
            out_pub.publish(out_msg);
        }

        /*
         * Convert YUV420 to planar RGB
         */
        void convertYUV420RGB(uint8_t * rgbImage,
                              uint8_t * yuvImage,
                              int32_t   width,
                              int32_t   height)
        {
            int32_t i, j;
            int32_t y, cb, cr;
            int32_t r, g,  b;
            uint8_t * src_ptr_y, *src_ptr_uv;
            uint8_t * dst_ptr_r, *dst_ptr_g, *dst_ptr_b;

            src_ptr_y  = yuvImage;
            src_ptr_uv = yuvImage + width*height;

            dst_ptr_r  = rgbImage;
            dst_ptr_g  = rgbImage + 1;
            dst_ptr_b  = rgbImage + 2;
 
            for (j = 0; j < height; j++)
            {
                for (i = 0; i < width; i++)
                {
                    y  = src_ptr_y[j * width + i];
                    cb = src_ptr_uv[(j >> 1)*width + (i>>1)*2];
                    cr = src_ptr_uv[(j >> 1)*width + (i>>1)*2 + 1];

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

        /*
         * Convert YUV422 (UYVY) to planar RGB
         */
        void convertYUV422RGB(uint8_t * rgbImage,
                              uint8_t * yuvImage,
                              int32_t   width,
                              int32_t   height)
        {
            int32_t i, j, n;
            int32_t y, cb, cr;
            int32_t r, g,  b;
            int32_t srcStride;
            uint8_t * src_ptr;
            uint8_t * dst_ptr_r, *dst_ptr_g, *dst_ptr_b;

            src_ptr   = yuvImage;

            dst_ptr_r = rgbImage;
            dst_ptr_g = rgbImage + 1;
            dst_ptr_b = rgbImage + 2;

            srcStride = width * 2;

            for (j = 0; j < height; j++)
            {
                for (i = 0; i < width; i+=2)
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
                    }
                }
            }
        }


    private:
        std::string                     input_yuv_topic_;
        std::string                     output_rgb_topic_;
        std::string                     yuv_format_;

        ros::NodeHandle                 nh;
        ros::Subscriber                 in_sub;
        image_transport::ImageTransport it;

        image_transport::Publisher      out_pub;
        sensor_msgs::Image              out_msg;

        int32_t                         width;
        int32_t                         height;
        uint8_t                       * outRgbIm;
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
        ros_app_yuv2rgb::Yuv2Rgb colorConv;

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }

}


