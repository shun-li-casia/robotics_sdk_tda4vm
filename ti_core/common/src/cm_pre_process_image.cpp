/*
 *  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Module headers. */
#include <string>
#include <cm_pre_process_image.h>

namespace ti_core_common
{

#define clip3(x, min, max) ((x) > (max)?(max):((x) < (min)?(min):(x)))
#define CM_normalize(x, mean, scale) (((x) - (mean))*(scale))

/**
 * \brief Function to convert a YUV420 image to planar RGB format for VISION CNN network
 *
 * \param [out] rgbImage RGB image data.
 *
 * \param [in] yuvImage YUV raw image data. yuvImage[0] will point to the Y
 *                      plane date and yuvImage[1] will point to the UV plane
 *                      data.
 *
 * \param [in] mean mean value of RGB channels 
 *
 * \param [in] scale standard deviation of RGB channels 
 *
 * \param [in] width Width of the image in pixels.
 *
 * \param [in] height Height of the image in pixels.
 *
 * \param [in] dataLayout tensor layout
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common
 */

template <typename InputT, typename OutputT>
int32_t CM_VISION_CNN_convertYUV2RGB(OutputT           *rgbImage,
                                     InputT            *yuvImage[2],
                                     std::vector<float> mean,
                                     std::vector<float> scale,
                                     int32_t            width,
                                     int32_t            height,
                                     const std::string  dataLayout)
{
    const InputT   *srcPtrY;
    const InputT   *srcPtrUV;
    float           meanR;
    float           meanG;
    float           meanB;
    float           scaleR;
    float           scaleG;
    float           scaleB;
    float           r;
    float           g;
    float           b;
    int32_t         cb;
    int32_t         cr;
    int32_t         i;
    int32_t         j;
    int32_t         y;
    int32_t         status = 0;

    if (mean.size() >= 3)
    {
        meanR  = mean[0];
        meanG  = mean[1];
        meanB  = mean[2];
    }
    else
    {
        meanR  = 0.0;
        meanG  = 0.0;
        meanB  = 0.0;
    }

    if (scale.size() >= 3)
    {
        scaleR = scale[0];
        scaleG = scale[1];
        scaleB = scale[2];
    }
    else
    {
        scaleR = 1.0;
        scaleG = 1.0;
        scaleB = 1.0;
    }

    srcPtrY  = yuvImage[0];
    srcPtrUV = yuvImage[1];

    if (dataLayout == "NCHW")
    {
        OutputT  *dstPtrR;
        OutputT  *dstPtrG;
        OutputT  *dstPtrB;

        dstPtrR  = rgbImage;
        dstPtrG  = dstPtrR + (width * height);
        dstPtrB  = dstPtrG + (width * height);

        for (j = 0; j < height; j++)
        {
            for (i = 0; i < width; i++)
            {
                y  = srcPtrY[j * width + i];
                cb = srcPtrUV[(j >> 1)*width + (i>>1)*2];
                cr = srcPtrUV[(j >> 1)*width + (i>>1)*2 + 1];

                y  = y  - 16;
                cb = cb - 128;
                cr = cr - 128;

                r = (float)clip3((298*y + 409*cr + 128) >> 8, 0, 255);
                g = (float)clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
                b = (float)clip3((298*y + 516*cb + 128) >> 8, 0, 255);

                //==> TODO: RGB or BGR?. use "reverse_channels" field
                *dstPtrR++ = static_cast<OutputT>(CM_normalize(r, meanR, scaleR));
                *dstPtrG++ = static_cast<OutputT>(CM_normalize(g, meanG, scaleG));
                *dstPtrB++ = static_cast<OutputT>(CM_normalize(b, meanB, scaleB));
            }
        }
    }
    else if (dataLayout == "NHWC")
    {
        OutputT  *dstPtr;
        dstPtr   = rgbImage;

        for (j = 0; j < height; j++)
        {
            for (i = 0; i < width; i++)
            {
                y  = srcPtrY[j * width + i];
                cb = srcPtrUV[(j >> 1)*width + (i>>1)*2];
                cr = srcPtrUV[(j >> 1)*width + (i>>1)*2 + 1];

                y  = y  - 16;
                cb = cb - 128;
                cr = cr - 128;

                r = (float)clip3((298*y + 409*cr + 128) >> 8, 0, 255);
                g = (float)clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
                b = (float)clip3((298*y + 516*cb + 128) >> 8, 0, 255);

                //==> TODO: RGB or BGR?. use "reverse_channels" field
                *dstPtr++ = static_cast<OutputT>(CM_normalize(r, meanR, scaleR));
                *dstPtr++ = static_cast<OutputT>(CM_normalize(g, meanG, scaleG));
                *dstPtr++ = static_cast<OutputT>(CM_normalize(b, meanB, scaleB));
            }
        }
    }
    else
    {
        status = -1;
    }

    return status;
}

CmPreprocessImage::CmPreprocessImage(const PreprocessImageConfig    &config):
    m_config(config)
{
}

int32_t CmPreprocessImage::operator()(const void **inData, VecDlTensorPtr &outData)
{
    void      **tmpPtr = const_cast<void**>(inData);
    uint8_t   **srcPtr = reinterpret_cast<uint8_t**>(tmpPtr);
    DlTensor   *tensor = outData[0];
    int32_t     status = 0;

    if (tensor->type == DlInferType_Int8)
    {
        int8_t  *dataBuff = reinterpret_cast<int8_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_UInt8)
    {
        uint8_t  *dataBuff = reinterpret_cast<uint8_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_Int16)
    {
        int16_t  *dataBuff = reinterpret_cast<int16_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_UInt16)
    {
        uint16_t  *dataBuff = reinterpret_cast<uint16_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_Int32)
    {
        int32_t  *dataBuff = reinterpret_cast<int32_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_UInt32)
    {
        uint32_t  *dataBuff = reinterpret_cast<uint32_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_Int64)
    {
        int64_t  *dataBuff = reinterpret_cast<int64_t*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else if (tensor->type == DlInferType_Float32)
    {
        float  *dataBuff = reinterpret_cast<float*>(tensor->data);
        status =
            CM_VISION_CNN_convertYUV2RGB(dataBuff,
                                     srcPtr,
                                     m_config.mean,
                                     m_config.scale,
                                     m_config.resizeWidth,
                                     m_config.resizeHeight,
                                     m_config.dataLayout);
    }
    else
    {
        LOG_ERROR("Invalid data type.\n");
        status = -1;
    }

    return status;
}

CmPreprocessImage* CmPreprocessImage::makePreprocessImageObj(const PreprocessImageConfig    &config)
{
    CmPreprocessImage   *cntxt;

    cntxt = new CmPreprocessImage(config);

    return cntxt;
}

CmPreprocessImage::~CmPreprocessImage()
{
}
} // namespace ti_core_common

