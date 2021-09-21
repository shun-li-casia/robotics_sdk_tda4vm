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
#include <cm_post_process_image_object_detect.h>

/**
 * \defgroup group_ticore_obj_detect Object Detection post-processing
 *
 * \brief Class implementing the image based object detection post-processing
 *        logic.
 *
 * \ingroup group_ticore_post_proc
 */

namespace ti_core_common
{

CmPostprocessImageObjDetect::CmPostprocessImageObjDetect(const PostprocessImageConfig   &config):
    CmPostprocessImage(config)
{
    if (m_config.normDetect)
    {
        m_scaleX = static_cast<float>(m_config.outDataWidth);
        m_scaleY = static_cast<float>(m_config.outDataHeight);
    }
    else
    {
        m_scaleX = static_cast<float>(m_config.outDataWidth)/m_config.inDataWidth;
        m_scaleY = static_cast<float>(m_config.outDataHeight)/m_config.inDataHeight;
    }
    LOG_INFO("Created\n");
}

void *CmPostprocessImageObjDetect::operator()(void           *inData,
                                              void           *outData,
                                              VecDlTensorPtr &results)
{
    /* The results has three vectors. We assume that the type
     * of all these is the same.
     */
    std::vector<int64_t>    lastDims;
    VecDlTensorPtr          resultRo;
    float                  *out = reinterpret_cast<float *>(outData);

    /* Extract the last dimension from each of the output
     * tensors.
     * last dimension will give the number of values present
     * in given tensor
     * Ex: if shape of a tensor is
     *  [1][1][100][4] -> there are 4 values in the given tensor and 100 entries
     *  [100]          -> 1 value in given tensor and 100 entries, should not
     *                    consider last dim when number of dim is 1
     * Need to ignore all dimensions with value 1 since it does not actually add
     * a dimension (this is similar to squeeze operation in numpy)
     */
    for (auto i = 0; i < results.size(); i++)
    {
        //==> BUG: resultIndices = {0,1,2} only, while results.size() can be 4 (TFL case)
        auto   *result = results[m_config.resultIndices[i]];
        auto   &shape = result->shape;
        auto    nDims = result->dim;

        resultRo.push_back(result);

        for (auto s: shape)
        {
           if (s == 1)
           {
               nDims--;
           }
        }

        if (nDims == 1)
        {
            lastDims.push_back(1);
        }
        else
        {
            lastDims.push_back(result->shape[result->dim - 1]);
        }
    }

    auto getVal = [&lastDims, &resultRo] (int32_t iter, int32_t pos)
    {
        int64_t cumuDims = 0;

        for (auto i=0; i < lastDims.size(); i++)
        {
            cumuDims += lastDims[i];
            auto offset = iter * lastDims[i] + pos - cumuDims + lastDims[i];

            if (pos < cumuDims)
            {
                if (resultRo[i]->type == DlInferType_Int8)
                {
                    return (float)reinterpret_cast<int8_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_UInt8)
                {
                    return (float)reinterpret_cast<uint8_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_Int16)
                {
                    return (float)reinterpret_cast<int16_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_UInt16)
                {
                    return (float)reinterpret_cast<uint16_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_Int32)
                {
                    return (float)reinterpret_cast<int32_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_UInt32)
                {
                    return (float)reinterpret_cast<uint32_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_Int64)
                {
                    return (float)reinterpret_cast<int64_t*>(resultRo[i]->data)[offset];
                }
                else if (resultRo[i]->type == DlInferType_Float32)
                {
                    return (float)reinterpret_cast<float*>(resultRo[i]->data)[offset];
                }
            }
        }

        return (float)0;
    };

    // pack the total number of detected objects
    int32_t   numEntries = resultRo[0]->numElem/lastDims[0];
    int32_t   num_boxes = 0;
    float    *count = out++;

    // LOG_INFO("resultRo[0]->numElem = %d lastDims[0] = %d "
    //          "Number of Entries = %d\n",
    //          resultRo[0]->numElem, lastDims[0], numEntries);

    // pack each of the detected objects
    for (auto i=0; i < numEntries; i++)
    {
        float   score = getVal(i, m_config.formatter[5]);
        int32_t label = getVal(i, m_config.formatter[4]);
        int32_t box[4];

        //==> TODO: assumption is that objects are sorted based on score in descending order
        if (score < m_config.vizThreshold)
        {
            break;
        }

        box[0] = getVal(i, m_config.formatter[0]) * m_scaleX;
        box[1] = getVal(i, m_config.formatter[1]) * m_scaleY;
        box[2] = getVal(i, m_config.formatter[2]) * m_scaleX;
        box[3] = getVal(i, m_config.formatter[3]) * m_scaleY;

        int32_t adj_class_id = m_config.labelOffsetMap.at(label);

        *out++ = static_cast<float>(box[0]);
        *out++ = static_cast<float>(box[1]);
        *out++ = static_cast<float>(box[2]);
        *out++ = static_cast<float>(box[3]);
        *out++ = static_cast<float>(adj_class_id);
        *out++ = score;

        num_boxes++;
    }

    *count = static_cast<float>(num_boxes);

    return outData;
}

CmPostprocessImageObjDetect::~CmPostprocessImageObjDetect()
{
}

} // namespace ti_core_common

