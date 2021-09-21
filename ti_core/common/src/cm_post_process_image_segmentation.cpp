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
#include <cm_post_process_image_segmentation.h>

namespace ti_core_common
{

CmPostprocessImageSemanticSeg::CmPostprocessImageSemanticSeg(const PostprocessImageConfig   &config):
    CmPostprocessImage(config)
{
}

void *CmPostprocessImageSemanticSeg::operator()(void             *inData,
                                                void             *outData,
                                                VecDlTensorPtr   &results)
{
    auto       *tensor = results[0];
    uint8_t    *dPtr = reinterpret_cast<uint8_t*>(outData);

    /* inData not used. */
    (void)inData;

    if (tensor->type == DlInferType_Int8)
    {
        int8_t  *sPtr = reinterpret_cast<int8_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_UInt8)
    {
        uint8_t  *sPtr = reinterpret_cast<uint8_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_Int16)
    {
        int16_t  *sPtr = reinterpret_cast<int16_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_UInt16)
    {
        uint16_t  *sPtr = reinterpret_cast<uint16_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_Int32)
    {
        int32_t  *sPtr = reinterpret_cast<int32_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_UInt32)
    {
        uint32_t  *sPtr = reinterpret_cast<uint32_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_Int64)
    {
        int64_t  *sPtr = reinterpret_cast<int64_t*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else if (tensor->type == DlInferType_Float32)
    {
        float  *sPtr = reinterpret_cast<float*>(tensor->data);
        CM_copyTensorData(sPtr, dPtr, tensor->numElem);
    }
    else
    {
        LOG_ERROR("Invalid data type.\n");
    }

    return outData;
}

CmPostprocessImageSemanticSeg::~CmPostprocessImageSemanticSeg()
{
}

} // namespace ti_core_common

