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
    CmPostprocessImage::CmPostprocessImage(const PostprocessImageConfig  &config):
    m_config(config)
{
}

CmPostprocessImage* CmPostprocessImage::makePostprocessImageObj(const PostprocessImageConfig    &config)
{
    CmPostprocessImage   *cntxt = nullptr;

    if (config.taskType == "detection")
    {
        LOG_INFO("Creating detectionObj...\n");
        cntxt = new CmPostprocessImageObjDetect(config);
    }
    else if (config.taskType == "segmentation")
    {
        cntxt = new CmPostprocessImageSemanticSeg(config);
    }
    else
    {
        LOG_ERROR("Invalid post-processing task type.\n");
    }

    return cntxt;
}

const std::string &CmPostprocessImage::getTaskType()
{
    return m_config.taskType;
}

CmPostprocessImage::~CmPostprocessImage()
{
}

} // namespace ti_core_common

