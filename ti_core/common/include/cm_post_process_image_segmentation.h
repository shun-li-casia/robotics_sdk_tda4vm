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

#ifndef _CM_POST_PROCESS_IMAGE_SEMANTIC_SEGMENTATION_H_
#define _CM_POST_PROCESS_IMAGE_SEMANTIC_SEGMENTATION_H_

/* Module headers. */
#include <cm_common.h>
#include <cm_post_process_image.h>

/**
 * \defgroup group_ticore_sem_seg Semantic Segmentation post-processing
 *
 * \brief Class implementing the image based semantic segmentation post-processing
 *        logic.
 *
 * \ingroup group_ticore_post_proc
 */

namespace ti_core_common
{
    /** Post-processing for image based semantic segmentation
     *
     * \ingroup group_ticore_sem_seg
     */
    class CmPostprocessImageSemanticSeg : public CmPostprocessImage
    {
        public:
            /** Constructor.
             *
             * @param config Configuration information not present in YAML
             */
            CmPostprocessImageSemanticSeg(const PostprocessImageConfig    &config);

            /** Function operator
             *
             * This is the heart of the class. The application uses this
             * interface to execute the functionality provided by this class.
             *
             * @param inData  Input data frame
             * @param outData Output data frame with results overlaid
             * @param results Segmentation output results from the inference
             */
            void *operator()(void              *inData,
                             void              *outData,
                             VecDlTensorPtr    &results);

            /** Destructor. */
            ~CmPostprocessImageSemanticSeg();

        private:
            /**
             * Assignment operator.
             *
             * Assignment is not required and allowed and hence prevent
             * the compiler from generating a default assignment operator.
             */
            CmPostprocessImageSemanticSeg &
                operator=(const CmPostprocessImageSemanticSeg& rhs) = delete;
    };
} // namespace ti_core_common

#endif /* _CM_POST_PROCESS_IMAGE_SEMANTIC_SEGMENTATION_H_ */

