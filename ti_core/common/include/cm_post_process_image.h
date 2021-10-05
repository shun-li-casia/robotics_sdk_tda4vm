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

#ifndef _CM_POST_PROCESS_IMAGE_H_
#define _CM_POST_PROCESS_IMAGE_H_

/* Module headers. */
#include <common/include/post_process_image_config.h>

/**
 * \defgroup group_ticore_post_proc TI Robotics SDK Image Post-processing
 *
 * \brief Class providing interface for generic post-processing logic.
 *
 * \ingroup group_ticore_base
 */

namespace ti_core_common
{
    using namespace ti::edgeai::common;

    /** Post-processing for CNN output image.
     *
     * \ingroup group_ticore_post_proc
     */
    class CmPostprocessImage
    {
        public:
            /** Constructor.
             *
             * @param config Configuration information not present in YAML
             */
            CmPostprocessImage(const PostprocessImageConfig  &config);

            /** Function operator
             *
             * This is the heart of the class. The application uses this
             * interface to execute the functionality provided by this class.
             */
            virtual void *operator()(void              *inData,
                                     void              *outData,
                                     VecDlTensorPtr    &results) = 0;

            /** Factory method for making a specifc post-process object based on the
             * configuration passed.
             *
             * @param config   Configuration information not present in YAML
             * @returns A valid post-process object if success. A nullptr otherwise.
             */
            static CmPostprocessImage* makePostprocessImageObj(const PostprocessImageConfig   &config);

            /** Return the task type string. */
            const std::string &getTaskType();

            /** Destructor. */
            ~CmPostprocessImage();

        protected:
            /** Configuration information. */
            const PostprocessImageConfig    m_config{};

            /** Title. */
            std::string                     m_title;

        private:
            /**
             * Assignment operator.
             *
             * Assignment is not required and allowed and hence prevent
             * the compiler from generating a default assignment operator.
             */
            CmPostprocessImage &
                operator=(const CmPostprocessImage& rhs) = delete;
    };

} // namespace ti_core_common

#endif /* _CM_POST_PROCESS_IMAGE_H_ */

