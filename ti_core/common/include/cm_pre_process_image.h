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

#ifndef _CM_PRE_PROCESS_IMAGE_H_
#define _CM_PRE_PROCESS_IMAGE_H_

/* Module headers. */
#include <common/include/pre_process_image_config.h>

/**
 * \defgroup group_ticore_pre_proc Image Pre-processing
 *
 * \brief Class providing interface for generic pre-processing logic.
 *
 * \ingroup group_ticore_common
 */

namespace ti_core_common
{
    using namespace ti::edgeai::common;

    /** Base class for images based post-processing. This class forms as a base
     * class for different concrete post-processing a;lgorithms. This does not
     * provide polymorphic operations since the language does not allow virtual
     * functions that are abstract and templated.
     *
     * The design is that this class holds common data across different objects
     * and provides helpe functions for parsing and storing configuration data.
     * Any configuration specific data needed beyoond this basic capability will
     * be handled by the sub-classes as needed.
     *
     * \ingroup group_ticore_pre_proc
     */
    class CmPreprocessImage
    {
        public:
            /** Constructor.
             *
             * @param config Configuration information not present in YAML
             */
            CmPreprocessImage(const PreprocessImageConfig &config);

            /** Function operator
             *
             * This is the heart of the class. The application uses this
             * interface to execute the functionality provided by this class.
             */
            virtual int32_t operator()(const void **inData, VecDlTensorPtr &outData);

            /** Destructor. */
            virtual ~CmPreprocessImage();

            /** Factory method for making a specifc pre-process object based on the
             * configuration passed.
             *
             * @param config   Configuration information not present in YAML
             * @returns A valid pre-process object if success. A nullptr otherwise.
             */
            static CmPreprocessImage* makePreprocessImageObj(const PreprocessImageConfig  &config);

        private:
            /**
             * Assignment operator.
             *
             * Assignment is not required and allowed and hence prevent
             * the compiler from generating a default assignment operator.
             */
            CmPreprocessImage & operator=(const CmPreprocessImage& rhs) = delete;

        private:
            /** Configuration information. */
            const PreprocessImageConfig m_config;
    };

} // namespace ti_core_common

#endif /* _CM_PRE_PROCESS_IMAGE_H_ */
