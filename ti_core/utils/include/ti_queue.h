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
#if !defined(_TI_QUEUE_H_)
#define _TI_QUEUE_H_

#include <queue>
#include <mutex>
#include <stdexcept>

#include <ti_semaphore.h>

/**
 * \defgroup group_ticore_utils_queue Utility functions for queue
 *
 * \brief Provides Queue class for multi-thread processing of graphs
 *
 * \ingroup  group_ticore_utils
 *
 */

namespace ti::utils {

    /**
     * \brief Dummy Mutex structure
     * \ingroup group_ticore_utils_queue
     */
    struct MutexDummy
    {
        void lock(){}
        void unlock(){};
    };

    /**
     * \brief Dummy Semaphore structure
     * \ingroup group_ticore_utils_queue
     */
    struct SemaphoreDummy
    {
        SemaphoreDummy(int32_t cnt = 0){}
        void notify(){}
        void wait(){}
        bool try_wait(){ return true; }
    };

    /**
     * \brief Queue class
     * \ingroup group_ticore_utils_queue
     */
    template <typename T,
              typename MutexT = MutexDummy,
              typename SemT = SemaphoreDummy>
    class Queue
    {
        public:
            /**
             * \brief Queue class constructor
             * \ingroup group_ticore_utils_queue
             */
            Queue()
            {
            }

            /**
             * \brief Get the oldest element from a queue
             * \ingroup group_ticore_utils_queue
             */
            T *peek()
            {
                std::unique_lock<MutexT>    lock(m_mutex);
                T                          *val{nullptr};

                /* Check if we have descriptors available. */
                if (!m_q.empty())
                {
                    val = m_q.front();
                }

                return val;
            }

            /**
             * \brief Pop the oldest element from a queue
             * \ingroup group_ticore_utils_queue
             */
            T *pop()
            {
                /* Check if we have data. */
                m_sem.wait();

                std::unique_lock<MutexT>    lock(m_mutex);
                T                          *val{nullptr};

                /* Check if we have descriptors available. */
                if (!m_q.empty())
                {
                    val = m_q.front();
                    m_q.pop();
                }

                return val;
            }

           /**
             * \brief Push a value into a queue
             * \ingroup group_ticore_utils_queue
             */
            int32_t push(T *val)
            {
                std::unique_lock<MutexT>    lock(m_mutex);

                m_q.push(val);

                m_sem.notify();

                return 0;
            }

           /**
             * \brief Flush a queue
             * \ingroup group_ticore_utils_queue
             */
            void flush()
            {
                std::unique_lock<MutexT>    lock(m_mutex);

                while( !m_q.empty() )
                {
                    m_q.pop();
                }
            }

            /**
             * \brief Get the number of elements in a queue
             * \ingroup group_ticore_utils_queue
             */
            int32_t size()
            {
                std::unique_lock<MutexT>    lock(m_mutex);
                return m_q.size();
            }

        private:
            /**
             * \brief A queue for holding free descriptors
             * \ingroup group_ticore_utils_queue
             */
            std::queue<T*>  m_q;

            /**
             * \brief Resource lock. Used get/put from/to freeQ
             * \ingroup group_ticore_utils_queue
             */
            MutexT          m_mutex;

            /**
             * \brief Synchronization control
             * \ingroup group_ticore_utils_queue
             */
            SemT            m_sem{};
    };

    /**
     * \brief Single thread safe queue with no mutex and semaphore control.
     * \ingroup group_ticore_utils_queue
     */
    template <typename T>
    using SingleThreadQ = Queue<T>;

    /**
     * \brief Multi-thread safe queue with mutex control.
     * \ingroup group_ticore_utils_queue
     */
    template <typename T, typename MutexT = std::mutex>
    using MultiThreadQ = Queue<T, MutexT>;

    /**
     * \brief Multi-thread safe queue with mutex control.
     * \ingroup group_ticore_utils_queue
     */
    template <typename T,
              typename MutexT = std::mutex,
              typename SemT = Semaphore>
    using MultiThreadTaskQ = Queue<T, MutexT, SemT>;

} // namespace ti::utils

#endif // _TI_QUEUE_H_

