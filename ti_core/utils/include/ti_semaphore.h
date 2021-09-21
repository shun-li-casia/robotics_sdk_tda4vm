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
#if !defined(_TI_SEMAPHORE_H_)
#define _TI_SEMAPHORE_H_

#include <iostream>
#include <cassert>
#include <mutex>
#include <condition_variable>
namespace ti::utils {
    class Semaphore
    {
        private:
            std::mutex              m_mutex;
            std::condition_variable m_cond;
            int32_t                 m_cnt;

        private:
            // Prohibit copying
            Semaphore(const Semaphore &s) = delete;
            Semaphore& operator=(const Semaphore &s) = delete;

        public:
            Semaphore(int32_t   cnt = 0):
                m_cnt(cnt)
            {
            }

            void notify()
            {
                // Acquire Mutex
                std::unique_lock<std::mutex> lock(m_mutex);

                // Increment the count
                m_cnt++;

                // Post the condition variable
                m_cond.notify_one();
            }

            void wait()
            {
                // Acquire Mutex
                std::unique_lock<std::mutex> lock(m_mutex);

                // Wait if the count is 0
                while ( !m_cnt )
                {
                    m_cond.wait(lock);
                }

                // Decrement the count
                m_cnt--;
            }

            bool try_wait()
            {
                // Acquire Mutex
                std::unique_lock<std::mutex> lock(m_mutex);

                while ( !m_cnt )
                {
                    return ( false );
                }

                return ( true );
            }

    }; // class Semaphore

} // nameapce ti::utils

#endif // !defined(_TI_SEMAPHORE_H_)

