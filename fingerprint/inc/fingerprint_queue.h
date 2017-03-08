/* Copyright (c) 2012, The Linux Foundataion. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __FINGERPRINT_QUEUE_H__
#define __FINGERPRINT_QUEUE_H__

#include "fingerprint_list.h"

//#include <hardware/fingerprint.h>

typedef bool (*match_fn_data)(void *data, int match_data);
typedef void (*release_data_fn)(void* data);
typedef bool (*match_fn)(void *data);

class fingerprintQueue {
public:
    fingerprintQueue();
    fingerprintQueue(release_data_fn data_rel_fn);
    virtual ~fingerprintQueue();
    bool enqueue(void *data);
    bool enqueueWithPriority(void *data);
    void flush();
    void flushNodes(match_fn match);
    void flushNodes(match_fn_data match, int spec_data);
    void* dequeue(bool bFromHead = true);
    bool isEmpty();
    int getCurrentSize() {return m_size;}
private:
    typedef struct {
        struct fingerprint_list list;
        void* data;
    } fingerprint_q_node;

    fingerprint_q_node m_head; // dummy head
    int m_size;
    pthread_mutex_t m_lock;
    release_data_fn m_dataFn;
};

#endif /* __FINGERPRINT_QUEUE_H__ */

