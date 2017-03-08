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

#include <utils/Errors.h>
#include <utils/Log.h>
#include <stdlib.h>
#include <string.h>
#include "fingerprint_queue.h"

#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "FingerprintHal"
#endif

/*===========================================================================
 * FUNCTION   : fingerprintQueue
 *
 * DESCRIPTION: default constructor of fingerprintQueue
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
fingerprintQueue::fingerprintQueue()
{
    pthread_mutex_init(&m_lock, NULL);
    fingerprint_list_init(&m_head.list);
    m_size = 0;
    m_dataFn = NULL;
	ALOGE("fingerprint queue init finish");
}

/*===========================================================================
 * FUNCTION   : fingerprintQueue
 *
 * DESCRIPTION: constructor of fingerprintQueue
 *
 * PARAMETERS :
 *   @data_rel_fn : function ptr to release node data internal resource
 *
 * RETURN     : None
 *==========================================================================*/
fingerprintQueue::fingerprintQueue(release_data_fn data_rel_fn)
{
    pthread_mutex_init(&m_lock, NULL);
    fingerprint_list_init(&m_head.list);
    m_size = 0;
    m_dataFn = data_rel_fn;
	ALOGE("fingerprint queue init finish with data_rel_fn");
}

/*===========================================================================
 * FUNCTION   : ~fingerprintQueue
 *
 * DESCRIPTION: deconstructor of fingerprintQueue
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
fingerprintQueue::~fingerprintQueue()
{
    flush();
    pthread_mutex_destroy(&m_lock);
	ALOGE("fingerprint queue deinit finish");
}

/*===========================================================================
 * FUNCTION   : isEmpty
 *
 * DESCRIPTION: return if the queue is empty or not
 *
 * PARAMETERS : None
 *
 * RETURN     : true -- queue is empty; false -- not empty
 *==========================================================================*/
bool fingerprintQueue::isEmpty()
{
    bool flag = true;
    pthread_mutex_lock(&m_lock);
    if (m_size > 0) {
        flag = false;
    }
    pthread_mutex_unlock(&m_lock);
    return flag;
}

/*===========================================================================
 * FUNCTION   : enqueue
 *
 * DESCRIPTION: enqueue data into the queue
 *
 * PARAMETERS :
 *   @data    : data to be enqueued
 *
 * RETURN     : true -- success; false -- failed
 *==========================================================================*/
bool fingerprintQueue::enqueue(void *data)
{
    fingerprint_q_node *node =
        (fingerprint_q_node *)malloc(sizeof(fingerprint_q_node));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_q_node", __func__);
        return false;
    }

    memset(node, 0, sizeof(fingerprint_q_node));
    node->data = data;

    pthread_mutex_lock(&m_lock);
    fingerprint_list_add_tail_node(&node->list, &m_head.list);
    m_size++;
    pthread_mutex_unlock(&m_lock);
    return true;
}

/*===========================================================================
 * FUNCTION   : enqueueWithPriority
 *
 * DESCRIPTION: enqueue data into queue with priority, will insert into the
 *              head of the queue
 *
 * PARAMETERS :
 *   @data    : data to be enqueued
 *
 * RETURN     : true -- success; false -- failed
 *==========================================================================*/
bool fingerprintQueue::enqueueWithPriority(void *data)
{
    fingerprint_q_node *node =
        (fingerprint_q_node *)malloc(sizeof(fingerprint_q_node));
    if (NULL == node) {
        ALOGE("%s: No memory for fingerprint_q_node", __func__);
        return false;
    }

    memset(node, 0, sizeof(fingerprint_q_node));
    node->data = data;

    pthread_mutex_lock(&m_lock);
    struct fingerprint_list *p_next = m_head.list.next;

    m_head.list.next = &node->list;
    p_next->prev = &node->list;
    node->list.next = p_next;
    node->list.prev = &m_head.list;

    m_size++;
    pthread_mutex_unlock(&m_lock);
    return true;
}

/*===========================================================================
 * FUNCTION   : dequeue
 *
 * DESCRIPTION: dequeue data from the queue
 *
 * PARAMETERS :
 *   @bFromHead : if true, dequeue from the head
 *                if false, dequeue from the tail
 *
 * RETURN     : data ptr. NULL if not any data in the queue.
 *==========================================================================*/
void* fingerprintQueue::dequeue(bool bFromHead)
{
    fingerprint_q_node* node = NULL;
    void* data = NULL;
    struct fingerprint_list *head = NULL;
    struct fingerprint_list *pos = NULL;

    pthread_mutex_lock(&m_lock);
	if(m_size <= 0) {
		pthread_mutex_unlock(&m_lock);
		return NULL;
	}

    head = &m_head.list;
    if (bFromHead) {
        pos = head->next;
    } else {
        pos = head->prev;
    }
    if (pos != head) {
        node = member_of(pos, fingerprint_q_node, list);
        fingerprint_list_del_node(&node->list);
        m_size--;
    }
    pthread_mutex_unlock(&m_lock);

    if (NULL != node) {
        data = node->data;
        free(node);
    }

    return data;
}

/*===========================================================================
 * FUNCTION   : flush
 *
 * DESCRIPTION: flush all nodes from the queue, queue will be empty after this
 *              operation.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
void fingerprintQueue::flush(){
    fingerprint_q_node* node = NULL;
    struct fingerprint_list *head = NULL;
    struct fingerprint_list *pos = NULL;

    pthread_mutex_lock(&m_lock);
    head = &m_head.list;
    pos = head->next;

    while(pos != head) {
        node = member_of(pos, fingerprint_q_node, list);
        pos = pos->next;
        fingerprint_list_del_node(&node->list);
        m_size--;

        if (NULL != node->data) {
            if (m_dataFn) {
                m_dataFn(node->data);
            }
            free(node->data);
        }
        free(node);

    }
    m_size = 0;
    pthread_mutex_unlock(&m_lock);
}

/*===========================================================================
 * FUNCTION   : flushNodes
 *
 * DESCRIPTION: flush only specific nodes, depending on
 *              the given matching function.
 *
 * PARAMETERS :
 *   @match   : matching function
 *
 * RETURN     : None
 *==========================================================================*/
void fingerprintQueue::flushNodes(match_fn match){
    fingerprint_q_node* node = NULL;
    struct fingerprint_list *head = NULL;
    struct fingerprint_list *pos = NULL;

    if ( NULL == match ) {
        return;
    }

    pthread_mutex_lock(&m_lock);
    head = &m_head.list;
    pos = head->next;

    while(pos != head) {
        node = member_of(pos, fingerprint_q_node, list);
        pos = pos->next;
        if ( match(node->data) ) {
            fingerprint_list_del_node(&node->list);
            m_size--;

            if (NULL != node->data) {
                if (m_dataFn) {
                    m_dataFn(node->data);
                }
                free(node->data);
            }
            free(node);
        }
    }
    pthread_mutex_unlock(&m_lock);
}

/*===========================================================================
 * FUNCTION   : flushNodes
 *
 * DESCRIPTION: flush only specific nodes, depending on
 *              the given matching function.
 *
 * PARAMETERS :
 *   @match   : matching function
 *
 * RETURN     : None
 *==========================================================================*/
void fingerprintQueue::flushNodes(match_fn_data match,int match_data){
    fingerprint_q_node* node = NULL;
    struct fingerprint_list *head = NULL;
    struct fingerprint_list *pos = NULL;

    if ( NULL == match ) {
        return;
    }

    pthread_mutex_lock(&m_lock);
    head = &m_head.list;
    pos = head->next;

    while(pos != head) {
		if(m_size <= 0) {
			ALOGE("fingerprint dequeue fail,break");
			break;
		}
        node = member_of(pos, fingerprint_q_node, list);
        pos = pos->next;
        if ( match(node->data, match_data) ) {
            fingerprint_list_del_node(&node->list);
            m_size--;

            if (NULL != node->data) {
                if (m_dataFn) {
                    m_dataFn(node->data);
                }
                free(node->data);
            }
            free(node);
        }
    }
    pthread_mutex_unlock(&m_lock);
}

