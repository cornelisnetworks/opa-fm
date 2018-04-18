/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2017, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include "vs_g.h"
#include "cs_g.h"
#include "cs_log.h"
#include "ib_status.h"
#include "cs_queue.h"

#define MinQueueSize ( 5 )

int cs_queue_IsEmpty( cs_Queue_ptr Q ) {
    return (Q->Size == 0);
}


int cs_queue_IsFull( cs_Queue_ptr Q ) {
    return (Q->Size == Q->Capacity);
}


cs_Queue_ptr cs_queue_CreateQueue( Pool_t *pool, uint32_t MaxElements ) {
    cs_Queue_ptr    que=NULL;
    Status_t        status;
    int             qsize;

    if( MaxElements < MinQueueSize ) {
        IB_LOG_ERROR( "Queue size is too small (minimum of 5 entries required):", MaxElements );
        return que;
    }

    status = vs_pool_alloc( pool, sizeof(cs_QueueRecord_t), (void *)&que );
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't allocate space for Queue, rc:", status);
		return NULL;
	}
    // allocate space for array of element pointers
    qsize = sizeof(cs_QueueElement_ptr) * MaxElements;
    status = vs_pool_alloc( pool, qsize, (void **)&que->qData );
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't allocate space for Queue data, rc:", status);
        status = vs_pool_free( pool, (void *)que );
        if (status != VSTATUS_OK) {
            IB_LOG_ERRORRC("can't free Queue space, rc:", status);
        }
		return NULL;
	}
    // initialize queue lock
	status = vs_lock_init(&que->lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("cs_queue_CreateQueue: can't initialize SM trap forwarding queue lock");
        vs_pool_free( pool, (void *)que );
        vs_pool_free( pool, (void *)que->qData );
        que = NULL;
	} else {
        que->Capacity = MaxElements;
        cs_queue_MakeEmpty( que );
        IB_LOG_VERBOSE_FMT(__func__,
               "Created Queue with entry count of %d and total Queue aize of %d",
               que->Capacity, qsize);
    }
    return que;
}

        
void cs_queue_MakeEmpty( cs_Queue_ptr Q )
{
    Q->Size = 0;
    Q->Front = 1;
    Q->Rear = 0;
}


void cs_queue_DisposeQueue( Pool_t *pool, cs_Queue_ptr Q )
{
    Status_t        status;

    if( Q != NULL )
    {
        vs_lock_delete (&Q->lock);
        if ((status = vs_pool_free( pool, (void *)Q->qData )) != VSTATUS_OK) {
            IB_LOG_ERRORRC("can't free Queue data, rc:", status);
        }
        if ((status = vs_pool_free( pool, (void *)Q )) != VSTATUS_OK) {
            IB_LOG_ERRORRC("can't free Queue, rc:", status);
        }
    }
    Q = NULL;
}


static int Succ( int Value, int Capacity )
{
    if( ++Value == Capacity )
        Value = 0;
    return Value;
}

// enqueue entry at rear of queue
Status_t cs_queue_Enqueue( cs_QueueElement_ptr X, cs_Queue_ptr Q )
{
    Status_t    status = VSTATUS_OK;
    unint       *ptr=Q->qData;

    if ((status = vs_lock(&Q->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock queue rc:", status);
    } else {
        if( cs_queue_IsFull( Q ) ) {
            IB_LOG_ERROR0("Queue is full ");
            // Cleanup (unlock) and exit
            if ((status = vs_unlock(&Q->lock)) != VSTATUS_OK) {
                IB_LOG_ERRORRC("Failed to unlock queue rc:", status);
            }
            return(VSTATUS_BAD);
        } else {
            Q->Size++;
            Q->Rear = Succ( Q->Rear, Q->Capacity );
            ptr[Q->Rear] = (unint)X;
            //IB_LOG_INFINI_INFO("inserted entry on Queue index ", Q->Rear);
            if (X == NULL) {
                IB_LOG_ERROR0("pointer to data on queue is NULL");
            } else {
                //IB_LOG_INFINI_INFOX("pointer to data on queue is ", (uint32_t)X);
                //IB_LOG_INFINI_INFOX("ptr to qdata[Q->Rear]=", (uint32_t)&ptr[Q->Rear]);
            }
        }
        if ((status = vs_unlock(&Q->lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock queue rc:", status);
        }
    }
    return status;
}

// get the entry at front of queue but don't dequeue it
cs_QueueElement_ptr cs_queue_Front( cs_Queue_ptr Q )
{
    Status_t    status = VSTATUS_OK;
    cs_QueueElement_ptr X=NULL;
    unint       *ptr=Q->qData;

    if ((status = vs_lock(&Q->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock queue rc:", status);
    } else {
        if( !cs_queue_IsEmpty( Q ) ) {
            X = (cs_QueueElement_ptr)ptr[ Q->Front ];
            if (X == NULL) {
                IB_LOG_ERROR0("pointer to data on queue is NULL");
            } else {
                //IB_LOG_INFINI_INFO("returning Queue entry #", Q->Front);
                //IB_LOG_INFINI_INFOX("returning entry qdata[Q->Front]=", (uint32_t)X);
            }
        }
        if ((status = vs_unlock(&Q->lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock queue rc:", status);
        }
    }
    
    return X;
}

// remove the entry at front of queue
void cs_queue_Dequeue( cs_Queue_ptr Q )
{
    Status_t    status = VSTATUS_OK;

    if ((status = vs_lock(&Q->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock queue rc:", status);
    } else {
        if( !cs_queue_IsEmpty( Q ) ) {
            Q->Size--;
            //IB_LOG_INFINI_INFO("removing Queue entry #", Q->Front);
            Q->Front = Succ( Q->Front, Q->Capacity );
            //IB_LOG_INFINI_INFO("free Queue entry count=", (Q->Capacity - Q->Size));
        }
        if ((status = vs_unlock(&Q->lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock queue rc:", status);
        }
    }
}

// dequeue and return the entry at front of quque
cs_QueueElement_ptr cs_queue_FrontAndDequeue( cs_Queue_ptr Q )
{
    cs_QueueElement_ptr X = NULL;
    Status_t    status = VSTATUS_OK;
    unint       *ptr=Q->qData;
//  int front=0;

    if ((status = vs_lock(&Q->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock queue rc:", status);
    } else {
        if( !cs_queue_IsEmpty( Q ) ) {
//          front = Q->Front;
            Q->Size--;
            X = (cs_QueueElement_ptr)ptr[ Q->Front ];
            Q->Front = Succ( Q->Front, Q->Capacity );
            //IB_LOG_INFINI_INFO("returning Queue entry #", front);
            if (X == NULL) IB_LOG_ERROR0("pointer to data on queue is NULL");
        }
        if ((status = vs_unlock(&Q->lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock queue rc:", status);
        }
    }
    return X;
}

