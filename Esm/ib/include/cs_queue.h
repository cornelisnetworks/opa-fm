/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015, Intel Corporation

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//===========================================================================//
//
// FILE NAME
//    cs_queue.h
//
// DESCRIPTION
//    Definitions for a generic queue
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    None
//
// DEPENDENCIES
//    None
//
//
//===========================================================================//

#ifndef	_CS_QUEUE_H_
#define	_CS_QUEUE_H_

#include "ib_types.h"
#include "vs_g.h"

    // pointer to a queue element
    typedef void *cs_QueueElement_ptr;

    // queue record structure definition
    typedef struct _cs_QueueRecord_
    {
        int    Capacity;
        int    Size;
        int    Front;
        int    Rear;
        Lock_t lock;
        void   *qData;
    } cs_QueueRecord_t;

    // pointer to a queue record
    typedef cs_QueueRecord_t *cs_Queue_ptr;

    // function definitions
    static __inline int cs_queue_Capacity( cs_Queue_ptr Q ) { return Q->Capacity; };
    int cs_queue_IsEmpty( cs_Queue_ptr Q );
    int cs_queue_IsFull( cs_Queue_ptr Q );
    cs_Queue_ptr cs_queue_CreateQueue( Pool_t *pool, uint32_t MaxElements );
    void cs_queue_DisposeQueue( Pool_t *pool, cs_Queue_ptr Q );
    void cs_queue_MakeEmpty( cs_Queue_ptr Q );
    Status_t cs_queue_Enqueue( cs_QueueElement_ptr X, cs_Queue_ptr Q );
    cs_QueueElement_ptr cs_queue_Front( cs_Queue_ptr Q );
    void cs_queue_Dequeue( cs_Queue_ptr Q );
    cs_QueueElement_ptr cs_queue_FrontAndDequeue( cs_Queue_ptr Q );

#endif  /* _CS_QUEUE_H_ */
