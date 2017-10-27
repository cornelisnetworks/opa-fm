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
//    cs_context.h
//
// DESCRIPTION
//    Definitions for a generic context pool for reliable single packet 
//  comunications
//
//  There are three models for using this:
//  1. Simple state based operation with multiple concurrent requests and
//     a sequence initiated via callbacks.  This model is used by the PM.
//     A set of operations are started by a main thread.
//     Then callbacks process the responses and issue the next packet(s).
//     When all packets are done, the callback wakes up the main thread.
//  2. A "polling based" operation with multiple concurrent requests and
//     a sequence initiated by polling a resp_queue.  This model is used by
//     the SM for GuidInfo queries (sm_send_request_async_and_queue_rsp).
//     In this model a callback is not allowed.  The main thread issues one
//     or more packets.  As the responses complete (or timeout) the cntxt_entry
//     is put on the resp_queue.  The main thread polls or waits on the
//     resp_queue and processes the responses and decides when ti issue the
//     next packet.
//  3. A dispatcher operation with a work queue and a sequence initiated by
//     callbacks.  This is very similar to mode #1.  This model is used by the
//     SM for LFT and MFT sets.  The main thread puts outbound packets on a
//     dispatcher queue (sm_send_request_async_dispatch*).  Then callbacks
//     process the responses.  A dispatcher is also called periodically to
//     take items off the queue and issue new packets.  When the dispatcher
//     queue is empty an event is signalled which wakes the main thread.
//
//  In all the above models the cs_context can handle retries and timeouts.
//  As part of this design, care is taken to ensure that callers can depend
//  on eventually getting a callback.  As such there are no allocations
//  needed within the callback.  Even in model #1, care is taken by the
//  callbacks to ensure there is never an attempt to have more than
//  poolSize requests outstanding, hence the context can always provide an
//  entry to use for issuing the packet.
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    None
//
// DEPENDENCIES
//    ib_types.h
//    ib_mad.h
//    ib_status.h
//
//
//===========================================================================//

#ifndef	_CS_CONTEXT_H_
#define	_CS_CONTEXT_H_

#include "vs_g.h"
#include "mai_g.h"
#include "cs_queue.h"

//
//  SM Notice context definitions
//

#define CNTXT_HASH_TABLE_DEPTH	    79

struct _cntxt_entry;

// response mad is supplied to callback only if Status is VSTATUS_OK
typedef void (*cntxt_callback_t)(struct _cntxt_entry *, Status_t, void *, Mai_t *mad);

typedef struct _cntxt_entry {
	uint64_t	    tstamp;
	uint64_t        tid;		// Tid for hash table search
	uint16_t        lid;		// Lid for hash table search
    uint16_t        index;      // entry index into list
    uint8_t         hashed;
    uint8_t         alloced;
	uint16_t        retries;    // retry count
    uint64_t        RespTimeout; // current response timeout value (13.6.3.1)
	uint64_t		cumulative_timeout; //sum of timeouts across retries
	uint64_t		totalTimeout; // total initial timeout value
    uint8_t         senderWantsResponse;    // 1 = sender wants the response on the resp_queue
    uint8_t			sendFailed;	// most recent attempt to send failed
    uint16_t        releasing;	// have begun to release
    Mai_t           mad;		// mad to send
								// also for entries on resp_queue it holds
								// the received mad (or original sent mad if
								// MAI_TYPE_ERROR)
	cntxt_callback_t callback;  // caller callback called when context is freed
	void *          callback_data; // caller-specific data passed to callback
	struct _cntxt_entry *next;	// Link List next pointer
	struct _cntxt_entry *prev;	// Link List prev pointer
} cntxt_entry_t;

typedef struct _generic_cntxt_ {
    int             hashTableDepth;		// can set to 0 if non-lid routed packets
										// otherwise set to CNTXT_HASH_TABLE_DEPTH
    int             poolSize;
    int             maxRetries;         // max number of times to time to try sending mad
    IBhandle_t	    ibHandle;           // IBHandle file descriptor used for sending mads
    uint64_t        defaultTimeout;
	uint64_t		totalTimeout;		//if using stepped retries, this is the total Max timeout (in microseconds) across all retries
	uint64_t		MinRespTimeout;		//if using stepped retries, smallest response timeout (in microseconds) with which to start
	int				errorOnSendFail;    // 0 = if fail to send a retry, leave
										// on hash for attempt at next retry
										// interval
										// 1 = if fail to send a retry, fail
										// the entry without doing more retries
	uint64_t		timeoutAdder;		// amount to add to timeouts in
										// cs_cntxt_age.  If OFED style timeout
										// handling is being done in the kernel
										// this will cause non-send failure
										// entries to be checked with a higher
										// just in case OFED losed the packet
										// otherwise OFED is expected to provide
										// response or MAI_TYPE_ERROR packet
										// much sooner
    int	            numAlloc;
 	int	            numFree;
    Lock_t          lock;
    cs_Queue_ptr    resp_queue;         // queue to post responses
										// cntxt_entry* will be posted
										// to this queue.  when removed
										// from queue they should be
										// cs_cntxt_retire or cs_cntxt_reuse
    int             numWaiters;         // num waiters for a free context
    Sema_t		    freeContextWaitSema;
 	cntxt_entry_t   *free_list;
    cntxt_entry_t 	*hash[CNTXT_HASH_TABLE_DEPTH];
    cntxt_entry_t 	*pool;      // array of context entries 'poolSize' deep
    Pool_t          *globalPool;    // pool to allocate data from if needed for queued messages
} generic_cntxt_t;

// external function definitions
Pool_t   *cs_cntxt_get_global_pool (generic_cntxt_t *cntx);
Status_t cs_cntxt_instance_init (Pool_t *pool, generic_cntxt_t *cntx, uint64_t timeout);
Status_t cs_cntxt_instance_free (Pool_t *pool, generic_cntxt_t *cntx);

void cs_cntxt_lock( generic_cntxt_t *cntx);
void cs_cntxt_unlock( generic_cntxt_t *cntx);

uint64_t cs_cntxt_age (generic_cntxt_t *cntx);

// Get routines will allocate a context entry, initialize its mad
// and place it on the hash list.
// To avoid copying a mad, can specify mad=NULL then
// build mad directly in entry->mad
// Wait should be FALSE on all calls, wait=TRUE is only for internal use
cntxt_entry_t *cs_cntxt_get_nolock ( Mai_t* mad, generic_cntxt_t *cntx, boolean wait );
cntxt_entry_t *cs_cntxt_get ( Mai_t* mad, generic_cntxt_t *cntx, boolean wait );
cntxt_entry_t *cs_cntxt_get_wait ( Mai_t* mad, generic_cntxt_t *cntx );

// find context entry matching received mad, invoke the callback and
// release the entry
void cs_cntxt_find_release (Mai_t *mad, generic_cntxt_t *cntx);

// find context entry matching failed mad, initiate retries or invoke the
// callback and release the entry as appropriate
void cs_cntxt_find_release_error (Mai_t *mad, generic_cntxt_t *cntx);

// This is only valid for use within a cntxt callback, it will reuse the
// entry and prevent the cntxt_release function (which invoked the callback)
// from freeing entry.  mad can be NULL just like in cs_cntxt_get* in which
// case caller must build output mad directly in entry->mad
void cs_cntxt_reuse_nolock( cntxt_entry_t *entry, generic_cntxt_t *cntx, Mai_t *mad );

// This will free the entry.
// If used within a callback, after the retire, caller can use
// cs_cntxt_get_nolock with wait=0 to allocate another entry.
void cs_cntxt_retire_nolock( cntxt_entry_t *entry, generic_cntxt_t *cntx  );
void cs_cntxt_retire( cntxt_entry_t *entry, generic_cntxt_t *cntx  );

void cs_cntxt_set_callback(cntxt_entry_t *cntxt, cntxt_callback_t cb, void *data);

// within a callback, it is valid to call cs_cntxt_send_mad_nolock to
// resend the same mad without change.  This can be useful when dealing
// with temporary errors (BUSY).  Note that when invoked from a callback, the
// entry will be re-hashed so its ok to change some aspects of the mad.
Status_t cs_cntxt_send_mad_nolock (cntxt_entry_t *entry, generic_cntxt_t *cntx);
Status_t cs_cntxt_send_mad (cntxt_entry_t *entry, generic_cntxt_t *cntx);

#endif	// _CS_CONTEXT_H
