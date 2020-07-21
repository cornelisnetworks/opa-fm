/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

//===========================================================================//
//
// FILE NAME
//    cs_context.c
//
// DESCRIPTION
//    This module provides a generic implementation of a context pool to be 
//    used for reliable trasmission of single packet mads only.
//
// DATA STRUCTURES
//    NONE
//
// FUNCTIONS
//    NONE
//
// DEPENDENCIES
//    ib_types.h
//    ib_mad.h
//    ib_status.h
//
//
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "vs_g.h"
#include "cs_g.h"
#include "cs_log.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_context.h"
#include "mai_g.h"
#include "ib_generalServices.h"

#include "sm_counters.h"
#include "pm_counters.h"

extern uint32_t smDebugPerf;

//
// delete a context entry
//
static void cntxt_delete_entry ( cntxt_entry_t **listCntx, cntxt_entry_t *entry ) {
    if (*listCntx == entry) {
        *listCntx = (*listCntx)->next;
    } else {
        if (entry->prev) entry->prev->next = entry->next;
        if (entry->next) entry->next->prev = entry->prev;
    }
    entry->next = NULL;
    entry->prev = NULL;
    //IB_LOG_INFINI_INFO("deleted context from list, index=", entry->index);
}


//
// insert a context entry into head of list
//
static void cntxt_insert_head ( cntxt_entry_t **listCntx, cntxt_entry_t *entry ) {
    if ( *listCntx ) (*listCntx)->prev = entry;
    entry->next = *listCntx;
    (*listCntx) = entry;
    entry->prev = NULL;
}

void cs_cntxt_lock( generic_cntxt_t *cntx ) {
    vs_lock(&cntx->lock);
}

void cs_cntxt_unlock( generic_cntxt_t *cntx ) {
    vs_unlock(&cntx->lock);
}

//
// Complete initialization of a context entry and put it on hash list.
// Very simple hashing implemented. This function is modular and can be 
// changed to use any algorithm, if hashing turns out to be bad.
//
static void cntxt_reserve( cntxt_entry_t *a_cntxt, generic_cntxt_t *cntx ) {
    int bucket ;

    DEBUG_ASSERT(a_cntxt->alloced);
    DEBUG_ASSERT(! a_cntxt->hashed);
    DEBUG_ASSERT(! a_cntxt->releasing);

    a_cntxt->lid = a_cntxt->mad.addrInfo.dlid;     // destination lid message is for
    a_cntxt->tid = a_cntxt->mad.base.tid;
    a_cntxt->RespTimeout = cntx->defaultTimeout;

    // This context needs to be inserted into the hash table
    if (cntx->hashTableDepth > 1) 
        bucket = a_cntxt->lid % cntx->hashTableDepth;
    else
        bucket = 0;
    a_cntxt->hashed = 1 ;
    vs_time_get( &a_cntxt->tstamp );
    cntxt_insert_head( &cntx->hash[ bucket ], a_cntxt );
}

// remove context entry from hash list
static void cntxt_unhash( cntxt_entry_t *entry, generic_cntxt_t *cntx)
{
    int bucket ;

    DEBUG_ASSERT(entry->alloced);
    if( entry->hashed ) {
        if (cntx->hashTableDepth > 1) 
            bucket = entry->lid % cntx->hashTableDepth;
        else
            bucket = 0;
        entry->hashed = 0 ;
        cntxt_delete_entry( &cntx->hash[ bucket ], entry );
    }
    entry->prev = NULL;
    entry->next = NULL;
}

//
// clear context entry and put back on pool free list
//
void cs_cntxt_retire_nolock( cntxt_entry_t *entry, generic_cntxt_t *cntx  )
{   uint16_t    index=0;

    //IB_LOG_INFINI_INFOX("retiring context at address=", (LogVal_t)entry);
    cntxt_unhash( entry, cntx);

    index = entry->index;       // save entry index
    // Reset all fields
    memset ((void *)entry, 0, sizeof(cntxt_entry_t));
    entry->index = index;       // put index back
    // put back on free list
    //entry->alloced = 0;    // already zeroed by memset
    //entry->releasing = 0;    // already zeroed by memset
    cntxt_insert_head( &cntx->free_list, entry );
    ++cntx->numFree;
    --cntx->numAlloc;

    //IB_LOG_INFINI_INFO("context back on free list, index=", entry->index);
    // wake up the waiter if one
    if (cntx->numWaiters) {
        //IB_LOG_INFINI_INFO("Waking up context queue user, current num waiters=", cntx->numWaiters);
        --cntx->numWaiters;    
        (void)cs_vsema(&cntx->freeContextWaitSema);
    }
}

void cs_cntxt_retire( cntxt_entry_t *entry, generic_cntxt_t *cntx  )
{
    Status_t    status;

    IB_ENTER(__func__, entry, cntx, 0, 0 );

    if ((status = vs_lock(&cntx->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock context rc:", status);
    } else {
        cs_cntxt_retire_nolock(entry, cntx);
        if (vs_unlock(&cntx->lock)) {
            IB_LOG_ERROR0("Failed to unlock context");
        }
    }

    IB_EXIT0( "cs_cntxt_retire" );
}

// This is only valid for use within a cntxt callback, it will reuse the
// context and prevent the cntxt_release function (which invoked the callback)
// from freeing entry
void cs_entxt_reuse_nolock( cntxt_entry_t *entry, generic_cntxt_t *cntx, Mai_t *mad ) {
    uint16_t    index=entry->index;       // save entry index

    DEBUG_ASSERT(entry->alloced);
    DEBUG_ASSERT(! entry->hashed);
    // Reset all fields
    memset ((void *)entry, 0, sizeof(cntxt_entry_t));
    entry->index = index;       // put index back
    entry->alloced = 1;         // restore alloced
    //entry->releasing = 0;     // already zeroed by memset above
    if (mad) {
        // save the output mad
        memcpy((void *)&entry->mad, mad, sizeof(Mai_t));
        cntxt_reserve( entry, cntx );    // we could wait until send_mad
    }
}

//
// find context entry matching input mad
//
static cntxt_entry_t *cntxt_find( Mai_t* mad, generic_cntxt_t *cntx ) {
    int 		    bucket;
    cntxt_entry_t*  a_cntxt = NULL;
    cntxt_entry_t*	req_cntxt = NULL;

    // Search the hash table for the context
    if (cntx->hashTableDepth > 1) 
        bucket = mad->addrInfo.slid % cntx->hashTableDepth;
    else
        bucket = 0;

    for (a_cntxt = cntx->hash[ bucket ]; a_cntxt ; a_cntxt = a_cntxt->next) {
        if( a_cntxt->lid == mad->addrInfo.slid  &&
            MAI_MASK_TID(a_cntxt->tid) == MAI_MASK_TID(mad->base.tid) ) {
            req_cntxt = a_cntxt;
            break ;
        }
    }
#if 0
    // Table is sorted with respect to timeout
    if( req_cntxt ) {
        // Touch current context
    	vs_time_get( &req_cntxt->tstamp );
        cntxt_delete_entry( &cntx->hash[ bucket ], req_cntxt );
        cntxt_insert_head( &cntx->hash[ bucket ], req_cntxt );
    }
#endif
    return req_cntxt;
} // end cntxt_find


//
// release a context entry
// This takes it off the hash list, invokes the callback and puts a_cntxt
// back on the free list.  As needed waiters are woken.
//
static Status_t cntxt_release( cntxt_entry_t *a_cntxt, generic_cntxt_t *cntx, Status_t s, Mai_t *mad ) {

    //IB_LOG_INFINI_INFOLX("releasing context index=", a_cntxt->index);
    cntxt_unhash( a_cntxt, cntx);
    a_cntxt->releasing = 1;

    if (a_cntxt->senderWantsResponse && cntx->resp_queue) {
		Status_t status;

		// presently the combination of a resp_queue and a callback are not
		// allowed, this is mainly bacause if the callback retired or reused
		// the a_cntxt, the cntxt_entry on the resp_queue would be stale
    	ASSERT(a_cntxt->callback == NULL);
        // post the response on the response queue
        if (mad) {
            memcpy((void *)&a_cntxt->mad, (void *)mad, sizeof(Mai_t));
        } else {
            a_cntxt->mad.type = MAI_TYPE_ERROR;
        }
		// provided resp_queue is sized the same as dispatcher, this should
		// not fail
        if ((status = cs_queue_Enqueue((cs_QueueElement_ptr)a_cntxt, cntx->resp_queue)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("unable to queue mad response for user, rc:", status);
			DEBUG_ASSERT(0);
        	cs_cntxt_retire_nolock( a_cntxt, cntx );
        }
    } else {
    	if (a_cntxt->callback != NULL)
        	a_cntxt->callback(a_cntxt, s, a_cntxt->callback_data, mad);

    	// if callback retires, reuses and/or re-gets the context, releasing
		// will be reset, in which case we leave the context unchanged.
    	// note that since we have lock, if callback retires context it can't be
    	// reclaimed and race with this test.
    	if( a_cntxt->releasing) {
        	cs_cntxt_retire_nolock( a_cntxt, cntx );
        	//IB_LOG_INFINI_INFOLX("successfully released the context, num free now=", cntx->numFree);
    	}
	}
    return VSTATUS_OK ;
}

//
// output the mad associated with the context
// assumes caller already had lock
//
Status_t cs_cntxt_send_mad_nolock (cntxt_entry_t *entry, generic_cntxt_t *cntx) {
    Status_t    status;
	uint32_t	datalen;

    if (! entry->hashed)
        cntxt_reserve( entry, cntx );	// clears sendFailed
    else
        entry->sendFailed = 0;

    //if (entry->retries == 0) {
        //IB_LOG_INFINI_INFOLX("sending notice mad, TID=", entry->tid);
    //} else {
        //IB_LOG_INFINI_INFOLX("resending sending notice mad, TID=", entry->tid);
    //}

    if (  (entry->mad.base.mclass == MAD_CV_SUBN_LR)
       || (entry->mad.base.mclass == MAD_CV_SUBN_DR) )
    {
        if (entry->retries) {
            INCREMENT_COUNTER(smCounterPacketRetransmits);
        }
        INCREMENT_COUNTER(smCounterSmPacketTransmits);
    } else if (entry->mad.base.mclass == MAD_CV_PERF)
    {
        if (entry->retries) {
            INCREMENT_PM_COUNTER(pmCounterPacketRetransmits);
        }
        INCREMENT_PM_COUNTER(pmCounterPmPacketTransmits);
    }

	//if (cntx->timeoutAdder) printf("mad_send tid=0x%lx timeout=%lu\n", entry->mad.base.tid, entry->RespTimeout);
    entry->retries++;
#if 0	// simulate failed sends to aid debug
if (entry->mad.base.mclass == 0x4) {
static unsigned count = 0;
//if (count > 200) {
if (count > 200 && count % 5 < 2) {
	count++;
    entry->sendFailed = 1;	// will avoid timeoutAddr
	return VSTATUS_BAD;
	//return VSTATUS_OK;	// pretend to send, let timeout cover us
} else {
count++;
}
}
#endif
	if (entry->mad.base.mclass == MAD_CV_PERF || entry->mad.base.mclass == MAD_CV_SUBN_ADM
			) {
		datalen = MIN((uint32_t)entry->mad.datasize, STL_GS_DATASIZE);
		if ((status = mai_send_stl_timeout(cntx->ibHandle, &entry->mad, &datalen, entry->RespTimeout)) != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				   "status %d sending %s[%s] MAD length %u in context entry[%d] to LID[0x%x], TID 0x%.16"CS64"X",
				   status, cs_getMethodText(entry->mad.base.method),
				   cs_getAidName(entry->mad.base.mclass, entry->mad.base.aid), (uint32)entry->mad.datasize,
				   entry->index, entry->mad.addrInfo.dlid, entry->tid);
			entry->sendFailed = 1;
		}
		return status;
	}
    if ((status = mai_send_timeout(cntx->ibHandle, &entry->mad, entry->RespTimeout)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, 
               "status %d sending %s[%s] MAD in context entry[%d] to LID[0x%x], TID 0x%.16"CS64"X",
               status, cs_getMethodText(entry->mad.base.method), 
               cs_getAidName(entry->mad.base.mclass, entry->mad.base.aid),
               entry->index, entry->mad.addrInfo.dlid, entry->tid);
        entry->sendFailed = 1;
    }
    return status;
}

//
//  get global context pool
//
Pool_t *cs_cntxt_get_global_pool (generic_cntxt_t *cntx) {
    return cntx->globalPool;
}

//
// init a context pool for use
//
Status_t cs_cntxt_instance_init (Pool_t *pool, generic_cntxt_t *cntx, uint64_t timeout) {
    Status_t        status;
    uint32_t        poollen;
    int             i;
    cntxt_entry_t   *entry;

    IB_ENTER(__func__, cntx, 0, 0, 0 );

	if (cntx->resp_queue) {
		if (cs_queue_Capacity(cntx->resp_queue) < cntx->poolSize) {
        	IB_LOG_ERROR_FMT(__func__, "resp_queue too small: %d need %d", cs_queue_Capacity(cntx->resp_queue), cntx->poolSize);
			return VSTATUS_ILLPARM;
		}
	}
    poollen = sizeof(cntxt_entry_t) * cntx->poolSize;
    //IB_LOG_INFINI_INFO("generic_cntxt_t size ", sizeof(generic_cntxt_t));
    //IB_LOG_INFINI_INFO("cntxt_entry_t size ", sizeof(cntxt_entry_t));
    IB_LOG_VERBOSE("allocating context pool size ", poollen);
    status = vs_pool_alloc( pool, poollen, (void *)&cntx->pool );
    if (status != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't allocate context pool, rc:", status);
        return status;
    }
    //IB_LOG_INFINI_INFOX("cntx->pool = ", (uint32_t)cntx->pool);
    memset( cntx->pool, 0, poollen);
    cntx->globalPool = pool;        // set global memory pool to use for queued messages
    cntx->defaultTimeout = timeout;
    cntx->numAlloc = 0;
    cntx->numFree = cntx->poolSize;
    cntx->free_list = NULL;
    entry = (cntxt_entry_t *)cntx->pool;
    for( i = 0, entry = (cntxt_entry_t *)cntx->pool; i < cntx->poolSize ; i++, entry++ ) {
        //IB_LOG_INFINI_INFOX("entry ptr=", (uint32_t)entry);
        entry->index = i+1;       // for debug/tracking purposes
        cntxt_insert_head(&cntx->free_list, entry);
    }
    // initialize queue lock
    status = vs_lock_init(&cntx->lock, VLOCK_FREE, VLOCK_THREAD);
    if (status != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't initialize context pool lock rc:", status);
        vs_pool_free( pool, (void *)cntx->pool );
    }
    // create the free context wait semaphore
    if ((status = cs_sema_create(&cntx->freeContextWaitSema, 0)) != VSTATUS_OK) {
        IB_FATAL_ERROR_NODUMP("cs_cntx_instance_init: can't initialize context semaphore");
    }

    IB_EXIT(__func__, status );
    return status;
}


//
// output the mad associated with the context
//
Status_t cs_cntxt_send_mad (cntxt_entry_t *entry, generic_cntxt_t *cntx) {
    Status_t    status;

    IB_ENTER(__func__, entry, cntx, 0, 0 );

    if ((status = vs_lock(&cntx->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock context rc:", status);
    } else {
        if ((status = cs_cntxt_send_mad_nolock(entry, cntx)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("can't send MAD rc:", status);
        }
        if (vs_unlock(&cntx->lock)) {
            IB_LOG_ERROR0("Failed to unlock context");
        }
    }

    IB_EXIT(__func__, 0 );
    return status;
}

// TBD - could the timeout calculation part of this be shared with SM
// non-dispatched timeouts too?
//
// This routine processes a timed out context entry.  It either performs
// the next retry or reports the entry as timedout and releases the entry.
// If a retry is attempted but fails, VSTATUS_BAD is returned and the
// caller can decide if a release or delay for later attempt is appropriate
// If entry exhausted retries, VSTATUS_TIMEOUT is returned
// If retry is successfully initiated, VSTATUS_OK is returned.
// cntxt_release will need to be invoked by caller as appropriate
static
Status_t cs_cntxt_timeout_entry(cntxt_entry_t *tout_cntxt, generic_cntxt_t *cntx,
							uint64_t timenow) {
    uint64_t	    timeout=0;
	Status_t status;

	//if (cntx->timeoutAdder) printf("cs_cntxt_timeout_entry tid=0x%lx\n", tout_cntxt->tid);

   	// Touch the entry
   	tout_cntxt->tstamp = timenow;

   	/* If using stepped retries, update the entry's RespTimeout to the
	 * new time out value
	 */
	if (cntx->MinRespTimeout) {
		/* PR 110945 - Stepped and randomized retries */
		tout_cntxt->cumulative_timeout += tout_cntxt->RespTimeout;
		/* For each attempt, use an increasing multiple of MinRespTimeout as the timeout*/
		timeout = (tout_cntxt->retries+1)*cntx->MinRespTimeout;
		/* Randomize timeout value within +/- 20% of the timeout*/
		timeout = GET_RANDOM((timeout + (timeout/5)), (timeout - (timeout/5)));
		/* Make sure this timeout value doesn't cause cumulative_timeout
		 * to overshoot the total timeout
		 * If it will cause an overshoot, then just use whatever is left
		 * from the total timeout
		 */
		if ((tout_cntxt->cumulative_timeout < tout_cntxt->totalTimeout) &&
			 ((tout_cntxt->cumulative_timeout + timeout) > tout_cntxt->totalTimeout)) {
			timeout = tout_cntxt->totalTimeout - tout_cntxt->cumulative_timeout;
			/* if the timeout left is less than min, set it to the min. This will cause cumulative_timeout to overshoot
			 * total_timeout by a bit, but that should be OK */
			if (timeout < cntx->MinRespTimeout)
				timeout = cntx->MinRespTimeout;
		}

		tout_cntxt->RespTimeout = timeout;
	}

	/* retry sending or release if max retries (no stepped retry logic)
	 * or max timeout (stepped retry logic) has exhausted
	 */
	if ((cntx->MinRespTimeout == 0) && (tout_cntxt->retries < cntx->maxRetries)) {
		/*no stepped retry logic, so retry till we reach maxRetries */
		//if (cntx->timeoutAdder) printf("retry send, tid=0x%lx timeout=%ld\n", tout_cntxt->tid, tout_cntxt->RespTimeout/1000);
		if ((status = cs_cntxt_send_mad_nolock (tout_cntxt, cntx)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("can't resend notice reliably rc:", status);
			return VSTATUS_BAD;	// send failed
		}
	} else if ((cntx->MinRespTimeout != 0) && (tout_cntxt->cumulative_timeout < tout_cntxt->totalTimeout)) {
		/*stepped retry logic is being used, so retry till cumulative timeout
		 * reaches total timeout
		 */
		//if (cntx->timeoutAdder) printf("retry send, tid=0x%lx timeout=%ld\n", tout_cntxt->tid, tout_cntxt->RespTimeout/1000);
		if ((status = cs_cntxt_send_mad_nolock (tout_cntxt, cntx)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("can't resend notice reliably rc:", status);
			return VSTATUS_BAD;	// send failed
		}
	} else {
		/* retries exhausted, report failure */
		if ((tout_cntxt->mad.base.mclass == MAD_CV_SUBN_LR)
			|| (tout_cntxt->mad.base.mclass == MAD_CV_SUBN_DR) ) {
			if (smDebugPerf) {
				// exhausted the retries, release the context
				IB_LOG_INFINI_INFO_FMT(__func__, "no response to %s of %s from lid 0x%x, TID 0x%.16"CS64"X",
					cs_getMethodText(tout_cntxt->mad.base.method), cs_getAidName(tout_cntxt->mad.base.mclass, tout_cntxt->mad.base.aid), 
					tout_cntxt->lid, tout_cntxt->tid);
			}
			INCREMENT_COUNTER(smCounterSmTxRetriesExhausted);
		} else if (tout_cntxt->mad.base.mclass == MAD_CV_PERF) {
			INCREMENT_PM_COUNTER(pmCounterPmTxRetriesExhausted);
		}
		//if (cntx->timeoutAdder) printf("retries exhausted tid=0x%lx\n", tout_cntxt->tid);
		return VSTATUS_TIMEOUT;
	}
	return VSTATUS_OK;
}

/*
 * Age context and do resends for those that timed out
 * 
 * Returns the smallest time left for the context that is going to expire next.
 */
uint64_t cs_cntxt_age(generic_cntxt_t *cntx)
{
    cntxt_entry_t*  a_cntxt = NULL ;
    cntxt_entry_t*	tout_cntxt ;
    int i;
    Status_t        status;
    uint64_t	    timenow=0, smallest_timeleft=0, time_left=0;

	IB_ENTER(__func__, cntx, 0, 0, 0 );

    if ((status = vs_lock(&cntx->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock context rc:", status);
    } else {
    	vs_time_get( &timenow );   
        for (i = 0; i < cntx->hashTableDepth; i++) {
            a_cntxt = (cntxt_entry_t *)cntx->hash[ i ];
            while( a_cntxt ) {
				// if a send failed, use normal timeouts.  If send did not
				// fail, we use timeoutAdder.  timeoutAdder will be 0 if this
				// is our primary aging mechanism.  If we are using OFED
				// timeouts, timeoutAdder will allow this mechanism to
				// be a safety net in case OFED fails to provide a return MAD
				uint64_t timeoutAdder= a_cntxt->sendFailed?0:cntx->timeoutAdder;
                // Iterate before the pointers are destroyed
                tout_cntxt = a_cntxt;
                a_cntxt = a_cntxt->next;
                //IB_LOG_INFINI_INFO("checking context index ", tout_cntxt->index);
                if ( timenow - tout_cntxt->tstamp >= tout_cntxt->RespTimeout + timeoutAdder) {
                    // Timeout this entry
					//if (cntx->timeoutAdder) printf("Entry aged out: tid=0x%lx send failed=%d delta=%lu adder=%lu\n", tout_cntxt->tid, tout_cntxt->sendFailed, timenow - tout_cntxt->tstamp, timeoutAdder);

#if 0
                    // Get timed out entry to head of hash
                    cntxt_delete_entry( &cntx->hash[ i ], tout_cntxt );
                    cntxt_insert_head( &cntx->hash[ i ], tout_cntxt );
#endif

					status = cs_cntxt_timeout_entry(tout_cntxt, cntx, timenow);
					if (status == VSTATUS_TIMEOUT
						|| (cntx->errorOnSendFail && status != VSTATUS_OK)) {
						/*if this context is being released, don't consider
						 * it for smallest_timeleft calculation
						 */
						cntxt_release( tout_cntxt, cntx, status, NULL );
						continue;
                    }
					// If errorOnSendFail = 0, for send errors (VSTATUS_BAD)
					// we leave entry in hash and will retry it next time
					// timeout expires.
					// Successful retry (VSTATUS_OK) also stays on hash
                }

				/* check remaining time for this context before it times out */
				time_left = tout_cntxt->RespTimeout - (timenow - tout_cntxt->tstamp);
				time_left += tout_cntxt->sendFailed?0:cntx->timeoutAdder;

				/* if it is the context with the smallest time left for its
				 * timeout, then update smallest_timeleft
				*/
				if (smallest_timeleft) {
					if ( time_left < smallest_timeleft)
						smallest_timeleft = time_left;
				} else {
					/* smallest_timeleft is 0, no previous time_left values */
					smallest_timeleft = time_left;
				}
            }
        }
        if (vs_unlock(&cntx->lock)) {
            IB_LOG_ERROR0("Failed to unlock context");
        }
    }
	IB_EXIT(__func__, 0 );
	return smallest_timeleft;
}

cntxt_entry_t*
cs_cntxt_get_nolock( Mai_t* mad, generic_cntxt_t *cntx, boolean wait )
{
	uint64_t	    now;
	cntxt_entry_t   *req_cntxt = NULL;

	IB_ENTER(__func__, mad, cntx, 0, 0 );

    vs_time_get( &now );
    // Allocate a new context
    req_cntxt = cntx->free_list;
    if( req_cntxt != NULL ) {
        // remove from free list
        cntxt_delete_entry( &cntx->free_list, req_cntxt );
        // increment number allocated
        ++cntx->numAlloc;
        --cntx->numFree;
        // reserve the context
        req_cntxt->alloced = 1;
        req_cntxt->totalTimeout = cntx->totalTimeout;
        if (mad) {
            // save the output mad
            memcpy((void *)&req_cntxt->mad, mad, sizeof(Mai_t));
            cntxt_reserve( req_cntxt, cntx );	// we could wait until send_mad
        }
        //IB_LOG_INFINI_INFO("context allocated, num free now ", cntx->numFree);
    } else if (wait) {
        ++cntx->numWaiters;
    } else {
        IB_LOG_INFINI_INFO0("no free context available at this time");
    }

    IB_EXIT(__func__, req_cntxt );
    return req_cntxt ;
}

cntxt_entry_t*
cs_cntxt_get( Mai_t* mad, generic_cntxt_t *cntx, boolean wait )
{
    cntxt_entry_t   *req_cntxt = NULL;
    Status_t        status;

    IB_ENTER(__func__, mad, cntx, 0, 0 );

    if ((status = vs_lock(&cntx->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock context rc:", status);
    } else {
        req_cntxt = cs_cntxt_get_nolock(mad, cntx, wait );
        if ((status = vs_unlock(&cntx->lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock context rc:", status);
        }
    }
    IB_EXIT(__func__, req_cntxt );
    return req_cntxt ;
}

// 
// wait for a free context
//
cntxt_entry_t*
cs_cntxt_get_wait( Mai_t* mad, generic_cntxt_t *cntx )
{
    cntxt_entry_t   *req_cntxt = NULL;

    IB_ENTER(__func__, mad, cntx, 0, 0 );

    while (req_cntxt == NULL) {
        req_cntxt = cs_cntxt_get(mad, cntx, TRUE);
        // wait on sema if required
        if (req_cntxt == NULL) {
            //IB_LOG_INFINI_INFO0("no free context available at this time, WAITING....");
            (void)cs_psema(&cntx->freeContextWaitSema);
        }
    }

    IB_EXIT(__func__, req_cntxt );
    return req_cntxt ;
}

//
//  got a response to a outbound MAD, release the appropriate context
//
void cs_cntxt_find_release (Mai_t *mad, generic_cntxt_t *cntx) {
    cntxt_entry_t *entry = NULL;
    Status_t status = 0;

    IB_ENTER(__func__, cntx, 0, 0, 0 );

    if ((status = vs_lock(&cntx->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock context rc:", status);
    } else {
        if ((entry = cntxt_find( mad, cntx)) != NULL) {
            //IB_LOG_INFINI_INFOLX("trying to release context with tid ", entry->tid);
            cntxt_release( entry, cntx, VSTATUS_OK, mad );
        } else {
            IB_LOG_INFO_FMT(__func__, "discarding unexpected response: Class: 0x%x Attr: 0x%x SLID: 0x%x TID: "FMT_U64, mad->base.mclass, mad->base.aid, mad->addrInfo.slid, mad->base.tid);
		}
        if (vs_unlock(&cntx->lock)) {
            IB_LOG_ERROR0("Failed to unlock context");
        }
    }

    IB_EXIT(__func__, 0 );
}


//
//  got indication that outbound MAD failed to send or never got a response.
//  Identify appropriate context and either fail it or start retries
void cs_cntxt_find_release_error (Mai_t *mad, generic_cntxt_t *cntx) {
    cntxt_entry_t *entry = NULL;
    Status_t status = 0;

    IB_ENTER(__func__, cntx, 0, 0, 0 );

	//if (cntx->timeoutAdder) printf("cs_cntxt_find_release_error tid=0x%lx\n", mad->base.tid);
    if ((status = vs_lock(&cntx->lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock context rc:", status);
    } else {
        if ((entry = cntxt_find( mad, cntx)) != NULL) {
			uint64_t timenow;
    		vs_time_get( &timenow );   
			status = cs_cntxt_timeout_entry(entry, cntx, timenow);

			if (status == VSTATUS_TIMEOUT
				|| (cntx->errorOnSendFail && status != VSTATUS_OK)) {
				cntxt_release( entry, cntx, status, NULL );
            }
        } else {
            IB_LOG_INFO_FMT(__func__, "discarding unexpected error: Class: 0x%x Attr: 0x%x SLID: 0x%x TID: "FMT_U64, mad->base.mclass, mad->base.aid, mad->addrInfo.slid, mad->base.tid);
		}
        if (vs_unlock(&cntx->lock)) {
            IB_LOG_ERROR0("Failed to unlock context");
        }
    }

    IB_EXIT(__func__, 0 );
}


//
// free context pool
//
Status_t
cs_cntxt_instance_free (Pool_t *pool, generic_cntxt_t *cntx) {
    Status_t status = 0;

    IB_ENTER(__func__, pool, cntx, 0, 0 );

    vs_lock_delete (&cntx->lock);
    status = vs_pool_free( pool, (void *)cntx->pool );
    if (status != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't free context pool, rc:", status);
    }
    (void)cs_sema_delete (&cntx->freeContextWaitSema);
    memset((void *)cntx, 0, sizeof(generic_cntxt_t));

    IB_EXIT(__func__, status );
    return status;
}

//
// simple setter for caller-specified callback
//
void cs_cntxt_set_callback(cntxt_entry_t *cntxt, cntxt_callback_t cb, void *data)
{
    if (cb == NULL) {
        cntxt->callback = cntxt->callback_data = NULL;
    } else {
        cntxt->callback = cb;
        cntxt->callback_data = data;
    }
}

