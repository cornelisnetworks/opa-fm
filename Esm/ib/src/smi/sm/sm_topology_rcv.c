/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									               
// FILE NAME								     
//    sm_topology_rcv.c		
//									   
// DESCRIPTION								     
//    This thread will process asynchronous SM set responses. Used for	    
//    ucast and mcast tables primarily.
//									    
// DATA STRUCTURES							  
//    None								     
//									     
// FUNCTIONS								     
//    sm_topology_rcv   			main entry point		     
//									     
// DEPENDENCIES								     
//    ib_mad.h								     
//    ib_status.h							     
//    ib_const.h							     
//									     
//									     
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_macros.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "cs_context.h"
#include "cs_queue.h"

//********** sm asynchronous send receive context **********
generic_cntxt_t     sm_async_send_rcv_cntxt;

// SM asynchronous send response queue
cs_Queue_ptr sm_async_rcv_resp_queue;

// static variables
static  uint32_t    topology_rcv_exit=0;


static void
createFilter(uint16_t aid, char* filterName, char* filterErrName) {

	Status_t	status=VSTATUS_OK;
	Filter_t	filter;

	//
	//	Set the filter for catching set responses on fd_atopology.
	//
	SM_Filter_Init(&filter);
	filter.value.bversion = STL_BASE_VERSION;
	filter.value.cversion = STL_SM_CLASS_VERSION;
	filter.value.mclass = MAD_CV_SUBN_LR & 0x7f;	// LR or DR
	//filter.value.mclass = MAD_CV_SUBN_LR;
	filter.value.method = MAD_CM_GET_RESP;
	filter.value.aid = aid;

	filter.mask.bversion  = MAI_FMASK_ALL;
	filter.mask.cversion  = MAI_FMASK_ALL;
	//filter.mask.mclass  = MAI_FMASK_ALL;
	filter.mask.mclass  = 0x7f;
	filter.mask.method  = MAI_FMASK_ALL;
	filter.mask.aid  = MAI_FMASK_ALL;
	MAI_SET_FILTER_NAME (&filter, filterName);

	status = mai_filter_create(fd_atopology->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("topology_rcv: can't create topology async receive filter for set responses rc:", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

	//
	//	Set the filter for catching set errors on fd_atopology.
	//
	SM_Filter_Init(&filter);
	filter.type = MAI_TYPE_ERROR;
	filter.value.bversion = STL_BASE_VERSION;
	filter.value.cversion = STL_SM_CLASS_VERSION;
	filter.value.mclass = MAD_CV_SUBN_LR & 0x7f;	// LR or DR
	//filter.value.method = MAD_CM_GET_RESP;
	filter.value.aid = aid;

	filter.mask.bversion  = MAI_FMASK_ALL;
	filter.mask.cversion  = MAI_FMASK_ALL;
	filter.mask.mclass  = 0x7f;
	//filter.mask.method  = MAI_FMASK_ALL;
	filter.mask.aid  = MAI_FMASK_ALL;
	MAI_SET_FILTER_NAME (&filter, filterErrName);

	status = mai_filter_create(fd_atopology->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("topology_rcv: can't create topology async receive filter for set errors rc:", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}
}

void
topology_rcv(uint32_t argc, uint8_t ** argv) {
	Status_t	status=VSTATUS_OK;
	Filter_t	filter;
	Mai_t		mad;
    uint64_t    timeout=0;
    uint64_t	lastTimeAged=0, now=0;

    topology_rcv_exit = 0;
	(void)vs_thread_name(&sm_threads[SM_THREAD_TOP_RCV].name);

    //
    // initialize the SM async send receive response queue
    // +1 for depth to be safe
	// round to to 32 to keep MinQueueSize happy and for some headroom
    if ((sm_async_rcv_resp_queue = cs_queue_CreateQueue( &sm_pool, MAX(32, sm_config.max_parallel_reqs)+1 )) == NULL) {
		IB_LOG_ERROR0("sm_async: failed to initialize the SM async receive response queue, terminating");
		(void)vs_thread_exit(&sm_threads[SM_THREAD_TOP_RCV].handle);
    }

    //
    // initialize SM send receive context pool
    //
    memset((void *)&sm_async_send_rcv_cntxt, 0, sizeof(sm_async_send_rcv_cntxt));
    // all LR packets - use hash
    sm_async_send_rcv_cntxt.hashTableDepth = CNTXT_HASH_TABLE_DEPTH;
    sm_async_send_rcv_cntxt.poolSize = MAX(32, sm_config.max_parallel_reqs);
    sm_async_send_rcv_cntxt.maxRetries = sm_config.max_retries;
    sm_async_send_rcv_cntxt.ibHandle = fd_atopology->fdMai;
    sm_async_send_rcv_cntxt.errorOnSendFail = 0;
#ifdef IB_STACK_OPENIB
	// for openib we let umad do the timeouts.  Hence we add 1 second to
	// the timeout as a safety net just in case umad loses our response.
    sm_async_send_rcv_cntxt.timeoutAdder = VTIMER_1S;
#endif

    sm_async_send_rcv_cntxt.resp_queue = sm_async_rcv_resp_queue;   // queue to post responses
	sm_async_send_rcv_cntxt.totalTimeout = (sm_config.rcv_wait_msec * sm_config.max_retries * 1000);     // (*1000) to convert from milliseconds to microseconds

	/* PR 110945 - Stepped and randomized retries */
	/* If MinRcvWaitInterval is available, then set the default timeout of the cntxt to the smallest
	 * retry interval.
	 */
	if (sm_config.min_rcv_wait_msec) {
		timeout = sm_config.min_rcv_wait_msec * 1000;     // convert from milliseconds to microseconds
		sm_async_send_rcv_cntxt.MinRespTimeout = timeout;
	}
	else {
		timeout = sm_config.rcv_wait_msec * 1000;     // convert from milliseconds to microseconds
		sm_async_send_rcv_cntxt.MinRespTimeout = 0;
	}

    status = cs_cntxt_instance_init (&sm_pool, &sm_async_send_rcv_cntxt, timeout);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("topology_rcv: failed to initialize the SM send receive context instance, rc:", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_TOP_RCV].handle);
	}

	createFilter(STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE, "top_rcv_lft", "top_rcv_lft_err");
	createFilter(STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE, "top_rcv_mft", "top_rcv_mft_err");
	createFilter(STL_MCLASS_ATTRIB_ID_PART_TABLE, "top_rcv_pkey", "top_rcv_pkey_err");
	createFilter(STL_MCLASS_ATTRIB_ID_PORT_INFO, "top_rcv_pi", "top_rcv_pi_err");
	createFilter(STL_MCLASS_ATTRIB_ID_PORT_STATE_INFO, "top_rcv_psi", "top_rcv_psi_err");

    //
    //	Set the filter for catching LID routed get GUIDINFO responses on fd_atopology.
    //
	SM_Filter_Init(&filter);
	filter.value.bversion = MAD_BVERSION;
	filter.value.cversion = MAD_CVERSION;
	filter.value.mclass = MAD_CV_SUBN_LR;
	filter.value.method = MAD_CM_GET_RESP;
	filter.value.aid = MAD_SMA_GUIDINFO;

	filter.mask.bversion  = 0xff;
	filter.mask.cversion  = 0xff;
	filter.mask.mclass  = MAI_FMASK_ALL;
	filter.mask.method  = 0xff;
	filter.mask.aid  = 0xffff;
	MAI_SET_FILTER_NAME (&filter, "top_rcv_guid");

	status = mai_filter_create(fd_atopology->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("topology_rcv: can't create topology async receive filter for GuidInfo set responses rc:", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

    //
    //	Set the filter for catching LID routed get GUIDINFO errors on fd_atopology.
    //
	SM_Filter_Init(&filter);
	filter.type = MAI_TYPE_ERROR;
	filter.value.bversion = MAD_BVERSION;
	filter.value.cversion = MAD_CVERSION;
	filter.value.mclass = MAD_CV_SUBN_LR;
	//filter.value.method = MAD_CM_GET_RESP;
	filter.value.aid = MAD_SMA_GUIDINFO;

	filter.mask.bversion  = 0xff;
	filter.mask.cversion  = 0xff;
	filter.mask.mclass  = MAI_FMASK_ALL;
	//filter.mask.method  = 0xff;
	filter.mask.aid  = 0xffff;
	MAI_SET_FILTER_NAME (&filter, "top_rcv_guid_er");

	status = mai_filter_create(fd_atopology->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("topology_rcv: can't create topology async receive filter for GuidInfo set errors rc:", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

    // tell topology thread we are ready
    (void)cs_vsema(&topology_rcv_sema);

    (void)vs_time_get(&lastTimeAged);
    // process sm responses
	while (topology_rcv_exit == 0) {
        // 
        // check for LFT RESP on file descriptor (QP0)
        //
		/* If using stepped retry logic, use the smallest timeout of a context otherwise use 50ms as timeout.
		 * timeout variable has been initialized above to sm_config.min_rcv_wait_msec*1000 and will get updated based
		 * on return value from cs_cntxt_age() below.
		 */
		if (sm_config.min_rcv_wait_msec) {
		    status = mai_recv(fd_atopology->fdMai, &mad, timeout);
		}
		else
	        status = mai_recv(fd_atopology->fdMai, &mad, 50000ull);

        if (status == VSTATUS_OK) {
            switch (mad.type) {
            case MAI_TYPE_EXTERNAL:
                switch (mad.base.method) {
                case MAD_CM_GET_RESP:
					INCREMENT_COUNTER(smCounterRxGetResp);
					INCREMENT_MAD_STATUS_COUNTERS(&mad);

                    if (sm_state == SM_STATE_MASTER) {
                        // find and release the context
                        cs_cntxt_find_release(&mad, &sm_async_send_rcv_cntxt);
                    }
                    break;
                default:
                    IB_LOG_ERROR("topology_rcv: bad method received on sm receive file descriptor (QP0), method:", mad.base.method);
                    (void)sm_dump_mai("topology_rcv", &mad);
                    break;
                }
                break;
			case MAI_TYPE_ERROR:
				// only happens for openib.  This return indicated the
				// lower level stack has returned our original sent MAD
				// to indicate a failure sending it or a lack of a response
                switch (mad.base.method) {
                case MAD_CM_GET:
                case MAD_CM_SET:
                    if (sm_state == SM_STATE_MASTER) {
                        // find and release the context
                        cs_cntxt_find_release_error(&mad, &sm_async_send_rcv_cntxt);
					}
                    break;
                default:
                    IB_LOG_ERROR("topology_rcv: bad method received on sm receive file descriptor (QP0), method:", mad.base.method);
                       (void)sm_dump_mai("sm_async", &mad);
                    break;
                }
            default:
                break;
            }
        } else if (status != VSTATUS_TIMEOUT) {
            IB_LOG_WARNRC("topology_rcv: error on mai_recv for SM async receive fd (QP0), rc:", status);
        }

        // age context entries if necessary
        (void)vs_time_get(&now);
        if ((now - lastTimeAged) >= timeout) {
			/* cs_cntxt_age returns the smallest timeout of all contexts */
            timeout = cs_cntxt_age(&sm_async_send_rcv_cntxt);
			/* if there are no context entries (timeout will be 0) or timeout
			 * value is too small, set timeout to sm_config.min_rcv_wait_msec/4
			 * this avoids giving too small a timeout value to mai_recv()
			 */
			if (timeout == 0 || timeout < (sm_config.min_rcv_wait_msec*1000/4))	
				timeout = sm_config.min_rcv_wait_msec *1000;
			// since this loop also checks for SM shutdown, make sure we don't
			// use too long a timeout either
			if (timeout > VTIMER_1S/10)
				timeout = VTIMER_1S/10;
            lastTimeAged = now;
        }

		sm_dispatch_update(&sm_asyncDispatch);
	} // end while loop

    //	Delete the filter.
	if (mai_filter_delete(fd_atopology->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE) != VSTATUS_OK) {
		IB_LOG_ERROR0("topology_rcv: can't delete topology async receive filter");
	}
	// release the SM async send rcv context pool
    status = cs_cntxt_instance_free (&sm_pool, &sm_async_send_rcv_cntxt);
    // free the SM SM async send rcv queue
    cs_queue_DisposeQueue( &sm_pool, sm_async_rcv_resp_queue );

	//IB_LOG_INFINI_INFO0("topology_rcv thread: Exiting OK");
} // end topology_rcv


void
topology_rcv_kill(void){
	topology_rcv_exit = 1;
}

