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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									               
// FILE NAME								     
//    pm_async_rcv.c		
//									   
// DESCRIPTION								     
//    This thread will process asynchronous PM responses.
//    (code here is based on sm_topology_rcv)
//									    
// DATA STRUCTURES							  
//    None								     
//									     
// FUNCTIONS								     
//    pm_async_rcv   			main entry point		     
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
#include "pm_l.h"
#include "cs_context.h"
#include "pm_topology.h"

extern SMXmlConfig_t sm_config;
extern int smValidateGsiMadPKey(Mai_t *maip, uint8_t mgmntAllowedRequired, uint8_t antiSpoof);

//********** pm asynchronous send receive context **********
generic_cntxt_t     *pm_async_send_rcv_cntxt;

// static variables
static  uint32_t    pm_async_rcv_exit=0;


static int
pm_gsi_check_packet_filter(Mai_t *maip)
{
    int rc = 0;

    // The following P_Key related security features shall be supported by all GSI administrators:
    //   - The GSI administrator shall be a full member of the management P_Key (0xffff),
    //     and shall use P_Key 0xffff when interacting with clients for responses, RMPP
    //     responses, Notices, etc.
    //   - All GSI administrator clients shall be full members of the management P_Key,
    //     and shall use P_Key 0xffff when interacting with the GSI Administrator (PA, etc)
    //     for requests, RMPP Acks, Notice Replies, etc. This restriction prevents non-management nodes from
    //     accessing administrative data for the fabric beyond the name services and information which the
    //     SA makes available.
    //   - The GSI administrator shall verify that all inbound requests, RMPP Acks,
    //     Notices replies use P_Key 0xffff and shall discard any packets with other
    //     P_keys without processing them.    
    if (!smValidateGsiMadPKey(maip, 0, sm_config.sma_spoofing_check)) {
        rc = 1;
        if (pm_config.debug_rmpp) {
            IB_LOG_WARN_FMT(__func__, 
                            "Dropping packet, invalid P_Key 0x%x, attribute 0x%x from LID [0x%x] with TID ["FMT_U64"] ", 
                            maip->addrInfo.pkey, maip->base.aid, maip->addrInfo.slid, maip->base.tid); 
        }
    }

    if (rc) {
        // if the MAD is not valid, then indicate to the MAI filter handling
        // code that no match has been found.  Also, change the MAI type to
        // indicate to the MAI filter handling code to just drop the packet.  
        maip->type = MAI_TYPE_DROP;
    }
    
    return (rc) ? 1 : 0;
}

void
pm_async_rcv(uint32_t argc, uint8_t ** argv) {
	Status_t	status=VSTATUS_OK;
	Filter_t	filter;
	Filter_t	filter2;
	Mai_t		mad;
    uint64_t    timeout=0;
    uint64_t	lastTimeAged=0, now=0;

	IB_LOG_INFO0("thread: Starting");
	g_pmAsyncRcvThreadRunning = TRUE;
    pm_async_rcv_exit = 0;

    //
    //	Set the filter for catching PMA responses on hpma
    //
	PM_Filter_Init(&filter);
	filter.value.bversion = STL_BASE_VERSION;
	filter.value.cversion = STL_PM_CLASS_VERSION;
	filter.value.mclass = MAD_CV_PERF;
	filter.value.method = MAD_CM_GET_RESP;
	//filter.value.aid = MAD_SMA_LFT;

	filter.mask.bversion  = MAI_FMASK_ALL;
	filter.mask.cversion  = MAI_FMASK_ALL;
	filter.mask.mclass  = MAI_FMASK_ALL;
	filter.mask.method  = MAI_FMASK_ALL;
	//filter.mask.aid  = MAI_FMASK_ALL;
    filter.mai_filter_check_packet = pm_gsi_check_packet_filter; 
	MAI_SET_FILTER_NAME (&filter, "pm_rcv");

	status = mai_filter_create(hpma, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create async receive filter for PMA responses rc:", status);
		goto done;
	}
    //
    //	Set the filter for catching failed PMA requests on hpma
    //
	PM_Filter_Init(&filter2);
	filter2.type = MAI_TYPE_ERROR;
	filter2.value.bversion = STL_BASE_VERSION;
	filter2.value.cversion = STL_PM_CLASS_VERSION;
	filter2.value.mclass = MAD_CV_PERF;
	//filter2.value.method = MAD_CM_GET_RESP;
	//filter2.value.aid = MAD_SMA_LFT;

	filter2.mask.bversion  = MAI_FMASK_ALL;
	filter2.mask.cversion  = MAI_FMASK_ALL;
	filter2.mask.mclass  = MAI_FMASK_ALL;
	//filter2.mask.method  = MAI_FMASK_ALL;
	//filter2.mask.aid  = MAI_FMASK_ALL;
    filter2.mai_filter_check_packet = pm_gsi_check_packet_filter; 
	MAI_SET_FILTER_NAME (&filter2, "pm_rcv_error");

	status = mai_filter_create(hpma, &filter2, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create async error filter for PMA responses rc:", status);
		goto freefilter;
	}

    // tell PM main thread we are ready
    (void)cs_vsema(&g_pmAsyncRcvSema);
	IB_LOG_INFO0("thread: Ready");

    (void)vs_time_get(&lastTimeAged);
    // process PMA responses
	while (pm_async_rcv_exit == 0) {
        // 
        // check for PMA RESP on file descriptor (QP1)
        //
		/* If using stepped retry logic, use the smallest timeout of a context otherwise use 50ms as timeout.
		 * timeout variable has been initialized above to pm_config.MinRcvWaitInterval*1000 and will get updated based
		 * on return value from cs_cntxt_age() below.
		 */

		if (pm_config.MinRcvWaitInterval) {
		    status = mai_recv(hpma, &mad, timeout);
		}
		else
	        status = mai_recv(hpma, &mad, 50000ull);
		//IB_LOG_DEBUG2RC("recv MAD rc:", status);

        if (status == VSTATUS_OK) {
			IB_LOG_DEBUG2_FMT(__func__, "recv MAD method: 0x%x Attr: 0x%x status: 0x%x tid: " FMT_U64, mad.base.method, mad.base.aid, mad.base.status, mad.base.tid);
			IB_LOG_DATA("imad.base:", &mad.base, sizeof(mad.base));
            switch (mad.type) {
            case MAI_TYPE_EXTERNAL:
                switch (mad.base.method) {
                case MAD_CM_GET_RESP:
					IB_LOG_DATA("imad.data:", &mad.data[0], sizeof(mad.data));
					INCREMENT_PM_COUNTER(pmCounterRxGetResp);
					INCREMENT_PM_MAD_STATUS_COUNTERS(&mad);

					// if TID or LID doesn't match any of the outstanding
					// context entries, the packet is discarded by
					// cs_cntxt_find_release
                    cs_cntxt_find_release(&mad, pm_async_send_rcv_cntxt);
                    break;
                default:
					// discard unexpected method
					IB_LOG_INFO_FMT( __func__,
		       			"Unexpected MAD received: method=0x%x SLID=0x%x TID=0x%016"CS64"x",
		       			mad.base.method, mad.addrInfo.slid, mad.base.tid);
                    //(void)sm_dump_mai("pm_async_rcv", &mad);
                    break;
                }
                break;
            case MAI_TYPE_ERROR:
				// only happens for openib.  This return indicates the
				// lower level stack has returned our orginal sent MAD to
				// indicate a failure sending it or a lack of a response
                switch (mad.base.method) {
                case MAD_CM_GET:
                case MAD_CM_SET:
					// if TID or LID doesn't match any of the outstanding
					// context entries, the packet is discarded by
					// cs_cntxt_find_release_error
                    cs_cntxt_find_release_error(&mad, pm_async_send_rcv_cntxt);
                    break;
                default:
					// discard unexpected method
					IB_LOG_INFO_FMT( __func__,
		       			"Unexpected MAD ERROR received: method=0x%x SLID=0x%x TID=0x%016"CS64"x",
		       			mad.base.method, mad.addrInfo.slid, mad.base.tid);
                    //(void)sm_dump_mai("pm_async_rcv", &mad);
                    break;
                }
            default:
                break;
            }
        } else if (status != VSTATUS_TIMEOUT) {
            IB_LOG_WARNRC("error on mai_recv for PM async receive rc:", status);
        }

        // age context entries if necessary
        (void)vs_time_get(&now);
        if ((now - lastTimeAged) >= timeout) {
			/* cs_cntxt_age returns the smallest timeout of all contexts */
            timeout = cs_cntxt_age(pm_async_send_rcv_cntxt);
			/* if there are no context entries (timeout will be 0) or timeout
			 * value is too small, set timeout to pm_config.MinRcvWaitInterval/4
			 * this avoids giving too small a timeout value to mai_recv()
			 */
			if (timeout == 0 || timeout < (pm_config.MinRcvWaitInterval*1000/4))	
				timeout = pm_config.MinRcvWaitInterval *1000;
			// since this loop also checks for PM shutdown, make sure we don't
			// use too long a timeout either
			if (timeout > VTIMER_1S/10)
				timeout = VTIMER_1S/10;
            lastTimeAged = now;
        }

		// no need to call pm dispatcher, handled in callbacks
	} // end while loop

	IB_LOG_INFO0("thread: Exiting OK");

    //	Delete the filters.
	if (mai_filter_delete(hpma, &filter2, VFILTER_SHARE | VFILTER_PURGE) != VSTATUS_OK) {
		IB_LOG_ERROR0("can't delete topology error receive filter");
	}
freefilter:
	if (mai_filter_delete(hpma, &filter, VFILTER_SHARE | VFILTER_PURGE) != VSTATUS_OK) {
		IB_LOG_ERROR0("can't delete topology async receive filter");
	}

done:
	g_pmAsyncRcvThreadRunning = FALSE;
} // end pm_async_rcv


void
pm_async_rcv_kill(void){
	pm_async_rcv_exit = 1;
}
