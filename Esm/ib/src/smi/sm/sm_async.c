/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_async.c							     //
//									     //
// DESCRIPTION								     //
//    This thread receives the asynchronous MADs from the fabric and 	     //
//    processes them.  These MADs include SubnGet(SMInfo), 		     //
//    SubnSet(SMInfo), Trap(*).						     //
//									     //
// DATA STRUCTURES							     //
//    NONE								     //
//									     //
// FUNCTIONS								     //
//    NONE								     //
//									     //
// DEPENDENCIES								     //
//    ib_types.h							     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "ib_macros.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "cs_context.h"
#include "cs_queue.h"
#include "sm_dbsync.h"

#ifdef __VXWORKS__
#include "tms/idb/icsSmMib.h"
#endif

extern	SmMaiHandle_t *fd_sminfo;
extern	Status_t policy_main(Mai_t *);
extern	Status_t policy_topology(Mai_t *);
extern	Status_t policy_node(Mai_t *);
extern	Status_t policy_port(Mai_t *);
extern	Status_t policy_send_data(Mai_t *, uint8_t *, uint32_t);
extern  Status_t sm_sa_forwardNotice(STL_NOTICE * noticep);
extern  int      sm_sa_getNoticeCount (STL_NOTICE * noticep);

#ifdef __VXWORKS__
extern  uint32_t sm_trapThreshold;
extern  uint8_t  sa_dynamicPlt[];
extern  STL_LID sm_mcast_mlid_table_cap;

char    pkey_description[100];
#endif

static  int async_main_exit = 0;
static  uint64_t new_sm_timer = 0xffffffffffffffffull;

// SM Notice context pool
generic_cntxt_t  sm_notice_cntxt;

static  uint32_t sm_notice_max_context = 0;
static  uint32_t sm_trap_forward_queue_size = 0;


// SM-SA trap forward request queue
cs_Queue_ptr sm_trap_forward_queue;


//
// process trap forward requests from the SM and SA
//
void sm_process_trap_forward_requests(void) {
    STL_NOTICE    *noticep    = NULL;
    uint32_t	pktcount	= 0;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if ((noticep = (STL_NOTICE *)cs_queue_Front( sm_trap_forward_queue )) != NULL) {
        pktcount = sm_sa_getNoticeCount(noticep);
        /* 
         * we want to hold off sending out new notices until all responses to previous notice are ACK'd 
         * we do that by waiting for numFree to equal poolSize again
         */
        if (pktcount && pktcount <= sm_notice_cntxt.numFree &&
            sm_notice_cntxt.numFree == sm_notice_cntxt.poolSize) {
            // forward trap reliably
            sm_sa_forwardNotice(noticep);
            // remove queue front entry after successful transmission
            cs_queue_Dequeue( sm_trap_forward_queue );
            // free the notice space
            vs_pool_free(&sm_pool, noticep);
        } else if (pktcount && pktcount > sm_notice_cntxt.numFree) {
			// this only happens if size of fabric exceeds SubnetSize
            IB_LOG_VERBOSE("not enough context available",
                               sm_notice_cntxt.numFree);
        } else if (!pktcount){
            /* remove queue front since no one cares about this event */
            cs_queue_Dequeue( sm_trap_forward_queue );
            /* free the notice space */
            vs_pool_free(&sm_pool, noticep);
        }
    }
	IB_EXIT(__func__, 0);
}

//
// initialize static data
//
void
async_init(void) {
	
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	sm_notice_max_context = sm_config.subnet_size + (sm_config.subnet_size / 10);
	sm_trap_forward_queue_size = MAX(1536, 2 * sm_notice_max_context);
	
	IB_EXIT(__func__, 0);
}

void
async_main(uint32_t argc, uint8_t ** argv) {
	uint64_t	now, nextTime;
	uint64_t	delta_time;
	Status_t	status;
	Filter_t	filter;
	Mai_t		mad;
    uint64_t	lastTimeAged=0, timeout=0;
    uint32_t    index = 99;
    IBhandle_t  handles[2];
	int			lastState;
    uint64_t    lastTimePortChecked=0;
	uint8_t		localPortFailure=0;
	STL_PORT_INFO	portInfo;

	IB_ENTER(__func__, 0, 0, 0, 0);

//
//      Get my thread name.
//
	(void)vs_thread_name(&sm_threads[SM_THREAD_ASYNC].name);

//
//	Shutdown any previous version of the SM which may be running.
//
	(void)sm_control_init();
	(void)sm_control_notify();

    //
    // initialize Sm Notice context pool
    //
    memset((void *)&sm_notice_cntxt, 0, sizeof(sm_notice_cntxt));
    sm_notice_cntxt.hashTableDepth = CNTXT_HASH_TABLE_DEPTH;
    sm_notice_cntxt.poolSize = sm_notice_max_context;
    sm_notice_cntxt.maxRetries = sm_config.max_retries;
    sm_notice_cntxt.ibHandle = fd_saTrap->fdMai;
    sm_notice_cntxt.errorOnSendFail = 0;
#ifdef IB_STACK_OPENIB
	// for openib we let umad do the timeouts.  Hence we add 1 second to
	// the timeout as a safety net just in case umad loses our response.
    sm_notice_cntxt.timeoutAdder = VTIMER_1S;
#endif
    timeout = sm_config.rcv_wait_msec * 1000;
    status = cs_cntxt_instance_init (&sm_pool, &sm_notice_cntxt, timeout);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: failed to initialize the SM context instance, status=%d", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}
    //
    // initialize the SM-SA trap forward request queue
    //
    if ((sm_trap_forward_queue = cs_queue_CreateQueue( &sm_pool, sm_trap_forward_queue_size )) == NULL) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: failed to initialize the SM-SA trap forward queue, terminating");
		IB_LOG_ERROR0("failed to initialize the SM-SA trap forward queue, terminating");
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
    }

	//
	//	Init dispatcher.
	//
	status = sm_dispatch_init(&sm_asyncDispatch, sm_config.max_parallel_reqs);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to initialize dispatcher, rc:", status);
		return;
	}
	
    //
    //	Create the Subn*(SMInfo) MAD filters for the state thread.
    //
	SM_Filter_Init(&filter);
	filter.value.bversion = STL_BASE_VERSION;
	filter.value.cversion = STL_SM_CLASS_VERSION;
	filter.value.mclass = MAD_CV_SUBN_LR & 0x7f;	// LR or DR
	filter.value.method = MAD_CM_GET;
	filter.value.aid = MAD_SMA_SMINFO;

	filter.mask.bversion  = 0xff;
	filter.mask.cversion  = 0xff;
	filter.mask.mclass  = 0x7f;
	filter.mask.method  = 0xff;
	filter.mask.aid  = 0xffff;
	MAI_SET_FILTER_NAME (&filter, "sm_async");

	status = mai_filter_create(fd_async->fdMai, &filter, VFILTER_SHARE);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: can't create Get(SMInfo) filter %d", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}
	SM_Filter_Init(&filter);
	filter.value.bversion = STL_BASE_VERSION;
	filter.value.cversion = STL_SM_CLASS_VERSION;
	filter.value.mclass = MAD_CV_SUBN_LR & 0x7f;	// LR or DR
	filter.value.method = MAD_CM_SET;
	filter.value.aid = MAD_SMA_SMINFO;

	filter.mask.bversion  = 0xff;
	filter.mask.cversion  = 0xff;
	filter.mask.mclass  = 0x7f;
	filter.mask.method  = 0xff;
	filter.mask.aid  = 0xffff;
	MAI_SET_FILTER_NAME (&filter, "sm_async");

	status = mai_filter_create(fd_async->fdMai, &filter, VFILTER_SHARE);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: can't create Set(SMInfo) filter %d", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

    //
    //	Create the Trap(*) MAD filter for the trap thread.
    //
	SM_Filter_Init(&filter);
	filter.value.mclass = MAD_CV_SUBN_LR;
	filter.value.method = MAD_CM_TRAP;
	filter.mask.mclass  = MAI_FMASK_ALL;
	filter.mask.method  = 0xff;
	MAI_SET_FILTER_NAME (&filter, "sm_async");

	status = mai_filter_create(fd_async->fdMai, &filter, VFILTER_SHARE);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: can't create Trap(*) filter %s", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

    //
    //	Set the filter for catching just CM_REPORT responses on fd_saTrap.
    //
	SA_Filter_Init(&filter);
	filter.value.mclass = MAD_CV_SUBN_ADM;
	filter.value.method = MAD_CM_REPORT_RESP;
	filter.value.aid = SA_NOTICE;
	filter.mask.mclass  = 0xff;
	filter.mask.method  = 0xff;
	filter.mask.aid  = 0xffff;
	MAI_SET_FILTER_NAME (&filter, "sm_saTrap");

	status = mai_filter_create(fd_saTrap->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: can't create CM_REPORT_RESP filter %d", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

    //
    //	Set the filter for catching just CM_REPORT errors on fd_saTrap.
    //
	SA_Filter_Init(&filter);

	filter.type = MAI_TYPE_ERROR;
	filter.value.mclass = MAD_CV_SUBN_ADM;
	filter.value.method = MAD_CM_REPORT;
	filter.value.aid = SA_NOTICE;
	filter.mask.mclass  = 0xff;
	filter.mask.method  = 0xff;
	filter.mask.aid  = 0xffff;
	MAI_SET_FILTER_NAME (&filter, "sm_saTrap_error");

	status = mai_filter_create(fd_saTrap->fdMai, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"sm_async: can't create CM_REPORT error filter %d", status);
		(void)vs_thread_exit(&sm_threads[SM_THREAD_ASYNC].handle);
	}

    //
    //	Start up the topology thread.
    //
	sm_trigger_sweep(SM_SWEEP_REASON_INIT);

    //
    //	All this thread does is read MADs and hand them off to the correct
    //	thread to process.
    //
	async_main_exit = 0;
    new_sm_timer = 0xffffffffffffffffull;
	delta_time = 50000ull;  /* 1/20 second in usecs */

    (void)vs_time_get(&lastTimeAged);
    nextTime = lastTimeAged + sm_masterCheckInterval;

    handles[0] = fd_async->fdMai;
    handles[1] = fd_saTrap->fdMai;
	while (1) {
        status = mai_recv_handles(handles, 2, delta_time, &index, &mad);

        if (status == VSTATUS_OK) {
            if ((mad.base.cversion != SA_MAD_CVERSION && mad.base.cversion != STL_SA_CLASS_VERSION) &&
                mad.base.bversion != MAD_BVERSION && mad.base.bversion != STL_BASE_VERSION)
                continue;

            /* see which handle had data */
            switch (index) {
            case 0:
                /* process switch traps and sminfo requests on fd_async */
                switch (mad.type) {
                case MAI_TYPE_EXTERNAL:
                    switch (mad.base.method) {
                    case MAD_CM_TRAP:
                        if (sm_state == SM_STATE_MASTER) {
                            (void)sa_Trap(&mad);
                        }
                        break;
                    case MAD_CM_GET:
						INCREMENT_COUNTER(smCounterRxGetSmInfo);
                        (void)state_event_mad(&mad);
                        break;
                    case MAD_CM_SET:
						INCREMENT_COUNTER(smCounterRxSetSmInfo);
                        (void)state_event_mad(&mad);
                        break;
                    default:
						smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
                        	"sm_async: bad method received on trap/sminfo file descriptor (QP0), method=%d", 
                                     mad.base.method);
                        (void)sm_dump_mai("sm_async", &mad);
                        break;
                    }
                    break;
                case MAI_TYPE_INTERNAL:
                    (void)sm_control(&mad);
                    break;
                default:
                    break;
                }
                break;
            case 1:
                /* process notification ACK on fd_saTrap */
                switch (mad.type) {
                case MAI_TYPE_EXTERNAL:
                    switch (mad.base.method) {
                    case MAD_CM_REPORT_RESP:
						INCREMENT_COUNTER(smCounterSaRxReportResponse);
                        cs_cntxt_find_release(&mad, &sm_notice_cntxt);
                        break;
                    default:
						smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
                        	"sm_async: bad method received on notice file descriptor (QP1), method=%d", mad.base.method);
                        (void)sm_dump_mai("sm_async", &mad);
                        break;
                    }
                    break;
				case MAI_TYPE_ERROR:
					// only happens for openib.  This return indicated the
					// lower level stack has returned our original sent MAD
					// to indicate a failure sending it or a lack of a response
                    switch (mad.base.method) {
                    case MAD_CM_REPORT:
                        cs_cntxt_find_release_error(&mad, &sm_notice_cntxt);
                        break;
                    default:
						smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
                        	"sm_async: bad method received on notice error file descriptor (QP1), method=%d", mad.base.method);
                        (void)sm_dump_mai("sm_async", &mad);
                        break;
                    }
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
        } else if (status != VSTATUS_TIMEOUT) {
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
           		"sm_async: receive error on notice or trap file descriptor, status=%d", status);
        } else {
            /* now process any trap forward requests on the queue */
            sm_process_trap_forward_requests();
        }

        /* age the trap forwarding context entries if necessary */
        (void)vs_time_get(&now);
        if ((now - lastTimeAged) >= 100000ull) {
            cs_cntxt_age(&sm_notice_cntxt);
            lastTimeAged = now;
        }

		if(async_main_exit == 1){
#ifdef __VXWORKS__
			ESM_LOG_ESMINFO("SM Async Task exiting OK.", 0);
#endif
            /* set sm state to inactive and release topology thread, if required */
			lastState = sm_state;
            (void)sm_transition(SM_STATE_NOTACTIVE);
			if (lastState == SM_STATE_STANDBY) {
				(void)state_event_timeout();
			}
            sm_trigger_sweep(SM_SWEEP_REASON_UNDETERMINED);
			break;
		}

        /* check for sweep rate change */
        if (new_sm_timer != 0xffffffffffffffffull) {
            /* 
             * If we were not sweeping and now want to, we must force the wakeup time to be set 
             */
            if (sm_config.timer == 0) {
                topology_wakeup_time = 1;
            } else if (new_sm_timer == 0) {
                /* we don't want to sweep at all starting now */
                topology_wakeup_time = 0;
            }
            sm_config.timer = new_sm_timer;
            new_sm_timer = 0xffffffffffffffffull;
			
			// May be set to INTERVAL_CHANGE in sm_SetSweepRate
			setResweepReason(SM_SWEEP_REASON_FORCED);

            /* force wakeup of topology thread now if waiting on semaphore */
            if (sm_config.timer != 0 && topology_wakeup_time > 0ull) 
                topology_wakeup_time = now;
        }

        /* 
         * If a change trap is received, we signal sm_top to rediscover the fabric  
         * Wait one second to allow traps to accumulate before asking
         */
		/*  PR 111216 - Even in case we have not received any traps, check the state of our
		 *  local port. After a switch reboot, we do not get any traps but our local
		 *  port can go DOWN and come to INIT state and if we are master, it will stay like
		 *  that until we do a sweep. In such a scenario, we need to request a Fabric
		 *  Discovery. Do this checking only once every 2 seconds as we will come here every
		 *  1/20th of a second due to the mai_recv_handles time out above.
		 *	
		 *  Its possible that we might be in INIT state because our HFI port went down
		 *  and came back up for some other reason and we are checking our port before the
		 *  switch got a chance to send a trap. But that still should not cause an
		 *  immediate double sweep even if the switch sends a trap immediately after
		 *  this, as we anyway throttle traps that come with in 1 second of having
		 *  requested a sweep.
		 */
		/* First time we come here, we will always check port state as lastTimePortChecked will be 0*/
		if ((sm_state == SM_STATE_MASTER) && !smFabricDiscoveryNeeded && !isSweeping && 
			((now - lastTimePortChecked) > (VTIMER_1S * 2))) {
			uint8 path[64];
			memset((void *)path, 0, 64);
			SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
			status = SM_Get_PortInfo(fd_sminfo, 1<<24, &addr, &portInfo);
			if (status != VSTATUS_OK) {
				IB_LOG_ERRORRC("can't get SM port PortInfo rc:", status);
			} else {
				if (portInfo.PortStates.s.PortState <= IB_PORT_INIT) {
					localPortFailure = 1;
					smFabricDiscoveryNeeded = 1;
				}
			}
			lastTimePortChecked = now;	
		}
        if (smFabricDiscoveryNeeded && lastTimeDiscoveryRequested == 0) {
            lastTimeDiscoveryRequested = now;
        } else if (smFabricDiscoveryNeeded) {
            uint64_t time_interval = VTIMER_1S;

            if (sm_resweep_reason == SM_SWEEP_REASON_TRAP_EVENT) time_interval *= sm_config.trap_hold_down;

            if ((now - lastTimeDiscoveryRequested) > time_interval) {
				// At this point discovery needed due to multiple traps (re-sweep reason already
				// set); or local port failure.
				if (localPortFailure) {
					localPortFailure = 0;
					setResweepReason(SM_SWEEP_REASON_LOCAL_PORT_FAIL);
				}
                topology_wakeup_time = now;
            }
        }

		if (topology_wakeup_time > 0ull) {
            /* standby must check on master every sm_masterCheckInterval seconds */
			if (((sm_state == SM_STATE_DISCOVERING || sm_state == SM_STATE_MASTER)
			      && now >= topology_wakeup_time)
				|| ((sm_state == SM_STATE_STANDBY || (sm_state == SM_STATE_MASTER && sm_config.monitor_standby_enable))
					 && now >= nextTime)) {
                nextTime = now + sm_masterCheckInterval;
				(void)state_event_timeout();
			}
		}
	}

	/* release the SM context pool */
    status = cs_cntxt_instance_free (&sm_pool, &sm_notice_cntxt);
    /* free the SM trap foward request queue */
    cs_queue_DisposeQueue( &sm_pool, sm_trap_forward_queue );
	sm_dispatch_destroy(&sm_asyncDispatch);
}


void
async_main_kill(void){
	async_main_exit = 1;
}

uint32_t sm_getSweepRate(void) {
    return sm_config.timer/1000000;
}

void sm_setSweepRate(uint32_t rate) {
	uint8_t isValid = 1;
    /* 0 or 3 seconds to 86400 seconds (24 hours) */
    if (rate < 3 || rate > 86400) {
        if (rate == 0) {
            new_sm_timer = 0;
        } else {
            printf("sm_setSweepRate: Sweep rate must be between 3 and 86400 (24 hrs) seconds!\n");
			isValid = 0;
        }
    } else {
        new_sm_timer = rate;
        new_sm_timer *= 1000000;
    }

	if (isValid) setResweepReason(SM_SWEEP_REASON_INTERVAL_CHANGE);
}

void sm_forceSweep(const char* reason) {
	if (new_sm_timer == 0xffffffffffffffffull) {
		IB_LOG_INFINI_INFO_FMT(__func__, "SM Forced Sweep scheduled: %s", reason);
	}
    new_sm_timer = sm_config.timer;
}

// send the XML configuration file to all standby and inactive SM's
int sm_send_xml_file(uint8_t activate) {
	
	SMDBSyncFilep syncFile = NULL;


	if (sm_state != SM_STATE_MASTER) {
		return -2;
	}

	syncFile = malloc(sizeof(SMDBSyncFile_t));
	if (syncFile == NULL) {
		return -2;
	}

	syncFile->version = DBSYNC_FILE_TRANSPORT_VERSION;
	syncFile->length = sizeof(SMDBSyncFile_t);
	StringCopy(syncFile->name, "opafm.xml", SMDBSYNCFILE_NAME_LEN);
	syncFile->type = DBSYNC_FILE_XML_CONFIG;
	syncFile->activate = activate;

	// sync the file to all standby SM's
	sm_dbsync_syncFile(DBSYNC_TYPE_BROADCAST_FILE, syncFile);
	free(syncFile);

	return 0;
}

#ifdef __VXWORKS__

uint32_t sm_getPriority(void) {
	return sm_config.priority;
}

uint32_t sm_getElevatedPriority(void) {
	return sm_config.elevated_priority;
}

uint32_t sm_getPmPriority(void) {
	return pm_config.priority;
}

uint32_t sm_getPmElevatedPriority(void) {
	return pm_config.elevated_priority;
}

uint32_t sm_getLMC(void) {
	return sm_config.lmc;
}

uint8_t sm_getSwitchLifetime(void) {
	return sm_config.switch_lifetime_n2;
}

uint8_t sm_getHoqLife(void) {
	return sm_config.hoqlife_n2;
}

uint8_t sm_getVLStall(void) {
	return sm_config.vlstall;
}

uint32_t sm_getPltValue(uint32_t index) {
	if (index < 1 || index > 9)
		return 0;
	return sa_dynamicPlt[index];
}

int sm_getDynamicPltSupport(void) {
	return sa_dynamicPlt[0];
}

unsigned long long sm_getMKey(void) {
	return sm_config.mkey;
}

unsigned long long sm_getKey(void) {
	return sm_config.sm_key;
}

unsigned long long sm_getGidPrefix(void) {
	return sm_config.subnet_prefix;
}

uint32_t sm_getSubnetSize(void) {
	return sm_config.subnet_size;
}

uint32_t sm_getTopoErrorThreshold(void) {
	return sm_config.topo_errors_threshold;
}

uint32_t sm_getTopoAbandonThreshold(void) {
	return sm_config.topo_abandon_threshold;
}

uint32_t sm_getMaxRetries(void) {
	return sm_config.max_retries;
}

uint32_t sm_getRcvWaitTime(void) {
	return sm_config.rcv_wait_msec;
}

uint32_t sm_getNonRespDropTime(void) {
	return sm_config.non_resp_tsec;
}

uint32_t sm_getNonRespDropSweeps(void) {
	return sm_config.non_resp_max_count;
}

uint32_t sm_getMcLidTableCap(void) {
	return sm_mcast_mlid_table_cap;
}

uint32_t sm_getMasterPingInterval(void) {
	return (sm_masterCheckInterval / VTIMER_1S);
}

uint32_t sm_getMasterPingFailures(void) {
	return sm_config.master_ping_max_fail;
}

uint32_t sm_getDbSyncInterval(void) {
	return sm_config.db_sync_interval;
}

uint32_t sm_getTrapThreshold(void) {
	return sm_trapThreshold;
}

uint32_t sm_getAppearanceMsgThresh(void) {
	return sm_config.node_appearance_msg_thresh ;
}

uint32_t sm_getMcastCheck(void) {
	return sm_mc_config.disable_mcast_check;
}

void acquireVfLock(void) {
	(void)vs_rdlock(&old_topology_lock);
}

void releaseVfLock(void) {
	(void)vs_rwunlock(&old_topology_lock);
}

// number of virtual fabrics
uint32_t sm_numberOfVfs(void) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	if (VirtualFabrics == NULL)
		return 0;
	return VirtualFabrics->number_of_vfs_all;
}

// virtual fabric name given index
char* sm_VfName(uint32_t index) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	if (VirtualFabrics == NULL)
		return NULL;
	if (VirtualFabrics->number_of_vfs_all == 0)
		return NULL;
	if (index >= VirtualFabrics->number_of_vfs_all)
		return NULL;
	return VirtualFabrics->v_fabric_all[index].name; 
}

// number of multicast groups given virtual fabric name
uint32_t sm_numberOfVfMcastGroups(char* vfName) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);
	if (vf == NULL)
		return 0;
	return vf->number_of_default_groups;
}

uint32_t sm_getMibOptionFlags(void) {

	uint32_t flags = 0;
	VF_t *vfp = NULL;

	(void)vs_rdlock(&old_topology_lock);
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;

	if (VirtualFabrics != NULL) {
		int vf;
		// find first active vf
		for (vf = 0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
			if (VirtualFabrics->v_fabric_all[vf].standby) continue;
			vfp = &VirtualFabrics->v_fabric_all[vf];
		}

		// get the SM_CREATE_MCGRP_MASK option flag
		if (VirtualFabrics->number_of_vfs_all != 0 && vfp->default_group != NULL) {
			if (vfp->default_group->def_mc_create)
				flags |= SM_CREATE_MCGRP_MASK;
		}
	}

	// get the SM_DYNAMICPLT_MASK option flag
	if (sa_dynamicPlt[0])
		flags |= SM_DYNAMICPLT_MASK;

	// get the SM_COMMON_MCAST_MTU_RATE option flag
	if (sm_mc_config.disable_mcast_check)
		flags |= SM_COMMON_MCAST_MTU_RATE;

	(void)vs_rwunlock(&old_topology_lock);

	return flags;
}

// virtual fabric PKey given name
uint16_t sm_getMibPKey(char* vfName) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);
	if (vf == NULL)
		return 0;
	return vf->pkey;
}

// virtual fabric PKey description
char* sm_getMibPKeyDescription(char* vfName) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);
	if (vf == NULL)
		StringCopy(pkey_description, "Default PKey", sizeof(pkey_description));
	else 
		sprintf(pkey_description, "Virtual Fabric - %s", vf->name);
	return pkey_description;
}

// virtual fabric Multicast Group PKey 
uint16_t sm_getDefMcGrpPKey(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_pkey;
}

// virtual fabric Multicast Group MTU
uint8_t sm_getDefMcGrpMtu(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_mtu_int;
}

// virtual fabric Multicast Group Rate Int
uint8_t sm_getDefMcGrpRate(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_rate_int;
}

// virtual fabric Multicast Group Service Level
uint8_t sm_getDefMcGrpSl(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_sl;
}

// virtual fabric Multicast Group QKey
uint32_t sm_getDefMcGrpQKey(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_qkey;
}

// virtual fabric Multicast Group Flow Label
uint32_t sm_getDefMcGrpFlowLabel(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_fl;
}

// virtual fabric Multicast Group TClass
uint8_t sm_getDefMcGrpTClass(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_tc;
}

// virtual fabric Multicast Create
uint8_t sm_getDefMcGrpCreate(char* vfName, uint32_t group) {
	VirtualFabrics_t* VirtualFabrics = old_topology.vfs_ptr;
	VFDg_t* g;
	uint32_t i = 0;
	VF_t* vf = findVfPointer(VirtualFabrics, vfName);

	if (vf == NULL || group > vf->number_of_default_groups - 1)
		return 0;
	g = vf->default_group;
	while (i++ < group) 
		g = g->next_default_group;

	return g->def_mc_create;
}

#endif // __VXWORKS__

