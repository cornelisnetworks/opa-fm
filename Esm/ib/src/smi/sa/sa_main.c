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
//									     //
// FILE NAME								     //
//    sa.c								     //
//									     //
// DESCRIPTION								     //
//    This file contains the main entry point into the SA subsystem. 	     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_main								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "ib_macros.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_l.h"
#include "sm_counters.h"
#include "sa_l.h"

//===========================================================================//
extern	uint32_t	sm_port;

extern	Pool_t		sm_pool;
extern	Sema_t		sa_sema;
extern 	uint8_t		nullData[];
extern  uint64_t	topology_wakeup_time;
uint32_t            sa_mft_reprog=0;

uint8_t				*sa_data;
uint32_t			sa_data_length;
uint32_t			sa_max_ib_path_records;
uint32_t            saDebugRmpp=0;  // control SA RMPP INFO debug messages; default off in ESM
uint32_t			saRmppCheckSum=0;  // control whether to checksum each rmpp response at start and end of transfer
STL_CLASS_PORT_INFO	saClassPortInfo;
SubscriberTable_t   saSubscribers;
ServiceRecTable_t   saServiceRecords;

SACache_t           saCache;
SACacheBuildFunc_t  saCacheBuildFunctions[SA_NUM_CACHES] = {
	sa_NodeRecord_BuildCACache,
	sa_NodeRecord_BuildSwitchCache,
};


//OpaServiceRecord_t	* globalServiceRecords = NULL;

/************ support for dynamic update of switch config parms *************/
uint8_t sa_dynamicPlt[DYNAMIC_PACKET_LIFETIME_ARRAY_SIZE]={1,16,17,17,18,18,18,18,19,19};   // entry zero set to 1 indicates table in use

static  Lock_t      sa_cntxt_lock;
static 	sa_cntxt_t	*sa_hash[ SA_CNTXT_HASH_TABLE_DEPTH ];
//static 	sa_cntxt_t	sa_cntxt_pool[ sa_max_cntxt ];
static 	sa_cntxt_t	*sa_cntxt_pool;
static 	sa_cntxt_t	*sa_cntxt_free_list ;
static  int	sa_cntxt_nalloc = 0 ;
static 	int	sa_cntxt_nfree = 0 ;
static  int sa_main_reader_exit = 0;
static  int sa_main_writer_exit = 0;

#define	INCR_SA_CNTXT_NFREE() {  \
    if (sa_cntxt_nfree < sa_max_cntxt) {        \
        sa_cntxt_nfree++;        \
		SET_PEAK_COUNTER(smMaxSaContextsFree, sa_cntxt_nfree); \
    } else {                      \
		IB_LOG_ERROR("sa_cntxt_retire: sa_cntxt_nfree already at max:", sa_cntxt_nfree);\
    }\
}

#define	DECR_SA_CNTXT_NFREE() {  \
    if (sa_cntxt_nfree) {        \
        sa_cntxt_nfree--;        \
    } else {                      \
		IB_LOG_ERROR0("sa_cntxt_get: can't decrement sa_cntxt_nfree, already zero");\
    }\
}

#define	INCR_SA_CNTXT_NALLOC() {  \
    if (sa_cntxt_nalloc < sa_max_cntxt) {        \
        sa_cntxt_nalloc++;        \
		SET_PEAK_COUNTER(smMaxSaContextsInUse, sa_cntxt_nalloc); \
    } else {                      \
		IB_LOG_ERROR("sa_cntxt_get: sa_cntxt_nalloc already at max:", sa_cntxt_nalloc);\
    }\
}

#define	DECR_SA_CNTXT_NALLOC() {  \
    if (sa_cntxt_nalloc) {        \
        sa_cntxt_nalloc--;        \
    } else {                      \
		IB_LOG_ERROR0("sa_cntxt_retire: can't decrement sa_cntxt_nalloc, already zero");\
    }\
}

#define sa_cntxt_delete_entry( list, entry )	\
	if( (list) == (entry) ) (list) = (list)->next ; \
	else { \
		(entry)->prev->next = (entry)->next ; \
		if( (entry)->next ) (entry)->next->prev = (entry)->prev ; \
	} \
	(entry)->next = (entry)->prev = NULL 

#define sa_cntxt_insert_head( list, entry ) 	\
	if( (list) ) (list)->prev = (entry) ;	\
	(entry)->next = (list) ;		\
	(list) = (entry) ;  \
	(entry)->prev = NULL ;

static uint64_t	timeLastAged=0;
static uint64_t timeMftLastUpdated=0;

int sa_filter_reports(Mai_t * data)
{
	if (data->base.method == SA_CM_REPORT)
	{
		return 1;
	}
	
	return 0;
}

static int sa_filter_validate_mad(Mai_t *maip, STL_SA_MAD_HEADER *samad)
{
    int rc = 0;
        
    if (samad->smKey)
        rc = 1;
    else {
        // In order to secure important fabric information, such as P_Key
        // configuration, routing tables, etc., the SA shall limit the information
        // made available to clients NOT using the 0xffff P_Key. Queries from such clients
        // will be restricted to the following queries. 
        switch (maip->base.aid) {
        case SA_PATH_RECORD:
        case SA_MULTIPATH_RECORD:
        case SA_NODE_RECORD:
        case SA_INFORMINFO:
        case SA_INFORM_RECORD:
        case SA_NOTICE:
        case SA_SERVICE_RECORD:
        case SA_MCMEMBER_RECORD:
        case SA_CLASSPORTINFO:
	case STL_SA_ATTR_VF_INFO_RECORD:
            break;
        case SA_PORTINFO_RECORD:
            if (maip->base.bversion == IB_BASE_VERSION)
                break;
        default:
            if (!smValidateGsiMadPKey(maip, 1, sm_config.sma_spoofing_check))
                rc = 1;
            break;
        }
    }

   if (rc && saDebugRmpp) {
        if (samad->smKey) 
            IB_LOG_WARN_FMT("sa_filter_validate_mad", 
                            "Dropping packet, invalid SM_Key "FMT_U64" %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            samad->smKey, sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
        else 
            IB_LOG_WARN_FMT("sa_filter_validate_mad", 
                            "Dropping packet, invalid P_Key 0x%x %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            maip->addrInfo.pkey, sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
    }

    if (rc) {
        // if the MAD is not valid, then indicate to the MAI filter handling
        // code that no match has been found.  Also, change the MAI type to
        // indicate to the MAI filter handling code to just drop the packet.  
        maip->type = MAI_TYPE_DROP;
    }
    
    return (rc) ? 1 : 0;
}

int sa_reader_filter(Mai_t *maip)
{
	STL_SA_MAD_HEADER samad;

    // check whether a previous filter has indicated to drop the packet 
    if (maip->type == MAI_TYPE_DROP)
        return 1;   // indicate that no match has been found

    BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER*)maip->data, &samad);
    if (sa_filter_validate_mad(maip, &samad))
        return 1;   // indicate that no match has been found

	if (maip->base.method == SA_CM_REPORT) return 1;
    // reader is only processing new requests
	if (maip->base.method == SA_CM_GETTABLE || maip->base.method == SA_CM_GETMULTI || maip->base.method == SA_CM_GETTRACETABLE)
	{
        // check for inflight rmpp
        if ((samad.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) && 
            ((samad.rmppType == RMPP_TYPE_ACK && samad.segNum != 0) 
                                  ||
              samad.rmppType == RMPP_TYPE_STOP || samad.rmppType == RMPP_TYPE_ABORT)) {
            return 1;  // not processing inflight rmpp stuff either
        }
	}
    if (saDebugRmpp) {
        IB_LOG_INFINI_INFO_FMT( "sa_reader_filter",
               "Processing request for %s[%s] from LID[0x%x], TID="FMT_U64, 
               sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
    }
	return 0;
}

int sa_writer_filter(Mai_t *maip)
{
	STL_SA_MAD_HEADER samad;

    BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER*)maip->data, &samad);
    if (sa_filter_validate_mad(maip, &samad))
        return 1;   // indicate that no match has been found

    // the sa writer thread is only responsible for in flight rmpp packets
	if (maip->base.method == SA_CM_GETTABLE || maip->base.method == SA_CM_GETMULTI || maip->base.method == SA_CM_GETTRACETABLE)
	{
        // check for inflight rmpp
        if ((samad.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) && 
            ((samad.rmppType == RMPP_TYPE_ACK && samad.segNum > 0) || samad.rmppType == RMPP_TYPE_STOP 
             || samad.rmppType == RMPP_TYPE_ABORT)) {
            if (saDebugRmpp) {
                IB_LOG_INFINI_INFO_FMT( "sa_writer_filter",
                       "Processing inflight Rmpp packet for %s[%s] from LID[0x%x], TID="FMT_U64, 
                       sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
            }
            return 0;  // process inflight rmpp responses
        }
	}
	return 1;
}

int
sa_main(void) {
	Status_t		status;
	int 			i;

	IB_ENTER("sa_main", 0, 0, 0, 0);

    if (sa_SubscriberInit() != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "sa_main: Can't allocate Subscriber hash table");
		return 1;
	}
    
    if (sa_ServiceRecInit() != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "sa_main: Can't allocate Service Record hash table");
		return 1;
	}
    
    // 
    //  Zero context hash table
    //
	memset( sa_hash, 0, sizeof( sa_hash ));
	sa_cntxt_pool = NULL;
    IB_LOG_VERBOSE("sa_main: Allocating SA context pool with num entries=", sa_max_cntxt);
	status = vs_pool_alloc(&sm_pool, sizeof(sa_cntxt_t) * sa_max_cntxt, (void *)&sa_cntxt_pool);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "sa_main: Can't allocate SA context pool");
		return 2;
	}
	memset( sa_cntxt_pool, 0, sizeof( sa_cntxt_t ) * sa_max_cntxt);
	sa_cntxt_free_list = NULL ;
	for( i = 0 ; i < sa_max_cntxt ; ++i ) {
		sa_cntxt_insert_head( sa_cntxt_free_list, &sa_cntxt_pool[i] );
	}
    sa_cntxt_nfree = sa_max_cntxt;
    sa_cntxt_nalloc = 0;

    // initialize SA context lock
	status = vs_lock_init(&sa_cntxt_lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("sa_main: can't initialize SA context pool lock rc:", status);
        return 3;
	}
    //
    //	Allocate the SA storage pool.
    //
	status = vs_pool_alloc(&sm_pool, sa_data_length, (void*)&sa_data);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "sa_main: can't allocate sa data");
		return 4;
	}
	memset(sa_data,0,sa_data_length);

    //
    //	Fill in my ClassPortInfo_t and add it to the database.
    //
	(void)memset((void *)&saClassPortInfo, 0, sizeof(STL_CLASS_PORT_INFO));
	saClassPortInfo.BaseVersion = STL_BASE_VERSION; //MAD_BVERSION;
	saClassPortInfo.ClassVersion = STL_SA_CLASS_VERSION; //SA_MAD_CVERSION;
	saClassPortInfo.CapMask =
		STL_CLASS_PORT_CAPMASK_CM2 |
		STL_SA_CAPABILITY_MULTICAST_SUPPORT |
		STL_SA_CAPABILITY_PORTINFO_CAPMASK_MATCH |
		STL_SA_CAPABILITY_PA_SERVICES_SUPPORT;
	saClassPortInfo.u1.s.CapMask2 =
		STL_SA_CAPABILITY2_QOS_SUPPORT |
		STL_SA_CAPABILITY2_MFTTOP_SUPPORT |
		STL_SA_CAPABILITY2_FULL_PORTINFO |
		STL_SA_CAPABILITY2_EXT_SUPPORT |
		STL_SA_CAPABILITY2_DGDTRECORD_SUPPORT;
	saClassPortInfo.u1.s.RespTimeValue = sm_config.sa_resp_time_n2;
	saClassPortInfo.u3.s.RedirectQP = 1;
	saClassPortInfo.u5.s.TrapHopLimit = 0xff;
	saClassPortInfo.u5.s.TrapQP = 1;

    //
    //	Init Sa Groups table and Set up the default Multicast group if one is set.
    //
    status = sa_McGroupInit();
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "sa_main: can't initialize SA McMember/Groups table lock");
        return 5;
	}
	sa_SetDefBcGrp();

	//
	//	Init SA caching
	//
	status = sa_cache_init();
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "sa_main: can't initialize SA caching");
		return 6;
	}

	return 0;
}


void
sa_main_reader(uint32_t argc, uint8_t ** argv) {
	Status_t	status;
	Mai_t		in_mad;
	Filter_t	filter;
	sa_cntxt_t	*sa_cntxt=NULL;
	uint64_t	now, delta, max_delta;
	int			tries=0, retry=0;
    uint64_t    reqTimeToLive=0;
    SAContextGet_t  cntxGetStatus=0;
    int         numContextBusy=0;

	IB_ENTER("sa_main_reader", 0, 0, 0, 0);

	sa_main_reader_exit = 0;
    
    /*
     *	Create the SubnAdm(*) MAD filter for the SA thread.
     */
	SA_Filter_Init(&filter);
	filter.value.mclass = MAD_CV_SUBN_ADM;
	filter.mask.mclass = 0xff;
	filter.value.method = 0x00;
	filter.mask.method = 0x80;
	filter.mai_filter_check_packet = sa_reader_filter;
	MAI_SET_FILTER_NAME (&filter, "SA Reader");

	if (mai_filter_create(fd_sa, &filter, VFILTER_SHARE) != VSTATUS_OK) {
		IB_LOG_ERROR0("sa_main_reader: can't create SubnAdm(*) filter");
		(void)vs_thread_exit(&sm_threads[SM_THREAD_SA_READER].handle);
	}

    timeMftLastUpdated = 0;
    /* 
     * calculate request time to live on queue
     * ~ 3.2secs for defaults: sa_packetLifetime=18 and sa_respTimeValue=18 
     */
    reqTimeToLive = 4ull * ( (2*(1 << sm_config.sa_packet_lifetime_n2)) + (1 << sm_config.sa_resp_time_n2) ); 
	while (1) {
		status = mai_recv(fd_sa, &in_mad, VTIMER_1S/4);

        if (sa_main_reader_exit == 1){
#ifdef __VXWORKS__
            ESM_LOG_ESMINFO("sa_main_reader: exiting OK.", 0);
#endif
            break;
        }
        /* don't process messages if not master SM or still doing first sweep */
		if ((sm_state != SM_STATE_MASTER) ||
		    (topology_passcount < 1)) {
            continue;
        }

		/* 
         * If the mai layer shuts down we end up in this infinite loop here.
		 * This may happen on initialization
         */
		if( status != VSTATUS_OK ){
            if (status != VSTATUS_TIMEOUT)
                IB_LOG_ERRORRC("sa_main_reader: error on mai_recv rc:", status);
        } else {
            /* 
             * Drop new requests that have been sitting on SA reader queue for too long 
             */
            if (in_mad.intime) {
				/* PR 110586 - On some RHEL 5 systems, we've seen  weird issues with gettimeofday() [used by vs_time_get()]
				 * where once in a while the time difference calculated from successive calls to gettimeofday()
				 * results in a negative value. Due to this, we might actually consider a request stale even if
				 * its not. Work around this by making calls to gettimeofday() till it returns us some
				 * sane values. Just to be extra cautious, bound the retries so that we don't get stuck in the loop.  
				 */
				tries = 0;
				/* Along with negative values also check for unreasonably high values of delta*/
				max_delta = 30*reqTimeToLive;
				do {
					vs_time_get( &now );
					delta = now - in_mad.intime;
					tries++;
					
					if ((now < in_mad.intime) || (delta > max_delta)) {
						vs_thread_sleep(1);
						retry = 1;
					} else {
						retry = 0;
					}	
				} while (retry && tries < 20);

                if (delta > reqTimeToLive) {
					INCREMENT_COUNTER(smCounterSaDroppedRequests);
                    if (smDebugPerf || saDebugPerf) {
                        IB_LOG_INFINI_INFO_FMT( "sa_main_reader",
                               "Dropping stale %s[%s] request from LID[0x%x], TID="FMT_U64"; On queue for %d.%d seconds.", 
                               sa_getMethodText((int)in_mad.base.method), sa_getAidName((int)in_mad.base.aid), in_mad.addrInfo.slid, 
                               in_mad.base.tid, (int)(delta/1000000), (int)((delta - delta/1000000*1000000))/1000);
                    }
                    /* drop the request without returning a response; sender will retry */
                    continue;
                }
            }
            /* 
             * get a context to process request; sa_cntxt can be:
             *   1. NULL if resources are scarce
             *   2. NULL if request is dup of existing request
             *   3. in progress getMulti request context
             *   4. New context for a brand new request 
             */
            cntxGetStatus = sa_cntxt_get( &in_mad, (void *)&sa_cntxt );
            if (cntxGetStatus == ContextAllocated) {
				/* process the new request */
				sa_process_mad( &in_mad, sa_cntxt );
				/* 
				 * This may not necessarily release context based on if someone else has reserved it
				 */
				if(sa_cntxt) sa_cntxt_release( sa_cntxt );
			} else if (cntxGetStatus == ContextExist) {
				INCREMENT_COUNTER(smCounterSaDuplicateRequests);
				/* this is a duplicate request */
				if (saDebugPerf || saDebugRmpp) {
					IB_LOG_INFINI_INFO_FMT( "sa_main_reader",
					       "SA_READER received duplicate %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
					       sa_getMethodText((int)in_mad.base.method), sa_getAidName((int)in_mad.base.aid),in_mad.addrInfo.slid, in_mad.base.tid);
				}
            } else if (cntxGetStatus == ContextNotAvailable) {
				INCREMENT_COUNTER(smCounterSaContextNotAvailable);
                /* we are swamped, return BUSY to caller */
                if (saDebugPerf || saDebugRmpp) { /* log msg before send changes method and lids */
                    IB_LOG_INFINI_INFO_FMT( "sa_main_reader",
                           "NO CONTEXT AVAILABLE, returning MAD_STATUS_BUSY to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!",
                           sa_getMethodText((int)in_mad.base.method), sa_getAidName((int)in_mad.base.aid), in_mad.addrInfo.slid, in_mad.base.tid);
                }
                in_mad.base.status = MAD_STATUS_BUSY;
                sa_send_reply( &in_mad, sa_cntxt );
                if ((++numContextBusy % sa_max_cntxt) == 0) {
                    IB_LOG_INFINI_INFO_FMT( "sa_main_reader",
                           "Had to drop %d SA requests since start due to no available contexts",
                           numContextBusy);
                }
            } else if (cntxGetStatus == ContextExistGetMulti) {
                /* continue processing the getMulti request */
                sa_process_getmulti( &in_mad, sa_cntxt );
                if(sa_cntxt) sa_cntxt_release( sa_cntxt );
            } else {
                IB_LOG_WARN("sa_main_reader: Invalid sa_cntxt_get return code:", cntxGetStatus);
            }
        }

        /* 
         * signal sm_top to reprogram the MFTs
         * Wait one second to allow mcmember requests to accumulate before asking
         */
        vs_time_get( &now );
        if (sa_mft_reprog && timeMftLastUpdated == 0) {
            timeMftLastUpdated = now;
        } else if (sa_mft_reprog && (now - timeMftLastUpdated) > VTIMER_1S) {
            topology_wakeup_time = 0ull;
            AtomicWrite(&sm_McGroups_Need_Prog, 1); /* tells Topoloy thread that MFT reprogramming is needed */

            sm_trigger_sweep(SM_SWEEP_REASON_MCMEMBER);
            /* clear the indicators */
            timeMftLastUpdated = 0;
            sa_mft_reprog = 0;
        }
	}
    /* cleanup before exit, but allow some time for the other threads to flush out first */
    (void)vs_thread_sleep(VTIMER_1S);     
    (void)sa_SubscriberDelete();
    (void)sa_ServiceRecDelete();
    (void)sa_McGroupDelete();
    sm_multicast_destroy_mlid_list();
	if (mai_filter_delete(fd_sa, &filter, VFILTER_SHARE) != VSTATUS_OK) {
		IB_LOG_ERROR0("sa_main_reader: can't delete SubnAdm(*) filter");
	}
	//IB_LOG_INFINI_INFO0("sa_main_reader thread: Exiting OK");
}


void
sa_main_writer(uint32_t argc, uint8_t ** argv) {
	Status_t	status;
	Mai_t		in_mad;
	Filter_t	filter;
	sa_cntxt_t	*sa_cntxt;
	uint64_t	now, srLastAged=0, cacheLastCleaned=0;
    uint32_t    records=0;

	IB_ENTER("sa_main_writer", 0, 0, 0, 0);

	sa_main_writer_exit = 0;
    
    //
    //	Create the SubnAdm(*) MAD filter for the SA thread.
    //
	SA_Filter_Init(&filter);
	filter.value.mclass = MAD_CV_SUBN_ADM;
	filter.mask.mclass = 0xff;
	filter.value.method = 0x00;
	filter.mask.method = 0x80;
	filter.mai_filter_check_packet = sa_writer_filter;
	MAI_SET_FILTER_NAME (&filter, "SA Writer");

	if (mai_filter_create(fd_sa_writer, &filter, VFILTER_SHARE) != VSTATUS_OK) {
		IB_LOG_ERROR0("esm_saw: can't create SubnAdm(*) filter");
		(void)vs_thread_exit(&sm_threads[SM_THREAD_SA_WRITER].handle);
	}

	while (1) {
		status = mai_recv(fd_sa_writer, &in_mad, VTIMER_1S/4);
        if (status != VSTATUS_OK && status != VSTATUS_TIMEOUT) {
            IB_LOG_ERRORRC("sa_main_writer: error on mai_recv rc:", status);
            vs_thread_sleep(VTIMER_1S/10);
        }

        if (sa_main_writer_exit == 1){
#ifdef __VXWORKS__
            ESM_LOG_ESMINFO("SA Writer Task exiting OK.", 0);
#endif
            break;
        }
        /* don't process messages if not master SM or still doing first sweep */
		if (sm_state != SM_STATE_MASTER || topology_passcount < 1) {
            continue;
        }
		/* 
         * process the rmpp ack and send out the next set of segments
         */
        if (status == VSTATUS_OK) {
            /* locate and process in flight rmpp request */
            sa_cntxt = sa_cntxt_find( &in_mad );
            if (sa_cntxt) {
                sa_process_inflight_rmpp_request( &in_mad, sa_cntxt );
                /*
                 * This may not necessarily release context
                 * based on if someone else has reserved it
                 */
                sa_cntxt_release( sa_cntxt );
            } else {
				INCREMENT_COUNTER(smCounterSaDeadRmppPacket);
                if (saDebugRmpp) {
                    IB_LOG_INFINI_INFO_FMT( "sa_main_writer", 
                           "dropping %s[%s] RMPP packet from LID[0x%x], TID ["FMT_U64"] already completed/aborted",
                           sa_getMethodText((int)in_mad.base.method), sa_getAidName(in_mad.base.aid), 
                           in_mad.addrInfo.slid, in_mad.base.tid);
                }
            }
        }

        /* age contexts if more than 1 second since last time */
        vs_time_get( &now );
        if ((now - timeLastAged) > (VTIMER_1S)) {
            (void) sa_cntxt_age();
        }

        /* age the service records */
        if ((now - srLastAged) > 5*VTIMER_1S) {
            srLastAged = now;
            if ((status = sa_ServiceRecord_Age(&records)) != VSTATUS_OK) {
                IB_LOG_ERRORRC("sa_main_writer: failed to age service records, rc:", status);
            } else if (records) {
                if (smDebugPerf) IB_LOG_INFINI_INFO("sa_main_writer: Number of service records aged out was", records);
            }
        }
		
		/* clean the SA cache */
		if ((now - cacheLastCleaned) > SA_CACHE_CLEAN_INTERVAL) {
			cacheLastCleaned = now;
			(void)vs_lock(&saCache.lock);
			sa_cache_clean();
			(void)vs_unlock(&saCache.lock);
		}
	}
    /* clean up cache before exit */
    sa_cache_clean();
    (void)vs_lock_delete(&saCache.lock);
	//IB_LOG_INFINI_INFO0("sa_main_writer thread: Exiting OK");
} // SA_MAIN_WRITER

/* Added function for vxWorks to exit this task, while we could just do a task delete,
   its possible that the task could be in a critical region at the time of deletion
   This is now used by HSM also, to cleanly exit the threads.
 */
void 
sa_main_kill(void){
	sa_main_reader_exit = 1;
	sa_main_writer_exit = 1;
}

// "free" function for SA Contexts that contain allocated (non-cached) data
//
static Status_t
sa_cntxt_free_data(sa_cntxt_t *cntxt)
{
	IB_ENTER( "sa_cntxt_retire", cntxt, 0, 0, 0 );
	
	if (cntxt->data)
		vs_pool_free(&sm_pool, cntxt->data);
	
	IB_EXIT( "sa_cntxt_retire", VSTATUS_OK);
	return VSTATUS_OK;
}

static void
sa_cntxt_retire( sa_cntxt_t* lcl_cntxt )
{
	IB_ENTER( "sa_cntxt_retire", lcl_cntxt, 0, 0, 0 );

	// release getMulti request buffer if allocated
	if( lcl_cntxt->reqData ) {
		vs_pool_free( &sm_pool, lcl_cntxt->reqData );
	}
	// If data was privately allocated, free it
	if (lcl_cntxt->freeDataFunc) {
		(void)lcl_cntxt->freeDataFunc(lcl_cntxt);
	}

	// Reset all fields
	lcl_cntxt->ref = 0 ;
    lcl_cntxt->isDS = 0;
    lcl_cntxt->reqDataLen = 0;
	lcl_cntxt->reqData = NULL ;
	lcl_cntxt->data = NULL ;
	lcl_cntxt->len = 0 ;
	lcl_cntxt->lid = 0 ;
	lcl_cntxt->tid = 0 ;
	lcl_cntxt->hashed = 0 ;
	lcl_cntxt->cache = NULL;
	lcl_cntxt->freeDataFunc = NULL;

	sa_cntxt_insert_head( sa_cntxt_free_list, lcl_cntxt );
	INCR_SA_CNTXT_NFREE();
    DECR_SA_CNTXT_NALLOC();

	IB_EXIT( "sa_cntxt_retire", 0 );
}

/*
 * safely release context while already under lock protection
 */
static void cntxt_release( sa_cntxt_t* sa_cntxt )
{
	int         bucket;

    if( sa_cntxt->ref == 0 ) {
        IB_LOG_INFINI_INFO0("cntxt_release: reference count is already zero");
        return;   /* context already retired */
    } else {
        --sa_cntxt->ref;
        if( sa_cntxt->ref == 0 ) {
            // This context needs to be removed from hash
            if( sa_cntxt->hashed ) {
                bucket = sa_cntxt->lid % SA_CNTXT_HASH_TABLE_DEPTH;
                sa_cntxt_delete_entry( sa_hash[ bucket ], sa_cntxt );
            }
            sa_cntxt->prev = sa_cntxt->next = NULL ;
            sa_cntxt_retire( sa_cntxt );
        }
    }
}

/*
 * Age context and do resends for those that timed out
 */
void sa_cntxt_age(void)
{
	sa_cntxt_t* sa_cntxt = NULL ;
	sa_cntxt_t*	tout_cntxt ;
    int i;
	Status_t	status;

    if ((status = vs_lock(&sa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("sa_cntxt_age: Failed to lock SA context rc:", status);
    } else {
        vs_time_get( &timeLastAged );
        for (i=0; i<SA_CNTXT_HASH_TABLE_DEPTH; i++) {
            sa_cntxt = sa_hash[ i ];
            while( sa_cntxt ) {
                // Iterate before the pointers are destroyed ;
                tout_cntxt = sa_cntxt ;
                sa_cntxt = sa_cntxt->next ;

                // Though list is sorted, it does not hurt to 
                // do a simple comparison
                if( timeLastAged - tout_cntxt->tstamp > tout_cntxt->RespTimeout) { // used to be VTIMER_1S
                    // Timeout this entry
                    sa_cntxt_delete_entry( sa_hash[ i ], tout_cntxt );
                    sa_cntxt_insert_head( sa_hash[ i ], tout_cntxt );
                    // Touch the entry
                    tout_cntxt->tstamp = timeLastAged ;
                    // Call timeout
                    tout_cntxt->sendFd = fd_sa_writer;       // use sa writer mai handle for restransmits
                    if (tout_cntxt->method == SA_CM_GETMULTI && tout_cntxt->reqInProg) {
                        // resend the getMulti request ACK
                        sa_getMulti_resend_ack(tout_cntxt);
                        /* if need to release the context.  Call local safe release */
                        if( tout_cntxt->retries > sm_config.max_retries ) cntxt_release(sa_cntxt);
                    } else {
                        // resend the reply
                        sa_send_reply( NULL, tout_cntxt );
                        if( tout_cntxt->retries > sm_config.max_retries ) {
                            /* need to release the context.  Call local safe release */
                            cntxt_release(tout_cntxt);
                        }
                    }
                }
            }
        }
        if ((status = vs_unlock(&sa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("cs_cntxt_age: Failed to unlock SA context rc:", status);
        }
    }
}

//
// find context entry matching input mad
//
sa_cntxt_t *sa_cntxt_find( Mai_t* mad ) {
	uint64_t	now ;
	int 		bucket;
	Status_t	status;
	sa_cntxt_t* sa_cntxt;
	sa_cntxt_t*	req_cntxt = NULL;

    if ((status = vs_lock(&sa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("sa_cntxt_find: Failed to lock SA context rc:", status);
    } else {
        vs_time_get( &now );
        // Search the hash table for the context 	
        bucket = mad->addrInfo.slid % SA_CNTXT_HASH_TABLE_DEPTH ;
        sa_cntxt = sa_hash[ bucket ];
        while( sa_cntxt ) {
            if( sa_cntxt->lid == mad->addrInfo.slid  && sa_cntxt->tid == mad->base.tid ) {
                req_cntxt = sa_cntxt ;
                sa_cntxt = sa_cntxt->next ;
                req_cntxt->tstamp = now ;
                break ;
            } else {
                sa_cntxt = sa_cntxt->next;
            }
        }
        // Table is sorted with respect to timeout
        if( req_cntxt ) {
            // Touch current context
            req_cntxt->tstamp = now ;
            sa_cntxt_delete_entry( sa_hash[ bucket ], req_cntxt );
            sa_cntxt_insert_head( sa_hash[ bucket ], req_cntxt );
            // A get on an existing context reserves it
            req_cntxt->ref ++ ;
        }
        if ((status = vs_unlock(&sa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("sa_cntxt_find: Failed to unlock SA context rc:", status);
        }
    }
	IB_EXIT("sa_cntxt_find", req_cntxt );
    return req_cntxt;
} // end cntxt_find

/*
 * Very simple hashing implemented. This function is modular and can be 
 * changed to use any algorithm, if hashing turns out to be bad.
 * Returns new context for new requests, existing context for in progress 
 * getMulti requests and NULL for duplicate requests
 */
SAContextGet_t
sa_cntxt_get( Mai_t* mad, void **context)
{
	uint64_t	now ;
	int 		bucket;
	Status_t	status;
	sa_cntxt_t* sa_cntxt;
    SAContextGet_t getStatus=0;
	sa_cntxt_t*	req_cntxt = NULL;

	IB_ENTER( "sa_cntxt_get", mad, 0, 0, 0 );

    if ((status = vs_lock(&sa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("sa_cntxt_get: Failed to lock SA context rc:", status);
    } else {
        vs_time_get( &now );
        // Search the hash table for the context 	
        bucket = mad->addrInfo.slid % SA_CNTXT_HASH_TABLE_DEPTH ;
        sa_cntxt = sa_hash[ bucket ];
        while( sa_cntxt ) {
            if( sa_cntxt->lid == mad->addrInfo.slid  &&
                    sa_cntxt->tid == mad->base.tid ) {
                req_cntxt = sa_cntxt ;
                break ;
            } else {
                sa_cntxt = sa_cntxt->next;
            }
        }
		if( req_cntxt ) {
			if ( req_cntxt->method == SA_CM_GETMULTI && req_cntxt->reqInProg ) {
				/* In progress getMulti request. Touch and reserve it. */
				getStatus = ContextExistGetMulti;
				req_cntxt->tstamp = now ;
				sa_cntxt_delete_entry( sa_hash[ bucket ], req_cntxt );
				sa_cntxt_insert_head( sa_hash[ bucket ], req_cntxt );
				/* A get on an existing context reserves it */
				req_cntxt->ref ++ ;
			} else {
				/* dup of an existing request */
				getStatus = ContextExist;
				req_cntxt = NULL;
			}
		} else {
			/* Allocate a new context and set appropriate status */
			req_cntxt = sa_cntxt_free_list ;
			if( req_cntxt ) {
				getStatus = ContextAllocated;
				sa_cntxt_delete_entry( sa_cntxt_free_list, req_cntxt );
				req_cntxt->ref = 1;  /* set ref count to 1 */
				INCR_SA_CNTXT_NALLOC();
				DECR_SA_CNTXT_NFREE();
				req_cntxt->lid = mad->addrInfo.slid ;
				req_cntxt->tid = mad->base.tid ;
				req_cntxt->method = mad->base.method ;
				req_cntxt->tstamp = now;
			} else {
				/* out of context */
				getStatus = ContextNotAvailable;
			}
		}
		if ((status = vs_unlock(&sa_cntxt_lock)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("sa_cntxt_get: Failed to unlock SA context rc:", status);
		}
	}
    /* return new context or existing getmulti context or NULL */
    *context = req_cntxt;

	IB_EXIT("sa_cntxt_get", getStatus );
	return getStatus;
}

/*
 * reserving context inserts it into the hash table if !hashed
 * and increments reference count
*/
Status_t
sa_cntxt_reserve( sa_cntxt_t* sa_cntxt )
{
	int         bucket;
	Status_t	status;

	IB_ENTER( "sa_cntxt_reserve", sa_cntxt, 0, 0, 0 );

    if ((status = vs_lock(&sa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("sa_cntxt_reserve: Failed to lock SA context rc:", status);
    } else {
        sa_cntxt->ref ++;
        if( sa_cntxt->hashed == 0 ) {
            // This context needs to be inserted into the hash table
            bucket = sa_cntxt->lid % SA_CNTXT_HASH_TABLE_DEPTH;
            sa_cntxt->hashed = 1 ;
            vs_time_get( &sa_cntxt->tstamp );
            sa_cntxt_insert_head( sa_hash[ bucket ], sa_cntxt );
        }
        if ((status = vs_unlock(&sa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("sa_cntxt_reserve: Failed to unlock SA context rc:", status);
        }
    }

	IB_EXIT( "sa_cntxt_reserve", VSTATUS_OK );
	return VSTATUS_OK ;
}

Status_t
sa_cntxt_release( sa_cntxt_t* sa_cntxt )
{
	int         bucket;
	Status_t	status;

	IB_ENTER( "sa_cntxt_release", sa_cntxt, 0, 0, 0 );

    if (!sa_cntxt) {
        IB_LOG_ERROR0("sa_cntxt_release: SA context is NULL!!!");
        return VSTATUS_OK ;
    }
    if ((status = vs_lock(&sa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("sa_cntxt_release: Failed to lock SA context rc:", status);
    } else {
        if( sa_cntxt->ref == 0 ) {
            IB_LOG_INFINI_INFO0("sa_cntxt_release: reference count is already zero");
        } else {
            --sa_cntxt->ref;
            if( sa_cntxt->ref == 0 ) {

                // This context needs to be removed from hash
                if( sa_cntxt->hashed ) {
                    bucket = sa_cntxt->lid % SA_CNTXT_HASH_TABLE_DEPTH;
                    sa_cntxt_delete_entry( sa_hash[ bucket ], sa_cntxt );
                }

                sa_cntxt->prev = sa_cntxt->next = NULL ;
                sa_cntxt_retire( sa_cntxt );

            }
        }
        if ((status = vs_unlock(&sa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("sa_cntxt_release: Failed to unlock SA context rc:", status);
        }
    }

	IB_EXIT( "sa_cntxt_release", VSTATUS_OK );
	return VSTATUS_OK ;
}

Status_t
sa_cntxt_data( sa_cntxt_t* sa_cntxt, void* buf, uint32_t len )
{
	Status_t	status ;

	IB_ENTER( "sa_cntxt_data", sa_cntxt, buf, len, 0 );

	status = VSTATUS_OK ;

	if (!buf || !len) {
		sa_cntxt->data = NULL;
        sa_cntxt->len = 0;
		sa_cntxt->freeDataFunc = NULL;
		goto done;
	}

	status = vs_pool_alloc(&sm_pool, len, (void*)&sa_cntxt->data);
	if (status == VSTATUS_OK) {
		sa_cntxt->len = len;
		memcpy(sa_cntxt->data, buf, len);
		sa_cntxt->freeDataFunc = sa_cntxt_free_data;
	} else {
        sa_cntxt->len = 0;
		sa_cntxt->data = NULL; /* If data is NULL, sa_send_reply will send error response to caller */
		sa_cntxt->freeDataFunc = NULL;
	}

done:
	IB_EXIT("sa_cntxt_data", status);
	return status ;
}

// Wraps setup of SA Context when caching is available.  Simply delegates to
// sa_cntxt_data if the cache is not valid.
//
Status_t
sa_cntxt_data_cached(sa_cntxt_t* sa_cntxt, void* buf, uint32_t len, SACacheEntry_t *cache)
{
	IB_ENTER("sa_cntxt_data_cached", sa_cntxt, buf, len, cache);

	if (!cache || !cache->valid) {
		// normal non-cached transfer
		sa_cntxt_data(sa_cntxt, buf, len);
	} else if (cache->transient) {
		// transient; the cache was built on the fly and should be copied. the
		// context can take care of deallocation
		sa_cntxt_data(sa_cntxt, cache->data, cache->len);
	} else {
		// cached transfer; just point the SA context to the cache buffer
		sa_cntxt->data = (char*)cache->data;
		sa_cntxt->len = cache->len;
		sa_cntxt->cache = cache;
		// use the cache-enabled free function
		sa_cntxt->freeDataFunc = sa_cache_cntxt_free;
	}
	
	IB_EXIT("sa_cntxt_data_cached", VSTATUS_OK);
	return VSTATUS_OK;
}

/*
 * clear the contents of the context pool
 */
void sa_cntxt_clear(void) {
    int i;
	Status_t status;

    if ((status = vs_lock(&sa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("sa_cntxt_clear: Failed to lock SA context, rc:", status);
    } else {
    	memset( sa_hash, 0, sizeof( sa_hash ));
    	memset( sa_cntxt_pool, 0, sizeof( sa_cntxt_t ) * sa_max_cntxt);
    	sa_cntxt_free_list = NULL ;
    	for( i = 0 ; i < sa_max_cntxt ; ++i ) {
    		sa_cntxt_insert_head( sa_cntxt_free_list, &sa_cntxt_pool[i] );
    	}
        if ((status = vs_unlock(&sa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("sa_cntxt_clear: Failed to unlock SA contex, rc:", status);
        }
    }
}

void setPacketLifetime(uint8_t plt) {
    if (plt > 0 && plt < 20) {
        sm_config.sa_packet_lifetime_n2 = plt;
        printf("packetLifetime set to %d; host will get on next path record request \n", (int)sm_config.sa_packet_lifetime_n2);
    } else {
        printf("sa_packetLifetime should be 1-19;  current value is %d \n", (int)sm_config.sa_packet_lifetime_n2);
    }
}

void setDynamicPlt(uint8_t h1, uint8_t h2, uint8_t h3, uint8_t h4, uint8_t h5) {
    if (h1 == 0) {
        printf("Turning off dynamic packetLifetime calculations \n");
        sa_dynamicPlt[0]=0;
    } else {
        sa_dynamicPlt[0]=1;
        sa_dynamicPlt[1]=h1;
        sa_dynamicPlt[2]=h2;
        sa_dynamicPlt[3]=h3;
        sa_dynamicPlt[4]=h4;
        sa_dynamicPlt[5]=h5;
        sa_dynamicPlt[6]=h5;
        sa_dynamicPlt[7]=h5;
        sa_dynamicPlt[8]=17;
        sa_dynamicPlt[9]=17;
        printf("Dynamic PLT ON using values: 1 hop =%d, 2 hops=%d, 3 hops=%d, 4 hops=%d \n",
               h1,  h2,  h3,  h4);
        printf("Dynamic PLT ON using values: 5 hops=%d, 6 hops=%d, 7 hops=%d, 7+hops=%d \n", 
               h5, sa_dynamicPlt[6], sa_dynamicPlt[7], sa_dynamicPlt[9]);
    }
}

void setRespTime(uint8_t respTime) {
    if (respTime > 0 && respTime <= 20) {
        saClassPortInfo.u1.s.RespTimeValue = respTime;
        printf("SA ClassPortInfo RespTimeValue set to %d\n", respTime);
    } else {
        printf("SA ClassPortInfo RespTimeValue must be from 1 to 20!\n");
    }
}

uint32_t sa_get_saPerfDebug(void)
{
	return saDebugPerf;
}
 
void saPerfDebugOn(void)
{
    saDebugPerf = 1;
	IB_LOG_INFINI_INFO0("saPerfDebugOn: Turning ON SaPerfDebug");
}

void saPerfDebugOff(void)
{
    saDebugPerf = 0;
	IB_LOG_INFINI_INFO0("saPerfDebugOff: Turning OFF SaPerfDebug");
}

void saPerfDebugToggle(void)
{
	if (saDebugPerf) {
		saPerfDebugOff();
	} else {
		saPerfDebugOn();
	}
}

void saRmppDebugOn(void)
{
    saDebugRmpp = 1;
	IB_LOG_INFINI_INFO0("saRmppDebugOn: Turning ON SaRmppDebug");
}

void saRmppDebugOff(void)
{
    saDebugRmpp = 0;
	IB_LOG_INFINI_INFO0("saRmppDebugOff: Turning OFF SaRmppDebug");
}

void saRmppDebugToggle(void)
{
	if (saDebugRmpp) {
		saRmppDebugOff();
	} else {
		saRmppDebugOn();
	}
}


#ifdef __VXWORKS__
void showSaParms() {
    printf("SA ClassPortInfo RespTimeValue set to %d\n", saClassPortInfo.u1.s.RespTimeValue);
    printf("SA packetLifetime set to %d\n", (int)sm_config.sa_packet_lifetime_n2);
    if (sa_dynamicPlt[0] == 0) {
        printf("Dynamic packetLifetime calculations are off\n");
    } else {
        printf("Dynamic PLT ON using values: 1 hop =%d, 2 hops=%d, 3 hops=%d, 4 hops=%d \n",
               sa_dynamicPlt[1],  sa_dynamicPlt[2],  sa_dynamicPlt[3],  sa_dynamicPlt[4]);
        printf("Dynamic PLT ON using values: 5 hops=%d, 6 hops=%d, 7 hops=%d, 7+hops=%d \n", 
               sa_dynamicPlt[5], sa_dynamicPlt[6], sa_dynamicPlt[7], sa_dynamicPlt[9]);
    }
}

int saSubscriberSize() {
	size_t size = sizeof(SubscriberKey_t) + sizeof(STL_INFORM_INFO_RECORD);
	sysPrintf("An SA Subscriber is %d bytes\n", size);
	return size;
}

int saServiceRecordSize() {
	size_t size = sizeof(OpaServiceRecord_t);
	sysPrintf("An SA Service Record is %d bytes\n", size);
	return size;
}

int saMcGroupSize() {
    size_t size = sizeof(McGroup_t) + 16;
    sysPrintf("An SA Multicast Group is %d bytes\n", size);
    return size;
}

int saMcMemberSize() {
    size_t size = sizeof(McMember_t) + 16;
    sysPrintf("An SA Multicast Member is %d bytes\n", size);
    return size;
}

int saNodeRecordSize() {
    size_t size = sizeof(STL_NODE_RECORD) + Calculate_Padding(sizeof(STL_NODE_RECORD));
    sysPrintf("An SA Node Record is %d bytes\n", size);
    return size;
}

int saPathRecordSize() {
    size_t size = sizeof(IB_PATH_RECORD) + Calculate_Padding(sizeof(IB_PATH_RECORD));
    sysPrintf("An SA Path Record is %d bytes\n", size);
    return size;
}

int saMaxResponseSize() {
	return sa_data_length;
}

#endif

