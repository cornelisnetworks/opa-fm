/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_Trap.c								     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the Trap type.						     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_Trap								     //
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
#include "cs_csm_log.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "cs_context.h"
#include "cs_queue.h"
#include "iba/stl_sa.h"
#include "iba/stl_sm.h"

static Status_t	sa_Trap_Forward(SubscriberKeyp, STL_NOTICE *);

extern	uint64_t	topology_wakeup_time;
extern  generic_cntxt_t sm_notice_cntxt;
extern  cs_Queue_ptr sm_trap_forward_queue;
uint32_t	saTrapCount = 1;// JSY - really need bitmap of unused records

/* lookup LID and format info about that trap issuer as best as possible
 * desc needs to be a buffer of at least 110 characters
 * This gets and releases the old_topology_lock as part of lookup of the LID
 */
static void sm_get_lid_info(char *desc, uint16 lid)
{
	Node_t	*nodep;
	Port_t	*portp;

	if (topology_passcount == 0) {
		sprintf(desc, "LID 0x%x", lid);
	} else {
		(void)vs_rdlock(&old_topology_lock);
		portp = sm_find_node_and_port_lid(&old_topology, lid, &nodep);
		if (nodep && sm_valid_port(portp)) {
			if (nodep->nodeInfo.NodeType == STL_NODE_SW)
				sprintf(desc, "%.*s Guid "FMT_U64" LID 0x%x",
					(int)sizeof(nodep->nodeDesc.NodeString), nodep->nodeDesc.NodeString,
					nodep->nodeInfo.NodeGUID, lid);
			else
				sprintf(desc, "%.*s Guid "FMT_U64" LID 0x%x Port %u",
					(int)sizeof(nodep->nodeDesc.NodeString), nodep->nodeDesc.NodeString,
					nodep->nodeInfo.NodeGUID, lid, (unsigned)portp->index);
		} else {
			sprintf(desc, "LID 0x%x", lid);
		}
		(void)vs_rwunlock(&old_topology_lock);
	}
}
			
// indicates a sweep is needed
void sm_discovery_needed(const char* reason, int lid)
{
	if (!smFabricDiscoveryNeeded) {
		if (!lid) {
			IB_LOG_INFINI_INFO_FMT( "sa_Trap", "SM Sweep scheduled: %s", reason);
		} else {
			char desc[110];

			sm_get_lid_info(desc, lid);
			IB_LOG_INFINI_INFO_FMT("sa_Trap", "SM Sweep scheduled: %s (from %s)", reason, desc);
		}
	}
	smFabricDiscoveryNeeded = 1;

	// Only Trap's call this function.
	setResweepReason(SM_SWEEP_REASON_TRAP_EVENT);
}

//
// helper function facilitating the queueing of trap forward request for reliable 
// transmission by the sm_async thread.  Callers are required to call this function 
// with a formated notice.
//
Status_t sm_sa_forward_trap (STL_NOTICE * noticep) {
    Status_t    status=VSTATUS_OK;
    STL_NOTICE    *noticeToQ=NULL;

    if ((status = vs_pool_alloc(&sm_pool, sizeof(STL_NOTICE), (void *)&noticeToQ)) != VSTATUS_OK) {
		IB_FATAL_ERROR("sm_sa_forward_trap: unable to allocate space for notice");
    } else {
        memcpy((void *)noticeToQ, (void *)noticep, sizeof(STL_NOTICE));
        // queue the request on the SM-SA trap forward request queue
        if ((status = cs_queue_Enqueue((cs_QueueElement_ptr)noticeToQ, sm_trap_forward_queue)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("sm_sa_forward_trap: unable to queue notice for transmission, rc:", status);
            vs_pool_free(&sm_pool, noticeToQ);
        } else {
            // successfully queued trap forward request on queue
            //IB_LOG_INFINI_INFO("sm_sa_forward_trap: successfully queued trap request on queue, trap number=", noticeToQ->Attributes.Generic.TrapNumber);
        }
    }
    return status;
}

//
// determine number of notices that would be sent out
//
int sm_sa_getNoticeCount (STL_NOTICE * noticep)
{
	int noticeCount=0;
	STL_INFORM_INFO_RECORD *	iRecordp = NULL;
    CS_HashTableItr_t itr;

	IB_ENTER("sa_notice_count", 0, 0, 0, 0);
	
	(void)vs_lock(&saSubscribers.subsLock);
    if (cs_hashtable_count(saSubscribers.subsMap) > 0)
    {
        cs_hashtable_iterator(saSubscribers.subsMap, &itr);
        do {
            iRecordp = cs_hashtable_iterator_value(&itr);
			if (iRecordp->InformInfoData.IsGeneric == noticep->Attributes.Generic.u.s.IsGeneric &&
                (iRecordp->InformInfoData.Type == TRAP_ALL || iRecordp->InformInfoData.Type == noticep->Attributes.Generic.u.s.Type) &&
                (iRecordp->InformInfoData.u.Generic.TrapNumber == TRAP_ALL || iRecordp->InformInfoData.u.Generic.TrapNumber == noticep->Attributes.Generic.TrapNumber))
                ++noticeCount;
        } while (cs_hashtable_iterator_advance(&itr));
    }
	(void)vs_unlock(&saSubscribers.subsLock);

	IB_EXIT("sa_notice_count", noticeCount);
	return(noticeCount);
}


/*
 * Used for Traps that come from the SM itself, like Mgroup create and destroy
 * In this case, the caller creates the notice structure
 */
Status_t
sm_sa_forwardNotice(STL_NOTICE * noticep)
{
	STL_INFORM_INFO_RECORD * iRecordp = NULL;
    SubscriberKeyp subsKeyp = NULL;
    CS_HashTableItr_t itr;

	IB_ENTER("sm_sa_forwardNotice", 0, 0, 0, 0);
	
	(void)vs_lock(&saSubscribers.subsLock);
    if (cs_hashtable_count(saSubscribers.subsMap) > 0)
    {
        cs_hashtable_iterator(saSubscribers.subsMap, &itr);
        do {
            subsKeyp = cs_hashtable_iterator_key(&itr);
            iRecordp = cs_hashtable_iterator_value(&itr);
			if (iRecordp->InformInfoData.IsGeneric != noticep->Attributes.Generic.u.s.IsGeneric) {
				continue;
			}
			if (iRecordp->InformInfoData.Type != TRAP_ALL && iRecordp->InformInfoData.Type != noticep->Attributes.Generic.u.s.Type) {
				continue;
			}
			if (iRecordp->InformInfoData.u.Generic.TrapNumber != TRAP_ALL && iRecordp->InformInfoData.u.Generic.TrapNumber != noticep->Attributes.Generic.TrapNumber) {
				continue;
			}
			if (iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != NODE_TYPE_ALL &&
                iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != 0 &&
                iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != noticep->Attributes.Generic.u.s.ProducerType) {
				/* 
                 * work around host bug that used channel adapter for gid in/out service
				 * and multicast group create destroy
				 */
				if (noticep->Attributes.Generic.TrapNumber >= MAD_SMT_PORT_UP && noticep->Attributes.Generic.TrapNumber <= MAD_SMT_MCAST_GRP_DELETED){
					if (iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != NOTICE_PRODUCERTYPE_CA) {
						continue;
					}
				} else {
					continue;
				}
			}
			sa_Trap_Forward(subsKeyp, noticep);
        } while (cs_hashtable_iterator_advance(&itr));
    }
	(void)vs_unlock(&saSubscribers.subsLock);

	IB_EXIT("sm_sa_forwardNotice", VSTATUS_OK);
	return(VSTATUS_OK);
}

// Handles throttling of log messages related to traps and  auto-disabling of ports
// by tracking trap counts. Calling this will increment the counts for the specified
// port by one and disable if necessary..
//
// NOTE: Needs to be called from under an old_topology lock.
//
static void
sa_updateTrapCountForPort(Lid_t lid, uint32_t portIndex, int disable)
{
	Node_t *nodep, *swnodep;
	Port_t *portp, *ext_portp, *swportp;
	uint64_t timenow, timedelta, trap_interval;
	
	IB_ENTER("sa_updateTrapCountForPort", lid, portIndex, 0, 0);
	
	// get node and LID port
	portp = sm_find_node_and_port_lid(&old_topology, lid, &nodep);
	if (!sm_valid_port(portp)) {
		IB_LOG_WARNX("sa_updateTrapCountForPort: failed to find node in topology; LID:", lid);
		IB_EXIT("sa_updateTrapCountForPort", 0);
		return;
	}
	
	// if switch, grab external port to track stats on
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		ext_portp = sm_get_port(nodep, portIndex);
		if (!sm_valid_port(ext_portp)) {
			IB_LOG_WARN_FMT("sa_updateTrapCountForPort", 
					"Failed to find port in topology; lid: 0x%x port number: %d", lid, portIndex);
			IB_EXIT("sa_updateTrapCountForPort", 0);
			return;
		}
	} else {
		ext_portp = portp;
	}
	
	(void)vs_time_get(&timenow);

	if (sm_config.trap_log_suppress_trigger_interval && ext_portp->portData->lastTrapTime) {
		trap_interval = timenow - ext_portp->portData->lastTrapTime;
		if (trap_interval < sm_config.trap_log_suppress_trigger_interval * VTIMER_1S) {
				if (ext_portp->portData->suppressTrapLog) {
					ext_portp->portData->logSuppressedTrapCount++;
				}
				else {
				/* this trap has already been logged, so only set the flag and not
				 * increment the counter
				 */
					ext_portp->portData->suppressTrapLog = 1;	
				}
		} else {
			ext_portp->portData->suppressTrapLog = 0;
			portp->portData->logTrapSummaryThreshold = SM_TRAP_LOG_SUMMARY_THRESHOLD_START;
		}
	}

	ext_portp->portData->lastTrapTime = timenow;

	// if sm_trapThreshold is disabled, return.
	// Otherwise check if the port needs to be disabled
	if ((sm_trapThreshold == 0) || (disable == 0)) {
		IB_EXIT("sa_updateTrapCountForPort", 0);
		return;
	}

	timedelta = timenow - ext_portp->portData->trapWindowStartTime;
	
	if (timedelta < sm_trapThresholdWindow) {
		// still in the window, update the count
		ext_portp->portData->trapWindowCount++;
	}

	if ((ext_portp->portData->trapWindowCount >= sm_trapThreshold_minCount) &&
		(timedelta < sm_trapThresholdWindow * 2)) {
		// Got minimum number of traps while within the window or close to it.
		// Now check the trap rate and disable the link if necessary
		if ((60ull * ext_portp->portData->trapWindowCount * VTIMER_1S) / timedelta >= sm_trapThreshold) {
			// exceeded threshold; disable the port
			if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				// node is a swtich; disable the external port
				swnodep = nodep;
				swportp = ext_portp;
				IB_LOG_INFINI_INFO_FMT( "sa_updateTrapCountForPort", 
				       "Trap threshold exceeded; disabling port: Node='%s', GUID="FMT_U64", PortIndex=%d",
				       sm_nodeDescString(swnodep), swnodep->nodeInfo.NodeGUID, swportp->index);
				if (ext_portp->portData->suppressTrapLog && ext_portp->portData->logSuppressedTrapCount)
					IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
									 ext_portp->portData->logSuppressedTrapCount);

			} else {
				// node is not a switch; find the switch port the node is
				// connected to and disable that instead
				swnodep = sm_find_node(&old_topology, ext_portp->nodeno);
				if (swnodep == NULL || swnodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
					IB_LOG_WARN("sa_updateTrapCountForPort: Failed to find neighbor switch in topology; NodeIndex:", ext_portp->nodeno);
					IB_EXIT("sa_updateTrapCountForPort", 0);
					return;
				}
				swportp = sm_get_port(swnodep, ext_portp->portno);
				if (!sm_valid_port(swportp)) {
					IB_LOG_WARN("sa_updateTrapCountForPort: Failed to find neighbor switch's port in topology; PortIndex:", ext_portp->portno);
					IB_EXIT("sa_updateTrapCountForPort", 0);
					return;
				}
				IB_LOG_INFINI_INFO_FMT( "sa_updateTrapCountForPort", 
				       "Trap threshold exceeded for: Node='%s', GUID="FMT_U64", PortIndex=%d;"
				       "Disabling neighboring switch port: Node='%s', GUID="FMT_U64", PortIndex=%d",
				       sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, ext_portp->index,
				       sm_nodeDescString(swnodep), swnodep->nodeInfo.NodeGUID, swportp->index);
				if (ext_portp->portData->suppressTrapLog && ext_portp->portData->logSuppressedTrapCount)
					IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
									 ext_portp->portData->logSuppressedTrapCount);
			}
			(void)sm_removedEntities_reportPort(swnodep, swportp,
			                                    SM_REMOVAL_REASON_TRAP_SUPPRESS);
			(void)sm_disable_port(&old_topology, swnodep, swportp);
			sm_discovery_needed("Trap Threshold Exceeded for a Port in Fabric", 0);
		}
	}

	if ((timedelta > sm_trapThresholdWindow)) {
		// create a new window
		ext_portp->portData->trapWindowStartTime = timenow;
		ext_portp->portData->trapWindowCount = 1;
	}
	
	IB_EXIT("sa_updateTrapCountForPort", 0);
}

int sa_TrapNeedsLogging(Port_t *portp, uint8_t *trap_count)
{
	if (!sm_valid_port(portp))
		return 1;

	if (!portp->portData->suppressTrapLog) {
		/* If the trap count was below threshold while logging was suppressed,
		 * that information would not have been logged. We can log that summary
		 * info along with trap being currently logged
		 */
		*trap_count = portp->portData->logSuppressedTrapCount;
		/* Since the previous trap count will be logged now,
		 * start with the trap count afresh
		 */
		portp->portData->logSuppressedTrapCount = 0;
		return 1;
	}
		
	/* Even if trap logging is suppressed, check if we have accumulated enough
	 * traps to reach the threshold required for logging trap summary
	 * information  about traps received since  last logging for this port.
	 */	
	if (portp->portData->logTrapSummaryThreshold == 0)
		portp->portData->logTrapSummaryThreshold = SM_TRAP_LOG_SUMMARY_THRESHOLD_START;

	if (portp->portData->logSuppressedTrapCount == portp->portData->logTrapSummaryThreshold) {
		*trap_count = portp->portData->logSuppressedTrapCount;
		/* Since the previous trap count will be logged now,
		 * start with the trap count afresh
		 */
		portp->portData->logSuppressedTrapCount = 0;
		/* Increase the threshold for trap summary logging everytime we hit the
		 * threshold to reduce frequency of logging if a port causes a lot of
		 * traps. Cap off threshold at SM_TRAP_LOG_SUMMARY_THRESHOLD_MAX.
		 */
		if (portp->portData->logTrapSummaryThreshold < SM_TRAP_LOG_SUMMARY_THRESHOLD_MAX)
			portp->portData->logTrapSummaryThreshold += SM_TRAP_LOG_SUMMARY_THRESHOLD_INCREMENT;
		return 1;
	}

	return 0;
}

/*
 * Used for forwarding traps that came from the outside.
 * In this case the caller pass the incoming mai packet
 */
Status_t
sa_Trap(Mai_t *maip) {
	STL_NOTICE  notice = {{{{0}}}};
	STL_TRAP_BAD_KEY_DATA pkeyTrap;
	STL_INFORM_INFO_RECORD *	iRecordp = NULL;
	SubscriberKeyp  subsKeyp = NULL;
	CS_HashTableItr_t itr;
    uint64_t    tid=0;
	Port_t *portp, *extPortp, *neighborPortp, *neighborExtPortp;
	Node_t *nodep, *neighborNodep;
	//Status_t    status;
	SmCsmNodeId_t csmNodeId, csmNeighborId;
	uint8_t	trap_count=0;
	char desc[110];

	IB_ENTER("sa_Trap", maip, 0, 0, 0);

	/* Get the trap type and number */
    (void)BSWAPCOPY_STL_NOTICE((STL_NOTICE *)STL_GET_SMP_DATA(maip), &notice);
    tid = maip->base.tid;

	INCREMENT_COUNTER(smCounterTrapsReceived);
	switch (notice.Attributes.Generic.TrapNumber) {
		case MAD_SMT_PORT_UP:
			INCREMENT_COUNTER(smCounterTrapPortUp);
			break;
		case MAD_SMT_PORT_DOWN:
			INCREMENT_COUNTER(smCounterTrapPortDown);
			break;
		case MAD_SMT_MCAST_GRP_CREATED:
			INCREMENT_COUNTER(smCounterTrapMcGrpCreate);
			break;
		case MAD_SMT_MCAST_GRP_DELETED:
			INCREMENT_COUNTER(smCounterTrapMcGrpDel);
			break;
		case MAD_SMT_UNPATH:
			INCREMENT_COUNTER(smCounterTrapUnPath);
			break;
		case MAD_SMT_REPATH:
			INCREMENT_COUNTER(smCounterTrapRePath);
			break;
		case MAD_SMT_PORT_CHANGE:
			INCREMENT_COUNTER(smCounterTrapPortStateChg);
			break;
		case MAD_SMT_LINK_INTEGRITY:
			INCREMENT_COUNTER(smCounterTrapLinkIntegrity);
			break;
		case MAD_SMT_BUF_OVERRUN:
			INCREMENT_COUNTER(smCounterTrapBufOverrun);
			break;
		case MAD_SMT_FLOW_CONTROL:
			INCREMENT_COUNTER(smCounterTrapFlowControl);
			break;
		case MAD_SMT_CAPABILITYMASK_CHANGE:
			INCREMENT_COUNTER(smCounterTrapLocalChg);
			break;
		case MAD_SMT_SYSTEMIMAGEGUID_CHANGE:
			INCREMENT_COUNTER(smCounterTrapSysImgChg);
			break;
		case MAD_SMT_BAD_MKEY:
			INCREMENT_COUNTER(smCounterTrapBadMKey);
			break;
		case MAD_SMT_BAD_PKEY:
			INCREMENT_COUNTER(smCounterTrapBadPKey);
			break;
		case MAD_SMT_BAD_QKEY:
			INCREMENT_COUNTER(smCounterTrapBadQKey);
			break;
		case MAD_SMT_BAD_PKEY_ONPORT:
			INCREMENT_COUNTER(smCounterTrapBadPKeySwPort);
			break;
	}

	/* Send a TrapRepress to the sender after inserting our Mkey */
    BSWAPCOPY_STL_MKEY(&sm_config.mkey, STL_GET_MAI_KEY(maip));
	(void)mai_reply(fd_async, maip);

	/* Look for subscribers */
	(void)vs_lock(&saSubscribers.subsLock);
	if (cs_hashtable_count(saSubscribers.subsMap) > 0)
	{
		cs_hashtable_iterator(saSubscribers.subsMap, &itr);
		do {
			subsKeyp = cs_hashtable_iterator_key(&itr);
			iRecordp = cs_hashtable_iterator_value(&itr);

			/* Check that the generic flags match */
			if (iRecordp->InformInfoData.IsGeneric != notice.Attributes.Generic.u.s.IsGeneric) {
				continue;
			}
			/* Check that the severity levels are OK */
			if ((iRecordp->InformInfoData.Type != TRAP_ALL) && (iRecordp->InformInfoData.Type != notice.Attributes.Generic.u.s.Type)) {
				continue;
			}
			/* Check that the trap number is OK */
			if ((iRecordp->InformInfoData.u.Generic.TrapNumber != TRAP_ALL) && (iRecordp->InformInfoData.u.Generic.TrapNumber != notice.Attributes.Generic.TrapNumber)) {
				continue;
			}
			if ((iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != NODE_TYPE_ALL) && (iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != 0) && 
					(iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != notice.Attributes.Generic.u.s.ProducerType)) {
				/* 
				 * work around host bug that used channel adapter for gid in/out service
				 * and multicast group create destroy
				 */
				if (notice.Attributes.Generic.TrapNumber >= MAD_SMT_PORT_UP && notice.Attributes.Generic.TrapNumber <= MAD_SMT_MCAST_GRP_DELETED){
					if (iRecordp->InformInfoData.u.Generic.u2.s.ProducerType != NOTICE_PRODUCERTYPE_CA) {
						continue;
					}
				} else {
					continue;
				}
			}
			/* This subscriber is OK, send them this Trap */
			(void)sa_Trap_Forward(subsKeyp, &notice);
		} while (cs_hashtable_iterator_advance(&itr));
	}
	(void)vs_unlock(&saSubscribers.subsLock);

	if (sm_config.IgnoreTraps) {
		// filter out all traps
		sm_get_lid_info(desc, notice.IssuerLID);
		IB_LOG_WARN_FMT(__func__, "Dropping trap recieved from %s", desc);
	/*
	 *	If it is a port state change or capabilitymask change, I must signal for a sweep
	 */
	} else if (notice.Attributes.Generic.TrapNumber == MAD_SMT_PORT_CHANGE) {
		if (smDebugPerf) {
			sm_get_lid_info(desc, notice.IssuerLID);
            IB_LOG_INFINI_INFO_FMT( "sa_Trap", 
                   "Received a PORT STATE CHANGE trap from %s, TID="FMT_U64, desc, tid);
		}
		/* Must tell sm_top to re-sweep fabric */
		sm_discovery_needed("Port State Change Trap", notice.IssuerLID);
	} else if (notice.Attributes.Generic.TrapNumber == MAD_SMT_CAPABILITYMASK_CHANGE) {
		STL_TRAP_CHANGE_CAPABILITY_DATA ccTrap;
		
		BSWAPCOPY_STL_TRAP_CHANGE_CAPABILITY_DATA((STL_TRAP_CHANGE_CAPABILITY_DATA*)notice.Data, &ccTrap);
		sm_get_lid_info(desc, notice.IssuerLID);

		if (ccTrap.u.AsReg16 == 0) {
			/* Change fields are zero so one of the capability bits must 
			 * have changed. Compare the new CapabilityMask to the previous
			 * CapabalityMask for this port to determine appropriate action
			 * to take*/
			(void)vs_wrlock(&old_topology_lock); 
			portp = sm_find_node_and_port_lid(&old_topology, notice.IssuerLID, &nodep);
			if (nodep && sm_valid_port(portp)) { 
				//determine which bits in Capability Mask changed
				ccTrap.CapabilityMask.AsReg32 ^= portp->portData->portInfo.CapabilityMask.AsReg32;
				
				if (ccTrap.CapabilityMask.s.IsSM){ //node's SM status changed, will need to trigger a sweep
					if (!portp->portData->portInfo.CapabilityMask.s.IsSM){
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_SM on) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64,
								ccTrap.CapabilityMask.AsReg32, desc, tid);
						sm_discovery_needed("Port CapabilityMask Change isSM on", 0);
					}else{
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_SM off) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64,
								ccTrap.CapabilityMask.AsReg32, desc, tid);
						//don't sweep if it was this sm whose bit was turned off
						if (notice.IssuerLID != sm_lid){
							sm_discovery_needed("Port CapabilityMask Change isSM off", 0);
						}
					} 
				}else{ //other capability bit changed, just get fresh port info
					STL_PORT_INFO portInfo;
					Status_t status = SM_Get_PortInfo_LR(fd_topology, (1 << 24) | portp->index, sm_lid, portp->portData->lid, &portInfo);
					if (status != VSTATUS_OK){
						IB_LOG_WARN_FMT(__func__, 
										"Cannot get PORTINFO for %s, TID="FMT_U64
										" status=%d", desc, tid, status);
					}else{
						portp->portData->portInfo = portInfo;
					}
				}
				/****************LOG WHICH OTHER BIT CHANGED******************/
				//portp->portData->portInfo contains old information
				if(ccTrap.CapabilityMask.s.IsAutomaticMigrationSupported){
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_AUTOMATIC_MIGRATION_SUPPORTED %s) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64,
								(!portp->portData->portInfo.CapabilityMask.s.IsAutomaticMigrationSupported?"on":"off"),
								ccTrap.CapabilityMask.AsReg32, desc, tid);
				}
				if(ccTrap.CapabilityMask.s.IsConnectionManagementSupported){
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_CONNECTION_MANAGEMENT_SUPPORTED %s) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64, 
								(!portp->portData->portInfo.CapabilityMask.s.IsConnectionManagementSupported?"on":"off"), 
								ccTrap.CapabilityMask.AsReg32, desc, tid);
				}
				if (ccTrap.CapabilityMask.s.IsDeviceManagementSupported){
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_DEVICE_MANAGEMENT_SUPPORTED %s) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64, 
								(!portp->portData->portInfo.CapabilityMask.s.IsDeviceManagementSupported?"on":"off"),
								ccTrap.CapabilityMask.AsReg32, desc, tid);
				}
				if(ccTrap.CapabilityMask.s.IsVendorClassSupported){
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_VENDOR_CLASS_SUPPORTED %s) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64, 
								(!portp->portData->portInfo.CapabilityMask.s.IsVendorClassSupported?"on":"off"),
								ccTrap.CapabilityMask.AsReg32, desc, tid);
				}
				if(ccTrap.CapabilityMask.s.IsCapabilityMaskNoticeSupported){
						IB_LOG_INFINI_INFO_FMT(__func__, 
								"Received an (IS_CAPABILITY_MASK_NOTICE_SUPPORTED %s) CAPABILITYMASK CHANGE [0x%.8X] trap from %s, TID="FMT_U64, 
								(!portp->portData->portInfo.CapabilityMask.s.IsCapabilityMaskNoticeSupported?"on":"off"),
								ccTrap.CapabilityMask.AsReg32, desc, tid);
				}
			}
				(void)vs_rwunlock(&old_topology_lock);
		} else {
			/* A local change has occurred. */
			IB_LOG_INFINI_INFO_FMT( "sa_Trap", 
				       "Received a (OtherLocalChanges) CAPABILITYMASK CHANGE trap from %s, TID="FMT_U64,
				        desc, tid);
			if (ccTrap.u.s.LinkSpeedEnabledChange) {
				IB_LOG_INFINI_INFO_FMT("sa_Trap", "OtherLocalChanges: PortInfo:LinkSpeedEnabled changed");
			}
			if (ccTrap.u.s.LinkWidthEnabledChange) {
				IB_LOG_INFINI_INFO_FMT( "sa_Trap", "OtherLocalChanges: PortInfo:LinkWidthEnabled changed");
			}
			if (ccTrap.u.s.LinkWidthDowngradeEnabledChange) {
				IB_LOG_INFINI_INFO_FMT( "sa_Trap", "OtherLocalChanges: PortInfo:LinkWidthDowngradeEnabled changed");
			}
			if (ccTrap.u.s.NodeDescriptionChange) {
				IB_LOG_INFINI_INFO_FMT( "sa_Trap", "OtherLocalChanges: NodeDescription changed");
				(void)vs_wrlock(&old_topology_lock);
				if (sm_find_node_and_port_lid(&old_topology, notice.IssuerLID, &nodep) != NULL) {
					nodep->nodeDescChgTrap = 1;
				}
				vs_time_get(&old_topology.lastNDTrapTime);
				(void)vs_rwunlock(&old_topology_lock);
			}
			sm_discovery_needed("Port CapabilityMask Change trap with OtherLocalChanges", 0);
		}
	} else if (notice.Attributes.Generic.TrapNumber == MAD_SMT_SYSTEMIMAGEGUID_CHANGE) {
		uint64_t    sysImageGuid;
		memcpy(&sysImageGuid, &notice.Data[6], 8);
		sysImageGuid = ntoh64(sysImageGuid);
		sm_get_lid_info(desc, notice.IssuerLID);
		IB_LOG_INFINI_INFO_FMT( "sa_Trap", 
		       "Received a SYSTEMIMAGEGUID CHANGE ["FMT_U64"] trap from %s TID="FMT_U64,
		       sysImageGuid, desc, tid);
		sm_discovery_needed("SystemImageGuid changed", 0);
    } else if (notice.Attributes.Generic.TrapNumber == STL_SMA_TRAP_LINK_WIDTH) {
		(void)vs_wrlock(&old_topology_lock);
		portp = sm_find_node_and_port_lid(&old_topology, notice.IssuerLID, &nodep);
		if (nodep && sm_valid_port(portp)) {
            if(sa_TrapNeedsLogging(portp, &trap_count)) {
                /* Updating only trap logging suppression related counts and not disabling ports */
                (void)sa_updateTrapCountForPort(notice.IssuerLID, portp->index, 0);

                smCsmFormatNodeId(&csmNodeId, (uint8_t*)sm_nodeDescString(nodep), notice.Data[4],
                                  portp->portData->guid);
                smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_LINK_ERROR, &csmNodeId, NULL,
                                "Received a LINKDOWNGRADE trap from LID=0x%.4X TID="FMT_U64,
                                notice.IssuerLID, tid);
                if (trap_count)
                    IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
                             (trap_count));
            }
        }
		(void)vs_rwunlock(&old_topology_lock);
	} else if (notice.Attributes.Generic.TrapNumber >= MAD_SMT_BAD_MKEY) {
		uint8_t log = 0;
		(void)vs_wrlock(&old_topology_lock);
		portp = sm_find_node_and_port_lid(&old_topology, notice.IssuerLID, &nodep);
		if (nodep && sm_valid_port(portp)) {
			smCsmFormatNodeId(&csmNodeId, (uint8_t*)sm_nodeDescString(nodep), portp->index, portp->portData->guid);
			if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
				/* Updating and checking counts only for FIs as for switches the LID and portp will
				 * correspond to switch port 0 and not the port that is raising the trap.
				 */
				log = sa_TrapNeedsLogging(portp, &trap_count);
				/* Updating only trap logging suppression related counts and not disabling ports */
				(void)sa_updateTrapCountForPort(notice.IssuerLID, portp->index, 0);
			} else {
				log = 1;
			}
			if (notice.Attributes.Generic.TrapNumber != MAD_SMT_BAD_PKEY) {
				if (log) {
					smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SECURITY_ERROR, &csmNodeId, NULL,
				                "Received a %s trap from LID 0x%.4X TID="FMT_U64, 
				                (notice.Attributes.Generic.TrapNumber == MAD_SMT_BAD_MKEY) ? "BAD MKEY" : 
				                  ((notice.Attributes.Generic.TrapNumber == MAD_SMT_BAD_QKEY) ? "BAD QKEY" : "BAD PKEY"),
			    	            notice.IssuerLID, tid);
					if (trap_count) 
						IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
								 (trap_count));
				}
			} else {
                (void)BSWAPCOPY_STL_TRAP_BAD_KEY_DATA((STL_TRAP_BAD_KEY_DATA *)notice.Data, &pkeyTrap);

				if (log) {
					smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SECURITY_ERROR, &csmNodeId, NULL,
				                "Received a BAD PKEY trap from LID 0x%.4X, PKEY= 0x%x (from LID= 0x%.4x to LID= 0x%.4x) on SL %d (QP1= %d,QP2 =%d) TID="FMT_U64,
				                notice.IssuerLID, pkeyTrap.Key, pkeyTrap.Lid1, pkeyTrap.Lid2, pkeyTrap.u.s.SL, pkeyTrap.qp1, pkeyTrap.qp2, tid);
					if (trap_count) 
							IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
									 (trap_count));
				}
			}
		} else {
			if (notice.Attributes.Generic.TrapNumber != MAD_SMT_BAD_PKEY) {
				smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SECURITY_ERROR, NULL, NULL,
			                "Received a %s trap from LID 0x%.4X TID="FMT_U64, 
			                (notice.Attributes.Generic.TrapNumber == MAD_SMT_BAD_MKEY) ? "BAD MKEY" :
			                  ((notice.Attributes.Generic.TrapNumber == MAD_SMT_BAD_QKEY) ? "BAD QKEY" : "BAD PKEY"),
			                notice.IssuerLID, tid);
			} else {
                (void)BSWAPCOPY_STL_TRAP_BAD_KEY_DATA((STL_TRAP_BAD_KEY_DATA *)notice.Data, &pkeyTrap);
				smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SECURITY_ERROR, NULL, NULL,
			                "Received a BAD PKEY trap from LID 0x%.4X, PKEY= 0x%x (from LID= 0x%.4x to LID= 0x%.4x) on SL %d (QP1= %d,QP2 =%d) TID="FMT_U64, 
				            notice.IssuerLID, pkeyTrap.Key, pkeyTrap.Lid1, pkeyTrap.Lid2, pkeyTrap.u.s.SL, pkeyTrap.qp1, pkeyTrap.qp2, tid);
			}
		}
		(void)vs_rwunlock(&old_topology_lock);

#if 0	// if 1, enables HACK to quiet SM when SusieQ HCA involved
	} else if (notice.Attributes.Generic.TrapNumber == 131) {
		// filter out Flow Control Update traps for now, too noisy in log
#endif
	} else if (notice.Attributes.Generic.TrapNumber >= 129 && notice.Attributes.Generic.TrapNumber <= 131) {
		char *name = notice.Attributes.Generic.TrapNumber == 129 ? "LOCAL LINK INTEGRITY" :
		            (notice.Attributes.Generic.TrapNumber == 130 ? "EXCESSIVE BUFFER OVERRUN" : "FLOW CONTROL UPDATE");
		if (topology_passcount == 0) {
			// no topology yet, just log notice info
			IB_LOG_INFINI_INFO_FMT( "sa_Trap", 
			       "Received a %s trap from LID=0x%.4X, SrcPort=%u TID="FMT_U64, 
			       name, notice.IssuerLID, notice.Data[4], tid);
		} else {
			(void)vs_wrlock(&old_topology_lock);
			if (sm_find_node_and_port_pair_lid(&old_topology, notice.IssuerLID, notice.Data[4],
				                               &nodep, &portp, &extPortp, &neighborNodep,
				                               &neighborPortp, &neighborExtPortp) == VSTATUS_OK) {

				if (sa_TrapNeedsLogging(extPortp, &trap_count)) {
					smCsmFormatNodeId(&csmNodeId, (uint8_t*)sm_nodeDescString(nodep), extPortp->index, portp->portData->guid);
					smCsmFormatNodeId(&csmNeighborId, (uint8_t*)sm_nodeDescString(neighborNodep), neighborExtPortp->index,
					                  neighborPortp->portData->guid);
					smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_LINK_ERROR, &csmNodeId, &csmNeighborId, 
					                "Received a %s trap from LID=0x%.4X TID="FMT_U64, name, notice.IssuerLID, tid);
					if (trap_count) 
						IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
								 (trap_count));
				}
			} else {
				portp = sm_find_node_and_port_lid(&old_topology, notice.IssuerLID, &nodep);
				if (nodep && sm_valid_port(portp)) {
					extPortp = sm_get_port(nodep, notice.Data[4]);
					if (sa_TrapNeedsLogging(extPortp, &trap_count)) {
						smCsmFormatNodeId(&csmNodeId, (uint8_t*)sm_nodeDescString(nodep), notice.Data[4],
					    	              portp->portData->guid);
						smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_LINK_ERROR, &csmNodeId, NULL,
					                	"Received a %s trap from LID=0x%.4X TID="FMT_U64,
						                name, notice.IssuerLID, tid);
						if (trap_count)
							IB_LOG_INFINI_INFO_FMT( "sa_Trap", "Received %d other traps from above port since last reported",
									 (trap_count));
					}
				} else {
					smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_LINK_ERROR, NULL, NULL,
					                "Received a %s trap from LID=0x%.4X TID="FMT_U64,
					                name, notice.IssuerLID, tid);
				}
			}
			/* Update trap counts for auto-disabling ports and also for
			 * checking if log suppression is required 
			 */
			(void)sa_updateTrapCountForPort(notice.IssuerLID, notice.Data[4], 1);
			(void)vs_rwunlock(&old_topology_lock);
		}
	} else {
		sm_get_lid_info(desc, notice.IssuerLID);
		IB_LOG_INFINI_INFO_FMT( "sa_Trap", 
		       "Received trap #%d from %s", 
		       notice.Attributes.Generic.TrapNumber, desc);
	}

	IB_EXIT("sa_Trap", VSTATUS_OK);
	return(VSTATUS_OK);
}

static void Stl2Ib_Notice(STL_NOTICE *stlp, IB_NOTICE *ibp) {
	STL_TRAP_GID *stlTrapDataDetails = (STL_TRAP_GID *)stlp->Data;
    TRAPS_64_65_66_67_DETAILS *ibTrapDataDetails = (TRAPS_64_65_66_67_DETAILS *)ibp->Data;

    memset(ibp, 0, sizeof(IB_NOTICE));
	ibp->u.Generic.u.AsReg32 = stlp->Attributes.Generic.u.AsReg32;
	ibp->u.Generic.TrapNumber = stlp->Attributes.Generic.TrapNumber;
	ibp->u.Vendor.u.AsReg32 = stlp->Attributes.Vendor.u.AsReg32;
	ibp->u.Vendor.DeviceID = stlp->Attributes.Vendor.DeviceID;
	ibp->IssuerLID = stlp->IssuerLID;
	ibp->Stats.Toggle = stlp->Stats.s.Toggle;
	ibp->Stats.Count = stlp->Stats.s.Count;
	ibp->IssuerGID = stlp->IssuerGID;

    // replicate Data Detail fields
    switch (ibp->u.Generic.TrapNumber) {
    case SMA_TRAP_GID_NOW_IN_SERVICE:
    case SMA_TRAP_GID_OUT_OF_SERVICE:
    case SMA_TRAP_ADD_MULTICAST_GROUP:
    case SMA_TRAP_DEL_MULTICAST_GROUP:
        memcpy(&ibTrapDataDetails->GIDAddress, &stlTrapDataDetails->Gid, sizeof(IB_GID));
        break;
    default:
        memcpy(ibp->Data, stlp->Data, sizeof(ibp->Data));
        break;
    }
}

static Status_t sa_Trap_Forward(SubscriberKeyp subsKeyp, STL_NOTICE * noticep) {
	Mai_t		out_mad;
    SA_MAD_HDR *sa_hdr = (SA_MAD_HDR*)out_mad.data;
	uint64_t	tid;
	Status_t	status;
    cntxt_entry_t *madcntxt=NULL;

	IB_ENTER("sa_Trap_Forward", subsKeyp, noticep, 0, 0);

	(void)mai_alloc_tid(fd_saTrap, MAD_CV_SUBN_ADM, &tid);

	INCREMENT_COUNTER(smCounterSaTxReportNotice);

    // Setup the outgoing MAD
	Mai_Init(&out_mad);
	AddrInfo_Init(&out_mad, sm_lid, subsKeyp->lid, 0, subsKeyp->pkey, MAI_GSI_QP, subsKeyp->qpn, GSI_WELLKNOWN_QKEY);
	LRMad_Init(&out_mad, MAD_CV_SUBN_ADM, MAD_CM_REPORT, tid, SA_NOTICE, 0, 0x0ull);	
    // Always return the trap with full pkey.  This assumes the SM/SA node must
    // always be a full member of every partition, or how would we be able to
    // respond?
    out_mad.addrInfo.pkey |= FULL_MEMBER;

    sa_hdr->RmppHdr.RmppVersion = 1;
    sa_hdr->RmppHdr.RmppType = 0;
    sa_hdr->RmppHdr.RmppStatus = 0;
    sa_hdr->RmppHdr.RmppFlags.s.RRespTime = 0;
    sa_hdr->RmppHdr.u1.SegmentNum = 0;
    sa_hdr->RmppHdr.u2.PayloadLen = 0;

    BSWAP_RMPP_HEADER(&sa_hdr->RmppHdr);

    // Convert the Notice to network byte order
    // 
	// Note, we don't copy the GIDs till after we bswap the rest of the notice.
    // This is becasue GIDs are stored in network byte order.
	if (subsKeyp->ibMode) {
        IB_GID issuerGID;
        IB_NOTICE *np = (IB_NOTICE *)(sa_hdr+1);

        memcpy(&issuerGID, &noticep->IssuerGID, sizeof(IB_GID));
    	// convert STL notice to IB notice 
    	(void)Stl2Ib_Notice(noticep, (IB_NOTICE*)(sa_hdr+1));
    	(void)BSWAP_IB_NOTICE((IB_NOTICE *)(sa_hdr+1));
        memcpy(&np->IssuerGID, &issuerGID, sizeof(IB_GID));

	} else {
        IB_GID issuerGID;
        STL_NOTICE *np = (STL_NOTICE *)(sa_hdr+1);

        memcpy(&issuerGID, &noticep->IssuerGID, sizeof(IB_GID));
    	(void)BSWAPCOPY_STL_NOTICE(noticep, (STL_NOTICE *)(sa_hdr+1));
        memcpy(&np->IssuerGID, &issuerGID, sizeof(IB_GID));

		out_mad.base.bversion=STL_BASE_VERSION;
		out_mad.base.cversion=STL_SA_CLASS_VERSION;
	}


    //IB_LOG_INFINI_INFO_FMT( "sa_Trap_Forward", 
    //       "Sending notice %d to LID 0x%.4X, ["FMT_U64"]", 
    //      (uint32_t)noticep->Attributes.Generic.TrapNumber, subsKeyp->lid, *(uint64_t *)&subsKeyp->subscriberGid[8]);

    // get a context for this report mad
    if ((madcntxt = cs_cntxt_get(&out_mad, &sm_notice_cntxt, FALSE)) == NULL) {
        // could not get a context, send report unreliably
        IB_LOG_WARN0("sa_Trap_Forward: can't allocate a notice context, sending unreliably");
		if ((status = mai_send(fd_saTrap, &out_mad)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("sa_Trap_Forward: can't send MAD unreliably rc:", status);
		}
    } else {
        // send notice out reliably
        if ((status = cs_cntxt_send_mad (madcntxt, &sm_notice_cntxt)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("sa_Trap_Forward: can't send MAD reliably rc:", status);
        }
    }

	IB_EXIT("sa_Trap_Forward", status);
	return(status);
}
