/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_fsm.c                                                               //
//									     //
// DESCRIPTION								     //
//    This thread implements the SM state machine.  The details can	     //
//    be found in Volume 1, Section 14.4.1				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    state_thread					main		     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

// JSY - do we need to check the SM_Key for every packet received?

#include "os_g.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "sm_l.h"
#include "sm_dbsync.h"

#ifdef IB_STACK_OPENIB
#include "mal_g.h"
#endif
#include "time.h"
#ifdef __VXWORKS__
#include "UiUtil.h"
#endif

#define LOCAL_MOD_ID            VIEO_SM_MOD_ID

extern	IBhandle_t	fd_async;
extern	IBhandle_t	fd_sminfo;
extern	uint64_t	topology_wakeup_time;

extern	Status_t sm_fsm_standby(Mai_t *, char *);
extern	Status_t sm_fsm_master(Mai_t *, char *);
extern	Status_t sm_fsm_notactive(Mai_t *, char *);
extern	Status_t sm_fsm_discovering(Mai_t *, char *);
extern	Status_t sm_fsm_default(Mai_t *, char *);

//---------------------------------------------------------------------------//

static char *getAmod (uint32_t amod) {
    switch(amod) {
    case SM_AMOD_HANDOVER:
        return("HANDOVER");
    case SM_AMOD_ACKNOWLEDGE:
        return("HANDOVER_ACK");
    case SM_AMOD_DISABLE:
        return("DISABLE");
    case SM_AMOD_STANDBY:
        return("STANDBY");
    case SM_AMOD_DISCOVER:
        return("DISCOVER");
    default:
        return("INVALID AMOD");
    }
}

Status_t
state_event_timeout(void) {
	uint64_t	now;
	Status_t	status;
	static uint64_t lastStandbyCheck=0;

	IB_ENTER(__func__, 0, 0, 0, 0);

//
//	If this is the STANDBY state, then we need to heartbeat the Master.
//	If this is the MASTER or DISCOVERING state, then we need to do a topology sweep.
//
//  Note that sweeping in DISCOVERING is required to avoid deadlocks when we
//  abort a sweep before transitioning out of DISCOVERING.
//
	if (lastStandbyCheck == 0) {
		(void)vs_time_get(&lastStandbyCheck);
	}

	if (sm_state == SM_STATE_STANDBY) {
#if defined(IB_STACK_OPENIB)
		status = ib_refresh_devport();
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("cannot refresh sm pkeys rc:", status);
		}
#endif
		status = sm_check_Master();
		if (status != VSTATUS_OK) {
            /* Master in trouble, start discovery */
            (void)sm_transition(SM_STATE_DISCOVERING);
			sm_trigger_sweep(SM_SWEEP_REASON_MASTER_TROUBLE);
		}
	} else if (sm_state == SM_STATE_DISCOVERING || sm_state == SM_STATE_MASTER) {
		(void)vs_time_get(&now);
		if ((topology_wakeup_time != 0ull) && (now >= topology_wakeup_time)) {
			topology_wakeup_time = 0ull;
			lastStandbyCheck = now;
			sm_trigger_sweep(SM_SWEEP_REASON_SCHEDULED);

		} else if (sm_config.monitor_standby_enable &&
				   topology_wakeup_time != 0ull && 
				   sm_state == SM_STATE_MASTER && 
				   ((now - lastStandbyCheck) > sm_masterCheckInterval * sm_config.master_ping_max_fail)) {
			sm_dbsync_standbyCheck();
			lastStandbyCheck = now;
		}
	} else if (sm_state == SM_STATE_NOTACTIVE) {
		// Notify master sm that we are going away.  Most HSMs and the X switches
		// don't currently support capabilityMask trap indicating IsSm off.
		status = sm_check_Master();
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

//---------------------------------------------------------------------------//

Status_t
state_event_mad(Mai_t *maip) {
	Status_t	status, mkeyCheckStatus, dbsyncCheckStat=VSTATUS_OK;
    STL_SM_INFO theirSmInfo, ourSmInfo;
    uint64_t    mkey=0, portguid=0;
    Node_t      *nodep=NULL;
    Port_t      *portp=NULL;
    char        nodescription[64]={0};
    int         identified=0;
	boolean		uptodate = 0;
	SmRecp      smrecp = NULL;

	IB_ENTER(__func__, maip, 0, 0, 0);

	status = VSTATUS_OK;

//
//	Redundancy check on the AID.
//
	if (maip->base.aid != MAD_SMA_SMINFO) {
		IB_LOG_ERROR("not an SMInfo MAD aid:", maip->base.aid);
		IB_EXIT(__func__, VSTATUS_BAD);
		return(VSTATUS_BAD);
	}

    /* 
     * get requester's sm info
     * ONLY OUR SM (as far as I know) provides its smInfo in the LR GET request
     * we'll use that to find the standby sm node in topology (only if passcount > 0)
     */
    BSWAPCOPY_STL_SM_INFO((STL_SM_INFO *)STL_GET_SMP_DATA(maip), &theirSmInfo);

    /* get requester's nodeInfo using the portGuid from the smInfo request */
    portguid = theirSmInfo.PortGUID;
    if (sm_state == SM_STATE_MASTER && topology_passcount > 0) {
        (void)vs_rdlock(&old_topology_lock);
        if (portguid != 0) {
            if ((portp = sm_find_port_guid(&old_topology, portguid)) != NULL)
                nodep = sm_find_port_node(&old_topology, portp);
        }
        if (nodep == NULL) {
    		if (maip->addrInfo.slid != PERMISSIVE_LID) {
            	/* try find by lid */
            	portp = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, &nodep);
			}
        } else if (portguid != 0 && portguid != sm_smInfo.PortGUID) {
            identified = 1;        /* remote included his identity */
        }
        if (nodep) {
            /* get all data that we need before releasing topology lock */
            (void)memcpy(nodescription, nodep->nodeDesc.NodeString, 64);
            if (portp->portData) portguid = portp->portData->guid;
        }
        (void)vs_rwunlock(&old_topology_lock);
        if ((identified || (portguid != 0)) && sm_smInfo.PortGUID != portguid) {
            /* see if it's time to do a full sync of this SM */
            dbsyncCheckStat = sm_dbsync_fullSyncCheck(portguid);
            /* get it's current dbsync state */
			uptodate = sm_dbsync_isUpToDate(portguid, NULL);
        }
    }

    /*
     *	For 1.1 (C14-53), SM in the NOTACTIVE state, replies to SubnGet/Set of SMInfo
     *	Do an MKey check
     */
	if ((mkeyCheckStatus = sm_mkey_check(maip, &mkey)) == VSTATUS_MISMATCH) {
        IB_LOG_WARN_FMT(__func__, "Mismatch mKey["FMT_U64"] SMInfo from node %s with , lid[0x%x], portguid "FMT_U64", TID="FMT_U64,
               mkey, nodescription, maip->addrInfo.slid, portguid, maip->base.tid);
		IB_EXIT(__func__, VSTATUS_BAD);
		return(VSTATUS_BAD);
	}

    /*
     *	If this is a Get method, then send the requestor our SMInfo.
     */
	if (maip->base.method == MAD_CM_GET) {
        if (smDebugPerf) {
			IB_LOG_INFINI_INFO_FMT(__func__, "got SMInfo GET request from node %s, lid[0x%x], portguid "FMT_U64", TID="FMT_U64,
			       nodescription, maip->addrInfo.slid, portguid, maip->base.tid);
        }
		sm_smInfo.ActCount++;
        /*
         *  Do not return our SMkey if the requester fails the Mkey check
         */
        memcpy((void *)&ourSmInfo, (void *)&sm_smInfo, sizeof(sm_smInfo));
        if (mkeyCheckStatus == VSTATUS_CONDITIONAL) {
            ourSmInfo.SM_Key = 0;
        }

        // If this SM is Master, then fill in the elapsed time.
        if (ourSmInfo.u.s.SMStateCurrent == SM_STATE_MASTER) {
            ourSmInfo.ElapsedTime = (uint32_t)difftime(time(NULL),sm_masterStartTime);
        } else {
            ourSmInfo.ElapsedTime = 0;
        }

        BSWAPCOPY_STL_SM_INFO(&ourSmInfo, (STL_SM_INFO *)STL_GET_SMP_DATA(maip));
		if ((status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO))) != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, "failed to send reply [status=%d] to SMInfo GET request from node %s, lid[0x%x], portguid "FMT_U64", TID="FMT_U64,
                   status, nodescription, maip->addrInfo.slid, portguid, maip->base.tid);
        }

        /*
         * THIS IS NOT IN THE SPEC!  GET request does not really include the requester's own smInfo
         * We made our SM work this way so that the handover would happen right away instead of waiting
         * until the next sweep.  If sweep is turned off, you may never handover!
         *
         * If we're master and get is from higher priority SM in standby or higher, force sweep so we'll handover 
         */
        if (identified && sm_smInfo.u.s.SMStateCurrent == SM_STATE_MASTER)
		switch (theirSmInfo.u.s.SMStateCurrent) {
        case SM_STATE_MASTER:
        case SM_STATE_STANDBY:

			/* If this is an SM that is not in our list, trigger a sweep to add it */

			/* lock out SM record table */
			if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
				IB_LOG_ERROR("Can't lock SM Record table, rc:", status);
			} else {
				if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &theirSmInfo.PortGUID)) != NULL) {

					//trigger sweep if theirSmInfo.u.s.SMStateCurrent changed
					if (smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent != theirSmInfo.u.s.SMStateCurrent) {
						IB_LOG_INFINI_INFO_FMT(__func__, 
											   "triggering a sweep, remote SM lid[0x%x] state changed from %s to %s", 
											   maip->addrInfo.dlid, sm_getStateText(smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent), 
											   sm_getStateText(theirSmInfo.u.s.SMStateCurrent));
						sm_trigger_sweep(SM_SWEEP_REASON_UPDATED_STANDBY);
					}

				} else {
					sm_trigger_sweep(SM_SWEEP_REASON_UNEXPECTED_SM);
				}
				(void)vs_unlock(&smRecords.smLock);
			}

            if ((sm_smInfo.u.s.Priority < theirSmInfo.u.s.Priority) ||
                (sm_smInfo.u.s.Priority == theirSmInfo.u.s.Priority && sm_smInfo.PortGUID > theirSmInfo.PortGUID)) {
                if (uptodate) {
					(void)vs_lock(&handover_sent_lock);
					/* clear flags if a handover has been sent*/		
					if (handover_sent) {
						handover_sent = 0;
						triggered_handover = 0;
					}
					(void)vs_unlock(&handover_sent_lock);
					/* Prevent multiple triggers for handover sweep while the topo thread
					 * is stll sweeping to do a handover due to our previous trigger.
					 * With this even if we get multiple Get SmInfo requests (due to
					 * standby SM pings) while topo thread is still processing our
					 * previous trigger, we do not cause multiple triggers.
					 */
					if (!triggered_handover) {
						IB_LOG_INFINI_INFO_FMT(__func__, 
											   "triggering a sweep to hand over to node %s, lid[0x%x], portguid "FMT_U64", TID="FMT_U64,
											   nodescription, maip->addrInfo.dlid, theirSmInfo.PortGUID, maip->base.tid);  // lids got swapped by mai_reply
						sm_trigger_sweep(SM_SWEEP_REASON_HANDOFF);
						triggered_handover = 1;
					}
                }
			} else if (sm_config.monitor_standby_enable) {
                sm_dbsync_standbyHello(theirSmInfo.PortGUID);
            }
			break;
        case SM_STATE_DISCOVERING:
        	if (dbsyncCheckStat != VSTATUS_NOT_FOUND) {
				IB_LOG_INFINI_INFO_FMT(__func__, 
					"Standby SM in Discovery, may have been restarted, node %s, lid[0x%x], portguid "FMT_U64,
					nodescription,  maip->addrInfo.dlid, theirSmInfo.PortGUID); 
				/* If it restarted, forget what we think we know about it */
				sm_dbsync_deleteSm(theirSmInfo.PortGUID);
			}
			break;
        case SM_STATE_NOTACTIVE:
			IB_LOG_INFINI_INFO_FMT(__func__, 
				"standby SM indicating no longer active, node %s, lid[0x%x], portguid "FMT_U64,
				nodescription,  maip->addrInfo.dlid, theirSmInfo.PortGUID); 
			sm_dbsync_deleteSm(theirSmInfo.PortGUID);

			break;

        } // end of case

		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	} // end if MAD_CM_GET

    if (smDebugPerf) {
        IB_LOG_INFINI_INFO_FMT(__func__, "got SMInfo SET request from node %s, Lid [0x%x], portguid "FMT_U64", TID="FMT_U64,
               nodescription, maip->addrInfo.slid, portguid, maip->base.tid);
    }
    /*
     *	This is now a Set method.  Let's check the transitions.  C14-38.1.1
     */
    if ( (maip->base.amod != SM_AMOD_ACKNOWLEDGE && theirSmInfo.u.s.SMStateCurrent != SM_STATE_MASTER) ) {
		maip->base.status = MAD_STATUS_BAD_FIELD;
        IB_LOG_WARN_FMT(__func__, 
               "SmInfo SET control packet not from a Master SM on node %s, lid [0x%x], portguid "FMT_U64", TID="FMT_U64,
               nodescription, maip->addrInfo.slid, portguid, maip->base.tid);
		if ((status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO))) != VSTATUS_OK)
            IB_LOG_WARN_FMT(__func__, 
                   "failed to send reply [status=%d] to SMInfo SET request from node %s, lid [0x%x], portguid "FMT_U64", TID="FMT_U64,
                   status, nodescription, maip->addrInfo.slid, portguid, maip->base.tid);
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
    }

	switch (sm_state) {
	case SM_STATE_STANDBY:
		(void)sm_fsm_standby(maip, nodescription);
		break;
	case SM_STATE_NOTACTIVE:
		(void)sm_fsm_notactive(maip, nodescription);
		break;
	case SM_STATE_MASTER:
		(void)sm_fsm_master(maip, nodescription);
		break;
	case SM_STATE_DISCOVERING:
		(void)sm_fsm_discovering(maip, nodescription);
		break;
	default:
		(void)sm_fsm_default(maip, nodescription);
		break;
	}

	IB_EXIT(__func__, status);
	return(status);
}

Status_t
sm_fsm_standby(Mai_t *maip, char *nodename)
{
	int		i;
	uint8_t		ipath[64];
	uint8_t		*rpath;
	uint8_t		*path;
	Status_t	status;
    long        new_state=-1;
    STL_SM_INFO    theirSmInfo;
	STL_SM_INFO smInfoCopy;
    STL_LID       slid=maip->addrInfo.slid;    // original source lid; mai_reply swaps slid-dlid
    STL_LID       dlid=maip->addrInfo.dlid;    // original destination lid(us); mai_reply swaps slid-dlid

	IB_ENTER(__func__, maip->base.amod, 0, 0, 0);

    BSWAPCOPY_STL_SM_INFO((STL_SM_INFO *)STL_GET_SMP_DATA(maip), &theirSmInfo);

	switch (maip->base.amod) {
	case SM_AMOD_DISCOVER:				// C14-48
		new_state = SM_STATE_DISCOVERING;
		break;
	case SM_AMOD_DISABLE:				// C14-19
		new_state = SM_STATE_NOTACTIVE;
		break;
	case SM_AMOD_HANDOVER:				// C14-50
		new_state = SM_STATE_MASTER;
		break;
	case SM_AMOD_ACKNOWLEDGE:			// from previous HANDOVER
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_ATTR;
		break;
	}

    if (maip->base.status > MAD_STATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
			"[%s] SM received invalid AMOD[%d] from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
			sm_getStateText(sm_smInfo.u.s.SMStateCurrent), maip->base.amod, nodename,
			maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
    } else {
        IB_LOG_INFINI_INFO_FMT(__func__, 
               "SM in [%s] state after processing %s smInfo control from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
               (new_state < 0 ? sm_getStateText(sm_smInfo.u.s.SMStateCurrent):sm_getStateText(new_state)), getAmod(maip->base.amod), 
               nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
    }
    
    /*
     * Reply to this Set(SMInfo).
     */
	sm_smInfo.ActCount++;
    BSWAPCOPY_STL_SM_INFO(&sm_smInfo, (STL_SM_INFO *)STL_GET_SMP_DATA(maip));
	status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO));
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("sm_fsm_standby - bad mai_reply rc:", status);
	}
    
	(void)vs_lock(&new_topology_lock);

    /* make appropriate none master transition if necessary */
    if (new_state >= SM_STATE_NOTACTIVE && new_state != SM_STATE_MASTER) {
        (void)sm_transition(new_state);
        if (new_state == SM_STATE_DISCOVERING) {
            sm_trigger_sweep(SM_SWEEP_REASON_STATE_TRANSITION); // wakeup the topology thread to start sweep
        }
    }

    /*
     * If this was a HANDOVER, we need to ACK back.
     */
	if (maip->base.amod == SM_AMOD_HANDOVER) {
		maip->addrInfo.destqp &= 0x00ffffff;

		if (maip->base.mclass == MAD_CV_SUBN_LR) {
			path = NULL;
            sm_topop->dlid = slid;    // this is original sender of SmInfo
            sm_topop->slid = dlid;    // this is us
            IB_LOG_INFINI_INFO_FMT(__func__, "sending LR HANDOVER ACK to node %s, Lid [0x%x], portguid "FMT_U64,
                   nodename, slid, theirSmInfo.PortGUID);
		} else {
            DRStlSmp_t *drsmp = (DRStlSmp_t *)maip->data;

			path = ipath;
			memset((void *)ipath, 0, 64);

			ipath[0] = maip->base.hopCount;
			rpath = drsmp->RetPath;
			for (i = 1; i <= maip->base.hopCount; i++) {
				ipath[i] = rpath[maip->base.hopCount + 1 - i];
			}
            IB_LOG_INFINI_INFO_FMT(__func__, "sending DR HANDOVER ACK to node %s, portguid "FMT_U64,
                   nodename, theirSmInfo.PortGUID);
		}
		smInfoCopy = sm_smInfo;
		status = SM_Set_SMInfo(fd_sminfo, SM_AMOD_ACKNOWLEDGE, path, &smInfoCopy, sm_config.mkey);
		if (status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                   "[%s] SM did not receive response to Handover Acknowledgement from SM node %s, LID [0x%x], portguid ["FMT_U64"]",
                   sm_getStateText(new_state), nodename, slid, theirSmInfo.PortGUID);
		} 
        /* make transition to MASTER state */
        (void)sm_transition(new_state);
        sm_trigger_sweep(SM_SWEEP_REASON_STATE_TRANSITION);
    }
	(void)vs_unlock(&new_topology_lock);

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_fsm_notactive(Mai_t *maip, char *nodename)
{
	Status_t	status;
    long        new_state=-1;
    STL_SM_INFO    theirSmInfo;

	IB_ENTER(__func__, maip->base.amod, sm_state, 0, 0);

    BSWAPCOPY_STL_SM_INFO((STL_SM_INFO *)STL_GET_SMP_DATA(maip), &theirSmInfo);

	switch (maip->base.amod) {
    case SM_AMOD_STANDBY:				// C14-54.1.1
        IB_LOG_INFINI_INFO_FMT(__func__, 
               "[%s] SM received request to transition to STANDBY from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
               sm_getStateText(sm_smInfo.u.s.SMStateCurrent), nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
		new_state = SM_STATE_STANDBY;
		break;
    case SM_AMOD_DISABLE:
        IB_LOG_WARN_FMT(__func__, 
               "[%s] SM received request to transition to NOTACTIVE from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
               sm_getStateText(sm_smInfo.u.s.SMStateCurrent), nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
		new_state = SM_STATE_NOTACTIVE;
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_ATTR;
		IB_LOG_WARN_FMT(__func__,
			"[%s] SM received invalid transition request %s (%u) from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
			sm_getStateText(sm_smInfo.u.s.SMStateCurrent), getAmod(maip->base.amod), maip->base.amod,
			nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
		break;
	}
    
    /*
     * Reply to this Set(SMInfo).
     */
	sm_smInfo.ActCount++;
    BSWAPCOPY_STL_SM_INFO(&sm_smInfo, (STL_SM_INFO *)STL_GET_SMP_DATA(maip));
	status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO));
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("sm_fsm_notactive - bad mai_reply rc:", status);
	}

    /* make appropriate transition if necessary */
    if (new_state >= SM_STATE_NOTACTIVE) 
        (void)sm_transition(new_state);
    
	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_fsm_master(Mai_t *maip, char *nodename)
{
	Status_t	status;
    long        new_state=-1;
    STL_SM_INFO    theirSmInfo;
	uint8_t		ipath[64];
	uint8_t		*rpath;
	uint8_t		*path;
    int         i, wakeTpThread=0;
	STL_SM_INFO 	smInfoCopy;
    STL_LID       slid=maip->addrInfo.slid;    // original source lid; mai_reply swaps slid-dlid
    STL_LID       dlid=maip->addrInfo.dlid;    // original destination lid(us); mai_reply swaps slid-dlid

	IB_ENTER(__func__, maip->base.amod, sm_state, 0, 0);

    BSWAPCOPY_STL_SM_INFO((STL_SM_INFO *)STL_GET_SMP_DATA(maip), &theirSmInfo);

	switch (maip->base.amod) {
    case SM_AMOD_HANDOVER:				// C14-61
        new_state = SM_STATE_MASTER;
        IB_LOG_INFINI_INFO_FMT(__func__, 
               "[%s] SM received '%s' from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
               sm_getStateText(sm_smInfo.u.s.SMStateCurrent), getAmod(maip->base.amod), nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
		break;
    case SM_AMOD_ACKNOWLEDGE:
        if (theirSmInfo.u.s.SMStateCurrent < SM_STATE_STANDBY) {  // C14-38.1.1 almost
            maip->base.status = MAD_STATUS_BAD_FIELD;
            IB_LOG_WARN_FMT(__func__, 
                   "[%s] SM received invalid Handover Ack from remote SM %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64"; remote not in STANDBY state [%s]",
                   sm_getStateText(sm_smInfo.u.s.SMStateCurrent), nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid, sm_getStateText(theirSmInfo.u.s.SMStateCurrent));
        } else {
            new_state = SM_STATE_STANDBY;
            wakeTpThread = 1;
            IB_LOG_INFINI_INFO_FMT(__func__, 
                   "[%s] SM received '%s' from SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
                   sm_getStateText(sm_smInfo.u.s.SMStateCurrent), getAmod(maip->base.amod), nodename, maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
        }
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_ATTR;
        IB_LOG_WARN_FMT(__func__, 
               "[%s] SM received invalid MASTER transition [%s] from remote [%s] SM %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
               sm_getStateText(sm_smInfo.u.s.SMStateCurrent), getAmod(maip->base.amod), sm_getStateText(theirSmInfo.u.s.SMStateCurrent), nodename, 
               maip->addrInfo.slid, theirSmInfo.PortGUID, maip->base.tid);
		break;
	}
    
    /*
     * Reply to this Set(SMInfo).
     */
	sm_smInfo.ActCount++;
    BSWAPCOPY_STL_SM_INFO(&sm_smInfo, (STL_SM_INFO *)STL_GET_SMP_DATA(maip));
	status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO));
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("sm_fsm_master - bad mai_reply rc:", status);
    }

	(void)vs_lock(&new_topology_lock);

    /* make appropriate transition if necessary */
    if (new_state >= SM_STATE_NOTACTIVE && new_state != sm_state)
        (void)sm_transition(new_state);
    if (wakeTpThread) {
        /* wakeup the topology thread to clean up SA tables */
        sm_trigger_sweep(SM_SWEEP_REASON_HANDOFF);
    }
    
    /*
     * If this was a HANDOVER, we need to ACK back.
     */
    if (maip->base.amod == SM_AMOD_HANDOVER) {
        maip->addrInfo.destqp &= 0x00ffffff;

        if (maip->base.mclass == MAD_CV_SUBN_LR) {
            path = NULL;
            sm_topop->dlid = slid;    // this is original sender of SmInfo
            sm_topop->slid = dlid;    // this is us
            IB_LOG_INFINI_INFO_FMT(__func__, "sending LR HANDOVER ACK to node %s, Lid [0x%x], portguid "FMT_U64,
                   nodename, slid, theirSmInfo.PortGUID);
        } else {
            DRStlSmp_t *drsmp = (DRStlSmp_t *)maip->data;

            path = ipath;
            memset((void *)ipath, 0, 64);

            ipath[0] = maip->base.hopCount;
			rpath = drsmp->RetPath;
            for (i = 1; i <= maip->base.hopCount; i++) {
                ipath[i] = rpath[maip->base.hopCount + 1 - i];
            }
            IB_LOG_INFINI_INFO_FMT(__func__, "sending DR HANDOVER ACK to node %s, portguid "FMT_U64,
                   nodename, theirSmInfo.PortGUID);
        }
        smInfoCopy = sm_smInfo;
        status = SM_Set_SMInfo(fd_sminfo, SM_AMOD_ACKNOWLEDGE, path, &smInfoCopy, sm_config.mkey);
        if (status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                   "[%s] SM did not receive response to Handover Acknowledgement from [%s] SM node %s, LID [0x%x], portguid ["FMT_U64"]",
                   sm_getStateText(sm_smInfo.u.s.SMStateCurrent), sm_getStateText(theirSmInfo.u.s.SMStateCurrent), nodename, slid, theirSmInfo.PortGUID);
        } else {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                   "[%s] SM successfully acknowleded Handover from remote SM node %s, LID [0x%x], portguid ["FMT_U64"]",
                   sm_getStateText(sm_smInfo.u.s.SMStateCurrent), nodename, slid, theirSmInfo.PortGUID);
        }
        /* make transition to MASTER state */
        (void)sm_transition(new_state);
        sm_trigger_sweep(SM_SWEEP_REASON_STATE_TRANSITION); // wakeup the topology thread
    }
	(void)vs_unlock(&new_topology_lock);

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_fsm_discovering(Mai_t *maip, char *nodename)
{
	Status_t	status;
    STL_SM_INFO theirSmInfo;
    STL_LID       slid=maip->addrInfo.slid;    // original source lid; mai_reply swaps slid-dlid

	IB_ENTER(__func__, maip->base.amod, sm_state, 0, 0);

    BSWAPCOPY_STL_SM_INFO((STL_SM_INFO *)STL_GET_SMP_DATA(maip), &theirSmInfo);
//
//	The SM in the DISCOVERING state does not have any valid transitions via
//	the Set(SMInfo) method.
//
	maip->base.status = MAD_STATUS_BAD_ATTR;
    
	sm_smInfo.ActCount++;
    BSWAPCOPY_STL_SM_INFO(&sm_smInfo, (STL_SM_INFO *)STL_GET_SMP_DATA(maip));

	status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO));
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("sm_fsm_discovering - bad mai_reply rc:", status);
	}
    IB_LOG_WARN_FMT(__func__, 
           "No transitions allowed from DISCOVERING state; Got %s request from [%s] SM node %s, LID [0x%x], portguid ["FMT_U64"], TID="FMT_U64,
           getAmod(maip->base.amod), sm_getStateText(theirSmInfo.u.s.SMStateCurrent), nodename, slid, theirSmInfo.PortGUID, maip->base.tid);

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_fsm_default(Mai_t *maip, char *nodename)
{
	Status_t	status;

	IB_ENTER(__func__, maip->base.amod, sm_state, 0, 0);

//
//	Send our reply to the requester.
//
	sm_smInfo.ActCount++;
    BSWAPCOPY_STL_SM_INFO(&sm_smInfo, (STL_SM_INFO *)STL_GET_SMP_DATA(maip));

	maip->base.status = MAD_STATUS_BAD_ATTR;
	status = mai_stl_reply(fd_async, maip, sizeof(STL_SM_INFO));

	IB_EXIT(__func__, status);
	return(VSTATUS_OK);
}

// -------------------------------------------------------------------------- //
// Fetch the master SMInfo and check the activity counter.
// If returns != VSTATUS_OK, this standby SM should enter DISCOVERING state

Status_t
sm_check_Master() {
	Status_t	status;
	STL_SM_INFO	theirSmInfo;
	STL_PORT_INFO	portInfo;
	uint8_t		path[64];

    static  uint32_t fsmCheckMasterFailed=0; // count of fails to check master
    static  uint32_t fsmMultMaxFail = 1; // multiplier for sm_config.master_ping_max_fail

    IB_ENTER(__func__, 0, 0, 0, 0);

    (void)memset((void *)path, 0, 64);

    if ((status = SM_Get_PortInfo(fd_sminfo, 1<<24, path, &portInfo)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("failed to get master SM Lid from my PortInfo, rc:", status);
        // having a local problem
        // reset count, must be healthy before we can consider becoming master
        fsmCheckMasterFailed = 0;
        goto stay_standby;
    }

    if (portInfo.LID == 0 || portInfo.MasterSMLID == 0
        || portInfo.PortStates.s.PortState != IB_PORT_ACTIVE
        ) {
        if (smDebugPerf) {
            if (portInfo.PortStates.s.PortState != IB_PORT_ACTIVE) {
                IB_LOG_INFINI_INFO("our portInfo indicates state not active; portState=", (int)portInfo.PortStates.s.PortState);
            } else
            if (portInfo.MasterSMLID == 0) {
                IB_LOG_INFINI_INFOX("our portInfo smLid is not set yet; Lid=", portInfo.LID);
            }
        }
        // stay in standby until link comes up
        if (portInfo.PortStates.s.PortState > IB_PORT_DOWN) {
            IB_LOG_WARN0("Switching to DISCOVERY state; Local port uninitialized");
            goto discovering;
        }
        goto stay_standby;
    }

    // make sure we aren't trying to talk to ourself during the handover window
    if (portInfo.LID == portInfo.MasterSMLID) {
        // we're talking to ourself.  if a master SM doesn't come along and
        // reprogram our SM LID in the timeout period, attempt a discovery.
        // Since for large fabrics, the master can take some time to program
        // our LID, use a higher upper limit for failure count to give the
        // master enough time to program our SM LID.
        fsmMultMaxFail = 2;
        if (++fsmCheckMasterFailed >= fsmMultMaxFail * sm_config.master_ping_max_fail) {
            IB_LOG_WARN0("Switching to DISCOVERY state; Timed out waiting for SM LID to get reprogrammed");
            goto discovering;
        }
        if (smDebugPerf) {
            IB_LOG_INFINI_INFOX("our portInfo smLid is not set yet; smLid=", portInfo.MasterSMLID);
        }
        goto stay_standby; // not yet at threshold
    }

    sm_topop->slid = portInfo.LID;
    sm_topop->dlid = portInfo.MasterSMLID;
    if ((status = SM_Get_SMInfo(fd_sminfo, 0, NULL, &theirSmInfo)) != VSTATUS_OK) {
        if (++fsmCheckMasterFailed >= fsmMultMaxFail * sm_config.master_ping_max_fail) {
            IB_LOG_WARNX("Switching to DISCOVERY state; Failed to get SmInfo from master SM at LID:", portInfo.MasterSMLID);
            goto discovering;
        } else {
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "failed to get SmInfo from master SM at LID[0x%X], retry count=%d",
                   portInfo.MasterSMLID, fsmCheckMasterFailed);
            goto stay_standby; // not yet at threshold
        }
    }

    sm_saw_another_sm = TRUE;

    /* 
     * PR 105313 - restart results in 2 standby SMs 
     * Must check the state we get back to make sure master is still master
     */
    if (theirSmInfo.u.s.SMStateCurrent != SM_STATE_MASTER) {
        IB_LOG_WARN_FMT(__func__,
               "SmInfo from SM at SMLID[0x%X] indicates SM is no longer master, switching to DISCOVERY state",
               portInfo.MasterSMLID);
        goto discovering;
    }

    // all OK, save data about master, reset fail count and threshold
    sm_topop->sm_count = theirSmInfo.ActCount;
    sm_topop->sm_key = theirSmInfo.SM_Key;

    fsmCheckMasterFailed = 0;
    fsmMultMaxFail = 1;

    if (smDebugPerf) {
        IB_LOG_INFINI_INFO_FMT(__func__, 
               "Master SM["FMT_U64"] at LID=0x%x has priority[%d] and smKey ["FMT_U64"]",
               theirSmInfo.PortGUID, portInfo.MasterSMLID, theirSmInfo.u.s.Priority, theirSmInfo.SM_Key);
    }

stay_standby:
    status = VSTATUS_OK;

done:
    IB_EXIT(__func__, status);
    return(status);

discovering:
    // reset count and threshold in case stay standby
    fsmCheckMasterFailed = 0;
    fsmMultMaxFail = 1;
    status = VSTATUS_BAD;
    goto done;
}
