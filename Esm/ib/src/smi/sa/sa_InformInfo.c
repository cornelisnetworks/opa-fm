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
//    sa_InformInfo.c    
//
// DESCRIPTION
//    This file contains the routines to process the SA requests for
//    records of the InformInfo type.
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    sa_InformInfo
//
// DEPENDENCIES
//    ib_mad.h
//    ib_status.h
//
//
//===========================================================================//


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include <iba/stl_mad_priv.h>
#include <iba/stl_sa_priv.h>
#include "stl_print.h"

extern IB_GID nullGid;

static Status_t sa_InformInfo_Subscribe(Mai_t *, STL_INFORM_INFO *, 
										uint8_t ibMode);
static Status_t sa_InformInfo_Unsubscribe(Mai_t *, STL_INFORM_INFO *);

uint16_t saInformCount = 1;		// JSY - need bitmap of unused records

void dumpSubscriptions(void);

/*****************************************************************************/

/*
 * make 64 bit hash from subscriber key
 */
static uint64_t
sa_SubscriberHashFromKey(void *ky)
{
	uint64_t hkey = 0;
	SubscriberKeyp k = (SubscriberKeyp) ky;

	hkey = ((uint64_t) k->lid) << 48;
	hkey |= ((uint64_t) k->trapnum) << 32;
	hkey |= ((uint64_t) k->qpn) << 8;
	hkey |= ((uint64_t) k->producer & 0x7) << 5;
	hkey |= (uint64_t) k->rtv;
	return hkey;
}


/*
 * subscriber map compare function
 * return 1 if the 2 entries match, 0 otherwise
 */
static int32_t
sa_SubscriberCompare(void *key1, void *key2)
{
	SubscriberKeyp skey1 = (SubscriberKeyp) key1;
	SubscriberKeyp skey2 = (SubscriberKeyp) key2;

	if ((0 ==
		 memcmp(skey1->subscriberGid, skey2->subscriberGid, sizeof(IB_GID)))
		&& skey1->lid == skey2->lid && skey1->trapnum == skey2->trapnum
		&& skey1->qpn == skey2->qpn && skey1->producer == skey2->producer
		&& skey1->rtv == skey2->rtv && skey1->pkey == skey2->pkey
		&& skey1->qkey == skey2->qkey) {
		return 1;				/* we have a match */
	} else {
		return 0;				/* no match found */
	}
}


/*
 * subscriber hash table initialization
 */
Status_t
sa_SubscriberInit(void)
{
	Status_t status = VSTATUS_OK;
	memset(&saSubscribers, 0, sizeof(SubscriberTable_t));
	if ((status =
		 vs_lock_init(&saSubscribers.subsLock, VLOCK_FREE,
					  VLOCK_THREAD)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize sa lock");
	} else {
		if (NULL ==
			(saSubscribers.subsMap =
			 cs_create_hashtable("sa_subscriber", 16,
								 sa_SubscriberHashFromKey,
								 sa_SubscriberCompare,
								 CS_HASH_KEY_ALLOCATED))) {
			status = VSTATUS_NOMEM;
			IB_FATAL_ERROR_NODUMP
				("sa_main: Can't allocate subscriber hash table");
		}
	}
	return status;
}

/*
 * subscriber hash table delete
 */
void
sa_SubscriberDelete(void)
{
	if (!saSubscribers.subsMap
		|| vs_lock(&saSubscribers.subsLock) != VSTATUS_OK)
		return;
	(void) cs_hashtable_destroy(saSubscribers.subsMap, TRUE);	/* free
																   everything 
																 */
	(void) vs_unlock(&saSubscribers.subsLock);
	(void) vs_lock_delete(&saSubscribers.subsLock);
	memset((void *) &saSubscribers, 0, sizeof(SubscriberTable_t));
}

/*
 * subscriber hash table clear
 */
void
sa_SubscriberClear(void)
{
	if (!saSubscribers.subsMap
		|| vs_lock(&saSubscribers.subsLock) != VSTATUS_OK)
		return;
	/* easier to destroy and recreate */
	(void) cs_hashtable_destroy(saSubscribers.subsMap, TRUE);	/* free
																   everything 
																 */
	if (NULL ==
		(saSubscribers.subsMap =
		 cs_create_hashtable("sa_subscriber", 16, sa_SubscriberHashFromKey,
							 sa_SubscriberCompare,
							 CS_HASH_KEY_ALLOCATED))) {
		IB_FATAL_ERROR_NODUMP
			("sa_SubscriberClear: Can't reallocate subscriber hash table");
	}
	(void) vs_unlock(&saSubscribers.subsLock);
}

/*****************************************************************************/


Status_t
sa_InformInfo(Mai_t * maip, sa_cntxt_t * sa_cntxt)
{
	STL_SA_MAD samad;
	Status_t status;
	STL_INFORM_INFO informInfo;
	uint16_t attribOffset;
	uint8_t *data = sa_data;

	IB_ENTER("sa_InformInfo", maip, sa_cntxt, 0, 0);

	// Check Method
	if (maip->base.method == SA_CM_SET) {
		INCREMENT_COUNTER(smCounterSaRxSetInformInfo);
	} else {
		maip->base.status = MAD_STATUS_BAD_METHOD;
		IB_LOG_WARN_FMT(__func__, "invalid Method: %s (%u)",
			cs_getMethodText(maip->base.method), maip->base.method);
		(void) sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	memset(&informInfo, 0, sizeof(STL_INFORM_INFO));

	// Check Base and Class Version
	if (maip->base.bversion == STL_BASE_VERSION && maip->base.cversion == STL_SA_CLASS_VERSION) {
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad,
						sizeof(STL_INFORM_INFO));
		BSWAPCOPY_STL_INFORM_INFO((STL_INFORM_INFO *) samad.data, 
						&informInfo);
	} else if (maip->base.bversion == IB_BASE_VERSION && maip->base.cversion == SA_MAD_CVERSION) {
		/* 
		 * Convert IB query to STL. Fortunately, the only real
		 * difference is that STL Lids are bigger.
		 */
		IB_INFORM_INFO *ibInformInfo;

		/* STL SA MADs have the same headers as IB SA MADs. */
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad,
						sizeof(IB_INFORM_INFO));
		ibInformInfo = (IB_INFORM_INFO*)&samad.data;

		informInfo.GID = ibInformInfo->GID;
		BSWAP_IB_GID(&informInfo.GID);

		informInfo.IsGeneric = ibInformInfo->IsGeneric;
		informInfo.Subscribe = ibInformInfo->Subscribe;
    	informInfo.LIDRangeBegin = IB2STL_LID(ntoh16(ibInformInfo->LIDRangeBegin));
    	informInfo.LIDRangeEnd = IB2STL_LID(ntoh16(ibInformInfo->LIDRangeEnd));
    	informInfo.Type = ntoh16(ibInformInfo->Type);
    	informInfo.Reserved1 = 0;
    	informInfo.u.Generic.TrapNumber = ntoh16(ibInformInfo->u.Generic.TrapNumber);
		/* NOTA BENE: THE NAMES OF THE UNIONS CHANGED BETWEEN IB AND STL... */
    	informInfo.u.Generic.u1.AsReg32 = ntoh32(ibInformInfo->u.Generic.u2.AsReg32);
    	informInfo.Type = ntoh16(ibInformInfo->Type);
    	informInfo.Reserved1 = 0;
    	informInfo.u.Generic.TrapNumber = ntoh16(ibInformInfo->u.Generic.TrapNumber);
		/* NOTA BENE: THE NAMES OF THE UNIONS CHANGED BETWEEN IB AND STL... */
    	informInfo.u.Generic.u1.AsReg32 = ntoh32(ibInformInfo->u.Generic.u2.AsReg32);
    	informInfo.u.Generic.u2.AsReg32 = ntoh32(ibInformInfo->u.Generic.u.AsReg32);
    	informInfo.u.Generic.u1.s.Reserved2 = 0;
    	informInfo.u.Generic.u2.s.Reserved3 = 0;
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	//
	//  Split the processing based on the subscription type.
	//
	if (informInfo.Subscribe == 1) {
		/* IBTA 1.2 C15-0.2.2 - subscriptions for securty type must be
		   from trusted source */
		if (informInfo.Type == NOTICE_TYPE_SECURITY && sm_smInfo.SM_Key
			&& samad.header.smKey != sm_smInfo.SM_Key) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_WARN_FMT(__func__,
				"Subscription for security trap not from trusted source[lid=0x%.8X], smkey=0x%"PRIX64", returning status 0x%.4X",
				maip->addrInfo.slid, samad.header.smKey, maip->base.status);
			status = VSTATUS_BAD;
		} else {
			status = sa_InformInfo_Subscribe(maip, &informInfo,
											(maip->base.cversion != STL_SA_CLASS_VERSION));
		}
	} else if (informInfo.Subscribe == 0) {
		status = sa_InformInfo_Unsubscribe(maip, &informInfo);
	} else {
		status = VSTATUS_BAD;
	}

	//
	//  If the status was bad, then turn off the subscription bit.
	//
	if (status != VSTATUS_OK) {
		informInfo.Subscribe = 0;
	}
	//
	//  If we haven't already set the bad status, do it here.
	//
	if ((status != VSTATUS_OK) && (maip->base.status == MAD_STATUS_OK)) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
	}

	//
	//  Send the reply back.
	//
	if (maip->base.cversion == STL_SA_CLASS_VERSION) {
		BSWAPCOPY_STL_INFORM_INFO(&informInfo, (STL_INFORM_INFO *)data);

		attribOffset = sizeof(STL_INFORM_INFO) + Calculate_Padding(sizeof(STL_INFORM_INFO));
		sa_cntxt->attribLen = attribOffset;
		sa_cntxt_data(sa_cntxt, data, attribOffset);
	} else {
		IB_INFORM_INFO *ibInformInfo = (IB_INFORM_INFO *)data;

		ibInformInfo->GID = informInfo.GID;
		BSWAP_IB_GID(&ibInformInfo->GID);

		ibInformInfo->IsGeneric = informInfo.IsGeneric;
		ibInformInfo->Subscribe = informInfo.Subscribe;

		ibInformInfo->LIDRangeBegin = STL2IB_LID(ntoh32(informInfo.LIDRangeBegin));
		ibInformInfo->LIDRangeEnd = STL2IB_LID(ntoh32(informInfo.LIDRangeEnd));
		ibInformInfo->Type = ntoh16(informInfo.Type);
		ibInformInfo->Reserved = 0;
		ibInformInfo->u.Generic.TrapNumber = ntoh16(informInfo.u.Generic.TrapNumber);
		/* NOTA BENE: THE NAMES OF THE UNIONS CHANGED BETWEEN IB AND STL... */
		ibInformInfo->u.Generic.u2.AsReg32 = ntoh32(informInfo.u.Generic.u1.AsReg32);
		ibInformInfo->u.Generic.u.AsReg32 = ntoh32(informInfo.u.Generic.u2.AsReg32);
		ibInformInfo->u.Generic.u2.s.Reserved = 0;
		ibInformInfo->u.Generic.u.s.Reserved = 0;

		attribOffset = sizeof(IB_INFORM_INFO) + Calculate_Padding(sizeof(IB_INFORM_INFO));
		sa_cntxt->attribLen = attribOffset;
		sa_cntxt_data(sa_cntxt, data, attribOffset);
	}
	(void) sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_InformInfo", VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
sa_InformInfo_Subscribe(Mai_t * maip, STL_INFORM_INFO * iip, uint8_t ibMode)
{
	int validateLidRange = 1;
	STL_LID lid;
	STL_LID topLid;
	STL_LID LIDRangeEnd;
	STL_LID LIDRangeBegin;
	Node_t *nodep;
	Port_t *portp;
	Port_t *subscriberPortp, subsPort;
	uint32_t logseverity;
	uint16_t tmp;
	Status_t status;
	uint64_t guid;
	STL_INFORM_INFO_RECORD *iRecordp, syncRecord;
	uint64_t gid[2];
	SubscriberKeyp subsKeyp;
	SubscriberKey_t syncKey;
	char *nodeName = "Unknown Node";
	uint8_t sync = TRUE;

	IB_ENTER("sa_InformInfo_Subscribe", maip, iip, 0, 0);

	iip->u.Generic.u1.s.QPNumber = maip->addrInfo.srcqp;

	iip->Reserved1 = 0;
	iip->u.Generic.u1.s.Reserved2 = 0;
	iip->u.Generic.u2.s.Reserved3 = 0;

	lid = maip->addrInfo.slid;
	gid[0] = ntoh64(iip->GID.AsReg64s.H);
	gid[1] = ntoh64(iip->GID.AsReg64s.L);

	(void) vs_wrlock(&old_topology_lock);

	if ((subscriberPortp =
		 sm_find_active_port_lid(&old_topology, lid)) == NULL) {
		maip->base.status =
			activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
		if (activateInProgress)
			logseverity = VS_LOG_VERBOSE;
		else
			logseverity = VS_LOG_INFINI_INFO;
		if (logseverity < VS_LOG_INFINI_INFO || saDebugPerf) {
			cs_log(logseverity, "sa_InformInfo_Subscribe",
				   "Can not find source lid of 0x%.8X in topology "
				   "in request to subscribe with GID " FMT_GID
				   ", start LID 0x%.8X, end LID 0x%.8X, "
				   "PKey 0x%.4X, isGeneric %d, subscribe %d, type 0x%.4X, trap number 0x%.4X, "
				   "QPN 0x%.8X, response time of 0x%.2X, returning status 0x%.4X",
				   lid, gid[0], gid[1], iip->LIDRangeBegin,
				   iip->LIDRangeEnd, maip->addrInfo.pkey, iip->IsGeneric,
				   iip->Subscribe, iip->Type, iip->u.Generic.TrapNumber,
				   iip->u.Generic.u1.s.QPNumber,
				   iip->u.Generic.u1.s.RespTimeValue, maip->base.status);
		}
		(void) vs_rwunlock(&old_topology_lock);
		IB_EXIT("sa_InformInfo_Subscribe - Can't find source lid",
				VSTATUS_BAD);
		return (VSTATUS_BAD);
	}
	if ((nodep =
		 sm_find_port_node(&old_topology, subscriberPortp)) != NULL)
		nodeName = sm_nodeDescString(nodep);
	memcpy(&subsPort, subscriberPortp, sizeof(Port_t));
	/* 
	 * If the GID was filled in, then we need to check that there is a path
	 * between the requester and the source of the Trap.
	 */
	if (memcmp(&iip->GID, &nullGid, sizeof(IB_GID)) != 0) {
		/* 
		 * find the port LID using the GUID in the lower 8 bytes of GID
		 */
		guid = ntoh64(iip->GID.AsReg64s.L);
		if (guid == 0x0ull) {
			(void) vs_rwunlock(&old_topology_lock);
			// sa_InformInfo_Cleanup(match, &iRecord);
			IB_EXIT("sa_InformInfo_Subscribe - bad Gid/Lid", VSTATUS_BAD);
			return (VSTATUS_BAD);
		} else {
			if ((portp =
				 sm_find_active_port_guid(&old_topology, guid)) == NULL) {
				maip->base.status =
					activateInProgress ? MAD_STATUS_BUSY :
					MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT("sa_InformInfo_Subscribe",
								 "Can not find port GUID " FMT_U64 " from "
								 "%s, port " FMT_U64
								 ", lid of 0x%.8X in topology "
								 "in request to subscribe with GID "
								 FMT_GID ", start LID 0x%.8X, "
								 "end LID 0x%.8X, PKey 0x%.4X, isGeneric %d, subscribe %d, type 0x%.4X, "
								 "trap number 0x%.4X, QPN 0x%.8X, response time of 0x%.2X, "
								 "returning status 0x%.4X", guid, nodeName,
								 subsPort.portData->guid, lid, gid[0],
								 gid[1], iip->LIDRangeBegin,
								 iip->LIDRangeEnd, maip->addrInfo.pkey,
								 iip->IsGeneric, iip->Subscribe, iip->Type,
								 iip->u.Generic.TrapNumber,
								 iip->u.Generic.u1.s.QPNumber,
								 iip->u.Generic.u1.s.RespTimeValue,
								 maip->base.status);
				(void) vs_rwunlock(&old_topology_lock);
				// sa_InformInfo_Cleanup(match, &iRecord);
				IB_EXIT("sa_InformInfo_Subscribe - bad Gid/Lid",
						VSTATUS_BAD);
				return (VSTATUS_BAD);
			}
		}
		LIDRangeBegin = portp->portData->lid;
		LIDRangeEnd = LIDRangeBegin;
	} else {
		if (iip->LIDRangeBegin == STL_LID_PERMISSIVE) {
			validateLidRange = 0;
			LIDRangeBegin = 1;
			LIDRangeEnd = STL_LID_UNICAST_END;
		} else if (iip->LIDRangeEnd != 0) {
			LIDRangeBegin = iip->LIDRangeBegin;
			LIDRangeEnd = iip->LIDRangeEnd;
		} else {
			LIDRangeBegin = iip->LIDRangeBegin;
			LIDRangeEnd = iip->LIDRangeBegin;
		}
	}

	/* 
	 * We need to check that this user has access to the Lid range
	 * in question.  If the user does not have access to all of the
	 * Lids, then we reject this subscription request.
	 * No validation required when LidRangeBegin is permissive in informInfo request
	 */
	if (validateLidRange)
		for_all_ca_nodes(&old_topology, nodep) {
		for_all_end_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
				continue;
			}

			topLid =
				portp->portData->lid + (1 << portp->portData->lmc) - 1;
			if ((LIDRangeBegin <= topLid)
				&& (portp->portData->lid <= LIDRangeEnd)) {
				status =
					sa_Authenticate_Path(maip->addrInfo.slid,
										 portp->portData->lid);
				if (status != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_REQ_INVALID;
					IB_LOG_ERROR_FMT("sa_InformInfo_Subscribe",
									 "Failed to authenticate path from "
									 "%s, port " FMT_U64
									 ", lid of 0x%.8X in topology "
									 "to subscribe with GID " FMT_GID
									 ", start LID 0x%.8X, "
									 "end LID 0x%.8X, PKey 0x%.4X, isGeneric %d, subscribe %d, type 0x%.4X, "
									 "trap number 0x%.4X, QPN 0x%.8X, response time of 0x%.2X, "
									 "returning status 0x%.4X", nodeName,
									 subsPort.portData->guid, lid, gid[0],
									 gid[1], iip->LIDRangeBegin,
									 iip->LIDRangeEnd, maip->addrInfo.pkey,
									 iip->IsGeneric, iip->Subscribe,
									 iip->Type, iip->u.Generic.TrapNumber,
									 iip->u.Generic.u1.s.QPNumber,
									 iip->u.Generic.u1.s.RespTimeValue,
									 maip->base.status);
					(void) vs_rwunlock(&old_topology_lock);
					// sa_InformInfo_Cleanup(match, &iRecord);
					IB_EXIT("sa_InformInfo_Subscribe - Path access",
							VSTATUS_BAD);
					return (VSTATUS_BAD);
				}

				status = sa_Authenticate_Access(SA_INFORMINFO,
												(STL_LID) 0,
												maip->addrInfo.slid,
												portp->portData->lid);
				if (status != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_REQ_INVALID;
					IB_LOG_ERROR_FMT("sa_InformInfo_Subscribe",
									 "Failed to authenticate access from "
									 "%s, port " FMT_U64
									 ", lid of 0x%.8X in topology "
									 "to subscribe with GID " FMT_GID
									 ", start LID 0x%.8X, "
									 "end LID 0x%.8X, PKey 0x%.4X, isGeneric %d, subscribe %d, type 0x%.4X, "
									 "trap number 0x%.4X, QPN 0x%.8X, response time of 0x%.2X, "
									 "returning status 0x%.4X", nodeName,
									 subsPort.portData->guid, lid, gid[0],
									 gid[1], iip->LIDRangeBegin,
									 iip->LIDRangeEnd, maip->addrInfo.pkey,
									 iip->IsGeneric, iip->Subscribe,
									 iip->Type, iip->u.Generic.TrapNumber,
									 iip->u.Generic.u1.s.QPNumber,
									 iip->u.Generic.u1.s.RespTimeValue,
									 maip->base.status);
					(void) vs_rwunlock(&old_topology_lock);
					// sa_InformInfo_Cleanup(match, &iRecord);
					IB_EXIT("sa_InformInfo_Subscribe - Auth access",
							VSTATUS_BAD);
					return (VSTATUS_BAD);
				}
			}
		}
		}

	(void) vs_rwunlock(&old_topology_lock);

	/* 
	 * see if this entry already exist in the hash table
	 * if it does, replace the informinfo data with the incoming
	 * if it does not, create a new one and add to the hash table
	 */
	/* allocate a subscriber key for searching hash table */
	subsKeyp = (SubscriberKeyp) malloc(sizeof(SubscriberKey_t));
	if (subsKeyp == NULL) {
		IB_FATAL_ERROR_NODUMP
			("sa_InformInfo_Subscribe: Can't allocate subscriber entry/key");
		return VSTATUS_NOMEM;
	}
	memset(subsKeyp, 0, sizeof(SubscriberKey_t));
	subsKeyp->lid = maip->addrInfo.slid;
	subsKeyp->trapnum = iip->u.Generic.TrapNumber;
	subsKeyp->qpn = iip->u.Generic.u1.s.QPNumber;
	subsKeyp->producer = iip->u.Generic.u2.s.ProducerType;
	subsKeyp->rtv = iip->u.Generic.u1.s.RespTimeValue;
	subsKeyp->pkey = maip->addrInfo.pkey;
	subsKeyp->qkey = maip->addrInfo.qkey;
	subsKeyp->startLid = LIDRangeBegin;
	subsKeyp->endLid = LIDRangeEnd;
	subsKeyp->ibMode = ibMode;
	memcpy(subsKeyp->subscriberGid, subsPort.portData->gid, sizeof(IB_GID));
	/* make copy of key for sync to stanbys */
	memcpy((void *) &syncKey, (void *) subsKeyp, sizeof(SubscriberKey_t));
	/* lock out subscribers table */
	if (vs_lock(&saSubscribers.subsLock) != VSTATUS_OK) {
		free(subsKeyp);
		return VSTATUS_BAD;
	}
	if (NULL ==
		(iRecordp =
		 (STL_INFORM_INFO_RECORD *) cs_hashtable_search(saSubscribers.
														subsMap,
														subsKeyp))) {
		/* allocate a subscriber iRecord for adding to hash table */
		iRecordp =
			(STL_INFORM_INFO_RECORD *)
			malloc(sizeof(STL_INFORM_INFO_RECORD));
		if (iRecordp == NULL) {
			free(subsKeyp);
			(void) vs_unlock(&saSubscribers.subsLock);
			IB_FATAL_ERROR_NODUMP
				("sa_InformInfo_Subscribe: Can't allocate informInfoRecord entry");
			IB_EXIT("sa_InformInfo_Subscribe - memory allocation failure",
					VSTATUS_NOMEM);
			return VSTATUS_NOMEM;
		}
		memset(iRecordp, 0, sizeof(STL_INFORM_INFO_RECORD));
		tmp = saInformCount++;	/* get next sequence number */
		iRecordp->RID.Enum = tmp;
		iRecordp->InformInfoData = *iip;
		iRecordp->RID.SubscriberLID = subsPort.portData->lid;
		/* make copy of record for sync to stanbys */
		memcpy((void *) &syncRecord, (void *) iRecordp,
			   sizeof(STL_INFORM_INFO_RECORD));
		if (!cs_hashtable_insert
			(saSubscribers.subsMap, subsKeyp, iRecordp)) {
			free(subsKeyp);
			free(iRecordp);
			(void) vs_unlock(&saSubscribers.subsLock);
			IB_LOG_ERROR_FMT("sa_InformInfo_Subscribe",
							 "Failed to insert subscription from %s, port "
							 FMT_U64
							 ", lid 0x%.8X in Subscriber hashtable",
							 nodeName, subsPort.portData->guid, lid);
			IB_EXIT("sa_InformInfo_Subscribe - hashtable insert failure",
					VSTATUS_BAD);
			return (VSTATUS_BAD);
		}
		if (saDebugPerf) {
			IB_LOG_INFINI_INFO_FMT("sa_InformInfo_Subscribe",
								   "Processed subscription from "
								   "%s, port " FMT_U64
								   ", lid of 0x%.8X in topology "
								   "to subscribe with GID " FMT_GID
								   ", start LID 0x%.8X, "
								   "end LID 0x%.8X, PKey 0x%.4X, isGeneric %d, subscribe %d, type 0x%.4X, "
								   "trap number 0x%.4X, QPN 0x%.8X, response time of 0x%.2X, Qkey of 0x%.8X, "
								   "Producer 0x%.6X, returning status 0x%.4X",
								   nodeName, subsPort.portData->guid, lid,
								   gid[0], gid[1], iip->LIDRangeBegin,
								   iip->LIDRangeEnd, maip->addrInfo.pkey,
								   iip->IsGeneric, iip->Subscribe,
								   iip->Type, iip->u.Generic.TrapNumber,
								   iip->u.Generic.u1.s.QPNumber,
								   iip->u.Generic.u1.s.RespTimeValue,
								   maip->addrInfo.qkey,
								   iip->u.Generic.u2.s.ProducerType,
								   maip->base.status);
		}
	} else {
		/* entry existed already - replace value (informinfo) with new
		   input and free allocated key */
		/* ONLY if different! */
		if (memcmp(&iRecordp->InformInfoData, iip, sizeof(STL_INFORM_INFO))
			!= 0) {
			iRecordp->InformInfoData = *iip;
			/* make copy of record for sync to stanbys */
			memcpy((void *) &syncRecord, (void *) iRecordp,
				   sizeof(STL_INFORM_INFO_RECORD));
		} else {
			// Else - the records match - no need to sync.
			sync = FALSE;
		}

		free(subsKeyp);
	}
	/* sync the subscription with standby SMs */
	if (sync == TRUE) {
		(void) sm_dbsync_syncInform(DBSYNC_TYPE_UPDATE, &syncKey,
									&syncRecord);
	}
	/* unlock subscription table */
	(void) vs_unlock(&saSubscribers.subsLock);

	IB_EXIT("sa_InformInfo_Subscribe", VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
sa_InformInfo_Unsubscribe(Mai_t * maip, STL_INFORM_INFO * iip)
{
	STL_INFORM_INFO_RECORD *iRecordp;
	SubscriberKey_t subsKey;
	Status_t status = VSTATUS_OK;
	Port_t *subscriberPortp, subsPort;
	uint64_t gid[2];

	IB_ENTER("InformInfo_Unsubscribe", maip, iip, 0, 0);

	gid[0] = ntoh64(iip->GID.AsReg64s.H);
	gid[1] = ntoh64(iip->GID.AsReg64s.L);

	(void) vs_wrlock(&old_topology_lock);
	/* find the subscriber in the old topology */
	if ((subscriberPortp =
		 sm_find_active_port_lid(&old_topology, maip->addrInfo.slid)) == NULL) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_INFINI_INFO_FMT("sa_InformInfo_Unsubscribe",
							   "Cannot find source lid of 0x%.8X in topology "
							   "in request to Unsubscribe with GID "
							   FMT_GID
							   ", start LID 0x%.8X, end LID 0x%.8X, "
							   "PKey 0x%.4X, isGeneric %d, subscribe %d, type 0x%.4X, trap number 0x%.4X, "
							   "QPN 0x%.8X, response time of 0x%.2X, returning status 0x%.4X",
							   maip->addrInfo.slid, gid[0], gid[1],
							   iip->LIDRangeBegin, iip->LIDRangeEnd,
							   maip->addrInfo.pkey, iip->IsGeneric,
							   iip->Subscribe, iip->Type,
							   iip->u.Generic.TrapNumber,
							   iip->u.Generic.u1.s.QPNumber,
							   iip->u.Generic.u1.s.RespTimeValue,
							   maip->base.status);
		(void) vs_rwunlock(&old_topology_lock);
		IB_EXIT("sa_InformInfo_Subscribe - Can't find source lid",
				VSTATUS_BAD);
		return (VSTATUS_BAD);
	} else {
		memcpy(&subsPort, subscriberPortp, sizeof(Port_t));
	}
	(void) vs_rwunlock(&old_topology_lock);

	/* fill in subscriber key for searching hash table */
	memcpy(subsKey.subscriberGid, subsPort.portData->gid, sizeof(IB_GID));
	subsKey.lid = maip->addrInfo.slid;
	subsKey.trapnum = iip->u.Generic.TrapNumber;
	subsKey.qpn = iip->u.Generic.u1.s.QPNumber;
	subsKey.producer = iip->u.Generic.u2.s.ProducerType;
	subsKey.rtv = iip->u.Generic.u1.s.RespTimeValue;
	subsKey.pkey = maip->addrInfo.pkey;
	subsKey.qkey = maip->addrInfo.qkey;
	subsKey.startLid = 0;
	subsKey.endLid = 0;
	/* lock out subscribers table */
	if (vs_lock(&saSubscribers.subsLock) != VSTATUS_OK)
		return VSTATUS_BAD;
	if (NULL ==
		(iRecordp =
		 (STL_INFORM_INFO_RECORD *) cs_hashtable_remove(saSubscribers.
														subsMap,
														&subsKey))) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
		IB_LOG_VERBOSE_FMT("sa_InformInfo_Unsubscribe",
						   "Unsubscribe failed from lid 0x%.8X in topology "
						   "GID " FMT_GID
						   ", PKey 0x%.4X, type 0x%.4X, Producer 0x%.6X, "
						   "trap number 0x%.4X, QPN 0x%.8X, response time of 0x%.2X, Qkey of 0x%.8X "
						   "returning status 0x%.4X", maip->addrInfo.slid,
						   ntoh64(iip->GID.AsReg64s.H),
						   ntoh64(iip->GID.AsReg64s.L), maip->addrInfo.pkey,
						   iip->Type, iip->u.Generic.u2.s.ProducerType,
						   iip->u.Generic.TrapNumber,
						   iip->u.Generic.u1.s.QPNumber,
						   iip->u.Generic.u1.s.RespTimeValue,
						   maip->addrInfo.qkey, maip->base.status);
	} else {
		/* sync the delete with standby SMs */
		(void) sm_dbsync_syncInform(DBSYNC_TYPE_DELETE, &subsKey,
									iRecordp);
		/* free the actual subscriber iInformInfoRecord - remove only
		   frees the key */
		free(iRecordp);
	}
	(void) vs_unlock(&saSubscribers.subsLock);

	IB_EXIT("sa_InformInfo_Unsubscribe", status);
	return (status);
}

#ifdef __VXWORKS__
void
dumpSubscriptions(void)
{
	CS_HashTableItr_t itr;
	STL_INFORM_INFO_RECORD *val;
	uint32_t numsubs = 0;
	uint32_t i = 0;
    PrintDest_t dest;

	if (topology_passcount < 1) {
		sysPrintf("\nSM is currently in the %s state, count = %d\n\n",
				  sm_getStateText(sm_state), (int) sm_smInfo.ActCount);
		return;
	}
	if (!saSubscribers.subsMap || vs_lock(&saSubscribers.subsLock)) {
		sysPrintf("Fabric Manager is not running!\n");
		return;
	}
	/* iterate through the hashtable */
	if ((numsubs = cs_hashtable_count(saSubscribers.subsMap)) > 0) {
		cs_hashtable_iterator(saSubscribers.subsMap, &itr);
		sysPrintf
			("******************************************************************\n");
		sysPrintf("                  There are %d subscriptions  \n", (int) numsubs);
        PrintDestInitFile(&dest, stdout);
        sysPrintf
            ("******************************************************************\n");

		do {
			val = cs_hashtable_iterator_value(&itr);

			if (i++) PrintSeparator(&dest);
            (void)PrintStlInformInfoRecord(&dest, 0, val);
		} while (cs_hashtable_iterator_advance(&itr));
	}

	sysPrintf
		("******************************************************************\n");
	sysPrintf("                  There are %d subscriptions  \n", (int) numsubs);
	sysPrintf
		("******************************************************************\n");
	(void) vs_unlock(&saSubscribers.subsLock);
}
#endif
