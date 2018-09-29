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
//    sa_McMemberRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the McMemberRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_McMemberRecord							     //
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
#include "cs_g.h"
#include "cs_csm_log.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "iba/ib_helper.h"
#include "stl_print.h"

McGroup_t   *sm_McGroups = NULL;
uint32_t    sm_numMcGroups = 0; 
ATOMIC_UINT sm_McGroups_Need_Prog = 0; 
Lock_t      sm_McGroups_lock;

extern	IB_GID nullGid;
extern	Pool_t			sm_pool;

/* Allow a McMemberRecord change to trigger a sweep */
extern  uint32_t		sa_mft_reprog;

/************ support for dynamic update of switch config parms *************/
extern uint8_t sa_dynamicPlt[];

static Status_t	sa_McMemberRecord_GetTable(Mai_t *, uint32_t *);
static Status_t	sa_McMemberRecord_IBGetTable(Mai_t *, uint32_t *);

static void	sa_updateMcDeleteCountForPort(Port_t *);

/*         0 0 0 0 0 0 0 0    0 0 1 1 1 1 1 1
 *         0 1 2 3 4 5 6 7    8 9 0 1 2 3 4 5
 * GID: 0xff12401bffff0000:0x00000000ffffffff
 *                pkey
 */

// Store in little endian format.
IB_GID broadcastGid = {.Raw =	{0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0xff, 0xff, 0x1b, 0x40, 0x10 | IB_LINK_LOCAL_SCOPE, 0xff}};
IB_GID allNodesGid = {.Raw =	{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0xff, 0xff, 0x1b, 0x40, 0x10 | IB_LINK_LOCAL_SCOPE, 0xff}};
IB_GID allRoutersGid = {.Raw =	{0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0xff, 0xff, 0x1b, 0x40, 0x10 | IB_LINK_LOCAL_SCOPE, 0xff}};
IB_GID mcastDnsGid = {.Raw =	{0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0xff, 0xff, 0x1b, 0x40, 0x10 | IB_LINK_LOCAL_SCOPE, 0xff}};
IB_GID otherGid = {.Raw =	{0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0xff, 0xff, 0x1b, 0x40, 0x10 | IB_LINK_LOCAL_SCOPE, 0xff}};

int	emptyBroadcastGroup = 0;

#define PKEY_INDEX 10
#define IPV6_INDEX 13

static uint32_t rateOneStepSmaller(uint32_t rate) {
	// TBD - Figure out what should be done, table
	// values interspersed or checks for EXT support(?)
	// A device that doesn't support FDR, which has
	// rate selector of less than 20G, does it make sense
	// to create it at 14G?
    switch (rate) {
    case IB_STATIC_RATE_2_5G:
        return IB_STATIC_RATE_MIN-1; //invalid
    case IB_STATIC_RATE_5G:
        return IB_STATIC_RATE_2_5G;
    case IB_STATIC_RATE_10G:
        return IB_STATIC_RATE_5G;
    case IB_STATIC_RATE_20G:
        return IB_STATIC_RATE_10G;
    case IB_STATIC_RATE_30G:
        return IB_STATIC_RATE_20G;
    case IB_STATIC_RATE_40G:
        return IB_STATIC_RATE_30G;
    case IB_STATIC_RATE_60G:
        return IB_STATIC_RATE_40G;
    case IB_STATIC_RATE_80G:
        return IB_STATIC_RATE_56G;
	// FDR
	case IB_STATIC_RATE_14G:
        return IB_STATIC_RATE_10G;
	case IB_STATIC_RATE_56G:
        return IB_STATIC_RATE_40G;
	case IB_STATIC_RATE_112G:
        return IB_STATIC_RATE_80G;
	case IB_STATIC_RATE_168G:
        return IB_STATIC_RATE_100G;
	// EDR
	case IB_STATIC_RATE_25G:
        return IB_STATIC_RATE_20G;
	case IB_STATIC_RATE_100G:
        return IB_STATIC_RATE_80G;
	case IB_STATIC_RATE_200G:
        return IB_STATIC_RATE_168G;
	case IB_STATIC_RATE_300G:
        return IB_STATIC_RATE_200G;

    default:
        return IB_STATIC_RATE_5G;
    }
}

Status_t sa_McGroupInit(void) {
    sm_McGroups = NULL;
    return vs_lock_init(&sm_McGroups_lock, VLOCK_FREE, VLOCK_THREAD);
}


// Deletes all multicast groups. Intended for SA shutdown.
void sa_McGroupDelete(void) {
    (void)clearBroadcastGroups(FALSE);
    (void)vs_lock_delete(&sm_McGroups_lock);
    memset((void *)&sm_McGroups_lock, 0, sizeof(sm_McGroups_lock));
}

Status_t
sa_McMemberRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
	uint32_t	records;
	uint16_t	attribOffset;
	uint16_t	rec_sz;

	IB_ENTER("sa_McMemberRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Base and Class Version
	if (maip->base.bversion == IB_BASE_VERSION && maip->base.cversion == SA_MAD_CVERSION) {
		rec_sz = sizeof(IB_MCMEMBER_RECORD);
	} else {
		// While originally in the spec, STL_MCMEMBER_RECORD was later dropped
		// from it. As a result, while we use it internally it is not supported
		// over the wire. Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

//
//	Check the method.  If this is a template lookup, then call the regular
//	GetTable(*) template lookup routine.
//
	switch (maip->base.method) {
	case SA_CM_SET:
		INCREMENT_COUNTER(smCounterSaRxSetMcMemberRecord);
		(void)sa_McMemberRecord_Set(NULL, maip, &records);
		break;
	case SA_CM_GET:
		INCREMENT_COUNTER(smCounterSaRxGetMcMemberRecord);
		if (maip->base.bversion == STL_BASE_VERSION)
			(void)sa_McMemberRecord_GetTable(maip, &records);
		else
			(void)sa_McMemberRecord_IBGetTable(maip, &records);
		break;
	case SA_CM_GETTABLE:
		INCREMENT_COUNTER(smCounterSaRxGetTblMcMemberRecord);
		if (maip->base.bversion == STL_BASE_VERSION)
			(void)sa_McMemberRecord_GetTable(maip, &records);
		else
			(void)sa_McMemberRecord_IBGetTable(maip, &records);
		break;
	case SA_CM_DELETE:
		INCREMENT_COUNTER(smCounterSaRxDeleteMcMemberRecord);
		(void)sa_McMemberRecord_Delete(NULL, maip, &records);
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_METHOD;
		IB_LOG_WARN_FMT(__func__, "invalid Method: %s (%u)",
			cs_getMethodText(maip->base.method), maip->base.method);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

//
//	Determine reply status
//
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_McMemberRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}


	if (maip->base.method == SA_CM_SET && maip->base.status == 0x0700) {
		// drop the request on the floor until hosts backoff Mcmember requests when we are busy
	} else {
		attribOffset = rec_sz + Calculate_Padding(rec_sz);
		sa_cntxt->attribLen = attribOffset;

		sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
		(void)sa_send_reply(maip, sa_cntxt);
	}

	IB_EXIT("sa_McMemberRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

//
// Adds a node to a Multicast Group, creating the group if necessary.
// Note:
// 1. The primary use of this function is to process incoming MADs but it is
//    also used internally within the SM.
// 2. The main code path works on the "old" topology and the global list of 
//    McGroups. If a different topology pointer is passed to this function as
//    an argument, this function will still alter the global McGroups list. In
//    addition, when a topology pointer is passed in, this function assumes the
//    caller already holds any required locks.
//
Status_t
sa_McMemberRecord_Set(Topology_t *topop, Mai_t *maip, uint32_t *records)
{
	uint8_t			mtu, maxMtu, vfMaxMtu;
	uint8_t			rate, maxRate, vfMaxRate;
	uint8_t 		activeRate;
	uint8_t			life;
	uint8_t			saLife, scope=0;
	STL_LID		    mLid;
	uint64_t		guid;
	uint64_t		prefix;
	// portp is port requested to join/leave MC group
	// req_portp, req_nodep is port which requested the change
	// req_nodeName is name of node which requested the change
	// in typical case, portp == req_portp
	Port_t			*portp, *mcmPortp;
	Port_t			*req_portp, *neighbor_portp;
	char *			req_nodeName			= "Unknown Node";
	STL_SA_MAD			samad;
	Status_t		status				= VSTATUS_OK;
	Node_t			*req_nodep;
	McGroup_t		*mcGroup;
	McMember_t		*mcMember;
	STL_MCMEMBER_RECORD	mcMemberRecord = {{{{0}}}};
	STL_MCMEMBER_RECORD	*mcmp;
	STL_NOTICE		notice;
	STL_NOTICE		*trap66				= (STL_NOTICE *)&notice;
	int				createdGroup		= 0, newJoinState=1;
	uint64_t		tempMask;
	uint64_t		mGid[2];
	uint8_t			hopLimit;
	uint32_t		qKey;
	SmCsmNodeId_t	csmReqNode, csmConnectedNode;
	SmCsmNodeId_t	*csmNeighborp = NULL;
	bitset_t        mcGroupVf;
	char			*vfp=NULL;
	IB_MCMEMBER_RECORD temporary_rec;
	boolean			got_old_topology_lock = FALSE;

	IB_ENTER("sa_McMemberRecord_Set", maip, *records, 0, 0);

	if (!bitset_init(&sm_pool, &mcGroupVf, MAX_VFABRICS)) {
        IB_LOG_WARN0("sa_McMemberRecord_Set: insufficient memory to process request");
        IB_EXIT("sa_McMemberRecord_Set", VSTATUS_NOMEM);
        return(VSTATUS_NOMEM);
    }

	memset(&notice, 0, sizeof(notice));
	notice.Attributes.Generic.u.s.IsGeneric		= 1;
	notice.Attributes.Generic.u.s.Type	= NOTICE_TYPE_INFO;
	notice.Attributes.Generic.u.s.ProducerType		= NOTICE_PRODUCERTYPE_CLASSMANAGER;
	notice.IssuerLID	= sm_lid;
	notice.Stats.s.Toggle	= 0;
	notice.Stats.s.Count	= 0;

	if (maip->base.bversion == STL_BASE_VERSION)
	{
		//
		//  Verify the size of the data received for the request
		//
		if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_MCMEMBER_RECORD) ) {
			IB_LOG_ERROR_FMT(__func__,
				"invalid MAD length; size of STL_MCMEMBER_RECORD[%"PRISZT"], datasize[%d]",
				sizeof(STL_MCMEMBER_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_EXIT("sa_McMemberRecord_Set", MAD_STATUS_SA_REQ_INVALID);
			return (MAD_STATUS_SA_REQ_INVALID);
		}
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_MCMEMBER_RECORD));
		BSWAPCOPY_STL_MCMEMBER_RECORD((STL_MCMEMBER_RECORD*)samad.data, &mcMemberRecord);
	}
	else
	{
		// If it's IB, just convert the record to STL.
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(IB_MCMEMBER_RECORD));
		BSWAPCOPY_IB_MCMEMBER_RECORD((IB_MCMEMBER_RECORD*)samad.data, &temporary_rec);
		CONVERT_IB2STL_MCMEMBER_RECORD(&temporary_rec, &mcMemberRecord);
		samad.header.mask = REBUILD_COMPMSK_IB2STL_MCMEMBER_RECORD(samad.header.mask);
	}

	mcmp = &mcMemberRecord;

	mcmp->Reserved = 0;
	mcmp->Reserved2 = 0;
	mcmp->Reserved3 = 0;
	mcmp->Reserved4 = 0;
	mcmp->Reserved5 = 0;

	mGid[0] = mcmp->RID.MGID.Type.Global.SubnetPrefix;
	mGid[1] = mcmp->RID.MGID.Type.Global.InterfaceID;

	prefix = mcmp->RID.PortGID.Type.Global.SubnetPrefix;
	guid = mcmp->RID.PortGID.Type.Global.InterfaceID;

	// The locks are already held when we are passed a topology.
	if (!topop) {
		(void)vs_rdlock(&old_topology_lock); 
		(void)vs_lock(&sm_McGroups_lock);
		got_old_topology_lock = TRUE;
		topop = &old_topology;
	}

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	if (topop->node_head == NULL ||
        (!sm_valid_port((mcmPortp = sm_get_port(topop->node_head,sm_config.port))))) {
		maip->base.status = MAD_STATUS_BUSY;
        if (saDebugPerf) {
            if (topop->node_head == NULL) {
                IB_LOG_INFINI_INFO_FMT( "sa_McMemberRecord_Set", "Not done discovery, failing request from LID "
                       "0x%.8X for port GID "FMT_GID", Group "FMT_GID" with status 0x%.4X",
                       maip->addrInfo.slid, prefix, guid, mGid[0], mGid[1], maip->base.status);
            } else {
                IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Failed to get SM port %d, "
                       "failing request for port GID "FMT_GID", Group "FMT_GID" with "
                       "status 0x%.4X",
                       sm_config.port, prefix, guid, mGid[0], mGid[1], maip->base.status);
            }
        }
		goto done;
	}

	memcpy(notice.IssuerGID.Raw, mcmPortp->portData->gid, sizeof(notice.IssuerGID.Raw));

//
//	Find the requestors port.
//
	if ((req_portp = sm_find_active_port_lid(topop, maip->addrInfo.slid)) == NULL) {
		//maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
        // temp patch for Sun Oregon
        maip->base.status = activateInProgress ? 0x0700 : MAD_STATUS_SA_REQ_INVALID;
        if (saDebugPerf) {
		IB_LOG_INFINI_INFO_FMT( "sa_McMemberRecord_Set", 
               "Can not find requester's LID (0x%.8X) or not active in current topology, "
			   "failing request for port GID "FMT_GID", Group "FMT_GID" with "
			   "status 0x%.4X",
			   maip->addrInfo.slid, prefix, guid, mGid[0], mGid[1], maip->base.status);
        }
		goto done;
	}

	req_nodep = sm_find_port_node(topop, req_portp);
	req_nodeName = sm_nodeDescString(req_nodep);

	smCsmFormatNodeId(&csmReqNode, (uint8_t*)req_nodeName, req_portp->index, req_nodep->nodeInfo.NodeGUID);

	if (  ((neighbor_portp = sm_find_port(topop, req_portp->nodeno, req_portp->portno)) != NULL)
	   && sm_valid_port(neighbor_portp)) {
		smCsmFormatNodeId(&csmConnectedNode, (uint8_t*)sm_nodeDescString(neighbor_portp->portData->nodePtr),     
		                  neighbor_portp->index, (neighbor_portp->portData->nodePtr)->nodeInfo.NodeGUID);                
		csmNeighborp = &csmConnectedNode;
	}
	
//
//	Find the port in question.
//
	if (guid == 0x0ull) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
		IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Port GID in request ("FMT_GID") from "
			   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X has a NULL GUID, returning status 0x%.4X",
			   prefix, guid, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
		goto done;
	}

	if (prefix != sm_config.subnet_prefix) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
		IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Port GID in request ("FMT_GID") from "
			   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X has an invalid prefix, returning status 0x%.4X",
			   prefix, guid, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
		goto done;
	}

	if ((portp = sm_find_active_port_guid(topop, guid)) == NULL) {
		maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_INFINI_INFO_FMT( "sa_McMemberRecord_Set", 
               "Port GID in request ("FMT_GID") from %s Port %d, PortGUID "FMT_U64", "
			   "LID 0x%.8X, for group "FMT_GID" can't be found or not active in current topology, "
			   "returning status 0x%.4X",
			   prefix, guid, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, mGid[0], mGid[1], maip->base.status);
		goto done;
	}

//
//	Check the MTU to be sure that this port can receive packets.
//

	tempMask = samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_MTU;

	smGetVfMaxMtu(portp, req_portp, mcmp, &vfMaxMtu, &vfMaxRate);

	if (sm_mc_config.disable_mcast_check == McGroupBehaviorRelaxed) {
		maxMtu = portp->portData->maxVlMtu;
	} else {
		maxMtu = Min(portp->portData->maxVlMtu, topop->maxMcastMtu);
		if (vfMaxMtu) {
			maxMtu = Min(maxMtu, vfMaxMtu);
		}
	}

	if (tempMask == STL_MCMEMBER_COMPONENTMASK_OK_MTU) {
		switch (mcmp->MtuSelector) {
			case IB_SELECTOR_GT:
				if (maxMtu > mcmp->Mtu) {
					mtu = maxMtu;
				} else {
					mtu = getNextMTU(mcmp->Mtu);
				}
				break;
			case IB_SELECTOR_LT:
				if (maxMtu < mcmp->Mtu) {
					mtu = maxMtu;
				} else {
					mtu = getPrevMTU(mcmp->Mtu);
				}
				break;
			case IB_SELECTOR_EQ:
				mtu = mcmp->Mtu;
				break;
			case IB_SELECTOR_MAX:
				mtu = maxMtu;
				break;
			default:
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Bad MTU selector of %d for request from "
					   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcmp->MtuSelector, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
				
		}

		// TODO: Check MTU vals.
		// MTU can be less than port's but not greater
		if ((mtu > STL_MTU_MAX) || (mtu < IB_MTU_256) || (mtu > maxMtu)) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, &csmReqNode, csmNeighborp,
				"MTU selector of %d with MTU of %s does not work with realizable MTU of %s for "
				"request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				mcmp->MtuSelector, IbMTUToText(mcmp->Mtu), IbMTUToText(maxMtu), req_nodeName,
				req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}
	} else if (tempMask != 0) { /* Only one of the two bits set, only max allowed */
		if (tempMask != STL_MCMEMBER_COMPONENTMASK_MTU_SEL || mcmp->MtuSelector != IB_SELECTOR_MAX) {
			maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
			IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Only one of MTU selector (%d) and MTU "
				   "value (%d) set in component mask "FMT_U64" in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, "
				   "returning status 0x%.4X",
				   mcmp->MtuSelector, mcmp->Mtu, samad.header.mask, req_nodeName, req_portp->index, req_portp->portData->guid,
				   maip->addrInfo.slid, maip->base.status);
			goto done;
		}
		mtu = maxMtu; /* Only selector was on with value of max */
	} else {
		if (sm_mc_config.disable_mcast_check == McGroupBehaviorRelaxed) {
			// default to the minimum of 2k mtu or port's mtu
			mtu = Min(maxMtu, IB_MTU_2048);
		} else {
			// default to the minimum of the fabric and port's mtu
			mtu = maxMtu;
		}
	}

//
//	Check the rate to be sure that this port can support it.
//

	activeRate = linkWidthToRate(portp->portData);
	tempMask = samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_RATE;

	if (sm_mc_config.disable_mcast_check == McGroupBehaviorRelaxed) {
		maxRate = activeRate;
	} else {
		maxRate = linkrate_lt(activeRate, topop->maxMcastRate) ? activeRate : topop->maxMcastRate;
		if (vfMaxRate) {
			maxRate = linkrate_lt(maxRate, vfMaxRate) ? maxRate : vfMaxRate;
		}
	}

	if (tempMask == STL_MCMEMBER_COMPONENTMASK_OK_RATE) {

		switch (mcmp->RateSelector) {
			case IB_SELECTOR_GT:
				if (linkrate_gt(maxRate, mcmp->Rate)) {
					rate = maxRate;
				} else {
					// Will be out of range if we go above maxRate. Set to fail later check.
					rate = IB_STATIC_RATE_MAX+1;
				}
				break;
			case IB_SELECTOR_LT:
				if (linkrate_lt(maxRate, mcmp->Rate)) {
					rate = maxRate;
				} else {
					rate = rateOneStepSmaller(mcmp->Rate);
				}
				break;
			case IB_SELECTOR_EQ:
				rate = mcmp->Rate;
				break;
			case IB_SELECTOR_MAX:
				rate = maxRate;
				break;
			default:
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Bad rate selector of %d for request from "
					   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcmp->RateSelector, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
		}

		if ((rate > IB_STATIC_RATE_MAX) || (rate < IB_STATIC_RATE_MIN) || linkrate_gt(rate, maxRate)) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, &csmReqNode, csmNeighborp,
			                "Rate selector of %d with rate of %s does not work with realizable rate "
			                "of %s for request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
			                mcmp->RateSelector, StlStaticRateToText(mcmp->Rate), StlStaticRateToText(maxRate), (uint8_t*)req_nodeName,
			                req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}
	} else if (tempMask != 0) {
		if (tempMask != STL_MCMEMBER_COMPONENTMASK_RATE_SEL || mcmp->RateSelector != IB_SELECTOR_MAX) {
			maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
			IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Only one of rate selector (%d) and rate (%d) "
				   "set in component mask "FMT_U64" in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, "
				   "returning status 0x%.4X",
				   mcmp->RateSelector, mcmp->Rate, samad.header.mask, req_nodeName,
				   req_portp->index, req_portp->portData->guid,
				   maip->addrInfo.slid, maip->base.status);
			goto done;
		}
		rate = maxRate;  /* Only selector was on with value of max */
	} else {
		if (sm_mc_config.disable_mcast_check == McGroupBehaviorRelaxed) {
			rate = linkrate_lt(activeRate, IB_STATIC_RATE_200G)
			         ? activeRate : IB_STATIC_RATE_200G;
		} else {
			rate = maxRate;
		}
	}

//
//	Check the lifetime to be sure that this port can support it.
//
	if (samad.header.mask & STL_MCMEMBER_COMPONENTMASK_HOP) {
		hopLimit = mcmp->HopLimit;
		if (hopLimit == 0) {
			saLife = 0;
		} else if (!sa_dynamicPlt[0]) {
			saLife = sm_config.sa_packet_lifetime_n2;
		} else if (mcmp->HopLimit < 8) {
			saLife = sa_dynamicPlt[hopLimit];
		} else {
			saLife = sa_dynamicPlt[9];
		}
	} else {
		hopLimit = 0xFF; /* Default to max hops */
		if (!sa_dynamicPlt[0]) {
			saLife = sm_config.sa_packet_lifetime_n2;
		} else {
			saLife = sa_dynamicPlt[9];
		}
	}

	tempMask = samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_LIFE;
	if (tempMask == STL_MCMEMBER_COMPONENTMASK_OK_LIFE) {
		switch (mcmp->PktLifeTimeSelector) {
			case IB_SELECTOR_GT:
				if (saLife > mcmp->PktLifeTime) {
					life = saLife;
				} else {
					life = mcmp->PktLifeTime + 1;
				}
				break;
			case IB_SELECTOR_LT:
				if (saLife < mcmp->PktLifeTime) {
					life = saLife;
				} else {
					life = mcmp->PktLifeTime - 1;
				}
				break;
			case IB_SELECTOR_EQ:
				life = mcmp->PktLifeTime;
				break;
			case IB_SELECTOR_MAX:
				life = saLife;
				break;
			default:
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Bad Life Selector of %d for request from "
					   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcmp->PktLifeTimeSelector, req_nodeName, req_portp->index,
					   req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
		}
	
		// Do not allow lifetime less than system life time
		if (life < saLife || life > DYNAMIC_PLT_MAX) {
			IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Life selector of %d with life of %d does not "
				   "work with port sa life of %d for request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, "
				   "returning status 0x%.4X",
				   mcmp->PktLifeTimeSelector, mcmp->PktLifeTime, saLife, req_nodeName, req_portp->index, req_portp->portData->guid,
				   maip->addrInfo.slid, maip->base.status);
			goto done;
		}
	} else if (tempMask != 0) {
		if (tempMask != STL_MCMEMBER_COMPONENTMASK_LIFE_SEL || mcmp->PktLifeTimeSelector != IB_SELECTOR_MAX) {
			maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
			IB_LOG_ERROR_FMT( "sa_McMemberRecord_Set", "Only one of life selector (%d) and life (%d) "
				   "set in component mask "FMT_U64" in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, "
				   "returning status 0x%.4X",
				   mcmp->PktLifeTimeSelector, mcmp->PktLifeTime, samad.header.mask, req_nodeName, req_portp->index, req_portp->portData->guid,
				   maip->addrInfo.slid, maip->base.status);
			goto done;
		}

		life = saLife; /* Only selector was on with value of max */
	} else {
		life = saLife; /* Default to port's lifetime */
	}

//
//	Check the parameters.
//
	if (samad.header.mask & STL_MCMEMBER_COMPONENTMASK_PKEY) {
		vfp = smGetVfName(mcmp->P_Key);
        if (PKEY_TYPE(mcmp->P_Key) != PKEY_TYPE_FULL) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Bad (limited member) PKey of 0x%.4X for "
			       "request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				   mcmp->P_Key, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}
	}

	if (samad.header.mask & STL_MCMEMBER_COMPONENTMASK_QKEY) {
		qKey = mcmp->Q_Key;
	} else {
		qKey = 0;
	}

	if (STL_MCMRECORD_GETJOINSTATE(mcmp) == 0) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Join state of 0 for request from "
			   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
			   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
		goto done;
	}

    /* set the scope if specified */
    if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_SCOPE) == STL_MCMEMBER_COMPONENTMASK_SCOPE) {
        scope = mcmp->Scope;
    } else {
		scope = IB_LINK_LOCAL_SCOPE;
	}

//
//	Check that the MCGid is valid.
//
	if (!(samad.header.mask & STL_MCMEMBER_COMPONENTMASK_MGID) || 
        ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_MGID) && !memcmp(&mcmp->RID.MGID, nullGid.Raw, sizeof(IB_GID)))
        ) {
        /* no MGID - must be create or ERROR */
		if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_CREATE) != STL_MCMEMBER_COMPONENTMASK_OK_CREATE) {
			maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
			IB_LOG_ERROR_FMT_VF( vfp, __func__,
				"Component mask of "FMT_U64" does not have bits required ("FMT_U64") to CREATE a new group "
				"in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				samad.header.mask, (uint64)(STL_MCMEMBER_COMPONENTMASK_OK_CREATE), req_nodeName, req_portp->index,
				req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}

		if (!(mcmp->JoinFullMember)) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Join state of 0x%.2X not full member "
				   "for NULL GID CREATE request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				   STL_MCMRECORD_GETJOINSTATE(mcmp), req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}

		if (sm_numMcGroups == MAX_MCAST_MGIDS) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES; 
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Maximum number groups reached (%d), "
				   "failing CREATE request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				   MAX_MCAST_MGIDS, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}

		if (sm_multicast_gid_assign(scope, mcmp->RID.MGID) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES; 
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Failed to assign GID "
				   "for CREATE request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}

		mcmp->MtuSelector = IB_SELECTOR_EQ;
		mcmp->Mtu = mtu;
		mcmp->RateSelector = IB_SELECTOR_EQ;
		mcmp->Rate = rate;
		mcmp->PktLifeTimeSelector = IB_SELECTOR_EQ;
		mcmp->PktLifeTime = life;
		mcmp->Q_Key = qKey;
		mcmp->HopLimit = hopLimit;
		mcmp->Scope = scope;

		if (sm_config.sm_debug_vf) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "sa_McMemberRecord_Set",
				"MC group create request for "FMT_U64" from node %s Port %d, sl= %d, pkey=0x%x, mgid= "FMT_GID,
				guid, req_nodeName, req_portp->index, mcmp->SL, mcmp->P_Key, mGid[0], mGid[1]);
		}


		if (VirtualFabrics != NULL && VirtualFabrics->securityEnabled &&
		    (status = smVFValidateMcGrpCreateParams(portp, portp->portData->guid != req_portp->portData->guid ?
		                                       req_portp : NULL, mcmp, &mcGroupVf)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "MC group create request denied for PortGUID "
			       FMT_U64" from %s Port %d, PortGUID "FMT_U64", LID 0x%08X, failed VF validation (mgid= "FMT_GID", sl= %d, pkey= 0x%x)",
				   guid, req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, mGid[0], mGid[1], mcmp->SL, mcmp->P_Key);
			goto done;
		}

		if ((status = sm_multicast_assign_lid(mcmp->RID.MGID, mcmp->Mtu, mcmp->Rate,
		                                      mcmp->P_Key, &mLid)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "No multicast LIDs available for CREATE request from "
				   "%s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
				   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
			goto done;
		}

		mcmp->MLID = mLid;

		McGroup_Create(mcGroup);
		createdGroup = 1;
		memcpy(&(mcGroup->mGid), &mcmp->RID.MGID, 16);
		mcGroup->mLid = mcmp->MLID;
		mcGroup->qKey = mcmp->Q_Key;
		mcGroup->pKey = mcmp->P_Key;
		mcGroup->mtu = mcmp->Mtu;
		mcGroup->rate = mcmp->Rate;
		mcGroup->life = mcmp->PktLifeTime;
		mcGroup->sl = mcmp->SL;
		mcGroup->tClass = mcmp->TClass;
		mcGroup->hopLimit = mcmp->HopLimit;
		mcGroup->scope = mcmp->Scope;
		bitset_copy(&mcGroup->vfMembers, &mcGroupVf);
		mcGroup->members_full++;

		McMember_Create(mcGroup, mcMember);
		mcMember->record = *mcmp;
		mcMember->portGuid = guid;
		mcMember->slid = maip->addrInfo.slid;
		mcMember->state = STL_MCMRECORD_GETJOINSTATE(mcmp); // FIXME: Pick a method for unionizing join state
		mcMember->proxy = (maip->addrInfo.slid == portp->portData->lid) ? 0 : 1; // JSY - check LMC aliasing

		status = VSTATUS_OK;
	} else {
        //
        // if MGID specified does not exist in group table
        // we will create if this is a create request.
        // It's an error if a Join and the MGID does not exist
        //
		if ((mcGroup = sm_find_multicast_gid(mcmp->RID.MGID)) == NULL) {
			//
			// Non-NULL MGID specified - validate the MGID
			//
			if ((status = sm_multicast_gid_valid(scope, mcmp->RID.MGID)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", 
				       "Invalid MGID ("FMT_GID") in CREATE/JOIN request from %s Port %d, PortGUID "FMT_U64", "
				       "LID 0x%.8X, returning status 0x%.4X",
				       mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid,
			           maip->addrInfo.slid, maip->base.status);
				goto done;
			}

            /* MGID does not exist; must be a CREATE request or its an ERROR */
            if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_CREATE) != STL_MCMEMBER_COMPONENTMASK_OK_CREATE) {

                if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_JOIN) == STL_MCMEMBER_COMPONENTMASK_OK_JOIN) {
                    maip->base.status = MAD_STATUS_SA_REQ_INVALID;
                    IB_LOG_ERROR_FMT_VF(  vfp,"sa_McMemberRecord_Set", 
                           "MGID "FMT_GID" does not exist; Failing JOIN "
                           "request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
                           mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
                    goto done;
                } else {
                    maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
                    IB_LOG_ERROR_FMT_VF( vfp, __func__,
						"Component mask ("FMT_U64") does not have bits required to create ("FMT_U64") a group for new MGID"
						" of "FMT_GID" for request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
						samad.header.mask, (uint64)(STL_MCMEMBER_COMPONENTMASK_OK_CREATE), mGid[0], mGid[1], req_nodeName,
						req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
                    goto done;
                }
			}

			if (!(mcmp->JoinFullMember)) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Join state of 0x%.2X not full member "
					   "for NEW MGID of "FMT_GID" for CREATE request from %s Port %d, PortGUID "FMT_U64", "
					   "LID 0x%.8X, returning status 0x%.4X",
					   STL_MCMRECORD_GETJOINSTATE(mcmp), mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid,
					   maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			if (sm_numMcGroups == MAX_MCAST_MGIDS) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES; 
				IB_LOG_ERROR_FMT_VF(  vfp,"sa_McMemberRecord_Set", "Maximum number groups reached (%d) "
					   "for new MGID of "FMT_GID" for CREATE request from %s Port %d, PortGUID "FMT_U64", "
					   "LID 0x%.8X, returning status 0x%.4X",
					   MAX_MCAST_MGIDS, mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid,
					   maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			mcmp->MtuSelector = IB_SELECTOR_EQ;
			mcmp->Mtu = mtu;
			mcmp->RateSelector = IB_SELECTOR_EQ;
			mcmp->Rate = rate;
			mcmp->PktLifeTimeSelector = IB_SELECTOR_EQ;
			mcmp->PktLifeTime = life;
			mcmp->Q_Key = qKey;
			mcmp->HopLimit = hopLimit;
			mcmp->Scope = scope;

			if (sm_config.sm_debug_vf) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "sa_McMemberRecord_Set",
					"MC group create request for node %s Port %d, sl= %d, pkey=0x%x, mgid= "FMT_GID,
					req_nodeName, portp->index, mcmp->SL, mcmp->P_Key, mGid[0], mGid[1]);
			}

			if (VirtualFabrics != NULL && VirtualFabrics->securityEnabled &&
			    (status = smVFValidateMcGrpCreateParams(portp,
								portp->portData->guid != req_portp->portData->guid ?  req_portp : NULL,
								mcmp, &mcGroupVf)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF(  vfp,"sa_McMemberRecord_Set", "MC group create request denied for PortGUID "
			       		FMT_U64" from %s Port %d, PortGUID "FMT_U64", LID 0x%08X, failed VF validation (mgid= "FMT_GID", sl= %d, pkey= 0x%x)",
				   		portp->portData->guid,
				   		req_nodeName, req_portp->index, req_portp->portData->guid,
				   		maip->addrInfo.slid, mGid[0], mGid[1], mcmp->SL, mcmp->P_Key);
				goto done;
			}

			if ((status = sm_multicast_assign_lid(mcmp->RID.MGID, mcmp->P_Key,
			                                      mcmp->Mtu, mcmp->Rate, &mLid)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "No multicast LIDs available for CREATE "
				       "request of group with MGID "FMT_GID" from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			mcmp->MLID = mLid;

			McGroup_Create(mcGroup);
			createdGroup = 1;
			memcpy(&(mcGroup->mGid), &mcmp->RID.MGID, 16);
			mcGroup->mLid = mcmp->MLID;
			mcGroup->qKey = mcmp->Q_Key;
			mcGroup->pKey = mcmp->P_Key;
			mcGroup->mtu = mcmp->Mtu;
			mcGroup->rate = mcmp->Rate;
			mcGroup->life = mcmp->PktLifeTime;
			mcGroup->sl = mcmp->SL;
			mcGroup->tClass = mcmp->TClass;
			mcGroup->hopLimit = hopLimit;
			mcGroup->scope = mcmp->Scope;
			bitset_copy(&mcGroup->vfMembers, &mcGroupVf);
			mcGroup->members_full++;

			McMember_Create(mcGroup, mcMember);
			mcMember->record = *mcmp;
			mcMember->portGuid = guid;
			mcMember->slid = maip->addrInfo.slid;
			mcMember->state = STL_MCMRECORD_GETJOINSTATE(mcmp);
			mcMember->proxy = (maip->addrInfo.slid == portp->portData->lid) ? 0 : 1;// JSY - check LMC aliasing
		} else {
            /*
             * Valid MGID specified is in group table already
             * see if we can join requester
             */
			if (!(samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_JOIN)) {
                /* check for join first */
				maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
				IB_LOG_ERROR_FMT_VF( vfp, __func__,
					"Component mask of "FMT_U64" does not have bits required ("FMT_U64") to JOIN group with"
					" MGID "FMT_GID" in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					samad.header.mask, (uint64)(STL_MCMEMBER_COMPONENTMASK_OK_JOIN), mGid[0], mGid[1], req_nodeName,
					req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			}
            /* validate all supplied fields before doing mcMember join */
			if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_QKEY) && (mcmp->Q_Key != mcGroup->qKey))	{
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "QKey of 0x%.8X does not match "
					   "group QKey of 0x%.8X for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcmp->Q_Key, mcGroup->qKey, mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid,
					   maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_PKEY) && (mcmp->P_Key != mcGroup->pKey)) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "PKey of 0x%.4X does not match "
					   "group PKey of 0x%.4X for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcmp->P_Key, mcGroup->pKey, mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid,
					   maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_SL) && (mcmp->SL != mcGroup->sl)) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "SL of %d does not match "
					   "group SL of %d for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcmp->SL, mcGroup->sl, mGid[0], mGid[1], req_nodeName, req_portp->index, req_portp->portData->guid,
					   maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			if (!(samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_MTU)) { // Wildcarded, port's MTU must be >= group's
                if (portp->portData->maxVlMtu < mcGroup->mtu) {
                    maip->base.status = MAD_STATUS_SA_REQ_INVALID;
                    IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", 
                           "Group MTU of %d greater than requester port mtu of %d "
                           "for group "FMT_GID" in request from %s Port %d, "
                           "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
                           mcGroup->mtu, portp->portData->maxVlMtu, mGid[0], mGid[1],
                           req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
                    goto done;
                }
			} else if (mcGroup->mtu < mtu && mcmp->MtuSelector != IB_SELECTOR_LT) { // They didn't choose less than so can't drop down
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group MTU of %d is too low "
					   "for requested MTU of %d, MTU selector of %d, and port MTU of %d "
					   "for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcGroup->mtu, mcmp->Mtu, mcmp->MtuSelector, portp->portData->maxVlMtu, mGid[0], mGid[1],
					   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			} else if (mcGroup->mtu > mtu && mcmp->MtuSelector != IB_SELECTOR_GT &&
					   mcmp->MtuSelector != IB_SELECTOR_MAX) { // They didn't choose greater than so can't rise up
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group MTU of %d is too high "
					   "for requested MTU of %d, MTU selector of %d, and port MTU of %d "
					   "for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcGroup->mtu, mcmp->Mtu, mcmp->MtuSelector, portp->portData->maxVlMtu, mGid[0], mGid[1], req_nodeName,
					   req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			} else if (mcGroup->mtu > portp->portData->maxVlMtu) { // Group's MTU is higher than port's
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group MTU of 0x%.2X is too high "
					   "for requested MTU of %d, MTU selector of %d, and port MTU of %d "
					   "for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   mcGroup->mtu, mcmp->Mtu, mcmp->MtuSelector, portp->portData->maxVlMtu, mGid[0], mGid[1], req_nodeName,
					   req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			}
			
			
			if (!(samad.header.mask & STL_MCMEMBER_COMPONENTMASK_OK_RATE)) { 
                /* Wildcarded, port's rate can be greater than or equal group's */
                if (linkrate_lt(activeRate, mcGroup->rate)) {
                    maip->base.status = MAD_STATUS_SA_REQ_INVALID;
                    IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group Rate of %s greater than "
                           "requester port rate of %s for group "FMT_GID" in request from %s Port %d, "
                           "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
                           StlStaticRateToText(mcGroup->rate),
						   StlStaticRateToText(activeRate), mGid[0], mGid[1],
                           req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
                    goto done;
                }
			} else if (linkrate_lt(mcGroup->rate, rate) && mcmp->RateSelector != IB_SELECTOR_LT) { // They didn't choose less than so can't drop down
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group Rate of %s is too low "
					   "for requested rate of %s, rate selector of %d, and port rate of %d "
					   "for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   StlStaticRateToText(mcGroup->rate),
					   StlStaticRateToText(mcmp->Rate), mcmp->RateSelector,
					   activeRate, mGid[0], mGid[1],
					   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;

			} else if (linkrate_gt(mcGroup->rate, rate) && mcmp->RateSelector != IB_SELECTOR_GT &&
					   mcmp->RateSelector != IB_SELECTOR_MAX) { // They didn't choose greater than so can't rise up
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group Rate of %s is too high "
					   "for requested rate of %s, rate selector of %d, and port rate of %d "
					   "for group "FMT_GID" in request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   StlStaticRateToText(mcGroup->rate),
					   StlStaticRateToText(mcmp->Rate),
					   mcmp->RateSelector, activeRate, mGid[0], mGid[1],
					   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			} else if (linkrate_gt(mcGroup->rate, activeRate)) { // Group's rate is not equal to port's
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Set", "Group Rate of %s is too high "
					   "for requested rate of %s, rate selector of %d, and port rate of %d "
					   "for group "FMT_GID" for request from %s Port %d, "
					   "PortGUID "FMT_U64", LID 0x%.8X, returning status 0x%.4X",
					   StlStaticRateToText(mcGroup->rate),
					   StlStaticRateToText(mcmp->Rate),
					   mcmp->RateSelector, activeRate, mGid[0], mGid[1],
					   req_nodeName, req_portp->index, req_portp->portData->guid, maip->addrInfo.slid, maip->base.status);
				goto done;
			}

			if (VirtualFabrics != NULL && VirtualFabrics->securityEnabled &&
			   bitset_test_intersection(&mcGroup->vfMembers, &req_portp->portData->vfMember) == 0) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, __func__, "Failing group join request for PortGUID "FMT_U64 
				       " from %s Port %d, PortGUID "FMT_U64", LID 0x%08X because it does not share a virtual fabric with MGid "FMT_GID,
					   portp->portData->guid, 
					   req_nodeName, req_portp->index, req_portp->portData->guid, 
					   maip->addrInfo.slid, mGid[0], mGid[1]);
				goto done;
			}

			if (VirtualFabrics != NULL && VirtualFabrics->securityEnabled &&
			   bitset_test_intersection(&mcGroup->vfMembers, &req_portp->portData->fullPKeyMember) == 0) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				IB_LOG_ERROR_FMT_VF( vfp, __func__, "Failing group join request for PortGUID "FMT_U64 
				       " from %s Port %d, PortGUID "FMT_U64", LID 0x%08X because it is not a full member of virtual fabric with MGid "FMT_GID,
					   portp->portData->guid, 
					   req_nodeName, req_portp->index, req_portp->portData->guid, 
					   maip->addrInfo.slid, mGid[0], mGid[1]);
				goto done;
			}

			mcmp->MLID = mcGroup->mLid;
			mcmp->MtuSelector = IB_SELECTOR_EQ;
			mcmp->Mtu = mcGroup->mtu;
			mcmp->RateSelector = IB_SELECTOR_EQ;
			mcmp->Rate = mcGroup->rate;
			mcmp->PktLifeTimeSelector = IB_SELECTOR_EQ;
			mcmp->PktLifeTime = mcGroup->life;
			mcmp->P_Key = mcGroup->pKey;
			mcmp->Q_Key = mcGroup->qKey;
			mcmp->SL = mcGroup->sl;
			mcmp->HopLimit = mcGroup->hopLimit;

			if (!(mcMember = sm_find_multicast_member(mcGroup, mcmp->RID.PortGID))) {
				McMember_Create(mcGroup, mcMember);
				if (mcmp->JoinFullMember) {
					mcGroup->members_full++;
				}
                /* new member: sync the group change with standby SMs */
                (void)sm_dbsync_syncGroup(DBSYNC_TYPE_UPDATE, &mcmp->RID.MGID);
			} else {
				mcmp->JoinFullMember |= mcMember->state & MCMEMBER_STATE_FULL_MEMBER;
				mcmp->JoinNonMember |= mcMember->state & MCMEMBER_STATE_NON_MEMBER;
				mcmp->JoinSendOnlyMember |= mcMember->state & MCMEMBER_STATE_SENDONLY_MEMBER;
				if (!(mcMember->state & MCMEMBER_STATE_FULL_MEMBER)) { /* If we were not a full member */
					if (mcmp->JoinFullMember) {	/* but are a full member now */
						mcGroup->members_full++;
					}
				}
                if (mcMember->state != STL_MCMRECORD_GETJOINSTATE(mcmp)) {
                    /* member change: sync the group change with standby SMs */
                    (void)sm_dbsync_syncGroup(DBSYNC_TYPE_UPDATE, &mcmp->RID.MGID);
                }
			}
			/* don't change state if trigger already set, but
			 * don't trigger sweep if joinstate has no change */
            newJoinState = (mcMember->state == STL_MCMRECORD_GETJOINSTATE(mcmp)) ? 0 : 1;
			mcMember->record = *mcmp; 
			mcMember->portGuid = guid;
			mcMember->slid = maip->addrInfo.slid;
			mcMember->state = STL_MCMRECORD_GETJOINSTATE(mcmp);
			mcMember->proxy = (maip->addrInfo.slid == portp->portData->lid) ? 0 : 1;// JSY - check LMC aliasing
		}
	}


done:

//
//	Update the fabric MFTs for this group.
//
	if (maip->base.status == MAD_STATUS_OK) {

		/* Previously we would try to setup the MFT inline with a call to sm_setup_multicast_group. 
		 * This call could take a long time as it talks to each switch chip in the fabric and
		 * blocks the SA. Instead, we will tigger a sweep which will pick up the group changes.
		 */
	
		/* If not already triggered,
		 * trigger sm_top to sweep and reprogram the switch MFTs */
		if (!sa_mft_reprog) {
			sa_mft_reprog = newJoinState; 
		}
	
		/* Note the results */
		*records = 1;
		if (maip->base.bversion == STL_BASE_VERSION)
		{
		BSWAPCOPY_STL_MCMEMBER_RECORD(mcmp, (STL_MCMEMBER_RECORD*)sa_data);
		}
		else
		{
			CONVERT_STL2IB_MCMEMBER_RECORD(mcmp, (IB_MCMEMBER_RECORD*)sa_data);
			BSWAP_IB_MCMEMBER_RECORD((IB_MCMEMBER_RECORD*)sa_data);
		}

	
		if (createdGroup) {
            /* sync the group creation with standby SMs */
            (void)sm_dbsync_syncGroup(DBSYNC_TYPE_UPDATE, &mcmp->RID.MGID);
			//	The group has been created, so send trap to subscribers	
			trap66->Attributes.Generic.TrapNumber = MAD_SMT_MCAST_GRP_CREATED;
			memcpy(trap66->IssuerGID.Raw, mcGroup->mGid.Raw, sizeof(trap66->IssuerGID.Raw));
			status = sm_sa_forward_trap(&notice);
		}
	} else if (status == VSTATUS_OK) {
		status = VSTATUS_BAD;
	}

#if DUMP_MCGROUPS
	if (saDebugPerf) {
		McGroup_t	*dGroup;

		for_all_multicast_groups(dGroup) {
			IB_LOG_INFINI_INFO_FMT( "McGroup mgid",""FMT_GID" (%04x)",
				dGroup->mGid.Type.Global.SubnetPrefix,
				dGroup->mGid.Type.Global.InterfaceID,
				dGroup->mLid); 

			IB_LOG_INFINI_INFO_FMT( "McGroup:","\tqKey = 0x%.8X  pKey = 0x%.4X  mtu = %d  rate = %d  life = %d  sl = %d",
				(int)dGroup->qKey, dGroup->pKey, dGroup->mtu, dGroup->rate,
				dGroup->life, dGroup->sl);

			bitset_info_log(&dGroup->vfMembers, "McGroup vfMembers");
		}
	}
#endif

	if (got_old_topology_lock) {
		vs_rwunlock(&old_topology_lock); 
		vs_unlock(&sm_McGroups_lock);
	}

	bitset_free(&mcGroupVf);

	IB_EXIT("sa_McMemberRecord_Set", status);
	return(status);
}

//
// Deletes a node from a Multicast Group, deleting the group if necessary.
// Note:
// 1. The primary use of this function is to process incoming MADs but it is
//    also used internally within the SM.
// 2. The main code path works on the "old" topology and the global list of 
//    McGroups. If a different topology pointer is passed to this function as
//    an argument, this function will still alter the global McGroups list. In
//    addition, when a topology pointer is passed in, this function assumes the
//    caller already holds any required locks.
//
Status_t
sa_McMemberRecord_Delete(Topology_t *topop, Mai_t *maip, uint32_t *records)
{
	Guid_t			guid;
	Guid_t			prefix;
	Guid_t			mcastGid[2];
	Port_t			*portp, *mcmPortp;
	Port_t 			*senderPort;
	Node_t 			*senderNode;
	char 			*senderName = "Unknown node";
	Guid_t			senderGuid = 0;
	STL_SA_MAD			samad;
	Status_t		status=VSTATUS_OK;
	McGroup_t		*mcGroup;
	McMember_t		*mcMember;
	STL_MCMEMBER_RECORD	mcMemberRecord = {{{{0}}}};
	STL_MCMEMBER_RECORD	*mcmp;
	STL_NOTICE		notice;
	STL_NOTICE		*trap67	= (STL_NOTICE *)&notice;
	char			*nodeName = 0;
	Node_t			*nodep = 0;
	uint8_t         joinstate=0;
	char			*vfp=NULL;
	boolean			got_old_topology_lock = FALSE;
	boolean			got_mcgroups_lock = FALSE;

	IB_ENTER("sa_McMemberRecord_Delete", maip, *records, 0, 0);

	memset(&notice, 0, sizeof(notice));
	notice.Attributes.Generic.u.s.IsGeneric		= 1;
	notice.Attributes.Generic.u.s.Type	= NOTICE_TYPE_INFO;
	notice.Attributes.Generic.u.s.ProducerType		= NOTICE_PRODUCERTYPE_CLASSMANAGER;
	notice.IssuerLID	= sm_lid;
	notice.Stats.s.Toggle	= 0;
	notice.Stats.s.Count	= 0;

	if (!topop) {
		topop = &old_topology;
		(void)vs_rdlock(&old_topology_lock);
		got_old_topology_lock = TRUE;
	}

	if (topop->node_head == NULL ||
        (!sm_valid_port((mcmPortp = sm_get_port(topop->node_head,sm_config.port))))) {
		maip->base.status = MAD_STATUS_BUSY;
		status = VSTATUS_BAD;
		goto done;
	}

	memcpy(notice.IssuerGID.Raw, mcmPortp->portData->gid, sizeof(notice.IssuerGID.Raw));


	if (maip->base.bversion == STL_BASE_VERSION)
	{
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_MCMEMBER_RECORD));
		BSWAPCOPY_STL_MCMEMBER_RECORD((STL_MCMEMBER_RECORD*)samad.data, &mcMemberRecord);
	}
	else
	{
		// If it's IB, just convert the record to STL.
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(IB_MCMEMBER_RECORD));
		CONVERT_IB2STL_MCMEMBER_RECORD((IB_MCMEMBER_RECORD*)samad.data, &mcMemberRecord);
		BSWAP_STL_MCMEMBER_RECORD(&mcMemberRecord);
		samad.header.mask = REBUILD_COMPMSK_IB2STL_MCMEMBER_RECORD(samad.header.mask);
	}

	mcmp = (STL_MCMEMBER_RECORD *)&mcMemberRecord;
	mcmp->Reserved = 0;
	mcmp->Reserved2 = 0;
	mcmp->Reserved3 = 0;
	mcmp->Reserved4 = 0;
	mcmp->Reserved5 = 0;

	/* setup return data if successfull */
	*records = 1;
	BSWAPCOPY_STL_MCMEMBER_RECORD(mcmp, (STL_MCMEMBER_RECORD*)sa_data);
	//FIXME: This BSWAPCOPY needs to take into account IB cases. (eekahn)

	guid = 0ull;
	prefix = 0ull;

	prefix = mcmp->RID.PortGID.Type.Global.SubnetPrefix;
	guid = mcmp->RID.PortGID.Type.Global.InterfaceID;

	// Get requestor info
	if ((senderPort = sm_find_active_port_lid(topop, maip->addrInfo.slid))) {
		senderGuid = senderPort->portData->guid;
		if ((senderNode = sm_find_port_node(topop, senderPort))) {
			senderName = sm_nodeDescString(senderNode);
		}
	} else {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_INFINI_INFO_FMT(__func__,
			"Can not find requester's LID (0x%.8X) or not active in current topology, failing request for port GID "FMT_GID,
			maip->addrInfo.slid, prefix, guid);
		goto done;
	}

	if (guid == 0x0ull) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
		status = VSTATUS_BAD;
		IB_LOG_ERROR_FMT( "sa_McMemberRecord_Delete", "PortGUID of 0 in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X",
				   senderName, senderPort->index, senderGuid,
				   maip->addrInfo.slid);
		goto done;
	}

	if (prefix != sm_config.subnet_prefix) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
		status = VSTATUS_BAD;
		IB_LOG_ERROR_FMT( "sa_McMemberRecord_Delete", "Bad port GID prefix ("FMT_U64") in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X",
				   prefix, senderName, senderPort->index, senderGuid,
				   maip->addrInfo.slid);
		goto done;
	}

	if ((portp = sm_find_active_port_guid(topop, guid)) == NULL) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_INFINI_INFO_FMT( "sa_McMemberRecord_Delete", "Port not active in current topology GUID "FMT_U64" in request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X",
				   guid, senderName, senderPort->index, senderGuid,
				   maip->addrInfo.slid);
		status = VSTATUS_BAD;
		goto done;
	}

	if ((nodep = sm_find_port_node(&old_topology, portp))!=0) {
		nodeName = sm_nodeDescString(nodep); 
	}

	if (samad.header.mask & STL_MCMEMBER_COMPONENTMASK_PKEY) {
		vfp = smGetVfName(mcmp->P_Key);
	}

//
//	Find the mcMember.
//
	if (got_old_topology_lock) {
    	(void)vs_lock(&sm_McGroups_lock);
		got_mcgroups_lock = TRUE;
	}
	mcastGid[0] = mcmp->RID.MGID.Type.Global.SubnetPrefix;
	mcastGid[1] = mcmp->RID.MGID.Type.Global.InterfaceID;
	if ((mcGroup = sm_find_multicast_gid(mcmp->RID.MGID)) == NULL) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
		IB_LOG_VERBOSE_FMT_VF( vfp, "sa_McMemberRecord_Delete",
			   "Cound not find multicast GID of "FMT_GID" in list of active multicast GIDs,"
			   " for request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X",
			   mcastGid[0], mcastGid[1], senderName, senderPort->index, senderGuid,
			   maip->addrInfo.slid);
		status = VSTATUS_BAD;
		goto done;
	}
	if ((mcMember = sm_find_multicast_member(mcGroup, mcmp->RID.PortGID)) == NULL) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
		IB_LOG_VERBOSE_FMT_VF( vfp, "sa_McMemberRecord_Delete",
			   "Cound not find multicast member with port GID of "FMT_GID" in list "
			   "for multicast group with GID "FMT_GID
			   " for request from %s Port %d, PortGUID "FMT_U64", LID 0x%.8X",
			   prefix, guid, mcastGid[0], mcastGid[1], senderName, senderPort->index, senderGuid,
			   maip->addrInfo.slid);
		status = VSTATUS_BAD;
		goto done;
	}

//
//	Check the joinState.
//	It must have at least one bit set that was set in the old
//	It must not have any bits set that were not set in the old
//
	if (((mcMember->state & STL_MCMRECORD_GETJOINSTATE(mcmp)) == 0)              ||
		((mcMember->state | STL_MCMRECORD_GETJOINSTATE(mcmp)) != mcMember->state)   ) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Delete", "New join state of 0x%.2X does not work "
			   "with old join state of 0x%.2X for request for %s Port %d, PortGUID "FMT_U64", multicast group "
			   "GID "FMT_GID,
			   STL_MCMRECORD_GETJOINSTATE(mcmp), mcMember->state, nodeName, portp->index, guid, mcastGid[0], mcastGid[1]);
		status = VSTATUS_BAD;
		goto done;
	}

//
//	Check to see if this port can delete the McMember.
//
	if (mcMember->proxy == 0) {
		if (portp->portData->lid != maip->addrInfo.slid) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_ERROR_FMT_VF( vfp, __func__,
				"Lid 0x%.8X, %s Port %d, PortGUID "FMT_U64" can not delete record set by "
				"LID 0x%.8X, %s Port %d, PortGUID "FMT_U64", multicast group GID "FMT_GID,
				maip->addrInfo.slid, senderName, senderPort->index, senderGuid,
				portp->portData->lid, nodeName, portp->index, guid, mcastGid[0], mcastGid[1]);
			status = VSTATUS_BAD;
			goto done;
		}
	} else {
		if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_PKEY) && mcmp->P_Key != mcMember->record.P_Key) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID; // invalid pkey error code?
			IB_LOG_ERROR_FMT_VF( vfp, "sa_McMemberRecord_Delete", "Sender gave bad P_Key of 0x%.4X while "
				   "record had P_Key of 0x%.4X on request for %s, GUID "FMT_U64", multicast group "
				   "GID "FMT_GID
				   " from %s Port %d, PortGUID "FMT_U64" LID 0x%.8X",
				   mcmp->P_Key, mcMember->record.P_Key, nodeName, guid, mcastGid[0], mcastGid[1],
				   senderName, senderPort->index, senderGuid, maip->addrInfo.slid);
			status = VSTATUS_BAD;
			goto done;
		}

	}

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	if (  VirtualFabrics != NULL && VirtualFabrics->securityEnabled && senderPort!=NULL
	   && (bitset_test_intersection(&mcGroup->vfMembers, &senderPort->portData->vfMember) == 0) 
	   && (bitset_test_intersection(&mcGroup->vfMembers, &portp->portData->vfMember) == 0) ) {

		IB_LOG_ERROR_FMT_VF( vfp, __func__,
			"Sender gave bad P_Key of 0x%.4X while record had P_Key of 0x%.4X on request for %s, GUID "FMT_U64
			", multicast group GID "FMT_GID" from %s Port %d, PortGUID "FMT_U64" LID 0x%.8X",
			mcmp->P_Key, mcMember->record.P_Key, nodeName, guid, mcastGid[0], mcastGid[1],
			senderName, senderPort->index, senderGuid, maip->addrInfo.slid);
		status = VSTATUS_BAD;
		goto done;
	}

	joinstate = mcMember->state & ~STL_MCMRECORD_GETJOINSTATE(mcmp);
	if (joinstate == 0) {
		//	Delete the multicast member and decrement full member count if needed
		if (mcMember->state & MCMEMBER_STATE_FULL_MEMBER) {
			IB_LOG_VERBOSE_FMT_VF( vfp, "sa_McMemberRecord_Delete", "full mcMember "FMT_U64" left multicast group "
				   "GID "FMT_GID,
				   mcMember->portGuid, mcastGid[0], mcastGid[1]);
			mcGroup->members_full--;
		} else {
			IB_LOG_VERBOSE_FMT_VF( vfp, "sa_McMemberRecord_Delete", "mcMember "FMT_U64" left multicast group "
				   "GID "FMT_GID,
				   mcMember->portGuid, mcastGid[0], mcastGid[1]);
		}
		McMember_Delete(mcGroup, mcMember);
	} else {
        /* decrement the number of full members in group if member is removing full membership */
		if (mcMember->state & MCMEMBER_STATE_FULL_MEMBER  && 
            mcmp->JoinFullMember) {
			IB_LOG_VERBOSE_FMT_VF( vfp, "sa_McMemberRecord_Delete", 
                   "full mcMember "FMT_U64" downgrading membership in multicast group "
				   "GID "FMT_GID,
				   mcMember->portGuid, mcastGid[0], mcastGid[1]);
			mcGroup->members_full--;
        }
		mcMember->state = joinstate;
	}

	/* If there are not any full members left, delete all the others
	 * so that we eventually delete the group
	 */
	if (mcGroup->members_full == 0) {
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "sa_McMemberRecord_Delete", "Last full member left multicast group "
			   "GID "FMT_GID", deleting group and all members", mcastGid[0], mcastGid[1]);

		sa_updateMcDeleteCountForPort(portp);

		while (mcGroup->mcMembers) {
			/* MUST copy head pointer into temp
			 * Passing mcGroup->members directly to the delete macro will corrupt the list
			 */
			mcMember = mcGroup->mcMembers;
            if (saDebugPerf) {
                IB_LOG_INFINI_INFO_FMT_VF( vfp, "sa_McMemberRecord_Delete", "Deleting non full mcMember "FMT_U64,
                       mcMember->portGuid);
            }
			McMember_Delete(mcGroup, mcMember);
		}
	}


//
//	Update the MFTs.
//


	/* Previously we would try to setup the MFT inline with a call to sm_setup_multicast_group. 
	 * This call could take a long time as it talks to each switch chip in the fabric and
	 * blocks the SA. Instead, we will tigger a sweep which will pick up the group changes.
	 */

	/* Trigger sm_top to sweep and reprogram the switch MFTs */
	sa_mft_reprog = 1;

//
//	If the group doesn't have any members, then delete it.
//
	if (mcGroup->mcMembers == NULL) {
        /* sync the group deletion with standby SMs */
        (void)sm_dbsync_syncGroup(DBSYNC_TYPE_DELETE, &mcmp->RID.MGID);
		memcpy(trap67->IssuerGID.Raw, mcGroup->mGid.Raw, sizeof(trap67->IssuerGID.Raw));
		McGroup_Delete(mcGroup);
		//
		//	The group has been deleted, so send trap to subscribers
		//
		trap67->Attributes.Generic.TrapNumber = MAD_SMT_MCAST_GRP_DELETED;
		status = sm_sa_forward_trap(&notice);
	} else {
        /* sync the group change with standby SMs */
        (void)sm_dbsync_syncGroup(DBSYNC_TYPE_UPDATE, &mcmp->RID.MGID);
    }

done:
	if (got_mcgroups_lock) {
    	(void)vs_unlock(&sm_McGroups_lock);
	}
	if (got_old_topology_lock) {
    	(void)vs_rwunlock(&old_topology_lock);
	}

	IB_EXIT("sa_McMemberRecord_Delete", status);
	return(status);
}

#define STL_MC_REMOVE_MTU_RATE_LIFE_MASK    0x0000000000000F30ull 
Status_t
sa_McMemberRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t			*data;
	uint32_t		bytes;
	STL_SA_MAD		samad;
	Status_t		status;
	McGroup_t		*mcgp;
	McMember_t		*mcmp;
	STL_MCMEMBER_RECORD	mcMemQuery;
//	McMemberRecord_t	mcMemTemp;
	STL_MCMEMBER_RECORD	mcMemTemp = {{{{0}}}};
	uint64_t		mask;
	Port_t 			*senderPort;

	IB_ENTER("sa_McMemberRecord_GetTable", maip, *records, 0, 0);

	mcgp = NULL;		// JSY - compiler is whining during development
	mcmp = NULL;		// JSY - compiler is whining during development

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_MCMEMBER_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_MCMEMBER_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_MCMEMBER_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_MCMEMBER_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_McMemberRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_MCMEMBER_RECORD));
	BSWAPCOPY_STL_MCMEMBER_RECORD((STL_MCMEMBER_RECORD*)samad.data, (STL_MCMEMBER_RECORD*)&mcMemQuery);

	mcMemQuery.Reserved = 0;
	mcMemQuery.Reserved2 = 0;
	mcMemQuery.Reserved3 = 0;
	mcMemQuery.Reserved4 = 0;
	mcMemQuery.Reserved5 = 0;

    //
    // remove MTU, Rate, and Lifetime from mask for now
    //
    mask = samad.header.mask & (~(STL_MC_REMOVE_MTU_RATE_LIFE_MASK));
//
//      Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_McMemberRecord_GetTable", VSTATUS_OK);
		return(VSTATUS_OK);
	}
//
//      Find all McMemberRecords which match the template.
//
	vs_rdlock(&old_topology_lock);

	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	// Get requestor info
	if ((senderPort = sm_find_active_port_lid(&old_topology, maip->addrInfo.slid)) == NULL) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_WARN_FMT( __func__, "Received McMember GetTable request from unknown lid 0x%08X",
		       maip->addrInfo.slid);
		vs_rwunlock(&old_topology_lock);
		return VSTATUS_OK;
	}

    (void)vs_lock(&sm_McGroups_lock);

	/* For trusted queries, we return all the members of all groups with all the information.
	   For non-trusted queries we return only one member per group.  We also clear the port guid,
	   join state, and proxy join.
	 */

	for_all_multicast_groups(mcgp)
	{
		 /* If they are looking for a specific MGID, check that first,
		  * no need to check every member's MGID */
		if (samad.header.mask & STL_MCMEMBER_COMPONENTMASK_MGID &&
			memcmp(&(mcgp->mGid), &mcMemQuery.RID.MGID, sizeof(mcgp->mGid))) {
			continue;
		}

		// Since groups can only contain full members (for now), we don't need to worry about
		// about limited members getting McMember records for other limited members.
		if (VirtualFabrics != NULL && 
			VirtualFabrics->securityEnabled &&
			(sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key) &&
		    bitset_test_intersection(&mcgp->vfMembers, &senderPort->portData->vfMember) == 0) {
			continue;
		}

		for_all_multicast_members(mcgp, mcmp) {
			if ((status = sa_check_len(data, sizeof(STL_MCMEMBER_RECORD), bytes)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( "sa_McMemberRecord_GetTable",
					   "Reached size limit at %d records", *records);
				goto done;
			}

			BSWAPCOPY_STL_MCMEMBER_RECORD(&mcmp->record, (STL_MCMEMBER_RECORD*)data);
			if (sa_template_test(samad.data, &data, sizeof(STL_MCMEMBER_RECORD), bytes, records) == VSTATUS_OK) {
				/* Not trusted queries can't see some fields and get one record per group */
				if (sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key)
				{
					data -= sizeof(STL_MCMEMBER_RECORD) + bytes;
					BSWAPCOPY_STL_MCMEMBER_RECORD((STL_MCMEMBER_RECORD*)data, &mcMemTemp);
					memset(&mcMemTemp.RID.PortGID, 0, sizeof(mcMemTemp.RID.PortGID));
					mcMemTemp.JoinSendOnlyMember = 0;
					mcMemTemp.JoinNonMember = 0;
					mcMemTemp.JoinFullMember = 0;
					mcMemTemp.ProxyJoin = 0;
					BSWAPCOPY_STL_MCMEMBER_RECORD(&mcMemTemp, (STL_MCMEMBER_RECORD*)data);
					data += sizeof(STL_MCMEMBER_RECORD) + bytes;
					break;
				} else if ((samad.header.mask & STL_MCMEMBER_COMPONENTMASK_MGID) == 0)
				{
					/* Per IBTA 15-0.2.5, only return one record per
					 * multicast group if the MGID is is wildcarded */
					break;
				}
			}
		}
	}

done:
	(void)vs_unlock(&sm_McGroups_lock);
	(void)vs_rwunlock(&old_topology_lock);

	if (saDebugPerf) {
        IB_LOG_INFINI_INFO("sa_McMemberRecord_GetTable: Number of member records to return is", *records);
    }

	IB_EXIT("sa_McMemberRecord_GetTable", status);
	return(status);
}


#define MC_REMOVE_MTU_RATE_LIFE_MASK    0x0000000000000F30ull 
Status_t
sa_McMemberRecord_IBGetTable(Mai_t *maip, uint32_t *records)
{
	uint8_t			*data;
	uint32_t		bytes;
	STL_SA_MAD		samad;
	Status_t		status;
	McGroup_t		*mcgp;
	McMember_t		*mcmp;
	IB_MCMEMBER_RECORD	mcMemQuery; //FIXME: CHANGE McMemberRecord_t to IB_MCMEMBER_RECORD IN ALL CASES.
	IB_MCMEMBER_RECORD	mcMemTemp;
	IB_MCMEMBER_RECORD	converted;
	uint64_t		mask;
	Port_t 			*senderPort;

	IB_ENTER(__func__, maip, *records, 0, 0);

	mcgp = NULL;		// JSY - compiler is whining during development
	mcmp = NULL;		// JSY - compiler is whining during development

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(IB_MCMEMBER_RECORD));
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(IB_MCMEMBER_RECORD));
	BSWAPCOPY_IB_MCMEMBER_RECORD((IB_MCMEMBER_RECORD*)samad.data, (IB_MCMEMBER_RECORD*)&mcMemQuery);

    //
    // remove MTU, Rate, and Lifetime from mask for now
    //
    mask = samad.header.mask & (~(MC_REMOVE_MTU_RATE_LIFE_MASK));
//
//      Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, mask);
	if (status != VSTATUS_OK) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}
//
//      Find all McMemberRecords which match the template.
//
	vs_rdlock(&old_topology_lock);

	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	// Get requestor info
	if ((senderPort = sm_find_active_port_lid(&old_topology, maip->addrInfo.slid)) == NULL) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_LOG_WARN_FMT( __func__, "Received McMember GetTable request from unknown lid 0x%08X",
		       maip->addrInfo.slid);
		vs_rwunlock(&old_topology_lock);
		return VSTATUS_OK;
	}

    (void)vs_lock(&sm_McGroups_lock);

	/* For trusted queries, we return all the members of all groups with all the information.
	   For non-trusted queries we return only one member per group.  We also clear the port guid,
	   join state, and proxy join.
	 */

	for_all_multicast_groups(mcgp) {
		if (mcgp->sl > 15)
			continue;

		 /* If they are looking for a specific MGID, check that first,
		  * no need to check every member's MGID */
		if (samad.header.mask & IB_MCMEMBER_RECORD_COMP_MGID &&
			memcmp(&(mcgp->mGid), &mcMemQuery.RID.MGID, sizeof(mcgp->mGid))) {
			continue;
		}

		// Since groups can only contain full members (for now), we don't need to worry about
		// about limited members getting McMember records for other limited members.
		if (VirtualFabrics != NULL && 
			VirtualFabrics->securityEnabled &&
			(sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key) &&
		    bitset_test_intersection(&mcgp->vfMembers, &senderPort->portData->vfMember) == 0) {
			continue;
		}

		for_all_multicast_members(mcgp, mcmp) {
			if ((status = sa_check_len(data, sizeof(IB_MCMEMBER_RECORD), bytes)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( __func__,
					   "Reached size limit at %d records", *records);
				goto done;
			}

			converted.RID.MGID = mcmp->record.RID.MGID;
			converted.RID.PortGID = mcmp->record.RID.PortGID;
			converted.Q_Key = mcmp->record.Q_Key;
			converted.MLID = MCAST32_TO_MCAST16(mcmp->record.MLID);
			converted.MtuSelector = mcmp->record.MtuSelector;
			converted.Mtu = mcmp->record.Mtu;
			converted.TClass = mcmp->record.TClass;
			converted.P_Key = mcmp->record.P_Key;
			converted.RateSelector = mcmp->record.RateSelector;
			converted.Rate = mcmp->record.Rate;
			converted.PktLifeTimeSelector = mcmp->record.PktLifeTimeSelector;
			converted.PktLifeTime = mcmp->record.PktLifeTime;
			converted.u1.s.SL = mcmp->record.SL;
			converted.u1.s.FlowLabel = 0; 
			converted.u1.s.HopLimit = mcmp->record.HopLimit;
			converted.Scope = mcmp->record.Scope;
			converted.JoinFullMember = mcmp->record.JoinFullMember;
			converted.JoinSendOnlyMember = mcmp->record.JoinSendOnlyMember;
			converted.JoinNonMember = mcmp->record.JoinNonMember;
			converted.ProxyJoin = mcmp->record.ProxyJoin;
			converted.Reserved = 0;
			converted.Reserved2 = 0;
			converted.Reserved3[0] = 0;
			converted.Reserved3[1] = 0;

			BSWAPCOPY_IB_MCMEMBER_RECORD(&converted, (IB_MCMEMBER_RECORD*)data);
			if (sa_template_test(samad.data, &data, sizeof(IB_MCMEMBER_RECORD), bytes, records) == VSTATUS_OK) {
				/* Not trusted queries can't see some fields and get one record per group */
				if (sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key)
				{
					data -= sizeof(IB_MCMEMBER_RECORD) + bytes;
					BSWAPCOPY_IB_MCMEMBER_RECORD((IB_MCMEMBER_RECORD*)data, &mcMemTemp);
					memset(&mcMemTemp.RID.PortGID, 0, sizeof(mcMemTemp.RID.PortGID));
					mcMemTemp.JoinFullMember = 0;
					mcMemTemp.JoinSendOnlyMember = 0;
					mcMemTemp.JoinNonMember = 0;
					mcMemTemp.ProxyJoin = 0;
					BSWAPCOPY_IB_MCMEMBER_RECORD(&mcMemTemp, (IB_MCMEMBER_RECORD*)data);
					data += sizeof(IB_MCMEMBER_RECORD) + bytes;
					break;
				} else if ((samad.header.mask & IB_MCMEMBER_RECORD_COMP_MGID) == 0)
				{
					/* Per IBTA 15-0.2.5, only return one record per
					 * multicast group if the MGID is is wildcarded */
					break;
				}
			}
		}
	}

done:
	if (saDebugPerf) {
        IB_LOG_INFINI_INFO("sa_McMemberRecord_GetTable: Number of member records to return is", *records);
    }
    (void)vs_unlock(&sm_McGroups_lock);
    (void)vs_rwunlock(&old_topology_lock);

	IB_EXIT(__func__, status);
	return(status);
}

// -------------------------------------------------------------------------- //
// invoked by the CLI interface.
void
showStlGroups(int termWidth, uint8_t showNodeName) { 
    uint32_t i = 0; 
    McGroup_t	*dGroup; 
    McMember_t	*dMember; 
    Status_t	status; 
    char		*nodeName = 0; 
    PrintDest_t dest; 
    
    if (topology_passcount < 1) {
        sysPrintf("\nSM is currently in the %s state, count = %d\n\n", sm_getStateText(sm_state), (int)sm_smInfo.ActCount); 
        return;
    }
    if (sm_McGroups_lock.type == 0) {
        sysPrintf("Fabric Manager is not running!\n"); 
        return;
    } else if (sm_McGroups == 0) {
        sysPrintf("There are no Multicast Groups at this time!\n"); 
        return;
    }
    
    if (showNodeName) {
        status = vs_rdlock(&old_topology_lock); 
        if (status != VSTATUS_OK) {
            sysPrintf("error locking old_topology_lock %lu\n", (long)status); 
            return;
        }
    }
    
    PrintDestInitFile(&dest, stdout); 
    
    status = vs_lock(&sm_McGroups_lock); 
    if (status != VSTATUS_OK) {
        sysPrintf("error locking sm_McGroups_lock %lu\n", (long)status); 
        return;
    }
    
    for_all_multicast_groups(dGroup) {
        for_all_multicast_members(dGroup, dMember) {
            Port_t *portp; 
            
            if (i++) 
                PrintSeparator(&dest); 
            (void)PrintStlMcMemberRecord(&dest, 0, (STL_MCMEMBER_RECORD *)&dMember->record); 
            
            if (showNodeName) {
                if (old_topology.node_head == NULL) {
                    sysPrintf(" %s\n", "?"); 
                    continue;
                }
                
                if ((portp = sm_find_port_guid(&old_topology, dMember->portGuid)) != NULL) {
                    Node_t *nodep; 
                    if ((nodep = sm_find_port_node(&old_topology, portp)) != 0) {
                        nodeName = sm_nodeDescString(nodep); 
                        sysPrintf("Node: %s\n", (nodeName == NULL) ? "?" : nodeName);
                    }
                }
            }
        }
    }
    
    vs_unlock(&sm_McGroups_lock); 
    if (showNodeName) 
        vs_rwunlock(&old_topology_lock); 
    
    return;
}

Status_t createBroadcastGroup(uint16_t pkey, uint8_t mtu, uint8_t rate, uint8_t sl, uint32_t qkey, uint32_t fl, uint8_t tc) {
	Status_t			status;
	McGroup_t			*mcGroup	= NULL;
	McMember_t			*mcMember	= NULL;
	STL_MCMEMBER_RECORD		*mcmp;
	STL_LID  			mLid;
	IB_GID	  			mGid;
	//uint16_t            netPKey;

    if (!pkey) {
        pkey = getDefaultPKey();
    }
	pkey = MAKE_PKEY(1, pkey);
    //cs_end16(&pkey, &netPKey);
	memcpy(&mGid, broadcastGid.Raw, sizeof(broadcastGid));
    memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));

	if (!mtu) {
		mtu = IB_MTU_2048;
	}

	if (!rate) {
		rate = IB_STATIC_RATE_200G;
	}
	
	if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
		return status;
	}

	mcGroup = sm_find_multicast_gid(mGid);
	if (mcGroup) {
		sysPrintf("Broadcast group already exists!\n");
		status = VSTATUS_BAD;
		goto done;
	}

	if ((status = sm_multicast_assign_lid(mGid, pkey, mtu, rate, &mLid)) != VSTATUS_OK) {
		sysPrintf("Failed to allocate LID for broadcast group!\n");
		goto done;
	}

	McGroup_Create(mcGroup);

	McMember_Create(mcGroup, mcMember);

	memcpy(&(mcGroup->mGid), &mGid, sizeof(mGid));
	mcGroup->mLid         = mLid;
	mcGroup->qKey         = qkey;
	mcGroup->pKey         = pkey;
	mcGroup->mtu          = mtu;
	mcGroup->rate         = rate;
	mcGroup->life         = sa_dynamicPlt[0] ? sa_dynamicPlt[9] : sm_config.sa_packet_lifetime_n2;
	mcGroup->sl		      = sl;
	mcGroup->flowLabel    = fl;
	mcGroup->tClass       = tc;
	mcGroup->hopLimit	  = 0xFF;
	mcGroup->scope     	  = IB_LINK_LOCAL_SCOPE;
	mcGroup->members_full = 1;

	mcMember->slid     = 0;
	mcMember->proxy    = 1;
	mcMember->state    = MCMEMBER_STATE_FULL_MEMBER;
	mcMember->nodeGuid = 0;
	mcMember->portGuid = SA_FAKE_MULTICAST_GROUP_MEMBER;

	mcmp = &mcMember->record;
	memcpy((void*)mcmp->RID.MGID.Raw, mcGroup->mGid.Raw, sizeof(mcGroup->mGid));
	memset((void*)mcmp->RID.PortGID.Raw, 0, sizeof(mcmp->RID.PortGID));
	mcmp->Q_Key			= mcGroup->qKey;
	mcmp->MLID			= mcGroup->mLid;
	mcmp->MtuSelector	= IB_SELECTOR_EQ;
	mcmp->Mtu		= mcGroup->mtu;
	mcmp->TClass		= mcGroup->tClass;
	mcmp->P_Key			= mcGroup->pKey;
	mcmp->RateSelector	= IB_SELECTOR_EQ;
	mcmp->Rate		= mcGroup->rate;
	mcmp->PktLifeTimeSelector	= IB_SELECTOR_EQ;
	mcmp->PktLifeTime		= mcGroup->life;
	mcmp->SL			= mcGroup->sl;
	mcmp->HopLimit		= mcGroup->hopLimit;
	mcmp->Scope			= mcGroup->scope;
	mcmp->JoinFullMember	= mcMember->state & MCMEMBER_STATE_FULL_MEMBER;
	mcmp->JoinNonMember	= mcMember->state & MCMEMBER_STATE_NON_MEMBER;
	mcmp->JoinSendOnlyMember = mcMember->state & MCMEMBER_STATE_SENDONLY_MEMBER;
	mcmp->ProxyJoin     = mcMember->proxy;

	emptyBroadcastGroup = 1;
	
done:

	vs_unlock(&sm_McGroups_lock);
	return status;
}


Status_t destroyBroadcastGroup(void) {
	Status_t	status;
	McGroup_t	*mcGroup;
	McMember_t	*mcMember;

	if (!emptyBroadcastGroup) {
		sysPrintf("No broadcast group has been created\n");
		return VSTATUS_BAD;
	}

	if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
		sysPrintf("Failed to get SA lock\n");
		return status;
	}

	if (!(mcGroup = sm_find_multicast_gid(broadcastGid))) {
		sysPrintf("Can't find broadcast group!\n");
		emptyBroadcastGroup = 0;
		status = VSTATUS_BAD;
		goto done;
	}

	while (mcGroup->mcMembers) {
		/* MUST copy head pointer into temp
		 * Passing mcGroup->members directly to the delete macro will corrupt the list
		 */
		mcMember = mcGroup->mcMembers;
		McMember_Delete(mcGroup, mcMember);
	}

	McGroup_Delete(mcGroup);

	emptyBroadcastGroup = 0;

done:
	vs_unlock(&sm_McGroups_lock);
	return status;
}


// Deletes all multicast groups, possibly recreating the default group(s).
// Used during shutdown, dbsync, and when we transition to inactive.
Status_t clearBroadcastGroups(int recreateGroup) {
	Status_t	status;
	McGroup_t	*mcGroup;
	McMember_t	*mcMember;

	if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
		sysPrintf("clearBroadcastGroups: Failed to get SA lock\n");
	} else {
        /* PRs 104779 and 104794 - cannot use for all groups macro when deleting */
        while (sm_McGroups) {
            mcGroup = sm_McGroups;
            while (mcGroup->mcMembers) {
                /* MUST copy head pointer into temp
                 * Passing mcGroup->members directly to the delete macro will corrupt the list
                 */
                mcMember = mcGroup->mcMembers;
                McMember_Delete(mcGroup, mcMember);
            }
            McGroup_Delete(mcGroup);
        }
        /* clear group globals */
        sm_McGroups = 0;
        sm_numMcGroups = 0;
        vs_unlock(&sm_McGroups_lock);
    }
    /* recreate the default group if needed */
    if (emptyBroadcastGroup && recreateGroup) {
        sa_SetDefBcGrp();
    }
	return status;
}



McGroup_t*
getBroadCastGroup(IB_GID * pGid, McGroup_t *pGroup){
	McGroup_t	*dGroup;
	Status_t	status;
		
	status = vs_lock(&sm_McGroups_lock);

	if(status != VSTATUS_OK){
		return NULL;
	}
	
	if((dGroup = sm_find_multicast_gid(*pGid)) != NULL){
		memcpy(pGroup,dGroup,sizeof(McGroup_t));
	}
	
	vs_unlock(&sm_McGroups_lock);

	if(dGroup)
		return pGroup;

	return NULL;
}

McGroup_t*
getNextBroadCastGroup(IB_GID * pGid, McGroup_t *pGroup){
	McGroup_t	*dGroup;
	Status_t	status;

	status = vs_lock(&sm_McGroups_lock);

	if(status != VSTATUS_OK){
		return NULL;
	}

	if((dGroup = sm_find_next_multicast_gid(*pGid)) != NULL){
		memcpy(pGid,&(dGroup->mGid),sizeof(IB_GID));
		memcpy(pGroup,dGroup,sizeof(McGroup_t));
	}

	vs_unlock(&sm_McGroups_lock);

	if(dGroup)
		return pGroup;

	return NULL;
}




McMember_t*
getBroadCastGroupMember(IB_GID * pGid, uint32_t index, McMember_t *pMember){
	McGroup_t	*dGroup=NULL;
	McMember_t	*dMember=NULL;
	Status_t	status;
		
	status = vs_lock(&sm_McGroups_lock);

	if(status != VSTATUS_OK){
		return NULL;
	}

	if((dGroup = sm_find_multicast_gid(*pGid)) != NULL){
		if((dMember = sm_find_multicast_member_by_index(dGroup,index)) != NULL){
			memcpy(pMember,dMember,sizeof(McMember_t));
		
		}
	}
	
	vs_unlock(&sm_McGroups_lock);

	if(dMember)
		return pMember;

	return NULL;
}

McMember_t*
getNextBroadCastGroupMember(IB_GID * pGid, uint32_t *index, McMember_t *pMember){
	McGroup_t	*dGroup=NULL;
	McMember_t	*dMember=NULL;
	Status_t	status;

	if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK){
		return NULL;
	}

	if((dGroup = sm_find_multicast_gid(*pGid)) != NULL){
		if((dMember = sm_find_next_multicast_member_by_index(dGroup,*index)) != NULL){
			memcpy(pMember,dMember,sizeof(McMember_t));
		}
	}

	if(!dGroup){
		*index = 0;
		while((dGroup = sm_find_next_multicast_gid(*pGid)) != NULL){
			memcpy(pGid,&(dGroup->mGid),sizeof(IB_GID));
			if((dMember = sm_find_next_multicast_member_by_index(dGroup,*index)) != NULL){
				memcpy(pMember,dMember,sizeof(McMember_t));
				*index = pMember->index;
				break;
			}
		}
	}
	
	vs_unlock(&sm_McGroups_lock);

	if(dMember)
		return pMember;

	return NULL;

}


void
test_bc_group_getnext(void){
	IB_GID		gid;
	McGroup_t	group;

	memset(gid.Raw,0,sizeof(IB_GID));

	while(getNextBroadCastGroup(&gid,&group) != NULL){
		sysPrintf("0x%02x%02x%02x%02x%02x%02x%02x%02x:%02x%02x%02x%02x%02x%02x%02x%02x (%08x)\n",
			group.mGid.Raw[0], group.mGid.Raw[1], group.mGid.Raw[2], group.mGid.Raw[3],
			group.mGid.Raw[4], group.mGid.Raw[5], group.mGid.Raw[6], group.mGid.Raw[7],
			group.mGid.Raw[8], group.mGid.Raw[9], group.mGid.Raw[10], group.mGid.Raw[11],
			group.mGid.Raw[12], group.mGid.Raw[13], group.mGid.Raw[14], group.mGid.Raw[15],
			(int)group.mLid); 
		
		sysPrintf("  qKey = 0x%.8lu  pKey = 0x%.4X  mtu = %d  rate = %d  life = %d  sl = %d\n",
			   (long)group.qKey, group.pKey, group.mtu, group.rate, group.life, group.sl);
		
	}

}

Status_t createMCastGroup(uint64_t* mgid, uint16_t pkey, uint8_t mtu, uint8_t rate, uint8_t sl, uint32_t qkey, uint32_t fl, uint8_t tc) {
	Status_t			status;
	McGroup_t			*mcGroup	= NULL;
	McMember_t			*mcMember	= NULL;
	STL_MCMEMBER_RECORD	*mcmp;
	STL_LID			    mLid;
	IB_GID				mGid;
	int vf = 0;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (!pkey) {
		pkey = getDefaultPKey();
	}

	mGid.AsReg64s.H = mgid[0];
	mGid.AsReg64s.L = mgid[1];

	pkey |= FULL_MEMBER;

	if (!mtu) {
		mtu = IB_MTU_2048;
	}

	if (!rate) {
		rate = IB_STATIC_RATE_200G;
	}
	
	if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
		return status;
	}

	mcGroup = sm_find_multicast_gid(mGid);
	if (mcGroup) {
		if (VirtualFabrics) {
			for (vf = 0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
				if (VirtualFabrics->v_fabric_all[vf].standby) continue;
				if ((PKEY_VALUE(VirtualFabrics->v_fabric_all[vf].pkey) == PKEY_VALUE(pkey)) &&
					(smVFValidateVfMGid(VirtualFabrics, vf, mgid) == VSTATUS_OK)) {
					bitset_set(&mcGroup->vfMembers, vf);
				}
			}
		}
		status = VSTATUS_OK;
		goto done;
	}

	if (sm_numMcGroups == MAX_MCAST_MGIDS) {
		sysPrintf("Maximum number of groups reached (%d) before group create, MGID "FMT_GID, MAX_MCAST_MGIDS, mgid[0], mgid[1]);
		status = VSTATUS_BAD;
		goto done;
	}

	if ((status = sm_multicast_assign_lid(mGid, pkey, mtu, rate, &mLid)) != VSTATUS_OK) {
		sysPrintf("Failed to allocate LID for broadcast group!\n");
		goto done;
	}

	McGroup_Create(mcGroup);

	McMember_Create(mcGroup, mcMember);

	if (VirtualFabrics) {
		for (vf = 0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
			if (VirtualFabrics->v_fabric_all[vf].standby) continue;
			if ((PKEY_VALUE(VirtualFabrics->v_fabric_all[vf].pkey) == PKEY_VALUE(pkey)) &&
				(smVFValidateVfMGid(VirtualFabrics, vf, mgid) == VSTATUS_OK)) {
				bitset_set(&mcGroup->vfMembers, vf);
			}
		}
	}

	memcpy(&(mcGroup->mGid), &mGid, sizeof(mGid));

	mcGroup->mLid         = mLid;
	mcGroup->qKey         = qkey;
	mcGroup->pKey         = pkey;
	mcGroup->mtu          = mtu;
	mcGroup->rate         = rate;
	mcGroup->life         = sa_dynamicPlt[0] ? sa_dynamicPlt[9] : sm_config.sa_packet_lifetime_n2;
	mcGroup->sl		      = sl;
	mcGroup->flowLabel    = fl;
	mcGroup->tClass       = tc;
	mcGroup->hopLimit	  = 0xFF;
	mcGroup->scope     	  = IB_LINK_LOCAL_SCOPE;
	mcGroup->members_full = 1;

	mcMember->slid     = 0;
	mcMember->proxy    = 1;
	mcMember->state    = MCMEMBER_STATE_FULL_MEMBER;
	mcMember->nodeGuid = 0;
	mcMember->portGuid = SA_FAKE_MULTICAST_GROUP_MEMBER;

	mcmp = &mcMember->record;
	memcpy((void*)mcmp->RID.MGID.Raw, mcGroup->mGid.Raw, sizeof(mcGroup->mGid));
	memset((void*)mcmp->RID.PortGID.Raw, 0, sizeof(mcmp->RID.PortGID));
	mcmp->Q_Key			= mcGroup->qKey;
	mcmp->MLID			= mcGroup->mLid;
	mcmp->MtuSelector	= IB_SELECTOR_EQ;
	mcmp->Mtu		= mcGroup->mtu;
	mcmp->TClass		= mcGroup->tClass;
	mcmp->P_Key			= mcGroup->pKey;
	mcmp->RateSelector	= IB_SELECTOR_EQ;
	mcmp->Rate		= mcGroup->rate;
	mcmp->PktLifeTimeSelector	= IB_SELECTOR_EQ;
	mcmp->PktLifeTime		= mcGroup->life;
	mcmp->SL			= mcGroup->sl;
	mcmp->HopLimit		= mcGroup->hopLimit;
	mcmp->Scope			= mcGroup->scope;
	mcmp->JoinFullMember	= mcMember->state & MCMEMBER_STATE_FULL_MEMBER;
	mcmp->JoinNonMember	= mcMember->state & MCMEMBER_STATE_NON_MEMBER;
	mcmp->JoinSendOnlyMember = mcMember->state & MCMEMBER_STATE_SENDONLY_MEMBER;
	mcmp->ProxyJoin     = mcMember->proxy;

	emptyBroadcastGroup = 1;
	
done:

	vs_unlock(&sm_McGroups_lock);
	return status;
}

Status_t createMCastGroups(int vf, uint16_t pkey, uint8_t mtu, uint8_t rate, uint8_t sl, uint32_t qkey, uint32_t fl, uint8_t tc) {
	IB_GID		mGid;
	//uint16_t	netPKey;
	uint8_t		ipv6=0x60;
	uint64_t	vfmGid[2];
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (!VirtualFabrics) return VSTATUS_BAD;

	if (vf >= VirtualFabrics->number_of_vfs_all) {
		return VSTATUS_BAD;
	}

	if (!pkey) {
		pkey = getDefaultPKey();
	}
	pkey = MAKE_PKEY(1, pkey);
	//cs_end16(&pkey, &netPKey);

	memcpy(&mGid, broadcastGid.Raw, sizeof(broadcastGid));
	memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));

	vfmGid[0] = mGid.AsReg64s.H;
	vfmGid[1] = mGid.AsReg64s.L;

	if (smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) {
		if (createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK) {
			IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
			   		vfmGid[0], vfmGid[1]);

			memcpy(&mGid, allNodesGid.Raw, sizeof(allNodesGid));
			memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));
			vfmGid[0] = mGid.AsReg64s.H;
			vfmGid[1] = mGid.AsReg64s.L;
			if (smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) {
				createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc);
				IB_LOG_VERBOSE_FMT_VF(VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
			   		vfmGid[0], vfmGid[1]);
			}

			memcpy(&mGid, otherGid.Raw, sizeof(otherGid));
			memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));

			vfmGid[0] = mGid.AsReg64s.H;
			vfmGid[1] = mGid.AsReg64s.L;
			if ((smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) &&
				(createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK)) {
				IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
		   				vfmGid[0], vfmGid[1]);
			}
			memcpy(&mGid, mcastDnsGid.Raw, sizeof(mcastDnsGid));
			memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));

			vfmGid[0] = mGid.AsReg64s.H;
			vfmGid[1] = mGid.AsReg64s.L;
			if ((smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) &&
				(createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK)) {
				IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
					vfmGid[0], vfmGid[1]);
            }
		}
	}


	memcpy(&mGid, allNodesGid.Raw, sizeof(allNodesGid));
	memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));
	memcpy(&(mGid.Raw[IPV6_INDEX]), &ipv6, sizeof(ipv6));

	vfmGid[0] = mGid.AsReg64s.H;
	vfmGid[1] = mGid.AsReg64s.L;
	if ((smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) &&
		(createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK)) {
		IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
		   		vfmGid[0], vfmGid[1]);
	}

	memcpy(&mGid, allRoutersGid.Raw, sizeof(allRoutersGid));
	memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));
	memcpy(&(mGid.Raw[IPV6_INDEX]), &ipv6, sizeof(ipv6));
	vfmGid[0] = mGid.AsReg64s.H;
	vfmGid[1] = mGid.AsReg64s.L;
	if ((smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) &&
		(createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK)) {
		IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
		   		vfmGid[0], vfmGid[1]);
	}

	memcpy(&mGid, otherGid.Raw, sizeof(otherGid));
	memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));
	memcpy(&(mGid.Raw[IPV6_INDEX]), &ipv6, sizeof(ipv6));
	vfmGid[0] = mGid.AsReg64s.H;
	vfmGid[1] = mGid.AsReg64s.L;
	if ((smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) &&
		(createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK)) {
		IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
		   		vfmGid[0], vfmGid[1]);
	}

	memcpy(&mGid, mcastDnsGid.Raw, sizeof(mcastDnsGid));
	memcpy(&(mGid.Raw[PKEY_INDEX]), &pkey, sizeof(pkey));
	memcpy(&(mGid.Raw[IPV6_INDEX]), &ipv6, sizeof(ipv6));
	vfmGid[0] = mGid.AsReg64s.H;
	vfmGid[1] = mGid.AsReg64s.L;
	if ((smVFValidateVfMGid(VirtualFabrics, vf, vfmGid) == VSTATUS_OK) &&
		(createMCastGroup(vfmGid, pkey, mtu, rate, sl, qkey, fl, tc) == VSTATUS_OK)) {
		IB_LOG_VERBOSE_FMT_VF( VirtualFabrics->v_fabric_all[vf].name, "createMCastGroups", "Creating multicast GID "FMT_GID,
		   		vfmGid[0], vfmGid[1]);
	}

	return VSTATUS_OK;
}

// Handles auto-disabling of ports when mc delete threshold is exceeded.
//
// NOTE: Needs to be called from under an old_topology lock.
//
static void
sa_updateMcDeleteCountForPort(Port_t *portp) {
	uint64_t timenow, mcDeleteInterval;
	Node_t *nodep;
	
	if (!sm_mcDosThreshold) return;

	IB_ENTER("sa_updateMcDeleteCountForPort", portp, 0, 0, 0);

	nodep = portp->portData->nodePtr;
	if (!nodep) return;

	(void)vs_time_get(&timenow);

	if (portp->portData->mcDeleteStartTime) {
		mcDeleteInterval = timenow - portp->portData->mcDeleteStartTime;
		if (mcDeleteInterval < sm_mcDosInterval) {
			portp->portData->mcDeleteCount++;
		} else {
			portp->portData->mcDeleteStartTime = timenow;
			portp->portData->mcDeleteCount = 1;
		}
	} else {
		portp->portData->mcDeleteStartTime = timenow;
		portp->portData->mcDeleteCount = 1;
	}

	if (portp->portData->mcDeleteCount >= sm_mcDosThreshold) {
		IB_LOG_WARN_FMT("sa_updateMcDeleteCountForPort", 
		       "MC Dos threshold exceeded for: Node='%s', GUID="FMT_U64", Port %d; %s port",
		       sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, sm_mcDosAction? "bouncing" : "disabling");

		portp->portData->mcDeleteStartTime = 0;
		portp->portData->mcDeleteCount = 0;

		if (sm_mcDosAction == 0) {
        	(void)sm_removedEntities_reportPort(nodep, portp, SM_REMOVAL_REASON_MC_DOS);
			(void)sm_disable_port(&old_topology, nodep, portp);
		} else {
			(void)sm_bounce_port(&old_topology, nodep, portp);
		}
	}

	IB_EXIT("sa_updateMcDeleteCountForPort", 0);
}
