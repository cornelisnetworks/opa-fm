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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//
// FILE NAME
//    sa_TraceRecord.c
//
// DESCRIPTION
//    This file contains the routines to process the SA requests for
//    records of the PathRecord type.
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    sa_TraceRecord
//
// DEPENDENCIES
//    ib_mad.h
//    ib_status.h
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

#define NO_MEM_RECORDS      0xFFFFFFFF
#define TRACE_RECORD_MASK   0x55555555

static Status_t sa_TraceRecord_ValidatePathRecord(STL_SA_MAD * samad, IB_PATH_RECORD * prp);
static Status_t sa_TraceRecord_Set(uint8_t ** cpp, STL_SA_MAD * samad, Port_t * src_portp,
								   STL_LID slid, Port_t * dst_portp, STL_LID dlid,
								   PKey_t pkey, uint32_t * records, uint32_t bytes);
static Status_t sa_TraceRecord_PostProcess(Mai_t *, uint32_t *);

Status_t sa_PathRecord_Selector_Check(IB_PATH_RECORD *, uint64_t);
uint32_t saDebugTrace = 0;

/************ support for dynamic update of switch config parms *************/
extern uint8_t sa_dynamicPlt[];


Status_t
sa_TraceRecord(Mai_t * maip, sa_cntxt_t * sa_cntxt)
{
	uint32_t bytes;
	uint32_t records;
	uint64_t prefix = 0, pf2 = 0;
	uint64_t guid = 0, sguid = 0;
	STL_SA_MAD samad;
	uint8_t *cp;
	Port_t *src_portp;
	Port_t *dst_portp;
	Port_t *req_portp;
	Node_t *req_nodep;
	IB_PATH_RECORD *prp;
	IB_PATH_RECORD pathRecord;
	PKey_t pkey = INVALID_PKEY;
	STL_LID slid, dlid;

	IB_ENTER("sa_TraceRecord", maip, 0, 0, 0);

	// 
	// Assume failure.
	// 
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GETTRACETABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTraceRecord);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_METHOD;
		IB_LOG_WARN_FMT(__func__, "invalid Method: %s (%u)",
			cs_getMethodText(maip->base.method), maip->base.method);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}
	// Check Base and Class Version
	if (maip->base.bversion == STL_BASE_VERSION && maip->base.cversion == STL_SA_CLASS_VERSION) {
		bytes = Calculate_Padding(sizeof(STL_TRACE_RECORD));
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	prp = &pathRecord;

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(IB_PATH_RECORD));
	memcpy(prp,samad.data,sizeof(IB_PATH_RECORD));
	BSWAP_IB_PATH_RECORD(prp);

	/* 
	 * Confirm that the path record used as input is valid.
	 */
	if (sa_TraceRecord_ValidatePathRecord(&samad, prp) != VSTATUS_OK) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	/* 
	 *  JSY - this is a quick cut-through check for temporarily static fields.
	 */
	if (sa_PathRecord_Selector_Check(prp, samad.header.mask) != VSTATUS_OK) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	/* 
	 *  Check the validity of the requested PKey.
	 */
	if (samad.header.mask & IB_PATH_RECORD_COMP_PKEY) {
		pkey = prp->P_Key;
	}

	// Lock the interface.
	(void) vs_rdlock(&old_topology_lock);

	/* 
	 *  Find the source port.
	 *  Note that LID (if present) is attempted before GID for fabrics
	 *    where LMC > 0.
	 */
	if ((samad.header.mask & IB_PATH_RECORD_COMP_SLID) != 0) {
		if ((src_portp = sm_find_active_port_lid(&old_topology, prp->SLID)) == NULL) {
			IB_LOG_INFINI_INFO
				("sa_TraceRecord: requested source Lid not found/active in current topology:",
				 prp->SLID);
			maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
			goto reply_TraceRecord;
		}
		slid = prp->SLID;
	} else if ((samad.header.mask & IB_PATH_RECORD_COMP_SGID) != 0) {
		prefix = prp->SGID.Type.Global.SubnetPrefix;
		guid = prp->SGID.Type.Global.InterfaceID;
		if (saDebugTrace)
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord", "Source port GID " FMT_GID, prefix, guid);

		if ((src_portp = sm_find_active_port_guid(&old_topology, guid)) == NULL) {
			if (saDebugTrace) {
				IB_LOG_INFINI_INFOLX
					("sa_TraceRecord: requested source GUID not found/active in current topology:",
					 guid);
			}
			maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID_GID;
			goto reply_TraceRecord;
		}
		if (src_portp->portData->gidPrefix != prefix || src_portp->portData->guid != guid) {
			pf2 = ntoh64(*(uint64_t *) & src_portp->portData->gid[0]);
			sguid = ntoh64(*(uint64_t *) & src_portp->portData->gid[8]);
			IB_LOG_WARN_FMT("sa_TraceRecord",
							"Source Gid " FMT_GID
							" in request from Lid 0x%x does not match found GID's " FMT_GID " "
							"port guid of " FMT_U64, prefix, guid, maip->addrInfo.slid, pf2, sguid,
							src_portp->portData->guid);
			maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
			goto reply_TraceRecord;
		}
		// use base LID for GID queries
		slid = src_portp->portData->lid;
	} else {
		IB_LOG_INFINI_INFO0("sa_TraceRecord: neither SGID nor SLID were specified");
		maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
		goto reply_TraceRecord;
	}

	/* 
	 *  Find the destination port.
	 */
	if ((samad.header.mask & IB_PATH_RECORD_COMP_DLID) != 0) {
		if ((dst_portp = sm_find_active_port_lid(&old_topology, prp->DLID)) == NULL) {
			if (saDebugTrace)
				IB_LOG_INFINI_INFO
					("sa_TraceRecord: requested destination Lid not found/active:",
					 prp->DLID);
			maip->base.status = MAD_STATUS_SA_NO_RECORDS;
			goto reply_TraceRecord;
		}
		dlid = prp->DLID;
	} else if ((samad.header.mask & IB_PATH_RECORD_COMP_DGID) != 0) {
		prefix = prp->DGID.Type.Global.SubnetPrefix;
		guid = prp->DGID.Type.Global.InterfaceID;
		if (saDebugTrace)
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord",
								   "Destination port GID " FMT_GID, prefix, guid);
		if ((dst_portp = sm_find_port_guid(&old_topology, guid)) == NULL) {
			maip->base.status = MAD_STATUS_SA_NO_RECORDS;
			if (saDebugTrace || !guid)
				IB_LOG_INFINI_INFOLX
					("sa_TraceRecord: requested destination GUID not an active port:", guid);
			goto reply_TraceRecord;
		} else if (dst_portp->state < IB_PORT_ACTIVE) {
			if (saDebugTrace)
				IB_LOG_INFINI_INFOLX
					("sa_TraceRecord: requested destination GUID not active in current topology:",
					 guid);
			maip->base.status = MAD_STATUS_SA_NO_RECORDS;
			goto reply_TraceRecord;
		}
		// use base LID for GID queries
		dlid = dst_portp->portData->lid;
	} else {
		IB_LOG_INFINI_INFO0("sa_TraceRecord: neither DGID nor DLID were specified");
		maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
		goto reply_TraceRecord;
	}

	/* 
	 *  Find the requested paths from the source port to the destination port.
	 */
	if (src_portp != NULL && dst_portp != NULL) {
		/* IBTA 1.2 C15-0.1.21 - pairwise pkey checks. */
		req_portp = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, &req_nodep);
		if (!sm_valid_port(req_portp) ||
			req_portp->state <= IB_PORT_DOWN ||
			(sa_Compare_Node_Port_PKeys(req_nodep, src_portp) != VSTATUS_OK) ||
			(sa_Compare_Node_Port_PKeys(req_nodep, dst_portp) != VSTATUS_OK)) {
			IB_LOG_ERROR_FMT("sa_TraceRecord",
							 "Failed pairwise PKey check for request (src= " FMT_U64 ", dst= "
							 FMT_U64 "", src_portp->portData->guid, dst_portp->portData->guid);
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			goto reply_TraceRecord;
		}

		if (samad.header.mask & IB_PATH_RECORD_COMP_PKEY) {
			if (!smValidatePortPKey2Way(pkey, src_portp, dst_portp)) {
				IB_LOG_ERROR_FMT("sa_TraceRecord",
								 "Failed PKey check for source " FMT_U64 " and destination "
								 FMT_U64 " for PKey 0x%x", src_portp->portData->guid,
								 dst_portp->portData->guid, pkey);
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				goto reply_TraceRecord;
			}

		} else if ((pkey = smGetCommonPKey(src_portp, dst_portp)) == INVALID_PKEY) {
			IB_LOG_WARN0("sa_TraceRecord: Failed common PKey validation for src/dst");
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			goto reply_TraceRecord;
		}

		/* PR#101615 - SM99% when host/line cards disappear in middle of sweep */
		if (dst_portp->state > IB_PORT_INIT) {
			/* Set pointer to global SA output buffer. */
			cp = sa_data;

			/* PR#101984 - SA99% when switch fails to find link back to another switch during
			   discovery */
			if (sa_TraceRecord_Set
				(&cp, & samad, src_portp, slid, dst_portp, dlid, pkey, &records,
				 bytes) != VSTATUS_OK) {
				IB_LOG_ERROR_FMT("sa_TraceRecord",
								 "Cannot find path to port LID 0x%x (port guid " FMT_U64
								 ") from port LID 0x%x (port guid " FMT_U64 ")",
								 dst_portp->portData->lid, dst_portp->portData->guid,
								 src_portp->portData->lid, src_portp->portData->guid);
			}
		} else {
			if (saDebugTrace) {
				IB_LOG_ERROR_FMT("sa_TraceRecord",
								 "destination port is not in active state; port LID: 0x%x (port GUID "
								 FMT_U64 ")", dst_portp->portData->lid,
								 dst_portp->portData->guid);
			}
			goto reply_TraceRecord;
		}
	}

	/* 
	 *  Now that we have the records, we need to post-process them.
	 *  In particular, C15-9, C15-11, and C15-18 need attention.
	 */
	if (records != 0) {
		if (sa_TraceRecord_PostProcess(maip, &records) != VSTATUS_OK) {
			records = 0;
		}
	}

	/* 
	 *  Determine reply status
	 */
reply_TraceRecord:
	(void) vs_rwunlock(&old_topology_lock);

	/* release the multicast group table if necessary */
	/* 
	 * Call the user exit to manipulate the records.
	 */
	(void) tracerecord_userexit(sa_data, &records);

	bytes = sizeof(STL_TRACE_RECORD) + Calculate_Padding(sizeof(STL_TRACE_RECORD));

	if (maip->base.status != MAD_STATUS_OK) {
		/* status already set */
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = bytes;

	if (saDebugTrace)
		IB_LOG_INFINI_INFO_FMT("sa_TraceRecord", "total records=%d", (int) records);

	(void) sa_cntxt_data(sa_cntxt, sa_data, records * bytes);
	(void) sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_TraceRecord", VSTATUS_OK);
	return (VSTATUS_OK);
}

static Status_t
sa_TraceRecord_ValidatePathRecord(STL_SA_MAD * samad, IB_PATH_RECORD * prp)
{
	Status_t rc = VSTATUS_OK;

	IB_ENTER("sa_TraceRecord_ValidatePathRecord", samad, prp, 0, 0);

	// note that the spec states that the path record should be completely
	// specified (all component mask bits set).  the validation here deviates
	// in that it only requires the minimum attributes required to identify
	// a unique path through the fabric and respond

	if (sm_config.lmc == 0) {
		// fabric LMC is zero; allow either LID or GID queries
		if ((samad->header.mask & (IB_PATH_RECORD_COMP_SLID | IB_PATH_RECORD_COMP_SGID)) == 0) {
			IB_LOG_WARNX
				("sa_TraceRecord_ValidatePathRecord: invalid component mask, source not specified mask:",
				 samad->header.mask);
			return VSTATUS_BAD;
		} else if ((samad->header.mask & (IB_PATH_RECORD_COMP_DLID | IB_PATH_RECORD_COMP_DGID)) == 0) {
			IB_LOG_WARNX
				("sa_TraceRecord_ValidatePathRecord: invalid component mask, destination not specified mask:",
				 samad->header.mask);
			return VSTATUS_BAD;
		}
	} else {
		// fabric LMC is > 0; only LID queries represent unique paths
		if ((samad->header.mask & (IB_PATH_RECORD_COMP_SLID | IB_PATH_RECORD_COMP_DLID))
			!= (IB_PATH_RECORD_COMP_SLID | IB_PATH_RECORD_COMP_DLID)) {
			IB_LOG_WARNX("sa_TraceRecord_ValidatePathRecord: invalid component mask:",
						 samad->header.mask);
			return VSTATUS_BAD;
		}
	}

	// only unicast LIDs are valid
	if ((samad->header.mask & IB_PATH_RECORD_COMP_SLID)
		&& (prp->SLID < UNICAST_LID_MIN || prp->SLID > STL_GET_UNICAST_LID_MAX())) {
		IB_LOG_WARNX("sa_TraceRecord_ValidatePathRecord: source LID is not unicast srcLid:",
					 prp->SLID);
		return VSTATUS_BAD;
	} else if ((samad->header.mask & IB_PATH_RECORD_COMP_DLID)
			   && (prp->DLID < UNICAST_LID_MIN || prp->DLID > STL_GET_UNICAST_LID_MAX())) {
		IB_LOG_WARNX
			("sa_TraceRecord_ValidatePathRecord: destination LID is not unicast dstLid:",
			 prp->DLID);
		return VSTATUS_BAD;
	}

	IB_EXIT("sa_TraceRecord_ValidatePathRecord", rc);
	return rc;
}

static Status_t
sa_TraceRecord_PostProcess(Mai_t * maip, uint32_t * records)
{
	uint32_t max_records = 0xFFFFFFFF;

	IB_ENTER("sa_TraceRecord_PostProcess", maip, *records, 0, 0);

	if (*records > max_records)
		*records = max_records;

	IB_EXIT("sa_TraceRecord_PostProcess", VSTATUS_OK);
	return (VSTATUS_OK);
}

static Status_t
sa_TraceRecord_Fill(uint8_t ** cp, STL_SA_MAD * samad, Node_t * nodep,
					uint8_t entryPortNo, Port_t * entry_portp,
					uint8_t exitPortNo, Port_t * exit_portp,
					uint32_t * records, uint32_t hopCount, uint32_t bytes)
{
	STL_TRACE_RECORD traceRecord;


	if (saDebugTrace) {
		switch (nodep->nodeInfo.NodeType) {
		case NI_TYPE_CA:
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Fill",
								   "CA node: guid=" FMT_U64 ", type=%d, sysimage-guid=" FMT_U64,
								   nodep->nodeInfo.NodeGUID, nodep->nodeInfo.NodeType,
								   nodep->nodeInfo.SystemImageGUID);
			break;
		case NI_TYPE_SWITCH:
		default:
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Fill",
								   "Switch node[%d]: guid=" FMT_U64 ", type=%d, sysimage-guid="
								   FMT_U64, (int) hopCount, nodep->nodeInfo.NodeGUID,
								   nodep->nodeInfo.NodeType, nodep->nodeInfo.SystemImageGUID);
			break;
		}

		IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Fill",
							   "entry port: guid=" FMT_U64 ", number=%d, state=%d",
							   entry_portp->portData->guid, entryPortNo, entry_portp->state);
		IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Fill",
							   "exit port:  guid=" FMT_U64 ", number=%d, state=%d",
							   exit_portp->portData->guid, exitPortNo, exit_portp->state);
	}

	(void) memset((void *) &traceRecord, 0, sizeof(STL_TRACE_RECORD));

	/* validate buffer length */
	if (sa_check_len(*cp, sizeof(STL_TRACE_RECORD), bytes) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT("sa_TraceRecord_Fill",
						 "Reached size limit while processing TRACE_RECORD request");
		return (VSTATUS_BAD);
	}

	traceRecord.IDGeneration = 0xff;	// set Generation of Component-ID values
	traceRecord.NodeType = nodep->nodeInfo.NodeType;	// set node type
	// set node ID (obfuscated)
	traceRecord.NodeID = nodep->nodeInfo.NodeGUID ^ TRACE_RECORD_MASK;	
	// set chassis ID (obfuscated)
	traceRecord.ChassisID = nodep->nodeInfo.SystemImageGUID ^ TRACE_RECORD_MASK;	

	/* Component-ID of the entry & exit ports */
	switch (nodep->nodeInfo.NodeType) {
	case NI_TYPE_CA:
		traceRecord.EntryPortID = entry_portp->portData->guid;	// set entry port number 
		break;
	case NI_TYPE_SWITCH:
		traceRecord.EntryPort = entryPortNo;	// set entry port number
		traceRecord.ExitPort = exitPortNo;	// set exit port number
		break;
	default:
		break;
	}

	// Obfuscate the guids...
	traceRecord.EntryPortID ^= TRACE_RECORD_MASK;
	traceRecord.ExitPortID ^= TRACE_RECORD_MASK;

	BSWAPCOPY_STL_TRACE_RECORD(&traceRecord, (STL_TRACE_RECORD*)*cp);

	/* Adjust global SA output buffer pointer and record count. */
	sa_increment_and_pad(cp, sizeof(STL_TRACE_RECORD), bytes, records);

	return (VSTATUS_OK);
}

static Status_t
sa_TraceRecord_Set(uint8_t ** cpp, STL_SA_MAD * samad, Port_t * src_portp, STL_LID slid,
				   Port_t * dst_portp, STL_LID dlid, PKey_t pkey, uint32_t * records,
				   uint32_t bytes)
{
	int32_t exit_portno = 0, entry_portno = 0;
	Node_t *next_nodep, *last_nodep;
	Port_t *last_portp, *next_portp, *entry_portp;
	uint32_t hopCount = 0;
	int firstNode = TRUE;
	uint8_t sc=0;


	IB_ENTER("sa_TraceRecord_Set", *cpp, src_portp, dst_portp, pkey);


	/* Locate the node which house these ports. */
	next_nodep = sm_find_port_node(&old_topology, src_portp);
	last_nodep = sm_find_port_node(&old_topology, dst_portp);
	if ((next_nodep == NULL) || (last_nodep == NULL)) {
		IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	next_portp = src_portp;
	last_portp = dst_portp;

	if (saDebugTrace) {
		IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set",
							   "src-node " FMT_U64 ", src-port " FMT_U64,
							   next_nodep->nodeInfo.NodeGUID, src_portp->portData->guid);
		IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set", "dst-node " FMT_U64 ", dst-port " FMT_U64,
							   last_nodep->nodeInfo.NodeGUID, dst_portp->portData->guid);
	}

	/* 
	 *  Find the MTU and the RATE of this path.
	 */
	if (src_portp == dst_portp) {
		hopCount = 0;
		if (saDebugTrace)
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set",
								   "Path: source & destination ports are the same.");
	} else if (IsDirectConnected(next_nodep, next_portp, last_nodep, last_portp)) {
		if (saDebugTrace) {
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set",
								   "Path: source & destination ports are connected directly.");
		}
		hopCount = 1;
	} else {
		if (saDebugTrace)
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set",
								   "Path: multiple component connections.");

		if (next_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			firstNode = FALSE;
			if (saDebugTrace)
				IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set", "First Node:");

			/* Fill in the TraceRecord with FI node and port related data. */
			if (sa_TraceRecord_Fill(cpp, samad, next_nodep, src_portp->index, src_portp,
									src_portp->index, src_portp, records, 0,
									bytes) != VSTATUS_OK) {
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}

			next_nodep = sm_find_node(&old_topology, src_portp->nodeno);
			// PR#103535 - data corruption results in invalid topology
			if (next_nodep == NULL) {
				IB_LOG_WARN_FMT("sa_TraceRecord_Set",
								"Cannot find path to destination port " FMT_U64
								" from source port " FMT_U64
								"; INVALID TOPOLOGY, next_nodep is NULL",
								dst_portp->portData->guid, src_portp->portData->guid);
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}
			next_portp = sm_get_port(next_nodep, 0);
			entry_portno = sm_get_route(&old_topology, next_nodep, 0, slid, &sc);
		}


		/* 
		 * Process all switch nodes in the path.
		 */
		while (next_nodep != last_nodep && next_nodep != NULL) {
			entry_portp = next_portp;

			if (entry_portp == NULL) {
				IB_LOG_ERROR_FMT("sa_TraceRecord_Set",
								 "Cannot find entry port to node " FMT_U64 " from node " FMT_U64
								 ": LFT entry for destination is 255 from switch %s (guid "
								 FMT_U64 ")", dst_portp->portData->guid,
								 src_portp->portData->guid, sm_nodeDescString(next_nodep),
								 next_nodep->nodeInfo.NodeGUID);
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}

			/* Get the port number of exit port from node forwarding table. */
			exit_portno = sm_get_route(&old_topology, next_nodep, entry_portp->index, dlid, &sc);

			if (exit_portno == 255) {
				/* PR#101984 - no path from this node for given Lid */
				IB_LOG_WARN_FMT("sa_TraceRecord_Set",
								"Cannot find path to port " FMT_U64 " from port " FMT_U64
								": LFT entry for destination is 255 from switch %s (nodeGuid "
								FMT_U64 ")", dst_portp->portData->guid,
								src_portp->portData->guid, sm_nodeDescString(next_nodep),
								next_nodep->nodeInfo.NodeGUID);
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}

			/* Get the port record associated with the exit port. */
			next_portp = sm_get_port(next_nodep, exit_portno);
			if (!sm_valid_port(next_portp) || next_portp->state < IB_PORT_ACTIVE) {
				// PR#103535 - data corruption results in invalid topology
				IB_LOG_WARN_FMT("sa_TraceRecord_Set",
								"Cannot find path to destination port " FMT_U64
								" from source port " FMT_U64
								"; INVALID TOPOLOGY, next_portp[%d] state=%d (NOT ACTIVE=4)",
								dst_portp->portData->guid, src_portp->portData->guid,
								exit_portno, (next_portp) ? next_portp->state : -1);
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}

			if (saDebugTrace && !firstNode)
				IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set", "Node Hop %d", hopCount + 1);
			else {
				firstNode = FALSE;
				if (saDebugTrace)
					IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set", "First Node:");
			}

			/* Fill in the TraceRecord with node and port related data. */
			if (sa_TraceRecord_Fill(cpp, samad, next_nodep, entry_portno, entry_portp,
									exit_portno, next_portp, records, ++hopCount,
									bytes) != VSTATUS_OK) {
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}

			/* Get the entry port number to the next node. */
			exit_portno = 0;
			entry_portno = next_portp->portno;

			/* Get the next node record. */
			next_nodep = sm_find_node(&old_topology, next_portp->nodeno);
		}


		/* Fill in the TraceRecord with node and port related data for the last node. */
		if (saDebugTrace)
			IB_LOG_INFINI_INFO_FMT("sa_TraceRecord_Set", "Last Node:");

		if (next_nodep==NULL) {
			IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
			return (VSTATUS_BAD);
		}

		if (last_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			if (sa_TraceRecord_Fill(cpp, samad, next_nodep, dst_portp->index, dst_portp,
									dst_portp->index, dst_portp, records, 0,
									bytes) != VSTATUS_OK) {
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}
		} else {
			if (sa_TraceRecord_Fill(cpp, samad, next_nodep, entry_portno, next_portp,
									exit_portno, dst_portp, records, ++hopCount,
									bytes) != VSTATUS_OK) {
				IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
				return (VSTATUS_BAD);
			}
		}

		IB_EXIT("sa_TraceRecord_Set", VSTATUS_OK);
		return (VSTATUS_OK);
	}


	/* Fill in the TraceRecord with node and port related data. */
	if (sa_TraceRecord_Fill(cpp, samad, next_nodep, entry_portno, src_portp,
							exit_portno, dst_portp, records, hopCount, bytes) != VSTATUS_OK) {
		IB_EXIT("sa_TraceRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	IB_EXIT("sa_TraceRecord_Set", VSTATUS_OK);
	return (VSTATUS_OK);
}
