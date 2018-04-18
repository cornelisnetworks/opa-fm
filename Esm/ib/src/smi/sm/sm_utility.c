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
//                                       //
// FILE NAME                                     //
//    sm_utility.c                               //
//                                       //
// DESCRIPTION                                   //
//    This file contains the SM utility routines (like those that        //
//    format MADs, etc.                              //
//                                       //
// DATA STRUCTURES                               //
//    None                                   //
//                                       //
// FUNCTIONS                                     //
//    sm_setup_node     get NodeInfo and PortInfo for node       //
//    sm_initialize_port    setup port for real work             //
//                                       //
// DEPENDENCIES                                  //
//    ib_mad.h                                   //
//    ib_status.h                                //
//                                       //
//                                       //
//===========================================================================//

#include <stdlib.h>				/* for qsort */
#include "os_g.h"
#include "ib_status.h"
#include "ib_mad.h"
#include "ib_macros.h"
#include "ib_sa.h"
#include "cs_g.h"
#include "cs_csm_log.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "stl_cca.h"
#include "sm_dbsync.h"
#include "cs_context.h"
#include "if3.h"
#include "topology.h"
#include "iquickmap.h"
#if defined(__VXWORKS__)
#include "bspcommon/h/usrBootManager.h"
#endif

#ifdef IB_STACK_OPENIB
#include "mal_g.h"
#endif
#include "iba/stl_sm_priv.h"
#include "time.h"
#ifdef __VXWORKS__
#include "UiUtil.h"
#endif

extern Lock_t tid_lock;
uint32_t sm_instance;
//#define TRACK_SEARCHES

uint32_t sm_node_guid_cnt = 0;
uint32_t sm_node_id_cnt = 0;
uint32_t sm_port_guid_cnt = 0;

boolean lid_space_exhausted = FALSE;
uint32_t sm_port_id_cnt = 0;
uint32_t sm_np_id_cnt = 0;
STL_LID sm_port_lid_cnt = 0;
STL_LID sm_np_lid_cnt = 0;
uint32_t sm_switch_id_cnt = 0;
uint32_t sm_port_node_cnt = 0;
uint32_t smDebugDynamicAlloc = 0;
uint32_t smDebugDynamicPortAlloc = 0;

// list of nodes/ports removed for various reasons
static RemovedEntities_t sm_removedEntities;

/*
 * Missing device queue. Keeps track of devices
 * that have disappeared from fabric for LID-recycling.
 */
static MissingLidEntry_t *missingDevHead = NULL;
static MissingLidEntry_t *missingDevTail = NULL;

static void
sm_lidmap_remove_missing(MissingLidEntry_t *m);

extern FabricData_t preDefTopology;
static void sm_util_free_port(Port_t * portp);

static STL_LID sm_find_next_lid(uint8_t lmc);
static STL_LID sm_clear_lid(Port_t * portp);
static int sm_lidmap_pop_and_search(STL_LID *lid, STL_LID delta);

/* return ascii sm state */
const char *
sm_getStateText(uint32_t state)
{
	switch (state) {
	case SM_STATE_NOTACTIVE:
		return ("NOTACTIVE");
	case SM_STATE_DISCOVERING:
		return ("DISCOVERING");
	case SM_STATE_STANDBY:
		return ("STANDBY");
	case SM_STATE_MASTER:
		return ("MASTER");
	default:
		return ("INVALID STATE");
	}
}

//----------------------------------------------------------------------------//

char *
sm_getAttributeIdText(uint32_t aid)
{
	switch (aid) {
	case (1): 		return "CLASSPORTINFO"; break;
	case (2): 		return "NOTICE"; break;
	case (3): 		return "INFORMINFO"; break;
	case (0x10): 	return "NODE DESCRIPTION"; break;
	case (0x11): 	return "NODEINFO"; break;
	case (0x12):	return "SWITCHINFO"; break;
	case (0x14):	return "(IB) GUIDINFO"; break;
	case (0x15):	return "PORTINFO"; break;
	case (0x16):	return "PARTITION TABLE"; break;
	case (0x17):	return "SL2SC TABLE"; break;
	case (0x18):	return "VLARB TABLE"; break;
	case (0x19):	return "LFT"; break;
	case (0x1A):	return "(IB) RFT"; break;
	case (0x1B):	return "MFT"; break;
	case (0x20):	return "SMINFO"; break;
	case (0x30):	return "(IB) VENDORDIAG"; break;
	case (0x31):	return "LEDINFO"; break;
	case (0x32):	return "CABLEINFO"; break;
	case (0x80):	return "AGGREGATE"; break;
	case (0x81):	return "SC2SC MAPPING TABLE"; break;
	case (0x82):	return "SC2SL MAPPING TABLE"; break;
	case (0x83):	return "SC2VLR MAPPING TABLE"; break;
	case (0x84):	return "SC2VLT MAPPING TABLE"; break;
	case (0x85):	return "SC2VLNT MAPPING TABLE"; break;
	case (0x86):	return "SWITCH LIFETIME"; break;
	case (0x87):	return "PORT STATE"; break;
	case (0x88):	return "PGFT"; break;
	case (0x89):	return "PORT GROUP"; break;
	case (0x8a):	return "BUFFER CONTROL"; break;
	case (0x8b):	return "CONGESTION INFO"; break;
	case (0x8c):	return "SWITCH CONGESTION LOG"; break;
	case (0x8d):	return "SWITCH CONGESTION SETTING"; break;
	case (0x8e):	return "SWITCH PORT CONGESTION SETTING"; break;
	case (0x8f):	return "HFI CONGESTION LOG"; break;
	case (0x90):	return "HFI CONGESTION SETTING"; break;
	case (0x91):	return "HFI CONGESTION CONTROL TABLE"; break;
	case (0x92):    return "BW ARBITRATION TABLE"; break;
	case (0x94):    return "SC2SC MULTI SET"; break;
	case (0x95):    return "EXTENDED BUFFER CONTROL"; break;
	case (0x96):    return "SWITCH CONGESTION EXCHANGE"; break;
	case (0x97):    return "SL PAIRS"; break;
	case (0xff19):	return "(IB) Port LFT"; break;
	case (0xff12):	return "(IB) VENDOR SWITCHINFO"; break;
	case (0xff21):	return "(IB) PORT GROUP"; break;
	case (0xff22):	return "(IB) AR LIDMASK"; break;
	default: 		return "UNKNOWN AID"; break;
	}
}


//Returns text of Mad Status, Mad status is a bitfield and this only returns
//one text code maximum.  Could rework to dynamically build string with all
//codes
char *
sm_getMadStatusText(uint16_t status)
{
	status = status & MAD_STATUS_MASK;
	if(status == MAD_STATUS_OK) return "OK";
	else if(status == MAD_STATUS_BUSY) return "Busy";
	else if(status == MAD_STATUS_REDIRECT) return "Redirect";
	else if(status == MAD_STATUS_BAD_CLASS) return "Bad Class";
	else if(status == MAD_STATUS_BAD_METHOD) return "Bad Method";
	else if(status == MAD_STATUS_BAD_ATTR) return "Bad Attribute";
	else if(status == MAD_STATUS_BAD_FIELD) return "Bad Field / Invalid";
	else return "Unknown";
}

//
// Renders a path into a string for logging.
//
// Takes an explicit path length to support rendering paths straight out of
// packet buffers.
//
#define PATH_STRING_MAX_LEN (IBA_MAX_PATHSIZE * 4 + 7) // "Path[...]\0"
void smGetPathString(uint8_t *path, size_t pathlen, char *buf, size_t buflen)
{
	int i, end;

	memset(buf, 0, buflen);

	cs_snprintfcat(&buf, &buflen, "Path[");

	for (i = 1, end = MIN(pathlen, IBA_MAX_PATHSIZE); i <= end; ++i) {
		if (i > 1) cs_snprintfcat(&buf, &buflen, " ");
		cs_snprintfcat(&buf, &buflen, "%d", path[i]);
	}

	cs_snprintfcat(&buf, &buflen, "]");
}

//
// Renders a node into an informational string for logging.
//
#define NODE_STRING_MAX_LEN 500
static void
smGetNodeString(uint8_t *path, STL_LID lid, Node_t *node, Port_t *port,
	char *buff, size_t buffLen)
{
	int i;
	int pIndex = -1;

	memset(buff, 0, buffLen);

	// If we have a path supplied, lets add it to buffer
	if (path != NULL){
		cs_snprintfcat(&buff, &buffLen, "Path:[");
		if(path[0] == 0)
		{
			cs_snprintfcat(&buff, &buffLen, "Local Port");
			if (!node && sm_topop) 
				node = sm_topop->node_head;
		} else {
			for (i = 1; i <= (int) path[0]; i++) {
				cs_snprintfcat(&buff, &buffLen, "%2d ", path[i]);
			}
			if (!node && sm_topop) {
				if (lid == STL_LID_RESERVED || lid == STL_LID_PERMISSIVE) {
					// pure DR: resolve path normally
					node = sm_find_node_by_path(sm_topop, NULL, path);
				} else {
					// mixed LR-DR: find node at beginning of DR path via LID
					sm_find_node_and_port_lid(sm_topop, lid, &node);
					if (node)
						node = sm_find_node_by_path(sm_topop, node, path);
				}
			}
		}
		cs_snprintfcat(&buff, &buffLen, "]");
	}

	// If we have a port - then the port index is valid
	if (port)
		pIndex = port->index;

	// If we don't have a node but we do have a lid, lets try to look it up in
	// topo
	if (lid){
		cs_snprintfcat(&buff, &buffLen, " Lid:[%d]", lid);
		if(!node && (lid <= STL_GET_UNICAST_LID_MAX()) && sm_topop)
			port = sm_find_node_and_port_lid(sm_topop, lid, &node);

		// If we just found the node by its LID, and its not a switch
		// then the pIndex is valid.
		if (port && node && (node->nodeInfo.NodeType != NI_TYPE_SWITCH)) {
			pIndex = port->index;
		}
	}

	// Print node info if we have it
	if (node) {
		if (pIndex!=-1) {
			cs_snprintfcat(&buff, &buffLen, " NodeGUID:[" FMT_U64 "] NodeDesc:[%s] PortIndex:[%d]",
				node->nodeInfo.NodeGUID, node->nodeDesc.NodeString, pIndex);
		} else {
			cs_snprintfcat(&buff, &buffLen, " NodeGUID:[" FMT_U64 "] NodeDesc:[%s] PortIndex:[unknown]",
				node->nodeInfo.NodeGUID, node->nodeDesc.NodeString);
		}
	} else {
		cs_snprintfcat(&buff, &buffLen, " [Can't find node in topology!]");
	}
}

void
smLogHelper(uint32_t level, const char *function, char *preamble, int aid, uint64_t tid,
			char *trailertxt, uint32_t final)
{
	cs_log(level, function, "%s AID[0x%x] '%s' with TID " FMT_U64 ", %s=%.8X",
		   preamble, aid, sm_getAttributeIdText(aid), tid, trailertxt, final);
}

Status_t
sm_validate_incoming_mad(Mai_t * out_mad, Mai_t * in_mad)
{
	Status_t status = VSTATUS_OK;	
	char nodeString[NODE_STRING_MAX_LEN];

	if (sm_config.sma_spoofing_check) {
		// If the mad was an error (oh no!), actually time it out
		if (in_mad->type == MAI_TYPE_ERROR) {
			status = VSTATUS_TIMEOUT;
			return status;
		}
		// GEN 2 M_KEY HANDLING GOES HERE
		((DRStlSmp_t *) (in_mad->data))->M_Key = 0;

		// Directed Route Return Path Validation
		if (out_mad->base.mclass == MAD_CV_SUBN_DR) {
			DRStlSmp_t *out_dr = (DRStlSmp_t *)out_mad->data;
			DRStlSmp_t *in_dr = (DRStlSmp_t *)in_mad->data;

			uint8_t outHops = MIN(out_mad->base.hopCount, IBA_MAX_PATHSIZE);
			uint8_t inHops = MIN(in_mad->base.hopCount, IBA_MAX_PATHSIZE);

			if (sm_topop->node_head && outHops > 0 &&
				(outHops != inHops || memcmp(out_dr->InitPath, in_dr->InitPath, sizeof(out_dr->InitPath)) != 0))
			{
				const char errorStr[] = "<path unrealizable>";
				STL_LID drlid = out_dr->DrSLID;
				uint8_t outHops = MIN(out_mad->base.hopCount, IBA_MAX_PATHSIZE);
				uint8_t inHops = MIN(in_mad->base.hopCount, IBA_MAX_PATHSIZE);
				char outPathString[PATH_STRING_MAX_LEN];
				char inPathString[PATH_STRING_MAX_LEN];
				Node_t *node = NULL;
				Port_t *port = NULL;

				// for logging, try to resolve the node neighboring the
				// destination using only the outbound (trusted) path.  keep in
				// mind the path can be pure DR or mixed LR-DR
				//
				// FIXME: this topology access needs to go away or be
				// implemented properly, as it is currently a data race.
				// for now, it is a requirement to lookup and render node info

				// resolve the LR portion of the path
				node = sm_topop->node_head;
				if (drlid != STL_LID_RESERVED && drlid != STL_LID_PERMISSIVE) {
					sm_find_node_and_port_lid(sm_topop, drlid, &node);
				}

				if (outHops > 1) {
					// from the LID-routed node, resolve the DR portion to the
					// neighbor of the destination node (trim path by 1)
					uint8_t path[outHops];
					path[0] = outHops - 1;
					memcpy(path + 1, out_dr->InitPath + 1, outHops - 1);
					node = sm_find_node_by_path(sm_topop, node, path);
				}

				if (node) {
					port = sm_find_node_port(sm_topop, node, out_dr->InitPath[outHops]);

					smGetNodeString(NULL,
						sm_valid_port(port) ? port->portData->portInfo.LID : 0,
						node, port, nodeString, NODE_STRING_MAX_LEN);
				}

				// render the raw paths
				smGetPathString(out_dr->InitPath, outHops, outPathString, PATH_STRING_MAX_LEN);
				smGetPathString(in_dr->InitPath, inHops, inPathString, PATH_STRING_MAX_LEN);
				
				IB_LOG_WARN_FMT(__func__,
					"Dropping packet, InitPaths do not match for TID[" FMT_U64
					"] on request with AID[0x%x] '%s'. DestNodeGuid[" FMT_U64
					"] Neighbor[%s] OutboundPath[%s] InboundPath[%s]",
					out_mad->base.tid, out_mad->base.aid, sm_getAttributeIdText(out_mad->base.aid), 
					sm_valid_port(port) ? port->portData->portInfo.NeighborNodeGUID : 0,
					node ? nodeString : errorStr, outPathString, inPathString);
				
				status = VSTATUS_TIMEOUT;
			}
		}
		// SLID/DLID Validation
		if (out_mad->addrInfo.dlid != in_mad->addrInfo.slid) {
			IB_LOG_WARN_FMT(__func__,
							"Dropping packet, Source lid 0x%x does not match request destination lid 0x%x for TID ["
							FMT_U64
							"] on request with MClass: 0x%02x, Method: 0x%02x, Attr: %s",
							in_mad->addrInfo.slid, out_mad->addrInfo.dlid, out_mad->base.tid,
							out_mad->base.mclass, out_mad->base.method, sm_getAttributeIdText(out_mad->base.aid));
			status = VSTATUS_TIMEOUT;
		}
	}

    // PKey Verification
#ifdef __VXWORKS__
	// Currently for the ESM, the PKey of SMI packets is not migrated up the
	// IbAccess stack.
	// Technically, SMI packets that do not have 0xffff/0x7fff (M_PKey/m_PKey)
	// will never make it past the SMI in the switch; therefore the SM will not
	// receive packets with invalid PKeys.  So, just verify the PKey of outbound
	// packets. 
    if (out_mad->addrInfo.pkey != STL_DEFAULT_FM_PKEY) {
#else
    if (out_mad->addrInfo.pkey != STL_DEFAULT_FM_PKEY
        || (in_mad->addrInfo.pkey != STL_DEFAULT_FM_PKEY && in_mad->addrInfo.pkey != STL_DEFAULT_CLIENT_PKEY)) {
#endif

		if (out_mad->base.mclass == MAD_CV_SUBN_DR) {
			DRStlSmp_t *in_dr = (DRStlSmp_t *)in_mad->data;
			uint8_t hops = MIN(in_mad->base.hopCount, IBA_MAX_PATHSIZE);
			uint8_t path[hops + 1];
			path[0] = hops;
			memcpy(path + 1, in_dr->InitPath + 1, hops);
			smGetNodeString(path, in_dr->DrSLID, NULL, NULL, nodeString, NODE_STRING_MAX_LEN);
		} else {
			smGetNodeString(NULL, in_mad->addrInfo.slid, NULL, NULL, nodeString, NODE_STRING_MAX_LEN);
		}

        IB_LOG_WARN_FMT(__func__,
                        "Dropping packet, invalid P_Key 0x%x from %s with TID ["
                        FMT_U64 "] on request with MClass: 0x%02x, Method: 0x%02x, AttrID: 0x%04x",
                        in_mad->addrInfo.pkey, nodeString, in_mad->base.tid, 
                        in_mad->base.mclass, in_mad->base.method, in_mad->base.aid);
        status = VSTATUS_TIMEOUT;
    }

	return status;
}

/**
	@param reqLength Request length.  Determines how much data is copied from @c buffer into the request MAD payload.
	@param bufferLength [in, cannot be NULL] On input, amount of space available in @c buffer for the response.
*/
Status_t
sm_send_stl_request_impl(IBhandle_t fd, uint32_t method, uint32_t aid,
						 uint32_t amod, SmpAddr_t * addr,
						 uint32_t reqLength, uint8_t * buffer, uint32_t * bufferLength,
						 uint32_t howToHandleReply, uint64_t mkey,
						 cntxt_callback_t cntxt_cb, void *cntxt_data,
						 uint32_t * madStatus)
{
	int i;
	uint64_t tid;
	uint32_t datalen;
	Filter_t filter;
	Filter_t filter2;
	Status_t status = 0, rc = 0;
	Mai_t in_mad;
	Mai_t out_mad;
	cntxt_entry_t *madcntxt = NULL;
	uint64_t timesent = 0, timercvd = 0, total_timeout = 0, timeout = 0, cumulative_timeout = 0;
	char msgbuf[256] = { 0 };
	char nodeString[NODE_STRING_MAX_LEN];
	Node_t *node = NULL;
	Port_t *port = NULL;
	boolean sendSuccess = TRUE; 


	IB_ENTER(__func__, aid, amod, addr, buffer);

	if (buffer == NULL || bufferLength == NULL) {
		IB_EXIT(__func__,  VSTATUS_ILLPARM);
		return (VSTATUS_ILLPARM);
	}

	memset(&out_mad, 0, sizeof(Mai_t));

	// 
	// Setup the headers.
	// 
	(void) mai_alloc_tid(fd, MAD_CV_SUBN_DR, &tid);

	// 
	// initialize MAI MAD header data 
	Mai_Init(&out_mad);
	AddrInfo_Init(&out_mad, addr->slid, addr->dlid, SMI_MAD_SL, STL_DEFAULT_FM_PKEY, MAI_SMI_QP, MAI_SMI_QP, 0x0);

	if (!mkey)
		mkey = sm_config.mkey;

	if (addr->path != NULL) {
		INCREMENT_COUNTER(smCounterDirectRoutePackets);
		if (!SMP_ADDR_IS_LRDR(addr)) {
			DEBUG_ASSERT(addr->path != NULL);
			DRStlMad_Init(&out_mad, method, tid, aid, amod, mkey, addr->path);
		} else {
			DEBUG_ASSERT(addr->slid != STL_LID_PERMISSIVE);
			DEBUG_ASSERT(addr->dlid != STL_LID_PERMISSIVE);
			LRDRStlMad_Init(&out_mad, method, tid, aid, amod, mkey, addr->path, addr->slid);
		}
		datalen = STL_SMP_DR_HDR_LEN;
	} else {
		INCREMENT_COUNTER(smCounterLidRoutedPackets);
		LRStlMad_Init(&out_mad, MAD_CV_SUBN_LR, method, tid, aid, amod, mkey);
		datalen = STL_SMP_LR_HDR_LEN;
	}

	// TODO why do we pad Get()s with reqLength for these specific attrs?
	if ((method == MAD_CM_SET) ||
		(method == MAD_CM_GET &&(aid == STL_MCLASS_ATTRIB_ID_SM_INFO)) ||
		(method == MAD_CM_GET &&(aid == STL_MCLASS_ATTRIB_ID_AGGREGATE))) {
		if (out_mad.base.mclass == MCLASS_SM_DIRECTED_ROUTE) {
			DRStlSmp_t *drp = (DRStlSmp_t *) out_mad.data;
			(void) memcpy((void *) drp->SMPData, (void *) buffer,
						  MIN(reqLength, STL_MAX_PAYLOAD_SMP_DR));
			datalen = MIN(STL_SMP_DR_HDR_LEN + reqLength, STL_MAD_PAYLOAD_SIZE);
		} else {
			LRStlSmp_t *lrp = (LRStlSmp_t *) out_mad.data;
			(void) memcpy((void *) lrp->SMPData, (void *) buffer,
						  MIN(reqLength, STL_MAX_PAYLOAD_SMP_LR));
			datalen = MIN(STL_SMP_LR_HDR_LEN + reqLength, STL_MAD_PAYLOAD_SIZE);
		}
	}

	//make sure datasize is set so packet isn't padded
	out_mad.datasize = datalen;

	if (howToHandleReply == WAIT_FOR_REPLY) {
		// 
		// Set the filter for catching just our reply.
		// 
		SM_Filter_Init(&filter);
		filter.value.bversion = STL_BASE_VERSION;
		filter.value.cversion = STL_SM_CLASS_VERSION;
		filter.value.mclass = MAD_CV_SUBN_LR & 0x7f;	// LR or DR
		filter.value.method = MAD_CM_GET_RESP;
		filter.value.aid = aid;
		filter.value.amod = amod;
		filter.value.tid = tid;

		filter.mask.bversion = 0xff;
		filter.mask.cversion = 0xff;
		filter.mask.mclass = 0x7f;
		filter.mask.method = 0xff;
		filter.mask.aid = 0xffff;
		filter.mask.amod = 0xffffffff;
		filter.mask.tid = 0xffffffffffffffffull;
		MAI_SET_FILTER_NAME(&filter, "sm_send_request");

		status = mai_filter_create(fd, &filter, VFILTER_SHARE | VFILTER_PURGE);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Can't create filter rc:", status);
			IB_EXIT(__func__, VSTATUS_BAD);
			return (VSTATUS_BAD);
		}
		// 
		// Set the filter for catching just our reply's error.
		// 
		SM_Filter_Init(&filter2);
		filter2.type = MAI_TYPE_ERROR;
		filter2.value.bversion = STL_BASE_VERSION;
		filter2.value.cversion = STL_SM_CLASS_VERSION;
		filter2.value.mclass = MAD_CV_SUBN_LR & 0x7f;	// LR or DR
		filter2.value.method = method;
		filter2.value.aid = aid;
		filter2.value.amod = amod;
		filter2.value.tid = tid;

		filter2.mask.bversion = 0xff;
		filter2.mask.cversion = 0xff;
		filter2.mask.mclass = 0x7f;
		filter2.mask.method = 0xff;
		filter2.mask.aid = 0xffff;
		filter2.mask.amod = 0xffffffff;
		filter2.mask.tid = 0xffffffffffffffffull;
		MAI_SET_FILTER_NAME(&filter2, "sm_send_req_err");

		status = mai_filter_create(fd, &filter2, VFILTER_SHARE | VFILTER_PURGE);
		if (status != VSTATUS_OK) {
			(void) mai_filter_delete(fd, &filter, VFILTER_SHARE | VFILTER_PURGE);
			IB_LOG_ERRORRC("Can't create filter rc:", status);
			IB_EXIT(__func__, VSTATUS_BAD);
			return (VSTATUS_BAD);
		}
		// 
		// Write out the MAD and wait for the reply.
		// 
		timeout = sm_config.rcv_wait_msec * 1000;	// time to wait for reply - convert from
													// msecs to usecs
		total_timeout = sm_popo_scale_timeout(&sm_popo, timeout * sm_config.max_retries);
		if (total_timeout == 0) {
			status = VSTATUS_TIMEOUT;
			goto skip_send;
		}

		(void) vs_time_get(&timesent);

		i = 0;
		cumulative_timeout = 0;
		do {
			if (i > 0) {
				// printf("retry: tid=0x%lx\n", out_mad.base.tid);
				INCREMENT_COUNTER(smCounterPacketRetransmits);
				/* With stepped retries, there could be excessive log messages, so display them 
				   when not using stepped retries */
				if (smDebugPerf && (sm_config.min_rcv_wait_msec == 0))
					smLogHelper(VS_LOG_INFINI_INFO, __func__, "retrying", aid, tid,
								"retry count", i + 1);
			}
			sm_smInfo.ActCount++;
			INCREMENT_COUNTER(smCounterSmPacketTransmits);
			if (sm_config.min_rcv_wait_msec) {
				/* PR 110945 - Stepped and randomized retries */
				/* For each attempt, use an increasing multiple of sm_config.min_rcv_wait_msec
				   as the timeout */
				timeout = (i + 1) * sm_config.min_rcv_wait_msec * 1000;	/* sm_config.min_rcv_wait_msec 
																		   is in msecs */
				/* Randomize timeout value within +/- 20% of the timeout */
				timeout = GET_RANDOM((timeout + (timeout / 5)), (timeout - (timeout / 5)));
				/* Make sure this timeout value doesn't cause cumulative_timeout to overshoot
				   the total_timeout */
				/* If it will cause an overshoot, then just use whatever is left from the
				   total_timeout */
				if (((cumulative_timeout + timeout) > total_timeout))
					timeout = total_timeout - cumulative_timeout;
				/* if the timeout left is less than min, set it to the min. This will cause
				   cumulative_timeout to overshoot total_timeout by a bit, but that should be
				   OK */
				if (timeout < sm_config.min_rcv_wait_msec * 1000)
					timeout = sm_config.min_rcv_wait_msec * 1000;
			}

			cumulative_timeout += timeout;
			status = mai_send_stl_timeout(fd, &out_mad, &datalen, timeout);

			if (status != VSTATUS_OK) {
				smGetNodeString(addr->path, addr->dlid, node, port, nodeString, NODE_STRING_MAX_LEN);
				IB_LOG_WARN_FMT(__func__, "Error Sending to %s. AID:[%s] TID:[" FMT_U64
						"] Status:[%s (0x%08x)]", nodeString, sm_getAttributeIdText(aid),tid, 
						sm_getMadStatusText(out_mad.base.status), out_mad.base.status);
				sendSuccess = FALSE;
				break;
			}
			memset((char *) &in_mad, 0, sizeof(in_mad));
#ifdef IB_STACK_OPENIB
			// expect packet or timeout from openib, 1 sec timeout is safety net
			status = mai_recv(fd, &in_mad, timeout + VTIMER_1S);
#else
			status = mai_recv(fd, &in_mad, timeout);
#endif
			// If the mad was an error, actually time it out
			if ((status == VSTATUS_OK) && (in_mad.type == MAI_TYPE_ERROR)) {
				status = VSTATUS_TIMEOUT;
			}

			if (status == VSTATUS_OK) {
				status = sm_validate_incoming_mad(&out_mad, &in_mad);
			}

			if (status == VSTATUS_OK) {
				if (in_mad.type != MAI_TYPE_ERROR) {
					INCREMENT_COUNTER(smCounterRxGetResp);
					INCREMENT_MAD_STATUS_COUNTERS(&in_mad);
					break;
				} else {
					// printf("error: tid=0x%lx\n", out_mad.base.tid);
					status = VSTATUS_TIMEOUT;	// for use outside loop
				}
			} else if (status != VSTATUS_TIMEOUT) {
				smLogHelper(VS_LOG_WARN, __func__, "Receive failed for", aid, tid,
							"status", status);
				break;
			}
			i++;
		}
		while (cumulative_timeout < total_timeout);

		(void) vs_time_get(&timercvd);

skip_send:
		if (status == VSTATUS_TIMEOUT) {
			if (smDebugPerf) {
				smGetNodeString(addr->path, addr->dlid, node, port, nodeString, NODE_STRING_MAX_LEN);
				IB_LOG_INFINI_INFO_FMT(__func__, "Timed out/Retries exhausted sending to %s. AID:[%s] TID:["FMT_U64
						"] Status:[%s (0x%08x)]", nodeString, sm_getAttributeIdText(aid), tid,
						sm_getMadStatusText(out_mad.base.status), out_mad.base.status);
			}
			INCREMENT_COUNTER(smCounterSmTxRetriesExhausted);
			sm_popo_report_timeout(&sm_popo, MAX(0, timercvd - timesent));
		} else {
			/* With stepped retries, there could be excessive log messages, so display them
			   when not using stepped retries */
			if (smDebugPerf && (sendSuccess==TRUE) && (sm_config.min_rcv_wait_msec == 0)) {
				i = (int) (timercvd - timesent);
				if (i > 10000) {
					if (addr->path != NULL) {
						rc = snprintf(msgbuf, 256,
								"sm_send_request: Response to AID[0x%x] '%s' took > %d msecs (%d usecs); path=",
								(int) aid, sm_getAttributeIdText(aid), (i / 1000),
								(int) (in_mad.intime ? (in_mad.intime - timesent) : 0));
						for (i = 1; i <= (int) addr->path[0] && rc < 256; i++) {
							rc += snprintf(msgbuf + rc, 256 - rc, "%2d ", addr->path[i]);
						}
						if(rc < 256)
							snprintf(msgbuf + rc, 256 - rc, " TID: 0x%"CS64"x", out_mad.base.tid);
						IB_LOG_INFO_FMT(__func__, msgbuf);
					}
				}
			}
		}

		// Delete the filter.
		rc = mai_filter_delete(fd, &filter2, VFILTER_SHARE | VFILTER_PURGE);
		if (rc != VSTATUS_OK) {
			(void) mai_filter_delete(fd, &filter, VFILTER_SHARE | VFILTER_PURGE);
			IB_LOG_ERRORRC("Can't delete filter, rc:", rc);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		rc = mai_filter_delete(fd, &filter, VFILTER_SHARE | VFILTER_PURGE);
		if (rc != VSTATUS_OK) {
			IB_LOG_ERRORRC("Can't delete filter, rc:", rc);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		// Check the status of the receive.
		if (status != VSTATUS_OK) {	// JSY - need to print status and path
			IB_EXIT(__func__, status);
			return status;
		}

		// MAD datasize is the size of the data plus mad and smp header info.
		// It may also include pad to 256 bytes with older style MAD pkts.
		// Also, it appears that actual data is padded to 8 bytes in some cases,
		// but not in others.  
		if (in_mad.base.mclass == MCLASS_SM_DIRECTED_ROUTE) {
			datalen = in_mad.datasize-STL_SMP_DR_HDR_LEN-sizeof(MAD_COMMON);
			if (datalen < *bufferLength) {
				// returned data smaller than expected
				memset(buffer, 0, *bufferLength);
			}
			DRStlSmp_t *drp = (DRStlSmp_t *) in_mad.data;
			(void) memcpy((void *) buffer, (void *) drp->SMPData, MIN(*bufferLength, datalen));
		} else {
			datalen = in_mad.datasize-STL_SMP_LR_HDR_LEN-sizeof(MAD_COMMON);
			if (datalen < *bufferLength) {
				// returned data smaller than expected
				memset(buffer, 0, *bufferLength);
			}

			LRStlSmp_t *lrp = (LRStlSmp_t *) in_mad.data;
			(void) memcpy((void *) buffer, (void *) lrp->SMPData, MIN(*bufferLength, datalen));
		}

// FIXME: cjking - Temporary patch to stop flooding console with this warning message.
// The padding length calculation is problematic for most attribute requests.
#if 0
		// 16B corner case: if the expected size is on flit (8 byte) boundary
		// the received size is 3 bytes extra (due to ICRC 4 bytes + LT byte).
		if (*bufferLength%8 == 0) {
			datalen -= 3;
		}
		// Log an error if the expected size does not match the received size.
		// Account for padding of the payload.
		uint32_t padRxLen = (datalen+7)&~0x7;
		uint32_t padExpLen = (*bufferLength+7)&~0x7;
		// We don't do this sanity check for SM_INFO due to problems with how its smaller
		// size is padded by stl_mai_to_wire(), and it will be always be padded more than this
		// calculation expects.
		if ((padRxLen != padExpLen) && (aid != STL_MCLASS_ATTRIB_ID_SM_INFO)) {	
			smGetNodeString(path, dlid, node, port, nodeString, NODE_STRING_MAX_LEN);	
			IB_LOG_WARN_FMT(__func__, "Unexpected Size MAD (datasize=%d) Received from %s. DataRx(%d), DataExp(%d), AID:%s(0x%x) Method:%02d  TID:" FMT_U64 " ",
					in_mad.datasize, nodeString, padRxLen, padExpLen, sm_getAttributeIdText(aid), aid, method, tid);
		}
#endif

		// Save off status for caller's use.
		if (madStatus != NULL)
			*madStatus = in_mad.base.status & MAD_STATUS_MASK;

		// If status is OK, then we need to check the status field in the MAD
		if (in_mad.base.status & MAD_STATUS_MASK) {
			smGetNodeString(addr->path, addr->dlid, node, port, nodeString, NODE_STRING_MAX_LEN);
			uint32_t sev = VS_LOG_WARN;
			if (aid == STL_MCLASS_ATTRIB_ID_PORT_INFO) {
				//PR: 128885 - Lower message severity to 'Info' when 'Bad MAD Status' occurs due to criteria below.	
				STL_PORT_STATES portStates = ((STL_PORT_INFO *)buffer)->PortStates;
				portStates.AsReg32 = ntoh32(portStates.AsReg32);
				if ((portStates.s.PortState == IB_PORT_ARMED) && (portStates.s.NeighborNormal == 0))
					sev = VS_LOG_INFO;
			}
			cs_log(sev, __func__,
				"Bad MAD status received from %s. TID:[" FMT_U64"] MAD Status:[%s](0x%08x) AID:%s(0x%x) Method:%02d",
				nodeString, tid, sm_getMadStatusText(in_mad.base.status), in_mad.base.status, sm_getAttributeIdText(aid), aid, method);
			IB_EXIT(__func__, in_mad.base.status);
			return VSTATUS_BAD;
		}
		IB_EXIT(__func__, status);
		return status;
	} else {
		// for async, caller already has lock
		// sending request asynchronous of reply receipt
		// get a context for this report mad

		total_timeout = sm_popo_scale_timeout(&sm_popo, sm_async_send_rcv_cntxt.totalTimeout);
		if (total_timeout == 0) {
			cntxt_cb(NULL, VSTATUS_TIMEOUT, cntxt_data, NULL);
			return VSTATUS_TIMEOUT;
		}

		if ((madcntxt = cs_cntxt_get_nolock(&out_mad, &sm_async_send_rcv_cntxt, FALSE)) == NULL) {
			// could not get a context, this should not happen, discard packet
			IB_LOG_ERROR0("Error allocating an SM async send/rcv context");
			if (cntxt_cb && cntxt_data)
				cntxt_cb(NULL, VSTATUS_BAD, cntxt_data, NULL);
			return VSTATUS_BAD;
		} else {
			cs_cntxt_set_callback(madcntxt, cntxt_cb, cntxt_data);
			madcntxt->totalTimeout = total_timeout;
			// send request out
			// if (smDebugPerf) {
			// smLogHelper(VS_LOG_WARN, "sm_send_request", "async SM send/rcv", aid, tid,
			// "cntx->index", (int)madcntxt->index);
			// }
			if (howToHandleReply == WANT_REPLY_ON_QUEUE) {
				// tell topology_rcv that response is wanted on queue
				madcntxt->senderWantsResponse = 1;
			}
			if ((status =
				 cs_cntxt_send_mad_nolock(madcntxt, &sm_async_send_rcv_cntxt)) != VSTATUS_OK) {
				// cs_cntxt_age will timeout the request and retry it later
				smLogHelper(VS_LOG_WARN, __func__, "async SM send/rcv error", aid, tid,
							"status", status);
			}
			// increment Sm activity count
			sm_smInfo.ActCount++;
			return status;
		}
	}
}


Status_t
sm_send_stl_request(IBhandle_t fd, uint32_t method, uint32_t aid, uint32_t amod, SmpAddr_t * addr,
	uint8_t * buffer, uint32_t * bufferLength, uint64_t mkey, uint32_t * madStatus)
{
	uint32_t reqLength = (bufferLength ? *bufferLength : 0);
	// send the request and wait for a response
	return sm_send_stl_request_impl(
		fd, method, aid, amod, addr, reqLength, buffer, bufferLength,
		WAIT_FOR_REPLY, mkey, NULL, NULL, madStatus);
}

Status_t
sm_send_stl_request_async_dispatch(IBhandle_t fd, uint32_t method, uint32_t aid, uint32_t amod,
								   uint8_t * path, uint8_t * buffer, uint32_t * bufferLength,
								   uint64_t mkey, Node_t * nodep, sm_dispatch_t * disp)
{
	Status_t status;
	sm_dispatch_req_t *req;
	sm_dispatch_send_params_t sendParams;

	sendParams.fd = fd;
	sendParams.method = method;
	sendParams.aid = aid;
	sendParams.amod = amod;
	sendParams.path = path;
	sendParams.slid = STL_LID_PERMISSIVE;
	sendParams.dlid = STL_LID_PERMISSIVE;
	sendParams.mkey = mkey;
	sendParams.bversion = STL_BASE_VERSION;
	sendParams.bufferLength = *bufferLength;

	memcpy((void *) sendParams.buffer, (void *) buffer, sizeof(sendParams.buffer));

	status = sm_dispatch_new_req(disp, &sendParams, nodep, &req);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to create dispatch request, rc:",
					  status);
		return status;
	}

	sm_dispatch_enqueue(req);

	return VSTATUS_OK;
}

Status_t
sm_send_stl_request_async_dispatch_lr(IBhandle_t fd, uint32_t method, uint32_t aid,
									  uint32_t amod, STL_LID slid, STL_LID dlid,
									  uint8_t * buffer, uint32_t * bufferLength, uint64_t mkey,
									  Node_t * nodep, sm_dispatch_t * disp)
{
	Status_t status;
	sm_dispatch_req_t *req;
	sm_dispatch_send_params_t sendParams;

	sendParams.fd = fd;
	sendParams.method = method;
	sendParams.aid = aid;
	sendParams.amod = amod;
	sendParams.path = NULL;
	sendParams.slid = slid;
	sendParams.dlid = dlid;
	sendParams.mkey = mkey;
	sendParams.bversion = STL_BASE_VERSION;
	sendParams.bufferLength = *bufferLength;

	memcpy((void *) sendParams.buffer, (void *) buffer, sizeof(sendParams.buffer));

	status = sm_dispatch_new_req(disp, &sendParams, nodep, &req);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to create dispatch request, rc:",
					  status);
		return status;
	}

	sm_dispatch_enqueue(req);

	return VSTATUS_OK;
}

//----------------------------------------------------------------------------//

Status_t
sm_get_stl_attribute(IBhandle_t fd, uint32_t aid, uint32_t amod, SmpAddr_t * addr,
	uint8_t * buffer, uint32_t * bufferLength)
{
	Status_t status;

	IB_ENTER(__func__, aid, amod, addr, buffer);
	status = sm_send_stl_request(fd, MAD_CM_GET, aid, amod, addr, buffer, bufferLength, 0, NULL);
	IB_EXIT(__func__, status);
	return (status);
}

Status_t
sm_set_stl_attribute(IBhandle_t fd, uint32_t aid, uint32_t amod, SmpAddr_t * addr,
	uint8_t * buffer, uint32_t * bufferLength, uint64_t mkey)
{
	Status_t status;

	IB_ENTER(__func__, aid, amod, addr, buffer);
	status = sm_send_stl_request(fd, MAD_CM_SET, aid, amod, addr, buffer, bufferLength, mkey, NULL);
	IB_EXIT(__func__, status);
	return (status);
}

Status_t
sm_set_stl_attribute_mad_status(IBhandle_t fd, uint32_t aid, uint32_t amod, SmpAddr_t * addr,
	uint8_t * buffer, uint32_t * bufferLength, uint64_t mkey, uint32_t * madStatus)
{
	Status_t status;

	IB_ENTER(__func__, aid, amod, addr, buffer);
	status = sm_send_stl_request(fd, MAD_CM_SET, aid, amod, addr, buffer, bufferLength, mkey, madStatus);
	IB_EXIT(__func__, status);
	return (status);
}

Status_t
sm_set_stl_attribute_async_dispatch(IBhandle_t fd, uint32_t aid, uint32_t amod, uint8_t * path,
									uint8_t * buffer, uint32_t * bufferLength, uint64_t mkey,
								   Node_t * nodep, sm_dispatch_t * disp)
{
	Status_t status;

	IB_ENTER(__func__, aid, amod, path, buffer);
	status =
		sm_send_stl_request_async_dispatch(fd, MAD_CM_SET, aid, amod, path, buffer,
										   bufferLength, mkey, nodep, disp);
	IB_EXIT(__func__, status);
	return (status);
}

Status_t
sm_set_stl_attribute_async_dispatch_lr(IBhandle_t fd, uint32_t aid, uint32_t amod,
									   STL_LID slid, STL_LID dlid, uint8_t * buffer,
									   uint32_t * bufferLength, uint64_t mkey, Node_t * nodep,
									   sm_dispatch_t * disp)
{
	Status_t status;

	IB_ENTER(__func__, aid, amod, dlid, buffer);
	status =
		sm_send_stl_request_async_dispatch_lr(fd, MAD_CM_SET, aid, amod, slid, dlid, buffer,
											  bufferLength, mkey, nodep, disp);
	IB_EXIT(__func__, status);
	return (status);
}

/*
 *	Get SMInfo from the SM at port passed in.
 */
static Status_t
sm_getSmInfo(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	STL_SM_INFO smInfo;
	Status_t status;
	STL_SMINFO_RECORD sminforec = { {0}, 0 };

	IB_ENTER(__func__, nodep, portp, 0, 0);

	/* 
	 *  Fetch the SMInfo from the other port.
	 */
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(PathToPort(nodep, portp));
	status = SM_Get_SMInfo(fd_topology, 0, &addr, &smInfo);
	if (status != VSTATUS_OK) {
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "failed to get SmInfo from remote SM %s : " FMT_U64,
							   sm_nodeDescString(nodep), portp->portData->guid);
		/* remove the SM from our list */
		(void) sm_dbsync_deleteSm(portp->portData->guid);
		status = sm_popo_port_error(&sm_popo, topop, portp, status);
		IB_EXIT(__func__, status);
		return status;
	} else {
		/* 
		 * add the found Sm to the topology
		 */
		sminforec.RID.LID = portp->portData->portInfo.LID;
		sminforec.SMInfo = smInfo;
		(void) sm_dbsync_addSm(nodep, portp, &sminforec);
		if (smInfo.SM_Key != sm_smInfo.SM_Key) {
			IB_LOG_WARN_FMT(__func__,
							"Remote SM %s [" FMT_U64 "] does not have the proper SM_Key["
							FMT_U64 "]", sm_nodeDescString(nodep), portp->portData->guid,
							smInfo.SM_Key);
		}
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}								/* sm_getSmInfo */


int
sm_check_node_cache_valid(Node_t * nodep)
{
	uint64_t curtime = 0ll;

	if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
		(void) vs_time_get(&curtime);

		if ((nodep->nonRespTime == 0) && (sm_config.non_resp_tsec != 0)) {
			nodep->nonRespTime = curtime;
			return 1;
		} else {
			if (((curtime - nodep->nonRespTime) < (sm_config.non_resp_tsec * VTIMER_1S)) ||
				(nodep->nonRespCount < sm_config.non_resp_max_count)) {
				return 1;
			}
		}
	}

	return 0;
}

/* Attempts to find a port's peer information from the old topoolgy. It then checks
   this info against use definable timers to verify if the cache is valid. */
int
sm_check_node_cache(Node_t * cnp, Port_t * cpp, Node_t ** nodep, Port_t ** portp)
{
	Node_t *cache_nodep = NULL;
	Port_t *cache_portp = NULL;

	if (!(cnp && cpp)) {
		return 0;
	}

	if (topology_passcount &&
		(cache_portp = sm_find_port_peer(&old_topology, cnp->nodeInfo.NodeGUID, cpp->index)) &&
		(cache_nodep = sm_find_port_node(&old_topology, cache_portp))) {
		if (cache_nodep->nodeInfo.NodeType == NI_TYPE_CA) {
			if (sm_check_node_cache_valid(cache_nodep)) {
				cache_nodep->nonRespCount++;
				*nodep = cache_nodep;
				*portp = cache_portp;
				IB_LOG_WARN_FMT(__func__,
								"Port non-responsive using cached info GUID[" FMT_U64
								"] Port: %d %s. If this persists verify health of node!",
								cache_nodep->nodeInfo.NodeGUID, cache_portp->index,
								sm_nodeDescString(cache_nodep));
				return 1;
			} else {
				IB_LOG_WARN_FMT(__func__,
								"Dropping Port from topology: Link is up but cache has expired GUID["
								FMT_U64 "] Port: %d %s. Check health of node!",
								cache_nodep->nodeInfo.NodeGUID, cache_portp->index,
								sm_nodeDescString(cache_nodep));
			}
		}
	}
	return 0;
}

int
sm_find_cached_node_port(Node_t * cnp, Port_t* cpp, Node_t ** nodep, Port_t ** portp) {
	Node_t *cache_nodep = NULL;
	Port_t *cache_portp = NULL;

	if(!(sm_config.use_cached_node_data || sm_config.use_cached_hfi_node_data)) {
		return 0;
	}

	if (!(cnp && cpp)) {
		return 0;
	}

	if(topology_passcount &&
		(cache_nodep = sm_find_guid(&old_topology, cnp->nodeInfo.NodeGUID)) &&
		(cache_portp = sm_find_node_port(&old_topology, cache_nodep, cpp->index))) {
		*nodep = cache_nodep;
		*portp = cache_portp;
		return 1;
	}

	return 0;
}

int
sm_find_cached_neighbor(Node_t * cnp, Port_t * cpp, Node_t ** nodep, Port_t ** portp)
{
	Node_t *cache_nodep = NULL;
	Port_t *cache_portp = NULL;

	if(!(sm_config.use_cached_node_data || sm_config.use_cached_hfi_node_data)) {
		return 0;
	}

	if (!(cnp && cpp)) {
		return 0;
	}

	if (topology_passcount &&
		(cache_portp = sm_find_port_peer(&old_topology, cnp->nodeInfo.NodeGUID, cpp->index)) &&
		(cache_nodep = sm_find_port_node(&old_topology, cache_portp))) {
		if (!cache_nodep->nodeDescChgTrap) {
			*nodep = cache_nodep;
			*portp = cache_portp;
			return 1;
		}
		else {
			// Invalidate the cache. Node would re-queried, Cache would be updated.
			// Reset the flag.
			cache_nodep->nodeDescChgTrap = 0;
		}
	}
	return 0;
}

//----------------------------------------------------------------------------//

Status_t
sm_mark_new_endnode(Node_t * nodep)
{
	while (new_endnodesInUse.nbits_m <= nodep->index) {
		if (!bitset_resize(&new_endnodesInUse, new_endnodesInUse.nbits_m + SM_NODE_NUM))
			return VSTATUS_BAD;
	}

	bitset_set(&new_endnodesInUse, nodep->index);

	return VSTATUS_OK;
}

static void
check_for_new_endnode(STL_NODE_INFO * nodeInfo, Node_t * nodep, Node_t * oldnodep)
{
	Port_t *oldportp;
	int match;

	if (oldnodep == NULL)
		oldnodep = sm_find_guid(&old_topology, nodeInfo->NodeGUID);
	if (!oldnodep) {
		// this is a new node; mark it
		if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
			topology_changed = 1;
	} else {
		// old node did exist; check the ports for a match
		match = 0;
		for_all_ports(oldnodep, oldportp) {
			if (oldportp
				&& oldportp->state > IB_PORT_DOWN
				&& oldportp->index == nodeInfo->u1.s.LocalPortNum) {
				match = 1;
				break;
			}
		}
		if (!match) {
			// found no matches
			// this isn't a new node, but it has a new port, so mark it
			if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
				topology_changed = 1;
		}
	}
}

uint32_t
sm_Node_release_pgft(Node_t * node)
{
	if (node->pgft) {
		vs_pool_free(&sm_pool, node->pgft);
		node->pgft = NULL;
		node->pgftSize = 0;
	}
	return 0;
}

size_t
sm_Node_get_pgft_size(const Node_t * node)
{
	return (size_t)(node->pgftSize);
}

size_t
sm_Node_compute_pgft_size(const Node_t * node)
{
	// The PGFT is generally the same length as the LFT, but
	// capped at PortGroupFDBCap (earlier gen1 FM hardcoded at 8K).
	uint32 pgftCap = node->switchInfo.PortGroupFDBCap ? node->switchInfo.PortGroupFDBCap :
						DEFAULT_MAX_PGFT_BLOCK_NUM * NUM_PGFT_ELEMENTS_BLOCK;
	
	if (node->switchInfo.LinearFDBTop < pgftCap) pgftCap = node->switchInfo.LinearFDBTop+1;
	return ROUNDUP(pgftCap, NUM_PGFT_ELEMENTS_BLOCK);
}

const PORT *
sm_Node_get_pgft(const Node_t * node)
{
	return node->pgft;
}

PORT *
sm_Node_get_pgft_wr(Node_t * node)
{
	if (!node->switchInfo.CapabilityMask.s.IsAdaptiveRoutingSupported)
		return NULL;

	size_t pgftLen = sm_Node_compute_pgft_size(node);
	if (!node->pgft) {
		Status_t s = vs_pool_alloc(&sm_pool, pgftLen, (void *) &node->pgft);
		if (s == VSTATUS_OK) {
			memset((void *) node->pgft, 0xFF, pgftLen);
			node->pgftSize = (uint32_t) pgftLen;
		}
	}
	else if (node->pgftSize < pgftLen) {
		PORT * newPgft = NULL;
		Status_t s = vs_pool_alloc(&sm_pool, pgftLen, (void *)&newPgft);
		if (s != VSTATUS_OK) {
			if (!newPgft)
				vs_pool_free(&sm_pool, newPgft);
			return NULL;
		}
		memcpy(newPgft, node->pgft, node->pgftSize);
		memset(newPgft + node->pgftSize, 0xFF, pgftLen - node->pgftSize);
		vs_pool_free(&sm_pool, node->pgft);
		node->pgft = newPgft;
		node->pgftSize = pgftLen;
	}

	return node->pgft;
}

PORT *
sm_Node_copy_pgft(Node_t * dest, const Node_t * src)
{
	if (!src->pgft)
		return dest->pgft;

	if (dest->pgft && sm_Node_get_pgft_size(dest) < sm_Node_get_pgft_size(src)) {
		sm_Node_release_pgft(dest);
	}

	if (!dest->pgft) {
		Status_t s = vs_pool_alloc(&sm_pool, sm_Node_get_pgft_size(src), (void *) &dest->pgft);

		if (s != VSTATUS_OK)
			return NULL;
		dest->pgftSize = sm_Node_get_pgft_size(src);
	}

	if (dest->pgft) {
		memcpy((void *) dest->pgft, src->pgft, sm_Node_get_pgft_size(dest));
		dest->pgftSize = src->pgftSize;
	}

	return dest->pgft;
}

void
sm_Node_invalidate_pgs(Node_t * node)
{
	if (node->nodeInfo.NodeType != NI_TYPE_SWITCH)
		return;

	if (node->pgt) {
		memset(node->pgt, 0, node->switchInfo.PortGroupTop * sizeof(STL_PORTMASK));
		node->switchInfo.PortGroupTop = node->pgtLen = 0;
	}

	if (node->pgft) {
		memset(node->pgft, 0xFF, node->pgftSize * sizeof(node->pgft[0]));
	}
}

void
sm_log_predef_field_violation(Topology_t* topop, uint32_t logQuarantineReasons, boolean forceLogAsWarn,
	Node_t *cnp, Port_t *cpp, STL_NODE_INFO* nodeInfo, STL_NODE_DESCRIPTION* nodeDesc, PortSelector* validationPort)
{
	int nodeDescViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_DESC;
	int nodeGuidViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_GUID;
	int portGuidViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_PORT_GUID;
	int undefinedLinkViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK;
	boolean logError = !forceLogAsWarn && // Override and Force to only Log as a Warn as the port will be bounced
		((nodeDescViolation && (sm_config.preDefTopo.fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_ENABLED)) ||
		(nodeGuidViolation && (sm_config.preDefTopo.fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED)) ||
		(portGuidViolation && (sm_config.preDefTopo.fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED)) ||
		(undefinedLinkViolation && (sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_ENABLED)));

	char expectedNodeDescPrint[128] = {0};
	char expectedNodeGuidPrint[64] = {0};
	char expectedPortGuidPrint[64] = {0};
	char undefinedLinkPrint[64] = {0};
	char foundPortGuidPrint[64] = {0};
	char basePortGuidPrint[64] = {0};

	// We shouldn't log if we don't have any of these
	if (cnp == NULL || cpp == NULL || nodeInfo == NULL || nodeDesc == NULL ||
		(validationPort == NULL && !undefinedLinkViolation)) {
		return;
	}

	snprintf(basePortGuidPrint, sizeof(basePortGuidPrint), " PortGUID: "FMT_U64", ", cpp->portData->guid);
	snprintf(foundPortGuidPrint, sizeof(foundPortGuidPrint), " PortGUID: "FMT_U64", ", nodeInfo->PortGUID);


	// If we have hit our log threshold, spit out a final message but then stop logging
	if(sm_config.preDefTopo.logMessageThreshold != 0 && 
			topop->preDefLogCounts.totalLogCount == sm_config.preDefTopo.logMessageThreshold) {
		IB_LOG_WARN_FMT(__func__,
			"Pre-Defined Topology: Log message threshold of %d messages per sweep has been reached. "
			"Suppressing further topology mismatch information.",
			sm_config.preDefTopo.logMessageThreshold);
		topop->preDefLogCounts.totalLogCount++;
	}

	// If we have an undefined link violation, don't log any of the other fields as they would be invalid anyways.
	if (undefinedLinkViolation) {
		nodeDescViolation = 0;
		nodeGuidViolation = 0;
		portGuidViolation = 0;

		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_WARN) {
			snprintf(undefinedLinkPrint, sizeof(undefinedLinkPrint), " No Link (Warn)");
			topop->preDefLogCounts.undefinedLinkWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(undefinedLinkPrint, sizeof(undefinedLinkPrint), " No Link (Quarantined)");
			topop->preDefLogCounts.undefinedLinkQuarantined++;
		}
	}

	// NodeDesc Violation Logging
	if (nodeDescViolation && (validationPort != NULL)) {
		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_WARN) {
			snprintf(expectedNodeDescPrint, sizeof(expectedNodeDescPrint), " NodeDesc: %s (Warn)", validationPort->NodeDesc);
			topop->preDefLogCounts.nodeDescWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(expectedNodeDescPrint, sizeof(expectedNodeDescPrint), " NodeDesc: %s (Quarantined)", validationPort->NodeDesc);
			topop->preDefLogCounts.nodeDescQuarantined++;
		}
	}

	// NodeGUID Violation Logging
	if (nodeGuidViolation) {
		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_WARN) {
			snprintf(expectedNodeGuidPrint, sizeof(expectedNodeGuidPrint),
				" NodeGUID: "FMT_U64" (Warn)", (validationPort ? validationPort->NodeGUID : 0));
			topop->preDefLogCounts.nodeGuidWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(expectedNodeGuidPrint, sizeof(expectedNodeGuidPrint),
				" NodeGUID: "FMT_U64" (Quarantined)", (validationPort ? validationPort->NodeGUID : 0));
			topop->preDefLogCounts.nodeGuidQuarantined++;
		}
	}

	// PortGUID Violation Logging
	if (portGuidViolation) {
		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.portGuid == FIELD_ENF_LEVEL_WARN) {
			snprintf(expectedPortGuidPrint, sizeof(expectedPortGuidPrint),
				" PortGUID: "FMT_U64" (Warn)", (validationPort ? validationPort->PortGUID : 0));
			topop->preDefLogCounts.portGuidWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(expectedPortGuidPrint, sizeof(expectedPortGuidPrint),
				" PortGUID: "FMT_U64" (Quarantined)", (validationPort ? validationPort->PortGUID : 0));
			topop->preDefLogCounts.portGuidQuarantined++;
		}
	}

	if (sm_config.preDefTopo.logMessageThreshold != 0 &&
		topop->preDefLogCounts.totalLogCount > sm_config.preDefTopo.logMessageThreshold) {
		return;
	}

	// If something is getting quarantined, log it as an error, otherwise log it as a warning.
	if (logError) {
		IB_LOG_ERROR_FMT(__func__, "Pre-Defined Topology: Found %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			" connected to %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			", but expected:%s%s"/* */"%s%s"/* */"%s%s",
			nodeDesc->NodeString, nodeInfo->NodeGUID, StlNodeTypeToText(nodeInfo->NodeType),
				nodeInfo->PortGUID ? foundPortGuidPrint : " ", nodeInfo->u1.s.LocalPortNum,
			sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, StlNodeTypeToText(cnp->nodeInfo.NodeType),
				cpp->portData->guid ? basePortGuidPrint : " ", cpp->index,
			expectedNodeDescPrint, nodeDescViolation && (nodeGuidViolation || portGuidViolation) ? "," : "",
			expectedNodeGuidPrint, nodeGuidViolation && portGuidViolation ? "," : "",
			expectedPortGuidPrint, undefinedLinkPrint);
	} else {
		IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Found %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			" connected to %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			", but expected:%s%s"/* */"%s%s"/* */"%s%s",
			nodeDesc->NodeString, nodeInfo->NodeGUID, StlNodeTypeToText(nodeInfo->NodeType),
				nodeInfo->PortGUID ? foundPortGuidPrint : " ", nodeInfo->u1.s.LocalPortNum,
			sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, StlNodeTypeToText(cnp->nodeInfo.NodeType),
				cpp->portData->guid ? basePortGuidPrint : " ", cpp->index,
			expectedNodeDescPrint, nodeDescViolation && (nodeGuidViolation || portGuidViolation) ? "," : "",
			expectedNodeGuidPrint, nodeGuidViolation && portGuidViolation ? "," : "",
			expectedPortGuidPrint, undefinedLinkPrint);
	}

	topop->preDefLogCounts.totalLogCount++;
}

static int portsel_match(PortSelector *ps, const char *nd, int pnum)
{
	return (ps && strncmp(nd, ps->NodeDesc,
		STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0 && pnum == ps->PortNum);
}

/*
 * Find all ExpectedLink elems in @fdp given NodeDesc and PortNum values.
 *
 * Can do partial matches if either @ndesc1 or @ndesc2 is NULL. Writes
 * up to @elSize found links to @elOut. Writes in order discovered so
 * if there are more than @elSize hits, additional matching
 * ExpectedLinks will not be written.
 *
 * @returns number of matches in fdp->ExpectedLinks, even if greater
 * than @elSize. Does not invalidate @elOut; only up to @return value
 * pointers in @elOut are guaranteed to be valid.
 */
static int
find_exp_links_by_desc_and_port(FabricData_t * fdp,
	const char *ndesc1, int pnum1,
	const char *ndesc2, int pnum2,
	ExpectedLink **elOut, int elSize)
{
	int hits = 0;
	LIST_ITEM *it;

	if (!ndesc1 && !ndesc2)
		return 0;

	for (it = QListHead(&fdp->ExpectedLinks); it != NULL;
		it = QListNext(&fdp->ExpectedLinks, it)) {
		int n1match, n2match; // Matching side
		n1match = n2match = 0;
		ExpectedLink *el = PARENT_STRUCT(it, ExpectedLink, ExpectedLinksEntry);

		if (!el->portselp1 || !el->portselp2)
			continue;

		if (ndesc1) {
			if (portsel_match(el->portselp1, ndesc1, pnum1))
				n1match = 1;
			else if (portsel_match(el->portselp2, ndesc1, pnum1))
				n1match = 2;
		}

		if (ndesc2) {
			if (portsel_match(el->portselp1, ndesc2, pnum2))
				n2match = 1;
			else if (portsel_match(el->portselp2, ndesc2, pnum2))
				n2match = 2;
		}

		// Both are defined, require complete match
		if (ndesc1 && ndesc2) {
			if (!n1match || !n2match)
				continue;
			if (n1match == n2match)
				continue;
		} else if (ndesc1 && !n1match) {
			continue;
		} else if (ndesc2 && !n2match)
			continue;

		if (hits < elSize)
			elOut[hits] = el;
		++hits;
	}

	return hits;
}

/*
* Validates the SM NodeGuid and PortGuid.
*
* Checks the ExpectedSM link for matching NodeGuid and PortGuid
*
* @returns if the SM is authentic or not
*/
int sm_validate_predef_sm_field(Topology_t* topop, FabricData_t* pdtop, STL_NODE_INFO* nodeInfo, STL_NODE_DESCRIPTION* nodeDesc)
{
	FSTATUS status;
	SmPreDefTopoXmlConfig_t *pdtCfg = &sm_config.preDefTopo;
	int authentic = 1;

	status = FindExpectedSMByNodeGuid(pdtop, nodeInfo->NodeGUID);
	if(status != FSUCCESS) {
		if(pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_WARN) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: %s (NodeGUID: " FMT_U64") was not found in input file (Warn).",
							(char*) nodeDesc->NodeString, nodeInfo->NodeGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.nodeGuidWarn++;
		}
		else if(pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_ERROR_FMT(__func__, "Pre-Defined Topology: %s (NodeGUID: " FMT_U64") was not found in input file (Quarantined).",
							(char*) nodeDesc->NodeString, nodeInfo->NodeGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.nodeGuidQuarantined++;
			authentic = 0;
		}
		// If we have hit our log threshold, spit out a final message but then stop logging
		if(topop->preDefLogCounts.totalLogCount == pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Log message threshold of %d messages per sweep has been reached. Suppressing further topology mismatch information.", 
							pdtCfg->logMessageThreshold);
			topop->preDefLogCounts.totalLogCount++;
		}
	}

	status = FindExpectedSMByPortGuid(pdtop, nodeInfo->PortGUID);
	if(status != FSUCCESS) {
		if(pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_WARN) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: %s (PortGUID: "FMT_U64") was not found in input file (Warn).",
							(char*) nodeDesc->NodeString, nodeInfo->PortGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.portGuidWarn++;
		}
		else if(pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_ERROR_FMT(__func__, "Pre-Defined Topology: %s (PortGUID: "FMT_U64") was not found in input file (Quarantined).",
							(char*) nodeDesc->NodeString, nodeInfo->PortGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.portGuidQuarantined++;
			authentic = 0;
		}
		// If we have hit our log threshold, spit out a final message but then stop logging
		if(topop->preDefLogCounts.totalLogCount == pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Log message threshold of %d messages per sweep has been reached. Suppressing further topology mismatch information.", 
							pdtCfg->logMessageThreshold);
			topop->preDefLogCounts.totalLogCount++;
		}
	}

	return authentic;
}


int
sm_validate_predef_fields(Topology_t* topop, FabricData_t* pdtop, Node_t * cnp, Port_t * cpp,
	STL_NODE_INFO* nodeInfo, STL_NODE_DESCRIPTION* nodeDesc, uint32_t* quarantineReasons,
	STL_EXPECTED_NODE_INFO* expNodeInfo, boolean forceLogAsWarn)
{
	int authentic = 1;
	ExpectedLink* validationLink;
	PortSelector* validationPort;
	uint8_t linkSide;
	uint32_t logQuarantineReasons = 0x00000000;
	SmPreDefTopoXmlConfig_t *pdtCfg = &sm_config.preDefTopo;

	if(pdtop == NULL || quarantineReasons == NULL || nodeInfo == NULL || nodeDesc == NULL) {
		authentic = 0;
		return authentic;
	}

	// If either of these are null, we're on the SM node looking at our self at the start of a sweep, so just make sure that we show up in the topology.
	// The logging for this is done separately from sm_log_predef_violation as this is a weird edge case
	if(cnp == NULL || cpp == NULL) {
		authentic = sm_validate_predef_sm_field(topop, pdtop, nodeInfo, nodeDesc);
		return authentic;
	}

	// UndefinedLink Validation
	validationLink = FindExpectedLinkByOneSide(pdtop, cnp->nodeInfo.NodeGUID, cpp->index, &linkSide);

	// Special case: match by NodeDesc and PortNum
	if(validationLink == NULL &&
		pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_DISABLED) {
		int hitCnt;
		int forcePrint = 0;

		if(pdtCfg->logMessageThreshold != 0 &&
			topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Validation link not found using (NodeGUID: "
				FMT_U64", PortNum: %d). Looking up link by (NodeDesc: %s,"
				" PortNum: %d). False matches may occur if node descriptions are"
				" not unique.", cnp->nodeInfo.NodeGUID, cpp->index,
				sm_nodeDescString(cnp), cpp->index);
			topop->preDefLogCounts.totalLogCount++;
			forcePrint = (topop->preDefLogCounts.totalLogCount >= pdtCfg->logMessageThreshold);
		}

		hitCnt = find_exp_links_by_desc_and_port(pdtop,
			(char*)cnp->nodeDesc.NodeString, cpp->index, NULL, 0,
			&validationLink, 1);

		if (hitCnt > 1 && pdtCfg->logMessageThreshold != 0 && (forcePrint ||
			topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold)) {
			IB_LOG_WARN_FMT(__func__, "Mutlitple expected links matched by node"
				" desc and port number. NodeGUID: "FMT_U64", NodeDesc: %s,"
				" PortNum: %d", cnp->nodeInfo.NodeGUID, sm_nodeDescString(cnp),
				cpp->index);
			if (!forcePrint)
				topop->preDefLogCounts.totalLogCount++;
		}

		if (validationLink) {
			if (strncmp(validationLink->portselp1->NodeDesc,
				(char*) cnp->nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
				linkSide = 1;
			else
				linkSide = 2;
		}
	}

	//Found a potential match, now verify it is connected to correct PortNum on other side
	if(validationLink) {
		validationPort = linkSide == 1 ? validationLink->portselp2 : validationLink->portselp1;
		if(validationPort->PortNum != nodeInfo->u1.s.LocalPortNum)
			validationLink = NULL;
	}

	if (validationLink == NULL) {
		if(pdtCfg->fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_DISABLED)
			return authentic;

		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK;
		if(pdtCfg->fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_ENABLED) {
			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK;
			authentic = 0;
		}
			
		sm_log_predef_field_violation(topop, logQuarantineReasons, forceLogAsWarn,
			cnp, cpp, nodeInfo, nodeDesc, NULL);

		return authentic;
	}


	// NodeGUID Validation
	if(pdtCfg->fieldEnforcement.nodeGuid != FIELD_ENF_LEVEL_DISABLED && nodeInfo->NodeGUID != validationPort->NodeGUID) {
		if(pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(expNodeInfo)
				expNodeInfo->nodeGUID = validationPort->NodeGUID;

			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_GUID;
			authentic = 0;
		}
		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_GUID;
	}

	if(pdtCfg->fieldEnforcement.nodeDesc != FIELD_ENF_LEVEL_DISABLED &&
			(validationPort->NodeDesc == NULL ||
			strncmp((char*) nodeDesc->NodeString, validationPort->NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE))) {
		if(pdtCfg->fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_ENABLED) {
			if(expNodeInfo){
				if(validationPort->NodeDesc == NULL)
					strncpy((char*) expNodeInfo->nodeDesc.NodeString, "<UNDEFINED>", STL_NODE_DESCRIPTION_ARRAY_SIZE);
				else
					strncpy((char*) expNodeInfo->nodeDesc.NodeString, validationPort->NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE);
			}

			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_DESC;
			authentic = 0;
		}
		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_DESC;
	}
	
	// PortGUID Validation
	// Only valid for HFIs, as Switches only have a single PortGUID for Port 0.
	// If we expected a switch and found an HFI, do a PortGUID comparison as it is technically an invalid PortGUID (we were expecting 0)
	if(pdtCfg->fieldEnforcement.portGuid != FIELD_ENF_LEVEL_DISABLED &&
			((validationPort->NodeType == NI_TYPE_CA && nodeInfo->PortGUID != validationPort->PortGUID) ||
			(validationPort->NodeType == NI_TYPE_SWITCH && nodeInfo->NodeType == NI_TYPE_CA && nodeInfo->PortGUID != validationPort->PortGUID))) {
		if(pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(expNodeInfo)
				expNodeInfo->portGUID = validationPort->PortGUID;

			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_PORT_GUID;
			authentic = 0;
		}
		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_PORT_GUID;
	}

	if(logQuarantineReasons) {
		sm_log_predef_field_violation(topop, logQuarantineReasons, forceLogAsWarn,
			cnp, cpp, nodeInfo, nodeDesc, validationPort);
	}

	return authentic;
}

static Status_t
sm_GetPortInfoLoop(Topology_t * topop, Node_t * nodep, bitset_t * needPortInfo, STL_PORT_INFO * newPortInfos)
{
	Status_t status;
	int i;

	for (i = bitset_find_first_one(needPortInfo); i != -1; i = bitset_find_next_one(needPortInfo, i + 1)) {
		if ((status = SM_Get_PortInfo(fd_topology, 1<<24 | STL_SM_CONF_START_ATTR_MOD | (uint32_t)i, nodep->path, newPortInfos + i)) == VSTATUS_OK) {
			bitset_clear(needPortInfo, i);
		} else {
			Port_t * portp = sm_find_node_port(topop, nodep, i);
			if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
				return status;
		}
	}

	return VSTATUS_OK;
}

/*
 * Perform a Get(Aggregate) of PortInfo's for a node.
 *
 * Only request the PortInfo's for the ports select in the @ports
 * bitmask. Resulting PortInfo structures are put in the @res array. Note
 * that @res will be indexed into directly by port number, hence filling only 
 * entries that we're actually requesting.
 *
 * Upon completion, @ports bitmask will leave set bits corresponding to ports that
 * had a error in getting their corresponding PortInfo.
 */
static Status_t
sm_GetPortInfoAggr(Topology_t *topop, Node_t *nodep, bitset_t *ports, STL_PORT_INFO *res)
{
	const size_t blockSize = sizeof(STL_AGGREGATE) + 8*((sizeof(STL_PORT_INFO) + 7)/8);
	const size_t portsPerAggr = (STL_MAX_PAYLOAD_SMP_DR / blockSize);
	uint8_t buffer[STL_MAX_PAYLOAD_SMP_DR];
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	int port;
	bitset_t local_ports;
	uint8_t i = 0;
	Status_t status = VSTATUS_OK;

	memset(buffer, 0, sizeof(buffer));
	bitset_init(&sm_pool, &local_ports, MAX_STL_PORTS + 1);
	bitset_copy(&local_ports, ports);

	bitset_clear_all(ports);

	port = bitset_find_first_one(&local_ports);

	while (port != -1) {

		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PORT_INFO;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.Reserved = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_PORT_INFO) + 7)/8;
		
		aggrHdr->AttributeModifier = (1<<24) | STL_SM_CONF_START_ATTR_MOD | (uint32_t)port;
		*((STL_PORT_INFO*)aggrHdr->Data) = (STL_PORT_INFO){0};

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);
		++i;

		if (port == bitset_find_last_one(&local_ports) || i == portsPerAggr) {
			uint32_t madStatus = 0;
			STL_AGGREGATE *lastSeg = NULL;

			status = SM_Get_Aggregate_DR(fd_topology, (STL_AGGREGATE*)buffer, aggrHdr,
				nodep->path, &lastSeg, &madStatus);
			status = sm_popo_port_error(&sm_popo, topop,
				sm_get_port(nodep, nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? 0 : port), status);
			if (status == VSTATUS_TIMEOUT_LIMIT)
				goto exit;

			if (status == VSTATUS_OK && lastSeg) {
				for (i = 0, aggrHdr = (STL_AGGREGATE*)buffer; aggrHdr < lastSeg; aggrHdr = STL_AGGREGATE_NEXT(aggrHdr), ++i) {
					STL_PORT_INFO *pPortInfo = (STL_PORT_INFO*)aggrHdr->Data;
					uint8_t cport;

					// Processing this PortInfo, clear it from the bitset.
					bitset_clear(&local_ports, cport = bitset_find_first_one(&local_ports));

					if (aggrHdr->Result.s.Error) {
						// Let caller know of failed ports.
						// Stop processing, rest of the Aggregate is bunk.
						bitset_set(ports, cport);
						break;
					}

					BSWAP_STL_PORT_INFO(pPortInfo);
					res[cport] = *pPortInfo;
				}
			} else {
				IB_LOG_ERROR_FMT(__func__,
					"Error on Get(Aggregate) DR from NodeGUID "FMT_U64" [%s]; status=%u",
					nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
				// Reset state on failure
				bitset_copy(ports, &local_ports);
				bitset_free(&local_ports);
				return status == VSTATUS_OK ? VSTATUS_BAD : status;
			}

			i = 0;
		}

		// Reset the port we're processing if we had an error.
		if (aggrHdr->Result.s.Error)
			port = bitset_find_first_one(&local_ports);
		else
			port = bitset_find_next_one(&local_ports, port + 1);

		if (i == 0)
			aggrHdr = (STL_AGGREGATE*)buffer;
		
	}

exit:
	bitset_free(&local_ports);

	return status;
	
}

// get_discovery_node_attributes()
// - Retrieves attributes from a new node that are useful during discovery. Currently retreives
//   nodeDesc, PortInfo, and SwitchInfo (in that order). When available, utilizes aggregate MADs.
//
//   Parameters:
//    path - DR path to the node
//    portNumber - port index of the node we will be interogating
//    nodeDesc - Pointer to where nodeDesc we retrieve should be placed
//    PortInfo - Pointer to where PortInfo we retreive should be placed
//    switchInfo - OPTINAL pointer to where swInfo we retreive should be placed (set to NULL if this is not a switch)
//    err - Pointer to buffer where we will place a string indicating error should we have one
//
//   Returns:
//   Pointer to *first* attribute we failed to get. NULL if we got everything sucessfully.
//
//
static void* 
get_discovery_node_attributes(uint8_t *path, uint8_t portNumber, STL_NODE_DESCRIPTION *nodeDesc, STL_PORT_INFO *PortInfo, STL_SWITCH_INFO *switchInfo, char *err, Status_t * retStatus)
{
	Status_t status = VSTATUS_OK;
	uint8_t buffer[3 * sizeof(STL_AGGREGATE) + 8*((sizeof(STL_NODE_DESCRIPTION) + 7)/8) + 8*((sizeof(STL_PORT_INFO) + 7)/8)
					+ 8*((sizeof(STL_SWITCH_INFO) + 7)/8)] = {0};
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	STL_AGGREGATE *lastSeg = NULL;

	if (sm_config.use_aggregates) {
		// 
		// Get the NodeDescription.
		// 
		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.Reserved = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_NODE_DESCRIPTION) + 7)/8;
		aggrHdr->AttributeModifier = 0;

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

		// Get the PortInfo.
		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PORT_INFO;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.Reserved = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_PORT_INFO) + 7)/8;
		aggrHdr->AttributeModifier = (1<<24) | STL_SM_CONF_START_ATTR_MOD | (uint32_t)portNumber;

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

		// If switch, get SwitchInfo too
		if (switchInfo) {
			aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_SWITCH_INFO;
			aggrHdr->Result.s.Error = 0;
			aggrHdr->Result.s.Reserved = 0;
			aggrHdr->Result.s.RequestLength = (sizeof(STL_SWITCH_INFO) + 7)/8;
			aggrHdr->AttributeModifier = 0;

			aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);
		}

		status = SM_Get_Aggregate_DR(fd_topology, (STL_AGGREGATE*)buffer, aggrHdr, path, &lastSeg, NULL);

		aggrHdr = (STL_AGGREGATE*)buffer;
	}

	if (!sm_config.use_aggregates || status != VSTATUS_OK) {
		if (sm_config.use_aggregates)
			err += sprintf(err, "Get(Aggregate) failed with status = %d, attempting fallback Gets()...", status);

		if ((status = SM_Get_NodeDesc(fd_topology, 0, path, nodeDesc)) != VSTATUS_OK) {
			sprintf(err, "Get(NodeDesc) failed with status = %d", status);
			if (retStatus) *retStatus = status;
			return (void *)nodeDesc;
		}

		if ((status = SM_Get_PortInfo(fd_topology, (1<<24) | STL_SM_CONF_START_ATTR_MOD | (uint32_t)portNumber, path, PortInfo)) != VSTATUS_OK) {
			sprintf(err, "Get(PortInfo) failed with status = %d", status);
			if (retStatus) *retStatus = status;
			return (void *)PortInfo;
		}

		if (switchInfo && (status = SM_Get_SwitchInfo(fd_topology, 0, path, switchInfo)) != VSTATUS_OK) {
			sprintf(err, "Get(SwitchInfo) failed with status = %d", status);
			if (retStatus) *retStatus = status;
			return (void *)switchInfo;

		}

	} else {
		// Aggregate itself was okay, but encapsulated NodeDesc or PortInfo was bad.
		if (status != VSTATUS_OK) {
			switch (lastSeg->AttributeID) {
				case STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION:
					sprintf(err, "Get(NodeDesc) failed with status = %d", status);
					if (retStatus) *retStatus = status;
					return (void *)nodeDesc;

				case STL_MCLASS_ATTRIB_ID_PORT_INFO:
					sprintf(err, "Get(PortInfo) failed with status = %d", status);
					if (retStatus) *retStatus = status;
					return (void *)PortInfo;

				case STL_MCLASS_ATTRIB_ID_SWITCH_INFO:
					sprintf(err, "Get(SwitchInfo) failed with status = %d", status);
					if (retStatus) *retStatus = status;
					return (void *)switchInfo;
				default:
					break;
			}
		} else {
			*PortInfo = *((STL_PORT_INFO*)STL_AGGREGATE_NEXT(aggrHdr)->Data);
			BSWAP_STL_PORT_INFO(PortInfo);

			if (switchInfo) {
				*switchInfo = *((STL_SWITCH_INFO*)STL_AGGREGATE_NEXT(STL_AGGREGATE_NEXT(aggrHdr))->Data);
				BSWAP_STL_SWITCH_INFO(switchInfo);
			}
		}

		*nodeDesc = *((STL_NODE_DESCRIPTION*)(aggrHdr)->Data);
		BSWAP_STL_NODE_DESCRIPTION(nodeDesc);
	}

	// Success
	if (retStatus) *retStatus = VSTATUS_OK;
	return NULL;
}

//Authenticates node described by neighborInfo and neighborPI, which is being discovered 
//through cnp and cpp. cnp and cpp should always be a trusted node. As called by sm_setup_node,
//cnp and cpp will always be either a switch or our SM node.
//neighborInfo must not be NULL for authentication to succeed
static __inline__ int sm_stl_authentic_node(Topology_t *tp, Node_t *cnp, Port_t *cpp,
                                            STL_NODE_INFO *neighborInfo, STL_PORT_INFO *neighborPI,
											STL_SWITCH_INFO *swInfo, uint32* quarantineReasons) {
    int authentic = 1;
	uint64 expectedPortGuid;

	if (quarantineReasons == NULL) {
		return 0;
	}

	if (neighborInfo == NULL) {
		return 0;
	}

	if (neighborPI && sm_valid_port(cpp) &&
		!(neighborPI->PortPacketFormats.Supported & cpp->portData->portInfo.PortPacketFormats.Supported)) {

		*quarantineReasons |= STL_QUARANTINE_REASON_BAD_PACKET_FORMATS;
		authentic = 0;
		return (authentic);
	}

	if (swInfo && tp->routingModule->funcs.routing_mode() == STL_ROUTE_LINEAR &&
		(swInfo->RoutingMode.Supported & STL_ROUTE_LINEAR) &&
		(swInfo->LinearFDBCap < STL_GET_UNICAST_LID_MAX())) {

		*quarantineReasons |= STL_QUARANTINE_REASON_MAXLID;
		return (authentic = 0);
	}

	if (neighborPI) {
		//Verify node MTU not less than 2048
		if (neighborPI->MTU.Cap < IB_MTU_2048) {
			*quarantineReasons |= STL_QUARANTINE_REASON_SMALL_MTU_SIZE;
			authentic = 0;
			return (authentic);
		}

		// Verify port supports min number required VLs needed for specified configuration
		// This test against LocalPortNum should likely be against NeighborPortNum instead, but as this code is
		// only called when communicating directly across the LocalPortNum link to this node, the point is moot
		if ((neighborInfo->NodeType != NI_TYPE_SWITCH) ||
			((neighborInfo->NodeType == NI_TYPE_SWITCH) && (neighborPI->LocalPortNum!=0))) {
			if (neighborPI->VL.s2.Cap < sm_needed_vls) {
				*quarantineReasons |= STL_QUARANTINE_REASON_VL_COUNT;
				authentic = 0;
				return (authentic);
			}
		}
	}

	if (!sm_config.sma_spoofing_check)
		return authentic;

	//if cnp and cpp are null, then we are at the first node (our own sm)
	//in discovery and have nothing to verify
	if (!(cnp && cpp))
		return authentic;

	//If device is appliance as configued in opafm.xml, we bypass security checks
	if (sm_stl_appliance(cpp->portData->portInfo.NeighborNodeGUID))
		return authentic;

	if (!sm_stl_port(cpp))
		return authentic;


	//This should never happen, but if cpp is unenhanced SWP0, then it doesn't support these checks
	//so we can bail out.
	if (cnp->nodeInfo.NodeType == NI_TYPE_SWITCH && cpp->index == 0 && !cnp->switchInfo.u2.s.EnhancedPort0)
		return authentic;

	if (cpp->portData->portInfo.NeighborNodeGUID != neighborInfo->NodeGUID ||
		StlNeighNodeTypeToNodeType(cpp->portData->portInfo.PortNeighborMode.NeighborNodeType) != neighborInfo->NodeType ||
		(neighborPI && cpp->portData->portInfo.NeighborPortNum != neighborPI->LocalPortNum) ||
		cpp->portData->portInfo.NeighborPortNum != neighborInfo->u1.s.LocalPortNum) {

		authentic = 0;
		*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
	}
	
	switch (neighborInfo->NodeType) {
	case NI_TYPE_SWITCH:
		if (sm_config.neighborFWAuthenEnable && cpp->portData->portInfo.PortNeighborMode.NeighborFWAuthenBypass != 0) {
			authentic = 0;
			*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
		}
		break;

	case NI_TYPE_CA:
		expectedPortGuid = NodeGUIDtoPortGUID(cpp->portData->portInfo.NeighborNodeGUID, cpp->portData->portInfo.NeighborPortNum);
		// for Gen-1 the HFI is never trusted, no need to check the
		// portInfo.PortNeighborMode.NeighborFWAuthenBypass field 
		if (expectedPortGuid != neighborInfo->PortGUID)	 {
			authentic = 0;
			*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
		}
		// During discovery, if spoof check security is enabled, and we discover a HFI node with a LID that doesn't match
		// the LID the switch thinks the HFI node has, it indicates that HFI has manually tinkered with its LID.
		//
		// The node should be quarantined because it appears the user is trying to spoof the fabric.
		//
		// This is only possible when the port is active, and security is enabled so the switch knows the neighbor LID.
		// If the port is down, the switch forgets the neighbor LID.
		//
		// Exception when switchport LID is permissive, and this switchport is not doing SLID checks.
		if (cnp->nodeInfo.NodeType == NI_TYPE_SWITCH && cpp->portData->portInfo.PortStates.s.PortState == IB_PORT_ACTIVE) {
			if (cpp->portData->portInfo.LID != STL_LID_PERMISSIVE
				&& neighborPI != NULL && cpp->portData->portInfo.LID != neighborPI->LID) {
				authentic = 0;
				*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
			}
		}
		break;

	default:
		authentic = 0;
		*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
		break;
	}	


	return (authentic);
}

// Log the reasons the current node is being quarantined or bounced.
static void
sm_stl_quarantine_reasons(Topology_t *tp, Node_t *cnp, Port_t *cpp, Node_t *qnp,
	uint32 quarantineReasons, STL_EXPECTED_NODE_INFO* expNodeInfo, const STL_PORT_INFO *pQPI) 
{
	if (pQPI) {
		if (quarantineReasons & STL_QUARANTINE_REASON_VL_COUNT) {
			IB_LOG_ERROR_FMT(__func__, "Node:%s guid:"FMT_U64" type:%s port:%d. Supported VLs(%d) too small(needs => %d).",
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID, StlNodeTypeToText(qnp->nodeInfo.NodeType), pQPI->LocalPortNum,
			pQPI->VL.s2.Cap, sm_needed_vls);
			return;
		}
		if (quarantineReasons & STL_QUARANTINE_REASON_SMALL_MTU_SIZE) {
			IB_LOG_ERROR_FMT(__func__, "Node:%s guid:"FMT_U64" type:%s port:%d. Supported MTU(%s) too small(needs => 2048).",
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID, StlNodeTypeToText(qnp->nodeInfo.NodeType), pQPI->LocalPortNum,
			IbMTUToText(pQPI->MTU.Cap));
			return;
		}
		if (quarantineReasons & STL_QUARANTINE_REASON_MAXLID) {
			IB_LOG_ERROR_FMT(__func__, "Node %s guid:"FMT_U64" type%s port:%d. Unable to support MaximumLID of %x.",
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID, StlNodeTypeToText(qnp->nodeInfo.NodeType), pQPI->LocalPortNum,
			STL_GET_UNICAST_LID_MAX() );
			return;
		}
	}

	if (cpp) {
		IB_LOG_ERROR_FMT(__func__, "Neighbor of %s %s NodeGUID "FMT_U64" port %d could not be authenticated. Reports: "
						"%s %s Guid "FMT_U64" Port %d: Actual: %s Guid "FMT_U64", Port %d",
						StlNodeTypeToText(cnp->nodeInfo.NodeType),
						sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, cpp->index,
						//reports
						StlNodeTypeToText(qnp->nodeInfo.NodeType),
						sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID,
						qnp->nodeInfo.u1.s.LocalPortNum,
						//actual
						OpaNeighborNodeTypeToText(cpp->portData->portInfo.PortNeighborMode.NeighborNodeType),
						cpp->portData->portInfo.NeighborNodeGUID,
						cpp->portData->portInfo.NeighborPortNum);
		IB_LOG_ERROR_FMT(__func__, "Authentication expected from the neighbor guid "FMT_U64" neighbor node type %s",
						cpp->portData->portInfo.NeighborNodeGUID,
						OpaNeighborNodeTypeToText(cpp->portData->portInfo.PortNeighborMode.NeighborNodeType));
	} else {
		// Yes, the SM can fail authentication. (Failure in validation of pre-defined topology).
		IB_LOG_ERROR_FMT(__func__, "SM's port failed authentication.");
	}
}

static void
sm_stl_quarantine_node(Topology_t *tp, Node_t *cnp, Port_t *cpp, Node_t *qnp,
	uint32 quarantineReasons, STL_EXPECTED_NODE_INFO* expNodeInfo, const STL_PORT_INFO *pQPI)
{
    QuarantinedNode_t * qnodep;

    // allocate memory for quarantined node list entry
    if (vs_pool_alloc(&sm_pool, sizeof(QuarantinedNode_t), (void *)&qnodep)) {
        IB_LOG_WARN0("sm_stl_quarantine_node: No memory for quarantined node entry"); 
    } else {
        // add to SM/SA repository quarantined link list used by the SA
        memset(qnodep, 0, sizeof(QuarantinedNode_t));
        qnodep->authenticNode = cnp;
        qnodep->authenticNodePort = cpp;
        qnodep->quarantinedNode = qnp;
		qnodep->quarantineReasons = quarantineReasons;
		memcpy(&qnodep->expNodeInfo, expNodeInfo, sizeof(STL_EXPECTED_NODE_INFO));
        Node_Enqueue(tp, qnodep, quarantined_node_head, quarantined_node_tail);
    }

	if (cpp)	
		cpp->portData->neighborQuarantined = 1;

    // add to SM/SA repository quarantined map used by the SM
    qnp->index = tp->num_quarantined_nodes++;
    if (cpp && cl_qmap_insert(tp->quarantinedNodeMap,
                       cpp->portData->portInfo.NeighborNodeGUID, // nodeGUID may be falsified, check with neighbor.
                       &qnp->mapQuarantinedObj.item) == &qnp->mapQuarantinedObj.item) {
        cl_qmap_set_obj(&qnp->mapQuarantinedObj, qnp);
    }

	sm_stl_quarantine_reasons(tp, cnp, cpp, qnp, quarantineReasons, expNodeInfo, pQPI);
}


// the special case of cnp==NULL and cpp==NULL only occurs when SM is trying its
// its own port, for all other nodes in fabric these will both be supplied
Status_t
sm_setup_node(Topology_t * topop, FabricData_t * pdtop, Node_t * cnp, Port_t * cpp, uint8_t * path)
{
	int i, portBytes;
	int new_node;
	int end_port;
	int start_port;
	int nextIdx, lastIdx;
	int switchPortChange = 0, authenticNode;
	uint8_t portNumber;
	uint64_t portGuid;
	Node_t *qnodep = NULL, *nodep = NULL, *oldnodep = NULL;
	Node_t *linkednodep = NULL;
	Port_t *portp, *oldportp, *swPortp = NULL;
	int use_cache = 0;
	Node_t *cache_nodep = NULL;
	Port_t *cache_portp = NULL;
	Status_t status = VSTATUS_OK;
	STL_NODE_DESCRIPTION nodeDesc;
	STL_NODE_INFO nodeInfo = { 0 };
	STL_PORT_INFO *portInfo;
	STL_PORT_INFO conPortInfo, *conPIp = NULL;
	STL_SWITCH_INFO switchInfo;
	STL_PORT_STATE_INFO *portStateInfo = NULL;
	STL_EXPECTED_NODE_INFO expNodeInfo = {{{ 0 }}};
	boolean local = !cnp || !cpp;

	// 
	// Note that this routine needs to be called with the topology locks set.
	// 

	IB_ENTER(__func__, topop, cnp, cpp, path);

	// validate upstream pointers:
	//   both NULL: discovering local node
	//   both !NULL: discovering neighbor of the specified port
	//   else: invalid
	DEBUG_ASSERT(!cnp == !cpp);

	//
	// Skip quarantined ports.
	//
	if (!local) {
		if (sm_popo_is_port_quarantined(&sm_popo, cpp)) {
			IB_LOG_WARN_FMT(__func__, "skipping port due to quarantine: node %s nodeGuid "FMT_U64" port %u",
				sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, cpp->index);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		if (sm_popo_is_node_quarantined(&sm_popo, cpp->portData->portInfo.NeighborNodeGUID)) {
			IB_LOG_WARN_FMT(__func__, "skipping node due to quarantine: source node %s nodeGuid "FMT_U64" port %u; dest nodeGuid "FMT_U64,
				sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, cpp->index,
				cpp->portData->portInfo.NeighborNodeGUID);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
	}

	//
	// Check to see if we have cached info on this node.
	//
	if (topology_passcount && !local && cpp->state == IB_PORT_ACTIVE) {
		use_cache = sm_find_cached_neighbor(cnp, cpp, &cache_nodep, &cache_portp);
	} 
	
	if (path[0] > 0 && !local && cpp->portData && 
		!sm_stl_appliance(cpp->portData->portInfo.NeighborNodeGUID) &&
		(nodep = sm_find_guid(topop, cpp->portData->portInfo.NeighborNodeGUID)) != NULL) {
		//
		// If we've already seen this node, use our stored copy of the NodeInfo.
		//
		nodeInfo = nodep->nodeInfo;
		nodeInfo.u1.s.LocalPortNum = cpp->portData->portInfo.NeighborPortNum;

	} else {
		// On SM resweep, first check if HFI nodeInfo can use cached data
		if (sm_config.use_cached_hfi_node_data && use_cache && path[0] > 0 && cache_nodep && cache_nodep->nodeInfo.NodeType == NI_TYPE_CA &&
					cache_nodep->nodeInfo.NodeGUID == cpp->portData->portInfo.NeighborNodeGUID) {
			nodeInfo = cache_nodep->nodeInfo;
			nodeInfo.u1.s.LocalPortNum = cpp->portData->portInfo.NeighborPortNum;
		} else if (sm_config.use_cached_node_data && use_cache && path[0] > 0 && cache_nodep && cache_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
					cache_nodep->nodeInfo.NodeGUID == cpp->portData->portInfo.NeighborNodeGUID) {
			nodeInfo = cache_nodep->nodeInfo;
			nodeInfo.u1.s.LocalPortNum = cpp->portData->portInfo.NeighborPortNum;
		} else if ((status = SM_Get_NodeInfo(fd_topology, 0, path, &nodeInfo)) != VSTATUS_OK) {
			// 
			// New node. Get the current NodeInfo struct.
			// 
			if (local) {
				IB_LOG_ERRORRC("Get NodeInfo failed for local node. rc:",
							   status);
				status = VSTATUS_UNRECOVERABLE;
			} else {
				IB_LOG_WARN_FMT(__func__,
					"Get NodeInfo failed for nodeGuid "FMT_U64" port %u, via node %s nodeGuid "FMT_U64" port %u; status=%d",
					cpp->portData ? cpp->portData->portInfo.NeighborNodeGUID : 0,
					cpp->portData ? cpp->portData->portInfo.NeighborPortNum : 0, 
					sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, cpp->index, status);
				// Set the topology changed flag if the node at other end was there 
				// on previous sweep 
				if (topology_passcount
					&& (nodep = sm_find_guid(&old_topology, cnp->nodeInfo.NodeGUID))) {
					if ((portp = sm_get_port(nodep, cpp->index)) == NULL) {
						IB_LOG_WARN_FMT(__func__,
										"Failed to get Port %d of Node " FMT_U64
										":%s, status=%d", cpp->index, cnp->nodeInfo.NodeGUID,
										sm_nodeDescString(cnp), status);
					} else {
						if (portp->portno)
							topology_changed = 1;
					}

					nodep = NULL;
				}
				status = sm_popo_port_error(&sm_popo, topop, cpp, status);
			}
			IB_EXIT(__func__, status);
			return (status);
		}
	}

	if (nodeInfo.NodeGUID == 0ull) {
		if (local) {
			IB_LOG_ERROR0("Received zero node guid for local node.");
		} else {
			IB_LOG_ERROR_FMT(__func__,
							 "Received zero node guid from node off Port %d of Node " FMT_U64
							 ":%s", cpp->index, cnp->nodeInfo.NodeGUID, sm_nodeDescString(cnp));
		}

		IB_EXIT(__func__, VSTATUS_BAD);
		return (VSTATUS_BAD);

	} else if (nodeInfo.PortGUID == 0ull && nodeInfo.NodeType == NI_TYPE_CA) {
		if (local) {
			IB_LOG_ERROR("Received zero PORTGUID for local node.", 0);
		} else {
			IB_LOG_ERROR_FMT(__func__,
							 "Received NULL PORTGUID from node off Port %d of Node " FMT_U64
							 ":%s", cpp->index, cnp->nodeInfo.NodeGUID, sm_nodeDescString(cnp));
		}
		IB_EXIT(__func__, VSTATUS_BAD);
		return (VSTATUS_BAD);
	// Quarantine check - don't trust nodeInfo.NodeGUID.
	} else if (!local && (qnodep = sm_find_quarantined_guid(topop, cpp->portData->portInfo.NeighborNodeGUID))) {
		IB_LOG_ERROR_FMT(__func__,
						 "Ignoring port connections to quarantined node %s guid " FMT_U64 "",
						 sm_nodeDescString(qnodep), cpp->portData->portInfo.NeighborNodeGUID);
		return (VSTATUS_BAD);
	}

	// sanity check port count, mainly to prevent simulator abuse
	if (nodeInfo.NumPorts > MAX_STL_PORTS) {
		IB_LOG_ERROR_FMT(__func__,
			"Node " FMT_U64 " exceeded %d port limit", nodeInfo.NodeGUID, MAX_STL_PORTS);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	portGuid = nodeInfo.PortGUID;
	portNumber = nodeInfo.u1.s.LocalPortNum;

	if (local) {
		portNumber = sm_config.port;
	}

	// 
	// Assume we've seen this node before.
	// 
	new_node = 0;

	// 
	// Check to see if this node is already in the current topology.
	// 
	if (!nodep) {
		uint32 quarantineReasons = 0x00000000;
		char errStr[256] = { '\0' };
		void *failedAttr;

		//
		// Flag this node for extra work.
		//
		new_node = 1;

		conPIp = &conPortInfo;

		// On re-sweep, attempt to re-use cached data for HFIs.
		if (!local && sm_config.use_cached_hfi_node_data && use_cache && cache_nodep && cache_nodep->nodeInfo.NodeType == NI_TYPE_CA &&
				cnp->nodeInfo.NodeType == NI_TYPE_SWITCH &&
				cache_nodep->nodeInfo.NodeGUID == cpp->portData->portInfo.NeighborNodeGUID &&
				sm_valid_port(cache_portp)) {
			// If the port hasn't changed we can just copy the data from the old topology!
			nodeDesc = cache_nodep->nodeDesc;
			conPortInfo = cache_portp->portData->portInfo;
			conPortInfo.LocalPortNum = cpp->portData->portInfo.NeighborPortNum;
			// LWD may have changed. Instead of checking manually, fill it in with the connected
			// switch ports LWD value.
			conPortInfo.LinkWidthDowngrade.TxActive = cpp->portData->portInfo.LinkWidthDowngrade.TxActive;
			conPortInfo.LinkWidthDowngrade.RxActive = cpp->portData->portInfo.LinkWidthDowngrade.RxActive;
			//LedInfo may have changed, update in case
			conPIp->PortStates.s.LEDEnabled = cnp->portStateInfo[cpp->index].PortStates.s.LEDEnabled;
			failedAttr = NULL;
		}
		else if (!local && sm_config.use_cached_node_data && use_cache && cache_nodep && cache_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
				cache_nodep->nodeInfo.NodeGUID == cpp->portData->portInfo.NeighborNodeGUID &&
				sm_valid_port(cache_portp)) {
			nodeDesc = cache_nodep->nodeDesc;
			conPortInfo = cache_portp->portData->portInfo;
			conPortInfo.LocalPortNum = cpp->portData->portInfo.NeighborPortNum;
			switchInfo = cache_nodep->switchInfo;
			failedAttr = NULL;
		}
		else {
			failedAttr = get_discovery_node_attributes(path, portNumber, &nodeDesc, conPIp,
												(nodeInfo.NodeType == NI_TYPE_SWITCH) ? &switchInfo : NULL,
												errStr, &status);
		}

		if (*errStr) {
			if (local)
				IB_LOG_WARN_FMT(__func__, "%s for local node", errStr);
			else
				IB_LOG_WARN_FMT(__func__, "%s for node off Port %d of Node"FMT_U64":%s", errStr,
								cpp->index, cnp->nodeInfo.NodeGUID, sm_nodeDescString(cnp));
		}

		if (failedAttr == (void*)&nodeDesc || failedAttr == (void*)&conPortInfo ) { 
			if (local)
				status = VSTATUS_UNRECOVERABLE;
			else
				status = sm_popo_port_error(&sm_popo, topop, cpp, status);

			IB_EXIT(__func__, status);
			return status;
		}

		// authenticate the node in order to determine whether to allow the
		// node to be part of the fabric.
		authenticNode = sm_stl_authentic_node(topop, cnp, cpp, &nodeInfo, conPIp,
											(nodeInfo.NodeType == NI_TYPE_SWITCH) ? &switchInfo : NULL,
											&quarantineReasons);

		// If the node is authentic and pre-defined topology is enabled, verify the node against the
		// input topology.
		if(authenticNode && sm_config.preDefTopo.enabled) {
			// If the Connected Port is Armed or Active, the port is going to be bounced so, force log only as Warn.
			boolean forceLogAsWarn = (conPIp->PortStates.s.PortState > IB_PORT_INIT) ? TRUE : FALSE;
			authenticNode = sm_validate_predef_fields(topop, pdtop, cnp, cpp,
				&nodeInfo, &nodeDesc, &quarantineReasons, &expNodeInfo, forceLogAsWarn);
		}
		nodep = Node_Create(topop, &nodeInfo, nodeInfo.NodeType, nodeInfo.NumPorts + 1, &nodeDesc,
					authenticNode);
		if (nodep == NULL) {
			IB_LOG_ERROR_FMT(__func__, "cannot create a new node for GUID: " FMT_U64,
							 nodeInfo.NodeGUID);
			IB_EXIT(__func__, VSTATUS_EIO);
			return (VSTATUS_EIO);
		}

		memcpy((void *) nodep->path, (void *) path, 64);
		//Once the node is created the switch info is copied, so it is not missed.
		if (nodeInfo.NodeType == NI_TYPE_SWITCH) {
			// all switch specific information has been retreived
			nodep->switchInfo = switchInfo;
		}

		// if node fails authentication, then quarantine the attacker node.
		if (!authenticNode) {
			status = VSTATUS_BAD;
			if (!local) {
				// Bounce any ports on node which are Armed or Active
				if(cpp->portData->portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW) {
					//node type may be spoofed so set it to correct value:
					nodep->nodeInfo.NodeType = NI_TYPE_SWITCH;
					status = sm_bounce_all_switch_ports(topop, nodep, NULL, path);
					if(status!=VSTATUS_OK) {
						IB_LOG_WARN_FMT(__func__, "Unable to bounce potentially active ports on quarantined switch %s. Status: %d",
											sm_nodeDescString(nodep), status);
					}

				} else { // HFI
					//node type may be spoofed so set it to correct value:
					nodep->nodeInfo.NodeType = NI_TYPE_CA;
					// if it is a HFI, the max num of ports is 1
					nodep->nodeInfo.NumPorts = 1;
					if(conPIp->PortStates.s.PortState > IB_PORT_INIT) {
						status = sm_bounce_link(topop, cnp, cpp);
					}
				}
			}
			if (status != VSTATUS_OK) {
				// Quarantine Node only if the bounce did not happen, as the bounce
				//  should set the port to DOWN and should be picked up and rechecked
				//  for quarantine Next sweep when it is back in INIT
				(void)sm_stl_quarantine_node(topop, cnp, cpp, nodep, quarantineReasons, &expNodeInfo, conPIp);
			} else {
				if (!local) {
					sm_stl_quarantine_reasons(topop, cnp, cpp, nodep, quarantineReasons, &expNodeInfo, conPIp);
					IB_LOG_WARN_FMT(__func__, "Link was bounced instead of quarantined to disable potentially unsecure traffic.");
				}
			}
			IB_EXIT(__func__, VSTATUS_BAD);
			return (VSTATUS_BAD);
		}

		nodep->index = topop->num_nodes++;

		// Check to see if the node existed in the previous topology.
		if (cache_nodep) {
			oldnodep = cache_nodep;
		} else {
			oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
		}

		if (oldnodep != NULL) {
			nodep->oldIndex = oldnodep->index;
			nodep->oldExists = 1;
			nodep->old = oldnodep;
			oldnodep->old = NULL;
		}

		/* keep track of how many switch chips in fabric and which are newly added */
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (oldnodep != NULL) {
				nodep->swIdx = oldnodep->swIdx;
			} else {
				lastIdx = -1;
				nextIdx = bitset_find_first_zero(&old_switchesInUse);
				while ((nextIdx < 0) || bitset_test(&new_switchesInUse, nextIdx)) {
					if (nextIdx == -1) {
						if (!bitset_resize
							(&old_switchesInUse, old_switchesInUse.nbits_m + SM_NODE_NUM)
							|| !bitset_resize(&new_switchesInUse,
											  new_switchesInUse.nbits_m + SM_NODE_NUM)) {
							// Can't track deltas, do full route check.
							topology_changed = 1;
							break;
						}
						if (lastIdx == -1) {
							nextIdx = bitset_find_first_zero(&old_switchesInUse);
						} else {
							nextIdx = bitset_find_next_zero(&old_switchesInUse, lastIdx);
						}
					}
					lastIdx = nextIdx;
					nextIdx = bitset_find_next_zero(&old_switchesInUse, nextIdx + 1);
				}
				if (nextIdx < 0) {
					// flag error
					nodep->swIdx = topop->num_sws;
				} else {
					nodep->swIdx = nextIdx;
				}
			}
			topop->num_sws++;
			topop->max_sws = MAX(topop->max_sws, nodep->swIdx + 1);
			bitset_set(&new_switchesInUse, nodep->swIdx);

		}

		// track all new switches and endnodes for LID additions
		if (!oldnodep) {
			if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
				topology_changed = 1;
		} else {
			check_for_new_endnode(&nodeInfo, nodep, oldnodep);
		}

		nodep->asyncReqsSupported = sm_config.sma_batch_size;


		// Add NODE to sorted topo tree here
		if (cl_qmap_insert
			(sm_newTopology.nodeIdMap, (uint64_t) nodep->index,
			 &nodep->nodeIdMapObj.item) != &nodep->nodeIdMapObj.item) {
			IB_LOG_ERROR_FMT(__func__,
				"Error adding Node Id: 0x%x to tree. Already in tree!",
				nodep->index);
		} else {
			cl_qmap_set_obj(&nodep->nodeIdMapObj, nodep);
		}
		if (cl_qmap_insert
			(sm_newTopology.nodeMap, nodep->nodeInfo.NodeGUID,
			 &nodep->mapObj.item) != &nodep->mapObj.item) {
			IB_LOG_ERROR_FMT(__func__,
							 "Error adding Node GUID: " FMT_U64 " to tree. Already in tree!",
							 nodep->nodeInfo.NodeGUID);
		} else {
			cl_qmap_set_obj(&nodep->mapObj, nodep);
		}

	} else { // existing node
		sm_popo_update_node(nodep->ponodep);

		uint32 quarantineReasons = 0x00000000;

		// attacker may attempt to use valid node GUID of an existing node, so
		// authenticate the node in order to determine whether to continue to
		// allow the node to be part of the fabric.
		authenticNode = sm_stl_authentic_node(topop, cnp, cpp, &nodeInfo, NULL, NULL, &quarantineReasons);
		nodeDesc = nodep->nodeDesc;

		// If the node is authentic and pre-defined topology is enabled, verify the node against the
		// input topology.
		if(authenticNode && sm_config.preDefTopo.enabled) {
			authenticNode = sm_validate_predef_fields(topop, pdtop, cnp, cpp,
				&nodeInfo, &nodeDesc, &quarantineReasons, &expNodeInfo, FALSE);
		}

		if (!authenticNode) {
			Node_t *attackerNodep;

			// get the node description from this attacker node, for it too may
			// have been spoofed
			if (SM_Get_NodeDesc(fd_topology, 0, path, &nodeDesc) != VSTATUS_OK)
				cs_strlcpy((char *) nodeDesc.NodeString, "Not Available", STL_NODE_DESCRIPTION_ARRAY_SIZE);

			// create new node as a placeholder for the attacker node
			attackerNodep = Node_Create(topop, &nodeInfo, nodeInfo.NodeType,
						nodeInfo.NumPorts + 1, &nodeDesc, authenticNode);
			if (attackerNodep == NULL) {
				IB_LOG_ERROR_FMT(__func__,
								 "cannot create a new node for attacker GUID: " FMT_U64,
								 nodeInfo.NodeGUID);
				IB_EXIT(__func__, VSTATUS_EIO);
				return (VSTATUS_EIO);
			}

			//save path to attacker node
			 memcpy((void *) attackerNodep->path, (void *) path, 64);
			// quarantine the attacker node
			(void) sm_stl_quarantine_node(topop, cnp, cpp, attackerNodep, quarantineReasons, &expNodeInfo, NULL);
			IB_EXIT(__func__, VSTATUS_BAD);
			return (VSTATUS_BAD);
		}
		
		if (!sm_config.loopback_mode && (nodep->nodeInfo.NodeType != nodeInfo.NodeType ||
				((nodep->nodeInfo.NodeType == NI_TYPE_CA && nodeInfo.NodeType == NI_TYPE_CA) && 
				(nodep->nodeInfo.PortGUID == nodeInfo.PortGUID || nodeInfo.u1.s.LocalPortNum == nodep->nodeInfo.u1.s.LocalPortNum)))) {
			char nodeDescStr[STL_NODE_DESCRIPTION_ARRAY_SIZE];
			memcpy(nodeDescStr, nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);
			nodeDescStr[STL_NODE_DESCRIPTION_ARRAY_SIZE - 1] = '\0';
			IB_LOG_ERROR_FMT(__func__,
							 "Duplicate NodeGuid for Node %s nodeType[%d] guid " FMT_U64 " and "
							 "existing node[%d] nodeType=%d, %s, guid " FMT_U64,
							 nodeDescStr, nodeInfo.NodeType, nodeInfo.NodeGUID,
							 nodep->index, nodep->nodeInfo.NodeType, sm_nodeDescString(nodep),
							 nodep->nodeInfo.NodeGUID);

			return VSTATUS_BAD;
		}

		// potentially new port on existing node; check previous topology
		// 
		// only applies to FIs... we can hit a switch multiple times, but the
		// end node is always the same port zero.  hitting a FI always
		// results in a different end port
		if (nodep->nodeInfo.NodeType == NI_TYPE_CA)
			check_for_new_endnode(&nodeInfo, nodep, NULL);
	}

	// 
	// JSY - Interop fix - CAL doesn't return the Guid for loopback discovery
	// 
	// MWHEINZ FIXME: Is this still needed?
	if ((nodep->index == 0) && (portNumber == sm_config.port)) {
		nodep->nodeInfo.PortGUID = portGuid;
	}

	// 
	// Initialize the port and link this node/port into the topology links.
	// 
	if ((portp = sm_get_port(nodep, portNumber)) == NULL) {
		cs_log(VS_LOG_ERROR, "sm_setup_node", "get portNumber %d for node %s failed",
			   portNumber, sm_nodeDescString(nodep));
		IB_EXIT(__func__, VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	if (sm_dynamic_port_alloc()) {
		/* allocate port record associated with the primary port of the node */
		if ((portp->portData = sm_alloc_port(topop, nodep, portNumber, &portBytes)) == NULL) {
			IB_LOG_ERROR_FMT(__func__, "cannot create primary port %d for node %s",
							 portNumber, sm_nodeDescString(nodep));
			IB_EXIT(__func__, VSTATUS_BAD);
			return (VSTATUS_BAD);
		}
	}

	if (!new_node) {
		// This port may have failed to discover for various reasons (e.g.
		// a failed Get(PortInfo)), or may have been legitimately DOWN at
		// the time of discovery.
		if (portp->state <= IB_PORT_DOWN) {
			IB_LOG_INFINI_INFO_FMT(__func__,
				"Discovering node %s nodeGuid "FMT_U64" via inbound port %u which previously failed discovery.",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portNumber);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}

		conPIp = &portp->portData->portInfo;
	}

	// There can be a race condition where the state of local port of the
	// connected node doesn't match its neighbor's because of the time
	// that elapsed between getting the neighbor's info and getting this
	// info. This can lead to inconsistencies programming the fabric.
	// If this happens, mark the link as down, we'll fix it in the next
	// sweep.
	if (!local &&
		((cpp->state == IB_PORT_ACTIVE && conPIp->PortStates.s.PortState <= IB_PORT_INIT  ) ||
		 (cpp->state <= IB_PORT_INIT   && conPIp->PortStates.s.PortState == IB_PORT_ACTIVE)))
	{
		IB_LOG_INFINI_INFO_FMT(__func__, "Port states mismatched for"
			" port[%d] of Node "FMT_U64 ":%s (State=%s), connected to port[%d]"
			" of Node "FMT_U64 ":%s (State=%s)", cpp->index, cnp->nodeInfo.NodeGUID,
			sm_nodeDescString(cnp), IbPortStateToText(cpp->state), portNumber, nodeInfo.NodeGUID,
			nodeDesc.NodeString, IbPortStateToText(conPIp->PortStates.s.PortState));
		// topology_discovery() will mark the link down
		sm_request_resweep(0, 0, SM_SWEEP_REASON_ROUTING_FAIL);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		if ((swPortp = sm_get_port(nodep, 0))) {
			if (sm_dynamic_port_alloc()) {
				/* allocate port record associated with the switch port */
				if (!(swPortp->portData = sm_alloc_port(topop, nodep, 0, &portBytes))) {
					IB_LOG_ERROR_FMT(__func__,
									 "failed to allocate switch port %d for node %s", 0,
									 sm_nodeDescString(nodep));
					IB_EXIT(__func__, VSTATUS_BAD);
					return (VSTATUS_BAD);
				}
			}
			swPortp->portData->guid = portGuid;
			// Not really necessary, but for consistency.  Default wire depth.
			swPortp->portData->initWireDepth = -1;
		} else {
			IB_LOG_ERROR_FMT(__func__, "failed to get switch port %d for node %s",
							 0, sm_nodeDescString(nodep));
			IB_EXIT(__func__, VSTATUS_BAD);
			return (VSTATUS_BAD);
		}

	} else {
		portp->portData->guid = portGuid;
		// Default wire depth.
		portp->portData->initWireDepth = -1;
	}

	if (new_node != 0) {
		portp->state = IB_PORT_DOWN;
	}

	if (portp->path[0] == 0xff) {
		(void) memcpy((void *) portp->path, (void *) path, 64);
	}

	if ((nodeInfo.NodeType == NI_TYPE_SWITCH) && (swPortp->path[0] == 0xff)) {
		(void) memcpy((void *) swPortp->path, (void *) path, 64);
	}

	if (!local) {
		cpp->nodeno = nodep->index;
		cpp->portno = portp->index;

		portp->nodeno = cnp->index;
		portp->portno = cpp->index;

		if (cnp->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				if (!portp->portData->isIsl) {
					portp->portData->isIsl = 1;
					nodep->numISLs++;
				}
				if (!cpp->portData->isIsl) {
					cpp->portData->isIsl = 1;
					cnp->numISLs++;
				}
				if (nodep->nodeInfo.SystemImageGUID != cnp->nodeInfo.SystemImageGUID) {
					cnp->externalLinks = 1;
					nodep->externalLinks = 1;
				} else if (cnp != nodep) {
					cnp->internalLinks = 1;
					nodep->internalLinks = 1;
				}
			} else {
				cnp->edgeSwitch = 1;
			}
		} else if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			nodep->edgeSwitch = 1;
		}
	}

	// We may have seen this node before, but not had both sides of the link set up
	// properly to evaluate link policy, so check again now.
	if(new_node == 0) {
		if(!sm_valid_port(portp))
			return(VSTATUS_BAD);

		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
			|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
			&& (portp->index > 0))) {	
		
			if(sm_verifyPortSpeedAndWidth(topop, nodep, portp) != VSTATUS_OK) {
				sm_mark_link_down(topop, portp);
				return(VSTATUS_BAD);
			}
		}
	}


	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		if (!local && cpp->state == IB_PORT_INIT) {
			if (cnp->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				/*
				IB_LOG_INFINI_INFO_FMT(__func__, "Switch reporting port change, node %s 
				guid "FMT_U64" link to node %s "FMT_U64" port %d", sm_nodeDescString(nodep),
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID,
				cpp->index); */

				topology_changed = 1;
			} else if (!topology_changed) {
				// Check for case when new HFI port was linked to an ISL on the last sweep.
				oldnodep = sm_find_guid(&old_topology, cnp->nodeInfo.NodeGUID);
				if (oldnodep) {
					oldportp = sm_get_port(oldnodep, cpp->index);
					if (sm_valid_port(oldportp) &&
						oldportp->state >= IB_PORT_INIT &&
						oldportp->portData->isIsl) {
						topology_changed = 1;
					}
				}
			}
		}
		// 
		// If we have already seen this node, just return.
		// 
		if (new_node == 0) {
			IB_EXIT(__func__, VSTATUS_OK);
			return (VSTATUS_OK);
		}
	}
	// 
	// Get the information from all of its ports.  If this node/port is a
	// Master SM, then we do the comparison to see if we should still be
	// the SM.
	// 
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {

		start_port = 0;
		end_port = nodep->nodeInfo.NumPorts;

		// if the switch supports adaptive routing, then allocate adaptive
		// routing related structures
		if (switchInfo.CapabilityMask.s.IsAdaptiveRoutingSupported) {
			if (switchInfo.AdaptiveRouting.s.Enable) {
				if (sm_adaptiveRouting.debug) {
					IB_LOG_INFINI_INFO_FMT(__func__,
                                               "Switchinfo for node %s guid " FMT_U64
                                               " indicates Adaptive Routing support",
                                               sm_nodeDescString(nodep),
                                               nodep->nodeInfo.NodeGUID);
				}
			}
			if (sm_adaptiveRouting.enable) {
				nodep->arSupport = 1;
				// We round up the allocation to a whole block.
				if ((status = vs_pool_alloc(&sm_pool,
					sizeof(STL_PORTMASK) * ROUNDUP(switchInfo.PortGroupCap+1,
					NUM_PGT_ELEMENTS_BLOCK),
					(void *) &nodep->pgt)) == VSTATUS_OK) {
					memset((void *) nodep->pgt, 0,
						sizeof(STL_PORTMASK) * (switchInfo.PortGroupCap));
				}
				if (status != VSTATUS_OK) {
					nodep->arSupport = 0;
					IB_LOG_WARN0("No memory for "
						"adaptive routing support");
				}
			}
		}
		if (nodep->switchInfo.u1.s.PortStateChange) {
			if (bitset_test(&old_switchesInUse, nodep->swIdx)) {
				// Set indicator that need to check for link down to switch.
				switchPortChange = 1;
			}
			if (!oldnodep) {
				oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
			}
			if (oldnodep) {
				/* save the portChange value in the old node in case this sweep gets
				aborted */
				oldnodep->switchInfo.u1.s.PortStateChange = 1;
			}
			topology_switch_port_changes = 1;
		}
		if (!topology_switch_port_changes) {
			/* Check if portChange flag is set in the old topology. If it is still set then the 
			   previous sweep had been aborted at some point. Set the
			   topology_switch_port_changes flag to ensure MFTs are programmed. */
			if (!oldnodep) {
				oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
			}
			if (oldnodep && oldnodep->switchInfo.u1.s.PortStateChange) {
				topology_switch_port_changes = 1;
			}
		}
		// Fill in our PortStateInfo information for this switch
		if((status = sm_get_node_port_states(topop, nodep, portp, path, &portStateInfo)) != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__, "Unable to get PortStateInfo information for switch %s. Status: %d",
								sm_nodeDescString(nodep), status);
			nodep->portStateInfo = NULL;
			if ((status = sm_popo_port_error(&sm_popo, topop, swPortp, status)) == VSTATUS_TIMEOUT_LIMIT) {
				IB_EXIT(__func__, status);
				return status;
			}
		} else {
			nodep->portStateInfo = portStateInfo;
		}
	} else {
		start_port = portNumber;
		end_port = portNumber;
	}

	STL_PORT_INFO *newPortInfos;
	bitset_t needPortInfo;
	bitset_t skipPorts;

	vs_pool_alloc(&sm_pool, sizeof(STL_PORT_INFO) * (end_port + 1), (void *)&newPortInfos );
	bitset_init(&sm_pool, &needPortInfo, end_port + 1);
	bitset_init(&sm_pool, &skipPorts, end_port + 1);

	for (i = start_port; i <= end_port; i++) {
		if ((portp = sm_get_port(nodep, i)) == NULL) {
			IB_LOG_ERROR_FMT(__func__, "get port %d for node %s nodeGuid "FMT_U64" failed",
				i, sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
			continue;
		}

		if (sm_dynamic_port_alloc()) {
			if ((portp->portData = sm_alloc_port(topop, nodep, i, &portBytes)) == NULL) {
				IB_LOG_ERROR_FMT(__func__, "cannot create port %d for node %s nodeGuid "FMT_U64,
					 i, sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
				continue;
			}
			// Default wire depth.
			portp->portData->initWireDepth = -1;
		}

		/* 
		 * Note: Only attempt to put the port GUID in the tree if it is non-zero (CA ports and switch port 0)
		 * Check for FI based SM port being inserted in the map twice (SM node = tp->node_head)
		 * This is to get around a quickmap abort when a duplicate portGuid is inserted in map
		 */
		if ((portp->portData->guid && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ||
			(portp->portData->guid
			 && (nodep != sm_topop->node_head
				 || !(sm_find_port_guid(&sm_newTopology, portp->portData->guid))))) {
			if (cl_qmap_insert
				(sm_newTopology.portMap, portp->portData->guid,
				 &portp->portData->mapObj.item) != &portp->portData->mapObj.item) {
				IB_LOG_ERROR_FMT(__func__,
								 "Error adding Port GUID: " FMT_U64
								 " to tree. Already in tree!", portp->portData->guid);
			} else {
				cl_qmap_set_obj(&portp->portData->mapObj, portp);
			}
		} else {
			if (portp->portData->guid) {
				bitset_set(&skipPorts, i);
				continue;		/* already seen this port, don't get its portInfo again */
			}
		}

		if (use_cache) {
			if(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && cache_nodep) {
				cache_portp = sm_get_port(cache_nodep, i);

				if (sm_valid_port(cache_portp) && nodep->portStateInfo) {
					// If the port hasn't changed we can just copy the data from the old topology!
					if(cache_portp->state == nodep->portStateInfo[i].PortStates.s.PortState) {
						newPortInfos[i] = cache_portp->portData->portInfo;
						//LedInfo may have changed, update in case
						newPortInfos[i].PortStates.s.LEDEnabled = nodep->portStateInfo[i].PortStates.s.LEDEnabled;
						//The Tx/RxActive values may have changed. Update in case.
						newPortInfos[i].LinkWidthDowngrade.TxActive = nodep->portStateInfo[i].LinkWidthDowngradeTxActive;
						newPortInfos[i].LinkWidthDowngrade.RxActive = nodep->portStateInfo[i].LinkWidthDowngradeRxActive;
						continue;
					}
				}
			}
		}

		if (portp->index==portNumber) {
			// Save the extra read since we've already read it.
			// and we know the read was good.
			newPortInfos[i] = *conPIp;
			continue;
		}

		// If this port is down, then we don't care about most it's portData and portInfo contents,
		// so just copy linkDownReasons and jump to the part where we clean it up.
		if (portStateInfo && portStateInfo[i].PortStates.s.PortState == IB_PORT_DOWN) {
			oldnodep = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
			if (oldnodep) {
				oldportp = sm_get_port(oldnodep, i);
				if (sm_valid_port(oldportp)) {
					topology_saveLdr(&sm_newTopology, nodep->nodeInfo.NodeGUID, oldportp);
				}
			}
			sm_mark_link_down(topop, portp);
			continue;
		}

		bitset_set(&needPortInfo, i);
	}

	if (!sm_config.use_aggregates) {
		status = sm_GetPortInfoLoop(topop, nodep, &needPortInfo, newPortInfos);
	} else {
		status = sm_GetPortInfoAggr(topop, nodep, &needPortInfo, newPortInfos);
		if (status != VSTATUS_OK && status != VSTATUS_TIMEOUT_LIMIT) {
			status = sm_GetPortInfoLoop(topop, nodep, &needPortInfo, newPortInfos);
		}
	}

	if (status == VSTATUS_TIMEOUT_LIMIT) {
		IB_EXIT(__func__, status);
		return status;
	}

	for (i = start_port; i <= end_port; i++) {
		portInfo = &newPortInfos[i];
		if ((portp = sm_get_port(nodep, i)) == NULL) {
			cs_log(VS_LOG_ERROR, "sm_setup_node", "get port %d for node %s failed", i,
				   sm_nodeDescString(nodep));
			continue;
		}
		
		if (bitset_test(&skipPorts, i)) continue;

		if (portStateInfo && portStateInfo[i].PortStates.s.PortState == IB_PORT_DOWN)
			goto cleanup_down_ports;

		// There was a problem getting this port
		if (bitset_test(&needPortInfo, i)) {
			/* indicate a fabric change has been detected to insure paths-lfts-etc are
			   recalculated */
			if (topology_passcount && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
				topology_changed = 1;
			IB_LOG_WARN_FMT(__func__,
							"Failed to get PortInfo from NodeGUID " FMT_U64
							" [%s] Port %d; Ignoring port!", nodep->nodeInfo.NodeGUID,
							sm_nodeDescString(nodep), portp->index);
			sm_mark_link_down(topop, portp);
			goto cleanup_down_ports;
		}

		if (portInfo->DiagCode.s.UniversalDiagCode != 0) {
			IB_LOG_ERROR_FMT(__func__,
							 "DiagCode of node %s index %d port %d: chain %d vendor %d universal %d",
							 sm_nodeDescString(nodep), nodep->index, portp->index,
							 portInfo->DiagCode.s.Chain, portInfo->DiagCode.s.VendorDiagCode,
							 portInfo->DiagCode.s.UniversalDiagCode);

			if (portInfo->DiagCode.s.UniversalDiagCode == DIAG_HARD_ERROR) {
				sm_mark_link_down(topop, portp);
				goto cleanup_down_ports;
			}
		}

		if (nodep->nodeInfo.NodeType == NI_TYPE_CA ||
			(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && i == 0)) {
			if (portInfo->CapabilityMask3.s.VLSchedulingConfig ==
					VL_SCHED_MODE_VLARB) {
				nodep->vlArb = 1;
			}
		}


		/* set to correct gid prefix in portp structure - we'll do the portInfo later */
		if (portInfo->SubnetPrefix != sm_config.subnet_prefix) {
			portp->portData->gidPrefix = sm_config.subnet_prefix;
		} else {
			portp->portData->gidPrefix = portInfo->SubnetPrefix;
		}
		(void) memcpy((void *) &portp->portData->gid[0], (void *) &portp->portData->gidPrefix,
					  8);
		(void) memcpy((void *) &portp->portData->gid[8], (void *) &portp->portData->guid, 8);
		*(uint64_t *) & portp->portData->gid[0] =
			ntoh64(*(uint64_t *) & portp->portData->gid[0]);
		*(uint64_t *) & portp->portData->gid[8] =
			ntoh64(*(uint64_t *) & portp->portData->gid[8]);
		portp->portData->capmask = portInfo->CapabilityMask.AsReg32;
		portp->portData->capmask3 = portInfo->CapabilityMask3.AsReg16;
		portp->portData->lid = portInfo->LID;
		portp->portData->lmc = portInfo->s1.LMC;
		portp->portData->mtuSupported = portInfo->MTU.Cap;
		portp->state = portInfo->PortStates.s.PortState;

		oldnodep = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
		if (oldnodep) {
			/* Copy over trap related counters from old topology so that they don't get lost
			   with every sweep. */
			/* Also copy over the LinkDownReason(s) if there are any */
			oldportp = sm_get_port(oldnodep, i);
			if (sm_valid_port(oldportp)) {
				portp->portData->numFailedActivate = oldportp->portData->numFailedActivate;
				portp->portData->trapWindowStartTime = oldportp->portData->trapWindowStartTime;
				portp->portData->trapWindowCount = oldportp->portData->trapWindowCount;
				portp->portData->lastTrapTime = oldportp->portData->lastTrapTime;
				portp->portData->suppressTrapLog = oldportp->portData->suppressTrapLog;
				portp->portData->logSuppressedTrapCount =
					oldportp->portData->logSuppressedTrapCount;
				portp->portData->logTrapSummaryThreshold =
					oldportp->portData->logTrapSummaryThreshold;
				portp->portData->linkPolicyViolation  =
					oldportp->portData->linkPolicyViolation;

				memcpy(portp->portData->LinkDownReasons, oldportp->portData->LinkDownReasons, sizeof(STL_LINKDOWN_REASON) * STL_NUM_LINKDOWN_REASONS);

				// Copy wire depth from old port data.
				portp->portData->initWireDepth = oldportp->portData->initWireDepth;

				if(nodep->nodeInfo.NodeType == NI_TYPE_CA && 
					portp->portData->lid != oldportp->portData->lid) {
					/* Don't warn about ports that merely bounced since the last sweep. */
					if (portp->state > IB_PORT_INIT || portp->portData->lid != 0) {
						IB_LOG_WARN_FMT(__func__,
							"Node %s (GUID: " FMT_U64 ") attempted to change"
							" LID on port %d from 0x%x to 0x%x."
							" Resetting to previous LID.",
							sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
							portp->index, oldportp->portData->lid, portp->portData->lid);
					}
					portp->portData->lid = oldportp->portData->lid;
					portp->portData->dirty.portInfo=1;
				}

				// reregistration stays pending across sweeps until success or state change
				portp->portData->reregisterPending =
					portp->state == IB_PORT_ACTIVE && oldportp->portData->reregisterPending;
			}
		}

		if (old_topology.ldrCache && (!oldnodep || !sm_valid_port(oldportp))) {
			// Port may have disappeared; try to get LinkDownReasons from cache
			LdrCacheKey_t key;
			cl_map_item_t *it;
			key.guid = nodep->nodeInfo.NodeGUID;
			key.index = portp->index;

			it = cl_qmap_get(old_topology.ldrCache, (long int)&key);
			if (it != cl_qmap_end(old_topology.ldrCache)) {
				LdrCacheEntry_t *c = PARENT_STRUCT(it, LdrCacheEntry_t, mapItem);
				memcpy(portp->portData->LinkDownReasons, c->ldr,
				  sizeof(STL_LINKDOWN_REASON) * STL_NUM_LINKDOWN_REASONS);
			}
		}

		/* save the portInfo */
		portp->portData->portInfo = newPortInfos[i];
	
		/***** applicable to all non management ports *****/
		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
			|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
			&& (portp->index > 0))) {	
		
			if(sm_verifyPortSpeedAndWidth(topop, nodep, portp) != VSTATUS_OK) {
				sm_mark_link_down(topop, portp);
				goto cleanup_down_ports;
			}

		}

		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH &&
			portp->portData->portInfo.PortStates.s.PortState != IB_PORT_INIT &&
			portp->portData->portInfo.CapabilityMask3.s.IsMAXLIDSupported && 
			portp->portData->portInfo.MaxLID > 0 && portp->portData->portInfo.MaxLID < STL_GET_UNICAST_LID_MAX() ) {

			sm_bounce_link(topop, nodep, portp);
			sm_mark_link_down(topop, portp);
			goto cleanup_down_ports;
		}


		if (portInfo->PortStates.s.PortState == IB_PORT_INIT) {
			if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
				if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
					topology_changed = 1;
			}
				
			// Update this port's list of LinkDownReasons if there is a new one
			if(portInfo->LinkDownReason != STL_LINKDOWN_REASON_NONE ||
			  portInfo->NeighborLinkDownReason != STL_LINKDOWN_REASON_NONE) {
				int indexToUse = STL_LINKDOWN_REASON_NEXT_INDEX(portp->portData->LinkDownReasons);
				portp->portData->LinkDownReasons[indexToUse].LinkDownReason = portInfo->LinkDownReason;
				portp->portData->LinkDownReasons[indexToUse].Timestamp = (uint64_t) time(NULL);
				portp->portData->LinkDownReasons[indexToUse].NeighborLinkDownReason = portInfo->NeighborLinkDownReason;
			}
		}



		portp->portData->vl0 = portInfo->VL.s2.Cap;
		/* we will only support one block of 8 guids for now */
		portp->portData->guidCap = PORT_GUIDINFO_DEFAULT_RECORDS;	/* portInfo.GUIDCap; */
		portp->portData->rate = linkWidthToRate(portp->portData);

		portp->portData->portSpeed = sm_GetSpeed(portp->portData);

		if (i == start_port || (nodep->index == 0 && i == sm_config.port)) {
			int newLidCount;
			sm_update_or_assign_lid(portp, 0, &newLidCount);

			if (sm_config.sm_debug_lid_assign) {
				IB_LOG_INFINI_INFO_FMT(__func__,
				   "assign lid to port " FMT_U64 ": lid 0x%x, lmc 0x%x",
				   portGuid, portp->portData->lid, portp->portData->lmc);
			}

			if (newLidCount) {
				if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
					topology_changed = 1;
			}
		}

		if ((i > 0) && (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)) {
			portp->portData->lid = STL_LID_RESERVED;
		}

		if ((portp->portData->vl0 < 1) || (portp->portData->vl0 > STL_MAX_VLS)) {
			portp->portData->vl0 = 1;
		}

		if ((nodep->index != 0) || (portp->index != sm_config.port)) {
			if ((portp->portData->capmask & PI_CM_IS_SM) != 0) {
				/* get and save the SmInfo of the SM node for later processing */
				/* Note: If we encountered a temporarily non-responding node, skip this step */
				status = sm_getSmInfo(topop, nodep, portp);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					IB_EXIT(__func__, status);
					return status;
				}
			}
		}

	  cleanup_down_ports:
		/* keep track of live ports */
		if (portp->state >= IB_PORT_INIT) {
			bitset_set(&nodep->activePorts, portp->index);
			INCR_PORT_COUNT(topop, nodep);

			if (portp->state == IB_PORT_INIT) {
				bitset_set(&nodep->initPorts, portp->index);
				nodep->portsInInit = nodep->initPorts.nset_m;
			}

			// if applicable, take it out of the list of removed ports
			(void) sm_removedEntities_clearPort(nodep, portp);
		} else if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (sm_dynamic_port_alloc() && !portp->portData->linkPolicyViolation) {
				/* free port record associated with the DOWN ports of switches */
				/* except when port is only down administatively */
				sm_free_port(topop, portp);
			}
			if (!topology_changed) {
				if (!oldnodep) {
					oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
				}
				if (oldnodep) {
					if (switchPortChange || oldnodep->switchInfo.u1.s.PortStateChange) {
						oldportp = sm_get_port(oldnodep, portp->index);
						if (oldportp && (oldportp->state >= IB_PORT_INIT)) {
							linkednodep = sm_find_node(&old_topology, oldportp->nodeno);
							if (linkednodep
								&& (linkednodep->nodeInfo.NodeType == NI_TYPE_SWITCH)) {
								// Was up and connected to a switch.  Flag topology change.
								topology_changed = 1;
								/* 
								   IB_LOG_INFINI_INFO_FMT(__func__, "Switch reporting
								   port change, node %s guid "FMT_U64" link down port=%d",
								   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								   portp->index); */
							}
						}
					}
				}
			}
		}
	}

	bitset_free(&needPortInfo);
	bitset_free(&skipPorts);
	vs_pool_free(&sm_pool, newPortInfos);

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		start_port = 1;
	}

	if (new_node == 1) {
		// initialize DGs for the node
		smSetupNodeDGs(nodep);

		// initialize VFs for the node
		smSetupNodeVFs(nodep);
	}

	nodep->aggregateEnable = sm_config.use_aggregates;
	if (oldnodep)
		nodep->aggregateEnable &= oldnodep->aggregateEnable;

	IB_EXIT(__func__, VSTATUS_OK);
	return (VSTATUS_OK);
}

/*
 * Find the node has description as "name"
*/
Node_t *
sm_find_node_by_name(Topology_t * topop, char *name)
{
	cl_map_item_t 	*cl_map_item;
	cl_qmap_t	*cl_map;
	Node_t		*nodep;

	IB_ENTER(__func__, topop, name, 0, 0);

	cl_map = topop->nodeMap;

	if (!cl_map) {
		IB_EXIT(__func__, NULL);
		return (NULL);
	}

	for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map);
			cl_map_item = cl_qmap_next(cl_map_item)) {
		nodep = (Node_t *) cl_qmap_obj((const cl_map_obj_t * const) cl_map_item);
		if (!strcmp(name, sm_nodeDescString(nodep))) {
				IB_EXIT(__func__, cl_map_item);
				return nodep;
		}
	}
	IB_EXIT(__func__, NULL);
	return (NULL);
}

/*
 * the input guid must be a node guid since this is looking in the node map
*/
Node_t *
sm_find_guid(Topology_t * topop, uint64_t guid)
{
	cl_map_item_t *pItem;

	IB_ENTER(__func__, topop, &guid, 0, 0);

#ifdef TRACK_SEARCHES
	sm_node_guid_cnt++;
#endif

	if (topop->nodeMap
		&& ((pItem = cl_qmap_get(topop->nodeMap, guid)) != cl_qmap_end(topop->nodeMap))) {
		IB_EXIT(__func__, pItem);
		return (Node_t *) cl_qmap_obj((const cl_map_obj_t * const) pItem);
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

Node_t *
sm_find_quarantined_guid(Topology_t * topop, uint64_t guid)
{
	cl_map_item_t *pItem;

	IB_ENTER(__func__, topop, &guid, 0, 0);

#ifdef TRACK_SEARCHES
	sm_node_guid_cnt++;
#endif

	if (topop->quarantinedNodeMap
		&& ((pItem = cl_qmap_get(topop->quarantinedNodeMap, guid)) !=
			cl_qmap_end(topop->quarantinedNodeMap))) {
		IB_EXIT(__func__, pItem);
		return (Node_t *) cl_qmap_obj((const cl_map_obj_t * const) pItem);
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

void
sm_print_node_item(IN cl_map_item_t * const p_map_item, IN void *context)
{
	printf("Node: Key: " FMT_U64 "\n", p_map_item->key);
	return;
}


void
sm_dump_node_map(Topology_t * topop)
{
	cl_qmap_apply_func(topop->nodeMap, sm_print_node_item, NULL);
}


/*
 * the input guid must be a node guid since this is looking in the node map
*/
Node_t *
sm_find_next_guid(Topology_t * topop, uint64_t guid)
{
	cl_map_item_t *pItem;

	IB_ENTER(__func__, topop, &guid, 0, 0);

#ifdef TRACK_SEARCHES
	sm_node_guid_cnt++;
#endif

	if ((pItem = cl_qmap_get_next(topop->nodeMap, guid)) != cl_qmap_end(topop->nodeMap)) {
		/* FIXME: Should never happen. but there must be a bug in the qmap code */
		if (pItem->key == guid) {
			IB_EXIT(__func__, pItem);
			return NULL;
		}
		IB_EXIT(__func__, pItem);
		return (Node_t *) cl_qmap_obj((const cl_map_obj_t * const) pItem);
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}


Node_t *
sm_find_node(Topology_t * topop, int32_t nodeno)
{
	cl_map_item_t *pItem;

	IB_ENTER(__func__, topop, nodeno, 0, 0);

#ifdef TRACK_SEARCHES
	sm_node_id_cnt++;
#endif

	if (nodeno < 0)
		return NULL;
	if (topop->nodeArray != NULL && nodeno < topop->num_nodes) {
		if (topop->nodeArray[nodeno] != NULL) {
		return topop->nodeArray[nodeno];
		}
		if (topop->nodeIdMap != NULL) {
			if ((pItem = cl_qmap_get(topop->nodeIdMap, (uint64_t) nodeno)) !=
 					cl_qmap_end(topop->nodeIdMap)) {
				IB_EXIT(__func__, pItem);
				return (Node_t *) cl_qmap_obj((const cl_map_obj_t * const) pItem);
			}
		}
		IB_EXIT(__func__, NULL);
		return NULL;
	}
	if (topop->nodeIdMap == NULL)
		return NULL;
	if ((pItem =
		 cl_qmap_get(topop->nodeIdMap, (uint64_t) nodeno)) != cl_qmap_end(topop->nodeIdMap)) {
		IB_EXIT(__func__, pItem);
		return (Node_t *) cl_qmap_obj((const cl_map_obj_t * const) pItem);
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}


Node_t *
sm_find_node_by_path(Topology_t * topop, Node_t * head, uint8_t * path)
{
	uint8_t pathIndex;
        Node_t *node_ptr;
        
	IB_ENTER(__func__, topop, path, 0, 0);

#ifdef TRACK_SEARCHES
	sm_node_id_cnt++;
#endif

	if ( (!path) || (path[0]>IBA_MAX_PATHSIZE) ) {
                IB_EXIT(__func__, NULL);
		return NULL;
	}
       
	node_ptr = head ? head : topop->node_head;

	if (!node_ptr) {
		IB_EXIT(__func__, NULL);
		return NULL;
	}

	for (pathIndex=1; pathIndex<=path[0]; pathIndex++) {
		Port_t * portp = sm_get_port(node_ptr, path[pathIndex]);
		if (!portp) {
			IB_EXIT(__func__, NULL);
			return NULL;
		}
		node_ptr = sm_find_node(topop, portp->nodeno);
		if (!node_ptr) {
			IB_EXIT(__func__, NULL);
			return NULL;
		}
	}

	IB_EXIT(__func__, node_ptr);
	return (node_ptr);
}


Node_t *
sm_find_switch(Topology_t * topop, uint16_t swIdx)
{
	Node_t *nodep;

	IB_ENTER(__func__, topop, swIdx, 0, 0);

#ifdef TRACK_SEARCHES
	sm_switch_id_cnt++;
#endif

	for_all_switch_nodes(topop, nodep) {
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && nodep->swIdx == swIdx) {
			IB_EXIT(__func__, swIdx);
			return nodep;
		}
	}

	IB_EXIT(__func__, nodep);
	return NULL;
}


Node_t *
sm_find_port_node(Topology_t * topop, Port_t * portp)
{
	uint32_t portno;
	Node_t *nodep;
	Port_t *nPortp;

	IB_ENTER(__func__, topop, portp, 0, 0);

#ifdef TRACK_SEARCHES
	sm_port_node_cnt++;
#endif
	if (portp == NULL) {
		IB_EXIT(__func__, NULL);
		return (NULL);
	}

	if (sm_valid_port(portp) && portp->portData->nodePtr) {
		return (Node_t *) portp->portData->nodePtr;
	}

	portno = portp->index;

	for_all_nodes(topop, nodep) {
		if (nodep->nodeInfo.NumPorts >= portno) {
			if ((nPortp = sm_get_port(nodep, portno)) && nPortp == portp) {
				IB_EXIT(__func__, nodep);
				return (nodep);
			}
		}
	}


	IB_EXIT(__func__, nodep);
	return (NULL);
}

Port_t *
sm_find_port(Topology_t * topop, int32_t nodeno, int32_t portno)
{
	Node_t *nodep;
	Port_t *portp;

	IB_ENTER(__func__, topop, nodeno, portno, 0);

#ifdef TRACK_SEARCHES
	sm_np_id_cnt++;
#endif

	nodep = sm_find_node(topop, nodeno);
	if (nodep && portno >= 0 && (int32_t) nodep->nodeInfo.NumPorts >= portno) {
		portp = sm_get_port(nodep, portno);
		IB_EXIT(__func__, portp);
		return (portp);
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

Port_t *
sm_find_node_port(Topology_t * topop, Node_t * nodep, int32_t portno)
{
	Port_t *portp;

	IB_ENTER(__func__, topop, nodep, portno, 0);

#ifdef TRACK_SEARCHES
	sm_np_id_cnt++;
#endif

	if (nodep && portno >= 0) {
		if ((int32_t) nodep->nodeInfo.NumPorts >= portno) {
			portp = sm_get_port(nodep, portno);
			IB_EXIT(__func__, portp);
			return (portp);
		} else {
			IB_EXIT(__func__, NULL);
			return (NULL);
		}
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

/*
 * return the port only if it's in active state
 */
Port_t *
sm_find_active_port_guid(Topology_t * topop, uint64_t guid)
{
	Port_t *portp = NULL;

	if ((portp = sm_find_port_guid(topop, guid)) && portp->state >= IB_PORT_ARMED)
		return portp;
	return NULL;
}

Port_t *
sm_find_port_guid(Topology_t * topop, uint64_t guid)
{
	cl_map_item_t *pItem;

	IB_ENTER(__func__, topop, &guid, 0, 0);

#ifdef TRACK_SEARCHES
	sm_port_guid_cnt++;
#endif

	if (topop->portMap
		&& ((pItem = cl_qmap_get(topop->portMap, guid)) != cl_qmap_end(topop->portMap))) {
		IB_EXIT(__func__, pItem);
		return (Port_t *) cl_qmap_obj((const cl_map_obj_t * const) pItem);
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

Port_t *
sm_find_port_peer(Topology_t * topop, uint64_t node_guid, int32_t port_no)
{
	Port_t *pPort;
	Node_t *nodep;

	IB_ENTER(__func__, topop, &node_guid, 0, 0);

	if ((nodep = sm_find_guid(topop, node_guid)) != NULL) {
		if ((pPort = sm_find_node_port(topop, nodep, port_no)) != NULL) {
			if (pPort->state == IB_PORT_ACTIVE) {
				return sm_find_port(topop, pPort->nodeno, pPort->portno);
			}
		}
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}


/*
 * Only return the port if if's active
 */
Port_t *
sm_find_active_port_lid(Topology_t * topop, STL_LID lid)
{
	Port_t *portp = NULL;

	if ((portp = sm_find_port_lid(topop, lid)) && portp->state >= IB_PORT_ARMED)
		return portp;
	return NULL;
}

Port_t *
sm_find_port_lid(Topology_t * topop, STL_LID lid)
{
	STL_LID topLid;
	Node_t *nodep = NULL;
	Port_t *portp = NULL;

	IB_ENTER(__func__, topop, lid, 0, 0);

#ifdef TRACK_SEARCHES
	sm_port_lid_cnt++;
#endif

	if (lid < STL_LID_UNICAST_BEGIN || lid > STL_GET_UNICAST_LID_MAX()) 
		return (NULL);
	/* see if we can find it quickly through lidmap */
	if (topop == &old_topology)
		nodep = lidmap[lid].oldNodep;
	else
		nodep = lidmap[lid].newNodep;

	if (nodep) {
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp)) {
				topLid = portp->portData->lid + (1 << portp->portData->lmc);
				if ((portp->portData->lid <= lid) && (lid < topLid)) {
					IB_EXIT(__func__, portp);
					return (portp);
				}
			}
		}
	}

	/* default brute force search */
	for_all_nodes(topop, nodep) {
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp)) {
				topLid = portp->portData->lid + (1 << portp->portData->lmc);
				if ((portp->portData->lid <= lid) && (lid < topLid)) {
					IB_EXIT(__func__, portp);
					return (portp);
				}
			}
		}
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

Port_t *
sm_find_node_and_port_lid(Topology_t * topop, STL_LID lid, Node_t ** nodePtr)
{
	STL_LID topLid;
	Node_t *nodep = NULL;
	Port_t *portp = NULL;

	IB_ENTER(__func__, topop, lid, 0, 0);

#ifdef TRACK_SEARCHES
	sm_np_lid_cnt++;
#endif
	*nodePtr = NULL;

	if (lid < STL_LID_UNICAST_BEGIN || lid > STL_GET_UNICAST_LID_MAX()) 
		return (NULL);
	/* see if we can find it quickly through lidmap (if it exists) */
	if (lidmap) {
		if (topop == &old_topology)
			nodep = lidmap[lid].oldNodep;
		else
			nodep = lidmap[lid].newNodep;
	}

	if (nodep) {
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp)) {
				topLid = portp->portData->lid + (1 << portp->portData->lmc);
				if ((portp->portData->lid <= lid) && (lid < topLid)) {
					*nodePtr = nodep;
					IB_EXIT(__func__, portp);
					return (portp);
				}
			}
		}
	}

	/* default brute force search */
	for_all_nodes(topop, nodep) {
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp)) {
				topLid = portp->portData->lid + (1 << portp->portData->lmc);
				if ((portp->portData->lid <= lid) && (lid < topLid)) {
					*nodePtr = nodep;
					IB_EXIT(__func__, portp);
					return (portp);
				}
			}
		}
	}

	IB_EXIT(__func__, NULL);
	return (NULL);
}

// Returns the node and port on the other side of the input Port_t structure.
//
Port_t *
sm_find_neighbor_node_and_port(Topology_t * topop, Port_t * portp, Node_t ** nodePtr)
{
	Node_t *neighborNodep = NULL;
	Port_t *neighborPortp = NULL;

	IB_ENTER(__func__, topop, portp, 0, 0);

	*nodePtr = NULL;

	neighborNodep = sm_find_node(topop, portp->nodeno);
	if (neighborNodep != NULL) {
		neighborPortp = sm_find_node_port(topop, neighborNodep, portp->portno);
		if (sm_valid_port(neighborPortp)) {
			*nodePtr = neighborNodep;
			IB_EXIT(__func__, neighborPortp);
			return neighborPortp;
		}
	}

	IB_EXIT(__func__, NULL);
	return NULL;
}

// For a given lid and port number, this function returns all of the topology
// data associated with both sides of the link at the referenced port.  To
// provide adequate diagnostic information, it returns both external ports and,
// in the case of switches, the internal port zero (for channel adapters, the
// two will be duplicates).
//
// To make the interface a little less cumbersome, it will skip NULL output
// pointers.
//
Status_t
sm_find_node_and_port_pair_lid(Topology_t * topop, STL_LID lid, uint32_t portIndex,
							   Node_t ** nodePtr, Port_t ** intPortPtr, Port_t ** extPortPtr,
							   Node_t ** neighborNodePtr, Port_t ** neighborIntPortPtr,
							   Port_t ** neighborExtPortPtr)
{
	Node_t *nodep, *neighborNodep;
	Port_t *portp, *extPortp, *neighborPortp, *neighborExtPortp;

	IB_ENTER(__func__, topop, lid, portIndex, 0);

	// grab the node and internal port
	portp = sm_find_node_and_port_lid(&old_topology, lid, &nodep);
	if (nodep != NULL && sm_valid_port(portp)) {
		// grab the external port
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			extPortp = sm_find_node_port(&old_topology, nodep, portIndex);
		} else {
			extPortp = portp;
		}
		if (sm_valid_port(extPortp)) {
			// grab the neighboring node and external port
			neighborExtPortp =
				sm_find_neighbor_node_and_port(&old_topology, extPortp, &neighborNodep);
			if (sm_valid_port(neighborExtPortp)) {
				// grab the neighboring internal port
				if (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
					neighborPortp = sm_find_node_port(&old_topology, neighborNodep, 0);
				} else {
					neighborPortp = neighborExtPortp;
				}
				if (sm_valid_port(neighborPortp)) {
					if (nodePtr != NULL)
						*nodePtr = nodep;
					if (intPortPtr != NULL)
						*intPortPtr = portp;
					if (extPortPtr != NULL)
						*extPortPtr = extPortp;
					if (neighborNodePtr != NULL)
						*neighborNodePtr = neighborNodep;
					if (neighborIntPortPtr != NULL)
						*neighborIntPortPtr = neighborPortp;
					if (neighborExtPortPtr != NULL)
						*neighborExtPortPtr = neighborExtPortp;
					IB_EXIT(__func__, VSTATUS_OK);
					return VSTATUS_OK;
				}
			}
		}
	}

	if (nodePtr != NULL)
		*nodePtr = NULL;
	if (intPortPtr != NULL)
		*intPortPtr = NULL;
	if (extPortPtr != NULL)
		*extPortPtr = NULL;
	if (neighborNodePtr != NULL)
		*neighborNodePtr = NULL;
	if (neighborIntPortPtr != NULL)
		*neighborIntPortPtr = NULL;
	if (neighborExtPortPtr != NULL)
		*neighborExtPortPtr = NULL;

	IB_EXIT(__func__, VSTATUS_BAD);
	return VSTATUS_BAD;
}

// -------------------------------------------------------------------------- //

// this just bundles a repetative CSM formatting block within the
// sm_initialize_port function.  note that unconditionally calling
// smCsmFormatNodeId is a little too expensive given how many times
// sm_initialize_port gets called
#define SM_INIT_PORT_PREP_CSM() \
	if (!csmNodesPrepped) { \
		smCsmFormatNodeId(&localNode, (uint8_t *)sm_nodeDescString(nodep), portp->index, nodep->nodeInfo.NodeGUID); \
		smCsmFormatNodeId(&remoteNode, (uint8_t *)sm_nodeDescString(new_nodep), new_portp->index, new_nodep->nodeInfo.NodeGUID); \
		csmNodesPrepped = 1; \
	}

uint8_t *
sm_set_last_hop_path(Node_t * nodep, Port_t * portp, uint8_t * last_hop_path)
{
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index != 0) {
		/* Get info of ports on the switch, so the DR packet does not have to go out of the
		   switch */
		last_hop_path[0] = 0;
		last_hop_path[1] = 0;
		return NULL;
	} else {
		uint8_t *path;

		path = PathToPort(nodep, portp);

		last_hop_path[0] = 1;
		last_hop_path[1] = path[path[0]];
	}
#if 0
	if (last_hop_path)
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "for port %d of node_guid " FMT_U64
							   "is path[0] is %d path[1] is %d\n", portp->index,
							   nodep->nodeInfo.NodeGUID, last_hop_path[0], last_hop_path[1]);
	else
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "LR init port for port %d of node guid " FMT_U64 "\n",
							   portp->index, nodep->nodeInfo.NodeGUID);
#endif
	return last_hop_path;
}

#define ALLOW_SM_TO_OVERRIDE_PORT_DEFINED_FLOW_DISABLE (1)
int
anyFlowDisableDifferences(Topology_t * topop, Node_t * nodep, Port_t * portp,
						  uint32 flowControlMask, uint32 * newMask)
{
	int vl;
	int vf;
	int anyChanged = 0;
	VlVfMap_t vlvfmap;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	*newMask = flowControlMask;	// new mask starts by looking like the current mask, gets
								// updated if there are found differences.

	topop->routingModule->funcs.select_vlvf_map(topop, nodep, portp, &vlvfmap);	// select is based
																			// on appropriate
																			// instance of
																			// _select_vlvf_map()

	// On a per-VL basis, the vf's are identified. 
	// If ANY of those VF explicitly disable flow control for that vl, then it is to be
	// disabled for all VF that use that vl via this port.
	for (vl = 0; vl < STL_MAX_VLS; vl++) {
		int disableFlowControlForThisVL = 0;
		uint32 vlMaskBitToChange;
		// VL 15 flow control superceds any VF-defined value
		// so use the passed-in value for the disable (rather than the VF's)
		if (vl == 15) {
			disableFlowControlForThisVL = (flowControlMask >> 15) & 1;
		}

		for (vf = 0; (vf = bitset_find_next_one(&vlvfmap.vf[vl], vf)) != -1; ++vf){
			if (VirtualFabrics->v_fabric_all[vf].flowControlDisable) {
				disableFlowControlForThisVL = 1;
				break;	// for efficiency, no more have to be checked.
			}
		}

		vlMaskBitToChange = 1 << vl;
		if (disableFlowControlForThisVL) {
			// If not already disabled, disable it in the new mask
			if (!(vlMaskBitToChange & flowControlMask)) {
				*newMask |= vlMaskBitToChange;
				anyChanged = 1;
			}
		} else {
#if ALLOW_SM_TO_OVERRIDE_PORT_DEFINED_FLOW_DISABLE
			// Via conditional compile, 
			// if the FM is allowed to over-ride any port-defined disable, then
			// if flow is disabled for the port when it should not be, re-enable it
			if (vlMaskBitToChange & flowControlMask) {
				*newMask &= ~vlMaskBitToChange;
				anyChanged = 1;
			}
#endif
		}
		bitset_free(&vlvfmap.vf[vl]);
	}

	return anyChanged;
}

Status_t 
sm_verifyPortSpeedAndWidth(Topology_t *topop, Node_t *nodep, Port_t *portp)
{

	Port_t *con_portp;
	uint16_t best_width, best_speed;
	SMLinkPolicyXmlConfig_t  sm_link_policy;
	char buf[80] = {0}, buf2[80] = {0};

	if(!sm_valid_port(portp))
		return(VSTATUS_BAD);
	
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
		portp->portData->portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW)
			sm_link_policy = sm_config.isl_link_policy;
		else
			sm_link_policy = sm_config.hfi_link_policy;



	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
		portp->portData->portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW)
			sm_link_policy = sm_config.isl_link_policy;
		else
			sm_link_policy = sm_config.hfi_link_policy;



	/**mark this port as down if speed or width violate policy.**/
	//check if LW.A outside policy 
	if(sm_link_policy.width_policy.enabled) {
		if(sm_link_policy.width_policy.policy &&
			(portp->portData->portInfo.LinkWidth.Active < sm_link_policy.width_policy.policy)) {
			IB_LOG_WARN_FMT(__func__,"node %s port %d: Link width (%s) lower than policy spec (%s)",
				sm_nodeDescString(nodep), portp->index,
				StlLinkWidthToText(portp->portData->portInfo.LinkWidth.Active, buf, sizeof(buf)),
				StlLinkWidthToText(sm_link_policy.width_policy.policy, buf2, sizeof(buf2)));
			if(portp->portData->portInfo.PortStates.s.PortState != IB_PORT_INIT)
				sm_bounce_port(topop, nodep, portp);
			else
				portp->portData->linkPolicyViolation |= LINK_POLICY_VIOLATION_MIN;
			return(VSTATUS_BAD);
		}
	}
	
	//check if LS.A outside policy 
	if(sm_link_policy.speed_policy.enabled) {
		if(sm_link_policy.speed_policy.policy &&
			(portp->portData->portInfo.LinkSpeed.Active < sm_link_policy.speed_policy.policy)) {
			IB_LOG_WARN_FMT(__func__,"node %s port %d: Link Speed (%s) lower than policy spec (%s)",
				sm_nodeDescString(nodep), portp->index,
				StlLinkSpeedToText(portp->portData->portInfo.LinkSpeed.Active, buf, sizeof(buf)),
				StlLinkSpeedToText(sm_link_policy.speed_policy.policy, buf2, sizeof(buf2)));
			if(portp->portData->portInfo.PortStates.s.PortState != IB_PORT_INIT)
				sm_bounce_port(topop, nodep, portp);
			else
				portp->portData->linkPolicyViolation |= LINK_POLICY_VIOLATION_MIN;
			return(VSTATUS_BAD);
		}
	}
	
	// no min policy violation detected, clear any previous flag set
	portp->portData->linkPolicyViolation &= ~LINK_POLICY_VIOLATION_MIN;
	
	/* 
	 *  find the other side of the cable.
	 */
	if (portp->index == 0) {
		con_portp = portp;
	} else {
		con_portp = sm_find_port(topop, portp->nodeno, portp->portno);
	}

	//We may not have discovered connected port yet in which case we can't enforce
	// the "supported" policy, so just return. 
	if(!sm_valid_port(con_portp))
		return(VSTATUS_OK);

	//check if LW.A is not highest common supported
	best_width = StlBestLinkWidth(portp->portData->portInfo.LinkWidth.Supported &
											con_portp->portData->portInfo.LinkWidth.Supported) ;
	if(portp->portData->portInfo.LinkWidth.Active != best_width) {
		IB_LOG_WARN_FMT(__func__,"node %s nodeGuid "FMT_U64" port %d: Link width (%u) not set to highest supported width (%u).",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, 
			StlLinkWidthToInt(portp->portData->portInfo.LinkWidth.Active),
			StlLinkWidthToInt(best_width));
		if(sm_link_policy.width_policy.enabled) {
			if(portp->portData->portInfo.PortStates.s.PortState != IB_PORT_INIT){
				//We will bounce port from the switch side to bring it back to init, 
				// This may cause problems in back to back configurations. Link Policy can be disabled
				// in this case to avoid using this feature.
				sm_bounce_link(topop, nodep, portp);
			}
			else {
				portp->portData->linkPolicyViolation |= LINK_POLICY_VIOLATION_SUP;
				con_portp->portData->linkPolicyViolation |= LINK_POLICY_VIOLATION_SUP;
			}
			return(VSTATUS_BAD);
		}
	}
	//Check if LS.A is not highest common supported
	best_speed = StlBestLinkSpeed(portp->portData->portInfo.LinkSpeed.Supported &
										con_portp->portData->portInfo.LinkSpeed.Supported) ;
	if(portp->portData->portInfo.LinkSpeed.Active != best_speed) {
		char buffer1[16], buffer2[16];
		IB_LOG_WARN_FMT(__func__,"node %s nodeGuid "FMT_U64" port %d: Link Speed (%s) not set to highest supported speed (%s)",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, 
			StlLinkSpeedToText(portp->portData->portInfo.LinkSpeed.Active, buffer1, 16),
			StlLinkSpeedToText(best_speed, buffer2, 16));
		if(sm_link_policy.speed_policy.enabled) {
			if(portp->portData->portInfo.PortStates.s.PortState != IB_PORT_INIT) {
				//We will bounce port from the switch side to bring it back to init,
				//This may cause problems in back to back configurations. Link Policy can be disabled
				// in this case to avoid using this feature.
				sm_bounce_link(topop, nodep, portp);
			}
			else {
				portp->portData->linkPolicyViolation |= LINK_POLICY_VIOLATION_SUP;
				con_portp->portData->linkPolicyViolation |= LINK_POLICY_VIOLATION_SUP;
			}
			return(VSTATUS_BAD);
		}
	}

	//link ok, make sure violation cleared on both ports portData so link
	//can reenter fabric if removed

	portp->portData->linkPolicyViolation &= ~LINK_POLICY_VIOLATION_SUP;
	con_portp->portData->linkPolicyViolation &= ~LINK_POLICY_VIOLATION_SUP;

	return(VSTATUS_OK);
}

/**
 * Test if LID is in multicast/collective space.
 * Can also just pass in multicast mask bit count, but matchBits
 * must be <= 20.
 *
 * @param matchBits number of leading 1's that must be in @c lid for @c lid to NOT be considered a unicast LID
 */
static int
is_valid_8B_10B_unicast_lid(STL_LID lid, uint8_t matchBits)
{
	DEBUG_ASSERT(matchBits <= 20);

	if (lid >= 0xFFFFF)
		return 0;

	uint32_t mask = ((1 << matchBits) - 1) << (20 - matchBits);
	if (mask != 0 && (lid & mask) == mask)
		return 0;

	return 1;
}

/**
 * Different from smValidatePortPKey() in that both PKeys must be
 * full PKeys, not at least one PKey must be full.
 */
static int
valid_full_portPKey(uint16_t pkey, uint16_t tblSize, Port_t *p, int is10B)
{
	if (PKEY_TYPE(pkey) != PKEY_TYPE_FULL)
		return 0;

	int i;
	for (i = 0; i < tblSize; ++i) {
		uint16_t k = p->portData->pPKey[i].AsReg16;
		if (PKEY_VALUE(k) == 0)
			continue; // TODO sanity check, 0x8000 should never happen. Can remove?

		if (is10B)
			k &= 0xfff0;

		if (PKEY_TYPE(k) == PKEY_TYPE_FULL &&
			PKEY_VALUE(k) == PKEY_VALUE(pkey))
			return 1;
	}

	return 0;
}

Status_t
sm_initialize_port(Topology_t * topop, Node_t * nodep, Port_t * portp, SmpAddr_t * addr)
{
	Node_t *new_nodep;
	Port_t *new_portp;
	Port_t *mask3Portp;
	Port_t *new_mask3Portp;
	Status_t status = VSTATUS_OK;
	STL_PORT_INFO portInfo;
	uint8_t needSet = 0;
	uint8_t smslChange = 0;
	uint8_t enforcePkey = 0;
	uint16_t highVlLimit = 0;
	SmCsmNodeId_t localNode, remoteNode;
	int csmNodesPrepped = 0;
	uint32_t amod = 0;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;
	uint8_t maxLinkMTU;
	SMLinkPolicyXmlConfig_t  sm_link_policy;
	uint16_t desiredPortPacketFormats;

	// Setup the port MTU per VL map.
	int vl, vf, mtu;
	VlVfMap_t vlvfmap;

	/* 
	 *  find the other side of the cable.
	 */
	if (portp->index == 0) {
		new_portp = portp;
	} else {
		new_portp = sm_find_port(topop, portp->nodeno, portp->portno);
	}

	/* PR 105589 - SM crash after failing to get portinfo from switch to switch link */
	if (new_portp == NULL || new_portp->state == IB_PORT_DOWN) {
		if (new_portp && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
			&& (new_nodep = sm_find_port_node(topop, new_portp))) {
			if (new_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {

				SM_INIT_PORT_PREP_CSM();
				smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_FABRIC_INIT_ERROR, &localNode,
								&remoteNode,
								"Failed to init switch-to-switch link which was reported down");

				IB_EXIT(__func__, VSTATUS_INVALID_PORT);
				return (VSTATUS_INVALID_PORT);
			}
		}
		IB_LOG_WARN_FMT(__func__,
						"port on other side of node %s index %d port %d is not active",
						sm_nodeDescString(nodep), nodep->index, portp->index);
		IB_EXIT(__func__, VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	new_nodep = sm_find_port_node(topop, new_portp);
	if (new_nodep == NULL) {
		IB_LOG_WARN_FMT(__func__, "Cannot find port-node %s index %d port %d",
						sm_nodeDescString(nodep), nodep->index, portp->index);
		IB_EXIT(__func__, VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	// Mask3 switch attributes are associated with switch port 0
	mask3Portp = portp;
	new_mask3Portp = new_portp;

	if (new_portp != portp) {
		// check remote node to see if switch,
		// if so, get switch port 0
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			mask3Portp = sm_get_port(nodep, 0);
		}
		if (new_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			new_mask3Portp = sm_get_port(new_nodep, 0);
		}
	}

	portInfo = portp->portData->portInfo;
	// don't complain about 1x if the port is only enabled for 1x
	// or its port 0 of a switch
	if (portp->index != 0 && portInfo.LinkWidth.Active & STL_LINK_WIDTH_1X
		&& (portInfo.LinkWidth.Enabled & ~STL_LINK_WIDTH_1X)
		&& (new_portp->portData->portInfo.LinkWidth.Enabled & ~STL_LINK_WIDTH_1X)) {
		SM_INIT_PORT_PREP_CSM();
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_FABRIC_INIT_ERROR, &localNode, &remoteNode,
						"Port is running at 1X width");
	}

	/* 
	 *  set some sane default values.
	 */
	maxLinkMTU = Min(portp->portData->mtuSupported, new_portp->portData->mtuSupported);

	portp->portData->vl1 = Min(portp->portData->vl0, new_portp->portData->vl0);

	// If we're configuring a pre-configured fabric and we have SCAE disabled, make sure to disable it on every port
	if (portp->portData->portInfo.PortMode.s.IsActiveOptimizeEnabled && !sm_is_scae_allowed(nodep)) {
		portInfo.PortMode.s.IsActiveOptimizeEnabled = 0;
		needSet = 1;
	}

	/***** not applicable to switch ports ****/
	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		if (portInfo.CapabilityMask3.s.IsAddrRangeConfigSupported) {
			if (portInfo.MultiCollectMask.MulticastMask != SM_DEFAULT_MULTICAST_MASK) {
				portInfo.MultiCollectMask.MulticastMask = SM_DEFAULT_MULTICAST_MASK;
				needSet = 1;
			}
			if (portInfo.MultiCollectMask.CollectiveMask != SM_DEFAULT_COLLECTIVE_MASK) {
				portInfo.MultiCollectMask.CollectiveMask = SM_DEFAULT_COLLECTIVE_MASK;
				needSet = 1;
			}
		}
	}

	// applicable to everyone except switch external ports
	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
		|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0)) {
		if (portInfo.M_Key != sm_config.mkey
			|| portInfo.M_KeyLeasePeriod != sm_mkey_lease_period
			|| portInfo.s1.M_KeyProtectBits != sm_mkey_protect_level) {
			if (smDebugPerf) {
				IB_LOG_INFINI_INFO_FMT(__func__,
									   "Setting Mkey[" FMT_U64 " was " FMT_U64
									   "], mkeyprotect[%d was %d], mkey lease[%d was %d] for %s ["
									   FMT_U64 "], index[%d], port[%d]", sm_config.mkey,
									   portInfo.M_Key, sm_mkey_protect_level,
									   portInfo.s1.M_KeyProtectBits, sm_mkey_lease_period,
									   portInfo.M_KeyLeasePeriod, sm_nodeDescString(nodep),
									   nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
			}
			portInfo.M_Key = sm_config.mkey;
			if (portInfo.M_Key) {
				portInfo.M_KeyLeasePeriod = sm_mkey_lease_period;
				portInfo.s1.M_KeyProtectBits = sm_mkey_protect_level;
			} else {
				portInfo.M_KeyLeasePeriod = 0;
				portInfo.s1.M_KeyProtectBits = 0;
			}
			needSet = 1;
		}
		if (portInfo.SubnetPrefix != sm_config.subnet_prefix) {
			portInfo.SubnetPrefix = sm_config.subnet_prefix;
			needSet = 1;
		}
		if (portp->portData->dirty.portInfo) {
			needSet = 1;
		}
		if (portInfo.LID != portp->portData->lid) {
			portInfo.LID = portp->portData->lid;
			needSet = 1;
		}
		if (portInfo.MasterSMLID != sm_lid) {
			portInfo.MasterSMLID = sm_lid;
			needSet = 1;
		}
		if (portInfo.MaxLID != STL_GET_UNICAST_LID_MAX()) {
			// Set to greater of max_supported_lid or UNICAST_LID_MAX so that ports need not be bounced
			// unnecessarily when max_supported_lid is altered.
			if (STL_GET_UNICAST_LID_MAX() > UNICAST_LID_MAX) {
				portInfo.MaxLID = STL_GET_UNICAST_LID_MAX();
				needSet = 1;
			}
			else {
				if (portInfo.MaxLID != UNICAST_LID_MAX) {
					portInfo.MaxLID = UNICAST_LID_MAX;
					needSet = 1;
				}
			}
		}
		if (portInfo.s1.LMC != portp->portData->lmc) {
			/* switch that does not support enhanced switch port zero */
			if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0
				&& !nodep->switchInfo.u2.s.EnhancedPort0) {
				portInfo.s1.LMC = 0;
			} else
				portInfo.s1.LMC = portp->portData->lmc;
			needSet = 1;
		}
		if (portInfo.s2.MasterSMSL != sm_masterSmSl) {
			portInfo.s2.MasterSMSL = sm_masterSmSl;
			needSet = 1;
			if ((nodep->index == 0) && (portp->index == sm_config.port)) {
				smslChange = 1;
			}
		}
		portInfo.Violations.M_Key = 0;
		portInfo.Violations.P_Key = 0;
		portInfo.Violations.Q_Key = 0;
		if (portInfo.Subnet.Timeout != 17) {
			portInfo.Subnet.Timeout = 17;	/* JMS-6/16/03; (2^subnetTimeout*4.096msec ~= .5sec 
											   lifetime) also used to determine frequency of
											   traps sent */
			needSet = 1;
		}
	}

	/* set required no ops here for non-enhanced switch port zero switches */
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0
		&& !nodep->switchInfo.u2.s.EnhancedPort0) {
		portInfo.s1.LMC = 0;
		portInfo.LinkWidth.Enabled = STL_LINK_WIDTH_NOP;
		portInfo.PortStates.s.PortState = IB_PORT_NOP;
		portInfo.PortStates.s.PortPhysicalState = IB_PORT_PHYS_NOP;
		portInfo.LinkSpeed.Enabled = STL_LINK_SPEED_NOP;
	}

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
		portp->portData->portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW)
			sm_link_policy = sm_config.isl_link_policy;
		else
			sm_link_policy = sm_config.hfi_link_policy;


	// applicable to everyone except enhanced switch port zero
	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
		|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
			&& (portp->index > 0
				|| (portp->index == 0 && nodep->switchInfo.u2.s.EnhancedPort0)))) {

		//SM does not change LS.E or LW.E
		//Determine LWD.E Setting based on value parsed in xml config and active value
		portInfo.LinkWidth.Enabled = STL_LINK_WIDTH_NOP;
		portInfo.LinkSpeed.Enabled = STL_LINK_SPEED_NOP;
		uint16_t lwde = portp->portData->portInfo.LinkWidth.Active;
		

		if(lwde == STL_LINK_WIDTH_4X){
			if(sm_link_policy.link_max_downgrade >= 3)
				lwde |= STL_LINK_WIDTH_3X | STL_LINK_WIDTH_2X | STL_LINK_WIDTH_1X;
			if(sm_link_policy.link_max_downgrade == 2)
				lwde |= STL_LINK_WIDTH_3X | STL_LINK_WIDTH_2X;
			if(sm_link_policy.link_max_downgrade == 1)
				lwde |= STL_LINK_WIDTH_3X;
		}else if(lwde == STL_LINK_WIDTH_3X){
			if(sm_link_policy.link_max_downgrade >= 2)
				lwde |= STL_LINK_WIDTH_2X | STL_LINK_WIDTH_1X;
			if(sm_link_policy.link_max_downgrade == 1)
				lwde |= STL_LINK_WIDTH_2X;
		}else if(lwde == STL_LINK_WIDTH_2X){
			if(sm_link_policy.link_max_downgrade >= 1)
				lwde |= STL_LINK_WIDTH_1X;
		}
		lwde &= portp->portData->portInfo.LinkWidthDowngrade.Supported;
		if (lwde != portp->portData->portInfo.LinkWidthDowngrade.Enabled) {
			needSet=1;
		}

		portInfo.LinkWidthDowngrade.Enabled = lwde;
		//
		portInfo.PortStates.s.PortState = IB_PORT_NOP;
		portInfo.PortStates.s.PortPhysicalState = IB_PORT_PHYS_NOP;
		if (portInfo.s1.LMC != portp->portData->lmc) {
			portInfo.s1.LMC = portp->portData->lmc;
			needSet = 1;
		}
		// Setup Flit Control & Premption configuration registers.
		//
		// Gen1 only supports implicit interleave indicated by non-zero Max Nest Level RX Supported.

		int explicitInterleaveEnable = 0;
		if ((nodep->nodeInfo.NodeType != NI_TYPE_SWITCH || portp->index > 0) &&
			sm_valid_port(mask3Portp) && sm_valid_port(new_mask3Portp)) {
			explicitInterleaveEnable = MIN(mask3Portp->portData->portInfo.CapabilityMask3.s.IsVLMarkerSupported,
										new_mask3Portp->portData->portInfo.CapabilityMask3.s.IsVLMarkerSupported);

		}

		portInfo.PortMode.s.IsVLMarkerEnabled = explicitInterleaveEnable;

		if (portInfo.PortMode.s.IsVLMarkerEnabled) {
			// set implicit interleave to disabled
			portInfo.FlitControl.Interleave.s.MaxNestLevelTxEnabled = 0;

		} else {
			// set implicit interleave to enabled if both sides support it
			portInfo.FlitControl.Interleave.s.MaxNestLevelTxEnabled =
				MIN(portp->portData->portInfo.FlitControl.Interleave.s.MaxNestLevelRxSupported,
					new_portp->portData->portInfo.FlitControl.Interleave.s.MaxNestLevelRxSupported);
		}


		if (sm_IsInterleaveEnabled(&portInfo)) {
			// set distance mode
			portInfo.FlitControl.Interleave.s.DistanceEnabled =
				MIN(portp->portData->portInfo.FlitControl.Interleave.s.DistanceSupported,
					new_portp->portData->portInfo.FlitControl.Interleave.s.DistanceSupported);

			// Set preemption data
			// The hardware is allowed to return different values than we set for
			// MinInitial and MinTail.
			// We don't want to get stuck every sweep trying to set a value the
			// hardware will never allow.
			// Only set if configured preemption values have changed.
			portInfo.FlitControl.Preemption.MinInitial = SM_PREEMPT_HEAD_DEF / SM_PREEMPT_HEAD_INC;
			portInfo.FlitControl.Preemption.MinTail = SM_PREEMPT_TAIL_DEF / SM_PREEMPT_TAIL_INC;

			portInfo.FlitControl.Preemption.LargePktLimit =
			    	sm_evalPreemptLargePkt(sm_config.preemption.large_packet, nodep);

			if (portp->portData->portInfo.FlitControl.Preemption.LargePktLimit !=
				portInfo.FlitControl.Preemption.LargePktLimit) {
				needSet = 1;
			}

			portInfo.FlitControl.Preemption.SmallPktLimit =
				MIN(sm_evalPreemptSmallPkt(sm_config.preemption.small_packet, nodep),
					portInfo.FlitControl.Preemption.MaxSmallPktLimit);

			if (portp->portData->portInfo.FlitControl.Preemption.SmallPktLimit !=
				portInfo.FlitControl.Preemption.SmallPktLimit) {
				needSet = 1;
			}

			portInfo.FlitControl.Preemption.PreemptionLimit =
				sm_evalPreemptLimit(sm_config.preemption.preempt_limit, nodep);

			if (portp->portData->portInfo.FlitControl.Preemption.PreemptionLimit !=
				portInfo.FlitControl.Preemption.PreemptionLimit) {
				needSet = 1;
			}

		} else {
			portInfo.FlitControl.Interleave.s.DistanceEnabled = STL_PORT_FLIT_DISTANCE_MODE_NONE;
		}

		if (portp->portData->portInfo.PortMode.s.IsVLMarkerEnabled !=
			portInfo.PortMode.s.IsVLMarkerEnabled) {
			needSet = 1;
		}
		if (portp->portData->portInfo.FlitControl.Interleave.s.MaxNestLevelTxEnabled !=
			portInfo.FlitControl.Interleave.s.MaxNestLevelTxEnabled) {
			needSet = 1;
		}
		if (portp->portData->portInfo.FlitControl.Interleave.s.DistanceEnabled !=
			portInfo.FlitControl.Interleave.s.DistanceEnabled) {
			needSet = 1;
		}

		// Setup the port MTU per VL map.
		topop->routingModule->funcs.select_vlvf_map(topop, nodep, portp, &vlvfmap);

		// No need to filter by VF membership, as neighbor Mtu does not consume 
		// any resources for unused VLs. Simplifies code.
		// At least 1 VL will belong to the default VF. 
		// This assures that maxVlMtu will be overwritten from default value.
		portp->portData->maxVlMtu = 0;
		for (vl = 0; vl < STL_MAX_VLS; vl++) {
			mtu = 0;
			// VL 15 is mgmt port and should not be assigned to a VF.
			if (vl == 15) {
				bitset_free(&vlvfmap.vf[vl]);
				continue;
			}
			for (vf = 0; (vf = bitset_find_next_one(&vlvfmap.vf[vl], vf)) != -1 ; ++vf){
				mtu = MAX(mtu, VirtualFabrics->v_fabric_all[vf].max_mtu_int);
			}
			// Do not need to change unless part of a VF.
			if (mtu != 0) {
				mtu = MIN(mtu, maxLinkMTU);
				portp->portData->maxVlMtu = MAX(mtu, portp->portData->maxVlMtu);
			}
			if (vl != 15 && mtu != GET_STL_PORT_INFO_NeighborMTU(&portInfo, vl)) {
				PUT_STL_PORT_INFO_NeighborMTU(&portInfo, vl, mtu);
				needSet = 1;
			}
			bitset_free(&vlvfmap.vf[vl]);
		}

		if (portp->portData->maxVlMtu == 0)
			portp->portData->maxVlMtu = maxLinkMTU;

		// Verify VL 15 (Mgmt Port) is at 2k
		mtu = MIN(IB_MTU_2048, portp->portData->maxVlMtu);
		if (mtu != GET_STL_PORT_INFO_NeighborMTU(&portInfo, 15)) {
			PUT_STL_PORT_INFO_NeighborMTU(&portInfo, 15, mtu);
			needSet = 1;
		}
		// PR123793: Check these even if needSet is clear ---
		// B/C the SM could have restarted and the port is already setup correctly,
		// but these multicast parameters are still defaulted.
		if ((portp->index != 0) && (nodep != new_nodep) &&
			(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
			(new_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)) {
			uint32_t rate = linkWidthToRate(portp->portData);
			if (rate<IB_STATIC_RATE_2_5G) {
				rate = IB_STATIC_RATE_2_5G;
				if (portp->state <= IB_PORT_INIT) {
					IB_LOG_ERROR_FMT(__func__,
						"Port %d of NodeGuid "FMT_U64" [%s] has an invalid "
						"link width or speed. Setting max multicast data rate "
						"to SDR.", portp->index, nodep->nodeInfo.NodeGUID,
						sm_nodeDescString(nodep));
				}
			}


			if ((sm_mc_config.disable_mcast_check == McGroupBehaviorStrict)) {
				// update fabric MTU for use in multicast group creation/join requests...
				// we only care about ISL's so that's all we're checking
				if (portp->portData->maxVlMtu < sm_topop->maxMcastMtu) {
					sm_topop->maxMcastMtu = portp->portData->maxVlMtu;
				}

				if (linkrate_lt(rate, sm_topop->maxMcastRate)) {
					sm_topop->maxMcastRate = rate;
				}
			}

			if (portp->portData->maxVlMtu > sm_topop->maxISLMtu) {
				sm_topop->maxISLMtu = portp->portData->maxVlMtu;
			}

			if (linkrate_gt(rate, sm_topop->maxISLRate)) {
				sm_topop->maxISLRate = rate;
			}
		}

		// Error Action is not applicable to Port 0, even if Enhanced
		if ( (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
			|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index > 0) ) {
			if (portInfo.PortErrorAction.AsReg32 != sm_config.defaultPortErrorAction) {
				portInfo.PortErrorAction.AsReg32 = sm_config.defaultPortErrorAction;
				needSet = 1;
			}
		}

		if (portInfo.s4.OperationalVL != portp->portData->vl1) {
			portInfo.s4.OperationalVL = portp->portData->vl1;	/* JSY - interop fix */
			needSet = 1;
		} else {
			portInfo.s4.OperationalVL = 0;
		}

		// FlowControlMask only applicable to switch external ports
		if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) && (portp->index > 0))   {
			// Update to reliable transport flow control enable/disable for VL15 here
			// The enable/disable for other VL can be configured at the port - keep that config
			// unchanged
			uint32 vl15Disabled = (portInfo.FlowControlMask >> 15) & 1;
			if (vl15Disabled != (sm_config.vl15FlowControlDisable & 1)) {
				portInfo.FlowControlMask = (portInfo.FlowControlMask & 0xffff7fff) |
					((sm_config.vl15FlowControlDisable & 1) << 15);
				needSet = 1;
			}
			// Update any other reliable transport enable/disables for VL that mapped into virtual
			// fabrics keeping the port-defined value unless there is an explicit over-ride based 
			// on the FM configuration
			uint32 newMask;
			if (anyFlowDisableDifferences(topop, nodep, portp, portInfo.FlowControlMask, &newMask)) {
				portInfo.FlowControlMask = newMask;
				needSet = 1;
			}
		}

		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			// Only modify switch external ports (not port 0)
			if (portp->index > 0) {
				// If the neighbor is an HFI and their VL15Credit rate is not the same as the
				// config file, update it
				if (portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_HFI) {
					if (portInfo.PortNeighborMode.MgmtAllowed || sm_stl_appliance(portInfo.NeighborNodeGUID)) {
						if (portInfo.BufferUnits.s.VL15CreditRate != 0) {
							portInfo.BufferUnits.s.VL15CreditRate = 0; 
							needSet = 1;
						}
					}  else {
						if (portInfo.BufferUnits.s.VL15CreditRate != sm_config.vl15_credit_rate) {
							portInfo.BufferUnits.s.VL15CreditRate = sm_config.vl15_credit_rate;
							needSet = 1;
						}
					}
				}

				uint8 numVls = (portp->portData->vl1 <= 15 ? portp->portData->vl1 : portp->portData->vl1+1);
				for (vl = 0; vl < MAX(numVls, 16); vl++) {
					if (numVls < 16 && vl==numVls)  // skip to VL15
						vl = 15;

					if (portInfo.XmitQ[vl].VLStallCount !=
						portp->portData->portInfo.XmitQ[vl].VLStallCount) {
						portInfo.XmitQ[vl].VLStallCount =
							portp->portData->portInfo.XmitQ[vl].VLStallCount;
						needSet = 1;
					}
					if (portInfo.XmitQ[vl].HOQLife != portp->portData->portInfo.XmitQ[vl].HOQLife) {
						portInfo.XmitQ[vl].HOQLife = portp->portData->portInfo.XmitQ[vl].HOQLife;
						needSet = 1;
					}
				}
			}
		} else {
			// Set to 0 for non-switch nodes
			portInfo.XmitQ[0].VLStallCount = 0x0;
			portInfo.XmitQ[0].HOQLife = 0x0;
		}
	} // end everyone except switch port zero

	if (portp->state != IB_PORT_DOWN) {
		if ((status = sm_set_portPkey(topop, nodep, portp, new_nodep, new_portp, addr, &enforcePkey, &highVlLimit)) != VSTATUS_OK) {
			IB_EXIT(__func__, status);
			return (status);
		}
	}

	if (((portp->index == 0) && !nodep->switchInfo.u2.s.EnhancedPort0) || (portp->portData->vl0 == 1)) {	// Only 1 VL supported
		highVlLimit = 0;
	}

	if (portInfo.VL.HighLimit != highVlLimit) {
		portInfo.VL.HighLimit = highVlLimit;
		needSet = 1;
	}

	if (portInfo.s3.LinkInitReason != STL_LINKINIT_REASON_LINKUP) {
		portInfo.s3.LinkInitReason  = STL_LINKINIT_REASON_LINKUP;
		needSet = 1;
	}

	/* check pkey enforcement on switch linked to FI */
	if (enforcePkey) {
		if (!portInfo.s3.PartitionEnforcementInbound) {
			portInfo.s3.PartitionEnforcementInbound = 1;
			needSet = 1;
		}
		if (!portInfo.s3.PartitionEnforcementOutbound) {
			portInfo.s3.PartitionEnforcementOutbound = 1;
			needSet = 1;
		}
	} else {
		if (portInfo.s3.PartitionEnforcementInbound) {
			portInfo.s3.PartitionEnforcementInbound = 0;
			needSet = 1;
		}
		if (portInfo.s3.PartitionEnforcementOutbound) {
			portInfo.s3.PartitionEnforcementOutbound = 0;
			needSet = 1;
		}
	}

	/* configure SLID security for ingress switch port */
	if (sm_stl_port(portp)) {
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index > 0
			&& new_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			// switch to switch links trusted, reset ingress switch port fields
			if (portInfo.LID || portInfo.s1.LMC) {
				portInfo.LID = 0;
				portInfo.s1.LMC = 0;
				needSet = 1;
			}
		} else if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
				   && new_nodep->nodeInfo.NodeType == NI_TYPE_CA) {
			// switch to HFI links not trusted
			if (!sm_config.sma_spoofing_check) {
				// debug mode: instruct SMA to disable SLID security checking 
				if (portInfo.LID != STL_LID_PERMISSIVE) {
					portInfo.LID = STL_LID_PERMISSIVE;
					portInfo.s1.LMC = 0;
					needSet = 1;
				}
			} else {
				// production mode: initialize ingress switch port fields with
				// neighbor FI fields
				if (portInfo.LID != new_portp->portData->lid) {
					portInfo.LID = new_portp->portData->lid;
					needSet = 1;
				}
				if (portInfo.s1.LMC != new_portp->portData->lmc) {
					portInfo.s1.LMC = new_portp->portData->lmc;
					needSet = 1;
				}
			}
		}

		// If the neighbor of this switch port is an appliance, give this port the permissive lid
		// This may also be done during SP0 initialization (see sm_initialize_switch_LR_DR())
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index > 0 &&
			sm_stl_appliance(portp->portData->portInfo.NeighborNodeGUID)) {
			portInfo.LID = STL_LID_PERMISSIVE;
			portInfo.s1.LMC = 0;
			needSet = 1;
		}
	}

	if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) && (portp->index == 0)) {
		/* Switch port zero */
		desiredPortPacketFormats = portInfo.PortPacketFormats.Supported;
	} else {
		desiredPortPacketFormats = portInfo.PortPacketFormats.Supported & new_portp->portData->portInfo.PortPacketFormats.Supported;
	}

	// 8B|10B eligiblity check is skipped for ISLs
	if (!portp->portData->isIsl) {
		// Disable 8B or 10B packet formats on ports if port a) does not have a
		// matching PKey in its PKey table or b) LID exceeds the 8B/10B space.
		//
		// Note that these checks may not disable 8B/10B on switch port peers of an HFI
		// if that switch port:
		// 	a) does not have the same LID as its HFI peer (switch port LID = permissive
		// 	LID; LID security not in effect)
		// or
		// 	b) PKey enforcement is disabled on that switch port because the HFI peer is
		// 	not a member of at least one security-enabled VF
		// But checks will still work accurately on the HFI side
		int validUnicastLid = 0;
		if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
			validUnicastLid = is_valid_8B_10B_unicast_lid(portInfo.LID,
				portInfo.MultiCollectMask.MulticastMask);
		} else if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			validUnicastLid = (portInfo.LID == STL_LID_PERMISSIVE ||
				is_valid_8B_10B_unicast_lid(portInfo.LID,
				nodep->switchInfo.MultiCollectMask.MulticastMask));
		}

		int skipPkey = (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
			portp->index > 0 && !enforcePkey);
		if ((desiredPortPacketFormats & STL_PORT_PACKET_FORMAT_8B) &&
			(!sm_config.P_Key_8B || !validUnicastLid ||
			 (!skipPkey && !valid_full_portPKey(sm_config.P_Key_8B,
				nodep->nodeInfo.PartitionCap, portp, 0)))) {
			desiredPortPacketFormats &= ~STL_PORT_PACKET_FORMAT_8B;
		}

		if ((desiredPortPacketFormats & STL_PORT_PACKET_FORMAT_10B) &&
			(!sm_config.P_Key_10B || !validUnicastLid ||
			 (!skipPkey && !valid_full_portPKey(sm_config.P_Key_10B,
				nodep->nodeInfo.PartitionCap, portp, 1)))) {
			desiredPortPacketFormats &= ~STL_PORT_PACKET_FORMAT_10B;
		}
	}

	if (portInfo.PortPacketFormats.Enabled != desiredPortPacketFormats) {
		portInfo.PortPacketFormats.Enabled = desiredPortPacketFormats;
		needSet = 1;
	}

	// Set the fabric wide P_Key_8B value for all ports
	if (portInfo.P_Keys.P_Key_8B != sm_config.P_Key_8B) {
		if ((!sm_config.P_Key_8B) || (portInfo.PortPacketFormats.Enabled & STL_PORT_PACKET_FORMAT_8B)) {
			portInfo.P_Keys.P_Key_8B = sm_config.P_Key_8B;
			needSet = 1;
		}
	}

	// Set the fabric wide P_Key_10B value for all ports
	if (portInfo.P_Keys.P_Key_10B != sm_config.P_Key_10B) {
		if ((!sm_config.P_Key_10B) || (portInfo.PortPacketFormats.Enabled & STL_PORT_PACKET_FORMAT_10B)) {
			portInfo.P_Keys.P_Key_10B = sm_config.P_Key_10B;
			needSet = 1;
		}
	}

	if (needSet == 1 || sm_config.forceAttributeRewrite) {
		amod = portp->index;
		status = SM_Set_PortInfo(fd_topology, (1 << 24) | amod, addr, &portInfo,
			(portp->portData->portInfo.M_Key == 0) ? sm_config.mkey : portp->portData->portInfo.M_Key);

		if (status != VSTATUS_OK) {
			SM_INIT_PORT_PREP_CSM();
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_FABRIC_INIT_ERROR, &localNode, &remoteNode,
							"Failed to set portinfo for node");
			status = sm_popo_port_error(&sm_popo, topop, portp, status);
			IB_EXIT(__func__, status);
			return (status);
		}

		portp->portData->dirty.portInfo=0;

		/* Don't check preempt values, hardware can round off. */

		/* 
		 *  update the STL_PORT_INFO for SA queries. Set returns back actual settings (Get)
		 */
		portp->portData->portInfo = portInfo;
		if (sm_config.mkey && sm_config.mkey != portInfo.M_Key) {
			SM_INIT_PORT_PREP_CSM();
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_FABRIC_INIT_ERROR, &localNode, &remoteNode,
							"Node returned MKEY of [" FMT_U64 "] when MKEY of [" FMT_U64
							"] was requested", portInfo.M_Key, sm_config.mkey);
		}
#if defined(IB_STACK_OPENIB)
		if (smslChange) {
			IB_LOG_INFINI_INFO0("smsl refresh");
			status = ib_refresh_devport();
			if (status != VSTATUS_OK) {
				IB_LOG_ERRORRC("cannot refresh smsl rc:", status);
			}
		}
#endif
	}


	if (portp->state == IB_PORT_DOWN) {
		IB_EXIT(__func__, VSTATUS_OK);
		return (VSTATUS_OK);
	}

	IB_EXIT(__func__, status);
	return (status);
}

uint8_t
is_switch_on_list(SwitchList_t * swlist_head, Node_t * switchp)
{
	SwitchList_t *sw;
	for_switch_list_switches(swlist_head, sw) {
		if (sw->switchp == switchp)
			return 1;
	}

	return 0;
}

/* Returns list of switches connected to the node, whose initDone flag is not set*/
Status_t
sm_get_uninit_connected_switch_list(Topology_t * topop, Node_t * nodep,
									SwitchList_t ** swlist_head)
{
	SwitchList_t *peer_sw, *prev_sw;
	Node_t *neighborNodep;
	Port_t *portp;

	*swlist_head = NULL;
	prev_sw = NULL;

	for_all_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;
		if (portp->index == 0) {
			continue;
		} else {
			neighborNodep = sm_find_node(topop, portp->nodeno);
			if (!neighborNodep)
				continue;
			if (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				if (neighborNodep->initDone)
					continue;
				if (is_switch_on_list(*swlist_head, neighborNodep))	/* we've already added the
																	   switch to the list */
					continue;
				peer_sw = (SwitchList_t *) malloc(sizeof(SwitchList_t));
				if (peer_sw == NULL) {
					IB_LOG_ERROR0("can't malloc node !");
					return VSTATUS_NOMEM;
				}
				if (*swlist_head == NULL)
					*swlist_head = peer_sw;

				peer_sw->switchp = neighborNodep;
				peer_sw->parent_portno = portp->index;
				peer_sw->next = NULL;
				if (prev_sw)
					prev_sw->next = peer_sw;
				prev_sw = peer_sw;
			}
		}
	}

	return VSTATUS_OK;
}

void
sm_delete_switch_list(SwitchList_t * sw)
{
	SwitchList_t *tmp;

	while (sw) {
		tmp = sw;
		sw = sw->next;
		free(tmp);
	}

}

Status_t
sm_prep_switch_layer_LR_DR(Topology_t * topop, Node_t * nodep, SwitchList_t * swlist_head,
						   int rebalance)
{
	Port_t *swportp,*temportp;
	Status_t status = VSTATUS_OK;
	STL_LID parent_lid;
	int routing_needed = 0;
	SwitchList_t *sw;
	int retry_count;

	swportp = sm_get_port(nodep, 0);

	if (!sm_valid_port(swportp)) {
		IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						nodep->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}
	if (swportp->state == IB_PORT_DOWN)
		return VSTATUS_BAD;

	parent_lid = swportp->portData->lid;

	for_switch_list_switches(swlist_head, sw) {
		temportp = sm_get_port(sw->switchp, 0);
		if (temportp->state == IB_PORT_DOWN)
			return VSTATUS_BAD;
		if ((!bitset_test(&old_switchesInUse, sw->switchp->swIdx) || rebalance) ||
			(sw->switchp->old &&
			 (sw->switchp->initPorts.nset_m ||
			  !bitset_equal(&sw->switchp->activePorts, &sw->switchp->old->activePorts)) &&
			 topop->routingModule->funcs.handle_fabric_change(topop, sw->switchp->old, sw->switchp))) {
			// New switch or rebalance or redundant ISL which may be used for switch traffic lost.
			if (sm_config.sm_debug_routing && topology_passcount)
				IB_LOG_INFINI_INFO_FMT(__func__, "Routing needed for switch %s: "FMT_U64,
								sm_nodeDescString(sw->switchp), sw->switchp->nodeInfo.NodeGUID);

			routing_needed = 1;
		} else {
			routing_needed = 0;
		}

		// IB_LOG_INFINI_INFO_FMT(__func__, "switch "FMT_U64,
		// sw->switchp->nodeInfo.NodeGUID);
		retry_count = 0;
		do {
			if (!nodep->noLRDRSupport && (retry_count < (SmaEnableLRDR_MAX_RETRIES - 1))) {
				if (retry_count > 0) {
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "Retrying %s programming/initialization for the switch "
										   FMT_U64 " with mixed LR-DR SMPs",
										   sm_fwd_table_type_str(topop),
										   sw->switchp->nodeInfo.NodeGUID);
				}
				status =
					sm_initialize_switch_LR_DR(sm_topop, sw->switchp, parent_lid,
											   sw->parent_portno, routing_needed);
			} else {
				if (retry_count > 0){
					/* Last attempt, fall back to using pure DR */
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "%s programming/initialization for the switch "
										   FMT_U64 " failed with mix LR-DR SMPs,"
										   "retrying with pure DR SMPs",
										   sm_fwd_table_type_str(topop),
										   sw->switchp->nodeInfo.NodeGUID);
				}
				status =
					sm_initialize_switch_LR_DR(sm_topop, sw->switchp, 0, sw->parent_portno,
											   routing_needed);
				if (status == VSTATUS_OK && !nodep->noLRDRSupport) {
					/* Succeeded with pure DR SMPs */
					nodep->noLRDRSupport = 1;	// mark the parent switch as not supporting LR
												// DR
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "switch " FMT_U64
										   " does not support initializing switches connected to it with mixed LR-DR SMPs ",
										   nodep->nodeInfo.NodeGUID);
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "All nodes connected to switch " FMT_U64
										   " will be initialized with pure DR SMPs",
										   nodep->nodeInfo.NodeGUID);
				}
			}
			retry_count++;
		} while (status != VSTATUS_OK && retry_count < SmaEnableLRDR_MAX_RETRIES);

		if (status != VSTATUS_OK ) {
					IB_LOG_INFINI_INFO_FMT(__func__,
								   "Basic initialization failed for switch " FMT_U64,
								   sw->switchp->nodeInfo.NodeGUID);
			break;
		}
	}

	return status;

}

/* Programs switches in LR-DR wave by traversing the switches starting at
 * the head switch (switch next to SM) and going through the connected switches
 * in the order they were discovered.
 * Head switch must already have been programmed before this function is called.
 */
Status_t
sm_setup_switches_lrdr_wave_discovery_order(Topology_t * topop, int rebalance,
											int routing_needed)
{
	SwitchList_t *swlist_head = NULL, *sw;
	Status_t status = FERROR;

	if (sm_config.sm_debug_routing && topology_passcount)
		IB_LOG_INFINI_INFO_FMT(__func__, "rebalance %d routing_needed %d", rebalance, routing_needed);

	Node_t *nodep;
	/* Switches are added to the switch linked list as they are discovered. Traversing the list 
	   of switches will traverse them in the order they were discovered. */
	for_all_switch_nodes(sm_topop, nodep) {
		/* Get list of all switches connected to this switch, which we have not yet initialized */
		status = sm_get_uninit_connected_switch_list(sm_topop, nodep, &swlist_head);

		if (status != VSTATUS_OK) {
			IB_LOG_INFINI_INFO_FMT(__func__,
								   "failed to get list of child switches for switch " FMT_U64,
								   nodep->nodeInfo.NodeGUID);
			if (swlist_head) {
				sm_delete_switch_list(swlist_head);
			}
			return status;
		}

		if (!swlist_head)
			continue;
#if 0
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "child sw list for switch " FMT_U64, nodep->nodeInfo.NodeGUID);
		for_switch_list_switches(swlist_head, sw) {
			IB_LOG_INFINI_INFO_FMT(__func__,
								   "switch " FMT_U64, sw->switchp->nodeInfo.NodeGUID);
		}
#endif

		/* First initialize port 0 of switch and setup minimal LFTs if routing is required */
		if ((status =
			 sm_prep_switch_layer_LR_DR(sm_topop, nodep, swlist_head,
										rebalance)) != VSTATUS_OK) {
				IB_LOG_INFINI_INFO_FMT(__func__,
									   "Basic %s programming using LR/DR SMPs "
									   "failed for switches connected to " FMT_U64 " status %d",
									   sm_fwd_table_type_str(sm_topop),
									   nodep->nodeInfo.NodeGUID, status);
			sm_delete_switch_list(swlist_head);
			return status;
		}

		if (routing_needed) {
			/* Setup full LFT routing */
			if ((status =
				 sm_routing_route_switch_LR(sm_topop, swlist_head, rebalance)) != VSTATUS_OK) {
				IB_LOG_INFINI_INFO_FMT(__func__,
									   "Full %s programming failed for switches connected to "
									   FMT_U64 " status %d",
									   sm_fwd_table_type_str(sm_topop),
									   nodep->nodeInfo.NodeGUID, status);
				sm_delete_switch_list(swlist_head);
				return status;
			}
		}

		for_switch_list_switches(swlist_head, sw) {
			sw->switchp->initDone = 1;
		}
		sm_delete_switch_list(swlist_head);
	}

	return status;
}


/*
 * Uses LR/DR MADs to perform the initial setup of a switch. Calls sm_routing_prep_new_switch() and sm_initialize_port().
 */
Status_t
sm_initialize_switch_LR_DR(Topology_t * topop, Node_t * nodep, STL_LID parent_switch_lid,
						   uint8_t parent_portno, int routing_needed)
{
	Port_t *portp, *extPortp;
	Status_t status;
	uint8_t last_hop_path[2];
	uint8_t *path;
	int is_sm_appliance = 0;
	SmpAddr_t addr;

	portp = sm_get_port(nodep, 0);

	if (!sm_valid_port(portp)) {
		IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						nodep->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}
	if(portp->state == IB_PORT_DOWN)
		return VSTATUS_BAD;


	/* If parent_switch_lid is provided, then do a one hop DR from that switch. In case of ESM, 
	   when parent_switch_lid = sm_lid, then its just a one hop from the local switch, so just
	   do a one hop pure DR instead of sending a mixed LR-DR SMP to the local switch with
	   LRH:SLID = LRH:DLID (which S20 does not seem to handle well) */
	if (parent_switch_lid && (parent_switch_lid != sm_lid)) {
		// IB_LOG_INFINI_INFO_FMT(__func__, "init port %d with LR/DR for node guid
		// "FMT_U64,portp->index, nodep->nodeInfo.NodeGUID);
		/* more than one hop, do LR-DR mix */
		last_hop_path[0] = 1;
		last_hop_path[1] = parent_portno;
		path = last_hop_path;
		SMP_ADDR_SET_LRDR(&addr, path, sm_lid, parent_switch_lid);
	} else {
		path = PathToPort(nodep, portp);
		SMP_ADDR_SET_DR(&addr, path);
	}

	if (routing_needed) {
		if ((status =
			sm_routing_prep_new_switch(topop, nodep, &addr)) != VSTATUS_OK) {
			IB_LOG_INFINI_INFO_FMT(__func__,
								   "Basic %s programming failed for switch " FMT_U64,
								   sm_fwd_table_type_str(topop),
								   nodep->nodeInfo.NodeGUID);
			return status;
		}
	}
#if 0
	IB_LOG_INFINI_INFO_FMT(__func__,
						   "curr portno %d lid %d dlid %d node Guid " FMT_U64, portp->index,
						   portp->portData->lid, topop->dlid, nodep->nodeInfo.NodeGUID);
#endif

	if ((status = sm_initialize_port(topop, nodep, portp, &addr)) != VSTATUS_OK) {
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "Port 0 initialization failed for switch %s " FMT_U64,
							   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
	}

	// determine if this is an appliance node by checking sm_stl_appliance()
	// on neighboring physical ports.  This works even if not all neighbor
	// ports are known as there has to be at least one known port nodep was 
	// discovered through
	for_all_physical_ports(nodep, extPortp) {
		Port_t * neighPortp;
		if (!sm_valid_port(extPortp))
			continue;

		neighPortp = sm_find_port(topop, extPortp->nodeno, extPortp->portno);

		if (!sm_valid_port(neighPortp))
			continue;

		if (sm_stl_appliance(neighPortp->portData->portInfo.NeighborNodeGUID)) {
			is_sm_appliance = 1;
			break;
		}
	}

	// For appliance nodes (e.g. fabric_sim), need to disable SLID-checking on all
	// neighboring external switch ports before the SM attempts to use
	// LR or LR-DR SMPs to configure the fabric
	if (is_sm_appliance) {
		Port_t * neighPortp;
		Node_t * neighNodep;

		for_all_physical_ports(nodep, extPortp) {
			if (!sm_valid_port(extPortp) || extPortp->state <= IB_PORT_DOWN)
				continue;

			neighPortp = sm_find_neighbor_node_and_port(topop, extPortp, &neighNodep);

			if (!neighPortp || !neighNodep)
				continue;

			if (!sm_valid_port(neighPortp) || neighPortp->state <= IB_PORT_DOWN)
				continue;

			if (neighNodep->nodeInfo.NodeType == NI_TYPE_SWITCH && neighPortp->index > 0) {
				// Assume cached information is up-to date
				STL_PORT_INFO portInfoUpd = neighPortp->portData->portInfo;
				portInfoUpd.LID = STL_LID_PERMISSIVE;
				portInfoUpd.s1.LMC = 0;

				// Just do pure DR to keep things simple
				uint8_t * neighPath = PathToPort(neighNodep, neighPortp);
				uint32_t amod = (1 << 24) | neighPortp->index;

				SmpAddr_t addr = SMP_ADDR_CREATE_DR(neighPath);
				Status_t setStatus = SM_Set_PortInfo(fd_topology, amod, &addr, &portInfoUpd,
					(portInfoUpd.M_Key == 0) ? sm_config.mkey : portInfoUpd.M_Key);

				if (setStatus != VSTATUS_OK) {
					IB_LOG_WARN_FMT(__func__,
						"Failed to disable ingress SLID-checking on node %s"
						" guid "FMT_U64 " port index %d",
						sm_nodeDescString(neighNodep), neighNodep->nodeInfo.NodeGUID, neighPortp->index);
					continue;
				}

				neighPortp->portData->portInfo = portInfoUpd;
			}
		}
	}

	return status;
}


Status_t
sm_initialize_port_LR_DR(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	Port_t *neighborPortp, *swportp;
	Node_t *neighborNodep;
	uint8_t *path;
	Status_t status;
	int retry_count;
	Node_t *swnodep;
	uint8_t last_hop_path[3];
	STL_LID slid, dlid;

	path = PathToPort(nodep, portp);
	if (path[0] <= 1) {
		/* just one hop, do a DR */
		// IB_LOG_INFINI_INFO_FMT(__func__, "init port %d with DR for node
		// guid "FMT_U64, portp->index, nodep->nodeInfo.NodeGUID);
		SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
		status = sm_initialize_port(topop, nodep, portp, &addr);
	} else {
		// IB_LOG_INFINI_INFO_FMT(__func__, "init port %d with LR/DR for
		// node guid "FMT_U64,portp->index, nodep->nodeInfo.NodeGUID);
		/* more than one hop, do LR-DR mix */
		slid = sm_lid;
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index != 0) {
			swportp = sm_get_port(nodep, 0);
			if (!sm_valid_port(swportp)) {
				IB_LOG_WARN_FMT(__func__,
								"Failed to get Port 0 of Switch " FMT_U64,
								nodep->nodeInfo.NodeGUID);
				return VSTATUS_BAD;
			}
			dlid = swportp->portData->lid;
			swnodep = nodep;
		} else {
			neighborPortp = sm_find_port(topop, portp->nodeno, portp->portno);
			if (!sm_valid_port(neighborPortp)) {
				IB_LOG_WARN_FMT(__func__,
								"Failed to get peer port for port %d of Node " FMT_U64,
								portp->index, nodep->nodeInfo.NodeGUID);
				return VSTATUS_BAD;
			}
			neighborNodep = sm_find_port_node(topop, neighborPortp);
			if (!neighborNodep) {
				IB_LOG_WARN_FMT(__func__,
								"Failed to get peer node for port %d of Node " FMT_U64,
								portp->index, nodep->nodeInfo.NodeGUID);
				return VSTATUS_BAD;
			}
			swportp = sm_get_port(neighborNodep, 0);
			if (!sm_valid_port(swportp)) {
				IB_LOG_WARN_FMT(__func__,
								"Failed to get Port 0 of Switch " FMT_U64,
								neighborNodep->nodeInfo.NodeGUID);
				return VSTATUS_BAD;
			}
			dlid = swportp->portData->lid;
			swnodep = neighborNodep;
		}
#if 0
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "curr portno %d lid %d dlid %d node Guid " FMT_U64, portp->index,
							   portp->portData->lid, tp->dlid, nodep->nodeInfo.NodeGUID);
#endif

		retry_count = 0;
		do {
			if (!swnodep->noLRDRSupport && (retry_count < (SmaEnableLRDR_MAX_RETRIES - 1))) {
				if (retry_count > 0) {
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "Retrying initialization of port %d of node " FMT_U64
										   " with mixed LR-DR SMPs", portp->index,
										   nodep->nodeInfo.NodeGUID);
				}
				path = sm_set_last_hop_path(nodep, portp, last_hop_path);
				SmpAddr_t addr = SMP_ADDR_CREATE_LRDR(path, slid, dlid);
				status = sm_initialize_port(topop, nodep, portp, &addr);
			} else {
				if (retry_count > 0) {
					/* Last attempt, fall back to using pure DR */
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "Port initialization of port %d of node " FMT_U64
										   " with mixed LR-DR SMPs failed, retrying with pure DR SMPs",
										   portp->index, nodep->nodeInfo.NodeGUID);
				}
				path = PathToPort(nodep, portp);
				SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
				status = sm_initialize_port(topop, nodep, portp, &addr);
				if (status == VSTATUS_OK && !swnodep->noLRDRSupport) {
					/* Succeeded with pure DR SMP */
					swnodep->noLRDRSupport = 1;	// mark the parent switch as not supporting LR
												// DR
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "switch " FMT_U64
										   " does not support initialization of its connected ports with mixed LR-DR SMPs.",
										   swnodep->nodeInfo.NodeGUID);
					IB_LOG_INFINI_INFO_FMT(__func__,
										   "All nodes connected to switch " FMT_U64
										   " will be initialized with pure DR SMPs",
										   swnodep->nodeInfo.NodeGUID);
				}
			}
			retry_count++;
		} while (status != VSTATUS_OK && retry_count < SmaEnableLRDR_MAX_RETRIES);

	}

	return status;
}

// -------------------------------------------------------------------------- //

Status_t
sm_get_CapabilityMask(IBhandle_t fd, uint8_t port, uint32_t * value)
{
	uint8_t path[8];
	Status_t status;
	STL_PORT_INFO portInfo;

	IB_ENTER(__func__, port, value, 0, 0);

	memset((void *) path, 0, 8);

//
//  Get the capabilityMask in the STL_PORT_INFO on our port.
//
	status = SM_Get_PortInfo(fd, (1 << 24) | port, path, &portInfo);
	if (status != VSTATUS_OK) {
		IB_EXIT(__func__, status);
		return (status);
	}

	*value = portInfo.CapabilityMask.AsReg32;

	IB_EXIT(__func__, VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
sm_set_CapabilityMask(IBhandle_t fd, uint8_t port, uint32_t value)
{
	uint8_t path[8];
	Status_t status;
	STL_PORT_INFO portInfo;

	IB_ENTER(__func__, port, value, 0, 0);

	memset((void *) path, 0, 8);

//
//  Set the capabilityMask in the STL_PORT_INFO on our port.
//
	status = SM_Get_PortInfo(fd, (1 << 24) | port, path, &portInfo);
	if (status != VSTATUS_OK) {
		IB_LOG_INFINI_INFORC("can't get STL_PORT_INFO rc:", status);
		IB_EXIT(__func__, status);
		return (status);
	}

	portInfo.CapabilityMask.AsReg32 = value;
	portInfo.LinkWidth.Enabled = STL_LINK_WIDTH_NOP;
	portInfo.PortStates.s.PortState = IB_PORT_NOP;
	portInfo.PortStates.s.PortPhysicalState = IB_PORT_PHYS_NOP;
	portInfo.LinkSpeed.Enabled = STL_LINK_SPEED_NOP;
	portInfo.s4.OperationalVL = 0;

	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
	status = SM_Set_PortInfo(fd, (1 << 24) | port, &addr, &portInfo,
		(portInfo.M_Key == 0) ? sm_config.mkey : portInfo.M_Key);
	if (status != VSTATUS_OK) {
		IB_LOG_INFINI_INFORC("can't set STL_PORT_INFO rc:", status);
		IB_EXIT(__func__, status);
		return (status);
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
sm_update_cableinfo(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	IB_ENTER(__func__, topop, nodep, portp, 0);

	Status_t status = VSTATUS_NOSUPPORT;

	STL_LID dlid;

	if (!sm_Port_t_IsCableInfoSupported(portp))
		goto fail;

	if (portp->state < IB_PORT_ARMED) {
		status = VSTATUS_BAD;
		goto fail;
	}

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t *swportp;
		swportp = sm_get_port(nodep, 0);

		if (!sm_valid_port(swportp)) {
			IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
							nodep->nodeInfo.NodeGUID);
			status = VSTATUS_BAD;
			goto fail;
		}
		dlid = swportp->portData->lid;
	} else {
		dlid = portp->portData->lid;
	}

	const int CiStartSeg = 2;
	const int CiSegCount = 2;

	if (!portp->portData->cableInfo) {
		portp->portData->cableInfo = sm_CableInfo_init();

		if (!portp->portData->cableInfo) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate memory for CableInfo");
			status = VSTATUS_BAD;
			goto fail;
		}
	}

	STL_CABLE_INFO *outCi = (STL_CABLE_INFO *) portp->portData->cableInfo->buffer;

	uint32_t madStatus;
	status = SM_Get_CableInfo_LR(fd_topology, portp->index,
									 CiStartSeg, CiSegCount, sm_lid, dlid, outCi, &madStatus);

	if (status != VSTATUS_OK) {
		if (madStatus == MAD_STATUS_UNSUPPORTED_METHOD || madStatus == MAD_STATUS_INVALID_ATTRIB) {
			status = VSTATUS_NOSUPPORT;
		} else {
			IB_LOG_WARN_FMT(__func__,
							"Cannot get CableInfo for node %s nodeGuid " FMT_U64
							" port %d status=%d madStatus %d", sm_nodeDescString(nodep),
							nodep->nodeInfo.NodeGUID, portp->index, status, madStatus);
			status = sm_popo_port_error(&sm_popo, topop, portp, status);
		}
		goto fail;
	}

	IB_EXIT(__func__, status);

	return status;

  fail:

	if (portp->portData->cableInfo) {
		sm_CableInfo_free(portp->portData->cableInfo);
		portp->portData->cableInfo = NULL;
	}

	IB_EXIT(__func__, status);
	return status;
}

/**
	sm_get_node_port_states will send Get(PortStateInfo) to nodep.
	If path is provided DR packets will be used and portp can be NULL.
	If path is NULL, LR packets will be used and portp must contain a valid port with valid DLID.

	@param psi On success, it is set to a valid PortStateInfo buffer which
		must be freed. Buffer is length 1 for HFIs and NumPorts+1 for switches.
*/
Status_t
sm_get_node_port_states(Topology_t * topop, Node_t * nodep, Port_t * portp, uint8_t* path, STL_PORT_STATE_INFO ** psi) 
{
	Status_t status = VSTATUS_OK;
	uint32_t numPackets = 1;
	uint32_t numPSIs = 1;
	uint32_t amod;
	STL_LID dlid=0;
	int i;
	STL_PORT_STATE_INFO *portStateInfo = NULL;
	uint32_t maxPSIsPerPacket = STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_PORT_STATE_INFO);

	IB_ENTER(__func__, topop, nodep, portp, 0);

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {

		numPSIs = nodep->nodeInfo.NumPorts + 1;
		// Calculate the number of packets to send for this switch
		if(numPSIs > maxPSIsPerPacket) {
			numPackets = numPSIs / maxPSIsPerPacket;

			if((numPSIs % maxPSIsPerPacket) != 0)
				numPackets += 1;
		}
	} else {
		numPackets = 1;
		numPSIs = 1;
	}

	if (psi) *psi = 0;

	// Allocate and clear space for all the port states
	status = vs_pool_alloc(&sm_pool, sizeof(STL_PORT_STATE_INFO) * numPSIs, (void*) &portStateInfo);
	if(status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "can't malloc portStateInfo status=%d", status);
		goto done;
	}
	memset(portStateInfo, 0, sizeof(STL_PORT_STATE_INFO) * numPSIs);

	if (! path) {
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			Port_t *swportp;
			swportp = sm_get_port(nodep, 0);
			if (!sm_valid_port(swportp)) {
				IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
								nodep->nodeInfo.NodeGUID);
				status = VSTATUS_BAD;
				goto done;
			}
			dlid = swportp->portData->lid;
		} else {
			dlid = portp->portData->lid;
		}
	}

	// Send out the PortStateInfo packets
	for(i = 0; i < numPackets; i++) {
		if(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if(i == numPackets - 1)
				amod = ((numPSIs % maxPSIsPerPacket) << 24) | (i * maxPSIsPerPacket);
			else
				amod = (maxPSIsPerPacket << 24) | (i * maxPSIsPerPacket);
		} else {
			// Node is an HFI, so we're doing this only for the port given as an arg
			amod = (numPSIs % maxPSIsPerPacket) << 24;
		}

		if(path) {
			status = SM_Get_PortStateInfo(fd_topology, amod, path, (portStateInfo + (maxPSIsPerPacket * i)));
		} else {
			status = SM_Get_PortStateInfo_LR(fd_topology, amod, sm_lid, dlid, (portStateInfo + (maxPSIsPerPacket * i)));
		}

		if(status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
							"Cannot get PORT_STATE_INFO for node %s nodeGuid " FMT_U64 " status=%d",
							sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
			goto done;
		}
	}

done:
	if (status != VSTATUS_OK && portStateInfo) {
		vs_pool_free(&sm_pool, portStateInfo);
		portStateInfo = NULL;
	}

	if (psi) *psi = portStateInfo;

	IB_EXIT(__func__, status);
	return status;
}

/**
	Sends a Set(PortStateInfo) to take portStates from srcState to dstState.

	If no ports are in the srcState state or the Set fails, *psi == NULL.

	On success, *psi points to the response buffer.  Buffer is length 1 for
	HFIs and NumPorts+1 for switches.
*/
Status_t
sm_set_node_port_states(Topology_t * topop, Node_t * nodep, Port_t * portp, uint8_t* path, uint32_t srcState, uint32_t dstState, OPTIONAL OUT STL_PORT_STATE_INFO ** psi)
{
	Status_t status = VSTATUS_OK;
	uint32_t numPackets = 1;
	uint32_t numPSIs = 1;
	uint32_t amod;
	int i;
	STL_PORT_STATE_INFO *portStateInfo = NULL;
	STL_LID dlid;
	uint32_t maxPSIsPerPacket = (path ? STL_MAX_PAYLOAD_SMP_DR : STL_MAX_PAYLOAD_SMP_LR) / sizeof(STL_PORT_STATE_INFO);

	IB_ENTER(__func__, topop, nodep, 0, 0);

	// Get the dlid of the node we need to talk to
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t *swportp;
		swportp = sm_get_port(nodep, 0);
		if (!sm_valid_port(swportp)) {
			IB_LOG_WARN_FMT(__func__,
				"Failed to get port 0 of node %s nodeGuid " FMT_U64,
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
			goto done;
		}
		dlid = swportp->portData->lid;

		numPSIs = nodep->nodeInfo.NumPorts + 1;
		// Calculate the number of packets to send for this switch
		if(numPSIs > maxPSIsPerPacket) {
			numPackets = numPSIs / maxPSIsPerPacket;
			if((numPSIs % maxPSIsPerPacket) != 0)
				numPackets += 1;
		}
	} else {
		dlid = portp->portData->lid;
	}

	// Bring all armed ports to active
	for_all_ports(nodep, portp) {
		if (portp->state != srcState)
			continue;

		if(portp->portData->neighborQuarantined)
			continue;
	
		if (!portStateInfo) {
			// at least one valid port: allocate buffer
			size_t len = sizeof(STL_PORT_STATE_INFO) * numPSIs;
			status = vs_pool_alloc(&sm_pool, len, (void *)&portStateInfo);
			if(status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__, "can't malloc portStateInfo: status=%u", status);
				goto done;
			}
			memset(portStateInfo, 0, len);
		}

		portStateInfo[portp->index].PortStates.s.PortState = dstState;
	}

	// no buffer?  nothing to do
	if (!portStateInfo)
		goto done;

	// Send out the PortStateInfo packets
	for(i = 0; i < numPackets; i++) {
		if(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if(i == numPackets - 1)
				amod = ((numPSIs % maxPSIsPerPacket) << 24) | (i * maxPSIsPerPacket);
			else
				amod = (maxPSIsPerPacket << 24) | (i * maxPSIsPerPacket);
		} else {
			// Node is an HFI, so we're doing this only for the port given as an arg
			amod = ((numPSIs % maxPSIsPerPacket) << 24) | portp->index;
		}

		if(path) {
			status = SM_Set_PortStateInfo(fd_topology, amod, path,
				portStateInfo + maxPSIsPerPacket * i, sm_config.mkey);
		} else {
			status = SM_Set_PortStateInfo_LR(fd_topology, amod, sm_lid, dlid,
				portStateInfo + maxPSIsPerPacket * i, sm_config.mkey);
		}

		if(status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				"Cannot set PORT_STATE_INFO for node %s nodeGuid "FMT_U64": status=%u",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
			goto done;
		}
	}

done:
	if (status != VSTATUS_OK && portStateInfo) {
		vs_pool_free(&sm_pool, portStateInfo);
		portStateInfo = NULL;
	}

	if (psi) *psi = portStateInfo;

	IB_EXIT(__func__, status);
	return status;
}

Status_t
sm_get_node_pkeys(Topology_t *topop, Node_t *nodep)
{
	uint8_t buffer[STL_MAX_PAYLOAD_SMP_DR] = {0};
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	STL_AGGREGATE *lastSeg;
	Port_t *portp;
	size_t pkeyBlkCnt;
	uint8_t pkeyCap;
	bitset_t outstanding;
	bool_t aggrFailure = 0;
	Status_t status = VSTATUS_OK;

	bitset_init(&sm_pool, &outstanding, MAX_STL_PORTS + 1);

	for_all_ports(nodep, portp) {
		bool_t skipPort = 0;

		if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN)
			skipPort = 1;

		if ( (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0) ||
			 (nodep->nodeInfo.NodeType == NI_TYPE_CA) ) {
			pkeyCap = nodep->nodeInfo.PartitionCap;
		} else {
			/* for switch external ports, use switchInfo->partCap */
			pkeyCap = nodep->switchInfo.PartitionEnforcementCap;
		}

		pkeyBlkCnt = (pkeyCap + NUM_PKEY_ELEMENTS_BLOCK - 1)/ NUM_PKEY_ELEMENTS_BLOCK;

		if (!skipPort)
			memset(portp->portData->pPKey, 0, sizeof(PKey_t) * SM_PKEYS);

		if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
			!(portp->index == 0 && nodep->nodeInfo.PartitionCap) &&
			!(portp->index > 0 && nodep->switchInfo.PartitionEnforcementCap))
			skipPort = 1;

		Node_t *cache_nodep = NULL;
		Port_t *cache_portp = NULL;
		if(!skipPort && sm_find_cached_node_port(nodep, portp, &cache_nodep, &cache_portp) && sm_valid_port(cache_portp)) {
			memcpy(portp->portData->pPKey, cache_portp->portData->pPKey, sizeof(portp->portData->pPKey));
			portp->portData->current.pkeys = 1;

			// Keep going in case we need to flush out Aggregates.
			skipPort = 1;
		}

		// If we decided to skip this port, just process the next one UNLESS we're doing aggregate processing.
		// We may need to send out a completed aggregate stil.
		if (skipPort && !sm_config.use_aggregates)
			continue;

		if (sm_config.use_aggregates && !aggrFailure) {
			const size_t blockSize = sizeof(STL_AGGREGATE) + 8 * ((sizeof(STL_PARTITION_TABLE) * pkeyBlkCnt + 7)/8);
			const size_t portsPerAggr = (STL_MAX_PAYLOAD_SMP_DR / blockSize);

			if (!skipPort) {
				aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PART_TABLE;
				aggrHdr->Result.s.Error = 0;
				aggrHdr->Result.s.Reserved = 0;

				aggrHdr->Result.s.RequestLength = ((sizeof(STL_PARTITION_TABLE) * pkeyBlkCnt) + 7)/ 8;
				aggrHdr->AttributeModifier = ((0xff & pkeyBlkCnt)<<24) |
											((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? portp->index : 0)<<16);

				aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

				bitset_set(&outstanding, portp->index);
			}

			if ((portp == sm_get_port(nodep, PORT_A1(nodep)) && bitset_nset(&outstanding) != 0) ||
				bitset_nset(&outstanding) == portsPerAggr) {
				uint32_t madStatus = 0;
				uint8_t cport = bitset_find_first_one(&outstanding);

				lastSeg = NULL;
				status = SM_Get_Aggregate_DR(fd_topology, (STL_AGGREGATE*)buffer, aggrHdr, nodep->path, &lastSeg, &madStatus);
				status = sm_popo_port_error(&sm_popo, topop,
					nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? sm_get_port(nodep, 0) : portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT)
					goto exit;

				if (lastSeg) {
					for (aggrHdr = (STL_AGGREGATE*)buffer; aggrHdr < lastSeg; aggrHdr = STL_AGGREGATE_NEXT(aggrHdr)) {
						Port_t *current = sm_get_port(nodep, cport);
						uint8_t i;

						if (!sm_valid_port(current)) {
							// should never happen
							bitset_clear(&outstanding, cport);
							cport = bitset_find_next_one(&outstanding, cport + 1);
							continue;
						}

						if (aggrHdr->Result.s.Error) {
							// Stop processing, rest of this Aggregate is bunk. Reset portp pointer so we can build
							// a new aggregate excluding the failed port.
							bitset_clear(&outstanding, cport);
							portp = current;
							portp->portData->current.pkeys = 0;
							IB_LOG_WARN_FMT(__func__, "Failed to get Partition Table for node %s guid "FMT_U64" node index"
											" %d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
											nodep->index, cport);
							break;
						}

						memcpy(current->portData->pPKey, aggrHdr->Data, sizeof(STL_PARTITION_TABLE) * pkeyBlkCnt);

						for (i = 0; i < pkeyBlkCnt; ++i)
							BSWAP_STL_PARTITION_TABLE((STL_PARTITION_TABLE*)&(current->portData->pPKey[i * NUM_PKEY_ELEMENTS_BLOCK]));

						current->portData->current.pkeys = 1;
						bitset_clear(&outstanding, cport);

						cport = bitset_find_next_one(&outstanding, cport + 1);
					}

					aggrHdr = (STL_AGGREGATE*)buffer;
				} else {
					// The Aggregate itself failed.
					portp = sm_get_port(nodep, cport);
					IB_LOG_WARN_FMT(__func__, "Get(Aggregate) failed for node %s guid "FMT_U64".",
									sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
					aggrFailure = 1;
				}
			}
		}

		// Trouble with Aggregates or disabled - try manually.
		if (!skipPort && portp && (aggrFailure || !sm_config.use_aggregates)) {
			uint32_t amod = ((0xff & pkeyBlkCnt)<<24) |
				((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? portp->index : 0)<<16);

			SmpAddr_t addr = SMP_ADDR_CREATE_DR(nodep->path);
			status = SM_Get_PKeyTable(fd_topology, amod, &addr, (STL_PARTITION_TABLE*)portp->portData->pPKey);
			if (status != VSTATUS_OK) {
				portp->portData->current.pkeys = 0;
				IB_LOG_WARN_FMT(__func__, "Failed to get Partition Table for node %s guid "FMT_U64" node index"
								" %d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								nodep->index, portp->index);
				if ((status = sm_popo_port_error(&sm_popo, topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
					goto exit;
			} else
				portp->portData->current.pkeys = 1;
		}
	}

exit:
	bitset_free(&outstanding);

	return status;
}

static STL_LID
sm_clear_lid(Port_t * portp)
{
	cl_map_item_t *it;

	if (!sm_valid_port(portp) || portp->portData->guid == 0)
		return STL_LID_RESERVED;

	it = cl_qmap_get(sm_GuidToLidMap, portp->portData->guid);
	if (it != cl_qmap_end(sm_GuidToLidMap)) {
		LidMap_t *lm = (LidMap_t*) cl_qmap_obj((cl_map_obj_t*)it);
		cl_qmap_remove_item(sm_GuidToLidMap, it);

		if (lm->missing) {
			sm_lidmap_remove_missing(lm->missing);
			vs_pool_free(&sm_pool, lm->missing);
			lm->missing = NULL;
		}

		STL_LID i = (lm - lidmap);
		for (; i <= STL_GET_UNICAST_LID_MAX(); ++i) {
			if (lidmap[i].guid != portp->portData->guid)
				break;

			if (sm_config.sm_debug_lid_assign) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					"clear lid for port " FMT_U64 ": lid 0x%x",
					lidmap[i].guid, i);
			}

			lidmap[i].guid = 0;
			lidmap[i].pass = 0;
			lidmap[i].newNodep = NULL;
			lidmap[i].newPortp = NULL;
			memset(&lidmap[i].mapObj, 0, sizeof(lidmap[i].mapObj));
			lidmap[i].missing = NULL;
		}
	}

	portp->portData->lid = STL_LID_RESERVED;
	portp->portData->lmc = 0;
	return STL_LID_RESERVED;
}

//
// Physically disables the specified port.
//
Status_t
sm_disable_port(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	Status_t status;
	STL_PORT_INFO portInfo;
	uint8_t *path;
	uint32_t amod = 0;

	IB_ENTER(__func__, nodep, portp, 0, 0);

	if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
		IB_LOG_WARN("disable failed: port is down or invalid", 0);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	path = PathToPort(nodep, portp);

	portInfo = portp->portData->portInfo;
	portInfo.PortStates.s.PortState = IB_PORT_NOP;
	portInfo.PortStates.s.PortPhysicalState = IB_PORT_PHYS_DISABLED;

	amod = portp->index;
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
	status = SM_Set_PortInfo(fd_topology, (1 << 24) | amod, &addr, &portInfo,
		(portInfo.M_Key == 0) ? sm_config.mkey : portInfo.M_Key);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
						"Failed to set portinfo for node %s guid " FMT_U64
						" node index %d port index %d", sm_nodeDescString(nodep),
						nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
		IB_EXIT(__func__, status);
		return (status);
	}

	portp->portData->portInfo = portInfo;
	if (sm_config.mkey && sm_config.mkey != portInfo.M_Key) {
		IB_LOG_WARN_FMT(__func__,
						"Node %s [" FMT_U64 "] port[%d] returned MKEY[" FMT_U64 "] when MKEY["
						FMT_U64 "] was requested!", sm_nodeDescString(nodep),
						nodep->nodeInfo.NodeGUID, portp->index, portInfo.M_Key, sm_config.mkey);
	}

	sm_mark_link_down(topop, portp);

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}



//
//sm_bounce_all_switch_ports: 
//This function will request the PSI from a switch, and if it finds
// and active ports it will then send PSI to bounce them. If path is not null
// DR MADs will be used, otherwise the dlid in portp will be used.
//
Status_t 
sm_bounce_all_switch_ports(Topology_t *topop, Node_t *nodep, Port_t *portp, uint8_t *path) {

	Status_t status;
	int needBounce = 0;
	int i;
	STL_PORT_STATE_INFO *portStateInfo = NULL;


	IB_ENTER(__func__, nodep, portp, path, 0);


	if((portp == NULL && path == NULL) || nodep == NULL)
		return VSTATUS_BAD;

	if(nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
		return VSTATUS_BAD;

	if((status = sm_get_node_port_states(topop, nodep, portp, path, &portStateInfo)) != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Unable to get PortStateInfo information for switch %s. Status: %d",
										sm_nodeDescString(nodep), status);
		goto bail;
	}

	for(i=0; i < nodep->nodeInfo.NumPorts + 1; i++) {
		uint8_t portState = portStateInfo[i].PortStates.s.PortState;
		memset(&portStateInfo[i], 0, sizeof(STL_PORT_STATE_INFO));
		if(i != 0 && portState > IB_PORT_INIT) {
			needBounce = 1;
			portStateInfo[i].PortStates.s.PortPhysicalState = IB_PORT_PHYS_POLLING;
		}
	}
	
	if(needBounce) {
		uint32_t numPackets = 1;
		uint32_t numPSIs;
		int32_t amod;
		STL_LID dlid=0;
		int32_t maxPSIsPerPacket = STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_PORT_STATE_INFO);

		numPSIs = nodep->nodeInfo.NumPorts + 1;
		// Calculate the number of packets to send for this switch
		if(numPSIs > maxPSIsPerPacket) {
			numPackets = numPSIs / maxPSIsPerPacket;

			if((numPSIs % maxPSIsPerPacket) != 0)
				numPackets += 1;
		}

		if (! path) {
			Port_t *swportp;
			swportp = sm_get_port(nodep, 0);
			if (!sm_valid_port(swportp)) {
				IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						nodep->nodeInfo.NodeGUID);
				status =  VSTATUS_BAD;
				goto bail;
			}
			dlid = swportp->portData->lid;	
		}

		// Send out the PortStateInfo packets
		for(i = 0; i < numPackets; i++) {
			if(i == numPackets - 1)
				amod = ((numPSIs % maxPSIsPerPacket) << 24) | (i * maxPSIsPerPacket);
			else
				amod = (maxPSIsPerPacket << 24) | (i * maxPSIsPerPacket);
	
			if(path) {//use DR mad
				status = SM_Set_PortStateInfo(fd_topology, amod, path, (portStateInfo + (maxPSIsPerPacket * i)), sm_config.mkey);
			}else {//use LR Mad
				status = SM_Set_PortStateInfo_LR(fd_topology, amod, sm_lid, dlid, (portStateInfo + (maxPSIsPerPacket * i)), sm_config.mkey);
			}

			if(status != VSTATUS_OK) {
				IB_LOG_WARN_FMT(__func__,
								"Cannot set PORT_STATE_INFO for node %s nodeGuid " FMT_U64 " status=%d",
								sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
				IB_EXIT(__func__, status);
				goto bail;
			}
		}
	}


bail:

	if(portStateInfo != NULL) 
		vs_pool_free(&sm_pool, portStateInfo);	

	IB_EXIT(__func__, status);
	return status;

}


/* For STL gen 1 it is advised to only bounce a link from the switch side
 *	sm_bounce_link will always bounce from switch side and return error
 * if neither side of link is switch.
 */


Status_t
sm_bounce_link(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	Port_t *swportp;
	Node_t *swnodep;
	Status_t retval;
	IB_ENTER(__func__, nodep, portp, 0, 0);

	if(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		swportp = portp;
		swnodep = nodep;
	} else {
		/* 
		 *  find the other side of the cable.
		 */
		swportp = sm_find_port(topop, portp->nodeno, portp->portno);
		if(swportp == NULL) {
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		
		swnodep = swportp->portData->nodePtr;
		if((!swnodep) || (swnodep->nodeInfo.NodeType == NI_TYPE_CA)) {
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;	
		}
	}
	retval = sm_bounce_port(topop, swnodep, swportp);

	IB_EXIT(__func__, retval);
	return(retval);
}

Status_t
sm_bounce_port(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	Status_t status;
	STL_PORT_INFO portInfo;
	uint8_t *path;
	uint32_t amod = 0;

	IB_ENTER(__func__, nodep, portp, 0, 0);

	if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
		IB_LOG_WARN("enable failed: port invalid or down", 0);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	if(nodep->nodeInfo.NodeType == NI_TYPE_CA){
		IB_LOG_WARN("Ignoring attempt to bounce HFI Port.",0);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	path = PathToPort(nodep, portp);

	portInfo = portp->portData->portInfo;
	portInfo.PortStates.s.PortState = IB_PORT_NOP;
	portInfo.PortStates.s.PortPhysicalState = IB_PORT_PHYS_POLLING;

	amod = portp->index;
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
	status = SM_Set_PortInfo(fd_topology, (1 << 24) | amod, &addr, &portInfo,
		(portInfo.M_Key == 0) ? sm_config.mkey : portInfo.M_Key);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
						"Failed to set portinfo for node %s guid " FMT_U64
						" node index %d port index %d", nodep->nodeDesc.NodeString,
						nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
		IB_EXIT(__func__, status);
		return (status);
	}

	portp->portData->portInfo = portInfo;
	if (sm_config.mkey && sm_config.mkey != portInfo.M_Key) {
		IB_LOG_WARN_FMT(__func__,
						"Node %s [" FMT_U64 "] port[%d] returned MKEY[" FMT_U64 "] when MKEY["
						FMT_U64 "] was requested!", nodep->nodeDesc.NodeString,
						nodep->nodeInfo.NodeGUID, portp->index, portInfo.M_Key, sm_config.mkey);
	}

	sm_mark_link_down(topop, portp);

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

/* Instead of always searching from the beginning of lid space, sm_find_lid_quick sets and uses
 * hints to remember from where it should start searching for a lid for a given lmc - sm_lmc_0
 * in case of no lmc, sm_lmc_e0 and sm_lmc for LMC of enhanced switch port 0 and LMC for FI ports
 * respectively.
 */
static STL_LID
sm_find_lid_quick(int lid_count)
{
	STL_LID lid1 = STL_LID_RESERVED, lid2, endLid = STL_GET_UNICAST_LID_MAX() - lid_count + 1, freeSpace;
	STL_LID freeLid, nextLid2;

	/* If freeLid hint is -1, then we have exhausted the lid space. For other cases, we should
	   start our search for free lid from the lid number indicated by the hint. */

	if (lid_count == 1) {
		if (sm_lmc_0_freeLid_hint == STL_LID_PERMISSIVE) {
			return STL_LID_RESERVED;
		}

		lid1 = sm_lmc_0_freeLid_hint;
	} else if ((lid_count == (1 << sm_config.lmc))) {
		if (sm_lmc_freeLid_hint == STL_LID_PERMISSIVE) {
			return STL_LID_RESERVED;
		}

		lid1 = sm_lmc_freeLid_hint;
	} else if (lid_count == (1 << sm_config.lmc_e0)) {
		if (sm_lmc_e0_freeLid_hint == STL_LID_PERMISSIVE) {
			return STL_LID_RESERVED;
		}
		lid1 = sm_lmc_e0_freeLid_hint;
	}

	/* Reset the hint depending on which lid number we are using, so that hint can be updated */
	if (lid1 == sm_lmc_0_freeLid_hint)
		sm_lmc_0_freeLid_hint = 0;
	if (lid1 == sm_lmc_e0_freeLid_hint)
		sm_lmc_e0_freeLid_hint = 0;
	if (lid1 == sm_lmc_freeLid_hint)
		sm_lmc_freeLid_hint = 0;

	freeLid = 0;
	for (; lid1 <= endLid; lid1 += lid_count) {
		nextLid2 = lid1 + lid_count;
		if (lidmap[lid1].guid == 0) {
			for (lid2 = lid1 + 1; lid2 < nextLid2; ++lid2) {
				if (lidmap[lid2].guid != 0) {
					/* We didn't find the required space, but note the space we did find for
					   the next time we need to search. */
					freeSpace = lid2 - lid1;

					if ((freeSpace >= 1) && !sm_lmc_0_freeLid_hint)
						sm_lmc_0_freeLid_hint = lid1;

					if ((freeSpace >= (1 << sm_config.lmc_e0)) && !sm_lmc_e0_freeLid_hint)
						sm_lmc_e0_freeLid_hint = lid1;

					if ((freeSpace >= (1 << sm_config.lmc)) && !sm_lmc_freeLid_hint)
						sm_lmc_freeLid_hint = lid1;

					break;
				}
			}

			if (lid2 >= nextLid2) {
				// Found free lid
				freeLid = lid1;
				break;
			}
		}
	}

	if (!freeLid) {
		/* We could not find the required lid space */
		/* Set the hint to STL_LID_PERMISSIVE (0xffffffff), to indicate we have exhausted the lid space */
		if (!sm_lmc_0_freeLid_hint)
			sm_lmc_0_freeLid_hint = STL_LID_PERMISSIVE;
		if (!sm_lmc_e0_freeLid_hint)
			sm_lmc_e0_freeLid_hint = STL_LID_PERMISSIVE;
		if (!sm_lmc_freeLid_hint)
			sm_lmc_freeLid_hint = STL_LID_PERMISSIVE;
		return STL_LID_RESERVED;
	}

	/* Remember where we need to start our search for the next time */
	lid2 = lid1 + lid_count;

	if (!sm_lmc_0_freeLid_hint) {
		if (lid2 <= endLid)
			sm_lmc_0_freeLid_hint = lid2;
		else
			sm_lmc_0_freeLid_hint = STL_LID_PERMISSIVE;
	}

	/* LMC is in use, so make sure the lid hint is aligned to the required LMC values */

	if (!sm_lmc_e0_freeLid_hint) {
		if (sm_config.lmc_e0) {
			lid1 = 1 << sm_config.lmc_e0;
			if (lid2 % lid1) {
				lid2 = ((lid2 / lid1) + 1) * lid1;
			}
		}
		if (lid2 <= endLid)
			sm_lmc_e0_freeLid_hint = lid2;
		else
			sm_lmc_e0_freeLid_hint = STL_LID_PERMISSIVE;
	}

	if (!sm_lmc_freeLid_hint) {
		if (sm_config.lmc) {
			lid1 = 1 << sm_config.lmc;
			if (lid2 % lid1) {
				lid2 = ((lid2 / lid1) + 1) * lid1;
			}
		}
		if (lid2 <= endLid)
			sm_lmc_freeLid_hint = lid2;
		else
			sm_lmc_freeLid_hint = STL_LID_PERMISSIVE;
	}

	return freeLid;
}

static STL_LID
sm_find_next_lid(uint8_t lmc)
{
	STL_LID delta = 1 << lmc; /* delta is the lid space we need to allocate */
	STL_LID endLid = STL_GET_UNICAST_LID_MAX() - delta + 1;

	// Try quick search first
	STL_LID lid1 = sm_find_lid_quick(delta);
	if (lid1 != STL_LID_RESERVED)
		return lid1;

	// Quick find based on previous hints failed, do an
	// exhaustive search over entire lid space.
	STL_LID freeLid;
	for (freeLid = delta; freeLid <= endLid; freeLid += delta) {
		STL_LID nextLid = freeLid + delta;
		for (lid1 = freeLid; lid1 < nextLid; ++lid1) {
			if (lidmap[lid1].guid != 0)
				break; // Space not big enough
		}

		if (lid1 >= nextLid)
			return freeLid;
	}

	// Last chance, recycle LIDs belonging to devices missing from
	// the fabric
	while (1) {
		int hasMore = sm_lidmap_pop_and_search(&lid1, delta);
		if (lid1 != STL_LID_RESERVED)
			return lid1;
		if (!hasMore)
			break;
	}

	static uint32_t lastMsgSweep = (uint32_t)(-1);

	if (topology_passcount != lastMsgSweep) {
		lastMsgSweep = topology_passcount;
		lid_space_exhausted = TRUE;
		IB_LOG_WARN_FMT(__func__, "LID space exhausted, MaximumLID is 0x%x!", STL_GET_UNICAST_LID_MAX());
	}

	return STL_LID_RESERVED;
}

Status_t
sm_lidmap_alloc(void)
{
	Status_t s;

	DEBUG_ASSERT(!lidmap);
	DEBUG_ASSERT(!sm_GuidToLidMap);
	DEBUG_ASSERT(!missingDevHead && !missingDevTail);

	lidmap = NULL;
	s = vs_pool_alloc(&sm_pool, sizeof(LidMap_t) * (STL_GET_UNICAST_LID_MAX() + 1), (void*)&lidmap);
	if (s != VSTATUS_OK || !lidmap) {
		s = VSTATUS_NOMEM;
		return s;
	}
	memset(lidmap, 0, sizeof(LidMap_t) * (STL_GET_UNICAST_LID_MAX() + 1));

	s = vs_pool_alloc(&sm_pool, sizeof(cl_qmap_t), (void *)&sm_GuidToLidMap);
	if (s != VSTATUS_OK || !sm_GuidToLidMap) {
		IB_LOG_ERROR0("can't malloc GuidToLidMap");
		s = VSTATUS_NOMEM;
		return s;
	}
	cl_qmap_init(sm_GuidToLidMap, NULL);

	missingDevHead = missingDevTail = NULL;

	return s;
}

void
sm_lidmap_free(void)
{
	if (lidmap) {
		vs_pool_free(&sm_pool, lidmap);
		lidmap = NULL;
	}

	if (sm_GuidToLidMap) {
		cl_qmap_remove_all(sm_GuidToLidMap);
		vs_pool_free(&sm_pool, sm_GuidToLidMap);
		sm_GuidToLidMap = NULL;
	}

	while (missingDevHead) {
		MissingLidEntry_t *m = missingDevHead->next;
		vs_pool_free(&sm_pool, missingDevHead);
		missingDevHead = m;
	}
	missingDevHead = missingDevTail = NULL;
}

Status_t
sm_lidmap_reset(void)
{
	cl_qmap_remove_all(sm_GuidToLidMap);
	memset(lidmap, 0, sizeof(LidMap_t) * (STL_GET_UNICAST_LID_MAX() + 1));

	while (missingDevHead) {
		MissingLidEntry_t *m = missingDevHead->next;
		vs_pool_free(&sm_pool, missingDevHead);
		missingDevHead = m;
	}
	missingDevHead = missingDevTail = NULL;

	return VSTATUS_OK;
}

static void
sm_lidmap_remove_missing(MissingLidEntry_t *m)
{
	if (m->prev)
		m->prev->next = m->next;

	if (m->next)
		m->next->prev = m->prev;

	// update end pointers (next and prev may be NULL)
	if (m == missingDevHead)
		missingDevHead = m->next;
	if (m == missingDevTail)
		missingDevTail = m->prev;
}

/*
 * Update missing device queue with devices that have
 * gone missing since last update call.
 */
Status_t
sm_lidmap_update_missing(void)
{
	cl_map_item_t *it = cl_qmap_head(sm_GuidToLidMap);

	for (; it != cl_qmap_end(sm_GuidToLidMap); it = cl_qmap_next(it)) {
		LidMap_t *lm = (LidMap_t*) cl_qmap_obj((cl_map_obj_t*)it);
		Guid_t guid = (Guid_t) it->key;

		if (lm->newNodep || lm->missing)
			continue;

		MissingLidEntry_t *m = NULL;
		Status_t s = vs_pool_alloc(&sm_pool, sizeof(MissingLidEntry_t), (void*)&m);
		if (s != VSTATUS_OK)
			return s;
		memset(m, 0, sizeof(MissingLidEntry_t));

		// missing entry points to base LID for guid in lidmap
		m->lm = lm;
		if (!missingDevHead)
			missingDevHead = m;

		if (missingDevTail) {
			missingDevTail->next = m;
			m->prev = missingDevTail;
		}

		missingDevTail = m;

		// all lidmap entries for guid share same missing entry
		for (; lm->guid == guid; ++lm)
			lm->missing = m;
	}

	return VSTATUS_OK;
}

/*
 * Release the LIDs reserved by the device at the front of the missing
 * device queue and search the updated space for @c delta unassigned LIDs
 * starting on a multiple of @delta.
 *
 * @return 1 if queue has more entries or 0 if it is empty
 */
static int
sm_lidmap_pop_and_search(STL_LID *lid, STL_LID delta)
{
	MissingLidEntry_t *m = missingDevHead;
	STL_LID i, start, end;

	*lid = STL_LID_RESERVED;

	if (!missingDevHead)
		return 0;

	start = (m->lm - lidmap);
	end = STL_GET_UNICAST_LID_MAX() + 1;

	Guid_t guid = m->lm->guid;
	cl_map_item_t *it = &lidmap[start].mapObj.item;
	cl_qmap_remove_item(sm_GuidToLidMap, it);
	sm_lidmap_remove_missing(m);
	vs_pool_free(&sm_pool, m);

	for (i = start; i < end; ++i) {
		if (lidmap[i].guid != guid)
			break;

		lidmap[i].guid = 0;
		lidmap[i].pass = 0;
		lidmap[i].oldNodep = NULL;
		lidmap[i].newNodep = NULL;
		lidmap[i].newPortp = NULL;
		lidmap[i].missing = NULL;
		memset(&lidmap[i].mapObj, 0, sizeof(lidmap[i].mapObj));
	}

	// There may be unused LIDs prior to start that can make
	// up the LID space; search backwards for lowest available
	// LID
	for (i = start; i > 0; --i) {
		if (lidmap[i].guid != 0)
			break;
	}

	start = ((i + 1) + (delta - 1)) & (~(delta - 1)); // Round up to nearest multiple of delta

	// Then search forward for other ports in desired LID range
	for (i = start; i < (start + delta); ++i) {
		if (lidmap[i].guid != 0)
			break;
	}

	if (i == (start + delta))
		*lid = start; // all LIDs in range checked, no other ports, success

	return !!(missingDevHead);
}

/*
 * Get expected (exp) (LID,LMC) for @c portp. Expected LID comes from
 * either @c lidmap (if @c portp is in sm_GuidToLidMap) or predefined
 * topology (if lid_strategy == LID_STRATEGY_TOPOLOGY).
 *
 * Will always set @c lmc. Won't set @c lid if expected LID
 * can't be found.
 *
 * @return VSTATUS_OK if lid and lmc are valid (even if lid = 0),
 *   !VSTATUS_OK on runtime error (lid and lmc are invalid)
 */
static Status_t
sm_get_exp_lid_lmc(const Port_t *portp, STL_LID *lid, uint8_t *lmc)
{
	if (!sm_valid_port(portp) || !lid || !lmc)
		return VSTATUS_ILLPARM;

	Node_t *nodep = portp->portData->nodePtr;
	*lid = STL_LID_RESERVED;
	*lmc = 0;

	// always try to determine the LMC
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
		nodep->switchInfo.u2.s.EnhancedPort0) {

		*lmc = sm_config.lmc_e0;
	} else if (nodep->nodeInfo.NodeType == NI_TYPE_CA)
		*lmc = sm_config.lmc;

	if (sm_config.lid_strategy == LID_STRATEGY_TOPOLOGY) {
		ExpectedNode *enodep;
		ExpectedPort *eportp;

		enodep = FindExpectedNodeByNodeGuid(&preDefTopology, nodep->nodeInfo.NodeGUID);

		if (!enodep) {
			enodep = FindExpectedNodeByNodeDesc(&preDefTopology,
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeType);
		}

		if (!enodep)
			return VSTATUS_OK;

		if (!enodep->ports || portp->index >= enodep->portsSize ||
			!enodep->ports[portp->index])
			return VSTATUS_OK;

		eportp = enodep->ports[portp->index];
		*lid = eportp->lid;
		// Predefined topology LMC overrides sm_config.lmc
		*lmc = eportp->lmc;
		return VSTATUS_OK;
	}


	if (sm_config.lid_strategy == LID_STRATEGY_SERIAL &&
		nodep->index == 0 && portp->index == sm_config.port &&
		sm_config.lid != STL_LID_RESERVED) {

		*lid = sm_config.lid;
		return VSTATUS_OK;
	}

	// Last chance: try sm_GuidToLidMap
	cl_map_item_t *it;
	it = cl_qmap_get(sm_GuidToLidMap, portp->portData->guid);
	if (it != cl_qmap_end(sm_GuidToLidMap)) {
		LidMap_t *lm = (LidMap_t*) cl_qmap_obj((cl_map_obj_t*)it);
		*lid = (STL_LID)(lm - lidmap);
		return VSTATUS_OK;
	}

	return VSTATUS_OK;
}

/**
 * Check that @c portp->portData->lid a) is in unicast LID range,
 * b) starts on a multiple of 1 << lmc, and c) that ports
 * in the range @c [lid,lid+(1 << lmc)) are either unmapped
 * or already mapped to @c portp.
 */
LidCheck_t
sm_check_lid(const Port_t * portp, STL_LID lid, uint8_t lmc)
{
	// lid must be within legal range
	if (lid < STL_LID_UNICAST_BEGIN ||
		lid > STL_GET_UNICAST_LID_MAX()) {

		return LIDCHECK_OUTOFRANGE;
	}

	if (lid % (1<< lmc) != 0)
		return LIDCHECK_OFFLMC;

	STL_LID endLid = lid + (1 << lmc);
	if (endLid > (STL_GET_UNICAST_LID_MAX() + 1))
		return LIDCHECK_OUTOFRANGE;

	STL_LID i;
	for (i = lid; i < endLid; ++i) {
		if (lidmap[i].guid != 0 && lidmap[i].guid != portp->portData->guid)
			return LIDCHECK_UNAVAIL;
	}

	return LIDCHECK_OK;
}

/**
 * Update LID in lidmap or assign a new LID to @c portp. Clear port<->LID(s) association if
 * @c portp can't be updated or a new one can't be assigned.
 *
 * @param assignIfFail - if current LID cannot be updated, try to find and assign a new LID
 * @param newLidCount [out] - number of newly-registered LIDs
 *
 */
Status_t
sm_update_or_assign_lid(Port_t *portp, boolean assignIfFail, int *newLidCount)
{
	Node_t *nodep;
	STL_LID testLid = STL_LID_RESERVED;
	uint8_t testLmc;
	if (!sm_valid_port(portp))
		return VSTATUS_ILLPARM;

	nodep = portp->portData->nodePtr;
	boolean evict = (nodep->index == 0 && portp->index == sm_config.port);

	if (newLidCount)
		*newLidCount = 0;

	// Try expected LID (predefined topology, sm_config.lid for SM port,
	// or LID from sm_GuidToLidMap) first
	if (sm_get_exp_lid_lmc(portp, &testLid, &testLmc) != VSTATUS_OK) {
		sm_clear_lid(portp);
		return VSTATUS_BAD;
	}

	if (testLid == STL_LID_RESERVED) {
		if (sm_config.lid_strategy == LID_STRATEGY_TOPOLOGY) {
			sm_clear_lid(portp);
			return VSTATUS_BAD;
		}


		// Try SMA-reported LID next
		if (portp->portData->lid != STL_LID_RESERVED)
			testLid = portp->portData->lid;
	}

	LidCheck_t lc = sm_check_lid(portp, testLid, testLmc);
	if (lc != LIDCHECK_OK) {
		testLid = STL_LID_RESERVED;
		sm_clear_lid(portp);
		if (nodep->index == 0 && portp->index == sm_config.port)
			sm_lid = STL_LID_RESERVED;

		if (!assignIfFail ||
			sm_config.lid_strategy == LID_STRATEGY_TOPOLOGY) {
			// Expected LID not available, nothing else to do
			return VSTATUS_BAD;
		}


		// Try to find new LID. For SM port, use sm_config.lid if specified
		if (nodep->index == 0 && portp->index == sm_config.port && sm_config.lid != STL_LID_RESERVED)
			testLid = sm_config.lid;
		else
			testLid = sm_find_next_lid(testLmc);

		if (testLid == STL_LID_RESERVED) {
			if (!evict)
				return VSTATUS_BAD;
			testLid = (1 << testLmc);
		}

		// Prevent SM from getting locked out of LID space - evict ports
		// from first LID for testLmc
		if (evict) {
			STL_LID i = testLid;
			STL_LID endLid = i + (1 << testLmc);
			for (; i < endLid; ++i) {
				if (lidmap[i].newPortp)
					sm_clear_lid(lidmap[i].newPortp);
			}
		}
	}

	// Update or assign
	int lidCount = sm_set_lid(portp, testLid, testLmc);
	if (lidCount < 0) {
		sm_clear_lid(portp);
		return VSTATUS_BAD;
	}

	if (newLidCount)
		*newLidCount = lidCount;

	if (nodep->index == 0 && portp->index == sm_config.port)
		sm_lid = testLid;

	return VSTATUS_OK;
}

int
sm_set_lid(Port_t * portp, STL_LID lid, uint8_t lmc)
{
	int newLidCount = 0;

	if (lid < UNICAST_LID_MIN || lid > STL_GET_UNICAST_LID_MAX())
		return -1;

	STL_LID endLid = lid + (1 << lmc);

	// only do availability and bounds checks here. Up to caller
	// to do things like LID is modulo LMC, topology-based LID
	// assignment, etc.
	if (endLid > (STL_GET_UNICAST_LID_MAX() + 1))
		return -1;

	STL_LID i;
	// check availability
	for (i = lid; i < endLid; ++i) {
		if (lidmap[i].guid != 0 && lidmap[i].guid != portp->portData->guid)
			return -1;
	}

	// portp<->LID(s) mapping will either be updated or cleared
	// Either way portp won't be missing any longer
	if (lidmap[lid].missing) {
		sm_lidmap_remove_missing(lidmap[lid].missing);
		vs_pool_free(&sm_pool, lidmap[lid].missing);
		lidmap[lid].missing = NULL;
	}

	// TODO in case of GUID moving, is it possible to 'move' map object
	// without removing and re-inserting?
	// If guid is already in map but base LID has changed, remove
	// and re-insert into sm_GuidToLidMap
	cl_map_item_t *it = cl_qmap_get(sm_GuidToLidMap, portp->portData->guid);
	if (it != &lidmap[lid].mapObj.item && it != cl_qmap_end(sm_GuidToLidMap))
		cl_qmap_remove_item(sm_GuidToLidMap, it);

	if (cl_qmap_insert(sm_GuidToLidMap, portp->portData->guid,
		&lidmap[lid].mapObj.item) != &lidmap[lid].mapObj.item)
		return -1;

	cl_qmap_set_obj(&lidmap[lid].mapObj, &lidmap[lid]);

	// then update entries
	for (i = lid; i < endLid; ++i) {
		if (lidmap[i].guid == 0)
			++newLidCount;

		lidmap[i].guid = portp->portData->guid;
		lidmap[i].pass = topology_passcount + 1;
		lidmap[i].newNodep = portp->portData->nodePtr;
		lidmap[i].newPortp = portp;
		lidmap[i].missing = NULL;
	}

#if 0
	// And handle LMC downsize
	for (i = endLid; i <= STL_GET_UNICAST_LID_MAX(); ++i) {
		if (lidmap[i].guid != portp->portData->guid)
			break;
		memset(&lidmap[i], 0, sizeof(LidMap_t));
	}
#endif

	portp->portData->lid = lid;
	portp->portData->lmc = lmc;

	return newLidCount;
}

Status_t
mai_stl_reply(IBhandle_t fd, Mai_t * maip, uint32_t attriblen)
{
	STL_LID  lid;
	uint32_t datalen = attriblen;
	Status_t status;
	// Mai_t drmad;

	IB_ENTER(__func__, fd, maip, 0, 0);

//
//  We need to determine which kind of MAD this was.
//  For a DR MAD, we need to do:
//      1) set the 'D' bit
//      2) set the reply method
//      3) set maip->port = sm_config.port
//      4) decrement the hopPointer
//  For a LR MAD, we need to do:
//      1) swap the DLID with the SLID
//      2) set the reply method
//      3) set maip->port = sm_config.port
//
	maip->addrInfo.destqp &= 0x00ffffff;

	if ((maip->base.method == MAD_CM_GET)
		|| (maip->base.method == MAD_CM_SET)) {
		maip->base.method = MAD_CM_GET_RESP;
		INCREMENT_COUNTER(smCounterTxGetRespSmInfo);
	} else if (maip->base.method == MAD_CM_TRAP) {
		maip->base.method = MAD_CM_TRAP_REPRESS;
        // Adjust the PKEY to PKEY_M, because the SMA has sent the trap
        // with a PKEY value of PKEY_m 
        maip->addrInfo.pkey = STL_DEFAULT_FM_PKEY;
		INCREMENT_COUNTER(smCounterTrapsRepressed);
	} else {
		maip->base.method |= MAD_CM_REPLY;
	}

	if (maip->base.mclass == MAD_CV_SUBN_DR) {
		INCREMENT_COUNTER(smCounterDirectRoutePackets);
		maip->base.status |= MAD_STATUS_D_BIT;
		maip->port = sm_config.port;
		datalen += STL_SMP_DR_HDR_LEN;
	} else if (maip->base.mclass == MAD_CV_SUBN_LR) {
		INCREMENT_COUNTER(smCounterLidRoutedPackets);
		lid = maip->addrInfo.dlid;
		maip->addrInfo.dlid = maip->addrInfo.slid;
		maip->addrInfo.slid = lid;
		maip->port = sm_config.port;
		datalen += STL_SMP_LR_HDR_LEN;
	} else {
		IB_LOG_ERROR("invalid mclass:", maip->base.mclass);
	}

//
//  Send the MAD out.
//
	INCREMENT_COUNTER(smCounterSmPacketTransmits);
	if ((status = mai_stl_send(fd, maip, &datalen)) != VSTATUS_OK)
		IB_LOG_ERRORRC("mai_stl_reply failed rc:", status);

	IB_EXIT(__func__, status);
	return (status);
}


Status_t
mai_reply(IBhandle_t fd, Mai_t * maip)
{
	STL_LID  lid;
	Status_t status;
	// Mai_t drmad;

	IB_ENTER(__func__, fd, maip, 0, 0);

//
//  We need to determine which kind of MAD this was.
//  For a DR MAD, we need to do:
//      1) set the 'D' bit
//      2) set the reply method
//      3) set maip->port = sm_config.port
//      4) decrement the hopPointer
//  For a LR MAD, we need to do:
//      1) swap the DLID with the SLID
//      2) set the reply method
//      3) set maip->port = sm_config.port
//
	maip->addrInfo.destqp &= 0x00ffffff;

	if ((maip->base.method == MAD_CM_GET)
		|| (maip->base.method == MAD_CM_SET)) {
		maip->base.method = MAD_CM_GET_RESP;
		INCREMENT_COUNTER(smCounterTxGetRespSmInfo);
	} else if (maip->base.method == MAD_CM_TRAP) {
		maip->base.method = MAD_CM_TRAP_REPRESS;
        // Adjust the PKEY to PKEY_M, because the SMA has sent the trap
        // with a PKEY value of PKEY_m 
        maip->addrInfo.pkey = STL_DEFAULT_FM_PKEY;
		INCREMENT_COUNTER(smCounterTrapsRepressed);
	} else {
		maip->base.method |= MAD_CM_REPLY;
	}

	if (maip->base.mclass == MAD_CV_SUBN_DR) {
		INCREMENT_COUNTER(smCounterDirectRoutePackets);
		maip->base.status |= MAD_STATUS_D_BIT;
		maip->port = sm_config.port;
	} else if (maip->base.mclass == MAD_CV_SUBN_LR) {
		INCREMENT_COUNTER(smCounterLidRoutedPackets);
		lid = maip->addrInfo.dlid;
		maip->addrInfo.dlid = maip->addrInfo.slid;
		maip->addrInfo.slid = lid;
		maip->port = sm_config.port;
	} else {
		IB_LOG_ERROR("invalid mclass:", maip->base.mclass);
	}

//
//  Send the MAD out.
//
	INCREMENT_COUNTER(smCounterSmPacketTransmits);
	if ((status = mai_send(fd, maip)) != VSTATUS_OK)
		IB_LOG_ERRORRC("mai_send failed rc:", status);

	IB_EXIT(__func__, status);
	return (status);
}


/*
 * Sends an entire LFT to a single node, using either LR or DR MADs.
 *
 * MADs may contain up to sm_config.lft_multi_block LFT entries each.
 *
 */
Status_t
sm_send_lft(Topology_t * topop, Node_t * cnp)
{
	uint16_t currentSet;
	STL_LID  currentLid;
	uint16_t numBlocks = 1;
	Status_t status = VSTATUS_OK;
	Port_t *swportp;
	PORT*	lft=cnp->lft;
	uint32_t amod;
	const uint16_t lids_per_mad = sm_config.lft_multi_block * MAX_LFT_ELEMENTS_BLOCK;

	swportp = sm_get_port(cnp, 0);
	if (!sm_valid_port(swportp)) {
		IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						cnp->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}


	for (currentSet = 0, currentLid = 0; currentLid <= topop->maxLid;
		 currentSet += sm_config.lft_multi_block, currentLid += lids_per_mad) {
		numBlocks =
			(currentLid + lids_per_mad <=
			 topop->maxLid) ? sm_config.lft_multi_block : sm_config.lft_multi_block -
			(lids_per_mad - (topop->maxLid - currentLid + 1)) / MAX_LFT_ELEMENTS_BLOCK;

		amod = (numBlocks << 24) | currentSet;

		status = SM_Set_LFT_Dispatch_LR(fd_topology, amod, sm_lid,
										swportp->portData->lid,
										(STL_LINEAR_FORWARDING_TABLE *) &lft[currentLid], numBlocks, sm_config.mkey, cnp,
										&sm_asyncDispatch);

		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to setup Linear Forwarding "
							 "Table for LIDs 0x%.8X through 0x%.8X on switch %s, node guid "
							 FMT_U64 ", index %d, with rc 0x%.8X",
							 currentLid, currentLid + numBlocks * MAX_LFT_ELEMENTS_BLOCK - 1,
							 sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, cnp->index,
							 status);
			break;
		}
	}

	return (status);
}

Status_t
sm_setup_lft_block(Topology_t * topop, Node_t * switchp, STL_LID lid_entry, SmpAddr_t * addr)
{
	Status_t status;
	uint16_t numBlocks = 1;
	int block;
	STL_LID start_lid;
	uint32_t amod;
	/* program the full LFT but write only the LFT blocks that correspond to the LID entry */

	/* program the LFT block that contains the LID entry */
	block = lid_entry >> 6;
	start_lid = lid_entry & ~0x3f;

//  IB_LOG_INFINI_INFO_FMT(__func__,
//         "lid_entry %d start_lid %d block %d calculating lft for node "FMT_U64,
//          lid_entry, start_lid, block,  switchp->nodeInfo.NodeGUID);


//  IB_LOG_INFINI_INFO_FMT(__func__,
//         "writing lft for node "FMT_U64": lid %d, block %d",
//         switchp->nodeInfo.NodeGUID, lid, lid >> 6);

//  IB_LOG_INFINI_INFO_FMT(__func__, "For switch "FMT_U64" path[0] %d path[1] %d",
//          switchp->nodeInfo.NodeGUID, path[0], path[1]);

	amod = (numBlocks << 24) | block;
	status = SM_Set_LFT(fd_topology, amod, addr,
		(STL_LINEAR_FORWARDING_TABLE *) & switchp->lft[start_lid], sm_config.mkey);

	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to setup Linear Forwarding Table "
						 "for LIDs 0x%.8X through 0x%.8X on switch %s, node guid " FMT_U64 ", "
						 "index %d, with rc 0x%.8X",
						 start_lid, start_lid+64, sm_nodeDescString(switchp),
						 switchp->nodeInfo.NodeGUID, switchp->index, status);
		return status;
	}

	return status;
}

Status_t sm_Node_init_lft(Node_t * switchp, size_t * outSize)
{
	if (switchp->nodeInfo.NodeType != NI_TYPE_SWITCH ||
		switchp->switchInfo.LinearFDBCap == 0) {
		// If we are in sm_Node_init_lft, and the node
		// is not a switch, or if it is a switch with
		// no LFT, then the node is bad.
		return VSTATUS_BAD;
	}

	if (switchp->lft) {
		vs_pool_free(&sm_pool, switchp->lft);
		switchp->lft = 0;
		if (outSize)
			*outSize = 0;
	}

	size_t sizeLft = sizeof(PORT) * ROUNDUP(switchp->switchInfo.LinearFDBTop+1, MAX_LFT_ELEMENTS_BLOCK);

	Status_t s = vs_pool_alloc(&sm_pool, sizeLft, (void**)&switchp->lft);
	if (s == VSTATUS_OK && switchp->lft) {
		memset((void *)switchp->lft, 0xff, sizeLft);
		if (outSize)
			*outSize = sizeLft;
		return s;
	}

	if (outSize)
		*outSize = 0;

	IB_LOG_ERROR_FMT(__func__,
		"Unable to allocate %"PRISZT" LFT memory for node \"%s\", nodeGuid "FMT_U64,
		sizeLft, switchp->nodeDesc.NodeString, switchp->nodeInfo.NodeGUID);

	return s;
}

Status_t
sm_calculate_lft(Topology_t * topop, Node_t * switchp)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	Port_t *portp;
	STL_LID currentLid;
	uint8_t xftPorts[128];

	if (sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__, "switch %s", switchp->nodeDesc.NodeString);

	status = sm_Node_init_lft(switchp, NULL);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP
			("sm_calculate_lft: CAN'T ALLOCATE SPACE FOR NODE'S LFT;  OUT OF MEMORY IN SM MEMORY POOL!  TOO MANY NODES!!");
		return status;
	}

	// Initialize port group top prior to setting up groups.
	switchp->switchInfo.PortGroupTop = 0;

	for_all_nodes(topop, nodep) {
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp) && portp->state > IB_PORT_DOWN) {
				if (portp->portData->lmc > 0) {
					IB_LOG_VERBOSE_FMT(__func__,
							"Setting up Linear Forwarding Table for node[%d] %s, port[%d], LMC=%d.",
							nodep->index, sm_nodeDescString(nodep), portp->index,
							portp->portData->lmc);
				}
				status = topop->routingModule->funcs.setup_xft(topop, switchp, nodep, portp, xftPorts);
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "Failed to setup xft for %s (0x%"PRIx64")",
							nodep->nodeDesc.NodeString, nodep->nodeInfo.NodeGUID);
					return status;
				}

				for_all_port_lids(portp, currentLid) {
					switchp->lft[currentLid] = xftPorts[currentLid - portp->portData->lid];
				}
			}

			if (topop->routingModule->funcs.setup_pgs && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				status = topop->routingModule->funcs.setup_pgs(topop, switchp, nodep);
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "Failed to setup port groups for %s (0x%"PRIx64")",
							nodep->nodeDesc.NodeString, nodep->nodeInfo.NodeGUID);
					return status;
				}
			}
		}
	}

	switchp->routingRecalculated = 1;

	return VSTATUS_OK;
}

/*
 * Use DR or LRDR to send just enough of the LFT to allow the target switch to 
 * communicate with the SM.
 */
Status_t
sm_write_minimal_lft_blocks(Topology_t * topop, Node_t * switchp, SmpAddr_t * addr)
{
	Status_t status;
	Port_t *swportp;

//  IB_LOG_INFINI_INFO_FMT(__func__, "ALLOC LFT switchp guid "FMT_U64" lft %x lfttop %d",
//              switchp->nodeInfo.NodeGUID, switchp->lft, switchp->switchInfo.LinearFDBTop);

	/* setup LFT block that contains the SM LID */
	if ((status =
		 sm_setup_lft_block(topop, switchp, sm_lid, addr)) == VSTATUS_OK) {
		swportp = sm_get_port(switchp, 0);
		if (!sm_valid_port(swportp)) {
			IB_LOG_WARN_FMT(__func__,
							"Failed to get Port 0 of Switch " FMT_U64,
							switchp->nodeInfo.NodeGUID);
			return VSTATUS_BAD;
		}
		/* Need to program switch LID only if it is NOT in the same block as the SM LID */
		if ((sm_lid >> 6) != (swportp->portData->lid >> 6)) {
			status =
				sm_setup_lft_block(topop, switchp, swportp->portData->lid, addr);
		}
	}

	return status;
}

/*
 * Send the LFTs to a set of switches.
 *
 * MADs may contain up to sm_config.lft_multi_block LFT entries each.
 *
 * For each switch, we only actually send the LFT if the we need a rebalance.
 */
Status_t
sm_write_full_lfts_by_block_LR(Topology_t * topop, SwitchList_t * swlist, int rebalance)
{
	Status_t status = VSTATUS_OK;
	SwitchList_t *sw;
	Node_t *switchp;
	Port_t *swportp;
	PORT* lft;
	int block;
	int sent_lft_sets = 0;
	uint16_t numBlocks = 1;
	uint32_t amod;

	// LFTs are already calculated
	// Write the LFT blocks for this switch list


	for (block = 0; block <= (topop->maxLid / MAX_LFT_ELEMENTS_BLOCK);
		 block += sm_config.lft_multi_block) {
		// IB_LOG_INFINI_INFO_FMT(__func__, "block %d topop->maxLid %d",
		// block, topop->maxLid);

		// If we're not doing multi-block avoid re-sending LFT block of SM LID.
		// If we ARE doing multi-block, that's already a bigger savings.
		if (sm_config.lft_multi_block == 1 && block == (sm_lid / MAX_LFT_ELEMENTS_BLOCK)) {
			continue;			/* SM lid block is already programmed in the switch */
		}
		for_switch_list_switches(swlist, sw) {
			switchp = sw->switchp;
			// If this is an old switch, and we don't need a rebalance, skip it.
			if (bitset_test(&old_switchesInUse, switchp->swIdx) &&
				!sm_config.force_rebalance && !rebalance)
				continue;
			swportp = sm_get_port(switchp, 0);
			if (!sm_valid_port(swportp)) {
				IB_LOG_WARN_FMT(__func__,
								"Failed to get Port 0 of Switch " FMT_U64,
								switchp->nodeInfo.NodeGUID);
				status = VSTATUS_BAD;
				goto clearDispatch;
			}
			if (sm_config.sm_debug_routing) 
				IB_LOG_INFINI_INFO_FMT(__func__,
					"setting block %d for switch %s", block, sm_nodeDescString(switchp));

			lft = switchp->lft;
			numBlocks =
				(block + sm_config.lft_multi_block - 1 <=
				 (topop->maxLid /
				  MAX_LFT_ELEMENTS_BLOCK)) ? sm_config.lft_multi_block : (topop->maxLid /
																		  MAX_LFT_ELEMENTS_BLOCK)
				- block + 1;

			// If we're not doing multi-block avoid re-sending LFT block of switch 
			// LID. If we ARE doing multi-block, that's already a bigger savings.
			if (numBlocks == 1 && block == (swportp->portData->lid / MAX_LFT_ELEMENTS_BLOCK)) {
				continue;		/* switch lid block is already programmed in the switch */
			}

			amod = (numBlocks << 24) | block;

			status =
				SM_Set_LFT_Dispatch_LR(fd_topology, amod, sm_lid, swportp->portData->lid,
									   (STL_LINEAR_FORWARDING_TABLE *) & lft[block * MAX_LFT_ELEMENTS_BLOCK],
										numBlocks, sm_config.mkey, switchp, &sm_asyncDispatch);
			sent_lft_sets = 1;
			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
								 "Failed to setup Linear Forwarding Table "
								 "for blocks [%d %d] on switch %s, node guid " FMT_U64 ", "
								 "index %d, with rc 0x%.8X", block, block + numBlocks - 1,
								 sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID,
								 switchp->index, status);
				goto clearDispatch;
			}
		}
	}

clearDispatch:
	if (!sent_lft_sets)
		return status;

	if (status != VSTATUS_OK) {
		sm_dispatch_clear(&sm_asyncDispatch);
	} else {
		status = sm_dispatch_wait(&sm_asyncDispatch);
		if (status != VSTATUS_OK) {
			sm_dispatch_clear(&sm_asyncDispatch);
			IB_LOG_INFINI_INFORC
				("sm_setup_full_lft_blocks: failed to service the dispatch queue for full LFT programming rc:",
				 status);
		}
	}

	return status;
}


Status_t
sm_setup_lft(Topology_t * topop, Node_t * cnp)
{
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, topop, cnp, 0, 0);

	if (sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__, "setting routing for %s", sm_nodeDescString(cnp));

	if ((status = topop->routingModule->funcs.calculate_routes(sm_topop, cnp)) != VSTATUS_OK)
		return status;

	// update lft to include loop paths for loop test
	(void) loopTest_userexit_updateLft(topop, cnp);

//
//  JSY - we need (during discovery) to bound the max LFT so that the
//  intersection of all switches contains the entire LFT range.
//
	status = sm_send_lft(topop, cnp);

	IB_EXIT(__func__, status);
	return status;
}

/*
 * Rewrite parts of the LFT.
 *
 */
Status_t
sm_send_partial_lft(Topology_t * topop, Node_t * cnp, bitset_t * lftBlocks)
{
	STL_LID currentLid=0;
	uint16_t numBlocks = 0;
	int currentBlk, firstBlkInSend=0;
	Status_t status = VSTATUS_OK;
	Port_t *swportp;
	uint32_t amod;

	IB_ENTER(__func__, topop, cnp, 0, 0);

	swportp = sm_get_port(cnp, 0);
	if (!sm_valid_port(swportp)) {
		IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						cnp->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}

	for (currentBlk = bitset_find_first_one(lftBlocks); currentBlk > -1;
		 currentBlk = bitset_find_next_one(lftBlocks, currentBlk + 1)) {
		IB_LOG_INFO_FMT(__func__, "Partial for %s block %d",
		 sm_nodeDescString(cnp), currentBlk);

		if (numBlocks == 0) {
			firstBlkInSend = currentBlk;
			currentLid = currentBlk * LFTABLE_LIST_COUNT;
		}

		numBlocks++;

		// Send if next block not included or max blocks per send reached
		if (!bitset_test(lftBlocks, currentBlk+1) ||
			(numBlocks >= sm_config.lft_multi_block)) {
			IB_LOG_INFO_FMT(__func__, "Sending LFT for LIDs "
			 "0x%.8X through 0x%.8X on switch %s numBlocks %d", currentLid, numBlocks *(currentLid + 63),
			 sm_nodeDescString(cnp), numBlocks);

			amod = (numBlocks << 24) | firstBlkInSend;

			status = SM_Set_LFT_Dispatch_LR(fd_topology, amod, sm_lid,
										swportp->portData->lid,
										(STL_LINEAR_FORWARDING_TABLE *) & cnp->
										lft[currentLid], numBlocks, sm_config.mkey, cnp,
										&sm_asyncDispatch);

			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
							 	"Failed to setup Linear Forwarding Table for LIDs "
							 	"0x%.8X through 0x%.8X on switch %s, node guid " FMT_U64
							 	", index %d, with " "rc 0x%.8X", currentLid, currentLid + 63,
							 	sm_nodeDescString(cnp), cnp->nodeInfo.NodeGUID, cnp->index,
							 	status);
				break;
			}
			numBlocks = 0;
		}
	}

	IB_EXIT(__func__, status);
	return (status);
}


Status_t
sm_setup_lft_deltas(Topology_t * oldtp, Topology_t * newtp, Node_t * cnp)
{

	STL_LID currentLid;
	uint16_t curBlock;
	Node_t *nodep;
	Port_t *portp, *portInUse;
	Status_t status;
	uint8_t xftPorts[128];
	bitset_t lidBlocks;
	int i, optSend = 0;

	IB_ENTER(__func__, newtp, cnp, 0, 0);

	optSend = bitset_init(&sm_pool, &lidBlocks, newtp->maxLid / LFTABLE_LIST_COUNT + 1);
	if (!optSend) {
		IB_LOG_INFINI_INFO("Can't allocate space for lidblock map, size= ",
						   newtp->maxLid / LFTABLE_LIST_COUNT + 1);
	}

	curBlock = 0xffff;
	for (currentLid = 0; currentLid <= newtp->maxLid; currentLid++) {
		if (cnp->lft[currentLid] == 0xff)
			continue;

		if (!lidmap[currentLid].newNodep) {
			if (lidmap[currentLid].oldNodep &&
				(lidmap[currentLid].oldNodep->nodeInfo.NodeType != NI_TYPE_SWITCH)) {
				portInUse = sm_get_port(cnp, cnp->lft[currentLid]);
				if (sm_valid_port(portInUse)
					&& portInUse->portData->lidsRouted > 0) {
					portInUse->portData->lidsRouted--;
				}
			}

			cnp->lft[currentLid] = 0xff;

			if (optSend && (curBlock != currentLid / LFTABLE_LIST_COUNT)) {
				// Set lft block num
				curBlock = currentLid / LFTABLE_LIST_COUNT;
				bitset_set(&lidBlocks, curBlock);
			}
		}
	}

	for (i = bitset_find_first_one(&new_endnodesInUse); i >= 0;
		 i = bitset_find_next_one(&new_endnodesInUse, i + 1)) {

		nodep = sm_find_node(newtp, i);
		if (!nodep) {
			IB_LOG_ERROR0("node not found");
			continue;
		}
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp) && portp->state > IB_PORT_DOWN) {
				newtp->routingModule->funcs.setup_xft(newtp, cnp, nodep, portp, xftPorts);
				curBlock = 0xffff;
				for_all_port_lids(portp, currentLid) {
					cnp->lft[currentLid] = xftPorts[currentLid - portp->portData->lid];
					if (optSend && (curBlock != currentLid / LFTABLE_LIST_COUNT)) {
						// Set lft block num
						curBlock = currentLid / LFTABLE_LIST_COUNT;
						bitset_set(&lidBlocks, curBlock);
					}
				}
			}
		}
		if (newtp->routingModule->funcs.setup_pgs) {
			status = newtp->routingModule->funcs.setup_pgs(newtp, cnp, nodep);
		}
	}

	// update lft to include loop paths for loop test
	(void) loopTest_userexit_updateLft(newtp, cnp);

	if (optSend && !esmLoopTestOn) {
		status = sm_send_partial_lft(newtp, cnp, &lidBlocks);
	} else {
		status = sm_send_lft(newtp, cnp);
	}

	if (optSend) {
		bitset_free(&lidBlocks);
	}

	IB_EXIT(__func__, status);
	return (status);
}

Status_t
sm_port_init_changes(Port_t * portp)
{
	if (!sm_valid_port(portp))
		return VSTATUS_BAD;

	if (portp->portData->changes.slsc || portp->portData->changes.scsl || portp->portData->dirty.init)
		return VSTATUS_BAD;

	portp->portData->dirty.init = 1;

	return VSTATUS_OK;
}

void
sm_port_release_changes(Port_t * portp)
{
	if (!sm_valid_port(portp) || !portp->portData->dirty.init)
		return;

	// All data pointed to by changes is assumed to be on the stack
	// and doesn't need to be freed here now
	memset(&portp->portData->changes, 0, sizeof(portp->portData->changes));
	memset(&portp->portData->dirty, 0, sizeof(portp->portData->dirty));
}

void
sm_node_release_changes(Node_t * nodep)
{
	Port_t * portp;
	for_all_ports(nodep, portp) {
		sm_port_release_changes(portp);
	}
}

// -------------------------------------------------------------------------- //


/*
 * Build SM Service Record
 */
static	void
buildServiceRecordSM(STL_SERVICE_RECORD * srp, uint8_t * servName, uint64_t servID,
					 uint64_t gid_prefix, uint64_t guid, uint16_t flags, uint32_t lease,
					 uint8_t smVersion, uint8_t smState, uint32_t smCapability)
{

	srp->RID.ServiceID = servID;
	srp->RID.ServiceGID.Type.Global.SubnetPrefix = gid_prefix;
	srp->RID.ServiceGID.Type.Global.InterfaceID = guid;
	srp->ServiceLease = lease;
	strncpy((void *) srp->ServiceName, (void *) servName, sizeof(srp->ServiceName));
	srp->ServiceData16[0] = flags;
	srp->RID.Reserved = 0;
	srp->Reserved = 0;

	// encode SM version and state into first bytes of data8
	srp->ServiceData8[0] = smVersion;
	srp->ServiceData8[1] = smState;

	// encode SM capability bits into data32[3]
	srp->ServiceData32[3] = smCapability;

	// add checksums to first 3 words of data32 and the consistency check levels to 
	// the bytes 2 and 3 of data8 if we are configured to do so
	srp->ServiceData32[0] = sm_config.consistency_checksum;
	srp->ServiceData32[1] = pm_config.consistency_checksum;
	(void)vs_rdlock(&old_topology_lock);
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;
	if (VirtualFabrics)
		srp->ServiceData32[2] = VirtualFabrics->consistency_checksum;
	else
		srp->ServiceData32[2] = 0;
	(void)vs_rwunlock(&old_topology_lock);
	srp->ServiceData8[2] = sm_config.config_consistency_check_level;
	srp->ServiceData8[3] = pm_config.config_consistency_check_level;
	// supply the current XM checksum version
	srp->ServiceData8[4] = FM_PROTOCOL_VERSION;

}// End of buildServiceRecordSM()

static
	Status_t
addServiceRecordSM(void)
{
	uint16_t flags;
	uint64_t lease;
	uint64_t time_now;
	STL_SERVICE_RECORD sr;
	ServiceRecKey_t *srkeyp;
	OpaServiceRecord_t *osrp;

	IB_ENTER(__func__, 0, 0, 0, 0);

	// Build master SM service record
	memset((void *) &sr, 0, sizeof(STL_SERVICE_RECORD));
	flags = sm_config.priority;
	lease = 0xFFFFFFFF;
	buildServiceRecordSM(&sr, (void *) SM_SERVICE_NAME, SM_SERVICE_ID,
						 sm_config.subnet_prefix, sm_smInfo.PortGUID, flags, lease,
						 SM_VERSION, SM_STATE_MASTER, SM_CAPABILITY_VSWINFO);

	// Allocate and build service record key
	srkeyp = (ServiceRecKey_t *) malloc(sizeof(ServiceRecKey_t));
	if (srkeyp == NULL) {
		IB_FATAL_ERROR_NODUMP("addServiceRecordSM: Can't allocate service record key");
		IB_EXIT(__func__, VSTATUS_NOMEM);
		return VSTATUS_NOMEM;
	}
	srkeyp->serviceId = sr.RID.ServiceID;
	memcpy(&srkeyp->serviceGid, &sr.RID.ServiceGID, sizeof(IB_GID));
	srkeyp->servicep_key = sr.RID.ServiceP_Key;

	// Lock service record table
	if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) {
		free(srkeyp);
		return VSTATUS_NOMEM;
	}
	// Allocate and add SM service record
	if (!((osrp = (OpaServiceRecord_t *) malloc(sizeof(OpaServiceRecord_t))))) {
		free(srkeyp);
		(void) vs_unlock(&saServiceRecords.serviceRecLock);
		IB_FATAL_ERROR_NODUMP("addServiceRecordSM: Can't allocate Service Record hash entry");
		IB_EXIT(__func__, VSTATUS_NOMEM);
		return VSTATUS_NOMEM;
	}
	osrp->serviceRecord = sr;
	osrp->pkeyDefined = 0;
	(void) vs_time_get(&time_now);
	if (sr.ServiceLease != 0xFFFFFFFF)
		osrp->expireTime = time_now + ((uint64_t) sr.ServiceLease * 1000000);
	else
		osrp->expireTime = VTIMER_ETERNITY;

	if (!cs_hashtable_insert(saServiceRecords.serviceRecMap, srkeyp, osrp)) {
		(void) vs_unlock(&saServiceRecords.serviceRecLock);
		free(srkeyp);
		free(osrp);
		IB_LOG_ERROR_FMT(__func__,
						 "Failed to ADD serviced record ID=" FMT_U64
						 ", serviceName[%s] to service record hashtable", sr.RID.ServiceID,
						 sr.ServiceName);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	// Unlock service record hash table
	(void) vs_unlock(&saServiceRecords.serviceRecLock);

	return VSTATUS_OK;

}								// End of addServiceRecordSM()

Status_t
sm_transition(uint32_t new_state)
{
	Status_t status;
	uint32_t old_state = sm_state;

	IB_ENTER(__func__, new_state, 0, 0, 0);

//
//  Assume failure.
//
	status = VSTATUS_BAD;

//
//  Check the validity of the transition.
//
	if (sm_state == new_state) {
		/* no transtition to do, already there */
		IB_LOG_INFO("No transition necessary ", new_state);
		sm_smInfo.u.s.SMStateCurrent = sm_state;
		IB_EXIT(__func__, VSTATUS_OK);
		return (VSTATUS_OK);
	}

	switch (sm_state) {
	case SM_STATE_NOTACTIVE:
		if (new_state == SM_STATE_DISCOVERING || new_state == SM_STATE_STANDBY) {
			sm_state = new_state;
			status = VSTATUS_OK;
		}
		break;
	case SM_STATE_DISCOVERING:
		if ((new_state == SM_STATE_STANDBY) || (new_state == SM_STATE_MASTER) ||
			(new_state == SM_STATE_NOTACTIVE)) {
			sm_state = new_state;
			status = VSTATUS_OK;
		}
		break;
	case SM_STATE_STANDBY:
		if ((new_state == SM_STATE_MASTER) || (new_state == SM_STATE_DISCOVERING) ||
			(new_state == SM_STATE_NOTACTIVE)) {
			sm_state = new_state;
			status = VSTATUS_OK;
		}
		break;
	case SM_STATE_MASTER:
		if ((new_state == SM_STATE_STANDBY) || (new_state == SM_STATE_NOTACTIVE)) {
			sm_state = new_state;
			status = VSTATUS_OK;
		}
		// if we're transitioning away from master, restore the priority
		sm_restorePriorityOnly();
		break;
	default:
		break;
	}

	if (new_state == SM_STATE_MASTER) {

		(void)vs_rdlock(&old_topology_lock);
		VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;
		if (VirtualFabrics) {
			(void)vs_rwunlock(&old_topology_lock);
			sa_SetDefBcGrp();
		}
		else
			(void)vs_rwunlock(&old_topology_lock);
	}
//
//  Reflect the state in the SMInfo struct.
//
	if (status == VSTATUS_OK) {
		switch (sm_state) {
		case SM_STATE_NOTACTIVE:
			INCREMENT_COUNTER(smCounterSmStateToInactive);
			break;
		case SM_STATE_DISCOVERING:
			INCREMENT_COUNTER(smCounterSmStateToDiscovering);
			break;
		case SM_STATE_STANDBY:
			INCREMENT_COUNTER(smCounterSmStateToStandby);
			break;
		case SM_STATE_MASTER:
			INCREMENT_COUNTER(smCounterSmStateToMaster);
			(void) addServiceRecordSM();
			sm_masterStartTime = (uint32_t)time(NULL);
			break;
		default:
			break;
		}

		if (new_state == SM_STATE_STANDBY)
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SM_STATE_TO_STANDBY, getMyCsmNodeId(),
							NULL, "transition from %s to %s", sm_getStateText(old_state),
							sm_getStateText(new_state));
		else if (new_state == SM_STATE_MASTER)
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SM_STATE_TO_MASTER, getMyCsmNodeId(), NULL,
							"transition from %s to %s", sm_getStateText(old_state),
							sm_getStateText(new_state));
		else if (new_state == SM_STATE_NOTACTIVE)
			smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SM_STATE_TO_NOTACTIVE, getMyCsmNodeId(),
							NULL, "transition from %s to %s", sm_getStateText(old_state),
							sm_getStateText(new_state));
		else
			IB_LOG_INFINI_INFO_FMT(__func__, "SM STATE TRANSITION from %s to %s",
								   sm_getStateText(old_state), sm_getStateText(new_state));

		sm_smInfo.u.s.SMStateCurrent = sm_state;
		sm_prevState = old_state;
		(void) sm_dbsync_updateSm(sm_smInfo.PortGUID, &sm_smInfo);
#ifdef __VXWORKS__
		/* set SM state in IDB */
		(void) idbSetSmState(sm_state);
#endif
	} else {
		IB_LOG_ERROR_FMT(__func__, "SM illegal STATE TRANSITION from %s to %s",
						 sm_getStateText(old_state), sm_getStateText(new_state));
	}

	IB_EXIT(__func__, status);
	return (status);
}

void
sm_elevatePriority(void)
{
	// Do not elevate priority if we have never seen another SM
	// This way, if the prefered secondary boots first, the prefered
	// master can take over.
	// Note, we never become master if our port is down, so we don't need to
	// worry about port state
	if (sm_saw_another_sm && sm_config.elevated_priority > sm_config.priority) {
		sm_smInfo.u.s.Priority = sm_config.elevated_priority;
		(void) sm_dbsync_updateSm(sm_smInfo.PortGUID, &sm_smInfo);
	}
}

void
sm_restorePriorityOnly(void)
{
	// undo elevated priority
	sm_smInfo.u.s.Priority = sm_config.priority;
	(void) sm_dbsync_updateSm(sm_smInfo.PortGUID, &sm_smInfo);
	if (sm_state == SM_STATE_MASTER)
		sm_forceSweep("SM Priority Restored");
}

// undo priority and reset negotiation
void
sm_restorePriority(void)
{
	sm_saw_another_sm = FALSE;
	sm_restorePriorityOnly();
}

// -------------------------------------------------------------------------- //

Status_t
sm_error(Status_t status)
{
	int8_t *cause;

	IB_ENTER(__func__, status, 0, 0, 0);

	if (sm_debug != 0) {
		switch (status) {
		case VSTATUS_OK:
			cause = (int8_t *) "good status";
			break;
		case VSTATUS_BAD:
			cause = (int8_t *) "bad status";
			break;
		case VSTATUS_AGAIN:
			cause = (int8_t *) "retry";
			break;
		case VSTATUS_CONDITIONAL:
			cause = (int8_t *) "mkey compare failed";
			break;
		case VSTATUS_DROP:
			cause = (int8_t *) "drop this MAD";
			break;
		case VSTATUS_EIO:
			cause = (int8_t *) "EIO";
			break;
		case VSTATUS_EXPIRED:
			cause = (int8_t *) "timer expired";
			break;
		case VSTATUS_FORWARD:
			cause = (int8_t *) "forward this MAD";
			break;
		case VSTATUS_NOT_MASTER:
			cause = (int8_t *) "not the MASTER";
			break;
		case VSTATUS_QEMPTY:
			cause = (int8_t *) "out of records";
			break;
		case VSTATUS_TIMEOUT:
			cause = (int8_t *) "timeout";
			break;
		default:
			cause = (int8_t *) "unknown error";
			break;
		}

		printf("SM : %s\n", cause);
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return (VSTATUS_OK);
}

// -------------------------------------------------------------------------- //

Status_t
sm_start_thread(SMThread_t * smthreadp)
{
	Status_t status;

	IB_ENTER(__func__, smthreadp, 0, 0, 0);

	if (sm_debug != 0) {
		printf("Starting %s\n", smthreadp->id);
	}
//
//  Sleep for half a second to let the previous thread startup.
//
	(void) vs_thread_sleep(VTIMER_1S / 2);

//
//  Start the thread.
//
	status = vs_thread_create(&smthreadp->handle,
							  (unsigned char *) smthreadp->id,
							  smthreadp->function, 0, NULL, SM_STACK_SIZE);

	IB_EXIT(__func__, status);
	return (status);
}

// -------------------------------------------------------------------------- //

// JSY #undef   IB_LOG_INFO
// JSY #define  IB_LOG_INFO(A,B)    printf(A)

Status_t
sm_dump_mai(const char *header, Mai_t * maip)
{
	int i;
	uint8_t *cp;
	MaiAddrInfo_t *addrInfop;
	Mad_t *madp;

	if (!IB_LOG_IS_INTERESTED(VS_LOG_INFO))
		return VSTATUS_OK;

	IB_ENTER(__func__, maip, 0, 0, 0);

	IB_LOG_INFO0(header);

	addrInfop = &maip->addrInfo;
	IB_LOG_INFO_FMT(__func__, "AddrInfo:\tSLID=%08x   DLID=%08x  SL=%u\n",
					addrInfop->slid, addrInfop->dlid, addrInfop->sl);
	IB_LOG_INFO_FMT(__func__, "         \tPKEY=%04x   SRC_QP=%u    DST_QP=%u",
					addrInfop->pkey, addrInfop->srcqp, addrInfop->destqp);
	IB_LOG_INFO_FMT(__func__, "         \tQKEY=%08x",
					addrInfop->qkey);

	madp = &maip->base;
	IB_LOG_INFO_FMT(__func__, "Mad:\tBVERSION=%d   CVERSION=%d", madp->bversion, madp->cversion);
	IB_LOG_INFO_FMT(__func__, "\tMCLASS=%x   METHOD=%x   STATUS=%x",
					madp->mclass, madp->method, madp->status);
	IB_LOG_INFO_FMT(__func__, "\tHOP_POINTER=%d   HOP_COUNT=%d", madp->hopPointer, madp->hopCount);
	IB_LOG_INFO_FMT(__func__, "\tTID=" FMT_U64 "   AID=%04x   AMOD=%08x",
					madp->tid, madp->aid, (int) madp->amod);

	if (madp->bversion == STL_BASE_VERSION) {
		DRStlSmp_t drStlSmp;
		BSWAPCOPY_STL_DR_SMP((DRStlSmp_t *) & (maip->data), &drStlSmp);
		IB_LOG_INFO_FMT(__func__, "\tMKEY=" FMT_U64 "", drStlSmp.M_Key);

		cp = drStlSmp.SMPData;
		IB_LOG_INFO0("DATA:");
		for (i = 0; i < 4; i++, cp += 16) {
			IB_LOG_INFO_FMT(__func__, "\t%02x %02x %02x %02x %02x %02x %02x %02x"
							" %02x %02x %02x %02x %02x %02x %02x %02x",
							*(cp + 0), *(cp + 1), *(cp + 2), *(cp + 3), *(cp + 4),
							*(cp + 5), *(cp + 6), *(cp + 7), *(cp + 8), *(cp + 9),
							*(cp + 10), *(cp + 11), *(cp + 12), *(cp + 13), *(cp + 14),
							*(cp + 15));
		}

		// Ipath
		cp = drStlSmp.InitPath;
		IB_LOG_INFO0("IPATH:");
		for (i = 0; i < 4; i++, cp += 16) {
			IB_LOG_INFO_FMT(__func__, "\t%02x %02x %02x %02x %02x %02x %02x %02x"
							" %02x %02x %02x %02x %02x %02x %02x %02x",
							*(cp + 0), *(cp + 1), *(cp + 2), *(cp + 3), *(cp + 4),
							*(cp + 5), *(cp + 6), *(cp + 7), *(cp + 8), *(cp + 9),
							*(cp + 10), *(cp + 11), *(cp + 12), *(cp + 13), *(cp + 14),
							*(cp + 15));
		}

		// Rpath
		cp = drStlSmp.RetPath;
		IB_LOG_INFO0("RPATH:");
		for (i = 0; i < 4; i++, cp += 16) {
			IB_LOG_INFO_FMT(__func__, "\t%02x %02x %02x %02x %02x %02x %02x %02x"
							" %02x %02x %02x %02x %02x %02x %02x %02x",
							*(cp + 0), *(cp + 1), *(cp + 2), *(cp + 3), *(cp + 4),
							*(cp + 5), *(cp + 6), *(cp + 7), *(cp + 8), *(cp + 9),
							*(cp + 10), *(cp + 11), *(cp + 12), *(cp + 13), *(cp + 14),
							*(cp + 15));
		}
	}

	IB_LOG_INFO0("============================================================");

	IB_EXIT(__func__, VSTATUS_OK);
	return (VSTATUS_OK);
}

//---------------------------------------------------------------------------//

void
sm_util_print_search_stats(void)
{

	sysPrintf("Node Guid: %u\n", (int) sm_node_guid_cnt);
	sysPrintf("Node ID  : %u\n", (int) sm_node_id_cnt);
	sysPrintf("Port Guid: %u\n", (int) sm_port_guid_cnt);
	sysPrintf("Port ID  : %u\n", (int) sm_port_id_cnt);
	sysPrintf("NP   ID  : %u\n", (int) sm_np_id_cnt);
	sysPrintf("Port Lid : %u\n", (int) sm_port_lid_cnt);
	sysPrintf("NP   Lid : %u\n", (int) sm_np_lid_cnt);
	sysPrintf("Switch ID: %u\n", (int) sm_switch_id_cnt);
	sysPrintf("Port Node: %u\n", (int) sm_port_node_cnt);

}

/*
 * Is this a master SM with a valid searchable topology
*/
uint32_t
sm_util_isTopologyValid()
{
	/* return true if master and valid searchable topology DB */
	if (sm_util_get_state() == SM_STATE_MASTER && sm_util_get_passcount())
		return 1;
	else
		return 0;
}


uint32_t
sm_util_get_passcount(void)
{
	return topology_passcount;
}


uint32_t
sm_util_get_state(void)
{
	return sm_state;
}


void
sm_util_set_non_resp(uint32_t sec, uint32_t max_count)
{
	sm_config.non_resp_tsec = sec;
	sm_config.non_resp_max_count = max_count;
}

void
sm_util_print_non_resp_config(void)
{
	sysPrintf("Timeout: %" CS64 "ull Count: %u\n", sm_config.non_resp_tsec,
			  (int) sm_config.non_resp_max_count);
}

#if defined(__VXWORKS__)
Port_t *
sm_get_port(const Node_t * nodep, uint32_t portIndex)
{
	if (portIndex > nodep->nodeInfo.NumPorts) {
		IB_LOG_ERROR("Invalid node port record index, index=", portIndex);
	} else {
		return &nodep->port[portIndex];
	}

	return NULL;
}
#endif

static Status_t
sm_util_alloc_port(Port_t * portp, int numPorts, int *bytes)
{
	Status_t status;

	if (smDebugDynamicPortAlloc)
		IB_ENTER(__func__, 0, 0, 0, 0);

	// Allocate port record associated with the port.
	status = vs_pool_alloc(&sm_pool, sizeof(PortData_t), (void *) &portp->portData);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR0("can't malloc port");
		(void) vs_pool_free(&sm_pool, (void *) portp->portData);
		portp->portData = NULL;
		if (smDebugDynamicPortAlloc)
			IB_EXIT(__func__, status);
		return (status);
	}
	// Initialize port record.
	memset(portp->portData, 0, sizeof(PortData_t));

	if (!bitset_init(&sm_pool, &portp->portData->vfMember, MAX_VFABRICS)) {
		status = VSTATUS_BAD;
		IB_LOG_ERROR0("can't malloc port data");
		(void) vs_pool_free(&sm_pool, (void *) portp->portData);
		portp->portData = NULL;
		if (smDebugDynamicPortAlloc)
			IB_EXIT(__func__, status);
		return (status);
	}

	if (!bitset_init(&sm_pool, &portp->portData->fullPKeyMember, MAX_VFABRICS)) {
		status = VSTATUS_BAD;
		IB_LOG_ERROR0("can't malloc port data");
		bitset_free(&portp->portData->vfMember);
		(void) vs_pool_free(&sm_pool, (void *) portp->portData);
		portp->portData = NULL;
		if (smDebugDynamicPortAlloc)
			IB_EXIT(__func__, status);
		return (status);
	}
	if (!bitset_init(&sm_pool, &portp->portData->pkey_idxs, SM_PKEYS)) {
		status = VSTATUS_BAD;
		IB_LOG_ERROR0("can't malloc port data");
		bitset_free(&portp->portData->vfMember);
		bitset_free(&portp->portData->fullPKeyMember);
		(void) vs_pool_free(&sm_pool, (void *) portp->portData);
		portp->portData = NULL;
		if (smDebugDynamicPortAlloc)
		IB_EXIT(__func__, status);
		return (status);
	}

	if (!bitset_init(&sm_pool, &portp->portData->dgMember, MAX_VFABRIC_GROUPS)) {
		status = VSTATUS_BAD;
		IB_LOG_ERROR0("can't malloc port data");
		bitset_free(&portp->portData->vfMember);
		bitset_free(&portp->portData->fullPKeyMember);
		bitset_free(&portp->portData->pkey_idxs);
		(void) vs_pool_free(&sm_pool, (void *) portp->portData);
		portp->portData = NULL;
		if (smDebugDynamicPortAlloc)
			IB_EXIT(__func__, status);
		return (status);
	}

	if (smDebugDynamicPortAlloc)
		IB_EXIT(__func__, VSTATUS_OK);

	return VSTATUS_OK;
}

PortData_t *
sm_alloc_port(Topology_t * topop, Node_t * nodep, uint32_t portIndex, int *bytes)
{
	if (smDebugDynamicPortAlloc)
		IB_ENTER(__func__, 0, 0, 0, 0);

	if (nodep->port[portIndex].portData == NULL) {

		if (smDebugDynamicPortAlloc) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Allocating port %d for node %d",
								   portIndex, nodep->index);
		}
		// Allocate port record associated with the port.
		if (sm_util_alloc_port(&nodep->port[portIndex], nodep->nodeInfo.NumPorts, bytes) !=
			VSTATUS_OK) {
			return NULL;
		}
		// Assign the node parent ptr to improve search performance.
		nodep->port[portIndex].portData->nodePtr = nodep;
	}

	if (smDebugDynamicPortAlloc)
		IB_EXIT(__func__, VSTATUS_OK);

	return nodep->port[portIndex].portData;
}

static void
sm_util_free_port(Port_t * portp)
{
	if (smDebugDynamicPortAlloc)
		IB_ENTER(__func__, 0, 0, 0, 0);

	// Undo the allocations done by sm_util_alloc_port
	bitset_free(&portp->portData->vfMember);
	bitset_free(&portp->portData->fullPKeyMember);
	bitset_free(&portp->portData->dgMember);
	bitset_free(&portp->portData->pkey_idxs);
	(void) vs_pool_free(&sm_pool, (void *) portp->portData);
	portp->portData = NULL;

	if (smDebugDynamicPortAlloc)
		IB_EXIT(__func__, VSTATUS_OK);
}

void
sm_free_port(Topology_t * topop, Port_t * portp)
{
	if (smDebugDynamicPortAlloc)
		IB_ENTER(__func__, 0, 0, 0, 0);

	if (sm_valid_port(portp)) {
		if (smDebugDynamicPortAlloc) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Freeing  port %d for node %d",
								   portp->index, ((Node_t *) portp->portData->nodePtr)->index);
		}

		if (topop->portMap)
			cl_qmap_remove(topop->portMap, portp->portData->guid);
		stl_sm_cca_hfi_ref_release(&portp->portData->congConRefCount);
		LIST_ITEM *p;
		int i;
		for (i=0; i<2; i++) {
			while (!QListIsEmpty(&portp->portData->scscMapList[i])) {
				p = QListTail(&portp->portData->scscMapList[i]);
				PortDataSCSCMapPortMask *pscsc = (PortDataSCSCMapPortMask *)QListObj(p);
				QListRemoveTail(&portp->portData->scscMapList[i]);
				vs_pool_free(&sm_pool, pscsc->SCSCMap);
				pscsc->SCSCMap = NULL;
				vs_pool_free(&sm_pool, pscsc);
			}
			QListDestroy(&portp->portData->scscMapList[i]);
		}

		sm_port_releaseNewArb(portp);

		sm_util_free_port(portp);
	}

	if (smDebugDynamicPortAlloc)
		IB_EXIT(__func__, VSTATUS_OK);
}

void
sm_node_free_port(Topology_t * topop, Node_t * nodep)
{
	Port_t * portp;

	if (!nodep) return;

	for_all_ports(nodep, portp)
		sm_free_port(topop, portp);
}

Status_t
sm_build_node_array(Topology_t * topop)
{
	Status_t status;
	int i;
	Node_t *nodep;

	status =
		vs_pool_alloc(&sm_pool, sizeof(Node_t *) * topop->num_nodes,
					  (void *) &topop->nodeArray);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN("failed to allocate node array", status);
		return status;
	}

	status = VSTATUS_OK;
	for (i = 0, nodep = topop->node_head; nodep != NULL; ++i, nodep = nodep->next) {
		if (i >= topop->num_nodes) {
			IB_LOG_WARN("found more nodes in node list than expected", 0);
			status = VSTATUS_BAD;
			break;
		} else if (nodep->index >= topop->num_nodes) {
			IB_LOG_WARN("found node with invalid index; skipping",
						nodep->index);
		} else {
			topop->nodeArray[nodep->index] = nodep;
		}
	}

	return status;
}

//---------------------------------------------------------------------------//

void
sm_setTrapThreshold(uint32_t trapThreshold, uint32_t min_trap_count)
{
	sm_trapThreshold = trapThreshold;
	sm_trapThreshold_minCount = min_trap_count;
	if (sm_trapThreshold == 0) {
		sm_trapThresholdWindow = 0;
	} else {
		sm_trapThresholdWindow =
			(60ull * VTIMER_1S * sm_trapThreshold_minCount) / sm_trapThreshold;
	}
}

//---------------------------------------------------------------------------//
//
// Methods for interacting generically with "removed" fabric entitites.
//

void
sm_removedEntities_init(void)
{
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, 0, 0, 0, 0);

	if (sm_removedEntities.initialized) {
		IB_LOG_VERBOSE0("already initialized!");
		return;
	}

	cl_qmap_init(&sm_removedEntities.nodeMap, NULL);

	status = vs_lock_init(&sm_removedEntities.lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_removedEntities_init: failed to initialize lock");
	}

	sm_removedEntities.initialized = 1;

	IB_EXIT(__func__, 0);
	return;
}

void
sm_removedEntities_destroy(void)
{
	(void) vs_lock_delete(&sm_removedEntities.lock);
	sm_removedEntities.initialized = 0;
}

static void
sm_removedEntities_freePort(RemovedPort_t * rp)
{
	(void) vs_pool_free(&sm_pool, rp);
}

static Status_t
sm_removedEntities_addNode(RemovedNode_t * rn)
{
	if (cl_qmap_insert(&sm_removedEntities.nodeMap, rn->guid, &rn->mapObj.item) !=
		&rn->mapObj.item)
		return VSTATUS_BAD;

	cl_qmap_set_obj(&rn->mapObj, rn);
	return VSTATUS_OK;
}

static Status_t
sm_removedEntities_addPortToNode(RemovedNode_t * rn, RemovedPort_t * rp)
{
	RemovedPort_t *prev = NULL, *curr = rn->ports;

	while (curr != NULL && curr->index < rp->index) {
		prev = curr;
		curr = curr->next;
	}

	// duplicate?
	if (curr != NULL && curr->index == rp->index)
		return VSTATUS_BAD;

	rp->next = curr;

	if (prev != NULL)
		prev->next = rp;
	else
		rn->ports = rp;

	return VSTATUS_OK;
}

static RemovedNode_t *
sm_removedEntities_findNodeByGuid(uint64_t guid)
{
	cl_map_item_t *mi;

	// find node; create if necessary
	mi = cl_qmap_get(&sm_removedEntities.nodeMap, guid);
	if (mi == cl_qmap_end(&sm_removedEntities.nodeMap))
		return NULL;

	return (RemovedNode_t *) cl_qmap_obj((const cl_map_obj_t * const) mi);
}

static RemovedPort_t *
sm_removedEntities_findPortByIndex(RemovedNode_t * rn, uint8_t index)
{
	RemovedPort_t *rp;

	if (rn == NULL)
		return NULL;

	rp = rn->ports;
	while (rp != NULL && rp->index != index)
		rp = rp->next;

	return rp;
}

static RemovedPort_t *
sm_removedEntities_removePortByIndex(RemovedNode_t * rn, uint8_t index)
{
	RemovedPort_t *prev = NULL, *curr = NULL;

	if (rn == NULL)
		return NULL;

	curr = rn->ports;
	while (curr != NULL && curr->index < index) {
		prev = curr;
		curr = curr->next;
	}

	if (curr == NULL || curr->index != index)
		return NULL;

	if (prev != NULL)
		prev->next = curr->next;
	else
		rn->ports = curr->next;

	return curr;
}

static Status_t
sm_removedEntities_removeNode(RemovedNode_t * rn, void (*clean) (RemovedPort_t *))
{
	RemovedPort_t *rp;
	cl_map_item_t *mi;

	mi = cl_qmap_get(&sm_removedEntities.nodeMap, rn->guid);
	if (mi == cl_qmap_end(&sm_removedEntities.nodeMap)) {
		return VSTATUS_BAD;
	}

	while (rn->ports != NULL) {
		rp = sm_removedEntities_removePortByIndex(rn, rn->ports->index);
		if (rp != NULL && clean != NULL)
			clean(rp);
	}

	cl_qmap_remove_item(&sm_removedEntities.nodeMap, mi);

	return VSTATUS_OK;
}

static void
sm_removedEntities_reasonToString(RemovedEntityReason_t reason, char *str, int len)
{
	switch (reason) {
	case SM_REMOVAL_REASON_1X_LINK:
		strncpy(str, "1x Link Width", MIN(14, len));
		break;
	case SM_REMOVAL_REASON_TRAP_SUPPRESS:
		strncpy(str, "Trap Threshold Exceeded", MIN(24, len));
		break;
	case SM_REMOVAL_REASON_MC_DOS:
		strncpy(str, "Mcast DOS Threshold Exceeded", MIN(32, len));
		break;
	default:
		strncpy(str, "Unknown", MIN(8, len));
		break;
	}
}

Status_t
sm_removedEntities_reportPort(Node_t * nodep, Port_t * portp, RemovedEntityReason_t reason)
{
	Status_t status;
	RemovedNode_t *rn;
	RemovedPort_t *rp;
	uint8_t newNode = 0;

	IB_ENTER(__func__, nodep, portp, 0, 0);

	if (nodep == NULL || !sm_valid_port(portp)) {
		IB_LOG_ERROR0("invalid node or port specified");
		IB_EXIT(__func__, VSTATUS_ILLPARM);
		return VSTATUS_ILLPARM;
	}

	(void) vs_lock(&sm_removedEntities.lock);

	rn = sm_removedEntities_findNodeByGuid(nodep->nodeInfo.NodeGUID);
	if (rn == NULL) {
		status = vs_pool_alloc(&sm_pool, sizeof(RemovedNode_t), (void *) &rn);
		if (status != VSTATUS_OK) {
			(void) vs_unlock(&sm_removedEntities.lock);
			IB_LOG_WARN_FMT(__func__,
							"Failed to allocate memory to store node info: Node='%s', GUID="
							FMT_U64 ", NodeIndex=%d, PortIndex=%d", sm_nodeDescString(nodep),
							nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
			IB_EXIT(__func__, status);
			return status;
		}
		rn->guid = nodep->nodeInfo.NodeGUID;
		rn->desc = nodep->nodeDesc;
		rn->ports = NULL;
		newNode = 1;
	}

	if (!newNode) {
		// node already exists; does the port?
		rp = sm_removedEntities_findPortByIndex(rn, portp->index);
		if (rp != NULL) {
			// tried to mark a port that's already been marked; warn somebody
			(void) vs_unlock(&sm_removedEntities.lock);
			IB_LOG_WARN_FMT(__func__,
							"Failed to add port to node (already exists): Node='%s', GUID="
							FMT_U64 ", NodeIndex=%d, PortIndex=%d", sm_nodeDescString(nodep),
							nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
	}
	// create port
	status = vs_pool_alloc(&sm_pool, sizeof(RemovedPort_t), (void *) &rp);
	if (status != VSTATUS_OK) {
		(void) vs_unlock(&sm_removedEntities.lock);
		IB_LOG_WARN_FMT(__func__,
						"Failed to allocate memory to store port info: Node='%s', GUID=" FMT_U64
						", NodeIndex=%d, PortIndex=%d", sm_nodeDescString(nodep),
						nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
		if (newNode)
			(void) vs_pool_free(&sm_pool, rn);
		IB_EXIT(__func__, status);
		return status;
	}

	rp->guid = portp->portData->guid;
	rp->index = portp->index;
	rp->reason = reason;

	// connect node and port
	status = sm_removedEntities_addPortToNode(rn, rp);
	if (status != VSTATUS_OK) {
		(void) vs_unlock(&sm_removedEntities.lock);
		IB_LOG_WARN_FMT(__func__,
						"Failed to add port to node: Node='%s', GUID=" FMT_U64
						", NodeIndex=%d, PortIndex=%d", sm_nodeDescString(nodep),
						nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
		if (newNode)
			(void) vs_pool_free(&sm_pool, rn);
		(void) vs_pool_free(&sm_pool, rp);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	// add port if node entry was just created
	if (newNode && sm_removedEntities_addNode(rn) != VSTATUS_OK) {
		(void) vs_unlock(&sm_removedEntities.lock);
		IB_LOG_WARN_FMT(__func__,
						"Failed to add node to map (already exists): Node='%s', GUID=" FMT_U64
						", NodeIndex=%d, PortIndex=%d", sm_nodeDescString(nodep),
						nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
		(void) vs_pool_free(&sm_pool, rn);
		(void) vs_pool_free(&sm_pool, rp);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	(void) vs_unlock(&sm_removedEntities.lock);

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

Status_t
sm_removedEntities_clearNode(Node_t * nodep)
{
	RemovedNode_t *rn;

	IB_ENTER(__func__, nodep, 0, 0, 0);

	(void) vs_lock(&sm_removedEntities.lock);

	rn = sm_removedEntities_findNodeByGuid(nodep->nodeInfo.NodeGUID);
	if (rn == NULL) {
		(void) vs_unlock(&sm_removedEntities.lock);
		return VSTATUS_BAD;
	}

	sm_removedEntities_removeNode(rn, sm_removedEntities_freePort);
	(void) vs_pool_free(&sm_pool, rn);

	(void) vs_unlock(&sm_removedEntities.lock);

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

Status_t
sm_removedEntities_clearPort(Node_t * nodep, Port_t * portp)
{
	RemovedNode_t *rn;
	RemovedPort_t *rp;

	IB_ENTER(__func__, nodep, portp, 0, 0);

	(void) vs_lock(&sm_removedEntities.lock);

	rn = sm_removedEntities_findNodeByGuid(nodep->nodeInfo.NodeGUID);
	if (rn == NULL) {
		(void) vs_unlock(&sm_removedEntities.lock);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	rp = sm_removedEntities_removePortByIndex(rn, portp->index);
	if (rp != NULL)
		(void) vs_pool_free(&sm_pool, rp);

	// no more ports? remove node
	if (rn->ports == NULL) {
		(void) sm_removedEntities_removeNode(rn, NULL);
		(void) vs_pool_free(&sm_pool, rn);
	}

	(void) vs_unlock(&sm_removedEntities.lock);

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

void
sm_removedEntities_displayPorts(void)
{
	cl_map_item_t *mi;
	RemovedNode_t *rn;
	RemovedPort_t *rp;
	char reason[32];

	if (topology_passcount < 1) {
		sysPrintf("\nSM is currently in the %s state, count = %d\n\n",
				  sm_getStateText(sm_state), (int) sm_smInfo.ActCount);
		return;
	}

	(void) vs_lock(&sm_removedEntities.lock);

	sysPrintf("Disabled Ports:\n");

	mi = cl_qmap_head(&sm_removedEntities.nodeMap);
	while (mi != cl_qmap_end(&sm_removedEntities.nodeMap)) {
		rn = (RemovedNode_t *) cl_qmap_obj((const cl_map_obj_t * const) mi);
		if (rn->ports != NULL) {
			sysPrintf("  Node " FMT_U64 ": %s\n", rn->guid, rn->desc.NodeString);
			for (rp = rn->ports; rp != NULL; rp = rp->next) {
				sm_removedEntities_reasonToString(rp->reason, reason, sizeof(reason));
				sysPrintf("    Port %3u: %s\n", (unsigned int) rp->index, reason);
			}
		}
		mi = cl_qmap_next(mi);
	}

	(void) vs_unlock(&sm_removedEntities.lock);

	return;
}

void
sm_clearSwitchPortChange(Topology_t * topop)
{
	Node_t *switchp;

	for_all_switch_nodes(topop, switchp) {
		if (switchp->switchInfo.u1.s.PortStateChange)
			switchp->switchInfo.u1.s.PortStateChange = 0;
	}
}

void
sm_compactSwitchSpace(Topology_t * topop, bitset_t * switchbits)
{
	Node_t *nodep;
	int fidx, lidx, i, j, ij, ji, oldij, oldji;
	uint16_t *cost = NULL;
	SmPathPortmask_t *path = NULL;
	size_t numSws = switchbits->nset_m;
	size_t bytesCost = numSws * numSws * sizeof(uint16_t);
	size_t bytesPath = SM_PATH_SIZE(numSws);

	if (switchbits->nset_m == 0)
		return;

	if (switchbits->nset_m == topop->max_sws)
		return;

	if ((((topop->max_sws - switchbits->nset_m) * 100) / topop->max_sws) < 10) {
		// compact if sparce
		return;
	}

	/* Allocate space for the cost array. */
	if (topop->cost) {
		if (vs_pool_alloc(&sm_pool, bytesCost, (void *) &cost) != VSTATUS_OK) {
			return;
		}

		if (vs_pool_alloc(&sm_pool, bytesPath, (void *) &path) != VSTATUS_OK) {
			vs_pool_free(&sm_pool, (void *) &cost);
			return;
		}
	}

	for (fidx = bitset_find_first_zero(switchbits); fidx >= 0;
		 fidx = bitset_find_next_zero(switchbits, fidx + 1)) {

		if ((lidx = bitset_find_last_one(switchbits)) >= 0) {
			if (fidx > lidx)
				break;

			for (nodep = topop->switch_tail; nodep != NULL; nodep = nodep->type_prev) {
				if (nodep->swIdx == lidx) {
					sm_topop->routingModule->funcs.process_swIdx_change(sm_topop, lidx, fidx, lidx);

					nodep->swIdx = fidx;
					bitset_set(switchbits, fidx);
					bitset_clear(switchbits, lidx);

					if (topop->cost) {
						for (j = 0; j < lidx; j++) {
							ij = Index(fidx, j);
							ji = Index(j, fidx);

							if (j == fidx) {
								topop->cost[ij] = topop->cost[ji] = 0;
								topop->path[ij] = topop->path[ji] = SM_PATH_PORTMASK_EMPTY;
								continue;
							}

							oldij = Index(lidx, j);
							oldji = Index(j, lidx);
							topop->cost[ij] = topop->cost[ji] = topop->cost[oldij];

							topop->path[ij] = topop->path[oldij];
							topop->path[ji] = topop->path[oldji];
						}
					}
					break;
				}
			}
		}
	}

	if (cost) {
		for (i = 0; i < numSws; i++) {
			(void) memcpy((void *) (cost + (i * numSws)),
						  (void *) (topop->cost + (i * topop->max_sws)),
						  numSws * sizeof(uint16_t));
		}

		(void) vs_pool_free(&sm_pool, (void *) topop->cost);

		topop->cost = cost;
		topop->bytesCost = bytesCost;
	}

	if (path) {
		for (i = 0; i < numSws; i++) {
			(void) memcpy((void *) (path + SM_PATH_OFFSET(i, numSws)),
						  (void *) (topop->path + SM_PATH_OFFSET(i, topop->max_sws)),
						  SM_PATH_SWITCH_SPAN(numSws));
		}

		(void) vs_pool_free(&sm_pool, (void *) topop->path);

		topop->path = path;
		topop->bytesPath = bytesPath;
	}

	topop->max_sws = switchbits->nset_m;

	return;
}

void
sm_log_topology(Topology_t * topop)
{

	Node_t *nodep = NULL;
	Node_t *linkednodep = NULL;
	Port_t *portp;
	int i;
	STL_LID lid;

	if (sm_debug == 0) {
		return;
	}

	for_all_nodes(topop, nodep) {
		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
			continue;

		IB_LOG_INFINI_INFO_FMT(__func__, "Switch Node %s : switch index= %d>>>>>>>>>>",
							   sm_nodeDescString(nodep), nodep->swIdx);

		bitset_info_log(&nodep->activePorts, "activePorts");
		bitset_info_log(&nodep->initPorts, "initPorts");

		for (i = bitset_find_first_one(&nodep->activePorts); i >= 0;
			 i = bitset_find_next_one(&nodep->activePorts, i + 1)) {
			portp = sm_get_port(nodep, i);
			if (!sm_valid_port(portp)) continue;
			if (i == 0) {
				IB_LOG_INFINI_INFO_FMT(__func__,
									   "smport zero (state= %d), base lid= 0x%x", portp->state,
									   portp->portData->lid);
				continue;
			}
			IB_LOG_INFINI_INFO_FMT(__func__, "port index= %d: state= %d, nodeno= %d",
								   i, portp->state, portp->nodeno);
			if (portp->nodeno > 0) {
				linkednodep = sm_find_node(topop, portp->nodeno);
				if (linkednodep) {
					IB_LOG_INFINI_INFO_FMT(__func__, "connected to node %s port %d",
										   sm_nodeDescString(linkednodep), portp->portno);
				}
			}
		}
		for (lid = 0; lid <= topop->maxLid; lid++) {
			if (nodep->lft[lid] == 0xff)
				continue;
			IB_LOG_INFINI_INFO_FMT(__func__, "lid  0x%x  port %d", lid,
								   nodep->lft[lid]);
		}
		for_all_ports(nodep, portp) {
			if (sm_valid_port(portp) && (portp->state > IB_PORT_DOWN)) {
				IB_LOG_ERROR_FMT(__func__, "port %d lidsRouted= 0x%x", portp->index,
								 portp->portData->lidsRouted);
			}
		}
	}
}

uint8_t
sm_isMaster(void)
{
	return (sm_state == SM_STATE_MASTER || sm_state == SM_STATE_DISCOVERING);
}

uint8_t
sm_isReady(void)
{
	return (sm_state == SM_STATE_MASTER || sm_state == SM_STATE_STANDBY);
}

uint8_t
sm_isActive(void)
{
	return (sm_state != SM_STATE_NOTACTIVE);
}

uint8_t
sm_isDeactivated(void)
{
	return (sm_state == SM_STATE_NOTACTIVE && sm_prevState != SM_STATE_NOTACTIVE);
}

uint8_t
sm_isValidDeviceConfigSettings(uint32_t modid, uint32_t device, uint32_t port,
							   uint64_t portGuid)
{
	uint8_t valid = TRUE;

	if (device != sm_config.hca) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "Hfi %u does not match SM Hfi %u",
							device, sm_config.hca);
	}

	if (port != sm_config.port) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "Port %u does not match SM Port %u",
							port, sm_config.port);
	}

	if (portGuid != 0 && portGuid != sm_config.port_guid) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL,
							"PortGUID " FMT_U64 " does not match SM PortGUID " FMT_U64,
							portGuid, sm_config.port_guid);
	}

	if (!valid)
		IB_LOG_MOD_WARN0(modid,
						 "Configured Device parameters do not match SM Device parameters, using SM parameters");
	return valid;
}

void
sm_getDeviceConfigSettings(uint32_t * device, uint32_t * port, uint64_t * portGuid)
{
	if (device)
		*device = sm_config.hca;

	if (port)
		*port = sm_config.port;

	if (portGuid)
		*portGuid = sm_config.port_guid;
}

uint8_t
sm_isValidMasterConfigSettings(uint32_t modid, int priority, int elevated_priority)
{
	uint8_t valid = TRUE;

	if (priority != sm_config.priority) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "Priority %d does not match SM Priority %d",
							priority, sm_config.priority);
	}

	if (elevated_priority != sm_config.elevated_priority) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL,
							"ElevatedPriority %u does not match SM ElevatedPriority %u",
							elevated_priority, sm_config.elevated_priority);
	}

	if (!valid)
		IB_LOG_MOD_WARN0(modid,
						 "Configured parameters do not match SM parameters, using SM parameters");
	return valid;
}

void
sm_getMasterConfigSettings(uint32_t * priority, uint32_t * elevated_priority)
{
	if (priority)
		*priority = sm_config.priority;
	if (elevated_priority)
		*elevated_priority = sm_config.elevated_priority;
}

uint8_t
sm_isValidSubnetSizeConfigSetting(uint32_t modid, uint32_t subnet_size)
{
	uint8_t valid = TRUE;

	if (subnet_size != sm_config.subnet_size) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "Subnet Size %u does not match SM Subnet Size %u",
							subnet_size, sm_config.subnet_size);
	}
	return valid;
}

void
sm_getSubnetSizeConfigSetting(uint32_t * subnet_size)
{
	if (subnet_size)
		*subnet_size = sm_config.subnet_size;
}

#ifdef __LINUX__
void
sm_wait_ready(uint32_t modid)
{
	int reported = 0;
	for (;;) {
		if (sm_isReady()) {
			if ( ((sm_state == SM_STATE_MASTER) && (topology_passcount >= 1)) ||
				 (sm_state == SM_STATE_STANDBY) )
				//if master, wait for at least one good sweep
				//if standby, allow pm to come up so that it can send images needed for dbsync
				break;
		}

		if (!reported) {
			IB_LOG_MOD_INFINI_INFO0(modid,
									"Service waiting for SM to go into MASTER/STANDBY state");
			reported = 1;
		}
		// sleep for half a second.
		(void) vs_thread_sleep(VTIMER_1S / 2);
	}
	if (reported) {
		IB_LOG_MOD_INFINI_INFO0(modid, "SM now ready, service starting");
	}
}

uint8_t
sm_isValidCoreDumpConfigSettings(uint32_t modid, const char *limit, const char *dir)
{
	uint8_t valid = TRUE;
	uint64_t sm_rlimit = 0;
	uint64_t rlimit = 0;

	if (0 != vs_getCoreDumpLimit(sm_config.CoreDumpLimit, &sm_rlimit)
		|| 0 != vs_getCoreDumpLimit(limit, &rlimit)) {
		valid = FALSE;
		// unexpected
		IB_LOG_MOD_WARN_FMT(modid, NULL, "Invalid CoreDumpLimit");
	}
	if (sm_rlimit != rlimit) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "CoreDumpLimit %s does not match SM CoreDumpLimit %s",
							limit, sm_config.CoreDumpLimit);
	}
	if (sm_rlimit) {
		if (0 != strcmp(dir, sm_config.CoreDumpDir)) {
			valid = FALSE;
			IB_LOG_MOD_WARN_FMT(modid, NULL, "CoreDumpDir %s does not match SM CoreDumpDir %s",
								limit, sm_config.CoreDumpDir);
		}
	}
	if (!valid)
		IB_LOG_MOD_WARN0(modid,
						 "Configured Core Dump parameters do not match SM Core Dump parameters, using SM parameters");

	return valid;
}

uint8_t
sm_isValidLogConfigSettings(uint32_t modid, uint32_t log_level, uint32_t log_mode,
							uint32_t log_masks[VIEO_LAST_MOD_ID + 1], char *logFile,
							char *syslogFacility)
{
	uint8_t valid = TRUE;

	if (log_mode != sm_config.syslog_mode) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "LogMode %u does not match SM LogMode %u",
							log_mode, sm_config.syslog_mode);
	}
	// LogLevel will be typical case, only do more detailed check if match
	if (log_level != sm_log_level) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "LogLevel %u does not match SM LogLevel %u",
							log_level, sm_log_level);
	} else if (0 != memcmp(&sm_log_masks[0], &log_masks[0], sizeof(sm_log_masks))) {
		int i;

		valid = FALSE;
		for (i = 0; i <= VIEO_LAST_MOD_ID; i++) {
			if (log_masks[i] != sm_log_masks[i])
				IB_LOG_MOD_WARN_FMT(modid, NULL,
									"%s_LogMask 0x%x does not match SM %s_LogMask 0x%x",
									cs_log_get_module_name(i), log_masks[i],
									cs_log_get_module_name(i), sm_log_masks[i]);
		}
	}

	if (logFile && strcmp(sm_config.log_file, logFile) != 0) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL, "LogFile %s does not match SM LogFile %s",
							logFile, sm_config.log_file);
	}

	if (syslogFacility && strcmp(sm_config.syslog_facility, syslogFacility) != 0) {
		valid = FALSE;
		IB_LOG_MOD_WARN_FMT(modid, NULL,
							"SyslogFacility %s does not match SM SyslogFacility %s",
							syslogFacility, sm_config.syslog_facility);
	}

	if (!valid)
		IB_LOG_MOD_WARN0(modid,
						 "Configured log parameters do not match SM log parameters, using SM parameters");

	return valid;
}

void
sm_getLogConfigSettings(uint32_t * logLevel, uint32_t * logMode,
						uint32_t log_masks[VIEO_LAST_MOD_ID + 1], char *logFile,
						char *syslogFacility)
{
	if (logLevel)
		*logLevel = sm_log_level;

	if (logMode)
		*logMode = sm_config.syslog_mode;

	if (log_masks)
		memcpy(log_masks, sm_log_masks, sizeof(sm_log_masks));

	if (logFile)
		cs_strlcpy(logFile, sm_config.log_file, LOGFILE_SIZE);

	if (syslogFacility)
		cs_strlcpy(syslogFacility, sm_config.syslog_facility, STRING_SIZE);
}
#endif

void
Switch_Enqueue_Type(Topology_t * topop, Node_t * nodep, int tier, int checkName)
{

	Node_t *prevNodep = NULL;
	Node_t *curNodep = NULL;
	char *nodeDesc = sm_nodeDescString(nodep);
	int prefixLen = -1;
	int c, suffixNum, curSuffixNum;
	char *endptr = NULL;

	if (tier<0) tier=0;

	nodep->tier = tier + 1;

	if (nodep->tier < topop->minTier)
		topop->minTier = nodep->tier;
	if (nodep->tier > topop->maxTier)
		topop->maxTier = nodep->tier;

	if (topop->tier_head[tier] == NULL) {
		nodep->sw_next = NULL;
		nodep->sw_prev = NULL;
		topop->tier_head[tier] = nodep;
		topop->tier_tail[tier] = nodep;
		return;
	}

	if (checkName) {
		for (c = 0; c < strlen(nodeDesc); c++) {
			if ((nodeDesc[c] >= '0') && (nodeDesc[c] <= '9')) {
				if (c > 0) {
					prefixLen = c;
					suffixNum = strtoul(nodeDesc + prefixLen, NULL, 10);
					break;
				}
			}
		}
	}

	if (prefixLen > -1) {
		curNodep = topop->tier_head[tier];
		while (curNodep && (strncmp(sm_nodeDescString(curNodep), nodeDesc, prefixLen) < 0)) {
			prevNodep = curNodep;
			curNodep = curNodep->sw_next;
		}

		while (curNodep && (strncmp(sm_nodeDescString(curNodep), nodeDesc, prefixLen) == 0)) {
			curSuffixNum = strtoul(sm_nodeDescString(curNodep) + prefixLen, &endptr, 10);

			if (endptr == sm_nodeDescString(curNodep) + prefixLen) {
				// alpha prefix of node inserted is subset of curNodep alpha prefix.
				break;
			}

			if (suffixNum < curSuffixNum)
				break;

			prevNodep = curNodep;
			curNodep = curNodep->sw_next;
		}

	} else {
		curNodep = topop->tier_head[tier];
		while (curNodep && (strcmp(sm_nodeDescString(curNodep), sm_nodeDescString(nodep)) < 0)) {
			prevNodep = curNodep;
			curNodep = curNodep->sw_next;
		}
	}

	if (!prevNodep) {
		// head[tier]
		nodep->sw_prev = NULL;
		nodep->sw_next = topop->tier_head[tier];
		topop->tier_head[tier]->sw_prev = nodep;
		topop->tier_head[tier] = nodep;

	} else if (!curNodep) {
		// tail[tier]
		nodep->sw_next = NULL;
		nodep->sw_prev = topop->tier_tail[tier];
		topop->tier_tail[tier]->sw_next = nodep;
		topop->tier_tail[tier] = nodep;
	} else {
		// insert
		nodep->sw_next = prevNodep->sw_next;
		nodep->sw_prev = curNodep->sw_prev;
		curNodep->sw_prev = nodep;
		prevNodep->sw_next = nodep;
	}
}

/// Dummy address on stack indicating that CableInfo is not supported; isn't a valid CableInfo_t struct
static char dummy_ciNotSupported;

CableInfo_t *
sm_CableInfo_init(void)
{
	CableInfo_t *retVal;
	if (vs_pool_alloc(&sm_pool, sizeof(CableInfo_t), (void *) &retVal) != VSTATUS_OK) {
		vs_pool_free(&sm_pool, (void *) retVal);
		return NULL;
	}

	memset(retVal, 0, sizeof(CableInfo_t));
	retVal->refcount = 1;
	return retVal;
}

CableInfo_t *
sm_CableInfo_copy(CableInfo_t * ci)
{
	if (ci == (CableInfo_t *) & dummy_ciNotSupported)
		return ci;

	if (!ci || ci->refcount == UINT8_MAX)
		return NULL;

	ci->refcount++;
	return ci;
}

CableInfo_t *
sm_CableInfo_free(CableInfo_t * ci)
{
	if (!ci)
		return NULL;

	// hack; the dummy ci is not a real ci but is treated as one
	if (ci == (CableInfo_t *) & dummy_ciNotSupported)
		return ci;

	if (ci->refcount)
		ci->refcount--;

	if (!ci->refcount) {
		vs_pool_free(&sm_pool, (void *) ci);
		ci = NULL;
	}

	return ci;
}

boolean
sm_Port_t_IsCableInfoSupported(Port_t * port)
{
	return sm_valid_port(port) && port->index > 0 &&
		port->portData->portInfo.PortPhysConfig.s.PortType == STL_PORT_TYPE_STANDARD &&
		port->portData->cableInfo != (CableInfo_t *) & dummy_ciNotSupported;
}

void
sm_Port_t_SetCableInfoSupported(Port_t * port, boolean support)
{
	if (sm_valid_port(port)) {
		if (support && port->portData->cableInfo == (CableInfo_t *) & dummy_ciNotSupported) {
			port->portData->cableInfo = NULL;
		} else if (!support) {
			if (port->portData->cableInfo != NULL
				&& port->portData->cableInfo != (CableInfo_t *) & dummy_ciNotSupported) {
				sm_CableInfo_free(port->portData->cableInfo);
			}
			port->portData->cableInfo = (CableInfo_t *) & dummy_ciNotSupported;
		}
	}
}

// If new_pg is not found in pgp, adds it to pgp, updates length, and set
// index to the index of the new port group and returns 1. 
// If it *is* already found in pgp, index points to the existing group and
// returns 0. If new_pg cannot be added (because length == cap), returns -1.
int
sm_Push_Port_Group(STL_PORTMASK pgp[], STL_PORTMASK new_pg,
				   uint8_t * index, uint8_t * length, uint8_t cap)
{
	int rc = -1;
	uint8_t i;

	for (i = 0; i < *length; i++) {
		if (new_pg == pgp[i])
			break;
	}

	*index = i;
	if (i == *length) {
		// New entry.
		if (*length < cap) {
			*length = i + 1;
			rc = 1;
			pgp[i] = new_pg;
		}						// else... leave rc set to -1.
	} else if (i < *length) {
		// match found.
		rc = 0;
	}

	return rc;
}

// Warning: This assumes the switch has <=64 ports.
STL_PORTMASK
sm_Build_Port_Group(SwitchportToNextGuid_t * ordered_ports, int olen)
{
	int i;
	STL_PORTMASK mask = 0;

	for (i = 0; i < olen; i++) {
		Port_t *pp = ordered_ports[i].portp;
		mask += (1 << ((pp->index) % STL_PORT_MASK_WIDTH));
	}

	return mask;
}

int
smGetDgIdx(char* dgName)
{
	//loop on all DGs
	//strcmp dgName with dgp->name
	//if found, return index of device group

	int idxToReturn = -1;
	int dgIdx;
	DGConfig_t* dgp;
	int numGroups = dg_config.number_of_dgs;

	for (dgIdx=0; dgIdx<numGroups; ++dgIdx) {
		dgp = dg_config.dg[dgIdx];
		if (strcmp(dgp->name, dgName) == 0) {
			idxToReturn = dgIdx;
			break;
		}
	}

	if (idxToReturn == -1)
		IB_LOG_WARN_FMT(__func__, "DgIdx not found for device group %s", dgName);

	return idxToReturn;
}

boolean
isDgMember(int dgIdx, PortData_t* portDataPtr)
{
	boolean dgIdxFound = FALSE;

	int loopIdx;
	for (loopIdx=0; loopIdx<MAX_DEVGROUPS; ++loopIdx) {
		int index = portDataPtr->dgMemberList[loopIdx];
		if (index == dgIdx) {
			dgIdxFound = TRUE;
			break;
		}
		else if (index == 65535) {
			//no more DGs
			break;
		}
	}

	return dgIdxFound;
}

boolean convertWildcardedString(char* inStr, char* outStr, RegexBracketParseInfo_t* regexInfo)
{
	//Convert nodeDesc into regular expression syntax
	//replace * with: ([a-z0-9A-Z])*
	//replace ? with: ([a-z0-9A-Z])?
	//if nodeDesc doesn't start with *, insert ^ at front
	//if nodeDesc doesn't end with *, insert $ at the end
	char* starWildCard = "(([_,=.a-z0-9A-Z[:space:]-])*)";
	int lengthStarWildCard = strlen(starWildCard);
	char* quesWildCard = "(([_,=.a-z0-9A-Z[:space:]-])?)";
	int lengthQuesWildCard = strlen(quesWildCard);
	char* bracketSyntax = "([0-9]+)";
	int lengthBracketSyntax = strlen(bracketSyntax);

	boolean isSyntaxValid = TRUE;
	int inStrIdx = 0;
	int outStrIdx = 0;
	int inputLen = strlen(inStr);

	boolean firstBracketValid = TRUE;
	boolean firstTimeThroughLoop = TRUE;

	while (inStrIdx<inputLen) {

		//Enforce rules for first character
		if (inStrIdx == 0) {

			if (inStr[inStrIdx] == '?') {

				//walk buffer to find next non ?..if *, insert ^ (?s will be ignored below when we process the characters)
				boolean breakLoop=FALSE;
				int curIdx = inStrIdx+1;
				while (!breakLoop) {

					if (curIdx < inputLen) {

						if (inStr[curIdx] != '?') {

							if (inStr[inStrIdx] != '*') { 
								//append ^ character
								strcat(&outStr[outStrIdx], "^");
								outStrIdx++;
							}
							breakLoop=TRUE;
						}
						else
							curIdx++;
					}
					else
						breakLoop=TRUE;
				}

			}
			else if (inStr[inStrIdx] != '*') {
				//if first character not '*', append ^ character
				strcat(&outStr[outStrIdx], "^");
				outStrIdx++;
			}

			if (inStr[inStrIdx] == '[') {
				//if first character '[', make sure there is another valid character after the ']'
				//set firstBracketValid to FALSE until we process another valid character
				firstBracketValid = FALSE;
			}
      
		}

		//if not the first time through loop, and we process a valid character, set the firstBracketValid flag to TRUE
		if ( (!firstTimeThroughLoop) && 
			 ((inStr[inStrIdx] != '[') || (inStr[inStrIdx] != ':')) )
			firstBracketValid = TRUE;


		//Process characters
		if (inStr[inStrIdx] == '*') {

			//append starWildCard to outStr
			strcat(&outStr[outStrIdx], starWildCard);
			outStrIdx += lengthStarWildCard;
			regexInfo->totalGroups+=2;

			//look ahead till next non wildcard value.... ignore all '*' or '?'...if next non wildcard is '[', syntax error
			boolean breakLoop=FALSE;
			int curIdx = inStrIdx+1;
			while (!breakLoop) {
				if (curIdx < inputLen) {

					if ( (inStr[curIdx] == '*') || (inStr[curIdx] == '?') ) {
						//increment index past this
						inStrIdx++;
					}
					else if (inStr[curIdx] == '[') {
						isSyntaxValid = FALSE;
						break;
					}
					else
						breakLoop=TRUE;

					curIdx++;
				}
				else
					breakLoop=TRUE;
			}

		}
		else if (inStr[inStrIdx] == '?') {

			boolean skipQues = FALSE;

			//look ahead till next non ? value.... ignore if it is a '*'...if next non wildcard is '[', syntax error
			boolean breakLoop=FALSE;
			int curIdx = inStrIdx+1;
			while (!breakLoop) {

				if (curIdx < inputLen) {

					if (inStr[curIdx] != '?') {

						if (inStr[curIdx] == '*') {
							//increment index past this ?
							skipQues = TRUE;
						}
						else if (inStr[curIdx] == '[') {
							isSyntaxValid = FALSE;
						}

						breakLoop=TRUE;

					}
					else
						curIdx++;
				}
				else
					breakLoop=TRUE;
			}
	  
			if (!skipQues) {
				//append quesWildCard to outStr
				strcat(&outStr[outStrIdx], quesWildCard);
				outStrIdx += lengthQuesWildCard;
				regexInfo->totalGroups+=2;
			}

		}
		else if (inStr[inStrIdx] == '[') {

			boolean isBracketSyntaxValid = processBracketSyntax(&inStr[0], inputLen, &inStrIdx, regexInfo, FALSE);

			if (isBracketSyntaxValid) {
				//append bracketWildCard to outStr
				strcat(&outStr[outStrIdx], bracketSyntax);
				outStrIdx += lengthBracketSyntax;
			}
			else {
				isSyntaxValid = FALSE;
			}

		}
		else if (inStr[inStrIdx] == ':') {
			//:[##-##] must be last in input sequence
			//skip past the ':'
			inStrIdx++;

			//next character has to be '['.....else, error
			if (inStr[inStrIdx] == '[') {
	  
				isSyntaxValid = processBracketSyntax(&inStr[0], inputLen, &inStrIdx, regexInfo, TRUE);

				if (inStrIdx+1 < inputLen)
					isSyntaxValid = FALSE;
			}
			else {
				isSyntaxValid = FALSE;
			}

		}
		//handle '.'
		else if (inStr[inStrIdx] == '.') {
			strcat(&outStr[outStrIdx], "\\");
			outStrIdx++;
			strncpy(&outStr[outStrIdx], &inStr[inStrIdx],1);
			outStrIdx++;
		}
		else { //all other character inputs

			if (isCharValid(inStr[inStrIdx])) {
				//copy the character from the input string and increment outStrIdx
				strncpy(&outStr[outStrIdx], &inStr[inStrIdx],1);
				outStrIdx++;
			}
			else {
				isSyntaxValid = FALSE;
			}

		}

		if (!isSyntaxValid)
			break;

		//Enforce rules for last character
		if (inStrIdx == inputLen-1) { 

			//if not '*', append $
			if (inStr[inStrIdx] != '*') {
				//append $ character
				strcat(&outStr[outStrIdx], "$");
				outStrIdx++;
			}

		}

		//increment input string index
		inStrIdx++;

		firstTimeThroughLoop = FALSE;

	} //end processing input sequence

	//NUL terminate the outStr
	outStr[outStrIdx] = '\0';


	if (!firstBracketValid)
		isSyntaxValid = FALSE;

	return isSyntaxValid;
}


boolean isCharValid(char inChar) {

	boolean isValid = FALSE;

	if ( (inChar >= '0' && inChar  <= '9') ||
		 (inChar >= 'a' && inChar  <= 'z') ||
		 (inChar >= 'A' && inChar  <= 'Z') ||
		 (inChar == '-') ||
		 (inChar == ',') ||
		 (inChar == '=') ||
		 (inChar == '_') ||
		 (inChar == ' ') ||
		 (inChar == '.') ) {
		isValid = TRUE;
	}

	return isValid;
}


boolean processBracketSyntax(char* inputStr, int inputLen, int* curIdxPtr, RegexBracketParseInfo_t* regexInfo, boolean isPortRange) {

	if (!inputStr)
		return FALSE;

	boolean isSyntaxValid = TRUE;
	int maxNumDigits = MAX_DIGITS_TO_PROCESS + 1; //MAX_DIGITS_TO_PROCESS + '\0'

	//increment curIdx past the '['
	int curIdx = *curIdxPtr;
	curIdx++;

	//flow control variables
	boolean parsingNum1 = TRUE;
	boolean countingLeadingZeros = TRUE;

	//will use these to build the regular expression
	int leadingZeroNum1=0, leadingZeroNum2=0;
	int numDigitsNum1=0, numDigitsNum2=0;
	char* num1charPtr = NULL;
	char* num2charPtr = NULL;

	//loop to process the rest of the [##-##] syntax
	int tempNumDigits = 0;
	int tempLeading0s = 0;
	while (curIdx<inputLen) {

		//look for "-" character
		if ( (parsingNum1) && (inputStr[curIdx] == '-') ) {

			//end of 1st number, switch to start parsing the 2nd number
			parsingNum1 = FALSE;
			leadingZeroNum1 = tempLeading0s;
			numDigitsNum1 = tempNumDigits;

			//handle a range that starts with 0
			if ( (numDigitsNum1 == 0) && (leadingZeroNum1==1) ) {
				//set num1charPtr 
				num1charPtr = &inputStr[curIdx-1];

				numDigitsNum1 = 1;
				leadingZeroNum1 = 0;
			}

			//transition to start process 2nd number
			tempLeading0s = 0;
			tempNumDigits = 0;
			curIdx++;
			countingLeadingZeros = TRUE;
		}
		else if ( (!parsingNum1) && (inputStr[curIdx] == ']') ) {
			//end of valid bracket expression
			leadingZeroNum2 = tempLeading0s;
			numDigitsNum2 = tempNumDigits;

			//handle a range that starts with 0
			if ( (numDigitsNum2 == 0) && (leadingZeroNum2==1) ) {
				numDigitsNum2 = 1;
				leadingZeroNum2 = 0;

				//set num2charPtr 
				num2charPtr = &inputStr[curIdx-1];
			}

			//update current index pointer
			*curIdxPtr = curIdx;

			//validate number information to this point
			if ( (numDigitsNum1 <= 0) || 
				 (numDigitsNum2 <= 0) ||
				 (numDigitsNum1>maxNumDigits-1) || 
				 (numDigitsNum2>maxNumDigits-1) ||
				 (numDigitsNum1 > numDigitsNum2) ||
				 ((numDigitsNum1==numDigitsNum2) && (leadingZeroNum1 != leadingZeroNum2)) ) {
				isSyntaxValid = FALSE;
			}

			break;
		}
		else if (inputStr[curIdx] >= '0' &&inputStr[curIdx]  <= '9') {

			//count number of leading 0s
			if ( (inputStr[curIdx] == '0') && (countingLeadingZeros) ) {
				tempLeading0s++;
			}
			else {
				//if first non-zero number
				if (tempNumDigits == 0) {
					countingLeadingZeros = FALSE;

					if (parsingNum1)
						num1charPtr = &inputStr[curIdx];
					else
						num2charPtr = &inputStr[curIdx];
				}

				tempNumDigits++;
			}

			curIdx++;

		}
		else {
			//if any other character, input is invalid
			isSyntaxValid = FALSE;
			break;
		}
	} //end loop through input syntax

	// Sanity check to prevent null pointers from passing into the next section.
	isSyntaxValid = isSyntaxValid && (num1charPtr != NULL) && (num2charPtr != NULL);

	//translate numbers to ints and save them off
	if (isSyntaxValid) {

		//convert 1st number
		int size = numDigitsNum1+1;
		char firstNum[maxNumDigits];
		firstNum[size-1] = '\0';
		strncpy(&firstNum[0], num1charPtr, numDigitsNum1);
		int num1 = atoi(&firstNum[0]);

		//convert 2nd number
		size = numDigitsNum2+1;
		char secondNum[maxNumDigits];
		secondNum[size-1] = '\0';
		strncpy(&secondNum[0], num2charPtr, numDigitsNum2);
		int num2 = atoi(&secondNum[0]);

		//validate num1 isn't larger than num2
		if (num1 > num2)
			isSyntaxValid = FALSE;

		//set regexInfo fields
		regexInfo->totalGroups++;

		if (isPortRange) {
			regexInfo->portRangeDefined = TRUE;
			regexInfo->portNum1 = num1;
			regexInfo->portNum2 = num2;
			regexInfo->lead0sPort1 = leadingZeroNum1;
			regexInfo->lead0sPort2 = leadingZeroNum2;
		}
		else {
			regexInfo->numBracketRangesDefined++;

			int idx = regexInfo->numBracketRangesDefined-1;
			regexInfo->number1[idx] = num1;
			regexInfo->numDigitsNum1[idx] = numDigitsNum1;
			regexInfo->number2[idx] = num2;
			regexInfo->numDigitsNum2[idx] = numDigitsNum2;
			regexInfo->leading0sNum1[idx] = leadingZeroNum1;
			regexInfo->leading0sNum2[idx] = leadingZeroNum2;
			regexInfo->bracketGroupNum[idx] = regexInfo->totalGroups;
		}

	}

	return isSyntaxValid;

}


boolean isNumberInRange(char* number, RegexBracketParseInfo_t* regexInfo, int bracketIdx) {

	boolean isValid = TRUE;
	boolean isInRange = FALSE;

	//local/temp and flow control variables
	int numDigits = 0;
	int numberLeading0s = 0;
	int curIdx = 0;
	int inputLen = strlen(number);

	boolean countingLeadingZeros = TRUE;
	char* charPtr = NULL;

	//convert number from char* to int (count leading 0s)
	while (curIdx<inputLen) {
    
		//verify this index is a valid number
		if (number[curIdx] >= '0' &&number[curIdx]  <= '9') {

			//count number of leading 0s
			if ( (number[curIdx] == '0') && (countingLeadingZeros) )
				numberLeading0s++;
			else {

				if (numDigits == 0) {
					countingLeadingZeros = FALSE;
					charPtr = &number[curIdx];
				}

				numDigits++;
			}

			curIdx++;
		}
		else {
			isValid = FALSE;
			break;
		}
	}

	if (isValid) {
		int maxNumDigits = MAX_DIGITS_TO_PROCESS + 1; //MAX_DIGITS_TO_PROCESS + '\0'
		char numArray[maxNumDigits];

		int size = numDigits+1;
		numArray[size-1] = '\0';
		strncpy(&numArray[0], charPtr, numDigits);
		int numberToEvalute = atoi(&numArray[0]);

		if ( (regexInfo->leading0sNum1[bracketIdx] == 0) && (regexInfo->leading0sNum1[bracketIdx] == 0) && (numberLeading0s == 0) ) {
			//no leading zeros, just do simple comparison
			if ( (numberToEvalute >= regexInfo->number1[bracketIdx]) && (numberToEvalute <= regexInfo->number2[bracketIdx]) )
				isInRange = TRUE;
		}
		else {
			//evaluate leading 0s
			int numToEvalDigitsDiff = numDigits - regexInfo->numDigitsNum1[bracketIdx];
			int num0sToExpect = regexInfo->leading0sNum1[bracketIdx] - numToEvalDigitsDiff;

			if (num0sToExpect == numberLeading0s) {
				//finish evaluation
				if ( (numberToEvalute >= regexInfo->number1[bracketIdx]) && (numberToEvalute <= regexInfo->number2[bracketIdx]) )
					isInRange = TRUE;
			}
		}

	}

	return isInRange;
}


void initializeRegexStruct(RegExp_t* regExprPtr) {

	regExprPtr->isSyntaxValid = FALSE;

	//clear regexString
	memset(regExprPtr->regexString, 0, sizeof(char)*MAX_NODE_DESC_REG_EXPR);

	//initialize RegexBracketParseInfo_t info
	RegexBracketParseInfo_t* regexInfo = &regExprPtr->regexInfo;
	regexInfo->totalGroups = 0;

	//clear port range fields
	regexInfo->portRangeDefined = FALSE;
	regexInfo->portNum1 = 0;
	regexInfo->portNum2 = 0;
	regexInfo->lead0sPort1 = 0;
	regexInfo->lead0sPort2 = 0;

	//clear number range bracket fields
	regexInfo->numBracketRangesDefined = 0;

	int idx;
	for (idx=0; idx<MAX_BRACKETS_SUPPORTED; idx++) {
		regexInfo->number1[idx] = 0;
		regexInfo->number2[idx] = 0;
		regexInfo->leading0sNum1[idx] = 0;
		regexInfo->leading0sNum2[idx] = 0;
		regexInfo->bracketGroupNum[idx] = 0;
	}
}

void printRegexStruct(RegexBracketParseInfo_t* regexInfo) {

	if ( (regexInfo->portRangeDefined == FALSE) && (regexInfo->numBracketRangesDefined == 0) ) {
		printf("Regex Info Struct: No port ranges or brackets defined\n");
	}
	else {
		printf("Regex Info Struct:\n");

		int idx;
		for (idx=0; idx<regexInfo->numBracketRangesDefined; idx++) {
			printf("Bracket Range Info %d:\n", idx);
			printf("             Num1: %d\n", regexInfo->number1[idx]);
			printf("             Num2: %d\n", regexInfo->number2[idx]);
			printf("    Leading0sNum1: %d\n", regexInfo->leading0sNum1[idx]);
			printf("    Leading0sNum1: %d\n", regexInfo->leading0sNum2[idx]);
			printf("         GroupNum: %d\n", regexInfo->bracketGroupNum[idx]);
		}

		if (regexInfo->portRangeDefined == TRUE) {
			printf("Port Range Info:\n");
			printf("         PortNum1: %d\n", regexInfo->portNum1);
			printf("         PortNum2: %d\n", regexInfo->portNum2);
			printf("    Leading0sNum1: %d\n", regexInfo->lead0sPort1);
			printf("    Leading0sNum2: %d\n", regexInfo->lead0sPort2);
		}
	}

}

struct _PortDataVLArb * sm_port_getNewArb(Port_t * portp)
{
	if (!sm_valid_port(portp))
		return NULL;

	if (!portp->portData->newArb) {
		vs_pool_alloc(&sm_pool, sizeof(struct _PortDataVLArb), (void*)&portp->portData->newArb);
	}

	return portp->portData->newArb;
}

void sm_port_releaseNewArb(Port_t * portp)
{
	if (!sm_valid_port(portp) || !portp->portData->newArb)
		return;
	vs_pool_free(&sm_pool, portp->portData->newArb);
	portp->portData->newArb = NULL;
}

boolean sm_config_valid(FMXmlCompositeConfig_t *xml_config, VirtualFabrics_t* newVfPtr, VirtualFabrics_t* oldVfPtr)
{
	boolean configValid = TRUE;
	
	//check that non-changable parameters haven't changed

	if ( (sm_config.disruptive_checksum != xml_config->fm_instance[sm_instance]->sm_config.disruptive_checksum) ||
		 (oldVfPtr->disruptive_checksum != newVfPtr->disruptive_checksum) ) {

		IB_LOG_WARN0("SM: Disruptive checksum mismatch for sm_config_checksum_match(); ignoring reconfigure request");
		//TODO: Improve the level of detail in the printout to give user more information about the checksum failure

		configValid = FALSE;
	}
	else {
		//Possible further validation required for parameters that changed before we apply?
	}

	return configValid;
}

boolean pm_config_valid(FMXmlCompositeConfig_t *xml_config)
{
	boolean configValid = TRUE;

	//check that non-changable parameters haven't changed
	if (pm_config.disruptive_checksum != xml_config->fm_instance[sm_instance]->pm_config.disruptive_checksum) {

		IB_LOG_WARN0("SM: Disruptive checksum mismatch for pm_config_checksum_match(); ignoring reconfigure request");
		//TODO: Improve the level of detail in the printout to give user more information about the checksum failure

		configValid = FALSE;
	}
	else {
		//Possible further validation required for parameters that changed before we apply?
	}

	return configValid;
}


Status_t
sm_enable_port_led(Node_t *nodep, Port_t *portp, boolean enabled) 
{
	STL_LED_INFO ledInfo;
	Status_t status;
	uint32_t amod;
	STL_LID dlid;

	IB_ENTER(__func__, nodep, portp, enabled, 0);

	//Do not send LedInfo to switch port0.
	if((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
		&& (portp->index == 0))
		return(VSTATUS_OK);


	memset(&ledInfo, 0, sizeof(STL_LED_INFO));
	ledInfo.u.s.LedMask = portp->portData->portInfo.PortStates.s.LEDEnabled;

	if(enabled == ledInfo.u.s.LedMask) {
		//no set needed, return to avoid sending uneeded MADs
		return VSTATUS_OK;
	}

	amod = portp->index;
	ledInfo.u.s.LedMask = enabled;
	// When Disabling an LED, we assume the port is working correctly, and thus reachable by LR MADs. 
	// When Enabling an LED, the port likely has been removed from routing tables and thus we must use DR MADs

	if(FALSE == enabled) {
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			Port_t *swportp;
			swportp = sm_get_port(nodep, 0);
			if (!sm_valid_port(swportp)) {
				IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
								nodep->nodeInfo.NodeGUID);
				return VSTATUS_BAD;
			}
			dlid = swportp->portData->lid;
		} else {
			dlid = portp->portData->lid;
		}
		status = SM_Set_LedInfo_LR(fd_topology, (1 << 24) | amod, sm_lid, dlid, &ledInfo, 
								(portp->portData->portInfo.M_Key == 0) ? 
								sm_config.mkey : portp->portData->portInfo.M_Key);
	} else {
		status = SM_Set_LedInfo(fd_topology, (1 << 24) | amod, nodep->path, &ledInfo, 
							(portp->portData->portInfo.M_Key == 0) ? 
							sm_config.mkey : portp->portData->portInfo.M_Key);
	}

	if(status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Set(LedInfo) failed. rc: %d ", status);
		IB_EXIT(__func__, status);
		return (status);
	}

	//save LedInfo
	portp->portData->portInfo.PortStates.s.LEDEnabled = ledInfo.u.s.LedMask;
	return VSTATUS_OK;
	IB_EXIT(__func__, nodep);
}


Status_t 
sm_set_linkinit_reason(Node_t *nodep, Port_t *portp, uint8_t initReason)
{
	STL_PORT_INFO portInfo;
	Status_t status;
	uint32_t amod;
	uint8_t *path;

	IB_ENTER(__func__, nodep, portp, initReason, 0);

	path = nodep->path;

	portInfo = portp->portData->portInfo;
	if(portInfo.s3.LinkInitReason == initReason){
		// local copy of linkInitReason already set, assume port has correct value
		return VSTATUS_OK;
	} 

	//Get(PortInfo) for port may not have occured yet, so get the latest to make sure
	//we set correctly
	amod = portp->index;

	status = SM_Get_PortInfo(fd_topology, (1 << 24) | amod, path, &portInfo);
	if(status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Get(PortInfo) failed. rc: %d", status);
		IB_EXIT(__func__, status);
		return (status);
	}

	portInfo.s3.LinkInitReason = initReason;

	//
	//set "No Change" attributes
	//
	portInfo.LinkSpeed.Enabled = 0;
	portInfo.LinkWidth.Enabled = 0;
	portInfo.PortStates.s.PortPhysicalState = 0;
	portInfo.s4.OperationalVL = 0;

	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
	status = SM_Set_PortInfo(fd_topology, (1 << 24) | amod, &addr, &portInfo,
		(portp->portData->portInfo.M_Key == 0) ? sm_config.mkey : portp->portData->portInfo.M_Key);
	if(status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Unable to set LinkInitReason; Set(PortInfo) failed. rc: %d ", status);
		IB_EXIT(__func__, status);
		return (status);
	}

	//save portInfo
	portp->portData->portInfo = portInfo;

	return VSTATUS_OK;
	IB_EXIT(__func__, nodep);
}



Status_t
sm_select_path_lids
	( Topology_t *topop
	, Port_t *srcPortp, STL_LID slid
	, Port_t *dstPortp, STL_LID dlid
	, STL_LID *outSrcLids, uint8_t *outSrcLen
	, STL_LID *outDstLids, uint8_t *outDstLen
	)
{
	int i;
	int srcLids, dstLids;

	srcLids = 1 << srcPortp->portData->lmc;
	dstLids = 1 << dstPortp->portData->lmc;

	if (slid == STL_LID_PERMISSIVE && dlid == STL_LID_PERMISSIVE) {
		*outSrcLen = srcLids;
		for (i = 0; i < srcLids; ++i)
			outSrcLids[i] = srcPortp->portData->lid + i;
		*outDstLen = dstLids;
		for (i = 0; i < dstLids; ++i)
			outDstLids[i] = dstPortp->portData->lid + i;
	} else if (slid == STL_LID_PERMISSIVE) {
		*outSrcLen = srcLids;
		for (i = 0; i < srcLids; ++i)
			outSrcLids[i] = srcPortp->portData->lid + i;
		*outDstLen = 1;
		outDstLids[0] = dlid;
	} else if (dlid == STL_LID_PERMISSIVE) {
		*outSrcLen = 1;
		outSrcLids[0] = slid;
		*outDstLen = dstLids;
		for (i = 0; i < dstLids; ++i)
			outDstLids[i] = dstPortp->portData->lid + i;
	} else {
		*outSrcLen = 1;
		outSrcLids[0] = slid;
		*outDstLen = 1;
		outDstLids[0] = dlid;
	}

	return VSTATUS_OK;
}


// Mark all links connected to a switch as down
void 
sm_mark_switch_down(Topology_t *topop, Node_t *swnodep)
{
	Port_t *portp;
	for_all_ports(swnodep, portp) {
		sm_mark_link_down(topop, portp);
	}
	return;
}


// Marks both sides of a link down in the topology.
// Does not affect the actual device port state.
//
// Ensures that:
//  - Port_t.state is IB_PORT_DOWN
//  - the port is removed from its node's init and active bitsets
//  - associated counters are decremented
//  - the neighbor, if avaiable, is also marked down
//
void
sm_mark_link_down(Topology_t *topop, Port_t *portp)
{
	if (!portp) return;

	if (portp->state > IB_PORT_DOWN) {
		portp->state = IB_PORT_DOWN;
		if (portp->portData) {
			Node_t * nodep = portp->portData->nodePtr;
			DEBUG_ASSERT(nodep);
			if (sm_debug)
				IB_LOG_INFINI_INFO_FMT(__func__, 
					"marking node %s nodeGuid "FMT_U64" port %u down",
					sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
			if (bitset_test(&nodep->activePorts, portp->index)) {
				bitset_clear(&nodep->activePorts, portp->index);
				DECR_PORT_COUNT(topop, nodep);
			}
			if (bitset_test(&nodep->initPorts, portp->index)) {
				bitset_clear(&nodep->initPorts, portp->index);
				nodep->portsInInit = nodep->portsInInit ? nodep->portsInInit - 1 : 0;
			}
		}
	}
		
	Port_t *nportp = sm_find_port(topop, portp->nodeno, portp->portno);
	if (nportp && nportp->state > IB_PORT_DOWN) {
		nportp->state = IB_PORT_DOWN;
		if (nportp->portData) {
			Node_t * nnodep = nportp->portData->nodePtr;
			DEBUG_ASSERT(nnodep);
			if (sm_debug)
				IB_LOG_INFINI_INFO_FMT(__func__, 
					"marking node %s nodeGuid "FMT_U64" port %u down",
					sm_nodeDescString(nnodep), nnodep->nodeInfo.NodeGUID, nportp->index);
			if (bitset_test(&nnodep->activePorts, nportp->index)) {
				bitset_clear(&nnodep->activePorts, nportp->index);
				DECR_PORT_COUNT(topop, nnodep);
			}
			if (bitset_test(&nnodep->initPorts, nportp->index)) {
				bitset_clear(&nnodep->initPorts, nportp->index);
				nnodep->portsInInit = nnodep->portsInInit ? nnodep->portsInInit - 1 : 0;
			}
		}
	}
}

void
sm_copyPortDataSCSCMap(Port_t *sportp, Port_t *dportp, int extended) {
	LIST_ITEM *p;
	PortDataSCSCMapPortMask *pSCSC, *pSCSC2;
	Status_t status;

	if (!sportp || !sportp->portData || !dportp || !dportp->portData)
		return;

	for (p = QListHead(&sportp->portData->scscMapList[extended]); p != NULL; p = QListNext(&sportp->portData->scscMapList[extended], p)) {
		pSCSC = (PortDataSCSCMapPortMask *)QListObj(p);

		status = vs_pool_alloc(&sm_pool, sizeof(PortDataSCSCMapPortMask), (void *)&pSCSC2);
		if (status != FSUCCESS) {
			IB_LOG_ERRORRC("Unable to allocate portMask memory rc: ", status);
			return;
		}

		ListItemInitState(&pSCSC2->SCSCMapItem);
		QListSetObj(&pSCSC2->SCSCMapItem, pSCSC2);

		status = vs_pool_alloc(&sm_pool, sizeof(STL_SCSCMAP), (void *)&pSCSC2->SCSCMap);
		if (status != FSUCCESS) {
			//memory error
			IB_LOG_ERRORRC("Unable to allocate SCSCMap memory rc: ", status);
			return;
		}
		QListInsertTail(&dportp->portData->scscMapList[extended], &pSCSC2->SCSCMapItem);

		memcpy(pSCSC2->SCSCMap, pSCSC->SCSCMap, sizeof(STL_SCSCMAP));
		memcpy(&pSCSC2->outports, &pSCSC->outports, sizeof(STL_PORTMASK) * STL_MAX_PORTMASK);
	}
}

// if a matching SCSC table already exists, add egress to its port mask
// otherwise, create a new entry in the scscmap list for this table
void
sm_addPortDataSCSCMap(Port_t *portp, uint8_t outport,  int extended, const STL_SCSCMAP *pSCSC) {
	LIST_ITEM *p;
	PortDataSCSCMapPortMask *pSCSC2;
	PortDataSCSCMapPortMask *pEmptySCSC2 = NULL;
	int entryFound = 0;
	Status_t status;

	if (!portp || !portp->portData)
		return;

	for (p = QListHead(&portp->portData->scscMapList[extended]); p != NULL; p = QListNext(&portp->portData->scscMapList[extended], p)) {
		pSCSC2 = (PortDataSCSCMapPortMask *)QListObj(p);
		if (!memcmp(pSCSC2->SCSCMap, pSCSC, sizeof(STL_SCSCMAP))) {
			StlAddPortToPortMask(pSCSC2->outports, outport);
			entryFound = 1;
		} else if (StlIsPortInPortMask(pSCSC2->outports, outport)) {
			// the maps don't match but the port does, remove & replace with new map
			StlClearPortInPortMask(pSCSC2->outports, outport);
			if (StlNumPortsSetInPortMask(pSCSC2->outports, portp->portData->nodePtr->nodeInfo.NumPorts) == 0) {
				pEmptySCSC2 = pSCSC2;
			}
		}
	}

	if (entryFound) {
		if (pEmptySCSC2) {
			QListRemoveItem(&portp->portData->scscMapList[extended], &pEmptySCSC2->SCSCMapItem);
		}
		return;
	}

	pSCSC2 = pEmptySCSC2;
	if (!pSCSC2) {
		// never found matching map, create new one
		status = vs_pool_alloc(&sm_pool, sizeof(PortDataSCSCMapPortMask), (void *)&pSCSC2);
		if (status != FSUCCESS) {
			// memory error
			IB_LOG_ERRORRC("Unable to allocate portMask memory rc: ", status); 
			return;
		}

		ListItemInitState(&pSCSC2->SCSCMapItem);
		QListSetObj(&pSCSC2->SCSCMapItem, pSCSC2);

		status = vs_pool_alloc(&sm_pool, sizeof(STL_SCSCMAP), (void *)&pSCSC2->SCSCMap);
		if (status != FSUCCESS) {
			// memory error
			IB_LOG_ERRORRC("Unable to allocate SCSCMap memory rc: ", status);
			return;
		}
		QListInsertTail(&portp->portData->scscMapList[extended], &pSCSC2->SCSCMapItem);
	}
	memcpy(pSCSC2->SCSCMap, pSCSC, sizeof(STL_SCSCMAP));
	StlAddPortToPortMask(pSCSC2->outports, outport);
}

STL_SCSCMAP*
sm_lookupPortDataSCSCMap(Port_t *portp, uint8_t outport, int extended) {
	LIST_ITEM *p;
	PortDataSCSCMapPortMask *pSCSC;

	if (!portp || !portp->portData) {
		return NULL;
	}

	for (p = QListHead(&portp->portData->scscMapList[extended]); p != NULL; p = QListNext(&portp->portData->scscMapList[extended], p)) {
		pSCSC = (PortDataSCSCMapPortMask *)QListObj(p);

		if (StlIsPortInPortMask(pSCSC->outports, outport))
			return (pSCSC->SCSCMap);
	}
	return NULL;
}
