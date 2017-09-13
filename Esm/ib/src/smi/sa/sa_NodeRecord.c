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
//
// FILE NAME
//    sa_NodeRecord.c
//
// DESCRIPTION
//    This file contains the routines to process the SA requests for
//    records of the NodeRecord type.
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    sa_NodeRecord
//
// DEPENDENCIES
//    ib_mad.h
//    ib_status.h
//
//===========================================================================//


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "iba/stl_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "stl_print.h"

static Status_t sa_NodeRecord_GetTable(Mai_t *, uint32_t *, SACacheEntry_t **);
static Status_t sa_IbNodeRecord_GetTable(Mai_t *, uint32_t *);
static uint32_t sa_NodeRecord_Matches(uint8_t ** data, STL_SA_MAD * samad, Node_t * nodep,
							   		  STL_NODE_RECORD * nr, uint32_t bytes);
static uint32_t sa_IbNodeRecord_Matches(uint8_t ** data, STL_SA_MAD * samad, Node_t * nodep,
							   		  IB_NODE_RECORD * nr, uint32_t bytes);
static Status_t sa_NodeRecord_CheckCache(STL_SA_MAD * samad, STL_NODE_RECORD * nr,
								  		 SACacheEntry_t ** outCache);

#define NO_MEM_RECORDS 0xFFFFFFFF

Status_t
sa_NodeRecord(Mai_t * maip, sa_cntxt_t * sa_cntxt)
{
	uint32_t records;
	uint32_t attribOffset;
	SACacheEntry_t *cache = NULL;

	IB_ENTER("sa_NodeRecord", maip, 0, 0, 0);

	//
	//  Assume failure.
	//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetNodeRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblNodeRecord);
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
		sa_NodeRecord_GetTable(maip, &records, &cache);
	} else if (maip->base.bversion == IB_BASE_VERSION && maip->base.cversion == SA_MAD_CVERSION) {
		sa_IbNodeRecord_GetTable(maip, &records);
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
	//  Determine reply status
	//
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_NodeRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	if (maip->base.cversion == STL_SA_CLASS_VERSION)  {
		attribOffset = sizeof(STL_NODE_RECORD) + Calculate_Padding(sizeof(STL_NODE_RECORD));
	} else {
		attribOffset = sizeof(IB_NODE_RECORD) + Calculate_Padding(sizeof(IB_NODE_RECORD));
	}

	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data_cached(sa_cntxt, sa_data, records * attribOffset, cache);

	// transient? we don't need it anymore
	if (cache && cache->transient)
		sa_cache_release(cache);

	(void) sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_NodeRecord", VSTATUS_OK);
	return (VSTATUS_OK);
}

static Status_t
sa_NodeRecord_Set(uint8_t * cp, Node_t * nodep, Port_t * portp)
{
	Lid_t lid;
	uint32_t portno;
	Port_t *nrPortp;
	STL_NODE_RECORD nodeRecord = {{0}};

	IB_ENTER("sa_NodeRecord_Set", cp, nodep, portp, 0);

	if (portp == NULL) {
		IB_LOG_ERROR_FMT("sa_NodeRecord_Set",
						 "NULL port parameter for Node Guid[" FMT_U64 "], %s",
						 nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_NodeRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	portno = (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : portp->index;

	if (!sm_valid_port((nrPortp = sm_get_port(nodep, portno)))) {
		IB_LOG_ERROR_FMT("sa_NodeRecord_Set",
						 "failed to get port %d for Node Guid[" FMT_U64 "], %s",
						 portno, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_NodeRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	lid = nrPortp->portData->lid;

	nodeRecord.RID.LID = lid;
	memcpy((void *) &nodeRecord.NodeDesc, (void *) &nodep->nodeDesc,
		   sizeof(nodeRecord.NodeDesc));
	nodeRecord.NodeDesc.NodeString[STL_NODE_DESCRIPTION_ARRAY_SIZE-1]=0;
	nodeRecord.NodeInfo = nodep->nodeInfo;
	nodeRecord.NodeInfo.u1.s.LocalPortNum = nodep->nodeInfo.u1.s.LocalPortNum;

	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		nodeRecord.NodeInfo.PortGUID = portp->portData->guid;
		if (!portp->portData->guid) {
			IB_LOG_ERROR_FMT("sa_NodeRecord_Set",
							 "NULL PORTGUID for Node Guid[" FMT_U64 "], %s, Lid 0x%x",
							 nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), lid);
		}
	}

	BSWAPCOPY_STL_NODE_RECORD(&nodeRecord, (STL_NODE_RECORD *) cp);

	IB_EXIT("sa_NodeRecord_Set", VSTATUS_OK);
	return (VSTATUS_OK);
}

static Status_t
sa_IbNodeRecord_Set(uint8_t * cp, Node_t * nodep, Port_t * portp)
{
	Lid_t lid;
	uint32_t portno;
	Port_t *nrPortp;
	IB_NODE_RECORD *ibNodeRecord = (IB_NODE_RECORD *)cp;

	IB_ENTER("sa_IbNodeRecord_Set", cp, nodep, portp, 0);

	if (cp == NULL) {
		IB_LOG_ERROR_FMT("sa_IbNodeRecord_Set", "NULL destination pointer.");
		IB_EXIT("sa_IbNodeRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	} else if (portp == NULL) {
		IB_LOG_ERROR_FMT("sa_IbNodeRecord_Set",
						 "NULL port parameter for Node Guid[" FMT_U64 "], %s",
						 nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_IbNodeRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	portno = (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : portp->index;

	if (!sm_valid_port((nrPortp = sm_get_port(nodep, portno)))) {
		IB_LOG_ERROR_FMT("sa_IbNodeRecord_Set",
						 "failed to get port %d for Node Guid[" FMT_U64 "], %s",
						 portno, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_IbNodeRecord_Set", VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	lid = nrPortp->portData->lid;

	//
	// Map STL data to IB record.
	//
	ibNodeRecord->RID.s.LID = lid;
	ibNodeRecord->RID.s.Reserved = 0;
	ibNodeRecord->RID.AsReg32 = ntoh32(ibNodeRecord->RID.AsReg32);
	ibNodeRecord->NodeInfoData.BaseVersion = IB_BASE_VERSION;
	ibNodeRecord->NodeInfoData.ClassVersion = IB_SUBN_ADM_CLASS_VERSION;
	ibNodeRecord->NodeInfoData.NodeType = nodep->nodeInfo.NodeType;
	ibNodeRecord->NodeInfoData.NumPorts = nodep->nodeInfo.NumPorts;
	ibNodeRecord->NodeInfoData.SystemImageGUID = ntoh64(nodep->nodeInfo.SystemImageGUID);
	ibNodeRecord->NodeInfoData.NodeGUID = ntoh64(nodep->nodeInfo.NodeGUID);
	ibNodeRecord->NodeInfoData.PortGUID = ntoh64(nodep->nodeInfo.PortGUID);
	ibNodeRecord->NodeInfoData.PartitionCap = ntoh16(nodep->nodeInfo.PartitionCap);
	ibNodeRecord->NodeInfoData.DeviceID = ntoh16(nodep->nodeInfo.DeviceID);
	ibNodeRecord->NodeInfoData.Revision = ntoh32(nodep->nodeInfo.Revision);
	ibNodeRecord->NodeInfoData.u1.s.LocalPortNum = portp->index;
	ibNodeRecord->NodeInfoData.u1.s.VendorID = nodep->nodeInfo.u1.s.VendorID;
	ibNodeRecord->NodeInfoData.u1.AsReg32 = ntoh32(ibNodeRecord->NodeInfoData.u1.AsReg32);

	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		ibNodeRecord->NodeInfoData.PortGUID = ntoh64(portp->portData->guid);
		if (!portp->portData->guid) {
			IB_LOG_ERROR_FMT("sa_IbNodeRecord_Set",
							 "NULL PORTGUID for Node Guid[" FMT_U64 "], %s, Lid 0x%x",
							 nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), lid);
		}
	}
	
	memcpy((void *) &ibNodeRecord->NodeDescData, (void *) &nodep->nodeDesc,
		   sizeof(NODE_DESCRIPTION));

	IB_EXIT("sa_IbNodeRecord_Set", VSTATUS_OK);
	return (VSTATUS_OK);
}

static Status_t
sa_NodeRecord_GetTable(Mai_t * maip, uint32_t * records, SACacheEntry_t ** outCache)
{
	uint8_t *data;
	uint32_t bytes;
	Node_t *nodep;
	STL_SA_MAD samad;
	Status_t status;
	STL_NODE_RECORD nodeRecord;
	uint32_t newRecords;
	Lid_t lid = 0;
	Port_t *portp;

	IB_ENTER("sa_NodeRecord_GetTable", maip, records, 0, 0);

	*outCache = NULL;
	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_NODE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_NODE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_NODE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_NODE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_NodeRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(STL_NODE_RECORD));

//
//  Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_NodeRecord_GetTable", status);
		return (status);
	}
//
//  Find all NodeRecords which match the template.
//
	(void) vs_rdlock(&old_topology_lock);
	/* 
	 * Return busy if source lid not in topology yet 
	 * This should throttle back the host
	 */
	lid = maip->addrInfo.slid;
	if ((portp = sm_find_active_port_lid(&old_topology, lid)) == NULL) {
		maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
		(void) vs_rwunlock(&old_topology_lock);
		IB_EXIT("sa_NodeRecord_GetTable", VSTATUS_OK);
		return (VSTATUS_OK);
	}

	BSWAPCOPY_STL_NODE_RECORD((STL_NODE_RECORD *) samad.data, &nodeRecord);

	// try to obtain data from cache first
	(void) sa_NodeRecord_CheckCache(&samad, &nodeRecord, outCache);


	if (*outCache) {
		*records = (*outCache)->records;
	} else if (!samad.header.mask) {
		/* doing for all nodes */
		// IB_LOG_INFINI_INFOLX("sa_NodeRecord_GetTable: Getting all the Node Records, mask=",
		// samad.mask);
		for_all_nodes(&old_topology, nodep) {
			// MWHEINZ FIXME: Replace this with a loop of sa_NodeRecord_Set() - don't do tests.
			if ((newRecords = sa_NodeRecord_Matches(&data, &samad, nodep,
													&nodeRecord, bytes)) == NO_MEM_RECORDS) {
				status = VSTATUS_NOMEM;
				break;
			}
			*records += newRecords;
		}						// for all nodes
	} else if (samad.header.mask & STL_NODE_RECORD_COMP_NODETYPE) {
		/* look for specific nodetype and clear nodetype mask to not try to match again */
		samad.header.mask = samad.header.mask & ~STL_NODE_RECORD_COMP_NODETYPE;
		if (nodeRecord.NodeInfo.NodeType == NI_TYPE_CA) {
			// IB_LOG_INFINI_INFOLX("sa_NodeRecord_GetTable: looking for FI Node Records,
			// mask=", samad.mask);
			for_all_ca_nodes(&old_topology, nodep) {
				if ((newRecords = sa_NodeRecord_Matches(&data, &samad, nodep,
														&nodeRecord,
														bytes)) == NO_MEM_RECORDS) {
					status = VSTATUS_NOMEM;
					break;
				}
				*records += newRecords;
			}
		} else if (nodeRecord.NodeInfo.NodeType == NI_TYPE_SWITCH) {
			// IB_LOG_INFINI_INFOLX("sa_NodeRecord_GetTable: looking for SWITCH Node Records,
			// mask=", samad.mask);
			for_all_switch_nodes(&old_topology, nodep) {
				if ((newRecords = sa_NodeRecord_Matches(&data, &samad, nodep,
														&nodeRecord,
														bytes)) == NO_MEM_RECORDS) {
					status = VSTATUS_NOMEM;
					break;
				}
				*records += newRecords;
			}
		} else if (nodeRecord.NodeInfo.NodeType == NI_TYPE_ROUTER) {
			// No routers in an STL fabric.
		} else {
			IB_LOG_WARN_FMT("sa_NodeRecord_GetTable",
							"Invalid node type[%d] in request from lid 0x%x",
							nodeRecord.NodeInfo.NodeType, lid);
		}
	} else if (samad.header.mask & STL_NODE_RECORD_COMP_NODEGUID) {
		/* look for a specific node guid */
		// IB_LOG_INFINI_INFOLX("sa_NodeRecord_GetTable: Getting all the Node Records with
		// GUID=", nodeRecord.NodeInfo.NodeGUID);
		if ((nodep = sm_find_guid(&old_topology, nodeRecord.NodeInfo.NodeGUID)) != NULL) {
			if ((newRecords = sa_NodeRecord_Matches(&data, &samad, nodep,
													&nodeRecord, bytes)) == NO_MEM_RECORDS) {
				status = VSTATUS_NOMEM;
			} else {
				*records += newRecords;
			}
		}
	} else {
		/* default: look at all the nodes and match */
		// IB_LOG_INFINI_INFOLX("sa_NodeRecord_GetTable: Getting all the Node Records by
		// default, mask=", samad.header.mask);
		for_all_nodes(&old_topology, nodep) {
			if ((newRecords = sa_NodeRecord_Matches(&data, &samad, nodep,
													&nodeRecord, bytes)) == NO_MEM_RECORDS) {
				status = VSTATUS_NOMEM;
				break;
			}
			*records += newRecords;
		}						// for all nodes
	}

	if (status == VSTATUS_NOMEM) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
		IB_LOG_ERROR_FMT("sa_NodeRecord_GetTable",
						 "Reached size limit at %d records processing request from lid 0x%x",
						 *records, lid);
	}

	(void) vs_rwunlock(&old_topology_lock);
	if (saDebugRmpp)
		IB_LOG_INFINI_INFO("sa_NodeRecord_GetTable: Number of Node Records found is  ",
						   *records);

	IB_EXIT("sa_NodeRecord_GetTable", status);
	return (status);
}

static Status_t
sa_IbNodeRecord_GetTable(Mai_t * maip, uint32_t * records)
{
	uint8_t *data;
	uint32_t bytes;
	Node_t *nodep;
	STL_SA_MAD samad;	// STL and IB have the same SA MAD format.
	Status_t status;
	IB_NODE_RECORD ibNodeRecord;
	uint32_t newRecords;
	Lid_t lid = 0;
	Port_t *portp;

	IB_ENTER("sa_IbNodeRecord_GetTable", maip, records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(IB_NODE_RECORD));

	// Again, small hack: We assume STL and IB have the same MAD structure.
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(IB_NODE_RECORD));

//
//  Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_IbNodeRecord_GetTable", status);
		return (status);
	}
//
//  Find all NodeRecords which match the template.
//
	(void) vs_rdlock(&old_topology_lock);
	/* 
	 * Return busy if source lid not in topology yet 
	 * This should throttle back the host
	 */
	lid = maip->addrInfo.slid;
	if ((portp = sm_find_active_port_lid(&old_topology, lid)) == NULL) {
		maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
		(void) vs_rwunlock(&old_topology_lock);
		IB_EXIT("sa_IbNodeRecord_GetTable", VSTATUS_OK);
		return (VSTATUS_OK);
	}

	BSWAPCOPY_IB_NODE_RECORD((IB_NODE_RECORD *) samad.data, &ibNodeRecord);

	if (!samad.header.mask) {
		/* doing for all nodes */
		for_all_nodes(&old_topology, nodep) {
			// MWHEINZ FIXME: Replace this with a loop of sa_IbNodeRecord_GetTable() - don't do tests.
			if ((newRecords = sa_IbNodeRecord_Matches(&data, &samad, nodep,
													  &ibNodeRecord, bytes)) == NO_MEM_RECORDS) {
				status = VSTATUS_NOMEM;
				break;
			}
			*records += newRecords;
		}
	} else if (samad.header.mask & NR_COMPONENTMASK_NI_NODETYPE) {
		/* look for specific nodetype and clear nodetype mask to not try to match again */
		samad.header.mask = samad.header.mask & ~NR_COMPONENTMASK_NI_NODETYPE;
		if (ibNodeRecord.NodeInfoData.NodeType == NI_TYPE_CA) {
			for_all_ca_nodes(&old_topology, nodep) {
				if ((newRecords = sa_IbNodeRecord_Matches(&data, &samad, nodep,
														  &ibNodeRecord,
														bytes)) == NO_MEM_RECORDS) {
					status = VSTATUS_NOMEM;
					break;
				}
				*records += newRecords;
			}
		} else if (ibNodeRecord.NodeInfoData.NodeType == NI_TYPE_SWITCH) {
			for_all_switch_nodes(&old_topology, nodep) {
				if ((newRecords = sa_IbNodeRecord_Matches(&data, &samad, nodep,
														&ibNodeRecord,
														bytes)) == NO_MEM_RECORDS) {
					status = VSTATUS_NOMEM;
					break;
				}
				*records += newRecords;
			}
		} else if (ibNodeRecord.NodeInfoData.NodeType == NI_TYPE_ROUTER) {
			// No routers in an STL fabric.
		} else {
			IB_LOG_WARN_FMT("sa_IbNodeRecord_GetTable",
							"Invalid node type[%d] in request from lid 0x%x",
							ibNodeRecord.NodeInfoData.NodeType, lid);
		}
	} else if (samad.header.mask & NR_COMPONENTMASK_NODEGUID) {
		/* look for a specific node guid */
		if ((nodep = sm_find_guid(&old_topology, ibNodeRecord.NodeInfoData.NodeGUID)) != NULL) {
			if ((newRecords = sa_IbNodeRecord_Matches(&data, &samad, nodep,
													&ibNodeRecord, bytes)) == NO_MEM_RECORDS) {
				status = VSTATUS_NOMEM;
			} else {
				*records += newRecords;
			}
		}
	} else {
		/* default: look at all the nodes and match */
		for_all_nodes(&old_topology, nodep) {
			if ((newRecords = sa_IbNodeRecord_Matches(&data, &samad, nodep,
													&ibNodeRecord, bytes)) == NO_MEM_RECORDS) {
				status = VSTATUS_NOMEM;
				break;
			}
			*records += newRecords;
		}						// for all nodes
	}

	if (status == VSTATUS_NOMEM) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
		IB_LOG_ERROR_FMT("sa_IbNodeRecord_GetTable",
						 "Reached size limit at %d records processing request from lid 0x%x",
						 *records, lid);
	}

	(void) vs_rwunlock(&old_topology_lock);
	if (saDebugRmpp)
		IB_LOG_INFINI_INFO("sa_IbNodeRecord_GetTable: Number of Node Records found is  ",
						   *records);

	IB_EXIT("sa_IbNodeRecord_GetTable", status);
	return (status);
}

static uint32_t
sa_NodeRecord_Matches(uint8_t ** data, STL_SA_MAD * samad, Node_t * nodep, STL_NODE_RECORD * nr,
					  uint32_t bytes)
{
	Port_t *portp = NULL;
	uint32_t records = 0;

	for_all_end_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
			continue;
		}

		if ((samad->header.mask & STL_NODE_RECORD_COMP_LID)
			&& (portp->portData->lid != nr->RID.LID)) {
			if ((portp->portData->lid <= nr->RID.LID)
				&& (nr->RID.LID <= sm_port_top_lid(portp))) {
				nr->RID.LID = portp->portData->lid;
				BSWAPCOPY_STL_NODE_RECORD((STL_NODE_RECORD *) nr,
										  (STL_NODE_RECORD *) samad->data);
			} else {
				continue;
			}
		}

		/* if specific portNumber is desired, create node record for that port only */
		if (!(samad->header.mask & STL_NODE_RECORD_COMP_LOCALPORTNUM) ||
			((samad->header.mask & STL_NODE_RECORD_COMP_LOCALPORTNUM)
			 && (portp->portno == nr->NodeInfo.u1.s.LocalPortNum))) {

			if (sa_check_len(*data, sizeof(STL_NODE_RECORD), bytes) != VSTATUS_OK) {
				records = NO_MEM_RECORDS;
				break;
			}

			if (sa_NodeRecord_Set(*data, nodep, portp) != VSTATUS_OK) {
				records = NO_MEM_RECORDS;
				break;
			}

			(void) sa_template_test_mask(samad->header.mask, samad->data, data, sizeof(STL_NODE_RECORD), bytes, &records);
		}
	}
	return (records);
}

static uint32_t
sa_IbNodeRecord_Matches(uint8_t ** data, STL_SA_MAD * samad, Node_t * nodep, IB_NODE_RECORD * nr,
					  uint32_t bytes)
{
	Port_t *portp = NULL;
	uint32_t records = 0;

	for_all_end_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
			continue;
		}

		if ((samad->header.mask & NR_COMPONENTMASK_LID)
			&& (portp->portData->lid != nr->RID.s.LID)) {
			if ((portp->portData->lid <= nr->RID.s.LID)
				&& (nr->RID.s.LID <= sm_port_top_lid(portp))) {
				nr->RID.s.LID = portp->portData->lid;
				// Fixes the LMC case. Replaces the LID in the original query with
				// the actual LID number in use.
				// Question: If we get here, why do we continue the loop?
				BSWAPCOPY_IB_NODE_RECORD((IB_NODE_RECORD *) nr,
										 (IB_NODE_RECORD *) samad->data);
			} else {
				continue;
			}
		}

		/* if specific portNumber is desired, create node record for that port only */
		if (!(samad->header.mask & NR_COMPONENTMASK_PORTNUMBER) ||
			((samad->header.mask & NR_COMPONENTMASK_PORTNUMBER)
			 && (portp->portno == nr->NodeInfoData.u1.s.LocalPortNum))) {

			if (sa_check_len(*data, sizeof(IB_NODE_RECORD), bytes) != VSTATUS_OK) {
				records = NO_MEM_RECORDS;
				break;
			}

			if (sa_IbNodeRecord_Set(*data, nodep, portp) != VSTATUS_OK) {
				records = NO_MEM_RECORDS;
				break;
			}

			(void) sa_template_test_mask(samad->header.mask, samad->data, data, sizeof(IB_NODE_RECORD), bytes, &records);
		}
	}
	return (records);
}


Status_t
sa_NodeRecord_CheckCache(STL_SA_MAD * samad, STL_NODE_RECORD * nr, SACacheEntry_t ** outCache)
{
	Status_t rc;
	SACacheEntry_t *caches[3];

	IB_ENTER("sa_NodeRecord_CheckCache", samad, nr, outCache, 0);

	*outCache = NULL;
	rc = VSTATUS_OK;

	(void) vs_lock(&saCache.lock);

	if (samad->header.mask == NR_COMPONENTMASK_NI_NODETYPE) {
		if (nr->NodeInfo.NodeType == NI_TYPE_CA) {
			(void) sa_cache_get(SA_CACHE_FI_NODES, outCache);
		} else if (nr->NodeInfo.NodeType == NI_TYPE_SWITCH) {
			(void) sa_cache_get(SA_CACHE_SWITCH_NODES, outCache);
		}
	} else if (!samad->header.mask) {
		// build a "transient" cache, which uses copies of already cached data
		// and is deleted immediately after use. helpful to avoid double-caching

		(void) sa_cache_get(SA_CACHE_FI_NODES, &caches[0]);
		(void) sa_cache_get(SA_CACHE_SWITCH_NODES, &caches[1]);

		rc = sa_cache_alloc_transient(caches, 2, outCache);
		if (rc != VSTATUS_OK) {
			IB_LOG_WARNRC("sa_NodeRecord_CheckCache: failed to build cache structure rc:", rc);
		}

		(void) sa_cache_release(caches[0]);
		(void) sa_cache_release(caches[1]);
	}

	(void) vs_unlock(&saCache.lock);

	IB_EXIT("sa_NodeRecord_CheckCache", rc);
	return rc;
}


// common utility function for the public cache building functions
//
static Status_t
sa_NodeRecord_BuildCache(Node_t * head, SACacheEntry_t * cachep)
{
	Status_t rc;
	Node_t *nodep;
	Port_t *portp;
	uint32_t bytes;
	uint32_t padBytes;
	uint32_t records;
	uint8_t *data;

	IB_ENTER("sa_NodeRecord_BuildCache", head, cachep, 0, 0);

	padBytes = Calculate_Padding(sizeof(STL_NODE_RECORD));

	// make an initial pass through the topology to get a record count for
	// the buffer allocation
	records = 0;
	for (nodep = head; nodep != NULL; nodep = nodep->type_next) {
		for_all_end_ports(nodep, portp) {
			if (sm_valid_port(portp) && portp->state > IB_PORT_DOWN)
				records++;
		}
	}

	if (records) {
		bytes = records * (sizeof(STL_NODE_RECORD) + padBytes);
		rc = vs_pool_alloc(&sm_pool, bytes, (void *) &cachep->data);
		if (rc != VSTATUS_OK) {
			IB_LOG_WARNRC
				("sa_NodeRecord_BuildCache: failed to allocate memory for cache buffer rc:",
				 rc);
			IB_EXIT("sa_NodeRecord_BuildCache", rc);
			return rc;
		} else {
			cachep->len = bytes;
		}

		data = cachep->data;

		for (nodep = head; nodep != NULL; nodep = nodep->type_next) {
			for_all_end_ports(nodep, portp) {
				if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
					continue;
				rc = sa_NodeRecord_Set(data, nodep, portp);
				if (rc) {
					IB_LOG_WARNRC("sa_NodeRecord_BuildCache: failed to build cache rc:", rc);
					vs_pool_free(&sm_pool, cachep->data);
					IB_EXIT("sa_NodeRecord_BuildCache", rc);
					return rc;
				}
				sa_increment_and_pad(&data, sizeof(STL_NODE_RECORD), padBytes, &cachep->records);
			}
		}
	} else {
		cachep->data = NULL;
		cachep->len = 0;
		cachep->records = 0;
	}

	cachep->valid = 1;

	rc = VSTATUS_OK;
	IB_EXIT("sa_NodeRecord_BuildCache", rc);
	return rc;
}


Status_t
sa_NodeRecord_BuildCACache(SACacheEntry_t * cachep, Topology_t * top)
{
	Status_t rc;

	IB_ENTER("sa_NodeRecord_BuildCACache", cachep, top, 0, 0);

	rc = sa_NodeRecord_BuildCache(top->ca_head, cachep);
	if (rc) {
		IB_EXIT("sa_NodeRecord_BuildCACache", rc);
		return rc;
	}

	sprintf(cachep->name, "CA NodeRecords");

	rc = VSTATUS_OK;
	IB_EXIT("sa_NodeRecord_BuildCACache", rc);
	return rc;
}


Status_t
sa_NodeRecord_BuildSwitchCache(SACacheEntry_t * cachep, Topology_t * top)
{
	Status_t rc;

	IB_ENTER("sa_NodeRecord_BuildSwitchCache", cachep, top, 0, 0);

	rc = sa_NodeRecord_BuildCache(top->switch_head, cachep);
	if (rc) {
		IB_EXIT("sa_NodeRecord_BuildSwitchCache", rc);
		return rc;
	}

	sprintf(cachep->name, "SW NodeRecords");

	rc = VSTATUS_OK;
	IB_EXIT("sa_NodeRecord_BuildSwitchCache", rc);
	return rc;
}

void showStlLids(void)
{ 
    Node_t *nodep; 
    STL_NODE_RECORD nodeRecord, tempNodeRecord; 
    PrintDest_t dest; 
    
    PrintDestInitFile(&dest, stdout); 
    
    (void)vs_rdlock(&old_topology_lock); 
    
    for_all_nodes(&old_topology, nodep) {
        Port_t *portp = NULL; 
        
        for_all_end_ports(nodep, portp) {
            if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) 
                continue; 
            (void)sa_NodeRecord_Set((uint8_t *)&tempNodeRecord, nodep, portp); 
            BSWAPCOPY_STL_NODE_RECORD(&tempNodeRecord, &nodeRecord); 
            PrintStlLid(&dest, 0, nodeRecord.RID.LID, 0);
        }
    }
    
    (void)vs_rwunlock(&old_topology_lock);
}

