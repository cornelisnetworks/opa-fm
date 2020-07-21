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
#include "os_g.h"
#include "iba/stl_sa_priv.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"

static Status_t sa_QuarantinedNodeRecord_GetTable(Mai_t *, uint32_t *);
static Status_t sa_QuarantinedNodeRecord_BuildRecord(uint8_t *cp, QuarantinedNode_t *nodep);

Status_t sa_QuarantinedNodeRecord(Mai_t *maip, sa_cntxt_t *sa_cntxt)
{
	uint32_t records;		//Number of records
	uint32_t attribOffset;

	// Assume failure
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblQuarantinedNodeRecord);
	} else {
		maip->base.status = MAD_STATUS_BAD_METHOD;
		IB_LOG_WARN_FMT(__func__, "invalid Method: %s (%u)",
			cs_getMethodText(maip->base.method), maip->base.method);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	// Check Base and Class Version
	if (maip->base.bversion == STL_BASE_VERSION && maip->base.cversion == STL_SA_CLASS_VERSION) {
		(void)sa_QuarantinedNodeRecord_GetTable(maip, &records);
	} else {
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	// Determine reply status
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_QuarantinedNodeRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_QUARANTINED_NODE_RECORD) + Calculate_Padding(sizeof(STL_QUARANTINED_NODE_RECORD));

	sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
	(void) sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_QuarantinedNodeRecord", VSTATUS_OK);
	return VSTATUS_OK;
}

static Status_t sa_QuarantinedNodeRecord_GetTable(Mai_t *maip, uint32_t *records)
{
	uint8_t *data;		        // Pointer to the sa_data
	uint32_t bytes;		        // Amount of padding required for an STL_QUARANTINED_NODE_RECORD
	QuarantinedNode_t *nodep;   // The current node in the topology data structure we look at
	STL_SA_MAD samad;	        // Our return SA Mad
	Status_t status;	        // Status of the request

	IB_ENTER("sa_QuarantinedNodeRecord_GetTable", maip, records, 0, 0);

	status = VSTATUS_OK;	// Assume everything is ok until it isn't
	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_QUARANTINED_NODE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_QUARANTINED_NODE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_QUARANTINED_NODE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_QUARANTINED_NODE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_QuarantinedNodeRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(STL_QUARANTINED_NODE_RECORD));

	// Get all the Quarantined Nodes
	(void) vs_rdlock(&old_topology_lock);

	for_all_quarantined_nodes(&old_topology, nodep)
	{
		// Check to make sure we still have memory in the sa_data
		if(sa_check_len(data, sizeof(STL_NODE_RECORD), bytes) != VSTATUS_OK)
		{
			return VSTATUS_NOMEM;
		}else
		{
			sa_QuarantinedNodeRecord_BuildRecord(data, nodep);
			sa_increment_and_pad(&data, sizeof(STL_QUARANTINED_NODE_RECORD), bytes, records);
		}
	}

	(void) vs_rwunlock(&old_topology_lock);

	if(saDebugRmpp)
		IB_LOG_INFINI_INFO("sa_QuarantinedNodeRecord_GetTable: Number of Node Records found is ", *records);

	IB_EXIT("sa_QuarantinedNodeRecord_GetTable", status);

	return status;
}

static Status_t sa_QuarantinedNodeRecord_BuildRecord(uint8_t *cp, QuarantinedNode_t *nodep)
{
	STL_QUARANTINED_NODE_RECORD qnRecord = { 0 };

	IB_ENTER("sa_QuarantinedNodeRecord_BuildRecord", cp, nodep, 0, 0);

	qnRecord.trustedLid = nodep->authenticNode->port[0].portData->lid;
	qnRecord.trustedNodeGUID = nodep->authenticNode->nodeInfo.NodeGUID;
	qnRecord.trustedPortNum = nodep->authenticNodePort->index;
	qnRecord.trustedNeighborNodeGUID = nodep->authenticNodePort->portData->portInfo.NeighborNodeGUID;
	memcpy((void *) &(qnRecord.NodeDesc), (void *) &(nodep->quarantinedNode->nodeDesc), sizeof(qnRecord.NodeDesc));
	qnRecord.NodeDesc.NodeString[STL_NODE_DESCRIPTION_ARRAY_SIZE-1] = 0;
	qnRecord.NodeInfo = nodep->quarantinedNode->nodeInfo;
	qnRecord.quarantineReasons = nodep->quarantineReasons;
	memcpy((void *) &(qnRecord.expectedNodeInfo), (void*) &(nodep->expNodeInfo), sizeof(STL_EXPECTED_NODE_INFO));

	BSWAPCOPY_STL_QUARANTINED_NODE_RECORD(&qnRecord, (STL_QUARANTINED_NODE_RECORD *) cp);

	IB_EXIT("sa_QuarantinedNodeRecord_Set", VSTATUS_OK);
	return VSTATUS_OK;
}
