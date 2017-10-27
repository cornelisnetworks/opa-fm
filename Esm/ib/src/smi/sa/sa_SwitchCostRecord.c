/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2017, Intel Corporation

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

//===========================================================================
//
// FILE NAME
//    sa_SwitchCostRecord.c
//
// DESCRIPTION
//    This file contains the routines to process the SA requests for
//    records of the SwitchCostRecord type.
//
// DATA STRUCTURES
//
//
// FUNCTIONS
//
//
// DEPENDENCIES
//
//
//
//
//===========================================================================//

#include "iba/stl_sa_priv.h"


#include "ib_status.h"
#include "ib_sa.h"

#include "sm_l.h"
#include "sa_l.h"
#include "sm_counters.h"

static Status_t sa_SwitchCostRecord_GetTable(Mai_t * maip, uint32_t *records);

Status_t
sa_SwitchCostRecord(Mai_t *maip, sa_cntxt_t *sa_cntxt)
{
	uint32_t records;

	uint32_t attribOffset;

	IB_ENTER(__func__, maip, 0, 0, 0);

	// Assume failure
	records = 0;

	// Basic Checks
	if (maip->base.method == SA_CM_GET){
		INCREMENT_COUNTER(smCounterSaRxGetSwitchCostRecord);
	} else if (maip->base.method == SA_CM_GETTABLE){
		INCREMENT_COUNTER(smCounterSaRxGetTblSwitchCostRecord);
	} else {
		maip->base.status = MAD_STATUS_BAD_METHOD;
		IB_LOG_WARN_FMT(__func__, "Invalid method: %s (%u)",
				cs_getMethodText(maip->base.method), maip->base.method);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	// Check Base and Class Version
	if (maip->base.bversion != STL_BASE_VERSION || maip->base.cversion != STL_SA_CLASS_VERSION) {
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "Invalid Base and/or Class Versions: Base %u, Class %u",
				maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	} else {
		(void)sa_SwitchCostRecord_GetTable(maip, &records);
	}

	// reply
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for GET: ", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_SWITCH_COST_RECORD) + Calculate_Padding(sizeof(STL_SWITCH_COST_RECORD));
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

static Status_t
sa_SwitchCostRecord_Set(uint8_t *cp, Node_t *source_nodep, cl_map_item_t **start_mi, bool_t whole_row)
{
	Port_t *source_portp;
	Node_t *dest_nodep;
	Port_t *dest_portp;

	STL_SWITCH_COST_RECORD costRecord = {0};
	int i;
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, cp, source_nodep, 0, 0);

	if (!sm_valid_port((source_portp = sm_get_port(source_nodep, 0)))) goto done;

	costRecord.SLID = source_portp->portData->lid;

	for (i = 0;
			*start_mi != cl_qmap_end(old_topology.switchLids) && (i < STL_SWITCH_COST_NUM_ENTRIES);
			*start_mi = cl_qmap_next(*start_mi)) {
		dest_nodep = PARENT_STRUCT(*start_mi, Node_t, switchLidMapObj);
		if (!dest_nodep || dest_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH ||
				!sm_valid_port(dest_portp = sm_get_port(dest_nodep, 0)))
			continue;

		if (!whole_row && (dest_portp->portData->lid >= source_portp->portData->lid)){ /* only need half of matrix */
			status =  VSTATUS_DONE;
			break;
		}

		costRecord.Cost[i].DLID = dest_portp->portData->lid;
		costRecord.Cost[i].value = old_topology.cost[Index(source_nodep->swIdx, dest_nodep->swIdx)];
		++i;
	}

	memcpy(cp, &costRecord, sizeof(STL_SWITCH_COST_RECORD));
	BSWAP_STL_SWITCH_COST_RECORD((STL_SWITCH_COST_RECORD *) cp);

done:
	IB_EXIT(__func__, status);
	return (status);
}

static Status_t
sa_SwitchCostRecord_GetTable(Mai_t *maip, uint32_t *records)
{
	uint8_t    *data;
	uint32_t   bytes;
	STL_SA_MAD samad;
	Status_t   status = VSTATUS_OK;
	Node_t     *nodep;
	uint32_t portLid = 0;
	bool_t checkLid;
	cl_map_item_t *mi;

	IB_ENTER(__func__, maip, records, 0, 0);


	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_SWITCH_COST_RECORD));

	// Verify the size of the data received for the request
	if (maip->datasize - sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SWITCH_COST_RECORD)) {
		IB_LOG_ERROR_FMT(__func__,
				"invalid MAD length; size of STL_SWITCH_COST_RECORD[%"PRISZT"], datasize[%d]",
				sizeof(STL_SWITCH_COST_RECORD), (int)(maip->datasize - sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT(__func__, MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(STL_SWITCH_COST_RECORD));
	BSWAP_STL_SWITCH_COST_RECORD((STL_SWITCH_COST_RECORD *)samad.data);

	checkLid = (samad.header.mask & STL_SWITCH_COST_REC_COMP_SLID);
	if (checkLid) {
		portLid = ((STL_SWITCH_COST_RECORD*)samad.data)->SLID;
		samad.header.mask ^= STL_SWITCH_COST_REC_COMP_SLID;
	}

	// Find the cost records
	(void)vs_rdlock(&old_topology_lock);

	mi = cl_qmap_head(old_topology.switchLids);
	if (mi == cl_qmap_end(old_topology.switchLids)) {
		IB_LOG_ERROR_FMT(__func__, "No switches found in fabric");
		IB_EXIT(__func__, status);
		return status;
	}

	if (checkLid) {
		Port_t *temp_port;
		temp_port = sm_find_node_and_port_lid(&old_topology, portLid, &nodep);

		if (!sm_valid_port(temp_port) || (temp_port->state <= IB_PORT_DOWN) || nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			goto done;
		}

		mi= cl_qmap_head(old_topology.switchLids);
		while(mi != cl_qmap_end(old_topology.switchLids)) {
			if (sa_check_len(data, sizeof(STL_SWITCH_COST_RECORD), bytes) != VSTATUS_OK) {
				status = VSTATUS_NOMEM;
				IB_LOG_ERROR_FMT(__func__, "Reached size limit at %d records", *records);
				goto done;
			}

			if ((status = sa_SwitchCostRecord_Set(data, nodep, &mi, checkLid)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				goto done;
			}

			sa_increment_and_pad(&data, sizeof(STL_SWITCH_COST_RECORD), bytes, records);
		}
		goto done;
	}

	cl_map_item_t *start_mi = cl_qmap_head(old_topology.switchLids);
	// collect cost records for each switch
	for_all_qmap_item(old_topology.switchLids, mi) {
		status = VSTATUS_OK;
		nodep = PARENT_STRUCT(mi, Node_t, switchLidMapObj);
		if (!nodep || nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) continue;
		while (status != VSTATUS_DONE && start_mi != cl_qmap_end(old_topology.switchLids)) {
			if (sa_check_len(data, sizeof(STL_SWITCH_COST_RECORD), bytes) != VSTATUS_OK) {
				status = VSTATUS_NOMEM;
				IB_LOG_ERROR_FMT(__func__, "Reached size limit at %d records", *records);
				goto done;
			}

			status = sa_SwitchCostRecord_Set(data, nodep, &start_mi, checkLid);
			sa_increment_and_pad(&data, sizeof(STL_SWITCH_COST_RECORD), bytes, records);
		}

		start_mi = cl_qmap_head(old_topology.switchLids);
	}

done:
	(void)vs_rwunlock(&old_topology_lock);
	IB_EXIT(__func__, status);
	return status;

}
