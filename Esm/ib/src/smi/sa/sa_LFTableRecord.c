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
//    sa_LFTableRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the LFTableRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_LFTableRecord							     //
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
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"

Status_t	sa_LFTableRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_LFTableRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
	uint32_t	records;
	uint16_t	attribOffset;

	IB_ENTER("sa_LFTableRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetLftRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblLftRecord);
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
		(void)sa_LFTableRecord_GetTable(maip, &records);
	} else {
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	attribOffset = Calculate_Padding(sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD)) + sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD);
//
//	Determine reply status
//

	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
        IB_LOG_WARN("sa_LFTableRecord: too many records for SA_CM_GET:", records);
        records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

    sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset );
	(void)sa_send_reply(maip, sa_cntxt );

	IB_EXIT("sa_LFTableRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_LFTableRecord_Set(uint8_t *lftp, Node_t *nodep, Port_t *portp, uint16_t index) {
	STL_LID		    lid;
	Port_t          *lftPortp;
	uint32_t		length;
	STL_LINEAR_FORWARDING_TABLE_RECORD lftRecord;

	IB_ENTER("sa_LFTableRecord_Set", lftp, nodep, portp, 0);

	if (!sm_valid_port((lftPortp = sm_get_port(nodep,0)))) {
		IB_LOG_ERROR0("sa_LFTableRecord_Set: failed to get port");
		IB_EXIT("sa_LFTableRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
	}

	lid = lftPortp->portData->lid;

	lftRecord.RID.LID = lid;
	lftRecord.RID.BlockNum = index;
	lftRecord.RID.Reserved = 0;

	if (nodep->lft == NULL) {
		IB_LOG_ERROR_FMT( "sa_LFTableRecord_Get",
			"LFT is NULL for node %*s "FMT_U64,
			STL_NODE_DESCRIPTION_ARRAY_SIZE,
			nodep->nodeDesc.NodeString,
			nodep->nodeInfo.NodeGUID);
		return(VSTATUS_BAD);
	}

	length = MIN(nodep->switchInfo.LinearFDBTop-MAX_LFT_ELEMENTS_BLOCK*index+1,
		MAX_LFT_ELEMENTS_BLOCK);

	memcpy((void *)lftRecord.LinearFdbData, (void *)&nodep->lft[MAX_LFT_ELEMENTS_BLOCK*index], length);

	if (length<MAX_LFT_ELEMENTS_BLOCK) {
		memset((void*)&lftRecord.LinearFdbData[length], 0xff,
			MAX_LFT_ELEMENTS_BLOCK-length);
	}

	BSWAPCOPY_STL_LINEAR_FORWARDING_TABLE_RECORD(&lftRecord, (STL_LINEAR_FORWARDING_TABLE_RECORD *)lftp);

	IB_EXIT("sa_LFTableRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_LFTableRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t		*data;
	uint32_t	bytes;
	uint32_t	index;
	Node_t		*nodep;
	STL_SA_MAD		samad;
	Status_t	status;
	bool_t		checkLid;
	uint16_t	portZeroLid = 0;

	IB_ENTER("sa_LFTableRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_LINEAR_FORWARDING_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_LFTableRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD));

	checkLid = (samad.header.mask & STL_LFT_RECORD_COMP_LID);
	if (checkLid) {
		portZeroLid = ntoh32(((STL_LINEAR_FORWARDING_TABLE_RECORD*)samad.data)->RID.LID);
		samad.header.mask ^= STL_LFT_RECORD_COMP_LID;
	}

//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_LFTableRecord_GetTable", status);
		return(status);
	}

//
//      Load the LFTableRecords in the sa_data response
//
	(void)vs_rdlock(&old_topology_lock);

	if (checkLid) {
		if (sm_find_node_and_port_lid(&old_topology, portZeroLid, &nodep) != NULL) {
			if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) goto done;
			for (index = 0; index <= nodep->switchInfo.LinearFDBTop; index += MAX_LFT_ELEMENTS_BLOCK) {
				if ((status = sa_check_len(data, sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD), bytes)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_ERROR_FMT( "sa_LFTableRecord_GetTable",
					   	"Reached size limit at %d records", *records);
					break;
				}
	
				if ((status = sa_LFTableRecord_Set(data, nodep, nodep->port, index/MAX_LFT_ELEMENTS_BLOCK)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					break;
				}
				(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD), bytes, records);
			}
		}
	} else {
		for_all_switch_nodes(&old_topology, nodep) {
			for (index = 0; index <= nodep->switchInfo.LinearFDBTop; index += MAX_LFT_ELEMENTS_BLOCK) {
				if ((status = sa_check_len(data, sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD), bytes)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_ERROR_FMT( "sa_LFTableRecord_GetTable",
					   	"Reached size limit at %d records", *records);
					break;
				}
	
				if ((status = sa_LFTableRecord_Set(data, nodep, nodep->port, index/MAX_LFT_ELEMENTS_BLOCK)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					break;
				}
	
				(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD), bytes, records);
			}
		}
	}

done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_LFTableRecord_GetTable", status);
	return(status);
}
