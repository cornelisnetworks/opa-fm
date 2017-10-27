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
//    sa_SwitchInfoRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the SwitchInfoRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_SwitchInfoRecord						     //
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

Status_t	sa_SwitchInfoRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_SwitchInfoRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	records;
	uint16_t	attribOffset;
	uint16_t	rec_sz;

	IB_ENTER("sa_SwitchInfoRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetSwitchInfoRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblSwitchInfoRecord);
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
		rec_sz = sizeof(STL_SWITCHINFO_RECORD);
		(void)sa_SwitchInfoRecord_GetTable(maip, &records);
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
//	Determine reply status
//
	if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_SwitchInfoRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
	attribOffset = sizeof(STL_SWITCHINFO_RECORD) + Calculate_Padding(rec_sz);
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_SwitchInfoRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_SwitchInfoRecord_Set(uint8_t *srp, Node_t *nodep, Port_t *portp) {
	STL_LID lid;
	Port_t *swiPortp;
	STL_SWITCHINFO_RECORD	switchInfoRecord = {{0}};

	IB_ENTER("sa_SwitchInfoRecord_Set", srp, nodep, portp, 0);

    if (portp == NULL) {
        IB_LOG_ERROR_FMT( "sa_SwitchInfoRecord_Set",
               "NULL port parameter for Node Guid["FMT_U64"], %s",
               nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_SwitchInfoRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
    }

    if (!sm_valid_port((swiPortp = sm_get_port(nodep,0)))) {
        IB_LOG_ERROR0("sa_SwitchInfoRecord_Set: failed to get port");
        IB_EXIT("sa_SwitchInfoRecord_Set", VSTATUS_BAD);
        return(VSTATUS_BAD);
    }

	lid = swiPortp->portData->lid;
	switchInfoRecord.RID.LID = lid;
	switchInfoRecord.Reserved = 0;
	switchInfoRecord.SwitchInfoData = nodep->switchInfo;
	BSWAPCOPY_STL_SWITCHINFO_RECORD(&switchInfoRecord, (STL_SWITCHINFO_RECORD*)srp);

	IB_EXIT("sa_SwitchInfoRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_SwitchInfoRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t		*data;
	uint32_t	bytes;
	Node_t		*nodep;
	STL_SA_MAD	samad;
	Status_t	status;
	STL_SWITCHINFO_RECORD sir;
	bool_t		checkLid;
	uint16_t	portLid=0;

	IB_ENTER("sa_SwitchInfoRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_SWITCHINFO_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SWITCHINFO_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SWITCHINFO_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SWITCHINFO_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_SwitchInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SWITCHINFO_RECORD));
	BSWAPCOPY_STL_SWITCHINFO_RECORD((STL_SWITCHINFO_RECORD*)samad.data, &sir);

	checkLid = (samad.header.mask & STL_SWITCHINFO_RECORD_COMP_LID);
	if (checkLid) {	
		portLid = sir.RID.LID;
		samad.header.mask ^= STL_SWITCHINFO_RECORD_COMP_LID;
	}
//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_SwitchInfoRecord_GetTable", status);
		return(status);
	}

//
//      Load the SwitchInfoRecords in the SADB
//
	(void)vs_rdlock(&old_topology_lock);

	if (checkLid) {
		Port_t *portp;
		portp = sm_find_node_and_port_lid(&old_topology, portLid, &nodep);

		if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN)) {
			goto done;
		}
		if ((status = sa_check_len(data, sizeof(STL_SWITCHINFO_RECORD), bytes)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
			IB_LOG_ERROR_FMT( "sa_SwitchInfoRecord_GetTable",
				   "Reached size limit at %d records", *records);
			goto done;
		}

		if ((status = sa_SwitchInfoRecord_Set(data, nodep, nodep->port)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
			goto done;
		}

		(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_SWITCHINFO_RECORD), bytes, records);
		goto done;
	}

	for_all_switch_nodes(&old_topology, nodep) {
		if ((status = sa_check_len(data, sizeof(STL_SWITCHINFO_RECORD), bytes)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
			IB_LOG_ERROR_FMT( "sa_SwitchInfoRecord_GetTable",
				   "Reached size limit at %d records", *records);
			break;
		}

		if ((status = sa_SwitchInfoRecord_Set(data, nodep, nodep->port)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
			break;
		}

		(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_SWITCHINFO_RECORD), bytes, records);
	}

done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_SwitchInfoRecord_GetTable", status);
	return(status);
}
