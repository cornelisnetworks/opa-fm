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

#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"

Status_t	sa_SCVLTableRecord_Get(Mai_t *, uint32_t *);
Status_t	sa_SCVLTableRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_SCVLTableRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	records;
	uint16_t	attribOffset;

	IB_ENTER("sa_SCVLTableRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		// Check Attribute ID
		switch (maip->base.aid) {
		case STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD:
			INCREMENT_COUNTER(smCounterSaRxGetSc2VltMappingRecord);
			break;
		case STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD:
			INCREMENT_COUNTER(smCounterSaRxGetSc2VlntMappingRecord);
			break;
		case STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD:
			INCREMENT_COUNTER(smCounterSaRxGetSc2VlrMappingRecord);
			break;
		default:
			// Generate an error response and return.
			maip->base.status = MAD_STATUS_BAD_ATTR;
			IB_LOG_WARN_FMT(__func__, "invalid Attribute: %s (%u)",
				cs_getAidName(maip->base.mclass, maip->base.aid), maip->base.aid);
			(void)sa_send_reply(maip, sa_cntxt);
			IB_EXIT(__func__, VSTATUS_OK);
			return VSTATUS_OK;
		}
	} else if (maip->base.method == SA_CM_GETTABLE) {
		// Check Attribute ID
		switch (maip->base.aid) {
		case STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD:
			INCREMENT_COUNTER(smCounterSaRxGetTblSc2VltMappingRecord);
			break;
		case STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD:
			INCREMENT_COUNTER(smCounterSaRxGetTblSc2VlntMappingRecord);
			break;
		case STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD:
			INCREMENT_COUNTER(smCounterSaRxGetTblSc2VlrMappingRecord);
			break;
		default:
			// Generate an error response and return.
			maip->base.status = MAD_STATUS_BAD_ATTR;
			IB_LOG_WARN_FMT(__func__, "invalid Attribute: %s (%u)",
				cs_getAidName(maip->base.mclass, maip->base.aid), maip->base.aid);
			(void)sa_send_reply(maip, sa_cntxt);
			IB_EXIT(__func__, VSTATUS_OK);
			return VSTATUS_OK;
		}
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
		(void)sa_SCVLTableRecord_GetTable(maip, &records);
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
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_SCVLTableRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
	attribOffset = sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD) + Calculate_Padding(sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD));
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_SCVLTableRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_SCVLTableRecord_Set(uint8_t *slp, Node_t *nodep, Port_t *portp, uint16_t attrib) {
	STL_LID			    lid;
    Port_t              *scvlPortp;
	STL_SC2VL_R_MAPPING_TABLE_RECORD scVLTableRecord;

	IB_ENTER("sa_SCVLTableRecord_Set", slp, nodep, 0, 0);

	memset((char *)&scVLTableRecord, 0, sizeof(scVLTableRecord));
    if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        if (!sm_valid_port((scvlPortp = sm_get_port(nodep,0)))) {
            IB_LOG_WARN_FMT( "sa_SCVLTableRecord_Set",
                   "failed to get port %d for switch Node Guid["FMT_U64"], %s",
                   0, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
            IB_EXIT("sa_SCVLTableRecord_Set", VSTATUS_BAD);
            return(VSTATUS_BAD);
        }
        lid = scvlPortp->portData->lid;
    } else {
        lid = portp->portData->lid;
    }

	scVLTableRecord.RID.LID = lid;
	scVLTableRecord.RID.Port = portp->index;

	switch (attrib) {
	case STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD:
		memcpy(scVLTableRecord.SCVLMap, &portp->portData->scvltMap, sizeof(STL_VL) * STL_MAX_VLS);
		break;
	case STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD:
		memcpy(scVLTableRecord.SCVLMap, &portp->portData->scvlntMap, sizeof(STL_VL) * STL_MAX_VLS);
		break;
	case STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD:
		memcpy(scVLTableRecord.SCVLMap, &portp->portData->scvlrMap, sizeof(STL_VL) * STL_MAX_VLS);
		break;
	default:
		IB_LOG_WARN_FMT( "sa_SCVLTableRecord_Set",
			   "failed to set SCVLTable attribute for Node Guid["FMT_U64"], %s",
			   nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_SCVLTableRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
	}
	BSWAPCOPY_STL_SC2VL_R_MAPPING_TABLE_RECORD(&scVLTableRecord, (STL_SC2VL_R_MAPPING_TABLE_RECORD*)slp);

	IB_EXIT("sa_SCVLTableRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_SCVLTableRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint32_t	bytes;
	Node_t		*nodep;
	Port_t		*portp;
	STL_SA_MAD		samad;
	Status_t	status;
	uint8_t		*data;
	bool_t		checkLid;
	STL_LID		portLid=0;
	bool_t		checkPort;
	uint8_t		portNum=0;

	IB_ENTER("sa_SCVLTableRecord_GetTable", maip, records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SC2VL_R_MAPPING_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_SCVLTableRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD));

	checkLid = (samad.header.mask & STL_SC2VL_R_RECORD_COMP_LID);
	if (checkLid) {	
		portLid = ntoh32(((STL_SC2VL_R_MAPPING_TABLE_RECORD*)samad.data)->RID.LID);
		samad.header.mask ^= STL_SC2VL_R_RECORD_COMP_LID;
	}

	checkPort = (samad.header.mask & STL_SC2VL_R_RECORD_COMP_PORT);
	if (checkPort) {	
		portNum = ((STL_SC2VL_R_MAPPING_TABLE_RECORD*)samad.data)->RID.Port;
		samad.header.mask ^= STL_SC2VL_R_RECORD_COMP_PORT;
	}
//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_SCVLTableRecord_GetTable", status);
		return(status);
	}

//
//      Find all SCSLTableRecords which match the template.
//
	(void)vs_rdlock(&old_topology_lock);

	if (checkLid) {
		Port_t *matched_portp;
		matched_portp = sm_find_node_and_port_lid(&old_topology, portLid, &nodep);
		if (! matched_portp) goto done;

		for_all_matched_ports(nodep, portp, matched_portp){
			if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN)) {
				continue;
			}
			if (checkPort && portNum != portp->index)
				continue;
			// SC2VLnt N/A for switch port 0
			if (maip->base.aid ==  STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD && portp->index == 0)
				continue;

			if (maip->base.aid == STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD &&
				!sm_IsVLrSupported(nodep, portp))
				continue;

			if ((status = sa_check_len(data, sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD), bytes)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( "sa_SCVLTableRecord_GetTable",
					   "Reached size limit at %d records", *records);
				goto done;
			}

			if ((status = sa_SCVLTableRecord_Set(data, nodep, portp, maip->base.aid)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				goto done;
			}

			(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD), bytes, records);
		}
		goto done;
	}

	for_all_nodes(&old_topology, nodep) {
		for_all_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
				continue;
			}
			if (checkPort && portNum != portp->index)
				continue;
			// SC2VLnt N/A for switch port 0
			if (maip->base.aid ==  STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD && portp->index == 0)
				continue;

			if (maip->base.aid == STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD &&
				!sm_IsVLrSupported(nodep, portp))
				continue;

			if ((status = sa_check_len(data, sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD), bytes)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( "sa_SCVLTableRecord_GetTable",
					   "Reached size limit at %d records", *records);
				goto done;
			}

			if ((status = sa_SCVLTableRecord_Set(data, nodep, portp, maip->base.aid)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				goto done;
			}

			(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD), bytes, records);
		}
	}

done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_SCVLTableRecord_GetTable", status);
	return(status);
}
