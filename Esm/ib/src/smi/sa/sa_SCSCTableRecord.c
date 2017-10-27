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

#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"

Status_t	sa_SCSCTableRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_SCSCTableRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	records;
	uint16_t	attribOffset;

	IB_ENTER("sa_SCSCTableRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetSc2ScMappingRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblSc2ScMappingRecord);
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
		(void)sa_SCSCTableRecord_GetTable(maip, &records);
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
		IB_LOG_WARN("sa_SCSCTableRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
	attribOffset = sizeof(STL_SC_MAPPING_TABLE_RECORD) + Calculate_Padding(sizeof(STL_SC_MAPPING_TABLE_RECORD));
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_SCSCTableRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_SCSCTableRecord_Set(uint8_t *slp, Node_t *nodep, Port_t *in_portp, Port_t *out_portp, STL_LID lid) {
	STL_SC_MAPPING_TABLE_RECORD scSCTableRecord;
	int i;

	IB_ENTER("sa_SCSCTableRecord_Set", slp, nodep, in_portp, out_portp);

    if (in_portp == NULL || out_portp == NULL) {
        IB_LOG_ERROR_FMT( "sa_SCSCTableRecord_Set",
               "NULL port parameter for Node Guid["FMT_U64"], %s",
               nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_SCSCTableRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
    }

	memset((char *)&scSCTableRecord, 0, sizeof(scSCTableRecord));

	scSCTableRecord.RID.LID = lid;
    if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) && (in_portp->index>0) && (out_portp->index>0)) {
        scSCTableRecord.RID.InputPort = in_portp->index; 
        scSCTableRecord.RID.OutputPort = out_portp->index;
		if (in_portp->portData->scscMap) {
        	memcpy(&scSCTableRecord.Map, &in_portp->portData->scscMap[out_portp->index-1], sizeof(scSCTableRecord.Map));
		} else {
			// Hard coded for POD Values as these are not touched or fetched by SM in PRR
			// [They are hard-coded for the unity fn.]
			for (i=0; i<STL_MAX_SCS; i++) scSCTableRecord.Map[i].SC=i;
		}
    } else {
        IB_LOG_ERROR_FMT( "sa_SCSCTableRecord_Set",
               "Invalid call for non-swith or port 0 for Node Guid["FMT_U64"], %s",
               nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_SCSCTableRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
    }

	BSWAPCOPY_STL_SC_MAPPING_TABLE_RECORD(&scSCTableRecord, (STL_SC_MAPPING_TABLE_RECORD*)slp);

	IB_EXIT("sa_SCSCTableRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_SCSCTableRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint32_t	bytes;
	Node_t		*nodep;
	Port_t		*in_portp;
	Port_t		*out_portp;
	STL_SA_MAD		samad;
	Status_t	status;
	uint8_t		*data;
	bool_t		checkLid;
	bool_t		checkInPort;
	bool_t		checkOutPort;
	STL_LID		lid = 0;
	STL_LID		portLid=0;
	uint16_t	inPort=0;
	uint16_t	outPort=0;

	IB_ENTER("sa_SCSCTableRecord_GetTable", maip, records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_SC_MAPPING_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SC_MAPPING_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SC_MAPPING_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SC_MAPPING_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_SCSCTableRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SC_MAPPING_TABLE_RECORD));

	checkLid = (samad.header.mask & STL_SC2SC_RECORD_COMP_LID);
	if (checkLid) {	
		portLid = ntoh32(((STL_SC_MAPPING_TABLE_RECORD*)samad.data)->RID.LID);
		samad.header.mask ^= STL_SC2SC_RECORD_COMP_LID;
	}

	checkInPort = (samad.header.mask & STL_SC2SC_RECORD_COMP_INPUTPORT);
	if (checkInPort) {	
		inPort = ntoh32(((STL_SC_MAPPING_TABLE_RECORD*)samad.data)->RID.InputPort);
		samad.header.mask ^= STL_SC2SC_RECORD_COMP_INPUTPORT;
	}

	checkOutPort = (samad.header.mask & STL_SC2SC_RECORD_COMP_OUTPUTPORT);
	if (checkOutPort) {	
		outPort = ntoh32(((STL_SC_MAPPING_TABLE_RECORD*)samad.data)->RID.OutputPort);
		samad.header.mask ^= STL_SC2SC_RECORD_COMP_OUTPUTPORT;
	}

//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_SCSCTableRecord_GetTable", status);
		return(status);
	}

//
//      Find all SCSCTableRecords which match the template.
//
    (void)vs_rdlock(&old_topology_lock);

    for_all_nodes(&old_topology, nodep) {
		// SC2SC only applicable to switch external ports
        if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) continue;

        // Update lid for Switches
        Port_t * port0 = sm_get_port(nodep, 0);
        if (!sm_valid_port(port0) || port0->state <= IB_PORT_DOWN) continue;
        lid = port0->portData->lid;
        if (checkLid && lid!=portLid) continue;

        // SC2SC is only for physical port to physical port
        for_all_physical_ports(nodep, in_portp) {
            if (!sm_valid_port(in_portp) || in_portp->state <= IB_PORT_DOWN) {
                continue;
            }
			if (checkInPort && in_portp->index != inPort) continue;

            // For the opposite end: Only do the physical end ports (switches start with 1)
            for_all_physical_ports(nodep, out_portp) {
                if (!sm_valid_port(out_portp) || out_portp->state <= IB_PORT_DOWN) {
                    continue;
                }
				if (checkOutPort && out_portp->index != outPort) continue;

                if ((status = sa_check_len(data, sizeof(STL_SC_MAPPING_TABLE_RECORD), bytes)) != VSTATUS_OK) {
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                    IB_LOG_ERROR_FMT( "sa_SCSCTableRecord_GetTable",
                           "Reached size limit at %d records", *records);
                    goto done;
                }

                if ((status = sa_SCSCTableRecord_Set(data, nodep, in_portp, out_portp, lid)) != VSTATUS_OK) {
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                    goto done;
                }

                (void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_SC_MAPPING_TABLE_RECORD), bytes, records);
				if (checkOutPort && out_portp->index == outPort) break;
            }
			if (checkInPort && in_portp->index == inPort) break;
        }
        if (checkLid && lid==portLid) break;
    }

done:
    (void)vs_rwunlock(&old_topology_lock);

    IB_EXIT("sa_SCSCTableRecord_GetTable", status);
    return(status);
}
