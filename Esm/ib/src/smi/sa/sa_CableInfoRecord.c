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
//                                                                           //
// FILE NAME                                                                 //
//    sa_CableInfoRecords.c                                                  //
//                                                                           //
// DESCRIPTION                                                               //
//    This file contains the routines to process the SA requests for         //
//    records of the Cable Info Record type.                                 //
//                                                                           //
// DATA STRUCTURES                                                           //
//    None                                                                   //
//                                                                           //
// DEPENDENCIES                                                              //
//    ib_mad.h                                                               //
//    ib_status.h                                                            //
//                                                                           //
//                                                                           //
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


static Status_t GetCableInfoRecord(Mai_t *maip, uint32_t *records);

Status_t
sa_CableInfoRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t records;
	uint32_t attribOffset;


	IB_ENTER("sa_CableInfoRecord", maip, 0, 0, 0);

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetCableInfoRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblCableInfoRecord);
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
		(void)GetCableInfoRecord(maip, &records);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_CableInfoRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else
		 maip->base.status = MAD_STATUS_OK;

	attribOffset = sizeof(STL_CABLE_INFO_RECORD) + Calculate_Padding(sizeof(STL_CABLE_INFO_RECORD));
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_CableInfoRecord", VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
GetCableInfoRecord(Mai_t *maip, uint32_t *records)
{
	uint8_t 	*data;
	uint32_t 	bytes;
	STL_SA_MAD 	samad;
	Status_t 	status;
	bool_t 		checkLid, checkPort;
	uint8_t		portNum=0;
	uint16_t	portLid=0;
	Node_t		*pNode;
	Port_t		*pPort;
	STL_CABLE_INFO_RECORD record = {{0}};
	uint32_t     bfrIdx = 0;
	uint8_t len, offset;
	uint16_t addr;


	IB_ENTER("GetCableInfoRecord", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_CABLE_INFO_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_CABLE_INFO_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_CABLE_INFO_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_CABLE_INFO_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("GetCableInfoRecord", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_CABLE_INFO_RECORD));

	checkLid = samad.header.mask & STL_CIR_COMP_LID;
	if (checkLid) {
		portLid = ntoh32(((STL_CABLE_INFO_RECORD*)(samad.data))->LID);
		samad.header.mask ^= STL_CIR_COMP_LID;
	}
	checkPort = samad.header.mask & STL_CIR_COMP_PORT;
	if (checkPort) {
		portNum = ((STL_CABLE_INFO_RECORD*)(samad.data))->Port;
		samad.header.mask ^= STL_CIR_COMP_PORT;
	}

	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("GetCableInfoRecord", VSTATUS_OK);
		return VSTATUS_OK;
	}

	if (!(samad.header.mask & STL_CIR_COMP_LEN) || !(samad.header.mask & STL_CIR_COMP_ADDR)) {
		maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
		IB_EXIT("GetCableInfoRecord", VSTATUS_OK);
		return VSTATUS_OK;
	}

	BSWAP_STL_CABLE_INFO_RECORD((STL_CABLE_INFO_RECORD*)samad.data);

	// Note: Len is NOT the real length, its the length-1, as defined in STL spec,
	len = ((STL_CABLE_INFO_RECORD*)(samad.data))->Length;
	addr = ((STL_CABLE_INFO_RECORD*)(samad.data))->u1.s.Address;
	samad.header.mask ^= (STL_CIR_COMP_LEN|STL_CIR_COMP_ADDR);

	// Cable info query is expected to query specific addresses / length
	// Check here for these expected values.
	if ((addr<STL_CIB_STD_HIGH_PAGE_ADDR) ||
        (addr>STL_CIB_STD_END_ADDR)   ||
        (len >= STL_CABLE_INFO_PAGESZ) || 
        ((addr+len) > STL_CIB_STD_END_ADDR)) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("GetCableInfoRecord", VSTATUS_OK);
		return VSTATUS_OK;
	}

	(void)vs_rdlock(&old_topology_lock);

	if (checkLid) {
		Port_t *matched_portp;
		if ((matched_portp = sm_find_node_and_port_lid(&old_topology, portLid, &pNode)) != NULL) {
			for_all_matched_ports(pNode, pPort, matched_portp) {
				if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) continue;
				if (!pPort->portData->cableInfo) continue;

				if (checkPort && portNum != pPort->index) continue;

				offset = 0;
				do {
					if ((status = sa_check_len(data, sizeof(STL_CABLE_INFO_RECORD), bytes)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						IB_LOG_ERROR_FMT("GetCableInfoRecord", "Reached size limit at %d records", *records);
						goto done;
					}
			
					record.LID = portLid;
					record.Port = pPort->index;
					record.u1.s.Address = addr + offset;
					record.Length = MIN(len - offset, STL_CABLE_INFO_MAXLEN);
					record.u1.s.PortType = pPort->portData->portInfo.PortPhysConfig.s.PortType;
					bfrIdx = addr - STL_CIB_STD_HIGH_PAGE_ADDR + offset;
					memcpy(record.Data, &pPort->portData->cableInfo->buffer[bfrIdx], record.Length + 1); 
					BSWAPCOPY_STL_CABLE_INFO_RECORD(&record, (STL_CABLE_INFO_RECORD*)data);
					(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_CABLE_INFO_RECORD), bytes, records);
				} while ((offset += STL_CIR_DATA_SIZE) < len);

				if (checkPort) break;
			}
		}
		
	} else {
		for_all_nodes(&old_topology, pNode) {
			for_all_ports(pNode, pPort) {
				if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) continue;
				if (!pPort->portData->cableInfo) continue;

				if (checkPort && portNum != pPort->index) continue;

				offset = 0;
				do {
					if ((status = sa_check_len(data, sizeof(STL_CABLE_INFO_RECORD), bytes)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						IB_LOG_ERROR_FMT("GetCableInfoRecord", "Reached size limit at %d records", *records);
						goto done;
					}
				
					record.LID = pNode->nodeInfo.NodeType == NI_TYPE_SWITCH ?
								pNode->port[0].portData->lid :
								pPort->portData->lid;
					record.Port = pPort->index;
					record.u1.s.Address = addr + offset;
					record.Length = MIN(len - offset, STL_CABLE_INFO_MAXLEN);
					record.u1.s.PortType = pPort->portData->portInfo.PortPhysConfig.s.PortType;
					bfrIdx = addr - STL_CIB_STD_HIGH_PAGE_ADDR + offset;
					memcpy(record.Data, &pPort->portData->cableInfo->buffer[bfrIdx], record.Length + 1);
					BSWAPCOPY_STL_CABLE_INFO_RECORD(&record, (STL_CABLE_INFO_RECORD*)data);
					(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_CABLE_INFO_RECORD), bytes, records);
				} while ((offset += STL_CIR_DATA_SIZE) < len);
				if (checkPort) break;
			}
		}
	}
	
done:
	(void)vs_rwunlock(&old_topology_lock);
	
	IB_EXIT("GetCableInfoRecord", VSTATUS_OK);
	return VSTATUS_OK;
}
