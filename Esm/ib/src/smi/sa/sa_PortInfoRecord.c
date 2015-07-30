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
//    sa_PortInfoRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the PortInfoRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_PortInfoRecord							     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
// RESPONSIBLE ENGINEER							     //
//    Jeff Young							     //
//									     //
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

Status_t	sa_PortInfoRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_PortInfoRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
	uint32_t			records;
	uint16_t			attribOffset;

	IB_ENTER("sa_PortInfoRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

//
//	Check the method.  If this is a template lookup, then call the regular
//	GetTable(*) template lookup routine.
//
	switch (maip->base.method) {
	case SA_CM_GET:
		INCREMENT_COUNTER(smCounterSaRxGetPortInfoRecord);
		(void)sa_PortInfoRecord_GetTable(maip, &records);
		break;
	case SA_CM_GETTABLE:
		INCREMENT_COUNTER(smCounterSaRxGetTblPortInfoRecord);
		(void)sa_PortInfoRecord_GetTable(maip, &records);
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_METHOD;
		(void)sa_send_reply(maip, sa_cntxt);
		IB_LOG_WARN("sa_PortInfoRecord: invalid METHOD:", maip->base.method);
		IB_EXIT("sa_PortInfoRecord", VSTATUS_OK);
		return VSTATUS_OK;
		break;
	}

//
//	Determine reply status
//
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_PortInfoRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset =  sizeof(STL_PORTINFO_RECORD) + Calculate_Padding(sizeof(STL_PORTINFO_RECORD));
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_PortInfoRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_PortInfoRecord_Set(uint8_t *prp, Node_t *nodep, Port_t *portp, STL_SA_MAD *samad) {
	uint32_t		    portno;
	Lid_t			    lid;
    Port_t              *piPortp;
	STL_PORTINFO_RECORD	portInfoRecord;

	IB_ENTER("sa_PortInfoRecord_Set", prp, nodep, portp, 0);

    if (portp == NULL) {
        IB_LOG_ERROR_FMT( "sa_PortInfoRecord_Set",
               "NULL port parameter for Node Guid["FMT_U64"], %s",
               nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_PortInfoRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
    }

	portno = (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : portp->index;

    if (!sm_valid_port((piPortp = sm_get_port(nodep,portno)))) {
        IB_LOG_ERROR0("sa_PortInfoRecord_Set: failed to get port");
        IB_EXIT("sa_PortInfoRecord_Set", VSTATUS_BAD);
        return(VSTATUS_BAD);
    }

	lid = piPortp->portData->lid;

	memset(&portInfoRecord,0,sizeof(portInfoRecord));

	portInfoRecord.RID.EndPortLID = lid;
	portInfoRecord.RID.PortNum = portp->index;
	portInfoRecord.PortInfo = portp->portData->portInfo;
	memcpy(portInfoRecord.LinkDownReasons, portp->portData->LinkDownReasons, sizeof(STL_LINKDOWN_REASON) * STL_NUM_LINKDOWN_REASONS);

    /* IBTA 1.2 C15-0.2.2 - zero out mKey if not trusted request */
	if (sm_smInfo.SM_Key && samad->header.smKey != sm_smInfo.SM_Key) {
        portInfoRecord.PortInfo.M_Key = 0ull;
    }

	BSWAP_STL_PORTINFO_RECORD(&portInfoRecord);
	memcpy(prp,&portInfoRecord,sizeof(STL_PORTINFO_RECORD));

	IB_EXIT("sa_PortInfoRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_PortInfoRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t		*data;
	uint32_t	bytes;
	Node_t		*nodep;
	Port_t		*portp;
	STL_SA_MAD	samad;
	Status_t	status;
	bool_t		checkLid;
	Lid_t		endPortLid=0;
	uint8_t		checkCapMask = 0;
	uint32_t	capMask = 0, cap;
	STL_PORTINFO_RECORD *portInfoRecp;

	IB_ENTER("sa_PortInfoRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_PORTINFO_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_PORTINFO_RECORD) ) {
		IB_LOG_ERROR_FMT("sa_PortInfoRecord_GetTable",
						 "invalid MAD length; size of STL_PORTINFO_RECORD[%lu], datasize[%d]", sizeof(STL_PORTINFO_RECORD), maip->datasize-sizeof(STL_SA_MAD_HEADER));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_PortInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_PORTINFO_RECORD));

	portInfoRecp = (STL_PORTINFO_RECORD *)samad.data;

	// IBTA 1.2.1 15.2.5.3 - We set the CapMaskMatchSupported bit in our SA ClassPortInfo
	checkLid = (samad.header.mask & STL_PORTINFO_RECORD_COMP_ENDPORTLID);
	if (checkLid) {	
		endPortLid = ntoh32(portInfoRecp->RID.EndPortLID);
		samad.header.mask ^= STL_PORTINFO_RECORD_COMP_ENDPORTLID;
	}

	// IBTA 1.2.1 15.2.5.3 - We set the CapMaskMatchSupported bit in our SA ClassPortInfo
	// So, if bit 31 of amod is set, matching of capmask
    // should be done only on those bits in the PortInfo:CapabilityMask
	// embedded in the query
	if ((samad.header.mask & STL_PORTINFO_RECORD_COMP_CAPABILITYMASK) && (maip->base.amod & (1 << 31))) {
		checkCapMask = 1;
		memcpy(&cap, &samad.data[24], 4);
		capMask = ntoh32(cap);
		samad.header.mask ^= STL_PORTINFO_RECORD_COMP_CAPABILITYMASK;
	}

//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_PortInfoRecord_GetTable", status);
		return(status);
	}

//
//      Load the PortInfoRecords in the SADB
//
	(void)vs_rdlock(&old_topology_lock);
    /* 
     * Return busy if source lid not in topology yet 
     * This should throttle back the host
     */
	if ( (portp = sm_find_active_port_lid(&old_topology, maip->addrInfo.slid)) == NULL) {
		maip->base.status = isSweeping ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
        (void)vs_rwunlock(&old_topology_lock);
        IB_EXIT("sa_PortInfoRecord_GetTable", VSTATUS_OK);
        return(VSTATUS_OK);
    }

	if (checkLid) {
		Port_t		*matched_portp;
		if ((matched_portp = sm_find_node_and_port_lid(&old_topology, endPortLid, &nodep)) != NULL) {
			for_all_matched_ports(nodep, portp, matched_portp) {
				if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
					continue;
				}

				if (checkCapMask) {
					if ((capMask & portp->portData->portInfo.CapabilityMask.AsReg32) != capMask)
						continue;
				}

				if ((status = sa_check_len(data, sizeof(STL_PORTINFO_RECORD), bytes)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_ERROR_FMT( "sa_PortInfoRecord_GetTable", "Bad buffer");
					goto done;
				}
				if ((status = sa_PortInfoRecord_Set(data, nodep, portp, &samad)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					goto done;
				}
				// if more mask bits, check for match, else give them all ports that are not down
				(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_PORTINFO_RECORD), bytes, records);
			}
		}
	} else {
		for_all_nodes(&old_topology, nodep) {
			for_all_ports(nodep, portp) {
				if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {  // used to be PORT_NOT_MINE
					continue;
				}

				if (checkCapMask) {
					if ((capMask & portp->portData->portInfo.CapabilityMask.AsReg32) != capMask)
						continue;
				}
	
				if ((status = sa_check_len(data, sizeof(STL_PORTINFO_RECORD), bytes)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_ERROR_FMT( "sa_PortInfoRecord_GetTable",
					   	"Reached size limit at %d records", *records);
					goto done;
				}
	
				if ((status = sa_PortInfoRecord_Set(data, nodep, portp, &samad)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					goto done;
				}
	
				
				(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_PORTINFO_RECORD), bytes, records);
			}
		}
	}

done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_PortInfoRecord_GetTable", status);
	return(status);
}
