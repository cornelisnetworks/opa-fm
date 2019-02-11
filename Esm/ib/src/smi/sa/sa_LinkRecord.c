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

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_LinkRecord.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the LinkRecord type.					     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_LinkRecord							     //
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

Status_t	sa_LinkRecord_Get(Mai_t *, uint32_t *);
Status_t	sa_LinkRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_LinkRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
	uint32_t	records;
	uint32_t	attribOffset;

	IB_ENTER("sa_LinkRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetLinkRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblLinkRecord);
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
		(void)sa_LinkRecord_GetTable(maip, &records);
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
		IB_LOG_WARN("sa_LinkRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}

	attribOffset =  sizeof(STL_LINK_RECORD) + Calculate_Padding(sizeof(STL_LINK_RECORD));
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_LinkRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

// Returns TRUE if nodep1/PortNumP1 is the "From" port of the pair
bool_t
isFromPort(Node_t *nodep1, Node_t *nodep2, uint32_t PortNumP1, uint32_t PortNumP2) {

        if (nodep1->nodeInfo.NodeGUID != nodep2->nodeInfo.NodeGUID) {
		return (nodep1->nodeInfo.NodeGUID < nodep2->nodeInfo.NodeGUID) ? TRUE : FALSE;
	}

	return (PortNumP1 < PortNumP2) ? TRUE : FALSE;
}

Status_t
sa_LinkRecord_Set(uint8_t *lrp, Node_t *nodep, Port_t *portp) {
	Port_t		*pNeighborPort;
	Node_t		*pNeighborNode;
	STL_LINK_RECORD linkRecord = {{0}};

	IB_ENTER("sa_LinkRecord_Set", lrp, nodep, portp, 0);

	pNeighborPort = sm_find_neighbor_node_and_port(&old_topology, portp, &pNeighborNode);
	if (pNeighborPort == NULL) {
		return (VSTATUS_BAD);
	}

	linkRecord.RID.FromLID = (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) ? portp->portData->lid : nodep->port[0].portData->lid;
	linkRecord.RID.FromPort = portp->index;
	linkRecord.ToLID = (pNeighborNode->nodeInfo.NodeType != NI_TYPE_SWITCH) ? pNeighborPort->portData->lid : pNeighborNode->port[0].portData->lid;
	linkRecord.ToPort = portp->portno;

	
	BSWAPCOPY_STL_LINK_RECORD(&linkRecord, (STL_LINK_RECORD*)lrp);
	IB_EXIT("sa_LinkRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);

}

Status_t
sa_LinkRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t		*data;
	uint32_t	bytes;
	Node_t		*nodep;
	Port_t		*portp;
	STL_SA_MAD		samad;
	Status_t	status;

	IB_ENTER("sa_LinkRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_LINK_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_LINK_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_LINK_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_LINK_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_LinkRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_LINK_RECORD));


//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_LinkRecord_GetTable", VSTATUS_OK);
		return(VSTATUS_BAD);
	}

//
//	Find the LinkRecords in the SADB
//
	(void)vs_rdlock(&old_topology_lock);

	for_all_nodes(&old_topology, nodep) {
		for_all_physical_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
				continue;
			}

			if ((status = sa_check_len(data, sizeof(STL_LINK_RECORD), bytes)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( "sa_LinkRecord_GetTable",
					   "Reached size limit at %d records", *records);
				goto done;
			}


			// if status is VSTATUS_BAD, the query should abort.
			// if status is VSTATUS_OK, the record is good and should be returned.
			// if status is VSTATUS_IGNORE, it means the link record didn't meet the LinkCondition criteria and should be ignored, but it is not an error.
			if ((status = sa_LinkRecord_Set(data, nodep, portp)) == VSTATUS_BAD) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				goto done;
			}

			if (status == VSTATUS_OK)
				(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_LINK_RECORD), bytes, records);
		}
	}
done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_LinkRecord_GetTable", VSTATUS_OK);
	return(VSTATUS_OK);
}
