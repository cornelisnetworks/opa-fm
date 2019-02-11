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
//    sa_VLArbitrationRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the VLArbitrationRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_VLArbitrationRecord						     //
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

Status_t	sa_VLArbitrationRecord_GetTable(Mai_t *, uint32_t *);

Status_t
sa_VLArbitrationRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	records;
	uint16		attribLen;

	IB_ENTER("sa_VLArbitrationRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetVlArbTableRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblVlArbTableRecord);
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
		(void)sa_VLArbitrationRecord_GetTable(maip, &records);
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
		IB_LOG_WARN("sa_VLArbitrationRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
	attribLen = sizeof(STL_VLARBTABLE_RECORD) + Calculate_Padding(sizeof(STL_VLARBTABLE_RECORD));
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = attribLen;
	sa_cntxt_data(sa_cntxt, sa_data, records * attribLen);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_VLArbitrationRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_VLArbitrationRecord_Set(uint8_t *vlarbp, Node_t *nodep, Port_t *portp, int ind) {
	STL_LID 			        lid;
	STL_VLARBTABLE_RECORD		vlArbRecord;

    if (portp == NULL) {
        IB_LOG_ERROR_FMT( "sa_VLArbitrationRecord_Set",
               "NULL port parameter for Node Guid["FMT_U64"], %s",
               nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		IB_EXIT("sa_VLArbitrationRecord_Set", VSTATUS_BAD);
		return(VSTATUS_BAD);
    }

	memset((char *)&vlArbRecord, 0, sizeof(vlArbRecord));
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t   	               *vlPortp;
    	if (!sm_valid_port((vlPortp = sm_get_port(nodep,0)))) {
        	IB_LOG_ERROR_FMT( "sa_VLArbitrationRecord_Set",
               	"failed to get port %d for Node Guid["FMT_U64"], %s",
               	0, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
                	IB_EXIT("sa_VLArbitrationRecord_Set", VSTATUS_BAD);
                	return(VSTATUS_BAD);
		}
		lid = vlPortp->portData->lid;
    } else {
		lid = portp->portData->lid;
	}


	vlArbRecord.RID.LID = lid;
	vlArbRecord.RID.OutputPortNum = portp->index;
	vlArbRecord.RID.BlockNum = ind;
    switch (ind) {
        case STL_VLARB_LOW_ELEMENTS:
            memcpy(vlArbRecord.VLArbTable.Elements, portp->portData->curArb.u.vlarb.vlarbLow, sizeof(portp->portData->curArb.u.vlarb.vlarbLow));
            break;
        case STL_VLARB_HIGH_ELEMENTS:
            memcpy(vlArbRecord.VLArbTable.Elements, portp->portData->curArb.u.vlarb.vlarbHigh, sizeof(portp->portData->curArb.u.vlarb.vlarbHigh));
            break;
        case STL_VLARB_PREEMPT_ELEMENTS:
            memcpy(vlArbRecord.VLArbTable.Elements, portp->portData->curArb.u.vlarb.vlarbPre, sizeof(portp->portData->curArb.u.vlarb.vlarbPre));
            break;
        case STL_VLARB_PREEMPT_MATRIX:
            memcpy(vlArbRecord.VLArbTable.Matrix, portp->portData->curArb.vlarbMatrix, sizeof(portp->portData->curArb.vlarbMatrix));
            break;
    }
	BSWAPCOPY_STL_VLARBTABLE_RECORD(&vlArbRecord, (STL_VLARBTABLE_RECORD *)vlarbp);

	IB_EXIT("sa_VLArbitrationRecord_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_VLArbitrationRecord_GetTable(Mai_t *maip, uint32_t *records) {
	int			i;
	uint8_t		*data;
	uint32_t	bytes;
	Node_t		*nodep;
	Port_t		*portp;
	STL_SA_MAD	samad;
	Status_t	status;
	STL_VLARBTABLE_RECORD vla;
	bool_t		checkLid;
	STL_LID		portLid=0;

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_VLARBTABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_VLARBTABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_VLARBTABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_VLARBTABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_VLArbitrationRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_VLARBTABLE_RECORD));
	BSWAPCOPY_STL_VLARBTABLE_RECORD((STL_VLARBTABLE_RECORD*)samad.data, &vla);

	checkLid = (samad.header.mask & STL_VLARB_COMPONENTMASK_LID);
	if (checkLid) {
		portLid = vla.RID.LID;
		samad.header.mask ^= STL_VLARB_COMPONENTMASK_LID;
	}

//
//	Create the template mask for the lookup.
//
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_VLArbitrationRecord_GetTable", status);
		return(status);
	}

//
//      Find all VLArbitrationRecords which match the template.
//
	(void)vs_rdlock(&old_topology_lock);

	if (checkLid) {
		Port_t *matched_portp;
		if ((matched_portp = sm_find_node_and_port_lid(&old_topology, portLid, &nodep)) != NULL) {
			if (!nodep->vlArb) {
				maip->base.status = MAD_STATUS_SA_REQ_INVALID;
				goto done;
			}

			for_all_matched_ports(nodep, portp, matched_portp) {
				if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;

				if (portp->index == 0) {			// Not a physical port
					continue;
				}

				for (i = 0; i < 4; ++i) {
					if ((status = sa_check_len(data, sizeof(STL_VLARBTABLE_RECORD), bytes)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						IB_LOG_ERROR_FMT( "sa_VLArbitrationRecord_GetTable",
						   	"Reached size limit at %d records", *records);
						goto done;
					}

					if ((status = sa_VLArbitrationRecord_Set(data, nodep, portp, i)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						goto done;
                   	}

					(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_VLARBTABLE_RECORD), bytes, records);
				}
			}
		}
	} else {
		for_all_nodes(&old_topology, nodep) {
			if (!nodep->vlArb) continue;

			for_all_ports(nodep, portp) {
				if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;

				if (portp->index == 0) {			// Not a physical port
					continue;
				}

				for (i = 0; i < 4; ++i) {
					if ((status = sa_check_len(data, sizeof(STL_VLARBTABLE_RECORD), bytes)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						IB_LOG_ERROR_FMT( "sa_VLArbitrationRecord_GetTable",
						   	"Reached size limit at %d records", *records);
						goto done;
					}

					if ((status = sa_VLArbitrationRecord_Set(data, nodep, portp, i)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						goto done;
                   	}

					(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_VLARBTABLE_RECORD), bytes, records);
				}
			}
		}
	}

done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_VLArbitrationRecord_GetTable", status);
	return(status);
}
