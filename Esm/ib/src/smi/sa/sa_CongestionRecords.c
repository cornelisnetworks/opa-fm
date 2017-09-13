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
//    sa_CongestionRecords.c                                                 //
//                                                                           //
// DESCRIPTION                                                               //
//    This file contains the routines to process the SA requests for         //
//    records of the Buffer Control Table Record type.                       //
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


static Status_t sa_CongInfoRecord_GetTable(Mai_t *maip, uint32_t *records);
static Status_t sa_SwitchCongRecord_GetTable(Mai_t *maip, uint32_t *records);
static Status_t sa_SwitchPortCongRecord_GetTable(Mai_t *maip, uint32_t *records);
static Status_t sa_HFICongRecord_GetTable(Mai_t *maip, uint32_t *records);
static Status_t sa_HFICongCtrlRecord_GetTable(Mai_t *maip, uint32_t *records);



Status_t
sa_CongInfoRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
    uint32_t    records;
    uint32_t    attribOffset;

    IB_ENTER("sa_CongInfoRecord", maip, 0, 0, 0);

    records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetCongInfoRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblCongInfoRecord);
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
		(void)sa_CongInfoRecord_GetTable(maip, &records);
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
        IB_LOG_WARN("sa_CongInfoRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_CONGESTION_INFO_RECORD) + Calculate_Padding(sizeof(STL_CONGESTION_INFO_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_CongInfoRecord", VSTATUS_OK);
    return(VSTATUS_OK);
}



Status_t
sa_SwitchCongRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
    uint32_t    records;
    uint32_t    attribOffset;

    IB_ENTER("sa_SwitchCongRecord", maip, 0, 0, 0);
	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetSwitchCongRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblSwitchCongRecord);
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
		(void)sa_SwitchCongRecord_GetTable(maip, &records);
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
        IB_LOG_WARN("sa_SwitchCongRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD) + Calculate_Padding(sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_SwitchCongRecord", VSTATUS_OK);
    return(VSTATUS_OK);
}

Status_t
sa_SwitchPortCongRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
    uint32_t    records;
    uint32_t    attribOffset;

    IB_ENTER("sa_SwitchPortCongRecord", maip, 0, 0, 0);

    records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetSwitchPortCongRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblSwitchPortCongRecord);
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
		(void)sa_SwitchPortCongRecord_GetTable(maip, &records);
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
        IB_LOG_WARN("sa_SwitchPortCongRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD) + Calculate_Padding(sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_SwitchPortCongRecord", VSTATUS_OK);
    return(VSTATUS_OK);
}

Status_t
sa_HFICongRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
    uint32_t    records;
    uint32_t    attribOffset;

    IB_ENTER("sa_HFICongRecord", maip, 0, 0, 0);

    records = 0;
	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetHFICongRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblHFICongRecord);
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
		(void)sa_HFICongRecord_GetTable(maip, &records);
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
        IB_LOG_WARN("sa_HFICongRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_HFI_CONGESTION_SETTING_RECORD) + Calculate_Padding(sizeof(STL_HFI_CONGESTION_SETTING_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_HFICongRecord", VSTATUS_OK);
    return(VSTATUS_OK);
}

Status_t
sa_HFICongCtrlRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
    uint32_t    records;
    uint32_t    attribOffset;

    IB_ENTER("sa_HFICongCtrlRecord", maip, 0, 0, 0);

    records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetHFICongCtrlRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblHFICongCtrlRecord);
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
		(void)sa_HFICongCtrlRecord_GetTable(maip, &records);
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
        IB_LOG_WARN("sa_HFICongCtrlRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD) + Calculate_Padding(sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_HFICongCtrlRecord", VSTATUS_OK);
    return(VSTATUS_OK);
}


Status_t
sa_CongInfoRecord_GetTable(Mai_t *maip, uint32_t *records) {
    uint8_t     *data;
    uint32_t    bytes;
    Node_t      *pNode;
    Port_t      *pPort;
    STL_SA_MAD  samad;
    Status_t    status;
    bool_t      checkLid;
    Lid_t       lid=0;
    STL_CONGESTION_INFO_RECORD record;

    IB_ENTER("sa_CongInfoRecord_GetTable", maip, *records, 0, 0);

    *records = 0;

	// Don't bother if congestion is disabled.
	if (!sm_config.congestion.enable) return VSTATUS_OK;
	
    data = sa_data;
    bytes = Calculate_Padding(sizeof(STL_CONGESTION_INFO_RECORD));


//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_CONGESTION_INFO_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_CONGESTION_INFO_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_CONGESTION_INFO_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_CongInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_CONGESTION_INFO_RECORD));

    checkLid = (samad.header.mask & CIR_COMPONENTMASK_COMP_LID);
    if (checkLid) { 
        lid = ntoh32(((STL_CONGESTION_INFO_RECORD*)(samad.data))->LID);
    }

    //  Create the template mask for the lookup.
    status = sa_create_template_mask(maip->base.aid, samad.header.mask);
    if (status != VSTATUS_OK) {
        IB_EXIT("sa_CongInfoRecord_GetTable", VSTATUS_OK);
        return(VSTATUS_OK);
    }

    //  Find the Congestion Info in the SADB
    (void)vs_rdlock(&old_topology_lock);

    if (checkLid) {
        if ((pPort = sm_find_node_and_port_lid(&old_topology, lid, &pNode)) != NULL) {
            if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) goto done;
            if ((status = sa_check_len(data, sizeof(STL_CONGESTION_INFO_RECORD), bytes)) != VSTATUS_OK) {
                maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                IB_LOG_ERROR_FMT( "sa_CongInfoRecord_GetTable","Reached size limit at %d records", *records);
                goto done;
            }

            memset(&record, 0, sizeof(record));
            record.LID = lid;
            record.CongestionInfo = pNode->congestionInfo;
            BSWAPCOPY_STL_CONGESTION_INFO_RECORD(&record, (STL_CONGESTION_INFO_RECORD*)data);
            (void)sa_template_test(samad.data, &data, sizeof(STL_CONGESTION_INFO_RECORD), bytes, records);
        }
    } else {
        // All nodes get reported
        for_all_nodes(&old_topology, pNode) {
            for_all_ports(pNode, pPort) {
                if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) continue;
                if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) && (pPort->index!=0)) break;
                if ((status = sa_check_len(data, sizeof(STL_CONGESTION_INFO_RECORD), bytes)) != VSTATUS_OK) {
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                    IB_LOG_ERROR_FMT( "sa_CongInfoRecord_GetTable", "Reached size limit at %d records", *records);
                    goto done;
                }

                memset(&record, 0, sizeof(record));
                record.LID = pPort->portData->lid;
                record.CongestionInfo = pNode->congestionInfo;
                BSWAPCOPY_STL_CONGESTION_INFO_RECORD(&record, (STL_CONGESTION_INFO_RECORD*)data);
                (void)sa_template_test(samad.data, &data, sizeof(STL_CONGESTION_INFO_RECORD), bytes, records);
            }
        }
    }

done:
    (void)vs_rwunlock(&old_topology_lock);

    IB_EXIT("sa_CongInfoRecord_GetTable", VSTATUS_OK);
    return(VSTATUS_OK);
}


Status_t
sa_SwitchCongRecord_GetTable(Mai_t *maip, uint32_t *records) {
    uint8_t     *data;
    uint32_t    bytes;
    Node_t      *pNode;
    Port_t      *pPort;
    STL_SA_MAD  samad;
    Status_t    status;
    bool_t      checkLid;
    Lid_t       lid=0;

    STL_SWITCH_CONGESTION_SETTING_RECORD record;

    IB_ENTER("sa_SwitchCongRecord_GetTable", maip, *records, 0, 0);

    *records = 0;

	// Don't bother if congestion is disabled.
	if (!sm_config.congestion.enable) return VSTATUS_OK;

    data = sa_data;
    bytes = Calculate_Padding(sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SWITCH_CONGESTION_SETTING_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_SwitchCongRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD));

    checkLid = (samad.header.mask & SWCSR_COMPONENTMASK_COMP_LID);
    if (checkLid) { 
        lid = ntoh32(((STL_SWITCH_CONGESTION_SETTING_RECORD*)(samad.data))->LID);
    }

    //  Create the template mask for the lookup.
    status = sa_create_template_mask(maip->base.aid, samad.header.mask);
    if (status != VSTATUS_OK) {
        IB_EXIT("sa_SwitchCongRecord_GetTable", VSTATUS_OK);
        return(VSTATUS_OK);
    }

    //  Find the Congestion Records in the SADB
    (void)vs_rdlock(&old_topology_lock);

    if (checkLid) {
        if ((pPort = sm_find_node_and_port_lid(&old_topology, lid, &pNode)) != NULL) {
            if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) goto done;
            if (pNode->nodeInfo.NodeType != NI_TYPE_SWITCH) goto done;
            if ((status = sa_check_len(data, sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD), bytes)) != VSTATUS_OK) {
                maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                IB_LOG_ERROR_FMT( "sa_SwitchCongRecord_GetTable", "Reached size limit at %d records", *records);
                goto done;
            }

            memset(&record, 0, sizeof(record));
            record.LID = lid;
            record.SwitchCongestionSetting = pNode->swCongestionSetting;
            BSWAPCOPY_STL_SWITCH_CONGESTION_SETTING_RECORD(&record, (STL_SWITCH_CONGESTION_SETTING_RECORD*)data);
            (void)sa_template_test(samad.data, &data, sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD), bytes, records);
        }
    } else {
        for_all_switch_nodes(&old_topology, pNode) {
            if (!sm_valid_port((pPort = sm_get_port(pNode,0)))) continue;
            if ((status = sa_check_len(data, sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD), bytes)) != VSTATUS_OK) {
                maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                IB_LOG_ERROR_FMT( "sa_SwitchCongRecord_GetTable", "Reached size limit at %d records", *records);
                goto done;
            }

            memset(&record, 0, sizeof(record));
            record.LID = pPort->portData->lid;
            record.SwitchCongestionSetting = pNode->swCongestionSetting;
            BSWAPCOPY_STL_SWITCH_CONGESTION_SETTING_RECORD(&record, (STL_SWITCH_CONGESTION_SETTING_RECORD*)data);
            (void)sa_template_test(samad.data, &data, sizeof(STL_SWITCH_CONGESTION_SETTING_RECORD), bytes, records);
        }
    }

done:
    (void)vs_rwunlock(&old_topology_lock);

    IB_EXIT("sa_SwitchCongRecord_GetTable", VSTATUS_OK);
    return(VSTATUS_OK);
}


Status_t
sa_SwitchPortCongRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t		 *data;
	uint32_t	 bytes;
	Node_t		*pNode;
	Port_t		*pPort;
	STL_SA_MAD	samad;
	Status_t 	status;
	bool_t		checkLid;
	Lid_t		lid=0;
	STL_SWITCH_PORT_CONGESTION_SETTING_RECORD record = {{0}};
	
	IB_ENTER(__func__, maip, *records, 0, 0);

	*records = 0;

	// Don't bother if congestion is disabled.
	if (!sm_config.congestion.enable) return VSTATUS_OK;

	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD));
	
	// Verify the size of the data received for the request.
	if (maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SWITCH_PORT_CONGESTION_SETTING_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT(__func__, MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD));
	
	checkLid = (samad.header.mask & SWPCSR_COMPONENTMASK_COMP_LID);
	
	if (checkLid) {
		lid = ntoh32(((STL_SWITCH_PORT_CONGESTION_SETTING_RECORD*)(samad.data))->RID.LID);
		samad.header.mask ^= SWPCSR_COMPONENTMASK_COMP_LID;
	}
	
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}

	(void)vs_rdlock(&old_topology_lock);

	for_all_switch_nodes(&old_topology, pNode) {
		if (checkLid) {
			Port_t *tempPort;

			// Override pNode. We short circuit out of this loop later.
			tempPort = sm_find_node_and_port_lid(&old_topology, lid, &pNode);
			if (!sm_valid_port(tempPort) || tempPort->state <= IB_PORT_DOWN) goto done;
			if (pNode->nodeInfo.NodeType != NI_TYPE_SWITCH) goto done;
		}
		for_all_ports(pNode, pPort) {
			if (!sm_valid_port(pPort)) continue;
			if ((status = sa_check_len(data, sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD), bytes)) != VSTATUS_OK) {
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
				IB_LOG_ERROR_FMT( __func__, "Reached size limit at %d records", *records);
				goto done;
           	}
			record.RID.LID = pNode->port[0].portData->lid;
			record.RID.Port = pPort->index;
			record.SwitchPortCongestionSetting.Elements[0] = pPort->portData->swPortCongSet;
			BSWAPCOPY_STL_SWITCH_PORT_CONGESTION_SETTING_RECORD(&record, (STL_SWITCH_PORT_CONGESTION_SETTING_RECORD*)data);
			(void)sa_template_test(samad.data, &data, sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD), bytes, records);
		}
		
		if (checkLid) break;
    }

done:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT(__func__, VSTATUS_OK);
    return(VSTATUS_OK);
}

Status_t
sa_HFICongRecord_GetTable(Mai_t *maip, uint32_t *records) {
    uint8_t     *data;
    uint32_t    bytes;
    Node_t      *pNode;
    Port_t      *pPort;
    STL_SA_MAD  samad;
    Status_t    status;
    bool_t      checkLid;
    Lid_t       lid=0;

    STL_HFI_CONGESTION_SETTING_RECORD record;

    IB_ENTER("sa_HFICongRecord_GetTable", maip, *records, 0, 0);

    *records = 0;

	// Don't bother if congestion is disabled.
	if (!sm_config.congestion.enable) return VSTATUS_OK;

    data = sa_data;
    bytes = Calculate_Padding(sizeof(STL_HFI_CONGESTION_SETTING_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_HFI_CONGESTION_SETTING_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_HFI_CONGESTION_SETTING_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_HFI_CONGESTION_SETTING_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_HFICongRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_HFI_CONGESTION_SETTING_RECORD));

    checkLid = (samad.header.mask & HCSR_COMPONENTMASK_COMP_LID);
    if (checkLid) { 
        lid = ntoh32(((STL_HFI_CONGESTION_SETTING_RECORD*)(samad.data))->LID);
    }

    //  Create the template mask for the lookup.
    status = sa_create_template_mask(maip->base.aid, samad.header.mask);
    if (status != VSTATUS_OK) {
        IB_EXIT("sa_HFICongRecord_GetTable", VSTATUS_OK);
        return(VSTATUS_OK);
    }

    //  Find the Congestion Info in the SADB
    (void)vs_rdlock(&old_topology_lock);

    if (checkLid) {
        if ((pPort = sm_find_node_and_port_lid(&old_topology, lid, &pNode)) != NULL) {
            if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) goto done;
            if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
               (!pNode->switchInfo.u2.s.EnhancedPort0)) goto done;
            if ((status = sa_check_len(data, sizeof(STL_HFI_CONGESTION_SETTING_RECORD), bytes)) != VSTATUS_OK) {
                maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                IB_LOG_ERROR_FMT( "sa_HFICongRecord_GetTable", "Reached size limit at %d records", *records);
                goto done;
            }

            memset(&record, 0, sizeof(record));
            record.LID = lid;
            record.HFICongestionSetting = pNode->hfiCongestionSetting;
            BSWAPCOPY_STL_HFI_CONGESTION_SETTING_RECORD(&record, (STL_HFI_CONGESTION_SETTING_RECORD*)data);
            (void)sa_template_test(samad.data, &data, sizeof(STL_HFI_CONGESTION_SETTING_RECORD), bytes, records);
        }
    } else {
        for_all_nodes(&old_topology, pNode) {
            if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
               (!pNode->switchInfo.u2.s.EnhancedPort0)) continue;

            for_all_ports(pNode, pPort) {
                if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) continue;
                if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) && (pPort->index!=0)) break;

                if ((status = sa_check_len(data, sizeof(STL_HFI_CONGESTION_SETTING_RECORD), bytes)) != VSTATUS_OK) {
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                    IB_LOG_ERROR_FMT( "sa_HFICongRecord_GetTable", "Reached size limit at %d records", *records);
                    goto done;
                }

                memset(&record, 0, sizeof(record));
                record.LID = pPort->portData->lid;
                record.HFICongestionSetting = pNode->hfiCongestionSetting;
                BSWAPCOPY_STL_HFI_CONGESTION_SETTING_RECORD(&record, (STL_HFI_CONGESTION_SETTING_RECORD*)data);
                (void)sa_template_test(samad.data, &data, sizeof(STL_HFI_CONGESTION_SETTING_RECORD), bytes, records);
            }

        }
    }

done:
    (void)vs_rwunlock(&old_topology_lock);

    IB_EXIT("sa_HFICongRecord_GetTable", VSTATUS_OK);
    return(VSTATUS_OK);
}


Status_t
sa_HFICongCtrlRecord_GetTable(Mai_t *maip, uint32_t *records) {
    uint8_t     *data;
    uint32_t    bytes;
    Node_t      *pNode;
    Port_t      *pPort;
    STL_SA_MAD  samad;
    Status_t    status;
    bool_t      checkLid;
    Lid_t       lid=0;
    bool_t      checkBlock;
    uint16_t    blockNum=0;
    uint16_t    numBlocks=0;

    STL_HFI_CONGESTION_CONTROL_TABLE_RECORD record;

    IB_ENTER("sa_HFICongCtrlRecord_GetTable", maip, *records, 0, 0);

    *records = 0;

	// Don't bother if congestion is disabled.
	if (!sm_config.congestion.enable) return VSTATUS_OK;

    data = sa_data;
    bytes = Calculate_Padding(sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_HFI_CONGESTION_CONTROL_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_HFICongCtrlRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD));

    checkLid = (samad.header.mask & HCCTR_COMPONENTMASK_COMP_LID);
    if (checkLid) { 
        lid = ntoh32(((STL_HFI_CONGESTION_CONTROL_TABLE_RECORD*)(samad.data))->RID.LID);
    }

    checkBlock = (samad.header.mask & HCCTR_COMPONENTMASK_COMP_BLOCK);
    if (checkBlock) {    
        blockNum = ntoh16(((STL_HFI_CONGESTION_CONTROL_TABLE_RECORD*)(samad.data))->RID.BlockNum);
    }

    //  Create the template mask for the lookup.
    status = sa_create_template_mask(maip->base.aid, samad.header.mask);
    if (status != VSTATUS_OK) {
        IB_EXIT("sa_HFICongCtrlRecord_GetTable", VSTATUS_OK);
        return(VSTATUS_OK);
    }

    //  Find the BufferControlTableRecords in the SADB
    (void)vs_rdlock(&old_topology_lock);

    if (checkLid) {
        if ((pPort = sm_find_node_and_port_lid(&old_topology, lid, &pNode)) != NULL) {
            if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) goto done;
            if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
               (!pNode->switchInfo.u2.s.EnhancedPort0)) goto done;
            if (pPort->portData->hfiCongCon == 0) goto done;

            numBlocks = pPort->portData->hfiCongCon->CCTI_Limit/STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES;
            if (checkBlock && blockNum>numBlocks) {
                maip->base.status = MAD_STATUS_SA_REQ_INVALID;
                IB_LOG_ERROR_FMT(__func__, "Requested block (%u) does not exist", blockNum);
                goto done;
            }
            memset(&record, 0, sizeof(record));
            record.RID.LID = lid;
            record.HFICongestionControlTable.CCTI_Limit = pPort->portData->hfiCongCon->CCTI_Limit;
            for (;blockNum<=numBlocks; blockNum++) {
                if ((status = sa_check_len(data, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD), bytes)) != VSTATUS_OK) {
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                    IB_LOG_ERROR_FMT( "sa_HFICongCtrlRecord_GetTable", "Reached size limit at %d records", *records);
                    goto done;
                }

                record.RID.BlockNum = blockNum;
                memcpy(&record.HFICongestionControlTable.CCT_Block_List[0],
                       &pPort->portData->hfiCongCon->CCT_Block_List[blockNum], 
                       sizeof(record.HFICongestionControlTable.CCT_Block_List));
                BSWAPCOPY_STL_HFI_CONGESTION_CONTROL_TABLE_RECORD(&record, (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD*)data);
                (void)sa_template_test(samad.data, &data, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD), bytes, records);
                if (checkBlock) goto done;// Return only requested block if specified.
            }
        }
    } else {
        for_all_nodes(&old_topology, pNode) {
            if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
               (!pNode->switchInfo.u2.s.EnhancedPort0)) continue;

            for_all_ports(pNode, pPort) {
                if (!sm_valid_port(pPort) || pPort->state <= IB_PORT_DOWN) continue;
                if ((pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) && (pPort->index!=0)) break;
                if (pPort->portData->hfiCongCon == 0) continue;
    
                numBlocks = pPort->portData->hfiCongCon->CCTI_Limit/STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES;
                memset(&record, 0, sizeof(record));
                record.RID.LID = pPort->portData->lid;
                record.HFICongestionControlTable.CCTI_Limit = pPort->portData->hfiCongCon->CCTI_Limit;
                for (blockNum=0 ;blockNum<=numBlocks; blockNum++) {
                    if ((status = sa_check_len(data, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD), bytes)) != VSTATUS_OK) {
                        maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                        IB_LOG_ERROR_FMT( "sa_HFICongCtrlRecord_GetTable", "Reached size limit at %d records", *records);
                        goto done;
                    }

                    record.RID.BlockNum = blockNum;
                    memcpy(&record.HFICongestionControlTable.CCT_Block_List[0],
                           &pPort->portData->hfiCongCon->CCT_Block_List[blockNum], 
                           sizeof(record.HFICongestionControlTable.CCT_Block_List));
                    BSWAPCOPY_STL_HFI_CONGESTION_CONTROL_TABLE_RECORD(&record, (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD*)data);
                    (void)sa_template_test(samad.data, &data, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD), bytes, records);
                }
            }
        }
    }

done:
    (void)vs_rwunlock(&old_topology_lock);

    IB_EXIT("sa_HFICongCtrlRecord_GetTable", VSTATUS_OK);
    return(VSTATUS_OK);
}


