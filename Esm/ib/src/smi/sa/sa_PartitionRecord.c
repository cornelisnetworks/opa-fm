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
//   sa_PartitionRecord.c                                                    //
//                                                                           //
// DESCRIPTION                                                               //
//   This file contains the routines to process the SA requests for          //
//   records of the PartitionRecord type.                                    //
//                                                                           //
// DATA STRUCTURES                                                           //
//   None                                                                    //
//                                                                           //
// FUNCTIONS                                                                 //
//   sa_PartitionRecord                                                      //
//                                                                           //
// DEPENDENCIES                                                              //
//   ib_mad.h                                                                //
//   ib_status.h                                                             //
//                                                                           //
//                                                                           //
//===========================================================================*/


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "stl_print.h"

/* *******************************************************************
 */
static void
sa_setPartitionRecord(uint8_t * data, Node_t * node, Port_t * port, int block)
{
	STL_P_KEY_TABLE_RECORD record;
	int pkey_idx = 0, i = 0;
	Port_t *swport;
    	Lid_t lid;

	memset(&record, 0, sizeof(record));

    if ((node->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
        (sm_valid_port((swport = sm_get_port(node,0))))) {
       	lid = swport->portData->lid;
	} else {
    	lid = port->portData->lid;
    }

	record.RID.LID = lid;
	record.RID.Blocknum = block;
	record.RID.PortNum = port->index;

	for (pkey_idx = (block * PKEY_TABLE_LIST_COUNT), i = 0;
	     pkey_idx < port->portData->num_pkeys; ++pkey_idx, ++i)
	{
		record.PKeyTblData.PartitionTableBlock[i] = port->portData->pPKey[pkey_idx];
	}

	BSWAPCOPY_STL_PARTITION_TABLE_RECORD(&record, (STL_P_KEY_TABLE_RECORD *)data);
}

/* *******************************************************************
 */
static Status_t
sa_getPartitionRecordTable(Mai_t *maip, uint32_t * records)
{
	Status_t status = VSTATUS_OK;
	Node_t * node = NULL;
	Port_t * port = NULL;
	STL_SA_MAD samad;
	uint8_t * data = NULL;
	int block = 0, bytes = 0;
	bool_t		checkLid;
	uint16_t	portLid=0;
	bool_t		checkPort;
	uint8_t		portNum=0;
	STL_P_KEY_TABLE_RECORD *pPartitionRec;

	IB_ENTER("sa_getPartitionRecordTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_P_KEY_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_P_KEY_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_P_KEY_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_P_KEY_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_getPartitionRecordTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_P_KEY_TABLE_RECORD));
	pPartitionRec = (STL_P_KEY_TABLE_RECORD *)samad.data;

	checkLid = (samad.header.mask & PKEY_COMPONENTMASK_PORTLID);
	if (checkLid) {	
		portLid = ntoh32(pPartitionRec->RID.LID);
		samad.header.mask ^= PKEY_COMPONENTMASK_PORTLID;
	}

	checkPort = (samad.header.mask & PKEY_COMPONENTMASK_PORTNUM);
	if (checkPort) {	
		portNum = pPartitionRec->RID.PortNum;
		samad.header.mask ^= PKEY_COMPONENTMASK_PORTNUM;
	}

	/* C15-0.2.2 - PKeyTableRecords and ServiceAssociationRecords shall only
	 * be provided in response to trusted requests
	 */
	if (sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key)
	{
		status = VSTATUS_INSUFFICIENT_PERMISSION;
		IB_EXIT("sa_getPartitionRecordTable", status);
		return status;
	}

	/* Create the template mask for the lookup. */
	if ((status = sa_create_template_mask(maip->base.aid,
	                                      samad.header.mask)) != VSTATUS_OK)
	{
		IB_EXIT("sa_getPartitionRecordTable", status);
		return status;
	}

	/* lock the topology while we do our dirty work */
	if ((status = vs_rdlock(&old_topology_lock)) != VSTATUS_OK)
	{
		IB_EXIT("sa_getPartitionRecordTable", status);
		return status;
	}

	if (checkLid) {
		Port_t *matched_portp;
		if ((matched_portp = sm_find_node_and_port_lid(&old_topology, portLid, &node)) != NULL) {
			for_all_matched_ports(node, port, matched_portp) {
				if (!sm_valid_port(port) || port->state <= IB_PORT_DOWN) continue;
				if (checkPort && portNum != port->index) continue;

				for (block = 0; (block * PKEY_TABLE_LIST_COUNT) < port->portData->num_pkeys; ++block) {
					if ((status = sa_check_len(data, sizeof(STL_P_KEY_TABLE_RECORD), bytes)) != VSTATUS_OK) {
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						IB_LOG_ERROR_FMT( "sa_getPartitionRecordTable",
				   			"Reached size limit at %d records", *records);
						break;
					}
	
					sa_setPartitionRecord(data, node, port, block);
	
					(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_P_KEY_TABLE_RECORD), bytes, records);
				}
				if (checkPort) break;
			}
		}
	} else {
		for_all_nodes(&old_topology, node)
		{
			for_all_ports(node, port)
			{
				if (!sm_valid_port(port) || port->state <= IB_PORT_DOWN) continue;
				if (checkPort && portNum != port->index) continue;
	
				for (block = 0; (block * PKEY_TABLE_LIST_COUNT) < port->portData->num_pkeys;
			     	++block)
				{
					if ((status = sa_check_len(data, sizeof(STL_P_KEY_TABLE_RECORD),
				                           	bytes)) != VSTATUS_OK)
					{
						maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
						IB_LOG_ERROR_FMT( "sa_getPartitionRecordTable",
						   	"Reached size limit at %d records", *records);
						break;
					}
	
					sa_setPartitionRecord(data, node, port, block);
	
					sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_P_KEY_TABLE_RECORD),
				                 	bytes, records);
				}
				if (checkPort) break;
			}
		}
	}

	status = vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_getPartitionRecordTable", status);
	return status;
}

/* *******************************************************************
 */
Status_t
sa_PartitionRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt)
{
	Status_t rc = VSTATUS_OK;
	uint32_t	records = 0;

	IB_ENTER("sa_PartitionRecord", maip, 0, 0, 0);

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetPKeyTableRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblPKeyTableRecord);
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
		sa_getPartitionRecordTable(maip, &records);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	/* Determine reply status */
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_PartitionRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = sizeof(STL_P_KEY_TABLE_RECORD);

	sa_cntxt_data(sa_cntxt, sa_data, records * sizeof(STL_P_KEY_TABLE_RECORD));

	rc = sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_PartitionRecord", rc);

	return rc;
}

/* *******************************************************************
 */
void showStlPKeys(void)
{
    uint32_t i = 0; 
    PrintDest_t dest; 
    Node_t *node = NULL; 
    Port_t *port = NULL; 
    STL_P_KEY_TABLE_RECORD partitionRec, tempPartitionRec; 
    
    PrintDestInitFile(&dest, stdout); 
    
    // lock the topology
    if (vs_rdlock(&old_topology_lock) != VSTATUS_OK) 
        return; 
    
    for_all_nodes(&old_topology, node) {
        for_all_ports(node, port) {
            int block;

            if (!sm_valid_port(port) || port->state <= IB_PORT_DOWN) 
                continue; 
            
            for (block = 0; (block * PKEY_TABLE_LIST_COUNT) < port->portData->num_pkeys; ++block) {
                sa_setPartitionRecord((uint8_t *)&tempPartitionRec, node, port, block); 
                BSWAPCOPY_STL_PARTITION_TABLE_RECORD(&tempPartitionRec, (STL_P_KEY_TABLE_RECORD *)&partitionRec);
                 
                if (i++) 
                    PrintSeparator(&dest); 
                PrintStlPKeyTableRecord(&dest, 0, &partitionRec);
            }
        }
    }
    
    // unlock the topology
    vs_rwunlock(&old_topology_lock);
}

