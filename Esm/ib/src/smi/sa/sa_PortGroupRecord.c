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
//                                                                           //
// FILE NAME                                                                 //
//    sa_PortGroupRecord.c                                                   //
//                                                                           //
// DESCRIPTION                                                               //
//    This file contains the routines to process the SA requests for         //
//    records of the Port Group Table Record type.                           //
//                                                                           //
// DATA STRUCTURES                                                           //
//    None                                                                   //
//                                                                           //
// FUNCTIONS                                                                 //
//    sa_PortGroupRecord                                                     //
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

static void GetPortGroupRecord(Mai_t *, uint32_t *);
static void GetPortGroupFwdRecord(Mai_t *, uint32_t *);

Status_t
sa_PortGroupRecord(Mai_t *maip, sa_cntxt_t *sa_cntxt) {
    uint32_t records;
    uint32_t attribOffset;

    IB_ENTER("sa_PortGroupRecord", maip, 0, 0, 0);

    //  Assume failure.
    records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetPortGroupRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblPortGroupRecord);
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
		GetPortGroupRecord(maip, &records);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

    //  Determine reply status
    if (maip->base.status != MAD_STATUS_OK) {
        records = 0;
    } else if (records == 0) {
        maip->base.status = MAD_STATUS_SA_NO_RECORDS;
    } else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
        IB_LOG_WARN("sa_PortGroupRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_PORT_GROUP_TABLE_RECORD) + Calculate_Padding(sizeof(STL_PORT_GROUP_TABLE_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_PortGroupRecord", VSTATUS_OK);
    return (VSTATUS_OK); 
}


Status_t
sa_PortGroupFwdRecord(Mai_t *maip, sa_cntxt_t *sa_cntxt) {
    uint32_t records;
    uint32_t attribOffset;

    IB_ENTER("sa_PortGroupFwdRecord", maip, 0, 0, 0);

    //  Assume failure.
    records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetPortGroupFwdRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblPortGroupFwdRecord);
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
		(void)GetPortGroupFwdRecord(maip, &records);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

    //  Determine reply status
    if (maip->base.status != MAD_STATUS_OK) {
        records = 0;
    } else if (records == 0) {
        maip->base.status = MAD_STATUS_SA_NO_RECORDS;
    } else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
        IB_LOG_WARN("sa_PortGroupFwdRecord: too many records for SA_CM_GET:", records);
        records = 0;
        maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
    } else {
        maip->base.status = MAD_STATUS_OK;
    }

    attribOffset =  sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD) + Calculate_Padding(sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD));
    sa_cntxt->attribLen = attribOffset;

    sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
    (void)sa_send_reply(maip, sa_cntxt);

    IB_EXIT("sa_PortGroupFwdRecord", VSTATUS_OK);
    return (VSTATUS_OK);
}

void
GetPortGroupRecord(Mai_t *maip, uint32_t *records)
{
    uint8_t     *data;
    uint32_t    bytes;
    STL_SA_MAD  samad;
    Status_t    status;
    Node_t      *nodep;
    bool_t      checkLid;
    STL_LID     lid=0, port0Lid=0;
    bool_t      checkBlock;
    uint8_t     blockNum=0;
    int			endBlock, blkIdx;
    STL_PORT_GROUP_TABLE_RECORD record = {{0}};

    IB_ENTER("GetPortGroupRecord", maip, *records, 0, 0);

    *records = 0;
    data = sa_data;
    bytes = Calculate_Padding(sizeof(STL_PORT_GROUP_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_PORT_GROUP_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_PORT_GROUP_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_PORT_GROUP_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("GetPortGroupRecord", MAD_STATUS_SA_REQ_INVALID);
		return;
	}

    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_PORT_GROUP_TABLE_RECORD));

    checkLid = (samad.header.mask & STL_PGTB_RECORD_COMP_LID);
    if (checkLid) { 
        lid = ntoh32(((STL_PORT_GROUP_TABLE_RECORD*)(samad.data))->RID.LID);
    	samad.header.mask ^= STL_PGTB_RECORD_COMP_LID;
    }

    // Block Num
    checkBlock = (samad.header.mask & STL_PGTB_RECORD_COMP_BLOCKNUM);
    if (checkBlock) {
	blockNum = ((STL_PORT_GROUP_TABLE_RECORD*)(samad.data))->RID.BlockNum;
    	samad.header.mask ^= STL_PGTB_RECORD_COMP_BLOCKNUM;
    }

    // Invalid to request a block without also a lid.
    // But its ok to request a lid without a block.
    if (checkBlock && !checkLid ) {
        maip->base.status = MAD_STATUS_SA_REQ_INVALID;
       return;
    }

    // Position, if specified, must be 0 for now
    if ((samad.header.mask & STL_PGTB_RECORD_COMP_POSITION) &&
	(((STL_PORT_GROUP_TABLE_RECORD*)(samad.data))->RID.Position!=0) ) {
        maip->base.status = MAD_STATUS_SA_REQ_INVALID;
        IB_LOG_ERROR_FMT( "GetPortGroupRecord","Block specified w/o Lid. Block %d", blockNum);
        return;
    }

    status = sa_create_template_mask(maip->base.aid, samad.header.mask);
    if (status != VSTATUS_OK) {
        IB_EXIT("GetPortGroupRecord", status);
        return;
    }

    (void)vs_rdlock(&old_topology_lock);

	// FIXME: make a common function, then optimize checkLid similar to
	// SLSCTableRecord
	for_all_switch_nodes(&old_topology, nodep) {
		port0Lid = (sm_get_port(nodep, 0))->portData->lid;
		if (checkLid && (port0Lid != lid)) continue;
		if (nodep->switchInfo.PortGroupCap == 0) continue;
		if (nodep->switchInfo.CapabilityMask.s.IsAdaptiveRoutingSupported == 0) continue;
		if (nodep->switchInfo.AdaptiveRouting.s.Enable == 0) continue;
		if (nodep->switchInfo.PortGroupTop == 0) continue;

		if (nodep->pgt == 0) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_ERROR_FMT("GetPortGroupRecord", "PG Table uninitialized. LID:%d", (int)port0Lid);
			goto done; // Need to unlock topology db.
		}

        endBlock = (nodep->switchInfo.PortGroupTop+7)/STL_PGTB_NUM_ENTRIES_PER_BLOCK-1;
        blkIdx = checkBlock ? blockNum : 0;
        if (blkIdx>endBlock) {
            maip->base.status = MAD_STATUS_SA_REQ_INVALID;
            IB_LOG_ERROR_FMT( "GetPortGroupRecord","Block(%d) out of range", blockNum);
            goto done;
        }
        for (; blkIdx <= endBlock; blkIdx++) {
            if ((status = sa_check_len(data, sizeof(STL_PORT_GROUP_TABLE_RECORD), bytes)) != VSTATUS_OK) {
                maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                IB_LOG_ERROR_FMT( "GetPortGroupRecord","Reached size limit at %d records", *records);
                goto done; // Need to unlock topology db.
            }

            memset(&record, 0, sizeof(record));
            record.RID.LID = port0Lid;
	    record.RID.BlockNum = blkIdx;
            record.RID.Position = 0;
            // Copy port group block from SM DB, without exceeding the capacity.
            int maxCopy = STL_PGTB_NUM_ENTRIES_PER_BLOCK;
            if (((blkIdx+1)*STL_PGTB_NUM_ENTRIES_PER_BLOCK) > nodep->switchInfo.PortGroupTop) {
                maxCopy = nodep->switchInfo.PortGroupTop % STL_PGTB_NUM_ENTRIES_PER_BLOCK;
            }

            memcpy(record.GroupBlock, &nodep->pgt[blkIdx*STL_PGTB_NUM_ENTRIES_PER_BLOCK], maxCopy*sizeof(STL_PORTMASK));
            BSWAPCOPY_STL_PORT_GROUP_TABLE_RECORD(&record, (STL_PORT_GROUP_TABLE_RECORD*)data);
            (void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_PORT_GROUP_TABLE_RECORD), bytes, records);

            if (checkBlock) goto done;
        }
        if (checkLid) goto done;
    }

done:
    (void)vs_rwunlock(&old_topology_lock);

    IB_EXIT("GetPortGroupRecord", maip->base.status);
    return;
}


void
GetPortGroupFwdRecord(Mai_t *maip, uint32_t *records)
{
    uint8_t     *data;
    uint32_t    bytes;
    STL_SA_MAD  samad;
    Status_t    status;
    Node_t      *nodep;
    bool_t      checkLid;
    STL_LID     lid=0, port0Lid=0;
    bool_t      checkBlock;
    uint32_t    blockNum=0;
    uint32_t    endBlock, blkIdx;
    STL_PORT_GROUP_FORWARDING_TABLE_RECORD record = {{0}};

    IB_ENTER("GetPortGroupFwdRecord", maip, *records, 0, 0);

    *records = 0;
    data = sa_data;
    bytes = Calculate_Padding(sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_PORT_GROUP_FORWARDING_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("GetPortGroupFwdRecord", MAD_STATUS_SA_REQ_INVALID);
		return;
	}

    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD));

    checkLid = (samad.header.mask & STL_PGFWDTB_RECORD_COMP_LID);
    if (checkLid) { 
        lid = ntoh32(((STL_PORT_GROUP_FORWARDING_TABLE_RECORD*)(samad.data))->RID.LID);
    }

    // Block Num
    checkBlock = (samad.header.mask & STL_PGFWDTB_RECORD_COMP_BLOCKNUM);
    if (checkBlock) { 
        blockNum = ((STL_PORT_GROUP_FORWARDING_TABLE_RECORD*)(samad.data))->RID.u1.s.BlockNum;
    }

    // Invalid to request a block without also a lid.
    // But its ok to request a lid without a block.
    if (checkBlock && !checkLid ) {
        maip->base.status = MAD_STATUS_SA_REQ_INVALID;
        IB_LOG_ERROR_FMT( "GetPortGroupFDBRecord","Block specified w/o Lid. Block: %d", blockNum);
        return;
    }

    status = sa_create_template_mask(maip->base.aid, samad.header.mask);
    if (status != VSTATUS_OK) {
        IB_EXIT("GetPortGroupFwdRecord", status);
        return;
    }

    (void)vs_rdlock(&old_topology_lock);

    for_all_switch_nodes(&old_topology, nodep) {
        port0Lid = (sm_get_port(nodep, 0))->portData->lid;
        if (checkLid && (port0Lid != lid)) continue;
        if (nodep->switchInfo.LinearFDBTop == 0) continue;
        if (nodep->switchInfo.CapabilityMask.s.IsAdaptiveRoutingSupported==0) continue;
		if (nodep->switchInfo.AdaptiveRouting.s.Enable==0) continue;

		//@todo: modify to respond with all ff's for PGFT records when AR is supported and enabled but hasn't been set on this switch in this topology
        if (nodep->pgft==0 || sm_Node_get_pgft_size(nodep) == 0) {
			// skip this node.
            continue;
        }

		// blocks range from 0 to # blocks - 1.
        endBlock = ROUNDUP(sm_Node_get_pgft_size(nodep),
					STL_PGFDB_NUM_ENTRIES_PER_BLOCK)/
					STL_PGFDB_NUM_ENTRIES_PER_BLOCK - 1; 
        blkIdx = checkBlock ? blockNum : 0;
        if (blkIdx>endBlock) {
            maip->base.status = MAD_STATUS_SA_REQ_INVALID;
            IB_LOG_ERROR_FMT("GetPortGroupFwdRecord","Block(%d) out of range", blockNum);
            goto done; // Need to unlock topology db.
        }
		if (checkBlock) endBlock = blockNum;
        for (; blkIdx <= endBlock; blkIdx++) {
            if ((status = sa_check_len(data, sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD), bytes)) != VSTATUS_OK) {
                maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                IB_LOG_ERROR_FMT( "GetPortGroupFwdRecord","Reached size limit at %d records", *records);
                goto done;
            }

            record.RID.LID = port0Lid;
            record.RID.u1.s.Reserved = 0; 
            record.RID.u1.s.BlockNum = blkIdx; 
            memset(&record.PGFdbData, 0xff, sizeof(record.PGFdbData)); 

            // Copy PG FDB block from SM DB, without exceeding the capacity.
            int maxCopy = STL_PGFDB_NUM_ENTRIES_PER_BLOCK;
            if (((blkIdx+1)*STL_PGFDB_NUM_ENTRIES_PER_BLOCK) > nodep->switchInfo.LinearFDBTop) {
                maxCopy = nodep->switchInfo.LinearFDBTop % STL_PGFDB_NUM_ENTRIES_PER_BLOCK + 1;
            }

            memcpy(record.PGFdbData, &nodep->pgft[blkIdx*STL_PGFDB_NUM_ENTRIES_PER_BLOCK], maxCopy*sizeof(PORT));
            BSWAPCOPY_STL_PORT_GROUP_FORWARDING_TABLE_RECORD(&record, (STL_PORT_GROUP_FORWARDING_TABLE_RECORD*)data);
            (void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD), bytes, records);

            if (checkBlock) goto done;
        }
        if (checkLid) goto done;
    }

done:
    (void)vs_rwunlock(&old_topology_lock);
    
    IB_EXIT("GetPortGroupFwdRecord", maip->base.status);
    return;
}

