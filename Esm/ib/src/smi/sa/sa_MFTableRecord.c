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
//    sa_MFTableRecord.c                                                     //
//                                                                           //
// DESCRIPTION                                                               //
//    This file contains the routines to process the SA requests for         //
//    records of the MFTableRecord type.                                     //
//                                                                           //
// DATA STRUCTURES                                                           //
//    None                                                                   //
//                                                                           //
// FUNCTIONS                                                                 //
//    sa_MFTableRecord                                                       //
//                                                                           //
// DEPENDENCIES                                                              //
//    ib_mad.h                                                               //
//    ib_status.h                                                            //
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

/* Uncomment for debugging
#define SANITY_CHECK 1
*/

static Status_t
sa_MFTableRecord_Set(uint8_t * mftp,  Node_t * nodep, uint8_t position, uint32_t blockNum)
{
	STL_MULTICAST_FORWARDING_TABLE_RECORD record;
	int i = 0;
    Port_t *mftPortp;

	IB_ENTER("sa_MFTableRecord_Set", mftp, nodep, blockNum, 0);

#if defined(SANITY_CHECK)
	if (position  > STL_MFTB_MAX_POSITION)
		IB_LOG_ERROR_FMT( "sa_MFTableRecordSet",
		       "position param exceeds type range value: %d",
		       position);

	if (blockNum > STL_MAX_MFT_BLOCK_NUM)
		IB_LOG_ERROR_FMT( "sa_MFTableRecordSet",
		       "blockNum param exceeds type range value: %d",
		       blockNum);

	for (i = 0; i < sm_mcast_mlid_table_cap; ++i)
	{
		int j = 0;

		for (j = 0; j < STL_MFTB_WIDTH; ++j)
		{
			if ( nodep->mft[i][j] != 0)
			{
                if (!sm_valid_port((mftPortp = sm_get_port(nodep,0)))) {
                    IB_LOG_ERROR_FMT( "sa_MFTableRecordSet",
                           "failed to get port 0 (block %d, pos %d) entry [%d][%d] "
                           "of array is set (value = %04X)",
                           blockNum, position, i, j, nodep->mft[i][j]);
                } else {
                    IB_LOG_ERROR_FMT( "sa_MFTableRecordSet",
                           "(node %d, block %d, pos %d) entry [%d][%d] "
                           "of array is set (value = %04X)",
                           mftPortp->lid, blockNum, position, i, j,
                           nodep->mft[i][j]);
                }
			}
		}
	}
#endif /* SANITY_CHECK */


	/* Start filling in the record */
	memset(&record, 0, sizeof(record));

    if (!sm_valid_port((mftPortp = sm_get_port(nodep,0)))) {
        IB_LOG_ERROR0("sa_MFTableRecordSet: failed to get port");
        IB_EXIT("sa_MFTableRecordSet", VSTATUS_BAD);
        return(VSTATUS_BAD);
    }

	record.RID.LID = mftPortp->portData->lid;
	record.RID.u1.s.Position = position;
	record.RID.u1.s.BlockNum = blockNum;

	// eekahn - The following comment is probably no longer entirely relevant/correct.
	
	/* This is somewhat confusing... First, the 'mft' member of the 
	 * Node_t structure is an array of pointers to arrays of 16 uint16_t
	 * values. Each array of uint16_t's corresponds to one multicast LID's
	 * port forwarding table. To access a specific LID's portmask,
	 * what we have is:
	 *
	 *    nodep->mft[mcastLid - 0xc000][portmaskIndex]
	 *
	 * Now, per the infiniband spec, the MFTableRecord returned by this
	 * query contains a block of 32 portmasks, each portmask corresponding
	 * to a separate multicast LID. This means that we need to slice the 2D
	 * mft array in the opposite direction that we would think it should
	 * be sliced... Meaning that the position parameter is invariant in the
	 * MFTableRecord and the LID changes.
	 *
	 * To map between (blockNum, position) to the values returned in the
	 * record we use the following mapping:
	 *
	 *    MCastLidRange = [(blockNum * 32) to (blockNum * 32) + 31]
	 *    portMaskIndex = position
	 *
	 * Yes, this is kinda dumb, but I guess the rationale for it in the spec
	 * is to reduce the size of the response to this query.
	 */

	for (i = 0; i < STL_NUM_MFT_ELEMENTS_BLOCK; ++i)
	{
		if (blockNum * STL_NUM_MFT_ELEMENTS_BLOCK + i < sm_mcast_mlid_table_cap)
			record.MftTable.MftBlock[i] = nodep->mft[blockNum * STL_NUM_MFT_ELEMENTS_BLOCK + i][position];
		else
			record.MftTable.MftBlock[i] = 0;
	}
	BSWAPCOPY_STL_MCFTB_RECORD(&record, (STL_MULTICAST_FORWARDING_TABLE_RECORD*)mftp);

	IB_EXIT("sa_MFTableRecord_Set", VSTATUS_OK);

	return VSTATUS_OK;
}

static Status_t
sa_MFTableRecord_GetTable(Mai_t * maip, uint32_t * records)
{
	uint8_t  * data = NULL;
	Node_t   * nodep = NULL;
	Status_t   status = 0;
	uint32_t   bytes = 0;
	uint32_t   lidIndex = 0;
	uint8_t    portMaskIndex = 0;
	STL_LID	   maxMcLid = 0, tmpLid = 0;
	STL_SA_MAD    samad;


	IB_ENTER("sa_MFTableRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_MULTICAST_FORWARDING_TABLE_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_MFTableRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD));

	/* Create the template mask for the lookup. */
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK)
	{
		IB_EXIT("sa_MFTableRecord_GetTable", status);
		return status;
	}

	maxMcLid = sm_multicast_get_max_lid();

	/* Load the MFTableRecords in the SADB */
	(void)vs_rdlock(&old_topology_lock);

	/* iterate through each switch node */
	for_all_switch_nodes(&old_topology, nodep)
	{
		/* iterate through each multicast LID */
		for (tmpLid = STL_LID_MULTICAST_BEGIN, lidIndex = 0;
		     (tmpLid <= maxMcLid) && (status == VSTATUS_OK);
		     tmpLid += STL_NUM_MFT_ELEMENTS_BLOCK, ++lidIndex)
		{
			/* Limit our scope of portmask blocks to the number of physical
			 * ports on a switch */
			for (portMaskIndex = 0;
			     (portMaskIndex < STL_PORT_MASK_WIDTH) && ((portMaskIndex * STL_PORT_MASK_WIDTH) <= nodep->nodeInfo.NumPorts);
			     ++portMaskIndex)
			{
				if ((status = sa_check_len(data, sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD),
				                           bytes)) != VSTATUS_OK)
				{
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_ERROR_FMT( "sa_MFTableRecord_GetTable",
						   "Reached size limit at %d records", *records);
					break;
				}

				if ((status = sa_MFTableRecord_Set(data, nodep, portMaskIndex,
                                                   lidIndex)) != VSTATUS_OK)
                {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					break;
                }

				(void)sa_template_test(samad.data, &data, sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD),
				                       bytes, records);
			}
		}
	}

	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_MFTableRecord_GetTable", status);
	return(status);
}

Status_t
sa_MFTableRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt)
{
	uint32_t	records;
	int		padding;

	IB_ENTER("sa_MFTableRecord", maip, 0, 0, 0);

	/* Assume failure. */
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetMftRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblMftRecord);
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
		(void)sa_MFTableRecord_GetTable(maip, &records);
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
        IB_LOG_WARN("sa_MFTableRecord: too many records for SA_CM_GET:",
		            records);
        records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}

	padding = Calculate_Padding(sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD));
    /* setup attribute offset for possible RMPP transfer */
    sa_cntxt->attribLen = sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD) + padding;
	sa_cntxt_data(sa_cntxt, sa_data, records * (sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD) + padding));
	sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_MFTableRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}
