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
//    sa_SMInfoRecord.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the SMInfoRecord type.					     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_SMInfoRecord							     //
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
#include "sm_dbsync.h"
#include "time.h"
#ifdef __VXWORKS__
#include "UiUtil.h"
#endif


Status_t sa_SMInfoRecord_GetTable(Mai_t *, uint32_t *);
 
Status_t
sa_SMInfoRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	records;
    uint16_t    attribOffset;

	IB_ENTER("sa_SMInfoRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetSmInfoRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblSmInfoRecord);
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
		(void)sa_SMInfoRecord_GetTable(maip, &records);
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
	if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
        IB_LOG_WARN("sa_SMInfoRecord: too many records for SA_CM_GET:", records);
        records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
    attribOffset =  sizeof(STL_SMINFO_RECORD) + Calculate_Padding(sizeof(STL_SMINFO_RECORD));
    /* setup attribute offset for possible RMPP transfer */
    sa_cntxt->attribLen = attribOffset;
	sa_cntxt_data( sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt );

	IB_EXIT("sa_SMInfoRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}


Status_t
sa_SMInfoRecord_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t		*data;
	uint32_t	bytes;
	STL_SA_MAD	samad;
	Status_t	status;
    SmRecp      smrecp;
    CS_HashTableItr_t itr;
    STL_SMINFO_RECORD * record;

	IB_ENTER("sa_SMInfoRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_SMINFO_RECORD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_SMINFO_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_SMINFO_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_SMINFO_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_SMInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_SMINFO_RECORD));

	/*
	 * Create the template mask for the lookup.
	 */
	status = sa_create_template_mask(maip->base.aid, samad.header.mask);
	if (status != VSTATUS_OK) {
		IB_EXIT("sa_SMInfoRecord_GetTable", VSTATUS_OK);
		return(VSTATUS_OK);
	}

	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
		IB_LOG_ERRORRC("sa_SMInfoRecord_GetTable: Can't lock SM Record table, rc:",
						status);
    } else if (cs_hashtable_count(smRecords.smMap) > 0) {
		cs_hashtable_iterator(smRecords.smMap, &itr);
		do {
			smrecp = cs_hashtable_iterator_value(&itr);

			// Support ElapsedTime...
			// Work with the outbound copy (Do not modify the SMs db)
			record = (STL_SMINFO_RECORD *)data;
			memcpy(record, &smrecp->smInfoRec, sizeof(*record));

			// If this SM is Master, then fill in the elapsed time.
			// [Verify that the master is us... just in case]
			if (record->SMInfo.u.s.SMStateCurrent == SM_STATE_MASTER) {
				if (smrecp->portguid != sm_smInfo.PortGUID) {
					IB_LOG_WARN64("sa_SMInfoRecord_GetTable: Master SM is not us:", smrecp->portguid);
				}
				record->SMInfo.ElapsedTime = (uint32_t)difftime(time(NULL),sm_masterStartTime);
			} else if ( (record->SMInfo.u.s.SMStateCurrent == SM_STATE_STANDBY) &&
						(smrecp->dbsync.fullSyncStatus > DBSYNC_STAT_UNINITIALIZED) &&
						(smrecp->dbsync.fullTimeLastSync!=0)) {
					record->SMInfo.ElapsedTime = (uint32_t)difftime(time(NULL), smrecp->dbsync.fullTimeLastSync);
			} else {
				record->SMInfo.ElapsedTime = 0;
				IB_LOG_INFO64("sa_SMInfoRecord_GetTable: SM never syncd:", smrecp->portguid);
			}

			record->Reserved = 0;

			BSWAP_STL_SMINFO_RECORD(record);
			(void)sa_template_test_mask(samad.header.mask, samad.data, &data, sizeof(STL_SMINFO_RECORD), 
									bytes, records);
		} while (cs_hashtable_iterator_advance(&itr));
	}

    (void)vs_unlock(&smRecords.smLock);

	IB_EXIT("sa_SMInfoRecord_GetTable", VSTATUS_OK);
	return(VSTATUS_OK);
}

