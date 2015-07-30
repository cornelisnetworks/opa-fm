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
//                                       //
// FILE NAME                                     //
//    sa_InformRecord.c                              //
//                                       //
// DESCRIPTION                                   //
//    This file contains the routines to process the SA requests for         //
//    records of the InformRecord type.                      //
//                                       //
// DATA STRUCTURES                               //
//    None                                   //
//                                       //
// FUNCTIONS                                     //
//    InformRecord                               //
//                                       //
// DEPENDENCIES                                  //
//    ib_mad.h                                   //
//    ib_status.h                                //
//                                       //
// RESPONSIBLE ENGINEER                              //
//    Jeff Young                                 //
//                                       //
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

Status_t sa_informInfoRecord_GetTable(Mai_t * maip, uint32_t * records);

Status_t
sa_InformRecord(Mai_t * maip, sa_cntxt_t * sa_cntxt)
{
	uint32_t records;
	uint16_t attribOffset;

	IB_ENTER("sa_InformRecord", maip, sa_cntxt, 0, 0);

	if (maip->base.cversion != STL_SA_CLASS_VERSION) {
		maip->base.status = MAD_STATUS_BAD_CLASS;
		(void) sa_send_reply(maip, sa_cntxt);
		IB_LOG_WARN("sa_InformRecord: invalid CLASS:",
					maip->base.cversion);
		IB_EXIT("sa_InformInfo", VSTATUS_OK);
		return (VSTATUS_OK);
	}
	// 
	// Assume failure.
	// 
	records = 0;

	// 
	// Check the method.  If this is a template lookup, then call the
	// regular
	// GetTable(*) template lookup routine.
	// 
	switch (maip->base.method) {
	case SA_CM_GET:
		INCREMENT_COUNTER(smCounterSaRxGetInformInfoRecord);
		(void) sa_informInfoRecord_GetTable(maip, &records);
		break;
	case SA_CM_GETTABLE:
		INCREMENT_COUNTER(smCounterSaRxGetTblInformInfoRecord);
		(void) sa_informInfoRecord_GetTable(maip, &records);
		break;
	default:
		maip->base.status = MAD_STATUS_BAD_METHOD;
		(void) sa_send_reply(maip, sa_cntxt);
		IB_EXIT("sa_InformRecord", VSTATUS_OK);
		return (VSTATUS_OK);
		break;
	}

	// 
	// Determine reply status
	//
 	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_InformRecord: too many records for SA_CM_GET:",
					records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}

	/* setup attribute offset for possible RMPP transfer */
	attribOffset =
		sizeof(STL_INFORM_INFO_RECORD) +
		Calculate_Padding(sizeof(STL_INFORM_INFO_RECORD));
	
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void) sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_InformRecord", VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
sa_informInfoRecord_GetTable(Mai_t * maip, uint32_t * records)
{
	uint8_t *iirp;
	uint8_t buffer[256];
	uint32_t bytes;
	STL_SA_MAD samad;
	Status_t status;
	STL_INFORM_INFO_RECORD *iRecordp = NULL;
	CS_HashTableItr_t itr;

	IB_ENTER("sa_informInfoRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	bytes = Calculate_Padding(sizeof(STL_INFORM_INFO_RECORD));
	memset(buffer, 0, sizeof(buffer));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_INFORM_INFO_RECORD) ) {
		IB_LOG_ERROR_FMT("sa_informInfoRecord_GetTable",
						 "invalid MAD length; size of STL_INFORM_INFO_RECORD[%lu], datasize[%d]", sizeof(STL_INFORM_INFO_RECORD), maip->datasize-sizeof(STL_SA_MAD_HEADER));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_informInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(STL_INFORM_INFO_RECORD));

	/* 
	 *  Create the template mask for the lookup.
	 */
	if ((status =
		 sa_create_template_mask(maip->base.aid,
								 samad.header.mask)) != VSTATUS_OK) {
		IB_LOG_WARNRC
			("sa_informInfoRecord_GetTable: failed to create template mask, rc:",
			 status);
		IB_EXIT("sa_informInfoRecord_GetTable", status);
		return (status);
	}

	/* 
	 *  iterate through the hashtable and return matching records
	 */
	iirp = sa_data;
	(void) vs_lock(&saSubscribers.subsLock);
	if (cs_hashtable_count(saSubscribers.subsMap) > 0) {
		cs_hashtable_iterator(saSubscribers.subsMap, &itr);
		do {
			iRecordp = cs_hashtable_iterator_value(&itr);
			/* IBTA 1.2 C15-0.2.2 - do not return real qp if not trusted
			   request */
			if (sm_smInfo.SM_Key && samad.header.smKey != sm_smInfo.SM_Key) {
				iRecordp->InformInfoData.u.Generic.u1.s.QPNumber = 0;
			}

			iRecordp->Reserved = 0;
			BSWAPCOPY_STL_INFORM_INFO_RECORD(iRecordp,
											 (STL_INFORM_INFO_RECORD *)
											 buffer);
			if (samad.header.mask == 0
				||
				((status =
				  sa_template_test_noinc(samad.data, buffer,
										 sizeof(STL_INFORM_INFO_RECORD)))
				 == VSTATUS_OK)) {
				if ((status =
					 sa_check_len(iirp, sizeof(STL_INFORM_INFO_RECORD),
								  bytes)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_WARN_FMT("sa_informInfoRecord_GetTable",
									"Reached size limit at %d records",
									*records);
					break;
				}
				memcpy(iirp, buffer, sizeof(STL_INFORM_INFO_RECORD));
				sa_increment_and_pad(&iirp, sizeof(STL_INFORM_INFO_RECORD),
									 bytes, records);
			}
		} while (cs_hashtable_iterator_advance(&itr));
	}
	(void) vs_unlock(&saSubscribers.subsLock);
	if (saDebugPerf)
		IB_LOG_INFINI_INFO
			("sa_informInfoRecord_GetTable: Num informInfo records found is",
			 *records);

	IB_EXIT("sa_informInfoRecord_GetTable", status);
	return (status);
}
