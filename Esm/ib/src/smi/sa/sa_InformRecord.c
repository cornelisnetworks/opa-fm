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

Status_t sa_InformInfoRecord_GetTable(Mai_t * maip, uint32_t * records);
Status_t sa_IbInformInfoRecord_GetTable(Mai_t * maip, uint32_t * records);

// Takes an STL record and converts it to a byte-swapped IB record.
static __inline
void
sa_CopyAndSwapInformInfoRecord(STL_INFORM_INFO_RECORD *Src, IB_INFORM_INFO_RECORD  *Dest) 
{
	Port_t *portp;

	memset(Dest,0, sizeof(IB_INFORM_INFO_RECORD));

	if ((portp = sm_find_active_port_lid(&old_topology, Src->RID.SubscriberLID)) == NULL) {
		IB_LOG_INFINI_INFOX("sa_InformRecord: requested subscriber Lid not "
			"found/active in current topology:", 
			Src->RID.SubscriberLID);
		return;
	}

	Dest->RID.SubscriberGID.Type.Global.SubnetPrefix = portp->portData->gidPrefix;
	Dest->RID.SubscriberGID.Type.Global.InterfaceID = portp->portData->guid;
	BSWAP_IB_GID(&Dest->RID.SubscriberGID);
    Dest->RID.Enum = ntoh16(Src->RID.Enum);

	/* The fields in the IB and STL versions are similar but they are in */
	/* positions, so we can't just copy the whole record. */
	Dest->InformInfoData.GID = Src->InformInfoData.GID;
	BSWAP_IB_GID(&Dest->InformInfoData.GID);
	
	/* Note that this will truncate the STL LID Ranges. */
	Dest->InformInfoData.LIDRangeBegin = ntoh16(Src->InformInfoData.LIDRangeBegin);
	Dest->InformInfoData.LIDRangeEnd = ntoh16(Src->InformInfoData.LIDRangeEnd);

	Dest->InformInfoData.IsGeneric = Src->InformInfoData.IsGeneric;
	Dest->InformInfoData.Subscribe = Src->InformInfoData.Subscribe;
	Dest->InformInfoData.Type = ntoh16(Src->InformInfoData.Type);

	/* Can do Generic since Vendor has same types, just different field names. */
	/* Note that the unions also have slightly different names. */
	Dest->InformInfoData.u.Generic.TrapNumber = ntoh16(Src->InformInfoData.u.Generic.TrapNumber);
	Dest->InformInfoData.u.Generic.u2.AsReg32 = ntoh32(Src->InformInfoData.u.Generic.u1.AsReg32);
	Dest->InformInfoData.u.Generic.u.AsReg32 = ntoh32(Src->InformInfoData.u.Generic.u2.AsReg32);
}

Status_t
sa_InformRecord(Mai_t * maip, sa_cntxt_t * sa_cntxt)
{
	uint32_t records;
	uint16_t attribOffset;

	IB_ENTER("sa_InformRecord", maip, sa_cntxt, 0, 0);

	// 
	// Assume failure.
	// 
	records = 0;

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetInformInfoRecord);
	} else if (maip->base.method == SA_CM_GETTABLE) {
		INCREMENT_COUNTER(smCounterSaRxGetTblInformInfoRecord);
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
		(void)sa_InformInfoRecord_GetTable(maip, &records);
	} else if (maip->base.bversion == IB_BASE_VERSION && maip->base.cversion == SA_MAD_CVERSION) {
		(void)sa_IbInformInfoRecord_GetTable(maip, &records);
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
	if (maip->base.cversion == STL_SA_CLASS_VERSION) {
		attribOffset =
			sizeof(STL_INFORM_INFO_RECORD) +
			Calculate_Padding(sizeof(STL_INFORM_INFO_RECORD));
	} else {
		attribOffset =
			sizeof(IB_INFORM_INFO_RECORD) +
			Calculate_Padding(sizeof(IB_INFORM_INFO_RECORD));
	}
	
	sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void) sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_InformRecord", VSTATUS_OK);
	return (VSTATUS_OK);
}

Status_t
sa_InformInfoRecord_GetTable(Mai_t * maip, uint32_t * records)
{
	uint8_t *iirp;
	uint8_t buffer[256];
	uint32_t bytes;
	STL_SA_MAD samad;
	Status_t status;
	STL_INFORM_INFO_RECORD *iRecordp = NULL;
	CS_HashTableItr_t itr;

	IB_ENTER("sa_InformInfoRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	bytes = Calculate_Padding(sizeof(STL_INFORM_INFO_RECORD));
	memset(buffer, 0, sizeof(buffer));
	memset((void*)&samad, 0, sizeof(STL_SA_MAD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_INFORM_INFO_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_INFORM_INFO_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_INFORM_INFO_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_InformInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
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
			("sa_InformInfoRecord_GetTable: failed to create template mask, rc:",
			 status);
		IB_EXIT("sa_InformInfoRecord_GetTable", status);
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
					IB_LOG_WARN_FMT("sa_InformInfoRecord_GetTable",
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
			("sa_InformInfoRecord_GetTable: Num informInfo records found is",
			 *records);

	IB_EXIT("sa_InformInfoRecord_GetTable", status);
	return (status);
}

Status_t
sa_IbInformInfoRecord_GetTable(Mai_t * maip, uint32_t * records)
{
	uint8_t *iirp;
	uint8_t buffer[256];
	uint32_t bytes;
	STL_SA_MAD samad;
	Status_t status;
	STL_INFORM_INFO_RECORD *iRecordp = NULL;
	CS_HashTableItr_t itr;

	IB_ENTER("sa_IbInformInfoRecord_GetTable", maip, *records, 0, 0);

	*records = 0;
	bytes = Calculate_Padding(sizeof(IB_INFORM_INFO_RECORD));
	memset(buffer, 0, sizeof(buffer));
	memset((void*)&samad, 0, sizeof(STL_SA_MAD));

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(STL_SA_MAD_HEADER) < sizeof(STL_INFORM_INFO_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of STL_INFORM_INFO_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(STL_INFORM_INFO_RECORD), (int)(maip->datasize-sizeof(STL_SA_MAD_HEADER)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_IbInformInfoRecord_GetTable", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}

	// This is okay because STL and IB have the same MAD header.
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *) maip->data, &samad, sizeof(STL_INFORM_INFO_RECORD));

	/* 
	 *  Create the template mask for the lookup.
	 */
	if ((status =
		 sa_create_template_mask(maip->base.aid,
								 samad.header.mask)) != VSTATUS_OK) {
		IB_LOG_WARNRC
			("sa_IbInformInfoRecord_GetTable: failed to create template mask, rc:",
			 status);
		IB_EXIT("sa_IbInformInfoRecord_GetTable", status);
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

			sa_CopyAndSwapInformInfoRecord(iRecordp,
											(IB_INFORM_INFO_RECORD *)buffer);
			if (samad.header.mask == 0 ||
				((status = sa_template_test_noinc(samad.data, buffer,
										 sizeof(IB_INFORM_INFO_RECORD)))
				 == VSTATUS_OK)) {
				if ((status = sa_check_len(iirp, sizeof(IB_INFORM_INFO_RECORD),
								  bytes)) != VSTATUS_OK) {
					maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
					IB_LOG_WARN_FMT("sa_IbInformInfoRecord_GetTable",
									"Reached size limit at %d records",
									*records);
					break;
				}
				memcpy(iirp, buffer, sizeof(IB_INFORM_INFO_RECORD));
				sa_increment_and_pad(&iirp, sizeof(IB_INFORM_INFO_RECORD),
									 bytes, records);
			}
		} while (cs_hashtable_iterator_advance(&itr));
	}
	(void) vs_unlock(&saSubscribers.subsLock);
	if (saDebugPerf)
		IB_LOG_INFINI_INFO
			("sa_IbInformInfoRecord_GetTable: Num informInfo records found is",
			 *records);

	IB_EXIT("sa_IbInformInfoRecord_GetTable", status);
	return (status);
}
