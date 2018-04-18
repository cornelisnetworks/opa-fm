/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/
/*!

  \file report.h

  $Revision$
  $Date$

  \brief Routines to handle reports for trap events
*/

#include <report.h>
#include <subscribe.h>
#include <sdi.h>
#include "iba/stl_mad_priv.h"
#include <imap.h>
#include <imemory.h>
#include <dbg.h>
#include <ib_debug.h>

/*!
  \brief Sends a ReportResponse message to the SM acknowledging we have received
         the report message. This should stop the SM from send us the report again.
*/
static __inline
FSTATUS
SendReportResponse(void *pContext, IBT_DGRM_ELEMENT *pReportDgrmList)
{
	FSTATUS          Status;
	uint32           ElementCount;
	IBT_DGRM_ELEMENT *pResponseDgrmElement;
	MAD              *pReportMad;
	MAD              *pResponseMad;
	        
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendReportResponse);

	//
	// Get a datagram element from the SA datagram pool
	//
	ElementCount = 1;
	Status = iba_gsi_dgrm_pool_get( sa_represp_poolhandle, &ElementCount, &pResponseDgrmElement );
#ifdef SA_ALLOC_DEBUG
	if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("get sa for report: Status %d\n", Status);
#endif
	if ( Status != FSUCCESS )
	{
		_DBG_ERROR( ( "Failed to get DGRM Element. Status 0x%x\n", Status ) );
		goto exit;
	}

	pResponseDgrmElement->Element.pContext = pContext;
	pResponseDgrmElement->RemoteQP         = pReportDgrmList->RemoteQP;
	pResponseDgrmElement->RemoteQKey       = pReportDgrmList->RemoteQKey;
	pResponseDgrmElement->RemoteLID        = pReportDgrmList->RemoteLID;
	pResponseDgrmElement->ServiceLevel     = pReportDgrmList->ServiceLevel;
	pResponseDgrmElement->PortGuid         = pReportDgrmList->PortGuid;
	pResponseDgrmElement->PathBits         = pReportDgrmList->PathBits;
	pResponseDgrmElement->StaticRate       = pReportDgrmList->StaticRate;
	pResponseDgrmElement->PkeyIndex 	   = pReportDgrmList->PkeyIndex;
	
	pReportMad = GsiDgrmGetRecvMad(pReportDgrmList);
	
	pResponseMad = GsiDgrmGetSendMad(pResponseDgrmElement);

	//
	// Set various fields within MAD itself now
	//
	MemoryClear( pResponseMad, sizeof(MAD) );
	
	pResponseMad->common.BaseVersion         = IB_BASE_VERSION;
	pResponseMad->common.MgmtClass           = MCLASS_SUBN_ADM;
	pResponseMad->common.ClassVersion        = IB_SUBN_ADM_CLASS_VERSION;
	pResponseMad->common.mr.AsReg8           = MMTHD_REPORT_RESP;
	pResponseMad->common.u.NS.Status.AsReg16 = MAD_STATUS_SUCCESS;
	pResponseMad->common.TransactionID       = pReportMad->common.TransactionID;
	pResponseMad->common.AttributeID         = MCLASS_ATTRIB_ID_NOTICE;
	pResponseMad->common.AttributeModifier   = 0;
	
	pResponseDgrmElement->Element.pBufferList->ByteCount = sizeof(MAD_COMMON);
	
	Status = iba_gsi_post_send( GsiSAHandle, pResponseDgrmElement );
#ifdef SA_ALLOC_DEBUG
	if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("send sa for report: Status %d\n", Status);
#endif
	if ( Status != FSUCCESS )
	{
		// 
		// This should not happen and if this happens it 
		//can be due to port status changinging to inactive
		// Free the TSL DGRM Element
		//
		
		iba_gsi_dgrm_pool_put( pResponseDgrmElement );
		_DBG_ERROR( ( "iba_gsi_post_send failed, Status = %d\n", Status ) );
	}

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}

/*!
  \brief Validates that the fields in the report make sense
*/
static __inline
FSTATUS
ValidateReport(IBT_DGRM_ELEMENT *pReportDgrmList)
{
	FSTATUS         Status = FSUCCESS;
	SA_MAD          *pReportMad = (SA_MAD *)GsiDgrmGetRecvMad(pReportDgrmList);
	uint32          ByteCount   = pReportDgrmList->Element.pBufferList->pNextBuffer->ByteCount;
	        
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ValidateReport);

	if ( ByteCount < sizeof(SA_MAD) )
	{
		_DBG_ERROR( ("Invalid ByteCount. Expected %d, Received %d\n",
			(int)(sizeof(SA_MAD) + sizeof(IB_NOTICE)),
			ByteCount) );
		Status = FERROR;
		goto exit;
	}
	
	if ( pReportDgrmList->Element.pNextElement != NULL )
	{
		// Should be single element
		_DBG_ERROR( ("Error Multi Element Request Received\n") );
		Status = FERROR;
		goto exit;
	}

	if ( pReportMad->common.AttributeID != MCLASS_ATTRIB_ID_NOTICE )
	{
		// Should be a Notice Attribute ID
		_DBG_ERROR( ("Invalid AttributeID. Expected %d, Received %d\n",
			MCLASS_ATTRIB_ID_NOTICE,
			pReportMad->common.AttributeID) );
		Status = FERROR;
		goto exit;
	}
	
	if ( !( pReportMad->common.AttributeModifier == 0 ) )
	{
		// Attribute Modifier must be zero
		_DBG_ERROR( ("Invalid AttributeModifier. Expected %d, Received %d\n",
			0,
			pReportMad->common.AttributeModifier) );
		Status = FERROR;
	}

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}

/*!
  \brief Swaps the Notice and the Notice Details data (based on trap) fields
*/
static __inline
void
BSwapNoticeRecord(IB_NOTICE *pNotice, boolean toHostOrder)
{
	void *pNoticeDataDetails;
	uint16 trapNumber = 0;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BSwapNoticeRecord);
	
	if (! toHostOrder)
		trapNumber = pNotice->u.Generic.TrapNumber;
	BSWAP_IB_NOTICE(pNotice);
	if (toHostOrder)
		trapNumber = pNotice->u.Generic.TrapNumber;

	pNoticeDataDetails = pNotice->Data;
	switch (trapNumber)
	{
		case SMA_TRAP_GID_NOW_IN_SERVICE:
			BSWAP_TRAP_64_DETAILS((TRAP_64_DETAILS *)pNoticeDataDetails);
			break;
		
		case SMA_TRAP_GID_OUT_OF_SERVICE:
			BSWAP_TRAP_65_DETAILS((TRAP_65_DETAILS *)pNoticeDataDetails);
			break;
		
		case SMA_TRAP_ADD_MULTICAST_GROUP:
			BSWAP_TRAP_66_DETAILS((TRAP_66_DETAILS *)pNoticeDataDetails);
			break;
		
		case SMA_TRAP_DEL_MULTICAST_GROUP:
			BSWAP_TRAP_67_DETAILS((TRAP_67_DETAILS *)pNoticeDataDetails);
			break;

		default:
			break;
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/*!
  \brief Performs the processing required to acknowledge, validate, adjust and inform clients
         when a report message has been received from the Subnet Manager
*/
FSTATUS 
ProcessReport( void *pContext, IBT_DGRM_ELEMENT *pReportDgrmList )
{
	FSTATUS Status;
	IB_NOTICE  *pNotice = (IB_NOTICE *)(((SA_MAD *)GsiDgrmGetRecvMad(pReportDgrmList))->Data);
	        
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReport);

	Status = SendReportResponse( pContext, pReportDgrmList );
	if (Status != FSUCCESS)
	{
		_DBG_ERROR( ( "SendReportResponse failed, Status = %d\n", Status ) );
		/* Continue on as report still could be valid */
		Status = FSUCCESS;
	}
	
	Status = ValidateReport( pReportDgrmList );
	if (Status != FSUCCESS)
	{
		_DBG_ERROR( ( "ValidateReport failed, Status = %d\n", Status ) );
		goto exit;
	}
	
	BSwapNoticeRecord(pNotice, TRUE);
	
	ProcessClientTraps( pContext, pNotice, pReportDgrmList->PortGuid );

	BSwapNoticeRecord(pNotice, FALSE);
exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}
