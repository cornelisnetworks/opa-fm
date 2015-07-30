/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

// Public header file
#include "ib_cm.h"

// Private header file
#include "cm_private.h"


//////////////////////////////////////////////////////////////////////////
// iba_cm_sidr_register
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_sidr_register(
	IN IB_HANDLE				hCEP,
	IN const SIDR_REGISTER_INFO* pSIDRRegisterInfo,
	PFN_CM_CALLBACK				pfnLookupCallback,
	void*						Context
	)
{
	CM_CEP_OBJECT* pCEP=(CM_CEP_OBJECT*)hCEP;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_sidr_register);

	//
	// Parameter check
	//
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);

	// We must be in idle state
	if (pCEP->State != CMS_IDLE)
	{
		SpinLockRelease(&gCM->ListLock);

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FINVALID_STATE;
	}

	pCEP->Mode = CM_MODE_PASSIVE;

	// Bind address is based on SID with secondary optional fields
	// (most of which are not applicable for SIDR)
	ASSERT(! pCEP->bPeer);

	// we do not put on LocalEndPoint map because multiple Service IDs could
	// have the same QPN/EECN response.  Also we never need to lookup nor
	// verify LocalEndPoint for CMS_REGISTER'ed CEPs
	MemoryClear(&pCEP->PrimaryPath, sizeof(pCEP->PrimaryPath));
	pCEP->LocalEndPoint.EECN = 0;
	pCEP->LocalEndPoint.CaGUID = 0;
	pCEP->Mtu = 0;	// not used
	pCEP->PartitionKey = 0;	// not used

	pCEP->RemoteEndPoint.QPN = pCEP->RemoteEndPoint.EECN = 0;
	pCEP->RemoteEndPoint.CaGUID = 0;

	pCEP->SID = pSIDRRegisterInfo->ServiceID;
	pCEP->LocalEndPoint.QPN = pSIDRRegisterInfo->QPN;

	// add to ListenMap (which inbound SIDR_REQ will search) and
	// make sure the bound address is unique
	if (! CM_MapTryInsert(&gCM->ListenMap, (uintn)pCEP, &pCEP->MapEntry,
							"LISTEN_LIST", pCEP))
	{
		// undo settings
		pCEP->SID = 0;
		pCEP->LocalEndPoint.QPN = 0;
		SpinLockRelease(&gCM->ListLock);

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FCM_ADDR_INUSE;
	}

	// This is used only for debugging purposes
	AssignLocalCommID(pCEP);
	pCEP->TransactionID = 0;	// not used, we are passive side

	pCEP->QKey = pSIDRRegisterInfo->QKey;
	pCEP->pfnUserCallback = pfnLookupCallback;
	pCEP->UserContext = Context;

	// By default, bSidrRegisterNotify is 0. We reset this in SIDRDeregister()
	if (pCEP->pfnUserCallback)
	{
		pCEP->bSidrRegisterNotify = 1;
	}

	CepSetState(pCEP, CMS_REGISTERED);
	
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return FSUCCESS;

} // iba_cm_sidr_register()


//////////////////////////////////////////////////////////////////////////
// iba_cm_sidr_deregister
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_sidr_deregister(
	IB_HANDLE		hCEP
	)
{
	CM_CEP_OBJECT* pCEP=(CM_CEP_OBJECT*)hCEP;
	CM_CEP_OBJECT* pCEPEntry=NULL;

	DLIST_ENTRY*	pListEntry=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_sidr_deregister);

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->State != CMS_REGISTERED)
	{
		SpinLockRelease(&gCM->ListLock);

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FINVALID_STATE;
	}
			
	// not on LocalEndPointMap, so must clear before ClearEndPoint in CepToIdle
	pCEP->LocalEndPoint.QPN = 0;
	pCEP->LocalEndPoint.EECN = 0;
	pCEP->bSidrRegisterNotify = 0;

	// Cleanup the pending list
	while (!DListIsEmpty(&pCEP->PendingList))
	{
		pListEntry = DListGetNext(&pCEP->PendingList);
		pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);

		// Move from PENDING to INIT list and destroy it
		ASSERT(pCEPEntry->pParentCEP == pCEP);
		CepRemoveFromPendingList(pCEPEntry);
		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEPEntry);

		DestroyCEP(pCEPEntry);
	}
	ASSERT(pCEP->PendingCount == 0);

	// Move from LISTEN to IDLE
	CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
	// ClearEndPoint in CepToIdle is benign
	CepToIdle(pCEP);

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return FSUCCESS;

} // iba_cm_sidr_deregister()


//////////////////////////////////////////////////////////////////////////
// iba_cm_sidr_response
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_sidr_response(
	IN IB_HANDLE				hCEP,
	IN const SIDR_RESP_INFO*	pSIDRResponse
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	CM_CEP_OBJECT* pCEPEntry = NULL;

	DLIST_ENTRY* pListEntry=NULL;

	uint32 Status=FSUCCESS;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_sidr_response);


	//
	// SIDR response - 
	//
	SpinLockAcquire(&gCM->ListLock);

	if (pCEP->State != CMS_REGISTERED)
	{
		SpinLockRelease(&gCM->ListLock);

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FINVALID_STATE;
	}

	// Get the request we are responding to
	if(DListIsEmpty(&pCEP->PendingList))
	{
		SpinLockRelease(&gCM->ListLock);

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		
		return FNOT_FOUND;
	}

	pListEntry = DListGetNext(&pCEP->PendingList);
	pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);

	// Cannot accept until the user has been notify thru wait
	/*
	if (!BitTest(pCEPEntry->EventFlags, USRE_WAIT))
	{
		SpinLockRelease(&gCM->ListLock);
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return FUNAVAILABLE;
	}

	// Clear the flag
	BitClear(pCEPEntry->EventFlags, USRE_WAIT);
	*/

	// Move from the PENDING to INIT list
	ASSERT(pCEPEntry->pParentCEP == pCEP);
	CepRemoveFromPendingList(pCEPEntry);
	// ClearEndPoint in CepToIdle is benign
	CepToIdle(pCEPEntry); // CMS_SIDR_RESP_SENT

	// Send the SIDR response.
	Status = SendSIDR_RESP(pCEP->SID, 
							pSIDRResponse->QPN, 
							pSIDRResponse->QKey, 
							pSIDRResponse->Status,
							pSIDRResponse->PrivateData, 
							CMM_SIDR_RESP_USER_LEN,
							pCEPEntry->TransactionID,
							pCEPEntry->RemoteCommID,
							pCEPEntry->pDgrmElement);

	// No further use for this obj. Destroy it.
	DestroyCEP(pCEPEntry);

	// If there are other pending request, retrigger the event
	if (pCEP->PendingCount)
	{
		//MsgOut("SIDR re-notify, pending not empty\n");
		SetNotification(pCEP, CME_RCVD_SIDR_REQ); //EventSet(pCEP, CME_RCVD_SIDR_REQ);
	}

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_sidr_response()


//////////////////////////////////////////////////////////////////////////
// iba_cm_sidr_query
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_sidr_query(
	IB_HANDLE			hCEP,
	const SIDR_REQ_INFO* pSIDRRequest,
	PFN_CM_CALLBACK		pfnQueryCB,
	void*				Context
	)
{
	CM_CEP_OBJECT* pCEP=(CM_CEP_OBJECT*)hCEP;
	FSTATUS Status=FSUCCESS;

	uint8				SrcGidIndex=0;
	IB_PORT_ATTRIBUTES	*pPortAttr=NULL;
	IB_LMC				lmc;
	EUI64				CaGuid;
	uint16				PkeyIndex = 0;
	IB_P_KEY			PartitionKey;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_sidr_query);

	// Parameter check
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
		goto done;

	Status = iba_get_ca_for_port(pSIDRRequest->PortGuid, &CaGuid);
	if (Status != FSUCCESS)
		goto done;

	// do these lookups outside the lock
	Status = iba_find_ca_gid(&pSIDRRequest->PathInfo.Path.SGID,
					CaGuid, NULL,
					&SrcGidIndex, &pPortAttr, NULL);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> FindCaGid() failed!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		Status = FINVALID_PARAMETER;
		goto done;
	}

	// if Partition key not specified, use P_Key from Path
	PartitionKey = pSIDRRequest->PartitionKey;
	if (0 == PartitionKey)
	{
		PartitionKey = pSIDRRequest->PathInfo.Path.P_Key;
	}
	if (! LookupPKey(pPortAttr, PartitionKey, FALSE, &PkeyIndex))
	{
		_DBG_WARN(("<cep 0x%p> LookupPKey() failed!!!\n", _DBG_PTR(pCEP)));
		Status = FINVALID_PARAMETER;
		MemoryDeallocate(pPortAttr);
		goto done;
	}
	lmc = pPortAttr->Address.LMC;
	MemoryDeallocate(pPortAttr);
	pPortAttr = NULL;

	SpinLockAcquire(&gCM->ListLock);

	// If CEP is being reused, it may have a send still in progress or it
	// may not have a Dgrm any longer
	if (pCEP->pDgrmElement && CmDgrmIsInUse(pCEP->pDgrmElement)) {
		_DBG_INFO(("<cep 0x%p> Releasing the dgrm <dgrm 0x%p>.\n", 
						_DBG_PTR(pCEP), _DBG_PTR(pCEP->pDgrmElement)));
		CmDgrmSetReleaseFlag(pCEP->pDgrmElement);
		pCEP->pDgrmElement = NULL;
	}
	if (pCEP->pDgrmElement == NULL)
	{
		pCEP->pDgrmElement = CmDgrmGet();
		if (!pCEP->pDgrmElement)
		{
			_DBG_WARNING(("<cep 0x%p> Unable to send SIDR_REQ: Out of CM Dgrms!!!\n",
						_DBG_PTR(pCEP)));
			Status = FINSUFFICIENT_RESOURCES;
			goto unlock;
		}
	}

	// Dgrm could be in use if we failed a SIDRQuery due to a timeout
	// but still did not have the send completion
	if (pCEP->State != CMS_IDLE || CmDgrmIsInUse(pCEP->pDgrmElement))
	{
		Status = FINVALID_STATE;
		goto unlock;
	}

	pCEP->Mode = CM_MODE_ACTIVE;
	ASSERT(! pCEP->bPeer);

	// This is the request ID
	AssignLocalCommID(pCEP);
	pCEP->TransactionID = pCEP->LocalCommID << 8;

	pCEP->pfnUserCallback = pfnQueryCB;
	pCEP->UserContext = Context;

	// Setup the dgrm's dest addr
	CmDgrmAddrSet(pCEP->pDgrmElement,
				pSIDRRequest->PortGuid,
				pSIDRRequest->PathInfo.Path.DLID,
				pSIDRRequest->PathInfo.Path.u2.s.SL,
				LidToPathBits(pSIDRRequest->PathInfo.Path.SLID, lmc),
				pSIDRRequest->PathInfo.Path.Rate,
				(!pSIDRRequest->PathInfo.bSubnetLocal),
				pSIDRRequest->PathInfo.Path.DGID,
				pSIDRRequest->PathInfo.Path.u1.s.FlowLabel,
				(uint8)SrcGidIndex,
				(uint8)pSIDRRequest->PathInfo.Path.u1.s.HopLimit,
				(uint8)pSIDRRequest->PathInfo.Path.TClass,
				PkeyIndex);

	// Format the SIDR REQ
	FormatSIDR_REQ((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement),
					pSIDRRequest->SID,
					PartitionKey,
					pSIDRRequest->PrivateData,
					CMM_SIDR_REQ_USER_LEN,
					pCEP->TransactionID,
					pCEP->LocalCommID);

	pCEP->RetryCount = 0;
	pCEP->MaxCMRetries = gCmParams.MaxReqRetry;

	// Send the SIDR REQ
	Status = CmDgrmSend(pCEP->pDgrmElement);
	if (Status != FSUCCESS)
	{
		// fall through and let timer retry the send later
		_DBG_WARN(("<cep 0x%p> DgrmSend() failed for SIDR_REQ!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		Status = FSUCCESS;
	} else {
		AtomicIncrementVoid(&gCM->Sent.SidrReq);
	}

	// Move from IDLE to QUERY
	CM_MapInsert(&gCM->QueryMap, pCEP->LocalCommID, &pCEP->MapEntry, QUERY_LIST, pCEP);
	CepSetState(pCEP, CMS_SIDR_REQ_SENT);

	pCEP->pfnUserCallback = pfnQueryCB;
	pCEP->UserContext = Context;

	pCEP->Mtu = 0;	// not used
	pCEP->PartitionKey = 0;	// not used
	pCEP->TargetAckDelay = 0;	// not used
	pCEP->PrimaryPath.LocalLID = pSIDRRequest->PathInfo.Path.SLID;
	pCEP->PrimaryPath.LocalGID = pSIDRRequest->PathInfo.Path.SGID;

	pCEP->PrimaryPath.RemoteLID = pSIDRRequest->PathInfo.Path.DLID;
	pCEP->PrimaryPath.RemoteGID = pSIDRRequest->PathInfo.Path.DGID;
	if (pSIDRRequest->PathInfo.Path.PktLifeTime > 31)
	{
		pCEP->PktLifeTime = 31;	// 2.4 hours !!!!!
	} else {
		pCEP->PktLifeTime = pSIDRRequest->PathInfo.Path.PktLifeTime;
	}

	// Start the query timer to time out this request if the response doesnt come in
	CmTimerStart(pCEP, SidrTimeout(pCEP), SIDR_REQ_TIMER);
	Status = FPENDING;

unlock:
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // iba_cm_sidr_query()
