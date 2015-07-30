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
// Listen
//
// Prepare CEP for Listening and put on ListenMap
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
//	Called with ListLock held, which protects lists and all CEPs
//
FSTATUS
Listen(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_LISTEN_INFO*			pListenInfo,
	IN PFN_CM_CALLBACK			pfnListenCB,
	IN void*					Context
	)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, Listen);

	// If we are binding to a specific port, we need the CaGUID of the port
	// so that during CmRemoveDevice(), we can cancel this listen
	pCEP->LocalEndPoint.CaGUID = pListenInfo->CaGUID;
	ASSERT(! pCEP->bPeer);

	// Bind address is based on SID with secondary optional
	// Local/Remote GID/LID and the Private Data Discriminator
	// (set via CmModifyCEP)
	// Hence more than one CEP could use the same SID, provided they have unique
	// combinations of values for the other fields.  Values of 0
	// represent wildcarding the given field (SID can't be wildcarded)
	pCEP->SID = pListenInfo->ListenAddr.EndPt.SID;
	pCEP->PrimaryPath.LocalLID = pListenInfo->ListenAddr.Port.LID;
	pCEP->PrimaryPath.LocalGID = pListenInfo->ListenAddr.Port.GID;

	pCEP->PrimaryPath.RemoteLID = pListenInfo->RemoteAddr.Port.LID;
	pCEP->PrimaryPath.RemoteGID = pListenInfo->RemoteAddr.Port.GID;

	pCEP->LocalEndPoint.QPN = pCEP->LocalEndPoint.EECN = 0;
	pCEP->RemoteEndPoint.QPN = pCEP->RemoteEndPoint.EECN = 0;

	// add to ListenMap and make sure bound address is unique
	if (! CM_MapTryInsert(&gCM->ListenMap, (uintn)pCEP, &pCEP->MapEntry, "LISTEN_LIST", pCEP))
	{
		_DBG_WARNING(("<cep 0x%p> Address in use!!!\n", _DBG_PTR(pCEP)));
		// clear out CEP fields
		MemoryClear(&pCEP->LocalEndPoint, sizeof(pCEP->LocalEndPoint));
		MemoryClear(&pCEP->RemoteEndPoint, sizeof(pCEP->RemoteEndPoint));
		MemoryClear(&pCEP->PrimaryPath, sizeof(pCEP->PrimaryPath));
		pCEP->LocalEndPoint.CaGUID = 0;
		pCEP->SID = 0;

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FCM_ADDR_INUSE;
	}

	// LocalCommID for listening CEP is used only for debugging purposes
	AssignLocalCommID(pCEP);
	pCEP->TransactionID = 0;	// not used, will use TID of remote client

	_DBG_INFO(("<cep 0x%p> Comm ID set at <lcid 0x%x>.\n",
			_DBG_PTR(pCEP), pCEP->LocalCommID));
	
	pCEP->Mode = CM_MODE_PASSIVE;
	pCEP->LocalCMTimeout = 0;
	pCEP->RemoteCMTimeout = 0;
	pCEP->MaxCMRetries = 0;
	pCEP->EventFlags = 0;
	pCEP->RetryCount = 0;
	//pCEP->pWaitEvent = NULL;
	DListInit(&pCEP->PendingList);

	if (pCEP->pDgrmElement)
	{
		// Return the datagram since listen objects do not use it
		_DBG_INFO(("<cep 0x%p> Releasing the dgrm <dgrm 0x%p>.\n", 
						_DBG_PTR(pCEP), _DBG_PTR(pCEP->pDgrmElement)));
		CmDgrmRelease(pCEP->pDgrmElement);
	}

	// Save user callback info
	pCEP->pfnUserCallback = pfnListenCB;
	pCEP->UserContext = Context;

	CepSetState(pCEP, CMS_LISTEN);

	_DBG_INFO(("<cep 0x%p> *** Listening on <sid 0x%"PRIx64" lid 0x%x gid 0x%"PRIx64":0x%"PRIx64">.***\n",
				_DBG_PTR(pCEP), pListenInfo->ListenAddr.EndPt.SID,
				pListenInfo->ListenAddr.Port.LID,
				pCEP->PrimaryPath.LocalGID.Type.Global.SubnetPrefix,
				pCEP->PrimaryPath.LocalGID.Type.Global.InterfaceID));

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	// At this point, we are listening for incoming connections on this obj.
	return FPENDING;

} // Listen()


// See GetConnInfo in cm_active.c for callback processing

//////////////////////////////////////////////////////////////////////////
// WaitP
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
//	FSUCCESS or FTIMEOUT.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
WaitP(
	IN CM_CEP_OBJECT*			pCEP,
	OUT CM_CONN_INFO*			pConnInfo,
	uint32						Timeout_us	// TODO: Put a max limit on this in addition to infinite
	)
{
	FSTATUS Status=FSUCCESS;

	uint32 WaitTimeUs=Timeout_us;
	uint64 TimeoutUs=Timeout_us;
	uint64 StartTimeUs=GetTimeStamp();

	uint64 CurrTimeUs=0;
	uint64 TimeElapsedUs=0;

	//CMM_MSG* pMsg=&((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement))->payload;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, WaitP);

waitagain:

	// Update the timeout
	if ((int32)WaitTimeUs != EVENT_NO_TIMEOUT)
	{
		if (CurrTimeUs >= StartTimeUs)
		{
			TimeElapsedUs = (CurrTimeUs - StartTimeUs);
		}
		else // wrap-around
		{
			TimeElapsedUs = (CurrTimeUs + ((uint64)(-1) - StartTimeUs));
		}

		if (TimeoutUs > TimeElapsedUs)
		{
			WaitTimeUs = (uint32) (TimeoutUs - TimeElapsedUs);
		}
		else
		{
			WaitTimeUs = 0;
		}
	}

	//_DBG_INFO(("<cep 0x%p> Waiting on obj... <timeout %ums lcid 0x%x %s>\n", 
	//			_DBG_PTR(pCEP), WaitTimeUs/1000, pCEP->LocalCommID,
	//			_DBG_PTR(CmGetStateString(pCEP->State))));

	Status = EventWaitOn(pCEP->pEventObj, WaitTimeUs);

	//_DBG_INFO(("<cep 0x%p> Waiting on obj... complete <%s %s>\n", 
	//			_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status)),
	//			_DBG_PTR(CmGetStateString(pCEP->State))));

	if (Status == FTIMEOUT)
	{
		if (Timeout_us == 0)
		{
			//_DBG_INFO(("<cep 0x%p> Obj state is nonsignaled.\n", _DBG_PTR(pCEP)));
		}
		else
		{
			_DBG_WARNING(("<cep 0x%p> EventWaitOn() returns FTIMEOUT!\n", _DBG_PTR(pCEP)));
		}
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FTIMEOUT;
	}

	ASSERT(Status == FSUCCESS);

	_DBG_INFO(("<cep 0x%p> Obj state is signaled <%s>.\n",
				_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

	// Get mutual ex to the object
	SpinLockAcquire(&gCM->ListLock);

	Status = GetConnInfo(pCEP, pConnInfo);

	if (Status != FSUCCESS)
	{
		// Handle the case when we get signal but we already process the event
		// with the previous signal
		// eg evt1 --> signal; evt2 --> signal
		//							->wait() returned
		_DBG_WARNING(("<cep 0x%p> GetConnInfo() returns %s!\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		
		ASSERT(pConnInfo->Status == 0);

		SpinLockRelease(&gCM->ListLock);

		goto waitagain;
	}

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // WaitP()

// given a CEP for which a FCM_CONNECT_REQUEST has been reported
// fetch the head of the pending list
static FSTATUS
GetPendingCEP(
	IN CM_CEP_OBJECT*			pCEP,
	OUT CM_CEP_OBJECT**			ppCEPEntry
	)
{
	CM_CEP_OBJECT* pCEPEntry = NULL;

	DLIST_ENTRY* pListEntry=NULL;
	FSTATUS		Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GetPendingCEP);

	switch (pCEP->State)
	{
	case CMS_IDLE:

		if (BitTest(pCEP->EventFlags, USRE_CANCEL))
		{
			// User cancelled the listen, but hasn't gotten callback yet
			Status = FCM_CONNECT_CANCEL;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_LISTEN:
	case CMS_REQ_RCVD:	// separate out for Peer, no PendingList need then
	case CMS_MRA_REQ_SENT:	// separate out for Peer, no PendingList need then
			// could have listen get the pCEPEntry and then have common code
			// with Peer REQ_RCVD/MRA_REQ_SENT cases
		//
		// Accepting new client or peer connection 
		//
		if (pCEP->bPeer)
		{
			_DBG_INFO(("<cep 0x%p> *** Accepting peer connection ***\n", _DBG_PTR(pCEP)));
		} else {
			_DBG_INFO(("<cep 0x%p> *** Accepting client connection ***\n", _DBG_PTR(pCEP)));
		}

		if (DListIsEmpty(&pCEP->PendingList))	
		{
			// If user reject() or cancel() and then call accept()
			_DBG_WARNING(("<cep 0x%p> Pending list is empty!\n", _DBG_PTR(pCEP)));
			Status = FNOT_FOUND;
			break;
		}
		
		// Note: For peer-to-peer connection (bPeer == TRUE), the entry in the PENDING list 
		// is the same as the one passed in i.e. pCEP == pCEPEntry
		pListEntry = DListGetNext(&pCEP->PendingList);
		pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);
		ASSERT(pCEPEntry->pParentCEP == pCEP);
		if (pCEP->bPeer)
		{
			ASSERT(pCEPEntry == pCEP);
		} else {
			ASSERT(pCEPEntry != pCEP);
		}

		// Cannot accept until the user has been notify thru wait
		/*
		if (!BitTest(pCEPEntry->EventFlags, USRE_WAIT))
		{
			_DBG_WARNING(("<cep 0x%p> Cannot accept - User must call wait() before accept()!\n",
							_DBG_PTR(pCEP)));
			// Re-signal
			EventSet(pCEP, CME_RCVD_REQ);

			Status = FUNAVAILABLE;
			break;
		}
		
		BitClear(pCEPEntry->EventFlags, USRE_WAIT);
		*/

		// Make sure we havent timed-out or receive a reject
		switch (pCEPEntry->State)
		{
		case CMS_REQ_RCVD:
		case CMS_MRA_REQ_SENT:
			*ppCEPEntry = pCEPEntry;
			Status = FSUCCESS;
			break;
		
		default:
			Status = FINVALID_STATE;
			break;
		}
		break;
	default:
		Status = FINVALID_STATE;
		break;
	} // switch()

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // GetPendingCEP

// process an inbound REQ for a passive CEP
FSTATUS
ProcessRequestP(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_REP_FAILOVER 			Failover,
	OUT CM_PROCESS_REQUEST_INFO *Info
	)
{
	FSTATUS Status;
	IB_CA_ATTRIBUTES *pCaAttr = NULL;
	CM_CEP_OBJECT*			pCEPEntry;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRequestP);

	// TBD - RD code here is not complete/accurate
	ASSERT(pCEP->Mode == CM_MODE_PASSIVE);

	Status = GetPendingCEP(pCEP, &pCEPEntry);
	if (Status != FSUCCESS)
		goto done;
	if (pCEPEntry->Type == CM_UD_TYPE)
	{
		Status = FINVALID_STATE;
		goto done;
	}
	if (Info == NULL)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	MemoryClear(Info, sizeof(*Info));	// just to be safe
 
	// Child must inherit parent's setting for now
	// Will be saved again when AcceptP
	pCEPEntry->bFailoverSupported = pCEP->bFailoverSupported;

	// get ca Attr
	pCaAttr = (IB_CA_ATTRIBUTES*)MemoryAllocate(sizeof(*pCaAttr), FALSE, CM_MEM_TAG);
	if (pCaAttr == NULL)
	{
		Status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	pCaAttr->PortAttributesListSize = 0;
	pCaAttr->PortAttributesList = NULL;
	Status = iba_query_ca_by_guid(pCEPEntry->LocalEndPoint.CaGUID, pCaAttr);
	if (Status != FINSUFFICIENT_MEMORY && Status != FSUCCESS)
		goto done;

	// set up attributes to move QP to Init
	GetInitAttrFromCep(pCEPEntry, &Info->QpAttrInit);

	// update LocalResponder/Initiator/SendPSN so we are ready in case
	// application calls iba_cm_prepare_rts before doing iba_cm_accept
	// Build Reply
	pCEPEntry->LocalRecvPSN = Info->ReplyInfo.Info.Reply.StartingPSN =
															GenerateStartPSN();
	// RDMA Read is not supported for UC, leave zero
	if (pCEPEntry->Type != CM_UC_TYPE) {
		pCEPEntry->LocalResponderResources =
			Info->ReplyInfo.Info.Reply.ArbResponderResources =
							MIN(pCEPEntry->LocalResponderResources,
									pCaAttr->MaxQPResponderResources);
		pCEPEntry->LocalInitiatorDepth =
			Info->ReplyInfo.Info.Reply.ArbInitiatorDepth =
							MIN(pCEPEntry->LocalInitiatorDepth,
									pCaAttr->MaxQPInitiatorDepth);
	}
	Info->ReplyInfo.Info.Reply.TargetAckDelay = pCaAttr->LocalCaAckDelay;
	// spec implies we should say NOT_SUPPORTED even if no alternate path
	// was provided in REQ
	if (pCaAttr->PathMigrationLevel == CAPathMigNone
			|| ! pCEPEntry->bFailoverSupported)
	{
		Info->ReplyInfo.Info.Reply.FailoverAccepted = CM_REP_FO_NOT_SUPPORTED;
	} else {
		Info->ReplyInfo.Info.Reply.FailoverAccepted = Failover;
	}
	// end to end credits and RNR not allowed for UC
	if (pCEPEntry->Type != CM_UC_TYPE) {
		// our CA supports RC credits
		Info->ReplyInfo.Info.Reply.EndToEndFlowControl = TRUE;
		Info->ReplyInfo.Info.Reply.RnRRetryCount =
									pCEPEntry->LocalRnrRetryCount; // 1st guess
		// caller must set approprite RnrRetryCount, this is just
		// a wild guess, caller can set higher or lower
	}

	// set up attributes to move QP to RTR
	GetRtrAttrFromCep(pCEPEntry,
			(Info->ReplyInfo.Info.Reply.FailoverAccepted == CM_REP_FO_ACCEPTED),
			&Info->QpAttrRtr);
done:
	if (pCaAttr)
		MemoryDeallocate(pCaAttr);
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// AcceptP_Async
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
//	FSUCCESS - The function has completed successfully. For server, this indicates the initialization.
//		>FCM_CONNECT_ESTABLISHED
//		>FCM_CONNECT_REJECT
//		>FCM_CONNECT_TIMEOUT
//		>FCM_CONNECT_CANCEL
//
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the reply ack packet
//	FTIMEOUT - The timeout interval expires
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
AcceptP_Async(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_CONN_INFO*		pSendConnInfo,		// Send REP
	IN EVENT_HANDLE				hEvent,
	IN EVENT_HANDLE				hWaitEvent,
	IN PFN_CM_CALLBACK			pfnCallback,
	IN void*					Context,
	IN boolean					willWait,
	OUT CM_CEP_OBJECT**			ppCEP
	)
{
	CM_CEP_OBJECT* pCEPEntry = NULL;

	FSTATUS		Status=FSUCCESS;

	uint32		rep_timeout_ms=0;
	uint64		elapsed_us=0;
	uint8		timewait;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, AcceptP_Async);

	if (ppCEP)
	{
		*ppCEP = NULL;
	}

	// Check QPN/EECN has been supplied
	if (! pSendConnInfo || (pSendConnInfo->Info.Reply.QPN == 0
							&& pSendConnInfo->Info.Reply.EECN == 0))
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}


	SpinLockAcquire(&gCM->ListLock);

	Status = GetPendingCEP(pCEP, &pCEPEntry);
	if (Status != FSUCCESS)
		goto unlock;

	//BitClear(pCEPEntry->EventFlags, CME_REQ_RCVD);

	elapsed_us = CmGetElapsedTime(pCEPEntry);

	// Update the turnaround time
	gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us, elapsed_us);

	_DBG_INFO(("Update turnaround time <tat %"PRIu64"us>.\n", gCM->turnaround_time_us));

	// check RDMA Read parameters have not been incorrectly increased
	if (pCEPEntry->LocalInitiatorDepth <
							pSendConnInfo->Info.Reply.ArbInitiatorDepth
		|| pCEPEntry->LocalResponderResources <
							pSendConnInfo->Info.Reply.ArbResponderResources)
	{
		Status = FINVALID_PARAMETER;
		goto unlock;
	}

	if (pCEPEntry->bPeer)
	{
		// API design decision, must use same EndPoint in CmAccept
		// as did in original PeerConnect.  We enforce this
		// (as opposed to blindly replacing Reply or CEP info)
		// to catch applications which end up with 2 QPs for the CEP
		if (pCEPEntry->LocalEndPoint.QPN != pSendConnInfo->Info.Reply.QPN
			|| pCEPEntry->LocalEndPoint.EECN != pSendConnInfo->Info.Reply.EECN)
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
	} else {
		// Save the relevent user reply info to this object
		pCEPEntry->LocalEndPoint.QPN = pSendConnInfo->Info.Reply.QPN;
		pCEPEntry->LocalEndPoint.EECN = pSendConnInfo->Info.Reply.EECN;
		if (! CM_MapTryInsert(&gCM->LocalEndPointMap, (uintn)pCEPEntry,
				&pCEPEntry->LocalEndPointMapEntry, "LOCAL_END_POINT_LIST", pCEPEntry))
		{
			Status = FCM_ADDR_INUSE;
			pCEPEntry->LocalEndPoint.QPN = 0;
			pCEPEntry->LocalEndPoint.EECN = 0;
			goto unlock;
		}

		// Save the disconnect callback
		pCEPEntry->pfnUserCallback = pfnCallback;
		pCEPEntry->UserContext = Context;

		pCEPEntry->bAsyncAccept = pCEP->bAsyncAccept;
		pCEPEntry->bTimewaitCallback = pCEP->bTimewaitCallback;
		pCEPEntry->bFailoverSupported = pCEP->bFailoverSupported;
		pCEPEntry->turnaround_time_us = pCEP->turnaround_time_us;

		// Use this user-specified event obj instead of the one we created internally
		if (hEvent)
		{
			OS_WAIT_OBJ_HANDLE hCEPEventObj = (OS_WAIT_OBJ_HANDLE)pCEPEntry->pEventObj;
			boolean            RemoveReference = FALSE;

			if (hCEPEventObj)
			{
 				if (pCEPEntry->bPrivateEvent)
				{
					EventDealloc(pCEPEntry->pEventObj);
				}
				else
				{
					RemoveReference = TRUE;
				}
			}

			pCEPEntry->bPrivateEvent = 0;

			pCEPEntry->pEventObj = (EVENT*)hEvent;

			OsWaitObjAddRef((OS_WAIT_OBJ_HANDLE)pCEPEntry->pEventObj);

			if (RemoveReference)
			{
				OsWaitObjRemoveRef(hCEPEventObj);
			}
		}

		if (hWaitEvent)
		{
			pCEPEntry->pWaitEvent = (EVENT*)hWaitEvent;
		}
	}

	if (pCEP->AlternatePath.LocalLID != 0)
	{
		if (pSendConnInfo->Info.Reply.FailoverAccepted != CM_REP_FO_ACCEPTED)
		{
			MemoryClear(&pCEP->AlternatePath, sizeof(pCEP->AlternatePath));
		}
		pCEPEntry->bFailoverSupported &= IsFailoverSupported(
							pSendConnInfo->Info.Reply.FailoverAccepted);
	}

	// save arbitrated RDMA Read parameters
	pCEPEntry->LocalInitiatorDepth =
							pSendConnInfo->Info.Reply.ArbInitiatorDepth;
	pCEPEntry->LocalResponderResources =
							pSendConnInfo->Info.Reply.ArbResponderResources;
	if (pSendConnInfo->Info.Reply.TargetAckDelay > pCEP->TargetAckDelay)
	{
		pCEP->TargetAckDelay = pSendConnInfo->Info.Reply.TargetAckDelay;
	}
	pCEP->LocalRecvPSN = pSendConnInfo->Info.Reply.StartingPSN;

	// update timewait 
	// some applications don't set TargetAckDelay properly
	// and to be safe we want our timewait to be no less than remote end
	// so be conservative and only increase it, the computation below
	// due to the round up of PktLifeTime, essentially always
	// adds TargetAckDelay to the timewait
	timewait = TimeoutTimeToMult(
						(TimeoutMultToTimeInUsec(pCEPEntry->PktLifeTime)<<1)
							+ TimeoutMultToTimeInUsec(pCEP->TargetAckDelay));
	if (pCEPEntry->Timewait < timewait)
		pCEPEntry->Timewait = timewait;

	FormatREP((CM_MAD*)GsiDgrmGetSendMad(pCEPEntry->pDgrmElement),
				&pSendConnInfo->Info.Reply,
				pCEPEntry->TransactionID,
				pCEPEntry->LocalCommID,
				pCEPEntry->RemoteCommID,
				pCEPEntry->LocalEndPoint.CaGUID);

	_DBG_INFO(("<cep 0x%p> REP's dgrm address field <portguid 0x%"PRIx64", dlid 0x%x, slm %d, pathbits %d, rate %d>\n",
				_DBG_PTR(pCEPEntry), 
				pCEPEntry->pDgrmElement->PortGuid,
				pCEPEntry->pDgrmElement->RemoteLID,
				pCEPEntry->pDgrmElement->ServiceLevel,
				pCEPEntry->pDgrmElement->PathBits,
				pCEPEntry->pDgrmElement->StaticRate));

	// Send out the REP
	Status = CmDgrmSend(pCEPEntry->pDgrmElement);
	if (FSUCCESS != Status)
	{
		// fall through and let timer retry the send later
		_DBG_WARN(("<cep 0x%p> DgrmSend() failed for REP!!! <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		Status = FSUCCESS;
	} else {
		AtomicIncrementVoid(&gCM->Sent.Rep);
	}
	
	_DBG_INFO(("<cep 0x%p> REP sent <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>.\n", 
				_DBG_PTR(pCEPEntry), pCEPEntry->LocalCommID, pCEPEntry->RemoteCommID,
				pCEPEntry->PrimaryPath.LocalLID, pCEPEntry->PrimaryPath.RemoteLID));

	CepRemoveFromPendingList(pCEPEntry);

	CepSetState(pCEPEntry, CMS_REP_SENT);

	// For peer connection, leave the entry in the LISTEN list

	// Start timer to timeout the REP i.e. wait for the RTU msg to arrive
	pCEPEntry->RetryCount = 0;
	rep_timeout_ms = TimeoutMultToTimeInMs(pCEPEntry->LocalCMTimeout);
	CmTimerStart(pCEPEntry, rep_timeout_ms, REP_TIMER);

	if (willWait)
	{
		// caller will be doing a WaitP
		// prevent a callback until caller WaitP completes
		AtomicIncrementVoid(&pCEPEntry->CallbackRefCnt);
		AtomicIncrementVoid(&gCM->TotalCallbackRefCnt);
	}
		
	// If another connection is pending for this server endpoint, 
	// we must set it again to wake up the Wait()
	if (pCEP->PendingCount)
	{
		SetNotification(pCEP, CME_RCVD_REQ);//EventSet(pCEP, CME_RCVD_REQ);
	}

	// Return the new CEP object to the user
	if (ppCEP)
	{
		*ppCEP = pCEPEntry;
	}
	Status = FPENDING;

unlock:
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // AcceptP_Asynch()


//////////////////////////////////////////////////////////////////////////
// AcceptP
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
//	FCM_CONNECT_ESTABLISHED - Connection established
//	FCM_CONNECT_REJECT		- Connection rejected
//	FCM_CONNECT_TIMEOUT		- Connection timeout
//	FCM_CONNECT_CANCEL		- Connection cancel
//
//	FINVALID_STATE			- The endpoint is not in the valid state for this call
//	FERROR					- Nothing to accept, cannot accept, or unable to send the reply packet
//	FTIMEOUT				- The timeout interval expires
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
AcceptP(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_CONN_INFO*		pSendConnInfo,		// Send REP
	OUT CM_CONN_INFO*			pRecvConnInfo,		// Rcvd RTU, REJ or TIMEOUT
	IN PFN_CM_CALLBACK			pfnCallback,
	IN void*					Context,
	OUT CM_CEP_OBJECT**			ppNewCEP
	)
{
	CM_CEP_OBJECT*	pNewCEP=NULL;
	FSTATUS		Status=FSUCCESS;
	uint32		rep_timeout_ms=0;
	boolean bPeer = pCEP->bPeer;	// save in case move Cep to Idle

	_DBG_ENTER_EXT(_DBG_LVL_FUNC_TRACE, AcceptP, == PASSIVE);

	UNUSED_VAR(rep_timeout_ms);

	if (! pRecvConnInfo && ! pCEP->bAsyncAccept)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	// Accept client or peer connection. Send the REP to the remote client 
	// use ppNewCEP if not-NULL so caller gets updated while in Lock in
	// AcceptP_Async
	Status = AcceptP_Async(pCEP, pSendConnInfo, NULL, NULL, pfnCallback, Context, ! pCEP->bAsyncAccept, ppNewCEP?ppNewCEP:&pNewCEP);

	if (Status != FSUCCESS && Status != FPENDING)
		goto done;
	if (ppNewCEP)
		pNewCEP = *ppNewCEP;

	if (bPeer)
	{
		ASSERT(pNewCEP == pCEP);
	}

	if (pCEP->bAsyncAccept)
	{
		// we will provide a callback when RTU or REJ arrives or timeout
		goto done;
	}

	// Wait for RTU or REJ to arrive or timeout event to occur
	// TODO: We should put a time limit here in case we dont get the timeout event ??
	Status = WaitP(pNewCEP, pRecvConnInfo, EVENT_NO_TIMEOUT);
	//rep_timeout_ms = TimeoutMultToTimeInMs(pNewCEP->LocalCMTimeout);
	//Status = WaitP(pNewCEP, pRecvConnInfo, 2*rep_timeout_ms, TRUE);
	ASSERT(Status == FSUCCESS);

	// This is the status we returned to the caller
	Status = pRecvConnInfo->Status;

	// Check the status here instead of the state since the status is what
	// we return to the caller
	if (Status == FCM_CONNECT_ESTABLISHED)
	{
		//
		// Note: Though the status is established, the pNewCEP state may not be in
		// CMS_ESTABLISHED since we may have already received a DREQ
		//
		_DBG_INFO(("<cep 0x%p> *** Connection established *** <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID, 
					pNewCEP->PrimaryPath.LocalLID, pNewCEP->PrimaryPath.RemoteLID,
					_DBG_PTR(CmGetStateString(pNewCEP->State))));

	} else {
		switch (Status)
		{
		case FCM_CONNECT_CANCEL: // USRE_CANCEL
			_DBG_INFO(("<cep 0x%p> *** Connection cancelled *** <lcid 0x%x rcid 0x%x %s %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pNewCEP->State)), _DBG_PTR(FSTATUS_MSG(Status))));
			break;

		case FCM_CONNECT_REJECT: // CME_RCVD_REJ
			_DBG_INFO(("<cep 0x%p> *** Connection rejected *** <lcid 0x%x rcid 0x%x %s %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pNewCEP->State)), _DBG_PTR(FSTATUS_MSG(Status))));
			break;

		case FCM_CONNECT_TIMEOUT: // CME_TIMEOUT_REP
			_DBG_INFO(("<cep 0x%p> *** Connection timed out *** <lcid 0x%x rcid 0x%x %s %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pNewCEP->State)), _DBG_PTR(FSTATUS_MSG(Status))));
			break;

		case FCM_DISCONNECT_REQUEST:
			_DBG_INFO(("<cep 0x%p> *** Disconnecting *** <lcid 0x%x rcid 0x%x %s %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pNewCEP->State)), _DBG_PTR(FSTATUS_MSG(Status))));
			break;
		
		case FCM_DISCONNECTED:
			_DBG_INFO(("<cep 0x%p> *** Disconnected *** <lcid 0x%x rcid 0x%x %s %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pNewCEP->State)), _DBG_PTR(FSTATUS_MSG(Status))));
			break;

		default:
			_DBG_INFO(("<cep 0x%p> *** Connection ??? *** <lcid 0x%x rcid 0x%x %s %s>.\n", 
					_DBG_PTR(pNewCEP), pNewCEP->LocalCommID, pNewCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pNewCEP->State)), _DBG_PTR(FSTATUS_MSG(Status))));
			ASSERT(0);
			break;
		
		} // switch()

		if (!bPeer)
		{
			// This is the one we created when a new connection request arrived
			iba_cm_destroy_cep(pNewCEP);
		}
		if (ppNewCEP)
			*ppNewCEP = NULL;
	}
	CmDoneCallback(pNewCEP);

done:
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);

	return Status;

} // AcceptP()


//////////////////////////////////////////////////////////////////////////
// RejectP
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
RejectP(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_REJECT_INFO*	pConnectReject
	)
{
	CM_CEP_OBJECT* pCEPEntry = NULL;
	DLIST_ENTRY* pListEntry=NULL;
	FSTATUS	Status=FSUCCESS;
	boolean bPeer = pCEP->bPeer;	// in case move Cep to Idle
		
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, RejectP);

	switch (pCEP->State)
	{
	case CMS_IDLE:
		if (BitTest(pCEP->EventFlags, USRE_CANCEL))
		{
			// User cancelled the listen, but hasn't gotten callback yet
			Status = FCM_CONNECT_CANCEL;
		}
		else
		{
			Status = FERROR;
		}
		break;

	case CMS_LISTEN:
	case CMS_REQ_RCVD:	// TBD only occurs for actual CEP (can be Peer), separate out
	case CMS_MRA_REQ_SENT:	// TBD only occurs for actual CEP (can be Peer), separate out
		// if separate out above and did same in AcceptP, won't need Peer on PendingList

		//
		// Rejecting the incoming connect or peer request
		//
		if (DListIsEmpty(&pCEP->PendingList))
		{
			// Nothing to reject
			_DBG_WARNING(("<cep 0x%p> Pending list is empty!\n", _DBG_PTR(pCEP)));

			Status = FNOT_FOUND;
			break;
		}

		// Pull the incoming connection
		pListEntry = DListGetNext(&pCEP->PendingList);
		pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);
		ASSERT(pCEPEntry->pParentCEP == pCEP);

		// Cannot reject until the user has been notify thru wait
		/*
		if (!BitTest(pCEPEntry->EventFlags, USRE_WAIT))
		{
			Status = FUNAVAILABLE;
			break;
		}

		BitClear(pCEPEntry->EventFlags, USRE_WAIT);
		*/

		ASSERT(pCEPEntry->State == CMS_REQ_RCVD || pCEPEntry->State == CMS_MRA_REQ_SENT);
		// Format the REJ msg
		FormatREJ((CM_MAD*)GsiDgrmGetSendMad(pCEPEntry->pDgrmElement),
					CM_REJECT_REQUEST,
					(uint16)pConnectReject->Reason,
					&pConnectReject->RejectInfo[0],
					pConnectReject->RejectInfoLen,
					&pConnectReject->PrivateData[0],
					CMM_REJ_USER_LEN,
					pCEPEntry->TransactionID,
					pCEPEntry->LocalCommID,
					pCEPEntry->RemoteCommID);

		// Send out the REJ. We do not need to check the return since
		// the other end will eventually timed-out if it does not received
		// either RTU or REJ
		Status = CmDgrmSend(pCEPEntry->pDgrmElement);
		if (FSUCCESS != Status)
		{
			// fall through and let timer retry the send later
			_DBG_WARN(("<cep 0x%p> DgrmSend() failed for REJ!!! <%s>\n",
							_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
			Status = FSUCCESS;
		} else {
			AtomicIncrementVoid(&gCM->Sent.RejReq);
		}
		
		_DBG_INFO(("<cep 0x%p> REJ sent, rejecting REQ <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x reason %d>.\n", 
					_DBG_PTR(pCEPEntry), pCEPEntry->LocalCommID, pCEPEntry->RemoteCommID,
					pCEPEntry->PrimaryPath.LocalLID, pCEPEntry->PrimaryPath.RemoteLID,
					pConnectReject->Reason));

		// Move from PENDING to INIT list
		CepRemoveFromPendingList(pCEPEntry);
		CepToIdle(pCEPEntry);

		if (bPeer)
		{
			// For peer request, the user created the obj 
			ASSERT(pCEPEntry == pCEP);
		} else {
			// This is the one we created when a new connection request arrived
			DestroyCEP(pCEPEntry);
		}

		// If another connection is pending for this server endpoint, 
		// we must set it again to wake up the Wait()
		if (pCEP->PendingCount)
		{
			SetNotification(pCEP, CME_RCVD_REQ);//EventSet(pCEP, CME_RCVD_REQ);
		}

		break;
	// TBD case CMS_REP_SENT: - allow and move to TimeWait, see Cancel
	// TBD case CMS_MRA_REP_RCVD: - allow and move to TimeWait, see Cancel

	default:

		Status = FINVALID_STATE;
		break;

	} // switch()
			
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // RejectP()

//////////////////////////////////////////////////////////////////////////
// CancelP
// CmCancel, DestroyCEP or remove CA while connecting
// 
// Cancel processing of a passive side connection
// all local information is dropped, remote end is left to timeout
// user will get a FCM_CONNECT_CANCEL callback at which point user
// should destroy the now-idle CEP
//
// In general this should be used only to cancel listeners.
// Internally it is used to do a quick cleanup for a Destroyed CEP
// or the removal of a CA, hence it does not attempt to send a Reject
// Applications should generally use CmReject instead of this to
// properly reject connections.
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
CancelP(
	IN CM_CEP_OBJECT*			pCEP
	)
{
	FSTATUS			Status=FPENDING;

	CM_CEP_OBJECT*	pCEPEntry=NULL;
	DLIST_ENTRY*		pListEntry=NULL;

	switch (pCEP->State)
	{
	case CMS_LISTEN:
		//
		// Cancel the listen request
		//

		// Clear the pending signal from the event obj since a REQ may be pending
		// but before the signal is waited on
		//CmEventReset(pCEP);

		//BitSet(pCEP->EventFlags, USRE_CANCEL);

		// Clear the pending list
		while (!DListIsEmpty(&pCEP->PendingList))
		{
			pListEntry = DListGetNext(&pCEP->PendingList);
			pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);
			// Move from PENDING to INIT list and destroy it
			ASSERT(pCEPEntry->pParentCEP == pCEP);
			CepRemoveFromPendingList(pCEPEntry);
			CepToIdle(pCEPEntry);

			// This will start the timer in timewait state
			DestroyCEP(pCEPEntry);
		}
		ASSERT(pCEP->PendingCount == 0);

		// Move from LISTEN to IDLE
		CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEP);

		// Set the cancel event
		SetNotification(pCEP, USRE_CANCEL);

		// This tells us that a callback is pending or we are in the callback
		//if (AtomicRead(&pCEP->CallbackRefCnt))
		//	Status = FPENDING;

		break;

	case CMS_REQ_RCVD:
	case CMS_MRA_REQ_SENT:
		// Internal use, removed CA or cancelling listening CEP
		ASSERT(!pCEP->bPeer || (pCEP->LocalEndPoint.QPN || pCEP->LocalEndPoint.EECN));

#if 0
		// TBD - call RejectP, will end up in Idle
#else
		// Move from PENDING to IDLE
		CepRemoveFromPendingList(pCEP);
		CepToIdle(pCEP);
#endif

		// Set the cancel event
		SetNotification(pCEP, USRE_CANCEL);

		break;
	
	case CMS_REP_SENT:
	case CMS_MRA_REP_RCVD:
		//
		// Cancel the connection while waiting for the RTU to arrive.
		// 

		// Cancel the timer for REP msg
		CmTimerStop(pCEP, REP_TIMER);

		// Clear the pending signal from the event obj since a RTU may be pending
		// but before the signal is waited on
		//CmEventReset(pCEP);

		//BitSet(pCEP->EventFlags, USRE_CANCEL);
#if 0
		// TBD - we should do this
		send a REJ (timeout or similar) - use RejectP
		CepToTimewait(pCEP)
		// if user wants a FCM_DISCONNECTED at end of timewait, lets not confuse
		// them with a FCM_CONNECT_CANCEL callback before it
		if (! pCEP->bTimewaitCallback)
		{
			if (pCEP->bAsyncAccept)
			{
				SetNotification(pCEP, USRE_CANCEL);	// callback
			} else {
				EventSet(pCEP, USRE_CANCEL, FALSE);	// wakeup CmAccept wait
			}
		}
#else
		CepToIdle(pCEP);

		if (pCEP->bAsyncAccept)
		{
			SetNotification(pCEP, USRE_CANCEL);	// callback
		} else {
			EventSet(pCEP, USRE_CANCEL, FALSE);	// wakeup CmAccept wait
		}
#endif
		// This tells us that a callback is pending or we are in the callback
		//if (AtomicRead(&pCEP->CallbackRefCnt))
		//	Status = FPENDING;

		//if (BitTest(pCEP->EventFlags, USRE_WAIT))
		//{
		//	Status = FPENDING;
		//}

		//EventSet(pCEP, USRE_CANCEL);
		
		break;

	case CMS_SIDR_REQ_RCVD:
		// Move from PENDING to INIT list
		CepRemoveFromPendingList(pCEP);
		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEP);

		// Set the cancel event - this CEP not user visible, so no callback
		EventSet(pCEP, USRE_CANCEL, FALSE);
		break;

	default:

		Status = FINVALID_STATE;
		break;

	} // switch()
	
	return Status;

} // CancelP()

// build QP Attributes to load new alternate path into QP
// and prepare a Reply
FSTATUS ProcessAltPathRequestP(
	IN CM_CEP_OBJECT* pCEP,
	IN CM_ALTPATH_INFO*			AltPathRequest,
	OUT CM_PROCESS_ALTPATH_REQUEST_INFO* Info
	)
{
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessAltPathRequestP);

	ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
	switch (pCEP->State)
	{
	case CMS_LAP_RCVD:
	case CMS_MRA_LAP_SENT:
		MemoryClear(Info, sizeof(*Info));	// just to be safe
		Info->QpAttrRts.RequestState = QPStateReadyToSend;
		Info->QpAttrRts.APMState = APMStateRearm;
		GetAVFromPath2(0, &AltPathRequest->AlternatePathInfo.Path, NULL,
						&Info->QpAttrRts.AltDestAV);
		Info->QpAttrRts.AltDestAV.PortGUID = pCEP->TempAlternateLocalPortGuid;
		Info->QpAttrRts.AltDestAV.GlobalRouteInfo.SrcGIDIndex =
											pCEP->TempAlternateLocalGidIndex;
		Info->QpAttrRts.AltDestAV.PathBits = pCEP->TempAlternateLocalPathBits;
		Info->QpAttrRts.AltPortGUID = pCEP->TempAlternateLocalPortGuid;
		Info->QpAttrRts.AltPkeyIndex = pCEP->TempAlternatePkeyIndex;
		Info->QpAttrRts.Attrs = IB_QP_ATTR_APMSTATE
						|IB_QP_ATTR_ALTDESTAV|IB_QP_ATTR_ALTPORTGUID
						| IB_QP_ATTR_ALTPKEYINDEX;
		if (pCEP->Type != CM_UC_TYPE) {
			Info->QpAttrRts.AltLocalAckTimeout = AltPathRequest->AlternateAckTimeout;
			Info->QpAttrRts.Attrs |= IB_QP_ATTR_ALTLOCALACKTIMEOUT;
		}
		Info->AltPathReply.APStatus = APS_PATH_LOADED;
		// caller must iba_modify_qp(...) then iba_altpath_reply
		break;

	default:
		Status = FINVALID_STATE;
		break;
	}
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}


//////////////////////////////////////////////////////////////////////////
// AltPathReplyP
//
// Reply to an alternate path request
// Only allowed for Server/Passive side of connection
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
//	FSUCCESS - APR queued to be sent.
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the apr packet
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
AltPathReplyP(
	IN CM_CEP_OBJECT* 			pCEP,
	IN CM_ALTPATH_REPLY_INFO*	pAprInfo		// Send APR
	)
{
	FSTATUS		Status;
	CM_MAD*		pMad = NULL;
	//uint64		elapsed_us=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, AltPathReplyP);

	ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
	switch (pCEP->State)
	{
		case CMS_LAP_RCVD:
		case CMS_MRA_LAP_SENT:
			pMad = (CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement);

			if (pAprInfo->APStatus == APS_PATH_LOADED)
			{
				// still have the LAP we received, now we have
				// accepted it as our alternate path
				ASSERT(pMad->common.AttributeID == MCLASS_ATTRIB_ID_LAP);
				CopyRcvdLAPToAltPath(&pMad->payload.LAP, &pCEP->AlternatePath);
				pCEP->AlternatePath.LocalPortGuid = pCEP->TempAlternateLocalPortGuid;
				pCEP->AlternatePath.LocalGidIndex = pCEP->TempAlternateLocalGidIndex;
				pCEP->AlternatePath.LocalPathBits = pCEP->TempAlternateLocalPathBits;
				pCEP->AlternatePath.PkeyIndex = pCEP->TempAlternatePkeyIndex;
			} else if (pAprInfo->APStatus == APS_UNSUPPORTED_REQ) {
				// in future no need to notify CEP of LAPs, it doesn't do APM
				pCEP->bFailoverSupported = FALSE;
			}

			// Update the turnaround time
			// uncomment the 3 lines below to have LAP/APR response count toward
			// average turnaround time
			//elapsed_us = CmGetElapsedTime(pCEPEntry);
			//gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us, elapsed_us);
			//_DBG_INFO(("Update turnaround time <tat %"PRIu64"us>.\n", gCM->turnaround_time_us));

			FormatAPR(pMad, (CM_APR_STATUS) pAprInfo->APStatus,
						pAprInfo->u.AddInfo.Info,
						pAprInfo->u.AddInfo.Len,
						pAprInfo->PrivateData,
						CM_APR_INFO_USER_LEN,
						pCEP->TransactionID,
						pCEP->LocalCommID,
						pCEP->RemoteCommID);

			// Send out the APR
			Status = CmDgrmSend(pCEP->pDgrmElement);
			if (FSUCCESS != Status)
			{
				// fall through and let timer retry the send later
				_DBG_WARN(("<cep 0x%p> DgrmSend() failed for APR!!! <%s>\n",
							_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
				Status = FSUCCESS;
			} else {
				if (pAprInfo->APStatus == APS_PATH_LOADED)
					AtomicIncrementVoid(&gCM->Sent.AprAcc);
				else
					AtomicIncrementVoid(&gCM->Sent.AprRej);
			}
	
			_DBG_INFO(("<cep 0x%p> APR sent <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>.\n", 
						_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
						pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));

			CepSetState(pCEP, CMS_ESTABLISHED);
			break;
		default:
			Status = FINVALID_STATE;
			break;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // AltPathReplyP()

// EOF
