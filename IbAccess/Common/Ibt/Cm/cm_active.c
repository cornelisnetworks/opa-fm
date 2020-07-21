/* BEGIN_ICS_COPYRIGHT4 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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
// Connect
//
// This routine is called by CmConnect() and CmConnectPeer()
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
//	This routine is called at IRQL_PASSIVE_LEVEL with ListLock held
//
FSTATUS
Connect(
	CM_CEP_OBJECT*			pCEP,
	const CM_REQUEST_INFO*	pConnectRequest,
	boolean					bPeer,		// TRUE if peer connect, FALSE otherwise
	PFN_CM_CALLBACK			pfnConnectCB,
	void*					Context
	)
{
	FSTATUS				Status=FSUCCESS;
	IB_PORT_ATTRIBUTES	*pPortAttr=NULL;
	uint32				pktlifetime=0;
	uint64				pktlifetime_us=0;
	uint32				req_timeout_ms=0;
	uint64				turnaround_time_us;
	IB_CA_ATTRIBUTES	*pCaAttr = NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, Connect);

	DEBUG_ASSERT(pCEP->State == CMS_IDLE);
	if (pCEP->State != CMS_IDLE) 
	{
		_DBG_ERROR(("<cep %p> invalid State for Connect!!! <%d>\n",
					_DBG_PTR(pCEP), pCEP->State));
		Status = FINVALID_PARAMETER;
		goto fail_type;
	}

	if (pCEP->Type != CM_RC_TYPE
		&& pCEP->Type != CM_UC_TYPE
		&& pCEP->Type != CM_RD_TYPE
		)
	{
		_DBG_WARN(("<cep 0x%p> invalid Type for Connect!!! <%d>\n",
						_DBG_PTR(pCEP), pCEP->Type));
		Status = FINVALID_PARAMETER;
		goto fail_type;
	}
	if (pConnectRequest->CEPInfo.QPN == 0
		&& pConnectRequest->CEPInfo.LocalEECN == 0)
	{
		_DBG_WARN(("<cep 0x%p> invalid QPN/EECN for Connect!!! <both 0>\n",
						_DBG_PTR(pCEP)));
		Status = FINVALID_PARAMETER;
		goto fail_type;
	}
	if (pConnectRequest->SID == 0)
	{
		_DBG_WARN(("<cep 0x%p> invalid SID for Connect!!! <%"PRIx64">\n",
						_DBG_PTR(pCEP), pConnectRequest->SID));
		Status = FINVALID_PARAMETER;
		goto fail_type;
	}
	// Set up addressing information
	pCEP->LocalEndPoint.CaGUID = pConnectRequest->CEPInfo.CaGUID;
	pCEP->bPeer = bPeer;

	// Set up Bind address, especially important for Peer connect
	// which will end up on ListenMap which is keyed by
	// SID, Local&Remote LID&GID, EndPoint and Private Data Discriminator

	pCEP->SID = pConnectRequest->SID;

	CopyPathInfoToCepPath(&pCEP->PrimaryPath, &pConnectRequest->PathInfo,
					pConnectRequest->CEPInfo.AckTimeout);
	Status = iba_find_ca_gid2(&pConnectRequest->PathInfo.Path.SGID,
					pConnectRequest->CEPInfo.CaGUID,
					&pCEP->PrimaryPath.LocalPortGuid,
					&pCEP->PrimaryPath.LocalGidIndex, &pPortAttr,
					&pCaAttr);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> FindCaGid() failed!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		Status = FINVALID_PARAMETER;
		goto fail_port;
	}
	// as a safety feature we quietly downgrade the RDMA Read resources to
	// match the CA capabilities
	if (pCEP->Type == CM_RC_TYPE) {
		pCEP->LocalInitiatorDepth = MIN(pCaAttr->MaxQPInitiatorDepth,
										pConnectRequest->CEPInfo.OfferedInitiatorDepth);
		pCEP->LocalResponderResources = MIN(pCaAttr->MaxQPResponderResources,
										pConnectRequest->CEPInfo.OfferedResponderResources);
	} else if (pCEP->Type == CM_RD_TYPE) {
		pCEP->LocalInitiatorDepth = MIN(pCaAttr->MaxEECInitiatorDepth,
										pConnectRequest->CEPInfo.OfferedInitiatorDepth);
		pCEP->LocalResponderResources = MIN(pCaAttr->MaxEECResponderResources,
										pConnectRequest->CEPInfo.OfferedResponderResources);
	} else {
		pCEP->LocalInitiatorDepth = 0;
		pCEP->LocalResponderResources = 0;
	}
	if (! LookupPKey(pPortAttr, pConnectRequest->PathInfo.Path.P_Key, FALSE, &pCEP->PrimaryPath.PkeyIndex))
	{
		_DBG_WARN(("<cep 0x%p> LookupPKey() failed!!!\n", _DBG_PTR(pCEP)));
		Status = FINVALID_PARAMETER;
		goto fail_port;
	}
	pCEP->PrimaryPath.LocalPathBits = LidToPathBits(pCEP->PrimaryPath.LocalLID, pPortAttr->Address.LMC);
	MemoryDeallocate(pPortAttr);
	pPortAttr = NULL;

	if (pConnectRequest->AlternatePathInfo.Path.SLID != 0)
	{
		if (pConnectRequest->PathInfo.Path.P_Key
			!= pConnectRequest->AlternatePathInfo.Path.P_Key)
		{
			Status = FINVALID_PARAMETER;
			goto fail_port;
		}
		CopyPathInfoToCepPath(&pCEP->AlternatePath, &pConnectRequest->AlternatePathInfo,
					pConnectRequest->CEPInfo.AlternateAckTimeout);
		Status = iba_find_ca_gid(&pConnectRequest->AlternatePathInfo.Path.SGID,
						pConnectRequest->CEPInfo.CaGUID,
						&pCEP->AlternatePath.LocalPortGuid,
						&pCEP->AlternatePath.LocalGidIndex, &pPortAttr, NULL);
		if (Status != FSUCCESS)
		{
			_DBG_WARN(("<cep 0x%p> Alternate FindCaGid() failed!!! <%s>\n",
							_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
			Status = FINVALID_PARAMETER;
			goto fail_port;
		}
		if (! LookupPKey(pPortAttr, pConnectRequest->AlternatePathInfo.Path.P_Key, FALSE, &pCEP->AlternatePath.PkeyIndex))
		{
			_DBG_WARN(("<cep 0x%p> Alternate LookupPKey() failed!!!\n",
					_DBG_PTR(pCEP)));
			Status = FINVALID_PARAMETER;
			goto fail_port;
		}
		pCEP->AlternatePath.LocalPathBits = LidToPathBits(pCEP->AlternatePath.LocalLID, pPortAttr->Address.LMC);
		MemoryDeallocate(pPortAttr);
		pPortAttr = NULL;
	} else {
		MemoryClear(&pCEP->AlternatePath, sizeof(pCEP->AlternatePath));
	}

	pCEP->Mtu = pConnectRequest->PathInfo.Path.Mtu;
	pCEP->PartitionKey = pConnectRequest->PathInfo.Path.P_Key;

	pCEP->TargetAckDelay = pCaAttr->LocalCaAckDelay;	// best guess to start

	MemoryDeallocate(pCaAttr);
	pCaAttr = NULL;

	// save info for later
	pCEP->LocalRetryCount = pConnectRequest->CEPInfo.RetryCount;
	pCEP->LocalRecvPSN= pConnectRequest->CEPInfo.StartingPSN;

		// Bind the local endpoint addr to the object
	pCEP->LocalEndPoint.QPN = pConnectRequest->CEPInfo.QPN;
	pCEP->LocalEndPoint.EECN = pConnectRequest->CEPInfo.LocalEECN;

	pCEP->RemoteEndPoint.QPN = pCEP->RemoteEndPoint.EECN = 0;

		// make sure LocalEndPoint is unique
	if (! CM_MapTryInsert(&gCM->LocalEndPointMap, (uintn)pCEP,
						&pCEP->LocalEndPointMapEntry, "LOCAL_END_POINT_LIST", pCEP))
	{
		Status = FCM_ADDR_INUSE;
		goto fail_endpoint;
	}
	// For peer connection, put it onto the LISTEN list
	// this also makes sure the bound listen address is unique
	// Once we associate the CEP with a remote end point, we will remove the CEP
	// from the LISTEN list. For the passive end, we will put it onto the
	// PENDING list to facilitate CmAccept issuing REP
	if (bPeer && ! CM_MapTryInsert(&gCM->ListenMap, (uintn)pCEP,
									&pCEP->MapEntry, "LISTEN_LIST", pCEP))
	{
		Status = FCM_ADDR_INUSE;
		goto fail;
	}

	pCEP->EventFlags = 0;
	pCEP->RetryCount = 0;
	DListInit(&pCEP->PendingList);

	pCEP->Mode = CM_MODE_ACTIVE;
	pCEP->bFailoverSupported = TRUE;	// assume the best until we know better

	// The comm ID is set here so that each connect request is unique since
	// user can reuse the object. This prevents duplicate msg arriving for
	// previous connect request using the same object.
	AssignLocalCommID(pCEP);
	pCEP->TransactionID = (pCEP->LocalCommID)<<8;
	_DBG_INFO(("<cep 0x%p> Comm ID set at <lcid 0x%x>.\n",
				_DBG_PTR(pCEP), pCEP->LocalCommID));

	pCEP->QKey	= pConnectRequest->CEPInfo.QKey;

	pktlifetime = pConnectRequest->PathInfo.Path.PktLifeTime;
	if (pktlifetime > 31)
		pktlifetime = 31;
	pCEP->PktLifeTime = pktlifetime;
	pCEP->Timewait = pConnectRequest->CEPInfo.AckTimeout;
	if (pCEP->Timewait < pCEP->PktLifeTime+1)
	{
		// round up Timewait to at least be 2*PktLifeTime
		// TBD could add in LocalCaAckDelay and adjust
		// AckTimeout as well
		pCEP->Timewait = pCEP->PktLifeTime+1;	 // log2 so +1 is *2
	}

	// Round-trip time in usec
	pktlifetime_us = TimeoutMultToTimeInUsec(pktlifetime);

	// turnaround_time_us should include CM and appl software time and
	// the hfi hardware Ca Ack Delay
	if (pCEP->turnaround_time_us)
	{
		turnaround_time_us = pCEP->turnaround_time_us;	// CEP specific time
	} else {
		turnaround_time_us = gCM->turnaround_time_us;	// use heuristic
	}

	// The remote timeout is used by the remote CM to determine if it should
	// send an MRA, in theory turnaround_time_us should be anticipated
	// turnaround time of remote end.
	pCEP->RemoteCMTimeout = TimeoutTimeToMult(turnaround_time_us);

	// The local timeout is used to timeout all waits for responses
	// from request packets (REQ, REP, LAP, DREQ), it is the time a remote
	// CM can expect our local CM to respond within (including fabric time)
	pCEP->LocalCMTimeout = TimeoutTimeToMult(turnaround_time_us + pktlifetime_us*2);

	pCEP->MaxCMRetries = gCmParams.MaxReqRetry;
		
	_DBG_INFO(("<cep 0x%p> Timeout setting <pkt lifetime %u rtt %"PRIu64"us turnaround time %"PRIu64"us local timeout %u rem timeout %u>.\n", 
			_DBG_PTR(pCEP), pCEP->PktLifeTime, pktlifetime_us*2,
			turnaround_time_us, pCEP->LocalCMTimeout, pCEP->RemoteCMTimeout));

	// Save user callback info
	pCEP->pfnUserCallback	= pfnConnectCB;
	pCEP->UserContext		= Context;

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
			_DBG_WARNING(("<cep 0x%p> Connection attempt failed: Out of CM Dgrms!!!\n",
						_DBG_PTR(pCEP)));
			Status = FINSUFFICIENT_RESOURCES;
			goto fail_dgrm;
		}
	}

	DEBUG_ASSERT(pCEP->pDgrmElement->Element.pBufferList->ByteCount == MAD_BLOCK_SIZE);
	if (pCEP->pDgrmElement->Element.pBufferList->ByteCount != MAD_BLOCK_SIZE) {
		_DBG_ERROR(("<cep %p> Discarding Corrupt Datagram!!! pDgrmElement = %p, pCEP->pDgrmElement->Element.pBufferList = %p, pCEP->pDgrmElement->Element.pBufferList->ByteCount = %d\n",
					_DBG_PTR(pCEP),
					_DBG_PTR(pCEP->pDgrmElement), 
					_DBG_PTR(pCEP->pDgrmElement->Element.pBufferList), 
					pCEP->pDgrmElement->Element.pBufferList->ByteCount));
		Status = FINSUFFICIENT_RESOURCES;
		pCEP->pDgrmElement = NULL;
		goto fail_dgrm;
	}

	// Format the REQ msg
	FormatREQ((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement),
				pConnectRequest,
				pCEP->TransactionID, 
				pCEP->LocalCommID, 
				QP1_WELL_KNOWN_Q_KEY, 
				pCEP->Type, 
				pCEP->LocalCMTimeout, 
				pCEP->RemoteCMTimeout, 
				pCEP->MaxCMRetries,
				pCEP->LocalInitiatorDepth,
				pCEP->LocalResponderResources);

	// Setup the dgrm's dest addr		
	_DBG_INFO(("<cep 0x%p> REQ's dgrm address field <portguid 0x%" PRIx64", dlid 0x%x, sl %d, pathbits %d, rate %d>\n",
				_DBG_PTR(pCEP), 
				pCEP->PrimaryPath.LocalPortGuid,
				pCEP->PrimaryPath.RemoteLID,
				pCEP->PrimaryPath.SL,
				pCEP->PrimaryPath.LocalPathBits,
				pCEP->PrimaryPath.StaticRate));


	CmDgrmAddrSet(pCEP->pDgrmElement,
				pCEP->PrimaryPath.LocalPortGuid,
				pCEP->PrimaryPath.RemoteLID,
				pCEP->PrimaryPath.SL,
				pCEP->PrimaryPath.LocalPathBits,
				pCEP->PrimaryPath.StaticRate,
				(! pCEP->PrimaryPath.bSubnetLocal),
				pCEP->PrimaryPath.RemoteGID,
				pCEP->PrimaryPath.FlowLabel,
				pCEP->PrimaryPath.LocalGidIndex,
				pCEP->PrimaryPath.HopLimit,
				(uint8)pCEP->PrimaryPath.TClass,
				pCEP->PrimaryPath.PkeyIndex);

	// Pass the REQ down to GSA. 
	// At this point, we may get the send or receive complete callback but the lock
	// will prevent nested call.
	// Also, we update the state and start the timer here instead of in the send callback
	// because we assume the send is done here (i.e. unreliable send)
	Status = CmDgrmSend(pCEP->pDgrmElement);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> DgrmSend() failed for REQ!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		// fall through and let timer retry the send later
		Status = FSUCCESS;
	} else {
		AtomicIncrementVoid(&gCM->Sent.Req);
	}
	_DBG_INFO(("<cep 0x%p> REQ sent for <lcid 0x%x slid 0x%x dlid 0x%x>.\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID,
					pCEP->PrimaryPath.LocalLID,
					pCEP->PrimaryPath.RemoteLID));

	Status = FPENDING;

	CepSetState(pCEP, CMS_REQ_SENT);

	// Start the REQ timer. Use the Local CM timeout.
	req_timeout_ms = TimeoutMultToTimeInMs(pCEP->LocalCMTimeout);
	CmTimerStart(pCEP, req_timeout_ms, REQ_TIMER);
		
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

fail_dgrm:
	if (bPeer)
		CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
	pCEP->Mode = CM_MODE_NONE;
fail:
	CM_MapRemoveEntry(&gCM->LocalEndPointMap, &pCEP->LocalEndPointMapEntry, LOCAL_END_POINT_LIST, pCEP);
fail_endpoint:
fail_port:
	if (pCaAttr != NULL)
		MemoryDeallocate(pCaAttr);
	if (pPortAttr != NULL)
		MemoryDeallocate(pPortAttr);
	// cleanup CEP
	pCEP->bPeer = FALSE;
	pCEP->SID = 0;
	MemoryClear(&pCEP->PrimaryPath, sizeof(pCEP->PrimaryPath));
	MemoryClear(&pCEP->AlternatePath, sizeof(pCEP->AlternatePath));
	MemoryClear(&pCEP->LocalEndPoint, sizeof(pCEP->LocalEndPoint));
	MemoryClear(&pCEP->RemoteEndPoint, sizeof(pCEP->RemoteEndPoint));
	pCEP->Mtu = 0;
	pCEP->PartitionKey = 0;
	pCEP->LocalInitiatorDepth = 0;
	pCEP->LocalResponderResources = 0;
	pCEP->LocalRetryCount = 0;
	pCEP->LocalRecvPSN = 0;
	pCEP->TargetAckDelay = 0;
fail_type:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
} // Connect()


//////////////////////////////////////////////////////////////////////////
// GetConnInfo
// Supports Connection Info (for use in preparing callback arguments)
// for both Active and Passive sides.  Where there are differences between
// active and passive behaviours, asserts or tests of pCEP->Mode are used
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
//	This routine is called at IRQL_PASSIVE_LEVEL. with ListLock held
//
FSTATUS
GetConnInfo(
	IN CM_CEP_OBJECT*			pCEP,
	OUT CM_CONN_INFO*			pConnInfo
	)
{
	FSTATUS Status=FSUCCESS;
	boolean bKeepQueued = FALSE;	// should additional queued events be kept
	CM_CEP_OBJECT* pCEPEntry = NULL;
	DLIST_ENTRY* pListEntry = NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GetConnInfo);

	pConnInfo->Status = 0;

	// Find out what state
	switch (pCEP->State)
	{
	case CMS_IDLE:
		if (BitTest(pCEP->EventFlags, SYSE_DISCONNECT))
		{
			// CA has been removed
			BitClear(pCEP->EventFlags, SYSE_DISCONNECT);
			if (pCEP->bTimewaitCallback)
				pConnInfo->Status = FCM_CA_REMOVED;
			else
				pConnInfo->Status = FCM_DISCONNECTED;	// legacy API
		}
		else if (BitTest(pCEP->EventFlags, USRE_CANCEL))
		{
			// user has cancelled listen, sidr_req or connect attempt
			BitClear(pCEP->EventFlags, USRE_CANCEL);
			pConnInfo->Status = FCM_CONNECT_CANCEL;
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_REQ))
		{
			// We timed out on the REQ msg i.e. no REP/REJ msg received.
			ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
			BitClear(pCEP->EventFlags, CME_TIMEOUT_REQ);				
			pConnInfo->Status = FCM_CONNECT_TIMEOUT;
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_REP))
		{
			// We timed out while waiting for the RTU to arrive
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			BitClear(pCEP->EventFlags, CME_TIMEOUT_REP);				
			pConnInfo->Status = FCM_CONNECT_TIMEOUT;
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_REJ))
		{
			// a REJ was received
			BitClear(pCEP->EventFlags, CME_RCVD_REJ);

			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));

			pConnInfo->Status = FCM_CONNECT_REJECT;
			bKeepQueued = TRUE; /* for RC_STALE, CME_TIMEOUT_TIMEWAIT queued */
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_DREQ))
		{
			// We timed out on the DREQ msg i.e. no DREP msg received.
			BitClear(pCEP->EventFlags, CME_TIMEOUT_DREQ);				
			// FCM_DISCONNECTED is same value as FCM_DISCONNECT_TIMEOUT
			pConnInfo->Status = FCM_DISCONNECTED;
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_DREP))
		{
			// received a DREP
			BitClear(pCEP->EventFlags, CME_RCVD_DREP);

			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			
			pConnInfo->Status = FCM_DISCONNECT_REPLY;
			bKeepQueued = TRUE; /* leave CME_TIMEOUT_TIMEWAIT queued */
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_DREQ))
		{
			// We received a DREQ but the user already sent a DREP
			// odd sequence, user must have done CmDisconnect with a reply
			// and it raced with a receipt of a DREQ
			BitClear(pCEP->EventFlags, CME_RCVD_DREQ);
			pConnInfo->Status = FCM_DISCONNECTED;
		}
		else if (BitTest(pCEP->EventFlags, CME_SIM_RCVD_DREQ))
		{
			// We received a REJ but the user was already issuing a Disconnect
			// odd sequence, user must have done CmDisconnect with a reply
			// and it raced with a receipt of a REJ
			BitClear(pCEP->EventFlags, CME_SIM_RCVD_DREQ);
			pConnInfo->Status = FCM_DISCONNECTED;
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_TIMEWAIT))
		{
			// Timewait state completed
			BitClear(pCEP->EventFlags, CME_TIMEOUT_TIMEWAIT);
			if (pCEP->bTimewaitCallback)
			{
				pConnInfo->Status = FCM_DISCONNECTED;
			} else {
				Status = FERROR;
			}
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_SIDR_RESP))
		{
			// SIDR_RESP was received
			ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
			BitClear(pCEP->EventFlags, CME_RCVD_SIDR_RESP);

			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));

			if (pConnInfo->Info.SidrResponse.Status == SRS_VALID_QPN)
				pConnInfo->Status = FSIDR_RESPONSE;
			else
				pConnInfo->Status = FSIDR_RESPONSE_ERR;
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_SIDR_REQ))
		{
			// We timed out while waiting for a response to our sidr req
			ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
			BitClear(pCEP->EventFlags, CME_TIMEOUT_SIDR_REQ);
			pConnInfo->Status = FSIDR_REQUEST_TIMEOUT;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_LISTEN:
	case CMS_REQ_RCVD:
	case CMS_MRA_REQ_SENT:
		// We received a client connect or peer connect request.
		ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
		//ASSERT(BitTest(pCEP->EventFlags, CME_RCVD_REQ));
		BitClear(pCEP->EventFlags, CME_RCVD_REQ);

		if (pCEP->State == CMS_LISTEN)
		{
			_DBG_INFO(("<cep 0x%p> *** We received a connect request ***\n",
							_DBG_PTR(pCEP)));
		} else {
			_DBG_INFO(("<cep 0x%p> *** We received a peer request ***\n",
							_DBG_PTR(pCEP)));
		}

		// Get the new CEP object from the pending list
		if (DListIsEmpty(&pCEP->PendingList))
		{
			_DBG_WARNING(("<cep 0x%p> Pending list is empty!\n",
							_DBG_PTR(pCEP)));

			Status = FERROR;
			break;
		}

		pListEntry = DListGetNext(&pCEP->PendingList);
		pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);

		// Make sure this is not the same one we returned previously. This can
		// happen if another request arrived before the user accept
		/*
		if (BitTest(pCEPEntry->EventFlags, USRE_WAIT))
		{
			_DBG_WARNING(("<cep 0x%p> Object is in signaled state but user is still processing the current event!\n",
							_DBG_PTR(pCEP)));

			// The user is still processing the current one. To maintain FIFO order,
			// we cannot notify the user of the new request until the current one is
			// accepted, rejected or timed out.
			Status = FERROR;
			//bWaitAgain = TRUE;
			break;
		}
		
		// Mark it so that we dont return this again in another wait()
		// We clear this in AcceptP() or RejectP()
		//BitSet(pCEPEntry->EventFlags, USRE_WAIT);
		*/
		CopyMadToConnInfo(&pConnInfo->Info,
							(CM_MAD*)GsiDgrmGetSendMad(pCEPEntry->pDgrmElement));

		// port on which REQ arrived
		pConnInfo->Info.Request.CEPInfo.PortGUID = pCEPEntry->pDgrmElement->PortGuid;
		// when we processed REQ, we negotiated these down based on
		// CA capabilities
		pConnInfo->Info.Request.CEPInfo.OfferedInitiatorDepth = pCEPEntry->LocalResponderResources;
		pConnInfo->Info.Request.CEPInfo.OfferedResponderResources = pCEPEntry->LocalInitiatorDepth;
		pConnInfo->Status = FCM_CONNECT_REQUEST;

		break;

	case CMS_REP_RCVD:
	case CMS_MRA_REP_SENT:
		ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
		// We received a reply to our connect request
		if (BitTest(pCEP->EventFlags, CME_RCVD_REP))
		{
			BitClear(pCEP->EventFlags, CME_RCVD_REP);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			pConnInfo->Status = FCM_CONNECT_REPLY;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_ESTABLISHED:
		if (BitTest(pCEP->EventFlags, CME_RCVD_RTU))
		{
			// received the RTU for our REP
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			BitClear(pCEP->EventFlags, CME_RCVD_RTU);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			pConnInfo->Status = FCM_CONNECT_ESTABLISHED;
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_APR))
		{
			ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
			BitClear(pCEP->EventFlags, CME_RCVD_APR);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			if (pConnInfo->Info.AltPathReply.APStatus == APS_PATH_LOADED)
			{
				// we populate CEP with additional information to help
				// application do ModifyQP
				CopyCepPathToPathInfo(&pConnInfo->Info.AltPathReply.u.Alternate.PathInfo,
						&pConnInfo->Info.AltPathReply.u.Alternate.AckTimeout,
						&pCEP->AlternatePath);

				pConnInfo->Info.AltPathReply.u.Alternate.PathInfo.Path.Mtu = pCEP->Mtu;
				pConnInfo->Info.AltPathReply.u.Alternate.PathInfo.Path.P_Key = pCEP->PartitionKey;
				pConnInfo->Status = FCM_ALTPATH_REPLY;
			} else {
				pConnInfo->Status = FCM_ALTPATH_REJECT;
			}
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_LAP))
		{
			ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
			BitClear(pCEP->EventFlags, CME_TIMEOUT_LAP);
			pConnInfo->Status = FCM_ALTPATH_TIMEOUT;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_LAP_RCVD:
	case CMS_MRA_LAP_SENT:
		if (BitTest(pCEP->EventFlags, CME_RCVD_RTU))
		{
			// RTU and LAP events are both queued
			// by the time the signal is delivered to us
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			BitClear(pCEP->EventFlags, CME_RCVD_RTU);
			// We have not yet delivered the RTU, however the pCEP->pDgrmElement
			// is now the LAP, so we deliver an RTU with no private data
			// (much the way a QP Async Event for Communication Established
			// would simulate an RTU)
			// this callback will be followed immediately by the
			// LAP request
			MemoryClear(&pConnInfo->Info.Rtu, sizeof(pConnInfo->Info.Rtu));
			pConnInfo->Status = FCM_CONNECT_ESTABLISHED;
			bKeepQueued = TRUE;	/* leave CME_RCVD_LAP queued */
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_LAP))
		{
			// received LAP
			BitClear(pCEP->EventFlags, CME_RCVD_LAP);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			// additional fields in AltPathInfo
			pConnInfo->Info.AltPathRequest.AlternatePathInfo.Path.P_Key= pCEP->PartitionKey;
			pConnInfo->Info.AltPathRequest.AlternatePathInfo.Path.Mtu	= pCEP->Mtu;
			pConnInfo->Status = FCM_ALTPATH_REQUEST;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_DREQ_RCVD:
		if (BitTest(pCEP->EventFlags,CME_RCVD_RTU))
		{
			// RTU and DREQ events are both queued
			// by the time the signal is delivered to us
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			BitClear(pCEP->EventFlags,CME_RCVD_RTU);
			// We have not yet delivered the RTU, however the pCEP->pDgrmElement
			// is now the DREQ, so we deliver an RTU with no private data
			// (much the way a QP Async Event for Communication Established
			// would simulate an RTU)
			// this callback will be followed immediately by the
			// disconnect request
			MemoryClear(&pConnInfo->Info.Rtu, sizeof(pConnInfo->Info.Rtu));
			pConnInfo->Status = FCM_CONNECT_ESTABLISHED;
			bKeepQueued = TRUE;	/* leave CME_RCVD_DREQ queued */
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_DREQ))
		{
			// Received disconnect request
			BitClear(pCEP->EventFlags, CME_RCVD_DREQ);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			pConnInfo->Status = FCM_DISCONNECT_REQUEST;
		} else {
			Status = FERROR;
		}

		break;

	case CMS_SIM_DREQ_RCVD:
		if (BitTest(pCEP->EventFlags,CME_RCVD_RTU))
		{
			// RTU and SIM DREQ events are both queued
			// by the time the signal is delivered to us
			//
			// Note this is only applicable to passive side, in present code
			// CMS_SIM_DREQ_RCVD could only happen on active side.  However
			// this is implemented for completeness and in case this state
			// is later utilized for a passive side simulated DREQ as well
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			BitClear(pCEP->EventFlags,CME_RCVD_RTU);
			// We have not yet delivered the RTU, however the pCEP->pDgrmElement
			// is now the DREQ, so we deliver an RTU with no private data
			// (much the way a QP Async Event for Communication Established
			// would simulate an RTU)
			// this callback will be followed immediately by the
			// disconnect request
			MemoryClear(&pConnInfo->Info.Rtu, sizeof(pConnInfo->Info.Rtu));
			pConnInfo->Status = FCM_CONNECT_ESTABLISHED;
			bKeepQueued = TRUE;	/* leave CME_SIM_RCVD_DREQ queued */
		}
		else if (BitTest(pCEP->EventFlags, CME_SIM_RCVD_DREQ))
		{
			// We received a REJ but are simulating a DREQ
			MemoryClear(&pConnInfo->Info.DisconnectRequest, sizeof(pConnInfo->Info.DisconnectRequest));
			BitClear(pCEP->EventFlags, CME_SIM_RCVD_DREQ);
			pConnInfo->Status = FCM_DISCONNECT_REQUEST;
		} else {
			Status = FERROR;
		}

		break;

	case CMS_TIMEWAIT:
		if (BitTest(pCEP->EventFlags, CME_RCVD_REJ))
		{
			// a REJ was received (must be RC_STALE_CONN)
			BitClear(pCEP->EventFlags, CME_RCVD_REJ);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			pConnInfo->Status = FCM_CONNECT_REJECT;
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_REP))
		{
			// timeout waiting for RTU
			ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
			if (pCEP->bTimewaitCallback)
			{
				// wait until timewait finishes to provide FCM_CONNECT_TIMEOUT
				Status = FERROR;
				bKeepQueued = TRUE;	/* keep CME_TIMEOUT_REP set till Idle */
			} else {
				BitClear(pCEP->EventFlags, CME_TIMEOUT_REP);
				pConnInfo->Status = FCM_CONNECT_TIMEOUT;
			}
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_DREP))
		{
			// Received disconnect reply
			BitClear(pCEP->EventFlags, CME_RCVD_DREP);
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));
			pConnInfo->Status = FCM_DISCONNECT_REPLY;
		}
		else if (BitTest(pCEP->EventFlags, CME_RCVD_DREQ))
		{
			// We received a DREQ but the user already sent a DREP
			// odd sequence, user must have done CmDisconnect with a reply
			// and it raced with a receipt of a DREQ
			BitClear(pCEP->EventFlags, CME_RCVD_DREQ);
			if (pCEP->bTimewaitCallback)
			{
				// wait until timewait finishes to provide FCM_DISCONNECTED
				Status = FERROR;
			} else {
				pConnInfo->Status = FCM_DISCONNECTED;
			}
		}
		else if (BitTest(pCEP->EventFlags, CME_SIM_RCVD_DREQ))
		{
			// We received a REJ but the user was already issuing a Disconnect
			// odd sequence, user must have done CmDisconnect with a reply
			// and it raced with a receipt of a REJ
			BitClear(pCEP->EventFlags, CME_SIM_RCVD_DREQ);
			if (pCEP->bTimewaitCallback)
			{
				// wait until timewait finishes to provide FCM_DISCONNECTED
				Status = FERROR;
			} else {
				pConnInfo->Status = FCM_DISCONNECTED;
			}
		}
		else if (BitTest(pCEP->EventFlags, CME_TIMEOUT_DREQ))
		{
			// We timed out on the DREQ msg i.e. no DREP msg received.
			BitClear(pCEP->EventFlags, CME_TIMEOUT_DREQ);				
			if (pCEP->bTimewaitCallback)
			{
				// wait until timewait finishes to provide FCM_DISCONNECTED
				Status = FERROR;
			} else {
				// FCM_DISCONNECTED is same value as FCM_DISCONNECT_TIMEOUT
				pConnInfo->Status = FCM_DISCONNECTED;
			}
		} else {
			Status = FERROR;
		}

		break;

	case CMS_REGISTERED:
		ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
		if (BitTest(pCEP->EventFlags, CME_RCVD_SIDR_REQ))
		{
			// Received SIDR request
			BitClear(pCEP->EventFlags, CME_RCVD_SIDR_REQ);

			if (DListIsEmpty(&pCEP->PendingList))
			{
				_DBG_WARNING(("<cep 0x%p> Pending list is empty!\n",
								_DBG_PTR(pCEP)));
				Status = FERROR;
				break;
			}

			pListEntry = DListGetNext(&pCEP->PendingList);
			pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);

			// Make sure this is not the same one we returned previously
			/*
			if (BitTest(pCEPEntry->EventFlags, USRE_WAIT))
			{
				_DBG_WARNING(("<cep 0x%p> Object is in signaled state but user is still processing the current event!\n",
								_DBG_PTR(pCEP)));

				// The user is still processing the current one. To maintain FIFO order,
				// we cannot notify the user of the new request until the current one is
				// accepted, rejected or timed out.
				//bWaitAgain = TRUE;
				Status = FERROR;
				break;
			}

			// Mark it so that we dont return this again in another wait()
			// We clear this in SIDRResponse()
			BitSet(pCEPEntry->EventFlags, USRE_WAIT);
			*/

			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEPEntry->pDgrmElement));
			// Copy additional info to the SidrRequest.
			pConnInfo->Info.SidrRequest.PathInfo.Path.DLID = pCEPEntry->PrimaryPath.RemoteLID;
			pConnInfo->Info.SidrRequest.SID = pCEPEntry->SID;
			pConnInfo->Info.SidrRequest.PortGuid = pCEPEntry->pDgrmElement->PortGuid;
			pConnInfo->Info.SidrRequest.PathInfo.Path.NumbPath = 1; // only 1 path
			pConnInfo->Info.SidrRequest.PathInfo.Path.SLID = pCEPEntry->pDgrmElement->PathBits; // TBD  BUGBUG | local port BaseLID;
			// IB does not have Static rate in UD LRH
			// so we can't be sure what rate of remote port is
			// we use a constant value for GSI QPs, so this is not accurate
			pConnInfo->Info.SidrRequest.PathInfo.Path.Rate = pCEPEntry->pDgrmElement->StaticRate;
			pConnInfo->Info.SidrRequest.PathInfo.Path.u2.s.SL = pCEPEntry->pDgrmElement->ServiceLevel;
			// TBD CopyMadToConnInfo has set Path.Pkey based on SIDR_REQ
			// alternately could lookup pDgrmElement->PkeyIndex

			if (pCEPEntry->pDgrmElement->GlobalRoute)
			{
				pConnInfo->Info.SidrRequest.PathInfo.bSubnetLocal = 0;

				memcpy(&pConnInfo->Info.SidrRequest.PathInfo.Path.DGID.Raw, pCEPEntry->pDgrmElement->GRHInfo.DestGID.Raw, sizeof(IB_GID));
				
				// BUGBUG!
				//memcpy(&pConnInfo->Info.SidrRequest.PathInfo.Path.SGID.Raw, 

				pConnInfo->Info.SidrRequest.PathInfo.Path.SGID.Type.Global.SubnetPrefix = DEFAULT_SUBNET_PREFIX;
				pConnInfo->Info.SidrRequest.PathInfo.Path.SGID.Type.Global.InterfaceID = pCEPEntry->pDgrmElement->PortGuid;

				pConnInfo->Info.SidrRequest.PathInfo.Path.u1.s.FlowLabel = pCEPEntry->pDgrmElement->GRHInfo.FlowLabel;
				pConnInfo->Info.SidrRequest.PathInfo.Path.u1.s.HopLimit = pCEPEntry->pDgrmElement->GRHInfo.HopLimit;
				pConnInfo->Info.SidrRequest.PathInfo.Path.TClass = pCEPEntry->pDgrmElement->GRHInfo.TrafficClass;

			} else {
				pConnInfo->Info.SidrRequest.PathInfo.bSubnetLocal = 1;
			}

			pConnInfo->Status = FSIDR_REQUEST;
		} else {
			Status = FERROR;
		}

		break;

#if 0
	case CMS_SIDR_RESP_RCVD:
		// Received SIDR response
		ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
		if (BitTest(pCEP->EventFlags, CME_RCVD_SIDR_RESP))
		{
			BitClear(pCEP->EventFlags, CME_RCVD_SIDR_RESP);

			// Copy the SIDR response to user buffer
			CopyMadToConnInfo(&pConnInfo->Info,
								(CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement));

			pConnInfo->Status = FSIDR_RESPONSE;
		}
		else
		{
			Status = FERROR;
		}

		break;
#endif
	default:

		Status = FERROR;
		break;
	} // switch()

	// Clear the event flags
	if (! bKeepQueued)
		pCEP->EventFlags = 0;
	if (pCEP->State == CMS_IDLE && pCEP->EventFlags == 0)
		pCEP->Mode = CM_MODE_NONE;

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // GetConnInfo()


//////////////////////////////////////////////////////////////////////////
// WaitA
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
WaitA(
	IN CM_CEP_OBJECT*			pCEP,
	OUT CM_CONN_INFO*			pConnInfo,
	uint32						Timeout_us // TODO: Put a max limit on this in addition to infinite
	)
{
	FSTATUS Status=FSUCCESS;

	uint32 WaitTimeUs=Timeout_us;
	uint64 TimeoutUs=Timeout_us;
	uint64 StartTimeUs=GetTimeStamp();

	uint64 CurrTimeUs=0;
	uint64 TimeElapsedUs=0;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, WaitA);


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
			//_DBG_INFO(("<cep 0x%p> Obj state is nonsignaled.\n",
			//				_DBG_PTR(pCEP)));
		}
		else
		{
			_DBG_WARNING(("<cep 0x%p> EventWaitOn() returns FTIMEOUT!\n",
							_DBG_PTR(pCEP)));
		}

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FTIMEOUT;
	}

	ASSERT(Status == FSUCCESS);

	_DBG_INFO(("<cep 0x%p> Obj state is signaled <%s>.\n", _DBG_PTR(pCEP),
				_DBG_PTR(CmGetStateString(pCEP->State))));

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

} // WaitA()


//////////////////////////////////////////////////////////////////////////
// AcceptA
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
AcceptA(
	IN CM_CEP_OBJECT*		pCEP,
	IN CM_CONN_INFO*		pSendConnInfo		// Send RTU
	)
{
	FSTATUS		Status=FSUCCESS;

	uint64		elapsed_us=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, AcceptA);

	SpinLockAcquire(&gCM->ListLock);

	switch (pCEP->State)
	{
	case CMS_IDLE:

		if (BitTest(pCEP->EventFlags, USRE_CANCEL))
		{
			Status = FCM_CONNECT_CANCEL;
		} else {
			Status = FERROR;
		}

		break;

	case CMS_REP_RCVD:
	case CMS_MRA_REP_SENT:
		//
		// Active accept - 
		// Accepting the connect reply on a connect or peer endpoint. 
		// Send a reply ack msg.
		//

		// Update the turnaround time
		elapsed_us = CmGetElapsedTime(pCEP);
		gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us, elapsed_us);
		_DBG_INFO(("Update turnaround time <tat %"PRIu64"us>.\n", gCM->turnaround_time_us));

		// Prepare RTU
		FormatRTU((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement), 
					pSendConnInfo?pSendConnInfo->Info.Rtu.PrivateData:NULL,
					pSendConnInfo?CMM_RTU_USER_LEN:0,
					pCEP->TransactionID, 
					pCEP->LocalCommID, 
					pCEP->RemoteCommID);

		_DBG_INFO(("<cep 0x%p> RTU's dgrm address field <portguid 0x%" PRIx64", dlid 0x%x, slm %d, pathbits %d, rate %d>\n",
					_DBG_PTR(pCEP), 
					pCEP->pDgrmElement->PortGuid,
					pCEP->pDgrmElement->RemoteLID,
					pCEP->pDgrmElement->ServiceLevel,
					pCEP->pDgrmElement->PathBits,
					pCEP->pDgrmElement->StaticRate));

		// Send out the RTU
		Status = CmDgrmSend(pCEP->pDgrmElement);
		if (Status != FSUCCESS)
		{
			// fall through and let timer retry the send later
			_DBG_WARN(("<cep 0x%p> DgrmSend() failed for RTU!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
			Status = FSUCCESS;
		} else {
			AtomicIncrementVoid(&gCM->Sent.Rtu);
		}

		// Update the status
		Status = FCM_CONNECT_ESTABLISHED;
		if (pSendConnInfo)
			pSendConnInfo->Status = Status;

		CepSetState(pCEP, CMS_ESTABLISHED);
	
		_DBG_INFO(("<cep 0x%p> *** Connection established *** <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x>.\n", 
					 _DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
					 pCEP->PrimaryPath.LocalLID, pCEP->PrimaryPath.RemoteLID));
		break;

	default:
		Status = FINVALID_STATE;
		break;

	} // switch()
	
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // AcceptA()


//////////////////////////////////////////////////////////////////////////
// RejectA
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
RejectA(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_REJECT_INFO*	pConnectReject
	)
{
	FSTATUS	Status=FSUCCESS;
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, RejectA);

	switch (pCEP->State)
	{
	case CMS_IDLE:

		if (BitTest(pCEP->EventFlags, USRE_CANCEL))
		{
			Status = FCM_CONNECT_CANCEL;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_REP_RCVD:
	case CMS_MRA_REP_SENT:
		//
		// Reject the connect reply from the server
		//

		// Format the REJ msg
		FormatREJ((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement),
					CM_REJECT_REPLY,
					(uint16)pConnectReject->Reason,
					pConnectReject->RejectInfo,
					pConnectReject->RejectInfoLen,
					pConnectReject->PrivateData,
					CMM_REJ_USER_LEN,
					pCEP->TransactionID, 
					pCEP->LocalCommID,
					pCEP->RemoteCommID);



		// Send out the REJ. We do not need to handle failures since
		// the other end will eventually time-out if it does not received
		// either RTU or REJ
		Status = CmDgrmSend(pCEP->pDgrmElement);
		if (Status != FSUCCESS)
		{
			// fall through and let timer retry the send later
			_DBG_WARN(("<cep 0x%p> DgrmSend() failed for REJ!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
			Status = FSUCCESS;
		} else {
			AtomicIncrementVoid(&gCM->Sent.RejRep);
		}
	
		_DBG_INFO(("<cep 0x%p> REJ sent, rejecting REP <lcid 0x%x rcid 0x%x slid 0x%x dlid 0x%x reason %d>.\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID, 
					pCEP->PrimaryPath.LocalLID,
					pCEP->PrimaryPath.RemoteLID,
					pConnectReject->Reason));

		CepToIdle(pCEP);
		break;

	//case CMS_REQ_SENT:
	//case CMS_REP_WAIT:
		// TODO: Client canceling the pending connection attempt
		//break;

	default:

		Status = FINVALID_STATE;
		break;

	} // switch()
	
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // RejectA()


//////////////////////////////////////////////////////////////////////////
// CancelA
// CmCancel or CEPDestroy on an active CEP
// CA removal while connecting
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
CancelA(
	IN CM_CEP_OBJECT*			pCEP
	)
{
	FSTATUS	Status=FPENDING;

	switch (pCEP->State)
	{
	case CMS_REQ_SENT:
	case CMS_REP_WAIT:
		//
		// Cancel the connect request
		//

		// Cancel the REQ timer
		CmTimerStop(pCEP, REQ_TIMER);

		if (pCEP->bPeer && pCEP->RemoteEndPoint.CaGUID == 0)
		{
			CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
		}
		CepToIdle(pCEP);

		// Set the cancel event
		SetNotification(pCEP, USRE_CANCEL);

		break;

	case CMS_REP_RCVD:
	case CMS_MRA_REP_SENT:

		// No need to check the return status. We are guaranteed to succeed 
		// since we are calling RejectA() within the lock (i.e. no state changed)
		{
			// keep const and static to save kernel stack space
			static CM_REJECT_INFO	ConnectReject = {
				Reason:RC_USER_REJ
			};

			RejectA(pCEP, &ConnectReject);
		}
		// CEP is now Idle

		// Set the cancel event
		SetNotification(pCEP, USRE_CANCEL);

		break;

	case CMS_SIDR_REQ_SENT:
		
		// Cancel the SIDR_REQ timer
		CmTimerStop(pCEP, SIDR_REQ_TIMER);

		// Move from QUERY to INIT list
		CM_MapRemoveEntry(&gCM->QueryMap, &pCEP->MapEntry, QUERY_LIST, pCEP);
		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEP);
	
		// Set the cancel event
		SetNotification(pCEP, USRE_CANCEL);
		break;

	default:

		Status = FINVALID_STATE;
		break;

	} // switch()
	
	return Status;

} // CancelA()


// build QP Attributes to move QP to Init, RTR and RTS
// and prepare a Rtu
FSTATUS ProcessReplyA(
	IN CM_CEP_OBJECT* 			pCEP,
	OUT CM_PROCESS_REPLY_INFO* Info
	)
{
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessReplyA);

	switch (pCEP->State)
	{
	case CMS_IDLE:
		if (BitTest(pCEP->EventFlags, USRE_CANCEL))
		{
			Status = FCM_CONNECT_CANCEL;
		} else {
			Status = FERROR;
		}
		break;

	case CMS_REP_RCVD:
	case CMS_MRA_REP_SENT:
		MemoryClear(Info, sizeof(*Info));	// just to be safe
		// set up attributes to move QP to Init
		// we used to return Init, but spec says this should be done before
		// send REQ so we removed it to reduce size of return structure
		//GetInitAttrFromCep(pCEP, &Info->QpAttrInit);

		// set up attributes to move QP to RTR
		GetRtrAttrFromCep(pCEP, pCEP->bFailoverSupported, &Info->QpAttrRtr);

		// set up attributes to move QP to RTS
		GetRtsAttrFromCep(pCEP, &Info->QpAttrRts);
		break;

	default:
		Status = FINVALID_STATE;
		break;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}

//////////////////////////////////////////////////////////////////////////
// AltPathRequestA
//
// Request a new alternate path
// Only allowed for Client/Active side of connection
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
//	FPENDING - LAP queued to be sent.
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the lap packet
//	FINVALID_PARAMETER - invalid pLapInfo
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
AltPathRequestA(
	IN CM_CEP_OBJECT* 			pCEP,
	IN const CM_ALTPATH_INFO*	pLapInfo		// Send LAP
	)
{
	FSTATUS		Status;
	IB_PORT_ATTRIBUTES	*pPortAttr=NULL;
	uint8 LocalCaAckDelay;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, AltPathRequestA);

	ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
	if (pCEP->State != CMS_ESTABLISHED)
	{
		Status = FINVALID_STATE;
		goto done;
	}
	if (! pCEP->bFailoverSupported)
	{
		// Remote End REP did not accept Failover
		Status = FINVALID_STATE;
		goto done;
	}
#if ! defined(CM_APM_RELOAD)
	// do not allow alternate path to be reloaded
	if (pCEP->AlternatePath.LocalLID != 0)
	{
		Status = FINVALID_STATE;
		goto done;
	}
#endif
	if (0 == CompareCepPathToPathInfo(&pCEP->PrimaryPath,
					&pLapInfo->AlternatePathInfo))
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = iba_find_ca_gid(&pLapInfo->AlternatePathInfo.Path.SGID,
					pCEP->LocalEndPoint.CaGUID,
					&pCEP->TempAlternateLocalPortGuid,
					&pCEP->TempAlternateLocalGidIndex, &pPortAttr,
					&LocalCaAckDelay);
	if (Status != FSUCCESS)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	if (! LookupPKey(pPortAttr, pCEP->PartitionKey, FALSE, &pCEP->TempAlternatePkeyIndex))
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	pCEP->TempAlternateLocalPathBits = LidToPathBits(pLapInfo->AlternatePathInfo.Path.SLID, pPortAttr->Address.LMC);
	pCEP->TempAlternateLocalAckTimeout = ComputeAckTimeout(
							pLapInfo->AlternatePathInfo.Path.PktLifeTime,
							pCEP->TargetAckDelay);
	MemoryDeallocate(pPortAttr);

	// Prepare LAP
	
	// RTU could still be on send Q, get a fresh Dgrm if needed
	Status = PrepareCepDgrm(pCEP);
	if (Status != FSUCCESS) {
		_DBG_WARNING(("<cep 0x%p> Unable to send LAP: Out of CM Dgrms!!!\n", _DBG_PTR(pCEP)));
		goto done;
	}

	pCEP->bLapSent = 1;	// pCEP->pDgrmElement is no longer an RTU
	FormatLAP((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement), 
					pCEP->RemoteEndPoint.QPN,	// TBD EECN
					pCEP->RemoteCMTimeout,
					pLapInfo,
					(pLapInfo->AlternateAckTimeout
						?pLapInfo->AlternateAckTimeout
						: ComputeAckTimeout(
							pLapInfo->AlternatePathInfo.Path.PktLifeTime,
							LocalCaAckDelay)),
					pCEP->TransactionID, 
					pCEP->LocalCommID, 
					pCEP->RemoteCommID);

	_DBG_INFO(("<cep 0x%p> LAP's dgrm address field <portguid 0x%" PRIx64", dlid 0x%x, slm %d, pathbits %d, rate %d>\n",
					_DBG_PTR(pCEP), 
					pCEP->pDgrmElement->PortGuid,
					pCEP->pDgrmElement->RemoteLID,
					pCEP->pDgrmElement->ServiceLevel,
					pCEP->pDgrmElement->PathBits,
					pCEP->pDgrmElement->StaticRate));

	// Send out the LAP
	Status = CmDgrmSend(pCEP->pDgrmElement);
	if (Status != FSUCCESS)
	{
		// fall through and let timer retry the send later
		_DBG_WARN(("<cep 0x%p> DgrmSend() failed for LAP!!! <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		Status = FSUCCESS;
	} else {
		AtomicIncrementVoid(&gCM->Sent.Lap);
	}

	Status = FPENDING;

	CepSetState(pCEP, CMS_LAP_SENT);
	
	pCEP->RetryCount = 0;
	CmTimerStart(pCEP, TimeoutMultToTimeInMs(pCEP->LocalCMTimeout), LAP_TIMER);

done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // AltPathRequestA()


// build QP Attributes to load new alternate path into QP
FSTATUS ProcessAltPathReplyA(
	IN CM_CEP_OBJECT* 				pCEP,
	IN const CM_ALTPATH_REPLY_INFO*	AltPathReply,
	OUT IB_QP_ATTRIBUTES_MODIFY*	QpAttrRts
	)
{
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessAltPathReplyA);

	ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
	if (pCEP->State != CMS_ESTABLISHED)
	{
		Status = FINVALID_STATE;
		goto done;
	}

	// possible risk - we trust pCEP->AlternatePath, however if the
	// application issues a Lap and gets a reply before this function
	// completes, the pCEP->AlternatePath and pCEP->TempAlternate fields
	// could change on us.  If this risk is real we would need to work
	// strictly from the AltPathReply
	MemoryClear(QpAttrRts, sizeof(*QpAttrRts));	// just to be safe
	QpAttrRts->RequestState = QPStateReadyToSend;
	QpAttrRts->APMState = APMStateRearm;
	GetAVFromCepPath(&pCEP->AlternatePath, &QpAttrRts->AltDestAV);
	QpAttrRts->AltPortGUID = pCEP->AlternatePath.LocalPortGuid;
	QpAttrRts->AltPkeyIndex = pCEP->AlternatePath.PkeyIndex;
	QpAttrRts->Attrs = IB_QP_ATTR_APMSTATE
					| IB_QP_ATTR_ALTDESTAV|IB_QP_ATTR_ALTPORTGUID
					| IB_QP_ATTR_ALTPKEYINDEX;
	if (pCEP->Type != CM_UC_TYPE) {
		QpAttrRts->AltLocalAckTimeout = pCEP->AlternatePath.LocalAckTimeout;
		QpAttrRts->Attrs |= IB_QP_ATTR_ALTLOCALACKTIMEOUT;
	}
	// caller must iba_modify_qp(...)
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}

//////////////////////////////////////////////////////////////////////////
// MigratedA
//
// Inform CM that alternate path has been migrated to by client or server
// TBD - hook into Async Event callbacks
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
//	FSUCCESS - CEP adjusted to reflect migration
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
MigratedA(
	IN CM_CEP_OBJECT*			pCEP
	)
{
	FSTATUS		Status = FSUCCESS;
	uint64		old_acktimeout_us;
	uint64		new_acktimeout_us;
	uint64		timeout_us;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, MigratedA);

	if (pCEP->State != CMS_ESTABLISHED)
	{
		// if CM_APM_RELOAD could be a race of migration with LAP
		Status = FINVALID_STATE;
		goto done;
	}

	if (pCEP->AlternatePath.LocalLID == 0)
	{
		// No alternate path
		Status = FINVALID_STATE;
		goto done;
	}

	// New path will have a different PktLifetime, so adjust our CM timers
	// accordingly, AckTimeout has a 2*PktLifetime component
	old_acktimeout_us = TimeoutMultToTimeInUsec(pCEP->PrimaryPath.AckTimeout);
	new_acktimeout_us = TimeoutMultToTimeInUsec(pCEP->AlternatePath.AckTimeout);

	// LocalCMTimeout has a 2*PktLifetime component
	timeout_us = TimeoutMultToTimeInUsec(pCEP->LocalCMTimeout)
						+ (new_acktimeout_us - old_acktimeout_us);
	pCEP->LocalCMTimeout = TimeoutTimeToMult(timeout_us);

	// RemoteCMTimeout has no PktLifetime component

	pCEP->Timewait = pCEP->AlternatePath.LocalAckTimeout;

	// Ack timeout is calculated as round-trip
	// This computation will be rounded up by CA Ack Delay/2
	if (pCEP->AlternatePath.AckTimeout >= 1)
	{
		// -1 for a log2 value is the same as /2
		pCEP->PktLifeTime = (pCEP->AlternatePath.AckTimeout -1);
	} else {
		pCEP->PktLifeTime = 0;
	}

	pCEP->PrimaryPath = pCEP->AlternatePath;
	MemoryClear(&pCEP->AlternatePath, sizeof(pCEP->AlternatePath));

	// adjust remote path for future CM dgrms to use
	CmDgrmAddrSet(pCEP->pDgrmElement,
				pCEP->PrimaryPath.LocalPortGuid,
				pCEP->PrimaryPath.RemoteLID,
				pCEP->PrimaryPath.SL,
				pCEP->PrimaryPath.LocalPathBits,
				pCEP->PrimaryPath.StaticRate,
				(! pCEP->PrimaryPath.bSubnetLocal),
				pCEP->PrimaryPath.RemoteGID,
				pCEP->PrimaryPath.FlowLabel,
				pCEP->PrimaryPath.LocalGidIndex,
				pCEP->PrimaryPath.HopLimit,
				(uint8)pCEP->PrimaryPath.TClass,
				pCEP->PrimaryPath.PkeyIndex);

done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // MigratedA()

FSTATUS
MigratedReloadA(
	IN CM_CEP_OBJECT* pCEP,
	OUT CM_PATH_INFO*			NewPrimaryPath OPTIONAL,
	OUT CM_ALTPATH_INFO*		AltPathRequest OPTIONAL
	)
{
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, MigratedReloadA);

	ASSERT(pCEP->Mode == CM_MODE_ACTIVE);
	if (NewPrimaryPath)
	{
		CopyCepPathToPathInfo(&NewPrimaryPath->PathInfo,
				&NewPrimaryPath->AckTimeout, &pCEP->AlternatePath);
		NewPrimaryPath->PathInfo.Path.P_Key = pCEP->PartitionKey;
		NewPrimaryPath->PathInfo.Path.Mtu = pCEP->Mtu;
		//NewPrimaryPath->PathInfo.Path.PktLifeTime may be inaccurate
	}
	if (AltPathRequest)
	{
		CopyCepPathToPathInfo(&AltPathRequest->AlternatePathInfo,
				&AltPathRequest->AlternateAckTimeout, &pCEP->PrimaryPath);
		AltPathRequest->AlternatePathInfo.Path.P_Key = pCEP->PartitionKey;
		AltPathRequest->AlternatePathInfo.Path.Mtu = pCEP->Mtu;
		//AltPathRequest->AlternatePathInfo.Path.PktLifeTime may be inaccurate
	}
	Status = MigratedA(pCEP);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;
} // MigratedReloadA()

//////////////////////////////////////////////////////////////////////////
// ShutdownA
// CA has been removed while disconnecting
// used for both Active and Passive CEPs
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
ShutdownA(
	IN CM_CEP_OBJECT*			pCEP
	)
{
	FSTATUS	Status=FPENDING;//FSUCCESS;

	switch (pCEP->State)
	{
	case CMS_DREQ_SENT:
		// Cancel the DREQ timer
		CmTimerStop(pCEP, DREQ_TIMER);
		/* FALLSTHROUGH */
	case CMS_DREQ_RCVD:
	case CMS_SIM_DREQ_RCVD:
		// Skip the CMS_TIMEWAIT, CA is gone
		CepToIdle(pCEP);
			
		// Set the shutdown event
		SetNotification(pCEP, SYSE_DISCONNECT);

		break;


	case CMS_TIMEWAIT:
		// Cancel the TIMEWAIT timer
		CmTimerStop(pCEP, TIMEWAIT_TIMER);

		// Skip the CMS_TIMEWAIT, CA is gone
		CepToIdle(pCEP);
			
		// Is this obj marked for destruction ?
		if (BitTest(pCEP->EventFlags, USRE_DESTROY))
		{
			// handle the case when the user callback returns and this routine
			// gets run because the timewait timer expired.
			// We let the CmSystemCallback() proceed with the destruction.
			if (AtomicRead(&pCEP->CallbackRefCnt) == 0)
			{
				DestroyCEP(pCEP);
			}
		} else {
			// make sure user gets an event to indicate end of timewait period
			if (pCEP->bTimewaitCallback)
			{
				SetNotification(pCEP, SYSE_DISCONNECT);
			}
		}
		Status = FSUCCESS;
		break;

	default:

		Status = FINVALID_STATE;
		break;

	} // switch()
	
	return Status;

} // ShutdownA()


//////////////////////////////////////////////////////////////////////////
// DisconnectA
// User has called CmDisconnect
// used for both Active and Passive CEPs
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
DisconnectA(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_DREQUEST_INFO*	pDRequest,	// Send DREQ
	IN const CM_DREPLY_INFO*	pDReply		// Send DREP
	)
{
	FSTATUS Status=FSUCCESS; // FPENDING ??

	uint32 dreq_timeout_ms=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DisconnectA);


	switch (pCEP->State)
	{
	case CMS_ESTABLISHED:
	case CMS_LAP_SENT:
	case CMS_MRA_LAP_RCVD:
	case CMS_LAP_RCVD:
	case CMS_MRA_LAP_SENT:
		
		if (pDRequest)
		{
			//pDRequest->Reserved3 = pCEP->LocalEndPoint.QPN;

			// an RTU or LAP/APR could be on the SendQ
			// RTU  -->
			// DREQ -->
			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS) {
				_DBG_WARNING(("<cep 0x%p> Unable to send DREQ: Out of CM Dgrms!!!\n", _DBG_PTR(pCEP)));
				break;
			}

			// Format the DREQ msg
			FormatDREQ((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement),
						pCEP->RemoteEndPoint.QPN,
						pDRequest->PrivateData,
						CMM_DREQ_USER_LEN,
						pCEP->TransactionID, 
						pCEP->LocalCommID,
						pCEP->RemoteCommID);

			// Send out the DREQ
			Status = CmDgrmSend(pCEP->pDgrmElement);
			if (Status != FSUCCESS)
			{
				// fall through and let timer retry the send later
				_DBG_WARN(("<cep 0x%p> DgrmSend() failed for DREQ!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
				Status = FSUCCESS;
			} else {
				AtomicIncrementVoid(&gCM->Sent.Dreq);
			}
	
			//Status = FPENDING;
			if (pCEP->State == CMS_LAP_SENT
				|| pCEP->State == CMS_MRA_LAP_RCVD)
			{
				CmTimerStop(pCEP, REQ_TIMER);
			}

			CepToDisconnecting(pCEP, CMS_DREQ_SENT);

			pCEP->RetryCount = 0;
			dreq_timeout_ms = TimeoutMultToTimeInMs(pCEP->LocalCMTimeout);
			CmTimerStart(pCEP, dreq_timeout_ms, DREQ_TIMER);
		}
		else
		{
			_DBG_ERROR(("<cep 0x%p> Invalid state!!! <%s>\n", _DBG_PTR(pCEP),
						_DBG_PTR(CmGetStateString(pCEP->State))));

			Status = FINVALID_STATE;
		}

		break;

	case CMS_DREQ_RCVD:

		if (pDReply)
		{
			// uncomment out these 3 lines to have DREQ processing time
			// factored into average turnaround time
			//uint64 elapsed_us;
			//elapsed_us = CmGetElapsedTime(pCEP);
			//gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us, elapsed_us);

			// just in case a LAP/APR or DREQ is in Send Q
			Status = PrepareCepDgrm(pCEP);
			if (Status != FSUCCESS) {
				_DBG_WARNING(("<cep 0x%p> Unable to send DREP: Out of CM Dgrms!!!\n", _DBG_PTR(pCEP)));
				break;
			}

			// Format the DREP msg
			FormatDREP((CM_MAD*)GsiDgrmGetSendMad(pCEP->pDgrmElement),
						pDReply->PrivateData,
						CMM_DREP_USER_LEN,
						pCEP->TransactionID, 
						pCEP->LocalCommID,
						pCEP->RemoteCommID);

			// Send out the DREP
			Status = CmDgrmSend(pCEP->pDgrmElement);
			if (Status != FSUCCESS)
			{
				// fall through when remote end retries, we will resend DREP
				_DBG_WARN(("<cep 0x%p> DgrmSend() failed for DREP!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
				Status = FSUCCESS;
			} else {
				AtomicIncrementVoid(&gCM->Sent.Drep);
			}

			CepToTimewait(pCEP);
		} else {
			_DBG_INFO(("<cep 0x%p> Already disconnecting!!! <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));
	
			Status = FCM_ALREADY_DISCONNECTING;
		}

		break;

	case CMS_SIM_DREQ_RCVD:
		// to handle the case of a REJ coming in after we sent RTU (possibly
		// because server never got our RTU), we simulated an inbound DREQ and
		// sent up a DISCONNECT_REQUEST callback.
		// As a result the application will call CmDisconnect,
		// but we don't want to put a DREP on the wire here.
		// Instead we simply move to the timewait state.
		// This permits the incoming REJ to simulate a DREQ API sequence.

		if (pDReply)
		{
			// uncomment out these 3 lines to have DREQ processing time
			// factored into average turnaround time
			//uint64 elapsed_us;
			//elapsed_us = CmGetElapsedTime(pCEP);
			//gCM->turnaround_time_us = UpdateTurnaroundTime(gCM->turnaround_time_us, elapsed_us);

			CepToTimewait(pCEP);
			Status = FSUCCESS;
		} else {
			_DBG_INFO(("<cep 0x%p> Already disconnecting!!! <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));
	
			Status = FCM_ALREADY_DISCONNECTING;
		}
		break;

	default:

		_DBG_ERROR(("<cep 0x%p> Invalid state!!! <lcid 0x%x %s>.\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID,
					_DBG_PTR(CmGetStateString(pCEP->State))));

		Status = FINVALID_STATE;
		break;

	} // switch()

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // DisconnectA()


// EOF
