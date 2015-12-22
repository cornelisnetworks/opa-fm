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


#include <datatypes.h>
#include <imemory.h>
#include <ilist.h>
#include <ievent.h>
#include <stl_sd.h>
#include <stl_helper.h>
#include <sdi.h>

// only add to list if FirstAttempt
// only get lock while adding to list if NeedLock
static void
QueueQuery(
    PQueryDetails pQueryElement,
	boolean FirstAttempt,
	boolean NeedLock
	)
{
	if (FirstAttempt)
	{
		pQueryElement->RetriesLeft = pQueryElement->ControlParameters.RetryCount;
		if (NeedLock)
		{
			LQListInsertTail(pQueryList, &pQueryElement->QueryListItem);
		} else {
			QListInsertTail(&pQueryList->m_List, &pQueryElement->QueryListItem);
		}
	}
}


// when called with FirstAttempt == FALSE, pQueryList->m_lock is held
// when called with FirstAttempt == TRUE, pQueryList->m_lock is not held
FSTATUS
SendQueryElement(
    PQueryDetails pQueryElement,
	boolean FirstAttempt
    )
{
	uint32 ElementCount;
	IBT_DGRM_ELEMENT *pIbtDgrmElement;
	FSTATUS Status;
	CA_PORT *pCaPort;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SendQueryElement);
	
	ASSERT(pQueryElement);

	if (AtomicIncrement(&Outstanding) > g_SdParams.MaxOutstanding)
	{
		AtomicDecrementVoid(&Outstanding);
		_DBG_INFO(("Max outstanding of %u reached, queuing query element %p\n",
						g_SdParams.MaxOutstanding, _DBG_PTR(pQueryElement)));
		// we will try again later
		pQueryElement->QState = ReadyToSend;
		QueueQuery(pQueryElement, FirstAttempt, FirstAttempt);
		goto done;
	}

	ElementCount = 1;
			
	Status = iba_gsi_dgrm_pool_get(sa_poolhandle, &ElementCount, &pIbtDgrmElement);
#ifdef SA_ALLOC_DEBUG
	if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50)
	{
		MsgOut("Dgrm Alloc Status %d\n", Status);
		MsgOut("SA Dgrm Pool=%u\n", iba_gsi_dgrm_pool_count(sa_poolhandle));
	}
#endif
	if (Status != FSUCCESS) 
	{
		_DBG_WARN(("Failed to allocate SA DGRM, Status = <%d>, exitting\n", Status));
		// we will try again later
		AtomicDecrementVoid(&Outstanding);
		pQueryElement->QState = ReadyToSend;
		QueueQuery(pQueryElement, FirstAttempt, FirstAttempt);
		goto done;
	}
	
	pQueryElement->CmdSentTime = GetTimeStamp();
	pIbtDgrmElement->Element.pContext = (void *)&pQueryElement->QueryListItem;
	
	SpinLockAcquire(pCaPortListLock);
	pCaPort = GetCaPort(pQueryElement->PortGuid);
	if (pCaPort == NULL)
	{
		SpinLockRelease(pCaPortListLock);
		_DBG_ERROR(("GetCaPort returned NULL for Port (GUID: %"PRIx64").\n", pQueryElement->PortGuid));
		// TBD we should return an error, however callers are not
		// prepared, so we queue for retries and timeout
		pQueryElement->QState = NotAbleToSend;
		QueueQuery(pQueryElement, FirstAttempt, FirstAttempt);
		goto cantsend;
	}

	// TBD - enable this code when ClassPortInfo works in SM
#if 0
	// this is necessary to handle a redirected SA
	if((pQueryElement->IsSelfCommand) && 
		(pQueryElement->SdSentCmd == GetClassportInfo) ) 
	{
		pIbtDgrmElement->RemoteLID = pCaPort->Lid;
		pIbtDgrmElement->ServiceLevel= pCaPort->ServiceLevel;
		pIbtDgrmElement->StaticRate = pCaPort->StaticRate;
		pIbtDgrmElement->RemoteQP = pCaPort->RemoteQp;
		pIbtDgrmElement->RemoteQKey = pCaPort->RemoteQKey;
		pIbtDgrmElement->PkeyIndex = pCaPort->PkeyIndex;
	} else {
		pIbtDgrmElement->RemoteLID = pCaPort->RedirectedLid;
		pIbtDgrmElement->ServiceLevel= pCaPort->RedirectedSL;
		pIbtDgrmElement->StaticRate = pCaPort->RedirectedStaticRate;
		pIbtDgrmElement->RemoteQP = pCaPort->RedirectedQP;
		pIbtDgrmElement->RemoteQKey= pCaPort->RedirectedQKey;
		pIbtDgrmElement->PkeyIndex = pCaPort->RedirectedPkeyIndex;
	}
#else
	pIbtDgrmElement->RemoteLID = pCaPort->Lid;
	pIbtDgrmElement->ServiceLevel = pCaPort->ServiceLevel;
	pIbtDgrmElement->StaticRate = pCaPort->StaticRate;
	pIbtDgrmElement->RemoteQP = pCaPort->RemoteQp;
	pIbtDgrmElement->RemoteQKey = pCaPort->RemoteQKey;
//	pIbtDgrmElement->PkeyIndex = pCaPort->PkeyIndex;
	pIbtDgrmElement->PkeyIndex = pCaPort->DefaultPkeyIndex;
#endif
	pIbtDgrmElement->PathBits = 0;
	pIbtDgrmElement->PortGuid = pQueryElement->PortGuid;
		
	MemoryCopy( GsiDgrmGetSendMad(pIbtDgrmElement),
		pQueryElement->u.pGmp, pQueryElement->TotalBytesInGmp );

	pIbtDgrmElement->Element.pBufferList->ByteCount = pQueryElement->TotalBytesInGmp;

	if(pCaPort->SdSMAddressValid == TRUE) //Send only if the we have atleast one port active
	{
		SpinLockRelease(pCaPortListLock);
		pQueryElement->QState = WaitingForResult;
		// probably not needed, but to be safe,
		// hold lock during post_send in case it fails
		// once we add to list, others (such as iba_sd_deregister) could
		// remove from list or change its state
		if (FirstAttempt)
			SpinLockAcquire(&pQueryList->m_Lock);
		QueueQuery(pQueryElement, FirstAttempt, FALSE);
		Status =iba_gsi_post_send( GsiSAHandle, pIbtDgrmElement);
#ifdef SA_ALLOC_DEBUG
		if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("Dgrm send Status %d\n", Status);
#endif
		if (Status != FSUCCESS)
		{
			pQueryElement->QState = NotAbleToSend;
			_DBG_ERROR(("iba_gsi_post_send failed, Status = %d\n", Status));
			if (FirstAttempt)
				SpinLockRelease(&pQueryList->m_Lock);
			goto cantsend;
		}
		if (FirstAttempt)
			SpinLockRelease(&pQueryList->m_Lock);
	} else {
		SpinLockRelease(pCaPortListLock);
		pQueryElement->QState = NotAbleToSend;
		QueueQuery(pQueryElement, FirstAttempt, FirstAttempt);
		_DBG_INFO(("Unable to Send, Sd SM Address not valid: Port 0x%016"PRIx64"\n", pQueryElement->PortGuid));
		goto cantsend;
	}

	_DBG_INFO(("Sent Query Element %p: %u Outstanding\n",
			_DBG_PTR(pQueryElement),
			AtomicRead(&Outstanding)));
done:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return FSUCCESS;

cantsend:
#ifdef SA_ALLOC_DEBUG
	if (iba_gsi_dgrm_pool_count(sa_poolhandle) < 50) MsgOut("Dgrm cant send\n");
#endif
	// unable to send for now, cleanup, timer will retry later
	AtomicDecrementVoid(&Outstanding);
	iba_gsi_dgrm_pool_put( pIbtDgrmElement);
	goto done;
}
