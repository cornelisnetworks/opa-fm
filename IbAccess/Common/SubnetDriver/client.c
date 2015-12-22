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
#include <sdi.h>
#include <subscribe.h>


uint32 LastClientId;
CLIENT_CONTROL_PARAMETERS DefaultClientParameters;

QUICK_LIST *pClientList = NULL;
SPIN_LOCK *pClientListLock = NULL;

// Initialize the default parameters that clients get
void
SetDefaultClientParameters(
	void
	)
{
    DefaultClientParameters.ControlParameters.RetryCount = g_SdParams.DefaultRetryCount;
    DefaultClientParameters.ControlParameters.Timeout =  g_SdParams.DefaultRetryInterval;
    DefaultClientParameters.ClientDebugFlags = g_SdParams.DefaultDebugFlags;
    DefaultClientParameters.OptionFlags = g_SdParams.DefaultOptionFlags;

	SpinLockAcquire(pClientListLock);
    LastClientId = 0;
    SpinLockRelease(pClientListLock);
}



FSTATUS
iba_sd_register(
    IN OUT CLIENT_HANDLE *ClientHandle,
    IN PCLIENT_CONTROL_PARAMETERS pClientParameters OPTIONAL
    )
{
    ClientListEntry *pClientEntry;
    
    //
    // Note: If the caller is running at low priority, the input pointers for
    // the client ID and the client parameters can be pageable. If the caller
    // is running at a high priority, it is the caller's responsibility to 
    // make sure that the pointers point to non-pageable memory.
    //

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_register);

    pClientEntry = (ClientListEntry*)MemoryAllocate2AndClear(sizeof(ClientListEntry), IBA_MEM_FLAG_NONE,
        SUBNET_DRIVER_TAG);
    if (pClientEntry == NULL)  
	{
        _DBG_ERROR(
            ("Cannot allocate memory for new client entry\n"));
        return(FINSUFFICIENT_MEMORY);
    }
	pClientEntry->ListItem.pObject = (void *)pClientEntry;
    if (pClientParameters == NULL)  
	{
		_DBG_INFO(("ClientParameters is NULL, Setting Driver Defaults\n"));
        MemoryCopy(&pClientEntry->ClientParameters, &DefaultClientParameters,
            sizeof(pClientEntry->ClientParameters));
    } else { 
        MemoryCopy(&pClientEntry->ClientParameters, pClientParameters,
            sizeof(pClientEntry->ClientParameters));
    }

    QListInit(&pClientEntry->ClientTraps);

    SpinLockAcquire(pClientListLock);
    pClientEntry->ClientId = LastClientId++;
    // 
    // Note: We are running at the priority level of the caller. If the caller
    // is running at low priority, the memory containing input client parameters
    // could be pageable and may not be touched at high priority with a spin
    // lock held. We therefor copy parameters to (non-pageable) stack location
    // before raising priority by getting a lock
    //
    
    QListInsertHead(pClientList, &pClientEntry->ListItem);
    SpinLockRelease(pClientListLock);

    //
    // Now we are back at the caller's priority, which could be low. In that 
    // case, it is now safe to touch the caller's potentially pageable memory.
    //
    //*pCid = Cid;
	*ClientHandle = (CLIENT_HANDLE)pClientEntry;

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
    return(FSUCCESS);

}

FSTATUS
iba_sd_deregister(
    IN CLIENT_HANDLE ClientHandle
    )
{
	return CommonSdDeregister(ClientHandle);
}

FSTATUS
CommonSdDeregister(
    IN CLIENT_HANDLE ClientHandle
    )
{
    ClientListEntry *pClientEntry = (ClientListEntry *)ClientHandle;
    FSTATUS Fstatus = FNOT_FOUND;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_deregister);

    SpinLockAcquire(pClientListLock);
	// &pClientEntry->ListItem does not dereference pClientEntry
	if(QListIsItemInList(pClientList, &pClientEntry->ListItem) == TRUE)
	{
		QListRemoveItem(pClientList, &pClientEntry->ListItem);
		Fstatus = FSUCCESS;
		_DBG_INFO(("Client Entry removal passed\n"));
	} else {
		Fstatus = FNOT_FOUND;
		_DBG_INFO(("Client Entry removal failed\n"));
	}
    SpinLockRelease(pClientListLock);
	
	// Clean up CompletionList and QueryList for this clienthandle
	if(Fstatus == FSUCCESS)
	{
		CancelClientQueries(ClientHandle);
		UnsubscribeClient(pClientEntry);
		MemoryDeallocate(pClientEntry);
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
    return(Fstatus);
}

static boolean
CompareClientHandle(
	IN LIST_ITEM* pListItem,
	IN void* Context)
{
	return (((PQueryDetails)QListObj(pListItem))->ClientHandle == (CLIENT_HANDLE)Context);
}

// cancel all queries in progress for client
// if we remove them from the pQueryList, any subsequent responses will
// be discarded, same as if the query had timed out
// if we cancel just before completions are reported, we could find some
// on the completion list for this client, so we clean them up too
//
// Note that child queries are never associated to a ClientHandle.
// Hence we will not be cancelling any child queries here.
// By cancelling the parent, the child will be discarded when it comes to
// the head of the pChildQueryList or when its response arrives.
void
CancelClientQueries(
	IN CLIENT_HANDLE ClientHandle
	)
{
	QueryDetails *pQueryElement;
	boolean schedule = FALSE;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CancelClientQueries);

	// first process the Query List
	SpinLockAcquire(&pQueryList->m_Lock);
	while (NULL != (pQueryElement = (QueryDetails*)QListFindFromHead(
											&pQueryList->m_List,
											CompareClientHandle, ClientHandle)))
	{
		QListRemoveItem(&pQueryList->m_List, &pQueryElement->QueryListItem);
		DEBUG_ASSERT( pQueryElement->QState != QueryDestroy);
		DEBUG_ASSERT( pQueryElement->pParentQuery == NULL);	// not a child
		pQueryElement->ClientHandle = NULL;	// disassociate from client
		if( pQueryElement->QState == ProcessingResponse
			|| pQueryElement->ChildProcessingResponseCount)
		{
			DEBUG_ASSERT(pQueryElement->QState == WaitingForChildToComplete
					|| ! pQueryElement->ChildProcessingResponseCount);
			// RecvHandler is still holding onto pQueryElement
			// mark state so RecvHandler will destroy it, leave off list
			pQueryElement->QState = QueryDestroy;
			SpinLockRelease(&pQueryList->m_Lock);
		} else {
			// no one else is using this element, we can unlock and change it
			SpinLockRelease(&pQueryList->m_Lock);
			if( pQueryElement->QState == WaitingForResult
				|| pQueryElement->QState == BusyRetryDelay)
				AtomicDecrementVoid(&Outstanding);
			// so we don't prempt here, defer destroy to premptable context
			pQueryElement->QState = QueryDestroy;
			LQListInsertHead(pCompletionList, &pQueryElement->QueryListItem);
			schedule = TRUE;
		}
		SpinLockAcquire(&pQueryList->m_Lock);
	}
	SpinLockRelease(&pQueryList->m_Lock);

	// now process the Completion List
	SpinLockAcquire(&pCompletionList->m_Lock);
	while (NULL != (pQueryElement = (QueryDetails*)QListFindFromHead(
											&pCompletionList->m_List,
											CompareClientHandle, ClientHandle)))
	{
		// any items in QueryDestroy state should no longer be associated to
		// a client. so we can assert that we are not in QueryDestroy state
		DEBUG_ASSERT(pQueryElement->QState == QueryComplete);
		DEBUG_ASSERT( pQueryElement->pParentQuery == NULL);	// not a child
		pQueryElement->ClientHandle = NULL;	// disassociate from client
		// so we don't prempt here, defer destroy to premptable context
		// can leave on list
		pQueryElement->QState = QueryDestroy;
		SpinLockRelease(&pCompletionList->m_Lock);
		schedule = TRUE;
		SpinLockAcquire(&pCompletionList->m_Lock);
	}
	SpinLockRelease(&pCompletionList->m_Lock);

	// completion processing will handle defered destroys
	if (schedule)
		ScheduleCompletionProcessing();
}

FSTATUS
iba_sd_get_client_control_parameters(
    IN CLIENT_HANDLE ClientHandle,
    IN OUT PCLIENT_CONTROL_PARAMETERS pClientControlParameters
    )
{
    FSTATUS Fstatus = FNOT_FOUND;
    CLIENT_CONTROL_PARAMETERS ClientParams;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_get_client_control_parameters);

    if (pClientControlParameters == NULL) {
		_DBG_INFO(("Invalid ClientControlParameters\n"));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
        return(FINVALID_PARAMETER);
    }
	
    // Note: If the caller is running at low priority, the input memory for
    // the client parameters can be pageable. If the caller is running at a 
    // high priority, it is the caller's responsibility to make sure that the
    // memory is non-pageable.
	Fstatus = ValidateClientHandle(ClientHandle, &ClientParams);

    // Now we are back to the caller's priority level and can safely touch
    // the memory to which the results have to be written.
    if (Fstatus == FSUCCESS)  
	{
        MemoryCopy(pClientControlParameters, &ClientParams,
            sizeof(ClientParams));
    }
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
    return(Fstatus);
}

FSTATUS
iba_sd_set_client_control_parameters(
    IN CLIENT_HANDLE ClientHandle,
    IN PCLIENT_CONTROL_PARAMETERS pClientControlParameters
    )
{
    FSTATUS Fstatus = FNOT_FOUND;
    CLIENT_CONTROL_PARAMETERS ClientParams;
	LIST_ITEM *pListItem = &((ClientListEntry *)ClientHandle)->ListItem;

    _DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_set_client_control_parameters);

    if (pClientControlParameters == NULL) 
	{
		_DBG_INFO(("Invalid ClientControlParameters\n"));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
        return(FINVALID_PARAMETER);
    }
    // Note: If the caller is running at low priority, the input memory for
    // the client parameters can be pageable. If the caller is running at a 
    // high priority, it is the caller's responsibility to make sure that the
    // memory is non-pageable.
    //
    // Note: We are running at caller's priority, which may be low. In that 
    // case, the caller could have supplied us a pointer to pageable memory that
    // cannot be touched at high priority with the spin lock held. We therefore
    // copy the parameters first to a local (non-pageable) stack location.
    MemoryCopy(&ClientParams, pClientControlParameters,
        			sizeof(ClientParams));

    SpinLockAcquire(pClientListLock);
	if (QListIsItemInList(pClientList, pListItem))
	{
    	ClientListEntry *pClientEntry = (ClientListEntry*)ClientHandle;
		Fstatus = FSUCCESS;
		MemoryCopy(&(pClientEntry->ClientParameters), &ClientParams,
                	sizeof(pClientEntry->ClientParameters));
    }
    SpinLockRelease(pClientListLock);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
    return(Fstatus);
}
