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
#include <query.h>
#include <isyscallback.h>
#include <itimer.h>
#include <ithread.h>
#include <imutex.h>
#include <subscribe.h>
#include <multicast.h>

//Refer this macro only in one c file
#ifdef ICS_LOGGING
_IB_DBG_PARAM_BLOCK(_DBG_LVL_ALL,_DBG_BREAK_ENABLE,SUBNET_DRIVER_TAG,"SubnetDrv",MOD_SDI,Sdi);
#else
_IB_DBG_PARAM_BLOCK(_DBG_LVL_ALL,_DBG_BREAK_ENABLE,SUBNET_DRIVER_TAG,"SubnetDrv");
#endif
SYS_CALLBACK_ITEM * pCallbackItemObject = NULL;
TIMER * pTimerObject = NULL;
ATOMIC_UINT IsTimerStarted;
ATOMIC_UINT SdWorkItemQueued;
ATOMIC_UINT SdWorkItemsInProcess;
boolean SdUnloading = FALSE;

SD_CONFIG_PARAMS g_SdParams = SD_DEFAULT_CONFIG_PARAMS;

FSTATUS
SdCommonCreate(
    void
    )
{
    //
    // This is where we would put common code that needs to execute when we
    // get a device open request. Currently we have nothing to do here.
    //
    return(FSUCCESS);
}

FSTATUS
SdCommonClose(
    void
    )
{
    //
    // This is where we would put common code that needs to execute when we
    // get a device close request. Currently we have nothing to do here.
    //
    return(FSUCCESS);
}

FSTATUS
InitializeDriver(
    void
    )
{
    FSTATUS Fstatus;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, InitializeDriver);

    // Read the global programmable driver parameters
	(void)ReadDriverTunables();

	_TRC_REGISTER();

	// Initialize the work item queue required to 
	// be called in the timer handler function
	pCallbackItemObject = InitializeWorkItemqueue();
	if(pCallbackItemObject == NULL)
	{
		_DBG_ERROR(("Could not initialize WorkItemqueue\n"));
		_DBG_BREAK;
		
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_RESOURCES);
	}

	// Initialize the timer handlers
	pTimerObject = (TIMER*)MemoryAllocateAndClear(sizeof(TIMER),FALSE,SUBNET_DRIVER_TAG);
	if(pTimerObject == NULL) 
	{
		_DBG_ERROR(("Memory Allocation for Timer Object Failed\n"));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
		return(FINSUFFICIENT_RESOURCES);
	}

	TimerInitState(pTimerObject);
	
	if (TimerInit(pTimerObject, TimerHandler, NULL) != TRUE) 
	{ 
		_DBG_ERROR(("TimerInit failed \n"));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_RESOURCES);
	}

		
    // Initialize the IB Transport components that we need
    Fstatus = InitializeIbt();

    if (Fstatus != FSUCCESS)  
	{
        _DBG_ERROR(("FStatus <0x%x> initializing IB Transport\n", Fstatus));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
        return(Fstatus);
    }

    // Initialize some data structures
    // pQueryList, pCompletionList, pClientList, pCaPortList
	pQueryList = LQListAllocateAndInit(FALSE, SUBNET_DRIVER_TAG);
	if (! pQueryList)  
	{
		_DBG_ERROR(("Unable to initialize client list\n"));
        _DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_MEMORY);
    }
    
	pCompletionList = LQListAllocateAndInit(FALSE, SUBNET_DRIVER_TAG);
    
	if (! pCompletionList)
	{
		_DBG_ERROR(("Unable to initialize Completion list\n"));
        _DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_MEMORY);
    }
    
	Fstatus = InitializeList(&pClientList,
				&pClientListLock);
    
	if (Fstatus != FSUCCESS)  
	{
        _DBG_ERROR(("Status 0x%x initializing client list\n", Fstatus));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
        return(Fstatus);
    }

	pChildQueryList = LQListAllocateAndInit(FALSE, SUBNET_DRIVER_TAG);
    
	if (! pChildQueryList)
	{
		_DBG_ERROR(("Unable to initialize ChildQueryList\n"));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_MEMORY);
    }

	pChildGrowParentMutex = (MUTEX*)MemoryAllocateAndClear(sizeof(MUTEX),FALSE,SUBNET_DRIVER_TAG);
	if(pChildGrowParentMutex == NULL) 
	{
		_DBG_ERROR(("Memory Allocation for Mutex Failed\n"));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);		
		return(FINSUFFICIENT_RESOURCES);
	}

	MutexInitState(pChildGrowParentMutex);
	
	if (MutexInit(pChildGrowParentMutex) != FSUCCESS) 
	{ 
		_DBG_ERROR(("MutexInit failed \n"));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_RESOURCES);
	}

	Fstatus = InitializeList(&pCaPortList, &pCaPortListLock);

	if (Fstatus != FSUCCESS)
	{
        _DBG_ERROR(("Status 0x%x initializing CaPortList\n", Fstatus));
		_DBG_BREAK;
		UnloadDriver();

		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
        return(Fstatus);
    }

	SetDefaultClientParameters();

	// Allocate and Initialize pLocalCaInfoLock SPIN_LOCK Structure
	// and also register for a port change event callback
	pLocalCaInfoLock = (SPIN_LOCK*)MemoryAllocateAndClear(sizeof(SPIN_LOCK),
		FALSE, SUBNET_DRIVER_TAG);
	
	if(pLocalCaInfoLock == NULL) 
	{
        _DBG_ERROR(("Memory Allocation failed for pLocalCaInfoLock\n"));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return (FINSUFFICIENT_MEMORY);
	}
		
	SpinLockInitState(pLocalCaInfoLock);

    if (SpinLockInit(pLocalCaInfoLock)!= TRUE)  
	{
		_DBG_ERROR(("Cannot initialize local CA info lock\n"));
		_DBG_BREAK;
		UnloadDriver();
		
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return (FINSUFFICIENT_MEMORY);
	}
	
	AtomicWrite(&Outstanding, 0);

	SubscribeInitialize();
#ifdef USE_SD_MULTICAST 
	Fstatus = MulticastInitialize();
	if (Fstatus != FSUCCESS)
	{
		_DBG_ERROR(("Fstatus <0x%x> MulticastInitialize\n", Fstatus));
		_DBG_BREAK;
		UnloadDriver();

		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(Fstatus);
	}
#endif
	//
    // Register with the IB transport to get notifications when there is
    // a local CA event (e.g. new link inserted, new CA inserted etc.).
    // NOTENOTE1:Chech which typecase is required
	//
    
	Fstatus = iba_register_notify(LocalCaChangeCallback,
		NULL, IB_NOTIFY_ANY|IB_NOTIFY_ON_REGISTER, IB_NOTIFY_SYNC, &CaNotifyHandle);
          
	if (Fstatus != FSUCCESS)  
	{
		_DBG_ERROR(("Fstatus <0x%x> RegisterNotify\n", Fstatus));
		_DBG_BREAK;
		UnloadDriver();
	
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(Fstatus);
	
    }
    
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);    
	return(FSUCCESS);
}


FSTATUS
UnloadDriver(
    void
    )
{
    _DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, UnloadDriver);

	_TRC_UNREGISTER();

#ifdef USE_SD_MULTICAST    
	MulticastTerminate();
#endif
	SubscribeTerminate();
	
	// DeregisterNotify Handle
	if (CaNotifyHandle) 
	{
		iba_deregister_notify(CaNotifyHandle);
		CaNotifyHandle = NULL;
	}
	
	SdUnloading = TRUE;	// prevent new queries

	// wait for any sys callbacks or outstanding queries to complete/timeout
	while (! LQListIsEmpty(pQueryList) || ! LQListIsEmpty(pCompletionList)
			|| ! LQListIsEmpty(pChildQueryList) || AtomicRead(&SdWorkItemQueued)
			|| AtomicRead(&SdWorkItemsInProcess))
		ThreadSuspend(10);

	// Destroy Timer Object
	if(pTimerObject) 
	{
		TimerDestroy(pTimerObject);
		MemoryDeallocate(pTimerObject);
		pTimerObject = NULL;
	}

	if(pCallbackItemObject ) 
	{
		SysCallbackPut(pCallbackItemObject );
		pCallbackItemObject = NULL;
	}

	// Destroy the Datagram pool and deregister with GSI
	DeRegisterWithGSI();

	// DeInit Lists and destroy
	LQListDeallocate(pQueryList);
	pQueryList = NULL;

	LQListDeallocate(pCompletionList);
	pCompletionList = NULL;

	DestroyList(pClientList,pClientListLock);
	pClientList = NULL;
	pClientListLock = NULL;

	LQListDeallocate(pChildQueryList);
	pChildQueryList = NULL;

	DestroyList(pCaPortList,pCaPortListLock);
	pCaPortList = NULL;
	pCaPortListLock = NULL;

	if(pChildGrowParentMutex) 
	{
		MutexDestroy(pChildGrowParentMutex);
		MemoryDeallocate(pChildGrowParentMutex);
		pChildGrowParentMutex = NULL;
	}

	// Deallocate global memory and destroy spinlocks
	if(pLocalCaInfo) 
	{
		uint32 count;
		for(count=0; count<NumLocalCAs; count++)
		{
			if(pLocalCaInfo[count].PortAttributesList)
				MemoryDeallocate(pLocalCaInfo[count].PortAttributesList);
		}
		MemoryDeallocate(pLocalCaInfo);
		pLocalCaInfo = NULL;
	}

	if(pLocalCaGuidList)
	{
		MemoryDeallocate(pLocalCaGuidList);
		pLocalCaGuidList = NULL;
	}

	if(pLocalCaInfoLock) 
	{
		SpinLockDestroy(pLocalCaInfoLock);
		MemoryDeallocate(pLocalCaInfoLock);
        pLocalCaInfoLock = NULL;
	}

	NumLocalCAs = 0;

	
	 _DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
    return(FSUCCESS);
}



void
TimerHandler(
	void *Context)
{

	PQueryDetails pQueryElement;
	uint64 CurrMicroTime;
	boolean StartTimer = FALSE;
	uint32 index = 0;  //Signed Integer
	LIST_ITEM*	pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, TimerHandler);

	
	CurrMicroTime = GetTimeStamp();

	SpinLockAcquire(&pQueryList->m_Lock);
	if (TimerCallbackStopped(pTimerObject))
		goto unlock; // stale timer callback

	if (AtomicExchange(&IsTimerStarted,0) == 0) {
        _DBG_PRINT(_DBG_LVL_WARN,("TimerHandler was 0\n"));
    }

	while(NULL != (pListItem = QListGetItemAt(&pQueryList->m_List, index)))
	{
		pQueryElement = (PQueryDetails)pListItem->pObject;

		DEBUG_ASSERT(pQueryElement->QState != QueryComplete);
		DEBUG_ASSERT(pQueryElement->QState != QueryDestroy);
		if (pQueryElement->QState == ReadyToSend)
		{
			// was unable to send due to MaxOutstanding or GSI buffers,
			// see if it can be sent now
			SendQueryElement(pQueryElement, FALSE /*! FirstAttempt*/);
			StartTimer = TRUE;
			index++;
		} else if (((pQueryElement->QState == WaitingForResult)
				   || (pQueryElement->QState == NotAbleToSend))
				   && ((CurrMicroTime - pQueryElement->CmdSentTime) > 
						   (pQueryElement->ControlParameters.Timeout * 1000)))
		{
			// Command has not complected with in the predefined expected time
			// Check for Retrycount and resend the command
			if (pQueryElement->QState == WaitingForResult) {
				// it was sent and timed out
				AtomicDecrementVoid(&Outstanding);
			}
			if(pQueryElement->RetriesLeft > 0) 
			{	//Retry it
				_DBG_WARN (("Timeout has occured on a Query! <Retry>\n"));
				_DBG_INFO(("Retrying a QueryElement Retries Left = %d\n",
								pQueryElement->RetriesLeft));
				--(pQueryElement->RetriesLeft);
				SendQueryElement(pQueryElement, FALSE /*! FirstAttempt*/);
				StartTimer = TRUE;
				index++;
			 } else {
				_DBG_WARN (("Timeout has occured on a Query, Retries Exhausted! <Fail>\n"));
				if(pQueryElement->IsSelfCommand != TRUE)
				{
					// RecvHandler calls ScheduleCompletionProcessing as needed
					SubnetAdmRecv(pQueryElement,NULL,0, NULL);
				} else {
					// defer destroy to premptable context
					QListRemoveItem( &pQueryList->m_List, (LIST_ITEM *)pQueryElement );
					pQueryElement->QState = QueryDestroy;
					LQListInsertHead(pCompletionList, &pQueryElement->QueryListItem);  
					ScheduleCompletionProcessing();
				}
				// Do not increment the index++
			}
		} else if ((pQueryElement->QState == BusyRetryDelay)
				   && ((CurrMicroTime - pQueryElement->CmdSentTime) > 
						   (g_SdParams.SaBusyBackoff * 1000)))
		{
			// got a SA Busy response and we have some Retries left
			// it was sent and got a busy response, but was not processed
			AtomicDecrementVoid(&Outstanding);
			ASSERT(pQueryElement->RetriesLeft > 0);
			//Retry it
			_DBG_WARN (("SA Busy has occured on a Query! <Retry>\n"));
			_DBG_INFO(("Retrying a QueryElement Retries Left = %d\n",
							pQueryElement->RetriesLeft));
			--(pQueryElement->RetriesLeft);
			SendQueryElement(pQueryElement, FALSE /*! FirstAttempt*/);
			StartTimer = TRUE;
			index++;
		} else {
			// Command has not timed out, it is still in the QueryList
			// Need to start Timer again
			// this applies also to commands in WaitingForChildToComplete
			// and ProcessingResponse states
			StartTimer = TRUE;
			index++;
		}
	}
	
	if (StartTimer == TRUE)
		StartTimerHandler(pTimerObject, g_SdParams.TimeInterval);

unlock:
	SpinLockRelease(&pQueryList->m_Lock);

	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return;
}

// if we are not at MaxOutstanding
// send the next QueryElements from given provider Id which is are the QueryList
// in ReadyToSend state
void
SdSendNextOutstanding(void)
{
	PQueryDetails pQueryElement;
	uint32 index = 0;
	LIST_ITEM*	pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SdSendNextOutstanding);
	
	SpinLockAcquire(&pQueryList->m_Lock);
	if (AtomicRead(&Outstanding) >= g_SdParams.MaxOutstanding)
		goto unlock;
	while(NULL != (pListItem = QListGetItemAt(&pQueryList->m_List, index)))
	{
		pQueryElement = (PQueryDetails)pListItem->pObject;
		if (pQueryElement->QState == ReadyToSend)
		{
			SendQueryElement(pQueryElement, FALSE /*! FirstAttempt*/);
			if (AtomicRead(&Outstanding) >= g_SdParams.MaxOutstanding)
				break;
		}
		index++;
	}
unlock:
	SpinLockRelease(&pQueryList->m_Lock);
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return;
}

void
ProcessChildQueryQueue(
	void
	)
{
	LIST_ITEM*	pListItem;

	if (AtomicRead(&ChildQueryActive) > g_SdParams.MaxChildQuery)
		return;

	while(NULL != (pListItem = LQListRemoveTail(pChildQueryList)))
	{
		PQueryDetails pQueryElement = (PQueryDetails)pListItem->pObject;
		DEBUG_ASSERT(pQueryElement->pParentQuery);
		// this does not dereference pParentQuery
		if (! LQListIsItemInList(pQueryList,&pQueryElement->pParentQuery->QueryListItem))
		{
			// discard this one, parent has been canceled
			FreeQueryElement(pQueryElement);
		} else {
			SendQueryElement(pQueryElement, TRUE /*FirstAttempt*/);		

			if (AtomicIncrement(&ChildQueryActive) > g_SdParams.MaxChildQuery)
				break;
		}
	}
}

void
ScheduleCompletionProcessing(void)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ScheduleCompletionProcessing);
	if (AtomicExchange(&SdWorkItemQueued, 1) == 0) {
		SysCallbackQueue(pCallbackItemObject, ProcessCompletionQueue,
       			NULL, FALSE);
	} else {
		_DBG_INFO(("ProcessCompletionQueue already queued\n"));
	}
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
}

// This is called in a thread context.
// The FreeQueryElement (and possibly client callbacks) can prempt
void
ProcessCompletionQueue(
	void* Key, //Functional Device OBJECT
	void *Context)
	
{
	QUERY_RESULT_VALUES QueryResError;
	PQUERY_RESULT_VALUES pQueryRes;
	PQueryDetails pQueryElement;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessCompletionQueue);
	
	AtomicIncrementVoid(&SdWorkItemsInProcess);
	if (AtomicExchange(&SdWorkItemQueued,0) == 0) {
        _DBG_ERROR(("ProcessCompletionQueue was 0\n"));
	}

	ProcessChildQueryQueue();
	
	QueryResError.ResultDataSize = 0;

	while (NULL != (pQueryElement = (PQueryDetails)LQListRemoveHead(pCompletionList)))
	{
		DEBUG_ASSERT(pQueryElement->QState == QueryComplete
					|| pQueryElement->QState == QueryDestroy);
		if (pQueryElement->QState == QueryComplete)
		{
			if (pQueryElement->SdSentCmd == UserFabricOperation)
			{
				// This is a FabOp response
				pQueryElement->cmd.op.FabOpCallback(pQueryElement->UserContext,
                						&(pQueryElement->cmd.op.FabOp), 
										pQueryElement->Status,
										pQueryElement->MadStatus);
			} else {
				ASSERT(pQueryElement->SdSentCmd == UserQueryFabricInformation);
				if(pQueryElement->cmd.query.pClientResult == NULL)
				{
					_DBG_INFO(("ClientResult is null\n"));
					pQueryRes = &QueryResError;
					QueryResError.Status = pQueryElement->Status;
					QueryResError.MadStatus = pQueryElement->MadStatus;
				} else {
					pQueryRes = pQueryElement->cmd.query.pClientResult;
				}

				ASSERT(pQueryElement->cmd.query.UserCallback);
				(*pQueryElement->cmd.query.UserCallback)(pQueryElement->UserContext,
						&(pQueryElement->cmd.query.UserQuery), pQueryRes);
			}
		}
		FreeQueryElement(pQueryElement);
	}
	AtomicDecrementVoid(&SdWorkItemsInProcess);

	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return;
}

// used after a premptable operation on a QueryElement to see if the query
// element is still active to continue processing of it.
// must be called with pQueryList->m_Lock held for a query which was on
// the pQueryList when caller prempted.
// returns TRUE -  query has been cancelled,
// 					if this is last reference, the query element is freed
// 					(releases and reacquires lock so free can preempt)
// returns FALSE - query is still active, lock is not released
//
boolean
FreeCancelledQueryElement(
	PQueryDetails pQueryElement
	)
{
	if (pQueryElement->QState == QueryDestroy)
	{
		// query was cancelled, it is no longer on the list
		// if there are children still holding reference, we can't
		// free until last has released its reference (each will
		// call this function as they release, last child's call will cause
		// us to FreeQueryElement for the given parent)
		if (! pQueryElement->ChildProcessingResponseCount)
		{
			// all referenences are gone, free in a preemptable context
			SpinLockRelease(&pQueryList->m_Lock);
			FreeQueryElement(pQueryElement);
			SpinLockAcquire(&pQueryList->m_Lock);
		}
		return TRUE;
	} else {
		return FALSE;
	}
}

// Free the given query element
// should be called without holding any locks
// caller must remove query from any lists prior to call
void
FreeQueryElement(
	PQueryDetails pQueryElement
	)
{
	DEBUG_ASSERT(pQueryElement->SdSentCmd != UnknownCommand);
	DEBUG_ASSERT(! pQueryElement->ChildProcessingResponseCount);
	if (pQueryElement->SdSentCmd == UserQueryFabricInformation
		&& pQueryElement->cmd.query.pClientResult)
	{
		MemoryDeallocate(pQueryElement->cmd.query.pClientResult);
	}
	if (pQueryElement->u.pGmp)
	{
		MemoryDeallocate(pQueryElement->u.pGmp);
	}
	MemoryDeallocate(pQueryElement);
}
