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
#include <ispinlock.h>
#include <stl_sd.h>
#include <sdi.h>
#include <ib_generalServices.h>
#include <itimer.h>
#include <ireaper.h>
#include <dbg.h>
#include <ib_debug.h>
#include <isysutil.h>
#include <subscribe.h>

// This maintains subscriptions with the SA for traps/notices
// As part of this it handles SM restarts, port down/up and local resource
// exhaustion and retries the subscription as needed to maintain it
// To properly maintain subscriptions, it keeps no more than 1
// subscribe/unsubsribe operation outstanding at the SM at a time.
// This way we can be assured of the order they are processed (if there
// were more than 1 queued to SD, SD retries could result in them being
// processed by the SM in an unpredicatable order).
//
// A TRAP_REFERENCE is retained for each trap on a given port which has a
// request outstanding or has local clients registered.  Only once there is no
// one registered and no outstanding ops is it destroyed.
// This ensures there is no more than one op outstanding for a trap and hence
// ensures the order they are processed by the SM.
//
// Unsubscribes are not heavily retried, as the consequence is minimal.
// However the code is persistent about Subscribes so it can ensure it will
// get the desired traps/notices

typedef enum _TRAP_REFERENCE_FLAGS
{
	TRAP_REF_FLAGS_NONE       = 0x0000,
	TRAP_REF_FLAGS_SUBSCRIBED = 0x0001,	// are we subscribed for this trap
	TRAP_REF_FLAGS_OPQUEUED   = 0x0002	// is a FabricOperation started
} TRAP_REFERENCE_FLAGS;

typedef struct _CLIENT_TRAP_INFO
{
	LIST_ITEM               ListItem;
	uint16                  TrapNumber;
	EUI64                   PortGuid;
	void                    *pContext;	// client context for callback
	SD_REPORT_NOTICE_CALLBACK *pReportNoticeCallback;	// client callback
} CLIENT_TRAP_INFO;

typedef struct _TRAP_REFERENCE
{
	LIST_ITEM            ListItem;
	TIMER                Timer;
	uint16               TrapNumber;
	EUI64                PortGuid;
	uint32               RefCount;
	TRAP_REFERENCE_FLAGS Flags;
} TRAP_REFERENCE;

static CLIENT_HANDLE SdClientHandle;
static QUICK_LIST    SubscribedTrapsList;
static SPIN_LOCK     SubscribedTrapsListLock;

// Delays used when queuing a timer to issue subscribe/unsubscribe later
#define RETRY_DELAY 	1000	// 1 second in ms, retry a failure
#define PORT_DOWN_DELAY 60000 	// 1 minute in ms, retry a send/down failure
#define DEFER_DELAY 	1000 	// 1 second in ms, defer to releave SA

static int           SubscribeInitialized = 0;

static void
ProcessTrapSubscribeResponse(void* Context, FABRIC_OPERATION_DATA* pFabOp, FSTATUS Status, uint32 MadStatus);

// reaper callback
static void
ReapTrapReference(LIST_ITEM *pItem)
{
	// Reaper uses QListObj, so we must use PARENT_STRUCT
	TRAP_REFERENCE *pTrapReference = PARENT_STRUCT(pItem, TRAP_REFERENCE, ListItem);
	TimerDestroy(&pTrapReference->Timer);
	MemoryDeallocate(pTrapReference);
}

// Queue a Trap timer callback to issue a subscribe or unsubscribe
// must be called with SubscribedTrapsListLock held
static void
QueueTrapTimer(TRAP_REFERENCE *pTrapReference, uint32 timeout_ms)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, QueueTrapTimer);

	if (SubscribeInitialized != 0)
	{
		// restart timer if running
		TimerStop(&pTrapReference->Timer);
		TimerStart(&pTrapReference->Timer, timeout_ms);
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

// must be called with SubscribedTrapsListLock held
static void
IssueTrapSubscribe(TRAP_REFERENCE        *pTrapReference)
{
	FSTATUS               Status;
	FABRIC_OPERATION_DATA *pFabOp;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, IssueTrapSubscribe);
	
	ASSERT (pTrapReference != NULL);

	pFabOp = (FABRIC_OPERATION_DATA*)MemoryAllocate2AndClear( sizeof(FABRIC_OPERATION_DATA), IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG );
	if ( pFabOp == NULL )
	{
		_DBG_ERROR(("SubscribeForTrap MemoryAllocateError\n"));
		QueueTrapTimer(pTrapReference, RETRY_DELAY);	// try again later
	} else {
		pFabOp->Type = FabOpSetInformInfo;
		memset(&pFabOp->Value.InformInfo.GID, 0, sizeof(pFabOp->Value.InformInfo.GID));
		pFabOp->Value.InformInfo.LIDRangeBegin = LID_PERMISSIVE;
		pFabOp->Value.InformInfo.LIDRangeEnd = 0;
		pFabOp->Value.InformInfo.Reserved = 0;
		pFabOp->Value.InformInfo.IsGeneric = 1;
		pFabOp->Value.InformInfo.Subscribe = 1;
		pFabOp->Value.InformInfo.Type = IB_NOTICE_ALL;

		pFabOp->Value.InformInfo.u.Generic.TrapNumber = pTrapReference->TrapNumber;
		pFabOp->Value.InformInfo.u.Generic.u2.s.QPNumber = GSI_QP;
		pFabOp->Value.InformInfo.u.Generic.u2.s.RespTimeValue = 18;
		pFabOp->Value.InformInfo.u.Generic.u.s.ProducerType = STL_NODE_FI;

		Status = iba_sd_port_fabric_operation( SdClientHandle, pTrapReference->PortGuid, pFabOp, ProcessTrapSubscribeResponse, NULL, pTrapReference );
		if ((Status != FPENDING) && (Status != FSUCCESS))
		{
			/* Port may not yet be active.
			 * ReRegisterTrapSubscriptions will be called when it goes active
			 * but we queue a slow timer just in case operation failed for
			 * another reason (such as low memory, etc)
			 */
			/* _DBG_ERROR(("PortFabricOperation Status = %d\n", Status)); */
			QueueTrapTimer(pTrapReference, (SystemGetRandom() % PORT_DOWN_DELAY));// try again later
		} else {
			pTrapReference->Flags = (TRAP_REFERENCE_FLAGS)(pTrapReference->Flags | TRAP_REF_FLAGS_OPQUEUED);
		}

		MemoryDeallocate( pFabOp );
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

// must be called with SubscribedTrapsListLock held
static
void IssueTrapUnSubscribe(TRAP_REFERENCE *pTrapReference)
{
	FSTATUS               Status;
	FABRIC_OPERATION_DATA *pFabOp;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, IssueTrapUnSubscribe);
	
	pFabOp = (FABRIC_OPERATION_DATA*)MemoryAllocate2AndClear( sizeof(FABRIC_OPERATION_DATA), IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG );
	if ( pFabOp == NULL )
	{
		_DBG_ERROR(("SubscribeForTrap MemoryAllocateError\n"));
		QueueTrapTimer(pTrapReference, RETRY_DELAY);	// try again later
	} else {
		pFabOp->Type = FabOpSetInformInfo;
		memset(&pFabOp->Value.InformInfo.GID, 0, sizeof(pFabOp->Value.InformInfo.GID));
		pFabOp->Value.InformInfo.LIDRangeBegin = LID_PERMISSIVE;
		pFabOp->Value.InformInfo.LIDRangeEnd = 0;
		pFabOp->Value.InformInfo.Reserved = 0;
		pFabOp->Value.InformInfo.IsGeneric = 1;
		pFabOp->Value.InformInfo.Subscribe = 0;
		pFabOp->Value.InformInfo.Type = IB_NOTICE_ALL;

		pFabOp->Value.InformInfo.u.Generic.TrapNumber = pTrapReference->TrapNumber;
		pFabOp->Value.InformInfo.u.Generic.u2.s.QPNumber = GSI_QP;
		pFabOp->Value.InformInfo.u.Generic.u2.s.RespTimeValue = 18;
		pFabOp->Value.InformInfo.u.Generic.u.s.ProducerType = STL_NODE_FI;

		Status = iba_sd_port_fabric_operation( SdClientHandle, pTrapReference->PortGuid, pFabOp, ProcessTrapSubscribeResponse, NULL, pTrapReference );
		if ((Status != FPENDING) && (Status != FSUCCESS))
		{
			/* Port may not yet be active.
			 * ReRegisterTrapSubscriptions will be called when it goes active
			 * but we queue a slow timer just in case operation failed for
			 * another reason (such as low memory, etc)
			 */
			/* _DBG_ERROR(("PortFabricOperation Status = %d\n", Status)); */
			QueueTrapTimer(pTrapReference, PORT_DOWN_DELAY);// try again later
		} else {
			// to be conservative, assume we are unsubscribed, just in case
			// SA processes request, but delivers response late
			// also avoids us retrying unsubscribe in event SM is down
			pTrapReference->Flags = (TRAP_REFERENCE_FLAGS)(pTrapReference->Flags & ~TRAP_REF_FLAGS_SUBSCRIBED);
			pTrapReference->Flags = (TRAP_REFERENCE_FLAGS)(pTrapReference->Flags | TRAP_REF_FLAGS_OPQUEUED);
		}

		MemoryDeallocate( pFabOp );
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

// check trap state and issue subscriptions or destroy object as needed
// must be called with SubscribedTrapsListLock held
static void
ProcessTrapStateChange(TRAP_REFERENCE *pTrapReference, uint32 delay_ms)
{
	// we only queue 1 op at a time, this way be can ensure they are
	// not processed out of order
	if (! (pTrapReference->Flags & TRAP_REF_FLAGS_OPQUEUED))
	{
		if (pTrapReference->RefCount && ! (pTrapReference->Flags & TRAP_REF_FLAGS_SUBSCRIBED))
		{
			// we need to subscribe
			if (delay_ms)
				QueueTrapTimer(pTrapReference, delay_ms);
			else
				IssueTrapSubscribe(pTrapReference);
		} else if (! pTrapReference->RefCount && (pTrapReference->Flags & TRAP_REF_FLAGS_SUBSCRIBED)) {
			// we need to unsubscribe
			if (delay_ms)
				QueueTrapTimer(pTrapReference, delay_ms);
			else
				IssueTrapUnSubscribe(pTrapReference);
		} else if (pTrapReference->RefCount == 0) {
			// unsubscribed and no one needs to subscribe, cleanup
			QListRemoveItem(&SubscribedTrapsList, &pTrapReference->ListItem);
			TimerStop(&pTrapReference->Timer);	// make sure no future callbacks
			ReaperQueue(&pTrapReference->ListItem, ReapTrapReference);
		} else {
			// we are subscribed as we want to be
			ASSERT(pTrapReference->RefCount && (pTrapReference->Flags & TRAP_REF_FLAGS_SUBSCRIBED));
		}
	} else {
		// let sd callback call us when op finishes or fails
	}
}

static void
TrapTimerCallback(void *Context)
{
	TRAP_REFERENCE *pTrapReference = (TRAP_REFERENCE *)Context;

	SpinLockAcquire(&SubscribedTrapsListLock);
	if (TimerCallbackStopped(&pTrapReference->Timer))
	{
		SpinLockRelease(&SubscribedTrapsListLock);
		return;
	}
	ProcessTrapStateChange(pTrapReference, 0);
	SpinLockRelease(&SubscribedTrapsListLock);
}

// callback when SA query completes or fails
static void
ProcessTrapSubscribeResponse(void* Context, FABRIC_OPERATION_DATA* pFabOp, FSTATUS Status, uint32 MadStatus)
{
	TRAP_REFERENCE *pTrapReference = (TRAP_REFERENCE *)Context;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessTrapSubscribeResponse);
	
	SpinLockAcquire(&SubscribedTrapsListLock);
	if (SubscribeInitialized == 0)
		goto unlock;	// we are shutting down, don't start anything new

	pTrapReference->Flags = (TRAP_REFERENCE_FLAGS)(pTrapReference->Flags & ~TRAP_REF_FLAGS_OPQUEUED);
	if (Status != FSUCCESS)
	{
		// our attempt failed
		if (pFabOp->Value.InformInfo.Subscribe == 0)
		{
			_DBG_WARN( ("Trap Unsubscribe Failure. Status = %s, MadStatus = 0x%x: %s.\n", 
                        _DBG_PTR(iba_fstatus_msg(Status)), MadStatus, _DBG_PTR(iba_sd_mad_status_msg(MadStatus))) );
		} else {
			_DBG_WARN( ("Trap Subscribe Failure. Status = %s, MadStatus = 0x%x: %s.\n", 
                        _DBG_PTR(iba_fstatus_msg(Status)), MadStatus, _DBG_PTR(iba_sd_mad_status_msg(MadStatus))) );
		}
		// delay any retry or adjustment so we don't beat on SA too hard
		if (MadStatus == MAD_STATUS_BUSY)
		{
			ProcessTrapStateChange(pTrapReference, (SystemGetRandom() % PORT_DOWN_DELAY));
			_DBG_PRINT(_DBG_LVL_INFO, ("SM Returned Busy. Delaying Subscription By %ld MS.\n", (SystemGetRandom() % PORT_DOWN_DELAY)));
		} else {
			ProcessTrapStateChange(pTrapReference, RETRY_DELAY);
		}
	} else {
		if (pFabOp->Value.InformInfo.Subscribe == 1)
		{
			/* We are now subscribed with the SA */
			pTrapReference->Flags = (TRAP_REFERENCE_FLAGS)(pTrapReference->Flags | TRAP_REF_FLAGS_SUBSCRIBED);
		} else {
			/* We are now completely unsubscribed from the SA */
			// flag was cleared when we issued unsubscribe request
		}
		ProcessTrapStateChange(pTrapReference, 0);
	}
unlock:
	SpinLockRelease(&SubscribedTrapsListLock);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

void
SubscribeInitialize()
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SubscribeInitialize);

	iba_sd_register( &SdClientHandle, NULL ); /* TODO: Error Handling*/
	QListInit(&SubscribedTrapsList);
	SpinLockInitState(&SubscribedTrapsListLock);
	SpinLockInit(&SubscribedTrapsListLock);

	SubscribeInitialized = 1;

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

void
SubscribeTerminate()
{
	LIST_ITEM *pTrapReferenceListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SubscribeTerminate);

	SpinLockAcquire(&SubscribedTrapsListLock);
	SubscribeInitialized = 0;
	SpinLockRelease(&SubscribedTrapsListLock);

	iba_sd_deregister(SdClientHandle);

	/* no more SD callbacks will occur now, so OPQUEUED is not important */
	SpinLockAcquire(&SubscribedTrapsListLock);
	while ( NULL != (pTrapReferenceListItem = QListRemoveHead(&SubscribedTrapsList)))
	{
		TRAP_REFERENCE *pTrapReference = (TRAP_REFERENCE *)QListObj(pTrapReferenceListItem);

		ASSERT(pTrapReference != NULL);
		TimerStop(&pTrapReference->Timer);
		// release lock so TimerDestroy can wait for callback
		SpinLockRelease(&SubscribedTrapsListLock);
		TimerDestroy(&pTrapReference->Timer);

		MemoryDeallocate(pTrapReference);

		SpinLockAcquire(&SubscribedTrapsListLock);
	}
	SpinLockRelease(&SubscribedTrapsListLock);

	QListDestroy(&SubscribedTrapsList);
	SpinLockDestroy(&SubscribedTrapsListLock);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static __inline
FSTATUS
IncrementTrapUsageCount(uint16 TrapNumber, EUI64 PortGuid)
{
	static int     Seeded = 0;

	FSTATUS Status = FSUCCESS;
	LIST_ITEM      *pTrapReferenceListItem;
	TRAP_REFERENCE *pTrapReference;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, IncrementTrapUsageCount);

	SpinLockAcquire(&SubscribedTrapsListLock);
	if (SubscribeInitialized == 0)
	{
		SpinLockRelease(&SubscribedTrapsListLock);
		Status = FINVALID_STATE;
		goto exit;
	}

	pTrapReferenceListItem = QListHead(&SubscribedTrapsList);

	while (pTrapReferenceListItem != NULL)
	{
		pTrapReference = (TRAP_REFERENCE *)QListObj(pTrapReferenceListItem);

		ASSERT(pTrapReference != NULL);

		if ((pTrapReference->TrapNumber == TrapNumber) &&
			(pTrapReference->PortGuid   == PortGuid))
		{
			pTrapReference->RefCount++;
			goto process;
		}

		pTrapReferenceListItem = QListNext(&SubscribedTrapsList, pTrapReferenceListItem);
	}

	SpinLockRelease(&SubscribedTrapsListLock);

	pTrapReference = (TRAP_REFERENCE*)MemoryAllocate2AndClear(sizeof(TRAP_REFERENCE), IBA_MEM_FLAG_NONE, SUBNET_DRIVER_TAG);
	if (pTrapReference == NULL)
	{
		_DBG_ERROR(("IncrementTrapUsageCount MemoryAllocateError\n"));
		Status = FINSUFFICIENT_MEMORY;
		goto exit;
	}

	TimerInitState(&pTrapReference->Timer);
	TimerInit(&pTrapReference->Timer, TrapTimerCallback, pTrapReference);
	pTrapReference->TrapNumber = TrapNumber;
	pTrapReference->PortGuid = PortGuid;
	pTrapReference->RefCount = 1;
	pTrapReference->Flags = TRAP_REF_FLAGS_NONE;

	QListSetObj(&pTrapReference->ListItem, pTrapReference);
	
	SpinLockAcquire(&SubscribedTrapsListLock);
	QListInsertTail(&SubscribedTrapsList, &pTrapReference->ListItem);

	if (!Seeded)
	{
		SystemSeedRandom((unsigned long)pTrapReference->PortGuid);
		Seeded = 1;
	}

process:
	// Subscribe as needed, defer since its likely we are just starting
	// the drivers and SA may not yet be ready for operations
	ProcessTrapStateChange(pTrapReference, DEFER_DELAY);
	SpinLockRelease(&SubscribedTrapsListLock);

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}

static __inline
void
DecrementTrapUsageCount(uint16 TrapNumber, EUI64 PortGuid)
{
	LIST_ITEM      *pTrapReferenceListItem;
	TRAP_REFERENCE *pTrapReference;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DecrementTrapUsageCount);

	SpinLockAcquire(&SubscribedTrapsListLock);
	
	pTrapReferenceListItem = QListHead(&SubscribedTrapsList);
	
	while (pTrapReferenceListItem != NULL)
	{
		pTrapReference = (TRAP_REFERENCE *)QListObj(pTrapReferenceListItem);
		
		ASSERT(pTrapReference != NULL);
		
		if ((pTrapReference->TrapNumber == TrapNumber) && 
			(pTrapReference->PortGuid   == PortGuid))
		{
			pTrapReference->RefCount--;
			// unsubscribe as needed
			ProcessTrapStateChange(pTrapReference, 0);
			break;
		}

		pTrapReferenceListItem = QListNext(&SubscribedTrapsList, pTrapReferenceListItem);
	}

	SpinLockRelease(&SubscribedTrapsListLock);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

// client is deregistering from SD, remove all their trap registrations
void
UnsubscribeClient(ClientListEntry *pClientEntry)
{
	LIST_ITEM *pClientTrapItem = QListRemoveHead(&pClientEntry->ClientTraps);

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, UnsubscribeClient);

	while (pClientTrapItem != NULL)
	{
		CLIENT_TRAP_INFO *pClientTrapInfo = (CLIENT_TRAP_INFO *)QListObj(pClientTrapItem);

		ASSERT(pClientTrapInfo != NULL);

		DecrementTrapUsageCount(pClientTrapInfo->TrapNumber, pClientTrapInfo->PortGuid);

		MemoryDeallocate(pClientTrapInfo);

		pClientTrapItem = QListRemoveHead(&pClientEntry->ClientTraps);
	}

	QListDestroy(&pClientEntry->ClientTraps);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

// due to a port down event or as part of reregistration, clear all our
// subscription state
// if port down, no use retrying subscriptions now, for reregistration
// caller will process state change
void
InvalidatePortTrapSubscriptions(EUI64 PortGuid)
{
	LIST_ITEM      *pTrapReferenceListItem;
	TRAP_REFERENCE *pTrapReference;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, InvalidatePortTrapSubscriptions);

	SpinLockAcquire(&SubscribedTrapsListLock);
	
	for (pTrapReferenceListItem = QListHead(&SubscribedTrapsList);
		 pTrapReferenceListItem != NULL;
		 pTrapReferenceListItem = QListNext(&SubscribedTrapsList, pTrapReferenceListItem))
	{
		pTrapReference = (TRAP_REFERENCE *)QListObj(pTrapReferenceListItem);

		ASSERT(pTrapReference != NULL);

		if (pTrapReference->PortGuid == PortGuid)
		{
			pTrapReference->Flags = (TRAP_REFERENCE_FLAGS)(pTrapReference->Flags & ~TRAP_REF_FLAGS_SUBSCRIBED);
		}
	}

	SpinLockRelease(&SubscribedTrapsListLock);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
}

// called for port SM, LID or PKey change.  Note LID change can imply
// an SM reregistration request as well
void
ReRegisterTrapSubscriptions(EUI64 PortGuid)
{
	LIST_ITEM      *pTrapReferenceListItem;
	TRAP_REFERENCE *pTrapReference;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ReRegisterTrapSubscriptions);
	
	InvalidatePortTrapSubscriptions(PortGuid);

	SpinLockAcquire(&SubscribedTrapsListLock);

	pTrapReferenceListItem = QListHead(&SubscribedTrapsList);

	while (pTrapReferenceListItem != NULL)
	{
		pTrapReference = (TRAP_REFERENCE *)QListObj(pTrapReferenceListItem);

		pTrapReferenceListItem = QListNext(&SubscribedTrapsList, pTrapReferenceListItem);
		ASSERT(pTrapReference != NULL);

		if (pTrapReference->PortGuid == PortGuid)
		{
			// we delay so SA can settle a little and get ready for requests
			ProcessTrapStateChange(pTrapReference, DEFER_DELAY);
		}
	}

	SpinLockRelease(&SubscribedTrapsListLock);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static __inline
CLIENT_TRAP_INFO *
FindClientTrapInfo(QUICK_LIST* pClientTraps, uint16 TrapNumber, EUI64 PortGuid)
{
	LIST_ITEM        *pClientTrapInfoItem = QListHead(pClientTraps);
	CLIENT_TRAP_INFO *pClientTrapInfo = NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FindClientTrapInfo);

	while (pClientTrapInfoItem != NULL)
	{
		pClientTrapInfo = (CLIENT_TRAP_INFO *)QListObj(pClientTrapInfoItem);

		ASSERT(pClientTrapInfo != NULL);

		if ((pClientTrapInfo->TrapNumber == TrapNumber) &&
			(pClientTrapInfo->PortGuid   == PortGuid))
		{
			break;
		}

		pClientTrapInfo = NULL;
		pClientTrapInfoItem = QListNext(pClientTraps, pClientTrapInfoItem);
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return pClientTrapInfo;
}

/*!
  \brief Looks through the list of Subnet Driver clients for those which have
         subscribed for callback when a specific trap report has been received
*/
void
ProcessClientTraps(void *pContext, IB_NOTICE *pNotice, EUI64 PortGuid)
{
	LIST_ITEM *pClientEntryListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessClientTraps);
	
	SpinLockAcquire(pClientListLock);

	pClientEntryListItem = QListHead(pClientList);
	while (pClientEntryListItem != NULL)
	{
		CLIENT_TRAP_INFO *pClientTrapInfo;
		ClientListEntry  *pClientListEntry = (ClientListEntry *)QListObj(pClientEntryListItem);

		ASSERT(pClientListEntry != NULL);

		/* Call Subscribed Clients  */

		pClientTrapInfo = FindClientTrapInfo(&pClientListEntry->ClientTraps, pNotice->u.Generic.TrapNumber, PortGuid);
		if ((pClientTrapInfo != NULL) &&
			(pClientTrapInfo->pReportNoticeCallback != NULL))
		{
			SpinLockRelease(pClientListLock); /* TODO: This needs more thought for other than discovery */
			(*pClientTrapInfo->pReportNoticeCallback)(pClientTrapInfo->pContext, pNotice, PortGuid);
			SpinLockAcquire(pClientListLock);
		}

		pClientEntryListItem = QListNext(pClientList, pClientEntryListItem);
	}

	SpinLockRelease(pClientListLock);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

FSTATUS
iba_sd_trap_notice_subscribe(CLIENT_HANDLE ClientHandle,
                    uint16 TrapNumber,
                    EUI64 PortGuid,
                    void *pContext,
                    SD_REPORT_NOTICE_CALLBACK *pReportNoticeCallback)
{
	FSTATUS                   Status;
	CLIENT_CONTROL_PARAMETERS ClientParams;
	ClientListEntry           *pClientEntry = (ClientListEntry *)ClientHandle;
	CLIENT_TRAP_INFO          *pClientTrapInfo;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_trap_notice_subscribe);

	Status = ValidateClientHandle(ClientHandle, &ClientParams);
	if (Status != FSUCCESS)
	{
		_DBG_ERROR(("Invalid Client Handle Specified\n"));
		goto exit;
	}

	SpinLockAcquire(pClientListLock);

	pClientTrapInfo = FindClientTrapInfo(&pClientEntry->ClientTraps, TrapNumber, PortGuid);
	if (pClientTrapInfo != NULL)
	{
		// Already Subscribed
		Status = FDUPLICATE;
		SpinLockRelease(pClientListLock);
		goto exit;
	}

	SpinLockRelease(pClientListLock);

	pClientTrapInfo = (CLIENT_TRAP_INFO*)MemoryAllocate2AndClear(sizeof(CLIENT_TRAP_INFO), IBA_MEM_FLAG_NONE, SUBNET_DRIVER_TAG);
	if (pClientTrapInfo == NULL)
	{
		Status = FINSUFFICIENT_MEMORY;
		goto exit;
	}

	pClientTrapInfo->TrapNumber = TrapNumber;
	pClientTrapInfo->PortGuid = PortGuid;
	pClientTrapInfo->pContext = pContext;
	pClientTrapInfo->pReportNoticeCallback = pReportNoticeCallback;

	QListSetObj(&pClientTrapInfo->ListItem, pClientTrapInfo);

	SpinLockAcquire(pClientListLock);
	QListInsertTail(&pClientEntry->ClientTraps, &pClientTrapInfo->ListItem);
	SpinLockRelease(pClientListLock);

	Status = IncrementTrapUsageCount(TrapNumber, PortGuid);
	if (Status != FSUCCESS)
	{
		SpinLockAcquire(pClientListLock);
		QListRemoveItem(&pClientEntry->ClientTraps, &pClientTrapInfo->ListItem);
		SpinLockRelease(pClientListLock);
		MemoryDeallocate(pClientTrapInfo);
	}

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}

FSTATUS
iba_sd_trap_notice_unsubscribe(CLIENT_HANDLE ClientHandle, uint16 TrapNumber, EUI64 PortGuid)
{
	FSTATUS                   Status;
	CLIENT_CONTROL_PARAMETERS ClientParams;
	ClientListEntry           *pClientEntry = (ClientListEntry *)ClientHandle;
	CLIENT_TRAP_INFO          *pClientTrapInfo;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_trap_notice_unsubscribe);

	Status = ValidateClientHandle(ClientHandle, &ClientParams);
	if (Status != FSUCCESS)
	{
		_DBG_ERROR(("Invalid Client Handle Specified\n"));
		goto exit;
	}

	SpinLockAcquire(pClientListLock);

	pClientTrapInfo = FindClientTrapInfo(&pClientEntry->ClientTraps, TrapNumber, PortGuid);
	if (pClientTrapInfo == NULL)
	{
		// Not Found
		Status = FERROR;
		SpinLockRelease(pClientListLock);
		goto exit;
	}

	QListRemoveItem(&pClientEntry->ClientTraps, &pClientTrapInfo->ListItem);

	SpinLockRelease(pClientListLock);

	MemoryDeallocate(pClientTrapInfo);

	DecrementTrapUsageCount(TrapNumber, PortGuid);

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}

// reports FALSE if we have failed to subscribe to any traps for given port
// returns TRUE if we succeeded to subscribe to all desired traps for
// given port or if we have requested no subscriptions for given port
boolean
iba_sd_port_traps_subscribed(EUI64 PortGuid)
{
	boolean        Subscribed = TRUE;
	LIST_ITEM      *pTrapReferenceListItem;
	TRAP_REFERENCE *pTrapReference;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_port_traps_subscribed);

	SpinLockAcquire(&SubscribedTrapsListLock);

	for (pTrapReferenceListItem = QListHead(&SubscribedTrapsList);
		 pTrapReferenceListItem != NULL;
		 pTrapReferenceListItem = QListNext(&SubscribedTrapsList, pTrapReferenceListItem))
	{
		pTrapReference = (TRAP_REFERENCE *)QListObj(pTrapReferenceListItem);

		ASSERT(pTrapReference != NULL);

		if (pTrapReference->PortGuid == PortGuid)
		{
			if (pTrapReference->RefCount && !(pTrapReference->Flags & TRAP_REF_FLAGS_SUBSCRIBED))
			{
				Subscribed = FALSE;
				break;
			}
		}
	}

	SpinLockRelease(&SubscribedTrapsListLock);
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Subscribed;
}
