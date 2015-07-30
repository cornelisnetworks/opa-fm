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

#include "ibnotify.h"

//
// Globals
//
IBT_NOTIFY_GROUP	NotifyGroup;


//
// Initialize the Notify Group
//
//INPUT:
// 	Callback - when a new User is added to the Notify group, this routine
//		is called to distribute initial events to User
void
IbtInitNotifyGroup (
   IN	IB_ROOT_CALLBACK		Callback
   )
{
	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, IbtInitNotifyGroup );

	ASSERT(! NotifyGroup.Initialized);

	MapInitState( &NotifyGroup.Lists );
	SpinLockInitState( &NotifyGroup.Lock );

	if (MapInit( &NotifyGroup.Lists ) != FSUCCESS)
	{
		_DBG_ERROR (("Failed to Initialize Notification Map\n"));
		goto done;
	}
	if (SpinLockInit( &NotifyGroup.Lock ) == FALSE)
	{
		_DBG_ERROR (("Failed to Initialize Notification lock\n"));
		goto faillock;
	}

	// Add root callback for new users
	ASSERT ( Callback );
	NotifyGroup.RootCallback = Callback;

	NotifyGroup.Initialized = TRUE;

done:
	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
	return;

faillock:
	MapDestroy ( &NotifyGroup.Lists );
	goto done;
}


//
// Cleanup for shutdown
void
IbtDestroyNotifyGroup ( void )
{
	QUICK_LIST				*pList;

	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, IbtDestroyNotifyGroup );

	if (! NotifyGroup.Initialized)
		goto done;

	// Empty list contents
	SpinLockAcquire( &NotifyGroup.Lock );

	while (NULL != (pList = (QUICK_LIST*)MapRemoveHead(&NotifyGroup.Lists)))
	{
		LIST_ITEM				*pListItem;

		while (NULL != (pListItem = QListRemoveHead(pList)))
		{
			IBT_NOTIFY_USER_BLOCK	*pUser = (IBT_NOTIFY_USER_BLOCK *)pListItem;
			MemoryDeallocate( pUser );
		}	
		QListDestroy(pList);
		MemoryDeallocate(pList);
	}
	NotifyGroup.Initialized = FALSE;

	SpinLockRelease( &NotifyGroup.Lock );

	MapDestroy( &NotifyGroup.Lists );
	SpinLockDestroy( &NotifyGroup.Lock );

done:
	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
}

// Notify all users in pList of pRec event
void
IbtNotifyList(
	QUICK_LIST				*pList,
	IB_NOTIFY_RECORD		*pRec
	 )
{
	LIST_ITEM				*pListFirst;
	IBT_NOTIFY_USER_BLOCK	*pUser;

	pListFirst = QListHead( pList );
			
	while (pListFirst)
	{
		// If event has been subscribed by user, notify
		if ( pRec->EventType & ((IBT_NOTIFY_USER_BLOCK *)pListFirst)->EventMask)
		{
			pUser = (IBT_NOTIFY_USER_BLOCK *)pListFirst;
			pRec->Context = pUser->Context;
			if (pUser->Locking == IB_NOTIFY_ASYNC)
			{
				pUser->RefCount++;
				SpinLockRelease( &NotifyGroup.Lock );
				( pUser->Callback )( *pRec );
				SpinLockAcquire( &NotifyGroup.Lock );
				pListFirst = QListNext( pList, pListFirst );
				if (--(pUser->RefCount) == 0)
				{
					QListRemoveItem ( pList, &pUser->ListItem );
					MemoryDeallocate( pUser );
				}
				continue;
			} else {
				ASSERT(pUser->RefCount);
				( pUser->Callback )( *pRec );
			}
		}
		pListFirst = QListNext( pList, pListFirst );
	}
}

// Notify all registered clients of an event against the given Guid
void
IbtNotifyGroup (
	IN	EUI64				Guid,			// Port/Node GUID
	IN	IB_NOTIFY_TYPE		EventType		// Type of event being reported
	)
{
	IB_NOTIFY_RECORD		Rec;
	QUICK_LIST*				pList;

	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, IbtNotifyGroup );

	_DBG_PRINT( _DBG_LVL_IBT_API,(
				"NotifyGroup\n"
				"\tEventType...:%"PRIxN"\n"
				"\tGUID........:x%016"PRIx64"\n",
				EventType, 
				Guid ));

	if (! NotifyGroup.Initialized)
		goto done;

	// Fill in common record info
	Rec.Guid = Guid;
	Rec.EventType = EventType;

	SpinLockAcquire( &NotifyGroup.Lock );
		
	// iterate on List in Map for Guid, then map for Guid==0
	pList = (QUICK_LIST*)MapGet(&NotifyGroup.Lists, Guid);
	if (pList != NULL)
	{
		IbtNotifyList(pList, &Rec);
	}

	pList = (QUICK_LIST*)MapGet(&NotifyGroup.Lists, 0);
	if (pList != NULL)
	{
		IbtNotifyList(pList, &Rec);
	}

	SpinLockRelease( &NotifyGroup.Lock );

done:
	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
}

// Send a notification to a single registered notify client
// used as part of RootCallback to send initial events to a newly
// registered client
// returns FALSE if client has deregistered itself in callback
boolean
IbtNotifyUserEvent (
	IN	void				*UserContext,
	IN	EUI64				Guid,			// Port/Node GUID
	IN	IB_NOTIFY_TYPE		EventType		// Type of event being reported
	)
{
	IB_NOTIFY_RECORD		Rec;
	IBT_NOTIFY_USER_BLOCK	*pUser = (IBT_NOTIFY_USER_BLOCK*)UserContext;

	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, IbtNotifyUserEvent );

	_DBG_PRINT( _DBG_LVL_IBT_API,(
			"NotifyGroup\n"
			"\tEventType...:%"PRIxN"\n"
			"\tGUID........:x%016"PRIx64"\n",
			EventType, 
			Guid ));

	if (! NotifyGroup.Initialized)
	{
		pUser = NULL;	// no more notifications needed
		goto done;
	}

	// Fill in common record info
	Rec.Guid = Guid;
	Rec.EventType = EventType;

	SpinLockAcquire( &NotifyGroup.Lock );

	// If event has been subscribed by user, notify
	if ( ( EventType & pUser->EventMask )
		&& (pUser->Guid == 0 || pUser->Guid == Guid))
	{
		Rec.Context = pUser->Context;
		if (pUser->Locking == IB_NOTIFY_ASYNC)
		{
			pUser->RefCount++;
			SpinLockRelease( &NotifyGroup.Lock );
			( pUser->Callback )( Rec );
			SpinLockAcquire( &NotifyGroup.Lock );
			if (--(pUser->RefCount) == 0)
			{
				QUICK_LIST*				pList;

				pList = (QUICK_LIST*)MapGet(&NotifyGroup.Lists, pUser->Guid);
				if (pList != NULL)
					QListRemoveItem ( pList, &pUser->ListItem );
				MemoryDeallocate( pUser );
				pUser = NULL;
			}
		} else {
			ASSERT(pUser->RefCount);
			( pUser->Callback )( Rec );
		}
	} 

	SpinLockRelease( &NotifyGroup.Lock );

done:
	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
	return (pUser != NULL);
}



//
// iba_register_notify
//
//	The caller uses this API to register for notifications of IB type events
//
//
// INPUTS:
//
//	Callback		-Callback routine to be registered for notifications
//
//	Context			-User specified context to return in callbacks
//	
//	EventMask		-Mask to subscribe to event types
//
//	Locking			-locking model for callback
//
//
//
// OUTPUTS:
//
//	NotifyHandle	-Notification handle on successful registration
//
//
// RETURNS:
//
//	FSUCCESS
//	FINSUFFICIENT_MEMORY
//
//
FSTATUS
iba_register_notify (
	IN	IB_NOTIFY_CALLBACK	Callback,
	IN	void				*Context,
	IN	IB_NOTIFY_TYPE		EventMask,
	IN	IB_NOTIFY_LOCKING	Locking,
	OUT	IB_HANDLE			*NotifyHandle
	)
{
	FSTATUS					status	= FSUCCESS;
	
	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, iba_register_notify );
	status = iba_register_guid_notify(0, Callback, Context, EventMask, Locking, NotifyHandle);
	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
	return status;
}


//
// iba_register_guid_notify
//
//	The caller uses this API to register for notifications of IB type events
//
//
// INPUTS:
//
//	Guid			-Guid user is interested in
//
//	Callback		-Callback routine to be registered for notifications
//
//	Context			-User specified context to return in callbacks
//	
//	EventMask		-Mask to subscribe to event types
//
//	Locking			-locking model for callback
//
//
//
// OUTPUTS:
//
//	NotifyHandle	-Notification handle on successful registration
//
//
// RETURNS:
//
//	FSUCCESS
//	FINSUFFICIENT_MEMORY
//
//
FSTATUS
iba_register_guid_notify (
	IN  EUI64				Guid,
	IN	IB_NOTIFY_CALLBACK	Callback,
	IN	void				*Context,
	IN	IB_NOTIFY_TYPE		EventMask,
	IN	IB_NOTIFY_LOCKING	Locking,
	OUT	IB_HANDLE			*NotifyHandle
	)
{
	FSTATUS					status	= FSUCCESS;
	IBT_NOTIFY_USER_BLOCK	*pUser;
	QUICK_LIST *pList;
	
	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, iba_register_notify );

	if (! NotifyGroup.Initialized)
	{
		status = FINVALID_STATE;
		goto done;
	}

	// Allocate memory to hold user registration
	pUser = (IBT_NOTIFY_USER_BLOCK*)MemoryAllocateAndClear(
										sizeof(IBT_NOTIFY_USER_BLOCK), FALSE,
										IBT_NOTIFY_TAG );
	if ( !pUser )
	{
		status = FINSUFFICIENT_MEMORY;
		_DBG_ERROR(("%s\n", _DBG_PTR(FSTATUS_MSG(status))));
		goto done;
	}

	// Fill in user details
	pUser->Guid = Guid;	// all Guids
	pUser->Callback = Callback;
	pUser->Context = Context;
	pUser->RefCount = 1;
	pUser->EventMask = EventMask;
	pUser->Locking = Locking;

	// Save user list in group
	SpinLockAcquire( &NotifyGroup.Lock );
	pList = (QUICK_LIST*)MapGet(&NotifyGroup.Lists, Guid);
	if (pList == NULL)
	{
		pList = (QUICK_LIST*)MemoryAllocateAndClear( sizeof(QUICK_LIST), FALSE,
										IBT_NOTIFY_TAG );
		QListInitState(pList);
		if (! QListInit(pList))
		{
			status = FINSUFFICIENT_RESOURCES;
			goto faillist;
		}
		status = MapInsert(&NotifyGroup.Lists, Guid, pList);
		if (status != FSUCCESS)
		{
			goto failmap;
		}
	}
	QListInsertHead( pList, &pUser->ListItem );
	SpinLockRelease( &NotifyGroup.Lock );

	*NotifyHandle = pUser;

	_DBG_PRINT(_DBG_LVL_IBT_API,("Created entry x%p\n", _DBG_PTR(pUser)));

	if (EventMask & IB_NOTIFY_ON_REGISTER)
	{
		// Notify Root to pass user initial events
		ASSERT (( NotifyGroup.RootCallback ));
		( NotifyGroup.RootCallback )( pUser );
	}

done:
	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
	return status;

failmap:
	QListDestroy(pList);
faillist:
	MemoryDeallocate(pList);
	goto done;
}



//
// iba_deregister_notify 
//
//	The caller uses this API to Deregister previously registered notifications
//
//
// INPUTS:
//
//	NotifyHandle		-Handle returned on successful registration
//
//
//
// OUTPUTS:
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_MEMORY
//
//
FSTATUS
iba_deregister_notify (
	IN 	IB_HANDLE			NotifyHandle
	)
{
	FSTATUS					status	= FSUCCESS;
	IBT_NOTIFY_USER_BLOCK	*pUser	= (IBT_NOTIFY_USER_BLOCK*)NotifyHandle;
	QUICK_LIST				*pList;


	_DBG_ENTER_LVL( _DBG_LVL_IBT_API, iba_deregister_notify );

	if (! NotifyGroup.Initialized)
	{
		// could have never registered if we didn't initialize
		status = FINVALID_PARAMETER;
		goto done;
	}
	SpinLockAcquire( &NotifyGroup.Lock );
	pList = (QUICK_LIST*)MapGet(&NotifyGroup.Lists, pUser->Guid);
	ASSERT(pList);
	if ( !QListIsItemInList( pList, &pUser->ListItem ) )
	{
		status = FINVALID_PARAMETER;
		_DBG_ERROR(("Could not deregister. Notification is not in List!!!\n"));
		goto done;
	}
	if (pUser->Locking == IB_NOTIFY_ASYNC)
	{
		if (--(pUser->RefCount) != 0)
		{
			// in use in callback
			goto done;
		}
	} else {
		ASSERT(pUser->RefCount == 1);
	}

	QListRemoveItem ( pList, &pUser->ListItem );
	MemoryDeallocate( pUser );

done:
	SpinLockRelease( &NotifyGroup.Lock );

	_DBG_LEAVE_LVL(_DBG_LVL_IBT_API);
	return status;
}
