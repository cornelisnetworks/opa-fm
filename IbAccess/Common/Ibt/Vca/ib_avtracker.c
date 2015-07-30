/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/

/* AV Tracker to help manage AVs, especially multicast AVs
 *
 * The AV tracker is used to track the current AV to send to a destination.
 * It also helps in handling the reference counting problem by
 * tracking previous AVs until their reference count hits zero.
 *
 * Typically 1 AV Tracker would be created per destination (such as a MC group)
 * and the current AV would reflect the latest AV for sending to that group.
 * When path record queries or Multicast joins succeed, a corresponding AV
 * should be created and IbAvTrackerSetCurrent.  If the patch record is stale
 * or the Multicast join is lost (port down, etc), the current should be cleared
 * via IbAvTrackerClearCurrent.
 *
 * Of course, AVs cannot be destroyed until all send WQEs which use them have
 * completed.  To this end, the AV tracker keeps a reference count per AV
 * and manages the destroy.  Each time a send needs to occur to the destination,
 * IbAvTrackerGetCurrent should be called.  The incRef argument will cause the
 * reference count for the AV to be incremented to prevent its destruction
 * while in use.  When the send completion occurs, IbAvTrackerDecRef should be
 * called.  This will destroy the AV when its count reaches 0 [SetCurrent counts
 * as a reference so the current AV will not be destroyed by DecRef].
 *
 * The actual meaning of references for GetCurrent/IncRef/DecRef is up to
 * the caller.  For example 1 reference count be counted per WQE.  An
 * alternative could be to count 1 reference per scheduled completion.
 */
#include "ib_debug.h"
#include "ib_debug_osd.h"
#include "ibt.h"

#include "precomp.h"
#define ALLOC_TAG VCA_MEM_TAG

/*++

Routine Description
	zero out fields in AV_TRACKER

Input:
	pTracker:	AV tracker object to initialize

Output:
	None
--*/
void
IbAvTrackerInitState(
	IN AV_TRACKER *pTracker
	)
{
	_DBG_ENTER_FUNC(IbAvTrackerInitState);
	SpinLockInitState(&pTracker->lock);
	pTracker->currentAV = NULL;
	QListInitState(&pTracker->avList);
	_DBG_LEAVE_FUNC();
}

/*++

Routine Description
	Initialize AV_TRACKER

Input:
	pTracker:	AV tracker object to initialize
	caHandle:	Open CA instance used to create AVs
	pdHandle:	PD instance used to create AVs

Output:
	status of the operation

Additional Information:
	caHandle and pdHandle are only used if IbAvTrackerMulticastCallback is used
--*/
FSTATUS
IbAvTrackerInit(
	IN AV_TRACKER *pTracker,
	IN IB_HANDLE caHandle,
	IN IB_HANDLE pdHandle
	)
{
	_DBG_ENTER_FUNC(IbAvTrackerInit);
	if (! SpinLockInit(&pTracker->lock))
		goto faillock;
	if (! QListInit(&pTracker->avList))
		goto faillist;
	pTracker->caHandle = caHandle;
	pTracker->pdHandle = pdHandle;
	_DBG_LEAVE_FUNC();
	return FSUCCESS;

faillist:
	SpinLockDestroy(&pTracker->lock);
faillock:
	_DBG_LEAVE_FUNC();
	return FERROR;
}

/*++

Routine Description
	Destroy AV_WRAP

Input:
	pAvWrap:	AV tracked object to destroy

Output:
	None

Additional Information:
	destroys the AV and frees AV_WRAP
	caller must have removed from avList first

--*/
static void
IbAvTrackerDestroyAV(
	IN AV_WRAP	*pAvWrap
	)
{
	_DBG_ENTER_FUNC(IbAvTrackerDestroyAV);
	(void)iba_destroy_av(pAvWrap->avHandle);
	MemoryDeallocate(pAvWrap);
	_DBG_LEAVE_FUNC();
}

/*++

Routine Description
	Destroy AV_TRACKER

Input:
	pTracker:	AV tracker object to destroy

Output:
	status of the operation

Additional Information:
	Any AVs still in the IbAvTracker will be destroyed by this
	operation.

--*/
void
IbAvTrackerDestroy(
	IN AV_TRACKER	*pTracker
	)
{
	LIST_ITEM *pItem;

	_DBG_ENTER_FUNC(IbAvTrackerDestroy);
	SpinLockAcquire(&pTracker->lock);
	while (NULL != (pItem=QListRemoveHead(&pTracker->avList)))
		IbAvTrackerDestroyAV((AV_WRAP*)QListObj(pItem));
	SpinLockRelease(&pTracker->lock);
	
	SpinLockDestroy(&pTracker->lock);
	pTracker->currentAV = NULL;
	QListDestroy(&pTracker->avList);
	_DBG_LEAVE_FUNC();
}


/*++

Routine Description
	Get current AV for tracker

Input:
	pTracker:	AV tracker object to act on
	incRef:		should reference count for returned AV be incremented

Output:
	AV handle of current AV, can be NULL if no current AV

Additional Information:
	Current AV is the last one Created/Set for the tracker
	This typically indicates the most recent path to the destination

--*/
IB_HANDLE
IbAvTrackerGetCurrent(
	IN AV_TRACKER	*pTracker,
	IN boolean		incRef
	)
{
	IB_HANDLE result = NULL;
	AV_WRAP *currentAV;

	_DBG_ENTER_FUNC(IbAvTrackerGetCurrent);
	SpinLockAcquire(&pTracker->lock);
	currentAV = pTracker->currentAV;
	if (currentAV) {
		result = currentAV->avHandle;
		if (incRef)
			AtomicIncrement(&currentAV->refCount);
	}
	SpinLockRelease(&pTracker->lock);
	_DBG_LEAVE_FUNC();
	return result;
}

/*++

Routine Description
	Find given AV in tracker

Input:
	pTracker:	AV tracker object to act on
	avHandle:	AV handle to search for

Output:
	AV_WRAP for given AV (or NULL)

Additional Information:
	AV tracker lock must be held when called
--*/
static AV_WRAP*
IbAvTrackerFindAV(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	)
{
	AV_WRAP *found;
	LIST_ITEM *pItem;

	_DBG_ENTER_FUNC(IbAvTrackerFindAV);
	/* typically we will be operating on currentAV */
	if (pTracker->currentAV && pTracker->currentAV->avHandle == avHandle) {
		_DBG_LEAVE_FUNC();
		return pTracker->currentAV;
	}
	found = NULL;
	for (pItem = QListHead(&pTracker->avList); pItem != NULL; pItem = QListNext(&pTracker->avList, pItem)) {
		AV_WRAP* pAvWrap = (AV_WRAP*)QListObj(pItem);
		if (pAvWrap->avHandle == avHandle) {
			found = pAvWrap;
			break;
		}
	}
	_DBG_LEAVE_FUNC();
	return found;
}

/*++

Routine Description
	add given AV to tracker

Input:
	pTracker:	AV tracker object to act on
	avHandle:	AV handle to add

Output:
	AV_WRAP for given AV (or NULL)

Additional Information:
	AV tracker lock must be held when called
	avHandle must not already be in the list
--*/
static AV_WRAP*
IbAvTrackerAddAV(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	)
{
	AV_WRAP *pAvWrap;

	_DBG_ENTER_FUNC(IbAvTrackerAddAV);
	pAvWrap = (AV_WRAP*)MemoryAllocate2AndClear(
						sizeof(AV_WRAP), IBA_MEM_FLAG_NONE, ALLOC_TAG);
	if (! pAvWrap)
		goto done;
	ListItemInitState(&pAvWrap->listItem);
	QListSetObj(&pAvWrap->listItem, pAvWrap);
	pAvWrap->avHandle = avHandle;
	AtomicWrite(&pAvWrap->refCount, 0);
	QListInsertHead(&pTracker->avList, &pAvWrap->listItem);
done:
	_DBG_LEAVE_FUNC();
	return pAvWrap;
}

/*++

Routine Description
	Clear current AV for tracker

Input:
	pTracker:	AV tracker object to act on

Output:
	none

Additional Information:
	If there is no current AV, this is a noop
	Reference count of current AV is decremented, if 0 its AV is destroyed
	Otherwise its will not be destroyed until a future IbAvTrackerDecRef

--*/
void
IbAvTrackerClearCurrent(
	IN AV_TRACKER	*pTracker
	)
{
	AV_WRAP *currentAV;

	_DBG_ENTER_FUNC(IbAvTrackerClearCurrent);
	SpinLockAcquire(&pTracker->lock);
	currentAV = pTracker->currentAV;
	if (! currentAV)
		goto unlock;	/* noop */
	pTracker->currentAV = NULL;
	if (AtomicDecrement(&currentAV->refCount))
		goto unlock;	/* leave in list, still has references */
	/* last reference removed, destroy AV and wrapper */
	QListRemoveItem(&pTracker->avList, &currentAV->listItem);
	IbAvTrackerDestroyAV(currentAV);
unlock:
	SpinLockRelease(&pTracker->lock);
	_DBG_LEAVE_FUNC();
	return;
}

/*++

Routine Description
	Set current AV for tracker

Input:
	pTracker:	AV tracker object to act on
	avHandle:	AV to set as current AV

Output:
	status of the operation

Additional Information:
	If the given AV is not already in the tracker, it is added
	If it is already current AV, this is a noop
	If there is an existing different current AV, it is cleared first

--*/
FSTATUS
IbAvTrackerSetCurrent(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	)
{
	AV_WRAP *currentAV;
	FSTATUS status = FSUCCESS;

	_DBG_ENTER_FUNC(IbAvTrackerSetCurrent);
	ASSERT(avHandle);
	SpinLockAcquire(&pTracker->lock);
	currentAV = pTracker->currentAV;
	if (currentAV) {
		if (currentAV->avHandle == avHandle)
			goto unlock;	/* noop */
		/* clear currentAV */
		if (0 == AtomicDecrement(&currentAV->refCount)) {
			/* last reference removed, destroy AV and wrapper */
			QListRemoveItem(&pTracker->avList, &currentAV->listItem);
			IbAvTrackerDestroyAV(currentAV);
		}
	}
	pTracker->currentAV = IbAvTrackerFindAV(pTracker, avHandle);
	if (! pTracker->currentAV) {
		pTracker->currentAV = IbAvTrackerAddAV(pTracker, avHandle);
		if (! pTracker->currentAV) {
			status = FINSUFFICIENT_MEMORY;
			goto unlock;
		}
	}
	AtomicIncrementVoid(&pTracker->currentAV->refCount);
unlock:
	SpinLockRelease(&pTracker->lock);
	_DBG_LEAVE_FUNC();
	return status;
}

/*++

Routine Description
	increment reference count for an AV in tracker

Input:
	pTracker:	AV tracker object to act on
	avHandle:	AV to increment reference count on

Output:
	status of operation

Additional Information:
	If the given AV is not already in the tracker, this fails

--*/
FSTATUS
IbAvTrackerIncRef(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	)
{
	AV_WRAP *pAvWrap;
	FSTATUS status = FSUCCESS;

	_DBG_ENTER_FUNC(IbAvTrackerIncRef);
	ASSERT(avHandle);
	SpinLockAcquire(&pTracker->lock);
	pAvWrap = IbAvTrackerFindAV(pTracker, avHandle);
	if (! pAvWrap) {
		status = FNOT_FOUND;
		goto unlock;
	}
	AtomicIncrementVoid(&pAvWrap->refCount);
unlock:
	SpinLockRelease(&pTracker->lock);
	_DBG_LEAVE_FUNC();
	return status;
}

/*++

Routine Description
	decrement reference count for an AV in tracker

Input:
	pTracker:	AV tracker object to act on
	avHandle:	AV to decrement reference count on

Output:
	status of operation

Additional Information:
	If the given AV is not already in the tracker, this fails
	If reference count reaches 0, AV is removed from tracker and destroyed

--*/
FSTATUS
IbAvTrackerDecRef(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	)
{
	AV_WRAP *pAvWrap;
	FSTATUS status = FSUCCESS;

	_DBG_ENTER_FUNC(IbAvTrackerDecRef);
	ASSERT(avHandle);
	SpinLockAcquire(&pTracker->lock);
	pAvWrap = IbAvTrackerFindAV(pTracker, avHandle);
	if (! pAvWrap) {
		status = FNOT_FOUND;
		goto unlock;
	}
	if (0 == AtomicDecrement(&pAvWrap->refCount)) {
		/* last reference removed, destroy AV and wrapper */
		ASSERT(pAvWrap != pTracker->currentAV);
		QListRemoveItem(&pTracker->avList, &pAvWrap->listItem);
		IbAvTrackerDestroyAV(pAvWrap);
	}
unlock:
	SpinLockRelease(&pTracker->lock);
	_DBG_LEAVE_FUNC();
	return status;
}

/*++

Routine Description
	subtract from reference count for an AV in tracker

Input:
	pTracker:	AV tracker object to act on
	avHandle:	AV to decrement reference count on
	count:		amount to subtract from reference count

Output:
	status of operation

Additional Information:
	Can be used to aid reference counting when completion coallescing used
	If the given AV is not already in the tracker, this fails
	If reference count reaches 0, AV is removed from tracker and destroyed

--*/
FSTATUS
IbAvTrackerSubtractRef(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle,
	IN uint32		count
	)
{
	AV_WRAP *pAvWrap;
	FSTATUS status = FSUCCESS;

	_DBG_ENTER_FUNC(IbAvTrackerSubtractRef);
	ASSERT(avHandle);
	SpinLockAcquire(&pTracker->lock);
	pAvWrap = IbAvTrackerFindAV(pTracker, avHandle);
	if (! pAvWrap) {
		status = FNOT_FOUND;
		goto unlock;
	}
	if (0 == AtomicSubtract(&pAvWrap->refCount, count)) {
		/* last reference removed, destroy AV and wrapper */
		ASSERT(pAvWrap != pTracker->currentAV);
		QListRemoveItem(&pTracker->avList, &pAvWrap->listItem);
		IbAvTrackerDestroyAV(pAvWrap);
	}
unlock:
	SpinLockRelease(&pTracker->lock);
	_DBG_LEAVE_FUNC();
	return status;
}

/*++

Routine Description
	Av tracker routine which can be used for iba_sd_join_mcgroup callback

Input:
	pContext:	AV tracker object to act on
	Status:		as provided from SD_MULTICAST_CALLBACK
	State:		as provided from SD_MULTICAST_CALLBACK
	pMcMemberRecord:		as provided from SD_MULTICAST_CALLBACK

Output:
	None

Additional Information:
	If the given AV is not already in the tracker, this fails
	If reference count reaches 0, AV is removed from tracker and destroyed

--*/
void
IbAvTrackerMulticastCallback(
    IN void               *pContext,
    IN FSTATUS            Status,
    IN MC_GROUP_STATE     State,
    IN IB_MCMEMBER_RECORD *pMcMemberRecord
	)
{
	AV_TRACKER	*pTracker = (AV_TRACKER*)pContext;

	_DBG_ENTER_FUNC(IbAvTrackerMulticastCallback);
	switch (State) {
	case MC_GROUP_STATE_JOIN_FAILED:
		_DBG_WARN(("Multicast join failed: MGID 0x%016"PRIx64":0x%016"PRIx64": %s (%d)\n",
			pMcMemberRecord->RID.MGID.AsReg64s.H,
			pMcMemberRecord->RID.MGID.AsReg64s.L,
			_DBG_PTR(iba_fstatus_msg(Status)), Status));
		IbAvTrackerClearCurrent(pTracker);
		break;
	case MC_GROUP_STATE_AVAILABLE:
		{
		IB_ADDRESS_VECTOR av;
		IB_HANDLE avHandle;

		_DBG_INFO(("Multicast available: MGID 0x%016"PRIx64":0x%016"PRIx64"\n",
			pMcMemberRecord->RID.MGID.AsReg64s.H,
			pMcMemberRecord->RID.MGID.AsReg64s.L));
		GetAVFromMcMemberRecord(pMcMemberRecord->RID.PortGID.Type.Global.InterfaceID, pMcMemberRecord, &av);
		Status = iba_create_av(pTracker->caHandle, pTracker->pdHandle,
								&av, &avHandle);
		if (Status != FSUCCESS) {
			_DBG_ERROR(("Unable to create AV for MGID 0x%016"PRIx64":0x%016"PRIx64": %s (%d)\n",
				pMcMemberRecord->RID.MGID.AsReg64s.H,
				pMcMemberRecord->RID.MGID.AsReg64s.L,
				_DBG_PTR(iba_fstatus_msg(Status)), Status));
			IbAvTrackerClearCurrent(pTracker);
		} else {
			Status = IbAvTrackerSetCurrent(pTracker, avHandle);
			_DBG_ERROR(("Unable to Set Current AV for MGID 0x%016"PRIx64":0x%016"PRIx64": %s (%d)\n",
				pMcMemberRecord->RID.MGID.AsReg64s.H,
				pMcMemberRecord->RID.MGID.AsReg64s.L,
				_DBG_PTR(iba_fstatus_msg(Status)), Status));
		}
		break;
		}

	case MC_GROUP_STATE_UNAVAILABLE:
		_DBG_INFO(("Multicast unavailable: MGID 0x%016"PRIx64":0x%016"PRIx64"\n",
			pMcMemberRecord->RID.MGID.AsReg64s.H,
			pMcMemberRecord->RID.MGID.AsReg64s.L));
		IbAvTrackerClearCurrent(pTracker);
		break;
	default:
		_DBG_ERROR(("Unexpected multicast callback state: %d\n", State));
	}
	_DBG_LEAVE_FUNC();
}
