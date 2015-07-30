/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_AVTRACKER_H_
#define _IBA_IB_AVTRACKER_H_

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

#include "iba/public/datatypes.h"
#include "iba/public/ilist.h"
#include "iba/public/ispinlock.h"
#include "iba/public/imemory.h"

#if defined (__cplusplus)
extern "C" {
#endif

/* private internal structure */
typedef struct _AV_WRAP {
	LIST_ITEM	listItem;	/* AV_TRACKER.avList entry */
	IB_HANDLE	avHandle;
	ATOMIC_UINT	refCount;
} AV_WRAP;

typedef struct _AV_TRACKER {
	SPIN_LOCK lock;
	AV_WRAP *currentAV;		/* current AV for MC group */
	QUICK_LIST	avList;		/* list of all active AVs in tracker */
	IB_HANDLE	caHandle;	/* open CA to create AVs against */
	IB_HANDLE	pdHandle;	/* open PD to create AVs against */
} AV_TRACKER;

/*++

Routine Description
	zero out fields in AV_TRACKER

Input:
	pTracker:	AV tracker object to initialize

Output:
	None
--*/
extern void
IbAvTrackerInitState(
	IN AV_TRACKER *pTracker
	);

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
extern FSTATUS
IbAvTrackerInit(
	IN AV_TRACKER *pTracker,
	IN IB_HANDLE caHandle,
	IN IB_HANDLE pdHandle
	);

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
extern void
IbAvTrackerDestroy(
	IN AV_TRACKER	*pTracker
	);


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
extern IB_HANDLE
IbAvTrackerGetCurrent(
	IN AV_TRACKER	*pTracker,
	IN boolean		incRef
	);

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
extern void
IbAvTrackerClearCurrent(
	IN AV_TRACKER	*pTracker
	);

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
extern FSTATUS
IbAvTrackerSetCurrent(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	);

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
extern FSTATUS
IbAvTrackerIncRef(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	);

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
extern FSTATUS
IbAvTrackerDecRef(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle
	);

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
extern FSTATUS
IbAvTrackerSubtractRef(
	IN AV_TRACKER	*pTracker,
	IN IB_HANDLE	avHandle,
	IN uint32		count
	);
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
extern void
IbAvTrackerMulticastCallback(
    IN void               *pContext,
    IN FSTATUS            Status,
    IN MC_GROUP_STATE     State,
    IN IB_MCMEMBER_RECORD *pMcMemberRecord
	);

#if defined (__cplusplus)
};
#endif

#endif /* _IBA_IB_AVTRACKER_H_ */
