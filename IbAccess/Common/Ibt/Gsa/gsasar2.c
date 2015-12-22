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


#include "gsamain.h"
#include "ithread.h"
#include "ireaper.h"

//#define VALIDATE_SAR_LIST

#if defined( DBG) ||defined( IB_DEBUG)
#define STATIC
#else
//#define STATIC static	// TBD use STATIC throughout?? 
#define STATIC
#endif

#if defined( DBG) ||defined( IB_DEBUG)
ATOMIC_UINT SarContexts;
ATOMIC_UINT ActiveSarContexts;
#endif

// 1.1 RMPP protocal Segmentation And Reassembly

// TBD -	when to print Atomic_Read(&g_GsaGlobalInfo->SARRecvDropped )

// Note when a transaction gets started but ultimately fails, we do not
// notify the user (typically Subnet Driver).  This lets the normal timeout
// handling treat it the same as a transaction for which no response was
// received.  This permits a backoff time (due to timeout), during which the
// remote note (SA) will have some time to recover from its situation.

//
// Definitions
//

#if defined( DBG) || defined( IB_DEBUG)
extern uint32	RmppRecvDiscardFreq; // enables discard for testing of retries/resend
extern uint32	RmppSendDiscardFreq; // enables discard for testing of retries/resend
#endif
// we advertize at most RmppWindowSize
// when window we have advertized reaches RmppWindowLowWater
// we readvertize RmppWindowSize
// if equal, there is no low water effect
// if both 1, we ack every packet
// Depending on design of sender, the RmppWindowLowWater may have no effect
// many senders will simply post everything on their send Q and hence an
// early ack actually makes more work for them than an ack at RmppWindowSize
extern uint32	RmppWindowSize;
extern uint32	RmppWindowLowWater;
extern uint32	RmppStrict;
extern uint32	RmppMaxReceive;

	// spec does not define max retries, but other equations for total timeout
	// imply a retry count of 8
#define	GSA_SAR_MAX_RETRIES_SEND			8		//our choice

	// RESP_TIMEOUT includes RESP_TIMEOUT_MULT factor as well
	// as packet lifetime
	// BUGBUG - need packet lifetime component in calculation
	// for now using hard coded 2*(1 second value for packet lifetime)
#define GSA_SAR_DEFAULT_RESP_TIMEOUT		(4300+2000)	// ms per IBTA 1.1
#define GSA_SAR_DEFAULT_RESP_TIMEOUT_MULT	20		// ms per IBTA 1.1
#define GSA_SAR_DEFAULT_TOTAL_TIMEOUT		40000	// ms per IBTA 1.1
#define GSA_SAR_RECV_RESP_TIMEOUT_MULT		17		// our choice
	// RESP_TIMEOUT includes RESP_TIMEOUT_MULT factor as well
	// as packet lifetime
	// BUGBUG - need packet lifetime component in calculation
	// for now using hard coded 1 second value for packet lifetime
#define GSA_SAR_RECV_RESP_TIMEOUT			(537+2000)	// ms

#ifdef  ICS_LOGGING
#define _DBG_PRINT_RMPP_HEADERS( LEVEL, TITLE, pRMPP_MAD, REMOTE_LID) \
	do { \
		_DBG_PRINT ((LEVEL), ( \
			TITLE 								\
			"\tClass..........:x%x\n"			\
			"\tRemoteLID......:x%x\n"			\
			"\tRmppType.......:%d\n"			\
			"\tRmppStatus.....:%d\n"			\
			"\tSegmentNum.....:%d\n"			\
			"\tPayLoadLen/WL..:%d\n",			\
			(pRMPP_MAD)->common.MgmtClass, 	\
			(REMOTE_LID),							\
			(pRMPP_MAD)->RmppHdr.RmppType,			\
			(pRMPP_MAD)->RmppHdr.RmppStatus,			\
			(pRMPP_MAD)->RmppHdr.u1.SegmentNum,			\
			(pRMPP_MAD)->RmppHdr.u2.PayloadLen			\
			));													\
		_DBG_PRINT ((LEVEL), ( \
			TITLE 								\
			"\tFlags..........:\n"				\
			"\t\tFirstPkt..:%d\n"				\
			"\t\tLastPkt...:%d\n"				\
			"\t\tActive....:%d\n"				\
			"\t\tRRespTime.:%d\n",				\
			(pRMPP_MAD)->RmppHdr.RmppFlags.s.FirstPkt,		\
			(pRMPP_MAD)->RmppHdr.RmppFlags.s.LastPkt,		\
			(pRMPP_MAD)->RmppHdr.RmppFlags.s.Active,		\
			(pRMPP_MAD)->RmppHdr.RmppFlags.s.RRespTime		\
		)); \
	} while (0)
#else
#define _DBG_PRINT_RMPP_HEADERS( LEVEL, TITLE, pRMPP_MAD, REMOTE_LID) \
	_DBG_PRINT ((LEVEL), ( \
		TITLE 								\
		"\tClass..........:x%x\n"			\
		"\tRemoteLID......:x%x\n"			\
		"\tRmppType.......:%d\n"			\
		"\tRmppStatus.....:%d\n"			\
		"\tSegmentNum.....:%d\n"			\
		"\tPayLoadLen/WL..:%d\n"			\
		"\tFlags..........:\n"				\
		"\t\tFirstPkt..:%d\n"				\
		"\t\tLastPkt...:%d\n"				\
		"\t\tActive....:%d\n"				\
		"\t\tRRespTime.:%d\n",				\
		(pRMPP_MAD)->common.MgmtClass, 	\
		(REMOTE_LID),							\
		(pRMPP_MAD)->RmppHdr.RmppType,			\
		(pRMPP_MAD)->RmppHdr.RmppStatus,			\
		(pRMPP_MAD)->RmppHdr.u1.SegmentNum,			\
		(pRMPP_MAD)->RmppHdr.u2.PayloadLen,			\
		(pRMPP_MAD)->RmppHdr.RmppFlags.s.FirstPkt,		\
		(pRMPP_MAD)->RmppHdr.RmppFlags.s.LastPkt,		\
		(pRMPP_MAD)->RmppHdr.RmppFlags.s.Active,		\
		(pRMPP_MAD)->RmppHdr.RmppFlags.s.RRespTime		\
		))
#endif

// state for an RMPP Context (basically which circle in IBTA 1.1 flow
// diagrams the context is "waiting" in
typedef enum {
		IB_RMPP_STATE_CREATED = 0,		// processing 1st packet
		IB_RMPP_STATE_ACTIVE = 1,		// in fig 176 or 178 circle - main flow
		IB_RMPP_STATE_DONE = 2,			// GotLast, handing up
		IB_RMPP_STATE_TIMEWAIT = 3,		// in fig 175 circle - term flow
		IB_RMPP_STATE_TIMEWAIT_RECV = 4,// in fig 177 circle - recv term flow
		IB_RMPP_STATE_DESTROY = 5		// being destroyed
} RMPP_CONTEXT_STATE;

// RMPP Sender/Receiver context
// for Segmentation and Reassembly of IBTA 1.1 RMPP messages
// Lock Heirachy (in order they can be acquired):
// 		Global Context List Lock
// 		SarContext.Lock
// 		Qp1Lock
typedef struct _GSA_SAR_CONTEXT {
	// Common info
	LIST_ITEM						ListItem;
	SPIN_LOCK						Lock;		// lock
		// key to context list is TID & MgmtClass & (LID or GID)
		// These are invariant once created, hence RmppFindContext
		// can test these without getting Lock
	uint64							TID;		// Client TID
	uint8							MgmtClass;
		// if GID is part of key, RemoteLID=LID_RESERVED
		// if LID is part of key, RemoteGID is 0, LID is valid
	IB_LID							RemoteLID;
	IB_GID							RemoteGID;

	// RMPP Defined protocol state
		// segment number mgmt and windowing
	uint32							WindowFirst;	// WF - sender only
	uint32							WindowLast;		// WL
	union {
		uint32						NextToSend;		// NS - sender only
		uint32						ExpectedSeg;	// ES - receiver only
	};

		// negotiated timeouts
	uint32							RespTimeout;	// ms
	uint32							TotalTimeout;	// ms - receiver only
	uint8							Direction:1;	// present transfer dir
#define IB_SAR_DIR_SENDER 0
#define IB_SAR_DIR_RECEIVER 1
	uint8							IsDoubleSided:1;
	uint8							State:3;		// RMPP_CONTEXT_STATE

	uint32							TotalSegs;	// total no. of segments
	uint32							TotalDataSize;	// size of data

	TIMER							Timer;
	ATOMIC_UINT						RefCount;

	// TBD - pointer to Gsi registered service this transaction is on behalf of
	// for R=0, it will be the requestor of the transaction from the original
	// send
	// for R=1, it will be the class manager
	// we should get RespTimeout for receiver, total transaction timeout, etc
	// from the GSI registered service entry
	
	// TBD - Sender saves Qp1 info on 1st send
	// however receiver looks up per ack sent
	// perhaps receiver could save qp handles here
	// BUGBUG - both sender and receiver are likely to have problems if
	// CA goes away mid transaction.  receiver might be a little better though
	// TBD - hold AV in this structure and free when free context
	// instead of allocate/free per packet sent.

	// TBD - union of sender and receiver context fields below?
	
	// Receiver Context Info		
	IBT_DGRM_ELEMENT				*RecvHead;	// list of good MADs recv'd
	IBT_DGRM_ELEMENT				*RecvTail;	// list of good MADs recv'd

	// Sender Context Info
		// sender determines Qp and builds a WorkRequest during GsiPostSend
		// a single MAD is on SendQ at a time and the WorkRequest
		// is re-used on send completion
		// User will have provided a single send image as:
		// 		MAD HEADER
		// 		RMPP HEADER
		// 		DATA - may be larger than a single packet
		// BUGBUG - TBD - SA_HDR follows RMPP and is not a standard
		// GSA header size
	uint32							RetryCount;	// number of retries w/o ACK
	GSA_POST_SEND_LIST				*PostList;	// from original DoSARSend
	SYS_CALLBACK_ITEM 				*SysCBItem;	// Syscallback item to invoke
												// when send completes
			// where to post our sends
	IB_HANDLE						Qp1Handle;	
	SPIN_LOCK						*QpLock;

	IB_L_KEY						LKey;
	IB_WORK_REQ						WorkRequest;	// prebuilt WQE

	IB_LOCAL_DATASEGMENT			*DataSeg;	// ptr to data segment in WQE
	uintn							DataStart;	// start of complete data
} GSA_SAR_CONTEXT;

//
// Function prototypes.
//

// ---------------------------------------------------------------------------
// Common RMPP Functions

#if defined( DBG) ||defined( IB_DEBUG)
STATIC void AssertInvariants( GSA_SAR_CONTEXT* pSarContext );
#else
static __inline void AssertInvariants( GSA_SAR_CONTEXT* pSarContext ) {}
#endif

GSA_SAR_CONTEXT*
FindSarContext( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	boolean					sender
	);

STATIC
RMPP_STATUS
RmppMadCheck(
	IN	MAD_RMPP				*pMadRmpp
	);

STATIC
void
RmppTimerCallback(
	IN	void					*TimerContext
	);


// ---------------------------------------------------------------------------
// RMPP Receiver

//boolean
//IsRmppMethod(
//	IN	MAD_RMPP				*pMadRmpp
//	);

GSA_SAR_CONTEXT*
RmppReceiverCreateContext( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

FSTATUS
RmppReceiverProcessRecv (
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
RmppReceiverSendAckIfNeeded(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

STATIC
FSTATUS
RmppReceiverGotLast(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
RmppReceiverTerminate(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	RMPP_CONTEXT_STATE		newState,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

STATIC void SendRmppReply(
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	RMPP_HEADER				*pRmppHdr
	);

STATIC void BuildRmppAck(
	IN	RMPP_HEADER				*pRmppHdr,
	IN	uint8					RRespTime,
	IN	uint32					SegmentNum,
	IN	uint32					NewWindowLast
	);

STATIC void BuildRmppStop(
	IN	RMPP_HEADER				*pRmppHdr,
	IN	uint8					RmppStatus
	);

STATIC void BuildRmppAbort(
	IN	RMPP_HEADER				*pRmppHdr,
	IN	uint8					RmppStatus
	);

STATIC void SendRmppAck(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

STATIC void SendRmppStopReply(
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	uint8					RmppStatus
	);

STATIC void SendRmppAbortReply(
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	uint8					RmppStatus
	);

void
RmppReceiverTimerCallback(
	IN	GSA_SAR_CONTEXT			*pSarContext
	);
	
void
RmppReceiverReapContext(
	IN LIST_ITEM *pItem
	);

void
RmppReceiverDestroyContext ( 
	IN GSA_SAR_CONTEXT 			*pSarContext,
	IN boolean					stopTimer
	);

FSTATUS
RemoveSARRecvListEntry(
	IN	LIST_ITEM				*ListItem
	);

// ---------------------------------------------------------------------------
// RMPP Sender
FSTATUS
SARPost(
	IN	GSA_SAR_CONTEXT			*pSarContext
	);

FSTATUS
RmppSenderProcessRecv (
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN MAD_RMPP					*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);


boolean
ResendReqSARSend ( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

boolean
AckReqSARSend( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
SARCloseSend(
	IN	GSA_SAR_CONTEXT			*pSarContext
	);

void
SARSendSegments(
	IN	void					*Key,
	IN	void					*TimerContext
	);

void
RmppSenderTimerCallback(
	IN	GSA_SAR_CONTEXT			*pSarContext
	);

// ---------------------------------------------------------------------------
// Internal functions

#ifdef VALIDATE_SAR_LIST
static
__inline
boolean
IsRecvSarListValid(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	IBT_ELEMENT	*pElement = &pSarContext->RecvHead->Element;
	IBT_BUFFER	*pBufferHdr = pElement->pBufferList;
	MAD_RMPP	*pMadRmpp = (MAD_RMPP*)pBufferHdr->pNextBuffer->pData;
	uint32_t	SegmentCount = 1;
	boolean		result = TRUE;
	
	_DBG_ENTER_LVL ( _DBG_LVL_SAR_INFO, IsRecvSarListValid );
	if (pMadRmpp->RmppHdr.u1.SegmentNum != SegmentCount)
	{
		_DBG_ERROR(("Segment Number Mismatch in First Segment. Found %d\n",
			pMadRmpp->RmppHdr.u1.SegmentNum));
		result = FALSE;
	}	

//	MsgOut("pMadRmpp->RmppHdr.u1.SegmentNum %d, SegmentCount %d\n", pMadRmpp->RmppHdr.u1.SegmentNum, SegmentCount);

	if (pElement->pNextElement == NULL)
	{
		if (pElement != &pSarContext->RecvTail->Element)
		{
			_DBG_ERROR(("Last Element In List Not Equal Tail. Expected %p Found %p\n",
				&pSarContext->RecvTail->Element, pElement));
			result = FALSE;
		}

//		MsgOut("pElement %p, RecvTail->Element %p\n", pElement,  &pSarContext->RecvTail->Element);
	}

	pElement = pElement->pNextElement;
	
	while (pElement)
	{
		SegmentCount++;

		pBufferHdr = pElement->pBufferList->pNextBuffer;
		pMadRmpp = (MAD_RMPP *)pBufferHdr->pData;

		if (pMadRmpp->RmppHdr.u1.SegmentNum != SegmentCount)
		{
			_DBG_ERROR(("Segment Number Mismatch. Expected %d. Found %d\n", 
				SegmentCount, pMadRmpp->RmppHdr.u1.SegmentNum));
			result = FALSE;
		}	
		
//		MsgOut("pMadRmpp->RmppHdr.u1.SegmentNum %d, SegmentCount %d\n", pMadRmpp->RmppHdr.u1.SegmentNum, SegmentCount);

		if (pElement->pNextElement == NULL)
		{
			if (pElement != &pSarContext->RecvTail->Element)
			{
				_DBG_ERROR(("Last Element In List Not Equal Tail. Expected %p Found %p\n",
					&pSarContext->RecvTail->Element, pElement));
				result = FALSE;
			}

//			MsgOut("pElement %p, RecvTail->Element %p\n", pElement,  &pSarContext->RecvTail->Element);
		}
		
		pElement = pElement->pNextElement;
	}

	if (SegmentCount != (pSarContext->ExpectedSeg - 1))
	{
		_DBG_ERROR(("Expected %d segments. Found %d segments\n",  (pSarContext->ExpectedSeg - 1), SegmentCount));
		result = FALSE;
	}		
	
//	MsgOut("Expected %d segments. Found %d segments\n",  (pSarContext->ExpectedSeg - 1), SegmentCount);
	
	_DBG_LEAVE_LVL( _DBG_LVL_SAR_INFO );
	return result;
}
#endif // VALIDATE_SAR_LIST

static
__inline
boolean
SarContextTimerArmed(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	return (pSarContext->State == IB_RMPP_STATE_ACTIVE
		|| pSarContext->State == IB_RMPP_STATE_TIMEWAIT
		|| pSarContext->State == IB_RMPP_STATE_TIMEWAIT_RECV);
}


STATIC
void
IncrementSarContextUsageCount(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	_DBG_ENTER_LVL ( _DBG_LVL_SAR_INFO, IncrementSarContextUsageCount );
	AtomicIncrementVoid( &pSarContext->RefCount );
	_DBG_LEAVE_LVL( _DBG_LVL_SAR_INFO );
}

STATIC
void
DecrementSarContextUsageCount(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	_DBG_ENTER_LVL ( _DBG_LVL_SAR_INFO, DecrementSarContextUsageCount );
	if ( AtomicDecrement( &pSarContext->RefCount ) == 0 )
	{
#if defined(DBG) ||defined( IB_DEBUG)
		AtomicDecrementVoid(&ActiveSarContexts);
		_DBG_PRINT(_DBG_LVL_SAR_INFO,
				("Unused Sar Context %p, total %d, active %d\n", _DBG_PTR(pSarContext),
					 AtomicRead(&SarContexts), AtomicRead(&ActiveSarContexts)));
#endif
		// remove via reaper since we can be in our own timer callback here
		ReaperQueue( &pSarContext->ListItem, RmppReceiverReapContext );
	}
	_DBG_LEAVE_LVL( _DBG_LVL_SAR_INFO );
}

STATIC
boolean
StartSarContextTimer(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	uint32					TimeOutMs
	)
{
	boolean	rval;
	
	_DBG_ENTER_LVL ( _DBG_LVL_SAR_INFO, StartSarContextTimer );
	IncrementSarContextUsageCount( pSarContext );
	_DBG_PRINT(_DBG_LVL_SAR_INFO,
				("Sar Context %p: %d timer start\n",
					 _DBG_PTR(pSarContext), pSarContext->RefCount));
	rval = TimerStart( &pSarContext->Timer, TimeOutMs );
	_DBG_LEAVE_LVL( _DBG_LVL_SAR_INFO );
	
	return ( rval );
}

STATIC
void
StopSarContextTimer(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	_DBG_ENTER_LVL ( _DBG_LVL_SAR_INFO, StopSarContextTimer );
	TimerStop( &pSarContext->Timer );
	if ( !TimerExpired( &pSarContext->Timer ) )
	{
		DecrementSarContextUsageCount( pSarContext );
		_DBG_PRINT(_DBG_LVL_SAR_INFO, ("Sar Context %p: %d timer stopped\n",
				 _DBG_PTR(pSarContext), pSarContext->RefCount));
	}
	_DBG_LEAVE_LVL( _DBG_LVL_SAR_INFO );
}

// ---------------------------------------------------------------------------
// Externally visible common functions used by GSA

// Destroy Contexts for any SAR in progress.
// designed to only be called in GsaUnload
// since RemoveDevice already destroyed QPs, we will not end up with
// new work on the list
void
DestroySARContexts()
{
	GSA_SAR_CONTEXT*	pSarContext;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, DestroySARContexts );

	// destroy sends in progress
	SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock);
	ASSERT(QListHead( &g_GsaGlobalInfo->SARSendList) == NULL);	// no sends yet
	SpinLockRelease( &g_GsaGlobalInfo->SARSendListLock);

	// destroy receives in progress
	SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock);
	while ( NULL!=(pSarContext =
					(GSA_SAR_CONTEXT*)QListHead(&g_GsaGlobalInfo->SARRecvList)))
	{
		_DBG_PRINT (_DBG_LVL_SAR_INFO,( "Destroy TID %"PRIx64"\n", pSarContext->TID ));
		SpinLockAcquire( &pSarContext->Lock );
		if (pSarContext->State == IB_RMPP_STATE_DESTROY)
		{
			// destroy in progress, skip
			SpinLockRelease( &pSarContext->Lock );
			
			// must release list lock so Context can remove itself from list
			SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock);
			
			// let DestroyContext we raced with have some time to complete
			ThreadSuspend(1);
		} else {
			IncrementSarContextUsageCount( pSarContext );
			
			// call with Context lock held (it releases), list lock unheld
			RmppReceiverDestroyContext( pSarContext, TRUE );

			// must release list lock so Context can remove itself from list
			SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock);

			RemoveSARRecvListEntry( &pSarContext->ListItem );
			
			RmppReceiverReapContext( &pSarContext->ListItem );
		}
		SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock);
	}
	SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock);
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// Completion Handlers shared by RMPP Receiver and Sender

// called on GSI Receive Q completion callback when a packet is received which
// is destined to a GSI Client who has requested GSI to do the
// RMPP Segmentation and Assembly
// we assume caller already verified and byte swapped MAD header
// This can be called for RMPP Contexts which are Senders or Receivers
// Returns:
// 		FCOMPLETED - SAR processing for this packet/transaction is done
// 			caller should pass *ppDgrm up to client
// 			Note *ppDgrm could now contain a completed RMPP message or
// 			a single packet which is not a multi-packet RMPP message
// 		FREJECT - packet is invalid and should be discarded by caller
// 		FPENDING - SAR processing has consumed packet
// 			caller should not free nor pass up to client
FSTATUS
ProcessSARRecvCompletion (
	IN	MAD						*pMad,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	FSTATUS						status;
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	MAD_RMPP					*pMadRmpp = (MAD_RMPP*)pMad;
	GSA_SAR_CONTEXT				*pSarContext = NULL;
#if defined( DBG) ||defined( IB_DEBUG)
	static int recv_discard_counter = 0;
#endif

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ProcessSARRecvCompletion );

	pDgrmPrivate			= (IBT_DGRM_ELEMENT_PRIV *)*ppDgrm;

	BSWAP_RMPP_HEADER( &pMadRmpp->RmppHdr );
	// IBTA 1.1 C13-21.1.3, pg 684 if Active == 0 ignore all other
	// RMPP fields (even version)
	if (! pMadRmpp->RmppHdr.RmppFlags.s.Active)
	{
		//	BSWAP_RMPP_HEADER( &pMadRmpp->Sar );	// pass up as swapped
		status = FCOMPLETED;	// Single Packet, pass it on as is
		goto done;
	}

#if defined( DBG) ||defined( IB_DEBUG)
	// discard occasional SAR packet received, will stress retry mechanism
	if (RmppRecvDiscardFreq
		&& ++recv_discard_counter % RmppRecvDiscardFreq == 1)
	{
		_DBG_PRINT_RMPP_HEADERS( _DBG_LVL_WARN, "Test Discarding Recv:\n",
						pMadRmpp, (*ppDgrm)->RemoteLID);
		goto discard;
	}
#endif

	_DBG_PRINT_RMPP_HEADERS( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
		"RMPP Received:\n", pMadRmpp, pDgrmPrivate->DgrmElement.RemoteLID);

	// Overload base, set if it is SAR Receive data for to tell GsiCoalesceDgrm
	// it has a SAR packet sequence to Coalesce
	// if this was RMPP DATA , it will end up on RecvHead and ultimately
	// in GsiCoalesceDgrm (or in TSL doing Coalesce), if it is an RMPP control
	// packet (ACK, ABORT, etc) it will be freed long before Coalesce is called
	pDgrmPrivate->Base = (GSA_POST_SEND_LIST*)1;

	// IBTA 1.1 Figure 174, pg 696 - RMPP Dispatcher
	// We must use the R bit to determine our overall role in transaction
	// (eg. requestor or responder).  This is necessary because TIDs are only
	// unique from a given requestor, so if we are both requestor and
	// responder for a given class, there could be TID duplication between
	// those roles.
	// This is consistent with C15-0.1.18 in IBTA 1.1 which
	// defines that all packets in a given direction shall have the
	// same Method/R bit (it defines it by 2 specific attribute/responses)
		// TBD can sender/receiver tables be moved out of gsamain.h into
		// gsasar[12].c as static globals?  How do we clean up tables if
		// shutdown while transactions are in progress?
		// Note we could be sender or receiver for either role!!!
	if ( MAD_IS_RESPONSE(pMad))
	{
		pSarContext = FindSarContext(pMadRmpp, *ppDgrm, FALSE);	// requestor
		if (pSarContext)
		{
			// pRmppContext->Lock is held
			status = RmppReceiverProcessRecv (pSarContext, pMadRmpp, ppDgrm);
			// pRmppContext->Lock was released in ProcessRecv
			goto done;
		}
	} else {
		pSarContext = FindSarContext(pMadRmpp, *ppDgrm, TRUE);	// responder
		if (pSarContext)
		{
			// pRmppContext->Lock is held
			status = RmppSenderProcessRecv (pSarContext, pMadRmpp, ppDgrm);
			// pRmppContext->Lock was released in ProcessRecv
			goto done;
		}
	}

	// new RMPP context/transaction

	// IBTA 1.1 C13-21.1.4 - If Active=1 => Version must be 1
	// fig 174 Version Check
	if ( RMPP_VERSION != pMadRmpp->RmppHdr.RmppVersion )
	{
		_DBG_ERROR(( "Received Unsupported RMPP Version <%d>!!! <Abort>\n",
					pMadRmpp->RmppHdr.RmppVersion));
		SendRmppAbortReply(*ppDgrm, RMPP_STATUS_UNSUPPORTED_VERSION);
		goto discard;
	}

	// fig 174 Type=DATA & SegNo == 1
	if (RMPP_TYPE_DATA != pMadRmpp->RmppHdr.RmppType
		|| 1 != pMadRmpp->RmppHdr.u1.SegmentNum)
	{
		_DBG_ERROR(("Received Unexpected RMPP Packet <TID=x%"PRIx64", Type=%d, Seg=%d>!!! <Abort>\n",
					pMadRmpp->common.TransactionID,
					pMadRmpp->RmppHdr.RmppType, pMadRmpp->RmppHdr.u1.SegmentNum));
		SendRmppAbortReply(*ppDgrm, RMPP_STATUS_BAD_RMPP_TYPE);
		goto discard;
	}
	// Res Avail?
	// I do this a little out of order, it makes more sense to check
	// we have a valid data packet (Seg=1, Type=1) in Fig 174
	// before we go and interpret the contents
	// Since our resource exhaustion can't be predicted by a remote node
	// this should still be compliant

	// PayloadLen can be 0 or the actual payload Length
	if (pMadRmpp->RmppHdr.u2.PayloadLen > RmppMaxReceive)
	{
		_DBG_ERROR((
				"RMPP Header contains a very large Payload(%d bytes)!\n"
				"GSA Max payload setting (RmppMaxReceive=%d bytes) <Stop>\n",
				pMadRmpp->RmppHdr.u2.PayloadLen,
				RmppMaxReceive));
		SendRmppStopReply(*ppDgrm, RMPP_STATUS_RESOURCES_EXHAUSTED);
		goto discard;
	}

	// fig 174 - allocate receive context
	pSarContext = RmppReceiverCreateContext( pMadRmpp, ppDgrm );
	if (pSarContext == NULL)
	{
		// error message already logged in CreateContext
		SendRmppStopReply(*ppDgrm, RMPP_STATUS_RESOURCES_EXHAUSTED);
		goto discard;
	}

	// enter fig 176 at triangle B
	// pRmppContext->Lock is held
	status = RmppReceiverProcessRecv ( pSarContext, pMadRmpp, ppDgrm);
	// pRmppContext->Lock was released in ProcessRecv

done:
	if ( pSarContext )
	{
		DecrementSarContextUsageCount( pSarContext );
		_DBG_PRINT(_DBG_LVL_SAR_INFO,("Sar Context %p: %d done recv\n",
							 _DBG_PTR(pSarContext), pSarContext->RefCount));
	}
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;

discard:
	AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return FREJECT;
}

// called on GSI Send Q completion callback when a packet is received which
// is destined to a GSI Client who has requested GSI to do the
// RMPP Segmentation and Assembly
// This can be called for RMPP Contexts which are Senders or Receivers
void
ProcessSARSendCompletion (
	IN	MAD						*pMad,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	FSTATUS						status;
	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
	GSA_POST_SEND_LIST		*pPostList;
	MAD_RMPP				*pMadRmpp = (MAD_RMPP*)pMad;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ProcessSARSendCompletion );

	pDgrmPrivate			= (IBT_DGRM_ELEMENT_PRIV *)pDgrm;
	pPostList		= pDgrmPrivate->Base;
	BSWAP_RMPP_HEADER (&pMadRmpp->RmppHdr);

	// Check for RMPP Control messages
	if( pMadRmpp->RmppHdr.RmppType != RMPP_TYPE_DATA)
	{
#if defined( DBG) ||defined( IB_DEBUG)
		_DBG_PRINT_RMPP_HEADERS( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
			"RMPP Control packet send completed\n",
			pMadRmpp, pDgrmPrivate->DgrmElement.RemoteLID);
#endif
		ASSERT ( pDgrmPrivate->AvHandle );
		status = PutAV(
						NULL,
						pDgrmPrivate->AvHandle );

		// Just return to pool
		status = DgrmPoolPut( pPostList->DgrmList );
		goto done;
	}

#if 1
	ASSERT(0);	// send not yet implemented
#else

	// TBD - make this a function in Sender code

	// must have been a send completion on behalf of RMPP Sender
	// The Dgrm came from the Gsi Client's pool so we can check for
	// information about the specific client
	// TBD - how does GSI Deregister handle outstanding sends in the DgrmPool??
	ASSERT(((GSA_SERVICE_CLASS_INFO*)pDgrmPrivate->MemPoolHandle->ServiceHandle)->bSARRequired);

	// TBD - will need to handle case of a Sar Recv of ACK occurring
	// before send completion, especially for final send of a message
	// include a DoubleSided transaction
				
	// BUGBUG If the completion had an error, such as flush for
	// unloading module, we may attempt to send SAR here
	// we should handle this special case and return to
	// user callback with an error indication
	// maybe we are ok, as we will post sends and get failures
	// until SAR is completed.
	// fix Deregister to wait for all send buffers to be returned to
	// pool before DestroyDgrmPool

	_DBG_PRINT_RMPP_HEADERS( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
		"SAR send completed:\n",
		pMadRmpp, pDgrmPrivate->DgrmElement.RemoteLID);

	SysCallbackQueue( 
		pDgrmPrivate->pSarContext->SysCBItem, 
		SARSendSegments,
		pDgrmPrivate->pSarContext, 
		FALSE );
#endif

done:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// ---------------------------------------------------------------------------
// Internal Common RMPP Functions

#if defined( DBG) ||defined( IB_DEBUG)
STATIC void AssertInvariants( GSA_SAR_CONTEXT* pSarContext )
{
	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, AssertInvariants );
	// implicit limitation in protocol of 4 Billion fragments
	// no problem since would need 1024G of RAM to compose such a message
	if (IB_SAR_DIR_SENDER == pSarContext->Direction)
	{
		_DBG_PRINT( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
				("WF=%d WL=%d NS=%d\n",
				pSarContext->WindowFirst,
				pSarContext->WindowLast,
				pSarContext->NextToSend));
		// 1 <= WF <= NS <= WL+1
		// on sender don't track ES
#if 0
		ASSERT(1 <= pSarContext->WindowFirst);
		ASSERT(pSarContext->WindowFirst <= pSarContext->WindowLast);
		ASSERT(pSarContext->WindowFirst <= pSarContext->NextToSend);
		ASSERT(pSarContext->NextToSend <= pSarContext->WindowLast+1);
#else
		if ((! (1 <= pSarContext->WindowFirst))
			|| (! (pSarContext->WindowFirst <= pSarContext->WindowLast))
			|| (! (pSarContext->WindowFirst <= pSarContext->NextToSend))
			|| (! (pSarContext->NextToSend <= pSarContext->WindowLast+1)))
		{
			_DBG_ERROR( ("AssertInvariants: WF=%d WL=%d NS=%d\n",
				pSarContext->WindowFirst,
				pSarContext->WindowLast,
				pSarContext->NextToSend));
		}
#endif
	} else {
		_DBG_PRINT( (_DBG_LVL_CALLBACK + _DBG_LVL_SAR_INFO),
				("WL=%d ES=%d TotalSegs=%d\n",
				pSarContext->WindowLast,
				pSarContext->ExpectedSeg,
				pSarContext->TotalSegs));
		// 1 <= WF <= NS <= WL+1
		// IBTA 1.1 says 1 <= WF <= ES < NS <= WL+1
		// on receiver we don't track WF nor NS so
		// 1 <= ES < WL+1
		// however, we never let WL pass TotalSegs:
		// 		if Total Segs!=0, ES could go 1 beyond TotalSegs hence:
		// 			ES < max(WL+1, TotalSegs+2)
		// 		else
		// 			ES < WL+1
		// which yields
		// 1 <= ES <= max(WL, TotalSegs+1)
#if 0
		ASSERT(1 <= pSarContext->ExpectedSeg);
		ASSERT(pSarContext->ExpectedSeg <= MAX(pSarContext->WindowLast,pSarContext->TotalSegs+1);
			// don't open window beyond total expected
		ASSERT((! pSarContext->TotalSegs)
				|| pSarContext->WindowLast <= pSarContext->TotalSegs);
#else
		if (! (1 <= pSarContext->ExpectedSeg))
		{
			_DBG_ERROR( 
				("AssertInvariants 1<=ES: WL=%d ES=%d TotalSegs=%d\n",
				pSarContext->WindowLast,
				pSarContext->ExpectedSeg,
				pSarContext->TotalSegs));
		}
		if (! (pSarContext->ExpectedSeg <= MAX(pSarContext->WindowLast, pSarContext->TotalSegs+1)))
		{
			_DBG_ERROR( 
				("AssertInvariants ES<=MAX(WL,TS+1): WL=%d ES=%d TotalSegs=%d\n",
				pSarContext->WindowLast,
				pSarContext->ExpectedSeg,
				pSarContext->TotalSegs));
		}
		if (! ((! pSarContext->TotalSegs)
					|| pSarContext->WindowLast <= pSarContext->TotalSegs))
		{
			_DBG_ERROR( 
				("AssertInvariants !TS || WL<=TS: WL=%d ES=%d TotalSegs=%d\n",
				pSarContext->WindowLast,
				pSarContext->ExpectedSeg,
				pSarContext->TotalSegs));
		}
#endif
	}
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}
#endif

// assumes pMadRmpp is MAD header from within pDgrm which we received
// sender - is this a sender or receiver context
// 		BUGBUG - gsamain.h has separate lists and locks for send/recv
// 		worse yet if a peer to peer RMPP protocol has requests initiated
// 		from both nodes at eachother with same TID at same time
// 		Context needs an additional aspect to its key.  Since peer to peer
// 		protocol could be bi-directional, may really need to key off of
// 		R bit and who initiated connection.  For now sender indicates if
// 		we initiated context or remote node did.  This is not properly
// 		implemented yet for the case of a bi-directional request
// 		because sender flag may not be set when we start to receive response.
// 		really need just one list with a flag in context which can be used
// 		to distinguish sense of context.  Ask Lamprey.
// returns NULL if no matching context is found
// if found, returns with pSarContext locked, but global list unlocked
GSA_SAR_CONTEXT*
FindSarContext( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	boolean					sender
	)
{
	GSA_SAR_CONTEXT				*pSarContext;
	LIST_ITEM					*pListNext;
	QUICK_LIST					*pList;
	SPIN_LOCK					*pListLock;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, FindSarContext);

	_DBG_PRINT (_DBG_LVL_SAR_INFO,(
			"Need to find TID x%"PRIx64"\n", 
			pMadRmpp->common.TransactionID ));
	// BUGBUG - HACK for now
	if (sender)
	{
		pListLock = &g_GsaGlobalInfo->SARSendListLock;
		pList = &g_GsaGlobalInfo->SARSendList;
	} else {
		pListLock = &g_GsaGlobalInfo->SARRecvListLock;
		pList = &g_GsaGlobalInfo->SARRecvList;
	}

	SpinLockAcquire( pListLock );

	for ( pListNext = QListHead( pList );
			pListNext != NULL;
			pListNext = QListNext( pList, pListNext ))
	{
		pSarContext = (GSA_SAR_CONTEXT*)pListNext;
		_DBG_PRINT (_DBG_LVL_SAR_INFO,( "...TID %"PRIx64"\n", pSarContext->TID ));

		//
		// IB defines an RMPP context as unique by the tuple:
		//  { MgmtClass, TID, LID } or { MgmtClass, TID, GID }
		//  To be IB Compliant, if a clients on given Node issue multiple 
		//  requests to the same manager at the same time, the TIDs must
		//  be unique.  Otherwise this test will locate an inaccurate Context
		// TBD - BUGBUG - when create a new context lookup in here first and
		// catch duplicates and provide error to local GSA client

		if (pSarContext->MgmtClass == pMadRmpp->common.MgmtClass
			&& pSarContext->TID == pMadRmpp->common.TransactionID
			&& ( pSarContext->RemoteLID == pDgrm->RemoteLID
				|| (pSarContext->RemoteLID == LID_RESERVED
					&& pDgrm->GlobalRoute
					&& 0 == MemoryCompare(&pSarContext->RemoteGID, &pDgrm->GRHInfo.DestGID, sizeof(IB_GID))
			)))
		{
			IncrementSarContextUsageCount( pSarContext );
			_DBG_PRINT(_DBG_LVL_SAR_INFO,("Sar Context %p: %d found\n",
										 _DBG_PTR(pSarContext), pSarContext->RefCount));
			SpinLockRelease(pListLock);
			SpinLockAcquire( &pSarContext->Lock );
			if (pSarContext->State != IB_RMPP_STATE_DESTROY)
			{
				goto found;
			} else {
				// destroy in progress, skip
				SpinLockRelease( &pSarContext->Lock );
				DecrementSarContextUsageCount( pSarContext );
				_DBG_PRINT(_DBG_LVL_SAR_INFO,
									("Sar Context %p: %d found in destroy\n",
										 _DBG_PTR(pSarContext), pSarContext->RefCount));
				goto notfound;
			}
		}
	}
	
	// not found in list
	SpinLockRelease(pListLock);

notfound:
	pSarContext = NULL;
	_DBG_INFO((
		"RMPP Packet: Context Not Found!\n"
			"\tClass........:x%x\n"
			"\tAttributeID..:x%x\n"
			"\tTID..........:x%"PRIx64"\n"
			"\tLID..........:x%x\n"
			"\tGID..........:x%"PRIx64":%"PRIx64"\n",
			pMadRmpp->common.MgmtClass, 
			pMadRmpp->common.AttributeID,
			pMadRmpp->common.TransactionID,
			pDgrm->RemoteLID,
			pDgrm->GlobalRoute?pDgrm->GRHInfo.DestGID.Type.Global.SubnetPrefix:0,
			pDgrm->GlobalRoute?pDgrm->GRHInfo.DestGID.Type.Global.InterfaceID:0
		));
	 _DBG_PRINT_RMPP_HEADERS( _DBG_LVL_INFO,
		"RMPP Packet: (cont)\n", pMadRmpp, pDgrm->RemoteLID);

found:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return pSarContext;
}

// perform basic MAD validation on an RMPP MAD
// returns RMPP_STATUS value, RMPP_STATUS_NORMAL if packet is OK
STATIC
RMPP_STATUS
RmppMadCheck(
	IN	MAD_RMPP				*pMadRmpp
	)
{
	uint32 rmppGsDataSize;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppMadCheck);
	ASSERT(pMadRmpp->RmppHdr.RmppFlags.s.Active);	// RecvCompletion checked this
	if ( RMPP_VERSION != pMadRmpp->RmppHdr.RmppVersion )
	{
		_DBG_ERROR(("Invalid RMPP Version <%d>!!! <Abort>\n",
					pMadRmpp->RmppHdr.RmppVersion));
		return RMPP_STATUS_UNSUPPORTED_VERSION;
	}
	switch (pMadRmpp->RmppHdr.RmppType)
	{
		case RMPP_TYPE_DATA:
			if (RMPP_STATUS_NORMAL != pMadRmpp->RmppHdr.RmppStatus)
			{
				if (RmppStrict)
				{
					_DBG_ERROR(("RMPP DATA packet with Bad Status: %s <%d>!!! <Abort>\n",
								_DBG_PTR(iba_rmpp_status_msg((RMPP_STATUS)pMadRmpp->RmppHdr.RmppStatus)),
								pMadRmpp->RmppHdr.RmppStatus));
					return RMPP_STATUS_ILLEGAL_STATUS;
				} else {
					_DBG_WARN(("RMPP DATA packet with Bad Status: %s <%d>\n",
								_DBG_PTR(iba_rmpp_status_msg((RMPP_STATUS)pMadRmpp->RmppHdr.RmppStatus)),
								pMadRmpp->RmppHdr.RmppStatus));
				}
			}
			if ((pMadRmpp->RmppHdr.u1.SegmentNum == 1)
				!= (pMadRmpp->RmppHdr.RmppFlags.s.FirstPkt == 1))
			{
				_DBG_ERROR(("RMPP First DATA packet with Bad SegmentNum: <%d>!!! <Abort>\n",
								pMadRmpp->RmppHdr.u1.SegmentNum));
				return RMPP_STATUS_INCONSISTENT_FIRST;
			}
			if (pMadRmpp->RmppHdr.u1.SegmentNum == 0)
			{
				_DBG_ERROR(("RMPP DATA packet with Bad SegmentNum: <%d>!!! <Abort>\n",
								pMadRmpp->RmppHdr.u1.SegmentNum));
				return RMPP_STATUS_UNSPECIFIED;	// TBD
			}
			// validate PayloadLen for a First packet
			rmppGsDataSize = (pMadRmpp->common.BaseVersion == IB_BASE_VERSION)
				? IBA_RMPP_GS_DATASIZE : STL_RMPP_GS_DATASIZE;
			if (pMadRmpp->RmppHdr.RmppFlags.s.FirstPkt
				&& ! pMadRmpp->RmppHdr.RmppFlags.s.LastPkt
				&& pMadRmpp->RmppHdr.u2.PayloadLen != 0
				&& pMadRmpp->RmppHdr.u2.PayloadLen <= rmppGsDataSize)
			{
				_DBG_ERROR(("RMPP First DATA packet with Bad PayloadLen: <%d>!!! <Abort>\n",
								pMadRmpp->RmppHdr.u2.PayloadLen));
				return RMPP_STATUS_INCONSISTENT_PAYLOAD_LEN;
			}
			// validate PayloadLen for a Last packet
			if (pMadRmpp->RmppHdr.RmppFlags.s.LastPkt
				&& ( pMadRmpp->RmppHdr.u2.PayloadLen == 0
					|| pMadRmpp->RmppHdr.u2.PayloadLen > rmppGsDataSize))
			{
				_DBG_ERROR(("RMPP Last DATA packet with Bad PayloadLen: <%d>!!! <Abort>\n",
								pMadRmpp->RmppHdr.u2.PayloadLen));
				return RMPP_STATUS_INCONSISTENT_PAYLOAD_LEN;
			}
			break;
		case RMPP_TYPE_ACK:
			if (RmppStrict && RMPP_STATUS_NORMAL != pMadRmpp->RmppHdr.RmppStatus)
			{
				_DBG_ERROR(("RMPP ACK with Bad Status: %s <%d>!!! <Abort>\n",
								_DBG_PTR(iba_rmpp_status_msg((RMPP_STATUS)pMadRmpp->RmppHdr.RmppStatus)),
								pMadRmpp->RmppHdr.RmppStatus));
				return RMPP_STATUS_ILLEGAL_STATUS;
			}
			if (pMadRmpp->RmppHdr.RmppFlags.s.FirstPkt
				|| pMadRmpp->RmppHdr.RmppFlags.s.LastPkt)
			{
				// not sure what to send here, sounds good
				_DBG_ERROR(("RMPP ACK with First or Last!!! <Abort>\n"));
				return RMPP_STATUS_BAD_RMPP_TYPE;
			}
			if (pMadRmpp->RmppHdr.u1.SegmentNum > pMadRmpp->RmppHdr.u2.NewWindowLast)
			{
				_DBG_ERROR(("RMPP ACK with New Window too small: Seg=%d, NWL=%d!!! <Abort>\n",
								pMadRmpp->RmppHdr.u1.SegmentNum,
								pMadRmpp->RmppHdr.u2.NewWindowLast));
				return RMPP_STATUS_NEW_WL_TOO_SMALL;
			}
			break;
		case RMPP_TYPE_STOP:
		case RMPP_TYPE_ABORT:
			// caller should not send an Abort/Stop in response to
			// Abort/Stop, merely returning a non-Normal code should
			// trigger caller to terminate and discard
			if (RMPP_STATUS_NORMAL == pMadRmpp->RmppHdr.RmppStatus)
			{
				_DBG_ERROR(("RMPP %s with Bad Status: %s <%d>!!! <Abort>\n",
								_DBG_PTR((pMadRmpp->RmppHdr.RmppType==RMPP_TYPE_STOP)
								? "STOP":"ABORT"),
								_DBG_PTR(iba_rmpp_status_msg((RMPP_STATUS)pMadRmpp->RmppHdr.RmppStatus)),
								pMadRmpp->RmppHdr.RmppStatus));
				return RMPP_STATUS_ILLEGAL_STATUS;
			}
			if (pMadRmpp->RmppHdr.RmppFlags.s.FirstPkt
				|| pMadRmpp->RmppHdr.RmppFlags.s.LastPkt)
			{
				_DBG_ERROR(("RMPP %s with First or Last!!! <Abort>\n",
								_DBG_PTR((pMadRmpp->RmppHdr.RmppType==RMPP_TYPE_STOP)
								? "STOP":"ABORT")));
				// not sure what to send here, sounds good
				return RMPP_STATUS_BAD_RMPP_TYPE;
			}
			break;
		default:
			_DBG_ERROR(("Invalid RMPP Type <%d>!!! <Abort>\n",
								pMadRmpp->RmppHdr.RmppType));
			return RMPP_STATUS_BAD_RMPP_TYPE;
	}
	if (RMPP_STATUS_START_RESERVED <= pMadRmpp->RmppHdr.RmppStatus
		&& pMadRmpp->RmppHdr.RmppStatus <= RMPP_STATUS_END_RESERVED)
	{
		_DBG_ERROR(("Invalid RMPP Status %s <%d>!!! <Abort>\n",
								_DBG_PTR(iba_rmpp_status_msg((RMPP_STATUS)pMadRmpp->RmppHdr.RmppStatus)),
								pMadRmpp->RmppHdr.RmppStatus));
		return RMPP_STATUS_ILLEGAL_STATUS;
	}
	return RMPP_STATUS_NORMAL;
}

// to support DoubleSided RMPP, we use a common timer
// hence we can use the same context twice
STATIC
void
RmppTimerCallback(
	IN	void					*TimerContext
	)
{
	GSA_SAR_CONTEXT		*pSarContext = (GSA_SAR_CONTEXT*)TimerContext;

	// if context is being destroyed, nothing to do here
	if (pSarContext->State != IB_RMPP_STATE_DESTROY)
	{
		if (IB_SAR_DIR_SENDER == pSarContext->Direction)
		{
			RmppSenderTimerCallback(pSarContext);
		} else {	// receiver
			RmppReceiverTimerCallback(pSarContext);
		}
	}
}
	

// ---------------------------------------------------------------------------
// RMPP Receiver

//
// Create RMPP Context for a Receiver
//
// Returns:
// 		NULL - unable to create context
// 		! NULL - a created context which is locked
GSA_SAR_CONTEXT*
RmppReceiverCreateContext( 
	IN MAD_RMPP					*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	GSA_SAR_CONTEXT				*pSarContext;
	IBT_DGRM_ELEMENT			*pDgrm = *ppDgrm;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppReceiverCreateContext);

	ASSERT(RMPP_TYPE_DATA == pMadRmpp->RmppHdr.RmppType);	// caller checked
	ASSERT(1 == pMadRmpp->RmppHdr.u1.SegmentNum);				// caller checked
	ASSERT(pMadRmpp->RmppHdr.RmppFlags.s.Active);				// caller checked

	// Looks sane, lets create the context now
	pSarContext = (GSA_SAR_CONTEXT*)MemoryAllocateAndClear(sizeof(GSA_SAR_CONTEXT), FALSE, GSA_MEM_TAG);
	if ( NULL == pSarContext )
	{
		_DBG_ERROR (( "Not enough memory for RMPP Context! <Stop>\n"));
		goto failedalloc;
	}

	SpinLockInitState(&pSarContext->Lock);
	TimerInitState(&pSarContext->Timer);
	if ( TRUE != SpinLockInit( &pSarContext->Lock ))
	{
		_DBG_ERROR (( "RMPP Lock Init failed! <Stop>\n" ));
		goto failedlockinit;
	}
	if ( TRUE != TimerInit( &pSarContext->Timer, RmppTimerCallback,
							pSarContext ))
	{
		_DBG_ERROR (( "RMPP Timer Init failed! <Stop>\n" ));
		goto failedtimerinit;
	}

	pSarContext->TID		= pMadRmpp->common.TransactionID;
	pSarContext->MgmtClass	= pMadRmpp->common.MgmtClass;
	if (pDgrm->GlobalRoute)
	{
		pSarContext->RemoteLID 	= LID_RESERVED;
		pSarContext->RemoteGID 	= pDgrm->GRHInfo.DestGID;
	} else {
		pSarContext->RemoteLID 	= pDgrm->RemoteLID;
		MemoryClear(&pSarContext->RemoteGID, sizeof(pSarContext->RemoteGID));
	}
	pSarContext->ExpectedSeg= 1;
	pSarContext->WindowLast	= 1;
	// receiver controls the RRespTime.  It is how fast sender
	// can expect us to turnaround ACKs
	// use our default
	pSarContext->RespTimeout = GSA_SAR_DEFAULT_RESP_TIMEOUT;
	pSarContext->TotalTimeout = GSA_SAR_DEFAULT_TOTAL_TIMEOUT;
	pSarContext->Direction = IB_SAR_DIR_RECEIVER;
	pSarContext->IsDoubleSided = 0;	// BUGBUG but ok for SA responses
	pSarContext->State= IB_RMPP_STATE_CREATED; // processing 1st packet
	AtomicWrite( &pSarContext->RefCount, 0 );

	// Add to Context to Global SAR recv list
	SpinLockAcquire( &g_GsaGlobalInfo->SARRecvListLock );
	QListInsertHead( &g_GsaGlobalInfo->SARRecvList, &pSarContext->ListItem );
	IncrementSarContextUsageCount( pSarContext );
	_DBG_PRINT(_DBG_LVL_SAR_INFO,("Sar Context %p: %d add to list\n",
							 _DBG_PTR(pSarContext), pSarContext->RefCount));
	SpinLockRelease( &g_GsaGlobalInfo->SARRecvListLock );

	AssertInvariants(pSarContext);

	SpinLockAcquire(&pSarContext->Lock);

	IncrementSarContextUsageCount( pSarContext );
	_DBG_PRINT(_DBG_LVL_SAR_INFO,("Sar Context %p: %d create ret\n",
							 _DBG_PTR(pSarContext), pSarContext->RefCount));

#if defined( DBG) ||defined( IB_DEBUG)
	AtomicIncrement(&SarContexts);
	AtomicIncrement(&ActiveSarContexts);
	_DBG_PRINT(_DBG_LVL_SAR_INFO,
				("Created Sar Context %p, total %d active %d\n", _DBG_PTR(pSarContext),
					 AtomicRead(&SarContexts), AtomicRead(&ActiveSarContexts)));
#endif
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return pSarContext;

	// failures during creation of Context
	// cleanup partially created context and send a Stop
failedtimerinit:
	SpinLockDestroy( &pSarContext->Lock );
failedlockinit:
	MemoryDeallocate ( pSarContext );
failedalloc:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return NULL;
}

// called with pSarContext Lock held, will unlock before returns
// Returns:
// 		FCOMPLETED - SAR processing for this packet/transaction is done
// 			caller should pass *ppDgrm up to client
// 			Note *ppDgrm could now contain a completed RMPP message or
// 			a single packet which is not a multi-packet RMPP message
// 		FREJECT - packet is invalid and should be discarded by caller
// 		FPENDING - SAR processing has consumed packet
// 			caller should not free nor pass up to client
FSTATUS
RmppReceiverProcessRecv (
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN MAD_RMPP					*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	IBT_DGRM_ELEMENT			*pDgrm;
	FSTATUS						status;
	RMPP_STATUS					RmppStatus;
	uint32						rmppGsDataSize;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppReceiverProcessRecv);

	pDgrm = *ppDgrm;

	// FindSarContext should have checked for Destroy
	ASSERT(pSarContext->State != IB_RMPP_STATE_DESTROY);

	AssertInvariants(pSarContext);
	// Mad OK fig 176 and fig 177 after get packet
	if (pSarContext->RecvHead)
	{
		// verify Common Mad Header consistent with first mad in transaction
		MAD_RMPP *pFirstMadRmpp = \
			(MAD_RMPP *)GsiDgrmGetRecvMad(pSarContext->RecvHead);
		ASSERT(pFirstMadRmpp->common.MgmtClass == pMadRmpp->common.MgmtClass);
		ASSERT(pFirstMadRmpp->common.TransactionID == pMadRmpp->common.TransactionID);
		if (pFirstMadRmpp->common.BaseVersion != pMadRmpp->common.BaseVersion
			|| pFirstMadRmpp->common.ClassVersion != pMadRmpp->common.ClassVersion
			|| pFirstMadRmpp->common.mr.s.Method != pMadRmpp->common.mr.s.Method
			|| pFirstMadRmpp->common.mr.s.R != pMadRmpp->common.mr.s.R
				// C15-0.1.18
				// Attribute contents in all but DATA packets shall be
				// ignored.
			|| (pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_DATA
				&& (pFirstMadRmpp->common.AttributeID!= pMadRmpp->common.AttributeID
					|| pFirstMadRmpp->common.AttributeModifier!= pMadRmpp->common.AttributeModifier))
			// don't check Status - TBD
		   )
		{
			if (RmppStrict)
			{
				_DBG_ERROR(("Received RMPP with inconsistent Mad Header!!! <Abort>\n"));
				SendRmppAbortReply(*ppDgrm, RMPP_STATUS_UNSPECIFIED);
				status = FREJECT;
				goto terminate;
			} else {
				_DBG_WARN(("Received RMPP with inconsistent Mad Header!!! <using First>\n"));
			}
		}
		// not sure how to do this, we use first failure status as overall
		// mad status
		if (pMadRmpp->common.u.NS.Status.AsReg16 != MAD_STATUS_SUCCESS
			&& pFirstMadRmpp->common.u.NS.Status.AsReg16 == MAD_STATUS_SUCCESS)
		{
			pFirstMadRmpp->common.u.NS.Status.AsReg16 = \
				pMadRmpp->common.u.NS.Status.AsReg16;
			_DBG_WARN(("Received RMPP with different Mad Status than original Mad Header!!! <Using First Error>\n"));
		}
	}
	RmppStatus = RmppMadCheck(pMadRmpp);
	if (RMPP_STATUS_NORMAL != RmppStatus)
	{
		// MadCheck already logged error message
		SendRmppAbortReply(*ppDgrm, RmppStatus);
		status = FREJECT;
		goto terminate;
	}
	if (pSarContext->State == IB_RMPP_STATE_TIMEWAIT)
	{
		// were in wait circle on fig 175
		goto discard;
	} else if (pSarContext->State == IB_RMPP_STATE_TIMEWAIT_RECV) {
		// were in wait circle on fig 177
		if (pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_DATA)
		{
			status = FREJECT;	// discard
			// ReceiverTerminate releases lock
			RmppReceiverTerminate(pSarContext, IB_RMPP_STATE_TIMEWAIT_RECV, pDgrm);
			AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );
			goto done_lockreleased;
		} else if (pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_ACK) {
			if (pSarContext->IsDoubleSided) {
				pSarContext->IsDoubleSided = FALSE;
				// TBD - initialize sender context setting WF from packet
				ASSERT(0);	// sender not yet implemented
				status = FREJECT;
				goto terminate;
			} else {
				_DBG_ERROR(("Received Bad RMPP Type <ACK> for Receiver TimeWait!!! <Abort>\n"));
				SendRmppAbortReply(*ppDgrm, RMPP_STATUS_BAD_RMPP_TYPE);
				status = FREJECT;
				goto terminate;
			}
		} else if (pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_STOP	// errata 3735
				|| pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_ABORT) {
			status = FREJECT;
			goto terminate;
		} else {
			_DBG_ERROR(("Received Bad RMPP Type <%d> for Receiver TimeWait!!! <Abort>\n",
							pMadRmpp->RmppHdr.RmppType));
			SendRmppAbortReply(*ppDgrm, RMPP_STATUS_BAD_RMPP_TYPE);
			status = FREJECT;
			goto terminate;
		}
	} else {
		// were in wait circle on fig 176
		if (pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_ABORT
			|| pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_STOP)
		{
			_DBG_ERROR(("RMPP Transaction x%"PRIx64" %s: RMPP Status: %s <%d>!!!\n",
				pMadRmpp->common.TransactionID,
				_DBG_PTR((pMadRmpp->RmppHdr.RmppType==RMPP_TYPE_ABORT?"Aborted":"Stopped")),
				_DBG_PTR(iba_rmpp_status_msg((RMPP_STATUS)pMadRmpp->RmppHdr.RmppStatus)),
				pMadRmpp->RmppHdr.RmppStatus
				));

			status = FREJECT;
			goto terminate;
		} else if (pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_ACK) {
			goto discard;
		} else {
			ASSERT(pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_DATA);
			if (pMadRmpp->RmppHdr.u1.SegmentNum != pSarContext->ExpectedSeg)
			{
				SendRmppAck(pSarContext, pDgrm);
				goto discard;
			}
			_DBG_PRINT(_DBG_LVL_SAR_INFO, ("Got expected segment x%"PRIx64":%d\n",
				pMadRmpp->common.TransactionID,
				pMadRmpp->RmppHdr.u1.SegmentNum));
			if (pMadRmpp->RmppHdr.u1.SegmentNum == 1)
			{
				// PayloadLen processing
					// PayloadLen is 0 or exact payload size
					// TBD - a ROUNDUP function
	rmppGsDataSize = (pMadRmpp->common.BaseVersion == IB_BASE_VERSION)
		? IBA_RMPP_GS_DATASIZE : STL_RMPP_GS_DATASIZE;
				pSarContext->TotalSegs	=
						(pMadRmpp->RmppHdr.u2.PayloadLen +rmppGsDataSize -1)
							/ rmppGsDataSize;
				pSarContext->TotalDataSize= pMadRmpp->RmppHdr.u2.PayloadLen;

				_DBG_PRINT (_DBG_LVL_SAR_INFO,( "RMPP Payload Length = %d bytes\n",
						pSarContext->TotalDataSize ));
				// compute TTIME and set wait timer
				// BUGBUG TBD need to get this from class registration
				// if PacketLifetimes + RespTimes >= 1 second
				// the equation on pg 691 quickly exceeds 320 sec
				// so we just use the default
				// The equation on pg 691 is problematic since we don't know
				// packet lifetimes for the path (that's why we are doing SA
				// query in the first place!!!)
				pSarContext->TotalTimeout = GSA_SAR_DEFAULT_TOTAL_TIMEOUT;
				ASSERT(pSarContext->State == IB_RMPP_STATE_CREATED);
				if( !StartSarContextTimer( pSarContext, pSarContext->TotalTimeout))
				{
					_DBG_ERROR (( "Error in Timer Start! <Stop>\n"));
					SendRmppStopReply(pDgrm, RMPP_STATUS_RESOURCES_EXHAUSTED);
					status = FREJECT;
					goto terminate;
				}
				pSarContext->State= IB_RMPP_STATE_ACTIVE; // in fig 176 Wait
				_DBG_PRINT(_DBG_LVL_SAR_INFO,
						("Sar Context %p: started total timer, state=%d\n",
									 _DBG_PTR(pSarContext), pSarContext->State));
				// Store Data -> first packet in message
				pSarContext->RecvHead	= pSarContext->RecvTail = pDgrm;
			} else {
				ASSERT(pSarContext->RecvTail);
				ASSERT(pDgrm->Element.pNextElement == NULL);
				if (pSarContext->RecvTail->Element.pNextElement != NULL
					|| ((MAD_RMPP *)GsiDgrmGetRecvMad(pSarContext->RecvTail))->RmppHdr.u1.SegmentNum != (pSarContext->ExpectedSeg-1))
				{
					_DBG_ERROR (("Datagram list corrupted pSarContext %p SegmentNum %d ExpectedSeg %d pNextElement %p\n",
						_DBG_PTR(pSarContext), ((MAD_RMPP *)GsiDgrmGetRecvMad(pSarContext->RecvTail))->RmppHdr.u1.SegmentNum,
						pSarContext->ExpectedSeg, _DBG_PTR(pSarContext->RecvTail->Element.pNextElement)));
					DumpDgrmElement(pSarContext->RecvHead);
					SendRmppStopReply(pDgrm, RMPP_STATUS_RESOURCES_EXHAUSTED);
					status = FREJECT;
					goto terminate;
				}
				// Store Data -> additional packet in message
				pSarContext->RecvTail->Element.pNextElement = (IBT_ELEMENT *)pDgrm;
				pSarContext->RecvTail = pDgrm;
				pSarContext->RecvHead->TotalRecvSize += pDgrm->Element.RecvByteCount;
			}
			// ES++
			++pSarContext->ExpectedSeg;
			status = FPENDING;

#ifdef VALIDATE_SAR_LIST
//			MsgOut("SarContext is %svalid\n", IsRecvSarListValid(pSarContext)==TRUE ? "" : "NOT ");
			if (IsRecvSarListValid(pSarContext) == FALSE)
			{
				_DBG_ERROR(("SarContext is NOT valid\n"));
			}
#endif // VALIDATE_SAR_LIST

			if (pMadRmpp->RmppHdr.RmppFlags.s.LastPkt)
			{
				status = RmppReceiverGotLast(pSarContext, pMadRmpp, ppDgrm);
				// since we stored the data above, on error we return FPENDING
				// pDgrm will be freed when we destroy the Context
				ASSERT(status != FREJECT);
				goto done_lockreleased;
			}

			// check if we already got as many as expected
			if (pSarContext->TotalSegs != 0
				&& pSarContext->ExpectedSeg > pSarContext->TotalSegs)
			{
				_DBG_ERROR(("Received non-Last Segment when expected Last <%d>!!! <Abort>\n",
							pMadRmpp->RmppHdr.u1.SegmentNum));
				SendRmppAbortReply(*ppDgrm, RMPP_STATUS_INCONSISTENT_PAYLOAD_LEN);
				// since we stored the data above, on error we return FPENDING
				// pDgrm will be freed when we destroy the Context
				status = FPENDING;
				goto terminate;
			}
			RmppReceiverSendAckIfNeeded(pSarContext, pDgrm);
			goto done;
		}
	}
done:
	SpinLockRelease(&pSarContext->Lock);
done_lockreleased:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;

discard:
	SpinLockRelease(&pSarContext->Lock);
	AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return FREJECT;

terminate:
	// ReceiverTerminate releases lock
	RmppReceiverTerminate(pSarContext, IB_RMPP_STATE_TIMEWAIT, pDgrm);
	AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
}	

// received last packet
// called in RecvCompletionCallback for
// Last Pkt?=Yes at bottom of Fig 176
// called with pSarContext Lock held, will unlock before returns
// returns:
//	FPENDING - premature Last, we will terminate SAR context
//	FCOMPLETED - good completion, *ppDgrm is list of all response data packets
STATIC
FSTATUS
RmppReceiverGotLast(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	uint32 rmppGsDataSize;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppReceiverGotLast);

	// check if got less than expected
	if (pSarContext->TotalSegs != 0
		&& pMadRmpp->RmppHdr.u1.SegmentNum != pSarContext->TotalSegs)
	{
		_DBG_ERROR(("Received Last Segment prematurely <%d>!!! <Abort>\n",
							pMadRmpp->RmppHdr.u1.SegmentNum));
		SendRmppAbortReply(*ppDgrm, RMPP_STATUS_INCONSISTENT_PAYLOAD_LEN);
		goto terminate;
	}

	// special Store Data processing for Last packet
	// 	- prep for delivery to client
	ASSERT(pMadRmpp->RmppHdr.RmppFlags.s.LastPkt);
	ASSERT(pMadRmpp->RmppHdr.RmppType == RMPP_TYPE_DATA);
	ASSERT(pSarContext->State == IB_RMPP_STATE_ACTIVE);
	StopSarContextTimer( pSarContext );	// transaction done, stop TotalTimeout
	pSarContext->State= IB_RMPP_STATE_DONE; // handing up message

	// compute overall payload size and put in first for use by client
		// last indicates amount of data in itself
	rmppGsDataSize = (pMadRmpp->common.BaseVersion == IB_BASE_VERSION)
		? IBA_RMPP_GS_DATASIZE : STL_RMPP_GS_DATASIZE;
	((MAD_RMPP *)GsiDgrmGetRecvMad(pSarContext->RecvHead))->RmppHdr.u2.PayloadLen =
				(pMadRmpp->RmppHdr.u1.SegmentNum-1) * rmppGsDataSize
				+ pMadRmpp->RmppHdr.u2.PayloadLen;

	// Return all segments to the user
	ASSERT(pSarContext->RecvHead);
	*ppDgrm = pSarContext->RecvHead;

	// delink from SAR Context so termination won't affect
	pSarContext->RecvHead = pSarContext->RecvTail = NULL;

	// ReceiverTerminate releases lock
	RmppReceiverTerminate(pSarContext, IB_RMPP_STATE_TIMEWAIT_RECV, *ppDgrm);

	// caller will Invoke the user callback for the completed datagram.
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return FCOMPLETED;

terminate:
	// ReceiverTerminate releases lock
	RmppReceiverTerminate(pSarContext, IB_RMPP_STATE_TIMEWAIT, *ppDgrm);
	AtomicIncrementVoid( &g_GsaGlobalInfo->SARRecvDropped );
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return FPENDING;
}

// handles the Time To Ack? block in fig 176
// pDgrm is used as template for ACK, but is unmodified
void
RmppReceiverSendAckIfNeeded(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppReceiverSendAckIfNeeded);
	if (pSarContext->ExpectedSeg > pSarContext->WindowLast)
	{
		// we must advance window
		pSarContext->WindowLast = pSarContext->ExpectedSeg + RmppWindowSize-1;
	} else if (pSarContext->WindowLast - (pSarContext->ExpectedSeg-1) >= RmppWindowLowWater)
	{
		// to keep things flowing, once we get to low water of window size
		// we advance windowsize
		pSarContext->WindowLast = pSarContext->ExpectedSeg + RmppWindowSize-1;
	} else {
		_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
		return;
	}
	// don't let window go beyond last, may confuse sender
	// before this test, caller should have checked for the error of
	// !Last but beyond TotalSegs,
	// so we know cutting back will still yield a valid WindowLast
	if (pSarContext->TotalSegs
		&& pSarContext->WindowLast > pSarContext->TotalSegs)
	{
		pSarContext->WindowLast = pSarContext->TotalSegs;
	}
	AssertInvariants(pSarContext);
	SendRmppAck(pSarContext, pDgrm);
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// called to terminate a Receiver Transaction by moving to the specified
// TIMEWAIT or TIMEWAIT_RECV state
// must be called with pSarContext Lock held, will unlock before returns
// (and possibly destroy the context)
// caller should not access context after making this call
// pDgrm is a template for a possible Stop reply if problems occur
void
RmppReceiverTerminate(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	RMPP_CONTEXT_STATE		newState,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	boolean timerArmed;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppReceiverTerminate);
	ASSERT(newState == IB_RMPP_STATE_TIMEWAIT
			|| newState == IB_RMPP_STATE_TIMEWAIT_RECV);
	timerArmed = SarContextTimerArmed(pSarContext);
	_DBG_PRINT(_DBG_LVL_SAR_INFO,
				("Sar Context %p: ReceiverTerminate, state=%d, timerArmed=%d\n",
								 _DBG_PTR(pSarContext), pSarContext->State, timerArmed));
	pSarContext->State = newState;
	if (newState == IB_RMPP_STATE_TIMEWAIT_RECV)
	{
		// top of fig 177
		SendRmppAck(pSarContext, pDgrm);
	} else {
		// top of fig 175
	}

	if (timerArmed) {
		// no worry about timer removing last reference, we should also
		// still be on RecvList
		StopSarContextTimer( pSarContext );
	}

	// Return any queued datagram segments to the datagram pool.
	if (pSarContext->RecvHead)
	{
		DgrmPoolPut( pSarContext->RecvHead );
		pSarContext->RecvHead = pSarContext->RecvTail = NULL;
	}

	// wait timer <- Resp
	if( !StartSarContextTimer( pSarContext, pSarContext->RespTimeout))
	{
		// we send stop, if sender is done, this will be benign
		// if sender never got our ack, they will know we have destroyed context
		_DBG_ERROR (( "Error in Timer Start! <Stop>\n"));
		SendRmppStopReply(pDgrm, RMPP_STATUS_RESOURCES_EXHAUSTED);
		RmppReceiverDestroyContext( pSarContext, FALSE);
		RemoveSARRecvListEntry( &pSarContext->ListItem );
	} else {
		SpinLockRelease(&pSarContext->Lock);
	}
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// Timer callback for RMPP Receiver Timeouts
void
RmppReceiverTimerCallback(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, RmppReceiverTimerCallback);

	SpinLockAcquire(&pSarContext->Lock);

	if (TimerCallbackStopped(&pSarContext->Timer))
	{
		// stale timer callback
		SpinLockRelease(&pSarContext->Lock);
		goto done;
	}

	if (pSarContext->State == IB_RMPP_STATE_DESTROY)
	{
		_DBG_INFO(("RMPP Receiver Timeout in destroy State\n"));
		SpinLockRelease(&pSarContext->Lock);
		goto done;
	}
	if (pSarContext->State == IB_RMPP_STATE_TIMEWAIT
		|| pSarContext->State == IB_RMPP_STATE_TIMEWAIT_RECV)
	{
		// in circle in fig 177
		// simple timewait period is done
		// don't assert invariants, could have aborted in a weird state
		// RecvHead could be empty or full depending on when bad one occurred
		_DBG_INFO(("RMPP Receiver TimeWait done\n"));
	} else {
		IBT_DGRM_ELEMENT                *pDgrm = pSarContext->RecvHead;
		// in circle in fig 176
		ASSERT(pSarContext->State == IB_RMPP_STATE_ACTIVE);
		// total transaction timeout
		ASSERT(pSarContext->RecvHead != NULL);  // must have gotten at least 1
		AssertInvariants(pSarContext);
		// use 1st valid Dgrm we received for this transaction as template
		// for Abort.  Only MAD Header and path information is used
		_DBG_ERROR (( "RMPP Total Transaction Timeout <TID x%"PRIx64", %d ms>! <Abort>\n",
								pSarContext->TID, pSarContext->TotalTimeout));
		SendRmppAbortReply(pDgrm, RMPP_STATUS_TOTAL_TIME_TOO_LONG);
	}

	RmppReceiverDestroyContext( pSarContext, FALSE );
	RemoveSARRecvListEntry( &pSarContext->ListItem );
	
done:
	DecrementSarContextUsageCount( pSarContext );
	_DBG_PRINT(_DBG_LVL_SAR_INFO, ("Sar Context %p: %d timeout\n",
				 _DBG_PTR(pSarContext), pSarContext->RefCount));
    _DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

void
RmppReceiverReapContext(
	IN LIST_ITEM *pItem
	)
{
	GSA_SAR_CONTEXT	*pSarContext = PARENT_STRUCT(pItem, GSA_SAR_CONTEXT, ListItem);
	
	// Return any queued datagram segments to the datagram pool.
	if (pSarContext->RecvHead)
	{
		DgrmPoolPut( pSarContext->RecvHead );
		pSarContext->RecvHead = pSarContext->RecvTail = NULL;
	}
	
	SpinLockDestroy( &pSarContext->Lock );
	TimerDestroy( &pSarContext->Timer );
#if defined( DBG) || defined(IB_DEBUG)
	AtomicDecrement(&SarContexts);
	_DBG_PRINT(_DBG_LVL_SAR_INFO,
			("Reaped Sar Context %p, total %d, active %d\n", _DBG_PTR(pSarContext),
				 AtomicRead(&SarContexts), AtomicRead(&ActiveSarContexts)));
#endif
	MemoryDeallocate( pSarContext );
}

void
RmppReceiverDestroyContext( 
	IN GSA_SAR_CONTEXT 	*pSarContext,
	IN boolean			stopTimer
	)
{
	boolean timerArmed;

	// tell RmppFindContext and RmppTimerCallback to ignore this context
	ASSERT(pSarContext->State != IB_RMPP_STATE_DESTROY);
	timerArmed = SarContextTimerArmed(pSarContext);
	pSarContext->State = IB_RMPP_STATE_DESTROY;
	// Return any queued datagram segments to the datagram pool.
	if (pSarContext->RecvHead)
	{
		DgrmPoolPut( pSarContext->RecvHead );
		pSarContext->RecvHead = pSarContext->RecvTail = NULL;
	}

	SpinLockRelease(&pSarContext->Lock);
	// since searches get list lock then sar lock, if a search finds us now
	// we will be marked as destroy, so they will do nothing.  however
	// since we get list lock below to remove from list, we know search is
	// done with us before we proceed to destroy ourselves

	if (stopTimer && timerArmed)
		StopSarContextTimer( pSarContext );
	// Stop should have ensured timer callback will not be called from here on
}

//
// Remove from Global SAR recv Q
//
FSTATUS
RemoveSARRecvListEntry(
	IN	LIST_ITEM					*pItem
	)
{
    FSTATUS							status = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_SAR_INFO, RemoveSARRecvListEntry);

	SpinLockAcquire(&g_GsaGlobalInfo->SARRecvListLock);
	ASSERT (QListIsItemInList( &g_GsaGlobalInfo->SARRecvList, pItem ));
	if ( !QListIsItemInList( &g_GsaGlobalInfo->SARRecvList, pItem ))
	{
		status						= FINVALID_PARAMETER;
		_DBG_ERROR(("Item not in List!!!\n"));
	} else {
		GSA_SAR_CONTEXT	*pSarContext = PARENT_STRUCT(pItem, GSA_SAR_CONTEXT, ListItem);

		QListRemoveItem ( &g_GsaGlobalInfo->SARRecvList, pItem );

		// Decrement reference to context
		DecrementSarContextUsageCount( pSarContext );
		_DBG_PRINT(_DBG_LVL_SAR_INFO,("Sar Context %p: %d remove from list\n",
						 _DBG_PTR(pSarContext), pSarContext->RefCount));
	}
	SpinLockRelease(&g_GsaGlobalInfo->SARRecvListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
}


// called with pSarContext->Lock held, must release first
#if 0
boolean
IsRmppMethod(
	IN MAD_RMPP				*pMadRmpp
	)
{
	switch (pMadRmpp->common.mr.AsReg8)
	{
	default:
		return FALSE;
		break;		// do nothing

		BUGBUG - fix this, also use #defines
	case 0x92:		// SubnAdminGetTableResp()
	case 0x14:		// SubnAdminGetMulti()
	case 0x94:		// SubnAdminGetMultiResp()

		return TRUE;
		break;
	}
}
#endif

STATIC void BuildRmppAck(
	IN	RMPP_HEADER				*pRmppHdr,
	IN	uint8					RRespTime,
	IN	uint32					SegmentNum,
	IN	uint32					NewWindowLast
	)
{
	MemoryClear(pRmppHdr, sizeof(*pRmppHdr));
	pRmppHdr->RmppVersion = RMPP_VERSION;
	pRmppHdr->RmppType = RMPP_TYPE_ACK;
	pRmppHdr->RmppFlags.s.RRespTime = RRespTime;
	pRmppHdr->RmppFlags.s.Active = 1;
	pRmppHdr->RmppStatus = RMPP_STATUS_NORMAL;
	pRmppHdr->u1.SegmentNum = SegmentNum;
	pRmppHdr->u2.NewWindowLast = NewWindowLast;
}

STATIC void BuildRmppStop(
	IN	RMPP_HEADER				*pRmppHdr,
	IN	uint8					RmppStatus
	)
{
	MemoryClear(pRmppHdr, sizeof(*pRmppHdr));
	pRmppHdr->RmppVersion = RMPP_VERSION;
	pRmppHdr->RmppType = RMPP_TYPE_STOP;
	pRmppHdr->RmppFlags.s.RRespTime = RMPP_RRESPTIME_NONE;
	pRmppHdr->RmppFlags.s.Active = 1;
	pRmppHdr->RmppStatus = RmppStatus;
}

STATIC void BuildRmppAbort(
	IN	RMPP_HEADER				*pRmppHdr,
	IN	uint8					RmppStatus
	)
{
	MemoryClear(pRmppHdr, sizeof(*pRmppHdr));
	pRmppHdr->RmppVersion = RMPP_VERSION;
	pRmppHdr->RmppType = RMPP_TYPE_ABORT;
	pRmppHdr->RmppFlags.s.RRespTime = RMPP_RRESPTIME_NONE;
	pRmppHdr->RmppFlags.s.Active = 1;
	pRmppHdr->RmppStatus = RmppStatus;
}

STATIC void SendRmppAck(
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	)
{
	RMPP_HEADER	RmppHdr;

	// 1 || to always specify
	// lower risk in case SA forgot to test for RRESPTIME_NONE
	if (1 || pSarContext->ExpectedSeg == 2)
	{
		// to avoid churn, we set RRespTime in our first ACK
		// then we specify None
		BuildRmppAck(&RmppHdr, GSA_SAR_RECV_RESP_TIMEOUT_MULT,
			pSarContext->ExpectedSeg-1, pSarContext->WindowLast);
		// now that we told sender, we can adjust out timeout for use
		// in our timewait state
		pSarContext->RespTimeout = GSA_SAR_RECV_RESP_TIMEOUT;
	} else {
		BuildRmppAck(&RmppHdr, RMPP_RRESPTIME_NONE,
			pSarContext->ExpectedSeg-1, pSarContext->WindowLast);
	}
	SendRmppReply(pDgrm, &RmppHdr);
}

STATIC void SendRmppStopReply(
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	uint8					RmppStatus
	)
{
	RMPP_HEADER	RmppHdr;

	BuildRmppStop(&RmppHdr, RmppStatus);
	SendRmppReply(pDgrm, &RmppHdr);
}

STATIC void SendRmppAbortReply(
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	uint8					RmppStatus
	)
{
	RMPP_HEADER	RmppHdr;

	BuildRmppAbort(&RmppHdr, RmppStatus);
	SendRmppReply(pDgrm, &RmppHdr);
}

// uses pDgrm as a template for building an RMPP reply
// (ABORT, STOP, ACK)
// MAD header and AV information is taken from pDgrm (including TID)
// with R bit inverted
// pRmppHdr is used entirely as given
// rest of MAD is zero'ed
// pDgrm is unaffected
STATIC void SendRmppReply(
	IN	IBT_DGRM_ELEMENT		*pDgrm,
	IN	RMPP_HEADER				*pRmppHdr
	)
{
	FSTATUS						status		= FSUCCESS;
	uint32						i;
	IBT_DGRM_ELEMENT			*pDgrmList;
	GSA_POST_SEND_LIST			*pPostList;
	MAD_RMPP					*pMadRmpp;
#if defined(DBG) || defined(IB_DEBUG)
	static int send_discard_counter = 0;
#endif

	_DBG_ENTER_LVL(_DBG_LVL_SAR_INFO, SendRmppReply);

#if defined(DBG) || defined(IB_DEBUG)
	if (RmppSendDiscardFreq)
	{
		// discard occasional SAR packet sent, will stress retry mechanism
		if (++send_discard_counter % RmppSendDiscardFreq == 1)
		{
			_DBG_PRINT_RMPP_HEADERS( _DBG_LVL_WARN, "Test Discarding Send:\n",
					(MAD_RMPP*)GsiDgrmGetRecvMad(pDgrm), pDgrm->RemoteLID);
			goto done;
		}
	}
#endif

	i = 1;
	status = DgrmPoolGet( g_GsaGlobalInfo->DgrmPoolRecvQ, &i, &pDgrmList );
	if (FSUCCESS != status || i != 1)
	{
		_DBG_ERROR(("SendRmppReply: No more elements in Pool!!!\n"));
		goto done;
	}
	ASSERT(NULL != pDgrmList);

	pMadRmpp = (MAD_RMPP*)GsiDgrmGetRecvMad(pDgrmList);

	// Copy MAD Headers from template
	MemoryCopy( pMadRmpp, GsiDgrmGetRecvMad(pDgrm), sizeof(MAD) );

	// Invert R bit
	pMadRmpp->common.mr.s.R	= ! pMadRmpp->common.mr.s.R;
			
	pMadRmpp->RmppHdr = *pRmppHdr;		// struct copy - RMPP header to use
	MemoryClear(&pMadRmpp->Data[0], STL_RMPP_GS_DATASIZE);	// no Data
	ASSERT(RMPP_STATUS_NORMAL == RmppMadCheck(pMadRmpp));

	// Set remote info
	GsiDgrmAddrCopy(pDgrmList, pDgrm);

	// generate a PostSendList from the first datagram in post
	pPostList		= &(((IBT_DGRM_ELEMENT_PRIV *)pDgrmList)->PostSendList);

	// Fill in list details
	pPostList->DgrmIn = 1;
	AtomicWrite(&pPostList->DgrmOut, 0);
	pPostList->DgrmList			= pDgrmList;
	pPostList->SARRequest		= TRUE;
	pPostList->DgrmList->Element.pBufferList->pNextBuffer->ByteCount =
		pDgrm->Element.pBufferList->pNextBuffer->ByteCount;

	status = GsiDoSendDgrm( NULL, pPostList);
	if (status != FSUCCESS)
	{
		// release AV
		DgrmPoolPut( pDgrmList );	
	}

done:
	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
}

// ---------------------------------------------------------------------------
// RMPP Sender

//
// DoSARSend
//
//	Used by GsiPostSendDgrm to perform a send for a class which
//	has requested Segmentation And Reassembly to be performed for it
//	by GSI
//
//
// INPUTS:
//
//	ServiceClass	- The clients class info
//
//	PostList		- Post list info
//
//	
//
// OUTPUTS:
//
//	None
//
//
//
// RETURNS:
//
//	FCANCELED - Segmentation not required, GsiPostSendDgrm should just send
//			the single packet request, header not swapped
//	FSUCCESS - Segmentation and transmission process has begun, headers swapped
//	other	- unable to start transmission, headers not swapped
//
FSTATUS
DoSARSend(
	IN	GSA_SERVICE_CLASS_INFO	*ServiceClass, 
	IN	GSA_POST_SEND_LIST		*PostList
	)
{
#if 1
	// not yet implemented
#if 0
	// HACK to get further with broken SM, fill in RMPP Version
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate =
				(IBT_DGRM_ELEMENT_PRIV *)PostList->DgrmList;
	MAD_RMPP				*pMadRmpp =
			(MAD_RMPP *)GsiDgrmGetSendMad(&pDgrmPrivate->DgrmElement);
	pMadRmpp->RmppHdr.RmppVersion = RMPP_VERSION;
	BSWAP_RMPP_HEADER(&pMadRmpp->RmppHdr);
#endif
	return FCANCELED;	// assume not segmented
#else
	FSTATUS						status		= FSUCCESS;

	uint32						sendSize;
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	MAD_COMMON					*pMadHdr;
	MULTI_PKT_HDR				*pSARHdr;
	IBT_BUFFER					*pBuffer;
	uint32						dgrmMsgSize, dgrmCount;
	IB_HANDLE					qp1Handle, cqHandle;
	SPIN_LOCK					*qpLock;
	IB_L_KEY					memLKey;
	GSA_SAR_CONTEXT				*pSarContext;
	IB_WORK_REQ					*pWorkRequest;
	IB_ADDRESS_VECTOR			avInfo;
	
	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, DoSARSend);

	
	TBD - here we want to see if this could have an RMPP response
	or is DoubleSided, in either case we want to create the SarContext now
	so its ready when receive comes in.  We can create it as a receiver
	or a sender context depending on whether this send needs RMPP
	or just reponse needs RMPP
	as part of creating context we need to verify the TID is unique.
	if the TID is not unique, its OK, we need to enter a wait for the
	context to complete its prior transaction then create a new context
	perhaps a simple sleep/retry type or a timer doing that for us
	perhaps a timer in our new context which starts out checking periodically
	via RmppFindContext, then once its now found, it proceeds to add itself
	to the global list and start its transaction
	//
	// Get total send requests
	//
	
	pDgrmPrivate		= (IBT_DGRM_ELEMENT_PRIV *)PostList->DgrmList;
	pDgrmPrivate->Base	= PostList;		// link for receives
	pDgrmPrivate->AvHandle = NULL;		// so can trigger for clearup
			
	pMadHdr				= 
		(MAD_COMMON *)GsiDgrmGetSendMad(&pDgrmPrivate->DgrmElement);

	pSARHdr				= 
		(MULTI_PKT_HDR *)((uintn)pMadHdr + sizeof(MAD_COMMON));

	sendSize			= 
		pSARHdr->PayloadLen + sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR);

	
	//
	// Validate user buffer size with payload
	//
	pBuffer				= pDgrmPrivate->DgrmElement.Element.pBufferList;
	dgrmMsgSize			= 0;

	while ( NULL != pBuffer )
	{
		dgrmMsgSize		+= pBuffer->ByteCount;
		pBuffer			= pBuffer->pNextBuffer;
	}

	for ( ;; )
	{
		if ( sendSize > dgrmMsgSize )
		{
			_DBG_ERROR ((
				"User passed Payload that overruns buffer!!!\n"
				"\tMsg size.........:x%x\n"
				"\tBuffer size......:x%x\n",
				sendSize,
				dgrmMsgSize ));

			status = FINVALID_PARAMETER;
			break;
		}
		
		//
		// Calculate segments to send
		//
		dgrmCount = 1;

		if ( sendSize > sizeof(MAD) )
		{
			sendSize -= (sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR));

			if (!( sendSize % MAD_SAR_DATA_SIZE))
				dgrmCount--;

			dgrmCount += ( sendSize / MAD_SAR_DATA_SIZE);
			
			_DBG_PRINT(_DBG_LVL_SAR_INFO,(
				"SAR segments = %d\n", 
				dgrmCount ));
		}
		else
		{
			_DBG_PRINT(_DBG_LVL_SAR_INFO,(
				"Only one SAR segment\n" ));
		}

		//
		// Init SAR Hdr
		//
		pSARHdr->SegmentNum						= 0;

		pSARHdr->FragmentFlag.AsReg8 = 0;
		pSARHdr->Window							= 0;
			
		
		//
		// If there are multiple SAR segments go here
		//
		if ( dgrmCount == 1 )
		{
			status								= FCANCELED;
			_DBG_PRINT(_DBG_LVL_SAR_INFO,(
				"No SAR required. Do a normal send\n" ));

			break;
		}

		// send one at a time 
		PostList->DgrmIn = 1;
		PostList->SARRequest					= TRUE;

		// TBD - could GsiDoSendDgrm be used here?
		// or make some common routes with GsiDoSendDgrm?
		//
		// Fill in Address Vector Info
		//
		avInfo.PortGUID			= pDgrmPrivate->DgrmElement.PortGuid;
		avInfo.DestLID			= pDgrmPrivate->DgrmElement.RemoteLID;
		avInfo.PathBits			= pDgrmPrivate->DgrmElement.PathBits;
		avInfo.ServiceLevel		= pDgrmPrivate->DgrmElement.ServiceLevel;
		avInfo.StaticRate		= pDgrmPrivate->DgrmElement.StaticRate;
		avInfo.GlobalRoute		= pDgrmPrivate->DgrmElement.GlobalRoute;

		if ( TRUE == avInfo.GlobalRoute )
		{
			/* structure copy */
			avInfo.GlobalRouteInfo = pDgrmPrivate->DgrmElement.GRHInfo;
		}

		//
		// Query transport specific info for all segments in one go
		//
		status = GetCaContextFromAvInfo(
			&avInfo,
			pDgrmPrivate->MemPoolHandle->MemList->CaMemIndex,
			&pDgrmPrivate->AvHandle,
			&qp1Handle,
			&cqHandle,
			&qpLock,
			&memLKey );

		if ( FSUCCESS != status )
		{
			_DBG_ERROR ((
				"Error in GetCaContextFromPortGuid()\n" ));
			break;
		}

		//
		// If this is a request, save the user's transaction id 
		// and use our own.  We will restore the user's transaction id
		// when the request completes.  If this is a not a request, we let
		// the client control the transaction ID, which may have been
		// been remotely generated.
		//
		if( MAD_IS_REQUEST((MAD*)pMadHdr))
		{
			pMadHdr->TransactionID = (uint64)\
				(pMadHdr->TransactionID & 0xffffffffffffff00ll);
			pMadHdr->TransactionID = \
				pMadHdr->TransactionID | ServiceClass->ClientId;
		}

		pSarContext = (GSA_SAR_CONTEXT*)MemoryAllocateAndClear(sizeof(GSA_SAR_CONTEXT), FALSE, GSA_MEM_TAG);
		if ( NULL == pSarContext )
		{
			status								= FINSUFFICIENT_RESOURCES;
			_DBG_ERROR ((
					"Not enough memory for Sar Context info!\n" ));
			break;
		}		

		//
		// Init Lock and Timer
		//
		SpinLockInitState( &pSarContext->Lock );
		SpinLockInit( &pSarContext->Lock );

		TimerInitState( &pSarContext->Timer);
		if (TRUE != TimerInit( 
					&pSarContext->Timer, 
					TimerCallbackSend, 
					pSarContext ))
		{
			status								= FINSUFFICIENT_RESOURCES;
			_DBG_ERROR ((
					"TimerInit failed!\n" ));

			SpinLockDestroy( &pSarContext->Lock );
			MemoryDeallocate( pSarContext );
			break;
		}		

		//
		// setup other fields
		//
		pSarContext->TotalSegs			= dgrmCount;
		pSarContext->CurrSeg			= 0;

		pSarContext->TID				= pMadHdr->TransactionID;
		pSarContext->MgmtClass			= pMadHdr->MgmtClass;
		if (PostList->DgrmList->GlobalRoute)
		{
			pSarContext->RemoteLID 		= LID_RESERVED;
			pSarContext->RemoteGID 		= PostList->DgrmList->GRHInfo.DestGID;
		} else {
			pSarContext->RemoteLID 		= PostList->DgrmList->RemoteLID;
			MemoryClear(&pSarContext->RemoteGID, sizeof(pSarContext->RemoteGID));
		}
		
		pSarContext->Qp1Handle						= qp1Handle;
		pSarContext->QpLock							= qpLock;
		pSarContext->PostList						= PostList;

		pBuffer									= pDgrmPrivate->DgrmElement.Element.pBufferList;

		pSarContext->TotalDataSize				= pSARHdr->PayloadLen;
		pSarContext->DataStart						= 
					(uintn)(pBuffer->pData) + \
					sizeof(MAD_COMMON) + \
					sizeof(MULTI_PKT_HDR);

		_DBG_PRINT (_DBG_LVL_SAR_INFO,(
					"SAR Payload Length = %d bytes\n",pSarContext->TotalDataSize ));

		//
		// Format SAR header
		//
		pSARHdr->SegmentNum						= 0;	
		pSARHdr->FragmentFlag.AsReg8				= 0;
		pSARHdr->FragmentFlag.F.FirstPkt			= 1;

		//
		// Pre create work request since it is common for all segments
		//
		pWorkRequest							= &pSarContext->WorkRequest;

		ASSERT_VALID_WORKREQID((uintn)pDgrmPrivate);
		pWorkRequest->WorkReqId		= BUILD_SQ_WORKREQID((uintn)pDgrmPrivate);
		
		//
		// SAR will Always be 3: MAD_COMMON, SAR_HDR, DATA
		//
		pWorkRequest->DSListDepth				= 3;		
		
		pWorkRequest->DSList					= pDgrmPrivate->DsList;
		pWorkRequest->Req.SendUD.AVHandle		= pDgrmPrivate->AvHandle;

		pWorkRequest->Operation					= WROpSend;

		// zero out the options
  		pWorkRequest->Req.SendUD.Options.AsUINT16				= 0;

		pWorkRequest->Req.SendUD.Options.s.SignaledCompletion	= TRUE; 

		//pWorkRequest->Req.SendUD.Qkey			= pDgrmPrivate->DgrmElement.RemoteQKey;
		pWorkRequest->Req.SendUD.QPNumber		= pDgrmPrivate->DgrmElement.RemoteQP;
		pWorkRequest->Req.SendUD.Qkey			= QP1_WELL_KNOWN_Q_KEY;
		pWorkRequest->Req.SendUD.PkeyIndex		= pDgrmPrivate->DgrmElement.PkeyIndex;

#if defined(DBG) ||defined( IB_DEBUG)

		if (!pWorkRequest->Req.SendUD.QPNumber) 
		{
			AtomicIncrementVoid( &ServiceClass->ErrorMsgs );
			
			if (!(AtomicRead(&ServiceClass->ErrorMsgs) % 20))
			{
				_DBG_ERROR((
					"Invalid QPNumber/Q_Key(substitution provided):\n"
					"\tClass...........:x%x\n"
					"\tResponder.......:%s\n"
					"\tTrapProcessor...:%s\n"
					"\tReportProcessor.:%s\n"
					"\tSARClient.......:%s\n"
					"\tQPNumber........:x%x\n"
					"\tQ_Key...........:x%x\n",
					ServiceClass->MgmtClass,
					GSI_IS_RESPONDER(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE",
					GSI_IS_TRAP_PROCESSOR(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE",
					GSI_IS_REPORT_PROCESSOR(ServiceClass->RegistrationFlags)
						? "TRUE":"FALSE",
					ServiceClass->bSARRequired ? "TRUE":"FALSE",
					pWorkRequest->Req.SendUD.QPNumber,
					pWorkRequest->Req.SendUD.Qkey
					));
			}

			pWorkRequest->Req.SendUD.Qkey		= QP1_WELL_KNOWN_Q_KEY;
			pWorkRequest->Req.SendUD.QPNumber	= 1;
		}

		if (pWorkRequest->Req.SendUD.QPNumber	!= 1)
		{
			_DBG_ERROR((
				"Send to RedirectedQP\n"
				"\tQPNumber.....:x%x\n",
				pWorkRequest->Req.SendUD.QPNumber
				));
		}

#endif			

		//
		// fill in scatter/gather DS list details
		//
		pWorkRequest->MessageLen				= 0;

		//
		// Fill in MAD Common info
		//
		pWorkRequest->DSList[0].Address			= (uintn)pBuffer->pData;
		pWorkRequest->DSList[0].Lkey			= memLKey;
		pWorkRequest->DSList[0].Length			= sizeof(MAD_COMMON);

		//
		// Fill in SAR Header info
		//
		pWorkRequest->DSList[1].Address			= \
			(uintn)(pBuffer->pData) + sizeof(MAD_COMMON);
		pWorkRequest->DSList[1].Lkey			= memLKey;
		pWorkRequest->DSList[1].Length			= sizeof(MULTI_PKT_HDR);

		//
		// Fill in Data info
		//
		pWorkRequest->DSList[2].Lkey			= memLKey;
		
		pSarContext->DataSeg					= &pWorkRequest->DSList[2];
	
		//
		// TBD: Add to master SAR send list
		//
		SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock );

		QListInsertHead(
			&g_GsaGlobalInfo->SARSendList, 
			&pSarContext->ListItem );

		SpinLockRelease( &g_GsaGlobalInfo->SARSendListLock );

		//
		// Add timer to Dgrm internal list
		//
		pDgrmPrivate->pSarContext					= pSarContext;

		pSarContext->SysCBItem						= SysCallbackGet( 
													IbtGlobal.DeviceObject );
		if ( NULL == pSarContext->SysCBItem )
		{
			status								= FINSUFFICIENT_RESOURCES;
			TimerDestroy( &pSarContext->Timer );
			SpinLockDestroy( &pSarContext->Lock );
			MemoryDeallocate ( pSarContext );
			_DBG_ERROR ((
				"SysCallback Create failed! Dropping SAR send req.\n" ));
			break;
		}
		
		SARPost(pSarContext);
		break;
	}			// for loop
	
	if (status != FSUCESSS && status != FCANCELED)
	{
		if (pDgrmPrivate->AvHandle)
		{
			// release AV
			(void)PutAV(NULL,pDgrmPrivate->AvHandle);
		}
	}

    _DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return status;
#endif
}


#if 0	// not yet implemented
FSTATUS
SARPost(
	IN	GSA_SAR_CONTEXT		*pSarContext
	)
{
	FSTATUS					status		=FSUCCESS;
	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
	IB_LOCAL_DATASEGMENT	*pDsList;
	IB_WORK_REQ				*pWorkRequest;
	MULTI_PKT_HDR			*pSARHdr;
	uint32					bytesSent, bytesToSend;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARPost);


	SpinLockAcquire( &pSarContext->Lock );

	pDgrmPrivate			= 
		(IBT_DGRM_ELEMENT_PRIV *)pSarContext->PostList->DgrmList;

	pWorkRequest			= &pSarContext->WorkRequest;
	pSARHdr					= \
		(MULTI_PKT_HDR *)((uintn)pWorkRequest->DSList[1].Address);

	//
	// Calculate bytes to be sent
	//
	pDsList					= pSarContext->DataSeg;

	bytesSent				= pSarContext->CurrSeg * MAD_SAR_DATA_SIZE;
	bytesToSend				= pSarContext->TotalDataSize - bytesSent;

	pDsList->Address		= pSarContext->DataStart + bytesSent;
	pDsList->Length			= MAD_SAR_DATA_SIZE;

	pWorkRequest->MessageLen= pDsList->Length + \
		sizeof(MAD_COMMON) + sizeof(MULTI_PKT_HDR);


	//
	// Check to see if we have multiple segments
	//
	if ( 0 != pSarContext->CurrSeg )
	{
		pSARHdr->FragmentFlag.F.FirstPkt = 0;

		if ( pSarContext->CurrSeg+1 == pSarContext->TotalSegs )
			pSARHdr->FragmentFlag.F.LastPkt = 1;
	}
	else
	{
		//
		// First segment. Validate payload needs multi segments
		//
		pSARHdr->FragmentFlag.F.FirstPkt = 1;
	}

	AtomicIncrementVoid( &pSarContext->CurrSeg );	// advance to next level
	pSARHdr->SegmentNum = pSarContext->CurrSeg;

	_DBG_PRINT (_DBG_LVL_SAR_INFO,(
		"SAR Header Info:\n"
		"\tPayloadLen...:%d bytes\n"
		"\tSegment......:%d\n",
		pSARHdr->PayloadLen,
		pSARHdr->SegmentNum
		));
	_DBG_PRINT (_DBG_LVL_SAR_INFO,(
		"SAR Header Info (cont):\n"
		"\tFragmentFlag.:\n"
		"\t\tFirstPkt..:x%x\n"
		"\t\tLastPkt...:x%x\n"
		"\t\tKeepAlive.:x%x\n"
		"\t\tResendReq.:x%x\n"
		"\t\tAck.......:x%x\n",
		(uint8)(pSARHdr->FragmentFlag.F.FirstPkt),
		(uint8)(pSARHdr->FragmentFlag.F.LastPkt),
		(uint8)(pSARHdr->FragmentFlag.F.KeepAlive),
		(uint8)(pSARHdr->FragmentFlag.F.ResendReq),
		(uint8)(pSARHdr->FragmentFlag.F.Ack)						
		));
	
	AtomicWrite(&pSarContext->PostList->DgrmOut, 0);
	pSarContext->RetryCount			= 0;
	pSarContext->ResendSent			= FALSE;

	BSWAP_MAD_HEADER ( (MAD*)((uintn)pWorkRequest->DSList[0].Address) );
	BSWAP_SAR_HEADER ( pSARHdr );

	pDgrmPrivate->DgrmElement.OnSendQ=TRUE;
	SpinLockAcquire( pSarContext->QpLock );
	status = iba_post_send( pSarContext->Qp1Handle, pWorkRequest );
	SpinLockRelease( pSarContext->QpLock );

	SpinLockRelease( &pSarContext->Lock );

	if (status != FSUCCESS)
	{
		pDgrmPrivate->DgrmElement.OnSendQ=FALSE;
		// swap header back to host order for consistency
		BSWAP_MAD_HEADER ( (MAD*)((uintn)pWorkRequest->DSList[0].Address) );
		BSWAP_SAR_HEADER ( pSARHdr );
	}

	_DBG_LEAVE_LVL (_DBG_LVL_SAR_INFO);
	return status;
}
#endif


#if 0	// not yet implemented
// returns TRUE if RMPP Context is found in which case it processes the
// 		resend request
// returns FALSE if RMPP Context is not found
boolean
ResendReqSARSend( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	GSA_SAR_CONTEXT				*pSarContext= NULL;
	boolean						bStatus		= FALSE;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, ResendReqSARSend);

	pSarContext = FindSarContext(pMadRmpp, *ppDgrm, TRUE);	// sender
	if (pSarContext != NULL)
	{
		// Reset Segment number to request
		pSarContext->CurrSeg = pMadRmpp->Sar.SegmentNum-1;
		SpinLockRelease( &pSarContext->Lock );
		bStatus = TRUE;
		_DBG_PRINT (_DBG_LVL_SAR_INFO,( "Timer restarted\n" ));
	}

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return (bStatus);
}	
#endif


#if 0	// not yet implemented
// returns TRUE if RMPP Context is found in which case it processes the ack
// returns FALSE if RMPP Context is not found
boolean
AckReqSARSend( 
	IN	MAD_RMPP				*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	GSA_SAR_CONTEXT				*pSarContext= NULL;
	boolean						bStatus		= FALSE;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, AckReqSARSend);

	pSarContext = FindSarContext(pMadRmpp, *ppDgrm, TRUE);	// sender

	if (pSarContext != NULL )
	{
		// Close send timer
		if ( pSarContext->TotalSegs == pSarContext->CurrSeg )
		{
			SpinLockRelease( &pSarContext->Lock );
			SARCloseSend(pSarContext);
		} else {
			// we never asked for this Ack, do nothing
			SpinLockRelease( &pSarContext->Lock );
		}

		bStatus = TRUE;
		_DBG_PRINT (_DBG_LVL_SAR_INFO,( "Timer killed with Ack\n" ));
	}

	_DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);
	return (bStatus);
}	
#endif

#if 0	// not yet implemented
//
// Close SAR send created
//
//	+ Destoy retry timer
//	+ Destroy AV
//	+ Destroy Syscallback
//	+ Call user or dump Dgrm
//	+ Free timer pool
//
void
SARCloseSend(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	IBT_DGRM_ELEMENT_PRIV		*pDgrmPrivate;
	GSA_SERVICE_CLASS_INFO		*classInfo;

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARCloseSend);

	pDgrmPrivate				= 
		(IBT_DGRM_ELEMENT_PRIV *)pSarContext->PostList->DgrmList;

	_DBG_PRINT (_DBG_LVL_SAR_INFO,
		("*** Killing Send retry timer ***\n"));
			
	StopSarContextTimer( pSarContext );		// stop timer before destroy
	TimerDestroy( &pSarContext->Timer );	// Destroy stops timer too

	PutAV(
		NULL,
		pDgrmPrivate->AvHandle );

	//
	// Remove from global list
	//
	SpinLockAcquire( &g_GsaGlobalInfo->SARSendListLock );

    ASSERT (QListIsItemInList( 
		&g_GsaGlobalInfo->SARSendList, 
		&pSarContext->ListItem ));


	if ( !QListIsItemInList( 
		&g_GsaGlobalInfo->SARSendList,
		&pSarContext->ListItem ))
	{
		_DBG_ERROR(("Item not in Send List!!!\n"));
	}
	else
	{
		QListRemoveItem ( 
			&g_GsaGlobalInfo->SARSendList,
			&pSarContext->ListItem );
	}
	
	SpinLockRelease(&g_GsaGlobalInfo->SARSendListLock);

	//
	// CallUser
	//
	classInfo					= pDgrmPrivate->MemPoolHandle->ServiceHandle;

	if ( classInfo->SendCompletionCallback )
	{
		(classInfo->SendCompletionCallback)(
				classInfo->ServiceClassContext,
				&pDgrmPrivate->DgrmElement );
	}
	else
	{
		//
		// return to pool
		//
		_DBG_PRINT (_DBG_LVL_SAR_INFO, (
			"No Class Agent for this SAR msg!\n"));
			
		DgrmPoolPut( &pDgrmPrivate->DgrmElement );	
	}

	SpinLockDestroy( &pSarContext->Lock );		// lock
	SysCallbackPut( pSarContext->SysCBItem );
	MemoryDeallocate ( pSarContext );

    _DBG_LEAVE_LVL(_DBG_LVL_SAR_INFO);

}	// SARCloseSend
#endif


//
// Callback invoked on SAR sends
//
//	+ There are no retires for send.
//	+ Completions for ISR callbacks trigger next send of segement.
//	+ Timer is fired after last send
//
void
SARSendSegments(
	IN	void				*Key,
	IN	void				*TimerContext
	)
{
	ASSERT(0);	// should not be called, send not yet implemented
#if 0	// not yet implemented
	GSA_SAR_CONTEXT			*pSarContext;
	IBT_DGRM_ELEMENT_PRIV	*pDgrmPrivate;
			

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, SARSendSegments);

	pSarContext				= TimerContext;
	pDgrmPrivate			= 
		(IBT_DGRM_ELEMENT_PRIV *)pSarContext->PostList->DgrmList;

	//
	// Verify last completion status
	//
	if ( FSUCCESS != pDgrmPrivate->DgrmElement.Element.Status )
	{
		_DBG_PRINT (_DBG_LVL_SAR_INFO,
			("*** Failed SAR send! Killing retry timer ***\n"));
			
		SARCloseSend( pSarContext );
	}
	else
	{
		//
		// Are we in the last segment?
		//
		if ( pSarContext->TotalSegs == pSarContext->CurrSeg )
		{
			_DBG_PRINT (_DBG_LVL_SAR_INFO,
				("*** Kicking off retry timer ***\n"));

			//
			// Fire timer for retires and deliveries
			//
			StartSarContextTimer( pSarContext, GSA_SAR_TIME_SEND );
		}
		else
		{
			SARPost(pSarContext);
		}
	}

    _DBG_LEAVE_LVL (_DBG_LVL_SAR_INFO);
#endif

}	// SARSendSegments


#if 0	// not yet implemented
//
// Timer callback invoked on SAR sends
//	
//	+ Here the retry counter will stop posts if there is no completion.
//	+ The timer waits for the Isr routine's callback to update the send
//	  completions
//
void
TimerCallbackSend(
	IN	void				*TimerContext
	)
{
	GSA_SAR_CONTEXT			*pSarContext;
		

	_DBG_ENTER_LVL (_DBG_LVL_SAR_INFO, TimerCallbackSend);

	pSarContext					= TimerContext;
	// TBD, not fully implemented, review and how to use TimerCallbackStopped
	// test within lock to prevent stale callbacks from executing
	SARCloseSend( pSarContext );

    _DBG_LEAVE_LVL (_DBG_LVL_SAR_INFO);
}
#endif

FSTATUS
RmppSenderProcessRecv (
	IN	GSA_SAR_CONTEXT			*pSarContext,
	IN MAD_RMPP					*pMadRmpp,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	)
{
	ASSERT(0);	// should not get here, send not yet implemented
	return FREJECT;
}

void
RmppSenderTimerCallback(
	IN	GSA_SAR_CONTEXT			*pSarContext
	)
{
	ASSERT(0);	// should not get here, send not yet implemented
}
