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
/* [ICS VERSION STRING: unknown] */

/* CM data structures private to the kernel CM */

/* Suppress duplicate loading of this file */
#ifndef _INSI_IB_CM_PRIVATE_H_
#define _INSI_IB_CM_PRIVATE_H_


#if defined (__cplusplus)
extern "C" {
#endif

#include "bldver.h"

/* OS independent routines */
#include "datatypes.h"
#include "ievent.h"
#include "itimer.h"
#include "imemory.h"
#include "ispinlock.h"
#include "idebug.h"
#include "ithread.h"
#include "ireaper.h"
#include "isyscallback.h"
#include "ibyteswap.h"
#include "iquickmap.h"
#include "ilist.h"
#include "stl_types.h"
#include "stl_mad_priv.h"
#include "stl_helper.h"
#include "ib_gsi.h"
#include "ibt.h"
#include "cm_common.h"

// Enable CM event handling, this allows COMM_ESTABLISHED event to
// be treated as an RTU
//#undef _CM_EVENT_ON_QP_
#define _CM_EVENT_ON_QP_

#ifdef _CM_EVENT_ON_QP_
#include "vpi.h"
#include "vpi_export.h"
#endif

/* CM header files */
#include "cm_common.h"
#include "cm_msg.h"

#include "vca_export.h" /* IBT_COMPONENT_INFO is defined here */

/*****************************************************************************
 * Defines
 */


/*****************************************************************************
 * CM-fixed params
 */
#define CM_MAD_SIZE						MAD_BLOCK_SIZE
#define CM_WELLKNOWN_QPN				1	/* QP 1 */
#define CM_DGRM_SIZE					CM_MAD_SIZE + sizeof(IB_GRH)

		/* Allow up to 92 bytes for discriminator value */
#define CM_MAX_DISCRIMINATOR_LEN		CMM_REQ_USER_LEN

	/* maximum Queued APC retries to process in a single timer event */
#define CM_APC_RETRY_LIMIT				200
#define CM_APC_RETRY_INTERVAL			100		/* ms */

/*****************************************************************************
 * CM_EVENT -
 * Events that affect the CEP state.
 */

	/* CM Message Events */
#define CME_RCVD_REP			0x0001	/* Received reply msg */
#define CME_RCVD_REQ			0x0002  /* Received request msg */
#define CME_RCVD_MRA			0x0004  /* Received mra msg */
#define CME_RCVD_REJ			0x0008  /* Received reject msg */
#define CME_RCVD_RTU			0x0010  /* Received rtu msg */
#define CME_RCVD_UMSG			0x0020  /* Received user data - NOT USED until verb send indication to CM */
#define CME_RCVD_DREQ			0x0040  /* Received disconnect request msg */
#define CME_RCVD_DREP			0x0080  /* Received disconnect reply msg */
#define CME_RCVD_LAP			0x0100	/* Received lap msg */
#define CME_RCVD_APR			0x0200	/* Received apr msg */
#define CME_RCVD_SIDR_REQ		0x0400	/* Received sidr request msg */
#define CME_RCVD_SIDR_RESP		0x0800	/* Received sidr response msg */

#define CME_TIMEOUT_REQ			0x1000  /* Request timeout (did not get REP) */
#define CME_TIMEOUT_REP			0x2000	/* Reply timeout (did not get RTU) */
#define CME_TIMEOUT_LAP			0x4000	/* Lap timeout (did not get APR) */
#define	CME_TIMEOUT_DREQ		0x8000	/* Disconnect request timeout */
#define	CME_TIMEOUT_TIMEWAIT	0x10000	/* Timewait timeout */
#define	CME_TIMEOUT_SIDR_REQ	0x20000	/* SIDR REQ timeout */
#define	CME_SIM_RCVD_DREQ		0x40000 /* simulating Received DREQ msg */

	/* User Call Events */
/*#define USRE_LISTEN				0x100000 */
#define USRE_CONNECT			0x200000
/*#define USRE_REGISTER			0x400000 */
/*#define USRE_WAIT				0x800000 */
#define USRE_DESTROY			0x1000000
#define USRE_CANCEL				0x2000000

	/* System Events */
#define SYSE_DISCONNECT			0x4000000
/*#define LAST_EVENT				0x80000000 */

/*
 * Datagram flags
 * CM uses the Dgrm->Element.Context field to hold these flags
 */
#define CM_DGRM_FLAG_IN_USE			0x0001	/* dgrm is send pending */
#define CM_DGRM_FLAG_RELEASE		0x0002	/* return send dgrm to pool
											 * when send completes
											 */


/*****************************************************************************
 * CM_STATE -
 * CEP object can be in different states depending on user input, 
 * messages coming in from the wire and timeout events
 */
typedef enum {
	CMS_IDLE=1,
	CMS_LISTEN,
	CMS_REQ_SENT,
	CMS_REQ_RCVD,
	CMS_REP_SENT,
	CMS_REP_RCVD,
	CMS_MRA_REQ_SENT,
	CMS_MRA_REP_SENT,
	CMS_REP_WAIT, /* eg. CMS_MRA_REQ_RCVD */
	CMS_MRA_REP_RCVD,
	CMS_ESTABLISHED,
	CMS_LAP_SENT,
	CMS_MRA_LAP_SENT,
	CMS_LAP_RCVD,
	CMS_MRA_LAP_RCVD,
	CMS_DREQ_SENT,
	CMS_DREQ_RCVD,
	CMS_SIM_DREQ_RCVD,	/* internal state, we simulated DREQ receipt
						 * possibly in response to receiving a REJ
						 * after user does a Disconnect or DestroyCEP
						 * we will move to Timewait
						 */
	CMS_TIMEWAIT,
	/* states for UD SIDR CEPs */
	CMS_REGISTERED,
	CMS_SIDR_REQ_SENT,
	CMS_SIDR_REQ_RCVD,
	/*CMS_SIDR_RESP_SENT, */
	/*CMS_SIDR_RESP_RCVD, */
} CM_CEP_STATE;

/*****************************************************************************
 * CM_CEP_MODE -
 * CEP object can be only be in 1 mode
 */
typedef enum {
	CM_MODE_NONE=0,
	CM_MODE_ACTIVE=1,
	CM_MODE_PASSIVE,
} CM_CEP_MODE;


typedef struct _CM_MSG_COUNTS {
	ATOMIC_UINT		ClassPortInfo;
	ATOMIC_UINT		ClassPortInfoResp;
	ATOMIC_UINT		Req;
	ATOMIC_UINT		Rep;
	ATOMIC_UINT		Rtu;
	ATOMIC_UINT		MraReq;
	ATOMIC_UINT		MraRep;
	ATOMIC_UINT		MraLap;
	ATOMIC_UINT		Lap;
	ATOMIC_UINT		AprAcc;	/* APR with good status */
	ATOMIC_UINT		AprRej;	/* APR with rejection status */
	ATOMIC_UINT		RejReq;
	ATOMIC_UINT		RejRep;
	ATOMIC_UINT		Dreq;
	ATOMIC_UINT		Drep;
	ATOMIC_UINT		SidrReq;
	ATOMIC_UINT		SidrResp;
	ATOMIC_UINT		Failed;
} CM_MSG_COUNTS;

/*****************************************************************************
 * CM_GLOBAL_CONTEXT -
 * Connection manager's global context.
 * There is only 1 instance of this
 */
typedef struct _CM_GLOBAL_CONTEXT {

	IB_HANDLE				GSAHandle;

	IB_HANDLE				GSADgrmPoolHandle;

	/* Last LocalCommID value assigned, starts at a semi-random value */
	uint32					CommID;	

	/* Last StartPSN value assigned, starts at a semi-random value */
	uint32					StartPSN;	

	/* Sanity check. This is assigned at CM startup and each CEP object created is set to this value */
	uint32					Signature;

	/* Track the average local consumer processing turnaround time */
	uint64					turnaround_time_us;

	/* List of CAs */
	DLIST_ENTRY				DeviceList;
	SPIN_LOCK				DeviceListLock;

	/* We use a single lock to provide access to the lists below. In essence,
	 * we are protecting the object too. In the future, we can let each list has
	 * its own lock to provide more concurrency. However, we will still need a common
	 * lock for all the lists so we can handle moving CEPs from one state to
	 * another (in which case the list it is on also changes).
	 */

	SPIN_LOCK				ListLock;

	QUICK_LIST				AllCEPs;		/* all the allocated CEPs */

	CM_MAP					TimerMap;		/* key is CEP pointer */

	/* Lists below are based on keys by which we lookup CEPs
	 * a given CEP will often be on multiple lists
	 */
	CM_MAP					LocalCommIDMap;	/* Key is LocalCommId */
	CM_MAP					LocalEndPointMap;	/* Key is LocalEndPoint
												 * not used for SIDR
												 */
	CM_MAP					RemoteEndPointMap;	/* Key is RemoteEndPoint */

	/* Lists below are based on State of CEP
	 * outstanding SIDR requests from SIDRQuery are placed here
	 * CEPs in CMS_SIDR_REQ_SENT
	 * MapEntry in CEP used
	 */
	CM_MAP					QueryMap;	/* Key is LocalCommID */

	/* Servers (CEPs in CMS_LISTEN), Sidr Listeners(CMS_REGISTERED) and
	 * Peer to Peer CEPs (CMS_REQ_SENT until a remote end point is associated)
	 * MapEntry in CEP used
	 */
	CM_MAP					ListenMap;		/* Key is bound listen address */

	EVENT					EventObj;

	EVENT					CallbackEventObj;

	ATOMIC_UINT				TotalCallbackRefCnt;

	/* If a single user application is heavily loaded with CM traffic, it
	 * could overflow its Apc buffer.  In which case we keep track of
	 * such ceps here and attempt to retry the APC events periodically
	 * such that the CM event/callback is not lost
	 */
	QUICK_LIST				ApcRetryList;	/* CEPs which failed a PostApcRecord
											 * scheduled for retry later
											 */
	TIMER					ApcRetryTimer;	/* for ApcRetryList processing */
	uint8					ApcRetryTimerRunning;/* is timer running */

	/* Statistics */
	CM_MSG_COUNTS			Sent;
	CM_MSG_COUNTS			Recv;

} CM_GLOBAL_CONTEXT;


/*****************************************************************************
 * CM_DEVICE_OBJECT -
 * Connection manager's device context. This is a per device context.
 */
typedef struct _CM_DEVICE_OBJECT {

	DLIST_ENTRY			DeviceListEntry;

	uint32				Signature;		/* From CM.Signature */

	uint64				CaGUID;

#ifdef _CM_EVENT_ON_QP_
	IB_HANDLE			CaHandle;		/* Returned from OpenCA() */
#endif
} CM_DEVICE_OBJECT;

/* A local or remote endpoint */
typedef struct _CM_END_POINT {
	uint64				CaGUID;			/* Used by non-SIDR connection */
	uint32				QPN:24;
	uint32				Reserved1:8;
	uint32				EECN:24;
	uint32				Reserved2:8;
} CM_END_POINT;

/* portions of IB_PATH_RECORD which can vary between Primary and
 * Alternate Path
 */
typedef struct _CM_CEP_PATH {
	EUI64	LocalPortGuid;
	IB_GID	LocalGID;
	IB_GID	RemoteGID;
	IB_LID	LocalLID;
	IB_LID	RemoteLID;

	uint32	FlowLabel:20;
	uint32	StaticRate:6;	/* enum IB_STATIC_RATE */
	uint32	TClass:6;

	uint16	SL:4;
	uint16	bSubnetLocal:1;
	uint16	AckTimeout:5; /* what we put on wire */
	uint16	LocalAckTimeout:5;	/* what we will use for our QP */
	uint16	Reserved:1;

	uint16	PkeyIndex;
	uint8	HopLimit;
	uint8	LocalGidIndex;	/* for LocalGID */
	uint8	LocalPathBits;	/* for LocalLID */
} CM_CEP_PATH;


/*****************************************************************************
 * CM_CEP_OBJECT -
 * Connection endpoint object. This object holds various informations and state
 * of a connection.
 *
 * List Usage:
 *
 * ListenMap: (uses CEP->MapEntry)
 * 		CMS_REGISTERED, CMS_LISTEN
 * 		CMS_REQ_SENT or CMS_MRA_REQ_RCVD for a Peer connect with no
 * 			associated remote end point	(eg. RemoteEndPoint.CaGUID == 0)
 * PendingList: (uses CEP->ListEntry)
 * 		CMS_REQ_RCVD are linked to parent (CMS_LISTEN)
 * 			for Peer, parent is itself (linked to own list)
 * 		CMS_SIDR_REQ_RCVD are linked to parent (CMS_REGISTERED)
 * LocalCommIDMap (uses CEP->LocalCommIDMapEntry)
 * 		all but CMS_REGISTERED, CMS_LISTEN, CMS_IDLE
 * LocalEndPointMap (uses CEP->LocalEndPointMapEntry)
 * 		CMS_REQ_SENT, ...
 * RemoteEndPointMap (uses CEP->RemoteEndPointMapEntry)
 * 		all but CMS_REGISTERED, CMS_LISTEN, CMS_IDLE
 * 			Non-Peer Connect CMS_REQ_SENT
 * 			Peer Connect CMS_REQ_SENT without associated remote end point
 * 			...
 * QueryMap (uses CEP->MapEntry)
 * 		CMS_SIDR_REQ_SENT
 */
typedef struct _CM_CEP_OBJECT {
	EVENT *				pWaitEvent;		/* APC Event for User mode callbacks */

	/* Event notification */
	EVENT *				pEventObj;		/* Event for use by CmWait for appls
										 * which do not want callbacks.
										 * bPrivateEvent indicates if this
										 * was supplied by appl
										 */

	EVENT*				pCallbackEvent; /* This is signaled when there is
										 * no outstanding callback.
										 */

	TIMER				TimerObj;		/* for CM response timeouts/timewait */
	uint32				Signature;		/* From CM.Signature */
	CM_CEP_STATE		State;			/* for CM State Machine */
	CM_CEP_MODE			Mode;			/* Active or passive */
	CM_CEP_TYPE			Type;			/* RC, UC, RD, UD */
	
	uint32				EventFlags;		/* indicates callback event */

	uint8				bPeer:1;		/* is this a Peer to Peer connect */
	uint8				bPrivateEvent:1;/* was pEventObj supplied by appl */
	/*uint8				bAutoCancel:1; */
	uint8				bAsyncAccept:1;	/* CM_FLAG_ASYNC_ACCEPT */
	uint8				bSidrRegisterNotify:1;
	uint8				RetryCount:4;	/* retries done thus far for given msg */

	uint8				ApcRetryCount;	/* number of failed PostApcRecords */

	uint64				timestamp_us;	/* when timer started */
	uint32				timeout_ms;		/* duration of timer */

	uint64				turnaround_time_us; /* appl specific turnaround time */

	uint32				ListenBackLog;	/* maximum pending CEPs on a listener */
	/*uint32				TimerRefCnt;		// When this obj is waited on, we increment this cnt to prevent deletion */

	ATOMIC_UINT			CallbackRefCnt;
	/* Properties */
	uint32				LocalCommID;
	uint32				RemoteCommID;
	uint64				TransactionID;

	uint32				LocalCMTimeout:5;	/* 4.096*2^n usec */
	uint32				RemoteCMTimeout:5;	/* 4.096*2^n usec */
	uint32				bTimewaitCallback:1;
	uint32				bFailoverSupported:1;/* best guess from REP */
	uint32				MaxCMRetries:4;

		/* PktLifeTime is not accurately available to the passive end
		 * so the value here is rounded up by using AckTimeout/2
		 * Since this is used for CM timeouts, being conservative is fine
		 */
	uint16				PktLifeTime:5;		/* 4.096*2^n usec */
	uint16				Timewait:5;			/* 4.096*2^n usec */
	uint16				bLapSent:1;			/* has a LAP been sent previously */
	uint16				TargetAckDelay:5;	/* only useful for Active side */

	CM_END_POINT		LocalEndPoint;	/* CaGuid used by non-SIDR connection */
	uint64				SID;
	CM_END_POINT		RemoteEndPoint;	/* CaGuid used by non-SIDR connection */

	CM_CEP_PATH			PrimaryPath;
	CM_CEP_PATH			AlternatePath;
	EUI64				TempAlternateLocalPortGuid;
	uint8				TempAlternateLocalGidIndex;
	uint8				TempAlternateLocalPathBits;
	uint16				TempAlternatePkeyIndex;
	uint8				TempAlternateLocalAckTimeout:5;
	uint8				Reserved:3;

	uint32				QKey;			/* Used by SIDR */
	uint16				PartitionKey;
	uint8				LocalInitiatorDepth;	// init depth we use in our QP
	uint8				LocalResponderResources;// resp res we use in our QP
	uint32				LocalRetryCount:3;		// our send retry count
	uint32				LocalRnrRetryCount:3;	// our send RNR retry count
	uint32				LocalSendPSN:24;		// our 1st send PSN
	uint32				Reserved2:2;
	uint32				LocalRecvPSN:24;		// our 1st expected recv PSN
	uint32				Mtu:6;			/* enum IB_MTU */
	uint32				Reserved3:2;

	LIST_ITEM			AllCEPsItem;
	LIST_ITEM			ApcRetryListItem;

	CM_MAP_ITEM			TimerMapEntry;	/*gCM->TimerMap while timer running */

	CM_MAP_ITEM			MapEntry;	/* ListenMap or QueryMap */
	CM_MAP_ITEM			LocalCommIDMapEntry;
	CM_MAP_ITEM			LocalEndPointMapEntry;
	CM_MAP_ITEM			RemoteEndPointMapEntry;

	DLIST_ENTRY			PendingList;	/* Queued REQs not yet Accepted */
	uint32				PendingCount;
	DLIST_ENTRY			PendingListEntry;
	struct _CM_CEP_OBJECT 	*pParentCEP;	/* while on Pending List */

	/* CM MAD is stored in here */
	IBT_DGRM_ELEMENT*	pDgrmElement;

	/* The fields below are used for user notification */

	/* User-specified callback function */
	SYS_CALLBACK_ITEM*	pCallbackItem;
	PFN_CM_CALLBACK		pfnUserCallback;
	void*				UserContext;	/* Passed to the user callback func */
	
	/* Private Data discriminator for listener */
	uint8				*Discriminator;
	uint8				DiscriminatorLen;
	uint8				DiscrimPrivateDataOffset;
} CM_CEP_OBJECT;


/* Extern global var */
extern CM_GLOBAL_CONTEXT* gCM;

/*****************************************************************************
 *
 *  Inline or macro routines
 */

/*
 * ValidateCEPHandle()
 */
static _inline FSTATUS
ValidateCEPHandle(IN IB_HANDLE hCEP)
{
	if (!gCM)
		return FCM_NOT_INITIALIZED;

	if (hCEP == NULL)
		return FINVALID_PARAMETER;

	/* Make sure this is a CEP_OBJECT */
	if (((CM_CEP_OBJECT*)hCEP)->Signature != gCM->Signature)
		return FCM_INVALID_HANDLE;

	return FSUCCESS;
} /* ValidateCEPHandle() */

static _inline boolean
AltPathValid(IN CM_CEP_OBJECT* pCEP)
{
	return (pCEP->AlternatePath.LocalLID != 0);
}


/*****************************************************************************
 *
 *  Events wrapper
 */
static _inline EVENT*
EventAlloc(void)
{
	EVENT* pEvent=NULL;

	pEvent = (EVENT*)MemoryAllocateAndClear(sizeof(EVENT), FALSE, CM_MEM_TAG);
	if (pEvent)
	{
		if (EventInit(pEvent) != TRUE)
		{
			MemoryDeallocate(pEvent);
			pEvent = NULL;
		}
	}
	return pEvent;
}

#define EventDealloc(pEvent) {\
	EventDestroy(pEvent);\
	MemoryDeallocate(pEvent);\
	pEvent = NULL;\
	}

/* EventSet macro is OS dependent. It is moved to cm_private_osd.h */

/*****************************************************************************
 *
 *  Datagram wrapper
 */

/*
 * CmDgrmCopyAddrAndMad()
 */
#define CmDgrmCopyAddrAndMad(pDgrmTo, pMad, pDgrmFrom) {\
	memcpy(GsiDgrmGetSendMad(pDgrmTo), pMad, sizeof(CM_MAD));\
	GsiDgrmAddrCopy(pDgrmTo, pDgrmFrom);\
	}

/*
 * CmDgrmAddrSet()
 */
#define CmDgrmAddrSet(pDgrmAddr, portguid, remotelid, servicelevel, pathbits, staticrate, globalr, dgid, fl, sgidindex, hop, tc, pkeyindex) {\
	(pDgrmAddr)->PathBits		= (uint8)pathbits;\
	(pDgrmAddr)->StaticRate		= (uint8)staticrate;\
	(pDgrmAddr)->PkeyIndex		= pkeyindex;\
	(pDgrmAddr)->ServiceLevel	= (uint8)servicelevel;\
	(pDgrmAddr)->PortGuid		= portguid;\
	(pDgrmAddr)->RemoteLID		= remotelid;\
	(pDgrmAddr)->RemoteQKey		= QP1_WELL_KNOWN_Q_KEY;\
	(pDgrmAddr)->RemoteQP		= CM_WELLKNOWN_QPN;\
	(pDgrmAddr)->GlobalRoute		= globalr;\
	memcpy(&(pDgrmAddr)->GRHInfo.DestGID, &dgid, sizeof(IB_GID));\
	(pDgrmAddr)->GRHInfo.FlowLabel	= fl;\
	(pDgrmAddr)->GRHInfo.SrcGIDIndex= sgidindex;\
	(pDgrmAddr)->GRHInfo.HopLimit	= hop;\
	(pDgrmAddr)->GRHInfo.TrafficClass	= tc;\
	}

/* Release flag */
#define CmDgrmSetReleaseFlag(pDgrm) \
	( (pDgrm)->Element.pContext = (void*)( (uintn)((pDgrm)->Element.pContext) | CM_DGRM_FLAG_RELEASE) )

#define CmDgrmClearReleaseFlag(pDgrm) \
	( (pDgrm)->Element.pContext = (void*)( (uintn)((pDgrm)->Element.pContext) & (~CM_DGRM_FLAG_RELEASE)) )

#define CmDgrmIsReleaseFlagSet(pDgrm) \
	( ( (uintn)((pDgrm)->Element.pContext) & CM_DGRM_FLAG_RELEASE) )

/* In-use flag */
#define CmDgrmSetInUse(pDgrm) \
	( (pDgrm)->Element.pContext = (void*)( (uintn)((pDgrm)->Element.pContext) | CM_DGRM_FLAG_IN_USE) )

#define CmDgrmClearInUse(pDgrm) \
	( (pDgrm)->Element.pContext = (void*)( (uintn)((pDgrm)->Element.pContext) & (~CM_DGRM_FLAG_IN_USE)) )

#define CmDgrmIsInUse(pDgrm) \
	( ( (uintn)((pDgrm)->Element.pContext) & CM_DGRM_FLAG_IN_USE) )

/*
 * CmDgrmSend()
 */
static _inline FSTATUS
CmDgrmSend(IBT_DGRM_ELEMENT* pDgrm)
{
	ASSERT(!CmDgrmIsInUse(pDgrm));
	CmDgrmSetInUse(pDgrm);
	if (iba_gsi_post_send(gCM->GSAHandle, pDgrm) == FSUCCESS)
	{
		return FSUCCESS;
	}
	else
	{
		AtomicIncrement(&gCM->Sent.Failed);
		CmDgrmClearInUse(pDgrm);
		return FERROR;
	}
} /* CmDgrmSend() */

/*
 * CmDgrmSendAndRelease()
 */
static _inline FSTATUS
CmDgrmSendAndRelease(IBT_DGRM_ELEMENT* pDgrm)
{
	ASSERT(!CmDgrmIsInUse(pDgrm));
	CmDgrmSetInUse(pDgrm);
	CmDgrmSetReleaseFlag(pDgrm);
	if (iba_gsi_post_send(gCM->GSAHandle, pDgrm) == FSUCCESS)
	{
		return FSUCCESS;
	}
	else
	{
		CmDgrmClearReleaseFlag(pDgrm);
		CmDgrmClearInUse(pDgrm);
		return FERROR;
	}
} /* CmDgrmSendAndRelease() */

extern void
CmDgrmGetFailed(void);

/*
 * CmDgrmGet()
 */
static _inline IBT_DGRM_ELEMENT*
CmDgrmGet(void)
{
	uint32 Count=1;
	IBT_DGRM_ELEMENT* pDgrm=NULL;

	if (iba_gsi_dgrm_pool_get(gCM->GSADgrmPoolHandle, &Count, &pDgrm) == FSUCCESS)
	{
		ASSERT(pDgrm->Element.pContext == 0);
	} else {
		pDgrm = NULL;
		CmDgrmGetFailed();
	}
	iba_gsi_dgrm_pool_grow_as_needed(gCM->GSADgrmPoolHandle,
						gCmParams.MinFreeDgrms,
						gCmParams.MaxDgrms, gCmParams.DgrmIncrement);
	return pDgrm;
}

/*
 * CmDgrmPut()
 */
#define CmDgrmPut(pDgrm) {\
	if (pDgrm) {\
		ASSERT(!CmDgrmIsInUse(pDgrm));\
		pDgrm->Element.pContext = 0;\
		iba_gsi_dgrm_pool_put(pDgrm);\
		pDgrm = NULL;\
		}\
	}
		
#define CmDgrmRelease(pDgrm) {\
	if (pDgrm) { \
		if (CmDgrmIsInUse(pDgrm)) { \
			CmDgrmSetReleaseFlag(pDgrm); \
		} else { \
			CmDgrmPut(pDgrm); \
		} \
		pDgrm = NULL;\
	} \
}

FSTATUS
PrepareCepDgrm(CM_CEP_OBJECT* pCEP);

/*****************************************************************************
 */

#define CepSetState(pCEP, state) {\
	pCEP->State = state;\
	_DBG_INFO(("<cep 0x%p> *** CEP STATE set to " #state " ***\n", _DBG_PTR(pCEP)));\
	}

/* move a CEP from some other state to Idle, assumes CEP already removed
 * from any state specific list/map for present state
 */
#define CepToIdle(pCEP) { \
	ASSERT(DListIsEmpty(&pCEP->PendingList)); \
	ASSERT(pCEP->PendingCount == 0); \
	ClearEndPoint(pCEP); \
	pCEP->bPeer = FALSE; \
	pCEP->LocalEndPoint.CaGUID = 0; \
	pCEP->bLapSent = 0; \
	pCEP->TargetAckDelay = 0; \
	ClearLocalCommID(pCEP); \
	CepSetState(pCEP, CMS_IDLE); \
	}

#define CepAddToPendingList(pParent, pCEP) { \
	DListInsertTail(&pParent->PendingList, &pCEP->PendingListEntry, PENDING_LIST, pCEP); \
	++(pParent->PendingCount); \
	pCEP->pParentCEP = pParent; \
	}

#define CepRemoveFromPendingList(pCEP) { \
	DListRemoveEntry(&pCEP->PendingListEntry, PENDING_LIST, pCEP); \
	--(pCEP->pParentCEP->PendingCount); \
	pCEP->pParentCEP = NULL; \
	}

/* move CEP to a disconnecting state (as provided) */
#define CepToDisconnecting(pCEP, state) { \
	if (gCmParams.ReuseQP) \
		ClearEndPoint(pCEP); \
	CepSetState(pCEP, state); \
	}

/* move a CEP from some other state to Timewait */
#define CepToTimewait(pCEP) { \
	uint32 timewait_timeout_ms=0; \
\
	if (gCmParams.ReuseQP) \
		ClearEndPoint(pCEP); \
	CepSetState(pCEP, CMS_TIMEWAIT); \
	/* Start the timewait timer to move back to IDLE */ \
	timewait_timeout_ms = TimeoutMultToTimeInMs(pCEP->Timewait); \
	CmTimerStart(pCEP, timewait_timeout_ms, TIMEWAIT_TIMER); \
	}


/*****************************************************************************
 *
 * TimeStamps for elapsed time computations
 */
static _inline uint64
CmGetElapsedTime(CM_CEP_OBJECT* pCEP)
{
	uint64 currtime_us=GetTimeStamp();
	uint64 elapsed_us=0;

	if (currtime_us >= pCEP->timestamp_us)
		elapsed_us = currtime_us - pCEP->timestamp_us;
	else
		elapsed_us = (currtime_us + ((uint64)(-1) - pCEP->timestamp_us));

	return elapsed_us;
}

static _inline uint64
CmComputeElapsedTime(CM_CEP_OBJECT* pCEP, uint64 currtime_us)
{
	uint64 elapsed_us=0;

	if (currtime_us >= pCEP->timestamp_us)
		elapsed_us = currtime_us - pCEP->timestamp_us;
	else
		elapsed_us = (currtime_us + ((uint64)(-1) - pCEP->timestamp_us));

	return elapsed_us;
}

#define CmStartElapsedTime(pCEP) \
	pCEP->timestamp_us = GetTimeStamp();

/*****************************************************************************
 *
 * Timers
 */
#define TIMER_RESOLUTION_US 0

static _inline boolean
CmTimerHasExpired(CM_CEP_OBJECT* pCEP)
{
	uint64 elapsed_us=0;
	uint64 timeinterval_us=pCEP->timeout_ms*1000;

	elapsed_us = CmGetElapsedTime(pCEP);

	if (( (elapsed_us + TIMER_RESOLUTION_US) >= timeinterval_us) && timeinterval_us)
		return TRUE;
	else
		return FALSE;
}


#define CmTimerStart(pCEP, interval_ms, type) {\
	ASSERT(interval_ms);\
	ASSERT(! pCEP->timeout_ms); \
	pCEP->timestamp_us = GetTimeStamp();\
	pCEP->timeout_ms = interval_ms;\
	_DBG_INFO(("<cep 0x%p> >>>"#type " started <lcid 0x%x %s interval %dms timestamp %"PRIu64"us>.\n",\
				_DBG_PTR(pCEP), pCEP->LocalCommID, \
				_DBG_PTR(CmGetStateString(pCEP->State)), \
				pCEP->timeout_ms, pCEP->timestamp_us));\
	TimerStart(&pCEP->TimerObj, pCEP->timeout_ms);\
	CM_MapInsert(&gCM->TimerMap, (uint64)(uintn)pCEP, &pCEP->TimerMapEntry, TIMER_LIST, pCEP);\
	}

#define CmTimerClear(pCEP) {\
	ASSERT(pCEP->timeout_ms); \
	pCEP->timeout_ms=0;\
	CM_MapRemoveEntry(&gCM->TimerMap, &pCEP->TimerMapEntry, TIMER_LIST, pCEP);\
	}

#define CmTimerStop(pCEP, type) {\
	if (CmTimerHasExpired(pCEP))\
	{\
		_DBG_INFO(("<cep 0x%p> <<<"#type " already expired <lcid 0x%x %s timestamp %"PRIu64"us>.\n",\
					_DBG_PTR(pCEP), pCEP->LocalCommID, \
					_DBG_PTR(CmGetStateString(pCEP->State)), \
					GetTimeStamp()));\
	}\
	else\
	{\
		TimerStop(&pCEP->TimerObj);\
		_DBG_INFO(("<cep 0x%p> <<<"#type " stopped <lcid 0x%x %s timestamp %"PRIu64"us>.\n",\
					_DBG_PTR(pCEP), pCEP->LocalCommID, \
					_DBG_PTR(CmGetStateString(pCEP->State)), \
					GetTimeStamp()));\
	}\
	if (pCEP->timeout_ms) \
		CmTimerClear(pCEP);\
	}

/*****************************************************************************/
/* Private function prototypes */

/* Private routines - mostly not thread-safe */
CM_CEP_OBJECT*
CreateCEP(
	IN CM_CEP_TYPE	TransportServiceType,
	IN EVENT_HANDLE	hEvent,
	IN EVENT_HANDLE hWaitEvent
	);

FSTATUS
DestroyCEP(
	IN CM_CEP_OBJECT* pCEP
	);

/* Helper routines */
void AssignLocalCommID(
	IN CM_CEP_OBJECT*			pCEP
	);

void ClearLocalCommID(
	IN CM_CEP_OBJECT*			pCEP
	);

CM_CEP_OBJECT* FindLocalCommID(
	IN uint32					LocalCommID
	);

uint32 GenerateStartPSN(void);

int
CepLocalEndPointCompare(uint64 key1, uint64 key2);

int
CepAddrLocalEndPointCompare(uint64 key1, uint64 key2);

int
CepRemoteEndPointCompare(uint64 key1, uint64 key2);

void
ClearEndPoint(
	IN CM_CEP_OBJECT*			pCEP
	);

void
DgrmDump(
	IN IBT_DGRM_ELEMENT* pDgrm, 
	IN char* pStr, 
	IN boolean bRecv
	);

FSTATUS
SendREJ(
	IN CM_REJECTION_CODE	Reason, 
	IN CMM_REJ_MSGTYPE		MsgRejected,
	IN uint64				TransactionID,
	IN uint32				LocalCommID,
	IN uint32				RemoteCommID,
	IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

FSTATUS
SendMRA(
	IN CMM_MRA_MSGTYPE		MsgMraed,
	IN uint8				ServiceTimeout,
	IN uint64				TransactionID,
	IN uint32				LocalCommID,
	IN uint32				RemoteCommID,
	IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

FSTATUS
SendRTU(
	IN uint64				TransactionID,
	IN uint32				LocalCommID,
	IN uint32				RemoteCommID,
	IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

FSTATUS
SendAPR(
	IN CM_APR_STATUS		APStatus, 
	IN uint64				TransactionID,
	IN uint32				LocalCommID,
	IN uint32				RemoteCommID,
	IBT_DGRM_ELEMENT*		pRemoteDgrm
	);

FSTATUS
SendSIDR_RESP(
	IN uint64 SID,
	IN uint32 QPN,
	IN uint32 QKey,
	IN uint32 Status,
	IN const uint8* pPrivateData,
	IN uint32 DataLen,
	IN uint64 TransactionID,
	IN uint32 RequestID,
	IN IBT_DGRM_ELEMENT*	pRemoteDgrm
	);

FSTATUS
SendDREQ(
	IN uint32 QPN,
	IN uint64 TransactionID,
	IN uint32 LocalCommID,
	IN uint32 RemoteCommID,
	IN IBT_DGRM_ELEMENT*	pRemoteDgrm
	);

FSTATUS
SendDREP(
	IN uint64 TransactionID,
	IN uint32 LocalCommID,
	IN uint32 RemoteCommID,
	IN IBT_DGRM_ELEMENT*	pRemoteDgrm
	);

FSTATUS
CopyMadToConnInfo(
	CONN_INFO* pConnInfo,
	const CM_MAD* pMad
	);

uint32 SidrTimeout(
	IN CM_CEP_OBJECT* pCEP
	);

uint64
UpdateTurnaroundTime(
	IN uint64 turnaround_us,
	IN uint64 elapsed_us
	);

uint32
CepWorstServiceTimeMult(
	IN CM_CEP_OBJECT *pCEP
	);

void
CmClearStats(void);

FSTATUS
RequestCheck(
	IN const CM_REQUEST_INFO* pRequest
	);

boolean
SetNotification(
	IN CM_CEP_OBJECT* pCEP,
	uint32 EventType
	);

void
QueueApcRetry(
	IN CM_CEP_OBJECT* pCEP
	);

void
ApcRetryTimerCallback(
	IN void*					Context
	);

FSTATUS
GetConnInfo(
	IN CM_CEP_OBJECT*			pCEP,
	OUT struct _CM_CONN_INFO*	pConnInfo
	);

void CopyPathInfoToCepPath(
	OUT CM_CEP_PATH *pCepPath,
	IN const CM_CEP_PATHINFO *pPathInfo,
	IN uint8 AckTimeout
	);

void GetAVFromCepPath(
	IN CM_CEP_PATH* CepPath,
	OUT IB_ADDRESS_VECTOR* DestAv
	);

void CopyCepPathToPathInfo(
	OUT CM_CEP_PATHINFO *pPathInfo,
	OUT uint8* pAckTimeout,
	IN CM_CEP_PATH *pCepPath
	);

int CompareCepPathToPathInfo(
	IN const CM_CEP_PATH *pCepPath,
	IN const CM_CEP_PATHINFO *pPathInfo
	);

void GetInitAttrFromCep(
	IN CM_CEP_OBJECT* pCEP,
	OUT IB_QP_ATTRIBUTES_MODIFY* QpAttrInit
	);

void GetRtrAttrFromCep(
	IN CM_CEP_OBJECT* pCEP,
	boolean enableFailover,
	OUT IB_QP_ATTRIBUTES_MODIFY* QpAttrRtr
	);

void GetRtsAttrFromCep(
	IN CM_CEP_OBJECT* pCEP,
	OUT IB_QP_ATTRIBUTES_MODIFY* QpAttrRts
	);

int
CepListenAddrCompare(uint64 key1, uint64 key2);

int
CepReqAddrCompare(uint64 key1, uint64 key2);

int
CepSidrReqAddrCompare(uint64 key1, uint64 key2);

int
CepReqRemoteEndPointCompare(uint64 key1, uint64 key2);

static __inline
boolean CepRemoteEndPointMatchReq(IN CM_CEP_OBJECT* pCEP, IN CMM_REQ *pREQ)
{
	return ( (pCEP->RemoteEndPoint.QPN == pREQ->u1.s.LocalQPN) &&
		(pCEP->RemoteEndPoint.EECN == pREQ->u2.s.LocalEECN) &&
		(pCEP->RemoteEndPoint.CaGUID == pREQ->LocalCaGUID));
}

static __inline
boolean CepRemoteEndPointMatchRep(IN CM_CEP_OBJECT* pCEP, IN CMM_REP *pREP)
{
	return ( (pCEP->RemoteEndPoint.QPN == pREP->u1.s.LocalQPN) &&
		(pCEP->RemoteEndPoint.EECN == pREP->u2.s.LocalEECN) &&
		(pCEP->RemoteEndPoint.CaGUID == pREP->LocalCaGUID));
}

static __inline
boolean CepLocalEndPointMatchReq(IN CM_CEP_OBJECT* pCEP, IN CMM_REQ *pREQ)
{
	return ( (pCEP->LocalEndPoint.CaGUID == pREQ->LocalCaGUID) &&
		(pCEP->LocalEndPoint.QPN == pREQ->u1.s.LocalQPN) &&
		(pCEP->LocalEndPoint.EECN == pREQ->u2.s.LocalEECN));
}

/* TBD move rest of CM_Map macros from cm_common.h */
static __inline
boolean CM_MapTryInsert(CM_MAP* pMap, uint64 Key, CM_MAP_ITEM* pEntry,
				const char* MapName, CM_CEP_OBJECT* pCEP)
{
	if (cl_qmap_insert(pMap, Key, pEntry) != pEntry)
			return FALSE;
	_DBG_INFO(("<cep 0x%p> --> Insert into %s\n", _DBG_PTR(pCEP), _DBG_PTR(MapName)));
	return TRUE;
}

const char*
CmGetStateString(CM_CEP_STATE state);

void
CopyRcvdLAPToAltPath(
	IN const CMM_LAP* pLAP,
	OUT CM_CEP_PATH *pAltPath
	);

void
CopySentLAPToAltPath(
	IN const CMM_LAP* pLAP,
	OUT CM_CEP_PATH *pAltPath
	);

int
CompareRcvdLAPToPath(
	IN const CMM_LAP* pLAP,
	IN const CM_CEP_PATH *pPath
	);

/* Callback routines */
#ifdef _CM_EVENT_ON_QP_

void 
VpdCompletionCallback(
	IN	void *CaContext, 
	IN	void *CqContext
	);

void
VpdAsyncEventCallback(
	IN	void				*CaContext, 
	IN IB_EVENT_RECORD		*EventRecord
	);
#endif

void
GsaSendCompleteCallback(
	IN void* Context,
	IN IBT_DGRM_ELEMENT* pDgrmList
	);

FSTATUS
GsaRecvCallback(
	IN void* Context,
	IN IBT_DGRM_ELEMENT* pDgrmList
	);

void
CmTimerCallback(
	IN void*					Context
	);

void
CmSystemCallback(
	IN void*					Key,
	IN void*					Context
	);

void
CmDoneCallback(
	IN CM_CEP_OBJECT*	pCEP
	);

/* Client-side routines */

FSTATUS
Connect(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_REQUEST_INFO*	pConnectRequest,
	IN boolean					bPeer,
	IN PFN_CM_CALLBACK			pfnConnectCB,
	IN void*					Context
	);

FSTATUS
AcceptA(
	IN CM_CEP_OBJECT*				pCEP,
	IN struct _CM_CONN_INFO* 	pSendConnInfo				/* Send RTU */
	);

FSTATUS
RejectA(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_REJECT_INFO*	pConnectReject
	);

FSTATUS
WaitA(
	IN CM_CEP_OBJECT*			pCEP,
	OUT struct _CM_CONN_INFO*	pConnInfo,
	IN uint32					Timeout_us
	);

FSTATUS
CancelA(
	IN CM_CEP_OBJECT*			pCEP
	);

FSTATUS
ProcessReplyA(
	IN CM_CEP_OBJECT* 			pCEP,
	OUT CM_PROCESS_REPLY_INFO* Info
	);

FSTATUS
AltPathRequestA(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_ALTPATH_INFO*	pLapInfo				/* Send LAP */
	);

FSTATUS
ProcessAltPathReplyA(
	IN CM_CEP_OBJECT* 				pCEP,
	IN const CM_ALTPATH_REPLY_INFO*	AltPathReply,
	OUT IB_QP_ATTRIBUTES_MODIFY*	QpAttrRts
	);

FSTATUS
MigratedA(
	IN CM_CEP_OBJECT*			pCEP
		 );

FSTATUS
MigratedReloadA(
	IN CM_CEP_OBJECT* pCEP,
	OUT CM_PATH_INFO*			NewPrimaryPath OPTIONAL,
	OUT CM_ALTPATH_INFO*		AltPathRequest OPTIONAL
	);

FSTATUS
ShutdownA(
	IN CM_CEP_OBJECT*			pCEP
	);

FSTATUS
DisconnectA(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_DREQUEST_INFO*	pDRequest,	/* Send DREQ */
	IN const CM_DREPLY_INFO*	pDReply		/* Send DREP */
	);

/* Server-side routines */
FSTATUS
Listen(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_LISTEN_INFO*			pListenInfo,
	IN PFN_CM_CALLBACK			pfnListenCB,
	IN void*					Context
	);

FSTATUS
WaitP(
	IN CM_CEP_OBJECT*			pCEP,
	OUT struct _CM_CONN_INFO*	pConnInfo,
	IN uint32					Timeout_us
	);

FSTATUS
ProcessRequestP(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_REP_FAILOVER 			Failover,
	OUT CM_PROCESS_REQUEST_INFO *Info
	);

FSTATUS
AcceptP_Async(
	IN CM_CEP_OBJECT*				pCEP,
	IN const struct _CM_CONN_INFO*	pSendConnInfo,		/* Send REP */
	IN EVENT_HANDLE					hEvent,
	IN EVENT_HANDLE					hWaitEvent,
	IN PFN_CM_CALLBACK				pfnCallback,
	IN void*						Context,
	IN boolean						willWait,
	OUT CM_CEP_OBJECT**				ppCEP
	);

FSTATUS
AcceptP(
	IN CM_CEP_OBJECT*				pCEP,
	IN const struct _CM_CONN_INFO* 	pSendConnInfo,		/* Send REP */
	OUT struct _CM_CONN_INFO*		pRecvConnInfo,		/* Recv RTU */
	IN PFN_CM_CALLBACK				pfnCallback,
	IN void*						Context,
	OUT CM_CEP_OBJECT**				ppNewCEP
	);

FSTATUS
RejectP(
	IN CM_CEP_OBJECT*			pCEP,
	IN const CM_REJECT_INFO*	pConnectReject
	);

FSTATUS
CancelP(
	IN CM_CEP_OBJECT*			pCEP
	);

FSTATUS
ProcessAltPathRequestP(
	IN CM_CEP_OBJECT* pCEP,
	IN CM_ALTPATH_INFO*			AltPathRequest,
	OUT CM_PROCESS_ALTPATH_REQUEST_INFO* Info
	);

FSTATUS
AltPathReplyP(
	IN CM_CEP_OBJECT*			pCEP,
	IN CM_ALTPATH_REPLY_INFO* 	pAprInfo		/* Send APR */
	);

/*
 * CmDeviceInit
 *
 *	Init a device. This routine is called when a new device is detected and added.
 *
 *
 *
 * INPUTS:
 *
 *	None.
 *	
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FERROR
 *
 */
FSTATUS
CmDeviceInit(
	IN	EUI64			CaGuid,
	OUT void			**Context
	);


/*
 * CmDeviceDestroy
 *
 *	Destroy the device. This routine is called when a device is removed.
 *
 *
 *
 * INPUTS:
 *
 *	None.
 *	
 *
 * OUTPUTS:
 *
 *	None.
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FERROR
 *
 */
FSTATUS
CmDeviceDestroy(
	IN	EUI64			CaGuid,
	IN void				*Context
	);

/* Internal use only */
IB_HANDLE
CmCreateCEPEx(
	IN	CM_CEP_TYPE	TransportServiceType,
	IN	EVENT_HANDLE hEvent,
	IN	EVENT_HANDLE hWaitEvent
	);


FSTATUS
CmAcceptEx(
	IN IB_HANDLE			hCEP,
	IN CM_CONN_INFO* 		pSendConnInfo,				/* Send REP, Send RTU */
	IN EVENT_HANDLE			hEvent,
	IN EVENT_HANDLE			hWaitEvent,
	IN PFN_CM_CALLBACK		pfnCallback,				/* For hNewCEP */
	IN void*				Context,
	OUT IB_HANDLE*			hNewCEP
	);

/* This defines some OS specific macros and functions */
#include "cm_private_osd.h"

#if defined (__cplusplus)
};
#endif


#endif  /* _INSI_IB_CM_PRIVATE_H_ */
