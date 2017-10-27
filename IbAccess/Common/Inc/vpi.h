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

#ifndef _IBA_VPI_H_
#define _IBA_VPI_H_

/*
 * This header file declares Common IB types used throughout IB verbs
 * and Service Level Interface
 */

#include "iba/ib_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Access controls. Applies to QPs, memory regions and memory windows.
 * Selectors permit the indicated operation or access mode.
 */

typedef	union _IB_ACCESS_CONTROL {
	uint16	AsUINT16;
	struct _IB_ACCESS_CONTROL_S {
		uint16		AtomicOp:	1;	/* QP only - enable atomics (not on UC QP) */
		uint16		RdmaWrite:	1;	/* Enable Remote RDMA Write */
		uint16		RdmaRead:	1;	/* Enable Remote RDMA Read (not on UC QP) */
		uint16		LocalWrite:	1;	/* MR/MW only, Enable HFI to write */
		uint16		Reserved:	11;
		uint16		MWBindable:	1;	/* MR only, can a MW be bound to MR */
	} s;
} IB_ACCESS_CONTROL;


/*
 * Queue pair definitions and data types
 */

/*
 * Queue pair transport service and special types
 */

typedef enum {
	QPTypeReliableConnected,
	QPTypeUnreliableConnected,
	QPTypeReliableDatagram,
	QPTypeUnreliableDatagram,
	QPTypeSMI,
	QPTypeGSI,
#if defined(INCLUDE_STLEEP)
	QPTypeSTLEEP,
#endif
	QPTypeRawDatagram,
	QPTypeRawIPv6,
	QPTypeRawEthertype
} IB_QP_TYPE;

/* convert IB_QP_TYPE to a string */
IBA_API const char* iba_qp_type_msg(IB_QP_TYPE qp_type);

/*
 * Queue pair states and state transitions
 * used in Modify QP and Query QP operations
 */

typedef enum {
	QPStateNoTransition,
	QPStateReset,		/* After QP creation */
	QPStateInit,		/* Posting of RecvQ WRs enabled */
	QPStateReadyToRecv,	/* RecvQ active */
	QPStateReadyToSend,	/* QP fully active */
	QPStateSendQDrain,	/* Suspend SendQ processing */
	QPStateSendQError,	/* SendQ in error - recovery possible depending upon */
						/* conditions. */
	QPStateError		/* QP in failed state - must reset to proceed */
} IB_QP_STATE;

/* convert IB_QP_STATE to a string */
IBA_API const char* iba_qp_state_msg(IB_QP_STATE qp_state);

/*
 * Completion Flags
 *
 * IB_COMPL_CNTL_FLAGS are or'ed into CompletionFlags
 * These allow a QP to indicate if completions should be controllable per
 * Work Request or should be unconditionally provided for all Work Requests.
 * When a flag is set, the WR has control, when not set, all WR's will
 * result in Completion Queue Entries.
 * Regardless of the setting, failed Work Requests always generate a completion
 *
 * This replaced the old SendSignaledCompletions boolean_t.
 * Provided application used TRUE/FALSE or 0/1 in SendCompletionFlags
 * (which was a boolean_t), this will be binary and source code backward
 * compatible.
 * Applications which only set TRUE/FALSE will never set IB_COMPL_CNTL_RECV
 * and will never see it returned
 */
typedef enum {
	IB_COMPL_CNTL_SEND = 0x01U, /* Enables per WR signaled completions on SendQ  */
	IB_COMPL_CNTL_RECV = 0x02U /* Enables per WR signaled completions on RecvQ. */
							  /* invalid to set if */
							  /* ! CA_IS_UNSIGNALED_RECV_SUPPORTED */
} IB_COMPL_CNTL_FLAGS;

/*
 * States in the automatic path migration state machine. A verbs consumer may
 * request transition to the migrated and rearm states.
 */

typedef enum {
	APMStateNoTransition,
	APMStateMigrated,		/* After QP creation */
	APMStateRearm,			/* Loads specified alternate path address vector */
							/* to QP context. */
	APMStateArmed			/* QP enabled for migration to alternate path */
} IB_QP_APM_STATE;

/* convert IB_QP_APM_STATE to a string */
IBA_API const char* iba_qp_apm_state_msg(IB_QP_APM_STATE qp_apm_state);

/*
 * Global Routing details for use in Address vectors
 * ultimately used to build GRH packet headers in packets which
 * traverse IB routers between IB subnets
 * SrcGIDIndex is always local port, DestGID is always remote port
 */
typedef struct _GLOBALROUTE_INFO {
		IB_GID		DestGID;		/* Dest GID */
		uint32		FlowLabel;
		uint8		SrcGIDIndex;	/* Index into the local port's GID Table */
		uint8		HopLimit;
		uint8		TrafficClass;
} IB_GLOBAL_ROUTE_INFO;

/*
 * The address vector used when creating address vector handles and passed to
 * CreateQP for RC & UC transports.
 */
typedef struct _IB_ADDRESS_VECTOR {
	EUI64		PortGUID;		/* Used only on CreateAV, ModifyAV, QueryAV */
#if INCLUDE_16B
	STL_LID		DestLID;		/* Destination's LID */
#else
	IB_LID		DestLID;		/* Destination's LID */
#endif
	IB_PATHBITS	PathBits;		/* Combines with the base SrcLID to indicate */
								/* SrcLID for pkts */
	IB_SL		ServiceLevel;
	uint8		StaticRate;		/* The maximum static rate supported by this */
								/* endpoint, enum IB_STATIC_RATE */
	boolean		GlobalRoute;	/* If true send Global Route header and the */
								/* GlobalRouteInfo structure is valid. */
	IB_GLOBAL_ROUTE_INFO	GlobalRouteInfo;
} IB_ADDRESS_VECTOR;

/* limit for RetryCount in QP (and CM messages which contain it) */
#define IB_MAX_RETRY_COUNT 7
/* limit for RnrRetryCount in QP (and CM messages which contain it) */
#define IB_MAX_RNR_RETRY_COUNT 6	/* max which is not infinite */
#define IB_RNR_RETRY_COUNT_INFINITE 7	/* retry forever */

/* Flags to indicate which attributes are given during queue pair Modify */
typedef enum {
		/* first set during create */
	IB_QP_ATTR_SENDQDEPTH			=0x00000001U,
	IB_QP_ATTR_RECVQDEPTH			=0x00000002U,
	IB_QP_ATTR_SENDDSLISTDEPTH		=0x00000004U,
	IB_QP_ATTR_RECVDSLISTDEPTH		=0x00000008U,
		/* first set in Reset->Init */
	IB_QP_ATTR_PORTGUID				=0x00000010U,
	IB_QP_ATTR_QKEY 				=0x00000020U,
	IB_QP_ATTR_PKEYINDEX			=0x00000040U,
	IB_QP_ATTR_ACCESSCONTROL		=0x00000080U,
		/* first set in Init->RTR */
	IB_QP_ATTR_RECVPSN				=0x00000100U,
	IB_QP_ATTR_DESTQPNUMBER			=0x00000200U,
	IB_QP_ATTR_PATHMTU				=0x00000400U,
	IB_QP_ATTR_DESTAV				=0x00000800U,
	IB_QP_ATTR_RESPONDERRESOURCES	=0x00001000U,
	IB_QP_ATTR_MINRNRTIMER			=0x00002000U,
		/* first optional in Init->RTR */
	IB_QP_ATTR_ALTPORTGUID			=0x00004000U,
	IB_QP_ATTR_ALTDESTAV			=0x00008000U,
	IB_QP_ATTR_ALTPKEYINDEX			=0x00010000U,
		/* first set in RTR->RTS */
	IB_QP_ATTR_SENDPSN				=0x00020000U,
	IB_QP_ATTR_INITIATORDEPTH		=0x00040000U,
	IB_QP_ATTR_LOCALACKTIMEOUT		=0x00080000U,
	IB_QP_ATTR_RETRYCOUNT			=0x00100000U,
	IB_QP_ATTR_RNRRETRYCOUNT		=0x00200000U,
	IB_QP_ATTR_FLOWCONTROL			=0x00400000U,
		/* first optional in RTR->RTS */
	IB_QP_ATTR_APMSTATE				=0x00800000U,
	IB_QP_ATTR_ALTLOCALACKTIMEOUT	=0x01000000U,
		/* first set in RTS->SQD */
	IB_QP_ATTR_ENABLESQDASYNCEVENT	=0x02000000U
} IB_QP_ATTRS;

/* permitted Attributes in Modify per QP type */
#define QP_ATTR_UD \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_PORTGUID | IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_SENDPSN \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))
	/* TBD BUGBUG - SMI and GSI should not have PKey Index */
#define QP_ATTR_SMI \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_SENDPSN \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))
#define QP_ATTR_GSI \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_SENDPSN \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))
#if defined(INCLUDE_STLEEP)
#define QP_ATTR_STLEEP \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))
#endif
#define QP_ATTR_RD \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_QKEY | IB_QP_ATTR_ACCESSCONTROL \
	| IB_QP_ATTR_MINRNRTIMER \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))
#define QP_ATTR_UC \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_PORTGUID | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_ACCESSCONTROL \
	| IB_QP_ATTR_RECVPSN | IB_QP_ATTR_DESTQPNUMBER | IB_QP_ATTR_PATHMTU \
	| IB_QP_ATTR_DESTAV \
	| IB_QP_ATTR_ALTPORTGUID | IB_QP_ATTR_ALTDESTAV | IB_QP_ATTR_ALTPKEYINDEX \
	| IB_QP_ATTR_SENDPSN | IB_QP_ATTR_APMSTATE \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))
#define QP_ATTR_RC \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_PORTGUID | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_ACCESSCONTROL \
	| IB_QP_ATTR_RECVPSN | IB_QP_ATTR_DESTQPNUMBER | IB_QP_ATTR_PATHMTU \
	| IB_QP_ATTR_DESTAV | IB_QP_ATTR_RESPONDERRESOURCES \
	| IB_QP_ATTR_MINRNRTIMER \
	| IB_QP_ATTR_ALTPORTGUID | IB_QP_ATTR_ALTDESTAV | IB_QP_ATTR_ALTPKEYINDEX \
	| IB_QP_ATTR_SENDPSN | IB_QP_ATTR_INITIATORDEPTH \
	| IB_QP_ATTR_LOCALACKTIMEOUT | IB_QP_ATTR_RETRYCOUNT \
	| IB_QP_ATTR_RNRRETRYCOUNT | IB_QP_ATTR_FLOWCONTROL \
	| IB_QP_ATTR_APMSTATE | IB_QP_ATTR_ALTLOCALACKTIMEOUT \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))

#define QP_ATTR_RAWDG \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_SENDPSN \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))

#define QP_ATTR_RAWIP \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_SENDPSN \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))

#define QP_ATTR_RAWETH \
	((IB_QP_ATTRS) \
	( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
	| IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
	| IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
	| IB_QP_ATTR_SENDPSN \
	| IB_QP_ATTR_ENABLESQDASYNCEVENT ))

/* Manditory and optional attributes per QP state transition
 * 	lists below when and'ed with attributes above per QP type
 * 	yields the exact list of attributes for a given QP type and transition
 */
#define IB_QP_ANY_TO_RESET_MAND ((IB_QP_ATTRS)0)
#define IB_QP_ANY_TO_RESET_OPT ((IB_QP_ATTRS)0)

#define IB_QP_RESET_TO_INIT_MAND \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_PORTGUID | IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
		  |	IB_QP_ATTR_ACCESSCONTROL ))
#define IB_QP_RESET_TO_INIT_OPT  \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH ))

#define IB_QP_INIT_TO_INIT_MAND ((IB_QP_ATTRS)0)
#define IB_QP_INIT_TO_INIT_OPT \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_PORTGUID | IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
		  | IB_QP_ATTR_ACCESSCONTROL \
		  | IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH ))

#define IB_QP_INIT_TO_RTR_MAND \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_RECVPSN | IB_QP_ATTR_DESTQPNUMBER | IB_QP_ATTR_DESTAV \
		  |	IB_QP_ATTR_RESPONDERRESOURCES | IB_QP_ATTR_MINRNRTIMER \
		  | IB_QP_ATTR_PATHMTU))
#define IB_QP_INIT_TO_RTR_OPT \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_ALTDESTAV | IB_QP_ATTR_ALTPORTGUID \
		  | IB_QP_ATTR_ALTPKEYINDEX \
		  | IB_QP_ATTR_QKEY | IB_QP_ATTR_PKEYINDEX \
		  | IB_QP_ATTR_ACCESSCONTROL \
		  | IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH ))

#define IB_QP_RTR_TO_RTS_MAND \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_SENDPSN \
		  | IB_QP_ATTR_INITIATORDEPTH \
		  | IB_QP_ATTR_FLOWCONTROL | IB_QP_ATTR_LOCALACKTIMEOUT \
		  | IB_QP_ATTR_RETRYCOUNT | IB_QP_ATTR_RNRRETRYCOUNT \
		  ))
#define IB_QP_RTR_TO_RTS_OPT \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_APMSTATE | IB_QP_ATTR_ALTLOCALACKTIMEOUT \
		  | IB_QP_ATTR_ACCESSCONTROL | IB_QP_ATTR_QKEY \
		  | IB_QP_ATTR_ALTDESTAV | IB_QP_ATTR_ALTPORTGUID \
		  | IB_QP_ATTR_ALTPKEYINDEX \
		  | IB_QP_ATTR_MINRNRTIMER \
		  | IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH ))

#define IB_QP_RTS_TO_RTS_MAND ((IB_QP_ATTRS)0)
#define IB_QP_RTS_TO_RTS_OPT \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_ACCESSCONTROL | IB_QP_ATTR_QKEY | IB_QP_ATTR_APMSTATE \
		  | IB_QP_ATTR_ALTDESTAV | IB_QP_ATTR_ALTPORTGUID \
		  | IB_QP_ATTR_ALTPKEYINDEX | IB_QP_ATTR_ALTLOCALACKTIMEOUT \
		  | IB_QP_ATTR_MINRNRTIMER \
		  | IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH ))

#define IB_QP_SQE_TO_RTS_MAND ((IB_QP_ATTRS)0)
#define IB_QP_SQE_TO_RTS_OPT \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_ACCESSCONTROL | IB_QP_ATTR_QKEY \
		  | IB_QP_ATTR_MINRNRTIMER \
		  | IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH ))

#define IB_QP_RTS_TO_SQD_MAND \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_ENABLESQDASYNCEVENT))
#define IB_QP_RTS_TO_SQD_OPT ((IB_QP_ATTRS)0)

#define IB_QP_SQD_TO_RTS_MAND ((IB_QP_ATTRS)0)
#define IB_QP_SQD_TO_RTS_OPT \
		((IB_QP_ATTRS) \
		( IB_QP_ATTR_DESTAV	| IB_QP_ATTR_PORTGUID \
		  | IB_QP_ATTR_RETRYCOUNT | IB_QP_ATTR_RNRRETRYCOUNT \
		  | IB_QP_ATTR_FLOWCONTROL | IB_QP_ATTR_LOCALACKTIMEOUT \
		  | IB_QP_ATTR_PATHMTU \
		  | IB_QP_ATTR_ALTDESTAV | IB_QP_ATTR_ALTPORTGUID \
		  | IB_QP_ATTR_ALTPKEYINDEX | IB_QP_ATTR_ALTLOCALACKTIMEOUT \
		  | IB_QP_ATTR_APMSTATE |	IB_QP_ATTR_RESPONDERRESOURCES \
		  | IB_QP_ATTR_INITIATORDEPTH | IB_QP_ATTR_QKEY \
		  | IB_QP_ATTR_PKEYINDEX | IB_QP_ATTR_ACCESSCONTROL \
		  | IB_QP_ATTR_SENDQDEPTH | IB_QP_ATTR_RECVQDEPTH \
		  | IB_QP_ATTR_SENDDSLISTDEPTH | IB_QP_ATTR_RECVDSLISTDEPTH \
		  | IB_QP_ATTR_MINRNRTIMER))

#define IB_QP_ANY_TO_ERR_MAND ((IB_QP_ATTRS)0)
#define IB_QP_ANY_TO_ERR_OPT ((IB_QP_ATTRS)0)

/* QP attributes for input to Create. */
typedef struct _IB_QP_ATTRIBUTES_CREATE {
	IB_QP_TYPE		Type;			/* RC, UC, RD, UD, RawD */
	IB_HANDLE		SendCQHandle;	/* Handle of CQ associated with SendQ */
	IB_HANDLE		RecvCQHandle;	/* Handle of CQ associated with RecvQ */
	IB_HANDLE		PDHandle;		/* Handle for Protection Domain */
	IB_HANDLE		RDDHandle;		/* Reliable Datagram Domain if RD QP */
				/* SendSignaledCompletions as a boolean is depricated */
	uint8			CompletionFlags;/* set bits in IB_COMPL_CNTL_FLAGS */
#define SendSignaledCompletions CompletionFlags	/* depricated */
	uint8			Reserved1;
	uint16			Reserved2;
	uint32			SendQDepth;		/* Outstanding WRs permitted on SendQ */
	uint32			RecvQDepth;		/* Outstanding WRs permitted on RecvQ */
	uint32			SendDSListDepth;/* Max depth of scatter/gather list */
									/* in a WR posted to SendQ */
	uint32			RecvDSListDepth;/* Max depth of scatter list for a WR */
										/* posted to RecvQ */
} IB_QP_ATTRIBUTES_CREATE;
	
/* QP attributes output by Create, Query and Modify */
typedef struct _IB_QP_ATTRIBUTES_QUERY {
	IB_QP_STATE			State;			/* Current QP state */
	boolean				SendDraining;	/* in SQD is Send Q still draining */
	uint8				Reserved1;
	uint16				Reserved2;
		/* first set during create */
	IB_QP_TYPE			Type;			/* RC, UC, RD, UD, RawD */
	uint32				QPNumber;
	IB_HANDLE			SendCQHandle;	/* CQ Handle associated with SendQ */
	IB_HANDLE			RecvCQHandle;	/* CQ Handle associated with RecvQ */
	IB_HANDLE			PDHandle;
	IB_HANDLE			RDDHandle;		/* Reliable Datagram Domain if RD */
				/* SendSignaledCompletions as a boolean is depricated */
	uint8				CompletionFlags;/* set bits in IB_COMPL_CNTL_FLAGS */
#define SendSignaledCompletions CompletionFlags	/* depricated */
	uint8				Reserved3;
	uint16				MaxInlineData;	/* depricated, for VAPI compatibility */
										/* valid when SendDSListDepth is valid */
	uint32				Attrs;		/* Flags to indicate the valid */
									/* attributes below. IB_QP_ATTRS */
	uint32				SendQDepth;		/* Outstanding WRs permitted on SQ */
	uint32				RecvQDepth;		/* Outstanding WRs permitted on RQ */
	uint32				SendDSListDepth;/* Max depth of scatter/gather list */
										/* in a WR posted to SendQ */
	uint32				RecvDSListDepth;/* Max depth of scatter list for a */
										/* WR posted to RecvQ */
		/* first set in Reset->Init */
	EUI64				PortGUID;		/* QP is assigned to this port */
	uint16				PkeyIndex;		/* Partition table entry in use */
	uint16				Reserved5;
	IB_Q_KEY			Qkey;			/* Client-supplied key */
	IB_ACCESS_CONTROL	AccessControl;
	uint16				Reserved6;
		/* first set in Init->RTR */
	uint32				RecvPSN;		/* Expected PSN on receive */
	uint32				DestQPNumber;
	uint8				PathMTU;		/* an IB_MTU */
	uint8				Reserved7;
	uint16				Reserved8;
	IB_ADDRESS_VECTOR	DestAV;			/* Dest address vector for RC & UC */
	uint8				ResponderResources; /* Number of outstanding RDMA */
										/* Read & Atomic ops permitted by */
										/* this QP as responder */
										/* should be >= remote QPs */
										/* InitiatorDepth */
	uint8				MinRnrTimer;	/* Receiver Not Ready NAK Timer */
	uint16				Reserved9;
		/* first optional in Init->RTR */
	EUI64				AltPortGUID;	/* Alternate path port */
	IB_ADDRESS_VECTOR	AltDestAV;		/* Alt path dest AV if RC */
	uint16				AltPkeyIndex;	/* Alt path Partition table entry */
	uint16				Reserved10;
		/* first set in RTR->RTS */
	uint32				SendPSN;		/* Next PSN to transmit */
	uint8				InitiatorDepth; /* Number of outstanding RDMA */
										/* Read & Atomic which can be */
										/* outstanding with this QP */
										/* as a requestor/initiator */
										/* should be <= remote QPs */
										/* ResponderResources */
	uint8				LocalAckTimeout;
	uint8				RetryCount;		/* Sequence error retry count */
	uint8				RnrRetryCount;	/* Receiver Not Ready retry count */
	boolean				FlowControl;	/* Does dest provide this endpoint */
										/* with end-to-end flow control */
										/* credits? */
		/* first optional in RTR->RTS */
	IB_QP_APM_STATE		APMState;		/* Current Automatic Path Migration */
										/* state. */
	uint8				AltLocalAckTimeout;
		/* first set in RTS->SQD */
	boolean				EnableSQDAsyncEvent;	/* Only valid in SQD state */
} IB_QP_ATTRIBUTES_QUERY;

/* QP attributes input to Modify */
typedef struct _IB_QP_ATTRIBUTES_MODIFY {
	IB_QP_STATE			RequestState;	/* required */
	uint32				Attrs;		/* Flags to indicate the supplied */
									/* attributes. IB_QP_ATTRS */
		/* only atttibutes below which have corresponding bit set */
		/* in Attrs will be processed.  Others assumed to have undefined */
		/* value and will be ignored.  Grouped by state 1st set in */

		/* first set during create */
	uint32				SendQDepth;		/* Outstanding WRs permitted on SQ */
	uint32				RecvQDepth;		/* Outstanding WRs permitted on RQ */
	uint32				SendDSListDepth;/* Max depth of scatter/gather list */
										/* in a WR posted to SendQ */
	uint32				RecvDSListDepth;/* Max depth of scatter list */
										/* in a WR posted to RecvQ */
		/* first set in Reset->Init */
	EUI64				PortGUID;	/* Port to use */
	IB_Q_KEY			Qkey;
	uint16				PkeyIndex;	/* Partition table entry */
	IB_ACCESS_CONTROL	AccessControl;
		/* first set in Init->RTR */
	uint32				RecvPSN;	/* Expected PSN on receive */
	uint32				DestQPNumber;
	uint8				PathMTU;	/* an IB_MTU */
									/* implicitly IB_MTU_256 for SMI/GSI */
	uint8				Reserved1;
	uint16				Reserved2;
	IB_ADDRESS_VECTOR	DestAV;		/* Dest AV for RC & UC */
	uint8				ResponderResources; /* for RC & RD */
										/* Number of outstanding RDMA */
										/* Read & Atomic ops permitted by */
										/* this QP as responder */
										/* should be >= remote QPs */
										/* InitiatorDepth */
	uint8				MinRnrTimer;	/* Receiver Not Ready NAK Timer */
										/* for RC & UC only */
	uint16				Reserved3;
		/* first optional in Init->RTR */
	EUI64				AltPortGUID;	/* Alt path port */
	IB_ADDRESS_VECTOR	AltDestAV;		/* Alt path dest AV if RC */
	uint16				AltPkeyIndex;	/* Alt path Partition */
										/* table entry to use */
	uint16				Reserved4;
		/* first set in RTR->RTS */
	uint32				SendPSN;		/* First PSN to transmit */
	uint8				InitiatorDepth; /* for RC & RD */
										/* Number of outstanding RDMA */
										/* Read & Atomic which can be */
										/* outstanding with this QP */
										/* as a requestor/initiator */
										/* should be <= remote QPs */
										/* ResponderResources */
	uint8				LocalAckTimeout;/* Local ACK timeout */
	uint8				RetryCount;		/* Sequence error retry count */
	uint8				RnrRetryCount;	/* Receiver Not Ready retry count */
	boolean				FlowControl;	/* Does dest provide this */
										/* endpoint with end-to-end */
										/* flow control credits? */
		/* first optional in RTR->RTS */
	IB_QP_APM_STATE		APMState;		/* Requested Auto Path Mig */
										/* state transition */
	uint8				AltLocalAckTimeout;
		/* first set in RTS->SQD */
	boolean				EnableSQDAsyncEvent;	/* Async Event when QP */
										/* drained in SQD state */
} IB_QP_ATTRIBUTES_MODIFY;

/*
 * Completion queue definitions and data types
 */

/*
 * Arms a completion queue for event generate on CQ update
 */

typedef enum {
	CQEventSelNextWC,	/* The next work completion written to the CQ will */
						/* generate an event. */
	CQEventSelNextSolicited	/* Only generates an event if the work completion */
						/* is for a received msg where the solicited event was */
						/* requested by the sender. */
} IB_CQ_EVENT_SELECT;


/*
 * Verbs consumer completion notification event and asynchronous event
 * callbacks and associated data types.
 */

/*
 * Async event types. Generally the event type indicates the resource type
 */

typedef enum {
	AsyncEventQP = 1,
	AsyncEventSendQ,
	AsyncEventRecvQ,
	AsyncEventCQ,
	AsyncEventCA,
	AsyncEventPort,			/* Port moved to Active or non-active state */
	AsyncEventPathMigrated,	/* APM has migrated QP to AlternatePath */
	AsyncEventEE,
	AsyncEventEEPathMigrated
} IB_ASYNC_EVENT;



/*
 * Send queue async event codes
 */

#define IB_AE_SQ_DOORBELL_OVRFLW	1
#define IB_AE_SQ_WQE_ACCESS			2	/* local access violation */
#define	IB_AE_SQ_ERROR_STATE		3
#define IB_AE_SQ_DRAIN				4	/* SendQ is drained when in SQD */

/*
 * Recv queue async event codes
 */

#define IB_AE_RQ_DOORBELL_OVRFLW	1
#define IB_AE_RQ_WQE_ACCESS			2	/* local access violation */
#define	IB_AE_RQ_ERROR_STATE		3
#define	IB_AE_RQ_COMM_ESTABLISHED	4	/* 1st packet received in RTR */
#define IB_AE_RQ_PATH_MIG_ERR		5

/*
 * Completion queue async event codes
 */

#define IB_AE_CQ_SW_ACCESS_INVALID_CQ	0
#define IB_AE_CQ_HW_ACCESS_INVALID_CQ	1
#define IB_AE_CQ_ACCESS_ERR				IB_AE_CQ_HW_ACCESS_INVALID_CQ
#define IB_AE_CQ_PROT_INDEX_OB			2
#define IB_AE_CQ_OVERRUN				IB_AE_CQ_PROT_INDEX_OB
#define	IB_AE_CQ_INVALID_TPT_ENTRY		3
#define IB_AE_CQ_INVALID_PROT_DOMAIN	4
#define IB_AE_CQ_NO_WRITE_ACCESS		5

/* EE async event codes */
#define IB_AE_EE_ERROR_STATE			3
#define	IB_AE_EE_COMM_ESTABLISHED		4	/* 1st packet received in RTR */
#define IB_AE_EE_PATH_MIG_ERR			5

/* IBTA Events are mapped as follows:
 * IBTA Event		EventType			EventCode			Context
 * ----------------------------------------------------------------
 * Path Migrated	PathMigrated		0					QP
 * Path Mig Error	RecvQ				RQ_PATH_MIG_ERR		QP
 * Comm Established	RecvQ				RQ_COMM_ESTABLISHED	QP
 * Send Q Drained	SendQ				SQ_DRAIN			QP
 * CQ Access Error	CQ					CQ_ACCESS_ERR		CQ
 * CQ Overrun		CQ					CQ_OVERRUN			CQ
 * Local Q Cat.		RecvQ				RQ_ERROR_STATE		QP
 * Invalid Q Req 	RecvQ				RQ_ERROR_STATE		QP
 * Local Q Acc Viol	RecvQ				RQ_WQE_ACCESS		QP
 * Local EE Cat.	EE					EE_ERROR_STATE		EE
 * EE Comm Est.		EE					EE_COMM_ESTABLISHED	EE
 * EE Path Migrated	EEPathMigrated		0					EE
 * EE Path Mig Err	EE					EE_PATH_MIG_ERR		EE
 * Local CA Cat.	CA					0					None
 * Port Active		Port				new port state		Port Number
 * Port Down		Port				new port state		Port Number
 * Other QP Error	QP					0					QP
 *
 * Some QP errors are directed toward the RecvQ, RecvQ errors are also
 * directed toward the CM to affect CEP state transitions
 *
 * Async event callbacks include the caller CA Context as an argument.
 * In addition where indicated above, the EventRecord.Context has the
 * caller context for the QP, CQ or EE.
 */

/*
 * Async event record format
 */
typedef struct _IB_EVENT_RECORD {
	void			*Context;	/* Context assigned on resource open/create/set */
								/* context - QP, CQ, CA. */
								/* or for AsyncEventPort, 1 based port # */
	IB_ASYNC_EVENT	EventType;
	uint32			EventCode;	/* Error or informational code specific to */
								/* resource generating async event. */
} IB_EVENT_RECORD;

/* convert IB_EVENT_RECORD to a string summarizing EventType/EventCode */
IBA_API const char* iba_event_record_msg(IB_EVENT_RECORD *p_event);

/*
 * The completion event handler provided by the verbs consumer
 */

typedef void
(*IB_COMPLETION_CALLBACK)(
	IN void *CaContext,
	IN void *CqContext
	);

/*
 * The asynchronous event handler provided by the verbs consumer
 * data pointed to only valid during duration of call
 */

typedef void
(*IB_ASYNC_EVENT_CALLBACK)(
	IN void *CaContext,
	IN IB_EVENT_RECORD *EventRecord
	);


/*
 * Work request and work completion definitions and data types
 */

/*
 * Work request operation types
 */

typedef enum {
	WROpSend = 0,
	WROpRdmaWrite,
	WROpRecv,
	WROpRdmaRead,
	WROpMWBind,
	WROpFetchAdd,
	WROpCompareSwap,
	WROpRecvUnsignaled	/* When IB_COMPL_CNTL_RECV is set in QP CompletionFlags */
						/* this can be used to issue a Recv WR without */
						/* a completion */
						/* (For backward compatibility reasons the Recv */
						/* WR does not have an Options field) */
} IB_WR_OP;

/* convert IB_WR_OP to a string */
IBA_API const char* iba_wr_op_msg(IB_WR_OP wr_op);

/*
 * Status codes found in work completions
 */

typedef enum {
	WRStatusSuccess,
	WRStatusLocalLength,	/* incoming RecvQ msg exceeded available data segs */
							/* the in RQ work request or max message for CA */
							/* outgoing SendQ msg exceeded max message for CA */
	WRStatusLocalOp,		/* Internal QP consistency error */
	WRStatusLocalProtection,/* local Data Segment references invalid MR for */
							/* requested operation */
	WRStatusTransport,		/* unknown transport error */
	WRStatusWRFlushed,		/* work request was flushed before completing */
	WRStatusRemoteConsistency,
	WRStatusMWBind,			/* MW bind error, insufficient access rights */
	WRStatusRemoteAccess,	/* remote mem access error on RDMA read/write or */
							/* atomic */
	WRStatusRemoteOp,		/* local prot error on the remote RQ */
	WRStatusRemoteInvalidReq,/* remote endpt does not support this work req op */
							/* - too little buffers for new RDMA or Atomic */
							/* - RDMA length too large */
							/* - operation not supported by RQ */
	WRStatusSeqErrRetry,	/* Sequence error retry limit exceeded */
	WRStatusRnRRetry,		/* RnR NAK retry limit exceeded */
	WRStatusTimeoutRetry,	/* Timeout error retry limit exceeded */
	WRStatusBadResponse,	/* unexpected transport opcode returned */
	WRStatusInvalidLocalEEC,/* Invalid EE Context Number */
	WRStatusInvalidLocalEECState,/* Invalid operation for EE Context State */
	WRStatusInvalidLocalEECOp,	/* internal EEC consistency error detected */
	WRStatusLocalRDDViolation,	/* RDD for local QP doesn't match RDD */
								/* of remote EEC */
	WRStatusRemoteInvalidRDReq,	/* remote rejected RD message */
								/* invalid QKey or RDD Violation */
	WRStatusLocalAccess,	/* protection error on local data buffer */
							/* during processing of inbound RDMA Write w/immed */
	WRStatusRemoteAborted	/* remote requestor aborted operation */
							/* - requester suspended */
							/* - requester moved QP to SQEr */
} IB_WC_STATUS;

/* convert IB_WC_STATE to a string */
IBA_API const char* iba_wc_status_msg(IB_WC_STATUS wc_status);

/* Pack the data segment structures, manually make sure multiple of 64 bits */
#include "iba/public/ipackon.h"

/*
 * Each data segment is an element of a scatter/gather list and describes a
 * local buffer.
 */

typedef struct _IB_LOCAL_DATASEGMENT {
	uint64			Address;
	IB_L_KEY		Lkey;
	uint32			Length;
} PACK_SUFFIX IB_LOCAL_DATASEGMENT;

/*
 * This data segment describes the contiguous buffer on a remote endpoint
 * which is the target of a read or write RDMA operation.
 */

typedef struct _IB_REMOTE_DATASEGMENT {
	uint64			Address;
	IB_R_KEY		Rkey;
	uint32			Reserved;
} PACK_SUFFIX IB_REMOTE_DATASEGMENT;

#include "iba/public/ipackoff.h"

/*
 * Options for send work queue requests. Also used on receive work queue
 * completions to indicate the presence of immediate data or a solicited
 * event. 
 */

typedef union _IB_SEND_OPTIONS {
	uint16	AsUINT16;
	struct _IB_SEND_OPTIONS_S {
		uint16		Reserved1:			3;	/* Must be zero */
		uint16		InlineData:			1;	/* post as inline send or RDMA Write */
											/* L-Keys in DSList[] ignored */
											/* depricated, for VAPI */
											/* compatibility only */
											/* N/A for SMI/GSI nor Raw QPs */
											/* N/A for RdmaRead nor Atomics */
		uint16		ImmediateData:		1;	/* N/A for RdmaRead nor Atomics */
		uint16		Fence:				1;	/* only for RC and RD */
		uint16		SignaledCompletion:	1;	/* Generate CQ entry on sendQ work */
											/* completion. */
		uint16		SolicitedEvent:		1;	/* Generate solicited event at */
											/* destination. N/A for RdmaRead */
											/* nor Atomics */
#if INCLUDE_16B
		uint16		SendFMH:			1;	/* Use 16b and FM Header for mad requests */
		uint16		Reserved2:			7;	/* Must be zero */
#else
		uint16		Reserved2:			8;	/* Must be zero */
#endif
	} s;
} IB_SEND_OPTIONS;

/*
 * Options for the bind operation
 */

typedef union _IB_MW_BIND_OPTIONS {
	uint16	AsUINT16;
	struct _IB_MW_BIND_OPTIONS_S {
		uint16		Reserved1:			5;	/* Must be zero */
		uint16		Fence:				1;
		uint16		SignaledCompletion:	1;	/* Generate CQ entry on sendQ work */
											/* completion. */
		uint16		Reserved2:			9;	/* Must be zero */
	} s;
} IB_MW_BIND_OPTIONS;

/*
 * The work request structure for all operations on the send and receive work
 * queues.
 */

/* this is depricated, use IB_WORK_REQ2 family of structures */
typedef struct _IB_WORK_REQ {
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint32					MessageLen;
	IB_WR_OP				Operation;
	union _IB_WR_REQ_TYPE {
		/* no additional information for Recv WQEs */
		struct _IB_SEND_CONNECTED {
			uint32			ImmediateData;
			IB_SEND_OPTIONS	Options;
			IB_REMOTE_DATASEGMENT	RemoteDS;
		} SendRC, SendUC;	/* Send, RdmaRead, RdmaWrite */
		struct _IB_ATOMIC_RC {
			IB_SEND_OPTIONS	Options;
			IB_REMOTE_DATASEGMENT	RemoteDS;
			uint64		SwapAddOperand;	/* for CompareSwap or FetchAdd */
			uint64		CompareOperand;	/* for CompareSwap only */
		} AtomicRC;			/* CompareSwap, FetchAdd */
		struct _IB_SEND_UD {
			uint32		ImmediateData;
			IB_SEND_OPTIONS	Options;
			uint32		QPNumber;	/* Dest QP number */
			IB_Q_KEY	Qkey;
			IB_HANDLE	AVHandle;	/* Handle to AV for dest */
			IB_P_KEY	PkeyIndex;	/* GSI only */
		} SendUD;			/* Send */
		struct _IB_RECV_UD {
			uint32		Reserved;
		} RecvUD, RecvRC, RecvUC, RecvRD;
		struct _IB_SEND_RD {	/* not supported */
			uint32		ImmediateData;
			IB_SEND_OPTIONS	Options;
			uint32		QPNumber;	/* Dest QP number */
			IB_Q_KEY	Qkey;
			uint32		DestEECNumber;	/* Dest EE Context number */
			IB_REMOTE_DATASEGMENT	RemoteDS;
		} SendRD;			/* Send, RdmaRead, RdmaWrite */
		struct _IB_SEND_RAWD {
			uint32		QPNumber;	/* Dest QP number */
#if INCLUDE_16B
			STL_LID		DestLID;	/* Destination's Base LID */
#else
			IB_LID		DestLID;	/* Destination's Base LID */
#endif
			IB_PATHBITS	PathBits;	/* Combines with the base SrcLID to */
									/* determine SrcLID for pkts. */
			IB_SL		ServiceLevel;	/* For dest */
			uint8		SignaledCompletion:1; /* Generate CQ entry on sendQ work */
											/* completion. */
			uint8		SolicitedEvent:1;	/* Generate solicited event at */
											/* destination. */
			uint8		StaticRate:6;		/* The maximum static rate */
											/* enum IB_STATIC_RATE */
			uint16		EtherType;			/* Ethernet type */
		} SendRawD;			/* Send */
		struct _IB_MW_BIND_REQ {
			IB_HANDLE	MWHandle;
			IB_R_KEY	CurrentRkey;
			IB_HANDLE	MRHandle;
			IB_LOCAL_DATASEGMENT	MWBindSegment;
			IB_ACCESS_CONTROL		AccessControl;
			IB_MW_BIND_OPTIONS		Options;
			IB_R_KEY				NewRkey;
		} MWBind;			/* Bind */
	} Req;
} IB_WORK_REQ;

/* Pack the work req structures so they can be overlaid, manually ensure
 * natural alignment of all fields
 */
#include "iba/public/ipackon.h"

/* supporting structures used in IB_WORK_REQ2
 * field names are upward compatible with IB_WORK_REQ, however have been
 * reordered to optimize memory usage
 */
/* Send, RdmaRead, RdmaWrite for UC or RC QP */
struct _IB_SEND_CONNECTED2 {
	IB_SEND_OPTIONS			Options;
	uint16					Reserved1;
	uint32					ImmediateData;
	IB_REMOTE_DATASEGMENT	RemoteDS;	/* for RdmaRead/RdmaWrite only */
} PACK_SUFFIX;
/* CompareSwap, FetchAdd for RC QP */
struct _IB_ATOMIC_RC2 {
	IB_SEND_OPTIONS			Options;
	uint16					Reserved1;
	uint32					Reserved2;
	IB_REMOTE_DATASEGMENT	RemoteDS;
	uint64					SwapAddOperand;	/* for CompareSwap or FetchAdd */
	uint64					CompareOperand;	/* for CompareSwap only */
} PACK_SUFFIX;
/* Send for UD, SMI or GSI QP */
struct _IB_SEND_UD2 {
	IB_SEND_OPTIONS			Options;
	IB_P_KEY				PkeyIndex;	/* GSI only */
	uint32					ImmediateData;
	uint32					QPNumber;	/* Dest QP number */
	IB_Q_KEY				Qkey;
	IB_HANDLE				AVHandle;	/* Handle to AV for dest */
} PACK_SUFFIX;
/* Send, RdmaRead, RdmaWrite for RD QP */
struct _IB_SEND_RD2 {
	IB_SEND_OPTIONS			Options;
	uint16					Reserved1;
	uint32					ImmediateData;
	uint32					QPNumber;	/* Dest QP number */
	IB_Q_KEY				Qkey;
	uint32					DestEECNumber;	/* Dest EE Context number */
	uint32					Reserved2;
	IB_REMOTE_DATASEGMENT	RemoteDS;	/* for RdmaRead/RdmaWrite only */
} PACK_SUFFIX;
/* CompareSwap, FetchAdd for RD QP */
struct _IB_ATOMIC_RD2 {
	IB_SEND_OPTIONS			Options;
	uint16					Reserved1;
	uint32					QPNumber;	/* Dest QP number */
	IB_Q_KEY				Qkey;
	uint32					DestEECNumber;	/* Dest EE Context number */
	IB_REMOTE_DATASEGMENT	RemoteDS;
	uint64					SwapAddOperand;	/* for CompareSwap or FetchAdd */
	uint64					CompareOperand;	/* for CompareSwap only */
} PACK_SUFFIX;
/* Send for Raw QP */
struct _IB_SEND_RAWD2 {
	uint32					QPNumber;	/* Dest QP number */
#if INCLUDE_16B
	STL_LID					DestLID;	/* Destination's Base LID */
#else
	IB_LID					DestLID;	/* Destination's Base LID */
#endif
	IB_PATHBITS				PathBits;	/* Combines with the base SrcLID to */
										/* determine SrcLID for pkts. */
	IB_SL					ServiceLevel;	/* For dest */
	uint8					SignaledCompletion:1; /* Generate CQ entry on */
												  /* SendQ work completion. */
	uint8					SolicitedEvent:1; /* Generate solicited event at */
											  /* destination. */
	uint8					StaticRate:6;/* The maximum static rate */
										 /* enum IB_STATIC_RATE */
	uint8					Reserved;
	uint16					EtherType;	 /* Ethernet type */
} PACK_SUFFIX;

#if defined(INCLUDE_STLEEP)
/* Send for the STLEEP QP */
struct _OPA_SEND_STLEEP2 {
	IB_SEND_OPTIONS			Options;
} PACK_SUFFIX;
#endif

/* this family of structures allows for future addition of new fields as well
 * as optimized memory usage by applications.  The APIs accept a IB_WORK_REQ2
 * structure, however, an appropriate variation from the IB_WORK_REQ2_* below
 * can be provided and cast.  The variations below are all a subset of the main
 * structure.  The API will select the _IB_WR_REQ_TYPE in the union based
 * on QP type and Operation.  Use of additional fields in the individual REQ
 * types is guided by Options selected.
 * In the future the IB_WORK_REQ2 structure may have new Req fields added to the
 * union or new fields may be added to individual request types.  However
 * binary compatibility will be maintained by only using fields indicated by
 * the Options, QP Type and Operation.  Applications must be careful to
 * initialize the Operation and clear any unused or reserved bits in Options.
 * Note with the exception of the new Next field, the field names have the same
 * names as in IB_WORK_REQ, hence making it easier to migrate existing
 * applications.
 */
typedef struct _IB_WORK_REQ2 {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;	/* operation to perform */
	uint32					Reserved;	/* 64 bit align Req union below */
	union _IB_WR_REQ_TYPE2 {
		/* no additional information for Recv WQEs */
		struct _IB_SEND_CONNECTED2 SendRC, SendUC;/* Send,RdmaRead,RdmaWrite */
		struct _IB_ATOMIC_RC2 AtomicRC;			/* CompareSwap, FetchAdd */
		struct _IB_SEND_UD2 SendUD;				/* Send */
		struct _IB_SEND_RD2 SendRD;				/* Send, RdmaRead, RdmaWrite */
		struct _IB_ATOMIC_RD2 AtomicRD;			/* CompareSwap, FetchAdd */
		struct _IB_SEND_RAWD2 SendRawD;			/* Send */
#if defined(INCLUDE_STLEEP)
		struct _OPA_SEND_STLEEP2 SendSTLEEP;	/* Send */
#endif
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2;

/* storage optimized subsets of IB_WORK_REQ2 */

/* receive WQE for any QP type */
typedef struct _IB_WORK_REQ2_RECV {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	/* no additional information for Recv WQEs */
} PACK_SUFFIX IB_WORK_REQ2_RECV;

/* Send, RdmaRead or RdmaWrite for RC or UC QP */
typedef struct _IB_WORK_REQ2_RCUC_SEND {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_CONNECTED2 SendRC, SendUC;/* Send,RdmaRead,RdmaWrite */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_RCUC_SEND;

/* any SendQ or RecvQ operation on a RC QP */
typedef struct _IB_WORK_REQ2_RC {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_CONNECTED2 SendRC;		/* Send, RdmaRead, RdmaWrite */
		struct _IB_ATOMIC_RC2 AtomicRC;			/* CompareSwap, FetchAdd */
		/* no additional information for Recv WQEs */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_RC;

/* any SendQ or RecvQ operation on a UD QP */
typedef struct _IB_WORK_REQ2_UD {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_UD2 SendUD;			/* Send */
		/* no additional information for Recv WQEs */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_UD;

/* any SendQ or RecvQ operation on a UC QP */
typedef struct _IB_WORK_REQ2_UC {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_CONNECTED2 SendUC;/* Send, RdmaWrite */
		/* no additional information for Recv WQEs */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_UC;

/* Send on a RD QP */
typedef struct _IB_WORK_REQ2_RD_SEND {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_RD2 SendRD;		/* Send, RdmaRead, RdmaWrite */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_RD_SEND;

/* any SendQ or RecvQ operation on a RD QP */
typedef struct _IB_WORK_REQ2_RD {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_RD2 SendRD;		/* Send, RdmaRead, RdmaWrite */
		struct _IB_ATOMIC_RD2 AtomicRD;	/* CompareSwap, FetchAdd */
		/* no additional information for Recv WQEs */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_RD;

/* any SendQ or RecvQ operation on a RAW QP */
typedef struct _IB_WORK_REQ2_RAW {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _IB_SEND_RAWD2 SendRawD;			/* Send */
		/* no additional information for Recv WQEs */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_RAW;

#if defined(INCLUDE_STLEEP)
/* any SendQ or RecvQ operation on the STLEEP QP */
typedef struct _IB_WORK_REQ2_STLEEP {
	struct _IB_WORK_REQ2	*Next;		/* link to next WQE being posted
										 * allows for more efficient operation
										 * by posting multiple WQEs in 1 call
										 */
	IB_LOCAL_DATASEGMENT	*DSList;	/* Ptr to scatter/gather list */
	uint64					WorkReqId;	/* Consumer supplied value returned */
										/* on a work completion */
	uint32					MessageLen;	/* overall size of message */
	uint32					DSListDepth;/* Number of data segments in */
										/* scatter/gather list. */
	IB_WR_OP				Operation;
	uint32					Reserved;	/* 64 bit align Req union below */
	union {
		struct _OPA_SEND_STLEEP2 SendSTLEEP;	/* Send */
		/* no additional information for Recv WQEs */
	} PACK_SUFFIX Req;
} PACK_SUFFIX IB_WORK_REQ2_STLEEP;
#endif

#include "iba/public/ipackoff.h"

/*
 * Completion type indicating the local operation performed
 */

typedef enum {
	WCTypeSend = 0,		/* SendQ */
	WCTypeRdmaWrite,	/* SendQ */
	WCTypeRecv,			/* RecvQ (WROpRecv or failed WROpRecvUnsignaled) */
	WCTypeRdmaRead,		/* SendQ */
	WCTypeMWBind,		/* SendQ */
	WCTypeFetchAdd,		/* SendQ */
	WCTypeCompareSwap	/* SendQ */
} IB_WC_TYPE;

/* convert IB_WC_TYPE to a string */
IBA_API const char* iba_wc_type_msg(IB_WC_TYPE wc_type);

typedef enum {
	WCRemoteOpSend = 0,	/* RecvQ remote did a send w/ or w/o immed */
	WCRemoteOpRdmaWrite	/* RecvQ remote did RdmaWrite w/immed */
} IB_WC_REMOTE_OP;

/* convert IB_WC_REMOTE_OP to a string */
IBA_API const char* iba_wc_remote_op_msg(IB_WC_REMOTE_OP wc_remote_op);

/*
 * Flags used to indicate conditions on completed receive work requests
 */

typedef union _IB_RECV_FLAGS {
	uint8	AsUINT8;
	struct _IB_RECV_FLAGS_S {
		uint8	GlobalRoute:	1;	/* Global route header present in */
									/* consumer's data buffer. */
		uint8	Reserved:		7;
	} s;
} IB_RECV_FLAGS;

/*
 * The work completion structure for all completed work requests
 *
 * This structure is a union to reduce overall size.  Hence the caller
 * must be aware of the QP type on receive completions in order to properly
 * identify which fields in the union to use.
 *
 * The simple approach is to only put QPs of the same type on a given CQ
 * (some Verbs drivers may force this limitation).
 *
 * An alternative is to use information in the WorkReqId (such as a session
 * pointer, etc) to identify the QP type the completion is for.
 *
 * Note that information relevant to failed completions is mostly in common
 * fields.  The exception being the RemoteOp field for RC and UC recv failures.
 * Hence there is no need for QP identification for most failures (such as
 * WRStatusWRFlushed).
 *
 * if Status != WRStatusSuccess, only the following fields are guaranteed to
 * be valid: WorkReqId, Status
 */

typedef struct _IB_WORK_COMPLETION {
	uint64			WorkReqId;	/* Value supplied by the consumer on the WR */
	IB_WC_TYPE		WcType;
	IB_WC_STATUS	Status;
	uint32			Length;		/* undefined on send completion for some HFIs */

	/* the fields below are only defined on recv completions */
	union _IB_WC_REQ {
		struct _IB_WC_RECV_CONNECTED {
			IB_SEND_OPTIONS	Options;	/* Possible flags are ImmediateData */
			uint32			ImmediateData;
			IB_WC_REMOTE_OP	RemoteOp;
		} RecvRC, RecvUC;
		struct _IB_WC_RECV_UD {
			IB_SEND_OPTIONS	Options;	/* Possible flags are ImmediateData */
			IB_RECV_FLAGS	Flags;
			uint32		ImmediateData;
			uint32		SrcQPNumber;
#if INCLUDE_16B
			STL_LID		SrcLID;
#else
			IB_LID		SrcLID;
#endif
			IB_P_KEY	PkeyIndex;		/* GSI only */
			IB_SL		ServiceLevel;
			IB_PATHBITS	DestPathBits;
		} RecvUD;
		struct _IB_WC_RECV_RD {
			IB_SEND_OPTIONS	Options;	/* Possible flags are ImmediateData */
			uint32			ImmediateData;
			uint32			SrcQPNumber;
			uint32			DestEECNumber;	/* Dest EE Context number */
#if INCLUDE_16B
			STL_LID			SrcLID;
#else
			IB_LID			SrcLID;
#endif
			IB_SL			ServiceLevel;
			uint8			FreedResourceCount;
		} RecvRD;
	} Req;
} IB_WORK_COMPLETION;


/*
 * Memory Region definitions
 */

/*
 * RegisterContigPhysMemRegion buffer list
 */

typedef struct _IB_MR_PHYS_BUFFER {
	uint64	PhysAddr;
	uint64	Length;		/* in bytes */
} IB_MR_PHYS_BUFFER;


/*
 * Modify Memory Region request type
 */

typedef union _IB_MR_MODIFY_REQ {
	uint16	AsUINT16;
	struct _IB_MR_MODIFY_REQ_S {
		uint16	Translation:		1;
		uint16	ProtectionDomain:	1;
		uint16	AccessControl:		1;
		uint16	Reserved:			13;
	} s;
} IB_MR_MODIFY_REQ;

#define DEFAULT_MAX_AVS_PER_PD	256	/* default AV limit per PD if SDK3 API used */

#ifdef __cplusplus
};
#endif

#endif  /* _IBA_VPI_H_ */
