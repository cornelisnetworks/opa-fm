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

/* Suppress duplicate loading of this file */
#ifndef _INSI_IB_CM_MSG_H_
#define _INSI_IB_CM_MSG_H_


#if defined (__cplusplus)
extern "C" {
#endif

#include "datatypes.h"
#include "stl_types.h"
#include "stl_mad.h"
#include "ibyteswap.h"

#include "ib_generalServices.h"
#include "ib_cm.h"

/****************************************************************************
 * IB CM attribute ID per 1.0a and 1.1 spec
 */

#define	MCLASS_ATTRIB_ID_CLASS_PORT_INFO		0x0001

#define MCLASS_ATTRIB_ID_REQ			0x0010	/* Connection Request */
#define MCLASS_ATTRIB_ID_MRA			0x0011	/* Message Receipt Ack */
#define MCLASS_ATTRIB_ID_REJ			0x0012	/* Connection Reject */
#define MCLASS_ATTRIB_ID_REP			0x0013	/* Connection Reply */
#define MCLASS_ATTRIB_ID_RTU			0x0014	/* Ready To Use */
#define MCLASS_ATTRIB_ID_DREQ			0x0015	/* Disconnect Request */
#define MCLASS_ATTRIB_ID_DREP			0x0016	/* Disconnect Reply */
#define MCLASS_ATTRIB_ID_SIDR_REQ		0x0017	/* Service ID Resolution Req */
#define MCLASS_ATTRIB_ID_SIDR_RESP		0x0018	/* Service ID Resolution Resp */
#define MCLASS_ATTRIB_ID_LAP			0x0019	/* Load Alternate Path */
#define MCLASS_ATTRIB_ID_APR			0x001A	/* Alternate Path Response */


/****************************************************************************
 * CMM_XXX
 * IB-defined connection messages on the wire. Each message is a fixed 232 bytes
 * in length and occupy the data portion of the CM_MAD.
 */

/* Private/user data length in bytes */
#define CMM_REQ_USER_LEN			92
#define CMM_REP_USER_LEN			196
#define CMM_REJ_USER_LEN			148 
#define CMM_RTU_USER_LEN			224
#define CMM_MRA_USER_LEN			222
#define CMM_DREQ_USER_LEN			220
#define CMM_DREP_USER_LEN			224
#define CMM_SIDR_REQ_USER_LEN		216
#define CMM_SIDR_RESP_USER_LEN		136 
#define CMM_LAP_USER_LEN			168
#define CMM_APR_USER_LEN			148
#define CMM_REJ_ADD_INFO_LEN		72	/* Additional reject info length */
#define CMM_SIDR_RESP_ADD_INFO_LEN	72	/* Additional Error Infomation length */
#define CMM_APR_ADD_INFO_LEN		72	/* Additional Alt Path Resp info len */

typedef enum {
	CM_REJECT_REQUEST=0,
	CM_REJECT_REPLY,
	CM_REJECT_UNKNOWN
} CMM_REJ_MSGTYPE;

typedef enum {
	CM_MRA_REQUEST=0,
	CM_MRA_REPLY,
	CM_MRA_LAP
} CMM_MRA_MSGTYPE;

#include "ipackon.h"

/*
 * CMM_REQ - 
 * Connection request message (REQ)
 */
typedef struct _CMM_REQ {

	uint32			LocalCommID;
	uint32			Reserved1;
	uint64			ServiceID;
	uint64			LocalCaGUID;
	uint32			Reserved2;
	uint32			LocalQKey;
	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					LocalQPN:24,
					OfferedResponderResources:8)
		} PACK_SUFFIX s;
	} u1;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					LocalEECN:24,
					OfferedInitiatorDepth:8)
		} PACK_SUFFIX s;
	} u2;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD4(uint32,
					RemoteEECN:24,
					RemoteCMTimeout:5,
					TransportServiceType:2,
					EndToEndFlowControl:1)
		} PACK_SUFFIX s;
	} u3;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD3(uint32,
					StartingPSN:24,
					LocalCMTimeout:5,
					RetryCount:3)
		} PACK_SUFFIX s;
	} u4;

	uint16			PartitionKey;

	IB_BITFIELD3(uint8,
					PathMTU:4,					/* see IB_MTU */
					RdcExists:1,
					RnRRetryCount:3)
	IB_BITFIELD2(uint8,
					MaxCMRetries:4,
					Reserved3:4)

	/* Primary Path Information */
	IB_LID			PrimaryLocalLID;
	IB_LID			PrimaryRemoteLID;
	IB_GID			PrimaryLocalGID;
	IB_GID			PrimaryRemoteGID;

	union {
		uint32		AsUint32;

		struct {
			IB_BITFIELD3(uint32,
					PrimaryFlowLabel:20,
					Reserved4:6,
					PrimaryPacketRate:6)
		} PACK_SUFFIX s;
	} u5;

	uint8			PrimaryTrafficClass;
	uint8			PrimaryHopLimit;
	IB_BITFIELD3(uint8,
					PrimarySL:4,
					PrimarySubnetLocal:1,
					Reserved5:3)
	IB_BITFIELD2(uint8,
					PrimaryLocalAckTimeout:5,
					Reserved6:3)

	/* Alternate Path Information */
	IB_LID			AlternateLocalLID;
	IB_LID			AlternateRemoteLID;
	IB_GID			AlternateLocalGID;
	IB_GID			AlternateRemoteGID;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD3(uint32,
					AlternateFlowLabel:20,
					Reserved7:6,
					AlternatePacketRate:6)
		} PACK_SUFFIX s;
	} u6;

	uint8			AlternateTrafficClass;
	uint8			AlternateHopLimit;
	IB_BITFIELD3(uint8,
					AlternateSL:4,
					AlternateSubnetLocal:1,
					Reserved8:3)
	IB_BITFIELD2(uint8,
					AlternateLocalAckTimeout:5,
					Reserved9:3)

	uint8			PrivateData[CMM_REQ_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_REQ;

/*
 * CMM_REP - 
 * Connection reply message (REP)
 */
typedef struct _CMM_REP {

	uint32			LocalCommID;
	uint32			RemoteCommID;
	uint32			LocalQKey;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					LocalQPN:24,
					Reserved1:8)
		} PACK_SUFFIX s;
	} u1;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					LocalEECN:24,
					Reserved2:8)
		} PACK_SUFFIX s;
	} u2;


	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					StartingPSN:24,
					Reserved3:8)
		} PACK_SUFFIX s;
	} u3;


	uint8			ArbResponderResources;
	uint8			ArbInitiatorDepth;

	IB_BITFIELD3(uint8,
					TargetAckDelay:5,
					FailoverAccepted:2, /* a CMM_REP_FAILOVER enum */
					EndToEndFlowControl:1)

	IB_BITFIELD2(uint8,
					RnRRetryCount:3,
					Reserved4:5)

	uint64			LocalCaGUID;

	uint8			PrivateData[CMM_REP_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_REP;

/*
 * CMM_REJ -
 * Connection reject message (REJ)
 */
typedef struct _CMM_REJ {

	uint32			LocalCommID;
	uint32			RemoteCommID;
	
	IB_BITFIELD2(uint8,
					MsgRejected:2,
					Reserved1:6)

	IB_BITFIELD2(uint8,
					RejectInfoLen:7,
					Reserved2:1)

	uint16			Reason;
	uint8			RejectInfo[CMM_REJ_ADD_INFO_LEN];

	uint8			PrivateData[CMM_REJ_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_REJ;

/*
 * CMM_RTU -
 * Connection ready-to-use message (RTU)
 */
typedef struct _CMM_RTU {

	uint32			LocalCommID;
	uint32			RemoteCommID;

	uint8			PrivateData[CMM_RTU_USER_LEN];	/* User-specified data */
} PACK_SUFFIX CMM_RTU;


/*
 * CMM_MRA -
 * Connection message receipt acknowledge (MRA)
 */
typedef struct _CMM_MRA {

	uint32			LocalCommID;
	uint32			RemoteCommID;

	IB_BITFIELD2(uint8,
					MsgMraed:2,
					Reserved1:6)

	IB_BITFIELD2(uint8,
					ServiceTimeout:5,
					Reserved2:3)

	uint8			PrivateData[CMM_MRA_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_MRA;

/*
 * CMM_DREQ -
 * Disconnect request message (DREQ)
 */
typedef struct _CMM_DREQ {

	uint32			LocalCommID;
	uint32			RemoteCommID;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					RemoteQPNorEECN:24,
					Reserved:8)
		} PACK_SUFFIX s;
	} u;

	uint8			PrivateData[CMM_DREQ_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_DREQ;

/*
 * CMM_DREP - 
 * Disconnect reply message (DREP)
 */
typedef struct _CMM_DREP {

	uint32			LocalCommID;
	uint32			RemoteCommID;

	uint8			PrivateData[CMM_DREP_USER_LEN];	/* User-specified data */
} PACK_SUFFIX CMM_DREP;


/*
 * CMM_SIDR_REQ - 
 * Service ID resolution request message (SIDR_REQ)
 */
typedef struct _CMM_SIDR_REQ {

	uint32			RequestID;
	uint16			PartitionKey;
	uint16			Reserved;

	uint64			ServiceID;
	uint8			PrivateData[CMM_SIDR_REQ_USER_LEN];/* User-specified data */
} PACK_SUFFIX CMM_SIDR_REQ;

/*
 * CMM_SIDR_RESP -
 * Service ID resolution reply message (SIDR_RESP)
 */
typedef struct _CMM_SIDR_RESP {

	uint32			RequestID;
	/* need to pass AddInfo and Len to user in SIDR Resp API */
	uint8			Status;
	uint8			AddInfoLen;		/* Length of AddInfo in bytes */
	uint16			Reserved;
	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD2(uint32,
					QPN:24,
					Reserved2:8)
		} PACK_SUFFIX s;
	} u;

	uint64			ServiceID;
	uint32			QKey;						/* (24 bits) */

	uint8			AddInfo[CMM_SIDR_RESP_ADD_INFO_LEN];/* Additional Info
														 * about a failure
														 * Status
														 */

	uint8			PrivateData[CMM_SIDR_RESP_USER_LEN];/* User-specified data */
} PACK_SUFFIX CMM_SIDR_RESP;



/*
 * CMM_LAP - 
 * Load alternate path message (LAP)
 */
typedef struct _CMM_LAP {

	uint32			LocalCommID;
	uint32			RemoteCommID;
	uint32			Reserved1;;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD3(uint32,
					RemoteQPNorEECN:24,
					RemoteCMTimeout:5,
					Reserved2:3)
		} PACK_SUFFIX s;
	} u1;
	uint32			Reserved3;
	IB_LID			AlternateLocalLID;
	IB_LID			AlternateRemoteLID;
	IB_GID			AlternateLocalGID;
	IB_GID			AlternateRemoteGID;

	union {
		uint32		AsUint32;
		struct {
			IB_BITFIELD3(uint32,
					AlternateFlowLabel:20,
					Reserved4:4,
					AlternateTrafficClass:8)
		} PACK_SUFFIX s;
	} u2;

	uint8			AlternateHopLimit:8;
	IB_BITFIELD2(uint8,
					Reserved5:2,
					AlternatePacketRate:6)
	IB_BITFIELD3(uint8,
					AlternateSL:4,
					AlternateSubnetLocal:1,
					Reserved6:3)
	IB_BITFIELD2(uint8,
					AlternateLocalAckTimeout:5,
					Reserved7:3)

	uint8			PrivateData[CMM_LAP_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_LAP;

/*
 * CMM_APR -
 * Alternate path response message (APR)
 */
typedef struct _CMM_APR {

	uint32			LocalCommID;
	uint32			RemoteCommID;
	uint8			AddInfoLen;		/* Length of AddInfo in bytes */
	uint8			APStatus;
	uint16			Reserved;
	uint8			AddInfo[CMM_APR_ADD_INFO_LEN];	/* Additional Information
													 * about a failure APStatus
													 */

	uint8			PrivateData[CMM_APR_USER_LEN];	/* User-specified data */

} PACK_SUFFIX CMM_APR;

typedef union _CMM_MSG {
	IB_CLASS_PORT_INFO		ClassPortInfo;
	CMM_REQ				REQ;
	CMM_REP				REP;
	CMM_REJ				REJ;
	CMM_RTU				RTU;
	CMM_DREQ			DREQ;
	CMM_DREP			DREP;
	CMM_MRA				MRA;
	CMM_LAP				LAP;
	CMM_APR				APR;
	CMM_SIDR_REQ		SIDR_REQ;
	CMM_SIDR_RESP		SIDR_RESP;
} PACK_SUFFIX CMM_MSG;


/*
 * CM_MAD -
 * Connection Manager's Mgmt Datagram. This datagram is passed
 * to GSI
 */
typedef struct _CM_MAD {
	MAD_COMMON				common;	/* 24 bytes header */
	CMM_MSG					payload;/* 232 bytes payload */
} PACK_SUFFIX CM_MAD;


/* Wire format is BIG-ENDIAN */
#if CPU_BE
#define		ConvertMadToNetworkByteOrder(pMad)
#define		ConvertMadToHostByteOrder(pMad)
#else
#define		ConvertMadToNetworkByteOrder(pMad)	ConvertToNetworkOrder(pMad)
#define		ConvertMadToHostByteOrder(pMad)		ConvertToHostOrder(pMad)
#endif /* CPU_LE */

/*
 * based on value of FailoverAccepted in Rep, does remote endpoint
 * support failover.  This can be inconclusive if REQ did not provide an
 * alternate path, it which case its likely the REP will accept it,
 * but that doesn't mean the remote endpoint supports failover.
 */
static __inline
boolean IsFailoverSupported(uint8 FailoverAccepted)
{
	return (FailoverAccepted == CM_REP_FO_ACCEPTED
			|| FailoverAccepted == CM_REP_FO_REJECTED_ALT);
}

#if defined( DBG ) || defined( IB_DEBUG )
#define DUMP_CLASS_PORT_INFO(pClassPortInfo)	DumpClassPortInfo(pClassPortInfo)
#define DUMP_REQ(pREQ)							DumpREQ(pREQ)
#define DUMP_REP(pREP)							DumpREP(pREP)
#define DUMP_REJ(pREJ)							DumpREJ(pREJ)
#define DUMP_LAP(pREJ)							DumpLAP(pREJ)
#define DUMP_APR(pREJ)							DumpAPR(pREJ)
#define DUMP_DREQ(pDREQ)						DumpDREQ(pDREQ)
#define DUMP_SIDR_REQ(pSIDR_REQ)				DumpSIDR_REQ(pSIDR_REQ)
#define DUMP_SIDR_RESP(pSIDR_RESP)				DumpSIDR_RESP(pSIDR_RESP)

#define DUMP_REQ_INFO(pReqInfo)					DumpReqInfo(pReqInfo)	
#define DUMP_REP_INFO(pRepInfo)					DumpRepInfo(pRepInfo)
#define DUMP_REJ_INFO(pRejInfo)					DumpRejInfo(pRejInfo)			
#define DUMP_LAP_INFO(pLapInfo)					DumpLapInfo(pLapInfo)			
#define DUMP_APR_INFO(pAprInfo)					DumpAprInfo(pAprInfo)			
#define DUMP_DREQ_INFO(pDReqInfo)				DumpDReqInfo(pDReqInfo)
#define DUMP_SIDR_REQ_INFO(pSidrReqInfo)		DumpSidrReqInfo(pSidrReqInfo)
#define DUMP_SIDR_RESP_INFO(pSidrRespInfo)		DumpSidrRespInfo(pSidrRespInfo)

#else
#define DUMP_CLASS_PORT_INFO(pClassPortInfo)
#define DUMP_REQ(pREQ)
#define DUMP_REP(pREP)
#define DUMP_REJ(pREJ)
#define DUMP_LAP(pREJ)
#define DUMP_APR(pREJ)
#define DUMP_DREQ(pDREQ)
#define DUMP_SIDR_REQ(pSIDR_REQ)
#define DUMP_SIDR_RESP(pSIDR_RESP)

#define DUMP_REQ_INFO(pReqInfo)	
#define DUMP_REP_INFO(pRepInfo)
#define DUMP_REJ_INFO(pRejInfo)		
#define DUMP_LAP_INFO(pLapInfo)
#define DUMP_APR_INFO(pAprInfo)
#define DUMP_DREQ_INFO(pDReqInfo)
#define DUMP_SIDR_REQ_INFO(pSidrReqInfo)
#define DUMP_SIDR_RESP_INFO(pSidrRespInfo)
#endif

/****************************************************************************
 * Private function prototypes
 */

void
FormatClassPortInfo(
		  IN CM_MAD*			Mad, 
		  IN uint64				TransactionID
		  );

void
FormatREQ(
		  IN CM_MAD*			Mad, 
		  IN const CM_REQUEST_INFO*	pRequest,
		  IN uint64				TransactionID,
		  IN uint32				LocalCommID,
		  IN uint32				CMQKey,
		  IN CM_CEP_TYPE		Type,
		  IN uint32				LocalCMTimeout, 
		  IN uint32				RemoteCMTimeout, 
		  IN uint32				MaxCMRetries,
		  IN uint8				OfferedInitiatorDepth,
		  IN uint8				OfferedResponderResources
		  );
void
CopyREQToReqInfo(
				IN const CMM_REQ* pReq,
				IN CM_REQUEST_INFO *pInfo
				);

void
FormatREP(
		  IN CM_MAD* Mad, 
		  IN const CM_REPLY_INFO* pConnectReply,
		  IN uint64 TransactionID,
		  IN uint32 LocalCommID,
		  IN uint32 RemoteCommID,
		  IN uint64 LocalCaGUID
		  );

void
CopyREPToRepInfo( 
	const IN CMM_REP *pREP,
	IN OUT CM_REPLY_INFO *pReplyInfo
	);


void
FormatREJ(
		  IN CM_MAD*	Mad,
		  IN uint8		MsgRejected,	/* CMM_REJ_MSGTYPE */
		  IN uint16		Reason,
		  IN const uint8*		pRejectInfo,
		  IN uint32		RejectInfoLen,
		  IN const uint8*		pPrivateData,
		  IN uint32		PrivateDataLen,
		  IN uint64 	TransactionID,
		  IN uint32		LocalCommID,
		  IN uint32		RemoteCommID
		  );

void
CopyREJToRejectInfo(
	IN const CMM_REJ* pREJ,
	IN OUT CM_REJECT_INFO *pRejectInfo
	);

void
FormatMRA(
		  IN CM_MAD* Mad, 
		  IN uint8	MsgMraed,			/* CMM_MRA_MSGTYPE */
		  IN uint8	ServiceTimeout,
		  IN const uint8*	pPrivateData,
		  IN uint32 PrivateDataLen,
		  IN uint64 TransactionID,
		  IN uint32 LocalCommID,
		  IN uint32 RemoteCommID
		  );

void
FormatRTU(
		  IN CM_MAD* Mad, 
		  const uint8* pPrivateData,
		  uint32 PrivateDataLen,
		  IN uint64 TransactionID,
		  IN uint32 LocalCommID,
		  IN uint32 RemoteCommID
		  );

void
FormatLAP(
		  IN CM_MAD* Mad, 
		  IN uint32 QPN,
		  IN uint32 RemoteCMTimeout,
		  IN const CM_ALTPATH_INFO*	pAltPath,
		  IN uint8  AlternateAckTimeout,
		  IN uint64 TransactionID,
		  IN uint32 LocalCommID,
		  IN uint32 RemoteCommID
		  );

void
CopyLAPToAltPathRequest(
	IN const CMM_LAP* pLAP,
	IN OUT CM_ALTPATH_INFO *pAltPath
	);

int
CompareLAP(
	IN const CMM_LAP* pLAP1,
	IN const CMM_LAP* pLAP2
	);

void
FormatAPR(
		  IN CM_MAD*	Mad,
		  IN CM_APR_STATUS	APStatus,
		  IN const uint8*		pAddInfo,
		  IN uint32		AddInfoLen,
		  IN const uint8*		pPrivateData,
		  IN uint32		PrivateDataLen,
		  IN uint64 	TransactionID,
		  IN uint32		LocalCommID,
		  IN uint32		RemoteCommID
		  );

void
CopyAPRToAltPathReply(
	IN const CMM_APR* pAPR,
	IN OUT CM_ALTPATH_REPLY_INFO *pAltPathReply
	);


void
FormatDREQ(
		   IN CM_MAD* Mad,
		   IN uint32 QPN,
		   IN const uint8* pPrivateData,
		   IN uint32 PrivateDataLen,
		   IN uint64 TransactionID,
		   IN uint32 LocalCommID,
		   IN uint32 RemoteCommID
		   );

void
FormatDREP(
		   IN CM_MAD* Mad,
		   IN const uint8*  pPrivateData,
		   IN uint32 PrivateDataLen,
		   IN uint64 TransactionID,
		   IN uint32 LocalCommID,
		   IN uint32 RemoteCommID
		   );

void
FormatSIDR_REQ(
				IN CM_MAD* Mad,
				IN uint64 SID,
				IN uint16 PartitionKey,
				IN const uint8* pPrivateData,
				IN uint32 PrivateDataLen,
		   		IN uint64 TransactionID,
				IN uint32 RequestID
			  );

void
CopySIDR_REQToSidrReqInfo(
				IN const CMM_SIDR_REQ* pSIDR_REQ,
				IN OUT SIDR_REQ_INFO *pSidrReqInfo
						);

void
FormatSIDR_RESP(
				IN CM_MAD* Mad,
				IN uint64 SID,
				IN uint32 QPN,
				IN uint32 QKey,
				IN uint32 Status,
				IN const uint8* pPrivateData,
				IN uint32 PrivateDataLen,
		   		IN uint64 TransactionID,
				IN uint32 RequestID
				);

void
CopySIDR_RESPToSidrRespInfo(
				IN const CMM_SIDR_RESP* pSIDR_RESP,
				IN OUT SIDR_RESP_INFO *pSidrRespInfo
				);


void
ConvertREQToNetworkOrder(
				IN OUT CMM_REQ* pRequest
				);

void
ConvertREPToNetworkOrder(
				IN OUT CMM_REP* pReply
				);
void
ConvertLAPToNetworkOrder(
				IN OUT CMM_LAP* pLap
				);

void
ConvertToNetworkOrder(
				IN OUT CM_MAD* pMad
				);

void
ConvertToHostOrder(
				IN OUT CM_MAD*	pMad
				);

#if defined( DBG ) || defined( IB_DEBUG )

void DumpClassPortInfo(const IB_CLASS_PORT_INFO* pClassPortInfo);
void DumpREQ(const CMM_REQ* pREQ);
void DumpReqInfo(const CM_REQUEST_INFO* pReqInfo);

void DumpREP(const CMM_REP* pREP);
void DumpRepInfo(const CM_REPLY_INFO* pRepInfo);

void DumpREJ(const CMM_REJ* pREJ);
void DumpRejInfo(const CM_REJECT_INFO* pRepInfo);

void DumpLAP(const CMM_LAP* pLAP);
void DumpLapInfo(const CM_ALTPATH_INFO* pLapInfo);

void DumpAPR(const CMM_APR* pAPR);
void DumpAprInfo(const CM_ALTPATH_REPLY_INFO* pAprInfo);

void DumpDREQ(const CMM_DREQ* pDREQ);
void DumpDReqInfo(const CM_DREQUEST_INFO* pDReqInfo);

void DumpSIDR_REQ(const CMM_SIDR_REQ* pSIDR_REQ);
void DumpSidrReqInfo(const SIDR_REQ_INFO* pSidrReqInfo);

void DumpSIDR_RESP(const CMM_SIDR_RESP* pSIDR_RESP);
void DumpSidrRespInfo(const SIDR_RESP_INFO* pSidrRespInfo);

#endif

#if defined (__cplusplus)
};
#endif

#include "ipackoff.h"

#endif  /* _INSI_IB_CM_MSG_H_ */
