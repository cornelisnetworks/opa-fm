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

#ifndef __IBA_IB_GENERAL_SERVICES_H__
#define __IBA_IB_GENERAL_SERVICES_H__ (1) /* suppress duplicate loading of this file */

#include "iba/stl_mad.h"
#include "iba/public/ibyteswap.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * This header defines Class specific MAD packet formats for
 * the IB General Services which are defined in IBTA 1.0a and 1.1
 * in Chapter 16 "General Services"
 * For each class, there is an additional header file which documents
 * specific attribute formats and Status codes specific to the class
 * For example:
 * 	ib_pm.h - performance management
 *	ib_dm.h - device management
 *	ib_sa_records.h - subnet administration
 *	ib_cc.h - congestion control
 */

/* these are valid in 1.0a and 1.1
 * however for 1.1, classes using RMPP have a smaller HDRSIZE and more DATA
 */
#define		IBA_GS_HDRSIZE			64 /* common + class specific header */
#define		IBA_GS_DATASIZE			(IB_MAD_BLOCK_SIZE - IBA_GS_HDRSIZE) /* what's left for class payload */

#define		STL_GS_HDRSIZE			24 /* common header */
#define		STL_IBA_GS_DATASIZE		(STL_MAD_BLOCK_SIZE - IBA_GS_HDRSIZE)
#define		STL_GS_DATASIZE			(STL_MAD_BLOCK_SIZE - STL_GS_HDRSIZE)
										/* can include SAR header */
/* IBA_RMPP_GS* is computed below, these are here as a reminder for users */
/*#define	IBA_RMPP_GS_DATASIZE	220*//* what's left for class payload */
/*#define	IBA_RMPP_GS_HDRSIZE		36 *//* common + RMPP header */
#define		IBA_SUBN_ADM_HDRSIZE	56 /* common + class specific header */
#define		IBA_SUBN_ADM_DATASIZE	(IB_MAD_BLOCK_SIZE - IBA_SUBN_ADM_HDRSIZE) /* what's left for class payload */

#define		IB_SUBN_ADM_DATASIZE (IB_MAD_BLOCK_SIZE - (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER) + sizeof(SA_HDR)))
#define		STL_SUBN_ADM_DATASIZE (STL_MAD_BLOCK_SIZE - (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER) + sizeof(SA_HDR)))

#define		STL_IBA_SUBN_ADM_DATASIZE	STL_SUBN_ADM_DATASIZE


										/* includes RMPP */
/* -------------------------------------------------------------------------- */
/* versions for class managers */
#define		IB_SUBN_ADM_CLASS_VERSION		2		/* Subnet Administration */
#define		IB_PERF_CLASS_VERSION			1		/* Performance Management */
#define		IB_BM_CLASS_VERSION				1		/* Baseboard Management */
#define		IB_DEV_MGT_CLASS_VERSION		1		/* Device Management */
													/* bad news, protocol */
													/* had changes in IBTA1.1 */
													/* but class revision did */
													/* not change */
#define		IB_CC_CLASS_VERSION				2		/* Congestion Control */
#define		IB_SNMP_CLASS_VERSION			1		/* SNMP Tunneling */
#define		IB_VENDOR_SPEC_CLASS_VERSION	1		/* Vendor Specific Management */
#define		IB_APP_SPEC_CLASS_VERSION		1		/* Application Specific */
#define		IB_COMM_MGT_CLASS_VERSION		2		/* Communication Management */
#define		IB_PA_CLASS_VERSION			    2		/* Performance Analysis */

/* -------------------------------------------------------------------------- */
/* wire protocol packets and sub-structures */
#include "iba/public/ipackon.h"

/*---------------------------------------------------------------- */
/* RMPP (Reliable Multi-Packet Protocol) data structures */
#define RMPP_VERSION 1

#ifndef RMPP_TYPE_DATA	// workaround conflict with FM's ib_sa.h header
typedef enum {
	RMPP_TYPE_NONE = 0,			/* not an RMPP packet */
	RMPP_TYPE_DATA = 1,			/* data from sender to receiver */
	RMPP_TYPE_ACK = 2,			/* ack from receiver to sender */
	RMPP_TYPE_STOP = 3,
	RMPP_TYPE_ABORT = 4
	/* 5-255 reserved */
} RMPP_TYPE;
#endif

#define RMPP_RRESPTIME_NONE	0x1f	/* no time value provided */

typedef enum {
	RMPP_STATUS_NORMAL = 0,			/* no errors */
	RMPP_STATUS_RESOURCES_EXHAUSTED = 1,	/* sender is out of resources */
	RMPP_STATUS_START_RESERVED = 2,			/* 2-117 reserved */
	RMPP_STATUS_END_RESERVED = 117,
	RMPP_STATUS_TOTAL_TIME_TOO_LONG = 118,	/* total transaction timeout */
	RMPP_STATUS_INCONSISTENT_PAYLOAD_LEN = 119,
	RMPP_STATUS_INCONSISTENT_FIRST = 120,
	RMPP_STATUS_BAD_RMPP_TYPE = 121,
	RMPP_STATUS_NEW_WL_TOO_SMALL = 122,
	RMPP_STATUS_SEGMENT_NUMBER_TOO_BIG = 123,
	RMPP_STATUS_ILLEGAL_STATUS = 124,
	RMPP_STATUS_UNSUPPORTED_VERSION = 125,
	RMPP_STATUS_TOO_MANY_RETRIES = 126,
	RMPP_STATUS_UNSPECIFIED = 127
	/* 128-191 class specific */
	/* 192-255 vendor specific */
} RMPP_STATUS;

typedef union _RMPP_FLAG {
	uint8		AsReg8;
	struct {
#if CPU_BE
		uint8	RRespTime:5;		/* similar to ClassPortInfo:RespTimeValue */
		uint8	LastPkt:1;			/* for RMPP_TYPE_DATA, is this last */
		uint8	FirstPkt:1;			/* for RMPP_TYPE_DATA, is this first */
		uint8	Active:1;			/* this is part of an RMPP transmission */
#else
		uint8	Active:1;			/* this is part of an RMPP transmission */
		uint8	FirstPkt:1;			/* for RMPP_TYPE_DATA, is this first */
		uint8	LastPkt:1;			/* for RMPP_TYPE_DATA, is this last */
		uint8	RRespTime:5;		/* similar to ClassPortInfo:RespTimeValue */
#endif
	} s;
} PACK_SUFFIX RMPP_FLAG;

/* Note - the SA data structures defined here are not well aligned
 * Also RMPP header is 12 bytes, affecting alignment of all that follows
 * may need to re-visit in RISC environment
 */
typedef struct _RMPP_HEADER {
	uint8		RmppVersion;		/* version of RMPP implemented */
									/* must be 0 if RMPP_FLAG.Active=0 */
	uint8		RmppType;			/* type of RMPP packet */
	RMPP_FLAG	RmppFlags;
	uint8		RmppStatus;
	union {
		uint32		AsReg32;
		uint32		SegmentNum;	/* DATA and ACK */
		uint32		Reserved1;	/* ABORT and STOP */
	} u1;
	union {
		uint32		AsReg32;
		uint32		PayloadLen;		/* first and last DATA */
		uint32		NewWindowLast;	/* ACK */
		uint32		Reserved2;		/* ABORT, STOP and middle DATA */
	} u2;
} PACK_SUFFIX RMPP_HEADER, *PRMPP_HEADER;

/* Common RMPP MAD packet */
#define	RMPP_GS_HDRSIZE      (sizeof(MAD_COMMON)+sizeof(RMPP_HEADER))
#define	IBA_RMPP_GS_DATASIZE (IB_MAD_BLOCK_SIZE - RMPP_GS_HDRSIZE)
#define	STL_RMPP_GS_DATASIZE (STL_MAD_BLOCK_SIZE - RMPP_GS_HDRSIZE)

typedef struct	_MAD_RMPP {
	MAD_COMMON			common;
	RMPP_HEADER			RmppHdr;
	uint8				Data[STL_RMPP_GS_DATASIZE];
} PACK_SUFFIX	MAD_RMPP;

/*---------------------------------------------------------------- */
/* Subnet Administration MAD format - Class Version 2 */
typedef struct _SA_HDR {
	uint64		SmKey;			/* SM_KEY */
	uint16		AttributeOffset;/* attribute record size in units of uint64's */
	uint16		Reserved;
	uint64		ComponentMask;	/* Component mask for query operation */
} PACK_SUFFIX SA_HDR;

typedef struct _SA_MAD_HDR {
	RMPP_HEADER	RmppHdr;		/* RMPP header */
	SA_HDR		SaHdr;			/* SA class specific header */
} PACK_SUFFIX SA_MAD_HDR;

typedef struct _IB_SA_MAD {
	RMPP_HEADER	RmppHdr;		/* RMPP header */
	SA_HDR		SaHdr;			/* SA class specific header */
	uint8		Data[STL_SUBN_ADM_DATASIZE];
} PACK_SUFFIX IB_SA_MAD;

typedef struct _SA_MAD {
	MAD_COMMON	common;	/* Generic MAD Header */
	RMPP_HEADER	RmppHdr;		/* RMPP header */
	SA_HDR		SaHdr;			/* SA class specific header */
	uint8		Data[STL_SUBN_ADM_DATASIZE];
} PACK_SUFFIX SA_MAD, *PSA_MAD;

/* -------------------------------------------------------------------------- */
/* Performance Management MAD format */
typedef struct _PERF_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */

	uint8		Resv[40];					/* class specific header */
	
	uint8		PerfData[IBA_GS_DATASIZE];	/* Performance Management Data */
} PACK_SUFFIX PERF_MAD, *PPERF_MAD;

typedef struct _STL_PERF_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */

	uint8		PerfData[STL_GS_DATASIZE];	/* Performance Management Data */
} PACK_SUFFIX STL_PERF_MAD, *PSTL_PERF_MAD;


/* -------------------------------------------------------------------------- */
/* BaseBoard Management MAD format */
typedef struct _BM_HDR {
	uint64		BKey;						/* B_Key */
	uint8		Resv[32];
} PACK_SUFFIX BM_HDR;

typedef struct _BM_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */
	
	BM_HDR		BmHdr;						/* class specific header */
	
	uint8		BMData[STL_IBA_GS_DATASIZE];	/* Baseboard Management Data */
} PACK_SUFFIX BM_MAD, *PBM_MAD;

/* -------------------------------------------------------------------------- */
/* Device Management MAD format */
typedef struct _DM_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */
	
	uint8		Resv[40];					/* class specific header */
	
	uint8		DMData[IBA_GS_DATASIZE];	/* Device Management Data */
} PACK_SUFFIX DM_MAD, *PDM_MAD;

/* -------------------------------------------------------------------------- */
/* CC MAD format (except CongestionLog attribute) */
typedef struct _CC_HDR {
	uint64		CC_Key;						/* CC__Key */
	uint8		Resv[32];
} PACK_SUFFIX CC_HDR;

typedef struct _CC_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */

	CC_HDR		CCHdr;						/* class specific header */
	
	uint8		CCData[STL_IBA_GS_DATASIZE];	/* Congestion Control Data */
} PACK_SUFFIX CC_MAD, *PCC_MAD;

/* CC CongestionLog MAD format (only for CongestionLog attribute) */
typedef struct _CC_LOG_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */
	uint64		CC_Key;						/* CC__Key */
	
	uint8		CCLogData[STL_IBA_GS_DATASIZE];	/* CongestionLog Data */
} PACK_SUFFIX CC_LOG_MAD, *PCC_LOG_MAD;

/* -------------------------------------------------------------------------- */
/* SNMP Tunneling MAD format
 * #### the multipacket MAD SAR protocol for SNMP tunneling data is different.
 */
typedef struct _SNMPTUNNEL_HDR {
	uint8		Resv[32];
	uint32		RAddr;						/* Raddress */
	uint8		PayloadLen;					/* Payload length */
	uint8		SegmentNum;					/* Segment number */
	uint16		SLID;						/* Source LID */
} PACK_SUFFIX SNMPTUNNEL_HDR;

typedef struct _SNMPTUNNEL_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */
	
	SNMPTUNNEL_HDR SnmpTunnelHdr;			/* class specific header */
	
	uint8		SNMPData[STL_IBA_GS_DATASIZE];	/* SNMP Tunneling Management Data */
} PACK_SUFFIX SNMPTUNNEL_MAD, *PSNMPTUNNEL_MAD;

/* -------------------------------------------------------------------------- */
#if 0 // redefine in opaswcommon.h
/* Vendor Specific Management MAD format */
typedef struct _VENDOR_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */
	
	uint8		VendorData[STL_IBA_GS_DATASIZE];	/* Device Management Data */
} PACK_SUFFIX VENDOR_MAD, *PVENDOR_MAD;
#endif

/* values for class specific fields in MAD STATUS
 * are defined in class manager specific header files
 */
#include "iba/public/ipackoff.h"

/* convert RMPP Status to a string
 * The string will only contain the message corresponding to the defined values
 * callers should also display the hex value in the event a
 * reserved, class or vendor specific value is returned by the manager/agent
 */
IBA_API const char* iba_rmpp_status_msg(RMPP_STATUS rmppStatus);

/* -------------------------------------------------------------------------- */
/* Swap between cpu and network byte order */

/* IBTA 1.1 */
static __inline
void
BSWAP_RMPP_HEADER(
	RMPP_HEADER			*RmppHdr
	)
{
#if CPU_LE
	RmppHdr->u1.AsReg32 = ntoh32( RmppHdr->u1.AsReg32 );
	RmppHdr->u2.AsReg32 = ntoh32( RmppHdr->u2.AsReg32 );
#endif
}

static __inline
void
BSWAP_SA_HDR(
	SA_HDR			*SaHdr
	)
{
#if CPU_LE
	SaHdr->SmKey = ntoh64( SaHdr->SmKey );
	SaHdr->AttributeOffset = ntoh16( SaHdr->AttributeOffset );
	SaHdr->ComponentMask = ntoh64( SaHdr->ComponentMask );
#endif
}

static __inline
void
BSWAPCOPY_SA_HDR(SA_MAD_HDR *Src, SA_MAD_HDR *Dest) 
{
    memcpy(Dest, Src, sizeof(SA_MAD_HDR));
    BSWAP_SA_HDR(&Dest->SaHdr);
    BSWAP_RMPP_HEADER(&Dest->RmppHdr);
}

static __inline
void
BSWAPCOPY_IB_SA_MAD(IB_SA_MAD *src, IB_SA_MAD *dst, size_t len) 
{
    memcpy(dst, src, sizeof(SA_MAD_HDR));
    BSWAP_SA_HDR(&dst->SaHdr);
    BSWAP_RMPP_HEADER(&dst->RmppHdr);
	memcpy(&dst->Data, &src->Data, len);
}

static __inline
void
BSWAP_BM_HDR(
	BM_HDR			*BmHdr
	)
{
#if CPU_LE
	BmHdr->BKey				= ntoh64( BmHdr->BKey );			
#endif
}

static __inline
void
BSWAP_SNMPTUNNEL_HDR(
	SNMPTUNNEL_HDR			*SnmpTunnelHdr
	)
{
#if CPU_LE
	SnmpTunnelHdr->RAddr	= ntoh32( SnmpTunnelHdr->RAddr );
	SnmpTunnelHdr->SLID		= ntoh16( SnmpTunnelHdr->SLID );
#endif
}

static __inline
void
BSWAP_CC_HDR(
	CC_HDR			*CCHdr
	)
{
#if CPU_LE
	CCHdr->CC_Key				= ntoh64( CCHdr->CC_Key );			
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* __IBA_IB_GENERAL_SERVICES_H__ */
