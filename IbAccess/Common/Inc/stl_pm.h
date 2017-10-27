/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef __IBA_STL_PM_H__
#define __IBA_STL_PM_H__

#include "iba/ib_generalServices.h"
#include "iba/stl_types.h"
#include "iba/stl_pa_types.h"

#if defined (__cplusplus)
extern "C" {
#endif

#include "iba/public/ipackon.h"

/*
 * Defines 
 */

#define STL_PM_CLASS_VERSION					0x80 	/* Performance Management version */

#define STL_PM_ATTRIB_ID_CLASS_PORTINFO			0x01
#define STL_PM_ATTRIB_ID_PORT_STATUS			0x40
#define STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS		0x41
#define STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS		0x42
#define STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS	0x43
#define STL_PM_ATTRIB_ID_ERROR_INFO				0x44

enum PM_VLs {
	PM_VL0 = 0,
	PM_VL1 = 1,
	PM_VL2 = 2,
	PM_VL3 = 3,
	PM_VL4 = 4,
	PM_VL5 = 5,
	PM_VL6 = 6,
	PM_VL7 = 7,
	PM_VL15 = 8,
	MAX_PM_VLS = 9
};
static __inline uint32 vl_to_idx(uint32 vl) {
	DEBUG_ASSERT(vl < PM_VL15 || vl == 15);
	if (vl >= PM_VL15) return (uint32)PM_VL15;
	return vl;
}
static __inline uint32 idx_to_vl(uint32 idx) {
	DEBUG_ASSERT(idx < MAX_PM_VLS);
	if (idx >= PM_VL15) return 15;
	return idx;
}

/* if AllPortSelect capability, use this as PortNum to operate on all ports */
#define PM_ALL_PORT_SELECT 0xff

#define	MAX_PM_PORTS							49		/* max in STL Gen1 (ports 0-48 on PRR switch) */


#define MAX_BIG_ERROR_COUNTERS					9
#define MAX_BIG_ERROR_COUNTERS_MIXED_MODE		5
#define MAX_BIG_VL_ERROR_COUNTERS				1

/* the default PM-PMA P_Key */
#define DEFAULT_PM_P_KEY						0x7fff

/* PortXmitData and PortRcvData are in units of flits (8 bytes) */
#define FLITS_PER_MiB ((uint64)1024*(uint64)1024/(uint64)8)
#define FLITS_PER_MB ((uint64)1000*(uint64)1000/(uint64)8)

/* MAD status codes for PM */
#define STL_PM_STATUS_REQUEST_TOO_LARGE			0x0100	/* response can't fit in a single MAD */
#define STL_PM_STATUS_NUMBLOCKS_INCONSISTENT	0x0200	/* attribute modifier number of blocks inconsistent with number of ports/blocks in MAD */
#define STL_PM_STATUS_OPERATION_FAILED			0x0300	/* an operation (such as clear counters) requested of the agent failed */

static __inline const char*
StlPmMadMethodToText(uint8 method)
{
	switch (method) {
	case MMTHD_GET: return "Get";
	case MMTHD_SET: return "Set";
	case MMTHD_GET_RESP: return "GetResp";
	default: return "Unknown";
	}
}

static __inline const char*
StlPmMadAttributeToText(uint16 aid)
{
	switch (aid) {
	case STL_PM_ATTRIB_ID_CLASS_PORTINFO: return "ClassPortInfo";
	case STL_PM_ATTRIB_ID_PORT_STATUS: return "PortStatus";
	case STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS: return "ClearPortStatus";
	case STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS: return "DataPortCounters";
	case STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS: return "ErrorportCounters";
	case STL_PM_ATTRIB_ID_ERROR_INFO: return "ErrorInfo";
	default: return "Unknown";
	}
}
static __inline void
StlPmClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		snprintf(buf, 80, "%s%s%s",
			(cmask & STL_CLASS_PORT_CAPMASK_TRAP) ? "Trap " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_NOTICE) ? "Notice " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_CM2) ? "CapMask2 " : "");
	}
}
static __inline void
StlPmClassPortInfoCapMask2(char buf[80], uint32 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		buf[0] = '\0';
	}
}

/* MAD structure definitions */

typedef struct _STL_PERF_MAD {
	MAD_COMMON	common;				/* Generic MAD Header */

	uint8		PerfData[STL_GS_DATASIZE];	/* Performance Management Data */
} PACK_SUFFIX STL_PERF_MAD, *PSTL_PERF_MAD;


/* STL Port Counters - small request, large response */

typedef struct _STL_Port_Status_Req {
	uint8	PortNumber;
	uint8	Reserved[3];;
	uint32	VLSelectMask;
} PACK_SUFFIX STLPortStatusReq, STL_PORT_STATUS_REQ;

typedef struct _STL_Port_Status_Rsp {
	uint8	PortNumber;
	uint8	Reserved[3];
	uint32	VLSelectMask;
	uint64	PortXmitData;
	uint64	PortRcvData;
	uint64	PortXmitPkts;
	uint64	PortRcvPkts;
	uint64	PortMulticastXmitPkts;
	uint64	PortMulticastRcvPkts;
	uint64	PortXmitWait;
	uint64	SwPortCongestion;
	uint64	PortRcvFECN;
	uint64	PortRcvBECN;
	uint64	PortXmitTimeCong;
	uint64	PortXmitWastedBW;
	uint64	PortXmitWaitData;
		
	uint64	PortRcvBubble;
	uint64	PortMarkFECN;
	uint64	PortRcvConstraintErrors;
	uint64	PortRcvSwitchRelayErrors;
	uint64	PortXmitDiscards;
	uint64	PortXmitConstraintErrors;
	uint64	PortRcvRemotePhysicalErrors;
	uint64	LocalLinkIntegrityErrors;
	uint64	PortRcvErrors;
	uint64	ExcessiveBufferOverruns;
	uint64	FMConfigErrors;
	uint32	LinkErrorRecovery;
	uint32	LinkDowned;
	uint8	UncorrectableErrors;
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD2(uint8,
				Reserved : 5,
				LinkQualityIndicator : 3)
		} PACK_SUFFIX s;
	} lq;
	uint8	Reserved2[6];
	struct _vls_pctrs {
		uint64	PortVLXmitData;
		uint64	PortVLRcvData;
		uint64	PortVLXmitPkts;
		uint64	PortVLRcvPkts;
		uint64	PortVLXmitWait;
		uint64	SwPortVLCongestion;
		uint64	PortVLRcvFECN;
		uint64	PortVLRcvBECN;
		uint64	PortVLXmitTimeCong;
		uint64	PortVLXmitWastedBW;
		uint64	PortVLXmitWaitData;
		uint64	PortVLRcvBubble;
		uint64	PortVLMarkFECN;
		uint64	PortVLXmitDiscards;
	} VLs[1]; /* n defined by number of bits in VLSelectmask */
} PACK_SUFFIX STLPortStatusRsp, STL_PORT_STATUS_RSP;

typedef struct _STL_Clear_Port_Status {
	uint64	PortSelectMask[4];
	CounterSelectMask_t CounterSelectMask;
} PACK_SUFFIX STLClearPortStatus, STL_CLEAR_PORT_STATUS;

/* The adders are added to the resolutions before performing the shift operation */
#define RES_ADDER_LLI 8
#define RES_ADDER_LER 2

/* STL Data Port Counters - small request, bigger response */
typedef struct _STL_Data_Port_Counters_Req {
	uint64	PortSelectMask[4];				/* signifies for which ports the PMA is to respond */
	uint32	VLSelectMask;					/* signifies for which VLs the PMA is to respond */
	union {
		uint32 AsReg32;
		struct { IB_BITFIELD3(uint32,
			Reserved : 24,
			LocalLinkIntegrityResolution : 4,	/* The PMA shall interpret the resolution fields as indicators of how many bits to right-shift */
			LinkErrorRecoveryResolution : 4)	/* the corresponding counters before adding them to the PortErrorCounterSummary */
		} PACK_SUFFIX s;
	} res;
} PACK_SUFFIX STLDataPortCountersReq, STL_DATA_PORT_COUNTERS_REQ;

struct _vls_dpctrs {
	uint64	PortVLXmitData;
	uint64	PortVLRcvData;
	uint64	PortVLXmitPkts;
	uint64	PortVLRcvPkts;
	uint64	PortVLXmitWait;
	uint64	SwPortVLCongestion;
	uint64	PortVLRcvFECN;
	uint64	PortVLRcvBECN;
	uint64	PortVLXmitTimeCong;
	uint64	PortVLXmitWastedBW;
	uint64	PortVLXmitWaitData;
	uint64	PortVLRcvBubble;
	uint64	PortVLMarkFECN;
};
 
struct _port_dpctrs {
	uint8	PortNumber;
	uint8	Reserved[3];
	union {
		uint32 AsReg32;
		struct { IB_BITFIELD2(uint32,
			Reserved : 29,
			LinkQualityIndicator : 3)
		} PACK_SUFFIX s;
	} lq;
	uint64	PortXmitData;
	uint64	PortRcvData;
	uint64	PortXmitPkts;
	uint64	PortRcvPkts;
	uint64	PortMulticastXmitPkts;
	uint64	PortMulticastRcvPkts;
	uint64	PortXmitWait;
	uint64	SwPortCongestion;
	uint64	PortRcvFECN;
	uint64	PortRcvBECN;
	uint64	PortXmitTimeCong;
	uint64	PortXmitWastedBW;
	uint64	PortXmitWaitData;
	uint64	PortRcvBubble;
	uint64	PortMarkFECN;
	uint64	PortErrorCounterSummary;		/* sum of all error counters for port */
	struct _vls_dpctrs VLs[1];							/* actual array size defined by number of bits in VLSelectmask */
};

typedef struct _STL_Data_Port_Counters_Rsp {
	uint64	PortSelectMask[4];
	uint32	VLSelectMask;
	union {
		uint32 AsReg32;
		struct { IB_BITFIELD3(uint32,
			Reserved : 24,
			LocalLinkIntegrityResolution : 4,
			LinkErrorRecoveryResolution : 4)
		} PACK_SUFFIX s;
	} res;
	struct _port_dpctrs Port[1];								/* actual array size defined by number of ports in attribute modifier */
} PACK_SUFFIX STLDataPortCountersRsp, STL_DATA_PORT_COUNTERS_RSP;

/* STL Error Port Counters - small request, bigger response */

typedef struct _STL_Error_Port_Counters_Req {
	uint64	PortSelectMask[4];				/* signifies for which ports the PMA is to respond */
	uint32	VLSelectMask;					/* signifies for which VLs the PMA is to respond */
	uint32	Reserved;
} PACK_SUFFIX STLErrorPortCountersReq, STL_ERROR_PORT_COUNTERS_REQ;

struct _vls_epctrs {
	uint64	PortVLXmitDiscards;
};
struct _port_epctrs {
	uint8	PortNumber;
	uint8	Reserved[7];
	uint64	PortRcvConstraintErrors;
	uint64	PortRcvSwitchRelayErrors;
	uint64	PortXmitDiscards;
	uint64	PortXmitConstraintErrors;
	uint64	PortRcvRemotePhysicalErrors;
	uint64	LocalLinkIntegrityErrors;
	uint64	PortRcvErrors;
	uint64	ExcessiveBufferOverruns;
	uint64	FMConfigErrors;
	uint32	LinkErrorRecovery;
	uint32	LinkDowned;
	uint8	UncorrectableErrors;
	uint8	Reserved2[7];
	struct _vls_epctrs VLs[1];
};
typedef struct _STL_Error_Port_Counters_Rsp {
	uint64	PortSelectMask[4];				/* echo from request */
	uint32	VLSelectMask;					/* echo from request */
	uint32	Reserved;
	struct _port_epctrs Port[1];				/* actual array size defined by number of ports in attribute modifier */
} PACK_SUFFIX STLErrorPortCountersRsp, STL_ERROR_PORT_COUNTERS_RSP;


struct _port_error_info {
	uint8	PortNumber;
	uint8	Reserved[7];

	/* PortRcvErrorInfo */
	struct {
		struct { IB_BITFIELD3(uint8,
			Status : 1,
			Reserved : 3,
			ErrorCode : 4)
		} PACK_SUFFIX s;
		union {
			uint8	AsReg8[17];			/* 136 bits of error info */
			struct {
				uint8	PacketFlit1[8];	/* first 64 bits of flit 1 */
				uint8	PacketFlit2[8];	/* first 64 bits of flit 2 */
				struct { IB_BITFIELD3(uint8,
					Flit1Bits : 1,		/* bit 64 of flit 1 */
					Flit2Bits : 1,		/* bit 64 of flit 2 */
					Reserved : 6)
				} PACK_SUFFIX s;
			} PACK_SUFFIX EI1to12;		/* error info for codes 1-12 */
			struct {
				uint8	PacketBytes[8];	/* first 64 bits of VL Marker flit */
				struct { IB_BITFIELD2(uint8,
					FlitBits : 1,		/* bit 64 of VL Marker flit */
					Reserved : 7)
				} PACK_SUFFIX s;
			} PACK_SUFFIX EI13;			/* error info for code 13 */
		} ErrorInfo;
		uint8 Reserved[6];
	} PACK_SUFFIX PortRcvErrorInfo;

	/* ExcessiveBufferOverrunInfo */
	struct {
		struct { IB_BITFIELD3(uint8,
			Status : 1,
			SC : 5,
			Reserved : 2)
		} PACK_SUFFIX s;
		uint8 Reserved[7];
	} PACK_SUFFIX ExcessiveBufferOverrunInfo;

	/* PortXmitConstraintErrorInfo */
	struct {
		struct { IB_BITFIELD2(uint8,
			Status : 1,
			Reserved : 7)
		} PACK_SUFFIX s;
		uint8	Reserved;
		uint16	P_Key;
		uint32	SLID;
	} PACK_SUFFIX PortXmitConstraintErrorInfo;

	/* PortRcvConstraintErrorInfo */
	struct {
		struct { IB_BITFIELD2(uint8,
			Status : 1,
			Reserved : 7)
		} PACK_SUFFIX s;
		uint8	Reserved;
		uint16	P_Key;
		uint32	SLID;
	} PACK_SUFFIX PortRcvConstraintErrorInfo;

	/* PortRcvSwitchRelayErrorInfo */
	struct {
		struct { IB_BITFIELD3(uint8,
			Status : 1,
			Reserved : 3,
			ErrorCode : 4)
		} PACK_SUFFIX s;
		uint8	Reserved[3];
		union {
			uint32	AsReg32;
			struct {
				uint32	DLID;
			} EI0; 						/* error code 0 */
			struct {
				uint8	EgressPortNum;
				uint8	Reserved[3];
			} EI2; 						/* error code 2 */
			struct {
				uint8	EgressPortNum;
				uint8	SC;
				uint16	Reserved;
			} EI3; 						/* error code 3 */
		} ErrorInfo;
	} PACK_SUFFIX PortRcvSwitchRelayErrorInfo;

	/* UncorrectableErrorInfo */
	struct {
		struct { IB_BITFIELD3(uint8,
			Status : 1,
			Reserved : 3,
			ErrorCode : 4)
		} PACK_SUFFIX s;
		uint8	Reserved;
	} PACK_SUFFIX UncorrectableErrorInfo;

	/* FMConfigErrorInfo */
	struct {
		struct { IB_BITFIELD3(uint8,
			Status : 1,
			Reserved : 3,
			ErrorCode : 4)
		} PACK_SUFFIX s;
		union {
			uint8	AsReg8;
			struct {
				uint8	Distance;
			} EI0to2; 					/* error code 0-2 */
			struct {
				uint8	VL;
			} EI3to5; 					/* error code 3-5 */
			struct {
				uint8	BadFlitBits;	/* bits [63:56] of bad packet */
			} EI6; 						/* error code 6 */
			struct {
				uint8	SC;
			} EI7; 						/* error code 7 */
		} ErrorInfo;
	} PACK_SUFFIX FMConfigErrorInfo;
	uint32	Reserved2;
} PACK_SUFFIX;

typedef struct _STL_Error_Info_Req {
	uint64	PortSelectMask[4];				/* signifies for which ports the PMA is to respond */
	union {
		uint32 AsReg32;
		struct { IB_BITFIELD8(uint32,
			PortRcvErrorInfo : 1,
			ExcessiveBufferOverrunInfo : 1,
			PortXmitConstraintErrorInfo : 1,
			PortRcvConstraintErrorInfo : 1,
			PortRcvSwitchRelayErrorInfo : 1,
			UncorrectableErrorInfo : 1,
			FMConfigErrorInfo : 1,
			Reserved : 25)
		} PACK_SUFFIX s;
	} ErrorInfoSelectMask;
	uint32	Reserved;
} PACK_SUFFIX STLErrorInfoReq, STL_ERROR_INFO_REQ;

typedef struct _STL_Error_Info_Rsp {
	uint64	PortSelectMask[4];				/* signifies for which ports the PMA is to respond */
	union {
		uint32 AsReg32;
		struct { IB_BITFIELD8(uint32,
			PortRcvErrorInfo : 1,
			ExcessiveBufferOverrunInfo : 1,
			PortXmitConstraintErrorInfo : 1,
			PortRcvConstraintErrorInfo : 1,
			PortRcvSwitchRelayErrorInfo : 1,
			UncorrectableErrorInfo : 1,
			FMConfigErrorInfo : 1,
			Reserved : 25)
		} PACK_SUFFIX s;
	} ErrorInfoSelectMask;
	uint32	Reserved;
	struct _port_error_info Port[1]; /* x defined by number of ports in attribute modifier */
} PACK_SUFFIX STLErrorInfoRsp, STL_ERROR_INFO_RSP;

/* Swap routines */

static __inline
void
BSWAP_STL_PORT_STATUS_REQ(STL_PORT_STATUS_REQ *Dest)
{
#if CPU_LE
	Dest->VLSelectMask = ntoh32(Dest->VLSelectMask);
#endif
}

static __inline
void
BSWAP_STL_PORT_STATUS_RSP(STL_PORT_STATUS_RSP *Dest)
{
#if CPU_LE
	int i;
	uint32 VLSelectMask;

	Dest->VLSelectMask =  VLSelectMask 	= ntoh32(Dest->VLSelectMask);
	Dest->PortXmitData 					= ntoh64(Dest->PortXmitData);
	Dest->PortRcvData 					= ntoh64(Dest->PortRcvData);
	Dest->PortXmitPkts 					= ntoh64(Dest->PortXmitPkts);
	Dest->PortRcvPkts 					= ntoh64(Dest->PortRcvPkts);
	Dest->PortMulticastXmitPkts 		= ntoh64(Dest->PortMulticastXmitPkts);
	Dest->PortMulticastRcvPkts 			= ntoh64(Dest->PortMulticastRcvPkts);
	Dest->PortXmitWait 					= ntoh64(Dest->PortXmitWait);
	Dest->SwPortCongestion 				= ntoh64(Dest->SwPortCongestion);
	Dest->PortRcvFECN 					= ntoh64(Dest->PortRcvFECN);
	Dest->PortRcvBECN 					= ntoh64(Dest->PortRcvBECN);
	Dest->PortXmitTimeCong 				= ntoh64(Dest->PortXmitTimeCong);
	Dest->PortXmitWastedBW 				= ntoh64(Dest->PortXmitWastedBW);
	Dest->PortXmitWaitData 				= ntoh64(Dest->PortXmitWaitData);
	Dest->PortRcvBubble 				= ntoh64(Dest->PortRcvBubble);
	Dest->PortMarkFECN 					= ntoh64(Dest->PortMarkFECN);
	Dest->PortRcvConstraintErrors 		= ntoh64(Dest->PortRcvConstraintErrors);
	Dest->PortRcvSwitchRelayErrors 		= ntoh64(Dest->PortRcvSwitchRelayErrors);
	Dest->PortXmitDiscards 				= ntoh64(Dest->PortXmitDiscards);
	Dest->PortXmitConstraintErrors 		= ntoh64(Dest->PortXmitConstraintErrors);
	Dest->PortRcvRemotePhysicalErrors 	= ntoh64(Dest->PortRcvRemotePhysicalErrors);
	Dest->LocalLinkIntegrityErrors 		= ntoh64(Dest->LocalLinkIntegrityErrors);
	Dest->PortRcvErrors 				= ntoh64(Dest->PortRcvErrors);
	Dest->ExcessiveBufferOverruns 		= ntoh64(Dest->ExcessiveBufferOverruns);
	Dest->FMConfigErrors 				= ntoh64(Dest->FMConfigErrors);
	Dest->LinkErrorRecovery 			= ntoh32(Dest->LinkErrorRecovery);
	Dest->LinkDowned 					= ntoh32(Dest->LinkDowned);
	/* Count the bits in the mask and process the VLs in succession */
	/* Assume that even though VL numbers may not be contiguous, that entries */
	/*   in the array are */
	for (i = 0; VLSelectMask && i < 32; VLSelectMask >>= (uint32)1) {
		if (VLSelectMask & (uint32)1) {
			Dest->VLs[i].PortVLXmitData 	= ntoh64(Dest->VLs[i].PortVLXmitData);
			Dest->VLs[i].PortVLRcvData 		= ntoh64(Dest->VLs[i].PortVLRcvData);
			Dest->VLs[i].PortVLXmitPkts 	= ntoh64(Dest->VLs[i].PortVLXmitPkts);
			Dest->VLs[i].PortVLRcvPkts 		= ntoh64(Dest->VLs[i].PortVLRcvPkts);
			Dest->VLs[i].PortVLXmitWait 	= ntoh64(Dest->VLs[i].PortVLXmitWait);
			Dest->VLs[i].SwPortVLCongestion = ntoh64(Dest->VLs[i].SwPortVLCongestion);
			Dest->VLs[i].PortVLRcvFECN 		= ntoh64(Dest->VLs[i].PortVLRcvFECN);
			Dest->VLs[i].PortVLRcvBECN 		= ntoh64(Dest->VLs[i].PortVLRcvBECN);
			Dest->VLs[i].PortVLXmitTimeCong = ntoh64(Dest->VLs[i].PortVLXmitTimeCong);
			Dest->VLs[i].PortVLXmitWastedBW = ntoh64(Dest->VLs[i].PortVLXmitWastedBW);
			Dest->VLs[i].PortVLXmitWaitData = ntoh64(Dest->VLs[i].PortVLXmitWaitData);
			Dest->VLs[i].PortVLRcvBubble 	= ntoh64(Dest->VLs[i].PortVLRcvBubble);
			Dest->VLs[i].PortVLMarkFECN 	= ntoh64(Dest->VLs[i].PortVLMarkFECN);
			Dest->VLs[i].PortVLXmitDiscards = ntoh64(Dest->VLs[i].PortVLXmitDiscards);
			i++;
		}
	}
#endif
}

static __inline void
BSWAP_STL_CLEAR_PORT_STATUS_REQ(STL_CLEAR_PORT_STATUS *Dest)
{
#if CPU_LE
	int i;
	for (i = 0; i < 4; i++)
		Dest->PortSelectMask[i] = ntoh64(Dest->PortSelectMask[i]);
	Dest->CounterSelectMask.AsReg32 = ntoh32(Dest->CounterSelectMask.AsReg32);
#endif
}

static __inline void
BSWAP_STL_DATA_PORT_COUNTERS_REQ(STL_DATA_PORT_COUNTERS_REQ *Dest)
{
#if CPU_LE
	int i;
	for (i = 0; i < 4; i++)
		Dest->PortSelectMask[i] = ntoh64(Dest->PortSelectMask[i]);
	Dest->VLSelectMask = ntoh32(Dest->VLSelectMask);
	Dest->res.AsReg32 = ntoh32(Dest->res.AsReg32);
#endif
}

static __inline void
BSWAP_STL_DATA_PORT_COUNTERS_RSP(STL_DATA_PORT_COUNTERS_RSP *Dest)
{
#if CPU_LE
	int i, j;
	uint8  PortCount = 0;
	uint8  VlCount = 0;
	uint64 PortSelectMask;
	uint32 VLSelectMask;
	struct _port_dpctrs *dPort;

	for (i = 0; i < 4; i++) {
		PortSelectMask = Dest->PortSelectMask[3-i] = ntoh64(Dest->PortSelectMask[3-i]);
		for (j = 0; PortSelectMask && j < 64; j++, PortSelectMask >>= (uint64)1) {
			if (PortSelectMask & (uint64)1) PortCount++;
		}
	}
	VLSelectMask = Dest->VLSelectMask = ntoh32(Dest->VLSelectMask);
	for (i = 0; VLSelectMask && i < 32; i++, VLSelectMask >>= (uint32)1) {
		if (VLSelectMask & (uint32)1) VlCount++;
	}
	Dest->res.AsReg32 = ntoh32(Dest->res.AsReg32);

	dPort = &Dest->Port[0];
	for (i = 0; i < PortCount; i++) {
		dPort->lq.AsReg32 				= ntoh32(dPort->lq.AsReg32);
		dPort->PortXmitData 			= ntoh64(dPort->PortXmitData);
		dPort->PortRcvData 				= ntoh64(dPort->PortRcvData);
		dPort->PortXmitPkts 			= ntoh64(dPort->PortXmitPkts);
		dPort->PortRcvPkts 				= ntoh64(dPort->PortRcvPkts);
		dPort->PortMulticastXmitPkts 	= ntoh64(dPort->PortMulticastXmitPkts);
		dPort->PortMulticastRcvPkts 	= ntoh64(dPort->PortMulticastRcvPkts);
		dPort->PortXmitWait 			= ntoh64(dPort->PortXmitWait);
		dPort->SwPortCongestion 		= ntoh64(dPort->SwPortCongestion);
		dPort->PortRcvFECN 				= ntoh64(dPort->PortRcvFECN);
		dPort->PortRcvBECN 				= ntoh64(dPort->PortRcvBECN);
		dPort->PortXmitTimeCong 		= ntoh64(dPort->PortXmitTimeCong);
		dPort->PortXmitWastedBW 		= ntoh64(dPort->PortXmitWastedBW);
		dPort->PortXmitWaitData 		= ntoh64(dPort->PortXmitWaitData);
		dPort->PortRcvBubble 			= ntoh64(dPort->PortRcvBubble);
		dPort->PortMarkFECN 			= ntoh64(dPort->PortMarkFECN);
		dPort->PortErrorCounterSummary 	= ntoh64(dPort->PortErrorCounterSummary);
		for (j = 0; j < VlCount; j++) {
			dPort->VLs[j].PortVLXmitData 		= ntoh64(dPort->VLs[j].PortVLXmitData);
			dPort->VLs[j].PortVLRcvData 		= ntoh64(dPort->VLs[j].PortVLRcvData);
			dPort->VLs[j].PortVLXmitPkts 		= ntoh64(dPort->VLs[j].PortVLXmitPkts);
			dPort->VLs[j].PortVLRcvPkts 		= ntoh64(dPort->VLs[j].PortVLRcvPkts);
			dPort->VLs[j].PortVLXmitWait 		= ntoh64(dPort->VLs[j].PortVLXmitWait);
			dPort->VLs[j].SwPortVLCongestion 	= ntoh64(dPort->VLs[j].SwPortVLCongestion);
			dPort->VLs[j].PortVLRcvFECN 		= ntoh64(dPort->VLs[j].PortVLRcvFECN);
			dPort->VLs[j].PortVLRcvBECN 		= ntoh64(dPort->VLs[j].PortVLRcvBECN);
			dPort->VLs[j].PortVLXmitTimeCong 	= ntoh64(dPort->VLs[j].PortVLXmitTimeCong);
			dPort->VLs[j].PortVLXmitWastedBW 	= ntoh64(dPort->VLs[j].PortVLXmitWastedBW);
			dPort->VLs[j].PortVLXmitWaitData 	= ntoh64(dPort->VLs[j].PortVLXmitWaitData);
			dPort->VLs[j].PortVLRcvBubble 		= ntoh64(dPort->VLs[j].PortVLRcvBubble);
			dPort->VLs[j].PortVLMarkFECN 		= ntoh64(dPort->VLs[j].PortVLMarkFECN);
		}
		dPort = (struct _port_dpctrs *)&dPort->VLs[VlCount];
	}
#endif
}

static __inline void
BSWAP_STL_ERROR_PORT_COUNTERS_REQ(STL_ERROR_PORT_COUNTERS_REQ *Dest)
{
#if CPU_LE
	int i;
	for (i = 0; i < 4; i++)
		Dest->PortSelectMask[i] = ntoh64(Dest->PortSelectMask[i]);
	Dest->VLSelectMask = ntoh32(Dest->VLSelectMask);
#endif
}

static __inline void
BSWAP_STL_ERROR_PORT_COUNTERS_RSP(STL_ERROR_PORT_COUNTERS_RSP *Dest)
{
#if CPU_LE
	int i, j;
	uint8  PortCount = 0;
	uint8  VlCount = 0;
	uint64 PortSelectMask;
	uint32 VLSelectMask;
	struct _port_epctrs *ePort;

	for (i = 0; i < 4; i++) {
		PortSelectMask = Dest->PortSelectMask[3-i] = ntoh64(Dest->PortSelectMask[3-i]);
		for (j = 0; PortSelectMask && j < 64; j++, PortSelectMask >>= (uint64)1) {
			if (PortSelectMask & (uint64)1) PortCount++;
		}
	}
	VLSelectMask = Dest->VLSelectMask = ntoh32(Dest->VLSelectMask);
	for (i = 0; VLSelectMask && i < 32; i++, VLSelectMask >>= (uint32)1) {
		if (VLSelectMask & (uint32)1) VlCount++;
	}

	ePort = &Dest->Port[0];
	for (i = 0; i < PortCount; i++) {
		ePort->PortRcvConstraintErrors 		= ntoh64(ePort->PortRcvConstraintErrors);
		ePort->PortRcvSwitchRelayErrors 	= ntoh64(ePort->PortRcvSwitchRelayErrors);
		ePort->PortXmitDiscards 			= ntoh64(ePort->PortXmitDiscards);
		ePort->PortXmitConstraintErrors 	= ntoh64(ePort->PortXmitConstraintErrors);
		ePort->PortRcvRemotePhysicalErrors 	= ntoh64(ePort->PortRcvRemotePhysicalErrors);
		ePort->LocalLinkIntegrityErrors 	= ntoh64(ePort->LocalLinkIntegrityErrors);
		ePort->PortRcvErrors 				= ntoh64(ePort->PortRcvErrors);
		ePort->ExcessiveBufferOverruns 		= ntoh64(ePort->ExcessiveBufferOverruns);
		ePort->FMConfigErrors 				= ntoh64(ePort->FMConfigErrors);
		ePort->LinkErrorRecovery 			= ntoh32(ePort->LinkErrorRecovery);
		ePort->LinkDowned 					= ntoh32(ePort->LinkDowned);

		for (j = 0; j < VlCount; j++) {
			ePort->VLs[j].PortVLXmitDiscards = ntoh64(ePort->VLs[j].PortVLXmitDiscards);
		}
		ePort = (struct _port_epctrs *)(&ePort->VLs[VlCount]);
	}
#endif
}

static __inline void
BSWAP_STL_ERROR_INFO_REQ(STL_ERROR_INFO_REQ *Dest)
{
#if CPU_LE
	int i;

	for (i = 0; i < 4; i++) {
		Dest->PortSelectMask[3-i] = ntoh64(Dest->PortSelectMask[3-i]);
	}
	Dest->ErrorInfoSelectMask.AsReg32 = ntoh32(Dest->ErrorInfoSelectMask.AsReg32);
#endif
}

static __inline void
BSWAP_STL_ERROR_INFO_RSP(STL_ERROR_INFO_RSP *Dest)
{
#if CPU_LE
	int i, j;
	uint8  PortCount = 0;
	uint64 PortSelectMask;

	for (i = 0; i < 4; i++) {
		PortSelectMask = Dest->PortSelectMask[3-i] = ntoh64(Dest->PortSelectMask[3-i]);
		for (j = 0; PortSelectMask && j < 64; j++, PortSelectMask >>= (uint64)1) {
			if (PortSelectMask & (uint64)1) PortCount++;
		}
	}
	for (i = 0; i < PortCount; i++) {
		Dest->Port[i].PortXmitConstraintErrorInfo.P_Key = ntoh16(Dest->Port[i].PortXmitConstraintErrorInfo.P_Key);
		Dest->Port[i].PortXmitConstraintErrorInfo.SLID 	= ntoh32(Dest->Port[i].PortXmitConstraintErrorInfo.SLID);
		Dest->Port[i].PortRcvConstraintErrorInfo.P_Key 	= ntoh16(Dest->Port[i].PortRcvConstraintErrorInfo.P_Key);
		Dest->Port[i].PortRcvConstraintErrorInfo.SLID 	= ntoh32(Dest->Port[i].PortRcvConstraintErrorInfo.SLID);
		if (Dest->Port[i].PortRcvSwitchRelayErrorInfo.s.ErrorCode == 0)
			Dest->Port[i].PortRcvSwitchRelayErrorInfo.ErrorInfo.AsReg32 = ntoh32(Dest->Port[i].PortRcvSwitchRelayErrorInfo.ErrorInfo.AsReg32);
	}
	Dest->ErrorInfoSelectMask.AsReg32 = ntoh32(Dest->ErrorInfoSelectMask.AsReg32);
#endif
}

/**
 * Copy data in a STL_PORT_COUNTERS_DATA variable into a STL_PortStatusData_t variable
 *
 * @param portCounters   - pointer to STL_PORT_COUNTERS_DATA variable from which to copy
 * @param portStatusData - pointer to STL_PortStatusData_t variable to copy to
 *
 */
static __inline void
StlPortCountersToPortStatus(STL_PORT_COUNTERS_DATA *portCounters, STL_PORT_STATUS_RSP *portStatusData)
{
	portStatusData->LinkErrorRecovery  = portCounters->linkErrorRecovery;
	portStatusData->LinkDowned  = portCounters->linkDowned;
	portStatusData->PortRcvErrors = portCounters->portRcvErrors;
	portStatusData->PortRcvRemotePhysicalErrors = portCounters->portRcvRemotePhysicalErrors;
	portStatusData->PortRcvSwitchRelayErrors = portCounters->portRcvSwitchRelayErrors;
	portStatusData->PortXmitDiscards = portCounters->portXmitDiscards;
	portStatusData->PortXmitConstraintErrors = portCounters->portXmitConstraintErrors;
	portStatusData->PortRcvConstraintErrors = portCounters->portRcvConstraintErrors;
	portStatusData->LocalLinkIntegrityErrors = portCounters->localLinkIntegrityErrors;
	portStatusData->ExcessiveBufferOverruns = portCounters->excessiveBufferOverruns;
	portStatusData->PortXmitData = portCounters->portXmitData;
	portStatusData->PortRcvData = portCounters->portRcvData;
	portStatusData->PortXmitPkts = portCounters->portXmitPkts;
	portStatusData->PortRcvPkts = portCounters->portRcvPkts;
	portStatusData->PortMulticastXmitPkts = portCounters->portMulticastXmitPkts;
	portStatusData->PortMulticastRcvPkts = portCounters->portMulticastRcvPkts;
	portStatusData->PortXmitWait = portCounters->portXmitWait;
	portStatusData->SwPortCongestion = portCounters->swPortCongestion;
	portStatusData->PortRcvFECN = portCounters->portRcvFECN;
	portStatusData->PortRcvBECN = portCounters->portRcvBECN;
	portStatusData->PortXmitTimeCong = portCounters->portXmitTimeCong;
	portStatusData->PortXmitWastedBW = portCounters->portXmitWastedBW;
	portStatusData->PortXmitWaitData = portCounters->portXmitWaitData;
	portStatusData->PortRcvBubble = portCounters->portRcvBubble;
	portStatusData->PortMarkFECN = portCounters->portMarkFECN;
	portStatusData->FMConfigErrors = portCounters->fmConfigErrors;
	portStatusData->UncorrectableErrors = portCounters->uncorrectableErrors;
	portStatusData->lq.AsReg8 = portCounters->lq.AsReg8;
}

#include "iba/public/ipackoff.h"

#if defined (__cplusplus)
};
#endif

#endif /* __IBA_STL_PM_H__ */
