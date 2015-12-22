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

#if defined(CHECK_HEADERS)

#ifndef __IBA_STL_PM_H__
#warning FIX ME!!! Your includes should use the stl_pm.h header and not the ib_pm.h header for STL builds
#endif

#endif

#ifndef _IBA_IB_PM_H_
#define _IBA_IB_PM_H_ (1) /* suppress duplicate loading of this file */

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"
#include "iba/ib_generalServices.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Performance Management methods */
#define	PERFMGT_GET				0x01		/*	MMTHD_GET */
#define	PERFMGT_SET				0x02		/*	MMTHD_SET */
#define	PERFMGT_GET_RESP		0x81		/*	MMTHD_GET_RESP */


/* Performance Management Attributes IDs */	/* Attribute Modifier values */
#define	PM_ATTRIB_ID_CLASS_PORTINFO			0x0001	/* 0x0000_0000 */
#define PM_ATTRIB_ID_PORT_SAMPLES_CONTROL	0x0010	/* 0x0000_0000 - 0xFFFF_FFFF */
										/*   sampling mechanism, FFFFFFFF -> all */
#define	PM_ATTRIB_ID_PORT_SAMPLES_RESULT	0x0011	/* 0x0000_0000 - 0xFFFF_FFFF */
										/*   sampling mechanism, FFFFFFFF -> all */
#define	PM_ATTRIB_ID_PORT_COUNTERS			0x0012	/* 0x0000_0000 */
/* optional attributes */
#define	PM_ATTRIB_ID_PORT_RCV_ERROR_DETAILS		0x0015	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_XMIT_DISCARD_DETAILS	0x0016	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_OP_RCV_COUNTERS		0x0017	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_FLOW_CTL_COUNTERS		0x0018	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_VL_OP_PACKETS			0x0019	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_VL_OP_DATA			0x001A	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_VL_XMIT_FLOW_CTL_UPDATE_ERRORS		0x001B	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_VL_XMIT_WAIT_COUNTERS	0x001C	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_COUNTERS_EXTENDED		0x001D	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_PORT_SAMPLES_RESULT_EXTENDED 0x001E	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_SW_PORT_VL_CONGESTION		0x0030	/* 0x0000_0000 */
#define	PM_ATTRIB_ID_VENDOR_PORT_COUNTERS		0xff00	/* 0xCC00_00PP */

#include "iba/public/ipackon.h"

/* if AllPortSelect capability, use this as PortNum to operate on all ports */
#define PM_ALL_PORT_SELECT 0xff

/*
 * PMA Class Port Info
 */

	/* Class-specific Capability Mask bits */
#define PM_CLASS_PORT_CAPMASK_SAMPLES_ONLY_SUPPORTED	0x0800
		/* PortSamplesControl values which can be sampled but not polled
		 * are available
		 */

#define PM_CLASS_PORT_CAPMASK_PORT_COUNTERS_XMIT_WAIT_SUPPORTED	0x1000
		/* PortCounters:PortXmitWait supported */

	/* Vendor-specific Capability Mask2 bits */
#define PM_CLASS_PORT_CAPMASK2_VENDOR_PORT_COUNTERS	0x04000000
		/* TrueScale VENDOR_PORT_COUNTERS attribute is supported */

#define PM_CLASS_PORT_CAPMASK2_VENDOR_PORT_COUNTERS2	0x02000000
		/* TrueScale VENDOR_PORT_COUNTERS allows clear thresholds */

/* CounterSelect values in PortSamplesControl */
typedef enum _SAMPLE_SELECT {
	/* 0x0000 reserved */
	SELECT_PORT_XMIT_DATA = 0x0001,
	SELECT_PORT_RCV_DATA = 0x0002,
	SELECT_PORT_XMIT_PKTS = 0x0003,
	SELECT_PORT_RCV_PKTS = 0x0004,
	SELECT_PORT_XMIT_WAIT = 0x0005,
	/* 0x0007-0x3fff reserved */
	/* optional */
	SELECT_PORT_XMIT_QUEUE_N = 0x4000,	/* 0x0n00 -> VL # */
	SELECT_PORT_XMIT_DATA_VL_N = 0x4001,	/* 0x0n00 -> VL # */
	SELECT_PORT_RCV_DATA_VL_N = 0x4002,	/* 0x0n00 -> VL # */
	SELECT_PORT_XMIT_PKT_VL_N = 0x4003,	/* 0x0n00 -> VL # */
	SELECT_PORT_RCV_PKT_VL_N = 0x4004,	/* 0x0n00 -> VL # */
	SELECT_PORT_RCV_ERROR_DETAILS_PORT_LOCAL_PHYSICAL_ERRORS = 0x4005,
	SELECT_PORT_RCV_ERROR_DETAILS_PORT_MALFORMED_PACKET_ERRORS = 0x4006,
	SELECT_PORT_RCV_ERROR_DETAILS_PORT_BUFFER_OVERRUN_ERRORS = 0x4007,
	SELECT_PORT_RCV_ERROR_DETAILS_PORT_DLID_MAPPING_ERRORS = 0x4008,
	SELECT_PORT_RCV_ERROR_DETAILS_PORT_VL_MAPPING_ERRORS = 0x4009,
	SELECT_PORT_RCV_ERROR_DETAILS_PORT_LOOPING_ERRORS = 0x400A,
	SELECT_PORT_XMIT_DiscardDETAILS_PORT_INACTIVE_DISCARDS = 0x400B,
	SELECT_PORT_XMIT_DiscardDETAILS_PORT_NEIGHBOR_MTU_DISCARDS = 0x400C,
	SELECT_PORT_XMIT_DiscardDETAILS_PORT_SW_LIFETIME_LIMIT_DISCARDS = 0x400D,
	SELECT_PORT_XMIT_DiscardDETAILS_PORT_SW_HOQ_LIFETIME_LIMIT_DISCARDS = 0x400E,
	SELECT_PORT_OP_RCV_COUNTERS_PORT_OP_RCV_PKTS = 0x400F,
	SELECT_PORT_OP_RCV_COUNTERS_PORT_OP_RCV_DATA = 0x4010,
	SELECT_PORT_FLOW_CTL_COUNTERS_PORT_XMIT_FLOW_PKTS = 0x4011,
	SELECT_PORT_FLOW_CTL_COUNTERS_PORT_RCV_FLOW_PKTS = 0x4012,
	SELECT_PORT_VL_OP_PACKETS_PORT_VL_OP_PACKETS_N = 0x4013,	/* 0x0n00 -> VL # */
	SELECT_PORT_VL_OP_DATA_PORT_VL_OP_DATA_N = 0x4014,	/* 0x0n00 -> VL # */
	SELECT_PORT_VL_XMIT_FLOW_CTL_UPDATE_ERRORS_PORT_VL_XMIT_FLOW_CTL_UPDATE_ERRORS_N = 0x4015,	/* 0x0n00 -> VL # */
	SELECT_PORT_VL_XMIT_WAIT_COUNTERS_PORT_VL_XMIT_WAIT_N = 0x4016,	/* 0x0n00 -> VL # */
	SELECT_SwPORT_VL_CONGESTION_SW_PORT_VL_CONGESTION = 0x4030,	/* 0x0n00 -> VL # */
	/* 0xc000 - 0xffff reserved */
} SAMPLE_SELECT;

/* mask bits used in OptionsMask and SamplesOnlyOptionMask */
typedef union {
		uint64	AsUint64;
		struct {
#if CPU_BE
			uint64	Reserved7:7;
			/* 2 fields below are new in IBTA 1.2.1 */
			uint64	PortRcvDataSL:1;
			uint64	PortXmitDataSL:1;
			uint64	PortVLXmitTimeCong_PortVLXmitTimeCong:1;
			uint64	PortXmitConCtrl_PortXmitTimeCong:1;
			uint64	PortSLRcvBECN_PortSLPktRcvBECN:1;
			uint64	PortSLRcvFECN_PortSLPktRcvFECN:1;
			uint64	PortRcvConCtrl_PortPktRcvBECN:1;
			uint64	PortRcvConCtrl_PortPktRcvFECN:1;
			uint64	SwPortVLCongestion_SWPortVLCongestion:1;
			uint64	Reserved6:24;
			uint64	PortVLXmitWaitCounters_PortVLXmitWait_n:1;
			uint64	PortVLXmitFlowCtlUpdateErrors_PortVLXmitFlowCtlUpdateErrors_n:1;
			uint64	PortVLOpData_PortVLOpData_n:1;
			uint64	PortVLOpPackets_PortVLOpPackets_n:1;
			uint64	PortFlowCtlCounters_PortRcvFlowPkts:1;
			uint64	PortFlowCtlCounters_PortXmitFlowPkts:1;
			uint64	PortOpRcvCounters_PortOpRcvData:1;
			uint64	PortOpRcvCounters_PortOpRcvPkts:1;
			uint64	PortXmitDiscardDetails_PortSwHOQLifetimeLimitDiscards:1;
			uint64	PortXmitDiscardDetails_PortSwLifetimeLimitDiscards:1;
			uint64	PortXmitDiscardDetails_PortNeighborMTUDiscards:1;
			uint64	PortXmitDiscardDetails_PortInactiveDiscards:1;
			uint64	PortRcvErrorDetails_PortLoopingErrors:1;
			uint64	PortRcvErrorDetails_PortVLMappingErrors:1;
			uint64	PortRcvErrorDetails_PortDLIDMappingErrors:1;
			uint64	PortRcvErrorDetails_PortBufferOverrunErrors:1;
			uint64	PortRcvErrorDetails_PortMalformedPacketErrors:1;
			uint64	PortRcvErrorDetails_PortLocalPhysicalErrors:1;
			uint64	PortRcvPktVL_n:1;
			uint64	PortXmitPktVL_n:1;
			uint64	PortRcvDataVL_n:1;
			uint64	PortXmitDataVL_n:1;
			uint64	PortXmitQueue_n:1;
			uint16	Reserved5:1;
#else			/* CPU_LE */
			uint16	Reserved5:1;
			uint64	PortXmitQueue_n:1;
			uint64	PortXmitDataVL_n:1;
			uint64	PortRcvDataVL_n:1;
			uint64	PortXmitPktVL_n:1;
			uint64	PortRcvPktVL_n:1;
			uint64	PortRcvErrorDetails_PortLocalPhysicalErrors:1;
			uint64	PortRcvErrorDetails_PortMalformedPacketErrors:1;
			uint64	PortRcvErrorDetails_PortBufferOverrunErrors:1;
			uint64	PortRcvErrorDetails_PortDLIDMappingErrors:1;
			uint64	PortRcvErrorDetails_PortVLMappingErrors:1;
			uint64	PortRcvErrorDetails_PortLoopingErrors:1;
			uint64	PortXmitDiscardDetails_PortInactiveDiscards:1;
			uint64	PortXmitDiscardDetails_PortNeighborMTUDiscards:1;
			uint64	PortXmitDiscardDetails_PortSwLifetimeLimitDiscards:1;
			uint64	PortXmitDiscardDetails_PortSwHOQLifetimeLimitDiscards:1;
			uint64	PortOpRcvCounters_PortOpRcvPkts:1;
			uint64	PortOpRcvCounters_PortOpRcvData:1;
			uint64	PortFlowCtlCounters_PortXmitFlowPkts:1;
			uint64	PortFlowCtlCounters_PortRcvFlowPkts:1;
			uint64	PortVLOpPackets_PortVLOpPackets_n:1;
			uint64	PortVLOpData_PortVLOpData_n:1;
			uint64	PortVLXmitFlowCtlUpdateErrors_PortVLXmitFlowCtlUpdateErrors_n:1;
			uint64	PortVLXmitWaitCounters_PortVLXmitWait_n:1;
			uint64	Reserved6:24;
			uint64	SwPortVLCongestion_SWPortVLCongestion:1;
			uint64	PortRcvConCtrl_PortPktRcvFECN:1;
			uint64	PortRcvConCtrl_PortPktRcvBECN:1;
			uint64	PortSLRcvFECN_PortSLPktRcvFECN:1;
			uint64	PortSLRcvBECN_PortSLPktRcvBECN:1;
			uint64	PortXmitConCtrl_PortXmitTimeCong:1;
			uint64	PortVLXmitTimeCong_PortVLXmitTimeCong:1;
			/* 2 fields below are new in IBTA 1.2.1 */
			uint64	PortXmitDataSL:1;
			uint64	PortRcvDataSL:1;
			uint64	Reserved7:7;
#endif
		} PACK_SUFFIX s;
	} PACK_SUFFIX PMA_OptionMask_t;

typedef struct __PortSamplesControl {
	uint8		OpCode;
	uint8		PortSelect;			/* 0xFF->all ports */
	uint8		Tick;
#if CPU_BE
	uint8		Reserved:5;
	uint8		CounterWidth:3;
#else			/* CPU_LE */
	uint8		CounterWidth:3;
	uint8		Reserved:5;
#endif
	union {
		uint32	AsUint32;
		struct {
#if CPU_BE
			uint32		Reserved2:2;
			uint32		Counter0Mask:3;
			uint32		Counter1Mask:3;
			uint32		Counter2Mask:3;
			uint32		Counter3Mask:3;
			uint32		Counter4Mask:3;
			uint32		Counter5Mask:3;
			uint32		Counter6Mask:3;
			uint32		Counter7Mask:3;
			uint32		Counter8Mask:3;
			uint32		Counter9Mask:3;
#else			/* CPU_LE */
			uint32		Counter9Mask:3;
			uint32		Counter8Mask:3;
			uint32		Counter7Mask:3;
			uint32		Counter6Mask:3;
			uint32		Counter5Mask:3;
			uint32		Counter4Mask:3;
			uint32		Counter3Mask:3;
			uint32		Counter2Mask:3;
			uint32		Counter1Mask:3;
			uint32		Counter0Mask:3;
			uint32		Reserved2:2;
#endif
		} s;
	} u1;
	union {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint16		Reserved3:1;
			uint16		Counter10Mask:3;
			uint16		Counter11Mask:3;
			uint16		Counter12Mask:3;
			uint16		Counter13Mask:3;
			uint16		Counter14Mask:3;
#else			/* CPU_LE */
			uint16		Counter14Mask:3;
			uint16		Counter13Mask:3;
			uint16		Counter12Mask:3;
			uint16		Counter11Mask:3;
			uint16		Counter10Mask:3;
			uint16		Reserved3:1;
#endif
		} s;
	} u2;
	uint8		SampleMechanisms;
#if CPU_BE
	uint8		Reserved4:6;
	uint8		SampleStatus:2;
#else			/* CPU_LE */
	uint8		SampleStatus:2;
	uint8		Reserved4:6;
#endif
	PMA_OptionMask_t OptionMask;	/* which counters are implemented */
	uint64		VendorMask;
	uint32		SampleStart;
	uint32		SampleInterval;
	uint16		Tag;
	uint16		CounterSelect[15];
	/* fields below are new in IBTA 1.2.1 */
	uint32		Reserved5;
	PMA_OptionMask_t SamplesOnlyOptionMask;	/* which counters are supported in samples-only mode */
} PACK_SUFFIX PortSamplesControl, PORT_SAMPLES_CONTROL;

typedef struct __PortSamplesResult {
	uint16		Tag;
	uint8		Reserved;
#if CPU_BE
	uint8		Reserved2:6;
	uint8		SampleStatus:2;
#else			/* CPU_LE */
	uint8		SampleStatus:2;
	uint8		Reserved2:6;
#endif
	uint32		Counter[15];
} PACK_SUFFIX PortSamplesResult, PORT_SAMPLES_RESULT;

typedef struct __PortCounters {
	uint8		Reserved;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortCountersSelect_u {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint16 PortRcvPkts:1;
			uint16 PortXmitPkts:1;
			uint16 PortRcvData:1;
			uint16 PortXmitData:1;
			uint16 VL15Dropped:1;
			uint16 ExcessiveBufferOverrunErrors:1;
			uint16 LocalLinkIntegrityErrors:1;
			uint16 PortRcvConstraintErrors:1;
			uint16 PortXmitConstraintErrors:1;
			uint16 PortXmitDiscards:1;
			uint16 PortRcvSwitchRelayErrors:1;
			uint16 PortRcvRemotePhysicalErrors:1;
			uint16 PortRcvErrors:1;
			uint16 LinkDownedCounter:1;
			uint16 LinkErrorRecoveryCounter:1;
			uint16 SymbolErrorCounter:1;
#else			/* CPU_LE */
			uint16 SymbolErrorCounter:1;
			uint16 LinkErrorRecoveryCounter:1;
			uint16 LinkDownedCounter:1;
			uint16 PortRcvErrors:1;
			uint16 PortRcvRemotePhysicalErrors:1;
			uint16 PortRcvSwitchRelayErrors:1;
			uint16 PortXmitDiscards:1;
			uint16 PortXmitConstraintErrors:1;
			uint16 PortRcvConstraintErrors:1;
			uint16 LocalLinkIntegrityErrors:1;
			uint16 ExcessiveBufferOverrunErrors:1;
			uint16 VL15Dropped:1;
			uint16 PortXmitData:1;
			uint16 PortRcvData:1;
			uint16 PortXmitPkts:1;
			uint16 PortRcvPkts:1;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint16		SymbolErrorCounter;
	uint8		LinkErrorRecoveryCounter;
	uint8		LinkDownedCounter;
	uint16		PortRcvErrors;
	uint16		PortRcvRemotePhysicalErrors;
	uint16		PortRcvSwitchRelayErrors;
	uint16		PortXmitDiscards;
	uint8		PortXmitConstraintErrors;
	uint8		PortRcvConstraintErrors;
	uint8		Reserved1;
#if CPU_BE
	uint8		LocalLinkIntegrityErrors:4;
	uint8		ExcessiveBufferOverrunErrors:4;
#else			/* CPU_LE */
	uint8		ExcessiveBufferOverrunErrors:4;
	uint8		LocalLinkIntegrityErrors:4;
#endif
	uint16		Reserved2;
	uint16		VL15Dropped;
	uint32		PortXmitData;
	uint32		PortRcvData;
	uint32		PortXmitPkts;
	uint32		PortRcvPkts;
	uint32		PortXmitWait;	/* new in IBTA 1.2.1 */
} PACK_SUFFIX PortCounters, PORT_COUNTERS;



/* optional attributes */
typedef struct __PortRcvErrorDetails {
	uint8		Reserved;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortRcvErrorDetailsSelect_u {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint64	Reserved1:10;
			uint16 PortLoopingErrors:1;
			uint16 PortVLMappingErrors:1;
			uint16 PortDLIDMappingErrors:1;
			uint16 PortBufferOverrunErrors:1;
			uint16 PortMalformedPacketErrors:1;
			uint16 PortLocalPhysicalErrors:1;
#else			/* CPU_LE */
			uint16 PortLocalPhysicalErrors:1;
			uint16 PortMalformedPacketErrors:1;
			uint16 PortBufferOverrunErrors:1;
			uint16 PortDLIDMappingErrors:1;
			uint16 PortVLMappingErrors:1;
			uint16 PortLoopingErrors:1;
			uint64	Reserved1:10;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint16 PortLocalPhysicalErrors;
	uint16 PortMalformedPacketErrors;
	uint16 PortBufferOverrunErrors;
	uint16 PortDLIDMappingErrors;
	uint16 PortVLMappingErrors;
	uint16 PortLoopingErrors;
} PACK_SUFFIX PortRcvErrorDetails, PORT_RCV_ERROR_DETAILS;

typedef struct __PortXmitDiscardDetails {
	uint8		Reserved;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortXmitDiscardDetailsSelect_u {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint64	Reserved1:12;
			uint16 PortSwHOQLifetimeLimitDiscards:1;
			uint16 PortSwLifetimeLimitDiscards:1;
			uint16 PortNeighborMTUDiscards:1;
			uint16 PortInactiveDiscards:1;
#else			/* CPU_LE */
			uint16 PortInactiveDiscards:1;
			uint16 PortNeighborMTUDiscards:1;
			uint16 PortSwLifetimeLimitDiscards:1;
			uint16 PortSwHOQLifetimeLimitDiscards:1;
			uint64	Reserved1:12;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint16 PortInactiveDiscards;
	uint16 PortNeighborMTUDiscards;
	uint16 PortSwLifetimeLimitDiscards;
	uint16 PortSwHOQLifetimeLimitDiscards;
} PACK_SUFFIX PortXmitDiscardDetails, PORT_XMIT_DISCARD_DETAILS;

typedef struct __PortOpRcvCounters {
	uint8		OpCode;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortOpRcvCountersSelect_u {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint64	Reserved1:14;
			uint16 PortOpRcvData:1;
			uint16 PortOpRcvPkts:1;
#else			/* CPU_LE */
			uint16 PortOpRcvPkts:1;
			uint16 PortOpRcvData:1;
			uint64	Reserved1:14;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint32 PortOpRcvPkts;
	uint32 PortOpRcvData;
} PACK_SUFFIX PortOpRcvCounters, PORT_OP_RCV_COUNTERS;

typedef struct __PortFlowCtlCounters {
	uint8		Reserved;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortFlowCtlCountersSelect_u {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint64	Reserved1:14;
			uint16 PortRcvFlowPkts:1;
			uint16 PortXmitFlowPkts:1;
#else			/* CPU_LE */
			uint16 PortXmitFlowPkts:1;
			uint16 PortRcvFlowPkts:1;
			uint64	Reserved1:14;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint32 PortXmitFlowPkts;
	uint32 PortRcvFlowPkts;
} PACK_SUFFIX PortFlowCtlCounters, PORT_FLOW_CTL_COUNTERS;

/* This same data structure is used for all per VL 16 bit counters:
 * 	PortVLOpPackets, PortVLXmitWaitCounters, SwPortVLCongestion
 */
typedef struct __PortVLCounters16 {
	uint8		OpCode;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortVLCounters16Select_u {
		uint16	AsUint16;	/* 1 << VL */
		struct {
#if CPU_BE
			uint16 PortVL15:1;
			uint16 PortVL14:1;
			uint16 PortVL13:1;
			uint16 PortVL12:1;
			uint16 PortVL11:1;
			uint16 PortVL10:1;
			uint16 PortVL9:1;
			uint16 PortVL8:1;
			uint16 PortVL7:1;
			uint16 PortVL6:1;
			uint16 PortVL5:1;
			uint16 PortVL4:1;
			uint16 PortVL3:1;
			uint16 PortVL2:1;
			uint16 PortVL1:1;
			uint16 PortVL0:1;
#else			/* CPU_LE */
			uint16 PortVL0:1;
			uint16 PortVL1:1;
			uint16 PortVL2:1;
			uint16 PortVL3:1;
			uint16 PortVL4:1;
			uint16 PortVL5:1;
			uint16 PortVL6:1;
			uint16 PortVL7:1;
			uint16 PortVL8:1;
			uint16 PortVL9:1;
			uint16 PortVL10:1;
			uint16 PortVL11:1;
			uint16 PortVL12:1;
			uint16 PortVL13:1;
			uint16 PortVL14:1;
			uint16 PortVL15:1;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint16 PortVLCounter[16];
} PACK_SUFFIX PortVLCounters16, PORT_VL_COUNTERS16,
 	PortVLOpPackets, PORT_VL_OP_PACKETS,
 	PortVLXmitWaitCounters, PORT_VL_XMIT_WAIT_COUNTERS,
 	SwPortVLCongestion, SW_PORT_VL_CONGESTION;

/* This same data structure is used for all per VL 32 bit counters:
 * 	PortVLOpData
 */
typedef struct __PortVLCounters32 {
	uint8		OpCode;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortVLCounters32Select_u {
		uint16	AsUint16;	/* 1 << VL */
		struct {
#if CPU_BE
			uint16 PortVL15:1;
			uint16 PortVL14:1;
			uint16 PortVL13:1;
			uint16 PortVL12:1;
			uint16 PortVL11:1;
			uint16 PortVL10:1;
			uint16 PortVL9:1;
			uint16 PortVL8:1;
			uint16 PortVL7:1;
			uint16 PortVL6:1;
			uint16 PortVL5:1;
			uint16 PortVL4:1;
			uint16 PortVL3:1;
			uint16 PortVL2:1;
			uint16 PortVL1:1;
			uint16 PortVL0:1;
#else			/* CPU_LE */
			uint16 PortVL0:1;
			uint16 PortVL1:1;
			uint16 PortVL2:1;
			uint16 PortVL3:1;
			uint16 PortVL4:1;
			uint16 PortVL5:1;
			uint16 PortVL6:1;
			uint16 PortVL7:1;
			uint16 PortVL8:1;
			uint16 PortVL9:1;
			uint16 PortVL10:1;
			uint16 PortVL11:1;
			uint16 PortVL12:1;
			uint16 PortVL13:1;
			uint16 PortVL14:1;
			uint16 PortVL15:1;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint32 PortVLCounter[16];
} PACK_SUFFIX PortVLCounters32, PORT_VL_COUNTERS32,
 	PortVLOpData, PORT_VL_OP_DATA;

/* This same data structure is used for all per VL 2 bit counters:
 * 	PortVLXmitFlowCtlUpdateErrors
 */
typedef struct __PortVLCounters2 {
	uint8		Reserved;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortVLCounters2Select_u {
		uint16	AsUint16;	/* 1 << VL */
		struct {
#if CPU_BE
			uint16 PortVL15:1;
			uint16 PortVL14:1;
			uint16 PortVL13:1;
			uint16 PortVL12:1;
			uint16 PortVL11:1;
			uint16 PortVL10:1;
			uint16 PortVL9:1;
			uint16 PortVL8:1;
			uint16 PortVL7:1;
			uint16 PortVL6:1;
			uint16 PortVL5:1;
			uint16 PortVL4:1;
			uint16 PortVL3:1;
			uint16 PortVL2:1;
			uint16 PortVL1:1;
			uint16 PortVL0:1;
#else			/* CPU_LE */
			uint16 PortVL0:1;
			uint16 PortVL1:1;
			uint16 PortVL2:1;
			uint16 PortVL3:1;
			uint16 PortVL4:1;
			uint16 PortVL5:1;
			uint16 PortVL6:1;
			uint16 PortVL7:1;
			uint16 PortVL8:1;
			uint16 PortVL9:1;
			uint16 PortVL10:1;
			uint16 PortVL11:1;
			uint16 PortVL12:1;
			uint16 PortVL13:1;
			uint16 PortVL14:1;
			uint16 PortVL15:1;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	IB_BITFIELD4( uint8,
			PortVLCounter0,
			PortVLCounter1,
			PortVLCounter2,
			PortVLCounter3)
	IB_BITFIELD4( uint8,
			PortVLCounter4,
			PortVLCounter5,
			PortVLCounter6,
			PortVLCounter7)
	IB_BITFIELD4( uint8,
			PortVLCounter8,
			PortVLCounter9,
			PortVLCounter10,
			PortVLCounter11)
	IB_BITFIELD4( uint8,
			PortVLCounter12,
			PortVLCounter13,
			PortVLCounter14,
			PortVLCounter15)
} PACK_SUFFIX PortVLCounters2, PORT_VL_COUNTERS2,
 PortVLXmitFlowCtlUpdateErrors, PORT_VL_XMIT_FLOW_CTL_UPDATE_ERRORS;

/* values for ExtendedWidth field */
#define PORT_SAMPLES_RESULT_EXTENDED_WIDTH_48	0x1
#define PORT_SAMPLES_RESULT_EXTENDED_WIDTH_64	0x2

typedef struct __PortSamplesResultExtended {
	uint16		Tag;
	uint8		Reserved;
#if CPU_BE
	uint8		Reserved2:6;
	uint8		SampleStatus:2;
#else			/* CPU_LE */
	uint8		SampleStatus:2;
	uint8		Reserved2:6;
#endif
	IB_BITFIELD2( uint32,
			ExtendedWidth:2,
			Reserved3:30)
	uint64		Counter[15];
} PACK_SUFFIX PortSamplesResultExtended, PORT_SAMPLES_RESULT_EXTENDED;

typedef struct __PortCountersExtended {
	uint8		Reserved;
	uint8		PortSelect;			/* 0xFF->all ports */
	union PortCountersExtendedSelect_u {
		uint16	AsUint16;
		struct {
#if CPU_BE
			uint16 Reserved:8;
			uint16 PortMulticastRcvPkts:1;
			uint16 PortMulticastXmitPkts:1;
			uint16 PortUnicastRcvPkts:1;
			uint16 PortUnicastXmitPkts:1;
			uint16 PortRcvPkts:1;
			uint16 PortXmitPkts:1;
			uint16 PortRcvData:1;
			uint16 PortXmitData:1;
#else			/* CPU_LE */
			uint16 PortXmitData:1;
			uint16 PortRcvData:1;
			uint16 PortXmitPkts:1;
			uint16 PortRcvPkts:1;
			uint16 PortUnicastXmitPkts:1;
			uint16 PortUnicastRcvPkts:1;
			uint16 PortMulticastXmitPkts:1;
			uint16 PortMulticastRcvPkts:1;
			uint16 Reserved:8;
#endif
		} c;
	} CounterSelect;				/* which counters to overwrite for Set */
	uint32	Reserved2;
	uint64 PortXmitData;
	uint64 PortRcvData;
	uint64 PortXmitPkts;
	uint64 PortRcvPkts;
	uint64 PortUnicastXmitPkts;
	uint64 PortUnicastRcvPkts;
	uint64 PortMulticastXmitPkts;
	uint64 PortMulticastRcvPkts;
} PACK_SUFFIX PortCountersExtended, PORT_COUNTERS_EXTENDED;

// Bits for CC field in Attribute Modifier
// These indicated which counters should be cleared in a Set
// For devices with VendorPortCounters2 capability, the Set can
// contain thresholds for each counter and only those whose
// CC field is selected and have a counter value > the threshold
// will be cleared.  A threshold of 0 will clear any non-zero counter
// A threshold of 0xffff for a 16 bit counter will never clear it.
// Values inbetween only clear when counter is > threshold.
// In all cases, the GetResp reports the counter value BEFORE any clear.
#define PM_VENDOR_PORT_COUNTERS_ALL_ERROR_COUNTERS	0x01000000U
#define PM_VENDOR_PORT_COUNTERS_ALL_DATA_COUNTERS	0x02000000U
#define PM_VENDOR_PORT_COUNTERS_TOTAL_CONGESTION	0x04000000U
#define PM_VENDOR_PORT_COUNTERS_ADAPTIVE_ROUTING	0x08000000U
#if 0
#define PM_VENDOR_PORT_COUNTERS_VL_DATA_COUNTERS	0x10000000U
#define PM_VENDOR_PORT_COUNTERS_VL_CONGESTION		0x20000000U
#define PM_VENDOR_PORT_COUNTERS_ALL					0x3f000000U
#else
#define PM_VENDOR_PORT_COUNTERS_ALL					0x0f000000U
#endif
// upper 2 bits reserved
// lower 8 bits are port selector

// PortCheckRate upper 3 bits indicate the nature of the value
#define PM_VENDOR_PORT_COUNTERS_RATE_VALUE_MASK		0x1fff
#define PM_VENDOR_PORT_COUNTERS_RATE_TYPE_MASK		0xe000
#define PM_VENDOR_PORT_COUNTERS_RATE_TYPE_CONG		0x0000	// Cong checks/sec
#define PM_VENDOR_PORT_COUNTERS_RATE_TYPE_PICO_TICK	0xe000	// XmitWait Tick ps

// Unlike other PMA packets this starts at byte 24 immediately after
// the MAD_COMMON header
typedef struct __VendorPortCounters {
	uint16		Reserved1;
	uint16		PortCheckRate;
	uint16		SymbolErrorCounter;
	uint8		LinkErrorRecoveryCounter;
	uint8		LinkDownedCounter;
	uint16		PortRcvErrors;
	uint16		PortRcvRemotePhysicalErrors;
	uint16		PortRcvSwitchRelayErrors;
	uint16		PortXmitDiscards;
	uint8		PortXmitConstraintErrors;
	uint8		PortRcvConstraintErrors;
	uint8		Reserved2;
	IB_BITFIELD2( uint8,
			LocalLinkIntegrityErrors:4,
			ExcessiveBufferOverrunErrors:4)
	uint16		Reserved3;
	uint16		VL15Dropped;
	uint64		PortXmitData;
	uint64		PortRcvData;
	uint64		PortXmitPkts;
	uint64		PortRcvPkts;
	uint64		PortXmitCongestion;	// TrueScale SW, Num of Xmit congestion events
	uint64		PortAdaptiveRouting;// TrueScale SW adaptive routing events
#if 0
	struct {
		uint32	PortRcvDataMB;	// VL RcvData in units of MBytes
		uint32	PortRcvKPkts;	// VL RcvPkts in units of KPackets
		uint32	PortXmitDataMB;	// VL XmitData in units of MBytes
		uint32	PortXmitKPkts;	// VL XmitPkts in units of KPackets
	} PACK_SUFFIX VL[8];
	uint32		VLPortXmitCongestion[8];	// per VL
#endif
} PACK_SUFFIX VendorPortCounters, VENDOR_PORT_COUNTERS;


/* End of packed data structures */
#include "iba/public/ipackoff.h"


/* Byte Swap macros - convert between host and network byte order */

static __inline void
BSWAP_PM_PORT_SAMPLES_CONTROL (PORT_SAMPLES_CONTROL *pPortSamplesControl)
{
#if CPU_LE
	int i;

	pPortSamplesControl->u1.AsUint32 = ntoh32(pPortSamplesControl->u1.AsUint32);
	pPortSamplesControl->u2.AsUint16 = ntoh16(pPortSamplesControl->u2.AsUint16);
	pPortSamplesControl->OptionMask.AsUint64 = ntoh64(pPortSamplesControl->OptionMask.AsUint64);
	pPortSamplesControl->VendorMask = ntoh64(pPortSamplesControl->VendorMask);
	pPortSamplesControl->SampleStart = ntoh32(pPortSamplesControl->SampleStart);
	pPortSamplesControl->SampleInterval = ntoh32(pPortSamplesControl->SampleInterval);
	pPortSamplesControl->Tag = ntoh16(pPortSamplesControl->Tag);
	for (i=0; i<15; ++i)
	{
		pPortSamplesControl->CounterSelect[i] = ntoh16(pPortSamplesControl->CounterSelect[i]);
	}
	pPortSamplesControl->SamplesOnlyOptionMask.AsUint64 = ntoh64(pPortSamplesControl->SamplesOnlyOptionMask.AsUint64);
#endif
}

static __inline void
BSWAP_PM_PORT_SAMPLES_RESULT (PORT_SAMPLES_RESULT *pPortSamplesResult)
{
#if CPU_LE
	int i;

	pPortSamplesResult->Tag = ntoh16(pPortSamplesResult->Tag);
	for (i=0; i<15; ++i)
	{
		pPortSamplesResult->Counter[i] = ntoh32(pPortSamplesResult->Counter[i]);
	}
#endif
}

static __inline void
BSWAP_PM_PORT_COUNTERS (PORT_COUNTERS *pPortCounters)
{
#if CPU_LE
	pPortCounters->CounterSelect.AsUint16 = ntoh16(pPortCounters->CounterSelect.AsUint16);
	pPortCounters->SymbolErrorCounter = ntoh16(pPortCounters->SymbolErrorCounter);
	pPortCounters->PortRcvErrors = ntoh16(pPortCounters->PortRcvErrors);
	pPortCounters->PortRcvRemotePhysicalErrors = ntoh16(pPortCounters->PortRcvRemotePhysicalErrors);
	pPortCounters->PortRcvSwitchRelayErrors = ntoh16(pPortCounters->PortRcvSwitchRelayErrors);
	pPortCounters->PortXmitDiscards = ntoh16(pPortCounters->PortXmitDiscards);
	pPortCounters->VL15Dropped = ntoh16(pPortCounters->VL15Dropped);
	pPortCounters->PortXmitData = ntoh32(pPortCounters->PortXmitData);
	pPortCounters->PortRcvData = ntoh32(pPortCounters->PortRcvData);
	pPortCounters->PortXmitPkts = ntoh32(pPortCounters->PortXmitPkts);
	pPortCounters->PortRcvPkts = ntoh32(pPortCounters->PortRcvPkts);
	pPortCounters->PortXmitWait = ntoh32(pPortCounters->PortXmitWait);
#endif
}

static __inline void
BSWAP_PM_PORT_RCV_ERROR_DETAILS (PORT_RCV_ERROR_DETAILS *pPortRcvErrorDetails)
{
#if CPU_LE
	pPortRcvErrorDetails->CounterSelect.AsUint16 = ntoh16(pPortRcvErrorDetails->CounterSelect.AsUint16);
	pPortRcvErrorDetails->PortLocalPhysicalErrors = ntoh16(pPortRcvErrorDetails->PortLocalPhysicalErrors);
	pPortRcvErrorDetails->PortMalformedPacketErrors = ntoh16(pPortRcvErrorDetails->PortMalformedPacketErrors);
	pPortRcvErrorDetails->PortBufferOverrunErrors = ntoh16(pPortRcvErrorDetails->PortBufferOverrunErrors);
	pPortRcvErrorDetails->PortDLIDMappingErrors = ntoh16(pPortRcvErrorDetails->PortDLIDMappingErrors);
	pPortRcvErrorDetails->PortVLMappingErrors = ntoh16(pPortRcvErrorDetails->PortVLMappingErrors);
	pPortRcvErrorDetails->PortLoopingErrors = ntoh16(pPortRcvErrorDetails->PortLoopingErrors);
#endif
}

static __inline void
BSWAP_PM_PORT_XMIT_DISCARD_DETAILS(PORT_XMIT_DISCARD_DETAILS *pPortXmitDiscardDetails)
{
#if CPU_LE
	pPortXmitDiscardDetails->CounterSelect.AsUint16 = ntoh16(pPortXmitDiscardDetails->CounterSelect.AsUint16);
	pPortXmitDiscardDetails->PortInactiveDiscards = ntoh16(pPortXmitDiscardDetails->PortInactiveDiscards);
	pPortXmitDiscardDetails->PortNeighborMTUDiscards = ntoh16(pPortXmitDiscardDetails->PortNeighborMTUDiscards);
	pPortXmitDiscardDetails->PortSwLifetimeLimitDiscards = ntoh16(pPortXmitDiscardDetails->PortSwLifetimeLimitDiscards);
	pPortXmitDiscardDetails->PortSwHOQLifetimeLimitDiscards = ntoh16(pPortXmitDiscardDetails->PortSwHOQLifetimeLimitDiscards);
#endif
}

static __inline void
BSWAP_PM_PORT_OP_RCV_COUNTERS(PORT_OP_RCV_COUNTERS *pPortOpRcvCounters)
{
#if CPU_LE
	pPortOpRcvCounters->CounterSelect.AsUint16 = ntoh16(pPortOpRcvCounters->CounterSelect.AsUint16);
	pPortOpRcvCounters->PortOpRcvPkts = ntoh32(pPortOpRcvCounters->PortOpRcvPkts);
	pPortOpRcvCounters->PortOpRcvData = ntoh32(pPortOpRcvCounters->PortOpRcvData);
#endif
}

static __inline void
BSWAP_PM_PORT_FLOW_CTL_COUNTERS(PORT_FLOW_CTL_COUNTERS *pPortFlowCtlCounters)
{
#if CPU_LE
	pPortFlowCtlCounters->CounterSelect.AsUint16 = ntoh16(pPortFlowCtlCounters->CounterSelect.AsUint16);
	pPortFlowCtlCounters->PortXmitFlowPkts = ntoh32(pPortFlowCtlCounters->PortXmitFlowPkts);
	pPortFlowCtlCounters->PortRcvFlowPkts = ntoh32(pPortFlowCtlCounters->PortRcvFlowPkts);
#endif
}

static __inline void
BSWAP_PM_PORT_VL_COUNTERS16 (struct __PortVLCounters16 *pPortVLCounters)
{
#if CPU_LE
	int i;

	pPortVLCounters->CounterSelect.AsUint16 = ntoh16(pPortVLCounters->CounterSelect.AsUint16);
	for (i=0; i<15; ++i) {
		pPortVLCounters->PortVLCounter[i]  = ntoh16(pPortVLCounters->PortVLCounter[i]);
	}
#endif
}

static __inline void
BSWAP_PM_PORT_VL_OP_PACKETS (PORT_VL_OP_PACKETS *pPortVLCounters)
{
	BSWAP_PM_PORT_VL_COUNTERS16 (pPortVLCounters);
}

static __inline void
BSWAP_PM_PORT_VL_XMIT_WAIT_COUNTERS (PORT_VL_XMIT_WAIT_COUNTERS *pPortVLCounters)
{
	BSWAP_PM_PORT_VL_COUNTERS16 (pPortVLCounters);
}

static __inline void
BSWAP_PM_SW_PORT_VL_CONGESTION (SW_PORT_VL_CONGESTION *pPortVLCounters)
{
	BSWAP_PM_PORT_VL_COUNTERS16 (pPortVLCounters);
}

static __inline void
BSWAP_PM_PORT_VL_COUNTERS32 (struct __PortVLCounters32 *pPortVLCounters)
{
#if CPU_LE
	int i;

	pPortVLCounters->CounterSelect.AsUint16 = ntoh16(pPortVLCounters->CounterSelect.AsUint16);
	for (i=0; i<15; ++i) {
		pPortVLCounters->PortVLCounter[i]  = ntoh32(pPortVLCounters->PortVLCounter[i]);
	}
#endif
}

static __inline void
BSWAP_PM_PORT_VL_OP_DATA (PORT_VL_OP_DATA *pPortVLCounters)
{
	BSWAP_PM_PORT_VL_COUNTERS32 (pPortVLCounters);
}

static __inline void
BSWAP_PM_PORT_VL_COUNTERS2 (struct __PortVLCounters2 *pPortVLCounters)
{
#if CPU_LE
	pPortVLCounters->CounterSelect.AsUint16 = ntoh16(pPortVLCounters->CounterSelect.AsUint16);
#endif
}

static __inline void
BSWAP_PM_PORT_VL_XMIT_FLOW_CTL_UPDATE_ERRORS (PORT_VL_XMIT_FLOW_CTL_UPDATE_ERRORS *pPortVLCounters)
{
	BSWAP_PM_PORT_VL_COUNTERS2 (pPortVLCounters);
}

static __inline void
BSWAP_PM_PORT_SAMPLES_RESULT_EXTENDED (PORT_SAMPLES_RESULT_EXTENDED *pPortSamplesResult)
{
#if CPU_LE
	int i;

	pPortSamplesResult->Tag = ntoh16(pPortSamplesResult->Tag);
	for (i=0; i<15; ++i)
	{
		pPortSamplesResult->Counter[i] = ntoh64(pPortSamplesResult->Counter[i]);
	}
#endif
}

static __inline void
BSWAP_PM_PORT_COUNTERS_EXTENDED (PORT_COUNTERS_EXTENDED *pPortCounters)
{
#if CPU_LE
	pPortCounters->CounterSelect.AsUint16 = ntoh16(pPortCounters->CounterSelect.AsUint16);
	pPortCounters->PortXmitData = ntoh64(pPortCounters->PortXmitData);
	pPortCounters->PortRcvData = ntoh64(pPortCounters->PortRcvData);
	pPortCounters->PortXmitPkts = ntoh64(pPortCounters->PortXmitPkts);
	pPortCounters->PortRcvPkts = ntoh64(pPortCounters->PortRcvPkts);
	pPortCounters->PortUnicastXmitPkts = ntoh64(pPortCounters->PortUnicastXmitPkts);
	pPortCounters->PortUnicastRcvPkts = ntoh64(pPortCounters->PortUnicastRcvPkts);
	pPortCounters->PortMulticastXmitPkts = ntoh64(pPortCounters->PortMulticastXmitPkts);
	pPortCounters->PortMulticastRcvPkts = ntoh64(pPortCounters->PortMulticastRcvPkts);
#endif
}

static __inline void
BSWAP_PM_VENDOR_PORT_COUNTERS (VENDOR_PORT_COUNTERS *pPortCounters)
{
#if CPU_LE
	//int i;

	pPortCounters->PortCheckRate = ntoh16(pPortCounters->PortCheckRate);
	pPortCounters->SymbolErrorCounter = ntoh16(pPortCounters->SymbolErrorCounter);
	pPortCounters->PortRcvErrors = ntoh16(pPortCounters->PortRcvErrors);
	pPortCounters->PortRcvRemotePhysicalErrors = ntoh16(pPortCounters->PortRcvRemotePhysicalErrors);
	pPortCounters->PortRcvSwitchRelayErrors = ntoh16(pPortCounters->PortRcvSwitchRelayErrors);
	pPortCounters->PortXmitDiscards = ntoh16(pPortCounters->PortXmitDiscards);
	pPortCounters->VL15Dropped = ntoh16(pPortCounters->VL15Dropped);
	pPortCounters->PortXmitData = ntoh64(pPortCounters->PortXmitData);
	pPortCounters->PortRcvData = ntoh64(pPortCounters->PortRcvData);
	pPortCounters->PortXmitPkts = ntoh64(pPortCounters->PortXmitPkts);
	pPortCounters->PortRcvPkts = ntoh64(pPortCounters->PortRcvPkts);
	pPortCounters->PortXmitCongestion = ntoh64(pPortCounters->PortXmitCongestion);
	pPortCounters->PortAdaptiveRouting = ntoh64(pPortCounters->PortAdaptiveRouting);
#if 0
	for (i=0; i<8; ++i) {
		pPortCounters->VL[i].PortRcvDataMB = ntoh32(pPortCounters->VL[i].PortRcvDataMB);
		pPortCounters->VL[i].PortRcvKPkts = ntoh32(pPortCounters->VL[i].PortRcvKPkts);
		pPortCounters->VL[i].PortXmitDataMB = ntoh32(pPortCounters->VL[i].PortXmitDataMB);
		pPortCounters->VL[i].PortXmitKPkts = ntoh32(pPortCounters->VL[i].PortXmitKPkts);
		pPortCounters->VLPortXmitCongestion[i] = ntoh32(pPortCounters->VLPortXmitCongestion[i]);
	}
#endif
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_PM_H_ */
