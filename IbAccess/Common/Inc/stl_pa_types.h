/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

#ifndef _STL_PA_TYPES_H_
#define _STL_PA_TYPES_H_ (1) /* suppress duplicate loading of this file */

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"
#include "iba/stl_mad_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iba/public/ipackon.h"

// XML Defines
#define STL_PM_MAX_DG_PER_PMPG	5		//Maximum number of Monitors allowed in a PmPortGroup

/** PA MAD Payloads */
#define STL_PM_GROUPNAMELEN		64
#define STL_PM_NODEDESCLEN		64
#define STL_PM_MAX_GROUPS		10
#define STL_PM_VFNAMELEN		64
#define STL_PM_MAX_VFS			32

#define STL_PM_UTIL_GRAN_PERCENT 10 /* granularity of utilization buckets */
#define STL_PM_UTIL_BUCKETS (100 / STL_PM_UTIL_GRAN_PERCENT)

#define STL_PM_CAT_GRAN_PERCENT 25  /* granularity of error buckets */
#define STL_PM_CATEGORY_BUCKETS ((100 / STL_PM_CAT_GRAN_PERCENT) + 1) // extra bucket is for those over threshold

/* ClassPortInfo Capability bits */

typedef STL_FIELDUNION8(STL_PA_CLASS_PORT_INFO_CAPABILITY_MASK, 16,
		Reserved1:                  2,      /* start of class dependent bits */
		IsPerImageListsSupported:   1,      /* query to get group/vf list from specific images */
		IsVFFocusTypesSupported:    1,      /* VF level Focus types */
		IsTopologyInfoSupported:    1,      /* query to get the topology info from PM */
		                                    /* (GetGroupNodeInfo and GetGroupLinkInfo) */
		IsExtFocusTypesSupported:   1,      /* RO - PA supports extended focus types */
		                                    /* (unexpclrport, norespport and */
		                                    /* skippedport) in GetFocusPorts and */
		                                    /* adds GetFocusPortsMultiSelect */
		Reserved3:                  1,
		IsAbsTimeQuerySupported:    1,      /* RO - PA supports AbsImageTime queries w/ absoluteTime in Image ID */
		Reserved2:                  8);     /* class independent bits*/

/*
 * PA capability mask defines
 */
#define STL_PA_CPI_CAPMASK_ABSTIMEQUERY   0x0100
#define STL_PA_CPI_CAPMASK_EXT_FOCUSTYPES 0x0400
#define STL_PA_CPI_CAPMASK_TOPO_INFO      0x0800
#define STL_PA_CPI_CAPMASK_VF_FOCUSTYPES  0x1000
#define STL_PA_CPI_CAPMASK_IMAGE_LISTS    0x2000

static __inline void
StlPaClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		snprintf(buf, 80, "%s%s%s%s%s%s%s%s",
			(cmask & STL_CLASS_PORT_CAPMASK_TRAP) ? "Trap " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_NOTICE) ? "Notice " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_CM2) ? "CapMask2 " : "",
			/* Class Specific */
			(cmask & STL_PA_CPI_CAPMASK_ABSTIMEQUERY) ? "AbsTime " : "",
			(cmask & STL_PA_CPI_CAPMASK_EXT_FOCUSTYPES) ? "ExtFocusTypes " : "",
			(cmask & STL_PA_CPI_CAPMASK_TOPO_INFO) ? "TopologyInfo " : "",
			(cmask & STL_PA_CPI_CAPMASK_VF_FOCUSTYPES) ? "VFFocusTypes " : "",
			(cmask & STL_PA_CPI_CAPMASK_IMAGE_LISTS) ? "ImageLists " : "");
	}
}

static __inline void
StlPaClassPortInfoCapMask2(char buf[80], uint32 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		buf[0] = '\0';
	}
}

// Check to see if the VF Focus select is an UTIL BW category
#define IS_FOCUS_SELECT_UTIL(SELECT) \
	((SELECT == STL_PA_SELECT_UTIL_HIGH) || \
	(SELECT == STL_PA_SELECT_UTIL_LOW) || \
	(SELECT == STL_PA_SELECT_VF_UTIL_HIGH) || \
	(SELECT == STL_PA_SELECT_VF_UTIL_LOW))

// Check to see if it is a VF Focus select
#define IS_VF_FOCUS_SELECT(SELECT) \
	((SELECT == STL_PA_SELECT_VF_UTIL_HIGH) || \
	(SELECT == STL_PA_SELECT_VF_UTIL_LOW) || \
	(SELECT == STL_PA_SELECT_VF_UTIL_PKTS_HIGH) || \
	(SELECT == STL_PA_SELECT_CATEGORY_VF_BUBBLE)  || \
	(SELECT == STL_PA_SELECT_CATEGORY_VF_CONG))

typedef struct _STL_PA_Group_List {
	char					groupName[STL_PM_GROUPNAMELEN];	// \0 terminated - actual number indicated by numGroups
} PACK_SUFFIX STL_PA_GROUP_LIST;

#define PACLIENT_IMAGE_CURRENT  0   // imageNumber of most recent sweep image
#define PACLIENT_IMAGE_TIMED   -1   // imageNumber of Image with particular time

typedef struct _STL_PA_Image_ID_Data {
	uint64					imageNumber;
	int32					imageOffset;
	union {
		uint32				absoluteTime;
		int32				timeOffset;
	} PACK_SUFFIX imageTime;
} PACK_SUFFIX STL_PA_IMAGE_ID_DATA;

/* Utilization statistical summary */
typedef struct _STL_PA_PM_Util_Stats {
	uint64					totalMBps;	/* MB per sec */
	uint64					totalKPps;	/* K pkts per sec */
	uint32					avgMBps;
	uint32					minMBps;
	uint32					maxMBps;
	uint32					numBWBuckets;
	uint32					BWBuckets[STL_PM_UTIL_BUCKETS];
	uint32					avgKPps;
	uint32					minKPps;
	uint32					maxKPps;
	uint16					pmaNoRespPorts;
	uint16					topoIncompPorts;
} PACK_SUFFIX STL_PA_PM_UTIL_STATS;

/* Error statistical summary */
typedef struct _STL_PA_PM_CATEGORY_SUMMARY {
	uint32					integrityErrors;
	uint32					congestion;
	uint32					smaCongestion;
	uint32					bubble;
	uint32					securityErrors;
	uint32					routingErrors;

	uint16					utilizationPct10; /* in units of 10% */
	uint16					discardsPct10;    /* in units of 10% */
	uint16					reserved[6];
} PACK_SUFFIX STL_PA_PM_CATEGORY_SUMMARY;

typedef struct _STL_PM_CATEGORY_THRESHOLD {
    uint32					integrityErrors;
	uint32					congestion;
	uint32					smaCongestion;
	uint32					bubble;
	uint32					securityErrors;
	uint32					routingErrors;
} PACK_SUFFIX STL_PM_CATEGORY_THRESHOLD;

typedef struct _STL_PM_CATEGORY_BUCKET {
	uint32					integrityErrors;
	uint32					congestion;
	uint32					smaCongestion;
	uint32					bubble;
	uint32					securityErrors;
	uint32					routingErrors;
} PACK_SUFFIX STL_PM_CATEGORY_BUCKET;

typedef struct _STL_PM_CATEGORY_STATS {
	STL_PA_PM_CATEGORY_SUMMARY		categoryMaximums;
	STL_PM_CATEGORY_BUCKET			ports[STL_PM_CATEGORY_BUCKETS];
} PACK_SUFFIX STL_PM_CATEGORY_STATS;

typedef struct _STL_PA_Group_Info_Data {
	char					groupName[STL_PM_GROUPNAMELEN]; // \0 terminated.
	STL_PA_IMAGE_ID_DATA	imageId;
	uint32					numInternalPorts;
	uint32					numExternalPorts;
	STL_PA_PM_UTIL_STATS	internalUtilStats;
	STL_PA_PM_UTIL_STATS	sendUtilStats;
	STL_PA_PM_UTIL_STATS	recvUtilStats;
	STL_PM_CATEGORY_STATS			internalCategoryStats;
	STL_PM_CATEGORY_STATS			externalCategoryStats;
	uint8					maxInternalRate;
	uint8					minInternalRate;
	uint8					maxExternalRate;
	uint8					minExternalRate;
	uint32					maxInternalMBps;
	uint32					maxExternalMBps;
} PACK_SUFFIX STL_PA_PM_GROUP_INFO_DATA;

typedef struct _STL_PA_Group_Cfg_Req {
	char					groupName[STL_PM_GROUPNAMELEN]; // \0 terminated.
	STL_PA_IMAGE_ID_DATA	imageId;
} PACK_SUFFIX STL_PA_PM_GROUP_CFG_REQ;

typedef struct _STL_PA_Group_Cfg_Rsp {
	STL_PA_IMAGE_ID_DATA	imageId;
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	STL_LID					nodeLid;
	uint8					portNumber;
	uint8					reserved[3];
} PACK_SUFFIX STL_PA_PM_GROUP_CFG_RSP;


// STL_PORT_COUNTERS_DATA.flags
#define STL_PA_PC_FLAG_DELTA            0x00000001 // are these DELTA(1) or running totals
#define STL_PA_PC_FLAG_UNEXPECTED_CLEAR 0x00000002 // was there an unexpected clear
#define STL_PA_PC_FLAG_SHARED_VL        0x00000004 // for vf port counters, vl is shared >1 vf
#define STL_PA_PC_FLAG_USER_COUNTERS    0x00000008 // for PA user controlled running counters
#define STL_PA_PC_FLAG_CLEAR_FAIL       0x00000010 // indicates clear query failed however, data should be valid


typedef struct _STL_PORT_COUNTERS_DATA {
	STL_LID				nodeLid;
	uint8				portNumber;
	uint8				reserved[3];
	uint32				flags;
	uint32				reserved1;
	uint64				reserved3;
	STL_PA_IMAGE_ID_DATA imageId;
	uint64				portXmitData;
	uint64				portRcvData;
	uint64				portXmitPkts;
	uint64				portRcvPkts;
	uint64				portMulticastXmitPkts;
	uint64				portMulticastRcvPkts;
	uint64				localLinkIntegrityErrors;
	uint64				fmConfigErrors;
	uint64				portRcvErrors;
	uint64				excessiveBufferOverruns;
	uint64				portRcvConstraintErrors;
	uint64				portRcvSwitchRelayErrors;
	uint64				portXmitDiscards;
	uint64				portXmitConstraintErrors;
	uint64				portRcvRemotePhysicalErrors;
	uint64				swPortCongestion;
	uint64				portXmitWait;
	uint64				portRcvFECN;
	uint64				portRcvBECN;
	uint64				portXmitTimeCong;
	uint64				portXmitWastedBW;
	uint64				portXmitWaitData;
	uint64				portRcvBubble;
	uint64				portMarkFECN;
	uint32				linkErrorRecovery;
	uint32				linkDowned;
	uint8				uncorrectableErrors;
	union {
		uint8			AsReg8;
		struct {		IB_BITFIELD3(uint8,
						numLanesDown : 4,
						reserved : 1,
						linkQualityIndicator : 3)
		} PACK_SUFFIX s;
	} lq;
	uint8				reserved2[6];
} PACK_SUFFIX STL_PORT_COUNTERS_DATA;

typedef union {
	uint32	AsReg32;

	struct stl_counter_select_mask { IB_BITFIELD28(uint32,
		PortXmitData : 1,
		PortRcvData : 1,
		PortXmitPkts : 1,
		PortRcvPkts : 1,
		PortMulticastXmitPkts : 1,
		PortMulticastRcvPkts : 1,
		PortXmitWait : 1,
		SwPortCongestion : 1,
		PortRcvFECN : 1,
		PortRcvBECN : 1,
		PortXmitTimeCong : 1,
		PortXmitWastedBW : 1,
		PortXmitWaitData : 1,
		PortRcvBubble : 1,
		PortMarkFECN : 1,
		PortRcvConstraintErrors : 1,
		PortRcvSwitchRelayErrors : 1,
		PortXmitDiscards : 1,
		PortXmitConstraintErrors : 1,
		PortRcvRemotePhysicalErrors : 1,
		LocalLinkIntegrityErrors : 1,
		PortRcvErrors : 1,
		ExcessiveBufferOverruns : 1,
		FMConfigErrors : 1,
		LinkErrorRecovery : 1,
		LinkDowned : 1,
		UncorrectableErrors : 1,
		Reserved : 5)
	} PACK_SUFFIX s;

} CounterSelectMask_t;

typedef struct _STL_CLR_PORT_COUNTERS_DATA {
	STL_LID		NodeLid;
	uint8		PortNumber;
	uint8		Reserved[3];
	uint64		Reserved2;
	CounterSelectMask_t CounterSelectMask;
} PACK_SUFFIX STL_CLR_PORT_COUNTERS_DATA;

typedef struct _STL_CLR_ALL_PORT_COUNTERS_DATA {
	CounterSelectMask_t CounterSelectMask;
} PACK_SUFFIX STL_CLR_ALL_PORT_COUNTERS_DATA;

typedef struct _STL_INTEGRITY_WEIGHTS {
	uint8					LocalLinkIntegrityErrors;
	uint8					PortRcvErrors;
	uint8					ExcessiveBufferOverruns;
	uint8					LinkErrorRecovery;
	uint8					LinkDowned;
	uint8					UncorrectableErrors;
	uint8					FMConfigErrors;
	uint8					LinkQualityIndicator;
	uint8					LinkWidthDowngrade;
	uint8					reserved[7];
} PACK_SUFFIX STL_INTEGRITY_WEIGHTS_T;

typedef struct _STL_PM_CATEGORY_THRESHOLDS {
	uint32					integrityErrors;
	uint32					congestion;
	uint32					smaCongestion;
	uint32					bubble;
	uint32					securityErrors;
	uint32					routingErrors;
} PACK_SUFFIX STL_PM_CATEGORY_THRESHOLDS;

typedef struct _STL_CONGESTION_WEIGHTS {
	uint8					PortXmitWait;
	uint8					SwPortCongestion;
	uint8					PortRcvFECN;
	uint8					PortRcvBECN;
	uint8					PortXmitTimeCong;
	uint8					PortMarkFECN;
	uint16					reserved;
} PACK_SUFFIX STL_CONGESTION_WEIGHTS_T;

/* 
 * PM configuration flags.
 */
#define STL_PM_NONE							0x0
#define STL_PM_PROCESS_VL_COUNTERS			0x00000001	// Enable Processing of VL Counters
#define STL_PM_PROCESS_HFI_COUNTERS			0x00000002	// Enable Processing of HFI Counters
#define STL_PM_PROCESS_CLR_DATA_COUNTERS	0x00000004	// Enable Clearing of Data Counters
#define STL_PM_PROCESS_CLR_64BIT_COUNTERS	0x00000008	// Enable Clearing of 64 bit Error Counters
#define STL_PM_PROCESS_CLR_32BIT_COUNTERS	0x00000010	// Enable Clearing of 32 bit Error Counters
#define STL_PM_PROCESS_CLR_8BIT_COUNTERS	0x00000020	// Enable Clearing of 8 bit Error Counters

static __inline
void StlFormatPmFlags(char buf[80], uint32 pmFlags)
{
	snprintf(buf, 80, "%s=%s %s=%s %s=%s %s=%s",
			"ProcessHFICntrs", pmFlags & STL_PM_PROCESS_HFI_COUNTERS ? "On" : "Off",
			"ProcessVLCntrs",  pmFlags & STL_PM_PROCESS_VL_COUNTERS ? "On" : "Off",
			"ClrDataCntrs",    pmFlags & STL_PM_PROCESS_CLR_DATA_COUNTERS ? "On" : "Off",
			"Clr64bitErrCntrs", pmFlags & STL_PM_PROCESS_CLR_64BIT_COUNTERS ? "On" : "Off");
}

static __inline
void StlFormatPmFlags2(char buf[80], uint32 pmFlags)
{
	snprintf(buf, 80, "%s=%s %s=%s",
			"Clr32bitErrCntrs", pmFlags & STL_PM_PROCESS_CLR_32BIT_COUNTERS ? "On" : "Off",
			"Clr8bitErrCntrs",  pmFlags & STL_PM_PROCESS_CLR_8BIT_COUNTERS ? "On" : "Off");
}

typedef struct _STL_PA_PM_Cfg_Data {
	uint32					sweepInterval;
	uint32					maxClients;
	uint32					sizeHistory;
	uint32					sizeFreeze;
	uint32					lease;
	uint32					pmFlags;
	STL_CONGESTION_WEIGHTS_T congestionWeights;
	STL_PM_CATEGORY_THRESHOLDS	categoryThresholds;
	STL_INTEGRITY_WEIGHTS_T	integrityWeights;
	uint64					memoryFootprint;
	uint32					maxAttempts;
	uint32					respTimeout;
	uint32					minRespTimeout;
	uint32					maxParallelNodes;
	uint32					pmaBatchSize;
	uint8					errorClear;
	uint8					reserved[3];
} PACK_SUFFIX STL_PA_PM_CFG_DATA;

typedef struct _STL_MOVE_FREEZE_DATA {
	STL_PA_IMAGE_ID_DATA	oldFreezeImage;
	STL_PA_IMAGE_ID_DATA	newFreezeImage;
} PACK_SUFFIX STL_MOVE_FREEZE_DATA;

#define STL_PA_SELECT_UNEXP_CLR_PORT	0x00010101
#define STL_PA_SELECT_NO_RESP_PORT		0x00010102
#define STL_PA_SELECT_SKIPPED_PORT		0x00010103
#define STL_PA_SELECT_UTIL_HIGH			0x00020001 // highest first, descending
#define STL_PA_SELECT_UTIL_MC_HIGH		0x00020081 // not supported
#define STL_PA_SELECT_UTIL_PKTS_HIGH	0x00020082
#define STL_PA_SELECT_VF_UTIL_HIGH	0x00020083 // Only used with VFs
#define STL_PA_SELECT_VF_UTIL_PKTS_HIGH	0x00020084 // Only used with VFs
#define STL_PA_SELECT_UTIL_LOW			0x00020101 // lowest first, ascending
#define STL_PA_SELECT_UTIL_MC_LOW		0x00020102 // not supported
#define STL_PA_SELECT_VF_UTIL_LOW	0x00020103   // Only used with VFs
#define STL_PA_SELECT_CATEGORY_INTEG	0x00030001 // hightest first, descending
#define STL_PA_SELECT_CATEGORY_CONG		0x00030002
#define STL_PA_SELECT_CATEGORY_SMA_CONG	0x00030003
#define STL_PA_SELECT_CATEGORY_BUBBLE	0x00030004
#define STL_PA_SELECT_CATEGORY_SEC		0x00030005
#define STL_PA_SELECT_CATEGORY_ROUT		0x00030006
#define STL_PA_SELECT_CATEGORY_VF_CONG		0x00030007  // Only used with VFs
#define STL_PA_SELECT_CATEGORY_VF_BUBBLE	0x00030008  // Only used with VFs

typedef struct _STL_FOCUS_PORTS_REQ {
	char					groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	STL_PA_IMAGE_ID_DATA	imageId;
	uint32					select;
	uint32					start;
	uint32					range;
} PACK_SUFFIX STL_FOCUS_PORTS_REQ;

typedef struct _STL_FOCUS_PORT_TUPLE {
	uint32 	select;
	uint8 	comparator;
	uint8 	reserved[3];
	uint64 	argument;
} PACK_SUFFIX STL_FOCUS_PORT_TUPLE;

#define MAX_NUM_FOCUS_PORT_TUPLES		8

#define FOCUS_PORTS_LOGICAL_OPERATOR_INVALID 0
#define FOCUS_PORTS_LOGICAL_OPERATOR_AND     1
#define FOCUS_PORTS_LOGICAL_OPERATOR_OR      2

#define FOCUS_PORTS_COMPARATOR_INVALID               0
#define FOCUS_PORTS_COMPARATOR_GREATER_THAN          1
#define FOCUS_PORTS_COMPARATOR_LESS_THAN             2
#define FOCUS_PORTS_COMPARATOR_GREATER_THAN_OR_EQUAL 3
#define FOCUS_PORTS_COMPARATOR_LESS_THAN_OR_EQUAL    4

typedef struct _STL_FOCUS_PORTS_MULTISELECT_REQ {
	char			groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	STL_PA_IMAGE_ID_DATA	imageId;
	uint32			start;
	uint32			range;
	STL_FOCUS_PORT_TUPLE	tuple[MAX_NUM_FOCUS_PORT_TUPLES];
	uint8			logical_operator;
	uint8			reserved[7];
} PACK_SUFFIX STL_FOCUS_PORTS_MULTISELECT_REQ;

#define STL_PA_FOCUS_STATUS_OK           0
#define STL_PA_FOCUS_STATUS_PMA_IGNORE   1
#define STL_PA_FOCUS_STATUS_PMA_FAILURE  2
#define STL_PA_FOCUS_STATUS_TOPO_FAILURE 3
#define STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR 4

static __inline
const char* StlFocusStatusToText(uint8 status)
{
	switch (status) {
	case STL_PA_FOCUS_STATUS_OK:				return "OK";
	case STL_PA_FOCUS_STATUS_PMA_IGNORE:		return "PMA Ignore";
	case STL_PA_FOCUS_STATUS_PMA_FAILURE:		return "PMA Failure";
	case STL_PA_FOCUS_STATUS_TOPO_FAILURE:		return "Topo Failure";
	case STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR:	return "Unexp Clear";
	default:									return "Unknown";
	}
}

static __inline
const char* StlFocusAttributeToText(uint32 attribute)
{
	switch (attribute) {
	case STL_PA_SELECT_UNEXP_CLR_PORT:    return "Unexpected Clear";
	case STL_PA_SELECT_NO_RESP_PORT:      return "No Response Ports";
	case STL_PA_SELECT_SKIPPED_PORT:      return "Skipped Ports";
	case STL_PA_SELECT_UTIL_HIGH:         return "High Utilization";
	case STL_PA_SELECT_UTIL_PKTS_HIGH:    return "Packet Rate";
	case STL_PA_SELECT_UTIL_LOW:          return "Low Utilization";
	case STL_PA_SELECT_VF_UTIL_HIGH:      return "VF High Utilization";
	case STL_PA_SELECT_VF_UTIL_PKTS_HIGH: return "VF Packet Rate";
	case STL_PA_SELECT_VF_UTIL_LOW:       return "VF Low Utilization";
	case STL_PA_SELECT_CATEGORY_INTEG:    return "Integrity Errors";
	case STL_PA_SELECT_CATEGORY_CONG:     return "Congestion";
	case STL_PA_SELECT_CATEGORY_SMA_CONG: return "SMA Congestion";
	case STL_PA_SELECT_CATEGORY_BUBBLE:   return "Bubble";
	case STL_PA_SELECT_CATEGORY_SEC:      return "Security Errors";
	case STL_PA_SELECT_CATEGORY_ROUT:     return "Routing Errors";
	case STL_PA_SELECT_CATEGORY_VF_CONG:  return "VF Congestion";
	case STL_PA_SELECT_CATEGORY_VF_BUBBLE:return "VF Bubble";
	default:                              return "Unknown";
	}
}

static __inline
const char* StlComparatorToText(uint8 comparator)
{
	switch (comparator) {
	case FOCUS_PORTS_COMPARATOR_GREATER_THAN:          return "greater than";
	case FOCUS_PORTS_COMPARATOR_LESS_THAN:             return "less than";
	case FOCUS_PORTS_COMPARATOR_GREATER_THAN_OR_EQUAL: return "greater than or equal to";
	case FOCUS_PORTS_COMPARATOR_LESS_THAN_OR_EQUAL:    return "less than or equal to";
	case FOCUS_PORTS_COMPARATOR_INVALID:
	default:                                           return "invalid";
	}
}

static __inline
const char* StlOperatorToText(uint8 logical_operator)
{
	switch (logical_operator) {
	case FOCUS_PORTS_LOGICAL_OPERATOR_AND:     return "AND";
	case FOCUS_PORTS_LOGICAL_OPERATOR_OR:      return "OR";
	case FOCUS_PORTS_LOGICAL_OPERATOR_INVALID:
	default:                                   return "invalid";
	}
}

typedef struct _STL_FOCUS_PORTS_RSP {
	STL_PA_IMAGE_ID_DATA	imageId;
	STL_LID					nodeLid;
	uint8					portNumber;
	uint8					rate;	// IB_STATIC_RATE - 5 bit value
	uint8					maxVlMtu;	// enum IB_MTU - 4 bit value
	IB_BITFIELD2(uint8,     localStatus : 4,
		                    neighborStatus : 4)
	uint64					value;		// list sorting factor
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	STL_LID					neighborLid;
	uint8					neighborPortNumber;
	uint8					reserved3[3];
	uint64					neighborValue;
	uint64					neighborGuid;
	char					neighborNodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
} PACK_SUFFIX STL_FOCUS_PORTS_RSP;

typedef struct _STL_FOCUS_PORTS_MULTISELECT_RSP {
	STL_PA_IMAGE_ID_DATA	imageId;
	STL_LID					nodeLid;
	uint8					portNumber;
	uint8					rate;	// IB_STATIC_RATE - 5 bit value
	uint8					maxVlMtu;	// enum IB_MTU - 4 bit value
	IB_BITFIELD2(uint8,	localStatus : 4,
				neighborStatus : 4)
	uint64					value[MAX_NUM_FOCUS_PORT_TUPLES];
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	STL_LID					neighborLid;
	uint8					neighborPortNumber;
	uint8					reserved3[3];
	uint64					neighborValue[MAX_NUM_FOCUS_PORT_TUPLES];
	uint64					neighborGuid;
	char					neighborNodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
} PACK_SUFFIX STL_FOCUS_PORTS_MULTISELECT_RSP;

typedef struct _STL_SMINFO_DATA {
	STL_LID					lid;
	IB_BITFIELD2(uint8,
		priority : 4,
		state : 4)
	uint8					portNumber;
	uint16					reserved;
	uint64					smPortGuid;
	char					smNodeDesc[64]; // can be 64 char w/o \0
} PACK_SUFFIX STL_SMINFO_DATA;

typedef struct _STL_PA_IMAGE_INFO_DATA {
	STL_PA_IMAGE_ID_DATA	imageId;
	uint64					sweepStart;
	uint32					sweepDuration;
	uint16					numHFIPorts;
	uint16					reserved3;
	uint16					reserved;
	uint16					numSwitchNodes;
	uint32					numSwitchPorts;
	uint32					numLinks;
	uint32					numSMs;
	uint32					numNoRespNodes;
	uint32					numNoRespPorts;
	uint32					numSkippedNodes;
	uint32					numSkippedPorts;
	uint32					numUnexpectedClearPorts;
	uint32					imageInterval;
	STL_SMINFO_DATA			SMInfo[2];
} PACK_SUFFIX STL_PA_IMAGE_INFO_DATA;

typedef struct _STL_PA_VF_LIST {
	char					vfName[STL_PM_VFNAMELEN];	// \0 terminated
} PACK_SUFFIX STL_PA_VF_LIST;

typedef struct _STL_PA_VF_INFO_DATA {
	char					vfName[STL_PM_VFNAMELEN];	// \0 terminated
	uint64					reserved;
	STL_PA_IMAGE_ID_DATA	imageId;
	uint32					numPorts;
	STL_PA_PM_UTIL_STATS	internalUtilStats;
	STL_PM_CATEGORY_STATS			internalCategoryStats;
	// these are added at the end to allow for forward and backward
	// compatibility.
	uint8					maxInternalRate;
	uint8					minInternalRate;
	uint32					maxInternalMBps;
} PACK_SUFFIX STL_PA_VF_INFO_DATA;

typedef struct _STL_PA_VF_Cfg_Req {
	char					vfName[STL_PM_VFNAMELEN]; // \0 terminated 
	uint64					reserved;
	STL_PA_IMAGE_ID_DATA	imageId;
} PACK_SUFFIX STL_PA_VF_CFG_REQ;

typedef struct _STL_PA_VF_Cfg_Rsp {
	STL_PA_IMAGE_ID_DATA	imageId;
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	STL_LID					nodeLid;
	uint8					portNumber;
	uint8					reserved[3];
} PACK_SUFFIX STL_PA_VF_CFG_RSP;

typedef struct _STL_PA_VF_PORT_COUNTERS_DATA {
	STL_LID				nodeLid;
	uint8				portNumber;
	uint8				reserved[3];
	uint32				flags;
	uint32				reserved1;
	uint64				reserved3;
	char				vfName[STL_PM_VFNAMELEN]; // \0 terminated 
	uint64				reserved2;
	STL_PA_IMAGE_ID_DATA imageId;
	uint64				portVFXmitData;
	uint64				portVFRcvData;
	uint64				portVFXmitPkts;
	uint64				portVFRcvPkts;
	uint64				portVFXmitDiscards;
	uint64				swPortVFCongestion;
	uint64				portVFXmitWait;
	uint64				portVFRcvFECN;
	uint64				portVFRcvBECN;
	uint64				portVFXmitTimeCong;
	uint64				portVFXmitWastedBW;
	uint64				portVFXmitWaitData;
	uint64				portVFRcvBubble;
	uint64				portVFMarkFECN;
} PACK_SUFFIX STL_PA_VF_PORT_COUNTERS_DATA;

typedef union {
	uint32	AsReg32;
	struct { IB_BITFIELD15(uint32,
		PortVLXmitData : 1,
		PortVLRcvData : 1,
		PortVLXmitPkts : 1,
		PortVLRcvPkts : 1,
		PortVLXmitDiscards : 1,
		SwPortVLCongestion : 1,
		PortVLXmitWait : 1,
		PortVLRcvFECN : 1,
		PortVLRcvBECN : 1,
		PortVLXmitTimeCong : 1,
		PortVLXmitWastedBW : 1,
		PortVLXmitWaitData : 1,
		PortVLRcvBubble : 1,
		PortVLMarkFECN : 1,
		reserved : 18)
	} PACK_SUFFIX s;
} STLVlCounterSelectMask;

typedef struct _STL_PA_CLEAR_VF_PORT_COUNTERS_DATA {
	STL_LID		nodeLid;
	uint8		portNumber;
	uint8		reserved[3];
	uint64		reserved4;
	char		vfName[STL_PM_VFNAMELEN]; // \0 terminated 
	uint64		reserved3;
	STLVlCounterSelectMask vfCounterSelectMask;
	uint32	        reserved2;
} PACK_SUFFIX STL_PA_CLEAR_VF_PORT_COUNTERS_DATA;

typedef struct _STL_PA_VF_FOCUS_PORTS_REQ {
	char				vfName[STL_PM_VFNAMELEN]; // \0 terminated 
	uint64				reserved;
	STL_PA_IMAGE_ID_DATA imageId;
	uint32				select;
	uint32				start;
	uint32				range;
} PACK_SUFFIX STL_PA_VF_FOCUS_PORTS_REQ;

typedef struct _STL_PA_VF_FOCUS_PORTS_RSP {
	STL_PA_IMAGE_ID_DATA	imageId;
	STL_LID					nodeLid;
	uint8					portNumber;
	uint8					rate;	// IB_STATIC_RATE - 5 bit value
	uint8					maxVlMtu;	// enum IB_MTU - 4 bit value
	IB_BITFIELD2(uint8,     localStatus : 4,
		                    neighborStatus : 4)
	uint64					value;		// list sorting factor
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	STL_LID					neighborLid;
	uint8					neighborPortNumber;
	uint8					reserved3[3];
	uint64					neighborValue;
	uint64					neighborGuid;
	char					neighborNodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
} PACK_SUFFIX STL_PA_VF_FOCUS_PORTS_RSP;

typedef struct _STL_PA_Group_NodeInfo_Req {
	char groupName[STL_PM_GROUPNAMELEN];
	STL_PA_IMAGE_ID_DATA imageId;
	uint64 nodeGUID;
	char nodeDesc[STL_PM_NODEDESCLEN];
	uint32 nodeLID;
	uint32 reserved;
} PACK_SUFFIX STL_PA_GROUP_NODEINFO_REQ;

typedef struct _STL_PA_Group_NodeInfo_Rsp {
	STL_PA_IMAGE_ID_DATA imageId;
	uint64 nodeGUID;
	char nodeDesc[STL_PM_NODEDESCLEN];
	uint32 nodeLID;
	uint8 nodeType;
	uint8 reserved[3];
	uint64 portSelectMask[4];
} PACK_SUFFIX STL_PA_GROUP_NODEINFO_RSP;

typedef struct _STL_PA_Group_LinkInfo_Req {
	STL_PA_IMAGE_ID_DATA imageId;
	char groupName[STL_PM_GROUPNAMELEN];
	uint32 lid;
	uint8 port;
	uint8 reserved[3];
} PACK_SUFFIX STL_PA_GROUP_LINKINFO_REQ;

typedef struct _STL_PA_Group_LinkInfo_Rsp {
	STL_PA_IMAGE_ID_DATA imageId;
	uint32 fromLID;
	uint32 toLID;
	uint8 fromPort;
	uint8 toPort;
	IB_BITFIELD2( uint8,
		mtu:4,
		activeSpeed:4)
	IB_BITFIELD2( uint8,
		txLinkWidthDowngradeActive:4,
		rxLinkWidthDowngradeActive:4)
	IB_BITFIELD2(uint8,
		localStatus : 4,
		neighborStatus : 4)
	uint8 reserved[3];
} PACK_SUFFIX STL_PA_GROUP_LINKINFO_RSP;


typedef struct _STL_PA_GRP_LIST2 {
	char                 groupName[STL_PM_GROUPNAMELEN]; // \0 terminated
	STL_PA_IMAGE_ID_DATA imageId;
} PACK_SUFFIX STL_PA_GROUP_LIST2;

typedef struct _STL_PA_VF_LIST2 {
	char                 vfName[STL_PM_VFNAMELEN]; // \0 terminated
	STL_PA_IMAGE_ID_DATA imageId;
} PACK_SUFFIX STL_PA_VF_LIST2;


/* End of packed data structures */
#include "iba/public/ipackoff.h"

/* Performance Analysis methods */
#define STL_PA_CMD_GET                (0x01)
#define STL_PA_CMD_SET                (0x02)
#define STL_PA_CMD_GET_RESP           (0x81)
#define STL_PA_CMD_GETTABLE           (0x12)
#define STL_PA_CMD_GETTABLE_RESP      (0x92)

/* Performance Analysis attribute IDs */
#define STL_PA_ATTRID_GET_CLASSPORTINFO  0x01
#define STL_PA_ATTRID_GET_GRP_LIST       0xA0
#define STL_PA_ATTRID_GET_GRP_INFO       0xA1
#define STL_PA_ATTRID_GET_GRP_CFG        0xA2
#define STL_PA_ATTRID_GET_PORT_CTRS      0xA3
#define STL_PA_ATTRID_CLR_PORT_CTRS      0xA4
#define STL_PA_ATTRID_CLR_ALL_PORT_CTRS  0xA5
#define STL_PA_ATTRID_GET_PM_CONFIG      0xA6
#define STL_PA_ATTRID_FREEZE_IMAGE       0xA7
#define STL_PA_ATTRID_RELEASE_IMAGE      0xA8
#define STL_PA_ATTRID_RENEW_IMAGE        0xA9
#define STL_PA_ATTRID_GET_FOCUS_PORTS    0xAA
#define STL_PA_ATTRID_GET_IMAGE_INFO     0xAB
#define STL_PA_ATTRID_MOVE_FREEZE_FRAME  0xAC
#define STL_PA_ATTRID_GET_VF_LIST        0xAD
#define STL_PA_ATTRID_GET_VF_INFO        0xAE
#define STL_PA_ATTRID_GET_VF_CONFIG      0xAF
#define STL_PA_ATTRID_GET_VF_PORT_CTRS   0xB0
#define STL_PA_ATTRID_CLR_VF_PORT_CTRS   0xB1
#define STL_PA_ATTRID_GET_VF_FOCUS_PORTS 0xB2
#define STL_PA_ATTRID_GET_FOCUS_PORTS_MULTISELECT 0xB4
#define STL_PA_ATTRID_GET_GRP_NODE_INFO  0xB5
#define STL_PA_ATTRID_GET_GRP_LINK_INFO  0xB6
#define STL_PA_ATTRID_GET_GRP_LIST2      0xB7
#define STL_PA_ATTRID_GET_VF_LIST2       0xB8

/* Performance Analysis MAD status values */
#define STL_MAD_STATUS_STL_PA_UNAVAILABLE	0x0A00  // Engine unavailable
#define STL_MAD_STATUS_STL_PA_NO_GROUP		0x0B00  // No such group
#define STL_MAD_STATUS_STL_PA_NO_PORT		0x0C00  // Port not found
#define STL_MAD_STATUS_STL_PA_NO_VF			0x0D00  // VF not found
#define STL_MAD_STATUS_STL_PA_INVALID_PARAMETER	0x0E00  // Invalid parameter
#define STL_MAD_STATUS_STL_PA_NO_IMAGE		0x0F00  // Image not found
#define STL_MAD_STATUS_STL_PA_NO_DATA		0x1000  // No Data when port was not queried
#define STL_MAD_STATUS_STL_PA_BAD_DATA		0x1100  // Bad Data when query was unsuccessful

/* PM Service Record values */
#define STL_PM_SERVICE_NAME     "Primary Intel OmniPath Performance Manager"
#define STL_PM_SERVICE_NAME_SEC "Secondary Intel OmniPath Performance Manager"
#define STL_PM_SERVICE_ID		(0x1100d03c34845555ull)
#define STL_PM_SERVICE_ID_SEC   (0x1100d03c34845555ull)
#define STL_PM_VERSION		0x01
#define STL_PM_MASTER		0x01		/* master state */
#define STL_PM_STANDBY		0x02		/* standby state */

#define STL_PA_DATA_OFFSET	32

#define STL_PA_CLASS_VERSION	0x80



/* LinkQualityIndicator values */
#define STL_LINKQUALITY_EXCELLENT	5	/* working as intended */
#define STL_LINKQUALITY_VERY_GOOD	4	/* slightly below preferred, */
										/* no action needed */
#define STL_LINKQUALITY_GOOD		3	/* low end of acceptable, */
										/* recommend corrective action on */
										/* next maintenance window */
#define STL_LINKQUALITY_POOR		2	/* below acceptable, */
										/* recommend timely corrective action */
#define STL_LINKQUALITY_BAD			1	/* far below acceptable, */
										/* immediate corrective action */
#define STL_LINKQUALITY_NONE		0	/* link down */

#ifdef __cplusplus
}
#endif

#endif /* _STL_PA_TYPES_H_ */
