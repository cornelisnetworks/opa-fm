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

#ifndef _IBA_STL_PA_H_
#define _IBA_STL_PA_H_ (1) /* suppress duplicate loading of this file */

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"
#include "iba/ib_generalServices.h"
#include "iba/ib_pa.h"
#include "iba/stl_pm.h"

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

typedef STL_FIELDUNION3(STL_PA_CLASS_PORT_INFO_CAPABILITY_MASK, 16,
		Reserved1: 					7,     	/* start of class dependent bits */
		IsAbsTimeQuerySupported:	1,		/* RO - PA supports queries */
											/* w/ absoluteTime in Image ID */
		Reserved2: 					8);		/* class independent bits*/

/*
 *PA capability mask defines
 */
#define STL_PA_CPI_CAPMASK_ABSTIMEQUERY 0x0100

static __inline void
StlPaClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		snprintf(buf, 80, "%s%s%s%s",
			(cmask & STL_CLASS_PORT_CAPMASK_TRAP) ? "Trap " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_NOTICE) ? "Notice " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_CM2) ? "CapMask2 " : "",
			/* Class Specific */
			(cmask & STL_PA_CPI_CAPMASK_ABSTIMEQUERY) ? "AbsTime " : "");
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
	uint32					nodeLid;
	uint8					portNumber;
	uint8					reserved[3];
} PACK_SUFFIX STL_PA_PM_GROUP_CFG_RSP;


// STL_PORT_COUNTERS_DATA.flags
#define STL_PA_PC_FLAG_DELTA            0x00000001 // are these DELTA(1) or running totals
#define STL_PA_PC_FLAG_UNEXPECTED_CLEAR 0x00000002 // was there an unexpected clear
#define STL_PA_PC_FLAG_SHARED_VL        0x00000004 // for vf port counters, vl is shared >1 vf
#define STL_PA_PC_FLAG_USER_COUNTERS    0x00000008 // for PA user controlled running counters

typedef struct _STL_PORT_COUNTERS_DATA {
	uint32				nodeLid;
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

typedef struct _STL_CLR_PORT_COUNTERS_DATA {
	uint32		NodeLid;
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

// PM configuration flags for pmFlags below
#define STL_PM_NONE							0
#define STL_PM_PROCESS_VL_COUNTERS			0x000000001	// Enable Processing of VL Counters
#define STL_PM_PROCESS_HFI_COUNTERS			0x000000002	// Enable Processing of HFI Counters
#define STL_PM_PROCESS_CLR_DATA_COUNTERS	0x000000004	// Enable Clearing of Data Counters
#define STL_PM_PROCESS_CLR_64BIT_COUNTERS	0x000000008	// Enable Clearing of 64 bit Error Counters
#define STL_PM_PROCESS_CLR_32BIT_COUNTERS	0x000000010	// Enable Clearing of 32 bit Error Counters
#define STL_PM_PROCESS_CLR_8BIT_COUNTERS	0x000000020	// Enable Clearing of 8 bit Error Counters

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

#define STL_PA_SELECT_UTIL_HIGH			0x00020001			// highest first, descending
#define STL_PA_SELECT_UTIL_MC_HIGH		0x00020081
#define STL_PA_SELECT_UTIL_PKTS_HIGH	0x00020082
#define STL_PA_SELECT_UTIL_LOW			0x00020101			// lowest first, ascending
#define STL_PA_SELECT_UTIL_MC_LOW		0x00020102
#define STL_PA_SELECT_CATEGORY_INTEG			0x00030001			// hightest first, descending
#define STL_PA_SELECT_CATEGORY_CONG			0x00030002
#define STL_PA_SELECT_CATEGORY_SMA_CONG		0x00030003
#define STL_PA_SELECT_CATEGORY_BUBBLE		0x00030004
#define STL_PA_SELECT_CATEGORY_SEC			0x00030005
#define STL_PA_SELECT_CATEGORY_ROUT			0x00030006

typedef struct _STL_FOCUS_PORTS_REQ {
	char					groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	STL_PA_IMAGE_ID_DATA	imageId;
	uint32					select;
	uint32					start;
	uint32					range;
} PACK_SUFFIX STL_FOCUS_PORTS_REQ;

#define STL_PA_FOCUS_STATUS_OK           0
#define STL_PA_FOCUS_STATUS_PMA_IGNORE   1
#define STL_PA_FOCUS_STATUS_PMA_FAILURE  2
#define STL_PA_FOCUS_STATUS_TOPO_FAILURE 3

static __inline
const char* StlFocusStatusToText(uint8 status)
{
	switch (status) {
	case STL_PA_FOCUS_STATUS_OK:           return "OK";
	case STL_PA_FOCUS_STATUS_PMA_IGNORE:   return "PMA Ignore";
	case STL_PA_FOCUS_STATUS_PMA_FAILURE:  return "PMA Failure";
	case STL_PA_FOCUS_STATUS_TOPO_FAILURE: return "Topo Failure";
	default:                               return "Unknown";
	}
}

typedef struct _STL_FOCUS_PORTS_RSP {
	STL_PA_IMAGE_ID_DATA	imageId;
	uint32					nodeLid;
	uint8					portNumber;
	uint8					rate;	// IB_STATIC_RATE - 5 bit value
	uint8					maxVlMtu;	// enum IB_MTU - 4 bit value
	IB_BITFIELD2(uint8,     localStatus : 4,
		                    neighborStatus : 4)
	uint64					value;		// list sorting factor
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	uint32					neighborLid;
	uint8					neighborPortNumber;
	uint8					reserved3[3];
	uint64					neighborValue;
	uint64					neighborGuid;
	char					neighborNodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
} PACK_SUFFIX STL_FOCUS_PORTS_RSP;

typedef struct _STL_SMINFO_DATA {
	STL_LID_32				lid;
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
	uint32					nodeLid;
	uint8					portNumber;
	uint8					reserved[3];
} PACK_SUFFIX STL_PA_VF_CFG_RSP;

typedef struct _STL_PA_VF_PORT_COUNTERS_DATA {
	uint32				nodeLid;
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
	uint32		nodeLid;
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
	uint32					nodeLid;
	uint8					portNumber;
	uint8					rate;	// IB_STATIC_RATE - 5 bit value
	uint8					maxVlMtu;	// enum IB_MTU - 4 bit value
	IB_BITFIELD2(uint8,     localStatus : 4,
		                    neighborStatus : 4)
	uint64					value;		// list sorting factor
	uint64					nodeGUID;
	char					nodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
	uint32					neighborLid;
	uint8					neighborPortNumber;
	uint8					reserved3[3];
	uint64					neighborValue;
	uint64					neighborGuid;
	char					neighborNodeDesc[STL_PM_NODEDESCLEN]; // \0 terminated.
} PACK_SUFFIX STL_PA_VF_FOCUS_PORTS_RSP;

/* End of packed data structures */
#include "iba/public/ipackoff.h"



/* Performance Analysis methods */
#define STL_PA_CMD_GET                (0x01)
#define STL_PA_CMD_SET                (0x02)
#define STL_PA_CMD_GET_RESP           (0x81)
#define STL_PA_CMD_GETTABLE           (0x12)
#define STL_PA_CMD_GETTABLE_RESP      (0x92)


#define	MAD_STL_PA_REPLY              0x80		// Reply bit for methods

/* Performance Analysis attribute IDs */

#define STL_PA_ATTRID_GET_CLASSPORTINFO	 0x01
#define STL_PA_ATTRID_GET_GRP_LIST		 0xA0
#define STL_PA_ATTRID_GET_GRP_INFO		 0xA1
#define STL_PA_ATTRID_GET_GRP_CFG		 0xA2
#define STL_PA_ATTRID_GET_PORT_CTRS		 0xA3
#define STL_PA_ATTRID_CLR_PORT_CTRS		 0xA4
#define STL_PA_ATTRID_CLR_ALL_PORT_CTRS	 0xA5
#define STL_PA_ATTRID_GET_PM_CONFIG		 0xA6
#define STL_PA_ATTRID_FREEZE_IMAGE		 0xA7
#define STL_PA_ATTRID_RELEASE_IMAGE		 0xA8
#define STL_PA_ATTRID_RENEW_IMAGE		 0xA9
#define STL_PA_ATTRID_GET_FOCUS_PORTS	 0xAA
#define STL_PA_ATTRID_GET_IMAGE_INFO	 0xAB
#define STL_PA_ATTRID_MOVE_FREEZE_FRAME	 0xAC
#define STL_PA_ATTRID_GET_VF_LIST      	 0xAD
#define STL_PA_ATTRID_GET_VF_INFO      	 0xAE
#define STL_PA_ATTRID_GET_VF_CONFIG    	 0xAF
#define STL_PA_ATTRID_GET_VF_PORT_CTRS 	 0xB0
#define STL_PA_ATTRID_CLR_VF_PORT_CTRS 	 0xB1
#define STL_PA_ATTRID_GET_VF_FOCUS_PORTS 0xB2

/* Performance Analysis MAD status values */

#define STL_MAD_STATUS_STL_PA_UNAVAILABLE	0x0A00  // Engine unavailable
#define STL_MAD_STATUS_STL_PA_NO_GROUP		0x0B00  // No such group
#define STL_MAD_STATUS_STL_PA_NO_PORT		0x0C00  // Port not found
#define STL_MAD_STATUS_STL_PA_NO_VF			0x0D00  // VF not found
#define STL_MAD_STATUS_STL_PA_INVALID_PARAMETER	0x0E00  // Invalid parameter
#define STL_MAD_STATUS_STL_PA_NO_IMAGE		0x0F00  // Image not found

/* PM Service Record values */

#define STL_PM_SERVICE_NAME		"Intel Performance Manager service Rev 1.0"
#define STL_PM_SERVICE_ID		(0x1100d03c34845555ull)
#define STL_PM_VERSION		0x01
#define STL_PM_MASTER		0x01		/* master state */
#define STL_PM_STANDBY		0x02		/* standby state */

#define STL_PA_DATA_OFFSET	32

#define STL_PA_CLASS_VERSION	0x80


/* Performance Analysis Response Structures */
#define STL_PA_IMAGE_ID_NSIZE					sizeof(STL_PA_IMAGE_ID_DATA)
#define STL_PA_REC_DESC_LEN						64
#define STL_PA_TABLE_REC_DATA_LEN				512
#define STL_PA_RECORD_NSIZE						sizeof(StlPaRecord_t)
#define STL_PA_TABLE_RECORD_NSIZE				sizeof(StlPaTableRecord_t)
#define STL_PA_GROUP_LIST_NSIZE					sizeof(STL_PA_GROUP_LIST)
#define STL_PA_GROUP_INFO_NSIZE					sizeof(STL_PA_PM_GROUP_INFO_DATA)
#define STL_PA_GROUP_CONFIG_NSIZE				sizeof(STL_PA_PM_GROUP_CFG_RSP)
#define STL_PA_PORT_COUNTERS_NSIZE				sizeof(STL_PORT_COUNTERS_DATA)
#define STL_PA_CLR_PORT_COUNTERS_NSIZE			sizeof(STL_CLR_PORT_COUNTERS_DATA)
#define STL_PA_CLR_ALL_PORT_COUNTERS_NSIZE		sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA)
#define STL_PA_PM_CONFIG_NSIZE					sizeof(STL_PA_PM_CFG_DATA)
#define STL_PA_FOCUS_PORTS_NSIZE				sizeof(STL_FOCUS_PORTS_RSP)
#define STL_PA_IMAGE_INFO_NSIZE					sizeof(STL_PA_IMAGE_INFO_DATA)
#define STL_PA_MOVE_FREEZE_NSIZE				sizeof(STL_MOVE_FREEZE_DATA)
#define STL_PA_VF_LIST_NSIZE					sizeof(STL_PA_VF_LIST)
#define STL_PA_VF_INFO_NSIZE					sizeof(STL_PA_VF_INFO_DATA)
#define STL_PA_VF_CONFIG_NSIZE					sizeof(STL_PA_VF_CFG_RSP)
#define STL_PA_VF_PORT_COUNTERS_NSIZE			sizeof(STL_PA_VF_PORT_COUNTERS_DATA)
#define STL_PA_CLR_VF_PORT_COUNTERS_NSIZE		sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA)
#define STL_PA_VF_FOCUS_PORTS_NSIZE				sizeof(STL_PA_VF_FOCUS_PORTS_RSP)

typedef struct StlPaRecord_s {
  uint16_t fieldUint16;  
  uint16_t fieldFiller;  
  uint32_t fieldUint32;
  uint64_t fieldUint64;             
  uint8_t desc[STL_PA_REC_DESC_LEN];
} StlPaRecord_t;

typedef struct StlPaTableRecord_s {
    uint16_t fieldUint16;  
    uint16_t fieldFiller;  
    uint32_t fieldUint32;
    uint64_t fieldUint64;             
  uint8_t desc[STL_PA_REC_DESC_LEN];
  uint8_t data[STL_PA_TABLE_REC_DATA_LEN];
} StlPaTableRecord_t;

typedef struct _STL_PA_TABLE_RECORD_RESULTS  {
    uint32 NumPaRecords;                /* Number of PA Records returned */
    PaTableRecord_t PaRecords[1];		/* list of PA records returned */
} STL_PA_TABLE_RECORD_RESULTS, *PSTL_PA_TABLE_RECORD_RESULTS;

typedef struct _STL_PA_GROUP_LIST_RESULTS  {
    uint32 NumGroupListRecords;                /* Number of PA Records returned */
    STL_PA_GROUP_LIST GroupListRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_LIST_RESULTS, *PSTL_PA_GROUP_LIST_RESULTS;

typedef struct _STL_PA_GROUP_INFO_RESULTS  {
    uint32 NumGroupInfoRecords;                /* Number of PA Records returned */
    STL_PA_PM_GROUP_INFO_DATA GroupInfoRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_INFO_RESULTS, *PSTL_PA_GROUP_INFO_RESULTS;

typedef struct _STL_PA_GROUP_CONFIG_RESULTS  {
    uint32 NumGroupConfigRecords;                /* Number of PA Records returned */
    STL_PA_PM_GROUP_CFG_RSP GroupConfigRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_CONFIG_RESULTS, *PSTL_PA_GROUP_CONFIG_RESULTS;

typedef struct _STL_PA_PORT_COUNTERS_RESULTS  {
    uint32 NumPortCountersRecords;                /* Number of PA Records returned */
    STL_PORT_COUNTERS_DATA PortCountersRecords[1];		/* list of PA records returned */
} STL_PA_PORT_COUNTERS_RESULTS, *PSTL_PA_PORT_COUNTERS_RESULTS;

typedef struct _STL_PA_CLR_PORT_COUNTERS_RESULTS  {
    uint32 NumClrPortCountersRecords;                /* Number of PA Records returned */
    STL_CLR_PORT_COUNTERS_DATA ClrPortCountersRecords[1];		/* list of PA records returned */
} STL_PA_CLR_PORT_COUNTERS_RESULTS, *PSTL_PA_CLR_PORT_COUNTERS_RESULTS;

typedef struct _STL_PA_CLR_ALL_PORT_COUNTERS_RESULTS  {
    uint32 NumClrAllPortCountersRecords;                /* Number of PA Records returned */
    STL_CLR_ALL_PORT_COUNTERS_DATA ClrAllPortCountersRecords[1];		/* list of PA records returned */
} STL_PA_CLR_ALL_PORT_COUNTERS_RESULTS, *PSTL_PA_CLR_ALL_PORT_COUNTERS_RESULTS;

typedef struct _STL_PA_PM_CONFIG_RESULTS  {
    uint32 NumPmConfigRecords;                /* Number of PA Records returned */
    STL_PA_PM_CFG_DATA PmConfigRecords[1];		/* list of PA records returned */
} STL_PA_PM_CONFIG_RESULTS, *PSTL_PA_PM_CONFIG_RESULTS;

typedef struct _STL_PA_IMAGE_ID_RESULTS  {
    uint32 NumImageIDRecords;                /* Number of PA Records returned */
    STL_PA_IMAGE_ID_DATA ImageIDRecords[1];		/* list of PA records returned */
} STL_PA_IMAGE_ID_RESULTS, *PSTL_PA_IMAGE_ID_RESULTS;

typedef struct _STL_PA_FOCUS_PORTS_RESULTS  {
    uint32 NumFocusPortsRecords;                /* Number of PA Records returned */
    STL_FOCUS_PORTS_RSP FocusPortsRecords[1];		/* list of PA records returned */
} STL_PA_FOCUS_PORTS_RESULTS, *PSTL_PA_FOCUS_PORTS_RESULTS;

typedef struct _STL_PA_IMAGE_INFO_RESULTS  {
    uint32 NumImageInfoRecords;                /* Number of PA Records returned */
    STL_PA_IMAGE_INFO_DATA ImageInfoRecords[1];		/* list of PA records returned */
} STL_PA_IMAGE_INFO_RESULTS, *PSTL_PA_IMAGE_INFO_RESULTS;

typedef struct _STL_MOVE_FREEZE_RESULTS  {
    uint32 NumMoveFreezeRecords;                /* Number of PA Records returned */
    STL_MOVE_FREEZE_DATA MoveFreezeRecords[1];		/* list of PA records returned */
} STL_MOVE_FREEZE_RESULTS, *PSTL_PA_MOVE_FREEZE_RESULTS;

typedef struct _STL_PA_VF_LIST_RESULTS  {
    uint32 NumVFListRecords;                /* Number of PA Records returned */
    STL_PA_VF_LIST VFListRecords[1];		/* list of PA records returned */
} STL_PA_VF_LIST_RESULTS, *PSTL_PA_VF_LIST_RESULTS;

typedef struct _STL_PA_VF_INFO_RESULTS  {
    uint32 NumVFInfoRecords;                /* Number of PA Records returned */
    STL_PA_VF_INFO_DATA VFInfoRecords[1];		/* list of PA records returned */
} STL_PA_VF_INFO_RESULTS, *PSTL_PA_VF_INFO_RESULTS;

typedef struct _STL_PA_VF_CONFIG_RESULTS  {
    uint32 NumVFConfigRecords;                /* Number of PA Records returned */
    STL_PA_VF_CFG_RSP VFConfigRecords[1];		/* list of PA records returned */
} STL_PA_VF_CONFIG_RESULTS, *PSTL_PA_VF_CONFIG_RESULTS;

typedef struct _STL_PA_VF_FOCUS_PORTS_RESULTS  {
    uint32 NumVFFocusPortsRecords;                /* Number of PA Records returned */
    STL_PA_VF_FOCUS_PORTS_RSP FocusPortsRecords[1];		/* list of PA records returned */
} STL_PA_VF_FOCUS_PORTS_RESULTS, *PSTL_PA_VF_FOCUS_PORTS_RESULTS;

static __inline void
BSWAP_STL_PA_RECORD(StlPaRecord_t  *pRecord)
{
#if CPU_LE
	pRecord->fieldUint16 = ntoh16(pRecord->fieldUint16);
	pRecord->fieldUint32 = ntoh32(pRecord->fieldUint32);
	pRecord->fieldUint64 = ntoh64(pRecord->fieldUint64);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_TABLE_RECORD(StlPaTableRecord_t  *pRecord)
{
#if CPU_LE
	pRecord->fieldUint16 = ntoh16(pRecord->fieldUint16);
	pRecord->fieldUint32 = ntoh32(pRecord->fieldUint32);
	pRecord->fieldUint64 = ntoh64(pRecord->fieldUint64);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_IMAGE_ID(STL_PA_IMAGE_ID_DATA *pRecord)
{
#if CPU_LE
	pRecord->imageNumber						= ntoh64(pRecord->imageNumber);
	pRecord->imageOffset						= ntoh32(pRecord->imageOffset);
	pRecord->imageTime.absoluteTime				= ntoh32(pRecord->imageTime.absoluteTime);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_PM_GROUP_INFO(STL_PA_PM_GROUP_INFO_DATA *pRecord, boolean isRequest)
{
#if CPU_LE
	int i;
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);

	if (isRequest) return;

	pRecord->numInternalPorts				= ntoh32(pRecord->numInternalPorts);
	pRecord->numExternalPorts				= ntoh32(pRecord->numExternalPorts);

	pRecord->internalUtilStats.totalMBps	= ntoh64(pRecord->internalUtilStats.totalMBps);
	pRecord->internalUtilStats.totalKPps	= ntoh64(pRecord->internalUtilStats.totalKPps);
	pRecord->internalUtilStats.avgMBps		= ntoh32(pRecord->internalUtilStats.avgMBps);
	pRecord->internalUtilStats.minMBps		= ntoh32(pRecord->internalUtilStats.minMBps);
	pRecord->internalUtilStats.maxMBps		= ntoh32(pRecord->internalUtilStats.maxMBps);
	pRecord->internalUtilStats.numBWBuckets		= ntoh32(pRecord->internalUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->internalUtilStats.BWBuckets[i] = ntoh32(pRecord->internalUtilStats.BWBuckets[i]);
	pRecord->internalUtilStats.avgKPps		= ntoh32(pRecord->internalUtilStats.avgKPps);
	pRecord->internalUtilStats.minKPps		= ntoh32(pRecord->internalUtilStats.minKPps);
	pRecord->internalUtilStats.maxKPps		= ntoh32(pRecord->internalUtilStats.maxKPps);
	pRecord->internalUtilStats.pmaNoRespPorts = ntoh16(pRecord->internalUtilStats.pmaNoRespPorts);
	pRecord->internalUtilStats.topoIncompPorts = ntoh16(pRecord->internalUtilStats.topoIncompPorts);

	pRecord->sendUtilStats.totalMBps		= ntoh64(pRecord->sendUtilStats.totalMBps);
	pRecord->sendUtilStats.totalKPps		= ntoh64(pRecord->sendUtilStats.totalKPps);
	pRecord->sendUtilStats.avgMBps			= ntoh32(pRecord->sendUtilStats.avgMBps);
	pRecord->sendUtilStats.minMBps			= ntoh32(pRecord->sendUtilStats.minMBps);
	pRecord->sendUtilStats.maxMBps			= ntoh32(pRecord->sendUtilStats.maxMBps);
	pRecord->sendUtilStats.numBWBuckets		= ntoh32(pRecord->sendUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->sendUtilStats.BWBuckets[i] = ntoh32(pRecord->sendUtilStats.BWBuckets[i]);
	pRecord->sendUtilStats.avgKPps			= ntoh32(pRecord->sendUtilStats.avgKPps);
	pRecord->sendUtilStats.minKPps			= ntoh32(pRecord->sendUtilStats.minKPps);
	pRecord->sendUtilStats.maxKPps			= ntoh32(pRecord->sendUtilStats.maxKPps);
	pRecord->sendUtilStats.pmaNoRespPorts   = ntoh16(pRecord->sendUtilStats.pmaNoRespPorts);
	pRecord->sendUtilStats.topoIncompPorts  = ntoh16(pRecord->sendUtilStats.topoIncompPorts);

	pRecord->recvUtilStats.totalMBps		= ntoh64(pRecord->recvUtilStats.totalMBps);
	pRecord->recvUtilStats.totalKPps		= ntoh64(pRecord->recvUtilStats.totalKPps);
	pRecord->recvUtilStats.avgMBps			= ntoh32(pRecord->recvUtilStats.avgMBps);
	pRecord->recvUtilStats.minMBps			= ntoh32(pRecord->recvUtilStats.minMBps);
	pRecord->recvUtilStats.maxMBps			= ntoh32(pRecord->recvUtilStats.maxMBps);
	pRecord->recvUtilStats.numBWBuckets		= ntoh32(pRecord->recvUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->recvUtilStats.BWBuckets[i] = ntoh32(pRecord->recvUtilStats.BWBuckets[i]);
	pRecord->recvUtilStats.avgKPps			= ntoh32(pRecord->recvUtilStats.avgKPps);
	pRecord->recvUtilStats.minKPps			= ntoh32(pRecord->recvUtilStats.minKPps);
	pRecord->recvUtilStats.maxKPps			= ntoh32(pRecord->recvUtilStats.maxKPps);
	pRecord->recvUtilStats.pmaNoRespPorts   = ntoh16(pRecord->recvUtilStats.pmaNoRespPorts);
	pRecord->recvUtilStats.topoIncompPorts  = ntoh16(pRecord->recvUtilStats.topoIncompPorts);

	pRecord->internalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->internalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.congestion);
	pRecord->internalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->internalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->internalCategoryStats.categoryMaximums.bubble				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.bubble);
	pRecord->internalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.securityErrors);
	pRecord->internalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.routingErrors);

        pRecord->internalCategoryStats.categoryMaximums.utilizationPct10 = ntoh16(pRecord->internalCategoryStats.categoryMaximums.utilizationPct10);
        pRecord->internalCategoryStats.categoryMaximums.discardsPct10    = ntoh16(pRecord->internalCategoryStats.categoryMaximums.discardsPct10);

	for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
		pRecord->internalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].integrityErrors);
		pRecord->internalCategoryStats.ports[i].congestion			= ntoh32(pRecord->internalCategoryStats.ports[i].congestion);
		pRecord->internalCategoryStats.ports[i].smaCongestion		= ntoh32(pRecord->internalCategoryStats.ports[i].smaCongestion);
		pRecord->internalCategoryStats.ports[i].bubble				= ntoh32(pRecord->internalCategoryStats.ports[i].bubble);
		pRecord->internalCategoryStats.ports[i].securityErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].securityErrors);
		pRecord->internalCategoryStats.ports[i].routingErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].routingErrors);
	}

	pRecord->externalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->externalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.congestion);
	pRecord->externalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->externalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->externalCategoryStats.categoryMaximums.bubble				= ntoh32(pRecord->externalCategoryStats.categoryMaximums.bubble);
	pRecord->externalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.securityErrors);
	pRecord->externalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->externalCategoryStats.categoryMaximums.routingErrors);

        pRecord->externalCategoryStats.categoryMaximums.utilizationPct10 = ntoh16(pRecord->externalCategoryStats.categoryMaximums.utilizationPct10);
        pRecord->externalCategoryStats.categoryMaximums.discardsPct10    = ntoh16(pRecord->externalCategoryStats.categoryMaximums.discardsPct10);

        for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
		pRecord->externalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].integrityErrors);
		pRecord->externalCategoryStats.ports[i].congestion			= ntoh32(pRecord->externalCategoryStats.ports[i].congestion);
		pRecord->externalCategoryStats.ports[i].smaCongestion		= ntoh32(pRecord->externalCategoryStats.ports[i].smaCongestion);
		pRecord->externalCategoryStats.ports[i].bubble				= ntoh32(pRecord->externalCategoryStats.ports[i].bubble);
		pRecord->externalCategoryStats.ports[i].securityErrors				= ntoh32(pRecord->externalCategoryStats.ports[i].securityErrors);
			pRecord->externalCategoryStats.ports[i].routingErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].routingErrors);
	}
	pRecord->maxInternalMBps				= ntoh32(pRecord->maxInternalMBps);
	pRecord->maxExternalMBps				= ntoh32(pRecord->maxExternalMBps);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_LIST(STL_PA_GROUP_LIST *pRecord)
{
#if CPU_LE
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_CONFIG_REQ(STL_PA_PM_GROUP_CFG_REQ *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_CONFIG_RSP(STL_PA_PM_GROUP_CFG_RSP *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->nodeGUID							= ntoh64(pRecord->nodeGUID);
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_PORT_COUNTERS(STL_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->flags								= ntoh32(pRecord->flags);
	pRecord->portXmitData						= ntoh64(pRecord->portXmitData);
	pRecord->portRcvData						= ntoh64(pRecord->portRcvData);
	pRecord->portXmitPkts						= ntoh64(pRecord->portXmitPkts);
	pRecord->portRcvPkts						= ntoh64(pRecord->portRcvPkts);
	pRecord->portMulticastXmitPkts				= ntoh64(pRecord->portMulticastXmitPkts);
	pRecord->portMulticastRcvPkts				= ntoh64(pRecord->portMulticastRcvPkts);
	pRecord->localLinkIntegrityErrors			= ntoh64(pRecord->localLinkIntegrityErrors);
	pRecord->fmConfigErrors						= ntoh64(pRecord->fmConfigErrors);
	pRecord->portRcvErrors						= ntoh64(pRecord->portRcvErrors);
	pRecord->excessiveBufferOverruns			= ntoh64(pRecord->excessiveBufferOverruns);
	pRecord->portRcvConstraintErrors			= ntoh64(pRecord->portRcvConstraintErrors);
	pRecord->portRcvSwitchRelayErrors			= ntoh64(pRecord->portRcvSwitchRelayErrors);
	pRecord->portXmitDiscards					= ntoh64(pRecord->portXmitDiscards);
	pRecord->portXmitConstraintErrors			= ntoh64(pRecord->portXmitConstraintErrors);
	pRecord->portRcvRemotePhysicalErrors		= ntoh64(pRecord->portRcvRemotePhysicalErrors);
	pRecord->swPortCongestion					= ntoh64(pRecord->swPortCongestion);
	pRecord->portXmitWait						= ntoh64(pRecord->portXmitWait);
	pRecord->portRcvFECN						= ntoh64(pRecord->portRcvFECN);
	pRecord->portRcvBECN						= ntoh64(pRecord->portRcvBECN);
	pRecord->portXmitTimeCong					= ntoh64(pRecord->portXmitTimeCong);
	pRecord->portXmitWastedBW					= ntoh64(pRecord->portXmitWastedBW);
	pRecord->portXmitWaitData					= ntoh64(pRecord->portXmitWaitData);
	pRecord->portRcvBubble						= ntoh64(pRecord->portRcvBubble);
	pRecord->portMarkFECN						= ntoh64(pRecord->portMarkFECN);
	pRecord->linkErrorRecovery					= ntoh32(pRecord->linkErrorRecovery);
	pRecord->linkDowned							= ntoh32(pRecord->linkDowned);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_CLR_PORT_COUNTERS(STL_CLR_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->NodeLid							= ntoh32(pRecord->NodeLid);
	pRecord->CounterSelectMask.AsReg32			= ntoh32(pRecord->CounterSelectMask.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(STL_CLR_ALL_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->CounterSelectMask.AsReg32			= ntoh32(pRecord->CounterSelectMask.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_PM_CFG(STL_PA_PM_CFG_DATA *pRecord)
{
#if CPU_LE
	pRecord->sweepInterval						= ntoh32(pRecord->sweepInterval);
	pRecord->maxClients							= ntoh32(pRecord->maxClients);
	pRecord->sizeHistory						= ntoh32(pRecord->sizeHistory);
	pRecord->sizeFreeze							= ntoh32(pRecord->sizeFreeze);
	pRecord->lease								= ntoh32(pRecord->lease);
	pRecord->pmFlags							= ntoh32(pRecord->pmFlags);
	pRecord->categoryThresholds.integrityErrors	= ntoh32(pRecord->categoryThresholds.integrityErrors);
	pRecord->categoryThresholds.congestion	= ntoh32(pRecord->categoryThresholds.congestion);
	pRecord->categoryThresholds.smaCongestion	= ntoh32(pRecord->categoryThresholds.smaCongestion);
	pRecord->categoryThresholds.bubble		= ntoh32(pRecord->categoryThresholds.bubble);
	pRecord->categoryThresholds.securityErrors		= ntoh32(pRecord->categoryThresholds.securityErrors);
	pRecord->categoryThresholds.routingErrors		= ntoh32(pRecord->categoryThresholds.routingErrors);
	pRecord->memoryFootprint					= ntoh64(pRecord->memoryFootprint);
	pRecord->maxAttempts						= ntoh32(pRecord->maxAttempts);
	pRecord->respTimeout						= ntoh32(pRecord->respTimeout);
	pRecord->minRespTimeout						= ntoh32(pRecord->minRespTimeout);
	pRecord->maxParallelNodes					= ntoh32(pRecord->maxParallelNodes);
	pRecord->pmaBatchSize						= ntoh32(pRecord->pmaBatchSize);
	// ErrorClear is uint8
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_FOCUS_PORTS_REQ(STL_FOCUS_PORTS_REQ *pRecord)
{
#if CPU_LE
	pRecord->select						= ntoh32(pRecord->select);
	pRecord->start						= ntoh32(pRecord->start);
	pRecord->range						= ntoh32(pRecord->range);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif
}

static __inline void
BSWAP_STL_PA_FOCUS_PORTS_RSP(STL_FOCUS_PORTS_RSP *pRecord)
{
#if CPU_LE
	pRecord->nodeLid					= ntoh32(pRecord->nodeLid);
	pRecord->value						= ntoh64(pRecord->value);
	pRecord->neighborLid				= ntoh32(pRecord->neighborLid);
	pRecord->neighborValue				= ntoh64(pRecord->neighborValue);
	pRecord->nodeGUID					= ntoh64(pRecord->nodeGUID);
	pRecord->neighborGuid				= ntoh64(pRecord->neighborGuid);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_IMAGE_INFO(STL_PA_IMAGE_INFO_DATA *pRecord)
{
#if CPU_LE
	int i;
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->sweepStart              = ntoh64(pRecord->sweepStart);
	pRecord->sweepDuration           = ntoh32(pRecord->sweepDuration);
	pRecord->numHFIPorts             = ntoh16(pRecord->numHFIPorts);
	pRecord->numSwitchNodes          = ntoh16(pRecord->numSwitchNodes);
	pRecord->numSwitchPorts          = ntoh32(pRecord->numSwitchPorts);
	pRecord->numLinks                = ntoh32(pRecord->numLinks);
	pRecord->numSMs                  = ntoh32(pRecord->numSMs);
	pRecord->numNoRespNodes          = ntoh32(pRecord->numNoRespNodes);
	pRecord->numNoRespPorts          = ntoh32(pRecord->numNoRespPorts);
	pRecord->numSkippedNodes         = ntoh32(pRecord->numSkippedNodes);
	pRecord->numSkippedPorts         = ntoh32(pRecord->numSkippedPorts);
	pRecord->numUnexpectedClearPorts = ntoh32(pRecord->numUnexpectedClearPorts);
	pRecord->imageInterval           = ntoh32(pRecord->imageInterval);
	for (i = 0; i < 2; i++) {
		pRecord->SMInfo[i].lid        = ntoh32(pRecord->SMInfo[i].lid);
		pRecord->SMInfo[i].smPortGuid = ntoh64(pRecord->SMInfo[i].smPortGuid);
	}
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_MOVE_FREEZE(STL_MOVE_FREEZE_DATA *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->oldFreezeImage);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->newFreezeImage);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_LIST(STL_PA_VF_LIST *pRecord)
{
#if CPU_LE
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_INFO(STL_PA_VF_INFO_DATA *pRecord, boolean isRequest)
{
#if CPU_LE
	int i;
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);

	if (isRequest) return;

	pRecord->numPorts						= ntoh32(pRecord->numPorts);

	pRecord->internalUtilStats.totalMBps	= ntoh64(pRecord->internalUtilStats.totalMBps);
	pRecord->internalUtilStats.totalKPps	= ntoh64(pRecord->internalUtilStats.totalKPps);
	pRecord->internalUtilStats.avgMBps		= ntoh32(pRecord->internalUtilStats.avgMBps);
	pRecord->internalUtilStats.minMBps		= ntoh32(pRecord->internalUtilStats.minMBps);
	pRecord->internalUtilStats.maxMBps		= ntoh32(pRecord->internalUtilStats.maxMBps);
	pRecord->internalUtilStats.numBWBuckets		= ntoh32(pRecord->internalUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->internalUtilStats.BWBuckets[i] = ntoh32(pRecord->internalUtilStats.BWBuckets[i]);
	pRecord->internalUtilStats.avgKPps		= ntoh32(pRecord->internalUtilStats.avgKPps);
	pRecord->internalUtilStats.minKPps		= ntoh32(pRecord->internalUtilStats.minKPps);
	pRecord->internalUtilStats.maxKPps		= ntoh32(pRecord->internalUtilStats.maxKPps);
	pRecord->internalUtilStats.pmaNoRespPorts = ntoh16(pRecord->internalUtilStats.pmaNoRespPorts);
	pRecord->internalUtilStats.topoIncompPorts = ntoh16(pRecord->internalUtilStats.topoIncompPorts);

	pRecord->internalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->internalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.congestion);
	pRecord->internalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->internalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->internalCategoryStats.categoryMaximums.bubble				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.bubble);
	pRecord->internalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.securityErrors);
	pRecord->internalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.routingErrors);

        pRecord->internalCategoryStats.categoryMaximums.utilizationPct10 = ntoh16(pRecord->internalCategoryStats.categoryMaximums.utilizationPct10);
        pRecord->internalCategoryStats.categoryMaximums.discardsPct10    = ntoh16(pRecord->internalCategoryStats.categoryMaximums.discardsPct10);

	for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
		pRecord->internalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].integrityErrors);
		pRecord->internalCategoryStats.ports[i].congestion			= ntoh32(pRecord->internalCategoryStats.ports[i].congestion);
		pRecord->internalCategoryStats.ports[i].smaCongestion		= ntoh32(pRecord->internalCategoryStats.ports[i].smaCongestion);
		pRecord->internalCategoryStats.ports[i].bubble				= ntoh32(pRecord->internalCategoryStats.ports[i].bubble);
		pRecord->internalCategoryStats.ports[i].securityErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].securityErrors);
		pRecord->internalCategoryStats.ports[i].routingErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].routingErrors);
	}

	pRecord->maxInternalMBps				= ntoh32(pRecord->maxInternalMBps);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_CFG_REQ(STL_PA_VF_CFG_REQ *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_CFG_RSP(STL_PA_VF_CFG_RSP *pRecord)
{
#if CPU_LE
	pRecord->nodeGUID						= ntoh64(pRecord->nodeGUID);
	pRecord->nodeLid						= ntoh32(pRecord->nodeLid);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_PORT_COUNTERS(STL_PA_VF_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->flags								= ntoh32(pRecord->flags);
	pRecord->portVFXmitData						= ntoh64(pRecord->portVFXmitData);
	pRecord->portVFRcvData						= ntoh64(pRecord->portVFRcvData);
	pRecord->portVFXmitPkts						= ntoh64(pRecord->portVFXmitPkts);
	pRecord->portVFRcvPkts						= ntoh64(pRecord->portVFRcvPkts);
	pRecord->portVFXmitDiscards					= ntoh16(pRecord->portVFXmitDiscards);
	pRecord->swPortVFCongestion					= ntoh64(pRecord->swPortVFCongestion);
	pRecord->portVFXmitWait						= ntoh64(pRecord->portVFXmitWait);
	pRecord->portVFRcvFECN						= ntoh64(pRecord->portVFRcvFECN);
	pRecord->portVFRcvBECN						= ntoh64(pRecord->portVFRcvBECN);
	pRecord->portVFXmitTimeCong					= ntoh64(pRecord->portVFXmitTimeCong);
	pRecord->portVFXmitWastedBW					= ntoh64(pRecord->portVFXmitWastedBW);
	pRecord->portVFXmitWaitData					= ntoh64(pRecord->portVFXmitWaitData);
	pRecord->portVFRcvBubble					= ntoh64(pRecord->portVFRcvBubble);
	pRecord->portVFMarkFECN						= ntoh64(pRecord->portVFMarkFECN);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_CLEAR_VF_PORT_COUNTERS(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->vfCounterSelectMask.AsReg32		= ntoh32(pRecord->vfCounterSelectMask.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_FOCUS_PORTS_REQ(STL_PA_VF_FOCUS_PORTS_REQ *pRecord)
{
#if CPU_LE
	pRecord->select						= ntoh32(pRecord->select);
	pRecord->start						= ntoh32(pRecord->start);
	pRecord->range						= ntoh32(pRecord->range);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif
}

static __inline void
BSWAP_STL_PA_VF_FOCUS_PORTS_RSP(STL_PA_VF_FOCUS_PORTS_RSP *pRecord)
{
#if CPU_LE
	pRecord->nodeLid					= ntoh32(pRecord->nodeLid);
	pRecord->value						= ntoh64(pRecord->value);
	pRecord->neighborLid				= ntoh32(pRecord->neighborLid);
	pRecord->neighborValue				= ntoh64(pRecord->neighborValue);
	pRecord->nodeGUID					= ntoh64(pRecord->nodeGUID);
	pRecord->neighborGuid				= ntoh64(pRecord->neighborGuid);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

#ifdef __cplusplus
}
#endif

#endif /* _IBA_STL_PA_H_ */
