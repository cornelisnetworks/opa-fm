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

#ifndef _IBA_STL_PA_H_
#warning FIX ME!!! Your includes should use the stl_pa.h header and not the ib_pa.h header for STL builds
#endif

#endif

#ifndef _IBA_IB_PA_H_
#define _IBA_IB_PA_H_ (1) /* suppress duplicate loading of this file */

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


#include "iba/public/ipackon.h"

/** PA MAD Payloads */

/* these should be the same as in pmlib.h */
#define PM_UTIL_GRAN_PERCENT 10 /* granularity of utilization buckets */
#define PM_UTIL_BUCKETS (100/PM_UTIL_GRAN_PERCENT)

#define PM_ERR_GRAN_PERCENT 25  /* granularity of error buckets */
#define PM_ERR_BUCKETS ((100/PM_ERR_GRAN_PERCENT)+1) // extra bucket is for those over threshold

#define PM_MAX_GROUPS		10
#define PM_GROUPNAMELEN		64

#define PORT_LIST_SIZE		1024

typedef struct _IMAGE_ID_DATA {
	uint64		imageNumber;
	int32		imageOffset;
	uint32		reserved;
} PACK_SUFFIX IMAGE_ID_DATA;

typedef struct _PMUTILSTATSTRUCT {
	uint64		totalMBps;
	uint64		totalKPps;
	uint32		avgMBps;
	uint32		minMBps;
	uint32		maxMBps;
	uint32		numBWBuckets;
	uint32		BWBuckets[PM_UTIL_BUCKETS];
	uint32		avgKPps;
	uint32		minKPps;
	uint32		maxKPps;
	uint32		reserved;
} PACK_SUFFIX PMUTILSTAT_T;

// TBD - why not make names and sizes of fields consistent with ErrorSummary_t
// so can use simple macros to copy fields?
// TBD - same question for other structures here
typedef struct _PMERRSUMMARYSTRUCT {
	uint32	integrityErrors;
	uint32	congestion;
	uint32	smaCongestion;
	uint32	securityErrors;
	uint32	routingErrors;
	uint16	discard;
	//uint16	waitPct10;
	uint16	congestionPct10;
	uint16	inefficiencyPct10;
	uint16	reserved;
	uint32	adaptiveRouting;
} PACK_SUFFIX PMERRSUMMARY_T;

typedef struct _PMERRTHRESHOLDSTRUCT {
	uint32	integrityErrors;
	uint32	congestion;
	uint32	smaCongestion;
	uint32	securityErrors;
	uint32	routingErrors;
	uint32	reserved;
} PACK_SUFFIX PMERRTHRESHOLD_T;

typedef struct PMERRBUCKETSTRUCT {
			uint32	integrityErrors;
			uint32	congestion;
			uint32	smaCongestion;
			uint32	securityErrors;
			uint32	routingErrors;
			uint32	reserved;
} PACK_SUFFIX PMERRBUCKET_T;

typedef struct PMERRSTATSTRUCT {
	PMERRSUMMARY_T	categoryMaximums;
	PMERRBUCKET_T	ports[PM_ERR_BUCKETS];
} PACK_SUFFIX PMERRSTAT_T;


typedef struct _GROUP_INFO_DATA {
	char					groupName[PM_GROUPNAMELEN];	// \0 terminated
	IMAGE_ID_DATA			imageId;
	uint32					numInternalPorts;
	uint32					numExternalPorts;
	PMUTILSTAT_T			internalUtilStats;
	PMUTILSTAT_T			sendUtilStats;
	PMUTILSTAT_T			recvUtilStats;
	PMERRSTAT_T				internalCategoryStats;
	PMERRSTAT_T				externalCategoryStats;
	// these are added at the end to allow for forward and backward
	// compatibility.
	uint8					maxInternalRate;
	uint8					minInternalRate;
	uint8					maxExternalRate;
	uint8					minExternalRate;
	uint32					maxInternalMBps;
	uint32					maxExternalMBps;
} PACK_SUFFIX GROUP_INFO_DATA;


typedef struct _GROUP_LIST_DATA {
	uint32					numGroups;
	char					groupNames[PM_MAX_GROUPS][PM_GROUPNAMELEN];	// \0 terminated
} PACK_SUFFIX GROUP_LIST_DATA;

// TBD - add nodeType?  Port Guid? for SW have numPorts? add mtu, rate? add neighbor?
typedef struct _PORT_CONFIG_DATA {
	uint64					nodeGuid;
	char					nodeDesc[64]; // can be 64 char w/o \0
	uint16					nodeLid;
	uint8					portNumber;
	uint8					reserved;
	uint32					reserved2;
} PACK_SUFFIX PORT_CONFIG_DATA;


typedef struct _GROUP_CONFIG_DATA {
	char					groupName[PM_GROUPNAMELEN];	// \0 terminated
	IMAGE_ID_DATA			imageId;
	uint32					numPorts;
	uint32					reserved;
	PORT_CONFIG_DATA		portList[1];
} PACK_SUFFIX GROUP_CONFIG_DATA;

// is this still used?
typedef struct _GROUP_CONFIG_DATA2 {
	char					groupName[PM_GROUPNAMELEN];	// \0 terminated
	IMAGE_ID_DATA			imageId;
	uint32					numPorts;
	uint32					reserved;
	PORT_CONFIG_DATA		portList[500];			// need this to hold lots of ports in the response
} PACK_SUFFIX GROUP_CONFIG_DATA2;

// PORT_COUNTERS_DATA.flags
#define PA_PC_FLAG_DELTA 0x00000001	// are these DELTA(1) or running totals
#define PA_PC_FLAG_UNEXPECTED_CLEAR 0x00000002	// was there an unexpected clear

typedef struct _PORT_COUNTERS_DATA {
	uint32				nodeLid;
	uint32				portNumber;
	uint32				flags;
	uint32				reserved1;
	uint16				SymbolErrorCounter;
	uint8				LinkErrorRecoveryCounter;
	uint8				LinkDownedCounter;
	uint16				PortRcvErrors;
	uint16				PortRcvRemotePhysicalErrors;
	uint16				PortRcvSwitchRelayErrors;
	uint16				PortXmitDiscards;
	uint8				PortXmitConstraintErrors;
	uint8				PortRcvConstraintErrors;
	uint8				reserved2;
	uint8				ExcessiveBufferOverrunErrors:4;
	uint8				LocalLinkIntegrityErrors:4;
	uint16				PortCheckRate;	// see ib_pm.h for masks and meaning
	uint16				VL15Dropped;
	uint32				PortXmitWait;
	uint64				PortXmitData;
	uint64				PortRcvData;
	uint64				PortXmitPkts;
	uint64				PortRcvPkts;
	uint64				PortXmitCongestion;
	uint64				PortAdaptiveRouting;
	uint16				CongestionPct10;
	uint16				InefficiencyPct10;
	uint32				reserved;
	IMAGE_ID_DATA		imageId;
} PACK_SUFFIX PORT_COUNTERS_DATA;

typedef struct _CLR_PORT_COUNTERS_DATA {
	uint32		nodeLid;
	uint32		portNumber;
	uint16		select;
	uint16		reserved;
} PACK_SUFFIX CLR_PORT_COUNTERS_DATA;

typedef struct _CLR_ALL_PORT_COUNTERS_DATA {
	uint16		select;
	uint16		reserved;
	uint32		reserved2;
} PACK_SUFFIX CLR_ALL_PORT_COUNTERS_DATA;

typedef struct _INTEGRITY_WEIGHTS {
	uint8					symbolErrorCounter;
	uint8					linkErrorRecoveryCounter;
	uint8					linkDownedCounter;
	uint8					portRcvErrors;
	uint8					localLinkIntegrityErrors;
	uint8					excessiveBufferOverrunErrors;
	uint16					reserved;
} PACK_SUFFIX INTEGRITY_WEIGHTS_T;

typedef struct _CONGESTION_WEIGHTS {
	uint16					portXmitDiscard;
	uint16					portXmitCongestionPct10;
	uint16					portXmitInefficiencyPct10;
	uint16					portXmitWaitCongestionPct10;
	uint16					portXmitWaitInefficiencyPct10;
	uint16					reserved;
	uint16					reserved2;
	uint16					reserved3;
} PACK_SUFFIX CONGESTION_WEIGHTS_T;

// PM configuration flags for pmFlags below
#define PM_NONE					0
#define PM_ENABLE_64BIT			0x000000001	// enable use of 64 bit counters
#define PM_ENABLE_SW_VENDOR		0x000000004	// enable use of SW vendor counters
#define PM_ENABLE_CA_VENDOR		0x000000008	// enable use of CA vendor counters
#define PM_CA_STATS				0x000000010	// monitor CA PMAs
#define PM_ENABLE_SW_VENDOR2	0x000000020	// enable use of SW vendor counters
#define PM_ENABLE_CA_VENDOR2	0x000000040	// enable use of CA vendor counters

static __inline
void FormatPmFlags(char buf[80], uint32 pmFlags)
{
	snprintf(buf, 80, "%s%s%s%s",
				pmFlags & PM_CA_STATS ? "" : "CaPmaAvoid ",
				pmFlags & PM_ENABLE_64BIT ? "Pma64Enable " : "" ,
				pmFlags & PM_ENABLE_CA_VENDOR ? "PmaCaVendorEnable " : "",
				pmFlags & PM_ENABLE_SW_VENDOR ? "PmaSwVendorEnable" : "");
}

static __inline
void FormatPmFlags2(char buf[80], uint32 pmFlags)
{
	snprintf(buf, 80, "%s%s",
				pmFlags & PM_ENABLE_CA_VENDOR2 ? "PmaCaVendor2Enable " : "",
				pmFlags & PM_ENABLE_SW_VENDOR2 ? "PmaSwVendor2Enable" : "");
}

typedef struct _PM_CONFIG_DATA {
	uint32					sweepInterval;
	uint32					MaxClients;
	uint32					sizeHistory;
	uint32					sizeFreeze;
	uint32					lease;
	uint32					pmFlags;
	CONGESTION_WEIGHTS_T	congestionWeights;
	PMERRTHRESHOLD_T		categoryThresholds;
	INTEGRITY_WEIGHTS_T		integrityWeights;
	uint64					memoryFootprint;
	uint32					MaxAttempts;
	uint32					RespTimeout;
	uint32					MinRespTimeout;
	uint32					MaxParallelNodes;
	uint32					PmaBatchSize;
	uint8					ErrorClear;
	uint8					reserved;
	uint16					reserved2;
	uint32					reserved3;
} PACK_SUFFIX PM_CONFIG_DATA;

// TBD is value2 always guid?
typedef struct _FOCUS_PORT_ENTRY {
	uint16					nodeLid;
	uint8					portNumber;
	IB_BITFIELD2(uint8,
							rate:5,	// IB_STATIC_RATE
							mtu:3);	// enum IB_MTU
	uint32					index;
	uint64					value;		// list sorting factor
	uint64					value2;		// good place for GUID
	char					nodeDesc[64]; // can be 64 char w/o \0
	uint16					neighborLid;
	uint8					neighborPortNumber;
	uint8					reserved2;
	uint32					reserved3;
	uint64					neighborValue;
	uint64					neighborGuid;
	char					neighborNodeDesc[64]; // can be 64 char w/o \0
} PACK_SUFFIX FOCUS_PORT_ENTRY;

typedef struct _FOCUS_PORTS_DATA {
	char					groupName[PM_GROUPNAMELEN];	// \0 terminated
	IMAGE_ID_DATA			imageId;
	uint32					select;
	uint32					start;
	uint32					range;
	uint32					numPorts;
	FOCUS_PORT_ENTRY		portList[1];
} PACK_SUFFIX FOCUS_PORTS_DATA;

// TBD - is this still used
typedef struct _FOCUS_PORTS_DATA2 {
	char					groupName[PM_GROUPNAMELEN];	// \0 terminated
	IMAGE_ID_DATA			imageId;
	uint32					select;
	uint32					start;
	uint32					range;
	uint32					numPorts;
	FOCUS_PORT_ENTRY		portList[500];				// need this to hold lots of ports in the response
} PACK_SUFFIX FOCUS_PORTS_DATA2;

typedef struct _SMINFO_DATA {
	uint16					lid;
#if CPU_LE
	uint8					priority:4;
	uint8					state:4;
#else
	uint8					state:4;
	uint8					priority:4;
#endif
	uint8					portNumber;
	uint32					reserved;
	uint64					smPortGuid;
	char					smNodeDesc[64]; // can be 64 char w/o \0
} PACK_SUFFIX SMINFO_DATA;

typedef struct _IMAGE_INFO_DATA {
	IMAGE_ID_DATA			imageId;
	uint64					sweepStart;
	uint32					sweepDuration;
	uint16					numHFIPorts;
	uint16					numTFIPorts;
	uint16					numRTRPorts;
	uint16					numSwitchNodes;
	uint32					numSwitchPorts;
	uint32					numLinks;
	uint32					numSMs;
	uint32					numNoRespNodes;
	uint32					numNoRespPorts;
	uint32					numSkippedNodes;
	uint32					numSkippedPorts;
	uint32					numUnexpectedClearPorts;
	uint32					reserved;
	SMINFO_DATA				SMInfo[2];
} PACK_SUFFIX IMAGE_INFO_DATA;

typedef struct _MOVE_FREEZE_DATA {
	IMAGE_ID_DATA			oldFreezeImage;
	IMAGE_ID_DATA			newFreezeImage;
} PACK_SUFFIX MOVE_FREEZE_DATA;

/* End of packed data structures */
#include "iba/public/ipackoff.h"



/* Performance Analysis methods */
#define PA_CMD_GET                (0x01)
#define PA_CMD_GET_RESP           (0x81)
#define PA_CMD_GETTABLE           (0x12)
#define PA_CMD_GETTABLE_RESP      (0x92)


#define	MAD_PA_REPLY              0x80		// Reply bit for methods

/* Performance Analysis attribute IDs */

#define PA_ATTRID_GET_CLASSPORTINFO	0x01
#define PA_ATTRID_GET_GRP_LIST		0xA0
#define PA_ATTRID_GET_GRP_INFO		0xA1
#define PA_ATTRID_GET_GRP_CFG		0xA2
#define PA_ATTRID_GET_PORT_CTRS		0xA3
#define PA_ATTRID_CLR_PORT_CTRS		0xA4
#define PA_ATTRID_CLR_ALL_PORT_CTRS	0xA5
#define PA_ATTRID_GET_PM_CONFIG		0xA6
#define PA_ATTRID_FREEZE_IMAGE		0xA7
#define PA_ATTRID_RELEASE_IMAGE		0xA8
#define PA_ATTRID_RENEW_IMAGE		0xA9
#define PA_ATTRID_GET_FOCUS_PORTS	0xAA
#define PA_ATTRID_GET_IMAGE_INFO	0xAB
#define PA_ATTRID_MOVE_FREEZE_FRAME	0xAC

/* Performance Analysis MAD status values */

#define MAD_STATUS_PA_UNAVAILABLE	0x0A00  // Engine unavailable
#define MAD_STATUS_PA_NO_GROUP		0x0B00  // No such group
#define MAD_STATUS_PA_NO_PORT		0x0C00  // Port not found

/* PM Service Record values */

#define PM_SERVICE_NAME		"Primary Intel OmniPath Performance Manager"
#define PM_SERVICE_ID		(0x1100d03c34845555ull)
#define PM_VERSION		0x01
#define PM_MASTER		0x01		/* master state */
#define PM_STANDBY		0x02		/* standby state */

#define PA_DATA_OFFSET	32

#define PA_CLASS_VERSION	0x01

/* Selection values for focus */

#define PA_SELECT_UTIL_HIGH					0x00020001		// highest first, descending
#define PA_SELECT_UTIL_PKTS_HIGH			0x00020081
#define PA_SELECT_UTIL_LOW					0x00020101		// lowest first, ascending
#define PA_SELECT_ERR_INTEG					0x00030001		// highest first, descending
#define PA_SELECT_ERR_SMACONG				0x00030002
#define PA_SELECT_ERR_CONG					0x00030003
#define PA_SELECT_ERR_SEC					0x00030004
#define PA_SELECT_ERR_ROUT					0x00030005
#define PA_SELECT_ERR_ADPTV_ROUT			0x00030006

#define IS_VALID_SELECT(s)					(((s) == PA_SELECT_UTIL_HIGH) || ((s) == PA_SELECT_UTIL_PKTS_HIGH) || ((s) == PA_SELECT_UTIL_LOW) || ((s) == PA_SELECT_ERR_INTEG) ||\
											((s) == PA_SELECT_ERR_SMACONG) || ((s) == PA_SELECT_ERR_CONG) || ((s) == PA_SELECT_ERR_SEC) ||  ((s) == PA_SELECT_ERR_ROUT) ||\
											((s) == PA_SELECT_ERR_ADPTV_ROUT))

/* Performance Analysis Response Structures */
#define PA_IMAGE_ID_NSIZE					sizeof(IMAGE_ID_DATA)
#define PA_REC_DESC_LEN						64
#define PA_TABLE_REC_DATA_LEN				512
#define PA_RECORD_NSIZE						sizeof(PaRecord_t)
#define PA_TABLE_RECORD_NSIZE				sizeof(PaTableRecord_t)
#define PA_GROUP_LIST_NSIZE					sizeof(GROUP_LIST_DATA)
#define PA_GROUP_INFO_NSIZE			sizeof(GROUP_INFO_DATA)
#define PA_GROUP_CONFIG_NSIZE				sizeof(GROUP_CONFIG_DATA)
#define PA_PORT_COUNTERS_NSIZE				sizeof(PORT_COUNTERS_DATA)
#define PA_CLR_PORT_COUNTERS_NSIZE			sizeof(CLR_PORT_COUNTERS_DATA)
#define PA_CLR_ALL_PORT_COUNTERS_NSIZE		sizeof(CLR_ALL_PORT_COUNTERS_DATA)
#define PA_PM_CONFIG_NSIZE					sizeof(PM_CONFIG_DATA)
#define PA_FOCUS_PORTS_NSIZE				sizeof(FOCUS_PORTS_DATA)
#define PA_IMAGE_INFO_NSIZE				sizeof(IMAGE_INFO_DATA)
#define PA_MOVE_FREEZE_NSIZE				sizeof(MOVE_FREEZE_DATA)

typedef struct PaRecord_s {
  uint16_t fieldUint16;  
  uint16_t fieldFiller;  
  uint32_t fieldUint32;
  uint64_t fieldUint64;             
  uint8_t desc[PA_REC_DESC_LEN];
} PaRecord_t;

typedef struct PaTableRecord_s {
    uint16_t fieldUint16;  
    uint16_t fieldFiller;  
    uint32_t fieldUint32;
    uint64_t fieldUint64;             
  uint8_t desc[PA_REC_DESC_LEN];
  uint8_t data[PA_TABLE_REC_DATA_LEN];
} PaTableRecord_t;

typedef struct _PA_TABLE_RECORD_RESULTS  {
    uint32 NumPaRecords;                /* Number of PA Records returned */
    PaTableRecord_t PaRecords[1];		/* list of PA records returned */
} PA_TABLE_RECORD_RESULTS, *PPA_TABLE_RECORD_RESULTS;

typedef struct _PA_GROUP_LIST_RESULTS  {
    uint32 NumGroupListRecords;                /* Number of PA Records returned */
    GROUP_LIST_DATA GroupListRecords[1];		/* list of PA records returned */
} PA_GROUP_LIST_RESULTS, *PPA_GROUP_LIST_RESULTS;

typedef struct _PA_GROUP_INFO_RESULTS  {
    uint32 NumGroupInfoRecords;                /* Number of PA Records returned */
    GROUP_INFO_DATA GroupInfoRecords[1];		/* list of PA records returned */
} PA_GROUP_INFO_RESULTS, *PPA_GROUP_INFO_RESULTS;

typedef struct _PA_GROUP_CONFIG_RESULTS  {
    uint32 NumGroupConfigRecords;                /* Number of PA Records returned */
    GROUP_CONFIG_DATA2 GroupConfigRecords[1];		/* list of PA records returned */
} PA_GROUP_CONFIG_RESULTS, *PPA_GROUP_CONFIG_RESULTS;

typedef struct _PA_PORT_COUNTERS_RESULTS  {
    uint32 NumPortCountersRecords;                /* Number of PA Records returned */
    PORT_COUNTERS_DATA PortCountersRecords[1];		/* list of PA records returned */
} PA_PORT_COUNTERS_RESULTS, *PPA_PORT_COUNTERS_RESULTS;

typedef struct _PA_CLR_PORT_COUNTERS_RESULTS  {
    uint32 NumClrPortCountersRecords;                /* Number of PA Records returned */
    CLR_PORT_COUNTERS_DATA ClrPortCountersRecords[1];		/* list of PA records returned */
} PA_CLR_PORT_COUNTERS_RESULTS, *PPA_CLR_PORT_COUNTERS_RESULTS;

typedef struct _PA_CLR_ALL_PORT_COUNTERS_RESULTS  {
    uint32 NumClrAllPortCountersRecords;                /* Number of PA Records returned */
    CLR_ALL_PORT_COUNTERS_DATA ClrAllPortCountersRecords[1];		/* list of PA records returned */
} PA_CLR_ALL_PORT_COUNTERS_RESULTS, *PPA_CLR_ALL_PORT_COUNTERS_RESULTS;

typedef struct _PA_PM_CONFIG_RESULTS  {
    uint32 NumPmConfigRecords;                /* Number of PA Records returned */
    PM_CONFIG_DATA PmConfigRecords[1];		/* list of PA records returned */
} PA_PM_CONFIG_RESULTS, *PPA_PM_CONFIG_RESULTS;

typedef struct _PA_IMAGE_ID_RESULTS  {
    uint32 NumImageIDRecords;                /* Number of PA Records returned */
    IMAGE_ID_DATA ImageIDRecords[1];		/* list of PA records returned */
} PA_IMAGE_ID_RESULTS, *PPA_IMAGE_ID_RESULTS;

typedef struct _PA_FOCUS_PORTS_RESULTS  {
    uint32 NumFocusPortsRecords;                /* Number of PA Records returned */
    FOCUS_PORTS_DATA2 FocusPortsRecords[1];		/* list of PA records returned */
} PA_FOCUS_PORTS_RESULTS, *PPA_FOCUS_PORTS_RESULTS;

typedef struct _PA_IMAGE_INFO_RESULTS  {
    uint32 NumImageInfoRecords;                /* Number of PA Records returned */
    IMAGE_INFO_DATA ImageInfoRecords[1];		/* list of PA records returned */
} PA_IMAGE_INFO_RESULTS, *PPA_IMAGE_INFO_RESULTS;

typedef struct _PA_MOVE_FREEZE_RESULTS  {
    uint32 NumMoveFreezeRecords;                /* Number of PA Records returned */
    MOVE_FREEZE_DATA MoveFreezeRecords[1];		/* list of PA records returned */
} PA_MOVE_FREEZE_RESULTS, *PPA_MOVE_FREEZE_RESULTS;

static __inline void
BSWAP_PA_RECORD(PaRecord_t  *pRecord)
{
#if CPU_LE
	pRecord->fieldUint16 = ntoh16(pRecord->fieldUint16);
	pRecord->fieldUint32 = ntoh32(pRecord->fieldUint32);
	pRecord->fieldUint64 = ntoh64(pRecord->fieldUint64);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_TABLE_RECORD(PaTableRecord_t  *pRecord)
{
#if CPU_LE
	pRecord->fieldUint16 = ntoh16(pRecord->fieldUint16);
	pRecord->fieldUint32 = ntoh32(pRecord->fieldUint32);
	pRecord->fieldUint64 = ntoh64(pRecord->fieldUint64);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_IMAGE_ID(IMAGE_ID_DATA *pRecord)
{
#if CPU_LE
	pRecord->imageNumber						= ntoh64(pRecord->imageNumber);
	pRecord->imageOffset						= ntoh32(pRecord->imageOffset);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_GROUP_INFO(GROUP_INFO_DATA *pRecord)
{
#if CPU_LE
	int i;
	pRecord->numInternalPorts				= ntoh32(pRecord->numInternalPorts);
	pRecord->numExternalPorts				= ntoh32(pRecord->numExternalPorts);

	pRecord->internalUtilStats.totalMBps	= ntoh64(pRecord->internalUtilStats.totalMBps);
	pRecord->internalUtilStats.totalKPps	= ntoh64(pRecord->internalUtilStats.totalKPps);
	pRecord->internalUtilStats.avgMBps		= ntoh32(pRecord->internalUtilStats.avgMBps);
	pRecord->internalUtilStats.minMBps		= ntoh32(pRecord->internalUtilStats.minMBps);
	pRecord->internalUtilStats.maxMBps		= ntoh32(pRecord->internalUtilStats.maxMBps);
	pRecord->internalUtilStats.numBWBuckets		= ntoh32(pRecord->internalUtilStats.numBWBuckets);
	for (i = 0; i < PM_UTIL_BUCKETS; i++)
		pRecord->internalUtilStats.BWBuckets[i] = ntoh32(pRecord->internalUtilStats.BWBuckets[i]);
	pRecord->internalUtilStats.avgKPps		= ntoh32(pRecord->internalUtilStats.avgKPps);
	pRecord->internalUtilStats.minKPps		= ntoh32(pRecord->internalUtilStats.minKPps);
	pRecord->internalUtilStats.maxKPps		= ntoh32(pRecord->internalUtilStats.maxKPps);

	pRecord->sendUtilStats.totalMBps		= ntoh64(pRecord->sendUtilStats.totalMBps);
	pRecord->sendUtilStats.totalKPps		= ntoh64(pRecord->sendUtilStats.totalKPps);
	pRecord->sendUtilStats.avgMBps			= ntoh32(pRecord->sendUtilStats.avgMBps);
	pRecord->sendUtilStats.minMBps			= ntoh32(pRecord->sendUtilStats.minMBps);
	pRecord->sendUtilStats.maxMBps			= ntoh32(pRecord->sendUtilStats.maxMBps);
	pRecord->sendUtilStats.numBWBuckets		= ntoh32(pRecord->sendUtilStats.numBWBuckets);
	for (i = 0; i < PM_UTIL_BUCKETS; i++)
		pRecord->sendUtilStats.BWBuckets[i] = ntoh32(pRecord->sendUtilStats.BWBuckets[i]);
	pRecord->sendUtilStats.avgKPps			= ntoh32(pRecord->sendUtilStats.avgKPps);
	pRecord->sendUtilStats.minKPps			= ntoh32(pRecord->sendUtilStats.minKPps);
	pRecord->sendUtilStats.maxKPps			= ntoh32(pRecord->sendUtilStats.maxKPps);

	pRecord->recvUtilStats.totalMBps		= ntoh64(pRecord->recvUtilStats.totalMBps);
	pRecord->recvUtilStats.totalKPps		= ntoh64(pRecord->recvUtilStats.totalKPps);
	pRecord->recvUtilStats.avgMBps			= ntoh32(pRecord->recvUtilStats.avgMBps);
	pRecord->recvUtilStats.minMBps			= ntoh32(pRecord->recvUtilStats.minMBps);
	pRecord->recvUtilStats.maxMBps			= ntoh32(pRecord->recvUtilStats.maxMBps);
	pRecord->recvUtilStats.numBWBuckets		= ntoh32(pRecord->recvUtilStats.numBWBuckets);
	for (i = 0; i < PM_UTIL_BUCKETS; i++)
		pRecord->recvUtilStats.BWBuckets[i] = ntoh32(pRecord->recvUtilStats.BWBuckets[i]);
	pRecord->recvUtilStats.avgKPps			= ntoh32(pRecord->recvUtilStats.avgKPps);
	pRecord->recvUtilStats.minKPps			= ntoh32(pRecord->recvUtilStats.minKPps);
	pRecord->recvUtilStats.maxKPps			= ntoh32(pRecord->recvUtilStats.maxKPps);

	pRecord->internalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->internalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.congestion);
	pRecord->internalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->internalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->internalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.securityErrors);
	pRecord->internalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.routingErrors);
	pRecord->internalCategoryStats.categoryMaximums.discard				= ntoh16(pRecord->internalCategoryStats.categoryMaximums.discard);
	//pRecord->internalCategoryStats.categoryMaximums.waitPct10			= ntoh16(pRecord->internalCategoryStats.categoryMaximums.waitPct10);
	pRecord->internalCategoryStats.categoryMaximums.congestionPct10			= ntoh16(pRecord->internalCategoryStats.categoryMaximums.congestionPct10);
	pRecord->internalCategoryStats.categoryMaximums.inefficiencyPct10			= ntoh16(pRecord->internalCategoryStats.categoryMaximums.inefficiencyPct10);
	pRecord->internalCategoryStats.categoryMaximums.adaptiveRouting			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.adaptiveRouting);
	for (i = 0; i < PM_ERR_BUCKETS; i++) {
		pRecord->internalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].integrityErrors);
		pRecord->internalCategoryStats.ports[i].congestion			= ntoh32(pRecord->internalCategoryStats.ports[i].congestion);
		pRecord->internalCategoryStats.ports[i].smaCongestion			= ntoh32(pRecord->internalCategoryStats.ports[i].smaCongestion);
		pRecord->internalCategoryStats.ports[i].securityErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].securityErrors);
		pRecord->internalCategoryStats.ports[i].routingErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].routingErrors);
	}

	pRecord->externalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->externalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.congestion);
	pRecord->externalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->externalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->externalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.securityErrors);
	pRecord->externalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->externalCategoryStats.categoryMaximums.routingErrors);
	pRecord->externalCategoryStats.categoryMaximums.discard				= ntoh16(pRecord->externalCategoryStats.categoryMaximums.discard);
	//pRecord->externalCategoryStats.categoryMaximums.waitPct10			= ntoh16(pRecord->externalCategoryStats.categoryMaximums.waitPct10);
	pRecord->externalCategoryStats.categoryMaximums.congestionPct10			= ntoh16(pRecord->externalCategoryStats.categoryMaximums.congestionPct10);
	pRecord->externalCategoryStats.categoryMaximums.inefficiencyPct10			= ntoh16(pRecord->externalCategoryStats.categoryMaximums.inefficiencyPct10);
	pRecord->externalCategoryStats.categoryMaximums.adaptiveRouting			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.adaptiveRouting);
	for (i = 0; i < PM_ERR_BUCKETS; i++) {
		pRecord->externalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].integrityErrors);
		pRecord->externalCategoryStats.ports[i].congestion			= ntoh32(pRecord->externalCategoryStats.ports[i].congestion);
		pRecord->externalCategoryStats.ports[i].smaCongestion			= ntoh32(pRecord->externalCategoryStats.ports[i].smaCongestion);
		pRecord->externalCategoryStats.ports[i].securityErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].securityErrors);
		pRecord->externalCategoryStats.ports[i].routingErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].routingErrors);
	}
	pRecord->maxInternalMBps				= ntoh32(pRecord->maxInternalMBps);
	pRecord->maxExternalMBps				= ntoh32(pRecord->maxExternalMBps);
	BSWAP_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_GROUP_LIST(GROUP_LIST_DATA *pRecord)
{
#if CPU_LE
	pRecord->numGroups               	= ntoh32(pRecord->numGroups);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_GROUP_CONFIG(GROUP_CONFIG_DATA *pRecord, int toNetwork)
{
#if CPU_LE
	uint32 i;
	if (!toNetwork)
		pRecord->numPorts					= ntoh32(pRecord->numPorts);
	for (i = 0; i < pRecord->numPorts; i++) {
		pRecord->portList[i].nodeLid	= ntoh16(pRecord->portList[i].nodeLid);
		pRecord->portList[i].nodeGuid	= ntoh64(pRecord->portList[i].nodeGuid);
	}
	if (toNetwork)
		pRecord->numPorts					= ntoh32(pRecord->numPorts);
	BSWAP_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_PORT_COUNTERS(PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->portNumber							= ntoh32(pRecord->portNumber);
	pRecord->flags								= ntoh32(pRecord->flags);
	pRecord->SymbolErrorCounter					= ntoh16(pRecord->SymbolErrorCounter);
	pRecord->PortRcvErrors						= ntoh16(pRecord->PortRcvErrors);
	pRecord->PortRcvRemotePhysicalErrors		= ntoh16(pRecord->PortRcvRemotePhysicalErrors);
	pRecord->PortRcvSwitchRelayErrors			= ntoh16(pRecord->PortRcvSwitchRelayErrors);
	pRecord->PortXmitDiscards					= ntoh16(pRecord->PortXmitDiscards);
	pRecord->PortCheckRate						= ntoh16(pRecord->PortCheckRate);
	pRecord->VL15Dropped						= ntoh16(pRecord->VL15Dropped);
	pRecord->PortXmitWait						= ntoh32(pRecord->PortXmitWait);
	pRecord->PortXmitData						= ntoh64(pRecord->PortXmitData);
	pRecord->PortRcvData						= ntoh64(pRecord->PortRcvData);
	pRecord->PortXmitPkts						= ntoh64(pRecord->PortXmitPkts);
	pRecord->PortRcvPkts						= ntoh64(pRecord->PortRcvPkts);
	pRecord->PortXmitCongestion					= ntoh64(pRecord->PortXmitCongestion);
	pRecord->PortAdaptiveRouting				= ntoh64(pRecord->PortAdaptiveRouting);
	pRecord->CongestionPct10				= ntoh16(pRecord->CongestionPct10);
	pRecord->InefficiencyPct10				= ntoh16(pRecord->InefficiencyPct10);
	BSWAP_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_CLR_PORT_COUNTERS(CLR_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->portNumber							= ntoh32(pRecord->portNumber);
	pRecord->select								= ntoh16(pRecord->select);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_CLR_ALL_PORT_COUNTERS(CLR_ALL_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->select								= ntoh16(pRecord->select);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_PM_CONFIG(PM_CONFIG_DATA *pRecord)
{
#if CPU_LE
	pRecord->sweepInterval						= ntoh32(pRecord->sweepInterval);
	pRecord->MaxClients							= ntoh32(pRecord->MaxClients);
	pRecord->sizeHistory						= ntoh32(pRecord->sizeHistory);
	pRecord->sizeFreeze							= ntoh32(pRecord->sizeFreeze);
	pRecord->lease								= ntoh32(pRecord->lease);
	pRecord->pmFlags							= ntoh32(pRecord->pmFlags);
	pRecord->congestionWeights.portXmitDiscard	= ntoh16(pRecord->congestionWeights.portXmitDiscard);
	pRecord->congestionWeights.portXmitCongestionPct10 = ntoh16(pRecord->congestionWeights.portXmitCongestionPct10);
	pRecord->congestionWeights.portXmitInefficiencyPct10 = ntoh16(pRecord->congestionWeights.portXmitInefficiencyPct10);
	pRecord->congestionWeights.portXmitWaitCongestionPct10 = ntoh16(pRecord->congestionWeights.portXmitWaitCongestionPct10);
	pRecord->congestionWeights.portXmitWaitInefficiencyPct10 = ntoh16(pRecord->congestionWeights.portXmitWaitInefficiencyPct10);
	pRecord->categoryThresholds.integrityErrors	= ntoh32(pRecord->categoryThresholds.integrityErrors);
	pRecord->categoryThresholds.congestion	= ntoh32(pRecord->categoryThresholds.congestion);
	pRecord->categoryThresholds.smaCongestion	= ntoh32(pRecord->categoryThresholds.smaCongestion);
	pRecord->categoryThresholds.securityErrors		= ntoh32(pRecord->categoryThresholds.securityErrors);
	pRecord->categoryThresholds.routingErrors		= ntoh32(pRecord->categoryThresholds.routingErrors);
	pRecord->memoryFootprint					= ntoh64(pRecord->memoryFootprint);
	pRecord->MaxAttempts						= ntoh32(pRecord->MaxAttempts);
	pRecord->RespTimeout						= ntoh32(pRecord->RespTimeout);
	pRecord->MinRespTimeout						= ntoh32(pRecord->MinRespTimeout);
	pRecord->MaxParallelNodes					= ntoh32(pRecord->MaxParallelNodes);
	pRecord->PmaBatchSize						= ntoh32(pRecord->PmaBatchSize);
	// ErrorClear is uint8
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_FOCUS_PORTS(FOCUS_PORTS_DATA *pRecord, int toNetwork)
{
#if CPU_LE
	uint32 i;
	pRecord->select						= ntoh32(pRecord->select);
	pRecord->start						= ntoh32(pRecord->start);
	pRecord->range						= ntoh32(pRecord->range);
	if (!toNetwork)
		pRecord->numPorts					= ntoh32(pRecord->numPorts);
	for (i = 0; i < pRecord->numPorts; i++) {
		pRecord->portList[i].nodeLid	= ntoh16(pRecord->portList[i].nodeLid);
		pRecord->portList[i].index		= ntoh32(pRecord->portList[i].index);
		pRecord->portList[i].value		= ntoh64(pRecord->portList[i].value);
		pRecord->portList[i].neighborLid	= ntoh16(pRecord->portList[i].neighborLid);
		pRecord->portList[i].neighborValue		= ntoh64(pRecord->portList[i].neighborValue);
		pRecord->portList[i].value2		= ntoh64(pRecord->portList[i].value2);
		pRecord->portList[i].neighborGuid		= ntoh64(pRecord->portList[i].neighborGuid);
	}
	if (toNetwork)
		pRecord->numPorts					= ntoh32(pRecord->numPorts);
	BSWAP_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_IMAGE_INFO(IMAGE_INFO_DATA *pRecord)
{
#if CPU_LE
	int i;
	BSWAP_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->sweepStart					= ntoh64(pRecord->sweepStart);
	pRecord->sweepDuration				= ntoh32(pRecord->sweepDuration);
	pRecord->numHFIPorts				= ntoh16(pRecord->numHFIPorts);
	pRecord->numTFIPorts				= ntoh16(pRecord->numTFIPorts);
	pRecord->numRTRPorts				= ntoh16(pRecord->numRTRPorts);
	pRecord->numSwitchNodes				= ntoh16(pRecord->numSwitchNodes);
	pRecord->numSwitchPorts				= ntoh32(pRecord->numSwitchPorts);
	pRecord->numLinks					= ntoh32(pRecord->numLinks);
	pRecord->numSMs						= ntoh32(pRecord->numSMs);
	pRecord->numNoRespNodes				= ntoh32(pRecord->numNoRespNodes);
	pRecord->numNoRespPorts				= ntoh32(pRecord->numNoRespPorts);
	pRecord->numSkippedNodes			= ntoh32(pRecord->numSkippedNodes);
	pRecord->numSkippedPorts			= ntoh32(pRecord->numSkippedPorts);
	pRecord->numUnexpectedClearPorts	= ntoh32(pRecord->numUnexpectedClearPorts);
	for (i = 0; i < 2; i++) {
		pRecord->SMInfo[i].lid			= ntoh16(pRecord->SMInfo[i].lid);
		pRecord->SMInfo[i].smPortGuid	= ntoh64(pRecord->SMInfo[i].smPortGuid);
	}
#endif /* CPU_LE */
}

static __inline void
BSWAP_PA_MOVE_FREEZE(MOVE_FREEZE_DATA *pRecord)
{
#if CPU_LE
	BSWAP_PA_IMAGE_ID(&pRecord->oldFreezeImage);
	BSWAP_PA_IMAGE_ID(&pRecord->newFreezeImage);
#endif /* CPU_LE */
}

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_PA_H_ */
