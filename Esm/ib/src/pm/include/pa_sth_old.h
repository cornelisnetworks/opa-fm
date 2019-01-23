/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

#include "pm_topology.h"

#include "iba/public/ipackon.h"

#define MAX_VFABRICS_OLD 32 // MAX_VFABRICS old value

typedef struct PmUtilStats_oldv10_s {
	uint64 TotMBps;
	uint64 TotKPps;
	uint32 AvgMBps;
	uint32 MinMBps;

	uint32 MaxMBps;
	uint32 BwPorts[STL_PM_UTIL_BUCKETS];
	uint32 AvgKPps;

	uint32 MinKPps;
	uint32 MaxKPps;

	uint16 pmaNoRespPorts;
	uint16 topoIncompPorts;
	uint32 reserved;
} PACK_SUFFIX PmUtilStats_oldv10_t;
typedef struct ErrorSummary_oldv10_s {
	uint32 Integrity;
	uint32 Congestion;
	uint32 SmaCongestion;
	uint32 Bubble;
	uint32 Security;
	uint32 Routing;
	uint16 UtilizationPct10;
	uint16 DiscardsPct10;
	uint32 Reserved;
} PACK_SUFFIX ErrorSummary_oldv10_t;
typedef struct ErrorBucket_oldv10_s {
	uint32 Integrity;
	uint32 Congestion;
	uint32 SmaCongestion;
	uint32 Bubble;
	uint32 Security;
	uint32 Routing;
} PACK_SUFFIX ErrorBucket_oldv10_t;
typedef struct PmErrStats_oldv10_s {
	ErrorSummary_oldv10_t Max;
	ErrorBucket_oldv10_t Ports[STL_PM_CATEGORY_BUCKETS];
} PACK_SUFFIX PmErrStats_oldv10_t;

typedef struct PmCompositeVF_oldv10_s {
	char	name[MAX_VFABRIC_NAME];
	uint32	numPorts;
	uint8	isActive;
	uint8	minIntRate;
	uint8	maxIntRate;
	uint8	reserved;
	PmUtilStats_oldv10_t intUtil;
	PmErrStats_oldv10_t  intErr;
} PACK_SUFFIX PmCompositeVF_oldv10_t;

typedef struct PmCompositeGroups_oldv10_s {
	char	name[STL_PM_GROUPNAMELEN];
	uint32	numIntPorts;
	uint32	numExtPorts;
	uint8	minIntRate;
	uint8	maxIntRate;
	uint8	minExtRate;
	uint8	maxExtRate;
	uint32	reserved;
	PmUtilStats_oldv10_t intUtil;
	PmUtilStats_oldv10_t sendUtil;
	PmUtilStats_oldv10_t recvUtil;
	PmErrStats_oldv10_t  intErr;
	PmErrStats_oldv10_t  extErr;
} PACK_SUFFIX PmCompositeGroup_oldv10_t;

typedef struct PmCompositePort_oldv10_s {
	uint64	guid;
	union {
		uint32	AsReg32;
		struct {IB_BITFIELD11(uint32,
				active:1,           // is port IB_PORT_ACTIVE (SW port 0 fixed up)
				mtu:4,              // enum IB_MTU - due to actual range, 3 bits
				txActiveWidth:4,    // LinkWidthDowngrade.txActive
				rxActiveWidth:4,    // LinkWidthDowngrade.rxActive
				activeSpeed:3,      // LinkSeed.Active
				Initialized:1,      // has group membership been initialized
				queryStatus:2,      // PMA query or clear result
				UnexpectedClear:1,  // PMA Counters unexpectedly cleared
				gotDataCntrs:1,     // Were Data Counters updated
				gotErrorCntrs:1,    // Were Error Counters updated
				Reserved:10)
		}
		s;
	} u;
	STL_LID neighborLid;

	PORT	portNum;
	PORT	neighborPort;
	uint8   InternalBitMask;
	uint8	numGroups;
	uint8	groups[PM_MAX_GROUPS_PER_PORT];

	uint32 numVFs;
	uint32 vlSelectMask;

	CounterSelectMask_t clearSelectMask;
	uint32 reserved99;

	PmCompositeVfvlmap_t compVfVlmap[MAX_VFABRICS_OLD];

	PmCompositePortCounters_t	stlPortCounters;
	PmCompositeVLCounters_t	stlVLPortCounters[MAX_PM_VLS];
	PmCompositePortCounters_t	DeltaStlPortCounters;
	PmCompositeVLCounters_t	DeltaStlVLPortCounters[MAX_PM_VLS];
} PACK_SUFFIX PmCompositePort_oldv10_t;

typedef struct PmCompositeNode_oldv10_s {
	uint64				NodeGUID;
	uint64				SystemImageGUID;
	char				nodeDesc[STL_NODE_DESCRIPTION_ARRAY_SIZE];
	STL_LID 			lid;
	uint8				nodeType;
	uint8				numPorts;

	uint8				Reserved;

	uint8				reserved;
	//PmCompositePort_oldv10_t **ports;
} PACK_SUFFIX PmCompositeNode_oldv10_t;

typedef struct PmCompositeImage_oldv10_s {
	PmFileHeader_t	header;
	uint64	sweepStart;
	uint32	sweepDuration;
	uint8	reserved[2];
	uint16	HFIPorts;
	uint16	switchNodes;
	uint16	reserved2;
	uint32	switchPorts;
	uint32	numLinks;
	uint32 	numSMs;
	uint32	noRespNodes;
	uint32	noRespPorts;
	uint32	skippedNodes;
	uint32	skippedPorts;
	uint32	unexpectedClearPorts;
	uint32  downgradedPorts;
	uint32	numGroups;
	uint32	numVFs;
	uint32	numVFsActive;
	STL_LID	maxLid;
	uint32	numPorts;
	PmCompositeSmInfo_t SMs[PM_HISTORY_MAX_SMS_PER_COMPOSITE];
	uint32	reserved3;
	PmCompositeGroup_oldv10_t allPortsGroup;
	PmCompositeGroup_oldv10_t groups[PM_MAX_GROUPS];
	PmCompositeVF_oldv10_t    VFs[MAX_VFABRICS_OLD];
	//PmCompositeNode_t **nodes;
} PACK_SUFFIX PmCompositeImage_oldv10_t;

#include "iba/public/ipackoff.h"

size_t PaSthOldCompositePortToCurrent(PmCompositePort_t *curr, uint8_t *buf);
size_t PaSthOldCompositeNodeToCurrent(PmCompositeNode_t *curr, uint8_t *buf);
size_t PaSthOldCompositeImageToCurrent(PmCompositeImage_t *curr, uint8_t *buf);
