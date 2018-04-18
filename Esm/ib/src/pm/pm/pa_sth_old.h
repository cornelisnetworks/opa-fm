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

#include "pm_topology.h"

#include "iba/public/ipackon.h"
typedef struct PmCompositeVfvlmap_old_s {
	uint32 vlmask;
	uint8 VF;           // Expanded to 32bit
	uint8 reserved[3];
} PmCompositeVfvlmap_old_t;

typedef struct PmCompositePortCounters_old_s {
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
		uint8 AsReg8;
		struct {
#if CPU_BE
			uint8 NumLanesDown:4;
			uint8 Reserved:1;
			uint8 LinkQualityIndicator:3;
#else
			uint8 LinkQualityIndicator:3;
			uint8 Reserved:1;
			uint8 NumLanesDown:4;
#endif	// CPU_BE
		} s;
	} lq;
	uint8	Reserved2[6]; /* Expanded to allow future use */
} PmCompositePortCounters_old_t;

typedef struct PmCompositePort_old_s {
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
	uint8   numVFs; /* Swapped with InternalBitMask below to use 32bit */
	uint8	numGroups;
	uint8	groups[PM_MAX_GROUPS_PER_PORT];

	uint8	InternalBitMask; // Swapped with Num VFs Above
	uint8	reserved2[3];
	uint32	vlSelectMask;

	CounterSelectMask_t clearSelectMask;
	uint32 reserved99;

	PmCompositeVfvlmap_old_t compVfVlmap[32]; /* struct chnage & Size array: 32->1000 */

	PmCompositePortCounters_old_t stlPortCounters;
	PmCompositeVLCounters_t	stlVLPortCounters[9]; /* VLs [0-7,15] */
	PmCompositePortCounters_old_t DeltaStlPortCounters;
	PmCompositeVLCounters_t	DeltaStlVLPortCounters[9]; /* VLs [0-7,15] */
} PACK_SUFFIX PmCompositePort_old_t; /* Sizeof Change */

typedef struct PmCompositeNode_old_s {
	uint64	guid; /* Renamed to NodeGuid */
	/* New field: uint64 SystemImageGUID */
	char	nodeDesc[STL_NODE_DESCRIPTION_ARRAY_SIZE];
	uint16	lid; /* Size & Type: STL_LID */
	uint8	nodeType;
	uint8	numPorts;
	uint32	reserved; /* Shrank to 2 bytes */
	PmCompositePort_old_t **ports;
} PACK_SUFFIX PmCompositeNode_old_t;

typedef struct PmCompositeImage_old_s {
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
	uint32	maxLid; /* Type: STL_LID */

	uint32	numPorts;
	struct PmCompositeSmInfo_old {
		uint16	smLid;  /* Size & Type: STL_LID */
#if CPU_BE
		uint8		priority:4;     // present priority
		uint8		state:4;        // present state
#else
		uint8		state:4;
		uint8		priority:4;
#endif
		uint8		reserved; /* Size: reserved[3] */
	} SMs[PM_HISTORY_MAX_SMS_PER_COMPOSITE];
	uint32	reserved3;
	PmCompositeGroup_t	allPortsGroup;
	PmCompositeGroup_t	groups[PM_MAX_GROUPS];
	PmCompositeVF_t		VFs[32]; /* Size array: 32->1000 */
	PmCompositeNode_old_t	**nodes;
} PACK_SUFFIX PmCompositeImage_old_t; /* Sizeof change */

#include "iba/public/ipackoff.h"

size_t PaSthOldCompositePortCountersToCurrent(PmCompositePortCounters_t *curr, uint8_t *buf);
size_t PaSthOldCompositePortToCurrent(PmCompositePort_t *curr, uint8_t *buf);
size_t PaSthOldCompositeNodeToCurrent(PmCompositeNode_t *curr, uint8_t *buf);
size_t PaSthOldCompositeImageToCurrent(PmCompositeImage_t *curr, uint8_t *buf);
