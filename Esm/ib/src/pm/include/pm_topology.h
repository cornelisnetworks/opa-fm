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

/* [ICS VERSION STRING: unknown] */

#ifndef _PM_TOPOLOGY_H
#define _PM_TOPOLOGY_H

#include "sm_l.h"
#include "pm_l.h"
#include <iba/ibt.h>
#include <iba/ipublic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#define _GNU_SOURCE
#include <iba/ib_mad.h>
#include <iba/stl_pm.h>
#include <iba/stl_pa_priv.h>
#include <iba/public/ispinlock.h>	// for ATOMIC_UINT
#include <iba/public/iquickmap.h>	// for cl_qmap_t
#include <limits.h>
#include "cs_context.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iba/public/ipackon.h"

// if 1, we compress groups such no gaps in portImage->groups
//		this speeds up PM sweeps
// if 0, we can have gaps, this speeds up Removing groups
#define PM_COMPRESS_GROUPS 1

		// used to mark unused entries in history and freezeFrame
		// also used in LastSweepIndex to indicate no sweeps done yet
#define PM_IMAGE_INDEX_INVALID 0xffffffff

// Used By Get/Clear Vf PortCounters to Access VL 15 Counters
#define HIDDEN_VL15_VF	"HIDDEN_VL15_VF"
#define PA_ALL_GROUP_NAME "All"

// special ImageId of 0 is used to access live data
// -1 is used to request Images by time 
// other non-zero values are of the format below
// This is an opaque format, the only user known ImageIds are 0 to access
// live data and -1 (0xffffffffffffffff) for images by time
#define IMAGEID_LIVE_DATA			 0	// 64 bit ImageId to access live data
#define IMAGEID_ABSOLUTE_TIME		-1	// 64 bit ImageID to request image by time

// values for ImageId.s.type field, used to determine which table to look in, or
// to determine if an image came from disk.
#define IMAGEID_TYPE_ANY			0	// Matches any image ID type
#define IMAGEID_TYPE_FREEZE_FRAME	1	// client requested Freeze Frame
#define IMAGEID_TYPE_HISTORY		2	// last sweep and recent history
#define IMAGEID_TYPE_HISTORY_DISK	3	// Recent history *disk only*

#define IMAGEID_MAX_INSTANCE_ID		256	// 8 bit field
										  
typedef union {
	uint64_t	AsReg64;
	struct {
		// this is opaque so bitt order doesn't matter, but we use IB_BITFIELD
		// so its more readable when displayed as a uint64 in debug logging
		IB_BITFIELD5(uint64,
			type:2,		// type of image
			clientId:6,	// bit number of client within Freeze Ref Count
			sweepNum:32,	// NumSweeps to provide uniqueness
			instanceId:8,	// instanceId ot provide uniqueness between PM instances
			index:16		// look aside index
		)
	} s;
} ImageId_t;

// TBD - if we malloc Pm_t.Groups[], maybe number of groups could be dynamic
#define PM_MAX_GROUPS 10	// max user configured groups
#define PM_MAX_GROUPS_PER_PORT 4	// we keep this small to bound compute needs
			// 4 groups plus the All group gives max of 5 groups per port
			// IntLinkFlags must be at least this many bits, presently 8 bits
			// and portImage->numGroups must be able to hold this value

// how much beyond maxLid to allocate to allow for growth without realloc
#define PM_LID_MAP_SPARE	512
// how much below maxLid to trigger free
#define PM_LID_MAP_FREE_THRESHOLD	1024
// TBD - pre-size based on subnet size?  Or perhaps have above be a function
// of subnet size?

#ifdef __VXWORKS__
/* Use 1000 to match HSM's define for MAX_VABRICS which is a redefinition
 * of MAX_ENABLED_VFABRICS. MAX_ENABLED_VFABRICS is defined in fm_xml.h.
 */
#define MAX_PM_COMP_VFABRICS 1000
#else /* __VXWORKS__ */
#define MAX_PM_COMP_VFABRICS MAX_VFABRICS
#endif /* __VXWORKS__ */

extern uint32_t g_pmDebugPerf;

// This is a consolidation of the counters of interest from PortStatus
// We use the same datatypes for each counter (hence same range) as in PMA
typedef struct PmCompositePortCounters_s {
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

	uint8	Reserved2[30];

} PmCompositePortCounters_t;

typedef struct _vls_pctrs PmCompositeVLCounters_t;


typedef struct PmCompositeVfvlmap_s {
	uint32	vlmask;
	uint32  VF; //index into vf array
} PmCompositeVfvlmap_t;

#define UPDATE_MAX(max, cnt) do { if (cnt > max) max = cnt; } while (0)
#define UPDATE_MIN(min, cnt) do { if (cnt < min) min = cnt; } while (0)


// for tracking Bandwidth utilization, we use MB/s in uint32 containers
// for reference the maximum theoretical MB/s is as follows:
// where MB = 1024*1024 Bytes
// Max MBps 1x SDR=238
// Max MBps 4x SDR=953
// Max MBps 4x DDR=1907
// Max MBps 4x QDR=3814
// Max MBps 8x QDR=7629
// Max MBps 8x EDR=15258
// Max MBps 8x HDR=30516
// Max MBps 12x HDR=45768

// for tracking packet rate, we use Kilo packet/s units in uint32 containers
// where KP = 1024 packets
// Max KPps 1x SDR=8704
// Max KPps 4x SDR=34852
// Max KPps 4x DDR=69741
// Max KPps 4x QDR=139483
// Max KPps 8x QDR=279003
// Max KPps 8x EDR=558006
// Max KPps 8x HDR=1116013
// Max KPps 12x HDR=1673801

// number of errors of each "error class" per interval (NOT per second).
// tracked per "half link".  Problem is associated with direction
// having problem, we associate count with "destination" port although
// both sides can be partial causes.
// counters are same size as PMA(PortCounters) since beyond that
// PMA will peg counter for given analysis interval
typedef struct ErrorSummary_s {
	uint32 Integrity;
	uint32 Congestion;
	uint32 SmaCongestion;
	uint32 Bubble;
	uint32 Security;
	uint32 Routing;

	uint16 UtilizationPct10;        	/* in units of 10% */
	uint16 DiscardsPct10;           	/* in units of 10% */
	uint32 Reserved;
} PACK_SUFFIX ErrorSummary_t;

// weight to use for each Integrity counter in weighted sum
typedef struct IntegrityWeights_s {
	uint8 LocalLinkIntegrityErrors;
	uint8 PortRcvErrors;
	uint8 ExcessiveBufferOverruns;
	uint8 LinkErrorRecovery;
	uint8 LinkDowned;
	uint8 UncorrectableErrors;
	uint8 FMConfigErrors;
	uint8 LinkQualityIndicator;
	uint8 LinkWidthDowngrade;
} IntegrityWeights_t;

// weight to use for each Congestion counter in weighted sum
typedef struct CongestionWeights_s {
	uint8 PortXmitWait;
	uint8 SwPortCongestion;
	uint8 PortRcvFECN;
	uint8 PortRcvBECN;
	uint8 PortXmitTimeCong;
	uint8 PortMarkFECN;
} CongestionWeights_t;

// this type counts number of ports in given "% bucket" of util/errors
// for a 20K node fabric with 4 FBB tiers, we can have 60K links with 120K ports
// hence we need a uint32
typedef uint32 pm_bucket_t;

// number of ports in this bucket for each class of errors
// error class association to PMA Counters is same as in ErrorSummary_t
// determination of % (to select bucket) is based on configured threshold
typedef struct ErrorBucket_s {
	pm_bucket_t Integrity;
	pm_bucket_t Congestion;
	pm_bucket_t SmaCongestion;
	pm_bucket_t Bubble;
	pm_bucket_t Security;
	pm_bucket_t Routing;
} PACK_SUFFIX ErrorBucket_t;

// summary of utilization statistics for a group of ports
typedef struct PmUtilStats_s {
	// internal intermediate data
	// TBD - might be useful to report for Ext of groups like SWs and HFIs
	uint64 TotMBps;	// Total of MBps of all selected ports, used to compute Avg
	uint64 TotKPps;	// Total of KPps of all selected ports, used to compute Avg

	// bandwidth
	uint32 AvgMBps;	// average MB per second of all selected ports
	uint32 MinMBps;	// minimum MB per second of all selected ports
	uint32 MaxMBps;	// maximum MB per second of all selected ports

	// Counter below counts number of ports within given % of BW utilization
	pm_bucket_t BwPorts[STL_PM_UTIL_BUCKETS];

	// packets/sec tracking
	uint32 AvgKPps;	// average kilo packets/sec of all selected ports
	uint32 MinKPps;	// minimum kilo packets/sec of all selected ports
	uint32 MaxKPps;	// maximum kilo packets/sec of all selected ports

	uint16 pmaNoRespPorts;  // Number of ports with failures but were still able
							// to be included in Group/Vf Stats
	uint16 topoIncompPorts; // Number of ports with failures that were not able
							// to be included in Group/Vf Stats
	// buckets for packets/sec % don't make much sense since theroretical
	// limit is a function of packet size, hence confusing to report

	uint32 reserved;

} PACK_SUFFIX PmUtilStats_t;

// summary of error statistics for a group of ports
typedef struct PmErrStats_s {
	// For between-group stats, we take Max of us and our neighbor
	// In context of Errors, Avg and Min is of limited value, hopefully
	// very few ports have errors so Avg would be low and Min would be 0
	// hence we only track Max
	ErrorSummary_t Max;	// maximum of each count for all selected ports

	// Number of "half-links"/ports exceeding threshold
	// for between-group buckets, we count one using the worst port in link
	// for in-group we count one for each port in group
	// buckets are based on % of configured threshold,
	// last bucket is for >=100% of threshold
	ErrorBucket_t Ports[STL_PM_CATEGORY_BUCKETS];// in group
} PACK_SUFFIX PmErrStats_t;

struct PmPort_s;
typedef boolean (*PmComparePortFunc_t)(struct PmPort_s *pmportp, char *groupName);

// a group is a set of ports.  A given link can be:
// 	in-group - both ports are within the same group
// 	between-group - one port is in and one port is outside
// 		in which case we talk about Send/Recv direction relative to group
// This allows customers to monitor traffic across selected links (such as
// to/from storage) by putting only 1 port of link in a given group
//
// For error statistics, root cause is less obvious, so when going between-group
// we consider an error on either side of the link as an error associated with
// the External Errors
//
// Should be able to fit in a single MAD all the Internal Stats
// 		(Ports, Util, Errors) 168 bytes
// On external stats
// 		(Ports, SendUtil, RecvUtil, Errors) 232 bytes
typedef struct PmGroup_s {
	// configuration  - unchanging, no lock needed
	char Name[STL_PM_GROUPNAMELEN];	// \0 terminated
	// function to decide if new ports in topology should be added to group
	PmComparePortFunc_t ComparePortFunc;
} PmGroup_t;

typedef struct PmGroupImage_s {
	uint32	NumIntPorts;	// # of ports in group for links in-group
	uint32	NumExtPorts;	// # of ports in group for links between-group

	// statistics
	PmUtilStats_t IntUtil;	// when both ports in group
	PmUtilStats_t SendUtil;	// send from group to outside
	PmUtilStats_t RecvUtil;	// recv by group from outside

// TBD better wording, don't want customer to confuse Internal to a group
// with Internal to a chassis
	// for Internal (in-group) we count one each port (both are in group)
	// for External (between-group), we count worst of our port and its neighbor
	PmErrStats_t IntErr;// in group
	PmErrStats_t ExtErr;// between groups
	uint8	MinIntRate;
	uint8	MaxIntRate;
	uint8	MinExtRate;
	uint8	MaxExtRate;
	uint32	padding;	// for alignment
} PmGroupImage_t;

typedef struct PmVF_s {
	char Name[MAX_VFABRIC_NAME]; // \0 terminated
	uint8 isActive;
} PmVF_t;

typedef	struct PmVFImage_s {
	uint32	NumPorts;		// # of ports in VF

	// statistics
	PmUtilStats_t IntUtil;	// all stats for VF are internal

	PmErrStats_t IntErr;// in VF

	uint8	MinIntRate;
	uint8	MaxIntRate;
} PmVFImage_t;

// for FI, one instance per Active Port
// for Switch, one instance per Switch
// This is not persee a node, but really a lid'ed port
typedef struct PmNode_s {
	ATOMIC_UINT		refCount;
	cl_map_item_t	AllNodesEntry;	// engine use only, key is portGuid

	// these fields do not change and are tracked once for the Node
	Guid_t			NodeGUID;
	Guid_t			SystemImageGUID;
// TBD - track system image guid?
	STL_NODE_DESCRIPTION		nodeDesc;	// we keep latest name, rarely changes
	uint32			changed_count;	// topology_changed_count when last saw node
	uint32			deviceRevision;	// NodeInfo.Device Revision
	union {
		struct PmPort_s **swPorts;	// for switches only
								// sized by numPorts
								// some may be NULL
		struct PmPort_s *caPortp;	// for FI and RTR
								// exactly 1 port per FI tracked per PmNode_t
								// one PmNode_t per active FI port
	} up;

	uint8			nodeType;	// for switches only
	uint8			numPorts;
	// keep latest flags here, they rarely change
	union {
		uint16		AsReg16;
		struct {
			uint16	PmaAvoid:1; 			// node does not have a working PMA or
											//  PM sweeping has been disabled for this Node
			uint16	PmaGotClassPortInfo:1;	// has Pma capabilities been init'ed

			uint16	Reserved:14;			// 14 spare bits

		} s;
	} u;

	// Path Information to talk to Node's PMA
	// we keep latest information here, only used when doing current sweep
	STL_LID			dlid;		// for PMA Redirect
	uint16			pkey;		// for PMA Redirect
	uint32			qpn:24;		// for PMA Redirect
	uint32			sl:4;		// set when update_path
	uint32			qkey;		// for PMA Redirect

	// per Image data protected by Pm.Image[].imageLock
	// must be last in structure so can dynamically size total images in future
	struct PmNodeImage_s {
		// can change per sweep, so track per sweep and can be Freeze Framed
		STL_LID		lid;		// for switch, its lid of port 0
	} Image[1];	// sized when allocate PmNode_t
} PmNode_t;

typedef	struct PmNodeImage_s PmNodeImage_t;

// queryStatus for Port
#define PM_QUERY_STATUS_OK			0x0	// query success (or not yet attempted)
#define PM_QUERY_STATUS_SKIP		0x1	// port skipped, no PMA or filtered
#define PM_QUERY_STATUS_FAIL_QUERY	0x2	// failed to get port counters,
										// path, or classportinfo
#define PM_QUERY_STATUS_FAIL_CLEAR	0x3	// query ok, but failed clear

typedef struct _vfmap {
	uint32 vlmask;
} vfmap_t;

typedef union {
	uint32 AsReg32;
	struct { IB_BITFIELD8(uint32,
		UtilBucket:4,	// MBps utilization bucket: 0 - PM_UTIL_BUCKETS-1
						// Error Buckets (0-PM_ERR_BUCKETS-1)

		IntegrityBucket:3, 		// Integrity
		CongestionBucket:3,		// Congestion
		SmaCongestionBucket:3,	// SMA Congestion
		BubbleBucket:3,			// Bubble
		SecurityBucket:3,		// Security
		RoutingBucket:3,		// Routing
		Reserved:10)
	} s;
} BucketMask_t;

// This tracks Switch, FI and router ports
typedef struct PmPort_s {
	// these fields do not change and are tracked once for the Port
	Guid_t			guid;           // can be 0 for switch portNum != 0
	PmNode_t		*pmnodep;
	uint32			capmask;        // keep latest, rarely changes

	uint8			portNum;
	// keep latest status here, they rarely change
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD2(uint8,
			PmaAvoid:1,				// PM should not sweep PMA on this Port
			Reserved:7)
		} s;
	} u;

	// lid/portnum of neighbor is temp data only used while doing sweep
	STL_LID 		neighbor_lid;
	PORT 			neighbor_portNum;	// only valid if neighbor_lid != 0

	bitset_t dgMember; // Copy of DeviceGroup Memebership from SM

	// count warnings
	uint32 groupWarnings;

	// protected by Pm_t.totalsLock
	PmCompositePortCounters_t StlPortCountersTotal; // running total
	PmCompositeVLCounters_t StlVLPortCountersTotal[MAX_PM_VLS];
	// somehow configure this based on pm_config.process_vl_counters

	// per Image data protected by Pm.Image[].imageLock
	// must be last in structure so can dynamically size total images in future
	struct PmPortImage_s {
		union {
			uint32	AsReg32;
			struct { IB_BITFIELD11(uint32,
				active:1, 			// is port IB_PORT_ACTIVE (SW port 0 fixed up)
				mtu:4,				// enum IB_MTU - due to actual range, 3 bits
				txActiveWidth:4,	// LinkWidthDowngrade.txActive
				rxActiveWidth:4,	// LinkWidthDowngrade.rxActive
				activeSpeed:3,		// LinkSeed.Active
				Initialized:1,		// has group membership been initialized
				queryStatus:2,		// PMA query or clear result
				UnexpectedClear:1,	// PMA Counters unexpectedly cleared
				gotDataCntrs:1,		// Were Data Counters updated
				gotErrorCntrs:1,	// Were Error Counters updated
				Reserved:10)
			} s;
		} u;
		struct PmPort_s	*neighbor;		// Pointer to Neighbor Port

		PmGroup_t 	*Groups[PM_MAX_GROUPS_PER_PORT];	// PortGroups (In additon to All) this port is a member of.
		uint8		numGroups;							// Number of PortGroups
		uint8		InternalBitMask;					// If Port is Internal to PortGroup Bit Mask (this and neighbor in group)

		uint32 		numVFs;                             // Number of VFs
		vfmap_t 	vfvlmap[MAX_VFABRICS];				// VFs this port is a member of.

		uint32_t 	vlSelectMask;                       // Aggreate of Active VLs used by VFs (also VL 15)
		CounterSelectMask_t 		clearSelectMask;	// Counter Mask of Counters Cleared after the above data was recorded

		// Raw PortCounters
		PmCompositePortCounters_t	StlPortCounters;					// Port Level Counters
		PmCompositeVLCounters_t		StlVLPortCounters[MAX_PM_VLS];		// VL Level Counters
		// Delta PortCounters
		PmCompositePortCounters_t	DeltaStlPortCounters;				// Port Level Counters
		PmCompositeVLCounters_t		DeltaStlVLPortCounters[MAX_PM_VLS];	// VL Level Counters


	} Image[1]; // sized when allocate PmPort_t
} PmPort_t;

#define PM_PORT_ERROR_SUMMARY(portImage, lli, ler)	((portImage)->StlPortCounters.PortRcvConstraintErrors + \
											(portImage)->StlPortCounters.PortRcvSwitchRelayErrors + \
											(portImage)->StlPortCounters.PortRcvSwitchRelayErrors + \
											(portImage)->StlPortCounters.PortXmitDiscards         + \
											(portImage)->StlPortCounters.PortXmitConstraintErrors + \
											(portImage)->StlPortCounters.PortRcvRemotePhysicalErrors + \
											((portImage)->StlPortCounters.LocalLinkIntegrityErrors >> (lli?(lli + RES_ADDER_LLI):0)) + \
											(portImage)->StlPortCounters.PortRcvErrors            + \
											(portImage)->StlPortCounters.ExcessiveBufferOverruns  + \
											(portImage)->StlPortCounters.FMConfigErrors           + \
											((portImage)->StlPortCounters.LinkErrorRecovery >> (ler?(ler + RES_ADDER_LER):0)) + \
											(portImage)->StlPortCounters.LinkDowned               + \
											(portImage)->StlPortCounters.UncorrectableErrors)

typedef	struct PmPortImage_s PmPortImage_t;

// FI port or 1st Port of switch
#define pm_node_lided_port(pmnodep) \
		((pmnodep->nodeType == STL_NODE_SW) \
		 	?pmnodep->up.swPorts[0]:pmnodep->up.caPortp)

// Image States
#define PM_IMAGE_INVALID 	0	// uninitialized
#define PM_IMAGE_VALID		1	// valid, available for PA queries
#define PM_IMAGE_INPROGRESS	2	// in process of being swept

// The dispatcher allows the PM to issue multiple requests in parallel
// A DispatcherNode is retained for each Node being queried in parallel
// 	(up to MaxParallelNodes)
// Within each DispatcherNode a list of DispatcherPorts is retained for each
// Port in the node being queries in parallel (up to PmaBatchSize)
typedef enum {
	PM_DISP_PORT_NONE					= 0,
	PM_DISP_PORT_GET_PORTSTATUS			= 1,	// Get(PortStatus) outstanding
	PM_DISP_PORT_GET_PORTCOUNTERS		= 2,	// Get(PortCounters) outstanding
	PM_DISP_PORT_DONE					= 3,	// all processing done for this port
} PmDispPortState_t;

struct PmDispatcherNode_s;

// Return Values for MergePortIntoPacket()
#define PM_DISP_SW_MERGE_DONE       0
#define PM_DISP_SW_MERGE_ERROR      1
#define PM_DISP_SW_MERGE_CONTINUE   2
#define PM_DISP_SW_MERGE_NOMERGE    3

typedef struct PmDispatcherPort_s {
	PmPort_t *pmportp;
    struct PmDispatcherSwitchPort_s *dispNodeSwPort;
	struct PmDispatcherNode_s *dispnode;	// setup once at boot
	PmPortImage_t *pPortImage;
	PmPortImage_t *pPortImagePrev;
} PmDispatcherPort_t;

typedef struct PmDispatcherPacket_s {
	uint64                      PortSelectMask[4];  // Ports in Packet
	uint32                      VLSelectMask;
	uint8                       numPorts;
	uint8                       numVLs;
	struct PmDispatcherNode_s  *dispnode;	        // setup once at boot
	PmDispatcherPort_t         *DispPorts;         
} PmDispatcherPacket_t;

typedef enum {
	PM_DISP_NODE_NONE				= 0,
	PM_DISP_NODE_CLASS_INFO			= 1,	// Get(ClassPortInfo) outstanding
											// Ports[0] has request
	PM_DISP_NODE_GET_DATACOUNTERS	= 2,	// Getting Data Counters for Ports[]
	PM_DISP_NODE_GET_ERRORCOUNTERS	= 3,	// Getting Error Counters for Ports[]
	PM_DISP_NODE_CLR_PORT_STATUS	= 4,	// Clearing Counters for Ports[]
	PM_DISP_NODE_DONE				= 5,	// all processing done for this node
} PmDispNodeState_t;

struct Pm_s;

typedef struct PmDispatcherSwitchPort_s {
	uint8	portNum;
	union {
		uint8	AsReg8;
		struct {
				uint8	IsDispatched:1;		// Port has been dispatched
				uint8	DoNotMerge:1;		// Query failed, retry with out mergeing to isolate port
				uint8	NeedsClear:1;		// Replaces 256-bit mask in Node Struct.
				uint8	NeedsError:1; 
				uint8	Skip:1;				// Any other reason we should skip this packet. 
				uint8	Reserved:3;
		} s;
	} flags;
	uint8	NumVLs;							// Number of active VLs in the Mask

	uint32	VLSelectMask;					// VLSelect Mask associated with port.  
} PmDispatcherSwitchPort_t;

typedef struct PmDispatcherNode_s {
	struct {
		PmNode_t *pmnodep;
		PmDispNodeState_t state;
		union {
			uint8	AsReg8;
			struct {
				uint8	failed:1;
				uint8	redirected:1;	// got PMA redirect response
				uint8	needError:1;	// Summary NeedsError from PmDispatcherSwitchPort_t
				uint8	needClearSome:1;
				uint8	canClearAll:1;
				// 3 spare bits
			} s;
		} u;
		uint32	clearCounterSelect;	                // assumed to be same for all ports
        uint8	numOutstandingPackets;	            // num packets in Dispatcher.Nodes[].Packets
		uint8	numPorts;							// pmnodep structs sometimes wrong; NOW HFI=1 (always) and SW=pmnodep->numPorts+1 to include port 0
        struct  PmDispatcherSwitchPort_s *nextPort; // next port to be dispatched within activePorts
        PmDispatcherSwitchPort_t *activePorts;      // Array of Structures to keep track usefull information relating to a port
	} info;
	struct Pm_s *pm;	                // setup once at boot
	PmDispatcherPacket_t *DispPackets;	// allocated array of PmaBatchSize
} PmDispatcherNode_t;

typedef struct PmImage_s {
	// These fields are protected by Pm.stateLock
	uint8		state;		// Image State
	uint8		nextClientId;// next clientId for FreezeFrame of this image
	uint32		sweepNum;	// NumSweeps when we did this sweep
	uint32 		historyIndex;// history index corresponding to this image
	uint64		ffRefCount;	// 1 bit per FF clientId, indicates image in
							// use by FreezeFrame with given ClientId
							// when 0, no FreezeFrames reference this Image
	time_t		lastUsed;	// timestamp of last reference, used to age FF


	Lock_t		imageLock;	// Lock image data (except state and imageId).
							// also protects Port.Image, Node.Image
							// and Group.Image for given imageIndex

	// for rapid lookup, we index by LID.  < 48K LIDs, so mem size tolerable
	// We dynamic allocate and size based on old_topology.maxLid
	// allocates PM_LID_MAP_SPARE extra when grows and only releases when
	// more than PM_LIB_MAP_FREE_THRESHOLD decrease in maxLid, hence
	// avoiding resizing for minor fabric changes.
// TBD - SM LidMap could similarly use an array for rapid lookup
// and keep lidmap, maxlid, size per sweep
	PmNode_t	**LidMap;
	STL_LID	lidMapSize;	// number of entries allocated in LidMap
	STL_LID	maxLid;

	time_t		sweepStart;	// when started sweep, seconds since 1970
	uint32		sweepDuration;	// in usec
	uint32      imageInterval; // in sec

	// counts of devices found during this sweep
	uint16		HFIPorts;		// count of active HFI ports
// TFI not included in Gen1
//	uint16		TFIPorts;		// count of active TFI ports
	uint16		SwitchNodes;	// count of Switch Nodes
	uint32		SwitchPorts;	// count of Switch Ports (includes Port 0)
	uint32		NumLinks;		// count of links (includes internal)
	uint32		NumSMs;			// count of SMs (including us)
	struct PmSmInfo {
		STL_LID	smLid;			// implies port, 0 if empty record
		uint8	priority:4;		// present priority
		uint8	state:4;		// present state
	} SMs[2];					// track just master and 1st secondary
	// summary of errors during of sweep
								// Nodes = Switch Node or a FI Port
	uint32		NoRespNodes;	// failed to get path or access PMA >=1 port
	uint32		NoRespPorts;	// failed to get path or access PMA
	uint32		SkippedNodes;	// Skipped all ports on Node
	uint32		SkippedPorts;	// No PMA or filtered
	uint32		UnexpectedClearPorts;	// Ports which whose counters decreased
	uint32		DowngradedPorts; // Ports whose Link Width has been downgraded

	// User Configured Groups + HFIs and SWs (All is implied)
	uint32 NumGroups;
	PmGroup_t Groups[PM_MAX_GROUPS];

	uint32 NumVFs;
	uint32 NumVFsActive;
	PmVF_t VFs[MAX_VFABRICS];
} PmImage_t;

// --------------- Short-Term PA History --------------------
//TBD: OPA_VERSION_MAJOR should be moved to a more generic location
#define OPA_VERSION_MAJOR 10
#define PM_HISTORY_VERSION (11 | (OPA_VERSION_MAJOR << 24))
// Old version currently supported by PA
#define PM_HISTORY_VERSION_OLD 10

#define PM_HISTORY_FILENAME_LEN 136		// max length of full filepath
										// MUST BE MULTIPLE OF 8
#define PM_HISTORY_MAX_IMAGES_PER_COMPOSITE 60
#define PM_HISTORY_MAX_SMS_PER_COMPOSITE 2
#define PM_HISTORY_MAX_LOCATION_LEN 111

#define PM_MAX_COMPRESSION_DIVISIONS 32
#define PM_HISTORY_STHFILE_LEN 15 // the exact length of the filename, not full path

typedef struct PmCompositePort_s {
	uint64	guid;
	union {
		uint32	AsReg32;
		struct { IB_BITFIELD11(uint32,
			active:1, 			// is port IB_PORT_ACTIVE (SW port 0 fixed up)
			mtu:4,				// enum IB_MTU - due to actual range, 3 bits
			txActiveWidth:4,	// LinkWidthDowngrade.txActive
			rxActiveWidth:4,	// LinkWidthDowngrade.rxActive
			activeSpeed:3,		// LinkSeed.Active
			Initialized:1,		// has group membership been initialized
			queryStatus:2,		// PMA query or clear result
			UnexpectedClear:1,	// PMA Counters unexpectedly cleared
			gotDataCntrs:1,		// Were Data Counters updated
			gotErrorCntrs:1,	// Were Error Counters updated
			Reserved:10)
		} s;
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

	PmCompositeVfvlmap_t compVfVlmap[MAX_PM_COMP_VFABRICS];

	PmCompositePortCounters_t	stlPortCounters;
	PmCompositeVLCounters_t	stlVLPortCounters[MAX_PM_VLS];
	PmCompositePortCounters_t	DeltaStlPortCounters;
	PmCompositeVLCounters_t	DeltaStlVLPortCounters[MAX_PM_VLS];
} PACK_SUFFIX PmCompositePort_t;

typedef struct PmCompositeNode_s {
	uint64				NodeGUID;
	uint64				SystemImageGUID;
	char				nodeDesc[STL_NODE_DESCRIPTION_ARRAY_SIZE];
	STL_LID 			lid;
	uint8				nodeType;
	uint8				numPorts;

	uint8				Reserved;

	uint8				reserved;
	PmCompositePort_t	**ports;
} PACK_SUFFIX PmCompositeNode_t;

typedef struct PmCompositeVF_s {
	char  name[MAX_VFABRIC_NAME];
	uint8 isActive;
	uint8 reserved[7];
} PACK_SUFFIX PmCompositeVF_t;

typedef struct PmCompositeGroups_s {
	char name[STL_PM_GROUPNAMELEN];
} PACK_SUFFIX PmCompositeGroup_t;

typedef struct PmHistoryHeaderCommon_s {
	uint32	historyVersion;			// Must remain fixed for all versions
	uint32	imageTime;
	char 	filename[PM_HISTORY_FILENAME_LEN];
	uint64	timestamp;
	uint8	isCompressed;
	uint8	reserved2;
	uint16	imagesPerComposite;
	uint32	imageSweepInterval;
	uint64	imageIDs[PM_HISTORY_MAX_IMAGES_PER_COMPOSITE];
} PACK_SUFFIX PmHistoryHeaderCommon_t;

typedef struct PmFileHeader_s {
	PmHistoryHeaderCommon_t common;
	uint64	flatSize;
	uint8	numDivisions;
	uint8	reserved[7];
	uint64	divisionSizes[PM_MAX_COMPRESSION_DIVISIONS];
} PACK_SUFFIX PmFileHeader_t;

typedef struct PmCompositeSmInfo_s {
	STL_LID	smLid;			// implies port, 0 if empty record
#if CPU_BE
	uint8		priority:4;		// present priority
	uint8		state:4;		// present state
#else
	uint8		state:4;
	uint8		priority:4;
#endif
	uint8		reserved[3];
} PACK_SUFFIX PmCompositeSmInfo_t;

typedef struct PmCompositeImage_s {
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
	uint32  reserved3;
	PmCompositeGroup_t  groups[PM_MAX_GROUPS];
	PmCompositeVF_t     VFs[MAX_PM_COMP_VFABRICS];
	PmCompositeNode_t   **nodes;
} PACK_SUFFIX PmCompositeImage_t;

#define INDEX_NOT_IN_USE 0xffffffff
typedef struct PmHistoryRecord_s {
	PmHistoryHeaderCommon_t header;
	uint32	index;
	struct _imageEntry {
		cl_map_item_t	historyImageEntry;	// key is image ID
		uint32 inx;
	} historyImageEntries[PM_HISTORY_MAX_IMAGES_PER_COMPOSITE];
	cl_map_item_t imageTimeEntry;
} PmHistoryRecord_t;

typedef struct _imageEntry PmHistoryImageEntry_t;

typedef struct PmShortTermHistory_s {
	char	filepath[PM_HISTORY_MAX_LOCATION_LEN];
	PmCompositeImage_t	*currentComposite;
	uint8 compositeWritten;
	uint32	currentRecordIndex;
	uint64	totalDiskUsage;
	cl_qmap_t	historyImages;	// map of all short term history Records, keyed by image IDs
	cl_qmap_t   imageTimes;       // map of all short term history images, keyed by start time
	uint32	totalHistoryRecords;
	uint8	currentInstanceId;
	PmCompositeImage_t *cachedComposite;
	struct _loaded_image {	
		PmImage_t *img;
	} LoadedImage;
	char	**invalidFiles; // keeps track of history filenames with a version mismatch
	uint32	oldestInvalid; // index of the oldest invalid file
	PmHistoryRecord_t	**historyRecords;
} PmShortTermHistory_t;

// ----------------------------------------------------------
typedef struct PmDispPerfMap_s {
	uint16 phase_aid;
	uint8  phase_node_type;
	uint8  phase_method;
	size_t phase_offset;
} PmDispPerfMap_t;

typedef struct PmDispatcherPerfPhase_s {
	uint64_t phase_start;
	uint64_t phase_end;
	uint64_t min_roundtrip_time;
	uint64_t max_roundtrip_time;
	uint64_t sum_roundtrip_time;
	uint64_t phase_count;
} PmDispatcherPerfPhase_t;
typedef struct PmDispatcherPerf_s {
	uint64_t callback_calc_time;
	PmDispatcherPerfPhase_t hfi_get_cpi;
	PmDispatcherPerfPhase_t sw_get_cpi;
	PmDispatcherPerfPhase_t hfi_get_cntrs;
	PmDispatcherPerfPhase_t sw_get_data_cntrs;
	PmDispatcherPerfPhase_t sw_get_error_cntrs;
	PmDispatcherPerfPhase_t hfi_clr_cntrs;
	PmDispatcherPerfPhase_t sw_clr_cntrs;
} PmDispatcherPerf_t;

// high level PM configuration and statistics
typedef struct Pm_s {
	ATOMIC_UINT		refCount;	// used to avoid race between engine shutdown
								// and PA client.  Counts number of PA client
								// queries in progress.
	Lock_t			stateLock;	// a RWTHREAD_LOCK.
							// Protects: LastSweepIndex, NumSweeps,
							//      lastHistoryIndex, history[], freezeFrames[]
							// and the following Image[] fields:
							//      state, nextClientId, sweepNum, ffRefCount,
							//      lastUsed, historyIndex
	uint32 LastSweepIndex;	// last completed sweep, see PM_SWEEP_INDEX_INVALID
	uint32 lastHistoryIndex;// history index corresponding to lastSweepIndex
	uint32 NumSweeps;	// total sweeps completed, only written by engine thread

	Lock_t			totalsLock;	// a RWTHREAD_LOCK.
							// Protects: PmPort_t.PortCountersTotal

	// these are look aside buffers to translate from a ImageId to an ImageIndex
	uint32 *history;			// exclusively for HISTORY
	uint32 *freezeFrames;		// exclusively for FREEZE_FRAME

	// configuration settings
	uint32 pmFlags;     // configured (see stl_pa_types.h pmFlags for a list)
	uint16 interval;	// in seconds
	// threshold is per interval NOT per second
	// set threshold to 0 to disable monitoring given Error type
	ErrorSummary_t Thresholds;	// configured
	// set weight to 0 to disable monitoring given Integrity counter
	IntegrityWeights_t integrityWeights;	// configured
	// set weight to 0 to disable monitoring given Congestion counter
	CongestionWeights_t congestionWeights;	// configured

	CounterSelectMask_t clearCounterSelect; 	// private - select all counters
	PmCompositePortCounters_t ClearThresholds;	// configured

	uint16		ErrorClear;		// Pm.ErrorClear config option

	// keep these as scratch area for use by current sweep, not kept per image
	// private to engine thread, not protected by lock
	STL_LID 	pm_slid;	// SLID for packets we send
	uint32		changed_count;	// last pass synchronized topology with SM
	uint32 		SweepIndex;	// sweep in progress, no lock needed
	cl_qmap_t	AllNodes;	// all PmNode_t keyed by portGuid, engine use only

	// these are private to engine, used to hold sizes for various structures
	// to account for the current pm_total_images value being used
	uint32		PmPortSize;	// PmPort_t size
	uint32		PmNodeSize;	// PmNode_t size

	struct PmDispatcher_s {
		generic_cntxt_t cntx;
		PmDispatcherPerf_t perf_stats;
		Event_t sweepDone;
		uint8	postedEvent;			// have we posted the sweepDone event
		STL_LID	nextLid;
		uint16	numOutstandingNodes;	// num nodes in Dispatcher.Nodes
		PmDispatcherNode_t *DispNodes;	// allocated array of PmMaxParallelNodes
	} Dispatcher;

	PmShortTermHistory_t ShortTermHistory;

	// must be last in structure so can dynamically size total images in future
	PmImage_t *Image;
} Pm_t;

typedef struct PmVFFocusPortComputeData_s {
	uint32 imageInterval;
	int vfIdx;
	CongestionWeights_t congestionWeights;
} PmVFFocusPortComputeData_t;

static __inline
void
BSWAP_PM_COMPOSITE_VFVLMAP(PmCompositeVfvlmap_t *Dest, uint32 numVFs)
{
#if CPU_LE
	uint32 i;

	for (i = 0; i < numVFs; i++) {
		Dest[i].vlmask = ntoh32(Dest[i].vlmask);
	}
#endif
}	// End of BSWAP_PM_COMPOSITE_VFVLMAP

static __inline
void
BSWAP_PM_COMPOSITE_PORT_COUNTERS(PmCompositePortCounters_t *Dest)
{
#if CPU_LE
	Dest->VLSelectMask = ntoh32(Dest->VLSelectMask);
	Dest->PortXmitData = ntoh64(Dest->PortXmitData);
	Dest->PortRcvData = ntoh64(Dest->PortRcvData);
	Dest->PortXmitPkts = ntoh64(Dest->PortXmitPkts);
	Dest->PortRcvPkts = ntoh64(Dest->PortRcvPkts);
	Dest->PortMulticastXmitPkts = ntoh64(Dest->PortMulticastXmitPkts);
	Dest->PortMulticastRcvPkts = ntoh64(Dest->PortMulticastRcvPkts);
	Dest->SwPortCongestion = ntoh64(Dest->SwPortCongestion);
	Dest->SwPortCongestion = ntoh64(Dest->SwPortCongestion);
	Dest->PortRcvFECN = ntoh64(Dest->PortRcvFECN);
	Dest->PortRcvBECN = ntoh64(Dest->PortRcvBECN);
	Dest->PortXmitTimeCong = ntoh64(Dest->PortXmitTimeCong);
	Dest->PortXmitWastedBW = ntoh64(Dest->PortXmitWastedBW);
	Dest->PortXmitWaitData = ntoh64(Dest->PortXmitWaitData);
	Dest->PortRcvBubble = ntoh64(Dest->PortRcvBubble);
	Dest->PortMarkFECN = ntoh64(Dest->PortMarkFECN);
	Dest->PortRcvConstraintErrors = ntoh64(Dest->PortRcvConstraintErrors);
	Dest->PortRcvSwitchRelayErrors = ntoh64(Dest->PortRcvSwitchRelayErrors);
	Dest->PortXmitDiscards = ntoh64(Dest->PortXmitDiscards);
	Dest->PortXmitConstraintErrors = ntoh64(Dest->PortXmitConstraintErrors);
	Dest->PortRcvRemotePhysicalErrors = ntoh64(Dest->PortRcvRemotePhysicalErrors);
	Dest->LocalLinkIntegrityErrors = ntoh64(Dest->LocalLinkIntegrityErrors);
	Dest->PortRcvErrors = ntoh64(Dest->PortRcvErrors);
	Dest->ExcessiveBufferOverruns = ntoh64(Dest->ExcessiveBufferOverruns);
	Dest->FMConfigErrors = ntoh64(Dest->FMConfigErrors);
	Dest->LinkErrorRecovery = ntoh32(Dest->LinkErrorRecovery);
	Dest->LinkDowned = ntoh32(Dest->LinkDowned);


#endif
}	// End of BSWAP_PM_COMPOSITE_PORT_COUNTERS

static __inline
void
BSWAP_PM_COMPOSITE_VL_COUNTERS(PmCompositeVLCounters_t *Dest, uint32 numVLs)
{
#if CPU_LE
	uint32 i;

	for (i = 0; i < numVLs; i++) {
		Dest[i].PortVLXmitData = ntoh64(Dest[i].PortVLXmitData);
		Dest[i].PortVLRcvData = ntoh64(Dest[i].PortVLRcvData);
		Dest[i].PortVLXmitPkts = ntoh64(Dest[i].PortVLXmitPkts);
		Dest[i].PortVLRcvPkts = ntoh64(Dest[i].PortVLRcvPkts);
		Dest[i].PortVLXmitWait = ntoh64(Dest[i].PortVLXmitWait);
		Dest[i].SwPortVLCongestion = ntoh64(Dest[i].SwPortVLCongestion);
		Dest[i].PortVLRcvFECN = ntoh64(Dest[i].PortVLRcvFECN);
		Dest[i].PortVLRcvBECN = ntoh64(Dest[i].PortVLRcvBECN);
		Dest[i].PortVLXmitTimeCong = ntoh64(Dest[i].PortVLXmitTimeCong);
		Dest[i].PortVLXmitWastedBW = ntoh64(Dest[i].PortVLXmitWastedBW);
		Dest[i].PortVLXmitWaitData = ntoh64(Dest[i].PortVLXmitWaitData);
		Dest[i].PortVLRcvBubble = ntoh64(Dest[i].PortVLRcvBubble);
		Dest[i].PortVLMarkFECN = ntoh64(Dest[i].PortVLMarkFECN);
		Dest[i].PortVLXmitDiscards = ntoh64(Dest[i].PortVLXmitDiscards);
	}
#endif
}	// End of BSWAP_PM_COMPOSITE_VL_COUNTERS

// Composite Ports are flattened (not array of pointers)
static __inline
void
BSWAP_PM_COMPOSITE_PORT(PmCompositePort_t *Dest, uint32 numPorts)
{
#if CPU_LE
	uint32 i;

	for (i = 0; i < numPorts; i++) {
		Dest[i].guid = ntoh64(Dest[i].guid);

		Dest[i].u.AsReg32 = ntoh32(Dest[i].u.AsReg32);
		Dest[i].neighborLid = ntoh32(Dest[i].neighborLid);

		Dest[i].vlSelectMask = ntoh32(Dest[i].vlSelectMask);
		Dest[i].clearSelectMask.AsReg32 = ntoh32(Dest[i].clearSelectMask.AsReg32);

		BSWAP_PM_COMPOSITE_VFVLMAP(Dest[i].compVfVlmap, MAX_VFABRICS);

		BSWAP_PM_COMPOSITE_PORT_COUNTERS(&Dest[i].stlPortCounters);
		BSWAP_PM_COMPOSITE_VL_COUNTERS(Dest[i].stlVLPortCounters, MAX_PM_VLS);

		BSWAP_PM_COMPOSITE_PORT_COUNTERS(&Dest[i].DeltaStlPortCounters);
		BSWAP_PM_COMPOSITE_VL_COUNTERS(Dest[i].DeltaStlVLPortCounters, MAX_PM_VLS);
	}
#endif
}	// End of BSWAP_PM_COMPOSITE_PORT

// Composite Nodes are flattened (not array of pointers)
static __inline
void
BSWAP_PM_COMPOSITE_NODE(PmCompositeNode_t *Dest, uint32 numNodes)
{
#if CPU_LE
	PmCompositeNode_t *cnode = Dest;
	uint32 i, numPorts;

	for (i = 0; i < numNodes; i++) {
		numPorts = (cnode->nodeType == STL_NODE_SW ? cnode->numPorts+1 : cnode->numPorts);
		cnode->NodeGUID = ntoh64(cnode->NodeGUID);
		cnode->SystemImageGUID = ntoh64(cnode->SystemImageGUID);
		cnode->lid = ntoh32(cnode->lid);
		BSWAP_PM_COMPOSITE_PORT((PmCompositePort_t *)&cnode->ports, numPorts);
		// Calc address of next (flattened) composite node
		cnode = (PmCompositeNode_t *)((size_t)cnode
				+ (sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t **))
				+ (sizeof(PmCompositePort_t) * numPorts));
	}
#endif
}	// End of BSWAP_PM_COMPOSITE_NODE

static __inline
void
BSWAP_PM_COMPOSITE_SM_INFO(PmCompositeSmInfo_t *Dest, uint32 numSMs)
{
#if CPU_LE
	uint32 i;
	for (i = 0; i < numSMs; i++)
		Dest[i].smLid = ntoh32(Dest[i].smLid);
#endif
}	// End of BSWAP_PM_COMPOSITE_SM_INFO

static __inline
void
BSWAP_PM_HISTORY_VERSION(uint32 *Dest)
{
#if CPU_LE
	*Dest = ntoh32(*Dest);
#endif
}   // End of BSWAP_PM_HISTORY_VERSION

static __inline
void
BSWAP_PM_HISTORY_HEADER_COMMON(PmHistoryHeaderCommon_t *Dest)
{
#if CPU_LE
	uint32 i;

	BSWAP_PM_HISTORY_VERSION(&Dest->historyVersion);
	Dest->imageTime = ntoh32(Dest->imageTime);
	Dest->timestamp = ntoh64(Dest->timestamp);
	Dest->imagesPerComposite = ntoh16(Dest->imagesPerComposite);
	Dest->imageSweepInterval = ntoh32(Dest->imageSweepInterval);
	for (i = 0; i < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; i++)
		Dest->imageIDs[i] = ntoh64(Dest->imageIDs[i]);

#endif
}	// End of BSWAP_PM_HISTORY_HEADER_COMMON

static __inline
void
BSWAP_PM_FILE_HEADER(PmFileHeader_t *Dest)
{
#if CPU_LE
	uint32 i;

	BSWAP_PM_HISTORY_HEADER_COMMON(&Dest->common);
	Dest->flatSize = ntoh64(Dest->flatSize);
	for (i = 0; i < PM_MAX_COMPRESSION_DIVISIONS; i++)
		Dest->divisionSizes[i] = ntoh64(Dest->divisionSizes[i]);
#endif
}	// End of BSWAP_PM_FILE_HEADER

// Byte-swap flattened Composite Image
static __inline
void
BSWAP_PM_COMPOSITE_IMAGE_FLAT(PmCompositeImage_t *Dest, boolean hton /*, uint32 historyVersion*/)
{
#if CPU_LE
	uint32 numNodes;
	PmCompositeNode_t *cnodes = (PmCompositeNode_t *)&Dest->nodes;

	// Note that header is swapped independently
	if (hton) {
		numNodes = Dest->maxLid + 1;
		Dest->maxLid = ntoh32(Dest->maxLid);
	} else {
		Dest->maxLid = ntoh32(Dest->maxLid);
		numNodes = Dest->maxLid + 1;
	}
	Dest->sweepStart = ntoh64(Dest->sweepStart);
	Dest->sweepDuration = ntoh32(Dest->sweepDuration);
	Dest->HFIPorts = ntoh16(Dest->HFIPorts);
	Dest->switchNodes = ntoh16(Dest->switchNodes);
	Dest->switchPorts = ntoh32(Dest->switchPorts);
	Dest->numLinks = ntoh32(Dest->numLinks);
	Dest->numSMs = ntoh32(Dest->numSMs);
	Dest->noRespNodes = ntoh32(Dest->noRespNodes);
	Dest->noRespPorts = ntoh32(Dest->noRespPorts);
	Dest->skippedNodes = ntoh32(Dest->skippedNodes);
	Dest->skippedPorts = ntoh32(Dest->skippedPorts);
	Dest->unexpectedClearPorts = ntoh32(Dest->unexpectedClearPorts);
	Dest->downgradedPorts = ntoh32(Dest->downgradedPorts);
	Dest->numGroups = ntoh32(Dest->numGroups);
	Dest->numVFs = ntoh32(Dest->numVFs);
	Dest->numVFsActive = ntoh32(Dest->numVFsActive);
	Dest->numPorts = ntoh32(Dest->numPorts);
	BSWAP_PM_COMPOSITE_SM_INFO(Dest->SMs, PM_HISTORY_MAX_SMS_PER_COMPOSITE);

	BSWAP_PM_COMPOSITE_NODE(cnodes, numNodes);
#endif
}	// End of BSWAP_PM_COMPOSITE_IMAGE_FLAT

void clearLoadedImage(PmShortTermHistory_t *sth);
size_t computeCompositeSize(void);
FSTATUS decompressAndReassemble(unsigned char *input_data, size_t input_size, uint8 divs, size_t *input_sizes, unsigned char *output_data, size_t output_size);
FSTATUS rebuildComposite(PmCompositeImage_t *cimg, unsigned char *data, uint32 history_version);
void writeImageToBuffer(Pm_t *pm, uint32 histindex, uint8_t isCompressed, uint8_t *buffer, uint32_t *bIndex);
void PmFreeComposite(PmCompositeImage_t *cimg);
FSTATUS PmLoadComposite(Pm_t *pm, PmHistoryRecord_t *record, PmCompositeImage_t **cimg);
FSTATUS PmFreezeComposite(Pm_t *pm, PmHistoryRecord_t *record);
FSTATUS PmFreezeCurrent(Pm_t *pm);
void PmReconstituteVFImage(PmCompositeVF_t *cVF, PmVF_t *pmVFP);
void PmReconstituteGroupImage(PmCompositeGroup_t *cgroup, PmGroup_t *pmGroupP);
PmPort_t *PmReconstitutePortImage(PmImage_t *img, PmCompositePort_t *cport);
PmNode_t *PmReconstituteNodeImage(PmImage_t *img, PmCompositeNode_t *cnode);
PmImage_t *PmReconstituteImage(PmCompositeImage_t *cimg);
FSTATUS PmReconstitute(PmShortTermHistory_t *sth, PmCompositeImage_t *cimg);

// Lock Heirachy (acquire in this order):
// 		SM topology locks
// 		Pm.stateLock
// 		Image.imageLock for freeze frames, (in index order, low to high)
// 		Image.imageLock for sweeps, (in index order, most recent to oldest)
// 		Pm.totalsLock
//
// Pm.stateLock is a rwlock, protects:
//     LastSweepIndex, NumSweeps, lastHistoryIndex, history[], freezeFrames[]
//     and the following Image[] fields:
//         state, nextClientId, sweepNum, ffRefCount, lastUsed, historyIndex
// Note that NumSweeps and LastSweepIndex are only changed by engine thread,
// hence engine thread can safely read it without a lock
// Pm.SweepIndex is for use by engine only, no lock needed
//
// Pm.Image[index].imageLock is a rwlock, protects:
// 	all data in image (including PmPort_t.Image[index], PmNode_t.Image[index]
// 		and Pmgroup_t.Image[index]
// 		except for fields protected by Pm.stateLock
// 	pa_access must have this lock and verify state == VALID
// 	Engine must get this lock in order to update topology or per image stats
//
// Pm.totalsLock is a rwlock, protects:
//     PmPort_t.PortCountersTotal
//
// INPROGRESS state helps avoid clients blocking for long duration once
// engine starts sweep.  It can also be used in ASSERTs as a secondary check
// to make sure clients are accessing valid data.
// Algorithm for stateLock allows client to check state before tring to
// get imageLock.
//
// pa_access query (for lastsweep, history or freeze frame query):
// 	rdlock Pm.stateLock
// 	index= convert image Id using Pm.LastSweepIndex	//copy to local while locked
//  if Pm.Image[index].state != VALID - error
//  		(client should not access a freeze area until gets response)
// 	rdlock Pm.Image[index].imageLock
// 	rwunlock Pm.stateLock
// 	if accessing PortCountersTotal, rdlock Pm.totalsLock (wrlock to clear Total)
// 	analyze data in Pm.Image[index]
// 	if accessed PortCountersTotal, rwunlock Pm.totalsLock
// 	rwunlock Pm.Image[index].imageLock
//
// Engine Sweep
// 	wrlock Pm.stateLock
//  index=Pm.SweepIndex	// engine can access SweepIndex anytime w/o a lock
//  Pm.Image[index].state = INPROGRESS
//  wrlock Pm.Image[index].imageLock	// make sure clients out
//  rwunlock Pm.stateLock - we have in progress flag set
// 	perform sweep - since it is the "active sweep" pa_access should not try to
// 		lock it while we sweep, INPROGRESS also protects it
// 		if alloc or resize lidmap, set to NULLs.
// 			As populate, inc ref count on node
// 		when done building lidmap, if have old lidmap to free, dec ref counts
// 			and free nodes now 0, then free lidmap
//  rwunlock Pm.Image[index].imageLock
// 	wrlock Pm.stateLock
// 	Pm.Image[index].state = VALID
// 	update Pm.lastSweepIndex
// 	rwunlock Pm.stateLock
//
// PA client Freeze Frame (very similar to engine sweeps):
// 	wrlock Pm.stateLock
// 	image = requested input image (must not be a freeze frame)
// 	if Pm.Image[image].state != VALID - error
// 	pick a Pm.freezeFrames[] to use (one with INVALID or already
// 			pointing to image)
// 			while searching, mark as invalid any freezeFrames which are stale
//	pick next unused clientId in Pm.Image, set Image[image].ffRefCount bit
//	Pm.freezeFrames[] = image
// 	rwunlock Pm.stateLock
//
// freeze Frame release:
//  index must specify a freeze frame type image
// 	wrlock Pm.stateLock
//	if Pm.Image[index].state == INVALID or INPROGRESS - error
// 	reset Pm.Image[index].ffRefCount bit for Freeze Frame Client Id
// 	rwunlock Pm.stateLock
//
// shutdown synchronization between PA and Engine
// Pm.refCount counts when PA is in PM, so don't free PM while client is
// still using.
// Engine shutdown:
// 		set not running
// 		wait for refCount to be 0
// 		PmDestroy
// 			if want to be paranoid, could wrlock each image before try to free
// 			that way can be really sure no one is inside the image
// PA client packet processing:
// 		increment Pm refCount
// 		check is running - dec refCount, fail query
// 		do normal processing algorithm:
// 			lock Pm.stateLock
// 			process state
// 			lock imageLock
// 			unlock Pm.stateLock
// 			process image
// 			send response packet
// 			unlock imageLock
// 		dec refCount

//
// PA protocol updates:
// - can specify freeze frame index
// - can specify history index 0 to N
// - bit to indicate if given index is history or freeze frame
// - in sweep summary query, have timestamps, maxLids, etc

#define PM_ENGINE_STOPPED 0
#define PM_ENGINE_STARTED 1
#define PM_ENGINE_STOPPING 2
extern int	g_pmEngineState;

extern boolean g_pmAsyncRcvThreadRunning;
extern Sema_t g_pmAsyncRcvSema;	// indicates AsyncRcvThread is ready
extern IBhandle_t hpma, pm_fd;

#define PM_ALLBITS_SET(select, mask) (((select) & (mask)) == (mask))

// Lookup a node in pmImage based on lid
// caller should have pmImage->imageLock held
PmNode_t *pm_find_node(PmImage_t *pmimagep, STL_LID lid);

// Lookup a port in pmImage based on lid and portNum
// does not have to be a "lid"'ed port
// caller should have pmImage->imageLock held
PmPort_t *pm_find_port(PmImage_t *pmImage, STL_LID lid, uint8 portNum);

// Lookup a node in Pm Topology based on nodeguid
PmNode_t *pm_find_nodeguid(Pm_t *pm, uint64 nodeGUID);

// Clear Running totals for a given Node.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write
FSTATUS PmClearNodeRunningCounters(PmNode_t *pmnodep, CounterSelectMask_t select);
FSTATUS PmClearNodeRunningVFCounters(PmNode_t *pmnodep, STLVlCounterSelectMask select,
	int vfIdx, boolean useHiddenVF);

// in mad_info.c
void PmUpdateNodePmaCapabilities(PmNode_t *pmnodep, Node_t *nodep, boolean ProcessHFICounters);
void PmUpdatePortPmaCapabilities(PmPort_t *pmportp, Port_t *portp);

// pm_mad.c
FSTATUS ProcessPmaClassPortInfo(PmNode_t* pmnodep, STL_CLASS_PORT_INFO *classp);

// pm_dispatch.c
Status_t PmDispatcherInit(Pm_t *pm);
void PmDispatcherDestroy(Pm_t *pm);
FSTATUS PmSweepAllPortCounters(Pm_t *pm);

// pm_async_rcv.c
extern generic_cntxt_t     *pm_async_send_rcv_cntxt;
void pm_async_rcv(uint32_t argc, uint8_t ** argv);
void pm_async_rcv_kill(void);

#define	PM_Filter_Init(FILTERP) {						\
	Filter_Init(FILTERP, 0, 0);						\
										\
	(FILTERP)->active |= MAI_ACT_ADDRINFO; \
	(FILTERP)->active |= MAI_ACT_BASE;					\
	(FILTERP)->active |= MAI_ACT_TYPE;					\
	(FILTERP)->active |= MAI_ACT_DATA;					\
	(FILTERP)->active |= MAI_ACT_DEV;					\
	(FILTERP)->active |= MAI_ACT_PORT;					\
	(FILTERP)->active |= MAI_ACT_QP;					\
	(FILTERP)->active |= MAI_ACT_FMASK;					\
										\
	(FILTERP)->type = MAI_TYPE_EXTERNAL;						\
										\
	(FILTERP)->dev = pm_config.hca;						\
	(FILTERP)->port = (pm_config.port == 0) ? MAI_TYPE_ANY : pm_config.port;		\
	(FILTERP)->qp = 1;							\
}

// pm_sweep.c
void PmClearAllNodes(Pm_t *pm);
void PmSkipPort(Pm_t *pm, PmPort_t *pmportp);
void PmSkipNode(Pm_t *pm, PmNode_t *pmnodep);

void PmFailPort(Pm_t *pm, PmPort_t *pmportp, uint8 queryStatus, uint8 method, uint16 aid);
void PmFailPacket(Pm_t *pm, PmDispatcherPacket_t *disppacket, uint8 queryStatus, uint8 method, uint16 aid);
void PmFailNode(Pm_t *pm, PmNode_t *pmnodep, uint8 queryStatus, uint8 method, uint16 aid);

// pm_debug.c
void DisplayPm(Pm_t *pm);

uint32 computeSendMBps(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeSendKPkts(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeIntegrity(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeSmaCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeBubble(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeSecurity(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeRouting(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeUtilizationPct10(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeDiscardsPct10(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeVFSendMBps(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeVFSendKPkts(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeVFCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeVFBubble(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
uint32 computeVFUtilizationPct10(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);


// Given a MBps transfer rate and a theoretical maxMBps, compute the 
//  utilization bucket number from 0 to PM_UTIL_BUCKETS-1
static __inline uint8 ComputeUtilBucket(uint32 SendMBps, uint32 maxMBps)
{
	if (maxMBps) {
		// directly compute bucket to reduce overflow chances
		uint8 utilBucket = (SendMBps * STL_PM_UTIL_BUCKETS) / maxMBps;
		if (utilBucket >= STL_PM_UTIL_BUCKETS)
			return STL_PM_UTIL_BUCKETS-1;
		else
			return utilBucket;
	} else {
		return 0;
	}
}

// Given a Counter Category Value and a threshold, compute the bucket number
//  from 0 to PM_ERR_BUCKETS-1
static __inline uint8 ComputeErrBucket(uint32 errCnt, uint32 errThreshold)
{
	uint8 errBucket;
	if (! errThreshold) return 0;

	errBucket = (errCnt * (STL_PM_CATEGORY_BUCKETS-1)) / errThreshold;
	if (errBucket >= STL_PM_CATEGORY_BUCKETS)
		 return STL_PM_CATEGORY_BUCKETS-1;
	else
		 return errBucket;
}

void PmPrintExceededPort(PmPort_t *pmportp, uint32 index, const char *statistic, uint32 threshold, uint32 value);
void PmPrintExceededPortDetailsIntegrity(PmPort_t *pmportp, PmPort_t *pmportneighborp, uint32 imageIndex, uint32 lastImageIndex);
void PmPrintExceededPortDetailsCongestion(PmPort_t *pmportp, PmPort_t *pmportneighborp, uint32 imageIndex, uint32 lastImageIndex);
void PmPrintExceededPortDetailsSmaCongestion(PmPort_t *pmportp, PmPort_t *pmportneighborp, uint32 imageIndex, uint32 lastImageIndex);
void PmPrintExceededPortDetailsBubble(PmPort_t *pmportp, PmPort_t *pmportneighborp, uint32 imageIndex, uint32 lastImageIndex);
void PmPrintExceededPortDetailsSecurity(PmPort_t *pmportp, PmPort_t *pmportneighborp, uint32 imageIndex, uint32 lastImageIndex);
void PmPrintExceededPortDetailsRouting(PmPort_t *pmportp, PmPort_t *pmportneighborp, uint32 imageIndex, uint32 lastImageIndex);
void PmFinalizePortStats(Pm_t *pm, PmPort_t *portp, uint32 index);
boolean PmTabulatePort(Pm_t *pm, PmPort_t *portp, uint32 index,
			   			uint32 *counterSelect);
void ClearGroupStats(PmGroupImage_t *groupImage);
void ClearVFStats(PmVFImage_t *vfImage);
void FinalizeGroupStats(PmGroupImage_t *groupImage);
void PmClearPortImage(PmPortImage_t *portImage);
void FinalizeVFStats(PmVFImage_t *vfImage);

uint32_t PmCalculateRate(uint32_t speed, uint32_t width);
void UpdateInGroupStats(Pm_t *pm, uint32 imageIndex, PmPort_t *port, PmGroupImage_t *groupImage, uint32 imageInterval);
void UpdateExtGroupStats(Pm_t *pm, uint32 imageIndex, PmPort_t *port, PmGroupImage_t *groupImage, uint32 imageInterval);
void UpdateVFStats(Pm_t *pm, uint32 imageIndex, PmPort_t *port, PmVFImage_t *vfImage, uint32 imageInterval);

// Clear Running totals for a given Port.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write
extern FSTATUS PmClearPortRunningCounters(PmPort_t *pmportp, CounterSelectMask_t select);
extern FSTATUS PmClearPortRunningVFCounters(PmPort_t *pmportp, STLVlCounterSelectMask select, int vfIdx, boolean useHiddenVF);

// ? PMA Counter control allows interval and auto restart of counters, can remove
// effect of PMA packet delays, etc.  Should we use it?  Does HW support it?

// compute theoretical limits for each rate
//extern void PM_InitLswfToMBps(void);
// ideally should be static, extern due to split of sweep.c and calc.c
uint32 s_StaticRateToMBps[IB_STATIC_RATE_MAX+1];

// This group of functions accept an index into the pmportp->Groups[]
// caller should search for appropriate entry in array to act on
// adds a port to a group. used by PmAddExtPort and PmAddIntPort
void PmAddPortToGroupIndex(PmPortImage_t* portImage, uint32 grpIndex, PmGroup_t *groupp, boolean internal);

// removes a port from a group. used by other higher level routines
void PmRemovePortFromGroupIndex(PmPortImage_t* portImage, uint32 grpIndex, PmGroup_t *groupp, uint8 compress);

boolean PmIsPortInGroup(PmImage_t *pmimagep, PmPortImage_t *portImage,
	int groupIndex, boolean isAllGroup, boolean *isInternal);
boolean PmIsPortInVF(PmImage_t *pmimagep, PmPortImage_t *portImage,
	int vfIndex);

// adds a port to a group where the neighbor of the port WILL NOT be in
// the given group
void PmAddExtPortToGroupIndex(PmPortImage_t* portImage, uint32 grpIndex, PmGroup_t *groupp, uint32 imageIndex);

// removes a port from a group. used by other higher level routines
void PmRemoveExtPortFromGroupIndex(PmPortImage_t* portImage, uint32 grpIndex, PmGroup_t *groupp, uint32 imageIndex, uint8 compress);

// adds a port to a group where the neighbor of the port WILL be in
// the given group
// This DOES NOT add the neighbor.  Caller must do that separately.
void PmAddIntPortToGroupIndex(PmPortImage_t* portImage, uint32 grpIndex, PmGroup_t *groupp, uint32 imageIndex);

// removes a port from a group. used by other higher level routines
void PmRemoveIntPortFromGroupIndex(PmPortImage_t* portImage, uint32 grpIndex, PmGroup_t *groupp, uint32 imageIndex, uint8 compress);

// compute reasonable clearThresholds based on given threshold and weights
// This can be used to initialize clearThreshold and then override just
// a few of the computed defaults in the even user wanted to control just a few
// and default the rest
void PmComputeClearThresholds(PmCompositePortCounters_t *clearThresholds,
							  CounterSelectMask_t *select, uint8 errorClear);

// build counter select to use when clearing counters
void PM_BuildClearCounterSelect(CounterSelectMask_t *select, boolean clearXfer, boolean clear64bit,
								 boolean clear32bit, boolean clear8bit);

//  insert a shortterm history file from the Master PM into the local history filelist
FSTATUS injectHistoryFile(Pm_t *pm, char *filename, uint8_t *buffer, uint32_t filelen);

void PmDispatcherPerfInit(PmDispatcherPerf_t *perf);

#include "iba/public/ipackoff.h"

#ifdef __cplusplus
};
#endif

#endif /* _PM_TOPOLOGY_H */
