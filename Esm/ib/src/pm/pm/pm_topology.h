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
#include <iba/stl_pa.h>
#include <iba/public/ispinlock.h>	// for ATOMIC_UINT
#include <iba/public/iquickmap.h>	// for cl_qmap_t
#include <limits.h>
#include "cs_context.h"

#ifdef __cplusplus
extern "C" {
#endif

// if 1, we compress groups such no gaps in portImage->groups
//		this speeds up PM sweeps
// if 0, we can have gaps, this speeds up Removing groups
#define PM_COMPRESS_GROUPS 1

		// used to mark unused entries in history and freezeFrame
		// also used in LastSweepIndex to indicate no sweeps done yet
#define PM_IMAGE_INDEX_INVALID 0xffffffff

// Used By Get/Clear Vf PortCounters to Access VL 15 Counters
#define HIDDEN_VL15_VF	"HIDDEN_VL15_VF"

// special ImageId of 0 is used to access live data
// non-zero values are of the format below
// This is an opaque format, the only user known ImageId is 0 to access
// live data
#define IMAGEID_LIVE_DATA			0	// 64 bit ImageId to access live data

// values for ImageId.s.type field, used to determine which table to look in
#define IMAGEID_TYPE_ANY			0	// Matches any image ID type
#define IMAGEID_TYPE_FREEZE_FRAME	1	// client requested Freeze Frame
#define IMAGEID_TYPE_HISTORY		2	// last sweep and recent history

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
			// and portImage->u.s.InGroups must be able to hold this value

// how much beyond maxLid to allocate to allow for growth without realloc
#define PM_LID_MAP_SPARE	512
// how much below maxLid to trigger free
#define PM_LID_MAP_FREE_THRESHOLD	1024
// TBD - pre-size based on subnet size?  Or perhaps have above be a function
// of subnet size?

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
			uint8 Reserved:5;
			uint8 LinkQualityIndicator:3;
		} s;
	} lq;
} PmCompositePortCounters_t;

typedef struct _vls_pctrs PmCompositeVLCounters_t;

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

	uint16 CongInefficiencyPct10;		/* in units of 10% */
	uint16 WaitInefficiencyPct10;   	/* in units of 10% */
	uint16 BubbleInefficiencyPct10; 	/* in units of 10% */
	uint16 DiscardsPct10;           	/* in units of 10% */
	uint16 CongestionDiscardsPct10; 	/* in units of 10% */
	uint16 UtilizationPct10;        	/* in units of 10% to help with context of above */
} ErrorSummary_t;

// weight to use for each Integrity counter in weighted sum
typedef struct IntegrityWeights_s {
	uint8 LocalLinkIntegrityErrors;
	uint8 PortRcvErrors;
	uint8 ExcessiveBufferOverruns;
	uint8 LinkErrorRecovery;
	uint8 LinkDowned;
	uint8 UncorrectableErrors;
	uint8 FMConfigErrors;
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
} ErrorBucket_t;

// we have 10 buckets each covering a 10% range.
// So we can say number of ports with 0-10% utilization, number with 10-20%
// ... number with 90-100%
#define PM_UTIL_GRAN_PERCENT 10	/* granularity of utilization buckets */
#define PM_UTIL_BUCKETS (100/PM_UTIL_GRAN_PERCENT)

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
	pm_bucket_t BwPorts[PM_UTIL_BUCKETS];

	// packets/sec tracking
	uint32 AvgKPps;	// average kilo packets/sec of all selected ports
	uint32 MinKPps;	// minimum kilo packets/sec of all selected ports
	uint32 MaxKPps;	// maximum kilo packets/sec of all selected ports

	// buckets for packets/sec % don't make much sense since theroretical
	// limit is a function of packet size, hence confusing to report
} PmUtilStats_t;

// we have 4 buckets each covering a 25% range and one extra bucket
// So we can say number of ports within 0-24% of threshold, number within 25-50%
// ... number within 75-100% and number exceeding threshold.
#define PM_ERR_GRAN_PERCENT 25	/* granularity of error buckets */
#define PM_ERR_BUCKETS ((100/PM_ERR_GRAN_PERCENT)+1) // extra bucket is for those over threshold

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
	ErrorBucket_t Ports[PM_ERR_BUCKETS];// in group
} PmErrStats_t;

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

	// group per Image data protected by Pm.Image[].imageLock
	// must be last in structure so can dynamically size total images in future
	struct PmGroupImage_s {
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
	} Image[1];	// sized when allocate PmGroup_t
} PmGroup_t;

typedef	struct PmGroupImage_s PmGroupImage_t;

typedef struct PmVF_s {
	// configuration  - unchanging, no lock needed
	char Name[MAX_VFABRIC_NAME];	// \0 terminated

	// VF per Image data protected by Pm.Image[].imageLock
	// must be last in structure so can dynamically size total images in future
	struct PmVFImage_s {
		uint8 	isActive;
		uint32	NumPorts;		// # of ports in VF

		// statistics
		PmUtilStats_t IntUtil;	// all stats for VF are internal

		PmErrStats_t IntErr;// in VF

		uint8	MinIntRate;
		uint8	MaxIntRate;
	} Image[1];	// sized when allocate PmVF_t
} PmVF_t;

typedef	struct PmVFImage_s PmVFImage_t;

// for FI, one instance per Active Port
// for Switch, one instance per Switch
// This is not persee a node, but really a lid'ed port
typedef struct PmNode_s {
	ATOMIC_UINT		refCount;
	cl_map_item_t	AllNodesEntry;	// engine use only, key is portGuid

	// these fields do not change and are tracked once for the Node
	Guid_t			guid;
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
	uint16			dlid;		// for PMA Redirect
	uint16			pkey;		// for PMA Redirect
	uint32			qpn:24;		// for PMA Redirect
	uint32			sl:4;		// set when update_path
	uint32			qkey;		// for PMA Redirect

	// per Image data protected by Pm.Image[].imageLock
	// must be last in structure so can dynamically size total images in future
	struct PmNodeImage_s {
		// can change per sweep, so track per sweep and can be Freeze Framed
		uint16			lid;		// for switch, its lid of port 0
	} Image[1];	// sized when allocate PmNode_t
} PmNode_t;

typedef	struct PmNodeImage_s PmNodeImage_t;

// Counters[].flags for Port
// first two flags are also used for PmNode.Image[].u.s.flags
#define PM_PORT_CLEARED_ALL				0x1	// cleared all counters
#define PM_PORT_CLEARED_SOME			0x2	// cleared all some counters
#define PM_PORT_GOT_DATAPORTCOUNTERS	0x4	// successfully got DataPortCounters
#define PM_PORT_GOT_ERRORPORTCOUNTERS	0x8	// successfully got ErrorPortCounters
#define PM_PORT_NEED_ERRORPORTCOUNTERS	0x80	// need to retrieve ErrorPortCounters

// queryStatus for Port
#define PM_QUERY_STATUS_OK			0x0	// query success (or not yet attempted)
#define PM_QUERY_STATUS_SKIP		0x1	// port skipped, no PMA or filtered
#define PM_QUERY_STATUS_FAIL_QUERY	0x2	// failed to get port counters,
										// path, or classportinfo
#define PM_QUERY_STATUS_FAIL_CLEAR	0x3	// query ok, but failed clear

typedef struct _vfmap {
	char vfName[MAX_VFABRIC_NAME];
	uint32 vl;
} vfmap_t;

// This tracks Switch, FI and router ports
typedef struct PmPort_s {
	// these fields do not change and are tracked once for the Port
	Guid_t			guid;			// can be 0 for switch portNum != 0
	PmNode_t		*pmnodep;
	uint32			capmask;		// keep latest, rarely changes

	uint8			portNum;
	// keep latest status here, they rarely change
	union {
		uint8	AsReg8;
		struct {
			uint8	PmaAvoid:1;				// initialized on create
								// does port have a working PMA
								// 0 for selected IBM eHFI devices
								// 0 for Non-Enhanced Switch Port 0's
			uint8	CountersIndex:1;	// most recent Counters[] index
								// which has valid data
		} s;
	} u;

	// lid/portnum of neighbor is temp data only used while doing sweep
	STL_LID_32 		neighbor_lid;
	PORT 			neighbor_portNum;	// only valid if neighbor_lid != 0

	// count warnings
	uint32 groupWarnings;

	// This is a scratch area during sweep.  Delta is kept
	// in Image and PortCountersTotal is kept below
	// and are the final values of interest.
	// These are the counters as fetched from the hardware during this sweep
	// Needed when computing Delta between sweeps (eg. when clear not used).
	// These are only for use by Engine Thread, not protected by lock
	// we keep last 2 sweeps, CountersIndex selects one to use for this sweep
	// CountersIndex^1 selects one from prev sweep
	struct PmAllPortCounters_s {
        CounterSelectMask_t clearSelectMask;
		uint32 NumSweep;	// sweep this data written during
		uint64 PortErrorCounterSummary;		/* sum of all error counters for port */
		uint8  flags;// PM_PORT_* flags (see above), private
		STL_PORT_STATUS_RSP *PortStatus;
	} Counters[2];

	// We keep interval delta per image and one copy of running totals.
	// Running totals will allow PM to emulate the counters for tools like
	// opareport which want to clear the counters and then get the value
	// over the opareport interval since the clear (which could be hours).
	// Newer tools should instead using the interval delta.
	// Later implementation of a history feature will remove the need for
	// supporting the clear approach, but lets walk before we run
	// For now, IBM will need to continue to use opafabricanalysis daily
	// runs with 1 symbol error per day type thresholds
	// protected by Pm_t.totalsLock

	PmCompositePortCounters_t StlPortCountersTotal;	// running total
	PmCompositeVLCounters_t StlVLPortCountersTotal[MAX_PM_VLS]; // somehow configure this based on pm_config.process_vl_counters

	// per Image data protected by Pm.Image[].imageLock
	// must be last in structure so can dynamically size total images in future
	struct PmPortImage_s {
		union {
			uint32	AsReg32;
			struct {
				// imageLock protects state, rate and mtu
				uint32	active:1;// is port IB_PORT_ACTIVE (SW port 0 fixed up)
				uint32	rate:5;	// IB_STATIC_RATE - due to actual range, 5 bits
				uint32	mtu:4;	// enum IB_MTU - due to actual range, 3 bits
				uint32	bucketComputed:1; // only r/w by engine, no lock
				uint32	Initialized:1;	// has group membership been initialized
				uint32	queryStatus:2;	// PMA query or clear result
				uint32	UnexpectedClear:1;	// PMA Counters unexpectedly cleared
				// From Counters->flags above
				uint32	gotErrorCntrs:1; // Should Always be true for HFI
				uint32	clearSent:1;	// PMA Counters were cleared by the PM
#if PM_COMPRESS_GROUPS
				uint32	InGroups:3;	// number of groups port is a member of
				// 12 spare bits
#else
				// 15 spare bits
#endif
			} s;
		} u;
		struct PmPort_s	*neighbor;

		// set of groups this port is IN
		// in addition all ports are implicitly in the AllPorts group
		PmGroup_t 	*Groups[PM_MAX_GROUPS_PER_PORT];
		uint16_t	dgMember[MAX_DEVGROUPS];
		PmVF_t 		*VFs[MAX_VFABRICS];
		VlVfMap_t 	vlvfmap;
		uint8 		numVFs;
		vfmap_t 	vfvlmap[MAX_VFABRICS];

		// for each group a bit is used to indicate if the given group contains
		// both this port and its neighbor (Internal Link)
		// It will affect how we tabulate statistics in PmGroup_t
		uint32 IntLinkFlags:8;	// one bit per Group
		uint32 UtilBucket:4;// MBps utilization bucket: 0 - PM_UTIL_BUCKETS-1
		// Error Buckets (0-PM_ERR_BUCKETS-1)
		uint32 IntegrityBucket:3;// Integrity Errors
		uint32 CongestionBucket:3;// Congestion Errors
		uint32 SmaCongestionBucket:3; // SMA Congestion Errors
		uint32 BubbleBucket:3; // Bubble Errors
		uint32 SecurityBucket:3; // Security Errors
		uint32 RoutingBucket:3;// Routing Errors
		uint32 spare:2;

		struct _vl_bucket_flagbucket_flags {
			uint32 IntLinkFlags:8;	// not used here
			uint32 UtilBucket:4;// MBps utilization bucket: 0 - PM_UTIL_BUCKETS-1
			// Error Buckets (0-PM_ERR_BUCKETS-1)
			uint32 IntegrityBucket:3;// Integrity Errors
			uint32 CongestionBucket:3;// Congestion Errors
			uint32 SmaCongestionBucket:3; // SMA Congestion Errors
			uint32 BubbleBucket:3; // Bubble Errors
			uint32 SecurityBucket:3; // Security Errors
			uint32 RoutingBucket:3;// Routing Errors
			uint32 spare:2;
		} VLBucketFlags [MAX_PM_VLS];

		// for statistics below, each interval we:
		// 	fetch new PMA counters, compute statistics, discard old counters
		// 	we keep raw counts here, we can compute % on fly as needed
		// only Image[pm->LastSweepIndex] should be accessed by APIs

		// We keep Raw Counters per image
		// Newer tools should use this instead of PortCountersTotal
		PmCompositePortCounters_t	StlPortCounters;	// Port Level Counters
		PmCompositeVLCounters_t 	StlVLPortCounters[MAX_PM_VLS]; // VL Level Counters - used for VFs
        CounterSelectMask_t 		clearSelectMask;	// what counters were cleared by PM after this image was recorded.

		// Use larger of Send-from and Recv-to (Send should be >= Recv)
		// keep our output stats here and look to neighbor for other direction
		uint32 SendMBps;
		uint32 SendKPps;
		uint32 VFSendMBps[MAX_VFABRICS];
		uint32 VFSendKPps[MAX_VFABRICS];
		// can compute Avg pkt size - (BW/pkts) - with rounding errors

		ErrorSummary_t Errors;	// errors associated with our receiver side
							// look at neighbor for other half of picture
		ErrorSummary_t VFErrors[MAX_VFABRICS]; // errors associated with our receiver side
	} Image[1];	// sized when allocate PmPort_t
} PmPort_t;

// macros to select proper Counters[] entry
#define PM_PORT_COUNTERS(pmportp)	((pmportp)->Counters[(pmportp)->u.s.CountersIndex])
#define PM_PORT_LAST_COUNTERS(pmportp)	((pmportp)->Counters[((pmportp)->u.s.CountersIndex^1)&1])

typedef	struct PmPortImage_s PmPortImage_t;
typedef struct PmAllPortCounters_s PmAllPortCounters_t;

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
	PmAllPortCounters_t *pCounters;
} PmDispatcherPort_t;

typedef struct PmDispatcherPacket_s {
	uint64                      PortSelectMask[4];  // Ports in Packet
	uint8                       numPorts;                
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
// RHB temporary work-around - PACK_SUFFIX not defined
#if 0
	STL_FIELDUNION6(flags, 8,
				IsDispatched:1,				// Port has been dispatched
				DoNotMerge:1,				// Query failed, retry with out mergeing to isolate port
				NeedsClear:1,				// Replaces 256-bit mask in Node Struct.
				NeedsError:1, 
				Skip:1,						// Any other reason we should skip this packet. 
				Reserved:3);
#else
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
#endif

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
	STL_LID_32	lidMapSize;	// number of entries allocated in LidMap
	STL_LID_32	maxLid;

	time_t		sweepStart;	// when started sweep, seconds since 1970
	uint32		sweepDuration;	// in usec

	// counts of devices found during this sweep
	uint16		HFIPorts;		// count of active HFI ports
// TFI not included in Gen1
//	uint16		TFIPorts;		// count of active TFI ports
	uint16		SwitchNodes;	// count of Switch Nodes
	uint32		SwitchPorts;	// count of Switch Ports (excludes Port 0)
	uint32		NumLinks;		// count of links (includes internal)
	uint32		NumSMs;			// count of SMs (including us)
	struct PmSmInfo {
		uint16	smLid;			// implies port, 0 if empty record
		uint8	priority:4;		// present priority
		uint8	state:4;		// present state
	} SMs[2];					// track just master and 1st secondary
	// summary of errors during of sweep
								// Nodes = Switch Node or a FI Port
	uint32		FailedNodes;	// failed to get path or access PMA >=1 port
	uint32		FailedPorts;	// failed to get path or access PMA
	uint32		SkippedNodes;	// Skipped all ports on Node
	uint32		SkippedPorts;	// No PMA or filtered
	uint32		UnexpectedClearPorts;	// Ports which whose counters decreased

} PmImage_t;

// --------------- Short-Term PA History --------------------

#define PM_HISTORY_FILENAME_LEN 133
#define PM_HISTORY_MAX_IMAGES_PER_COMPOSITE 60
#define PM_HISTORY_MAX_LOCATION_LEN 111
#define PM_HISTORY_VERSION 1
#define PM_MAX_COMPRESSION_DIVISIONS 32

typedef struct PmCompositePort_s {
	Guid_t	guid;
	uint8	portNum;
	union {
		uint32 AsReg32;
		struct {
			uint32 active:1;
			uint32 rate:5;
			uint32 mtu:4;
			uint32 bucketComputed:1;
			uint32 Initialized:1;
			uint32 queryStatus:2;
			uint32 UnexpectedClear:1;
#if PM_COMPRESS_GROUPS
			uint32 InGroups:2;
#endif
		} s;
	} u;
	STL_LID_32 neighborLid;
	PORT	neighborPort;
	uint8	numVFs;
	uint32	sendMBps;
	uint32	sendKPps;
	uint32	VFSendMBps[MAX_VFABRICS];
	uint32	VFSendKPps[MAX_VFABRICS];
	uint8	groups[PM_MAX_GROUPS_PER_PORT];
	uint8	VFs[MAX_VFABRICS];
	VlVfMap_t vlvfmap;
	vfmap_t	vfvlmap[MAX_VFABRICS];

	uint32 intLinkFlags:8;
	uint32 utilBucket:4;
	uint32 integrityBucket:3;
	uint32 congestionBucket:3;
	uint32 smaCongestionBucket:3;
	uint32 bubbleBucket:3;
	uint32 securityBucket:3;
	uint32 routingBucket:3;
	uint32 spare:2;

	struct _vl_bucket_flagbucket_flags VLBucketFlags[MAX_PM_VLS];
	PmCompositePortCounters_t	stlPortCounters;
	PmCompositeVLCounters_t	stlVLPortCounters[MAX_PM_VLS];
	CounterSelectMask_t clearSelectMask;
	ErrorSummary_t	errors;
	ErrorSummary_t	VFErrors[MAX_VFABRICS];
} PmCompositePort_t;

typedef struct PmCompositeNode_s {
	Guid_t	guid;
	char nodeDesc[STL_NODE_DESCRIPTION_ARRAY_SIZE];
	uint16	lid;
	uint8	nodeType;
	uint8	numPorts;
	PmCompositePort_t	**ports;
} PmCompositeNode_t;

typedef struct PmCompositeVF_s {
	char	name[MAX_VFABRIC_NAME];
	uint8	isActive;
	uint32	numPorts;
	uint8	minIntRate;
	uint8	maxIntRate;
	PmUtilStats_t	intUtil;
	PmErrStats_t	intErr;
} PmCompositeVF_t;

typedef struct PmCompositeGroups_s {
	char	name[STL_PM_GROUPNAMELEN];
	uint32	numIntPorts;
	uint32	numExtPorts;
	uint8	minIntRate;
	uint8	maxIntRate;
	uint8	minExtRate;
	uint8	maxExtRate;
	PmUtilStats_t	intUtil;
	PmUtilStats_t	sendUtil;
	PmUtilStats_t	recvUtil;
	PmErrStats_t	intErr;
	PmErrStats_t	extErr;
} PmCompositeGroup_t;

typedef struct PmHistoryHeaderCommon_s {
	char 	filename[PM_HISTORY_FILENAME_LEN];
	time_t 	timestamp;
	boolean isCompressed;
	uint16	imagesPerComposite;
	uint32	imageSweepInterval;
	uint64	imageIDs[PM_HISTORY_MAX_IMAGES_PER_COMPOSITE];
} PmHistoryHeaderCommon_t;

typedef struct PmFileHeader_s {
	PmHistoryHeaderCommon_t common;
	uint16	historyVersion;
	size_t	flatSize;
	uint8 	numDivisions;
	size_t 	divisionSizes[PM_MAX_COMPRESSION_DIVISIONS];
} PmFileHeader_t;

typedef struct PmCompositeImage_s {
	PmFileHeader_t	header;
	time_t	sweepStart;
	uint32	sweepDuration;
	boolean	topologyChanged;
	boolean	written;
	uint16	HFIPorts;
	uint16	switchNodes;
	uint32	switchPorts;
	uint32	numLinks;
	uint32 	numSMs;
	uint32	failedNodes;
	uint32	failedPorts;
	uint32	skippedNodes;
	uint32	skippedPorts;
	uint32	unexpectedClearPorts;
	uint32	numGroups;
	uint32	numVFs;
	uint32	numVFsActive;
	uint32	maxLid;
	uint32	numPorts;
	struct PmCompositeSmInfo {
		uint16	smLid;			// implies port, 0 if empty record
		uint8	priority:4;		// present priority
		uint8	state:4;		// present state
	} SMs[2];
	PmCompositeGroup_t	allPortsGroup;
	PmCompositeGroup_t	groups[PM_MAX_GROUPS];
	PmCompositeVF_t		VFs[MAX_VFABRICS];
	PmCompositeNode_t	**nodes;
} PmCompositeImage_t;

#define INDEX_NOT_IN_USE 0xffffffff
typedef struct PmHistoryRecord_s {
	PmHistoryHeaderCommon_t header;
	uint32	index;
	struct _imageEntry {
		cl_map_item_t	historyImageEntry;	// key is image ID
		uint32 inx;
	} historyImageEntries[PM_HISTORY_MAX_IMAGES_PER_COMPOSITE];
} PmHistoryRecord_t;

typedef struct _imageEntry PmHistoryImageEntry_t;

typedef struct PmShortTermHistory_s {
	char	filepath[PM_HISTORY_MAX_LOCATION_LEN];
	PmCompositeImage_t	*currentComposite;
	uint32	currentRecordIndex;
	uint64	totalDiskUsage;
	cl_qmap_t	historyImages;	// map of all short term history Records, keyed by image IDs
	cl_map_item_t	historyImagesBase;	
	uint32	totalHistoryRecords;
	uint8	currentInstanceId;
	PmCompositeImage_t *cachedComposite;
	struct _loaded_image {	
		PmImage_t *img;
		PmGroup_t *AllGroup;
		PmGroup_t *Groups[PM_MAX_GROUPS];
		PmVF_t *VFs[MAX_VFABRICS];
	} LoadedImage;
	PmHistoryRecord_t	**historyRecords;
} PmShortTermHistory_t;

// ----------------------------------------------------------

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

	// group per Image data protected by Pm.Image[].imageLock
	// other group data is not changing and hence no lock needed
	PmGroup_t *AllPorts;	// default group including all ports
	// user configured list of groups
	uint8 NumGroups;		// how many of list below are configured/valid
	PmGroup_t *Groups[PM_MAX_GROUPS];

	uint8 numVFs;
	uint8 numVFsActive;
	PmVF_t *VFs[MAX_VFABRICS];

	// these are look aside buffers to translate from a ImageId to an ImageIndex
	uint32 *history;			// exclusively for HISTORY
	uint32 *freezeFrames;		// exclusively for FREEZE_FRAME

	// configuration settings
	uint16 flags;	// configured (see ib_pa.h pmFlags for a list)
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
	uint16		pm_slid;	// SLID for packets we send
	uint32		changed_count;	// last pass synchronized topology with SM
	uint32 		SweepIndex;	// sweep in progress, no lock needed
	cl_qmap_t	AllNodes;	// all PmNode_t keyed by portGuid, engine use only

	// these are private to engine, used to hold sizes for various structures
	// to account for the current pm_total_images value being used
	uint32		PmPortSize;	// PmPort_t size
	uint32		PmNodeSize;	// PmNode_t size

	struct PmDispatcher_s {
		generic_cntxt_t cntx;
		Event_t sweepDone;
		uint8	postedEvent;			// have we posted the sweepDone event
		uint16	nextLid;
		uint16	numOutstandingNodes;	// num nodes in Dispatcher.Nodes
		PmDispatcherNode_t *DispNodes;	// allocated array of PmMaxParallelNodes
	} Dispatcher;

	PmShortTermHistory_t ShortTermHistory;

	// must be last in structure so can dynamically size total images in future
	 PmImage_t *Image;
} Pm_t;

void clearLoadedImage(PmShortTermHistory_t *sth);
size_t computeCompositeSize(void);
FSTATUS decompressAndReassemble(unsigned char *input_data, size_t input_size, uint8 divs, size_t *input_sizes, unsigned char *output_data, size_t output_size);
FSTATUS rebuildComposite(PmCompositeImage_t *cimg, unsigned char *data);
void writeImageToBuffer(Pm_t *pm, uint32 histindex, uint8_t *buffer, uint32_t *bIndex);
void PmFreeComposite(PmCompositeImage_t *cimg);
FSTATUS PmLoadComposite(Pm_t *pm, PmHistoryRecord_t *record, PmCompositeImage_t **cimg);
FSTATUS PmFreezeComposite(Pm_t *pm, PmHistoryRecord_t *record);
PmVF_t *PmReconstituteVFImage(PmCompositeVF_t *cVF);
PmGroup_t *PmReconstituteGroupImage(PmCompositeGroup_t *cgroup);
PmPort_t *PmReconstitutePortImage(PmShortTermHistory_t *sth, PmCompositePort_t *cport);
PmNode_t *PmReconstituteNodeImage(PmShortTermHistory_t *sth, PmCompositeNode_t *cnode);
PmImage_t *PmReconstituteImage(PmShortTermHistory_t *sth, PmCompositeImage_t *cimg);
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
// 	paAccess must have this lock and verify state == VALID
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
// paAccess query (for lastsweep, history or freeze frame query):
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
// 	perform sweep - since it is the "active sweep" paAccess should not try to
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

extern ErrorSummary_t g_pmThresholds;
extern PmThresholdsExceededMsgLimitXmlConfig_t g_pmThresholdsExceededMsgLimit;
extern IntegrityWeights_t g_pmIntegrityWeights;
extern CongestionWeights_t g_pmCongestionWeights;

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
PmNode_t *pm_find_node(PmImage_t *pmimagep, STL_LID_32 lid);

// Lookup a port in pmImage based on lid and portNum
// does not have to be a "lid"'ed port
// caller should have pmImage->imageLock held
PmPort_t *pm_find_port(PmImage_t *pmImage, STL_LID_32 lid, uint8 portNum);

// Clear Running totals for a given Node.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write
FSTATUS PmClearNodeRunningCounters(PmNode_t *pmnodep,
					CounterSelectMask_t select);
FSTATUS PmClearNodeRunningVFCounters(PmNode_t *pmnodep,
					STLVlCounterSelectMask select, char *vfName);

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

void PmFailPort(Pm_t *pm, PmPort_t *pmportp, uint8 queryStatus, const char* message);
void PmFailPacket(Pm_t *pm, PmDispatcherPacket_t *disppacket, uint8 queryStatus, const char* message);
void PmFailNode(Pm_t *pm, PmNode_t *pmnodep, uint8 queryStatus, const char* message);

// pm_debug.c
void DisplayPm(Pm_t *pm, uint32 imageIndex);

void PmFinalizePortStats(Pm_t *pm, PmPort_t *portp, uint32 index);
boolean PmTabulatePort(Pm_t *pm, PmPort_t *portp, uint32 index,
			   			uint32 *counterSelect);
void ClearGroupStats(PmGroupImage_t *groupImage);
void ClearVFStats(PmVFImage_t *vfImage);
void FinalizeGroupStats(PmGroupImage_t *groupImage);
void PmClearPortImage(PmPortImage_t *portImage);
void FinalizeVFStats(PmVFImage_t *vfImage);

void UpdateInGroupStats(Pm_t *pm, PmGroupImage_t *groupImage, PmPortImage_t *portImage);
void UpdateExtGroupStats(Pm_t *pm, PmGroupImage_t *groupImage, PmPortImage_t *portImage, PmPortImage_t *portImage2);
void UpdateVFStats(Pm_t *pm, PmVFImage_t *vfImage, PmPortImage_t *portImage);

// Clear Running totals for a given Port.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write
extern FSTATUS PmClearPortRunningCounters(PmPort_t *pmportp, CounterSelectMask_t select);
extern FSTATUS PmClearPortRunningVFCounters(PmPort_t *pmportp, STLVlCounterSelectMask select, char *vfName);

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

void PmAddPortToVFIndex(PmPortImage_t * portImage, uint32 vfIndex, PmVF_t *vfp);

boolean PmIsPortInGroup(Pm_t *pm, PmPort_t *pmport,
						PmPortImage_t *portImage, PmGroup_t *groupp, boolean sth);
boolean PmIsPortInVF(Pm_t *pm, PmPort_t *pmportp,
						PmPortImage_t *portImage, PmVF_t *vfp);

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

#ifdef __cplusplus
};
#endif

#endif /* _PM_TOPOLOGY_H */
