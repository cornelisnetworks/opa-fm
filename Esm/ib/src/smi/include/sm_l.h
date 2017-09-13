/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_l.h								     //
//									     //
// DESCRIPTION								     //
//    This file contains the localized SM definitions.  These definitions    //
//    do NOT appear in the IBTA specs.  They are implementation dependent.   //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    None								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//    ib_const.h							     //
//									     //
//									     //
//===========================================================================//

#ifndef	_SM_L_H_
#define	_SM_L_H_

#include <stdio.h>
#include <signal.h>
#include "ib_types.h"
#include "ispinlock.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "ib_sm.h"
#include "ib_sa.h"
#include "cs_context.h"
#include "mai_g.h"
#include "cs_g.h"
#include "cs_log.h"
#include "cs_bitset.h"

#include "ilist.h"
#include "iquickmap.h"
#include <iba/stl_sm.h>
#include <iba/stl_helper.h>

#ifdef __VXWORKS__
#include "regexp.h"
#else
#include "regex.h"
#endif

#include <fm_xml.h>
#include "topology.h"

#define	SM_PKEYS    32

#ifdef __VXWORKS__
#define	SM_NODE_NUM 		500
#else
#define	SM_NODE_NUM 		5000
#endif

#define USE_FIXED_SCVL_MAPS

// Support for STL MTU enum, which allows for 8k and 10k
//   In the SM MC Spanning tree, a tree is maintained for each MTU
//     and the original code uses the MTU enum as an index. 
//   This concept works, except that when iterating through loops, the 
//     unused values must be skipped. 
//   NOTE: This function will return values greater than the max mtu
//     to allow for..loop termination.
static __inline__ int getNextMTU(int mtu) {
    if (mtu == IB_MTU_4096) {
        return (STL_MTU_8192);
    } else {
        return (mtu+1);
    }
}

//   NOTE: This function will return values less  than the lowest mtu
//     to allow for..loop termination.
static __inline__ int getPrevMTU(int mtu) {
    if (mtu == STL_MTU_8192) {
        return (IB_MTU_4096);
    } else {
        return (mtu-1);
    }
}



#define MAX_VFABRICS		MAX_ENABLED_VFABRICS
#define MAX_DEVGROUPS		32
#define DEFAULT_DEVGROUP_ID 0xffff
#define DEFAULT_PKEY		0x7fff
#define INVALID_PKEY		0
#define FULL_MEMBER			0x8000
#define ALL_GUIDS			0xffffffffffffffff
#define NULL_SERVICE_MASK	0xffffffffffffffff
#define UNDEFINED_PKEY		0xffffffff
#define PKEY_TYPE_FULL		1

#define TITAN_SYSTEM_IMAGE_BASIC      0x00066A00E0000000ll
#define QLOGIC_SYSTEM_IMAGE_BASIC     0x00066A0000000000ll

//
// These pools are referenced by the macros below
//
extern Pool_t sm_pool;

/* 
 * Revert to using pure DR after SmaEnableLRDR_MAX_RETRIES-1 attempts.
 */
#define SmaEnableLRDR_FALLBACK_OPTION		2	
#define	SmaEnableLRDR_MAX_RETRIES			3	//total retries while using LR-DR SMPs approach

extern int activateInProgress;
extern int isSweeping;
extern int forceRebalanceNextSweep;
extern int topology_changed;  /* indicates a change in the topology during discovery */
extern int topology_changed_count;  /* how many times has topology changed */
extern int topology_switch_port_changes;  /* indicates there are switches with switch port change flag set */
extern int topology_cost_path_changes;
extern int routing_recalculated; /* indicates all routing tables have been recalculated this sweep */
extern uint8_t topology_set_client_reregistration; //flag to force sm_activate_port() to set the client rereg bit
extern uint32_t topology_port_bounce_log_num; /* Number of times a log message has been generated about a port bounce this sweep*/
extern bitset_t old_switchesInUse;
extern bitset_t new_switchesInUse;
extern bitset_t new_endnodesInUse;
extern SMXmlConfig_t sm_config;
extern DGXmlConfig_t dg_config;
extern VFXmlConfig_t vf_config;

//********** SM topology asynchronous send receive context **********
extern  generic_cntxt_t     sm_async_send_rcv_cntxt;
//**********  SM topology asynchronous send receive response queue **********
extern  cs_Queue_ptr sm_async_rcv_resp_queue;

//Link policy violation types used in portData->linkPolicyViolation
#define LINK_POLICY_VIOLATION_SUP 0x1
#define LINK_POLICY_VIOLATION_MIN 0x2


//
//	Internal data structures for simplicity.
//

#define DEFAULT_LOOP_PATH_LENGTH		3
#define	DEFAULT_LOOP_INJECT_EACH_SWEEP	1

//
//  Loop test path structure
//
typedef struct _LoopPath {
    STL_LID     lid;        // lid assigned to the path
    uint16_t    startNodeno;// node number of start/end of loop in topology
    // path from SM node to loop node and loop from node back to itself
    uint16_t    nodeIdx[64];// index of node (node number) in topology
    uint8_t     path[64];   // port through node
    struct _LoopPath *next; // pointer to next loop path
} LoopPath_t;

#define PORT_GUIDINFO_DEFAULT_RECORDS       1       // Maximum records is normally 8
#define IB_VLARB_COUNT						5

//inLoopCount is 8 bits hence max value is 255
//Increment such that, overflow doesn't result in the value becoming 0 as
//zero value of inLoopCount would be considered as the port not being in any loop
#define INC_IN_LOOP_COUNT(P)	(P = (P == 255) ? 1 : (P+1))

// maximum times do immediate resweep after an individual port fails to move
// to the proper state after requesting it to move to Armed or Active.
// Once this count is exceeded, the fabric will not be reswept until the next
// normally scheduled fabric sweep or fabric port change.
#define ACTIVATE_RETRY_LIMIT 3

typedef struct {
	uint8 refcount;
	uint8 buffer[128];
} CableInfo_t;


#define MAX_NODE_DESC_ENTRIES 30
typedef struct {
	int numPortRanges;
	int port1[MAX_NODE_DESC_ENTRIES];
	int port2[MAX_NODE_DESC_ENTRIES];
} PortRangeInfo_t;

typedef struct {
	STL_VLARB_TABLE_ELEMENT  vlarbLow[STL_MAX_LOW_CAP];
	STL_VLARB_TABLE_ELEMENT  vlarbHigh[STL_MAX_LOW_CAP];
	STL_VLARB_TABLE_ELEMENT  vlarbPre[STL_MAX_PREEMPT_CAP];
} VlarbTableData;

typedef struct _PortDataVLArb {
	uint32				vlarbMatrix[STL_MAX_VLS];
	VlarbTableData		vlarb;
} PortDataVLArb;

//
//	Per Port structure.
//
typedef	struct _PortData {
	uint64_t	guid;		// port GUID
	uint64_t	gidPrefix;	// Gid Prefix
	uint8_t		gid[16];	// Gid (network byte order)
	uint32_t	capmask;	// capacity mask
// lid, lmc and vl could be much smaller fields, lid=16bits, lmc=3, vls=3 bits
	STL_LID		lid;		// base lid
	uint8_t		lmc;		// LMC for this port
	uint8_t		vl0;		// VLs supported
	uint8_t		vl1;		// VLs actual
	uint8_t		mtuSupported:4;		// MTUs supported
    uint8_t		maxVlMtu:4;   // Largest mtu of VLs/VFs on this port.
    uint8_t     rate;       // static rate of link (speed*width)
	uint16_t    portSpeed;  // calculated portSpeed used in cost (stored due to high usage)
    uint8_t     lsf;        // calculated link speed data rate factor used in cost
	uint8_t		guidCap;	// # of GUIDs
	uint8_t		numFailedActivate;	// how many times has sm_activate_port
   							// failed to activate this port
	uint32_t	flags;		// local flags
	STL_LID		pLid;		// Persistent lid
	STL_PKEY_ELEMENT pPKey[SM_PKEYS];// Persistent PKeys
	uint16_t	num_pkeys;	// Persistent number of PKeys
	uint64_t	trapWindowStartTime;//timestamp of last trap within the thresholdWindow
	uint32_t	trapWindowCount;/* trap counter maintained during the thresholdWindow for
								   disabling runaway ports */
	uint64_t	lastTrapTime; //time stamp of last trap 
	uint8_t		suppressTrapLog; // if set to 1, trap info of this port will not be logged
	uint8_t		logSuppressedTrapCount; // count of traps while logging is suppressed
	uint8_t		logTrapSummaryThreshold;/*threshold of traps when summary of traps
										 should be logged even if logging is suppressed*/
	uint8_t		mcDeleteCount;
	uint64_t	mcDeleteStartTime;
	STL_PORT_INFO	portInfo;	// PortInfo (for SA)

	PortDataVLArb	curArb;		// Current values on SMA
	PortDataVLArb	*newArb;	// Not yet set on SMA

	STL_SLSCMAP slscMap;		// SL2SC mapping table
	STL_SCSLMAP scslMap;		// SC2SL mapping table
	STL_SCVLMAP scvltMap;
	STL_SCVLMAP scvlntMap;
	STL_SCVLMAP scvlrMap;
	STL_SCSCMAP *scscMap;
	STL_BUFFER_CONTROL_TABLE bufCtrlTable;
	STL_HFI_CONGESTION_CONTROL_TABLE *hfiCongCon; // HFI Port or EH SWP 0 only.
	STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT swPortCongSet; // Switch port only.
	bitset_t	vfMember;
	bitset_t	dgMember;		// Bitset indicating the index values of all device groups.
	uint16_t	dgMemberList[MAX_DEVGROUPS]; // Indices of the 1st 32 device groups. Used by PM.
	bitset_t	fullPKeyMember;
	uint16_t		lidsRouted; 	// Number of lids routed through this port
	uint16_t		baseLidsRouted; // Number of base lids routed through this port

	/* Flags*/
	uint8_t		qosHfiFilter:1;
	uint8_t		isIsl:1;		// Switch port is linked to another switch
	uint8_t		uplink:1;		// Uplink port in fat tree
	uint8_t		downlink:1;		// Downlink port in fat tree
	uint8_t		reregisterPending:1;	// True if we need to send a PortInfo with ClientReregister

	uint8_t		inLoopCount;	//number of times this port has been included in a loop as part of loop test

	/// Shared cable information; use sm_CableInfo_*() functions to create and manage
	CableInfo_t * cableInfo;

	STL_LINKDOWN_REASON LinkDownReasons[STL_NUM_LINKDOWN_REASONS];
	uint8_t neighborQuarantined;
	uint8_t linkPolicyViolation:2;  //port outside speed/width policy. 
									//bit[0]: link below max supported
									//bit[1]: port below minimum allowed

	cl_map_obj_t	mapObj;		// quickmap item to sort on GUIDs

	struct _Node *nodePtr; 	// Ptr to the node record this port belongs.
	//void       *pmData;		// Pointer to Pm per port Data

	/**
		Whether stored values are up-to date with values on SMA.
		Not the same as whether values on SMA match the expected or correct values.
	*/
	struct {
		uint8_t slsc:1;
		uint8_t scsl:1;
		uint8_t scvlt:1;
		uint8_t scvlnt:1;
		uint8_t scvlr:1;
		uint8_t scsc:1;
		uint8_t vlarbLow:1;
		uint8_t vlarbHigh:1;
		uint8_t vlarbPre:1;
		uint8_t vlarbMatrix:1;
		uint8_t	pkeys:1;
		uint8_t bfrctrl:1;
	} current;

	struct {
		uint8_t init:1;
		uint8_t slsc:1;
		uint8_t scsl:1;
		uint8_t portInfo:1;
	} dirty;

	struct {
		STL_SLSCMAP * slsc;
		STL_SCSLMAP * scsl;
	} changes;

	void		*routingData; 	// Private data used by topology algorithm.
	uint16_t	routingCost;	// Used for enhanced hypercube
	int32_t		initWireDepth;		// Initial wire depth to use for buffer calculations.
} PortData_t;

// This structure is retained for all switch ports and has summary information
typedef	struct _Port {
	uint8_t		index;		// my port number
	uint8_t		state;		// portState (IB_PORT_DOWN ... IB_PORT_ACTIVE)
	uint32_t	nodeno;		// node index of other end
	uint32_t	portno;		// port index of other end
	uint8_t		path[64];	// path to get to this port
    PortData_t	*portData;  // Pointer to additional port related fields. 
							// may be NULL when dynamic port alloc enabled
							// TBD - is dynamic port alloc needed?  Should it
							// be removed and this changed to a structure
							// instead of a pointer?
} Port_t;

//
//	Per Node structure.
//

#define SIZE_LFT (UNICAST_LID_MAX + 1)
#define MAX_SIZE_MFT ((MULTICAST_LID_MAX-MULTICAST_LID_MIN)+1)
#ifdef __VXWORKS__
#define MAX_MCAST_MGIDS  1000
#else
#define MAX_MCAST_MGIDS  20000
#endif
typedef	struct _Node {
	uint32_t	index;		// index in linked list
	uint32_t	oldIndex;	// index in linked list in old topology
    uint16_t    swIdx;      // switch index in switch list
	uint32_t	flags;		// FabricExec flags
	STL_NODE_INFO	nodeInfo;	// NodeInfo (for SA)
	STL_NODE_DESCRIPTION	nodeDesc;	// NodeInfo (for SA)
	STL_SWITCH_INFO	switchInfo;	// SwitchInfo (for SA)	// TBD make a pointer and only allocate for switches
	STL_PORT_STATE_INFO *portStateInfo;
	STL_CONGESTION_INFO congestionInfo;
	uint8_t		path[64];	// directed path to this node
	PORT		*lft;
	STL_PORTMASK **mft;		// 2D array of port masks for MFT.
	STL_PORTMASK *pgt;		// 1D array of port masks for Port Group Table. 
	uint8_t		pgtLen;		// Current length of PGT. 
	PORT		*pgft;		///< Port Group Forwarding Table
	uint32_t 	pgftSize; 	///< amount of memory allocated for pgft; should track actual memory allocation, not highest entry in use
	Port_t		*port;		// ports on this node
	bitset_t	activePorts;
	bitset_t	initPorts;
	bitset_t	vfMember;
	bitset_t	fullPKeyMember;
	uint16_t	nonRespCount;	// Number of times the node has not responded to gets/sets
	uint64_t	nonRespTime;	// Timestamp of first failure to respond.
	uint8_t     asyncReqsSupported; // number of async requests node can handle
	uint8_t     asyncReqsOutstanding; // number of async requests on the wire
	uint8_t		portsInInit; 
	uint8_t		activeVLs; 
	uint8_t		vlCap;			// capability of switch internal
	uint8_t		numISLs; 
	uint8_t		tier; 			// tier switch resides on in fat tree
	uint16_t	tierIndex;		// switch index within it's tier
	uint8_t		uplinkTrunkCnt; // number of uplink ports in trunk group
	uint16_t		numLidsRouted;	// used to build balanced LFTs.
	uint16_t		numBaseLidsRouted;	// used to build balanced LFTs when LMC enabled.
	char*		nodeDescString;	// Only used when nodeDesc is 64 chars (not null terminated).
	cl_map_obj_t	mapObj;		// Quickmap item to sort on guids
	cl_map_obj_t	nodeIdMapObj;	// Quickmap item to sort on guids
	cl_map_obj_t	mapQuarantinedObj;		// Quickmap item to sort on guids

	void		*routingData;	// Routing specific data.

	/* There exist a case where these are both used (Enhanced SWP0 switch). We'll just
	 store both */
	STL_HFI_CONGESTION_SETTING hfiCongestionSetting;
	STL_SWITCH_CONGESTION_SETTING swCongestionSetting;

	struct		_Node *next;		// linked list pointer (for_all_nodes())
	struct		_Node *prev;		// linked list pointer (for_all_nodes())
	struct		_Node *type_next;	// linked list pointer (NodeType based)
	struct		_Node *type_prev;	// linked list pointer (NodeType based)
    struct      _Node *sw_next;		// linked list pointer (tier based)
    struct      _Node *sw_prev;		// linked list pointer (tier based)
    struct      _Node *old;			// Pointer to a copy of this node in an older topology. MAY BE NULL.

	/* Flags*/
	uint8_t		initDone:1; 		//temporary flag during sweep that indicates if the switch is initialized
	uint8_t		noLRDRSupport:1;	//indicates that the switch is not handling LR DR packets correctly*/
	uint8_t		hasSendOnlyMember:1; //switch has a multicast sendonly member attached */ 
	uint8_t		mftChange:1;  		// indicates if mft of switch has to be completely reprogrammed
	uint8_t		mftPortMaskChange:1;// indicates if mft entries of switch have changed since last sweep
	uint8_t		arChange:1;			// indicates adaptive routing tables of switch need to be programmed
	uint8_t		arSupport:1;		// indicates switch supports adaptive routing
	uint8_t		arDenyUnpause:1;	///< Normally 0, set to 1 to prevent unpausing AR on an SMA because the SM was not able to update PGT/PGFTs fully on the SMA
	uint8_t		oldExists:1;  		// this node is also in the old topology
	uint8_t		uniformVL:1;  
	uint8_t		internalLinks:1;  
	uint8_t		externalLinks:1;  
	uint8_t		edgeSwitch:1;  		// switch has HFIs attached
	uint8_t		trunkGrouped:1;		// ports in fat tree trunk are grouped by contiguous port numbers
	uint8_t		skipBalance:1;  
	uint8_t		routingRecalculated:1; // routing has been recalculated during this sweep
	uint8_t		nodeDescChgTrap:1;  
	uint8_t		mcSpanningChkDone:1; //indicates switch has been checked as part of spanning tree construction
	//uint8_t		pmFlags;		// PM flag - TBD how many bits needed
	uint8_t		slscChange:1;		//indicates if sl2sc of the node needs to be programmed
	uint8_t		aggregateEnable:1;
} Node_t;

typedef struct _QuarantinedNode {
    Node_t * authenticNode;
    Port_t * authenticNodePort; 
    Node_t * quarantinedNode; 
	uint32 quarantineReasons;
	STL_EXPECTED_NODE_INFO expNodeInfo;
	struct _QuarantinedNode *next; // linked list pointer
	struct _QuarantinedNode *prev; // linked list pointer
} QuarantinedNode_t;

typedef struct _SwitchList 
{
	Node_t				*switchp;
	struct _SwitchList	*next;
	uint8_t				parent_portno;	//port number of parent switch through which switchp was added to the list
} SwitchList_t;

/*
 * The set of quickmaps and ordered device groups associated with a topology 
 * structure.
 *
 * NOTA BENE: The deviceGroup[] array has one more member than the 
 * deviceGroupName[] or deviceGroupIndex[] arrays. This is to allow all
 * un-weighted LIDs to be routed correctly.
 */
typedef struct	_DGTopology {
	uint8_t			dgCount;
	uint8_t			allGroup;
	char			deviceGroupName[MAX_DGROUTING_ORDER][MAX_VFABRIC_NAME+1];
	uint16_t		deviceGroupIndex[MAX_DGROUTING_ORDER];
	cl_qmap_t		deviceGroup[MAX_DGROUTING_ORDER+1];
} DGTopology;

//
// Path Matrix Port Masks
//

/**
	Path matrix portmask array length in units of 64 bits.
	Separate from STL_MAX_PORTMASK to limit the size of path matrix.
 */
#define SM_PATH_MAX_PORTMASK ((MAX_STL_PORTS + 63) / 64)

/**
	Fixed-size portmask used by the path matrix.

	Local path helpers interpret ports as 1-255 and 0 as not found,
	unlike stl_helper which encodes 0-254 (with 255 as not found).
	This allows 64 port switches to be encoded in a single uint64.
 */
typedef struct _SmPathPortmask {
	uint64_t masks[SM_PATH_MAX_PORTMASK];
} SmPathPortmask_t;

#define SM_PATH_PORTMASK_EMPTY ((SmPathPortmask_t){{0}})

static __inline__ void sm_path_portmask_set(SmPathPortmask_t * ports, int port) {
	assert(0 < port && port <= MAX_STL_PORTS);
	ports->masks[(port - 1) / 64] |= (uint64_t)1 << ((port - 1) % 64);
}

static __inline__ void sm_path_portmask_clear(SmPathPortmask_t * ports, int port) {
	assert(0 < port && port <= MAX_STL_PORTS);
	ports->masks[(port - 1) / 64] &= ~((uint64_t)1 << ((port - 1) % 64));
}

static __inline__ void sm_path_portmask_merge(SmPathPortmask_t * dst, SmPathPortmask_t * src) {
	int i;
	for (i = 0; i < SM_PATH_MAX_PORTMASK; ++i)
		dst->masks[i] |= src->masks[i];
}

static __inline__ int sm_path_portmask_pop_first(SmPathPortmask_t * ports) {
	int i, base, bit;
	for (i = base = 0; i < SM_PATH_MAX_PORTMASK; ++i, base += 64) {
		bit = __builtin_ffsll(ports->masks[i]);
		if (bit) {
			ports->masks[i] ^= (uint64_t)1 << (bit - 1);
			return base + bit;
		}
	}
	return 0;
}

/**
	The array offset to the i-th block of destination switches.
 */
#define SM_PATH_OFFSET(I, NumSwitches) ((I) * (NumSwitches))

/**
	The span of switches and ports for the i-th source switch in bytes.
 */
#define SM_PATH_SWITCH_SPAN(NumSwitches) ((NumSwitches) * sizeof(SmPathPortmask_t))

/**
	The size of the path matrix in bytes.
 */
#define SM_PATH_SIZE(NumSwitches) ((NumSwitches) * SM_PATH_SWITCH_SPAN(NumSwitches))

//
// Routing structures
//

#define SM_DOR_MAX_DIMENSIONS 6
#define SM_DOR_MAX_WARN_THRESHOLD 255	/*size of warning counter in sm_dor.c is 1 byte*/

struct _Topology;
struct _VlVfMap;
struct _VlBwMap;
struct Qos;
struct _RoutingModule;
struct SwitchportToNextGuid;

typedef struct _RoutingFuncs {
	Status_t (*pre_process_discovery)(struct _Topology *, void **);
	Status_t (*discover_node)(struct _Topology *, struct _Node *, void *);
	Status_t (*discover_node_port)(struct _Topology *, struct _Node *, struct _Port *, void *);
	Status_t (*post_process_discovery)(struct _Topology *, Status_t, void  *);

	/* These functions allow topology-specific code to examine the new 
 	 * topology and the old topology and determine if the SM needs to update
 	 * the routing tables of the switches. Implementations should set the
 	 * rebalance flag to non-zero if updates are needed, but should not
 	 * modify it if no update is needed.
 	 */
	Status_t (*post_process_routing)(struct _Topology *, struct _Topology *, int * rebalance);
	Status_t (*post_process_routing_copy)(struct _Topology *, struct _Topology *, int * rebalance);

	// Allows for topology specific intialization and calculation of the cost matrix used
	// to calculate routes.
	Status_t (*allocate_cost_matrix)(struct _Topology *);
	Status_t (*initialize_cost_matrix)(struct _Topology *);
	Status_t (*calculate_cost_matrix)(struct _Topology *, int switches, unsigned short * cost, SmPathPortmask_t *);

	/**
		Copy xFT from src to dest where src is assumed to be a predecessor of dest.

		See @fn sm_routing_func_copy_routing_lfts() for a typical implementation.

		@return VSTATUS_OK if no problems occurred, non-OK otherwise.
	*/
	Status_t (*copy_routing)(struct _Topology * src, struct _Topology * dest);

	/**
		Perform routing algorithm/topology-specific xFT (LFT, MFT, PGT, PGFT) initialization for switches.  Implementation should do initialization necessary for SM to send LID-routed to any switch.

		See @fn topology_setup_switches_LR_DR()

		@param routing_needed [in,out] As input, true indicates xFT initialization is needed to all nodes, false only to new nodes.
		@param rebalance [out] Implementation should set this to '1' (really non-zero) if 

		@return VSTATUS_OK on success, non-OK to indicate a fatal error.
	*/
	Status_t (*init_switch_routing)(struct _Topology * topop, int * routing_needed, int * rebalance);
	Status_t (*setup_switches_lrdr)(struct _Topology *, int, int);

	// Optionally override this function to change the order in which routing tables are built.
	// Leave null to accept the default function.
	Status_t (*calculate_routes)(struct _Topology *, struct _Node *);

	/**
		@param switchp the destination switch
		@param nodep the source node or switch
		@param orig_port the port from which LFT and PGT/PGFTs are built.
	*/
	Status_t (*setup_xft)(struct _Topology *, struct _Node *, struct _Node *, struct _Port *, uint8_t *);
	int (*select_ports)(struct _Topology *, Node_t *, int, struct SwitchportToNextGuid *, boolean);

	/**
		Compute the portgroup assignments on @c srcSw for all equally-best paths from @c srcSw to
		@c dstSw and the end nodes attached to @c dstSw.

		Implementations may accept only switch pairs or (switch, node) pairs, although obviously
		node pairs don't make particular sense.  Implementation should set @c switchp->arChange to 1
		if either the PGT or PGFT were changed in the process.

		Optional for a topology module; NULL indiciates not implemented.

		@param [optional, out] portnos 
		@return VSTATUS_OK on success, something else if srcSw and dstSw aren't switches or if other bad stuff happens.
	*/
	Status_t (*setup_pgs)(struct _Topology *topop, struct _Node * srcSw, struct _Node * dstSw);

	int (*get_port_group)(struct _Topology *, struct _Node *, struct _Node *, uint8_t *);
	Status_t (*select_slsc_map)(struct _Topology *, struct _Node *, struct _Port *, struct _Port *, STL_SLSCMAP *);
	Status_t (*select_scsl_map)(struct _Topology *, struct _Node *, struct _Port *, struct _Port *, STL_SCSLMAP *);
	Status_t (*select_scsc_map)(struct _Topology *, struct _Node *, int , int *, STL_SCSC_MULTISET** scscmap);
	Status_t (*select_scvl_map)(struct _Topology *, struct _Node *, struct _Port *, struct _Port *, STL_SCVLMAP *);
	Status_t (*select_vlvf_map)(struct _Topology *, struct _Node *, struct _Port *, struct _VlVfMap *);
	Status_t (*select_vlbw_map)(struct _Topology *, struct _Node *, struct _Port*, struct _VlBwMap *);
	Status_t (*select_scvlr_map)(struct _Topology *, uint8_t, STL_SCVLMAP *);
	Status_t (*fill_stl_vlarb_table)(struct _Topology *, struct _Node *, struct _Port *, struct _PortDataVLArb * arb);
	Status_t (*select_path_lids)(struct _Topology *, struct _Port *, uint16_t, struct _Port *, uint16_t, uint16_t[], uint8_t *, uint16_t[], uint8_t *);
	Status_t (*process_swIdx_change) (struct _Topology *, int old_idx, int new_idx, int last_idx);

	/**
 		Used to determine if fabric change requires fabric routing reprogramming.
 	*/
	int (*check_switch_path_change) (struct _Topology *, struct _Topology *, struct _Node *);

	/**
		Predicate function to determine if any routing recalculation is required.

		Implementation should return zero if no recalculation is required, non-zero otherwise.
	*/
	boolean (*needs_routing_recalc)(struct _Topology * topop, Node_t * nodep);

	boolean (*can_send_partial_routes)(void);

	/**
 		Used by sm_routing_func_select_ports to determine spine first routing
 	*/
	boolean (*do_spine_check)(struct _Topology *topop, Node_t *switchp);

	/** 
 		Differ by write of LFT or HFT
	*/
	Status_t (*write_minimal_routes)(struct _Topology *, struct _Node *, int, uint8_t *);
	Status_t (*write_full_routes_LR)(struct _Topology *, struct _SwitchList *, int);
	Status_t (*route_old_switch)(struct _Topology *, struct _Topology *, struct _Node *);

	/**
		Override the building of the multicast spanning trees.
	*/
	void (*build_spanning_trees)(void);

	/**
 		Used to determine if local switch change requires routing change.
 	*/
	boolean (*handle_fabric_change)(struct _Topology *, struct _Node *, struct _Node *);

	/**
		QOS functions
	*/

	/**
		Implementation should assign/adjust bandwidth on virtual fabrics objects in
		@c VirtualFabrics->v_fabric_all. Unassigned bandwidth indicated by
		@c VF_t.percent_bandwidth == UNDEFINED_XML8.
	*/
	Status_t (*update_bw)(struct _RoutingModule *rm, VirtualFabrics_t *VirtualFabrics);

	/**
		Compute global SLSC mppings. Only applicable to implementations
		that use a single SLSC map for all SLSC-eligible ports.
	*/
	Status_t (*assign_scs_to_sls)(struct _RoutingModule *rm, VirtualFabrics_t *VirtualFabrics);

	/**
		Assign SLs to VF_t objects in @c VirtualFabrics.
	*/
	Status_t (*assign_sls)(struct _RoutingModule *rm, VirtualFabrics_t *VirtualFabrics);

	/**
		Does this routing algorithm require multicast isolation
	*/
	boolean (*mcast_isolation_required)(void);

	/**
		Minimum number of vls supported by routing algorithm
		Note: 1 vl is always supported, even though min_vls may return more than 1
	*/
	int (*min_vls)(void);

	/**
		Maximum number of vls supported by routing algorithm
	*/
	int (*max_vls)(void);

	/**
		Number of routing SCs required by SL. mc_sl should be true if SL is for
		multicast, false otherwise.
	*/
	int (*num_routing_scs)(int sl, boolean mc_sl);

} RoutingFuncs_t;

typedef struct _RoutingModule {
	const char * name;

	RoutingFuncs_t funcs;

	// Private data used by topology algorithm.
	void	*data;

	/**
		Initialize new instance @c dest to be a copy of @c src.  Implementation is responsible for all fields in @c dest.  If no implementation is provided, caller should do a memcpy(dest, src, size).

		Caller should not expect implementation to handle @c dest with non-NULL pointer members (@c dest has already been initialized).

		Typically only called at the start of a new sweep when there is a need to copy data gathered in the prior sweep to the current sweep.

		@return VSTATUS_Ok on success or non-OK to indicate failure.
	*/
	Status_t (*copy)(struct _RoutingModule * dest, const struct _RoutingModule * src);

	/**
		Called when module is released , usually at end of sweep (see @fn topology_release_saved_topology() and @fn topology_clearNew()) or when a  module is unloaded.
		Note that release() can be called without unload() being called.

		Implementation is responsible for lifetime of fields of @c rm but not lifetime of @c rm itself.

		See also @fn RoutingFuncs_t::destroy().  release() will be called after destroy().

		@return VSTATUS_OK on success or non-OK if an unrecoverable runtime error occurs.
	*/
	Status_t (*release)(struct _RoutingModule * rm);

} RoutingModule_t;

/**
	Function type for initializing RoutingModule_t instance.  Instance is already allocated, factory function should initialize the instance fields and do any allocation for the fields.

	@return VSTATUS_OK on success, non-OK on failure.
*/
typedef Status_t (*routing_mod_factory)(RoutingModule_t * module);

Status_t sm_routing_addModuleFac(const char * name, routing_mod_factory fac);

/**
	Allocate and initialize module instance using factory function associated with @c name.

	@param module [out] Location to store pointer to newly-allocated module.
*/
Status_t sm_routing_makeModule(const char * name, RoutingModule_t ** module);

/**
	Allocate and initialize routing module instance as copy of @srcMod.
	Does copy of @c srcMod using @c srcMod->copy if @c srcMod->copy is non-NULL.
	Does memcpy(*newMod, srcMod) otherwise.

	@param newMod [out] Location to store pointer to newly-allocated module.
*/
Status_t sm_routing_makeCopy(RoutingModule_t **newMod, RoutingModule_t *srcMod);
/**
	Free routing module memory. Prefer this to direct vs_pool_free() since internal memory management details may change.

	@param module [out] If @c *module is freed, @c *module will be set to NULL.

	@return VSTATUS_ILLPARM if either @c module or @c *module is NULL.  VSTATUS_OK otherwise.
*/
Status_t sm_routing_freeModule(RoutingModule_t ** module);

typedef struct _PreDefTopoLogCounts {
	uint32_t 	totalLogCount;
	uint32_t 	nodeDescWarn;
	uint32_t 	nodeDescQuarantined;
	uint32_t 	nodeGuidWarn;
	uint32_t 	nodeGuidQuarantined;
	uint32_t 	portGuidWarn;
	uint32_t 	portGuidQuarantined;
	uint32_t 	undefinedLinkWarn;
	uint32_t 	undefinedLinkQuarantined;
} PreDefTopoLogCounts;

//
//	Per Topology structure.
//
typedef	struct _Topology {
	STL_LID		lid;		// the last lid used
	uint32_t	lmc;		// the global LMC
	uint32_t	num_nodes;	// count of how many nodes have been found
	uint32_t	num_sws;	// count of how many switch chips have been found
	uint32_t	max_sws;	// size of the switch index space (including sparseness)
	uint32_t    num_ports;	// num of live ports in fabric (init state or better)
    uint32_t    num_endports; // num end ports (init state or better)
	uint32_t	num_quarantined_nodes;	// count of how many nodes have been found
	uint32_t	nodeno;		// node index of new node
	uint32_t	portno;		// port index of new node
	uint32_t	state;		// SM state
	STL_LID		maxLid;		// maximum Lid assigned

	uint16_t         *cost; // array for path resolution
	SmPathPortmask_t *path; // array for path resolution (next-hop matrix, each entry is a portmask of best next hops)
	Node_t		*node_head;	// linked list of nodes
	Node_t		*node_tail;	// ditto
	Node_t		*ca_head;	// linked list ofFI nodes
	Node_t		*ca_tail;	// ditto
	Node_t		*switch_head;	// linked list of SWITCH nodes
	Node_t		*switch_tail;	// ditto
	QuarantinedNode_t *quarantined_node_head;	// linked list of nodes that failed authentication
	QuarantinedNode_t *quarantined_node_tail;	// linked list of nodes that failed authentication
    Node_t      *tier_head[MAX_TIER]; // array of linked lists of switches indexed by tier
    Node_t      *tier_tail[MAX_TIER]; // ditto

	size_t		bytesCost;		// cost/path array length
	size_t		bytesPath;
	uint8_t		sm_path[72];	// DR path to SM (STANDBY mode only)
	uint32_t	sm_count;	// actCount of master SM (STANDBY mode only)
    uint64_t    sm_key;    // the smKey of the master SM (STANDBY mode only)
	STL_LID		slid;		// Source lid for next send
	STL_LID		dlid;		// Destination lid for next send
    uint16_t    numSm;      // number of SMs in fabric
    //uint16_t    numMasterSms; // number of master SMs in fabric
    //TopologySm_t smRecs[SM_MAX_SUPPORTED]; // list of SMs in topology
	uint32_t    numRemovedPorts; // number of ports removed during discovery
    // loop test variables
	uint16_t	numLoopPaths; // number of loop paths in fabric
	LoopPath_t	*loopPaths; // loop paths in fabric
	uint8_t		pad[65536];	// general scratch pad area
	Node_t		**nodeArray;	// Array of all nodes
	cl_qmap_t	*nodeMap;	// Sorted GUID tree of all nodes
	cl_qmap_t	*portMap;	// Sorted GUID tree of all ports
	cl_qmap_t	*nodeIdMap; // Sorted Node tree based on the locally assigned node id.
	cl_qmap_t	*quarantinedNodeMap;	// Sorted GUID tree of all nodes
	uint8_t		maxMcastMtu; // Maximum MTU supported by fabric for multicast traffic
	uint8_t		maxMcastRate; // Maximum rate supported by fabric for multicast traffic
	uint8_t		maxISLMtu;		// Maximum ISL MTU value seen in the fabric
	uint8_t		maxISLRate;		// Maximum ISL rate value seen in the fabric
	uint8_t		qosEnforced; // QoS is being enforced based on fabric parameters

	/// Allocated in sm_routing_makeModule(), freed in either topology_release_saved_topology() and/or topology_clearNew()
	RoutingModule_t * routingModule;

	uint64_t	sweepStartTime;
	uint64_t	lastNDTrapTime;

	bitset_t    deltaLidBlocks;
	// Was bitset_init(...&deltaLidBlocks) called?
	uint8_t     deltaLidBlocks_init;

	bitset_t    *smaChanges;
	VirtualFabrics_t *vfs_ptr;

	// Per sweep pre-defined topology log counts
	PreDefTopoLogCounts preDefLogCounts;

	/// Store link down reasons for ports that disappeared;
	/// see LdrCacheEntry_t
	cl_qmap_t *ldrCache;
} Topology_t;

typedef struct {
	uint64_t guid;
	uint8_t index;
} LdrCacheKey_t;

typedef struct {
	cl_map_item_t mapItem;
	LdrCacheKey_t key;
	STL_LINKDOWN_REASON ldr[STL_NUM_LINKDOWN_REASONS];
} LdrCacheEntry_t;

/**
 * Allocates topo->ldrCache in sm_pool if NULL.
 */
void topology_saveLdr(Topology_t * topo, uint64_t guid, Port_t * port);

typedef	struct {
	Guid_t		guid;		// guid for this index (ie, Lid)
	uint32_t	pass;		// last pass count I saw this Guid
	Node_t		*oldNodep;	// active pointer to node in old topology
	cl_map_obj_t	mapObj;	// for storing this item in the GuidToLidMap tree
    /* 
     * the following fields are for topology thread use only 
     * They are transient and are valid only when SM is discovering
     */
	Node_t		*newNodep;	// pointer to node in new topology
	Port_t      *newPortp;
} LidMap_t;

//
//	Multicast structures.
//
typedef	struct _McMember {
	struct _McMember	*next;
	STL_LID			slid;
	uint8_t			proxy;
	uint8_t			state;
	uint64_t		nodeGuid;
	uint64_t		portGuid;
	STL_MCMEMBER_RECORD	record;
	uint32_t		index; /* Used by OOB mgmt interface to track record */
} McMember_t;

typedef enum {
	McGroupBehaviorStrict = 0,
	McGroupBehaviorRelaxed = 1,
	McGroupBehaviorSpanningTree = 2,
} McGroupJoinBehavior;

typedef enum {
	McGroupUnrealizable = (1 << 0), // Signifies that the group has become unrealizable
} McGroupFlags;

typedef	struct _McGroup {
	struct _McGroup		*next;
	IB_GID			mGid;
	uint32_t		members_full;
	uint32_t		qKey;
	uint16_t		pKey;
	STL_LID			mLid;
	uint8_t			mtu;
	uint8_t			rate;
	uint8_t			life;
	uint8_t			sl;
	uint32_t		flowLabel;
	uint8_t			hopLimit;
	uint8_t			tClass;
	uint8_t			scope;
	McMember_t		*mcMembers;
	uint32_t		index_pool; /* Next index to use for new Mc Member records */
	bitset_t		vfMembers;
	McGroupFlags	flags;      // Flags associated with this group
} McGroup_t;

#if defined(__VXWORKS__)
#define SM_STACK_SIZE (24 * 1024)
#else
#define SM_STACK_SIZE (256 * 1024)
#endif
//
//	Thread structure.
//
typedef struct {
	Thread_t		handle;
	Threadname_t	name;
	void			(*function)(uint32_t, uint8_t **);
	uint8_t			*id;
} SMThread_t;

#define	SM_THREAD_SA_READER		0
#define	SM_THREAD_SA_WRITER		1
#define	SM_THREAD_TOPOLOGY		2
#define	SM_THREAD_ASYNC			3
#define SM_THREAD_DBSYNC        4
#define SM_THREAD_TOP_RCV       5
#define SM_THREAD_PM            6

#ifndef FE_THREAD_SUPPORT_ENABLED
#define	SM_THREAD_MAX			SM_THREAD_PM
#else
#define SM_THREAD_FE            7
#define	SM_THREAD_MAX			SM_THREAD_FE
#endif

//
// Removed fabric entities
//

typedef enum {
	SM_REMOVAL_REASON_INVALID = 0,
	SM_REMOVAL_REASON_1X_LINK,
	SM_REMOVAL_REASON_TRAP_SUPPRESS,
	SM_REMOVAL_REASON_MC_DOS,
} RemovedEntityReason_t;

typedef struct _RemovedPort {
	uint64_t              guid;
	uint32_t              index;
	RemovedEntityReason_t reason;
	struct _RemovedPort   *next;
} RemovedPort_t;

typedef struct {
	uint64_t              guid;
	STL_NODE_DESCRIPTION  desc;
	cl_map_obj_t          mapObj;
	RemovedPort_t         *ports;
} RemovedNode_t;

typedef struct {
	cl_qmap_t nodeMap;
	Lock_t    lock;
	uint8_t   initialized;
} RemovedEntities_t;

//
// throttling MAD dispatcher
//

// actual timeout passed to vs_event_wait in the wait loop
#define SM_DISPATCH_EVENT_TIMEOUT VTIMER_1S

// number of loops (roughly each of length EVENT_TIMEOUT) that can pass with
// the dispatcher state remaining unchanged before we give up and assume
// that the dispatcher has stalled (likely due to a leak in the async
// context code)
#define SM_DISPATCH_STALL_THRESHOLD 7 // (0 - 255)

typedef struct sm_dispatch_send_params {
	IBhandle_t fd;
	uint32_t method;
	uint32_t aid;
	uint32_t amod;
	uint8_t *path;
	uint16_t slid;
	uint16_t dlid;
	uint8_t  buffer[STL_MAD_PAYLOAD_SIZE];
    uint32_t bufferLength;
	uint64_t mkey;
	uint32_t reply;
    uint8_t bversion; 
} sm_dispatch_send_params_t;

typedef struct sm_dispatch_req {
	sm_dispatch_send_params_t sendParams;
	Node_t *nodep;
	uint32_t sweepPasscount;
	struct sm_dispatch *disp;
	struct sm_dispatch_req *next, *prev;
	LIST_ITEM item;
} sm_dispatch_req_t;

// we use sm_async_send_rcv_cntxt.lock to protect this structure
typedef struct sm_dispatch {
	int initialized;
	uint32_t reqsOutstanding;
	uint32_t reqsSupported;
	uint32_t sweepPasscount;
	Event_t evtEmpty;
	struct sm_dispatch_req *queueHead, *queueTail;
	QUICK_LIST queue;
} sm_dispatch_t;

// Adaptive Routing control settings
typedef struct {
	uint8_t enable;
	uint8_t debug;
	uint8_t lostRouteOnly;
	uint8_t algorithm;
	uint8_t	arFrequency;
	uint8_t threshold;
} SmAdaptiveRouting_t;

//
// Used for LMC port selection during routing
//

typedef struct SwitchportToNextGuid
{
	uint64_t guid;
	uint64_t sysGuid;
	uint16_t sortIndex;
	Port_t   *portp;
	Node_t	 *nextSwp;
} SwitchportToNextGuid_t;

//
// Use for slsc/vlarb setup
//
typedef struct _VlVfMap
{
    int8_t       vf[STL_MAX_VLS][MAX_VFABRICS];
} VlVfMap_t;

typedef struct _VlBwMap
{
    uint8_t       bw[STL_MAX_VLS];
    uint8_t       highPriority[STL_MAX_VLS];
} VlBwMap_t;

//
// Use for vlarb/bw setup
//
typedef struct _VlarbList
{
	uint8_t				cap;		// Low BW cap
	uint8_t				mtu;		// MTU of this port 
    STL_VLARB_TABLE_ELEMENT vlarbLow[STL_MAX_LOW_CAP];  // vlarb bw table
	struct _VlarbList	*next;
} VlarbList_t;

typedef struct Qos
{
	uint8_t		numVLs;
	uint8_t		activeVLs;
	STL_SCVLMAP	scvl;  /* scvlr = scvl */
	bitset_t	highPriorityVLs;
	bitset_t	lowPriorityVLs;
	uint8_t		dimensions;		// used for dor only
	VlBwMap_t	vlBandwidth;
	uint8_t		weightMultiplier;	// used for STL1 vlarb
	VlVfMap_t	vlvf;
	VlarbList_t	*vlarbList;		// Used to cache common vlarb bw data
} Qos_t;

typedef enum {
	SM_SWEEP_REASON_INIT = 0,
	SM_SWEEP_REASON_SCHEDULED,
	SM_SWEEP_REASON_RECONFIG,
	SM_SWEEP_REASON_MCMEMBER,
	SM_SWEEP_REASON_ACTIVATE_FAIL,
	SM_SWEEP_REASON_ROUTING_FAIL,
	SM_SWEEP_REASON_MC_ROUTING_FAIL,
	SM_SWEEP_REASON_UNEXPECTED_BOUNCE,
	SM_SWEEP_REASON_LOCAL_PORT_FAIL,
	SM_SWEEP_REASON_STATE_TRANSITION,
	SM_SWEEP_REASON_SECONDARY_TROUBLE,
	SM_SWEEP_REASON_MASTER_TROUBLE,
	SM_SWEEP_REASON_UPDATED_STANDBY,
	SM_SWEEP_REASON_HANDOFF,
	SM_SWEEP_REASON_UNEXPECTED_SM,
	SM_SWEEP_REASON_FORCED,
	SM_SWEEP_REASON_INTERVAL_CHANGE,
	SM_SWEEP_REASON_FAILED_SWEEP,
	SM_SWEEP_REASON_TRAP_EVENT,
	SM_SWEEP_REASON_UNDETERMINED
} SweepReason_t;

extern SweepReason_t sm_resweep_reason;

static __inline__ void setResweepReason(SweepReason_t reason) {
	if (sm_resweep_reason == SM_SWEEP_REASON_UNDETERMINED)
		sm_resweep_reason = reason;
}

//
// API for interacting with topology_activate()'s retry logic.
//

typedef struct ActivationRetry * pActivationRetry_t;

// returns the number of attempts made so far
// 0 = initial, 1 = 1st retry, etc.
uint8_t activation_retry_attempts(pActivationRetry_t);

// increments the failure count for the current attempt
void activation_retry_inc_failures(pActivationRetry_t);

//
//    IEEE defined OUIs
//
#define	OUI_INTEL		0x00d0b7

#define	INCR_PORT_COUNT(TP,NP) {			\
    TP->num_ports++;                        \
	if (NP->nodeInfo.NodeType == NI_TYPE_CA) {           \
        TP->num_endports++;                 \
    }                                       \
}

#define	DECR_PORT_COUNT(TP,NP) {			\
    TP->num_ports--;                        \
	if (NP->nodeInfo.NodeType == NI_TYPE_CA) {           \
        TP->num_endports--;                 \
    }                                       \
}

//
//	Macros for path resolution.
//
//
/* using switches only in floyd algo, requiring index in switch list instead of node index */
#define	Index(X,Y)		(((X) * sm_topop->max_sws) + (Y))
#define	Cost_Infinity	(32767)  /* small enough to ensure that inf+inf does not overflow */
#define	Min(X,Y)		((X) < (Y)) ? (X) : (Y)
#define	Max(X,Y)		((X) > (Y)) ? (X) : (Y)

#define	PathToPort(NP,PP)	((NP)->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 	\
				(NP)->path : (PP)->path

//
//	Macros for SM filter creation.
//
#define	SM_Filter_Init(FILTERP) {						\
	Filter_Init(FILTERP, 0, 0);						\
										\
	(FILTERP)->active |= MAI_ACT_ADDRINFO;					\
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
	(FILTERP)->dev = sm_config.hca;						\
	(FILTERP)->port = (sm_config.port == 0) ? MAI_TYPE_ANY : sm_config.port;		\
	(FILTERP)->qp = 0;							\
}

//
//	Macros for Node enqueueing.
//
#define	Node_Enqueue(TOPOP,NODEP,HEAD,TAIL) {					\
	if (TOPOP->TAIL == NULL) {						\
		NODEP->next = NULL;						\
		NODEP->prev = NULL;						\
		TOPOP->HEAD = NODEP;						\
		TOPOP->TAIL = NODEP;						\
	} else {								\
		NODEP->next = NULL;						\
		NODEP->prev = TOPOP->TAIL;					\
		(TOPOP->TAIL)->next = NODEP;					\
		TOPOP->TAIL = NODEP;						\
	}									\
}

#define	Node_Enqueue_Type(TOPOP,NODEP,HEAD,TAIL) {				\
	if (TOPOP->TAIL == NULL) {						\
		NODEP->type_next = NULL;					\
		NODEP->type_prev = NULL;					\
		TOPOP->HEAD = NODEP;						\
		TOPOP->TAIL = NODEP;						\
	} else {								\
		NODEP->type_next = NULL;					\
		NODEP->type_prev = TOPOP->TAIL;					\
		(TOPOP->TAIL)->type_next = NODEP;				\
		TOPOP->TAIL = NODEP;						\
	}									\
}

void Switch_Enqueue_Type(Topology_t *, Node_t *, int, int);

//
//	Macros for allocating Nodes and Ports.
//
#define sm_dynamic_port_alloc() \
    (sm_config.dynamic_port_alloc)

#define STL_MFTABLE_POSITION_COUNT 4
#define	Node_Create(TOPOP,NODEP,NODEINFO,TYPE,COUNT,NODEDESC,AUTHENTIC) {			\
	int		local_i;													\
	size_t		local_size;												\
	Status_t	local_status;											\
	char		nodeDescStr[ND_LEN+1];									\
																		\
	local_size = sizeof(Node_t) + (COUNT) * sizeof(Port_t) + 16;	\
	local_status = vs_pool_alloc(&sm_pool, local_size, (void *)&NODEP);	\
	if (local_status == VSTATUS_OK) {									\
		memset((void *)NODEP, 0, local_size);							\
		NODEP->nodeInfo = NODEINFO;										\
		NODEP->nodeDesc = NODEDESC; \
		memcpy(nodeDescStr, NODEDESC.NodeString, ND_LEN);					\
		if (strlen(nodeDescStr) == ND_LEN) {							\
			local_status = vs_pool_alloc(&sm_pool, ND_LEN+1, (void *)&NODEP->nodeDescString);	\
			if (local_status != VSTATUS_OK) {							\
                IB_FATAL_ERROR("Can't allocate space for node's description"); 	\
			}															\
			memcpy((void *)NODEP->nodeDescString, nodeDescStr, ND_LEN+1);	\
		}																\
        if (AUTHENTIC) {                                                  \
    		if (TYPE == NI_TYPE_CA) {										\
    			Node_Enqueue_Type(TOPOP, NODEP, ca_head, ca_tail);			\
    		} else if (TYPE == NI_TYPE_SWITCH) {							\
    		  if (sm_mcast_mlid_table_cap) {							\
                local_status = vs_pool_alloc(&sm_pool, sizeof(STL_PORTMASK*) * sm_mcast_mlid_table_cap, (void*)&NODEP->mft); \
                if (local_status == VSTATUS_OK) { 						\
                    local_status = vs_pool_alloc(&sm_pool, sizeof(STL_PORTMASK) * sm_mcast_mlid_table_cap * STL_MFTABLE_POSITION_COUNT, (void*)&NODEP->mft[0]); \
                    if (local_status == VSTATUS_OK) { 					\
                        memset((void *)NODEP->mft[0], 0, (sizeof(STL_PORTMASK) * sm_mcast_mlid_table_cap * STL_MFTABLE_POSITION_COUNT));	\
                        for (local_i = 1; local_i < sm_mcast_mlid_table_cap; ++local_i) { 	\
                            NODEP->mft[local_i] = NODEP->mft[local_i - 1] + STL_MFTABLE_POSITION_COUNT; 	\
                        } 												\
                        Node_Enqueue_Type(TOPOP, NODEP, switch_head, switch_tail);	\
                    } else {											\
                        IB_FATAL_ERROR("Can't allocate space for node's mft"); 	\
                    }													\
                } else {												\
                    IB_FATAL_ERROR("Can't allocate space for node's mft pointers"); \
                }														\
    		  } else {														\
                Node_Enqueue_Type(TOPOP, NODEP, switch_head, switch_tail);	\
    		  } 															\
    		}																\
    		if (!bitset_init(&sm_pool, &NODEP->activePorts, COUNT) ||		\
    			!bitset_init(&sm_pool, &NODEP->initPorts, COUNT) ||			\
    			!bitset_init(&sm_pool, &NODEP->vfMember, MAX_VFABRICS) ||	\
    			!bitset_init(&sm_pool, &NODEP->fullPKeyMember, MAX_VFABRICS)) {	\
    			IB_FATAL_ERROR("Can't allocate space for node's activePorts"); 	\
    		}																\
    		if (local_status == VSTATUS_OK) {								\
    																		\
    			Node_Enqueue(TOPOP, NODEP, node_head, node_tail);			\
    																		\
    		}																\
		}																	\
		if (local_status == VSTATUS_OK) {									\
			int portBytes;													\
			NODEP->port = (Port_t *)(NODEP+1);						        \
    		NODEP->uniformVL = 1;										\
			for (local_i = 0; local_i < (int)(COUNT); local_i++) {		\
   				(NODEP->port)[local_i].state = IB_PORT_NOP;			    \
   				(NODEP->port)[local_i].path[0] = 0xff;				    \
   				(NODEP->port)[local_i].index = local_i;				    \
                (NODEP->port)[local_i].nodeno = -1;                     \
                (NODEP->port)[local_i].portno = -1;                     \
                if (sm_dynamic_port_alloc()) {                          \
                     (NODEP->port)[local_i].portData = NULL;             \
                     if (local_i == sm_config.port) {                           \
                         if (((NODEP->port)[local_i].portData = sm_alloc_port(TOPOP, NODEP, local_i, &portBytes)) == NULL) {  \
                              IB_FATAL_ERROR("Can't allocate SM port for the node"); 	\
                         }                                               \
                     }                                                   \
                } else {                                                \
                     if (((NODEP->port)[local_i].portData = sm_alloc_port(TOPOP, NODEP, local_i, &portBytes)) == NULL) {  \
                           IB_FATAL_ERROR("Can't allocate port for the node"); 	\
                    }                                                   \
                }                                                       \
			}															\
            (NODEP->port)[0].nodeno = topop->num_nodes;             \
            (NODEP->port)[0].portno = 0;                            \
		}																	\
	} else { 															\
		IB_FATAL_ERROR("can't allocate space");							\
	} 																	\
}

#define	Node_Delete(TOPOP, NODEP) {							\
	Status_t	local_status;						\
										\
	if (NODEP->mft) { \
		local_status = vs_pool_free(&sm_pool, NODEP->mft[0]); \
		if (local_status != VSTATUS_OK) { \
			IB_FATAL_ERROR("Failed to free node's mft"); \
		} \
		local_status = vs_pool_free(&sm_pool, NODEP->mft); \
		if (local_status != VSTATUS_OK) { \
			IB_FATAL_ERROR("Failed to free node's mft pointers"); \
		} \
	} \
    if (NODEP->lft) { \
        local_status = vs_pool_free(&sm_pool, NODEP->lft); \
        if (local_status != VSTATUS_OK) { \
            IB_FATAL_ERROR("Failed to free node's lft"); \
        } \
    } \
	if (NODEP->pgt)	{	\
		vs_pool_free(&sm_pool, NODEP->pgt);		\
	}\
	if (NODEP->pgft)	{	\
		sm_Node_release_pgft(NODEP);		\
	}\
	if (NODEP->routingData)	{	\
		vs_pool_free(&sm_pool, NODEP->routingData);	\
	}\
	if (NODEP->nodeDescString)	{	\
		vs_pool_free(&sm_pool, NODEP->nodeDescString);	\
	}\
	if (NODEP->portStateInfo) {  \
		vs_pool_free(&sm_pool, NODEP->portStateInfo); \
	}\
	bitset_free(&NODEP->activePorts);					\
	bitset_free(&NODEP->initPorts);						\
	bitset_free(&NODEP->vfMember);						\
	bitset_free(&NODEP->fullPKeyMember);				\
	sm_node_free_port(TOPOP, NODEP);   \
	sm_node_release_changes((NODEP));   \
	local_status = vs_pool_free(&sm_pool, (void *)NODEP);			\
	if (local_status != VSTATUS_OK) {					\
		IB_FATAL_ERROR("can't free space");				\
	}									\
}

#define	Node_Quarantined_Delete(NODEP) {							        \
    if (NODEP->quarantinedNode)	{	                                        \
    	if (NODEP->quarantinedNode->nodeDescString)	{	                    \
    		vs_pool_free(&sm_pool, NODEP->quarantinedNode->nodeDescString);	\
    	}                                                                   \
    	if (vs_pool_free(&sm_pool, (void *)NODEP->quarantinedNode)) {       \
    		IB_FATAL_ERROR("can't free space");                             \
    	}                                                                   \
	}                                                                       \
    if (vs_pool_free(&sm_pool, (void *)NODEP)) {                            \
        IB_FATAL_ERROR("can't free space");                                 \
    }                                                                       \
}


//
//      Topology traversal macros.
//
#define for_all_nodes(TP,NP)							\
	for (NP = (TP)->node_head; NP != NULL; NP = NP->next)

#define for_all_ca_nodes(TP,NP)							\
	for (NP = (TP)->ca_head; NP != NULL; NP = NP->type_next)

#define for_all_edge_switches(TP,NP)                            \
    for (NP = (TP)->tier_head[0]; NP != NULL; NP = NP->sw_next)

#define for_all_tier_switches(TP,NP, tier)                           \
    for (NP = (TP)->tier_head[tier]; NP != NULL; NP = NP->sw_next)

#define for_all_switch_nodes(TP,NP)						\
	for (NP = (TP)->switch_head; NP != NULL; NP = NP->type_next)

#define for_switch_list_switches(HD, NP)					\
	for (NP = (HD); NP != NULL; NP = NP->next)

#define for_all_quarantined_nodes(TP,NP)					\
	for (NP = (TP)->quarantined_node_head; NP != NULL; NP = NP->next)
// --------------------------------------------------------------------------- //
static __inline__ int sm_valid_port(Port_t * portp) {
    return ((portp != NULL) && (portp->portData != NULL));
}

static __inline__ int sm_valid_port_mgmt_allowed_pkey(Port_t * portp) {
	if (portp->portData->pPKey[STL_DEFAULT_FM_PKEY_IDX].AsReg16 == STL_DEFAULT_FM_PKEY ||
		portp->portData->pPKey[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 == STL_DEFAULT_FM_PKEY)
		return 1;
    else
		return 0;
}


#ifndef __VXWORKS__
static inline Port_t * sm_get_port(const Node_t *nodep, uint32_t portIndex) {
    if (portIndex <= nodep->nodeInfo.NumPorts) {
        return &nodep->port[portIndex];
    }
    return NULL;
}
#else
Port_t *    sm_get_port(const Node_t *nodep, uint32_t portIndex);
#endif

static __inline__ int Is_Switch_Queued(Topology_t *tp, Node_t *nodep) {
	int	t;	
	if (nodep->sw_prev) return 1;
	for (t=0; t<MAX_TIER;t++) {	
		if (tp->tier_head[t] == nodep) return 1;
	}
	return 0;
}

static __inline__ STL_LID sm_port_top_lid(Port_t * portp) {
	return ((!portp) ? 0 : portp->portData->lid + (1 << portp->portData->lmc) - 1);
}

#define	PORT_A0(NP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : 1)

#define	PORT_A1(NP)								\
	NP->nodeInfo.NumPorts

#define for_all_ports(NP,PP)							\
	if((PP = sm_get_port(NP,PORT_A0(NP))) == NULL) {}			\
		else for (;PP <= sm_get_port(NP,PORT_A1(NP)); PP++)


// --------------------------------------------------------------------------- //
// useful when sm_find_node_and_port_lid has returned a matching port (MPP)
// and want to walk all matching ports in given device. For a HFI this walks
// just the matched port (since other ports will have different LIDs) and for
// a switch it matches all ports (since only port 0 had a LID)
#define	PORT_A0_MATCHED(NP, MPP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : MPP->index)

#define	PORT_A1_MATCHED(NP, MPP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? NP->nodeInfo.NumPorts : MPP->index)

#define for_all_matched_ports(NP,PP,MPP)							\
	if((PP = sm_get_port(NP,PORT_A0_MATCHED(NP, MPP))) == NULL) {}				\
		else for (;PP <= sm_get_port(NP,PORT_A1_MATCHED(NP, MPP)); PP++)

// --------------------------------------------------------------------------- //

#define	PORT_E0(NP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : 1)

#define	PORT_E1(NP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : NP->nodeInfo.NumPorts)

#define for_all_end_ports(NP,PP)						\
	if((PP = sm_get_port(NP,PORT_E0(NP))) == NULL) {}			\
		else for (;PP <= sm_get_port(NP,PORT_E1(NP)); PP++)

#define for_all_end_ports2Nodes(NP1, PP1, NP2, PP2)		\
	if ((PP1 = sm_get_port(NP1,PORT_E0(NP1))) == NULL || (PP2 = sm_get_port(NP2,PORT_E0(NP2))) == NULL) {}\
		else for (;PP1 <= sm_get_port(NP1,PORT_E1(NP1)); PP1++, PP2++)
	
static __inline__ Port_t * sm_get_node_end_port(Node_t *nodep) {
	Port_t *portp = NULL;
	for_all_end_ports(nodep, portp) {
		if(sm_valid_port(portp))
			return portp;
	}
	return NULL;
}

// --------------------------------------------------------------------------- //

#define	PORT_P0(NP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 1 : 1)

#define	PORT_P1(NP)								\
	NP->nodeInfo.NumPorts

#define for_all_physical_ports(NP,PP)						\
	if((PP = sm_get_port(NP,PORT_P0(NP))) == NULL) {}			\
		else for (;PP <= sm_get_port(NP,PORT_P1(NP)); PP++)

// --------------------------------------------------------------------------- //

#define	PORT_S0(NP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : 1)

#define	PORT_S1(NP)								\
	((NP->nodeInfo.NodeType == NI_TYPE_SWITCH) ? 0 : NP->nodeInfo.NumPorts)

#define for_all_sma_ports(NP,PP)						\
	if((PP = sm_get_port(NP,PORT_S0(NP))) == NULL) {}			\
		else for (;PP <= sm_get_port(NP,PORT_S1(NP)); PP++)

// --------------------------------------------------------------------------- //

#define for_all_port_lids(PP,X)							\
	for ((X) = (PP)->portData->lid; (X) < (int)((PP)->portData->lid + (1 << (PP)->portData->lmc)); (X)++)

// --------------------------------------------------------------------------- //

//
//	Macros for allocating Multicast Groups and Members
//
#define	McGroup_Enqueue(GROUPP) {						\
	if (sm_McGroups == NULL) {						\
		GROUPP->next = NULL;						\
		sm_McGroups = GROUPP;						\
	} else {								\
		GROUPP->next = sm_McGroups;					\
		sm_McGroups = GROUPP;						\
	}									\
}

#define	McGroup_Dequeue(GROUPP) {						\
	McGroup_t		*localGroup;					\
										\
	if (sm_McGroups == GROUPP) {						\
		sm_McGroups = (GROUPP)->next;					\
	} else {								\
		for_all_multicast_groups(localGroup) {				\
			if (localGroup->next == GROUPP) {			\
				localGroup->next = (GROUPP)->next;		\
				break;						\
			}							\
		}								\
	}									\
}

#define	McGroup_Create(GROUPP) { 						\
	size_t		local_size;						\
	Status_t	local_status;						\
										\
	local_size = sizeof(McGroup_t) + 16;					\
	local_status = vs_pool_alloc(&sm_pool, local_size, (void *)&GROUPP);	\
	if (local_status != VSTATUS_OK) {					\
		IB_FATAL_ERROR("can't allocate space");				\
	}									\
										\
	memset((void *)GROUPP, 0, local_size);					\
	if (!bitset_init(&sm_pool, &GROUPP->vfMembers, MAX_VFABRICS)) { \
		IB_FATAL_ERROR("McGroup_Create: can't allocate space");				\
	}									\
	McGroup_Enqueue(GROUPP);						\
	++sm_numMcGroups; \
}

#define	McGroup_Delete(GROUPP) {		\
	Status_t	local_status;			\
										\
	--sm_numMcGroups;					\
										\
	sm_multicast_decommision_group(GROUPP); \
	McGroup_Dequeue(GROUPP);						\
	bitset_free(&GROUPP->vfMembers); \
	local_status = vs_pool_free(&sm_pool, (void *)GROUPP);			\
	if (local_status != VSTATUS_OK) {					\
		IB_FATAL_ERROR("can't free space");				\
	}									\
}

#define	McMember_Enqueue(GROUPP,MEMBERP) {					\
	if (GROUPP->mcMembers == NULL) {					\
		MEMBERP->next = NULL;						\
		GROUPP->mcMembers = MEMBERP;					\
	} else {								\
		MEMBERP->next = GROUPP->mcMembers;				\
		GROUPP->mcMembers = MEMBERP;					\
	}									\
}

#define	McMember_Dequeue(GROUPP,MEMBERP) {					\
	McMember_t		*localMember;					\
										\
	if ((GROUPP)->mcMembers == MEMBERP) {					\
		(GROUPP)->mcMembers = (MEMBERP)->next;				\
	} else {								\
		for_all_multicast_members(GROUPP, localMember) {		\
			if (localMember->next == MEMBERP) {			\
				localMember->next = (MEMBERP)->next;		\
				break;						\
			}							\
		}								\
	}									\
}

#define	McMember_Create(GROUPP,MEMBERP) { 					\
	size_t		local_size;						\
	Status_t	local_status;						\
										\
	local_size = sizeof(McMember_t) + 16;					\
	local_status = vs_pool_alloc(&sm_pool, local_size, (void *)&MEMBERP);	\
	if (local_status != VSTATUS_OK) {					\
		IB_FATAL_ERROR("can't allocate space");				\
	}									\
										\
	memset((void *)MEMBERP, 0, local_size);					\
	MEMBERP->index = ++GROUPP->index_pool;				 \
	McMember_Enqueue(GROUPP, MEMBERP);					\
}

#define	McMember_Delete(GROUPP,MEMBERP) {					\
	Status_t	local_status;						\
										\
	McMember_Dequeue(GROUPP, MEMBERP);					\
	local_status = vs_pool_free(&sm_pool, (void *)MEMBERP);			\
	if (local_status != VSTATUS_OK) {					\
		IB_FATAL_ERROR("can't free space");				\
	}									\
}

// --------------------------------------------------------------------------- //

#define for_all_multicast_groups(GP)						\
	for (GP = sm_McGroups; GP != NULL; GP = GP->next)

// --------------------------------------------------------------------------- //

#define for_all_multicast_members(GP,MP)					\
	for (MP = GP->mcMembers; MP != NULL; MP = MP->next)

// --------------------------------------------------------------------------- //

//
//	Miscellaneous macros
//
#define	IsDirectConnected(SN,SP,DN,DP)						\
	((((SP)->nodeno == (DN)->index) && ((SP)->portno == (DP)->index)) &&		\
	(((DP)->nodeno == (SN)->index) && ((DP)->portno == (SP)->index)))

// Converts a rate into a "speed factor" used in cost calculations.
// factor ~= rate * 64/66 * 2.
// Currently only handles 12.5 and 25G 
#define	Decode_SpeedFactor(Z) \
	(((Z) == STL_LINK_SPEED_12_5G) ? 24: 50)

// Currently only handles 12.5 and 25G 
#define Decode_Speed(Z) \
		(((Z) == STL_LINK_SPEED_12_5G) ? 12500000000ull : 25781250000ull)
			 
// Cost should be evenly divisable by (width * SpeedFactor)
#define	SpeedWidth_to_Cost(X)	((X) ? 2400/(X) : 2400)

static __inline__ uint8_t sm_GetLsf(STL_PORT_INFO *portInfo) {
	return (Decode_SpeedFactor(portInfo->LinkSpeed.Active));
}

static __inline__ uint64_t sm_GetBandwidth(STL_PORT_INFO *portInfo) {
uint64_t bw;
	// Calculate bytes/sec BW: speed * width * 64/66 (encoding overhead) * 1/8 (bits to bytes)
	bw = Decode_Speed(portInfo->LinkSpeed.Active)*StlLinkWidthToInt(portInfo->LinkWidth.Active)*4/33;
	return bw;
}

static __inline__ uint16_t sm_GetSpeed(PortData_t *portData) {
	return (portData->lsf * StlLinkWidthToInt(portData->portInfo.LinkWidth.Active));
}

static __inline__ uint16_t sm_GetCost(PortData_t *portData) {
	return (SpeedWidth_to_Cost(sm_GetSpeed(portData)));
}

static __inline__ char* sm_nodeDescString(Node_t *nodep) {
	return ((nodep->nodeDescString) ? nodep->nodeDescString : (char *)nodep->nodeDesc.NodeString);
}

static __inline__ int sm_stl_port(Port_t *portp) { 
    return (portp->portData->portInfo.PortLinkMode.s.Active & STL_PORT_LINK_MODE_STL);
}

static __inline__ int sm_IsInterleaveEnabled(STL_PORT_INFO *portInfo) {
	return(portInfo->PortMode.s.IsVLMarkerEnabled ||
		portInfo->FlitControl.Interleave.s.MaxNestLevelTxEnabled);
}

static __inline__ int sm_stl_appliance(uint64 nodeGuid) { 
    int i, appliance = 0; 

    if (sm_config.appliances.enable && nodeGuid) {
        for (i = 0; i < MAX_SM_APPLIANCES; i++) {
            if (sm_config.appliances.guids[i] == nodeGuid)
                appliance = 1;
        }
    }

    return appliance;
}

static __inline__ int sm_is_scae_allowed(Node_t * nodep) {
	return sm_config.switchCascadeActivateEnable == SCAE_ALL
		|| (sm_config.switchCascadeActivateEnable == SCAE_SW_ONLY && nodep->nodeInfo.NodeType == STL_NODE_SW);
}

// --------------------------------------------------------------------------- //

//
//	Externs
//
extern SMXmlConfig_t		sm_config;
extern PMXmlConfig_t		pm_config;
extern FEXmlConfig_t		fe_config;
extern SMDPLXmlConfig_t 	sm_dpl_config;
extern SMMcastConfig_t 		sm_mc_config;
extern SmMcastMlidShare_t 	sm_mls_config;
extern SMMcastDefGrpCfg_t	sm_mdg_config;

extern 	uint32_t 	sm_def_mc_group;
extern  uint8_t		sm_env[32];
extern	STL_LID 	sm_lid;
extern	STL_LID 	sm_lmc_0_freeLid_hint;
extern	STL_LID 	sm_lmc_e0_freeLid_hint;
extern	STL_LID 	sm_lmc_freeLid_hint;
extern	STL_LID 	sm_lid_lo;
extern	STL_LID 	sm_lid_hi;
extern	uint32_t	sm_state;
extern  uint32_t	sm_prevState;
extern  int			sm_saw_another_sm;
extern  int			sm_hfi_direct_connect;
extern  uint8_t     sm_mkey_protect_level;
extern  uint16_t    sm_mkey_lease_period;
extern	uint64_t	sm_masterCheckInterval;
extern  uint32_t    sm_masterCheckMaxFail;
extern	uint32_t	sm_mcDosThreshold;
extern	uint32_t	sm_mcDosAction;
extern	uint64_t	sm_mcDosInterval;
extern	uint64_t	sm_portguid;
extern	STL_SM_INFO	sm_smInfo;
extern	uint32_t	sm_masterStartTime;
extern	uint8_t		sm_description[65];
extern	LidMap_t	* lidmap;
extern	cl_qmap_t	* sm_GuidToLidMap;

extern	uint32_t	topology_passcount;

extern 	boolean		vf_config_changed;
extern 	boolean		reconfig_in_progress;
extern	Lock_t		old_topology_lock;	// a RW Thread Lock
extern	Lock_t		new_topology_lock;	// a Thread Lock

extern	Lock_t		handover_sent_lock;
extern	uint32_t	handover_sent;
extern	uint32_t	triggered_handover;

#ifdef __LINUX__
extern Lock_t linux_shutdown_lock; // a RW Thread Lock
#endif

extern	Topology_t	old_topology;
extern	Topology_t	sm_newTopology;
extern  Topology_t  *sm_topop;

extern  boolean     sweepsPaused;

extern  sm_dispatch_t sm_asyncDispatch;


extern  uint32_t    sm_log_level;
extern 	uint32_t	sm_log_level_override;
extern  uint32_t	sm_log_masks[VIEO_LAST_MOD_ID+1];
extern  int			sm_log_to_console;
extern  char		sm_config_filename[256];

extern	Sema_t		topology_sema;
extern	Sema_t		topology_rcv_sema;
extern	uint64_t	topology_sema_setTime;
extern	uint64_t	topology_sema_runTime;
extern	uint64_t	topology_wakeup_time;
extern  Lock_t		sa_lock;
extern uint32_t     sm_debug;
extern  uint32_t    smDebugPerf;  // control SM/SA performance messages; default off in ESM
extern  uint32_t    smFabricDiscoveryNeeded;
extern  uint32_t    smDebugDynamicAlloc; // control SM/SA memory allocation messages; default off in ESM
extern  uint32_t    sm_trapThreshold; // threshold of traps/min for port auto-disable
extern  uint32_t	sm_trapThreshold_minCount; //minimum number of traps to validate sm_trapThreshold
extern  uint64_t    sm_trapThresholdWindow; // time window for observing traps in milliseconds
extern	STL_LID 	sm_mcast_mlid_table_cap;

extern	uint16_t    sm_masterSmSl;
extern  bitset_t	sm_linkSLsInuse;
extern	int			sm_QosConfigChange;

extern SmAdaptiveRouting_t sm_adaptiveRouting;

extern SmDorRouting_t smDorRouting;

extern uint32_t	sm_useIdealMcSpanningTreeRoot;
extern uint32_t	sm_mcSpanningTreeRoot_useLeastWorstCaseCost;
extern uint32_t	sm_mcSpanningTreeRoot_useLeastTotalCost;
extern uint32_t	sm_mcRootCostDeltaThreshold;
/************ Multicast Globals *********************************/
extern uint32_t sm_numMcGroups;
extern uint32_t sm_McGroups_Need_Prog;
extern	McGroup_t	*sm_McGroups;
extern  Lock_t      sm_McGroups_lock;

extern uint64_t sm_mcSpanningTreeRootGuid;
extern Lock_t sm_mcSpanningTreeRootGuidLock;
/************ DOR Globals ***************************************/
extern Lock_t sm_datelineSwitchGUIDLock;
extern uint64_t sm_datelineSwitchGUID;

/************ dynamic update of switch config parms *************/
extern uint8_t sa_dynamicPlt[];

/************ loop test externs from sm_userexits ***************/
extern uint16_t loopPathLidStart;
extern uint16_t loopPathLidEnd;
extern int esmLoopTestOn;
extern int esmLoopTestAllowAdaptiveRouting;
extern int esmLoopTestFast;
extern int esmLoopTestNumPkts;
extern int esmLoopTestPathLen;
extern uint32_t esmLoopTestTotalPktsInjected;
extern int esmLoopTestInjectNode;
extern uint8_t	esmLoopTestInjectEachSweep;
extern uint8_t	esmLoopTestForceInject;
extern int    	esmLoopTestMinISLRedundancy;
//
//	Miscellaneous stuff.
//
//#define	MAD_RETRIES		4     // moved to "ib/include/ib_const.h"

static __inline__ void block_sm_exit(void) {
	#ifdef __LINUX__
	vs_rdlock(&linux_shutdown_lock);
	#endif
}

static __inline__ void unblock_sm_exit(void) {
	#ifdef __LINUX__
	vs_rwunlock(&linux_shutdown_lock);
	#endif
}
//
//	Convenience macros for Get(*) and Set(*)
//
Status_t	SM_Get_NodeDesc(IBhandle_t, uint32_t, uint8_t *, STL_NODE_DESCRIPTION *);
Status_t	SM_Set_NodeDesc(IBhandle_t, uint32_t, uint8_t *, NodeDesc_t *, uint64_t);
Status_t	SM_Get_NodeInfo(IBhandle_t, uint32_t, uint8_t *, STL_NODE_INFO *);
Status_t	SM_Set_NodeInfo(IBhandle_t, uint32_t, uint8_t *, STL_NODE_INFO *, uint64_t);
Status_t	SM_Get_PortInfo(IBhandle_t, uint32_t, uint8_t *, STL_PORT_INFO *);
Status_t	SM_Get_PortInfo_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_PORT_INFO *pip);
Status_t	SM_Set_PortInfo(IBhandle_t, uint32_t, uint8_t *, STL_PORT_INFO *, uint64_t, int);
Status_t	SM_Set_PortInfo_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_PORT_INFO *pip, uint64_t mkey, uint32_t* madStatus);
Status_t	SM_Get_PortStateInfo(IBhandle_t, uint32_t, uint8_t *, STL_PORT_STATE_INFO *);
Status_t	SM_Get_PortStateInfo_LR(IBhandle_t, uint32_t, uint16_t, uint16_t, STL_PORT_STATE_INFO *);
Status_t 	SM_Set_PortStateInfo(IBhandle_t, uint32_t, uint8_t *, STL_PORT_STATE_INFO *, uint64_t);
Status_t 	SM_Set_PortStateInfo_LR(IBhandle_t, uint32_t, uint16_t, uint16_t, STL_PORT_STATE_INFO *, uint64_t);
Status_t	SM_Get_SwitchInfo(IBhandle_t, uint32_t, uint8_t *, STL_SWITCH_INFO *);
Status_t	SM_Set_SwitchInfo(IBhandle_t, uint32_t, uint8_t *, STL_SWITCH_INFO *, uint64_t);
Status_t	SM_Get_SMInfo(IBhandle_t, uint32_t, uint8_t *, STL_SM_INFO *);
Status_t	SM_Set_SMInfo(IBhandle_t, uint32_t, uint8_t *, STL_SM_INFO *, uint64_t);
Status_t	SM_Get_VLArbitration(IBhandle_t, uint32_t, uint8_t *, STL_VLARB_TABLE *);
Status_t	SM_Get_VLArbitration_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_VLARB_TABLE *vlp);
Status_t	SM_Set_VLArbitration(IBhandle_t, uint32_t, uint8_t *, STL_VLARB_TABLE *, uint64_t);

/**
	Does not implement multiblock requests.

	@param vlpSize the amount of data to copy back from the SMA into @c vlp.
*/
Status_t	SM_Set_VLArbitration_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_VLARB_TABLE *vlp, size_t vlpSize, uint64_t mkey);
Status_t	SM_Get_SLSCMap(IBhandle_t, uint32_t, uint8_t *, STL_SLSCMAP *);
Status_t	SM_Get_SLSCMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SLSCMAP *slscp);
Status_t	SM_Set_SLSCMap(IBhandle_t, uint32_t, uint8_t *, STL_SLSCMAP *, uint64_t);
Status_t	SM_Set_SLSCMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SLSCMAP *slscp, uint64_t mkey);
Status_t	SM_Get_SCSLMap(IBhandle_t, uint32_t, uint8_t *, STL_SCSLMAP *);
Status_t	SM_Get_SCSLMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SCSLMAP *scslp);
Status_t	SM_Set_SCSLMap(IBhandle_t, uint32_t, uint8_t *, STL_SCSLMAP *, uint64_t);
Status_t	SM_Set_SCSLMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SCSLMAP *scslp, uint64_t mkey);
Status_t	SM_Get_SCVLtMap(IBhandle_t, uint32_t, uint8_t *, STL_SCVLMAP *);
Status_t	SM_Get_SCVLtMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SCVLMAP *scvlp);
Status_t	SM_Set_SCVLtMap(IBhandle_t, uint32_t, uint8_t *, STL_SCVLMAP *, uint64_t);
Status_t	SM_Set_SCVLtMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SCVLMAP *scvlp, uint64_t mkey);
Status_t	SM_Get_SCVLntMap(IBhandle_t, uint32_t, uint8_t *, STL_SCVLMAP *);
Status_t	SM_Get_SCVLntMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SCVLMAP *scvlp);
Status_t	SM_Set_SCVLntMap(IBhandle_t, uint32_t, uint8_t *, STL_SCVLMAP *, uint64_t);
Status_t	SM_Set_SCVLntMap_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_SCVLMAP *scvlp, uint64_t mkey);
Status_t	SM_Get_SCVLrMap_LR(IBhandle_t, uint32_t, STL_LID, STL_LID, STL_SCVLMAP *);
Status_t	SM_Set_SCVLrMap_LR(IBhandle_t, uint32_t, STL_LID, STL_LID, STL_SCVLMAP *, uint64_t);
Status_t	SM_Set_SCSC_LR(IBhandle_t, uint32_t, STL_LID, STL_LID, STL_SCSCMAP *, uint64_t);
Status_t	SM_Set_LFT(IBhandle_t, uint32_t, uint8_t *, STL_LINEAR_FORWARDING_TABLE *, uint64_t, int);
Status_t	SM_Set_LFT_Dispatch_DR(IBhandle_t, uint32_t, uint8_t *, STL_LINEAR_FORWARDING_TABLE *, uint16_t, uint64_t, Node_t *, sm_dispatch_t *);
Status_t	SM_Set_LFT_Dispatch_LR(IBhandle_t, uint32_t, uint16_t, uint16_t, STL_LINEAR_FORWARDING_TABLE *,uint16_t, uint64_t, Node_t *, sm_dispatch_t *);
Status_t	SM_Set_MFT_Dispatch(IBhandle_t, uint32_t, uint8_t *, STL_MULTICAST_FORWARDING_TABLE *, uint64_t, Node_t *, sm_dispatch_t *);
Status_t	SM_Set_MFT_DispatchLR(IBhandle_t, uint32_t, uint16_t, uint16_t, STL_MULTICAST_FORWARDING_TABLE *, uint64_t, Node_t *, sm_dispatch_t *);
Status_t	SM_Get_PKeyTable(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_PARTITION_TABLE *pkp, int);
Status_t	SM_Set_PKeyTable(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_PARTITION_TABLE *pkp, uint64_t, int);
Status_t	SM_Get_PortGroup(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_PORT_GROUP_TABLE *pgp, uint8_t blocks);
Status_t	SM_Set_PortGroup(IBhandle_t fd, uint32_t amod, uint8_t *path, uint16_t slid, uint16_t dlid, STL_PORT_GROUP_TABLE *pgp, uint8_t blocks, uint64_t mkey);
Status_t	SM_Get_PortGroupFwdTable(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_PORT_GROUP_FORWARDING_TABLE *pp, uint8_t blocks);
Status_t	SM_Set_PortGroupFwdTable(IBhandle_t fd, uint32_t amod, uint8_t *path, uint16_t slid, uint16_t dlid, STL_PORT_GROUP_FORWARDING_TABLE *pp, uint8_t blocks, uint64_t mkey);
Status_t    SM_Get_BufferControlTable(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_BUFFER_CONTROL_TABLE pbct[]);
Status_t    SM_Set_BufferControlTable_LR(IBhandle_t fd, uint32_t amod, STL_LID slid, STL_LID dlid, STL_BUFFER_CONTROL_TABLE pbct[], uint64_t mkey, uint32_t* madStatus);
Status_t    SM_Set_BufferControlTable(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_BUFFER_CONTROL_TABLE pbct[], uint64_t mkey, uint32_t* madStatus);
Status_t	SM_Get_CongestionInfo(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_CONGESTION_INFO * congestionInfo);
Status_t	SM_Get_CongestionInfo_LR(IBhandle_t fd, uint32_t amod, STL_LID slid, STL_LID dlid, STL_CONGESTION_INFO * congestionInfo);
Status_t	SM_Get_HfiCongestionSetting(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_HFI_CONGESTION_SETTING *hfics);
Status_t	SM_Set_HfiCongestionSetting(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_HFI_CONGESTION_SETTING *hfics, uint64_t mkey);
Status_t	SM_Get_HfiCongestionControl(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_HFI_CONGESTION_CONTROL_TABLE *hficct);
Status_t	SM_Set_HfiCongestionControl(IBhandle_t fd, uint16 CCTI_Limit, const uint8_t numBlocks, uint32_t amod, uint8_t *path, STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK *hficct, uint64_t mkey);
Status_t	SM_Get_SwitchCongestionSetting(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_SWITCH_CONGESTION_SETTING *swcs);
Status_t	SM_Set_SwitchCongestionSetting(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_SWITCH_CONGESTION_SETTING *swcs, uint64_t mkey);
Status_t	SM_Get_SwitchPortCongestionSetting(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_SWITCH_PORT_CONGESTION_SETTING *swpcs);
Status_t	SM_Set_HfiCongestionSetting_LR(IBhandle_t fd, uint32_t amod, STL_LID slid, STL_LID dlid, STL_HFI_CONGESTION_SETTING *hfics, uint64_t mkey);
Status_t	SM_Set_HfiCongestionControl_LR(IBhandle_t fd, uint16 CCTI_Limit, const uint8_t numBlocks, uint32_t amod, STL_LID slid ,STL_LID dlid, STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK *hficct, uint64_t mkey);
Status_t	SM_Set_SwitchCongestionSetting_LR(IBhandle_t fd, uint32_t amod, STL_LID slid, STL_LID dlid, STL_SWITCH_CONGESTION_SETTING *swcs, uint64_t mkey);
Status_t	SM_Get_SwitchPortCongestionSetting_LR(IBhandle_t fd, uint32_t amod, STL_LID slid, STL_LID dlid, STL_SWITCH_PORT_CONGESTION_SETTING *swpcs);


Status_t SM_Set_LedInfo(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_LED_INFO *li, uint64_t mkey);
Status_t SM_Set_LedInfo_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_LED_INFO *li, uint64_t mkey);
Status_t SM_Get_LedInfo(IBhandle_t fd, uint32_t amod, uint8_t *path, STL_LED_INFO *li);
Status_t SM_Get_LedInfo_LR(IBhandle_t fd, uint32_t amod, uint16_t slid, uint16_t dlid, STL_LED_INFO *li);
/**
 	Do an SMA Get(CableInfo) on the specified endpoint.

	Operates in 64-byte half-pages even though CableInfo is stored in 128-byte pages.

 	@param startSeg the starting 64-byte segment
	@param segCount the number of 64-byte segments to request
 	@param [out] ci points to start of output.  Must have enough space for all request segments.
	@param [out] madStatus The MAD status of the Get(CableInfo); set to 0 at start
*/
Status_t SM_Get_CableInfo_LR(IBhandle_t fd, uint8_t portIdx, uint8_t startSeg, uint8_t segCount, uint16_t slid, uint16_t dlid, STL_CABLE_INFO * ci, uint32_t * madStatus);

Status_t SM_Get_CableInfo(IBhandle_t fd, uint8_t portIdx, uint8_t startSeg, uint8_t segCount, uint8_t *path, STL_CABLE_INFO * ci, uint32_t * madStatus);

/**
	Perform Get(Aggregate) with the requests in [start, end).  See @c SM_Aggregate_LR_impl() for more details.
*/
Status_t
SM_Get_Aggregate_LR(IBhandle_t fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	STL_LID_32 srcLid, STL_LID_32 destLid, STL_AGGREGATE ** lastSeg, uint32_t * madStatus);

/**
  Perform Set(Aggregate) with the requests in [start, end).  See @c SM_Aggregate_LR_impl() for more details.
*/
Status_t
SM_Set_Aggregate_LR(IBhandle_t fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	STL_LID_32 srcLid, STL_LID_32 destLid, uint64_t mkey, STL_AGGREGATE ** lastSeg, uint32_t * madStatus);

//
//	Multicast data structures.
//
typedef	struct _McNode {
	int32_t		index;		// node index
	int32_t		nodeno;		// closest node in the tree
	int32_t		portno;		// port on closest node
	int32_t		height; 	// distance from the root. 
	struct _McNode *parent; // parent in spanning tree
	STL_LID		mft_mlid_init;
} McNode_t;

typedef	struct _McSpanningTree {
	int32_t		num_nodes;
	McNode_t	*nodes;
	STL_LID		first_mlid;
	uint32_t	reserved; 	// Padding to make the size a multiple of 8 bytes
} McSpanningTree_t;

typedef struct McSpaningTrees {
	McSpanningTree_t	*spanningTree;
	uint8_t				copy;	// Is this a shallow copy of another tree?
} McSpanningTrees_t;

//
//	Externs.
//
extern	SMThread_t	* sm_threads;

extern	IBhandle_t	fd_sa;
extern	IBhandle_t	fd_sa_w;
extern	IBhandle_t	fd_multi;
extern	IBhandle_t	fd_async;
extern	IBhandle_t	fd_saTrap;
extern	IBhandle_t	fd_policy;
extern	IBhandle_t	fd_topology;
extern	IBhandle_t	fd_loopTest;
extern  IBhandle_t	fd_atopology;

extern	int ib_mad_dump_flag;	// JSY - debug only

extern McSpanningTree_t **uniqueSpanningTrees;
extern int	uniqueSpanningTreeCount;

extern Node_t *sm_mcSpanningTreeRootSwitch;
extern uint64_t sm_mcSpanningTreeRootGuid;

//
//	Prototypes.
//
const char *sm_getStateText (uint32_t state);
Status_t	sa_Get_PKeys(STL_LID, uint16_t *, uint32_t *);
Status_t	sa_Compare_PKeys(STL_PKEY_ELEMENT *, STL_PKEY_ELEMENT *);
Status_t	sa_Compare_Node_PKeys(Node_t *, Node_t *);
Status_t    sa_Compare_Port_PKeys(Port_t *port1, Port_t *port2);

uint8_t		linkWidthToRate(PortData_t *portData);

Status_t	sm_mkey_check(Mai_t *,  uint64_t *);
Status_t	sm_dump_mai(const char *, Mai_t *);
Status_t	sm_transition(uint32_t);

Status_t	sm_error(Status_t);
Status_t	sm_start_thread(SMThread_t *);

Status_t	mai_reply(IBhandle_t, Mai_t *);
Status_t	mai_stl_reply(IBhandle_t, Mai_t *, uint32_t);
//
// sm_topology.c prototypes
//

extern void sm_request_resweep(int, int, SweepReason_t);
extern void sm_trigger_sweep(SweepReason_t);
extern void sm_discovery_needed(const char*, int);

//
// sm_async.c prototypes
//
void		async_init(void);
void		async_main(uint32_t, uint8_t **);
int 		sm_send_xml_file(uint8_t);

//
// sm_fsm prototypes
// 
extern	Status_t state_event_mad(Mai_t *);
extern	Status_t state_event_timeout(void);

//
// sm_qos
//
void		sm_setup_SC2VL(VirtualFabrics_t *);

Status_t 	QosFillVlarbTable(Topology_t *, Node_t *, Port_t *, Qos_t *, struct _PortDataVLArb *);
Status_t 	QosFillVlarbTable(Topology_t *, Node_t *, Port_t *, Qos_t *, struct _PortDataVLArb *);
Status_t	QosFillBwVlarbTable(Qos_t*, uint8_t, PortDataVLArb *, VlVfMap_t*);

void sm_FillVlarbTableDefault(struct _PortDataVLArb * arb, uint8_t numVLs);
Status_t sm_select_vlvf_map(Topology_t *, Node_t *, Port_t *, VlVfMap_t *);
Status_t sm_select_vlbw_map(Topology_t *, Node_t *, Port_t *, VlBwMap_t *);
Status_t sm_fill_stl_vlarb_table(Topology_t *, Node_t *, Port_t *, struct _PortDataVLArb *);
Status_t sm_select_slsc_map(Topology_t *, Node_t *, Port_t *, Port_t *, STL_SLSCMAP *);
Status_t sm_select_scsl_map(Topology_t *, Node_t *, Port_t *, Port_t *, STL_SCSLMAP *);
Status_t sm_select_scvlr_map(Topology_t *, uint8_t, STL_SCVLMAP *);
Status_t sm_select_scvl_map(Topology_t *, Node_t *, Port_t *, Port_t *, STL_SCVLMAP *);
void sm_destroy_qos(void);

//
// sm_topology_rcv.c prototypes
//
void		topology_rcv(uint32_t, uint8_t **);

//
//	sm_utility.c prototypes.
//

#define WAIT_FOR_REPLY      0
#define RCV_REPLY_AYNC      1
#define WANT_REPLY_ON_QUEUE 2

Status_t	sm_setup_xft(Topology_t *, Node_t *, Node_t *, Port_t *, uint8_t *);
Status_t	sm_calculate_lft(Topology_t *topop, Node_t *switchp);
Status_t	sm_write_minimal_lft_blocks(Topology_t *topop, Node_t *switchp, int use_lr_dr_mix, uint8_t * path);
Status_t	sm_write_full_lfts_by_block_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance);
/// Recomputes and sends LFTs, PGTs, and PGFTs
Status_t	sm_setup_lft(Topology_t *, Node_t *);
Status_t	sm_send_lft(Topology_t *, Node_t *);
Status_t	sm_setup_lft_deltas(Topology_t *, Topology_t *, Node_t *);
Status_t	sm_send_partial_lft(Topology_t *, Node_t *, bitset_t *);
Status_t	sm_calculate_all_lfts(Topology_t *);
Status_t	sm_dr_init(Mai_t *, uint32_t, uint32_t, uint32_t, uint64_t, uint8_t *);
Status_t	sm_lr_init(Mai_t *, uint32_t, uint32_t, uint32_t, uint64_t, uint16_t, uint16_t);
Status_t	sm_select_path_lids(Topology_t *, Port_t *, uint16_t, Port_t *, uint16_t , uint16_t *, uint8_t *, uint16_t *, uint8_t *);
Status_t	sm_get_attribute(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *);
Status_t	sm_get_stl_attribute(IBhandle_t fd, uint32_t aid, uint32_t amod, uint8_t *path, uint8_t *buffer, uint32_t *bufferLength);
Status_t	sm_get_attribute_lrdr(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, int);
Status_t	sm_get_stl_attribute_lrdr(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint32_t *, int);
Status_t    sm_get_attribute_async_and_queue_rsp(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *);
Status_t	sm_set_attribute(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint64_t);
Status_t	sm_set_stl_attribute(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint32_t *, uint64_t);
Status_t 	sm_set_stl_attribute_mad_status(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint32_t *, uint64_t, uint32_t*);
Status_t	sm_set_attribute_lrdr(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint64_t, int);
Status_t	sm_set_stl_attribute_lrdr(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint32_t *, uint64_t, int);
Status_t    sm_set_attribute_async_dispatch(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint64_t, Node_t *, sm_dispatch_t *);
Status_t    sm_set_attribute_async_dispatch_lr(IBhandle_t, uint32_t, uint32_t, uint16_t, uint16_t, uint8_t *, uint64_t, Node_t *, sm_dispatch_t *);
Status_t    sm_set_stl_attribute_async_dispatch(IBhandle_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint32_t *, uint64_t, Node_t *, sm_dispatch_t *);
Status_t    sm_set_stl_attribute_async_dispatch_lr(IBhandle_t, uint32_t, uint32_t, uint16_t, uint16_t, uint8_t *, uint32_t *, uint64_t, Node_t *, sm_dispatch_t *);

Status_t sm_send_stl_request(IBhandle_t fd, uint32_t method, uint32_t aid, uint32_t amod, uint8_t *path, uint8_t *buffer, uint32_t *bufferLength, uint64_t mkey, uint32_t *madStatus);
Status_t    sm_send_request_impl(IBhandle_t, uint32_t, uint32_t, uint32_t, uint8_t *, uint8_t *, uint32_t, uint64_t, cntxt_callback_t, void *, int);
Status_t    sm_send_stl_request_impl(IBhandle_t, uint32_t, uint32_t, uint32_t, uint8_t *, uint32_t, uint8_t *, uint32_t *, uint32_t, uint64_t, cntxt_callback_t, void *, int, uint32_t *);
Status_t	sm_setup_node(Topology_t *, FabricData_t *, Node_t *, Port_t *, uint8_t *);
int 		sm_find_cached_node_port(Node_t *cnp, Port_t *cpp, Node_t **nodep, Port_t **portp);
int 		sm_find_cached_neighbor(Node_t *cnp, Port_t *cpp, Node_t **nodep, Port_t **portp);
int			sm_check_node_cache(Node_t *cnp, Port_t *cpp, Node_t **nodep, Port_t **portp);
Status_t	sm_check_Master(void);
Status_t	sm_initialize_port(Topology_t *, Node_t *, Port_t *, int, uint8_t *);
Status_t	sm_initialize_port_LR_DR(Topology_t *topop, Node_t *nodep, Port_t *portp);
Status_t	sm_initialize_switch_LR_DR(Topology_t *topop, Node_t *nodep, uint16_t parent_switch_lid, uint8_t parent_portno, int routing_needed);
Status_t	sm_prep_switch_layer_LR_DR(Topology_t *topop, Node_t *nodep, SwitchList_t *swlist_head, int rebalance);
Status_t	sm_setup_switches_lrdr_wave_discovery_order(Topology_t *topop, int rebalance, int routing_needed);
Status_t	sm_initialize_VLArbitration(Topology_t *, Node_t *, Port_t *);

//Methods for dynamic reconfiguration
void        sm_config_apply(FMXmlCompositeConfig_t *xml_config);
void        pm_config_apply(FMXmlCompositeConfig_t *xml_config);
void        fe_config_apply(FMXmlCompositeConfig_t *xml_config);
boolean     sm_config_valid(FMXmlCompositeConfig_t *xml_config, VirtualFabrics_t* newVfPtr, VirtualFabrics_t* oldVfPtr);
boolean     pm_config_valid(FMXmlCompositeConfig_t *xml_config);
boolean     fe_config_valid(FMXmlCompositeConfig_t *xml_config);
uint32_t    findVfIdxInActiveList(VF_t* vfToFind, VirtualFabrics_t *VirtualFabrics, uint8_t logWarning);

/**
	Update SLSC, SCSL, SCVLt/nt, and VLArb values on @c nodep with values from old topology and/or SMA at @c smaportp.

	For switches, get values for all ports via @c smaportp.  For HFIs, get values only for @c smaportp.

	@return VSTATUS_OK if all values were updated successfully, something else otherwise.
*/
Status_t
sm_node_updateFields(IBhandle_t fd, uint16_t slid, Node_t * nodep, Port_t * smaportp);

Status_t	sm_initialize_Node_SLMaps(Topology_t *, Node_t *, Port_t *);
Status_t	sm_initialize_Switch_SLMaps(Topology_t *, Node_t *);
Status_t	sm_initialize_Port_BfrCtrl(Topology_t * topop, Node_t * nodep, Port_t * out_portp, STL_BUFFER_CONTROL_TABLE *bct);

/**
	Updates cableInfo stored in @c portp->portData->cableInfo using SMA Get(CableInfo).

	Allocates memory for @c portp->portData->cableInfo if not already allocated; otherwise uses existing memory.

	@return VSTATUS_OK on success, VSTATUS_NOSUPPORT if @c portp does not support Get(CableInfo), or VSTATUS_BAD on runtime error.
*/
Status_t sm_update_cableinfo(Topology_t *, Node_t *, Port_t * portp);

Status_t 	sm_get_node_port_states(Topology_t *, Node_t *, Port_t *, uint8_t *, STL_PORT_STATE_INFO **);
Status_t 	sm_set_node_port_states(Topology_t *, Node_t *, Port_t *, uint8_t *, uint32_t, uint32_t, STL_PORT_STATE_INFO **);
Status_t	sm_disable_port(Topology_t *, Node_t *, Port_t *);
Status_t	sm_bounce_port(Topology_t *, Node_t *, Port_t *);
Status_t	sm_bounce_link(Topology_t *, Node_t *, Port_t *);
Status_t    sm_bounce_all_switch_ports(Topology_t *topop, Node_t *nodep, Port_t *portp, uint8_t *path);
Status_t	sm_get_CapabilityMask(IBhandle_t, uint8_t, uint32_t *);
Status_t	sm_set_CapabilityMask(IBhandle_t, uint8_t, uint32_t);
Status_t	sm_process_notice(Notice_t *);
Node_t		*sm_find_guid(Topology_t *, uint64_t);
Node_t      *sm_find_quarantined_guid(Topology_t *topop, uint64_t guid);
Node_t		*sm_find_next_guid(Topology_t *, uint64_t);
Node_t	 	*sm_find_node(Topology_t *, int32_t);
Node_t		*sm_find_node_by_path(Topology_t *, Node_t *, uint8_t *);
Port_t      *sm_find_node_port(Topology_t *, Node_t *, int32_t);
Node_t	 	*sm_find_switch(Topology_t *, uint16_t);
Node_t	 	*sm_find_port_node(Topology_t *, Port_t *);
Port_t	 	*sm_find_port(Topology_t *, int32_t, int32_t);
Port_t		*sm_find_port_guid(Topology_t *, uint64_t);
Port_t      *sm_find_port_peer(Topology_t *topop, uint64_t node_guid, int32_t port_no);
Port_t		*sm_find_active_port_guid(Topology_t *, uint64_t);
Port_t		*sm_find_port_lid(Topology_t *, STL_LID);
Port_t		*sm_find_active_port_lid(Topology_t *, STL_LID);
Port_t		*sm_find_node_and_port_lid(Topology_t *, STL_LID, Node_t **);
Port_t		*sm_find_neighbor_node_and_port(Topology_t *, Port_t *, Node_t **);
Status_t	sm_find_node_and_port_pair_lid(Topology_t *, STL_LID, uint32_t, Node_t **, Port_t **, Port_t **, Node_t **, Port_t **, Port_t **);
void  		sm_dump_node_map(Topology_t *topop);
int			sm_check_lid(Node_t *nodep, Port_t * portp, uint8_t newLmc);
int			sm_set_lid(Node_t *nodep, Port_t * portp, uint8_t lmc);
void		sm_setTrapThreshold(uint32_t, uint32_t);
void		sm_removedEntities_init(void);
void		sm_removedEntities_destroy(void);
Status_t	sm_removedEntities_reportPort(Node_t *, Port_t *, RemovedEntityReason_t);
Status_t	sm_removedEntities_clearNode(Node_t *);
Status_t	sm_removedEntities_clearPort(Node_t *, Port_t *);
void		sm_removedEntities_displayPorts(void);
void		sm_elevatePriority(void);
void		sm_restorePriority(void);
void		sm_restorePriorityOnly(void);
void		sm_compactSwitchSpace(Topology_t *, bitset_t *);
void		sm_clearSwitchPortChange(Topology_t *topop);
Status_t	sm_activate_switch(Topology_t *, Node_t *);
int			sm_select_port(Topology_t *, Node_t *, int, uint8_t *);
int			sm_select_ports(Topology_t *, Node_t *, int, SwitchportToNextGuid_t *);
void		sm_balance_ports(SwitchportToNextGuid_t *, int);
void		smLogHelper(uint32_t, const char *, char *, int, uint64_t, char *, uint32_t);
Status_t	sm_get_uninit_connected_switch_list(Topology_t *topop, Node_t *nodep, SwitchList_t **swlist_head);
void		sm_delete_switch_list(SwitchList_t *sw);
uint8_t		is_switch_on_list(SwitchList_t *swlist_head, Node_t *switchp);
Status_t    sm_set_linkinit_reason(Node_t *nodep, Port_t *portp, uint8_t initReason);
Status_t	sm_verifyPortSpeedAndWidth(Topology_t *topop, Node_t *nodep, Port_t *portp);
Status_t    sm_enable_port_led(Node_t *nodep, Port_t *portp, boolean enabled);
void		sm_mark_link_down(Topology_t *topop, Port_t *portp);

//
// sm_activate.c prototypes
//

void sm_arm_node(Topology_t *, Node_t *);
void sm_activate_all_hfi_first_safe(Topology_t *, pActivationRetry_t);
void sm_activate_all_hfi_first(Topology_t *, pActivationRetry_t);
void sm_activate_all_switch_first(Topology_t *, pActivationRetry_t);

//
// sm_routing.c prototypes
//
Status_t sm_routing_copy_cost_matrix(Topology_t *src_topop, Topology_t *dst_topop);
Status_t sm_routing_prep_new_switch(Topology_t *topop, Node_t *nodep, int, uint8_t *path);
Status_t sm_routing_route_switch_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance);
Status_t sm_routing_route_new_switch_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance);
Status_t sm_routing_route_old_switch(Topology_t *src_topop, Topology_t *dst_topop, Node_t *nodep);
int      sm_balance_base_lids(SwitchportToNextGuid_t *ordered_ports, int olen);
Status_t sm_routing_init(void);

//
// sm_routing_funcs.c prototypes
//
extern Status_t sm_routing_func_pre_process_discovery_noop(Topology_t *topop, void **outContext);
extern Status_t sm_routing_func_discover_node_noop(Topology_t *topop, Node_t *nodep, void *context);
extern Status_t sm_routing_func_discover_node_port_noop(Topology_t *topop, Node_t *nodep, Port_t *portp, void *context);
extern Status_t sm_routing_func_post_process_discovery_noop(Topology_t *topop, Status_t discoveryStatus, void *context);
extern Status_t sm_routing_func_post_process_routing_noop(Topology_t *topop, Topology_t *old_topop, int *rebalance);
extern Status_t sm_routing_func_post_process_routing_copy_noop(Topology_t *src_topop, Topology_t *dst_topop, int *rebalance);
extern Status_t sm_routing_func_alloc_cost_matrix_floyds(Topology_t *topop);
extern Status_t sm_routing_func_init_cost_matrix_floyds(Topology_t *topop);
extern Status_t sm_routing_func_calc_cost_matrix_floyds(Topology_t * topop, int switches, unsigned short * cost, SmPathPortmask_t * path);
extern Status_t sm_routing_func_copy_routing_noop(Topology_t *src_topop, Topology_t *dst_topop);
extern Status_t sm_routing_func_copy_routing_lfts(Topology_t *src_topop, Topology_t *dst_topop);
extern Status_t sm_routing_func_init_switch_routing_lfts(Topology_t * topop, int * routing_needed, int * rebalance);
extern Status_t sm_routing_func_setup_switches_lrdr_wave_discovery_order(Topology_t * topop, int rebalance, int routing_needed);
extern Status_t sm_routing_func_calculate_lft(Topology_t * topop, Node_t * switchp);
extern Status_t sm_routing_func_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *nodep, Port_t *orig_portp, uint8_t *portnos);
extern int sm_routing_func_select_ports(Topology_t *topop, Node_t *switchp, int endIndex, SwitchportToNextGuid_t *ordered_ports, boolean selectBest);
extern Status_t sm_routing_func_setup_pgs(struct _Topology *topop, struct _Node * srcSw, struct _Node * dstSw);
extern int sm_routing_func_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *nodep, uint8_t *portnos);
extern Status_t sm_routing_func_select_slsc_map(Topology_t *topop, Node_t *nodep, Port_t *in_portp, Port_t *out_portp, STL_SLSCMAP *outSlscMap);
extern Status_t sm_routing_func_select_scsl_map(Topology_t *topop, Node_t *nodep, Port_t *in_portp, Port_t *out_portp, STL_SCSLMAP *outScslMap);
extern Status_t sm_routing_func_select_scsc_map(Topology_t *topop, Node_t *switchp, int getSecondary, int *numBlocks, STL_SCSC_MULTISET** scscmap);
extern Status_t sm_routing_func_select_scvl_map_fixedmap(Topology_t *topop, Node_t *nodep, Port_t *in_portp, Port_t *out_portp, STL_SCVLMAP *outScvlMap);
extern Status_t sm_routing_func_select_vlvf_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlVfMap_t * vlvfmap);
extern Status_t sm_routing_func_select_vlbw_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlBwMap_t * vlvfmap);
extern Status_t sm_routing_func_select_scvlr_map(Topology_t *topop, uint8_t vlCap, STL_SCVLMAP *outScvlMap);
extern Status_t sm_routing_func_fill_stl_vlarb_table(Topology_t *topop, Node_t *nodep, Port_t *portp, PortDataVLArb* arbp);
extern Status_t sm_routing_func_select_path_lids(Topology_t *topop, Port_t *srcPortp, uint16_t slid, Port_t *dstPortp, uint16_t dlid, uint16_t *outSrcLids, uint8_t *outSrcLen, uint16_t *outDstLids, uint8_t *outDstLen);
extern Status_t sm_routing_func_process_swIdx_change_noop(Topology_t * topop, int old_idx, int new_idx, int last_idx);
extern int sm_routing_func_check_switch_path_change(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp);
extern boolean sm_routing_func_needs_routing_recalc_false(Topology_t * topop, Node_t * nodep);
extern boolean sm_routing_func_needs_routing_recalc_true(Topology_t * topop, Node_t * nodep);
extern boolean sm_routing_func_needs_lft_recalc(Topology_t * topop, Node_t * nodep);
extern boolean sm_routing_func_can_send_partial_routes_true(void);
extern boolean sm_routing_func_can_send_partial_routes_false(void);
extern boolean sm_routing_func_do_spine_check(Topology_t * topop, Node_t * switchp);
extern Status_t sm_routing_func_write_minimal_lft_blocks(Topology_t * topop, Node_t * switchp, int use_lr_dr_mix, uint8_t * path);
extern Status_t sm_routing_func_write_full_lfts_LR(Topology_t * topop, SwitchList_t * swlist, int rebalance);
extern Status_t sm_routing_func_route_old_switch(Topology_t *src_topop, Topology_t *dst_topop, Node_t *nodep);
extern boolean sm_routing_func_handle_fabric_change(Topology_t *topop, Node_t *oldSwitchp, Node_t *switchp);
extern Status_t sm_routing_func_update_bw(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern Status_t sm_routing_func_assign_scs_to_sls_fixedmap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern Status_t sm_routing_func_assign_sls(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern boolean sm_routing_func_mcast_isolation_is_required(void);
extern boolean sm_routing_func_mcast_isolation_not_required(void);
extern int sm_routing_func_min_vls(void);
extern int sm_routing_func_max_vls(void);
extern int sm_routing_func_one_routing_scs(int sl, boolean mc_sl);

//
// sm_shortestpath.c prototypes
//

Status_t sm_shortestpath_init(void);

//
// sm_fattree.c prototypes
//

Status_t sm_fattree_init(void);

// sm_qos.c prototypes
void sm_cong_exch_copy(void);

//
// sm_dgmh.c prototypes
//

Status_t sm_dgmh_init(void);

//
// sm_hypercube.c prototypes
//

Status_t sm_hypercube_init(void);

//
// sm_dor.c prototypes
//
Status_t sm_dor_init(void);

//
// sm_dispatch.c prototypes
//

void sm_dispatch_destroy(sm_dispatch_t *disp);
Status_t sm_dispatch_new_req(sm_dispatch_t *, sm_dispatch_send_params_t *, Node_t *, sm_dispatch_req_t **);
Status_t sm_dispatch_enqueue(sm_dispatch_req_t *req);
Status_t sm_dispatch_init(sm_dispatch_t *disp, uint32_t reqsSupported);
Status_t sm_dispatch_wait(sm_dispatch_t *disp);
Status_t sm_dispatch_clear(sm_dispatch_t *disp);
Status_t sm_dispatch_update(sm_dispatch_t *disp);
void sm_dispatch_bump_passcount(sm_dispatch_t *disp);

//
// sm_partMgr.c prototypes
//

int 		checkPKey(uint16_t);
int			smCheckPortPKey(uint16_t, Port_t*);

uint16_t	smGetPortPkey(uint16_t, Port_t*);

Status_t    setPKey(uint32_t, uint16_t, int);
Status_t	sm_set_portPkey(Topology_t *, Node_t *, Port_t *, Node_t *, Port_t *, uint8_t*, uint8_t*, uint16_t*, int);
Status_t    sm_set_local_port_pkey(STL_NODE_INFO *nodeInfop);

uint16_t	getDefaultPKey(void);
uint16_t	getPKey(uint8_t);		 // getPKey is needed for esm (sm mib).

uint16_t	smGetRequestPkeyIndex(uint8_t,  STL_LID);
uint16_t	smGetCommonPKey(Port_t*,  Port_t*);
int 		smValidatePortPKey(uint16_t, Port_t*);
int			smValidatePortPKey2Way(uint16_t, Port_t*, Port_t*);

void		smGetVfMaxMtu(Port_t*, Port_t*, STL_MCMEMBER_RECORD*, uint8_t*, uint8_t*);

// validate serviceId for VF.  Also covers the unmatchedSid option
Status_t	smVFValidateVfServiceId(int vf, uint64_t serviceId);
Status_t	smVFValidateVfMGid(int vf, uint64_t mGid[2]);
Status_t	smGetValidatedServiceIDVFs(Port_t*, Port_t*, uint16_t, uint8_t, uint8_t, uint64_t, bitset_t*);
Status_t	smGetValidatedVFs(Port_t*, Port_t*, uint16_t, uint8_t, uint8_t, bitset_t*);
Status_t	smVFValidateMcGrpCreateParams(Port_t * joiner, Port_t * requestor,
                                          STL_MCMEMBER_RECORD * mcMemberRec, bitset_t * vfMembers);
Status_t	smVFValidateMcDefaultGroup(int, uint64_t*);
void		smVerifyMcastPkey(uint64_t*, uint16_t);
void		smSetupNodeVFs(Node_t *nodep);
void        smSetupNodeDGs(Node_t *nodep);
boolean     smEvaluateNodeDG(Node_t* nodep, int dgIdxToEvaluate, PortRangeInfo_t* portInfo);
boolean     smEvaluatePortDG(Node_t* nodep, Port_t* portp, int dgIdxToEvaluate, bitset_t* dgMemberForPort, bitset_t* dgsEvaluated);
void		smLogVFs(void);
char*		smGetVfName(uint16_t);

//
// sm_ar.c prototypes
//
Status_t	sm_SetARPause(Node_t *, uint8_t);

/**
  Send PGT and PGFT to node if necessary.
*/
Status_t	sm_AdaptiveRoutingSwitchUpdate(Topology_t*, Node_t*);
uint8_t		sm_VerifyAdaptiveRoutingConfig(Node_t *p);

//
// sm_multicast.c prototypes
//

extern IB_GID nullGid;

//
// mLid usage structure
//
typedef struct {
	STL_LID lid;
	uint16_t usageCount;
} LidUsage_t;

typedef struct {
	cl_map_item_t mapItem; // Map item for group class usageMap
	uint16_t lidsUsed; // number of lids in the lid class
	uint16_t pKey; // pkey for this lid class
	uint16_t mtu; // mtu for this lid class
	uint16_t rate; // rate for this lid class
	// array of Lids & their usage counts in this lid class
	LidUsage_t * lids;
} LidClass_t;

typedef struct {
	// Maximum number of mLids that this group class
	// can use before the SM starts reusing mLids for
	// multiple MC groups. A value of zero effectively
	// disables mLid agragation.
	uint16_t maximumLids;

	// Number of mLids currently in use by the group class
	uint16_t currentLids;

	// Mask and value for MGid's belonging to this group class
	IB_GID mask;
	IB_GID value;

	// QMap of LidClass_t structure, indexed by {pkey, mtu, rate}
	cl_qmap_t usageMap;
} McGroupClass_t;

/**
	Allocate a @c CableInfo_t on the heap and initialize the reference count to 1.
*/
CableInfo_t * sm_CableInfo_init(void);

/**
	@return A pointer to the same data as @c ci (may even be @c ci) or NULL if the data could not be copied (reference count exhausted).
*/
CableInfo_t * sm_CableInfo_copy(CableInfo_t * ci);

/**
	Decrement <tt>ci->refcount</tt> by one and free @c ci and its members
	if <tt>ci->refcount == 0</tt>.

	@param ci Created with @c sm_CableInfo_init()

	@return @c ci or NULL if @c ci was freed because reference count hit 0.
*/
CableInfo_t *  sm_CableInfo_free(CableInfo_t * ci);

/**
	Does @c port support Get(CableInfo)?

	@c port must not be port 0, be a QSFP connector, and @c port->portData must be allocated.

	Does not check whether port state is high enough for an SMA Get() to succeed.
*/
boolean sm_Port_t_IsCableInfoSupported(Port_t * port);

/**
  Mark @c port as supporting or not supporting Get(CableInfo).

	Implementation uses a special address that does not point to a valid CableInfo_t (or possibly valid memory).  Use @c sm_Port_t_IsCableInfoSupported() to check if the port may have valid cable data before assuming a non-NULL port->portData->cableInfo pointer means that the data is valid.
 
	Frees non-null CableInfo associated with @c port when going from supported -> !supported.
*/
void sm_Port_t_SetCableInfoSupported(Port_t * port, boolean supported);

// If new_pg is not found in pgp, adds it to pgp, updates length, and set
// index to the index of the new port group and returns 1. 
// If it *is* already found in pgp, index points to the existing group and
// returns 0. If new_pg cannot be added (because length == cap), returns -1.
int sm_Push_Port_Group(STL_PORTMASK pgp[], STL_PORTMASK new_pg,
	uint8_t *index, uint8_t *length, uint8_t cap);

// Takes a set of ports and builds an STL Port Mask from them.
STL_PORTMASK sm_Build_Port_Group(SwitchportToNextGuid_t *ordered_ports, 
	int olen);

// Methods used for device group membership evaluation
int         smGetDgIdx(char* dgName);
boolean     isDgMember(int dgIdx, PortData_t* portDataPtr);
boolean     convertWildcardedString(char* inStr, char* outStr, RegexBracketParseInfo_t* regexInfo);
boolean     isCharValid(char inChar);
boolean     processBracketSyntax(char* inputStr, int inputLen, int* curIdxPtr, RegexBracketParseInfo_t* regexInfo, boolean isPortRange);
boolean     isNumberInRange(char* number, RegexBracketParseInfo_t* regexInfo, int bracketIdx);
void        initializeRegexStruct(RegExp_t* regExprPtr);
void        printRegexStruct(RegexBracketParseInfo_t* regexInfo);


Status_t	sm_ideal_spanning_tree(McSpanningTree_t *mcST, int filter_mtu_rate, int32_t mtu, int32_t rate, int num_nodes, int *complete);
Status_t	sm_spanning_tree(int32_t, int32_t, int *);
void		sm_build_spanning_trees(void);
void		sm_spanning_tree_resetGlobals(void);
Status_t	sm_multicast_add_group_class(IB_GID mask, IB_GID value, uint16_t maxLids);
Status_t	sm_multicast_set_default_group_class(uint16_t maxLids);
Status_t	sm_multicast_sync_lid(IB_GID mGid, PKey_t pKey, uint8_t mtu, uint8_t rate, STL_LID lid);
Status_t    sm_multicast_check_sync_consistancy(McGroupClass_t * groupClass, PKey_t pKey,
                                                uint8_t mtu, uint8_t rate, STL_LID lid);
STL_LID		sm_multicast_get_max_lid(void);
Status_t	sm_multicast_gid_check(IB_GID);
Status_t	sm_multicast_gid_assign(uint32_t, IB_GID);
Status_t	sm_multicast_gid_valid(uint8_t, IB_GID);
Status_t	sm_calculate_mfts(void);
Status_t	sm_set_all_mft(int, Topology_t *);
void		sm_multicast_switch_mft_copy(void);
McGroup_t	*sm_find_multicast_gid(IB_GID);
Status_t	sm_multicast_assign_lid(IB_GID mGid, PKey_t pKey, uint8_t mtu, uint8_t rate, STL_LID * lid);
Status_t	sm_multicast_decommision_group(McGroup_t * group);
McGroup_t	*sm_find_next_multicast_gid(IB_GID);
McMember_t	*sm_find_multicast_member(McGroup_t *, IB_GID);
McMember_t	*sm_find_multicast_member_by_index(McGroup_t *mcGroup, uint32_t index);
McMember_t	*sm_find_next_multicast_member_by_index(McGroup_t *mcGroup, uint32_t index);

void		topology_main(uint32_t, uint8_t **);
Status_t	topology_userexit(void);
Status_t	forwarding_userexit(uint16_t *, int16_t *);
Status_t	authorization_userexit(void);
Status_t	license_userexit(void);
Status_t    loopTest_userexit_findPaths(Topology_t *);
void        loopTest_userexit_updateLft(Topology_t *, Node_t *);
void        printDgVfMemberships(void);

//
//      Startup and shutdown control values.
//
#define SM_CONTROL_SHUTDOWN             0x01
#define SM_CONTROL_STANDBY              0x02
#define SM_CONTROL_RESTART              0x03
#define SM_CONTROL_HEARTBEAT            0x04
#define SM_CONTROL_REGISTER             0x05
#define SM_CONTROL_RECONFIG             0x06

Status_t        sm_control(Mai_t *);
Status_t        sm_control_shutdown(Mai_t *);
Status_t        sm_control_standby(Mai_t *);
Status_t        sm_control_restart(Mai_t *);
Status_t        sm_control_heartbeat(Mai_t *);
Status_t        sm_control_register(Mai_t *);
Status_t        sm_control_reconfig(void);
Status_t        sm_control_init(void);
Status_t        sm_control_notify(void);

#ifdef __VXWORKS__
void		idbSetSmState(ulong_t newState);
uint32_t    idbGetSmSweepRate();
uint32_t	idbGetSmPriority();
uint32_t	idbSetSmPriority(uint32_t newPriority);
uint32_t	idbSetSmElevatedPriority(uint32_t value);
uint32_t	idbGetSmElevatedPriority();
uint64_t    idbGetSmKey();
uint32_t    idbSetSmKey(uint64_t newSmkey);
uint64_t    idbGetSmMKey();
uint32_t    idbSetSmMKey(uint64_t newSmkey);
uint8_t		idbGetSmSwitchLifetime();
uint32_t	idbSetSmSwitchLifetime(uint8_t newSwitchLifetime);
uint8_t		idbGetSmHoqLife();
uint32_t	idbSetSmHoqLife(uint8_t newHoqLife);
uint8_t		idbGetSmVLStall();
uint32_t	idbSetSmVLStall(uint8_t newVLStall);
uint32_t    idbGetSmSubnetSize();
uint32_t    idbSetSmSubnetSize(uint32_t newSubnetSize);
uint32_t    idbGetSmLMC();
uint32_t    idbSetSmLMC(uint32_t newLMC);
uint32_t	idbSetSmLidOffset(uint32_t offset);
uint32_t	idbGetSmLidOffset();
uint32_t	idbSetSmTopoErrorThreshold(uint32_t value);
uint32_t	idbGetSmTopoErrorThreshold();
uint32_t	idbSetSmTopoAbandonThreshold(uint32_t value);
uint32_t	idbGetSmTopoAbandonThreshold();
uint32_t	idbSetSmMaxRetries(uint32_t value);
uint32_t	idbGetSmMaxRetries();
uint32_t	idbSetSmRcvWaitTime(uint32_t value);
uint32_t	idbGetSmRcvWaitTime();
uint32_t	idbSetSmNonrespDropTime(uint32_t value);
uint32_t	idbGetSmNonrespDropTime();
uint32_t	idbSetSmNonrespDropSweeps(uint32_t value);
uint32_t	idbGetSmNonrespDropSweeps();
uint32_t	idbSetSmLogLevel(uint32_t value);
uint32_t	idbGetSmLogLevel();
uint32_t	idbSetSmLogFilter(uint32_t value);
uint32_t	idbGetSmLogFilter();
uint32_t	idbSetSmLogMask(uint32_t value);
uint32_t	idbGetSmLogMask();
uint32_t	idbSetSmMcLidTableCap(uint32_t value);
uint32_t	idbGetSmMcLidTableCap();
uint32_t	idbSetSmDefMcGrpPKey(uint32_t value);
uint32_t	idbGetSmDefMcGrpPKey();
uint32_t	idbSetSmDefMcGrpQKey(uint32_t value);
uint32_t	idbGetSmDefMcGrpQKey();
uint32_t	idbSetSmDefMcGrpMtu(uint32_t value);
uint32_t	idbGetSmDefMcGrpMtu();
uint32_t	idbSetSmDefMcGrpRate(uint32_t value);
uint32_t	idbGetSmDefMcGrpRate();
uint32_t	idbSetSmDefMcGrpSl(uint32_t value);
uint32_t	idbGetSmDefMcGrpSl();
uint32_t	idbSetSmDefMcGrpFlowLabel(uint32_t value);
uint32_t	idbGetSmDefMcGrpFlowLabel();
uint32_t	idbSetSmDefMcGrpTClass(uint32_t value);
uint32_t	idbGetSmDefMcGrpTClass();
uint32_t	idbSetSmLoopTestPackets(uint32_t value);
uint32_t	idbGetSmLoopTestPackets();
uint32_t	idbSetSmMasterPingInterval(uint32_t value);
uint32_t	idbGetSmMasterPingInterval();
uint32_t	idbSetSmMaxMasterPingFailures(uint32_t value);
uint32_t	idbGetSmMaxMasterPingFailures();
uint32_t	idbSetSmDbSyncInterval(uint32_t value);
uint32_t	idbGetSmDbSyncInterval();
uint32_t	idbSetSmStartAtBootSelector(uint32_t value);
uint32_t	idbGetSmStartAtBootSelector();
uint32_t	idbGetSmDbSyncInterval();
uint32_t	idbSetSmDbSyncInterval(uint32_t);
uint64_t	idbGetGidPrefix();
uint32_t	idbSetGidPrefix(uint64_t prefix);
uint32_t	idbGetRmppAlt();
uint32_t	idbGetLegacyHwSupport();
uint32_t	idbGetFullPkeySupport();
uint32_t	idbGetVlArbSupport();
uint32_t	idbGetEnhPortZeroSupport();
uint32_t	idbGetMcGrpCreate();
uint32_t	idbGetDynamicPltSupport();
uint32_t    idbGetSmPltValue(uint32_t index);
uint32_t	idbGetSmPerfDebug();
uint32_t	idbGetSaPerfDebug();
uint32_t	idbGetRmppDebug();
uint32_t	idbGetRmppChksumDebug();
uint32_t	idbGetTopologyDebug();
uint32_t	idbGetLooptest();
uint32_t	idbSetSmTrapThreshold(uint32_t value);
uint32_t	idbGetSmTrapThreshold();
uint32_t	idbSetSmAppearanceMsgThresh(uint32_t value);
uint32_t	idbGetSmAppearanceMsgThresh();
uint32_t	idbGetDisableCommonMcastMtuAndRate(void);
#else
Status_t	sm_dump_state(const char * dumpDir);
#endif


/**
	Common switch LFT allocation code; frees LFT if already allocated.
	Uses switchp->..LinearFDBTop for size.  Rounds up to the next MAX_LFT_ELEMENTS_BLOCK entries.  Sets all entries to 0xFF.

	@param outSize [optional, output only] Amount of memory that was allocated.
	@return VSTATUS_OK on successful allocation, or the status that vs_pool_alloc() returned on error.
*/
Status_t sm_Node_init_lft(Node_t * switchp, size_t * outSize);

void sm_node_release_changes(Node_t * nodep);

Status_t sm_port_init_changes(Port_t * portp);
void sm_port_release_changes(Port_t * portp);

///@return number of remaining references to the PGFT
uint32 sm_Node_release_pgft(Node_t * node);

/**
	Get the PGFT for @c node; should be preferred to direct access since it allows for deferred allocation meaning the memory isn't allocated until it's needed.

	Memsets memory to 0 first time it needs to be allocated.  Does not allocate memory if capability mask for @c node indicates adaptive routing is not supported on it.
*/
const PORT * sm_Node_get_pgft(const Node_t * node);

/**
	Get PGFT for writing, create if it does not exist already.
*/
PORT * sm_Node_get_pgft_wr(Node_t * node);
/**
	Copy PGFT from @c src to @c dest.  Allocate if @c src does not already have a PGFT, free existing memory otherwise.

	@return PGFT of @c dest
*/
PORT * sm_Node_copy_pgft(Node_t * dest, const Node_t * src);

/// @return amount of memory allocated in bytes or 0 if not allocated
size_t sm_Node_get_pgft_size(const Node_t * node);

/// @return Amount of memory that should be allocated for PGFT on @c node
size_t sm_Node_compute_pgft_size(const Node_t * node);

/**
	Invalidate (reset to PODs) the PGT and PGFT mappings on @c node.  Modifies the
	state of PGT and PGFT on @c node but does not generate or send the MADs necessary
	to change these on the SMA.
*/
void sm_Node_invalidate_pgs(Node_t * node);

Status_t sm_Node_prune_portgroups(Node_t * newSw);

void sm_set_log_level(uint32_t log_level);
void sm_set_log_mode(uint32_t log_mode);
void sm_set_log_mask(const char* mod, uint32_t mask);
void sm_init_log_setting(void);

uint32_t    sm_getSweepRate(void);
void		sm_setSweepRate(uint32_t rate);
void 		sm_forceSweep(const char* reason);
uint32_t    sm_util_isTopologyValid(void);
uint32_t	sm_util_get_state(void);
uint32_t    sm_util_get_passcount(void);
uint32_t	sm_get_smPerfDebug(void);
uint32_t	sa_get_saPerfDebug(void);
void 		smPerfDebugToggle(void);
void 		saPerfDebugToggle(void);
void 		saRmppDebugToggle(void);
void		smForceRebalanceToggle(void);
uint32_t	sm_get_smAdaptiveRouting(void);
void		smAdaptiveRoutingToggle(uint32_t);
void		smSetAdaptiveRouting(uint32_t);
void        smPauseResumeSweeps(boolean);
void		smProcessReconfigureRequest(void);
PortData_t *sm_alloc_port(Topology_t *topop, Node_t *nodep, uint32_t portIndex, int *bytes);
void        sm_free_port(Topology_t *topop, Port_t * portp);
void        sm_node_free_port(Topology_t *topop, Node_t *nodep);
Status_t    sm_build_node_array(Topology_t *topop);
Status_t    sm_clearIsSM(void);
void        sm_clean_vfdg_memory(void);
extern uint8_t sm_isActive(void);
extern uint8_t sm_isDeactivated(void);
extern uint8_t sm_isMaster(void);
#ifdef __LINUX__
extern void sm_wait_ready(uint32_t modid);
extern uint8_t sm_isValidCoreDumpConfigSettings(uint32_t modid, const char *limit, const char *dir);
extern uint8_t sm_isValidLogConfigSettings(uint32_t modid, uint32_t logLevel, uint32_t logMode, uint32_t log_masks[VIEO_LAST_MOD_ID+1], char *logFile, char *syslogFacility);
extern void sm_getLogConfigSettings(uint32_t *logLevel, uint32_t *logMode, uint32_t log_masks[VIEO_LAST_MOD_ID+1], char *logFile, char *syslogFacility);
#endif
extern uint8_t sm_isValidDeviceConfigSettings(uint32_t modid, uint32_t device, uint32_t port, uint64_t portGuid);
extern void sm_getDeviceConfigSettings(uint32_t *device, uint32_t *port, uint64_t *portGuid);
extern uint8_t sm_isValidMasterConfigSettings(uint32_t modid, int priority, int elevated_priority);
extern void sm_getMasterConfigSettings(uint32_t *priority, uint32_t *elevated_priority);
extern uint8_t sm_isValidSubnetSizeConfigSetting(uint32_t modid, uint32_t subnet_size);
extern void sm_getSubnetSizeConfigSetting(uint32_t *subnet_size);
extern int smValidateGsiMadPKey(Mai_t *maip, uint8_t mgmntAllowedRequired, uint8_t antiSpoof);

extern Status_t sm_get_buffer_control_tables(IBhandle_t fd_topology, Node_t *nodep, uint8_t start_port, uint8_t end_port);
/** =========================================================================
 * NOTE: bcts array must store all tables for start - end port.
 *
 * Side affect: on success port objects within the node object will be updated
 * with the tables sent to the SMA.
 */
extern Status_t sm_set_buffer_control_tables(IBhandle_t fd_topology, Node_t *nodep, uint8_t start_port, uint8_t end_port, STL_BUFFER_CONTROL_TABLE bcts[]);

extern void sm_set_force_attribute_rewrite(uint32_t force_attr_rewrite);
extern void sm_set_skip_attribute_write(char * datap);

/**
	Compare XmitQ values for VLs [0, @c actVlCount) and VL15.
*/
boolean sm_eq_XmitQ(const struct XmitQ_s * a, const struct XmitQ_s * b, uint8 actVlCount);

int sm_evalPreemptLargePkt(int cfgLarge, Node_t * nodep);
int sm_evalPreemptSmallPkt(int cfgSmall, Node_t * nodep);
int sm_evalPreemptLimit(int cfgLimit, Node_t * nodep);

/**
	Alloc portp->portData->newArb on demand and return pointer or null if !sm_port_valid(portp)
*/
struct _PortDataVLArb * sm_port_getNewArb(Port_t * portp);

/**
	Release portp->portData->newArb if allocated.
*/
void sm_port_releaseNewArb(Port_t * portp);

#endif	// _SM_L_H_

