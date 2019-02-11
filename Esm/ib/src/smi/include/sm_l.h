/* BEGIN_ICS_COPYRIGHT2 ****************************************

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
#include <stdint.h>
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
#include "mai_g.h"
#include "sm_maihandle.h"
#include "ifs_g.h"

#include "ilist.h"
#include "iquickmap.h"
#include <iba/stl_sm_priv.h>
#include <iba/stl_helper.h>

#ifdef __VXWORKS__
#include "regexp.h"
#else
#include "regex.h"
#endif

#include <fm_xml.h>
#include "topology.h"

#include "sm_parallelsweep.h"

#define	SM_PKEYS    64

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


static __inline__ uint32_t getCongTableSize(uint8_t numBlocks) {
	return (CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ + sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK) * numBlocks);
}

static __inline__ uint32_t getCongrefTableSize(uint8_t numBlocks) {
        return (sizeof(uint32) + CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ + sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK) * numBlocks);
}


#define MAX_VFABRICS		MAX_ENABLED_VFABRICS
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

extern Sema_t topo_terminated_sema;
extern int activateInProgress;
extern int isSweeping;
extern int forceRebalanceNextSweep;
extern int topology_changed;  /* indicates a change in the topology during discovery */
extern int topology_changed_count;  /* how many times has topology changed */
extern int topology_switch_port_changes;  /* indicates there are switches with switch port change flag set */
extern int topology_cost_path_changes;
extern int routing_recalculated; /* indicates all routing tables have been recalculated this sweep */
extern uint32_t topology_port_bounce_log_num; /* Number of times a log message has been generated about a port bounce this sweep*/
extern bitset_t old_switchesInUse;
extern bitset_t new_switchesInUse;
extern bitset_t new_endnodesInUse;
extern SMXmlConfig_t sm_config;

extern FabricData_t preDefTopology;

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
	union {
		VlarbTableData	vlarb;
	} u;
} PortDataVLArb;

typedef struct _PortDataSCSCMapPortMask {
	LIST_ITEM	SCSCMapItem;
	STL_PORTMASK	outports[STL_MAX_PORTMASK];
	STL_SCSCMAP	*SCSCMap;
} PortDataSCSCMapPortMask;

//
// Persistent Topology API
//

typedef enum {
	POPO_QUARANTINE_NONE  = 0,
	POPO_QUARANTINE_SHORT = 1,
	POPO_QUARANTINE_LONG = 2,
} PopoQuarantineType_t;

typedef enum {
	POPO_LONGTERM_NONE = 0,
	POPO_LONGTERM_FLAPPING = 1,
	POPO_LONGTERM_PERSISTENT_TIMEOUT = 2, //Not Implemented. Future Work
} PopoLongTermQuarantineReason_t;

typedef struct {
	PopoLongTermQuarantineReason_t reason;
	uint64_t monitoredTime;	//timestamp port was placed in monitor state
	uint64_t quarantinedTime;//Timestamp port was placed in quarantine state
	void *data; //Data specific to quarantine reason about state over sliding window
} PopoPortLongTermData_t;



typedef struct {
	struct _PopoNode * ponodep;
	uint8_t portNum;
	STL_PORT_STATES portStates;
	struct {
		PopoQuarantineType_t type;
		boolean monitored;
		PopoPortLongTermData_t longTermData; //Tracks data for monitored/long term quarantined ports 
		LIST_ITEM quarantineEntry; //Entry for either shorterm or longterm quarantine list. Cannot be in both.
		LIST_ITEM monitorEntry;    //Entry for monitored list.
	} quarantine;
	struct {
		uint8_t	reregisterPending:1;	// True if we need to send a PortInfo with ClientReregister
	} registration;
} PopoPort_t;

typedef struct _PopoNode {
	cl_map_item_t nodesEntry;
	struct {
		uint32_t pass;
		uint8_t nodeType;
		uint8_t numPorts;
		uint64_t guid;
	} info;
	struct {
		uint8_t errorCount;
	} quarantine;
	PopoPort_t ports[0];
} PopoNode_t;

typedef struct {
	Lock_t lock;
	cl_qmap_t nodes;
	struct {
		// the cumulative time spent in timeouts during this sweep
		ATOMIC_UINT cumulativeTimeout;
		// indicates a trap is pending; tracks whether a trap was received
		// at any point in a series of failed sweeps, and indicates that a
		// resweep should be scheduled
		boolean trapPending;
	} errors;
	struct {
		QUICK_LIST monitored; //Ports that are being considered for inclusion in longTerm quarantine. 
		QUICK_LIST shortTerm; //Ports that will be quarantined only until the next successful SM sweep
		QUICK_LIST longTerm;  //Ports that will be quarantined for multiple sweeps until underlying condition resolves
	} quarantine;
} Popo_t;

typedef struct {
	uint32 refCount;
	STL_HFI_CONGESTION_CONTROL_TABLE hfiCongCon;
} HfiCongestionControlTableRefCount_t;

//
//	Per Port structure.
//
typedef	struct _PortData {
	uint64_t	guid;		// port GUID
	uint64_t	gidPrefix;	// Gid Prefix
	uint8_t		gid[16];	// Gid (network byte order)
	uint32_t	capmask;	// capability mask
	uint16_t	capmask3;	// capability mask3
// lid, lmc and vl could be much smaller fields, lid=16bits, lmc=3, vls=3 bits
	STL_LID		lid;		// base lid
	uint8_t		lmc;		// LMC for this port
	uint8_t		vl0;		// VLs supported
	uint8_t		vl1;		// VLs actual
	uint8_t		mtuSupported:4;		// MTUs supported
    uint8_t		maxVlMtu:4;   // Largest mtu of VLs/VFs on this port.
    uint8_t     rate;       // static rate of link (speed*width)
	uint16_t	portSpeed;	// calculated portSpeed used in cost (stored due to high usage)
	uint8_t		guidCap;	// # of GUIDs
	uint8_t		numFailedActivate;	// how many times has sm_activate_port
   							// failed to activate this port
	uint32_t	flags;		// local flags
	STL_LID  	pLid;		// Persistent lid
	STL_PKEY_ELEMENT pPKey[SM_PKEYS];// Persistent PKeys
	bitset_t	pkey_idxs;	// Persistent number of PKeys
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
	QUICK_LIST  scscMapList[2];	// 0 is regular, 1 is Ext

	STL_BUFFER_CONTROL_TABLE bufCtrlTable;
	HfiCongestionControlTableRefCount_t *congConRefCount; // HFI Port or EH SWP 0 only.
	STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT swPortCongSet; // Switch port only.

	bitset_t	vfMember;
	bitset_t	qosMember;
	bitset_t	dgMember;		// Bitset indicating the index values of all device groups.
	bitset_t	fullPKeyMember;
	STL_LID		lidsRouted; 	// Number of lids routed through this port
	STL_LID		baseLidsRouted; // Number of base lids routed through this port

	/* Flags*/
	uint8_t		qosHfiFilter:1;
	uint8_t		isIsl:1;		// Switch port is linked to another switch
	uint8_t		uplink:1;		// Uplink port in fat tree
	uint8_t		downlink:1;		// Downlink port in fat tree
	uint8_t		delayedPkeyWrite:1; // True if Pkeys need to be wrote after topo release

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
		uint8_t pkeys:1;
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
	PopoPort_t	*poportp;	// pointer to persistent port structure
    PortData_t	*portData;  // Pointer to additional port related fields. 
							// may be NULL when dynamic port alloc enabled
							// TBD - is dynamic port alloc needed?  Should it
							// be removed and this changed to a structure
							// instead of a pointer?
} Port_t;

//
//	Per Node structure.
//

static __inline__ STL_LID STL_GET_UNICAST_LID_MAX(void) {
	return (sm_config.max_supported_lid);
}
#define MAX_SIZE_MFT ((STL_LID_MULTICAST_END-STL_LID_MULTICAST_BEGIN)+1)
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
	bitset_t	dgMembership; 
	uint16_t	nonRespCount;	// Number of times the node has not responded to gets/sets
	uint64_t	nonRespTime;	// Timestamp of first failure to respond.
	uint8_t     asyncReqsSupported; // number of async requests node can handle
	uint8_t     asyncReqsOutstanding; // number of async requests on the wire
	uint8_t		portsInInit; 
	uint8_t		activeVLs; 
	uint8_t		arbCap;			// capability of switch internal
	uint8_t		vlCap; 			// capability of switch internal
	uint8_t		numISLs; 
	uint8_t		tier; 			// tier switch resides on in fat tree
	uint16_t	tierIndex;		// switch index within it's tier
	uint8_t		uplinkTrunkCnt; // number of uplink ports in trunk group
	STL_LID		numLidsRouted;	// used to build balanced LFTs.
	STL_LID		numBaseLidsRouted;	// used to build balanced LFTs when LMC enabled.
	char*		nodeDescString;	// Only used when nodeDesc is 64 chars (not null terminated).
	cl_map_obj_t	mapObj;		// Quickmap item to sort on guids
	cl_map_obj_t	nodeIdMapObj;	// Quickmap item to sort on guids
	cl_map_obj_t	mapQuarantinedObj;		// Quickmap item to sort on guids
	cl_map_obj_t	switchLidMapObj;	// Quickmap item to sort switch lids
	PopoNode_t	*ponodep;		// pointer to persistent node structure
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
	uint8_t		arChange:1;			// indicates adaptive routing tables of switch need to be programmed, leave pause if set
	uint8_t		arSupport:1;		// indicates switch supports adaptive routing
	uint8_t		congConfigDone:1;	// indicates if this node's congestion control configuration is done.
	uint8_t		oldExists:1;  		// this node is also in the old topology
	uint8_t		uniformVL:1;  
	uint8_t		deltasRequired:1;	// indicates a node was added to the switch
	uint8_t		vlArb:1;  
	uint8_t		internalLinks:1;  
	uint8_t		externalLinks:1;  
	uint8_t		edgeSwitch:1;  		// switch has HFIs attached
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
	char			deviceGroupName[MAX_DGROUTING_ORDER][MAX_VFABRIC_NAME];
	uint16_t		deviceGroupIndex[MAX_DGROUTING_ORDER];
	cl_qmap_t		deviceGroup[MAX_DGROUTING_ORDER+1];
} DGTopology;

typedef struct {
	uint8_t * path;
	STL_LID slid;
	STL_LID dlid;
} SmpAddr_t;

// uint8_t *, STL_LID, STL_LID
#define SMP_ADDR_CREATE(PATH, SLID, DLID) { (PATH), (SLID), (DLID) }

// uint8_t *
#define SMP_ADDR_CREATE_DR(PATH) { (PATH), STL_LID_PERMISSIVE, STL_LID_PERMISSIVE }
// uint8_t *, STL_LID, STL_LID
#define SMP_ADDR_CREATE_LRDR(PATH, SLID, DLID) { (PATH), (SLID), (DLID) }
// STL_LID, STL_LID
#define SMP_ADDR_CREATE_LR(SLID, DLID) { NULL, (SLID), (DLID) }

// SmpAddr_t *, uint8_t *
#define SMP_ADDR_SET_DR(ADDR, PATH) \
	do { \
		(ADDR)->path = (PATH); \
		(ADDR)->slid = STL_LID_PERMISSIVE; \
		(ADDR)->dlid = STL_LID_PERMISSIVE; \
	} while(0)
// SmpAddr_t *, uint8_t *, STL_LID, STL_LID
#define SMP_ADDR_SET_LRDR(ADDR, PATH, SLID, DLID) \
	do { \
		(ADDR)->path = (PATH); \
		(ADDR)->slid = (SLID); \
		(ADDR)->dlid = (DLID); \
	} while(0)
// SmpAddr_t *, STL_LID, STL_LID
#define SMP_ADDR_SET_LR(ADDR, SLID, DLID) \
	do { \
		(ADDR)->path = NULL; \
		(ADDR)->slid = (SLID); \
		(ADDR)->dlid = (DLID); \
	} while(0)

// SmpAddr_t *
#define SMP_ADDR_IS_LRDR(ADDR) \
	((ADDR)->path && (ADDR)->slid != STL_LID_PERMISSIVE && (ADDR)->dlid != STL_LID_PERMISSIVE)

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
struct _VlQosMap;
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

	int (*routing_mode) (void);


	boolean (*extended_scsc_in_use) (void);

	/**
		Indicates DR routing initialization is required instead of LRDR.
	*/
	boolean (*requires_dr) (struct _Topology *, Node_t *);


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
	Status_t (*select_scsc_map)(struct _Topology *, struct _Node *, int, int *, STL_SCSC_MULTISET** scscmap);
	Status_t (*select_scvl_map)(struct _Topology *, struct _Node *, struct _Port *, struct _Port *, STL_SCVLMAP *);
	Status_t (*select_vlvf_map)(struct _Topology *, struct _Node *, struct _Port *, struct _VlVfMap *);
	Status_t (*select_vlqos_map)(struct _Topology *, struct _Node *, struct _Port *, struct _VlQosMap *);
	Status_t (*select_vlbw_map)(struct _Topology *, struct _Node *, struct _Port *, struct _VlBwMap *);
	Status_t (*select_scvlr_map)(struct _Topology *, uint8_t, STL_SCVLMAP *);

	Status_t (*fill_stl_vlarb_table)(struct _Topology *, struct _Node *, struct _Port *, struct _PortDataVLArb * arb);
	Status_t (*select_path_lids)(struct _Topology *, struct _Port *, STL_LID, struct _Port *, STL_LID, STL_LID[], uint8_t *, STL_LID[], uint8_t *);
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
		Differ by write of LFT
	*/
	Status_t (*write_minimal_routes)(struct _Topology *, struct _Node *, SmpAddr_t *);
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

	/**
		Delete any routing data associated with a node in the topology.
	*/
	void (*delete_node)(struct _Node *);

	/**
		Number of VLs that can be shared between the same SL type of different
		QoS groups.
	*/
	int (*oversubscribe_factor)(int sl, boolean mc_sl);

	/**
		Multiple routing SCs are used, but okay to overlay mcast with ucast.
	*/
	boolean (*overlay_mcast)(void);

	/**
		Process XML config to see that it is valid.
	*/
	Status_t (*process_xml_config)(void);

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

	uint16_t         *cost; // array for path resolution (cost matrix)
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
	uint8_t		minTier; // minimum tier one or more switches are assigned to
	uint8_t		maxTier; // Maximum tier one or more switches are assigned to

	size_t		bytesCost;		// cost/path array length
	size_t		bytesPath;
	uint8_t		sm_path[72];	// DR path to SM (STANDBY mode only)
	uint32_t	sm_count;	// actCount of master SM (STANDBY mode only)
    uint64_t    sm_key;    // the smKey of the master SM (STANDBY mode only)
    uint16_t    numSm;      // number of SMs in fabric
    //uint16_t    numMasterSms; // number of master SMs in fabric
    //TopologySm_t smRecs[SM_MAX_SUPPORTED]; // list of SMs in topology
	uint32_t    numRemovedPorts; // number of ports removed during discovery
    // loop test variables
	uint16_t	numLoopPaths; // number of loop paths in fabric
	LoopPath_t	*loopPaths; // loop paths in fabric
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

	VirtualFabrics_t *vfs_ptr;

	// Per sweep pre-defined topology log counts
	PreDefTopoLogCounts preDefLogCounts;

	/// Store link down reasons for ports that disappeared;
	/// see LdrCacheEntry_t
	cl_qmap_t *ldrCache;


	/// Store switch lids for efficient cost queries
	cl_qmap_t	* switchLids;
#ifdef __VXWORKS__
	uint8_t		pad[8192];	// general scratch pad area for ESM
#endif
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

Status_t sm_lidmap_alloc(void);
void sm_lidmap_free(void);
Status_t sm_lidmap_reset(void);
Status_t sm_lidmap_update_missing(void);

struct MissingLidEntry;

typedef	struct {
	Guid_t		guid;		// guid for this index (ie, Lid)
	uint32_t	pass;		// last pass count I saw this Guid
	Node_t		*oldNodep;	// active pointer to node in old topology
	cl_map_obj_t	mapObj;	// for storing this item in the GuidToLidMap tree
	struct MissingLidEntry *missing;

    /* 
     * the following fields are for topology thread use only 
     * They are transient and are valid only when SM is discovering
     */
	Node_t		*newNodep;	// pointer to node in new topology
	Port_t      *newPortp;
} LidMap_t;

typedef struct MissingLidEntry {
	LidMap_t *lm;
	struct MissingLidEntry *prev;
	struct MissingLidEntry *next;
} MissingLidEntry_t;

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

#define MCMEMBER_STATE_FULL_MEMBER       0x01
#define MCMEMBER_STATE_NON_MEMBER        0x02
#define MCMEMBER_STATE_SENDONLY_MEMBER   0x04

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
	bitset_t		new_vfMembers;
	McGroupFlags	flags;      // Flags associated with this group
} McGroup_t;

#if defined(__VXWORKS__)
#define SM_STACK_SIZE (29 * 1024)
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
#define SM_THREAD_EM			7


#ifndef FE_THREAD_SUPPORT_ENABLED
#define	SM_THREAD_MAX			SM_THREAD_EM
#else
#define SM_THREAD_FE            (SM_THREAD_EM + 1)
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
	SmMaiHandle_t *fd;
	uint32_t method;
	uint32_t aid;
	uint32_t amod;
	uint8_t *path;
	STL_LID  slid;
	STL_LID  dlid;
	uint8_t  buffer[STL_MAD_PAYLOAD_SIZE];
	uint32_t bufferLength;
	uint64_t mkey;
	uint32_t reply;
	uint8_t  bversion;
	cntxt_callback_t callback;
	void *callbackContext;
} sm_dispatch_send_params_t;

typedef struct sm_dispatch_req {
	sm_dispatch_send_params_t sendParams;
	Node_t *nodep;
	uint64_t sendTime;
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
    bitset_t       vf[STL_MAX_VLS];
} VlVfMap_t;

typedef struct _VlBwMap
{
	boolean       has_qos;
    uint8_t       qos[STL_MAX_VLS];
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
	SM_SWEEP_REASON_UNQUARANTINE,
	SM_SWEEP_REASON_UNDETERMINED
} SweepReason_t;

extern SweepReason_t sm_resweep_reason;

static __inline__ void setResweepReason(SweepReason_t reason) {
	if (sm_resweep_reason == SM_SWEEP_REASON_UNDETERMINED)
		sm_resweep_reason = reason;
}

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
#define	Index(X,Y)		((X) * sm_topop->max_sws + (Y))
#define	Cost_Infinity	(32767)  /* small enough to ensure that inf+inf does not overflow */
#define	Min(X,Y)		((X) < (Y) ? (X) : (Y))
#define	Max(X,Y)		((X) > (Y) ? (X) : (Y))

#define	PathToPort(NP,PP)	((NP)->nodeInfo.NodeType == NI_TYPE_SWITCH ? 	\
				(NP)->path : (PP)->path)

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

#define Node_Dequeue_Type(TOPOP,NODEP,HEAD,TAIL) { \
	if (NODEP->type_prev) \
		NODEP->type_prev->type_next = NODEP->type_next; \
	if (NODEP->type_next) \
		NODEP->type_next->type_prev = NODEP->type_prev; \
	if (TOPOP->HEAD == NODEP) \
		TOPOP->HEAD = NODEP->type_next; \
	if (TOPOP->TAIL == NODEP) \
		TOPOP->TAIL = NODEP->type_prev; \
	NODEP->type_next = NODEP->type_prev = NULL; \
}


//
//	Macros for allocating Nodes and Ports.
//
#define sm_dynamic_port_alloc() \
    (sm_config.dynamic_port_alloc)

#define STL_MFTABLE_POSITION_COUNT 4

static __inline__ int bitsForInteger(int x) {
	int i=0;

	while (x > 0) {
		x /= 2;
		i++;
	}
	return i;
}


//
//      Topology traversal macros.
//
#define for_all_nodes(TP,NP)							\
	for (NP = (TP)->node_head; NP != NULL; NP = NP->next)

#define for_all_ca_nodes(TP,NP)							\
	for (NP = (TP)->ca_head; NP != NULL; NP = NP->type_next)

#define for_all_tier_switches(TP,NP, tier)                           \
    for (NP = (TP)->tier_head[tier]; NP != NULL; NP = NP->sw_next)

#define for_all_switch_nodes(TP,NP)						\
	for (NP = (TP)->switch_head; NP != NULL; NP = NP->type_next)

#define for_switch_list_switches(HD, NP)					\
	for (NP = (HD); NP != NULL; NP = NP->next)


#define for_all_quarantined_nodes(TP,NP)					\
	for (NP = (TP)->quarantined_node_head; NP != NULL; NP = NP->next)

#define for_each_list_item(LIST, ITEM) \
	for (ITEM = QListHead(LIST); ITEM != NULL; ITEM = QListNext(LIST, ITEM))

// --------------------------------------------------------------------------- //
static __inline__ int sm_valid_port(const Port_t * portp) {
    return ((portp != NULL) && (portp->portData != NULL));
}

static __inline__ int sm_valid_port_mgmt_allowed_pkey(Port_t * portp) {
	if (portp->portData->pPKey[STL_DEFAULT_FM_PKEY_IDX].AsReg16 == STL_DEFAULT_FM_PKEY ||
		portp->portData->pPKey[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 == STL_DEFAULT_FM_PKEY)
		return 1;
    else
		return 0;
}


static inline
const char * sm_fwd_table_type_str(Topology_t *topop)
{
	return "LFT";
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

static __inline__ boolean is_cc_supported_by_enhanceport0(Node_t *nodep) {
	if (nodep->switchInfo.u2.s.EnhancedPort0 && (nodep->congestionInfo.ControlTableCap != 0)) {
		return TRUE;
        }
	return FALSE;
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

#define	McGroup_Create(GROUPP) {						\
	size_t		local_size;						\
	Status_t	local_status;						\
										\
	local_size = sizeof(McGroup_t) + 16;					\
	local_status = vs_pool_alloc(&sm_pool, local_size, (void *)&GROUPP);	\
	if (local_status != VSTATUS_OK) {					\
		IB_FATAL_ERROR_NODUMP("can't allocate space");				\
	}									\
										\
	memset((void *)GROUPP, 0, local_size);					\
	if (!bitset_init(&sm_pool, &GROUPP->vfMembers, MAX_VFABRICS)) { \
		IB_FATAL_ERROR_NODUMP("McGroup_Create: can't allocate space");				\
	}									\
	if (!bitset_init(&sm_pool, &GROUPP->new_vfMembers, MAX_VFABRICS)) { \
		IB_FATAL_ERROR_NODUMP("McGroup_Create: can't allocate space");				\
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
	bitset_free(&GROUPP->new_vfMembers); \
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
		IB_FATAL_ERROR_NODUMP("can't allocate space");				\
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
			 
// Cost should be evenly divisable by (width * SpeedFactor)
#define   SpeedWidth_to_Cost(X)   ((X) ? 1200/(X) : 1200)

static __inline__ uint8_t sm_GetLsf(STL_PORT_INFO *portInfo) {
	switch (portInfo->LinkSpeed.Active) {
	case STL_LINK_SPEED_25G: return 25;
	default:
		DEBUG_ASSERT(FALSE);
		return 25;
	}
}

static __inline__ uint64_t sm_GetBandwidth(STL_PORT_INFO *portInfo) {
	uint64_t speed;

	switch (portInfo->LinkSpeed.Active) {
	case STL_LINK_SPEED_25G: speed = 25781250000LL; break;
	default:
		DEBUG_ASSERT(FALSE);
		speed = 25781250000LL;
		break;
	}

	// Calculate bytes/sec BW: speed * width * 64/66 (encoding overhead) * 1/8 (bits to bytes)
	return speed * StlLinkWidthToInt(portInfo->LinkWidth.Active)*4/33;;
}

static __inline__ uint16_t sm_GetSpeed(PortData_t *portData) {
	return (sm_GetLsf(&portData->portInfo) * StlLinkWidthToInt(portData->portInfo.LinkWidth.Active));
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

static __inline__ int sm_IsVLrSupported(const Node_t *nodep, const Port_t *portp) {

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t * port0 = sm_get_port(nodep, 0);
		if (sm_valid_port(port0) && port0->state > IB_PORT_DOWN &&
			port0->portData->portInfo.CapabilityMask3.s.IsVLrSupported)
			return 1;

	} else if (portp->portData->portInfo.CapabilityMask3.s.IsVLrSupported)
		return 1;

	return 0;
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

static __inline__ boolean is_swport(Port_t *portp) {
	return portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH;
}

static __inline__ boolean is_hfiport(Port_t *portp) {
	return portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_CA;
}

static __inline__ boolean is_swport0(Port_t *portp) {
	return (portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0);
}

static __inline__ boolean is_extswport(Port_t *portp) {
	return (portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index > 0);
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

extern 	uint32_t 	sm_needed_vls;
extern 	uint32_t 	sm_def_mc_group;
extern  uint8_t		sm_env[32];
extern	STL_LID 	sm_lid;
extern	STL_LID 	sm_lmc_0_freeLid_hint;
extern	STL_LID 	sm_lmc_e0_freeLid_hint;
extern	STL_LID 	sm_lmc_freeLid_hint;
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

extern int topology_main_exit;

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
extern uint32_t     sm_debug;
extern  uint32_t    smDebugPerf;  // control SM/SA performance messages; default off in ESM
extern  uint32_t    smFabricDiscoveryNeeded;  // smFabricDiscoveryNeeded is shared between threads to synchronize sweeps
extern  uint64_t    lastTimeDiscoveryRequested;  // lastTimeDiscoveryRequested is shared between threads to synchronize sweeps
extern  uint32_t    smDebugDynamicAlloc; // control SM/SA memory allocation messages; default off in ESM
extern  uint32_t    sm_trapThreshold; // threshold of traps/min for port auto-disable
extern  uint32_t	sm_trapThreshold_minCount; //minimum number of traps to validate sm_trapThreshold
extern  uint64_t    sm_trapThresholdWindow; // time window for observing traps in milliseconds
extern	STL_LID 	sm_mcast_mlid_table_cap;

extern	uint16_t    sm_masterSmSl;
extern  bitset_t	sm_linkSLsInuse;
extern	int			sm_QosConfigChange;

extern SmAdaptiveRouting_t sm_adaptiveRouting;

extern uint32_t	sm_useIdealMcSpanningTreeRoot;
extern uint32_t	sm_mcSpanningTreeRoot_useLeastWorstCaseCost;
extern uint32_t	sm_mcSpanningTreeRoot_useLeastTotalCost;
extern uint32_t	sm_mcRootCostDeltaThreshold;
/************ Multicast Globals *********************************/
extern uint32_t sm_numMcGroups;
extern ATOMIC_UINT sm_McGroups_Need_Prog;
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
extern STL_LID  loopPathLidStart;
extern STL_LID  loopPathLidEnd;
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

extern Popo_t sm_popo;

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
Status_t	SM_Get_NodeDesc(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_NODE_DESCRIPTION *);
Status_t	SM_Get_NodeInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_NODE_INFO *);
Status_t	SM_Get_PortInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_PORT_INFO *);
Status_t	SM_Set_PortInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_PORT_INFO *, uint64_t, uint32_t *);
Status_t	SM_Set_PortInfo_Dispatch(SmMaiHandle_t *, uint32_t amod, SmpAddr_t *addr, STL_PORT_INFO *pip, uint64_t mkey, Node_t *nodep, sm_dispatch_t *disp, cntxt_callback_t callback, void *cb_data);
Status_t	SM_Get_PortStateInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_PORT_STATE_INFO *);
Status_t	SM_Get_PortStateInfo_Dispatch(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, Node_t *nodep, sm_dispatch_t *disp, cntxt_callback_t callback, void *cb_data);
Status_t 	SM_Set_PortStateInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_PORT_STATE_INFO *, uint64_t);
Status_t	SM_Get_SwitchInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SWITCH_INFO *);
Status_t	SM_Set_SwitchInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SWITCH_INFO *, uint64_t);
Status_t	SM_Get_SMInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SM_INFO *);
Status_t	SM_Set_SMInfo(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SM_INFO *, uint64_t);
Status_t	SM_Get_VLArbitration(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_VLARB_TABLE *);
Status_t	SM_Set_VLArbitration(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *, STL_VLARB_TABLE *vlp, size_t vlpSize, uint64_t mkey);

Status_t	SM_Get_SLSCMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *, STL_SLSCMAP *slscp);
Status_t	SM_Set_SLSCMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *, STL_SLSCMAP *slscp, uint64_t mkey);
Status_t	SM_Get_SCSLMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCSLMAP *);
Status_t	SM_Set_SCSLMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCSLMAP *, uint64_t);
Status_t	SM_Get_SCVLtMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCVLMAP *);
Status_t	SM_Set_SCVLtMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCVLMAP *, uint64_t);
Status_t	SM_Get_SCVLntMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCVLMAP *);
Status_t	SM_Set_SCVLntMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCVLMAP *, uint64_t);
Status_t	SM_Get_SCVLrMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCVLMAP *);
Status_t	SM_Set_SCVLrMap(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCVLMAP *, uint64_t);
Status_t	SM_Set_SCSC(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_SCSCMAP *, uint64_t);
Status_t	SM_Set_SCSCMultiSet(SmMaiHandle_t *fd, uint32_t, SmpAddr_t *, STL_SCSC_MULTISET *, uint64_t);
Status_t	SM_Set_LFT(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_LINEAR_FORWARDING_TABLE *, uint64_t);
Status_t	SM_Set_LFT_Dispatch(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_LINEAR_FORWARDING_TABLE *, uint16_t, uint64_t, Node_t *, sm_dispatch_t *);


Status_t	SM_Set_MFT_Dispatch(SmMaiHandle_t *, uint32_t, SmpAddr_t *, STL_MULTICAST_FORWARDING_TABLE *, uint64_t, Node_t *, sm_dispatch_t *);
Status_t	SM_Get_PKeyTable(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PARTITION_TABLE *pkp);
Status_t	SM_Set_PKeyTable(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PARTITION_TABLE *pkp, uint64_t);
Status_t	SM_Set_PKeyTable_Dispatch(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PARTITION_TABLE *pkp, uint64_t mkey, Node_t *nodep, sm_dispatch_t *disp, cntxt_callback_t callback, void *cb_data);
Status_t	SM_Get_PortGroup(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_GROUP_TABLE *pgp, uint8_t blocks);
Status_t	SM_Set_PortGroup(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_GROUP_TABLE *pgp, uint8_t blocks, uint64_t mkey);
Status_t	SM_Get_PortGroupFwdTable(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_GROUP_FORWARDING_TABLE *pp, uint8_t blocks);
Status_t	SM_Set_PortGroupFwdTable(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_GROUP_FORWARDING_TABLE *pp, uint8_t blocks, uint64_t mkey);
Status_t	SM_Get_BufferControlTable(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_BUFFER_CONTROL_TABLE pbct[]);
Status_t	SM_Set_BufferControlTable(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_BUFFER_CONTROL_TABLE pbct[], uint64_t mkey, uint32_t* madStatus);
Status_t	SM_Get_CongestionInfo(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_CONGESTION_INFO * congestionInfo);
Status_t	SM_Get_HfiCongestionSetting(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_SETTING *hfics);
Status_t	SM_Set_HfiCongestionSetting(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_SETTING *hfics, uint64_t mkey);
Status_t	SM_Get_HfiCongestionControl(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_CONTROL_TABLE *hficct);
Status_t	SM_Set_HfiCongestionControl(SmMaiHandle_t * fd, uint16 CCTI_Limit, const uint8_t numBlocks, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK *hficct, uint64_t mkey);
Status_t	SM_Get_SwitchCongestionSetting(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_CONGESTION_SETTING *swcs);
Status_t	SM_Set_SwitchCongestionSetting(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_CONGESTION_SETTING *swcs, uint64_t mkey);
Status_t	SM_Get_SwitchPortCongestionSetting(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_PORT_CONGESTION_SETTING *swpcs);
Status_t	SM_Get_LedInfo(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_LED_INFO *li);
Status_t	SM_Set_LedInfo(SmMaiHandle_t * fd, uint32_t amod, SmpAddr_t *addr, STL_LED_INFO *li, uint64_t mkey);

/**
 	Do an SMA Get(CableInfo) on the specified endpoint.

	Operates in 64-byte half-pages even though CableInfo is stored in 128-byte pages.

 	@param startSeg the starting 64-byte segment
	@param segCount the number of 64-byte segments to request
 	@param [out] ci points to start of output.  Must have enough space for all request segments.
	@param [out] madStatus The MAD status of the Get(CableInfo); set to 0 at start
*/
Status_t SM_Get_CableInfo(SmMaiHandle_t * fd, uint8_t portIdx, uint8_t startSeg, uint8_t segCount, SmpAddr_t *addr, STL_CABLE_INFO * ci, uint32_t * madStatus);

/**
	Perform Get(Aggregate) with the requests in [start, end).  See @c SM_Aggregate_impl() for more details.
*/
Status_t
SM_Get_Aggregate_LR(SmMaiHandle_t * fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	size_t lastSegReqLen, STL_LID srcLid, STL_LID destLid, STL_AGGREGATE ** lastSeg,
	uint32_t * madStatus);

/**
  Perform Set(Aggregate) with the requests in [start, end).  See @c SM_Aggregate_impl() for more details.
*/
Status_t
SM_Set_Aggregate_LR(SmMaiHandle_t * fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	STL_LID srcLid, STL_LID destLid, uint64_t mkey, STL_AGGREGATE ** lastSeg, uint32_t * madStatus);

/**
	Perform DR Get(Aggregate) with the requests in [start, end).  See @c SM_Aggregate_impl() for more details.
*/
Status_t
SM_Get_Aggregate_DR(SmMaiHandle_t * fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	size_t lastSegReqLen, uint8_t *path, STL_AGGREGATE ** lastSeg, uint32_t * madStatus);

//
//	Multicast data structures.
//
typedef	struct _McNode {
	int32_t		index;		// node index
	int32_t		nodeno;		// closest node in the tree
	int32_t		portno;		// port on closest node
	int32_t		height;		// distance from the root
	struct _McNode *parent; // parent in spanning tree
	STL_LID		mft_mlid_init;
} McNode_t;

typedef	struct _McSpanningTree {
	int32_t		num_nodes;
	McNode_t	*nodes;
	STL_LID		first_mlid;
	uint32_t    reserved;   // Padding to make the size a multiple of 8 bytes
} McSpanningTree_t;

typedef struct McSpaningTrees {
	McSpanningTree_t	*spanningTree;
	uint8_t				copy;	// Is this a shallow copy of another tree?
} McSpanningTrees_t;


//
//	Externs.
//
extern	SMThread_t	* sm_threads;

extern	SmMaiHandle_t	*fd_sa;
extern	SmMaiHandle_t	*fd_sa_writer;
extern	SmMaiHandle_t	*fd_multi;
extern	SmMaiHandle_t	*fd_async;
extern	SmMaiHandle_t	*fd_async_request;
extern	SmMaiHandle_t	*fd_saTrap;
extern	SmMaiHandle_t	*fd_policy;
extern	SmMaiHandle_t	*fd_topology;
extern	SmMaiHandle_t	*fd_loopTest;
extern	SmMaiHandle_t	*fd_atopology;
extern	SmMaiHandle_t	*fd_flapping_port;

extern	int ib_mad_dump_flag;	// JSY - debug only

extern McSpanningTree_t **uniqueSpanningTrees;
extern int	uniqueSpanningTreeCount;

extern Node_t *sm_mcSpanningTreeRootSwitch;
extern uint64_t sm_mcSpanningTreeRootGuid;

extern Status_t sm_update_mc_groups(VirtualFabrics_t *newVF, VirtualFabrics_t *oldVF);


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
extern void sm_discovery_needed(const char*, STL_LID);

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
void	sm_setup_SC2VL(VirtualFabrics_t *);

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

void sm_portinfo_nop_init(STL_PORT_INFO *pi);

Status_t	sm_calculate_lft(Topology_t *topop, Node_t *switchp);
Status_t	sm_write_minimal_lft_blocks(Topology_t *topop, Node_t *switchp, SmpAddr_t * addr);
Status_t	sm_write_full_lfts_by_block_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance);
/// Recomputes and sends LFTs, PGTs, and PGFTs
Status_t	sm_setup_lft(Topology_t *, Node_t *);
Status_t	sm_send_lft(Topology_t *, Node_t *);
Status_t	sm_setup_lft_deltas(Topology_t *, Topology_t *, Node_t *);
Status_t	sm_send_partial_lft(Topology_t *, Node_t *, bitset_t *);
Status_t	sm_calculate_all_lfts(Topology_t *);

Status_t	sm_select_path_lids(Topology_t *, Port_t *, STL_LID, Port_t *, STL_LID , STL_LID *, uint8_t *, STL_LID *, uint8_t *);

Status_t	sm_get_stl_attribute(SmMaiHandle_t * fd, uint32_t aid, uint32_t amod, SmpAddr_t * addr, uint8_t *buffer, uint32_t *bufferLength);
Status_t	sm_set_stl_attribute(SmMaiHandle_t *, uint32_t, uint32_t, SmpAddr_t *, uint8_t *, uint32_t *, uint64_t);
Status_t 	sm_set_stl_attribute_mad_status(SmMaiHandle_t *, uint32_t, uint32_t, SmpAddr_t *, uint8_t *, uint32_t *, uint64_t, uint32_t*);

Status_t	sm_get_stl_attribute_async_dispatch(SmMaiHandle_t *, uint32_t, uint32_t, SmpAddr_t *, uint8_t *, uint32_t *, Node_t *, sm_dispatch_t *, cntxt_callback_t, void *);
Status_t	sm_set_stl_attribute_async_dispatch(SmMaiHandle_t *, uint32_t, uint32_t, SmpAddr_t *, uint8_t *, uint32_t *, uint64_t, Node_t *, sm_dispatch_t *, cntxt_callback_t, void *);
/**
 * Simple generic response check function for the dispatcher callback function
 * to use.
 *
 * @param cntxt   generic callback struct containg send context which contains
 *  			  request mad.
 * @param status  Context status returned
 * @param nodep   optional Pointer to node (for Node GUID and Description)
 * @param portp   optional Pointer to port (for port number)
 * @param mad     response mad
 *
 * @return boolean TRUE if context status and MAD status OK otherwise FALSE
 */
boolean sm_callback_check(cntxt_entry_t *cntxt, Status_t status, Node_t *nodep, Port_t *portp, Mai_t *mad);

Status_t	sm_send_stl_request(SmMaiHandle_t * fd, uint32_t method, uint32_t aid, uint32_t amod, SmpAddr_t * addr, uint8_t *buffer, uint32_t *bufferLength, uint64_t mkey, uint32_t *madStatus);
Status_t	sm_send_stl_request_impl(SmMaiHandle_t *, uint32_t, uint32_t, uint32_t, SmpAddr_t *, uint32_t, uint8_t *, uint32_t *, uint32_t, uint64_t, cntxt_callback_t, void *, uint32_t *);
int 		sm_find_cached_node_port(Node_t *cnp, Port_t *cpp, Node_t **nodep, Port_t **portp);
int 		sm_find_cached_neighbor(Node_t *cnp, Port_t *cpp, Node_t **nodep, Port_t **portp);
int			sm_check_node_cache(Node_t *cnp, Port_t *cpp, Node_t **nodep, Port_t **portp);
Status_t	sm_check_Master(void);
Status_t	sm_initialize_port(ParallelSweepContext_t *psc, SmMaiHandle_t *, Topology_t *, Node_t *, Port_t *, SmpAddr_t *);
Status_t	sm_initialize_port_LR_DR(ParallelSweepContext_t *psc, SmMaiHandle_t *fd, Topology_t *topop, Node_t *nodep, Port_t *portp);
Status_t	sm_initialize_switch_LR_DR(Topology_t *topop, Node_t *nodep, STL_LID parent_switch_lid, uint8_t parent_portno, int routing_needed);
Status_t	sm_prep_switch_layer_LR_DR(Topology_t *topop, Node_t *nodep, SwitchList_t *swlist_head, int rebalance);
Status_t	sm_setup_switches_lrdr_wave_discovery_order(Topology_t *topop, int rebalance, int routing_needed);

//Methods for dynamic reconfiguration
void        sm_config_apply(FMXmlCompositeConfig_t *xml_config);
void        pm_config_apply(FMXmlCompositeConfig_t *xml_config);
void        fe_config_apply(FMXmlCompositeConfig_t *xml_config);
boolean     sm_config_valid(FMXmlCompositeConfig_t *xml_config, VirtualFabrics_t* newVfPtr, VirtualFabrics_t* oldVfPtr);
boolean     pm_config_valid(FMXmlCompositeConfig_t *xml_config);
boolean     fe_config_valid(FMXmlCompositeConfig_t *xml_config);


/**
	Update SLSC, SCSL, SCVLt/nt, and VLArb values on @c nodep with values from old topology and/or SMA at @c smaportp.

	For switches, get values for all ports via @c smaportp.  For HFIs, get values only for @c smaportp.

	@return VSTATUS_OK if all values were updated successfully, something else otherwise.
*/
Status_t
sm_node_updateFields(SmMaiHandle_t * fd, STL_LID slid, Node_t * nodep, Port_t * smaportp);

/**
	Updates cableInfo stored in @c portp->portData->cableInfo using SMA Get(CableInfo).

	Allocates memory for @c portp->portData->cableInfo if not already allocated; otherwise uses existing memory.

	@return VSTATUS_OK on success, VSTATUS_NOSUPPORT if @c portp does not support Get(CableInfo), or VSTATUS_BAD on runtime error.
*/
Status_t sm_update_cableinfo(ParallelSweepContext_t *, SmMaiHandle_t *, Topology_t *, Node_t *, Port_t * portp);

Status_t 	sm_get_node_port_states(SmMaiHandle_t * fd, Topology_t *, Node_t *, Port_t *, uint8_t *, STL_PORT_STATE_INFO **, ParallelSweepContext_t *psc);
Status_t 	sm_set_node_port_states(SmMaiHandle_t * fd, Topology_t *, Node_t *, Port_t *, uint8_t *, uint32_t, uint32_t, STL_PORT_STATE_INFO **);
Status_t	sm_disable_port(Topology_t *, Node_t *, Port_t *);
Status_t	sm_bounce_port(Topology_t *, Node_t *, Port_t *);
Status_t	sm_bounce_link(Topology_t *, Node_t *, Port_t *);
Status_t    sm_bounce_all_switch_ports(SmMaiHandle_t * fd, Topology_t *topop, Node_t *nodep, Port_t *portp, uint8_t *path, ParallelSweepContext_t *psc);
Status_t	sm_get_CapabilityMask(SmMaiHandle_t *, uint8_t, uint32_t *);
Status_t	sm_set_CapabilityMask(SmMaiHandle_t *, uint8_t, uint32_t);
Node_t		*sm_find_guid(Topology_t *, uint64_t);
Node_t 		*sm_find_node_by_name(Topology_t * topop, char *name);
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

/*
 * lidmap functions
 */
boolean		sm_get_expected_lid_lmc(const Port_t *pp, STL_LID *expLid, uint8_t *expLmc);

typedef enum {
	LIDCHECK_OK = 0,
	LIDCHECK_OUTOFRANGE = 1, // outside unicast range
	LIDCHECK_OFFLMC = 2, // LID is not % lid_offset (usually 1 << LMC)
	LIDCHECK_UNAVAIL = 3 // LID is assigned to another port
} LidCheck_t;
LidCheck_t	sm_check_lid(const Port_t *pp, STL_LID lid, uint8_t lmc);

/**
 * @return number of LIDs newly-registered to @c portp
 * or -1 on error.
 */
int			sm_set_lid(Port_t * portp, STL_LID lid, uint8_t lmc);

Status_t sm_update_or_assign_lid(Port_t *pp, boolean assignIfFail, int *newLidCount, Node_t *neighbor);

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
void		sm_log_topology(Topology_t *);
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
Status_t    sm_enable_port_led(SmMaiHandle_t * fd, Node_t *nodep, Port_t *portp, boolean enabled);
void		sm_mark_link_down(Topology_t *topop, Port_t *portp);
void		sm_mark_switch_down(Topology_t *topop, Node_t *swnodep);
Status_t	sm_mark_new_endnode(Node_t * nodep);

//
// sm_routing.c prototypes
//

int      sm_get_route(Topology_t *, Node_t *, uint8_t, STL_LID, uint8_t*);
uint8_t  sm_get_slsc(Topology_t *, Port_t*, uint8_t);
Status_t sm_routing_copy_cost_matrix(Topology_t *src_topop, Topology_t *dst_topop);
Status_t sm_routing_prep_new_switch(Topology_t *topop, Node_t *nodep, SmpAddr_t *addr);
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
extern int sm_routing_func_routing_mode_noop(void);
extern int sm_routing_func_routing_mode_linear(void);
extern boolean sm_routing_func_false(void);
extern boolean sm_routing_func_true(void);
extern boolean sm_routing_func_false(void);
extern boolean sm_routing_func_true(void);
extern Status_t sm_routing_func_copy_routing_noop(Topology_t *src_topop, Topology_t *dst_topop);
extern Status_t sm_routing_func_copy_routing_lfts(Topology_t *src_topop, Topology_t *dst_topop);
extern Status_t sm_routing_func_init_switch_routing_lfts(Topology_t * topop, int * routing_needed, int * rebalance);
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
extern Status_t sm_routing_func_select_path_lids(Topology_t *topop, Port_t *srcPortp, STL_LID slid, Port_t *dstPortp, STL_LID dlid, STL_LID *outSrcLids, uint8_t *outSrcLen, STL_LID *outDstLids, uint8_t *outDstLen);
extern Status_t sm_routing_func_select_updn_path_lids_noop(Topology_t *topop, Port_t *srcPortp, Port_t *dstPortp, STL_LID *outSrcLid, STL_LID *outDstLid);
extern Status_t sm_routing_func_process_swIdx_change_noop(Topology_t * topop, int old_idx, int new_idx, int last_idx);
extern int sm_routing_func_check_switch_path_change(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp);
extern boolean sm_routing_func_needs_routing_recalc_false(Topology_t * topop, Node_t * nodep);
extern boolean sm_routing_func_needs_routing_recalc_true(Topology_t * topop, Node_t * nodep);
extern boolean sm_routing_func_needs_lft_recalc(Topology_t * topop, Node_t * nodep);
extern boolean sm_routing_func_do_spine_check(Topology_t * topop, Node_t * switchp);
extern Status_t sm_routing_func_write_minimal_lft_blocks(Topology_t * topop, Node_t * switchp, SmpAddr_t * addr);
extern Status_t sm_routing_func_write_full_lfts_LR(Topology_t * topop, SwitchList_t * swlist, int rebalance);
extern Status_t sm_routing_func_route_old_switch(Topology_t *src_topop, Topology_t *dst_topop, Node_t *nodep);
extern boolean sm_routing_func_handle_fabric_change(Topology_t *topop, Node_t *oldSwitchp, Node_t *switchp);
extern Status_t sm_routing_func_update_bw(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern Status_t sm_routing_func_assign_scs_to_sls_fixedmap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern Status_t sm_routing_func_assign_scs_to_sls_nonfixedmap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern Status_t sm_routing_func_assign_sls(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern int sm_routing_func_min_vls(void);
extern int sm_routing_func_max_vls(void);
extern int sm_routing_func_one_routing_scs(int sl, boolean mc_sl);
extern int sm_routing_func_no_oversubscribe(int sl, boolean mc_sl);
extern Status_t sm_routing_func_process_xml_config_noop(void);
//
// sm_shortestpath.c prototypes
//

Status_t sm_shortestpath_init(void);

//
// sm_fattree.c prototypes
//

Status_t sm_fattree_init(void);

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
void sm_dispatch_clear(sm_dispatch_t *disp);
Status_t sm_dispatch_update(sm_dispatch_t *disp);
void sm_dispatch_bump_passcount(sm_dispatch_t *disp);

//
// sm_partMgr.c prototypes
//

int 		checkPKey(uint16_t);
int			smCheckPortPKey(uint16_t, Port_t*);

uint16_t	smGetPortPkey(uint16_t, Port_t*);

Status_t	setPKey(uint32_t, uint16_t, int);
Status_t	sm_set_portPkey(ParallelSweepContext_t *psc, SmMaiHandle_t *, Topology_t *, Node_t *, Port_t *, Node_t *, Port_t *, SmpAddr_t *, uint8_t*, uint16_t*);
Status_t	sm_set_local_port_pkey(STL_NODE_INFO *nodeInfop);
Status_t	sm_set_delayed_pkeys(void);

uint16_t	getDefaultPKey(void);
uint16_t	getPKey(uint8_t);		 // getPKey is needed for esm (sm mib).

uint16_t	smGetRequestPkeyIndex(uint8_t,  STL_LID);
uint16_t	smGetCommonPKey(Port_t*,  Port_t*);
int 		smValidatePortPKey(uint16_t, Port_t*);
int			smValidatePortPKey2Way(uint16_t, Port_t*, Port_t*);

void		smGetVfMaxMtu(Port_t*, Port_t*, STL_MCMEMBER_RECORD*, uint8_t*, uint8_t*);

// validate serviceId for VF.  Also covers the unmatchedSid option
Status_t	smVFValidateVfServiceId(int vf, uint64_t serviceId);
Status_t	smVFValidateVfMGid(VirtualFabrics_t *VirtualFabrics, int vf, uint64_t mGid[2]);
Status_t	smGetValidatedVFs(Port_t*, Port_t*, uint16_t, uint8_t, uint8_t, uint64_t, uint8_t, bitset_t*);
Status_t	smVFValidateMcGrpCreateParams(Port_t * joiner, Port_t * requestor,
                                          STL_MCMEMBER_RECORD * mcMemberRec, bitset_t * vfMembers);
void		smVerifyMcastPkey(uint64_t*, uint16_t);
void		smSetupNodeVFs(Node_t *nodep);
void        smSetupNodeDGs(Node_t *nodep);
boolean     smEvaluateNodeDG(Node_t* nodep, int dgIdxToEvaluate, PortRangeInfo_t* portInfo);
boolean     smEvaluatePortDG(Node_t* nodep, Port_t* portp, int dgIdxToEvaluate, bitset_t* dgMemberForPort, bitset_t* dgsEvaluated);
void		smLogVFs(void);
char*		smGetVfName(uint16_t);

//
// sm_ar.c prototypes
//	 Send PGT and PGFT to node if necessary.
//
Status_t	sm_AdaptiveRoutingSwitchUpdate(ParallelSweepContext_t *, SmMaiHandle_t *, Topology_t*, Node_t*);
uint8_t		sm_VerifyAdaptiveRoutingConfig(Node_t *p);

//
// sm_multicast.c prototypes
//

extern IB_GID nullGid;

//
// mLid usage structure
//
typedef struct {
	cl_map_item_t PKeyItem;
	uint16_t PKey;
	uint16_t PKeyCount;
} PKeyUsage_t;

typedef struct {
	STL_LID lid;
	uint16_t usageCount;
} LidUsage_t;


typedef struct {
	cl_map_item_t mapItem; // Map item for group class usageMap
	uint32_t lidsUsed; // number of lids in the lid class
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
	uint32_t maximumLids;

	// Number of mLids currently in use by the group class
	uint32_t currentLids;
	// Number of mlids that can be shared within the same PKey
	uint32_t maximumLidsperPkey;

	// Mask and value for MGid's belonging to this group class
	IB_GID mask;
	IB_GID value;

	// QMap of LidClass_t structure, indexed by {pkey, mtu, rate}
	cl_qmap_t usageMap;
	// QMap of PKeyUsage_t structure, indexed by {pkey}
	cl_qmap_t PKeyUsageMap;
} McGroupClass_t;

typedef struct {
	ParallelSweepContext_t *psc;
} SweepContext_t;

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
int         smGetDgIdx(DGXmlConfig_t *dg_config, char* dgName);
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
Status_t	sm_multicast_add_group_class(IB_GID mask, IB_GID value, STL_LID maxLidsi, uint32_t maxLidsPkey);
Status_t	sm_multicast_set_default_group_class(STL_LID maxLids, uint32_t maxLidsPKey);
Status_t	sm_multicast_sync_lid(IB_GID mGid, PKey_t pKey, uint8_t mtu, uint8_t rate, STL_LID lid);
Status_t    sm_multicast_check_sync_consistancy(McGroupClass_t * groupClass, PKey_t pKey,
                                                uint8_t mtu, uint8_t rate, STL_LID lid);
STL_LID		sm_multicast_get_max_lid(void);
void		sm_multicast_mark_lid_in_use(STL_LID mlid);
Status_t	sm_multicast_gid_check(IB_GID);
Status_t	sm_multicast_gid_assign(uint32_t, IB_GID);
Status_t	sm_multicast_gid_valid(uint8_t, IB_GID);
Status_t	sm_calculate_mfts(void);
Status_t	sm_set_all_mft(int force, Topology_t *curr_tp, Topology_t *prev_tp);
Status_t	sm_multicast_switch_mft_copy(void);
McGroup_t	*sm_find_multicast_gid(IB_GID);
Status_t	sm_multicast_assign_lid(IB_GID mGid, PKey_t pKey, uint8_t mtu, uint8_t rate, STL_LID requestedLid, STL_LID * lid);
Status_t	sm_multicast_decommision_group(McGroup_t * group);
McGroup_t	*sm_find_next_multicast_gid(IB_GID);
McMember_t	*sm_find_multicast_member(McGroup_t *, IB_GID);
McMember_t	*sm_find_multicast_member_by_index(McGroup_t *mcGroup, uint32_t index);
McMember_t	*sm_find_next_multicast_member_by_index(McGroup_t *mcGroup, uint32_t index);
void		sm_multicast_init_mlid_list(void);
void		sm_multicast_destroy_mlid_list(void);

void		topology_main(uint32_t, uint8_t **);
Status_t	sweep_userexit(SweepContext_t *);
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
void		sm_get_smAdaptiveRouting(fm_ar_config_t *);
void		smAdaptiveRoutingUpdate(uint32_t, fm_ar_config_t);
void        smPauseResumeSweeps(boolean);
void		smProcessReconfigureRequest(void);
PortData_t *sm_alloc_port(Topology_t *topop, Node_t *nodep, uint32_t portIndex);
void        sm_free_port(Topology_t *topop, Port_t * portp);
void        sm_node_free_port(Topology_t *topop, Node_t *nodep);
Status_t    sm_build_node_array(Topology_t *topop);
Status_t    sm_clearIsSM(void);
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

extern void sm_copyPortDataSCSCMap(Port_t *sportp, Port_t *dportp, int extended);
extern void sm_addPortDataSCSCMap(Port_t *portp, uint8_t outport, int extended, const STL_SCSCMAP *pSCSC);
extern STL_SCSCMAP * sm_lookupPortDataSCSCMap(Port_t *portp, uint8_t outport, int extended);

extern Status_t sm_get_buffer_control_tables(SmMaiHandle_t * fd_topology, Node_t *nodep, uint8_t start_port, uint8_t end_port);

extern void sm_set_force_attribute_rewrite(uint32_t force_attr_rewrite);
extern void sm_set_skip_attribute_write(uint32_t skip_attr_write);

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

//
// persistent topology api
//

extern void sm_popo_init(Popo_t * popop);
extern void sm_popo_destroy(Popo_t * popop);
extern void sm_popo_report(Popo_t * popop);
extern PopoNode_t * sm_popo_get_node(Popo_t * popop, const STL_NODE_INFO * nodeInfo);
extern PopoPort_t * sm_popo_get_port(Popo_t * popop, PopoNode_t * ponodep, uint8_t port);

extern boolean sm_popo_is_port_quarantined(Popo_t * popop, Port_t * portp);
extern boolean sm_popo_is_port_monitored(Popo_t * popop, Port_t * portp);
extern boolean sm_popo_is_port_quarantined_unsafe(Popo_t * popop, Port_t * portp);
extern boolean sm_popo_is_port_monitored_unsafe(Popo_t * popop, Port_t * portp);
extern boolean sm_popo_is_node_quarantined(Popo_t * popop, uint64_t guid);
extern boolean sm_popo_clear_short_quarantine(Popo_t * popop);

extern PopoQuarantineType_t sm_popo_get_quarantine_type(Popo_t * popop, Port_t * portp);
extern PopoLongTermQuarantineReason_t sm_popo_get_quarantine_reason(Popo_t * popop, Port_t * portp);
extern PopoQuarantineType_t sm_popo_get_quarantine_type_unsafe(Popo_t * popop, Port_t * portp);
extern PopoLongTermQuarantineReason_t sm_popo_get_quarantine_reason_unsafe(Popo_t * popop, Port_t * portp);

extern uint64_t sm_popo_scale_timeout(Popo_t * popop, uint64_t timeout);
extern void sm_popo_report_timeout(Popo_t * popop, uint64_t timeout);
extern Status_t sm_popo_port_error(Popo_t * popop, Topology_t * topop, Port_t * portp, Status_t status);
extern void sm_popo_reset_errors(Popo_t * popop);
extern boolean sm_popo_should_abandon(Popo_t * popop);
extern void sm_popo_report_trap(Popo_t * popop);
extern void sm_popo_update_node(PopoNode_t * ponodep);
extern void sm_popo_end_sweep(Popo_t * popop);

extern void sm_popo_monitor_port(Popo_t *popop, Port_t *portp, PopoLongTermQuarantineReason_t reason);
extern void sm_popo_update_node_port_states(Popo_t *popop, Node_t * nodep, STL_PORT_STATE_INFO *psi);
extern void sm_popo_update_port_state(Popo_t *popop, Port_t *portp, STL_PORT_STATES *pstatep);
extern void sm_popo_clear_port_trappedDown(Popo_t * popop);

/**
 * popo locks should not be taken for calling the above popo APIs unless
 * needLock is set to FALSE in API call. popo locks are publicly available 
 * for popo extensions that directly modify popo protected state 
 */
extern void sm_popo_lock(Popo_t * popop);
extern void sm_popo_unlock(Popo_t * popop);


//port Flapping API
extern boolean sm_flap_report_port_change_trap(Popo_t *popop, Node_t *nodep);
extern void sm_flap_state_handler(Popo_t *popop, PopoPort_t *poportp);
extern void sm_flap_gc(PopoPort_t *poportp);


static inline
Node_t * Node_Create(Topology_t *topop, const STL_NODE_INFO *nodeInfo,
  uint8 nodeType, uint8 portCount, const STL_NODE_DESCRIPTION *nodeDesc, boolean authentic)
{
	int		local_i;
	size_t		local_size;
	Status_t	local_status;
	//char		nodeDescStr[STL_NODE_DESCRIPTION_ARRAY_SIZE+1];
	char		nodeDescStr[ND_LEN+1];
	Node_t *nodep = NULL;

	local_size = sizeof(Node_t) + (portCount * sizeof(Port_t)) + 16;
	local_status = vs_pool_alloc(&sm_pool, local_size, (void *)&nodep);
	if (local_status == VSTATUS_OK) {
		memset((void *)nodep, 0, local_size);
		nodep->nodeInfo = *nodeInfo;
		nodep->nodeDesc = *nodeDesc;
		memcpy(nodeDescStr, nodeDesc->NodeString, ND_LEN);
		if (strlen(nodeDescStr) == ND_LEN) {
			local_status = vs_pool_alloc(&sm_pool, ND_LEN+1, (void *)&nodep->nodeDescString);
			if (local_status != VSTATUS_OK) {
				IB_FATAL_ERROR_NODUMP("Can't allocate space for node's description");
			}
			memcpy((void *)nodep->nodeDescString, nodeDescStr, ND_LEN+1);
		}
		if (authentic) {
			if (nodeType == NI_TYPE_CA) {
				Node_Enqueue_Type(topop, nodep, ca_head, ca_tail);
			} else if (nodeType == NI_TYPE_SWITCH) {
			  if (sm_mcast_mlid_table_cap) {
				local_status = vs_pool_alloc(&sm_pool, sizeof(STL_PORTMASK*) * sm_mcast_mlid_table_cap, (void*)&nodep->mft);
				if (local_status == VSTATUS_OK) {
					local_status = vs_pool_alloc(&sm_pool, sizeof(STL_PORTMASK) * sm_mcast_mlid_table_cap * STL_MFTABLE_POSITION_COUNT, (void*)&nodep->mft[0]);
					if (local_status == VSTATUS_OK) {
						memset((void *)nodep->mft[0], 0, (sizeof(STL_PORTMASK) * sm_mcast_mlid_table_cap * STL_MFTABLE_POSITION_COUNT));
						for (local_i = 1; local_i < sm_mcast_mlid_table_cap; ++local_i) {
							nodep->mft[local_i] = nodep->mft[local_i - 1] + STL_MFTABLE_POSITION_COUNT;
						}
						Node_Enqueue_Type(topop, nodep, switch_head, switch_tail);
					} else {
						IB_FATAL_ERROR_NODUMP("Can't allocate space for node's mft");
					}
				} else {
					IB_FATAL_ERROR_NODUMP("Can't allocate space for node's mft pointers");
				}
			  } else {
				Node_Enqueue_Type(topop, nodep, switch_head, switch_tail);
			  }
			}
			if (!bitset_init(&sm_pool, &nodep->activePorts, portCount) ||
				!bitset_init(&sm_pool, &nodep->initPorts, portCount) ||

				!bitset_init(&sm_pool, &nodep->vfMember, MAX_VFABRICS) ||
				!bitset_init(&sm_pool, &nodep->fullPKeyMember, MAX_VFABRICS) ||
				!bitset_init(&sm_pool, &nodep->dgMembership, MAX_VFABRIC_GROUPS)) {
				IB_FATAL_ERROR_NODUMP("Can't allocate space for node's activePorts");
			}
			if (local_status == VSTATUS_OK) {

				Node_Enqueue(topop, nodep, node_head, node_tail);

			}
		}
		if (local_status == VSTATUS_OK) {
			nodep->port = (Port_t *)(nodep+1);
			nodep->uniformVL = 1;
			nodep->ponodep = sm_popo_get_node(&sm_popo, nodeInfo);
			for (local_i = 0; local_i < (int)(portCount); local_i++) {
				(nodep->port)[local_i].state = IB_PORT_NOP;
				(nodep->port)[local_i].path[0] = 0xff;
				(nodep->port)[local_i].index = local_i;
				(nodep->port)[local_i].nodeno = -1;
				(nodep->port)[local_i].portno = -1;
				(nodep->port)[local_i].poportp = sm_popo_get_port(&sm_popo, nodep->ponodep, local_i);
				if (sm_dynamic_port_alloc() && !(topop->num_nodes == 0 && local_i == sm_config.port)) {
					(nodep->port)[local_i].portData = NULL;
					continue;
				}

				if (((nodep->port)[local_i].portData = sm_alloc_port(topop, nodep, local_i)) == NULL) {
					if (topop->num_nodes == 0 && local_i == sm_config.port) {
						IB_FATAL_ERROR_NODUMP("Can't allocate SM port for the node");
					} else {
						IB_FATAL_ERROR_NODUMP("Can't allocate port for the node");
					}
				}
			}
			(nodep->port)[0].nodeno = topop->num_nodes;
			(nodep->port)[0].portno = 0;
		}
	} else {
		IB_FATAL_ERROR_NODUMP("can't allocate space");
	}

	return nodep;
}

static inline
void Node_Delete(Topology_t *topop, Node_t *nodep)
{
	// TODO should not FATAL_ERROR when freeing unless it's a truly
	// unrecoverable-from problem. Consider that passing a NULL pointer
	// into vs_pool_free() will return VSTATUS_ILLPARM
	Status_t	local_status;

	if (nodep->mft) {
		local_status = vs_pool_free(&sm_pool, nodep->mft[0]);
		if (local_status != VSTATUS_OK) {
			IB_FATAL_ERROR("Failed to free node's mft");
		}
		local_status = vs_pool_free(&sm_pool, nodep->mft);
		if (local_status != VSTATUS_OK) {
			IB_FATAL_ERROR("Failed to free node's mft pointers");
		}
	}
    if (nodep->lft) {
        local_status = vs_pool_free(&sm_pool, nodep->lft);
        if (local_status != VSTATUS_OK) {
            IB_FATAL_ERROR("Failed to free node's lft");
        }
    }
	if (nodep->pgt)	{
		vs_pool_free(&sm_pool, nodep->pgt);
	}
	if (nodep->pgft)	{
		sm_Node_release_pgft(nodep);
	}
	if (nodep->routingData) {
		topop->routingModule->funcs.delete_node(nodep);
	}
	if (nodep->nodeDescString)	{
		vs_pool_free(&sm_pool, nodep->nodeDescString);
	}
	if (nodep->portStateInfo) {
		vs_pool_free(&sm_pool, nodep->portStateInfo);
	}
	bitset_free(&nodep->activePorts);
	bitset_free(&nodep->initPorts);

	bitset_free(&nodep->vfMember);
	bitset_free(&nodep->fullPKeyMember);
	bitset_free(&nodep->dgMembership);
	sm_node_free_port(topop, nodep);
	sm_node_release_changes((nodep));
	local_status = vs_pool_free(&sm_pool, (void *)nodep);
	if (local_status != VSTATUS_OK) {
		IB_FATAL_ERROR("can't free space");
	}
}

static inline
void Node_Quarantined_Delete(Topology_t *topop, QuarantinedNode_t *nodep)
{
	if (nodep->quarantinedNode)	{
		Node_Delete(topop, nodep->quarantinedNode);
	}

	if (vs_pool_free(&sm_pool, nodep))
		IB_FATAL_ERROR("can't free space");
}

#endif	// _SM_L_H_
