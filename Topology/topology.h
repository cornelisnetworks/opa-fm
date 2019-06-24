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

#ifndef _TOPOLOGY_H
#define _TOPOLOGY_H

#include <iba/ibt.h>

#include <iba/ipublic.h>
#if !defined(VXWORKS) || defined(BUILD_DMC)
#include <iba/ib_dm.h>
#endif
#include <iba/stl_sm_priv.h>
#include <iba/stl_sa_priv.h>
#include <iba/stl_sd.h>
#include <iba/stl_pm.h>
#include <iba/stl_types.h>


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/stat.h>

#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

#include <ixml_ib.h>

#ifdef __cplusplus
extern "C" {
#endif


#define CL_TIME_DIVISOR             1000000

struct ExpectedLink_s;
struct ExpectedNode_s;
struct ExpectedSM_s;
struct NodeData_s;
struct SystemData_s;
struct omgt_port;

// Selection indicies for SCVL* tables
typedef enum { Enum_SCVLt, Enum_SCVLnt, Enum_SCVLr } ScvlEnum_t;

// selection of PortState for FindPortStatePoint, includes all of IB_PORT_STATE
// plus these additional special searches
#define PORT_STATE_SEARCH_NOTACTIVE (IB_PORT_STATE_MAX+1)	// != active
#define PORT_STATE_SEARCH_INITARMED (IB_PORT_STATE_MAX+2)	// init or armed

typedef struct PortMaskSC2SCMap_s {
	LIST_ITEM	SC2SCMapListEntry;	// QOSData.SC2SCMapList
	STL_PORTMASK	outports[STL_MAX_PORTMASK]; // egress ports for this map
	STL_SCSCMAP	*SC2SCMap;
} PortMaskSC2SCMap;

#define SC2SCMAPLIST_MAX 1
// QOS information for a Port in the fabric
typedef struct QOSData_s {
	union {
		STL_VLARB_TABLE VLArbTable[4]; // One table per type, low, high, preempt, preempt matrix
	} u;

	QUICK_LIST	SC2SCMapList[SC2SCMAPLIST_MAX];	// PortMaskSC2SCMap
	STL_SLSCMAP	*SL2SCMap; // only defined for HFI and SW port 0
	STL_SCSLMAP	*SC2SLMap; // only defined for HFI and SW port 0
	STL_SCVLMAP	SC2VLMaps[3]; // VL_t, _nt, and _r; indicies given by ScvlEnum_t
} QOSData;


// How many STL_CABLE_INFO structs to store per port
#define PORTDATA_CABLEINFO_SIZE	4
#define IFACE_MACLIST_SIZE		512

// information about an IB Port in the fabric
// for switches a GUID and LID are only available for Port 0 of the switch
// for all other switch ports, LID is port 0 LID (LID for whole switch)
// and port GUID is NA and hence 0.
typedef struct PortData_s {
	cl_map_item_t	NodePortsEntry;	// NodeData.Ports, key is PortNum
	cl_map_item_t	AllLidsEntry;	// g_AllLids, key is LID (if GUID non-zero)
	LIST_ITEM		AllPortsEntry;	// g_AllPorts
	EUI64 PortGUID;					// 0 for all but port 0 of a switch
	struct PortData_s *neighbor;	// adjacent port this is cabled to
	struct NodeData_s *nodep;		// parent node
	uint8 PortNum;					// port number within Node
	uint8	from:1;					// is this the from port in link record
									// (avoids double reporting of links)
	uint8	PmaGotClassPortInfo:1;	// have issued a ClassPortInfo
	uint8	spare:6;
	uint32 rate;			// Active rate for this port
	STL_LID	EndPortLID;				// LID to get to device with this port
	STL_PORT_INFO	PortInfo;			// do not use LocalPortNum,use PortNum above


	STL_LED_INFO LedInfo;			//Led Info for this port
	IB_PATH_RECORD *pathp;			// Path Record to send to this port
	STL_PORT_COUNTERS_DATA *pPortCounters;
	struct ExpectedLink_s *elinkp;	// if supplied in topology input
	QOSData		*pQOS;				// optional QOS
	STL_PKEY_ELEMENT	*pPartitionTable;	// optional Partition Table

	union {
		struct {
			uint32		downlinkBasePaths;
			uint32		uplinkBasePaths;
			uint32		downlinkAllPaths;
			uint32		uplinkAllPaths;
		} fatTreeRoutes;	// for TabulateRoutes of fattree
		struct {
			uint32		recvBasePaths;
			uint32		xmitBasePaths;
			uint32		recvAllPaths;
			uint32		xmitAllPaths;
		} routes;			// for TabulateRoutes of any topology
	} analysisData;	// per port holding space for transient analysis data
	STL_BUFFER_CONTROL_TABLE *pBufCtrlTable;
	// 128 table entries allocate when needed
	STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY *pCongestionControlTableEntries;
    uint16_t CCTI_Limit;

	// CableInfo is organized in 128-byte pages but is stored in
	// 64-byte half-pages
	// We only store STL_CIB_STD_HIGH_PAGE_ADDR to STL_CIB_STD_END_ADDR with
	// STL_CIB_STD_HIGH_PAGE_ADDR stored starting at pCableInfoData[0]
	uint8_t *pCableInfoData;
	void *context;					// application specific field
} PortData;

// additional information about cable for a link, from topology input
// we limit sizes to make output formatting easier
#define CABLE_LENGTH_STRLEN 10
#define CABLE_LABEL_STRLEN 57
#define CABLE_DETAILS_STRLEN 64
typedef struct CableData_s {
	char *length;	// user specified length
	char *label;	// user label on cable
	char *details;	// user description of cable
} CableData;

typedef struct ExpectedNode_s ExpectedNode;
#define PORT_DETAILS_STRLEN 64
// port selector from topology input
typedef struct PortSelector_s {
	EUI64 PortGUID;					// 0 if not specified
	EUI64 NodeGUID;					// 0 if not specified
	uint8 PortNum;					// 0-255 are valid port numbers
	uint8 gotPortNum;				// 0 if PortNum not specified
	uint8 NodeType;					// 0 if not specified
	char *NodeDesc;					// NULL if not specified
	char *details;					// user description of port
	ExpectedNode *enodep;			// associated ExpectedNode, set by
									// TopologyValidate if match is found
} PortSelector;

#define LINK_DETAILS_STRLEN 64
// additional information about a link, from topology input
typedef struct ExpectedLink_s {
	LIST_ITEM		ExpectedLinksEntry;	// g_ExpectedLinks
	PortSelector *portselp1;	// input selector
	PortSelector *portselp2;	// input selector
	PortData *portp1;		// NULL if not found
	PortData *portp2;		// NULL if not found
	// the 4 fields below could become bit fields to save space if necessary
	// MTU=4 bits, rate=6 bits, internal=1 bit, matchLevel = 3 bits
	uint8 expected_rate;	// Expected Active rate for this link, IB_STATIC_RATE enum
	uint8 expected_mtu;	// Expected Active MTU for this link, an IB_MTU enum
	uint8 internal;	// Is link an internal link
	uint8 matchLevel; // degree of match with portp1 and 2, higher is better
					// used internally when matching ports to links to pick best
					// matches
	char *details;	// user description of link
	CableData CableData;	// user supplied info about cable
	uint64	lineno;	// line number in XML of starting tag, for error messages
} ExpectedLink;

/*
 * Limited port information possibly produced originally from
 * full PortData.
 */
typedef struct ExpectedPort_s {
	uint8 PortNum;
	STL_LID lid;
	uint8 lmc;
	EUI64 PortGuid;
	ExpectedLink *elinkp;
} ExpectedPort;

#define NODE_DETAILS_STRLEN 64
// additional information about a node, from topology input
struct ExpectedNode_s {
	LIST_ITEM	ExpectedNodesEntry;	// g_ExpectedFIs, g_ExpectedSWs
	cl_map_item_t ExpectedNodeGuidMapEntry;	// key is NodeGuid
	struct NodeData_s *nodep;		// NULL if not found
	EUI64 NodeGUID;					// 0 if not specified
	EUI64 SystemImageGUID;			// 0 if not specified
	uint8 NodeType;					// 0 if not specified
	char *NodeDesc;					// NULL if not specified
	char *details;					// user description of node
	uint8 portsSize;
	ExpectedPort **ports;
	uint8 connected; //internally used to validate node is reachable, set by TopologyValidate
	void *context;					// application specific field
	uint64	lineno;	// line number in XML of starting tag, for error messages
};

#define SM_DETAILS_STRLEN 64
// additional information about a SM, from topology input
typedef struct ExpectedSM_s {
	LIST_ITEM	ExpectedSMsEntry;	// g_ExpectedSMs
	struct SMData_s *smp;			// NULL if not found
	EUI64 PortGUID;					// 0 if not specified
	EUI64 NodeGUID;					// 0 if not specified
	uint8 PortNum;					// 0-255 are valid port numbers
	uint8 gotPortNum;				// 0 if PortNum not specified
	uint8 NodeType;					// 0 if not specified
	char *NodeDesc;					// NULL if not specified
	char *details;					// user description of SM
} ExpectedSM;

#if !defined(VXWORKS) || defined(BUILD_DMC)
struct IouData_s;

typedef struct IocData_s {
	LIST_ITEM		IouIocsEntry;	// IouData.Iocs, sorted by IOC slot #
	cl_map_item_t	AllIOCsEntry;	// g_AllIOCs, key is IOC GUID
	IOC_PROFILE		IocProfile;
	uint8			IocSlot;
	struct IouData_s *ioup;	// parent IOU
	IOC_SERVICE		*Services;	// IO Services indexed by service num
	void *context;					// application specific field
} IocData;

typedef struct IouData_s {
	LIST_ITEM		AllIOUsEntry;	// g_AllIOUs
	struct NodeData_s *nodep;		// parent node
	IOUnitInfo			IouInfo;
	QUICK_LIST			Iocs;	// Iocs, IocData sorted by IOC slot #
	void *context;					// application specific field
} IouData;
#endif



// detailed information specific to an IB Switch in the fabric
typedef struct SwitchData_s {
	/**
		Lower upper-bound on number of entries allocated.

		Should be set to LinearFDBTop + 1.  Should be set to 0 when LinearFDBTop is invalid.

		Prefer LinearFDBTop & LinearFDBCap from NodeData when available.
	*/
	uint32	LinearFDBSize;
	STL_LINEAR_FORWARDING_TABLE	*LinearFDB;


	/**
		Lower upper-bound on number of entries allocated.

		Should be set to MulticastFDBTop + 1.  Should be set to 0 when 
		MulticastFDBTop is invalid.  Users are responsible for knowing where 
		the multicast address space starts.

		Prefer MulticastFDBTop & MulticastFDBCap from NodeData when available.
	*/
	uint32	MulticastFDBSize;
	STL_PORTMASK *MulticastFDB[STL_NUM_MFT_POSITIONS_MASK];
	uint8	HasBeenVisited;			//indicates 0: no; 1: yes when searching for MC routes

	/**
		Lower upper-bound on number of entries allocated.

		Should be set to PortGroupTop + 1.  Should be set to 0 when 
		PortGroupTop is invalid.  

		Prefer PortGroupTop & PortGroupCap from NodeData when available.
	*/
	uint16	PortGroupSize;
	STL_PORTMASK* PortGroupElements;

	uint32	PortGroupFDBSize;
	STL_PORT_GROUP_FORWARDING_TABLE	*PortGroupFDB;

} SwitchData;

/*
	Information about a Node in the fabric.
	The SA reports a NodeRecord per port, we coallese all nodes with
	the same GUID into a single structure.
	As such the Port specific fields in the NodeInfo (PortGUID
	and LocalPortNumber) are zeroed and should not be used.
	@c NodeInfo.(Base|Class)Version should have the same values as the data
	that was used to create this record.
*/
typedef struct NodeData_s {
	cl_map_item_t	AllNodesEntry;	// g_AllNodes, key is NodeGuid
	cl_map_item_t	SystemNodesEntry;	// SystemData.Nodes, key is NodeGuid
	LIST_ITEM		AllTypesEntry;	// g_AllFIs, g_AllSWs
	struct SystemData_s *systemp;	// parent system
	STL_NODE_INFO		NodeInfo; 		// port specific fields are 0
	STL_NODE_DESCRIPTION NodeDesc;
	cl_qmap_t Ports;				// items are PortData, key is PortNum
#if !defined(VXWORKS) || defined(BUILD_DMC)
	IouData			*ioup;			// optional IOU
#endif
	STL_SWITCHINFO_RECORD *pSwitchInfo;	// optional SwitchInfo
									// also holds LID for accessing switch
									// for devices without vendor specific
									// capabilities, extra fields and capability
									// mask is zeroed
	SwitchData		*switchp;		// optional Switch specific data
	struct ExpectedNode_s *enodep;	// if supplied in topology input
	void *context;					// application specific field
	uint8	PmaAvoid:1;				// node PMA has instability
	uint8	PmaAvoidClassPortInfo:1;	// node has instability in ClassPortInfo
	uint8	PmaValidateRedirectQP:1;	// validate QP in response
	uint8	analysis:5;					// for TabulateRoutes, tier in fabric

	STL_CONGESTION_INFO CongestionInfo;
	/* CCA CongestionSetting */
	union {
		STL_SWITCH_CONGESTION_SETTING Switch;
		STL_HFI_CONGESTION_SETTING Hfi;
	} CongestionSetting;
	union {
		STL_SWITCH_CONGESTION_LOG Switch;
		STL_HFI_CONGESTION_LOG Hfi;
	} CongestionLog;

	uint8_t coreSwitch;  // (fabric_sim) fat tree non-edge switch
	uint8_t visited; // (opasnapconfig) has this node been visited
	uint8_t valid; // (opasnapconfig) whether node exists in proper location from snapshot
	uint8_t path[64]; // (opasnapconfig) path we traversed to get to this node
} NodeData;


typedef uint8 MCROUTESTATUS;
#define MAXMCROUTESTATUS	5	// 5 different status for MC routes
#define MC_NO_TRACE 	0x00
#define MC_NOT_FOUND	0x01
#define MC_UNAVAILABLE	0x02
#define MC_LOOP 		0x03
#define MC_NOGROUP		0x04

typedef struct McGroupData_s {		// entry to AllMcGroups
									// holds info of McGroup Members as well as switches that 
									// belong to the group
	LIST_ITEM	AllMcGMembersEntry;
	QUICK_LIST	AllMcGroupMembers;	//datatype: McMemberData
	int     	NumOfMembers;
	STL_LID		MLID;
	IB_GID		MGID;
	IB_MCMEMBER_RECORD	GroupInfo;	//Fields corresponding to individual members should not be used
	QUICK_LIST	EdgeSwitchesInGroup;//datatype:McHFISwitchData
} McGroupData;

typedef struct McEdgeSwitchData_s {	// entry in McGroupData
	LIST_ITEM		McEdgeSwitchEntry;
	uint8   		EntryPort;
	PortData		*pPort;
	uint64  		NodeGUID;
} McEdgeSwitchData;

typedef struct McMemberData_s { // entry in McGroupData
	LIST_ITEM		McMembersEntry;

	// Do not use 16-bit MemberInfo.MLID; use 32-bit MLID of containing McGroupData
	IB_MCMEMBER_RECORD	MemberInfo;

	PortData		*pPort;	
} McMemberData;

typedef struct McNodeLoopInc_s {
	LIST_ITEM	McNodeEntry;
	PortData	*pPort;
	uint16  	entryPort;
	uint16  	exitPort;
} McNodeLoopInc;

typedef struct McLoopInc_s {
	QUICK_LIST	AllMcNodeLoopIncR;	//datatype: McNodeLoopInc
	LIST_ITEM	LoopIncEntry;
	MCROUTESTATUS	status;
	STL_LID  	mlid;
} McLoopInc;

typedef struct McRouteStatus_s {
	QUICK_LIST	AllMcRouteStatus;
	MCROUTESTATUS	status;
} McRouteStatus;

typedef struct clConnPathData_s {
   IB_PATH_RECORD    path; 
} clConnPathData_t; 

typedef struct clConnData_s {
   LIST_ITEM        AllConnectionEntry; 
   uint64           FromDeviceGUID;      // GUID of the HFI, TFI, switch
   uint8            FromPortNum; 
   uint64           ToDeviceGUID;        // GUID of the HFI, TFI, switch
   uint8            ToPortNum; 
   uint32           Rate;                // active rate for this port
   STL_VL           VL;
   clConnPathData_t PathInfo;
} clConnData_t; 

#define CREDIT_LOOP_DEVICE_MAX_CONNECTIONS      66
#define DIJKSTRA_INFINITY                       0xffff

typedef struct clDeviceData_s {
   cl_map_item_t   AllDevicesEntry;          // key is NodeGuid
   LIST_ITEM       AllDeviceTypesEntry; 
   STL_LID         Lid;
   NodeData        *nodep; 
   clConnData_t    *Connections[CREDIT_LOOP_DEVICE_MAX_CONNECTIONS][STL_MAX_SCS];       // 36 port switch + 1 for port zero
   cl_qmap_t       map_dlid_to_route;
} clDeviceData_t; 

typedef struct clRouteData_s {
   cl_map_item_t   AllRoutesEntry; // key is DLID
   PortData        *portp;
} clRouteData_t; 

typedef struct clVertixData_s {
   cl_map_item_t   AllVerticesEntry; // key is connection memory address
   clConnData_t    *Connection; 
   uint32          Id; 
   uint32          RefCount; 
   uint32          OutboundCount; 
   uint32          OutboundInuseCount; 
   uint32          OutboundLength; 
   int             *Outbound; 
   uint32          InboundCount; 
   uint32          InboundInuseCount; 
   uint32          InboundLength; 
   int             *Inbound;
} clVertixData_t; 

typedef struct clArcData_s {
   cl_map_item_t   AllArcsEntry; // key is combination of source & sink ids
   cl_map_item_t   AllArcIdsEntry; // key is arc list id
   uint32          Id; 
   uint32          Source; 
   uint32          Sink; 
   union {
      uint64	AsReg64; 
      
      struct {
         uint32	Source;
         uint32	Sink;
      } s;
   } u1;
} clArcData_t; 

typedef struct clGraphData_s {
   uint32          NumVertices; 
   uint32          NumActiveVertices; 
   uint32          VerticesLength; 
   clVertixData_t  **Vertices; 
   uint32          NumArcs; 
   //QUICK_LIST      Arcs; 
   cl_qmap_t       Arcs; 
   cl_qmap_t       map_arc_key_to_arc;
   QUICK_LIST      map_conn_to_vertex;
   cl_qmap_t       map_conn_to_vertex_conn;
} clGraphData_t; 

typedef struct clVertixDataDistance_s {
   cl_map_item_t   AllVerticesEntry;          // key is Vertix Id
   clVertixData_t  *vertixp;
} clVertixDataDistance_t; 

typedef struct clDijkstraDistancesAndRoutes_s {
    uint32          **distances;
    uint32          **routes;
    uint32          nRows;
    uint32          nCols;
} clDijkstraDistancesAndRoutes_t;

/**
 * Get start of an entry in MulticastFDB. Should use bits MLID[13:0].
 * @c pos is the portmask position; each position is 64 bits (ports) wide.
 */
static inline
STL_PORTMASK *GetMulticastFDBEntry(NodeData *nodep, uint32 entry, uint8 pos)
{
	if (! nodep->switchp || !nodep->switchp->MulticastFDB[pos])
		return NULL;

	return (&nodep->switchp->MulticastFDB[pos][entry]);
}

// information about an IB enabled system in the fabric
// Each system is a set of nodes with the same SystemImageGUID
// Some 3rd party devices report a SystemImageGUID of 0, in which case
// the node GUID (which should still be unique among systems with
// SystemImageGUIDs of 0) is used as the key for g_AllSystems
typedef struct SystemData_s {
	cl_map_item_t	AllSystemsEntry;	// g_AllSystems, key is SystemImageGUID
	EUI64	SystemImageGUID;
	cl_qmap_t Nodes;					// items are NodeData, key is NodeGuid
	void *context;					// application specific field
} SystemData;

typedef struct SMData_s {
	cl_map_item_t	AllSMsEntry;			// g_AllSMs, key is PortGUID in SMInfo
	STL_SMINFO_RECORD	SMInfoRecord; // also holds LID for accessing SM
	struct PortData_s *portp;	// port SM is running on
	ExpectedSM *esmp; 			// if supplied in topology input
	void *context;					// application specific field
} SMData;

typedef struct MasterSMData_s {
	uint64	serviceID;			// Service ID of QLogic master SM (else 0)
	uint32	capabilityMask;		// Capability mask of QLogic master SM
	uint8	version;			// SM Version
} MasterSMData_t;

typedef struct {
	LIST_ITEM AllVFsEntry;
	STL_VFINFO_RECORD record;
} VFData_t;

typedef enum {
	FF_NONE				=0,
	FF_STATS			=0x000000001,	// PortCounters fetched
	// flags for topology_input which was provided
	FF_EXPECTED_NODES	=0x000000002,	// topology_input w/<Nodes>
	FF_EXPECTED_LINKS	=0x000000004,	// topology_input w/<LinkSummary>
	FF_EXPECTED_EXTLINKS=0x000000008,	// topology_input w/<ExtLinkSummary>
	FF_CABLEDATA		=0x000000010,	// topology_input w/ some <CableData>
	FF_LIDARRAY			=0x000000020,	// Keep AllLids as array instead of qmap
										// allows faster FindLid, but takes
										// approx 300K more memory
	FF_PMADIRECT		=0x000000080,	// Force direct PMA access of port counters
	FF_ROUTES			=0x000000100,	// Routing tables (linear/mcast) fetched
	FF_QOSDATA			=0x000000200,	// QOS data fetched
	FF_SMADIRECT		=0x000000400,	// Force direct SMA access
	FF_BUFCTRLTABLE		=0x000000800,	// BufferControlData collected
	FF_DOWNPORTINFO		=0x000001000,	// Get PortInfo for Down switch ports
	FF_CABLELOWPAGE		=0x000004000,	//Get Lower memory of Cable Info
} FabricFlags_t;

// Handling for LIDs up to 24 bits
#define TOPLM_BLOCK_BITS 10
#define TOPLM_ENTRY_BITS 14
#define TOPLM_BLOCKS (1 << TOPLM_BLOCK_BITS)  /* There are 2^10 blocks in the map */
#define TOPLM_ENTRIES (1 << TOPLM_ENTRY_BITS) /* Each block contains 2^14 pointers to PortData */
#define TOPLM_BLOCK_NUM(lid)	((lid) >> TOPLM_ENTRY_BITS)
#define TOPLM_ENTRY_NUM(lid)	((lid) & (TOPLM_ENTRIES -1))
#define TOPLM_LID_MAX ((1 << (TOPLM_BLOCK_BITS + TOPLM_ENTRY_BITS)) - 1) // Map can handle 2^24 lids

typedef struct TopLidMap_s {
	PortData **LidBlocks[TOPLM_BLOCKS];
} TopLidMap_t;

typedef struct FabricData_s {
	time_t	time;			// when fabric data was obtained from a real fabric
	FabricFlags_t	flags;	// what data is available in FabricData

	// data from live fabric or snapshot
	cl_qmap_t AllNodes;		// items are NodeData, key is node guid
	union {
		cl_qmap_t AllLids;		// items are PortData, key is LID
		TopLidMap_t LidMap;
	} u;
	uint32 lidCount;
	cl_qmap_t AllSystems;	// items are SystemData, key is system image guid
	QUICK_LIST AllPorts;	// sorted by NodeGUID+PortNum
	QUICK_LIST AllFIs;		// sorted by NodeGUID
	QUICK_LIST AllSWs;		// sorted by NodeGUID
	QUICK_LIST AllVFs;		// list of VFData_t
#if !defined(VXWORKS) || defined(BUILD_DMC)
	QUICK_LIST AllIOUs;		// sorted by NodeGUID
	// AllIOCs uses IOCGUID as the primary key and NodeGUID as secodary key
	cl_qmap_t AllIOCs;		// items are IocData
#endif
	cl_qmap_t AllSMs;		// items are SMData, key is PortGuid

	//	Multicast  related structures
	QUICK_LIST	AllMcGroups;	// items are MCGroups, Key is MGID
	McRouteStatus	AllMcLoopIncRoutes[MAXMCROUTESTATUS];	// items are list of loop and incomplete MC routes.
	uint32		NumOfMcGroups;

	MasterSMData_t	MasterSMData;	// Master SM data
	uint32 LinkCount;		// number of links in fabric
	uint32 ExtLinkCount;	// number of external links in fabric
	uint32 FILinkCount;		// number of FI links in fabric
	uint32 ISLinkCount;		// number of inter-switch links in fabric
	uint32 ExtISLinkCount;	// number of external inter-switch links in fabric

	// additional information for credit loop
	uint32 ConnectionCount;                         
	uint32 RouteCount;                         
	QUICK_LIST FIs;
	QUICK_LIST Switches;
	cl_qmap_t map_guid_to_ib_device;	// items are devices, ky is node guid
	clGraphData_t Graph;

	// additional information from topology input file
	QUICK_LIST ExpectedLinks;	// in order read from topology input file
	QUICK_LIST ExpectedFIs;		// in order read from topology input file
	QUICK_LIST ExpectedSWs;		// in order read from topology input file
	QUICK_LIST ExpectedSMs;		// in order read from topology input file
	//topology input data optimized for search
	cl_qmap_t  ExpectedNodeGuidMap; //all expected FIs/SWs mapped by NodeGuid

	void *context;				// application specific field
	int ms_timeout;
} FabricData_t;

// these callbacks are called when an object with a non-null application
// specific context field is freed.  They should free the object pointed to by
// the context field and remove any other application references to the object
// being freed
typedef void (PortDataFreeCallback)(FabricData_t *fabricp, PortData *portp);
#if !defined(VXWORKS) || defined(BUILD_DMC)
typedef void (IocDataFreeCallback)(FabricData_t *fabricp, IocData *iocp);
typedef void (IouDataFreeCallback)(FabricData_t *fabricp, IouData *ioup);
#endif
typedef void (NodeDataFreeCallback)(FabricData_t *fabricp, NodeData *nodep);
typedef void (SystemDataFreeCallback)(FabricData_t *fabricp, SystemData *systemp);
typedef void (SMDataFreeCallback)(FabricData_t *fabricp, SMData *smp);
typedef void (FabricDataFreeCallback)(FabricData_t *fabricp);

typedef struct Top_FreeCallbacks_s {
	PortDataFreeCallback	*pPortDataFreeCallback;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	IocDataFreeCallback	*pIocDataFreeCallback;
	IouDataFreeCallback	*pIouDataFreeCallback;
#endif
	NodeDataFreeCallback	*pNodeDataFreeCallback;
	SystemDataFreeCallback	*pSystemDataFreeCallback;
	SMDataFreeCallback	*pSMDataFreeCallback;
	FabricDataFreeCallback	*pFabricDataFreeCallback;
} Top_FreeCallbacks;

// For functions which generate Points, is it node pair or just node
#define PAIR_FLAG_NONE		0x01	/* no pair exists */
#define PAIR_FLAG_NODE		0x02	/* pair exists */

// Identifies side of pair
#define LSIDE_PAIR		0x01	/* left side of pair */
#define RSIDE_PAIR		0x02	/* right side of pair */

typedef struct NodePairList_s {
	DLIST			nodePairList1;     //members of left side of pair
	DLIST			nodePairList2;     //members of right side of pair
} NodePairList_t;

/* struct Point_s identifies a particular point in the fabric and
 * topology.xml.
 * Used for trace route and other "focused" reports
 * Presently coded in C, but a good candidate for a C++ abstract class
 * with a subclass for each point type
 */
typedef enum {
	POINT_TYPE_NONE,
	POINT_TYPE_PORT,
	POINT_TYPE_PORT_LIST,
	POINT_TYPE_NODE,
	POINT_TYPE_NODE_LIST,
#if !defined(VXWORKS) || defined(BUILD_DMC)
	POINT_TYPE_IOC,
	POINT_TYPE_IOC_LIST,
#endif
	POINT_TYPE_SYSTEM,
	POINT_TYPE_NODE_PAIR_LIST,
} PointType;

typedef enum {
	POINT_ENODE_TYPE_NONE,
	POINT_ENODE_TYPE_NODE,
	POINT_ENODE_TYPE_NODE_LIST,
} PointEnodeType;

typedef enum {
	POINT_ESM_TYPE_NONE,
	POINT_ESM_TYPE_SM,
	POINT_ESM_TYPE_SM_LIST,
} PointEsmType;

typedef enum {
	POINT_ELINK_TYPE_NONE,
	POINT_ELINK_TYPE_LINK,
	POINT_ELINK_TYPE_LINK_LIST,
} PointElinkType;


typedef struct Point_s {

	/* object(s) matched in the fabric */
	PointType	Type;	/* if POINT_TYPE_NONE, u undefined */
	boolean		haveSW;	/* indicates the point has SW*/
	boolean		haveFI;	/* indicates the point has FI*/
	union {
		PortData	*portp;
		NodeData	*nodep;
#if !defined(VXWORKS) || defined(BUILD_DMC)
		IocData		*iocp;
#endif
		SystemData	*systemp;
		DLIST		nodeList;
		DLIST		portList;
		DLIST		iocList;
		NodePairList_t		nodePairList;
	} u;

	/* ExpectedNode(s) matched in topology file */
	PointEnodeType	EnodeType;	/* if POINT_ENODE_TYPE_NONE, u2 undefined */
	union {
		ExpectedNode	*enodep;
		DLIST			enodeList;
	} u2;

	/* ExpectedSM(s) matched in topology file */
	PointEsmType	EsmType;	/* if POINT_ESM_TYPE_NONE, u3 undefined */
	union {
		ExpectedSM	*esmp;
		DLIST		esmList;
	} u3;

	/* ExpectedLink(s) matched in topology file */
	PointElinkType	ElinkType;	/* if POINT_ELINK_TYPE_NONE, u4 undefined */
	union {
		ExpectedLink	*elinkp;
		DLIST			elinkList;
	} u4;
} Point;

#if !defined(VXWORKS) || defined(BUILD_DMC)
/* IOC types we can focus on */
typedef enum {
	IOC_TYPE_SRP,
	IOC_TYPE_OTHER
} IocType;
#endif

// used for output of snapshot
typedef struct {
	FabricData_t *fabricp;	// fabric to dump to snapshot file
	int argc;				// args to program ran
	char **argv;			// args to program ran
	// Point *focus;
} SnapshotOutputInfo_t;

// used for output of ValidateAllCreditLoopRoutes
typedef struct ValidateCreditLoopRoutesContext_s {
	uint8 format;
	int indent;
	int detail;
	int quiet;
} ValidateCreditLoopRoutesContext_t;

typedef enum {
	QUAL_EQ,
	QUAL_GE,
	QUAL_LE
} LinkQualityCompare;

// simple way to convert time_t to a localtime date string in dest
// (from Topology/getdate.c)
extern void Top_formattime(char *dest, size_t max, time_t t);

// FabricData_t handling (Topology/fabricdata.c)
extern FSTATUS InitFabricData( FabricData_t *pFabric, FabricFlags_t flags);
extern PortSelector* GetPortSelector(PortData *portp);
// Determine if portp and its neighbor are an internal link within a single
// system.  Note that some 3rd party products report SystemImageGuid of 0 in
// which case we can only consider links between the same node as internal links
extern boolean isInternalLink(PortData *portp);
// Determine if portp and its neighbor are an Inter-switch Link
extern boolean isISLink(PortData *portp);
// Determine if portp or its neighbor is an FI
extern boolean isFILink(PortData *portp);
// Determine if elinkp is an external link within a single system.
// To make sure verification is thorough, this will return true when
// either the expected or resolved link(s) are external or when the
// expected link is not specific as to whether it is internal or external.
extern boolean isExternalExpectedLink(ExpectedLink *elinkp);
// Determine if elinkp is an inter-switch link
// To make sure verification is thorough, this will return true when
// either the expected or resolved link(s) are SW-SW
extern boolean isISExpectedLink(ExpectedLink *elinkp);
// Determine if elinkp is an FI link
// To make sure verification is thorough, this will return true when
// either the expected or resolved link(s) include an FI
extern boolean isFIExpectedLink(ExpectedLink *elinkp);
extern void PortDataFreePartitionTable(FabricData_t *fabricp, PortData *portp);
extern FSTATUS PortDataAllocatePartitionTable(FabricData_t *fabricp, PortData *portp);
// capacity of Partition Table for the given port
extern uint16 PortPartitionTableSize(PortData *portp);
// Lookup PKey
// ignores the Full/Limited bit, only checks low 15 bits for a match
// returns index of pkey in overall table or -1 if not found
// if the Partition Table for the port is not available, returns -1
extern int FindPKey(PortData *portp, uint16 pkey);
// Determine if the given port is a member of the given vFabric
// We don't really have all the right data here, especially if the FM has
// combined multiple vFabrics into the same SL and PKey
// but for most cases, we can safely conclude that if the port has the SL and
// PKey configured it is a member of the vFabric.
// This function will return FALSE if QoS data or SL2SCMap is not available
// or if the port is not Armed/Active.
// Given the current FM implementation (and the dependency on SL2SCMap), this
// routine will return FALSE if invoked for non-endpoints
extern boolean isVFMember(PortData *portp, VFData_t *pVFData);
// count the number of armed/active links in the node
extern uint32 CountInitializedPorts(FabricData_t *fabricp, NodeData *nodep);
extern FSTATUS NodeDataAllocateSwitchData(FabricData_t *fabricp, NodeData *nodep,
				uint32 LinearFDBSize, uint32 MulticastFDBSize);
extern FSTATUS NodeDataAllocateFDB(FabricData_t *fabricp, NodeData *nodep,
                uint32 LinearFDBSize);
extern FSTATUS SwitchDataAllocateQOS(SwitchData *sw);

/**
	Adjust amount of memory allocated for Linear and Multicast FDB tables.

	Copies existing values into new memory, adjusts FDB Top values, and frees existing memory.  When resizing MulticastFDB, the new entry size (entries per LID) must be the same as the old size.

	If the multicast FDB table is to be resized and there is already a multicast FDB table and the existing MulticastFDBTop is less than LID_MCAST_START then no values will be copied from the old table to the new table, though the table will still be resized.
	In this case, the value of @c nodep->switchp->MulticastFDBSize will be LID_MCAST_START + @c newMfdbSize rather than MulticastFDBTop + 1.

	@param newLfdbSize If 0, don't change.  Otherwise, realloc to min(newLfdbSize, LinearFDBCap).
	@param newMfdbSize If 0, don't change.  Otherwise, realloc to min(newMfdbSize, MulticastFDBCap).

	@return FSUCCESS on success, non-FSUCCESS on failure.  Existing memory is not destroyed until after new tables are successfully allocated and copied.
*/
FSTATUS NodeDataSwitchResizeFDB(NodeData * nodep, uint32 newLfdbSize, uint32 newMfdbSize);
FSTATUS NodeDataSwitchResizeLinearFDB(NodeData * nodep, uint32 newLfdbSize);
FSTATUS NodeDataSwitchResizeMcastFDB(NodeData * nodep, uint32 newMfdbSize);

extern FSTATUS PortDataAllocateAllQOSData(FabricData_t *fabricp);
extern FSTATUS PortDataAllocateAllBufCtrlTable(FabricData_t *fabricp);
extern FSTATUS PortDataAllocateAllPartitionTable(FabricData_t *fabricp);
extern FSTATUS PortDataAllocateAllCableInfo(FabricData_t *fabricp);
extern FSTATUS PortDataAllocateAllCongestionControlTableEntries(FabricData_t *fabricp);
extern FSTATUS PortDataAllocateAllGuidTable(FabricData_t *fabricp);


/// @param fabricp optional, can be NULL
extern void PortDataFreeQOSData(FabricData_t *fabricp, PortData *portp);
/// @param fabricp optional, can be NULL
extern FSTATUS PortDataAllocateQOSData(FabricData_t *fabricp, PortData *portp);

/// @param fabricp optional, can be NULL
extern void PortDataFreeBufCtrlTable(FabricData_t *fabricp, PortData *portp);
/// @param fabricp optional, can be NULL
extern FSTATUS PortDataAllocateBufCtrlTable(FabricData_t *fabricp, PortData *portp);

/// @param fabricp optional, can be NULL
extern void PortDataFreeCableInfoData(FabricData_t *fabricp, PortData *portp);
/// @param fabricp optional, can be NULL
extern FSTATUS PortDataAllocateCableInfoData(FabricData_t *fabricp, PortData *portp);

/// @param fabricp optional, can be NULL
extern void PortDataFreeCongestionControlTableEntries(FabricData_t *fabricp, PortData *portp);
/// @param fabricp optional, can be NULL
extern FSTATUS PortDataAllocateCongestionControlTableEntries(FabricData_t *fabricp, PortData *portp);


// these routines can be used to manually build FabricData.  Use as follows:
// for each node
// 		FabricDataAddNode (can be any state, including down)
// 		if switch node
// 			NodeDataSetSwitchInfo
// 		for each port of Node
// 			NodeDataAddPort
// 	BuildFabricDataLists
// 	for each physical link
// 		FabricDataAddLink or FabricDataAllLinkRecord
// 		(FabricDataAddLink may be called before BuildFabricDataLists if desired)
// if desired, after building the basic topology, can build empty SMA tables:
//		NodeDataAllocateAllSwitchData
//		PortDataAllocateAllQOSData
//		PortDataAllocateAllBufCtrlTable
//		PortDataAllocateAllPartitionTable
//		PortDataAllocateAllGuidTable
//	when process Set(PortInfo)
//		validate PortInfo given
//		fill in empty/noop/readonly fields with previous PortInfo
//		PortDataSetPortInfo
extern PortData* NodeDataAddPort(FabricData_t *fabricp, NodeData *nodep, EUI64 guid, STL_PORTINFO_RECORD *pPortInfo);
extern FSTATUS NodeDataSetSwitchInfo(NodeData *nodep, STL_SWITCHINFO_RECORD *pSwitchInfo);
extern NodeData *FabricDataAddNode(FabricData_t *fabricp, STL_NODE_RECORD *pNodeRecord, boolean *new_nodep);

extern FSTATUS AddEdgeSwitchToGroup(FabricData_t *fabricp, McGroupData *mcgroupp, NodeData *groupswitch, uint8 SWentryport);
extern McGroupData *FabricDataAddMCGroup(FabricData_t *fabricp, struct omgt_port *port, int quiet, IB_MCMEMBER_RECORD *pMCGRecord,
		boolean *new_nodep, FILE *verbose_file);

extern FSTATUS GetAllMCGroups(EUI64 portGuid, FabricData_t *fabricp, Point *focus, int quiet);
extern FSTATUS GetAllMCGroupMember(FabricData_t *fabricp, McGroupData *mcgroupp,struct omgt_port *portp,int quiet, FILE *g_verbose_file);

extern boolean SupportsVendorPortCounters(NodeData *nodep,
                                        IB_CLASS_PORT_INFO *classPortInfop);
extern boolean SupportsVendorPortCounters2(NodeData *nodep,
                                        IB_CLASS_PORT_INFO *classPortInfop);

/* build the Fabric.AllPorts, ALLFIs, and AllSWs lists such that
 * AllPorts is sorted by NodeGUID, PortNum
 * AllFIs, ALLSWs, AllIOUs is sorted by NodeGUID
 */
extern void BuildFabricDataLists(FabricData_t *fabricp);

extern FSTATUS FabricDataAddLink(FabricData_t *fabricp, PortData *p1, PortData *p2);
extern FSTATUS FabricDataAddLinkRecord(FabricData_t *fabricp, STL_LINK_RECORD *pLinkRecord);
extern FSTATUS FabricDataRemoveLink(FabricData_t *fabricp, PortData *p1);

extern void QOSDataAddSCSCMap(PortData *portp, uint8_t outport, int extended, const STL_SCSCMAP *pSCSC);
extern STL_SCSCMAP * QOSDataLookupSCSCMap(PortData *portp, uint8_t outport, int extended);

// Set update lookup tables due to PortState, LID or LMC change for port.
// Typical use is when a SetPortInfo on neighbor causes a link state change
// For use by fabric simulator.
extern void PortDataStateChanged(FabricData_t *fabricp, PortData *portp);
// Set new Port Info based on a Set(PortInfo).  For use by fabric simulator
// assumes pInfo already validated and any Noop fields filled in with correct
// values.
extern void PortDataSetPortInfo(FabricData_t *fabricp, PortData *portp, STL_PORT_INFO *pInfo);

extern void DestroyFabricData(FabricData_t *fabricp);

extern NodeData * CLDataAddDevice(FabricData_t *fabricp, NodeData *nodep, STL_LID lid, int verbose, int quiet);
extern FSTATUS CLDataAddConnection(FabricData_t *fabricp, PortData *portp1, PortData *portp2, clConnPathData_t *pathInfo, uint8 sc, int verbose, int quiet);
extern FSTATUS CLDataAddRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid, PortData *sportp, int verbose, int quiet);
extern clGraphData_t * CLGraphDataSplit(clGraphData_t *graphp, int verbose);
extern FSTATUS CLFabricDataDestroy(FabricData_t *fabricp, void *context);
extern void CLGraphDataFree(clGraphData_t *graphp, void *context);
extern FSTATUS CLDijkstraFindDistancesAndRoutes(clGraphData_t *graphp, clDijkstraDistancesAndRoutes_t *respData, int verbose);
extern void CLDijkstraFreeDistancesAndRoutes(clDijkstraDistancesAndRoutes_t *drp);

// search routines (Topology/search.c)
// For functions which generate Points, indicate what is needed
#define FIND_FLAG_FABRIC	0x01	/* find in live fabric */
#define FIND_FLAG_ENODE		0x02	/* find in ExpectedNodes */
#define FIND_FLAG_ESM		0x04	/* find in ExpectedSMs */
#define FIND_FLAG_ELINK		0x08	/* find in ExpectedLinks */

// The search routines involving Points tend to limit the search to the
// specific objects indicated by find_flag and expect later caller use of
// Compare functions to invoke the necessary Compare functions to cover the
// fabric vs expected objects of interest.  One exception is the comparison
// for Details and Cable information which is only available in topology.xml
// In this case, the fabric seach will check the resolved expected object
// This permits the majority of fabric related operations, such as opareport,
// to make use of Details and Cable information from topology.xml while
// only considering fabric objects.

// search for the PortData corresponding to the given node and port number
extern PortData * FindNodePort(NodeData *nodep, uint8 port);
// search for the PortData corresponding to the given lid
extern PortData * FindLid(FabricData_t* fabricp, STL_LID lid);
// search for the PortData corresponding to the given port Guid
extern PortData * FindPortGuid(FabricData_t* fabricp, EUI64 guid);
extern FSTATUS FindPortGuidPoint(FabricData_t *fabricp, EUI64 guid, Point *pPoint, uint8 find_flag, int silent);
extern FSTATUS FindGidPoint(FabricData_t *fabricp, IB_GID gid, Point *pPoint, uint8 find_flag, int silent);
// search for the NodeData corresponding to the given node Guid
extern NodeData * FindNodeGuid(const FabricData_t* fabricp, EUI64 guid);
extern FSTATUS FindNodeGuidPoint(FabricData_t *fabricp, EUI64 guid, Point *pPoint, uint8 find_flag, int silent);
extern FSTATUS FindNodeNamePoint(FabricData_t* fabricp, char *name, Point *pPoint, uint8 find_flag, int silent);
extern FSTATUS FindNodeNamePatPoint(FabricData_t* fabricp, char *pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindNodeNamePatPointUncompress(FabricData_t *fabricp, char *pattern, Point *pPoint, uint8 find_flag);

extern FSTATUS FindNodePatPairs(FabricData_t *fabricp, char *pattern, NodePairList_t *nodePatPairs, uint8 find_flag, uint8 side);
extern FSTATUS FindNodeDetailsPatPoint(FabricData_t* fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindNodeTypePoint(FabricData_t* fabricp, NODE_TYPE type, Point *pPoint, uint8 find_flag);
#if !defined(VXWORKS) || defined(BUILD_DMC)
extern FSTATUS FindIocNamePoint(FabricData_t* fabricp, char *name, Point *pPoint, uint8 find_flag);
extern FSTATUS FindIocNamePatPoint(FabricData_t* fabricp, char *pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindIocTypePoint(FabricData_t* fabricp, IocType type, Point *pPoint, uint8 find_flag);
extern FSTATUS FindIocGuid(FabricData_t* fabricp, EUI64 guid, Point *pPoint);
#endif
extern SystemData * FindSystemGuid(FabricData_t* fabricp, EUI64 guid);
extern FSTATUS FindRatePoint(FabricData_t* fabricp, uint32 rate, Point *pPoint, uint8 find_flag);
extern FSTATUS FindPortStatePoint(FabricData_t* fabricp, uint8 state, Point *pPoint, uint8 find_flag);
extern FSTATUS FindLedStatePoint(FabricData_t* fabricp, uint8 state, Point *pPoint, uint8 find_flag);
extern FSTATUS FindPortPhysStatePoint(FabricData_t* fabricp, IB_PORT_PHYS_STATE state, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCableLabelPatPoint(FabricData_t* fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCableLenPatPoint(FabricData_t* fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCableDetailsPatPoint(FabricData_t* fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCabinfLenPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCabinfVendNamePatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCabinfVendPNPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCabinfVendRevPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCabinfVendSNPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindCabinfCableTypePoint(FabricData_t *fabricp, char *cablearg, Point *pPoint, uint8 find_flag);
extern FSTATUS FindLinkDetailsPatPoint(FabricData_t* fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindPortDetailsPatPoint(FabricData_t* fabricp, const char* pattern, Point *pPoint, uint8 find_flag);
extern FSTATUS FindMtuPoint(FabricData_t* fabricp, IB_MTU mtu, Point *pPoint, uint8 find_flag);
extern PortData * FindMasterSm(FabricData_t* fabricp);
extern FSTATUS FindSmDetailsPatPoint(FabricData_t *fabricp,const char* pattern, Point *pPoint, uint8 find_flag);
// search for the SMData corresponding to the given PortGuid
extern SMData * FindSMPort(FabricData_t *fabricp,EUI64 PortGUID);
// search for the PortData corresponding to the given lid and port number
// For FIs lid completely defines the port
// For Switches, lid will identify the switch and port is used to select port
extern PortData * FindLidPort(FabricData_t *fabricp, STL_LID lid, uint8 port);
extern PortData * FindNodeGuidPort(FabricData_t *fabricp,EUI64 nodeguid, uint8 port);
extern ExpectedNode* FindExpectedNodeByNodeGuid(const FabricData_t* fabricp, EUI64 nodeGuid);
extern ExpectedNode* FindExpectedNodeByNodeDesc(const FabricData_t* fabricp, const char* nodeDesc, uint8 NodeType);
extern ExpectedLink* FindExpectedLinkByOneSide(const FabricData_t* fabricp, EUI64 nodeGuid, uint8 portNum, uint8* side);
extern FSTATUS FindLinkQualityPoint(FabricData_t *fabricp, uint16 quality, LinkQualityCompare comp, Point *pPoint, uint8 find_flag);
extern FSTATUS FindExpectedSMByPortGuid(FabricData_t *fabricp, EUI64 portGuid);
extern FSTATUS FindExpectedSMByNodeGuid(FabricData_t *fabricp, EUI64 nodeGuid);

extern void setTopologyMadVerboseFile(FILE* verbose_file);
extern void setTopologyMadRetryCount(int retries);
// mad queries to SMA (from Topology/mad.c)
extern FSTATUS SmaGetNodeDesc(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_NODE_DESCRIPTION *pNodeDesc);
extern FSTATUS SmaGetNodeInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_NODE_INFO *pNodeInfo);
extern FSTATUS SmaGetSwitchInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_SWITCH_INFO *pSwitchInfo);
extern FSTATUS SmaGetPortInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, uint8_t smConfigStarted, STL_PORT_INFO *pPortInfo);
extern FSTATUS SmaGetPartTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, uint16_t block, STL_PARTITION_TABLE *pPartTable);
extern FSTATUS SmaGetVLArbTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, uint8_t part, STL_VLARB_TABLE *pVLArbTable);
extern FSTATUS SmaGetSLSCMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_SLSCMAP *pSLSCMap);
extern FSTATUS SmaGetSCSLMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_SCSLMAP *pSCSLMap);
extern FSTATUS SmaGetSCSCMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t in_port, uint8_t out_port, STL_SCSCMAP *pSCSCMap);
extern FSTATUS SmaGetSCVLMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t port_num, STL_SCVLMAP *pSCVLMap, uint16_t attr);
extern FSTATUS SmaGetLinearFDBTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint16_t block, STL_LINEAR_FORWARDING_TABLE *pFDB);
extern FSTATUS SmaGetMulticastFDBTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint32_t block, uint8_t position, STL_MULTICAST_FORWARDING_TABLE *pFDB);
extern FSTATUS SmaGetPortGroupFDBTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint16 block, STL_PORT_GROUP_FORWARDING_TABLE *pFDB);
extern FSTATUS SmaGetPortGroupTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint16 block, STL_PORT_GROUP_TABLE *pPGT);

extern FSTATUS SmaGetBufferControlTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t startPort, uint8_t endPort, STL_BUFFER_CONTROL_TABLE pBCT[]);
extern FSTATUS SmaGetCableInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, uint16_t addr, uint8_t len, uint8_t *data);

extern FSTATUS SmaSetSwitchInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_SWITCH_INFO *pSwitchInfo);
extern FSTATUS SmaSetPortInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, STL_PORT_INFO *pPortInfo);
extern FSTATUS SmaSetPartTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, uint16_t block, STL_PARTITION_TABLE *pPartTable);
extern FSTATUS SmaSetVLArbTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t portNum, uint8_t part, STL_VLARB_TABLE *pVLArbTable);
extern FSTATUS SmaSetSLSCMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_SLSCMAP *pSLSCMap);
extern FSTATUS SmaSetSCSLMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_SCSLMAP *pSCSLMap);
extern FSTATUS SmaSetSCSCMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t in_port, uint8_t out_port, STL_SCSCMAP *pSCSCMap);
extern FSTATUS SmaSetSCVLMappingTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, boolean asyncUpdate, boolean allPorts, uint8_t portNum, STL_SCVLMAP *pSCVLMap, uint16_t attr);
extern FSTATUS SmaSetBufferControlTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint8_t startPort, uint8_t endPort, STL_BUFFER_CONTROL_TABLE pBCT[]);
extern FSTATUS SmaSetLinearFDBTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint16_t block, STL_LINEAR_FORWARDING_TABLE *pFDB);
extern FSTATUS SmaSetMulticastFDBTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint32_t block, uint8_t position, STL_MULTICAST_FORWARDING_TABLE *pFDB);

extern FSTATUS SmaGetCongestionInfo(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, STL_CONGESTION_INFO *pCongestionInfo);
extern FSTATUS SmaGetHFICongestionControlTable(struct omgt_port *port, STL_LID dlid, STL_LID slid, uint8_t* path, uint16_t block, uint16_t numBlocks, STL_HFI_CONGESTION_CONTROL_TABLE *pHfiCongestionControl);

// mad queries to PMA and DMA (from Topology/mad.c)
extern FSTATUS InitMad(EUI64 portguid, FILE *verbose_file);
extern void DestroyMad(void);
extern FSTATUS InitSmaMkey(uint64 mkey);
extern boolean NodeHasPma(NodeData *nodep);
extern boolean PortHasPma(PortData *portp);
extern void UpdateNodePmaCapabilities(NodeData *nodep, boolean ProcessHFICounters);
extern FSTATUS STLPmGetClassPortInfo(struct omgt_port *port, PortData *portp);
extern FSTATUS STLPmGetPortStatus(struct omgt_port *port, PortData *portp, uint8 portNum, STL_PORT_STATUS_RSP *pPortStatus);
extern FSTATUS STLPmClearPortCounters(struct omgt_port *port, PortData *portp, uint8 lastPortIndex, uint32 counterselect);
#if !defined(VXWORKS) || defined(BUILD_DMC)
extern FSTATUS DmGetIouInfo(struct omgt_port *port, IB_PATH_RECORD *pathp, IOUnitInfo *pIouInfo);
extern FSTATUS DmGetIocProfile(struct omgt_port *port, IB_PATH_RECORD *pathp, uint8 slot,
						IOC_PROFILE *pIocProfile);
extern FSTATUS DmGetServiceEntries(struct omgt_port *port, IB_PATH_RECORD *pathp, uint8 slot,
							uint8 first, uint8 last, IOC_SERVICE *pIocServices);
#endif

// POINT routines (from Topology/point.c)
extern void PointInit(Point *point);
extern boolean PointIsInInit(Point *point);
extern void PointFabricDestroy(Point *point);
extern void PointEnodeDestroy(Point *point);
extern void PointEsmDestroy(Point *point);
extern void PointElinkDestroy(Point *point);
extern void PointDestroy(Point *point);
extern boolean PointValid(Point *point);
extern FSTATUS PointFabricCopy(Point *dest, Point *src);
extern FSTATUS PointEnodeCopy(Point *dest, Point *src);
extern FSTATUS PointEsmCopy(Point *dest, Point *src);
extern FSTATUS PointElinkCopy(Point *dest, Point *src);
extern FSTATUS PointCopy(Point *dest, Point *src);
extern FSTATUS PointListAppend(Point *point, PointType type, void *object);
extern FSTATUS PointEnodeListAppend(Point *point, PointEnodeType type, void *object);
extern FSTATUS PointEsmListAppend(Point *point, PointEsmType type, void *object);
extern FSTATUS PointElinkListAppend(Point *point, PointElinkType type, void *object);
extern FSTATUS PointNodePairListAppend(Point *point, uint8 side, void *object);
extern FSTATUS PointPopulateNodePairList(Point *pPoint, NodePairList_t *nodePatPairs);
extern void PointFabricCompress(Point *point);
extern void PointEnodeCompress(Point *point);
extern void PointEsmCompress(Point *point);
extern void PointElinkCompress(Point *point);
extern void PointCompress(Point *point);
extern FSTATUS PointListAppendUniquePort(Point *point, PortData *portp);
/* These compare functions will compare the supplied object to the
 * specific relevant portion of the point.  Callers who wish to consider
 * both expected and fabric objects should call all the relevant routines
 * for the fabric and linked expected objects
 * This approach provides greater flexibility for callers, and removes the need
 * for the Point construction and parsing routines to check all the linked
 * fabric and expected objects.
 */

/* compare the supplied port to the given point
 * this is used to identify focus for reports
 * if ! PointValid will report TRUE for all ports
 */
extern boolean ComparePortPoint(PortData *portp, Point *point);
/* compare the supplied node to the given point
 * this is used to identify focus for reports
 * if ! PointValid will report TRUE for all nodes
 */
extern boolean CompareNodePoint(NodeData *nodep, Point *point);
/* compare the supplied ExpectedNode to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ExpectedNodes
 */
extern boolean CompareExpectedNodePoint(ExpectedNode *enodep, Point *point);
/* compare the supplied ExpectedSM to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ExpectedSMs
 */
extern boolean CompareExpectedSMPoint(ExpectedSM *esmp, Point *point);
/* compare the supplied ExpectedLink to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ExpectedLinks
 */
extern boolean CompareExpectedLinkPoint(ExpectedLink *elinkp, Point *point);
#if !defined(VXWORKS) || defined(BUILD_DMC)
extern boolean CompareIouPoint(IouData *ioup, Point *point);
extern boolean CompareIocPoint(IocData *iocp, Point *point);
#endif
/* compare the supplied SM to the given point
 * this is used to identify focus for reports
 * if ! PointValid will report TRUE for all SMs
 */
extern boolean CompareSmPoint(SMData *smp, Point *point);
extern boolean CompareSystemPoint(SystemData *systemp, Point *point);

/* check arg to see if 1st characters match prefix, if so return pointer
 * into arg just after prefix, otherwise return NULL
 */
extern char* ComparePrefix(char *arg, const char *prefix);
extern FSTATUS ParsePoint(FabricData_t *fabricp, char* arg, Point* pPoint, uint8 find_flag, char **pp);

/* check if point is of Type NodePairList */
extern boolean PointTypeIsNodePairList(Point *pPoint);
/* check if point is of Type NodeList */
extern boolean PointIsTypeNodeList(Point *pPoint);
/* check if haveSW flag is set */
extern boolean PointHaveSw(Point *pPoint);
/* check if haveFI flag is set */
extern boolean PointHaveFI(Point *pPoint);

// snapshot input/output routines (from Topology/snapshot.c)
extern void Xml2PrintSnapshot(FILE *file, SnapshotOutputInfo_t *info);

struct IXmlParserState;

/**
	Completion callback manipulators.  These get called inside custom end_func
	definitions and typically do things like insert the completed
	structure into a broader data structure.

	The basic idea is to separate how the entity is parsed from how it is used by the parent.
*/
typedef int (*ParseCompleteFn)(struct IXmlParserState * state, void * object, void * parent);

void SetPortDataComplete(ParseCompleteFn fn);

/**
	only FF_LIDARRAY flag is used, others set based on file read
	@param allocFull When true, adjust allocated memory for linear and multicast forwarding tables to their cap values after reading in all data.
*/
#ifndef __VXWORKS__
extern FSTATUS Xml2ParseSnapshot(const char *input_file, int quiet, FabricData_t *fabricp, FabricFlags_t flags, boolean allocFull);
#else
extern FSTATUS Xml2ParseSnapshot(const char *input_file, int quiet, FabricData_t *fabricp, FabricFlags_t flags, boolean allocFull, XML_Memory_Handling_Suite* memsuite);
#endif
 
// expected topology input/output routines (from Topology/topology.c)

// validation options for Xml2ParseTopology
// None - no attempt to cross reference ExpectedLinks to ExpectedNodes
// 		the ExpectedLink.portselp1 may be NULL
// 		the ExpectedLink.portselp1->enodep will be NULL
// 		the ExpectedLink.portselp2 may be NULL
// 		the ExpectedLink.portselp2->enodep will be NULL
// 		the ExpectedNode.ports[] may be empty
// 		the ExpectedNode.ports[]->elinkp will be NULL
// Loose - do a best attempt to resolve (eg. cross reference) ExpectedLinks
//		to ExpectedNodes filling in the elinkp and enodep pointers mentioned
//		above where possible. Also checks that when Node and Port GUIDs are
//		supplied in ExpectedNodes, that switch port 0's GUIDs are consistent.
//		However no error return reported by	Xml2ParseTopology when unable
//		to resolve some links. 	Does not require ExpectedNodes to
//		explicitly list ports in topology file. Will auto-create
//		ExpectedNode.ports[] entries if a ExpectedLink references the given
//		node's port.
// Somewhat strict - cross references all ExpectedLinks to ExpectedNodes as
//		in Loose.  Also checks that all nodes are connected to the same fabric.
//		Any errors in resolution or checks are reported as an error by
//		Xml2ParseTopology.
// Strict - performs all checks in Somewhat strict.  Also requires that all
//		ExpectedLinks explicitly list PortNum in topology file and
//		all ExpectedNodes explicitly list ports in topology file.
// In all cases its valid for a ExpectedNode to list a port which is not
// connected to the fabric (eg. not referenced by an ExpectedLink).
//
typedef enum {
	TOPOVAL_NONE			=0,
	TOPOVAL_LOOSE			=1,	// resolve what we can, no graph closure checks
	TOPOVAL_SOMEWHAT_STRICT	=2,	// same as strict but don't require <Port> under <Node>
	TOPOVAL_STRICT			=3
} TopoVal_t;
									// in ExpectedNode in topology file
#ifndef __VXWORKS__
extern FSTATUS Xml2ParseTopology(const char *input_file, int quiet, FabricData_t *fabricp, TopoVal_t validation);
#else
extern FSTATUS Xml2ParseTopology(const char *input_file, int quiet, FabricData_t *fabricp, XML_Memory_Handling_Suite* memsuite, TopoVal_t validation);
#endif
extern void Xml2PrintTopology(FILE *file, FabricData_t *fabricp);

// live fabric analysis routines (from Topology/sweep.c)
extern FSTATUS InitSweepVerbose(FILE *verbose_file);
/* flags for Sweep */
typedef enum {
	SWEEP_BASIC					=0,		// Systems, Nodes, Ports, Links
#if !defined(VXWORKS) || defined(BUILD_DMC)
	SWEEP_IOUS			=0x000000001,	// IOU and IOC Info
#endif
	SWEEP_SWITCHINFO	=0x000000002,	// Switch Info
	SWEEP_SM			=0x000000003,	// SM Info
	SWEEP_ALL			=0x000000003
} SweepFlags_t;

extern FSTATUS Sweep(EUI64 portGuid, FabricData_t *fabricp, FabricFlags_t fflags, SweepFlags_t flags, int quiet, int ms_timeout);

//extern FSTATUS GetPathToPort(EUI64 portGuid, PortData *portp, uint16 pkey);
extern FSTATUS GetPaths(struct omgt_port *port, PortData *portp1, PortData *portp2,
						PQUERY_RESULT_VALUES *ppQueryResults);
extern FSTATUS GetTraceRoute(struct omgt_port *port, IB_PATH_RECORD *pathp,
							 PQUERY_RESULT_VALUES *ppQueryResults);
extern FSTATUS GetAllPortCounters(EUI64 portGuid, IB_GID localGid, FabricData_t *fabricp,
			   	Point *focus, boolean limitstats, boolean quiet, uint32 begin, uint32 end);
extern FSTATUS GetAllFDBs( EUI64 portGuid, FabricData_t *fabricp, Point *focus,
				int quiet );
extern FSTATUS GetAllPortVLInfo(EUI64 portGuid, FabricData_t *fabricp, Point *focus, int quieti, int *use_scsc);
extern PQUERY_RESULT_VALUES GetAllQuarantinedNodes(struct omgt_port *port, FabricData_t *fabricp,
													Point *focus, int quiet);
extern FSTATUS GetAllBCTs(EUI64 portGuid, FabricData_t *fabricp, Point *focus, int quiet);

extern FSTATUS ClearAllPortCounters(EUI64 portGuid, IB_GID localGid, FabricData_t *fabricp,
			  	Point *focus, uint32 counterselect, boolean limitstats,
			   	boolean quiet, uint32 *node_countp, uint32 *port_countp,
			   	uint32 *fail_node_countp, uint32 *fail_port_countp);

extern PQUERY_RESULT_VALUES GetAllDeviceGroupMemberRecords(struct omgt_port *port, FabricData_t *fabricp,
													Point *focus, int quiet);
extern void XmlPrintHex64(const char *tag, uint64 value, int indent);
extern FSTATUS ParseFocusPoint(EUI64 portGuid, FabricData_t *fabricp, char* arg, Point* pPoint, uint8 find_flag, char **pp, boolean allow_route);

// utility routines (from Topology/util.c)
extern void Top_setcmdname(const char *name);	// for error messages
extern void Top_setFreeCallbacks(Top_FreeCallbacks *callbacks);
extern void ProgressPrint(boolean newline, const char *format, ...);
extern const char* Top_truncate_str(const char *name);
//#define PROGRESS_PRINT(newline, format, args...) if (! g_quiet) { ProgressPrint(newline, format, ##args); } 


// functions to perform routing analysis using LFT tables in FabricData
//(from Topology/route.c)

// For each device along the path, entry and exit port of device is provided
// For the CA at the start of the route, only an exit port is provided
// For the CA at the end of the route, only a entry port is provided
// For switches along the route, both entry and exit ports are provided
// When a switch Port 0 is the start of the route, it will be the entry port
//    along with the physical exit port
// When a switch Port 0 is the end of the route, it will be the exit port
//    along with the physical entry port
// The above approach parallels how TraceRoute records are reported by the
// SM, so if desired a callback could build a SM TraceRoute style response
// for use in other routines.
typedef FSTATUS (RouteCallback_t)(PortData *entryPortp, PortData *exitPortp, uint8 vl,
			   						void *context);

// returns status of callback (if not FSUCCESS)
// FUNAVAILABLE - no routing tables in FabricData given
// FNOT_FOUND - unable to find starting port
// FNOT_DONE - unable to trace route, dlid is a dead end
extern FSTATUS WalkRoutePort(FabricData_t *fabricp,
			   		PortData *portp, STL_LID dlid, uint8 SL, uint8 rc,
			  		RouteCallback_t *callback, void *context);
// walk by slid to dlid
extern FSTATUS WalkRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid,
			  		RouteCallback_t *callback, void *context);

// caller must free *ppTraceRecords
extern FSTATUS GenTraceRoutePort(FabricData_t *fabricp,
			   	PortData *portp, STL_LID dlid, uint8 rc, 
	   			STL_TRACE_RECORD **ppTraceRecords, uint32 *pNumTraceRecords);
extern FSTATUS GenTraceRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid, uint8 rc, 
	   			STL_TRACE_RECORD **ppTraceRecords, uint32 *pNumTraceRecords);
extern FSTATUS GenTraceRoutePath(FabricData_t *fabricp, IB_PATH_RECORD *pathp, uint8 rc, 
	   			STL_TRACE_RECORD **ppTraceRecords, uint32 *pNumTraceRecords);

// Generate possible Path records from portp1 to portp2
// We don't know SM config, so we just guess and generate paths of the form
// 0-0, 1-1, ....
// This corresponds to the PathSelection=Minimal FM config option
// when LMC doesn't match we start back at base lid for other port
// These may not be accurate for Torus, however the dlid is all that really
// matters for route analysis, so this should be fine
extern FSTATUS GenPaths(FabricData_t *fabricp,
		PortData *portp1, PortData *portp2,
		IB_PATH_RECORD **ppPathRecords, uint32 *pNumPathRecords);

// tabulate all the ports along the route from slid to dlid
extern FSTATUS TabulateRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid,
					boolean fatTree);
// tabulate all routes from portp1 to portp2
extern FSTATUS TabulateRoutes(FabricData_t *fabricp,
			   		PortData *portp1, PortData *portp2, uint32 *totalPaths,
					uint32 *badPaths, boolean fatTree);
// tabulate all the routes between FIs, exclude loopback routes
extern FSTATUS TabulateCARoutes(FabricData_t *fabricp, Point *focus, uint32 *totalPaths,
					uint32 *badPaths, boolean fatTree);

typedef void (*ReportCallback_t)(PortData *portp1, PortData *portp2,
			   		STL_LID dlid, boolean isBaseLid,
				   	boolean flag /* TRUE=uplink or Recv */, void *context);

// report all routes from portp1 to portp2 that cross reportPort
extern FSTATUS ReportRoutes(FabricData_t *fabricp,
			   		PortData *portp1, PortData *portp2,
				   	PortData *reportPort,
				   	ReportCallback_t callback, void *context, boolean fatTree);
// report all the routes between FIs that cross reportPort,
// exclude loopback routes
extern FSTATUS ReportCARoutes(FabricData_t *fabricp,
			   		PortData *reportPort,
				   	ReportCallback_t callback, void *context, boolean fatTree);

// callback used to indicate that an incomplete route exists between p1 and p2
typedef void (*ValidateCallback_t)(PortData *portp1, PortData *portp2,
			   		STL_LID dlid, boolean isBaseLid, uint8 sl, void *context);

// callback used to indicate a port along an incomplete route
typedef void (*ValidateCallback2_t)(PortData *portp, uint8 vl, void *context);

// validate all routes from portp1 to portp2
extern FSTATUS ValidateRoutes(FabricData_t *fabricp,
			   		PortData *portp1, PortData *portp2,
					uint32 *totalPaths, uint32 *badPaths,
					uint32 usedSLs, uint8,
				   	ValidateCallback_t callback, void *context,
				   	ValidateCallback2_t callback2, void *context2);
// validate all the routes between all LIDs
// exclude loopback routes
extern FSTATUS ValidateAllRoutes(FabricData_t *fabricp, EUI64 portGuid,
					uint8 rc, uint32 *totalPaths, uint32 *badPaths,
				   	ValidateCallback_t callback, void *context,
				   	ValidateCallback2_t callback2, void *context2,
					uint8 useSCSC);

// validate all MC routes between all members
// exclude loopback routes
extern FSTATUS ValidateAllMCRoutes(FabricData_t *fabricp,
					uint32 *totalPaths);
extern FSTATUS ValidateMCRoutes(FabricData_t *fabricp, McGroupData *mcgroupp,
			McEdgeSwitchData *swp, uint32 *pathCount);
extern FSTATUS WalkMCRoute(FabricData_t *fabricp, McGroupData *mcgroupp, PortData *portp, int hop,
		uint8 EntryPort, McLoopInc *pMcLoopInc, uint32 *pathCount);
extern FSTATUS AddSwtichToGroup(FabricData_t *fabricp, McGroupData *mcgroupp, NodeData *groupswitch);
extern void FreeValidateMCRoutes(FabricData_t *fabricp);


typedef void (*ValidateCLRouteCallback_t)(PortData *portp1, PortData *portp2, void *context); 
typedef void (*ValidateCLFabricSummaryCallback_t)(FabricData_t *fabricp, const char *name,
						  uint32 totalPaths, uint32 totalBadPaths,
						  void *context);
typedef void (*ValidateCLDataSummaryCallback_t)(clGraphData_t *graphp, const char *name, void *context);
typedef void (*ValidateCLRouteSummaryCallback_t)(uint32 routesPresent, 
                                                 uint32 routesMissing, 
                                                 uint32 hopsHistogramEntries, 
                                                 uint32 *hopsHistogram, 
                                                 void *context); 
typedef void (*ValidateCLPathSummaryCallback_t)(FabricData_t *fabricp, clConnData_t *connp, int indent, void *context);
typedef void (*ValidateCLLinkSummaryCallback_t)(uint32 id, const char *name, uint32 cycle, uint8 header, int indent, void *context);
typedef void (*ValidateCLLinkStepSummaryCallback_t)(uint32 id, const char *name, uint32 step, uint8 header, int indent, void *context); 

#ifndef __VXWORKS__
typedef FSTATUS (*ValidateCLTimeGetCallback_t)(uint64_t *address, pthread_mutex_t *lock);

extern pthread_mutex_t g_cl_lock; 
extern FSTATUS ValidateAllCreditLoopRoutes(FabricData_t *fabricp, EUI64 portGuid, uint8 rc, 
                                           ValidateCLRouteCallback_t routeCallback,
                                           ValidateCLFabricSummaryCallback_t fabricSummaryCallback,
                                           ValidateCLDataSummaryCallback_t dataSummaryCallback,
                                           ValidateCLRouteSummaryCallback_t routeSummaryCallback,
                                           ValidateCLLinkSummaryCallback_t linkSummaryCallback,
                                           ValidateCLLinkStepSummaryCallback_t linkStepSummaryCallback,
                                           ValidateCLPathSummaryCallback_t pathSummaryCallback,
                                           ValidateCLTimeGetCallback_t timeGetCallback,
                                           void *context, 
                                           uint8 snapshotInFile,
                                           uint8 useSCSC);
extern void CLFabricSummary(FabricData_t *fabricp, const char *name, ValidateCLFabricSummaryCallback_t callback,
			    uint32 totalPaths, uint32 totalBadPaths, void *context);
extern void CLGraphDataSummary(clGraphData_t *graphp, const char *name, ValidateCLDataSummaryCallback_t callback, void *context);
extern FSTATUS CLFabricDataBuildRouteGraph(FabricData_t *fabricp,
                                           ValidateCLRouteSummaryCallback_t routeSummaryCallback,
                                           ValidateCLTimeGetCallback_t timeGetCallback,
                                           void *context,
                                           uint32 usedSLs);
extern void CLGraphDataPrune(clGraphData_t *graphp, ValidateCLTimeGetCallback_t timeGetCallback, int verbose, int quiet);
extern void CLDijkstraFindCycles(FabricData_t *fabricp,
                                 clGraphData_t *graphp,
                                 clDijkstraDistancesAndRoutes_t *drp,
                                 ValidateCLLinkSummaryCallback_t linkSummaryCallback,
                                 ValidateCLLinkStepSummaryCallback_t linkStepSummaryCallback,
                                 ValidateCLPathSummaryCallback_t pathSummaryCallback,
                                 void *context);
extern FSTATUS CLTimeGet(uint64_t *address); 
#endif

extern PortData *GetMapEntry(FabricData_t *fabricp, STL_LID lid);
extern FSTATUS SetMapEntry(FabricData_t *fabricp, STL_LID lid, PortData *pd);
extern void FreeLidMap(FabricData_t *fabricp);

static __inline uint32 ComputeMulticastFDBSize(const STL_SWITCH_INFO *pSwitchInfo)
{
	//STL Volg1 20.2.2.6.5
	return (pSwitchInfo->MulticastFDBTop >= STL_LID_MULTICAST_BEGIN) ? 
			pSwitchInfo->MulticastFDBTop-STL_LID_MULTICAST_BEGIN+1 : 0;
}


static __inline int getVLArb(PortData *p, int *res)
{
	NodeData *n = p->nodep;

	if (n->NodeInfo.NodeType == STL_NODE_SW && p->PortNum != 0) {
		cl_map_item_t *it = cl_qmap_get(&n->Ports, 0);
		if (it == cl_qmap_end(&n->Ports))
			return 1;

		p = PARENT_STRUCT(it, PortData, NodePortsEntry);
	}

	*res = p->PortInfo.CapabilityMask3.s.VLSchedulingConfig == STL_VL_SCHED_MODE_VLARB;
	return 0;
}

// TODO MLID offset mask should be derived from MulticastMask in
// SwitchInfo or PortInfo, not hardcoded to lower 14 bits
#define MULTICAST_LID_OFFSET_MASK 0x3FFF

/**
 * Use this function to get offset in multicast FDB without
 * having to know multicast LID format.
 */
static inline uint32 GetMulticastOffset(STL_LID mlid)
{
	// TODO should use MulticastMask and CollectiveMask from PortInfo
	// or SwitchInfo to determine a) if mlid is a multicast LID and b)
	// how many bits in mlid are offset bits
	return (mlid & MULTICAST_LID_OFFSET_MASK);
}

static inline PortData* getCapabilityPortData(NodeData *nodep, PortData *portp)
{
	if (nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum != 0)
		return FindNodePort(nodep, 0);

	return portp;
}

static inline int getIsVLrSupported(NodeData *nodep, PortData *portp)
{
	PortData* port = getCapabilityPortData(nodep, portp);
	if (port && port->PortInfo.CapabilityMask3.s.IsVLrSupported)
		return 1;
	return 0;
}

static inline int getIsAsyncSC2VLSupported(NodeData *nodep, PortData *portp)
{
	PortData* port = getCapabilityPortData(nodep, portp);
	if (port && port->PortInfo.CapabilityMask3.s.IsAsyncSC2VLSupported)
		return 1;
	return 0;
}

typedef enum {
	SC2VL_UPDATE_TYPE_NONE = 0,
	SC2VL_UPDATE_TYPE_SYNC,
	SC2VL_UPDATE_TYPE_ASYNC
} SC2VLUpdateType;

static inline SC2VLUpdateType getSC2VLUpdateType(NodeData *nodep, PortData *portp, ScvlEnum_t scvlx)
{
	uint8_t currentPortState = portp->PortInfo.PortStates.s.PortState;
	boolean currentPortAsyncSC2VL = getIsAsyncSC2VLSupported(nodep, portp);

	// port must be at least in the Init state
	if (currentPortState < IB_PORT_INIT)
		return SC2VL_UPDATE_TYPE_NONE;

	// switch port 0 - special case
	if (nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum == 0) {
		if (scvlx == Enum_SCVLt)
			return SC2VL_UPDATE_TYPE_SYNC;

		if (scvlx == Enum_SCVLr) {
			if (currentPortState == IB_PORT_INIT)
				return SC2VL_UPDATE_TYPE_SYNC;
			if (currentPortAsyncSC2VL)
				return SC2VL_UPDATE_TYPE_ASYNC;
		}

		return SC2VL_UPDATE_TYPE_NONE;
	}

	// in link state Init only sync update is allowed
	if (currentPortState == IB_PORT_INIT)
		return SC2VL_UPDATE_TYPE_SYNC;

	// async update is allowed only when the link state is Armed or Active and both ports support async update
	if (currentPortAsyncSC2VL) {
		boolean neighborPortAsyncSC2VL = FALSE;

		if (portp->neighbor && portp->neighbor->nodep->valid)
			neighborPortAsyncSC2VL = getIsAsyncSC2VLSupported(portp->neighbor->nodep, portp->neighbor);

		if (neighborPortAsyncSC2VL)
			return SC2VL_UPDATE_TYPE_ASYNC;
	}

	return SC2VL_UPDATE_TYPE_NONE;
}

/* Port Iterator structure to hold information of the port being iterated */
typedef struct _PortIteratorData
{
	boolean lastPortFlag;
}PortIteratorData;

/* Port List Iterator structure to hold information of the port List being iterated */
typedef struct _PortListIteratorData
{
	LIST_ITERATOR currentPort;
}PortListIteratorData;

/* Node Iterator structure to hold information of the node being iterated */
typedef struct _NodeIteratorData
{
	boolean lastNodeFlag;
	cl_map_item_t *pCurrentPort;
}NodeIteratorData;

/* Node List Iterator structure to hold information of the node list being iterated */
typedef struct _NodeListIteratorData
{
	LIST_ITERATOR currentNode;
	cl_map_item_t *pCurrentPort;
}NodeListIteratorData;

/* Ioc Iterator structure to hold information of the Ioc being iterated */
#if !defined(VXWORKS) || defined(BUILD_DMC)
typedef struct _IocIteratorData
{
	boolean lastIocFlag;
	cl_map_item_t *pCurrentPort;
}IocIteratorData;

/* Ioc List Iterator structure to hold information of the Ioc Lst being iterated */
typedef struct _IocListIteratorData
{
	LIST_ITERATOR currentNode;
	cl_map_item_t *pCurrentPort;
}IocListIteratorData;
#endif

/* System Iterator structure to hold information of the system being iterated */
typedef struct _SystemListIteratorData
{
	boolean lastSystemFlag;
	cl_map_item_t *pCurrentNode;
	cl_map_item_t *pCurrentPort;
}SystemListIteratorData;

/* Iterator structure to hold information of the point being iterated */
typedef struct _FIPortIterator
{
	Point *pPoint;
	union {
		PortIteratorData		PortIter;
		NodeIteratorData		NodeIter;
#if !defined(VXWORKS) || defined(BUILD_DMC)
		IocIteratorData			IocIter;
		IocListIteratorData 	IocListIter;
#endif
		SystemListIteratorData	SystemIter;
		NodeListIteratorData	NodeListIter;
		PortListIteratorData	PortListIter;
	} u;
}FIPortIterator;

/*Finds the next non-SW port for each type in the point*/
extern PortData *FIPortIteratorHead(FIPortIterator *pFIPortIterator, Point *pFocus);
/* Finds the first non-SW port for each type in the point.*/
extern PortData *FIPortIteratorNext(FIPortIterator *pFIPortIterator);

#ifdef __cplusplus
};
#endif

#endif /* _TOPOLOGY_H */
