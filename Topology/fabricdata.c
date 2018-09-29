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

#include "topology.h"
#include "topology_internal.h"
#include <stl_helper.h>
#include <stl_convertfuncs.h>


#define	VTIMER_1S		    1000000ull
#define VTIMER_1_MILLISEC           1000ull
#define VTIMER_10_MILLISEC          10000ull
#define VTIMER_100_MILLISEC         100000ull
#define VTIMER_200_MILLISEC         200000ull
#define VTIMER_500_MILLISEC         500000ull
#define RESP_WAIT_TIME		    1000

#define CL_MAX_THREADS              4

typedef struct clListSearchData_s {
   cl_map_item_t      AllListEntry; // key is numeric id
} clListSearchData_t; 

#ifndef __VXWORKS__
typedef struct clThreadContext_s {
   FabricData_t *fabricp; 
   clGraphData_t *graphp; 
   uint32 srcHcaListLength; 
   LIST_ITEM *srcHcaList;
   uint32 threadId; 
   int verbose;
   int quiet; 
   uint32 numVertices; 
   uint8 threadExit; 
   uint64_t sTime, eTime; 
   FSTATUS threadStatus;
   ValidateCLTimeGetCallback_t timeGetCallback;
   uint32 usedSLs;
} clThreadContext_t; 

pthread_mutex_t g_cl_lock; 

static uint32 present_routes = 0, missing_routes = 0; 
static uint32 hops_histogram_entries = 0, hops_histogram_length = 0; 
static uint32 *hops_histogram = NULL; 
static cl_qmap_t hopsHistogramLstMap; 
static cl_qmap_t *routesLstMap; 

#endif

PortData *GetMapEntry(FabricData_t *fabricp, STL_LID lid)
{
	if (lid > TOPLM_LID_MAX) return NULL;
	if (!fabricp->u.LidMap.LidBlocks[TOPLM_BLOCK_NUM(lid)]) return NULL;
	return fabricp->u.LidMap.LidBlocks[TOPLM_BLOCK_NUM(lid)][TOPLM_ENTRY_NUM(lid)];
}

FSTATUS SetMapEntry(FabricData_t *fabricp, STL_LID lid, PortData *pd)
{
	if (lid > TOPLM_LID_MAX) return FERROR;
	if (!fabricp->u.LidMap.LidBlocks[TOPLM_BLOCK_NUM(lid)]) {
		if (!pd) return FSUCCESS;

		fabricp->u.LidMap.LidBlocks[TOPLM_BLOCK_NUM(lid)] =
			(PortData **)MemoryAllocate2AndClear(sizeof(PortData *)*(TOPLM_ENTRIES), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (!fabricp->u.LidMap.LidBlocks[TOPLM_BLOCK_NUM(lid)]) {
			// Failed to allocate
			return FERROR;
		}
	}
	fabricp->u.LidMap.LidBlocks[TOPLM_BLOCK_NUM(lid)][TOPLM_ENTRY_NUM(lid)] = pd;
	return FSUCCESS;
}

void FreeLidMap(FabricData_t *fabricp)
{
	int i;
	for (i = 0; i < TOPLM_BLOCKS; i++) {
		if (fabricp->u.LidMap.LidBlocks[i]) {
			MemoryDeallocate(fabricp->u.LidMap.LidBlocks[i]);
			fabricp->u.LidMap.LidBlocks[i] = NULL;
		}
	}
}


// only FF_LIDARRAY,FF_PMADIRECT,FF_SMADIRECT,FF_DOWNPORTINFO
// flags are used, others ignored
FSTATUS InitFabricData(FabricData_t *fabricp, FabricFlags_t flags)
{
	MemoryClear(fabricp, sizeof(*fabricp));
	cl_qmap_init(&fabricp->AllNodes, NULL);
	if (!(flags & FF_LIDARRAY)) {
		cl_qmap_init(&fabricp->u.AllLids, NULL);
	}
	fabricp->ms_timeout = RESP_WAIT_TIME;
	fabricp->flags = flags & (FF_LIDARRAY|FF_PMADIRECT|FF_SMADIRECT|FF_DOWNPORTINFO);
	cl_qmap_init(&fabricp->AllSystems, NULL);
	cl_qmap_init(&fabricp->ExpectedNodeGuidMap, NULL);
	QListInitState(&fabricp->AllPorts);
	if (! QListInit(&fabricp->AllPorts)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
	QListInitState(&fabricp->ExpectedLinks);
	if (! QListInit(&fabricp->ExpectedLinks)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
	QListInitState(&fabricp->AllFIs);
	if (! QListInit(&fabricp->AllFIs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
	QListInitState(&fabricp->AllSWs);
	if (! QListInit(&fabricp->AllSWs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
	QListInitState(&fabricp->AllVFs);
	if (! QListInit(&fabricp->AllVFs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
#if !defined(VXWORKS) || defined(BUILD_DMC)
	QListInitState(&fabricp->AllIOUs);
	if (! QListInit(&fabricp->AllIOUs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
#endif
	QListInitState(&fabricp->ExpectedFIs);
	if (! QListInit(&fabricp->ExpectedFIs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
	QListInitState(&fabricp->ExpectedSWs);
	if (! QListInit(&fabricp->ExpectedSWs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
	QListInitState(&fabricp->ExpectedSMs);
	if (! QListInit(&fabricp->ExpectedSMs)) {
		fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname);
		goto fail;
	}
#if !defined(VXWORKS) || defined(BUILD_DMC)
	cl_qmap_init(&fabricp->AllIOCs, NULL);
#endif
	cl_qmap_init(&fabricp->AllSMs, NULL);

	// MC routes related structures
	QListInitState(&fabricp->AllMcGroups);
	if (!QListInit(&fabricp->AllMcGroups)) {
		fprintf(stderr, "%s: Unable to initialize List of mcast Members\n", g_Top_cmdname);
		goto fail;
	}

        // credit-loop related lists
        cl_qmap_init(&fabricp->map_guid_to_ib_device, NULL); 
        QListInitState(&fabricp->FIs); 
        if (!QListInit(&fabricp->FIs)) {
           fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname); 
           goto fail;
        }
        QListInitState(&fabricp->Switches); 
        if (!QListInit(&fabricp->Switches)) {
           fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname); 
           goto fail;
        }
        cl_qmap_init(&fabricp->Graph.Arcs, NULL); 
        cl_qmap_init(&fabricp->Graph.map_arc_key_to_arc, NULL); 

	return FSUCCESS;

fail:
	if (flags & FF_LIDARRAY) {
		FreeLidMap(fabricp);
	}
	return FERROR;

}

// create SystemData as needed
// add this Node to the appropriate System
// This should only be invoked once per node (eg. not per NodeRecord)
FSTATUS AddSystemNode(FabricData_t *fabricp, NodeData *nodep)
{
	SystemData* systemp;
	cl_map_item_t *mi;
	FSTATUS status;
	uint64 key;

	systemp = (SystemData *)MemoryAllocate2AndClear(sizeof(SystemData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! systemp) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	cl_qmap_init(&systemp->Nodes, NULL);
	systemp->SystemImageGUID = nodep->NodeInfo.SystemImageGUID;
	key = systemp->SystemImageGUID;
	if (! key)
		key = nodep->NodeInfo.NodeGUID;

	// There can be more than 1 node per system, only create 1 SystemData
	mi = cl_qmap_insert(&fabricp->AllSystems, key, &systemp->AllSystemsEntry);
	if (mi != &systemp->AllSystemsEntry)
	{
		MemoryDeallocate(systemp);
		systemp = PARENT_STRUCT(mi, SystemData, AllSystemsEntry);
	}

	nodep->systemp = systemp;
	if (cl_qmap_insert(&systemp->Nodes, nodep->NodeInfo.NodeGUID, &nodep->SystemNodesEntry) != &nodep->SystemNodesEntry)
	{
		fprintf(stderr, "%s: Duplicate NodeGuids found in nodeRecords: 0x%"PRIx64"\n",
					   g_Top_cmdname, nodep->NodeInfo.NodeGUID);
		goto fail;
	}
	status = FSUCCESS;
done:
	return status;

fail:
	status = FERROR;
	goto done;
}

/* build the fabricp->AllPorts, ALLFIs, and AllSWs lists such that
 * AllPorts is sorted by NodeGUID, PortNum
 * AllFIs, ALLSWs, AllIOUs is sorted by NodeGUID
 */
void BuildFabricDataLists(FabricData_t *fabricp)
{
	cl_map_item_t *p;

	// this list may already be filled when parsing Port definitions
	boolean AllPortsIsEmpty = QListIsEmpty(&fabricp->AllPorts);

	for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
		NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
		cl_map_item_t *q;

		switch (nodep->NodeInfo.NodeType) {
		case STL_NODE_FI:
			QListInsertTail(&fabricp->AllFIs, &nodep->AllTypesEntry);
			break;
		case STL_NODE_SW:
			QListInsertTail(&fabricp->AllSWs, &nodep->AllTypesEntry);
			break;
		default:
			fprintf(stderr, "%s: Ignoring Unknown node type: 0x%x\n",
					g_Top_cmdname, nodep->NodeInfo.NodeType);
			break;
		}
#if !defined(VXWORKS) || defined(BUILD_DMC)
		if (nodep->ioup) {
			QListInsertTail(&fabricp->AllIOUs, &nodep->ioup->AllIOUsEntry);
		}
#endif

		if (AllPortsIsEmpty) {
			for (q=cl_qmap_head(&nodep->Ports); q != cl_qmap_end(&nodep->Ports); q = cl_qmap_next(q)) {
				PortData *portp = PARENT_STRUCT(q, PortData, NodePortsEntry);
				QListInsertTail(&fabricp->AllPorts, &portp->AllPortsEntry);
			}
		}
	}
}

PortSelector* GetPortSelector(PortData *portp)
{
	ExpectedLink *elinkp = portp->elinkp;
	if (elinkp) {
		if (elinkp->portp1 == portp)
			return elinkp->portselp1;
		if (elinkp->portp2 == portp)
			return elinkp->portselp2;
	}
	return NULL;
}

// Determine if portp and its neighbor are an internal link within a single
// system.  Note that some 3rd party products report SystemImageGuid of 0 in
// which case we can only consider links between the same node as internal links
boolean isInternalLink(PortData *portp)
{
	PortData *neighbor = portp->neighbor;

	return (neighbor
			&& portp->nodep->NodeInfo.SystemImageGUID
				== neighbor->nodep->NodeInfo.SystemImageGUID
			&& (portp->nodep->NodeInfo.SystemImageGUID
				|| portp->nodep->NodeInfo.NodeGUID
					== neighbor->nodep->NodeInfo.NodeGUID));
}

// Determine if portp and its neighbor are an Inter-switch Link
boolean isISLink(PortData *portp)
{
	PortData *neighbor = portp->neighbor;

	return (neighbor
			&& portp->nodep->NodeInfo.NodeType == STL_NODE_SW
			&& neighbor->nodep->NodeInfo.NodeType == STL_NODE_SW);
}

// Determine if portp or its neighbor is an FI
boolean isFILink(PortData *portp)
{
	PortData *neighbor = portp->neighbor;

	return (neighbor
			&& (portp->nodep->NodeInfo.NodeType == STL_NODE_FI
				|| neighbor->nodep->NodeInfo.NodeType == STL_NODE_FI));
}

// The routines below are primarily for use when classifying ExpectedLinks
// which did not resolve properly.  So we want them to be a little loose
// and select a link which is expected to be of the given type based on input
// or where the port resolved for either side is connected to a real link of
// the given type.

// Due to this more inclusive definition, oriented toward verification,
// isExternal is not the same as ! isInternal
// Given this definition both isExternal and isInternal could be true for
// the same resolved or vaguely defined link, such as when the elinkp
// has not specified if its internal or external (default is ! internal)
// and the elink is resolved to two disconnected ports where one is an internal
// link and the other is an external link.
// Note: Currently there is no use case for an isInternalExpectedLink function
// and given the defaulting behavior of elinkp->internal such a function would
// not be implementable under this inclusive definition.
//
// Similarly isISExpectedLink is not the same as ! isFIExpectedLink
// isIS will include a link where expected link or either resolved real link
// have at least 1 switch to switch link, even if other expected or real links
// are FI links.  Similarly isFIExpectedLink will match any link where an FI is
// expected on either side or is found on either side of a resolved real link.
// As such the case of an expected link which is SW-SW which matches two
// unrelated ports where one port is part of a FI-SW link and the other is
// part of a SW-SW link, will return true for both functions.
// Note portselp*->NodeType is optional (default NONE) and can resolve a port
// without matching NodeType.  So it is even possible for an expected link
// of SW-SW to resolve to a FI-SW link (verify reports will report the
// incorrect node types as part of fully verifying the link)

// Determine if elinkp is an external link within a single system.
boolean isExternalExpectedLink(ExpectedLink *elinkp)
{
	return (
			// elinkp was specified as external (or Internal not specified)
			(! elinkp->internal)
			// either real port resolved to the elinkp is an external link
			|| (elinkp->portp1 && ! isInternalLink(elinkp->portp1))
			|| (elinkp->portp2 && ! isInternalLink(elinkp->portp2))
		);
}

// Determine if elinkp is an inter-switch link
boolean isISExpectedLink(ExpectedLink *elinkp)
{
	return (
			// elinkp was expicitly specified as an ISL
			((elinkp->portselp1 && elinkp->portselp1->NodeType == STL_NODE_SW)
			 && (elinkp->portselp2 && elinkp->portselp2->NodeType == STL_NODE_SW))
			// either real port resolved to the elinkp is an ISL link
			|| (elinkp->portp1 && isISLink(elinkp->portp1))
			|| (elinkp->portp2 && isISLink(elinkp->portp2))
		);
}

// Determine if elinkp is an FI link
boolean isFIExpectedLink(ExpectedLink *elinkp)
{
	return (
			// either side of elinkp was specified as an FI
			(elinkp->portselp1 && elinkp->portselp1->NodeType == STL_NODE_FI)
			|| (elinkp->portselp2 && elinkp->portselp2->NodeType == STL_NODE_FI)
			// either real port resolved to a link including an FI
			|| (elinkp->portp1 && isFILink(elinkp->portp1))
			|| (elinkp->portp2 && isFILink(elinkp->portp2))
		);
}

// Lookup PKey
// ignores the Full/Limited bit, only checks low 15 bits for a match
// returns index of pkey in overall table or -1 if not found
// if the Partition Table for the port is not available, returns -1
int FindPKey(PortData *portp, uint16 pkey)
{
	uint16 ix, ix_capacity;
	STL_PKEY_ELEMENT *pPartitionTable = portp->pPartitionTable;

	if (! pPartitionTable)
		return -1;
	ix_capacity = PortPartitionTableSize(portp);
	for (ix = 0; ix < ix_capacity; ix++)
	{
		if ((pPartitionTable[ix].AsReg16 & 0x7FFF) == (pkey & 0x7FFF))
			return ix;
	}
	return -1;
}

// Determine if the given port is a member of the given vFabric
// We don't really have all the right data here, especially if the FM has
// combined multiple vFabrics into the same SL and PKey
// but for most cases, we can safely conclude that if the port has the SL and
// PKey configured it is a member of the vFabric.
// This function will return FALSE if QoS data or SL2SCMap is not available
// or if the port is not Armed/Active.
// Given the current FM implementation (and the dependency on SL2SCMap), this
// routine will return FALSE if invoked for non-endpoints
boolean isVFMember(PortData *portp, VFData_t *pVFData) 
{
	STL_VFINFO_RECORD *pR = &pVFData->record;
	uint8 sl = pR->s1.slBase;
	uint8 slResp = (pR->slResponseSpecified? pR->slResponse: sl);
	uint8 slMcast = (pR->slMulticastSpecified? pR->slMulticast: sl);

	// VF only valid if port initialized
	if (! IsPortInitialized(portp->PortInfo.PortStates))
		return FALSE;
	// there is no saquery to find out if a port is a member of a vfabric
	// so a port is assumed to be a member of a vfabric if:
	//	for the base SL of the vfabric, that port has an SC assigned (SC!=15)
	//	and the port has the VF's pkey

	if (! portp->pQOS || ! portp->pQOS->SL2SCMap)
		return FALSE;

	if (portp->pQOS->SL2SCMap->SLSCMap[sl].SC == 15 &&
		portp->pQOS->SL2SCMap->SLSCMap[slResp].SC == 15 &&
		portp->pQOS->SL2SCMap->SLSCMap[slMcast].SC == 15)
		return FALSE;

	return (-1 != FindPKey(portp, pR->pKey));
}

// count the number of armed/active links in the node
uint32 CountInitializedPorts(FabricData_t *fabricp, NodeData *nodep)
{
	cl_map_item_t *p;
	uint32 count = 0;
	for (p=cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p))
	{
		PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
		if (IsPortInitialized(portp->PortInfo.PortStates))
			count++;
	}
	return count;
}

void PortDataFreeQOSData(FabricData_t *fabricp, PortData *portp)
{
	if (portp->pQOS) {
		QOSData *pQOS = portp->pQOS;

		LIST_ITEM *p;
		int i;
		for (i=0; i< SC2SCMAPLIST_MAX; i++) {
			while (!QListIsEmpty(&pQOS->SC2SCMapList[i])) {
				p = QListTail(&pQOS->SC2SCMapList[i]);
				PortMaskSC2SCMap *pSC2SC = (PortMaskSC2SCMap *)QListObj(p);
				QListRemoveTail(&pQOS->SC2SCMapList[i]);
				MemoryDeallocate(pSC2SC->SC2SCMap);
				pSC2SC->SC2SCMap = NULL;
				MemoryDeallocate(pSC2SC);
			}
		}

		if (pQOS->SL2SCMap)
			MemoryDeallocate(pQOS->SL2SCMap);
		if (pQOS->SC2SLMap)
			MemoryDeallocate(pQOS->SC2SLMap);

		MemoryDeallocate(pQOS);
	}
	portp->pQOS = NULL;
}

void PortDataFreeBufCtrlTable(FabricData_t *fabricp, PortData *portp)
{
	if (portp->pBufCtrlTable) {
		MemoryDeallocate(portp->pBufCtrlTable);
	}
	portp->pBufCtrlTable = NULL;
}


void PortDataFreePartitionTable(FabricData_t *fabricp, PortData *portp)
{
	if (portp->pPartitionTable) {
		MemoryDeallocate(portp->pPartitionTable);
	}
	portp->pPartitionTable = NULL;
}


void PortDataFreeCableInfoData(FabricData_t *fabricp, PortData *portp)
{
	if (portp->pCableInfoData) {
		MemoryDeallocate(portp->pCableInfoData);
	}
	portp->pCableInfoData = NULL;
}

void PortDataFreeCongestionControlTableEntries(FabricData_t *fabricp, PortData *portp)
{
	if (portp->pCongestionControlTableEntries) {
		MemoryDeallocate(portp->pCongestionControlTableEntries);
	}
	portp->pCongestionControlTableEntries = NULL;
}

void AllLidsRemove(FabricData_t *fabricp, PortData *portp)
{
	if (! portp->PortGUID)
		return;	// quietly ignore
	switch (portp->PortInfo.PortStates.s.PortState) {
	default:
	case IB_PORT_DOWN:
	case IB_PORT_INIT:
		// may be in AllLids
		if (FindLid(fabricp, portp->EndPortLID) != portp)
			return;
		break;
	case IB_PORT_ARMED:
	case IB_PORT_ACTIVE:
		// should be in AllLids
		break;
	}
	if (fabricp->flags & FF_LIDARRAY) {
		STL_LID first_lid = portp->EndPortLID;
		STL_LID last_lid = portp->EndPortLID |
					((1<<portp->PortInfo.s1.LMC)-1);
		while (first_lid <= last_lid) {
			SetMapEntry(fabricp, first_lid++, NULL);
		}
	} else {
		cl_qmap_remove_item(&fabricp->u.AllLids, &portp->AllLidsEntry);
	}
	fabricp->lidCount -= (1<<portp->PortInfo.s1.LMC);
}

void PartialLidsRemove(FabricData_t *fabricp,  PortData *portp, uint32 count)
{
	STL_LID first_lid = portp->EndPortLID;
	STL_LID last_lid = portp->EndPortLID + count;
	while (first_lid < last_lid) {
		SetMapEntry(fabricp, first_lid++, NULL);
		fabricp->lidCount--;
	}
}

// add the LID for a non-down port to the LID lists
// does nothing for ports in DOWN or INIT state
FSTATUS AllLidsAdd(FabricData_t *fabricp, PortData *portp, boolean force)
{
	cl_map_item_t *mi;

	if (! portp->PortGUID)
		return FSUCCESS;	// quietly ignore

	// we are less strict about switch port 0.  We have seen odd state's
	// reported and unfortunately SM just passes them along.
	// Since Port 0 is in SM DB it must have a trustable LID (note that
	// for simulator, PortDataStateChanged will only call this for Armed/Active)
	if (IB_PORT_ACTIVE != portp->PortInfo.PortStates.s.PortState
		&& IB_PORT_ARMED != portp->PortInfo.PortStates.s.PortState
		&& ! (portp->nodep->NodeInfo.NodeType == STL_NODE_SW
				&& 0 == portp->PortNum
			   	&& IB_PORT_NOP == portp->PortInfo.PortStates.s.PortState))
		return FSUCCESS;	// quietly ignore
	if (portp->EndPortLID > TOPLM_LID_MAX)
		return FERROR;
	if (fabricp->flags & FF_LIDARRAY) {
		STL_LID perm_first_lid = portp->EndPortLID;
		STL_LID first_lid = portp->EndPortLID;
		STL_LID last_lid = portp->EndPortLID |
					((1<<portp->PortInfo.s1.LMC)-1);
		FSTATUS status = FSUCCESS;

		for(;first_lid <= last_lid;first_lid++) {
			PortData *testportp = GetMapEntry(fabricp, first_lid);
			if(testportp != portp) {
				if(testportp != NULL) {
					status = FDUPLICATE;
					if (! force) {
						PartialLidsRemove(fabricp, portp, first_lid - perm_first_lid);
						return status;
					} else {
						AllLidsRemove(fabricp, testportp);
					}
				}
				fabricp->lidCount++;
				SetMapEntry(fabricp, first_lid, portp);
			}
		}
		return status;
	} else {
		// TBD does not verify potential overlap of lid+(1<<lmc)-1 range
		// lidCount will produce incorrect results in case of such overlap
		mi = cl_qmap_insert(&fabricp->u.AllLids, portp->EndPortLID, &portp->AllLidsEntry);
		if (mi != &portp->AllLidsEntry) {
			if (force) {
				AllLidsRemove(fabricp, PARENT_STRUCT(mi, PortData, AllLidsEntry));
				mi = cl_qmap_insert(&fabricp->u.AllLids, portp->EndPortLID, &portp->AllLidsEntry);
				ASSERT(mi == &portp->AllLidsEntry);
				fabricp->lidCount += (1<<portp->PortInfo.s1.LMC);
			}
			return FDUPLICATE;
		} else {
			fabricp->lidCount += (1<<portp->PortInfo.s1.LMC);
			return FSUCCESS;
		}
	}
}

STL_SCSCMAP * QOSDataLookupSCSCMap(PortData *portp, uint8_t outport, int extended) {
	LIST_ITEM *p;
	PortMaskSC2SCMap *pSC2SC;
	QOSData *pQOS = portp->pQOS;

	if (!pQOS)
		return NULL;
	if ((extended >= SC2SCMAPLIST_MAX) || (extended < 0))
		return NULL;
 
	for (p = QListHead(&pQOS->SC2SCMapList[extended]); p != NULL; p = QListNext(&pQOS->SC2SCMapList[extended], p)) {
		pSC2SC = (PortMaskSC2SCMap *)QListObj(p);

		if (StlIsPortInPortMask(pSC2SC->outports, outport))
			return (pSC2SC->SC2SCMap);
	}
	return NULL;
}

// Add a SCSC table to QOS data
// If a matching SCSC table already exists, add egress to its port mask
// Otherwise, create a new entry in the SCSCMap list for this table
void QOSDataAddSCSCMap(PortData *portp, uint8_t outport, int extended, const STL_SCSCMAP *pSCSC) {
	LIST_ITEM *p;
	QOSData *pQOS = portp->pQOS;
	PortMaskSC2SCMap *pSC2SC2;
	PortMaskSC2SCMap *pEmptySC2SC2 = NULL;
	int entryFound = 0;

	if (!pQOS)
		return;
	if ((extended >= SC2SCMAPLIST_MAX) || (extended < 0))
		return;

	for (p = QListHead(&pQOS->SC2SCMapList[extended]); p != NULL; p = QListNext(&pQOS->SC2SCMapList[extended], p)) {
		pSC2SC2 = (PortMaskSC2SCMap *)QListObj(p);

		if (!memcmp(pSC2SC2->SC2SCMap, pSCSC, sizeof(STL_SCSCMAP))) {
			StlAddPortToPortMask(pSC2SC2->outports, outport);
			entryFound = 1;
		} else if (StlIsPortInPortMask(pSC2SC2->outports, outport)) {
			// the maps don't match but the port does, remove & replace with new map
			StlClearPortInPortMask(pSC2SC2->outports, outport);
			if (StlNumPortsSetInPortMask(pSC2SC2->outports, portp->nodep->NodeInfo.NumPorts) ==0) {
				pEmptySC2SC2 = pSC2SC2;
			}
		}
	}

	if (entryFound) {
		if(pEmptySC2SC2) {
			QListRemoveItem(&pQOS->SC2SCMapList[extended], &pEmptySC2SC2->SC2SCMapListEntry);
		}
		return;
	}

	pSC2SC2 = pEmptySC2SC2;
	if (!pSC2SC2) {
		// never found a matching map, create a new one
		pSC2SC2 = (PortMaskSC2SCMap *)MemoryAllocate2AndClear(sizeof(PortMaskSC2SCMap), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (!pSC2SC2) {
			// memory error
			fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
			return;
		}

		ListItemInitState(&pSC2SC2->SC2SCMapListEntry);
		QListSetObj(&pSC2SC2->SC2SCMapListEntry, pSC2SC2);

		pSC2SC2->SC2SCMap = (STL_SCSCMAP *)MemoryAllocate2AndClear(sizeof(STL_SCSCMAP), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (!pSC2SC2->SC2SCMap) {
			// memory error
			fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
			return;
		}
		QListInsertTail(&pQOS->SC2SCMapList[extended], &pSC2SC2->SC2SCMapListEntry);
	}

	memcpy(pSC2SC2->SC2SCMap, pSCSC, sizeof(STL_SCSCMAP));
	StlAddPortToPortMask(pSC2SC2->outports, outport);
}

// Set new Port Info based on a Set(PortInfo).  For use by fabric simulator
// assumes pInfo already validated and any Noop fields filled in with correct
// values.
// MODIFIED FOR STL
void PortDataStateChanged(FabricData_t *fabricp, PortData *portp)
{
	if (IB_PORT_ACTIVE == portp->PortInfo.PortStates.s.PortState
		|| IB_PORT_ARMED == portp->PortInfo.PortStates.s.PortState) {
		if (FSUCCESS != AllLidsAdd(fabricp, portp, TRUE)) {
			fprintf(stderr, "%s: Overwrote Duplicate LID found in portRecords: PortGUID 0x%016"PRIx64" LID 0x%x Port %u Node %.*s\n",
				g_Top_cmdname,
				portp->PortGUID, portp->EndPortLID,
				portp->PortNum, STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)portp->nodep->NodeDesc.NodeString);
		}
	} else {
		AllLidsRemove(fabricp, portp);
	}

}

// Set new Port Info based on a Set(PortInfo).  For use by fabric simulator
// assumes pInfo already validated and any Noop fields filled in with correct
// values.
// MODIFIED FOR STL
void PortDataSetPortInfo(FabricData_t *fabricp, PortData *portp, STL_PORT_INFO *pInfo)
{
	portp->PortInfo = *pInfo;
	if (portp->PortGUID) {
		portp->EndPortLID = pInfo->LID;
		/* Only port 0 in a switch has a GUID.  It's LID is the switch's LID */
		if (portp->nodep->pSwitchInfo) {
			cl_map_item_t *q;
			NodeData *nodep = portp->nodep;

			nodep->pSwitchInfo->RID.LID = pInfo->LID;
			/* also set EndPortLID in all of switch's PortInfoRecords */
			for (q=cl_qmap_head(&nodep->Ports); q != cl_qmap_end(&nodep->Ports); q = cl_qmap_next(q)) {
				PortData *p = PARENT_STRUCT(q, PortData, NodePortsEntry);
				p->EndPortLID = pInfo->LID;
			}
		}
	}
	PortDataStateChanged(fabricp, portp);
}

// remove Port from lists and free it
// Only removes from AllLids and NodeData.Ports, caller must remove from
// any other lists if called after BuildFabricLists
void PortDataFree(FabricData_t *fabricp, PortData *portp)
{
	NodeData *nodep = portp->nodep;

	if (portp->context && g_Top_FreeCallbacks.pPortDataFreeCallback)
		(*g_Top_FreeCallbacks.pPortDataFreeCallback)(fabricp, portp);

	if (portp->PortGUID)
		AllLidsRemove(fabricp, portp);
	cl_qmap_remove_item(&nodep->Ports, &portp->NodePortsEntry);
	if (portp->pPortStatus)
		MemoryDeallocate(portp->pPortStatus);
	PortDataFreeQOSData(fabricp, portp);
	PortDataFreeBufCtrlTable(fabricp, portp);
	PortDataFreePartitionTable(fabricp, portp);


	PortDataFreeCableInfoData(fabricp, portp);
	PortDataFreeCongestionControlTableEntries(fabricp, portp);
	MemoryDeallocate(portp);
}

FSTATUS PortDataAllocateQOSData(FabricData_t *fabricp, PortData *portp)
{
	QOSData *pQOS;
	int i;

	ASSERT(! portp->pQOS);	// or could free if present
	portp->pQOS = (QOSData *)MemoryAllocate2AndClear(sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! portp->pQOS) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}
	pQOS = portp->pQOS;
	if (portp->nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum) {
		for (i=0; i<SC2SCMAPLIST_MAX; i++) {
		// external switch ports get SC2SC map
			QListInitState(&pQOS->SC2SCMapList[i]);
			if (!QListInit(&pQOS->SC2SCMapList[i])) {
				fprintf(stderr, "%s: Unable to initialize SC2SCMaps member list\n", g_Top_cmdname);
				goto fail;
			}
		}
	} else {
		// HFI and Switch Port 0 get SL2SC and SC2SL
		pQOS->SL2SCMap = (STL_SLSCMAP *)MemoryAllocate2AndClear(sizeof(STL_SLSCMAP), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (! pQOS->SL2SCMap) {
			fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
			goto fail;
		}

		pQOS->SC2SLMap = (STL_SCSLMAP *)MemoryAllocate2AndClear(sizeof(STL_SCSLMAP), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (!pQOS->SC2SLMap) {
			goto fail;
		}
	}

	return FSUCCESS;
fail:
	PortDataFreeQOSData(fabricp, portp);
	return FINSUFFICIENT_MEMORY;
}

FSTATUS PortDataAllocateAllQOSData(FabricData_t *fabricp)
{
	LIST_ITEM *p;
	FSTATUS status = FSUCCESS;

	for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
		PortData *portp = (PortData *)QListObj(p);
		FSTATUS s;
		s = PortDataAllocateQOSData(fabricp, portp);
		if (FSUCCESS != s)
			status = s;
	}
	return status;
}

FSTATUS PortDataAllocateBufCtrlTable(FabricData_t *fabricp, PortData *portp)
{
	ASSERT(! portp->pBufCtrlTable);	// or could free if present
	portp->pBufCtrlTable = MemoryAllocate2AndClear(sizeof(STL_BUFFER_CONTROL_TABLE), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! portp->pBufCtrlTable) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	return FSUCCESS;
fail:
	//PortDataFreeBufCtrlTable(fabricp, portp);
	return FINSUFFICIENT_MEMORY;
}


FSTATUS PortDataAllocateAllBufCtrlTable(FabricData_t *fabricp)
{
	LIST_ITEM *p;
	FSTATUS status = FSUCCESS;

	for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
		PortData *portp = (PortData *)QListObj(p);
		FSTATUS s;
		s = PortDataAllocateBufCtrlTable(fabricp, portp);
		if (FSUCCESS != s)
			status = s;
	}
	return status;
}

uint16 PortPartitionTableSize(PortData *portp)
{
	NodeData *nodep = portp->nodep;
	if (nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum) {
		// Switch External Ports table size is defined in SwitchInfo
		if (! nodep->pSwitchInfo) {
			// guess the limits, we don't have SwitchInfo
			// or we haven't yet read it from the snapshot while parsing
			return nodep->NodeInfo.PartitionCap;
		} else {
			return nodep->pSwitchInfo->SwitchInfoData.PartitionEnforcementCap;
		}
	} else {
		// Switch Port 0 and FI table size is defined by NodeInfo
		return nodep->NodeInfo.PartitionCap;
	}
}

FSTATUS PortDataAllocatePartitionTable(FabricData_t *fabricp, PortData *portp)
{
	uint16 size;

	ASSERT(! portp->pPartitionTable);	// or could free if present
	size = PortPartitionTableSize(portp);
	portp->pPartitionTable = (STL_PKEY_ELEMENT *)MemoryAllocate2AndClear(sizeof(STL_PKEY_ELEMENT)*size, IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! portp->pPartitionTable) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	return FSUCCESS;
fail:
	//PortDataFreePartitionTable(fabricp, portp);
	return FINSUFFICIENT_MEMORY;
}

FSTATUS PortDataAllocateAllPartitionTable(FabricData_t *fabricp)
{
	LIST_ITEM *p;
	FSTATUS status = FSUCCESS;

	for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
		PortData *portp = (PortData *)QListObj(p);
		FSTATUS s;
		s = PortDataAllocatePartitionTable(fabricp, portp);
		if (FSUCCESS != s)
			status = s;
	}
	return status;
}


FSTATUS PortDataAllocateCableInfoData(FabricData_t *fabricp, PortData *portp)
{
	ASSERT(! portp->pCableInfoData);	// or could free if present
	portp->pCableInfoData = MemoryAllocate2AndClear(STL_CIB_STD_LEN, IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! portp->pCableInfoData) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	return FSUCCESS;
fail:
	//PortDataFreeCableInfoData(fabricp, portp);
	return FINSUFFICIENT_MEMORY;
}

FSTATUS PortDataAllocateAllCableInfo(FabricData_t *fabricp)
{
	LIST_ITEM *p;
	FSTATUS status = FSUCCESS;

	for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
		PortData *portp = (PortData *)QListObj(p);
		FSTATUS s;
		// skip switch port 0
		if (! portp->PortNum)
			continue;
		s = PortDataAllocateCableInfoData(fabricp, portp);
		if (FSUCCESS != s)
			status = s;
	}
	return status;
}

FSTATUS PortDataAllocateCongestionControlTableEntries(FabricData_t *fabricp, PortData *portp)
{
	ASSERT(! portp->pCongestionControlTableEntries);	// or could free if present
	if (! portp->nodep->CongestionInfo.ControlTableCap)
		return FSUCCESS;
	portp->pCongestionControlTableEntries = MemoryAllocate2AndClear(
							sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY)
								* STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES
								* portp->nodep->CongestionInfo.ControlTableCap,
							IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! portp->pCongestionControlTableEntries) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	return FSUCCESS;
fail:
	//PortDataFreeCongestionControlTableEntries(fabricp, portp);
	return FINSUFFICIENT_MEMORY;
}

FSTATUS PortDataAllocateAllCongestionControlTableEntries(FabricData_t *fabricp)
{
	LIST_ITEM *p;
	FSTATUS status = FSUCCESS;

	for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
		PortData *portp = (PortData *)QListObj(p);
		FSTATUS s;
		// only applicable to HFIs and enhanced switch port 0
		if (portp->nodep->NodeInfo.NodeType == STL_NODE_SW
			&& portp->PortNum)
			continue;
		if (! portp->nodep->CongestionInfo.ControlTableCap)
			continue;
		s = PortDataAllocateCongestionControlTableEntries(fabricp, portp);
		if (FSUCCESS != s)
			status = s;
	}
	return status;
}

// guid is the PortGUID as found in corresponding NodeRecord
// we adjust as needed to account for Switch Ports (only switch port 0 has guid)
PortData* NodeDataAddPort(FabricData_t *fabricp, NodeData *nodep, EUI64 guid, STL_PORTINFO_RECORD *pPortInfo)
{
	PortData *portp;

	portp = (PortData*)MemoryAllocate2AndClear(sizeof(PortData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! portp) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	ListItemInitState(&portp->AllPortsEntry);
	QListSetObj(&portp->AllPortsEntry, portp);
	portp->PortInfo = pPortInfo->PortInfo;
	portp->EndPortLID = pPortInfo->RID.EndPortLID;

	if (nodep->NodeInfo.NodeType == STL_NODE_SW) {
		// a switch only gets 1 port Guid, we save it for switch
		// port 0 (the "virtual management port")
		portp->PortNum = pPortInfo->RID.PortNum;
		portp->PortGUID = portp->PortNum?0:guid;
	} else {
		portp->PortNum = pPortInfo->PortInfo.LocalPortNum;
		portp->PortGUID = guid;
	}
	portp->rate = StlLinkSpeedWidthToStaticRate(
					(portp->PortInfo.LinkSpeed.Active),
					(portp->PortInfo.LinkWidth.Active));
	portp->nodep = nodep;
	//DisplayPortInfoRecord(&pPortInfoRecords[i], 0);
	//printf("process PortNumber: %u\n", portp->PortNum);

	if (cl_qmap_insert(&nodep->Ports, portp->PortNum, &portp->NodePortsEntry) != &portp->NodePortsEntry)
	{
		fprintf(stderr, "%s: Duplicate PortNums found in portRecords: LID 0x%x Port %u Node: %.*s\n",
					   	g_Top_cmdname,
					   	portp->EndPortLID,
					   	portp->PortNum, STL_NODE_DESCRIPTION_ARRAY_SIZE,
						(char*)nodep->NodeDesc.NodeString);
		MemoryDeallocate(portp);
		goto fail;
	}
	if (FSUCCESS != AllLidsAdd(fabricp, portp, FALSE))
	{
		fprintf(stderr, "%s: Duplicate LIDs found in portRecords: PortGUID 0x%016"PRIx64" LID 0x%x Port %u Node %.*s\n",
			   	g_Top_cmdname,
			   	portp->PortGUID, portp->EndPortLID,
			   	portp->PortNum, STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString);
		cl_qmap_remove_item(&nodep->Ports, &portp->NodePortsEntry);
		MemoryDeallocate(portp);
		goto fail;
	}
	//DisplayPortInfoRecord(pPortInfo, 0);

	return portp;

fail:
	return NULL;
}

FSTATUS NodeDataSetSwitchInfo(NodeData *nodep, STL_SWITCHINFO_RECORD *pSwitchInfo)
{
	ASSERT(! nodep->pSwitchInfo);
	nodep->pSwitchInfo = (STL_SWITCHINFO_RECORD*)MemoryAllocate2AndClear(sizeof(STL_SWITCHINFO_RECORD), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! nodep->pSwitchInfo) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		return FERROR;
	}
	// leave the extra vendor specific fields zeroed
	nodep->pSwitchInfo->RID.LID = pSwitchInfo->RID.LID;
	nodep->pSwitchInfo->SwitchInfoData = pSwitchInfo->SwitchInfoData;
	return FSUCCESS;
}

// TBD - routines to add/set IOC and IOU information

NodeData *FabricDataAddNode(FabricData_t *fabricp, STL_NODE_RECORD *pNodeRecord, boolean *new_nodep)
{
	NodeData *nodep = (NodeData*)MemoryAllocate2AndClear(sizeof(NodeData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	cl_map_item_t *mi;
	boolean new_node = TRUE;

	if (! nodep) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}
	//printf("process NodeRecord LID: 0x%x\n", pNodeRecords->RID.s.LID);
	//DisplayNodeRecord(pNodeRecord, 0);

	cl_qmap_init(&nodep->Ports, NULL);

        nodep->NodeInfo = pNodeRecord->NodeInfo;

	nodep->NodeDesc = pNodeRecord->NodeDesc;

	// zero out port specific fields, the PortData will handle these
	nodep->NodeInfo.PortGUID=0;
	nodep->NodeInfo.u1.s.LocalPortNum=0;
	ListItemInitState(&nodep->AllTypesEntry);
	QListSetObj(&nodep->AllTypesEntry, nodep);

	// when sweeping we get 1 NodeRecord per port on a node, so only save
	// 1 NodeData structure per node and discard the duplicates
	mi = cl_qmap_insert(&fabricp->AllNodes, nodep->NodeInfo.NodeGUID, &nodep->AllNodesEntry);
	if (mi != &nodep->AllNodesEntry)
	{
		MemoryDeallocate(nodep);
		nodep = PARENT_STRUCT(mi, NodeData, AllNodesEntry);
		new_node = FALSE;
	}

	if (new_node) {
		if (FSUCCESS != AddSystemNode(fabricp, nodep)) {
			cl_qmap_remove_item(&fabricp->AllNodes, &nodep->AllNodesEntry);
			MemoryDeallocate(nodep);
			goto fail;
		}
	}

	if (new_nodep)
		*new_nodep = new_node;
	return nodep;

fail:
	return NULL;
}


#ifdef PRODUCT_OPENIB_FF


McGroupData *FabricDataAddMCGroup(FabricData_t *fabricp, struct omgt_port *port, int quiet, IB_MCMEMBER_RECORD *pMCGRecord,
		boolean *new_nodep, FILE *verbose_file)
{
	FSTATUS  status;
	McGroupData *mcgroupp = (McGroupData*)MemoryAllocate2AndClear(sizeof(McGroupData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	boolean new_node = TRUE;

	if (! mcgroupp) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		return NULL;
	}

	// init list of McGroupMembers
	QListInitState(&mcgroupp->AllMcGroupMembers);
	if ( !QListInit(&mcgroupp->AllMcGroupMembers)) {
		fprintf(stderr, "%s: Unable to initialize MCGroup member list\n", g_Top_cmdname);
		MemoryDeallocate(mcgroupp);
		return NULL;
	}

	// init list of switches in a group
	QListInitState(&mcgroupp->EdgeSwitchesInGroup);
	if ( !QListInit(&mcgroupp->EdgeSwitchesInGroup)) {
		fprintf(stderr, "%s: Unable to initialize list of Switches in a MC group\n", g_Top_cmdname);
		MemoryDeallocate(mcgroupp);
		return NULL;
	}
	mcgroupp->MGID = pMCGRecord->RID.MGID;
	mcgroupp->MLID = MCAST16_TO_MCAST32(pMCGRecord->MLID);


	ListItemInitState(&mcgroupp->AllMcGMembersEntry);
	QListSetObj(&mcgroupp->AllMcGMembersEntry, mcgroupp);

	// search members of the group using opasaquery -o mcmember -m nodep->MLID
	// and add them to the list of group members
	status = GetAllMCGroupMember(fabricp, mcgroupp, port, quiet, verbose_file);

	if (status != FSUCCESS) {
		fprintf(stderr, "%s: Unable to get MCGroup member list\n", g_Top_cmdname);
		MemoryDeallocate(mcgroupp);
		if (status == FINSUFFICIENT_MEMORY)
			fprintf(stderr, "%s: Insufficient memory to allocate MC Member\n", g_Top_cmdname);
			return NULL;
	}

	// this part needs to be optimized
	LIST_ITEM *p;
	boolean found = FALSE;
	p=QListHead(&fabricp->AllMcGroups);

// insert everything in the fabric structure ordered by MGID
	while (!found && p != NULL) {
		McGroupData *pMCG = (McGroupData *)QListObj(p);
		if ( pMCG->MLID > mcgroupp->MLID)
			p = QListNext(&fabricp->AllMcGroups, p);
		else {
			// insert mcmember element
				QListInsertNext(&fabricp->AllMcGroups,p, &mcgroupp->AllMcGMembersEntry);
				found = TRUE;
		}
	} //end while

	if (!found)
			QListInsertTail(&fabricp->AllMcGroups, &mcgroupp->AllMcGMembersEntry);

	if (new_nodep)
		*new_nodep = new_node;

	return mcgroupp;
}

#endif


FSTATUS AddEdgeSwitchToGroup(FabricData_t *fabricp, McGroupData *mcgroupp, NodeData *groupswitch, uint8 SWentryport)
{
	LIST_ITEM *p;
	boolean found;
	FSTATUS status;

	// this linear insertion needs to be optimized
	found = FALSE;
	p=QListHead(&mcgroupp->EdgeSwitchesInGroup);
	while (!found && (p != NULL)) {
		McEdgeSwitchData *pSW = (McEdgeSwitchData *)QListObj(p);
		if	(pSW->NodeGUID == groupswitch->NodeInfo.NodeGUID)
			found = TRUE;
		else
			p = QListNext(&mcgroupp->EdgeSwitchesInGroup, p);
	}
	if (!found) {
			McEdgeSwitchData *mcsw = (McEdgeSwitchData*)MemoryAllocate2AndClear(sizeof(McEdgeSwitchData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
			if (! mcsw) {
				status = FINSUFFICIENT_MEMORY;
				return status;
			}
			mcsw->NodeGUID = groupswitch->NodeInfo.NodeGUID;
			mcsw->pPort = FindPortGuid(fabricp, groupswitch->NodeInfo.NodeGUID );
			mcsw->EntryPort = SWentryport;
			QListSetObj(&mcsw->McEdgeSwitchEntry, mcsw);
			QListInsertTail(&mcgroupp->EdgeSwitchesInGroup, &mcsw->McEdgeSwitchEntry);
	}
	return FSUCCESS;
}

FSTATUS FabricDataAddLink(FabricData_t *fabricp, PortData *p1, PortData *p2)
{
	// The Link Records will be reported in both directions
	// so discard duplicates
	if (p1->neighbor == p2 && p2->neighbor == p1)
		goto fail;
	if (p1->neighbor) {
		fprintf(stderr, "%s: Skipping Duplicate Link Record reference to port: LID 0x%x Port %u Node %.*s\n",
			   	g_Top_cmdname,
			   	p1->EndPortLID,
			   	p1->PortNum, NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p1->nodep->NodeDesc.NodeString);
		goto fail;
	}
	if (p2->neighbor) {
		fprintf(stderr, "%s: Skipping Duplicate Link Record reference to port: LID 0x%x Port %u Node %.*s\n",
			   	g_Top_cmdname,
			   	p2->EndPortLID,
			   	p2->PortNum, NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p2->nodep->NodeDesc.NodeString);
		goto fail;
	}
	//DisplayLinkRecord(pLinkRecord, 0);
	p1->neighbor = p2;
	p2->neighbor = p1;
	// for consistency between runs, we always report the "from" port
	// as the one with lower numbered NodeGUID/PortNum
	if (p1->nodep->NodeInfo.NodeGUID != p2->nodep->NodeInfo.NodeGUID) {
		if (p1->nodep->NodeInfo.NodeGUID < p2->nodep->NodeInfo.NodeGUID) {
			p1->from = 1;
			p2->from = 0;
		} else {
			p1->from = 0;
			p2->from = 1;
		}
	} else {
		if (p1->PortNum < p2->PortNum) {
			p1->from = 1;
			p2->from = 0;
		} else {
			p1->from = 0;
			p2->from = 1;
		}
	}
	if (p1->rate != p2->rate) {
		fprintf(stderr, "\n%s: Warning: Ignoring Inconsistent Active Speed/Width for link between:\n", g_Top_cmdname);
		fprintf(stderr, "  %4s 0x%016"PRIx64" %3u %s %.*s\n",
				StlStaticRateToText(p1->rate),
				p1->nodep->NodeInfo.NodeGUID,
				p1->PortNum,
				StlNodeTypeToText(p1->nodep->NodeInfo.NodeType),
				NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p1->nodep->NodeDesc.NodeString);
		fprintf(stderr, "  %4s 0x%016"PRIx64" %3u %s %.*s\n",
				StlStaticRateToText(p2->rate),
				p2->nodep->NodeInfo.NodeGUID,
				p2->PortNum,
				StlNodeTypeToText(p2->nodep->NodeInfo.NodeType),
				NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p2->nodep->NodeDesc.NodeString);
	}
	++(fabricp->LinkCount);
	if (! isInternalLink(p1))
		++(fabricp->ExtLinkCount);
	if (isFILink(p1))
		++(fabricp->FILinkCount);
	if (isISLink(p1))
		++(fabricp->ISLinkCount);
	if (! isInternalLink(p1)&& isISLink(p1))
		++(fabricp->ExtISLinkCount);

	return FSUCCESS;

fail:
	return FERROR;
}

FSTATUS FabricDataAddLinkRecord(FabricData_t *fabricp, STL_LINK_RECORD *pLinkRecord)
{
	PortData *p1, *p2;

	//printf("process LinkRecord LID: 0x%x\n", pLinkRecord->RID.s.LID);
	p1 = FindLidPort(fabricp, pLinkRecord->RID.FromLID, pLinkRecord->RID.FromPort);
	if (! p1) {
		fprintf(stderr, "%s: Can't find \"From\" Lid 0x%x Port %u: Skipping\n",
				g_Top_cmdname,
				pLinkRecord->RID.FromLID, pLinkRecord->RID.FromPort);
		return FERROR;
	}
	p2 = FindLidPort(fabricp, pLinkRecord->ToLID, pLinkRecord->ToPort);
	if (! p2) {
		fprintf(stderr, "%s: Can't find \"To\" Lid 0x%x Port %u: Skipping\n",
				g_Top_cmdname,
				pLinkRecord->ToLID, pLinkRecord->ToPort);
		return FERROR;
	}
	return FabricDataAddLink(fabricp, p1, p2);
}

// This allows a link to be removed, its mainly a helper for manual
// construction of topologies which need to "undo links" they created
FSTATUS FabricDataRemoveLink(FabricData_t *fabricp, PortData *p1)
{
	PortData *p2 = p1->neighbor;

	if (! p1->neighbor)
		goto fail;
	if (p1 == p2) {
		fprintf(stderr, "%s: Corrupted Topology, port linked to self: LID 0x%x Port %u Node %.*s\n",
			   	g_Top_cmdname,
			   	p1->EndPortLID,
			   	p1->PortNum, NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p1->nodep->NodeDesc.NodeString);
		goto fail;
	}
	if (p2->neighbor != p1) {
		fprintf(stderr, "%s: Corrupted Topology, port linked inconsistently: LID 0x%x Port %u Node %.*s\n",
			   	g_Top_cmdname,
			   	p1->EndPortLID,
			   	p1->PortNum, NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p1->nodep->NodeDesc.NodeString);
		goto fail;
	}
	--(fabricp->LinkCount);
	if (! isInternalLink(p1))
		--(fabricp->ExtLinkCount);
	if (isFILink(p1))
		--(fabricp->FILinkCount);
	if (isISLink(p1))
		--(fabricp->ISLinkCount);
	if (! isInternalLink(p1)&& isISLink(p1))
		--(fabricp->ExtISLinkCount);
	p1->rate = 0;
	p1->neighbor = NULL;
	p1->from = 0;
	p2->rate = 0;
	p2->neighbor = NULL;
	p2->from = 0;

	return FSUCCESS;

fail:
	return FERROR;
}
#if !defined(VXWORKS) || defined(BUILD_DMC)
// remove Ioc from lists and free it
// Only removes from AllIOCs and IouData.Iocs, caller must remove from
// any other lists if called after BuildFabricLists
void IocDataFree(FabricData_t *fabricp, IocData *iocp)
{
	IouData *ioup = iocp->ioup;

	if (iocp->context && g_Top_FreeCallbacks.pIocDataFreeCallback)
		(*g_Top_FreeCallbacks.pIocDataFreeCallback)(fabricp, iocp);

	QListRemoveItem(&ioup->Iocs, &iocp->IouIocsEntry);
	cl_qmap_remove_item(&fabricp->AllIOCs, &iocp->AllIOCsEntry);
	if (iocp->Services)
		MemoryDeallocate(iocp->Services);
	MemoryDeallocate(iocp);
}

// Free all parsed Iocs for this Iou
// Only removes Iocs from AllIOCs and IouData.Iocs, caller must remove from
// any other lists if called after BuildFabricLists
void IouDataFreeIocs(FabricData_t *fabricp, IouData *ioup)
{
	LIST_ITEM *p;

	// this list is sorted/keyed by NodeGUID
	for (p=QListHead(&ioup->Iocs); p != NULL; p = QListHead(&ioup->Iocs)) {
		IocDataFree(fabricp, (IocData*)QListObj(p));
	}
}

// remove Iou from NodeData.ioup and free it and its IOCs
// Only removes Iocs from AllIOCs and IouData.Iocs, caller must remove from
// any other lists if called after BuildFabricLists
void IouDataFree(FabricData_t *fabricp, IouData *ioup)
{
	if (ioup->context && g_Top_FreeCallbacks.pIouDataFreeCallback)
		(*g_Top_FreeCallbacks.pIouDataFreeCallback)(fabricp, ioup);

	IouDataFreeIocs(fabricp, ioup);
	if (ioup->nodep && ioup->nodep->ioup)
		ioup->nodep->ioup = NULL;
	MemoryDeallocate(ioup);
}
#endif
// Free all ports for this node
// Only removes ports from AllLids and NodeData.Ports, caller must remove from
// any other lists if called after BuildFabricLists
void NodeDataFreePorts(FabricData_t *fabricp, NodeData *nodep)
{
	cl_map_item_t *p;

	// this list is sorted/keyed by NodeGUID
	for (p=cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_head(&nodep->Ports)) {
		PortDataFree(fabricp, PARENT_STRUCT(p, PortData, NodePortsEntry));
	}
}


void NodeDataFreeSwitchData(FabricData_t *fabricp, NodeData *nodep)
{
	uint8_t i;

	if (nodep->switchp) {
		SwitchData *switchp = nodep->switchp;

		if (switchp->LinearFDB)
			MemoryDeallocate(switchp->LinearFDB);
		if (switchp->PortGroupFDB)
			MemoryDeallocate(switchp->PortGroupFDB);
		if (switchp->PortGroupElements)
			MemoryDeallocate(switchp->PortGroupElements);
		for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i)
			if (switchp->MulticastFDB[i])
				MemoryDeallocate(switchp->MulticastFDB[i]);


		MemoryDeallocate(switchp);
	}
	nodep->switchp = NULL;
}


FSTATUS NodeDataAllocateFDB(FabricData_t *fabricp, NodeData *nodep,
							uint32 LinearFDBSize) {

	if (NodeDataSwitchResizeLinearFDB(nodep, LinearFDBSize) != FSUCCESS) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}


	return FSUCCESS;
}

FSTATUS NodeDataAllocateSwitchData(FabricData_t *fabricp, NodeData *nodep,
				uint32 LinearFDBSize, uint32 MulticastFDBSize)
{
	SwitchData *switchp;

	if (nodep->switchp)	{
		// Free prior switch data
		NodeDataFreeSwitchData(fabricp, nodep);
	}

	nodep->switchp = (SwitchData *)MemoryAllocate2AndClear(sizeof(SwitchData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! nodep->switchp) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		goto fail;
	}

	switchp = nodep->switchp;

	if (NodeDataAllocateFDB(fabricp, nodep, LinearFDBSize) != FSUCCESS) {
		goto fail;
	}

	if (NodeDataSwitchResizeMcastFDB(nodep, MulticastFDBSize) != FSUCCESS) {
		goto fail;
	}

	// These are for the new STL PG Adaptative routing. Note that we allocate
	// in multiples of full "blocks" which simplifies sending PG MADs to the
	// switches and supports up to 64 ports per port group. Supporting more
	// than 64 ports will require a different memory management technique.
	switchp->PortGroupSize = ROUNDUP(nodep->pSwitchInfo->SwitchInfoData.PortGroupCap, NUM_PGT_ELEMENTS_BLOCK);
	switchp->PortGroupElements = 
		MemoryAllocate2AndClear(switchp->PortGroupSize * 
			sizeof(STL_PORTMASK), IBA_MEM_FLAG_PREMPTABLE, MYTAG);



	return FSUCCESS;
fail:
	NodeDataFreeSwitchData(fabricp, nodep);
	return FINSUFFICIENT_MEMORY;
}



//----------------------------------------------------------------------------

FSTATUS NodeDataSwitchResizeMcastFDB(NodeData * nodep, uint32 newMfdbSize)
{
	int errStatus = FINSUFFICIENT_MEMORY;
	int i;
	STL_PORTMASK * newMfdb[STL_NUM_MFT_POSITIONS_MASK] = {NULL};
	assert(nodep->NodeInfo.NodeType == STL_NODE_SW && nodep->switchp && nodep->pSwitchInfo);

	STL_SWITCH_INFO * switchInfo = &nodep->pSwitchInfo->SwitchInfoData;

	STL_LID newMtop = switchInfo->MulticastFDBTop;

	if (newMfdbSize > 0) {
		newMfdbSize = MIN(newMfdbSize, switchInfo->MulticastFDBCap);

		for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i) {
			newMfdb[i] = (STL_PORTMASK*) MemoryAllocate2AndClear(newMfdbSize * sizeof(STL_PORTMASK),
				IBA_MEM_FLAG_PREMPTABLE, MYTAG);
			if (!newMfdb[i]) goto fail;
		}

		if (switchInfo->MulticastFDBTop >= STL_LID_MULTICAST_BEGIN) {
			size_t size = MIN(newMfdbSize, switchInfo->MulticastFDBTop - STL_LID_MULTICAST_BEGIN + 1);

			for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i) {
				if (!nodep->switchp->MulticastFDB[i])
					continue;

				memcpy(newMfdb[i], nodep->switchp->MulticastFDB[i], size * sizeof(STL_PORTMASK));
				newMtop = MIN((size - 1) + STL_LID_MULTICAST_BEGIN, newMtop);
			}
		}
	}

	if (newMfdbSize) {
		for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i) {
			if (nodep->switchp->MulticastFDB[i]) {
				MemoryDeallocate(nodep->switchp->MulticastFDB[i]);
				nodep->switchp->MulticastFDB[i] = NULL;
			}
		}

		memcpy(nodep->switchp->MulticastFDB, newMfdb, sizeof(newMfdb));
		switchInfo->MulticastFDBTop = newMtop;

		if (newMtop >= STL_LID_MULTICAST_BEGIN)
			nodep->switchp->MulticastFDBSize = newMtop + 1;
		else
			nodep->switchp->MulticastFDBSize = newMfdbSize + STL_LID_MULTICAST_BEGIN;

	}

	return FSUCCESS;
fail:
	for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i) {
		if (newMfdb[i]) {
			MemoryDeallocate(newMfdb[i]);
			newMfdb[i] = NULL;
		}
	}

	return errStatus;
}

FSTATUS NodeDataSwitchResizeLinearFDB(NodeData * nodep, uint32 newLfdbSize)
{
	int errStatus = FINSUFFICIENT_MEMORY;
	uint32 newPgdbSize;
	STL_LINEAR_FORWARDING_TABLE * newLfdb = NULL;
	STL_PORT_GROUP_FORWARDING_TABLE * newPgdb = NULL;
	DEBUG_ASSERT(nodep->NodeInfo.NodeType == STL_NODE_SW && nodep->switchp && nodep->pSwitchInfo);

	STL_SWITCH_INFO * switchInfo = &nodep->pSwitchInfo->SwitchInfoData;

	STL_LID newLtop = switchInfo->LinearFDBTop;

	if (newLfdbSize > 0 &&
		switchInfo->RoutingMode.Enabled == STL_ROUTE_LINEAR) {
		newLfdbSize = MIN(newLfdbSize, switchInfo->LinearFDBCap);
		newLfdb = (STL_LINEAR_FORWARDING_TABLE*) MemoryAllocate2AndClear(
			ROUNDUP(newLfdbSize, MAX_LFT_ELEMENTS_BLOCK), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

		if (!newLfdb) goto fail;

		memset(newLfdb, 0xff, ROUNDUP(newLfdbSize, MAX_LFT_ELEMENTS_BLOCK));
		if (nodep->switchp->LinearFDB) {
			size_t size = MIN(newLfdbSize, switchInfo->LinearFDBTop + 1);
			memcpy(newLfdb, nodep->switchp->LinearFDB, size * sizeof(PORT));
			newLtop = MIN(size - 1, newLtop);
		}

		// Port Group Forwarding table same as Linear FDB but capped
		// at PortGroupFDBCap (for early STL1 HW hardcoded cap of 8kb)
		newPgdbSize = MIN(newLfdbSize,
					  switchInfo->PortGroupFDBCap ? switchInfo->PortGroupFDBCap : DEFAULT_MAX_PGFT_LID+1);
		newPgdb = (STL_PORT_GROUP_FORWARDING_TABLE*) MemoryAllocate2AndClear(
			ROUNDUP(newPgdbSize, NUM_PGFT_ELEMENTS_BLOCK), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

		if (!newPgdb) goto fail;

		memset(newPgdb, 0xff, ROUNDUP(newPgdbSize, NUM_PGFT_ELEMENTS_BLOCK));
		if (nodep->switchp->PortGroupFDB) {
			size_t size = MIN(newPgdbSize, switchInfo->LinearFDBTop + 1);
			memcpy(newPgdb, nodep->switchp->PortGroupFDB, size * sizeof(PORT));
		}
	}

	if (newLfdb) {
		STL_LINEAR_FORWARDING_TABLE * oldLfdb = nodep->switchp->LinearFDB;
		nodep->switchp->LinearFDB = newLfdb;
		nodep->switchp->LinearFDBSize = newLfdbSize;
		switchInfo->LinearFDBTop = newLtop;
		MemoryDeallocate(oldLfdb);
	}

	if (newPgdb) {
		STL_PORT_GROUP_FORWARDING_TABLE * oldPgdb = nodep->switchp->PortGroupFDB;
		nodep->switchp->PortGroupFDB = newPgdb;
		nodep->switchp->PortGroupFDBSize = newPgdbSize;
		MemoryDeallocate(oldPgdb);
	}

	return FSUCCESS;
fail:
	if (newLfdb) {
		MemoryDeallocate(newLfdb);
		newLfdb = NULL;
	}

	if (newPgdb) {
		MemoryDeallocate(newPgdb);
		newPgdb = NULL;
	}

	return errStatus;
}

FSTATUS NodeDataSwitchResizeFDB(NodeData * nodep, uint32 newLfdbSize, uint32 newMfdbSize)
{
	if ((NodeDataSwitchResizeLinearFDB(nodep, newLfdbSize) != FSUCCESS) ||
		(NodeDataSwitchResizeMcastFDB(nodep, newMfdbSize) != FSUCCESS)) {
		fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

// remove Node from lists and free it
// Only removes node from AllNodes and SystemData.Nodes
// Only removes ports from AllLids and NodeData.Ports
// Only removes Iou from NodeData.ioup
// Only removes Iocs from AllIOCs and IouData.Iocs, caller must remove from
// any other lists if called after BuildFabricLists
void NodeDataFree(FabricData_t *fabricp, NodeData *nodep)
{
	if (nodep->context && g_Top_FreeCallbacks.pNodeDataFreeCallback)
		(*g_Top_FreeCallbacks.pNodeDataFreeCallback)(fabricp, nodep);

	cl_qmap_remove_item(&nodep->systemp->Nodes, &nodep->SystemNodesEntry);
	if (cl_qmap_count(&nodep->systemp->Nodes) == 0) {
		if (nodep->systemp->context && g_Top_FreeCallbacks.pSystemDataFreeCallback)
			(*g_Top_FreeCallbacks.pSystemDataFreeCallback)(fabricp, nodep->systemp);

		cl_qmap_remove_item(&fabricp->AllSystems, &nodep->systemp->AllSystemsEntry);
		MemoryDeallocate(nodep->systemp);
	}
	cl_qmap_remove_item(&fabricp->AllNodes, &nodep->AllNodesEntry);
	NodeDataFreePorts(fabricp, nodep);
#if !defined(VXWORKS) || defined(BUILD_DMC)
	if (nodep->ioup)
		IouDataFree(fabricp, nodep->ioup);
#endif
	if (nodep->pSwitchInfo)
		MemoryDeallocate(nodep->pSwitchInfo);
	NodeDataFreeSwitchData(fabricp, nodep);
	MemoryDeallocate(nodep);
}

// remove all Nodes from lists and free them
// Only removes nodes from AllNodes and SystemData.Nodes
// Only removes ports from AllLids and NodeData.Ports
// Only removes Iou from NodeData.ioup
// Only removes Iocs from AllIOCs and IouData.Iocs, caller must remove from
// any other lists if called after BuildFabricLists
void NodeDataFreeAll(FabricData_t *fabricp)
{
	cl_map_item_t *p;

	// this list is sorted/keyed by NodeGUID
	for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_head(&fabricp->AllNodes)) {
		NodeDataFree(fabricp, PARENT_STRUCT(p, NodeData, AllNodesEntry));
	}
}

// remove SM from lists and free it
// Only removes SM from AllSMs, caller must remove from
// any other lists if called after BuildFabricLists
void SMDataFree(FabricData_t *fabricp, SMData *smp)
{
	if (smp->context && g_Top_FreeCallbacks.pSMDataFreeCallback)
		(*g_Top_FreeCallbacks.pSMDataFreeCallback)(fabricp, smp);

	cl_qmap_remove_item(&fabricp->AllSMs, &smp->AllSMsEntry);
	MemoryDeallocate(smp);
}

// remove all SMs from lists and free them
// Only removes SMs from AllSMs, caller must remove from
// any other lists if called after BuildFabricLists
void SMDataFreeAll(FabricData_t *fabricp)
{
	cl_map_item_t *p;

	for (p=cl_qmap_head(&fabricp->AllSMs); p != cl_qmap_end(&fabricp->AllSMs); p = cl_qmap_head(&fabricp->AllSMs)) {
		SMDataFree(fabricp, PARENT_STRUCT(p, SMData, AllSMsEntry));
	}
}

void MCGroupFree(FabricData_t *fabricp, McGroupData *mcgroupp)
{
	LIST_ITEM *p;

	while (!QListIsEmpty(&mcgroupp->AllMcGroupMembers)) {
		p = QListTail(&mcgroupp->AllMcGroupMembers);
		McGroupData *group = (McGroupData *)QListObj(p);
		QListRemoveTail(&mcgroupp->AllMcGroupMembers);
		MemoryDeallocate(group);
	}

	QListRemoveItem(&fabricp->AllMcGroups, &mcgroupp->AllMcGMembersEntry);
	MemoryDeallocate(mcgroupp);
}

void MCDataFreeAll(FabricData_t *fabricp)
{
	LIST_ITEM *p;
	while (!QListIsEmpty(&fabricp->AllMcGroups)) {
		p = QListTail(&fabricp->AllMcGroups);
		MCGroupFree(fabricp, (McGroupData *)QListObj(p));
	}

}

void VFDataFreeAll(FabricData_t *fabricp)
{
	LIST_ITEM *i;
	for (i = QListHead(&fabricp->AllVFs); i;) {
		LIST_ITEM *next = QListNext(&fabricp->AllVFs, i);
		MemoryDeallocate(QListObj(i));
		i = next;
	}
}

void CableDataFree(CableData *cablep)
{
	if (cablep->length)
		MemoryDeallocate(cablep->length);
	if (cablep->label)
		MemoryDeallocate(cablep->label);
	if (cablep->details)
		MemoryDeallocate(cablep->details);
	cablep->length = cablep->label = cablep->details = NULL;
}

void PortSelectorFree(PortSelector *portselp)
{
	if (portselp->NodeDesc)
		MemoryDeallocate(portselp->NodeDesc);
	if (portselp->details)
		MemoryDeallocate(portselp->details);
	MemoryDeallocate(portselp);
}

// remove Link from lists and free it
void ExpectedLinkFree(FabricData_t *fabricp, ExpectedLink *elinkp)
{
	if (elinkp->portp1 && elinkp->portp1->elinkp == elinkp)
		elinkp->portp1->elinkp = NULL;
	if (elinkp->portp2 && elinkp->portp2->elinkp == elinkp)
		elinkp->portp2->elinkp = NULL;
	if (ListItemIsInAList(&elinkp->ExpectedLinksEntry))
		QListRemoveItem(&fabricp->ExpectedLinks, &elinkp->ExpectedLinksEntry);
	if (elinkp->portselp1)
		PortSelectorFree(elinkp->portselp1);
	if (elinkp->portselp2)
		PortSelectorFree(elinkp->portselp2);
	if (elinkp->details)
		MemoryDeallocate(elinkp->details);
	CableDataFree(&elinkp->CableData);
	MemoryDeallocate(elinkp);
}

// remove all Links from lists and free them
void ExpectedLinkFreeAll(FabricData_t *fabricp)
{
	LIST_ITEM *p;

	// free all link data
	for (p=QListHead(&fabricp->ExpectedLinks); p != NULL;) {
		LIST_ITEM *nextp = QListNext(&fabricp->ExpectedLinks, p);
		ExpectedLinkFree(fabricp, (ExpectedLink *)QListObj(p));
		p = nextp;
	}
}

// remove Expected Node from lists and free it
void ExpectedNodeFree(FabricData_t *fabricp, ExpectedNode *enodep, QUICK_LIST *listp)
{
	if (enodep->nodep && enodep->nodep->enodep == enodep)
		enodep->nodep->enodep = NULL;
	if (ListItemIsInAList(&enodep->ExpectedNodesEntry))
		QListRemoveItem(listp, &enodep->ExpectedNodesEntry);
	if (enodep->NodeGUID)
		cl_qmap_remove(&fabricp->ExpectedNodeGuidMap, enodep->NodeGUID);
	if (enodep->NodeDesc)
		MemoryDeallocate(enodep->NodeDesc);
	if (enodep->details)
		MemoryDeallocate(enodep->details);
	if (enodep->ports) {
		int i;
		for (i = 0; i < enodep->portsSize; ++i) {
			if (enodep->ports[i])
				MemoryDeallocate(enodep->ports[i]);
		}
		MemoryDeallocate(enodep->ports);
	}
	MemoryDeallocate(enodep);
}

// remove all Expected Nodes from lists and maps and free them
void ExpectedNodesFreeAll(FabricData_t *fabricp, QUICK_LIST *listp)
{
	LIST_ITEM *p;

	// free all link data
	for (p=QListHead(listp); p != NULL;) {
		LIST_ITEM *nextp = QListNext(listp, p);
		ExpectedNodeFree(fabricp, (ExpectedNode *)QListObj(p), listp);
		p = nextp;
	}
}

// remove Expected SM from lists and free it
void ExpectedSMFree(FabricData_t *fabricp, ExpectedSM *esmp)
{
	if (esmp->smp && esmp->smp->esmp == esmp)
		esmp->smp->esmp = NULL;
	if (ListItemIsInAList(&esmp->ExpectedSMsEntry))
		QListRemoveItem(&fabricp->ExpectedSMs, &esmp->ExpectedSMsEntry);
	if (esmp->NodeDesc)
		MemoryDeallocate(esmp->NodeDesc);
	if (esmp->details)
		MemoryDeallocate(esmp->details);
	MemoryDeallocate(esmp);
}

// remove all Expected SMs from lists and free them
void ExpectedSMsFreeAll(FabricData_t *fabricp)
{
	LIST_ITEM *p;

	// free all link data
	for (p=QListHead(&fabricp->ExpectedSMs); p != NULL;) {
		LIST_ITEM *nextp = QListNext(&fabricp->ExpectedSMs, p);
		ExpectedSMFree(fabricp, (ExpectedSM *)QListObj(p));
		p = nextp;
	}
}

void DestroyFabricData(FabricData_t *fabricp)
{
	if (fabricp->context && g_Top_FreeCallbacks.pFabricDataFreeCallback)
		(*g_Top_FreeCallbacks.pFabricDataFreeCallback)(fabricp);

	ExpectedLinkFreeAll(fabricp);	// ExpectedLinks
	ExpectedNodesFreeAll(fabricp, &fabricp->ExpectedFIs);	// ExpectedFIs
	ExpectedNodesFreeAll(fabricp, &fabricp->ExpectedSWs);	// ExpectedSWs
	ExpectedSMsFreeAll(fabricp);	// ExpectedSMs

	SMDataFreeAll(fabricp); // SMs
	NodeDataFreeAll(fabricp);	// Nodes, Ports, IOUs, Systems
	VFDataFreeAll(fabricp);

	if (fabricp->flags & FF_LIDARRAY)
		FreeLidMap(fabricp);

	// make sure no stale pointers in lists, etc
	// also clear counters and flags
	MemoryClear(fabricp, sizeof(*fabricp));
}

#ifndef __VXWORKS__

static uint8 CLListUint32Find(cl_qmap_t *arrayMap, uint32 entry) 
{ 
   uint8 found = 0; 
   cl_map_item_t *mi; 
   
   mi = cl_qmap_get(arrayMap, entry); 
   if (mi != cl_qmap_end(arrayMap)) {
      found = 1;;
   }
   return found;
}

static uint8 CLListIntFind(cl_qmap_t *arrayMap, int entry) 
{ 
   uint8 found = 0; 
   cl_map_item_t *mi; 
   
   mi = cl_qmap_get(arrayMap, entry); 
   if (mi != cl_qmap_end(arrayMap)) {
      found = 1;;
   }
   return found;
}

static FSTATUS CLListUint32Add(cl_qmap_t *arrayMap, uint32 entry) 
{ 
   FSTATUS status = FSUCCESS; 
   cl_map_item_t *mi; 
   clListSearchData_t *clListp; 
   
   if (CLListUint32Find(arrayMap, entry)) 
      return status; 
   
   clListp = (clListSearchData_t *)MemoryAllocate2AndClear(sizeof(clListSearchData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);        
   if (!clListp) {
      status = FINSUFFICIENT_MEMORY; 
      fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
   } else {
      mi = cl_qmap_insert(arrayMap, entry, &clListp->AllListEntry); 
      if (mi != &clListp->AllListEntry) {
         fprintf(stderr, "%s: Duplicate entry found in numeric list map: %d\n", g_Top_cmdname, entry); 
         MemoryDeallocate(clListp); 
         clListp = PARENT_STRUCT(mi, clListSearchData_t, AllListEntry);
      }
   }
   
   return status;
}

static FSTATUS CLListIntAdd(cl_qmap_t *arrayMap, int entry) 
{ 
   FSTATUS status = FSUCCESS; 
   cl_map_item_t *mi; 
   clListSearchData_t *clListp; 
   
   if (CLListIntFind(arrayMap, entry)) 
      return status; 
   
   clListp = (clListSearchData_t *)MemoryAllocate2AndClear(sizeof(clListSearchData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);        
   if (!clListp) {
      status = FINSUFFICIENT_MEMORY; 
      fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
   } else {
      mi = cl_qmap_insert(arrayMap, entry, &clListp->AllListEntry); 
      if (mi != &clListp->AllListEntry) {
         fprintf(stderr, "%s: Duplicate entry found in numeric list map: %d\n", g_Top_cmdname, entry); 
         MemoryDeallocate(clListp); 
         clListp = PARENT_STRUCT(mi, clListSearchData_t, AllListEntry);
      }
   }
   
   return status;
}

//PYTHON: def ib_connection_source_to_str (conn) :
//NOTA BENE: NOTE THAT THIS FUNCTION IS NOT THREAD SAFE!
static char* ib_connection_source_to_str(FabricData_t *fabricp, clConnData_t *connp) 
{ 
   cl_map_item_t *mi; 
   clDeviceData_t * device1p,*device2p; 
   static char str[2*NODE_DESCRIPTION_ARRAY_SIZE+11]; 
   
   /*
   return '%s port %d' % \
          (fabric.map_guid_to_ib_device[conn.guid1].name, conn.port1)
   */
   mi = cl_qmap_get(&fabricp->map_guid_to_ib_device, connp->FromDeviceGUID); 
   if (mi == cl_qmap_end(&fabricp->map_guid_to_ib_device)) 
      return NULL; 
   device1p = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
   
   mi = cl_qmap_get(&fabricp->map_guid_to_ib_device, connp->ToDeviceGUID); 
   if (mi == cl_qmap_end(&fabricp->map_guid_to_ib_device)) 
      return NULL; 
   device2p = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
   
   snprintf(str, 2*NODE_DESCRIPTION_ARRAY_SIZE+10, "%s:%d to %s:%d VL:%d",
           device1p->nodep->NodeDesc.NodeString, connp->FromPortNum, 
           device2p->nodep->NodeDesc.NodeString, connp->ToPortNum,
           connp->VL.VL); 
   return str;
}

static void CLThreadSleep(uint64_t sleep_time) 
{ 
   struct timespec   ts; 
   struct timespec   ts_actual; 
   long              micro_seconds; 
   
   ts.tv_sec       = (time_t)(sleep_time / 1000000ULL); 
   micro_seconds   = (long)(sleep_time % 1000000ULL); 
   ts.tv_nsec      = micro_seconds * 1000; 
   (void)nanosleep(&ts, &ts_actual); 
   
   return;
}

static clArcData_t* CLGraphFindIdArc(clGraphData_t *graphp, uint32 arcId) 
{ 
   cl_map_item_t *mi; 
   
   mi = cl_qmap_get(&graphp->Arcs, arcId); 
   if (mi != cl_qmap_end(&graphp->Arcs)) {
      clArcData_t *arcp = PARENT_STRUCT(mi, clArcData_t, AllArcIdsEntry); 
      return arcp;
   }
   
   return NULL;
}

void CLGraphDataFree(clGraphData_t *graphp, void *context) 
{ 
   #define DEF_MAX_FREE_ENTRY   5000
   int i; 
   cl_map_item_t *mi;
   uint32_t arcListCount, arcCount = 0; 
   ValidateCreditLoopRoutesContext_t *cp = (ValidateCreditLoopRoutesContext_t *)context; 
   
   
   if (!graphp) 
      return;
    
   // free all arc entries
   arcListCount = cl_qmap_count(&graphp->Arcs);
   for (mi = cl_qmap_head(&graphp->Arcs); 
        mi != cl_qmap_end(&graphp->Arcs); 
        mi = cl_qmap_head(&graphp->Arcs), arcCount++) {
      clArcData_t *arcp; 
      
      if (NULL != (arcp = PARENT_STRUCT(mi, clArcData_t, AllArcIdsEntry))) {
         cl_qmap_remove_item(&graphp->Arcs, &arcp->AllArcIdsEntry); 
         MemoryDeallocate(arcp);
      }
      if (arcListCount >= DEF_MAX_FREE_ENTRY) {
         if (arcCount % PROGRESS_FREQ == 0 || arcCount == 0) {
            if (!cp->quiet) 
               ProgressPrint(FALSE, "Deallocated %6d of %6d Arcs...", arcCount, arcListCount);
         }
      }
   }

   // clear line used to display progress report
   if (!cp->quiet && arcListCount >= DEF_MAX_FREE_ENTRY)
      ProgressPrint(TRUE, "Done Deallocating All Arcs");
   
   // free all vertices associated with the graph
   if (graphp->Vertices) {
      for (i = 0; i < graphp->NumVertices; i++) {
         if (graphp->Vertices[i]) {
            if (graphp->Vertices[i]->Inbound)
               MemoryDeallocate(graphp->Vertices[i]->Inbound);
            if (graphp->Vertices[i]->Outbound)
               MemoryDeallocate(graphp->Vertices[i]->Outbound);
            MemoryDeallocate(graphp->Vertices[i]);
         }
         if (graphp->NumVertices >= DEF_MAX_FREE_ENTRY) {
            if (i % PROGRESS_FREQ == 0 || i == 0) {
               if (!cp->quiet) 
                  ProgressPrint(FALSE, "Deallocated %6d of %6d Vertices...", i, graphp->NumVertices);
            }
         }
      }

      // clear line used to display progress report
      if (!cp->quiet && graphp->NumVertices >= DEF_MAX_FREE_ENTRY)
         ProgressPrint(TRUE, "Done Deallocating All Vertices");
      MemoryDeallocate(graphp->Vertices);
   }
}

static void CLDeviceDataFreeAll(FabricData_t *fabricp, void *context) 
{
   #define DEF_MAX_FREE_DEV   5000
   int i, j;
   cl_map_item_t * mi,*ri; 
   uint32 deviceCount = 0, deviceListCount, routeCount = 0;
// uint32 routeListCount;
   ValidateCreditLoopRoutesContext_t *cp = (ValidateCreditLoopRoutesContext_t *)context; 
   
   deviceListCount = cl_qmap_count(&fabricp->map_guid_to_ib_device);
   // free all device entries
   for (mi = cl_qmap_head(&fabricp->map_guid_to_ib_device); 
        mi != cl_qmap_end(&fabricp->map_guid_to_ib_device); 
        mi = cl_qmap_head(&fabricp->map_guid_to_ib_device), deviceCount++) {
      clDeviceData_t *ccDevicep; 
      
      if (NULL == (ccDevicep = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry)))
         break;
      else {
         cl_qmap_remove_item(&fabricp->map_guid_to_ib_device, &ccDevicep->AllDevicesEntry); 
         // free all connections associated with the device
         for (i = 0; i < CREDIT_LOOP_DEVICE_MAX_CONNECTIONS; i++) {
            for (j = 0; j < STL_MAX_VLS; j++) {
               if (ccDevicep->Connections[i][j])
                  MemoryDeallocate(ccDevicep->Connections[i][j]);
            }
         }
         
//       routeListCount = cl_qmap_count(&ccDevicep->map_dlid_to_route);
         // free all routes associated with the device
         for (ri = cl_qmap_head(&ccDevicep->map_dlid_to_route), routeCount = 0; 
              ri != cl_qmap_end(&ccDevicep->map_dlid_to_route); 
              ri = cl_qmap_head(&ccDevicep->map_dlid_to_route), routeCount++) {
            clRouteData_t *ccRoutep; 
            
            if (NULL == (ccRoutep = PARENT_STRUCT(ri, clRouteData_t, AllRoutesEntry)))
               break;
            else {
               cl_qmap_remove_item(&ccDevicep->map_dlid_to_route, &ccRoutep->AllRoutesEntry); 
               MemoryDeallocate(ccRoutep);
            }

            /* 
              if (routeCount % PROGRESS_FREQ == 0 || routeCount == 0) {
               if (!cp->quiet) 
                  ProgressPrint(FALSE, "Deallocated %6d of %6d Routes...", routeCount, routeListCount);
            } 
            */ 
         }
         
         if (deviceListCount >= DEF_MAX_FREE_DEV) {
            if (deviceCount % PROGRESS_FREQ == 0 || deviceCount == 0) {
               if (!cp->quiet) 
                  ProgressPrint(FALSE, "Deallocated %6d of %6d Devices...", deviceCount, deviceListCount);
            }
         }

         // free device
         MemoryDeallocate(ccDevicep);
      }
   }

   // clear line used to display progress report
   if (!cp->quiet && deviceListCount >= DEF_MAX_FREE_DEV)
      ProgressPrint(TRUE, "Done Deallocating All Routes");
}

static FSTATUS CLGetDynamicArray(void **array, uint32 *arrayEntries, uint32 entryLength, uint32 newEntryIndice, int verbose) 
{ 
   #define ARRAY_INCR_NUM_ENTRIES  1000
   
   FSTATUS status = FERROR; 
   
   uint32 newEntries; 
   void *newBufferp; 
   
   
   if (array && arrayEntries) {
      status = FSUCCESS; 
      if (!*array || newEntryIndice >= *arrayEntries) {
         if (verbose >= 6) 
            printf("Old size: array=%p, *array=%p, arrayEntries=%d, entryLength=%d , newEntryIndice=%d\n", 
                   array, *array, *arrayEntries, entryLength, newEntryIndice); 
         
         // array has not been initialized, set entries to default values
         if (!*array) {
            //printf("\tALLOC:\n");
            *arrayEntries = 0;
         }
         
         // extend the buffer by an addition fixed number of entries  
         newEntries = *arrayEntries + ARRAY_INCR_NUM_ENTRIES; 
         newBufferp = MemoryAllocate2AndClear(newEntries * entryLength, IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
         if (!newBufferp) {
            status = FINSUFFICIENT_MEMORY; 
            fprintf(stderr, "%s: Unable to resize the buffer\n", g_Top_cmdname);
         } else {
            // replicate the old array into the new array
            if (*array) {
               //printf("\tRESIZE:\n");
               memcpy(newBufferp, *array, (*arrayEntries * entryLength)); 
               MemoryDeallocate(*array);
            }
            // point to new array
            *array = newBufferp; 
            *arrayEntries = newEntries; 
            //status = FSUCCESS;
         }
         if (verbose >= 6) 
            printf("New size: array=%p, arrayEntries=%d, array size=%d\n", *array, *arrayEntries, (*arrayEntries * entryLength));
      }
   }
   
   return status;
}

static clDeviceData_t* CLFindGuidDevice(FabricData_t *fabricp, uint64 guid) 
{ 
   cl_map_item_t *mi; 
   
   mi = cl_qmap_get(&fabricp->map_guid_to_ib_device, guid); 
   if (mi != cl_qmap_end(&fabricp->map_guid_to_ib_device)) {
      clDeviceData_t *ccDevicep = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
      return ccDevicep;
   }
   return NULL;
}

//PYTHON: def lookup_vertex (self, ref) :
static clVertixData_t* CLGraphFindConnVertex(clGraphData_t *graphp, clConnData_t *ccConnp) 
{ 
   cl_map_item_t *mi; 
   clVertixData_t *ccVertexp = NULL; 
   
   if (graphp->Vertices) {
      mi = cl_qmap_get(&graphp->map_conn_to_vertex_conn, (uint64)ccConnp); 
      if (mi != cl_qmap_end(&graphp->map_conn_to_vertex_conn)) {
         ccVertexp = PARENT_STRUCT(mi, clVertixData_t, AllVerticesEntry);
      }
   }
   return ccVertexp;
}

//PYTHON: def add_vertex (self, ref) :
static int CLGraphDataAddVertex(clGraphData_t *graphp, clConnData_t *ccConnp, int verbose) 
{ 
   cl_map_item_t *mi; 
   
   //PYTHON: vertex = self.map_ref_to_vertex.get(ref)
   /* CJKING: Addtional thought required on how to map a connection
    * to a Vertex.  Currently, doing linear search via CLGraphFindConnVertex()
    */
   clVertixData_t *ccVertexp = CLGraphFindConnVertex(graphp, ccConnp); 
   
   //PYTHON: if vertex :
   if (ccVertexp) {
      if (verbose >= 5) 
         fprintf(stderr, "Warning, Vector Id %d already exist\n", ccVertexp->Id); 
      return ccVertexp->Id;
   } else {
      ccVertexp = (clVertixData_t *)MemoryAllocate2AndClear(sizeof(clVertixData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);        
      if (!ccVertexp) {
         fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
      } else {
         //PYTHON: id = len(self.vertices)
         //PYTHON: vertex = Vertex(id, ref)
         ccVertexp->Id = graphp->NumVertices; 
         ccVertexp->Connection = ccConnp; 
         if (!graphp->Vertices) 
            cl_qmap_init(&graphp->map_conn_to_vertex_conn, NULL); 
         if (!CLGetDynamicArray((void **)&graphp->Vertices, &graphp->VerticesLength, 
                                sizeof(graphp->Vertices[0]), ccVertexp->Id, verbose)) {
            //PYTHON: self.vertices.append(vertex)
            graphp->Vertices[ccVertexp->Id] = ccVertexp; 
            //PYTHON: self.map_ref_to_vertex[ref] = vertex
            mi = cl_qmap_insert(&graphp->map_conn_to_vertex_conn, (uint64)ccConnp, &ccVertexp->AllVerticesEntry); 
            if (mi != &ccVertexp->AllVerticesEntry) {
               //if (verbose >= 4) {
               fprintf(stderr, "%s: Duplicate connection found in vertix map: %p\n", g_Top_cmdname, ccVertexp->Connection); 
               //}
               MemoryDeallocate(ccVertexp); 
               ccVertexp = PARENT_STRUCT(mi, clVertixData_t, AllVerticesEntry);
            }
            //PYTHON: self.num_vertices += 1
            graphp->NumVertices++; 
            graphp->NumActiveVertices++; 
            if (verbose >= 4) 
               printf("Adding vector Id %d\n", ccVertexp->Id); 
            return ccVertexp->Id;
         } else {
            MemoryDeallocate(ccVertexp);
         }
      }
   }
   
   return -1;
}

//PYTHON: def get_arc_key (source, sink) :
static clArcData_t* CLGraphFindSrcSinkArc(clGraphData_t *graphp, uint32 source, uint32 sink) 
{ 
   cl_map_item_t *mi; 
   clArcData_t *arcp = NULL; 
   
   if (cl_qmap_count(&graphp->map_arc_key_to_arc)) {
      clArcData_t arc; 
      
      arc.u1.s.Source = source; 
      arc.u1.s.Sink = sink; 
      mi = cl_qmap_get(&graphp->map_arc_key_to_arc, arc.u1.AsReg64); 
      if (mi != cl_qmap_end(&graphp->map_arc_key_to_arc)) {
         arcp = PARENT_STRUCT(mi, clArcData_t, AllArcsEntry);
      }
   }
   return arcp;
}

//PYTHON: def del_arc (self, id) :
static void CLGraphDataDelArc(clGraphData_t *graphp, int arcId) 
{ 
   uint32 i; 
   //LIST_ITEM *p;
   cl_map_item_t *mi; 
   clArcData_t *ccArcp = NULL; 
   
   
   if (arcId < 0) 
      return; 
   
   
   //PYTHON: arc = self.arcs[id]
   mi = cl_qmap_get(&graphp->Arcs, arcId); 
   if (mi != cl_qmap_end(&graphp->Arcs)) {
      ccArcp = PARENT_STRUCT(mi, clArcData_t, AllArcIdsEntry); 
      if (ccArcp) {
         clArcData_t *ssArcp; 
         
         // remove arc entry from arc IDs map
         cl_qmap_remove_item(&graphp->Arcs, &ccArcp->AllArcIdsEntry); 
         // remove arc entry from arc Source/Sink map
         if ((ssArcp = CLGraphFindSrcSinkArc(graphp, ccArcp->Source, ccArcp->Sink))) 
            cl_qmap_remove_item(&graphp->map_arc_key_to_arc, &ssArcp->AllArcsEntry);
      }
   }
   
   if (!ccArcp) {
      fprintf(stderr, "Warning, failed to find arc with id %d\n", arcId); 
      exit(0);
   } else {
      /*
       * process up-link references
       */
      //PYTHON: self.vertices[arc.source].outbound.remove(id)
      for (i = 0; i < graphp->Vertices[ccArcp->Source]->OutboundCount; i++) {
         if (arcId == graphp->Vertices[ccArcp->Source]->Outbound[i]) {
            graphp->Vertices[ccArcp->Source]->Outbound[i] = -1; 
            //PYTHON: self.vertices[arc.source].refcount -= 1
            graphp->Vertices[ccArcp->Source]->RefCount--; 
            graphp->Vertices[ccArcp->Source]->OutboundInuseCount--;
         }
      }
      //PYTHON: if self.vertices[arc.source].refcount == 0 :        
      if (graphp->Vertices[ccArcp->Source]->RefCount == 0) 
         graphp->NumActiveVertices--;       //PYTHON:   self.num_vertices -= 1
      
      /*
       * remove down-link references
       */ 
      //PYTHON: self.vertices[arc.sink].inbound.remove(id)
      for (i = 0; i < graphp->Vertices[ccArcp->Sink]->InboundCount; i++) {
         if (arcId == graphp->Vertices[ccArcp->Sink]->Inbound[i]) {
            graphp->Vertices[ccArcp->Sink]->Inbound[i] = -1; 
            //PYTHON: self.vertices[arc.sink].refcount -= 1
            graphp->Vertices[ccArcp->Sink]->RefCount--; 
            graphp->Vertices[ccArcp->Sink]->InboundInuseCount--;
         }
      }
      //PYTHON: if self.vertices[arc.sink].refcount == 0 :
      if (graphp->Vertices[ccArcp->Sink]->RefCount == 0) 
         graphp->NumActiveVertices--;   //PYTHON: self.num_vertices -= 1
      
      //PYTHON: self.num_arcs -= 1
      graphp->NumArcs--; 
      MemoryDeallocate(ccArcp);
   }
}

//PYTHON: def add_arc (self, source, sink) :
static int CLGraphDataAddArc(clGraphData_t *graphp, uint32 source, uint32 sink, int verbose) 
{ 
   int arcId = -1; 
   cl_map_item_t *mi; 
   
   
   //PYTHON: if source == sink :
   if (source == sink) 
      fprintf(stderr, "Warning, Ignoring arc with same source and sink of %d\n", source); 
   else if (source >= graphp->NumVertices)   //PYTHON: if source < 0 or source >= len(self.vertices) :
      fprintf(stderr, "Warning, Ignoring out of range source vertex %d\n", source); 
   else if (sink >= graphp->NumVertices)       //PYTHON: if sink < 0 or sink >= len(self.vertices) :
      fprintf(stderr, "Warning, Ignoring out of range sink vertex %d\n", sink); 
   else {
      //PYTHON: arc_key = get_arc_key(source, sink)
      clArcData_t *ccArcp = CLGraphFindSrcSinkArc(graphp, source, sink); 
      
      if (ccArcp) {
         //PYTHON: id = self.map_arc_key_to_arc[arc_key]
         arcId = ccArcp->Id; 
         if (verbose >= 6) 
            fprintf(stderr, "Warning, arc %d from %d to %d already exists\n", arcId, source, sink);
      } else {
         ccArcp = (clArcData_t *)MemoryAllocate2AndClear(sizeof(clArcData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);        
         if (!ccArcp) {
            fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
         } else {
            FSTATUS status; 
            
            //PYTHON: id = len(self.arcs)
            ccArcp->Id = cl_qmap_count(&graphp->Arcs); 
            //PYTHON: self.arcs.append(Arc(self, source, sink))
            ccArcp->Source = source; 
            ccArcp->Sink = sink; 
            mi = cl_qmap_insert(&graphp->Arcs, ccArcp->Id, &ccArcp->AllArcIdsEntry); 
            if (mi != &ccArcp->AllArcIdsEntry) {
               //if (verbose >= 4) {
               fprintf(stderr, "%s: Duplicate connection found in arc ID map: %d\n", g_Top_cmdname, ccArcp->Id); 
               //}
               //MemoryDeallocate(ccArcp);
               //ccArcp = PARENT_STRUCT(mi, clArcData_t, AllArcsEntry);
            }
            //PYTHON: self.map_arc_key_to_arc[arc_key] = id
            ccArcp->u1.s.Source = source; 
            ccArcp->u1.s.Sink = sink; 
            mi = cl_qmap_insert(&graphp->map_arc_key_to_arc, ccArcp->u1.AsReg64, &ccArcp->AllArcsEntry); 
            if (mi != &ccArcp->AllArcsEntry) {
               //if (verbose >= 4) {
               fprintf(stderr, "%s: Duplicate connection found in arc source-sink map: 0x%016"PRIx64"\n", g_Top_cmdname, ccArcp->u1.AsReg64); 
               //}
               MemoryDeallocate(ccArcp); 
               ccArcp = PARENT_STRUCT(mi, clArcData_t, AllArcsEntry);
            }
            
            if (!(status = CLGetDynamicArray((void **)&graphp->Vertices[source]->Outbound, 
                                             &graphp->Vertices[source]->OutboundLength, 
                                             sizeof(int), graphp->Vertices[source]->OutboundCount, verbose))) {
               //PYTHON: self.vertices[source].outbound.append(id)
               graphp->Vertices[source]->Outbound[graphp->Vertices[source]->OutboundCount] = ccArcp->Id; 
               graphp->Vertices[source]->OutboundCount++; 
               graphp->Vertices[source]->OutboundInuseCount++; 
               //PYTHON: self.vertices[source].refcount += 1
               graphp->Vertices[source]->RefCount++;
            }
            
            if (!status) {
               if (!(status = CLGetDynamicArray((void **)&graphp->Vertices[sink]->Inbound, 
                                                &graphp->Vertices[sink]->InboundLength, 
                                                sizeof(int), graphp->Vertices[sink]->InboundCount, verbose))) {
                  //PYTHON: self.vertices[sink].inbound.append(id)
                  graphp->Vertices[sink]->Inbound[graphp->Vertices[sink]->InboundCount] = ccArcp->Id; 
                  graphp->Vertices[sink]->InboundCount++; 
                  graphp->Vertices[sink]->InboundInuseCount++; 
                  //PYTHON: self.vertices[sink].refcount += 1
                  graphp->Vertices[sink]->RefCount++;
               }
            }
            
            if (status) 
               (void)CLGraphDataDelArc(graphp, ccArcp->Id); 
            else {
               arcId = ccArcp->Id; 
               //PYTHON: self.num_arcs += 1
               graphp->NumArcs++; 
               if (verbose >= 4) 
                  printf("Add arc from %d to %d\n", source, sink);
            }
         }
      }
   }
   
   return arcId;
}

//PYTHON: def build_routing_graph (fabric) :
static void* CLFabricDataBuildRouteGraphThread(void *context) 
{ 
   FabricData_t *fabricp = ((clThreadContext_t *)context)->fabricp; 
   int verbose = ((clThreadContext_t *)context)->verbose; 
   FSTATUS status = FSUCCESS; 
   uint32 hops = 0, l = 0; 
   LIST_ITEM *dstHcaLstp;
   LIST_ITEM *srcHcaLstp; 
   uint32 thrdId = ((clThreadContext_t *)context)->threadId; 
   ValidateCLTimeGetCallback_t timeGetCallback = ((clThreadContext_t *)context)->timeGetCallback;
   uint32 usedSLs = ((clThreadContext_t*)context)->usedSLs;
   
   //PYTHON: for src_hfi in fabric.hfis :
   for (srcHcaLstp = ((clThreadContext_t *)context)->srcHcaList;
         l < ((clThreadContext_t *)context)->srcHcaListLength;
        srcHcaLstp = QListNext(&fabricp->FIs, srcHcaLstp), l++) {
      clDeviceData_t *src_hfip = (clDeviceData_t *)QListObj(srcHcaLstp); 
      
      if (verbose >= 3) {
         timeGetCallback(&((clThreadContext_t *)context)->sTime, &g_cl_lock); 
         printf("[%d] START build all routes from %s (SLID 0x%08x)\n", 
                (int)thrdId, src_hfip->nodep->NodeDesc.NodeString, src_hfip->Lid);
      }
      
      //PYTHON: for dst_hfi in fabric.hfis :
      for (dstHcaLstp = QListHead(&fabricp->FIs); dstHcaLstp != NULL; dstHcaLstp = QListNext(&fabricp->FIs, dstHcaLstp)) {
         clDeviceData_t *dst_hfip = (clDeviceData_t *)QListObj(dstHcaLstp); 
         
         if (srcHcaLstp != dstHcaLstp) {
            uint8 sl;
            for (sl = 0; sl < STL_MAX_SLS; sl++) { // loop over SLs
               if (usedSLs != 0) {
                  if (!((usedSLs >> sl) & 1)) {
                     continue;
                  }
               } else {
                  if (sl != 0) continue;
               }
               uint32 links;
               uint16 slid, dlid;
               int this_vertex_id;
               uint32 previous_vertex_id = 0;              //PYTHON: previous_vertex_id = None
               clDeviceData_t *hfip = src_hfip;            //PYTHON: hfi = src_hfip;
               clConnData_t *this_connection = NULL;
               clConnData_t *previous_connection = NULL;  //PYTHON: previous_connection = None
               uint8 sc = 0, previous_sc = 0;
            
               //PYTHON: slid = src_hfi.lid
               //PYTHON: dlid = dst_hfi.lid
               //PYTHON: links = 0
               slid = src_hfip->Lid;
               dlid = dst_hfip->Lid;
               links = 0; 
               if (verbose >= 4)
                  printf("[%d]Build route from %s to %s (SLID 0x%04x to DLID 0x%04x):\n",
                         (int)thrdId, src_hfip->nodep->NodeDesc.NodeString, dst_hfip->nodep->NodeDesc.NodeString, slid, dlid); 

               // get global lock
               pthread_mutex_lock(&g_cl_lock); 

               //PYTHON: while hfi != dst_hfi :
               while (hfip != dst_hfip) {
                  cl_map_item_t *mi; 
                  PortData *portp = NULL; 
                  clRouteData_t *ccRoutep = NULL; 
                  //PYTHON: if dlid in hfi.routes :
                  mi = cl_qmap_get(&hfip->map_dlid_to_route, dlid);
                  if (mi != cl_qmap_end(&hfip->map_dlid_to_route))
                     ccRoutep = PARENT_STRUCT(mi, clRouteData_t, AllRoutesEntry);

                  if (ccRoutep) {
                     //PYTHON: port = hfi.routes[dlid]
                     portp = ccRoutep->portp;
                     if (verbose >= 4)
                        printf("  [%d] GUID 0x%016"PRIx64": use port %3.3u\n", (int)thrdId, hfip->nodep->NodeInfo.NodeGUID, portp->PortNum);
                  } else {
                     if (verbose >= 4)
                        printf("  [%d] GUID 0x%016"PRIx64": no route to DLID 0x%04x\n", (int)thrdId, hfip->nodep->NodeInfo.NodeGUID, dlid); 
                     break;
                  }
                  // look up the sc
                  if (usedSLs) {
                     // if this is the 1st hop, use the SL2SC table
                     if (hfip == src_hfip) {
                        if (portp->pQOS && portp->pQOS->SL2SCMap) {
                           sc = portp->pQOS->SL2SCMap->SLSCMap[sl].SC;
                        }
                     } else if (previous_connection) {
                     // SC to SC' transition
                        if (!hfip->nodep) {
                           fprintf(stderr, "[%d] Error, no node data found for GUID 0x%016"PRIx64"\n", (int)thrdId, previous_connection->ToDeviceGUID);
                           status = FERROR;
                           break;
                        }
                        if (hfip->nodep->NodeInfo.NodeType == STL_NODE_SW) { // should always be a switch
                           mi = cl_qmap_get(&hfip->nodep->Ports, previous_connection->ToPortNum);
                           if (mi == cl_qmap_end(&hfip->nodep->Ports)) {
                              fprintf(stderr, "[%d] Error, unable to find port %d for GUID 0x%016"PRIx64"\n", (int)thrdId, previous_connection->ToPortNum, previous_connection->ToDeviceGUID);
                              status = FERROR;
                              break;
                           }
                           PortData *prev_port = PARENT_STRUCT(mi, PortData, NodePortsEntry);
                           if (prev_port && prev_port->pQOS && !(QListIsEmpty(&prev_port->pQOS->SC2SCMapList[0]))){
                             STL_SCSCMAP *pSCSC = QOSDataLookupSCSCMap(prev_port, portp->PortNum, 0);
                             if (pSCSC) {
                                 sc = pSCSC->SCSCMap[previous_sc].SC;
                              }
                           }
                        }
                     }
                  }
                  //PYTHON: this_connection = hfi.connections[port]
                  this_connection = hfip->Connections[portp->PortNum][sc];
                  if (!this_connection){ // no connection found for this port/vl, broken route
                     fprintf(stderr, "[%d] Warning, broken route for GUID 0x%016"PRIx64" port %3.3u\n", (int)thrdId, hfip->nodep->NodeInfo.NodeGUID, portp->PortNum);
                     break;
                  }
 
                  //PYTHON: graph.add_vertex(this_connection)
                  if (-1 == (this_vertex_id = CLGraphDataAddVertex(&fabricp->Graph, this_connection, verbose))) {
                     break;
                  }
                  //PYTHON: if previous_connection :
                  if (previous_connection) {
                     //PYTHON: graph.add_arc(previous_vertex_id, this_vertex_id)
                     if (-1 == CLGraphDataAddArc(&fabricp->Graph, previous_vertex_id, this_vertex_id, verbose)) {
                        status = FERROR; 
                        break;
                     }
                  }

                  //PYTHON: guid = this_connection.guid2
                  //PYTHON: hfi = fabric.map_guid_to_ib_device[guid]
                  if (!(hfip = CLFindGuidDevice(fabricp, this_connection->ToDeviceGUID))) {
                     fprintf(stderr, "[%d] Error, no device found for GUID 0x%016"PRIx64" to DLID 0x%04x\n", (int)thrdId, this_connection->ToDeviceGUID, dlid); 
                     status = FERROR;
                     break;
                  }

                  //PYTHON: links += 1
                  //PYTHON: previous_connection = this_connection
                  //PYTHON: previous_vertex_id = this_vertex_id
                  links++;
                  previous_connection = this_connection;
                  previous_vertex_id = this_vertex_id;
                  previous_sc = sc;
               } // end while loop

               // release global lock
               pthread_mutex_unlock(&g_cl_lock);

               //PYTHON: if hfi == dst_hfi :
               if (hfip == dst_hfip) {
                  //PYTHON: present_routes += 1
                  //PYTHON: hops = links - 1
                  present_routes++;
                  hops = links - 1;

                  if (verbose >= 4)
                     printf("  [%d] %d links traversed, %d hops\n", (int)thrdId, links, hops);

                  // get global lock
                  pthread_mutex_lock(&g_cl_lock);

                  if (!CLGetDynamicArray((void **)&hops_histogram, &hops_histogram_length, sizeof(uint32), hops, verbose)) {
                     //PYTHON: if hops in hops_histogram
                     //CJKING: if (CLListUint32Find(hops, hops_histogram, hops_histogram_entries, verbose))
                     if (CLListUint32Find(&hopsHistogramLstMap, hops))
                        hops_histogram[hops]++;    //PYTHON: hops_histogram[hops] += 1
                     else {
                        //CJKING: Add entry to search list for hops_histogram array
                        if (!CLListUint32Add(&hopsHistogramLstMap, hops))
                           hops_histogram[hops] = 1;  //PYTHON: histogram[hops] = 1
                        else {
                           status = FERROR;
                           pthread_mutex_unlock(&g_cl_lock);
                           break;
                        }
                     }
                     hops_histogram_entries = MAX(hops_histogram_entries, hops);
                  }

                  // release global lock
                  pthread_mutex_unlock(&g_cl_lock);
                  } else {
                  missing_routes += 1;    //PYTHON: missing_routes += 1
               }
            }
         }
      }
      
      if (verbose >= 3) {
         timeGetCallback(&((clThreadContext_t *)context)->eTime, &g_cl_lock); 
         printf("[%d] END build all routes from %s (SLID 0x%08x); elapsed time(usec)=%d, (sec)=%d\n", 
                (int)thrdId, src_hfip->nodep->NodeDesc.NodeString, src_hfip->Lid, 
                (int)(((clThreadContext_t *)context)->eTime - ((clThreadContext_t *)context)->sTime), 
                ((int)(((clThreadContext_t *)context)->eTime - ((clThreadContext_t *)context)->sTime)) / CL_TIME_DIVISOR); 
         
         printf("[%d] %d of %d nodes completed\n", 
                 (int)thrdId, l+1, ((clThreadContext_t *)context)->srcHcaListLength);
      } else {
	 if (l%PROGRESS_FREQ == 0 || l == 0) {
            // get global lock
            pthread_mutex_lock(&g_cl_lock); 
            if (!((clThreadContext_t *)context)->quiet)
               ProgressPrint(FALSE, "[%d]Processed %6d of %6d Routes...",
                               (int)thrdId, l+1, ((clThreadContext_t *)context)->srcHcaListLength);
            // release global lock
            pthread_mutex_unlock(&g_cl_lock); 
         }
      }
      
      (void)CLThreadSleep(VTIMER_1_MILLISEC);
   }
   
   ((clThreadContext_t *)context)->threadExit = 1; 
   ((clThreadContext_t *)context)->threadStatus = status; 
   
   return NULL;
}

//PYTHON: def split_out_vertex (self, new_graph, vertex) :
static FSTATUS CLGraphDataSplitOutVertex(clGraphData_t *graphp, clGraphData_t *newGraphp, clVertixData_t *vertexp, int verbose) 
{ 
   FSTATUS status = FSUCCESS; 
   int ii, nn, new_vertex_id, new_sink_id, new_source_id; 
   uint32 neighborsLength = 0, neighborsCount = 0; 
   clVertixData_t * sink,*source; 
   clArcData_t *arcp; 
   clVertixData_t **neighbors = NULL; 
   
   
   //PYTHON: if vertex.refcount :
   if (vertexp->RefCount) {
      //PYTHON: new_vertex_id = new_graph.add_vertex(vertex.ref)
      new_vertex_id = CLGraphDataAddVertex(newGraphp, vertexp->Connection, verbose); 
      
      //PYTHON: neighbors = []
      /* initialization of neighbors array handled via CLGetDynamicArray() */
      
      //PYTHON: for arc_id in vertex.outbound :
      for (ii = 0; ii < vertexp->OutboundCount; ii++) {
         if (vertexp->Outbound[ii] >= 0 && (arcp = CLGraphFindIdArc(graphp, vertexp->Outbound[ii]))) {
            //PYTHON: sink = self.vertices[self.arcs[arc_id].sink]
            sink = graphp->Vertices[arcp->Sink]; 
            
            if (CLGetDynamicArray((void **)&neighbors, &neighborsLength, sizeof(neighbors), neighborsCount, verbose)) {
               status = FERROR; 
               break;
            }
            
            //PYTHON: neighbors.append(sink)
            neighbors[neighborsCount++] = sink; 
            
            //PYTHON: new_sink_id = new_graph.add_vertex(sink.ref)
            if (-1 == (new_sink_id = CLGraphDataAddVertex(newGraphp, sink->Connection, verbose))) {
               status = FERROR; 
               break;
            }
            //PYTHON: new_graph.add_arc(new_vertex_id, new_sink_id)
            if (-1 == CLGraphDataAddArc(newGraphp, new_vertex_id, new_sink_id, verbose)) {
               status = FERROR; 
               break;
            }
         }
      }
      
      //PYTHON: for arc_id in vertex.outbound :
      for (ii = 0; ii < vertexp->OutboundCount; ii++) {
         //PYTHON: self.del_arc(arc_id)
         if (vertexp->Outbound[ii] >= 0) 
            (void)CLGraphDataDelArc(graphp, vertexp->Outbound[ii]);
      }
      
      //PYTHON: for arc_id in vertex.inbound :
      for (ii = 0; ii < vertexp->InboundCount; ii++) {
         if (vertexp->Inbound[ii] >= 0 && (arcp = CLGraphFindIdArc(graphp, vertexp->Inbound[ii]))) {
            //PYTHON: source = self.vertices[self.arcs[arc_id].source]
            source = graphp->Vertices[arcp->Source]; 
            if (CLGetDynamicArray((void **)&neighbors, &neighborsLength, sizeof(neighbors), neighborsCount, verbose)) {
               status = FERROR; 
               break;
            }
            //PYTHON: neighbors.append(source)
            neighbors[neighborsCount++] = source; 
            
            //PYTHON: new_source_id = new_graph.add_vertex(source.ref)
            if (-1 == (new_source_id = CLGraphDataAddVertex(newGraphp, source->Connection, verbose))) {
               status = FERROR; 
               break;
            }
            
            //PYTHON: new_graph.add_arc(new_source_id, new_vertex_id)
            if (-1 == CLGraphDataAddArc(newGraphp, new_source_id, new_vertex_id, verbose)) {
               status = FERROR; 
               break;
            }
         }
      }
      
      //PYTHON: for arc_id in vertex.inbound :
      for (ii = 0; ii < vertexp->InboundCount; ii++) {
         //PYTHON: self.del_arc(arc_id)
         if (vertexp->Inbound[ii] >= 0) 
            (void)CLGraphDataDelArc(graphp, vertexp->Inbound[ii]);
      }
      
      //PYTHON: for n in neighbors :
      for (nn = 0; nn < neighborsCount; nn++) {
         //PYTHON: self.split_out_vertex(new_graph, n)
         CLGraphDataSplitOutVertex(graphp, newGraphp, neighbors[nn], verbose);
      }
   }
   
   // deallocate dynamic array
   MemoryDeallocate(neighbors); 
   
   return status;
}

static void CLDijkstraFreeArray(void **array, uint32 nRows) 
{ 
   int i; 
   
   if (array) {
      for (i = 0; i < nRows; i++) {
         if (array[i]) 
            MemoryDeallocate(array[i]);
      }
      MemoryDeallocate(array);
   }
}

static void** CLDijkstraAllocArray(uint32 nRows, uint32 nCols, uint32 entryLenth) 
{ 
   int i; 
   void **array; 
   
   array = MemoryAllocate2AndClear(nRows * sizeof(void *), IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
   if (!array) 
      fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
   else {
      for (i = 0; i < nRows; i++) {
         array[i] = MemoryAllocate2AndClear(nCols * entryLenth, IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
         if (!array[i]) {
            fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
            (void)CLDijkstraFreeArray(array, i); 
            return NULL;
         }
      }
   }
   
   return array;
}

static clVertixDataDistance_t* CLDijkstraAddVertexDistance(cl_qmap_t *mapp, clVertixData_t *vertixp, int verbose) 
{ 
   cl_map_item_t *mi; 
   clVertixDataDistance_t *entryp = NULL; 
   
   if (mapp && vertixp) {
      entryp = (clVertixDataDistance_t *)MemoryAllocate2AndClear(sizeof(clVertixDataDistance_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
      if (!entryp) 
         fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
      else {
         entryp->vertixp = vertixp; 
         mi = cl_qmap_insert(mapp, vertixp->Id, &entryp->AllVerticesEntry); 
         if (verbose >= 4) 
            printf("Add distance vertix Id %d\n", vertixp->Id); 
         if (mi != &entryp->AllVerticesEntry) {
            MemoryDeallocate(entryp); 
            entryp = NULL; 
            fprintf(stderr, "Error, failed to add vertix Id %d\n", vertixp->Id);
         }
      }
   }
   
   return entryp;
}

static int CLDijkstraVertexPopDistance(cl_qmap_t *mapp) 
{ 
   uint32 id = -1; 
   cl_map_item_t *mi; 
   clVertixDataDistance_t *entryp; 
   
   mi = cl_qmap_head(mapp); 
   if (mi != cl_qmap_end(mapp)) {
      if (NULL != (entryp = PARENT_STRUCT(mi, clVertixDataDistance_t, AllVerticesEntry))) {
         id = entryp->vertixp->Id; 
         cl_qmap_remove_item(mapp, &entryp->AllVerticesEntry); 
         MemoryDeallocate(entryp);
      }
   }
   
   return id;
}

//PYTHON: def get_distance (distances, src, dst) :
static FSTATUS CLDijkstraGetDistance(uint32 **distances, uint32 nRows, uint32 nCols, uint32 src, uint32 dst, uint32 *value) 
{ 
   FSTATUS status = FSUCCESS; 
   
   //PYTHON: if distances.get(src) == None :
   //PYTHON:    return infinity
   *value = DIJKSTRA_INFINITY; 
   if (distances == NULL) 
      status = FERROR; 
   else if (src >= nRows || dst >= nCols) {
      status = FINVALID_PARAMETER; 
      fprintf(stderr, "Warning, invalid distance parameters: src %d, dst %d\n", src, dst);
   } else {
      //PYTHON: return distances.get(src).get(dst)
      *value = distances[src][dst];
   }
   
   return status;
}

//PYTHON: def check_distances_and_routes (graph, distances, routes) :
static int CLDijkstraCheckDistancesAndRoutes(clGraphData_t *graphp, uint32 **distances, uint32 **routes, uint32 nRows, uint32 nCols, uint32 verbose) 
{ 
   uint32 ii, jj, errors = 0;      //PYTHON: errors = 0
   clVertixData_t * i,*j; 
   
   //PYTHON: for i in graph.vertices :
   for (ii = 0; ii < graphp->VerticesLength; ii++) {
      //PYTHON: if i.refcount :
      if ((i = graphp->Vertices[ii]) && i->RefCount) {
         //PYTHON: for j in graph.vertices :
         for (jj = 0; jj < graphp->VerticesLength; jj++) {
            uint32 d, distance; 
            
            //PYTHON: if j.refcount :
            if ((j = graphp->Vertices[jj]) && j->RefCount) {
               uint32 k, found; 
               
               d = 0;      //PYTHON: d = 0                    
               k = i->Id;  //PYTHON: k = i.id
               
               //PYTHON: while j.id in routes[k] :
               //CJKING: while ((found = CLListUint32Find(j->Id, routes[k], nCols, verbose))) {
               while ((found = CLListUint32Find(routesLstMap, j->Id))) {
                  uint32 l; 
                  
                  l = routes[k][j->Id];   //PYTHON: l = routes[k][j.id]
                  d++;                    //PYTHON: d += 1
                                          //PYTHON: if l == j.id :
                  if (l == j->Id) 
                     break; 
                  k = l;                  //PYTHON: k = l
               }
               //PYTHON: else :
               //PYTHON: d = infinity
               if (!found) 
                  d = DIJKSTRA_INFINITY; 
               
               //PYTHON: distance = get_distance(distances, i.id, j.id)
               if (FERROR == CLDijkstraGetDistance(distances, nRows, nCols, i->Id, j->Id, &distance)) 
                  return -1; 
               if (distance == d) {        //if distance == d :
                  if (verbose >= 4) 
                     printf("Check route from %d to %d of %d hops: OK\n", i->Id, j->Id, distance);
               } else {
                  errors++;        //PYTHON: errors += 1
                  if (verbose >= 4) 
                     fprintf(stderr, "Warning, check route from %d to %d of %d hops: failed, got %d hops\n", 
                             i->Id, j->Id, distance, d);
               }
            }
         }
      }
   }
   
   if (verbose >= 3) 
      printf("Check route summary : %d dependency graph routes have errors\n", errors); 
   
   return errors;
}

FSTATUS CLFabricDataDestroy(FabricData_t *fabricp, void *context) 
{ 
   (void)CLDeviceDataFreeAll(fabricp, context); 
   (void)CLGraphDataFree(&fabricp->Graph, context); 
   memset(&fabricp->Graph, 0, sizeof(fabricp->Graph)); 
   
   cl_qmap_init(&fabricp->map_guid_to_ib_device, NULL); 
   QListInitState(&fabricp->FIs); 
   if (!QListInit(&fabricp->FIs)) {
      fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname); 
      goto fail;
   }
   QListInitState(&fabricp->Switches); 
   if (!QListInit(&fabricp->Switches)) {
      fprintf(stderr, "%s: Unable to initialize List\n", g_Top_cmdname); 
      goto fail;
   }
   cl_qmap_init(&fabricp->Graph.Arcs, NULL); 
   cl_qmap_init(&fabricp->Graph.map_arc_key_to_arc, NULL); 
   cl_qmap_init(&fabricp->Graph.map_conn_to_vertex_conn, NULL); 
   
   return FSUCCESS; 
   
fail:
   return FERROR;
}

//PYTHON: def add_device (fabric, guid, lid, hfi, name) :
NodeData* CLDataAddDevice(FabricData_t *fabricp, NodeData *nodep, STL_LID lid, int verbose, int quiet) 
{ 
   cl_map_item_t *mi; 
   clDeviceData_t *ccDevicep; 
   
   if (nodep) {
      NodeData *p; 
      
      //PYTHON: device = fabric.map_guid_to_ib_device.get(guid)
      mi = cl_qmap_get(&fabricp->map_guid_to_ib_device, nodep->NodeInfo.NodeGUID); 
      if (mi != cl_qmap_end(&fabricp->map_guid_to_ib_device)) {
         ccDevicep = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
         if (verbose >= 5) 
            fprintf(stderr, "%s: Duplicate NodeGuids found in nodeRecords: 0x%"PRIx64"\n", 
                    g_Top_cmdname, nodep->NodeInfo.NodeGUID); 
         
         p = ccDevicep->nodep; 
         //PYTHON: if device.guid != guid or device.lid != lid or
         //PYTHON:    device.hfi != hfi or device.name != name :
         if (ccDevicep->Lid != lid || 
             p->NodeInfo.NodeGUID != nodep->NodeInfo.NodeGUID || 
             p->NodeInfo.NodeType != nodep->NodeInfo.NodeType || 
             strcmp((char *)p->NodeDesc.NodeString, (char *)nodep->NodeDesc.NodeString)) {
             if (verbose >= 5) 
                fprintf(stderr, "Warning, Duplicate device (%s, 0x%016"PRIx64", LID 0x%08x, %s) differs from (%s, 0x%016"PRIx64", LID 0x%04x, %s) with same GUID\n", 
                        (char *)nodep->NodeDesc.NodeString, nodep->NodeInfo.NodeGUID, lid, StlNodeTypeToText(nodep->NodeInfo.NodeType), 
                        (char *)p->NodeDesc.NodeString, p->NodeInfo.NodeGUID, ccDevicep->Lid, StlNodeTypeToText(p->NodeInfo.NodeType));
         }
         return nodep;
      }
      
      //PYTHON: device = IbDevice(fabric, guid, lid, hfi, name)
      ccDevicep = (clDeviceData_t *)MemoryAllocate2AndClear(sizeof(clDeviceData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
      if (!ccDevicep) {
         fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
         return NULL;
      }
      
      ccDevicep->nodep = nodep; 
      ccDevicep->Lid = lid; 
      cl_qmap_init(&ccDevicep->map_dlid_to_route, NULL); 
      //PYTHON: fabric.devices.append(device)
      //PYTHON: fabric.map_guid_to_ib_device[guid] = device
      mi = cl_qmap_insert(&fabricp->map_guid_to_ib_device, nodep->NodeInfo.NodeGUID, &ccDevicep->AllDevicesEntry); 
      if (mi != &ccDevicep->AllDevicesEntry) {
         MemoryDeallocate(ccDevicep); 
         ccDevicep = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
         fprintf(stderr, "Error, failed to add %s %s with GUID 0x%016"PRIx64" and LID 0x%08x\n", 
                 StlNodeTypeToText(nodep->NodeInfo.NodeType), 
                 (char *)nodep->NodeDesc.NodeString, 
                 nodep->NodeInfo.NodeGUID, 
                 lid); 
         return NULL;
      } else {
         ListItemInitState(&ccDevicep->AllDeviceTypesEntry); 
         QListSetObj(&ccDevicep->AllDeviceTypesEntry, ccDevicep); 
         
         if (nodep->NodeInfo.NodeType == STL_NODE_FI) 
            QListInsertTail(&fabricp->FIs, &ccDevicep->AllDeviceTypesEntry);       //PYTHON: fabric.hfis.append(device)
         else 
            QListInsertTail(&fabricp->Switches, &ccDevicep->AllDeviceTypesEntry);   //PYTHON: fabric.switches.append(device)
         if (verbose >= 4 && !quiet) 
            ProgressPrint(TRUE, "Add %s %s with GUID 0x%016"PRIx64" and LID 0x%08x", 
                   StlNodeTypeToText(nodep->NodeInfo.NodeType), 
                   (char *)nodep->NodeDesc.NodeString, 
                   nodep->NodeInfo.NodeGUID, 
                   lid);
      }
      return nodep;
   }
   
   return NULL;
}

//PYTHON: def add_connection (fabric, guid1, port1, guid2, port2, rate) :
FSTATUS CLDataAddConnection(FabricData_t *fabricp, PortData *portp1, PortData *portp2, clConnPathData_t *pathInfo, uint8 sc, int verbose, int quiet) 
{ 
   FSTATUS status = FSUCCESS; 
   cl_map_item_t *mi; 
   clConnData_t *ccConnp; 
   clDeviceData_t *ccDevicep; 
   
   
   //PYTHON: ib_device1 = fabric.map_guid_to_ib_device.get(guid1)
   mi = cl_qmap_get(&fabricp->map_guid_to_ib_device, portp1->nodep->NodeInfo.NodeGUID); 
   if (mi == cl_qmap_end(&fabricp->map_guid_to_ib_device)) {
      fprintf(stderr, "Error, add connection for missing device GUID 0x%016"PRIx64"\n", portp1->nodep->NodeInfo.NodeGUID); 
      return FINVALID_PARAMETER;
   }
   ccDevicep = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
   
   if (portp1->PortNum >= CREDIT_LOOP_DEVICE_MAX_CONNECTIONS) {
      fprintf(stderr, "Error, port number %d exceeds maximum ports supported by a device\n", portp1->PortNum); 
      return FINVALID_PARAMETER;
   }
   
   //PYTHON: connection = ib_device1.connections.get(port1)
   ccConnp = ccDevicep->Connections[portp1->PortNum][sc];
   if (ccConnp) {
      //PYTHON: if connection.guid1 != guid1 or connection.port1 != port1 or
      //PYTHON:    connection.guid2 != guid2 or connection.port2 != port2 or
      //PYTHON:    connection.rate != rate :
      if ((ccConnp->FromDeviceGUID != portp1->nodep->NodeInfo.NodeGUID || ccConnp->FromPortNum != portp1->PortNum) || 
          (ccConnp->ToDeviceGUID != portp2->nodep->NodeInfo.NodeGUID || ccConnp->ToPortNum != portp2->PortNum) || 
          ((ccConnp->Rate && portp1->rate) && (ccConnp->Rate != portp1->rate))) {
         status = FERROR; 
         fprintf(stderr, "Error, new connection (0x%016"PRIx64":%3.3u to 0x%016"PRIx64":%3.3u at rate %s) differs from (0x%016"PRIx64":%3.3u to 0x%016"PRIx64":%3.3u at rate %s)\n", 
                 ccConnp->FromDeviceGUID, ccConnp->FromPortNum, ccConnp->ToDeviceGUID, ccConnp->ToPortNum, StlStaticRateToText(ccConnp->Rate), 
                 portp1->nodep->NodeInfo.NodeGUID, portp1->PortNum, portp2->nodep->NodeInfo.NodeGUID, portp2->PortNum, StlStaticRateToText(portp1->rate));
      }
   } else {
      //PYTHON: ib_device1.connections[port1] = IbConnection(fabric, guid1, port1, guid2, port2, rate)
      ccConnp = ccDevicep->Connections[portp1->PortNum][sc] =
         (clConnData_t *)MemoryAllocate2AndClear(sizeof(clConnData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
      
      if (!ccConnp) {
         status = FINSUFFICIENT_MEMORY; 
         fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
      } else {
         //PYTHON: fabric.connections += 1
         fabricp->ConnectionCount++; 
         ccConnp->FromDeviceGUID = portp1->nodep->NodeInfo.NodeGUID;         // GUID of the HFI, TFI, switch
         ccConnp->FromPortNum = portp1->PortNum; 
         ccConnp->ToDeviceGUID = portp2->nodep->NodeInfo.NodeGUID;           // GUID of the HFI, TFI, switch
         ccConnp->ToPortNum = portp2->PortNum; 
         ccConnp->PathInfo = *pathInfo;
         if (portp1->pQOS)
            ccConnp->VL = portp1->pQOS->SC2VLMaps[Enum_SCVLt].SCVLMap[sc];
         
         if (verbose >= 4 && !quiet) 
            ProgressPrint(TRUE, "Add connection 0x%016"PRIx64":%3.3u to 0x%016"PRIx64":%3.3u at rate %s",
                   portp1->nodep->NodeInfo.NodeGUID, portp1->PortNum, portp2->nodep->NodeInfo.NodeGUID,
                   portp2->PortNum, StlStaticRateToText(portp1->rate));
      }
   }
 
   return status;
}

//PYTHON: add_route(fabric, slid, dlid, route_sguid, route_sport)
FSTATUS CLDataAddRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid, PortData *sportp, int verbose, int quiet) 
{ 
   FSTATUS status = FSUCCESS; 
   cl_map_item_t *mi; 
   clDeviceData_t *ccDevicep; 
   clRouteData_t *ccRoutep = NULL; 
   
   
   //PYTHON: ib_device = fabric.map_guid_to_ib_device.get(guid)
   //PYTHON: if not ib_device :
   mi = cl_qmap_get(&fabricp->map_guid_to_ib_device, sportp->nodep->NodeInfo.NodeGUID); 
   if (mi == cl_qmap_end(&fabricp->map_guid_to_ib_device)) {
      fprintf(stderr, "Warning, add connection for missing device GUID 0x%016"PRIx64"\n", sportp->nodep->NodeInfo.NodeGUID); 
      return FNOT_FOUND;
   }
   ccDevicep = PARENT_STRUCT(mi, clDeviceData_t, AllDevicesEntry); 
   
   //PYTHON: route = ib_device.routes.get(dlid)
   mi = cl_qmap_get(&ccDevicep->map_dlid_to_route, dlid); 
   if (mi != cl_qmap_end(&ccDevicep->map_dlid_to_route)) 
      ccRoutep = PARENT_STRUCT(mi, clRouteData_t, AllRoutesEntry); 
   
   if (ccRoutep) {
      if (ccRoutep->portp != sportp) {
         status = FERROR; 
         fprintf(stderr, "Error, new route entry from SLID 0x%08x to DLID 0x%08x at 0x%016"PRIx64" has port %3.3u not expected port %3.3u\n", 
                 slid, dlid, sportp->nodep->NodeInfo.NodeGUID, sportp->PortNum, ccRoutep->portp->PortNum);
      }
   } else {
      ccRoutep = (clRouteData_t *)MemoryAllocate2AndClear(sizeof(clRouteData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
      
      if (!ccRoutep) {
         status = FINSUFFICIENT_MEMORY; 
         fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
      } else {
         //PYTHON: ib_device.routes[dlid] = port
         ccRoutep->portp = sportp; 
         mi = cl_qmap_insert(&ccDevicep->map_dlid_to_route, dlid, &ccRoutep->AllRoutesEntry); 
         if (mi != &ccRoutep->AllRoutesEntry) {
            if (verbose >= 4) {
               fprintf(stderr, "%s: Duplicate DLID found in routes map: 0x%08x\n", g_Top_cmdname, dlid);
            }
            MemoryDeallocate(ccRoutep); 
            ccRoutep = PARENT_STRUCT(mi, clRouteData_t, AllRoutesEntry);
         } else {
            //PYTHON: fabric.routes += 1
            fabricp->RouteCount++; 
            if (verbose >= 4 && !quiet) 
               ProgressPrint(TRUE, "Add route SLID 0x%08x to DLID 0x%08x at GUID 0x%016"PRIx64" using port %3.3u",
                      slid, dlid, sportp->nodep->NodeInfo.NodeGUID, sportp->PortNum);
         }
      }
   }
   
   return status;
}

//PYTHON: def IbFabric.summary (self, name) :
void CLFabricSummary(FabricData_t *fabricp, const char *name,
                     ValidateCLFabricSummaryCallback_t callback,
                     uint32 totalPaths, uint32 totalBadPaths,
                     void *context) 
{ 
   callback(fabricp, name, totalPaths, totalBadPaths, context);
}

//PYTHON: def Graph.summary (self, name) :
void CLGraphDataSummary(clGraphData_t *graphp, const char *name, ValidateCLDataSummaryCallback_t callback, void *context) 
{ 
   callback(graphp, name, context);
}

FSTATUS CLFabricDataBuildRouteGraph(FabricData_t *fabricp,  ValidateCLRouteSummaryCallback_t routeSummaryCallback, ValidateCLTimeGetCallback_t timeGetCallback, void *context, uint32 usedSLs) 
{ 
   FSTATUS status = FSUCCESS; 
   uint32 i = 0, l = 0, t, threadsActive = 0; 
   LIST_ITEM *srcHcaLstp; 
   pthread_t threadId; 
   uint64_t sTime = 0, eTime = 0; 
   ValidateCreditLoopRoutesContext_t *cp = (ValidateCreditLoopRoutesContext_t *)context; 
   int xmlFmt = (cp->format == 1) ? 1 : 0; 
   int verbose = (xmlFmt) ? 0 : cp->detail; 
   uint32 maxHcaListEntry = QListCount(&fabricp->FIs) / CL_MAX_THREADS; 
   clThreadContext_t *threadContexts; 
   
   
   cl_qmap_init(&hopsHistogramLstMap, NULL); 
   if (hops_histogram != NULL) 
      memset(hops_histogram, 0, hops_histogram_length); 
   
   threadContexts = 
      (clThreadContext_t *)MemoryAllocate2AndClear(sizeof(clThreadContext_t) * CL_MAX_THREADS, IBA_MEM_FLAG_PREMPTABLE, MYTAG); 
   
   if (!threadContexts) {
      status = FINSUFFICIENT_MEMORY; 
      fprintf(stderr, "%s: Unable to allocate thread contexts\n", g_Top_cmdname); 
      return status;
   }
   
   if (!xmlFmt && verbose >= 3) {
      timeGetCallback(&sTime, &g_cl_lock); 
      printf("START build graphical layout of all the routes\n"); 
   }
   
   pthread_mutex_init(&g_cl_lock, NULL); 
   
   //PYTHON: for src_hfi in fabric.hfis :
   if (QListCount(&fabricp->FIs) < 100) {
      threadContexts[0].srcHcaList = QListHead(&fabricp->FIs); 
      
      // determine whether to start worker thread
      threadContexts[0].fabricp = fabricp; 
      threadContexts[0].verbose = verbose; 
      threadContexts[0].quiet = cp->quiet; 
      threadContexts[0].threadId = 0; 
      threadContexts[0].srcHcaListLength = QListCount(&fabricp->FIs);
      threadContexts[0].timeGetCallback = timeGetCallback; 
      threadContexts[0].usedSLs = usedSLs;
      threadsActive++; 
      pthread_create(&threadId, NULL, CLFabricDataBuildRouteGraphThread, &threadContexts[0]);
   } else {
      for (srcHcaLstp = QListHead(&fabricp->FIs); srcHcaLstp != NULL; srcHcaLstp = QListNext(&fabricp->FIs, srcHcaLstp)) {
         // set first hfi entry to hfi list
         if (l == 0) 
            threadContexts[i].srcHcaList = srcHcaLstp; 
         l++; 
         
         // determine whether to start worker thread
         if (i < CL_MAX_THREADS - 1 && l >= maxHcaListEntry) {
            l = 0; 
            threadContexts[i].fabricp = fabricp; 
            threadContexts[i].verbose = verbose; 
            threadContexts[i].quiet = cp->quiet; 
            threadContexts[i].threadId = i; 
            threadContexts[i].srcHcaListLength = maxHcaListEntry;
            threadContexts[i].timeGetCallback = timeGetCallback;
            threadContexts[i].usedSLs = usedSLs;
            threadsActive++; 
            pthread_create(&threadId, NULL, CLFabricDataBuildRouteGraphThread, &threadContexts[i++]);
         }
      }
      
      // start last worker thread
      threadContexts[i].fabricp = fabricp; 
      threadContexts[i].verbose = verbose; 
      threadContexts[i].quiet = cp->quiet; 
      threadContexts[i].threadId = i; 
      threadContexts[i].srcHcaListLength = maxHcaListEntry + (QListCount(&fabricp->FIs) % CL_MAX_THREADS);
      threadContexts[i].timeGetCallback = timeGetCallback;
      threadContexts[i].usedSLs = usedSLs;
      threadsActive++; 
      pthread_create(&threadId, NULL, CLFabricDataBuildRouteGraphThread, &threadContexts[i]);
   }
   
   //
   // wait for worker threads to complete processing
   while (threadsActive > 0) {
      (void)CLThreadSleep(VTIMER_1_MILLISEC); 
      for (t = 0; t < CL_MAX_THREADS; t++) {
         if (threadContexts[t].threadExit) {
            threadContexts[t].threadExit = 0; 
            threadsActive--; 
         }
      }
   }

   // clear line used to display progress report
   if (!cp->quiet)
      ProgressPrint(TRUE, "Done Building Graphical Layout of All Routes");
   
   if (!xmlFmt && verbose >= 3) {
      timeGetCallback(&eTime, &g_cl_lock); 
      printf("END build graphical layout of all the routes; elapsed time(usec)=%d, (sec)=%d\n", 
             (int)(eTime - sTime), ((int)(eTime - sTime)) / CL_TIME_DIVISOR); 
   }
   
   //
   // report routes summary
   if (hops_histogram != NULL) {
      if (verbose >= 3) 
         routeSummaryCallback(present_routes, missing_routes, hops_histogram_entries, hops_histogram, context); 
      MemoryDeallocate(hops_histogram);
   }
   MemoryDeallocate(threadContexts); 
   
   return status;
}

//PYTHON: Graph.def split (self) :
clGraphData_t* CLGraphDataSplit(clGraphData_t *graphp, int verbose) 
{ 
   uint32 vv; 
   clGraphData_t *newGraphp; 
   clVertixData_t *rootVertexp = NULL;     //PYTHON: root = None
   
   
   //PYTHON: for vertex in self.vertices :
   for (vv = 0; vv < graphp->NumVertices; vv++) {
      clVertixData_t *vertexp = graphp->Vertices[vv]; 
      
      //PYTHON: if vertex.refcount :
      if (vertexp->RefCount) {
         rootVertexp = vertexp;      //PYTHON: root = vertex
         break;
      }
   }
   
   //PYTHON: if not root :
   if (!rootVertexp) 
      return NULL; 
   
   //PYTHON: new_graph = Graph()
   newGraphp = (clGraphData_t *)MemoryAllocate2AndClear(sizeof(clGraphData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);        
   if (!newGraphp) {
      fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname);
   } else {
      cl_qmap_init(&newGraphp->Arcs, NULL); 
      cl_qmap_init(&newGraphp->map_arc_key_to_arc, NULL); 
      //PYTHON: self.split_out_vertex(new_graph, root);
      CLGraphDataSplitOutVertex(graphp, newGraphp, rootVertexp, verbose);
   }
   return newGraphp;
}

void CLGraphDataPrune(clGraphData_t *graphp, ValidateCLTimeGetCallback_t timeGetCallback, int verbose, int quiet) 
{ 
   uint32 ii, vv, progress = 1, round = 0; 
   uint64_t sTime = 0, eTime = 0; 
   
   if (verbose >= 3 && !quiet) {
      timeGetCallback(&sTime, &g_cl_lock); 
      ProgressPrint(TRUE, "START pruning of graphical layout of all the routes");
   }
   
   //PYTHON: progress = 1;
   //PYTHON: round = 0;
   //PYTHON: while progress :
   while (progress) {          // DEBUG_CODE --- Possible infinite loop, look into later????
      progress = 0; 
      //PYTHON: for vertex in self.vertices :
      for (vv = 0; vv < graphp->NumVertices; vv++) {
         clVertixData_t *vertexp = graphp->Vertices[vv]; 
         
         //PYTHON: if vertex.refcount :
         if (vertexp->RefCount) {
            //PYTHON: if len(vertexp->outbound) == 0 :
            if (vertexp->OutboundInuseCount == 0) {
               //PYTHON: for arc_id in vertexp->inbound :
               for (ii = 0; ii < vertexp->InboundCount; ii++) {
                  if (vertexp->Inbound[ii] >= 0) {
                     if (verbose >= 4 && !quiet) 
                        ProgressPrint(TRUE, "Remove arc id %d since vertex id %d is pure sink", 
                               vertexp->Inbound[ii], vertexp->Id); 
                     //PYTHON: self.del_arc(arc_id);
                     CLGraphDataDelArc(graphp, vertexp->Inbound[ii]); 
                     //PYTHON: progress += 1
                     progress++;
                  }
               }
            }
            
            //PYTHON: if len(vertexp->inbound) == 0 :
            if (vertexp->InboundInuseCount == 0) {
               //PYTHON: for arc_id in vertex.outbound :
               for (ii = 0; ii < vertexp->OutboundCount; ii++) {
                  if (vertexp->Outbound[ii] >= 0) {
                     if (verbose >= 4 && !quiet) 
                        ProgressPrint(TRUE, "Remove arc id %d since vertex id %d is pure source", 
                               vertexp->Outbound[ii], vertexp->Id); 
                     //PYTHON: self.del_arc(arc_id);
                     CLGraphDataDelArc(graphp, vertexp->Outbound[ii]); 
                     //PYTHON: progress += 1
                     progress++;
                  }
               }
            }
         }
      }
      
      //PYTHON: round += 1
      round++; 
      if (verbose >= 4 && !quiet) 
         ProgressPrint(TRUE, "Graph pruning round %d : deleted %d arcs", round, progress);
   }
   
   if (verbose >= 3 && !quiet) {
      timeGetCallback(&eTime, &g_cl_lock); 
      ProgressPrint(TRUE, "END pruning of graphical layout of all the routes; elapsed time(usec)=%d, (sec)=%d", 
             (int)(eTime - sTime), ((int)(eTime - sTime)) / CL_TIME_DIVISOR);
   }
}

void CLDijkstraFreeDistancesAndRoutes(clDijkstraDistancesAndRoutes_t *drp) 
{ 
   if (drp->distances) 
      (void)CLDijkstraFreeArray((void **)drp->distances, drp->nRows); 
   if (drp->routes) 
      (void)CLDijkstraFreeArray((void **)drp->routes, drp->nRows); 
   memset(drp, 0, sizeof(clDijkstraDistancesAndRoutes_t));
}

//PYTHON: def find_distances_and_routes_dijkstra (graph) :
FSTATUS CLDijkstraFindDistancesAndRoutes(clGraphData_t *graphp, clDijkstraDistancesAndRoutes_t *respData, int verbose) 
{ 
   FSTATUS status = FERROR; 
   int c; 
   uint32 nRows, nCols;
   uint32 ii, jj, d, current = 0; 
   uint32 next, numVertices, maxDistance; 
   uint32 **distances = NULL,**routes = NULL; 
   clVertixData_t *i = NULL,*j = NULL; 
   uint32 *dist = NULL,*route = NULL,*previous = NULL; 
   cl_qmap_t previousLstMap; 
   
   
   // allocate persistant multidimensional arrays and temporary buffers
   if (!graphp || !respData) 
      return FINVALID_PARAMETER;
   nRows = graphp->VerticesLength;
   nCols = graphp->VerticesLength; 
   if (!(distances    = (uint32 **)CLDijkstraAllocArray(nRows, nCols, sizeof(uint32))) ||  //PYTHON: distances = {}
            !(routes       = (uint32 **)CLDijkstraAllocArray(nRows, nCols, sizeof(uint32))) ||  //PYTHON: routes = {}
            !(dist         = MemoryAllocate2AndClear(nCols * sizeof(uint32), IBA_MEM_FLAG_PREMPTABLE, MYTAG)) || 
            !(route        = MemoryAllocate2AndClear(nCols * sizeof(uint32), IBA_MEM_FLAG_PREMPTABLE, MYTAG)) || 
            !(previous     = MemoryAllocate2AndClear(nCols * sizeof(uint32), IBA_MEM_FLAG_PREMPTABLE, MYTAG)) || 
            !(routesLstMap = MemoryAllocate2AndClear(nRows * sizeof(cl_qmap_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
      status = FINSUFFICIENT_MEMORY; 
      fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
      goto fail;
   }
   
   //PYTHON: num_vertices = graph.num_vertices
   numVertices = graphp->NumVertices; 
   if (verbose >= 3) 
      printf("Calculating distances for %d vertices in graph\n", numVertices); 
   
   //PYTHON: for i in graph.vertices :
   for (ii = 0; ii < graphp->VerticesLength; ii++) {
      cl_qmap_t todo; 
      
      //PYTHON: if i.refcount :
      if ((i = graphp->Vertices[ii]) && i->RefCount) {
         //PYTHON: dist = {}
         memset(dist, 0, sizeof(nCols * sizeof(uint32))); 
         //PYTHON: previous = {}
         memset(previous, 0, sizeof(nCols * sizeof(uint32))); 
         //PYTHON: todo = []
         cl_qmap_init(&todo, NULL); 
         cl_qmap_init(&routesLstMap[i->Id], NULL); 
         // CJKING: Note, in order to save time, purposely not deallocating memory
         // allocated for each list entry.  May need to visit this later for extremely
         // large fabrics.
         cl_qmap_init(&previousLstMap, NULL); 
         
         //PYTHON: for j in graph.vertices :
         for (jj = 0; jj < graphp->VerticesLength; jj++) {
            if ((j = graphp->Vertices[jj]) && j->RefCount) {
               dist[j->Id] = DIJKSTRA_INFINITY; 
               //PYTHON: todo.append(j.id)
               if (!CLDijkstraAddVertexDistance(&todo, j, verbose)) 
                  goto fail; 
               //PYTHON: previous[j.id] = None
               previous[j->Id] = 0;
            }
         }
         
         //PYTHON: dist[i.id] = 0
         dist[i->Id] = 0; 
         //PYTHON: previous[i.id] = i.id
         previous[i->Id] = i->Id; 
         //CJKING: Add entry to search list for previous array 
         if (CLListUint32Add(&previousLstMap, i->Id)) 
            goto fail; 
         
         //current = i
         current = ii; 
         
         //while todo :
         while (cl_qmap_count(&todo)) {
            /*CJKING: Not certain of the purpose of this code in the script.

              PYTHON: global vertex_dist
              PYTHON: vertex_dist = dist
              PYTHON: todo.sort(vertex_distance_compare)
              PYTHON: vertex_dist = {}
            */
            
            //PYTHON: current = todo.pop(0)
            if (-1 == (current = CLDijkstraVertexPopDistance(&todo))) {
               if (verbose >= 3) 
                  fprintf(stderr, "Warning, pop of vertices list failed\n"); 
               break;
            }
            
            //PYTHON: if dist[current] == infinity :
            if (verbose >= 4 && dist[current] == DIJKSTRA_INFINITY) {
               fprintf(stderr, "Warning, distance of current id %d should not be infinite for fully connected graph\n", current);
            }
            
            //PYTHON: for c in graph.vertices[current].outbound :
            for (c = 0; c < graphp->Vertices[current]->OutboundCount; c++) {
               clVertixData_t *j; 
               clArcData_t *arcp; 
               
               if (graphp->Vertices[current]->Outbound[c] >= 0 && (arcp = CLGraphFindIdArc(graphp, graphp->Vertices[current]->Outbound[c]))) {
                  //PYTHON: j = graph.vertices[graph.arcs[c].sink]
                  j = graphp->Vertices[arcp->Sink]; 
                  
                  //PYTHON: if not j :
                  if (!j) 
                     fprintf(stderr, "Warning, cannot follow connection for vertex %d\n", current); 
                  else {  //PYTHON: elif j.id in todo :   /* CJKING: Unclear about this elif.  Might use the following */
                     /* else if (CLDijkstraIsVertexDistance(&todo, j->Id))          */
                     //PYTHON: d = dist[current] + 1
                     d = dist[current] + 1; 
                     //PYTHON: if d < dist[j.id] :
                     if (d < dist[j->Id]) {
                        if (verbose >= 4) 
                           printf("Distance from %d to %d reduces from %d to %d (via %d)\n", 
                                  i->Id, j->Id, dist[j->Id], d, current); 
                        //PYTHON: dist[j.id] = d
                        dist[j->Id] = d; 
                        //PYTHON: previous[j.id] = current
                        previous[j->Id] = current; 
                        //CJKING: Add entry to search list for previous array 
                        if (CLListUint32Add(&previousLstMap, current)) 
                           goto fail;
                     }
                  }
               }
            }
         }
         
         //PYTHON: route = {}
         memset(route, 0, sizeof(nCols * sizeof(uint32))); 
         //PYTHON: for j in graph.vertices :
         for (jj = 0; jj < graphp->VerticesLength; jj++) {
            //PYTHON: if j.refcount and dist[j.id] != infinity :
            if ((j = graphp->Vertices[jj]) && j->RefCount && (dist[j->Id] != DIJKSTRA_INFINITY)) {
               if (verbose >= 4) 
                  printf("Route backwards from vertex %d\n", j->Id); 
               
               //PYTHON: current = j.id
               current = j->Id; 
               //PYTHON: next = current
               next = current; 
               
               //PYTHON: while current != i.id :
               while (current != i->Id) {
                  //next = current
                  next = current; 
                  //PYTHON: if not current in previous :
                  //CJKING: if (!CLListUint32Find(current, previous, nCols, verbose)) {
                  if (!CLListUint32Find(&previousLstMap, current)) {
                     if (verbose >= 5) 
                        fprintf(stderr, "Warning, cannot route backwards from vertex %d to %d at %d\n", 
                                j->Id, i->Id, current);
                  }
                  
                  //PYTHON: current = previous[current]
                  current = previous[current]; 
                  if (verbose >= 4 && next != j->Id) 
                     printf("\tvia %d\n", next);
               }
               
               if (verbose >= 4) 
                  printf("\tto %d (next is %d)\n", i->Id, next); 
               //PYTHON: route[j.id] = next
               route[j->Id] = next; 
               //CJKING: Add entry to search list for route array 
               if (CLListUint32Add(&routesLstMap[i->Id], next)) 
                  goto fail;
            }
         }
         
         //PYTHON: routes[i.id] = route
         //PYTHON: distances[i.id] = dist
         memcpy(routes[i->Id], route, nCols); 
         memcpy(distances[i->Id], dist, nCols);
      }
   }
   
   // recompute distance to self for shortest loop (instead of 0)
   //PYTHON: for i in graph.vertices :
   for (ii = 0; ii < graphp->VerticesLength; ii++) {
      //PYTHON: if i.refcount :
      if ((i = graphp->Vertices[ii]) && i->RefCount) {
         uint32 route = 0;       //PYTHON: route = None
         
         //PYTHON: d = infinity
         d = DIJKSTRA_INFINITY; 
         
         //PYTHON: for c in graph.vertices[i.id].outbound :
         for (c = 0; c < graphp->Vertices[i->Id]->OutboundCount; c++) {
            clVertixData_t *j; 
            clArcData_t *arcp; 
            
            if (graphp->Vertices[i->Id]->Outbound[c] >= 0 && (arcp = CLGraphFindIdArc(graphp, graphp->Vertices[i->Id]->Outbound[c]))) {
               //PYTHON: j = graph.vertices[graph.arcs[c].sink]
               j = graphp->Vertices[arcp->Sink]; 
               //PYTHON: if not j :
               if (!j) 
                  fprintf(stderr, "Warning, cannot follow connection for vertex %d\n", i->Id); 
               else if (distances[j->Id][i->Id] < d) { //PYTHON: elif distances[j.id][i.id] < d :
                                                       //PYTHON: d = distances[j.id][i.id] + 1
                  d = distances[j->Id][i->Id] + 1; 
                  //PYTHON: route = j.id
                  route = j->Id;
               }
            }
         }
         
         //PYTHON: if d == infinity :
         if (d == DIJKSTRA_INFINITY) {
            if (verbose >= 5) 
               printf("Self-distance for %d is infinite (no routes to self)\n", i->Id);
         } else {
            //PYTHON: distances[i.id][i.id] = d
            distances[i->Id][i->Id] = d; 
            //PYTHON: routes[i.id][i.id] = route
            routes[i->Id][i->Id] = route; 
            if (verbose >= 4) 
               printf("Self-distance for %d is %d via %d\n", i->Id, d, route);
         }
      }
   }
   
   //PYTHON: max_distance = 0
   maxDistance = 0; 
   //PYTHON: for i in graph.vertices :
   for (ii = 0; ii <  graphp->VerticesLength; ii++) {
      if ((i = graphp->Vertices[ii])) {
         //PYTHON: for j in graph.vertices :
         for (jj = 0; jj < graphp->VerticesLength; jj++) {
            //PYTHON: if i.refcount and j.refcount :
            if (i->RefCount && ((j = graphp->Vertices[jj]) && j->RefCount)) {
               //PYTHON: d = get_distance(distances, i.id, j.id)
               if (FERROR == (status = CLDijkstraGetDistance(distances, nRows, nCols, i->Id, j->Id, &d))) {
                  fprintf(stderr, "Error, unable to get distance from vertex %d to vertex %d\n", i->Id, j->Id); 
                  goto fail;
               }
               
               if (verbose >= 5 && d == DIJKSTRA_INFINITY) 
                  fprintf(stderr, "Warning, no path from %d to %d\n", i->Id, j->Id); 
               else if (verbose >= 4 && d != DIJKSTRA_INFINITY) 
                  printf("Distance from vertex %d to vertex %d is %d arcs\n", 
                         i->Id, j->Id, d); 
               
               //PYTHON: if d > max_distance and d != infinity :
               if (d > maxDistance && d != DIJKSTRA_INFINITY) 
                  maxDistance = d;    //PYTHON: max_distance = d
            }
         }
      }
   }
   
   if (verbose >= 3) 
      printf("Maximum distance is %d\n", maxDistance); 
   
   // deallocate temporary buffers
   if (dist) 
      MemoryDeallocate(dist); 
   if (route) 
      MemoryDeallocate(route); 
   if (previous) 
      MemoryDeallocate(previous); 
   
   //PYTHON: check_distances_and_routes(graph, distances, routes)
   CLDijkstraCheckDistancesAndRoutes(graphp, distances, routes, nRows, nCols, verbose); 
   
   // initialize response data
   respData->distances = distances; 
   respData->routes = routes; 
   respData->nRows = nRows; 
   respData->nCols = nCols; 
   
   return FSUCCESS; 
   
fail:
   
   // deallocate multidimensional arrays
   (void)CLDijkstraFreeArray((void **)distances, nRows); 
   (void)CLDijkstraFreeArray((void **)routes, nRows); 
   
   // deallocate temporary buffers
   if (dist) 
      MemoryDeallocate(dist); 
   if (route) 
      MemoryDeallocate(route); 
   if (previous) 
      MemoryDeallocate(previous); 
   
   return status;
}

//PYTHON: def find_cycles (graph, distances, routes, fn) :
void CLDijkstraFindCycles(FabricData_t *fabricp, 
                          clGraphData_t *graphp, 
                          clDijkstraDistancesAndRoutes_t *drp, 
                          ValidateCLLinkSummaryCallback_t linkSummaryCallback, 
                          ValidateCLLinkStepSummaryCallback_t linkStepSummaryCallback, 
                          ValidateCLPathSummaryCallback_t pathSummaryCallback, 
                          void *context) 
{ 
   uint32 ii, dii, next_id, cycles = 0;        //PYTHON: cycles = 0
   int done_vertices_entries = 0, *done_vertices = NULL; 
   uint32 cycle_histogram_entries = 0, *cycle_histogram = NULL, *distance_list = NULL; 
   clVertixData_t *i = NULL; 
   ValidateCreditLoopRoutesContext_t *cp = (ValidateCreditLoopRoutesContext_t *)context; 
   int xmlFmt = (cp->format == 1) ? 1 : 0; 
   int verbose = (xmlFmt) ? 0 : cp->detail; 
   cl_qmap_t cycleHistogramLstMap, doneVerticesLstMap; 
   
   
   if (!(done_vertices     = MemoryAllocate2AndClear(drp->nCols * sizeof(int), IBA_MEM_FLAG_PREMPTABLE, MYTAG)) ||  //PYTHON: done_vertices = []
       !(cycle_histogram   = MemoryAllocate2AndClear(drp->nCols * sizeof(uint32), IBA_MEM_FLAG_PREMPTABLE, MYTAG)) ||  //PYTHON: cycle_histogram = {}
       !(distance_list     = MemoryAllocate2AndClear(drp->nCols * sizeof(uint32), IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
      fprintf(stderr, "%s: Unable to allocate memory\n", g_Top_cmdname); 
      goto done;
   }
   
   cl_qmap_init(&doneVerticesLstMap, NULL); 
   cl_qmap_init(&cycleHistogramLstMap, NULL); 
   memset(done_vertices, -1, sizeof(*done_vertices) * drp->nCols); 
   
   //PYTHON: for i in graph.vertices :
   for (ii = 0; ii < graphp->VerticesLength; ii++) {
      //PYTHON: if i.refcount and not i.id in done_vertices :
      if ((i = graphp->Vertices[ii]) && 
          (i->RefCount && !CLListIntFind(&doneVerticesLstMap, i->Id))) {   //CJKING: (i->RefCount && !ListIntFind(i->Id, done_vertices, done_vertices_entries, verbose))) {
                                                                           //PYTHON: dii = get_distance(distances, i.id, i.id)
         if (FERROR == CLDijkstraGetDistance(drp->distances, drp->nRows, drp->nCols, i->Id, i->Id, &dii)) {
            fprintf(stderr, "Error, unable to find cycles: distances not specified\n"); 
            goto done;
         }
         
         //PYTHON: if dii != infinity :
         if (dii != DIJKSTRA_INFINITY) {
            int indent = cp->indent;  //4;
            uint32 step; 
            clVertixData_t *j; 
            
            if (dii || verbose >= 3) {
               (void)linkSummaryCallback(i->Id, ib_connection_source_to_str(fabricp, i->Connection), dii, 1, indent, context); 
               if (verbose >= 4)
                   (void)pathSummaryCallback(fabricp, i->Connection, indent + 4, context);                                
            }
            
            j = i;      //PYTHON: j = i
            cycles++;   //PYTHON: cycles += 1
            
            //PYTHON: if dii in cycle_histogram :
            //CJKING: if (CLListUint32Find(dii, cycle_histogram, drp->nCols, verbose))
            if (CLListUint32Find(&cycleHistogramLstMap, dii)) 
               cycle_histogram[dii]++;     //PYTHON: cycle_histogram[dii] += 1
            else {
               //CJKING: Add entry to search list for cycle_histogram array 
               if (CLListUint32Add(&cycleHistogramLstMap, dii)) 
                  goto done; 
               else 
                  cycle_histogram[dii] = 1;   //PYTHON: cycle_histogram[dii] = 1
            }
            cycle_histogram_entries = MAX(cycle_histogram_entries, dii); 
            
            //PYTHON: if routes :
            if (drp->routes) {
               //PYTHON: for step in range(0, dii) :
               for (step = 0; step < dii; step++) {
                  (void)linkStepSummaryCallback(j->Id, ib_connection_source_to_str(fabricp, j->Connection), step, 1, indent + 4, context);
                  if (verbose >= 4)
                      (void)pathSummaryCallback(fabricp, j->Connection, indent + 8, context);                   
                  // insert XML token to indicate the end of link step summary section
                  if (xmlFmt) 
                     (void)linkStepSummaryCallback(0, 0, 0, 0, indent + 4, context); 
                  
                  done_vertices[done_vertices_entries++] = j->Id; //PYTHON: done_vertices.append(j.id)                        
                                                                  //CJKING: Add entry to search list for done_vertices array 
                  if (CLListIntAdd(&doneVerticesLstMap, j->Id)) 
                     goto done; 
                  next_id = drp->routes[j->Id][i->Id];            //PYTHON: next_id = routes[j.id][i.id]                        
                  j = graphp->Vertices[next_id];                  //PYTHON: j = graph.vertices[next_id]
               }
            } else {
               uint8 found = 0; 
               
               //PYTHON: for step in range(0, dii) :
               for (step = 0; step < dii; step++) {
                  uint32 jj; 
                  
                  (void)linkStepSummaryCallback(j->Id, ib_connection_source_to_str(fabricp, j->Connection), step, 1, indent + 4, context); 
                  if (verbose >= 4)
                      (void)pathSummaryCallback(fabricp, j->Connection, indent + 8, context); 
                  // insert XML token to indicate the end of link step summary section
                  if (xmlFmt) 
                     (void)linkStepSummaryCallback(0, 0, 0, 0, indent + 4/*indent*/, context); 
                  
                  done_vertices[done_vertices_entries++] = j->Id;     //PYTHON: done_vertices.append(j.id)
                                                                      //CJKING: Add entry to search list for done_vertices array 
                  if (CLListIntAdd(&doneVerticesLstMap, j->Id)) 
                     goto done; 
                  found = 0;                        
                  memset(distance_list, 0, sizeof(drp->nCols * sizeof(uint32)));    //PYTHON: distance_list = []
                  
                  //PYTHON: for arc_id in j.outbound :
                  for (jj = 0; jj < j->OutboundCount; jj++) {
                     clVertixData_t *k; 
                     clArcData_t *arcp; 
                     
                     if (!(j->Outbound[jj] >= 0 && (arcp = CLGraphFindIdArc(graphp, j->Outbound[jj])))) {
                        fprintf(stderr, "Warning, arc not available cannot follow connection for vertex %d\n", jj); 
                        continue;
                     }
                     
                     //PYTHON: k = graph.vertices[graph.arcs[arc_id].sink]
                     k = graphp->Vertices[arcp->Sink]; 
                     //PYTHON: if k.refcount :
                     if (k->RefCount) {
                        uint32 dki; 
                        
                        //PYTHON: dki = get_distance(distances, k.id, i.id)
                        if (FERROR == CLDijkstraGetDistance(drp->distances, drp->nRows, drp->nCols, k->Id, i->Id, &dki)) {
                           fprintf(stderr, "Error, unable to find cycles: distances not specified\n"); 
                           goto done;
                        }
                        /*
                          #print '  %d: vertex %d via arc %d to vertex %d' % \
                          #      (step, j.id, arc_id, k.id)
                          found = 1
                          j = k
                        */
                        //PYTHON: if dki != infinity :
                        if (dki != DIJKSTRA_INFINITY) 
                           distance_list[dki] = dki;   //PYTHON: distance_list.append(dki)
                        
                        if (dki != DIJKSTRA_INFINITY &&                 //PYTHON: if dki != infinity and
                            ((dki == dii - step - 1) ||                 //PYTHON:    ((dki == dii - step - 1) or
                             (k->Id == i->Id && step == dii - 1))) {    //PYTHON:     (k.id == i.id and step == dii - 1))
                                                                        //printf("  %d: vertex %d via arc %d to vertex %d\N", step, j->Id, arc_id, k->Id);
                           found = 1;  //PYTHON: found = 1
                           j = k;      //PYTHON: j = k
                        }
                        break;
                     }
                  }
                  //PYTHON: if not found :
                  if (!found) {
                     char *s = ib_connection_source_to_str(fabricp, i->Connection);
                     printf("  %d: could not find cycle for vertex %d (%s) on step %d of %d (warning), looking for distance %d\n", 
                            step, i->Id, (s)?(s):"<null>", step, dii, dii - step - 1); 
                     break;
                  }
               }
            }
            
            // insert XML token to indicate the end of link summary section
            if (xmlFmt) 
               (void)linkSummaryCallback(0, 0, 0, 0, cp->indent/*0*/, context);
         }
      }
   }
   
   if (verbose >= 3) {
      if (!cycles) 
         printf("Routes are deadlock free!\n"); 
      else {
         uint32 c; 
         
         printf("Deadlock - dependency graph routes contain %d cycle(s):\n", cycles); 
         //PYTHON: for c in cycle_histogram :
         for (c = 0; c < cycle_histogram_entries; c++) printf("  %d cycle(s) of circumference %d\n", cycle_histogram[c], c);
      }
   }
   
done:
   
   if (done_vertices) 
      MemoryDeallocate(done_vertices); 
   if (cycle_histogram) 
      MemoryDeallocate(cycle_histogram); 
   if (distance_list) 
      MemoryDeallocate(distance_list);
}

#endif
