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

// functions to perform routing analysis using LFT tables in FabricData

#include <opamgt_sa_priv.h>

#include "topology.h"
#include "topology_internal.h"

// For each device along the path, entry and exit port of device is provided
// For the FI at the start of the route, only an exit port is provided
// For the FI at the end of the route, only a entry port is provided
// For switches along the route, both entry and exit ports are provided
// When a switch Port 0 is the start of the route, it will be the entry port
//    along with the physical exit port
// When a switch Port 0 is the end of the route, it will be the exit port
//    along with the physical entry port
// The above approach parallels how TraceRoute records are reported by the
// SM, so if desired a callback could build a SM TraceRoute style response
// for use in other routines.
//typedef FSTATUS (RouteCallback_t)(PortData *entryPortp, PortData *exitPortp, void *context);


// lookup dlid in routing table of switch and return portp to exit switch
static PortData *LookupRoute(NodeData *nodep, STL_LID dlid, PortData* inportp, int vl, int sc, int rc)
{
	uint8 portNum = 0xff;
	PortData *portp;

	DEBUG_ASSERT(nodep->switchp);	//caller checks
	if (!dlid)
		return NULL;

	if (nodep->pSwitchInfo->SwitchInfoData.RoutingMode.Enabled == STL_ROUTE_LINEAR) {
		DEBUG_ASSERT(nodep->switchp->LinearFDB);

		if (dlid >= nodep->switchp->LinearFDBSize)
			return NULL;

		portNum = STL_LFT_PORT_BLOCK(nodep->switchp->LinearFDB, dlid);
	}

	if (portNum == 0xff)
		return NULL;	// invalid table entry, no route

	portp = FindNodePort(nodep, portNum);
	// Analysis is focused on datapath routes.  While VL15 can route through an
	// Init port, that analysis is atypical.  Prior to FF_DOWNPORTINFO
	// we would tend not to have ports Down or Init in our DB so
	// search above would fail.
	if (! portp || ! IsPortInitialized(portp->PortInfo.PortStates)
		|| (portNum != 0 && ! portp->neighbor))
		return NULL;	// Non viable route
	return portp;
}

// Walk route from portp to dlid.
// returns status of callback (if not FSUCCESS)
// FUNAVAILABLE - no routing tables in FabricData given
// FNOT_FOUND - unable to find starting port
// FNOT_DONE - unable to trace route, dlid is a dead end
FSTATUS WalkRoutePort(FabricData_t *fabricp, PortData *portp, STL_LID dlid, uint8 SL, uint8 rc,
			  		RouteCallback_t *callback, void *context)
{
	PortData *portp2;	// next port in route
	FSTATUS status;
	PortData *hops[64];
	uint8 numhops = 0;
	uint8 sc = 0, vl = 0;
	if (portp->pQOS) {
		sc = portp->pQOS->SL2SCMap->SLSCMap[SL].SC;
		if (sc == 15) {
			return FNOT_DONE;	// invalid SC
		}
		vl = portp->pQOS->SC2VLMaps[Enum_SCVLt].SCVLMap[sc].VL;
		if (vl == 15) {
			return FNOT_DONE;
		}
	}		

	if (portp->nodep->NodeInfo.NodeType != STL_NODE_SW) {
		// first device in route
		status = (*callback)(NULL, portp, vl, context);
		if (status != FSUCCESS)
			return status;
		portp = portp->neighbor;	// entry port to next device
	}

	// 1st loop we can start at port 0 of a switch.  If we arrive at port 0
	// of a switch any other loop, it must be our destination.
	while (portp->nodep->NodeInfo.NodeType == STL_NODE_SW
				&& (numhops == 0 || portp->PortNum != 0)) {
		SwitchData *switchp;
		uint8 i;

		if (numhops >= 64)
			return FNOT_DONE;	// too long a path
		for (i=0; i< numhops; i++) {
			if (hops[i] == portp)
				return FNOT_DONE;	// looping path
		}
		hops[numhops++] = portp;

		// portp is entry port to a switch
		switchp = portp->nodep->switchp;
		if (!switchp || (!switchp->LinearFDB &&
			(portp->nodep->pSwitchInfo->SwitchInfoData.RoutingMode.Enabled == STL_ROUTE_NOP ||
			 portp->nodep->pSwitchInfo->SwitchInfoData.RoutingMode.Enabled == STL_ROUTE_LINEAR)))
			// TBD - way to check this before start
			return FUNAVAILABLE;	// no routing tables in snapshot

		portp2 = LookupRoute(portp->nodep, dlid, portp, vl, sc, rc);
		if (! portp2)
			return FNOT_DONE;	// no route from slid to dlid

		if (portp->pQOS && portp->PortNum != 0 && portp2->PortNum != 0) {
			STL_SCSCMAP *pSCSC = QOSDataLookupSCSCMap(portp, portp2->PortNum, 0);
			if (pSCSC) {
				uint8 newsc = pSCSC->SCSCMap[sc].SC;
				if (newsc == 15) {
					return FNOT_DONE; // invalid SC
				}
				vl = portp->pQOS->SC2VLMaps[Enum_SCVLt].SCVLMap[newsc].VL;
				if (vl == 15) {
					return FNOT_DONE; // invalid VL
				}
				sc = newsc;
			}
		}

		// hop through a switch
		status = (*callback)(portp, portp2, vl, context);
		if (status != FSUCCESS)
			return status;

		// find next device
		if (portp2->nodep->NodeInfo.NodeType == STL_NODE_SW
			&& portp2->PortNum == 0)
			portp = portp2;
		else
			portp = portp2->neighbor;	// entry port to next device
	}

	// at destination of dlid: FI or Port 0 of SW
	if (dlid < portp->PortInfo.LID
		|| dlid > (portp->PortInfo.LID
				 | ((1<<portp->PortInfo.s1.LMC)-1)) )
		return FNOT_DONE;	// arrived at wrong destination

	if (portp->nodep->NodeInfo.NodeType != STL_NODE_SW) {
		// last device in route
		status = (*callback)(portp, NULL, vl, context);
		if (status != FSUCCESS)
			return status;
	}
	return FSUCCESS;
}

// walk route from slid to dlid
FSTATUS WalkRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid,
			  		RouteCallback_t *callback, void *context)
{
	PortData *portp;

	portp = FindLid(fabricp, slid);
	if (! portp)
		return FNOT_FOUND;

	return WalkRoutePort(fabricp, portp, dlid, 0, 0, callback, context);
}

struct GenTraceRouteContext_s {
	uint32 NumTraceRecords;
	STL_TRACE_RECORD *pTraceRecords;
};

static FSTATUS GenTraceRouteCallback(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	struct GenTraceRouteContext_s *TraceContext = (struct GenTraceRouteContext_s *)context;
	STL_TRACE_RECORD *p;
	NodeData *nodep;

	ASSERT(entryPortp || exitPortp);
	ASSERT(! (entryPortp && exitPortp) ||entryPortp->nodep == exitPortp->nodep);

	// add 1 more trace record
	TraceContext->NumTraceRecords++;
	p = TraceContext->pTraceRecords;
	TraceContext->pTraceRecords = (STL_TRACE_RECORD*)MemoryAllocate2AndClear(sizeof(STL_TRACE_RECORD)*TraceContext->NumTraceRecords,  IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! TraceContext->pTraceRecords) {
		if (p)
			MemoryDeallocate(p);
		return FINSUFFICIENT_MEMORY;
	}

	if (p) {
		MemoryCopy(TraceContext->pTraceRecords, p, sizeof(STL_TRACE_RECORD)*(TraceContext->NumTraceRecords-1));
		MemoryDeallocate(p);
	}
	p = &TraceContext->pTraceRecords[TraceContext->NumTraceRecords-1];

	nodep = (entryPortp?entryPortp->nodep:exitPortp->nodep);

	p->IDGeneration = 0xff;	// should be same for all records in a route
	p->NodeType = nodep->NodeInfo.NodeType;
	p->NodeID = nodep->NodeInfo.NodeGUID;
	p->ChassisID = nodep->NodeInfo.SystemImageGUID;
	
	if(p->NodeType == STL_NODE_SW) {
		p->EntryPort = entryPortp?entryPortp->PortNum : 0;
		p->ExitPort = exitPortp?exitPortp->PortNum : 0;
		p->EntryPortID = 0;

	} else {

		//entry and exit port should be 0 if not a switch
		p->EntryPort = 0;
		p->ExitPort = 0;
		p->EntryPortID = entryPortp? entryPortp->PortGUID : 0;
		if(p->EntryPortID == 0) p->EntryPortID = exitPortp? exitPortp->PortGUID : 0;
	}

	//Exit port ID should be 0 if not a router
	p->ExitPortID = 0;

	
	return FSUCCESS;
}


// caller must free *ppTraceRecords
FSTATUS GenTraceRoutePort(FabricData_t *fabricp, PortData *portp, STL_LID dlid, uint8 rc,
	   			STL_TRACE_RECORD **ppTraceRecords, uint32 *pNumTraceRecords)
{
	struct GenTraceRouteContext_s context = { 0, NULL};
	FSTATUS status;

	status = WalkRoutePort(fabricp, portp, dlid, 0, rc, GenTraceRouteCallback, &context);

	if (status != FSUCCESS) {
		if (context.pTraceRecords)
			MemoryDeallocate(context.pTraceRecords);
		*ppTraceRecords = NULL;
		*pNumTraceRecords = 0;
		return status;
	}

	*ppTraceRecords = context.pTraceRecords;
	*pNumTraceRecords = context.NumTraceRecords;
	return FSUCCESS;
}

// caller must free *ppTraceRecords
FSTATUS GenTraceRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid, uint8 rc,
	   			STL_TRACE_RECORD **ppTraceRecords, uint32 *pNumTraceRecords)
{
	PortData *portp;

	portp = FindLid(fabricp, slid);
	if (! portp) {
		*ppTraceRecords = NULL;
		*pNumTraceRecords = 0;
		return FNOT_FOUND;
	}

	return GenTraceRoutePort(fabricp, portp, dlid, rc, ppTraceRecords, pNumTraceRecords);
}

// caller must free *ppTraceRecords
FSTATUS GenTraceRoutePath(FabricData_t *fabricp, IB_PATH_RECORD *pathp, uint8 rc,
	   			STL_TRACE_RECORD **ppTraceRecords, uint32 *pNumTraceRecords)
{
	return GenTraceRoute(fabricp, pathp->SLID, pathp->DLID, rc,
				   			ppTraceRecords, pNumTraceRecords);
}

static FSTATUS GenPath(PortData *portp1, PortData *portp2,
			   	STL_LID slid, STL_LID dlid,
			   	IB_PATH_RECORD **ppPathRecords, uint32 *pNumPathRecords)
{
	IB_PATH_RECORD *p;

	ASSERT(portp1 && portp2);

	// add 1 more path record
	(*pNumPathRecords)++;
	p = *ppPathRecords;
	*ppPathRecords = (IB_PATH_RECORD*)MemoryAllocate2AndClear(sizeof(IB_PATH_RECORD)* *pNumPathRecords,  IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! *ppPathRecords) {
		if (p)
			MemoryDeallocate(p);
		return FINSUFFICIENT_MEMORY;
	}

	if (p) {
		MemoryCopy(*ppPathRecords, p, sizeof(IB_PATH_RECORD)*(*pNumPathRecords-1));
		MemoryDeallocate(p);
	}
	p = &(*ppPathRecords)[*pNumPathRecords-1];

	p->SGID.Type.Global.SubnetPrefix = portp1->PortInfo.SubnetPrefix;
	p->SGID.Type.Global.InterfaceID = portp1->PortGUID;
	p->DGID.Type.Global.SubnetPrefix = portp2->PortInfo.SubnetPrefix;
	p->DGID.Type.Global.InterfaceID = portp2->PortGUID;
	p->SLID = slid;
	p->DLID = dlid;
	p->Reversible = 1;
	p->u1.s.RawTraffic = 0;
	p->u1.s.FlowLabel = 0;
	p->u1.s.HopLimit = 0;
	p->TClass = 0;
	p->P_Key = 0;	// invalid value to show we don't know
	p->PktLifeTime = 0;	// invalid value to show we don't know
	p->u2.s.SL = 0;	// best guess, could be wrong for Torus
	p->Mtu = 0;	// invalid value to show we don't know
	p->Rate = 0;	// invalid value to show we don't know
	p->Preference = 0;
	// other fields zeroed above when allocate w/clear
	
	return FSUCCESS;
}

// Generate possible Path records from portp1 to portp2
// We don't know SM config, so we just guess and generate paths of the form
// 0-0, 1-1, ....
// This corresponds to the PathSelection=Minimal FM config option
// when LMC doesn't match we start back at base lid for other port
// These may not be accurate for Torus, however the dlid is all that really
// matters for route analysis, so this should be fine
FSTATUS GenPaths(FabricData_t *fabricp, PortData *portp1, PortData *portp2,
		IB_PATH_RECORD **ppPathRecords, uint32 *pNumPathRecords)
{
	STL_LID slid_base, slid_mask, slid_offset, dlid_base, dlid_mask, dlid_offset;
	FSTATUS status;

	*ppPathRecords = NULL;
	*pNumPathRecords = 0;

	slid_base = portp1->PortInfo.LID;
	slid_mask = (1<<portp1->PortInfo.s1.LMC)-1;

	dlid_base = portp2->PortInfo.LID;
	dlid_mask = (1<<portp2->PortInfo.s1.LMC)-1;

	if (! slid_base || ! dlid_base)
		return FSUCCESS;	// no path, probably a non port 0 on a switch

	for (slid_offset=0, dlid_offset=0;
			slid_offset <= slid_mask && dlid_offset <= dlid_mask;
	   		slid_offset++, dlid_offset++) {
		status = GenPath(portp1, portp2,
			   	slid_base | (slid_offset & slid_mask),
			   	dlid_base | (dlid_offset & dlid_mask),
			   	ppPathRecords, pNumPathRecords);
		if (status != FSUCCESS) {
			if (*ppPathRecords)
				MemoryDeallocate(*ppPathRecords);
			*ppPathRecords = NULL;
			*pNumPathRecords = 0;
			return status;
		}
	}
	return FSUCCESS;
}


// clear holding areas for analysis data
void ClearAnalysisData(FabricData_t *fabricp)
{
	cl_map_item_t *n;
	cl_map_item_t *p;

	for (n=cl_qmap_head(&fabricp->AllNodes); n != cl_qmap_end(&fabricp->AllNodes); n = cl_qmap_next(n)) {
		NodeData *nodep = PARENT_STRUCT(n, NodeData, AllNodesEntry);
		nodep->analysis = 0;
		for (p=cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p)) {
			PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
			MemoryClear(&portp->analysisData, sizeof(portp->analysisData));
		}
	}
}

// for Fat Tree analysis
void DetermineSwitchTiers(FabricData_t *fabricp)
{
	LIST_ITEM *n;
	cl_map_item_t *p;
	boolean found;
	unsigned tier;

	// switches connected to FIs are tier 1
	for (n=QListHead(&fabricp->AllFIs); n != NULL; n = QListNext(&fabricp->AllFIs, n)) {
		NodeData *nodep = (NodeData *)QListObj(n);
		for (p=cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p)) {
			PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
			if (portp->neighbor && portp->neighbor->nodep->NodeInfo.NodeType == STL_NODE_SW)
				portp->neighbor->nodep->analysis = 1;
		}
	}

	// switches connected to tier1 switches are tier 2, etc
	tier=2;
	do {
		found = FALSE;
		for (n=QListHead(&fabricp->AllSWs); n != NULL; n = QListNext(&fabricp->AllSWs, n)) {
			NodeData *nodep = (NodeData *)QListObj(n);
			if (nodep->analysis != tier-1)
				continue;
			for (p=cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p)) {
				PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
				if (portp->neighbor
						&& portp->nodep->NodeInfo.NodeType == STL_NODE_SW
					   	&& ! portp->neighbor->nodep->analysis) {
					portp->neighbor->nodep->analysis = tier;
					found = TRUE;
				}
			}
		}
		tier++;
	} while (found);
}

// tabulate a device along a route in a fat tree
// called for each device in each route
// context != NULL  => non-base LID path
static FSTATUS TabulateRouteCallbackFatTree(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	if (exitPortp) {
		if (exitPortp->neighbor && exitPortp->nodep->analysis < exitPortp->neighbor->nodep->analysis) {
			exitPortp->analysisData.fatTreeRoutes.uplinkAllPaths++;
			if (! context)
				exitPortp->analysisData.fatTreeRoutes.uplinkBasePaths++;
		} else {
			// for now == tier or no neighbor unexpected, but treat as downlink
			exitPortp->analysisData.fatTreeRoutes.downlinkAllPaths++;
			if (! context)
				exitPortp->analysisData.fatTreeRoutes.downlinkBasePaths++;
		}
	}
	return FSUCCESS;
}

// tabulate a device along a route
// called for each device in each route
// context != NULL  => non-base LID path
static FSTATUS TabulateRouteCallback(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	if (entryPortp) {
		// increment a counter in entryPortp, maybe context indicates which
		entryPortp->analysisData.routes.recvAllPaths++;
		if (! context)
			entryPortp->analysisData.routes.recvBasePaths++;
	}
	
	if (exitPortp) {
		// increment a counter in exitPortp, maybe context indicates which
		exitPortp->analysisData.routes.xmitAllPaths++;
		if (! context)
			exitPortp->analysisData.routes.xmitBasePaths++;
	}
	return FSUCCESS;
}

// tabulate all the ports along the route from slid to dlid
FSTATUS TabulateRoute(FabricData_t *fabricp, STL_LID slid, STL_LID dlid,
						boolean fatTree)
{
	return WalkRoute(fabricp, slid, dlid,
				   	fatTree?TabulateRouteCallbackFatTree:TabulateRouteCallback,
				   	NULL);
}

// tabulate all routes from portp1 to portp2
FSTATUS TabulateRoutes(FabricData_t *fabricp, PortData *portp1,
			   			PortData *portp2, uint32 *totalPaths, 
						uint32 *badPaths, boolean fatTree)
{
	int  offset;
	int  count = (1<<portp1->PortInfo.s1.LMC);
	FSTATUS status;

	*totalPaths = 0;
	*badPaths = 0;

	// IB is destination routed, so just need starting port, no need to
	// iterate on all SLIDs for that port
	status = WalkRoutePort(fabricp, portp1,
				   portp2->PortInfo.LID, 0, 0,
				   fatTree?TabulateRouteCallbackFatTree:TabulateRouteCallback,
				   NULL);	// Base LID
	if (status == FUNAVAILABLE)
		return status;
	(*totalPaths)++;
	if (status == FNOT_DONE)
		(*badPaths)++;

	for (offset = 1; offset < count; offset++) {
		status = WalkRoutePort(fabricp, portp1,
				   portp2->PortInfo.LID|offset, 0, 0,
				   fatTree?TabulateRouteCallbackFatTree:TabulateRouteCallback,
				   (void*)1);	// LMC LID
		if (status == FUNAVAILABLE)
			return status;
		(*totalPaths)++;
		if (status == FNOT_DONE)
			(*badPaths)++;
	}
	return FSUCCESS;
}

// tabulate all the routes between FIs
FSTATUS TabulateCARoutes(FabricData_t *fabricp, Point *focus, uint32 *totalPaths,
							uint32 *badPaths, boolean fatTree)
{
	LIST_ITERATOR i, j;
	cl_map_item_t *p1, *p2;
	LIST_ITEM *n1, *n2;
	FSTATUS status;
	uint32 pathCount, badPathCount;
	int noOfLeftNodes, noOfRightNodes;

	*totalPaths = 0;
	*badPaths = 0;

	ClearAnalysisData(fabricp);

	if (fatTree)
		DetermineSwitchTiers(fabricp);

	/* If there is FI in the node pair list only tabulate routes for the specified pairs*/
	if(PointHaveFI(focus) && PointTypeIsNodePairList(focus)){

		DLIST *pList1 = &focus->u.nodePairList.nodePairList1;
		DLIST *pList2 = &focus->u.nodePairList.nodePairList2;

		noOfLeftNodes = ListCount(pList1);
		noOfRightNodes = ListCount(pList2);

		if (noOfLeftNodes != noOfRightNodes) {
			fprintf(stderr, "%s: Pairs are not complete \n", g_Top_cmdname);
			return FINVALID_PARAMETER;
		}

		for (i = ListHead(pList1), j = ListHead(pList2); (i != NULL && j != NULL);
				i = ListNext(pList1, i), j = ListNext(pList2, j) ) {
			NodeData *nodepFromList1 = (NodeData*)ListObj(i);
			NodeData *nodepFromList2 = (NodeData*)ListObj(j);

			//skip switches
			if ((nodepFromList1->NodeInfo.NodeType == STL_NODE_SW) || (nodepFromList2->NodeInfo.NodeType == STL_NODE_SW))
				continue;

			for (p1=cl_qmap_head(&nodepFromList1->Ports); p1 != cl_qmap_end(&nodepFromList1->Ports); p1 = cl_qmap_next(p1)) {
				PortData *portp1 = PARENT_STRUCT(p1, PortData, NodePortsEntry);

				for (p2=cl_qmap_head(&nodepFromList2->Ports); p2 != cl_qmap_end(&nodepFromList2->Ports); p2 = cl_qmap_next(p2)) {
					PortData *portp2 = PARENT_STRUCT(p2, PortData, NodePortsEntry);

					// skip loopback paths
					if (portp1 == portp2) continue;
					status = TabulateRoutes(fabricp, portp1, portp2, &pathCount, &badPathCount, fatTree);
					if (status == FUNAVAILABLE)
						return status;
					*totalPaths += pathCount;
					*badPaths += badPathCount;
				}
			}
		}
	/* If there is FI in the list, make N x N pairs and  tabulate routes for the formed pairs*/
	}else if(PointHaveFI(focus)){
		FIPortIterator a, b;
		PortData *portp1, *portp2;
		//point type which haveFI and only matches 1 node is invalid and should return error
		if((POINT_TYPE_PORT == focus->Type) || (POINT_TYPE_NODE == focus->Type) ||
#if !defined(VXWORKS) || defined(BUILD_DMC)
			(POINT_TYPE_IOC == focus->Type) ||
			((POINT_TYPE_IOC_LIST == focus->Type) && (1 == ListCount(&focus->u.iocList)))||
#endif
			((POINT_TYPE_PORT_LIST == focus->Type) && (1 == ListCount(&focus->u.portList)))||
			((POINT_TYPE_NODE_LIST == focus->Type) && (1 == ListCount(&focus->u.nodeList)))||
			((POINT_TYPE_SYSTEM == focus->Type) && (1 == cl_qmap_count(&focus->u.systemp->Nodes)))){
			status = FINVALID_PARAMETER;
			return status;
		}
		for(portp1 = FIPortIteratorHead(&a, focus); portp1 != NULL; portp1 = FIPortIteratorNext(&a) ) {
			for(portp2 = FIPortIteratorHead(&b, focus); portp2 != NULL; portp2 = FIPortIteratorNext(&b) ) {
				// skip loopback paths
				if (portp1 == portp2)
					continue;
				status = TabulateRoutes(fabricp, portp1, portp2, &pathCount, &badPathCount, fatTree);
				if (status == FUNAVAILABLE)
					return status;
				*totalPaths += pathCount;
				*badPaths += badPathCount;
			}
		}
	} else {
		// TBD - because IB is DLID routed, can save effort by getting routes from
		// 1 FI per switch to all other FIs (or all FIs on other switches) and
		// then multiply the result for that FI by the number of FIs on that switch
		// In a pure fattree such an approach would reduce N*N to (N/18)*(N/18)
		// so it would have a 18*18 reduction in effort, provided the cost of
		// finding FIs is not to high, maybe we can loop here based on all switches?
		for (n1=QListHead(&fabricp->AllFIs); n1 != NULL; n1 = QListNext(&fabricp->AllFIs, n1)) {
			NodeData *nodep1 = (NodeData *)QListObj(n1);
			for (p1=cl_qmap_head(&nodep1->Ports); p1 != cl_qmap_end(&nodep1->Ports); p1 = cl_qmap_next(p1)) {
				PortData *portp1 = PARENT_STRUCT(p1, PortData, NodePortsEntry);

				for (n2=QListHead(&fabricp->AllFIs); n2 != NULL; n2 = QListNext(&fabricp->AllFIs, n2)) {
					NodeData *nodep2 = (NodeData *)QListObj(n2);

					// enable this if we want to skip node to self paths
					//if (nodep1 == nodep2) continue;

					for (p2=cl_qmap_head(&nodep2->Ports); p2 != cl_qmap_end(&nodep2->Ports); p2 = cl_qmap_next(p2)) {
						PortData *portp2 = PARENT_STRUCT(p2, PortData, NodePortsEntry);
						// skip loopback paths
						if (portp1 == portp2) continue;
							status = TabulateRoutes(fabricp, portp1, portp2, &pathCount, &badPathCount, fatTree);
						if (status == FUNAVAILABLE)
							return status;
						*totalPaths += pathCount;
						*badPaths += badPathCount;
					}
				}
			}
		}
	}
	return FSUCCESS;
}

typedef struct ReportContext_s {
	PortData *portp1;
	PortData *portp2;
	STL_LID dlid;
	boolean isBaseLid;
	PortData *reportPort;
	ReportCallback_t callback;
	void *context;
} ReportContext_t;

// report a device along a route in a fat tree
// called for each device in each route
static FSTATUS ReportRouteCallbackFatTree(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	ReportContext_t *ReportContext = (ReportContext_t*)context;

	if (exitPortp == ReportContext->reportPort) {
		if (exitPortp->neighbor && exitPortp->nodep->analysis < exitPortp->neighbor->nodep->analysis) {
			(*ReportContext->callback)(ReportContext->portp1, ReportContext->portp2, ReportContext->dlid, ReportContext->isBaseLid, TRUE, ReportContext->context);
		} else {
			// for now == tier or no neighbor unexpected, but treat as downlink
			(*ReportContext->callback)(ReportContext->portp1, ReportContext->portp2, ReportContext->dlid, ReportContext->isBaseLid, FALSE, ReportContext->context);
		}
	}
	return FSUCCESS;
}

// report a device along a route
// called for each device in each route
static FSTATUS ReportRouteCallback(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	ReportContext_t *ReportContext = (ReportContext_t*)context;
	if (entryPortp == ReportContext->reportPort) {
		(*ReportContext->callback)(ReportContext->portp1, ReportContext->portp2, ReportContext->dlid, ReportContext->isBaseLid, TRUE, ReportContext->context);
	}
	
	if (exitPortp == ReportContext->reportPort) {
		(*ReportContext->callback)(ReportContext->portp1, ReportContext->portp2, ReportContext->dlid, ReportContext->isBaseLid, FALSE, ReportContext->context);
	}
	return FSUCCESS;
}

// report all routes from portp1 to portp2 that cross reportPort
FSTATUS ReportRoutes(FabricData_t *fabricp,
			   			PortData *portp1, PortData *portp2,
					   	PortData *reportPort, ReportCallback_t callback,
					   	void *context, boolean fatTree)
{
	int  offset;
	int  count = (1<<portp1->PortInfo.s1.LMC);
	FSTATUS status;
	ReportContext_t ReportContext;

	ReportContext.portp1 = portp1;
	ReportContext.portp2 = portp2;
	ReportContext.reportPort = reportPort;
	ReportContext.callback = callback;
	ReportContext.context = context;

	// IB is destination routed, so just need starting port, no need to
	// iterate on all SLIDs for that port
	ReportContext.dlid = portp2->PortInfo.LID;
	ReportContext.isBaseLid = TRUE;
	status = WalkRoutePort(fabricp, portp1,
				   portp2->PortInfo.LID, 0, 0,
				   fatTree?ReportRouteCallbackFatTree:ReportRouteCallback,
				   &ReportContext);	// Base LID
	if (status == FUNAVAILABLE)
		return status;
	// TabulateRoutes reports bad paths and will have been used
	// prior to this, so no need to report FNOT_DONE routes

	for (offset = 1; offset < count; offset++) {
		ReportContext.dlid = portp2->PortInfo.LID|offset;
		ReportContext.isBaseLid = FALSE;
		status = WalkRoutePort(fabricp, portp1,
				   portp2->PortInfo.LID|offset, 0, 0,
				   fatTree?ReportRouteCallbackFatTree:ReportRouteCallback,
				   &ReportContext);	// LMC LID
		if (status == FUNAVAILABLE)
			return status;
		// TabulateRoutes reports bad paths and will have been used
		// prior to this, so no need to report FNOT_DONE routes
	}
	return FSUCCESS;
}


// report all the routes between FIs that cross reportPort,
// exclude loopback routes
FSTATUS ReportCARoutes(FabricData_t *fabricp,
			   				PortData *reportPort, ReportCallback_t callback,
							void *context, boolean fatTree)
{
	LIST_ITEM *n1, *n2;
	cl_map_item_t *p1, *p2;
	FSTATUS status;

	// TBD - because IB is DLID routed, can save effort by getting routes from
	// 1 FI per switch to all other FIs (or all FIs on other switches) and
	// then multiply the result for that FI by the number of FIs on that switch
	// In a pure fattree such an approach would reduce N*N to (N/18)*(N/18)
	// so it would have a 18*18 reduction in effort, provided the cost of
	// finding FIs is not to high, maybe we can loop here based on all switches?
	for (n1=QListHead(&fabricp->AllFIs); n1 != NULL; n1 = QListNext(&fabricp->AllFIs, n1)) {
		NodeData *nodep1 = (NodeData *)QListObj(n1);
		for (p1=cl_qmap_head(&nodep1->Ports); p1 != cl_qmap_end(&nodep1->Ports); p1 = cl_qmap_next(p1)) {
			PortData *portp1 = PARENT_STRUCT(p1, PortData, NodePortsEntry);

			for (n2=QListHead(&fabricp->AllFIs); n2 != NULL; n2 = QListNext(&fabricp->AllFIs, n2)) {
				NodeData *nodep2 = (NodeData *)QListObj(n2);

				// enable this if we want to skip node to self paths
				//if (nodep1 == nodep2) continue;

				for (p2=cl_qmap_head(&nodep2->Ports); p2 != cl_qmap_end(&nodep2->Ports); p2 = cl_qmap_next(p2)) {
					PortData *portp2 = PARENT_STRUCT(p2, PortData, NodePortsEntry);
					// skip loopback paths
					if (portp1 == portp2) continue;
					status = ReportRoutes(fabricp, portp1, portp2, reportPort, callback, context, fatTree);
					if (status == FUNAVAILABLE)
						return status;
					// TabulateRoutes reports bad paths and will have been used
					// prior to this, so no need to report FNOT_DONE routes
				}
			}
		}
	}
	return FSUCCESS;
}




// callback for all the ports along a route
static FSTATUS ValidateRouteCallback(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	// nothing special to be done while walking routes
	return FSUCCESS;
}

typedef struct ValidateContext2_s {
	ValidateCallback2_t callback;
	void *context;
} ValidateContext2_t;


// callback for all the ports along an incomplete  route
static FSTATUS ValidateRouteCallback2(PortData *entryPortp, PortData *exitPortp, uint8 vl, void *context)
{
	ValidateContext2_t *ValidateContext2 = (ValidateContext2_t*)context;

	if (entryPortp)
		(*ValidateContext2->callback)(entryPortp, vl, ValidateContext2->context);

	if (exitPortp)
		(*ValidateContext2->callback)(exitPortp, vl, ValidateContext2->context);

	return FSUCCESS;
}

static FSTATUS getSLSCInfo(FabricData_t *fabricp, EUI64 portGuid, int quiet, uint32 *usedSLs, uint32 *usedSCs) {
	uint32 usedSCsSave = 0;
	LIST_ITEM *i;

	for (i = QListHead(&fabricp->AllVFs); i; i = QListNext(&fabricp->AllVFs, i)) {
		STL_VFINFO_RECORD *vfr = &((VFData_t*)QListObj(i))->record;
		(*usedSLs) |= 1 << vfr->s1.slBase;
		if (vfr->slResponseSpecified)
			(*usedSLs) |= 1 << vfr->slResponse;
	}

	if (!usedSCs) {
		return FSUCCESS;
	}

	cl_map_item_t *n;
	for (n = cl_qmap_head(&fabricp->AllNodes); n != cl_qmap_end(&fabricp->AllNodes) && (*usedSCs) != 0xff7f; n = cl_qmap_next(n)) {
		NodeData *nodep = PARENT_STRUCT(n, NodeData, AllNodesEntry);
		cl_map_item_t *p;
		for (p = cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p)) {
			PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);

			// for each host and switch port 0
			if (nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum != 0) {
				continue;
			}

			// for each SL
			int sl;
			for (sl = 0; sl < STL_MAX_SLS; sl++) {
				if (((*usedSLs) >> sl) & 1) {
					STL_SC sc = portp->pQOS->SL2SCMap->SLSCMap[sl];
					// if SL2SC is a valid SC
					if (sc.SC != 15) {
						// add SC to used SCs
						(*usedSCs) |= (1 << sc.SC);
					}
				}
			}
		}
	}

	do {
		usedSCsSave = (*usedSCs);
		// for each switch in fabric
		LIST_ITEM *sw;
		for (sw = QListHead(&fabricp->AllSWs); sw != NULL; sw = QListNext(&fabricp->AllSWs, sw)) {
			NodeData *nodep = (NodeData *)QListObj(sw);
			cl_map_item_t *p;
			// for each ingress/egress port pair
			for (p = cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p)) {
				PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);

				if (portp->PortNum == 0) {
					continue;
				}

				int e;
				for (e = 0; e < nodep->NodeInfo.NumPorts; e++) {
					uint8 sc;
					// for each used SC
					for (sc = 0; ((*usedSCs) >> sc); sc++) {
						if (((*usedSCs) >> sc) & 1) {
						// add SC2SC[ingress,egress,inputSC] to used SCs
							STL_SCSCMAP *pSCSC = QOSDataLookupSCSCMap(portp, e, 0);
							if (!pSCSC) continue;
							STL_SC newSC = pSCSC->SCSCMap[sc];
							if (newSC.SC != 15) {
								(*usedSCs) |= (1 << newSC.SC);
							}
						}
					}
				}
			}
		}
	} while ((*usedSCs) != usedSCsSave);

	return FSUCCESS;
}

// report all routes from portp1 to portp2
FSTATUS ValidateRoutes(FabricData_t *fabricp,
			   			PortData *portp1, PortData *portp2,
						uint32 *totalPaths, uint32 *badPaths,
						uint32 usedSLs, uint8 rc,
					   	ValidateCallback_t callback, void *context,
			   			ValidateCallback2_t callback2, void *context2)
{
	int  offset;
	int  count = (1<<portp2->PortInfo.s1.LMC);
	FSTATUS status = FSUCCESS;
	ValidateContext2_t ValidateContext2 ={callback:callback2, context:context2};

	*totalPaths = 0;
	*badPaths = 0;

	// IB is destination routed, so just need starting port, no need to
	// iterate on all SLIDs for that port
	// loop over the used SLs
	uint8 sl;
	for (sl = 0; sl < STL_MAX_SLS; sl++) {
 		if ((usedSLs == 0 && sl == 0) || (usedSLs >> sl) & 1) {
			status = WalkRoutePort(fabricp, portp1,
				portp2->PortInfo.LID, sl, rc,
				ValidateRouteCallback, NULL);	// Base LID
		} else {
			continue;
		}
		if (status == FUNAVAILABLE) {
			return status;
		}
		(*totalPaths)++;
		if (status != FSUCCESS) {
			(*badPaths)++;
			(*callback)(portp1, portp2, portp2->PortInfo.LID,
					   	TRUE, sl, context);
			if (callback2) {
				// re-walk route and output details of each hop
				(void)WalkRoutePort(fabricp, portp1,
							portp2->PortInfo.LID, 0, rc,
							ValidateRouteCallback2, &ValidateContext2);
				(*callback2)(NULL, 0, context2);	// close out path
			}
		}

		for (offset = 1; offset < count; offset++) {
			status = WalkRoutePort(fabricp, portp1,
				   portp2->PortInfo.LID|offset, sl, rc,
				   ValidateRouteCallback, NULL);	// LMC LID
			if (status == FUNAVAILABLE)
				return status;
			(*totalPaths)++;
			if (status != FSUCCESS) {
				(*badPaths)++;
				(*callback)(portp1, portp2, portp2->PortInfo.LID|offset,
					   	FALSE, sl, context);
				if (callback2) {
					// re-walk route and output details of each hop
					(void)WalkRoutePort(fabricp, portp1,
							portp2->PortInfo.LID, 0, rc,
							ValidateRouteCallback2, &ValidateContext2);
					(*callback2)(NULL, 0, context2); 	// close out path
				}
			}
		}
		if (usedSLs == 0) break;
	}
	return FSUCCESS;
}


// validate all the routes between all LIDs, exclude loopback routes
FSTATUS ValidateAllRoutes(FabricData_t *fabricp, EUI64 portGuid, uint8 rc,
			   				uint32 *totalPaths, uint32 *badPaths,
			   				ValidateCallback_t callback, void *context,
			   				ValidateCallback2_t callback2, void *context2,
							uint8 useSCSC)
{
	cl_map_item_t *n1, *n2;
	cl_map_item_t *p1, *p2;
	FSTATUS status;
	uint32 pathCount, badPathCount;
	uint32 usedSLs = 0;

	*totalPaths = 0;
	*badPaths = 0;

	if (useSCSC) {
		// get the SL information, SCs not needed
		if ((status = getSLSCInfo(fabricp, portGuid, 1, &usedSLs, NULL)) != FSUCCESS) {
			return status;
		}
	}

	for (n1=cl_qmap_head(&fabricp->AllNodes); n1 != cl_qmap_end(&fabricp->AllNodes); n1 = cl_qmap_next(n1)) {
		NodeData *nodep1 = PARENT_STRUCT(n1, NodeData, AllNodesEntry);
		for (p1=cl_qmap_head(&nodep1->Ports); p1 != cl_qmap_end(&nodep1->Ports); p1 = cl_qmap_next(p1)) {
			PortData *portp1 = PARENT_STRUCT(p1, PortData, NodePortsEntry);
			if (nodep1->NodeInfo.NodeType == STL_NODE_SW && portp1->PortNum != 0) {
				// only port 0 of a switch has a LID
				continue;
			}

			for (n2=cl_qmap_head(&fabricp->AllNodes); n2 != cl_qmap_end(&fabricp->AllNodes); n2 = cl_qmap_next(n2)) {
				NodeData *nodep2 = PARENT_STRUCT(n2, NodeData, AllNodesEntry);

				// enable this if we want to skip node to self paths
				//if (nodep1 == nodep2) continue;

				for (p2=cl_qmap_head(&nodep2->Ports); p2 != cl_qmap_end(&nodep2->Ports); p2 = cl_qmap_next(p2)) {
					PortData *portp2 = PARENT_STRUCT(p2, PortData, NodePortsEntry);
					if (nodep2->NodeInfo.NodeType == STL_NODE_SW && portp2->PortNum != 0) {
						// only port 0 of a switch has a LID
						continue;
					}
					// skip loopback paths
					if (portp1 == portp2) continue;
					status = ValidateRoutes(fabricp, portp1, portp2, &pathCount, &badPathCount, usedSLs, rc, callback, context, callback2, context2);
					if (status == FUNAVAILABLE)
						return status;
					(*totalPaths) += pathCount;
					(*badPaths) += badPathCount;
				}
			}
		}
	}
	return FSUCCESS;
}


//////////////////////////////////////////////////////
// multicast routes functions                       //
//////////////////////////////////////////////////////

static STL_PORTMASK *LookupMFT(PortData *portp, STL_LID mlid, uint8 pos)
{
	STL_PORTMASK *PortMask;
	uint32 entry;
	uint32 MCFDBSize;

	DEBUG_ASSERT(portp->nodep->switchp && portp->nodep->switchp->MulticastFDB[0]);	//caller checks

	// Get index independent of mlid format (16-bit or 32-bit)
	entry = GetMulticastOffset((uint32)mlid);
	MCFDBSize = portp->nodep->switchp->MulticastFDBSize & MULTICAST_LID_OFFSET_MASK;
	if (entry >=MCFDBSize)
		return NULL;

	if (pos >= STL_NUM_MFT_POSITIONS_MASK)
		return NULL;

	PortMask= GetMulticastFDBEntry(portp->nodep, entry, pos);

	return PortMask;
}

// copying the route with problems to main list for later display.
static FSTATUS CopyAndInsertMcLoopInc(FabricData_t *fabricp, MCROUTESTATUS MCRouteStatus, McLoopInc *pMcLoopInc)
{
	LIST_ITEM *p;

	McLoopInc *pMcLoopIncR = (McLoopInc*)MemoryAllocate2AndClear(sizeof(McLoopInc), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! pMcLoopIncR) {
		fprintf(stderr, "Unable to allocate memory to init a list of MC loop and incomplete routes\n");
		return FINSUFFICIENT_MEMORY;
	}
	//copying contents of pMcLoopInc to the new record.
	pMcLoopIncR->status = MCRouteStatus;
	pMcLoopIncR->mlid = pMcLoopInc->mlid;


	QListInitState(&pMcLoopIncR->AllMcNodeLoopIncR);
	if (!QListInit(&pMcLoopIncR->AllMcNodeLoopIncR)) {
		fprintf(stderr, "Unable to initialize List of nodes with not found MC routes\n");
		//deallocate mem
		MemoryDeallocate(pMcLoopIncR);
		return FINSUFFICIENT_MEMORY;
	}

	// copying list of nodes
	for (p = QListHead(&pMcLoopInc->AllMcNodeLoopIncR); p!= NULL; p = QListNext( &pMcLoopInc->AllMcNodeLoopIncR,p) ){
		// retrieve current node
		McNodeLoopInc *pmcnode = (McNodeLoopInc *) QListObj(p);
		// create new node
		McNodeLoopInc *pMcNodeLoopIncR = (McNodeLoopInc*)MemoryAllocate2AndClear(sizeof(McNodeLoopInc), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (! pMcNodeLoopIncR) {
			fprintf(stderr, "Unable to allocate memory to init a list of MC loop and incomplete routes\n");
			MemoryDeallocate(pMcLoopIncR);
			return FINSUFFICIENT_MEMORY;
		}
		// copy node to node
		pMcNodeLoopIncR->pPort = pmcnode->pPort;
		pMcNodeLoopIncR->entryPort = pmcnode->entryPort;
		pMcNodeLoopIncR->exitPort = pmcnode->exitPort;
		//insert new node
		QListSetObj(&pMcNodeLoopIncR->McNodeEntry, pMcNodeLoopIncR);
		QListInsertTail(&pMcLoopIncR->AllMcNodeLoopIncR , &pMcNodeLoopIncR->McNodeEntry);
	} // end for


	//When all nodes were copied add route item to the main list
	QListSetObj(&pMcLoopIncR->LoopIncEntry, pMcLoopIncR);
	QListInsertTail(&fabricp->AllMcLoopIncRoutes[MCRouteStatus].AllMcRouteStatus, &pMcLoopIncR->LoopIncEntry);

	return FSUCCESS;
}

static FSTATUS IsMemberMcGroup(McGroupData *mcgroupp, PortData *portp)
{
	LIST_ITEM *p;

	for (p=QListHead(&mcgroupp->AllMcGroupMembers); p!= NULL; p = QListNext(&mcgroupp->AllMcGroupMembers,p)) {
		McMemberData *mcmp = (McMemberData *)QListObj(p);
		if (mcmp->MemberInfo.RID.PortGID.AsReg64s.L == portp->nodep->NodeInfo.NodeGUID) {
			/*if (!mcmp->MemberInfo.JoinFullMember)
				return FNOT_FOUND;
			else */return FSUCCESS;
		}
	}
	return FNOT_FOUND;
}

// Walk MC route from portp for mlid group.
//
// returns status of MC route (if not FSUCCESS)
// FUNAVAILABLE - no routing tables in FabricData given
// FNOT_FOUND - unable to find starting port
// FNOT_DONE - unable to trace route, mlid is a dead end
// FDUPLICATE - loop detected in mc routes

FSTATUS WalkMCRoute(FabricData_t *fabricp, McGroupData *mcgroupp, PortData *portp, int hop,
	uint8 EntryPort, McLoopInc *pMcLoopInc, uint32 *pathCount)
{
	PortData *portp2, *portn;	// next port in route
	STL_PORTMASK *pp;
	FSTATUS status=FSUCCESS;
	uint64 Port_res;
	SwitchData *switchp;
	MCROUTESTATUS mcroutestat;
	McNodeLoopInc McNodeLoopIncR, *pMcNodeLoopIncR;

	pMcNodeLoopIncR = &McNodeLoopIncR;

	ListItemInitState(&pMcNodeLoopIncR->McNodeEntry);
	pMcNodeLoopIncR->pPort = portp;
	pMcNodeLoopIncR->entryPort = EntryPort;
	pMcNodeLoopIncR->exitPort = 0;

	QListSetObj(&pMcNodeLoopIncR->McNodeEntry, pMcNodeLoopIncR);
	QListInsertTail(&pMcLoopInc->AllMcNodeLoopIncR , &pMcNodeLoopIncR->McNodeEntry);

	if ((hop) >= 64) {
		// add list to NOT_DONE
		mcroutestat=MC_NO_TRACE;
		status = CopyAndInsertMcLoopInc(fabricp, mcroutestat, pMcLoopInc);
		if (status== FINSUFFICIENT_MEMORY) {
			fprintf(stderr, "Unable to allocate memory\n");
			return FERROR;
		}
		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
		(*pathCount)++;
		return FNOT_DONE; // too long a path
	}
// if port is FI them check membership, if OK then reach end-of-route
// if port is SW; Port0 then check membership, if OK then reach end-of-route
	if (( (hop > 1) && (portp->nodep->NodeInfo.NodeType == STL_NODE_SW)
		&& (EntryPort == 0) && portp->nodep->pSwitchInfo->SwitchInfoData.u2.s.EnhancedPort0) // this is a enhanced Port0;
		|| (portp->nodep->NodeInfo.NodeType == STL_NODE_FI)) { // these are end-nodes
		status = IsMemberMcGroup(mcgroupp, portp);
		(*pathCount)++;
		if (status != FSUCCESS) {
			mcroutestat=MC_NOGROUP;
			status = CopyAndInsertMcLoopInc(fabricp, mcroutestat, pMcLoopInc);
			if (status == FINSUFFICIENT_MEMORY) {
				fprintf(stderr, "Unable to allocate memory\n");
				return FERROR;
			}
			QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
			return FUNAVAILABLE; // not a member
		}
		else {
			QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
		}
		return FSUCCESS;
	}
	if ((EntryPort == 0) && (hop > 1)){ // Port 0 cannot be switch external port
		mcroutestat=MC_NOGROUP;
		status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
		if (status== FINSUFFICIENT_MEMORY) {
			fprintf(stderr, "Unable to allocate memory\n");
			return FERROR;
		}
		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
		return FNOT_DONE;
	}
// if we are here, port can be a switch
 	switchp = portp->nodep->switchp;

// 	//test if there is a switch
	if (!switchp) {
 		mcroutestat=MC_NO_TRACE;
 		status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
 		if (status== FINSUFFICIENT_MEMORY) {
 			fprintf(stderr, "Unable to allocate memory\n");
 			return FERROR;
 		}
		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
		(*pathCount)++;
		return FNOT_DONE;
	}

//test if there is a routing table
	if (!switchp->MulticastFDB) {
		mcroutestat=MC_UNAVAILABLE;
		status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
		if (status== FINSUFFICIENT_MEMORY) {
			fprintf(stderr, "Unable to allocate memory\n");
			return FERROR;
		}
		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
		(*pathCount)++;
		return FUNAVAILABLE;
	}

// check if the size of the table is enough
// check if MulticastFDBTop <= MulticastFDBCap
 	uint32 MCTableSize;
// lower 14 bits of top
	MCTableSize = portp->nodep->pSwitchInfo->SwitchInfoData.MulticastFDBTop & MULTICAST_LID_OFFSET_MASK;
	if (MCTableSize > portp->nodep->pSwitchInfo->SwitchInfoData.MulticastFDBCap){
		mcroutestat=MC_UNAVAILABLE;
 		status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
 		if (status== FINSUFFICIENT_MEMORY) {
 			fprintf(stderr, "Unable to allocate memory\n");
 			return FERROR;
 		}
 		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
 		(*pathCount)++;
		return FUNAVAILABLE;
	}

// get index to retrieve routing ports for MC from the MC table
	int ix_lid = GetMulticastOffset((uint32)pMcLoopInc->mlid);
	//verify that index lies within the table size
 	if ( ix_lid > MCTableSize) {
 		mcroutestat=MC_UNAVAILABLE;
 		status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
 		if (status== FINSUFFICIENT_MEMORY) {
 			fprintf(stderr, "Unable to allocate memory\n");
 			return FERROR;
 		}
 		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
 		(*pathCount)++;
 		return FUNAVAILABLE;
 	}


	// is this my first visit here?
	if (!portp->nodep->switchp->HasBeenVisited)
		portp->nodep->switchp->HasBeenVisited = 1;
	else {   //if not... there is a loop.
		mcroutestat=MC_LOOP;
		status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
		if (status== FINSUFFICIENT_MEMORY) {
			fprintf(stderr, "Unable to allocate memory\n");
			return FERROR;
		}
		QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
		(*pathCount)++;
		return FDUPLICATE;
	}

	// getting information of number of ports for the current switch to run the loop only for those values
	// limiting max. num of ports to 256
	uint8 swmaxnumport = portp->nodep->NodeInfo.NumPorts;
	if (swmaxnumport > STL_MAX_PORTS) {
		fprintf(stderr, "Cannot handle more than %d different ports\n", STL_MAX_PORTS);
		return FERROR;
	}

	uint8 pos=0;
	int ix_port;
	for (ix_port=0; ix_port <= swmaxnumport; ix_port ++) {
		if (EntryPort != ix_port) {
			//select correct mask to test given 256 max ports
			pos = ix_port / STL_PORT_MASK_WIDTH;
			pp = LookupMFT(portp, (uint32)pMcLoopInc->mlid, pos);
			if (pp==NULL) {
				mcroutestat=MC_UNAVAILABLE;
				status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
				if (status== FINSUFFICIENT_MEMORY) {
					fprintf(stderr, "Unable to allocate memory\n");
					return FERROR;
				}
				QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);
				(*pathCount)++;
				return FUNAVAILABLE;
			}
			Port_res = ((uint64_t)1<< (ix_port % STL_PORT_MASK_WIDTH)) & *pp;/// this must be compatible with 4 columns of 64 bits each
			if (Port_res != 0) { //matching port
				pMcNodeLoopIncR->exitPort = ix_port; // save exit port
				if (ix_port == 0) {
					status = WalkMCRoute( fabricp, mcgroupp, portp, (hop+1), 0, pMcLoopInc, pathCount);
					if (status == FERROR)
						return FERROR;
				}
				else {
					portp2 = FindNodePort(portp->nodep, ix_port);
					if (! portp2 || ! IsPortInitialized(portp2->PortInfo.PortStates)
						|| (!portp2->neighbor)) {
						mcroutestat=MC_NO_TRACE;
						status = CopyAndInsertMcLoopInc(fabricp,mcroutestat, pMcLoopInc);
						if (status== FINSUFFICIENT_MEMORY) {
							fprintf(stderr, "Unable to allocate memory\n");
							return FERROR;
						}
						(*pathCount)++;
						status = FNOT_DONE;
					} // Non viable route
					else {
						portn = portp2->neighbor; // mist be the entry port of the next switch
						status = WalkMCRoute( fabricp, mcgroupp, portn, (hop+1), portn->PortNum, pMcLoopInc, pathCount);
						if (status == FERROR)
						// just return all the way
							return FERROR;
					} //end else
				} //end else
			} // end of there is a match
		}// end of entry port != exit port
	} // end for looking for next port
	QListRemoveTail(&pMcLoopInc->AllMcNodeLoopIncR);

	return status;
}

// report all MC routes
FSTATUS ValidateMCRoutes(FabricData_t *fabricp, McGroupData *mcgroupp,
			McEdgeSwitchData *swp, uint32 *pathCount)
{
	FSTATUS status;

	SwitchData *switchp;
	PortData *portp1;
	LIST_ITEM *n1;
	portp1 = swp->pPort;

	switchp = portp1->nodep->switchp;
	if (!switchp) {
		printf("No switch connected to HFI \n");
		return FNOT_DONE;
	}

	McLoopInc mcLoop;
	QListInitState(&mcLoop.AllMcNodeLoopIncR);
	if (!QListInit(&mcLoop.AllMcNodeLoopIncR)) {
		fprintf(stderr, "Unable to initialize List of nodes for loops and incomplete MC routes\n");
		return FERROR;
	}

	mcLoop.mlid = mcgroupp->MLID; // identifies the route to analyze

	/* clear visited flag set in previous call to WalkMCRoute for every node */
	for (n1 = QListHead(&fabricp->AllSWs ); n1 != NULL; n1= QListNext(&fabricp->AllSWs, n1)) {
		NodeData * node = (NodeData*)QListObj(n1);
		node->switchp->HasBeenVisited=0;
	}
	status = WalkMCRoute( fabricp, mcgroupp, portp1, 1, swp->EntryPort, &mcLoop, pathCount);

	return status;
}

FSTATUS InitListofLoopAndIncMCRoutes(FabricData_t *fabricp)

{	int i;

	// init list of loops and incomplete MC routes

	fabricp->AllMcLoopIncRoutes[0].status=MC_NO_TRACE;
	fabricp->AllMcLoopIncRoutes[1].status=MC_NOT_FOUND;
	fabricp->AllMcLoopIncRoutes[2].status=MC_UNAVAILABLE;
	fabricp->AllMcLoopIncRoutes[3].status=MC_LOOP;
	fabricp->AllMcLoopIncRoutes[4].status=MC_NOGROUP;


	for (i=0; i<MAXMCROUTESTATUS; i++) {
		QListInitState(&fabricp->AllMcLoopIncRoutes[i].AllMcRouteStatus);
		if (!QListInit(&fabricp->AllMcLoopIncRoutes[i].AllMcRouteStatus)) {
			fprintf(stderr, "Unable to initialize List of MC loops and incomplete routes\n");
			return FERROR;
		}
	}
	return FSUCCESS;
}


FSTATUS ValidateAllMCRoutes(FabricData_t *fabricp, uint32 *totalPaths )

{	LIST_ITEM *n1, *p1, *q1;
	FSTATUS status;
	McMemberData *pMCM1;
	uint32 pathCount;

	*totalPaths = 0;

	status = InitListofLoopAndIncMCRoutes(fabricp);
	if (status!=FSUCCESS)
		return FERROR;

	// init all switches as never been visited
	for (n1 = QListHead(&fabricp->AllSWs ); n1 != NULL; n1= QListNext(&fabricp->AllSWs, n1)) {
		NodeData * node = (NodeData*)QListObj(n1);
		node->switchp->HasBeenVisited=0;
	}

	for (n1 = QListHead(&fabricp->AllMcGroups); n1 != NULL; n1= QListNext(&fabricp->AllMcGroups, n1)) {
		McGroupData *pmcgmember = (McGroupData *)QListObj(n1);
		//for this group get all member information
			p1 = QListHead(&pmcgmember->AllMcGroupMembers);
			pMCM1 = (McMemberData *)QListObj(p1);
			// do not validate routes empty groups or groups with 1 member
			if ((pMCM1->MemberInfo.RID.PortGID.AsReg64s.H != 0) || (pMCM1->MemberInfo.RID.PortGID.AsReg64s.L!=0)) {
				if (pmcgmember->NumOfMembers > 1) {
					for (q1 = QListHead(&pmcgmember->EdgeSwitchesInGroup); q1 != NULL; q1= QListNext(&pmcgmember->EdgeSwitchesInGroup, q1)) {
						pathCount=0;
						McEdgeSwitchData *swp = (McEdgeSwitchData *)QListObj(q1);
						status = ValidateMCRoutes(fabricp, pmcgmember, swp, &pathCount);
						if (status ==FERROR) {
							fprintf(stderr, "Unable to validate MC routes\n");
							return FERROR;
						}
						(*totalPaths) += pathCount;
					}
				} // end if num members >1
//			}// end evaluating groups with non-zero members*/
		} // end evaluating one MC group
	} // end for all MC groups

	return FSUCCESS;
} // End of ValidateAllMCRoutes

void FreeValidateMCRoutes(FabricData_t *fabricp)
{ int i;
  LIST_ITEM *p, *q;

	for (i=0;i<MAXMCROUTESTATUS;i++){
		//if the list is empty there is nothing to release
		while (!QListIsEmpty(&fabricp->AllMcLoopIncRoutes[i].AllMcRouteStatus) ){
			p = QListTail(&fabricp->AllMcLoopIncRoutes[i].AllMcRouteStatus);
			McLoopInc *pmcloop = (McLoopInc *) QListObj(p);
				//remove members in each mc route
			while (!QListIsEmpty(&pmcloop->AllMcNodeLoopIncR)) {
				q = QListTail(&pmcloop->AllMcNodeLoopIncR);
				McNodeLoopInc *pmcnodeloop = (McNodeLoopInc *) QListObj(q);
				QListRemoveTail(&pmcloop->AllMcNodeLoopIncR);
				MemoryDeallocate (pmcnodeloop);
			} // end while q
			QListRemoveTail(&fabricp->AllMcLoopIncRoutes[i].AllMcRouteStatus);
			MemoryDeallocate (pmcloop);
		}// end  while p
	}// end for 0..4
	return;

}

////////// /////////// end of MC-related functions



#ifndef __VXWORKS__

static FSTATUS CLGetRoute(FabricData_t *fabricp, EUI64 portGuid, uint8 rc,
                          PortData *portp1, PortData *portp2, 
                          IB_PATH_RECORD *pathp, uint32 *totalPaths, uint32 *totalBadPaths, 
                          ValidateCLRouteCallback_t callback, void *context, uint8 snapshotInFile,
                          uint32 usedSCs)
{
   FSTATUS status; 
   STL_TRACE_RECORD	*pTraceRecords = NULL; 
   uint32 NumTraceRecords = 0; 
   int i = -1, detail = 0, quiet = 0; 
   uint32 links = 0; 
   PortData *p = portp1,*fromPortp = portp1; 
   int p_shown = 0; 
   clConnPathData_t pathInfo;
   PQUERY_RESULT_VALUES pQueryResults = NULL; 
   ValidateCreditLoopRoutesContext_t *cp = (ValidateCreditLoopRoutesContext_t *)context; 
   
   
   if (cp) detail = cp->detail; 
   if (cp) quiet = (cp->quiet == 1) ? 1 : 0;
   
   // add source and destination HFIs endnodes to device list 
   if (portp1->nodep->NodeInfo.NodeType != STL_NODE_FI || portp2->nodep->NodeInfo.NodeType != STL_NODE_FI) {
      status = FINVALID_PARAMETER; 
      goto done; 
   } else if (!CLDataAddDevice(fabricp, portp1->nodep, pathp->SLID, detail, quiet) || 
              !CLDataAddDevice(fabricp, portp2->nodep, pathp->DLID, detail, quiet)) {
      status = FINSUFFICIENT_MEMORY; 
      goto done; 
   }
   
   if (portp1 == portp2) {
      /* special case, internal loopback */
      status = FSUCCESS; 
      goto done;
   }
   if (portp1->neighbor == portp2) {
      /* special case, single link traversed */
      // Since portp1 has a neighbor, neither port is SW Port 0
      status = FSUCCESS; 
      goto done;
   }

   // get trace route for path
   status = GenTraceRoutePath(fabricp, pathp, rc, &pTraceRecords, &NumTraceRecords);
   
   if (NumTraceRecords <= 0)
      goto badroute;
   //ASSERT(NumTraceRecords > 0); 
   
   /* the first Trace record should be the exit from portp1, however
    * not all versions of the SM report this record
    */
   if (pTraceRecords[0].NodeType != portp1->nodep->NodeInfo.NodeType) {
      /* workaround SM bug, did not report initial exit port */
      // assume portp1 is not a Switch Port 0
      p = portp1->neighbor; 
      if (!p) {
         goto badroute;
      }
   }

   // replicate path record for later use
   MemoryCopy(&pathInfo.path, pathp, sizeof(IB_PATH_RECORD));

   for (i = 0; i < NumTraceRecords; i++) {
      // add switches to device list 
      if (!CLDataAddDevice(fabricp, p->nodep, 0, detail, quiet)) {
         status = FINSUFFICIENT_MEMORY; 
         goto done; 
      }
      if (p != portp1) {
         // add up-link and down-link connections to connection list

         if (usedSCs && fromPortp->pQOS) {
            uint8 sc;
            for (sc = 0; (usedSCs >> sc); sc++) {
               if ((usedSCs >> sc) & 1) {
                  if ((status = CLDataAddConnection(fabricp, fromPortp, p, &pathInfo, sc, detail, quiet)) ||
                      (status = CLDataAddConnection(fabricp, p, fromPortp, &pathInfo, sc, detail, quiet))) {
                     if (status == FINSUFFICIENT_MEMORY) goto done;
                     goto badroute;
                  }
                  links++;
                  p_shown = 1;
               }
            }
            if ((status = CLDataAddRoute(fabricp, pathp->SLID, pathp->DLID, fromPortp, detail, quiet))) {
               if (status == FINSUFFICIENT_MEMORY) goto done;
               goto badroute;
            }
         } else {
            if ((status = CLDataAddConnection(fabricp, fromPortp, p, &pathInfo, 0, detail, quiet)) ||
                (status = CLDataAddConnection(fabricp, p, fromPortp, &pathInfo, 0, detail, quiet)) ||
                (status = CLDataAddRoute(fabricp, pathp->SLID, pathp->DLID, fromPortp, detail, quiet))) {
               if (status == FINSUFFICIENT_MEMORY) goto done;
               goto badroute;
            }
            links++;
            p_shown = 1;
         }
      }
      if (pTraceRecords[i].NodeType != STL_NODE_FI) {
         p = FindNodePort(p->nodep, pTraceRecords[i].ExitPort); 
         if (!p) {
            goto badroute;
         }
         if (0 == p->PortNum) {
            /* Switch Port 0 thus must be final port */
            if (i + 1 != NumTraceRecords) {
               goto badroute;
            }
            break;
         }
         
         fromPortp = p; 
         if (p == portp2) {
            // this should not happen.  If we reach portp2 as the exit
            // port of a switch, that implies portp2 must be port 0 of
            // the switch which the test above should have caught
            // but it doesn't hurt to have this redundant test here to be
            // safe.
            /* final port must be Switch Port 0 */
            if (i + 1 != NumTraceRecords) {
               goto badroute;
            }
         } else {
            p = p->neighbor; 
            if (!p) {
               goto badroute;
            }
            p_shown = 0;
         }
      } else if (i == 0) {
         /* since we caught FI to FI case above, SM must have given us
          * initial Node in path
          */
         /* unfortunately spec says Exit and Entry Port are 0 for CA, so
          * can't verify consistency with portp1
          */
         p = portp1->neighbor; 
         if (!p) {
            goto badroute;
         }
         p_shown = 0;
      } else if (i + 1 != NumTraceRecords) {
         goto badroute;
      }
   }
   if (!p_shown) {
      /* workaround SM bug, did not report final hop in route */
      // add up-link and down-link connections to connection list 
      if ((status = CLDataAddConnection(fabricp, fromPortp, portp2, &pathInfo, 0, detail, quiet)) ||
          (status = CLDataAddConnection(fabricp, portp2, fromPortp, &pathInfo, 0, detail, quiet)) ||
          (status = CLDataAddRoute(fabricp, pathp->SLID, pathp->DLID, fromPortp, detail, quiet))) {
         if (status == FINSUFFICIENT_MEMORY) goto done;
         goto badroute;
	 *totalPaths += 1;
      }
   }
   if (p != portp2) {
      goto badroute;
   }

   *totalPaths += 1;

done:
   if (pQueryResults)
      omgt_free_query_result_buffer(pQueryResults); 
   if (pTraceRecords)
      MemoryDeallocate(pTraceRecords); 
   return status; 
   
badroute:
   *totalBadPaths += 1; 
   status = FSUCCESS;  // might as well process what we can
   (void)callback(portp1, portp2, context);

   goto done;
}

static FSTATUS CLGetRoutes(FabricData_t *fabricp, EUI64 portGuid, uint8 rc,  
                           PortData *portp1, PortData *portp2, 
                           uint32 *totalPaths, uint32 *totalBadPaths, 
                           ValidateCLRouteCallback_t callback, void *context,
                           uint8 snapshotInFile, uint32 usedSCs)
{ 
   FSTATUS status; 
   int i; 
   uint32 NumPathRecords; 
   IB_PATH_RECORD *pPathRecords = NULL; 
   

   status = GenPaths(fabricp, portp1, portp2, &pPathRecords, &NumPathRecords);

   for (i = 0; i < NumPathRecords; i++) {
      CLGetRoute(fabricp, portGuid, rc, portp1, portp2, &pPathRecords[i], 
                 totalPaths, totalBadPaths, callback, context, snapshotInFile,
                 usedSCs);
   }
   
   if (pPathRecords)
      MemoryDeallocate(pPathRecords);
   
   return status;
}

FSTATUS ValidateAllCreditLoopRoutes(FabricData_t *fabricp, EUI64 portGuid, uint8 rc,  
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
                                    uint8 useSCSC)
{ 
   FSTATUS status; 
   int detail = 0, xmlFmt; 
   uint32 nodeCount = 1, totalPaths = 0, totalBadPaths = 0; 
   LIST_ITEM * n1,*n2; 
   cl_map_item_t * p1,*p2; 
   uint64_t sTime = 0, eTime = 0; 
   uint64_t sTotalTime = 0, eTotalTime = 0; 
   ValidateCreditLoopRoutesContext_t *cp = (ValidateCreditLoopRoutesContext_t *)context; 
   uint32 usedSCs = 0, usedSLs = 0;
   
   if (!cp) 
      return FNOT_DONE; 
   
   detail = cp->detail; 
   xmlFmt = (cp->format == 1) ? 1 : 0; 
   
   if (!xmlFmt && cp->detail >= 3) {
      timeGetCallback(&sTotalTime, &g_cl_lock); 
      timeGetCallback(&sTime, &g_cl_lock); 
      printf("START build all the routes\n"); 
   }
   
   // find the used SCs
   if (useSCSC) {
      if ((status = getSLSCInfo(fabricp, portGuid, cp->quiet, &usedSLs, &usedSCs)) != FSUCCESS) {
         return status;
      }
   }

   //
   // collect routing information between all endnodes within the fabric 
   for (n1 = QListHead(&fabricp->AllFIs); n1 != NULL; n1 = QListNext(&fabricp->AllFIs, n1)) {
      NodeData *nodep1 = (NodeData *)QListObj(n1); 
      if (!xmlFmt && cp->detail >= 3) {
         printf("START build all routes from %s\n", nodep1->NodeDesc.NodeString);
      }

      for (p1 = cl_qmap_head(&nodep1->Ports); p1 != cl_qmap_end(&nodep1->Ports); p1 = cl_qmap_next(p1), nodeCount++) {
         PortData *portp1 = PARENT_STRUCT(p1, PortData, NodePortsEntry); 

	 if (nodep1->NodeInfo.NodeType == STL_NODE_SW && portp1->PortNum != 0) {
            continue;
         }
         
         for (n2 = QListHead(&fabricp->AllFIs); n2 != NULL; n2 = QListNext(&fabricp->AllFIs, n2)) {
            NodeData *nodep2 = (NodeData *)QListObj(n2); 
            
            // enable this if we want to skip node to self paths
            //if (nodep1 == nodep2) continue;
            
            for (p2 = cl_qmap_head(&nodep2->Ports); p2 != cl_qmap_end(&nodep2->Ports); p2 = cl_qmap_next(p2)) {
               PortData *portp2 = PARENT_STRUCT(p2, PortData, NodePortsEntry); 
               if (nodep2->NodeInfo.NodeType == STL_NODE_SW && portp2->PortNum != 0) {
                  continue;
               }
               // skip loopback paths
               if (portp1 == portp2) 
                  continue; 
               
               if (FUNAVAILABLE == (status = CLGetRoutes(fabricp, portGuid, rc, portp1, portp2, 
                                                         &totalPaths, &totalBadPaths, 
                                                         routeCallback, context, snapshotInFile,
                                                         usedSCs)))
                  return status;
            }
         }
      }
      if (!xmlFmt && cp->detail >= 3) {
         printf("END build all routes from %s\n", nodep1->NodeDesc.NodeString); 
         printf("%d of %d HFI nodes completed\n", ++nodeCount, QListCount(&fabricp->AllFIs));
      } else {
         if (nodeCount % PROGRESS_FREQ == 0 || nodeCount == 1) 
            if (!cp->quiet) 
               ProgressPrint(FALSE, "Processed %6d of %6d Nodes...", nodeCount, QListCount(&fabricp->AllFIs));
      }
   }
   
   if (!xmlFmt && cp->detail >= 3) {
      timeGetCallback(&eTime, &g_cl_lock); 
      printf("END build all the routes; elapsed time(usec)=%d, (sec)=%d\n", 
             (int)(eTime - sTime), ((int)(eTime - sTime)) / CL_TIME_DIVISOR); 
   }
   
   // clear line used to display progress report
   if (!cp->quiet)
      ProgressPrint(TRUE, "Done Building All Routes");
      
   //PYTHON: fabric.summary('Fabric')
   (void)CLFabricSummary(fabricp, "Fabric", fabricSummaryCallback, totalPaths, totalBadPaths, cp); 
   
   if (!cl_qmap_count(&fabricp->map_guid_to_ib_device) || !fabricp->ConnectionCount || !fabricp->RouteCount) 
      return FNOT_DONE; 
   
   /* build graphical layout of all the routes */
   //PYTHON: full_graph = build_routing_graph(fabric)
   if (!CLFabricDataBuildRouteGraph(fabricp, routeSummaryCallback, timeGetCallback, cp, usedSLs)) {
      clGraphData_t *graphp; 
      
      if (detail >= 3) {
         //PYTHON: full_graph.summary('Full graph')
         (void)CLGraphDataSummary(&fabricp->Graph, (xmlFmt) ? "FullGraph" : "Full graph", dataSummaryCallback, cp); 
      }      
      
      /* prune the graph data */
      //PYTHON: pruned_graph.prune()
      (void)CLGraphDataPrune(&fabricp->Graph, timeGetCallback, detail, cp->quiet); 
      
      if (detail >= 3) {
         //PYTHON: pruned_graph.summary('Pruned graph')
         (void)CLGraphDataSummary(&fabricp->Graph, (xmlFmt) ? "PrunedGraph" : "Pruned graph", dataSummaryCallback, cp);
      }
      
      /* split the graph data */
      //PYTHON: split_graph = pruned_graph.split()
      if (!(graphp = CLGraphDataSplit(&fabricp->Graph, detail))) {
         if (!xmlFmt) 
            printf("Routes are deadlock free (No credit loops detected)\n");
      } else {
         int count = 0; 
         char title[100]; 
         clDijkstraDistancesAndRoutes_t dijkstraInfo; 
         
         if (!xmlFmt) 
            printf("Deadlock detected in routes (Credit loops detected)\n"); 
         memset(&dijkstraInfo, 0, sizeof(dijkstraInfo)); 
         
         //PYTHON: while split_graph :
         while (graphp) {
            if (detail >= 3) {
               if (xmlFmt) 
                  sprintf(title, "SplitGraph%d", count); 
               else 
                  sprintf(title, "Split graph %d", count); 
               //PYTHON: split_graph.summary('Split graph %d' % count)
               (void)CLGraphDataSummary(graphp, title, dataSummaryCallback, cp); 
            }
            
            /* find route distances via Dijkstra algorithm */
            //PYTHON: (distances, routes) = find_distances_and_routes_dijkstra(split_graph)
            if (CLDijkstraFindDistancesAndRoutes(graphp, &dijkstraInfo, detail)) 
               break; 
            else {
               //PYTHON: find_cycles(split_graph, distances, routes, ib_connection_source_to_str)
               (void)CLDijkstraFindCycles(fabricp, graphp, &dijkstraInfo, 
                                          linkSummaryCallback, linkStepSummaryCallback, 
                                          pathSummaryCallback, cp); 
               /* free existing distances and routes relaed data */
               (void)CLDijkstraFreeDistancesAndRoutes(&dijkstraInfo); 
               //PYTHON: split_graph = pruned_graph.split()
               CLGraphDataFree(graphp, cp);
               MemoryDeallocate(graphp); // graphp was allocated by CLGraphDataSplit
               graphp = CLGraphDataSplit(&fabricp->Graph, detail);
            }
            count += 1;
         }
         
         if (detail >= 3 && count > 1) 
            printf("Dependencies split into %d disconnected graphs\n", count);
      }
      
      // free all credit loop related data
      if (graphp) MemoryDeallocate(graphp);
      if (CLFabricDataDestroy(fabricp, cp)) 
         fprintf(stderr, "Warning, failed to deallocate route credit loop data\n");
   }
   
   if (!xmlFmt && cp->detail >= 3) {
      timeGetCallback(&eTotalTime, &g_cl_lock); 
      printf("END Credit loop validation; elapsed time(usec)=%12"PRIu64", (sec)=%12"PRIu64"\n", 
             (eTotalTime - sTotalTime), ((eTotalTime - sTotalTime)) / CL_TIME_DIVISOR); 
   }
   
   return FSUCCESS;
}

#endif
