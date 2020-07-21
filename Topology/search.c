/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

#ifndef __VXWORKS__
#include <fnmatch.h>
#endif

// Functions to search the topology


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
PortData * FindNodePort(NodeData *nodep, uint8 port)
{
	cl_map_item_t *mi;

	mi = cl_qmap_get(&nodep->Ports, port);
	if (mi == cl_qmap_end(&nodep->Ports))
		return NULL;
	return PARENT_STRUCT(mi, PortData, NodePortsEntry);
}

// a cl_pfm_qmap_item_compare_t function
// compare a LID (in key) against the range of Lids assigned to the
// given PortData item
// each PortData has LIDs from LID to LID|((1<<LMC)-1)
// Hence when LMC=0 (eg. 1<<0 -> 1), PortData has exactly 1 LID
// When LMC=1, PortData has 2 LIDs [LID to LID+1]
// When LMC=2, PortData has 4 LIDs [LID to LID+3]
// etc.
// Since non-zero LMC requires low "LMC" bits of LID to be 0 we use |
// however so we could also use an + instead of a |
static int CompareLid(const cl_map_item_t *mi, const uint64 key)
{
	PortData *portp = PARENT_STRUCT(mi, PortData, AllLidsEntry);

	if ((portp->PortInfo.LID |
					((1<<portp->PortInfo.s1.LMC)-1)) < key)
		return -1;
	else if (portp->PortInfo.LID > key)
		return 1;
	else
		return 0;
}

// search for the PortData corresponding to the given lid
// accounts for LMC giving a port a range of lids
PortData * FindLid(FabricData_t *fabricp, STL_LID lid)
{
	if (fabricp->flags & FF_LIDARRAY) {
		return GetMapEntry(fabricp, lid);
	} else {
		cl_map_item_t *mi;

		mi = cl_qmap_get_item_compare(&fabricp->u.AllLids, lid, CompareLid);
		if (mi == cl_qmap_end(&fabricp->u.AllLids))
			return NULL;
		return PARENT_STRUCT(mi, PortData, AllLidsEntry);
	}
}

// search for the PortData corresponding to the given port Guid
PortData * FindPortGuid(FabricData_t *fabricp, EUI64 guid)
{
	LIST_ITEM *p;

	for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
		PortData *portp = (PortData *)QListObj(p);

		if (portp->PortGUID == guid)
			return portp;
	}
	return NULL;
}

// search for the PortData, ExpectedNode, ExpectedSM and ExpectedLink
// corresponding to the given port guid
// and update the point with all those which match
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindPortGuidPoint(FabricData_t *fabricp, EUI64 guid, Point *pPoint, uint8 find_flag, int silent)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	ASSERT(guid);
	if (0 == find_flag)
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		pPoint->u.portp = FindPortGuid(fabricp, guid);
		if (pPoint->u.portp)
			pPoint->Type = POINT_TYPE_PORT;
	}
	if (find_flag & FIND_FLAG_ENODE) {
		EUI64 nodeGUID = PortGUIDtoNodeGUID(guid);
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		// while there should be no duplicates, be safe and assume there might
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->NodeGUID == nodeGUID)
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}
	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		// while there should be no duplicates, be safe and assume there might
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			if (esmp->PortGUID == guid)
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if ((elinkp->portselp1 && elinkp->portselp1->PortGUID == guid)
				|| (elinkp->portselp2 && elinkp->portselp2->PortGUID == guid))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		if(!silent)
			fprintf(stderr, "%s: Port GUID Not Found: 0x%016"PRIx64"\n", g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the PortData, ExpectedNode, ExpectedSM and ExpectedLink
// corresponding to the given GID
// and update the point with all those which match
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindGidPoint(FabricData_t *fabricp, IB_GID gid, Point *pPoint, uint8 find_flag, int silent)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	ASSERT(gid.AsReg64s.H && gid.AsReg64s.L);
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ENODE|FIND_FLAG_ESM|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		pPoint->u.portp = FindPortGuid(fabricp, gid.Type.Global.InterfaceID);
		if (pPoint->u.portp &&
			pPoint->u.portp->PortInfo.SubnetPrefix
				== gid.Type.Global.SubnetPrefix) {
			pPoint->Type = POINT_TYPE_PORT;
		}
	}
	if (find_flag & FIND_FLAG_ENODE) {
		EUI64 nodeGUID = PortGUIDtoNodeGUID(gid.Type.Global.InterfaceID);
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		// while there should be no duplicates, be safe and assume there might
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->NodeGUID == nodeGUID)
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}
	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		// while there should be no duplicates, be safe and assume there might
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			/* we should also check SubnetPrefix, but its not in ExpectedSM,
			 * so just check PortGuid
			 */
			if (esmp->PortGUID == gid.Type.Global.InterfaceID)
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			/* we should also check SubnetPrefix, but its not in ExpectedLink,
			 * so just check PortGuid
			 */
			if ((elinkp->portselp1 && elinkp->portselp1->PortGUID == gid.Type.Global.InterfaceID)
				|| (elinkp->portselp2 && elinkp->portselp2->PortGUID == gid.Type.Global.InterfaceID))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		if(!silent) {
			fprintf(stderr, "%s: GID Not Found: 0x%016"PRIx64":0x%016"PRIx64"\n",
					g_Top_cmdname, 
					gid.Type.Global.SubnetPrefix, gid.Type.Global.InterfaceID);
		}
		if ((find_flag & FIND_FLAG_FABRIC) && pPoint->u.portp
				&& pPoint->u.portp->PortInfo.SubnetPrefix
					!= gid.Type.Global.SubnetPrefix) {
			if (! silent) {
				fprintf(stderr, "%s: Subnet Prefix: 0x%016"PRIx64" does not match selected port: 0x%016"PRIx64"\n",
						g_Top_cmdname, gid.Type.Global.SubnetPrefix, 
						pPoint->u.portp->PortInfo.SubnetPrefix);
			}
			pPoint->u.portp = NULL;
			pPoint->Type = POINT_TYPE_NONE;
		}
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the NodeData corresponding to the given node Guid
NodeData * FindNodeGuid(const FabricData_t *fabricp, EUI64 guid)
{
	cl_map_item_t *mi;

	mi = cl_qmap_get(&fabricp->AllNodes, guid);
	if (mi == cl_qmap_end(&fabricp->AllNodes))
		return NULL;
	return PARENT_STRUCT(mi, NodeData, AllNodesEntry);
}

// search for the NodeData, ExpectedNode, ExpectedSM and ExpectedLink
// corresponding to the given node guid
// and update the point with all those which match
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodeGuidPoint(FabricData_t *fabricp, EUI64 guid, Point *pPoint, uint8 find_flag, int silent)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	ASSERT(guid);
	if (0 == find_flag)
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		pPoint->u.nodep = FindNodeGuid(fabricp, guid);
		if (pPoint->u.nodep)
			pPoint->Type = POINT_TYPE_NODE;
	}
	if (find_flag & FIND_FLAG_ENODE) {
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		// while there should be no duplicates, be safe and assume there might
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->NodeGUID == guid)
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}
	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		// if an SM runs on multiple ports of a device, could validly be
		// multiple matches of the same NodeGUID
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			if (esmp->NodeGUID == guid)
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if ((elinkp->portselp1 && elinkp->portselp1->NodeGUID == guid)
				|| (elinkp->portselp2 && elinkp->portselp2->NodeGUID == guid))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		if(!silent)
			fprintf(stderr, "%s: Node GUID Not Found: 0x%016"PRIx64"\n", g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the NodeData, ExpectedNode, ExpectedSM and ExpectedLink
// corresponding to the given node name
// and update the point with all those which match
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodeNamePoint(FabricData_t *fabricp, char *name, Point *pPoint, uint8 find_flag, int silent)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == find_flag)
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
			NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
			if (strncmp((char*)nodep->NodeDesc.NodeString,
						name, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_NODE_LIST, nodep);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ENODE) {
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->NodeDesc && strncmp(enodep->NodeDesc,
							name, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}
	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			if (esmp->NodeDesc && strncmp(esmp->NodeDesc,
						name, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if ((elinkp->portselp1 && elinkp->portselp1->NodeDesc
					&& strncmp(elinkp->portselp1->NodeDesc,
						name, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
				|| (elinkp->portselp2 && elinkp->portselp2->NodeDesc
					&& strncmp(elinkp->portselp2->NodeDesc,
						name, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		if(!silent)
			fprintf(stderr, "%s: Node name Not Found: %s\n", g_Top_cmdname, name);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

#ifndef __VXWORKS__

/* append searched objects that match the pattern */
FSTATUS PopoulateNodePatPairs(NodePairList_t *nodePatPairs, uint8 side, void *object)
{
	DLIST *pList;

	if (side == LSIDE_PAIR){
		pList = &nodePatPairs->nodePairList1;
	} else if(side == RSIDE_PAIR) {
		pList = &nodePatPairs->nodePairList2;
	} else {
		return FINVALID_OPERATION;
	}

	if (!ListInsertTail(pList, object)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

// search for the NodeData
// corresponding to the given node name pattern for the given Node Pair
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag or side contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodePatPairs(FabricData_t *fabricp, char *pattern, NodePairList_t *nodePatPairs,
								uint8 find_flag, uint8 side)
{
	FSTATUS status;

	if (0 == find_flag)
		return FINVALID_OPERATION;

	if (0 == side)
		return FINVALID_OPERATION;

	if (find_flag & FIND_FLAG_FABRIC){
		cl_map_item_t *p;
		/* the node can be of  type FI or SW */
		for (p = cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)){
			NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
			/* find all SWs and FIs that match the pattern */
			if (fnmatch(pattern, (char*)nodep->NodeDesc.NodeString, 0) == 0){
				status = PopoulateNodePatPairs(nodePatPairs, side, nodep);
				if (FSUCCESS != status)
					return status;
			}
		}
		return FSUCCESS;
	}
	return FNOT_FOUND;
}

// search for the NodeData, ExpectedNode and ExpectedSM
// corresponding to the given node name pattern
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodeNamePatPoint(FabricData_t *fabricp, char *pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(pPoint);
	status = FindNodeNamePatPointUncompress(fabricp, pattern, pPoint, find_flag);
	if (FSUCCESS == status)
		PointCompress(pPoint);

	return status;
}

// search for the NodeData, ExpectedNode and ExpectedSM
// corresponding to the given node name pattern
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodeNamePatPointUncompress(FabricData_t *fabricp, char *pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(pPoint);
	if (0 == find_flag)
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
			NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
			if (fnmatch(pattern, (char*)nodep->NodeDesc.NodeString, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_NODE_LIST, nodep);
				if (FSUCCESS != status)
					return status;
				//Set flag if the node is a switch or FI
				if (nodep->NodeInfo.NodeType == STL_NODE_SW)
					pPoint->haveSW = TRUE;
				else if (nodep->NodeInfo.NodeType == STL_NODE_FI)
					pPoint->haveFI = TRUE;
			}
		}
	}
	if (find_flag & FIND_FLAG_ENODE) {
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->NodeDesc
					&& 0 == fnmatch(pattern, enodep->NodeDesc, 0))
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}
	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			if (esmp->NodeDesc
				&& 0 == fnmatch(pattern, esmp->NodeDesc, 0))
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if ((elinkp->portselp1 && elinkp->portselp1->NodeDesc
					&& 0 == fnmatch(pattern, elinkp->portselp1->NodeDesc, 0))
				|| (elinkp->portselp2 && elinkp->portselp2->NodeDesc
					&& 0 == fnmatch(pattern, elinkp->portselp2->NodeDesc, 0)))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Node name pattern Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	return FSUCCESS;
}

// search for nodes whose ExpectedNode has the given node details
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodeDetailsPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ENODE)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
			NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
			if (nodep->enodep && nodep->enodep->details
				&& fnmatch(pattern, nodep->enodep->details, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_NODE_LIST, nodep);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ENODE) {
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->details
					&& fnmatch(pattern, enodep->details, 0) == 0)
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}

	// N/A for FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Node Details Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}
#endif

// search for the NodeData corresponding to the given node type
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindNodeTypePoint(FabricData_t *fabricp, NODE_TYPE type, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == find_flag)
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
			NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
			if (nodep->NodeInfo.NodeType == type)
			{
				status = PointListAppend(pPoint, POINT_TYPE_NODE_LIST, nodep);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ENODE) {
		LIST_ITEM *p;
		QUICK_LIST *pList = &fabricp->ExpectedFIs;
		while (pList != NULL) {
			for (p=QListHead(pList); p != NULL; p = QListNext(pList, p)) {
				ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
				if (enodep->NodeType == type)
				{
					status = PointEnodeListAppend(pPoint, POINT_ENODE_TYPE_NODE_LIST, enodep);
					if (FSUCCESS != status)
						return status;
				}
			}
			if (pList == &fabricp->ExpectedFIs)
				pList = &fabricp->ExpectedSWs;
			else
				pList = NULL;
		}
	}
	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			if (esmp->NodeType == type)
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if ( (elinkp->portselp1 && elinkp->portselp1->NodeType == type)
				|| (elinkp->portselp2 && elinkp->portselp2->NodeType == type))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: No nodes of type %s found\n",
						g_Top_cmdname, StlNodeTypeToText(type));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
// search for the Ioc corresponding to the given ioc name
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindIocNamePoint(FabricData_t *fabricp, char *name, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllIOCs); p != cl_qmap_end(&fabricp->AllIOCs); p = cl_qmap_next(p)) {
			IocData *iocp = PARENT_STRUCT(p, IocData, AllIOCsEntry);

			if (strncmp((char*)iocp->IocProfile.IDString, name, IOC_IDSTRING_SIZE) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_IOC_LIST, iocp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK
	
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: IOC name Not Found: %s\n", g_Top_cmdname, name);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

#ifndef __VXWORKS__
// search for the Ioc corresponding to the given ioc name pattern
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindIocNamePatPoint(FabricData_t *fabricp, char *pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllIOCs); p != cl_qmap_end(&fabricp->AllIOCs); p = cl_qmap_next(p)) {
			IocData *iocp = PARENT_STRUCT(p, IocData, AllIOCsEntry);
			char Name[IOC_IDSTRING_SIZE+1];

			strncpy(Name, (char*)iocp->IocProfile.IDString, IOC_IDSTRING_SIZE);
			Name[IOC_IDSTRING_SIZE] = '\0';
			if (fnmatch(pattern, Name, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_IOC_LIST, iocp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: IOC name pattern Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}
#endif

/**
 * Determine IocType of an IocData object
 * @param iocp IocData profile data to determine type from
 * @return The IocType of this IOC
 */
IocType GetIocType(IocData *iocp)
{
	if ((iocp->IocProfile.IOClass == 0xff00
							|| iocp->IocProfile.IOClass == 0x0100)
						&& iocp->IocProfile.IOSubClass == 0x609e
						&& iocp->IocProfile.Protocol == 0x108
						&& iocp->IocProfile.ProtocolVer == 1)
	{
		return IOC_TYPE_SRP;
	}
	//add cases for new IocTypes here
	
	return IOC_TYPE_OTHER;
}

const char * GetIocTypeName(IocType type)
{
	switch(type){
	case IOC_TYPE_SRP:
		return "SRP";
		break;
	case IOC_TYPE_OTHER:
		return "OTHER";
		break;
	default:
		return "";
		break;
	}
}

// search for the Ioc corresponding to the given ioc type
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindIocTypePoint(FabricData_t *fabricp, IocType type, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllIOCs); p != cl_qmap_end(&fabricp->AllIOCs); p = cl_qmap_next(p)) {
			IocData *iocp = PARENT_STRUCT(p, IocData, AllIOCsEntry);
			if (type == GetIocType(iocp))
			{
				status = PointListAppend(pPoint, POINT_TYPE_IOC_LIST, iocp);
				if (FSUCCESS != status)
					return status;
			} 
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: IOC type Not Found: %s\n",
						g_Top_cmdname, GetIocTypeName(type));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

static int CompareGuid(const cl_map_item_t *p_item, const uint64 key)
{
	IocData *iocp = PARENT_STRUCT(p_item, IocData, AllIOCsEntry);
	if( key == iocp->IocProfile.IocGUID) {
		return 0;
	} else if(key < iocp->IocProfile.IocGUID) {
		return 1;
	} else {
		return -1;
	}

}

static cl_map_item_t *GetLowestIocGuid(cl_map_item_t *p, EUI64 guid, cl_qmap_t* IocMap)
{
	cl_map_item_t *returnItem = NULL;
	while(p != cl_qmap_end(IocMap))
	{
		IocData *iocp = PARENT_STRUCT(p, IocData, AllIOCsEntry);
		if(iocp->IocProfile.IocGUID != guid)
			break;
		returnItem = p;
		p =  cl_qmap_prev(p);
	}
	return returnItem;
}

static FSTATUS IocAppend(cl_map_item_t *p, EUI64 guid, Point *pPoint, cl_qmap_t* IocMap)
{
	FSTATUS status;
	while(p != cl_qmap_end(IocMap)) {
		IocData *iocp = PARENT_STRUCT(p, IocData, AllIOCsEntry);
		if(iocp->IocProfile.IocGUID != guid)
			break;
		status = PointListAppend(pPoint, POINT_TYPE_IOC_LIST, iocp);
		if (FSUCCESS != status)
			return status;
		p = cl_qmap_next(p);
	}
	if (! PointValid(pPoint)) {
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the Ioc corresponding to the given Ioc Guid
FSTATUS FindIocGuid(FabricData_t *fabricp, EUI64 guid, Point *pPoint)
{
	FSTATUS status = FNOT_FOUND;
	cl_map_item_t *p, *lowestItem;
	p = cl_qmap_get_item_compare(&fabricp->AllIOCs, guid, CompareGuid);
	if(p == cl_qmap_end(&fabricp->AllIOCs))
		return status;
	lowestItem = GetLowestIocGuid(p, guid, &fabricp->AllIOCs);
	if(lowestItem)
		status = IocAppend(lowestItem, guid, pPoint, &fabricp->AllIOCs);
	return status;


}
#endif

// search for the SystemData corresponding to the given system image Guid
SystemData * FindSystemGuid(FabricData_t *fabricp, EUI64 guid)
{
	cl_map_item_t *mi;

	mi = cl_qmap_get(&fabricp->AllSystems, guid);
	if (mi == cl_qmap_end(&fabricp->AllSystems))
		return NULL;
	return PARENT_STRUCT(mi, SystemData, AllSystemsEntry);
}

// search for the PortData corresponding to the given port rate
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindRatePoint(FabricData_t *fabricp, uint32 rate, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	ASSERT(rate);
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			/* omit switch port 0, rate is often odd */
			if (portp->PortNum == 0)
				continue;
			if (portp->rate == rate) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM

	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if (elinkp->expected_rate == rate)
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Rate Not Found: %s\n",
						g_Top_cmdname, StlStaticRateToText(rate));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}


// search for the PortData corresponding to the given LED state
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindLedStatePoint(FabricData_t *fabricp, uint8 state, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			if (   (state == 1 &&
					portp->PortInfo.PortStates.s.LEDEnabled == 1 )
				|| (state == 0 &&
					portp->PortInfo.PortStates.s.LEDEnabled == 0 ) ) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: LED State Not Found: %d\n",
						g_Top_cmdname,
						state);
		return FNOT_FOUND;
	} 
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the PortData corresponding to the given port state
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindPortStatePoint(FabricData_t *fabricp, uint8 state, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			///* omit switch port 0, state can be odd */
			//if (portp->PortNum == 0)
			//	continue;
			if ((state == PORT_STATE_SEARCH_NOTACTIVE
					&& portp->PortInfo.PortStates.s.PortState != IB_PORT_ACTIVE)
				|| (state == PORT_STATE_SEARCH_INITARMED
					&& (portp->PortInfo.PortStates.s.PortState == IB_PORT_INIT
						|| portp->PortInfo.PortStates.s.PortState == IB_PORT_ARMED))
				|| (portp->PortInfo.PortStates.s.PortState == state)) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Port State Not Found: %s\n",
						g_Top_cmdname,
							 (state==PORT_STATE_SEARCH_NOTACTIVE)?"Not Active":
							 (state==PORT_STATE_SEARCH_INITARMED)?"Init Armed":
													StlPortStateToText(state));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the PortData corresponding to the given port phys state
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindPortPhysStatePoint(FabricData_t *fabricp, IB_PORT_PHYS_STATE physstate, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			///* omit switch port 0, state can be odd */
			//if (portp->PortNum == 0)
			//	continue;
			if (portp->PortInfo.PortStates.s.PortPhysicalState == physstate) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Port Phys State Not Found: %s\n",
						g_Top_cmdname, StlPortPhysStateToText(physstate));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

#ifndef __VXWORKS__
// search for ports whose ExpectedLink has the given cable label
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCableLabelPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			if (! portp->elinkp || ! portp->elinkp->CableData.label)
				continue;	// no cable information
			if (fnmatch(pattern, portp->elinkp->CableData.label, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM
	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if (! elinkp->CableData.label)
				continue;	// no cable information
			if (fnmatch(pattern, elinkp->CableData.label, 0) == 0)
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Cable Label Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for ports whose ExpectedLink has the given cable length
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCableLenPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			if (! portp->elinkp || ! portp->elinkp->CableData.length)
				continue;	// no cable information
			if (fnmatch(pattern, portp->elinkp->CableData.length, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM

	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if (! elinkp->CableData.length)
				continue;	// no cable information
			if (fnmatch(pattern, elinkp->CableData.length, 0) == 0)
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Cable Length Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for ports whose ExpectedLink has the given cable details
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCableDetailsPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			if (portp->elinkp && portp->elinkp->CableData.details
				&& fnmatch(pattern, portp->elinkp->CableData.details, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM

	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if (elinkp->CableData.details
				&& fnmatch(pattern, elinkp->CableData.details, 0) == 0)
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Cable Details Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for ports whose CABLE_INFO has the given length
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCabinfLenPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;
	char *cur, scrubbed_pat[strlen(pattern) + 1];
	char cablen_str[10] = {0}; // strlen("255") + 1 = 4
	int cableInfoHighPageAddressOffset;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;

	// User may suffix pattern with 'm'. Scrub it as to not intefere with later calls
	// to fnmatch().
	snprintf(scrubbed_pat, sizeof(scrubbed_pat), "%s", pattern);
	if (NULL != (cur = strchr(scrubbed_pat, 'm')))
		*cur = '\0';

	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		//For CableHealth Report the Low Addr page and High address page are accessed, so got to specify the offset.
		if(fabricp->flags & FF_CABLELOWPAGE)
			cableInfoHighPageAddressOffset = STL_CIB_STD_HIGH_PAGE_ADDR;
		else
			cableInfoHighPageAddressOffset = 0;

		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			STL_CABLE_INFO_STD *pCableInfo;
			STL_CABLE_INFO_UP0_DD *pCableInfoDD;
			CableTypeInfoType cableTypeInfo;
			boolean qsfp_dd;
			boolean cableLenValid;

			if (!portp->pCableInfoData)
				continue;

			pCableInfo = (STL_CABLE_INFO_STD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			pCableInfoDD = (STL_CABLE_INFO_UP0_DD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			qsfp_dd = (portp->pCableInfoData[0] == STL_CIB_STD_QSFP_DD);

			if (!qsfp_dd) {
				StlCableInfoDecodeCableType(pCableInfo->dev_tech.s.xmit_tech, pCableInfo->connector, pCableInfo->ident, &cableTypeInfo);
				cableLenValid = cableTypeInfo.cableLengthValid;

				StlCableInfoOM4LengthToText(pCableInfo->len_om4, cableLenValid, sizeof(cablen_str), cablen_str);
				if (NULL != (cur = strchr(cablen_str, 'm')))
					*cur = '\0';
				if (fnmatch(scrubbed_pat, cablen_str, 0) != 0)
					continue;

				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			} else {
				StlCableInfoDecodeCableType(pCableInfoDD->cable_type, pCableInfoDD->connector, pCableInfoDD->ident, &cableTypeInfo);
				cableLenValid = cableTypeInfo.cableLengthValid;

				StlCableInfoDDCableLengthToText(pCableInfoDD->cableLengthEnc, cableLenValid, sizeof(cablen_str), cablen_str);
				if (NULL != (cur = strchr(cablen_str, 'm')))
					*cur = '\0';
				if (fnmatch(scrubbed_pat, cablen_str, 0) != 0)
					continue;

				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: CABLE_INFO length Not Found: %s\n",
			g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}

	PointCompress(pPoint);
	return FSUCCESS;

}	// End of FindCabinfLenPatPoint()

// search for ports whose CABLE_INFO has the given vendor name
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCabinfVendNamePatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;
	uint32 len_pattern;
	STL_CABLE_INFO_STD *pCableInfo;
	STL_CABLE_INFO_UP0_DD *pCableInfoDD;
	char bf_pattern[sizeof(pCableInfo->vendor_name) + 1];
	boolean qsfp_dd;
	int cableInfoHighPageAddressOffset;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	len_pattern = strnlen(pattern, sizeof(pCableInfo->vendor_name));
	memset (bf_pattern, ' ', sizeof(bf_pattern));
	memcpy (bf_pattern, pattern, len_pattern);
	if (! len_pattern || (bf_pattern[len_pattern-1] != ' ' && bf_pattern[len_pattern-1] != '*'))
		bf_pattern[sizeof(pCableInfo->vendor_name)] = '\0';
	else
		bf_pattern[len_pattern] = '\0';

	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		//For CableHealth Report the Low Addr page and High address page are accessed, so got to specify the offset.
		if(fabricp->flags & FF_CABLELOWPAGE)
			cableInfoHighPageAddressOffset = STL_CIB_STD_HIGH_PAGE_ADDR;
		else
			cableInfoHighPageAddressOffset = 0;

		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			char tempStr[sizeof(pCableInfo->vendor_name) + 1];

			if (!portp->pCableInfoData)
				continue;

			pCableInfo = (STL_CABLE_INFO_STD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			pCableInfoDD = (STL_CABLE_INFO_UP0_DD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			qsfp_dd = (portp->pCableInfoData[0] == STL_CIB_STD_QSFP_DD);
			if (!qsfp_dd) {
				memcpy(tempStr, pCableInfo->vendor_name, sizeof(pCableInfo->vendor_name));
				tempStr[sizeof(pCableInfo->vendor_name)] = '\0';
			} else {
				memcpy(tempStr, pCableInfoDD->vendor_name, sizeof(pCableInfoDD->vendor_name));
				tempStr[sizeof(pCableInfoDD->vendor_name)] = '\0';
			}
			if (fnmatch(bf_pattern, (const char *)tempStr, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: CABLE_INFO vendor name Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;

}	// End of FindCabinfVendNamePatPoint()

// search for ports whose CABLE_INFO has the given vendor part number
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCabinfVendPNPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;
	uint32 len_pattern;
	STL_CABLE_INFO_STD *pCableInfo;
	STL_CABLE_INFO_UP0_DD *pCableInfoDD;
	char bf_pattern[sizeof(pCableInfo->vendor_pn) + 1];
	boolean qsfp_dd;
	int cableInfoHighPageAddressOffset;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	len_pattern = strnlen(pattern, sizeof(pCableInfo->vendor_pn));
	memset (bf_pattern, ' ', sizeof(bf_pattern));
	memcpy (bf_pattern, pattern, len_pattern);
	if (! len_pattern || (bf_pattern[len_pattern-1] != ' ' && bf_pattern[len_pattern-1] != '*'))
		bf_pattern[sizeof(pCableInfo->vendor_pn)] = '\0';
	else
		bf_pattern[len_pattern] = '\0';

	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		//For CableHealth Report the Low Addr page and High address page are accessed, so got to specify the offset.
		if(fabricp->flags & FF_CABLELOWPAGE)
			cableInfoHighPageAddressOffset = STL_CIB_STD_HIGH_PAGE_ADDR;
		else
			cableInfoHighPageAddressOffset = 0;

		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			char tempStr[sizeof(pCableInfo->vendor_pn) + 1];

			if (!portp->pCableInfoData)
				continue;

			pCableInfo = (STL_CABLE_INFO_STD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			pCableInfoDD = (STL_CABLE_INFO_UP0_DD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			qsfp_dd = (portp->pCableInfoData[0] == STL_CIB_STD_QSFP_DD);
			if (!qsfp_dd) {
				memcpy(tempStr, pCableInfo->vendor_pn, sizeof(pCableInfo->vendor_pn));
				tempStr[sizeof(pCableInfo->vendor_pn)] = '\0';
			} else {
				memcpy(tempStr, pCableInfoDD->vendor_pn, sizeof(pCableInfoDD->vendor_pn));
				tempStr[sizeof(pCableInfoDD->vendor_pn)] = '\0';
			}
			if (fnmatch(bf_pattern, (const char *)tempStr, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: CABLE_INFO vendor PN Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;

}	// End of FindCabinfVendPNPatPoint()

// search for ports whose CABLE_INFO has the given vendor rev
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCabinfVendRevPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;
	uint32 len_pattern;
	STL_CABLE_INFO_STD *pCableInfo;
	STL_CABLE_INFO_UP0_DD *pCableInfoDD;
	char bf_pattern[sizeof(pCableInfo->vendor_rev) + 1];
	boolean qsfp_dd;
	int cableInfoHighPageAddressOffset;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	len_pattern = strnlen(pattern, sizeof(pCableInfo->vendor_rev));
	memset (bf_pattern, ' ', sizeof(bf_pattern));
	memcpy (bf_pattern, pattern, len_pattern);
	if (! len_pattern || (bf_pattern[len_pattern-1] != ' ' && bf_pattern[len_pattern-1] != '*'))
		bf_pattern[sizeof(pCableInfo->vendor_rev)] = '\0';
	else
		bf_pattern[len_pattern] = '\0';

	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		//For CableHealth Report the Low Addr page and High address page are accessed, so got to specify the offset.
		if(fabricp->flags & FF_CABLELOWPAGE)
			cableInfoHighPageAddressOffset = STL_CIB_STD_HIGH_PAGE_ADDR;
		else
			cableInfoHighPageAddressOffset = 0;

		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			char tempStr[sizeof(pCableInfo->vendor_rev) + 1];

			if (!portp->pCableInfoData)
				continue;

			pCableInfo = (STL_CABLE_INFO_STD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			pCableInfoDD = (STL_CABLE_INFO_UP0_DD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			qsfp_dd = (portp->pCableInfoData[0] == STL_CIB_STD_QSFP_DD);
			if (!qsfp_dd) {
				memcpy(tempStr, pCableInfo->vendor_rev, sizeof(pCableInfo->vendor_rev));
				tempStr[sizeof(pCableInfo->vendor_rev)] = '\0';
			} else {
				memcpy(tempStr, pCableInfoDD->vendor_rev, sizeof(pCableInfoDD->vendor_rev));
				tempStr[sizeof(pCableInfoDD->vendor_rev)] = '\0';
			}
			if (fnmatch(bf_pattern, (const char *)tempStr, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: CABLE_INFO vendor rev Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;

}	// End of FindCabinfVendRevPatPoint()

// search for ports whose CABLE_INFO has the given vendor serial number
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCabinfVendSNPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;
	uint32 len_pattern;
	STL_CABLE_INFO_STD *pCableInfo;
	STL_CABLE_INFO_UP0_DD *pCableInfoDD;
	char bf_pattern[sizeof(pCableInfo->vendor_sn) + 1];
	boolean qsfp_dd;
	int cableInfoHighPageAddressOffset;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	len_pattern = strnlen(pattern, sizeof(pCableInfo->vendor_sn));
	memset (bf_pattern, ' ', sizeof(bf_pattern));
	memcpy (bf_pattern, pattern, len_pattern);
	if (! len_pattern || (bf_pattern[len_pattern-1] != ' ' && bf_pattern[len_pattern-1] != '*'))
		bf_pattern[sizeof(pCableInfo->vendor_sn)] = '\0';
	else
		bf_pattern[len_pattern] = '\0';

	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		//For CableHealth Report the Low Addr page and High address page are accessed, so got to specify the offset.
		if(fabricp->flags & FF_CABLELOWPAGE)
			cableInfoHighPageAddressOffset = STL_CIB_STD_HIGH_PAGE_ADDR;
		else
			cableInfoHighPageAddressOffset = 0;

		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			char tempStr[sizeof(pCableInfo->vendor_sn) + 1];

			if (!portp->pCableInfoData)
				continue;

			pCableInfo = (STL_CABLE_INFO_STD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			pCableInfoDD = (STL_CABLE_INFO_UP0_DD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			qsfp_dd = (portp->pCableInfoData[0] == STL_CIB_STD_QSFP_DD);
			if (!qsfp_dd) {
				memcpy(tempStr, pCableInfo->vendor_sn, sizeof(pCableInfo->vendor_sn));
				tempStr[sizeof(pCableInfo->vendor_sn)] = '\0';
			} else {
				memcpy(tempStr, pCableInfoDD->vendor_sn, sizeof(pCableInfoDD->vendor_sn));
				tempStr[sizeof(pCableInfoDD->vendor_sn)] = '\0';
			}
			if (fnmatch(bf_pattern, (const char *)tempStr, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: CABLE_INFO vendor SN Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;

}	// End of FindCabinfVendSNPatPoint()

// search for the PortData corresponding to the given cable type
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindCabinfCableTypePoint(FabricData_t *fabricp, char *pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;
	int len;
	int cableInfoHighPageAddressOffset;

	len = strlen(pattern);

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		//For CableHealth Report the Low Addr page and High address page are accessed, so got to specify the offset.
		if(fabricp->flags & FF_CABLELOWPAGE)
			cableInfoHighPageAddressOffset = STL_CIB_STD_HIGH_PAGE_ADDR;
		else
			cableInfoHighPageAddressOffset = 0;

		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			STL_CABLE_INFO_STD *pCableInfo;
			STL_CABLE_INFO_UP0_DD *pCableInfoDD;
			uint8 xmit_tech;
			boolean qsfp_dd;

			/* omit switch port 0, no cable connected to port0 */
			if (portp->PortNum == 0)
				continue;

			if (!portp->pCableInfoData)
				continue;

			pCableInfo = (STL_CABLE_INFO_STD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			pCableInfoDD = (STL_CABLE_INFO_UP0_DD *)(portp->pCableInfoData + cableInfoHighPageAddressOffset);
			qsfp_dd = (portp->pCableInfoData[0] == STL_CIB_STD_QSFP_DD);
			if (!qsfp_dd)
				xmit_tech = pCableInfo->dev_tech.s.xmit_tech;
			else
				xmit_tech = pCableInfoDD->cable_type;
			if (strncmp(pattern, "optical", len) == 0) // this includes AOL and Optical Transceiver
				if (xmit_tech <= STL_CIB_STD_TXTECH_1490_DFB && (xmit_tech >= STL_CIB_STD_TXTECH_850_VCSEL)
						&& (xmit_tech != STL_CIB_STD_TXTECH_OTHER)) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
			if (strncmp(pattern, "unknown", len) == 0)
				if (xmit_tech == STL_CIB_STD_TXTECH_OTHER) {
					status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
					if (FSUCCESS != status)
						return status;
			}
			if (strncmp(pattern, "passive_copper", len) == 0)
				if (xmit_tech == STL_CIB_STD_TXTECH_CU_UNEQ || (xmit_tech == STL_CIB_STD_TXTECH_CU_PASSIVEQ)) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
			if (strncmp(pattern, "active_copper", len) == 0)
				if (xmit_tech <= STL_CIB_STD_TXTECH_CU_LINACTEQ && (xmit_tech >= STL_CIB_STD_TXTECH_CU_NFELIMACTEQ)) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: CABLE_INFO: No %s cables found\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;

} // End of FindCabinfCableTypePoint()

// search for ports whose ExpectedLink has the given link details
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindLinkDetailsPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;	
	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			if (portp->elinkp && portp->elinkp->details
				&& fnmatch(pattern, portp->elinkp->details, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM

	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if (elinkp->details && fnmatch(pattern, elinkp->details, 0) == 0)
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Link Details Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for ports whose ExpectedLink has the given port details
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindPortDetailsPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			PortSelector *portselp = GetPortSelector(portp);

			if (portselp && portselp->details
				&& fnmatch(pattern, portselp->details, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM

	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if ((elinkp->portselp1 && elinkp->portselp1->details
					&& fnmatch(pattern, elinkp->portselp1->details, 0) == 0)
				|| (elinkp->portselp2 && elinkp->portselp2->details
					&& fnmatch(pattern, elinkp->portselp2->details, 0) == 0))
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Port Details Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}
#endif

// search for the PortData corresponding to the given port MTU capability
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindMtuPoint(FabricData_t *fabricp, IB_MTU mtu, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	ASSERT(mtu);
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			/* omit switch port 0, mtu is often odd */
			if (portp->PortNum == 0)
				continue;
			if(MIN(portp->PortInfo.MTU.Cap, portp->neighbor->PortInfo.MTU.Cap) == mtu){
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE and FIND_FLAG_ESM

	if (find_flag & FIND_FLAG_ELINK) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
			ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
			if(elinkp->expected_mtu == mtu)
			{
				status = PointElinkListAppend(pPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: MTU Not Found: %s\n",
						g_Top_cmdname, IbMTUToText(mtu));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// search for the master SM (should be only 1, so we return 1st master found)
PortData * FindMasterSm(FabricData_t *fabricp)
{
	cl_map_item_t *p;

	for (p=cl_qmap_head(&fabricp->AllSMs); p != cl_qmap_end(&fabricp->AllSMs); p = cl_qmap_next(p)) {
		SMData *smp = PARENT_STRUCT(p, SMData, AllSMsEntry);
		if (smp->SMInfoRecord.SMInfo.u.s.SMStateCurrent == SM_MASTER)
			return smp->portp;
	}
	return NULL;
}

#ifndef __VXWORKS__
// search for SMData whose ExpectedSM has the given SM details
// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindSmDetailsPatPoint(FabricData_t *fabricp, const char* pattern, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ESM)))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		cl_map_item_t *p;
		for (p=cl_qmap_head(&fabricp->AllSMs); p != cl_qmap_end(&fabricp->AllSMs); p = cl_qmap_next(p)) {
			SMData *smp = PARENT_STRUCT(p, SMData, AllSMsEntry);
			if (! smp->esmp || ! smp->esmp->details)
				continue;	// no SM details information
			if (fnmatch(pattern, smp->esmp->details, 0) == 0)
			{
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, smp->portp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE

	if (find_flag & FIND_FLAG_ESM) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
			ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
			if (! esmp->details)
				continue;	// no SM details information
			if (fnmatch(pattern, esmp->details, 0) == 0)
			{
				status = PointEsmListAppend(pPoint, POINT_ESM_TYPE_SM_LIST, esmp);
				if (FSUCCESS != status)
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: SM Details Not Found: %s\n",
						g_Top_cmdname, pattern);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}
#endif

// search for the SMData corresponding to the given PortGuid
SMData * FindSMPort(FabricData_t *fabricp, EUI64 PortGUID)
{
	cl_map_item_t *mi;

	mi = cl_qmap_get(&fabricp->AllSMs, PortGUID);
	if (mi == cl_qmap_end(&fabricp->AllSMs))
		return NULL;
	return PARENT_STRUCT(mi, SMData, AllSMsEntry);
}

// search for the PortData corresponding to the given lid and port number
// For FIs lid completely defines the port
// For Switches, lid will identify the switch and port is used to select port
PortData * FindLidPort(FabricData_t *fabricp, STL_LID lid, uint8 port)
{
	PortData *portp;

	portp = FindLid(fabricp, lid);
	if (! portp)
		return NULL;
	/* lid is only valid for port 0 on switches
	 * ignore port for non-switches, in which case LID is all we need
	 */
	if (portp->PortNum == port
		|| portp->nodep->NodeInfo.NodeType != STL_NODE_SW)
		return portp;
	/* must be a port w/o a LID on a switch */
	return FindNodePort(portp->nodep, port);
}

// search for the PortData corresponding to the given nodeguid and port number
PortData *FindNodeGuidPort(FabricData_t *fabricp, EUI64 nodeguid, uint8 port)
{
	NodeData *nodep = FindNodeGuid(fabricp, nodeguid);
	if (! nodep)
		return NULL;
	return FindNodePort(nodep, port);
}

// search ExpectedNodeGuidMap for GUID
// Search the ExpectedNodeGuidMap for an ExpectedNode with the given nodeGuid
// This will cover all ExpectedFIs and ExpectedSWs which have Guids specified.
ExpectedNode* FindExpectedNodeByNodeGuid(const FabricData_t *fabricp, EUI64 nodeGuid)
{
	cl_map_item_t *mi;
	ExpectedNode *enodep;

	if(fabricp == NULL)
		return NULL;

	mi = cl_qmap_get(&fabricp->ExpectedNodeGuidMap, nodeGuid);
	if (mi == cl_qmap_end(&fabricp->ExpectedNodeGuidMap))
		return NULL;

	enodep = PARENT_STRUCT(mi, ExpectedNode, ExpectedNodeGuidMapEntry);
	return enodep;
}

// Search through the ExpectedFIs and ExpectedSWs for an ExpectedNode with the
// given node description
// NodeType is optional and may limit scope of search
ExpectedNode* FindExpectedNodeByNodeDesc(const FabricData_t* fabricp, const char* nodeDesc, uint8 NodeType)
{
	LIST_ITEM *p;

	if(fabricp == NULL)
		return NULL;

	if (nodeDesc == NULL)
		return NULL;

	// Since switches have multiple ports, when this is called as part of
	// topology analysis or ExpectedLink analysis we have a better chance
	// of finding switch, plus there are often less switches than FIs in a
	// fabric, so we check switches first to improve performance on many use
	// cases

	// First check through the switches
	if(NodeType != STL_NODE_FI && QListHead(&fabricp->ExpectedSWs) != NULL) {
		for(p = QListHead(&fabricp->ExpectedSWs); p != NULL; p = QListNext(&fabricp->ExpectedSWs, p)) {
			ExpectedNode* enodep = PARENT_STRUCT(p, ExpectedNode, ExpectedNodesEntry);

			if (enodep->NodeDesc && 0 == strncmp(enodep->NodeDesc,
                                        nodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE))
				return enodep;
		}
	}

	// Check through FIs if it wasn't a switch
	if(NodeType != STL_NODE_SW && QListHead(&fabricp->ExpectedFIs) != NULL) {
		for(p = QListHead(&fabricp->ExpectedFIs); p != NULL; p = QListNext(&fabricp->ExpectedFIs, p)) {
			ExpectedNode* enodep = PARENT_STRUCT(p, ExpectedNode, ExpectedNodesEntry);

			if (enodep->NodeDesc && 0 == strncmp(enodep->NodeDesc,
                                        nodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE))
				return enodep;
		}
	}

	return NULL;
}

// Search for the ExpectedLink by one side of the link with nodeGuid & portNum. 
// (OPTIONAL) Side is which portsel in the ExpectedLink that was the one given.
ExpectedLink* FindExpectedLinkByOneSide(const FabricData_t *fabricp, EUI64 nodeGuid, uint8 portNum, uint8* side)
{
	ExpectedLink *elinkp;
	ExpectedNode *enodep = FindExpectedNodeByNodeGuid(fabricp, nodeGuid);
	if(!enodep)
		return NULL;

	if(portNum >= enodep->portsSize ||  enodep->ports[portNum] == NULL) {
		return NULL;
	}
	elinkp = enodep->ports[portNum]->elinkp;

	if(!elinkp)
		return NULL;
		
	if(elinkp->portselp1->NodeGUID == nodeGuid && elinkp->portselp1->PortNum == portNum) {
		if(side)
			*side = 1;
	} else if (elinkp->portselp2->NodeGUID == nodeGuid && elinkp->portselp2->PortNum == portNum) {
		if(side)
			*side = 2;	
	}
	
	return elinkp;

}

// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindLinkCRCPoint(FabricData_t *fabricp, uint16 crc, LinkCRCCompare comp, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			boolean match = FALSE;
			/* omit switch port 0, CRC is often odd and N/A */
			if (portp->PortNum == 0)
				continue;
			switch (comp) {
			case CRC_EQ:
				match = (portp->PortInfo.PortLTPCRCMode.s.Active == crc );
				break;
			case CRC_NE:
				match = (portp->PortInfo.PortLTPCRCMode.s.Active != crc );
				break;
			default:
				break;
			}
			if (match) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status) 
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		char tempbuf[20];
		fprintf(stderr, "%s: Link CRC Not Found: %s %s\n",
						g_Top_cmdname,
						(comp == CRC_EQ)? "EQ"
						: (comp == CRC_NE) ? "NE"
						: "", /* should not happen */
						StlPortLtpCrcModeToText(crc, tempbuf, sizeof(tempbuf)));
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

// FNOT_FOUND - no instances found
// FINVALID_OPERATION - find_flag contains no applicable searches
// other - error allocating memory or initializing structures
FSTATUS FindLinkQualityPoint(FabricData_t *fabricp, uint16 quality, LinkQualityCompare comp, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			boolean match = FALSE;
			if (!portp->pPortCounters)
				continue;
			switch (comp) {
			case QUAL_EQ:
				match = (uint16)(portp->pPortCounters->lq.s.linkQualityIndicator) == quality;
				break;
			case QUAL_GE:
				match = (uint16)(portp->pPortCounters->lq.s.linkQualityIndicator) >= quality;
				break;
			case QUAL_LE:
				match = (uint16)(portp->pPortCounters->lq.s.linkQualityIndicator) <= quality;
				break;
			default:
				break;
			}
			if (match) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status) 
					return status;
			}
		}
	}

	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK

	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Link Quality Not Found: %s %d\n",
						g_Top_cmdname,
						(comp == QUAL_EQ)? "EQ"
						: (comp == QUAL_GE) ? "GE"
						: (comp == QUAL_LE) ? "LE"
						: "", /* should not happen */
						quality);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}

FSTATUS FindLinkDownReasonPoint(FabricData_t *fabricp, uint8 ldr, Point *pPoint, uint8 find_flag)
{
	FSTATUS status;

	ASSERT(PointIsInInit(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;

	if (find_flag & FIND_FLAG_FABRIC) {
		LIST_ITEM *p;
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);

			boolean match = FALSE;
			int i;
			if (fabricp->flags & FF_SMADIRECT) { // SMA only
				match = ldr == IB_UINT8_MAX
					? portp->PortInfo.LinkDownReason != STL_LINKDOWN_REASON_NONE
					: portp->PortInfo.LinkDownReason == ldr;
				match |= ldr == IB_UINT8_MAX
					? portp->PortInfo.NeighborLinkDownReason != STL_LINKDOWN_REASON_NONE
					: portp->PortInfo.NeighborLinkDownReason == ldr;
			} else { // SA
				for (i = 0; i < STL_NUM_LINKDOWN_REASONS; ++i) {
					STL_LINKDOWN_REASON *ldrp = &portp->LinkDownReasons[i];
					if (ldrp->Timestamp != 0) {
						match = ldr == IB_UINT8_MAX
							? ldrp->LinkDownReason != STL_LINKDOWN_REASON_NONE
							: ldrp->LinkDownReason == ldr;
						match |= ldr == IB_UINT8_MAX
							? ldrp->NeighborLinkDownReason != STL_LINKDOWN_REASON_NONE
							: ldrp->NeighborLinkDownReason == ldr;
					}
					if (match) break;
				}
			}
			if (match) {
				status = PointListAppend(pPoint, POINT_TYPE_PORT_LIST, portp);
				if (FSUCCESS != status) 
					return status;
			}
		}
	}
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Link Down Reason Not Found: %d\n", g_Top_cmdname, ldr);
		return FNOT_FOUND;
	}
	PointCompress(pPoint);
	return FSUCCESS;
}


// Search through the ExpectedSMs for matching portGuid
// FNOT_FOUND - no instances found
// FINVALID_PARAMETER - input parameter not valid
// FSUCCESS - when a match is found
FSTATUS FindExpectedSMByPortGuid(FabricData_t *fabricp, EUI64 portGuid) {
	LIST_ITEM *p;

	if(fabricp == NULL)
		return FINVALID_PARAMETER;

	// check through the SMs
	for(p = QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
		ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
		if (esmp->PortGUID == portGuid)
			return FSUCCESS;
	}
	return FNOT_FOUND;
}

// Search through the ExpectedSMs for matching nodeGuid
// FNOT_FOUND - no instances found
// FINVALID_PARAMETER - input parameter not valid
// FSUCCESS - when a match is found
FSTATUS FindExpectedSMByNodeGuid(FabricData_t *fabricp, EUI64 nodeGuid) {
	LIST_ITEM *p;

	if(fabricp == NULL)
		return FINVALID_PARAMETER;

	// check through the SMs
	for(p = QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
		ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
		if (esmp->NodeGUID == nodeGuid)
			return FSUCCESS;
	}
	return FNOT_FOUND;
}
