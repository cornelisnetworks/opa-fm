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

#ifndef __VXWORKS__
#include <strings.h>
#include <fnmatch.h>
#else
#include "icsBspUtil.h"
#endif

#define FILENAME				256 //Length of file name
#define MIN_LIST_ITEMS				100 // minimum items for ListInit to allocate for
#define NODE_PAIR_DELIMITER_SIZE		1 //Length of delimiter while parsing for node pairs



// Functions to Parse Focus arguments and build POINTs for matches
typedef boolean (PointPortElinkCompareCallback_t)(ExpectedLink *elinkp, void *nodecmp, uint8 portnum);
static FSTATUS ParsePointPort(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, PointPortElinkCompareCallback_t *callback, void *nodecmp, char **pp);

/* check arg to see if 1st characters match prefix, if so return pointer
 * into arg just after prefix, otherwise return NULL
 */
char* ComparePrefix(char *arg, const char *prefix)
{
	int len = strlen(prefix);
	if (strncmp(arg, prefix, len) == 0)
		return arg+len;
	else
		return NULL;
}

static FSTATUS ParseGidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	IB_GID gid;
	
	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToGid(&gid.AsReg64s.H, &gid.AsReg64s.L, arg, pp, TRUE)) {
		fprintf(stderr, "%s: Invalid GID format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! gid.AsReg64s.H || ! gid.AsReg64s.L) {
		fprintf(stderr, "%s: Invalid GID: 0x%016"PRIx64":0x%016"PRIx64"\n",
					g_Top_cmdname, gid.AsReg64s.H, gid.AsReg64s.L);
		return FINVALID_PARAMETER;
	}
#if 0
	/* This could be an optimization of the error case, testing SubnetPrefix
	 * below should be sufficient until we encounter IB routers
	 */
	/* TBD - if we allow for MC GIDs, this test is inappropriate */
	/* TBD - cleanup use of g_portAttrib global */
	if (g_portAttrib
		&& gid.Type.Global.SubnetPrefix != g_portAttrib->GIDTable[0].Type.Global.SubnetPrefix) {
		fprintf(stderr, "%s: Invalid GID: 0x%016"PRIx64":0x%016"PRIx64"\n",
					g_Top_cmdname, 
					gid.Type.Global.SubnetPrefix, gid.Type.Global.InterfaceID);
		fprintf(stderr, "%s: Subnet Prefix: 0x%016"PRIx64" does not match local port: 0x%016"PRIx64"\n",
					g_Top_cmdname, gid.Type.Global.SubnetPrefix, 
					g_portAttrib->GIDTable[0].Type.Global.SubnetPrefix);
		return FNOT_FOUND;
	}
#endif
	return FindGidPoint(fabricp, gid, pPoint, find_flag, 0);
}

static FSTATUS ParseLidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	STL_LID lid;
	PortData *portp = NULL;
	char *param;
	
	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToUint32(&lid, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid LID format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! lid) {
		fprintf(stderr, "%s: Invalid LID: 0x%x\n", g_Top_cmdname, lid);
		return FINVALID_PARAMETER;
	}

	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		portp = FindLid(fabricp, lid);
		if (portp) {
			pPoint->u.portp = portp;
			pPoint->Type = POINT_TYPE_PORT;
		}
	}
	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK
	if (PointValid(pPoint)) {
		ASSERT(portp);	// since we only handle FIND_FLAG_FABRIC
		if (NULL != (param = ComparePrefix(*pp, ":node"))) {
			*pp = param;
			pPoint->u.nodep = portp->nodep;
			pPoint->Type = POINT_TYPE_NODE;
		} else if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
			pPoint->u.nodep = portp->nodep;
			pPoint->Type = POINT_TYPE_NODE;
			return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
		}
		return FSUCCESS;
	} else {
		fprintf(stderr, "%s: LID Not Found: 0x%x\n", g_Top_cmdname, lid);
		return FNOT_FOUND;
	}
}

static FSTATUS ParsePortGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	EUI64 guid;
	
	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToUint64(&guid, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid Port GUID format: '%s'\n",
						g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! guid) {
		fprintf(stderr, "%s: Invalid Port GUID: 0x%016"PRIx64"\n",
						g_Top_cmdname, guid);
		return FINVALID_PARAMETER;
	}
	return FindPortGuidPoint(fabricp, guid, pPoint, find_flag, 0);
}

/* Parse a :port:# suffix, this will limit the Point to the list of ports
 * with the given number
 */
static FSTATUS ParsePointPort(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, PointPortElinkCompareCallback_t *callback, void *nodecmp, char **pp)
{
	uint8 portnum;
	FSTATUS status;
	Point newPoint;
	boolean fabric_fail = FALSE;

	ASSERT(PointValid(pPoint));
	PointInit(&newPoint);
	if (FSUCCESS != StringToUint8(&portnum, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid Port Number format: '%s'\n",
						g_Top_cmdname, arg);
		status = FINVALID_PARAMETER;
		goto fail;
	}
	// for fabric we try to provide a more detailed message using the
	// fabric_fail flag and leaving some of the information in the pPoint
	if (find_flag & FIND_FLAG_FABRIC) {
		switch (pPoint->Type) {
		case POINT_TYPE_NONE:
			break;
		case POINT_TYPE_PORT:
			ASSERT(0);	// should not be called for this type of point
			status = FINVALID_PARAMETER;
			goto fail;
		case POINT_TYPE_PORT_LIST:
			ASSERT(0);	// should not be called for this type of point
			status = FINVALID_PARAMETER;
			goto fail;
		case POINT_TYPE_NODE_PAIR_LIST:
			ASSERT(0);	// should not be called for this type of point
			status = FINVALID_PARAMETER;
			goto fail;
		case POINT_TYPE_NODE:
			{
			PortData *portp = FindNodePort(pPoint->u.nodep, portnum);
			if (! portp) {
				fabric_fail = TRUE;
			} else {
				pPoint->Type = POINT_TYPE_PORT;
				pPoint->u.portp = portp;
			}
			}
			break;
		case POINT_TYPE_NODE_LIST:
			{
			LIST_ITERATOR i;
			DLIST *pNodeList = &pPoint->u.nodeList;

			for (i=ListHead(pNodeList); i != NULL; i = ListNext(pNodeList, i)) {
				NodeData *nodep = (NodeData*)ListObj(i);
				PortData *portp = FindNodePort(nodep, portnum);
				if (portp) {
					status = PointListAppend(&newPoint, POINT_TYPE_PORT_LIST, portp);
					if (FSUCCESS != status)
						goto fail;
				}
			}
			if (! PointValid(&newPoint)) {
				fabric_fail = TRUE;
			} else {
				PointFabricCompress(&newPoint);
				status = PointFabricCopy(pPoint, &newPoint);
				PointDestroy(&newPoint);
				if (FSUCCESS != status)
					goto fail;
			}
			}
			break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
		case POINT_TYPE_IOC:
			{
			PortData *portp = FindNodePort(pPoint->u.iocp->ioup->nodep, portnum);
			if (! portp) {
				fabric_fail = TRUE;
			} else {
				pPoint->Type = POINT_TYPE_PORT;
				pPoint->u.portp = portp;
			}
			}
			break;
		case POINT_TYPE_IOC_LIST:
			{
			LIST_ITERATOR i;
			DLIST *pIocList = &pPoint->u.iocList;

			for (i=ListHead(pIocList); i != NULL; i = ListNext(pIocList, i)) {
				IocData *iocp = (IocData*)ListObj(i);
				PortData *portp = FindNodePort(iocp->ioup->nodep, portnum);
				if (portp) {
					status = PointListAppend(&newPoint, POINT_TYPE_PORT_LIST, portp);
					if (FSUCCESS != status)
						goto fail;
				}
			}
			if (! PointValid(&newPoint)) {
				fabric_fail = TRUE;
			} else {
				PointFabricCompress(&newPoint);
				status = PointFabricCopy(pPoint, &newPoint);
				PointDestroy(&newPoint);
				if (FSUCCESS != status)
					goto fail;
			}
			}
			break;
#endif
		case POINT_TYPE_SYSTEM:
			{
			cl_map_item_t *p;
			SystemData *systemp = pPoint->u.systemp;

			for (p=cl_qmap_head(&systemp->Nodes); p != cl_qmap_end(&systemp->Nodes); p = cl_qmap_next(p)) {
				NodeData *nodep = PARENT_STRUCT(p, NodeData, SystemNodesEntry);
				PortData *portp = FindNodePort(nodep, portnum);
				if (portp) {
					status = PointListAppend(&newPoint, POINT_TYPE_PORT_LIST, portp);
					if (FSUCCESS != status)
						goto fail;
				}
			}
			if (! PointValid(&newPoint)) {
				fabric_fail = TRUE;
			} else {
				PointFabricCompress(&newPoint);
				status = PointFabricCopy(pPoint, &newPoint);
				PointDestroy(&newPoint);
				if (FSUCCESS != status)
					goto fail;
			}
			}
			break;
		}
	}

	// for FIND_FLAG_ENODE leave any selected ExpectedNodes as is
	
	if (find_flag & FIND_FLAG_ESM) {
		switch (pPoint->EsmType) {
		case POINT_ESM_TYPE_NONE:
			break;
		case POINT_ESM_TYPE_SM:
			if (! (pPoint->u3.esmp->gotPortNum
					 && pPoint->u3.esmp->PortNum == portnum)) {
				PointEsmDestroy(pPoint);
			}
			break;
		case POINT_ESM_TYPE_SM_LIST:
			{
			LIST_ITERATOR i;
			DLIST *pEsmList = &pPoint->u3.esmList;

			for (i=ListHead(pEsmList); i != NULL; i = ListNext(pEsmList, i)) {
				ExpectedSM *esmp = (ExpectedSM*)ListObj(i);
				if (esmp->gotPortNum && esmp->PortNum == portnum) {
					status = PointEsmListAppend(&newPoint, POINT_ESM_TYPE_SM_LIST, esmp);
					if (FSUCCESS != status)
						goto fail;
				}
			}
			if (! PointValid(&newPoint)) {
				PointEsmDestroy(pPoint);
			} else {
				PointEsmCompress(&newPoint);
				status = PointEsmCopy(pPoint, &newPoint);
				PointDestroy(&newPoint);
				if (FSUCCESS != status)
					goto fail;
			}
			}
			break;
		}
	}

	if ((find_flag & FIND_FLAG_ELINK) && callback) {
		// rather than retain extra information to indicate which side of
		// the ELINK matched the original search criteria (nodeguid, nodedesc,
		// nodedescpat, nodetype) we use a callback to
		// recompare with the original criteria and also check against portnum
		// N/A search critieria: nodedetpat, iocname, ioctype, iocnamepat
		switch (pPoint->ElinkType) {
		case POINT_ELINK_TYPE_NONE:
			break;
		case POINT_ELINK_TYPE_LINK:
			if (! (*callback)(pPoint->u4.elinkp, nodecmp, portnum)){
				PointElinkDestroy(pPoint);
			}
			break;
		case POINT_ELINK_TYPE_LINK_LIST:
			{
			LIST_ITERATOR i;
			DLIST *pElinkList = &pPoint->u4.elinkList;

			for (i=ListHead(pElinkList); i != NULL; i = ListNext(pElinkList, i)) {
				ExpectedLink *elinkp = (ExpectedLink*)ListObj(i);
				if ((*callback)(elinkp, nodecmp, portnum)){
					status = PointElinkListAppend(&newPoint, POINT_ELINK_TYPE_LINK_LIST, elinkp);
					if (FSUCCESS != status)
						goto fail;
				}
			}
			if (! PointValid(&newPoint)) {
				PointElinkDestroy(pPoint);
			} else {
				PointElinkCompress(&newPoint);
				status = PointElinkCopy(pPoint, &newPoint);
				PointDestroy(&newPoint);
				if (FSUCCESS != status)
					goto fail;
			}
			}
			break;
		}
	} else {
		DEBUG_ASSERT(pPoint->ElinkType == POINT_ELINK_TYPE_NONE);
	}

	if (fabric_fail && pPoint->EnodeType == POINT_ENODE_TYPE_NONE
			&& pPoint->EsmType == POINT_ESM_TYPE_NONE
			&& pPoint->ElinkType == POINT_ELINK_TYPE_NONE) {
		// we failed fabric and had no enode, esm, nor elink, so output a good
		// message specific to the fabric point we must have started with
		switch (pPoint->Type) {
		case POINT_TYPE_NONE:
		case POINT_TYPE_PORT:
		case POINT_TYPE_PORT_LIST:
		case POINT_TYPE_NODE_PAIR_LIST:
			ASSERT(0);	// should not get here
			break;
		case POINT_TYPE_NODE:
			fprintf(stderr, "%s: Node %.*s GUID 0x%016"PRIx64" Port %u Not Found\n",
				g_Top_cmdname, STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)pPoint->u.nodep->NodeDesc.NodeString,
				pPoint->u.nodep->NodeInfo.NodeGUID, portnum);
			break;
		case POINT_TYPE_NODE_LIST:
#if !defined(VXWORKS) || defined(BUILD_DMC)
		case POINT_TYPE_IOC_LIST:
#endif
		case POINT_TYPE_SYSTEM:
			fprintf(stderr, "%s: Port %u Not Found\n", g_Top_cmdname, portnum);
			break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
		case POINT_TYPE_IOC:
			fprintf(stderr, "%s: IOC %.*s GUID 0x%016"PRIx64" Port %u Not Found\n",
					g_Top_cmdname, IOC_IDSTRING_SIZE,
					(char*)pPoint->u.iocp->IocProfile.IDString,
					pPoint->u.iocp->IocProfile.IocGUID, portnum);
			break;
#endif
		}
		status = FNOT_FOUND;
	} else if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Port %u Not Found\n", g_Top_cmdname, portnum);
		status = FNOT_FOUND;
	} else {
		return FSUCCESS;
	}

fail:
	PointDestroy(&newPoint);
	PointDestroy(pPoint);
	return status;
}

static boolean PointPortElinkCompareNodeGuid(ExpectedLink *elinkp, void *nodecmp, uint8 portnum)
{
	EUI64 guid = *(EUI64*)nodecmp;

	return ((elinkp->portselp1 && elinkp->portselp1->NodeGUID == guid
				 && elinkp->portselp1->gotPortNum
				 && elinkp->portselp1->PortNum == portnum)
			|| (elinkp->portselp2 && elinkp->portselp2->NodeGUID == guid
				 && elinkp->portselp2->gotPortNum
				 && elinkp->portselp2->PortNum == portnum));
}

static FSTATUS ParseNodeGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *param;
	EUI64 guid;
	FSTATUS status;
	
	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToUint64(&guid, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid Node GUID format: '%s'\n",
						g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! guid) {
		fprintf(stderr, "%s: Invalid Node GUID: 0x%016"PRIx64"\n",
						g_Top_cmdname, guid);
		return FINVALID_PARAMETER;
	}
	status = FindNodeGuidPoint(fabricp, guid, pPoint, find_flag, 0);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag, 
								PointPortElinkCompareNodeGuid, &guid, pp);
	} else {
		return FSUCCESS;
	}
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
static FSTATUS ParseIocGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *param;
	EUI64 guid;
	FSTATUS status;
	
	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToUint64(&guid, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid IOC GUID format: '%s'\n",
						g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! guid) {
		fprintf(stderr, "%s: Invalid IOC GUID: 0x%016"PRIx64"\n",
						g_Top_cmdname, guid);
		return FINVALID_PARAMETER;
	}
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		status = FindIocGuid(fabricp, guid, pPoint);
	}
	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK
	if (status == FSUCCESS) {
		if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
			return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
		}
		return FSUCCESS;
	} else {
		fprintf(stderr, "%s: IOC GUID Not Found: 0x%016"PRIx64"\n",
						g_Top_cmdname, guid);
		return status;
	}
}
#endif

static FSTATUS ParseSystemGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	EUI64 guid;
	char *param;

	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToUint64(&guid, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid System Image GUID format: '%s'\n",
						g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! guid) {
		fprintf(stderr, "%s: Invalid System Image GUID: 0x%016"PRIx64"\n",
						g_Top_cmdname, guid);
		return FINVALID_PARAMETER;
	}
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		pPoint->u.systemp = FindSystemGuid(fabricp, guid);
		if (pPoint->u.systemp)
			pPoint->Type = POINT_TYPE_SYSTEM;
	}
	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK
	if (PointValid(pPoint)) {
		if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
			return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
		}
		return FSUCCESS;
	} else {
		fprintf(stderr, "%s: System Image GUID Not Found: 0x%016"PRIx64"\n",
						g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
}

static boolean PointPortElinkCompareNodeName(ExpectedLink *elinkp, void *nodecmp, uint8 portnum)
{
	char *name = (char*)nodecmp;

	return ((elinkp->portselp1 && elinkp->portselp1->NodeDesc
				&& elinkp->portselp1->gotPortNum
				&& elinkp->portselp1->PortNum == portnum
				&& 0 == strncmp(elinkp->portselp1->NodeDesc,
								name, STL_NODE_DESCRIPTION_ARRAY_SIZE))
			|| (elinkp->portselp2 && elinkp->portselp2->NodeDesc
				&& elinkp->portselp2->gotPortNum
				&& elinkp->portselp2->PortNum == portnum
				&& 0 == strncmp(elinkp->portselp2->NodeDesc,
								name, STL_NODE_DESCRIPTION_ARRAY_SIZE)));
}

static FSTATUS ParseNodeNamePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Name[STL_NODE_DESCRIPTION_ARRAY_SIZE+1];
	char *param;
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Node name format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > STL_NODE_DESCRIPTION_ARRAY_SIZE) {
			fprintf(stderr, "%s: Node name Not Found (too long): %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Name, arg, sizeof(Name));
		Name[p-arg] = '\0';
		*pp = p;
		arg = Name;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindNodeNamePoint(fabricp, arg, pPoint, find_flag, 0);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag,
								PointPortElinkCompareNodeName, arg, pp);
	} else {
		return FSUCCESS;
	}
}

#ifndef __VXWORKS__
static boolean PointPortElinkCompareNodeNamePat(ExpectedLink *elinkp, void *nodecmp, uint8 portnum)
{
	char *pattern = (char*)nodecmp;

	return ( (elinkp->portselp1 && elinkp->portselp1->NodeDesc
				&& elinkp->portselp1->gotPortNum
				&& elinkp->portselp1->PortNum == portnum
				&& 0 == fnmatch(pattern, elinkp->portselp1->NodeDesc, 0))
			|| (elinkp->portselp2 && elinkp->portselp2->NodeDesc
				&& elinkp->portselp2->gotPortNum
				&& elinkp->portselp2->PortNum == portnum
				&& 0 == fnmatch(pattern, elinkp->portselp2->NodeDesc, 0)));
}

/* Initialize the nodepatPairs list*/
static FSTATUS InitNodePatPairs(NodePairList_t *nodePatPairs)
{
	//Initialize the left side of the list here.
	ListInitState(&nodePatPairs->nodePairList1);
	if (! ListInit(&nodePatPairs->nodePairList1, MIN_LIST_ITEMS)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}

	//Initialize the right side of the list here.
	ListInitState(&nodePatPairs->nodePairList2);
	if (! ListInit(&nodePatPairs->nodePairList2, MIN_LIST_ITEMS)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		if(&nodePatPairs->nodePairList1)
			ListDestroy(&nodePatPairs->nodePairList1);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

/* Delete the nodepatpairs list*/
FSTATUS DeleteNodePatPairs(NodePairList_t *nodePatPairs)
{
	if(nodePatPairs) {
		//delete the left side of the list.
		ListDestroy(&nodePatPairs->nodePairList1);
		//delete the right side of the list.
		ListDestroy(&nodePatPairs->nodePairList2);
	}
	return FSUCCESS;
}

/* Parse the node pairs/nodes file */
static FSTATUS ParseNodePairPatFilePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, uint8 pair_flag, char **pp)
{
	char *p, *pEol;
	char patternLine[STL_NODE_DESCRIPTION_ARRAY_SIZE*2+NODE_PAIR_DELIMITER_SIZE+1];
	char pattern[STL_NODE_DESCRIPTION_ARRAY_SIZE+1];
	FSTATUS status;
	char nodePatFileName[FILENAME] = {0};
	struct stat fileStat;
	FILE *fp;
	NodePairList_t nodePatPairs;

	ASSERT(PointIsInInit(pPoint));

	if (0 == pair_flag)
		return FINVALID_OPERATION;

	if (NULL == arg) {
		fprintf(stderr, "%s: Node pattern Filename missing\n", g_Top_cmdname);
		return FINVALID_PARAMETER;
	}

	if (strlen(arg) > FILENAME - 1) {
		fprintf(stderr, "%s: Node pattern Filename too long: %.*s\n",
						g_Top_cmdname, (int)strlen(arg), arg);
		return FINVALID_PARAMETER;
	}

	if ((PAIR_FLAG_NODE != pair_flag) && (PAIR_FLAG_NONE != pair_flag) ) {
		fprintf(stderr, "%s: Pair flag argument is invalid: %d\n",
						g_Top_cmdname, pair_flag);
		return FINVALID_PARAMETER;
	}
	//Get file name
	StringCopy(nodePatFileName, arg, (int)sizeof(nodePatFileName));
	nodePatFileName[strlen(arg)] = '\0';

	//There are no further focus to evaluate for node pair list
	*pp = arg + strlen(arg);

	// Check if file is present
	if (stat(nodePatFileName, &fileStat) < 0) {
		fprintf(stderr, "Error taking stat of file {%s}: %s\n",
			nodePatFileName, strerror(errno));
			return FINVALID_PARAMETER;
	}

	//Open file
	if ((fp = fopen(nodePatFileName, "r")) == NULL) {
		fprintf(stderr, "Error opening file %s for input: %s\n", nodePatFileName, strerror(errno));
		return FINVALID_PARAMETER;
	}

	memset(patternLine, 0, sizeof(patternLine));

	//Get one line at a time
	while(fgets(patternLine, sizeof(patternLine), fp) != NULL){
		//remove newline
		if ((pEol = strrchr(patternLine, '\n')) != NULL) {
			*pEol= '\0';
		}
		//When node pairs are given
		if (PAIR_FLAG_NODE == pair_flag ) {
			p = strchr(patternLine, ':');
			if (p) {
				memset(pattern, 0, sizeof(pattern));
				//just log the error meesage
				if (p - patternLine > STL_NODE_DESCRIPTION_ARRAY_SIZE) {
					fprintf(stderr, "%s: Left side node name Not Found (too long): %.*s\n",
						g_Top_cmdname, (int)(p - patternLine), patternLine);
				}
				// copy the Left side of node pair. +1 for '\0'
				StringCopy(pattern, patternLine, (p - patternLine + 1));
				pattern[(p - patternLine) + 1] = '\0';
				status = InitNodePatPairs(&nodePatPairs);
				if(FSUCCESS != status){
					fprintf(stderr, "%s: Insufficient Memory\n",g_Top_cmdname );
					goto fail;
				}
				// Find all the nodes that match the pattern for the left side
				status = FindNodePatPairs(fabricp, pattern, &nodePatPairs, find_flag, LSIDE_PAIR);
				// When there is invalid entry in the Left side of each line, the entire line is skipped
				if ((FSUCCESS != status)){
					DeleteNodePatPairs(&nodePatPairs);
					continue;
				}
				memset(pattern, 0, sizeof(pattern));
				//When there is invalid entry in the right side don't mark error as corresponding left side entry could be a Switch
				if (pEol - p <= STL_NODE_DESCRIPTION_ARRAY_SIZE) {
					// copy the Right side of node pair. +1 is for ':'
					StringCopy(pattern, patternLine + (p - patternLine + 1), (pEol - p + 1));
					// Find all the nodes that match the pattern for the right side
					status = FindNodePatPairs(fabricp, pattern, &nodePatPairs, find_flag, RSIDE_PAIR);
					// Only when there is insufficient memory error mark as error and return
					if ((FSUCCESS != status) && (FINVALID_OPERATION != status) &&
						(FNOT_FOUND != status)){
						DeleteNodePatPairs(&nodePatPairs);
						goto fail;
					}
				} else {
					//just log the error meesage
					fprintf(stderr, "%s: Right side node name (too long): %.*s\n",
						g_Top_cmdname, (int)(pEol - p), p);
				}
			}else {
				//just log error message
				fprintf(stderr, "%s: Node pair is missing: %.*s\n",
					g_Top_cmdname, (int)sizeof(patternLine), patternLine);
			}
			// The complete node pair List N*M  is populated for a single line in file
			status = PointPopulateNodePairList(pPoint, &nodePatPairs);
			if (FSUCCESS != status) {
				//Log error and return if it fails to fom N*M list
				fprintf(stderr, "%s: Error creating node pairs\n", g_Top_cmdname);
				DeleteNodePatPairs(&nodePatPairs);
				goto fail;
			}
			// Now the line is parsed and nadepat pairs are created, so free nodePatPairs
			DeleteNodePatPairs(&nodePatPairs);
		//When only one node is given
		} else {
			if (strlen(patternLine) <= STL_NODE_DESCRIPTION_ARRAY_SIZE) {
				// Only when there is insufficient memory error or invalid operation error mark as error and return
				// else proceed with next node in list
				status = FindNodeNamePatPointUncompress(fabricp, patternLine, pPoint, find_flag);
				if ((FSUCCESS != status) && (FNOT_FOUND != status))
					goto fail;
			} else {
				//just log the error message and parse next line
				fprintf(stderr, "%s: Node name (too long): %.*s\n",
					g_Top_cmdname, (int)sizeof(patternLine), patternLine);
			}
		}
		memset(patternLine, 0, sizeof(patternLine));
	}
	PointCompress(pPoint);
	fclose(fp);
	return FSUCCESS;

fail:
	fclose(fp);
	return status;
}

static FSTATUS ParseNodeNamePatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[STL_NODE_DESCRIPTION_ARRAY_SIZE*5+1];
	char *param;
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Node name pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Node name pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindNodeNamePatPoint(fabricp, arg, pPoint, find_flag);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag,
								PointPortElinkCompareNodeNamePat, arg, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseNodeDetPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[NODE_DETAILS_STRLEN*5+1];
	char *param;
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Node Details pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Node Details pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ENODE))
		&& ! QListCount(&fabricp->ExpectedFIs)
		&& ! QListCount(&fabricp->ExpectedSWs))
		fprintf(stderr, "%s: Warning: No Node Details supplied via topology_input\n", g_Top_cmdname);
	status = FindNodeDetailsPatPoint(fabricp, arg, pPoint, find_flag);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
	} else {
		return FSUCCESS;
	}
}
#endif /* __VXWORKS__ */

static boolean PointPortElinkCompareNodeType(ExpectedLink *elinkp, void *nodecmp, uint8 portnum)
{
	NODE_TYPE type = *(NODE_TYPE*)nodecmp;

	return ((elinkp->portselp1 && elinkp->portselp1->NodeType == type
				 && elinkp->portselp1->gotPortNum
				 && elinkp->portselp1->PortNum == portnum)
			|| (elinkp->portselp2 && elinkp->portselp2->NodeType == type
				 && elinkp->portselp2->gotPortNum
				 && elinkp->portselp2->PortNum == portnum));
}

static FSTATUS ParseNodeTypePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char *param;
	FSTATUS status;
	NODE_TYPE type;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Node type format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg != 2) {
			fprintf(stderr, "%s: Invalid Node type: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		*pp = p;
	} else {
		if (strlen(arg) != 2) {
			fprintf(stderr, "%s: Invalid Node type: %s\n", g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		*pp = arg + strlen(arg);
	}
	if (strncasecmp(arg, "FI", 2) == 0)
		type = STL_NODE_FI;
	else if (strncasecmp(arg, "SW", 2) == 0)
		type = STL_NODE_SW;
	else {
		fprintf(stderr, "%s: Invalid Node type: %.*s\n", g_Top_cmdname, 2, arg);
		*pp = arg; /* back up to start of type for syntax error */
		return FINVALID_PARAMETER;
	}

	status = FindNodeTypePoint(fabricp, type, pPoint, find_flag);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag,
								PointPortElinkCompareNodeType, &type, pp);
	} else {
		return FSUCCESS;
	}
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
static FSTATUS ParseIocNamePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Name[IOC_IDSTRING_SIZE+1];
	char *param;
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid IOC name format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > IOC_IDSTRING_SIZE) {
			fprintf(stderr, "%s: IOC name Not Found (too long): %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Name, arg, sizeof(Name));
		Name[p-arg] = '\0';
		*pp = p;
		arg = Name;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindIocNamePoint(fabricp, arg, pPoint, find_flag);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseIocNamePatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[IOC_IDSTRING_SIZE*5+1];
	char *param;
	FSTATUS status;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid IOC name pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: IOC name pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindIocNamePatPoint(fabricp, arg, pPoint, find_flag);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseIocTypePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Type[5+1]; //"OTHER" is longest valid type name  
	char *param;
	FSTATUS status;
	int len;
	IocType type;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid IOC type format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Type)-1) {
			fprintf(stderr, "%s: Invalid IOC type: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		len = (int)(p-arg);
		StringCopy(Type, arg, sizeof(Type));
		Type[len] = '\0';
		*pp = p;
		arg = Type;
	} else {
		len = strlen(arg);
		*pp = arg + len;
	}
	if (strncasecmp(arg, "SRP", len) == 0) 
		type = IOC_TYPE_SRP;
	else if (strncasecmp(arg, "OTHER", len) == 0){
		type = IOC_TYPE_OTHER;
	}
	else {
		fprintf(stderr, "%s: Invalid IOC type: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindIocTypePoint(fabricp, type, pPoint, find_flag);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, find_flag, NULL, NULL, pp);
	} else {
		return FSUCCESS;
	}
}
#endif

static FSTATUS ParseRatePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Rate[8];	// 37_5g is largest valid rate name, but lets use a couple more chars for future expansion
	FSTATUS status;
	int len;
	uint32 rate;
	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Rate format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Rate)-1) {
			fprintf(stderr, "%s: Invalid Rate: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		len = (int)(p-arg);
		StringCopy(Rate, arg, sizeof(Rate));
		Rate[len] = '\0';
		*pp = p;
		arg = Rate;
	} else {
		len = strlen(arg);
		*pp = arg + len;
	}
	if (strncmp(arg, "12.5g", len) == 0)
		rate = IB_STATIC_RATE_14G;
	else if (strncmp(arg, "25g", len) == 0)
		rate = IB_STATIC_RATE_25G;
	else if (strncmp(arg, "37.5", len) == 0)
		rate = IB_STATIC_RATE_40G;
	else if (strncmp(arg, "50g", len) == 0)
		rate = IB_STATIC_RATE_56G;
	else if (strncmp(arg, "75g", len) == 0)
		rate = IB_STATIC_RATE_80G;
	else if (strncmp(arg, "100g", len) == 0)
		rate = IB_STATIC_RATE_100G;
	else if (strncmp(arg, "150g", len) == 0)
		rate = IB_STATIC_RATE_168G;
	else if (strncmp(arg, "200g", len) == 0)
		rate = IB_STATIC_RATE_200G;
	else {
		fprintf(stderr, "%s: Invalid Rate: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindRatePoint(fabricp, rate, pPoint, find_flag);
	return status;
}

static FSTATUS ParseLedPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	FSTATUS status;
	char *p;
	char LedState[3+1];	// "off" is largest valid state name
	int len;
	boolean ledon;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Led State format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(LedState)-1) {
			fprintf(stderr, "%s: Invalid Led State: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		len = (int)(p-arg);
		StringCopy(LedState, arg, sizeof(LedState));
		LedState[len] = '\0';
		*pp = p;
		arg = LedState;
	} else {
		len = strlen(arg);
		*pp = arg + len;
	}

	if (strncasecmp(arg, "off", len) == 0)
		ledon = FALSE;
	else if (strncasecmp(arg, "on", len) == 0)
		ledon = TRUE;
	else {
		fprintf(stderr, "%s: Invalid Led State: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindLedStatePoint(fabricp, ledon, pPoint, find_flag);
	return status;
}


static FSTATUS ParsePortStatePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char State[6+1];	// active is largest valid state name
	FSTATUS status;
	int len;
	IB_PORT_STATE state;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Port State format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(State)-1) {
			fprintf(stderr, "%s: Invalid Port State: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		len = (int)(p-arg);
		StringCopy(State, arg, sizeof(State));
		State[len] = '\0';
		*pp = p;
		arg = State;
	} else {
		len = strlen(arg);
		*pp = arg + len;
	}
	if (strncasecmp(arg, "down", len) == 0)
		state = IB_PORT_DOWN;
	else if (strncasecmp(arg, "init", len) == 0)
		state = IB_PORT_INIT;
	else if (strncasecmp(arg, "armed", len) == 0)
		state = IB_PORT_ARMED;
	else if (strncasecmp(arg, "active", len) == 0)
		state = IB_PORT_ACTIVE;
	else if (strncasecmp(arg, "notactive", len) == 0)
		state = PORT_STATE_SEARCH_NOTACTIVE;
	else if (strncasecmp(arg, "initarmed", len) == 0)
		state = PORT_STATE_SEARCH_INITARMED;
	else {
		fprintf(stderr, "%s: Invalid Port State: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindPortStatePoint(fabricp, state, pPoint, find_flag);
	return status;
}

static FSTATUS ParsePortPhysStatePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char PhysState[8+1];	// recovery is largest valid state name
	FSTATUS status;
	int len;
	IB_PORT_PHYS_STATE physstate;

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Port Phys State format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(PhysState)-1) {
			fprintf(stderr, "%s: Invalid Port Phys State: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		len = (int)(p-arg);
		StringCopy(PhysState, arg, sizeof(PhysState));
		PhysState[len] = '\0';
		*pp = p;
		arg = PhysState;
	} else {
		len = strlen(arg);
		*pp = arg + len;
	}
	// SLEEP is N/A to OPA
	if (strncasecmp(arg, "polling", len) == 0)
		physstate = IB_PORT_PHYS_POLLING;
	else if (strncasecmp(arg, "disabled", len) == 0)
		physstate = IB_PORT_PHYS_DISABLED;
	else if (strncasecmp(arg, "training", len) == 0)
		physstate = IB_PORT_PHYS_TRAINING;
	else if (strncasecmp(arg, "linkup", len) == 0)
		physstate = IB_PORT_PHYS_LINKUP;
	else if (strncasecmp(arg, "recovery", len) == 0)
		physstate = IB_PORT_PHYS_LINK_ERROR_RECOVERY;
	else if (strncasecmp(arg, "offline", len) == 0)
		physstate = STL_PORT_PHYS_OFFLINE;
	else if (strncasecmp(arg, "test", len) == 0)
		physstate = STL_PORT_PHYS_TEST;
	else {
		fprintf(stderr, "%s: Invalid Port Phys State: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindPortPhysStatePoint(fabricp, physstate, pPoint, find_flag);
	return status;
}

static FSTATUS ParseMtuPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	uint16 mtu_int;
	IB_MTU mtu;

	ASSERT(! PointValid(pPoint));
	if (FSUCCESS != StringToUint16(&mtu_int, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid MTU format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! mtu_int) {
		fprintf(stderr, "%s: Invalid MTU: %u\n",
						g_Top_cmdname, (unsigned)mtu_int);
		return FINVALID_PARAMETER;
	}
	mtu = GetMtuFromBytes(mtu_int);

	return FindMtuPoint(fabricp, mtu, pPoint, find_flag);
}

static FSTATUS ParseCableLabelPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[CABLE_LABEL_STRLEN*5+1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Cable Label pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Cable Label pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK))
		&& ! (fabricp->flags & FF_CABLEDATA))
		fprintf(stderr, "%s: Warning: No Cable Data supplied via topology_input\n", g_Top_cmdname);
	return FindCableLabelPatPoint(fabricp, arg, pPoint, find_flag);
}

static FSTATUS ParseCableLenPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[CABLE_LENGTH_STRLEN*5+1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Cable Length pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Cable Length pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK))
		&& ! (fabricp->flags & FF_CABLEDATA))
		fprintf(stderr, "%s: Warning: No Cable Data supplied via topology_input\n", g_Top_cmdname);
	return FindCableLenPatPoint(fabricp, arg, pPoint, find_flag);
}

static FSTATUS ParseCableDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[CABLE_DETAILS_STRLEN*5+1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Cable Details pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Cable Details pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK))
		&& ! (fabricp->flags & FF_CABLEDATA))
		fprintf(stderr, "%s: Warning: No Cable Data supplied via topology_input\n", g_Top_cmdname);
	return FindCableDetailsPatPoint(fabricp, arg, pPoint, find_flag);
}

static FSTATUS ParseCabinfLenPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid CableInfo Cable Length pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: CableInfo Cable Length pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfLenPatPoint(fabricp, arg, pPoint, find_flag);

}	// End of ParseCabinfLenPatPoint()

static FSTATUS ParseCabinfVendNamePatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Vendor Name pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Vendor Name pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendNamePatPoint(fabricp, arg, pPoint, find_flag);

}	// End of ParseCabinfVendNamePatPoint()

static FSTATUS ParseCabinfVendPNPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Vendor PN pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Vendor PN pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendPNPatPoint(fabricp, arg, pPoint, find_flag);

}	// End of ParseCabinfVendPNPatPoint()

static FSTATUS ParseCabinfVendRevPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Vendor Rev pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Vendor Rev pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendRevPatPoint(fabricp, arg, pPoint, find_flag);

}	// End of ParseCabinfVendRevPatPoint()

static FSTATUS ParseCabinfVendSNPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Vendor SN pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Vendor SN pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendSNPatPoint(fabricp, arg, pPoint, find_flag);

}	// End of ParseCabinfVendSNPatPoint()


static FSTATUS ParseCabinfCableTypePoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char cabletype[STL_CIB_STD_MAX_STRING + 1];	// 'optical' is largest valid cable type name, but we use some more chars for future expansion
	int len;
	FSTATUS status;


	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Cable Type format: '%s'\n", g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(cabletype)-1) {
			fprintf(stderr, "%s: Invalid Cable Type: %.*s\n", g_Top_cmdname, (int)(p-arg), arg);
			fprintf(stderr, "%s: Available Cable Types are: optical, passive_copper, active_copper and unknown.\n",g_Top_cmdname);
			return FINVALID_PARAMETER;
		}
		StringCopy(cabletype, arg, sizeof(cabletype));
		cabletype[p-arg] = '\0';
		*pp = p;
		arg = cabletype;
	} else {
		*pp = arg +  strlen(arg);
	}

	len = strlen(arg);

	if (!((strncmp(arg, "unknown", len) == 0) || (strncmp(arg, "optical", len) == 0)
			|| (strncmp(arg, "active_copper", len) == 0) || (strncmp(arg, "passive_copper", len) == 0)))
		 {
		fprintf(stderr, "%s: Invalid Cable Type: %.*s\n", g_Top_cmdname, len, arg);
		fprintf(stderr, "%s: Available Cable Types are: optical, passive_copper, active_copper and unknown.\n",g_Top_cmdname);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindCabinfCableTypePoint(fabricp, arg, pPoint, find_flag);
	return status;

} // End of ParseCabinCableTypePoint()

static FSTATUS ParseLinkDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[LINK_DETAILS_STRLEN*5+1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Link Details pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Link Details pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK))
		&& ! QListCount(&fabricp->ExpectedLinks))
		fprintf(stderr, "%s: Warning: No Link Details supplied via topology_input\n", g_Top_cmdname);
	return FindLinkDetailsPatPoint(fabricp, arg, pPoint, find_flag);
}

static FSTATUS ParsePortDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[PORT_DETAILS_STRLEN*5+1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid Port Details pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: Port Details pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ELINK))
		&& ! QListCount(&fabricp->ExpectedLinks))
		fprintf(stderr, "%s: Warning: No Port Details supplied via topology_input\n", g_Top_cmdname);
	return FindPortDetailsPatPoint(fabricp, arg, pPoint, find_flag);
}

static FSTATUS ParseSmPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{

	*pp = arg;
	ASSERT(! PointValid(pPoint));
	if (0 == (find_flag & FIND_FLAG_FABRIC))
		return FINVALID_OPERATION;
	if (find_flag & FIND_FLAG_FABRIC) {
		pPoint->u.portp = FindMasterSm(fabricp);
		if (pPoint->u.portp) {
			pPoint->Type = POINT_TYPE_PORT;
		}
	}
	// N/A for FIND_FLAG_ENODE, FIND_FLAG_ESM and FIND_FLAG_ELINK
	// while FIND_FLAG_ESM may seem applicable, we can't identify a master SM
	// from what is in topology.xml
	if (! PointValid(pPoint)) {
		fprintf(stderr, "%s: Master SM Not Found\n", g_Top_cmdname);
		return FNOT_FOUND;
	}
	return FSUCCESS;
}

static FSTATUS ParseSmDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	char *p;
	char Pattern[SM_DETAILS_STRLEN*5+1];

	ASSERT(! PointValid(pPoint));
	p = strchr(arg, ':');
	if (p) {
		if (p == arg) {
			fprintf(stderr, "%s: Invalid SM Details pattern format: '%s'\n",
							g_Top_cmdname, arg);
			return FINVALID_PARAMETER;
		}
		if (p - arg > sizeof(Pattern)-1) {
			fprintf(stderr, "%s: SM Details pattern too long: %.*s\n",
							g_Top_cmdname, (int)(p-arg), arg);
			return FINVALID_PARAMETER;
		}
		StringCopy(Pattern, arg, sizeof(Pattern));
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (0 == (find_flag & (FIND_FLAG_FABRIC|FIND_FLAG_ESM))
		&& ! QListCount(&fabricp->ExpectedSMs))
		fprintf(stderr, "%s: Warning: No SM Details supplied via topology_input\n", g_Top_cmdname);
	return FindSmDetailsPatPoint(fabricp, arg, pPoint, find_flag);
}

static FSTATUS ParseLinkQualityPoint(FabricData_t *fabricp, char *arg, Point *pPoint, uint8 find_flag, char **pp)
{
	LinkQualityCompare comp;
	uint16 quality;
	char *Quality;
	
	ASSERT(! PointValid(pPoint));
	*pp = arg;
	if (NULL != (Quality = ComparePrefix(arg, "LE:"))) {
		comp = QUAL_LE; // below means get all qualities less than given quality
	} else if (NULL != (Quality = ComparePrefix(arg, "GE:"))) {
		comp = QUAL_GE; // above means get all qualities greater than the given quality
	} else if (NULL != (Quality = ComparePrefix(arg, ":"))) {
		comp = QUAL_EQ; // get exact quality
	} else {
		fprintf(stderr, "%s: Invalid Link Quality format: '%s'\n",
				g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}

	*pp = Quality;
	if (FSUCCESS != StringToUint16(&quality, Quality, pp, 0, TRUE)) {
		fprintf(stderr, "%s: Invalid Link Quality format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	
	if (quality > STL_LINKQUALITY_EXCELLENT) { // 5 (excellent) is the max value for link quality, anything greater is invalid
		fprintf(stderr, "%s: Link Quality Too Large: %d\n", g_Top_cmdname, quality);
		return FINVALID_PARAMETER;
	}

	return FindLinkQualityPoint(fabricp, quality, comp, pPoint, find_flag);
}

/* parse the arg string and find the mentioned Point
 * arg string formats:
 * 	gid:subnet:guid
 *	lid:lid
 *	lid:lid:node
 *	lid:lid:port:#
 *	lid:lid:veswport:#
 *	portguid:guid
 *	nodeguid:guid
 *	nodeguid:guid:port:#
 *	iocguid:guid
 *	iocguid:guid:port:#
 *	systemguid:guid
 *	systemguid:guid:port:#
 *	node:node name
 *	node:node name:port:#
 *	nodepat:node name pattern
 *	nodepat:node name pattern:port:#
 *	nodedetpat:node details pattern
 *	nodedetpat:node details pattern:port:#
 *	nodetype:node type
 *	nodetype:node type:port:#
 *	ioc:ioc name
 *	ioc:ioc name:port:#
 *	iocpat:ioc name pattern
 *	iocpat:ioc name pattern:port:#
 *	ioctype:ioc type
 *	ioctype:ioc type:port:#
 *	rate:rate string
 *	portstate:state string
 *	portphysstate:phys state string
 *	mtu:#
 *	cableinftype:cable type
 *	labelpat:cable label pattern
 *	led:on/off
 *	lengthpat:cable length pattern
 *	cabledetpat:cable details pattern
 *	cabinflenpat:cable info cable length pattern
 *	cabinfvendnamepat:cable info vendor name pattern
 *	cabinfvendpnpat:cable info vendor part number pattern
 *	cabinfvendrevpat:cable info vendor rev pattern
 *	cabinfvendsnpat:cable info vendor serial number pattern
 *	linkdetpat:cable details pattern
 *	portdetpat:port details pattern
 *	sm
 *  smdetpat:sm details pattern
 *  linkqual:link quality level
 *  linkqualLE:link quality level
 *  linkqualGE:link quality level
 */
FSTATUS ParsePoint(FabricData_t *fabricp, char* arg, Point* pPoint, uint8 find_flag, char **pp)
{
	char* param;
	FSTATUS status;

	*pp = arg;
	PointInit(pPoint);
	if (NULL != (param = ComparePrefix(arg, "gid:"))) {
		status = ParseGidPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "lid:"))) {
		status = ParseLidPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portguid:"))) {
		status = ParsePortGuidPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodeguid:"))) {
		status = ParseNodeGuidPoint(fabricp, param, pPoint, find_flag, pp);
#if !defined(VXWORKS) || defined(BUILD_DMC)
	} else if (NULL != (param = ComparePrefix(arg, "iocguid:"))) {
		status = ParseIocGuidPoint(fabricp, param, pPoint, find_flag, pp);
#endif
	} else if (NULL != (param = ComparePrefix(arg, "systemguid:"))) {
		status = ParseSystemGuidPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "node:"))) {
		status = ParseNodeNamePoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "led:"))) {
		status = ParseLedPoint(fabricp, param, pPoint, find_flag, pp);
#ifndef __VXWORKS__
	} else if (NULL != (param = ComparePrefix(arg, "nodepat:"))) {
		status = ParseNodeNamePatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodedetpat:"))) {
		status = ParseNodeDetPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodepatfile:"))) {
		status = ParseNodePairPatFilePoint(fabricp, param, pPoint, find_flag, PAIR_FLAG_NONE, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodepairpatfile:"))) {
		status = ParseNodePairPatFilePoint(fabricp, param, pPoint, find_flag, PAIR_FLAG_NODE, pp);
#endif
	} else if (NULL != (param = ComparePrefix(arg, "nodetype:"))) {
		status = ParseNodeTypePoint(fabricp, param, pPoint, find_flag, pp);
#if !defined(VXWORKS) || defined(BUILD_DMC)
	} else if (NULL != (param = ComparePrefix(arg, "ioc:"))) {
		status = ParseIocNamePoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "iocpat:"))) {
		status = ParseIocNamePatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "ioctype:"))) {
		status = ParseIocTypePoint(fabricp, param, pPoint, find_flag, pp);
#endif
	} else if (NULL != (param = ComparePrefix(arg, "rate:"))) {
		status = ParseRatePoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portstate:"))) {
		status = ParsePortStatePoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portphysstate:"))) {
		status = ParsePortPhysStatePoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "mtucap:"))) {
		status = ParseMtuPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "labelpat:"))) {
		status = ParseCableLabelPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "lengthpat:"))) {
		status = ParseCableLenPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabledetpat:"))) {
		status = ParseCableDetailsPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinflenpat:"))) {
		status = ParseCabinfLenPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendnamepat:"))) {
		status = ParseCabinfVendNamePatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendpnpat:"))) {
		status = ParseCabinfVendPNPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendrevpat:"))) {
		status = ParseCabinfVendRevPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendsnpat:"))) {
		status = ParseCabinfVendSNPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinftype:"))) {
		status = ParseCabinfCableTypePoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "linkdetpat:"))) {
		status = ParseLinkDetailsPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portdetpat:"))) {
		status = ParsePortDetailsPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "smdetpat:"))) {
		status = ParseSmDetailsPatPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "sm"))) {
		status = ParseSmPoint(fabricp, param, pPoint, find_flag, pp);
	} else if (NULL != (param = ComparePrefix(arg, "linkqual"))) {
		status = ParseLinkQualityPoint(fabricp, param, pPoint, find_flag, pp);
	} else {
		fprintf(stderr, "%s: Invalid format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (status == FINVALID_OPERATION) {
		fprintf(stderr, "%s: Format Not Allowed: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// FIPortIteratorHead
//
// Description:
//	Finds the first non-SW port for each type in the point.
//
// Inputs:
//	pFIPortIterator	- Pointer to FIPortIterator object
//	pFocus	- Pointer to user's compare function
//
// Outputs:
//	None
//
// Returns:
//	Pointer to port data.
//
///////////////////////////////////////////////////////////////////////////////
PortData *FIPortIteratorHead(FIPortIterator *pFIPortIterator, Point *pFocus)
{
	pFIPortIterator->pPoint = pFocus;

	switch(pFocus->Type)
	{
		case POINT_TYPE_PORT:
		{
			//return NULL if port belongs to a switch else return the port
			if( STL_NODE_SW !=  pFocus->u.portp->nodep->NodeInfo.NodeType) {
				//Flag to indicate the port is the only port and it is already returned
				pFIPortIterator->u.PortIter.lastPortFlag = TRUE;
				return pFocus->u.portp;
			}
			return NULL;
		}
		break;
		case POINT_TYPE_PORT_LIST:
		{
			LIST_ITERATOR i;
			//Go through the port list to find the first FI port. Skip all SW ports
			for (i = ListHead(&pFocus->u.portList);
				i != NULL; i = ListNext(&pFocus->u.portList, i)) {
				PortData *portp = (PortData *)ListObj(i);
				if(STL_NODE_SW == portp->nodep->NodeInfo.NodeType)
					continue;
				else {
					//Store where in the point the current iterator is.
					pFIPortIterator->u.PortListIter.currentPort = i;
					return portp;
				}
			}
			return NULL;
		}
		break;
		case POINT_TYPE_NODE:
		{
			NodeData *nodep = pFocus->u.nodep;
			// Get the first port only if it is a FI node. Return NULL for switch nodes.
			if(STL_NODE_SW != nodep->NodeInfo.NodeType) {
				cl_map_item_t * p = cl_qmap_head(&nodep->Ports);
				if(p != cl_qmap_end(&nodep->Ports)) {
					PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
					//Store where in the point the current iterator is.
					pFIPortIterator->u.NodeIter.pCurrentPort = p;
					pFIPortIterator->u.NodeIter.lastNodeFlag = TRUE;
					return portp;
				}
			}
			return NULL;
		}
		break;
		case POINT_TYPE_NODE_LIST:
		{
			LIST_ITERATOR i;
			//Iterate over the node list to get the first FI node.
			for (i = ListHead(&pFocus->u.nodeList);
				i != NULL; i = ListNext(&pFocus->u.nodeList, i)) {
				NodeData *nodep = (NodeData*)ListObj(i);

				//skip switches
				if(nodep->NodeInfo.NodeType == STL_NODE_SW)
					continue;
				else {
					cl_map_item_t *p = cl_qmap_head(&nodep->Ports);
					if(p != cl_qmap_end(&nodep->Ports)){
						PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
						//Store where in the point the current iterator is.
						pFIPortIterator->u.NodeListIter.currentNode = i;
						pFIPortIterator->u.NodeListIter.pCurrentPort = p;
						return portp;
					}
				}
			}
			return NULL;
		}
		break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
		case POINT_TYPE_IOC:
		{
			NodeData *nodep = pFocus->u.iocp->ioup->nodep;
			//Get the first port for a FI node.
			if(STL_NODE_SW != nodep->NodeInfo.NodeType) {
				cl_map_item_t *p = cl_qmap_head(&nodep->Ports);
				if(p != cl_qmap_end(&nodep->Ports))
				{
					PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
					//Store where in the point the current iterator is.
					pFIPortIterator->u.IocIter.pCurrentPort = p;
					pFIPortIterator->u.IocIter.lastIocFlag = TRUE;
					return portp;
				}
			}
			return NULL;
		}
		break;
		case POINT_TYPE_IOC_LIST:
		{
			LIST_ITERATOR i;
			//Iterate over the Ioc list to get the first FI node.
			for (i = ListHead(&pFocus->u.iocList);
				i != NULL; i = ListNext(&pFocus->u.iocList, i)) {
				IocData *iocp = (IocData*)ListObj(i);
				NodeData *nodep = iocp->ioup->nodep;
				//skip switches
				if(nodep->NodeInfo.NodeType == STL_NODE_SW)
					continue;
				else {
					cl_map_item_t *p = cl_qmap_head(&nodep->Ports);
					if (p != cl_qmap_end(&nodep->Ports))
					{
						PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
						//Store where in the point the current iterator is.
						pFIPortIterator->u.IocListIter.currentNode = i;
						pFIPortIterator->u.IocListIter.pCurrentPort = p;
						return portp;
					}
				}
			}
			return NULL;
		}
		break;
#endif
		case POINT_TYPE_SYSTEM:
		{
			cl_map_item_t *s;
			//Iterate over the nodes in the system to get the first FI node.
			for (s = cl_qmap_head(&pFocus->u.systemp->Nodes);
				s != cl_qmap_end(&pFocus->u.systemp->Nodes); s = cl_qmap_next(s)){
				NodeData *nodep = PARENT_STRUCT(s, NodeData, SystemNodesEntry);
				if(nodep->NodeInfo.NodeType == STL_NODE_SW)
					continue;
				else {
					cl_map_item_t *p = cl_qmap_head(&nodep->Ports);
					if (p != cl_qmap_end(&nodep->Ports))
					{
						PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
						//Store where in the point the current iterator is.
						pFIPortIterator->u.SystemIter.pCurrentNode = s;
						pFIPortIterator->u.SystemIter.pCurrentPort = p;
						pFIPortIterator->u.SystemIter.lastSystemFlag = TRUE;
						return portp;
					}
				}
			}
			return NULL;
		}
		break;
		case POINT_TYPE_NONE:
		case POINT_TYPE_NODE_PAIR_LIST:
		default:
			return NULL;
		break;
	}
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// FIPortIteratorNext
//
// Description:
//	Finds the next non-SW port for each type in the point
//
// Inputs:
//	pFIPortIterator	- Pointer to FIPortIterator object
//
// Outputs:
//	None
//
// Returns:
//	Pointer to port data.
//
///////////////////////////////////////////////////////////////////////////////
PortData *FIPortIteratorNext(FIPortIterator *pFIPortIterator)
{
	Point *pFocus = pFIPortIterator->pPoint;

	switch(pFocus->Type)
	{
		case POINT_TYPE_PORT:
		{
			//Check if port is iterated over
			if(pFIPortIterator->u.PortIter.lastPortFlag)
				pFIPortIterator->u.PortIter.lastPortFlag = FALSE;
			return NULL;
		}
		break;
		case POINT_TYPE_PORT_LIST:
		{
			LIST_ITERATOR i;
			//Get the next FI port using the stored position of the iterator in the point
			for (i = ListNext(&pFocus->u.portList, pFIPortIterator->u.PortListIter.currentPort);
				i != NULL; i = ListNext(&pFocus->u.portList,i)) {
				PortData *portp = (PortData *)ListObj(i);
				if(STL_NODE_SW == portp->nodep->NodeInfo.NodeType)
					continue;
				else {
					//Store where in the point the current iterator is.
					pFIPortIterator->u.PortListIter.currentPort = i;
					return portp;
				}
			}
			return NULL;
		}
		break;
		case POINT_TYPE_NODE:
		{
			if (pFIPortIterator->u.NodeIter.lastNodeFlag){
				//Get the next port using the stored position of the iterator in the point. It is an FI port.
				cl_map_item_t *p = cl_qmap_next(pFIPortIterator->u.NodeIter.pCurrentPort);
				if(p != cl_qmap_end(&pFocus->u.nodep->Ports)){
					PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
					//Store where in the point the current iterator is.
					pFIPortIterator->u.NodeIter.pCurrentPort = p;
					return portp;
				} else
					pFIPortIterator->u.NodeIter.lastNodeFlag = FALSE;
			}
			return NULL;
		}
		break;
		case POINT_TYPE_NODE_LIST:
		{
			LIST_ITERATOR i;
			//Get the next port of the node using the stored position of the iterator in the point. It is an FI port.
			NodeData *nodep = (NodeData*)ListObj(pFIPortIterator->u.NodeListIter.currentNode);
			cl_map_item_t *p = cl_qmap_next(pFIPortIterator->u.NodeListIter.pCurrentPort);
			if(p != cl_qmap_end(&nodep->Ports)){
				PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
				//Store where in the point the current iterator is.
				pFIPortIterator->u.NodeIter.pCurrentPort = p;
				return portp;
			} else {
				//Iterate over the next node of the list
				for (i = ListNext(&pFocus->u.nodeList, pFIPortIterator->u.NodeListIter.currentNode);
					i != NULL; i = ListNext(&pFocus->u.nodeList, i)) {
					nodep = (NodeData*)ListObj(i);
					//skip switches
					if(nodep->NodeInfo.NodeType == STL_NODE_SW)
						continue;
					else {
						p = cl_qmap_head(&nodep->Ports);
						if(p != cl_qmap_end(&nodep->Ports)){
							PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
							//Store where in the point the current iterator is.
							pFIPortIterator->u.NodeListIter.currentNode = i;
							pFIPortIterator->u.NodeListIter.pCurrentPort = p;
							return portp;
						}
					}
				}
			}
			return NULL;
		}
		break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
		case POINT_TYPE_IOC:
		{
			if (pFIPortIterator->u.IocIter.lastIocFlag){
				//Get the next port using the stored position of the iterator in the point. It is an FI port.
				cl_map_item_t *p = cl_qmap_next(pFIPortIterator->u.IocIter.pCurrentPort);
				NodeData *nodep = pFocus->u.iocp->ioup->nodep;
				if(p != cl_qmap_end(&nodep->Ports)){
					PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
					//Store where in the point the current iterator is.
					pFIPortIterator->u.IocIter.pCurrentPort = p;
					return portp;
				} else
					pFIPortIterator->u.IocIter.lastIocFlag = FALSE;
			}
			return NULL;
		}
		break;
		case POINT_TYPE_IOC_LIST:
		{
			LIST_ITERATOR i;
			//Get the next port of the node using the stored position of the iterator in the point. It is an FI port.
			IocData *iocp = (IocData*)ListObj(pFIPortIterator->u.IocListIter.currentNode);
			NodeData *nodep = iocp->ioup->nodep;
			cl_map_item_t *p = cl_qmap_next(pFIPortIterator->u.IocListIter.pCurrentPort);
			if(p != cl_qmap_end(&nodep->Ports)){
				PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
				//Store where in the point the current iterator is.
				pFIPortIterator->u.IocListIter.pCurrentPort = p;
				return portp;
			} else {
				//Iterate over the next Ioc of the list
				for (i = ListNext(&pFocus->u.iocList, pFIPortIterator->u.IocListIter.currentNode);
					i != NULL; i = ListNext(&pFocus->u.iocList, i)) {
					iocp = (IocData*)ListObj(i);
					nodep = iocp->ioup->nodep;
					//skip switches
					if(nodep->NodeInfo.NodeType == STL_NODE_SW)
						continue;
					else {
						p = cl_qmap_head(&nodep->Ports);
						if(p != cl_qmap_end(&nodep->Ports)){
							PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
							//Store where in the point the current iterator is.
							pFIPortIterator->u.IocListIter.currentNode = i;
							pFIPortIterator->u.IocListIter.pCurrentPort = p;
							return portp;
						}
					}
				}
			}
			return NULL;
		}
		break;
#endif
		case POINT_TYPE_SYSTEM:
		{
			cl_map_item_t *s;
			if (pFIPortIterator->u.SystemIter.lastSystemFlag){
				//Get the next port using the stored position of the iterator in the point. It is an FI port.
				NodeData *nodep = PARENT_STRUCT(pFIPortIterator->u.SystemIter.pCurrentNode, NodeData, SystemNodesEntry);
				cl_map_item_t *p = cl_qmap_next(pFIPortIterator->u.SystemIter.pCurrentPort);
				if(p != cl_qmap_end(&nodep->Ports)){
					PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
					//Store where in the point the current iterator is.
					pFIPortIterator->u.SystemIter.pCurrentPort = p;
					return portp;
				} else {
					//Iterate over the next node of the list
					for (s = cl_qmap_next(pFIPortIterator->u.SystemIter.pCurrentNode);
						s != cl_qmap_end(&pFocus->u.systemp->Nodes); s = cl_qmap_next(s)) {
						nodep = PARENT_STRUCT(s, NodeData, SystemNodesEntry);
						//skip switches
						if(nodep->NodeInfo.NodeType == STL_NODE_SW)
							continue;
						else {
							p = cl_qmap_head(&nodep->Ports);
							if(p != cl_qmap_end(&nodep->Ports)){
								PortData *portp = PARENT_STRUCT(p, PortData, NodePortsEntry);
								//Store where in the point the current iterator is.
								pFIPortIterator->u.SystemIter.pCurrentNode = s;
								pFIPortIterator->u.SystemIter.pCurrentPort = p;
								return portp;
							}
						}
					}
					pFIPortIterator->u.SystemIter.lastSystemFlag = FALSE;
				}
			}
			return NULL;
		}
		break;
		case POINT_TYPE_NONE:
		case POINT_TYPE_NODE_PAIR_LIST:
		default:
			return NULL;
		break;
	}
	return NULL;
}
