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

#include "topology.h"
#include "topology_internal.h"

#ifndef __VXWORKS__
#include <strings.h>
#else
#include "icsBspUtil.h"
#endif

// Functions to Parse Focus arguments and build POINTs for matches
static FSTATUS ParsePointPort(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp);

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

static FSTATUS ParseGidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	IB_GID gid;
	
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
	/* TBD - cleanup use of global */
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
	pPoint->u.portp = FindPortGuid(fabricp, gid.Type.Global.InterfaceID);
	if (! pPoint->u.portp) {
		fprintf(stderr, "%s: GID Not Found: 0x%016"PRIx64":0x%016"PRIx64"\n",
					g_Top_cmdname, 
					gid.Type.Global.SubnetPrefix, gid.Type.Global.InterfaceID);
		return FNOT_FOUND;
	}
	if (pPoint->u.portp->PortInfo.SubnetPrefix
		!= gid.Type.Global.SubnetPrefix) {
		fprintf(stderr, "%s: GID Not Found: 0x%016"PRIx64":0x%016"PRIx64"\n",
					g_Top_cmdname, 
					gid.Type.Global.SubnetPrefix, gid.Type.Global.InterfaceID);
		fprintf(stderr, "%s: Subnet Prefix: 0x%016"PRIx64" does not match selected port: 0x%016"PRIx64"\n",
					g_Top_cmdname, gid.Type.Global.SubnetPrefix, 
					pPoint->u.portp->PortInfo.SubnetPrefix);
		return FNOT_FOUND;
	}
	pPoint->Type = POINT_TYPE_PORT;
	return FSUCCESS;
}

static FSTATUS ParseLidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	IB_LID lid;
	PortData *portp;
	char *param;
	
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
	if (FSUCCESS != StringToUint16(&lid, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid LID format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	if (! lid) {
		fprintf(stderr, "%s: Invalid LID: 0x%x\n", g_Top_cmdname, lid);
		return FINVALID_PARAMETER;
	}

	portp = FindLid(fabricp, lid);
	if (! portp) {
		fprintf(stderr, "%s: LID Not Found: 0x%x\n", g_Top_cmdname, lid);
		return FNOT_FOUND;
	}
	if (NULL != (param = ComparePrefix(*pp, ":node"))) {
		*pp = param;
		pPoint->u.nodep = portp->nodep;
		pPoint->Type = POINT_TYPE_NODE;
	} else if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		pPoint->u.nodep = portp->nodep;
		pPoint->Type = POINT_TYPE_NODE;
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		pPoint->u.portp = portp;
		pPoint->Type = POINT_TYPE_PORT;
	}
	return FSUCCESS;
}

static FSTATUS ParsePortGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	EUI64 guid;
	
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
	pPoint->u.portp = FindPortGuid(fabricp, guid);
	if (! pPoint->u.portp) {
		fprintf(stderr, "%s: Port GUID Not Found: 0x%016"PRIx64"\n",
					   	g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
	pPoint->Type = POINT_TYPE_PORT;
	return FSUCCESS;
}

/* Parse a :port:# suffix, this will limit the Point to the list of ports
 * with the given number
 */
static FSTATUS ParsePointPort(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	uint8 portnum;
	FSTATUS status;
	Point newPoint;

	ASSERT(pPoint->Type != POINT_TYPE_NONE);
	PointInit(&newPoint);
	if (FSUCCESS != StringToUint8(&portnum, arg, pp, 0, TRUE))  {
		fprintf(stderr, "%s: Invalid Port Number format: '%s'\n",
					   	g_Top_cmdname, arg);
		status = FINVALID_PARAMETER;
		goto fail;
	}
	switch (pPoint->Type) {
	case POINT_TYPE_PORT:
		ASSERT(0);	// should not be called for this type of point
		status = FINVALID_PARAMETER;
		goto fail;
	case POINT_TYPE_PORT_LIST:
		ASSERT(0);	// should not be called for this type of point
		status = FINVALID_PARAMETER;
		goto fail;
	case POINT_TYPE_NODE:
		{
		NodeData *nodep = pPoint->u.nodep;

		pPoint->u.portp = FindNodePort(nodep, portnum);
		if (! pPoint->u.portp) {
			fprintf(stderr, "%s: Node %.*s GUID 0x%016"PRIx64" Port %u Not Found\n",
				g_Top_cmdname, STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)nodep->NodeDesc.NodeString,
				nodep->NodeInfo.NodeGUID, portnum);
			status = FNOT_FOUND;
			goto fail;
		}
		pPoint->Type = POINT_TYPE_PORT;
		return FSUCCESS;
		}
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
		if (newPoint.Type == POINT_TYPE_NONE) {
			fprintf(stderr, "%s: Port %u Not Found\n", g_Top_cmdname, portnum);
			status = FNOT_FOUND;
			goto fail;
		} else {
			PointCompress(&newPoint);
			status = PointCopy(pPoint, &newPoint);
			PointDestroy(&newPoint);
			return status;
		}
		}
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		{
		IocData *iocp = pPoint->u.iocp;

		pPoint->u.portp = FindNodePort(iocp->ioup->nodep, portnum);
		if (! pPoint->u.portp) {
			fprintf(stderr, "%s: IOC %.*s GUID 0x%016"PRIx64" Port %u Not Found\n",
				g_Top_cmdname, IOC_IDSTRING_SIZE,
				(char*)iocp->IocProfile.IDString,
				iocp->IocProfile.IocGUID, portnum);
			status = FNOT_FOUND;
			goto fail;
		}
		pPoint->Type = POINT_TYPE_PORT;
		return FSUCCESS;
		}
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
		if (newPoint.Type == POINT_TYPE_NONE) {
			fprintf(stderr, "%s: Port %u Not Found\n", g_Top_cmdname, portnum);
			status = FNOT_FOUND;
			goto fail;
		} else {
			PointCompress(&newPoint);
			status = PointCopy(pPoint, &newPoint);
			PointDestroy(&newPoint);
			return status;
		}
		}
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
		if (newPoint.Type == POINT_TYPE_NONE) {
			fprintf(stderr, "%s: Port %u Not Found\n", g_Top_cmdname, portnum);
			status = FNOT_FOUND;
			goto fail;
		} else {
			PointCompress(&newPoint);
			status = PointCopy(pPoint, &newPoint);
			PointDestroy(&newPoint);
			return status;
		}
		}
	default:
		return FSUCCESS;
	}
fail:
	PointDestroy(&newPoint);
	PointDestroy(pPoint);
	return status;
}

static FSTATUS ParseNodeGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	NodeData *nodep;
	char *param;
	EUI64 guid;
	
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
	nodep = FindNodeGuid(fabricp, guid);
	if (! nodep) {
		fprintf(stderr, "%s: Node GUID Not Found: 0x%016"PRIx64"\n",
					   	g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
	pPoint->u.nodep = nodep;
	pPoint->Type = POINT_TYPE_NODE;
	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
static FSTATUS ParseIocGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	IocData *iocp;
	char *param;
	EUI64 guid;
	
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
	iocp = FindIocGuid(fabricp, guid);
	if (! iocp) {
		fprintf(stderr, "%s: IOC GUID Not Found: 0x%016"PRIx64"\n",
					   	g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
	pPoint->u.iocp = iocp;
	pPoint->Type = POINT_TYPE_IOC;
	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}
#endif

static FSTATUS ParseSystemGuidPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	EUI64 guid;
	char *param;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
	pPoint->u.systemp = FindSystemGuid(fabricp, guid);
	if (! pPoint->u.systemp) {
		fprintf(stderr, "%s: System Image GUID Not Found: 0x%016"PRIx64"\n",
					   	g_Top_cmdname, guid);
		return FNOT_FOUND;
	}
	pPoint->Type = POINT_TYPE_SYSTEM;
	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseNodeNamePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Name[STL_NODE_DESCRIPTION_ARRAY_SIZE+1];
	char *param;
	FSTATUS status;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Name, arg, p-arg);
		Name[p-arg] = '\0';
		*pp = p;
		arg = Name;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindNodeName(fabricp, arg, pPoint, 0);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseNodeNamePatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[STL_NODE_DESCRIPTION_ARRAY_SIZE*5+1];
	char *param;
	FSTATUS status;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindNodeNamePat(fabricp, arg, pPoint);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseNodeDetPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[NODE_DETAILS_STRLEN*5+1];
	char *param;
	FSTATUS status;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! QListCount(&fabricp->ExpectedFIs) && ! QListCount(&fabricp->ExpectedSWs))
		fprintf(stderr, "%s: Warning: No Node Details supplied via topology_input\n", g_Top_cmdname);
	status = FindNodeDetailsPat(fabricp, arg, pPoint);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseNodeTypePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char *param;
	FSTATUS status;
	NODE_TYPE type;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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

	status = FindNodeType(fabricp, type, pPoint);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
static FSTATUS ParseIocNamePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Name[IOC_IDSTRING_SIZE+1];
	char *param;
	FSTATUS status;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Name, arg, p-arg);
		Name[p-arg] = '\0';
		*pp = p;
		arg = Name;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindIocName(fabricp, arg, pPoint);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseIocNamePatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[IOC_IDSTRING_SIZE*5+1];
	char *param;
	FSTATUS status;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	status = FindIocNamePat(fabricp, arg, pPoint);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}

static FSTATUS ParseIocTypePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Type[5+1]; //"OTHER" is longest valid type name  
	char *param;
	FSTATUS status;
	int len;
	IocType type;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Type, arg, len);
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
		type = 	IOC_TYPE_OTHER;
	}
	else {
		fprintf(stderr, "%s: Invalid IOC type: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindIocType(fabricp, type, pPoint);
	if (FSUCCESS != status)
		return status;

	if (NULL != (param = ComparePrefix(*pp, ":port:"))) {
		return ParsePointPort(fabricp, param, pPoint, pp);
	} else {
		return FSUCCESS;
	}
}
#endif

static FSTATUS ParseRatePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Rate[8];	// 37_5g is largest valid rate name, but lets use a couple more chars for future expansion
	FSTATUS status;
	int len;
	uint32 rate;
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Rate, arg, len);
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
	else {
		fprintf(stderr, "%s: Invalid Rate: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindRate(fabricp, rate, pPoint);
	return status;
}

static FSTATUS ParseLedPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	FSTATUS status;
	int len;
	boolean ledon;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);

	len = strlen(arg);
	*pp = arg + len;

	if (strncasecmp(arg, "off", len) == 0)
		ledon = FALSE;
	else if (strncasecmp(arg, "on", len) == 0)
		ledon = TRUE;
	else {
		fprintf(stderr, "%s: Invalid Led State: %.*s\n", g_Top_cmdname, len, arg);
		*pp -= len;	/* back up for syntax error report */
		return FINVALID_PARAMETER;
	}

	status = FindLedState(fabricp, ledon, pPoint);
	return status;
}


static FSTATUS ParsePortStatePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char State[6+1];	// active is largest valid state name
	FSTATUS status;
	int len;
	IB_PORT_STATE state;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(State, arg, len);
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

	status = FindPortState(fabricp, state, pPoint);
	return status;
}

static FSTATUS ParsePortPhysStatePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char PhysState[8+1];	// recovery is largest valid state name
	FSTATUS status;
	int len;
	IB_PORT_PHYS_STATE physstate;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(PhysState, arg, len);
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

	status = FindPortPhysState(fabricp, physstate, pPoint);
	return status;
}

static FSTATUS ParseMtuPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	uint16 mtu_int;
	IB_MTU mtu;

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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

	// @TODO: Change functionality to not rely on the MTU of the first VL of the port (average them?)
	return FindMtu(fabricp, mtu, 0, pPoint);
}

static FSTATUS ParseCableLabelPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[CABLE_LABEL_STRLEN*5+1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! (fabricp->flags & FF_CABLEDATA))
		fprintf(stderr, "%s: Warning: No Cable Data supplied via topology_input\n", g_Top_cmdname);
	return FindCableLabelPat(fabricp, arg, pPoint);
}

static FSTATUS ParseCableLenPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[CABLE_LENGTH_STRLEN*5+1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! (fabricp->flags & FF_CABLEDATA))
		fprintf(stderr, "%s: Warning: No Cable Data supplied via topology_input\n", g_Top_cmdname);
	return FindCableLenPat(fabricp, arg, pPoint);
}

static FSTATUS ParseCableDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[CABLE_DETAILS_STRLEN*5+1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! (fabricp->flags & FF_CABLEDATA))
		fprintf(stderr, "%s: Warning: No Cable Data supplied via topology_input\n", g_Top_cmdname);
	return FindCableDetailsPat(fabricp, arg, pPoint);
}

static FSTATUS ParseCabinfLenPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfLenPat(fabricp, arg, pPoint);

}	// End of ParseCabinfLenPatPoint()

static FSTATUS ParseCabinfVendNamePatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendNamePat(fabricp, arg, pPoint);

}	// End of ParseCabinfVendNamePatPoint()

static FSTATUS ParseCabinfVendPNPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendPNPat(fabricp, arg, pPoint);

}	// End of ParseCabinfVendPNPatPoint()

static FSTATUS ParseCabinfVendRevPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendRevPat(fabricp, arg, pPoint);

}	// End of ParseCabinfVendRevPatPoint()

static FSTATUS ParseCabinfVendSNPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[STL_CIB_STD_MAX_STRING + 1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	return FindCabinfVendSNPat(fabricp, arg, pPoint);

}	// End of ParseCabinfVendSNPatPoint()


static FSTATUS ParseCabinfCableTypePoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char cabletype[STL_CIB_STD_MAX_STRING + 1];	// 'optical' is largest valid cable type name, but we use some more chars for future expansion
	int len;
	FSTATUS status;


	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(cabletype, arg, p-arg);
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

	status = FindCabinfCableType(fabricp, arg, pPoint);
	return status;

} // End of ParseCabinCableTypePoint()

static FSTATUS ParseLinkDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[LINK_DETAILS_STRLEN*5+1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! QListCount(&fabricp->ExpectedLinks))
		fprintf(stderr, "%s: Warning: No Link Details supplied via topology_input\n", g_Top_cmdname);
	return FindLinkDetailsPat(fabricp, arg, pPoint);
}

static FSTATUS ParsePortDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[PORT_DETAILS_STRLEN*5+1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! QListCount(&fabricp->ExpectedLinks))
		fprintf(stderr, "%s: Warning: No Port Details supplied via topology_input\n", g_Top_cmdname);
	return FindPortDetailsPat(fabricp, arg, pPoint);
}

static FSTATUS ParseSmPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{

	*pp = arg;
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
	pPoint->u.portp = FindMasterSm(fabricp);
	if (! pPoint->u.portp) {
		fprintf(stderr, "%s: Master SM Not Found\n", g_Top_cmdname);
		return FNOT_FOUND;
	}
	pPoint->Type = POINT_TYPE_PORT;
	return FSUCCESS;
}

static FSTATUS ParseSmDetailsPatPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	char *p;
	char Pattern[SM_DETAILS_STRLEN*5+1];

	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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
		strncpy(Pattern, arg, p-arg);
		Pattern[p-arg] = '\0';
		*pp = p;
		arg = Pattern;
	} else {
		*pp = arg + strlen(arg);
	}

	if (! QListCount(&fabricp->ExpectedSMs))
		fprintf(stderr, "%s: Warning: No SM Details supplied via topology_input\n", g_Top_cmdname);
	return FindSmDetailsPat(fabricp, arg, pPoint);
}

static FSTATUS ParseLinkQualityPoint(FabricData_t *fabricp, char *arg, Point *pPoint, char **pp)
{
	LinkQualityCompare comp;
	uint16 quality;
	char *Quality;
	
	ASSERT(pPoint->Type == POINT_TYPE_NONE);
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

	return FindLinkQuality(fabricp, quality, comp, pPoint);
}

/* parse the arg string and find the mentioned Point
 * arg string formats:
 * 	gid:subnet:guid
 *	lid:lid
 *	lid:lid:node
 *	lid:lid:port:#
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
FSTATUS ParsePoint(FabricData_t *fabricp, char* arg, Point* pPoint, char **pp)
{
	char* param;

	*pp = arg;
	PointInit(pPoint);
	if (NULL != (param = ComparePrefix(arg, "gid:"))) {
		return ParseGidPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "lid:"))) {
		return ParseLidPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portguid:"))) {
		return ParsePortGuidPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodeguid:"))) {
		return ParseNodeGuidPoint(fabricp, param, pPoint, pp);
#if !defined(VXWORKS) || defined(BUILD_DMC)
	} else if (NULL != (param = ComparePrefix(arg, "iocguid:"))) {
		return ParseIocGuidPoint(fabricp, param, pPoint, pp);
#endif
	} else if (NULL != (param = ComparePrefix(arg, "systemguid:"))) {
		return ParseSystemGuidPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "node:"))) {
		return ParseNodeNamePoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "led:"))) {
		return ParseLedPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodepat:"))) {
		return ParseNodeNamePatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodedetpat:"))) {
		return ParseNodeDetPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "nodetype:"))) {
		return ParseNodeTypePoint(fabricp, param, pPoint, pp);
#if !defined(VXWORKS) || defined(BUILD_DMC)
	} else if (NULL != (param = ComparePrefix(arg, "ioc:"))) {
		return ParseIocNamePoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "iocpat:"))) {
		return ParseIocNamePatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "ioctype:"))) {
		return ParseIocTypePoint(fabricp, param, pPoint, pp);
#endif
	} else if (NULL != (param = ComparePrefix(arg, "rate:"))) {
		return ParseRatePoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portstate:"))) {
		return ParsePortStatePoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portphysstate:"))) {
		return ParsePortPhysStatePoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "mtucap:"))) {
		return ParseMtuPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "labelpat:"))) {
		return ParseCableLabelPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "lengthpat:"))) {
		return ParseCableLenPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabledetpat:"))) {
		return ParseCableDetailsPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinflenpat:"))) {
		return ParseCabinfLenPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendnamepat:"))) {
		return ParseCabinfVendNamePatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendpnpat:"))) {
		return ParseCabinfVendPNPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendrevpat:"))) {
		return ParseCabinfVendRevPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinfvendsnpat:"))) {
		return ParseCabinfVendSNPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "cabinftype:"))) {
		return ParseCabinfCableTypePoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "linkdetpat:"))) {
		return ParseLinkDetailsPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "portdetpat:"))) {
		return ParsePortDetailsPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "smdetpat:"))) {
		return ParseSmDetailsPatPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "sm"))) {
		return ParseSmPoint(fabricp, param, pPoint, pp);
	} else if (NULL != (param = ComparePrefix(arg, "linkqual"))) {
		return ParseLinkQualityPoint(fabricp, param, pPoint, pp);
	} else {
		fprintf(stderr, "%s: Invalid format: '%s'\n", g_Top_cmdname, arg);
		return FINVALID_PARAMETER;
	}
	return FSUCCESS;
}
