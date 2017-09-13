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

// functions to build and compare POINTs

#define MIN_LIST_ITEMS 100	// minimum items for ListInit to allocate for

void PointInit(Point *point)
{
	point->Type = POINT_TYPE_NONE;
	point->EnodeType = POINT_ENODE_TYPE_NONE;
	point->EsmType = POINT_ESM_TYPE_NONE;
	point->ElinkType = POINT_ELINK_TYPE_NONE;
}

/* initialize a non-list point */
static void PointInitSimple(Point *point, PointType type, void *object)
{
	point->Type = type;

	switch (type) {
	case POINT_TYPE_NONE:
		ASSERT(object == NULL);
		return;
	case POINT_TYPE_PORT:
		ASSERT(object);
		point->u.portp = (PortData*)object;
		return;
	case POINT_TYPE_NODE:
		ASSERT(object);
		point->u.nodep = (NodeData*)object;
		return;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		ASSERT(object);
		point->u.iocp = (IocData*)object;
		return;
#endif
	case POINT_TYPE_SYSTEM:
		ASSERT(object);
		point->u.systemp = (SystemData*)object;
		return;
	case POINT_TYPE_PORT_LIST:
	case POINT_TYPE_NODE_LIST:
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC_LIST:
#endif
	default:
		ASSERT(0);
		point->Type = POINT_TYPE_NONE;
		return;
	}
}

/* initialize a non-list ExpectedNode point */
static void PointInitEnodeSimple(Point *point, PointEnodeType type, void *object)
{
	point->EnodeType = type;

	switch (type) {
	case POINT_ENODE_TYPE_NONE:
		ASSERT(object == NULL);
		return;
	case POINT_ENODE_TYPE_NODE:
		ASSERT(object);
		point->u2.enodep = (ExpectedNode*)object;
		return;
	case POINT_ENODE_TYPE_NODE_LIST:
	default:
		ASSERT(0);
		point->EnodeType = POINT_ENODE_TYPE_NONE;
		return;
	}
}

/* initialize a non-list ExpectedSM point */
static void PointInitEsmSimple(Point *point, PointEsmType type, void *object)
{
	point->EsmType = type;

	switch (type) {
	case POINT_ESM_TYPE_NONE:
		ASSERT(object == NULL);
		return;
	case POINT_ESM_TYPE_SM:
		ASSERT(object);
		point->u3.esmp = (ExpectedSM*)object;
		return;
	case POINT_ESM_TYPE_SM_LIST:
	default:
		ASSERT(0);
		point->EsmType = POINT_ESM_TYPE_NONE;
		return;
	}
}

/* initialize a non-list ExpectedLink point */
static void PointInitElinkSimple(Point *point, PointElinkType type, void *object)
{
	point->ElinkType = type;

	switch (type) {
	case POINT_ELINK_TYPE_NONE:
		ASSERT(object == NULL);
		return;
	case POINT_ELINK_TYPE_LINK:
		ASSERT(object);
		point->u4.elinkp = (ExpectedLink*)object;
		return;
	case POINT_ELINK_TYPE_LINK_LIST:
	default:
		ASSERT(0);
		point->ElinkType = POINT_ELINK_TYPE_NONE;
		return;
	}
}

/* initialize a list point - sets it as an empty list */
static FSTATUS PointInitList(Point *point, PointType type)
{
	DLIST *pList;

	point->Type = POINT_TYPE_NONE;
	switch (type) {
	case POINT_TYPE_PORT_LIST:
		pList = &point->u.portList;
		break;
	case POINT_TYPE_NODE_LIST:
		pList = &point->u.nodeList;
		break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC_LIST:
		pList = &point->u.iocList;
		break;
#endif
	default:
		ASSERT(0);
		return FINVALID_OPERATION;
	}
	ListInitState(pList);
	if (! ListInit(pList, MIN_LIST_ITEMS)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}
	point->Type = type;
	return FSUCCESS;
}

/* initialize an ExpectedNode list point - sets it as an empty list */
static FSTATUS PointInitEnodeList(Point *point, PointEnodeType type)
{
	DLIST *pList;

	point->EnodeType = POINT_ENODE_TYPE_NONE;
	switch (type) {
	case POINT_ENODE_TYPE_NODE_LIST:
		pList = &point->u2.enodeList;
		break;
	default:
		ASSERT(0);
		return FINVALID_OPERATION;
	}
	ListInitState(pList);
	if (! ListInit(pList, MIN_LIST_ITEMS)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}
	point->EnodeType = type;
	return FSUCCESS;
}

/* initialize an ExpectedSM list point - sets it as an empty list */
static FSTATUS PointInitEsmList(Point *point, PointEsmType type)
{
	DLIST *pList;

	point->EsmType = POINT_ESM_TYPE_NONE;
	switch (type) {
	case POINT_ESM_TYPE_SM_LIST:
		pList = &point->u3.esmList;
		break;
	default:
		ASSERT(0);
		return FINVALID_OPERATION;
	}
	ListInitState(pList);
	if (! ListInit(pList, MIN_LIST_ITEMS)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}
	point->EsmType = type;
	return FSUCCESS;
}

/* initialize an ExpectedLink list point - sets it as an empty list */
static FSTATUS PointInitElinkList(Point *point, PointElinkType type)
{
	DLIST *pList;

	point->ElinkType = POINT_ELINK_TYPE_NONE;
	switch (type) {
	case POINT_ELINK_TYPE_LINK_LIST:
		pList = &point->u4.elinkList;
		break;
	default:
		ASSERT(0);
		return FINVALID_OPERATION;
	}
	ListInitState(pList);
	if (! ListInit(pList, MIN_LIST_ITEMS)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		return FINSUFFICIENT_MEMORY;
	}
	point->ElinkType = type;
	return FSUCCESS;
}

void PointFabricDestroy(Point *point)
{
	switch (point->Type) {
	case POINT_TYPE_PORT_LIST:
		ListDestroy(&point->u.portList);
		break;
	case POINT_TYPE_NODE_LIST:
		ListDestroy(&point->u.nodeList);
		break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC_LIST:
		ListDestroy(&point->u.iocList);
		break;
#endif
	default:
		break;
	}
	point->Type = POINT_TYPE_NONE;
}

void PointEnodeDestroy(Point *point)
{
	switch (point->EnodeType) {
	case POINT_ENODE_TYPE_NODE_LIST:
		ListDestroy(&point->u2.enodeList);
		break;
	default:
		break;
	}
	point->EnodeType = POINT_ENODE_TYPE_NONE;
}

void PointEsmDestroy(Point *point)
{
	switch (point->EsmType) {
	case POINT_ESM_TYPE_SM_LIST:
		ListDestroy(&point->u3.esmList);
		break;
	default:
		break;
	}
	point->EsmType = POINT_ESM_TYPE_NONE;
}

void PointElinkDestroy(Point *point)
{
	switch (point->ElinkType) {
	case POINT_ELINK_TYPE_LINK_LIST:
		ListDestroy(&point->u4.elinkList);
		break;
	default:
		break;
	}
	point->ElinkType = POINT_ELINK_TYPE_NONE;
}

void PointDestroy(Point *point)
{
	PointFabricDestroy(point);
	PointEnodeDestroy(point);
	PointEsmDestroy(point);
	PointElinkDestroy(point);
}

/* is the point valid (eg. successfully parsed and/or populated)
 * PointInit and PointDestroy will return a point to an invalid state
 * PointParseFocus if sucessful will leave a point in a valid state
 */
boolean PointValid(Point *point)
{
	return (point && (point->Type != POINT_TYPE_NONE
			|| point->EnodeType != POINT_ENODE_TYPE_NONE
			|| point->EsmType != POINT_ESM_TYPE_NONE
			|| point->ElinkType != POINT_ELINK_TYPE_NONE));
}

/* append object to the list
 * if this is the 1st insert to a "None" Point, it will initialize the
 * list and set the point type.
 * On failure, the point is destroyed by this routine
 */
FSTATUS PointListAppend(Point *point, PointType type, void *object)
{
	FSTATUS status;
	DLIST *pList;

	if (point->Type == POINT_TYPE_NONE) {
		status = PointInitList(point, type);
		if (FSUCCESS != status) {
			PointDestroy(point);
			return status;
		}
	} else if (type != point->Type) {
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	switch (type) {
	case POINT_TYPE_PORT_LIST:
		pList = &point->u.portList;
		break;
	case POINT_TYPE_NODE_LIST:
		pList = &point->u.nodeList;
		break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC_LIST:
		pList = &point->u.iocList;
		break;
#endif
	default:
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	if (! ListInsertTail(pList, object)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		PointDestroy(point);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

/* append object to the ExpectedNode list
 * if this is the 1st insert to a "None" Point, it will initialize the
 * list and set the point enode type.
 * Failures imply a caller bug or a failure to allocate memory
 * On failure, the point is destroyed by this routine
 */
FSTATUS PointEnodeListAppend(Point *point, PointEnodeType type, void *object)
{
	FSTATUS status;
	DLIST *pList;

	if (point->EnodeType == POINT_ENODE_TYPE_NONE) {
		status = PointInitEnodeList(point, type);
		if (FSUCCESS != status) {
			PointDestroy(point);
			return status;
		}
	} else if (type != point->EnodeType) {
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	switch (type) {
	case POINT_ENODE_TYPE_NODE_LIST:
		pList = &point->u2.enodeList;
		break;
	default:
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	if (! ListInsertTail(pList, object)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		PointDestroy(point);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

/* append object to the ExpectedSM list
 * if this is the 1st insert to a "None" Point, it will initialize the
 * list and set the point enode type.
 * Failures imply a caller bug or a failure to allocate memory
 * On failure, the point is destroyed by this routine
 */
FSTATUS PointEsmListAppend(Point *point, PointEsmType type, void *object)
{
	FSTATUS status;
	DLIST *pList;

	if (point->EsmType == POINT_ESM_TYPE_NONE) {
		status = PointInitEsmList(point, type);
		if (FSUCCESS != status) {
			PointDestroy(point);
			return status;
		}
	} else if (type != point->EsmType) {
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	switch (type) {
	case POINT_ESM_TYPE_SM_LIST:
		pList = &point->u3.esmList;
		break;
	default:
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	if (! ListInsertTail(pList, object)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		PointDestroy(point);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

/* append object to the ExpectedLink list
 * if this is the 1st insert to a "None" Point, it will initialize the
 * list and set the point enode type.
 * Failures imply a caller bug or a failure to allocate memory
 * On failure, the point is destroyed by this routine
 */
FSTATUS PointElinkListAppend(Point *point, PointElinkType type, void *object)
{
	FSTATUS status;
	DLIST *pList;

	if (point->ElinkType == POINT_ELINK_TYPE_NONE) {
		status = PointInitElinkList(point, type);
		if (FSUCCESS != status) {
			PointDestroy(point);
			return status;
		}
	} else if (type != point->ElinkType) {
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	switch (type) {
	case POINT_ELINK_TYPE_LINK_LIST:
		pList = &point->u4.elinkList;
		break;
	default:
		ASSERT(0);
		PointDestroy(point);
		return FINVALID_OPERATION;
	}
	if (! ListInsertTail(pList, object)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		PointDestroy(point);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

/* Failures imply a caller bug or a failure to allocate memory */
/* On failure will Destroy the whole dest point leaving it !PointValid */
FSTATUS PointFabricCopy(Point *dest, Point *src)
{
	FSTATUS status;
	LIST_ITERATOR i;
	DLIST *pSrcList;

	PointFabricDestroy(dest);

	pSrcList = NULL;
	switch (src->Type) {
	case POINT_TYPE_PORT:
	case POINT_TYPE_NODE:
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
#endif
	case POINT_TYPE_SYSTEM:
	default:
		*dest = *src;
		break;
	case POINT_TYPE_PORT_LIST:
		pSrcList = &src->u.portList;
		break;
	case POINT_TYPE_NODE_LIST:
		pSrcList = &src->u.nodeList;
		break;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC_LIST:
		pSrcList = &src->u.iocList;
		break;
#endif
	}
	if (pSrcList) {
		for (i=ListHead(pSrcList); i != NULL; i = ListNext(pSrcList, i)) {
			/* on failure Append will Destroy the whole point */
			status = PointListAppend(dest, src->Type, ListObj(i));
			if (FSUCCESS != status)
				return status;
		}
	}
	return FSUCCESS;
}


/* Failures imply a caller bug or a failure to allocate memory */
/* On failure will Destroy the whole dest point leaving it !PointValid */
FSTATUS PointEnodeCopy(Point *dest, Point *src)
{
	FSTATUS status;
	LIST_ITERATOR i;
	DLIST *pSrcList;

	PointEnodeDestroy(dest);
	pSrcList = NULL;
	switch (src->EnodeType) {
	case POINT_ENODE_TYPE_NONE:
	case POINT_ENODE_TYPE_NODE:
		*dest = *src;
		break;
	case POINT_ENODE_TYPE_NODE_LIST:
		pSrcList = &src->u2.enodeList;
		break;
	}

	if (pSrcList) {
		for (i=ListHead(pSrcList); i != NULL; i = ListNext(pSrcList, i)) {
			/* on failure Append will Destroy the whole point */
			status = PointEnodeListAppend(dest, src->EnodeType, ListObj(i));
			if (FSUCCESS != status)
				return status;
		}
	}
	return FSUCCESS;
}

/* Failures imply a caller bug or a failure to allocate memory */
/* On failure will Destroy the whole dest point leaving it !PointValid */
FSTATUS PointEsmCopy(Point *dest, Point *src)
{
	FSTATUS status;
	LIST_ITERATOR i;
	DLIST *pSrcList;

	PointEsmDestroy(dest);
	pSrcList = NULL;
	switch (src->EsmType) {
	case POINT_ESM_TYPE_NONE:
	case POINT_ESM_TYPE_SM:
		*dest = *src;
		break;
	case POINT_ESM_TYPE_SM_LIST:
		pSrcList = &src->u3.esmList;
		break;
	}

	if (pSrcList) {
		for (i=ListHead(pSrcList); i != NULL; i = ListNext(pSrcList, i)) {
			/* on failure Append will Destroy the whole point */
			status = PointEsmListAppend(dest, src->EsmType, ListObj(i));
			if (FSUCCESS != status)
				return status;
		}
	}
	return FSUCCESS;
}

/* Failures imply a caller bug or a failure to allocate memory */
/* On failure will Destroy the whole dest point leaving it !PointValid */
FSTATUS PointElinkCopy(Point *dest, Point *src)
{
	FSTATUS status;
	LIST_ITERATOR i;
	DLIST *pSrcList;

	PointElinkDestroy(dest);
	pSrcList = NULL;
	switch (src->ElinkType) {
	case POINT_ELINK_TYPE_NONE:
	case POINT_ELINK_TYPE_LINK:
		*dest = *src;
		break;
	case POINT_ELINK_TYPE_LINK_LIST:
		pSrcList = &src->u4.elinkList;
		break;
	}

	if (pSrcList) {
		for (i=ListHead(pSrcList); i != NULL; i = ListNext(pSrcList, i)) {
			/* on failure Append will Destroy the whole point */
			status = PointElinkListAppend(dest, src->ElinkType, ListObj(i));
			if (FSUCCESS != status)
				return status;
		}
	}
	return FSUCCESS;
}

/* Failures imply a caller bug or a failure to allocate memory */
/* On failure will Destroy the whole dest point leaving it !PointValid */
FSTATUS PointCopy(Point *dest, Point *src)
{
	FSTATUS status;

	status = PointFabricCopy(dest, src);
	if (FSUCCESS != status)
		return status;
	status = PointEnodeCopy(dest, src);
	if (FSUCCESS != status)
		return status;
	status = PointEsmCopy(dest, src);
	if (FSUCCESS != status)
		return status;
	return PointElinkCopy(dest, src);
}

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
 * if !PointValid will report TRUE for all ports
 */
boolean ComparePortPoint(PortData *portp, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->Type) {
	case POINT_TYPE_NONE:
		return FALSE;
	case POINT_TYPE_PORT:
		return (portp == point->u.portp);
	case POINT_TYPE_PORT_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.portList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			PortData *portp2 = (PortData*)ListObj(i);
			if (portp == portp2)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_NODE:
		return (portp->nodep == point->u.nodep);
	case POINT_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			NodeData *nodep = (NodeData*)ListObj(i);
			if (portp->nodep == nodep)
				return TRUE;
		}
		}
		return FALSE;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		return (portp->nodep == point->u.iocp->ioup->nodep);
	case POINT_TYPE_IOC_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			IocData *iocp = (IocData*)ListObj(i);
			if (portp->nodep == iocp->ioup->nodep)
				return TRUE;
		}
		}
		return FALSE;
#endif
	case POINT_TYPE_SYSTEM:
		return (portp->nodep->systemp == point->u.systemp);
	}
	return TRUE;	// should not get here
}

/* compare the supplied node to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all nodes
 */
boolean CompareNodePoint(NodeData *nodep, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->Type) {
	case POINT_TYPE_NONE:
		return FALSE;
	case POINT_TYPE_PORT:
		return (nodep == point->u.portp->nodep);
	case POINT_TYPE_PORT_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.portList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			PortData *portp = (PortData*)ListObj(i);
			if (nodep == portp->nodep)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_NODE:
		return (nodep == point->u.nodep);
	case POINT_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			NodeData *nodep2 = (NodeData*)ListObj(i);
			if (nodep == nodep2)
				return TRUE;
		}
		}
		return FALSE;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		return (nodep == point->u.iocp->ioup->nodep);
	case POINT_TYPE_IOC_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			IocData *iocp = (IocData*)ListObj(i);
			if (nodep == iocp->ioup->nodep)
				return TRUE;
		}
		}
		return FALSE;
#endif
	case POINT_TYPE_SYSTEM:
		return (nodep->systemp == point->u.systemp);
	}
	return TRUE;	// should not get here
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
/* compare the supplied iou to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ious
 */
boolean CompareIouPoint(IouData *ioup, Point *point)
{
	/* each IOU is associated with a single node, and a given node is
	 * associated with 0 or 1 IOUs.  So we can simply use a Node compare
	 * for all point->Type values, even if the point is an IOC or IOC_LIST
	 */
	return CompareNodePoint(ioup->nodep, point);
}

/* compare the supplied ioc to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all iocs
 */
boolean CompareIocPoint(IocData *iocp, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->Type) {
	case POINT_TYPE_IOC:
		return (iocp == point->u.iocp);
	case POINT_TYPE_IOC_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			IocData *iocp2 = (IocData*)ListObj(i);
			if (iocp == iocp2)
				return TRUE;
		}
		}
		return FALSE;
	default:
		return CompareNodePoint(iocp->ioup->nodep, point);
	}
}
#endif

/* compare the supplied SM to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all SMs
 */
boolean CompareSmPoint(SMData *smp, Point *point)
{
	return ComparePortPoint(smp->portp, point);
}

/* compare the supplied system to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all systems
 */
boolean CompareSystemPoint(SystemData *systemp, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->Type) {
	case POINT_TYPE_NONE:
		return FALSE;
	case POINT_TYPE_PORT:
		return (systemp == point->u.portp->nodep->systemp);
	case POINT_TYPE_PORT_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.portList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			PortData *portp = (PortData*)ListObj(i);
			if (systemp == portp->nodep->systemp)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_NODE:
		return (systemp == point->u.nodep->systemp);
	case POINT_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			NodeData *nodep = (NodeData*)ListObj(i);
			if (systemp == nodep->systemp)
				return TRUE;
		}
		}
		return FALSE;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		return (systemp == point->u.iocp->ioup->nodep->systemp);
	case POINT_TYPE_IOC_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			IocData *iocp = (IocData*)ListObj(i);
			if (systemp == iocp->ioup->nodep->systemp)
				return TRUE;
		}
		}
		return FALSE;
#endif
	case POINT_TYPE_SYSTEM:
		return (systemp == point->u.systemp);
	}
	return TRUE;	// should not get here
}

/* compare the supplied ExpectedNode to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ExpectedNodes
 */
boolean CompareExpectedNodePoint(ExpectedNode *enodep, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->EnodeType) {
	case POINT_ENODE_TYPE_NONE:
		return FALSE;
	case POINT_ENODE_TYPE_NODE:
		return (enodep == point->u2.enodep);
	case POINT_ENODE_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u2.enodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			ExpectedNode *enodep2 = (ExpectedNode*)ListObj(i);
			if (enodep == enodep2)
				return TRUE;
		}
		}
		return FALSE;
	}
	return TRUE;	// should not get here
}

/* compare the supplied ExpectedSM to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ExpectedSM
 */
boolean CompareExpectedSMPoint(ExpectedSM *esmp, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->EsmType) {
	case POINT_ESM_TYPE_NONE:
		return FALSE;
	case POINT_ESM_TYPE_SM:
		return (esmp == point->u3.esmp);
	case POINT_ESM_TYPE_SM_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u3.esmList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			ExpectedSM *esmp2 = (ExpectedSM*)ListObj(i);
			if (esmp == esmp2)
				return TRUE;
		}
		}
		return FALSE;
	}
	return TRUE;	// should not get here
}

/* compare the supplied ExpectedLink to the given point
 * this is used to identify focus for reports
 * if !PointValid will report TRUE for all ExpectedLink
 */
boolean CompareExpectedLinkPoint(ExpectedLink *elinkp, Point *point)
{
	if (!PointValid(point))
		return TRUE;
	switch (point->ElinkType) {
	case POINT_ELINK_TYPE_NONE:
		return FALSE;
	case POINT_ELINK_TYPE_LINK:
		return (elinkp == point->u4.elinkp);
	case POINT_ELINK_TYPE_LINK_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u4.elinkList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			ExpectedLink *elinkp2 = (ExpectedLink*)ListObj(i);
			if (elinkp == elinkp2)
				return TRUE;
		}
		}
		return FALSE;
	}
	return TRUE;	// should not get here
}

/* append the given port to the PORT_LIST point, if its already
 * in the list, do nothing
 */
FSTATUS PointListAppendUniquePort(Point *point, PortData *portp)
{
	if (point->Type == POINT_TYPE_PORT_LIST && ComparePortPoint(portp, point))
		return FSUCCESS;
	return PointListAppend(point, POINT_TYPE_PORT_LIST, portp);
}

/* If possible compress a point into a simpler format
 * This looks for lists which consist of a single entry or
 * lists which include all the components of a higher level type
 */
void PointFabricCompress(Point *point)
{
	switch (point->Type) {
	case POINT_TYPE_NONE:
		break;
	case POINT_TYPE_PORT:
		break;
	case POINT_TYPE_PORT_LIST:
		{
		PortData *portp;
		LIST_ITERATOR head = ListHead(&point->u.portList);
		ASSERT(head);

		ASSERT(ListCount(&point->u.portList) >= 1);

		portp = (PortData*)ListObj(head);
		if (ListCount(&point->u.portList) == 1) {
			/* degenerate case, simplify as a single port */
			PointFabricDestroy(point);
			PointInitSimple(point, POINT_TYPE_PORT, portp);
		} else if (ListCount(&point->u.portList) == cl_qmap_count(&portp->nodep->Ports)) {
			/* maybe we can consolidate to a single node */
			LIST_ITERATOR i;
			DLIST *pList = &point->u.portList;

			for (i=ListHead(pList); portp && i != NULL; i = ListNext(pList, i)) {
				if (portp->nodep != ((PortData*)ListObj(i))->nodep)
					portp = NULL;	/* not in same node, flag for below */
			}
			if (portp) {
				/* degenerate case, simplify as a single node */
				PointFabricDestroy(point);
				PointInitSimple(point, POINT_TYPE_NODE, portp->nodep);
			}
#if 0
		} else {
			// the likelihood of this is low for port oriented searches
			// and it would present just the system image guide in the summary
			// and may be less obvious to the user than a list of ports
			/* maybe we can consolidate to a single system */
			LIST_ITERATOR i;
			DLIST *pList = &point->u.portList;

			for (i=ListHead(pList); portp && i != NULL; i = ListNext(pList, i)) {
				if (portp->nodep->systemp != ((PortData*)ListObj(i))->nodep->systemp)
					portp = NULL;	/* not in same system, flag for below */
			}
			if (portp) {
				/* all ports are in same system. is it a complete list? */
				/* count ports in the system */
				uint32 count = 0;
				cl_map_item_t *p;

				for (p=cl_qmap_head(&portp->nodep->systemp->Nodes); p != cl_qmap_end(&portp->nodep->systemp->Nodes); p = cl_qmap_next(p)) {
					NodeData *nodep = PARENT_STRUCT(p, NodeData, SystemNodesEntry);
					count += cl_qmap_count(&nodep->Ports);
				}
				if (ListCount(&point->u.portList) != count)
					portp = NULL;	/* incomplete list, flag for below */
			}
			if (portp) {
				/* degenerate case, simplify as a single system */
				PointFabricDestroy(point);
				PointInitSimple(point, POINT_TYPE_SYSTEM, portp->nodep->systemp);
			}
#endif
		}
		break;
		}
	case POINT_TYPE_NODE:
		break;
	case POINT_TYPE_NODE_LIST:
		{
		NodeData *nodep;
		LIST_ITERATOR head = ListHead(&point->u.nodeList);
		ASSERT(head);

		ASSERT(ListCount(&point->u.nodeList) >= 1);
		nodep = (NodeData*)ListObj(head);
		if (ListCount(&point->u.nodeList) == 1) {
			/* degenerate case, simplify as a single node */
			PointFabricDestroy(point);
			PointInitSimple(point, POINT_TYPE_NODE, nodep);
		} else if (ListCount(&point->u.nodeList) == cl_qmap_count(&nodep->systemp->Nodes)) {
			/* maybe we can consolidate to a single system */
			LIST_ITERATOR i;
			DLIST *pList = &point->u.nodeList;

			for (i=ListHead(pList); nodep && i != NULL; i = ListNext(pList, i)) {
				if (nodep->systemp != ((NodeData*)ListObj(i))->systemp)
					nodep = NULL;	/* not in same system, flag for below */
			}
			if (nodep) {
				/* degenerate case, simplify as a single system */
				PointFabricDestroy(point);
				PointInitSimple(point, POINT_TYPE_SYSTEM, nodep->systemp);
			}
		}
		break;
		}
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		break;
	case POINT_TYPE_IOC_LIST:
		{
		IocData *iocp;
		LIST_ITERATOR head = ListHead(&point->u.iocList);
		ASSERT(head);

		ASSERT(ListCount(&point->u.iocList) >= 1);
		iocp = (IocData*)ListObj(head);
		if (ListCount(&point->u.iocList) == 1) {
			/* degenerate case, simplify as a single IOC */
			PointFabricDestroy(point);
			PointInitSimple(point, POINT_TYPE_IOC, iocp);
		} else if (ListCount(&point->u.iocList) == QListCount(&iocp->ioup->Iocs)) {
			/* maybe we can consolidate to a single node */
			LIST_ITERATOR i;
			DLIST *pList = &point->u.iocList;

			for (i=ListHead(pList); iocp && i != NULL; i = ListNext(pList, i)) {
				if (iocp->ioup != ((IocData*)ListObj(i))->ioup)
					iocp = NULL;	/* not in same iou, flag for below */
			}
			if (iocp) {
				/* degenerate case, simplify as a single node */
				PointFabricDestroy(point);
				PointInitSimple(point, POINT_TYPE_NODE, iocp->ioup->nodep);
			}
#if 0
		} else {
			// the likelihood of this is low for ioc oriented searches
			// and it would present just the system image guide in the summary
			// and may be less obvious to the user than a list of iocs
			/* maybe we can consolidate to a single system */
			LIST_ITERATOR i;
			DLIST *pList = &point->u.iocList;

			for (i=ListHead(pList); iocp && i != NULL; i = ListNext(pList, i)) {
				if (iocp->ioup->nodep->systemp != ((IocData*)ListObj(i))->ioup->nodep->systemp)
					iocp = NULL;	/* not in same system, flag for below */
			}
			if (iocp) {
				/* all IOCs are in same system. is it a complete list? */
				/* count IOCs in the system */
				uint32 count = 0;
				cl_map_item_t *p;

				for (p=cl_qmap_head(&iocp->ioup->nodep->systemp->Nodes); p != cl_qmap_end(&iocp->ioup->nodep->systemp->Nodes); p = cl_qmap_next(p)) {
					NodeData *nodep = PARENT_STRUCT(p, NodeData, SystemNodesEntry);
					if (nodep->ioup)
						count += QListCount(&nodep->ioup->Iocs);
				}
				if (ListCount(&point->u.iocList) != count)
					iocp = NULL;	/* incomplete list, flag for below */
			}
			if (iocp) {
				/* degenerate case, simplify as a single system */
				PointFabricDestroy(point);
				PointInitSimple(point, POINT_TYPE_SYSTEM, iocp->ioup->nodep->systemp);
			}
#endif
		}
		break;
		}
#endif
	case POINT_TYPE_SYSTEM:
		break;
	}
}

/* If possible compress a point into a simpler format
 * This looks for lists which consist of a single entry
 */
void PointEnodeCompress(Point *point)
{
	switch (point->EnodeType) {
	case POINT_ENODE_TYPE_NONE:
		break;
	case POINT_ENODE_TYPE_NODE:
		break;
	case POINT_ENODE_TYPE_NODE_LIST:
		{
		ASSERT(ListCount(&point->u2.enodeList) >= 1);
		if (ListCount(&point->u2.enodeList) == 1) {
			/* degenerate case, simplify as a single ExpectedNode */
			LIST_ITERATOR head = ListHead(&point->u2.enodeList);
			ExpectedNode *enodep;
			ASSERT(head);
			enodep = (ExpectedNode*)ListObj(head);
			PointEnodeDestroy(point);
			PointInitEnodeSimple(point, POINT_ENODE_TYPE_NODE, enodep);
		}
		break;
		}
	}
}

/* If possible compress a point into a simpler format
 * This looks for lists which consist of a single entry
 */
void PointEsmCompress(Point *point)
{
	switch (point->EsmType) {
	case POINT_ESM_TYPE_NONE:
		break;
	case POINT_ESM_TYPE_SM:
		break;
	case POINT_ESM_TYPE_SM_LIST:
		{
		ASSERT(ListCount(&point->u3.esmList) >= 1);
		if (ListCount(&point->u3.esmList) == 1) {
			/* degenerate case, simplify as a single ExpectedSM */
			LIST_ITERATOR head = ListHead(&point->u3.esmList);
			ExpectedSM *esmp;
			ASSERT(head);
			esmp = (ExpectedSM*)ListObj(head);
			PointEsmDestroy(point);
			PointInitEsmSimple(point, POINT_ESM_TYPE_SM, esmp);
		}
		break;
		}
	}
}

/* If possible compress a point into a simpler format
 * This looks for lists which consist of a single entry
 */
void PointElinkCompress(Point *point)
{
	switch (point->ElinkType) {
	case POINT_ELINK_TYPE_NONE:
		break;
	case POINT_ELINK_TYPE_LINK:
		break;
	case POINT_ELINK_TYPE_LINK_LIST:
		{
		ASSERT(ListCount(&point->u4.elinkList) >= 1);
		if (ListCount(&point->u4.elinkList) == 1) {
			/* degenerate case, simplify as a single ExpectedLink */
			LIST_ITERATOR head = ListHead(&point->u4.elinkList);
			ExpectedLink *elinkp;
			ASSERT(head);
			elinkp = (ExpectedLink*)ListObj(head);
			PointElinkDestroy(point);
			PointInitElinkSimple(point, POINT_ELINK_TYPE_LINK, elinkp);
		}
		break;
		}
	}
}

/* If possible compress a point into a simpler format
 * This looks for lists which consist of a single entry or
 * lists which include all the components of a higher level type
 */
void PointCompress(Point *point)
{
	PointFabricCompress(point);
	PointEnodeCompress(point);
	PointEsmCompress(point);
	PointElinkCompress(point);
}
