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

void PointDestroy(Point *point)
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
		if (FSUCCESS != status)
			return status;
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
		return FINVALID_OPERATION;
	}
	if (! ListInsertTail(pList, object)) {
		fprintf(stderr, "%s: unable to allocate memory\n", g_Top_cmdname);
		PointDestroy(point);
		return FINSUFFICIENT_MEMORY;
	}
	return FSUCCESS;
}

FSTATUS PointCopy(Point *dest, Point *src)
{
	FSTATUS status;
	LIST_ITERATOR i;
	DLIST *pSrcList;

	PointDestroy(dest);

	switch (src->Type) {
	case POINT_TYPE_PORT:
	case POINT_TYPE_NODE:
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
#endif
	case POINT_TYPE_SYSTEM:
	default:
		*dest = *src;
		return FSUCCESS;
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

	for (i=ListHead(pSrcList); i != NULL; i = ListNext(pSrcList, i)) {
		status = PointListAppend(dest, src->Type, ListObj(i));
		if (FSUCCESS != status)
			return status;
	}
	return FSUCCESS;
}

/* compare the supplied port to the given point
 * this is used to identify focus for reports
 * POINT_TYPE_NONE will report TRUE for all ports
 */
boolean ComparePortPoint(PortData *portp, Point *point)
{
	switch (point->Type) {
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
	default:
		return TRUE;
	}
}

/* compare the supplied node to the given point
 * this is used to identify focus for reports
 * POINT_TYPE_NONE will report TRUE for all nodes
 */
boolean CompareNodePoint(NodeData *nodep, Point *point)
{
	switch (point->Type) {
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
	default:
		return TRUE;
	}
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
/* compare the supplied iou to the given point
 * this is used to identify focus for reports
 * POINT_TYPE_NONE will report TRUE for all ious
 */
boolean CompareIouPoint(IouData *ioup, Point *point)
{
	switch (point->Type) {
	case POINT_TYPE_PORT:
		return (ioup->nodep == point->u.portp->nodep);
	case POINT_TYPE_PORT_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.portList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			PortData *portp = (PortData*)ListObj(i);
			if (ioup->nodep == portp->nodep)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_NODE:
		return (ioup->nodep == point->u.nodep);
	case POINT_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			NodeData *nodep = (NodeData*)ListObj(i);
			if (ioup->nodep == nodep)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_IOC:
		return (ioup == point->u.iocp->ioup);
	case POINT_TYPE_IOC_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			IocData *iocp = (IocData*)ListObj(i);
			if (ioup == iocp->ioup)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_SYSTEM:
		return (ioup->nodep->systemp == point->u.systemp);
	default:
		return TRUE;
	}
}

/* compare the supplied ioc to the given point
 * this is used to identify focus for reports
 * POINT_TYPE_NONE will report TRUE for all iocs
 */
boolean CompareIocPoint(IocData *iocp, Point *point)
{
	switch (point->Type) {
	case POINT_TYPE_PORT:
		return (iocp->ioup->nodep == point->u.portp->nodep);
	case POINT_TYPE_PORT_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.portList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			PortData *portp = (PortData*)ListObj(i);
			if (iocp->ioup->nodep == portp->nodep)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_NODE:
		return (iocp->ioup->nodep == point->u.nodep);
	case POINT_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			NodeData *nodep = (NodeData*)ListObj(i);
			if (iocp->ioup->nodep == nodep)
				return TRUE;
		}
		}
		return FALSE;
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
	case POINT_TYPE_SYSTEM:
		return (iocp->ioup->nodep->systemp == point->u.systemp);
	default:
		return TRUE;
	}
}
#endif

/* compare the supplied SM to the given point
 * this is used to identify focus for reports
 * POINT_TYPE_NONE will report TRUE for all SMs
 */
boolean CompareSmPoint(SMData *smp, Point *point)
{
	switch (point->Type) {
	case POINT_TYPE_PORT:
		return (smp->portp == point->u.portp);
	case POINT_TYPE_PORT_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.portList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			PortData *portp = (PortData*)ListObj(i);
			if (smp->portp == portp)
				return TRUE;
		}
		}
		return FALSE;
	case POINT_TYPE_NODE:
		return (smp->portp->nodep == point->u.nodep);
	case POINT_TYPE_NODE_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			NodeData *nodep = (NodeData*)ListObj(i);
			if (smp->portp->nodep == nodep)
				return TRUE;
		}
		}
		return FALSE;
#if !defined(VXWORKS) || defined(BUILD_DMC)
	case POINT_TYPE_IOC:
		return (smp->portp->nodep == point->u.iocp->ioup->nodep);
	case POINT_TYPE_IOC_LIST:
		{
		LIST_ITERATOR i;
		DLIST *pList = &point->u.nodeList;

		for (i=ListHead(pList); i != NULL; i = ListNext(pList, i)) {
			IocData *iocp = (IocData*)ListObj(i);
			if (smp->portp->nodep == iocp->ioup->nodep)
				return TRUE;
		}
		}
		return FALSE;
#endif
	case POINT_TYPE_SYSTEM:
		return (smp->portp->nodep->systemp == point->u.systemp);
	default:
		return TRUE;
	}
}

/* compare the supplied system to the given point
 * this is used to identify focus for reports
 * POINT_TYPE_NONE will report TRUE for all systems
 */
boolean CompareSystemPoint(SystemData *systemp, Point *point)
{
	switch (point->Type) {
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
	default:
		return TRUE;
	}
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
void PointCompress(Point *point)
{
	switch (point->Type) {
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
			PointDestroy(point);
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
				PointDestroy(point);
				PointInitSimple(point, POINT_TYPE_NODE, portp->nodep);
			}
		}
		// TBD see if we can simplify to a single system, eg. all Ports in all Nodes of System
		break;
		}
	case POINT_TYPE_NODE:
		return;
	case POINT_TYPE_NODE_LIST:
		{
		NodeData *nodep;
		LIST_ITERATOR head = ListHead(&point->u.nodeList);
		ASSERT(head);

		ASSERT(ListCount(&point->u.nodeList) >= 1);
		nodep = (NodeData*)ListObj(head);
		if (ListCount(&point->u.nodeList) == 1) {
			/* degenerate case, simplify as a single node */
			PointDestroy(point);
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
				PointDestroy(point);
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
			PointDestroy(point);
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
				PointDestroy(point);
				PointInitSimple(point, POINT_TYPE_NODE, iocp->ioup->nodep);
			}
		}
		break;
		}
#endif
	case POINT_TYPE_SYSTEM:
		break;
	default:
		break;
	}
}
