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

#include "sm_l.h"
#include "pm_l.h"
#include <iba/ibt.h>
#include "pm_topology.h"
#include "sm_dbsync.h"
#include <limits.h>
#include "if3.h"

// Dispatcher Design.
// The dispatcher has 4 primary entry points:
// PmSweepAllPortCounters - used by PMEngine to start a sweep
// DispatchNodeCallback - callback when a Node level single packet response or timeout
// DispatchPacketCallback - callback when a Node level multi-packet response or timeout
//
// Each entry point is responsible for progressing the PM sweep.
// They do this by ensuring the configured number of parallel nodes and ports
// per node are in progress.  As such a single call could issue one or
// more packets.  Each call must hence be diligent in trying to find work to
// be done, especially if a given port or node is now done.
//
// Each call returns:
// 	VSTATUS_OK - as much work as possible is in progress
// 				- typically indicates new work was started.  Could also
// 					indicate that while a port finished on a node, and there
// 					are no other ports to start, there are other ports not yet
// 					done so we can't start anything else
// 	VSTATUS_NOT_FOUND - did not start any new work
// 	other - assorted errors from low level routines, did not start any new work
//
// One complicating factor is when send's fail immediately.  This is atypical
// but could be caused by problems in the IB stack or perhaps if the local
// port is down.  In this case, the call will continue to try and find other
// work.  Send failures could be transient or perhaps due to a simple problem
// like a duplicate TID.
//
// Each entry point starts from a different perspective (fabric, node or port)
// and tries to find work.  It does this by checking state for the node/port
// it is focused on, and if there is no more work to be done for that node/port
// it must advance to a different node/port.  Once the last port on a device is
// completed, it must advance to a different node, until there are no nodes left
// or until all the possible work is started.
//
// The walking to new Nodes is done only in these three routines.  As such
// when the processing of a given Node or Port is completed, these callers
// will try to start work on the next node available (and may conclude there is
// no new work to be started)
//
// A special case is the rare case of a Send call failing.
// The present approach is for the callback to immediately try to find other
// work (B below).
// An alternative approach is to ignore the Send failure and let the
// cs_context aging and timeout mechanisms handle it (A below)
// This is a trade-off between:
// A. Increasing the PM sweep time when some or all sends fail
// B. spending more time in a callback to search/try more work

static Status_t DispatchNextPacket(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket);

static uint8 DispatcherStartSweepAllPorts(Pm_t *pm, PmDispatcherNode_t *dispnode);

static Status_t DispatchNextNode(Pm_t *pm, PmDispatcherNode_t *dispnode);

static Status_t DispatchNodeNextStep(Pm_t *pm, PmNode_t *pmnodep, PmDispatcherNode_t *dispnode);
static Status_t DispatchPacketNextStep(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket);

static void DispatchPacketCallback(cntxt_entry_t *entry, Status_t status, void *data, Mai_t *mad);
static void DispatchNodeCallback(cntxt_entry_t *entry, Status_t status, void *data, Mai_t *mad);

static void DispatchPacketDone(Pm_t *pm, PmDispatcherPacket_t *disppacket);
static void DispatchNodeDone(Pm_t *pm, PmDispatcherNode_t *dispnode); 

int MergePortIntoPacket(Pm_t *pm, PmDispatcherNode_t *dispnode, PmPort_t *pmportp, PmDispatcherPacket_t *disppacket);

size_t CalculatePortInPacket(PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket);

// -------------------------------------------------------------------------
// Utility routines
// -------------------------------------------------------------------------

static void PmFailNodeQuery(cntxt_entry_t *entry, PmDispatcherNode_t *dispnode, uint8 method, uint16 aid)
{
	dispnode->info.u.s.failed = 1;
	PmFailNode(dispnode->pm, dispnode->info.pmnodep, PM_QUERY_STATUS_FAIL_QUERY, method, aid);
	if (entry)
		cs_cntxt_retire_nolock( entry, &dispnode->pm->Dispatcher.cntx  );
	DispatchNodeDone(dispnode->pm, dispnode);
}

static void PmFailPacketClear(cntxt_entry_t *entry, PmDispatcherPacket_t *disppacket, uint8 method, uint16 aid)
{
	PmDispatcherNode_t *dispnode = disppacket->dispnode;

	dispnode->info.u.s.failed = 1;
	PmFailPacket(dispnode->pm, disppacket, PM_QUERY_STATUS_FAIL_CLEAR, method, aid);
	if (entry)
		cs_cntxt_retire_nolock( entry, &dispnode->pm->Dispatcher.cntx  );
	DispatchPacketDone(dispnode->pm, disppacket);
}

static void PmFailPacketQuery(cntxt_entry_t *entry, PmDispatcherPacket_t *disppacket, uint8 method, uint16 aid)
{
	PmDispatcherNode_t *dispnode = disppacket->dispnode;

	dispnode->info.u.s.failed = 1;
	PmFailPacket(dispnode->pm, disppacket, PM_QUERY_STATUS_FAIL_QUERY, method, aid);
	if (entry)
		cs_cntxt_retire_nolock( entry, &dispnode->pm->Dispatcher.cntx  );
	DispatchPacketDone(dispnode->pm, disppacket);
}

static void PmNodeMadFail(Mai_t *mad, PmNode_t *pmnodep)
{
	IB_LOG_INFO_FMT(__func__, "PMA response for %s(%s) with bad status: 0x%x From %.*s Guid "FMT_U64" LID 0x%x",
		StlPmMadMethodToText(mad->base.method), StlPmMadAttributeToText(mad->base.aid), mad->base.status,
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid);
}

static void PmMadFailNodeQuery(cntxt_entry_t *entry, Mai_t *mad, PmDispatcherNode_t *dispnode)
{
	PmNodeMadFail(mad, dispnode->info.pmnodep);
	PmFailNodeQuery(entry, dispnode, mad->base.method, mad->base.aid);
}

static void PmPacketMadFail(Mai_t *mad, PmDispatcherPacket_t *disppacket)
{
#ifndef VIEO_TRACE_DISABLED
	PmNode_t *pmnodep = disppacket->dispnode->info.pmnodep;

	IB_LOG_INFO_FMT(__func__, "PMA response for %s(%s) with bad status: 0x%x From %.*s Guid "FMT_U64" LID 0x%x Ports %u",
		StlPmMadMethodToText(mad->base.method), StlPmMadAttributeToText(mad->base.aid), mad->base.status,
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, (mad->base.amod >> 24) & 0xFF);
#endif
}

static void PmMadFailPacketQuery(cntxt_entry_t *entry, Mai_t *mad, PmDispatcherPacket_t *disppacket)
{
	PmPacketMadFail(mad, disppacket);
	PmFailPacketQuery(entry, disppacket, mad->base.method, mad->base.aid);
}

static void PmNodeMadAttrWrong(Mai_t *mad, PmNode_t *pmnodep)
{
	IB_LOG_INFO_FMT(__func__, "PMA response for %s(%s) with wrong Attr: 0x%x From %.*s Guid "FMT_U64" LID 0x%x",
		StlPmMadMethodToText(mad->base.method), StlPmMadAttributeToText(mad->base.aid), mad->base.aid,
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid);
}

static void PmMadAttrWrongNodeQuery(cntxt_entry_t *entry, Mai_t *mad, PmDispatcherNode_t *dispnode)
{
	PmNodeMadAttrWrong(mad, dispnode->info.pmnodep);
	PmFailNodeQuery(entry, dispnode, mad->base.method, mad->base.aid);
}

static void PmNodeMadSelectWrong(Mai_t *mad, PmNode_t *pmnodep)
{
	IB_LOG_INFO_FMT(__func__, "PMA response for %s(%s) with wrong Port/VL Select From %.*s Guid "FMT_U64" LID 0x%x",
		StlPmMadMethodToText(mad->base.method), StlPmMadAttributeToText(mad->base.aid),
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid);
}

static void PmMadAttrWrongPacketQuery(cntxt_entry_t *entry, Mai_t *mad, PmDispatcherPacket_t *disppacket)
{
	PmNodeMadAttrWrong(mad, disppacket->dispnode->info.pmnodep);
	PmFailPacketQuery(entry, disppacket, mad->base.method, mad->base.aid);
}

static void PmMadSelectWrongPacketQuery(cntxt_entry_t *entry, Mai_t *mad, PmDispatcherPacket_t *disppacket)
{
	PmNodeMadSelectWrong(mad, disppacket->dispnode->info.pmnodep);
	PmFailPacketQuery(entry, disppacket, mad->base.method, mad->base.aid);
}
// Copy port counters in STL_PORT_STATUS_RSP to port counters as referenced
//   by PmDispatcherPort_t.pPortImage->StlPortCounters/StlVLPortCounters
static void PmCopyPortStatus(STL_PORT_STATUS_RSP *madp, PmDispatcherPacket_t * disppacket)
{
	uint32 vl, i, vlSelMask;
	size_t size_counters;
	PmCompositePortCounters_t * pCounters = &disppacket->DispPorts[0].pPortImage->StlPortCounters;
	PmCompositeVLCounters_t * pVlCounters = &disppacket->DispPorts[0].pPortImage->StlVLPortCounters[0];
#ifndef VIEO_TRACE_DISABLED
	PmPort_t *pmportp = disppacket->DispPorts[0].pmportp;
	PmNode_t *pmnodep = pmportp->pmnodep;

	IB_LOG_INFO_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x Port %u",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, pmportp->portNum);
#endif

	size_counters = (size_t)&madp->UncorrectableErrors - (size_t)&madp->PortXmitData + 1;
	memcpy( &pCounters->PortXmitData, &madp->PortXmitData, size_counters );
	pCounters->lq.s.LinkQualityIndicator = madp->lq.s.LinkQualityIndicator;

	for (vl = 0, i = 0, vlSelMask = madp->VLSelectMask; (vl < STL_MAX_VLS) && vlSelMask;
			vl++, vlSelMask >>= 1) {
		if (vlSelMask & 0x1) {
			memcpy( &pVlCounters[vl_to_idx(vl)], &madp->VLs[i++], sizeof(struct _vls_pctrs) );
		}
	}

}	// End of PmCopyPortStatus()

// Copy data port counters in STL_DATA_PORT_COUNTERS_RSP to port counters
//   as referenced by disppacket; VLSelectMask has been checked == 0 if
//   process_vl_counters == 0
static void PmCopyDataPortCounters(STL_DATA_PORT_COUNTERS_RSP *madp, PmDispatcherPacket_t * disppacket)
{
	uint32 i, vl, j, vlSelMask;
	size_t size_counters, size_port;
	STL_DATA_PORT_COUNTERS_RSP * respp = madp;
	PmCompositePortCounters_t * pCounters;
	PmCompositeVLCounters_t * pVlCounters;
	PmDispatcherNode_t * dispnode = disppacket->dispnode;
	PmDispatcherPort_t * dispport;
#ifndef VIEO_TRACE_DISABLED
	PmNode_t *pmnodep = dispnode->info.pmnodep;

	IB_LOG_INFO_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x PortSelectMask[3] "FMT_U64" VLSelectMask 0x%08x",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, madp->PortSelectMask[3], madp->VLSelectMask);
#endif

	size_port = CalculatePortInPacket(dispnode, disppacket);
	size_counters = (size_t)&madp->Port->PortErrorCounterSummary - (size_t)&madp->Port->PortXmitData;

	// Copy DataPortCounters
	for (i = 0; i < disppacket->numPorts; i++) {
		dispport = &disppacket->DispPorts[i];
		pCounters = &dispport->pPortImage->StlPortCounters;
		pVlCounters = &dispport->pPortImage->StlVLPortCounters[0];
		pCounters->lq.s.LinkQualityIndicator = respp->Port->lq.s.LinkQualityIndicator;

		memcpy(&pCounters->PortXmitData, &respp->Port->PortXmitData, size_counters);
		for ( vl = 0, j = 0, vlSelMask = madp->VLSelectMask; (vl < STL_MAX_VLS) && vlSelMask;
				vl++, vlSelMask >>= 1 ) {
			if (vlSelMask & 0x1) {
				memcpy( &pVlCounters[vl_to_idx(vl)],
					&respp->Port->VLs[j++], sizeof(struct _vls_dpctrs) );
			}
		}
		dispport->pPortImage->u.s.gotDataCntrs = 1;
		// Get Error if sum is non-zero and previous image is invalid
		//  OR get Error if previous image is valid and
		//       previous sum is different than current sum
		if ( (respp->Port->PortErrorCounterSummary != 0 && dispport->pPortImagePrev == NULL) ||
			  (dispport->pPortImagePrev &&
			    (PM_PORT_ERROR_SUMMARY(dispport->pPortImagePrev, 
									   StlResolutionToShift(pm_config.resolution.LocalLinkIntegrity, RES_ADDER_LLI),
									   StlResolutionToShift(pm_config.resolution.LinkErrorRecovery, RES_ADDER_LER)) != respp->Port->PortErrorCounterSummary) ) ) {
			dispport->dispNodeSwPort->flags.s.NeedsError = 1;
			disppacket->dispnode->info.u.s.needError = 1;
		}

		respp = (STL_DATA_PORT_COUNTERS_RSP *)((uint8 *)respp + size_port);
	}

}	// End of PmCopyDataPortCounters()

// Copy error port counters in STL_ERROR_PORT_COUNTERS_RSP to port counters
//   as referenced by disppacket; VLSelectMask has been checked == 0 if
//   process_vl_counters == 0
static void PmCopyErrorPortCounters(STL_ERROR_PORT_COUNTERS_RSP *madp, PmDispatcherPacket_t * disppacket)
{
	uint32 i, vl, j, vlSelMask;
	size_t size_port, size_counters;

	STL_ERROR_PORT_COUNTERS_RSP * respp = madp;
	PmCompositePortCounters_t * pCounters;
	PmCompositeVLCounters_t * pVlCounters;
	PmDispatcherPort_t * dispport;
#ifndef VIEO_TRACE_DISABLED
	PmNode_t *pmnodep = disppacket->dispnode->info.pmnodep;

	IB_LOG_INFO_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x PortSelectMask[3] "FMT_U64" VLSelectMask 0x%08x",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid,
		madp->PortSelectMask[3], madp->VLSelectMask);
#endif

	size_port = CalculatePortInPacket(disppacket->dispnode, disppacket);
	size_counters = (size_t)&madp->Port->UncorrectableErrors - (size_t)&madp->Port->PortRcvConstraintErrors + 1;

	// Copy ErrorPortCounters
	for (i = 0; i < disppacket->numPorts; i++) {
		dispport = &disppacket->DispPorts[i];
		pCounters = &dispport->pPortImage->StlPortCounters;
		pVlCounters = &dispport->pPortImage->StlVLPortCounters[0];

		memcpy(&pCounters->PortRcvConstraintErrors, &respp->Port->PortRcvConstraintErrors, size_counters);
		for (vl = 0, j = 0, vlSelMask = madp->VLSelectMask; (vl < STL_MAX_VLS) && vlSelMask;
				vl++, vlSelMask >>= 1) {
			if (vlSelMask & 0x1) {
				pVlCounters[vl_to_idx(vl)].PortVLXmitDiscards = respp->Port->VLs[j++].PortVLXmitDiscards;
			}
		}
		dispport->pPortImage->u.s.gotErrorCntrs = 1;

		respp = (STL_ERROR_PORT_COUNTERS_RSP *)((uint8 *)respp + size_port);
	}

}	// End of PmCopyErrorPortCounters()

// -------------------------------------------------------------------------
// PMA Outbound Mad processing
// -------------------------------------------------------------------------

static void PmSetMadAddressAndTid(Pm_t *pm, PmNode_t *pmnodep, cntxt_entry_t *entry)
{
	entry->mad.addrInfo.sl = pmnodep->sl;
	entry->mad.addrInfo.slid = pm->pm_slid;
	entry->mad.addrInfo.dlid =pmnodep->dlid;	// always set,used by redirect retry
	entry->mad.addrInfo.pkey = pmnodep->pkey;
	entry->mad.addrInfo.destqp = pmnodep->qpn;
	entry->mad.addrInfo.qkey = pmnodep->qkey;

	(void) mai_alloc_tid(hpma, MAD_CV_PERF, &entry->mad.base.tid);
	IB_LOG_DEBUG2LX("send MAD tid:", entry->mad.base.tid);
}

// returns NULL if unable to allocate an entry.  Since entry pool is pre-sized
// errors allocating the entry are unexpected.
static cntxt_entry_t *PmInitMad(Pm_t *pm, PmNode_t *pmnodep,
			uint8 method, uint32 attr, uint32 modifier)
{
    cntxt_entry_t *entry=NULL;

    if ((entry = cs_cntxt_get_nolock(NULL, &pm->Dispatcher.cntx, FALSE)) == NULL) {
        // could not get a context
        IB_LOG_ERROR0("Error allocating an PM async send/rcv context");
        //cntxt_cb(NULL, VSTATUS_BAD, cntxt_data, NULL);
		return NULL;	// let caller handle it, doing a callback could be recursive and cause a deep stack
	}

	STL_BasicMadInit(&entry->mad, MAD_CV_PERF, method, attr, modifier,
					pm->pm_slid, pmnodep->dlid, pmnodep->sl);

	PmSetMadAddressAndTid(pm, pmnodep, entry);
	return entry;
}

// caller must retire entry on failure
static __inline Status_t PmDispatcherSend(Pm_t *pm, cntxt_entry_t *entry)
{
	// Alternative for send failures is to ignore them and let timeout fire
	// (void)cs_cntxt_send_mad_nolock (entry, &pm->Dispatcher.cntx);
	// return VSTATUS_OK;
    return cs_cntxt_send_mad_nolock (entry, &pm->Dispatcher.cntx);
}

// on success a Get(ClassPortInfo) has been sent and Dispatcher.cntx is
// ready to process the response
// On failure, no request nor context entry is outstanding.
static Status_t PmSendGetClassPortInfo(Pm_t *pm, PmDispatcherNode_t *dispnode)
{
	cntxt_entry_t *entry;
	PmNode_t *pmnodep = dispnode->info.pmnodep;

	INCREMENT_PM_COUNTER(pmCounterGetClassPortInfo);
	entry = PmInitMad(pm, pmnodep, MMTHD_GET, STL_PM_ATTRIB_ID_CLASS_PORTINFO, 0);
	if (!entry)
		goto fail;

	entry->mad.datasize = 0;
	// fill in rest of entry->mad.data as needed

	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid);

	cs_cntxt_set_callback(entry, DispatchNodeCallback, dispnode);
	if (VSTATUS_OK ==  PmDispatcherSend(pm, entry))
		return VSTATUS_OK;
fail:
	PmFailNodeQuery(entry, dispnode, MMTHD_GET, STL_PM_ATTRIB_ID_CLASS_PORTINFO);
	return VSTATUS_NOT_FOUND;   // no work started
}

void setPortSelectMask(uint64 *selectMask, uint8 portNum, boolean clear)
{
	// selectMask points to an array of 4 uint64s representing sel mask

	int index;
	if (clear) memset(&selectMask[0], 0, 4 * sizeof(uint64));

	// find index
	index = (3 - (portNum / 64));

	selectMask[index] |= (uint64)(1) << (portNum - (64 * (3 - index)));

	return;
}

// on success a Set(PortCounters) has been sent and Dispatcher.cntx is
// ready to process the response
// On failure, no request nor context entry is outstanding.
static Status_t PmSendClearPortStatus(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	cntxt_entry_t *entry;
	PmNode_t *pmnodep = dispnode->info.pmnodep;
	STL_CLEAR_PORT_STATUS *p;

	// only called if we successfully got port counters
	DEBUG_ASSERT(disppacket->DispPorts[0].pPortImage->u.s.gotDataCntrs || disppacket->DispPorts[0].pPortImage->u.s.gotErrorCntrs);

	INCREMENT_PM_COUNTER(pmCounterSetClearPortStatus);
	entry = PmInitMad(pm, pmnodep, MMTHD_SET, STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS, 1 << 24);
	if (!entry)
		goto fail;

	entry->mad.datasize = sizeof(STL_CLEAR_PORT_STATUS);
	// fill in rest of entry->mad.data as needed
	p = (STL_CLEAR_PORT_STATUS *)&entry->mad.data;
	memset(p, 0, sizeof(STL_CLEAR_PORT_STATUS));
	memcpy(p->PortSelectMask, disppacket->PortSelectMask, sizeof(uint64)*4);
	p->CounterSelectMask.AsReg32 = disppacket->DispPorts[0].pPortImage->clearSelectMask.AsReg32;

	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x PortSelectMask[3] "FMT_U64" SelMask 0x%04x",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, p->PortSelectMask[3], p->CounterSelectMask.AsReg32);

	BSWAP_STL_CLEAR_PORT_STATUS_REQ(p);
	cs_cntxt_set_callback(entry, DispatchPacketCallback, disppacket);
	if (VSTATUS_OK ==  PmDispatcherSend(pm, entry))
		return VSTATUS_OK;
fail:
	PmFailPacketClear(entry, disppacket, MMTHD_SET, STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS);
	return VSTATUS_NOT_FOUND;   // no work started
}

// on success a Get(PortStatus) has been sent and Dispatcher.cntx is
// ready to process the response
// On failure, no request nor context entry is outstanding.
static Status_t PmSendGetPortStatus(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	cntxt_entry_t *entry;
	PmNode_t *pmnodep = dispnode->info.pmnodep;
	STL_PORT_STATUS_REQ *p;

	INCREMENT_PM_COUNTER(pmCounterGetPortStatus);
	entry = PmInitMad(pm, pmnodep, MMTHD_GET, STL_PM_ATTRIB_ID_PORT_STATUS, (disppacket->numPorts) << 24);
	if (!entry)
		goto fail;

	entry->mad.datasize = sizeof(STL_PORT_STATUS_REQ);
	// fill in rest of entry->mad.data as needed
	p = (STL_PORT_STATUS_REQ *)entry->mad.data;
	p->PortNumber = disppacket->DispPorts[0].pmportp->portNum;
	p->VLSelectMask = disppacket->VLSelectMask;
	memset(p->Reserved, 0, sizeof(p->Reserved));

	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x Port %d",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, disppacket->DispPorts[0].pmportp->portNum);

	BSWAP_STL_PORT_STATUS_REQ(p);
	cs_cntxt_set_callback(entry, DispatchPacketCallback, disppacket);
	if (VSTATUS_OK == PmDispatcherSend(pm, entry))
		return VSTATUS_OK;
fail:
	PmFailPacketQuery(entry, disppacket, MMTHD_GET, STL_PM_ATTRIB_ID_PORT_STATUS);
	return VSTATUS_NOT_FOUND;   // no work started

}   // End of PmSendGetPortStatus()

// on success a Get(DataPortCounters) has been sent and Dispatcher.cntx is
// ready to process the response
// On failure, no request nor context entry is outstanding.
static Status_t PmSendGetDataPortCounters(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	cntxt_entry_t *entry;
	PmNode_t *pmnodep = dispnode->info.pmnodep;
	STL_DATA_PORT_COUNTERS_REQ *p;

	INCREMENT_PM_COUNTER(pmCounterGetDataPortCounters);
	entry = PmInitMad(pm, pmnodep, MMTHD_GET, STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS, (disppacket->numPorts) << 24);
	if (!entry)
		goto fail;

	entry->mad.datasize = sizeof(STL_DATA_PORT_COUNTERS_REQ);
	// fill in rest of entry->mad.data as needed
	p = (STL_DATA_PORT_COUNTERS_REQ *)entry->mad.data;
	memcpy(p->PortSelectMask, disppacket->PortSelectMask, sizeof(uint64)*4);

	p->VLSelectMask = disppacket->VLSelectMask;

	p->res.s.LocalLinkIntegrityResolution = StlResolutionToShift(pm_config.resolution.LocalLinkIntegrity, RES_ADDER_LLI);
	p->res.s.LinkErrorRecoveryResolution = StlResolutionToShift(pm_config.resolution.LinkErrorRecovery, RES_ADDER_LER);

	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x PortSelectMask[3]: "FMT_U64,
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, p->PortSelectMask[3]);

	BSWAP_STL_DATA_PORT_COUNTERS_REQ(p);
	cs_cntxt_set_callback(entry, DispatchPacketCallback, disppacket);
	if (VSTATUS_OK == PmDispatcherSend(pm, entry))
		return VSTATUS_OK;
fail:
	PmFailPacketQuery(entry, disppacket, MMTHD_GET, STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS);
	return VSTATUS_NOT_FOUND;   // no work started

}   // End of PmSendGetDataPortCounters()

// on success a Get(ErrorPortCounters) has been sent and Dispatcher.cntx is
// ready to process the response
// On failure, no request nor context entry is outstanding.
static Status_t PmSendGetErrorPortCounters(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	cntxt_entry_t *entry;
	PmNode_t *pmnodep = dispnode->info.pmnodep;
	STL_ERROR_PORT_COUNTERS_REQ *p;

	INCREMENT_PM_COUNTER(pmCounterGetErrorPortCounters);
	entry = PmInitMad(pm, pmnodep, MMTHD_GET, STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS, (disppacket->numPorts) << 24);
	if (!entry)
		goto fail;

	entry->mad.datasize = sizeof(STL_ERROR_PORT_COUNTERS_REQ);
	// fill in rest of entry->mad.data as needed
	p = (STL_ERROR_PORT_COUNTERS_REQ *)entry->mad.data;
	memcpy(p->PortSelectMask, disppacket->PortSelectMask, sizeof(uint64)*4);

	p->VLSelectMask = disppacket->VLSelectMask;

	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x PortSelectMask[3]: "FMT_U64" VLSelectMask 0x%08x",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, p->PortSelectMask[3], p->VLSelectMask);

	BSWAP_STL_ERROR_PORT_COUNTERS_REQ(p);
	cs_cntxt_set_callback(entry, DispatchPacketCallback, disppacket);
	if (VSTATUS_OK == PmDispatcherSend(pm, entry))
		return VSTATUS_OK;
fail:
	PmFailPacketQuery(entry, disppacket, MMTHD_GET, STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS);
	return VSTATUS_NOT_FOUND;   // no work started

}   // End of PmSendGetErrorPortCounters()

static void DispatchPacketDone(Pm_t *pm, PmDispatcherPacket_t *disppacket)
{
	PmDispatcherNode_t *dispnode = disppacket->dispnode;

	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x",
		(int)sizeof(dispnode->info.pmnodep->nodeDesc.NodeString),
		dispnode->info.pmnodep->nodeDesc.NodeString,
		dispnode->info.pmnodep->guid, dispnode->info.pmnodep->dlid);

	DEBUG_ASSERT(dispnode->info.state != PM_DISP_NODE_DONE);
	// if dispport->failed,
	// 		then already incremented pm->Image[pm->SweepIndex].NoRespPorts
	dispnode->info.numOutstandingPackets--;
	//Cleanup DispPacket
	memset(disppacket->PortSelectMask, 0, sizeof(uint64)*4);
	disppacket->numPorts = 0;
	vs_pool_free(&pm_pool, (void *)disppacket->DispPorts);
	disppacket->DispPorts = NULL;
}
// -------------------------------------------------------------------------
// PM Packet Processing State Machine
// -------------------------------------------------------------------------

// cs_send_mad callback for all operations against multiple ports
static void DispatchPacketCallback(cntxt_entry_t *entry, Status_t status, void *data, Mai_t *mad)
{
	PmDispatcherPacket_t *disppacket = (PmDispatcherPacket_t *)data;
	PmDispatcherNode_t *dispnode = disppacket->dispnode;
	PmDispatcherPort_t *dispport;
	STL_CLEAR_PORT_STATUS *clearPortStatusMad;

	IB_LOG_DEBUG3_FMT( __func__,"%.*s Guid "FMT_U64" LID 0x%x NodeState %u Ports %u PortSelectMask[3] "FMT_U64,
		(int)sizeof(dispnode->info.pmnodep->nodeDesc.NodeString), dispnode->info.pmnodep->nodeDesc.NodeString,
		dispnode->info.pmnodep->guid, dispnode->info.pmnodep->dlid, dispnode->info.state,
		disppacket->numPorts, disppacket->PortSelectMask[3] );

	switch (dispnode->info.state) {
	case PM_DISP_NODE_GET_DATACOUNTERS:
		if (dispnode->info.pmnodep->nodeType == STL_NODE_FI) {
			if (status != VSTATUS_OK || mad == NULL) {
				PmFailPacketQuery(entry, disppacket, (mad ? mad->base.method : MMTHD_GET),
					(mad ? mad->base.aid : STL_PM_ATTRIB_ID_PORT_STATUS));
				goto nextpacket;	// PacketDone called on Fail
			} else if (mad->base.status != MAD_STATUS_SUCCESS) {
				PmMadFailPacketQuery(entry, mad, disppacket);
				goto nextpacket;	// PacketDone called on Fail
			} else if (mad->base.aid != STL_PM_ATTRIB_ID_PORT_STATUS) {
				PmMadAttrWrongPacketQuery(entry, mad, disppacket);
				goto nextpacket;    // PacketDone called on Fail
			} else {
				// process port status
				STL_PORT_STATUS_RSP *portStatusMad = (STL_PORT_STATUS_RSP *)&mad->data;
				BSWAP_STL_PORT_STATUS_RSP(portStatusMad);
				PmCopyPortStatus(portStatusMad, disppacket);

				dispport = &disppacket->DispPorts[0];
				dispport->pPortImage->u.s.gotDataCntrs = 1;
				dispport->pPortImage->u.s.gotErrorCntrs = 1;
			}
		}
		// DataPortCounters
		else if (dispnode->info.pmnodep->nodeType == STL_NODE_SW) {
			if (status != VSTATUS_OK || mad == NULL) {
				PmFailPacketQuery(entry, disppacket, (mad ? mad->base.method : MMTHD_GET),
					(mad ? mad->base.aid : STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS));
				goto nextpacket;    // PacketDone called on Fail
			} else if (mad->base.status != MAD_STATUS_SUCCESS) {
				PmMadFailPacketQuery(entry, mad, disppacket);
				goto nextpacket;    // PacketDone called on Fail
			} else if (mad->base.aid != STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS) {
				PmMadAttrWrongPacketQuery(entry, mad, disppacket);
				goto nextpacket;    // PacketDone called on Fail
			} else {
				STL_DATA_PORT_COUNTERS_RSP *dataPortCountersMad = (STL_DATA_PORT_COUNTERS_RSP *)&mad->data;
				BSWAP_STL_DATA_PORT_COUNTERS_RSP(dataPortCountersMad);
				if ((memcmp(dataPortCountersMad->PortSelectMask, disppacket->PortSelectMask, sizeof(uint64)*4) != 0) ||
					(dataPortCountersMad->VLSelectMask != disppacket->VLSelectMask)) {
					PmMadSelectWrongPacketQuery(entry, mad, disppacket);
					goto nextpacket;    // PacketDone called on Fail
				}
				// Process Data Port Counters MAD
				PmCopyDataPortCounters(dataPortCountersMad, disppacket);
			}
		}
		break;

	case PM_DISP_NODE_GET_ERRORCOUNTERS:
		// ErrorPortCounters
		if (status != VSTATUS_OK || mad == NULL) {
			PmFailPacketQuery(entry, disppacket, (mad ? mad->base.method : MMTHD_GET),
				(mad ? mad->base.aid : STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS));
			goto nextpacket;    // PacketDone called on Fail
		} else if (mad->base.status != MAD_STATUS_SUCCESS) {
			PmMadFailPacketQuery(entry, mad, disppacket);
			goto nextpacket;    // PacketDone called on Fail
		} else if (mad->base.aid != STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS) {
			PmMadAttrWrongPacketQuery(entry, mad, disppacket);
			goto nextpacket;    // PacketDone called on Fail
		} else {
			STL_ERROR_PORT_COUNTERS_RSP *errorPortCountersMad = (STL_ERROR_PORT_COUNTERS_RSP *)&mad->data;
			BSWAP_STL_ERROR_PORT_COUNTERS_RSP(errorPortCountersMad);
			if ((memcmp(errorPortCountersMad->PortSelectMask, disppacket->PortSelectMask, sizeof(uint64)*4) != 0) ||
				(errorPortCountersMad->VLSelectMask != disppacket->VLSelectMask)) {
				PmMadSelectWrongPacketQuery(entry, mad, disppacket);
				goto nextpacket;    // PacketDone called on Fail
			}
			// process error port counters
			PmCopyErrorPortCounters(errorPortCountersMad, disppacket);
		}
		break;

	case PM_DISP_NODE_CLR_PORT_STATUS:
		// we only get here if we successfully got counters
		// we already tabulated, so on failure we simply do not set the Clear
		// flags in the pCounters.  Hence next sweep will assume we didn't clear
		//DEBUG_ASSERT(dispport->pCounters->flags & PM_PORT_GOT_COUNTERS);
		if (status != VSTATUS_OK || mad == NULL) {
			PmFailPacketClear(entry, disppacket, (mad ? mad->base.method : MMTHD_GET),
				(mad ? mad->base.aid : STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS));
			goto nextpacket;
#if 0
		} else if (mad->base.status != MAD_STATUS_SUCCESS) {
			PmMadFailPacketClear(entry, mad, disppacket, "Set(ClearPortCounters)");
			goto nextpacket;	// PacketDone called on Fail
		} else if (mad->base.aid != PM_ATTRIB_ID_PORT_COUNTERS) {
			PmMadAttrWrongPacketClear(entry, mad, disppacket, "Set(ClearPortCounters)");
			goto nextpacket;	// PacketDone called on Fail
#endif
		} else {
			dispport = &disppacket->DispPorts[0];
			uint32 select = dispport->pPortImage->clearSelectMask.AsReg32;
			clearPortStatusMad = (STL_CLEAR_PORT_STATUS *)&mad->data;
			BSWAP_STL_CLEAR_PORT_STATUS_REQ(clearPortStatusMad);
			if ((memcmp(clearPortStatusMad->PortSelectMask, disppacket->PortSelectMask, sizeof(uint64)*4) != 0) ||
				(clearPortStatusMad->CounterSelectMask.AsReg32 != select)) {
				PmFailPacketClear(entry, disppacket, mad->base.method, mad->base.aid);
				goto nextpacket;
			}
			// process clear port counters
			//PmClearSelectedPortCounters(dispnode->pm, disppacket);
		}
		break;

	default:
		ASSERT(0);	// or log error
	}    // End of switch (dispnode->info.state)
	cs_cntxt_retire_nolock( entry, &dispnode->pm->Dispatcher.cntx  );

	DispatchPacketDone(dispnode->pm, disppacket);

nextpacket:

	if (VSTATUS_OK == DispatchNextPacket(dispnode->pm, dispnode, disppacket))
		return;

	if (dispnode->info.numOutstandingPackets)
		return;

	// all Ports Done
	if (VSTATUS_OK == DispatchNodeNextStep(dispnode->pm, dispnode->info.pmnodep, dispnode))
		return;

	// if NodeNextStep returns ! OK, then Node will be done
	DEBUG_ASSERT(dispnode->info.state == PM_DISP_NODE_DONE);

	// loops til finds a node or none left, wake main thread if all done
	(void)DispatchNextNode(dispnode->pm, dispnode);

}	// End of DispatchPacketCallback()

// -------------------------------------------------------------------------
// PM Node Processing State Machine
// -------------------------------------------------------------------------

// Compare function used by qsort() to sort active ports for data counters
// return n < 0 for a < b   
//        n = 0 for a == b  
//        n > 0 for a > b   
int PmDispatcherSwitchPortDataCompare(const void *a, const void *b)
{
	// First Level Compare: NULL check -> Decending 
	PmDispatcherSwitchPort_t *A = (PmDispatcherSwitchPort_t *)a;
	PmDispatcherSwitchPort_t *B = (PmDispatcherSwitchPort_t *)b;
    
	if (A->flags.s.Skip && B->flags.s.Skip)		return 0;
	if (A->flags.s.Skip)						return 1;
	if (B->flags.s.Skip)						return -1;
	if (pm_config.process_vl_counters) {
		// Second Level Compare: numVLs -> Decending
		if (A->NumVLs < B->NumVLs) return 1;
		if (A->NumVLs > B->NumVLs) return -1;

		// Third Level Compare: VLSelectMask -> Ascending
		if (A->VLSelectMask < B->VLSelectMask) return -1;
		if (A->VLSelectMask > B->VLSelectMask) return 1;
	}
	return 0;
}	// End of PmDispatcherSwitchPortDataCompare()

// Compare function used by qsort() to sort active ports for error counters
// return n < 0 for a < b   
//        n = 0 for a == b  
//        n > 0 for a > b   
int PmDispatcherSwitchPortErrorCompare(const void *a, const void *b)
{
	// First Level Compare: NULL check -> Decending 
	PmDispatcherSwitchPort_t *A = (PmDispatcherSwitchPort_t *)a;
	PmDispatcherSwitchPort_t *B = (PmDispatcherSwitchPort_t *)b;
    
	if (A->flags.s.Skip && B->flags.s.Skip)					return 0;
	if (A->flags.s.Skip)									return 1;
	if (B->flags.s.Skip)									return -1;
	if (A->flags.s.NeedsError && B->flags.s.NeedsError)		return 0;
    if (A->flags.s.NeedsError)							return -1;
    if (B->flags.s.NeedsError)							return 1;
    if (pm_config.process_vl_counters) {
        // Second Level Compare: numVLs -> Decending
        if (A->NumVLs < B->NumVLs) return 1;
        if (A->NumVLs > B->NumVLs) return -1;

		// Third Level Compare: VLSelectMask -> Ascending
        if (A->VLSelectMask < B->VLSelectMask) return -1;
        if (A->VLSelectMask > B->VLSelectMask) return 1;
    }
    return 0;
}	// End of PmDispatcherSwitchPortErrorCompare()

// Compare function used by qsort() to sort active ports for clear counters
// return n < 0 for a < b   
//        n = 0 for a == b  
//        n > 0 for a > b   
int PmDispatcherSwitchPortClearCompare(const void *a, const void *b)
{
	// First Level Compare: NULL check -> Decending 
	PmDispatcherSwitchPort_t *A = (PmDispatcherSwitchPort_t *)a;
	PmDispatcherSwitchPort_t *B = (PmDispatcherSwitchPort_t *)b;
    
	if (A->flags.s.Skip && B->flags.s.Skip)					return 0;
	if (A->flags.s.Skip)									return 1;
	if (B->flags.s.Skip)                                  return -1;
	if (A->flags.s.NeedsClear && B->flags.s.NeedsClear)		return 0;
    if (A->flags.s.NeedsClear)							return -1;
    if (B->flags.s.NeedsClear)							return 1;

    return 0;
}	// End of PmDispatcherSwitchPortClearCompare()

// Compare function used by qsort() to sort DispPorts array in disppacket struct
// return n < 0 for a < b
//        n = 0 for a == b
//        n > 0 for a > b
int PmDispatcherPortNumCompare(const void *a, const void *b)
{
	// First Level Compare: NULL check -> Decending
	PmDispatcherPort_t *A = (PmDispatcherPort_t *)a;
	PmDispatcherPort_t *B = (PmDispatcherPort_t *)b;

	return (A->pmportp->portNum - B->pmportp->portNum);
}	// End of PmDispatcherPortNumCompare()

// start processing of a node
// returns OK if a node was dispatched, returns NOT_FOUND if none dispatched
static Status_t DispatchNode(Pm_t *pm, PmNode_t *pmnodep, PmDispatcherNode_t *dispnode)
{
	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid);

	memset(&dispnode->info, 0, sizeof(dispnode->info));
	dispnode->info.pmnodep = pmnodep;
	dispnode->info.numPorts = ((pmnodep->nodeType == STL_NODE_SW) ? pmnodep->numPorts + 1 : 1);
	pm->Dispatcher.numOutstandingNodes++;
	return DispatchNodeNextStep(pm, pmnodep, dispnode);
}

// Continue processing of a node, use when 1st start processing and on
// callbacks to advance to next state/operation on the node
// return:
// 	OK - next packet inflight
// 	VSTATUS_NOT_FOUND - no more work needed/possible, NodeDone called
static Status_t DispatchNodeNextStep(Pm_t *pm, PmNode_t *pmnodep, PmDispatcherNode_t *dispnode)
{
	uint8 ret;
	int i, j;
	size_t size;
	Status_t status;
    uint32 VlSelectMask;
	uint32 clearCounterSelect;
	PmPort_t *pmportp;

	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x NodeState %u",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, dispnode->info.state);

	switch (dispnode->info.state) {
	case PM_DISP_NODE_NONE:
		if (! pmnodep->u.s.PmaGotClassPortInfo) {
			dispnode->info.state = PM_DISP_NODE_CLASS_INFO;
			return PmSendGetClassPortInfo(pm, dispnode);
		}
	case PM_DISP_NODE_CLASS_INFO:
		dispnode->info.state = PM_DISP_NODE_GET_DATACOUNTERS;
        // Allocate activePorts[]
		size = sizeof(PmDispatcherSwitchPort_t) * dispnode->info.numPorts;

        status = vs_pool_alloc(&pm_pool, size, (void *)&dispnode->info.activePorts);
        if (status != VSTATUS_OK || !dispnode->info.activePorts) {
            IB_LOG_ERROR_FMT(__func__, "PM: Failed to allocate Dispatcher activePorts for Node:%.*s status:%u",
                (int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString, status);
            return VSTATUS_NOT_FOUND;
        }
        memset(dispnode->info.activePorts, 0, size);
 
		// Initialize activePorts[] for DataPortCounters
		for (i = 0; i < dispnode->info.numPorts; i++) { 

			if (pmnodep->nodeType == STL_NODE_SW) {
				pmportp = pmnodep->up.swPorts[i];
			} else {
				pmportp = pmnodep->up.caPortp;
			}
			if ( !(pmportp) || !(pmportp->Image[pm->SweepIndex].u.s.active) /* || !(pmportp->u.s.PmaAvoid) */) {
				
				dispnode->info.activePorts[i].portNum = 0xFF;
				dispnode->info.activePorts[i].flags.s.Skip = 1;
				continue;
			}
			dispnode->info.activePorts[i].portNum = pmportp->portNum;
			dispnode->info.activePorts[i].flags.AsReg8 = 0;
		
			if (pm_config.process_vl_counters) {
				dispnode->info.activePorts[i].VLSelectMask = pmportp->Image[pm->SweepIndex].vlSelectMask;
				for ( j = 0, VlSelectMask = dispnode->info.activePorts[i].VLSelectMask;
						j < STL_MAX_VLS; j++, VlSelectMask >>= 1 ) 
					if (VlSelectMask & 0x1)
						dispnode->info.activePorts[i].NumVLs++;
			}
		}

		dispnode->info.nextPort = dispnode->info.activePorts;
        
		// Sort activePorts[], ordering by number of VLs and VLSelectMask;
		//  shift non-active ports to the end
		if (dispnode->info.numPorts > 1) {
			qsort( dispnode->info.activePorts, (size_t)dispnode->info.numPorts,
				sizeof(PmDispatcherSwitchPort_t), PmDispatcherSwitchPortDataCompare );
		}

		for (i = 0; i < dispnode->info.numPorts; i++)
			IB_LOG_DEBUG4_FMT( __func__,"%.*s Post-qsort: activePorts[%d]: numVLs %u portNum %u flags 0x%02x VLSelect 0x%08x", 
				(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString, i,
				dispnode->info.activePorts[i].NumVLs, dispnode->info.activePorts[i].portNum, 
				dispnode->info.activePorts[i].flags.AsReg8, dispnode->info.activePorts[i].VLSelectMask);
        
		ret = DispatcherStartSweepAllPorts(pm, dispnode);

		if (ret)
			return VSTATUS_OK;
		// fallthrough, no ports with packets inflight

	case PM_DISP_NODE_GET_DATACOUNTERS:

		if (pmnodep->nodeType == STL_NODE_SW) {
			if (dispnode->info.u.s.needError) {
				dispnode->info.state = PM_DISP_NODE_GET_ERRORCOUNTERS;
				// Initialize activePorts[] for ErrorPortCounters
				if (!dispnode->info.activePorts) {
					IB_LOG_ERROR_FMT(__func__, "PM: No Dispatcher activePorts for Node:%.*s",
						(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString );
					return VSTATUS_NOT_FOUND;
				}
				// Sort activePorts[], ordering by number of VLs and VLSelectMask;
				//  shift non-active ports and ports not needing error counters query to the end
				if (dispnode->info.numPorts > 1) {
					qsort( dispnode->info.activePorts, (size_t)dispnode->info.numPorts,
						   sizeof(PmDispatcherSwitchPort_t), PmDispatcherSwitchPortErrorCompare );
				}
				for (i = 0; i < dispnode->info.numPorts && dispnode->info.activePorts[i].flags.s.NeedsError; i++) {
					dispnode->info.activePorts[i].flags.s.IsDispatched = 0;
				}
				ret = DispatcherStartSweepAllPorts(pm, dispnode);

				if (ret)
					return VSTATUS_OK;
			}
		}
		// fallthrough, no ports with packets inflight
		
	case PM_DISP_NODE_GET_ERRORCOUNTERS:
		// Tabulate ports and collect information on needed clears
		dispnode->info.u.s.needClearSome = 0;
		dispnode->info.u.s.canClearAll = 1;
		dispnode->info.clearCounterSelect = clearCounterSelect = 0;
		for (i = 0; i < dispnode->info.numPorts && !dispnode->info.activePorts[i].flags.s.Skip; i++) {
			if (pmnodep->nodeType == STL_NODE_SW) {
				pmportp = pmnodep->up.swPorts[dispnode->info.activePorts[i].portNum];
			} else {
				pmportp = pmnodep->up.caPortp;
			}
			
			if (PmTabulatePort(pm, pmportp, pm->SweepIndex, &clearCounterSelect)) {
				if (dispnode->info.clearCounterSelect && (clearCounterSelect != dispnode->info.clearCounterSelect))
					dispnode->info.u.s.canClearAll = 0;

				dispnode->info.activePorts[i].flags.s.NeedsClear = 1;
				dispnode->info.u.s.needClearSome = 1;
			}
		}

		if (dispnode->info.u.s.needClearSome) {
			dispnode->info.state = PM_DISP_NODE_CLR_PORT_STATUS;
			// Initialize activePorts[] for ClearPortCounters
			if (!dispnode->info.activePorts) {
				IB_LOG_ERROR_FMT(__func__, "PM: No Dispatcher activePorts for Node:%.*s",
					(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString );
				return VSTATUS_NOT_FOUND;
			}
			// Sort activePorts[]
			//  shift non-active ports and ports not needing clear to the end
			if (dispnode->info.numPorts > 1) {
				qsort( dispnode->info.activePorts, (size_t)dispnode->info.numPorts,
					   sizeof(PmDispatcherSwitchPort_t), PmDispatcherSwitchPortClearCompare );
			}
			for (i = 0; i < dispnode->info.numPorts && dispnode->info.activePorts[i].flags.s.NeedsClear; i++) {
				dispnode->info.activePorts[i].flags.s.IsDispatched = 0;
			}
			ret = DispatcherStartSweepAllPorts(pm, dispnode);
				
			if (ret)
				return VSTATUS_OK;
		}
		// fallthrough, no ports with packets inflight
        
	case PM_DISP_NODE_CLR_PORT_STATUS:
		DispatchNodeDone(pm, dispnode);
		return VSTATUS_NOT_FOUND;

	case PM_DISP_NODE_DONE:
		DEBUG_ASSERT(0);	// unexpected state
		return VSTATUS_NOT_FOUND;	// nothing else to do for this node

	default:
		DEBUG_ASSERT(0);	// unexpected state
		DispatchNodeDone(pm, dispnode);
		return VSTATUS_NOT_FOUND;
	}

}	// End of DispatchNodeNextStep()

// returns number of ports with packets in flight
// if 0 then caller need not wait, no packets in flight
static uint8 DispatcherStartSweepAllPorts(Pm_t *pm, PmDispatcherNode_t *dispnode)
{
	uint8 slot;
	Status_t status;
#ifndef VIEO_TRACE_DISABLED
	PmNode_t *pmnodep = dispnode->info.pmnodep;

	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x NodeState %u",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, dispnode->info.state);
#endif

    for (slot = 0; slot < pm_config.PmaBatchSize; ) {
	// reset numVLs back to 0 in case it has changed since the last PM sweep
	dispnode->DispPackets[slot].numVLs = 0;
        status = DispatchNextPacket(pm, dispnode, &dispnode->DispPackets[slot]);
		switch (status) {
		case VSTATUS_OK:
            slot++;
			continue;
		case VSTATUS_NOPORT:
			return slot;
		case VSTATUS_NOT_FOUND:
			break;
		default:
			IB_LOG_ERROR_FMT(__func__, "PM: Unknown Return Status (%u) from DispatchNextPacket() Node:%.*s", status,
				(int)sizeof(dispnode->info.pmnodep->nodeDesc.NodeString), dispnode->info.pmnodep->nodeDesc.NodeString);
		}
    }

	return slot;

}	// End of DispatcherStartSweepAllPorts()

// returns OK if a packet was dispatched, returns NOT_FOUND if none dispatched
static Status_t DispatchNextPacket(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	PmNode_t *pmnodep = dispnode->info.pmnodep;
    PmPort_t *pmportp;

	if (pm_shutdown || g_pmEngineState != PM_ENGINE_STARTED) {
		IB_LOG_INFO0("PM Engine shut down requested");
		dispnode->info.u.s.needClearSome = FALSE;
		goto abort;
	}
      
	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x Node State %u",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, dispnode->info.state);

	dispnode->info.nextPort = dispnode->info.activePorts; //reset pointer to the top to grab and skipped over ports

	while ( dispnode->info.nextPort->flags.s.Skip != 1 ) {
		if (pmnodep->nodeType == STL_NODE_SW)
			pmportp = pmnodep->up.swPorts[dispnode->info.nextPort->portNum];
		else
			pmportp = pmnodep->up.caPortp;

		//FIXME: ACG - Should this be added to parent function like above and removed?
		// Commented out because it will cause Inf loop when true.
		// I am not removing this code, because It may be usefull when multiport queries fail.
		//if (pmportp->Image[pm->SweepIndex].u.s.queryStatus != PM_QUERY_STATUS_OK) 
		//	continue;
        
		switch(MergePortIntoPacket(pm, dispnode, pmportp, disppacket))
        {
		case PM_DISP_SW_MERGE_NOMERGE:
			if (disppacket->numPorts == 0) {
				if (dispnode->info.nextPort != &dispnode->info.activePorts[dispnode->info.numPorts-1]) {
					dispnode->info.nextPort++;
					continue;
				}
				else goto doneNode;

			}
        case PM_DISP_SW_MERGE_CONTINUE:
			if (dispnode->info.nextPort != &dispnode->info.activePorts[dispnode->info.numPorts-1]) {
				dispnode->info.nextPort++;
				continue;
			}
		case PM_DISP_SW_MERGE_DONE:
			if (VSTATUS_OK == DispatchPacketNextStep(pm, dispnode, disppacket))
                return VSTATUS_OK;
            goto abort; 
        case PM_DISP_SW_MERGE_ERROR:
				IB_LOG_ERROR_FMT( __func__,"PM: MergePortIntoPacket() Failure" );
            goto abort;   
        default:
            goto abort;
        }
	} // End of While()
	
	if (disppacket->numPorts) {
		if (VSTATUS_OK == DispatchPacketNextStep(pm, dispnode, disppacket))
                return VSTATUS_OK;
	}
doneNode:
	return VSTATUS_NOPORT;
abort:
	return VSTATUS_NOT_FOUND;
}
int MergePortIntoPacket(Pm_t *pm, PmDispatcherNode_t *dispnode, PmPort_t *pmportp, PmDispatcherPacket_t *disppacket)
{
                           // TOTAL - (SM LID-Routed packet header + Mkey) = PM PAYLOAD = 2024
    size_t sizeRemaining = STL_MAX_PAYLOAD_SMP_LR + sizeof(uint64); 
    size_t sizePortInMad;
	Status_t status;
	PmPort_t *pmFirstPort;

	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x Node State %u PortSelectMask[3] "FMT_U64,
		(int)sizeof(dispnode->info.pmnodep->nodeDesc.NodeString), dispnode->info.pmnodep->nodeDesc.NodeString,
		dispnode->info.pmnodep->guid, dispnode->info.pmnodep->dlid, 
		dispnode->info.state, disppacket->PortSelectMask[3]);

    switch (dispnode->info.state) {
	case PM_DISP_NODE_GET_DATACOUNTERS:
		if (dispnode->info.pmnodep->nodeType == STL_NODE_SW) 
			sizeRemaining -= (sizeof(STL_DATA_PORT_COUNTERS_RSP) - sizeof(struct _port_dpctrs));
        break;
	case PM_DISP_NODE_GET_ERRORCOUNTERS:
        sizeRemaining -= (sizeof(STL_ERROR_PORT_COUNTERS_RSP) - sizeof(struct _port_epctrs));
        break;
	case PM_DISP_NODE_CLR_PORT_STATUS:   
        sizeRemaining -= sizeof(STL_CLEAR_PORT_STATUS);
        break;
	default:
        //LOG ERROR 
        return PM_DISP_SW_MERGE_ERROR;
    }

    sizePortInMad = CalculatePortInPacket(dispnode, disppacket);
    if(sizePortInMad == (size_t)(-1)) 
        return PM_DISP_SW_MERGE_ERROR;  
    
	if (disppacket->numPorts == 0) {
		switch (dispnode->info.state) {
		case PM_DISP_NODE_GET_DATACOUNTERS:
			if (dispnode->info.pmnodep->nodeType == STL_NODE_SW) 
				status = vs_pool_alloc(&pm_pool, sizeof(PmDispatcherPort_t) * (sizeRemaining/sizePortInMad), (void *)&disppacket->DispPorts);
			else
				status = vs_pool_alloc(&pm_pool, sizeof(PmDispatcherPort_t), (void *)&disppacket->DispPorts);
			break;
		case PM_DISP_NODE_GET_ERRORCOUNTERS:
			status = vs_pool_alloc(&pm_pool, sizeof(PmDispatcherPort_t) * (sizeRemaining/sizePortInMad), (void *)&disppacket->DispPorts);
			break;
		case PM_DISP_NODE_CLR_PORT_STATUS:   
			status = vs_pool_alloc(&pm_pool, sizeof(PmDispatcherPort_t), (void *)&disppacket->DispPorts);
			break;
		default:
			return PM_DISP_SW_MERGE_ERROR;
		}
		if (status != VSTATUS_OK) {
			return PM_DISP_SW_MERGE_ERROR;
		}
	}
    sizeRemaining -= sizePortInMad * disppacket->numPorts;
    
    // Check if packet is complete;
    if (sizeRemaining > sizePortInMad) {                                                
        // Check if port was already dispatched
        if (dispnode->info.nextPort->flags.s.IsDispatched)
			goto nomerge;
        //-Check if port is marked as DoNotMerge
        if (dispnode->info.nextPort->flags.s.DoNotMerge) {                          
            //--if Packet is not empty skip this port.  
            disppacket->DispPorts[0].pmportp = pmportp;
            disppacket->DispPorts[0].dispNodeSwPort = dispnode->info.nextPort;
            disppacket->DispPorts[0].dispnode = dispnode;
            disppacket->DispPorts[0].pPortImage = &pmportp->Image[pm->SweepIndex];
			disppacket->DispPorts[0].pPortImagePrev =
				(pm->LastSweepIndex != PM_IMAGE_INDEX_INVALID ? &pmportp->Image[pm->LastSweepIndex] : NULL);
            disppacket->DispPorts[0].dispNodeSwPort->flags.s.IsDispatched = 1;
            disppacket->numPorts = 1;
            setPortSelectMask(disppacket->PortSelectMask, pmportp->portNum, TRUE);
            disppacket->VLSelectMask = dispnode->info.nextPort->VLSelectMask;
            //--if packet is empty copy single packet and mark packet as full

            return PM_DISP_SW_MERGE_DONE;              
        }  
        if (disppacket->numPorts) {
            if (dispnode->info.state == PM_DISP_NODE_CLR_PORT_STATUS) {
				pmFirstPort = disppacket->DispPorts[0].pmportp;
                if (pmportp->Image[pm->SweepIndex].clearSelectMask.AsReg32 ==
						pmFirstPort->Image[pm->SweepIndex].clearSelectMask.AsReg32) {
                    disppacket->numPorts++;
					setPortSelectMask(disppacket->PortSelectMask, pmportp->portNum, FALSE);
					dispnode->info.nextPort->flags.s.IsDispatched = 1;
                }
                else return PM_DISP_SW_MERGE_NOMERGE; 
			}
            else {
                //check to make sure port can be combined into packet. check for similar VL Mask.
                if( ( dispnode->info.nextPort->VLSelectMask | disppacket->VLSelectMask)
                        == disppacket->VLSelectMask) {
                    disppacket->DispPorts[disppacket->numPorts].pmportp = pmportp;
                    disppacket->DispPorts[disppacket->numPorts].dispNodeSwPort = dispnode->info.nextPort;
                    disppacket->DispPorts[disppacket->numPorts].dispnode = dispnode;
                    disppacket->DispPorts[disppacket->numPorts].pPortImage = &pmportp->Image[pm->SweepIndex];
					disppacket->DispPorts[disppacket->numPorts].pPortImagePrev =
						(pm->LastSweepIndex != PM_IMAGE_INDEX_INVALID ? &pmportp->Image[pm->LastSweepIndex] : NULL);
					disppacket->DispPorts[disppacket->numPorts].dispNodeSwPort->flags.s.IsDispatched = 1;
                    disppacket->numPorts++;
					setPortSelectMask(disppacket->PortSelectMask, pmportp->portNum, FALSE);
                }
                else return PM_DISP_SW_MERGE_NOMERGE; 
            }
        }
        else {
            disppacket->DispPorts[0].pmportp = pmportp;
            disppacket->DispPorts[0].dispNodeSwPort = dispnode->info.nextPort;
            disppacket->DispPorts[0].dispnode = dispnode;
            disppacket->DispPorts[0].pPortImage = &pmportp->Image[pm->SweepIndex];
			disppacket->DispPorts[0].pPortImagePrev =
				(pm->LastSweepIndex != PM_IMAGE_INDEX_INVALID ? &pmportp->Image[pm->LastSweepIndex] : NULL);
            disppacket->DispPorts[0].dispNodeSwPort->flags.s.IsDispatched = 1;
            disppacket->numPorts = 1;
			setPortSelectMask(disppacket->PortSelectMask, pmportp->portNum, TRUE);
			disppacket->VLSelectMask = dispnode->info.nextPort->VLSelectMask;
        }
        return PM_DISP_SW_MERGE_CONTINUE; 
    }
    else return PM_DISP_SW_MERGE_DONE;

    return PM_DISP_SW_MERGE_ERROR; //-Sanity Check

nomerge:
	if (disppacket->DispPorts && !disppacket->numPorts) {
		vs_pool_free(&pm_pool, disppacket->DispPorts);
		disppacket->DispPorts = NULL;
	}
	return PM_DISP_SW_MERGE_NOMERGE;

}	// End of MergePortIntoPacket()

size_t CalculatePortInPacket(PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	uint8   NumVLs = 0;

	switch (dispnode->info.state) {
	case PM_DISP_NODE_NONE:                      return(-1);
	case PM_DISP_NODE_CLASS_INFO:                return(-1);
	case PM_DISP_NODE_GET_DATACOUNTERS:
		if (pm_config.process_vl_counters) {
			if (!disppacket->numVLs)
				disppacket->numVLs = dispnode->info.nextPort->NumVLs;
			NumVLs = disppacket->numVLs;
		}
		if (dispnode->info.pmnodep->nodeType == STL_NODE_SW)
			return(sizeof(struct _port_dpctrs) + (NumVLs - 1) * sizeof(struct _vls_dpctrs));
		else
			return(sizeof(STL_PORT_STATUS_RSP) + (NumVLs - 1) * sizeof(struct _vls_pctrs));
	case PM_DISP_NODE_GET_ERRORCOUNTERS:
		if (pm_config.process_vl_counters) {
			if (!disppacket->numVLs)
				disppacket->numVLs = dispnode->info.nextPort->NumVLs;
			NumVLs = disppacket->numVLs;
		}
		return(sizeof(struct _port_epctrs) + (NumVLs - 1) * sizeof(struct _vls_epctrs));
	case PM_DISP_NODE_CLR_PORT_STATUS:           return(0);
	case PM_DISP_NODE_DONE:                      return(-1);
	}
	return(-1);
}
static Status_t DispatchPacketNextStep(Pm_t *pm, PmDispatcherNode_t *dispnode, PmDispatcherPacket_t *disppacket)
{
	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x Node State %u PortSelectMask[3] "FMT_U64,
		(int)sizeof(dispnode->info.pmnodep->nodeDesc.NodeString),
		dispnode->info.pmnodep->nodeDesc.NodeString,
		dispnode->info.pmnodep->guid, dispnode->info.pmnodep->dlid,
		dispnode->info.state, disppacket->PortSelectMask[3]);
                
    DEBUG_ASSERT(dispnode->info.state != PM_DISP_NODE_DONE);
	if (dispnode->info.state == PM_DISP_NODE_DONE)	// safety net
		return VSTATUS_NOT_FOUND;	// nothing else to do for this port        
    dispnode->info.numOutstandingPackets++;    

	if (disppacket->numPorts > 1 && dispnode->info.state != PM_DISP_NODE_CLR_PORT_STATUS) {
		// Sort DispPorts by Port Number, so that it will match response MAD order
		qsort(disppacket->DispPorts, (size_t)disppacket->numPorts,
			sizeof(PmDispatcherPort_t), PmDispatcherPortNumCompare);
	}
    switch (dispnode->info.state) {
	case PM_DISP_NODE_GET_DATACOUNTERS:
		if ( dispnode->info.pmnodep->nodeType == STL_NODE_FI ) {
			return PmSendGetPortStatus(pm, dispnode, disppacket);
		} 
		else {
			dispnode->info.state = PM_DISP_NODE_GET_DATACOUNTERS;
			return PmSendGetDataPortCounters(pm, dispnode, disppacket);
		}
	
	case PM_DISP_NODE_GET_ERRORCOUNTERS:
		return PmSendGetErrorPortCounters(pm, dispnode, disppacket);
		
	case PM_DISP_NODE_CLR_PORT_STATUS:  
		return PmSendClearPortStatus(pm, dispnode, disppacket);
	
	default:
		DEBUG_ASSERT(0);	// unexpected state
		DispatchPacketDone(pm, disppacket);
		return VSTATUS_NOT_FOUND;
	}
}	// End of DispatchPacketNextStep()

// cs_send_mad callback for all operations against a whole node
// (Get(ClassPortInfo), Clear all Port Counters)
static void DispatchNodeCallback(cntxt_entry_t *entry, Status_t status, void *data, Mai_t *mad)
{
	PmDispatcherNode_t *dispnode = (PmDispatcherNode_t*)data;
	PmNode_t *pmnodep = dispnode->info.pmnodep;

	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x Node State %u ",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->guid, pmnodep->dlid, dispnode->info.state);

	switch (dispnode->info.state) {
	case PM_DISP_NODE_CLASS_INFO:
		if (status != VSTATUS_OK) {
			PmFailNodeQuery(entry, dispnode, MMTHD_GET, STL_PM_ATTRIB_ID_CLASS_PORTINFO);
			goto nextnode;	// NodeDone called on Fail
		} else if (mad->base.status != MAD_STATUS_SUCCESS) {
			PmMadFailNodeQuery(entry, mad, dispnode);
			goto nextnode;	// NodeDone called on Fail
		} else if (mad->base.aid != STL_PM_ATTRIB_ID_CLASS_PORTINFO) {
			PmMadAttrWrongNodeQuery(entry, mad, dispnode);
			goto nextnode;	// NodeDone called on Fail
		} else if (FSUCCESS != ProcessPmaClassPortInfo(pmnodep,(STL_CLASS_PORT_INFO*)mad->data) ){
			PmFailNodeQuery(entry, dispnode, MMTHD_GET, STL_PM_ATTRIB_ID_CLASS_PORTINFO);
			goto nextnode;	// NodeDone called on Fail
		}
		break;
	default:
		ASSERT(0);	// or log error
	}
	cs_cntxt_retire_nolock( entry, &dispnode->pm->Dispatcher.cntx  );

	if (VSTATUS_OK == DispatchNodeNextStep(dispnode->pm, pmnodep, dispnode))
		return;
	// if NodeNextStep returns ! OK, then Node will be done

nextnode:
	// fails above which goto nextport will also have called done
	DEBUG_ASSERT(dispnode->info.state == PM_DISP_NODE_DONE);

	// loops til finds a node or none left, wake main thread if all done
	(void)DispatchNextNode(dispnode->pm, dispnode);

}	// End of DispatchNodeCallback()

// given node is completed
static void DispatchNodeDone(Pm_t *pm, PmDispatcherNode_t *dispnode)
{
	IB_LOG_DEBUG3_FMT(__func__,"%.*s Guid "FMT_U64" LID 0x%x",
		(int)sizeof(dispnode->info.pmnodep->nodeDesc.NodeString),
		dispnode->info.pmnodep->nodeDesc.NodeString,
		dispnode->info.pmnodep->guid, dispnode->info.pmnodep->dlid);

	DEBUG_ASSERT(dispnode->info.state != PM_DISP_NODE_DONE);
	// we handle this once when all ports done, hence we will only increment
	// once even if multiple ports fail in the same node
	if (dispnode->info.u.s.failed) {
		pm->Image[pm->SweepIndex].NoRespNodes++;
		INCREMENT_PM_COUNTER(pmCounterPmNoRespNodes);
	}
	dispnode->info.state = PM_DISP_NODE_DONE;
        if(dispnode->info.activePorts) {
            vs_pool_free(&pm_pool, dispnode->info.activePorts);
            dispnode->info.activePorts = NULL;
        }

	pm->Dispatcher.numOutstandingNodes--;
}

// -------------------------------------------------------------------------
// PM Sweep Main Loop
// -------------------------------------------------------------------------

// returns number of nodes started,
// if 0 then caller need not wait, nothing to do
// caller should check for EngineShutdown before calling
static Status_t DispatcherStartSweepAllNodes(Pm_t *pm)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
	PmDispatcherNode_t *dispnode;
	uint16 slot;

	cs_cntxt_lock(&pm->Dispatcher.cntx);
	// initialize Dispatcher for a new sweep
	pm->Dispatcher.nextLid = 1;
	pm->Dispatcher.numOutstandingNodes = 0;
	pm->Dispatcher.postedEvent = 0;
	for (slot=0; slot < pm_config.MaxParallelNodes; ++slot) {
		pm->Dispatcher.DispNodes[slot].info.pmnodep = NULL;
		pm->Dispatcher.DispNodes[slot].info.state = PM_DISP_NODE_NONE;
	}
	
	for (slot = 0,dispnode = &pm->Dispatcher.DispNodes[slot];
		slot < pm_config.MaxParallelNodes && pm->Dispatcher.nextLid <=pmimagep->maxLid;
		) {
		if (VSTATUS_OK == DispatchNextNode(pm, dispnode))
			dispnode++,slot++;
	}
	cs_cntxt_unlock(&pm->Dispatcher.cntx);
	return slot;	// number of nodes started
}

// returns OK if a node was dispatched, returns NOT_FOUND if none dispatched
static Status_t DispatchNextNode(Pm_t *pm, PmDispatcherNode_t *dispnode)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];

	if (pm_shutdown || g_pmEngineState != PM_ENGINE_STARTED) {
		IB_LOG_INFO0("PM Engine shut down requested");
		goto abort;
	}
	while (pm->Dispatcher.nextLid <= pmimagep->maxLid) {
		PmNode_t *pmnodep = pmimagep->LidMap[pm->Dispatcher.nextLid];
		pm->Dispatcher.nextLid++;
		if (! pmnodep)
			continue;
		// we only keep active LIDed ports in LidMap
		DEBUG_ASSERT(pm_node_lided_port(pmnodep)->Image[pm->SweepIndex].u.s.active);

		if (pmnodep->u.s.PmaAvoid) { // ! NodeHasPma(nodep)
			PmSkipNode(pm, pmnodep); // No PMA
			continue;
		}
		if (VSTATUS_OK == DispatchNode(pm, pmnodep, dispnode))
			return VSTATUS_OK;
	}
abort:
	// be sure we only post event once per sweep
	if (! pm->Dispatcher.numOutstandingNodes && ! pm->Dispatcher.postedEvent) {
		pm->Dispatcher.postedEvent = 1;
		vs_event_post(&pm->Dispatcher.sweepDone, VEVENT_WAKE_ONE, (Eventset_t)1);
	}
	return VSTATUS_NOT_FOUND;
}

FSTATUS PmSweepAllPortCounters(Pm_t *pm)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
	Eventset_t events;
	Status_t rc;

	IB_LOG_INFO0("START Sweeping All Port Counters...");
	pmimagep->NoRespNodes = pmimagep->NoRespPorts = 0;
	pmimagep->SkippedNodes = pmimagep->SkippedPorts = 0;
	pmimagep->UnexpectedClearPorts = 0;
	pmimagep->DowngradedPorts = 0;
	(void)PmClearAllNodes(pm);	// clear link based counts

	(void)DispatcherStartSweepAllNodes(pm);
	do {
		rc = vs_event_wait(&pm->Dispatcher.sweepDone, VTIMER_1S, (Eventset_t)1, &events);
	} while (rc == VSTATUS_TIMEOUT);

	IB_LOG_INFO0("DONE Sweeping All Port Counters");
	if (pmimagep->NoRespPorts)
		IB_LOG_WARN_FMT(__func__, "Unable to get %u Ports on %u Nodes", pmimagep->NoRespPorts, pmimagep->NoRespNodes);
	if (pmimagep->UnexpectedClearPorts)
		IB_LOG_WARN_FMT(__func__, "%u Ports were unexpectedly cleared", pmimagep->UnexpectedClearPorts);
	if (pmimagep->DowngradedPorts)
		IB_LOG_INFO_FMT(__func__, "%u Ports were Downgraded", pmimagep->DowngradedPorts);
	if (pm_shutdown || g_pmEngineState != PM_ENGINE_STARTED)
		return FNOT_DONE;
	return FSUCCESS;
}

// -------------------------------------------------------------------------
// PM Dispatch Initialization
// -------------------------------------------------------------------------

Status_t PmDispatcherInit(Pm_t *pm)
{
	struct PmDispatcher_s *disp = &pm->Dispatcher;
	PmDispatcherNode_t *dispnode;
	Status_t status;
	uint32 size;
	uint16 slot;
	uint64_t timeout=0;

	memset(disp, 0, sizeof(*disp));

	disp->cntx.hashTableDepth = CNTXT_HASH_TABLE_DEPTH;
	disp->cntx.poolSize = pm_config.MaxParallelNodes * pm_config.PmaBatchSize;
	disp->cntx.maxRetries = pm_config.MaxRetries;
	disp->cntx.ibHandle = hpma;
	disp->cntx.resp_queue = NULL;	// no need for a resp queue
	disp->cntx.totalTimeout = (pm_config.RcvWaitInterval * pm_config.MaxRetries * 1000);
	if (pm_config.MinRcvWaitInterval) {
		timeout = pm_config.MinRcvWaitInterval * 1000;
		disp->cntx.MinRespTimeout = timeout;
	} else {
		timeout = pm_config.RcvWaitInterval * 1000;
		disp->cntx.MinRespTimeout = 0;
	}
	disp->cntx.errorOnSendFail = 1;
#ifdef IB_STACK_OPENIB
	// for openib we let umad do the timeouts.  Hence we add 1 second to
	// the timeout as a safety net just in case umad loses our response.
	disp->cntx.timeoutAdder = VTIMER_1S;
#endif
	status = cs_cntxt_instance_init(&pm_pool, &disp->cntx, timeout);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to create Dispatcher Context rc:", status);
		goto fail;
	}

	status = vs_event_create(&disp->sweepDone, (unsigned char*)"PM Sweep Done",
					(Eventset_t)0);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to create Dispatcher Event rc:", status);
		goto freecontext;
	}

	size = sizeof(PmDispatcherNode_t)*pm_config.MaxParallelNodes;
	status = vs_pool_alloc(&pm_pool, size, (void*)&disp->DispNodes);
	if (status != VSTATUS_OK || !disp->DispNodes) {
		IB_LOG_ERRORRC("Failed to allocate Dispatcher Nodes rc:", status);
		goto freeevent;
	}
	memset(disp->DispNodes, 0, size);

	for (dispnode=&disp->DispNodes[0], slot=0; slot<pm_config.MaxParallelNodes; ++slot,++dispnode) {
		uint8 pslot;
		dispnode->pm = pm;
		size = sizeof(PmDispatcherPacket_t)*pm_config.PmaBatchSize;
		status = vs_pool_alloc(&pm_pool, size, (void*)&dispnode->DispPackets);
		if (status != VSTATUS_OK || !dispnode->DispPackets) {
			IB_LOG_ERRORRC("Failed to allocate Dispatcher Packets rc:", status);
			goto freeports;
		}
		dispnode->info.activePorts=NULL;
		memset(dispnode->DispPackets, 0, size);
		for (pslot=0; pslot<pm_config.PmaBatchSize; ++pslot)
			dispnode->DispPackets[pslot].dispnode = dispnode;
	}

	return VSTATUS_OK;

freeports:
	for (dispnode=&disp->DispNodes[0], slot=0; slot<pm_config.MaxParallelNodes; ++slot,++dispnode) {
		if (dispnode->DispPackets)
			vs_pool_free(&pm_pool, dispnode->DispPackets);
	}
	vs_pool_free(&pm_pool, disp->DispNodes);
freeevent:
	vs_event_delete(&disp->sweepDone);
freecontext:
	(void)cs_cntxt_instance_free(&pm_pool, &disp->cntx);
fail:
	return VSTATUS_BAD;
}

void PmDispatcherDestroy(Pm_t *pm)
{
	struct PmDispatcher_s *disp = &pm->Dispatcher;
	uint32_t slot;

	for (slot=0; slot<pm_config.MaxParallelNodes; ++slot) {
		if (disp->DispNodes[slot].DispPackets)
			vs_pool_free(&pm_pool, disp->DispNodes[slot].DispPackets);
	}
	vs_pool_free(&pm_pool, disp->DispNodes);
	vs_event_delete(&disp->sweepDone);
	(void)cs_cntxt_instance_free(&pm_pool, &disp->cntx);
}
