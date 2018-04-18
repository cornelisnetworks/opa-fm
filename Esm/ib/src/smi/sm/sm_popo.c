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

#include "sm_l.h"

static void
_popo_lock(Popo_t * popop)
{
	if (vs_lock(&popop->lock) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("failed to lock popo");
}

static void
_popo_unlock(Popo_t * popop)
{
	if (vs_unlock(&popop->lock) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("failed to unlock popo");
}

// Garbage collects a persistent port.
//
static void
_popo_gc_port(Popo_t * popop, PopoPort_t * poportp)
{
	switch (poportp->quarantine.type) {
		case POPO_QUARANTINE_NONE:
			break;
		case POPO_QUARANTINE_SHORT:
			QListRemoveItem(&popop->quarantine.shortTerm, &poportp->quarantine.entry);
			break;
	}
}

// Garbage collects a persistent node.
//
static void
_popo_gc_node(Popo_t * popop, PopoNode_t * ponodep)
{
	int i;

	for (i = 0; i < ponodep->info.numPorts; ++i) {
		PopoPort_t * poportp = ponodep->ports + i;
		_popo_gc_port(popop, poportp);
	}

	cl_qmap_remove_item(&popop->nodes, &ponodep->nodesEntry);

	vs_pool_free(&sm_pool, ponodep);
}

// Garbage collect persistent nodes if they've not been seen
// after some number of sweeps.
//
static void
_popo_gc(Popo_t * popop, boolean force)
{
	PopoNode_t * ponodep;
	cl_map_item_t * i, * j;
	uint32_t pass = topology_passcount;

	_popo_lock(popop);

	for (i = cl_qmap_head(&popop->nodes); i && i != cl_qmap_end(&popop->nodes); i = j) {
		j = cl_qmap_next(i);
		ponodep = PARENT_STRUCT(i, PopoNode_t, nodesEntry);
		if (ponodep && (force || pass - ponodep->info.pass > MAX(2, sm_config.non_resp_max_count)))
			_popo_gc_node(popop, ponodep);
	}

	_popo_unlock(popop);
}

void
sm_popo_init(Popo_t * popop)
{
	if (vs_lock_init(&popop->lock, VLOCK_FREE, VLOCK_THREAD) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("failed to init popo lock");

	cl_qmap_init(&popop->nodes, NULL);
	popop->errors.cumulativeTimeout = 0;
	QListInit(&popop->quarantine.shortTerm);
}

void
sm_popo_destroy(Popo_t * popop)
{
	_popo_gc(popop, TRUE);
	vs_lock_delete(&popop->lock);
	memset(popop, 0, sizeof(Popo_t));
}

void
sm_popo_report(Popo_t * popop)
{
	_popo_lock(popop);
	IB_LOG_INFINI_INFO_FMT(__func__, "short term quarantine: %u",
		QListCount(&popop->quarantine.shortTerm));
	_popo_unlock(popop);
}

PopoNode_t *
sm_popo_get_node(Popo_t * popop, const STL_NODE_INFO * nodeInfo)
{
	Status_t status;
	PopoNode_t * ponodep;
	size_t len;

	_popo_lock(popop);
	
	cl_map_item_t * item = cl_qmap_get(&popop->nodes, nodeInfo->NodeGUID);
	if (item == cl_qmap_end(&popop->nodes)) {
		len = sizeof(PopoNode_t) + sizeof(PopoPort_t) * (nodeInfo->NumPorts + 1);
		if ((status = vs_pool_alloc(&sm_pool, len, (void **)&ponodep)) != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("failed to allocate persistent port");
		memset(ponodep, 0, len);
		ponodep->info.pass = topology_passcount;
		ponodep->info.nodeType = nodeInfo->NodeType;
		ponodep->info.numPorts = nodeInfo->NumPorts;
		if (cl_qmap_insert(&popop->nodes, nodeInfo->NodeGUID, &ponodep->nodesEntry) != &ponodep->nodesEntry)
			IB_FATAL_ERROR_NODUMP("failed to insert persistent port into map");
	} else {
		ponodep = PARENT_STRUCT(item, PopoNode_t, nodesEntry);
	}

	_popo_unlock(popop);
	return ponodep;
}

PopoPort_t *
sm_popo_get_port(Popo_t * popop, PopoNode_t * ponodep, uint8_t port)
{
	PopoPort_t * poportp;

	_popo_lock(popop);

	if (port > ponodep->info.numPorts) {
		_popo_unlock(popop);
		return NULL;
	}

	poportp = ponodep->ports + port;
	poportp->ponodep = ponodep;

	_popo_unlock(popop);
	return poportp;
}

boolean
sm_popo_is_port_quarantined(Popo_t * popop, Port_t * portp)
{
	return sm_valid_port(portp) && portp->poportp->quarantine.type != POPO_QUARANTINE_NONE;
}

// An entire switch is considered quarantined if port zero is quarantined.
//
boolean
sm_popo_is_node_quarantined(Popo_t * popop, uint64_t guid)
{
	_popo_lock(popop);

	cl_map_item_t * item = cl_qmap_get(&popop->nodes, guid);
	if (item == cl_qmap_end(&popop->nodes)) {
		_popo_unlock(popop);
		return FALSE;
	}

	PopoNode_t * ponodep = PARENT_STRUCT(item, PopoNode_t, nodesEntry);
	boolean q = ponodep->info.nodeType == NI_TYPE_SWITCH &&
		ponodep->ports[0].quarantine.type != POPO_QUARANTINE_NONE;

	_popo_unlock(popop);

	return q;
}

static void
sm_popo_quarantine_port_unsafe(Popo_t * popop, PopoPort_t * poportp, PopoQuarantineType_t type)
{
	PopoQuarantineType_t prevType = poportp->quarantine.type;
	if (prevType == type)
		return;

	switch (prevType) {
		case POPO_QUARANTINE_NONE:
			break;
		case POPO_QUARANTINE_SHORT:
			QListRemoveItem(&popop->quarantine.shortTerm, &poportp->quarantine.entry);
			break;
	}

	poportp->quarantine.type = type;

	switch (type) {
		case POPO_QUARANTINE_NONE:
			break;
		case POPO_QUARANTINE_SHORT:
			QListInsertTail(&popop->quarantine.shortTerm, &poportp->quarantine.entry);
			break;
	}
}

void
sm_popo_quarantine_port(Popo_t * popop, Port_t * portp, PopoQuarantineType_t type)
{
	DEBUG_ASSERT(type != POPO_QUARANTINE_NONE);

	if (!sm_valid_port(portp)) return;

	_popo_lock(popop);
	sm_popo_quarantine_port_unsafe(popop, portp->poportp, type);
	_popo_unlock(popop);
}

static boolean
_clear_quarantine(char * name, QUICK_LIST * list)
{
	if (QListIsEmpty(list))
		return FALSE;

	IB_LOG_INFINI_INFO_FMT(__func__, "POPO: clearing quarantine: name[%s] count[%u]",
		name, QListCount(list));

	LIST_ITEM * li, * next;
	for (li = QListHead(list); li; li = next) {
		next = QListNext(list, li);
		QListRemoveItem(list, li);
		PopoPort_t * poportp = PARENT_STRUCT(li, PopoPort_t, quarantine.entry);
		poportp->quarantine.type = POPO_QUARANTINE_NONE;
	}

	return TRUE;
}

// Returns TRUE if caller should schedule a resweep.
boolean
sm_popo_clear_short_quarantine(Popo_t * popop)
{
	boolean resweep = FALSE;

	_popo_lock(popop);

	resweep = _clear_quarantine("short", &popop->quarantine.shortTerm);

	// Only resweep if a trap is pending from a previous failed sweep.
	// Otherwise, rely on the normal sweep schedule or future traps as per default behavior.
	resweep = resweep && popop->errors.trapPending;
	popop->errors.trapPending = FALSE;

	_popo_unlock(popop);

	return resweep;
}

uint64_t sm_popo_scale_timeout(Popo_t * popop, uint64_t timeout)
{
	uint64_t limit = sm_config.cumulative_timeout_limit;
	if (limit == 0) return timeout;

	uint64_t total = AtomicRead(&popop->errors.cumulativeTimeout);
	total = MIN(total, limit);
	return (uint64_t)(timeout * (1.0 - (double)total / limit));
}

void sm_popo_report_timeout(Popo_t * popop, uint64_t timeout)
{
	AtomicAdd(&popop->errors.cumulativeTimeout, timeout);
}

Status_t
sm_popo_port_error(Popo_t * popop, Topology_t * topop, Port_t * portp, Status_t status)
{
	if (!portp || status != VSTATUS_TIMEOUT || !sm_config.cumulative_timeout_limit)
		return status;

	_popo_lock(popop);

	sm_popo_quarantine_port_unsafe(popop, portp->poportp, POPO_QUARANTINE_SHORT);

	Port_t * nportp = sm_find_port(topop, portp->nodeno, portp->portno);
	if (sm_valid_port(nportp))
		sm_popo_quarantine_port_unsafe(popop, nportp->poportp, POPO_QUARANTINE_SHORT);

	Status_t retStatus = status;
	if (AtomicRead(&popop->errors.cumulativeTimeout) >= sm_config.cumulative_timeout_limit)
		retStatus = VSTATUS_TIMEOUT_LIMIT;

	_popo_unlock(popop);

	return retStatus;
}

void
sm_popo_reset_errors(Popo_t * popop)
{
	AtomicWrite(&popop->errors.cumulativeTimeout, 0);
}

boolean
sm_popo_should_abandon(Popo_t * popop)
{
	uint32_t limit = sm_config.cumulative_timeout_limit;
	return limit && AtomicRead(&popop->errors.cumulativeTimeout) >= limit;
}

 void
 sm_popo_report_trap(Popo_t * popop)
 {
 	_popo_lock(popop);
	popop->errors.trapPending = TRUE;
	_popo_unlock(popop);
 }

void sm_popo_update_node(PopoNode_t * ponodep)
{
	ponodep->info.pass = topology_passcount;
}

void sm_popo_end_sweep(Popo_t * popop)
{
	sm_popo_reset_errors(popop);
	_popo_gc(popop, FALSE);
}

