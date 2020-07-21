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

#include "sm_l.h"


static Thread_t quar_monitor_thread;
static boolean shutdown_monitor;
static Sema_t monitor_shutdown_sema;



void
sm_popo_lock(Popo_t * popop)
{
	if (vs_lock(&popop->lock) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("failed to lock popo");
}

void
sm_popo_unlock(Popo_t * popop)
{
	if (vs_unlock(&popop->lock) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("failed to unlock popo");
}



static void 
_popo_update_longterm_quarantine_list(Popo_t *popop)
{
	sm_popo_lock(popop);
	QUICK_LIST * list = &popop->quarantine.longTerm;
	LIST_ITEM * li, * next;
	for (li = QListHead(list); li; li = next) {
		next = QListNext(list, li);	
		PopoPort_t * poportp = PARENT_STRUCT(li, PopoPort_t, quarantine.quarantineEntry);

		if(poportp->quarantine.type != POPO_QUARANTINE_LONG)
			continue;

		switch (poportp->quarantine.longTermData.reason) {
			case POPO_LONGTERM_NONE:
				//this is an invalid state.  
				break;
			case POPO_LONGTERM_FLAPPING:
				sm_flap_state_handler(popop, poportp);
				break;
			case POPO_LONGTERM_PERSISTENT_TIMEOUT:
			default:
				//_popo_persistent_to_state_handler(popop, poportp);
				//Other quarantine reasons not yet implemented
				break;
		}
	}
	sm_popo_unlock(popop);
}

static void 
_popo_update_longterm_monitor_list(Popo_t *popop)
{
	sm_popo_lock(popop);
	QUICK_LIST * list = &popop->quarantine.monitored;
	LIST_ITEM * li, * next;
	for (li = QListHead(list); li; li = next) {
		next = QListNext(list, li);	
		PopoPort_t * poportp = PARENT_STRUCT(li, PopoPort_t, quarantine.monitorEntry);

		switch (poportp->quarantine.longTermData.reason) {
			case POPO_LONGTERM_NONE:
				//this is an invalid state.  
				break;
			case POPO_LONGTERM_FLAPPING:
				sm_flap_state_handler(popop, poportp);
				break;
			case POPO_LONGTERM_PERSISTENT_TIMEOUT:
			default:
				//_popo_persistent_to_state_handler(popop, poportp);
				//Other quarantine reasons not yet implemented
				break;
		}
	}
	sm_popo_unlock(popop);
}

#define POPO_MONITOR_THREAD_PERIOD 60 //run monitor algorithms every x secs


static void 
_popo_port_monitor_thread(uint32_t argc, uint8_t **argv)
{
	int sleep_cnt=0;

	Popo_t *popop = &sm_popo;

	while(!shutdown_monitor){
		vs_thread_sleep(VTIMER_1S); 
		if(sleep_cnt++ < POPO_MONITOR_THREAD_PERIOD) {
			continue;
		}
		sleep_cnt = 0;

		_popo_update_longterm_monitor_list(popop);
		_popo_update_longterm_quarantine_list(popop);

	}

	cs_vsema(&monitor_shutdown_sema);
	return;
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
			QListRemoveItem(&popop->quarantine.shortTerm, &poportp->quarantine.quarantineEntry);
			break;
		case POPO_QUARANTINE_LONG:
			QListRemoveItem(&popop->quarantine.longTerm, &poportp->quarantine.quarantineEntry);
			break;
	}

	if(poportp->quarantine.monitored)
		QListRemoveItem(&popop->quarantine.monitored, &poportp->quarantine.monitorEntry);

	if(poportp->quarantine.longTermData.reason == POPO_LONGTERM_FLAPPING){
		sm_flap_gc(poportp);
	}
}

// Garbage collects a persistent node.
//
static void
_popo_gc_node(Popo_t * popop, PopoNode_t * ponodep)
{
	int i;

	for (i = 0; i <= ponodep->info.numPorts; ++i) {
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

	sm_popo_lock(popop);

	for (i = cl_qmap_head(&popop->nodes); i && i != cl_qmap_end(&popop->nodes); i = j) {
		j = cl_qmap_next(i);
		ponodep = PARENT_STRUCT(i, PopoNode_t, nodesEntry);
		if (ponodep && (force || pass - ponodep->info.pass > MAX(2, sm_config.non_resp_max_count)))
			_popo_gc_node(popop, ponodep);
	}

	sm_popo_unlock(popop);
}

void
sm_popo_init(Popo_t * popop)
{
	if (vs_lock_init(&popop->lock, VLOCK_FREE, VLOCK_THREAD) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("failed to init popo lock");

	cl_qmap_init(&popop->nodes, NULL);
	popop->errors.cumulativeTimeout = 0;
	QListInit(&popop->quarantine.shortTerm); 
	
	if(sm_config.port_quarantine.enabled) {
		//start monitor thread
		QListInit(&popop->quarantine.monitored);
		QListInit(&popop->quarantine.longTerm);
		cs_sema_create(&monitor_shutdown_sema, 0);
		vs_thread_create(&quar_monitor_thread,(unsigned char *) "long_quar", 
							  _popo_port_monitor_thread, 0, NULL, SM_STACK_SIZE);
	}
}

void	
sm_popo_destroy(Popo_t * popop)
{
	if(sm_config.port_quarantine.enabled) {
		shutdown_monitor = TRUE;

		cs_psema(&monitor_shutdown_sema);
		cs_sema_delete(&monitor_shutdown_sema);
	}
	_popo_gc(popop, TRUE);
	vs_lock_delete(&popop->lock);
	memset(popop, 0, sizeof(Popo_t));
}

void
sm_popo_report(Popo_t * popop)
{
	sm_popo_lock(popop);
	IB_LOG_INFINI_INFO_FMT(__func__, "short term quarantine: %u",
		QListCount(&popop->quarantine.shortTerm));
	IB_LOG_INFINI_INFO_FMT(__func__, "long term quarantine: %u",
		QListCount(&popop->quarantine.longTerm));
	IB_LOG_INFINI_INFO_FMT(__func__, "monitor: %u",
		QListCount(&popop->quarantine.monitored));
	sm_popo_unlock(popop);
}

PopoNode_t *
sm_popo_get_node(Popo_t * popop, const STL_NODE_INFO * nodeInfo)
{
	Status_t status;
	PopoNode_t * ponodep;
	size_t len;

	sm_popo_lock(popop);
	
	cl_map_item_t * item = cl_qmap_get(&popop->nodes, nodeInfo->NodeGUID);
	if (item == cl_qmap_end(&popop->nodes)) {
		len = sizeof(PopoNode_t) + sizeof(PopoPort_t) * (nodeInfo->NumPorts + 1);
		if ((status = vs_pool_alloc(&sm_pool, len, (void **)&ponodep)) != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("failed to allocate persistent port");
		memset(ponodep, 0, len);
		ponodep->info.guid = nodeInfo->NodeGUID;
		ponodep->info.pass = topology_passcount;
		ponodep->info.nodeType = nodeInfo->NodeType;
		ponodep->info.numPorts = nodeInfo->NumPorts;
		if (cl_qmap_insert(&popop->nodes, nodeInfo->NodeGUID, &ponodep->nodesEntry) != &ponodep->nodesEntry)
			IB_FATAL_ERROR_NODUMP("failed to insert persistent port into map");
	} else {
		ponodep = PARENT_STRUCT(item, PopoNode_t, nodesEntry);
		ponodep->info.pass = topology_passcount;
	}

	sm_popo_unlock(popop);
	return ponodep;
}

PopoPort_t *
sm_popo_get_port(Popo_t * popop, PopoNode_t * ponodep, uint8_t port)
{
	PopoPort_t * poportp;

	sm_popo_lock(popop);

	if (port > ponodep->info.numPorts) {
		sm_popo_unlock(popop);
		return NULL;
	}

	poportp = ponodep->ports + port;
	poportp->ponodep = ponodep;
	poportp->portNum = port;
	sm_popo_unlock(popop);
	return poportp;
}

boolean
sm_popo_is_port_quarantined(Popo_t * popop, Port_t * portp)
{
	boolean ret = FALSE;
	sm_popo_lock(popop);
	if(portp && portp->poportp->quarantine.type != POPO_QUARANTINE_NONE)
		ret = TRUE;
	sm_popo_unlock(popop);
	return ret;
}

boolean
sm_popo_is_port_monitored(Popo_t * popop, Port_t * portp)
{
	boolean ret = FALSE;
	sm_popo_lock(popop);
	if(portp && portp->poportp->quarantine.monitored == TRUE)
		ret = TRUE;
	sm_popo_unlock(popop);
	return ret;
}


PopoQuarantineType_t
sm_popo_get_quarantine_type(Popo_t * popop, Port_t * portp)
{	
	PopoQuarantineType_t type = POPO_QUARANTINE_NONE;
	sm_popo_lock(popop);
	if(portp)
		type = portp->poportp->quarantine.type;
	sm_popo_unlock(popop);
	return type;
}


PopoLongTermQuarantineReason_t
sm_popo_get_quarantine_reason(Popo_t * popop, Port_t * portp)
{
	PopoLongTermQuarantineReason_t reason = POPO_LONGTERM_NONE;
	sm_popo_lock(popop);
	if(portp)
		reason = portp->poportp->quarantine.longTermData.reason;
	sm_popo_unlock(popop);
	return reason;
}

boolean
sm_popo_is_port_quarantined_unsafe(Popo_t * popop, Port_t * portp)
{
	boolean ret = FALSE;
	if(portp && portp->poportp->quarantine.type != POPO_QUARANTINE_NONE)
		ret = TRUE;
	return ret;
}

boolean
sm_popo_is_port_monitored_unsafe(Popo_t * popop, Port_t * portp)
{
	boolean ret = FALSE;
	if(portp && portp->poportp->quarantine.monitored == TRUE)
		ret = TRUE;
	return ret;
}


PopoQuarantineType_t
sm_popo_get_quarantine_type_unsafe(Popo_t * popop, Port_t * portp)
{	
	PopoQuarantineType_t type = POPO_QUARANTINE_NONE;
	if(portp)
		type = portp->poportp->quarantine.type;
	return type;
}


PopoLongTermQuarantineReason_t
sm_popo_get_quarantine_reason_unsafe(Popo_t * popop, Port_t * portp)
{
	PopoLongTermQuarantineReason_t reason = POPO_LONGTERM_NONE;
	if(portp)
		reason = portp->poportp->quarantine.longTermData.reason;
	return reason;
}
void
sm_popo_get_ldr_log(Popo_t * popop, Port_t * portp, STL_LINKDOWN_REASON *log)
{
	if (portp && log) {
		sm_popo_lock(popop);
		memcpy(log, portp->poportp->ldr.ldr_log, sizeof(STL_LINKDOWN_REASON) * STL_NUM_LINKDOWN_REASONS);
		sm_popo_unlock(popop);
	}
}
uint64_t
sm_popo_get_sweep_start(Popo_t * popop)
{
	uint64_t sweep_start;
	sm_popo_lock(popop);
	sweep_start = popop->ldr.sweepStart;
	sm_popo_unlock(popop);
	return sweep_start;
}
boolean
sm_popo_find_lastest_ldr(Popo_t * popop, Port_t * portp, STL_LINKDOWN_REASON *ldr)
{
	boolean ret = FALSE;
	if (!portp || !ldr)
		return FALSE;

	sm_popo_lock(popop);
	int idx = STL_LINKDOWN_REASON_LAST_INDEX(portp->poportp->ldr.ldr_log);
	if (idx != -1 && portp->poportp->ldr.ldr_log[idx].Timestamp != 0) {
		memcpy(ldr, &portp->poportp->ldr.ldr_log[idx], sizeof(STL_LINKDOWN_REASON));
		ret = TRUE;
	}
	sm_popo_unlock(popop);
	return ret;
}

static void
 _popo_clear_cache_nonresp_unsafe(PopoNode_t *ponodep)
{
	ponodep->nonresp.nrSweepNum = 0;
	ponodep->nonresp.nrTime = 0;
	ponodep->nonresp.nrCount = 0;
}

void sm_popo_clear_cache_nonresp(Popo_t *popop, Node_t *nodep)
{
	if (nodep && nodep->nodeInfo.NodeType == STL_NODE_FI) {
		sm_popo_lock(popop);
		_popo_clear_cache_nonresp_unsafe(nodep->ponodep);
		sm_popo_unlock(popop);
	}
}

boolean sm_popo_is_nonresp_this_sweep(Popo_t *popop, Node_t *nodep)
{
	boolean isNonResp = FALSE;

	/* If both options are disabled, so is feature */
	if (sm_config.non_resp_tsec == 0 && sm_config.non_resp_max_count == 0) return isNonResp;

	if (nodep && nodep->nodeInfo.NodeType == STL_NODE_FI) {
		sm_popo_lock(popop);
		/* If the Nodes was marked NonResp this Sweep Then the node is NonResp */
		isNonResp = (nodep->ponodep->nonresp.nrSweepNum == topology_passcount)
			&& nodep->ponodep->nonresp.nrCount > 0;
		sm_popo_unlock(popop);
	}
	return isNonResp;
}

boolean sm_popo_use_cache_nonresp(Popo_t * popop, Node_t *nodep)
{
	uint64_t curtime, difftime;
	boolean isValid = FALSE;

	/* If both options are disabled, so is feature */
	if (sm_config.non_resp_tsec == 0 && sm_config.non_resp_max_count == 0) return isValid;

	if (nodep && nodep->nodeInfo.NodeType == STL_NODE_FI) {
		sm_popo_lock(popop);
		(void) vs_time_get(&curtime);

		difftime = curtime - nodep->ponodep->nonresp.nrTime;

		/* If NonResp Time is still less than Config
		* OR SweepCount is less than MaxCount
		* Then cache is still valid
		*/
		if (difftime < (sm_config.non_resp_tsec * VTIMER_1S)
			|| nodep->ponodep->nonresp.nrCount < sm_config.non_resp_max_count)
		{
			isValid = TRUE;
		}
		sm_popo_unlock(popop);
	}
	return isValid;
}
boolean sm_popo_inc_and_use_cache_nonresp(Popo_t * popop, Node_t *nodep, Status_t *retStatus)
{
	uint64_t curtime, difftime;
	boolean isValid = FALSE;

	/* If both options are disabled, so is feature */
	if (sm_config.non_resp_tsec == 0 && sm_config.non_resp_max_count == 0) return isValid;
	/* If status is not Time out don't use cache */
	if (retStatus && *retStatus != VSTATUS_TIMEOUT) return isValid;

	if (nodep && nodep->nodeInfo.NodeType == STL_NODE_FI) {

		/* Check if Timeout is exceeded for sweep */
		if (AtomicRead(&popop->errors.cumulativeTimeout) >= sm_config.cumulative_timeout_limit) {
			if (retStatus) *retStatus = VSTATUS_TIMEOUT_LIMIT;
			return isValid;
		}

		sm_popo_lock(popop);
		(void) vs_time_get(&curtime);

		/* If NonRespSweepCount Check is enabled and it is a new sweep, then Update count */
		if (sm_config.non_resp_max_count != 0
			&& nodep->ponodep->nonresp.nrSweepNum != topology_passcount)
		{
			nodep->ponodep->nonresp.nrCount++;
			nodep->ponodep->nonresp.nrSweepNum = topology_passcount;
		}

		/* If NonRespTime Check is enabled and Time has not been set, then set it */
		if (sm_config.non_resp_tsec != 0
			&& nodep->ponodep->nonresp.nrTime == 0)
		{
			nodep->ponodep->nonresp.nrTime = curtime;
			nodep->ponodep->nonresp.nrSweepNum = topology_passcount;
		}

		difftime = curtime - nodep->ponodep->nonresp.nrTime;

		/* If NonResp Time is still less than Config
		* OR SweepCount is less than MaxCount
		* Then cache is still valid
		*/
		if (difftime < (sm_config.non_resp_tsec * VTIMER_1S)
			|| nodep->ponodep->nonresp.nrCount < sm_config.non_resp_max_count)
		{
			isValid = TRUE;
		}
		sm_popo_unlock(popop);
	}
	return isValid;
}
// An entire switch is considered quarantined if port zero is quarantined.
//
boolean
sm_popo_is_node_quarantined(Popo_t * popop, uint64_t guid)
{
	sm_popo_lock(popop);

	cl_map_item_t * item = cl_qmap_get(&popop->nodes, guid);
	if (item == cl_qmap_end(&popop->nodes)) {
		sm_popo_unlock(popop);
		return FALSE;
	}

	PopoNode_t * ponodep = PARENT_STRUCT(item, PopoNode_t, nodesEntry);
	boolean q = ponodep->info.nodeType == NI_TYPE_SWITCH &&
		ponodep->ports[0].quarantine.type != POPO_QUARANTINE_NONE;

	sm_popo_unlock(popop);

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
			QListRemoveItem(&popop->quarantine.shortTerm, &poportp->quarantine.quarantineEntry);
			break;
		case POPO_QUARANTINE_LONG:
			QListRemoveItem(&popop->quarantine.longTerm, &poportp->quarantine.quarantineEntry);
			break;
	}

	poportp->quarantine.type = type;

	switch (type) {
		case POPO_QUARANTINE_NONE:
			break;
		case POPO_QUARANTINE_SHORT:
			QListInsertTail(&popop->quarantine.shortTerm, &poportp->quarantine.quarantineEntry);
			break;
		case POPO_QUARANTINE_LONG:
			QListInsertTail(&popop->quarantine.longTerm, &poportp->quarantine.quarantineEntry);
			break;
	}
}


void
sm_popo_monitor_port(Popo_t *popop, Port_t *portp, PopoLongTermQuarantineReason_t reason)
{

	if (!portp) return;

	if (reason == POPO_LONGTERM_NONE) return;

	sm_popo_lock(popop);

	if (!sm_config.port_quarantine.enabled) goto bail;

	if (portp->poportp->quarantine.monitored) goto bail;

	if (portp->poportp->quarantine.type == POPO_QUARANTINE_LONG) goto bail;


	if(reason == POPO_LONGTERM_FLAPPING){

		if(sm_config.port_quarantine.flapping.window_size == 0){
			//flapping detection disabled, don't do anything.
			goto bail;
		}

		QUICK_LIST *list;
		vs_pool_alloc(&sm_pool, sizeof(QUICK_LIST), (void**) &list);
		QListInit(list);
		portp->poportp->quarantine.longTermData.data = list;
	} else  {
		//Only monitor state for LONGTERM_FLAPPING is currently implemented. As
		//new reasons are implemented, new code blocks must be added here to
		//allocate longTermData to the reasons specific type.
		goto bail;
	}

	portp->poportp->quarantine.monitored = TRUE;
	portp->poportp->quarantine.longTermData.reason = reason;
	vs_time_get(&portp->poportp->quarantine.longTermData.monitoredTime);
	QListInsertTail(&popop->quarantine.monitored, &portp->poportp->quarantine.monitorEntry);

bail:
	sm_popo_unlock(popop);
	return;
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
		PopoPort_t * poportp = PARENT_STRUCT(li, PopoPort_t, quarantine.quarantineEntry);
		poportp->quarantine.type = POPO_QUARANTINE_NONE;
	}

	return TRUE;
}

// Returns TRUE if caller should schedule a resweep.
boolean
sm_popo_clear_short_quarantine(Popo_t * popop)
{
	boolean resweep = FALSE;

	sm_popo_lock(popop);

	resweep = _clear_quarantine("short", &popop->quarantine.shortTerm);

	// Only resweep if a trap is pending from a previous failed sweep.
	// Otherwise, rely on the normal sweep schedule or future traps as per default behavior.
	resweep = resweep && popop->errors.trapPending;
	popop->errors.trapPending = FALSE;

	sm_popo_unlock(popop);

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
	Status_t retStatus = status;
	if (!portp || (status != VSTATUS_TIMEOUT && status != VSTATUS_TIMEOUT_LIMIT))
		return status;

	sm_popo_lock(popop);

	if(sm_config.cumulative_timeout_limit){
		if(portp->poportp->quarantine.type != POPO_QUARANTINE_LONG) {
			sm_popo_quarantine_port_unsafe(popop, portp->poportp, POPO_QUARANTINE_SHORT);

			Port_t * nportp = sm_find_port(topop, portp->nodeno, portp->portno);
			if (nportp && nportp->poportp->quarantine.type != POPO_QUARANTINE_LONG)
				sm_popo_quarantine_port_unsafe(popop, nportp->poportp, POPO_QUARANTINE_SHORT);

		}

		if (AtomicRead(&popop->errors.cumulativeTimeout) >= sm_config.cumulative_timeout_limit)
				retStatus = VSTATUS_TIMEOUT_LIMIT;
	}

	sm_popo_unlock(popop);

	sm_popo_monitor_port(popop, portp, POPO_LONGTERM_FLAPPING);

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
 	sm_popo_lock(popop);
	popop->errors.trapPending = TRUE;
	sm_popo_unlock(popop);
 }


void
sm_popo_update_port_state(Popo_t *popop, Port_t *portp, STL_PORT_STATES *pstatep)
{
	if(!portp)
		return;

	sm_popo_lock(popop);
	// On Port Arm/Activate LDR is cleared during Set
	if (pstatep->s.PortState > IB_PORT_INIT) {
		portp->poportp->ldr.updateLog = FALSE;
	}
	memcpy(&portp->poportp->portStates, pstatep, sizeof(STL_PORT_STATES));
	sm_popo_unlock(popop);

	return;
}
void
sm_popo_update_port_state_with_ldr(Popo_t *popop, Port_t *portp, STL_PORT_STATES *pstatep,
	uint8_t ldr, uint8_t nldr)
{
	if(!portp)
		return;

	sm_popo_lock(popop);
	if ((ldr || nldr) && !portp->poportp->ldr.updateLog) {
		int idx = STL_LINKDOWN_REASON_NEXT_INDEX(portp->poportp->ldr.ldr_log);
		if (idx != -1) {
			portp->poportp->ldr.ldr_log[idx].LinkDownReason = ldr;
			portp->poportp->ldr.ldr_log[idx].NeighborLinkDownReason = nldr;
			portp->poportp->ldr.ldr_log[idx].Timestamp = popop->ldr.sweepStart;
			// Mark update flag to stop from double counting until PortInfo.LDR is cleared on Arm/Activate
			portp->poportp->ldr.updateLog = TRUE;
		}
	}

	// On Port Arm/Activate LDR is cleared during Set
	if (pstatep->s.PortState > IB_PORT_INIT) {
		portp->poportp->ldr.updateLog = FALSE;
	}
	memcpy(&portp->poportp->portStates, pstatep, sizeof(STL_PORT_STATES));
	sm_popo_unlock(popop);

	return;
}

void
sm_popo_update_node_port_states(Popo_t *popop, Node_t * nodep, STL_PORT_STATE_INFO *psi)
{
	int i;
	if(!nodep || !psi)
		return;
	PopoNode_t *ponodep = nodep->ponodep;
	sm_popo_lock(popop);

	for(i=0; i< ponodep->info.numPorts + 1; i++) {
		PopoPort_t *poportp = &ponodep->ports[i];
		memcpy(&poportp->portStates, &psi[i].PortStates, sizeof(STL_PORT_STATES));
	}

	sm_popo_unlock(popop);
	return;
}

void sm_popo_update_node(PopoNode_t * ponodep)
{
	ponodep->info.pass = topology_passcount;
}

void sm_popo_begin_sweep(Popo_t *popop, uint64_t start_sweep)
{
	cl_map_item_t *ni = NULL;

	sm_popo_lock(popop);
	// Sweep Start (in seconds) used for LinkdownReason Timestamp
	popop->ldr.sweepStart = start_sweep;

	if (topology_passcount == 0) {
		/* If this is a new sweep as Master then reset nonresp values to avoid edge cases */
		for (ni = cl_qmap_head(&popop->nodes); ni != cl_qmap_end(&popop->nodes); ni = cl_qmap_next(ni)) {
			PopoNode_t *ponodep = PARENT_STRUCT(ni, PopoNode_t, nodesEntry);
			_popo_clear_cache_nonresp_unsafe(ponodep);
		}
	}
	sm_popo_unlock(popop);
}

void sm_popo_end_sweep(Popo_t * popop)
{
	sm_popo_reset_errors(popop);
	_popo_gc(popop, FALSE);
}

