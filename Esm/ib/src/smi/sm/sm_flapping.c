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

typedef struct{
	LIST_ITEM entry;	
	uint64 timestamp;
} sm_flap_event_t;

typedef struct {
	Popo_t *popop;
	uint64	NodeGUID;
	STL_LID lid;
} flapping_context_t;

void
sm_flap_gc(PopoPort_t *poportp)
{
	QUICK_LIST *list = (QUICK_LIST*)poportp->quarantine.longTermData.data;
	LIST_ITEM * li, * next;
	for (li = QListHead(list); li; li = next) {
		next = QListNext(list, li);
		QListRemoveItem(list, li);
		sm_flap_event_t *event = PARENT_STRUCT(li, sm_flap_event_t, entry);
		vs_pool_free(&sm_pool, event);
	}
	vs_pool_free(&sm_pool, poportp->quarantine.longTermData.data);
}


static void
_sm_flap_age_out_events(QUICK_LIST *list, uint64_t window_start)
{
	LIST_ITEM * li, * next;
	for (li = QListHead(list); li; li = next) {
		next = QListNext(list, li);
		sm_flap_event_t *event = PARENT_STRUCT(li, sm_flap_event_t, entry);

		//age out old events.
		if(event->timestamp < window_start) {
			QListRemoveItem(list, li);
			vs_pool_free(&sm_pool, event);
		} else{
			break;
		}
	}
}

/*
 * sm_flap_state_handler is called periodically by monitor thread.
 * Makes decisions about the following possible state transitions:
 * Monitored -> Quarantined
 * Quarantined -> Monitored
 * Monitored -> Unmonitored
 *
 * This function is called with popo lock held.
 */
void
sm_flap_state_handler(Popo_t *popop, PopoPort_t *poportp)
{
	uint64_t current_time, window_start, window_size;
	vs_time_get(&current_time);
	window_size = sm_config.port_quarantine.flapping.window_size*60*1000000;//convert mins to us
	window_start = current_time - window_size;

	int high_thresh = sm_config.port_quarantine.flapping.high_thresh;
	int low_thresh = sm_config.port_quarantine.flapping.low_thresh;

	QUICK_LIST *list = (QUICK_LIST *)poportp->quarantine.longTermData.data;

	//Remove events from list that occurred outside of sliding window period
	_sm_flap_age_out_events(list, window_start);
	int event_count = QListCount(list);
	

	if(poportp->quarantine.monitored){
		if(event_count >= high_thresh) {
			/* TRANSITION Monitor -> Quarantine */
			//add to long term quarantine
			PopoQuarantineType_t prevType = poportp->quarantine.type;
			if(prevType == POPO_QUARANTINE_SHORT)
				QListRemoveItem(&popop->quarantine.shortTerm, &poportp->quarantine.quarantineEntry);
			poportp->quarantine.type = POPO_QUARANTINE_LONG;
			QListInsertTail(&popop->quarantine.longTerm, &poportp->quarantine.quarantineEntry);
			vs_time_get(&poportp->quarantine.longTermData.quarantinedTime);
			//remove from monitor
			poportp->quarantine.monitored = 0;
			QListRemoveItem(&popop->quarantine.monitored, &poportp->quarantine.monitorEntry);
			IB_LOG_WARN_FMT(__func__, "Port Quarantined for flapping! Guid: " FMT_U64 " PortIndex: %d", 
				poportp->ponodep->info.guid, poportp->portNum);

		} else if (event_count <= low_thresh) {

			if(window_start >= poportp->quarantine.longTermData.monitoredTime) {
				/* TRANSITION: Monitored -> Unmonitored */
				//remove from monitor
				poportp->quarantine.monitored = 0;
				QListRemoveItem(&popop->quarantine.monitored, &poportp->quarantine.monitorEntry);
				//IB_LOG_WARN_FMT(__func__, "Port unmonitored for flapping! Guid: " FMT_U64 " PortIndex: %d", 
					//poportp->ponodep->info.guid, poportp->portNum); //, debugging
				//garbage collect
				poportp->quarantine.longTermData.reason = POPO_LONGTERM_NONE;
				sm_flap_gc(poportp);
			}
		}

	} else if (poportp->quarantine.type == POPO_QUARANTINE_LONG) {

		if (event_count <= low_thresh) {
			if(window_start >= poportp->quarantine.longTermData.quarantinedTime) {
				/* Transition Quarantine -> Monitored */
				//remove from quarantine
				QListRemoveItem(&popop->quarantine.longTerm, &poportp->quarantine.quarantineEntry);
				poportp->quarantine.type = POPO_QUARANTINE_NONE;
				//add to monitor				
				poportp->quarantine.monitored = 1;
				vs_time_get(&poportp->quarantine.longTermData.monitoredTime);
				QListInsertTail(&popop->quarantine.monitored, &poportp->quarantine.monitorEntry);
				IB_LOG_WARN_FMT(__func__, "Port Unquarantined for flapping. Guid: " FMT_U64 " PortIndex: %d", 
					poportp->ponodep->info.guid, poportp->portNum);
			}
		}
	}
	return;
}

static boolean
_sm_flap_does_node_have_flapping(Popo_t *popop, Node_t *nodep)
{
	
	PopoNode_t *ponodep = nodep->ponodep;
	int i;
	sm_popo_lock(popop);
	for (i = 0; i <= ponodep->info.numPorts; ++i) {
		PopoPort_t * poportp = ponodep->ports + i;

		if(poportp->quarantine.type != POPO_QUARANTINE_LONG &&
			!poportp->quarantine.monitored)
			continue;

		if(poportp->quarantine.longTermData.reason != POPO_LONGTERM_FLAPPING)
			continue;
	
		sm_popo_unlock(popop);
		return TRUE;
	}

	sm_popo_unlock(popop);
	return FALSE;	
}


static 
void _sm_flap_log_event(Popo_t *popop, Port_t *portp){

	sm_flap_event_t *event;
	vs_pool_alloc(&sm_pool, sizeof(sm_flap_event_t), (void**) &event);
    vs_time_get(&event->timestamp);
	QListInsertTail(portp->poportp->quarantine.longTermData.data, &event->entry);
}


//_sm_flap_compare_psi compares a fresh Get(PortStateInfo) for a node against
//the last known port states recorded in the Node->Port->PortStates structure
//
static
boolean _sm_flap_compare_psi(Popo_t *popop, Node_t *nodep, STL_PORT_STATE_INFO *psi)
{
	boolean discovery_needed = 0;
	boolean state_change_found = 0;
	Port_t *portp;

	STL_PORT_STATES *newState;
	STL_PORT_STATES lastState;
	sm_popo_lock(popop);
	for_all_physical_ports(nodep, portp) {

		if(!portp){
			discovery_needed = 1;
			continue;
		}

		lastState = portp->poportp->portStates;
		newState = &psi[portp->index].PortStates;

		//if logical portState regressed it indicates this port bounced	
		if(newState->s.PortState < lastState.s.PortState) {
			state_change_found = 1;
			if(sm_popo_is_port_monitored_unsafe(&sm_popo, portp)) {
				if(sm_popo_get_quarantine_reason_unsafe(&sm_popo, portp) == POPO_LONGTERM_FLAPPING)
					_sm_flap_log_event(&sm_popo, portp);
				discovery_needed = 1;
				continue;
			}
			if(!sm_popo_is_port_quarantined_unsafe(&sm_popo, portp)) {
				discovery_needed = 1;
				continue;
			}
			if(sm_popo_get_quarantine_reason_unsafe(&sm_popo, portp) == POPO_LONGTERM_FLAPPING) {
				_sm_flap_log_event(&sm_popo, portp);
				//no discovery needed
				continue;
			}
		}

		//If port state moved forward in state machine,
		//and this is a quarantined port, block discovery sweep
		if(newState->s.PortState > lastState.s.PortState) {
			if(sm_popo_is_port_quarantined_unsafe(&sm_popo, portp)) {
				state_change_found = 1;
				continue;
			}
		}

		//if physical portState regressed it indicates this port bounced or 
		//is having trouble completing LNI
		if((lastState.s.PortPhysicalState == IB_PORT_PHYS_TRAINING) &&
			(newState->s.PortPhysicalState < IB_PORT_PHYS_TRAINING)) {
			state_change_found = 1;
			if(sm_popo_is_port_monitored_unsafe(&sm_popo, portp)) {
				if(sm_popo_get_quarantine_reason_unsafe(&sm_popo, portp) == POPO_LONGTERM_FLAPPING)
					_sm_flap_log_event(&sm_popo, portp);
				discovery_needed = 1;
				continue;
			}
			if(!sm_popo_is_port_quarantined_unsafe(&sm_popo, portp)) {
				discovery_needed = 1;
				continue;
			}
			if(sm_popo_get_quarantine_reason_unsafe(&sm_popo, portp) == POPO_LONGTERM_FLAPPING) {
				_sm_flap_log_event(&sm_popo, portp);
				//no discovery needed
				continue;
			}
		}

		//if IsSMConfigurationStarted was cleared it indicates this port bounced
		if(newState->s.IsSMConfigurationStarted == 0 &&
			lastState.s.IsSMConfigurationStarted == 1) {
			state_change_found = 1;
			if(sm_popo_is_port_monitored_unsafe(&sm_popo, portp)) {
				if(sm_popo_get_quarantine_reason_unsafe(&sm_popo, portp) == POPO_LONGTERM_FLAPPING)
					_sm_flap_log_event(&sm_popo, portp);
				discovery_needed = 1;
				continue;
			}
			if(!sm_popo_is_port_quarantined_unsafe(&sm_popo, portp)) {
				discovery_needed = 1;
				continue;
			}
			if(sm_popo_get_quarantine_reason_unsafe(&sm_popo, portp) == POPO_LONGTERM_FLAPPING) {
				_sm_flap_log_event(&sm_popo, portp);
				//no discovery needed
				continue;
			}
		}
	}

	if(!state_change_found){
		discovery_needed = 1;
	}

	sm_popo_unlock(popop);
	return discovery_needed;
}


static 
void _trap_psi_callback(cntxt_entry_t *cntxt, Status_t status, void *data, Mai_t *mad)
{

	flapping_context_t *ctx = (flapping_context_t *) data;
	boolean discovery_needed = 0;

	vs_rdlock(&old_topology_lock); 
	Node_t *nodep = sm_find_guid(&old_topology, ctx->NodeGUID);
	if(!nodep){
		discovery_needed = 1;
		goto cleanup;
	}

	if(!sm_callback_check(cntxt, status, nodep, NULL, mad)) {
		discovery_needed = 1;
		goto cleanup;
	}

	STL_PORT_STATE_INFO * psi = (STL_PORT_STATE_INFO *)stl_mai_get_smp_data(mad);
    BSWAP_STL_PORT_STATE_INFO(psi, nodep->nodeInfo.NumPorts +1);
	discovery_needed = _sm_flap_compare_psi(ctx->popop, nodep, psi);
	sm_popo_update_node_port_states(ctx->popop, nodep, psi);

cleanup:
	vs_unlock(&old_topology_lock);
	if(discovery_needed) {
		//Callbacks are currently done on async thread. Async thread is
		//the only caller of sm_discovery_needed, which modifies shared globals
		//IF callbacks are taken off async thread locks will probably be needed here. 
		sm_discovery_needed("Port State Change Trap", ctx->lid);
	}
	vs_pool_free(&sm_pool, ctx);
	return;
}


boolean
sm_flap_report_port_change_trap(Popo_t *popop, Node_t *nodep)
{

	if(!nodep)
		return TRUE;

	if(!_sm_flap_does_node_have_flapping(popop, nodep))
		return TRUE;

	Port_t *portp = sm_get_node_end_port(nodep);
	if(!portp)
		return TRUE;

	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, portp->portData->lid);
	uint32_t amod = (nodep->nodeInfo.NumPorts + 1) << 24;
	flapping_context_t *ctx;
	vs_pool_alloc(&sm_pool, sizeof(flapping_context_t), (void**) &ctx);
	ctx->NodeGUID = nodep->nodeInfo.NodeGUID; 
	ctx->lid = portp->portData->lid;
	ctx->popop = popop;

	SM_Get_PortStateInfo_Dispatch(fd_flapping_port, amod, &addr, nodep,
		&sm_asyncDispatch, _trap_psi_callback, ctx); 

	return FALSE; //Don't trigger a resweep just yet
}
