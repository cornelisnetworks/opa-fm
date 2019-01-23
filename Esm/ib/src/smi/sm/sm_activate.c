/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2018, Intel Corporation

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
#include "sm_activate.h"

// time after arming when we're in the idle flit propagation window
#define IDLE_FLIT_PROPAGATION_TIME (50 * VTIMER_1_MILLISEC)

//
// API for interacting with sweep_activate()'s retry logic.
//

static uint8_t
_activation_retry_attempts(ActivationRetry_t * retry)
{
	uint8_t r;

	MutexAcquire(&retry->mutex);
	r = retry ? retry->attempts : 0;
	MutexRelease(&retry->mutex);
	return r;
}

static void
_activation_retry_inc_failures(ActivationRetry_t * retry)
{
	if (retry) {
		MutexAcquire(&retry->mutex);
		++ retry->failures;
		MutexRelease(&retry->mutex);
	}
}

static void
_activation_retry_init(ActivationRetry_t * retry)
{
	MutexInitState(&retry->mutex);
	MutexInit(&retry->mutex);
}

static void
_activation_retry_destroy(ActivationRetry_t * retry)
{
	MutexDestroy(&retry->mutex);
}

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
	ActivationRetry_t * retry;
} ActivateWorkItem_t;

static ActivateWorkItem_t *
_activate_workitem_alloc(Node_t *nodep, ActivationRetry_t * retry, 
	PsWorker_t workfunc)
{
	ActivateWorkItem_t *workitem = NULL;
	if (vs_pool_alloc(&sm_pool, sizeof(ActivateWorkItem_t),
		(void**)&workitem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workitem, 0, sizeof(ActivateWorkItem_t));

	workitem->nodep = nodep;
	workitem->retry = retry;
	workitem->item.workfunc = workfunc;

	return workitem;
}

static void
_activate_workitem_free(ActivateWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static __inline__ int
_needs_reregistration(Node_t * nodep, Port_t * portp, ActivationRetry_t * retry)
{
	// general conditions are only checked on first attempt.
	// on retries, only reregister if it remains pending from previous attempts
	if (_activation_retry_attempts(retry))
		return portp->poportp->registration.reregisterPending;

	// reregister if new sm
	if (topology_passcount < 1) return TRUE;

	// reregister if previous attempts haven't yet succeeded
	if (portp->poportp->registration.reregisterPending) return TRUE;

	// reregister if node/port is new this sweep
	Node_t * oldnodep = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
	if (!oldnodep) return TRUE;
	Port_t * oldportp = sm_find_node_port(&old_topology, oldnodep, portp->index);
	if (!oldportp) return TRUE;

	// reregister if old port was DOWN
	// (implying we removed its groups/subscriptions)
	return oldportp->state <= IB_PORT_DOWN;
}

static void
_handle_activate_failure(Port_t * portp)
{
	// limit resweeps, if a device consistently can't be activated, it may
	// have a HW or config issue
	if (++portp->portData->numFailedActivate <= ACTIVATE_RETRY_LIMIT)
		sm_request_resweep(1, 1, SM_SWEEP_REASON_ACTIVATE_FAIL);
}

static void
_handle_activate_bounce(Node_t * nodep, Port_t * portp)
{
	if(sm_config.portBounceLogLimit == PORT_BOUNCE_LOG_NO_LIMIT ||
		topology_port_bounce_log_num < sm_config.portBounceLogLimit) {

		IB_LOG_WARN_FMT(__func__, "Arm/activate port for node %s guid " FMT_U64
			" port %d: port has bounced since programming start, requesting resweep",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);

		topology_port_bounce_log_num++;

		if(topology_port_bounce_log_num == sm_config.portBounceLogLimit) {
			IB_LOG_WARN_FMT(__func__,
				"Port bounce log limit of %d reached. Further port bounce events will be suppressed.",
				sm_config.portBounceLogLimit);
		}
	}

	sm_request_resweep(0, 0, SM_SWEEP_REASON_UNEXPECTED_BOUNCE);
}

static void
_handle_activate_retry(Node_t * nodep, Port_t * portp, ActivationRetry_t * retry)
{
	_activation_retry_inc_failures(retry);

	if(_activation_retry_attempts(retry) == sm_config.neighborNormalRetries) {
		IB_LOG_ERROR_FMT(__func__,
			"Port will not go active for node %s nodeGuid "FMT_U64" port %d after %d retries",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, _activation_retry_attempts(retry));
		_handle_activate_failure(portp);
	}
}

static Status_t
_arm_port(SmMaiHandle_t *ib_fd, Topology_t * topop, Node_t * nodep, Port_t * portp,
	ParallelSweepContext_t *psc)
{
	Status_t status = VSTATUS_OK;
	STL_PORT_INFO portInfo;
	STL_LID dlid;
	uint32_t madStatus = 0;
	SmpAddr_t addr;

	IB_ENTER(__func__, topop, nodep, portp, 0);

	//do not try to activate ports connected to quarantined nodes
	if(portp->portData->neighborQuarantined) {
		IB_EXIT(__func__, status);
		return (status);
	}

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t *swportp;
		swportp = sm_get_port(nodep, 0);
		if (!sm_valid_port(swportp)) {
			IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
							nodep->nodeInfo.NodeGUID);
			status = VSTATUS_BAD;
			goto bail;
		}
		dlid = swportp->portData->lid;
	} else {
		dlid = portp->portData->lid;
	}

	portInfo = portp->portData->portInfo;

	if (portInfo.PortStates.s.PortState < IB_PORT_INIT) {
		IB_LOG_WARN_FMT(__func__, "Node %s guid "FMT_U64
			" port %u isn't in INIT state.",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
		status = VSTATUS_BAD;
		goto bail;
	} else if (portInfo.PortStates.s.PortState > IB_PORT_INIT) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Node %s guid "FMT_U64
			" port %u already armed or active.",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
		status = VSTATUS_BAD;
		goto bail;
	}

	// Set the "No change" attributes.
	sm_portinfo_nop_init(&portInfo);

	portInfo.PortStates.s.PortState = IB_PORT_ARMED;
	portInfo.LinkDownReason = STL_LINKDOWN_REASON_NONE;
	portInfo.NeighborLinkDownReason = STL_LINKDOWN_REASON_NONE;


	if (sm_is_scae_allowed(nodep))
		portInfo.PortMode.s.IsActiveOptimizeEnabled = 1;

	//
	//  Tell the port its new state.
	//
	SMP_ADDR_SET_LR(&addr, sm_lid, dlid);
	psc_unlock(psc);
	status = SM_Set_PortInfo(ib_fd, (1 << 24) | (portp->index),
		&addr, &portInfo, portp->portData->portInfo.M_Key, &madStatus);
	psc_lock(psc);
	if (status != VSTATUS_OK && madStatus != MAD_STATUS_INVALID_ATTRIB) {
		IB_LOG_WARN_FMT(__func__,
			"Cannot set PORTINFO for node %s nodeGuid " FMT_U64
			" port %d status=%d", sm_nodeDescString(nodep),
			nodep->nodeInfo.NodeGUID, portp->index, status);

	} else if(madStatus == MAD_STATUS_INVALID_ATTRIB &&
		portInfo.PortStates.s.IsSMConfigurationStarted == 0) {
		_handle_activate_bounce(nodep, portp);
	} else if (portInfo.PortStates.s.PortState != IB_PORT_ARMED &&
		!(madStatus == MAD_STATUS_INVALID_ATTRIB)) {

		IB_LOG_WARN_FMT(__func__,
						"Activate port for node %s guid " FMT_U64
						" port %d: tried to set state to %d but returned %d",
						sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index,
						IB_PORT_ARMED, portInfo.PortStates.s.PortState);
		// limit resweeps, if a device consistently can't be activated, it may
		// have a HW or config issue
		_handle_activate_failure(portp);
	}

	// To save an additional Set(PortInfo) per port, the HOQLife/XmitQ values may not have been
	// sent for a port in topology_assignments() if at that time the port was not
	// ARMED or ACTIVE since at least one more Set(PortInfo) will be required to move
	// the port to ARMED.  Thus it is necessary to check the request/response
	// XmitQ values here as well
	if (status == VSTATUS_OK &&
		!sm_eq_XmitQ(portInfo.XmitQ, portp->portData->portInfo.XmitQ, STL_MAX_VLS)) {
		IB_LOG_ERROR_FMT(__func__,
			 "XmitQ requested/response value mismatch for node %s guid " FMT_U64
			 " port %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
			 portp->index);
		status = VSTATUS_BAD;
	} else if (status == VSTATUS_OK) {
		//
		//  Save the port state and the port info
		//
		portp->state = portInfo.PortStates.s.PortState;
		portp->portData->portInfo = portInfo;
		sm_popo_update_port_state(&sm_popo, portp, &portp->portData->portInfo.PortStates);

		// Clear the LED if it was previously turned on.
		psc_unlock(psc);
		sm_enable_port_led(ib_fd, nodep, portp, FALSE);
		psc_lock(psc);
	}

bail:
	IB_EXIT(__func__, status);
	return (status);
}

static Port_t *
_arm_switch(SmMaiHandle_t *ib_fd, Topology_t *topop, Node_t *switchp,
	ParallelSweepContext_t *psc)
{
	Port_t *swportp, *portp;
	STL_LID dlid;
	const size_t blockSize = sizeof(STL_AGGREGATE) + 8*((sizeof(STL_PORT_INFO) + 7)/8) ;
	const size_t portsPerAggr = STL_MAX_PAYLOAD_SMP_LR / blockSize;
	uint8_t buffer[STL_MAX_PAYLOAD_SMP_LR] = {0};
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	uint8_t resetIndex = 0;
	uint8_t enqueued = 0;
	Status_t status;

	if (!sm_valid_port(swportp = sm_get_port(switchp, 0))) {
		IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						switchp->nodeInfo.NodeGUID);
		return swportp;
	}
	dlid = swportp->portData->lid;

	for_all_ports(switchp, portp) {
		STL_PORT_INFO *pPortInfo = (STL_PORT_INFO*)aggrHdr->Data;

		if (sm_valid_port(portp) && !(portp->portData->neighborQuarantined)) {
			if (portp->state != IB_PORT_INIT) {
				if (portp->state == IB_PORT_DOWN && portp->portData->linkPolicyViolation) {
					//port marked down administratively for link policy violation but real port state
					//should still be in init, so set linkInitReason here
					sm_set_linkinit_reason(switchp, portp, STL_LINKINIT_OUTSIDE_POLICY);
					psc_unlock(psc);
					sm_enable_port_led(ib_fd, switchp, portp, TRUE);
					psc_lock(psc);
				}

			} else {
				aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PORT_INFO;
				aggrHdr->Result.s.Error = 0;
				aggrHdr->Result.s.Reserved = 0;
				aggrHdr->Result.s.RequestLength = (sizeof(STL_PORT_INFO) + 7)/8;

				aggrHdr->AttributeModifier =  (1<<24) | (uint32_t)portp->index;

				*pPortInfo = portp->portData->portInfo;

				// Set the "No change" attributes.
				sm_portinfo_nop_init(pPortInfo);

				pPortInfo->PortStates.s.PortState = IB_PORT_ARMED;
				pPortInfo->LinkDownReason = STL_LINKDOWN_REASON_NONE;
				pPortInfo->NeighborLinkDownReason = STL_LINKDOWN_REASON_NONE;
				pPortInfo->PortStates.s.LEDEnabled = 0;

				if (sm_is_scae_allowed(switchp))
					pPortInfo->PortMode.s.IsActiveOptimizeEnabled = 1;

				BSWAP_STL_PORT_INFO(pPortInfo);
				aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

				if (enqueued++ < 1)
					resetIndex = portp->index;
			}
		}

		// Send out the Aggregate.
		if ((enqueued > 0) &&
			(portp->index == switchp->nodeInfo.NumPorts || (enqueued == portsPerAggr))) {
			uint32_t madStatus = 0;
			STL_AGGREGATE *lastSeg = NULL;

			psc_unlock(psc);
			status = SM_Set_Aggregate_LR(ib_fd, (STL_AGGREGATE*)buffer,
				aggrHdr, sm_lid, dlid, sm_config.mkey, &lastSeg, &madStatus);
			psc_lock(psc);

			// Process the results
			if (lastSeg) {
				for (aggrHdr = (STL_AGGREGATE*)buffer; aggrHdr < lastSeg; aggrHdr = STL_AGGREGATE_NEXT(aggrHdr)) {
					Port_t *retPort;

					retPort = sm_get_port(switchp, aggrHdr->AttributeModifier & 0xff);
					if (!retPort) {
						IB_LOG_ERROR_FMT(__func__, "Invalid port number in Set(PortInfo) response for node %s",
										sm_nodeDescString(switchp));
						continue;
					}

					pPortInfo = (STL_PORT_INFO*)aggrHdr->Data;

					// Aggregate fails, re-process this port the old fashion way.
					if (aggrHdr->Result.s.Error) {
						IB_LOG_ERROR_FMT(__func__, "Error on Set(Aggregate) of PortInfo for port %d of node %s",
										retPort->index, sm_nodeDescString(switchp));
						// All following Set(PortInfo) in the aggregate weren't processed. Rebuild aggregate from port following failure.
						portp = retPort;
						break;
					}
					BSWAP_STL_PORT_INFO(pPortInfo);


					// Some error cases that allow us to continue working on the Aggregate, but should be handled (taken from sm_activate_port)
					if (pPortInfo->PortStates.s.PortState != IB_PORT_ARMED && pPortInfo->PortStates.s.NeighborNormal != 0) {
						IB_LOG_WARN_FMT(__func__, "Arm port for node %s guid " FMT_U64" port %d failed",
										sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, retPort->index);

						// limit resweeps, if a device consistently can't be activated, it may
						// have a HW or config issue
						if (++retPort->portData->numFailedActivate <= ACTIVATE_RETRY_LIMIT)
							sm_request_resweep(1, 1, SM_SWEEP_REASON_ACTIVATE_FAIL);

						continue;
					}

					if (!sm_eq_XmitQ(pPortInfo->XmitQ, retPort->portData->portInfo.XmitQ, STL_MAX_VLS)) {
						IB_LOG_ERROR_FMT(__func__,
										 "XmitQ requested/response value mismatch for node %s guid " FMT_U64
										 " port %d", sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID,
										 retPort->index);
						continue;
					}


					retPort->state = pPortInfo->PortStates.s.PortState;
					retPort->portData->portInfo = *pPortInfo;
					if (retPort->state == IB_PORT_ACTIVE)
						retPort->portData->numFailedActivate = 0;

					psc_unlock(psc);
					sm_enable_port_led(ib_fd, switchp, retPort, FALSE);
					psc_lock(psc);
					bitset_clear(&switchp->initPorts, retPort->index);
				}
			} else {
				// Aggregate fell flat on it's face. Reset the port pointer, we'll build another one
				// from the next port.
				IB_LOG_ERROR_FMT(__func__, "Error on Set(Aggregate) for Node "FMT_U64 ":%s, status = %d",
								switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), status);
				portp = sm_get_port(switchp, resetIndex);
			}

			// ARM this port manually if we had Aggregate troubles.
			if (!lastSeg || aggrHdr->Result.s.Error) {
				if (sm_valid_port(portp) && _arm_port(ib_fd, topop, switchp, portp,
					psc) != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "TT(ta): can't ARM switch %s "
						"nodeGuid "FMT_U64" node index %d port index %d",
						sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID,
						switchp->index, portp->index);
					psc_unlock(psc);
					sm_enable_port_led(ib_fd, switchp, portp, TRUE);
					psc_lock(psc);
				}
			}

			aggrHdr = (STL_AGGREGATE*)buffer;
			enqueued = 0;
		}
	}

	// NULL = Success
	return NULL;
}

Status_t
sm_activate_port(SmMaiHandle_t *fd, Topology_t * topop, Node_t * nodep, Port_t * portp,
	uint8_t forceReregister, ActivationRetry_t * retry)
{
	Status_t status = VSTATUS_OK;
	STL_PORT_INFO portInfo;
	STL_LID dlid;
	uint32_t madStatus = 0;
	int reregisterable = TRUE;
	SmpAddr_t addr;

	IB_ENTER(__func__, topop, nodep, portp, forceReregister);

	//do not try to activate ports connected to quarantined nodes
	if(portp->portData->neighborQuarantined)
		return VSTATUS_OK;

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t *swportp;
		swportp = sm_get_port(nodep, 0);
		if (!sm_valid_port(swportp)) {
			IB_LOG_WARN_FMT(__func__,
				"Failed to find node %s nodeGuid "FMT_U64" port 0",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
			sm_enable_port_led(fd, nodep, portp, TRUE);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		dlid = swportp->portData->lid;

		reregisterable = portp->index == 0 && nodep->switchInfo.u2.s.EnhancedPort0;
	} else {
		dlid = portp->portData->lid;
	}

	portInfo = portp->portData->portInfo;

	if (portInfo.PortStates.s.PortState < IB_PORT_ARMED) {
		IB_LOG_WARN_FMT(__func__,
			"Node %s nodeGuid "FMT_U64" port %u: state isn't ARMED or ACTIVE; state=%u",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, portInfo.PortStates.s.PortState);
		sm_enable_port_led(fd, nodep, portp, TRUE);
		IB_EXIT(__func__, VSTATUS_BAD);
		return (VSTATUS_BAD);
	}

	if (portInfo.PortStates.s.PortState == IB_PORT_ACTIVE) {
		if (reregisterable && (_needs_reregistration(nodep, portp, retry) || forceReregister)) {
			// In this case, the port is already active but we need to tell it to
			// re-register any multicast groups. This might be because the FM was
			// restarted or because the node just appeared in the fabric and
			// ActiveOptimize is enabled.

			// Indicate that reregistration is pending; will persist across
			// sweeps until it succeeds or is no longer required (e.g. state change)
			portp->poportp->registration.reregisterPending = 1;
		}
		// If the port is already active, we don't need to actually send the MAD.
		portp->state = portInfo.PortStates.s.PortState;
		portp->portData->numFailedActivate = 0;
		IB_EXIT(__func__, VSTATUS_OK);
		return (VSTATUS_OK);
	}

	// Set the "No change" attributes.
	sm_portinfo_nop_init(&portInfo);

	portInfo.PortStates.s.PortState = IB_PORT_ACTIVE;

	//
	//  Tell the port its new state.
	//
	SMP_ADDR_SET_LR(&addr, sm_lid, dlid);
	status = SM_Set_PortInfo(fd, (1 << 24) | (portp->index),
		&addr, &portInfo, portp->portData->portInfo.M_Key, &madStatus);

	if (status != VSTATUS_OK && madStatus != MAD_STATUS_INVALID_ATTRIB) {
		IB_LOG_WARN_FMT(__func__,
			"Cannot set PORTINFO for node %s nodeGuid "FMT_U64" port %d: status=%d madStatus=%u",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, status, madStatus);
		sm_enable_port_led(fd, nodep, portp, TRUE);
		IB_EXIT(__func__, status);
		return (status);
	}

	// check for failures to activate
	if (portInfo.PortStates.s.PortState != IB_PORT_ACTIVE) {
		if (madStatus == MAD_STATUS_INVALID_ATTRIB && !portInfo.PortStates.s.IsSMConfigurationStarted) {
			_handle_activate_bounce(nodep, portp);
		} else if (madStatus == MAD_STATUS_INVALID_ATTRIB && portp->index > 0 && !portInfo.PortStates.s.NeighborNormal) {
			// check for obvious "neighbor normal not ready" failures
			_handle_activate_retry(nodep, portp, retry);
		} else if (madStatus == MAD_STATUS_INVALID_ATTRIB && portp->index > 0
				&& !portp->portData->portInfo.PortStates.s.NeighborNormal
				&& portInfo.PortStates.s.NeighborNormal) {
			// we have observed NeighborNormal transitioning while the SMA
			// is processing the Set.  if NeighborNormal changed state during
			// this Set, conservatively assume it was a race and trigger a
			// retry.  if it fails on the retry, we'll fall through to the
			// general failure case.
			_handle_activate_retry(nodep, portp, retry);
		} else {
			IB_LOG_WARN_FMT(__func__,
				"Activate port for node %s nodeGuid "FMT_U64" port %d: tried to set state to %d but returned %d",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index,
				IB_PORT_ACTIVE, portInfo.PortStates.s.PortState);
			_handle_activate_failure(portp);
		}
	}

	//
	//  Save the port state and the port info
	//
	portp->state = portInfo.PortStates.s.PortState;
	portp->portData->portInfo = portInfo;
	sm_popo_update_port_state(&sm_popo, portp, &portp->portData->portInfo.PortStates);
	if (portp->state == IB_PORT_ACTIVE)
		portp->portData->numFailedActivate = 0;

	IB_EXIT(__func__, VSTATUS_OK);
	return (VSTATUS_OK);
}

static Status_t
_activate_switch_via_portinfo(SmMaiHandle_t *fd, Topology_t * topop, Node_t * nodep,
	ActivationRetry_t * retry, ParallelSweepContext_t *psc)
{
	Status_t status;
	Port_t * portp;

	IB_ENTER(__func__, topop, nodep, retry, 0);

	for_all_physical_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state < IB_PORT_ARMED) continue;
		psc_unlock(psc);
		status = sm_activate_port(fd, topop, nodep, portp, FALSE, retry);
		psc_lock(psc);
		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				"Failed to activate node %s nodeGuid "FMT_U64" port 0; status=%u",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
			if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
				return status;
		}
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

static Status_t
_activate_switch_via_portstateinfo(SmMaiHandle_t *fd, Topology_t * topop,
	Node_t * nodep, Port_t * port0p, ActivationRetry_t * retry, 
	ParallelSweepContext_t *psc)
{
	Status_t status = VSTATUS_OK;
	int i;

	IB_ENTER(__func__, topop, nodep, retry, 0);

	STL_PORT_STATE_INFO * psi = NULL;
	psc_unlock(psc);
	status = sm_set_node_port_states(fd, topop, nodep, port0p, NULL, IB_PORT_ARMED,
		IB_PORT_ACTIVE, &psi);
	psc_lock(psc);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
			"Failed Set(PortStateInfo) for node %s nodeGuid "FMT_U64": status=%u",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);

		if ((status = sm_popo_port_error(&sm_popo, sm_topop, port0p, status)) == VSTATUS_TIMEOUT_LIMIT)
			return status;

		// if Set failed, see if we can Get and take advantage of any ports
		// that succeeded
		status = sm_get_node_port_states(fd, topop, nodep, port0p, NULL, &psi, psc);
		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				"Failed Get(PortStateInfo) for node %s nodeGuid "FMT_U64": status=%u",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);

			if ((status = sm_popo_port_error(&sm_popo, sm_topop, port0p, status)) == VSTATUS_TIMEOUT_LIMIT)
				return status;

			// last resort... hit each port individually
			status = _activate_switch_via_portinfo(fd, topop, nodep, retry, psc);
			IB_EXIT(__func__, status);
			return status;
		}
	}

	if (!psi) {
		// success and no buffer?  nothing needed to be done
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	// we have a buffer: either Set succeeded, or it failed and we did a Get
	for(i = 0; i <= nodep->nodeInfo.NumPorts; i++) {
		Port_t * portp = sm_get_port(nodep, i);
		if (!sm_valid_port(portp) || portp->state < IB_PORT_ARMED) continue;

		portp->portData->portInfo.PortStates = psi[i].PortStates;
		portp->state = psi[i].PortStates.s.PortState;

		switch (psi[i].PortStates.s.PortState) {
			case IB_PORT_ACTIVE:
				// success
				portp->portData->numFailedActivate = 0;
				break;
			case IB_PORT_ARMED:
				// still armed or never got processed
				if (!psi[i].PortStates.s.IsSMConfigurationStarted) {
					_handle_activate_bounce(nodep, portp);
				} else if (i > 0 && !psi[i].PortStates.s.NeighborNormal) {
					_handle_activate_retry(nodep, portp, retry);
				} else {
					// looks okay; most likely that the Set(PortStateInfo) failed
					// before reaching this port, or that NeighborNormal propagated
					// between the set and get.  attempt directly via Set(PortInfo)
					psc_unlock(psc);
					status = sm_activate_port(fd_topology, topop, nodep, portp, FALSE, retry);
					psc_lock(psc);
					if (status != VSTATUS_OK) {
						IB_LOG_WARN_FMT(__func__,
							"Failed to activate node %s nodeGuid "FMT_U64" port 0: status=%u",
							sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
						if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
							break;
					}
				}
				break;
			default:
				// state < ARMED: port bounce
				_handle_activate_bounce(nodep, portp);
				break;
		}
	}

	vs_pool_free(&sm_pool, psi);

	IB_EXIT(__func__, status);
	return status;
}

static Status_t
_activate_switch(SmMaiHandle_t *fd, Topology_t * topop, Node_t * nodep,
	ActivationRetry_t * retry, ParallelSweepContext_t *psc)
{
	Status_t status;

	IB_ENTER(__func__, topop, nodep, retry, 0);

	Port_t *port0p = sm_get_port(nodep, 0);
	if (!sm_valid_port(port0p) || port0p->state < IB_PORT_INIT) {
		IB_LOG_WARN_FMT(__func__,
			"Switch %s nodeGuid "FMT_U64" has invalid port 0: unable to activate",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);

		// fail outright without access to port 0
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	// activate port 0 via Set(PortInfo) if
	//   - it's enhanced, AND...
	//       - we're ARMED and retrying (as we only Set(PortStateInfo) on first attempt)
	//       - we're ACTIVE and need a reregistration
	if (  nodep->switchInfo.u2.s.EnhancedPort0
	   && (  (port0p->state == IB_PORT_ARMED && _activation_retry_attempts(retry))
	      || (port0p->state == IB_PORT_ACTIVE && _needs_reregistration(nodep, port0p, retry))))
	{
		psc_unlock(psc);
		status = sm_activate_port(fd, topop, nodep, port0p, FALSE, retry);
		psc_lock(psc);
		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				"Failed to activate node %s nodeGuid "FMT_U64" port 0: status=%u",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
			if ((status = sm_popo_port_error(&sm_popo, sm_topop, port0p, status)) == VSTATUS_TIMEOUT_LIMIT)
				return status;
		}
	}

	// for remaining switch ports, use PortStateInfo on the first attempt
	status = _activation_retry_attempts(retry)
		? _activate_switch_via_portinfo(fd, topop, nodep, retry,  psc)
		: _activate_switch_via_portstateinfo(fd, topop, nodep, port0p, retry,  psc);

	IB_EXIT(__func__, status);
	return status;
}

static Status_t
_activate_hfi(SmMaiHandle_t *fd, Topology_t * topop, Node_t * nodep,
	ActivationRetry_t * retry, ParallelSweepContext_t *psc)
{
	Status_t status;
	Port_t * portp;

	IB_ENTER(__func__, topop, nodep, retry, 0);

	for_all_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state < IB_PORT_ARMED) continue;
		psc_unlock(psc);
		status = sm_activate_port(fd, topop, nodep, portp, FALSE, retry);
		psc_lock(psc);
		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				"Failed to activate node %s nodeGuid "FMT_U64" port %u: status=%u",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, status);
			if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
				return status;
		}
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

static Status_t
_arm_node(Topology_t *topop, Node_t *nodep, ParallelSweepContext_t *psc)
{
	Status_t status = VSTATUS_OK;
	Port_t *portp;

	MaiPool_t *mpp = psc_get_mai(psc);
	if (!mpp) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate MAI handle.");
		return VSTATUS_NOMEM;
	}

	psc_lock(psc);
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && sm_config.use_aggregates)  {
		_arm_switch(mpp->fd, topop, nodep, psc);
	} else {
		for_all_ports(nodep, portp) {
			if (sm_valid_port(portp) && portp->state == IB_PORT_INIT) {
				status = _arm_port(mpp->fd, topop, nodep, portp, psc);
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "TT(ta): can't ARM node %s nodeGuid "FMT_U64" node index %d port index %d",
									sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
					psc_unlock(psc);
					sm_enable_port_led(mpp->fd, nodep, portp, TRUE);
					psc_lock(psc);
					if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
						goto bail;
				} else {
					bitset_clear(&nodep->initPorts, portp->index);
				}
			} else if(sm_valid_port(portp) && portp->state == IB_PORT_DOWN && portp->portData->linkPolicyViolation) {
				//port marked down administratively for link policy violation but real port state
				//should still be in init, so set linkInitReason here
				sm_set_linkinit_reason(nodep, portp, STL_LINKINIT_OUTSIDE_POLICY);
				psc_unlock(psc);
				sm_enable_port_led(mpp->fd, nodep, portp, TRUE);
				psc_lock(psc);
			}
		}
	}

bail:
	psc_unlock(psc);
	psc_free_mai(psc, mpp);
	return status;
}

// Activates a node "efficiently", e.g. leveraging PortStateInfo when available.
static void
_activate_node(ParallelSweepContext_t *psc, 
	ParallelWorkItem_t *pwi)
{
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	ActivateWorkItem_t * wi = PARENT_STRUCT(pwi, ActivateWorkItem_t, item);
	Node_t *nodep = wi->nodep;

	MaiPool_t *mpp = psc_get_mai(psc);
	if (!psc || !mpp) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate MAI handle.");
		status = VSTATUS_NOMEM;
		goto bail;
	}


	psc_lock(psc);
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		status = _activate_switch(mpp->fd, sm_topop, nodep, wi->retry, psc);
	} else if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
		status = _activate_hfi(mpp->fd, sm_topop, nodep, wi->retry, psc);
	}
	psc_unlock(psc);

bail:
	psc_free_mai(psc, mpp);
	psc_set_status(psc, status);
	if (status == VSTATUS_TIMEOUT_LIMIT) psc_stop(psc);
	_activate_workitem_free(wi);

	IB_EXIT(__func__, status);
}

// Activates a node conservatively: one port at a time via PortInfo only.
static void 
_safe_activate_node(ParallelSweepContext_t *psc,
	ParallelWorkItem_t *pwi)
{
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	ActivateWorkItem_t * wi = PARENT_STRUCT(pwi, ActivateWorkItem_t, item);
	Port_t *portp = NULL;
	Node_t *nodep = wi->nodep;

	MaiPool_t *mpp = psc_get_mai(psc);
	if (!psc || !mpp) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate MAI handle.");
		status = VSTATUS_NOMEM;
	} else {
		psc_lock(psc);
		for_all_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state < IB_PORT_ARMED) continue;
			psc_unlock(psc);
			status = sm_activate_port(mpp->fd, sm_topop, nodep, portp, FALSE, wi->retry);
			psc_lock(psc);
			if (status != VSTATUS_OK) {
				IB_LOG_WARN_FMT(__func__,
					"Failed to activate node %s nodeGuid "FMT_U64" port %u; status=%u",
					sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, status);
				if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT)
					psc_stop(psc);
				break;
			}
		}
		psc_unlock(psc);

		psc_free_mai(psc, mpp);
	}

	psc_set_status(psc, status);
	_activate_workitem_free(wi);

	IB_EXIT(__func__, status);
}

static Status_t
_activate_all_hfi_first_safe(ParallelSweepContext_t *psc, ActivationRetry_t * retry)
{
	Status_t status = VSTATUS_OK;
	Node_t * nodep;
	ActivateWorkItem_t *awip;

	for_all_ca_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, retry, _safe_activate_node);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &awip->item);
	}

	if (status == VSTATUS_OK) for_all_switch_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, retry, _safe_activate_node);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &awip->item);
	}

	return status;
}

static Status_t
_activate_all_hfi_first(ParallelSweepContext_t *psc, ActivationRetry_t * retry)
{
	Status_t status = VSTATUS_OK;
	Node_t * nodep;
	ActivateWorkItem_t *awip;

	for_all_ca_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, retry, _activate_node);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &awip->item);
	}

	if (status == VSTATUS_OK) for_all_switch_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, retry, _activate_node);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &awip->item);
	}

	return status;
}

static Status_t
_activate_all_switch_first(ParallelSweepContext_t *psc, Topology_t * topop, ActivationRetry_t * retry)
{
	Status_t status = VSTATUS_OK;
	Node_t * nodep;
	ActivateWorkItem_t *awip;

	for_all_switch_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, retry, _activate_node);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &awip->item);
	}

	if (status == VSTATUS_OK) for_all_ca_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, retry, _activate_node);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &awip->item);
	}

	return status;
}

static void
_set_all_reregisters_callback(cntxt_entry_t *cntxt, Status_t status, void *data, Mai_t *mad)
{
	Node_t *nodep = (Node_t *)data;
	Port_t *portp = (nodep ? sm_get_node_end_port(nodep) : NULL);
	boolean skip = (sm_config.skipAttributeWrite & SM_SKIP_WRITE_PORTINFO ? 1 : 0);

	if (!skip && !sm_callback_check(cntxt, status, nodep, portp, mad)) {
		// Handle Failure
	} else if (sm_valid_port(portp)) {
		portp->poportp->registration.reregisterPending = 0;
	}
}

Status_t
sm_set_all_reregisters(void)
{
	Node_t *nodep;
	Port_t *portp;
	Status_t status;

	for_all_nodes(&old_topology, nodep) {
		portp = sm_get_node_end_port(nodep);
		if (!sm_valid_port(portp) || portp->state != IB_PORT_ACTIVE) continue;
		if (!portp->poportp->registration.reregisterPending) continue;

		/* Only need to lock the Copy out of the port */
		(void)vs_rdlock(&old_topology_lock);
		STL_PORT_INFO pi = portp->portData->portInfo;
		(void)vs_rwunlock(&old_topology_lock);

		// Set the "No change" attributes.
		sm_portinfo_nop_init(&pi);

		pi.Subnet.ClientReregister = 1;

		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, portp->portData->lid);
		status = SM_Set_PortInfo_Dispatch(fd_topology, (1 << 24) | portp->index,
			&addr, &pi, portp->portData->portInfo.M_Key, nodep, &sm_asyncDispatch,
			_set_all_reregisters_callback, nodep);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to Send Set Client Reregister on port of node %s nodeGuid "FMT_U64" port %u",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
		}
	}

	status = sm_dispatch_wait(&sm_asyncDispatch);
	if (status != VSTATUS_OK) {
		sm_dispatch_clear(&sm_asyncDispatch);
		IB_LOG_INFINI_INFO_FMT(__func__,
			"Failed to wait on the dispatch queue for Client Reregistration rc: %s",
			cs_convert_status(status));
	}

	return status;
}

static void
_arm_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *psi)
{
	Status_t status;
	ActivateWorkItem_t *awip = PARENT_STRUCT(psi, ActivateWorkItem_t, item);

	status = _arm_node(sm_topop, awip->nodep, psc);
	if (status == VSTATUS_TIMEOUT_LIMIT) {
		psc_set_status(psc, status);
		psc_stop(psc);
	}

	_activate_workitem_free(awip);
}

Status_t
sweep_arm(SweepContext_t *sweep_context)
{
	Status_t	status = VSTATUS_OK;
	Node_t		*nodep;
	ActivateWorkItem_t *awip;

	IB_ENTER(__func__, 0, 0, 0, 0);

	// If we aren't the master SM, then we go no further.
	if (sm_state != SM_STATE_MASTER) {
		status = VSTATUS_NOT_MASTER;
		goto bail;
	}

	// We are about to activate new nodes, so mark us as sweeping now.
	activateInProgress = 1;
	topology_port_bounce_log_num = 0;

	// Enable the workers.
	psc_go(sweep_context->psc);

	// Transition ports from INIT to ARMED per DN0567

	for_all_ca_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, 0, _arm_worker);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(sweep_context->psc, &awip->item);
	}

	if (status == VSTATUS_OK) for_all_switch_nodes(sm_topop, nodep) {
		awip = _activate_workitem_alloc(nodep, 0, _arm_worker);
		if (awip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(sweep_context->psc, &awip->item);
	}

	if (status == VSTATUS_OK) {
		// Wait for the workers to arm all the ports.
		status = psc_wait(sweep_context->psc);
	} else {
		// In this case, we failed to queue all the work items, so
		// issue a terminate and wait for the workers to quiesce.
		psc_stop(sweep_context->psc);
		(void)psc_wait(sweep_context->psc);
	}

bail:
	// When we get here, we're either done and the work queue is empty
	// or we hit an error and stopped early. Either way, drain the work
	// item queue just to be sure.
	psc_drain_work_queue(sweep_context->psc);
	IB_EXIT(__func__, status);
	return status;
}

Status_t
sweep_activate(SweepContext_t *sweep_context)
{
	Status_t	status = VSTATUS_OK;
	ActivationRetry_t retry = {0};

	IB_ENTER(__func__, 0, 0, 0, 0);

	//	If we aren't the master SM, then we go no further.
	if (sm_state != SM_STATE_MASTER)
		return VSTATUS_NOT_MASTER;

	/* we are about to activate new nodes so mark us as sweeping now */
	activateInProgress = 1;
	topology_port_bounce_log_num = 0;

	// sending of armed idle flits can be delayed by several millis, preventing
	// activation due to NeighborNormal not being set.  simply wait out the
	// expected propagation time to avoid spamming the log with failures due
	// to expected conditions, and to avoid complicating the code with edge
	// cases.  for sweeps proportional to this time, performance isn't critical,
	// and for significantly longer sweeps, this delay is a non-issue
	vs_thread_sleep(IDLE_FLIT_PROPAGATION_TIME);

	_activation_retry_init(&retry);

	// Transition ports from ARMED to ACTIVE per DN0567
	do {

		psc_go(sweep_context->psc);
		retry.failures = 0;

		switch (sm_config.switchCascadeActivateEnable) {
			case SCAE_SW_ONLY:
				// only switches will cascade, as HFI activation should be paced.
				// activate HFIs first to allow traffic flow a bit sooner and
				// switches will implicitly cascade as a result
				status = _activate_all_hfi_first(sweep_context->psc, &retry);
				break;
			case SCAE_ALL:
				// all ports will cascade: do switches first for faster overall
				// cascade at scale, relying on activating switches to implicitly
				// activate hfis
				status = _activate_all_switch_first(sweep_context->psc, sm_topop, &retry);
				break;
			default:
				// failsafe activation: activate ports serially via Set(PortInfo)
				// only. activate HFIs first to allow traffic flow a bit sooner
				status = _activate_all_hfi_first_safe(sweep_context->psc, &retry);
				break;
		}
		if (status == VSTATUS_OK) {
			// Wait for the workers to arm all the ports.
			status = psc_wait(sweep_context->psc);
		} else {
			// In this case, we failed to queue all the work items, so
			// issue a terminate and wait for the workers to quiesce.
			// Note that we preserve the previous status
			psc_stop(sweep_context->psc);
			(void)psc_wait(sweep_context->psc);
		}

		// When we got here, we're either done and the work queue is empty
		// or we hit an error and stopped early. Either way, drain the work
		// item queue just to be sure.
		psc_drain_work_queue(sweep_context->psc);

		if (sm_debug && (retry.failures || retry.attempts))
			IB_LOG_WARN_FMT(__func__, "%u ports failed to go active on "
				"attempt %u", retry.failures, retry.attempts);

		if (retry.failures)
			vs_thread_sleep(MIN(VTIMER_10_MILLISEC * (1 << retry.attempts),
				VTIMER_1S));

	} while ((status != VSTATUS_TIMEOUT_LIMIT) &&
		(status != VSTATUS_UNRECOVERABLE) && (retry.failures &&
		++retry.attempts <= sm_config.neighborNormalRetries));

	if (retry.failures) {
		IB_LOG_ERROR_FMT(__func__, "%d ports failed to go active due to NeighborNormal never being set", retry.failures);
	}

	_activation_retry_destroy(&retry);
	return status;
}
