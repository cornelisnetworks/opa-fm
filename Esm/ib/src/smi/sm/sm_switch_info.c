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

#include "sm_l.h"
#include "sm_switch_info.h"

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} GetSwInfoWorkItem_t;

static GetSwInfoWorkItem_t *
_switch_info_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	GetSwInfoWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(GetSwInfoWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workItem, 0, sizeof(GetSwInfoWorkItem_t));

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_switch_info_work_item_free(GetSwInfoWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

// -Iterate over all switches in the fabric and update the switch info atttribute
// -Retry getting switch info if it had failed earlier during discovery sweep.
// -Verify RoutingMode and change only when ports are not Active.
// -Configure set swinfo attibute's routing related components
//     based on the enabled RoutingMode.
// -Handle, in linear forwarding mode, the corresponding FDBTop and FDBCap.
// -Set routing pause state during the changes to the routing.
// -Check if MFTCap is sufficient and set multi-cast FDB Top.
// -Check for state change in any of switch ports and leave it as is
//     to clear the state bit (set bit to clear).
// -Set switch life time.
// -Call the lower level function to send the switch info set SMP.
//
// Possible Error cases:
//  If any of the Top values exceeds the defined Cap values
//  If enabled RoutingMode is set and differs from new value
//  If any Packet operation (Get/Set) fails


static void
_switch_info_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Node_t *nodep = NULL;
	Port_t	*portp = NULL;
	Status_t status = VSTATUS_OK;
	SmpAddr_t addr;

	uint32_t	doSet=0;
	uint8_t		pauseState=0;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	GetSwInfoWorkItem_t *getSwInfoWorkItem =
		PARENT_STRUCT(pwi, GetSwInfoWorkItem_t, item);
	nodep = getSwInfoWorkItem->nodep;

	MaiPool_t *maiPoolp = psc_get_mai(psc);
	if (maiPoolp == NULL) {
		IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
			sm_nodeDescString(nodep));
		_switch_info_work_item_free(getSwInfoWorkItem);
		status = VSTATUS_NOMEM;
		psc_set_status(psc, status);
		psc_stop(psc);
		IB_EXIT(__func__, status);
		return;
	}

	psc_lock(psc);

	if (!sm_valid_port((portp = sm_get_port(nodep,0)))) {
		IB_LOG_WARN_FMT(__func__, "Failed getting port for %s\n",
			sm_nodeDescString(nodep));
		goto exit;
	}

	/* if getSwitchInfo failed during discovery sweep, try now */
	if (nodep->switchInfo.LinearFDBCap == 0 && nodep->switchInfo.MulticastFDBCap == 0) {
		SMP_ADDR_SET_DR(&addr, nodep->path);

		psc_unlock(psc);
		status = SM_Get_SwitchInfo(maiPoolp->fd, 0, &addr, &nodep->switchInfo);
		psc_lock(psc);

		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to get Switchinfo for node %s guid "FMT_U64": status = %d",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, status);
			status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
			if (status == VSTATUS_TIMEOUT_LIMIT) {
				IB_LOG_WARN_FMT(__func__, "Timeout limit getting switch info for "
					"node %s guid "FMT_U64" node index "
					"%d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
					nodep->index, portp->index);
			}
			goto exit;
		}
	}
	/* check MFTCap is sufficient */
	if (nodep->switchInfo.MulticastFDBCap < sm_mcast_mlid_table_cap) {
		   IB_LOG_WARN_FMT(__func__,
				   "MLIDTableCap %u exceeds MFT Cap %u of Switch %s guid "FMT_U64,
				   sm_mcast_mlid_table_cap, nodep->switchInfo.MulticastFDBCap,
				   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
	}


	if (nodep->switchInfo.RoutingMode.Enabled != STL_ROUTE_LINEAR) {
		nodep->switchInfo.RoutingMode.Enabled = STL_ROUTE_LINEAR;
		doSet = 1;
	}

	if (nodep->switchInfo.LinearFDBTop != sm_topop->maxLid) {
		nodep->switchInfo.LinearFDBTop = sm_topop->maxLid;
		if (nodep->switchInfo.RoutingMode.Enabled == STL_ROUTE_LINEAR &&
			nodep->switchInfo.LinearFDBCap <= sm_topop->maxLid) {
			IB_LOG_ERROR_FMT(__func__,
				"Switchinfo LinearFDBCap too low for node %s guid "FMT_U64": cap = %d, maxLid = %d",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
				nodep->switchInfo.LinearFDBCap, sm_topop->maxLid);
			IB_FATAL_ERROR_NODUMP("Exiting: LinearFDBCap less than max lid");
		}
		doSet = 1;
	}

	if (nodep->switchInfo.CapabilityMask.s.IsAddrRangeConfigSupported) {
		if ((nodep->switchInfo.MultiCollectMask.MulticastMask != SM_DEFAULT_MULTICAST_MASK)
			|| (nodep->switchInfo.MultiCollectMask.CollectiveMask != SM_DEFAULT_COLLECTIVE_MASK)) {
			nodep->switchInfo.MultiCollectMask.MulticastMask = SM_DEFAULT_MULTICAST_MASK;
			nodep->switchInfo.MultiCollectMask.CollectiveMask = SM_DEFAULT_COLLECTIVE_MASK;
			doSet = 1;
		}
	}

	{
		STL_LID maxMCLid;
		if (nodep->switchInfo.MulticastFDBTop != (maxMCLid = sm_multicast_get_max_lid())) {
			nodep->switchInfo.MulticastFDBTop = maxMCLid;
			doSet = 1;
		}
	}

	// Check for pause indication, needs to be set if routing changes are being made or additional
	// alternate routes indicated.
	pauseState = (nodep->arSupport && (topology_changed || topology_passcount == 0 ||
		forceRebalanceNextSweep ||
		!bitset_equal(&old_switchesInUse, &new_switchesInUse) ||
		new_endnodesInUse.nset_m ||
		old_topology.num_endports != sm_newTopology.num_endports ||
		old_topology.num_sws != sm_newTopology.num_sws)) ? 1 : 0;

	if (sm_VerifyAdaptiveRoutingConfig(nodep)) {
		doSet = 1;
		if (sm_adaptiveRouting.debug) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Switch node %s guid "FMT_U64" setting config vendor info",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
		}
	}

	if (pauseState != nodep->switchInfo.AdaptiveRouting.s.Pause) {
		// Pause indicated or switch is in bad pause state.
		doSet = 1;

	} else if (sm_adaptiveRouting.debug) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Switch node %s guid "FMT_U64" already has correct pause state %d",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, pauseState);
	}

	/*
	 * portChange indicates switch had a port state change event on one of it's ports
	 * We have to echo this back to clear the bit
	 */
	if (nodep->switchInfo.u1.s.PortStateChange) {
		doSet = 1;
		IB_LOG_INFO_FMT(__func__,
			   "Switch node %s guid "FMT_U64": port mkey="FMT_U64", SM mkey="FMT_U64" had a port state change",
			   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->portData->portInfo.M_Key, sm_config.mkey);
	}
	// Check the switch lifetime value.
	if (nodep->switchInfo.u1.s.LifeTimeValue != sm_config.switch_lifetime_n2) {
		nodep->switchInfo.u1.s.LifeTimeValue = sm_config.switch_lifetime_n2;
		doSet = 1;
		IB_LOG_INFO_FMT(__func__,
			"Switch node %s guid "FMT_U64": updating switch lifetime value to %d",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, sm_config.switch_lifetime_n2);
	}
	if (doSet || sm_config.forceAttributeRewrite) {
		// make the necessary changes at the switch, update the SwitchInfo
		// data of the switch
		nodep->switchInfo.AdaptiveRouting.s.Pause = pauseState ? 1 : 0;
		SMP_ADDR_SET_DR(&addr, nodep->path);
		psc_unlock(psc);
		status = SM_Set_SwitchInfo(maiPoolp->fd, 0, &addr, &nodep->switchInfo, portp->portData->portInfo.M_Key);
		psc_lock(psc);

		if (status == VSTATUS_OK) {
			nodep->switchInfo.AdaptiveRouting.s.Pause = pauseState ? 1 : 0;
			if (sm_adaptiveRouting.debug) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					"Setting adaptive routing pause (%d) for switch %s guid "FMT_U64,
					pauseState, sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
			}
		} else {
			status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
			if (status == VSTATUS_TIMEOUT_LIMIT) {
				IB_LOG_WARN_FMT(__func__, "Timeout limit while setting switch info for "
					"node %s guid "FMT_U64" node index "
					"%d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
					nodep->index, portp->index);
			}
			goto exit;
		}
	}

exit:
	psc_set_status(psc, status);
	// Only abort the sweep if we are in an unrecoverable situation.
	if (status == VSTATUS_UNRECOVERABLE || status == VSTATUS_TIMEOUT_LIMIT) {
		psc_stop(psc);
	}
	psc_unlock(psc);
	psc_free_mai(psc, maiPoolp);
	_switch_info_work_item_free(getSwInfoWorkItem);
	IB_EXIT(__func__, status);
}

Status_t
sweep_assignments_switchinfo(SweepContext_t *sweep_context)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	GetSwInfoWorkItem_t *wip;

	IB_ENTER(__func__, 0, 0, 0, 0);

	if (sm_state != SM_STATE_MASTER)
		return VSTATUS_NOT_MASTER;

	psc_go(sweep_context->psc);

	for_all_switch_nodes(sm_topop, nodep) {
		wip = _switch_info_workitem_alloc(nodep, _switch_info_worker);
		if (wip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(sweep_context->psc, &wip->item);
	}

	if (status == VSTATUS_OK) {
		status = psc_wait(sweep_context->psc);
	} else {
		psc_stop(sweep_context->psc);
		(void)psc_wait(sweep_context->psc);
	}

	psc_drain_work_queue(sweep_context->psc);

	IB_EXIT(__func__, status);

	return status;
}
