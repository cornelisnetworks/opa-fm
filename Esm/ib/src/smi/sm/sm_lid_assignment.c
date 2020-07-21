/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

#include "os_g.h"
#include "ib_types.h"
#include "ib_macros.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "cs_csm_log.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "cs_queue.h"
#include "cs_bitset.h"
#include "topology.h"
#include "sm_parallelsweep.h"
#include "sm_lid_assignment.h"

extern Topology_t *sm_topop;

extern Status_t topology_sm_port_init_failure(void);
extern void reset_port_retry_count(void);

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} LidAssignmentWorkItem_t;

static LidAssignmentWorkItem_t *
_lid_assignment_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	LidAssignmentWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(LidAssignmentWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workItem, 0, sizeof(LidAssignmentWorkItem_t));

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_lid_assignment_work_item_free(LidAssignmentWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static void
_lid_assignment_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Node_t *nodep;
	Status_t status = VSTATUS_OK;
	Port_t *portp;
	uint8_t vlInUse = 0xff;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	LidAssignmentWorkItem_t *LidAssignmentWorkItem =
		PARENT_STRUCT(pwi, LidAssignmentWorkItem_t, item);
	portp = NULL;
	nodep = LidAssignmentWorkItem->nodep;

	MaiPool_t *maiPoolp = NULL;

	if (psc) {
		maiPoolp = psc_get_mai(psc);
		if (maiPoolp == NULL) {
			IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
				sm_nodeDescString(nodep));
			_lid_assignment_work_item_free(LidAssignmentWorkItem);
			psc_set_status(psc, VSTATUS_NOMEM);
			psc_stop(psc);
			IB_EXIT(__func__, VSTATUS_NOMEM);
			return;
		}
	} else {
		IB_LOG_ERROR_FMT(__func__, "Invalid parallel sweep context provided for %s\n",
			sm_nodeDescString(nodep));
		_lid_assignment_work_item_free(LidAssignmentWorkItem);
		IB_EXIT(__func__, VSTATUS_UNRECOVERABLE);
		return;
	}

	psc_lock(psc);

	for_all_ports(nodep, portp) {
		if (sm_valid_port(portp) && portp->state > IB_PORT_DOWN) {
			if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
				(portp->index != 0 || nodep->switchInfo.u2.s.EnhancedPort0)) {
				if (vlInUse == 0xff) {
					vlInUse = portp->portData->vl1;
				} else if (vlInUse != portp->portData->vl1) {
					nodep->uniformVL = 0;
				}
				nodep->vlCap = MAX(nodep->vlCap, portp->portData->vl0);
				nodep->arbCap = MAX(nodep->arbCap, portp->portData->portInfo.VL.ArbitrationHighCap);
			}

			if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0) ||
				(nodep == sm_topop->node_head && portp->index == sm_config.port))
				continue;       /* Port 0 of switches and SM port are already initialized above*/

			/* Initialize ports using mixed LR-DR SMPs*/
			status = sm_initialize_port_LR_DR(psc, maiPoolp->fd, sm_topop, nodep, portp);
			if (status != VSTATUS_OK) {
				/* see if we are unable to initialize the local port */
				if (nodep == sm_topop->node_head && portp->index == sm_config.port) {
					status = topology_sm_port_init_failure();
					status = VSTATUS_UNRECOVERABLE;
					psc_stop(psc);
					goto exit;
				}

				/* when not local port */
				reset_port_retry_count(); /*reset the counter used for retry backoff*/
				if (topology_main_exit == 1) {
#ifdef __VXWORKS__
					ESM_LOG_ESMINFO("topology_assignments: SM has been stopped", 0);
#endif
					status = VSTATUS_OK;
					psc_stop(psc);
					goto exit;
				}

				sm_mark_link_down(sm_topop, portp);
				topology_changed = 1;   /* indicates a fabric change has been detected */
				IB_LOG_ERROR_FMT(__func__,
					"Marking port[%d] of node[%d] %s guid "FMT_U64" DOWN in the topology",
					portp->index, nodep->index, sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);

				status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					psc_stop(psc);
					goto exit;
				}
			}
		}
	}

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && nodep->uniformVL) {
		nodep->activeVLs = vlInUse;
	}

exit:
	psc_unlock(psc);
	psc_free_mai(psc, maiPoolp);
	psc_set_status(psc, status);
	_lid_assignment_work_item_free(LidAssignmentWorkItem);
	IB_EXIT(__func__, status);
}

Status_t parallel_endnode_lid_assignments(SweepContext_t *sweep_context)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	LidAssignmentWorkItem_t *wip;

	IB_ENTER(__func__, 0, 0, 0, 0);

	psc_go(sweep_context->psc);

	for_all_nodes(sm_topop, nodep) {
		wip = _lid_assignment_workitem_alloc(nodep, _lid_assignment_worker);
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
