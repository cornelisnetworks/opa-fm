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
#include "sm_congestion.h"
#include "stl_cca.h"

extern Topology_t *sm_topop;
extern int topology_sweep_triggered(void);

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} CongestionWorkItem_t;

static void
_congestion_work_item_free(CongestionWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static CongestionWorkItem_t *
_congestion_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	CongestionWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(CongestionWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workItem, 0, sizeof(CongestionWorkItem_t));

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_congestion_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Node_t *nodep = NULL;
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	CongestionWorkItem_t *CongestionWorkItem =
		PARENT_STRUCT(pwi, CongestionWorkItem_t, item);
	nodep = CongestionWorkItem->nodep;

	MaiPool_t *maiPoolp = NULL;

	if (psc) {
		maiPoolp = psc_get_mai(psc);
		if (maiPoolp == NULL) {
			IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
				sm_nodeDescString(nodep));
			_congestion_work_item_free(CongestionWorkItem);
			psc_set_status(psc, VSTATUS_BAD);
			psc_stop(psc);
			IB_EXIT(__func__, VSTATUS_BAD);
			return;
		}
	} else {
		IB_LOG_ERROR_FMT(__func__, "Invalid parallel sweep context provided for %s\n",
			sm_nodeDescString(nodep));
			_congestion_work_item_free(CongestionWorkItem);
			IB_EXIT(__func__, VSTATUS_BAD);
			return;
	}

	psc_lock(psc);

	if (topology_sweep_triggered()) {
		IB_LOG_INFINI_INFO0("New sweep request, delaying Congestion Configuration until after next sweep");
		status = VSTATUS_OK;
		psc_set_status(psc, status);
		psc_stop(psc);
		goto exit;
	}

	if (nodep->congConfigDone) {
		goto exit;
	}
	if (sm_config.congestion.enable) {
		Port_t *portp = sm_get_node_end_port(nodep);
		if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN
			) {
			goto exit;
		}

		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, portp->portData->lid);

		psc_unlock(psc);
		status = SM_Get_CongestionInfo(maiPoolp->fd, 0, &addr, &nodep->congestionInfo);
		psc_lock(psc);

		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to get Congestion Info for NodeGUID "FMT_U64" [%s]; rc: %d",
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
			goto exit;
		}
	}

	/* Need to call these functions even if disabled so we can clear CCA attributes */
	if (nodep->nodeInfo.NodeType == STL_NODE_SW)
		status = stl_sm_cca_configure_sw(psc, maiPoolp->fd, nodep);
	else
		status = stl_sm_cca_configure_hfi(psc, maiPoolp->fd, nodep);

	if (status != VSTATUS_OK) {
		IB_LOG_INFO_FMT(__func__, "Failed to configure CCA for NodeGuid "FMT_U64" [%s]; rc: %d",
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
	}
	else {
		nodep->congConfigDone = 1;
	}

exit:
	psc_unlock(psc);
	psc_free_mai(psc, maiPoolp);
	_congestion_work_item_free(CongestionWorkItem);
	IB_EXIT(__func__, status);
}

Status_t parallel_congestion(ParallelSweepContext_t *psc)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	CongestionWorkItem_t *wip;
	IB_ENTER(__func__, 0, 0, 0, 0);

	psc_go(psc);

	for_all_nodes(sm_topop, nodep) {
		wip = _congestion_workitem_alloc(nodep, _congestion_worker);
		if (wip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(psc, &wip->item);
	}

	if (status == VSTATUS_OK) {
		status = psc_wait(psc);
	} else {
		psc_stop(psc);
		(void)psc_wait(psc);
	}

	psc_drain_work_queue(psc);

	IB_EXIT(__func__, status);

	return status;
}
