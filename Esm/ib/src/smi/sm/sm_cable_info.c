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
#include "sm_cable_info.h"

extern Topology_t *sm_topop;
int topology_resweep_requested(void);

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} CableInfoWorkItem_t;

static CableInfoWorkItem_t *
_cable_info_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	CableInfoWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(CableInfoWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workItem, 0, sizeof(CableInfoWorkItem_t));

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_cable_info_work_item_free(CableInfoWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static void
_cable_info_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Node_t *nodep;
	Status_t status = VSTATUS_OK;
	Port_t *portp;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	CableInfoWorkItem_t *CableInfoWorkItem =
		PARENT_STRUCT(pwi, CableInfoWorkItem_t, item);
	portp = NULL;
	nodep = CableInfoWorkItem->nodep;

	MaiPool_t *maiPoolp = NULL;

	if (psc) {
		maiPoolp = psc_get_mai(psc);
		if (maiPoolp == NULL) {
			IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
				sm_nodeDescString(nodep));
			_cable_info_work_item_free(CableInfoWorkItem);
			psc_set_status(psc, VSTATUS_BAD);
			psc_stop(psc);
			IB_EXIT(__func__, VSTATUS_BAD);
			return;
		}
	} else {
		IB_LOG_ERROR_FMT(__func__, "Invalid parallel sweep context provided for %s\n",
			sm_nodeDescString(nodep));
		_cable_info_work_item_free(CableInfoWorkItem);
                IB_EXIT(__func__, VSTATUS_BAD);
                return;
        }

	psc_lock(psc);

	for_all_ports(nodep, portp) {
		// CableInfo updates take a long time, so allow us to short circuit out of them for now if
		// we have been told to resweep.
		if (topology_resweep_requested()) {
			status = VSTATUS_OK;
			psc_set_status(psc, status);
			psc_stop(psc);
			goto exit;
		}
		if (sm_valid_port(portp) && !portp->portData->cableInfo &&
			portp->state == IB_PORT_ACTIVE && sm_Port_t_IsCableInfoSupported(portp)) {

			Port_t * neighPort = NULL;
			if (sm_config.cableInfoPolicy == CIP_LINK) {
				Node_t * neighNode = NULL;
				neighPort = sm_find_neighbor_node_and_port(&sm_newTopology, portp, &neighNode);
			}

			if (neighPort && neighPort->portData->cableInfo) {
				portp->portData->cableInfo = sm_CableInfo_copy(neighPort->portData->cableInfo);
			}
			else {
				status = sm_update_cableinfo(psc, maiPoolp->fd, sm_topop, nodep, portp);

				if (status == VSTATUS_NOSUPPORT) {
					IB_LOG_INFO_FMT(__func__,
						"CableInfo not supported for node %s nodeGuid "FMT_U64" port %d",
						sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
						sm_Port_t_SetCableInfoSupported(portp, FALSE);
				}
				else if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__,
						"Failed to Get(CableInfo) for node %s nodeGuid "FMT_U64" port %d",
						sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
					if (status == VSTATUS_TIMEOUT_LIMIT) {
						psc_set_status(psc, status);
						psc_stop(psc);
						goto exit;
					}
				}
			}
		}
	}
exit:
	psc_unlock(psc);
	psc_free_mai(psc, maiPoolp);
	_cable_info_work_item_free(CableInfoWorkItem);
	IB_EXIT(__func__, status);
}

Status_t parallel_cable_info(SweepContext_t *sweep_context)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	CableInfoWorkItem_t *wip;
	IB_ENTER(__func__, 0, 0, 0, 0);

	psc_go(sweep_context->psc);

	for_all_nodes(sm_topop, nodep) {
		wip = _cable_info_workitem_alloc(nodep, _cable_info_worker);
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
