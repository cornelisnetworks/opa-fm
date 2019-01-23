/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

//======================================================================//
//									    								//
// FILE NAME							   								//
//    sm_ar.c															//
//																		//
// DESCRIPTION								    						//
//    This file contains SM adaptive routing routines for setting 		//	
// 	  programming OPA switches with Adaptive routing tables.			//
//									    								//
// FUNCTIONS								 						    //
//    LogPortGroupTable 				Log port groups					//
//    LogPortGroupFwdTable				Log port group FDB				//
//    sm_AdaptiveRoutingSwitchUpdate	Program AR tables 				//
//																		//
//======================================================================//
#include "ib_types.h"
#include "sm_l.h"
#include "sm_parallelsweep.h"
#include "sm_ar.h"

extern int topology_main_exited(void);

/* Adaptive Routing parallel sweep work item */
typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} ArWorkItem_t;

void LogPortGroupTable(Node_t *switchp) 
{
    int i;

	IB_LOG_INFINI_INFO_FMT(__func__, "Switch %s guid "FMT_U64,
	   	sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);

    for (i=0; i<switchp->pgtLen; ++i) {
		IB_LOG_INFINI_INFO_FMT(__func__, "PortGroup[%03u] = %"PRIx64,
       		i, switchp->pgt[i]);
    }
}

void LogPortGroupFwdTable(Node_t *switchp) 
{
    int i;

	IB_LOG_INFINI_INFO_FMT(__func__, "Switch %s guid "FMT_U64,
	   	sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);

	if (switchp->pgft) {
		for (i=0; i < sm_Node_get_pgft_size(switchp); ++i) {
		IB_LOG_INFINI_INFO_FMT(__func__, "[%05u][%02u]", 
			i, switchp->pgft[i]);
		}
	}
	else {
		IB_LOG_INFINI_INFO_FMT(__func__, "No PortGroupFwdTable data");
	}
}

//
//Note: Assumes that the port group table is no more than 64 bits wide...
//
Status_t
sm_AdaptiveRoutingSwitchUpdate(ParallelSweepContext_t *psc, SmMaiHandle_t *fd, Topology_t* topop, Node_t* switchp) 
{
	Status_t 	status = VSTATUS_OK;
	STL_LID	dlid;
	
	if (!sm_adaptiveRouting.enable || !switchp->arSupport) return VSTATUS_OK;

	if (!switchp->initPorts.nset_m &&
		!switchp->arChange &&
		bitset_equal(&old_switchesInUse, &new_switchesInUse) &&
		bitset_test(&old_switchesInUse, switchp->swIdx)) {
		if (switchp->oldExists) {
			Node_t *oldnodep = switchp->old;
			if (oldnodep &&
				oldnodep->arSupport &&
				!oldnodep->arChange &&
				(bitset_equal(&switchp->activePorts, &oldnodep->activePorts))) {
				// Already programmed.
				return status;
			}
		}
		// Set AR Change indicator.  If this sweep fails too,
		// carry forward to next sweep.
		switchp->arChange = 1;
	}

	Port_t *portp = sm_get_port(switchp, 0);
	dlid = portp->portData->lid;

	// Update the port group table. By definition, this will never take more 
	// than two MADs, so we send the first half and, if needed, send the rest.
	if (switchp->pgt) {
		uint32_t	amod = 0l;
		uint8_t		blocks = ROUNDUP(switchp->switchInfo.PortGroupTop+1,
								NUM_PGT_ELEMENTS_BLOCK)/NUM_PGT_ELEMENTS_BLOCK;

		// Note that the Port Group table cannot be more than 256 entries long,
		// which means it will never be more than MAX_PGT_BLOCK_NUM in 
		// length. This also means we can always fit the entire PGT in a single
		// MAD (assuming switches with 64 ports or less). 
		if (blocks > MAX_PGT_BLOCK_NUM) {
			IB_LOG_WARN_FMT(__func__, 
				"PGT is too long for node %s nodeGuid "FMT_U64
				" length = %d. Truncating to %d blocks.",
				sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, 
				blocks*8, MAX_PGT_BLOCK_NUM);
			blocks = MAX_PGT_BLOCK_NUM;
		}

		// AMOD = NNNN NNNN PP 00 0000 0000 00AB BBBB
		// AMOD = # blocks  00 ------------ --00 0000
		amod = blocks<<24;
		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
		
		psc_unlock(psc);
		status = SM_Set_PortGroup(fd, amod, &addr,
			(STL_PORT_GROUP_TABLE*)switchp->pgt, blocks, sm_config.mkey);
		psc_lock(psc);

		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__, 
				"SET(PGT)failed for node %s nodeGuid "FMT_U64
				" status = %d",
				sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, status);
		} else if (blocks > MAX_PGT_BLOCK_NUM) {
			// AMOD = NNNN NNNN PP 00 0000 0000 00AB BBBB
			// AMOD = last blks 00 ------------ --00 001f
			amod = ((blocks - MAX_PGT_BLOCK_NUM)<<24) | MAX_PGT_BLOCK_NUM;
		
			psc_unlock(psc);
			status = SM_Set_PortGroup(fd, amod, &addr,
				(STL_PORT_GROUP_TABLE*)&switchp->pgt[MAX_PGT_BLOCK_NUM*NUM_PGT_ELEMENTS_BLOCK], 
				blocks, sm_config.mkey);
			psc_lock(psc);

			if (status != VSTATUS_OK) {
				IB_LOG_WARN_FMT(__func__, 
					"Failed to send end of Port Group Table to node %s nodeGuid "
					FMT_U64" status = %d",
					sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, 
					status);
			}
		}
	} else if (switchp->pgtLen > 0) {
		// pointer is null but length is > 0. That's a problem.
		IB_LOG_WARN_FMT(__func__, 
			"Port Group Table is NULL for node %s nodeGuid "FMT_U64,
			sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
			status = VSTATUS_EIO;
	}

	// Update the port group forwarding table. The PGFT is similar to the LFT,
	// with a different cap, so we use the LFT method to break the PGFT down into
	// multiple MADs.
	if (status == VSTATUS_OK) {
		STL_LID		currentLid;
		uint16_t	currentSet, numBlocks;
		uint32_t	amod;
		SmpAddr_t	addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
		const uint16_t  lids_per_mad = sm_config.lft_multi_block * MAX_LFT_ELEMENTS_BLOCK;
		PORT * pgft = sm_Node_get_pgft_wr(switchp);
		const uint32_t pgftSize = sm_Node_get_pgft_size(switchp);
		const uint32_t pgftCap = (topop->maxLid > pgftSize)?pgftSize:topop->maxLid;

		if (pgft == NULL) {
			IB_LOG_ERROR_FMT(__func__, "Could not allocate pgft for node %s nodeGuid "FMT_U64,
			sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
			status = VSTATUS_NOMEM;
			return status;
		}

		for (currentSet = 0, currentLid = 0; 
			(currentLid < pgftCap) &&
			(status == VSTATUS_OK);
			currentSet += sm_config.lft_multi_block, 
			currentLid += lids_per_mad) {

			// The # of blocks we can send in this MAD. Normally 
			// sm_config.lft_multi_block but will be less for the last send.
			numBlocks = ( currentLid + lids_per_mad <= pgftCap) ?  sm_config.lft_multi_block :
				1 + (pgftCap - (currentLid+1))/NUM_PGFT_ELEMENTS_BLOCK;
			// AMOD = NNNN NNNN 0000 0ABB BBBB BBBB BBBB BBBB
			// AMOD = numBlocks 0000 00[[[[[[current set]]]]]
			amod = (numBlocks<<24) | currentSet;

			psc_unlock(psc);
			status = SM_Set_PortGroupFwdTable(fd, amod, &addr,
				(STL_PORT_GROUP_FORWARDING_TABLE*)&pgft[currentLid],
				numBlocks, sm_config.mkey);
			psc_lock(psc);

			if (status != VSTATUS_OK) {
				IB_LOG_WARN_FMT(__func__, 
					"SET(PGFT) failed for node %s nodeGuid "FMT_U64
					" status = %d",
					sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, 
					status);
			}
		}
	} else if (switchp->pgft == NULL && switchp->pgtLen != 0) {
		IB_LOG_WARN_FMT(__func__, 
			"PGFT is NULL for node %s nodeGuid "FMT_U64", pgtLen is %d",
			sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID,
			switchp->pgtLen);
		status = VSTATUS_EIO;
	}

	if (status == VSTATUS_OK) {
		// Set(SwitchInfo) to update PortGroupTop on the target switch
		// and clear the pause
		switchp->switchInfo.AdaptiveRouting.s.Pause = 0;
		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, portp->portData->lid);

		psc_unlock(psc);
		status = SM_Set_SwitchInfo(fd, 0, &addr, &switchp->switchInfo, sm_config.mkey);
		psc_lock(psc);

		if (status == VSTATUS_OK) {
			switchp->arChange = 0;
		} else {
			switchp->switchInfo.AdaptiveRouting.s.Pause = 1;
			IB_LOG_WARN_FMT(__func__,
				"Set(SwitchInfo) failed for node %s nodeGuid "FMT_U64,
				sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
		}
	}

	return status;
}

uint8_t
sm_VerifyAdaptiveRoutingConfig(Node_t *switchp) {
	if (switchp->switchInfo.CapabilityMask.s.IsAdaptiveRoutingSupported) {
		if (switchp->switchInfo.AdaptiveRouting.s.Enable != sm_adaptiveRouting.enable ||
			switchp->switchInfo.AdaptiveRouting.s.Algorithm != sm_adaptiveRouting.algorithm ||
			switchp->switchInfo.AdaptiveRouting.s.Frequency != sm_adaptiveRouting.arFrequency ||
			switchp->switchInfo.AdaptiveRouting.s.LostRoutesOnly != sm_adaptiveRouting.lostRouteOnly ||
			switchp->switchInfo.AdaptiveRouting.s.Threshold != sm_adaptiveRouting.threshold ||
			(switchp->switchInfo.PortGroupTop && !sm_adaptiveRouting.enable)) {

			switchp->switchInfo.AdaptiveRouting.s.Enable = sm_adaptiveRouting.enable ? 1 : 0;
			switchp->switchInfo.AdaptiveRouting.s.LostRoutesOnly = sm_adaptiveRouting.lostRouteOnly ? 1 : 0;
			switchp->switchInfo.AdaptiveRouting.s.Algorithm = sm_adaptiveRouting.algorithm;
			switchp->switchInfo.AdaptiveRouting.s.Frequency = sm_adaptiveRouting.arFrequency;
			switchp->switchInfo.AdaptiveRouting.s.Threshold = sm_adaptiveRouting.threshold;

			if (!switchp->switchInfo.AdaptiveRouting.s.Enable)
				switchp->switchInfo.PortGroupTop = 0;

			return 1;
		}
	}
	return 0;
}

/* Parallel Sweep */
static void
_ar_work_item_free(ArWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static ArWorkItem_t *
_ar_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	ArWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(ArWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workItem, 0, sizeof(ArWorkItem_t));

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_ar_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Node_t *nodep = NULL;
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	ArWorkItem_t *ArWorkItem =
		PARENT_STRUCT(pwi, ArWorkItem_t, item);
	nodep = ArWorkItem->nodep;

	MaiPool_t *maiPoolp = NULL;

	if (psc) {
		maiPoolp = psc_get_mai(psc);
		if (maiPoolp == NULL) {
			IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
				sm_nodeDescString(nodep));
			_ar_work_item_free(ArWorkItem);
			psc_set_status(psc, VSTATUS_BAD);
			psc_stop(psc);
			IB_EXIT(__func__, VSTATUS_BAD);
			return;
		}
	} else {
		IB_LOG_ERROR_FMT(__func__, "Invalid parallel sweep context provided for %s\n",
			sm_nodeDescString(nodep));
			_ar_work_item_free(ArWorkItem);
			IB_EXIT(__func__, VSTATUS_BAD);
			return;
	}

	psc_lock(psc);

//
//	For every switch that supports adaptive routing, we need to do adaptive routing switch table setup.
//

	status = sm_AdaptiveRoutingSwitchUpdate(psc, maiPoolp->fd, sm_topop, nodep);

	if (status != VSTATUS_OK) {
		if (topology_main_exited()) {
#ifdef __VXWORKS__
			ESM_LOG_ESMINFO("topology_adaptiverouting: SM has been stopped", 0);
#endif
			IB_EXIT(__func__, VSTATUS_OK);
			status = VSTATUS_OK;
			psc_set_status(psc, status);
			psc_stop(psc);
			goto exit;
		}

		status = sm_popo_port_error(&sm_popo, sm_topop, sm_get_port(nodep, 0), status);
		if (status == VSTATUS_TIMEOUT_LIMIT) {
			psc_set_status(psc, status);
			psc_stop(psc);
			goto exit;
		}
	}

exit:
	psc_unlock(psc);
	psc_free_mai(psc, maiPoolp);
	_ar_work_item_free(ArWorkItem);
	IB_EXIT(__func__, status);
}

Status_t parallel_ar(ParallelSweepContext_t *psc)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	ArWorkItem_t *wip;
	IB_ENTER(__func__, 0, 0, 0, 0);

	psc_go(psc);

	for_all_switch_nodes(sm_topop, nodep) {
		if (nodep->arSupport) {
			wip = _ar_workitem_alloc(nodep, _ar_worker);
			if (wip == NULL) {
				status = VSTATUS_NOMEM;
				break;
			}
			psc_add_work_item(psc, &wip->item);
		}
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
