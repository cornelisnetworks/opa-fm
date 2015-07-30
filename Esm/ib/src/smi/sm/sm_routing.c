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

#include "ib_types.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "fm_xml.h"

// Macro hacks for compiler warnings on non 64-bit systems;
// replace with uintptr_t if available on all platforms.
// See also fm_xml.h:XML_QMAP_CHAR_CAST
#ifdef __VXWORKS__

// Following macros only work for 32-bit VxWorks; fail if we know it's not 32-bit
#if defined(__WORDSIZE) && __WORDSIZE != 32
#error "Expected 32-bit word size."
#endif

#define UGLY_U64_CHAR(P) (char *)(uint32)(P)
#define UGLY_CHAR_U64(P) (uint64)(uint32)(P)

#else

#define UGLY_U64_CHAR(P) (char*)(P)
#define UGLY_CHAR_U64(P) (uint64)(P)

#endif

static cl_qmap_t moduleFacMap = {};


// Dummy module stuff; this is the module that's used if no other routing module is specified.  Basically causes the sweep to abort ASAP.
static Status_t
_dummy_warn_load(RoutingModule_t * rm)
{
	IB_LOG_WARN_FMT(__func__, "Loading dummy routing module; won't do anything.");
	return VSTATUS_OK;
}

static Status_t
_dummy_pre_process_discovery(Topology_t * topop, void ** routingContext)
{
	return VSTATUS_BAD;
}

static RoutingModule_t dummyModule = {
	.name = "dummyModule",

	.funcs.pre_process_discovery = _dummy_pre_process_discovery,
	.load = _dummy_warn_load,
};


static Status_t
_dummy_mod_fac(RoutingModule_t * module)
{
	memcpy(module, &dummyModule, sizeof(RoutingModule_t));
	return VSTATUS_OK;
}

/// Wrapper/adapter for cl_qmap_t with string-keys
static int
_qmap_str_cmp(uint64 a, uint64 b)
{
	return strcmp(UGLY_U64_CHAR(a), UGLY_U64_CHAR(b));
}

Status_t
sm_routing_addModuleFac(const char * name, routing_mod_factory fac)
{
	Status_t s;
	cl_map_obj_t * wrp = NULL;

	if (name == NULL || strlen(name) == 0 || fac == NULL)
		return VSTATUS_ILLPARM;

	if (cl_qmap_get(&moduleFacMap, UGLY_CHAR_U64(name)) != cl_qmap_end(&moduleFacMap))
		return VSTATUS_BAD;

	if ((s = vs_pool_alloc(&sm_pool, sizeof(cl_map_obj_t), (void*)&wrp)) != VSTATUS_OK)
		return s;

	memset(wrp, 0, sizeof(*wrp));

	wrp->p_object = fac;

	if (cl_qmap_insert(&moduleFacMap, UGLY_CHAR_U64(name), &wrp->item) == NULL)
		return VSTATUS_BAD;

	return VSTATUS_OK;
}

Status_t
sm_routing_makeModule(const char * name, RoutingModule_t ** module)
{
	cl_map_item_t * it;
	*module = NULL;

	if (name == NULL || strlen(name) == 0)
		return VSTATUS_ILLPARM;

	if ((it = cl_qmap_get(&moduleFacMap, UGLY_CHAR_U64(name))) == cl_qmap_end(&moduleFacMap))
		return VSTATUS_BAD;

	routing_mod_factory fac = cl_qmap_obj((const cl_map_obj_t * const) it);

	Status_t s;
	if ((s = vs_pool_alloc(&sm_pool, sizeof(RoutingModule_t), (void*)module)) != VSTATUS_OK) {
		return s;
	}

	return fac(*module);
}

Status_t
sm_routing_freeModule(RoutingModule_t ** module, Topology_t * topop)
{
	if (module == NULL || *module == NULL)
		return VSTATUS_ILLPARM;

	if (topop != NULL&& (*module)->funcs.destroy != NULL) {
		(*module)->funcs.destroy(topop);
	}

	if ((*module)->release != NULL)
		(*module)->release(*module);

	vs_pool_free(&sm_pool, *module);
	*module = NULL;

	return VSTATUS_OK;
}

RoutingModule_t *
sm_routing_getCurrentModule(Topology_t * topop)
{
	return topop->routingModule;
}

void
sm_routing_setCurrentModule(Topology_t * topop, RoutingModule_t * rm)
{
	if (topop->routingModule != NULL) {
		//TODO: should this call rm->copy() to copy data from the existing module to the new module?
		if (topop->routingModule->unload != NULL)
			topop->routingModule->unload(topop->routingModule);

		sm_routing_freeModule(&topop->routingModule, topop);
	}

	topop->routingModule = rm;
	if (rm != NULL && rm->load != NULL)
		rm->load(rm);
}

Status_t
sm_routing_alloc_floyds(Topology_t *topop)
{
	Status_t status;
	size_t   bytesCost;

	/* Allocate space for the cost array. */
	bytesCost = topop->max_sws * topop->max_sws * sizeof(uint16_t);
	if (bytesCost > topop->bytes) {
		topop->bytes = 0;

		if (topop->cost != NULL) {
			(void)vs_pool_free(&sm_pool, (void *)topop->cost);
			topop->cost = NULL;
		}

		status = vs_pool_alloc(&sm_pool, bytesCost, (void *)&topop->cost);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("can't malloc cost array rc:", status);
			IB_EXIT(__func__, status);
			return status;
		}

		topop->bytes = bytesCost;
	}

	return VSTATUS_OK;
}

void
sm_routing_init_floyds(Topology_t *topop)
{
	int i, j, k, ij, ik, ki, iNumNodes;
	Node_t *nodep, *neighborNodep;
	Port_t *portp;

	/* Set initial values. */
	for (i = 0, iNumNodes = 0; i < topop->max_sws; i++, iNumNodes += topop->max_sws) {
		for (j = 0, ij = iNumNodes; j <= i; j++, ij++) {
			topop->cost[ij] = topop->cost[Index(j,i)] = Cost_Infinity;
		}
	}
    
	/* Set the costs for known edges. */
	for_all_switch_nodes(topop, nodep) {
		i = nodep->swIdx;
		for_all_physical_ports(nodep, portp) {
			if (sm_valid_port(portp) && portp->state > IB_PORT_DOWN) {
				k = portp->nodeno;
				neighborNodep = sm_find_node(topop, k);
				if (neighborNodep == NULL) {
					IB_LOG_WARN("Node not found, can't adjust cost from node index", k);
					continue;
				} else if (neighborNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
					/* we do not use end nodes in the algorithm, just switches */
					continue;
				} else {
					/* k is the switch's index in the switch list, not index in node list */
					k = neighborNodep->swIdx;
				}

				ik = Index(i, k);
				ki = Index(k, i);
				topop->cost[ik] = topop->cost[ki] = sm_GetCost(portp->portData);
			}
		}
	}

	/* Clear the costs for each node to itself. */
	for (i = 0, ij = 0; i < topop->max_sws; i++, ij += topop->max_sws + 1) {
		topop->cost[ij] = 0;
	}
}

void
sm_routing_calc_floyds(int switches, unsigned short * cost)
{
	int i, j, k;
	int ij, ik, kj, oldik;
	int kNumNodes, iNumNodes, oldiNumNodes;
	unsigned short value;
	unsigned int total_cost = 0;
	unsigned int leastTotalCost = 0;
	unsigned int max_cost = 0;
	unsigned int leastWorstCaseCost = 0;
	uint64_t leastWorstCostSwitchGuid = 0;
	uint64_t leastTotalCostSwitchGuid = 0;
	unsigned int currRootTotalCost = 0;
	unsigned int currRootWorstCaseCost = 0;
	int curr_selection_sm_neighbor = 0;
	uint8_t	old_root_exists = 0;
	Node_t *nodep;
	Node_t *sw = sm_topop->switch_head;

	for (k = 0, kNumNodes = 0; k < switches; k++, kNumNodes += switches) {
		for (i = 0, iNumNodes = 0; i < switches; i++, iNumNodes += switches) {

			ik = iNumNodes + k;

			if (cost[ik] == Cost_Infinity) {
				continue;
			}

			for (j = i + 1, ij = iNumNodes + i + 1, kj = kNumNodes + i + 1;
			     j < switches; ++j, ++ij, ++kj) {

				if ((value = cost[ik] + cost[kj]) < cost[ij]) {
					cost[ij] = value;
					// Take advantage of the fact that cost array is symmetric
					cost[Index(j, i)] = value;
				}
			}
		}

		if (smDebugPerf && (k & 0xFF) == 0xFF) {
			IB_LOG_INFINI_INFO_FMT(__func__, "completed run %d of %d", k, switches);
		}
	}

	/* All floyd costs are fully computed now and can be analyzed */
	for (k = 0; k < switches; k++) {
		total_cost = 0;
		max_cost = 0;
		for (i = 0, iNumNodes = 0, oldiNumNodes = 0; i < switches;
			 i++, iNumNodes += switches, oldiNumNodes += old_topology.max_sws) {

			ik = iNumNodes + k;

			if (i == k) {
				continue;
			}

			if (sm_useIdealMcSpanningTreeRoot && (cost[Index(k, i)] < Cost_Infinity)) {
				/* Calculate costs of switches to determine the best MC spanning tree root. */
				if (sm_mcSpanningTreeRoot_useLeastTotalCost) {
					total_cost += cost[Index(k, i)];
				} else if (sm_mcSpanningTreeRoot_useLeastWorstCaseCost) {
					if (cost[Index(k, i)] > max_cost)
						max_cost = cost[Index(k, i)];	
				}
			}
			
			if (topology_passcount == 0)
				continue;

			/* PR 115770. If there is any switch cost/path change (including removal of switches),
			 * set topology_cost_path_changes which will force full LFT reprogramming for all switches.
			 */

			if (topology_cost_path_changes)
				continue;

			if (switches < old_topology.max_sws) {
				topology_cost_path_changes = 1;
				continue;
			}

			if ((k >= old_topology.max_sws) || (i >= old_topology.max_sws))
				continue;

			oldik = oldiNumNodes + k;

			if (old_topology.cost[oldik] != cost[ik]) {
				topology_cost_path_changes = 1;
			}			
		}

		if (sm_useIdealMcSpanningTreeRoot) {
			if (sw && sw->nodeInfo.NodeGUID == sm_mcSpanningTreeRootGuid) {
				/* Note the current cost of the root switch. Its cost could have changed
				 * due to topology changes.
				 */
				if (sm_mcSpanningTreeRoot_useLeastTotalCost)
					currRootTotalCost = total_cost;

				if (sm_mcSpanningTreeRoot_useLeastWorstCaseCost)
					currRootWorstCaseCost = max_cost;
			}

			if (sm_mcSpanningTreeRoot_useLeastTotalCost) {
				if ((leastTotalCost == 0 || total_cost < leastTotalCost)) {
					leastTotalCost = total_cost;
					if (sw) {
						leastTotalCostSwitchGuid = sw->nodeInfo.NodeGUID;
						if (sw->swIdx == 0)
							curr_selection_sm_neighbor = 1;
						else
							curr_selection_sm_neighbor = 0;
					}
				} else if (curr_selection_sm_neighbor && (total_cost == leastTotalCost)) {
					/* prefer non SM neighbor when costs are same */
					if (sw && (sw->swIdx != 0)) {
						leastTotalCostSwitchGuid = sw->nodeInfo.NodeGUID;
						curr_selection_sm_neighbor = 0;
					}
				}
			}

			if (sm_mcSpanningTreeRoot_useLeastWorstCaseCost) {
				if (leastWorstCaseCost == 0 || max_cost < leastWorstCaseCost) {
					leastWorstCaseCost = max_cost;
					if (sw) {
						leastWorstCostSwitchGuid = sw->nodeInfo.NodeGUID;
						if (sw->swIdx == 0)
							curr_selection_sm_neighbor = 1;
						else
							curr_selection_sm_neighbor = 0;
					}
				} else if (curr_selection_sm_neighbor && (max_cost == leastWorstCaseCost)) {
					/* prefer non SM neighbor when costs are same */
					if (sw && (sw->swIdx != 0)) {
						leastWorstCostSwitchGuid = sw->nodeInfo.NodeGUID;
						curr_selection_sm_neighbor = 0;
					}

				}
			}

			if (sw)
				sw = sw->type_next;
		}
	}

	if (!sm_useIdealMcSpanningTreeRoot)
		return;

	/* Based on the calculations, select the MC Spanning Tree Root Switch */

	old_root_exists = 0;
	if (sm_mcSpanningTreeRootGuid) {
		/* If we identified a root in a previous sweep or if we have just take over
		 * from a master, does the old root still exist ?*/
		if (sm_find_guid(sm_topop, sm_mcSpanningTreeRootGuid)) 
			old_root_exists = 1;
	}
	
	if (sm_mcSpanningTreeRoot_useLeastTotalCost) {
		if (!old_root_exists) {
			if (smDebugPerf && sm_mcSpanningTreeRootGuid) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					 "MC Spanning tree root switch disappeared, changing to new one.");
			}
			if (vs_lock(&sm_mcSpanningTreeRootGuidLock) != VSTATUS_OK) {
				IB_LOG_ERROR0("error in getting mcSpanningTreeRootGuidLock");
				return;
			}
			sm_mcSpanningTreeRootGuid = leastTotalCostSwitchGuid; 
	        (void)vs_unlock(&sm_mcSpanningTreeRootGuidLock);
			currRootTotalCost = leastTotalCost;
			if (smDebugPerf) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					 "MC Spanning Tree Root switch selected. Guid "FMT_U64, sm_mcSpanningTreeRootGuid);
			}
		} else {
			/* change root only if the delta between current root cost and
			 * the newly identified switch's cost is greater than the threshold */
			 if (currRootTotalCost && (leastTotalCost < currRootTotalCost)) {
				unsigned int delta = currRootTotalCost - leastTotalCost;
				if (((delta*100)/currRootTotalCost) >= sm_mcRootCostDeltaThreshold) {
					if (smDebugPerf) {
						IB_LOG_INFINI_INFO_FMT(__func__,
						 "Changing MC Spanning Tree Root Switch. "
						 "Least total cost Old Root %d New Root %d. Delta is above threshold value of %d%%.",
						 currRootTotalCost, leastTotalCost, sm_mcRootCostDeltaThreshold);
					}
					if (vs_lock(&sm_mcSpanningTreeRootGuidLock) != VSTATUS_OK) {
						IB_LOG_ERROR0("error in getting mcSpanningTreeRootGuidLock");
						return;
					}
					sm_mcSpanningTreeRootGuid = leastTotalCostSwitchGuid; 
			        (void)vs_unlock(&sm_mcSpanningTreeRootGuidLock);
					currRootTotalCost = leastTotalCost;
				} else if (smDebugPerf) {
					IB_LOG_INFINI_INFO_FMT(__func__,
					 "Not changing MC root switch. "
					 "Delta of Current root cost %d new least total cost %d below threshold value of %d%%",
					 currRootTotalCost, leastTotalCost, sm_mcRootCostDeltaThreshold);
				}
			} 
		}
	}

	if (sm_mcSpanningTreeRoot_useLeastWorstCaseCost) {
		if (!old_root_exists) {
			if (smDebugPerf && sm_mcSpanningTreeRootGuid) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					 "MC Spanning tree root switch disappeared, changing to new one.");
			}
			if (vs_lock(&sm_mcSpanningTreeRootGuidLock) != VSTATUS_OK) {
				IB_LOG_ERROR0("error in getting mcSpanningTreeRootGuidLock");
				return;
			}
			sm_mcSpanningTreeRootGuid = leastWorstCostSwitchGuid;
	        (void)vs_unlock(&sm_mcSpanningTreeRootGuidLock);
			currRootWorstCaseCost = leastWorstCaseCost;
			if (smDebugPerf) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					 "MC Spanning Tree Root switch selected. Guid "FMT_U64, sm_mcSpanningTreeRootGuid);
			}
		} else {
			/* change root only if the delta between current root cost and
			 * the newly identified switch's cost is greater than the threshold */
			 if (currRootWorstCaseCost && (leastWorstCaseCost < currRootWorstCaseCost)) {
				unsigned int delta = currRootWorstCaseCost - leastWorstCaseCost;
				if (((delta*100)/currRootWorstCaseCost) >= sm_mcRootCostDeltaThreshold)	{
					if (smDebugPerf) {
						IB_LOG_INFINI_INFO_FMT(__func__,
						 "Changing MC Spanning Tree Root Switch."
						 "Least worst case cost Old Root %d New Root %d. Delta is above threshold value of %d%%.",
						 currRootWorstCaseCost, leastWorstCaseCost, sm_mcRootCostDeltaThreshold);
					}
					if (vs_lock(&sm_mcSpanningTreeRootGuidLock) != VSTATUS_OK) {
						IB_LOG_ERROR0("error in getting mcSpanningTreeRootGuidLock");
						return;
					}
					sm_mcSpanningTreeRootGuid = leastWorstCostSwitchGuid;
			        (void)vs_unlock(&sm_mcSpanningTreeRootGuidLock);
					currRootWorstCaseCost = leastWorstCaseCost;
				} else if (smDebugPerf) {
					IB_LOG_INFINI_INFO_FMT(__func__,
					 "Not changing MC root switch. "
					 "Delta of Current root cost %d new least worst case cost %d below threshold value of %d%%",
					 currRootWorstCaseCost, leastWorstCaseCost, sm_mcRootCostDeltaThreshold);
				}

			} 
		}
	}

	/* communicate MC Root switch guid to standby SMs*/
	(void)sm_dbsync_syncMCRoot(DBSYNC_TYPE_FULL);

	if (smDebugPerf) {
		int found = 0;

		for_all_switch_nodes(sm_topop, nodep) {
			if (sm_mcSpanningTreeRoot_useLeastTotalCost &&
					(nodep->nodeInfo.NodeGUID == leastTotalCostSwitchGuid)) {
				IB_LOG_INFINI_INFO_FMT(__func__,
						 "Least total cost is %d. Switch is %s Guid "FMT_U64,
						 leastTotalCost, sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
				found++;
			}
		
			if (sm_mcSpanningTreeRoot_useLeastWorstCaseCost &&
					(nodep->nodeInfo.NodeGUID == leastWorstCostSwitchGuid)) {
				IB_LOG_INFINI_INFO_FMT(__func__,
						 "Least worst case cost is %d. Switch is %s Guid "FMT_U64,
						 leastWorstCaseCost, sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
				found++;
			}

			if (nodep->nodeInfo.NodeGUID == sm_mcSpanningTreeRootGuid) {
				IB_LOG_INFINI_INFO_FMT(__func__,
						 "Current Multicast Spanning Tree Root is %s Guid "FMT_U64,
						 sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, currRootTotalCost,
						 currRootWorstCaseCost);
				if (currRootTotalCost) {
					IB_LOG_INFINI_INFO_FMT(__func__,
						 "Current Spanning Tree Root Total cost is %d ", currRootTotalCost);
				}
				if (currRootWorstCaseCost) {
					IB_LOG_INFINI_INFO_FMT(__func__,
						 "Current Spanning Tree Root Least worst case cost is %d ", currRootWorstCaseCost);
				}

				found++;
			}

			if (found == 3)
				break;
		}
	}
}

Status_t
sm_routing_copy_floyds(Topology_t *src_topop, Topology_t *dst_topop)
{
	Status_t status;
	size_t   bytesCost;

	bytesCost = src_topop->max_sws * src_topop->max_sws * sizeof(uint16_t);

	status = vs_pool_alloc(&sm_pool, bytesCost, (void *)&dst_topop->cost);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("TT(topop): can't malloc cost array; rc:", status);
		IB_EXIT(__func__, status);
		return status;
	}

	dst_topop->bytes = bytesCost;

	memcpy((void *)dst_topop->cost, (void *)src_topop->cost, bytesCost);

	return VSTATUS_OK;
}

Status_t
sm_routing_copy_lfts(Topology_t *src_topop, Topology_t *dst_topop)
{
	Status_t status;
	Node_t   *nodep, *oldNodep;
	int      lftLength;

	for_all_switch_nodes(dst_topop, nodep) {

		if (nodep->switchInfo.LinearFDBCap == 0) {
			IB_LOG_ERROR_FMT(__func__, "switch doesn't support lft: %s",
			       sm_nodeDescString(nodep));
			continue;
		}

		oldNodep = sm_find_guid(src_topop, nodep->nodeInfo.NodeGUID);
		if (  oldNodep == NULL || oldNodep->lft == NULL
		   || nodep->switchInfo.LinearFDBTop != oldNodep->switchInfo.LinearFDBTop) {
			// IB_LOG_INFINI_INFO_FMT(__func__, "Full LFT for switch %s on old node checks", sm_nodeDescString(nodep));
			status = sm_setup_lft(dst_topop, nodep);
			continue;
		}

		if (  nodep->initPorts.nset_m
		   || !bitset_equal(&nodep->activePorts, &oldNodep->activePorts)) {
			// Active ports changed or moved, recalculate LFTs for this switch.
			// IB_LOG_INFINI_INFO_FMT(__func__, "Full LFT on port change for switch %s", sm_nodeDescString(nodep));
			status = sm_setup_lft(dst_topop, nodep);
			continue;
		}

		// PR-119954: This PR identified a memory leak that resulted in code in sm_routing_route_old_switch() being executed when 
		//			  nodep->lft pointed to an lft that was already allocated.  This resulted in a new lft being allocated without
		//			  freeing the already allocated lft.  The following is similar code.  While the following
		//			  code is not expected to be executed when node->lft is not zero, the following code has been added to 
		//			  free the lft if node->lft is found to be non NULL.  This will insure that no memory leak can occur.
		if (nodep->lft) {
			IB_LOG_INFINI_INFO_FMT(__func__, "new lft - switch %s nodep %p nodep->index %ld nodep->lft %p", 
														  sm_nodeDescString(nodep),   nodep,   nodep->index,    nodep->lft);
			vs_pool_free(&sm_pool, nodep->lft);
			nodep->lft = NULL;
		}
		lftLength = sizeof(PORT) * ROUNDUP(nodep->switchInfo.LinearFDBTop+1,MAX_LFT_ELEMENTS_BLOCK);
		if ((status = vs_pool_alloc(&sm_pool, lftLength, (void *)&nodep->lft)) != VSTATUS_OK) {
			IB_FATAL_ERROR("sm_routing_copy_lfts: CAN'T ALLOCATE SPACE FOR NODE'S LFT;  OUT OF MEMORY IN SM MEMORY POOL!  TOO MANY NODES!!");
			return status;
		}

		memcpy((void *)nodep->lft, (void *)oldNodep->lft, lftLength);
		if (nodep->arSupport) {
			if (nodep->pgt && oldNodep->pgt) {
				memcpy((void *)nodep->pgt, (void *)oldNodep->pgt, sizeof(STL_PORTMASK)*(nodep->switchInfo.PortGroupCap));
				nodep->pgtLen = oldNodep->pgtLen;
			}
			if (nodep->arSupport && oldNodep->pgft) {
				sm_Node_copy_pgft(nodep, oldNodep);
			}

			sm_Node_prune_portgroups(nodep);
		}

		if (  new_endnodesInUse.nset_m
		   || src_topop->num_endports != dst_topop->num_endports) {
			// End node change, calculate deltas LFT changes to switch.
			// IB_LOG_INFINI_INFO_FMT(__func__, "Delta route calc for additional endports for switch %s", sm_nodeDescString(nodep));
			status = sm_setup_lft_deltas(src_topop, dst_topop, nodep);
		}
	}

	status = sm_dispatch_wait(&sm_asyncDispatch);
	if (status != VSTATUS_OK) {
		sm_dispatch_clear(&sm_asyncDispatch);
		IB_LOG_ERROR_FMT(__func__,
			"Failed to wait for dispatcher completion (rc %d)", status);
		return status;
	}

	return VSTATUS_OK;
}

/*
 * Builds the LFT for a switch, then sends the part needed for the switch to 
 * talk to the SM.
 */
Status_t
sm_routing_prep_new_switch(Topology_t *topop, Node_t *nodep, int use_lr_dr_mix, uint8_t *path)
{
	Status_t	status;
	
	if (nodep->switchInfo.LinearFDBCap == 0) {
		IB_LOG_ERROR_FMT(__func__,
				"switch %s doesn't support LFT, can't program minimal LFT for early LID routing",
		       sm_nodeDescString(nodep));
		return VSTATUS_BAD;
	}

	if (topop->routingModule->funcs.needs_lft_recalc(topop, nodep)) {
		/* calculate the full LFTs for this switch*/
		if ((status = sm_calculate_lft(sm_topop, nodep)) != VSTATUS_OK)
			return status;
	}

	/* setup LFT blocks for the SM LID and the switch's own LID*/
	//IB_LOG_INFINI_INFO_FMT(__func__,
	//		 "writing minimal lft for switch "FMT_U64, nodep->nodeInfo.NodeGUID);
	status = sm_write_minimal_lft_blocks(sm_topop, nodep, use_lr_dr_mix, path);

	return status;
}


Status_t
sm_routing_route_new_switch_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance)
{
	Status_t	status = FERROR;

	if (esmLoopTestOn) {
			Node_t	*nodep;
			SwitchList_t *sw;
			for_switch_list_switches(swlist, sw) {
				nodep = sw->switchp;
				if (nodep->switchInfo.LinearFDBCap != 0)
					status = sm_setup_lft(topop, nodep);
			}
	} else {
		// IB_LOG_INFINI_INFO0("Setting up full lfts for swlist");

	 	if ((status = sm_write_full_lfts_by_block_LR(topop, swlist, rebalance)) != VSTATUS_OK) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Error in writing full lfts");
		}
	}

	return status;
}

Status_t
sm_routing_route_old_switch(Topology_t *src_topop, Topology_t *dst_topop, Node_t *nodep)
{
	Status_t status;
	Node_t   *oldNodep;
	int      lftLength;

	if (nodep->switchInfo.LinearFDBCap == 0) {
		IB_LOG_ERROR_FMT(__func__, "switch doesn't support lft %s",
			   sm_nodeDescString(nodep));
		return VSTATUS_BAD;
	}

	// Kludge for dealing with this func requires dst_topop == sm_topop
	if (dst_topop != sm_topop) {
		IB_LOG_ERROR_FMT(__func__, "This function only works for dst_topop == sm_topop");
		return VSTATUS_ILLPARM;
	}

	if (dst_topop->routingModule->funcs.check_switch_path_change(src_topop, dst_topop, nodep)) {
		//IB_LOG_INFINI_INFO_FMT(__func__, "Full route calc for switch with path change %s", sm_nodeDescString(nodep));
		if (dst_topop->routingModule->funcs.needs_lft_recalc(dst_topop, nodep)) {
			status = sm_setup_lft(dst_topop, nodep);
		} else {
			IB_LOG_INFINI_INFO_FMT(__func__,
				"Full LFT send switch with path change %s", sm_nodeDescString(nodep));
			status = sm_send_lft(dst_topop, nodep); 
		}
		return status;
	}

	oldNodep = sm_find_guid(src_topop, nodep->nodeInfo.NodeGUID);
	if (!oldNodep || !oldNodep->lft) {
		if (dst_topop->routingModule->funcs.needs_lft_recalc(dst_topop, nodep)) {
			status = sm_setup_lft(dst_topop, nodep);
		} else {
			IB_LOG_INFINI_INFO_FMT(__func__,
				"Full LFT send switch with no old data %s", sm_nodeDescString(nodep));
			status = sm_send_lft(dst_topop, nodep);
		}
		return status;
	}

	if (  nodep->initPorts.nset_m
	   || !bitset_equal(&nodep->activePorts, &oldNodep->activePorts)) {
		// Active ports changed or moved, recalculate LFTs for this switch.
		if (dst_topop->routingModule->funcs.needs_lft_recalc(dst_topop, nodep)) {
			status = sm_setup_lft(dst_topop, nodep);
		} else {
			IB_LOG_INFINI_INFO_FMT(__func__,
				"Full LFT send switch with Port Change %s", sm_nodeDescString(nodep));
			status = sm_send_lft(dst_topop, nodep); 
		}
		//IB_LOG_INFINI_INFO_FMT(__func__, "Full route calc for switch with added/removed link %s", sm_nodeDescString(nodep));
		return status;
	}

	// Why do we use sm_topop here instead of src_topop or dst_topop?
	if (sm_topop->routingModule->funcs.can_send_partial_lft()) {
		IB_LOG_INFO_FMT(__func__, "Partial LFT send - switch %s", sm_nodeDescString(nodep));
		sm_send_partial_lft(sm_topop, nodep, &sm_topop->deltaLidBlocks);
		return VSTATUS_OK;
	}

	// PR-119954: This PR identified a memory leak that resulted from the following vs_pool_alloc() being executed when nodep->lft pointed to
	//  		  a lft that was already allocated.  A fix was made to topology_setup_switches_LR_DR() so that this code should never execute
	//  		  when nodep->lft points to an lft.  Although this should no longer happen, the following code is added to prevent a memory leak
	// 			  by freeing the lft, if node->lft is found to be non NULL.
	if (nodep->lft) {
		IB_LOG_INFINI_INFO_FMT(__func__, "new lft - switch %s nodep %p nodep->index %ld nodep->lft %p", 
																sm_nodeDescString(nodep),   nodep,   nodep->index,    nodep->lft);
		vs_pool_free(&sm_pool, nodep->lft);
		nodep->lft = NULL;
	}
	// Just additions, adjust LFT blocks with removed or new lids.
	lftLength = sizeof(PORT) * ROUNDUP(nodep->switchInfo.LinearFDBTop+1,MAX_LFT_ELEMENTS_BLOCK);
	if ((status = vs_pool_alloc(&sm_pool, lftLength, (void *)&nodep->lft)) != VSTATUS_OK) {
		IB_FATAL_ERROR("sm_routing_route_old_switch: CAN'T ALLOCATE SPACE FOR NODE'S LFT;  OUT OF MEMORY IN SM MEMORY POOL!  TOO MANY NODES!!");
		return VSTATUS_NOMEM;	/*calling function can use this value to abort programming old switches*/
	}

	if (nodep->switchInfo.LinearFDBTop > oldNodep->switchInfo.LinearFDBTop) {
		memcpy((void *)nodep->lft, (void *)oldNodep->lft, 
			sizeof(PORT) * (oldNodep->switchInfo.LinearFDBTop + 1));
		memset((void *)&(nodep->lft[oldNodep->switchInfo.LinearFDBTop + 1]), 
			0xff, lftLength - oldNodep->switchInfo.LinearFDBTop);
	} else {
		memcpy((void *)nodep->lft, (void *)oldNodep->lft, 
			sizeof(PORT) * (nodep->switchInfo.LinearFDBTop + 1));
	}

	if (nodep->pgt && oldNodep->pgt) {
		memcpy((void *)nodep->pgt, (void *)oldNodep->pgt, 
			sizeof(STL_PORTMASK)*(nodep->switchInfo.PortGroupCap));
		nodep->pgtLen = oldNodep->pgtLen;
	}

	if (oldNodep->pgft) {
		sm_Node_copy_pgft(nodep, oldNodep);
	}

	//IB_LOG_INFINI_INFO_FMT(__func__, "Delta route calc for switch with same paths %s", sm_nodeDescString(nodep));
	return sm_setup_lft_deltas(src_topop, dst_topop, nodep);

}

Status_t sm_routing_route_switch_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance)
{
// PR-119954 reported a memory leak.  It occurs in the case when sm_calculate_lft() is called to allocate a new lft for the new   
//  		 root node structure and this lft is replaced by another lft in sm_routing_route_old_switch_LR() without
//  		 freeing the lft allocated in sm_calculate_lft().  topology_setup_switches_LR_DR() has been updated to set the third parameter (rebalance) 
//  		 to route_root_switch ORed with rebalance when calling sm_routing_route_switch_LR().
//  		 This prevents sm_routing_route_old_switch_LR() from being called in the above scenario.  This fixes the memory leak.  
//  		 No changes needed to be made to sm_routing_route_switch_LR().

	Status_t status;
	SwitchList_t *sw;

	/* Check and program any new switches in the switch list*/
	if ((status = sm_routing_route_new_switch_LR(topop, swlist, rebalance)) != VSTATUS_OK) {
		goto dispatch_wait;
	}

	if (esmLoopTestOn) {
		// full LFTs just programmed for entire swlist, no need to check old switches.
		goto dispatch_wait; 
	}

	if (!old_switchesInUse.nset_m || sm_config.force_rebalance || rebalance)
		goto dispatch_wait;	/* In this case old switches would also have been handled above*/

	/* Check and program any old switches in the switch list*/
	for_switch_list_switches(swlist, sw) {
		if (!bitset_test(&old_switchesInUse, sw->switchp->swIdx))
			continue;
		//IB_LOG_INFINI_INFO_FMT(__func__, "routing old switch "FMT_U64, nodep->nodeInfo.NodeGUID);
		status = sm_routing_route_old_switch(&old_topology, topop, sw->switchp);
		if (status != VSTATUS_OK)
			break;
	}

dispatch_wait:
	if (status != VSTATUS_OK) {
		sm_dispatch_clear(&sm_asyncDispatch);
	} else {
		status = sm_dispatch_wait(&sm_asyncDispatch);
		if (status != VSTATUS_OK) {
			sm_dispatch_clear(&sm_asyncDispatch);
			IB_LOG_ERROR_FMT(__func__,
					"Failed to wait for dispatcher completion (rc %d)", status);
		}
	}

	return status;
}

int
sm_balance_base_lids(SwitchportToNextGuid_t *ordered_ports, int olen)
{
	int i, j;
	uint16_t bestLidsRouted = 0xffff;
	uint16_t bestSwLidsRouted = 0xffff;

	for (i = 0, j = 0; i < olen; ++i) {
		if (ordered_ports[i].portp->portData->baseLidsRouted < bestLidsRouted) {
			bestLidsRouted = ordered_ports[i].portp->portData->baseLidsRouted;
			bestSwLidsRouted = ordered_ports[i].nextSwp->numBaseLidsRouted;
			j = i;
		} else if (ordered_ports[i].portp->portData->baseLidsRouted == bestLidsRouted) {
			if (ordered_ports[i].nextSwp->numBaseLidsRouted < bestSwLidsRouted) {
				bestSwLidsRouted = ordered_ports[i].nextSwp->numBaseLidsRouted;
				j = i;
			}
		}
	}

	return j;
}

static __inline__ void
incr_lids_routed(Topology_t *topop, Node_t *switchp, int port) {

	Port_t *swPortp;
	Node_t *nextSwp;

	if (sm_valid_port((swPortp = sm_get_port(switchp, port))) &&
						swPortp->state >= IB_PORT_INIT) {
		nextSwp = sm_find_node(topop, swPortp->nodeno);
		if (nextSwp) {
   			nextSwp->numLidsRouted++;
		}
		swPortp->portData->lidsRouted++;
	}
}

Status_t sm_calculate_balanced_lfts_systematic(Topology_t *topop)
{
	Node_t *nodep, *switchp, *toSwitchp;
	Port_t *portp, *swPortp, *toSwitchPortp;
	Status_t status;
	uint8_t portGroup[128];
	int upDestCount = 0;
	int downDestCount = 0;
	int currentLid, numPorts, t, i;
	int r, routingIterations;
	uint64_t sTime, eTime;
	int ht, hfiTierEnd=0;
	int isUp = 1;

	if (smDebugPerf) {
		vs_time_get(&sTime);
		if (sm_config.ftreeRouting.debug) {
			IB_LOG_INFINI_INFO0("entry");
		}
	}

	// If we can't guarantee that all HFIs/FIs are on the same tier,
	// balancing the LFTs is more complicated.
	if (!sm_config.ftreeRouting.fis_on_same_tier) {
		hfiTierEnd = sm_config.ftreeRouting.tierCount-1;
	}

	for_all_switch_nodes(topop, switchp) {
		if (!Is_Switch_Queued(topop, switchp)) continue;

		status = sm_Node_init_lft(switchp, NULL);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
			return status;
		}
	}

	// if route last is indicated, balance over those HFIs on a second pass
	routingIterations = (strlen(sm_config.ftreeRouting.routeLast.member) == 0) ? 1 : 2;

	for (r=0; r<routingIterations; ++r) {
		for (t=0; t<sm_config.ftreeRouting.tierCount; ++t) {
			for_all_tier_switches(topop, switchp, t) {
				upDestCount = downDestCount = 0;

				// hfiTierEnd will be set to zero when config indicates all 
				// HFIs are on same the same tier.  If not, we need to visit 
				// all tiers since HFIs may be attached anywhere.
				for (ht=0; ht<=hfiTierEnd; ht++) {
					// Iterate over all switches on the FI/HFI tier.
					for_all_tier_switches(topop, toSwitchp, ht) {
						// If this switch has no FI/HFIs, skip it.
						if (!toSwitchp->edgeSwitch) continue;

						if ((r==1) && !toSwitchp->skipBalance) {
							// Second pass, done with this switch's attached HFIs.
							continue;
						}

						if (switchp == toSwitchp) {
							// handle direct attach - build LFT entries for 
							// FI/HFIs attached to this switch.
							numPorts = 0;
						} else {
							// get a list of valid egress parts from switchp to toSwitchp.
							numPorts = topop->routingModule->funcs.get_port_group(topop, switchp, toSwitchp, portGroup);
							if (!numPorts) continue;
		
							// portGroup will all be up or down per spine first routing.
							swPortp = sm_get_port(switchp, portGroup[0]);
	
							isUp = (swPortp && swPortp->portData->uplink);
						}
		
						for_all_physical_ports(toSwitchp, toSwitchPortp) {
			   				if (!sm_valid_port(toSwitchPortp) || toSwitchPortp->state <= IB_PORT_DOWN) continue;
	
							// If the node connected to this port isn't a FI or is
							// part of the skip balance list, skip to the next port.
							// It will be setup on second pass.
			   				nodep = sm_find_node(topop, toSwitchPortp->nodeno);
			   				if (!nodep) continue;
			   				if (nodep->nodeInfo.NodeType != NI_TYPE_CA) continue;
							if ((r==0) && nodep->skipBalance) {
								toSwitchp->skipBalance = 1;
								continue;
							}
	
							for_all_end_ports(nodep, portp) {
								if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
		
								i = 0;
								for_all_port_lids(portp, currentLid) {
									// Handle the case where switchp == toSwitchp. 
									// In this case, the target LID(s) are directly
									// connected to the local switchp port.
									if (!numPorts) {
										switchp->lft[currentLid] = toSwitchPortp->index;
										continue;
									}
	
									if (isUp) {
										switchp->lft[currentLid] = portGroup[(i+upDestCount + switchp->tierIndex*switchp->uplinkTrunkCnt) % numPorts];
									} else {
	  									switchp->lft[currentLid] = portGroup[(i+downDestCount + switchp->tierIndex) % numPorts];
									}
	
									if (sm_config.ftreeRouting.debug) 
										IB_LOG_INFINI_INFO_FMT(__func__, "Switch %s to %s lid 0x%x outport %d (of %d) tierIndex %d uplinkTrunk %d", 
											sm_nodeDescString(switchp), sm_nodeDescString(nodep), currentLid,
											switchp->lft[currentLid], numPorts, switchp->tierIndex, switchp->uplinkTrunkCnt);
		
									incr_lids_routed(topop, switchp, switchp->lft[currentLid]);
	
									// Disperse lmc lids to different upstream switches
									if (isUp && (switchp->uplinkTrunkCnt > 1) && switchp->trunkGrouped) {
										// Uplink trunk group is "grouped" ports:
										// that is, port 1-4 to same switch, 5-8 next switch, etc.
										i += switchp->uplinkTrunkCnt;
										if ((i % numPorts) == 0) {
											// add one so we don't use the same port
											i++;
										}
									} else {
										// If uplink trunk group, it has staggered ports:
										// that is, ports 1,5,9,13 to same switch, ports 2,6,10,14
										// to next switch, etc.
										i++;
									}
								}
								if (isUp) {
									upDestCount += 1;
								} else {
									downDestCount += 1;
								}
							}
						}
					}
				}
			}
		}
	}

	// Switch-to-Switch routes are very lightly used (management traffic
	// only) so we don't bother trying to balance them.
	for_all_switch_nodes(topop, switchp) {
		if (Is_Switch_Queued(topop, switchp)) {
			// Setup switch to switch routing
			for_all_switch_nodes(topop, nodep) {
				for_all_end_ports(nodep, portp) {
					if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
					topop->routingModule->funcs.setup_xft(topop, switchp, nodep, portp, portGroup);
					for_all_port_lids(portp, currentLid) {
						switchp->lft[currentLid] = portGroup[currentLid - portp->portData->lid];
					}
				}
				if (topop->routingModule->funcs.setup_pgs) {
					status = topop->routingModule->funcs.setup_pgs(topop, switchp, nodep);
				}
			}
		} else {
			// Switch may not be added to tier switch group when no FIs in down path.
			// If so, doesn't matter if it is balanced.
			sm_calculate_lft(topop, switchp);
		}
	}

	if (smDebugPerf) {
		vs_time_get(&eTime);
		IB_LOG_INFINI_INFO("END; elapsed time(usecs)=", (int)(eTime-sTime));
	} else if (sm_config.ftreeRouting.debug) {
		IB_LOG_INFINI_INFO0("_systematic:exit");
	}

	return VSTATUS_OK;
}

Status_t sm_copy_balanced_lfts(Topology_t *topop)
{
	int			lftLength;
	Node_t		*switchp, *oldnodep;
	Port_t		*portp, *oldportp;
	Status_t	status=VSTATUS_OK;

	IB_ENTER(__func__, topop, 0, 0, 0);

	if (sm_config.ftreeRouting.debug) {
		IB_LOG_INFINI_INFO0("entry");
	}

	for_all_switch_nodes(topop, switchp) {
		lftLength = sizeof(PORT) * ROUNDUP(switchp->switchInfo.LinearFDBTop+1, MAX_LFT_ELEMENTS_BLOCK);
		if ((status = vs_pool_alloc(&sm_pool, lftLength, (void *)&switchp->lft)) != VSTATUS_OK) {
			IB_FATAL_ERROR("sm_copy_balanced_lfts: CAN'T ALLOCATE SPACE FOR NODE'S LFT;  OUT OF MEMORY IN SM MEMORY POOL!  TOO MANY NODES!!");
			return VSTATUS_NOMEM;	/*calling function can use this value to abort programming old switches*/
		}

		oldnodep = switchp->old;
		if (!oldnodep) {
			// shouldn't get here
			continue;
		}
		
		switchp->numLidsRouted = oldnodep->numLidsRouted;

		// Don't copy the portgroups if the switch-switch linkage may have changed.
		boolean copyPgs = (!switchp->initPorts.nset_m && bitset_equal(&switchp->activePorts, &oldnodep->activePorts));

		if (switchp->switchInfo.LinearFDBTop > oldnodep->switchInfo.LinearFDBTop) {
			memcpy((void *)switchp->lft, (void *)oldnodep->lft, 
				sizeof(uint8_t) * (oldnodep->switchInfo.LinearFDBTop + 1));
			if (oldnodep->switchInfo.LinearFDBTop < lftLength) {
				memset((void *)&(switchp->lft[oldnodep->switchInfo.LinearFDBTop + 1]), 
					0xff, lftLength - oldnodep->switchInfo.LinearFDBTop - 1);
			}
		} else {
			memcpy((void *)switchp->lft, (void *)oldnodep->lft, 
				sizeof(uint8_t) * (switchp->switchInfo.LinearFDBTop + 1));
			if (switchp->switchInfo.LinearFDBTop < lftLength) {
				memset((void *)&(switchp->lft[switchp->switchInfo.LinearFDBTop + 1]), 
					0xff, lftLength - switchp->switchInfo.LinearFDBTop - 1);
			}
		}

		if (copyPgs) {
			if (switchp->pgt && oldnodep->pgt) {
				memcpy((void *)switchp->pgt, (void *)oldnodep->pgt, 
					sizeof(STL_PORTMASK)*(switchp->switchInfo.PortGroupCap));
				switchp->pgtLen = oldnodep->pgtLen;
			}
			if (oldnodep->pgft) {
				sm_Node_copy_pgft(switchp, oldnodep);
			}

			sm_Node_prune_portgroups(switchp);
		}

		for_all_ports(switchp, portp) {
			if (sm_valid_port(portp) && (portp->state > IB_PORT_DOWN)) {
				oldportp = sm_get_port(oldnodep, portp->index);
				if (oldportp && sm_valid_port(oldportp)) {
					portp->portData->lidsRouted = oldportp->portData->lidsRouted;
				}
			}
		}
	}

	return VSTATUS_OK;
}

Status_t
sm_Node_prune_portgroups(Node_t * switchp)
{
	if (!switchp->pgt) {
		return VSTATUS_OK;
	}

	int i;
	for (i = 0; i < switchp->switchInfo.PortGroupTop; ++i) {
		uint8_t start = (i/STL_PORT_MASK_WIDTH)*STL_PORT_MASK_WIDTH;
		uint8_t portIdx = start + 1;
		STL_PORTMASK pm = switchp->pgt[i];
		uint8_t maxRate = 0;

		STL_PORTMASK newPm = 0;
		while (pm != 0) {
			if ((pm & 0x1) == 0) {
				portIdx++;
				pm >>= 1;
				continue;
			}

			Port_t * port = &switchp->port[portIdx];
			if (!sm_valid_port(port) || port->state < IB_PORT_INIT) continue;

			if (linkrate_ge(port->portData->rate, maxRate)) {
				if (linkrate_gt(port->portData->rate,maxRate)) {
					newPm = 0;
					maxRate = port->portData->rate;
				}
				newPm |= ((STL_PORTMASK)1) << (port->index - 1) % STL_PORT_MASK_WIDTH;
			}

			portIdx++;
			pm >>= 1;
		}

		if (newPm != switchp->pgt[i]) {
			switchp->pgt[i] = newPm;
			switchp->arChange = 1;
		}
	}

	return VSTATUS_OK;
}


Status_t sm_calculate_balanced_lft_deltas(Topology_t *topop)
{
	uint16_t	lid, curBlock;
	Node_t		*switchp, *nodep;
	Port_t		*portp;
	uint8_t		xftPorts[128];
	uint16_t	curLid;

	IB_ENTER(__func__, topop, 0, 0, 0);

	if (smDebugPerf || sm_config.ftreeRouting.debug) {
		IB_LOG_INFINI_INFO0("entry");
	}

	sm_copy_balanced_lfts(topop);

	curBlock = 0xffff;

	for (lid=0; lid <= topop->maxLid; ++lid) {
		nodep = lidmap[lid].newNodep;
		portp = lidmap[lid].newPortp;

		// Handle the case where an HFI has disappeared from the fabric.
		if (!nodep &&
			lidmap[lid].oldNodep &&
			lidmap[lid].oldNodep->nodeInfo.NodeType == NI_TYPE_CA) {

			// Remove the HFI from all LFTs.
			for_all_switch_nodes(topop, switchp) {
				switchp->lft[lid] = 0xff;
				if (curBlock != lid/LFTABLE_LIST_COUNT) {
					// Set lft block num
					curBlock = lid/LFTABLE_LIST_COUNT;
					// if (!bitset_test(&topop->deltaLidBlocks, curBlock))
					//  	IB_LOG_INFINI_INFO_FMT(__func__,
					//			"Setting curBlock %d due to delete of lid 0x%x", curBlock, lid);

					bitset_set(&topop->deltaLidBlocks, curBlock);
				}
			}
			continue;
		}

		// If Node (switch or HFI) has left the fabric, skip it.
		if (!nodep) continue;
		if (!sm_valid_port(portp)) continue;
		if (portp->state < IB_PORT_INIT) continue;

		// Base lid?
		if (lid != portp->portData->lid) continue;
		if (!bitset_test(&new_endnodesInUse, nodep->index)) continue;

		// This is a new node. Add it to the list of LFT records that have to be
		// sent.
		if (curBlock != lid/LFTABLE_LIST_COUNT) {
			// Set lft block num
			curBlock = lid/LFTABLE_LIST_COUNT;
			// if (!bitset_test(&topop->deltaLidBlocks, curBlock))
			//		 IB_LOG_INFINI_INFO_FMT(__func__,
			//			"Setting curBlock %d due to ADD of lid 0x%x", curBlock, lid);
			bitset_set(&topop->deltaLidBlocks, curBlock);
		}

		// Add the node to all switch LFTs.
		for_all_switch_nodes(topop, switchp) {
			topop->routingModule->funcs.setup_xft(topop, switchp, nodep, portp, xftPorts);
			for_all_port_lids(portp, curLid) {
				switchp->lft[curLid] = xftPorts[curLid - portp->portData->lid];
			}
		}
	}

	if (topop->routingModule->funcs.setup_pgs) {
		// By the time this function has been called, the port group
		// mappings should have been invalidated for all switches that
		// had ports/links change directly on them.
		//
		// All switch pairs will have to have their PG mappings recomputed.
		// This does not imply that all switch PG mappings will change.
		for_all_switch_nodes(topop, switchp) {
			Node_t * otherSwitch = NULL;
			for_all_switch_nodes(topop, otherSwitch) {
				topop->routingModule->funcs.setup_pgs(topop, switchp, otherSwitch);
			}
		}
	}

	bitset_info_log(&topop->deltaLidBlocks, "deltaLidBlocks");
	return VSTATUS_OK;
}

/*
 * Called when fabric routing is to be updated, calculates routing for
 * all switches in the fabric.  This is the default routing method.
 *
 * Prior method of setting up routing by traversing switch list then LIDs 
 * lead to highly unbalanced routing.  This method iterates over LIDs then 
 * switches to balance the paths across all uplinks in a tree.
 */
Status_t
sm_calculate_all_lfts(Topology_t * topop)
{
	Node_t *nodep, *switchp;
	Port_t *portp;
	Status_t status;
	int programLft,i;
	uint16_t lid, portLid;
	uint8_t xftPorts[128];

	for_all_switch_nodes(topop, switchp) {
		status = sm_Node_init_lft(switchp, NULL);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
			return status;
		}
	}

	for (i=0; i<2; i++) {
		for (lid = 0; lid <= topop->maxLid; ++lid) {
			// grab the node corresponding to the current lid
			nodep = lidmap[lid].newNodep;
			portp = lidmap[lid].newPortp;
			// we'll program the entire LMC range in one go, so only
			// program if we're at the beginning of the range
			programLft = (nodep != NULL && sm_valid_port(portp)
					  	&& portp->state > IB_PORT_DOWN && lid == portp->portData->lid)
					? 1 : 0;
			if (!programLft)
				continue;

			// While slighly more costly, iterating over edge switches first gives 
			// the best dispersion of routes.  
			// TODO:  If too costly, fall back to one pass over switches or separate
			// the switch lists by type.  One pass is still a huge improvement over
			// prior routing methods.
			for_all_switch_nodes(topop, switchp) {
				if ((i==0) && !switchp->edgeSwitch) continue;
				if ((i==1) && switchp->edgeSwitch) continue;
				status = topop->routingModule->funcs.setup_xft(topop, switchp, nodep, portp, xftPorts);
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "Failed to setup xft for %s (0x%"PRIx64")",
						nodep->nodeDesc.NodeString,
						nodep->nodeInfo.NodeGUID);
					return status;
				}
	
				if (topop->routingModule->funcs.setup_pgs && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
					status = topop->routingModule->funcs.setup_pgs(topop, switchp, nodep); 
					if (status != VSTATUS_OK) {
						IB_LOG_ERROR_FMT(__func__, "Failed to setup port groups for %s (0x%"PRIx64")",
							nodep->nodeDesc.NodeString,
							nodep->nodeInfo.NodeGUID);
						return status;
					}
				}
				for_all_port_lids(portp, portLid) {
					switchp->lft[portLid] = xftPorts[portLid - lid];
				}
			}
	    }
	}
	return VSTATUS_OK;
}

Status_t
sm_routing_init(Topology_t * topop)
{
	Status_t status = VSTATUS_BAD;

    cl_qmap_init(&moduleFacMap, _qmap_str_cmp);

    if ((status = sm_routing_addModuleFac("dummyModule", _dummy_mod_fac)) != VSTATUS_OK)
        return status;

	status = sm_shortestpath_init(topop);
	if (status != VSTATUS_OK)
		return status;

	status = sm_dgmh_init(topop);
	return status;
}
