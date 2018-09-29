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

/* [ICS VERSION STRING: unknown] */

#include "ib_types.h"
#include "sm_l.h"
#include "sm_qos.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "fm_xml.h"

// Internal Helper Functions

typedef struct GuidCounter
{
	uint64_t guid;
	uint32_t offset;
	uint32_t count;
} GuidCounter_t;

static int
_compare_guids(const void * arg1, const void * arg2)
{
	SwitchportToNextGuid_t * sport1 = (SwitchportToNextGuid_t *)arg1;
	SwitchportToNextGuid_t * sport2 = (SwitchportToNextGuid_t *)arg2;

	if (sport1->guid < sport2->guid)
		return -1;
	else if (sport1->guid > sport2->guid)
		return 1;
	else
		return 0;
}

static int
_compare_lids_routed(const void * arg1, const void * arg2)
{
	SwitchportToNextGuid_t * sport1 = (SwitchportToNextGuid_t *)arg1;
	SwitchportToNextGuid_t * sport2 = (SwitchportToNextGuid_t *)arg2;

	if (sport1->portp->portData->lidsRouted < sport2->portp->portData->lidsRouted)
		return -1;
	else if (sport1->portp->portData->lidsRouted > sport2->portp->portData->lidsRouted)
		return 1;
	else if (sport1->nextSwp->numLidsRouted < sport2->nextSwp->numLidsRouted) 
		return -1;
	else if (sport1->nextSwp->numLidsRouted > sport2->nextSwp->numLidsRouted)
		return 1;
	else
		return 0;
}

static int
_compare_guids_then_lids_routed(const void * arg1, const void * arg2)
{
	SwitchportToNextGuid_t * sport1 = (SwitchportToNextGuid_t *)arg1;
	SwitchportToNextGuid_t * sport2 = (SwitchportToNextGuid_t *)arg2;

	if (sport1->guid < sport2->guid)
		return -1;
	else if (sport1->guid > sport2->guid)
		return 1;
	else if (sport1->portp->portData->lidsRouted < sport2->portp->portData->lidsRouted)
		return -1;
	else if (sport1->portp->portData->lidsRouted > sport2->portp->portData->lidsRouted)
		return 1;
	else if (sport1->nextSwp->numLidsRouted < sport2->nextSwp->numLidsRouted) 
		return -1;
	else if (sport1->nextSwp->numLidsRouted > sport2->nextSwp->numLidsRouted)
		return 1;
	else
		return 0;
}

static int
_compare_lids_routed_then_guids(const void * arg1, const void * arg2)
{
	SwitchportToNextGuid_t * sport1 = (SwitchportToNextGuid_t *)arg1;
	SwitchportToNextGuid_t * sport2 = (SwitchportToNextGuid_t *)arg2;

	if (sport1->portp->portData->lidsRouted < sport2->portp->portData->lidsRouted)
		return -1;
	else if (sport1->portp->portData->lidsRouted > sport2->portp->portData->lidsRouted)
		return 1;
	else if (sport1->nextSwp->numLidsRouted < sport2->nextSwp->numLidsRouted) 
		return -1;
	else if (sport1->nextSwp->numLidsRouted > sport2->nextSwp->numLidsRouted)
		return 1;
	else if (sport1->guid < sport2->guid)
		return -1;
	else if (sport1->guid > sport2->guid)
		return 1;
	else if (sport1->portp->index < sport2->portp->index)
		return -1;
	else if (sport1->portp->index > sport2->portp->index)
		return 1;
	else
		return 0;
}

// -------------------------------------------------------------------------- //
//
//	This re-sorts an array of SwitchportToNextGuid_t structures by cycling
//  either the NodeGUID or SystemImageGUID.  The input is assumed to already
//  be numerically sorted by the desired GUID, and temporary data is stored
//  in a scratch space specified in the argument list, which must have room
//  for up to MaxNumberOfSwitchPorts * sizeof(GuidCounter_t)
//
//  As a superficial example, [1,1,2,2,2,3,3,3] would become [1,2,3,1,2,3,2,3]
//
static void
_guid_cycle_sort(SwitchportToNextGuid_t *ports, int portCount, void *scratch, int useSysGuid)
{
	int i, j;
	int uniqueGuids = 0;
	GuidCounter_t *gc = (GuidCounter_t*)scratch;
	SwitchportToNextGuid_t tmpPort;
	uint64_t prevGuid = 0;
	uint16_t sortIndex = 0;
	
	// any numerically ordered 1 or 2 element array is already done
	if (portCount <= 2)
		return;
	
	// collect info about how many of each guid there are, and
	// where in the array they are located
	//
	for (i = 0; i < portCount; i++) {
		if ((useSysGuid ? ports[i].sysGuid : ports[i].guid) != prevGuid) {
			gc[uniqueGuids].guid = useSysGuid ? ports[i].sysGuid : ports[i].guid;
			gc[uniqueGuids].offset = i;
			gc[uniqueGuids].count = 1;
			uniqueGuids++;
		} else {
			gc[uniqueGuids-1].count++;
		}
		ports[i].sortIndex = 0;
		prevGuid = useSysGuid ? ports[i].sysGuid : ports[i].guid;
	}
	
	// easy out scenarios: can't cycle [1,1,1,1] or [1,2,3,4]
	if (uniqueGuids <= 1 || uniqueGuids == portCount)
		return;
	
	// sort the array.  to preserve existing sort order of remaining
	// elements while determining cyclic order, we sort in 2 linear passes.
	// the first pass marks each element's final sort position
	//
	for (i = 0, j = 0; i < portCount; i++, j = (j + 1) % uniqueGuids) {
		// cycle to an available guid if necessary
		while (gc[j].count == 0) {
			j = (j + 1) % uniqueGuids;
		}
		ports[gc[j].offset++].sortIndex = sortIndex++;
		gc[j].count--;
	}
	
	// since the value we're sorting on is a final array index, we
	// can sort in one pass with < N swaps
	for (i = 0; i < portCount; i++) {
		// until we have the correct element in this position,
		// swap the current element into its final position
		while (ports[i].sortIndex != i) {
			memcpy(&tmpPort, &ports[i], sizeof(SwitchportToNextGuid_t));
			memcpy(&ports[i], &ports[tmpPort.sortIndex], sizeof(SwitchportToNextGuid_t));
			memcpy(&ports[tmpPort.sortIndex], &tmpPort, sizeof(SwitchportToNextGuid_t));
		}
	}
}

typedef struct {
	int matching; // true if we've found spines and are only
	              // considering them for routing
} SpineFirstState_t;

typedef enum {
	SPINE_FIRST_NONE,   // disabled or not matching
	SPINE_FIRST_FIRST,  // first match found
	SPINE_FIRST_MATCH,  // currently matching; match found
	SPINE_FIRST_NOMATCH // currently matching; no match found
} SpineFirstResult_t;

static __inline__ SpineFirstResult_t
_spine_first_test(SpineFirstState_t *state, Node_t *switchp, Port_t *portp, Node_t *next_nodep)
{
	if (switchp->nodeInfo.SystemImageGUID != next_nodep->nodeInfo.SystemImageGUID &&
		!portp->portData->uplink) {
		if (state->matching)
			// previously found spine but this isn't one; no match
			return SPINE_FIRST_NOMATCH;
		else
			// have not previously found a spine and this isn't one
			// either; keep returning none
			return SPINE_FIRST_NONE;
	}
	// sysimageguid matches or marked as uplink; this is a "spine"
	if (state->matching) return SPINE_FIRST_MATCH;
	// haven't matched yet; this is the first spine
	state->matching = 1;
	return SPINE_FIRST_FIRST;
}

static void
_balance_ports(Node_t *switchp, SwitchportToNextGuid_t *ordered_ports, int olen)
{
	int i, j;
#ifdef __VXWORKS__
	GuidCounter_t *scratch = (GuidCounter_t *)(ordered_ports + olen);
#else
	GuidCounter_t scratch[MAX_STL_PORTS] = {{0}};
#endif /* __VXWORKS__ */
	if (olen <= 0) return;

	if (switchp->internalLinks) {
		// no reason to sort on guid since only single link to each remote switch
		qsort(ordered_ports, olen, sizeof(SwitchportToNextGuid_t),
	      	_compare_lids_routed);
	} else {
		qsort(ordered_ports, olen, sizeof(SwitchportToNextGuid_t),
	      	_compare_guids_then_lids_routed);

		_guid_cycle_sort(ordered_ports, olen, (void*)(scratch), FALSE);
		for (i = 0, j = 0; i < olen; i++) {
			if (i == olen - 1 || ordered_ports[i].guid > ordered_ports[i+1].guid) {
				qsort(&ordered_ports[j], i - j + 1, sizeof(SwitchportToNextGuid_t),
			      	_compare_lids_routed_then_guids);
				j = i + 1;
			}
		}
	}
}

static Status_t
_handle_preassigned_sl(RoutingModule_t *rm, VirtualFabrics_t *vfs, bitset_t *usedSLs, int *numSCs)
{
	int qos;
	Status_t ret = VSTATUS_OK;

	for (qos=0; qos < vfs->number_of_qos_all; qos++) {
		QosConfig_t *pQos = &vfs->qos_all[qos];

		if (!pQos->qos_enable) {
			// Ignore any SLs set on nonQos VFs
			// Don't warn here. Config parser should have warned.
			if (pQos->base_sl != UNDEFINED_XML8) {
				pQos->base_sl = UNDEFINED_XML8;
			}
			if (pQos->resp_sl != UNDEFINED_XML8) {
				pQos->resp_sl = UNDEFINED_XML8;
			}
			if (pQos->mcast_sl != UNDEFINED_XML8) {
				pQos->mcast_sl = UNDEFINED_XML8;
			}
			continue;
		}

		if (pQos->base_sl != UNDEFINED_XML8) {
			if(!bitset_test(usedSLs, pQos->base_sl))
				*numSCs += rm->funcs.num_routing_scs(pQos->base_sl, 0);
			bitset_set(usedSLs, pQos->base_sl);
			if (rm->funcs.mcast_isolation_required() && pQos->contains_mcast) {
				if (pQos->mcast_sl == UNDEFINED_XML8) {
					IB_LOG_ERROR_FMT(__func__,
						"QOSGroup %s: Routing algorithm %s requires multicast isolation, but MulticastSL not configured",
						pQos->name, rm->name);
					ret = VSTATUS_BAD;
				} else if (pQos->mcast_sl == pQos->base_sl) {
					IB_LOG_ERROR_FMT(__func__,
						"QOSGroup %s: Routing algorithm %s requires multicast isolation, but MulticastSL matches BaseSL",
						pQos->name, rm->name);
					ret = VSTATUS_BAD;
				}
			}
		}
		if (pQos->resp_sl != UNDEFINED_XML8) {
			if(!bitset_test(usedSLs, pQos->resp_sl))
				*numSCs += rm->funcs.num_routing_scs(pQos->resp_sl, 0);
			bitset_set(usedSLs, pQos->resp_sl);
		}
		if (pQos->mcast_sl != UNDEFINED_XML8) {
			if(!bitset_test(usedSLs, pQos->mcast_sl))
				*numSCs += rm->funcs.num_routing_scs(pQos->mcast_sl, 1);
			bitset_set(usedSLs, pQos->mcast_sl);
			if (!rm->funcs.mcast_isolation_required() && pQos->base_sl != pQos->mcast_sl) {
					IB_LOG_WARN_FMT(__func__,
						"QOSGroup %s: Including configured MulticastSL %d, although routing algorithm %s doesn't require multicast isolation",
						pQos->name, (unsigned)pQos->mcast_sl, rm->name);
			}
		}
	}

	return ret;
}

static Status_t
_handle_unassigned_sl(QosConfig_t *pQos, uint8_t *sl, bitset_t *usedSLs, int *noqos, boolean highsls,
	int maxsl, char *text, int *numSCs, int scspersl)
{
	int newsl;
	if (*sl == UNDEFINED_XML8) {
		if (pQos->qos_enable || *noqos == -1){
			if (!highsls) {
				newsl = bitset_find_first_zero(usedSLs);
				if (newsl >= maxsl) {
					IB_LOG_ERROR_FMT(__func__, "QOSGroup %s: no unused SLs < %d available for %s",
						pQos->name, maxsl, text);
					return VSTATUS_BAD;
				}
			} else {
				newsl = bitset_find_last_zero(usedSLs);
				if (newsl == -1) {
					IB_LOG_ERROR_FMT(__func__, "QOSGroup: %s no unused SLs available for %s",
						pQos->name, text);
					return VSTATUS_BAD;
				}
			}
			bitset_set(usedSLs, newsl);
			*numSCs += scspersl;
			*sl = newsl;
			if (!pQos->qos_enable) *noqos = newsl;
		} else {
			*sl = *noqos;
		}
	}
	return VSTATUS_OK;
}

// The following functions are defined in sm_qos.c. They are
// particular to default implementation and so not exposed
// in sm_l.h
extern Status_t sm_update_bw(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
extern Status_t sm_assign_scs_to_sls_FixedMap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);

// Routing Functions available to any routing algorithm

Status_t
sm_routing_func_pre_process_discovery_noop(Topology_t *topop, void **outContext)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_discover_node_noop(Topology_t *topop, Node_t *nodep, void *context)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_discover_node_port_noop(Topology_t *topop, Node_t *nodep, Port_t *portp, void *context)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_post_process_discovery_noop(Topology_t *topop, Status_t discoveryStatus, void *context)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_post_process_routing_noop(Topology_t *topop, Topology_t *old_topop, int *rebalance)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_post_process_routing_copy_noop(Topology_t *src_topop, Topology_t *dst_topop, int *rebalance)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_alloc_cost_matrix_floyds(Topology_t *topop)
{
	Status_t status;
	size_t   req_bytesCost, req_bytesPath;

	/* Allocate space for the cost and path matrix. */
	req_bytesCost = topop->max_sws * topop->max_sws * sizeof(uint16_t);
	if (req_bytesCost > topop->bytesCost) {
		topop->bytesCost = 0;

		if (topop->cost != NULL) {
			(void)vs_pool_free(&sm_pool, (void *)topop->cost);
			topop->cost = NULL;
		}

		status = vs_pool_alloc(&sm_pool, req_bytesCost, (void *)&topop->cost);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("can't malloc cost array rc:", status);
			IB_EXIT(__func__, status);
			return status;
		}


		topop->bytesCost = req_bytesCost;
	}

	req_bytesPath = SM_PATH_SIZE(topop->max_sws);
	if (req_bytesPath > topop->bytesPath) {
		topop->bytesPath = 0;

		if (topop->path != NULL) {
			(void)vs_pool_free(&sm_pool, (void *)topop->path);
			topop->path = NULL;
		}

		status = vs_pool_alloc(&sm_pool, req_bytesPath, (void *)&topop->path);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("can't malloc path array rc:", status);
			IB_EXIT(__func__, status);
			return status;
		}

		memset(topop->path, 0, req_bytesPath);
		topop->bytesPath = req_bytesPath;
	}

	return VSTATUS_OK;
}

Status_t
sm_routing_func_init_cost_matrix_floyds(Topology_t *topop)
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
				topop->cost[ik] = topop->cost[ki] = MIN(topop->cost[ik], sm_GetCost(portp->portData));

				sm_path_portmask_set(topop->path + ik, portp->index);
				sm_path_portmask_set(topop->path + ki, portp->portno);
			}
		}
	}

	/* Clear the costs for each node to itself. */
	for (i = 0, ij = 0; i < topop->max_sws; i++, ij += topop->max_sws + 1) {
		topop->cost[ij] = 0;
	}

	return VSTATUS_OK;
}

Status_t
sm_routing_func_calc_cost_matrix_floyds(Topology_t * topop, int switches, unsigned short * cost, SmPathPortmask_t * path)
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
	Node_t *sw = topop->switch_head;

	if (switches != old_topology.max_sws || topology_passcount == 0) {
		topology_cost_path_changes = 1;
	}

	for (k = 0, kNumNodes = 0; k < switches; k++, kNumNodes += switches) {
#ifndef __VXWORKS__
#pragma omp parallel for private(j, ij, ik, kj, value) shared(cost, path)
#endif
		for (i = 0; i < switches; ++i) {

			ik = Index(i, k);

			if (cost[ik] == Cost_Infinity) {
				continue;
			}

			for (j = i + 1, ij = Index(i, j), kj = kNumNodes + i + 1; j < switches; ++j, ++ij, ++kj) {

				if ((value = cost[ik] + cost[kj]) < cost[ij]) {
					cost[ij] = value;
					// Take advantage of the fact that cost array is symmetric
					cost[Index(j, i)] = value;

					path[ij] = path[ik];
					path[Index(j, i)] = path[Index(j, k)];
					
				} else if (value == cost[ij]) {
					sm_path_portmask_merge(path + ij, path + ik);
					sm_path_portmask_merge(path + Index(j, i), path + Index(j, k));
				}
			}
		}

		if (smDebugPerf && (k & 0xFF) == 0xFF) {
			IB_LOG_INFINI_INFO_FMT(__func__, "completed run %d of %d", k, switches);
		}
	}

	/* All floyd costs are fully computed now and can be analyzed */
	while(sw) {
		total_cost = 0;
		max_cost = 0;
		k = sw->swIdx;
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
			

			/* PR 115770. If there is any switch cost/path change (including removal of switches),
			 * set topology_cost_path_changes which will force full LFT reprogramming for all switches.
			 */

			if (topology_cost_path_changes)
				continue;

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

		}
		sw = sw->type_next;
	}




	if (!sm_useIdealMcSpanningTreeRoot)
		return VSTATUS_OK;

	/* Based on the calculations, select the MC Spanning Tree Root Switch */

	old_root_exists = 0;
	if (sm_mcSpanningTreeRootGuid) {
		/* If we identified a root in a previous sweep or if we have just take over
		 * from a master, does the old root still exist ?*/
		if (sm_find_guid(topop, sm_mcSpanningTreeRootGuid)) 
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
				return VSTATUS_OK;
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
						return VSTATUS_OK;
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
				return VSTATUS_OK;
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
						return VSTATUS_OK;
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

		for_all_switch_nodes(topop, nodep) {
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
					sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
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
	return VSTATUS_OK;
}

int
sm_routing_func_routing_mode_noop(void)
{
	return STL_ROUTE_NOP;
}

int
sm_routing_func_routing_mode_linear(void)
{
	return STL_ROUTE_LINEAR;
}

int
sm_routing_func_false(void)
{
	return 0;
}

int
sm_routing_func_true(void)
{
	return 1;
}


STL_LID
sm_routing_func_get_reserved_lid(Topology_t * topop, Node_t * nodep, const Port_t* portp)
{
	return STL_LID_RESERVED;
}

Status_t
sm_routing_func_copy_routing_noop(Topology_t *src_topop, Topology_t *dst_topop)
{
	return VSTATUS_OK;
}

Status_t
sm_routing_func_copy_routing_lfts(Topology_t *src_topop, Topology_t *dst_topop)
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
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__, "Full LFT for switch %s on old node checks", sm_nodeDescString(nodep));
			status = sm_setup_lft(dst_topop, nodep);
			continue;
		}

		if (  nodep->initPorts.nset_m
		   || !bitset_equal(&nodep->activePorts, &oldNodep->activePorts)) {
			// Active ports changed or moved, recalculate LFTs for this switch if change involved ISL.
			if (dst_topop->routingModule->funcs.handle_fabric_change(dst_topop, oldNodep, nodep)) {
				if (sm_config.sm_debug_routing)
					IB_LOG_INFINI_INFO_FMT(__func__, "Full LFT on port change for switch %s", sm_nodeDescString(nodep));
				status = sm_setup_lft(dst_topop, nodep);
				continue;
			}
		}

		// PR-119954: This PR identified a memory leak that resulted in code in sm_routing_route_old_switch() being executed when 
		//			  nodep->lft pointed to an lft that was already allocated.  This resulted in a new lft being allocated without
		//			  freeing the already allocated lft.  The following is similar code.  While the following
		//			  code is not expected to be executed when node->lft is not zero, the following code has been added to 
		//			  free the lft if node->lft is found to be non NULL.  This will insure that no memory leak can occur.
		if (nodep->lft) {
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__, "new lft - switch %s nodep %p nodep->index %u nodep->lft %p",
						sm_nodeDescString(nodep), nodep, nodep->index, nodep->lft);
			vs_pool_free(&sm_pool, nodep->lft);
			nodep->lft = NULL;
		}
		lftLength = sizeof(PORT) * ROUNDUP(nodep->switchInfo.LinearFDBTop+1,MAX_LFT_ELEMENTS_BLOCK);
		if ((status = vs_pool_alloc(&sm_pool, lftLength, (void *)&nodep->lft)) != VSTATUS_OK) {
			IB_FATAL_ERROR_NODUMP("default_copy_routing_lfts: CAN'T ALLOCATE SPACE FOR NODE'S LFT;  OUT OF MEMORY IN SM MEMORY POOL!  TOO MANY NODES!!");
			return status;
		}

		memcpy((void *)nodep->lft, (void *)oldNodep->lft, lftLength);

		// Recover lidsRouted
		if (nodep->oldExists) {
			Node_t *oldnodep = nodep->old;
			if (oldnodep) {
				Port_t *portp, *oldportp;
				if (!nodep->numLidsRouted) nodep->numLidsRouted = oldnodep->numLidsRouted;
				for_all_ports(nodep, portp) {
					if (sm_valid_port(portp)) {
						oldportp = sm_get_port(oldnodep, portp->index);
						if (sm_valid_port(oldportp)) {
							portp->portData->lidsRouted = oldportp->portData->lidsRouted;
							/* PR: 143104 Get numLidsRouted from the next Switch, if it existed in the old topology. 
							 * Note the NodeGUID comparison below is a sanity check */
							Node_t *nextSwp = sm_find_node(dst_topop, portp->nodeno);
							if (nextSwp && (nextSwp->nodeInfo.NodeType == NI_TYPE_SWITCH) && nextSwp->oldExists) {
								Node_t *oldnextSwp = nextSwp->old;
								if (!nextSwp->numLidsRouted && (nextSwp->nodeInfo.NodeGUID == oldnextSwp->nodeInfo.NodeGUID))
									nextSwp->numLidsRouted = oldnextSwp->numLidsRouted;
							}
						}
					}
				}
			}
		}

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
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__, "Delta route calc for additional endports for switch %s",
						sm_nodeDescString(nodep));
			status = sm_setup_lft_deltas(src_topop, dst_topop, nodep);

		} else if (sm_config.sm_debug_routing) {
				IB_LOG_INFINI_INFO_FMT(__func__, "Copied LFTs for switch %s",
						sm_nodeDescString(nodep));
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


Status_t
sm_routing_func_init_switch_routing_lfts(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s = VSTATUS_OK;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (topology_cost_path_changes || *rebalance) {

		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		// If not, copy and delta updates handled by main topology method.
		s = sm_calculate_all_lfts(topop);
		*rebalance = 1;
		routing_recalculated = 1;
	}

	return s;
}


Status_t
sm_routing_func_calculate_lft(Topology_t * topop, Node_t * switchp)
{
	return sm_calculate_lft(topop, switchp);
}


// -------------------------------------------------------------------------- //
//
//	This is a common routine used by the LFT and RFT routines to parse
//	out what the path is through the fabric in order to setup the routing
//	tables.
//  See sm_l.h for parameter documentation
Status_t
sm_routing_func_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *nodep, Port_t *orig_portp, uint8_t *portnos)
{
	int i, j;
	uint8_t numLids;
	int lidsRoutedInc;
	int offset=0;
	int end_port = 0;
#ifdef __VXWORKS__
        SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
        memset(ordered_ports, 0, (sizeof(SwitchportToNextGuid_t) + sizeof(GuidCounter_t)) * switchp->nodeInfo.NumPorts);
#else
        SwitchportToNextGuid_t ordered_ports[MAX_STL_PORTS] = {{0}};
#endif /* __VXWORKS__ */

	IB_ENTER(__func__, switchp, nodep, orig_portp, 0);
	
	numLids =  1 << orig_portp->portData->lmc;
	lidsRoutedInc = ((nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) ? 1 : 0);
	memset((void*)portnos, 0xff, sizeof(uint8_t) * numLids);
	
	//
	//	If this is a FI, then we look for a path to the
	//	switch it is connected to.
	//
    i = switchp->swIdx;
    if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        j = nodep->swIdx;
    } else {
		Node_t *ntp = sm_find_node(topop, orig_portp->nodeno);
		if (ntp) {
			j = ntp->swIdx;
		} else {
			Node_t *ntp = orig_portp->portData->nodePtr;
			IB_LOG_ERROR_FMT(__func__,
				"Failed to find neighbor node %u, "FMT_U64" from NodeGUID "FMT_U64" Port %d",
				orig_portp->nodeno, ntp->nodeInfo.NodeGUID, nodep->nodeInfo.NodeGUID, orig_portp->index);
			j = -1;
			return VSTATUS_BAD;
		}
    }

	//
	//	If this node is hooked directly to the switch in question, we know the
	//	answer.
	//
	if (j == i) {
		// Nota Bene: note the implicit assumption that port numbers 
		// are 8 bits long..
		memset(portnos, orig_portp->portno, numLids);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	//
	//	We now are in a situation where in order to get from node[i]
	//	to node[j], we need to send to node[nodeno].  We need to find
	//	the ports which go to node[nodeno].
	//
	if (orig_portp->portData->lmc == 0) {
		// select best port, _select_ports will return 1 or 0 (no path)
		if ((end_port =  topop->routingModule->funcs.select_ports(topop, switchp, j, ordered_ports, 1)) == 0) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to find an outbound port on NodeGUID "FMT_U64" to NodeGUID "FMT_U64" Port %d",
				switchp->nodeInfo.NodeGUID, nodep->nodeInfo.NodeGUID, orig_portp->index);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}

		portnos[0] = ordered_ports[0].portp->index;

		// update number of LIDs routed through the chosen port
		if (portnos[0] != 0xff && nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			ordered_ports[0].portp->portData->lidsRouted += lidsRoutedInc;
			ordered_ports[0].nextSwp->numLidsRouted += lidsRoutedInc;
		}

	} else { // lmc > 0
		end_port =  topop->routingModule->funcs.select_ports(topop, switchp, j, ordered_ports, 0);
		if (!end_port) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to find outbound ports on NodeGUID "FMT_U64" to NodeGUID "FMT_U64" Port %d",
				switchp->nodeInfo.NodeGUID, nodep->nodeInfo.NodeGUID, orig_portp->index);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		if(end_port >= MAX_STL_PORTS) {
			IB_LOG_ERROR_FMT(__func__,"Number of ports are greater than maximum number of ports");
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}

		// balance the port order to filter the best to the top
		_balance_ports(switchp, ordered_ports, end_port);

		// reduce to the best 2^LMC paths
		if (end_port > numLids) end_port = numLids;

		// balance the final set of paths in terms of the base lid
		// by selecting an appropriate offset

		offset = sm_balance_base_lids(ordered_ports, end_port);

		// fill in outbound port number array
		for (i = 0; i < numLids; i++) {
			j = (i + offset) % end_port;
			portnos[i] = ordered_ports[j].portp->index;
			ordered_ports[j].portp->portData->lidsRouted += lidsRoutedInc;
			ordered_ports[j].nextSwp->numLidsRouted += lidsRoutedInc;
		}
		++ordered_ports[offset].portp->portData->baseLidsRouted;
		++ordered_ports[offset].nextSwp->numBaseLidsRouted;
	}

	if (portnos[0] == 0xff && smDebugPerf) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Failed to setup LID 0x%.4X for switch[%d : "FMT_U64"] %s",
		       orig_portp->portData->lid, switchp->index, switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

// selects all best ports to the provided switch index
// returns the number of ports found (0 if none)
//
// ** Note: This is one of the hottest functions in the FM. **MODIFY WITH CAUTION**
int
sm_routing_func_select_ports(Topology_t *topop, Node_t *switchp, int endIndex, SwitchportToNextGuid_t *ordered_ports, boolean selectBest)
{
	int      i, j;
	uint16_t cur_speed = 0;
	uint16_t best_speed = 0;
	uint16_t best_lidsRouted = 0xffff;
	uint32_t best_switchLidsRouted = 0xffffffff;
	Node_t   *next_nodep;
	Port_t   *portp;
	SpineFirstState_t sfstate;
	SpineFirstResult_t sfres;
	SmPathPortmask_t ports;
	uint8_t cport;
	uint8_t doSpineCheck;
	uint8_t end_port = 0;

	i = switchp->swIdx;
	j = endIndex;
	ports = topop->path[Index(i, j)];

	sfstate.matching = 0;

	doSpineCheck =  topop->routingModule->funcs.do_spine_check(topop, switchp);

	while ((cport = sm_path_portmask_pop_first(&ports))) {

		portp = sm_get_port(switchp, cport);
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;
		
		next_nodep = sm_find_node(topop, portp->nodeno);
		if (next_nodep == NULL)
			continue;

		if (i == next_nodep->swIdx)
			continue; // avoid loopback links

		if (doSpineCheck) {
			sfres = _spine_first_test(&sfstate, switchp, portp, next_nodep);
			switch (sfres) {
			case SPINE_FIRST_FIRST:
				// spine first is enabled and this is the first.
				// override anything we've previously seen with this
				best_speed = sm_GetSpeed(portp->portData);
				best_lidsRouted = portp->portData->lidsRouted;
				best_switchLidsRouted = next_nodep->numLidsRouted;
				ordered_ports[0].portp = portp;
				ordered_ports[0].guid = next_nodep->nodeInfo.NodeGUID;
				ordered_ports[0].sysGuid = next_nodep->nodeInfo.SystemImageGUID;
				ordered_ports[0].nextSwp = next_nodep;
				end_port = 1;
				continue;
			case SPINE_FIRST_NOMATCH:
				// we're only considering spines, and this isn't one.
				// discard it
				continue;
			case SPINE_FIRST_MATCH:
				// we've seen at least one spine so far... is this one better?
				// fall through to default behavior to determine
			case SPINE_FIRST_NONE:
				// spine first is not enabled or there are no spines off this
				// node so far.  balance normally
				break;
			}
		}

		cur_speed = portp->portData->portSpeed;
		if (cur_speed >= best_speed) {
			if (cur_speed > best_speed) {
				best_speed = cur_speed;
				best_lidsRouted = portp->portData->lidsRouted;
				best_switchLidsRouted = next_nodep->numLidsRouted;
				end_port = 0;
			}
			else if (selectBest) {
				if (portp->portData->lidsRouted < best_lidsRouted) {
					best_lidsRouted = portp->portData->lidsRouted;
					best_switchLidsRouted = next_nodep->numLidsRouted;
					end_port = 0;
				}
				else if (portp->portData->lidsRouted == best_lidsRouted &&
						next_nodep->numLidsRouted < best_switchLidsRouted) {
					best_switchLidsRouted = next_nodep->numLidsRouted;
					end_port = 0;
				}
				else {
					continue;
				}
			}

			ordered_ports[end_port].portp = portp;
			ordered_ports[end_port].guid = next_nodep->nodeInfo.NodeGUID;
			ordered_ports[end_port].sysGuid = next_nodep->nodeInfo.SystemImageGUID;
			ordered_ports[end_port].nextSwp = next_nodep;
			++end_port;
		}
	}

	return end_port;
}

Status_t
sm_routing_func_setup_pgs(struct _Topology *topop, struct _Node * srcSw, struct _Node * dstSw)
{
#ifdef __VXWORKS__
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
	memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * srcSw->nodeInfo.NumPorts);
#else
	SwitchportToNextGuid_t ordered_ports[MAX_STL_PORTS] = {{0}};
#endif /* __VXWORKS__ */

	if (!srcSw || !dstSw) {
		IB_LOG_ERROR_FMT(__func__, "Invalid source or destination pointer.");
		return VSTATUS_BAD;
	}

	if (srcSw->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		IB_LOG_ERROR_FMT(__func__, "%s (0x%"PRIx64") is not a switch.",
			srcSw->nodeDesc.NodeString,
			srcSw->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}

	// Optimization. Don't waste time if AR is turned off, if
	// the destination isn't a switch or if source == dest.
	if (dstSw->nodeInfo.NodeType != NI_TYPE_SWITCH ||
        srcSw->swIdx == dstSw->swIdx ||
		!sm_adaptiveRouting.enable || !srcSw->arSupport) {
		return VSTATUS_OK;
	}

	// If port0 isn't valid, we can't finish the calculations.
	if (!sm_valid_port(&dstSw->port[0])) {
		IB_LOG_ERROR_FMT(__func__, "%s (0x%"PRIx64") does not have valid port0 data.",
			srcSw->nodeDesc.NodeString,
			srcSw->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}

	int end_port =  topop->routingModule->funcs.select_ports(topop, srcSw, dstSw->swIdx, ordered_ports, 0);
	if (end_port <= 1) {
		return VSTATUS_OK;
	}

	STL_PORTMASK pgMask = 0;
	int i;
	for (i = 0; i < end_port; ++i) {
		if (ordered_ports[i].portp->index == 0 ||
			ordered_ports[i].portp->index > sizeof(pgMask)*8) {
			continue;
		}

		// Cast is necessary to prevent compiler from interpreting '1' as a signed
		// int32, converting it to an int64, then or'ing
		pgMask |= (((uint64)1) << (ordered_ports[i].portp->index - 1));
	}

	uint8_t pgid;

	// This just adds PGs to the PGT until all entries
	// are exhausted; it doesn't do anything to ensure that the PGs added are optimal or better than others
	int rc = sm_Push_Port_Group(srcSw->pgt, pgMask, &pgid, &srcSw->pgtLen, srcSw->switchInfo.PortGroupCap);

	if (rc >= 0) {
		srcSw->arChange |= (rc > 0);
		srcSw->switchInfo.PortGroupTop = srcSw->pgtLen;

		//PGFT is independent of LFT with LMC, though it's supposed to re-use the LMC data
		PORT * pgft = sm_Node_get_pgft_wr(srcSw);
		uint32_t pgftLen = sm_Node_get_pgft_size(srcSw);

		if (!pgft) {
			IB_LOG_ERROR_FMT(__func__, "Failed to acquire memory for PGFT");
			return VSTATUS_BAD;
		}

		// Add every lid of dstSw to srSw's pgft.
		// (assuming the lid is < the pgftLen)
		STL_LID portLid = 0;
		for_all_port_lids(&dstSw->port[0], portLid) {
			if (portLid < pgftLen) {
				srcSw->arChange |= (pgft[portLid] != pgid);
				pgft[portLid] = pgid;
			}
		}

		// iterate through the end nodes attached to dstSw,
		// adding their LIDs to the pgft.
		// (assuming the lid is < the pgftLen)
		Port_t * edgePort = NULL;
		for_all_physical_ports(dstSw, edgePort) {
			if (!sm_valid_port(edgePort) || edgePort->state <= IB_PORT_DOWN)
				continue;
			Node_t * endNode = NULL;
			Port_t * endPort = sm_find_neighbor_node_and_port(topop, edgePort, &endNode);

			if (!endNode || endNode->nodeInfo.NodeType != NI_TYPE_CA)
				continue;
			if (!endPort || !sm_valid_port(endPort))
				continue;

			for_all_port_lids(endPort, portLid) {
				if (portLid < pgftLen) {
				srcSw->arChange |= (pgft[portLid] != pgid);
				pgft[portLid] = pgid;
			}
		}
	}
	}

	return VSTATUS_OK;
}

int
sm_routing_func_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *nodep, uint8_t *portnos)
{
	int i, j;
	int end_port = 0;
#ifdef __VXWORKS__
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
	memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * switchp->nodeInfo.NumPorts);
#else
	SwitchportToNextGuid_t ordered_ports[MAX_STL_PORTS] = {{0}};
#endif /* __VXWORKS__ */

	IB_ENTER(__func__, switchp, nodep, 0, 0);
	
	memset((void*)portnos, 0xff, sizeof(uint8_t)*128);
	
	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		IB_EXIT(__func__, VSTATUS_OK);
		return 0;
	}

	i = switchp->swIdx;
	j = nodep->swIdx;

	if (j == i) {
		IB_EXIT(__func__, VSTATUS_OK);
		return 0;
	}


	end_port =  topop->routingModule->funcs.select_ports(topop, switchp, j, ordered_ports, 0);

	qsort(ordered_ports, end_port, sizeof(SwitchportToNextGuid_t), _compare_guids);

	for (i=0; i<end_port; i++) {
		portnos[i] = ordered_ports[i].portp->index;
	}

	if (portnos[0] == 0xff && smDebugPerf) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Failed to get portGroup from switch %s to switch %s",
		       sm_nodeDescString(switchp), sm_nodeDescString(nodep));
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return end_port;
}

Status_t
sm_routing_func_select_slsc_map(Topology_t *topop, Node_t *nodep,
    Port_t *in_portp, Port_t *out_portp, STL_SLSCMAP *outSlscMap)
{
	return sm_select_slsc_map(topop, nodep, in_portp, out_portp, outSlscMap);
}

Status_t
sm_routing_func_select_scsl_map(Topology_t *topop, Node_t *nodep,
    Port_t *in_portp, Port_t *out_portp, STL_SCSLMAP *outScslMap)
{
	return sm_select_scsl_map(topop, nodep, in_portp, out_portp, outScslMap);
}

Status_t
sm_routing_func_select_scsc_map(Topology_t *topop, Node_t *switchp, int getSecondary, int *numBlocks, STL_SCSC_MULTISET** scscmap)
{
	int i;
	Port_t * portp = NULL;
	STL_SCSC_MULTISET *scsc=NULL;
	int portToSet = 0;
	int needsSet = 0;
	STL_SCSCMAP *scscTmp= NULL;

	*numBlocks = 0;

	if ((topology_passcount && !topology_switch_port_changes && !sm_config.forceAttributeRewrite) || getSecondary)
		return VSTATUS_OK;

	if (vs_pool_alloc(&sm_pool, sizeof(STL_SCSC_MULTISET), (void *) &scsc) != VSTATUS_OK) {
		return VSTATUS_BAD;
	}

	memset(scsc, 0, sizeof(STL_SCSC_MULTISET));

	// TBD: Fattree is simple 1:1 mapping.
	// leaving in as test vehicle but may remove later.
	for (i=0; i<STL_MAX_SCS; i++) {
		scsc->SCSCMap.SCSCMap[i].SC = i;
	}

	for_all_physical_ports(switchp, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;

		needsSet = !portp->portData->current.scsc ||  sm_config.forceAttributeRewrite;
		if (!needsSet) {
			for (i=1; i<=switchp->nodeInfo.NumPorts; i++) {
				scscTmp = sm_lookupPortDataSCSCMap(portp, i-1, 0);
				if (!scscTmp || (memcmp((void *)&scsc->SCSCMap, (void *)scscTmp, sizeof(STL_SCSCMAP)) != 0)) {
					needsSet = 1;
					break;
				}
			}
		}

		if (needsSet) {
			StlAddPortToPortMask(scsc->IngressPortMask, portp->index);
			portToSet = 1;
		}
	}
	
	if (portToSet) {
		// Set entire table for any ingress port needing a set
		for (i=1; i<=switchp->nodeInfo.NumPorts; i++)
			StlAddPortToPortMask(scsc->EgressPortMask, i);

		*numBlocks = 1;
		*scscmap = scsc;

	} else {
		(void) vs_pool_free(&sm_pool, scsc);
	}

	return VSTATUS_OK;
}

Status_t
sm_routing_func_select_scvl_map_fixedmap(Topology_t *topop, Node_t *nodep,
    Port_t *in_portp, Port_t *out_portp, STL_SCVLMAP *outScvlMap)
{
    Qos_t *qos = sm_get_qos(out_portp->portData->vl1);

    memcpy(outScvlMap, &qos->scvl, sizeof(STL_SCVLMAP));
    return VSTATUS_OK;
}

Status_t
sm_routing_func_select_vlvf_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlVfMap_t * vlvfmap)
{
    return sm_select_vlvf_map(topop, nodep, portp, vlvfmap);
}

Status_t
sm_routing_func_select_vlbw_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlBwMap_t * vlbwmap)
{
    Qos_t *qos = sm_get_qos(portp->portData->vl1);

    memcpy(vlbwmap, &qos->vlBandwidth, sizeof(VlBwMap_t));
    return VSTATUS_OK;
}

Status_t
sm_routing_func_select_scvlr_map(Topology_t *topop, uint8_t vlCap, STL_SCVLMAP *outScvlMap)
{
    return sm_select_scvlr_map(topop, vlCap, outScvlMap);
}

Status_t
sm_routing_func_fill_stl_vlarb_table(Topology_t *topop, Node_t *nodep, Port_t *portp, PortDataVLArb* arbp)
{
    return sm_fill_stl_vlarb_table(topop, nodep, portp, arbp);
}

Status_t
sm_routing_func_select_path_lids(Topology_t *topop, Port_t *srcPortp, STL_LID slid, Port_t *dstPortp,
	STL_LID dlid, STL_LID *outSrcLids, uint8_t *outSrcLen, STL_LID *outDstLids, uint8_t *outDstLen)
{
	return sm_select_path_lids(topop, srcPortp, slid, dstPortp, dlid, outSrcLids, outSrcLen, outDstLids, outDstLen);
}

Status_t
sm_routing_func_process_swIdx_change_noop(Topology_t * topop, int old_idx, int new_idx, int last_idx)
{
	return VSTATUS_OK;
}


int
sm_routing_func_check_switch_path_change(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp)
{
	/* PR 115770. If there is any switch cost change, we recompute full LFT for all switches.
	 * Just checking cost/path of this switch to other switches is not fully reliable.
	 * This is because there may be multiple paths between switches with same cost
	 * and routes are distributed over these paths. The cost/path matrix stores only one best
	 * cost and path - so if one of the paths is not possible, just checking cost/path
	 * matrix may not tell us that there may be changes somewhere along one of the paths.
	 *
	 */
	if (topology_cost_path_changes)
		return 1;

	return 0;
}

boolean
sm_routing_func_needs_routing_recalc_false(Topology_t * topop, Node_t * nodep)
{
	return 0;
}

boolean
sm_routing_func_needs_routing_recalc_true(Topology_t * topop, Node_t * nodep)
{
	return 1;
}

boolean
sm_routing_func_needs_lft_recalc(Topology_t * topop, Node_t * nodep)
{
	return !routing_recalculated && !nodep->routingRecalculated;
}

boolean
sm_routing_func_can_send_partial_routes_true(void)
{
	return 1;
}

boolean
sm_routing_func_can_send_partial_routes_false(void)
{
	return 0;
}

boolean
sm_routing_func_do_spine_check(Topology_t * topop, Node_t * switchp)
{
	return sm_config.spine_first_routing && switchp->internalLinks;
}

Status_t
sm_routing_func_write_minimal_lft_blocks(Topology_t * topop, Node_t * switchp, SmpAddr_t * addr)
{
	return sm_write_minimal_lft_blocks(topop, switchp, addr);
}


Status_t
sm_routing_func_write_full_lfts_LR(Topology_t * topop, SwitchList_t * swlist, int rebalance)
{
	return sm_write_full_lfts_by_block_LR(topop, swlist, rebalance);
}


Status_t
sm_routing_func_route_old_switch(Topology_t *src_topop, Topology_t *dst_topop, Node_t *nodep)
{
	return sm_routing_route_old_switch(src_topop, dst_topop, nodep);
}

boolean
sm_routing_func_handle_fabric_change(Topology_t *topop, Node_t *oldSwitchp, Node_t *switchp)
{
	Port_t*	portp = NULL;
	int port;

	for (port= bitset_find_first_one(&switchp->initPorts); port >=0;
		 port= bitset_find_next_one(&switchp->initPorts, port+1)) {

		portp = sm_get_port(switchp, port);
		if (!sm_valid_port(portp)) continue;

		if (portp->portData->isIsl)
			// New ISL coming up
			return 1;

		if (bitset_test(&oldSwitchp->activePorts, port)) {
			// If it was active, check to see if it was an ISL
			portp = sm_get_port(oldSwitchp, port);
			if (!sm_valid_port(portp) || portp->portData->isIsl)
				// Moved from switch port to HFI
				return 1;
		}
	}

	if (!bitset_equal(&oldSwitchp->activePorts, &switchp->activePorts)) {
		for (port= bitset_find_first_one(&oldSwitchp->activePorts); port >=0;
			 port= bitset_find_next_one(&oldSwitchp->activePorts, port+1)) {
			if (bitset_test(&switchp->activePorts, port)) continue;

			portp = sm_get_port(oldSwitchp, port);
			if (!sm_valid_port(portp)) continue;

			if (portp->portData->isIsl)
				// Lost an ISL
				return 1;
		}
	}

	return 0;
}

Status_t
sm_routing_func_update_bw(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics)
{
	return sm_update_bw(rm, VirtualFabrics);
}

Status_t
sm_routing_func_assign_scs_to_sls_fixedmap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics)
{
	return sm_assign_scs_to_sls_FixedMap(rm, VirtualFabrics);
}

Status_t
sm_routing_func_assign_sls(RoutingModule_t *rm, VirtualFabrics_t *vfs)
{
    int noqos_base = -1;
    int noqos_resp = -1;
    int noqos_mcast = -1;
    int qos;
    Status_t ret = VSTATUS_OK;
    bitset_t usedSLs;
	int numSCs = 0;

    if (!vfs)  {
        return ret;
    }

    bitset_init(&sm_pool, &usedSLs, STL_MAX_SLS);

	if (VSTATUS_OK != (ret = _handle_preassigned_sl(rm, vfs, &usedSLs, &numSCs))) {
		goto bail;
	}

    // Now assign the unspecified SLs.
    // Assign SLs to QOSGroups in the order they appear in the config file.
    // [This provides a predicable output for users.]
    for (qos=0; qos < vfs->number_of_qos_all; qos++) {
        QosConfig_t *pQos = &vfs->qos_all[qos];

		if (VSTATUS_OK != (ret = _handle_unassigned_sl(pQos, &pQos->base_sl, &usedSLs, &noqos_base,
			0, MAX_SLS, "BaseSL", &numSCs, rm->funcs.num_routing_scs(pQos->resp_sl, 0))))
			goto bail;
		if (pQos->requires_resp_sl) {
			if (VSTATUS_OK != (ret = _handle_unassigned_sl(pQos, &pQos->resp_sl, &usedSLs, &noqos_resp,
				1, STL_MAX_SLS, "RespSL", &numSCs, rm->funcs.num_routing_scs(pQos->resp_sl, 0))))
				goto bail;
		} else
		if (pQos->resp_sl == UNDEFINED_XML8) {
			pQos->resp_sl = pQos->base_sl;
        	if (sm_config.sm_debug_vf)
            	IB_LOG_INFINI_INFO_FMT_VF(pQos->name, "",
            	"Assigning RespSL to BaseSL %d", pQos->resp_sl);
		}
		if (pQos->contains_mcast && rm->funcs.mcast_isolation_required()) {
			if (VSTATUS_OK != (ret = _handle_unassigned_sl(pQos, &pQos->mcast_sl, &usedSLs,
				&noqos_mcast, 0, MAX_SLS, "MulticastSL", &numSCs,
				rm->funcs.num_routing_scs(pQos->mcast_sl, 1))))
				goto bail;
			numSCs += rm->funcs.num_routing_scs(pQos->mcast_sl, 1);
		} else if (pQos->mcast_sl == UNDEFINED_XML8) {
			pQos->mcast_sl = pQos->base_sl;
        	if (sm_config.sm_debug_vf)
            	IB_LOG_INFINI_INFO_FMT_VF(pQos->name, "",
            	"Assigning multicast SL to base SL %d", pQos->mcast_sl);
		}
    }

    IB_LOG_INFINI_INFO_FMT(__func__, "%d QOSGroups require %d SLs and %d SCs for operation",
                           vfs->number_of_qos_all, (int)bitset_nset(&usedSLs), numSCs);

bail:
    bitset_free(&usedSLs);
    return ret;
}

boolean
sm_routing_func_mcast_isolation_not_required(void)
{
	return 0;
}

boolean
sm_routing_func_mcast_isolation_is_required(void)
{
	return 1;
}

int
sm_routing_func_min_vls(void)
{
	return sm_config.min_supported_vls;
}

int
sm_routing_func_max_vls(void)
{
	return SCVLMAP_BASE;
}

int
sm_routing_func_one_routing_scs(int sl, boolean mcast_sl)
{
	return 1;
}

RoutingFuncs_t defaultRoutingFuncs = {
    pre_process_discovery:		sm_routing_func_pre_process_discovery_noop,
    discover_node:				sm_routing_func_discover_node_noop,
    discover_node_port:			sm_routing_func_discover_node_port_noop,
    post_process_discovery:		sm_routing_func_post_process_discovery_noop,
    post_process_routing:		sm_routing_func_post_process_routing_noop,
    post_process_routing_copy:	sm_routing_func_post_process_routing_copy_noop,
    allocate_cost_matrix:		sm_routing_func_alloc_cost_matrix_floyds,
    initialize_cost_matrix:		sm_routing_func_init_cost_matrix_floyds,
    calculate_cost_matrix:		sm_routing_func_calc_cost_matrix_floyds,
    routing_mode:				sm_routing_func_routing_mode_linear,

    extended_scsc_in_use:		sm_routing_func_false,

    copy_routing:				sm_routing_func_copy_routing_lfts,
    init_switch_routing:		sm_routing_func_init_switch_routing_lfts,
    calculate_routes: 			sm_routing_func_calculate_lft,
    setup_xft:					sm_routing_func_setup_xft,
    select_ports:				sm_routing_func_select_ports,
    setup_pgs:					sm_routing_func_setup_pgs,
    get_port_group:				sm_routing_func_get_port_group,
    select_slsc_map:			sm_routing_func_select_slsc_map,
    select_scsl_map:			sm_routing_func_select_scsl_map,
    select_scsc_map:			sm_routing_func_select_scsc_map,
    select_scvl_map:			sm_routing_func_select_scvl_map_fixedmap,
    select_vlvf_map:			sm_routing_func_select_vlvf_map,
    select_vlbw_map:			sm_routing_func_select_vlbw_map,
    select_scvlr_map:			sm_routing_func_select_scvlr_map,
    fill_stl_vlarb_table:		sm_routing_func_fill_stl_vlarb_table,
    select_path_lids:			sm_routing_func_select_path_lids,
    process_swIdx_change:		sm_routing_func_process_swIdx_change_noop,
    check_switch_path_change:	sm_routing_func_check_switch_path_change,
    needs_routing_recalc: 		sm_routing_func_needs_lft_recalc,
    can_send_partial_routes:	sm_routing_func_can_send_partial_routes_false,
	do_spine_check:				sm_routing_func_do_spine_check,
    write_minimal_routes:		sm_routing_func_write_minimal_lft_blocks,
    write_full_routes_LR:		sm_routing_func_write_full_lfts_LR,
    route_old_switch:			sm_routing_func_route_old_switch,
	build_spanning_trees:		sm_build_spanning_trees,
    handle_fabric_change:		sm_routing_func_handle_fabric_change,
	update_bw:					sm_routing_func_update_bw,
	assign_scs_to_sls:			sm_routing_func_assign_scs_to_sls_fixedmap,
	assign_sls:					sm_routing_func_assign_sls,
	mcast_isolation_required:	sm_routing_func_mcast_isolation_not_required,
	min_vls:					sm_routing_func_min_vls,
	max_vls:					sm_routing_func_max_vls,
	num_routing_scs:			sm_routing_func_one_routing_scs
};
