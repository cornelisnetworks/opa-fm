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
#include "sm_dbsync.h"

static __inline__ uint16_t hypercube_GetCost(PortData_t *portData) {
	return (portData->routingCost ? portData->routingCost : 10);
}

static int
hypercube_compare_lids_routed(const void * arg1, const void * arg2)
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


// selects all best ports to the provided switch index
// returns the number of ports found (0 if none)
//
static int
hypercube_select_ports(Topology_t *topop, Node_t *switchp, int endIndex, SwitchportToNextGuid_t *ordered_ports, boolean selectBest)
{
	int      i, j, k;
	uint16_t cur_speed = 0;
	uint16_t best_speed = 0;
	uint16_t best_cost = 0xffff;
	uint16_t best_lidsRouted = 0xffff;
	uint32_t best_switchLidsRouted = 0xffffffff;
	int      end_port = 0;
	int 	 port;
	Node_t   *next_nodep;
	Node_t   *first_nodep = 0;
	Port_t   *portp;
	uint8_t  *portOrder = (uint8_t*)switchp->routingData;

	i = switchp->swIdx;
	j = endIndex;
	best_cost = topop->cost[Index(i, j)];

	for (port = 0; port < switchp->nodeInfo.NumPorts; port++) {
		portp = sm_get_port(switchp, portOrder ? portOrder[port] : port + 1);

		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;
		next_nodep = sm_find_node(topop, portp->nodeno);
		if (next_nodep == NULL)
			continue;

		// cost and path arrays include switches only, so skip ports to non switches
		// and get the switch index in the switch list not the node index from node list
		if (next_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
			continue;
		k = next_nodep->swIdx;
		if (i == k) continue; // avoid loopback links

		if (topop->cost[Index(i, k)] + topop->cost[Index(k, j)] == best_cost) {
			if (!first_nodep) {
				first_nodep = next_nodep;
			} else if (next_nodep != first_nodep) {
				continue;
			}
			cur_speed = sm_GetSpeed(portp->portData);
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
				ordered_ports[end_port].nextSwp = next_nodep;
				++end_port;
			}
		}
	}

	return end_port;
}

static void
_balance_ports(Node_t *switchp, SwitchportToNextGuid_t *ordered_ports, int olen)
{
	if (olen <= 0) return;

	qsort(ordered_ports, olen, sizeof(SwitchportToNextGuid_t), hypercube_compare_lids_routed);
}

// -------------------------------------------------------------------------- //
//
//	This is a common routine used by the LFT and RFT routines to parse
//	out what the path is through the fabric in order to setup the routing
//	tables.
//  See sm_l.h for parameter documentation
static Status_t
hypercube_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *nodep, Port_t *orig_portp, uint8_t *portnos) {
	int i, j;
	uint8_t numLids;
	int lidsRoutedInc;
	int offset=0;
	int end_port = 0;
#ifdef __VXWORKS__
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
	memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * switchp->nodeInfo.NumPorts);
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

		// select best port, hypercube_select_ports will return 1 or 0 (no path)
		if ((end_port = hypercube_select_ports(topop, switchp, j, ordered_ports, 1)) == 0) {
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

		end_port = hypercube_select_ports(topop, switchp, j, ordered_ports, 0);
		if (!end_port) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to find outbound ports on NodeGUID "FMT_U64" to NodeGUID "FMT_U64" Port %d",
				switchp->nodeInfo.NodeGUID, nodep->nodeInfo.NodeGUID, orig_portp->index);
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

static void
_setup_routing_ctrl(Node_t *nodep)
{
	STL_NODE_INFO *nodeInfo = &nodep->nodeInfo;
	SmSPRoutingCtrl_t *routeCtrlData;
	SmSPRoutingCtrl_t *defaultData = NULL;
	SmSPRoutingCtrl_t *nodeData = NULL;
	SmSPRoutingPort_t *ports;
	uint16_t portCount;
	uint16_t defaultCost = 0;
	int NumPorts = nodeInfo->NumPorts;
	Port_t *portp;
	portMap_t pportMap[STL_MAX_PORTS/(sizeof(portMap_t)*8) + 1];
	portMap_t vportMap[STL_MAX_PORTS/(sizeof(portMap_t)*8) + 1];
	int i;
	int pport;
	int vport;
	uint8_t *portOrder;
	Status_t status;
	uint16_t cost;
	int dgIdx;

	if (nodeInfo->NodeType != NI_TYPE_SWITCH) {
		return;
	}

	if (nodep->routingData) {
		return;
	}
	portp = sm_get_port(nodep, 0);

	for (routeCtrlData = sm_config.hypercubeRouting.enhancedRoutingCtrl; routeCtrlData; routeCtrlData = routeCtrlData->next) {
		if (!strlen(routeCtrlData->switches.member)) {
			defaultData = routeCtrlData;
		} else {
			dgIdx = routeCtrlData->switches.dg_index;
			if (dgIdx < 0) {
				IB_LOG_WARN_FMT(__func__, "HypercubeTopology has undefined EnhancedRoutingCtrl Switches group %s",
											routeCtrlData->switches.member);
				continue;
			}
			if (sm_valid_port(portp) &&
				bitset_test(&portp->portData->dgMember, dgIdx)) {
				if (!nodeData) {
					nodeData = routeCtrlData;
				} else {
					IB_LOG_WARN_FMT(__func__,
						 "Switch %s Guid "FMT_U64" using enhanced hypercube routing control group %s, ignoring duplicate membership in switch group %s",
						 sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
						 nodeData->switches.member, nodeData->switches.member);
				}
			}
		}
	}

	routeCtrlData = nodeData ? nodeData : defaultData;
	if (!routeCtrlData) {
		// nothing to do
		return;
	}

	if (nodeData) {
		// look for a default cost within the node's entry
		ports = nodeData->ports;
		portCount = nodeData->portCount;
		for (i = 0; i < portCount; i++, ports++) {
			if (!ports->pport) {
				defaultCost = ports->cost;
				break;
			}
		}
	}
	if (!defaultCost && defaultData) {
		// the node didn't have a default cost, look for a global one
		ports = defaultData->ports;
		portCount = defaultData->portCount;
		for (i = 0; i < portCount; i++, ports++) {
			if (!ports->pport) {
				defaultCost = ports->cost;
				break;
			}
		}
	}

	if (defaultCost) {
		// pre-initialize cost to the default cost if there is one
		for_all_ports(nodep, portp) {
			if (portp->portData) {
				portp->portData->routingCost = defaultCost;
			}
		}
	}

	status = vs_pool_alloc(&sm_pool, NumPorts*sizeof(*portOrder),
						   (void *)&portOrder);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "can't malloc portOrder status=%d",
						 status);
		return;
	}
	nodep->routingData = (void*)portOrder;

	memcpy(pportMap, routeCtrlData->pportMap, sizeof(pportMap));
	memcpy(vportMap, routeCtrlData->vportMap, sizeof(vportMap));

	// process the node's entries
	ports = routeCtrlData->ports;
	portCount = routeCtrlData->portCount;
	for (i = 0; i < portCount; i++, ports++) {
		pport = ports->pport;
		vport = ports->vport;
		cost = ports->cost;
		if (cost) {
			portp = sm_get_port(nodep, pport);
			if (portp && portp->portData) {
				portp->portData->routingCost = cost;
			}
		}
		if (vport && (vport <= NumPorts) && pport && (pport <= NumPorts)) {
			portOrder[vport] = pport;
		}
	}

	// add in port order entries from default if not already there
	if (defaultData && nodeData) {
		ports = defaultData->ports;
		portCount = defaultData->portCount;
		for (i = 0; i < portCount; i++, ports++) {
			pport = ports->pport;
			vport = ports->vport;
			cost = ports->cost;
			if (cost) {
				portp = sm_get_port(nodep, pport);
				if (portp && portp->portData && !portp->portData->routingCost) {
					portp->portData->routingCost = cost;
				}
			}
			if (portMapTest(pportMap, pport) || portMapTest(vportMap, vport)) {
				continue;
			}
			if (vport && (vport <= NumPorts) && pport && (pport <= NumPorts)) {
				portOrder[vport] = pport;
				portMapSet(pportMap, pport);
				portMapSet(vportMap, vport);
			}
		}
	}

	pport = 1;
	for (vport = 1; vport <= NumPorts; vport++) {
		if (!portMapTest(vportMap, vport)) {
			while (pport <= NumPorts) {
				if (!portMapTest(pportMap, pport)) {
					portOrder[vport] = pport++;
					break;
				}
				pport++;
			}
		}
	}

	if (sm_config.hypercubeRouting.debug) {
		for_all_physical_ports(nodep, portp) {
			if (!sm_valid_port(portp)) continue;
			IB_LOG_INFINI_INFO_FMT(__func__,"Switch %s [Guid: "FMT_U64"] Port %d Cost %d", 
				 sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, portp->portData->routingCost);
		}
	}

	return;
}

/*
 * Traverse the list of nodes in the topology and assign weights to their ports.
 */
static Status_t
hypercube_post_process_discovery(Topology_t *topop, Status_t discoveryStatus, void *context)
{
	Node_t* nodep;
	Port_t* portp;
	int dgIdx;

	for_all_switch_nodes(sm_topop, nodep) {
		_setup_routing_ctrl(nodep);
	}

	if (strlen(sm_config.hypercubeRouting.routeLast.member) == 0)
		return VSTATUS_OK;

	// check for ca route last
	dgIdx = sm_config.hypercubeRouting.routeLast.dg_index;
	if (dgIdx == -1) {
		IB_LOG_WARN_FMT(__func__, "HypercubeTopology has undefined RouteLast Switches group %s",
									sm_config.hypercubeRouting.routeLast.member);
		return VSTATUS_OK;
	}

	for_all_ca_nodes(topop, nodep) {
		for_all_physical_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
				continue;

			// is it a member of the route last device group
			if (bitset_test(&portp->portData->dgMember, dgIdx)) {
				nodep->skipBalance = 1;
				return VSTATUS_OK;
			}
		}
	}
	return VSTATUS_OK;
}

Status_t
hypercube_routing_init_floyds(Topology_t *topop)
{
	int i, j, k, ij, ik, iNumNodes;
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
				topop->cost[ik] = hypercube_GetCost(portp->portData);
				sm_path_portmask_set(topop->path + ik, portp->index);
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
hypercube_routing_calc_floyds(Topology_t *topop, int switches, unsigned short * cost, SmPathPortmask_t * path)
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
		for (i = 0, iNumNodes = 0; i < switches; i++, iNumNodes += switches) {

			ik = iNumNodes + k;

			if (cost[ik] == Cost_Infinity) {
				continue;
			}

			for (j = 0, ij = iNumNodes, kj = kNumNodes;
				 j < switches;
				 ++j, ++ij, ++kj) {

				if ((value = cost[ik] + cost[kj]) < cost[ij]) {
					cost[ij] = value;
					path[ij] = path[ik];
					
				} else if (value == cost[ij]) {
					sm_path_portmask_merge(path + ij, path + ik);
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

static int
hypercube_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *nodep, uint8_t *portnos) {
	int i, j;
	int end_port = 0;
#ifdef __VXWORKS__
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
	memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * switchp->nodeInfo.NumPorts);
#else
	SwitchportToNextGuid_t ordered_ports[MAX_STL_PORTS] = {{0}};
#endif /* __VXWORKS__ */


	IB_ENTER(__func__, switchp, nodep, 0, 0);

	memset((void*)portnos, 0xff, sizeof(uint8_t)*256);

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

	end_port = topop->routingModule->funcs.select_ports(topop, switchp, j, ordered_ports, 0);

	qsort(ordered_ports, end_port, sizeof(SwitchportToNextGuid_t), hypercube_compare_lids_routed);

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

static Status_t
hypercube_calculate_all_lfts(Topology_t * topop)
{
	Node_t *switchp, *toSwitchp, *nodep;
	Port_t *portp, *toSwitchPortp;
	Status_t status = VSTATUS_OK;
	int i, j, currentLid, numPorts;
	uint8_t portGroup[256];
	int routingIterations, r;

	for_all_switch_nodes(topop, switchp) {
		status = sm_Node_init_lft(switchp, NULL);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
			return status;
		}

		// Initialize port group top prior to setting up groups.
		switchp->switchInfo.PortGroupTop = 0;
	}

	// if route last is indicated, balance over those HFIs on a second pass
	routingIterations = (strlen(sm_config.hypercubeRouting.routeLast.member) == 0) ? 1 : 2;

	for (r=0; r<routingIterations; ++r) {
		for_all_switch_nodes(topop, switchp) {
			i = 0;
			for_all_switch_nodes(topop, toSwitchp) {
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
				}

				for_all_physical_ports(toSwitchp, toSwitchPortp) {
			   		if (!sm_valid_port(toSwitchPortp) || toSwitchPortp->state <= IB_PORT_DOWN) continue;

					if (toSwitchPortp->portData->isIsl) continue;

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

					if ((r==1) && !nodep->skipBalance) {
						// already programmed
						continue;
					}

					j = 0;
					for_all_end_ports(nodep, portp) {
						if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
						for_all_port_lids(portp, currentLid) {
							// Handle the case where switchp == toSwitchp. 
							// In this case, the target LID(s) are directly
							// connected to the local switchp port.
							if (!numPorts) {
								switchp->lft[currentLid] = toSwitchPortp->index;
								continue;
							}

							switchp->lft[currentLid] = portGroup[(i+j)%numPorts];
							j++;

							incr_lids_routed(topop, switchp, switchp->lft[currentLid]);
						}
						if (j) i++;
					}
				}
			}
		}
	}

	// Switch-to-Switch routes are very lightly used (management traffic
	// only) so we don't bother trying to balance them.
	for_all_switch_nodes(topop, switchp) {
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
	}
	return status;
}

static Status_t
hypercube_init_switch_lfts(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s = VSTATUS_OK;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (topology_cost_path_changes || *rebalance) {
		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		// If not, copy and delta updates handled by main topology method.
		s = hypercube_calculate_all_lfts(topop);
		*rebalance = 1;
		routing_recalculated = 1;
	}

	return s;
}

static Status_t
hypercube_calculate_lft(Topology_t * topop, Node_t * switchp)
{
	Node_t *toSwitchp, *nodep;
	Port_t *portp, *toSwitchPortp;
	Status_t status = VSTATUS_OK;
	int i, j,currentLid, numPorts;
	uint8_t portGroup[256];
	int routingIterations, r;

	if (sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__, "switch %s", switchp->nodeDesc.NodeString);

	status = sm_Node_init_lft(switchp, NULL);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
		return status;
	}

	// Initialize port group top prior to setting up groups.
	switchp->switchInfo.PortGroupTop = 0;

	// if route last is indicated, balance over those HFIs on a second pass
	routingIterations = (strlen(sm_config.hypercubeRouting.routeLast.member) == 0) ? 1 : 2;

	for (r=0; r<routingIterations; ++r) {
		i = 0;
		for_all_switch_nodes(topop, toSwitchp) {
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
			}

			for_all_physical_ports(toSwitchp, toSwitchPortp) {
		   		if (!sm_valid_port(toSwitchPortp) || toSwitchPortp->state <= IB_PORT_DOWN) continue;

				if (toSwitchPortp->portData->isIsl) continue;

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

				if ((r==1) && !nodep->skipBalance) {
					// already programmed
					continue;
				}

				for_all_end_ports(nodep, portp) {
					if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;

					j = 0;
					for_all_port_lids(portp, currentLid) {
						// Handle the case where switchp == toSwitchp. 
						// In this case, the target LID(s) are directly
						// connected to the local switchp port.
						if (!numPorts) {
							switchp->lft[currentLid] = toSwitchPortp->index;
							continue;
						}

						switchp->lft[currentLid] = portGroup[(i+j)%numPorts];
						j++;

						incr_lids_routed(topop, switchp, switchp->lft[currentLid]);
					}
					if (j) i++;
				}
			}
		}
	}
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

	switchp->routingRecalculated = 1;

	return status;
}

Status_t
sm_hypercube_make_routing_module(RoutingModule_t * rm)
{
	rm->name = "hypercube";
	rm->funcs.post_process_discovery = hypercube_post_process_discovery;
	rm->funcs.initialize_cost_matrix = hypercube_routing_init_floyds;
	rm->funcs.calculate_cost_matrix = hypercube_routing_calc_floyds;
	rm->funcs.setup_xft = hypercube_setup_xft;
	rm->funcs.select_ports = hypercube_select_ports;
	rm->funcs.calculate_routes = hypercube_calculate_lft;
	rm->funcs.init_switch_routing = hypercube_init_switch_lfts;
	rm->funcs.get_port_group = hypercube_get_port_group;

	return VSTATUS_OK;
}

Status_t
sm_hypercube_init(void)
{
	Status_t s;
	s = sm_routing_addModuleFac("hypercube", sm_hypercube_make_routing_module);
	return s;
}
