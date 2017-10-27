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
/*
 * Device Group/Min Hop Routing (dgmh) Functions
 *
 * DGMH is a minhop/shortestpath routing algorithm that inherits much of its 
 * functionality from sm_routing_funcs.c. It deviates from that algorithm by 
 * allowing the administrator to specify the order in which LIDs are assigned
 * to switch egress ports. Depending on the layout of a fabric this can 
 * improve the static balancing of the overall traffic load of the fabric.
 */
#include "ib_types.h"
#include "sm_l.h"
#include "fm_xml.h"

/*
 * Allows ports to be ordered by their weight/priority. (Technically, we only 
 * care about ports with LIDs, not all ports.)
 *
 * Ports with lower weights are routed before those with higher weights.
 */
typedef struct _DGItem {
	cl_map_item_t	mapItem;
	uint64_t		guid;		
	uint8_t			weight;		// useful for debugging.
	Port_t			*port;
} DGItem;

/*
 * Create a new DGTopology structure and associate it with the provided
 * topology structure. outContext is not used.
 */
static Status_t
dgmh_pre_process_discovery(Topology_t *topop, void **outContext)
{
	Status_t status; 
	DGTopology *dgp;
	int i, allFound, dgIndex;

	IB_LOG_INFO_FMT(__func__, "Initializing DGShortestPath.");

	if (sm_config.dgRouting.dgCount == 0) {
		IB_LOG_ERROR_FMT(__func__,"DG Routing selected but no Routing Order specified.");
		return VSTATUS_BAD;
	}

	// NOTA BENE: vs_pool_alloc() zeroes out the allocated memory.
	// We will be assuming that that is the case...
	status = vs_pool_alloc(&sm_pool, sizeof(DGTopology), (void**)&dgp);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate DG data structure; rc:",
			status);
		return status;
	} else {
		IB_LOG_DEBUG4_FMT(__func__, "dgp = %p", dgp);
	}
	topop->routingModule->data = dgp;

	// We read the config on every sweep out of an excess of caution - 
	// in the future we may have to deal with device groups changing
	// from sweep to sweep.
	dgp->dgCount = sm_config.dgRouting.dgCount;
	allFound = 0;
	for (i=0; i<dgp->dgCount; i++) { 
		cl_qmap_init(&(dgp->deviceGroup[i]), NULL);
		strncpy(dgp->deviceGroupName[i],sm_config.dgRouting.dg[i].member,
			MAX_VFABRIC_NAME);
		dgp->deviceGroupName[i][MAX_VFABRIC_NAME]=0;
		if ((dgIndex = smGetDgIdx(dgp->deviceGroupName[i])) < 0) {
			IB_LOG_ERROR_FMT(__func__,"DGMinHopTopology has undefined RoutingOrder DeviceGroup %s",
							dgp->deviceGroupName[i]);
			IB_FATAL_ERROR_NODUMP("FM cannot continue.");
			return VSTATUS_BAD;
		}
		dgp->deviceGroupIndex[i] = dgIndex;
		if (dg_config.dg[dgp->deviceGroupIndex[i]]->select_all && !allFound) {
			// The first group we find that has select_all set is our
			// "All" group and is handled specially.
			dgp->allGroup=i;
			allFound=1;
		}

		IB_LOG_DEBUG4_FMT(__func__,"Group[%d] = %s, Index = %d", 
			i, dgp->deviceGroupName[i], dgp->deviceGroupIndex[i]);
	}

	//If none of the ordered groups has the "all" flag set, we create
	//an "all" group of our own at the end of the list.
	if (!allFound) {
		dgp->allGroup = dgp->dgCount; 
		cl_qmap_init(&(dgp->deviceGroup[dgp->allGroup]), NULL);
	}
	
	IB_LOG_DEBUG4_FMT(__func__,"All Group = %d",dgp->allGroup);

	// This context is only valid inside topology_discovery(), 
	// that's not helpful for dg routing.
	*outContext = NULL;
	return VSTATUS_OK;
}

/*
 * This function is called when fabric discovery has completed.
 *
 * Here we traverse the list of nodes in the topology and assign weights to their 
 * ports.
 */
Status_t
dgmh_post_process_discovery(Topology_t *topop, Status_t discoveryStatus, void *context)
{
	Status_t status; 
	cl_qmap_t	*nodeMap = topop->nodeMap;
	DGTopology 	*dgp = topop->routingModule->data;
	cl_map_item_t *qp, *tqp;
	uint32_t 	i, start, end;
	
	IB_LOG_INFO_FMT(__func__, "Initializing DGShortestPath.");

	// Examine every node in the fabric.
	for (qp = cl_qmap_head(nodeMap); qp != cl_qmap_end(nodeMap);
		qp = cl_qmap_next(qp)) {
		Node_t *nodep = (Node_t*)PARENT_STRUCT(qp, Node_t, mapObj);
		DGItem *dgip;

		if (nodep == NULL || nodep->port == NULL) {
			IB_LOG_ERROR_FMT(__func__, "Invalid node map.");
			return VSTATUS_BAD;
		} else {
			IB_LOG_DEBUG4_FMT(__func__,"Processing %s (0x%"PRIx64")",
				nodep->nodeDesc.NodeString,
				nodep->nodeInfo.NodeGUID);
		}

		// NOTA BENE: This loop is a bit funky because for switches
		// we only want to consider port 0, but for HFIs we want to
		// consider every port **except** port 0 (HFIs don't have a port 0).
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			start = end = 0;
		} else {
			start = 1; 
			end = nodep->nodeInfo.NumPorts;
		}

		for (i=start; i<=end; i++) {
			Port_t 		*portp = &(nodep->port[i]);
			PortData_t 	*pdp = portp->portData;
			uint32_t 	j;

			// Don't mess with downed ports.
			if (pdp == NULL || portp->state < IB_PORT_INIT) {
				IB_LOG_DEBUG4_FMT(__func__,"Skipping %s (0x%"PRIx64"): pdp = %p",
					nodep->nodeDesc.NodeString, nodep->nodeInfo.NodeGUID,
					pdp);
				continue;
			}

			status = vs_pool_alloc(&sm_pool, sizeof(DGItem), (void**)&dgip);
			if (status != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate DG item; rc:", status);
				return status;
			}

			dgip->guid = pdp->guid;
			dgip->port = portp;

			/* This is a little tricky.
			 *
			 * We put the port in the highest priority group that it belongs
			 * to that isn't the allGroup. But, if after checking all other 
			 * groups, we haven't placed the port, then we add it to the allGroup. 
			 * Note that this means the allGroup doesn't have to be the lowest
			 * priority! That means, for example, we can define a fabric where
			 * the default group is routed first with only a few special nodes
			 * routed later. (This effectively replicates the "skip balance" 
			 * feature of the fattree routing algorithm.
			 */
			for (j=0; j<dgp->dgCount; j++) {
				if (j == dgp->allGroup) 
					continue;
				if (bitset_test(&(pdp->dgMember), dgp->deviceGroupIndex[j])) 
					break;
			}

			// If we didn't find a group for this port, put it in the
			// All group.
			if (j>=dgp->dgCount) 
				j = dgp->allGroup;

			dgip->weight=j;

			// Insert the port into the appropriate device group.
			tqp = cl_qmap_insert(&(dgp->deviceGroup[j]),dgip->guid,
				(cl_map_item_t*)dgip);
			if (tqp != (cl_map_item_t*)dgip) {
				IB_LOG_ERROR_FMT(__func__,
					"Failed to insert LID %u, Port GUID = 0x%"PRIx64
					" to %s", pdp->lid, dgip->guid, 
					dgp->deviceGroupName[dgip->weight]);
				return VSTATUS_BAD;
			} else {
				IB_LOG_DEBUG4_FMT(__func__,"LID %u, Port GUID = 0x%"PRIx64
					" added to \"%s\"", pdp->lid, dgip->guid, 
					dgp->deviceGroupName[dgip->weight]);
			}
		}
	}

	for (i=0; i<dgp->dgCount; i++) {
		IB_LOG_INFO_FMT(__func__, "Assigned %"PRISZT" LIDs to Group \"%s\"",
			cl_qmap_count(&(dgp->deviceGroup[i])), dgp->deviceGroupName[i]);
	}

	if (dgp->allGroup >= dgp->dgCount) {
		IB_LOG_INFO_FMT(__func__, "Assigned %"PRISZT" LIDs to Default Group",
			cl_qmap_count(&(dgp->deviceGroup[dgp->allGroup])));
	}
	
	return VSTATUS_OK;
}

/*
 * Destroy any DG data associated with an old topology.
 */
static Status_t
dgmh_destroy(RoutingModule_t *rm)
{
	DGTopology *dgp = (DGTopology*)rm->data;
	int i;

	IB_LOG_DEBUG4_FMT(__func__, "dgp = %p", dgp);

	if (dgp) {
		for (i=0; i<dgp->dgCount; i++) {
			cl_map_item_t *p, *pp;

			IB_LOG_DEBUG4_FMT(__func__, "Freeing %s ports.",
				dgp->deviceGroupName[i]);

			for (p = cl_qmap_head(&(dgp->deviceGroup[i]));
				p != cl_qmap_end(&(dgp->deviceGroup[i]));
				p = pp )
			{
				pp = cl_qmap_next(p);
	
				cl_qmap_remove_item(&(dgp->deviceGroup[i]), p);
				vs_pool_free(&sm_pool,p);		
			}
		}

		IB_LOG_DEBUG4_FMT(__func__, "Freeing DGTopology structure.");
	
		vs_pool_free(&sm_pool,dgp);
	}

	rm->data = NULL;

	return VSTATUS_OK;
}

/*
 * The core function of dgrouting. The default version builds an LFT
 * by traversing the list of nodes in the order they were discovered.
 *
 * By contrast, the dgmh version breaks that list up into sub groups,
 * ordered by their weight/priority but otherwise uses the same mechanisms
 * to decide which ports are valid candidates to use.
 */
static Status_t
dgmh_calculate_lft(Topology_t * topop, Node_t * switchp)
{
	DGTopology 	*dgp = topop->routingModule->data;
	cl_map_item_t *qp;
	int i, end;

	Status_t status = VSTATUS_OK;
	
	uint16_t portLid;
	uint8_t xftPorts[256];

	if (sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__, "switch %s", switchp->nodeDesc.NodeString);

	status = sm_Node_init_lft(switchp, NULL);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
		return status;
	}

	// Initialize port group top prior to setting up groups.
	switchp->switchInfo.PortGroupTop = 0;

	IB_LOG_DEBUG4_FMT(__func__, "Building LFT for 0x%"PRIx64, switchp->nodeInfo.NodeGUID);

	end = (dgp->allGroup<dgp->dgCount)?dgp->dgCount:dgp->allGroup+1;

	for (i=0; (i<end) && (status == VSTATUS_OK); i++) {
		IB_LOG_DEBUG4_FMT(__func__,"Traversing group %s", dgp->deviceGroupName[i]);
		for (qp = cl_qmap_head(&(dgp->deviceGroup[i])); 
			(status == VSTATUS_OK) && (qp != cl_qmap_end(&(dgp->deviceGroup[i])));
			qp = cl_qmap_next(qp)) {
			PortData_t 	*pdp;
			Node_t 		*nodep;
			Port_t 		*portp;
			DGItem 		*dgip;

			dgip = (DGItem*)PARENT_STRUCT(qp, DGItem, mapItem);
			portp = dgip->port;
			if (!portp) {
				IB_LOG_ERROR_FMT(__func__, "Unable to map DG entry to device port structure.");
				status = VSTATUS_BAD;
				break;
			}
			pdp = portp->portData;
			if (!pdp) {
				IB_LOG_ERROR_FMT(__func__, "Unable to map DG entry to device port data.");
				status = VSTATUS_BAD;
				break;
			}

			nodep = pdp->nodePtr;

			// skip dead ports.
			if ((nodep == NULL) ||  (!sm_valid_port(portp)) ||
				(portp->state == IB_PORT_DOWN)) {
				IB_LOG_DEBUG4_FMT(__func__, "SKIPPING lid = %u port = %u\n", 
					pdp->lid, portp->index);
				continue;
			}
			
			status = topop->routingModule->funcs.setup_xft(topop, switchp, 
				nodep, portp, xftPorts);
			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__, "Failed to build XFT for destination %s 0x%"
					PRIx64", LID=%u.", nodep->nodeDesc.NodeString, 
					dgip->guid, pdp->lid);
				break;
			}

			if (topop->routingModule->funcs.setup_pgs) {
				status = topop->routingModule->funcs.setup_pgs(topop, switchp, nodep); 
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "Failed to build PGS for destination %s 0x%"
						PRIx64", LID=%u.", nodep->nodeDesc.NodeString, 
						dgip->guid, pdp->lid);
					break;
				}
			}

			for_all_port_lids(portp, portLid) {
				switchp->lft[portLid] = xftPorts[portLid - pdp->lid];
				IB_LOG_DEBUG4_FMT(__func__, "Setting LID %u to egress port %u", 
					portLid, xftPorts[portLid - pdp->lid]);
			}
		}
	}

	switchp->routingRecalculated = 1;

	return status;
}

/*
 * A core function of dgrouting. The default version builds an LFT
 * by traversing the list of nodes in the order they were discovered.
 *
 * By contrast, the dgmh version breaks that list up into sub groups,
 * ordered by their weight/priority but otherwise uses the same mechanisms
 * to decide which ports are valid candidates to use.
 *
 * Prior method of setting up routing by traversing switch list then LIDs 
 * leads to highly unbalanced routing.  This method iterates over LIDs then 
 * switches to balance the paths.
 */
static Status_t
dgmh_calculate_all_lfts(Topology_t * topop)
{
	Node_t *switchp;
	Status_t status = VSTATUS_OK;
	int i, end;
	uint16_t portLid;
	uint8_t xftPorts[128];

	DGTopology 	*dgp = topop->routingModule->data;
	cl_map_item_t *qp;

	for_all_switch_nodes(topop, switchp) {
		status = sm_Node_init_lft(switchp, NULL);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
			return status;
		}
	}

	end = (dgp->allGroup<dgp->dgCount)?dgp->dgCount:dgp->allGroup+1;

	for (i=0; (i<end) && (status == VSTATUS_OK); i++) {
		IB_LOG_DEBUG4_FMT(__func__,"Traversing group %s", dgp->deviceGroupName[i]);
		for (qp = cl_qmap_head(&(dgp->deviceGroup[i])); 
			(status == VSTATUS_OK) && (qp != cl_qmap_end(&(dgp->deviceGroup[i])));
			qp = cl_qmap_next(qp)) {
			PortData_t 	*pdp;
			Node_t 		*nodep;
			Port_t 		*portp;
			DGItem 		*dgip;

			dgip = (DGItem*)PARENT_STRUCT(qp, DGItem, mapItem);
			portp = dgip->port;
			if (!portp) {
				IB_LOG_ERROR_FMT(__func__, "Unable to map DG entry to device port structure.");
				status = VSTATUS_BAD;
				break;
			}
			pdp = portp->portData;
			if (!pdp) {
				IB_LOG_ERROR_FMT(__func__, "Unable to map DG entry to device port data.");
				status = VSTATUS_BAD;
				break;
			}

			nodep = pdp->nodePtr;

			// skip dead ports.
			if ((nodep == NULL) ||  (!sm_valid_port(portp)) ||
				(portp->state == IB_PORT_DOWN)) {
				IB_LOG_DEBUG4_FMT(__func__, "SKIPPING lid = %u port = %u\n", 
					pdp->lid, portp->index);
				continue;
			}
		
			for_all_switch_nodes(topop, switchp) {
				status = topop->routingModule->funcs.setup_xft(topop, switchp, 
					nodep, portp, xftPorts);
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__, "Failed to build XFT for destination %s 0x%"
						PRIx64", LID=%u.", nodep->nodeDesc.NodeString, 
						dgip->guid, pdp->lid);
					break;
				}
	
				if (topop->routingModule->funcs.setup_pgs) {
					status = topop->routingModule->funcs.setup_pgs(topop, switchp, nodep); 
					if (status != VSTATUS_OK) {
						IB_LOG_ERROR_FMT(__func__, "Failed to build PGS for destination %s 0x%"
							PRIx64", LID=%u.", nodep->nodeDesc.NodeString, 
							dgip->guid, pdp->lid);
						break;
					}
				}

				for_all_port_lids(portp, portLid) {
					switchp->lft[portLid] = xftPorts[portLid - pdp->lid];
					IB_LOG_DEBUG4_FMT(__func__, "Setting LID %u to egress port %u", 
						portLid, xftPorts[portLid - pdp->lid]);
				}
			}
		}
	}
	return status;
}

static Status_t
_init_switch_lfts_dg(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s = VSTATUS_OK;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (topology_cost_path_changes || *rebalance) {
		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		// If not, copy and delta updates handled by main topology method.
		s = dgmh_calculate_all_lfts(topop);
		*rebalance = 1;
		routing_recalculated = 1;
	}

	return s;
}

/*
 *	Scan the nodemap to see if anybody has moved to a new device group.
 *
 *	Currently this can only happen if a device group is defined by the 
 *	node descriptions of its members and a description changes, but there 
 *	may be other reasons in the future. As it is, we just compare the bitsets
 *	from the current and old versions of the node.
 */
static int
dgmh_compare_device_groups(Topology_t *topop)
{
	int rebalance_needed = 0;
	cl_qmap_t	*nodeMap = topop->nodeMap;
	cl_map_item_t *qp;

	IB_LOG_DEBUG4_FMT(__func__,"Checking device group membership.");

	// Examine every node in the fabric - but we can stop the first
	// time we hit a change. That means we're slowest when there's been
	// no change!
	for (qp = cl_qmap_head(nodeMap); 
		qp != cl_qmap_end(nodeMap) && rebalance_needed == 0;
		qp = cl_qmap_next(qp)) {
		uint32_t 	i, start, end;
		Node_t 		*nodep = (Node_t*)PARENT_STRUCT(qp, Node_t, mapObj);
		Node_t 		*oldnodep;

		if (!nodep->oldExists || !nodep->old) {
			IB_LOG_INFO_FMT(__func__,"Found new node 0x%"PRIx64,
				nodep->nodeInfo.NodeGUID);
			rebalance_needed = 1;
			break;
		} 

		oldnodep = nodep->old;
		if (!bitset_equal(&(nodep->activePorts),&(oldnodep->activePorts))) {
			IB_LOG_INFO_FMT(__func__,"Node 0x%"PRIx64" ports changed.",
				nodep->nodeInfo.NodeGUID);
			rebalance_needed = 1;
			break;
		} 

		// NOTA BENE: This loop is a bit funky because for switches
		// we only want to consider port 0, but for HFIs we want to
		// consider every port **except** port 0 (HFIs don't have a port 0).
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			start = end = 0;
		} else {
			start = 1; 
			end = nodep->nodeInfo.NumPorts;
		}
		for (i = start; i <= end; i++) {
			Port_t *pp = &nodep->port[i];
			Port_t *opp = &oldnodep->port[i];

			if (pp->state != opp->state) {
				IB_LOG_INFO_FMT(__func__,"Node 0x%"PRIx64
					" port %d changed state.",
					nodep->nodeInfo.NodeGUID, i);
				rebalance_needed = 1;
				break;
			} else if (pp->state == IB_PORT_ACTIVE && !bitset_equal(&(pp->portData->dgMember),&(opp->portData->dgMember))) {
				// NOTA BENE: the dgMember bitset may not be initialized for a down port.
				IB_LOG_INFO_FMT(__func__,"Node 0x%"PRIx64
					" membership changed for port %d.",
					nodep->nodeInfo.NodeGUID, i);
				rebalance_needed = 1;
				break;
			} 
		}
	}

	IB_LOG_INFO_FMT(__func__,"Rebalance Needed = %d", rebalance_needed);
	return rebalance_needed;
}

/*
 * While the intended use for this API hook is to validate the cost matrix,
 * we're taking advantage of it to detect whether any device group memberships
 * have changed since the last sweep.
 *
 * Compares the device group memberships of end nodes in the two topologies
 * and sets *rebalance to true if they do not agree.
 */
static Status_t
dgmh_post_process_routing(Topology_t *new_topop, Topology_t *old_topop, int *rebalance)
{
	IB_LOG_INFO_FMT(__func__, "New = %p, Num Nodes = %u, Num Switches = %u",
		new_topop, (new_topop != NULL)?new_topop->num_nodes:0,
		(new_topop != NULL)?new_topop->num_sws:0);

	IB_LOG_INFO_FMT(__func__, "Old = %p, Num Nodes = %u, Num Switches = %u",
		old_topop, (old_topop != NULL)?old_topop->num_nodes:0,
		(old_topop != NULL)?old_topop->num_sws:0);

	if (!new_topop) {
		IB_LOG_ERROR_FMT(__func__, "New topology is NULL!");
		return VSTATUS_BAD;
	} else if (!rebalance) {
		IB_LOG_ERROR_FMT(__func__, "Rebalance pointer is NULL!");
		return VSTATUS_BAD;
	}

	/*
 	 * First, let's see if we can avoid checking the whole fabric.
 	 */
	if (*rebalance == 0) {
		if (!old_topop) {
			// Brand new topology.
			*rebalance = 1;
		} else if ((new_topop->num_nodes != old_topop->num_nodes) ||
			(new_topop->num_sws != old_topop->num_sws) ||
			(new_topop->num_ports != old_topop->num_ports)) {
			// # of nodes, switches or ports have changed.
			*rebalance = 1;
		} else {
			*rebalance = dgmh_compare_device_groups(new_topop);
		}
	}

	IB_LOG_INFO_FMT(__func__, "Rebalance = %d",
		*rebalance);
	return VSTATUS_OK;
}

/*
 * While the intended use for this API hook is to validate the cost matrix,
 * we're taking advantage of it to detect whether any device group memberships
 * have changed since the last sweep.
 *
 * Compares the device group memberships of end nodes in the two topologies
 * and sets *rebalance to true if they do not agree.
 */
static Status_t
dgmh_post_process_routing_copy(Topology_t *src_topop, Topology_t *dst_topop, int *rebalance)
{
	IB_LOG_INFO_FMT(__func__, "Src = %p, Num Nodes = %u, Num Switches = %u",
		src_topop, (src_topop != NULL)?src_topop->num_nodes:0,
		(src_topop != NULL)?src_topop->num_sws:0);

	IB_LOG_INFO_FMT(__func__, "Dest = %p, Num Nodes = %u, Num Switches = %u",
		dst_topop, (dst_topop != NULL)?dst_topop->num_nodes:0,
		(dst_topop != NULL)?dst_topop->num_sws:0);

	if (!dst_topop) {
		IB_LOG_ERROR_FMT(__func__, "Destination topology is NULL!");
		return VSTATUS_BAD;
	} else if (!src_topop) {
		IB_LOG_ERROR_FMT(__func__, "Source topology is NULL!");
		return VSTATUS_BAD;
	} else if (!rebalance) {
		IB_LOG_ERROR_FMT(__func__, "Rebalance pointer is NULL!");
		return VSTATUS_BAD;
	}

	/*
 	 * First, let's see if we can avoid checking the whole fabric.
	 * If we're already rebalancing, don't bother checking.
	 *
	 * HOWEVER - just because the cost matrix didn't change doesn't
	 * mean we don't have to check. Changes to the device group membership
	 * don't alter the cost matrix.
 	 */
	if (*rebalance == 0) {
		*rebalance = dgmh_compare_device_groups(dst_topop);
	}

	IB_LOG_INFO_FMT(__func__, "Rebalance = %d", *rebalance);
	return VSTATUS_OK;
}

static Status_t
dgmh_setup_switches_lrdr(Topology_t *topop, int rebalance, int routing_needed)
{
	IB_LOG_DEBUG4_FMT(__func__, "Rebalance = %d, Routing Needed = %d",
		rebalance, routing_needed);
	return	sm_setup_switches_lrdr_wave_discovery_order(topop, rebalance,
		routing_needed);
}

static Status_t
dgmh_copy(struct _RoutingModule * dest, const struct _RoutingModule * src)
{
	memcpy(dest, src, sizeof(RoutingModule_t));

	// Don't copy routing module data.  This will be recalculated.
	dest->data = NULL;

	return VSTATUS_OK;
}

Status_t
dgmh_make_routing_module(RoutingModule_t *rm)
{
	// Initialize functions which are different from defaults.
	rm->name = "dgshortestpath";
	rm->funcs.pre_process_discovery = dgmh_pre_process_discovery;
	rm->funcs.post_process_discovery = dgmh_post_process_discovery;
	rm->funcs.post_process_routing = dgmh_post_process_routing;
	rm->funcs.post_process_routing_copy = dgmh_post_process_routing_copy;
	rm->funcs.setup_switches_lrdr = dgmh_setup_switches_lrdr;
	rm->funcs.calculate_routes = dgmh_calculate_lft;
	rm->release = dgmh_destroy;
	rm->copy = dgmh_copy;

	if (sm_config.shortestPathBalanced) {
		rm->funcs.init_switch_routing = _init_switch_lfts_dg;
	}

	rm->data = NULL;

	return VSTATUS_OK;
}

Status_t
sm_dgmh_init(void)
{
	return sm_routing_addModuleFac("dgshortestpath", dgmh_make_routing_module);
}

