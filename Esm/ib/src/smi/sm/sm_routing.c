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

extern RoutingFuncs_t defaultRoutingFuncs;

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
sm_routing_copy(RoutingModule_t *newMod, const RoutingModule_t *srcMod)
{
	memcpy(newMod, srcMod, sizeof(RoutingModule_t));

	return VSTATUS_OK;
}

Status_t
sm_routing_release(struct _RoutingModule * rm)
{
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
	memset(*module, 0, sizeof(RoutingModule_t));

	(**module).funcs = defaultRoutingFuncs;
	(**module).copy = sm_routing_copy;
	(**module).release = sm_routing_release;

	return fac(*module);
}

Status_t
sm_routing_makeCopy(RoutingModule_t **newMod, RoutingModule_t *srcMod)
{
	Status_t s;

	if (newMod == NULL || srcMod == NULL)
		return VSTATUS_ILLPARM;

	if ((s = vs_pool_alloc(&sm_pool, sizeof(RoutingModule_t),
		(void*)newMod)) != VSTATUS_OK)
		return s;

	if ((s = srcMod->copy(*newMod, srcMod)) != VSTATUS_OK) {
		vs_pool_free(&sm_pool, *newMod);
		*newMod = NULL;
		return s;
	}

	return VSTATUS_OK;
}

Status_t
sm_routing_freeModule(RoutingModule_t ** module)
{
	if (module == NULL || *module == NULL)
		return VSTATUS_ILLPARM;

	(*module)->release(*module);

	vs_pool_free(&sm_pool, *module);
	*module = NULL;

	return VSTATUS_OK;
}

Status_t
sm_routing_copy_cost_matrix(Topology_t *src_topop, Topology_t *dst_topop)
{
	Status_t status;
	size_t   bytesCost, bytesPath;

	if (!src_topop->cost) {
		// routing module not using cost matrix
		return VSTATUS_OK;
	}

	bytesCost = src_topop->max_sws * src_topop->max_sws * sizeof(uint16_t);

	status = vs_pool_alloc(&sm_pool, bytesCost, (void *)&dst_topop->cost);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("TT(topop): can't malloc cost array; rc:", status);
		IB_EXIT(__func__, status);
		return status;
	}

	dst_topop->bytesCost = bytesCost;

	memcpy((void *)dst_topop->cost, (void *)src_topop->cost, bytesCost);


	bytesPath = SM_PATH_SIZE(src_topop->max_sws);

	status = vs_pool_alloc(&sm_pool, bytesPath, (void *)&dst_topop->path);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("TT(topop): can't malloc cost array; rc:", status);
		IB_EXIT(__func__, status);
		return status;
	}

	dst_topop->bytesPath = bytesPath;

	memcpy((void *)dst_topop->path, (void *)src_topop->path, bytesPath);

	return VSTATUS_OK;
}

/*
 * Builds the routing tables for a switch, then sends the part needed for the switch to
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

	if (topop->routingModule->funcs.needs_routing_recalc(topop, nodep)) {
		/* calculate the full LFTs for this switch*/
		if ((status = topop->routingModule->funcs.calculate_routes(sm_topop, nodep)) != VSTATUS_OK)
			return status;
	}

	if (sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__,
				"writing minimal routes for switch "FMT_U64, nodep->nodeInfo.NodeGUID);

	/* setup route blocks for the SM LID and the switch's own LID*/
	status = topop->routingModule->funcs.write_minimal_routes(sm_topop, nodep, use_lr_dr_mix, path);

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
	 	if ((status = topop->routingModule->funcs.write_full_routes_LR(topop, swlist, rebalance)) != VSTATUS_OK) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Error in writing full routes");
		}
	}

	return status;
}

Status_t
sm_routing_route_old_switch(Topology_t *src_topop, Topology_t *dst_topop, Node_t *nodep)
{
	Status_t status;
	Node_t   *oldNodep;
	Port_t   *oldPortp, *portp;
	size_t   lftLength;

	if (nodep->switchInfo.LinearFDBCap == 0) {
		IB_LOG_ERROR_FMT(__func__, "switch doesn't support lft %s",
			   sm_nodeDescString(nodep));
		return VSTATUS_BAD;
	}

	// This func requires dst_topop == sm_topop
	if (dst_topop != sm_topop) {
		IB_LOG_ERROR_FMT(__func__, "This function only works for dst_topop == sm_topop");
		return VSTATUS_ILLPARM;
	}

	if (dst_topop->routingModule->funcs.check_switch_path_change(src_topop, dst_topop, nodep)) {
		if (dst_topop->routingModule->funcs.needs_routing_recalc(dst_topop, nodep)) {
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__, "Full route calc for switch with path change %s",
									sm_nodeDescString(nodep));
			status = sm_setup_lft(dst_topop, nodep);
		} else {
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__,
						"Full LFT send switch with path change %s", sm_nodeDescString(nodep));
			status = sm_send_lft(dst_topop, nodep);
		}
		return status;
	}

	oldNodep = sm_find_guid(src_topop, nodep->nodeInfo.NodeGUID);
	if (!oldNodep || !oldNodep->lft ||
		(esmLoopTestOn &&
		 nodep->switchInfo.LinearFDBTop != oldNodep->switchInfo.LinearFDBTop)) {
		// Top will differ if loop test enabled since last sweep.

		if (dst_topop->routingModule->funcs.needs_routing_recalc(dst_topop, nodep)) {
			status = sm_setup_lft(dst_topop, nodep);
		} else {
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__,
						"Full LFT send switch with no old data %s", sm_nodeDescString(nodep));
			status = sm_send_lft(dst_topop, nodep);
		}
		return status;
	}

	if (  nodep->initPorts.nset_m
	   || !bitset_equal(&nodep->activePorts, &oldNodep->activePorts)) {
		if (dst_topop->routingModule->funcs.handle_fabric_change(dst_topop, oldNodep, nodep)) {
			if (dst_topop->routingModule->funcs.needs_routing_recalc(dst_topop, nodep)) {
				status = sm_setup_lft(dst_topop, nodep);
			} else {
				IB_LOG_INFINI_INFO_FMT(__func__,
					"Full LFT send switch with Port Change %s", sm_nodeDescString(nodep));
				status = sm_send_lft(dst_topop, nodep);
			}
			return status;
		}
	}

	// Recover lidsRouted
	nodep->numLidsRouted = oldNodep->numLidsRouted;
	for_all_ports(nodep, portp) {
		if (sm_valid_port(portp)) {
			oldPortp = sm_get_port(oldNodep, portp->index);
			if (sm_valid_port(oldPortp)) {
				portp->portData->lidsRouted = oldPortp->portData->lidsRouted;
			}
		}
	}

	if ((new_endnodesInUse.nset_m ||
		 src_topop->num_endports != dst_topop->num_endports) &&
		dst_topop->routingModule->funcs.can_send_partial_routes()) {
		if (sm_config.sm_debug_routing)
			IB_LOG_INFINI_INFO_FMT(__func__, "Partial LFT send - switch %s", sm_nodeDescString(nodep));
		sm_send_partial_lft(sm_topop, nodep, &sm_topop->deltaLidBlocks);
		return VSTATUS_OK;
	}

	// PR-119954: This PR identified a memory leak that resulted from the following vs_pool_alloc() being executed when nodep->lft pointed to
	//  		  a lft that was already allocated.  A fix was made to topology_setup_switches_LR_DR() so that this code should never execute
	//  		  when nodep->lft points to an lft.  Although this should no longer happen, the following code is added to prevent a memory leak
	// 			  by freeing the lft, if node->lft is found to be non NULL.
	if (nodep->lft && sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__, "new lft - switch %s nodep %p nodep->index %u nodep->lft %p",
				sm_nodeDescString(nodep), nodep, nodep->index, nodep->lft);

	// Just additions, adjust LFT blocks with removed or new lids.
	if ((status = sm_Node_init_lft(nodep, &lftLength)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_routing_route_old_switch: CAN'T ALLOCATE SPACE FOR NODE'S LFT;  OUT OF MEMORY IN SM MEMORY POOL!  TOO MANY NODES!!");
		return VSTATUS_NOMEM;	/*calling function can use this value to abort programming old switches*/
	}

	if (nodep->switchInfo.LinearFDBTop > oldNodep->switchInfo.LinearFDBTop) {
		memcpy((void *)nodep->lft, (void *)oldNodep->lft,
			sizeof(PORT) * (oldNodep->switchInfo.LinearFDBTop + 1));
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

	if (new_endnodesInUse.nset_m ||
		src_topop->num_endports != dst_topop->num_endports) {
		if (sm_config.sm_debug_routing)
			IB_LOG_INFINI_INFO_FMT(__func__, "Delta route calc for switch with same paths %s",
									sm_nodeDescString(nodep));
		return sm_setup_lft_deltas(src_topop, dst_topop, nodep);
	}


	if (sm_config.sm_debug_routing)
		IB_LOG_INFINI_INFO_FMT(__func__, "Copied old LFTs for switch %s", sm_nodeDescString(nodep));

	return VSTATUS_OK;
}

Status_t sm_routing_route_switch_LR(Topology_t *topop, SwitchList_t *swlist, int rebalance)
{
// PR-119954 reported a memory leak.  It occurs in the case when calculate_routes() is called to allocate a new lft for the new
//  		 root node structure and this lft is replaced by another lft in sm_routing_route_old_switch_LR() without
//  		 freeing the lft allocated in calculate_routes().  topology_setup_switches_LR_DR() has been updated to set the third parameter (rebalance)
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
		status = topop->routingModule->funcs.route_old_switch(&old_topology, topop, sw->switchp);
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
	STL_LID bestLidsRouted = 0xffff; // Gen1 uses 16 bit lids.
	STL_LID bestSwLidsRouted = 0xffff; // Gen1 uses 16 bit lids.

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
	STL_LID lid, portLid;
	uint8_t xftPorts[128];

	for_all_switch_nodes(topop, switchp) {
		status = sm_Node_init_lft(switchp, NULL);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
			return status;
		}

		// Initialize port group top prior to setting up groups.
		switchp->switchInfo.PortGroupTop = 0;
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
sm_routing_init(void)
{
	Status_t status = VSTATUS_BAD;

	cl_qmap_init(&moduleFacMap, _qmap_str_cmp);

	status = sm_shortestpath_init();
	if (status != VSTATUS_OK)
		return status;

	status = sm_fattree_init();
	if (status != VSTATUS_OK)
		return status;

	status = sm_dgmh_init();
	if (status != VSTATUS_OK)
		return status;

	status = sm_hypercube_init();
	if (status != VSTATUS_OK)
		return status;

	status = sm_dor_init();
	return status;
}
