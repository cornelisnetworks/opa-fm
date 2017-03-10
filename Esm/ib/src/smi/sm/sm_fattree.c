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

Status_t sm_shortestpath_make_routing_module(RoutingModule_t * rm);

static __inline__ void
_is_route_last(Node_t * nodep)
{
	Port_t* portp;
	int dgIdx;

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
		return;

	if (strlen(sm_config.ftreeRouting.routeLast.member) == 0)
		return;

	dgIdx = smGetDgIdx(sm_config.ftreeRouting.routeLast.member);
	if (dgIdx == -1) 
		return;

	for_all_physical_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;

		// is it a member of the route last device group
		if (bitset_test(&portp->portData->dgMember, dgIdx)) {
			nodep->skipBalance = 1;
			return;
		}
	}
}

static __inline__ int
_is_core(Node_t* switchp) {

	Port_t* portp;
	int dgIdx = smGetDgIdx(sm_config.ftreeRouting.coreSwitches.member);

	if (dgIdx < 0) {
		IB_LOG_WARN_FMT(__func__, "FatTreeTopology has undefined CoreSwitches device group%s",
						sm_config.ftreeRouting.coreSwitches.member);
		return 0;
	}

	// get switch cport
	portp = sm_get_port(switchp, 0);
	if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
		return 0;

	// is it a member of the root/core switches?
	if (!bitset_test(&portp->portData->dgMember, dgIdx))
		return 0;

	return 1;
}

static __inline__ void
_incr_trunk_count(Node_t* switchp, Port_t* swportp, int *trunkCnt, uint8_t *trunkGrpCnt, uint16_t *trunkGrpNode) {
	int i;
	int cnt = *trunkCnt;

	for (i=0; i<cnt; i++) {
		if (trunkGrpNode[i] == swportp->nodeno) {
			trunkGrpCnt[i]++;
			break;
		}
	}
	if (i==cnt) {
		trunkGrpNode[cnt] = swportp->nodeno;
		trunkGrpCnt[cnt] = 1;
		*trunkCnt = ++cnt;
	}

	return;
}

static __inline__ void
_set_trunk_count(Node_t* switchp, int trunkCnt, uint8_t *trunkGrpCnt, uint16_t *trunkGrpNode) {
	int min, i;

	switchp->uplinkTrunkCnt = trunkGrpCnt[0];
	min = switchp->uplinkTrunkCnt;
	for (i=1; i<trunkCnt; i++) {
		if (trunkGrpCnt[i] > switchp->uplinkTrunkCnt) {
			switchp->uplinkTrunkCnt = trunkGrpCnt[i];
		}
		if (min < trunkGrpCnt[i]) {
			min = trunkGrpCnt[i];
		}
	}
	if (abs(switchp->uplinkTrunkCnt - min) > 1) {
		switchp->uplinkTrunkCnt = 1;
	}
}

static Status_t
_post_process_discovery(Topology_t *topop, Status_t discoveryStatus, void *context)
{
	Node_t		*switchp, *nodep;
	Port_t		*swportp, *remotePortp;
	bitset_t	processedSwitches;
	uint8_t		trunkGrpCnt[255];
	uint16_t	trunkGrpNode[255];
	int			trunkCnt, i, t;

	memset(trunkGrpCnt,0,sizeof(trunkGrpCnt));

	if (topop->num_sws == 0) {
		return VSTATUS_OK;
	}

	if (!bitset_init(&sm_pool, &processedSwitches, bitset_find_last_one(&new_switchesInUse)+1)) {
		IB_LOG_ERROR0("No memory error.");
		return VSTATUS_BAD;
	}

	// check for ca route last
	if (strlen(sm_config.ftreeRouting.routeLast.member)) {
		for_all_ca_nodes(topop, nodep) {
			_is_route_last(nodep);
		}
	}

	// HFIs at lowest tier in fat tree, easier to process
	if (sm_config.ftreeRouting.fis_on_same_tier) {
		for_all_switch_nodes(topop, switchp) {
			// find tier0 switches
			for_all_physical_ports(switchp, swportp) {
				if (!sm_valid_port(swportp) ||
					swportp->state <= IB_PORT_DOWN) continue;
				nodep = sm_find_node(topop, swportp->nodeno);
				if (!nodep) continue;
				if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
					swportp->portData->downlink = 1;
					if (bitset_test(&processedSwitches, switchp->swIdx)) continue;
	
					Switch_Enqueue_Type(topop, switchp, 0, 1);
					bitset_set(&processedSwitches, switchp->swIdx);
				}
			}
		}

		// move up tree	
		for (t=1; t<sm_config.ftreeRouting.tierCount; t++) {
			for_all_tier_switches(topop, switchp, t-1) {
				trunkCnt = 0;
				switchp->trunkGrouped = 1;
				for_all_physical_ports(switchp, swportp) {
					if (!sm_valid_port(swportp) ||
						swportp->state <= IB_PORT_DOWN) continue;
					nodep = sm_find_node(topop, swportp->nodeno);
					if (!nodep) continue;
					if (nodep->nodeInfo.NodeType == NI_TYPE_CA) continue;
					if (!bitset_test(&processedSwitches, nodep->swIdx) ||
						(nodep->tier > switchp->tier)) {

						// port is up link
						swportp->portData->uplink = 1;
 						remotePortp=sm_find_port(topop, swportp->nodeno, swportp->portno);
						if (remotePortp)
							remotePortp->portData->downlink = 1;

						_incr_trunk_count(switchp, swportp, &trunkCnt, trunkGrpCnt, trunkGrpNode);
					}

					if (bitset_test(&processedSwitches, nodep->swIdx)) continue;

					Switch_Enqueue_Type(topop, nodep, t, 1);
					bitset_set(&processedSwitches, nodep->swIdx);
				}

				_set_trunk_count(switchp, trunkCnt, trunkGrpCnt, trunkGrpNode);
			}

			if (processedSwitches.nset_m == new_switchesInUse.nset_m) break;
		}

	} else {
		// HFIs are at various tiers of tree, more difficult to determine tree structure
		// Determine core switches from config information.
		for_all_switch_nodes(topop, switchp) {
			if (_is_core(switchp)) {
				Switch_Enqueue_Type(topop, switchp, sm_config.ftreeRouting.tierCount-1, 0);
				bitset_set(&processedSwitches, switchp->swIdx);
			}
		}
	
		// Work down from top tier that was just determined
		for (t=sm_config.ftreeRouting.tierCount-1; t>0; t--) {
			for_all_tier_switches(topop, switchp, t) {
				trunkCnt = 0;
				switchp->trunkGrouped = 1;
				for_all_physical_ports(switchp, swportp) {
					if (!sm_valid_port(swportp) ||
						swportp->state <= IB_PORT_DOWN) continue;

					nodep = sm_find_node(topop, swportp->nodeno);
					if (!nodep) continue;
					if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
						swportp->portData->downlink = 1;
 						continue;
					}

					if (!swportp->portData->uplink) {
						// Higher tier hasn't set this port to uplink, must be down
						swportp->portData->downlink = 1;
 						remotePortp=sm_find_port(topop, swportp->nodeno, swportp->portno);
						if (remotePortp)
							remotePortp->portData->uplink = 1;

					} else {
						if (nodep->tier > switchp->tier) {
							_incr_trunk_count(switchp, swportp, &trunkCnt, trunkGrpCnt, trunkGrpNode);
							continue;
						}
					}

					if (bitset_test(&processedSwitches, nodep->swIdx)) continue;

					Switch_Enqueue_Type(topop, nodep, t-1, 1);
					bitset_set(&processedSwitches, nodep->swIdx);
				}

				if (trunkCnt) {
					_set_trunk_count(switchp, trunkCnt, trunkGrpCnt, trunkGrpNode);
				}
			}
		}

		// Process lowest tier, calculating uplink trunk count
		for_all_tier_switches(topop, switchp, 0) {
			trunkCnt = 0;
			switchp->trunkGrouped = 1;
			for_all_physical_ports(switchp, swportp) {
				if (!sm_valid_port(swportp) ||
					swportp->state <= IB_PORT_DOWN) continue;
				nodep = sm_find_node(topop, swportp->nodeno);
				if (!nodep) continue;
				if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
					swportp->portData->downlink = 1;
 					continue;
				}

				_incr_trunk_count(switchp, swportp, &trunkCnt, trunkGrpCnt, trunkGrpNode);
			}
			_set_trunk_count(switchp, trunkCnt, trunkGrpCnt, trunkGrpNode);
		}
	}

	// List is sorted, index set post list create.
	// Setup tier index for each switch
	for (t=0; t<sm_config.ftreeRouting.tierCount; t++) {
		i = 0;
		for_all_tier_switches(topop, switchp, t) {
			switchp->tierIndex = i++;
			
			// Debug dump of fat tree info
			if (sm_config.ftreeRouting.debug) {
				for_all_physical_ports(switchp, swportp) {
					if (!sm_valid_port(swportp) || swportp->state <= IB_PORT_DOWN)
						continue;
					IB_LOG_INFINI_INFO_FMT(__func__, "Switch %s port %d: type %s tier %d, tierIndex %d, trunk %d grouped %d",
						sm_nodeDescString(switchp), swportp->index,
						swportp->portData->uplink ? "uplink" : (swportp->portData->downlink ? "downlink" : "unknown"),
						switchp->tier, switchp->tierIndex, switchp->uplinkTrunkCnt, switchp->trunkGrouped);
				}
			}
		}
	}

	if (processedSwitches.nset_m == 0) {
		IB_LOG_WARN0("Disabling fattree due to topology error, setting to shortestpath");
		// Zero out existing pointers.  No data to release, though
		memset(topop->routingModule, 0, sizeof(RoutingModule_t));
		sm_shortestpath_make_routing_module(topop->routingModule);
	}

	if (processedSwitches.nset_m != new_switchesInUse.nset_m) {
		IB_LOG_ERROR_FMT(__func__, "Switch(es) unassigned to/out-of-bound (fattree) tier. Check TierCount in config file.");
		return VSTATUS_BAD;
	}
	bitset_free(&processedSwitches);

	return VSTATUS_OK;
}

static boolean
_do_spine_check(Topology_t *topop, Node_t *switchp)
{
	return sm_config.spine_first_routing;
}

static __inline__ void
incr_lids_routed(Topology_t *topop, Node_t *switchp, int port)
{

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
_calculate_balanced_lfts_systematic(Topology_t *topop)
{
	Node_t *nodep, *switchp, *toSwitchp;
	Port_t *portp, *swPortp, *toSwitchPortp;
	Status_t status;
	uint8_t portGroup[128];
	int upDestCount = 0;
	int downDestCount = 0;
	STL_LID currentLid;
	int numPorts, t, i;
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
			topop->routingModule->funcs.calculate_routes(sm_topop, switchp);
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

static Status_t
_copy_balanced_lfts(Topology_t *topop)
{
	size_t		lftLength;
	Node_t		*switchp, *oldnodep;
	Port_t		*portp, *oldportp;
	Status_t	status=VSTATUS_OK;

	IB_ENTER(__func__, topop, 0, 0, 0);

	if (sm_config.ftreeRouting.debug) {
		IB_LOG_INFINI_INFO0("entry");
	}

	for_all_switch_nodes(topop, switchp) {
		oldnodep = switchp->old;
		if (!oldnodep) {
			oldnodep = sm_find_guid(&old_topology, switchp->nodeInfo.NodeGUID);
		}

		if (!oldnodep || !oldnodep->lft) {
			// Shouldn't get here
			if (sm_config.sm_debug_routing)
				IB_LOG_INFINI_INFO_FMT(__func__, "Full LFT for switch %s on old node checks",
								sm_nodeDescString(switchp));
			status = sm_setup_lft(topop, switchp);
			continue;
		}

		status = sm_Node_init_lft(switchp, &lftLength);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
			return status;
		}

		switchp->numLidsRouted = oldnodep->numLidsRouted;

		// Don't copy the portgroups if the switch-switch linkage may have changed.
		// TBD - what if only a new HFI linked to switch.
		boolean copyPgs = (!switchp->initPorts.nset_m && bitset_equal(&switchp->activePorts, &oldnodep->activePorts));

		if (switchp->switchInfo.LinearFDBTop > oldnodep->switchInfo.LinearFDBTop) {
			memcpy((void *)switchp->lft, (void *)oldnodep->lft,
				sizeof(uint8_t) * (oldnodep->switchInfo.LinearFDBTop + 1));
		} else {
			memcpy((void *)switchp->lft, (void *)oldnodep->lft,
				sizeof(uint8_t) * (switchp->switchInfo.LinearFDBTop + 1));
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

static Status_t
_calculate_balanced_lft_deltas(Topology_t *topop)
{
	STL_LID		lid;
	int			curBlock;
	Node_t		*switchp, *nodep;
	Port_t		*portp;
	uint8_t		xftPorts[128];
	STL_LID		curLid;

	IB_ENTER(__func__, topop, 0, 0, 0);

	if (smDebugPerf || sm_config.ftreeRouting.debug) {
		IB_LOG_INFINI_INFO0("entry");
	}

	_copy_balanced_lfts(topop);

	curBlock = -1;

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
					if (sm_config.sm_debug_routing && !bitset_test(&topop->deltaLidBlocks, curBlock))
						IB_LOG_INFINI_INFO_FMT(__func__,
							"Setting curBlock %d due to delete of lid 0x%x", curBlock, lid);

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
		// TBD: what if lmc range straddles block?
		if (curBlock != lid/LFTABLE_LIST_COUNT) {
			// Set lft block num
			curBlock = lid/LFTABLE_LIST_COUNT;
			if (sm_config.sm_debug_routing && !bitset_test(&topop->deltaLidBlocks, curBlock))
				IB_LOG_INFINI_INFO_FMT(__func__,
						"Setting curBlock %d due to ADD of lid 0x%x", curBlock, lid);
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

static Status_t
_init_switch_lfts(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s=VSTATUS_OK;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (topology_cost_path_changes || *rebalance) {
		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		s = _calculate_balanced_lfts_systematic(topop);
		*rebalance = 1;
		routing_recalculated = 1;

	} else if (	new_endnodesInUse.nset_m ||
					old_topology.num_endports != topop->num_endports) {
		// If I am following this, the deltas are now done in setup_floyds/copy_lfts.
		// Disabling this for balanced, since it needs to be done across all switches.
		// Setting routing needed so partial lfts are written.

		if (!bitset_init(&sm_pool, &topop->deltaLidBlocks, topop->maxLid/LFTABLE_LIST_COUNT+1)) {
			IB_LOG_ERROR_FMT(__func__, "Failed to init deltaLidBlocks");
			return VSTATUS_BAD;
		}
		topop->deltaLidBlocks_init = 1;

		s = _calculate_balanced_lft_deltas(topop);
		*routing_needed = 1;

	} else if (!*routing_needed) {
			s = _copy_balanced_lfts(topop);
	}

	return s;
}

static Status_t
_calculate_balanced_lft(Topology_t *topop, Node_t *switchp)
{
	Node_t *nodep, *toSwitchp;
	Port_t *portp, *swPortp, *toSwitchPortp;
	Status_t status;
	uint8_t portGroup[128];
	int upDestCount = 0;
	int downDestCount = 0;
	int currentLid, numPorts, i;
	int r, routingIterations;
	int ht, hfiTierEnd=0;
	int isUp = 1;

	if (sm_config.sm_debug_routing || sm_config.ftreeRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__, "switch %s", switchp->nodeDesc.NodeString);

	if (!Is_Switch_Queued(topop, switchp)) {
		if (sm_config.ftreeRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__, "Switch not queued %s", sm_nodeDescString(switchp));

		// Switch may not be added to tier switch group when no FIs in down path.
		// If so, doesn't matter if it is balanced.
		return sm_calculate_lft(topop, switchp);
	}

	// If we can't guarantee that all HFIs/FIs are on the same tier,
	// balancing the LFTs is more complicated.
	if (!sm_config.ftreeRouting.fis_on_same_tier) {
		hfiTierEnd = sm_config.ftreeRouting.tierCount-1;
	}

	status = sm_Node_init_lft(switchp, NULL);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
		return status;
	}

	// if route last is indicated, balance over those HFIs on a second pass
	routingIterations = (strlen(sm_config.ftreeRouting.routeLast.member) == 0) ? 1 : 2;

	for (r=0; r<routingIterations; ++r) {
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

	return VSTATUS_OK;
}

static Status_t
_make_fattree(RoutingModule_t * rm)
{
	rm->name = "fattree";
	rm->funcs.copy_routing = sm_routing_func_copy_routing_noop;
	rm->funcs.can_send_partial_routes = sm_routing_func_can_send_partial_routes_true;
	rm->funcs.init_switch_routing = _init_switch_lfts;
	rm->funcs.calculate_routes = _calculate_balanced_lft;
	rm->funcs.post_process_discovery = _post_process_discovery;
	rm->funcs.do_spine_check = _do_spine_check;

	return VSTATUS_OK;
}

Status_t
sm_fattree_init(void)
{
	Status_t s;
	s = sm_routing_addModuleFac("fattree", _make_fattree);

	return s;
}
