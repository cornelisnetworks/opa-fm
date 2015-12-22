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


static Status_t
_setup_pgs(struct _Topology *topop, struct _Node * srcSw, const struct _Node * dstSw);

typedef struct GuidCounter
{
	uint64_t guid;
	uint32_t offset;
	uint32_t count;
} GuidCounter_t;

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
	if (uniqueGuids == 1 || uniqueGuids == portCount)
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

static void
_spine_first_reset(SpineFirstState_t *state)
{
	state->matching = 0;
}

static SpineFirstResult_t
_spine_first_test(SpineFirstState_t *state, Node_t *switchp, Port_t *portp, Node_t *next_nodep)
{
	if (!sm_config.spine_first_routing) return SPINE_FIRST_NONE;
	if ((switchp->nodeInfo.SystemImageGUID != next_nodep->nodeInfo.SystemImageGUID) &&
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

// selects all best ports to the provided switch index
// returns the number of ports found (0 if none)
//
static int
_select_ports(Topology_t *topop, Node_t *switchp, int endIndex, SwitchportToNextGuid_t *ordered_ports, boolean selectBest)
{
	int      i, j, k;
	uint16_t cur_speed = 0;
	uint16_t best_speed = 0;
	uint16_t best_cost = 0xffff;
	uint16_t best_lidsRouted = 0xffff;
	uint32_t best_switchLidsRouted = 0xffffffff;
	int      end_port = 0;
	Node_t   *next_nodep;
	Port_t   *portp;
	SpineFirstState_t sfstate;
	SpineFirstResult_t sfres;

	i = switchp->swIdx;
	j = endIndex;
	best_cost = topop->cost[Index(i, j)];
	_spine_first_reset(&sfstate);

	for_all_physical_ports(switchp, portp) {
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
				ordered_ports[0].nextSwp = next_nodep;
				end_port = 1;
				break;
			case SPINE_FIRST_NOMATCH:
				// we're only considering spines, and this isn't one.
				// discard it
				break;
			case SPINE_FIRST_MATCH:
				// we've seen at least one spine so far... is this one better?
				// fall through to default behavior to determine
			case SPINE_FIRST_NONE:
				// spine first is not enabled or there are no spines off this
				// node so far.  balance normally
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
							break;
						}
					}

					ordered_ports[end_port].portp = portp;
					ordered_ports[end_port].guid = next_nodep->nodeInfo.NodeGUID;
					ordered_ports[end_port].nextSwp = next_nodep;
					++end_port;
				}
				break;
			}
		}
	}

	return end_port;
}

static void
_balance_ports(Node_t *switchp, SwitchportToNextGuid_t *ordered_ports, int olen)
{
	int i, j;

	if (olen <= 0) return;

	if (switchp->internalLinks) {
		// no reason to sort on guid since only single link to each remote switch
		qsort(ordered_ports, olen, sizeof(SwitchportToNextGuid_t),
	      	_compare_lids_routed);
	} else {
		qsort(ordered_ports, olen, sizeof(SwitchportToNextGuid_t),
	      	_compare_guids_then_lids_routed);

		_guid_cycle_sort(ordered_ports, olen, (void*)(ordered_ports + olen), FALSE);
		for (i = 0, j = 0; i < olen; i++) {
			if (i == olen - 1 || ordered_ports[i].guid > ordered_ports[i+1].guid) {
				qsort(&ordered_ports[j], i - j + 1, sizeof(SwitchportToNextGuid_t),
			      	_compare_lids_routed_then_guids);
				j = i + 1;
			}
		}
	}
}

static int
_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *nodep, uint8_t *portnos) {
	int i, j;
	int end_port = 0;
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;

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

	memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * switchp->nodeInfo.NumPorts);

	end_port = _select_ports(topop, switchp, j, ordered_ports, 0);
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

// -------------------------------------------------------------------------- //
//
//	This is a common routine used by the LFT and RFT routines to parse
//	out what the path is through the fabric in order to setup the routing
//	tables.
//  See sm_l.h for parameter documentation
static Status_t
_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *nodep, Port_t *orig_portp, uint8_t *portnos) {
	int i, j;
	uint8_t numLids;
	int lidsRoutedInc;
	int offset=0;
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
	int end_port = 0;

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
				"Failed to find neighbor node "FMT_U64" from NodeGUID "FMT_U64" Port %d",
				orig_portp->nodeno, ntp->nodeInfo.NodeGUID, orig_portp->index);
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
		memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t));

		// select best port, _select_ports will return 1 or 0 (no path)
		if ((end_port = _select_ports(topop, switchp, j, ordered_ports, 1)) == 0) {
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
		memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * switchp->nodeInfo.NumPorts);

		end_port = _select_ports(topop, switchp, j, ordered_ports, 0);
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

static Status_t
_setup_pgs(struct _Topology *topop, struct _Node * srcSw, const struct _Node * dstSw)
{
	SwitchportToNextGuid_t * ordered_ports = (SwitchportToNextGuid_t*) topop->pad;

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

	memset(ordered_ports, 0, sizeof(SwitchportToNextGuid_t) * srcSw->nodeInfo.NumPorts);

	int end_port = _select_ports(topop, srcSw, dstSw->swIdx, ordered_ports, 0);
	if (end_port <= 1) {
		srcSw->switchInfo.PortGroupTop = 0;
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
		srcSw->switchInfo.PortGroupTop = srcSw->pgtLen; //MAX(srcSw->switchInfo.PortGroupTop, srcSw->pgtLen);

		//PGFT is independent of LFT with LMC, though it's supposed to re-use the LMC data
		PORT * pgft = sm_Node_get_pgft_wr(srcSw);
		uint32_t pgftLen = sm_Node_get_pgft_size(srcSw);

		if (!pgft) {
			IB_LOG_ERROR_FMT(__func__, "Failed to acquire memory for PGFT");
			return VSTATUS_BAD;
		}

		// Add every lid of dstSw to srSw's pgft.
		// (assuming the lid is < the pgftLen)
		STL_LID_32 portLid = 0;
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


static Status_t
_pre_process_discovery(Topology_t *topop, void **outContext)
{
	return VSTATUS_OK;
}

static Status_t
_discover_node(Topology_t *topop, Node_t *nodep, void *context)
{
	return VSTATUS_OK;
}

static Status_t
_discover_node_port(Topology_t *topop, Node_t *nodep, Port_t *portp, void *context)
{
	return VSTATUS_OK;
}

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
			if (i != (cnt-1)) {
				// non-contiguous set of ports in group
				switchp->trunkGrouped = 0;
			}
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

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	if (VirtualFabrics && VirtualFabrics->qosEnabled) {
		topop->qosEnforced = 1;
	}

	if (topop->routingModule->alg != SM_ROUTE_ALG_FATTREE) {
		return VSTATUS_OK;
	}

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

	bitset_free(&processedSwitches);

	return VSTATUS_OK;
}

static Status_t
_setup_switches_lrdr(Topology_t *topop, int rebalance, int routing_needed)
{
	return	sm_setup_switches_lrdr_wave_discovery_order(topop, rebalance, routing_needed);
}

static Status_t
_post_process_routing(Topology_t *topop, Topology_t *old_topop, int *rebalance)
{
	return VSTATUS_OK;
}

static Status_t
_post_process_routing_copy(Topology_t *src_topop, Topology_t *dst_topop, int *rebalance)
{
	return VSTATUS_OK;
}

static Status_t
_select_slsc_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SLSCMAP *outSlscMap)
{
    uint8_t sl, vf; 
    STL_SLSCMAP slsc;

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	// the current VF implementation sets up generic SL2SC mappings, based on the
	// IB legacy SL2VL mapping (SC is VL).  In order to generate unique SL2SC
	// map for this ingress port, filter the SLs based on this port's VF memberships.
    for (vf = 0; vf < MAX_VFABRICS; vf++) {
        bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].base_sl);
    }

	if (bitset_nset(&sm_linkSLsInuse) == 0)
		return VSTATUS_BAD;

	//
    // construct SLSC mapping table
	memset(&slsc, 0, sizeof(slsc));

    // populate the SL2SC map table with entries from the baseline SL2SC
    // map table, based on SLs associated with this port.
    for (sl = 0; sl < STL_MAX_SLS; sl++) {
		if (!bitset_test(&sm_linkSLsInuse, sl))
            slsc.SLSCMap[sl].SC = 15;   // mark as invalid
        else
            slsc.SLSCMap[sl].SC = sm_SLtoSC[sl];
    }

	memcpy(outSlscMap, &slsc, sizeof(STL_SLSCMAP));
	return VSTATUS_OK;
}

static Status_t
_select_scsl_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SCSLMAP *outScslMap)
{
    uint8_t sl, sc, vf; 
    STL_SCSLMAP scsl;

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

    for (vf = 0; vf < MAX_VFABRICS; vf++) {
        bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].base_sl);
    }

	if (bitset_nset(&sm_linkSLsInuse) == 0)
		return VSTATUS_BAD;

	//
    // construct SLSC mapping table
	memset(&scsl, 0, sizeof(scsl));

    // populate the SC2SL map table with entries from the baseline SC2SL
    // map table, based on SLs associated with this port.
    for (sc = 0; sc < STL_MAX_SCS; sc++) {
        sl = sm_SCtoSL[sc];
		if (!bitset_test(&sm_linkSLsInuse, sl))
            scsl.SCSLMap[sc].SL = 15;   // mark as invalid
        else
            scsl.SCSLMap[sc].SL = sl;
    }

	memcpy(outScslMap, &scsl, sizeof(STL_SCSLMAP));
	return VSTATUS_OK;
}

static Status_t
_select_scvl_map(Topology_t *topop, Node_t *nodep,
    Port_t *in_portp, Port_t *out_portp, STL_SCVLMAP *outScvlMap)
{
#ifdef USE_FIXED_SCVL_MAPS
    // No more filtering. Just copy the full scvl map.
	Qos_t *qos = GetQos(out_portp->portData->vl1);
    memcpy(outScvlMap, &qos->scvl, sizeof(STL_SCVLMAP));
#else
    STL_SCVLMAP scvl;
	Qos_t *qos = GetQos(out_portp->portData->vl1);
    int sc, sl, vf;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

    if (in_portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH &&
        out_portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        // for ISL 1:1 mapping, so use default QoS map
        memcpy(outScvlMap, &qos->scvl, sizeof(STL_SCVLMAP));
    } else {
        Port_t *portp;

        bitset_clear_all(&sm_linkSLsInuse);

        // filtering based on VF membership via the HFI port, because VF
        // membership for switches is always zero. 
        portp = (in_portp->portData->nodePtr->nodeInfo.NodeType != NI_TYPE_SWITCH) ? in_portp : out_portp;

        for (vf = 0; vf < MAX_VFABRICS; vf++) {
            if (bitset_test(&portp->portData->vfMember, vf) == 0)
                continue;  // not a member of VF

            bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric[vf].base_sl);
            //IB_LOG_INFINI_INFO_FMT(__func__, "Member VF %s, base_sl=%d,rt_sls=%d, rt_scs=%d", 
            //                       VirtualFabrics->v_fabric[vf].name, VirtualFabrics->v_fabric[vf].base_sl, 
            //                       VirtualFabrics->v_fabric[vf].routing_sls, VirtualFabrics->v_fabric[vf].routing_scs);
        }

        if (bitset_nset(&sm_linkSLsInuse) == 0)
            return VSTATUS_BAD;

        // construct SC2VL from the baseline QoS SL2SC/SL2VL mapping table, which has
        // already been setup to address the generic expansion, contraction,
        // and conservation mapping of SL2SC/SL2VL. 
        memset(&scvl, 0, sizeof(scvl));

        for (sc = 0; sc < STL_MAX_SCS; sc++) {
            sl = sm_SCtoSL[sc];
            if (!bitset_test(&sm_linkSLsInuse, sl))
                scvl.SCVLMap[sc].VL = 15;   // mark as invalid
            else
                scvl.SCVLMap[sc].VL = qos->scvl.SCVLMap[sc].VL;
        }

        memcpy(outScvlMap, &scvl, sizeof(STL_SCVLMAP));
    }
#endif

    return VSTATUS_OK;
}

static Status_t
_select_vlvf_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlVfMap_t * vlvfmap)
{
    // Each VL can map to 0 to 32 possible VFs.  If VL-VF unused, set to -1

    // IMPLEMENTATION NOTES - 12/5/2013 - ASB
    // This code is potentially called per port, and should therefore be optimized.
    //   [and not re-evaluated for every port if it yeilds the same result]
    // There are several optimization opportunities:
    //
    // UNIFORM SL-VL MAPPING (Implemented):
    //   For initial implementation in STL G1, only "legacy" SL-VL was done.
    //   This implies that SC-SC is 1-1, and SC-VL is also 1-1
    //    (or SC-VL is constant throughout the fabric)
    //   No SC-VL Expansion Mapping, Contraction Mapping, or Conservation mapping
    //   Thus, there is a single VL-VF map for the entire fabric.
    //   Evaluate once, and copy all other times.
    //
    // VARYING SC-SC or SL-SC / UNIFORM SC-VL MAPPING  (Not implemented)
    //   Here, SC-VL is 1-1 or is constant throughout fabric
    //   No SC-VL Expansion Mapping, Contraction Mapping, or Conservation mapping
    //   However, the SC may be remapped as it moves through a switch.
    //   This may be inconsequential if the SC change results with the same
    //   VL-VF assignments.
    //   Create a limited subset of VL-VF maps as needed, and copy all other times.
    //
    // VARYING SC-SC or SL-SC / VARYING SC-VL MAPPING  (Not implemented)
    //   Same as above, except the SC-VL map may vary due to implementation
    //   of Expansion Mapping, Contraction Mapping, or Conservation mapping
    //   Although such mapping may vary from port to port, however, there may
    //   exist a limited subset of SC-VL mappings, and therefore, a limited
    //   subset of VL-VF.
    //   Create a limited subset of VL-VF maps as needed, and copy all other times.
    //
    // OTHER OPTIMIZATION OPPORTUNITIES MAY EXIST IN THE ROUTING ALGS THEMSELVES.
    //
    // WORST CASE: Evaluate VL-VF for every port. Options:
    //   1. Obtain SL-SC maps and SC-VL maps from topology route algorithms
    //   2. Obtain SL-SC maps and SC-VL maps from topology function pointers
    //   3. Obtain SL-SC maps and SC-VL maps from Port Data structure
    //       - requires such maps to be initialized BEFORE this fn is called.
    //       - currently SC-VL map is not available in Port Data.
    //

    Qos_t *qos = GetQos(portp->portData->vl1);

    Port_t *neighbor_portp = NULL; // Will make a parameter(?)
    bitset_t * vfmember = NULL;
    int vl, vf, idx, idx2;

    // Need neighbor node / port for switches.
    if (portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        if (neighbor_portp==NULL) {
            neighbor_portp=sm_find_port(topop, portp->nodeno, portp->portno);
        }

        if (neighbor_portp!=NULL) {
            if (neighbor_portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH) {
                memcpy (vlvfmap, &qos->vlvf, sizeof(qos->vlvf));
                return VSTATUS_OK; // We are done. No filtering for switch
            } else {
                vfmember = &neighbor_portp->portData->vfMember;
            }
        } else {
            // Null port encountered.  Should never happen.
            memset (vlvfmap, -1, sizeof(*vlvfmap));
            return VSTATUS_BAD;
        }
    } else {
        // HFI - use the ports vf member data.
        vfmember = &portp->portData->vfMember;
    }

    // Default to an empty map.
    memset (vlvfmap, -1, sizeof(*vlvfmap));

    // Should be at least a member of default VF..
    if (bitset_nset(vfmember)==0)
        return VSTATUS_BAD;

    for (vl=0; vl<STL_MAX_VLS; vl++) {
        for (idx=0, idx2=0; idx<MAX_VFABRICS; idx++) {
            vf = qos->vlvf.vf[vl][idx];
            if (vf == -1) break;
            if (bitset_test(vfmember, vf) != 0) {
                vlvfmap->vf[vl][idx2++]=vf;
            }
        }
    }

    //char bfr[256];
    //char * tmp = bfr;
    //tmp+=sprintf(tmp, "Node: %s, Port: %d\n", nodep->nodeDesc.NodeString, portp->index);
    //for (vl=0; vl<STL_MAX_VLS; vl++) {
    //  tmp+=sprintf(tmp, "VL[%d]: ", vl);
    //  for (idx=0, idx2=0; idx<MAX_VFABRICS; idx++) {
    //        if (vlvfmap->vf[vl][idx]==-1) break;
    //      tmp+=sprintf(tmp, "%d ", vlvfmap->vf[vl][idx]);
    //    }
    //    if (vlvfmap->vf[vl][0]!=-1) printf("%s\n", bfr);
    //  tmp=bfr;
    //}

    return VSTATUS_OK;
}

static Status_t
_fill_stl_vlarb_table(Topology_t *topop, Node_t *nodep, Port_t *portp, struct _PortDataVLArb * arbp)
{
	uint8_t		numVls;
	int 		i, j, vf;
    Qos_t *    qos = GetQos(portp->portData->vl1);
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	if (portp->portData->qosHfiFilter) {
		// filtered by hca vf membership, vlarb already setup on slvl mapping call
		return VSTATUS_OK;
	}

	numVls = portp->portData->vl1;
	memset(arbp, 0, sizeof(*arbp));

	if (!VirtualFabrics || !VirtualFabrics->qosEnabled || numVls == 1) {
		sm_FillVlarbTableDefault(portp, arbp);
		return VSTATUS_OK;
	}

	if (portp->portData->portInfo.FlitControl.Interleave.s.MaxNestLevelTxEnabled!=0) {
		VlVfMap_t  vlvfmap;
		uint8_t    vlRank[STL_MAX_VLS];
		// Determine the max rank of a VL
		memset(vlRank, 0, sizeof(vlRank));
		topop->routingModule->funcs.select_vlvf_map(topop, nodep, portp, &vlvfmap);

		for (i=0; i<STL_MAX_VLS; i++) {
			for (j=0;j<MAX_VFABRICS; j++) {
				vf=vlvfmap.vf[i][j];
				if (vf==-1) break;
				if (i >= numVls || i==15) {
					IB_LOG_WARN("Unexpected VF:: Mapping: VL=", i);
					break;
				}
				vlRank[i] = MAX(vlRank[i], VirtualFabrics->v_fabric[vf].preempt_rank); 
			}
		}

		// Iterate through all the VLs, creating the preemption matrix.
		// Note - possibly move this to Qos_t and determine once and done.
		for (i = 0; i < numVls; i++ ) {
			for (j = 0; j < numVls; j++) {
				if ( (vlRank[j]!=0) && (vlRank[i]!=0) && (vlRank[i]>vlRank[j]) ) {
					arbp->vlarbMatrix[i] |= 1 << j;
				}
			}
		}
	}

	return QosFillStlVlarbTable(topop, nodep, portp, qos, arbp);
}

static Status_t
_select_updn_path_lids
	( Topology_t *topop
	, Port_t *srcPortp
	, Port_t *dstPortp
	, uint16_t *outSrcLid
	, uint16_t *outDstLid
	)
{
	return VSTATUS_OK;
}

static Status_t
_select_path_lids
	( Topology_t *topop
	, Port_t *srcPortp, uint16_t slid
	, Port_t *dstPortp, uint16_t dlid
	, uint16_t *outSrcLids, uint8_t *outSrcLen
	, uint16_t *outDstLids, uint8_t *outDstLen
	)
{
	int i;
	int srcLids, dstLids;

	srcLids = 1 << srcPortp->portData->lmc;
	dstLids = 1 << dstPortp->portData->lmc;

	if (slid == PERMISSIVE_LID && dlid == PERMISSIVE_LID) {
		*outSrcLen = srcLids;
		for (i = 0; i < srcLids; ++i)
			outSrcLids[i] = srcPortp->portData->lid + i;
		*outDstLen = dstLids;
		for (i = 0; i < dstLids; ++i)
			outDstLids[i] = dstPortp->portData->lid + i;
	} else if (slid == PERMISSIVE_LID) {
		*outSrcLen = srcLids;
		for (i = 0; i < srcLids; ++i)
			outSrcLids[i] = srcPortp->portData->lid + i;
		*outDstLen = 1;
		outDstLids[0] = dlid;
	} else if (dlid == PERMISSIVE_LID) {
		*outSrcLen = 1;
		outSrcLids[0] = slid;
		*outDstLen = dstLids;
		for (i = 0; i < dstLids; ++i)
			outDstLids[i] = dstPortp->portData->lid + i;
	} else {
		*outSrcLen = 1;
		outSrcLids[0] = slid;
		*outDstLen = 1;
		outDstLids[0] = dlid;
	}

	return VSTATUS_OK;
}

static int
_get_sl_for_path(Topology_t *topop, Node_t *srcNodep, Port_t *srcPortp, uint32_t slid,
                 Node_t *dstNodep, Port_t *dstPortp, uint32_t dlid)
{
	return 0;
}

static Status_t process_swIdx_change(Topology_t * topop, int old_idx, int new_idx, int last_idx)
{
	return VSTATUS_OK;
}

static int _check_switch_path_change(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp)
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

static Status_t
_copy_lfts_ft(Topology_t * src, Topology_t * dest)
{
	return VSTATUS_OK;
}

static Status_t
_init_switch_lfts_ft(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (*routing_needed) {
		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		s = sm_calculate_balanced_lfts_systematic(topop);
		*rebalance = 1;

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

		s = sm_calculate_balanced_lft_deltas(topop);
		*routing_needed = 1;
	} else {
		s = sm_copy_balanced_lfts(topop);
	}

	return s;
}

static Status_t
_init_switch_lfts_sp(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s = VSTATUS_OK;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (*routing_needed) {
		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		// If not, copy and delta updates handled by main topology method.
		s = sm_calculate_all_lfts(topop);
		*rebalance = 1;
	}

	return s;
}

static boolean
_needs_lft_recalc(Topology_t * topop, Node_t * nodep)
{
	if (sm_config.shortestPathBalanced) {
		// For balanced shortest path, LFTs are calculated in bulk in
		// init_switch_lfts method.
		return 0;
	}

	return (topop->routingModule->alg != SM_ROUTE_ALG_FATTREE);
}

static boolean
_can_send_partial_lft_sp(void)
{
	return 0;
}

static boolean
_can_send_partial_lft_ft(void)
{
	return 1;
}

static void
_destroy(Topology_t *topop)
{
}

static Status_t
_load(RoutingModule_t * rm)
{
	return VSTATUS_OK;
}

static Status_t
_unload(RoutingModule_t * rm)
{
	return VSTATUS_OK;
}

static Status_t
_release(RoutingModule_t * rm)
{
	return VSTATUS_OK;
}

static Status_t
_copy(struct _RoutingModule * dest, const struct _RoutingModule * src)
{
    memcpy(dest, src, sizeof(RoutingModule_t));
	return VSTATUS_OK;
}


// Fattree-specific code in _post_process_discovery() also uses this function
// to change the current routing module to shortestpath without having to
// unload the current module.
Status_t
sm_shortestpath_make_routing_module(RoutingModule_t * rm)
{
	rm->name = "shortestpath";
	rm->funcs.pre_process_discovery = _pre_process_discovery;
	rm->funcs.discover_node = _discover_node;
	rm->funcs.discover_node_port = _discover_node_port;
	rm->funcs.post_process_discovery = _post_process_discovery;
	rm->funcs.post_process_routing = _post_process_routing;
	rm->funcs.post_process_routing_copy = _post_process_routing_copy;
	rm->funcs.copy_lfts = sm_routing_copy_lfts;
	rm->funcs.setup_switches_lrdr = _setup_switches_lrdr;
	rm->funcs.get_port_group = _get_port_group;
	rm->funcs.setup_xft = _setup_xft;
	rm->funcs.setup_pgs = _setup_pgs;
	rm->funcs.select_slsc_map = _select_slsc_map;
	rm->funcs.select_scsl_map = _select_scsl_map;
	rm->funcs.select_scvl_map = _select_scvl_map;
	rm->funcs.select_vlvf_map = _select_vlvf_map;
	rm->funcs.fill_stl_vlarb_table = _fill_stl_vlarb_table;
	rm->funcs.select_path_lids = _select_path_lids;
	rm->funcs.select_updn_path_lids = _select_updn_path_lids;
	rm->funcs.get_sl_for_path = _get_sl_for_path;
	rm->funcs.process_swIdx_change = process_swIdx_change;
	rm->funcs.check_switch_path_change = _check_switch_path_change;
	rm->funcs.needs_lft_recalc = _needs_lft_recalc;
	rm->funcs.can_send_partial_lft = _can_send_partial_lft_sp;
	rm->funcs.destroy = _destroy;
    rm->load = _load;
    rm->unload = _unload;
    rm->release = _release;
    rm->copy = _copy;

	if (sm_config.shortestPathBalanced) {
		rm->funcs.init_switch_lfts = _init_switch_lfts_sp;
	}

	rm->alg = SM_ROUTE_ALG_SHORTESTPATH;

	return VSTATUS_OK;
}

static Status_t
_make_fattree(RoutingModule_t * rm)
{
	Status_t s;
	s = sm_shortestpath_make_routing_module(rm);
	if (s == VSTATUS_OK) {
		rm->name = "fattree";
		rm->funcs.copy_lfts = _copy_lfts_ft;
		rm->funcs.init_switch_lfts = _init_switch_lfts_ft;
		rm->funcs.can_send_partial_lft = _can_send_partial_lft_ft;
		rm->alg = SM_ROUTE_ALG_FATTREE;
	}

	return s;
}

Status_t
sm_shortestpath_init(Topology_t *topop)
{
	Status_t s;
	s = sm_routing_addModuleFac("shortestpath", sm_shortestpath_make_routing_module);
	if (s == VSTATUS_OK)
		s = sm_routing_addModuleFac("fattree", _make_fattree);

	return s;
}
