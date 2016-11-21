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
#include "sm_dor.h"


//pointer to block of memory treated as a two-dimenstional array of size PORT_PAIR_WARN_SIZE * PORT_PAIR_WARN_SIZE
//to keep track of number of warnings for each port pair combination.
uint8_t *port_pair_warnings;
uint8_t incorrect_ca_warnings = 0;
uint8_t invalid_isl_found = 0;

//===========================================================================//
// DEBUG ROUTINES
//

static int
_coord_to_string(Topology_t *topop, int8_t *c, char *str)
{
	uint8_t i, l, n = 0;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	l = MIN(dorTop->numDimensions, SM_DOR_MAX_DIMENSIONS);
	n += sprintf(str, "(");
	for (i = 0; i < l; ++i) {
		n += sprintf(str + n, "%d", c[i]);
		if (i < l - 1)
			n += sprintf(str + n, ",");
	}
	n += sprintf(str + n, ")");
	return n;
}

static void
_dump_node_coordinates(Topology_t *topop)
{
	char c[32];
	Node_t *nodep;

	for_all_switch_nodes(topop, nodep) {
		memset((void *)c, 0, sizeof(c));
		_coord_to_string(topop, ((DorNode_t*)nodep->routingData)->coords, c);
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "NodeGUID "FMT_U64" [%s]: %s",
		       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), c);
	}
}

//===========================================================================//
// TOPOLOGY VALIDATION ROUTINES
//

static void
_verify_connectivity(Topology_t *topop)
{
	Node_t *switchp;
	int i, brokenDim;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;
	DorNode_t *switchDnp;

	for_all_switch_nodes(topop, switchp) {
		switchDnp = (DorNode_t*)switchp->routingData;
		brokenDim = 0;
		for (i = 0; i < dorTop->numDimensions; ++i) {
			if (dorTop->toroidal[i]) {
				if (switchDnp->right[i] && switchDnp->left[i]) continue;
			} else {
				if ((switchDnp->coords[i] == 0) && switchDnp->right) continue;
				if ((switchDnp->coords[i] == (dorTop->dimensionLength[i]-1)) && switchDnp->left) continue;
				if (switchDnp->right[i] && switchDnp->left[i]) continue;
			}
			brokenDim++;
			if (brokenDim > 1) 
				break;
		}

		if (brokenDim > 1)  {
			switchDnp->multipleBrokenDims = 1;
			IB_LOG_INFINI_INFO_FMT(__func__,
				"Switch "FMT_U64" [%s] broken in multiple dimensions",
				switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));
		}
	}
}

static void
_verify_coordinates(Topology_t *topop)
{
	char c1[32] = {0};
	char c2[32] = {0};
	Node_t *switchp, *oldnodep;
	int i;

	if (topology_passcount == 0 || !topology_changed) return;

	for_all_switch_nodes(topop, switchp) {
		oldnodep = switchp->old;
		if (!oldnodep)
			oldnodep = sm_find_guid(&old_topology, switchp->nodeInfo.NodeGUID);
		if (!oldnodep) continue;

		for (i = 0; i < smDorRouting.dimensionCount; i++) {
			if (((DorNode_t*)switchp->routingData)->coords[i] ==
				((DorNode_t*)oldnodep->routingData)->coords[i]) continue;

			_coord_to_string(topop, ((DorNode_t*)switchp->routingData)->coords, c1);
			_coord_to_string(topop, ((DorNode_t*)oldnodep->routingData)->coords, c2);

			IB_LOG_INFINI_INFO_FMT(__func__,
				"Coordinate Change: NodeGUID "FMT_U64" [%s]: old %s new %s",
				switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), c2, c1);
			break;
		}
	}
}

static void
_verify_dimension_lengths(Topology_t *topop)
{
	Node_t *switchp;
	int i;
	DorTopology_t *dorTop = (DorTopology_t *)topop->routingModule->data;
	DorNode_t *switchDnp;
	uint8_t flaggedDims = 0;

	for_all_switch_nodes(topop, switchp) {
		switchDnp = (DorNode_t*)switchp->routingData;

		for (i = 0; i < dorTop->numDimensions; ++i) {
			// only measure toroidal dimension starting from switch at 0 coordinate
			// also skip dimensions we have already warned about
			if ((flaggedDims && (1 << i)) || switchDnp->coords[i] != 0 || dorTop->toroidal[i] == 0) continue;

			uint8_t expectedLength = dorTop->dimensionLength[i];
			uint8_t measuredLength = 1;

			DorNode_t *nextSwitch = switchDnp->right[i];
			while(nextSwitch && nextSwitch != switchDnp) {
				measuredLength++;
				nextSwitch = nextSwitch->right[i];
			}
			// if nextSwitch != switchDnp then it was a broken dimension, don't measure
			if (nextSwitch == switchDnp && expectedLength != measuredLength) {
				flaggedDims |= (uint8_t)(1 << i);
				IB_LOG_WARN_FMT(__func__,
					"Dimension length does not match configured length; Actual: %d  Configured: %d",
					measuredLength, expectedLength);
			}
		}
	}

}

//===========================================================================//
// GENERAL UTILITIES
//

static Node_t *
_get_switch(Topology_t *topop, Node_t *nodep, Port_t *portp)
{
	if (nodep == NULL || nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
		return nodep;

	nodep = sm_find_node(topop, portp->nodeno);
	if (nodep == NULL || nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
		return NULL;

	return nodep;
}

// Extracts the dimension being traversed between two nodes.
// Assumes the two nodes are initialized neighbors.
//
static void
_find_dimension_difference(Node_t *nodep, Node_t *neighborNodep,
	uint8_t *dimension, int8_t *direction)
{
	int i, diff;
	DorNode_t *dnodep = (DorNode_t*)nodep->routingData;
	DorNode_t *ndnodep = (DorNode_t*)neighborNodep->routingData;

	for (i = 0; i < SM_DOR_MAX_DIMENSIONS; i++) {
		diff = ndnodep->coords[i] - dnodep->coords[i];
		if (diff != 0) {
			*dimension = i;
			if (diff == 1)       // forward edge
				*direction = 1;
			else if (diff > 1)   // backward wrap-around edge
				*direction = -1;
			else if (diff == -1) // backward edge
				*direction = -1;
			else if (diff < -1)  // forward wrap-around edge
				*direction = 1;
			return;
		}
	}

	*dimension = 0;
	*direction = 0;
}

inline int port_pair_needs_warning(uint8_t p1, uint8_t p2)
{
	int idx = 0;

	if (!port_pair_warnings)
		return 1;

	if ((p1 > PORT_PAIR_WARN_ARR_SIZE) || (p2 > PORT_PAIR_WARN_ARR_SIZE))
		return 1;

	idx = PORT_PAIR_WARN_IDX(p1, p2);

	if (port_pair_warnings[idx] < smDorRouting.warn_threshold)	{
		port_pair_warnings[idx]++;
		return 1;
	} else {
		return 0;
	}

}

//===========================================================================//
// DOR COORDINATE ASSIGNMENT
//

static Status_t
_create_dimension(DorDiscoveryState_t *state, uint8_t p, uint8_t q, DorDimension_t **outDim)
{
	Status_t status;
	DorDimension_t *dim;
	int i, j;
	int8_t direction = 0;

	// add forward direction
	status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
		return status;
	}

	for (i = 0; i < smDorRouting.dimensionCount; i++) {
		for (j = 0; j < smDorRouting.dimension[i].portCount; j++) {
			if (p == smDorRouting.dimension[i].portPair[j].port1) {
				direction = 1;
				break;
			} else if (p == smDorRouting.dimension[i].portPair[j].port2) {
				direction = -1;
				break;
			}
		}
	}

	if (!direction) {
		// never found p in a configured port pair
		IB_LOG_ERROR_FMT(__func__, "Failed to find port %d in configured port pairs", p);
		return VSTATUS_BAD;
	}

	dim->ingressPort = q;
	dim->dimension = state->nextDimension;
	dim->direction = direction;
	dim->hyperlink = (p==q);
	state->dimensionMap[p] = dim;

	*outDim = dim;

	if (p != q) {
		// add reverse direction, unless it's a hyperlink
		status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
			return status;
		}

		dim->ingressPort = p;
		dim->dimension = state->nextDimension;
		dim->direction = -1 * direction;
		dim->hyperlink = 0;
		state->dimensionMap[q] = dim;
	}

	++state->nextDimension;
	if (state->nextDimension > SM_DOR_MAX_DIMENSIONS) {
		IB_LOG_ERROR("Maximum number of DOR dimensions exceeded; invalid topology. limit:", SM_DOR_MAX_DIMENSIONS);
		return VSTATUS_BAD;
	}

	return VSTATUS_OK;
}

static Status_t
_extend_dimension(DorDiscoveryState_t *state, uint8_t p, uint8_t q,
	uint8_t dimension, int8_t direction, DorDimension_t **outDim)
{
	Status_t status;
	DorDimension_t *dim;

	status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
		return status;
	}

	dim->ingressPort = q;
	dim->dimension = dimension;
	dim->direction = direction;
	dim->hyperlink = (p==q);
	state->dimensionMap[p] = dim;

	*outDim = dim;

	if (p != q) {
		// add reverse direction, unless it's a hyperlink
		status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
			return status;
		}

		dim->ingressPort = p;
		dim->dimension = dimension;
		dim->direction = -direction;
		dim->hyperlink = 0;
		state->dimensionMap[q] = dim;
	}
	return VSTATUS_OK;
}

static DimLookupRval_t
_lookup_dimension(DorDiscoveryState_t *state, uint8_t p, uint8_t q, DorDimension_t **outDim)
{
	if (state->dimensionMap[p] == NULL)
		return DIM_LOOKUP_RVAL_NOTFOUND;
	
	if (state->dimensionMap[p]->ingressPort != q)
		return DIM_LOOKUP_RVAL_INVALID;

	*outDim = state->dimensionMap[p];
	return DIM_LOOKUP_RVAL_FOUND;
}

// Returns 0 if we've maxed on the number of toroidal dimensions
// available.  1 if we successfully marked it.
static int
_mark_toroidal_dimension(DorDiscoveryState_t *state, Topology_t *topop, uint8_t dimension)
{
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (dorTop->numToroidal > SM_DOR_MAX_TOROIDAL_DIMENSIONS) {
		state->toroidalOverflow = 1;
		return 0;
	}

	dorTop->toroidal[dimension] = 1;
	dorTop->toroidalMap[dimension] = dorTop->toroidal_count++;

	return 1;
}


/* This function assumes that ports p1 and p2 are part of the configured dimension config_dim.
 * It also assumes that the dimension has been created.
 */
static int _get_dimension_and_direction(DorDiscoveryState_t *state, int config_dim, int p1, int p2, uint8_t * dimension, int8_t *direction)
{
	int i, idx = 0;
	DorDimension_t *dim = NULL;
	DimLookupRval_t rval = DIM_LOOKUP_RVAL_NOTFOUND;

	for (i = 0; i < smDorRouting.dimension[config_dim].portCount; i++) {
		if (p1 == smDorRouting.dimension[config_dim].portPair[i].port1) {
			idx = 1;
			break;
		} else if (p1 == smDorRouting.dimension[config_dim].portPair[i].port2) {
			idx = 2;
			break;
		}
	}


	for (i = 0; i < smDorRouting.dimension[config_dim].portCount; i++) {
		if (idx == 1) {
			rval = _lookup_dimension(state, smDorRouting.dimension[config_dim].portPair[i].port1,
								 smDorRouting.dimension[config_dim].portPair[i].port2, &dim);
		} else if (idx == 2) {
			rval = _lookup_dimension(state, smDorRouting.dimension[config_dim].portPair[i].port2,
								 smDorRouting.dimension[config_dim].portPair[i].port1, &dim);
		}
		if (rval == DIM_LOOKUP_RVAL_FOUND) {
			*dimension = dim->dimension;
			*direction = dim->direction;	
			return 1;	
		}
	}
	return 0;
}

static int
is_configured_toroidal(int p1, int p2)
{
	int i, j;

	for (i = 0; i < smDorRouting.dimensionCount; i++) {
		if (!smDorRouting.dimension[i].toroidal)
			continue;
		for (j = 0; j < smDorRouting.dimension[i].portCount; j++) {
			if (((p1 == smDorRouting.dimension[i].portPair[j].port1) &&
				(p2 == smDorRouting.dimension[i].portPair[j].port2)) ||
				((p1 == smDorRouting.dimension[i].portPair[j].port2) &&
				(p2 == smDorRouting.dimension[i].portPair[j].port1)))
				return 1;
		}
	}

	return 0;
}

static int
get_configured_dimension(int p1, int p2)
{
	int i, j;
	int l = MIN(MAX_DOR_DIMENSIONS, smDorRouting.dimensionCount);

	for (i = 0; i < l; i++) {
		for (j = 0; j < smDorRouting.dimension[i].portCount; j++) {
			if (((p1 == smDorRouting.dimension[i].portPair[j].port1) &&
				(p2 == smDorRouting.dimension[i].portPair[j].port2)) ||
				((p1 == smDorRouting.dimension[i].portPair[j].port2) &&
				(p2 == smDorRouting.dimension[i].portPair[j].port1)))
				return i;
		}
	}

	return -1;
}

static int
get_configured_dimension_for_port(int p)
{
	int i, j;

	for (i = 0; i < smDorRouting.dimensionCount; i++) {
		for (j = 0; j < smDorRouting.dimension[i].portCount; j++) {
			if ((p == smDorRouting.dimension[i].portPair[j].port1) ||
				(p == smDorRouting.dimension[i].portPair[j].port2))
				return i;
		}
	}

	return -1;
}

static int
get_configured_port_pos_in_dim(int d, int p)
{
	int j;

	for (j = 0; j < smDorRouting.dimension[d].portCount; j++) {
		if (p == smDorRouting.dimension[d].portPair[j].port1)
			return 1;
		if (p == smDorRouting.dimension[d].portPair[j].port2)
			return 2;
	}	

	return -1;
}

static Status_t
_propagate_coord_through_port(DorDiscoveryState_t *state,
	Topology_t *topop, Node_t *nodep, Port_t *portp)
{
	Status_t status;
	Node_t *neighborNodep;
	DorDimension_t *dim = NULL;
	DimLookupRval_t rval;
	uint8_t dimension;
	int8_t direction = 0;
	int config_dim;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	neighborNodep = sm_find_node(topop, portp->nodeno);
	if (neighborNodep == NULL) {
		IB_LOG_ERROR0("Failed to find neighbor node");
		return VSTATUS_BAD;
	}

	if (neighborNodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
		return VSTATUS_OK;

	// found an ISL, update SC support
	if (portp->portData->vl0 < state->scsAvailable)
		state->scsAvailable = portp->portData->vl0;

	if (state->scsAvailable < dorTop->minReqScs) {
		IB_LOG_ERROR_FMT(__func__,
				"NodeGUID "FMT_U64" [%s] Port %d has only %d SC(s). "
				"Minimum required SCs for ISLs for given DOR configuration is %d.",
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
				state->scsAvailable, dorTop->minReqScs);
		return VSTATUS_BAD;
	}

	// determine the dimension of this link
	rval = _lookup_dimension(state, portp->index, portp->portno, &dim);
	switch (rval) {
	case DIM_LOOKUP_RVAL_FOUND:
		// success
		break;
	case DIM_LOOKUP_RVAL_NOTFOUND:
		// new port mapping found; add it to the list of known mappings
		if (neighborNodep->routingData != NULL) {
			// we've seen this node before; not a new dimension, but
			// instead a redundant link
			_find_dimension_difference(nodep, neighborNodep, &dimension, &direction);
			status = _extend_dimension(state, portp->index, portp->portno, dimension, direction, &dim);
			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
				       "Failed to extend dimension %d, direction %d in map between "
				       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d; rc: %d",
				       dimension, direction,
				       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
				       neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno,
				       status);
				return status;
			}
		} else {
			// Before creating a new dimension, first check to see if this port mapping is part of a dimension
			// which has already been created.
			config_dim = get_configured_dimension(portp->index, portp->portno);

			if (config_dim >=0) {
				if (smDorRouting.dimension[config_dim].created) {
					if (_get_dimension_and_direction(state, config_dim, portp->index, portp->portno, &dimension, &direction)) {
						status = _extend_dimension(state, portp->index, portp->portno, dimension, direction, &dim);
						if (status != VSTATUS_OK) {
							IB_LOG_ERROR_FMT(__func__,
							       "Failed to extend dimension %d, direction %d in map between "
							       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d; rc: %d",
							       dimension, direction,
							       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
							       neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno,
							       status);
							return status;
						}
						break;
					} else {
						IB_LOG_ERROR_FMT(__func__,
						       "Failed to find created dimension which corresponds to port pair %d %d while traversing link between "
						       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d",
						       portp->index, portp->portno,
						       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
					    	   neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
						return VSTATUS_BAD;
					}
				}
			}

			// never seen before; new dimemsion
			status = _create_dimension(state, portp->index, portp->portno, &dim);
			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
				       "Failed to create dimension in map between "
				       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d; rc: %d",
				       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
				       neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno,
				       status);
				return status;
			}

			//check if this dimension is configured as toroidal
			if (is_configured_toroidal(portp->index, portp->portno)) {
				_mark_toroidal_dimension(state, topop, dim->dimension);
			}

			dorTop->dimensionLength[dim->dimension] = smDorRouting.dimension[dim->dimension].length;

			dorTop->coordMinimums[dim->dimension] = !dorTop->toroidal[dim->dimension] ? 0 : (0 -
										(dorTop->dimensionLength[dim->dimension] / 2) +
										(dorTop->dimensionLength[dim->dimension]%2 ? 0 : 1));

			dorTop->coordMaximums[dim->dimension] = !dorTop->toroidal[dim->dimension] ? 
										(dorTop->dimensionLength[dim->dimension] - 1) :
										(dorTop->dimensionLength[dim->dimension] / 2);

			if (smDorRouting.debug)
				IB_LOG_INFINI_INFO_FMT(__func__,
						"Dimension %d length %d coordMinimum %d coordMaximum %d",
						dim->dimension, dorTop->dimensionLength[dim->dimension],
						dorTop->coordMinimums[dim->dimension], dorTop->coordMaximums[dim->dimension]);

			//mark config information that the dimension has been created
			config_dim = get_configured_dimension(portp->index, portp->portno);
			if (config_dim >= 0) {
				smDorRouting.dimension[config_dim].created = 1;
			}
		}
		break;
	case DIM_LOOKUP_RVAL_INVALID:
		// egress port was found, but ingress port on the other side is
		// not what was expected based on a previously discovered mapping
		IB_LOG_ERROR_FMT(__func__,
		       "NodeGUID "FMT_U64" [%s] Port %d is not cabled consistently with the rest of the fabric",
		       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index);
		return VSTATUS_BAD;
		break;
	}

	// init node if this is the first time we've seen it
	DorNode_t *neighborDorNode = NULL;
	DorNode_t *dorNode = (DorNode_t*)nodep->routingData;
	if (neighborNodep->routingData == NULL) {
		status = vs_pool_alloc(&sm_pool, sizeof(DorNode_t), &neighborNodep->routingData);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to allocate storage for DOR node structure; rc:", status);
			neighborNodep->routingData = NULL;
			return status;
		}
		neighborDorNode = (DorNode_t*)neighborNodep->routingData;
		memset(neighborNodep->routingData, 0, sizeof(DorNode_t));
		neighborDorNode->node = neighborNodep;

		// copy over existing coordinates and increment the dimension
		// we traveled along
		memcpy((void *)neighborDorNode->coords, (void *)dorNode->coords, sizeof(dorNode->coords));
		neighborDorNode->coords[dim->dimension] += dim->direction;

		if (dorTop->toroidal[dim->dimension]) {
			if (neighborDorNode->coords[dim->dimension] > dorTop->coordMaximums[dim->dimension]) 
				neighborDorNode->coords[dim->dimension] -= dorTop->dimensionLength[dim->dimension];
			else if (neighborDorNode->coords[dim->dimension] < dorTop->coordMinimums[dim->dimension])
				neighborDorNode->coords[dim->dimension] += dorTop->dimensionLength[dim->dimension];
		}

	} else {
		// the neighbor has an existing coordinate, check for a wrap-around edge
		neighborDorNode = (DorNode_t*)neighborNodep->routingData;
		if (dim->hyperlink) {
			if (is_configured_toroidal(portp->index, portp->portno)) {
				IB_LOG_WARN_FMT(__func__, "Hyperlink configured as toroidal between "
	 					        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
								" ingoring config, it cannot be toroidal",
				 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	               			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
			}
		} else if (dim->direction > 0) {
			if (dorNode->coords[dim->dimension] == dorTop->coordMaximums[dim->dimension] &&
			    neighborDorNode->coords[dim->dimension] == dorTop->coordMinimums[dim->dimension]) {
				if (is_configured_toroidal(portp->index, portp->portno)) {
					if (smDorRouting.debug) {
						IB_LOG_VERBOSE_FMT(__func__,
						       "Found toroidal link for dimension %d in (direction %d, coord bounds [%d, %d]) between "
						       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d",
						       dim->dimension, dim->direction,
					    	   dorTop->coordMinimums[dim->dimension],
						       dorTop->coordMaximums[dim->dimension],
						       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
						       neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
					}
				} else {
					if (port_pair_needs_warning(portp->index, portp->portno)) {
						IB_LOG_WARN_FMT(__func__, "Disabling toroidal link between "
	 							        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
										" as the dimension has not been configured as toroidal",
						 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	                				    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
					}
					sm_mark_link_down(topop, portp);
					topology_changed = 1;
				}
			} else if ((neighborDorNode->coords[dim->dimension] - dorNode->coords[dim->dimension]) > 1) {
				/* This is a wrap around link that is not between the maximum and minimum co-ordinates */
				IB_LOG_WARN_FMT(__func__, "Disabling wrap around link between "
	 					        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
								" as this link is not between the end nodes of this dimension",
				 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	               			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);

				sm_mark_link_down(topop, portp);
				topology_changed = 1;
			}
		} else {
			if (dorNode->coords[dim->dimension] == dorTop->coordMinimums[dim->dimension] &&
			    neighborDorNode->coords[dim->dimension] == dorTop->coordMaximums[dim->dimension]) {
				if (is_configured_toroidal(portp->index, portp->portno)) {
					if (smDorRouting.debug) {
						IB_LOG_VERBOSE_FMT(__func__,
						       "Found toroidal link for dimension %d in (direction %d, coord bounds [%d, %d]) between "
						       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d",
						       dim->dimension, dim->direction,
					    	   dorTop->coordMinimums[dim->dimension],
						       dorTop->coordMaximums[dim->dimension],
						       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
						       neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
					}
				} else {
					if (port_pair_needs_warning(portp->index, portp->portno)) {
						IB_LOG_WARN_FMT(__func__, "Disabling toroidal link between "
		 						        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
										" as the dimension has not been configured as toroidal",
						 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	            	    			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
					}
					sm_mark_link_down(topop, portp);
					topology_changed = 1;
				}
			} else if ((dorNode->coords[dim->dimension] - neighborDorNode->coords[dim->dimension]) > 1) {
				/* This is a wrap around link that is not between the maximum and minimum co-ordinates */
				IB_LOG_WARN_FMT(__func__, "Disabling wrap around link between "
	 					        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
								" as this link is not between the end nodes of this dimension",
				 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	               			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
				sm_mark_link_down(topop, portp);
				topology_changed = 1;
			}
		}
	}

	// update neighbor pointers for every link we find
	if (dim->hyperlink) {
		_find_dimension_difference(nodep, neighborNodep, &dimension, &direction);
		if (direction > 0) {
			dorNode->right[dim->dimension] = neighborDorNode;
			neighborDorNode->left[dim->dimension] = dorNode;
		} else {
			dorNode->left[dim->dimension] = neighborDorNode;
			neighborDorNode->right[dim->dimension] = dorNode;
		}
	} else if (dim->direction > 0) {
		dorNode->right[dim->dimension] = neighborDorNode;
		neighborDorNode->left[dim->dimension] = dorNode;
	} else {
		dorNode->left[dim->dimension] = neighborDorNode;
		neighborDorNode->right[dim->dimension] = dorNode;
	}

	return VSTATUS_OK;
}

//===========================================================================//
// DOR ROUTING ROUTINES
//

static int
_is_path_realizable(Topology_t *topop, Node_t *src, Node_t *dst, DorDirection dir) {

	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;
	DorNode_t *srcDnp = (DorNode_t*)src->routingData;
	DorNode_t *dstDnp = (DorNode_t*)dst->routingData;
	int i, si = -1, di = -1;
	int	goRight = 0;
	int goLeft = 0;
	int ij = DorBitMapsIndex(src->swIdx, dst->swIdx);

	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
			"Entry [%s] to [%s] dir %s",
		   	sm_nodeDescString(src), sm_nodeDescString(dst), dir == DorAny ? "Any" : (dir == DorRight ? "Right" : "Left"));

	if (src == dst) {
		// reached destination
		return 1;
	}

	if (ijTest(dorTop->dorLeft, ij) || ijTest(dorTop->dorRight, ij)) {
		// been here before
		return 1;
	}

	// Find first dimension which does not share a path
	// and check if neighbors are realizable.
	for (i = 0; i < dorTop->numDimensions; ++i) {
		si = srcDnp->coords[i];
		di = dstDnp->coords[i];

		if (si == di) continue;

		if (dir == DorAny) {
			if (si < di) {
				// source has smaller index; would normally go right
				if (dorTop->toroidal[i] &&
					(si + dorTop->dimensionLength[i] - di <= di - si)) {
					goLeft = 1;
					if (si + dorTop->dimensionLength[i] - di == di - si) {
						goRight = 1;
					}
				} else {
					goRight = 1;
				}

			} else {
				// source has larger index; would normally go left
				if (dorTop->toroidal[i] &&
					(di - si + dorTop->dimensionLength[i] <= si - di)) {
					goRight = 1;
					if (di - si + dorTop->dimensionLength[i] == si - di) {
						goLeft = 1;
					}
				} else {
					goLeft = 1;
				}
			}
		} else if (dir == DorRight) {
			goRight = 1;
		} else {
			goLeft = 1;
		}

		if (goRight) {
			// continue right until we reach next dimension, then use any dir
			if (srcDnp->right[i] && 
				(srcDnp->right[i] == dstDnp || 
				 (!srcDnp->right[i]->multipleBrokenDims &&
				  _is_path_realizable(topop, srcDnp->right[i]->node, dst, srcDnp->right[i]->coords[i] == di ? DorAny : DorRight)))) {

				ijSet(dorTop->dorRight, ij);

			} else if (dir == DorAny && !goLeft && srcDnp->left[i]) {
				// try a longer path
				IB_LOG_INFINI_INFO_FMT(__func__,
							"Path from %d [%s] to %d [%s] has broken toroidal link",\
							src->swIdx, sm_nodeDescString(src), dst->swIdx, sm_nodeDescString(dst));

				if (!srcDnp->left[i]->multipleBrokenDims &&
					_is_path_realizable(topop, srcDnp->left[i]->node, dst, DorLeft)) {
					ijSet(dorTop->dorLeft, ij);
				}
				break;
			}
		}

		if (goLeft) {
			// continue left until we reach next dimension, then use any dir
			if (srcDnp->left[i] && 
				(srcDnp->left[i] == dstDnp ||
				(!srcDnp->left[i]->multipleBrokenDims &&
				 _is_path_realizable(topop, srcDnp->left[i]->node, dst, srcDnp->left[i]->coords[i] == di ? DorAny : DorLeft)))) {
				ijSet(dorTop->dorLeft, ij);

			} else if (dir == DorAny && !goRight && srcDnp->right[i]) {
				// try a longer path
				if (smDorRouting.debug)
					IB_LOG_INFINI_INFO_FMT(__func__,
							"Path from %d [%s] to %d [%s] has broken toroidal link",\
							src->swIdx, sm_nodeDescString(src), dst->swIdx, sm_nodeDescString(dst));
				if (!srcDnp->right[i]->multipleBrokenDims &&
					_is_path_realizable(topop, srcDnp->right[i]->node, dst, DorRight)) {
					ijSet(dorTop->dorRight, ij);
				}
			}
		}
		break;
	}

	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
			"Path from %d [%s] to %d [%s] routes left %d right %d",
		   	src->swIdx, sm_nodeDescString(src), dst->swIdx, sm_nodeDescString(dst),
			ijTest(dorTop->dorLeft, ij), ijTest(dorTop->dorRight, ij));

	return (ijTest(dorTop->dorLeft, ij) || ijTest(dorTop->dorRight, ij));
}

static int
_get_alternate_path_port_group(Topology_t *topop, Node_t *src, Node_t *dst, uint8_t *portnos)
{
	int				i,j,k;
	int				port, end_port = 0;
	int				brokenPath = 0;
	Node_t			*next_nodep;
	Port_t			*portp;
	uint16_t		best_cost = 0xffff;

	i = src->swIdx;
	j = dst->swIdx;
	best_cost = topop->cost[Index(i, j)];

	// No DOR closure, use shortestpath switch for next hop 
	// TBD: route around closer to failure, verify limited disruption
	for (port = 1; port <= src->nodeInfo.NumPorts; port++) {
		portp = sm_get_port(src, port);

		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;

		next_nodep = sm_find_node(topop, portp->nodeno);
		if (next_nodep == NULL ||
			next_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
			continue;

		k = next_nodep->swIdx;
		if (i == k) continue; // avoid loopback links

		if (topop->cost[Index(i, k)] + topop->cost[Index(k, j)] == best_cost) {
			// Only use broken path if only path
			if (next_nodep != dst) {
				if (((DorNode_t*)next_nodep->routingData)->multipleBrokenDims) {
					if (!brokenPath && end_port > 0) continue;
					brokenPath = 1;
				} else if (brokenPath) {
					brokenPath = 0;
					end_port = 0;
				}
			}
			portnos[end_port] = port;
			++end_port;
		}
	}

	return end_port;
}

//===========================================================================//
// DOR CLOSURE CALCULATION
//

static Status_t
_calc_dor_closure(Topology_t *topop)
{
	Status_t status;
	size_t s;
	Node_t *ni, *nj;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	s = (sizeof(uint32_t) * topop->max_sws * topop->max_sws / 32) + 1;
	dorTop->dorClosureSize = s;

	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTop->dorLeft);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}
	memset((void *)dorTop->dorLeft, 0, s);

	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTop->dorRight);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}
	memset((void *)dorTop->dorRight, 0, s);

	// Find paths which are valid - DOR closure exists
	for_all_switch_nodes(topop, ni) {
		for_all_switch_nodes(topop, nj) {
			if (ni == nj || _is_path_realizable(topop, ni, nj, DorAny)) continue;
			if (smDorRouting.debug)
				IB_LOG_WARN_FMT(__func__,
						"Path from %d [%s] to %d [%s] is NOT realizable",
						ni->swIdx, sm_nodeDescString(ni), nj->swIdx, sm_nodeDescString(nj));
		}
	}
	return VSTATUS_OK;
}

static Status_t
_copy_dor_closure(Topology_t *src_topop, Topology_t *dst_topop)
{
	Status_t status;
	DorTopology_t	*dorTopSrc = (DorTopology_t *)src_topop->routingModule->data;
	DorTopology_t	*dorTopDst = (DorTopology_t *)dst_topop->routingModule->data;
	size_t s, max_sws = dorTopSrc->closure_max_sws;

	dorTopDst->closure_max_sws= max_sws;
	s = (sizeof(uint32_t) * max_sws * max_sws / 32) + 1;

	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTopDst->dorLeft);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}
	memcpy((void *)dorTopDst->dorLeft, (void *)dorTopSrc->dorLeft, s);

	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTopDst->dorRight);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}
	memcpy((void *)dorTopDst->dorRight, (void *)dorTopSrc->dorRight, s);

	return VSTATUS_OK;
}

static __inline__ boolean
isDatelineSwitch(Topology_t *topop, Node_t *switchp, uint8_t dimension)
{
	return (((DorTopology_t *)topop->routingModule->data)->toroidal[dimension] &&
			((DorNode_t*)switchp->routingData)->coords[dimension] == 0);
}

//===========================================================================//
// DOR QOS ROUTINES
//

// TBD: is the following needed.
static __inline__ void
get_hca_qos_stl_setup(Topology_t *topop, Node_t *nodep, Port_t *portp, Node_t *switchp, Port_t *swportp) {
	int			vf;
	int			numVls = portp->portData->vl1;
	Qos_t		qos;
	STL_VLARB_TABLE_ELEMENT *vlblockp;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	qos.numVLs = numVls;

	if (numVls < 2) {
		struct _PortDataVLArb * arbp;
		if (switchp) {
			arbp = sm_port_getNewArb(swportp);
			if (!arbp) return;
			swportp->portData->qosHfiFilter = 1;
		} else {
			arbp = sm_port_getNewArb(portp);
			if (!arbp) return;
			portp->portData->qosHfiFilter = 1;
		}

		memset(arbp, 0, sizeof(*arbp));

		vlblockp = arbp->vlarbLow;
		vlblockp[0].Weight = 255;
		return;
	}

	if (!bitset_init(&sm_pool, &qos.highPriorityVLs, MAX_VIRTUAL_LANES) ||
		!bitset_init(&sm_pool, &qos.lowPriorityVLs, MAX_VIRTUAL_LANES)) {
		IB_FATAL_ERROR("get_hca_qos_setup: No memory for slvl setup, exiting.");
	}

	memset(qos.vlBandwidth, 0, sizeof(uint8_t)*MAX_VIRTUAL_LANES);

	for (vf=0; vf<VirtualFabrics->number_of_vfs; vf++) {
		uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;
		if (bitset_test(&portp->portData->vfMember, vfIdx)) {
			if (VirtualFabrics->v_fabric[vf].priority == 1) {
				bitset_set(&qos.highPriorityVLs, numVls-1);
			}
		}
	}
	
	if (switchp) {
		swportp->portData->qosHfiFilter = 1;
		QosFillStlVlarbTable(topop, switchp, swportp, &qos, sm_port_getNewArb(swportp));
	} else {
		portp->portData->qosHfiFilter = 1;
		QosFillStlVlarbTable(topop, nodep, portp, &qos, sm_port_getNewArb(portp));
	}

	bitset_free(&qos.highPriorityVLs);
	bitset_free(&qos.lowPriorityVLs);
}

static Status_t
_generate_scsc_map(Topology_t *topop, Node_t *switchp, int getSecondary, int *numBlocks, STL_SCSC_MULTISET** scscmap)
{
	int i,j, dimension, p1, p2;
	Port_t * ingressPortp = NULL;
	Port_t * egressPortp = NULL;
	int firstSC, secondSC, setsComplete;

	STL_SCSC_MULTISET *scsc=NULL;
	STL_SCSCMAP scscNoChg;
	STL_SCSCMAP	scscPlus1;
	STL_SCSCMAP	scsc0;
	STL_SCSCMAP	scscBadTurn;

	uint8_t	portDim[switchp->nodeInfo.NumPorts+1];
	uint8_t	portPos[switchp->nodeInfo.NumPorts+1];

	DorTopology_t *dorTop = (DorTopology_t *)topop->routingModule->data;

	int portToSet = 0;
	int needsSet = 0;

	boolean datelineSwitch;

	int changeDimBlk, crossMeridianBlk, sameDimBlk, illegalTurn;
	int curBlock = 0;

	*numBlocks = 0;

	if ((topology_passcount && !topology_switch_port_changes) || getSecondary) 
		return VSTATUS_OK;

	memset(portDim, 0xff, sizeof(uint8_t)*(switchp->nodeInfo.NumPorts+1));
	memset(portPos, 0, sizeof(uint8_t)*(switchp->nodeInfo.NumPorts+1));

	// Setup illegal turn to drop or (if escape VLs in use) to start at 2nd set of SCs
	memset(&scscBadTurn, 15, sizeof(scscBadTurn));
	if (smDorRouting.routingSCs > 1) {
		for (i=0; i<STL_MAX_SLS; i++) {
			if (i==15 || !bitset_test(&sm_linkSLsInuse, i)) continue;
			firstSC = sm_SLtoSC[i];
			secondSC = -1;
			for (j=firstSC+1; j<STL_MAX_SCS; j++) {
				if (sm_SCtoSL[j] != i) continue;
				if (smDorRouting.routingSCs == 2) {
					// mesh or no escape VLs - set first SC to use escape
					scscBadTurn.SCSCMap[firstSC].SC = j;
					scscBadTurn.SCSCMap[j].SC = j;
					break;
				}
				if (secondSC < 0) {
					secondSC = j;
					continue;
				}
				// torus - set first set of SCs to 2nd set start
				scscBadTurn.SCSCMap[firstSC].SC = scscBadTurn.SCSCMap[secondSC].SC = j;
				if (j+1 < STL_MAX_SCS)
					scscBadTurn.SCSCMap[j].SC = scscBadTurn.SCSCMap[j+1].SC = j;
				break;
			}
		}
	}

	// Setup 1:1 mapping
	memset(&scscNoChg, 15, sizeof(scscNoChg));
	for (i=0; i<STL_MAX_SCS; i++) {
		if (bitset_test(&sm_linkSLsInuse, sm_SCtoSL[i]))
			scscNoChg.SCSCMap[i].SC = i;
	}

	// Setup mapping for moving back to first SC mapped to SL
	// Setup mapping for moving to second SC mapped to SL
	memset(&scsc0, 15, sizeof(scsc0));
	memset(&scscPlus1, 15, sizeof(scscPlus1));

	if (smDorRouting.routingSCs > 1) {
		for (i=0; i<STL_MAX_SLS; i++) {
			if (i==15 || !bitset_test(&sm_linkSLsInuse, i)) continue;
			firstSC = sm_SLtoSC[i];
			setsComplete = 0;
			for (j=firstSC+1; j<STL_MAX_SCS; j++) {
				if (sm_SCtoSL[j] != i) continue;

				if (firstSC < 0) {
					firstSC = j;
					continue;
				}

				scsc0.SCSCMap[firstSC].SC = scsc0.SCSCMap[j].SC = firstSC;
				scscPlus1.SCSCMap[firstSC].SC = scscPlus1.SCSCMap[j].SC = j;

				setsComplete++;
				if (setsComplete == 1 && smDorRouting.routingSCs == 2) break;
				if (setsComplete == 2 && smDorRouting.routingSCs == 4) break;

				firstSC = -1;
			}
		}
	}

	if (!smDorRouting.routingSCs) {
		if (vs_pool_alloc(&sm_pool, sizeof(STL_SCSC_MULTISET), (void *) &scsc) != VSTATUS_OK)
			return VSTATUS_BAD;

		memset(scsc, 0, sizeof(STL_SCSC_MULTISET));
	} else {
		// TBD refine later? Acquiring worst case size.
		if (smDorRouting.debug) 
			IB_LOG_INFINI_INFO_FMT(__func__, "Switch [%s] has %d ISLs", sm_nodeDescString(switchp), switchp->numISLs);
		if (vs_pool_alloc(&sm_pool, sizeof(STL_SCSC_MULTISET)*switchp->numISLs*switchp->numISLs+2, (void *) &scsc) != VSTATUS_OK)
			return VSTATUS_BAD;
		memset(scsc, 0, sizeof(STL_SCSC_MULTISET)*switchp->numISLs*switchp->numISLs+2);
	}

	// SC2SC - no change
	// HFI -> All - toroidal
	// All -> All - mesh
	for_all_physical_ports(switchp, ingressPortp) {
		if (!sm_valid_port(ingressPortp) || ingressPortp->state <= IB_PORT_DOWN) continue;

		if (!ingressPortp->portData->scscMap) {
			// unexpected error
			(void) vs_pool_free(&sm_pool, scsc);
			return VSTATUS_BAD;
		}

		if (ingressPortp->portData->isIsl && smDorRouting.routingSCs > 1) continue;

		needsSet = !ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite;
		if (!needsSet) {
			for (i=1; i<=switchp->nodeInfo.NumPorts; i++) {
				if (memcmp((void *)&scscNoChg, (void *)&ingressPortp->portData->scscMap[i-1], sizeof(STL_SCSCMAP)) != 0) {
					needsSet = 1;
					break;
				}
			}
		}
		if (needsSet) {
			StlAddPortToPortMask(scsc[curBlock].IngressPortMask, ingressPortp->index);
			portToSet = 1;
		}
	}
	
	if (portToSet) {
		// To all ports
		for (i=1; i<=switchp->nodeInfo.NumPorts; i++)
			StlAddPortToPortMask(scsc[curBlock].EgressPortMask, i);

		scsc[curBlock].SCSCMap = scscNoChg;
		curBlock++;
	}

	// Is this a mesh?
	if (smDorRouting.routingSCs == 1) 
		goto done;

	// Any ISL -> HFI: SCSC0 (works for PRR, APR?)
	portToSet = 0;
	for_all_physical_ports(switchp, ingressPortp) {
		if (!sm_valid_port(ingressPortp) || ingressPortp->state <= IB_PORT_DOWN) continue;

		if (!ingressPortp->portData->isIsl) continue;

		portDim[ingressPortp->index] = get_configured_dimension_for_port(ingressPortp->index);
		portPos[ingressPortp->index] = get_configured_port_pos_in_dim(portDim[ingressPortp->index], ingressPortp->index);

		for_all_physical_ports(switchp, egressPortp) {
			if (!sm_valid_port(egressPortp)) continue;

			if (egressPortp->portData->isIsl) continue;

			if (!ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite ||
				(memcmp((void *)&scsc0, (void *)&ingressPortp->portData->scscMap[egressPortp->index-1], sizeof(STL_SCSCMAP)) != 0)) {

				StlAddPortToPortMask(scsc[curBlock].IngressPortMask, ingressPortp->index);
				StlAddPortToPortMask(scsc[curBlock].EgressPortMask, egressPortp->index);
				portToSet = 1;
			}
		}
	}
	
	if (portToSet) {
		scsc[curBlock].SCSCMap = scsc0;
		curBlock++;
	}

	for (dimension=0; dimension<dorTop->numDimensions; dimension++) {
		datelineSwitch = isDatelineSwitch(topop, switchp, dimension);
		changeDimBlk = illegalTurn = crossMeridianBlk = sameDimBlk = -1;
		int lastPos = -1;
		for (p1=1; p1<=switchp->nodeInfo.NumPorts; p1++) {
			if (portDim[p1] != dimension) continue;

			if (lastPos == -1 || (datelineSwitch && lastPos != portPos[p1])) {
				// Can group if in same direction, otherwise need to check if dateline is crossed.
				crossMeridianBlk = sameDimBlk = -1;
				lastPos = portPos[p1];
			}

			ingressPortp = sm_get_port(switchp, p1);
			if (!sm_valid_port(ingressPortp)) continue;

			// Setup scsc for illegal turn (to lower dimension)
			for (p2=1; p2<=switchp->nodeInfo.NumPorts; p2++) {
				if (portDim[p2] == 0xff) continue; // HFI
				if (portDim[p2] >= dimension) continue;

				if (!ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite ||
					(memcmp((void *)&scscBadTurn, (void *)&ingressPortp->portData->scscMap[p2-1], sizeof(STL_SCSCMAP)) != 0)) {
					if (illegalTurn == -1) {
						illegalTurn = curBlock;
						scsc[illegalTurn].SCSCMap = scscBadTurn;
						curBlock++;
					}
					StlAddPortToPortMask(scsc[illegalTurn].IngressPortMask, p1);
					StlAddPortToPortMask(scsc[illegalTurn].EgressPortMask, p2);
				}
			}

			if (smDorRouting.topology == DOR_MESH) {
				// Setup scsc for same or higher dimension to no change
				for (p2=1; p2<=switchp->nodeInfo.NumPorts; p2++) {
					if (portDim[p2] == 0xff) continue; // HFI
					if (portDim[p2] < dimension) continue;

					if (!ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite ||
						(memcmp((void *)&scscNoChg, (void *)&ingressPortp->portData->scscMap[p2-1], sizeof(STL_SCSCMAP)) != 0)) {

						if (changeDimBlk == -1) {
							changeDimBlk = curBlock;
							scsc[changeDimBlk].SCSCMap = scscNoChg;
							curBlock++;
						}
						StlAddPortToPortMask(scsc[changeDimBlk].IngressPortMask, p1);
						StlAddPortToPortMask(scsc[changeDimBlk].EgressPortMask, p2);
					}
				}
				continue;
			}

			// Setup scsc for change in direction (to higher dimension) to drop back to SL/SC map
			for (p2=1; p2<=switchp->nodeInfo.NumPorts; p2++) {
				if (portDim[p2] == 0xff) continue; // HFI
				if (portDim[p2] <= dimension) continue;

				if (!ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite ||
					(memcmp((void *)&scsc0, (void *)&ingressPortp->portData->scscMap[p2-1], sizeof(STL_SCSCMAP)) != 0)) {

					if (changeDimBlk == -1) {
						changeDimBlk = curBlock;
						scsc[changeDimBlk].SCSCMap = scsc0;
						curBlock++;
					}
					StlAddPortToPortMask(scsc[changeDimBlk].IngressPortMask, p1);
					StlAddPortToPortMask(scsc[changeDimBlk].EgressPortMask, p2);
				}
			}

			// Setup scsc for same dimension
			for (p2=1; p2<=switchp->nodeInfo.NumPorts; p2++) {
				// Is port in our dimension
				if (portDim[p2] != dimension) continue;

				if ((portPos[p1] != portPos[p2]) && datelineSwitch) {
					// isl -> port in same dim across meridian
					// SCx->SCx+1
					if (!ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite ||
						(memcmp((void *)&scscPlus1, (void *)&ingressPortp->portData->scscMap[p2-1], sizeof(STL_SCSCMAP)) != 0)) {

						if (crossMeridianBlk == -1) {
							crossMeridianBlk = curBlock;
							scsc[crossMeridianBlk].SCSCMap = scscPlus1;
							curBlock++;
						}
	
						StlAddPortToPortMask(scsc[crossMeridianBlk].IngressPortMask, ingressPortp->index);
						StlAddPortToPortMask(scsc[crossMeridianBlk].EgressPortMask, p2);
					}
					if (smDorRouting.debug)
						IB_LOG_INFINI_INFO_FMT(__func__, "Dateline %s (%d->%d)", sm_nodeDescString(switchp), ingressPortp->index, p2);

				} else {
					// isl -> port in same dim no cross
					if (!ingressPortp->portData->current.scsc ||  sm_config.forceAttributeRewrite ||
						(memcmp((void *)&scscNoChg, (void *)&ingressPortp->portData->scscMap[p2-1], sizeof(STL_SCSCMAP)) != 0)) {

						if (sameDimBlk == -1) {
							sameDimBlk = curBlock;
							scsc[sameDimBlk].SCSCMap = scscNoChg;
							curBlock++;
						}
						StlAddPortToPortMask(scsc[sameDimBlk].IngressPortMask, ingressPortp->index);
						StlAddPortToPortMask(scsc[sameDimBlk].EgressPortMask, p2);
					}
				}
			}
		}
	}
	
done:
	if (curBlock == 0) {
		(void) vs_pool_free(&sm_pool, scsc);
		scsc = NULL;
	}

	if (smDorRouting.debug) {
		IB_LOG_INFINI_INFO_FMT(__func__,
				   "Switch with NodeGUID "FMT_U64" [%s] has %d scsc multiset blocks",
				   switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), curBlock);

		for (i=0;i<curBlock;i++) {
			STL_SC *scscmap = scsc[i].SCSCMap.SCSCMap;
			char	iports[80];
			char	eports[80];
			FormatStlPortMask(iports, scsc[i].IngressPortMask, switchp->nodeInfo.NumPorts, 80);
			FormatStlPortMask(eports, scsc[i].EgressPortMask, switchp->nodeInfo.NumPorts, 80);
			
			IB_LOG_INFINI_INFO_FMT(__func__,
			   "SCSC[%d] %s ingress %s egress %s "
				"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
			   	i, sm_nodeDescString(switchp), iports, eports,
				scscmap[0].SC, scscmap[1].SC, scscmap[2].SC, scscmap[3].SC, scscmap[4].SC, scscmap[5].SC, scscmap[6].SC, scscmap[7].SC,
				scscmap[8].SC, scscmap[9].SC, scscmap[10].SC, scscmap[11].SC, scscmap[12].SC, scscmap[13].SC, scscmap[14].SC, scscmap[15].SC,
				scscmap[16].SC, scscmap[17].SC, scscmap[18].SC, scscmap[19].SC, scscmap[20].SC, scscmap[21].SC, scscmap[22].SC, scscmap[23].SC,
				scscmap[24].SC, scscmap[25].SC, scscmap[26].SC, scscmap[27].SC, scscmap[28].SC, scscmap[29].SC, scscmap[30].SC, scscmap[31].SC);
		}
	}

	*scscmap = scsc;
	*numBlocks = curBlock;

	return VSTATUS_OK;
}

//===========================================================================//
// XFT CALCULATION AND HELPERS
//

static int
_compare_lids_routed(const void * arg1, const void * arg2)
{
	SwitchportToNextGuid_t *p1 = (SwitchportToNextGuid_t *)arg1;
	SwitchportToNextGuid_t *p2 = (SwitchportToNextGuid_t *)arg2;
	if (p1->portp->portData->lidsRouted < p2->portp->portData->lidsRouted)
		return -1;
	else if (p1->portp->portData->lidsRouted > p2->portp->portData->lidsRouted)
		return 1;
	else
		return 0;
}

static inline int routingDimension(Topology_t *topop, const struct _Node *switchp, const struct _Node *toSwitchp) {

	DorNode_t *srcDnp = (DorNode_t*)switchp->routingData;
	DorNode_t *dstDnp = (DorNode_t*)toSwitchp->routingData;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;
	int i, si, di;

	for (i = 0; i < dorTop->numDimensions; ++i) {
		si = srcDnp->coords[i];
		di = dstDnp->coords[i];
		if (si != di) 
			return i;
	}
	return 0;
}

static Status_t
_get_outbound_port_dor(Topology_t *topop, Node_t *switchp, Node_t *endNodep,
                       Port_t *endPortp, uint8_t *portnos)
{
	uint8_t numLids = 1 << endPortp->portData->lmc;
	Node_t *endSwitchp = NULL;
	int i;

	memset((void*)portnos, 0xff, sizeof(uint8_t) * numLids);

	endSwitchp = _get_switch(topop, endNodep, endPortp);
	if (endSwitchp == NULL) {
		IB_LOG_NOTICE_FMT(__func__,
		       "Failed to find destination switch in path from NodeGUID "FMT_U64" [%s] to NodeGUID "FMT_U64" [%s]",
		       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endNodep->nodeInfo.NodeGUID, sm_nodeDescString(endNodep));
		return VSTATUS_BAD;
	}

	if (switchp->swIdx == endSwitchp->swIdx) {
		for (i = 0; i < numLids; i++)
			portnos[i] = endPortp->portno;
		return VSTATUS_OK;
	}

	topop->routingModule->funcs.get_port_group(topop, switchp, endSwitchp, portnos);

	if (portnos[0] == 0xff && smDebugPerf)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "Failed to setup LID 0x%.4X for switch %d, NodeGUID "FMT_U64" [%s]",
		       endPortp->portData->lid, switchp->index, switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));

	return VSTATUS_OK;
}

static Status_t
_setup_pgs(struct _Topology *topop, struct _Node * srcSw, struct _Node * dstSw)
{
	int endPort = 0;
	uint8_t	portGroup[256] = { 0xff };

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

	endPort = topop->routingModule->funcs.get_port_group(topop, srcSw, dstSw, portGroup);

	if (endPort <= 1) {
		srcSw->switchInfo.PortGroupTop = 0;
		return VSTATUS_OK;
	}

	STL_PORTMASK pgMask = 0;
	int i;
	for (i = 0; i < endPort; ++i) {
		if (portGroup[i] == 0 ||
			portGroup[i] > sizeof(pgMask)*8) {
			continue;
		}

		// Cast is necessary to prevent compiler from interpreting '1' as a signed
		// int32, converting it to an int64, then or'ing
		pgMask |= (((uint64)1) << (portGroup[i] - 1));
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

static inline void
_add_ports(Node_t *switchp, uint32_t neighborNode, SwitchportToNextGuid_t *ordered_ports, int* endPorts) {

	int i = *endPorts;
	Port_t	*portp;

	for_all_physical_ports(switchp, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
		if (portp->nodeno == neighborNode) {
			ordered_ports[i++].portp = portp;
		}
	}
	*endPorts = i;
}

static int
_get_dor_port_group(Topology_t *topop, Node_t *switchp, int ij, int routingDim, uint8_t *portnos)
{	
	int i;
	DorNode_t *srcDnp = (DorNode_t*)switchp->routingData;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;
	SwitchportToNextGuid_t *ordered_ports = (SwitchportToNextGuid_t *)topop->pad;
	int count = 0;

	if (routingDim >= SM_DOR_MAX_DIMENSIONS) {
		IB_LOG_ERROR_FMT(__func__, "Dimension out of range.");
		return 0;
	}

	if (ijTest(dorTop->dorLeft, ij)) {
		_add_ports(switchp, srcDnp->left[routingDim]->node->index, ordered_ports, &count);
	}

	if (ijTest(dorTop->dorRight, ij)) {
		_add_ports(switchp, srcDnp->right[routingDim]->node->index, ordered_ports, &count);
	}

	qsort(ordered_ports, count, sizeof(SwitchportToNextGuid_t), _compare_lids_routed);
 
	for (i=0; i<count; i++) {
		portnos[i] = ordered_ports[i].portp->index;
	}

	return count;
}

static int
_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *toSwitchp, uint8_t *portnos)
{		
	int				count=0;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (dorClosure(dorTop, switchp->swIdx, toSwitchp->swIdx)) {
		count = _get_dor_port_group(topop, switchp, 
					DorBitMapsIndex(switchp->swIdx, toSwitchp->swIdx),
					routingDimension(topop, switchp, toSwitchp), portnos);
	} else {
		count = _get_alternate_path_port_group(topop, switchp, toSwitchp, portnos);
	}

	if (count == 0 && smDebugPerf) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Failed to get portGroup from switch %s to switch %s",
   								sm_nodeDescString(switchp), sm_nodeDescString(toSwitchp));
	}

	return count;
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
_calculate_lft(Topology_t * topop, Node_t *switchp)
{
	Node_t *toSwitchp, *nodep;
	Port_t *portp, *toSwitchPortp;
	Status_t status = VSTATUS_OK;
	int i, currentLid, numPorts;
	uint8_t portGroup[256];

	status = sm_Node_init_lft(switchp, NULL);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate space for LFT.");
		return status;
	}

	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__, "Switch %s", sm_nodeDescString(switchp));

	for_all_switch_nodes(topop, toSwitchp) {
		i = 0;
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

			// If the node connected to this port isn't an HFI,
			// skip to the next port.
	   		nodep = sm_find_node(topop, toSwitchPortp->nodeno);
	   		if (!nodep) continue;
	   		if (nodep->nodeInfo.NodeType != NI_TYPE_CA) continue;

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

					switchp->lft[currentLid] = portGroup[i%numPorts];
					i++;

					incr_lids_routed(topop, switchp, switchp->lft[currentLid]);

					if (smDorRouting.debug)
						IB_LOG_VERBOSE_FMT(__func__, "Switch %s to %s lid 0x%x outport %d (of %d)", 
							sm_nodeDescString(switchp), sm_nodeDescString(nodep), currentLid,
							switchp->lft[currentLid], numPorts);
				}
			}
		}

		// Setup switch to switch routing - no need to balance
		for_all_end_ports(toSwitchp, toSwitchPortp) {
			if (!sm_valid_port(toSwitchPortp) || toSwitchPortp->state <= IB_PORT_DOWN) continue;
			for_all_port_lids(toSwitchPortp, currentLid) {
				switchp->lft[currentLid] = (numPorts ? portGroup[0] : toSwitchPortp->index);
			}
		}
		if (topop->routingModule->funcs.setup_pgs) {
			status = topop->routingModule->funcs.setup_pgs(topop, switchp, toSwitchp);
		}
	}

	return status;
}

static Status_t
_init_switch_lfts(Topology_t * topop, int * routing_needed, int * rebalance)
{
	Status_t s = VSTATUS_OK;
	Node_t	*switchp;

	// Only work on sm_topop/sm_newTopology for now
	if (topop != sm_topop)
		return VSTATUS_BAD;

	if (topology_cost_path_changes || *rebalance) {
		// A topology change was indicated.  Re-calculate lfts with big hammer (rebalance).
		// If not, copy and delta updates handled by main topology method.
		for_all_switch_nodes(topop, switchp) {
			if ((s = topop->routingModule->funcs.calculate_lft(topop, switchp)) != VSTATUS_OK) break;
		}
		*rebalance = 1;
		routing_recalculated = 1;
	}

	return s;
}

static Status_t
_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *endNodep,
                 Port_t *endPortp, uint8_t *portnos)
{
	Status_t status = _get_outbound_port_dor(topop, switchp, endNodep, endPortp, portnos);

	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
			"Routing SW "FMT_U64" [%s] to DLID 0x%04x via DOR: EgressPort %d",
			switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endPortp->portData->lid, portnos[0]);

	return status;
}

//===========================================================================//
// ROUTING HOOKS
//

static Status_t
_pre_process_discovery(Topology_t *topop, void **outContext)
{
	Status_t status;
	DorDiscoveryState_t *state;
	DorTopology_t	*dorTop;
	int i;

	status = vs_pool_alloc(&sm_pool, sizeof(DorTopology_t), &topop->routingModule->data);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate routingModule data; rc:", status);
		return status;
	}
	dorTop = (DorTopology_t*)topop->routingModule->data;
	memset(dorTop, 0, sizeof(DorTopology_t));

	status = vs_pool_alloc(&sm_pool, sizeof(DorDiscoveryState_t),
		(void *)&state);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate up/down state; rc:", status);
		return status;
	}
	memset((void *)state, 0, sizeof(*state));

	// @TODO: Adjust to STL Max once new VL ranges have been defined
	// default VLs available to max
	state->scsAvailable = STL_MAX_SCS;

	*outContext = (void *)state;

	for (i = 0; i < smDorRouting.dimensionCount; i++) {
		if (smDorRouting.dimension[i].toroidal) {
			dorTop->numToroidal++;
		}
		smDorRouting.dimension[i].created = 0;
	}

	dorTop->minReqScs = smDorRouting.routingSCs;

	status = vs_pool_alloc(&sm_pool, (PORT_PAIR_WARN_ARR_SIZE * PORT_PAIR_WARN_ARR_SIZE), (void *)&port_pair_warnings);
	if (status != VSTATUS_OK) {
		port_pair_warnings = NULL;
		IB_LOG_WARNRC("Failed to allocate port pair warnings array. Warnings will not be throttled rc:", status);
	} else {
		memset(port_pair_warnings, 0, (PORT_PAIR_WARN_ARR_SIZE * PORT_PAIR_WARN_ARR_SIZE));
	}

	incorrect_ca_warnings = 0;
	invalid_isl_found = 0;

	return VSTATUS_OK;
}

static int get_node_information(Node_t *nodep, Port_t *portp, uint8_t *path, STL_NODE_INFO *neighborNodeInfo)
{
	int			use_cache = 0;
	Node_t		*cache_nodep = NULL;
	Port_t		*cache_portp = NULL;
	Status_t	status;
	

	memset(neighborNodeInfo, 0, sizeof(STL_NODE_INFO));
	if ((status = SM_Get_NodeInfo(fd_topology, 0, path, neighborNodeInfo)) != VSTATUS_OK) {
		use_cache = sm_check_node_cache(nodep, portp, &cache_nodep, &cache_portp);
		if (use_cache) {
			memcpy(neighborNodeInfo, &cache_nodep->nodeInfo, sizeof(STL_NODE_INFO));
			neighborNodeInfo->PortGUID = cache_portp->portData->guid;
			neighborNodeInfo->u1.s.LocalPortNum = cache_portp->index;
		} else {
			return 0;
		}
	}	

	if (neighborNodeInfo->NodeGUID == 0ull)
		return 0;

	return 1;
}

static int get_node_desc(Node_t *nodep, Port_t *portp, uint8_t *path, STL_NODE_DESCRIPTION *nodeDesc)
{
	int			use_cache = 0;
	Node_t		*cache_nodep = NULL;
	Port_t		*cache_portp = NULL;
	Status_t	status;

	if ((status = SM_Get_NodeDesc(fd_topology, 0, path, nodeDesc)) != VSTATUS_OK) {
		if((use_cache = sm_check_node_cache(nodep, portp, &cache_nodep, &cache_portp)) != 0){
			memcpy(nodeDesc, &cache_nodep->nodeDesc, sizeof(STL_NODE_DESCRIPTION));
		} else {
			return 0;
		}
	}

	return 1;
}

static Status_t
_discover_node(Topology_t *topop, Node_t *nodep, void *context)
{
	uint8_t	path[72];
	uint8_t	dim_count = 0;
	int incorrect_ca = 0, invalid = 0, i=0, j=0, known_dim = 0, dim, pos=0;
	int port_down = 0, connected = 0;
	Node_t	*neighbor;
	Port_t	*neighbor_portp, *p;
	uint64_t	neighborNodeGuid, disable_neighborNodeGuid;
	STL_NODE_DESCRIPTION neighborNodeDesc;
	STL_NODE_INFO  neighborNodeInfo;
	uint8_t		neighborType;
	char		*disable_neighborDescString = NULL;
	uint8_t		neighbor_portno, disable_portno, disable_neighbor_portno;
	detected_dim_t		*detected_dim;
	Status_t	status;

	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
		return VSTATUS_OK;

	/* validate all the links of the switch to make sure they are valid according to the
	 * DOR port pair configuration. If they are not, then mark those links as DOWN in
	 * the topology.
	 */

	status = vs_pool_alloc(&sm_pool, (sizeof(detected_dim_t) * (nodep->nodeInfo.NumPorts)), (void *) &detected_dim);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("unable to allocate memory to verify ISLs; rc:", status);
		return status;
	}

	(void)memcpy((void *)path, (void *)nodep->path, 64);
	path[0]++; // element zero is path length
 	neighborNodeDesc.NodeString[sizeof(STL_NODE_DESCRIPTION) - 1] = '\0';  // NULL terminate node desc

	for (j = 1; j < nodep->nodeInfo.NumPorts; j++) {
		if ((p = sm_get_port(nodep,j)) == NULL || p->state <= IB_PORT_DOWN) {
			continue;
		}
		incorrect_ca = 0;
		invalid = 0;
		path[path[0]] = p->index;
		neighbor = sm_find_node(topop, p->nodeno);
		if (neighbor) {
			neighborType = neighbor->nodeInfo.NodeType;
			if ((neighborType == NI_TYPE_CA) && !smDorRouting.debug)
				continue;
			neighborNodeGuid = neighbor->nodeInfo.NodeGUID;
			neighbor_portno = p->portno;
			neighbor_portp = sm_get_port(neighbor, neighbor_portno);
			if (sm_valid_port(neighbor_portp)) {
				if (neighbor_portp->state <= IB_PORT_DOWN) {
					sm_mark_link_down(topop, p);
					topology_changed = 1;
					continue;
				}
			}
		} else {
			/* its possible that p->nodeno for this port is not valid yet, but the neighbor might be in the topology*/
			if (!get_node_information(nodep, p, path, &neighborNodeInfo))
				continue;
			neighborType = neighborNodeInfo.NodeType;
			if ((neighborType == NI_TYPE_CA) && !smDorRouting.debug)
				continue;
			neighborNodeGuid = neighborNodeInfo.NodeGUID;
			neighbor_portno = neighborNodeInfo.u1.s.LocalPortNum;

			neighbor = sm_find_guid(topop, neighborNodeGuid);
			if (neighbor) {
				neighbor_portp = sm_get_port(neighbor, neighbor_portno);
				if (sm_valid_port(neighbor_portp)) {
					if (neighbor_portp->state <= IB_PORT_DOWN) {
						sm_mark_link_down(topop, p);
						topology_changed = 1;
						continue;
					}
				}
			}
		}
		
		if (neighborType == NI_TYPE_SWITCH) {
			dim = get_configured_dimension(p->index, neighbor_portno);
			if (dim < 0 || dim >= MAX_DOR_DIMENSIONS) {
				invalid = 1;
			} else {
				/* Valid port pair, but make sanity check against other ISLs we have seen on this switch to
				 * make sure that this ISL will not result in any conflicts with existing ones.
				 */
				known_dim = 0;
				pos = get_configured_port_pos_in_dim(dim, p->index);
				for (i = 0; i < dim_count; i++) {
					if ((detected_dim[i].dim == dim) && (detected_dim[i].pos == pos)) {
						//there is an existing valid ISL in same dimension and same direction
						if (detected_dim[i].neighbor_nodeGuid != neighborNodeGuid) {
						//but it is not to the same switch
							invalid = 2;
							break;
						}
						known_dim = 1;
					} else if ((detected_dim[i].dim != dim) && (detected_dim[i].neighbor_nodeGuid == neighborNodeGuid)) {
						//there is a pre-existing ISL to the same switch but that is in a different dimension
						//we cannot have ISLs between the same two switches to be in differnet dimensions.
						invalid = 3;
						break;
					}
				}
				if (!invalid && !known_dim) {
					/* This ISL is fine, add it to list of valid ISLs seen till now*/
					detected_dim[dim_count].dim = dim;
					detected_dim[dim_count].neighbor_nodeGuid = neighborNodeGuid;
					detected_dim[dim_count].port = p->index;
					detected_dim[dim_count].neighbor_port = neighbor_portno;
					detected_dim[dim_count].pos = pos;
					detected_dim[dim_count].neighbor_nodep = neighbor;
					dim_count++;
				}
			}	
		} else if (neighborType == NI_TYPE_CA) {
			dim = get_configured_dimension_for_port(p->index);
			if (dim > 0 && dim < MAX_DOR_DIMENSIONS) {
				/* the switch port has been specified as an ISL in config but it is connected to an HFI*/

				/* It could be a mesh topology in which case the end switches in a dimension can have HFIs connected to them */
				/* Since the discovery is still in progress, we don't know if the current node is an end switch in a dimension */
				/* So for now do this check only if the dimension has been marked toroidal, in which case the port pairs for end */
				/* switches should also be connected to switches. */
				if (smDorRouting.dimension[dim].toroidal == 0)
					continue;
				incorrect_ca = 1;
			} else  {
				continue;
			}
		
		}

		if (!invalid && !incorrect_ca)
			continue;

		if ((invalid == 2) && neighbor && !detected_dim[i].neighbor_nodep) {
			/* Prefer to disable the link to the switch which is not yet part of the topology*/
			disable_portno = detected_dim[i].port;
			disable_neighborNodeGuid = detected_dim[i].neighbor_nodeGuid;
			disable_neighbor_portno = detected_dim[i].neighbor_port;
			/* replace link to switch that is part of topology to be one of the valid detected ISLs*/
			detected_dim[i].dim = dim;
			detected_dim[i].neighbor_nodeGuid = neighbor->nodeInfo.NodeGUID;
			detected_dim[i].port = p->index;
			detected_dim[i].neighbor_port = neighbor_portno;
			detected_dim[i].pos = pos;
			detected_dim[i].neighbor_nodep = neighbor;

			p = sm_get_port(nodep, disable_portno);
			path[path[0]] = disable_portno;
			if (get_node_desc(nodep, p, path, &neighborNodeDesc)) {
				disable_neighborDescString = (char*)neighborNodeDesc.NodeString;
			}
		} else if ((invalid) || (incorrect_ca)) {
			disable_portno = j;
			disable_neighbor_portno = neighbor_portno;
			disable_neighborNodeGuid = neighborNodeGuid;
			if (neighbor) {
				disable_neighborDescString = sm_nodeDescString(neighbor);
			} else 	if (get_node_desc(nodep, p, path, &neighborNodeDesc)) {
				disable_neighborDescString = (char*)neighborNodeDesc.NodeString;
			}
		}

		if (invalid) {
			p = sm_get_port(nodep, disable_portno);
			if (sm_valid_port(p)) {
				sm_mark_link_down(topop, p);
				topology_changed = 1;
			}
			port_down = 1;
			invalid_isl_found = 1;
		} else if (incorrect_ca) {
			if (incorrect_ca_warnings < smDorRouting.warn_threshold) {
				IB_LOG_WARN_FMT(__func__,
							"NodeGuid "FMT_U64" [%s] port %d is specified as part of ISL port pair but"
							" is connected to the HFI NodeGuid "FMT_U64" [%s] port %d."
							" This may result in incorrect number of dimensions.",
							nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), disable_portno,
							disable_neighborNodeGuid, disable_neighborDescString, disable_neighbor_portno);
				incorrect_ca_warnings++;
			}
			/* Currently only warning in case of incorrect FI connection for a port specified as part of ISL */
			continue;
		}

		if (!port_pair_needs_warning(disable_portno, disable_neighbor_portno))
			continue;

		if (invalid == 1) {
			IB_LOG_WARN_FMT(__func__,
				"Ignoring NodeGuid "FMT_U64" [%s] port %d which connects to NodeGuid "FMT_U64" [%s] port %d "
				"as this port pair does not match any of the PortPairs in the DOR configuration",
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), disable_portno,
				disable_neighborNodeGuid, disable_neighborDescString, disable_neighbor_portno);
		} else if (invalid == 2) {
			if (detected_dim[i].neighbor_nodep) {
				IB_LOG_WARN_FMT(__func__,
					"Ignoring NodeGuid "FMT_U64" [%s] port %d which connects to NodeGuid "FMT_U64" [%s] port %d"
					" as it conflicts with the another inter-switch link in the same dimension from port %d but"
					" to a different switch NodeGuid "FMT_U64" [%s] port %d",
					nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), disable_portno,
					disable_neighborNodeGuid, disable_neighborDescString, disable_neighbor_portno,
					detected_dim[i].port, detected_dim[i].neighbor_nodeGuid, sm_nodeDescString(detected_dim[i].neighbor_nodep),
					detected_dim[i].neighbor_port);	
			} else {
				IB_LOG_WARN_FMT(__func__,
					"Ignoring NodeGuid "FMT_U64" [%s] port %d which connects to NodeGuid "FMT_U64" [%s] port %d"
					" as it conflicts with the another inter-switch link in the same dimension from port %d but"
					" to a different switch NodeGuid "FMT_U64" port %d",
					nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), disable_portno,
					disable_neighborNodeGuid, disable_neighborDescString, disable_neighbor_portno,
					detected_dim[i].port, detected_dim[i].neighbor_nodeGuid, detected_dim[i].neighbor_port);
			}			
		} else if (invalid == 3) {
			IB_LOG_WARN_FMT(__func__,
					"Ignoring NodeGuid "FMT_U64" [%s] port %d which connects to NodeGuid "FMT_U64" [%s] port %d"
					" as it conflicts with the the other inter-switch link from port %d to port %d between these switches"
					" which is configured to be in a different dimension",
					nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), disable_portno,
					disable_neighborNodeGuid, disable_neighborDescString, disable_neighbor_portno,
				   	detected_dim[i].port, detected_dim[i].neighbor_port);
		}
	}

	if (port_down && (nodep != topop->switch_head)) {
		/* Due to invalid links, we might have brought ports down through which we have
		 * discovered this node. Check to see if we have any more connections to the already
		 * discovered fabric.
		 */
		for (i=0; i < dim_count; i++) {
			if (detected_dim[i].neighbor_nodep) {
				/* if the detected ISL has a valid neighbor_nodep, then that port was a link to already
				 * discovered fabric. Check to see if we marked it down.
				 */
				p = sm_get_port(nodep, detected_dim[i].port);
				if (sm_valid_port(p) && (p->state > IB_PORT_DOWN)) {
					connected = 1;
					break;
				}
			}
		}

		if (!connected) {
			IB_LOG_WARN_FMT(__func__, "After ignoring invalid links NodeGuid "FMT_U64" [%s] is"
				" no longer connected to rest of the already discovered fabric !",
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
			IB_LOG_WARN_FMT(__func__, "Please verify above reported invalid links !");
		
			vs_pool_free(&sm_pool, detected_dim);
			return VSTATUS_BAD;
		}
	}

	vs_pool_free(&sm_pool, detected_dim);

	
	return VSTATUS_OK;
}

static Status_t
_discover_node_port(Topology_t *topop, Node_t *nodep, Port_t *portp, void *context)
{
	Status_t status;

	if (nodep->routingData == NULL) {
		// should only apply to the first node, since propagation will
		// take care of the rest
		status = vs_pool_alloc(&sm_pool, sizeof(DorNode_t), &nodep->routingData);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("_discover_node: Failed to allocate storage for DOR node structure; rc:", status);
			return status;
		}
		memset(nodep->routingData, 0, sizeof(DorNode_t));

		((DorNode_t*)nodep->routingData)->node = nodep;
	}

	return _propagate_coord_through_port((DorDiscoveryState_t *)context, topop, nodep, portp);
}

static Status_t
_post_process_discovery(Topology_t *topop, Status_t discoveryStatus, void *context)
{
	int i, j, idx, general_warning = 0, specific_warning = 0;
	DorDiscoveryState_t *state = (DorDiscoveryState_t *)context;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	if (VirtualFabrics && VirtualFabrics->qosEnabled) {
		topop->qosEnforced = 1;
	}

	/* Even in the case where discovery did not go well, display warnings which might be useful
	 * to understand what was wrong with the fabric.
	 */

	if (incorrect_ca_warnings) {
		IB_LOG_WARN0("HFIs were found connected to switch ports specified as PortPairs in MeshTorusTopology dimensions.");
	}

	if (invalid_isl_found && port_pair_warnings) {
		for (i = 1; i < PORT_PAIR_WARN_ARR_SIZE; i++) {
			for (j = 1; j < PORT_PAIR_WARN_ARR_SIZE; j++) {
				idx = PORT_PAIR_WARN_IDX(i, j);
				if (port_pair_warnings[idx]) {
					if (!general_warning) {
						general_warning = 1;
						IB_LOG_WARN0("Invalid inter-switch links were found and ignored in the topology !");
						IB_LOG_WARN0("Please verify your inter-switch links to make sure the fabric is setup correctly !");
					}
					if (!specific_warning) {
						specific_warning = 1;
						IB_LOG_WARN0("Invalid inter-switch links have been found between the following switch port numbers:");
					}
					if (i > j) {
						idx = PORT_PAIR_WARN_IDX(j, i);
						if (port_pair_warnings[idx])
							continue;		//port pair info i,j has been logged. logging j,i again doesn't make sense here
					}
					IB_LOG_WARN_FMT(__func__, "%d %d", i, j);
				}
			}	
		}
	}

	if (port_pair_warnings) {
		vs_pool_free(&sm_pool, port_pair_warnings);
		port_pair_warnings = NULL;
	}

	// discovery didn't go well... just deallocate and return
	if (discoveryStatus != VSTATUS_OK) {
		for (i = 0; i < 256; ++i)
			if (state->dimensionMap[i] != NULL) {
				vs_pool_free(&sm_pool, state->dimensionMap[i]);
				state->dimensionMap[i] = NULL;
			}
		vs_pool_free(&sm_pool, state);
		return VSTATUS_OK;
	}

	dorTop->numDimensions = state->nextDimension;

	if (dorTop->numDimensions < smDorRouting.dimensionCount) {
		if (dorTop->numDimensions)
			IB_LOG_WARN_FMT(__func__, "Only %d of the %d configured dimensions discovered",
							dorTop->numDimensions, smDorRouting.dimensionCount);
		else
			IB_LOG_WARN_FMT(__func__, "No dimensions discovered ! (%d dimensions were configured)",
							smDorRouting.dimensionCount);
		for (i = 0; i < smDorRouting.dimensionCount; i++) {
			if (smDorRouting.dimension[i].created)
				continue;
			IB_LOG_WARN_FMT(__func__, "Dimension containing port pair %d %d not found",
							smDorRouting.dimension[i].portPair[0].port1, smDorRouting.dimension[i].portPair[0].port2);
		}
	} else 	if (dorTop->numDimensions > smDorRouting.dimensionCount) {
		IB_LOG_WARN_FMT(__func__, "Fabric programming inconsistency ! %d dimensions found but only %d dimensions were configured",
							dorTop->numDimensions, smDorRouting.dimensionCount);
	}


	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "Number of dimensions: %d",
		       dorTop->numDimensions);

	for (i = 0; i < dorTop->numDimensions; ++i) {
		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Dimension %d: length %d [%s]",
			       i, dorTop->dimensionLength[i],
			       dorTop->toroidal[i] ? "toroidal" : "not toroidal");
	}

	if (state->toroidalOverflow)
		IB_LOG_WARN_FMT(__func__,
		       "Too many toroidal dimensions found. Only the first %d will be routed cycle-free", SM_DOR_MAX_TOROIDAL_DIMENSIONS);

	if (state->scsAvailable < 2)
		IB_LOG_WARN("not enough SCs available to route cycle-free SCs:", state->scsAvailable);

	else if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Routing with credit loop avoidance (%d SCs available)",
			       MIN(1 << (state->scsAvailable - 1), 15));

	for (i = 0; i < 256; ++i)
		if (state->dimensionMap[i] != NULL) {
			vs_pool_free(&sm_pool, state->dimensionMap[i]);
			state->dimensionMap[i] = NULL;
		}
	vs_pool_free(&sm_pool, state);

	// check for fault regions
	_verify_connectivity(topop);

	_verify_dimension_lengths(topop);

	if (smDorRouting.debug)
		_dump_node_coordinates(topop);

	if (smDorRouting.debug)
		_verify_coordinates(topop);

	return VSTATUS_OK;
}

static Status_t
_post_process_routing_copy(Topology_t *src_topop, Topology_t *dst_topop, int *rebalance)
{
	Status_t status;

	status = _copy_dor_closure(src_topop, dst_topop);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to copy dimension-ordered connectivity; rc:", status);
		return status;
	}

	return VSTATUS_OK;
}

static Status_t
_post_process_routing(Topology_t *topop, Topology_t * old_topop, int *rebalance)
{
	Status_t status;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	dorTop->closure_max_sws = topop->max_sws;

	if (topop->max_sws == 0)
		return VSTATUS_OK;	/* HSM connected back to back with a host*/
	
	if (topology_cost_path_changes) {
		status = _calc_dor_closure(topop);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to calculate dimension-ordered connectivity; rc:", status);
			return status;
		}

	} else {
		// No change to cost matrix, loss/gain of redundant path, no
		// need to recalculate DOR closure.
		_post_process_routing_copy(old_topop, topop, rebalance);
	}

	return VSTATUS_OK;
}

/* old_idx is the old index of the switch, new_idx is the new index
 * last_idx is the last switch index.
 */
static Status_t _process_swIdx_change(Topology_t * topop, int old_idx, int new_idx, int last_idx)
{
	int i, ij, ji, oldij, oldji;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (dorTop->dorLeft == NULL && dorTop->dorRight == NULL)
		return VSTATUS_OK;

	for (i = 0; i < last_idx; i++) {
		ij = DorBitMapsIndex(i, new_idx);
		oldij = DorBitMapsIndex(i, old_idx);

		ji = DorBitMapsIndex(new_idx, i);
		oldji = DorBitMapsIndex(old_idx, i);

		if (i == new_idx) {
			oldij = oldji = DorBitMapsIndex(old_idx, old_idx);
		}

		if (dorTop->dorLeft != NULL) {
			if (ijTest(dorTop->dorLeft, oldij)) {
				ijSet(dorTop->dorLeft, ij);
				//reset old index value to 0 as it is no longer valid
				dorTop->dorLeft[oldij >> 5] &= ~((uint32_t)(1 << (ij & 0x1f)));
			} else {
				ijClear(dorTop->dorLeft, ij);
			}

			if (ijTest(dorTop->dorLeft, oldji)) {
				ijSet(dorTop->dorLeft, ji);
				//reset old index value to 0
				dorTop->dorLeft[oldji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			} else {
				ijClear(dorTop->dorLeft, ji);
			}
		}

		if (dorTop->dorRight != NULL) {
			if (ijTest(dorTop->dorRight, oldij)) {
				ijSet(dorTop->dorRight, ij);
				//reset old index value to 0 as it is no longer valid
				dorTop->dorRight[oldij >> 5] &= ~((uint32_t)(1 << (ij & 0x1f)));
			} else {
				ijClear(dorTop->dorRight, ij);
			}

			if (ijTest(dorTop->dorRight, oldji)) {
				ijSet(dorTop->dorRight, ji);
				//reset old index value to 0
				dorTop->dorRight[oldji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			} else {
				ijClear(dorTop->dorRight, ji);
			}
		}
	}

	return VSTATUS_OK;	
}

static void
_destroy(Topology_t *topop)
{
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (dorTop->dorRight != NULL) {
		vs_pool_free(&sm_pool, (void *)dorTop->dorRight);
		dorTop->dorRight = NULL;
	}

	if (dorTop->dorLeft != NULL) {
		vs_pool_free(&sm_pool, (void *)dorTop->dorLeft);
		dorTop->dorLeft = NULL;
	}

	vs_pool_free(&sm_pool, (void *)dorTop);
	topop->routingModule->data = NULL;
}

static Status_t
_copy(struct _RoutingModule * dest, const struct _RoutingModule * src)
{
    memcpy(dest, src, sizeof(RoutingModule_t));

	// Don't copy routing module data.  This will be setup and compared
	// against old data.
	dest->data = NULL;

	return VSTATUS_OK;
}


static Status_t
_make_routing_module(RoutingModule_t * rm)
{
	sm_shortestpath_make_routing_module(rm);

	rm->name = "dor";
	rm->funcs.pre_process_discovery = _pre_process_discovery;
	rm->funcs.discover_node = _discover_node;
	rm->funcs.discover_node_port = _discover_node_port;
	rm->funcs.post_process_discovery = _post_process_discovery;
	rm->funcs.post_process_routing = _post_process_routing;
	rm->funcs.post_process_routing_copy = _post_process_routing_copy;
	rm->funcs.setup_pgs = _setup_pgs;
	rm->funcs.calculate_lft = _calculate_lft;
	rm->funcs.init_switch_lfts = _init_switch_lfts;
	rm->funcs.get_port_group = _get_port_group;
	rm->funcs.setup_xft = _setup_xft;
	rm->funcs.select_scsc_map = _generate_scsc_map;
	rm->funcs.process_swIdx_change = _process_swIdx_change;
	rm->funcs.destroy = _destroy;
	rm->copy = _copy;

	return VSTATUS_OK;
}

Status_t
sm_dor_init(Topology_t *topop)
{
	return sm_routing_addModuleFac("dor", _make_routing_module);
}

