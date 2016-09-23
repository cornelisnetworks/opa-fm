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

/* Idea of sm_UpDnBaseLid is to be able to control if Up/Dn Lid is the base lid. 
 * Currently it is required that sm_UpDnBaseLid be 1 i.e. Up/Dn Lid is the base lid,
 * because for fabric programming using mixed LR-DR SMPs, we use the base lid.
 * If the base lid is a DOR lid, there could be problems during fabric disruptions.
 * Hence currently we need that base lid is Up/Dn Lid.
 *
 * In the future, if we can have the up/dn lid as the last lid instead of the base
 * lid, then we can turn this into configurable option.
 */

static uint32_t	sm_UpDnBaseLid = 1;

//When switches disappear from the fabric and sm_compactSwitchSpace() runs
//it changes switch indices and the max switch count. This compaction is
//done after the closure bit maps were allocated and set based on
//the old max switch count. Though the closure bit maps are altered
// to take into account the change in switch indices (done in  process_swIdx_change),
// the bit map space isn't reallocated, so any index calculations should be based
//on the old count of switches when the closure was computed

#define	DorBitMapsIndex(X, Y)	(((X) * (((DorTopology_t *)(sm_topop->routingModule->data))->closure_max_sws)) + (Y))

typedef struct _detected_dim {
	uint8_t		dim;
	uint64_t	neighbor_nodeGuid;
	uint8_t		port;
	uint8_t		neighbor_port;
	int			pos;
	Node_t		*neighbor_nodep;
} detected_dim_t;

#define PORT_PAIR_WARN_ARR_SIZE	40
//pointer to block of memory treated as a two-dimenstional array of size PORT_PAIR_WARN_SIZE * PORT_PAIR_WARN_SIZE
//to keep track of number of warnings for each port pair combination.
uint8_t *port_pair_warnings;
#define PORT_PAIR_WARN_IDX(X, Y)	(((X) * PORT_PAIR_WARN_ARR_SIZE) + (Y))

#define SM_DOR_MAX_TOROIDAL_DIMENSIONS 6

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

typedef struct _DorDimension {
	uint8_t ingressPort;
	uint8_t dimension;
	int8_t direction;
	uint8_t hyperlink;
	cl_map_obj_t portObj;
} DorDimension_t;

typedef struct _DorDiscoveryState {
	uint8_t nextDimension;
	uint8_t toroidalOverflow;
	uint8_t scsAvailable; // min of SC support of all fabric ISLs
	DorDimension_t *dimensionMap[256]; // indexed by egress port
} DorDiscoveryState_t;

static Status_t
_create_dimension(DorDiscoveryState_t *state, uint8_t p, uint8_t q, DorDimension_t **outDim)
{
	Status_t status;
	DorDimension_t *dim;

	// add forward direction
	status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
		return status;
	}

	dim->ingressPort = q;
	dim->dimension = state->nextDimension;
	dim->direction = 1;
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
		dim->direction = -1;
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

typedef enum {
	UPDN_LOOKUP_RVAL_FOUND,
	UPDN_LOOKUP_RVAL_NOTFOUND,
	UPDN_LOOKUP_RVAL_INVALID
} UpdnLookupRval_t;

static UpdnLookupRval_t
_lookup_dimension(DorDiscoveryState_t *state, uint8_t p, uint8_t q, DorDimension_t **outDim)
{
	if (state->dimensionMap[p] == NULL)
		return UPDN_LOOKUP_RVAL_NOTFOUND;
	
	if (state->dimensionMap[p]->ingressPort != q)
		return UPDN_LOOKUP_RVAL_INVALID;

	*outDim = state->dimensionMap[p];
	return UPDN_LOOKUP_RVAL_FOUND;
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
	UpdnLookupRval_t rval = UPDN_LOOKUP_RVAL_NOTFOUND;

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
		if (rval == UPDN_LOOKUP_RVAL_FOUND) {
			*dimension = dim->dimension;
			*direction = dim->direction;	
			return 1;	
		}
	}
	return 0;
}

static void disable_isl_ports(Topology_t *topop, Node_t *nodep, Port_t *portp, Node_t *neighborNodep)
{
	Port_t *neighborPortp = sm_find_node_port(topop, neighborNodep, portp->portno);

	if (sm_valid_port(portp)) {
		portp->state = IB_PORT_DOWN;
		DECR_PORT_COUNT(topop, nodep);
	}

	if (sm_valid_port(neighborPortp)) {
		neighborPortp->state = IB_PORT_DOWN;
		DECR_PORT_COUNT(topop, neighborNodep);
	}

	topology_changed = 1;
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
	UpdnLookupRval_t rval;
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
	case UPDN_LOOKUP_RVAL_FOUND:
		// success
		break;
	case UPDN_LOOKUP_RVAL_NOTFOUND:
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

			//mark config information that the dimension has been created
			config_dim = get_configured_dimension(portp->index, portp->portno);
			if (config_dim >= 0) {
				smDorRouting.dimension[config_dim].created = 1;
			}
		}
		break;
	case UPDN_LOOKUP_RVAL_INVALID:
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

		if (neighborDorNode->coords[dim->dimension] < dorTop->coordMinimums[dim->dimension])
			dorTop->coordMinimums[dim->dimension] = neighborDorNode->coords[dim->dimension];

		else if (neighborDorNode->coords[dim->dimension] > dorTop->coordMaximums[dim->dimension])
			dorTop->coordMaximums[dim->dimension] = neighborDorNode->coords[dim->dimension];

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
						IB_LOG_INFINI_INFO_FMT(__func__,
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
					disable_isl_ports(topop, nodep, portp, neighborNodep);
				}
			} else if ((neighborDorNode->coords[dim->dimension] - dorNode->coords[dim->dimension]) > 1) {
				/* This is a wrap around link that is not between the maximum and minimum co-ordinates */
				IB_LOG_WARN_FMT(__func__, "Disabling wrap around link between "
	 					        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
								" as this link is not between the end nodes of this dimension",
				 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	               			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);

				disable_isl_ports(topop, nodep, portp, neighborNodep);
			}
		} else {
			if (dorNode->coords[dim->dimension] == dorTop->coordMinimums[dim->dimension] &&
			    neighborDorNode->coords[dim->dimension] == dorTop->coordMaximums[dim->dimension]) {
				if (is_configured_toroidal(portp->index, portp->portno)) {
					if (smDorRouting.debug) {
						IB_LOG_INFINI_INFO_FMT(__func__,
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
					disable_isl_ports(topop, nodep, portp, neighborNodep);
				}
			} else if ((dorNode->coords[dim->dimension] - neighborDorNode->coords[dim->dimension]) > 1) {
				/* This is a wrap around link that is not between the maximum and minimum co-ordinates */
				IB_LOG_WARN_FMT(__func__, "Disabling wrap around link between "
	 					        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
								" as this link is not between the end nodes of this dimension",
				 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	               			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);
				disable_isl_ports(topop, nodep, portp, neighborNodep);
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
// UP/DOWN SPANNING TREE
//

typedef struct _UpdnSpanningNode {
	struct _UpdnSpanningNode *parent;
	uint8_t	parent_portno;
	Node_t *nodep;
} UpdnSpanningNode_t;

typedef struct _UpdnSpanningTree {
	UpdnSpanningNode_t *nodes;
	size_t numNodes;
	uint32_t *topoToUpdnIndexMap;
} UpdnSpanningTree_t;

UpdnSpanningTree_t updn_spanning_tree;

static Status_t
_build_ideal_spanning_tree(Topology_t *tp, McSpanningTree_t *mcST, int *complete)
{
	int					i;
	McNode_t			*mcNodes;
	Node_t				*nodep;
	Status_t			status;

	if (!mcST)
		return VSTATUS_BAD;

	/*
     * Initialize the nodes.
	 */
	i = 0;
	mcNodes = mcST->nodes;
	for_all_switch_nodes(sm_topop, nodep) {
		if (sm_useIdealMcSpanningTreeRoot &&
			 (sm_mcSpanningTreeRootGuid == nodep->nodeInfo.NodeGUID)) {
			sm_mcSpanningTreeRootSwitch = nodep;
		}
		mcNodes[i].index = nodep->index;
		mcNodes[i].nodeno = -1;
		mcNodes[i].portno = -1;
        mcNodes[i].parent = NULL;
		mcNodes[i].mft_mlid_init = 0;
		nodep->mcSpanningChkDone = 0;
		i++;
	}

	if (sm_useIdealMcSpanningTreeRoot) {
		if (sm_mcSpanningTreeRootSwitch) {
			status = sm_ideal_spanning_tree(mcST, 0, 0, 0, tp->num_sws, complete);
			return status;
		}
	}

	if (!sm_mcSpanningTreeRootSwitch)
		IB_LOG_INFINI_INFO_FMT(__func__, "No Ideal MC spanning tree root switch. sm_useIdealMcSpanningTreeRoot=%d",
								sm_useIdealMcSpanningTreeRoot);

	return VSTATUS_BAD;
}

static Status_t
_copy_ideal_tree_to_updn_tree(UpdnSpanningTree_t *tree, McSpanningTree_t *mcST)
{
	UpdnSpanningNode_t *updnNodes;
	McNode_t		   *mcNodes;
	Node_t			   *nodep, *parent_nodep;
	int 			   i;

	if (!tree || !mcST)
		return VSTATUS_BAD;

	updnNodes = tree->nodes;
	mcNodes = mcST->nodes;

	if (!updnNodes || !mcNodes)
		return VSTATUS_BAD;
	
	if (tree->numNodes < mcST->num_nodes) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Cannot copy Updn tree numNodes (%"PRISZT") < mcST numNodes (%d)",
			tree->numNodes, mcST->num_nodes);
		return VSTATUS_BAD;
	}

	tree->numNodes = mcST->num_nodes;

	/* setup the updn tree node pointers and the index map first*/
	for (i = 0; i < mcST->num_nodes; i++) {
		nodep = sm_find_node(sm_topop, mcNodes[i].index);
		if (!nodep) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Error in finding node with index %d", mcNodes[i].index);
			return VSTATUS_BAD;
		}
		updnNodes[i].nodep = nodep;
		tree->topoToUpdnIndexMap[nodep->swIdx] = i;
	}

	/*setup the updn tree parent pointers using the index map*/
	for (i = 0; i < mcST->num_nodes; i++) {
		if (mcNodes[i].portno == -1)
			continue;

		updnNodes[i].parent_portno = mcNodes[i].portno;

		parent_nodep = sm_find_node(sm_topop, mcNodes[i].nodeno);	

		if (!parent_nodep) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Error in finding parent node with index %d", mcNodes[i].nodeno);
			return VSTATUS_BAD;
		}
	
		updnNodes[i].parent = tree->nodes + tree->topoToUpdnIndexMap[parent_nodep->swIdx];
	}

	return VSTATUS_OK;
}

static Status_t
_build_mc_style_tree(Topology_t *tp, UpdnSpanningTree_t *tree)
{
	McSpanningTree_t	*mcST;
	Status_t			status;
	int					num_nodes = 0;
	int					complete = 0;
	uint32_t			bytes = 0;

    /*
     * Count the nodes.
     */
	num_nodes = tp->num_sws; 
    /*
     * Allocate space for the nodes.
     */
	bytes = 32 + sizeof(McSpanningTree_t) + (num_nodes * sizeof(McNode_t));
	if (vs_pool_alloc(&sm_pool, bytes, (void **)&mcST) != VSTATUS_OK) {
		IB_EXIT(__func__, VSTATUS_BAD);
		return(VSTATUS_NOMEM);
	}

	memset((void *)mcST, 0, bytes);

	mcST->num_nodes = num_nodes;
	mcST->nodes = (McNode_t *)(mcST+1);

	/* build a spanning tree using the same root as multicast. If every link supports atleast 2K MTU
	 * then the updn spanning tree getting built here will match the 2K MTU spanning tree and that
	 * will ensure that no credit loops will be caused due to the mix of updn unicast and general
	 * multicast traffic.
	 */

	status = _build_ideal_spanning_tree(sm_topop, mcST, &complete);

	if (status != VSTATUS_OK) {
		IB_LOG_INFINI_INFO_FMT(__func__,
				 "Error in building ideal spanning tree for updn");
		goto exit;
	}

	if (complete) {
		status = _copy_ideal_tree_to_updn_tree(tree, mcST);
		goto exit;	
	}

exit:
	vs_pool_free(&sm_pool, (void *)mcST);
	if (!complete) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Could not build a MC style complete spanning tree for UPDN");
		status = VSTATUS_BAD;
	}
	return status;
}

static Status_t
_build_updn_spanning_tree(Topology_t *topop, UpdnSpanningTree_t *tree)
{
	Status_t status;
	int i;
	Node_t *nodep;
	Port_t *portp;
	UpdnSpanningNode_t *parent, *child;

	tree->numNodes = topop->num_sws;
	status = vs_pool_alloc(&sm_pool, tree->numNodes * sizeof(UpdnSpanningNode_t),
		(void *)&tree->nodes);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate spanning tree; rc:", status);
		tree->nodes = NULL;
		tree->numNodes = 0;
		tree->topoToUpdnIndexMap = NULL;
		return status;
	}
	memset((void *)tree->nodes, 0, tree->numNodes * sizeof(UpdnSpanningNode_t));

	// here we just overallocate up to the allowed sparcity of
	// the switch index space
	status = vs_pool_alloc(&sm_pool, topop->max_sws * sizeof(uint32_t),
		(void *)&tree->topoToUpdnIndexMap);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate index map; rc:", status);
		vs_pool_free(&sm_pool, (void *)tree->nodes);
		tree->nodes = NULL;
		tree->numNodes = 0;
		tree->topoToUpdnIndexMap = NULL;
		return status;
	}
	memset((void *)tree->topoToUpdnIndexMap, 0, topop->max_sws * sizeof(uint32_t));

	if (smDorRouting.updn_mc_same_spanning_tree && sm_useIdealMcSpanningTreeRoot) {
		/* If sm_useIdealMcSpanningTreeRoot is 1, then use the multicast spanning
		   tree building logic to build the updn spanning tree.
		   If sm_useIdealMcSpanningTreeRoot is 0, then multicast is using SM Neighbor.
		   In that case, we will fall back to using the default updn spanning tree building
		   logic for  building the spanning tree which is the same as SM neighbor */
		if (smDebugPerf) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Using multicast spanning tree settings to build Up-Down spanning tree");
		}
		status = _build_mc_style_tree(topop, tree);
		if (status == VSTATUS_OK) {
			return VSTATUS_OK;
		} else {
			IB_LOG_WARN0("Unable to build a Multicast similar spanning tree. Falling back to default spanning tree");
		}
	}

	if (smDebugPerf) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Building Up-Down spanning tree using switch next to the SM as root");
	}
	// link the spanning tree to the topology and create an index mapping
	i = 0;
	for_all_switch_nodes(topop, nodep) {
		tree->nodes[i].nodep = nodep;
		tree->topoToUpdnIndexMap[nodep->swIdx] = i;
		++i;
	}

	// build the spanning tree by linking the parent fields
	for (i = 0; i < tree->numNodes; ++i) {
		parent = tree->nodes + i;
		for_all_physical_ports(parent->nodep, portp) {
//			if (!sm_valid_port(portp) || portp->state < IB_PORT_ACTIVE)
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
				continue;

			nodep = sm_find_node(topop, portp->nodeno);
			if (nodep == NULL || nodep->nodeInfo.NodeType != NI_TYPE_SWITCH)
				continue;

			if (parent->parent && parent->parent->nodep == nodep)
				continue;

			child = tree->nodes + tree->topoToUpdnIndexMap[nodep->swIdx];
			if (  child != tree->nodes // don't give the root node a parent
			   && child->parent == NULL) {
				child->parent = parent;
				child->parent_portno = portp->index;
			}
		}
	}

	return VSTATUS_OK;
}

static void
_free_spanning_tree(UpdnSpanningTree_t *tree)
{
	if (tree->numNodes == 0)
		return;
		
	if (tree->nodes != NULL) {
		vs_pool_free(&sm_pool, (void *)tree->nodes);
		tree->nodes = NULL;
	}

	if (tree->topoToUpdnIndexMap != NULL) {
		vs_pool_free(&sm_pool, (void *)tree->topoToUpdnIndexMap);
		tree->topoToUpdnIndexMap = NULL;
	}
}

static Status_t
_calc_updn_closure(Topology_t *topop)
{
	Status_t status;
	int i, j, ij;
	size_t s;
	UpdnSpanningTree_t *tree;
	UpdnSpanningNode_t *curr, *parent, *prev;
	UpdnSpanningNode_t *n1, *n2, *p1, *p2;
	uint16_t cost = 0, cost1 = 0, cost2 = 0;
	Port_t *portp;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	tree = &updn_spanning_tree;

	memset((void *)tree, 0, sizeof(*tree));

	if (((DorTopology_t*)topop->routingModule->data)->alternateRouting) {
		status = _build_updn_spanning_tree(topop, tree);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to build Up/Down spanning tree; rc:", status);
			return status;
		}
	}

	s = (sizeof(uint32_t) * topop->max_sws * topop->max_sws / 32) + 1;
	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTop->updnDownstream);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for Up/Down closure array; rc:", status);
		_free_spanning_tree(tree);
		return status;
	}
	memset((void *)dorTop->updnDownstream, 0, s);

	if (!tree->nodes) {
		return VSTATUS_OK;
	}

	// mark each node as downstream of all ancestors
	for (i = 0; i < tree->numNodes; ++i) {
		curr = tree->nodes + i;
		parent = curr->parent;
		cost = 0;
		prev = curr;
		while (parent != NULL) {
			ij = DorBitMapsIndex(parent->nodep->swIdx, curr->nodep->swIdx);
			dorTop->updnDownstream[ij >> 5] |= 1 << (ij & 0x1f);
			/* While calculating DOR closure, floyd costs are set to infinity if there
			 * is no DOR closure. Set the cost from curr to all its parents in the Up/Dn
			 * spanning tree if the cost is infinity.
			 * Note - floyd costs are not used in DOR or Up/Dn but are used by the job management code.
			 */
			portp = sm_get_port(parent->nodep, prev->parent_portno);
			if (sm_valid_port(portp)) {
				cost += sm_GetCost(portp->portData);
			}

			if (topop->cost[Index(parent->nodep->swIdx, curr->nodep->swIdx)] == Cost_Infinity) {
					topop->cost[Index(parent->nodep->swIdx, curr->nodep->swIdx)] = cost;
					topop->cost[Index(curr->nodep->swIdx, parent->nodep->swIdx)] = cost;
			}

			prev = parent;
			parent = parent->parent;
		}
	}

	/* Nodes that share a parent-child relationship and have a Cost_Infinity have been
	 * handled above. Any remaining pairings that have a Cost_Infinity will 
	 * be ones that do not share a parent-child relationship i.e each node is in
	 * a separate sub tree. Check for those now.
	 */
	for (i = 0; i < tree->numNodes; ++i) {
		n1 = tree->nodes + i;
		if (!(n1->parent))
			continue;
		for (j = i+1; j < tree->numNodes; ++j) {
			n2 = tree->nodes + j;
			if (topop->cost[Index(n1->nodep->swIdx, n2->nodep->swIdx)] != Cost_Infinity)
				continue;
			/* cost between n1, n2 will be the sum of costs to the common parent from each of
			 * the nodes as that will be the path connecting these two nodes.
			 * To find common parent, check the parents of n1 till we find a parent which
			 * is up stream of n2.
			 */
			p1 = n1->parent;
			cost1 = 0;
			prev = n1;
			while (p1) {
				portp = sm_get_port(p1->nodep, prev->parent_portno);
				if (sm_valid_port(portp)) {
					cost1 += sm_GetCost(portp->portData);
				}

				ij = DorBitMapsIndex(p1->nodep->swIdx, n2->nodep->swIdx);
				if ((dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f)))) {
					break;
				}
				prev = p1;
				p1 = p1->parent;
			}

			p2 = n2->parent;
			cost2 = 0;
			prev = n2;

			while (p2) {
				portp = sm_get_port(p2->nodep, prev->parent_portno);
				if (sm_valid_port(portp)) {
					cost2 += sm_GetCost(portp->portData);
				}
				if (p2 == p1)
					break;
				prev = p2;
				p2 = p2->parent;
			} 
	
			if (p2 == p1) {
				cost = cost1 + cost2;
				topop->cost[Index(n1->nodep->swIdx, n2->nodep->swIdx)] = cost;
				topop->cost[Index(n2->nodep->swIdx, n1->nodep->swIdx)] = cost;
			}
		}
	}

	_free_spanning_tree(tree);

	return VSTATUS_OK;
}

static Status_t
_copy_updn_closure(Topology_t *src_topop, Topology_t *dst_topop)
{
	Status_t status;
	DorTopology_t	*dorTopSrc = (DorTopology_t *)src_topop->routingModule->data;
	DorTopology_t	*dorTopDst = (DorTopology_t *)dst_topop->routingModule->data;
	size_t s, max_sws = dorTopSrc->closure_max_sws;

//	s = (sizeof(uint32_t) * dst_topop->max_sws * dst_topop->max_sws / 32) + 1;
	dorTopDst->closure_max_sws= max_sws;
	s = (sizeof(uint32_t) * max_sws * max_sws / 32) + 1;
	status = vs_pool_alloc(&sm_pool, s,
		(void *)&dorTopDst->updnDownstream);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for up/down closure array; rc:", status);
		return status;
	}

	memcpy((void *)dorTopDst->updnDownstream, (void *)dorTopSrc->updnDownstream, s);

	return VSTATUS_OK;
}
void _prune_conn_swlist(Topology_t *topop, Node_t *nodep, SwitchList_t **conn_swlist, SwitchList_t **tail)
{
	SwitchList_t *csw = *conn_swlist, *prev_csw;
	int ij, include, isDownstream;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	*tail = NULL;

	while (csw) {
		/* out of the connected switches, we should consider only those that have a direct connection
		 * in the updn spanning tree i.e those that are children or a parent of nodep - since only
		 * those would be base-LID routable via updn from nodep independent of other switches.
		 */
		include = 0;
		/* check if csw->switchp is parent of nodep*/
		ij = DorBitMapsIndex(nodep->swIdx, csw->switchp->swIdx);
		isDownstream = (dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
		if (isDownstream) {
			include = 1;
		} else {
			/* check if csw->switchp is child of nodep*/
			ij = DorBitMapsIndex(csw->switchp->swIdx, nodep->swIdx);
			isDownstream = (dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
			if (isDownstream) {
				include = 1;
			}
		}
		if (!include) {
			/* Remove csw from conn_swlist*/
			if (csw == *conn_swlist) {
				/* If removing head, change the head*/
				*conn_swlist = csw->next;
			} else {
				if (*tail) {
					/* tail is pointing to the previous switch in the
					   linked list. Point it to csw's next to effectively
					   remove csw from the list */
					(*tail)->next = csw->next;
				}
			}
			prev_csw = csw;
			csw = csw->next;
			free(prev_csw);
		} else {
			*tail = csw;
			csw = csw->next;
		}
	}
}

/* Programs switches in LR-DR wave by traversing the updn spanning tree starting at
 * the head switch (switch next to SM) and going through the connected switches.
 * Head switch must already have been programmed before this function is called.
 */
Status_t _setup_switches_lrdr_wave_updn_order(Topology_t *topop, int rebalance, int routing_needed)
{
	SwitchList_t *sw, *conn_swlist = NULL;
	SwitchList_t *head_switch, *swlist_tail, *conn_swlist_tail, *prev_sw, *s;
	Node_t		 *nodep;
	Status_t	 status;

	head_switch = (SwitchList_t *) malloc(sizeof(SwitchList_t));
	if (head_switch == NULL) {
		IB_LOG_ERROR0("can't malloc node !");
		return VSTATUS_NOMEM;
	}

	head_switch->switchp = topop->switch_head;
	head_switch->next = NULL;

	sw = head_switch;
	swlist_tail = head_switch;

	while (sw) {
		conn_swlist_tail = NULL;
		nodep = sw->switchp;
		/* Get list of all switches connected to this switch, which we have not yet initialized*/
		status = sm_get_uninit_connected_switch_list(sm_topop, nodep, &conn_swlist);

		if (status != VSTATUS_OK) {
			IB_LOG_INFINI_INFO_FMT(__func__,
					"failed to get list of child switches for switch "FMT_U64, nodep->nodeInfo.NodeGUID);
			sm_delete_switch_list(sw);
			sm_delete_switch_list(conn_swlist);
			return status;
		}

		if (conn_swlist) {
			/* prune connected list based on the updn spanning tree to include only
			 * those switches which are directly connected to nodep in the updn spanning tree.
			 * This will make sure we can program those switches from nodep with a one-hop DR
			 */
			_prune_conn_swlist(sm_topop, nodep, &conn_swlist, &conn_swlist_tail);
		}

		/* add the connected swlist to the main switch list, to form the crest of the next wave*/
		if (swlist_tail)
			swlist_tail->next = conn_swlist;

		if (conn_swlist_tail)
			swlist_tail = conn_swlist_tail; /* remember the new end of the switch list*/
		prev_sw = sw;
		sw = sw->next;
		free(prev_sw); 
		if (swlist_tail == prev_sw) swlist_tail = NULL;

		if (!conn_swlist)
			continue;

		/* First initialize port 0 of switch and setup minimal LFTs if routing is required*/
		if ( (status = sm_prep_switch_layer_LR_DR(sm_topop, nodep, conn_swlist, rebalance)) != VSTATUS_OK) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Basic LFT programming using LR/DR SMPs"
					"failed for switches connected to "FMT_U64" status %d",	nodep->nodeInfo.NodeGUID, status);
			sm_delete_switch_list(sw);
			return status;

		}
		
		if (routing_needed) {
			/* Setup full LFTs*/
			if ((status = sm_routing_route_switch_LR(sm_topop, conn_swlist, rebalance)) != VSTATUS_OK) {
				IB_LOG_INFINI_INFO_FMT(__func__,
						 "Full LFT programming failed for switches connected to "FMT_U64" status %d",
						 nodep->nodeInfo.NodeGUID, status);
				sm_delete_switch_list(sw);
				return status;
			}
		}

		for_switch_list_switches(conn_swlist, s) {
			s->switchp->initDone = 1;
		}
	}

	return status;
}

//===========================================================================//
// DOR UTILITIES
//

static Node_t *
_step_to(Topology_t *topop, Node_t *src, Node_t *dst)
{
	int i, si = -1, di = -1;
	DorNode_t *dnp;
	DorNode_t *srcDnp = (DorNode_t*)src->routingData;
	DorNode_t *dstDnp = (DorNode_t*)dst->routingData;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	// travel along the first dimension that the source
	// does not currently share with the destination
	for (i = 0; i < dorTop->numDimensions; ++i) {
		si = srcDnp->coords[i];
		di = dstDnp->coords[i];
		if (si < di) {
			// source has smaller index; would normally go right
			if (dorTop->toroidal[i] && srcDnp->left[i] &&
			    si + dorTop->dimensionLength[i] - di < di - si) {
				// topology is toroidal and crossing over the boundary
				// is quicker, so go left instead
				dnp = srcDnp->left[i];
				return dnp == NULL ? NULL : dnp->node;
			} else {
				// not toroidal or toroidal link is down, or normal route; go right
				dnp = srcDnp->right[i];
				return dnp == NULL ? NULL : dnp->node;
			}
		} else if (si > di) {
			// source has larger index; would normally go left
			if (dorTop->toroidal[i] && srcDnp->right[i] &&
			    di - si + dorTop->dimensionLength[i] < si - di) {
				// topology is toroidal and crossing over the boundary
				// is quicker, so go right instead
				dnp = srcDnp->right[i];
				return dnp == NULL ? NULL : dnp->node;
			} else {
				// not toroidal or toroidal link is down, or normal route; go left
				dnp = srcDnp->left[i];
				return dnp == NULL ? NULL : dnp->node;
			}
		}
	}

	// coords ended up the same?  eh... return the destination
	if ((si != -1) && (di != -1)) {
		if (si == di)
			return dst;
	}

	return NULL;
}

static int
_is_path_realizable(Topology_t *topop, Node_t *src, Node_t *dst)
{
	Node_t *n;
	int step = 0, dim = -1, i;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;
	DorNode_t *srcDnp = (DorNode_t*)src->routingData;
	DorNode_t *dstDnp = (DorNode_t*)dst->routingData;
	DorNode_t *nDnp = NULL;

	if (src == dst) return 1;

	// just step from neighbor to neighbor until we find a hole.
	// if we arrive at the destination, the DOR path is good
	n = _step_to(topop, src, dst);
	while (n != dst) {
		if (n == NULL)
			return 0;

		nDnp = (DorNode_t*)n->routingData;

		/* find the dimension on which we are traveling and count the number of steps */
		for (i = 0; i < dorTop->numDimensions; ++i) {
			if (nDnp->coords[i] == dstDnp->coords[i]) {
				if (i == dim) {
					step = 0;
					dim = -1;
					break;
				}
				continue;
			}
	
			/* Check if we have looped around in this dimenion. */
			/* Can happen in irregular DOR topologies which can get created after disruptions */

			if (n == src) {
					return 0;
			}
		
			if (nDnp->coords[i] != srcDnp->coords[i]) {
				dim = i;
				step++;
				/* If we have taken more steps than the length of the dimension, then we are looping */
				if (step > dorTop->dimensionLength[i]) {
					return 0;
				}
				break;
			}
		}

		n = _step_to(topop, n, dst);
	}
	return 1;
}

// TBD: is the following required?
// Here I'm defining the toroidal-ness of a path along a given dimension
// to hold if that path would cross the wrap-around edge of the mesh.
//
static __inline__ int
_is_path_toroidal_in_dimension(Topology_t *topop, Node_t *srcNodep, Node_t *dstNodep, int dimension)
{
	int cs, cd, toroidal;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	// can't be toroidal unless this dimension is wired as such
	if (!dorTop->toroidal[dimension])
		return 0;
	
	cs = ((DorNode_t*)(srcNodep->routingData))->coords[dimension];
	cd = ((DorNode_t*)(dstNodep->routingData))->coords[dimension];

	// not toroidal if the path would not take use over the wrap-around edge
	if (cs == cd || (cs < cd && cd - cs <= (dorTop->dimensionLength[dimension] + cs - cd)) ||
	    (cs > cd && cs - cd <= (dorTop->dimensionLength[dimension] + cd - cs)))
		toroidal = 0;
	else
		toroidal = 1;

	if (smDorRouting.debug)
		IB_LOG_VERBOSE_FMT(__func__,
		       "Checking path for toroidal property in dimension %d, src %d, dst %d: toroidal? %d\n",
		       dimension, cs, cd, toroidal);

	// definitely toroidal in this dimension
	return toroidal;
}

//===========================================================================//
// DOR CLOSURE CALCULATION
//

static Status_t
_calc_dor_closure(Topology_t *topop)
{
	Status_t status;
	int ij;
	size_t s;
	Node_t *ni, *nj;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	s = (sizeof(uint32_t) * topop->max_sws * topop->max_sws / 32) + 1;
	dorTop->dorClosureSize = s;
	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTop->dorClosure);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}
	memset((void *)dorTop->dorClosure, 0, s);

	for_all_switch_nodes(topop, ni) {
		for_all_switch_nodes(topop, nj) {
			if (ni == nj || _is_path_realizable(topop, ni, nj)) {
				if (smDorRouting.debug)
					IB_LOG_VERBOSE_FMT(__func__,
					       "Path from %d [%s] to self is realizable",
					       ni->swIdx, sm_nodeDescString(ni));
				ij = DorBitMapsIndex(ni->swIdx, nj->swIdx);
				dorTop->dorClosure[ij >> 5] |= 1 << (ij & 0x1f);
			} else {
				 /* Set floyd cost to Infinity for this pair as there is no DOR closure */
				 /* The cost will be set later on based on the Up/Dn spanning tree calculation */
				 /* Note - floyd costs are not used in DOR or Up/Dn but are used by the job management code. */
				topop->cost[Index(ni->swIdx, nj->swIdx)] = Cost_Infinity;
				topop->cost[Index(nj->swIdx, ni->swIdx)] = Cost_Infinity;

				if (smDorRouting.debug)
					IB_LOG_INFINI_INFO_FMT(__func__,
					       "Path from %d [%s] to %d [%s] is NOT realizable",\
					       ni->swIdx, sm_nodeDescString(ni), nj->swIdx, sm_nodeDescString(nj));
			}
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
	status = vs_pool_alloc(&sm_pool, s, (void *)&dorTopDst->dorClosure);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}

	memcpy((void *)dorTopDst->dorClosure, (void *)dorTopSrc->dorClosure, s);

	return VSTATUS_OK;
}

static __inline__ boolean
isDatelineSwitch(Topology_t *topop, Node_t *switchp, uint8_t dimension)
{
	return (((DorTopology_t *)topop->routingModule->data)->toroidal[dimension] &&
			((DorNode_t*)switchp->routingData)->coords[dimension] == 0);
}

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

	if (getSecondary) 
		return VSTATUS_OK;

	memset(portDim, 0xff, sizeof(uint8_t)*(switchp->nodeInfo.NumPorts+1));
	memset(portPos, 0, sizeof(uint8_t)*(switchp->nodeInfo.NumPorts+1));

	// Setup illegal turn to drop
	memset(&scscBadTurn, 15, sizeof(scscBadTurn));

	// Setup 1:1 mapping
	memset(&scscNoChg, 15, sizeof(scscNoChg));
	for (i=0; i<STL_MAX_SCS; i++) {
		if (bitset_test(&sm_linkSLsInuse, sm_SCtoSL[i]))
			scscNoChg.SCSCMap[i].SC = i;
	}

	// Setup mapping for moving back to first SC mapped to SL
	memset(&scsc0, 15, sizeof(scsc0));
	for (i=0; i<STL_MAX_SCS; i++) {
		if ((i==15) || (sm_SCtoSL[i] == 15)) {
			continue;
		}
		if (bitset_test(&sm_linkSLsInuse, sm_SCtoSL[i])) {
			scsc0.SCSCMap[i].SC = sm_SLtoSC[sm_SCtoSL[i]];
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

	// Setup mapping to increment to next SC in group
	memset(&scscPlus1, 15, sizeof(scscPlus1));
	for (i=0; i<STL_MAX_SCS; i++) {
		if (i==15) continue;
		if (!bitset_test(&sm_linkSLsInuse, sm_SCtoSL[i])) continue;
		// Set 1:1
		scscPlus1.SCSCMap[i].SC = i;
		for (j=i+1; j<STL_MAX_SCS; j++) {
			if (j==15) continue;
			if (sm_SCtoSL[i] == sm_SCtoSL[j]) {
				// next SC which is mapped to this SL
				scscPlus1.SCSCMap[i].SC = j;
				break;
			}
		}
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

static Status_t
_get_outbound_port_dor(Topology_t *topop, Node_t *switchp, Node_t *endNodep,
                       Port_t *endPortp, uint8_t *portnos)
{
	int i, j;
	Node_t *neighborNodep = NULL, *nodep = NULL, *endSwitchp = NULL;
	Port_t *portp;
	int endPort = 0;
	uint16_t curSpeed = 0;
	uint16_t bestSpeed = 0;
	uint16_t bestLidsRouted = 0xffff;
	uint8_t numLids = 1 << endPortp->portData->lmc;
	int lidsRoutedInc = ((endNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) ? 1 : 0);
	int offset;
	SwitchportToNextGuid_t *orderedPorts = (SwitchportToNextGuid_t *)topop->pad;

	if (((DorTopology_t*)topop->routingModule->data)->alternateRouting && endPortp->portData->lmc > 0) {
		numLids--;		//leave one lid for updn programming
		if (sm_UpDnBaseLid) {
			//leave base lid for updn programming
			portnos++;		
		}
	}

	memset((void*)portnos, 0xff, sizeof(uint8_t) * numLids);

	if (((DorTopology_t*)topop->routingModule->data)->alternateRouting && endPortp->portData->lmc == 0) {
		/* Both Up/Dn and DOR are not possible for this port, as end port has LMC of 0*/
		/* Default to using Up/Dn */
		/* Switch port 0 can have an LMC of 0 even if LMC is enabled */
		goto end;
	}

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

	neighborNodep = _step_to(topop, switchp, endSwitchp);
	if (neighborNodep == NULL) {
		IB_LOG_NOTICE_FMT(__func__,
		       "Failed to find neighbor in path from NodeGUID "FMT_U64" [%s] to NodeGUID "FMT_U64" [%s]",
		       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endNodep->nodeInfo.NodeGUID, sm_nodeDescString(endNodep));
		return VSTATUS_BAD;
	} else {
		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "[%s]: Found neighbor NodeGUID "FMT_U64" [%s]",
			       sm_nodeDescString(switchp), neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep));
	}

	if (endPortp->portData->lmc == 0) {
		for_all_physical_ports(switchp, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
			if ((nodep = sm_find_node(topop, portp->nodeno)) == NULL) continue;
			if (nodep != neighborNodep) continue;
			curSpeed = sm_GetSpeed(portp->portData);
			if (curSpeed > bestSpeed) {
				bestSpeed = curSpeed;
				bestLidsRouted = portp->portData->lidsRouted;
				portnos[0] = portp->index;
			} else if (  curSpeed == bestSpeed
			          && portp->portData->lidsRouted < bestLidsRouted) {
				bestLidsRouted = portp->portData->lidsRouted;
				portnos[0] = portp->index;
			}
		}

		if (portnos[0] != 0xff && endNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			if (sm_valid_port(portp = sm_get_port(switchp, portnos[0]))) {
				portp->portData->lidsRouted++;
			} else {
				IB_LOG_ERROR_FMT(__func__,
				       "Failed to get port %d for NodeGUID "FMT_U64" [%s]",
				       portnos[0], switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));
			}
		}
	} else {
		for_all_physical_ports(switchp, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
			if ((nodep = sm_find_node(topop, portp->nodeno)) == NULL) continue;
			if (nodep != neighborNodep) continue;
			curSpeed = sm_GetSpeed(portp->portData);
			if (curSpeed >= bestSpeed) {
				if (curSpeed > bestSpeed) {
					bestSpeed = curSpeed;
					endPort = 0;
				}
				orderedPorts[endPort].portp = portp;
				orderedPorts[endPort].guid = nodep->nodeInfo.NodeGUID;
				orderedPorts[endPort].sysGuid = nodep->nodeInfo.SystemImageGUID;
				orderedPorts[endPort].nextSwp = nodep;
				++endPort;
			}
		}

		if (endPort) {
			qsort(orderedPorts, endPort, sizeof(SwitchportToNextGuid_t),
			      _compare_lids_routed);
			if (endPort > numLids) endPort = numLids;

			offset = sm_balance_base_lids(orderedPorts, endPort);
			
			for (i = 0; i < numLids; i++) {
				j = (i + offset) % endPort;
				portnos[i] = orderedPorts[j].portp->index;
				orderedPorts[j].portp->portData->lidsRouted += lidsRoutedInc;
			}
 			++orderedPorts[offset].portp->portData->baseLidsRouted;
		}
	}

end:
	if (portnos[0] == 0xff && smDebugPerf)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "Failed to setup LID 0x%.4X for switch %d, NodeGUID "FMT_U64" [%s]",
		       endPortp->portData->lid, switchp->index, switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));

	return VSTATUS_OK;
}

static Status_t
_get_outbound_port_updn(Topology_t *topop, Node_t *switchp, Node_t *endNodep,
                        Port_t *endPortp, uint8_t *portnos)
{
	int i, ij;
	Node_t *endSwitchp = NULL, *nodep = NULL;
	Port_t *portp;
	uint16_t curSpeed = 0;
	uint16_t bestSpeed = 0;
	uint16_t bestLidsRouted = 0xffff;
	uint8_t numLids = 1 << endPortp->portData->lmc;
	uint32_t wantDownstream, isDownstream;
//	uint16_t updnLid = endPortp->portData->lmc ? numLids - 1 : 0;
	uint16_t updnLid;
	uint8_t endSwitchFound = 0;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (sm_UpDnBaseLid) {
		updnLid = 0;
	} else {
		updnLid = endPortp->portData->lmc ? numLids - 1 : 0;
	}

	portnos[updnLid] = 0xff;

	if ((!sm_UpDnBaseLid) && (updnLid == 0)) {
		/* End port has LMC of 0 and base LID is being used for DOR. Can't do Up/Dn*/
		goto end;
	}


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

	ij = DorBitMapsIndex(switchp->swIdx, endSwitchp->swIdx);
	wantDownstream = dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f));
	if (wantDownstream)
		wantDownstream = 1;

	for_all_physical_ports(switchp, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
		if ((nodep = sm_find_node(topop, portp->nodeno)) == NULL) continue;
		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			// FIs are always downstream
			continue;
		} else if (nodep != endSwitchp){
			if (endSwitchFound && (nodep != endSwitchp)) {
				/* we already found a port that takes us to the destination switch,
				 * so ignore other ports and continue so that we can check if there
				 * are other ports that connect to the destination switch.
				 */
				continue;
			}
			ij = DorBitMapsIndex(switchp->swIdx, nodep->swIdx);
			isDownstream = (dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
			if (isDownstream)
				isDownstream = 1;
			if (wantDownstream != isDownstream) {
				continue;
			} else if (wantDownstream) {
				/* end switch is down stream to switchp  and nodep is also down stream to switchp*/
				/* check if endSwitchp is down stream to nodep */
				ij = DorBitMapsIndex(nodep->swIdx, endSwitchp->swIdx);
				isDownstream = (dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
				if (!isDownstream)
					continue;	//though nodep is down stream to switchp, it does not lead towards the end switch
								// check for other down stream switches of switchp
			} else if (!wantDownstream) {
				/* We have to go upstream. consider nodep only if it is upstream to switchp*/
				ij = DorBitMapsIndex(nodep->swIdx, switchp->swIdx);
				isDownstream = (dorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
				if (!isDownstream) {
					continue;
				}
			}
		} else if (nodep == endSwitchp) {
			if (!endSwitchFound) {
				/* first direct link we found to the destination switch */
				/* ignore any other ports we might have selected before */
				bestSpeed = sm_GetSpeed(portp->portData);
				bestLidsRouted = portp->portData->lidsRouted;
				portnos[updnLid] = portp->index;
				endSwitchFound = 1;
				continue;
			}
			/* found another link to destination switch, code below will find which one is better */
		}

		curSpeed = sm_GetSpeed(portp->portData);
		if (curSpeed > bestSpeed) {
			bestSpeed = curSpeed;
			bestLidsRouted = portp->portData->lidsRouted;
			portnos[updnLid] = portp->index;
		} else if (  curSpeed == bestSpeed
		          && portp->portData->lidsRouted < bestLidsRouted) {
			bestLidsRouted = portp->portData->lidsRouted;
			portnos[updnLid] = portp->index;
		}
	}
	if (portnos[updnLid] != 0xff && endNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		if (sm_valid_port(portp = sm_get_port(switchp, portnos[updnLid]))) {
			portp->portData->lidsRouted++;
		} else {
			IB_LOG_ERROR_FMT(__func__,
			       "Failed to get port %d for NodeGUID "FMT_U64" [%s]",
			       portnos[updnLid], switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));
		}
	}

end:
	if (portnos[updnLid] == 0xff && smDebugPerf)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "Failed to setup LID 0x%.4X for switch %d, NodeGUID "FMT_U64" [%s]",
		       endPortp->portData->lid, switchp->index, switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));

	return VSTATUS_OK;
}

static Status_t
_setup_pgs(struct _Topology *topop, struct _Node * srcSw, const struct _Node * dstSw)
{
	SwitchportToNextGuid_t * orderedPorts = (SwitchportToNextGuid_t*) topop->pad;
	int ij, endPort = 0;
	Node_t* neighborNodep = NULL;
	Node_t* nodep = NULL;
	Port_t*	portp = NULL;
	uint16_t curSpeed = 0;
	uint16_t bestSpeed = 0;

	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

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

	memset(orderedPorts, 0, sizeof(SwitchportToNextGuid_t) * srcSw->nodeInfo.NumPorts);

	if (!dorTop->alternateRouting) {
		ij = DorBitMapsIndex(srcSw->swIdx, dstSw->swIdx);
		if ((dorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f)))) {
			neighborNodep = _step_to(topop, srcSw, (Node_t*)dstSw);
			if (neighborNodep == NULL) {
				IB_LOG_NOTICE_FMT(__func__,
		       		"Failed to find neighbor in path from NodeGUID "FMT_U64" [%s] to NodeGUID "FMT_U64" [%s]",
		       		srcSw->nodeInfo.NodeGUID, sm_nodeDescString(srcSw), dstSw->nodeInfo.NodeGUID, dstSw->nodeDesc.NodeString);
				return VSTATUS_BAD;
			} else {
				if (smDorRouting.debug)
					IB_LOG_INFINI_INFO_FMT(__func__,
			       		"Found neighbor NodeGUID "FMT_U64" [%s]",
			       		srcSw->nodeInfo.NodeGUID, sm_nodeDescString(srcSw));
			}
		
			for_all_physical_ports(srcSw, portp) {
				if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) continue;
				if ((nodep = sm_find_node(topop, portp->nodeno)) == NULL) continue;
				if (nodep != neighborNodep) continue;
				curSpeed = sm_GetSpeed(portp->portData);
				if (curSpeed >= bestSpeed) {
					if (curSpeed > bestSpeed) {
						bestSpeed = curSpeed;
						endPort = 0;
					}
					orderedPorts[endPort].portp = portp;
					orderedPorts[endPort].guid = nodep->nodeInfo.NodeGUID;
					orderedPorts[endPort].sysGuid = nodep->nodeInfo.SystemImageGUID;
					++endPort;
				}
			}
		}
	}	

	if (endPort <= 1) {
		srcSw->switchInfo.PortGroupTop = 0;
		return VSTATUS_OK;
	}

	STL_PORTMASK pgMask = 0;
	int i;
	for (i = 0; i < endPort; ++i) {
		if (orderedPorts[i].portp->index == 0 ||
			orderedPorts[i].portp->index > sizeof(pgMask)*8) {
			continue;
		}

		// Cast is necessary to prevent compiler from interpreting '1' as a signed
		// int32, converting it to an int64, then or'ing
		pgMask |= (((uint64)1) << (orderedPorts[i].portp->index - 1));
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

static int
_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *endNodep, uint8_t *portnos)
{		
	// unused in DOR
	return 0;
}

static Status_t
_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *endNodep,
                 Port_t *endPortp, uint8_t *portnos)
{
	Status_t status;
	int ij;
	Node_t *endSwitchp;
	DorTopology_t	*dorTop= (DorTopology_t *)topop->routingModule->data;

	endSwitchp = _get_switch(topop, endNodep, endPortp);
	if (endSwitchp == NULL) {
		IB_LOG_ERROR_FMT(__func__,
		       "Failed to find destination switch attached to NodeGUID "FMT_U64" [%s] Port %d",
		       endNodep->nodeInfo.NodeGUID, sm_nodeDescString(endNodep), endPortp->index);
		return VSTATUS_BAD;
	}

	memset(portnos, 0xff, (1 << endPortp->portData->lmc) * sizeof(uint8_t));

	//Setup the DOR route
	if (!dorTop->alternateRouting || endPortp->portData->lmc > 0) {
		ij = DorBitMapsIndex(switchp->swIdx, endSwitchp->swIdx);
		if ((dorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f)))) {
			status = _get_outbound_port_dor(topop, switchp, endNodep, endPortp, portnos);
			if (smDorRouting.debug)
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Routing SW "FMT_U64" [%s] to DLID 0x%04x via DOR: EgressPort %d",
				       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endPortp->portData->lid, portnos[dorTop->alternateRouting?1:0]);
			if (status != VSTATUS_OK) {
				/* Fill portnos with 0xff. If alternate routing is enabled, 
				 * _get_outbound_port_updn will overwrite the 0xff for the port number
				 * corresponding to the updn lid
				 */
				memset(portnos, 0xff, (1 << endPortp->portData->lmc) * sizeof(uint8_t));
				if (!dorTop->alternateRouting)
					return status;
			}
		}
	}

	//If alternate routing is possible, setup the UpDn route
	if (dorTop->alternateRouting) {
		status = _get_outbound_port_updn(topop, switchp, endNodep, endPortp, portnos);
		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Routing SW "FMT_U64" [%s] to DLID 0x%04x via Up/Down: EgressPort %d",
			       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endPortp->portData->lid, portnos[0]);
		return status;
	}

	return VSTATUS_OK;
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

	if (dorTop->numToroidal) {
		dorTop->minReqScs = 4;
	} else {
		dorTop->minReqScs = 2;
	}

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
	Node_t	*neighbor, *disable_neighbor_nodep;
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
					p->state = IB_PORT_DOWN;
	                DECR_PORT_COUNT(topop, nodep);
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
						p->state = IB_PORT_DOWN;
	                	DECR_PORT_COUNT(topop, neighbor);
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
			disable_neighbor_nodep = NULL;
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
			disable_neighbor_nodep = neighbor;
			if (neighbor) {
				disable_neighborDescString = sm_nodeDescString(neighbor);
			} else 	if (get_node_desc(nodep, p, path, &neighborNodeDesc)) {
				disable_neighborDescString = (char*)neighborNodeDesc.NodeString;
			}
		}

		if (invalid) {
			p = sm_get_port(nodep, disable_portno);
			if (sm_valid_port(p)) {
				p->state = IB_PORT_DOWN;
                DECR_PORT_COUNT(topop, nodep);
				if (disable_neighbor_nodep) {
					/* If the neighbor port is not DOWN, mark it as DOWN*/
					neighbor_portp = sm_get_port(disable_neighbor_nodep, disable_neighbor_portno);
					if (sm_valid_port(neighbor_portp) && neighbor_portp->state > IB_PORT_DOWN) {
						neighbor_portp->state = IB_PORT_DOWN;
	                	DECR_PORT_COUNT(topop, disable_neighbor_nodep);
					}
				}
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
		       "Number of dimensions: %d\n",
		       dorTop->numDimensions);

	for (i = 0; i < dorTop->numDimensions; ++i) {
		dorTop->dimensionLength[i] =
			dorTop->coordMaximums[i] - dorTop->coordMinimums[i] + 1;

		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Dimension %d: length %d [%s]\n",
			       i, dorTop->dimensionLength[i],
			       dorTop->toroidal[i] ? "toroidal" : "not toroidal");
	}

	if (state->toroidalOverflow)
		IB_LOG_WARN_FMT(__func__,
		       "Too many toroidal dimensions found. Only the first 4 will be routed cycle-free");

	if (dorTop->numToroidal == 0) {
		dorTop->cycleFreeRouting = 0;
	} else if (state->scsAvailable < 2) {
		IB_LOG_WARN("not enough SCs available to route cycle-free SCs:",
		            state->scsAvailable);
		dorTop->cycleFreeRouting = 0;
	} else {
		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Routing with credit loop avoidance (%d SCs available)",
			       MIN(1 << (state->scsAvailable - 1), 15));
		dorTop->cycleFreeRouting = 1;
	}

	if (!smDorRouting.useUpDownOnDisruption) {
		dorTop->alternateRouting = 0;

	} else if (!dorTop->cycleFreeRouting) {
		// no cycle-free routing, so the only condition is that
		// there be more than 1 SC available
		if (state->scsAvailable < 2) {
			IB_LOG_WARN("not enough SCs available on the fabric to route around mesh failures SCs:",
			            state->scsAvailable);
			dorTop->alternateRouting = 0;
		} else {
			if (smDorRouting.debug)
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Routing with alternate fabric (%d SCs available)",
				       MIN(1 << (state->scsAvailable - 1), 15));
			dorTop->alternateRouting = 1;
		}
	} else {
		// cycle-free routing... check SC space
		if (state->scsAvailable < 2) {
			IB_LOG_WARN("not enough SCs available on the fabric to route around mesh failures SCs:",
			            state->scsAvailable);
			dorTop->alternateRouting = 0;
		} else {
			if (smDorRouting.debug)
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Routing with alternate fabric (%d SCs available)",
				       MIN(1 << (state->scsAvailable - 1), 15));
			dorTop->alternateRouting = 1;
		}
	}

	for (i = 0; i < 256; ++i)
		if (state->dimensionMap[i] != NULL) {
			vs_pool_free(&sm_pool, state->dimensionMap[i]);
			state->dimensionMap[i] = NULL;
		}
	vs_pool_free(&sm_pool, state);

	if (smDorRouting.debug)
		_dump_node_coordinates(topop);

	return VSTATUS_OK;
}

static Status_t
_setup_switches_lrdr(Topology_t *topop, int rebalance, int routing_needed)
{
	if (((DorTopology_t*)topop->routingModule->data)->alternateRouting &&
		smDorRouting.updn_mc_same_spanning_tree && sm_useIdealMcSpanningTreeRoot) {
		/* In this case, the spanning tree root is not next to the SM and hence
		 * the default order of setting up switches in the order they have been
		 * discovered will not work for mixed LR-DR programming. Hence initialize
		 * the switches using the basis of the updn spanning tree.
		 */
		return _setup_switches_lrdr_wave_updn_order(topop, rebalance, routing_needed);
	} else {
		return sm_setup_switches_lrdr_wave_discovery_order(topop, rebalance, routing_needed);
	}
}

static Status_t
_post_process_routing(Topology_t *topop, Topology_t * old_topop, int *rebalance)
{
	Status_t status;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	dorTop->closure_max_sws = topop->max_sws;

	if (topop->max_sws == 0)
		return VSTATUS_OK;	/* HSM connected back to back with a host*/
	
	/* topology costs used by job management are altered by these closure functions.
	 * __calc_dor_closure() will set cost to Infinity if there is no DOR closure.
	 * Later on __calc_updn_closure() will set the actual costs for such pairs based
	 * on the up/dn spanning tree.
	 */

	status = _calc_dor_closure(topop);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to calculate dimension-ordered connectivity; rc:", status);
		return status;
	}

	status = _calc_updn_closure(topop);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to calculate up/down connectivity; rc:", status);
		return status;
	}

	if ((topology_passcount >= 1) &&
		((((DorTopology_t *)old_topop->routingModule->data)->dorClosureSize != dorTop->dorClosureSize) ||
		 (memcmp(((DorTopology_t *)old_topop->routingModule->data)->dorClosure, dorTop->dorClosure,
				dorTop->dorClosureSize)))) {
		/* PR 115155. There is a change in DOR closure which means DOR paths and
		 * LIDs previously in use might not work any more. Also previously
		 * unavailable DOR paths might be available now. Force setting of
		 * client re-registration bit, so that end nodes do their path queries etc
		 * again to get the info based on latest topology data.
		 */
		topop->force_client_reregistration = 1;
	}

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

	status = _copy_updn_closure(src_topop, dst_topop);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to copy up/down connectivity; rc:", status);
		return status;
	}

	return VSTATUS_OK;
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
	STL_LID srcLids, dstLids;

	if (!((DorTopology_t*)topop->routingModule->data)->alternateRouting)
		return VSTATUS_OK;

	srcLids = 1 << srcPortp->portData->lmc;
	dstLids = 1 << dstPortp->portData->lmc;

	if (sm_UpDnBaseLid) {
		*outSrcLid = srcPortp->portData->lid;
		*outDstLid = dstPortp->portData->lid;
	} else {
		*outSrcLid = srcPortp->portData->lid + srcLids - 1;
		*outDstLid = dstPortp->portData->lid + dstLids - 1;
	}
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
	int i, j, ij;
	Node_t *srcSwitchp, *dstSwitchp;
	int updn, invalid;
	STL_LID srcLids, dstLids;
	STL_LID srcUpDnLid, dstUpDnLid, dor_lid_offset = 0;
	uint32_t closure;
	DorTopology_t	*dorTop = (DorTopology_t*)topop->routingModule->data;

	srcSwitchp = _get_switch(topop, srcPortp->portData->nodePtr, srcPortp);
	dstSwitchp = _get_switch(topop, dstPortp->portData->nodePtr, dstPortp);

	if (srcSwitchp == NULL || dstSwitchp == NULL) {
		*outSrcLen = *outDstLen = 0;
		return VSTATUS_OK;
	}

	ij = DorBitMapsIndex(srcSwitchp->swIdx, dstSwitchp->swIdx);
	closure = dorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	if (closure) {
		/* check if reverse path also has dor closure*/
		ij = DorBitMapsIndex(dstSwitchp->swIdx, srcSwitchp->swIdx);
		closure = dorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	}

	if (closure && !dorTop->alternateRouting) {
		return sm_select_path_lids(topop, srcPortp, slid, dstPortp, dlid, outSrcLids, outSrcLen, outDstLids, outDstLen);
	}

	srcLids = 1 << srcPortp->portData->lmc;
	dstLids = 1 << dstPortp->portData->lmc;

	if ((srcLids != dstLids) || (dstLids == 1)) {
		//switch Port 0 can have an LMC of 0, in which case we have to use UpDn LIDs if base lids
		// are up/dn lids.
		if (!sm_UpDnBaseLid) {
			*outSrcLen = *outDstLen = 0;
			return VSTATUS_OK;
		}
		closure = 0;
	}

	// determine if we're selecting an updn path
	invalid = 0;
	if (!dorTop->alternateRouting) {
		// updn not enabled; must be dor
		updn = 0;
	} else {
		// updn enabled
		if (sm_UpDnBaseLid) {
			srcUpDnLid = srcPortp->portData->lid;
			dstUpDnLid = dstPortp->portData->lid;
			dor_lid_offset = 1;
		} else {
			srcUpDnLid = srcPortp->portData->lid + srcLids - 1;
			dstUpDnLid = dstPortp->portData->lid + dstLids - 1;
			dor_lid_offset = 0;
		}

		if (slid == STL_LID_PERMISSIVE && dlid == STL_LID_PERMISSIVE) {
			// both lids are permissive; use dor closure to determine
			updn = closure ? 0 : 1;
		} else if (srcPortp->portData->lmc == 0 || dstPortp->portData->lmc == 0) {
			// one of the ports doesn't support lmc... must use updn
			updn = 1;
		} else if (  (  slid == STL_LID_PERMISSIVE
		             || slid == srcUpDnLid)
		          && (  dlid == STL_LID_PERMISSIVE
		             || dlid == dstUpDnLid)) {
			// both lids are updn lids or one is updn and one is permissive.
			// use updn
			updn = 1;
		} else if (  slid == srcUpDnLid
		          || dlid == dstUpDnLid) {
			// only one of the lids is updn and the other isn't permissive.
			// mixed lids... invalid
			invalid = 1;
		} else {
			// neither lid is updn; must be two dor lids or a combination of
			// dor and permissive lids; check dor closure
			if (!closure) invalid = 1;
			else updn = 0;
		}
	}

	if (invalid) {
		// they specified a dor lid but there isn't closure, or they specified
		// mixed lids
		*outSrcLen = *outDstLen = 0;
	} else if (updn) {
		// they specified an updn lid, or they specified permissive lids and
		// there isn't closure between them
		*outSrcLen = *outDstLen = 1;
		outSrcLids[0] = srcUpDnLid;
		outDstLids[0] = dstUpDnLid;
	} else {
		// alternate routing isn't enabled, or they specified permissive lids
		// and closure exists, or they specified a dor lid
		if (slid == STL_LID_PERMISSIVE) {
			*outSrcLen = dorTop->alternateRouting
			          	? srcLids - 1 : srcLids;
			for (i = 0, j = dor_lid_offset; i < *outSrcLen; ++i, j++)
				outSrcLids[i] = srcPortp->portData->lid + j;
		} else {
			*outSrcLen = 1;
			*outSrcLids = slid;
		}
		if (dlid == STL_LID_PERMISSIVE) {
			*outDstLen = dorTop->alternateRouting
			          ? dstLids - 1 : dstLids;
			for (i = 0, j = dor_lid_offset; i < *outDstLen; ++i, j++)
				outDstLids[i] = dstPortp->portData->lid + j;
		} else {
			*outDstLen = 1;
			*outDstLids = dlid;
		}
	}
	return VSTATUS_OK;
}

static int
_get_sl_for_path(Topology_t *topop, Node_t *srcNodep, Port_t *srcPortp, uint32_t slid,
                 Node_t *dstNodep, Port_t *dstPortp, uint32_t dlid)
{
	int sl = 0;
	int ij;
	uint32_t closure;
	Node_t *srcSwitchp, *dstSwitchp;
	DorTopology_t	*dorTop = (DorTopology_t*)topop->routingModule->data;

	if (!dorTop->cycleFreeRouting && !dorTop->alternateRouting)
		goto end;

	srcSwitchp = _get_switch(topop, srcNodep, srcPortp);
	dstSwitchp = _get_switch(topop, dstNodep, dstPortp);

	if (srcSwitchp == NULL || dstSwitchp == NULL || srcSwitchp == dstSwitchp)
		goto end;

	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "Path is between SwitchGUID "FMT_U64" [%s] SwIdx %d to SwitchGUID "FMT_U64" [%s] SwIdx %d\n",
		       srcSwitchp->nodeInfo.NodeGUID, sm_nodeDescString(srcSwitchp), srcSwitchp->swIdx,
		       dstSwitchp->nodeInfo.NodeGUID, sm_nodeDescString(dstSwitchp), dstSwitchp->swIdx);

	// route on the backup SC if there's no DOR path between the two nodes
	ij = DorBitMapsIndex(srcSwitchp->swIdx, dstSwitchp->swIdx);
	closure = dorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	if (closure) {
		/* check DOR closure on reverse path too*/
		ij = DorBitMapsIndex(dstSwitchp->swIdx, srcSwitchp->swIdx);
		closure = dorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	}

	if (closure && dorTop->alternateRouting) {
			// if either of the ports have an LMC of 0 or the slid/dlid are up/dn lid
			// then can't use DOR and have to use only Up/Dn

		if ((srcPortp->portData->lmc == 0) || (dstPortp->portData->lmc == 0)) {
				closure = 0;
		} else if (sm_UpDnBaseLid) {
			if ((slid == srcPortp->portData->lid) || (dlid == srcPortp->portData->lid))
				closure = 0;
		} else if (!sm_UpDnBaseLid) {
			if ((slid == (srcPortp->portData->lid + ((1 << srcPortp->portData->lmc) - 1))) ||
				(dlid == (srcPortp->portData->lid + ((1 << srcPortp->portData->lmc) - 1))))
				closure = 0;
		}
	}

	if (closure && dorTop->alternateRouting) {
		// but only if the alternate route is enabled for use
		sl = 1;
	}

end:
	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "SL for path from NodeGUID "FMT_U64" [%s] to NodeGUID "FMT_U64" [%s]: %d\n",
		       srcNodep->nodeInfo.NodeGUID, sm_nodeDescString(srcNodep),
		       dstNodep->nodeInfo.NodeGUID, sm_nodeDescString(dstNodep), sl);

	return sl;
}

/* old_idx is the old index of the switch, new_idx is the new index
 * last_idx is the last switch index.
 */
static Status_t _process_swIdx_change(Topology_t * topop, int old_idx, int new_idx, int last_idx)
{
	int i, ij, ji, oldij, oldji;
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (dorTop->dorClosure == NULL && dorTop->updnDownstream == NULL)
		return VSTATUS_OK;

	for (i = 0; i < last_idx; i++) {
		ij = DorBitMapsIndex(i, new_idx);
		oldij = DorBitMapsIndex(i, old_idx);

		ji = DorBitMapsIndex(new_idx, i);
		oldji = DorBitMapsIndex(old_idx, i);

		if (i == new_idx) {
			oldij = oldji = DorBitMapsIndex(old_idx, old_idx);
		}

		if (dorTop->dorClosure != NULL) {
			if (dorTop->dorClosure[oldij >> 5] & (1 << (oldij & 0x1f))) {
				dorTop->dorClosure[ij >> 5] |= 1 << (ij & 0x1f);
				//reset old index value to 0 as it is no longer valid
				dorTop->dorClosure[oldij >> 5] &= ~((uint32_t)(1 << (ij & 0x1f)));
			} else {
				dorTop->dorClosure[ij >> 5] &=  ~((uint32_t)(1 << (ij & 0x1f)));
			}

			if (dorTop->dorClosure[oldji >> 5] & (1 << (oldji & 0x1f))) {
				dorTop->dorClosure[ji >> 5] |= 1 << (ji & 0x1f);
				//reset old index value to 0
				dorTop->dorClosure[oldji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			} else {
				dorTop->dorClosure[ji >> 5] &=  ~((uint32_t)(1 << (ji & 0x1f)));
			}
		}

		if (dorTop->updnDownstream != NULL) {
			if (dorTop->updnDownstream[oldij >> 5] & (1 << (oldij & 0x1f))) {
				dorTop->updnDownstream[ij >> 5] |= 1 << (ij & 0x1f);
				//reset old index value to 0
				dorTop->updnDownstream[oldij >> 5] &= ~((uint32_t)(1 << (ij & 0x1f)));
			} else {
				dorTop->updnDownstream[ij >> 5] &=  ~((uint32_t)(1 << (ij & 0x1f)));
			}

			if (dorTop->updnDownstream[oldji >> 5] & (1 << (oldji & 0x1f))) {
				dorTop->updnDownstream[ji >> 5] |= 1 << (ji & 0x1f);
				//reset old index value to 0
				dorTop->updnDownstream[oldji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			} else {
				dorTop->updnDownstream[ji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			}
		}

	}

	return VSTATUS_OK;	
}


static int check_closure_changes(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp,
								 int check_dor_closure, int check_updn_closure)
{
	DorTopology_t	*oldDorTop = (DorTopology_t *)oldtp->routingModule->data;
	DorTopology_t	*newDorTop = (DorTopology_t *)newtp->routingModule->data;
	int oldnumsws = oldDorTop->closure_max_sws;
	int numsws = newDorTop->closure_max_sws;
	int i, j, ij, oldij, ji, oldji;
	uint32_t new_closure, old_closure;

	i = switchp->swIdx;
	//Check if this switch's dor and updn closure has changed with other switches
   	for (j = 0, ij = i * numsws, oldij = i * oldnumsws; ((j < numsws) && (j < oldnumsws)); j++, ij++, oldij++) {

		ji = (j * numsws) + i;
		oldji = (j * oldnumsws) + i;

		if (check_dor_closure) {
			//dor closure
			new_closure = newDorTop->dorClosure[ij >> 5] & (1 << (ij & 0x1f));
			old_closure = oldDorTop->dorClosure[oldij >> 5] & (1 << (oldij & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;
	
			if (new_closure != old_closure) 
				return 1;	

			//check ji
			new_closure = newDorTop->dorClosure[ji >> 5] & (1 << (ji & 0x1f));
			old_closure = oldDorTop->dorClosure[oldji >> 5] & (1 << (oldji & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;	

			if (new_closure != old_closure) 
				return 1;

		}

		if (check_updn_closure) {
			new_closure = newDorTop->updnDownstream[ij >> 5] & (1 << (ij & 0x1f));
			old_closure = oldDorTop->updnDownstream[oldij >> 5] & (1 << (oldij & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;

			if (new_closure != old_closure) 
				return 1;

			//check ji
			new_closure = newDorTop->updnDownstream[ji >> 5] & (1 << (ji & 0x1f));
			old_closure = oldDorTop->updnDownstream[oldji >> 5] & (1 << (oldji & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;
		
			if (new_closure != old_closure) 
				return 1;
		}
	}

	return 0;
}

static int _check_switch_path_change(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp)
{

	Node_t *neighborNodep;
	Port_t *portp;
	int check_dor_closure = 1;
	int check_updn_closure = 1;
	DorTopology_t *newDorTop = (DorTopology_t *)newtp->routingModule->data;
	DorTopology_t *oldDorTop = (DorTopology_t *)oldtp->routingModule->data;

	if (newDorTop->dorClosure == NULL  || oldDorTop->dorClosure == NULL)
		check_dor_closure = 0;

	if (newDorTop->updnDownstream == NULL || oldDorTop->updnDownstream == NULL)
		check_updn_closure = 0;

	if (!check_dor_closure && !check_updn_closure)
		return 1;

	if (check_closure_changes(oldtp, newtp, switchp, check_dor_closure, check_updn_closure))
		return 1;

	if (!check_updn_closure)
		return 1;

	/* No change in closures for this switch.
	 * But check if updn closures have changed for switches connected to this switch. This is because, even
	 * though other switches might still be down stream to this switch in the updn spanning tree (meaning
	 * there are no updn closure changes), they might have moved from one sub-tree of this switch to another
	 * sub-tree.
	 */

	for_all_physical_ports(switchp, portp) {
		if (!sm_valid_port(portp))
			continue;

		neighborNodep = sm_find_node(newtp, portp->nodeno);
		if (neighborNodep == NULL) {
			continue;
		}

		if (neighborNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			continue;
		}

		if (check_closure_changes(oldtp, newtp, neighborNodep, 0, 1)) {
			return 1;
		}
	}

	return 0;
}

static boolean
_needs_lft_recalc(Topology_t * topop, Node_t * nodep)
{
	return 1;
}

static void
_destroy(Topology_t *topop)
{
	DorTopology_t	*dorTop = (DorTopology_t *)topop->routingModule->data;

	if (dorTop->dorClosure != NULL) {
		vs_pool_free(&sm_pool, (void *)dorTop->dorClosure);
		dorTop->dorClosure = NULL;
	}

	if (dorTop->updnDownstream != NULL) {
		vs_pool_free(&sm_pool, (void *)dorTop->updnDownstream);
		dorTop->updnDownstream = NULL;
	}

	vs_pool_free(&sm_pool, (void *)dorTop);
	topop->routingModule->data = NULL;
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

	// Don't copy routing module data.  This will be setup and compared
	// against old data.
	dest->data = NULL;

	return VSTATUS_OK;
}


static Status_t
_make_routing_module(RoutingModule_t * rm)
{
	rm->name = "dor";
	rm->funcs.pre_process_discovery = _pre_process_discovery;
	rm->funcs.discover_node = _discover_node;
	rm->funcs.discover_node_port = _discover_node_port;
	rm->funcs.post_process_discovery = _post_process_discovery;
	rm->funcs.post_process_routing = _post_process_routing;
	rm->funcs.post_process_routing_copy = _post_process_routing_copy;
	rm->funcs.allocate_cost_matrix = sm_routing_alloc_cost_matrix;
	rm->funcs.initialize_cost_matrix = sm_routing_init_floyds;
 	rm->funcs.calculate_cost_matrix = sm_routing_calc_floyds;
	rm->funcs.setup_switches_lrdr = _setup_switches_lrdr;
	rm->funcs.setup_pgs = _setup_pgs;
	rm->funcs.get_port_group = _get_port_group;
	rm->funcs.setup_xft = _setup_xft;
	rm->funcs.copy_lfts = sm_routing_copy_lfts;
	rm->funcs.select_slsc_map = sm_select_slsc_map;
	rm->funcs.select_scsl_map = sm_select_scsl_map;
	rm->funcs.select_scsc_map = _generate_scsc_map;
	rm->funcs.select_scvl_map = sm_select_scvl_map;
	rm->funcs.select_vlvf_map = sm_select_vlvf_map;
	rm->funcs.fill_stl_vlarb_table = sm_fill_stl_vlarb_table;
	rm->funcs.select_path_lids = _select_path_lids;
	rm->funcs.select_updn_path_lids = _select_updn_path_lids;
	rm->funcs.get_sl_for_path = _get_sl_for_path;
	rm->funcs.process_swIdx_change = _process_swIdx_change;
	rm->funcs.check_switch_path_change = _check_switch_path_change;
	rm->funcs.needs_lft_recalc = _needs_lft_recalc;
	rm->funcs.destroy = _destroy;
	rm->load = _load;
	rm->unload = _unload;
	rm->release = _release;
	rm->copy = _copy;

	return VSTATUS_OK;
}

Status_t
sm_dor_init(Topology_t *topop)
{
	return sm_routing_addModuleFac("dor", _make_routing_module);
}

