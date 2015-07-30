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
#define	DorBitMapsIndex(X, Y)	(((X) * (sm_topop->routing.data.dor.closure_max_sws)) + (Y))

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

uint8_t incorrect_ca_warnings = 0;
uint8_t invalid_isl_found = 0;
//===========================================================================//
// DEBUG ROUTINES
//

static int
_coord_to_string(Topology_t *topop, int8_t *c, char *str)
{
	uint8_t i, l, n = 0;

	l = MIN(topop->routing.data.dor.numDimensions, SM_DOR_MAX_DIMENSIONS);
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
		_coord_to_string(topop, nodep->routing.dor->coords, c);
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "NodeGUID "FMT_U64" [%s]: %s\n",
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

	for (i = 0; i < SM_DOR_MAX_DIMENSIONS; ++i) {
		diff = neighborNodep->routing.dor->coords[i] - nodep->routing.dor->coords[i];
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
	state->dimensionMap[p] = dim;

	*outDim = dim;

	// add reverse direction
	status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
		return status;
	}

	dim->ingressPort = p;
	dim->dimension = state->nextDimension;
	dim->direction = -1;
	state->dimensionMap[q] = dim;

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
	state->dimensionMap[p] = dim;

	*outDim = dim;

	status = vs_pool_alloc(&sm_pool, sizeof(*dim), (void *)&dim);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate dimension data structure; rc:", status);
		return status;
	}

	dim->ingressPort = p;
	dim->dimension = dimension;
	dim->direction = -direction;
	state->dimensionMap[q] = dim;

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
	if (topop->routing.data.dor.numToroidal > SM_DOR_MAX_TOROIDAL_DIMENSIONS) {
		state->toroidalOverflow = 1;
		return 0;
	}

	topop->routing.data.dor.toroidal[dimension] = 1;
	topop->routing.data.dor.toroidalMap[dimension] =
		topop->routing.data.dor.toroidal_count++;

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

	if (state->scsAvailable < topop->routing.data.dor.minReqScs) {
		IB_LOG_ERROR_FMT(__func__,
		       "NodeGUID "FMT_U64" [%s] Port %d has only %d SC(s). "
		       "Minimum required SCs for ISLs for given DOR configuration is %d.",
		       nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index, state->scsAvailable,
		       topop->routing.data.dor.minReqScs);
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
		if (neighborNodep->routing.dor != NULL) {
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
	if (neighborNodep->routing.dor == NULL) {
		status = vs_pool_alloc(&sm_pool, sizeof(DorNode_t),
			(void *)&neighborNodep->routing.dor);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to allocate storage for DOR node structure; rc:", status);
			neighborNodep->routing.dor = NULL;
			return status;
		}
		memset((void *)neighborNodep->routing.dor, 0, sizeof(DorNode_t));
		neighborNodep->routing.dor->node = neighborNodep;

		// update neighbor pointers for every link we find
		if (dim->direction > 0) {
			if (nodep->routing.dor->right[dim->dimension] != NULL);
			neighborNodep->routing.dor->left[dim->dimension] = nodep->routing.dor;
		} else {
			nodep->routing.dor->left[dim->dimension] = neighborNodep->routing.dor;
			neighborNodep->routing.dor->right[dim->dimension] = nodep->routing.dor;
		}

		// copy over existing coordinates and increment the dimension
		// we traveled along
		memcpy((void *)neighborNodep->routing.dor->coords,
		       (void *)nodep->routing.dor->coords,
		       sizeof(nodep->routing.dor->coords));
		neighborNodep->routing.dor->coords[dim->dimension] += dim->direction;

		if (neighborNodep->routing.dor->coords[dim->dimension]
		      < topop->routing.data.dor.coordMinimums[dim->dimension])
			topop->routing.data.dor.coordMinimums[dim->dimension] =
				neighborNodep->routing.dor->coords[dim->dimension];
		else if (neighborNodep->routing.dor->coords[dim->dimension]
		           > topop->routing.data.dor.coordMaximums[dim->dimension])
			topop->routing.data.dor.coordMaximums[dim->dimension] =
				neighborNodep->routing.dor->coords[dim->dimension];
	} else {
		// the neighbor has an existing coordinate, check for a wrap-around edge
		if (dim->direction > 0) {
			if (  nodep->routing.dor->coords[dim->dimension]
			        == topop->routing.data.dor.coordMaximums[dim->dimension]
			   && neighborNodep->routing.dor->coords[dim->dimension]
			        == topop->routing.data.dor.coordMinimums[dim->dimension]) {
				if (is_configured_toroidal(portp->index, portp->portno)) {
					if (smDorRouting.debug) {
						IB_LOG_INFINI_INFO_FMT(__func__,
						       "Found toroidal link for dimension %d in (direction %d, coord bounds [%d, %d]) between "
						       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d",
						       dim->dimension, dim->direction,
					    	   topop->routing.data.dor.coordMinimums[dim->dimension],
						       topop->routing.data.dor.coordMaximums[dim->dimension],
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
			} else if ((neighborNodep->routing.dor->coords[dim->dimension] - nodep->routing.dor->coords[dim->dimension]) > 1) {
				/* This is a wrap around link that is not between the maximum and minimum co-ordinates */
				IB_LOG_WARN_FMT(__func__, "Disabling wrap around link between "
	 					        "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d"
								" as this link is not between the end nodes of this dimension",
				 		        nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), portp->index,
	               			    neighborNodep->nodeInfo.NodeGUID, sm_nodeDescString(neighborNodep), portp->portno);

				disable_isl_ports(topop, nodep, portp, neighborNodep);
			}
		} else {
			if (  nodep->routing.dor->coords[dim->dimension]
			        == topop->routing.data.dor.coordMinimums[dim->dimension]
			   && neighborNodep->routing.dor->coords[dim->dimension]
			        == topop->routing.data.dor.coordMaximums[dim->dimension]) {
				if (is_configured_toroidal(portp->index, portp->portno)) {
					if (smDorRouting.debug) {
						IB_LOG_INFINI_INFO_FMT(__func__,
						       "Found toroidal link for dimension %d in (direction %d, coord bounds [%d, %d]) between "
						       "NodeGUID "FMT_U64" [%s] Port %d and NodeGUID "FMT_U64" [%s] Port %d",
						       dim->dimension, dim->direction,
					    	   topop->routing.data.dor.coordMinimums[dim->dimension],
						       topop->routing.data.dor.coordMaximums[dim->dimension],
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
			} else if ((nodep->routing.dor->coords[dim->dimension] - neighborNodep->routing.dor->coords[dim->dimension]) > 1) {
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
	if (dim->direction > 0) {
		nodep->routing.dor->right[dim->dimension] = neighborNodep->routing.dor;
		neighborNodep->routing.dor->left[dim->dimension] = nodep->routing.dor;
	} else {
		nodep->routing.dor->left[dim->dimension] = neighborNodep->routing.dor;
		neighborNodep->routing.dor->right[dim->dimension] = nodep->routing.dor;
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
		IB_LOG_INFINI_INFO_FMT(__func__, "Cannot copy Updn tree numNodes (%d) < mcST numNodes (%d)",
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

	tree = &updn_spanning_tree;

	memset((void *)tree, 0, sizeof(*tree));

	status = _build_updn_spanning_tree(topop, tree);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to build Up/Down spanning tree; rc:", status);
		return status;
	}

	s = (sizeof(uint32_t) * topop->max_sws * topop->max_sws / 32) + 1;
	status = vs_pool_alloc(&sm_pool, s,
		(void *)&topop->routing.data.dor.updnDownstream);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for Up/Down closure array; rc:", status);
		_free_spanning_tree(tree);
		return status;
	}
	memset((void *)topop->routing.data.dor.updnDownstream, 0, s);

	// mark each node as downstream of all ancestors
	for (i = 0; i < tree->numNodes; ++i) {
		curr = tree->nodes + i;
		parent = curr->parent;
		cost = 0;
		prev = curr;
		while (parent != NULL) {
			ij = DorBitMapsIndex(parent->nodep->swIdx, curr->nodep->swIdx);
			topop->routing.data.dor.updnDownstream[ij >> 5] |= 1 << (ij & 0x1f);
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
				if ((topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f)))) {
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
	size_t s, max_sws = src_topop->routing.data.dor.closure_max_sws;

//	s = (sizeof(uint32_t) * dst_topop->max_sws * dst_topop->max_sws / 32) + 1;
	dst_topop->routing.data.dor.closure_max_sws= max_sws;
	s = (sizeof(uint32_t) * max_sws * max_sws / 32) + 1;
	status = vs_pool_alloc(&sm_pool, s,
		(void *)&dst_topop->routing.data.dor.updnDownstream);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for up/down closure array; rc:", status);
		return status;
	}

	memcpy((void *)dst_topop->routing.data.dor.updnDownstream,
	       (void *)src_topop->routing.data.dor.updnDownstream,
	       s);

	return VSTATUS_OK;
}
void _prune_conn_swlist(Topology_t *topop, Node_t *nodep, SwitchList_t **conn_swlist, SwitchList_t **tail)
{
	SwitchList_t *csw = *conn_swlist, *prev_csw;
	int ij, include, isDownstream;

	*tail = NULL;

	while (csw) {
		/* out of the connected switches, we should consider only those that have a direct connection
		 * in the updn spanning tree i.e those that are children or a parent of nodep - since only
		 * those would be base-LID routable via updn from nodep independent of other switches.
		 */
		include = 0;
		/* check if csw->switchp is parent of nodep*/
		ij = DorBitMapsIndex(nodep->swIdx, csw->switchp->swIdx);
		isDownstream = (topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
		if (isDownstream) {
			include = 1;
		} else {
			/* check if csw->switchp is child of nodep*/
			ij = DorBitMapsIndex(csw->switchp->swIdx, nodep->swIdx);
			isDownstream = (topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
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

	// travel along the first dimension that the source
	// does not currently share with the destination
	for (i = 0; i < topop->routing.data.dor.numDimensions; ++i) {
		si = src->routing.dor->coords[i];
		di = dst->routing.dor->coords[i];
		if (si < di) {
			// source has smaller index; would normally go right
			if (  topop->routing.data.dor.toroidal[i] && src->routing.dor->left[i]
			   && si + topop->routing.data.dor.dimensionLength[i] - di < di - si) {
				// topology is toroidal and crossing over the boundary
				// is quicker, so go left instead
				dnp = src->routing.dor->left[i];
				return dnp == NULL ? NULL : dnp->node;
			} else {
				// not toroidal or toroidal link is down, or normal route; go right
				dnp = src->routing.dor->right[i];
				return dnp == NULL ? NULL : dnp->node;
			}
		} else if (si > di) {
			// source has larger index; would normally go left
			if (  topop->routing.data.dor.toroidal[i] && src->routing.dor->right[i]
			   && di - si + topop->routing.data.dor.dimensionLength[i] < si - di) {
				// topology is toroidal and crossing over the boundary
				// is quicker, so go right instead
				dnp = src->routing.dor->right[i];
				return dnp == NULL ? NULL : dnp->node;
			} else {
				// not toroidal or toroidal link is down, or normal route; go left
				dnp = src->routing.dor->left[i];
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

	if (src == dst) return 1;

	// just step from neighbor to neighbor until we find a hole.
	// if we arrive at the destination, the DOR path is good
	n = _step_to(topop, src, dst);
	while (n != dst) {
		if (n == NULL)
			return 0;

		/* find the dimension on which we are traveling and count the number of steps */
		for (i = 0; i < topop->routing.data.dor.numDimensions; ++i) {
			if (n->routing.dor->coords[i] == dst->routing.dor->coords[i]) {
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
		
			if (n->routing.dor->coords[i] != src->routing.dor->coords[i]) {
				dim = i;
				step++;
				/* If we have taken more steps than the length of the dimension, then we are looping */
				if (step > topop->routing.data.dor.dimensionLength[i]) {
					return 0;
				}
				break;
			}
		}

		n = _step_to(topop, n, dst);
	}
	return 1;
}

// Here I'm defining the toroidal-ness of a path along a given dimension
// to hold if that path would cross the wrap-around edge of the mesh.
//
static int
_is_path_toroidal_in_dimension(Topology_t *topop, Node_t *srcNodep, Node_t *dstNodep, int dimension)
{
	int cs, cd, toroidal;

	// can't be toroidal unless this dimension is wired as such
	if (!topop->routing.data.dor.toroidal[dimension])
		return 0;
	
	cs = srcNodep->routing.dor->coords[dimension];
	cd = dstNodep->routing.dor->coords[dimension];

	// not toroidal if the path would not take use over the wrap-around edge
	if (  cs == cd
	   || (cs < cd && cd - cs <= (topop->routing.data.dor.dimensionLength[dimension] + cs - cd))
	   || (cs > cd && cs - cd <= (topop->routing.data.dor.dimensionLength[dimension] + cd - cs)))
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

	s = (sizeof(uint32_t) * topop->max_sws * topop->max_sws / 32) + 1;
	topop->routing.data.dor.dorClosureSize = s;
	status = vs_pool_alloc(&sm_pool, s, (void *)&topop->routing.data.dor.dorClosure);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}
	memset((void *)topop->routing.data.dor.dorClosure, 0, s);

	for_all_switch_nodes(topop, ni) {
		for_all_switch_nodes(topop, nj) {
			if (ni == nj || _is_path_realizable(topop, ni, nj)) {
				if (smDorRouting.debug)
					IB_LOG_VERBOSE_FMT(__func__,
					       "Path from %d [%s] to self is realizable\n",
					       ni->swIdx, sm_nodeDescString(ni));
				ij = DorBitMapsIndex(ni->swIdx, nj->swIdx);
				topop->routing.data.dor.dorClosure[ij >> 5] |= 1 << (ij & 0x1f);
			} else {
				 /* Set floyd cost to Infinity for this pair as there is no DOR closure */
				 /* The cost will be set later on based on the Up/Dn spanning tree calculation */
				 /* Note - floyd costs are not used in DOR or Up/Dn but are used by the job management code. */
				topop->cost[Index(ni->swIdx, nj->swIdx)] = Cost_Infinity;
				topop->cost[Index(nj->swIdx, ni->swIdx)] = Cost_Infinity;

				if (smDorRouting.debug)
					IB_LOG_INFINI_INFO_FMT(__func__,
					       "Path from %d [%s] to %d [%s] is NOT realizable\n",\
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
	size_t s, max_sws = src_topop->routing.data.dor.closure_max_sws;;

	dst_topop->routing.data.dor.closure_max_sws= max_sws;
	s = (sizeof(uint32_t) * max_sws * max_sws / 32) + 1;
	status = vs_pool_alloc(&sm_pool, s, (void *)&dst_topop->routing.data.dor.dorClosure);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate memory for DOR closure array; rc:", status);
		return status;
	}

	memcpy((void *)dst_topop->routing.data.dor.dorClosure,
	       (void *)src_topop->routing.data.dor.dorClosure,
	       s);

	return VSTATUS_OK;
}

//===========================================================================//
// SLSC TABLES
//

static __inline__ int
_halfway_low(Topology_t *topop, uint8_t dimension)
{
	uint8_t len = topop->routing.data.dor.dimensionLength[dimension];

	if (len % 2)
		return topop->routing.data.dor.coordMinimums[dimension] + len / 2;
	else
		return topop->routing.data.dor.coordMinimums[dimension] + len / 2;
}

static __inline__ int
_halfway_high(Topology_t *topop, uint8_t dimension)
{
	uint8_t len = topop->routing.data.dor.dimensionLength[dimension];

	if (len % 2)
		return topop->routing.data.dor.coordMinimums[dimension] + len / 2;
	else
		return topop->routing.data.dor.coordMinimums[dimension] + len / 2 + 1;
}

// Although the SLSC maps table defines 32 SLs, for now it will adhere to the
// IB SLVL maps table that uses upto 16 SLs.
//
// All SLSC maps are statically allocated to save space.
// If alternate routing is enabled, SL 0 and SC 0 are for
// Up/Dn. SL 1 onwards and SC 1 and SC 2 are for DOR.
//
// Index legend:
//         0: default map for no toroidal dimensions
//    1 -  5: maps for 1 toroidal dimension
//    6 - 10: maps for 2 toroidal dimensions
//   11 - 15: maps for 3 toroidal dimensions
//   16 - 20: maps for 4 toroidal dimensions
static uint8_t _slsc_maps[21][STL_MAX_SLS] =
{
	// 0d default
	{ 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 1d default
	{ 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 1d dim 0
	{ 0x0, 0x1, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 1d dim 1 (a spacer for now)
	{ 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 1d dim 2 (a spacer for now)
	{ 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 1d dim 3 (a spacer for now)
	{ 0x0, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 2d default
	{ 0x0, 0x1, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 2d dim 0
	{ 0x0, 0x1, 0x2, 0x1, 0x2, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 2d dim 1
	{ 0x0, 0x1, 0x1, 0x2, 0x2, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 2d dim 2 (a spacer for now)
	{ 0x0, 0x1, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 2d dim 3 (a spacer for now)
	{ 0x0, 0x1, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 3d default
	{ 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	  0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 3d dim 0
	{ 0x0, 0x1, 0x2, 0x1, 0x2, 0x1, 0x2, 0x1,
	  0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 3d dim 1
	{ 0x0, 0x1, 0x1, 0x2, 0x2, 0x1, 0x1, 0x2,
	  0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 3d dim 2
	{ 0x0, 0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x2,
	  0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 3d dim 3 (a spacer for now)
	{ 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	  0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// For now in 4d, we will use the IB SL2VL map model that uses upto 16 SLs
    // for cycle free routing, and uses no  VLs for alternate routing 4d default
	{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 4d dim 0
	{ 0x0, 0x1, 0x0, 0x1, 0x0, 0x1, 0x0, 0x1,
	  0x0, 0x1, 0x0, 0x1, 0x0, 0x1, 0x0, 0x1,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 4d dim 1
	{ 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1,
	  0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 4d dim 2
	{ 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1,
	  0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
	// 4d dim 3
	{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	  0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
      0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
};

Qos_t	dor_qos;

// dor_slsc[0]  - HFIs
// dor_slsc[1]  - Mesh, Torus/default
// dor_slsc[2]  - HFIs, Torus/dim0 over meridian
// dor_slsc[3]  - HFIs, Torus/dim1 over meridian
// dor_slsc[4]  - HFIs, Torus/dim2 over meridian
// dor_slsc[5]  - HFIs, Torus/dim3 over meridian
STL_SLSCMAP dor_slsc[6];

static void
setSlscEntry(uint8_t numToroidal, uint8_t base_sl, uint8_t base_sc, uint8_t sc0, uint8_t sc1, uint8_t sc2)
{

	// 1d torus
	dor_slsc[0].SLSCMap[base_sl].SC = base_sc;
	dor_slsc[0].SLSCMap[base_sl+1].SC = base_sc;
	dor_slsc[0].SLSCMap[base_sl+2].SC = base_sc;

	dor_slsc[1].SLSCMap[base_sl].SC = sc0;
	dor_slsc[1].SLSCMap[base_sl+1].SC = sc1;
	dor_slsc[1].SLSCMap[base_sl+2].SC = sc1;

	dor_slsc[2].SLSCMap[base_sl].SC = sc0;
	dor_slsc[2].SLSCMap[base_sl+1].SC = sc1;
	dor_slsc[2].SLSCMap[base_sl+2].SC = sc2;
			
	if (numToroidal >= 2) {
		// 2d torus
		dor_slsc[0].SLSCMap[base_sl+3].SC = base_sc;
		dor_slsc[0].SLSCMap[base_sl+4].SC = base_sc;

		dor_slsc[1].SLSCMap[base_sl+3].SC = sc1;
		dor_slsc[1].SLSCMap[base_sl+4].SC = sc1;

		dor_slsc[2].SLSCMap[base_sl+3].SC = sc1;
		dor_slsc[2].SLSCMap[base_sl+4].SC = sc2;

		dor_slsc[3].SLSCMap[base_sl].SC = sc0;
		dor_slsc[3].SLSCMap[base_sl+1].SC = sc1;
		dor_slsc[3].SLSCMap[base_sl+2].SC = sc1;
		dor_slsc[3].SLSCMap[base_sl+3].SC = sc2;
		dor_slsc[3].SLSCMap[base_sl+4].SC = sc2;
	}

	if (numToroidal == 3) {
		// 3d torus
		dor_slsc[0].SLSCMap[base_sl+5].SC = base_sc;
		dor_slsc[0].SLSCMap[base_sl+6].SC = base_sc;
		dor_slsc[0].SLSCMap[base_sl+7].SC = base_sc;
		dor_slsc[0].SLSCMap[base_sl+8].SC = base_sc;

		dor_slsc[1].SLSCMap[base_sl+5].SC = sc1;
		dor_slsc[1].SLSCMap[base_sl+6].SC = sc1;
		dor_slsc[1].SLSCMap[base_sl+7].SC = sc1;
		dor_slsc[1].SLSCMap[base_sl+8].SC = sc1;

		dor_slsc[2].SLSCMap[base_sl+5].SC = sc1;
		dor_slsc[2].SLSCMap[base_sl+6].SC = sc2;
		dor_slsc[2].SLSCMap[base_sl+7].SC = sc1;
		dor_slsc[2].SLSCMap[base_sl+8].SC = sc2;

		dor_slsc[3].SLSCMap[base_sl+5].SC = sc1;
		dor_slsc[3].SLSCMap[base_sl+6].SC = sc1;
		dor_slsc[3].SLSCMap[base_sl+7].SC = sc1;
		dor_slsc[3].SLSCMap[base_sl+8].SC = sc2;

		dor_slsc[4].SLSCMap[base_sl].SC = sc0;
		dor_slsc[4].SLSCMap[base_sl+1].SC = sc1;
		dor_slsc[4].SLSCMap[base_sl+2].SC = sc1;
		dor_slsc[4].SLSCMap[base_sl+3].SC = sc1;
		dor_slsc[4].SLSCMap[base_sl+4].SC = sc1;
		dor_slsc[4].SLSCMap[base_sl+5].SC = sc2;
		dor_slsc[4].SLSCMap[base_sl+6].SC = sc2;
		dor_slsc[4].SLSCMap[base_sl+7].SC = sc2;
		dor_slsc[4].SLSCMap[base_sl+8].SC = sc2;
	}
}

static void
setup_slscmap(Topology_t *topop, uint8_t vlsInUse) {
	int d;
	int next_sc=0, base_sc=0, sc0=0,sc1,sc2, vf, sc, i;
	static uint8_t initComplete=0;
	uint8_t	configErr=0;
	uint8_t nonQosProcessed=0;
	uint8_t numscs;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	if (!VirtualFabrics->qosEnabled) {
		topop->qosEnforced = 0;
		memset(&dor_slsc[0], 0, sizeof(STL_SLSCMAP));
		if (!topop->routing.data.dor.numToroidal) {
			memcpy(&dor_slsc[1], _slsc_maps[0], sizeof(STL_SLSCMAP));
		} else if (topop->routing.data.dor.numToroidal <= SM_DOR_MAX_TOROIDAL_DIMENSIONS) {
			for (d=1; d<6; d++) {
				memcpy(&dor_slsc[d], _slsc_maps[5 * (topop->routing.data.dor.numToroidal - 1) + d], sizeof(STL_SLSCMAP));
			}
		} else {
			configErr=1;
			goto done;
		}
		return;
	}

    // Note, for the initial STL drops the number of SCs utilized is the
    // IB limit of 16 VLs
	numscs = vlsInUse;

	if (!initComplete) {
		dor_qos.dimensions = 0xff;
		dor_qos.numVLs = 0;
		if (!bitset_init(&sm_pool, &dor_qos.lowPriorityVLs, MAX_VIRTUAL_LANES) ||
			!bitset_init(&sm_pool, &dor_qos.highPriorityVLs, MAX_VIRTUAL_LANES)) {
			IB_FATAL_ERROR("setup_slscmap: No memory for slsc setup, exiting.");
		}
		initComplete = 1;
	}

	if (smDorRouting.debug) {
		IB_LOG_INFINI_INFO_FMT(__func__, "numToroidal= %d, dimension= %d, routingSCs= %d, shareScOnDisruption=%d",
						topop->routing.data.dor.numToroidal, smDorRouting.dimensionCount,
						smDorRouting.routingSCs, smDorRouting.shareScOnDisruption);
	}

	if ((dor_qos.dimensions == topop->routing.data.dor.numToroidal) &&
		(dor_qos.numVLs == numscs)) {
		// skip setup if nothing has changed.
		topop->qosEnforced = dor_qos.qosEnforced;
		return;
	}

	memset(dor_slsc[0].SLSCMap, 15, sizeof(STL_SLSCMAP));
	memset(dor_slsc[1].SLSCMap, 15, sizeof(STL_SLSCMAP));
	memset(dor_slsc[2].SLSCMap, 15, sizeof(STL_SLSCMAP));
	memset(dor_slsc[3].SLSCMap, 15, sizeof(STL_SLSCMAP));
	for (i=0; i<4; i++) dor_slsc[i].SLSCMap[0].SC = 0; 	// Set SL0 to valid value 

	if (topop->routing.data.dor.numToroidal > SM_DOR_MAX_TOROIDAL_DIMENSIONS) {
		IB_LOG_WARN_FMT(__func__, "Mesh/Torus topology QoS not supported on %dD Torus", topop->routing.data.dor.numToroidal);
		configErr = 1;
	}

	if (numscs < smDorRouting.scsNeeded) {
		IB_LOG_WARN_FMT(__func__, "Mesh/Torus topology QoS compromised by insufficient Switch SCs (SCs Available = %d, SCs Required = %d)",
						numscs, smDorRouting.scsNeeded);
		configErr = 1;

	} else if ((topop->routing.data.dor.numToroidal != smDorRouting.numToroidal) ||
			   (topop->routing.data.dor.numToroidal && (smDorRouting.topology == DOR_MESH))) {

		if (topop->routing.data.dor.numToroidal && (smDorRouting.topology == DOR_MESH)) {
			IB_LOG_ERROR_FMT(__func__, "Mesh/Torus topology configuration indicates mesh, but actual topology is %d-d torus, QoS conflict",
								smDorRouting.dimensionCount);
			configErr = 1;
		} else if (!topop->routing.data.dor.numToroidal && (smDorRouting.topology != DOR_MESH)) {
			IB_LOG_WARN_FMT(__func__, "Mesh/Torus topology configuration indicates %d-d torus but actual topology is a mesh. QoS still operational",
						smDorRouting.dimensionCount);
		} else if (topop->routing.data.dor.numToroidal > smDorRouting.numToroidal) {
			IB_LOG_ERROR_FMT(__func__, "Mesh/Torus topology configuration indicates %d-d torus but actual topology is a %d-torus, QoS conflict",
						smDorRouting.dimensionCount, topop->routing.data.dor.numToroidal);
			configErr = 1;
		} else if (topop->routing.data.dor.numToroidal) {
			IB_LOG_WARN_FMT(__func__, "Mesh/Torus topology configuration indicates %d-d torus but actual topology is a %d-d torus, QoS still operational",
						smDorRouting.dimensionCount, topop->routing.data.dor.numToroidal);

		}
	}

	if (configErr) goto done;

	Qos_t* qos = GetQos(vlsInUse);

	bitset_copy(&dor_qos.lowPriorityVLs, &qos->lowPriorityVLs);
	bitset_copy(&dor_qos.highPriorityVLs, &qos->highPriorityVLs);
	dor_qos.numVLs = qos->numVLs;
	dor_qos.dimensions = topop->routing.data.dor.numToroidal;
	memcpy(dor_qos.vlBandwidth, qos->vlBandwidth, sizeof(uint8_t)*MAX_VIRTUAL_LANES);

	for (vf=0; vf<VirtualFabrics->number_of_vfs; vf++) {
		if (!VirtualFabrics->v_fabric[vf].qos_enable) {
			if (nonQosProcessed) continue;
			nonQosProcessed = 1;
		}

        // FIXME - verify new mappings (SL::SC)
		//base_sc = qos.slsc.SLSCMap[VirtualFabrics->v_fabric[vf].base_sl].SC;

		if (VirtualFabrics->v_fabric[vf].updown_only) {
			for (i=0; i<5; i++) {
				dor_slsc[i].SLSCMap[VirtualFabrics->v_fabric[vf].base_sl].SC = base_sc;
			}
			continue;
		}

        // FIXME - verify new mappings (SL::SC)
		//next_sc = qos.slsc.SLSCMap[VirtualFabrics->v_fabric[vf].base_sl+1].SC;

		if (!topop->routing.data.dor.numToroidal) {
			dor_slsc[0].SLSCMap[VirtualFabrics->v_fabric[vf].base_sl].SC = base_sc;
			dor_slsc[0].SLSCMap[VirtualFabrics->v_fabric[vf].base_sl+1].SC = base_sc;
	
			dor_slsc[1].SLSCMap[VirtualFabrics->v_fabric[vf].base_sl].SC = base_sc;
			dor_slsc[1].SLSCMap[VirtualFabrics->v_fabric[vf].base_sl+1].SC = next_sc;

		} else {
			sc1 = base_sc;
			sc2 = next_sc;
            // FIXME - verify new mappings (SL::SC)
			//sc0 = qos.slsc.SLSCMap[VirtualFabrics->v_fabric[vf].base_sl+2].SC;
			setSlscEntry(topop->routing.data.dor.numToroidal, VirtualFabrics->v_fabric[vf].base_sl, base_sc, sc0, sc1, sc2);
		}
	}
	dor_qos.qosEnforced = 1;

done:	
	if (configErr) {
		dor_qos.qosEnforced = 0;
		for (vf=0; vf<VirtualFabrics->number_of_vfs; vf++) {
			if (VirtualFabrics->v_fabric[vf].updown_only) {
				for (i=0; i<5; i++) {
					dor_slsc[i].SLSCMap[VirtualFabrics->v_fabric[vf].base_sl].SC = 0;
				}
				continue;
			}
			setSlscEntry(topop->routing.data.dor.numToroidal, VirtualFabrics->v_fabric[vf].base_sl, 0, 0, 1, 2);
		}

	} else if (smDorRouting.debug) {
		bitset_info_log(&dor_qos.highPriorityVLs, "highPriorityVLs");
		bitset_info_log(&dor_qos.lowPriorityVLs, "lowPriorityVLs");
		for (sc=0;sc<15;sc++) {
			IB_LOG_INFINI_INFO_FMT(__func__, "sc %d bandwidth %d",
									sc, dor_qos.vlBandwidth[sc]);
		}

		for (d=0; d<4; d++) {
			IB_LOG_INFINI_INFO_FMT(__func__,
				"slsc%d= 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", dor_qos.numVLs,
				dor_slsc[d].SLSCMap[0].SC, dor_slsc[d].SLSCMap[1].SC, dor_slsc[d].SLSCMap[2].SC, dor_slsc[d].SLSCMap[3].SC,
				dor_slsc[d].SLSCMap[4].SC, dor_slsc[d].SLSCMap[5].SC, dor_slsc[d].SLSCMap[6].SC, dor_slsc[d].SLSCMap[7].SC);
	
			IB_LOG_INFINI_INFO_FMT(__func__,
				"\t\t  0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
				dor_slsc[d].SLSCMap[8].SC,  dor_slsc[d].SLSCMap[9].SC,  dor_slsc[d].SLSCMap[10].SC, dor_slsc[d].SLSCMap[11].SC,
				dor_slsc[d].SLSCMap[12].SC, dor_slsc[d].SLSCMap[13].SC, dor_slsc[d].SLSCMap[14].SC, dor_slsc[d].SLSCMap[15].SC);
		}
	}
	topop->qosEnforced = dor_qos.qosEnforced;
}

static void
get_hca_qos_stl_setup(Topology_t *topop, Node_t *nodep, Port_t *portp, Node_t *switchp, Port_t *swportp, STL_SLSCMAP *slscmap) {
	int			sl, vf, vl;
	int			numscs = portp->portData->vl1;
	Qos_t		qos;
	STL_VLARB_TABLE_ELEMENT *vlblockp;
	int			slscNotInUse=15;
	int			maxLow = numscs-1;
	int			minOldSL = 0xff, minOldSC = 0xff;
    uint8_t		isUsingSL[MAX_SLS] = { 0 };
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	qos.numVLs = numscs;

	if (numscs < 2) {
		memset(slscmap, 0, sizeof(STL_SLSCMAP));

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

		memset(arbp->vlarbLow, 0, sizeof(arbp->vlarbLow));
		memset(arbp->vlarbHigh, 0, sizeof(arbp->vlarbHigh));
		memset(arbp->vlarbPre, 0, sizeof(arbp->vlarbPre));
		memset(arbp->vlarbMatrix, 0, sizeof(arbp->vlarbMatrix));

		vlblockp = arbp->vlarbLow;
		vlblockp[0].Weight = 255;
		return;
	}

    // FIXME --- FD3 Patch cjking:  Need to understand the impact of 0xff
    // value for SCs.  This maybe a trigger for the upper level DOR code
    // FIXME - verify new mappings (SL::SC)
	//memset(&qos.slsc, 0xff, sizeof(STL_SLSCMAP));
	if (!bitset_init(&sm_pool, &qos.highPriorityVLs, MAX_VIRTUAL_LANES) ||
		!bitset_init(&sm_pool, &qos.lowPriorityVLs, MAX_VIRTUAL_LANES)) {
		IB_FATAL_ERROR("get_hca_qos_setup: No memory for slvl setup, exiting.");
	}

	memset(qos.vlBandwidth, 0, sizeof(uint8_t)*MAX_VIRTUAL_LANES);

	for (vf=0; vf<VirtualFabrics->number_of_vfs; vf++) {
		if (bitset_test(&portp->portData->vfMember, vf)) {
			for (sl=0; sl< VirtualFabrics->v_fabric[vf].routing_sls; sl++) {
				isUsingSL[VirtualFabrics->v_fabric[vf].base_sl+sl] = 1;
			}

			if (VirtualFabrics->v_fabric[vf].priority == 1) {
				for (sl=0; sl<VirtualFabrics->v_fabric[vf].routing_sls; sl++) {
                    // FIXME - verify new mappings (SL::SC)
					//qos.slsc.SLSCMap[VirtualFabrics->v_fabric[vf].base_sl+sl].SC = numscs-1;
				}
				bitset_set(&qos.highPriorityVLs, numscs-1);
				if (numscs > 1) maxLow = numscs-2;
			}
		}
	}

	for (sl=0; sl<MAX_SLS; sl++) {
		if ((dor_slsc[0].SLSCMap[sl].SC == 15) ||
			!isUsingSL[sl]) {
            // FIXME - verify new mappings (SL::SC)
			//qos.slsc.SLSCMap[sl].SC = slscNotInUse;
		}
	}

	vl=0;	
	while (1) {
		// find unassigned min
		minOldSC = 0xff;
		for (sl=0; sl<MAX_SLS; sl++) {
            // FIXME - verify new mappings (SL::SC)
			//if (qos.slsc.SLSCMap[sl].SC != 0xff) continue;
			if (dor_slsc[0].SLSCMap[sl].SC < minOldSC) {
				minOldSC = dor_slsc[0].SLSCMap[sl].SC;
				minOldSL = sl;
			}
		}
		if (minOldSC == 0xff) break;
		
		for (sl=minOldSL; sl<MAX_SLS; sl++) {
			if (dor_slsc[0].SLSCMap[minOldSL].SC != dor_slsc[0].SLSCMap[sl].SC) break;
            // FIXME - verify new mappings (SL::SC)
			//qos.slsc.SLSCMap[sl].SC = vl;
		}

		bitset_set(&qos.lowPriorityVLs, vl);
 		qos.vlBandwidth[vl] += sm_SlBandwidthAllocated[minOldSL];

		if (vl < maxLow) vl++;
	}
	
    // FIXME - verify new mappings (SL::SC)
	//memcpy (slscmap, &qos.slsc, sizeof(STL_SLSCMAP));

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

static void
_select_slsc_map_hca_default(Topology_t *topop, Node_t *nodep, Port_t *portp, Node_t* switchp, Port_t* swportp, STL_SLSCMAP *outSlscMap)
{
	int		sl;
	int		numscs = portp->portData->vl1;

	if (topop->qosEnforced &&
 		(numscs < smDorRouting.scsNeeded)) {

		get_hca_qos_stl_setup(topop, nodep, portp, switchp, swportp, outSlscMap);

	} else {
		memcpy(outSlscMap, &dor_slsc[0], sizeof(STL_SLSCMAP));
	}
}

static void
_select_slsc_map_switch_default(Topology_t *topop, uint8_t vl1, STL_SLSCMAP *outSlscMap)
{
	if (vl1 < 2) {
		memset(outSlscMap, 0, sizeof(STL_SLSCMAP));
    
	} else {
		memcpy(outSlscMap, &dor_slsc[1], sizeof(STL_SLSCMAP));
	}
}

static void
_select_slsc_map(Topology_t *topop, uint8_t vl1, uint8_t dimension, STL_SLSCMAP *outSlscMap)
{
	if (vl1 < 2) {
		memset(outSlscMap, 0, sizeof(STL_SLSCMAP));
    
	} else {
		if (topop->routing.data.dor.numToroidal == 0) {
			memcpy(outSlscMap, &dor_slsc[1], sizeof(STL_SLSCMAP));
		} else {
			memcpy(outSlscMap, &dor_slsc[1 + topop->routing.data.dor.toroidalMap[dimension] + 1], sizeof(STL_SLSCMAP));;
		}
	}
}

static Status_t
_generate_slsc_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SLSCMAP *outSlscMap)
{
	uint8_t dimension;
	int8_t direction = 0;
	Node_t *neighborNodep;
	Port_t *neighborPortp;
	uint8_t	vl1;

	vl1 = out_portp->portData->vl1;

	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
	   || !topop->routing.data.dor.cycleFreeRouting) {
		/* If we have only one SC, then make sure all SLs map to SC 0.
		 * Its possible that FIs have only one SC while ISLs have more than 1.
		 * In which case, we need to make sure that for paths that use other SLs
		 * for the FI, the SC is still SC 0. 
		 * If ISLs have only one SC, then we will not have cycleFreeRouting or
		 * alternate routing and we will end upcoming here.
	   	 */
		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			_select_slsc_map_hca_default(topop, nodep, out_portp, NULL, NULL, outSlscMap);

		} else if (vl1 < out_portp->portData->vl0) {
			// Negotiated down, use hca setup
			neighborNodep = sm_find_node(topop, out_portp->nodeno);
			if (neighborNodep == NULL) {
				IB_LOG_ERROR("Failed to lookup neighbor node in topology nodeno:", out_portp->nodeno);
				memset(outSlscMap, 0, sizeof(STL_SLSCMAP));
				return VSTATUS_BAD;
			}

			if (neighborNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
				neighborPortp = sm_find_node_port(topop, neighborNodep, out_portp->portno);
				if (sm_valid_port(neighborPortp) ) {
					_select_slsc_map_hca_default(topop, neighborNodep, neighborPortp, nodep, out_portp, outSlscMap);
				} else {
					memset(outSlscMap, 0, sizeof(STL_SLSCMAP));
				}
				return VSTATUS_OK;
			}

			_select_slsc_map_switch_default(topop, vl1, outSlscMap);

		} else {
			_select_slsc_map_switch_default(topop, vl1, outSlscMap);
		}
		return VSTATUS_OK;
	}

	neighborNodep = sm_find_node(topop, out_portp->nodeno);
	if (neighborNodep == NULL) {
		IB_LOG_ERROR("Failed to lookup neighbor node in topology nodeno:", out_portp->nodeno);
		memset(outSlscMap, 0, sizeof(STL_SLSCMAP));
		return VSTATUS_BAD;
	}

	if (neighborNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		if (vl1 < out_portp->portData->vl0) {
			// Negotiated down, use hca setup
			neighborPortp = sm_find_node_port(topop, neighborNodep, out_portp->portno);
			if (sm_valid_port(neighborPortp) ) {
				_select_slsc_map_hca_default(topop, neighborNodep, neighborPortp, nodep, out_portp, outSlscMap);
			} else {
				memset(outSlscMap, 0, sizeof(STL_SLSCMAP));
			}
		} else {
			_select_slsc_map_switch_default(topop, vl1, outSlscMap);
		}
		return VSTATUS_OK;
	}

	_find_dimension_difference(nodep, neighborNodep, &dimension, &direction);
	if (!topop->routing.data.dor.toroidal[dimension]) {
		_select_slsc_map_switch_default(topop, vl1, outSlscMap);
		return VSTATUS_OK;
	}

	if (  direction == 1
	   && neighborNodep->routing.dor->coords[dimension]
	        < _halfway_low(topop, dimension)) {
		_select_slsc_map(topop, vl1, dimension, outSlscMap);
	} else if (  direction == -1
	        && neighborNodep->routing.dor->coords[dimension]
	             > _halfway_high(topop, dimension)) {
		_select_slsc_map(topop, vl1, dimension, outSlscMap);
	} else {
		_select_slsc_map_switch_default(topop, vl1, outSlscMap);
	}
	return VSTATUS_OK;
}

static Status_t
_generate_scsl_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SCSLMAP *outScslMap)
{
    IB_LOG_ERROR0("Not implemented yet"); 
	return VSTATUS_NOSUPPORT;
}


static Status_t
_select_scvl_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SCVLMAP *outScvlMap)
{
    // Fill in SCVL map.
    // Currently using a 1:1 map, e.g. SC0 maps to VL0, SC1 to VL1...
    // In STL1, the number of SCs in use will be limited to number of 
    //   VLs supported by the port pair / link.
    // So the same default map can be used for each QOS type.
    // In furture implementatons, SC2VL_t and SC2VL_r maps may have 
    //   varying numbers of SC vs VLs.
    //   [Expansion Mapping, Contraction Mapping, or Conservation mapping]

    static uint8_t scvl_default[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f };

    memcpy(outScvlMap, &scvl_default, sizeof(scvl_default));
	return VSTATUS_OK;
}

static Status_t 
_select_vlvf_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlVfMap_t * vlvfmap)
{
    // Each VL can map to 0 to 32 possible VFs.  If VL-VF unused, set to -1
    // See implementation notes in shortest path algorithm.
    // For now- implemented the worst-case, least optimized scenario.

    uint8_t vf, sl, sc, vl, slMax, i;
    Status_t status = VSTATUS_OK;
    STL_SLSCMAP slscmap; 
    STL_SCVLMAP scvlmap; 
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

    // Default to no VF per VL
    memset(vlvfmap,-1, sizeof(VlVfMap_t));

    if (VirtualFabrics->number_of_vfs >= MAX_VFABRICS) {
        IB_LOG_ERROR("Unexpected number of VFs", VirtualFabrics->number_of_vfs);
		return VSTATUS_BAD;
    }

    status = topop->routingModule->funcs.select_slsc_map(topop, nodep, portp, portp, &slscmap);
    if (status != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to get SLSC - rc:",status);
        return status;
    }

    status = topop->routingModule->funcs.select_scvl_map(topop, nodep, portp, portp, &scvlmap);
    if (status != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to get SCVL - rc:",status);
        return status;
    }

    // Looping through the VFs, find the SL's in use, then find SC for SLs, then VLs for SCs
	for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
        sl = VirtualFabrics->v_fabric[vf].base_sl;
        slMax = sl + VirtualFabrics->v_fabric[vf].routing_sls;
        for (;sl < slMax; sl++) {
            sc = slscmap.SLSCMap[sl].SC; 
            if (sc==15) continue; // This sc is unused for this port.
            if (sc >=STL_MAX_SCS) {
                IB_LOG_WARN("Unexpected SL:SC Mapping: SC=", sc);
                continue;
            }
            vl = scvlmap.SCVLMap[sc].VL;
            if (vl >= STL_MAX_VLS) {
                IB_LOG_WARN("Unexpected SC:VL Mapping: VL=", vl);
                continue;
            }

            for (i=0; i<MAX_VFABRICS; i++) {
                if (vlvfmap->vf[vl][i] == vf) break; // No dup vfs
                if (vlvfmap->vf[vl][i] == -1) {
                    vlvfmap->vf[vl][i] = vf; 
                    break;
                }
            }
        }
	}
    return status;
}

//===========================================================================//
// VL ARBITRATION
//
// Here we just add the 2 DOR SCs and the 1 Up/Down SC with equal weight.
// One per QoS group.
//
static Status_t
_fill_stl_vlarb_table(Topology_t *topop, Node_t *nodep, Port_t *portp, struct _PortDataVLArb * arbp)
{
	int			currentSc, numScs;

	if (portp->portData->qosHfiFilter) {
		// Use vlarb setup during filtering of HFI by vf for slvl setup.
		return VSTATUS_OK;
	}

    numScs = portp->portData->vl1;
	if (!topop->qosEnforced) {
		if (nodep->nodeInfo.NodeType == NI_TYPE_CA) {
			numScs = 1;
		} else {
			if (numScs > 3)
				numScs = 3;
			if ((numScs > 2) && !topop->routing.data.dor.cycleFreeRouting)
				--numScs;
			if ((numScs > 2) && !topop->routing.data.dor.alternateRouting)
				--numScs;
		}

		memset(arbp, 0, sizeof(*arbp));
		STL_VLARB_TABLE_ELEMENT * vlblockp = arbp->vlarbLow;

		for (currentSc = 0; currentSc < numScs; currentSc++) {
			vlblockp[currentSc].s.VL = currentSc;
			vlblockp[currentSc].Weight = 255;
		}
		return VSTATUS_OK;
	}

#error DOR-specific code removed from FillHighRR().  This function must implement this in some fashion if this code is to be used again
#if 0
		if ((smDorRouting.routingSCs != 2)
			|| (vlweight != weight * (qos->vlBandwidth[currentVl] / weightMultiplier)))
			continue;

		if (currentEntry >= portp->portData->portInfo.VL.ArbitrationHighCap)
			break;
		vlblockp[currentEntry].s.VL = currentVl;
		vlweight = weight * (qos->vlBandwidth[currentVl] / weightMultiplier);
		if (!vlweight)
			vlweight = weight;
		vlblockp[currentEntry++].Weight = vlweight;

		currentVl = bitset_find_next_one(vlsInUse, currentVl + 1);
#endif

	return QosFillStlVlarbTable(topop, nodep, portp, &dor_qos, arbp);
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
	int dor_lid_offset = 0;

	if (topop->routing.data.dor.alternateRouting && endPortp->portData->lmc > 0) {
		numLids--;		//leave one lid for updn programming
		if (sm_UpDnBaseLid) {
			//leave base lid for updn programming
			portnos++;		
			dor_lid_offset = 1;
		}
	}

	memset((void*)portnos, 0xff, sizeof(uint8_t) * numLids);

	if (topop->routing.data.dor.alternateRouting && endPortp->portData->lmc == 0) {
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
			IB_LOG_VERBOSE_FMT(__func__,
			       "Found neighbor NodeGUID "FMT_U64" [%s]",
			       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp));
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
				++endPort;
			}
		}

		if (endPort) {
			qsort(orderedPorts, endPort, sizeof(SwitchportToNextGuid_t),
			      _compare_lids_routed);
			if (endPort > numLids) endPort = numLids;

			offset = sm_balance_base_lids(topop, orderedPorts, endPort);
			
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
	wantDownstream = topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f));
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
			isDownstream = (topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
			if (isDownstream)
				isDownstream = 1;
			if (wantDownstream != isDownstream) {
				continue;
			} else if (wantDownstream) {
				/* end switch is down stream to switchp  and nodep is also down stream to switchp*/
				/* check if endSwitchp is down stream to nodep */
				ij = DorBitMapsIndex(nodep->swIdx, endSwitchp->swIdx);
				isDownstream = (topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
				if (!isDownstream)
					continue;	//though nodep is down stream to switchp, it does not lead towards the end switch
								// check for other down stream switches of switchp
			} else if (!wantDownstream) {
				/* We have to go upstream. consider nodep only if it is upstream to switchp*/
				ij = DorBitMapsIndex(nodep->swIdx, switchp->swIdx);
				isDownstream = (topop->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f)));
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

static int
_get_port_group(Topology_t *topop, Node_t *switchp, Node_t *endNodep, uint8_t *portnos)
{		
//#warning MWHEINZ FIXME - this function is a no-op and is never called.
	return 0;
}

static Status_t
_setup_xft(Topology_t *topop, Node_t *switchp, Node_t *endNodep,
                 Port_t *endPortp, uint8_t *portnos)
{
	Status_t status;
	int ij;
	Node_t *endSwitchp;

	endSwitchp = _get_switch(topop, endNodep, endPortp);
	if (endSwitchp == NULL) {
		IB_LOG_ERROR_FMT(__func__,
		       "Failed to find destination switch attached to NodeGUID "FMT_U64" [%s] Port %d",
		       endNodep->nodeInfo.NodeGUID, sm_nodeDescString(endNodep), endPortp->index);
		return VSTATUS_BAD;
	}

	memset(portnos, 0xff, (1 << endPortp->portData->lmc) * sizeof(uint8_t));

	//Setup the DOR route
	if (!topop->routing.data.dor.alternateRouting || endPortp->portData->lmc > 0) {
		ij = DorBitMapsIndex(switchp->swIdx, endSwitchp->swIdx);
		if ((topop->routing.data.dor.dorClosure[ij >> 5] & (1 << (ij & 0x1f)))) {
			status = _get_outbound_port_dor(topop, switchp, endNodep, endPortp, portnos);
			if (smDorRouting.debug)
				IB_LOG_VERBOSE_FMT(__func__,
				       "Routing SW "FMT_U64" [%s] to DLID 0x%04x via DOR: EgressPort %d",
				       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endPortp->portData->lid, portnos[0]);
			if (status != VSTATUS_OK) {
				/* Fill portnos with 0xff. If alternate routing is enabled, 
				 * _get_outbound_port_updn will overwrite the 0xff for the port number
				 * corresponding to the updn lid
				 */
				memset(portnos, 0xff, (1 << endPortp->portData->lmc) * sizeof(uint8_t));
				if (!topop->routing.data.dor.alternateRouting)
					return status;
			}
		}
	}
	//If alternate routing is possible, setup the UpDn route
	if (topop->routing.data.dor.alternateRouting) {
		status = _get_outbound_port_updn(topop, switchp, endNodep, endPortp, portnos);
		if (smDorRouting.debug)
			IB_LOG_VERBOSE_FMT(__func__,
			       "Routing SW "FMT_U64" [%s] to DLID 0x%04x via Up/Down: EgressPort %d",
			       switchp->nodeInfo.NodeGUID, sm_nodeDescString(switchp), endPortp->portData->lid, portnos[0]);
		return status;
	}
	//Both routes are not possible
	return VSTATUS_BAD;
}

//===========================================================================//
// ROUTING HOOKS
//

static Status_t
_pre_process_discovery(Topology_t *topop, void **outContext)
{
	Status_t status;
	DorDiscoveryState_t *state;
	int i;

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
			topop->routing.data.dor.numToroidal++;
		}
		smDorRouting.dimension[i].created = 0;
	}

	if (topop->routing.data.dor.numToroidal) {
		topop->routing.data.dor.minReqScs = 4;
	} else {
		topop->routing.data.dor.minReqScs = 2;
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
	DorNode_t *dnodep = nodep->routing.dor;

	if (dnodep == NULL) {
		// should only apply to the first node, since propagation will
		// take care of the rest
		status = vs_pool_alloc(&sm_pool, sizeof(*dnodep), (void *)&dnodep);
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("_discover_node: Failed to allocate storage for DOR node structure; rc:", status);
			return status;
		}
		memset((void *)dnodep, 0, sizeof(*dnodep));

		dnodep->node = nodep;
		nodep->routing.dor = dnodep;
	}

	return _propagate_coord_through_port((DorDiscoveryState_t *)context,
		topop, nodep, portp);
}

static Status_t
_post_process_discovery(Topology_t *topop, Status_t discoveryStatus, void *context)
{
	int i, j, idx, general_warning = 0, specific_warning = 0;
	DorDiscoveryState_t *state = (DorDiscoveryState_t *)context;

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

	topop->routing.data.dor.numDimensions = state->nextDimension;

	if (topop->routing.data.dor.numDimensions < smDorRouting.dimensionCount) {
		if (topop->routing.data.dor.numDimensions)
			IB_LOG_WARN_FMT(__func__, "Only %d of the %d configured dimensions discovered",
							topop->routing.data.dor.numDimensions, smDorRouting.dimensionCount);
		else
			IB_LOG_WARN_FMT(__func__, "No dimensions discovered ! (%d dimensions were configured)",
							smDorRouting.dimensionCount);
		for (i = 0; i < smDorRouting.dimensionCount; i++) {
			if (smDorRouting.dimension[i].created)
				continue;
			IB_LOG_WARN_FMT(__func__, "Dimension containing port pair %d %d not found",
							smDorRouting.dimension[i].portPair[0].port1, smDorRouting.dimension[i].portPair[0].port2);
		}
	} else 	if (topop->routing.data.dor.numDimensions > smDorRouting.dimensionCount) {
		IB_LOG_WARN_FMT(__func__, "Fabric programming inconsistency ! %d dimensions found but only %d dimensions were configured",
							topop->routing.data.dor.numDimensions, smDorRouting.dimensionCount);
	}


	if (smDorRouting.debug)
		IB_LOG_INFINI_INFO_FMT(__func__,
		       "Number of dimensions: %d\n",
		       topop->routing.data.dor.numDimensions);

	for (i = 0; i < topop->routing.data.dor.numDimensions; ++i) {
		topop->routing.data.dor.dimensionLength[i] =
			topop->routing.data.dor.coordMaximums[i] - topop->routing.data.dor.coordMinimums[i] + 1;

		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Dimension %d: length %d [%s]\n",
			       i, topop->routing.data.dor.dimensionLength[i],
			       topop->routing.data.dor.toroidal[i] ? "toroidal" : "not toroidal");
	}

	if (state->toroidalOverflow)
		IB_LOG_WARN_FMT(__func__,
		       "Too many toroidal dimensions found. Only the first 4 will be routed cycle-free");

	if (topop->routing.data.dor.numToroidal == 0) {
		topop->routing.data.dor.cycleFreeRouting = 0;
	} else if (state->scsAvailable < 2) {
		IB_LOG_WARN("not enough SCs available to route cycle-free SCs:",
		            state->scsAvailable);
		topop->routing.data.dor.cycleFreeRouting = 0;
	} else if (topop->routing.data.dor.numToroidal > 4) {
		IB_LOG_WARN0("not enough SLs available to route cycle-free");
		topop->routing.data.dor.cycleFreeRouting = 0;
	} else {
		if (smDorRouting.debug)
			IB_LOG_INFINI_INFO_FMT(__func__,
			       "Routing with credit loop avoidance (%d SCs available)",
			       MIN(1 << (state->scsAvailable - 1), 15));
		topop->routing.data.dor.cycleFreeRouting = 1;
	}

	if (!topop->routing.data.dor.cycleFreeRouting) {
		// no cycle-free routing, so the only condition is that
		// there be more than 1 SC available
		if (state->scsAvailable < 2) {
			IB_LOG_WARN("not enough SCs available on the fabric to route around mesh failures SCs:",
			            state->scsAvailable);
			topop->routing.data.dor.alternateRouting = 0;
		} else {
			if (smDorRouting.debug)
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Routing with alternate fabric (%d SCs available)",
				       MIN(1 << (state->scsAvailable - 1), 15));
			topop->routing.data.dor.alternateRouting = 1;
		}
	} else {
		// cycle-free routing... check SC space
		if (state->scsAvailable < 2) {
			IB_LOG_WARN("not enough SCs available on the fabric to route around mesh failures SCs:",
			            state->scsAvailable);
			topop->routing.data.dor.alternateRouting = 0;
		} else if (topop->routing.data.dor.numToroidal > 3) {
			IB_LOG_WARN0("not enough SLs available on the fabric to route around mesh failures");
			topop->routing.data.dor.alternateRouting = 0;
		} else {
			if (smDorRouting.debug)
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Routing with alternate fabric (%d SCs available)",
				       MIN(1 << (state->scsAvailable - 1), 15));
			topop->routing.data.dor.alternateRouting = 1;
		}
	}

	setup_slscmap(topop, state->scsAvailable);

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
	if (smDorRouting.updn_mc_same_spanning_tree && sm_useIdealMcSpanningTreeRoot) {
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
_post_process_routing(Topology_t *topop, Topology_t * old_topop)
{
	Status_t status;

	topop->routing.data.dor.closure_max_sws = topop->max_sws;

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
		((old_topop->routing.data.dor.dorClosureSize != topop->routing.data.dor.dorClosureSize) ||
		 (memcmp(old_topop->routing.data.dor.dorClosure, topop->routing.data.dor.dorClosure,
				topop->routing.data.dor.dorClosureSize)))) {
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
_post_process_routing_copy(Topology_t *src_topop, Topology_t *dst_topop)
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
	int srcLids, dstLids;

	if (!topop->routing.data.dor.alternateRouting) {
		return VSTATUS_BAD;
	}

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
	int srcLids, dstLids;
	int srcUpDnLid, dstUpDnLid, dor_lid_offset = 0;
	uint32_t closure;

	srcSwitchp = _get_switch(topop, srcPortp->portData->nodePtr, srcPortp);
	dstSwitchp = _get_switch(topop, dstPortp->portData->nodePtr, dstPortp);

	if (srcSwitchp == NULL || dstSwitchp == NULL) {
		*outSrcLen = *outDstLen = 0;
		return VSTATUS_OK;
	}


	ij = DorBitMapsIndex(srcSwitchp->swIdx, dstSwitchp->swIdx);
	closure = topop->routing.data.dor.dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	if (closure) {
		/* check if reverse path also has dor closure*/
		ij = DorBitMapsIndex(dstSwitchp->swIdx, srcSwitchp->swIdx);
		closure = topop->routing.data.dor.dorClosure[ij >> 5] & (1 << (ij & 0x1f));
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
	if (!topop->routing.data.dor.alternateRouting) {
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

		if (slid == PERMISSIVE_LID && dlid == PERMISSIVE_LID) {
			// both lids are permissive; use dor closure to determine
			updn = closure ? 0 : 1;
		} else if (srcPortp->portData->lmc == 0 || dstPortp->portData->lmc == 0) {
			// one of the ports doesn't support lmc... must use updn
			updn = 1;
		} else if (  (  slid == PERMISSIVE_LID
		             || slid == srcUpDnLid)
		          && (  dlid == PERMISSIVE_LID
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
		if (slid == PERMISSIVE_LID) {
			*outSrcLen = topop->routing.data.dor.alternateRouting
			          	? srcLids - 1 : srcLids;
			for (i = 0, j = dor_lid_offset; i < *outSrcLen; ++i, j++)
				outSrcLids[i] = srcPortp->portData->lid + j;
		} else {
			*outSrcLen = 1;
			*outSrcLids = slid;
		}
		if (dlid == PERMISSIVE_LID) {
			*outDstLen = topop->routing.data.dor.alternateRouting
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

// Expressing a binary number as a set of permutations of boolean
// values where each bit represents whether the path would cross
// the wrap-around edge in that dimension, then the SL is simply
// the final decimal representation of the permutation cooresponding
// to the given path (+ 1, if we have alternate routing, in which case
// SL 0 is for Up/Dn).
//
// Note that this means we can only support 4d tori (or > 4d partial
// tori), as that would require 2^4 = 16 SLs.
//
static int
_get_sl_for_path(Topology_t *topop, Node_t *srcNodep, Port_t *srcPortp, uint32_t slid,
                 Node_t *dstNodep, Port_t *dstPortp, uint32_t dlid)
{
	int d, p = 0;
	int sl = 0;
	int ij;
	uint32_t closure;
	Node_t *srcSwitchp, *dstSwitchp;

	if (  !topop->routing.data.dor.cycleFreeRouting
	   && !topop->routing.data.dor.alternateRouting)
		goto end;

	srcSwitchp = _get_switch(topop, srcNodep, srcPortp);
	dstSwitchp = _get_switch(topop, dstNodep, dstPortp);

	if (srcSwitchp == NULL || dstSwitchp == NULL || srcSwitchp == dstSwitchp)
		goto end;

	if (smDorRouting.debug)
		IB_LOG_VERBOSE_FMT(__func__,
		       "Path is between SwitchGUID "FMT_U64" [%s] SwIdx %d to SwitchGUID "FMT_U64" [%s] SwIdx %d\n",
		       srcSwitchp->nodeInfo.NodeGUID, sm_nodeDescString(srcSwitchp), srcSwitchp->swIdx,
		       dstSwitchp->nodeInfo.NodeGUID, sm_nodeDescString(dstSwitchp), dstSwitchp->swIdx);

	// route on the backup SC if there's no DOR path between the two nodes
	ij = DorBitMapsIndex(srcSwitchp->swIdx, dstSwitchp->swIdx);
	closure = topop->routing.data.dor.dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	if (closure) {
		/* check DOR closure on reverse path too*/
		ij = DorBitMapsIndex(dstSwitchp->swIdx, srcSwitchp->swIdx);
		closure = topop->routing.data.dor.dorClosure[ij >> 5] & (1 << (ij & 0x1f));
	}

	if (closure && topop->routing.data.dor.alternateRouting) {
			/* if either of the ports have an LMC of 0 or the slid/dlid are up/dn lid
			 * then can't use DOR and have to use only Up/Dn
			 */
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

	if (!closure) {
		// but only if the alternate route is enabled for use
		if (topop->routing.data.dor.alternateRouting)
			sl = 0;
	} else if (topop->routing.data.dor.cycleFreeRouting) {
		// it's DOR and we're routing cycle-free. compute the SL from the path
		for ( d = 0
		    ; d < topop->routing.data.dor.numDimensions
		    ; ++d) {
			if (_is_path_toroidal_in_dimension(topop, srcSwitchp, dstSwitchp, d)) {
				if (!p)
					p = 1;
				else
					p = p << 1;
				sl |= p;
			}
		}
		if (topop->routing.data.dor.alternateRouting)
			sl++;	//sl 0 is for Up/Dn Lid
	} else {
		/* closure exists but no cycle free routing. Can probably use sl 0 or 1.
		 * Since Up/Dn uses SL 0, SL 1 might be a better choice.
		 */
		sl = 1;
	}

end:
	if (smDorRouting.debug)
		IB_LOG_VERBOSE_FMT(__func__,
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

	if (topop->routing.data.dor.dorClosure == NULL && topop->routing.data.dor.updnDownstream == NULL)
		return VSTATUS_OK;

	for (i = 0; i < last_idx; i++) {
		ij = DorBitMapsIndex(i, new_idx);
		oldij = DorBitMapsIndex(i, old_idx);

		ji = DorBitMapsIndex(new_idx, i);
		oldji = DorBitMapsIndex(old_idx, i);

		if (i == new_idx) {
			oldij = oldji = DorBitMapsIndex(old_idx, old_idx);
		}

		if (topop->routing.data.dor.dorClosure != NULL) {
			if (topop->routing.data.dor.dorClosure[oldij >> 5] & (1 << (oldij & 0x1f))) {
				topop->routing.data.dor.dorClosure[ij >> 5] |= 1 << (ij & 0x1f);
				//reset old index value to 0 as it is no longer valid
				topop->routing.data.dor.dorClosure[oldij >> 5] &= ~((uint32_t)(1 << (ij & 0x1f)));
			} else {
				topop->routing.data.dor.dorClosure[ij >> 5] &=  ~((uint32_t)(1 << (ij & 0x1f)));
			}

			if (topop->routing.data.dor.dorClosure[oldji >> 5] & (1 << (oldji & 0x1f))) {
				topop->routing.data.dor.dorClosure[ji >> 5] |= 1 << (ji & 0x1f);
				//reset old index value to 0
				topop->routing.data.dor.dorClosure[oldji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			} else {
				topop->routing.data.dor.dorClosure[ji >> 5] &=  ~((uint32_t)(1 << (ji & 0x1f)));
			}
		}

		if (topop->routing.data.dor.updnDownstream != NULL) {
			if (topop->routing.data.dor.updnDownstream[oldij >> 5] & (1 << (oldij & 0x1f))) {
				topop->routing.data.dor.updnDownstream[ij >> 5] |= 1 << (ij & 0x1f);
				//reset old index value to 0
				topop->routing.data.dor.updnDownstream[oldij >> 5] &= ~((uint32_t)(1 << (ij & 0x1f)));
			} else {
				topop->routing.data.dor.updnDownstream[ij >> 5] &=  ~((uint32_t)(1 << (ij & 0x1f)));
			}

			if (topop->routing.data.dor.updnDownstream[oldji >> 5] & (1 << (oldji & 0x1f))) {
				topop->routing.data.dor.updnDownstream[ji >> 5] |= 1 << (ji & 0x1f);
				//reset old index value to 0
				topop->routing.data.dor.updnDownstream[oldji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			} else {
				topop->routing.data.dor.updnDownstream[ji >> 5] &= ~((uint32_t)(1 << (ji & 0x1f)));
			}
		}

	}

	return VSTATUS_OK;	
}


static int check_closure_changes(Topology_t * oldtp, Topology_t * newtp, Node_t *switchp,
								 int check_dor_closure, int check_updn_closure)
{
	int oldnumsws = oldtp->routing.data.dor.closure_max_sws;
	int numsws = newtp->routing.data.dor.closure_max_sws;
	int i, j, ij, oldij, ji, oldji;
	uint32_t new_closure, old_closure;

	i = switchp->swIdx;
	//Check if this switch's dor and updn closure has changed with other switches
   	for (j = 0, ij = i * numsws, oldij = i * oldnumsws; ((j < numsws) && (j < oldnumsws)); j++, ij++, oldij++) {

		ji = (j * numsws) + i;
		oldji = (j * oldnumsws) + i;

		if (check_dor_closure) {
			//dor closure
			new_closure = newtp->routing.data.dor.dorClosure[ij >> 5] & (1 << (ij & 0x1f));
			old_closure = oldtp->routing.data.dor.dorClosure[oldij >> 5] & (1 << (oldij & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;
	
			if (new_closure != old_closure) 
				return 1;	

			//check ji
			new_closure = newtp->routing.data.dor.dorClosure[ji >> 5] & (1 << (ji & 0x1f));
			old_closure = oldtp->routing.data.dor.dorClosure[oldji >> 5] & (1 << (oldji & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;	

			if (new_closure != old_closure) 
				return 1;

		}

		if (check_updn_closure) {
			new_closure = newtp->routing.data.dor.updnDownstream[ij >> 5] & (1 << (ij & 0x1f));
			old_closure = oldtp->routing.data.dor.updnDownstream[oldij >> 5] & (1 << (oldij & 0x1f));

			if (new_closure)
				new_closure = 1;

			if (old_closure)
				old_closure = 1;

			if (new_closure != old_closure) 
				return 1;

			//check ji
			new_closure = newtp->routing.data.dor.updnDownstream[ji >> 5] & (1 << (ji & 0x1f));
			old_closure = oldtp->routing.data.dor.updnDownstream[oldji >> 5] & (1 << (oldji & 0x1f));

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

	if (newtp->routing.data.dor.dorClosure == NULL  || oldtp->routing.data.dor.dorClosure == NULL)
		check_dor_closure = 0;

	if (newtp->routing.data.dor.updnDownstream == NULL || oldtp->routing.data.dor.updnDownstream == NULL)
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
	if (topop->routing.data.dor.dorClosure != NULL) {
		vs_pool_free(&sm_pool, (void *)topop->routing.data.dor.dorClosure);
		topop->routing.data.dor.dorClosure = NULL;
	}

	if (topop->routing.data.dor.updnDownstream != NULL) {
		vs_pool_free(&sm_pool, (void *)topop->routing.data.dor.updnDownstream);
		topop->routing.data.dor.updnDownstream = NULL;
	}
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


static Status_t
_make_routing_module(RoutingModule_t * rm)
{
	rm->name = "dor-updown";
	rm->funcs.pre_process_discovery = _pre_process_discovery;
	rm->funcs.discover_node = _discover_node;
	rm->funcs.discover_node_port = _discover_node_port;
	rm->funcs.post_process_discovery = _post_process_discovery;
	rm->funcs.post_process_routing = _post_process_routing;
	rm->funcs.post_process_routing_copy = _post_process_routing_copy;
	rm->funcs.setup_switches_lrdr = _setup_switches_lrdr;
	rm->funcs.setup_xft = _setup_xft;
	rm->funcs.get_port_group = _get_port_group;
	rm->funcs.select_slsc_map = _generate_slsc_map;
	rm->funcs.select_scsl_map = _generate_scsl_map;
	rm->funcs.select_scvl_map = _select_scvl_map;
	rm->funcs.select_vlvf_map = _select_vlvf_map;
	rm->funcs.fill_stl_vlarb_table = _fill_stl_vlarb_table;
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
	return sm_routing_addModuleFac("dor-updown", _make_routing_module);
}

