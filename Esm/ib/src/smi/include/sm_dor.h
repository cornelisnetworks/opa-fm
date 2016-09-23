/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//			
// FILE NAME
//    sm_dor.h
//
// DESCRIPTION
//    This file contains the SM Dor/Torus Routing SM definitions.
//
//===========================================================================//

#ifndef	_SM_DOR_H_
#define	_SM_DOR_H_


#define SM_UPDN_MC_SAME_SPANNING_TREE	1   /* controls default behavior of whether updn and multicast have same spanning tree */

typedef struct  _DorTopology {
	// mesh sizing
	uint8_t numDimensions;
	uint8_t dimensionLength[SM_DOR_MAX_DIMENSIONS];
	// per-dimension coordinate bounds.  coordinates are assumed
	// contiguous within these bounds
	int8_t coordMaximums[SM_DOR_MAX_DIMENSIONS];
	int8_t coordMinimums[SM_DOR_MAX_DIMENSIONS];
	// true if cycle-free routing is enabled
	uint8_t cycleFreeRouting;
	// true if the alternate VL for routing around switch failures is enabled
	uint8_t alternateRouting;
	// for each dimension, true if toroidal
	uint8_t toroidal[SM_DOR_MAX_DIMENSIONS];
	// the configured number of toroidal dimensions
	uint8_t numToroidal;
	// count of toroidal dimensions found till now
	uint8_t toroidal_count;
	// minimum number of SCs required to route the fabric correctly
	uint8_t minReqScs;
	// maps the index of all dimensions to an index into only the
	// toroidal dimensions
	uint8_t toroidalMap[SM_DOR_MAX_DIMENSIONS];
	// flat 2d bitfield indexed on switch indices marking whether
	// the cycle-free DOR path is realizable between nodes
	size_t dorClosureSize;
	uint32_t *dorClosure;
	// flat 2d bitfield indexed on switch indices marking whether
	// j is downstream of i for a given (i, j) pair using up/down
	// routing (basically directional closure)
	uint32_t *updnDownstream;
	uint32_t closure_max_sws;   //the switch count when the closures were computed
} DorTopology_t;


typedef struct _DorNode {
	int8_t coords[SM_DOR_MAX_DIMENSIONS];
	Node_t *node;
	struct _DorNode *left[SM_DOR_MAX_DIMENSIONS];
	struct _DorNode *right[SM_DOR_MAX_DIMENSIONS];
} DorNode_t;

#endif
