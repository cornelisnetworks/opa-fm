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

//======================================================================//
//                                                                      //
// FILE NAME                                                            //
//    sm_qos.c                                                          //
//                                                                      //
// DESCRIPTION                                                          //
//    This file contains SM QoS routines for setting up SL/SC/VL        //  
//    mapping and VL Arbitration tables.                                //
//                                                                      //
// FUNCTIONS                                                            //
//    sm_initialize_VLArbitration   Program VLArb                       //
//                                                                      //
//======================================================================//

#include <stdlib.h>				/* for qsort */
#include "os_g.h"
#include "ib_status.h"
#include "sm_l.h"
#include "sm_qos.h"

//			VF, SL, SC, VL mapping:
//
//			The relationship of VFs, SLs, SCs and VLs is defined as
//
//			   n [1-3]  1   m    1   1
//			VF <---> SL <---> SC <---> VL
//
//			Each VF will be mapped to between 1 and 3 SLs. An SL can be shared
//			by any number of VFs.
//
//			Each SL will be mapped to at least one SC. An SC can be mapped to
//			only one SL.
//
//			Each SC will be mapped to 1 VL and each VL is mapped to 1 SC.
//
//			In the case where an SL is mapped to more than one SC, this entry
//			will correspond to the base SC, leaving the mapping of the SL to
//			base SC + 1, base SC + 2, etc..
uint8_t sm_SLtoSC[STL_MAX_SLS];
uint8_t sm_SCtoSL[STL_MAX_SCS];

/**
  Copy back the results from an aggregate response into the topology, performing byte swapping as necessary.
	@c aggr and @c end must denote a contiguous range of memory <tt>[aggr, end)<\tt>.

	@parma smaportp the SMA port (port 0 on a switch, port 1 on an HFI, typically).
	@return VSTATUS_BAD on unhandleable runtime errors, VSTATUS_OK otherwise, even if there's a segment with the error bit set.
*/
static Status_t
sm_node_handleGetRespAggr(Node_t * nodep, Port_t * smaportp, STL_AGGREGATE * aggr, STL_AGGREGATE * end);

/**
	Update SCVLt/SCVLnt for a node (usu. switch) from @c aggr.  Assumes aggr->Data is in network order.
*/
static Status_t
sm_aggregateToScvl(Node_t * nodep, STL_AGGREGATE * aggr);

/**
	Update curArb.vlarb* fields from @c agrr.  Assumes aggr->Data is in network order.
*/ 
static Status_t
sm_aggregateToVlarb(Node_t * nodep, STL_AGGREGATE * aggr);


/**
	Update BufferControlTable  fields from @c agrr.  Assumes aggr->Data is in network order.
*/
static Status_t
sm_aggregateToBfrctrl(Node_t * nodep, STL_AGGREGATE * aggr);

/**
	Do LR Get(Aggregate) of attributes on @c nodep with values from SMA at @c smaportp.

	See @fn sm_node_updateFields() for interface details.

	@return VSTATUS_OK only if all operations succeed, non-OK otherwise.  Will return VSTATUS_BAD if an individual aggregate-segment operation failed.
*/
static Status_t
sm_node_updateFromSma_aggregate(IBhandle_t fd, STL_LID slid, Node_t * nodep, Port_t * smaportp);

/**
	Fallback code for SMAs that don't support aggregate operations.

	See @fn sm_node_updateFields() for interface details.
*/
static Status_t
sm_node_updateFromSma_solo(IBhandle_t fd, STL_LID slid, Node_t * nodep, Port_t * smaportp);

/**
	Update @c nodep SLSC, SCSL, SCVLt, and SCVLnt values from @c nodep found in @c srcTopop (usually topology from prior sweep).

	@return VSTATUS_OK on success (even if not updated), something else if things go really wrong.
*/
static Status_t
sm_node_updateFromTopo(Node_t * nodep, Topology_t * oldTopop, Topology_t * curTopop);

#define DO_INLINE_SET 0

/**
	Send all changes pending for all nodes in @c topop.  Continues sending even if updating one or more SMAs fails.  Releases changes for a node if update was sucessful.

	@param firstError [out,optional] If not NULL then on non-OK return val (except Node_t::index out-of range errors), @c *firstError will be the node that triggered the error.
*/
static Status_t
sm_syncSmaChanges(Topology_t * topop, Node_t ** firstError);

static Status_t
sm_node_syncSma_aggregate(Topology_t * topop, Node_t * nodep, Port_t * smaportp);

static Status_t
sm_node_syncSma_solo(Topology_t * topop, Node_t * nodep, Port_t * smaportp);

static void
setWeightMultiplier(Qos_t * qos);

// The following is for uniform qos
static Qos_t *sm_Qos = NULL;

int sm_check_node_cache_valid(Node_t *);

static Status_t
sm_setSmaChanged(Topology_t * topop, Node_t * nodep, boolean changed)
{
	if (!topop->smaChanges) {
		if (topop->num_nodes == 0)
			return VSTATUS_BAD;
		if (vs_pool_alloc(&sm_pool, sizeof(bitset_t), (void**)&topop->smaChanges) != VSTATUS_OK)
			return VSTATUS_BAD;

		bitset_init(&sm_pool, topop->smaChanges, topop->num_nodes + 1);
	}

	if (changed)
		bitset_set(topop->smaChanges, nodep->index);
	else
		bitset_clear(topop->smaChanges, nodep->index);
	return VSTATUS_OK;
}

static Status_t
sm_markSmaChanged(Topology_t * topop, Node_t * nodep)
{
	return sm_setSmaChanged(topop, nodep, 1);
}

static Status_t
 sm_clearSmaChanged(Topology_t * topop, Node_t * nodep)
{
	return sm_setSmaChanged(topop, nodep, 0);
}

static Status_t PopulateSCtoSL(RoutingModule_t *rm, const Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics, uint8_t *SLtoSC, uint8_t *SCtoSL);
void sm_setup_qos(RoutingModule_t *rm, Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics, const uint8_t *SLtoSC);
void sm_setup_qos_1vl(RoutingModule_t *rm, Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics);

Status_t
sm_update_bw(RoutingModule_t *rm, VirtualFabrics_t *vfs)
{
	int bwInUse = 0;
	int nonQosVfs = 0;
	int qosNoBwVfs = 0;
	int vf;

	if (!vfs)  {
		return VSTATUS_OK;
	}

	IB_LOG_INFINI_INFO_FMT("", "VF Bandwidth Allocations :");

	// Total up allocated BW (of active VFs)
	for (vf = 0; vf < vfs->number_of_vfs_all; vf++) {
		VF_t *vfp = &vfs->v_fabric_all[vf];
		if (vfp->standby) continue;
		if (vfp->qos_enable) {
			if (!vfp->priority) {
				if (vfp->percent_bandwidth == UNDEFINED_XML8) {
					qosNoBwVfs++;
				} else {
					bwInUse += vfp->percent_bandwidth;
				}
			}
		} else {
			nonQosVfs++;
		}
	}

	// One share for all nonQos VFs combined (minimum of 5% BW)
	// and one share for each Qos VF without bandwidth (minimum of 1% BW)
	int freeBw = 100 - bwInUse;
	int neededFreeBw = (nonQosVfs?5:0) + qosNoBwVfs;
	int shareSz = 0;
	int nonQosShareSz = 0;

	if (freeBw) {
		IB_LOG_INFINI_INFO_FMT("", "Unallocated BW is %d%%.", freeBw);
	}
	if (freeBw < neededFreeBw) {
		IB_LOG_ERROR_FMT(__func__, "Not enough free bandwidth for"
			" bandwidth-unconfigured VFs");
		return VSTATUS_BAD;
	}
	if (nonQosVfs || qosNoBwVfs) {
		if (!nonQosVfs) {
			shareSz = freeBw / qosNoBwVfs;
		} else {
			shareSz = freeBw / (qosNoBwVfs + 1); // +1 for nonQosVfs
			nonQosShareSz = freeBw - (shareSz * qosNoBwVfs);
			if (nonQosShareSz < 5) {
				// NonQos are supposed to share at least 5%
				nonQosShareSz = 5;
				// qosNoBwVfs must be !0, or nonQosShareSz would have been >= 5%
				// however, for code safety we coerce it to be at least 1.
				shareSz = (freeBw - nonQosShareSz) / (qosNoBwVfs?qosNoBwVfs:1);
			}
			freeBw -= nonQosShareSz;
		}
		if (qosNoBwVfs == 1) {
			IB_LOG_INFINI_INFO_FMT("", "Assigning %d%%"
				" BW to BW-unconfigured VF.", freeBw);
		} else if (qosNoBwVfs > 1) {
			IB_LOG_INFINI_INFO_FMT("", "Dividing %d%%"
				" BW across %d bandwidth-unconfigured VFs (%d%% per VF).",
				freeBw, qosNoBwVfs, shareSz);
		}
		if (nonQosVfs == 1) {
			IB_LOG_INFINI_INFO_FMT("", "Allocating %d%%"
				" BW for non QoS VF.",
				nonQosShareSz);
		} else if (nonQosVfs > 1) {
			IB_LOG_INFINI_INFO_FMT("", "Sharing %d%%"
				" BW among %d non QoS VFs.",
				nonQosShareSz, nonQosVfs);
		}
	}

	for (vf = 0; vf < vfs->number_of_vfs_all; vf++) {
		VF_t *vfp = &vfs->v_fabric_all[vf];
		if (vfp->standby) continue;

		if (vfp->qos_enable) {
			if (vfp->priority) continue;
			if (vfp->percent_bandwidth == UNDEFINED_XML8) {
				vfp->percent_bandwidth = shareSz;
				freeBw -= shareSz;
				qosNoBwVfs--;
				if (qosNoBwVfs)
					shareSz = freeBw / qosNoBwVfs;
				else
					shareSz = 0;
			}
		} else {
			// Store bandwitdth shared by all nonQos VFs
			vfp->percent_bandwidth = nonQosShareSz;
		}
	}
	// Report VFs with bandwidth allocated, and total up allocated BW
	for (vf = 0; vf < vfs->number_of_vfs_all; vf++) {
		VF_t *vfp = &vfs->v_fabric_all[vf];
		char SLs[256] = { 0 };

		if (vfp->standby) continue;

		if (vfp->resp_sl != vfp->base_sl) {
			if (vfp->mcast_sl != vfp->base_sl) {
				snprintf(SLs,sizeof(SLs),"between BaseSL %d, RespSL %d, and MulticastSL %d",
					vfp->base_sl, vfp->resp_sl, vfp->mcast_sl);
			} else {
				snprintf(SLs,sizeof(SLs),"between BaseSL %d, and RespSL %d",
					vfp->base_sl, vfp->resp_sl);
			}
		} else if (vfp->mcast_sl != vfp->base_sl) {
			snprintf(SLs,sizeof(SLs),"between BaseSL %d, and MulticastSL %d",
				vfp->base_sl, vfp->mcast_sl);
		} else {
			snprintf(SLs,sizeof(SLs),"on BaseSL %d", vfp->base_sl);
		}
		if (vfp->qos_enable) {
			if (vfp->priority) {
				IB_LOG_INFINI_INFO_FMT_VF(vfp->name, "",
					"High Priority VF; HP VFs don't participate in Bandwidth limits.");
			} else {
				if (vfp->percent_bandwidth == UNDEFINED_XML8) {
					vfp->percent_bandwidth = shareSz;
				}
				IB_LOG_INFINI_INFO_FMT_VF(vfp->name, "",
					"QoS VF; Assigned %d%% Bandwidth %s",vfp->percent_bandwidth,SLs);
			}
		} else {
			IB_LOG_INFINI_INFO_FMT_VF(vfp->name, "",
				"Non QoS VF; Sharing %d%% Bandwidth %s",nonQosShareSz,SLs);
		}
	}
	return VSTATUS_OK;
}

Status_t
sm_assign_scs_to_sls_FixedMap(RoutingModule_t *rm, VirtualFabrics_t *vfs)
{
    Status_t ret = VSTATUS_BAD;
	uint8_t SLtoSC[STL_MAX_SLS];
	uint8_t SCtoSL[STL_MAX_SCS];
	Qos_t *qos = NULL;
	int i;

	if (rm->funcs.min_vls() > rm->funcs.max_vls()) {
		IB_LOG_ERROR_FMT(__func__, "Configuration cannot be mapped. MinSupportedVLs (%d) cannot exceed MaxSupportedVLs (%d).",
			rm->funcs.min_vls(),rm->funcs.max_vls());
		return ret;
	}

	qos = sm_alloc_qos();

    memset(SLtoSC, 15, sizeof(SLtoSC));
    memset(SCtoSL, 0xff, sizeof(SCtoSL));

	for (i = 1; i <= rm->funcs.max_vls(); i++) {
    	sm_fill_SCVLMap(&qos[i]);
	}

	// Only perform SC to SL assignment on minimum supported VLs
	ret = PopulateSCtoSL(rm, &qos[sm_config.max_fixed_vls], vfs, SLtoSC, SCtoSL);
    if (ret != VSTATUS_OK) {
		sm_free_qos(qos);
    	return ret;
    }

	// 1 VL is a special case, always set it up
	if(rm->funcs.min_vls() > 1)
		sm_setup_qos_1vl(rm, &qos[1], vfs);

	// Set Qos for all supported number of VLs
	for (i = rm->funcs.min_vls(); i <= rm->funcs.max_vls(); i++) {
    	sm_setup_qos(rm, &qos[i], vfs, SLtoSC);
	}

    memcpy(sm_SLtoSC, SLtoSC, sizeof(SLtoSC));
    memcpy(sm_SCtoSL, SCtoSL, sizeof(SCtoSL));

    sm_install_qos(qos);

    return ret;
}

/*
 * Used to round VL bandwidths up/off to nearest multiple of 5. This is
 * for VLArb SMAs. All this does is set the weightMultiplier in the
 * origQos. It does not change the bw values in the origQos.
 */
static void
sm_roundVLBandwidths(Qos_t * origQos)
{
	int bwTotal;
	int vl;
	Qos_t qos;

	memcpy(&qos, origQos, sizeof(Qos_t));

	bwTotal = 0;
	for (vl = 0; vl < qos.activeVLs; vl++) {
		bwTotal += qos.vlBandwidth.bw[vl];
		assert(bwTotal <= 100);
	}

	// The following rounds the bandwidth to steps of 5
	for (vl= 0; vl < qos.activeVLs; vl++) {
		if (qos.vlBandwidth.bw[vl]) {
			if (qos.vlBandwidth.bw[vl] % 5) {
				if (qos.vlBandwidth.bw[vl] % 5 > 2) {
					qos.vlBandwidth.bw[vl] += (5 - (qos.vlBandwidth.bw[vl] % 5));
				} else {
					qos.vlBandwidth.bw[vl] -= (qos.vlBandwidth.bw[vl] % 5);
					if (!qos.vlBandwidth.bw[vl])
						qos.vlBandwidth.bw[vl] = 5;
				}
			}
		}
	}

	setWeightMultiplier(&qos);
	origQos->weightMultiplier = qos.weightMultiplier;
}

void
sm_DbgPrintQOS(Qos_t * qos)
{
    if (sm_config.sm_debug_vf)
    {
        bitset_info_log(&qos->lowPriorityVLs, "lowPriorityVLs");
        bitset_info_log(&qos->highPriorityVLs, "highPriorityVLs");
        IB_LOG_INFINI_INFO_FMT( __func__,
                                "SCVL%2d = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", qos->numVLs,
                               qos->scvl.SCVLMap[0].VL, qos->scvl.SCVLMap[1].VL, qos->scvl.SCVLMap[2].VL,
                               qos->scvl.SCVLMap[3].VL, qos->scvl.SCVLMap[4].VL, qos->scvl.SCVLMap[5].VL,
                               qos->scvl.SCVLMap[6].VL, qos->scvl.SCVLMap[7].VL);

        IB_LOG_INFINI_INFO_FMT( __func__,
                                "         0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                               qos->scvl.SCVLMap[8].VL, qos->scvl.SCVLMap[9].VL, qos->scvl.SCVLMap[10].VL,
                               qos->scvl.SCVLMap[11].VL, qos->scvl.SCVLMap[12].VL, qos->scvl.SCVLMap[13].VL,
                               qos->scvl.SCVLMap[14].VL, qos->scvl.SCVLMap[15].VL);

        IB_LOG_INFINI_INFO_FMT( __func__,
                                "         0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                               qos->scvl.SCVLMap[16].VL, qos->scvl.SCVLMap[17].VL, qos->scvl.SCVLMap[18].VL,
                               qos->scvl.SCVLMap[19].VL, qos->scvl.SCVLMap[20].VL, qos->scvl.SCVLMap[21].VL,
                               qos->scvl.SCVLMap[22].VL, qos->scvl.SCVLMap[23].VL);

        IB_LOG_INFINI_INFO_FMT( __func__,
                                "         0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                               qos->scvl.SCVLMap[24].VL, qos->scvl.SCVLMap[25].VL, qos->scvl.SCVLMap[26].VL,
                               qos->scvl.SCVLMap[27].VL, qos->scvl.SCVLMap[28].VL, qos->scvl.SCVLMap[29].VL,
                               qos->scvl.SCVLMap[30].VL, qos->scvl.SCVLMap[31].VL);

        int vl;
        int totalBw = 0;
        for (vl = bitset_find_first_one(&qos->lowPriorityVLs); vl > -1;
             vl = bitset_find_next_one(&qos->lowPriorityVLs, vl+ 1)) {
            IB_LOG_INFINI_INFO_FMT( __func__, "lowPriority VL%d has bandwidth %d", vl,
                                   qos->vlBandwidth.bw[vl]);
            totalBw += qos->vlBandwidth.bw[vl];
        }
        IB_LOG_INFINI_INFO_FMT( __func__, "lowPriority has total bandwidth %d", totalBw);
    }
}

/*
 * Maps SCs to SLs. Requires that the SC to VL map is sequential.
 * (i.e., that if SC0 maps to VL0, then SC1 to VL1 and so on) In addition,
 * the function prohibits oversubscribing VLs.
 */
static Status_t
PopulateSCtoSL(RoutingModule_t *rm, const Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics, uint8_t *SLtoSC, uint8_t *SCtoSL)
{
	// Populate the SLtoSC and SCtoSL
	int sc, sl, vl, vf, numSCs;
	bitset_t mappedSLs;
	bitset_t freeVLs;
	bitset_t neededVLs;
	Status_t ret = VSTATUS_BAD;

	if (!bitset_init(&sm_pool, &mappedSLs, STL_MAX_SLS)
	||  !bitset_init(&sm_pool, &neededVLs, qos->activeVLs)
	||  !bitset_init(&sm_pool, &freeVLs, qos->activeVLs)) {
		IB_FATAL_ERROR("PopulateSCtoSL: No memory for QoS setup, exiting.");
	}

	// Mark all VLs as free, except VL15, which is reserved for SMA traffic.
	bitset_set_all(&freeVLs);
	if (qos->activeVLs > 15)
		bitset_clear(&freeVLs, 15);

	// Assign SCs to SLs. Must map all VFs (not just active)
	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		VF_t *vfp = &VirtualFabrics->v_fabric_all[vf];
		int sl_i, vl_i;

		for (sl_i = 0; sl_i < 3; sl_i++) {
			boolean mc_sl = 0;

			// Map base_sl, resp_sl, and mcast_sl.
			//
			// If the specified sl is already mapped, skip over it.
			// That situation will arise when there are multiple non-QOS
			// VFs in the configuration, or if resp_sl (or mcast_sl)
			// points to the same SL as base_sl.
			//
			switch (sl_i) {
				default:
				case 0: sl = vfp->base_sl; mc_sl = 0; break;
				case 1:	sl = vfp->resp_sl; mc_sl = 0; break;
				case 2: sl = vfp->mcast_sl; mc_sl = 1; break;
			}

			if (bitset_test(&mappedSLs, sl)) {
				continue;
			}
			bitset_set(&mappedSLs, sl);

			// Returns the # of VLs the routing module requires to route
			// this SL. In theory this could be different for every SL.
			numSCs = rm->funcs.num_routing_scs(sl, mc_sl);

			for (vl = bitset_find_first_one(&freeVLs);
				 vl != -1;
				 vl = bitset_find_next_one(&freeVLs, vl + 1)) {

				// Find the lowest numbered SC that is mapped to this VL.
				// We have a weird problem that we are not guaranteed that
				// the first #numVL SC entries are mapped to the first 
				// #numVL VLs, so we do a brute-force search.
				for (sc = 0; sc <= (STL_MAX_SCS - numSCs); sc++) {
					if (qos->scvl.SCVLMap[sc].VL == vl) break;
				}

				if (sc >= (STL_MAX_SCS - numSCs)) {
					// This vl isn't mapped to an SC
					// Take it out of the free list, and find the next one
					bitset_clear(&freeVLs, vl);
					bitset_set(&neededVLs, vl);
					continue;
				}
				// Make sure we have #numSCs sequential SCs mapped to #numSCs
				// sequential free VLs. As above, we can't guarantee a unique
				// SC2VL mapping, so we have to check it here. The requirement
				// that the VLs be sequential is a hardware limitation.
				for (vl_i = 0; vl_i < numSCs; vl_i++) {
					if ((qos->scvl.SCVLMap[sc + vl_i].VL != vl + vl_i)
						|| (!bitset_test(&freeVLs, vl + vl_i))) {
						break;
					}
				}
				if (vl_i != numSCs) {
					// Not enough consecutive SCs/VLs, keep looking
					continue;
				}
				// Found a set of SC/VLs that work
				break;
			}
			if (vl == -1) {
				IB_LOG_ERROR_FMT(__func__, "Configuration cannot be mapped. Requires more than %d VLs.",qos->activeVLs);
				goto fail;
			}

			// The SL to SC map records the first of numSCs mapped to the SL,
			// we assume the others are adjacent. However, the reverse SC to SL
			// map explicitly shows all SC to SL mappings.
			for (vl_i = 0; vl_i < numSCs; vl_i++) {
				bitset_clear(&freeVLs, vl + vl_i);
				bitset_set(&neededVLs, vl + vl_i);
				SCtoSL[sc + vl_i] = sl;
			}
			SLtoSC[sl] = sc;
		}
	}
	ret = VSTATUS_OK;
	sm_needed_vls = bitset_nset(&neededVLs); // update to just the number VLs of needed for the configuration specified
	if (sm_config.sm_debug_vf) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Number of VLs needed for specified configuration : %d\n",
			sm_needed_vls);
	}

fail:
	bitset_free(&neededVLs);
	bitset_free(&freeVLs);
	bitset_free(&mappedSLs);

    return ret;
}

void
DivideBwUp(RoutingModule_t *rm, Qos_t *qos, int bw, int base_sl, int resp_sl,
	int mcast_sl, const uint8_t *SLtoSC)
{
	int sl, vl, i = 0;
	int num_scs, num_vls = 0;
	int bw_part = 0;

	// Tally up the number of VLs to divide the bandwidth into.
	// It's okay for num_vls to refer to dupes - but to avoid rounding
	// errors in integer division, we ignore resp_sl and mcast_sl if they
	// refer to the same SL as base_sp.
	num_vls = rm->funcs.num_routing_scs(base_sl, 0);
	if (resp_sl != base_sl) { 
		num_vls += rm->funcs.num_routing_scs(resp_sl, 0); 
	}
	if (mcast_sl != base_sl && mcast_sl != resp_sl) { 
		num_vls += rm->funcs.num_routing_scs(mcast_sl, 1); 
	}

	// Now we know the total # of shares, allocate the bw. Again,
	// it's okay if the same SL is processed multiple times, but
	// we also want to minimize rounding errors.
	for (i = 2; i >= 0; i--) {
		if (i == 0) {
			sl = base_sl;
		} else if (i == 1) {
			sl = resp_sl;
			if (sl == base_sl) continue;
		} else {
			sl = mcast_sl;
			if ((sl == base_sl) || (sl == resp_sl)) continue;
		}

		num_scs = rm->funcs.num_routing_scs(sl, (i==2));

		// Note the interesting effect of this loop - by doing the division
		// repeatedly we make sure we use all the assigned bandwidth, possibly
		// not evenly. For example, if we are dividing 30% of bandwidth over
		// 8 VLs, the first VL will be assigned 30/8 = 3%, the next will get
		// 27/7 = 3%, 24/6 = 4%, 20/5 = 4%, 16/4 = 4%, 12/3 = 4%, 8/2 = 4%,
		// 4/1 = 4%
		// 
		// Note the assumption that SCs are assigned to SLs in a contiguous
		// block.
		int j;
		for (j=0; j<num_scs; j++) {
			vl = qos->scvl.SCVLMap[SLtoSC[sl]+j].VL;
			bw_part = bw / num_vls;
			bw -= bw_part;
			qos->vlBandwidth.bw[vl] += bw_part;
			num_vls--;
		}
	}
}

void
sm_setup_qos(RoutingModule_t *rm, Qos_t * qos, VirtualFabrics_t *VirtualFabrics, const uint8_t *SLtoSC)
{
	// Assign SCs to SLs for Fixed mapping.
	int sl, sc, vl, vf, numSCs;
	boolean mcast_sl = 0;

	for (vl = 0; vl < STL_MAX_VLS; vl++) {
		if (!bitset_init(&sm_pool, &qos->vlvf.vf[vl], MAX_VFABRICS)) {
			IB_FATAL_ERROR("sm_setup_qos: Out of memory, exiting.");
		}
	}
	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		VF_t *vfp = &VirtualFabrics->v_fabric_all[vf];
		int i;

		if (vfp->standby) continue;

		for (i = 0; i < 3; i++) {
			if (i == 0) {
				sl = vfp->base_sl; mcast_sl = 0;
			} else if (i == 1) {
				sl = vfp->resp_sl; mcast_sl = 0;
				if (sl == vfp->base_sl) continue;
			} else {
				sl = vfp->mcast_sl; mcast_sl = 1;
				if (sl == vfp->base_sl || sl == vfp->resp_sl) continue;
			}

			assert(sl >= 0 && sl < STL_MAX_SLS);

			numSCs = rm->funcs.num_routing_scs(sl, mcast_sl);
			sc = SLtoSC[sl];

        	assert(sc >= 0 && (sc + numSCs) <= STL_MAX_SCS);

			int j;
			for (j = 0; j < numSCs; j++) {
				vl = qos->scvl.SCVLMap[sc + j].VL;
				if (vl == 15 || vl >= qos->numVLs) {
					IB_LOG_WARN_FMT(__func__, "Unexpected SC:VL Mapping:"
						"SL=%02d SC=%02d VL=%02d", sl, sc+j, vl);
					bitset_free(&qos->vlvf.vf[vl]);
					continue;
				}

				if (vfp->priority) {
					bitset_set(&qos->highPriorityVLs, vl);
					qos->vlBandwidth.highPriority[vl] = 1;
				} else {
					bitset_set(&qos->lowPriorityVLs, vl);
				}
				bitset_set(&qos->vlvf.vf[vl], vf);
			}
		}
	}

	int nonQosBw = 0;
	int nonQos_base_sl = -1, nonQos_resp_sl = -1, nonQos_mcast_sl = -1;
	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		VF_t *vfp = &VirtualFabrics->v_fabric_all[vf];

		if (vfp->standby) continue;
		if (vfp->qos_enable) {
			if (vfp->priority) continue;

			// Qos LowPriority
			DivideBwUp(rm, qos, vfp->percent_bandwidth, vfp->base_sl,
				vfp->resp_sl, vfp->mcast_sl, SLtoSC);
		} else {
			// NonQos only gets bandwidth once.
			nonQosBw = vfp->percent_bandwidth;
			nonQos_base_sl = vfp->base_sl;
			if (vfp->resp_sl != vfp->base_sl) {
				nonQos_resp_sl = vfp->resp_sl;
			}
			if (vfp->mcast_sl != vfp->base_sl) {
				nonQos_mcast_sl = vfp->mcast_sl;
			}
		}
	}
	// Add in the nonQosBw last
	if (nonQosBw) {
		if (nonQos_resp_sl == -1) nonQos_resp_sl = nonQos_base_sl;
		if (nonQos_mcast_sl == -1) nonQos_mcast_sl = nonQos_base_sl;
		DivideBwUp(rm, qos, nonQosBw, nonQos_base_sl, nonQos_resp_sl,
			nonQos_mcast_sl, SLtoSC);
	}

    sm_DbgPrintQOS(qos);
}

// Special QOS level where all non-SMA traffic goes over VL0.
void
sm_setup_qos_1vl(RoutingModule_t *rm, Qos_t * qos, VirtualFabrics_t *VirtualFabrics)
{
	int vf;

	bitset_set(&qos->lowPriorityVLs, 0);

	if (!bitset_init(&sm_pool, &qos->vlvf.vf[0], MAX_VFABRICS)) {
			IB_FATAL_ERROR("sm_setup_qos_1vl: Out of memory, exiting.");
	}

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;
		bitset_set(&qos->vlvf.vf[0], vf);
	}
	qos->vlBandwidth.bw[0] = 100;

	sm_DbgPrintQOS(qos);
}

static void
free_vlarbList(Qos_t *qos) {
	VlarbList_t *nextVlarb;		// Cached STL1 data
	while (qos->vlarbList) {
		nextVlarb = qos->vlarbList->next;
		vs_pool_free(&sm_pool, qos->vlarbList);
		qos->vlarbList = nextVlarb;
	}

}

Qos_t*
sm_alloc_qos(void)
{
	Qos_t *qos = NULL;
	int i, j;

	vs_pool_alloc(&sm_pool, sizeof(Qos_t) * (STL_MAX_VLS + 1), (void*)&qos);
	if (!qos)
		IB_FATAL_ERROR("sm_alloc_qos: No memory for QoS structures, exiting.");

	for (i=1; i<=STL_MAX_VLS; i++) {
		memset(&qos[i], 0, sizeof(qos[i]));
		qos[i].numVLs = i;
		if (!bitset_init(&sm_pool, &qos[i].highPriorityVLs, STL_MAX_VLS)
		|| !bitset_init(&sm_pool, &qos[i].lowPriorityVLs, STL_MAX_VLS)) {
			IB_FATAL_ERROR("sm_alloc_qos: No memory for QoS structures, exiting.");
		}
		for (j=0; j< STL_MAX_SCS; j++) {
			qos[i].scvl.SCVLMap[j].VL=15; // Invalid VL
		}
	}
	return qos;
}

void
sm_free_qos(Qos_t* qos)
{
	int i, j;

	if (!qos) return;

	for (i=1; i<=STL_MAX_VLS; i++) {
		bitset_free(&qos[i].highPriorityVLs);
		bitset_free(&qos[i].lowPriorityVLs);
		free_vlarbList(&qos[i]);
		for (j = 0; j < STL_MAX_VLS; j++){
			bitset_free(&qos[i].vlvf.vf[j]);
		}
	}
	(void) vs_pool_free(&sm_pool, qos);

	return;
}

void
sm_destroy_qos(void)
{
	sm_free_qos(sm_Qos);
	sm_Qos = NULL;
}

void
sm_install_qos(Qos_t *qos)
{
	Qos_t *tmpQos = sm_Qos;
	sm_Qos = qos;
	sm_free_qos(tmpQos);
}

Qos_t*
sm_get_qos(uint8_t vl)
{
	// Range of data VLs is 1 - 31
	if ((vl>=1) && (vl<STL_MAX_VLS)) {
		return &sm_Qos[vl];
	}
	return &sm_Qos[1];
}

static Status_t
sm_initialize_Switch_SLSCMap(Topology_t * topop, Node_t * switchp,
    STL_SLSCMAP * slscmapp)
{
    Status_t status = VSTATUS_OK; 
    Port_t *swportp; 

    IB_ENTER(__func__, topop, switchp, 0, 0); 

    // Let all switch ports default to the power on default for a switch port,
    // which is a default SCtoSC table which is
    // 1:1 for all combinations of ingress and egress switch ports.
    // 
    // The SL2SC map will only be set for switch port zero, as defined by the
    // following requirement in section 20.2.2.6.10 SLtoSCMappingTable:
    // For switches, it is only used to traffic sent by switch port 0
    swportp = sm_get_port(switchp, 0); 
    if (!sm_valid_port(swportp)) {
        IB_LOG_WARN_FMT(__func__, 
                        "Failed to get Port 0 of Switch " FMT_U64, 
                        switchp->nodeInfo.NodeGUID); 
        IB_EXIT(__func__, VSTATUS_BAD);
        return VSTATUS_BAD;
    }

    if (!swportp->portData->current.slsc) {
        IB_LOG_WARN_FMT(__func__,
            "SLSC for node %s nodeGuid "FMT_U64" port %d is not current",
            sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, swportp->index);
    }

    STL_SLSCMAP * curSlsc = &swportp->portData->slscMap;

    status = topop->routingModule->funcs.select_slsc_map(topop, switchp, swportp, swportp, slscmapp); 
    if (status != VSTATUS_OK) {
        IB_LOG_WARNRC("Failed to get SLSC "
                      "map from routing algorithm;  rc:", 
                      status);
        IB_EXIT(__func__, status);
        return status;
    }
    
    // Compare the port's current SLSC map against what the topology says it
    // should be. If they're different, send the new one.
    if (!swportp->portData->current.slsc ||
        memcmp((void *)curSlsc, (void *)slscmapp, sizeof(*slscmapp)) != 0 ||
        sm_config.forceAttributeRewrite) {
#if DO_INLINE_SET
        status = SM_Set_SLSCMap_LR(fd_topology, amod, sm_lid, swportp->portData->lid, slscmapp, sm_config.mkey); 
        
        if (status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                            "Failed to set SLSC Map for switch node %s nodeGuid " FMT_U64, 
                            sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID); 
            switchp->slscChange = 1;
            status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
        }
        swportp->portData->current.slsc = (status == VSTATUS_OK);
#else
        swportp->portData->dirty.slsc = 1;
        sm_markSmaChanged(topop, switchp);
#endif
    }

#if DO_INLINE_SET
    // Set SLSC Map for the switch port 0
    swportp->portData->slscMap = *slscmapp; 
#endif


    IB_EXIT(__func__, status);
    return (status);
}

static Status_t
sm_initialize_Switch_SCSLMap(Topology_t * topop, Node_t * switchp,
    STL_SCSLMAP * scslmapp)
{
    Status_t status = VSTATUS_OK;
    Port_t *swportp; 
    
    IB_ENTER(__func__, topop, switchp, 0, 0); 
    
    //
    // For switches this table is only used for traffic from switch port 0    
    swportp = sm_get_port(switchp, 0); 
    if (!sm_valid_port(swportp)) {
        IB_LOG_WARN_FMT(__func__, 
                        "Failed to get Port 0 of Switch " FMT_U64, 
                        switchp->nodeInfo.NodeGUID); 
        IB_EXIT(__func__, VSTATUS_BAD);
        return VSTATUS_BAD;
    }

    if (!swportp->portData->current.scsl) {
        IB_LOG_WARN_FMT(__func__,
          "SCSL for node %s nodeGuid "FMT_U64" port %d is not current",
          sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, swportp->index);
    }
    STL_SCSLMAP * curScsl = &swportp->portData->scslMap;

    status = topop->routingModule->funcs.select_scsl_map(topop, switchp, swportp, swportp, scslmapp); 
    if (status != VSTATUS_OK) {
        IB_LOG_WARNRC("Failed to get SCSL "
                      "map from routing algorithm; rc:", 
                      status);
        IB_EXIT(__func__, status);
        return status;
    }
    
    // compare the port's current SCSL map against what the topology says it
    // should be. If they're different, send the new one.
    if (!swportp->portData->current.scsl ||
        memcmp((void *)curScsl, (void *)scslmapp, sizeof(*scslmapp)) != 0 || sm_config.forceAttributeRewrite) {

#if DO_INLINE_SET
        status = SM_Set_SCSLMap_LR(fd_topology, amod, sm_lid, 
                                   swportp->portData->lid, scslmapp, sm_config.mkey); 
        
        if (status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                            "Failed to set SCSL Map for switch node %s nodeGuid " FMT_U64, 
                            sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID); 
            switchp->slscChange = 1;
            status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
        }
        swportp->portData->current.scsl = (status == VSTATUS_OK);
#else
        swportp->portData->dirty.scsl = 1;
        sm_markSmaChanged(topop, switchp);
#endif
    }

#if DO_INLINE_SET
    swportp->portData->scslMap = *scslmapp; 
#endif

    IB_EXIT(__func__, status);
    return (status);
}

static Status_t
WriteGen1SCSC(Topology_t * topop, Node_t * switchp, STL_LID dlid,  int numScscBlocks, STL_SCSC_MULTISET *scsc)
{
	uint32_t		amod, egressAmod;
	Status_t		status=VSTATUS_OK;
	Port_t			*swportp;
	STL_PORTMASK	ingressPorts[STL_MAX_PORTMASK];
	int 			e, i, b, numBlocks;
	int 			ingress=0, lastIngress=0;
	int 			egress=0, lastEgress=0;

	// interate over multiset blocks
	for (b=0; b<numScscBlocks; b++) {
		while (StlNumPortsSetInPortMask(scsc[b].EgressPortMask, switchp->nodeInfo.NumPorts)) {
			if (StlTestAndClearPortInPortMask(scsc[b].EgressPortMask, switchp->nodeInfo.NumPorts)) {
				// B=1, all Egress
				egress = switchp->nodeInfo.NumPorts;
				for (e=switchp->nodeInfo.NumPorts-1; e>0; e--) {
					if (!StlTestAndClearPortInPortMask(scsc[b].EgressPortMask, e))
						break;
					egress = e;
				}
				lastEgress = switchp->nodeInfo.NumPorts;
				egressAmod = (1 << 16) | egress; 	// Set B=1/egress

			} else {
				// B=0, numBlocks is multiple egress
				egress = StlGetFirstPortInPortMask(scsc[b].EgressPortMask);
				StlClearPortInPortMask(scsc[b].EgressPortMask, egress);
				numBlocks = 1;
				for (i=egress+1; i<=switchp->nodeInfo.NumPorts; i++) {
					if (((numBlocks+1) > STL_NUM_SCSC_BLOCKS_PER_LID_SMP) ||
						(!StlTestAndClearPortInPortMask(scsc[b].EgressPortMask, i)))
						break;
					numBlocks++;
				}

				lastEgress = egress + numBlocks - 1;
				egressAmod = (numBlocks << 24) | egress;
			}

			// program all ingress ports with egress data
			memcpy(ingressPorts, scsc[b].IngressPortMask, STL_PORT_SELECTMASK_SIZE);
			while (StlNumPortsSetInPortMask(ingressPorts, switchp->nodeInfo.NumPorts)) {
				amod = egressAmod;
				if (StlTestAndClearPortInPortMask(ingressPorts, switchp->nodeInfo.NumPorts)) {
					ingress = switchp->nodeInfo.NumPorts;

					// Check if other ports in this range
					for (i=switchp->nodeInfo.NumPorts-1; i>0; i--) {
						if (!StlTestAndClearPortInPortMask(ingressPorts, i))
							break;
						ingress = i;
					}

					lastIngress = switchp->nodeInfo.NumPorts;

					amod |= ((1 << 17) | (ingress<<8));
					if (amod & (1 << 16)) {
						// Both B=1 and A=1, set one block
						amod |= (1 << 24);
					}

				} else if (amod & (1 << 16)) {
					// B=1, send multiple ingress ports
					ingress = StlGetFirstPortInPortMask(ingressPorts);
					StlClearPortInPortMask(ingressPorts, ingress);
					numBlocks = 1;
					for (i=ingress+1; i<=switchp->nodeInfo.NumPorts; i++) {
						if (((numBlocks+1) > STL_NUM_SCSC_BLOCKS_PER_LID_SMP) ||
							(!StlTestAndClearPortInPortMask(ingressPorts, i)))
							break;
						numBlocks++;
					}

					lastIngress = ingress + numBlocks - 1;
					amod |= ((numBlocks << 24) | (ingress<<8));

				} else {
					// program single ingress port with block of egress port data
					ingress = StlGetFirstPortInPortMask(ingressPorts);
					StlClearPortInPortMask(ingressPorts, ingress);
					lastIngress = ingress;

					amod |= (ingress<<8);
				}

				status = SM_Set_SCSC_LR(fd_topology, amod, sm_lid, dlid, (STL_SCSCMAP*)&scsc[b].SCSCMap, sm_config.mkey);
				if (status != VSTATUS_OK) {
					IB_LOG_WARN_FMT(__func__, "Failed to set SCSC Map for switch %s nodeGuid " FMT_U64,
									sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
	
				} else {
					// update cached data on success
					for (i=ingress; i<=lastIngress; i++) {
						swportp = sm_get_port(switchp, i);
						if (!sm_valid_port(swportp)) continue;
						for (e=egress; e<=lastEgress; e++) {
							sm_addPortDataSCSCMap(swportp, e-1, 0, &scsc[b].SCSCMap);
						}
						swportp->portData->current.scsc = 1;
					}
				}
			}
		}
	}
	return VSTATUS_OK;
}

static Status_t
sm_initialize_Switch_SCSCMap(Topology_t * topop, Node_t * switchp)
{
	Status_t status = VSTATUS_OK;
	STL_LID	dlid;
	STL_SCSC_MULTISET *scsc=NULL;
	int	numBlocks=0;
	int s;
	int setCnt = 1;


	/* Note: If node was previously non-responding. Don't bother going any further. */
	if (switchp->nonRespCount) {
		IB_LOG_WARN_FMT(__func__, "node is unresponsive %s", sm_nodeDescString(switchp));
		IB_EXIT(__func__, 0);
		return VSTATUS_OK;
	}

	if (switchp->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		IB_EXIT(__func__, 1);
		return VSTATUS_BAD;
	}

	Port_t *swportp;
	swportp = sm_get_port(switchp, 0);
	if (!sm_valid_port(swportp)) {
		IB_LOG_WARN_FMT(__func__, "Failed to get Port 0 of Switch " FMT_U64,
						switchp->nodeInfo.NodeGUID);
		return VSTATUS_BAD;
	}
	dlid = swportp->portData->lid;


	// Second loop handles secondary SCSC tables (doesn't apply to fattree)
	for (s=0; s<setCnt; s++) {
		status = topop->routingModule->funcs.select_scsc_map(topop, switchp, s, &numBlocks, &scsc);
		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__, "Unable to setup SCSC map for switch %s nodeGuid " FMT_U64,
							sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
			return VSTATUS_BAD;
		}

		if (numBlocks == 0) {
			continue;
		}

		{
			// Use STL1 SMP format
			status = WriteGen1SCSC(topop, switchp, dlid, numBlocks, scsc);
		}
	
		(void) vs_pool_free(&sm_pool, scsc);
	}

	return status;
}

static Status_t
sm_initialize_Switch_SCVLMaps(Topology_t * topop, Node_t * switchp)
{
    uint32_t amod = 0; 
    uint8_t synchModeGen1 = 1;
    Status_t status = VSTATUS_OK; 
    Node_t *neighborNodep = NULL; 
    Port_t * out_portp,*neighborPortp, *swportp = NULL, *neighborSwPortp = NULL; 
    STL_SCVLMAP scvlmap; 
    int sentSCVLt = 0;
    int doAll = switchp->uniformVL;
    int interleaveEnabled = 0;
    uint8_t sc;

    IB_ENTER(__func__, topop, switchp, 0, 0);

    swportp = sm_get_port(switchp, 0);
    if (!sm_valid_port(swportp)) {
        IB_LOG_WARN_FMT(__func__, 
            "Failed to get Port 0 of Switch " FMT_U64,
            switchp->nodeInfo.NodeGUID);
        status = VSTATUS_BAD;
        goto fail;
    }

    // for every switch, we need to setup the SC->VL_r mapping.  This is
    // shared across ports. 
    if (swportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) {
        amod = (1 << 24) | (1 << 8) | 0;   // 1 block, all ingress & cport

        STL_SCVLMAP * curScvl = &swportp->portData->scvlrMap;

        status = topop->routingModule->funcs.select_scvlr_map(topop, switchp->vlCap, &scvlmap);
        if (status != VSTATUS_OK) {
            IB_LOG_WARNRC("sm_initialize_Switch_SCVLMaps: Failed to get SCVL "
                    "map from routing algorithm; rc:",
                    status); 
            return status;
        }

        // Compare the port's current SCVL map against what the topology says it
        // should be.  This is a switch-wide setting so just check cport.   
        // If they're different, send the new one.
        if (!swportp->portData->current.scvlr ||
            memcmp((void *)curScvl, (void *)&scvlmap, sizeof(scvlmap)) != 0) {

            // Verify that change is okay
            for (sc=0; sc<STL_MAX_SCS; sc++) {
                if (sc == 15) {
                    if (scvlmap.SCVLMap[sc].VL != 15) {
                        IB_LOG_ERROR_FMT(__func__, "SC15 mapped to VL%d, should be mapped to VL15",
                                        scvlmap.SCVLMap[sc].VL);
                        return VSTATUS_BAD;
                    }
                    continue;
                }
                if (curScvl->SCVLMap[sc].VL != scvlmap.SCVLMap[sc].VL &&
                    curScvl->SCVLMap[sc].VL != 15 &&
                    scvlmap.SCVLMap[sc].VL != 15) {
                    // changing an sc2vl mapping (not just enable/disable)

                    for_all_physical_ports(switchp, out_portp) {
                        // check to see an any active ports are using interleaving
                        if (!sm_valid_port(out_portp) || out_portp->state <= IB_PORT_DOWN) continue;
                        if ((out_portp->state > IB_PORT_INIT) &&
                            sm_IsInterleaveEnabled(&out_portp->portData->portInfo)) {
                            interleaveEnabled = 1;
                            break;
                        }
                    }
                    break;
                }
            }

            if (interleaveEnabled) {
                IB_LOG_WARN_FMT(__func__, 
                                "Mismatch/Unable to set SCVL_r Map for node %s nodeGuid " FMT_U64,
                                sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
            } else {

                status = SM_Set_SCVLrMap_LR(fd_topology, amod, sm_lid, swportp->portData->lid, &scvlmap, sm_config.mkey); 

                if (status != VSTATUS_OK) {
                    IB_LOG_WARN_FMT("sm_initialize_Switch_SCVLMaps",
                                    "Failed to set SCVL_r Map for node %s nodeGuid " FMT_U64
                                    " output port %d", sm_nodeDescString(switchp),
                                    switchp->nodeInfo.NodeGUID, swportp->index);
                    status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
                    if (status == VSTATUS_TIMEOUT_LIMIT)
                        goto fail;
                }
                swportp->portData->current.scvlr = (status == VSTATUS_OK);
                swportp->portData->scvlrMap = scvlmap;

                // set SCVL_r Map for the switch
                for_all_ports(switchp, out_portp) {
                    if (!sm_valid_port(out_portp) || out_portp->state <= IB_PORT_DOWN) 
                       continue; 
                    out_portp->portData->scvlrMap = scvlmap; 
                    out_portp->portData->current.scvlr = (status == VSTATUS_OK);
                }
            }
        }
    }

    sm_initialize_Switch_SCSCMap(topop, switchp);

    // 
    // for every node/port combo, we need to setup the SC->VL_t and SC_->VL_nt
    // mapping.  According to Volume 1, Section 20.2.2.6.14 SCtoVLxMappingTable.
    //
    if (doAll) {
        // switch supports scvl multiset, check to see if all ports need to be updated.
        for_all_physical_ports(switchp, out_portp) {
            if (!sm_valid_port(out_portp) || out_portp->state <= IB_PORT_DOWN) continue;
            if (out_portp->state > IB_PORT_INIT) {
                doAll = 0;
                break;
            }
        }
    }
    
    for_all_ports(switchp, out_portp) {
        if (!sm_valid_port(out_portp) || out_portp->state <= IB_PORT_DOWN) 
            continue; 

        if ((neighborNodep = sm_find_node(topop, out_portp->nodeno)) == NULL || 
            (neighborPortp = sm_find_node_port(topop, neighborNodep, out_portp->portno)) == NULL || 
            !sm_valid_port(neighborPortp)) {
            IB_LOG_WARN_FMT(__func__, 
                            "Unable to get neighbor to node %s nodeGuid " FMT_U64
                            " output port %d", sm_nodeDescString(switchp), 
                            switchp->nodeInfo.NodeGUID, out_portp->index); 
            status = VSTATUS_BAD;
            goto fail;
        }

        if (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
            neighborSwPortp = sm_get_port(neighborNodep, 0);
            if (!sm_valid_port(neighborSwPortp)) {
                IB_LOG_WARN_FMT(__func__, 
                    "Failed to get Port 0 of Switch " FMT_U64,
                    neighborNodep->nodeInfo.NodeGUID);
                status = VSTATUS_BAD;
                goto fail;
            }
        }

        if (!out_portp->portData->current.scvlt) {
            IB_LOG_WARN_FMT(__func__,
                "SCVLt for node %s nodeGuid "FMT_U64" port %d stale, will attempt update",
                sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID, out_portp->index);
        }

        if (out_portp->index > 0 &&
            !neighborPortp->portData->current.scvlnt) {
            IB_LOG_WARN_FMT(__func__,
                "SCVLnt for node %s nodeGuid "FMT_U64" port %d stale, will attempt update",
                sm_nodeDescString(neighborNodep), neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
        }

        // the SCtoVL_nt table must be configured consistently with the SCtoVL_t table at its neighbor. When the
        // link state is Init the SM shall have the responsibility of updating both the SCtoVL_t table and the
        // neighbor's SCtoVL_nt table ("synchronous" update). When the link state is Armed or Active, the SM
        // shall update the SCtoVL_t table; and the SMAs of both ports shall have the responsibility of updating
        // the neighbor's SCtoVL_nt table ("asynchronous" update of SCtoVL_t only).        
        //
        // section 9.7.14.3 of the STL spec, "Optional Mechanism for Changes While in LinkArmed or LinkActive"
        // The mechanisms in this section are only available when the ports on both side of a link report
        // IsAsyncSC2VLSupported. If either port reports it does not have this capability, the FM shall not
        // attempt to perform the changes outlined in this section.
        // It is anticipated that STL Gen1 will not support this capability.
        // Switch port 0 is a special case (no neighbor and SCtoVL_nt not supported), so SCtoVL_t can be changed at any time.
        synchModeGen1 = 1;
        if ((out_portp->state > IB_PORT_INIT ||
            neighborPortp->state > IB_PORT_INIT) &&
            out_portp->index != 0) {
            // when asynchronous mode is supported in Gen2 additional checks should
            // be done against the IsAsyncSC2VLSupported field. 
            synchModeGen1 = 0;
        }

        // initialize the SC2VL_t map of the switch port

        STL_SCVLMAP * curScvl = &out_portp->portData->scvltMap;

        if (!doAll || !sentSCVLt) {
            status = topop->routingModule->funcs.select_scvl_map(topop, switchp, out_portp, neighborPortp, &scvlmap);
            if (status != VSTATUS_OK) {
                IB_LOG_WARNRC("Failed to get SCVL "
                            "map from routing algorithm; rc:", status);
                goto fail;
            }
        }

        // compare the port's current SCVL map against what the topology says it
        // should be. If they're different, send the new one.
        if (!out_portp->portData->current.scvlt ||
            memcmp((void *)curScvl, (void *)&scvlmap, sizeof(scvlmap)) != 0) {
            if (synchModeGen1) {
                if (!doAll || !sentSCVLt) {
                    amod = ((out_portp->state == IB_PORT_INIT) || (out_portp->index == 0)) ?
                                1 << 24 : (1 << 24) | 1 << 12;   // 1 block, synch/asynch respectively
                    amod |= (uint32_t)out_portp->index;
                    if (doAll && (out_portp->index > 0))
                        amod |= (1 << 8);

                    status = SM_Set_SCVLtMap_LR(fd_topology, amod, sm_lid, swportp->portData->lid, &scvlmap, sm_config.mkey); 

                    out_portp->portData->current.scvlt = (status == VSTATUS_OK);
                    if (status != VSTATUS_OK) {
                        IB_LOG_WARN_FMT(__func__, 
                                    "Failed to set SCVL_t Map for node %s nodeGuid " FMT_U64
                                    " output port %d", sm_nodeDescString(switchp), 
                                    switchp->nodeInfo.NodeGUID, out_portp->index);
                        status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
                        goto fail;
                    }
                    if (out_portp->index > 0)
                        sentSCVLt = 1;
                } else {
                    out_portp->portData->current.scvlt = (status == VSTATUS_OK);
                }
            } else {
                IB_LOG_WARN_FMT(__func__, 
                                "Mismatch/Unable to set SCVL_t Map for node %s nodeGuid " FMT_U64
                                " output port %d", sm_nodeDescString(switchp), 
                                switchp->nodeInfo.NodeGUID, out_portp->index);
            }
        }
        
        // set SCVL_t Map for the port
        out_portp->portData->scvltMap = scvlmap; 

        //
        // initialize the SCVL_nt map of the neighbor port.  When the link state is Armed or Active, the
        // SMAs of both ports shall have the responsibility of updating the neighbor's SCtoVL_nt table
        // ("asynchronous" update of SCtoVL_t only).
        amod = (1 << 24) | neighborPortp->index;   // 1 block, sych update

        STL_SCVLMAP * curScvlnt = &neighborPortp->portData->scvlntMap;

        // SCVLnt not supported on switch port 0
        if (out_portp->index > 0 &&
            (!neighborPortp->portData->current.scvlnt ||
            memcmp((void *)curScvlnt, (void *)&scvlmap, sizeof(scvlmap)) != 0)) {
            if (synchModeGen1) {
                status = SM_Set_SCVLntMap_LR(fd_topology,
                                             amod,
                                             sm_lid, 
                                             (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? neighborSwPortp->portData->lid : neighborPortp->portData->lid,
                                             &scvlmap,
                                             sm_config.mkey); 
                
                neighborPortp->portData->current.scvlnt = (status == VSTATUS_OK);
                if (status != VSTATUS_OK) {
                    IB_LOG_WARN_FMT(__func__, 
                                    "Failed to set SCVL_nt Map for node %s nodeGuid " FMT_U64
                                    " output port %d", sm_nodeDescString(neighborNodep), 
                                    neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
                    status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
                    goto fail;
                }
            } else {
                IB_LOG_WARN_FMT(__func__, 
                                "Neighbor Mismatch/Unable to set SCVL_nt Map for node %s nodeGuid " FMT_U64
                                " output port %d", sm_nodeDescString(neighborNodep), 
                                neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
            }
        }
        
        // set SCVL_nt Map for the neighbor port
        neighborPortp->portData->scvlntMap = scvlmap;
    }

fail:
    IB_EXIT(__func__, status);
    return (status);
}

static Status_t
sm_initialize_Node_Port_SLSCMap(Topology_t * topop, Node_t * nodep, Port_t * out_portp, STL_SLSCMAP * slscmapp)
{ 
    Status_t status = VSTATUS_OK; 
    
    IB_ENTER(__func__, topop, nodep, out_portp, 0); 
    
    // 
    // for every node/port combo, we need to setup the SL->SC mapping.
    // According to Volume 1, Section 20.2.2.6.10 SLtoSCMappingTable.
    
    // 
    // if this port is DOWN, then we do nothing.
    // 
    if (out_portp->state <= IB_PORT_DOWN) {
        IB_EXIT(__func__, 1); 
        return (VSTATUS_OK);
    }
    
    // note: if node was previously non-responding. Don't bother going any further.
    if (nodep->nonRespCount) {
        return (VSTATUS_OK);
    }

    if (!out_portp->portData->current.slsc) {
        IB_LOG_WARN_FMT(__func__,
            "SCSL for node %s nodeGuid "FMT_U64" port %d is not current",
            sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, out_portp->index);
    }
    STL_SLSCMAP * curSlsc = &out_portp->portData->slscMap;

    status = topop->routingModule->funcs.select_slsc_map(topop, nodep, out_portp, out_portp, slscmapp); 
    if (status != VSTATUS_OK) {
        IB_LOG_WARNRC("Failed to get SLSC "
                      "map from routing algorithm; rc:", 
                      status);
        return status;
    }
    
    // 
    // Compare the port's current SLSC map against what the topology says it
    // should be. If they're different, send the new one.
    // 
    if (!out_portp->portData->current.slsc ||
        memcmp((void *)curSlsc, (void *)slscmapp, sizeof(*slscmapp)) != 0 || sm_config.forceAttributeRewrite) {
#if DO_INLINE_SET
        status = SM_Set_SLSCMap_LR(fd_topology, amod, sm_lid, out_portp->portData->lid, slscmapp, sm_config.mkey); 
        
        if (status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                            "Failed to set SLSC Map for node %s nodeGuid " FMT_U64
                            " output port %d", sm_nodeDescString(nodep), 
                            nodep->nodeInfo.NodeGUID, out_portp->index); 
            nodep->slscChange = 1;
        }
        out_portp->portData->current.slsc = (status == VSTATUS_OK);
#else
        out_portp->portData->dirty.slsc = 1;
        sm_markSmaChanged(topop, nodep);
#endif
    }

#if DO_INLINE_SET
    // Set SLSC Map for the port
    out_portp->portData->slscMap = *slscmapp; 
#endif
    
    IB_EXIT(__func__, 0); 
    return (status);
}

static Status_t
sm_initialize_Node_Port_SCSLMap(Topology_t * topop, Node_t * nodep, Port_t * in_portp, STL_SCSLMAP * scslmapp)
{ 
//  uint8_t *path;
    Status_t status = VSTATUS_OK; 
    
    IB_ENTER(__func__, topop, nodep, in_portp, 0); 
    
//  path = PathToPort(nodep, in_portp);
    
    // 
    // for every node/port combo, we need to setup the SC->SL mapping.
    // According to Volume 1, Section 20.2.2.6.13 SCtoSLMappingTable.
    
    if (in_portp->state <= IB_PORT_DOWN) {
        IB_EXIT(__func__, 1); 
        return (VSTATUS_OK);
    } else if (nodep->nodeInfo.NodeType != NI_TYPE_CA) {
        
        IB_EXIT(__func__, 2); 
        return (VSTATUS_OK);
    }
    
    // 
    // set up the SC->SL mapping just for this port.
    
    // note: if node was previously non-responding. Don't bother going any further.
    if (nodep->nonRespCount) {
        return (VSTATUS_OK);
    }

    if (!in_portp->portData->current.scsl) {
        IB_LOG_WARN_FMT(__func__,
          "SCSL for node %s nodeGuid "FMT_U64" port %d is not current",
          sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, in_portp->index);
    }
    STL_SCSLMAP * curScsl = &in_portp->portData->scslMap;

    status = topop->routingModule->funcs.select_scsl_map(topop, nodep, in_portp, in_portp, scslmapp); 
    if (status != VSTATUS_OK) {
        IB_LOG_WARNRC
           ("sm_initialize_Node_Port_SCSLMap: Failed to get SCSL "
            "map from routing algorithm; rc:", 
            status);
        return status;
    }

    // 
    // compare the port's current SCSL map against what the topology says it
    // should be. If they're different, send the new one.
    if (!in_portp->portData->current.scsl ||
        memcmp((void *)curScsl, (void *)scslmapp, sizeof(*scslmapp)) != 0 || sm_config.forceAttributeRewrite) {
#if DO_INLINE_SET
        status = SM_Set_SCSLMap_LR(fd_topology, amod, sm_lid, in_portp->portData->lid, scslmapp, sm_config.mkey); 
            
        if (status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                            "Failed to set SCSL Map for node %s nodeGuid " FMT_U64
                            " output port %d", sm_nodeDescString(nodep), 
                            nodep->nodeInfo.NodeGUID, in_portp->index);
        }
        in_portp->portData->current.scsl = (status == VSTATUS_OK);
#else
        in_portp->portData->dirty.scsl = 1;
        sm_markSmaChanged(topop, nodep);
#endif
    }

#if DO_INLINE_SET
    // set SCSL Map for the port
    in_portp->portData->scslMap = *scslmapp; 
#endif
    
    IB_EXIT(__func__, 0); 
    return (status);
}

static Status_t
sm_initialize_Node_Port_SCVLMaps(Topology_t * topop, Node_t * nodep, Port_t * in_portp)
{ 
//  uint8_t * path,*neighborPath;
    uint32_t amod = 0; 
    uint8_t synchModeGen1 = 1;
    Status_t status = VSTATUS_OK; 
    Node_t *neighborNodep; 
    Port_t * neighborPortp,*swportp = NULL; 
    STL_SCVLMAP scvlmap;
    STL_SCVLMAP * curScvlt, * curScvlnt;

    IB_ENTER(__func__, topop, nodep, in_portp, 0); 

//  path = PathToPort(nodep, in_portp);

    // 
    // for every node/port combo, we need to setup the SC->VL_t and SC_->VL_nt
    // mapping.  According to Volume 1, Section 20.2.2.6.14 SCtoVLxMappingTable.
    
    if (in_portp->state <= IB_PORT_DOWN) {
        IB_EXIT(__func__, 1); 
        return (VSTATUS_OK);
    } else if (nodep->nodeInfo.NodeType != NI_TYPE_CA) {
        IB_EXIT(__func__, 2); 
        return (VSTATUS_OK);
    }
    
    // 
    // set up the SC->VL mappings just for this port and neighbor port.
    
    // note: if node was previously non-responding. Don't bother going any further.
    if (nodep->nonRespCount) 
        return (VSTATUS_OK); 
    
    if ((neighborNodep = sm_find_node(topop, in_portp->nodeno)) == NULL || 
        (neighborPortp = sm_find_node_port(topop, neighborNodep, in_portp->portno)) == NULL || 
        !sm_valid_port(neighborPortp)) {
        IB_LOG_WARN_FMT(__func__, 
                        "Unable to get neighbor to node %s nodeGuid " FMT_U64
                        " output port %d", sm_nodeDescString(nodep), 
                        nodep->nodeInfo.NodeGUID, in_portp->index); 
        return (VSTATUS_BAD);
    }
    
    if (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        swportp = sm_get_port(neighborNodep, 0); 
        if (!sm_valid_port(swportp)) {
            IB_LOG_WARN_FMT(__func__, 
                            "Failed to get Port 0 of Switch " FMT_U64, 
                            neighborNodep->nodeInfo.NodeGUID); 
            return VSTATUS_BAD;
        }
    }

    if (in_portp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) {
        amod = (1 << 24) | in_portp->index;

        STL_SCVLMAP * curScvl = &in_portp->portData->scvlrMap;

        status = topop->routingModule->funcs.select_scvlr_map(topop, in_portp->portData->vl0, &scvlmap);
        if (status != VSTATUS_OK) {
            IB_LOG_WARNRC("sm_initialize_Switch_SCVLMaps: Failed to get SCVL "
                    "map from routing algorithm; rc:",
                    status); 
            return status;
        }

        // Compare the port's current SCVL map against what the topology says it
        // should be.  If they're different, send the new one.
        if (!in_portp->portData->current.scvlr ||
            memcmp((void *)curScvl, (void *)&scvlmap, sizeof(scvlmap)) != 0) {
            status = SM_Set_SCVLrMap_LR(fd_topology, amod, sm_lid, in_portp->portData->lid, &scvlmap, sm_config.mkey); 

            if (status != VSTATUS_OK) {
               IB_LOG_WARN_FMT("sm_initialize_Switch_SCVLMaps", 
                               "Failed to set SCVL_r Map for node %s nodeGuid " FMT_U64
                               " port %d", sm_nodeDescString(nodep), 
                               nodep->nodeInfo.NodeGUID, in_portp->index);
            }
            in_portp->portData->current.scvlr = (status == VSTATUS_OK);
            in_portp->portData->scvlrMap = scvlmap; 
        }
    }

    if (!in_portp->portData->current.scvlt) {
        IB_LOG_WARN_FMT(__func__,
            "SCVLt for node %s nodeGuid "FMT_U64" port %d stale, will attempt update",
            sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, in_portp->index);
    }

    if (!neighborPortp->portData->current.scvlnt) {
        IB_LOG_WARN_FMT(__func__,
            "SCVLnt for node %s nodeGuid "FMT_U64" port %d stale, will attempt update",
            sm_nodeDescString(neighborNodep), neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
    }

    curScvlt = &in_portp->portData->scvltMap;
    curScvlnt = &neighborPortp->portData->scvlntMap;

//  neighborPath = PathToPort(neighborNodep, neighborPortp);
    //    
    // the SCtoVL_nt table must be configured consistently with the SCtoVL_t table at its neighbor. When the
    // link state is Init the SM shall have the responsibility of updating both the SCtoVL_t table and the
    // neighbor's SCtoVL_nt table ("synchronous" update). When the link state is Armed or Active, the SM
    // shall update the SCtoVL_t table; and the SMAs of both ports shall have the responsibility of updating
    // the neighbor's SCtoVL_nt table ("asynchronous" update of SCtoVL_t only).
    //
    // section 9.7.14.3 of the STL spec, "Optional Mechanism for Changes While in LinkArmed or LinkActive"
    // The mechanisms in this section are only available when the ports on both side of a link report
    // IsAsyncSC2VLSupported. If either port reports it does not have this capability, the FM shall not
    // attempt to perform the changes outlined in this section.
    // It is anticipated that STL Gen1 will not support this capability.
    if (in_portp->state > IB_PORT_INIT || neighborPortp->state > IB_PORT_INIT) {
        // when asynchronous mode is supported in Gen2 additional checks should
        // be done against the IsAsyncSC2VLSupported field. 
        synchModeGen1 = 0;
    }

    //
    // get SCVL_t map of the port
    amod = (in_portp->state == IB_PORT_INIT) ? 1 << 24 : (1 << 24) | 1 << 12;   // 1 block, synch/asynch respectively
    amod |= (uint32_t)in_portp->index;

    status = topop->routingModule->funcs.select_scvl_map(topop, nodep, in_portp, in_portp, &scvlmap); 
    if (status != VSTATUS_OK) {
        IB_LOG_WARNRC("Failed to get SCVL "
                      "map from routing algorithm; using default; rc:", 
                      status);
    }

    // 
    // compare the port's current SCVL map against the computed SCVLt map.
    // If they're different, send the new one.
    if (!in_portp->portData->current.scvlt ||
        memcmp((void *)curScvlt, (void *)&scvlmap, sizeof(scvlmap)) != 0) {
        if (synchModeGen1) {
            status = SM_Set_SCVLtMap_LR(fd_topology, amod, sm_lid, in_portp->portData->lid, &scvlmap, sm_config.mkey); 

            in_portp->portData->current.scvlt = (status == VSTATUS_OK);
            if (status != VSTATUS_OK) {
                IB_LOG_WARN_FMT(__func__, 
                                "Failed to set SCVL_t Map for node %s nodeGuid " FMT_U64
                                " output port %d", sm_nodeDescString(nodep), 
                                nodep->nodeInfo.NodeGUID, in_portp->index);
                return status;
            }
        } else {
            IB_LOG_WARN_FMT(__func__, 
                            "Mismatch/Unable to set SCVL_t Map for node %s nodeGuid " FMT_U64
                            " output port %d", sm_nodeDescString(nodep), 
                            nodep->nodeInfo.NodeGUID, in_portp->index);
        }
    }

    // set SCVL_t Map for the port
    in_portp->portData->scvltMap = scvlmap;
    //
    // set SCVL_nt map of the neighbor port
    amod = (1 << 24) | neighborPortp->index; // 1 block, port

    if (!neighborPortp->portData->current.scvlnt ||
        memcmp((void *)curScvlnt, (void *)&scvlmap, sizeof(scvlmap)) != 0) {
        if (synchModeGen1) {
            status = SM_Set_SCVLntMap_LR(fd_topology, amod,
                                         sm_lid, 
                                         (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? swportp->portData->lid : neighborPortp->portData->lid, 
                                         &scvlmap, sm_config.mkey); 

            neighborPortp->portData->current.scvlnt = (status == VSTATUS_OK);
            if (status != VSTATUS_OK) {
                IB_LOG_WARN_FMT(__func__, 
                                "Failed to set SCVL_nt Map for neighbor node %s nodeGuid " FMT_U64
                                " output port %d", sm_nodeDescString(neighborNodep), 
                                neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
                return status;
            }
        } else {
            IB_LOG_WARN_FMT(__func__, 
                            "Mismatch/Unable to set SCVL_nt Map for neighbor node %s nodeGuid " FMT_U64
                            " output port %d", sm_nodeDescString(neighborNodep), 
                            neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
        }
    }

    // set SCVL_nt Map for the neighbor port
    neighborPortp->portData->scvlntMap = scvlmap;
    
    IB_EXIT(__func__, 0); 
    return (status);
}

Status_t
sm_initialize_Node_SLMaps(Topology_t * topop, Node_t * nodep, Port_t * out_portp)
{ 
    Status_t status; 

    STL_SCSLMAP scslmap;
    STL_SLSCMAP slscmap;

    status = sm_port_init_changes(out_portp);
    if (status != VSTATUS_OK)
      return status;

    out_portp->portData->changes.slsc = &slscmap;
    out_portp->portData->changes.scsl = &scslmap;

    {
        // initialize the SL2SC mapping table for the egress port
        status = sm_initialize_Node_Port_SLSCMap(sm_topop, nodep, out_portp,
            out_portp->portData->changes.slsc); 
        if (status != VSTATUS_OK)
            return status;
 
        // initialize the SC2SL mapping table for the egress port
        status = sm_initialize_Node_Port_SCSLMap(sm_topop, nodep, out_portp,
            out_portp->portData->changes.scsl);
        if (status != VSTATUS_OK)
            return status;
    }

    // initialize the SC2VL* mapping tables for the egress port
    status = sm_initialize_Node_Port_SCVLMaps(sm_topop, nodep, out_portp); 
    if(status != VSTATUS_OK)
        return status;

    Node_t * lastNode = NULL;
    status = sm_syncSmaChanges(topop, &lastNode);

    if (status != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__,
            "Error on synching SMA updates; status : 0x%02x; last-node processed NodeGUID : "FMT_U64,
            status, ( lastNode ? lastNode->nodeInfo.NodeGUID : 0));
    }

    return status;
}

Status_t
sm_initialize_Switch_SLMaps(Topology_t * topop, Node_t * nodep)
{
    IB_ENTER(__func__, topop, nodep, 0, 0);
    Status_t status;
    Port_t * swportp = sm_get_port(nodep, 0);

    STL_SCSLMAP scslmap;
    STL_SLSCMAP slscmap;

    if (!sm_valid_port(swportp)) {
        return VSTATUS_BAD;
    }

    status = sm_port_init_changes(swportp);
    if (status != VSTATUS_OK)
        return status;

    swportp->portData->changes.slsc = &slscmap;
    swportp->portData->changes.scsl = &scslmap;

    {
        // initialize the SL2SC mapping table for all ports.
        status = sm_initialize_Switch_SLSCMap(sm_topop, nodep,
            swportp->portData->changes.slsc);
        if (status != VSTATUS_OK)
            return status;

        // initialize the SC2SL mapping table for all ports.
        status = sm_initialize_Switch_SCSLMap(sm_topop, nodep,
            swportp->portData->changes.scsl);
        if (status != VSTATUS_OK)
            return status;
    }

    // initialize the SC2VL* mapping tables for all ports.
    status = sm_initialize_Switch_SCVLMaps(sm_topop, nodep);
    if(status != VSTATUS_OK)
        return status;

    Node_t * lastNode = NULL;
    status = sm_syncSmaChanges(topop, &lastNode);

    if (status != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__,
            "Error on synching SMA updates; status : 0x%02x; last-node processed NodeGUID : "FMT_U64,
            status, ( lastNode ? lastNode->nodeInfo.NodeGUID : 0));
    }

    IB_EXIT(__func__, status);
    return status;
}

/**
	Update the SMA of @c nodep with all changes stored on @c nodep.
*/
static Status_t
sm_node_syncSmaChanges(Topology_t * topop, Node_t * nodep, Port_t * smaportp)
{
	Status_t s;

	if (nodep->aggregateEnable) {
		s = sm_node_syncSma_aggregate(topop, nodep, smaportp);
	}

	if (!nodep->aggregateEnable || s != VSTATUS_OK) {
		s = sm_node_syncSma_solo(topop, nodep, smaportp);

		if (nodep->aggregateEnable && s == VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				"Disabling aggregate support on node %s nodeGUID "FMT_U64,
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
				nodep->aggregateEnable = 0;
		}
	}
	return s;
}

static Status_t
sm_node_syncSma_aggregate(Topology_t * topop, Node_t * nodep, Port_t * smaportp)
{
	Status_t s = VSTATUS_OK;
	STL_AGGREGATE * aggrBuffer = NULL;

	STL_LID destLid;

	if (!sm_valid_port(smaportp))
		return VSTATUS_BAD;

	destLid = smaportp->portData->portInfo.LID;

	const size_t reqMem =
		smaportp->portData->dirty.scsl * (sizeof(STL_AGGREGATE) + sizeof(STL_SCSLMAP)) +
		smaportp->portData->dirty.slsc * (sizeof(STL_AGGREGATE) + sizeof(STL_SLSCMAP));

	if (reqMem == 0)
		return VSTATUS_OK;

  vs_pool_alloc(&sm_pool, reqMem, (void*)&aggrBuffer);
  if (!aggrBuffer)
    return VSTATUS_BAD;

	STL_AGGREGATE * aggrHdr = aggrBuffer;

	if (smaportp->portData->dirty.scsl) {
		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_SLSCMAP) + 7)/8;
		aggrHdr->AttributeModifier = 0;

		memcpy(aggrHdr->Data, smaportp->portData->changes.scsl, sizeof(STL_SCSLMAP));
		BSWAP_STL_SCSLMAP((STL_SCSLMAP *)aggrHdr->Data);

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);
	}

	if (smaportp->portData->dirty.slsc) {
		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_SLSCMAP) + 7)/8;

		aggrHdr->AttributeModifier = 0;

		memcpy(aggrHdr->Data, smaportp->portData->changes.slsc, sizeof(STL_SLSCMAP));
		BSWAP_STL_SLSCMAP((STL_SLSCMAP *)aggrHdr->Data);
		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);
	}

	uint32_t madStatus = 0;
	STL_AGGREGATE * lastSeg = NULL;

	s = SM_Set_Aggregate_LR(fd_topology, aggrBuffer, aggrHdr,
		sm_lid, destLid, sm_config.mkey, &lastSeg, &madStatus);

	if (!lastSeg && s == VSTATUS_OK) {
		s = VSTATUS_BAD;
		goto fail; 
	}

	if (lastSeg) {
		// Can still process partial aggregate response on MAD error
		Status_t tmpStatus = sm_node_handleGetRespAggr(nodep, smaportp, aggrBuffer, lastSeg);
		s = (s != VSTATUS_OK? s : tmpStatus);
	}

	if (s != VSTATUS_OK || madStatus != MAD_STATUS_SUCCESS) {
		IB_LOG_ERROR_FMT(__func__,
			"Error on Set(Aggregate): NodeGUID : "FMT_U64"; NodeDesc : \"%s\"; status : %d; madStatus : 0x%02x",
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), s, madStatus);
		if (madStatus != MAD_STATUS_SUCCESS && lastSeg != NULL) {
			IB_LOG_ERROR_FMT(__func__,
				"First error: AttributeID : 0x%02x; AttributeModifier : 0x%08x",
				lastSeg->AttributeID, lastSeg->AttributeModifier);
		}

		//@todo: do dirty.slsc & dirty.scsl make slscChange redundant?
		nodep->slscChange = smaportp->portData->dirty.slsc || smaportp->portData->dirty.scsl;
		goto fail;
	}

	boolean dirty = smaportp->portData->dirty.slsc || smaportp->portData->dirty.scsl;

	if (!dirty) {
		sm_clearSmaChanged(topop, nodep);
	}

fail:
	vs_pool_free(&sm_pool, aggrBuffer);

	return s;
}

static Status_t
sm_node_syncSma_solo(Topology_t * topop, Node_t * nodep, Port_t * smaportp)
{
	Status_t status = VSTATUS_OK;

	STL_LID destLid;

	if (!sm_valid_port(smaportp))
		return VSTATUS_BAD;

	destLid = smaportp->portData->lid;

	if (smaportp->portData->dirty.slsc) {
		uint32_t amod = 0;
		STL_SLSCMAP slsc = *smaportp->portData->changes.slsc;

		status = SM_Set_SLSCMap_LR(fd_topology, amod, sm_lid, destLid, &slsc, sm_config.mkey); 
		
		if (status != VSTATUS_OK) {
			nodep->slscChange = 1;
			goto fail;
		}

		smaportp->portData->slscMap = slsc;
		smaportp->portData->dirty.slsc = 0;
	}

	if (smaportp->portData->dirty.scsl) {
		uint32_t amod = 0;
		STL_SCSLMAP scsl = *smaportp->portData->changes.scsl;
		status = SM_Set_SCSLMap_LR(fd_topology, amod, sm_lid, destLid, &scsl, sm_config.mkey);

		if (status != VSTATUS_OK) {
			nodep->slscChange = 1;
			goto fail;
		}

		smaportp->portData->scslMap = scsl;
		smaportp->portData->dirty.scsl = 0;
	}

	boolean clean = !(smaportp->portData->dirty.slsc || smaportp->portData->dirty.scsl);

	if (clean) {
		sm_clearSmaChanged(topop, nodep);
	}

fail:
	return status;
}

static Status_t
sm_syncSmaChanges(Topology_t * topop, Node_t ** firstError)
{
	Status_t s = VSTATUS_OK;
	Status_t retStat = VSTATUS_OK;
	int nodeIdx;

	if (!topop->smaChanges)
		return VSTATUS_OK;

	nodeIdx = bitset_find_first_one(topop->smaChanges);
	while (nodeIdx >= 0) {
		if (nodeIdx > topop->num_nodes) {
			IB_LOG_ERROR_FMT(__func__,
				"nodeIdx > num_nodes; nodeIdx : %d; num_nodes : %d",
				nodeIdx, topop->num_nodes);
				return VSTATUS_BAD;
		}

		Node_t * nodep = sm_find_node(topop, nodeIdx);

		if (nodep) {
			Port_t * smaportp = NULL;

			for_all_sma_ports(nodep, smaportp) {
				if (!sm_valid_port(smaportp) || smaportp->state < IB_PORT_INIT)
					continue;

				s = sm_node_syncSmaChanges(topop, nodep, smaportp);

				if (s != VSTATUS_OK) {
					s = sm_popo_port_error(&sm_popo, topop, smaportp, s);
					if (retStat == VSTATUS_OK) {
						if (firstError)
							*firstError = nodep;
						retStat = s;
					}

					IB_LOG_WARN_FMT(__func__,
							"Failed to sync changes for node %s, node GUID "FMT_U64", port %d",
							sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index);

					if (s == VSTATUS_TIMEOUT_LIMIT)
						return s;
				}
			}

			// dirty and change values are not currently copied sweep-to sweep, so free to
			// release them whether or not Set() was successful
			sm_node_release_changes(nodep);
		}

		nodeIdx = bitset_find_next_one(topop->smaChanges, nodeIdx + 1);
	}

	return retStat;
}

static Status_t
sm_node_handleGetRespAggr(Node_t * nodep, Port_t * smaportp, STL_AGGREGATE * aggr, STL_AGGREGATE * end)
{
	Status_t s = VSTATUS_OK;
	for ( ; aggr < end; aggr = STL_AGGREGATE_NEXT(aggr)) {
		switch (aggr->AttributeID) {
			//@todo: any way to combine this code with the code in SM_Set_<T>*()?
			case STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE:
				smaportp->portData->dirty.slsc &= aggr->Result.s.Error;
				if (!aggr->Result.s.Error) {
					if ((aggr->Result.s.RequestLength * 8) < sizeof(STL_SLSCMAP)) {
						s = VSTATUS_BAD;
						break;
					}

					memcpy(&smaportp->portData->slscMap, aggr->Data, sizeof(STL_SLSCMAP));
					ZERO_RSVD_STL_SLSCMAP(&smaportp->portData->slscMap);
					smaportp->portData->current.slsc = 1;
				}
				break;
			case STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE:
				smaportp->portData->dirty.scsl &= aggr->Result.s.Error;
				if (!aggr->Result.s.Error) {
					if ((aggr->Result.s.RequestLength * 8) < sizeof(STL_SCSLMAP)) {
						s = VSTATUS_BAD;
						break;
					}

					memcpy(&smaportp->portData->scslMap, aggr->Data, sizeof(STL_SCSLMAP));
					BSWAP_STL_SCSLMAP(&smaportp->portData->scslMap);
					smaportp->portData->current.scsl = 1;
				}
				break;
			case STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE:
			case STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE:
			case STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE:
				s = sm_aggregateToScvl(nodep, aggr);
				break;
			case STL_MCLASS_ATTRIB_ID_VL_ARBITRATION:
				s = sm_aggregateToVlarb(nodep, aggr);
				break;
			case STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE:
				s = sm_aggregateToBfrctrl(nodep, aggr);
				break;
			default:
				return VSTATUS_BAD;
		}

		if (s != VSTATUS_OK)
			break;
	}

	return s;
}

static Status_t
sm_aggregateToScvl(Node_t * nodep, STL_AGGREGATE * aggr)
{
	// Note: a GetResp() can be caused by either a Get() or a Set()
	//
	//
	//@todo: any way to combine this with code from sm_Get_SCVL*
	if (!aggr->Result.s.Error) {
		uint8 blkCount, startPort;
		uint16 endPort; // [startPort, endPort)
		boolean allPorts = (aggr->AttributeModifier >> 8) & 0x1;

		// The block count of the response is constrained by the size of the segment
		blkCount = (uint8)(aggr->AttributeModifier >> 24);

		startPort = (uint8) aggr->AttributeModifier;
		if (allPorts)
			endPort = nodep->nodeInfo.NumPorts + 1;
		else {
			if ((uint16)startPort + blkCount > nodep->nodeInfo.NumPorts + 1) {
				IB_LOG_ERROR_FMT(__func__,
					"Computed end port exceeds number of ports: computed end port : %d; number of ports : %d",
					(uint16)startPort + blkCount, nodep->nodeInfo.NumPorts);
				return VSTATUS_BAD;
			}
			endPort = startPort + blkCount;
		}

		if (blkCount == 0) {
			IB_LOG_ERROR_FMT(__func__, "Amod block count cannot be zero");
			return VSTATUS_BAD;
		}

		size_t actBlkCap = (aggr->Result.s.RequestLength * 8)/sizeof(STL_SCVLMAP);
		if (blkCount > actBlkCap) {
			IB_LOG_ERROR_FMT(__func__,
				"Amod block count exceeds payload block capacity.  Amod block count : %d; payload block capacity : %"PRISZT,
				blkCount, actBlkCap);
			return VSTATUS_BAD;
		}

		// swap & zero data in advance
		uint8 i, j;
		for (i = 0; i < blkCount; ++i) {
			BSWAP_STL_SCVLMAP(&((STL_SCVLMAP*)aggr->Data)[i]);
		}

		/*
			Following code requires that all ports in a multiblock range were valid when the request was composed,
			otherwise the requester shouldn't compose a multiblock request that includes ports that weren't valid.
		*/
		Port_t * portp = NULL;
		for (i = startPort, j = 0; i < endPort; ++i, j = (j + 1) % blkCount) {
			portp = sm_get_port(nodep, i);

			if (!sm_valid_port(portp) || (portp->state <=IB_PORT_DOWN))
				continue;

			switch (aggr->AttributeID) {
				case STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE:
					memcpy(&portp->portData->scvltMap, &((STL_SCVLMAP*)aggr->Data)[j], sizeof(STL_SCVLMAP));
					portp->portData->current.scvlt = 1;

					break;
				case STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE:
					memcpy(&portp->portData->scvlntMap, &((STL_SCVLMAP*)aggr->Data)[j], sizeof(STL_SCVLMAP));
					portp->portData->current.scvlnt = 1;

					break;
				case STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE:
					memcpy(&portp->portData->scvlrMap, &((STL_SCVLMAP*)aggr->Data)[j], sizeof(STL_SCVLMAP));
					portp->portData->current.scvlr = 1;

					break;
			}
		}
	}

	return VSTATUS_OK;
}

static Status_t
sm_aggregateToVlarb(Node_t * nodep, STL_AGGREGATE * aggr)
{
	// It's ok to have an error, just means that nothing will get updated
	if (aggr->Result.s.Error)
		return VSTATUS_OK;

	uint32_t amod = aggr->AttributeModifier;
	uint8_t blkCount, section, startPort;
	blkCount = (uint8_t)(amod >> 24);
	section = (uint8_t)(amod >> 16);
	startPort = (uint8_t)amod;

	size_t respSize = aggr->Result.s.RequestLength * 8;

	if ((blkCount * sizeof(STL_VLARB_TABLE)) > respSize)
		return VSTATUS_BAD;

	STL_VLARB_TABLE * resp = (STL_VLARB_TABLE*)aggr->Data;

	uint8_t i;
	for (i = 0; i < blkCount; ++i) {
		Port_t * portp = sm_get_port(nodep, i + startPort);

		if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
			continue;

		struct _PortDataVLArb * arbp = &portp->portData->curArb;
		uint8_t * dest = NULL;
		size_t cpySize = 0;
		switch (section) {
			case STL_VLARB_LOW_ELEMENTS:
				dest = (uint8_t*)arbp->u.vlarb.vlarbLow;
				cpySize = sizeof(arbp->u.vlarb.vlarbLow);
				break;
			case STL_VLARB_HIGH_ELEMENTS:
				dest = (uint8_t*)arbp->u.vlarb.vlarbHigh;
				cpySize = sizeof(arbp->u.vlarb.vlarbHigh);
				break;
			case STL_VLARB_PREEMPT_ELEMENTS:
				dest = (uint8_t*)arbp->u.vlarb.vlarbPre;
				cpySize = sizeof(arbp->u.vlarb.vlarbPre);
				break;
			case STL_VLARB_PREEMPT_MATRIX:
				dest = (uint8_t*)arbp->vlarbMatrix;
				cpySize = sizeof(arbp->vlarbMatrix);
				break;
			default:
				return VSTATUS_BAD;
		}

		BSWAP_STL_VLARB_TABLE(&resp[i], section);

		memcpy(dest, (uint8_t*)(&resp[i]), cpySize);

		switch (section) {
			case STL_VLARB_LOW_ELEMENTS:
				portp->portData->current.vlarbLow = 1;
				break;
			case STL_VLARB_HIGH_ELEMENTS:
				portp->portData->current.vlarbHigh = 1;
				break;
			case STL_VLARB_PREEMPT_ELEMENTS:
				portp->portData->current.vlarbPre = 1;
				break;
			case STL_VLARB_PREEMPT_MATRIX:
				portp->portData->current.vlarbMatrix = 1;
				break;
		}
	}

	return VSTATUS_OK;
}


static Status_t
sm_aggregateToBfrctrl(Node_t * nodep, STL_AGGREGATE * aggr)
{
	const uint8_t *data = (uint8_t*)aggr->Data;
	const uint8_t startPort = (uint8_t)(aggr->AttributeModifier & 0xff);
	const uint8_t count = (nodep->nodeInfo.NodeType == NI_TYPE_CA) ? 1 : (uint8_t)(aggr->AttributeModifier >> 24);
	uint8_t i = 0;

	if (aggr->Result.s.Error)
		return VSTATUS_OK;

	if ((aggr->Result.s.RequestLength * 8) < (sizeof(STL_BUFFER_CONTROL_TABLE) * count))
		return VSTATUS_BAD;

	do {
		Port_t *portp = sm_get_port(nodep, startPort + i);

		if (sm_valid_port(portp)) {
			portp->portData->bufCtrlTable = *((STL_BUFFER_CONTROL_TABLE*)data);
			portp->portData->current.bfrctrl = 1;
			BSWAP_STL_BUFFER_CONTROL_TABLE(&(portp->portData->bufCtrlTable));
		}
		data += STL_BFRCTRLTAB_PAD_SIZE;
	} while (++i < count);

	return VSTATUS_OK;
}

static Status_t
WriteVLArbTables(Node_t* nodep, Port_t* portp,  STL_LID dlid, PortDataVLArb* arbp)
{
	uint32_t amod;
	uint16_t numPorts = 1;
	uint32_t dataSize = 0;
	Status_t status = VSTATUS_OK;

	// 
	// High priority table.
	// 
	if (!portp->portData->current.vlarbHigh || !portp->portData->current.vlarbLow || !portp->portData->current.vlarbMatrix) {
		IB_LOG_WARN_FMT(__func__,
			"VLArb information is not current on node %s nodeGUID "FMT_U64" port %d",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index);
	}

	amod = (numPorts << 24) | (STL_VLARB_HIGH_ELEMENTS << 16) | portp->index;

	dataSize = MIN(portp->portData->portInfo.VL.ArbitrationHighCap * sizeof(STL_VLARB_TABLE_ELEMENT), sizeof(portp->portData->curArb.u.vlarb.vlarbHigh));
	if (!portp->portData->current.vlarbHigh ||
		memcmp(portp->portData->curArb.u.vlarb.vlarbHigh, arbp->u.vlarb.vlarbHigh,
			dataSize) != 0 || sm_config.forceAttributeRewrite) {
		status = SM_Set_VLArbitration_LR(fd_topology, amod, sm_lid, dlid, (STL_VLARB_TABLE *)arbp->u.vlarb.vlarbHigh, sizeof(arbp->u.vlarb.vlarbHigh), sm_config.mkey);
	
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
							 "SET of VL Arbitration high priority table has failed for node %s guid "
							 FMT_U64 " status=%d", sm_nodeDescString(nodep),
							 nodep->nodeInfo.NodeGUID, status);
		}
		portp->portData->current.vlarbHigh = (status == VSTATUS_OK);
	}

	memcpy(portp->portData->curArb.u.vlarb.vlarbHigh, arbp->u.vlarb.vlarbHigh, dataSize);

	/* 
	 *  Low priority table.
	 */
	amod = (numPorts << 24) | (STL_VLARB_LOW_ELEMENTS << 16) | portp->index;

	dataSize = MIN(portp->portData->portInfo.VL.ArbitrationLowCap * sizeof(STL_VLARB_TABLE_ELEMENT), sizeof(portp->portData->curArb.u.vlarb.vlarbLow));
	if (!portp->portData->current.vlarbLow ||
		memcmp(portp->portData->curArb.u.vlarb.vlarbLow, arbp->u.vlarb.vlarbLow,
			dataSize) != 0 || sm_config.forceAttributeRewrite) {
			status = SM_Set_VLArbitration_LR(fd_topology, amod, sm_lid, dlid, (STL_VLARB_TABLE*) arbp->u.vlarb.vlarbLow, sizeof(arbp->u.vlarb.vlarbLow), sm_config.mkey);

		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
							 "SET of VL Arbitration low priority table has failed for node %s guid "
							 FMT_U64 " status=%d", sm_nodeDescString(nodep),
							 nodep->nodeInfo.NodeGUID, status);
		}
		portp->portData->current.vlarbLow = (status == VSTATUS_OK);
	}

	memcpy(portp->portData->curArb.u.vlarb.vlarbLow, arbp->u.vlarb.vlarbLow, dataSize);

	/* 
	 *  Preemption Matrix.
	 */
	if (sm_IsInterleaveEnabled(&portp->portData->portInfo)) {
		amod = (numPorts << 24) | (STL_VLARB_PREEMPT_MATRIX << 16) | portp->index;

		if (!portp->portData->current.vlarbMatrix ||
			memcmp(portp->portData->curArb.vlarbMatrix, arbp->vlarbMatrix,
				sizeof(portp->portData->curArb.vlarbMatrix)) != 0 || sm_config.forceAttributeRewrite) {
			status = SM_Set_VLArbitration_LR(fd_topology, amod, sm_lid, dlid, (STL_VLARB_TABLE*) arbp->vlarbMatrix, sizeof(arbp->vlarbMatrix), sm_config.mkey);
			
			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
								 "SET of VL Arbitration Matrix has failed for node %s guid "
								 FMT_U64 " status=%d", sm_nodeDescString(nodep),
								 nodep->nodeInfo.NodeGUID, status);
			}
			portp->portData->current.vlarbMatrix = (status == VSTATUS_OK);
		}

		memcpy(portp->portData->curArb.vlarbMatrix, arbp->vlarbMatrix, sizeof(arbp->vlarbMatrix));
	}

	return status;
}


Status_t
sm_initialize_VLArbitration(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	Status_t status = VSTATUS_OK;
	STL_LID dlid=0;
	IB_ENTER(__func__, topop, nodep, portp, 0);

	/* Note: If node was previously non-responding. Don't bother going any further. */
	if (nodep->nonRespCount) {
		IB_EXIT(__func__, 0);
		return (VSTATUS_OK);
	}

	if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {	// Port is down
		IB_EXIT(__func__, 1);
		return (VSTATUS_OK);
	}

	if ((portp->index == 0) &&	// Not a physical port
		!nodep->switchInfo.u2.s.EnhancedPort0) {
		IB_EXIT(__func__, 2);
		return (VSTATUS_OK);
	}

	if (portp->portData->vl0 == 1) {	// Only 1 VL supported
		IB_EXIT(__func__, 3);
		return (VSTATUS_OK);
	}
                                    
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t *swportp;
		swportp = sm_get_port(nodep, 0);
		if (!sm_valid_port(swportp)) {
			IB_LOG_WARN_FMT(__func__,
							"Failed to get Port 0 of Switch " FMT_U64,
							nodep->nodeInfo.NodeGUID);
			return VSTATUS_BAD;
		}
		dlid = swportp->portData->lid;
	} else {
		dlid = portp->portData->lid;
	}

	struct _PortDataVLArb * arbp = sm_port_getNewArb(portp);

	status = topop->routingModule->funcs.fill_stl_vlarb_table(topop, nodep, portp, arbp);

	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("Failed to get VL Arbitration "
			"data from routing algorithm; using default; rc:", status);
		sm_FillVlarbTableDefault(nodep, arbp, portp->portData->vl1);
	}

	if (nodep->vlArb) {
		status = WriteVLArbTables(nodep, portp, dlid, arbp);
	}

	// Whether things succeeded or not, have no use for newArb anymore
	sm_port_releaseNewArb(portp);

	IB_EXIT(__func__, status);
	return (status);
}


// We need to set the VLARB tables with equal cost slices.
// If minimum BW is 10% or a multiple of 10%, fits nicely
// into table.  
// If minimum BW is 5% and we had 20 entry arb table, this
// would return 5.  STL1 has 16 entries, not enough room 
// in vlarb table for 5% increments so adjustments are made.
static void
setWeightMultiplier(Qos_t * qos)
{
	int currentVl;
	int currentVlBw;
	uint8_t minBandwidth = 0xff;
	int tbase = 1;
	bitset_t* vlsInUse = &qos->lowPriorityVLs;

	// set default
	qos->weightMultiplier = 100;

	if (vlsInUse->nset_m <= 1) {
		return;
	}

	for (currentVl = bitset_find_first_one(vlsInUse); currentVl > -1;
		 currentVl = bitset_find_next_one(vlsInUse, currentVl + 1)) {
		if (qos->vlBandwidth.bw[currentVl] > 0) {
			currentVlBw = qos->vlBandwidth.bw[currentVl];
			if (minBandwidth > currentVlBw) {
				minBandwidth = currentVlBw;
			}
			if ((currentVlBw % 10) > 2)
				// not base 10 (may be rounded down)
				tbase = 0;
		}
	}

	if (tbase) {
		// base 10 fits in table without adjustments
		if ((minBandwidth != 100) &&
			(minBandwidth != 50)) {
			for (currentVl = bitset_find_first_one(vlsInUse); currentVl > -1;
			 	currentVl = bitset_find_next_one(vlsInUse, currentVl + 1)) {
				if (qos->vlBandwidth.bw[currentVl] > 0) {
					currentVlBw = qos->vlBandwidth.bw[currentVl];
					if (currentVlBw % minBandwidth) {
						minBandwidth = 10;
						break;
					}
				}
			}
		}
		qos->weightMultiplier = minBandwidth;
		return;
	}

	if (minBandwidth > 5) {
		// 5% doesn't fit into table as neatly as 10% increments
		// Make sure we have lowest bw instead of returning 5
		for (currentVl = bitset_find_first_one(vlsInUse); currentVl > -1;
			 currentVl = bitset_find_next_one(vlsInUse, currentVl + 1)) {
			if (qos->vlBandwidth.bw[currentVl] > 0) {
				currentVlBw = qos->vlBandwidth.bw[currentVl];
				if (currentVlBw % minBandwidth) {
					// not neatly devisable, start at 5%
					minBandwidth = 5;
					break;
				}
			}
		}
	} else
		minBandwidth = 5;

	qos->weightMultiplier = minBandwidth;
}

// This method sets a VLARB entry.
//
// twoSlicesPerSlot:
// 	For 5% BW slices, 16 entry table will not hold all entries.
// 	In this case, where total slices exceeds table, the 10% slices
// 	get weight*2.  Any 5% increment will get weight.  While this is
// 	not an equal slice, it is a best effort.
// bwFitsInTable:
//  This table is based on 16 entry STL HW. If the number of slices 
//  required does not fit into STL table, will give equal cost slices.
static void
SetVlarbEntry(STL_VLARB_TABLE_ELEMENT * vlblockp, uint8_t vl, int* entry,
	int weight, int bwFitsInTable, int twoSlicesPerSlot, 
	uint8_t* vlSlices, uint8_t* vlSlots, uint16_t* totalSlots)
{

	vlblockp[*entry].s.VL = vl;

	if (bwFitsInTable && !twoSlicesPerSlot) {
		vlblockp[*entry].Weight = weight;
		vlSlices[vl]--;
		vlSlots[vl]--;
		*totalSlots = *totalSlots-1;

	} else if (bwFitsInTable) {
		// bw fits in table if we double up the slices.
		// If one slot left, give it weight, otherwise double.
		if (vlSlices[vl] == 1) {
			// 5% remainder gets weight
			vlblockp[*entry].Weight = weight;
			vlSlices[vl]--;
			vlSlots[vl]--;
			*totalSlots = *totalSlots-1;
		} else {
			// 10% slot, decrement by two since slices are 5%
			// 254 is the maximum Weight supported by the PRR.
			// 255 means unlimited traffic.
			if (weight*2 > 254) {
				vlblockp[*entry].Weight = 254;
			} else {
				vlblockp[*entry].Weight = weight*2;
			}
			vlSlices[vl]-=2;
			vlSlots[vl]--;
			*totalSlots = *totalSlots-1;
		}
	} else {
		// Just give equal weight to all VLs.
		vlblockp[*entry].Weight = weight;
		vlSlices[vl] = 0;
		*totalSlots -= vlSlots[vl];
		vlSlots[vl] = 0;
	}
	*entry = *entry + 1;
}

// It is best for arbitration to use multiple equal size weight slices in a round-robin
// assignment than a single large slice per VL.
// This code is optimized for VL arb table with 16 entries or greater.
static void
FillLowRR(Node_t * nodep, Port_t * portp, STL_VLARB_TABLE_ELEMENT * vlblockp, Qos_t * qos, int weight)
{
	int currentEntry;
	uint8_t vlSlices[STL_MAX_VLS] = {0};
	uint8_t vlSlots[STL_MAX_VLS] = {0};
	int i, interleave = 0;
	int currentVl;
	int highbw = -1;
	uint16_t totalSlices = 0;
	uint16_t totalSlots = 0;
	int bwFitsInTable = 1;
	int bwRequiresTwoSlicesPerSlot = 0;
	bitset_t vlsInUse;
	bitset_t highbwVLs;
	VlarbList_t *mtuVlarbs = NULL;

	// Check to see if we have already calculated the VLARB table for this MTU value.
	for (mtuVlarbs = qos->vlarbList; mtuVlarbs; mtuVlarbs = mtuVlarbs->next) {
		if ((mtuVlarbs->mtu == portp->portData->maxVlMtu) &&
			(mtuVlarbs->cap == portp->portData->portInfo.VL.ArbitrationLowCap)) {
			break;
		}
	}
	if (mtuVlarbs) {
		memcpy(vlblockp, mtuVlarbs->vlarbLow, sizeof(STL_VLARB_TABLE_ELEMENT) * portp->portData->portInfo.VL.ArbitrationLowCap);
		return;
	}

	// Nothing cached, fill the table.
	sm_roundVLBandwidths(qos);

	if (!bitset_init(&sm_pool, &highbwVLs, STL_MAX_VLS)) {
		IB_FATAL_ERROR_NODUMP("FillLowRR: No memory for QoS setup, exiting.");
	}

	if (!bitset_init(&sm_pool, &vlsInUse, STL_MAX_VLS)) {
		IB_FATAL_ERROR_NODUMP("FillLowRR: No memory for QoS setup, exiting.");
	}

	bitset_copy(&vlsInUse, &qos->lowPriorityVLs);

	if ((qos->weightMultiplier == 5) &&
		(portp->portData->portInfo.VL.ArbitrationLowCap < 20)) {
		// Method expects at least 16 entries.
		// If 5% increments are used and they do not fit in the table,
		// 5% will get base weight and 10% will getweight*2.
		bwRequiresTwoSlicesPerSlot = 1;
	}

	for (currentVl = bitset_find_first_one(&vlsInUse); currentVl >= 0;
		 currentVl = bitset_find_next_one(&vlsInUse, currentVl + 1)) {

		vlSlices[currentVl] = qos->vlBandwidth.bw[currentVl] / qos->weightMultiplier;
		if (((qos->vlBandwidth.bw[currentVl] - (vlSlices[currentVl] * qos->weightMultiplier)) * 2) >=
			qos->weightMultiplier) {
			vlSlices[currentVl] += 1;
		}
		if (vlSlices[currentVl] <= 0)
			vlSlices[currentVl] = 1;

		// interleave large bw group so not duplicated at end of table.  
		if (qos->vlBandwidth.bw[currentVl] > highbw) {
			if (highbw >= 0) {
				bitset_clear_all(&highbwVLs);
			}
			highbw = qos->vlBandwidth.bw[currentVl];
			bitset_set(&highbwVLs, currentVl);
		} else if (qos->vlBandwidth.bw[currentVl] == highbw) {
			bitset_set(&highbwVLs, currentVl);
		}

		totalSlices += vlSlices[currentVl];

		if (bwRequiresTwoSlicesPerSlot) {
			vlSlots[currentVl] = vlSlices[currentVl]/2;
			if (vlSlices[currentVl] % 2)
				vlSlots[currentVl]++;
		} else {
			vlSlots[currentVl] += vlSlices[currentVl];
		}
		totalSlots += vlSlots[currentVl];
	}

	if (bwRequiresTwoSlicesPerSlot && totalSlots > portp->portData->portInfo.VL.ArbitrationLowCap) {
		// Handle oddball case where 1 VL has > 92% (10 slots) and
		// 7 other VLs share the remaining bandwidth (7 slots).
		// and
		// Handle oddball case where 2 VLs share > 95% (11 slots) and
		// 6 other VLs share the remaining bandwidth (6 slots).
		int bestChoice = -1;

		for (currentVl = bitset_find_first_one(&vlsInUse); currentVl >= 0;
		 	currentVl = bitset_find_next_one(&vlsInUse, currentVl + 1)) {
			// Find the VL with the fewest slots, but more than 1
			if (vlSlots[currentVl] > 1) {
				if (bestChoice < 0 || vlSlots[currentVl] < vlSlots[bestChoice]) {
					bestChoice = currentVl;
				}
			}
		}
		if (bestChoice >= 0) {
			int newSlices;

			// bestChoice is forced to lose 1 slice worth of bandwidth
			totalSlots--;
			vlSlots[bestChoice]--;

			// Remove old bestChoice slices from totalSlices
			// Calculate new bestChoice slices
			// Add new bestChoice slices back to totalSlices
			totalSlices -= vlSlices[bestChoice];
			newSlices = vlSlots[bestChoice] * 2;
			vlSlices[bestChoice] = newSlices;
			totalSlices += vlSlices[bestChoice];
		}
	}
	// If totalSlots is still more than ArbitrarionLowCap, then we
	// must have encountered a small ArbirtrationLowCap (less than 16)
	if (totalSlots > portp->portData->portInfo.VL.ArbitrationLowCap) {
		// Unexpected device (less than 16 vlarb entries) and BW configured with
		// too small of an increment.  Disable BW and log a warning.  Will allow
		// config to run, but QoS BW disbled on this device port.
           IB_LOG_WARN_FMT(__func__,
				"Disabling QoS BW allocation for node %s guid "
				FMT_U64", VLARB table is too small for this configuration",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
		bwFitsInTable = 0;
	}

	for (i = bitset_find_first_one(&highbwVLs); i>=0; 
		 i = bitset_find_next_one(&highbwVLs, i+1)) {
		bitset_clear(&vlsInUse, i);
	}

	currentVl = bitset_find_first_one(&vlsInUse);
	currentEntry = 0;

	while (currentEntry < portp->portData->portInfo.VL.ArbitrationLowCap) {
		for (i = bitset_find_first_one(&highbwVLs); i>=0; 
		 	 i = bitset_find_next_one(&highbwVLs, i+1)) {

			if (!interleave) {
				interleave = (int)(totalSlots - (bitset_nset(&highbwVLs)*vlSlots[i])) / vlSlots[i];
			}

			if (vlSlices[i] == 0)
				break;

			if (currentEntry >= portp->portData->portInfo.VL.ArbitrationLowCap)
				break;

			SetVlarbEntry(vlblockp, i, &currentEntry, weight, bwFitsInTable,
							bwRequiresTwoSlicesPerSlot, vlSlices, vlSlots, &totalSlots);

			if (!vlSlices[i] || !vlSlots[i]) {
				bitset_clear(&highbwVLs, i);
			}
		}

		for (i=0; i<interleave; i++) {
			if (currentVl < 0)
				break;

			if (currentEntry >= portp->portData->portInfo.VL.ArbitrationLowCap)
				break;

			SetVlarbEntry(vlblockp, currentVl, &currentEntry, weight, bwFitsInTable,
							bwRequiresTwoSlicesPerSlot, vlSlices, vlSlots, &totalSlots);

			if (!vlSlices[currentVl]) {
				bitset_clear(&vlsInUse, currentVl);
			}

			currentVl = bitset_find_next_one(&vlsInUse, currentVl + 1);
			if (currentVl < 0) {
				currentVl = bitset_find_first_one(&vlsInUse);
			}
		}
		interleave = 0;

		if (bitset_nset(&highbwVLs) == 0) {
			if ((bitset_nset(&vlsInUse) > 0) && totalSlots) {
				interleave = totalSlots;
			} else {
				 break;
			}
		}
	}

	bitset_free(&vlsInUse);
	bitset_free(&highbwVLs);

	// Cache for next port
	// DEBUG_ASSERT(!qos->vlarbList);
	vs_pool_alloc(&sm_pool, sizeof(VlarbList_t), (void*)&mtuVlarbs);
	if (mtuVlarbs) {
		memcpy(mtuVlarbs->vlarbLow, vlblockp, sizeof(STL_VLARB_TABLE_ELEMENT) * portp->portData->portInfo.VL.ArbitrationLowCap);
		mtuVlarbs->mtu = portp->portData->maxVlMtu;
		mtuVlarbs->cap = portp->portData->portInfo.VL.ArbitrationLowCap;
		mtuVlarbs->next = qos->vlarbList;
		qos->vlarbList = mtuVlarbs;
	}
}

Status_t
QosFillVlarbTable(Topology_t * topop, Node_t * nodep, Port_t * portp, Qos_t * qos, struct _PortDataVLArb * arbp)
{
	int currentVl;
	STL_VLARB_TABLE_ELEMENT *vlblockp;
	uint8_t currentEntry;
	int weight = 0;

	memset(arbp->u.vlarb.vlarbLow, 0, sizeof(arbp->u.vlarb.vlarbLow));
	memset(arbp->u.vlarb.vlarbHigh, 0, sizeof(arbp->u.vlarb.vlarbHigh));
	memset(arbp->u.vlarb.vlarbPre, 0, sizeof(arbp->u.vlarb.vlarbPre));

	weight = GetBytesFromMtu(portp->portData->maxVlMtu) / 64;

	// Setup high priority table.
	// No bandwidth associated with high priority.
	if (bitset_nset(&qos->highPriorityVLs)) {
		vlblockp = arbp->u.vlarb.vlarbHigh;
		currentVl = bitset_find_first_one(&qos->highPriorityVLs);

		for (currentEntry = 0; 
			 currentEntry < portp->portData->portInfo.VL.ArbitrationHighCap;
			 currentEntry++) {
			if (currentVl < 0)
				break;

			vlblockp[currentEntry].s.VL = currentVl;
			vlblockp[currentEntry].Weight = weight;

			currentVl = bitset_find_next_one(&qos->highPriorityVLs, currentVl + 1);
		}
	}

	// Setup low priority BW table
	if (qos->lowPriorityVLs.nset_m) {
		vlblockp = arbp->u.vlarb.vlarbLow;
		FillLowRR(nodep, portp, vlblockp, qos, weight);
	}

	return VSTATUS_OK;
}

/*
 * Print BW info about all VFs associated with all VL bits set in vlSet
 * Output is CSV-ish with tuples; not necessarily easy to machine-parse
 * but not impossible either
 *
 * VLs,<VLSet>[,SCs,<SCSet>,SLs,<SLSet>],VFs,{<VFSet>}
 *
 * Where:
 * 	VLSet :
 * 		VL
 * 		VL,VLSet
 *
 * 	SCSet :
 * 		SC
 * 		SC,SCSet
 *
 * 	SLSet :
 * 		SL
 * 		SL,SLSet
 *
 * 	VFSet :
 * 		VFDetails
 * 		VFDetails,VFSet
 *
 * 	VFDetails :
 * 		<Name> (<VFModifiers>)
 *
 *	VFModifiers :
 *		<SLTypeSet>,[<QoSModifiers>]
 *
 * 	SLTypeSet :
 * 		req
 * 		resp
 * 		req,resp
 *
 * 	QoSModifiers :
 * 		noqos | hiprio
 */
#define checked_snprintf(BUF_PTR,BUF_SIZE,...) \
	{ \
		int SNPRINTF_PR_COUNT = snprintf((BUF_PTR),(BUF_SIZE), __VA_ARGS__); \
		DEBUG_ASSERT(SNPRINTF_PR_COUNT < (BUF_SIZE)); \
		(BUF_SIZE) -= SNPRINTF_PR_COUNT; \
		(BUF_PTR) += SNPRINTF_PR_COUNT; \
	}

static void
DbgSprintVLVFBwInfo(Topology_t *topop, char *buf, int bufSize, Qos_t* qos,
	VirtualFabrics_t *vfs, STL_SLSCMAP *slsc, uint32_t vlSet)
{
	VF_t *vfSet[MAX_VFABRICS] = {0};
	uint32_t slSet = 0;
	uint32_t scSet = 0;

	DEBUG_ASSERT(vlSet);
	checked_snprintf(buf, bufSize, "VLs,{");

	// O(n^3) FTW!
	int i, vfIdx;
	for (i = 0; (vlSet >> i) != 0 &&  i < qos->activeVLs; ++i) {
		if (((vlSet >> i) & 0x1) == 0)
			continue;

		if (*(buf - 1) != '{')
			checked_snprintf(buf, bufSize, ",");
		checked_snprintf(buf, bufSize, "%d", i);
		for (vfIdx = 0; (vfIdx = bitset_find_next_one(&qos->vlvf.vf[i], vfIdx)) != -1; ++vfIdx) {
			uint8_t sl_i;

			VF_t *vf = &vfs->v_fabric_all[vfIdx];

			if (vf->standby) continue;

			for (sl_i = 0; sl_i < 3; sl_i++) {
				uint8_t sl, sc, high_sc;
				boolean mc_sl = 0;

				switch (sl_i) {
					default:
					case 0:
						sl = vf->base_sl; mc_sl = 0;
						break;
					case 1:
						sl = vf->resp_sl; mc_sl = 0;
						if (sl == vf->base_sl) continue;
						break;
					case 2:
						sl = vf->mcast_sl; mc_sl = 1;
						if (sl == vf->base_sl) continue;
						break;
				}

				sc = slsc->SLSCMap[sl].SC;
				high_sc = sc + topop->routingModule->funcs.num_routing_scs(sl, mc_sl);
				while (sc < high_sc) {
					if (qos->scvl.SCVLMap[sc].VL == i) {
						slSet |= (1 << sl);
						scSet |= (1 << sc);
						break;
					}
					sc++;
				}
			}

			int k;
			for (k = 0; k < MAX_VFABRICS; ++k) {
				if (vfSet[k] == vf)
					break;
				if (vfSet[k] == NULL) {
					vfSet[k] = vf;
					break;
				}
			}
			DEBUG_ASSERT(k < MAX_VFABRICS);
		}
	}

	checked_snprintf(buf, bufSize, "},SCs,{");
	for (i = 0; scSet >> i != 0 && i < STL_MAX_SCS; ++i) {
		if (((scSet >> i) & 0x1) == 0)
			continue;
		if (*(buf - 1) != '{')
			checked_snprintf(buf, bufSize, ",");
		checked_snprintf(buf, bufSize, "%d", i);
	}

	checked_snprintf(buf, bufSize, "},SLs,{");
	for (i = 0; (slSet >> i) != 0 && i < STL_MAX_SLS; ++i) {
		if (((slSet >> i) & 0x1) == 0)
			continue;
		if (*(buf - 1) != '{')
			checked_snprintf(buf, bufSize, ",");
		checked_snprintf(buf, bufSize, "%d", i);
	}

	checked_snprintf(buf, bufSize, "},VFs,{");
	for (i = 0; i < MAX_VFABRICS; ++i) {
		VF_t *vf = vfSet[i];
		if (!vf)
			break;

		if (*(buf - 1) != '{')
			checked_snprintf(buf, bufSize, ",");
		checked_snprintf(buf, bufSize, "%s (", vf->name);

		if (slSet != 0) {
			int hasBase = 0;
			if ((slSet >> vf->base_sl) & 0x1) {
				hasBase = 1;
				checked_snprintf(buf, bufSize, "base");
			}

			if (vf->base_sl != vf->resp_sl && ((slSet >> vf->resp_sl) & 0x1)) {
				if (hasBase) {
					checked_snprintf(buf, bufSize, ",");
				}
				checked_snprintf(buf, bufSize, "resp");
			}

			if (vf->base_sl != vf->mcast_sl && ((slSet >> vf->mcast_sl) & 0x1)) {
				if (hasBase) {
					checked_snprintf(buf, bufSize, ",");
				}
				checked_snprintf(buf, bufSize, "mcast");
			}

			if (!vf->qos_enable) {
				checked_snprintf(buf, bufSize, ",noqos");
			} else if (vf->priority) {
				checked_snprintf(buf, bufSize, ",hiprio");
			} else {
				checked_snprintf(buf, bufSize, ",lowprio");
			}
		}
		checked_snprintf(buf, bufSize, ")");

	}

	checked_snprintf(buf, bufSize, "}");
}
#undef checked_snprintf

static void
PrintVLVFBwInfo(Topology_t *topop, Node_t *nodep, Port_t *portp, Qos_t *qos,
	VirtualFabrics_t *vfs, PortDataVLArb *arbp)
{
	STL_SLSCMAP *dbgSlsc;
	int i, j;

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		Port_t *swp0 = sm_get_port(nodep, 0);
		dbgSlsc = &swp0->portData->slscMap;
	} else {
		dbgSlsc = &portp->portData->slscMap;
	}

	if (!nodep->vlArb) {
		return;
	}

	STL_VLARB_TABLE_ELEMENT *tbl[3] = { arbp->u.vlarb.vlarbLow,
		arbp->u.vlarb.vlarbHigh, arbp->u.vlarb.vlarbPre };
	int tblSize[3] = { STL_MAX_LOW_CAP, STL_MAX_LOW_CAP, STL_MAX_PREEMPT_CAP };

	for (i = 0; i < 3; ++i) {
		uint32_t vlSet = 0;
		uint32_t vlBw[STL_MAX_VLS] = {0};
		uint32_t totalBw = 0;

		for (j = 0; j < tblSize[i]; ++j) {
			uint8_t vl = tbl[i][j].s.VL;
			if (vl == 15 || tbl[i][j].Weight == 0)
				continue;

			vlSet |= (1 << vl);
			vlBw[vl] += tbl[i][j].Weight;
			totalBw += tbl[i][j].Weight;
		}

		if (totalBw == 0)
			continue;

		for (j = 0; j < STL_MAX_VLS; ++j) {
			char buf[1024] = "";
			int bufSize = sizeof(buf);

			if ((vlSet >> j) == 0)
				break;

			if (((vlSet >> j) & 0x1) == 0)
				continue;

			DbgSprintVLVFBwInfo(topop, buf, bufSize, qos, vfs,
				dbgSlsc, (1 << j)); // one VL at a time
			IB_LOG_DEBUG1_FMT(NULL, "Node,%s,Port,%d,VlArbTbl,%s,BwPct,%d,%s",
				sm_nodeDescString(nodep), portp->index,
				StlVlarbSecToText(STL_VLARB_LOW_ELEMENTS + i),
				(vlBw[j]*100)/totalBw, buf);
		}
	}
}


#if 0
// TEST
void dumpit(int32_t memSize, int16_t * pbw, uint8_t* mtu, int32_t wd, int32_t au, STL_BUFFER_CONTROL_TABLE * pBfrCtrl)
{
    int j, total= 0;
    //printf("Port %d: AU: %d\n",0, au);
    printf("  RxMem(AU/kB):      %6d/%6d\n", memSize, ((memSize*au)/1024));
    printf("  WireDepth(AU/B):   %6d/%6d\n", wd, (wd*au));
    printf("  Tot Shared(AU/kB): %6d/%6d\n", pBfrCtrl->TxOverallSharedLimit, ((pBfrCtrl->TxOverallSharedLimit*au)/1024));

    printf("\t");         for (j=0; j < 16; j++) { printf("VL%-4d", j);}
    printf("\n\t");       for (j=16;j < 32; j++) { printf("VL%-4d", j);}
    printf("\n  QOS\t");  for (j=0; j < 16; j++) { printf("%-6d", pbw[j]);}
    printf("\n\t");       for (j=16;j < 32; j++) { printf("%-6d", pbw[j]);}
    printf("\n  MTU\t");  for (j=0; j < 16; j++) { printf("%-6d", mtu[j]);}
    printf("\n\t");       for (j=16;j < 32; j++) { printf("%-6d", mtu[j]);}
    printf("\n  DED\t");  for (j=0; j < 16; j++) { printf("%-6d", pBfrCtrl->VL[j].TxDedicatedLimit); total+=pBfrCtrl->VL[j].TxDedicatedLimit; }
    printf("\n\t");       for (j=16;j < 32; j++) { printf("%-6d", pBfrCtrl->VL[j].TxDedicatedLimit); total+=pBfrCtrl->VL[j].TxDedicatedLimit; }
    printf("\n  SHRD\t"); for (j=0; j < 16; j++) { printf("%-6d", pBfrCtrl->VL[j].TxSharedLimit);}
    printf("\n\t");       for (j=16;j < 32; j++) { printf("%-6d", pBfrCtrl->VL[j].TxSharedLimit);}

    printf("\n");

    // Checks 
    if (pBfrCtrl->TxOverallSharedLimit+total != memSize) {
        printf("!!! ERROR: Sizes / Totals  do not match\n");
        printf("   TxOverallSharedLimit:  %d\n",pBfrCtrl->TxOverallSharedLimit);
        printf("   Dedicated Totals:      %d\n",total);
        printf("   Rx Memory Size :       %d\n",memSize);
    }
    printf("-------------------------\n");
}
#endif 


#define PROTOCOL_HEADER_SIZE 128

// The following defines are represented in terms of bytes
//  and will later be converted to buffer credits.
// [Buffer credit sizes may change in future generations]
//
// These defines are derived from tuning algorithms to determine the
//  additional latency incurred from using sideband credit returns and
//  packet credit returns.
// The buffer allocation schemes much account for this latency.
#define CR_LATENCY_SIDEBAND 2304   // 36 credits
#define CR_LATENCY_PACKET 4224     // 66 credits

static int bwCompare(const void * a, const void * b) { return (*(int16_t*)a - *(int16_t*)b);}

static Status_t 
setupBufferControl(int32_t memSize, int16_t * pbw, uint8_t* pmtu, int32_t wd, int32_t au,
                   bool_t shmem, bool_t wh, bool_t vl15,
				   STL_BUFFER_CONTROL_TABLE * pBfrCtrl)
{
    int       i, j;
    int32_t   intervalSize;
    int32_t   remainingSize;
    int32_t   packetSize[STL_MAX_VLS];
    int16_t   rankBw[STL_MAX_VLS];
    int32_t   header = PROTOCOL_HEADER_SIZE;
    int32_t   maxNoDed;
    uint8_t   mult = (uint8_t)sm_config.dedicatedVLMemMulti;  
    int32_t   minShared; 

    enum RULES {
        RULE_A = 1,
        RULE_B = 2, 
        RULE_C = 3
    } applied;

	// If this port does not support shared memory,
	// Ensure use of dedicated memory 
    if ((shmem==FALSE) && (mult==0)) {
		mult = 1;
    }

    // Ensure consistency of requests.
    minShared = (sm_config.minSharedVLMem * memSize)/100;
    if ((minShared>0) && (shmem==FALSE)) {
        minShared=0;
    }

    for (i=0; i<STL_MAX_VLS; i++) {
        // Initialize VLs buffer space to 0
        pBfrCtrl->VL[i].TxDedicatedLimit=0;
        pBfrCtrl->VL[i].TxSharedLimit=0;

        // Initialize the packet sizes.
        // Do everything in terms of au's
        // Ensure dedicated buffer space for VL15 if the neighbor needs VL15.
        if (i == 15) {
			if (TRUE == vl15) {
				packetSize[i] = (2048+header)*2;
				packetSize[i] = (packetSize[i] + au - 1)/au; /* round-up */

				// check for unexpected limitations
				//  which prevent even VL15 from having dedicated memory
				if (memSize-minShared-packetSize[i]-wd<0) {
					if (minShared > (packetSize[i] + wd)) {
						minShared -= (packetSize[i] + wd);
					} else {
						minShared = 0;
					}
				}
			} else {
				packetSize[i] = 0;
			}
        } else if (pbw[i]>0) {
                packetSize[i] = (GetBytesFromMtu(pmtu[i])+header)*mult;

            packetSize[i] = (packetSize[i] + au - 1)/au; /* round-up */
        } else {
            packetSize[i] = 0;
        }
    }


    // Attempt Rule A: allocate a packet-size plus wire depth
    remainingSize = memSize-minShared;
    intervalSize = 1;
    applied = RULE_A;
    for (i=0; i<STL_MAX_VLS; i++) {
        if (packetSize[i]>0) {
            remainingSize -= (packetSize[i]+wd);
            if (remainingSize < 0) break;

            intervalSize  += packetSize[i];
            pBfrCtrl->VL[i].TxDedicatedLimit = packetSize[i]+wd;
        }
    }

    // If no shared memory, apply RULE_A iteratively until all memory consumed.
    intervalSize = remainingSize/intervalSize;
    if ((shmem==FALSE) && (intervalSize >0)) {
        for (i=0; i<STL_MAX_VLS; i++) {
            if (packetSize[i]>0) {
                pBfrCtrl->VL[i].TxDedicatedLimit+=packetSize[i]*intervalSize; 
            }
        }
    }

    // Rank the QOS for subsequent rules
    if (remainingSize < 0) {
        memcpy(rankBw, pbw, sizeof(rankBw));
        qsort(rankBw, STL_MAX_VLS, sizeof(int16_t), bwCompare); 
    }

    // Attempt Rule B
    // (only applicable if Wire Depth is non-zero, otherwise result is rule A)
    if ((remainingSize < 0) && (wd > 0)){
        applied = RULE_B;

        for (i=0; i<STL_MAX_VLS;i++) {
            // optimize for duplicate rank values.
            if ((i<STL_MAX_VLS-1) && (rankBw[i]==rankBw[i+1])) continue;

            remainingSize = memSize-minShared;

            for (j=0; j<STL_MAX_VLS;j++) {
                if (j == 15 && !vl15) {
                    continue;
                }
                if (pbw[j]>rankBw[i] || j == 15) {
                    remainingSize -= (packetSize[j]+wd);
                    pBfrCtrl->VL[j].TxDedicatedLimit = packetSize[j]+wd; 
                } else {
                    remainingSize -= packetSize[j];
                    pBfrCtrl->VL[j].TxDedicatedLimit = packetSize[j]; 
                }
                if (remainingSize<0) break; 
            }

            if (remainingSize>=0) break; 
        }
    }

    // Attempt Rule C (must work)
    if (remainingSize < 0) {
        applied = RULE_C;

        for (i=0; i<STL_MAX_VLS;i++) {
            // optimize for duplicate rank values.
            if ((i<STL_MAX_VLS-1) && (rankBw[i]==rankBw[i+1])) continue;

            maxNoDed = 0;
            remainingSize = memSize-minShared;

            for (j=0; j<STL_MAX_VLS;j++) {
                if (j == 15 && !vl15) {
                    continue;
                }
                if (pbw[j]>rankBw[i] || j == 15) {
                    remainingSize -= (packetSize[j]+wd);
                    if (remainingSize<0) break; 

                    pBfrCtrl->VL[j].TxDedicatedLimit = packetSize[j]+wd; 
                } else {
                    maxNoDed = MAX(maxNoDed, (packetSize[j]+wd));
                    pBfrCtrl->VL[j].TxDedicatedLimit = 0;
                }
            }

            if (remainingSize>=0) {
                if (shmem==FALSE)
                    break;
                else if (remainingSize+minShared >= maxNoDed)
                     break; 
            }
        }
    }

    if (shmem==TRUE) {
        remainingSize+=minShared;
    } else {
        remainingSize=0;
    }
    pBfrCtrl->TxOverallSharedLimit = remainingSize;

    for (i = 0; i < STL_MAX_VLS; i++) {
        if (i == 15 && !vl15) {
            continue;
        }
        if (i == 15 || pbw[i]>0) {
            if (applied == RULE_C) {
                // In worst case scenarios, only allow VL to shared if dedicated is 0.
                if (pBfrCtrl->VL[i].TxDedicatedLimit==0) {
                    pBfrCtrl->VL[i].TxSharedLimit=remainingSize; 
                } 
            } else {
                pBfrCtrl->VL[i].TxSharedLimit=remainingSize; 
            }
        }
    }

    // Rule C works, but it might prevent forward progress.
    // We need to alert the user of this condition, but do not have
    //  meaningful information to put in the log / error messsage.
    // Return an error here so that caller can create appropriate log msg.
    if (applied == RULE_C) {
        return (VSTATUS_BAD);
    }

    return (VSTATUS_OK);
}

Status_t
sm_initialize_Port_BfrCtrl(Topology_t * topop, Node_t * nodep, Port_t * portp,
                            STL_BUFFER_CONTROL_TABLE *bct)
{
    Port_t*     neighborPort = 0;
    Node_t*     neighborNode = 0;
    int32_t     rxMemSize;
    int32_t     wd;
    int32_t     au;
    bool_t      shmem;
    int16_t     bw[STL_MAX_VLS];
    uint8_t     mtu[STL_MAX_VLS];
    int         vf, vl;
    VlVfMap_t   vlvfmap;
    VlBwMap_t   vlbwmap;
    VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;
    bool_t      needWd;
    bool_t      needVl15 = TRUE;
  
    // Find the neighbor port (Get neighbors recv buffer size and allocation units)
    neighborPort = sm_find_neighbor_node_and_port(topop, portp, &neighborNode);
    if (neighborPort==NULL) {
        IB_LOG_ERROR_FMT(__func__,
                         "Unable to find neighbor port for node %s guid "
                         FMT_U64 " Port number=%d", sm_nodeDescString(nodep),
                         nodep->nodeInfo.NodeGUID, portp->index);

        return (VSTATUS_BAD);
    }


    au=8*(1 << neighborPort->portData->portInfo.BufferUnits.s.BufferAlloc);
    shmem = neighborPort->portData->portInfo.CapabilityMask3.s.IsSharedSpaceSupported;


    // Wire Depth is calculated based on the modes the port pairs
    // are capable of and not what is currently enabled.
    needWd = (portp->portData->portInfo.PortLinkMode.s.Supported &
              neighborPort->portData->portInfo.PortLinkMode.s.Supported &
              STL_PORT_LINK_MODE_STL) == STL_PORT_LINK_MODE_STL;

    if (neighborNode->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        // Switch port zero has mask3 bits, look up switch port zero.
        // Note: PRR is setting up external switch ports instead of switch port zero,
        // so check both.
        Port_t* smaportp;
        smaportp = sm_get_port(neighborNode, 0);
        if (sm_valid_port(smaportp))
            shmem |= smaportp->portData->portInfo.CapabilityMask3.s.IsSharedSpaceSupported;
    }

    rxMemSize = neighborPort->portData->portInfo.OverallBufferSpace;
    if (rxMemSize==0) {
        IB_LOG_ERROR_FMT(__func__,
                         "Overall Rx Buffer size is zero for node %s guid "
                         FMT_U64 " Port number=%d", sm_nodeDescString(neighborNode),
                         neighborNode->nodeInfo.NodeGUID, neighborPort->index);
        return (VSTATUS_BAD);
    }

    // Intent is to initialize the initial WireDepth if:
    //  - First time SM is run (restart SM case, in which WD will be defaulted)
    //  - First time encountering the port (which may be active, WD will be defaulted)
    //  - Bounced ports (Port will be in init)
    //  - Initial configuration (Port will be in init)
    //
    // We always want to use the same value for Wire Depth even though it may vary
    // throughout the life of the connection even though the port may not bounce.
    // This value will be the max wire depth of the port pair.
	if ((portp->portData->initWireDepth == -1) ||
		(portp->state == IB_PORT_INIT)){
        if (!needWd) {
            wd = portp->portData->initWireDepth = 0;
        } else {
            // Tests to override wire depth / replay depths from configuration.
            if ((int32_t)(sm_config.wireDepthOverride) == -1) {
                // No override for Wire depth
                // Choose the largest LTP Round Trip among self and neighbor.
                wd = MAX(portp->portData->portInfo.ReplayDepth.WireDepth,
                    neighborPort->portData->portInfo.ReplayDepth.WireDepth);
                if ((int32_t)(sm_config.replayDepthOverride) == -1) {
                    // No override for Replay depth
                    // Choose the min of wire depth / replay depth, and covert to bytes.
                    wd = BYTES_PER_LTP * MIN(wd, portp->portData->portInfo.
                            ReplayDepth.BufferDepth);
                } else if (sm_config.replayDepthOverride == 0) {
                    // Do not consider replay depth, convert wire depth to bytes.
                    wd = wd * BYTES_PER_LTP;
                }
                else {
                    // Replay depth overriden.
                    // Choose the min of wire depth / replay depth, and covert to bytes.
                    wd = MIN(wd * BYTES_PER_LTP, sm_config.replayDepthOverride);
                }
            } else if (sm_config.wireDepthOverride == 0) {
                // Do not consider wire depth
                if ((int32_t)(sm_config.replayDepthOverride) == -1) {
                    // No override for replay depth. Choose replay depth; convert to bytes.
                    wd = portp->portData->portInfo.ReplayDepth.BufferDepth *
                            BYTES_PER_LTP;
                } else if (sm_config.replayDepthOverride == 0) {
                    // Do not consider either wire depth or replay depth from port info.
                    wd = 0;
                } else {
                    // Replay depth overridden. (Already in bytes)
                    wd = sm_config.replayDepthOverride;
                }
            } else {
                // Wire Depth overridden
                if ((int32_t)(sm_config.replayDepthOverride) == -1) {
                    // No override for replay depth.
                    // Choose min of wire depth (override) and replay depth. Convert to bytes.
                    wd = MIN(sm_config.wireDepthOverride,
                             portp->portData->portInfo.ReplayDepth.BufferDepth *
                                BYTES_PER_LTP);
                } else if (sm_config.replayDepthOverride == 0) {
                    // Do not consider replay depth; only overridden wire depth remains, already in bytes.
                    wd = sm_config.wireDepthOverride;
                } else {
                    // Both wire depth and reply depth overridden. Choose min, already in bytes.
                    wd = MIN(sm_config.wireDepthOverride,
                            sm_config.replayDepthOverride);
                }
            }

            // Add in "Extra Credits" to account for credit return latencies.
            // This is based on whether credits are returned in-band through idle flits
            // or through idle packets, which is based on CRC mode.
            // The "Extra Credits" are expressed in terms of bytes to allow for future expansion.
            if (portp->portData->portInfo.PortLTPCRCMode.s.Active ==
                    STL_PORT_LTP_CRC_MODE_14) {
                wd += CR_LATENCY_SIDEBAND;
            } else {
                wd += CR_LATENCY_PACKET;
            }

            // Convert WD from bytes to AU's (rounding up)
            wd = (wd + au - 1) / au;

            portp->portData->initWireDepth = wd;
            }
    } else {
        // Assumes port state is ARMED or ACTIVE
        // and that SM has seen this port before.
        wd = portp->portData->initWireDepth;
    }

    // Setup BW and MTU per VL based on this ports VL membership in VFs
    topop->routingModule->funcs.select_vlvf_map(topop, nodep, portp, &vlvfmap);
    topop->routingModule->funcs.select_vlbw_map(topop, nodep, portp, &vlbwmap);

    // Evaluate MTU and QOS for this VL.
    // Note mtu[...] has buffer space requirement in units of MTU or AU
    // depending on whether wh is false or true.
    for (vl=0;vl<STL_MAX_VLS;vl++) {
        mtu[vl]=0;
        bw[vl]=0;

        // If VL has no VFs associated with it, VL is not in use
        if (!bitset_nset(&vlvfmap.vf[vl])) {
            bitset_free(&vlvfmap.vf[vl]);
            continue;
        }

        for(vf = 0; (vf = bitset_find_next_one(&vlvfmap.vf[vl], vf)) != -1; ++vf) {
            mtu[vl] = MAX(mtu[vl], VirtualFabrics->v_fabric_all[vf].max_mtu_int);
        }
        if (vlbwmap.highPriority[vl]) {
            bw[vl] = 100;
        } else {
            bw[vl] = MAX(vlbwmap.bw[vl], 1);
        }
        mtu[vl] = MIN(mtu[vl], portp->portData->maxVlMtu);

        //free bitsets allocated in select
        bitset_free(&vlvfmap.vf[vl]);
    }

    // Setup the buffer control map.
    if (setupBufferControl(rxMemSize, bw, mtu,  wd, au, shmem, FALSE, needVl15,
        bct)!= VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__,
                         "Errors encountered for setup Buffer Control for node %s guid "
                         FMT_U64 " Port number=%d", sm_nodeDescString(nodep),
                         nodep->nodeInfo.NodeGUID, portp->index);
    }


    //printf("Node Description: %s, port:%d wd:%d TxD: %d\n", 
    //        sm_nodeDescString(nodep),
    //      portp->index,
    //      (portp->portData->portInfo.ReplayDepth.WireDepth * BYTES_PER_LTP),
    //      (portp->portData->portInfo.ReplayDepth.BufferDepth *BYTES_PER_LTP));
    //dumpit(rxMemSize, bw, mtu, wd, au, bct);

    return VSTATUS_OK;
}

/**
  @return Number of aggregate segments required to send @c n blocks of size @c s per block
*/
static __inline__
size_t stl_ReqAggrSegCount(size_t n, size_t s)
{
  if (s == 0) return 0;
  size_t blksPerSeg = STL_MAX_PAYLOAD_AGGREGATE/s;
  return (n + (blksPerSeg - 1))/blksPerSeg;
}

/**
  @return Amount of memory required to do aggregate send @c n blocks of size @c s per block
*/
static __inline__
size_t stl_ReqAggrSegMem(size_t n, size_t s)
{
  size_t segsReq = stl_ReqAggrSegCount(n, s);
  return segsReq * sizeof(STL_AGGREGATE) + n * s;
}

Status_t
sm_node_updateFields(IBhandle_t fd, STL_LID slid, Node_t * nodep, Port_t * smaportp)
{
    Status_t s = sm_node_updateFromTopo(nodep, &old_topology, sm_topop);

    if (s != VSTATUS_OK)
        return s;

    if (!sm_valid_port(smaportp))
        return VSTATUS_BAD;

    /*
     * For switches, smaportp refers to port 0.
     */
    if (nodep->aggregateEnable) {
        s = sm_node_updateFromSma_aggregate(fd, slid, nodep, smaportp);
    }

    // Note that for a multi-port HFI, if one aggregate operation fails and non-aggregate
    // succeeds, aggregates will be disabled for all ports on that HFI
    if (!nodep->aggregateEnable || s != VSTATUS_OK) {
        s = sm_node_updateFromSma_solo(fd, slid, nodep, smaportp);

        // Aggregate update failed but non-aggregate update succeeded, disable aggregates on this node
        if (nodep->aggregateEnable && s == VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__,
                "Disabling aggregate support on node %s nodeGUID "FMT_U64,
                sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
            nodep->aggregateEnable = 0;
        }
    }

    return s;
}

static Status_t
sm_node_updateFromSma_aggregate(IBhandle_t fd, STL_LID slid, Node_t * nodep, Port_t * smaportp)
{
    Status_t s = VSTATUS_BAD;

    uint8 startPort, numPorts;

    startPort = smaportp->index;

    switch (nodep->nodeInfo.NodeType) {
        case NI_TYPE_CA:
            numPorts = 1;
            break;
        case NI_TYPE_SWITCH:
            numPorts = nodep->nodeInfo.NumPorts + 1;
            break;
        default:
            return VSTATUS_BAD;
    }

    if (!sm_valid_port(smaportp)) {
        IB_LOG_ERROR_FMT(__func__,
            "Failed to get SMA port for node %s, node GUID "FMT_U64", port %d",
            sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index);
        return VSTATUS_BAD;
    }

    size_t reqMem =
        (sizeof(STL_AGGREGATE) + sizeof(STL_SLSCMAP)) +
        (sizeof(STL_AGGREGATE) + sizeof(STL_SCSLMAP)) +
        3 * stl_ReqAggrSegMem(numPorts, sizeof(STL_SCVLMAP)) + // 3 * -> SCVLt and SCVLnt and SCVLr

        // Not optimal; in the worst case, can do 2 VLArb blocks/segment
        4 * (numPorts * (sizeof(STL_AGGREGATE) + sizeof(STL_VLARB_TABLE))) + // vlarbLow, vlarbHigh, and preempt matrix/table use the same wire-size structure even though they are not the same size
		stl_ReqAggrSegMem(numPorts, STL_BFRCTRLTAB_PAD_SIZE);

    Port_t * portp = NULL;
    boolean getScvlt = FALSE;
    boolean getScvlnt = FALSE;
    boolean getScvlr = FALSE;
    boolean getVlarbLow = FALSE;
    boolean getVlarbHigh = FALSE;
	boolean getVlarbPre = FALSE;
    boolean getVlarbMatrix = FALSE;
	boolean getBufferCtrl = FALSE;

	boolean getSlsc = !smaportp->portData->current.slsc;
	boolean getScsl = !smaportp->portData->current.scsl;
    {
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, startPort + i);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            getScvlt |= !portp->portData->current.scvlt;
            getScvlnt |= !portp->portData->current.scvlnt;
            getScvlr |= (!portp->portData->current.scvlr && smaportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported);
            getBufferCtrl |= !portp->portData->current.bfrctrl;

            if (portp->portData->vl0 > 1) {
                if (nodep->vlArb) {
                    getVlarbHigh |= !portp->portData->current.vlarbHigh;
                    getVlarbLow |=  !portp->portData->current.vlarbLow;
                    getVlarbPre |=  !portp->portData->current.vlarbPre;
                    getVlarbMatrix |= !portp->portData->current.vlarbMatrix;

                }
            }

            if (getScvlt && getScvlnt &&
                (getScvlr || !smaportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) &&
                (getVlarbLow && getVlarbHigh && getVlarbPre && getVlarbMatrix) &&
                getBufferCtrl)
                break;
        }
    }

	if (!getScvlt && !getScvlnt && !getScvlr && 
		!getVlarbLow && !getVlarbHigh && !getVlarbPre && !getVlarbMatrix &&
		!getBufferCtrl && !getSlsc && !getScsl) {
		// Nothing to update
		return VSTATUS_OK;
	}

    STL_AGGREGATE *aggrBuffer;
    vs_pool_alloc(&sm_pool, reqMem, (void*)&aggrBuffer);
    if (!aggrBuffer)
        return VSTATUS_BAD;

    memset(aggrBuffer, 0, reqMem);
    STL_AGGREGATE *segHdr = aggrBuffer;

	const struct s_aggrList { boolean get; size_t size; uint32_t aid; boolean multiport; } aggrList[] = {
		 {getScvlt, sizeof(STL_SCVLMAP), STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE, TRUE},
		 {getScvlnt, sizeof(STL_SCVLMAP), STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE, TRUE},
		 {getScvlr, sizeof(STL_SCVLMAP), STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE, TRUE},
		 // NOTE: Bfrctrl tables are not 8 byte aligned, so must round up.
		 {getBufferCtrl, STL_BFRCTRLTAB_PAD_SIZE, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE, TRUE},
		 {getSlsc, sizeof(STL_SLSCMAP), STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, FALSE},
		 {getScsl, sizeof(STL_SCSLMAP), STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, FALSE} };
	size_t aggrListIndx = 0;

	do {
        const size_t portsPerSeg = STL_MAX_PAYLOAD_AGGREGATE/aggrList[aggrListIndx].size;
        const size_t segsReq = aggrList[aggrListIndx].multiport ? stl_ReqAggrSegCount(numPorts, aggrList[aggrListIndx].size) : 1;
        size_t i;
		uint8 port = startPort;
		uint8 blkCount = 1;

		if (!aggrList[aggrListIndx].get) continue;

        for (i = 0; i < segsReq; ++i) {

			if (aggrList[aggrListIndx].multiport) {
				port = (i * portsPerSeg) + startPort;
				blkCount = (segsReq > (i + 1)? portsPerSeg : numPorts - ((numPorts > 1) ? port : 0));
			}

            segHdr->AttributeID = aggrList[aggrListIndx].aid;
            segHdr->Result.s.Error = 0;
            segHdr->Result.s.RequestLength = (blkCount * aggrList[aggrListIndx].size + 7)/8;
            segHdr->AttributeModifier = aggrList[aggrListIndx].multiport ? ( (blkCount << 24) | port ) : 0;
            segHdr = STL_AGGREGATE_NEXT(segHdr);
        }
	} while (++aggrListIndx < (sizeof(aggrList)/sizeof(aggrList[0])) );


    if (nodep->vlArb) {
    const int SEC_COUNT = 4;
    boolean getVlarb[] = { getVlarbHigh, getVlarbLow, getVlarbPre, getVlarbMatrix };
    uint8_t vlarbSec[] = { STL_VLARB_HIGH_ELEMENTS, STL_VLARB_LOW_ELEMENTS, STL_VLARB_PREEMPT_ELEMENTS, STL_VLARB_PREEMPT_MATRIX };
    int i;
    for (i = 0; i < SEC_COUNT; ++i) {
        if (!getVlarb[i])
            continue;

        size_t blkSize = sizeof(STL_VLARB_TABLE);
        size_t portsPerSeg = STL_MAX_PAYLOAD_AGGREGATE/blkSize;

        assert(portsPerSeg > 0);

        uint8 blkCount = 0;

        Port_t * portp, * END_PORT; // END_PORT is one past the end

        if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
            portp = sm_get_port(nodep, PORT_A0(nodep));
            END_PORT = sm_get_port(nodep, PORT_A1(nodep)) + 1;
        }
        else {
            portp = smaportp;
            END_PORT = smaportp + 1;
        }

        Port_t * endPortp = NULL;

        // SMA will return an error for a Get(VLArb) on a port with only one data VL
        // So have to build request selectively
        while (portp && portp != END_PORT) {
            // Find first port for which we can get vlarb data
            while (portp != END_PORT &&
                (!sm_valid_port(portp) || portp->portData->vl0 <= 1 || portp->state <= IB_PORT_DOWN)) {
                ++portp;
            }

            if (portp == END_PORT)
                break;

            // Find up to blkCount contiguous ports that support more than one data VL
            endPortp = portp;
            while (endPortp != END_PORT && blkCount < portsPerSeg &&
                sm_valid_port(endPortp) && endPortp->portData->vl0 > 1 && portp->state > IB_PORT_DOWN) {
                ++blkCount;
                ++endPortp;
            }

            segHdr->AttributeID = STL_MCLASS_ATTRIB_ID_VL_ARBITRATION;
            segHdr->Result.s.Error = 0;
            segHdr->Result.s.RequestLength = (blkCount * blkSize + 7)/8;
            segHdr->AttributeModifier = (blkCount << 24) | (vlarbSec[i] << 16) | portp->index;
            segHdr = STL_AGGREGATE_NEXT(segHdr);

            blkCount = 0;
            portp = endPortp;
        }
    }

    }

    uint32_t madStatus;
    STL_AGGREGATE * lastSeg = NULL;

    STL_LID dlid = smaportp->portData->lid;
    s = SM_Get_Aggregate_LR(fd, aggrBuffer, segHdr, slid, dlid, &lastSeg, &madStatus);

    if (!lastSeg && s == VSTATUS_OK) {
        s = VSTATUS_BAD;
        goto bail; 
    }

    if (lastSeg) {
        // Can still process partial aggregate response on MAD error
        Status_t tmpStatus = sm_node_handleGetRespAggr(nodep, smaportp, aggrBuffer, lastSeg);
        s = (s != VSTATUS_OK? s : tmpStatus);
    }

    if (s != VSTATUS_OK || madStatus != MAD_STATUS_SUCCESS) {
        IB_LOG_ERROR_FMT(__func__,
            "Error on Get(Aggregate): NodeGUID : "FMT_U64"; NodeDesc : \"%s\"; port : %d; status : %d; madStatus : 0x%02x",
            nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), smaportp->index, s, madStatus);
        if (lastSeg) {
            IB_LOG_ERROR_FMT(__func__,
                "First error: AttributeID : 0x%02x; AttributeModifier : 0x%08x",
                lastSeg->AttributeID, lastSeg->AttributeModifier);
        }

        // Set s to BAD if it was an SMA error on a particular segment
        if (s == VSTATUS_OK)
            s = VSTATUS_BAD;
        goto bail;
    }

bail:
    vs_pool_free(&sm_pool, aggrBuffer);

    return s;
}

// NOTE:  Why is this done every sweep?
// 
static Status_t
sm_node_updateFromSma_solo(IBhandle_t fd, STL_LID slid, Node_t * nodep, Port_t * smaportp)
{
    uint32_t amod;
    Status_t s = VSTATUS_OK;
    uint8 buffer[STL_MAX_PAYLOAD_SMP_LR];
    uint8 startPort, numPorts;

    startPort = smaportp->index;

    switch (nodep->nodeInfo.NodeType) {
        case NI_TYPE_CA:
            numPorts = 1;
            break;
        case NI_TYPE_SWITCH:
            numPorts = nodep->nodeInfo.NumPorts + 1;
            break;
        default:
            return VSTATUS_BAD;
    }

    if (!sm_valid_port(smaportp)) {
        IB_LOG_ERROR_FMT(__func__,
            "Failed to get SMA port for node %s, node GUID "FMT_U64", port %d",
            sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index);
        return VSTATUS_BAD;
    }

    STL_LID dlid = smaportp->portData->lid;

    if (!smaportp->portData->current.slsc
		) {

        s = SM_Get_SLSCMap_LR(fd, 0, slid, dlid, (STL_SLSCMAP*)buffer);
        if (s != VSTATUS_OK)
            return s;

        smaportp->portData->slscMap = *((STL_SLSCMAP*)buffer);
        smaportp->portData->current.slsc = 1;
    }

    if (!smaportp->portData->current.scsl
        ) {
        // Get(SCSL)
        s = SM_Get_SCSLMap_LR(fd, 0, slid, dlid, (STL_SCSLMAP*)buffer);
        if (s != VSTATUS_OK)
            return s;
        smaportp->portData->scslMap = *((STL_SCSLMAP*)buffer);
        smaportp->portData->current.scsl = 1;
    }

    boolean getScvlt = FALSE;
    boolean getScvlnt = FALSE;
    boolean getScvlr = FALSE;
    boolean getVlarbLow = FALSE;
    boolean getVlarbHigh = FALSE;
    boolean getVlarbPre = FALSE;
    boolean getVlarbMatrix = FALSE;
	boolean getBufferCtrl= FALSE;

    Port_t * portp = NULL;

    {
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, startPort + i);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            getScvlt |= !portp->portData->current.scvlt;
            getScvlnt |= !portp->portData->current.scvlnt;
            getScvlr |= (!smaportp->portData->current.scvlr && portp->portData->portInfo.CapabilityMask3.s.IsVLrSupported);
			getBufferCtrl |= !portp->portData->current.bfrctrl;

            if (portp->portData->vl0 > 1) {
                if (nodep->vlArb) {
                    getVlarbHigh |= !portp->portData->current.vlarbHigh;
                    getVlarbLow |=  !portp->portData->current.vlarbLow;
                    getVlarbPre |=  !portp->portData->current.vlarbPre;
                    getVlarbMatrix |= !portp->portData->current.vlarbMatrix;
                }
            }

            if (getScvlt && getScvlnt &&
                (getScvlr || !smaportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) &&
                (getVlarbLow && getVlarbHigh && getVlarbPre && getVlarbMatrix) &&
                getBufferCtrl)
                break;
        }
    }

    if (getScvlt) {
        // Get(SCVLt) - fallback code so get ports individually
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, i + startPort);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            amod = (1 << 24) | portp->index;

            s = SM_Get_SCVLtMap_LR(fd, amod, slid, dlid, (STL_SCVLMAP*)buffer);

            if (s != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__,
                    "Failed to get SCVLt for nodeGUID 0x"FMT_U64", port %d",
                    nodep->nodeInfo.NodeGUID, portp->index);
                //TODO: should continue but make sure non-zero status is returned
            }
            else {
                portp->portData->current.scvlt = 1;
                portp->portData->scvltMap = *((STL_SCVLMAP*)buffer);
            }
        }
    }

    if (getScvlnt) {
        // Get(SCVLnt) - fallback code so get ports individually
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, i + startPort);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            amod = (1 << 24) | portp->index;

            s = SM_Get_SCVLntMap_LR(fd, amod, slid, dlid, (STL_SCVLMAP*)buffer);

            if (s != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__,
                    "Failed to get SCVLnt for nodeGUID 0x"FMT_U64", port %d",
                    nodep->nodeInfo.NodeGUID, portp->index);
                //TODO: should continue but make sure non-zero status is returned
            }
            else {
                portp->portData->current.scvlnt = 1;
                portp->portData->scvlntMap = *((STL_SCVLMAP*)buffer);
            }
        }
    }

    if (getScvlr) {
        // Get(SCVLr) - fallback code so get ports individually
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, i + startPort);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            amod = (1 << 24) | portp->index;

            s = SM_Get_SCVLrMap_LR(fd, amod, slid, dlid, (STL_SCVLMAP*)buffer);

            if (s != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__,
                    "Failed to get SCVLr for nodeGUID 0x"FMT_U64", port %d",
                    nodep->nodeInfo.NodeGUID, portp->index);
                //TODO: should continue but make sure non-zero status is returned
            }
            else {
                portp->portData->current.scvlr = 1;
                portp->portData->scvlrMap = *((STL_SCVLMAP*)buffer);
            }
        }
    }

	if (getBufferCtrl) {
		s = sm_get_buffer_control_tables(fd_topology, nodep, 1, PORT_P1(nodep));

		if (s == VSTATUS_OK) {
			Port_t *temp;

			for_all_ports(nodep, temp) {
				if (sm_valid_port(temp))
					temp->portData->current.bfrctrl = 1;
			}
		}
	}

    if (nodep->vlArb) {
    const int SEC_COUNT = 4;
    boolean getVlarb[] = { getVlarbHigh, getVlarbLow, getVlarbPre, getVlarbMatrix };
    uint8_t vlarbSec[] = { STL_VLARB_HIGH_ELEMENTS, STL_VLARB_LOW_ELEMENTS, STL_VLARB_PREEMPT_ELEMENTS, STL_VLARB_PREEMPT_MATRIX };

    int i;
    for (i = 0; i < SEC_COUNT; ++i) {

        if (!getVlarb[i])
            continue;

        // Wire block size is the same for low, high, and preempt matrix even though
        // internal sizes may be different
        size_t blkSize = sizeof(STL_VLARB_TABLE);

        // SMA will return an error on a multiport request if any of the ports do not
        // support more than one data VL.  So hardcode blksPerMad to '1' for now to avoid this problem
        size_t blksPerMad = 1; // = STL_MAX_PAYLOAD_SMP_LR/blkSize

        uint8 j;
        for (j = 0; j < numPorts; ++j) {
            portp = sm_get_port(nodep, j + startPort);

            if (blksPerMad == 1) {
                if (!sm_valid_port(portp) || portp->portData->vl0 <= 1 || portp->state <= IB_PORT_DOWN)
                    continue;
            }

            // May have to do multiple blocks for switches to get VLArb values for all ports
            if (j % blksPerMad == 0) {
                uint8_t blkCount = MIN(numPorts - j, blksPerMad);
                amod = (blkCount << 24) | (vlarbSec[i] << 16) | ( j + startPort);

                s = SM_Get_VLArbitration_LR(fd, amod, slid, dlid, (STL_VLARB_TABLE*)buffer);
                if (s != VSTATUS_OK) {
                    //TODO: should continue but make sure non-OK status is returned
                    return s;
                }
            }

            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            struct _PortDataVLArb * arbp = &portp->portData->curArb;
            uint8_t * dest = NULL;
            size_t cpySize = 0;
            switch (vlarbSec[i]) {
                case STL_VLARB_LOW_ELEMENTS:
                        dest = (uint8_t*)arbp->u.vlarb.vlarbLow;
                        cpySize = sizeof(arbp->u.vlarb.vlarbLow);
                        portp->portData->current.vlarbLow = 1;
                    break;
                case STL_VLARB_HIGH_ELEMENTS:
                        dest = (uint8_t*)arbp->u.vlarb.vlarbHigh;
                        cpySize = sizeof(arbp->u.vlarb.vlarbHigh);
                        portp->portData->current.vlarbHigh = 1;
                    break;
                case STL_VLARB_PREEMPT_ELEMENTS:
                        dest = (uint8_t*)arbp->u.vlarb.vlarbPre;
                        cpySize = sizeof(arbp->u.vlarb.vlarbPre);
                        portp->portData->current.vlarbPre = 1;
                    break;
                case STL_VLARB_PREEMPT_MATRIX:
                    dest = (uint8_t*)arbp->vlarbMatrix;
                    cpySize = sizeof(arbp->vlarbMatrix);
                        portp->portData->current.vlarbMatrix = 1;
                    break;
                default:
                    return VSTATUS_BAD;
            }

            memcpy(dest, ((uint8_t*)buffer) + (j%blksPerMad)*blkSize, cpySize);
            }
        }

    }

    return s;
}

static Status_t
sm_node_updateFromTopo(Node_t * nodep, Topology_t * oldTopop, Topology_t * curTopop)
{
	Port_t * portp;

	if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH && !sm_config.use_cached_hfi_node_data)
	 	return VSTATUS_OK;

	if (&old_topology != oldTopop)
		return VSTATUS_BAD; // only support copying from old_topology now

	// If we have a valid topology, and we believe that nothing has
	// changed, iterate through the ports copying data from the old topo to
	// the new one.
	uint32_t skipWrite = (sm_config.skipAttributeWrite & (SM_SKIP_WRITE_MAPS | SM_SKIP_WRITE_VLARB) ? 1 : 0); 

	for_all_physical_ports(nodep, portp) {
		if (!sm_valid_port(portp)) continue;
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (!QListInit(&portp->portData->scscMapList[0])) return VSTATUS_BAD;

		}
	}

	if ((topology_passcount >= 1) && 
		(!sm_config.forceAttributeRewrite || (sm_config.forceAttributeRewrite == skipWrite)) && 
		old_topology.vfs_ptr && curTopop->vfs_ptr &&
		(old_topology.vfs_ptr->qosEnabled == curTopop->vfs_ptr->qosEnabled)) {

		if (nodep->oldExists) {
			Node_t * oldNodep = nodep->old;
			if (oldNodep && !oldNodep->slscChange) {
				// Already programmed, copy the mappings and return.
				for_all_ports(nodep, portp) {
					if (!sm_valid_port(portp) || portp->state < IB_PORT_ACTIVE) 
						continue; 
					
					Port_t * oldPortp = sm_get_port(oldNodep, portp->index); 
					if (!sm_valid_port(oldPortp) || oldPortp->state < IB_PORT_ARMED)
						continue;

					// Copy sl, sc, and vl related mapping tables
					portp->portData->slscMap = oldPortp->portData->slscMap;
					portp->portData->current.slsc = 1;
					portp->portData->scslMap = oldPortp->portData->scslMap;
					portp->portData->current.scsl = 1;
					portp->portData->scvltMap = oldPortp->portData->scvltMap;
					portp->portData->current.scvlt = 1;
					portp->portData->scvlntMap = oldPortp->portData->scvlntMap;
					portp->portData->current.scvlnt = 1;
					portp->portData->scvlrMap = oldPortp->portData->scvlrMap;
					portp->portData->current.scvlr = 1;
					portp->portData->bufCtrlTable = oldPortp->portData->bufCtrlTable;
					portp->portData->current.bfrctrl = 1;

					if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && !QListIsEmpty(&oldPortp->portData->scscMapList[0])) {
						sm_copyPortDataSCSCMap(oldPortp, portp, 0);
						portp->portData->current.scsc = 1;

						if (!QListIsEmpty(&oldPortp->portData->scscMapList[1]))
							sm_copyPortDataSCSCMap(oldPortp, portp, 1);
					}

					if (nodep->vlArb) {
						//
						// Copy vlarb
						memcpy(portp->portData->curArb.u.vlarb.vlarbLow, oldPortp->portData->curArb.u.vlarb.vlarbLow, sizeof(portp->portData->curArb.u.vlarb.vlarbLow)); 
						portp->portData->current.vlarbLow = 1;
						memcpy(portp->portData->curArb.u.vlarb.vlarbHigh, oldPortp->portData->curArb.u.vlarb.vlarbHigh, sizeof(portp->portData->curArb.u.vlarb.vlarbHigh)); 
						portp->portData->current.vlarbHigh = 1;
						memcpy(portp->portData->curArb.u.vlarb.vlarbPre, oldPortp->portData->curArb.u.vlarb.vlarbPre, sizeof(portp->portData->curArb.u.vlarb.vlarbPre)); 
						portp->portData->current.vlarbPre = 1;
						memcpy(portp->portData->curArb.vlarbMatrix, oldPortp->portData->curArb.vlarbMatrix, sizeof(portp->portData->curArb.vlarbMatrix));
						portp->portData->current.vlarbMatrix = 1;
					}
				}
			}
		}
	}

	return VSTATUS_OK;
}

void
sm_FillVlarbTableDefault(Node_t *nodep, struct _PortDataVLArb * arb, uint8_t numVls)
{
	int i, currentVl;

	if (nodep->vlArb) {
		// One entry per data VL, one credit per entry, fill the rest with (15,0)
		for (i = currentVl = 0; i < numVls && currentVl < STL_MAX_LOW_CAP; currentVl++, i++) {
			if (currentVl == 15) currentVl++; // Skip VL15.
			arb->u.vlarb.vlarbLow[i].s.VL = currentVl;
			arb->u.vlarb.vlarbLow[i].Weight = 1;
		}
		for (; i< STL_MAX_LOW_CAP; i++) {
			arb->u.vlarb.vlarbLow[i].s.VL = 15;
			arb->u.vlarb.vlarbLow[i].Weight = 0;
		}

		// Filling high table with (15,0)
		for (i = 0; i < STL_MAX_LOW_CAP; i++) {
			arb->u.vlarb.vlarbHigh[i].s.VL = 15;
			arb->u.vlarb.vlarbHigh[i].Weight = 0;
		}

		// Filling pre-empt table with (15,0).
		for (i = 0; i < STL_MAX_PREEMPT_CAP; i++) {
			arb->u.vlarb.vlarbPre[i].s.VL = 15;
			arb->u.vlarb.vlarbPre[i].Weight = 0;
		}

	}

	memset(arb->vlarbMatrix, 0, sizeof(arb->vlarbMatrix));
}

Status_t
sm_select_vlvf_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlVfMap_t * vlvfmap)
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

    Qos_t *qos = sm_get_qos(portp->portData->vl1);

    Port_t *neighbor_portp = NULL; // Will make a parameter(?)
    bitset_t * vfmember = NULL;
    int vl;

	if (portp->portData->isIsl) {
		for (vl = 0; vl < STL_MAX_VLS; vl++) {
			if (!bitset_init(&sm_pool, &(vlvfmap->vf[vl]), MAX_VFABRICS)) {
				IB_FATAL_ERROR("sm_select_vlvf_map: Out of memory, exiting.");
			}
			bitset_copy(&vlvfmap->vf[vl], &qos->vlvf.vf[vl]);
		}
		return VSTATUS_OK; // We are done. No filtering for switch
	}

    if (portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_CA) {
        // HFI - use the ports vf member data.
        vfmember = &portp->portData->vfMember;

    } else {
    	// Need neighbor node / port for switches.
        if (neighbor_portp==NULL) {
            neighbor_portp=sm_find_port(topop, portp->nodeno, portp->portno);
        }

        if (neighbor_portp!=NULL) {
           vfmember = &neighbor_portp->portData->vfMember;

        } else {
            // Null port encountered.  Should never happen.
            return VSTATUS_BAD;
        }
    }

    // Should be at least a member of default VF..
    if (bitset_nset(vfmember)==0)
        return VSTATUS_BAD;

    // The vlvf.vf table contains only active VF indexes
    for (vl=0; vl < STL_MAX_VLS; vl++) {
        if (!bitset_init(&sm_pool, &(vlvfmap->vf[vl]), MAX_VFABRICS)) {
            IB_FATAL_ERROR("sm_select_vlvf_map: Out of memory, exiting.");
        }
		bitset_set_intersection(&qos->vlvf.vf[vl], vfmember, &vlvfmap->vf[vl]);
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


Status_t
sm_fill_stl_vlarb_table(Topology_t *topop, Node_t *nodep, Port_t *portp, PortDataVLArb* arbp)
{
	uint8_t		numVls;
	int 		i, j, vf;
    Qos_t *    qos;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;
	Status_t	status = VSTATUS_OK;
	uint8_t		vlRank[STL_MAX_VLS];

	if (portp && portp->portData->qosHfiFilter) {
		// filtered by hca vf membership, vlarb already setup on sl/sc/vl mapping call
		return VSTATUS_OK;
	}

	if (portp) {
		numVls = portp->portData->vl1;
	} else {
		numVls = nodep->vlCap;
	}

 	qos = sm_get_qos(numVls);

	memset(arbp, 0, sizeof(*arbp));

	if (!VirtualFabrics) {
		sm_FillVlarbTableDefault(nodep, arbp, numVls);
		return VSTATUS_OK;
	}

	VlVfMap_t  vlvfmap;
	if (portp) {
		topop->routingModule->funcs.select_vlvf_map(topop, nodep, portp, &vlvfmap);
	} else {
		// Internal table (INQ/subswitch)
		vlvfmap = qos->vlvf;
	}

	// Generate Preemption Matrix
	// Note - possibly move this to Qos_t and determine once and done.
	memset(vlRank, 0, sizeof(vlRank));
	// Determine the max each VL
	for (i=0; i<STL_MAX_VLS; i++) {
		// All VFs on a single VL have the same preemption rank
		vf = bitset_find_first_one(&vlvfmap.vf[i]);
		if (vf==-1) continue;
		if (i >= numVls || i==15) {
			IB_LOG_WARN("Unexpected VF:: Mapping: VL=", i);
			break;
		}
		vlRank[i] = VirtualFabrics->v_fabric_all[vf].preempt_rank; 
	}
	// Iterate through all the VLs, creating the preemption matrix.
	for (i = 0; i < numVls; i++ ) {
		for (j = 0; j < numVls; j++) {
			if ( (vlRank[j]!=0) && (vlRank[i]!=0) && (vlRank[i]>vlRank[j]) ) {
				arbp->vlarbMatrix[i] |= 1 << j;
			}
		}
	}

	if (nodep->vlArb && portp) {
		// port always passed for STL1 vlarb mode
		status = QosFillVlarbTable(topop, nodep, portp, qos, arbp);
	}

	if (portp) {
		for(i = 0; i < STL_MAX_VLS; i++){
			bitset_free(&vlvfmap.vf[i]);
		}
	}

	if (IB_LOG_IS_INTERESTED(VS_LOG_DEBUG1) && status == VSTATUS_OK &&
		portp && nodep->nodeInfo.NodeType == NI_TYPE_CA) {
		// This debug output doesn't handle SC->SC' mappings,
		// so skip switches entirely for now
		PrintVLVFBwInfo(topop, nodep, portp, qos, VirtualFabrics, arbp);
	}

	return status;
}

Status_t
sm_select_slsc_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SLSCMAP *outSlscMap)
{
	uint8_t sl;
	int vf = 0;
	STL_SLSCMAP slsc;

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	/* loop will look for each bit set starting from bit 0 to last bit set,
	 * which will not exceed bitset size
	 */
	for (vf = 0; (vf = bitset_find_next_one(&out_portp->portData->vfMember, vf)) != -1; vf++) {
		/* In order to generate unique SL2SC map for this egress port,
		 * filter the SLs based on this port's VF memberships.
		 */
		bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].base_sl);
		if (VirtualFabrics->v_fabric_all[vf].base_sl != VirtualFabrics->v_fabric_all[vf].resp_sl)
			bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].resp_sl);
		if (VirtualFabrics->v_fabric_all[vf].base_sl != VirtualFabrics->v_fabric_all[vf].mcast_sl)
			bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].mcast_sl);
	}

	if (bitset_nset(&sm_linkSLsInuse) == 0)
		return VSTATUS_BAD;

	// SLSC default mappings
	memset(&slsc, 15, sizeof(slsc));

	// populate the SL2SC map table with entries from the baseline SL2SC
	// map table, based on SLs associated with this port.
	for (sl = 0; sl < STL_MAX_SLS; sl++) {
		if (bitset_test(&sm_linkSLsInuse, sl))
			slsc.SLSCMap[sl].SC = sm_SLtoSC[sl];
	}

	memcpy(outSlscMap, &slsc, sizeof(STL_SLSCMAP));
	return VSTATUS_OK;
}

Status_t
sm_select_scsl_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SCSLMAP *outScslMap)
{
	uint8_t sl, sc;
	int vf = 0;
	STL_SCSLMAP scsl;

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	for (vf = 0; (vf = bitset_find_next_one(&in_portp->portData->vfMember, vf)) != -1; vf++) {
		bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].base_sl);
		if (VirtualFabrics->v_fabric_all[vf].base_sl != VirtualFabrics->v_fabric_all[vf].resp_sl)
			bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].resp_sl);
		if (VirtualFabrics->v_fabric_all[vf].base_sl != VirtualFabrics->v_fabric_all[vf].mcast_sl)
			bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].mcast_sl);
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
		if (!bitset_test(&sm_linkSLsInuse, sl)) {
			// No actual way to mark an ingress SC as invalid in SCVL table
			// but set to SL15 anyway
			scsl.SCSLMap[sc].SL = 15;
		} else
			scsl.SCSLMap[sc].SL = sl;
	}

	memcpy(outScslMap, &scsl, sizeof(STL_SCSLMAP));
	return VSTATUS_OK;
}

Status_t
sm_select_scvlr_map(Topology_t *topop, uint8_t vlCap, STL_SCVLMAP *outScvlMap)
{
	int sc, vf;
	Qos_t *qos = sm_get_qos(vlCap);

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	memset(outScvlMap, 0, sizeof(STL_SCVLMAP));

	// In order to generate unique SL2SC map for this egress port, 
	// filter the SLs based on this port's VF memberships.
	for (vf = 0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		// Find all the SLs. Can't just use sm_linkSLsInuse, because it's used as a scratch variable
		bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].base_sl);
		bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].resp_sl);
		bitset_set(&sm_linkSLsInuse, VirtualFabrics->v_fabric_all[vf].mcast_sl);
	}

	for (sc=0; sc < STL_MAX_SCS; sc++) {
		if (bitset_test(&sm_linkSLsInuse, sm_SCtoSL[sc])) {
			outScvlMap->SCVLMap[sc].VL = qos->scvl.SCVLMap[sc].VL;
		} else {
			outScvlMap->SCVLMap[sc].VL = 15;
		}
	}
	return VSTATUS_OK;
}


void
sm_printf_vf_debug(VirtualFabrics_t *vfs)
{
    int vf;
    for (vf=0; vf<vfs->number_of_vfs_all; vf++) {
		if (vfs->v_fabric_all[vf].standby) continue;
		IB_LOG_INFINI_INFO_FMT_VF(vfs->v_fabric_all[vf].name, "",
            "Base SL:%d Resp SL:%d Requires Resp SL:%d Multicast SL:%d QOS:%d HP:%d PKey:0x%04x",
            vfs->v_fabric_all[vf].base_sl, vfs->v_fabric_all[vf].resp_sl,
            vfs->v_fabric_all[vf].requires_resp_sl, vfs->v_fabric_all[vf].mcast_sl,
            vfs->v_fabric_all[vf].qos_enable, vfs->v_fabric_all[vf].priority,
            vfs->v_fabric_all[vf].pkey);
    }

    if (sm_config.sm_debug_vf)
    {
        // TODO do this for all (maybe just 2,4,8,9) VLs
        // TODO do in a loop
        // Dump SC to SL
        IB_LOG_INFINI_INFO_FMT(__func__, "SCSL = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SCtoSL[0], sm_SCtoSL[1], sm_SCtoSL[2], sm_SCtoSL[3],
                               sm_SCtoSL[4], sm_SCtoSL[5], sm_SCtoSL[6], sm_SCtoSL[7]);

        IB_LOG_INFINI_INFO_FMT(__func__, "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SCtoSL[8], sm_SCtoSL[9], sm_SCtoSL[10], sm_SCtoSL[11],
                               sm_SCtoSL[12], sm_SCtoSL[13], sm_SCtoSL[14], sm_SCtoSL[15]);

        IB_LOG_INFINI_INFO_FMT(__func__, "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SCtoSL[16], sm_SCtoSL[17], sm_SCtoSL[18], sm_SCtoSL[19],
                               sm_SCtoSL[20], sm_SCtoSL[21], sm_SCtoSL[22], sm_SCtoSL[23]);

        IB_LOG_INFINI_INFO_FMT(__func__, "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SCtoSL[24], sm_SCtoSL[25], sm_SCtoSL[26], sm_SCtoSL[27],
                               sm_SCtoSL[28], sm_SCtoSL[29], sm_SCtoSL[30], sm_SCtoSL[31]);

        // Dump SL to SC
        IB_LOG_INFINI_INFO_FMT(__func__, "SLSC = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SLtoSC[0], sm_SLtoSC[1], sm_SLtoSC[2], sm_SLtoSC[3],
                               sm_SLtoSC[4], sm_SLtoSC[5], sm_SLtoSC[6], sm_SLtoSC[7]);

        IB_LOG_INFINI_INFO_FMT(__func__, "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SLtoSC[8], sm_SLtoSC[9], sm_SLtoSC[10], sm_SLtoSC[11],
                               sm_SLtoSC[12], sm_SLtoSC[13], sm_SLtoSC[14], sm_SLtoSC[15]);

        IB_LOG_INFINI_INFO_FMT(__func__, "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SLtoSC[16], sm_SLtoSC[17], sm_SLtoSC[18], sm_SLtoSC[19],
                               sm_SLtoSC[20], sm_SLtoSC[21], sm_SLtoSC[22], sm_SLtoSC[23]);

        IB_LOG_INFINI_INFO_FMT(__func__, "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                               sm_SLtoSC[24], sm_SLtoSC[25], sm_SLtoSC[26], sm_SLtoSC[27],
                               sm_SLtoSC[28], sm_SLtoSC[29], sm_SLtoSC[30], sm_SLtoSC[31]);

    }
}
