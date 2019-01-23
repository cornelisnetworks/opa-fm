/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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
//======================================================================//

#include <stdlib.h>				/* for qsort */
#include "os_g.h"
#include "ib_status.h"
#include "sm_l.h"
#include "sm_parallelsweep.h"
#include "sm_qos.h"

//
//			Parallel Sweep API functions
//
typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} QosWorkItem_t;

static QosWorkItem_t *
_qos_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	QosWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(QosWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_qos_workitem_free(QosWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

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
Status_t
sm_node_handleGetRespAggr(Node_t * nodep, Port_t * smaportp, STL_AGGREGATE * aggr, STL_AGGREGATE * end);

/**
	Update SCVLt/SCVLnt for a node (usu. switch) from @c aggr.  Assumes aggr->Data is in network order.
*/
static Status_t
_aggregate_to_scvl(Node_t * nodep, STL_AGGREGATE * aggr);

/**
	Update curArb.vlarb* fields from @c agrr.  Assumes aggr->Data is in network order.
*/
static Status_t
_aggregate_to_vlarb(Node_t * nodep, STL_AGGREGATE * aggr);


/**
	Update BufferControlTable  fields from @c agrr.  Assumes aggr->Data is in network order.
*/
static Status_t
_aggregate_to_bfrctrl(Node_t * nodep, STL_AGGREGATE * aggr);

/**
	Send all changes pending for the node.
*/
static Status_t
_node_syncSmaChanges(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t *nodep);

static Status_t
_port_syncSma_aggregate(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * smaportp);

static Status_t
_port_syncSma_solo(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * smaportp);

static void
_set_weight_multiplier(Qos_t * qos);

// The following is for uniform qos
static Qos_t *sm_Qos = NULL;

int sm_check_node_cache_valid(Node_t *);

static Status_t _populate_sctosl(RoutingModule_t *rm, const Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics, uint8_t *SLtoSC, uint8_t *SCtoSL);
static Status_t _setup_qos(RoutingModule_t *rm, Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics, const uint8_t *SLtoSC, const uint8_t *SCtoSL);
static Status_t _setup_qos_1vl(RoutingModule_t *rm, Qos_t * qos,
	VirtualFabrics_t *VirtualFabrics);

Status_t
sm_update_bw(RoutingModule_t *rm, VirtualFabrics_t *vfs)
{
	int bwInUse = 0;
	int i;

	// Bandwidth is now allocated in renderVirtualFabricsConfig.
	// Just sanity test it here.

	if (!vfs)  {
		return VSTATUS_OK;
	}

	// Sanity checks
	for (i = 0; i < vfs->number_of_qos_all; i++) {
		QosConfig_t *qos = &vfs->qos_all[i];
		if (!qos->priority) {
			DEBUG_ASSERT(qos->percent_bandwidth != UNDEFINED_XML8);
			bwInUse += qos->percent_bandwidth;
		}
	}
	DEBUG_ASSERT(bwInUse >= 0 && bwInUse <= 100);

	IB_LOG_INFINI_INFO_FMT(__func__, "QOSGroup Bandwidth Allocations : Total allocated bandwidth is %d%%",bwInUse);

	// Report QoSGroups with bandwidth allocated
	for (i = 0; i < vfs->number_of_qos_all; i++) {
		QosConfig_t *qos = &vfs->qos_all[i];
		char SLs[256] = { 0 };

		if (qos->resp_sl != qos->base_sl) {
			if (qos->mcast_sl != qos->base_sl) {
				snprintf(SLs,sizeof(SLs),"between BaseSL %d, RespSL %d, and MulticastSL %d",
					qos->base_sl, qos->resp_sl, qos->mcast_sl);
			} else {
				snprintf(SLs,sizeof(SLs),"between BaseSL %d, and RespSL %d",
					qos->base_sl, qos->resp_sl);
			}
		} else if (qos->mcast_sl != qos->base_sl) {
			snprintf(SLs,sizeof(SLs),"between BaseSL %d, and MulticastSL %d",
				qos->base_sl, qos->mcast_sl);
		} else {
			snprintf(SLs,sizeof(SLs),"on BaseSL %d", qos->base_sl);
		}
		if (qos->qos_enable) {
			if (qos->priority) {
				IB_LOG_INFINI_INFO_FMT( __func__,
					"High Priority QoS(%s) shared by %d VFs; HP QoS doesn't participate in Bandwidth limits.",qos->name,qos->num_vfs);
			} else {
				IB_LOG_INFINI_INFO_FMT( __func__,
					"QoS(%s) shared by %d VFs; Assigned %d%% Bandwidth %s",qos->name,qos->num_vfs,qos->percent_bandwidth,SLs);
			}
		} else {
			IB_LOG_INFINI_INFO_FMT( __func__,
				"Non QoS shared by %d VFs; Assigned %d%% Bandwidth %s",qos->num_vfs,qos->percent_bandwidth,SLs);
		}
	}
	return VSTATUS_OK;
}

static Qos_t*
_alloc_qos(void)
{
	Qos_t *qos = NULL;
	int i, j;

	vs_pool_alloc(&sm_pool, sizeof(Qos_t) * (STL_MAX_VLS + 1), (void*)&qos);
	if (!qos)
		IB_FATAL_ERROR("_alloc_qos: No memory for QoS structures, exiting.");

	for (i=1; i<=STL_MAX_VLS; i++) {
		memset(&qos[i], 0, sizeof(qos[i]));
		qos[i].numVLs = i;
		if (!bitset_init(&sm_pool, &qos[i].highPriorityVLs, STL_MAX_VLS)
		|| !bitset_init(&sm_pool, &qos[i].lowPriorityVLs, STL_MAX_VLS)) {
			IB_FATAL_ERROR("_alloc_qos: No memory for QoS structures, exiting.");
		}
		for (j=0; j< STL_MAX_SCS; j++) {
			qos[i].scvl.SCVLMap[j].VL=15; // Invalid VL
		}
	}
	return qos;
}

static void
_free_vlarbList(Qos_t *qos) {
	VlarbList_t *nextVlarb;		// Cached STL1 data
	while (qos->vlarbList) {
		nextVlarb = qos->vlarbList->next;
		vs_pool_free(&sm_pool, qos->vlarbList);
		qos->vlarbList = nextVlarb;
	}

}

static void
_free_qos(Qos_t* qos)
{
	int i, j;

	if (!qos) return;

	for (i=1; i<=STL_MAX_VLS; i++) {
		bitset_free(&qos[i].highPriorityVLs);
		bitset_free(&qos[i].lowPriorityVLs);
		_free_vlarbList(&qos[i]);
		for (j = 0; j < STL_MAX_VLS; j++){
			bitset_free(&qos[i].vlvf.vf[j]);
		}
	}
	(void) vs_pool_free(&sm_pool, qos);

	return;
}

static void
_install_qos(Qos_t *qos)
{
	Qos_t *tmpQos = sm_Qos;
	sm_Qos = qos;
	_free_qos(tmpQos);
}

static Status_t
_assign_scs_to_sls(RoutingModule_t *rm, VirtualFabrics_t *vfs, int fixed)
{
	Status_t ret = VSTATUS_OK;
	uint8_t SLtoSC[STL_MAX_SLS];
	uint8_t SCtoSL[STL_MAX_SCS];
	Qos_t *qos = NULL;
	int i;

	if (rm->funcs.min_vls() > rm->funcs.max_vls()) {
		IB_LOG_ERROR_FMT(__func__, "Configuration cannot be mapped. MinSupportedVLs (%d) cannot exceed MaxSupportedVLs (%d).",
			rm->funcs.min_vls(), rm->funcs.max_vls());
		return VSTATUS_BAD;
	}

	qos = _alloc_qos();

	// Flag the maps as invalid
	memset(SLtoSC, 15, sizeof(SLtoSC));
	memset(SCtoSL, 0xff, sizeof(SCtoSL));

	// Populate the SCVL maps for each QOS level
	for (i=1; i <= rm->funcs.max_vls(); i++)
		sm_fill_SCVLMap(&qos[i]);

	if (fixed) {
		// Will ensure all nodes in fabric have the same SL-SC-VL mapping, even if
		// they support a different number of VLs. Will perform calculations
		// based on the minimum number of VLs.
		ret = _populate_sctosl(rm, &qos[sm_config.max_fixed_vls], vfs, SLtoSC, SCtoSL);
	}
	else {
		// Will allow nodes in fabric to have varying SL-SC-VL mapping depending on
		// how many VLs they support. Will perform calculations based on the
		// maximum number of VLs.
		ret = _populate_sctosl(rm, &qos[rm->funcs.max_vls()], vfs, SLtoSC, SCtoSL);
	}

	if (ret != VSTATUS_OK) {
		_free_qos(qos);
		return ret;
	}

	// 1 VL is a special case, always set it up
	if(rm->funcs.min_vls() > 1) {
		ret = _setup_qos_1vl(rm, &qos[1], vfs);
		if (ret != VSTATUS_OK) {
			_free_qos(qos);
			return ret;
		}
	}

	// Set Qos for all supported number of VLs
	int startVl = sm_config.allow_mixed_vls ? 1 : rm->funcs.min_vls();
	for (i=startVl; i <= rm->funcs.max_vls(); i++) {
		ret = _setup_qos(rm, &qos[i], vfs, SLtoSC, SCtoSL);

		if (ret != VSTATUS_OK) {
			_free_qos(qos);
			return ret;
		}
	}

	memcpy(sm_SLtoSC, SLtoSC, sizeof(SLtoSC));
	memcpy(sm_SCtoSL, SCtoSL, sizeof(SCtoSL));

	_install_qos(qos);
	return VSTATUS_OK;
}

Status_t
sm_assign_scs_to_sls_FixedMap(RoutingModule_t *rm, VirtualFabrics_t *vfs)
{
	return _assign_scs_to_sls(rm, vfs, 1);
}

Status_t
sm_assign_scs_to_sls_NonFixedMap(RoutingModule_t *rm, VirtualFabrics_t *vfs)
{
	return _assign_scs_to_sls(rm, vfs, 0);
}

/*
 * Used to round VL bandwidths up/off to nearest multiple of 5. This is
 * for VLArb SMAs. All this does is set the weightMultiplier in the
 * origQos. It does not change the bw values in the origQos.
 */
static void
_roundVLBandwidths(Qos_t * origQos)
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

	_set_weight_multiplier(&qos);
	origQos->weightMultiplier = qos.weightMultiplier;
}

static void
_dbg_print_qos(Qos_t * qos)
{
    if (sm_config.sm_debug_vf) {
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

// starting with the SC AFTER starting_sc, return an SC that maps to vl.
// (to find the first SC that maps to VL, pass -1 as the starting_sc)
static inline int
_find_next_sc(const Qos_t *qos, int vl, int starting_sc)
{
	int sc;

	for (sc=starting_sc+1; sc < STL_MAX_SCS; sc++) {
		if (qos->scvl.SCVLMap[sc].VL == vl) break;
	}

	return sc;
}

// Returns the Nth SC mapped to an SL, where N is between 1 and numSCs.
// Returns STL_MAX_SCS if we run off the end of the table.
static inline int
_find_nth_sc(const uint8_t *SLtoSC, const uint8_t *SCtoSL, int sl, int n)
{
	int sc, i=0;

	for (sc = SLtoSC[sl]; sc < STL_MAX_SCS; sc++)
	{
		if (SCtoSL[sc] == sl) i++;
		if (i == n) break;
	}

	return sc;
}

// Assign VLs starting at the bottom of the free list.
// Returns lowest assigned VL, or -1 on error.
static inline int
_assign_low_vls(bitset_t *freeVLs, int vls_needed, int sl,
	RoutingModule_t *rm, const Qos_t *qos, uint8_t *SLtoSC, uint8_t *SCtoSL)
{
	int vl, sc, i, first_sc;

	vl = bitset_find_first_one(freeVLs);
	if (vl == -1) {
		IB_LOG_ERROR_FMT(__func__, "Configuration cannot be mapped. Requires more than %d VLs.", qos->activeVLs);
		return vl;
	}
	// Mark off sequential VLs
	for (i=0, first_sc=-1; i < vls_needed; i++) {
		// Ensure we have enough consecutive VLs free
		if ((vl+i) > rm->funcs.max_vls()) {
			IB_LOG_ERROR_FMT(__func__, "Assigning %d VL(s) to SL %d starting at VL %d failed. VL %d above maximum %d",
				vls_needed, sl, vl, vl+i, rm->funcs.max_vls());
			vl = -1;
			break;
		}
		if (!bitset_test(freeVLs, vl+i)){
			IB_LOG_ERROR_FMT(__func__, "Assigning %d VL(s) to SL %d starting at VL %d failed. VL %d already in use",
				vls_needed, sl, vl, vl+i);
			vl = -1;
			break;
		}
		bitset_clear(freeVLs, vl+i);
		sc = _find_next_sc(qos, vl+i, -1);
		if (i == 0)
			first_sc = sc;

		// Check that the SCs are sequential (they should be, but just to be safe)
		if (sc != (first_sc+i)) {
			IB_LOG_ERROR_FMT(__func__, "SCs not linearly increasing between VL %d (SC %d) and VL %d (SC %d)",
				vl, first_sc, vl+i, sc);
			vl = -1;
			break;
		}
		if (sc >= STL_MAX_SCS) {
			IB_LOG_ERROR_FMT(__func__, "Not enough SCs for SL %d to VL %d", sl, vl+i);
			vl = -1;
			break;
		}
		SCtoSL[sc] = sl;
		if (SLtoSC[sl] == 15) {
			// SL to SC map only contains the first SC of an SL.
			SLtoSC[sl] = sc;
		}
	}

	return vl;
}

// Assign VLs starting at the top of the free list.
// Returns lowest assigned VL, or -1 on error.
static inline int
_assign_high_vls(bitset_t *freeVLs, int vls_needed, int sl,
	RoutingModule_t *rm, const Qos_t *qos, uint8_t *SLtoSC, uint8_t *SCtoSL)
{
	int vl, sc, i, first_sc;

	// find free high VL
	for (vl=rm->funcs.max_vls(); vl >= 0; vl--) {
		if (bitset_test(freeVLs, vl)) break;
	}
	if (vl == -1) {
		IB_LOG_ERROR_FMT(__func__, "Configuration cannot be mapped. Requires more than %d VLs.",qos->activeVLs);
		return vl;
	}
	// Mark off sequential VLs counting down from the first
	for (i=0; i < vls_needed; i++) {
		// Ensure we have enough consecutive VLs free
		if ((vl-i) < 0) {
			IB_LOG_ERROR_FMT(__func__, "Assigning %d VL(s) to SL %d starting at VL %d failed. VL %d below 0.",
				vls_needed, sl, vl, vl-i);
			vl = -1;
			break;
		}
		if (!bitset_test(freeVLs, vl-i)){
			IB_LOG_ERROR_FMT(__func__, "Assigning %d VL(s) to SL %d starting at VL %d failed. VL %d already in use",
				vls_needed, sl, vl, vl-i);
			vl = -1;
			break;
		}
		bitset_clear(freeVLs, vl-i);
		sc = _find_next_sc(qos, vl-i, -1);
		if (i == 0)
			first_sc = sc;

		// Check that the SCs are sequential (they should be, but just to be safe)
		if (sc != (first_sc-i)) {
			IB_LOG_ERROR_FMT(__func__, "SCs not linearly decreasing between VL %d (SC %d) and VL %d (SC %d)",
				vl, first_sc, vl-i, sc);
			vl = -1;
			break;
		}
		if (sc >= STL_MAX_SCS) {
			IB_LOG_ERROR_FMT(__func__, "Not enough SCs for Base SL to VL %d", vl+i);
			vl = -1;
			break;
		}
		SCtoSL[sc] = sl;
		if (SLtoSC[sl] == 15) {
			// SL to SC map only contains the first SC of an SL.
			SLtoSC[sl] = sc;
		}
	}

	if (vl != -1)
		vl = vl - (vls_needed-1); // return lowest VL
	return vl;
}

// Returns 0 on success, -1 on failure.
// Of the VLs available for sharing, start at the back of the list.
// Since Multicast only uses 1 VL currently, they'll always share the same one.
static inline int
_assign_shared_vls(int vls_available, int vls_needed, int starting_vl, int starting_sl,
	int sl, const Qos_t *qos, uint8_t *SLtoSC, uint8_t *SCtoSL)
{
	int vl, sc, starting_sc, first_sc;
	int i=0;

	// When sharing a VL, we want to use a different SC that maps to the same VL.
	starting_sc = _find_nth_sc(SLtoSC, SCtoSL, starting_sl, vls_available);
	if (starting_sc == STL_MAX_SCS) {
		IB_LOG_ERROR_FMT(__func__, "Could not find highest SC for SL %d", starting_sl);
		return -1;
	}

	for (i=0; i < vls_needed; i++) {
		// Start from the top of the list
		vl = starting_vl + (vls_available - 1) - i;
		// Use a difference SC than other SL
		sc = _find_next_sc(qos, vl, starting_sc);
		if (sc >= STL_MAX_SCS) {
			IB_LOG_ERROR_FMT(__func__, "Not enough SCs to oversubscribe VL %d", vl);
			return -1;
		}

		// Check that the SCs are sequential (they should be, but just to be safe)
		if (i == 0)
			first_sc = sc;
		if (sc != (first_sc-i)) {
			IB_LOG_ERROR_FMT(__func__, "SCs not linearly decreasing between VL %d (SC %d) and VL %d (SC %d)",
				starting_vl + (vls_available - 1), first_sc, vl, sc);
			vl = -1;
			break;
		}
		SCtoSL[sc] = sl;
		if (SLtoSC[sl] == 15) {
			// SL to SC map only contains the first SC of an SL.
			SLtoSC[sl] = sc;
		}
	}

	return 0;
}

/*
 * Maps SCs to SLs. Requires that the SC to VL map is sequential.
 * (i.e., that if SC0 maps to VL0, then SC1 to VL1 and so on) In addition,
 * the function prohibits oversubscribing VLs unless specifically allowed
 * by the routing algorithm.
 */
static Status_t
_populate_sctosl(RoutingModule_t *rm, const Qos_t *qos,
	VirtualFabrics_t *VirtualFabrics, uint8_t *SLtoSC, uint8_t *SCtoSL)
{
	// Populate the SLtoSC and SCtoSL
	int qs;
	int base_vl=-1, resp_vl=-1, mcast_vl=-1;
	int prev_base_sl=-1, prev_base_vl=-1;
	int prev_resp_sl=-1, prev_resp_vl=-1;
	int prev_mcast_sl=-1, prev_mcast_vl=-1;
	int vls_needed=0, base_vls_needed=0, resp_vls_needed=0, mcast_vls_needed=0;
	int base_oversub_fact=0, resp_oversub_fact=0, mcast_oversub_fact=0;
	bitset_t mappedSLs;
	bitset_t freeVLs;
	Status_t ret = VSTATUS_BAD;

	if (!bitset_init(&sm_pool, &mappedSLs, STL_MAX_SLS)
	||  !bitset_init(&sm_pool, &freeVLs, qos->activeVLs)) {
		IB_FATAL_ERROR("_populate_sctosl: No memory for QoS setup, exiting.");
	}

	// Mark all VLs as free, except VL15, which is reserved for SMA traffic.
	bitset_set_all(&freeVLs);
	if (qos->activeVLs > 15)
		bitset_clear(&freeVLs, 15);

	// TODO: This is mostly duplicated in sm_routing_func_assign_sls. The two
	// should be converged.

	// For every QOSGroup, determine number of VLs needed for base, resp, and
	// multicast.  if overlay_mcast and numSCs for BaseSL > 1, multicast will
	// share one of base VLs. if mcast_isolation, the MulticastSL for each QoS
	// Group wil be assigned to something unique of the BaseSL, but different
	// for each QoS Group. We will want to map those different Mcast SLs to the
	// same SCs/VLs if routing allows it.
	for (qs=0; qs < VirtualFabrics->number_of_qos_all; qs++) {
		QosConfig_t *qosp = &VirtualFabrics->qos_all[qs];
		base_oversub_fact = rm->funcs.oversubscribe_factor(qosp->base_sl, 0);
		resp_oversub_fact = rm->funcs.oversubscribe_factor(qosp->resp_sl, 0);
		mcast_oversub_fact = rm->funcs.oversubscribe_factor(qosp->mcast_sl, 1);

		bitset_set(&mappedSLs, qosp->base_sl);
		if (base_vls_needed == 0) {
			base_vls_needed = rm->funcs.num_routing_scs(qosp->base_sl, 0);
			vls_needed += base_vls_needed;
		}
		else {
			vls_needed += (rm->funcs.num_routing_scs(qosp->base_sl, 0) - base_oversub_fact);
		}

		if (!bitset_test(&mappedSLs, qosp->resp_sl)) {
			bitset_set(&mappedSLs, qosp->resp_sl);
			if (resp_vls_needed == 0) {
				resp_vls_needed = rm->funcs.num_routing_scs(qosp->resp_sl, 0);
				vls_needed += resp_vls_needed;
			}
			else {
				vls_needed += (rm->funcs.num_routing_scs(qosp->resp_sl, 0) - resp_oversub_fact);
			}
		}

		if (!bitset_test(&mappedSLs, qosp->mcast_sl)) {
			bitset_set(&mappedSLs, qosp->mcast_sl);
			if (rm->funcs.overlay_mcast()) {
				// No new VLs need for Multicast, as it will share Base VLs.
				// Just ensure that there are enough.
				if (base_vls_needed <= rm->funcs.num_routing_scs(qosp->mcast_sl, 1)) {
					IB_LOG_ERROR_FMT(__func__, "Cannot overlay %d multicast VL(s) onto %d base VL(s)",
						rm->funcs.num_routing_scs(qosp->mcast_sl, 1), base_vls_needed);
					goto fail;
				}
			}
			else if (mcast_vls_needed == 0) {
				mcast_vls_needed = rm->funcs.num_routing_scs(qosp->mcast_sl, 1);
				vls_needed += mcast_vls_needed;
			}
			else {
				vls_needed += (rm->funcs.num_routing_scs(qosp->mcast_sl, 1) - mcast_oversub_fact);
			}
		}
	}

	bitset_clear_all(&mappedSLs);

	if (vls_needed > qos->activeVLs) {
		IB_LOG_ERROR_FMT(__func__, "Configuration requires at least %d VLs, which exceeds the number supported (%d).",
			vls_needed, qos->activeVLs);
		goto fail;
	}

	sm_needed_vls = vls_needed;

	if (sm_config.sm_debug_vf)
		IB_LOG_INFINI_INFO_FMT(__func__, "VLs needed: %d", vls_needed);

	vls_needed = 0;
	// Assign SCs to SLs. Must map all QoS Groups, even those without any Active VFs.
	for (qs=0; qs < VirtualFabrics->number_of_qos_all; qs++) {
		QosConfig_t *qosp = &VirtualFabrics->qos_all[qs];
		base_vls_needed = rm->funcs.num_routing_scs(qosp->base_sl, 0);
		resp_vls_needed = rm->funcs.num_routing_scs(qosp->resp_sl, 0);
		mcast_vls_needed = rm->funcs.num_routing_scs(qosp->mcast_sl, 1);
		vls_needed = base_vls_needed + resp_vls_needed + mcast_vls_needed;
		base_oversub_fact = rm->funcs.oversubscribe_factor(qosp->base_sl, 0);
		resp_oversub_fact = rm->funcs.oversubscribe_factor(qosp->resp_sl, 0);
		mcast_oversub_fact = rm->funcs.oversubscribe_factor(qosp->mcast_sl, 1);

		// First assign the base SL to VL(s)
		bitset_set(&mappedSLs, qosp->base_sl);

		if (prev_base_vl > -1 && base_oversub_fact > 0 && vls_needed > bitset_nset(&freeVLs)) {
			// Not enough VLs left, share the last n Base VLs of the previous QOS Group
			base_vl = _assign_low_vls(&freeVLs, base_vls_needed - base_oversub_fact, qosp->base_sl, rm, qos, SLtoSC, SCtoSL);
			if (_assign_shared_vls(base_vls_needed, base_oversub_fact, prev_base_vl, prev_base_sl, qosp->base_sl, qos, SLtoSC, SCtoSL) == -1)
				goto fail;
			IB_LOG_WARN_FMT(__func__, "Oversubscribed %d VL(s), starting at VL%d and decreasing, between Base SL %d and %d",
				base_oversub_fact, prev_base_vl+(base_vls_needed-1), prev_base_sl, qosp->base_sl);
		}
		else {
			base_vl = _assign_low_vls(&freeVLs, base_vls_needed, qosp->base_sl, rm, qos, SLtoSC, SCtoSL);
		}
		if (base_vl == -1)
			goto fail;
		prev_base_vl = base_vl;
		prev_base_sl = qosp->base_sl;

		// Next assign resp SL to VL(s)
		if (!bitset_test(&mappedSLs, qosp->resp_sl)){
			bitset_set(&mappedSLs, qosp->resp_sl);
			if (prev_resp_vl > -1 && resp_oversub_fact > 0 && vls_needed > bitset_nset(&freeVLs)) {
				// Not enough VLs left, share the last n Resp VLs of the previous QOS Group
				resp_vl = _assign_low_vls(&freeVLs, resp_vls_needed - resp_oversub_fact, qosp->resp_sl, rm, qos, SLtoSC, SCtoSL);
				if (_assign_shared_vls(resp_vls_needed, resp_oversub_fact, prev_resp_vl, prev_resp_sl, qosp->resp_sl, qos, SLtoSC, SCtoSL) == -1)
					goto fail;
				IB_LOG_WARN_FMT(__func__, "Oversubscribed %d VL(s), starting at VL%d and decreasing, between Resp SL %d and %d",
					resp_oversub_fact, prev_resp_vl+(resp_vls_needed-1), prev_resp_sl, qosp->resp_sl);
			}
			else {
				resp_vl = _assign_low_vls(&freeVLs, resp_vls_needed, qosp->resp_sl, rm, qos, SLtoSC, SCtoSL);
			}
			if (resp_vl == -1)
				goto fail;
			prev_resp_vl = resp_vl;
			prev_resp_sl = qosp->resp_sl;
		}

		// Finally, assign multicast SL to VL(s)
		if (bitset_test(&mappedSLs, qosp->mcast_sl))
			continue;

		bitset_set(&mappedSLs, qosp->mcast_sl);
		if (rm->funcs.overlay_mcast()) {
			if (_assign_shared_vls(base_vls_needed, mcast_vls_needed, base_vl, qosp->base_sl, qosp->mcast_sl, qos, SLtoSC, SCtoSL) == -1)
				goto fail;
		}
		else{
			if (prev_mcast_vl > -1 && mcast_oversub_fact > 0 && vls_needed > bitset_nset(&freeVLs)) {
				// Not enough VLs left, share the last n Mcast VLs of the previous QOS Group
				if (rm->funcs.mcast_isolation_required()) {
					mcast_vl = _assign_high_vls(&freeVLs, mcast_vls_needed - mcast_oversub_fact, qosp->mcast_sl, rm, qos, SLtoSC, SCtoSL);
				}
				else {
					// User specified Multicast SL but it isn't required, keep base assignment
					mcast_vl = _assign_low_vls(&freeVLs, mcast_vls_needed - mcast_oversub_fact, qosp->mcast_sl, rm, qos, SLtoSC, SCtoSL);
				}
				if (_assign_shared_vls(mcast_vls_needed, mcast_oversub_fact, prev_mcast_vl, prev_mcast_sl, qosp->mcast_sl, qos, SLtoSC, SCtoSL) == -1)
					goto fail;
				IB_LOG_WARN_FMT(__func__, "Oversubscribed %d VL(s), starting at VL%d and decreasing, between Multicast SL %d and %d",
					mcast_oversub_fact, prev_mcast_vl+(mcast_vls_needed-1), prev_mcast_sl, qosp->mcast_sl);
			}
			else {
				if (rm->funcs.mcast_isolation_required()) {
					mcast_vl = _assign_high_vls(&freeVLs, mcast_vls_needed, qosp->mcast_sl, rm, qos, SLtoSC, SCtoSL);
				}
				else{
					// User specified Multicast SL but it isn't required, keep base assignment
					mcast_vl = _assign_low_vls(&freeVLs, mcast_vls_needed, qosp->mcast_sl, rm, qos, SLtoSC, SCtoSL);
				}
			}

			if (mcast_vl == -1)
				goto fail;
			prev_mcast_vl = mcast_vl;
			prev_mcast_sl = qosp->mcast_sl;
		}
	}

	ret = VSTATUS_OK;

fail:
	bitset_free(&freeVLs);
	bitset_free(&mappedSLs);
	return ret;
}

static void
_divide_bw_up(RoutingModule_t *rm, Qos_t *qos, int bw, int base_sl, int resp_sl,
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

static Status_t
_map_sl_to_qos(RoutingModule_t *rm, Qos_t * qos, QosConfig_t *qosConfig, int qosIdx,
	const uint8_t *SLtoSC, const uint8_t *SCtoSL, int sl, boolean mcast_sl)
{
	Status_t ret = VSTATUS_BAD;
	bitset_t mappedVLs;
	int sc, vl, i, numSCs;

	assert(sl >= 0 && sl < STL_MAX_SLS);

	if (!bitset_init(&sm_pool, &mappedVLs, qos->numVLs)) {
		IB_FATAL_ERROR(add_quotes(__func__)": No memory for QoS setup, exiting.");
	}

	numSCs = rm->funcs.num_routing_scs(sl, mcast_sl);

	for (i = 1; i <= numSCs; i++) {
		sc = _find_nth_sc(SLtoSC, SCtoSL, sl, i);
		if (sc == 15 || sc >= STL_MAX_SCS) {
			IB_LOG_ERROR_FMT(__func__, "Missing SC:SL Mapping:"
				"SL=%02d needs %d SCs, could not find the %d SC.",
				sl, numSCs, i);
			goto fail;
		}

		vl = qos->scvl.SCVLMap[sc].VL;
		if (vl == 15 || vl >= qos->numVLs) {
			IB_LOG_ERROR_FMT(__func__, "Unexpected SC:VL Mapping:"
				"SL=%02d SC=%02d VL=%02d", sl, sc+i, vl);
			goto fail;
		}

		// If a VL is shared, keep the first settings
		if (!bitset_test(&mappedVLs, vl)) {
			if (qosConfig->priority) {
				bitset_set(&qos->highPriorityVLs, vl);
				qos->vlBandwidth.highPriority[vl] = 1;
			} else {
				bitset_set(&qos->lowPriorityVLs, vl);
			}
			qos->vlBandwidth.qos[vl] = qosIdx;
			bitset_set(&mappedVLs, vl);
		}
	}

	ret = VSTATUS_OK;
fail:
	bitset_free(&mappedVLs);
	return ret;
}

static Status_t
_setup_qos(RoutingModule_t *rm, Qos_t * qos, VirtualFabrics_t *VirtualFabrics,
	const uint8_t *SLtoSC, const uint8_t *SCtoSL)
{
	// Assign QOS settings to the SLs.
	int vl, vf, i;

	for (vl = 0; vl < STL_MAX_VLS; vl++) {
		if (!bitset_init(&sm_pool, &qos->vlvf.vf[vl], MAX_VFABRICS)) {
			IB_FATAL_ERROR("_setup_qos: Out of memory, exiting.");
		}
	}
	memset(qos->vlBandwidth.qos, 0xFF, sizeof(qos->vlBandwidth.qos));
	qos->vlBandwidth.has_qos = TRUE;

	for (i=0; i < VirtualFabrics->number_of_qos_all; i++) {
		QosConfig_t *qosConfig = &VirtualFabrics->qos_all[i];
		if (!qosConfig->num_vfs) continue;

		_map_sl_to_qos(rm, qos, qosConfig, i, SLtoSC, SCtoSL, qosConfig->base_sl, FALSE);
		if (qosConfig->base_sl != qosConfig->resp_sl)
			_map_sl_to_qos(rm, qos, qosConfig, i, SLtoSC, SCtoSL, qosConfig->resp_sl, FALSE);
		if (qosConfig->base_sl != qosConfig->mcast_sl)
			_map_sl_to_qos(rm, qos, qosConfig, i, SLtoSC,  SCtoSL, qosConfig->mcast_sl, TRUE);

	}
	for (vl = 0; vl < STL_MAX_VLS; vl++) {
		for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
			if (qos->vlBandwidth.qos[vl] == VirtualFabrics->v_fabric_all[vf].qos_index) {
				bitset_set(&qos->vlvf.vf[vl], vf);
			}
		}
	}

	for (i=0; i < VirtualFabrics->number_of_qos_all; i++) {
		QosConfig_t *qosConfig = &VirtualFabrics->qos_all[i];

		if (qosConfig->priority) continue;

		// Qos LowPriority
		_divide_bw_up(rm, qos, qosConfig->percent_bandwidth, qosConfig->base_sl,
			qosConfig->resp_sl, qosConfig->mcast_sl, SLtoSC);
	}

	_dbg_print_qos(qos);
	return VSTATUS_OK;
}

// Special QOS level where all non-SMA traffic goes over VL0.
static Status_t
_setup_qos_1vl(RoutingModule_t *rm, Qos_t * qos, VirtualFabrics_t *VirtualFabrics)
{
	int vf;

	bitset_set(&qos->lowPriorityVLs, 0);
	memset(qos->vlBandwidth.qos, 0xFF, sizeof(qos->vlBandwidth.qos));
	qos->vlBandwidth.has_qos = FALSE;

	if (!bitset_init(&sm_pool, &qos->vlvf.vf[0], MAX_VFABRICS)) {
			IB_FATAL_ERROR("_setup_qos_1vl: Out of memory, exiting.");
	}

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;
		bitset_set(&qos->vlvf.vf[0], vf);
	}
	qos->vlBandwidth.bw[0] = 100;

	_dbg_print_qos(qos);
	return VSTATUS_OK;
}

void
sm_destroy_qos(void)
{
	_free_qos(sm_Qos);
	sm_Qos = NULL;
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

//	Note that this function should not do I/O, does not have the context or
//	handle.
static Status_t
_initialize_Switch_SLSCMap(Topology_t * topop,
	Node_t * switchp, STL_SLSCMAP * slscmapp)
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
		IB_LOG_WARNRC("Failed to get SLSC map from routing algorithm;  rc:",
			status);
		IB_EXIT(__func__, status);
		return status;
	}

	// Compare the port's current SLSC map against what the topology says it
	// should be. If they're different, send the new one.
	if (!swportp->portData->current.slsc ||
		memcmp((void *)curSlsc, (void *)slscmapp, sizeof(*slscmapp)) != 0 ||
		sm_config.forceAttributeRewrite) {
		swportp->portData->dirty.slsc = 1;
	}

	IB_EXIT(__func__, status);
	return (status);
}

// Note that this function should not do I/O and does not have the context or
// ib handle.
static Status_t
_initialize_Switch_SCSLMap(Topology_t * topop, Node_t * switchp,
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
		swportp->portData->dirty.scsl = 1;
	}

	IB_EXIT(__func__, status);
	return (status);
}

// FIXME: This function returns success even when it fails.
static Status_t
_write_gen1_scsc(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * switchp, STL_LID dlid, int numScscBlocks,
	STL_SCSC_MULTISET *scsc)
{
	uint32_t		amod, egressAmod;
	Status_t		status=VSTATUS_OK;
	Port_t			*swportp;
	STL_PORTMASK	ingressPorts[STL_MAX_PORTMASK];
	int 			e, i, b, numBlocks;
	int 			ingress=0, lastIngress=0;
	int 			egress=0, lastEgress=0;
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

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

				psc_unlock(psc);
				status = SM_Set_SCSC(fd, amod, &addr, (STL_SCSCMAP*)&scsc[b].SCSCMap, sm_config.mkey);
				psc_lock(psc);
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
_initialize_Switch_SCSCMap(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * switchp)
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
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

	if (switchp->switchInfo.CapabilityMask.s.IsExtendedSCSCSupported &&
		topop->routingModule->funcs.extended_scsc_in_use())
		setCnt = 2;

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

		if (switchp->switchInfo.CapabilityMask.s.IsExtendedSCSCSupported) {
			int b;
			for (b=0; b<numBlocks; b+= STL_NUM_SCSC_MULTI_BLOCKS_PER_LRSMP) {
				int blocks = MIN(STL_NUM_SCSC_MULTI_BLOCKS_PER_LRSMP, numBlocks-b);
				uint32_t amod = (blocks<<24) | (s<<20);

				psc_unlock(psc);
				status = SM_Set_SCSCMultiSet(fd, amod, &addr, &scsc[b], sm_config.mkey);
				psc_lock(psc);

				if (status != VSTATUS_OK) {
					IB_LOG_WARN_FMT(__func__, "Failed to set SCSC Map for switch %s nodeGuid " FMT_U64,
						sm_nodeDescString(switchp), switchp->nodeInfo.NodeGUID);
				} else {
					int b2;
					// update cached data on success
					for (b2=b; b2<blocks; b2++) {
						int i,j;
						for (i=1; i<=switchp->nodeInfo.NumPorts; i++) {
							if (!StlIsPortInPortMask(scsc[b2].IngressPortMask, i)) continue;
							swportp = sm_get_port(switchp, i);
							if (!sm_valid_port(swportp)) continue;
							for (j=1; j<=switchp->nodeInfo.NumPorts; j++) {
								if (!StlIsPortInPortMask(scsc[b2].EgressPortMask, j)) continue;
								sm_addPortDataSCSCMap(swportp, j-1, s, &scsc[b2].SCSCMap);
							}
							swportp->portData->current.scsc = 1;
						}
					}
				}
			}

		} else {
			// Use STL1 SMP format
			status = _write_gen1_scsc(psc, fd, topop, switchp, dlid, numBlocks,
				scsc);
		}

		(void) vs_pool_free(&sm_pool, scsc);
	}

	return status;
}

static Status_t
_initialize_Switch_SCVLMaps(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * switchp)
{
	uint32_t amod = 0;
	uint8_t synchModeGen1 = 1;
	Status_t status = VSTATUS_OK;
	Node_t *neighborNodep = NULL;
	Port_t * out_portp,*neighborPortp, *swportp = NULL, *neighborSwPortp = NULL;
	STL_SCVLMAP scvlmap;
	int sentSCVLt = 0;
	int doAll = switchp->uniformVL;
	boolean isMultiPort = FALSE;
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
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, swportp->portData->lid);

	// for every switch, we need to setup the SC->VL_r mapping.  This is
	// shared across ports.
	if (swportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) {
		amod = (1 << 24) | (1 << 8) | 0;   // 1 block, all ingress & cport

		STL_SCVLMAP * curScvl = &swportp->portData->scvlrMap;

		status = topop->routingModule->funcs.select_scvlr_map(topop, switchp->vlCap, &scvlmap);
		if (status != VSTATUS_OK) {
			IB_LOG_WARNRC("_initialize_Switch_SCVLMaps: Failed to get SCVL "
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
				psc_unlock(psc);
				status = SM_Set_SCVLrMap(fd, amod, &addr, &scvlmap, sm_config.mkey);
				psc_lock(psc);

				if (status != VSTATUS_OK) {
					IB_LOG_WARN_FMT("_initialize_Switch_SCVLMaps",
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

	_initialize_Switch_SCSCMap(psc, fd, topop, switchp);

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
				sm_nodeDescString(neighborNodep), neighborNodep->nodeInfo.NodeGUID,
				neighborPortp->index);
		}

		// the SCtoVL_nt table must be configured consistently with the SCtoVL_t
		// table at its neighbor. When the link state is Init the SM shall have
		// the responsibility of updating both the SCtoVL_t table and the
		// neighbor's SCtoVL_nt table ("synchronous" update). When the link
		// state is Armed or Active, the SM shall update the SCtoVL_t table; and
		// the SMAs of both ports shall have the responsibility of updating
		// the neighbor's SCtoVL_nt table ("asynchronous" update of SCtoVL_t only).
		//
		// section 9.7.14.3 of the STL spec, "Optional Mechanism for Changes
		// While in LinkArmed or LinkActive" The mechanisms in this section are
		// only available when the ports on both side of a link report
		// IsAsyncSC2VLSupported. If either port reports it does not have this
		// capability, the FM shall not attempt to perform the changes outlined
		// in this section.
		//
		// It is anticipated that STL Gen1 will not support this capability.
		// Switch port 0 is a special case (no neighbor and SCtoVL_nt not
		// supported), so SCtoVL_t can be changed at any time.
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
		isMultiPort = (doAll && (out_portp->index > 0));
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
					if (isMultiPort)
						amod |= (1 << 8);

					SMP_ADDR_SET_LR(&addr, sm_lid, swportp->portData->lid);
					psc_unlock(psc);
					status = SM_Set_SCVLtMap(fd, amod, &addr, &scvlmap, sm_config.mkey);
					psc_lock(psc);

					out_portp->portData->current.scvlt = (status == VSTATUS_OK);
					if (status != VSTATUS_OK) {
						IB_LOG_WARN_FMT(__func__,
							"Failed to set SCVL_t Map for node %s nodeGuid " FMT_U64
							" output port %d (%s)", sm_nodeDescString(switchp),
							switchp->nodeInfo.NodeGUID, out_portp->index, isMultiPort ? "multi-port" : "single-port");
						if(status == VSTATUS_TIMEOUT || isMultiPort) {
							status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
							goto fail;
						}
						else {
							sm_mark_link_down(sm_topop, out_portp);
							status = VSTATUS_OK;
						}
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
		// initialize the SCVL_nt map of the neighbor port.  When the link state
		// is Armed or Active, the SMAs of both ports shall have the
		// responsibility of updating the neighbor's SCtoVL_nt table
		// ("asynchronous" update of SCtoVL_t only).
		amod = (1 << 24) | neighborPortp->index;   // 1 block, sych update

		STL_SCVLMAP * curScvlnt = &neighborPortp->portData->scvlntMap;

		// SCVLnt not supported on switch port 0
		if (out_portp->index > 0 &&
			(!neighborPortp->portData->current.scvlnt ||
			 memcmp((void *)curScvlnt, (void *)&scvlmap, sizeof(scvlmap)) != 0)) {
			if (synchModeGen1) {
				SMP_ADDR_SET_LR(&addr, sm_lid, (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? neighborSwPortp->portData->lid : neighborPortp->portData->lid);
				psc_unlock(psc);
				status = SM_Set_SCVLntMap(fd, amod, &addr, &scvlmap,
					sm_config.mkey);
				psc_lock(psc);

				neighborPortp->portData->current.scvlnt = (status == VSTATUS_OK);
				if (status != VSTATUS_OK) {
					IB_LOG_WARN_FMT(__func__,
						"Failed to set SCVL_nt Map for node %s nodeGuid " FMT_U64
						" output port %d", sm_nodeDescString(neighborNodep),
						neighborNodep->nodeInfo.NodeGUID, neighborPortp->index);
					sm_mark_link_down(sm_topop, neighborPortp);
					status = sm_popo_port_error(&sm_popo, sm_topop, swportp, status);
					if (status == VSTATUS_TIMEOUT_LIMIT)
						goto fail;
					else
						status = VSTATUS_OK; //reset status: switch did not fail, only neighbor
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

// Note this function shouldn't do I/O, does not have the context or handle.
static Status_t
_initialize_Node_Port_SLSCMap(Topology_t * topop, Node_t * nodep, Port_t * out_portp, STL_SLSCMAP * slscmapp)
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
		out_portp->portData->dirty.slsc = 1;
	}

	IB_EXIT(__func__, 0);
	return (status);
}

// Note this function shouldn't do I/O, does not have the context or handle.
static Status_t
_initialize_Node_Port_SCSLMap(Topology_t * topop, Node_t * nodep, Port_t * in_portp, STL_SCSLMAP * scslmapp)
{
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, topop, nodep, in_portp, 0);

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
		IB_LOG_WARNRC("_initialize_Node_Port_SCSLMap: Failed to get SCSL "
		  "map from routing algorithm; rc:", status);
		return status;
	}

	//
	// compare the port's current SCSL map against what the topology says it
	// should be. If they're different, send the new one.
	if (!in_portp->portData->current.scsl ||
		memcmp((void *)curScsl, (void *)scslmapp, sizeof(*scslmapp)) != 0 || sm_config.forceAttributeRewrite) {
		in_portp->portData->dirty.scsl = 1;
	}

	IB_EXIT(__func__, 0);
	return (status);
}

static Status_t
_initialize_Node_Port_SCVLMaps(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * in_portp)
{
	uint32_t amod = 0;
	uint8_t synchModeGen1 = 1;
	Status_t status = VSTATUS_OK;
	Node_t *neighborNodep;
	Port_t * neighborPortp,*swportp = NULL;
	STL_SCVLMAP scvlmap;
	STL_SCVLMAP * curScvlt, * curScvlnt;

	IB_ENTER(__func__, topop, nodep, in_portp, 0);
	SmpAddr_t addr;

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
			IB_LOG_WARNRC("_initialize_Switch_SCVLMaps: Failed to get SCVL "
				"map from routing algorithm; rc:",
				status);
			return status;
		}

		// Compare the port's current SCVL map against what the topology says it
		// should be.  If they're different, send the new one.
		if (!in_portp->portData->current.scvlr ||
			memcmp((void *)curScvl, (void *)&scvlmap, sizeof(scvlmap)) != 0) {
			SMP_ADDR_SET_LR(&addr, sm_lid, in_portp->portData->lid);
			psc_unlock(psc);
			status = SM_Set_SCVLrMap(fd, amod, &addr, &scvlmap, sm_config.mkey);
			psc_lock(psc);

			if (status != VSTATUS_OK) {
				IB_LOG_WARN_FMT("_initialize_Switch_SCVLMaps",
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
	// the SCtoVL_nt table must be configured consistently with the SCtoVL_t
	// table at its neighbor. When the link state is Init the SM shall have the
	// responsibility of updating both the SCtoVL_t table and the neighbor's
	// SCtoVL_nt table ("synchronous" update). When the link state is Armed or
	// Active, the SM shall update the SCtoVL_t table; and the SMAs of both
	// ports shall have the responsibility of updating the neighbor's SCtoVL_nt
	// table ("asynchronous" update of SCtoVL_t only).
	//
	// section 9.7.14.3 of the STL spec, "Optional Mechanism for Changes While
	// in LinkArmed or LinkActive" The mechanisms in this section are only
	// available when the ports on both side of a link report
	// IsAsyncSC2VLSupported. If either port reports it does not have this
	// capability, the FM shall not attempt to perform the changes outlined in
	// this section. It is anticipated that STL Gen1 will not support this
	// capability.
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
			SMP_ADDR_SET_LR(&addr, sm_lid, in_portp->portData->lid);
			psc_unlock(psc);
			status = SM_Set_SCVLtMap(fd, amod, &addr, &scvlmap, sm_config.mkey);
			psc_lock(psc);

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
			SMP_ADDR_SET_LR(&addr, sm_lid, (neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ? swportp->portData->lid : neighborPortp->portData->lid);
			psc_unlock(psc);
			status = SM_Set_SCVLntMap(fd, amod, &addr, &scvlmap,
				sm_config.mkey);
			psc_lock(psc);

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


static Status_t
_initialize_Node_SLMaps(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * out_portp)
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
		status = _initialize_Node_Port_SLSCMap(sm_topop, nodep, out_portp,
											   out_portp->portData->changes.slsc);
		if (status != VSTATUS_OK)
			return status;

		// initialize the SC2SL mapping table for the egress port
		status = _initialize_Node_Port_SCSLMap(sm_topop, nodep, out_portp,
											   out_portp->portData->changes.scsl);
		if (status != VSTATUS_OK)
			return status;
	}

	// initialize the SC2VL* mapping tables for the egress port
	status = _initialize_Node_Port_SCVLMaps(psc, fd, sm_topop, nodep,
											out_portp);
	if(status != VSTATUS_OK)
		return status;

	status = _node_syncSmaChanges(psc, fd, topop, nodep);

	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Error on synching SMA updates; status : 0x%02x; last-node processed NodeGUID : "
			FMT_U64, status, nodep->nodeInfo.NodeGUID);
	}

	return status;
}

static Status_t
_initialize_Switch_SLMaps(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep)
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
		status = _initialize_Switch_SLSCMap(sm_topop, nodep,
				swportp->portData->changes.slsc);
		if (status != VSTATUS_OK)
			return status;

		// initialize the SC2SL mapping table for all ports.
		status = _initialize_Switch_SCSLMap(sm_topop, nodep,
				swportp->portData->changes.scsl);
		if (status != VSTATUS_OK)
			return status;
	}

	// initialize the SC2VL* mapping tables for all ports.
	status = _initialize_Switch_SCVLMaps(psc, fd, sm_topop, nodep);
	if(status != VSTATUS_OK)
		return status;

	status = _node_syncSmaChanges(psc, fd, topop, nodep);

	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Error on synching SMA updates; status : 0x%02x; last-node processed NodeGUID : "
			FMT_U64, status, nodep->nodeInfo.NodeGUID);
	}

	IB_EXIT(__func__, status);
	return status;
}

/**
	Update the SMA of @c nodep with all changes stored on @c nodep.
*/
static Status_t
_port_syncSmaChanges(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * smaportp)
{
	Status_t s;

	if (nodep->aggregateEnable) {
		s = _port_syncSma_aggregate(psc, fd, topop, nodep, smaportp);
	}

	if (!nodep->aggregateEnable || s != VSTATUS_OK) {
		s = _port_syncSma_solo(psc, fd, topop, nodep, smaportp);

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
_port_syncSma_aggregate(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * smaportp)
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

	psc_unlock(psc);
	s = SM_Set_Aggregate_LR(fd, aggrBuffer, aggrHdr,
							sm_lid, destLid, sm_config.mkey, &lastSeg, &madStatus);
	psc_lock(psc);

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

fail:
	vs_pool_free(&sm_pool, aggrBuffer);

	return s;
}

static Status_t
_port_syncSma_solo(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * smaportp)
{
	Status_t status = VSTATUS_OK;

	STL_LID dlid;
	if (!sm_valid_port(smaportp))
		return VSTATUS_BAD;

	dlid = smaportp->portData->lid;
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

	if (smaportp->portData->dirty.slsc) {
		uint32_t amod = 0;
		STL_SLSCMAP slsc = *smaportp->portData->changes.slsc;

		psc_unlock(psc);
		status = SM_Set_SLSCMap(fd, amod, &addr, &slsc, sm_config.mkey);
		psc_lock(psc);

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
		psc_unlock(psc);
		status = SM_Set_SCSLMap(fd, amod, &addr, &scsl, sm_config.mkey);
		psc_lock(psc);

		if (status != VSTATUS_OK) {
			nodep->slscChange = 1;
			goto fail;
		}

		smaportp->portData->scslMap = scsl;
		smaportp->portData->dirty.scsl = 0;
	}


fail:
	return status;
}

static Status_t
_node_syncSmaChanges(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t *nodep)
{
	Status_t status = VSTATUS_OK;

	if (nodep) {
		Port_t * smaportp = NULL;

		for_all_sma_ports(nodep, smaportp) {
			if (!sm_valid_port(smaportp) || smaportp->state < IB_PORT_INIT)
				continue;

			 status = _port_syncSmaChanges(psc, fd, topop, nodep, smaportp);

			if (status != VSTATUS_OK) {
				status = sm_popo_port_error(&sm_popo, topop, smaportp, status);

				IB_LOG_WARN_FMT(__func__,
					"Failed to sync changes for node %s, node GUID "FMT_U64", port %d",
					sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index);
				if (status == VSTATUS_TIMEOUT_LIMIT)
					return status;
			}
		}

		// dirty and change values are not currently copied sweep-to sweep, so free to
		// release them whether or not Set() was successful
		sm_node_release_changes(nodep);
	}

	return status;
}

Status_t
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
				s = _aggregate_to_scvl(nodep, aggr);
				break;
			case STL_MCLASS_ATTRIB_ID_VL_ARBITRATION:
				s = _aggregate_to_vlarb(nodep, aggr);
				break;
			case STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE:
				s = _aggregate_to_bfrctrl(nodep, aggr);
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
_aggregate_to_scvl(Node_t * nodep, STL_AGGREGATE * aggr)
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
_aggregate_to_vlarb(Node_t * nodep, STL_AGGREGATE * aggr)
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
_aggregate_to_bfrctrl(Node_t * nodep, STL_AGGREGATE * aggr)
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
_write_vlarb_tables(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Node_t* nodep, Port_t* portp,  STL_LID dlid, PortDataVLArb* arbp)
{
	uint32_t amod;
	uint16_t numPorts = 1;
	uint32_t dataSize = 0;
	Status_t status = VSTATUS_OK;
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
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
		psc_unlock(psc);
		status = SM_Set_VLArbitration(fd, amod, &addr, (STL_VLARB_TABLE *)arbp->u.vlarb.vlarbHigh, sizeof(arbp->u.vlarb.vlarbHigh), sm_config.mkey);
		psc_lock(psc);

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
			psc_unlock(psc);
			status = SM_Set_VLArbitration(fd, amod, &addr, (STL_VLARB_TABLE*) arbp->u.vlarb.vlarbLow, sizeof(arbp->u.vlarb.vlarbLow), sm_config.mkey);
			psc_lock(psc);

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
			psc_unlock(psc);
			status = SM_Set_VLArbitration(fd, amod, &addr, (STL_VLARB_TABLE*) arbp->vlarbMatrix, sizeof(arbp->vlarbMatrix), sm_config.mkey);
			psc_lock(psc);

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


static void
_fill_vlarb_table_default(Node_t *nodep, struct _PortDataVLArb * arb, uint8_t numVls)
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

static Status_t
_initialize_VLArbitration(ParallelSweepContext_t *psc, SmMaiHandle_t *fd,
	Topology_t * topop, Node_t * nodep, Port_t * portp)
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
		_fill_vlarb_table_default(nodep, arbp, portp->portData->vl1);
	}

	if (nodep->vlArb) {
		status = _write_vlarb_tables(psc, fd, nodep, portp, dlid, arbp);
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
_set_weight_multiplier(Qos_t * qos)
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

/**
    Update the and head of queue lifetime limit (HLL) values on
    @c nodep and @c portp, respectively. Accounts for 5 bits
    vs. 4 bits for HLL storage on the SMA when determining if an
    update is required but does not convert values.

	@param updHoq [out] Does HOQ need to be updated on this port?

	@return VSTATUS_OK on success or something else on error.
*/
static Status_t
_update_hoq(Node_t * nodep, Port_t * curPort, boolean * updHoq)
{
	VlVfMap_t vlvfmap;
	VlBwMap_t vlbwmap;
	Status_t status = VSTATUS_OK;
	int vl;
	uint32_t maxHoqVals[STL_MAX_VLS];

	uint32_t preemptibleVLs = 0;

	// Update HLL, only if not port 0
	if (curPort->index == 0) return status;

	memset(maxHoqVals, 0, sizeof(maxHoqVals));

	// Default VL15 to SM instance values.
	maxHoqVals[15] = sm_config.hoqlife_n2;

	status = sm_topop->routingModule->funcs.select_vlvf_map(sm_topop, nodep, curPort, &vlvfmap);

	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			 "Unable to acquire VF-VL map for node %s guid "FMT_U64", port %d, status = %d",
			 sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, curPort->index, status);
		return status;
	}

	status = sm_topop->routingModule->funcs.select_vlbw_map(sm_topop, nodep, curPort, &vlbwmap);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			 "Unable to acquire VF-BW map for node %s guid "FMT_U64", port %d, status = %d",
			 sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, curPort->index, status);

		for (vl = 0; vl < STL_MAX_VLS; vl++) {
			bitset_free(&vlvfmap.vf[vl]);
		}
		return status;
	}


	// Bitfields of active VLs (based on VF) for this port
	bitset_t isVlActive;
	bitset_init(&sm_pool, &isVlActive, STL_MAX_VLS);
	bitset_set(&isVlActive, 15);

	VirtualFabrics_t *vfs = sm_topop->vfs_ptr;
	uint8_t numVls = (curPort->portData->vl1 <= 15 ? curPort->portData->vl1 : curPort->portData->vl1+1);
	if (vlbwmap.has_qos) for (vl = 0; vl < numVls; vl++) {
		if (vlbwmap.qos[vl] >= MAX_QOS_GROUPS) continue;
		if (vl == 15) continue;

		if (bitset_nset(&vlvfmap.vf[vl]))
			bitset_set(&isVlActive, vl);

		maxHoqVals[vl] = vfs->qos_all[vlbwmap.qos[vl]].hoqlife_qos;

		// get a bitmask of preemptible VLs.
		preemptibleVLs |= curPort->portData->curArb.vlarbMatrix[vl];
	} else {
		// No QOS. One VL.
		maxHoqVals[0] = sm_config.hoqlife_n2;
		bitset_set(&isVlActive, 0);
	}

	// Bitfield indicating hi priority VLs
	bitset_t isVlHigh;
	bitset_init(&sm_pool, &isVlHigh, STL_MAX_VLS);

	uint8_t idx;
	for (idx=0; idx<STL_MAX_LOW_CAP; idx++) {
		if (curPort->portData->curArb.u.vlarb.vlarbHigh[idx].Weight > 0) {
			bitset_set(&isVlHigh, curPort->portData->curArb.u.vlarb.vlarbHigh[idx].s.VL);
		}
	}

	// Setup the values per-VL values for this port.
	for (vl = 0; vl < MAX(numVls, 16); vl++) {
		if (numVls < 16 && vl==numVls)
			vl = 15;   // skip to VL15

		// Since perVL-VLStallCnt is limited in HW implementation,
		// just copy the SM-wide VLStallCnt into all active VLs
		// (including VL 15).
		uint8_t vlStallCnt = sm_config.vlstall;
		if (curPort->portData->portInfo.XmitQ[vl].VLStallCount != vlStallCnt) {
			curPort->portData->portInfo.XmitQ[vl].VLStallCount = vlStallCnt;
			*updHoq = 1;
		}

		if (bitset_test(&isVlActive, vl)) {
			boolean incrTimeouts = 0;

			// Increment to next timer value under specific conditions.
			if (vl != 15 && maxHoqVals[vl] < IB_LIFETIME_MAX) {
				incrTimeouts = (!bitset_test(&isVlHigh, vl) && (bitset_nset(&isVlHigh) > 0 || vlbwmap.bw[vl] < 25)) ||
						(curPort->portData->portInfo.FlitControl.Preemption.PreemptionLimit == STL_PORT_PREEMPTION_LIMIT_NONE &&
						 ((1<<vl) & preemptibleVLs)!=0);
			}

			if (incrTimeouts && (sm_config.timerScalingEnable!=0)) {
				uint32_t oldHoq;

				oldHoq = maxHoqVals[vl];
				if (maxHoqVals[vl] < IB_LIFETIME_MAX)
					maxHoqVals[vl]++;

				IB_LOG_DEBUG1_FMT(__func__,
								  "Adjusting HOQ Lifetime values.  "
								  "Node %s guid "FMT_U64": port %d; VL : %d;"
								  " current value HOQLife : %d; new values HOQLife : %d",
								  sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, curPort->index,
								  vl, oldHoq, maxHoqVals[vl]);
			}

			uint8 vlHll = maxHoqVals[vl];
			if (curPort->portData->portInfo.XmitQ[vl].HOQLife != vlHll) {
				curPort->portData->portInfo.XmitQ[vl].HOQLife = vlHll;
				*updHoq = 1;
			}
		}
	}

	for (vl = 0; vl < STL_MAX_VLS; vl++) {
		bitset_free(&vlvfmap.vf[vl]);
	}
	bitset_free(&isVlHigh);
	bitset_free(&isVlActive);

	return status;
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
_set_vlarb_entry(STL_VLARB_TABLE_ELEMENT * vlblockp, uint8_t vl, int* entry,
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
_fill_low_rr(Node_t * nodep, Port_t * portp, STL_VLARB_TABLE_ELEMENT * vlblockp, Qos_t * qos, int weight)
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
	_roundVLBandwidths(qos);

	if (!bitset_init(&sm_pool, &highbwVLs, STL_MAX_VLS)) {
		IB_FATAL_ERROR_NODUMP("_fill_low_rr: No memory for QoS setup, exiting.");
	}

	if (!bitset_init(&sm_pool, &vlsInUse, STL_MAX_VLS)) {
		IB_FATAL_ERROR_NODUMP("_fill_low_rr: No memory for QoS setup, exiting.");
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

			_set_vlarb_entry(vlblockp, i, &currentEntry, weight, bwFitsInTable,
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

			_set_vlarb_entry(vlblockp, currentVl, &currentEntry, weight, bwFitsInTable,
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

static Status_t
_fill_vlarb_table(Topology_t * topop, Node_t * nodep, Port_t * portp, Qos_t * qos, struct _PortDataVLArb * arbp)
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
		_fill_low_rr(nodep, portp, vlblockp, qos, weight);
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
_dbg_sprintf_vlvf_bw_info(Topology_t *topop, char *buf, int bufSize, Qos_t* qos,
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
						sl = vfs->qos_all[vf->qos_index].base_sl; mc_sl = 0;
						break;
					case 1:
						sl = vfs->qos_all[vf->qos_index].resp_sl; mc_sl = 0;
						if (sl == vfs->qos_all[vf->qos_index].base_sl) continue;
						break;
					case 2:
						sl = vfs->qos_all[vf->qos_index].mcast_sl; mc_sl = 1;
						if (sl == vfs->qos_all[vf->qos_index].base_sl) continue;
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
			if ((slSet >> vfs->qos_all[vf->qos_index].base_sl) & 0x1) {
				hasBase = 1;
				checked_snprintf(buf, bufSize, "base");
			}

			if (vfs->qos_all[vf->qos_index].base_sl != vfs->qos_all[vf->qos_index].resp_sl &&
					((slSet >> vfs->qos_all[vf->qos_index].resp_sl) & 0x1)) {
				if (hasBase) {
					checked_snprintf(buf, bufSize, ",");
				}
				checked_snprintf(buf, bufSize, "resp");
			}

			if (vfs->qos_all[vf->qos_index].base_sl != vfs->qos_all[vf->qos_index].mcast_sl &&
					((slSet >> vfs->qos_all[vf->qos_index].mcast_sl) & 0x1)) {
				if (hasBase) {
					checked_snprintf(buf, bufSize, ",");
				}
				checked_snprintf(buf, bufSize, "mcast");
			}

			if (!vfs->qos_all[vf->qos_index].qos_enable) {
				checked_snprintf(buf, bufSize, ",noqos");
			} else if (vfs->qos_all[vf->qos_index].priority) {
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
_print_vlvf_bw_info(Topology_t *topop, Node_t *nodep, Port_t *portp, Qos_t *qos,
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

			_dbg_sprintf_vlvf_bw_info(topop, buf, bufSize, qos, vfs,
				dbgSlsc, (1 << j)); // one VL at a time
			IB_LOG_DEBUG1_FMT(NULL, "Node,%s,Port,%d,VlArbTbl,%s,BwPct,%d,%s",
				sm_nodeDescString(nodep), portp->index,
				StlVlarbSecToText(STL_VLARB_LOW_ELEMENTS + i),
				(vlBw[j]*100)/totalBw, buf);
		}
	}
}


Status_t
sm_select_vlvf_map(Topology_t *topop, Node_t *nodep, Port_t *portp, VlVfMap_t * vlvfmap)
{
	// Each VL can map to 0 to 1000 possible VFs.  If VL-VF unused, set to -1

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

	return VSTATUS_OK;
}

Status_t
sm_fill_stl_vlarb_table(Topology_t *topop, Node_t *nodep, Port_t *portp, PortDataVLArb* arbp)
{
	uint8_t		numVls;
	int 		i, j;
	Qos_t * 	qos;
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
		_fill_vlarb_table_default(nodep, arbp, numVls);
		return VSTATUS_OK;
	}

	VlBwMap_t   vlbwmap;
	if (portp) {
		topop->routingModule->funcs.select_vlbw_map(topop, nodep, portp, &vlbwmap);
	} else {
		// Internal table (INQ/subswitch)
		vlbwmap = qos->vlBandwidth;
	}

	// Generate Preemption Matrix
	// Note - possibly move this to Qos_t and determine once and done.
	memset(vlRank, 0, sizeof(vlRank));
	// Determine the max each VL
	if (vlbwmap.has_qos) for (i=0; i<STL_MAX_VLS; i++) {
		if (vlbwmap.qos[i] >= MAX_QOS_GROUPS) continue;
		if (i >= numVls || i==15) {
			IB_LOG_WARN("Unexpected VF:: Mapping: VL=%d", i);
			break;
		}
		vlRank[i] = VirtualFabrics->qos_all[vlbwmap.qos[i]].preempt_rank;
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
		status = _fill_vlarb_table(topop, nodep, portp, qos, arbp);
	}

	if (IB_LOG_IS_INTERESTED(VS_LOG_DEBUG1) && status == VSTATUS_OK &&
		portp && nodep->nodeInfo.NodeType == NI_TYPE_CA) {
		// This debug output doesn't handle SC->SC' mappings,
		// so skip switches entirely for now
		_print_vlvf_bw_info(topop, nodep, portp, qos, VirtualFabrics, arbp);
	}

	return status;
}

Status_t
sm_select_slsc_map(Topology_t *topop, Node_t *nodep,
	Port_t *in_portp, Port_t *out_portp, STL_SLSCMAP *outSlscMap)
{
	uint8_t sl;
	int qs;
	STL_SLSCMAP slsc;

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	/* loop will look for each bit set starting from bit 0 to last bit set,
	 * which will not exceed bitset size
	 */
	for (qs = 0; (qs = bitset_find_next_one(&out_portp->portData->qosMember, qs)) != -1; qs++) {
		/* In order to generate unique SL2SC map for this egress port,
		 * filter the SLs based on this port's VF memberships.
		 */
		QosConfig_t *qosp = &VirtualFabrics->qos_all[qs];
		bitset_set(&sm_linkSLsInuse, qosp->base_sl);
		if (qosp->base_sl != qosp->resp_sl)
			bitset_set(&sm_linkSLsInuse, qosp->resp_sl);
		if (qosp->base_sl != qosp->mcast_sl)
			bitset_set(&sm_linkSLsInuse, qosp->mcast_sl);
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
	int qs = 0;
	STL_SCSLMAP scsl;

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	for (qs = 0; (qs = bitset_find_next_one(&in_portp->portData->qosMember, qs)) != -1; qs++) {
		QosConfig_t * qosp = &VirtualFabrics->qos_all[qs];
		bitset_set(&sm_linkSLsInuse, qosp->base_sl);
		if (qosp->base_sl != qosp->resp_sl)
			bitset_set(&sm_linkSLsInuse, qosp->resp_sl);
		if (qosp->base_sl != qosp->mcast_sl)
			bitset_set(&sm_linkSLsInuse, qosp->mcast_sl);
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
	int sc, qs;
	Qos_t *qos = sm_get_qos(vlCap);

	bitset_clear_all(&sm_linkSLsInuse);

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	memset(outScvlMap, 0, sizeof(STL_SCVLMAP));

	// In order to generate unique SL2SC map for this egress port,
	// filter the SLs based on this port's VF memberships.
	for (qs = 0; qs < VirtualFabrics->number_of_qos_all; qs++) {
		// Find all the SLs. Can't just use sm_linkSLsInuse, because it's used as a scratch variable
		QosConfig_t *qosp = &VirtualFabrics->qos_all[qs];
		bitset_set(&sm_linkSLsInuse, qosp->base_sl);
		bitset_set(&sm_linkSLsInuse, qosp->resp_sl);
		bitset_set(&sm_linkSLsInuse, qosp->mcast_sl);
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
		uint32_t qos_idx = vfs->v_fabric_all[vf].qos_index;
		IB_LOG_INFINI_INFO_FMT_VF(vfs->v_fabric_all[vf].name, "",
			"Base SL:%d Resp SL:%d Requires Resp SL:%d Multicast SL:%d QOS:%d HP:%d PKey:0x%04x",
			vfs->qos_all[qos_idx].base_sl, vfs->qos_all[qos_idx].resp_sl,
			vfs->qos_all[qos_idx].requires_resp_sl, vfs->qos_all[qos_idx].mcast_sl,
			vfs->qos_all[qos_idx].qos_enable, vfs->qos_all[qos_idx].priority,
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

static void
_qos_switch_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Status_t	status = VSTATUS_OK;

	QosWorkItem_t 	*wip = PARENT_STRUCT(pwi, QosWorkItem_t, item);
	Node_t 			*nodep = wip->nodep;
	Port_t 			*portp = NULL;

	DEBUG_ASSERT(psc);

	MaiPool_t		*mpp = psc_get_mai(psc);
	if (!mpp) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate MAI handle.");
		psc_set_status(psc, VSTATUS_NOMEM);
		psc_stop(psc);
		return;
	}

	psc_lock(psc);

	if (sm_state != SM_STATE_MASTER) {
		psc_stop(psc);
		goto bail;
	}

	// Node is a switch.
	if ((status = _initialize_Switch_SLMaps(psc, mpp->fd, sm_topop, nodep)) != VSTATUS_OK) {
		if (topology_main_exit == 1) {
#ifdef __VXWORKS__
			ESM_LOG_ESMINFO(add_quotes(__func__)": SM has been stopped", 0);
#endif
			psc_stop(psc);
			goto bail;
		}

		// sm_popo_port_error() would have already been called. No need to call it again.
		sm_mark_switch_down(sm_topop, nodep);
		topology_changed = 1;
		IB_LOG_ERROR_FMT(__func__,
						"Failed to init SL Maps; Switch %s nodeGuid "
						FMT_U64 "; Setting Switch Down",
						sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);

		goto bail;
	}

	// Setup vlarb for each port on switch
	if (nodep->vlArb) {
		for_all_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
				continue;
			}

			if ((status = _initialize_VLArbitration(psc, mpp->fd, sm_topop,
											nodep, portp)) != VSTATUS_OK) {
				if (topology_main_exit == 1) {
#ifdef __VXWORKS__
					ESM_LOG_ESMINFO(add_quotes(__func__)": SM has been stopped", 0);
#endif
					psc_stop(psc);
					goto bail;
				}

				sm_mark_link_down(sm_topop, portp);
				topology_changed = 1;   /* indicates a fabric change has been detected */
				IB_LOG_ERROR_FMT(__func__,
								"Failed to init VL Arb on node %s nodeGuid "
								FMT_U64 " node index %d port index %d; Setting Port Down",
								sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								nodep->index, portp->index);

				status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					goto bail;
				}
			}
		}
	} else {
		// Physically unpossible, but let's handle it anyway.
		sm_mark_switch_down(sm_topop, nodep);
		topology_changed = 1;
		IB_LOG_ERROR_FMT(__func__, "Switch %s nodeGuid " FMT_U64
						" has no arbitration capability. Marking switch down.",
						sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
		goto bail;
	}

	// Do HOQLife initialization
	uint8_t port;
	portp = sm_get_port(nodep, 0);

	if (!sm_valid_port(portp)) {
		goto bail;
	}

	boolean updHoq = 0;
	for (port = 0; port <= nodep->nodeInfo.NumPorts; port++) {
		Port_t * curPort = sm_get_port(nodep, port);

		if (!sm_valid_port(curPort) || curPort->state <= IB_PORT_DOWN) continue;

		status = _update_hoq(nodep, curPort, &updHoq);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
							"Failed to compute HOQ values for node %s nodeGuid "FMT_U64" port %d status = %d",
							sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, curPort->index, status);

			goto bail;
		}

		// If the port is already ARMED, the SM should make sure that
		// HOQLife gets updated.  Otherwise, the SM will update HOQLife when
		// it goes to ARM or ACTIVE-ate the port
		if (updHoq && curPort->state >= IB_PORT_ARMED) {
			struct XmitQ_s xmitQVals[STL_MAX_VLS];

			memcpy(xmitQVals, &curPort->portData->portInfo.XmitQ, sizeof(xmitQVals));

			uint32 amod = (1 << 24) | curPort->index;

			Port_t * switchP0 = sm_get_port(nodep, 0);
			STL_LID dlid = switchP0->portData->lid;
			SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
			psc_unlock(psc);
			status = SM_Set_PortInfo(mpp->fd, amod, &addr,
							&curPort->portData->portInfo,
							curPort->portData->portInfo.M_Key, NULL);
			psc_lock(psc);
			sm_popo_update_port_state(&sm_popo, curPort,
							&curPort->portData->portInfo.PortStates);

			if (status != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
								"Failed to set PortInfo for node %s nodeGuid "FMT_U64", port %d: status = %d",
								sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, curPort->index, status);
				status = sm_popo_port_error(&sm_popo, sm_topop, switchP0, status);
				goto bail;
			}

			if (!sm_eq_XmitQ(xmitQVals, curPort->portData->portInfo.XmitQ, curPort->portData->portInfo.s4.OperationalVL)) {
				IB_LOG_ERROR_FMT(__func__,
								"Mismatch between expected and actual XmitQ values on port.  Failed to set PortInfo for node %s nodeGuid "FMT_U64", port %d: status = %d.",
								sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, curPort->index, status);
				status = VSTATUS_BAD;
				goto bail;
			}
		}
	}

bail:
	if (status == VSTATUS_TIMEOUT_LIMIT || status == VSTATUS_UNRECOVERABLE) {
		psc_stop(psc);
	}
	_qos_workitem_free(wip);
	psc_set_status(psc, status);
	psc_unlock(psc);
	psc_free_mai(psc, mpp);
}


static void
_qos_hfi_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Status_t	status = VSTATUS_OK;

	QosWorkItem_t 	*wip = PARENT_STRUCT(pwi, QosWorkItem_t, item);
	Node_t 			*nodep = wip->nodep;
	Port_t 			*portp = NULL;

	DEBUG_ASSERT(psc);

	MaiPool_t		*mpp = psc_get_mai(psc);
	if (!mpp) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate MAI handle.");
		psc_set_status(psc, VSTATUS_NOMEM);
		psc_stop(psc);
		return;
	}

	psc_lock(psc);

	if (sm_state != SM_STATE_MASTER) {
		psc_stop(psc);
		goto bail;
	}

	for_all_ports(nodep, portp) {
		// Iterate over the ports of this HFI.
		if (sm_valid_port(portp) && portp->state > IB_PORT_DOWN) {
			if ((status=_initialize_Node_SLMaps(psc, mpp->fd, sm_topop,
				nodep, portp)) != VSTATUS_OK) {
				if (topology_main_exit == 1) {
#ifdef __VXWORKS__
					ESM_LOG_ESMINFO(add_quotes(__func__)": SM has been stopped", 0);
#endif
					psc_stop(psc);
					goto bail;
				}

				sm_mark_link_down(sm_topop, portp);
				topology_changed = 1;	/* indicates a fabric change has been detected */
				IB_LOG_ERROR_FMT(__func__,
					"Failed to init SL2SC/SC2SL Map (ignoring port) on node %s nodeGuid "FMT_U64" node index %d port index %d",
					sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, nodep->index, portp->index);

				status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					psc_stop(psc);
					goto bail;
				}

				continue;
			}

			if ((status = _initialize_VLArbitration(psc, mpp->fd, sm_topop, nodep,
				portp)) != VSTATUS_OK) {
				if (topology_main_exit == 1) {
#ifdef __VXWORKS__
					ESM_LOG_ESMINFO(add_quotes(__func__)": SM has been stopped", 0);
#endif
					psc_stop(psc);
					goto bail;
				}

				sm_mark_link_down(sm_topop, portp);
				topology_changed = 1;	/* indicates a fabric change has been detected */
				IB_LOG_ERROR_FMT(__func__,
					"Failed to init VL Arb (ignoring port) on node %s nodeGuid "FMT_U64" node index %d port index %d",
					sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, nodep->index, portp->index);

				status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					psc_stop(psc);
					goto bail;
				}
			}
		}
	}

bail:
	_qos_workitem_free(wip);
	psc_set_status(psc, status);
	psc_unlock(psc);
	psc_free_mai(psc, mpp);
}

Status_t
sweep_assignments_vlarb(SweepContext_t *sweep_context)
{
	Status_t		status = VSTATUS_OK;
	Node_t			*nodep;
	QosWorkItem_t 	*wip;

	IB_ENTER(__func__, 0, 0, 0, 0);

	if (sm_state != SM_STATE_MASTER)
		return VSTATUS_NOT_MASTER;

	psc_go(sweep_context->psc);

	for_all_switch_nodes(sm_topop, nodep) {
		wip = _qos_workitem_alloc(nodep, _qos_switch_worker);
		if (wip == NULL) {
			psc_set_status(sweep_context->psc, VSTATUS_NOMEM);
			psc_stop(sweep_context->psc);
			goto bail;
		}

		psc_add_work_item(sweep_context->psc, &wip->item);
	}

	for_all_ca_nodes(sm_topop, nodep) {
		wip = _qos_workitem_alloc(nodep, _qos_hfi_worker);
		if (wip == NULL) {
			psc_set_status(sweep_context->psc, VSTATUS_NOMEM);
			psc_stop(sweep_context->psc);
			goto bail;
		}

		psc_add_work_item(sweep_context->psc, &wip->item);
	}

bail:
	status = psc_wait(sweep_context->psc);
	psc_drain_work_queue(sweep_context->psc);

	IB_EXIT(__func__, status);
	return status;
}
