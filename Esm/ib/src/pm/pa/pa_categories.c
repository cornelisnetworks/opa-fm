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

/*
 * This File is for any function related to PA Categories
 *
 */


#include "pm_topology.h"
/* Compute Utilization values */
uint32 computeSendMBps(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 interval = (data ? *(uint32 *)data : pm->interval);
	uint32 SendMBps, SendMBps2 = 0;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	port2 = port->Image[imageIndex].neighbor;

	SendMBps = (uint32)(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData / FLITS_PER_MB / interval);

	if (port2) {
		SendMBps2 = (uint32)(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvData / FLITS_PER_MB / interval);
	}

	return MAX(SendMBps, SendMBps2);
}
uint32 computeSendKPkts(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 interval = (data ? *(uint32 *)data : pm->interval);
	uint32 SendKPkts, SendKPkts2 = 0;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	port2 = port->Image[imageIndex].neighbor;

	SendKPkts = (uint32)(port->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts / 1000 / interval);

	if (port2)
		SendKPkts2 = (uint32)(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvPkts / 1000 / interval);

	return MAX(SendKPkts, SendKPkts2);
}

/* Compute PA Categories */
uint32 computeIntegrity(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	IntegrityWeights_t *weights = (data ? (IntegrityWeights_t *)data : &pm->integrityWeights);
	uint32 Integrity, LQI;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	port2 = port->Image[imageIndex].neighbor;

	LQI = (port->Image[imageIndex].StlPortCounters.lq.s.LinkQualityIndicator <= STL_LINKQUALITY_EXCELLENT ?
		( (1 << (STL_LINKQUALITY_EXCELLENT - port->Image[imageIndex].StlPortCounters.lq.s.LinkQualityIndicator) ) - 1) : 0);

	Integrity = (uint32)(
		port->Image[imageIndex].DeltaStlPortCounters.LocalLinkIntegrityErrors * weights->LocalLinkIntegrityErrors +
		port->Image[imageIndex].DeltaStlPortCounters.PortRcvErrors * weights->PortRcvErrors +
		port->Image[imageIndex].DeltaStlPortCounters.LinkErrorRecovery * weights->LinkErrorRecovery +
		port->Image[imageIndex].DeltaStlPortCounters.LinkDowned * weights->LinkDowned +
		port->Image[imageIndex].DeltaStlPortCounters.UncorrectableErrors * weights->UncorrectableErrors +
		port->Image[imageIndex].DeltaStlPortCounters.FMConfigErrors * weights->FMConfigErrors +
		port->Image[imageIndex].StlPortCounters.lq.s.NumLanesDown * weights->LinkWidthDowngrade +
		LQI * weights->LinkQualityIndicator);

	if (port2)
		Integrity += (uint32)(
			port2->Image[imageIndex].DeltaStlPortCounters.ExcessiveBufferOverruns * weights->ExcessiveBufferOverruns);

	return Integrity;
}

uint32 computeCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	CongestionWeights_t *weights = (data ? (CongestionWeights_t *)data : &pm->congestionWeights);
	uint32 Congestion;
	uint64_t DeltaXmitWait;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	DeltaXmitWait = port->Image[imageIndex].DeltaStlPortCounters.PortXmitWait;
	port2 = port->Image[imageIndex].neighbor;

	if (pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS && DeltaXmitWait) {
		/* If VlXmitWait counters are available, use worst VL to properly weight the
		 * port-level counter to prevent low levels of congestion appearing as severe congestion.
		 */
		uint64_t MaxDeltaVLXmitWait = 0;
		uint32_t i, vlmask = port->Image[imageIndex].vlSelectMask;
		for (i = 0; i < STL_MAX_VLS && vlmask; i++, vlmask >>= 1) {
			if (vlmask & 0x1) {
				UPDATE_MAX(MaxDeltaVLXmitWait, port->Image[imageIndex].DeltaStlVLPortCounters[vl_to_idx(i)].PortVLXmitWait);
			}
		}
		DeltaXmitWait = (uint64_t)((double)MaxDeltaVLXmitWait * (double)MaxDeltaVLXmitWait / (double)DeltaXmitWait);
	}

	uint64 XmitWaitFlitTimes = DeltaXmitWait;
	uint64 XmitTimeCongFlitTimes = port->Image[imageIndex].DeltaStlPortCounters.PortXmitTimeCong;

	/* Convert switch wait counter units from cycle time to flit time */
	if (port->pmnodep->nodeType == STL_NODE_SW) {
		XmitWaitFlitTimes = XmitWaitFlitTimes * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
		XmitTimeCongFlitTimes = XmitTimeCongFlitTimes * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
	}
	Congestion = (uint32)(
		(XmitWaitFlitTimes ?
			(XmitWaitFlitTimes * weights->PortXmitWait * 10000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData +
				XmitWaitFlitTimes) : 0) +

		(XmitTimeCongFlitTimes ?
			(XmitTimeCongFlitTimes * weights->PortXmitTimeCong * 1000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData +
				XmitTimeCongFlitTimes) : 0) +

		(port->Image[imageIndex].DeltaStlPortCounters.PortRcvPkts ?
			((port->pmnodep->nodeType == STL_NODE_FI ? 1 : 0) *
				(port->Image[imageIndex].DeltaStlPortCounters.PortRcvBECN * weights->PortRcvBECN * 1000)) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortRcvPkts) : 0) +

		(port->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts ?
			(port->Image[imageIndex].DeltaStlPortCounters.PortMarkFECN * weights->PortMarkFECN * 1000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts) : 0) +

		(port->Image[imageIndex].DeltaStlPortCounters.SwPortCongestion * weights->SwPortCongestion));

	if (port2) {
		Congestion += (uint32)(
			(port2->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts ?
				(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvFECN * weights->PortRcvFECN * 1000) /
				(port2->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts) : 0));
	}

	return Congestion;
}

uint32 computeSmaCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	CongestionWeights_t *weights = (data ? (CongestionWeights_t *)data : &pm->congestionWeights);
	uint32 SmaCongestion;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;
	if ((pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) == 0) return 0;

	port2 = port->Image[imageIndex].neighbor;
	

	uint64 VlXmitWaitFlitTimes = port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitWait;
	uint64 VlXmitTimeCongFlitTimes = port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitTimeCong;

	/* Convert switch wait counter units from cycle time to flit time */
	if (port->pmnodep->nodeType == STL_NODE_SW) {
		VlXmitWaitFlitTimes = VlXmitWaitFlitTimes * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
		VlXmitTimeCongFlitTimes = VlXmitTimeCongFlitTimes * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
	}

	SmaCongestion = (uint32)(
		(VlXmitWaitFlitTimes ?
			(VlXmitWaitFlitTimes * weights->PortXmitWait * 10000) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitData +
				VlXmitWaitFlitTimes) : 0) +

		(VlXmitTimeCongFlitTimes ?
			(VlXmitTimeCongFlitTimes * weights->PortXmitTimeCong * 1000) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitData +
				VlXmitTimeCongFlitTimes) : 0) +

		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvPkts ?
			((port->pmnodep->nodeType == STL_NODE_FI ? 1 : 0) *
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvBECN * weights->PortRcvBECN * 1000)) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvPkts) : 0) +

		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitPkts ?
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLMarkFECN * weights->PortMarkFECN * 1000) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitPkts) : 0) +
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].SwPortVLCongestion * weights->SwPortCongestion));

	if (port2)
		SmaCongestion += (uint32)(
			(port2->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitPkts ?
				(port2->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvFECN * weights->PortRcvFECN * 1000) /
				(port2->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitPkts) : 0) );

	return SmaCongestion;
}

uint32 computeBubble(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 Bubble, Bubble2 = 0;
	uint64 PortXmitBubble;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;
	if (data != NULL) return 0;

	port2 = port->Image[imageIndex].neighbor;


	PortXmitBubble =
		(port->Image[imageIndex].DeltaStlPortCounters.PortXmitWastedBW +
		port->Image[imageIndex].DeltaStlPortCounters.PortXmitWaitData);

	/* Convert switch wait counter units from cycle time to flit time */
	if (port->pmnodep->nodeType == STL_NODE_SW) {
		PortXmitBubble = PortXmitBubble * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
	}
	Bubble = (uint32)(
		(PortXmitBubble ? (PortXmitBubble * 10000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData + PortXmitBubble) : 0));

	if (port2) {
		uint64 RcvBubbleFlitTimes = port->Image[imageIndex].DeltaStlPortCounters.PortRcvBubble;
		/* Convert switch wait counter units from cycle time to flit time */
		if (port->pmnodep->nodeType == STL_NODE_SW) {
			RcvBubbleFlitTimes = RcvBubbleFlitTimes * 2 *
				(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
		}
		Bubble2 = (uint32)(
			(RcvBubbleFlitTimes ?
				(RcvBubbleFlitTimes * 10000) /
				(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvData +
				RcvBubbleFlitTimes) : 0));
	}

	return MAX(Bubble, Bubble2);
}

uint32 computeSecurity(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 Security;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;
	if (data != NULL) return 0;

	port2 = port->Image[imageIndex].neighbor;

	Security = port->Image[imageIndex].DeltaStlPortCounters.PortXmitConstraintErrors;

	if (port2)
		Security += port2->Image[imageIndex].DeltaStlPortCounters.PortRcvConstraintErrors;

	return Security;
}
uint32 computeRouting(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	/*PmPort_t *port2 = NULL;*/
	uint32 Routing;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	if (data != NULL) return 0;
	/*port2 = port->Image[imageIndex].neighbor;*/

	Routing = port->Image[imageIndex].DeltaStlPortCounters.PortRcvSwitchRelayErrors;
	/*
	if (port2)
		Routing += 0;
	*/
	return Routing;
}
uint32 computeUtilizationPct10(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	/*PmPort_t *port2 = NULL;*/
	uint32 UtilizationPct10;
	uint32 SendMBps, MaxMBps;
	uint32 rate;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	/*port2 = port->Image[imageIndex].neighbor;*/

	rate = PmCalculateRate(port->Image[imageIndex].u.s.activeSpeed, port->Image[imageIndex].u.s.rxActiveWidth);
	if (rate == IB_STATIC_RATE_1GB) return 0;

	SendMBps = computeSendMBps(pm, imageIndex, port, data);

	MaxMBps = s_StaticRateToMBps[rate];

	UtilizationPct10 = SendMBps * 1000 / MaxMBps;

	/*
	if (port2)
		UtilizationPct10 = 0;
	*/
	return UtilizationPct10;
}
uint32 computeDiscardsPct10(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	/*PmPort_t *port2 = NULL;*/
	uint32 DiscardsPct10;
	uint64 AttemptedPkts;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	if (data != NULL) return 0;
	/*port2 = port->Image[imageIndex].neighbor;*/

	AttemptedPkts = port->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts +
		port->Image[imageIndex].DeltaStlPortCounters.PortXmitDiscards;

	DiscardsPct10 = AttemptedPkts ?
		(port->Image[imageIndex].DeltaStlPortCounters.PortXmitDiscards * 1000) /
			(AttemptedPkts) : 0;
	/*
	if (port2)
		DiscardsPct10 += 0;
	*/
	return DiscardsPct10;
}

/* Compute Utilization values */
uint32 computeVFSendMBps(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 interval = (data ? ((PmVFFocusPortComputeData_t *)data)->imageInterval : pm->interval);
	uint32 SendMBps, SendMBps2 = 0;
	uint64 PortVFXmitData = 0, PortVFRcvData = 0;
	uint32 vl, vlmask;
	int vfIdx = (data ? ((PmVFFocusPortComputeData_t *)data)->vfIdx : -1);

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID ||
		vfIdx == -1) return 0;

	port2 = port->Image[imageIndex].neighbor;

	vlmask = port->Image[imageIndex].vfvlmap[vfIdx].vlmask;
	// Iterate through All VLs within this VF
	for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
		// Skip unassigned VLs (0) in VF
		if (vlmask & 0x1) {
			PortVFXmitData += port->Image[imageIndex].DeltaStlVLPortCounters[vl_to_idx(vl)].PortVLXmitData;
		}
	}
	SendMBps = (uint32)(PortVFXmitData / FLITS_PER_MB / interval);

	if (port2) {
		vlmask = port2->Image[imageIndex].vfvlmap[vfIdx].vlmask;
		// Iterate through All VLs within this VF
		for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
			// Skip unassigned VLs (0) in VF
			if (vlmask & 0x1) {
				PortVFRcvData += port2->Image[imageIndex].DeltaStlVLPortCounters[vl_to_idx(vl)].PortVLRcvData;
			}
		}
		SendMBps2 = (uint32)(PortVFRcvData / FLITS_PER_MB / interval);
	}

	return MAX(SendMBps, SendMBps2);
}

uint32 computeVFSendKPkts(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 interval = (data ? ((PmVFFocusPortComputeData_t *)data)->imageInterval : pm->interval);
	uint32 SendKPkts, SendKPkts2 = 0;
	uint64 PortVFXmitPkts = 0, PortVFRcvPkts = 0;
	uint32 vl, vlmask;
	int vfIdx = (data ? ((PmVFFocusPortComputeData_t *)data)->vfIdx : -1);

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID ||
		vfIdx == -1) return 0;

	port2 = port->Image[imageIndex].neighbor;

	vlmask = port->Image[imageIndex].vfvlmap[vfIdx].vlmask;
	// Iterate through All VLs within this VF
	for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
		// Skip unassigned VLs (0) in VF
		if (vlmask & 0x1) {
			PortVFXmitPkts += port->Image[imageIndex].DeltaStlVLPortCounters[vl_to_idx(vl)].PortVLXmitPkts;
		}
	}
	SendKPkts = (uint32)(PortVFXmitPkts / FLITS_PER_MB / interval);

	if (port2)
	{
		vlmask = port2->Image[imageIndex].vfvlmap[vfIdx].vlmask;
		// Iterate through All VLs within this VF
		for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
			// Skip unassigned VLs (0) in VF
			if (vlmask & 0x1) {
				PortVFRcvPkts += port2->Image[imageIndex].DeltaStlVLPortCounters[vl_to_idx(vl)].PortVLRcvPkts;
			}
		}
		SendKPkts2 = (uint32)(PortVFRcvPkts / FLITS_PER_MB / interval);
	}
	return MAX(SendKPkts, SendKPkts2);
}

uint32 computeVFUtilizationPct10(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	uint32 UtilizationPct10;
	uint32 SendMBps, MaxMBps;
	uint32 rate;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	rate = PmCalculateRate(port->Image[imageIndex].u.s.activeSpeed, port->Image[imageIndex].u.s.rxActiveWidth);
	if (rate == IB_STATIC_RATE_1GB) return 0;

	SendMBps = computeVFSendMBps(pm, imageIndex, port, data);

	MaxMBps = s_StaticRateToMBps[rate];

	UtilizationPct10 = SendMBps * 1000 / MaxMBps;

	return UtilizationPct10;
}

uint32 computeVFCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	CongestionWeights_t *weights = (data ? &((PmVFFocusPortComputeData_t *)data)->congestionWeights :
		&pm->congestionWeights);
	uint32 Congestion;
	uint64_t DeltaVLXmitWait = 0;
	uint32 idx, vl, vlmask;
	int vfIdx = (data ? ((PmVFFocusPortComputeData_t *)data)->vfIdx : -1);
	PmCompositeVLCounters_t deltaVFPortCounters, deltaVFPortCounters2;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID ||
		vfIdx == -1) return 0;

	memset(&deltaVFPortCounters, 0, sizeof(PmCompositeVLCounters_t));

	port2 = port->Image[imageIndex].neighbor;

	vlmask = port->Image[imageIndex].vfvlmap[vfIdx].vlmask;
	// Iterate through All VLs within this VF
	for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
		// Skip unassigned VLs (0) in VF
		if (vlmask & 0x1) {
			idx = vl_to_idx(vl);
			deltaVFPortCounters.PortVLXmitData += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitData;
			deltaVFPortCounters.PortVLRcvData += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLRcvData;
			deltaVFPortCounters.PortVLXmitPkts += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitPkts;
			deltaVFPortCounters.PortVLRcvPkts += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLRcvPkts;
			deltaVFPortCounters.PortVLXmitWait += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitWait;
			deltaVFPortCounters.SwPortVLCongestion += port->Image[imageIndex].DeltaStlVLPortCounters[idx].SwPortVLCongestion;
			deltaVFPortCounters.PortVLRcvBECN += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLRcvBECN;
			deltaVFPortCounters.PortVLMarkFECN += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLMarkFECN;
			deltaVFPortCounters.PortVLXmitTimeCong += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitTimeCong;
		}
	}

	uint64 XmitWaitFlitTimes = DeltaVLXmitWait;
	uint64 XmitTimeCongFlitTimes = port->Image[imageIndex].DeltaStlPortCounters.PortXmitTimeCong;
	/* Convert switch wait counter units from cycle time to flit time */
	if (port->pmnodep->nodeType == STL_NODE_SW) {
		XmitWaitFlitTimes = XmitWaitFlitTimes * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
		XmitTimeCongFlitTimes = XmitTimeCongFlitTimes * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
	}
	Congestion = (uint32)(
		(XmitWaitFlitTimes ?
			(XmitWaitFlitTimes * weights->PortXmitWait * 10000) /
			(deltaVFPortCounters.PortVLXmitData + XmitWaitFlitTimes) : 0) +

		(XmitTimeCongFlitTimes ?
			(XmitTimeCongFlitTimes * weights->PortXmitTimeCong * 1000) /
			(deltaVFPortCounters.PortVLXmitData + XmitTimeCongFlitTimes) : 0) +

		(deltaVFPortCounters.PortVLRcvPkts ?
			((port->pmnodep->nodeType == STL_NODE_FI ? 1 : 0) *
			(deltaVFPortCounters.PortVLRcvBECN * weights->PortRcvBECN * 1000)) /
			(deltaVFPortCounters.PortVLRcvPkts) : 0) +

		(deltaVFPortCounters.PortVLXmitPkts ?
			(deltaVFPortCounters.PortVLMarkFECN * weights->PortMarkFECN * 1000) /
			(deltaVFPortCounters.PortVLXmitPkts) : 0) +

		(deltaVFPortCounters.SwPortVLCongestion * weights->SwPortCongestion));

	if (port2) {

		memset(&deltaVFPortCounters2, 0, sizeof(PmCompositeVLCounters_t));

		vlmask = port2->Image[imageIndex].vfvlmap[vfIdx].vlmask;
		// Iterate through All VLs within this VF
		for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1 ) {
			// Skip unassigned VLs (0) in VF
			if (vlmask & 0x1) {
				idx = vl_to_idx(vl);
				deltaVFPortCounters2.PortVLXmitPkts += port2->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitPkts;
				deltaVFPortCounters2.PortVLMarkFECN += port2->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLMarkFECN;
			}
		}
		Congestion += (uint32)(
			(deltaVFPortCounters2.PortVLXmitPkts ?
				(deltaVFPortCounters2.PortVLMarkFECN * weights->PortRcvFECN * 1000) /
				(deltaVFPortCounters2.PortVLXmitPkts) : 0));
	}
	return Congestion;
}

uint32 computeVFBubble(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	uint32 Bubble, Bubble2 = 0;
	uint64 PortVLXmitBubble;
	uint32 idx, vl, vlmask;
	int vfIdx = (data ? ((PmVFFocusPortComputeData_t *)data)->vfIdx : -1);
	PmCompositeVLCounters_t deltaVFPortCounters, deltaVFPortCounters2;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID ||
		vfIdx == -1 ) return 0;

	port2 = port->Image[imageIndex].neighbor;

	memset(&deltaVFPortCounters, 0, sizeof(PmCompositeVLCounters_t));

	vlmask = port->Image[imageIndex].vfvlmap[vfIdx].vlmask;
	// Iterate through All VLs within this VF
	for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
		// Skip unassigned VLs (0) in VF
		if (vlmask & 0x1) {
			idx = vl_to_idx(vl);
			deltaVFPortCounters.PortVLXmitData += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitData;
			deltaVFPortCounters.PortVLXmitWaitData += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitWaitData;
			deltaVFPortCounters.PortVLXmitWastedBW += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitWastedBW;
		}
	}

	PortVLXmitBubble =
		(deltaVFPortCounters.PortVLXmitWastedBW +
		deltaVFPortCounters.PortVLXmitWaitData);

	/* Convert switch wait counter units from cycle time to flit time */
	if (port->pmnodep->nodeType == STL_NODE_SW) {
		PortVLXmitBubble = PortVLXmitBubble * 2 *
			(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
	}
	Bubble = (uint32)(
		(PortVLXmitBubble ? (PortVLXmitBubble * 10000) /
			(deltaVFPortCounters.PortVLXmitData + PortVLXmitBubble) : 0));

	if (port2) {
		memset(&deltaVFPortCounters2, 0, sizeof(PmCompositeVLCounters_t));
		vlmask = port2->Image[imageIndex].vfvlmap[vfIdx].vlmask;
		// Iterate through All VLs within this VF
		for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
			// Skip unassigned VLs (0) in VF
			if (vlmask & 0x1) {
				idx = vl_to_idx(vl);
				deltaVFPortCounters2.PortVLRcvBubble += port2->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLRcvBubble;
				deltaVFPortCounters2.PortVLRcvData += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLRcvData;
				deltaVFPortCounters2.PortVLXmitWastedBW += port->Image[imageIndex].DeltaStlVLPortCounters[idx].PortVLXmitWastedBW;
			}
		}
		uint64 RcvVLBubbleFlitTimes = deltaVFPortCounters2.PortVLRcvBubble;
		/* Convert switch wait counter units from cycle time to flit time */
		if (port->pmnodep->nodeType == STL_NODE_SW) {
			RcvVLBubbleFlitTimes = RcvVLBubbleFlitTimes * 2 *
				(4 / StlLinkWidthToInt(port->Image[imageIndex].u.s.txActiveWidth));
		}
		Bubble2 = (uint32)(
			(RcvVLBubbleFlitTimes ?
			(RcvVLBubbleFlitTimes * 10000) /
			(deltaVFPortCounters2.PortVLRcvData + RcvVLBubbleFlitTimes) : 0));
	}

	return MAX(Bubble, Bubble2);
}
