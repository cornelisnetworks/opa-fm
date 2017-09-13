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

	if (port2)
		SendMBps2 = (uint32)(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvData / FLITS_PER_MB / interval);

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
	uint32 Congestion, vlmask;
	uint64 XmitWait = 0, maxPortVLXmitWait = 0;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;

	port2 = port->Image[imageIndex].neighbor;

	/* Handle PortXmitWait */
	XmitWait = port->Image[imageIndex].DeltaStlPortCounters.PortXmitWait;
	if (pm->flags & STL_PM_PROCESS_VL_COUNTERS && XmitWait) {
		uint8 i, numVLs = 0;
		vlmask = port->Image[imageIndex].vlSelectMask;
		for (i = 0; i < STL_MAX_VLS && vlmask; i++, vlmask >>= 1) {
			if (vlmask & 0x1) {
				UPDATE_MAX(maxPortVLXmitWait, port->Image[imageIndex].DeltaStlVLPortCounters[vl_to_idx(i)].PortVLXmitWait);
				numVLs++;
			}
		}
		// PortXmitWait counter is incremented once if any VLs experienced congestion.
		// It is possible that minimal congestion in sequence could be treated as severe port congestion.
		// So modify PortXmitWait taking into consideration the VL xmit wait counter data.
		XmitWait = (maxPortVLXmitWait * maxPortVLXmitWait * numVLs) / XmitWait;
	}

	Congestion = (uint32)(
		(XmitWait ? (XmitWait * weights->PortXmitWait * 10000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData + XmitWait) : 0)+

		(port->Image[imageIndex].DeltaStlPortCounters.PortXmitTimeCong ?
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitTimeCong * weights->PortXmitTimeCong * 1000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData +
				port->Image[imageIndex].DeltaStlPortCounters.PortXmitTimeCong) : 0) +

		(port->Image[imageIndex].DeltaStlPortCounters.PortRcvPkts ?
			( (port->pmnodep->nodeType == STL_NODE_FI ? 1 : 0) *
				(port->Image[imageIndex].DeltaStlPortCounters.PortRcvBECN * weights->PortRcvBECN * 1000) ) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortRcvPkts) : 0) +

		(port->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts ?
			(port->Image[imageIndex].DeltaStlPortCounters.PortMarkFECN * weights->PortMarkFECN * 1000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts) : 0) +

		(port->Image[imageIndex].DeltaStlPortCounters.SwPortCongestion * weights->SwPortCongestion) );

	if (port2)
		Congestion += (uint32)(
			(port2->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts ?
				(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvFECN * weights->PortRcvFECN * 1000) /
				(port2->Image[imageIndex].DeltaStlPortCounters.PortXmitPkts) : 0) );

	return Congestion;
}
uint32 computeSmaCongestion(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data)
{
	PmPort_t *port2 = NULL;
	CongestionWeights_t *weights = (data ? (CongestionWeights_t *)data : &pm->congestionWeights);
	uint32 SmaCongestion;

	if (port == NULL || imageIndex == PM_IMAGE_INDEX_INVALID) return 0;
	if ((pm->flags & STL_PM_PROCESS_VL_COUNTERS) == 0) return 0;

	port2 = port->Image[imageIndex].neighbor;

	SmaCongestion = (uint32)(
		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitWait ?
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitWait * weights->PortXmitWait * 10000) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitData +
				port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitWait) : 0)+

		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitTimeCong ?
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitTimeCong * weights->PortXmitTimeCong * 1000) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitData +
				port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitTimeCong) : 0) +

		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvPkts ?
			( (port->pmnodep->nodeType == STL_NODE_FI ? 1 : 0) *
				(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvBECN * weights->PortRcvBECN * 1000) ) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLRcvPkts) : 0) +

		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitPkts ?
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLMarkFECN * weights->PortMarkFECN * 1000) /
			(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].PortVLXmitPkts) : 0) +

		(port->Image[imageIndex].DeltaStlVLPortCounters[PM_VL15].SwPortVLCongestion * weights->SwPortCongestion) );

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

	Bubble = (uint32)(
		(PortXmitBubble ? (PortXmitBubble * 10000) /
			(port->Image[imageIndex].DeltaStlPortCounters.PortXmitData + PortXmitBubble) : 0) );

	if (port2)
		Bubble2 = (uint32)(
			(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvBubble ?
				(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvBubble * 10000) /
				(port2->Image[imageIndex].DeltaStlPortCounters.PortRcvData +
					port2->Image[imageIndex].DeltaStlPortCounters.PortRcvBubble) : 0) );

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
