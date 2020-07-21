/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

// low level computational functions
// except where noted, caller of these routines must hold imageLock for write
// for image being computed

#include "pm_topology.h"
#include <limits.h>
#include <iba/stl_helper.h>
#include "stl_print.h" /* PrintDest_t */

extern CounterSelectMask_t LinkDownIgnoreMask;

static void ClearUtilStats(PmUtilStats_t *utilp)
{
	utilp->TotMBps = 0;
	utilp->TotKPps = 0;

	utilp->MaxMBps = 0;
	utilp->MinMBps = IB_UINT32_MAX;

	utilp->MaxKPps = 0;
	utilp->MinKPps = IB_UINT32_MAX;
	MemoryClear(&utilp->BwPorts[0], sizeof(utilp->BwPorts[0]) * STL_PM_UTIL_BUCKETS);
}

static void ClearErrStats(PmErrStats_t *errp)
{
	MemoryClear(errp, sizeof(*errp));
}

// clear stats for a group
void ClearGroupStats(PmGroupImage_t *groupImage)
{
	ClearUtilStats(&groupImage->IntUtil);
	ClearUtilStats(&groupImage->SendUtil);
	ClearUtilStats(&groupImage->RecvUtil);
	ClearErrStats(&groupImage->IntErr);
	ClearErrStats(&groupImage->ExtErr);
	groupImage->MinIntRate = IB_STATIC_RATE_MAX;
	groupImage->MaxIntRate = IB_STATIC_RATE_14G; // = 12.5g for STL;
	groupImage->MinExtRate = IB_STATIC_RATE_MAX;
	groupImage->MaxExtRate = IB_STATIC_RATE_14G; // = 12.5g for STL;
}

// clear stats for a VF
void ClearVFStats(PmVFImage_t *vfImage)
{
	ClearUtilStats(&vfImage->IntUtil);
	ClearErrStats(&vfImage->IntErr);
	vfImage->MinIntRate = IB_STATIC_RATE_MAX;
	vfImage->MaxIntRate = IB_STATIC_RATE_14G; // = 12.5g for STL;
}

// The Update routines take a portSweep argument to reduce overhead
// by letting caller index the PortSweepData once and pass pointer
static void UpdateUtilStats(PmUtilStats_t *utilp, Pm_t *pm, uint32 imageIndex, PmPort_t *port, uint32 imageInterval)
{
	uint32 SendMBps = computeSendMBps(pm, imageIndex, port, (void *)&imageInterval);
	uint32 SendKPkts = computeSendKPkts(pm, imageIndex, port, (void *)&imageInterval);
	uint8 utilBucket = 0;
	if (port != NULL) {
		uint32 rxRate = PmCalculateRate(port->Image[imageIndex].u.s.activeSpeed, port->Image[imageIndex].u.s.rxActiveWidth);
		utilBucket = ComputeUtilBucket(SendMBps, s_StaticRateToMBps[rxRate]);
	}

	utilp->TotMBps += SendMBps;
	utilp->TotKPps += SendKPkts;

	utilp->BwPorts[utilBucket]++;

	UPDATE_MAX(utilp->MaxMBps, SendMBps);
	UPDATE_MIN(utilp->MinMBps, SendMBps);

	UPDATE_MAX(utilp->MaxKPps, SendKPkts);
	UPDATE_MIN(utilp->MinKPps, SendKPkts);
}

// called when both ports in a link are in the given group
static void UpdateErrStatsInGroup(PmErrStats_t *errp, Pm_t *pm, uint32 imageIndex, PmPort_t *port,
	ErrorSummary_t *thresholds, IntegrityWeights_t *iWeights, CongestionWeights_t *cWeights, uint32 imageInterval)
{
	BucketMask_t buckets = {0};

#define UPDATE_MAX_ERROR(stat, data) do { \
		uint32 val = compute##stat(pm, imageIndex, port, data); \
		UPDATE_MAX(errp->Max.stat, val); \
	} while (0)
#define UPDATE_ERROR(stat, data) do { \
		uint32 val = compute##stat(pm, imageIndex, port, data); \
		buckets.s.stat##Bucket = ComputeErrBucket(val, thresholds->stat); \
		UPDATE_MAX(errp->Max.stat, val); \
		errp->Ports[buckets.s.stat##Bucket].stat++; \
	} while (0)
	UPDATE_ERROR(Integrity, iWeights);
	UPDATE_ERROR(Congestion, cWeights);
	UPDATE_ERROR(SmaCongestion, cWeights);
	UPDATE_ERROR(Bubble, NULL);
	UPDATE_ERROR(Security, NULL);
	UPDATE_ERROR(Routing, NULL);
	UPDATE_MAX_ERROR(DiscardsPct10, (void *)&imageInterval);
	UPDATE_MAX_ERROR(UtilizationPct10, (void *)&imageInterval);
#undef UPDATE_ERROR
#undef UPDATE_MAX_ERROR
}

// called when only one of ports in a link are in the given group
static void UpdateErrStatsExtGroup(PmErrStats_t *errp, Pm_t *pm, uint32 imageIndex, PmPort_t *port,
	ErrorSummary_t *thresholds, IntegrityWeights_t *iWeights, CongestionWeights_t *cWeights, uint32 imageInterval)
{
	PmPort_t *port2 = port->Image[imageIndex].neighbor;
	BucketMask_t buckets = {0};
	BucketMask_t buckets2 = {0};

	// for between-group counters we use the worst of the 2 ports
#define UPDATE_MAX_ERROR(stat, data) do { \
		uint32 val = compute##stat(pm, imageIndex, port, data); \
		uint32 val2 = compute##stat(pm, imageIndex, port2, data); \
		UPDATE_MAX(errp->Max.stat, val); \
		UPDATE_MAX(errp->Max.stat, val2); \
	} while (0)
#define UPDATE_ERROR(stat, data) do { \
		uint32 val = compute##stat(pm, imageIndex, port, data); \
		uint32 val2 = compute##stat(pm, imageIndex, port2, data); \
		buckets.s.stat##Bucket = ComputeErrBucket(val, thresholds->stat); \
		buckets2.s.stat##Bucket = ComputeErrBucket(val2, thresholds->stat); \
		UPDATE_MAX(errp->Max.stat, val); \
		UPDATE_MAX(errp->Max.stat, val2); \
		errp->Ports[MAX(buckets.s.stat##Bucket, buckets2.s.stat##Bucket)].stat++; \
	} while (0)
	UPDATE_ERROR(Integrity, iWeights);
	UPDATE_ERROR(Congestion, cWeights);
	UPDATE_ERROR(SmaCongestion, cWeights);
	UPDATE_ERROR(Bubble, NULL);
	UPDATE_ERROR(Security, NULL);
	UPDATE_ERROR(Routing, NULL);
	UPDATE_MAX_ERROR(DiscardsPct10, (void *)&imageInterval);
	UPDATE_MAX_ERROR(UtilizationPct10, (void *)&imageInterval);
#undef UPDATE_ERROR
#undef UPDATE_MAX_ERROR
}

uint32_t PmCalculateRate(uint32_t speed, uint32_t width) {
	switch (speed) {
		case STL_LINK_SPEED_12_5G:
			switch (width) {
			case STL_LINK_WIDTH_1X: return IB_STATIC_RATE_14G; // STL_STATIC_RATE_12_5G;
			case STL_LINK_WIDTH_2X: return IB_STATIC_RATE_25G;
			case STL_LINK_WIDTH_3X: return IB_STATIC_RATE_40G; // STL_STATIC_RATE_37_5G;
			case STL_LINK_WIDTH_4X: return IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G
			default: return IB_STATIC_RATE_1GB; // Invalid!
		}
		case STL_LINK_SPEED_25G:
			switch (width) {
			case STL_LINK_WIDTH_1X: return IB_STATIC_RATE_25G;
			case STL_LINK_WIDTH_2X: return IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G;
			case STL_LINK_WIDTH_3X: return IB_STATIC_RATE_80G; // STL_STATIC_RATE_75G;
			case STL_LINK_WIDTH_4X: return IB_STATIC_RATE_100G;
			default: return IB_STATIC_RATE_1GB; // Invalid!
		}
		default:return IB_STATIC_RATE_1GB; // Invalid!
	}
}

// update stats for a port whose neighbor is also in the group
void UpdateInGroupStats(Pm_t *pm, uint32 imageIndex, PmPort_t *port, PmGroupImage_t *groupImage, uint32 imageInterval)
{
	UpdateUtilStats(&groupImage->IntUtil, pm, imageIndex, port, imageInterval);
	UpdateErrStatsInGroup(&groupImage->IntErr, pm, imageIndex, port, &pm->Thresholds,
		&pm->integrityWeights, &pm->congestionWeights, imageInterval);

	uint32 rxRate = PmCalculateRate(port->Image[imageIndex].u.s.activeSpeed, port->Image[imageIndex].u.s.rxActiveWidth);
	if (s_StaticRateToMBps[rxRate] >= s_StaticRateToMBps[groupImage->MaxIntRate])
		groupImage->MaxIntRate = rxRate;
	if (s_StaticRateToMBps[rxRate] <= s_StaticRateToMBps[groupImage->MinIntRate])
		groupImage->MinIntRate = rxRate;
}

// update stats for a port whose neighbor is not also in the group
void UpdateExtGroupStats(Pm_t *pm, uint32 imageIndex, PmPort_t *port, PmGroupImage_t *groupImage, uint32 imageInterval)
{
	PmPort_t *port2 = port->Image[imageIndex].neighbor;

	// our stats are what portp Sent
	UpdateUtilStats(&groupImage->SendUtil, pm, imageIndex, port, imageInterval);
	// neighbor's stats are what it sent (and portp received)
	UpdateUtilStats(&groupImage->RecvUtil, pm, imageIndex, port2, imageInterval);

	UpdateErrStatsExtGroup(&groupImage->ExtErr, pm, imageIndex, port, &pm->Thresholds,
		&pm->integrityWeights, &pm->congestionWeights, imageInterval);

	uint32 rxRate = PmCalculateRate(port->Image[imageIndex].u.s.activeSpeed, port->Image[imageIndex].u.s.rxActiveWidth);
	if (s_StaticRateToMBps[rxRate] >= s_StaticRateToMBps[groupImage->MaxExtRate])
		groupImage->MaxExtRate = rxRate;
	if (s_StaticRateToMBps[rxRate] <= s_StaticRateToMBps[groupImage->MinExtRate])
		groupImage->MinExtRate = rxRate;
}

// update stats for a port whose neighbor is also in the group
void UpdateVFStats(Pm_t *pm, uint32 imageIndex, PmPort_t *port, PmVFImage_t *vfImage, uint32 imageInterval)
{
	UpdateUtilStats(&vfImage->IntUtil, pm, imageIndex, port, imageInterval);
	UpdateErrStatsInGroup(&vfImage->IntErr, pm, imageIndex, port, &pm->Thresholds,
		&pm->integrityWeights, &pm->congestionWeights, imageInterval);

	uint32 rxRate = PmCalculateRate(port->Image[imageIndex].u.s.activeSpeed, port->Image[imageIndex].u.s.rxActiveWidth);
	if (s_StaticRateToMBps[rxRate] >= s_StaticRateToMBps[vfImage->MaxIntRate])
		vfImage->MaxIntRate = rxRate;
	if (s_StaticRateToMBps[rxRate] <= s_StaticRateToMBps[vfImage->MinIntRate])
		vfImage->MinIntRate = rxRate;
}

// compute averages
void FinalizeGroupStats(PmGroupImage_t *groupImage)
{
	if (groupImage->NumIntPorts) {
		groupImage->IntUtil.AvgMBps = groupImage->IntUtil.TotMBps/groupImage->NumIntPorts;
		groupImage->IntUtil.AvgKPps = groupImage->IntUtil.TotKPps/groupImage->NumIntPorts;
	} else {
		// avoid any possible confusion, remove UINT_MAX value
		groupImage->IntUtil.MinMBps = 0;
		groupImage->IntUtil.MinKPps = 0;
		groupImage->MinIntRate = IB_STATIC_RATE_DONTCARE;
		groupImage->MaxIntRate = IB_STATIC_RATE_DONTCARE;
	}
	if (groupImage->NumExtPorts) {
		groupImage->SendUtil.AvgMBps = groupImage->SendUtil.TotMBps/groupImage->NumExtPorts;
		groupImage->SendUtil.AvgKPps = groupImage->SendUtil.TotKPps/groupImage->NumExtPorts;
		groupImage->RecvUtil.AvgMBps = groupImage->RecvUtil.TotMBps/groupImage->NumExtPorts;
		groupImage->RecvUtil.AvgKPps = groupImage->RecvUtil.TotKPps/groupImage->NumExtPorts;
	} else {
		// avoid any possible confusion, remove UINT_MAX value
		groupImage->SendUtil.MinMBps = 0;
		groupImage->SendUtil.MinKPps = 0;
		groupImage->RecvUtil.MinMBps = 0;
		groupImage->RecvUtil.MinKPps = 0;
		groupImage->MinExtRate = IB_STATIC_RATE_DONTCARE;
		groupImage->MaxExtRate = IB_STATIC_RATE_DONTCARE;
	}
}

// compute averages
void FinalizeVFStats(PmVFImage_t *vfImage)
{
	if (vfImage->NumPorts) {
		vfImage->IntUtil.AvgMBps = vfImage->IntUtil.TotMBps/vfImage->NumPorts;
		vfImage->IntUtil.AvgKPps = vfImage->IntUtil.TotKPps/vfImage->NumPorts;
	} else {
		// avoid any possible confusion, remove UINT_MAX value
		vfImage->IntUtil.MinMBps = 0;
		vfImage->IntUtil.MinKPps = 0;
		vfImage->MinIntRate = IB_STATIC_RATE_DONTCARE;
		vfImage->MaxIntRate = IB_STATIC_RATE_DONTCARE;
	}
}

void PmPrintExceededPort(char *buf, size_t bufSize, PmPort_t *pmportp, uint32 index,
	const char *statistic, uint32 threshold, uint32 value)
{
	PmPort_t *pmportp2 = pmportp->Image[index].neighbor;

	if (pmportp2) {
		snprintf(buf, bufSize, "%s of %u Exceeded Threshold of %u. %.*s Guid "FMT_U64" LID 0x%x Port %u."
			"  Neighbor: %.*s Guid "FMT_U64" LID 0x%x Port %u.",
			statistic, value, threshold,
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[index].lid, pmportp->portNum,
			(int)sizeof(pmportp2->pmnodep->nodeDesc.NodeString), pmportp2->pmnodep->nodeDesc.NodeString,
			pmportp2->pmnodep->NodeGUID, pmportp2->pmnodep->Image[index].lid, pmportp2->portNum);
	} else {
		snprintf(buf, bufSize, "%s of %u Exceeded Threshold of %u. %.*s Guid "FMT_U64" LID 0x%x Port %u.",
			statistic, value, threshold,
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString),pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[index].lid, pmportp->portNum);
	}
}

#define GET_DELTA_COUNTER(cntr) \
	(portImage->DeltaStlPortCounters.cntr)
#define GET_NEIGHBOR_DELTA_COUNTER(cntr) \
	(portImageNeighbor ? portImageNeighbor->DeltaStlPortCounters.cntr : 0)
#define GET_DELTA_VLCOUNTER(vlcntr, vl) \
	(portImage->DeltaStlVLPortCounters[vl].vlcntr)
#define GET_NEIGHBOR_DELTA_VLCOUNTER(vlcntr, vl) \
	(portImageNeighbor ? portImageNeighbor->DeltaStlVLPortCounters[vl].vlcntr : 0)

static __inline void
FormatStlPortErrorInfo(char *buf, size_t bufsize, PmCompositeErrorInfo_t *err, STLErrorInfoMask_t *mask) {
	int i;
	PrintDest_t str;
	PrintDestInitBuffer(&str, buf, bufsize);

	// If mask is given and no bits are set return.
	if (mask && mask->AsReg32 == 0) return;

	// If mask is not given assume print any with a valid status.
	if ((!mask || mask->s.UncorrectableErrorInfo) && err->UncorrectableErrorInfo.s.Status) {
		PrintFunc(&str, "UncErr[%u:%s] ", err->UncorrectableErrorInfo.s.ErrorCode,
			UncorrectableErrorInfoToText(err->UncorrectableErrorInfo.s.ErrorCode));
	}

	if ((!mask || mask->s.PortRcvErrorInfo) && err->PortRcvErrorInfo.s.Status) {
		PrintFunc(&str, "RcvErr[%u:%s]", err->PortRcvErrorInfo.s.ErrorCode,
			PortRcvErrorInfoToText(err->PortRcvErrorInfo.s.ErrorCode));

		if (err->PortRcvErrorInfo.s.ErrorCode == 1
			|| (err->PortRcvErrorInfo.s.ErrorCode >= 4 && err->PortRcvErrorInfo.s.ErrorCode <= 12))
		{
			// Flit 1
			PrintFunc(&str, "={F1=0x");
			for (i = 0; i < 8; i++) {
				PrintFunc(&str, "%02x", err->PortRcvErrorInfo.ErrorInfo.EI1to12.PacketFlit1[i]);
			}
			PrintFunc(&str, ":%u", err->PortRcvErrorInfo.ErrorInfo.EI1to12.s.Flit1Bits);
			//Flit 2
			PrintFunc(&str, ";F2=0x");
			for (i = 0; i < 8; i++) {
				PrintFunc(&str, "%02x", err->PortRcvErrorInfo.ErrorInfo.EI1to12.PacketFlit2[i]);
			}
			PrintFunc(&str, ":%u} ", err->PortRcvErrorInfo.ErrorInfo.EI1to12.s.Flit2Bits);
		} else if (err->PortRcvErrorInfo.s.ErrorCode == 13) {
			PrintFunc(&str, "={F=0x");
			for (i = 0; i < 8; i++) {
				PrintFunc(&str, "%02x", err->PortRcvErrorInfo.ErrorInfo.EI13.PacketBytes[i]);
			}
			PrintFunc(&str, ":%u} ", err->PortRcvErrorInfo.ErrorInfo.EI13.s.FlitBits);
		} else {
			PrintFunc(&str, " ");
		}
	}

	if ((!mask || mask->s.ExcessiveBufferOverrunInfo) && err->ExcessiveBufferOverrunInfo.s.Status) {
		PrintFunc(&str, "ExsBufOvrn={SC=%u} ", err->ExcessiveBufferOverrunInfo.s.SC);
	}

	if ((!mask || mask->s.FMConfigErrorInfo) && err->FMConfigErrorInfo.s.Status) {
		PrintFunc(&str, "FmCfgErr[%u:%s]", err->FMConfigErrorInfo.s.ErrorCode,
			FMConfigErrorInfoToText(err->FMConfigErrorInfo.s.ErrorCode));
		switch (err->FMConfigErrorInfo.s.ErrorCode) {
		case 0:
		case 1:
		case 2:
		case 8:
			PrintFunc(&str, "={Distance=%u} ", err->FMConfigErrorInfo.ErrorInfo.EI0to2_8.Distance);
			break;
		case 3:
		case 4:
		case 5:
			PrintFunc(&str, "={VL=%u} ", err->FMConfigErrorInfo.ErrorInfo.EI3to5.VL);
			break;
		case 6:
			PrintFunc(&str, "={BadBits=%x} ", err->FMConfigErrorInfo.ErrorInfo.EI6.BadFlitBits);
			break;
		case 7:
			PrintFunc(&str, "={SC=%x} ", err->FMConfigErrorInfo.ErrorInfo.EI7.SC);
			break;
		default:
			PrintFunc(&str, " ");
		}
	}

	if ((!mask || mask->s.PortRcvSwitchRelayErrorInfo) && err->PortRcvSwitchRelayErrorInfo.s.Status) {
		PrintFunc(&str, "RxSwRlyErr[%u:%s]",
			err->PortRcvSwitchRelayErrorInfo.s.ErrorCode,
			PortRcvSwitchRelayErrorInfoToText(err->PortRcvSwitchRelayErrorInfo.s.ErrorCode));
		switch (err->PortRcvSwitchRelayErrorInfo.s.ErrorCode) {
		case 0:
			PrintFunc(&str, "={DLID=0x%x;SLID=0x%x;SC=%u;RC=%u} ",
				err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI0.DLID,
				((err->PortRcvSwitchRelayErrorInfo.SLID_23_16 << 16)
					| err->PortRcvSwitchRelayErrorInfo.SLID_15_0),
				err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI0.SC,
				err->PortRcvSwitchRelayErrorInfo.s.RC);
			break;
		case 2:
			PrintFunc(&str, "={DLID=0x%x;SLID=0x%x;EPortNum=%u;RC=%u} ",
				((err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI2.DLID_23_16 << 16)
					| err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI2.DLID_15_0),
				((err->PortRcvSwitchRelayErrorInfo.SLID_23_16 << 16)
					| err->PortRcvSwitchRelayErrorInfo.SLID_15_0),
				err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI2.EgressPortNum,
				err->PortRcvSwitchRelayErrorInfo.s.RC);
			break;
		case 3:
			PrintFunc(&str, "={EPortNum=%u;SC=%u} ",
				err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI3.EgressPortNum,
				err->PortRcvSwitchRelayErrorInfo.ErrorInfo.EI3.SC);
			break;
		default:
			PrintFunc(&str, " ");
		}
	}

	if ((!mask || mask->s.PortXmitConstraintErrorInfo) && err->PortXmitConstraintErrorInfo.s.Status) {
		PrintFunc(&str, "TxCnstErr={pkey=0x%04x;SLID=0x%x} ",
			err->PortXmitConstraintErrorInfo.P_Key, err->PortXmitConstraintErrorInfo.SLID);
	}

	if ((!mask || mask->s.PortRcvConstraintErrorInfo) && err->PortRcvConstraintErrorInfo.s.Status) {
		PrintFunc(&str, "RxCnstErr[%u:%s]={pkey=0x%04x;SLID=0x%x} ",
			err->PortRcvConstraintErrorInfo.s.ErrorCode,
			PortRcvConstraintErrorInfoToText(err->PortRcvConstraintErrorInfo.s.ErrorCode),
			err->PortRcvConstraintErrorInfo.P_Key, err->PortRcvConstraintErrorInfo.SLID);
	}
}

static void PmPrintExceededPortDetailsErrorInfo(PmPort_t *pmportp,  uint32 imageIndex, STLErrorInfoMask_t mask, char *buf)
{

	// If one reg has a valid status and can be printed then print port
	if (mask.AsReg32) {
		IB_LOG_WARN_FMT(NULL, "ErrorInfo was set on %.*s Guid "FMT_U64" LID 0x%x Port %u: %s",
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), (char *)pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum, buf);
	} else {
		IB_LOG_INFO_FMT(NULL, "ErrorInfo was set on %.*s Guid "FMT_U64" LID 0x%x Port %u: No data found",
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), (char *)pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
	}
}

void PmPrintExceededPortDetailsIntegrity(char *exceededMessage, Pm_t *pm, PmPort_t *pmportp,
	PmPort_t *pmportneighborp, uint32 imageIndex, uint8 printErrorInfo)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmPortImage_t *portImageNeighbor = (pmportneighborp ? &pmportneighborp->Image[imageIndex] : NULL);

	char message[256] = {0};
	char * logMessage = message;
	int buffSpace = sizeof(message);

	uint64 LocalLinkIntegrityErrors = GET_DELTA_COUNTER(LocalLinkIntegrityErrors);
	uint64 PortRcvErrors            = GET_DELTA_COUNTER(PortRcvErrors);
	uint32 LinkErrorRecovery        = GET_DELTA_COUNTER(LinkErrorRecovery);
	uint32 LinkDowned               = GET_DELTA_COUNTER(LinkDowned);
	uint8  LinkQualityIndicator     = portImage->StlPortCounters.lq.s.LinkQualityIndicator;
	uint32 LwdTx                    = StlLinkWidthToInt(portImage->u.s.txActiveWidth);
	uint32 LwdRx                    = StlLinkWidthToInt(portImage->u.s.rxActiveWidth);
	uint8  UncorrectableErrors      = GET_DELTA_COUNTER(UncorrectableErrors);
	uint64 FMConfigErrors           = GET_DELTA_COUNTER(FMConfigErrors);
	uint64 ExcessiveBufferOverruns  = GET_NEIGHBOR_DELTA_COUNTER(ExcessiveBufferOverruns);

	//for each counter, determine if it contributed to error condition
	if (LocalLinkIntegrityErrors && pm->integrityWeights.LocalLinkIntegrityErrors) {
		snprintf(logMessage, buffSpace, "LLI=%"PRIu64" ", LocalLinkIntegrityErrors);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (PortRcvErrors && pm->integrityWeights.PortRcvErrors) {
		snprintf(logMessage, buffSpace, "RxE=%"PRIu64" ", PortRcvErrors);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (LinkErrorRecovery && pm->integrityWeights.LinkErrorRecovery) {
		snprintf(logMessage, buffSpace, "LER=%u ", LinkErrorRecovery);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (LinkDowned && pm->integrityWeights.LinkDowned) {
		snprintf(logMessage, buffSpace, "LD=%u ", LinkDowned);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (LinkQualityIndicator < STL_LINKQUALITY_EXCELLENT && pm->integrityWeights.LinkQualityIndicator) {
		snprintf(logMessage, buffSpace, "LQI=%u ", LinkQualityIndicator);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if ((LwdTx < StlLinkWidthToInt(STL_LINK_WIDTH_4X) || LwdRx < StlLinkWidthToInt(STL_LINK_WIDTH_4X)) && pm->integrityWeights.LinkWidthDowngrade) {
		snprintf(logMessage, buffSpace, "LWD.Tx=%ux Rx=%ux ", LwdTx, LwdRx);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (UncorrectableErrors && pm->integrityWeights.UncorrectableErrors) {
		snprintf(logMessage, buffSpace, "Unc=%u ", UncorrectableErrors);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (FMConfigErrors && pm->integrityWeights.FMConfigErrors) {
		snprintf(logMessage, buffSpace, "FMC=%"PRIu64" ", FMConfigErrors);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (ExcessiveBufferOverruns && pm->integrityWeights.ExcessiveBufferOverruns) {
		snprintf(logMessage, buffSpace, "neighbor EBO=%"PRIu64" ", ExcessiveBufferOverruns);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	IB_LOG_WARN_FMT(NULL, "%s Details: %s", exceededMessage, message);

	if (printErrorInfo) {
		if (portImage->u.s.gotErrorInfo) {
			char buf[256] = {0};
			STLErrorInfoMask_t mask = {0};

			mask.s.PortRcvErrorInfo = portImage->ErrorInfo.PortRcvErrorInfo.s.Status;
			mask.s.UncorrectableErrorInfo = portImage->ErrorInfo.UncorrectableErrorInfo.s.Status;
			mask.s.FMConfigErrorInfo = portImage->ErrorInfo.FMConfigErrorInfo.s.Status;

			FormatStlPortErrorInfo(buf, sizeof(buf), &portImage->ErrorInfo, &mask);
			PmPrintExceededPortDetailsErrorInfo(pmportp, imageIndex, mask, buf);
		}

		if (portImageNeighbor && portImageNeighbor->u.s.gotErrorInfo) {
			char buf[256] = {0};
			STLErrorInfoMask_t mask = {0};

			mask.s.ExcessiveBufferOverrunInfo = portImageNeighbor->ErrorInfo.ExcessiveBufferOverrunInfo.s.Status;

			FormatStlPortErrorInfo(buf, sizeof(buf), &portImageNeighbor->ErrorInfo, &mask);
			PmPrintExceededPortDetailsErrorInfo(pmportneighborp, imageIndex, mask, buf);
		}
	}
}

void PmPrintExceededPortDetailsCongestion(char *exceededMessage, Pm_t *pm, PmPort_t *pmportp,
	PmPort_t *pmportneighborp, uint32 imageIndex, uint8 printErrorInfo)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmPortImage_t *portImageNeighbor = (pmportneighborp ? &pmportneighborp->Image[imageIndex] : NULL);
	char message[256] = {0};
	char * logMessage = message;
	int buffSpace = sizeof(message);
	uint64 DeltaXmitData = GET_DELTA_COUNTER(PortXmitData);
	uint64 DeltaXmitPkts = GET_DELTA_COUNTER(PortXmitPkts);
	uint64 DeltaRcvPkts = GET_DELTA_COUNTER(PortRcvPkts);
	uint64 DeltaXmitPkts_N = GET_NEIGHBOR_DELTA_COUNTER(PortXmitPkts);
	uint64 DeltaXmitWait = GET_DELTA_COUNTER(PortXmitWait);
	uint64 DeltaXmitTimeCong = GET_DELTA_COUNTER(PortXmitTimeCong);
	uint64 DeltaRcvBECN = GET_DELTA_COUNTER(PortRcvBECN);
	uint64 DeltaMarkFECN = GET_DELTA_COUNTER(PortMarkFECN);
	uint64 DeltaRcvFECN_N = GET_NEIGHBOR_DELTA_COUNTER(PortRcvFECN);
	uint64 DeltaSwPortCong = GET_DELTA_COUNTER(SwPortCongestion);
	uint32 XmitWaitPct, XmitTimeCongPct;

	if ((pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) && DeltaXmitWait) {
		/* If VlXmitWait counters are available, use worst VL to properly weight the
		 * port-level counter to prevent low levels of congestion appearing as severe congestion.
		 */
		uint64_t MaxDeltaVLXmitWait = 0;
		uint32_t i, vlmask = portImage->vlSelectMask;
		for (i = 0; i < STL_MAX_VLS && vlmask; i++, vlmask >>= 1) {
			if (vlmask & 0x1) {
				UPDATE_MAX(MaxDeltaVLXmitWait, GET_DELTA_VLCOUNTER(PortVLXmitWait, vl_to_idx(i)));
			}
		}
		DeltaXmitWait = (uint64_t)((double)MaxDeltaVLXmitWait * (double)MaxDeltaVLXmitWait / (double)DeltaXmitWait);
	}

	/* Convert switch wait counter units from cycle time to flit time */
	if (pmportp->pmnodep->nodeType == STL_NODE_SW) {
		DeltaXmitWait = DeltaXmitWait * 2 *
			(4 / StlLinkWidthToInt(portImage->u.s.txActiveWidth));
		DeltaXmitTimeCong = DeltaXmitTimeCong * 2 *
			(4 / StlLinkWidthToInt(portImage->u.s.txActiveWidth));
	}
	XmitWaitPct = (uint32)(DeltaXmitWait ?
		(DeltaXmitWait * 10000) / (DeltaXmitWait + DeltaXmitData) : 0);
	XmitTimeCongPct = (uint32)(DeltaXmitTimeCong ?
		(DeltaXmitTimeCong * 1000) / (DeltaXmitTimeCong + DeltaXmitData) : 0);

	uint32 RcvFECNPct = (uint32)(DeltaXmitPkts_N ?
		(DeltaRcvFECN_N * 1000) / (DeltaXmitPkts_N) : 0);

	uint32 RcvBECNPct = (uint32)(DeltaRcvPkts ?
		(DeltaRcvBECN * 1000 * (pmportp->pmnodep->nodeType & STL_NODE_FI)) / (DeltaRcvPkts) : 0);
	uint32 MarkFECNPct = (uint32)(DeltaXmitPkts ?
		(DeltaMarkFECN * 1000) / (DeltaXmitPkts) : 0);

	if (DeltaXmitWait && pm->congestionWeights.PortXmitWait){
		snprintf(logMessage, buffSpace, "TxW=%"PRIu64" TxWPct=%u ", DeltaXmitWait, XmitWaitPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaSwPortCong && pm->congestionWeights.SwPortCongestion){
		snprintf(logMessage, buffSpace, "CD=%"PRIu64" ", DeltaSwPortCong);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaRcvBECN && pm->congestionWeights.PortRcvBECN){
		snprintf(logMessage, buffSpace, "RxB=%"PRIu64" RxBPct=%u ", DeltaRcvBECN, RcvBECNPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaXmitTimeCong && pm->congestionWeights.PortXmitTimeCong){
		snprintf(logMessage, buffSpace, "TxTC=%"PRIu64" TxTCPct=%u ", DeltaXmitTimeCong, XmitTimeCongPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaMarkFECN && pm->congestionWeights.PortMarkFECN){
		snprintf(logMessage, buffSpace, "MkF=%"PRIu64" MkFPct=%u ", DeltaMarkFECN, MarkFECNPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaRcvFECN_N && pm->congestionWeights.PortRcvFECN){
		snprintf(logMessage, buffSpace, "neighbor RxF=%"PRIu64" neighbor RxFPct=%u ", DeltaRcvFECN_N, RcvFECNPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}

	IB_LOG_WARN_FMT(NULL, "%s Details: %s", exceededMessage, message);
}

void PmPrintExceededPortDetailsSmaCongestion(char *exceededMessage, Pm_t *pm, PmPort_t *pmportp,
	PmPort_t *pmportneighborp, uint32 imageIndex, uint8 printErrorInfo)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmPortImage_t *portImageNeighbor = (pmportneighborp ? &pmportneighborp->Image[imageIndex] : NULL);

	char message[256] = {0};
	char * logMessage = message;
	int buffSpace = sizeof(message);
	uint64 DeltaVLXmitData = GET_DELTA_VLCOUNTER(PortVLXmitData, PM_VL15);
	uint64 DeltaVLXmitPkts = GET_DELTA_VLCOUNTER(PortVLXmitPkts, PM_VL15);
	uint64 DeltaVLRcvPkts = GET_DELTA_VLCOUNTER(PortVLRcvPkts, PM_VL15);
	uint64 DeltaVLXmitPkts_N = GET_NEIGHBOR_DELTA_VLCOUNTER(PortVLXmitPkts, PM_VL15);
	uint64 DeltaVLXmitWait = GET_DELTA_VLCOUNTER(PortVLXmitWait, PM_VL15);
	uint64 DeltaVLXmitTimeCong = GET_DELTA_VLCOUNTER(PortVLXmitTimeCong, PM_VL15);
	uint64 DeltaVLRcvBECN = GET_DELTA_VLCOUNTER(PortVLRcvBECN, PM_VL15);
	uint64 DeltaVLMarkFECN = GET_DELTA_VLCOUNTER(PortVLMarkFECN, PM_VL15);
	uint64 DeltaVLRcvFECN_N = GET_NEIGHBOR_DELTA_VLCOUNTER(PortVLRcvFECN, PM_VL15);
	uint64 DeltaVLSwPortCong = GET_DELTA_VLCOUNTER(SwPortVLCongestion, PM_VL15);
	uint32 VLXmitWaitPct, VLXmitTimeCongPct;

	/* Convert switch wait counter units from cycle time to flit time */
	if (pmportp->pmnodep->nodeType == STL_NODE_SW) {
		DeltaVLXmitWait = DeltaVLXmitWait * 2 *
			(4 / StlLinkWidthToInt(portImage->u.s.txActiveWidth));
		DeltaVLXmitTimeCong = DeltaVLXmitTimeCong * 2 *
			(4 / StlLinkWidthToInt(portImage->u.s.txActiveWidth));
	}
	VLXmitWaitPct = (uint32)(DeltaVLXmitWait ?
		(DeltaVLXmitWait * 10000) / (DeltaVLXmitWait + DeltaVLXmitData) : 0);
	VLXmitTimeCongPct = (uint32)(DeltaVLXmitTimeCong ?
		(DeltaVLXmitTimeCong * 1000) / (DeltaVLXmitTimeCong + DeltaVLXmitData) : 0);

	uint32 VLRcvFECNPct = (uint32)(DeltaVLXmitPkts_N ?
		(DeltaVLRcvFECN_N * 1000) / (DeltaVLXmitPkts_N) : 0);

	uint32 VLRcvBECNPct = (uint32)(DeltaVLRcvPkts ?
		(DeltaVLRcvBECN * 1000 * (pmportp->pmnodep->nodeType & STL_NODE_FI)) / (DeltaVLRcvPkts) : 0);
	uint32 VLMarkFECNPct = (uint32)(DeltaVLXmitPkts ?
		(DeltaVLMarkFECN * 1000) / (DeltaVLXmitPkts) : 0);

	if (DeltaVLXmitWait && pm->congestionWeights.PortXmitWait){
		snprintf(logMessage, buffSpace, "VLTxW[15]=%"PRIu64" VLTxWPct[15]=%u ", DeltaVLXmitWait, VLXmitWaitPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaVLSwPortCong && pm->congestionWeights.SwPortCongestion){
		snprintf(logMessage, buffSpace, "VLCD[15]=%"PRIu64" ", DeltaVLSwPortCong);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaVLRcvBECN && pm->congestionWeights.PortRcvBECN){
		snprintf(logMessage, buffSpace, "VLRxB[15]=%"PRIu64" VLRxBPct[15]=%u ", DeltaVLRcvBECN, VLRcvBECNPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaVLXmitTimeCong && pm->congestionWeights.PortXmitTimeCong){
		snprintf(logMessage, buffSpace, "VLTxTC[15]=%"PRIu64" VLTxTCPct[15]=%u ", DeltaVLXmitTimeCong, VLXmitTimeCongPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaVLMarkFECN && pm->congestionWeights.PortMarkFECN){
		snprintf(logMessage, buffSpace, "VLMkF[15]=%"PRIu64" VLMkFPct[15]=%u ", DeltaVLMarkFECN, VLMarkFECNPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}
	if (DeltaVLRcvFECN_N && pm->congestionWeights.PortRcvFECN){
		snprintf(logMessage, buffSpace, "neighbor VLRxF[15]=%"PRIu64" neighbor VLRxFPct[15]=%u ", DeltaVLRcvFECN_N, VLRcvFECNPct);
		buffSpace -= strlen(logMessage);
		logMessage += strlen(logMessage);
	}

	IB_LOG_WARN_FMT(NULL, "%s Details: %s", exceededMessage, message);
}

void PmPrintExceededPortDetailsBubble(char *exceededMessage, Pm_t *pm, PmPort_t *pmportp,
	PmPort_t *pmportneighborp, uint32 imageIndex, uint8 printErrorInfo)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmPortImage_t *portImageNeighbor = (pmportneighborp ? &pmportneighborp->Image[imageIndex] : NULL);
	uint64 DeltaXmitData = GET_DELTA_COUNTER(PortXmitData);
	uint64 DeltaRcvData_N = GET_NEIGHBOR_DELTA_COUNTER(PortRcvData);
	uint64 DeltaXmitWastedBW = GET_DELTA_COUNTER(PortXmitWastedBW);
	uint64 DeltaXmitWaitData = GET_DELTA_COUNTER(PortXmitWaitData);
	uint64 DeltaRcvBubble_N = GET_NEIGHBOR_DELTA_COUNTER(PortRcvBubble);
	uint64 DeltaXmitBubble = DeltaXmitWastedBW + DeltaXmitWaitData;
	uint32 XmitBubblePct, RcvBubblePct;

	/* Convert switch wait counter units from cycle time to flit time */
	if (pmportp->pmnodep->nodeType == STL_NODE_SW) {
		DeltaXmitBubble = DeltaXmitBubble * 2 *
			(4 / StlLinkWidthToInt(portImage->u.s.txActiveWidth));
	}
	if (pmportneighborp && pmportneighborp->pmnodep->nodeType == STL_NODE_SW) {
		DeltaRcvBubble_N = DeltaRcvBubble_N * 2 *
			(4 / StlLinkWidthToInt(portImageNeighbor->u.s.txActiveWidth));
	}
	XmitBubblePct = (uint32)(DeltaXmitBubble ?
		(DeltaXmitBubble * 10000) / (DeltaXmitData + DeltaXmitBubble) : 0);
	RcvBubblePct = (uint32)(DeltaRcvBubble_N ?
		(DeltaRcvBubble_N * 10000) / (DeltaRcvData_N + DeltaRcvBubble_N) : 0);

	IB_LOG_WARN_FMT(NULL, "%s Details: WBW=%"PRIu64", TxWD=%"PRIu64", TxBbPct=%u, neighbor RxBb=%"PRIu64" neighbor RxBbPct=%u ",
		exceededMessage, DeltaXmitWastedBW, DeltaXmitWaitData, XmitBubblePct, DeltaRcvBubble_N, RcvBubblePct);
}

void PmPrintExceededPortDetailsSecurity(char *exceededMessage, Pm_t *pm, PmPort_t *pmportp,
	PmPort_t *pmportneighborp, uint32 imageIndex, uint8 printErrorInfo)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmPortImage_t *portImageNeighbor = (pmportneighborp ? &pmportneighborp->Image[imageIndex] : NULL);

	IB_LOG_WARN_FMT(NULL, "%s Details: TxCE=%"PRIu64", neighbor RxCE=%"PRIu64" ", exceededMessage,
		GET_DELTA_COUNTER(PortXmitConstraintErrors),
		GET_NEIGHBOR_DELTA_COUNTER(PortRcvConstraintErrors));

	if (printErrorInfo) {
		if (portImage->u.s.gotErrorInfo) {
			char buf[256] = {0};
			STLErrorInfoMask_t mask = {0};

			mask.s.PortXmitConstraintErrorInfo = portImage->ErrorInfo.PortXmitConstraintErrorInfo.s.Status;
			FormatStlPortErrorInfo(buf, sizeof(buf), &portImage->ErrorInfo, &mask);
			PmPrintExceededPortDetailsErrorInfo(pmportp, imageIndex, mask, buf);
		}

		if (portImageNeighbor && portImageNeighbor->u.s.gotErrorInfo) {
			char buf[256] = {0};
			STLErrorInfoMask_t mask = {0};

			mask.s.PortRcvConstraintErrorInfo =
				portImageNeighbor->ErrorInfo.PortRcvConstraintErrorInfo.s.Status;
			FormatStlPortErrorInfo(buf, sizeof(buf), &portImageNeighbor->ErrorInfo, &mask);
			PmPrintExceededPortDetailsErrorInfo(pmportneighborp, imageIndex, mask, buf);
		}
	}
}

void PmPrintExceededPortDetailsRouting(char *exceededMessage, Pm_t *pm, PmPort_t *pmportp,
	PmPort_t *pmportneighborp, uint32 imageIndex, uint8 printErrorInfo)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];

	IB_LOG_WARN_FMT(NULL, "%s Details: RxSR=%"PRIu64" ", exceededMessage,
		GET_DELTA_COUNTER(PortRcvSwitchRelayErrors));

	if (printErrorInfo && portImage->u.s.gotErrorInfo) {
		char buf[256] = {0};
		STLErrorInfoMask_t mask = {0};

		mask.s.PortRcvSwitchRelayErrorInfo = portImage->ErrorInfo.PortRcvSwitchRelayErrorInfo.s.Status;
		FormatStlPortErrorInfo(buf, sizeof(buf), &portImage->ErrorInfo, &mask);
		PmPrintExceededPortDetailsErrorInfo(pmportp, imageIndex, mask, buf);
	}
}

#undef GET_DELTA_COUNTER
#undef GET_NEIGHBOR_DELTA_COUNTER
#undef GET_DELTA_VLCOUNTER
#undef GET_NEIGHBOR_DELTA_VLCOUNTER

static void PmUnexpectedClear(Pm_t *pm, PmPort_t *pmportp, uint32 imageIndex,
	CounterSelectMask_t unexpectedClear)
{
	PmImage_t *pmimagep = &pm->Image[imageIndex];
	PmNode_t *pmnodep = pmportp->pmnodep;
	char *detail = "";
	detail=": Make sure no other tools are clearing fabric counters";
	char CounterNameBuffer[128];
	FormatStlCounterSelectMask(CounterNameBuffer, unexpectedClear);

	if (pmimagep->NoRespNodes + pmimagep->NoRespPorts
		+ pmimagep->UnexpectedClearPorts < pm_config.SweepErrorsLogThreshold) {
		IB_LOG_WARN_FMT(NULL, "Unexpected counter clear for %.*s Guid "FMT_U64" LID 0x%x Port %u%s (Mask 0x%08x: %s)",
			(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
			pmnodep->NodeGUID, pmnodep->Image[imageIndex].lid, pmportp->portNum, detail,
			unexpectedClear.AsReg32, CounterNameBuffer);
	} else {
		IB_LOG_INFO_FMT(NULL, "Unexpected counter clear for %.*s Guid "FMT_U64" LID 0x%x Port %u%s (Mask 0x%08x: %s)",
			(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
			pmnodep->NodeGUID, pmnodep->Image[imageIndex].lid, pmportp->portNum, detail,
			unexpectedClear.AsReg32, CounterNameBuffer);
	}
	pmimagep->UnexpectedClearPorts++;
	INCREMENT_PM_COUNTER(pmCounterPmUnexpectedClearPorts);
}

// After all individual ports have been tabulated, we tabulate totals for
// all groups.  We must do this after port tabulation because some counters
// need to look at both sides of a link to pick the max or combine error
// counters.  Hence we could not accurately indicate buckets nor totals for a
// group until first pass through all ports has been completed.
// We also compute RunningTotals here because caller will have the appropriate
// locks.
//
// caller must hold imageLock for write on this image (index)
// and totalsLock for write and imageLock held for write
void PmFinalizePortStats(Pm_t *pm, PmPort_t *pmportp, uint32 index)
{
	PmPortImage_t *portImage = &pmportp->Image[index];

	PmCompositePortCounters_t *pRunning = &pmportp->StlPortCountersTotal;
	PmCompositeVLCounters_t *pVLRunning = &pmportp->StlVLPortCountersTotal[0];

	PmCompositePortCounters_t *pImgPortCounters = &portImage->StlPortCounters;
	PmCompositeVLCounters_t *pImgPortVLCounters = &portImage->StlVLPortCounters[0];

	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID) {
		// Initialize LQI
		pRunning->lq.s.LinkQualityIndicator = pImgPortCounters->lq.s.LinkQualityIndicator;
		return;
	}
	int i;
	// Local Ports Previous Image
	uint32 imageIndexPrevious = pm->LastSweepIndex;
	PmPortImage_t *portImagePrev = &pmportp->Image[imageIndexPrevious];
	PmCompositePortCounters_t *pImgPortCountersPrev = &portImagePrev->StlPortCounters;
	PmCompositeVLCounters_t *pImgPortVLCountersPrev = &portImagePrev->StlVLPortCounters[0];
	// Counter Select Mask of Counter cleared by the PM during LastSweep
	CounterSelectMask_t prevPortClrSelMask = portImagePrev->clearSelectMask;

	// Delta's used for Category Calulations and Running Counters
	PmCompositePortCounters_t *pDeltaPortCounters = &portImage->DeltaStlPortCounters;
	PmCompositeVLCounters_t *pDeltaVLCounters = &portImage->DeltaStlVLPortCounters[0];

	CounterSelectMask_t unexpectedClear = {0};

	IB_LOG_DEBUG3_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x Port %u",
		(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
		pmportp->pmnodep->NodeGUID, pmportp->pmnodep->dlid, pmportp->portNum);

	// If LinkWidth.Active is greater than LinkWidthDowngrade.txActive then port is downgraded
	if (pImgPortCounters->lq.s.NumLanesDown) {
		pm->Image[index].DowngradedPorts++;
	}
	// If this flag is still set at this point, then we have a valid ErrorInfo on the port.
	if (portImage->u.s.gotErrorInfo) {
		pm->Image[index].ErrorInfoPorts++;
	}
	if (portImage->u.s.gotDataCntrs) {
		// Copy MIN LQI into Delta Struct
		pDeltaPortCounters->lq.s.LinkQualityIndicator =
			MIN(pImgPortCounters->lq.s.LinkQualityIndicator,
				pImgPortCountersPrev->lq.s.LinkQualityIndicator);
		// Copy MAX Number of Lanes (LWD) into Delta Struct
		pDeltaPortCounters->lq.s.NumLanesDown =
			MAX(pImgPortCounters->lq.s.NumLanesDown,
				pImgPortCountersPrev->lq.s.NumLanesDown);

#define GET_DELTA_PORTCOUNTERS(cntr) do { \
		if (pImgPortCounters->cntr < pImgPortCountersPrev->cntr || prevPortClrSelMask.s.cntr) { \
			unexpectedClear.s.cntr = !prevPortClrSelMask.s.cntr; \
			pDeltaPortCounters->cntr = pImgPortCounters->cntr; \
		} else { \
			pDeltaPortCounters->cntr = pImgPortCounters->cntr - pImgPortCountersPrev->cntr; \
		} } while (0)

#define GET_DELTA_VLCOUNTERS(vlcntr, vl, cntr) do { \
		if (pImgPortVLCounters[vl].vlcntr < pImgPortVLCountersPrev[vl].vlcntr || prevPortClrSelMask.s.cntr) { \
			unexpectedClear.s.cntr = !prevPortClrSelMask.s.cntr; \
			pDeltaVLCounters[vl].vlcntr = pImgPortVLCounters[vl].vlcntr; \
		} else { \
			pDeltaVLCounters[vl].vlcntr = pImgPortVLCounters[vl].vlcntr - pImgPortVLCountersPrev[vl].vlcntr; \
		} } while (0)

// Copy Previous Image Counters if Counters were not queried this sweep
#define COPY_PORTCOUNTERS(cntr) \
		pImgPortCounters->cntr = pImgPortCountersPrev->cntr
#define COPY_VLCOUNTERS(vlcntr, vl) \
		pImgPortVLCounters[vl].vlcntr = pImgPortVLCountersPrev[vl].vlcntr

		{
			GET_DELTA_PORTCOUNTERS(SwPortCongestion);
			GET_DELTA_PORTCOUNTERS(PortRcvFECN);
			GET_DELTA_PORTCOUNTERS(PortRcvBECN);
			GET_DELTA_PORTCOUNTERS(PortMarkFECN);
		}
		GET_DELTA_PORTCOUNTERS(PortXmitData);
		GET_DELTA_PORTCOUNTERS(PortXmitPkts);
		GET_DELTA_PORTCOUNTERS(PortRcvData);
		GET_DELTA_PORTCOUNTERS(PortRcvPkts);
		GET_DELTA_PORTCOUNTERS(PortMulticastXmitPkts);
		GET_DELTA_PORTCOUNTERS(PortMulticastRcvPkts);
		GET_DELTA_PORTCOUNTERS(PortXmitWait);
		GET_DELTA_PORTCOUNTERS(PortXmitTimeCong);
		GET_DELTA_PORTCOUNTERS(PortXmitWastedBW);
		GET_DELTA_PORTCOUNTERS(PortXmitWaitData);
		GET_DELTA_PORTCOUNTERS(PortRcvBubble);
		if (pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) {
			for (i = 0; i < MAX_PM_VLS; i++) {
				GET_DELTA_VLCOUNTERS(PortVLXmitData,     i, PortXmitData);
				GET_DELTA_VLCOUNTERS(PortVLXmitPkts,     i, PortXmitPkts);
				GET_DELTA_VLCOUNTERS(PortVLRcvData,      i, PortRcvData);
				GET_DELTA_VLCOUNTERS(PortVLRcvPkts,      i, PortRcvPkts);
				GET_DELTA_VLCOUNTERS(PortVLXmitWait,     i, PortXmitWait);
				GET_DELTA_VLCOUNTERS(PortVLXmitTimeCong, i, PortXmitTimeCong);
				GET_DELTA_VLCOUNTERS(PortVLXmitWastedBW, i, PortXmitWastedBW);
				GET_DELTA_VLCOUNTERS(PortVLXmitWaitData, i, PortXmitWaitData);
				GET_DELTA_VLCOUNTERS(PortVLRcvBubble,    i, PortRcvBubble);
				{
					GET_DELTA_VLCOUNTERS(SwPortVLCongestion, i, SwPortCongestion);
					GET_DELTA_VLCOUNTERS(PortVLRcvFECN,      i, PortRcvFECN);
					GET_DELTA_VLCOUNTERS(PortVLRcvBECN,      i, PortRcvBECN);
					GET_DELTA_VLCOUNTERS(PortVLMarkFECN,     i, PortMarkFECN);
				}
			}
		}
	} else { /* portImage->u.s.gotDataCntrs == 0 */
		{
			COPY_PORTCOUNTERS(SwPortCongestion);
			COPY_PORTCOUNTERS(PortRcvFECN);
			COPY_PORTCOUNTERS(PortRcvBECN);
			COPY_PORTCOUNTERS(PortMarkFECN);
		}
		COPY_PORTCOUNTERS(PortXmitData);
		COPY_PORTCOUNTERS(PortXmitPkts);
		COPY_PORTCOUNTERS(PortRcvData);
		COPY_PORTCOUNTERS(PortRcvPkts);
		COPY_PORTCOUNTERS(PortMulticastXmitPkts);
		COPY_PORTCOUNTERS(PortMulticastRcvPkts);
		COPY_PORTCOUNTERS(PortXmitWait);
		COPY_PORTCOUNTERS(PortXmitTimeCong);
		COPY_PORTCOUNTERS(PortXmitWastedBW);
		COPY_PORTCOUNTERS(PortXmitWaitData);
		COPY_PORTCOUNTERS(PortRcvBubble);
		if (pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) {
			for (i = 0; i < MAX_PM_VLS; i++) {
				COPY_VLCOUNTERS(PortVLXmitData, i);
				COPY_VLCOUNTERS(PortVLXmitPkts, i);
				COPY_VLCOUNTERS(PortVLRcvData, i);
				COPY_VLCOUNTERS(PortVLRcvPkts, i);
				COPY_VLCOUNTERS(PortVLXmitWait, i);
				COPY_VLCOUNTERS(PortVLXmitTimeCong, i);
				COPY_VLCOUNTERS(PortVLXmitWastedBW, i);
				COPY_VLCOUNTERS(PortVLXmitWaitData, i);
				COPY_VLCOUNTERS(PortVLRcvBubble, i);
				{
					COPY_VLCOUNTERS(SwPortVLCongestion, i);
					COPY_VLCOUNTERS(PortVLRcvFECN, i);
					COPY_VLCOUNTERS(PortVLRcvBECN, i);
					COPY_VLCOUNTERS(PortVLMarkFECN, i);
				}
			}
		}
		COPY_PORTCOUNTERS(lq.s.LinkQualityIndicator);
	}

	if (portImage->u.s.gotErrorCntrs) {
		GET_DELTA_PORTCOUNTERS(PortRcvConstraintErrors);
		GET_DELTA_PORTCOUNTERS(PortXmitDiscards);
		GET_DELTA_PORTCOUNTERS(PortXmitConstraintErrors);
		GET_DELTA_PORTCOUNTERS(PortRcvSwitchRelayErrors);
		GET_DELTA_PORTCOUNTERS(PortRcvRemotePhysicalErrors);
		GET_DELTA_PORTCOUNTERS(LocalLinkIntegrityErrors);
		GET_DELTA_PORTCOUNTERS(PortRcvErrors);
		GET_DELTA_PORTCOUNTERS(ExcessiveBufferOverruns);
		GET_DELTA_PORTCOUNTERS(FMConfigErrors);
		GET_DELTA_PORTCOUNTERS(LinkErrorRecovery);
		GET_DELTA_PORTCOUNTERS(LinkDowned);
		GET_DELTA_PORTCOUNTERS(UncorrectableErrors);
		if (pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) {
			for (i = 0; i < MAX_PM_VLS; i++) {
				GET_DELTA_VLCOUNTERS(PortVLXmitDiscards, i, PortXmitDiscards);
			}
		}
	} else { /* portImage->u.s.gotErrorCntrs == 0 */
		COPY_PORTCOUNTERS(PortRcvConstraintErrors);
		COPY_PORTCOUNTERS(PortXmitDiscards);
		COPY_PORTCOUNTERS(PortXmitConstraintErrors);
		COPY_PORTCOUNTERS(PortRcvSwitchRelayErrors);
		COPY_PORTCOUNTERS(PortRcvRemotePhysicalErrors);
		COPY_PORTCOUNTERS(LocalLinkIntegrityErrors);
		COPY_PORTCOUNTERS(PortRcvErrors);
		COPY_PORTCOUNTERS(ExcessiveBufferOverruns);
		COPY_PORTCOUNTERS(FMConfigErrors);
		COPY_PORTCOUNTERS(LinkErrorRecovery);
		COPY_PORTCOUNTERS(LinkDowned);
		COPY_PORTCOUNTERS(UncorrectableErrors);
		if (pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) {
			for (i = 0; i < MAX_PM_VLS; i++) {
				COPY_VLCOUNTERS(PortVLXmitDiscards, i);
			}
		}
	}
#undef GET_DELTA_VLCOUNTERS
#undef GET_DELTA_PORTCOUNTERS
#undef COPY_VLCOUNTERS
#undef COPY_PORTCOUNTERS


	if (pDeltaPortCounters->LinkDowned) {
		if (unexpectedClear.AsReg32 & ~LinkDownIgnoreMask.AsReg32) {
			CounterSelectMask_t tempMask;
			tempMask.AsReg32 = unexpectedClear.AsReg32 & ~LinkDownIgnoreMask.AsReg32 ;
			portImage->u.s.UnexpectedClear = 1;
			PmUnexpectedClear(pm, pmportp, index, tempMask);
		}
	} else {
		if (unexpectedClear.AsReg32) {
			portImage->u.s.UnexpectedClear = 1;
			PmUnexpectedClear(pm, pmportp, index, unexpectedClear);
		}
	}
	//Copy In the Unexpected Clears to the previous image so the PA can handle them correctly
	portImagePrev->clearSelectMask.AsReg32 |= unexpectedClear.AsReg32;

#define INC_RUNNING(cntr, max) do { \
		if (pRunning->cntr >= (max - pDeltaPortCounters->cntr)) { \
			pRunning->cntr = max; \
		} else { \
			pRunning->cntr += pDeltaPortCounters->cntr; \
		} } while (0)

	// running totals for this port

	INC_RUNNING(PortXmitWait, IB_UINT64_MAX);
	INC_RUNNING(PortXmitData, IB_UINT64_MAX);
	INC_RUNNING(PortRcvData, IB_UINT64_MAX);
	INC_RUNNING(PortXmitPkts, IB_UINT64_MAX);
	INC_RUNNING(PortRcvPkts, IB_UINT64_MAX);
	INC_RUNNING(PortMulticastXmitPkts, IB_UINT64_MAX);
	INC_RUNNING(PortMulticastRcvPkts, IB_UINT64_MAX);
	INC_RUNNING(SwPortCongestion, IB_UINT64_MAX);
	INC_RUNNING(PortRcvFECN, IB_UINT64_MAX);
	INC_RUNNING(PortRcvBECN, IB_UINT64_MAX);
	INC_RUNNING(PortXmitTimeCong, IB_UINT64_MAX);
	INC_RUNNING(PortXmitWastedBW, IB_UINT64_MAX);
	INC_RUNNING(PortXmitWaitData, IB_UINT64_MAX);
	INC_RUNNING(PortRcvBubble, IB_UINT64_MAX);
	INC_RUNNING(PortMarkFECN, IB_UINT64_MAX);
	INC_RUNNING(PortXmitDiscards, IB_UINT64_MAX);
	INC_RUNNING(PortXmitConstraintErrors, IB_UINT64_MAX);
	INC_RUNNING(PortRcvConstraintErrors, IB_UINT64_MAX);
	INC_RUNNING(PortRcvSwitchRelayErrors, IB_UINT64_MAX);
	INC_RUNNING(PortRcvRemotePhysicalErrors, IB_UINT64_MAX);
	INC_RUNNING(LocalLinkIntegrityErrors, IB_UINT64_MAX);
	INC_RUNNING(PortRcvErrors, IB_UINT64_MAX);
	INC_RUNNING(ExcessiveBufferOverruns, IB_UINT64_MAX);
	INC_RUNNING(FMConfigErrors, IB_UINT64_MAX);

	INC_RUNNING(LinkErrorRecovery, IB_UINT32_MAX);
	INC_RUNNING(LinkDowned, IB_UINT32_MAX);
	INC_RUNNING(UncorrectableErrors, IB_UINT8_MAX);

#undef INC_RUNNING
	pRunning->lq.s.LinkQualityIndicator = pImgPortCounters->lq.s.LinkQualityIndicator;
	pRunning->lq.s.NumLanesDown = pImgPortCounters->lq.s.NumLanesDown;

	if (pm->pmFlags & STL_PM_PROCESS_VL_COUNTERS) {
#define INC_VLRUNNING(vlcntr, vl, max) do { \
	if (pVLRunning[vl].vlcntr >= (max - pDeltaVLCounters[vl].vlcntr)) { \
		pVLRunning[vl].vlcntr = max; \
	} else { \
		pVLRunning[vl].vlcntr += pDeltaVLCounters[vl].vlcntr; \
	} } while (0)
		for (i = 0; i < MAX_PM_VLS; i++) {
			INC_VLRUNNING(PortVLXmitData,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvData,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitPkts,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvPkts,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitWait,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(SwPortVLCongestion, i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvFECN,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvBECN,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitTimeCong, i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitWastedBW, i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitWaitData, i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvBubble,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLMarkFECN,	  i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitDiscards, i, IB_UINT64_MAX);
		}
#undef INC_VLRUNNING
	}

} // End of PmFinalizePortStats()


// for a port clear counters which can tabulate information from both
// sides of a link
void PmClearPortImage(PmPortImage_t *portImage)
{
	portImage->u.s.queryStatus = PM_QUERY_STATUS_OK;
	portImage->u.s.UnexpectedClear = 0;
	portImage->u.s.gotDataCntrs = 0;
	portImage->u.s.gotErrorCntrs = 0;
	portImage->u.s.gotErrorInfo = 0;
	portImage->u.s.Reserved = 0;

	portImage->clearSelectMask.AsReg32 = 0;

	memset(&portImage->StlPortCounters, 0, sizeof(PmCompositePortCounters_t));
	memset(&portImage->StlVLPortCounters[0], 0, MAX_PM_VLS * sizeof(PmCompositeVLCounters_t));

	memset(&portImage->DeltaStlPortCounters, 0, sizeof(PmCompositePortCounters_t));
	memset(&portImage->DeltaStlVLPortCounters[0], 0, MAX_PM_VLS * sizeof(PmCompositeVLCounters_t));
	/* Initializing the linkquality of the links avoided due to PmaAvoid flag */
	portImage->StlPortCounters.lq.s.LinkQualityIndicator = STL_LINKQUALITY_EXCELLENT;
}

// Returns TRUE if need to Clear some counters for Port, FALSE if not
// TRUE means *counterSelect indicates counters to clear
// FALSE means no counters need to be cleared and *counterSelect is 0
//
// When possible we will return the same counterSelect for all switch ports.
// Caller can use AllPortSelect if all active ports need to be cleared with the
// same counterSelect.
//
// caller must have imageLock held

boolean PmTabulatePort(Pm_t *pm, PmPort_t *pmportp, uint32 imageIndex, uint32 *counterSelect) {

	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmCompositePortCounters_t *pImgPortCounters = &portImage->StlPortCounters;

	// Thresholds are calulated base upon MAX_UINT## * (ErrorClear/8)
	*counterSelect = 0;
	if (pm->clearCounterSelect.AsReg32) {
		CounterSelectMask_t select;

		select.AsReg32 = 0;
#define IF_EXCEED_CLEARTHRESHOLD(cntr) \
		do { if (IB_EXPECT_FALSE(pImgPortCounters->cntr \
									 > pm->ClearThresholds.cntr)) \
				select.s.cntr = pm->clearCounterSelect.s.cntr; } while (0)

		IF_EXCEED_CLEARTHRESHOLD(PortXmitData);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvData);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitPkts);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvPkts);
		IF_EXCEED_CLEARTHRESHOLD(PortMulticastXmitPkts);
		IF_EXCEED_CLEARTHRESHOLD(PortMulticastRcvPkts);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitWait);
		IF_EXCEED_CLEARTHRESHOLD(SwPortCongestion);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvFECN);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvBECN);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitTimeCong);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitWastedBW);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitWaitData);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvBubble);
		IF_EXCEED_CLEARTHRESHOLD(PortMarkFECN);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvConstraintErrors);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvSwitchRelayErrors);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitDiscards);
		IF_EXCEED_CLEARTHRESHOLD(PortXmitConstraintErrors);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvRemotePhysicalErrors);
		IF_EXCEED_CLEARTHRESHOLD(LocalLinkIntegrityErrors);
		IF_EXCEED_CLEARTHRESHOLD(PortRcvErrors);
		IF_EXCEED_CLEARTHRESHOLD(ExcessiveBufferOverruns);
		IF_EXCEED_CLEARTHRESHOLD(FMConfigErrors);
		IF_EXCEED_CLEARTHRESHOLD(LinkErrorRecovery);
		IF_EXCEED_CLEARTHRESHOLD(LinkDowned);
		IF_EXCEED_CLEARTHRESHOLD(UncorrectableErrors);
#undef IF_EXCEED_CLEARTHRESHOLD

		*counterSelect |= select.AsReg32;
	}
	portImage->clearSelectMask.AsReg32 = *counterSelect;

	return (*counterSelect != 0);
} // End of PmTabulatePort

// build counter select to use when clearing counters
void PM_BuildClearCounterSelect(CounterSelectMask_t *select, boolean clearXfer, boolean clear64bit, boolean clear32bit, boolean clear8bit)
{
	// Set CounterSelect for use during Clear of counters.

	// Data Xfer Counters
	select->s.PortXmitData = clearXfer;
	select->s.PortRcvData = clearXfer;
	select->s.PortXmitPkts = clearXfer;
	select->s.PortRcvPkts = clearXfer;
	select->s.PortMulticastXmitPkts = clearXfer;
	select->s.PortMulticastRcvPkts = clearXfer;

	// Error Counters - 64-bit
	select->s.PortXmitWait = clear64bit;
	select->s.SwPortCongestion = clear64bit;
	select->s.PortRcvFECN = clear64bit;
	select->s.PortRcvBECN = clear64bit;
	select->s.PortXmitTimeCong = clear64bit;
	select->s.PortXmitWastedBW = clear64bit;
	select->s.PortXmitWaitData = clear64bit;
	select->s.PortRcvBubble = clear64bit;
	select->s.PortMarkFECN = clear64bit;
	select->s.PortRcvConstraintErrors = clear64bit;
	select->s.PortRcvSwitchRelayErrors = clear64bit;
	select->s.PortXmitDiscards = clear64bit;
	select->s.PortXmitConstraintErrors = clear64bit;
	select->s.PortRcvRemotePhysicalErrors = clear64bit;
	select->s.LocalLinkIntegrityErrors = clear64bit;
	select->s.PortRcvErrors = clear64bit;
	select->s.ExcessiveBufferOverruns = clear64bit;
	select->s.FMConfigErrors = clear64bit;

	// Error Counters - 32-bit
	select->s.LinkErrorRecovery = clear32bit;
	select->s.LinkDowned = clear32bit;

	// Error Counters - 8-bit
	select->s.UncorrectableErrors = clear8bit;
}

// compute reasonable clearThresholds based on given threshold and weights
// This can be used to initialize clearThreshold and then override just
// a few of the computed defaults in the even user wanted to control just a few
// and default the rest
// used during startup, no lock needed
void PmComputeClearThresholds(PmCompositePortCounters_t *clearThresholds, CounterSelectMask_t *select, uint8 errorClear)
{
	if (errorClear > 7)  errorClear = 7;

	MemoryClear(clearThresholds, sizeof(clearThresholds));	// be safe

#define COMPUTE_THRESHOLD(counter, max) do { \
		clearThresholds->counter = (max/8)*(select->s.counter?errorClear:8); \
		} while (0)

	COMPUTE_THRESHOLD(PortXmitData, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvData, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitPkts, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvPkts, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortMulticastXmitPkts, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortMulticastRcvPkts, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitWait, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(SwPortCongestion, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvFECN, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvBECN, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitTimeCong, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitWastedBW, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitWaitData, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvBubble, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortMarkFECN, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvConstraintErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvSwitchRelayErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitDiscards, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortXmitConstraintErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvRemotePhysicalErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(LocalLinkIntegrityErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(PortRcvErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(ExcessiveBufferOverruns, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(FMConfigErrors, IB_UINT64_MAX);
	COMPUTE_THRESHOLD(LinkErrorRecovery, IB_UINT32_MAX);
	COMPUTE_THRESHOLD(LinkDowned, IB_UINT32_MAX);
	COMPUTE_THRESHOLD(UncorrectableErrors, IB_UINT8_MAX);
#undef COMPUTE_THRESHOLD
}

// Clear Running totals for a given Port.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.  Only counters selected are cleared.
// caller must have totalsLock held for write and imageLock held for read
FSTATUS PmClearPortRunningCounters(PmPort_t *pmportp, CounterSelectMask_t select)
{
	PmCompositePortCounters_t *pRunning;

	pRunning = &pmportp->StlPortCountersTotal;
#define CLEAR_SELECTED(counter) \
		do { if (select.s.counter) pRunning->counter = 0; } while (0)

	CLEAR_SELECTED(PortXmitData);
	CLEAR_SELECTED(PortRcvData);
	CLEAR_SELECTED(PortXmitPkts);
	CLEAR_SELECTED(PortRcvPkts);
	CLEAR_SELECTED(PortMulticastXmitPkts);
	CLEAR_SELECTED(PortMulticastRcvPkts);
	CLEAR_SELECTED(PortXmitWait);
	CLEAR_SELECTED(SwPortCongestion);
	CLEAR_SELECTED(PortRcvFECN);
	CLEAR_SELECTED(PortRcvBECN);
	CLEAR_SELECTED(PortXmitTimeCong);
	CLEAR_SELECTED(PortXmitWastedBW);
	CLEAR_SELECTED(PortXmitWaitData);
	CLEAR_SELECTED(PortRcvBubble);
	CLEAR_SELECTED(PortMarkFECN);
	CLEAR_SELECTED(PortRcvConstraintErrors);
	CLEAR_SELECTED(PortRcvSwitchRelayErrors);
	CLEAR_SELECTED(PortXmitDiscards);
	CLEAR_SELECTED(PortXmitConstraintErrors);
	CLEAR_SELECTED(PortRcvRemotePhysicalErrors);
	CLEAR_SELECTED(LocalLinkIntegrityErrors);
	CLEAR_SELECTED(PortRcvErrors);
	CLEAR_SELECTED(ExcessiveBufferOverruns);
	CLEAR_SELECTED(FMConfigErrors);
	CLEAR_SELECTED(LinkErrorRecovery);
	CLEAR_SELECTED(LinkDowned);
	CLEAR_SELECTED(UncorrectableErrors);
	pRunning->lq.s.LinkQualityIndicator = STL_LINKQUALITY_EXCELLENT;
#undef CLEAR_SELECTED

	return FSUCCESS;
}

// Clear Running totals for a given Node.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write and imageLock held for read
FSTATUS PmClearNodeRunningCounters(PmNode_t *pmnodep, CounterSelectMask_t select)
{
	FSTATUS status = FSUCCESS;
	if (pmnodep->nodeType == STL_NODE_SW) {
		uint8_t i;

		for (i=0; i<=pmnodep->numPorts; ++i) {
			PmPort_t *pmportp = pmnodep->up.swPorts[i];
			if (pmportp)
				status = PmClearPortRunningCounters(pmportp, select);
			if (IB_EXPECT_FALSE(status != FSUCCESS)){
				IB_LOG_WARN_FMT(__func__,"Failed to Clear Counters on Port: %u", i);
				break;
			}
		}
		return status;
	} else {
		return PmClearPortRunningCounters(pmnodep->up.caPortp, select);
	}
}

FSTATUS PmClearPortRunningVFCounters(Pm_t *pm, PmPort_t *pmportp,
	STLVlCounterSelectMask select, int vfIdx, boolean useHiddenVF)
{
	PmCompositeVLCounters_t *pVLRunning;
	PmPortImage_t *portImage = &pmportp->Image[pm->LastSweepIndex];
	int vl = 0, j = 0;
	int i = (useHiddenVF ? -1 : vfIdx);
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;

	pVLRunning = &pmportp->StlVLPortCountersTotal[0];

#define CLEAR_SELECTED(counter) \
	do { if (select.s.counter) pVLRunning[j].counter = 0; } while (0)

	uint32 vlmask = (i == -1) ? 0x8000 : portImage->vfvlmap[i].vlmask;
	for (vl = 0; vl < STL_MAX_VLS && vlmask; vl++, vlmask >>= 1) {
		if ((vlmask & 0x1) == 0) continue;
		j = vl_to_idx(vl);
		CLEAR_SELECTED(PortVLXmitData);
		CLEAR_SELECTED(PortVLRcvData);
		CLEAR_SELECTED(PortVLXmitPkts);
		CLEAR_SELECTED(PortVLRcvPkts);
		CLEAR_SELECTED(PortVLXmitDiscards);
		CLEAR_SELECTED(SwPortVLCongestion);
		CLEAR_SELECTED(PortVLXmitWait);
		CLEAR_SELECTED(PortVLRcvFECN);
		CLEAR_SELECTED(PortVLRcvBECN);
		CLEAR_SELECTED(PortVLXmitTimeCong);
		CLEAR_SELECTED(PortVLXmitWastedBW);
		CLEAR_SELECTED(PortVLXmitWaitData);
		CLEAR_SELECTED(PortVLRcvBubble);
		CLEAR_SELECTED(PortVLMarkFECN);
		status = FSUCCESS;
	}
#undef CLEAR_SELECTED
	return status;
}

// Clear Running totals for a given Node.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write and imageLock held for read
FSTATUS PmClearNodeRunningVFCounters(Pm_t *pm, PmNode_t *pmnodep,
	STLVlCounterSelectMask select, int vfIdx, boolean useHiddenVF)
{
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
	if (pmnodep->nodeType == STL_NODE_SW) {
		uint8_t i;

		for (i=0; i<=pmnodep->numPorts; ++i) {
			PmPort_t *pmportp = pmnodep->up.swPorts[i];
			if (pmportp)
				status |= PmClearPortRunningVFCounters(pm, pmportp, select, vfIdx, useHiddenVF);
		}
		return status;
	} else {
		return PmClearPortRunningVFCounters(pm, pmnodep->up.caPortp, select, vfIdx, useHiddenVF);
	}
}
