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

// low level computational functions
// except where noted, caller of these routines must hold imageLock for write
// for image being computed

#include "pm_topology.h"
#include <limits.h>
#include <iba/ib_helper.h>

extern Pm_t g_pmSweepData;

static void ClearUtilStats(PmUtilStats_t *utilp)
{
	utilp->TotMBps = 0;
	utilp->TotKPps = 0;

	utilp->MaxMBps = 0;
	utilp->MinMBps = IB_UINT32_MAX;

	utilp->MaxKPps = 0;
	utilp->MinKPps = IB_UINT32_MAX;
	MemoryClear(&utilp->BwPorts[0], sizeof(utilp->BwPorts[0]) * PM_UTIL_BUCKETS);
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
	groupImage->MaxIntRate = IB_STATIC_RATE_MIN;
	groupImage->MinExtRate = IB_STATIC_RATE_MAX;
	groupImage->MaxExtRate = IB_STATIC_RATE_MIN;
}

// clear stats for a VF
void ClearVFStats(PmVFImage_t *vfImage)
{
	ClearUtilStats(&vfImage->IntUtil);
	ClearErrStats(&vfImage->IntErr);
	vfImage->MinIntRate = IB_STATIC_RATE_MAX;
	vfImage->MaxIntRate = IB_STATIC_RATE_MIN;
}

// The Update routines take a portSweep argument to reduce overhead
// by letting caller index the PortSweepData once and pass pointer
static void UpdateUtilStats(PmUtilStats_t *utilp, PmPortImage_t *portImage)
{
	utilp->TotMBps += portImage->SendMBps;
	utilp->TotKPps += portImage->SendKPps;

	utilp->BwPorts[portImage->UtilBucket]++;

	UPDATE_MAX(utilp->MaxMBps, portImage->SendMBps);
	UPDATE_MIN(utilp->MinMBps, portImage->SendMBps);

	UPDATE_MAX(utilp->MaxKPps, portImage->SendKPps);
	UPDATE_MIN(utilp->MinKPps, portImage->SendKPps);
}

// called when both ports in a link are in the given group
static void UpdateErrStatsInGroup(PmErrStats_t *errp, PmPortImage_t *portImage)
{
#define UPDATE_MAX_ERROR(stat) do { \
		UPDATE_MAX(errp->Max.stat, portImage->Errors.stat); \
	} while (0)
#define UPDATE_ERROR(stat) do { \
		UPDATE_MAX(errp->Max.stat, portImage->Errors.stat); \
		errp->Ports[portImage->stat##Bucket].stat++; \
	} while (0)
	UPDATE_ERROR(Integrity);
	UPDATE_ERROR(Congestion);
	UPDATE_ERROR(SmaCongestion);
	UPDATE_ERROR(Bubble);
	UPDATE_ERROR(Security);
	UPDATE_ERROR(Routing);
	UPDATE_MAX_ERROR(CongInefficiencyPct10);
	UPDATE_MAX_ERROR(WaitInefficiencyPct10);
	UPDATE_MAX_ERROR(BubbleInefficiencyPct10);
	UPDATE_MAX_ERROR(DiscardsPct10);
	UPDATE_MAX_ERROR(CongestionDiscardsPct10);
	UPDATE_MAX_ERROR(UtilizationPct10);
#undef UPDATE_ERROR
#undef UPDATE_MAX_ERROR
}

// called when only one of ports in a link are in the given group
static void UpdateErrStatsExtGroup(PmErrStats_t *errp,
			   	PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	// for between-group counters we use the worst of the 2 ports
#define UPDATE_MAX_ERROR(stat) do { \
		UPDATE_MAX(errp->Max.stat, portImage->Errors.stat); \
		UPDATE_MAX(errp->Max.stat, portImage2->Errors.stat); \
	} while (0)
#define UPDATE_ERROR(stat) do { \
		UPDATE_MAX(errp->Max.stat, portImage->Errors.stat); \
		UPDATE_MAX(errp->Max.stat, portImage2->Errors.stat); \
		errp->Ports[MAX(portImage->stat##Bucket, portImage2->stat##Bucket)].stat++; \
	} while (0)
	UPDATE_ERROR(Integrity);
	UPDATE_ERROR(Congestion);
	UPDATE_ERROR(SmaCongestion);
	UPDATE_ERROR(Bubble);
	UPDATE_ERROR(Security);
	UPDATE_ERROR(Routing);
	UPDATE_MAX_ERROR(CongInefficiencyPct10);
	UPDATE_MAX_ERROR(WaitInefficiencyPct10);
	UPDATE_MAX_ERROR(BubbleInefficiencyPct10);
	UPDATE_MAX_ERROR(DiscardsPct10);
	UPDATE_MAX_ERROR(CongestionDiscardsPct10);
	UPDATE_MAX_ERROR(UtilizationPct10);
#undef UPDATE_ERROR
#undef UPDATE_MAX_ERROR
}

// update stats for a port whose neighbor is also in the group
void UpdateInGroupStats(Pm_t *pm, PmGroupImage_t *groupImage,
				PmPortImage_t *portImage)
{
	UpdateUtilStats(&groupImage->IntUtil, portImage);
	UpdateErrStatsInGroup(&groupImage->IntErr, portImage);
	if (s_StaticRateToMBps[portImage->u.s.rate] > s_StaticRateToMBps[groupImage->MaxIntRate])
		groupImage->MaxIntRate = portImage->u.s.rate;
	if (s_StaticRateToMBps[portImage->u.s.rate] < s_StaticRateToMBps[groupImage->MinIntRate])
		groupImage->MinIntRate = portImage->u.s.rate;
}

// update stats for a port whose neighbor is not also in the group
void UpdateExtGroupStats(Pm_t *pm, PmGroupImage_t *groupImage,
			   	PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	ASSERT(portImage2); // only ports with neighbors could be Ext
	// our stats are what portp Sent
	UpdateUtilStats(&groupImage->SendUtil, portImage);
	// neighbor's stats are what it sent (and portp received)
	UpdateUtilStats(&groupImage->RecvUtil, portImage2);
	UpdateErrStatsExtGroup(&groupImage->ExtErr, portImage, portImage2);
	if (s_StaticRateToMBps[portImage->u.s.rate] > s_StaticRateToMBps[groupImage->MaxExtRate])
		groupImage->MaxExtRate = portImage->u.s.rate;
	if (s_StaticRateToMBps[portImage->u.s.rate] < s_StaticRateToMBps[groupImage->MinExtRate])
		groupImage->MinExtRate = portImage->u.s.rate;
}

// update stats for a port whose neighbor is also in the group
void UpdateVFStats(Pm_t *pm, PmVFImage_t *vfImage,
				PmPortImage_t *portImage)
{
	UpdateUtilStats(&vfImage->IntUtil, portImage);
	UpdateErrStatsInGroup(&vfImage->IntErr, portImage);
	if (s_StaticRateToMBps[portImage->u.s.rate] > s_StaticRateToMBps[vfImage->MaxIntRate])
		vfImage->MaxIntRate = portImage->u.s.rate;
	if (s_StaticRateToMBps[portImage->u.s.rate] < s_StaticRateToMBps[vfImage->MinIntRate])
		vfImage->MinIntRate = portImage->u.s.rate;
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

// given a MBps transfer rate and a theoretical maxMBps, compute
// the utilization bucket number from 0 to PM_UTIL_BUCKETS-1
static __inline uint8 ComputeUtilBucket(uint32 SendMBps, uint32 maxMBps)
{
	// MaxMBps at 120g (QDR 12x) is 11444, so no risk of uint32 overflow
	if (maxMBps) {
		// directly compute bucket to reduce overflow chances
		uint8 utilBucket = (SendMBps * PM_UTIL_BUCKETS) / maxMBps;
		if (utilBucket >= PM_UTIL_BUCKETS)
			return PM_UTIL_BUCKETS-1;
		else
			return utilBucket;
	} else {
		return 0;
	}
}

// given a error count and a threshold, compute
// the err bucket number from 0 to PM_ERR_BUCKETS-1
static __inline uint8 ComputeErrBucket(uint32 errCnt, uint32 errThreshold)
{
	uint8 errBucket;

	// directly compute bucket to reduce overflow chances
	if (! errThreshold)	// ignore class of errors
		return 0;
	errBucket = (errCnt * (PM_ERR_BUCKETS-1)) / errThreshold;
	if (errBucket >= PM_ERR_BUCKETS)
		 return PM_ERR_BUCKETS-1;
	else
		 return errBucket;
}

static __inline void ComputeBuckets(Pm_t *pm, PmPortImage_t *portImage)
{
#define COMPUTE_ERR_BUCKET(stat) do { \
		portImage->stat##Bucket = ComputeErrBucket( portImage->Errors.stat, \
				   							pm->Thresholds.stat); \
	} while (0)

	if (! portImage->u.s.bucketComputed) {
		portImage->UtilBucket = ComputeUtilBucket(portImage->SendMBps,
				   					s_StaticRateToMBps[portImage->u.s.rate]);
		COMPUTE_ERR_BUCKET(Integrity);
		COMPUTE_ERR_BUCKET(Congestion);
		COMPUTE_ERR_BUCKET(SmaCongestion);
		COMPUTE_ERR_BUCKET(Bubble);
		COMPUTE_ERR_BUCKET(Security);
		COMPUTE_ERR_BUCKET(Routing);
		portImage->u.s.bucketComputed = 1;
	}
#undef COMPUTE_ERR_BUCKET
}

static void PmPrintExceededPort(PmPort_t *pmportp, uint32 index,
				const char *statistic, uint32 threshold, uint32 value)
{
	PmPort_t *pmportp2 = pmportp->Image[index].neighbor;

	if (pmportp2) {
		IB_LOG_WARN_FMT(__func__, "%s of %u Exceeded Threshold of %u. %.*s Guid "FMT_U64" LID 0x%x Port %u  Neighbor: %.*s Guid "FMT_U64" LID 0x%x Port %u",
					statistic, value, threshold,
					sizeof(pmportp->pmnodep->nodeDesc.NodeString),
					pmportp->pmnodep->nodeDesc.NodeString,
					pmportp->pmnodep->guid,
					pmportp->pmnodep->Image[index].lid,
					pmportp->portNum,
					sizeof(pmportp2->pmnodep->nodeDesc.NodeString),
					pmportp2->pmnodep->nodeDesc.NodeString,
					pmportp2->pmnodep->guid,
					pmportp2->pmnodep->Image[index].lid,
					pmportp2->portNum);
	} else {
		IB_LOG_WARN_FMT(__func__, "%s of %u Exceeded Threshold of %u. %.*s Guid "FMT_U64" LID 0x%x Port %u",
					statistic, value, threshold,
					sizeof(pmportp->pmnodep->nodeDesc.NodeString),
					pmportp->pmnodep->nodeDesc.NodeString,
					pmportp->pmnodep->guid,
					pmportp->pmnodep->Image[index].lid,
					pmportp->portNum);
	}
}
static void PmPrintExceededPortDetailsIntegrity(PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	IB_LOG_WARN_FMT(__func__, "LocalLinkIntegrityErrors=%"PRIu64", PortRcvErrors=%"PRIu64", LinkErrorRecovery=%u, LinkDowned=%u, ",
					portImage->StlPortCounters.LocalLinkIntegrityErrors,
					portImage->StlPortCounters.PortRcvErrors,
					portImage->StlPortCounters.LinkErrorRecovery,
					portImage->StlPortCounters.LinkDowned
					);
	IB_LOG_WARN_FMT(__func__, "UncorrectableErrors=%u, FMConfigErrors=%"PRIu64", neighbor ExcessiveBufferOverruns=%"PRIu64" ",
					portImage->StlPortCounters.UncorrectableErrors,
					portImage->StlPortCounters.FMConfigErrors,
					portImage2 ? portImage2->StlPortCounters.ExcessiveBufferOverruns : 0
					);
}
static void PmPrintExceededPortDetailsCongestion(PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	IB_LOG_WARN_FMT(__func__, "XmitWait=%"PRIu64", CongDiscards=%"PRIu64", neighbor RcvFECN=%"PRIu64", ",
					portImage->StlPortCounters.PortXmitWait,
					portImage->StlPortCounters.SwPortCongestion,
					portImage2 ? portImage2->StlPortCounters.PortRcvFECN : 0
				   );
	IB_LOG_WARN_FMT(__func__, "RcvBECN=%"PRIu64", XmitTimeCong=%"PRIu64", MarkFECN=%"PRIu64"",
					portImage->StlPortCounters.PortRcvBECN,
					portImage->StlPortCounters.PortXmitTimeCong,
					portImage->StlPortCounters.PortMarkFECN
					);
}
static void PmPrintExceededPortDetailsSmaCongestion(PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	IB_LOG_WARN_FMT(__func__, "VLXmitWait[15]=%"PRIu64", VLCongDiscards[15]=%"PRIu64", neighbor VLRcvFECN[15]=%"PRIu64", ",
					portImage->StlVLPortCounters[15].PortVLXmitWait,
					portImage->StlVLPortCounters[15].SwPortVLCongestion,
					portImage2 ? portImage2->StlVLPortCounters[15].PortVLRcvFECN : 0
				   );
	IB_LOG_WARN_FMT(__func__, "VLRcvBECN[15]=%"PRIu64", VLXmitTimeCong[15]=%"PRIu64", VLMarkFECN[15]=%"PRIu64" ",
				   portImage->StlVLPortCounters[15].PortVLRcvBECN,
				   portImage->StlVLPortCounters[15].PortVLXmitTimeCong,
				   portImage->StlVLPortCounters[15].PortVLMarkFECN
				   );
}
static void PmPrintExceededPortDetailsBubble(PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	IB_LOG_WARN_FMT(__func__, "XmitWastedBW=%"PRIu64", XmitWaitData=%"PRIu64", neighbor RcvBubble=%"PRIu64" ",
					portImage->StlPortCounters.PortXmitWastedBW,
					portImage->StlPortCounters.PortXmitWaitData,
					portImage2 ? portImage2->StlPortCounters.PortRcvBubble : 0
				   );
}
static void PmPrintExceededPortDetailsSecurity(PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	IB_LOG_WARN_FMT(__func__, "XmitConstraintErrors=%"PRIu64", neighbor RcvConstraintErrors=%"PRIu64" ",
					portImage->StlPortCounters.PortXmitConstraintErrors,
					portImage2 ? portImage2->StlPortCounters.PortRcvConstraintErrors : 0
					);
}
static void PmPrintExceededPortDetailsRouting(PmPortImage_t *portImage, PmPortImage_t *portImage2)
{
	IB_LOG_WARN_FMT(__func__, "RcvSwitchRelayErrors=%"PRIu64" ",
					portImage->StlPortCounters.PortRcvSwitchRelayErrors
					);
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
	int i;
	uint32 imageIndexPrevious = (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID ?
		(index == 0 ? pm_config.total_images-1 : index-1) : pm->LastSweepIndex);

	PmPortImage_t *portImage = &pmportp->Image[index];
	PmPortImage_t *portImagePrev = (pm->Image[imageIndexPrevious].state != PM_IMAGE_VALID ?
		NULL : &pmportp->Image[imageIndexPrevious]);

	PmPortImage_t *portImage2 = NULL;

	PmCompositePortCounters_t *pRunning = &pmportp->StlPortCountersTotal;
	PmCompositePortCounters_t *pImgPortCounters = &portImage->StlPortCounters;
	PmCompositePortCounters_t ImgPortCounters = {0};
	PmCompositeVLCounters_t *pVLRunning = &pmportp->StlVLPortCountersTotal[0];
	PmCompositeVLCounters_t *pImgPortVLCounters = &portImage->StlVLPortCounters[0];
	PmCompositeVLCounters_t ImgPortVLCounters[MAX_PM_VLS] = {{0}};

	PmCompositePortCounters_t *pImgPortCountersPrev =
		(portImagePrev ? &portImagePrev->StlPortCounters : &ImgPortCounters);
	PmCompositeVLCounters_t *pImgPortVLCountersPrev =
		(portImagePrev ? &portImagePrev->StlVLPortCounters[0] : &ImgPortVLCounters[0]);

	CounterSelectMask_t prevPortClrSelMask;
	prevPortClrSelMask.AsReg32 = (portImagePrev ? portImagePrev->clearSelectMask.AsReg32 : 0xFFFFFFFF);

	if (portImage->neighbor) {
		portImage2 = &portImage->neighbor->Image[index];
	}

	ComputeBuckets(pm, portImage);
	if (portImage2)
		ComputeBuckets(pm, portImage2);

#define INC_RUNNING(cntr, max) 																\
	do { 																					\
		if (IB_EXPECT_FALSE(pRunning->cntr >= max - (pImgPortCounters->cntr - 				\
				(prevPortClrSelMask.s.cntr ? 0 : pImgPortCountersPrev->cntr)) ) ) {			\
			if(IB_EXPECT_TRUE(pImgPortCounters->cntr > pImgPortCountersPrev->cntr))	{		\
				pRunning->cntr = max;														\
			} else {																		\
				pRunning->cntr += pImgPortCounters->cntr;									\
			}																				\
		} else {																			\
			pRunning->cntr += (pImgPortCounters->cntr -										\
							(prevPortClrSelMask.s.cntr ? 0 : pImgPortCountersPrev->cntr));	\
		}																					\
	} while (0)

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
	if(portImage->u.s.gotErrorCntrs) {
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
	}
#undef INC_RUNNING
	if (IB_EXPECT_TRUE(pm->NumSweeps > 1)){
		UPDATE_MIN(pRunning->lq.s.LinkQualityIndicator, pImgPortCounters->lq.s.LinkQualityIndicator);
	} else {// Initialize LinkQualityIndicator
		pRunning->lq.s.LinkQualityIndicator = pImgPortCounters->lq.s.LinkQualityIndicator;
	}

	if (pm_config.process_vl_counters) {
#define INC_VLRUNNING(vlcntr, pcntr, vl, max) 													\
	do { if (IB_EXPECT_FALSE(pVLRunning[vl].vlcntr >= max - (pImgPortVLCounters[vl].vlcntr -	\
				(prevPortClrSelMask.s.pcntr ? 0 : pImgPortVLCountersPrev[vl].vlcntr)) ) )		\
		pVLRunning[vl].vlcntr = max; 															\
	else 																						\
		pVLRunning[vl].vlcntr += (pImgPortVLCounters[vl].vlcntr - 								\
					(prevPortClrSelMask.s.pcntr ? 0 : pImgPortVLCountersPrev[vl].vlcntr));		\
	} while (0)
		for (i = 0; i < MAX_PM_VLS; i++) {
			INC_VLRUNNING(PortVLXmitData,		PortXmitData,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvData,		PortRcvData,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitPkts,		PortXmitPkts,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvPkts,		PortRcvPkts,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitWait,		PortXmitWait,		i, IB_UINT64_MAX);
			INC_VLRUNNING(SwPortVLCongestion,	SwPortCongestion,	i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvFECN,		PortRcvFECN,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvBECN,		PortRcvBECN,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitTimeCong,	PortXmitTimeCong,	i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitWastedBW,	PortXmitWastedBW,	i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLXmitWaitData,	PortXmitWaitData,	i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLRcvBubble,		PortRcvBubble,		i, IB_UINT64_MAX);
			INC_VLRUNNING(PortVLMarkFECN,		PortMarkFECN, 		i, IB_UINT64_MAX);
			if(portImage->u.s.gotErrorCntrs) {
				INC_VLRUNNING(PortVLXmitDiscards,	PortXmitDiscards,	i, IB_UINT64_MAX);
			}
		}
#undef INC_VLRUNNING
	}

	// Log Ports which exceeded threshold up to ThresholdExceededMsgLimit
#define LOG_EXCEEDED_THRESHOLD(stat) \
	do { \
		if (portImage->stat##Bucket >= (PM_ERR_BUCKETS-1) \
			&& pm->AllPorts->Image[index].IntErr.Ports[PM_ERR_BUCKETS-1].stat \
					< pm_config.thresholdsExceededMsgLimit.stat) { \
			PmPrintExceededPort(pmportp, index, \
				#stat, pm->Thresholds.stat, portImage->Errors.stat); \
			PmPrintExceededPortDetails##stat(portImage, portImage2); \
		} \
	} while (0)
	LOG_EXCEEDED_THRESHOLD(Integrity);
	LOG_EXCEEDED_THRESHOLD(Congestion);
	LOG_EXCEEDED_THRESHOLD(SmaCongestion);
	LOG_EXCEEDED_THRESHOLD(Bubble);
	LOG_EXCEEDED_THRESHOLD(Security);
	LOG_EXCEEDED_THRESHOLD(Routing);
#undef LOG_EXCEEDED_THRESHOLD

}

// for a port clear counters which can tabulate information from both
// sides of a link
void PmClearPortImage(PmPortImage_t *portImage)
{
	portImage->u.s.bucketComputed = 0;
	portImage->u.s.queryStatus = PM_QUERY_STATUS_OK;
	portImage->u.s.UnexpectedClear = 0;
	portImage->u.s.gotErrorCntrs = 0;
	portImage->u.s.clearSent = 0;
	portImage->SendMBps = 0;
	portImage->SendKPps = 0;

	memset(&portImage->Errors, 0, sizeof(ErrorSummary_t));
	memset(portImage->VFSendMBps, 0, sizeof(uint32) * MAX_VFABRICS);
	memset(portImage->VFSendKPps, 0, sizeof(uint32) * MAX_VFABRICS);
	memset(portImage->VFErrors, 0, sizeof(ErrorSummary_t) * MAX_VFABRICS);
}

static void PmUnexpectedClear(Pm_t *pm, PmPort_t *pmportp, uint32 imageIndex,
	CounterSelectMask_t unexpectedClear)
{
	PmImage_t *pmimagep = &pm->Image[imageIndex];
	PmNode_t *pmnodep = pmportp->pmnodep;
	char *detail = "";
	detail=": Make sure no other tools are clearing fabric counters";

	if (pmimagep->FailedNodes + pmimagep->FailedPorts
				   	+ pmimagep->UnexpectedClearPorts < pm_config.SweepErrorsLogThreshold)
	{
		IB_LOG_WARN_FMT(__func__, "Unexpected counter clear for %.*s Guid "FMT_U64" LID 0x%x Port %u%s (Mask 0x%08x)",
			sizeof(pmnodep->nodeDesc.NodeString),
		   	pmnodep->nodeDesc.NodeString,
		   	pmnodep->guid,
			pmnodep->Image[imageIndex].lid, pmportp->portNum, detail,
			unexpectedClear.AsReg32);
	} else {
		IB_LOG_INFO_FMT(__func__, "Unexpected counter clear for %.*s Guid "FMT_U64" LID 0x%x Port %u%s (Mask 0x%08x)",
			sizeof(pmnodep->nodeDesc.NodeString),
		   	pmnodep->nodeDesc.NodeString,
		   	pmnodep->guid,
			pmnodep->Image[imageIndex].lid, pmportp->portNum, detail,
			unexpectedClear.AsReg32);
	}
	pmimagep->UnexpectedClearPorts++;
	INCREMENT_PM_COUNTER(pmCounterPmUnexpectedClearPorts);
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
//
// Note that when this routine is called, CounterSelect field in pCounters
// and pLastCounters is undefined (some paths in pm_dispatch, such as when an
// AllPortSelect is used or no clear is done, will not have initialized this).

boolean PmTabulatePort(Pm_t *pm, PmPort_t *pmportp, uint32 imageIndex,
					   uint32 *counterSelect) {
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	PmCompositePortCounters_t *pImgPortCounters = &portImage->StlPortCounters;
	PmCompositeVLCounters_t *pImgPortVLCounters = &portImage->StlVLPortCounters[0];
	PmCompositePortCounters_t DeltaPortCounters = {0};
	PmCompositeVLCounters_t DeltaVLCounters[MAX_PM_VLS] = {{0}};
	PmPort_t *pmportp2 = portImage->neighbor;
	// TBD what if prev image neighbor is different?

	// we simply alternate these two scratch areas, current and last
	// kept as private data.  If ever need as private data could move this
	// field to portImage and use portImage
	PmAllPortCounters_t *pCounters = &PM_PORT_COUNTERS(pmportp);
	PmAllPortCounters_t *pLastCounters = &PM_PORT_LAST_COUNTERS(pmportp);

	uint32 imageIndexPrev = (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID ?
		(imageIndex == 0 ? pm_config.total_images-1 : imageIndex-1) : pm->LastSweepIndex);
	CounterSelectMask_t unexpectedClear = {0};
	int i;

	IB_LOG_DEBUG3_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x Port %u",
					  sizeof(pmportp->pmnodep->nodeDesc.NodeString),
					  pmportp->pmnodep->nodeDesc.NodeString,
					  pmportp->pmnodep->guid, pmportp->pmnodep->dlid,
					  pmportp->portNum);

	if (pCounters->flags & PM_PORT_GOT_DATAPORTCOUNTERS) {

#define COPY_PORTCTRS(cntr) 																				\
			do {																							\
				pImgPortCounters->cntr = pCounters->PortStatus->cntr; 										\
				if (IB_EXPECT_FALSE(pCounters->PortStatus->cntr < pLastCounters->PortStatus->cntr || 		\
																pLastCounters->clearSelectMask.s.cntr)) {	\
					unexpectedClear.s.cntr = !pLastCounters->clearSelectMask.s.cntr;						\
					DeltaPortCounters.cntr = pCounters->PortStatus->cntr;									\
				} else {																					\
					DeltaPortCounters.cntr = pCounters->PortStatus->cntr - pLastCounters->PortStatus->cntr;	\
				} 																							\
			} while (0)

#define COPY_VLPORTCTRS(cntr, vl, pcntr) \
			do { 																				\
				pImgPortVLCounters[vl].cntr = pCounters->PortStatus->VLs[vl].cntr; 				\
				if (IB_EXPECT_FALSE(pCounters->PortStatus->VLs[vl].cntr < 						\
									               pLastCounters->PortStatus->VLs[vl].cntr || 	\
									               pLastCounters->clearSelectMask.s.pcntr)) {	\
					unexpectedClear.s.pcntr = !pLastCounters->clearSelectMask.s.pcntr;			\
					DeltaVLCounters[vl].cntr = pCounters->PortStatus->VLs[vl].cntr;				\
				} else {																		\
					DeltaVLCounters[vl].cntr = pCounters->PortStatus->VLs[vl].cntr - 			\
												pLastCounters->PortStatus->VLs[vl].cntr;		\
				}																				\
			} while (0)

		pImgPortCounters->lq.s.LinkQualityIndicator = pCounters->PortStatus->lq.s.LinkQualityIndicator;
		COPY_PORTCTRS(PortXmitData);
		COPY_PORTCTRS(PortXmitPkts);
		COPY_PORTCTRS(PortRcvData);
		COPY_PORTCTRS(PortRcvPkts);
		COPY_PORTCTRS(PortMulticastXmitPkts);
		COPY_PORTCTRS(PortMulticastRcvPkts);
		COPY_PORTCTRS(PortXmitWait);
		COPY_PORTCTRS(SwPortCongestion);
		COPY_PORTCTRS(PortRcvFECN);
		COPY_PORTCTRS(PortRcvBECN);
		COPY_PORTCTRS(PortXmitTimeCong);
		COPY_PORTCTRS(PortXmitWastedBW);
		COPY_PORTCTRS(PortXmitWaitData);
		COPY_PORTCTRS(PortRcvBubble);
		COPY_PORTCTRS(PortMarkFECN);
		if (pm_config.process_vl_counters) {
			for (i = 0; i < MAX_PM_VLS; i++) {
				COPY_VLPORTCTRS(PortVLXmitData,     i, PortXmitData);
				COPY_VLPORTCTRS(PortVLXmitPkts,     i, PortXmitPkts);
				COPY_VLPORTCTRS(PortVLRcvData,      i, PortRcvData);
				COPY_VLPORTCTRS(PortVLRcvPkts,      i, PortRcvPkts);
				COPY_VLPORTCTRS(PortVLXmitWait,     i, PortXmitWait);
				COPY_VLPORTCTRS(SwPortVLCongestion, i, SwPortCongestion);
				COPY_VLPORTCTRS(PortVLRcvFECN,      i, PortRcvFECN);
				COPY_VLPORTCTRS(PortVLRcvBECN,      i, PortRcvBECN);
				COPY_VLPORTCTRS(PortVLXmitTimeCong, i, PortXmitTimeCong);
				COPY_VLPORTCTRS(PortVLXmitWastedBW, i, PortXmitWastedBW);
				COPY_VLPORTCTRS(PortVLXmitWaitData, i, PortXmitWaitData);
				COPY_VLPORTCTRS(PortVLRcvBubble,    i, PortRcvBubble);
				COPY_VLPORTCTRS(PortVLMarkFECN,     i, PortMarkFECN);
			}
		}
		if (pCounters->flags & PM_PORT_GOT_ERRORPORTCOUNTERS) {
			COPY_PORTCTRS(PortRcvConstraintErrors);
			COPY_PORTCTRS(PortXmitDiscards);
			COPY_PORTCTRS(PortXmitConstraintErrors);
			COPY_PORTCTRS(PortRcvSwitchRelayErrors);
			COPY_PORTCTRS(PortRcvRemotePhysicalErrors);
			COPY_PORTCTRS(LocalLinkIntegrityErrors);
			COPY_PORTCTRS(PortRcvErrors);
			COPY_PORTCTRS(ExcessiveBufferOverruns);
			COPY_PORTCTRS(FMConfigErrors);
			COPY_PORTCTRS(LinkErrorRecovery);
			COPY_PORTCTRS(LinkDowned);
			COPY_PORTCTRS(UncorrectableErrors);
			if (pm_config.process_vl_counters) {
				for (i = 0; i < MAX_PM_VLS; i++) {
					COPY_VLPORTCTRS(PortVLXmitDiscards, i, PortXmitDiscards);
				}
			}
			portImage->u.s.gotErrorCntrs = 1;
		} else if (pm->Image[imageIndexPrev].state == PM_IMAGE_VALID) {
#undef COPY_VLPORTCTRS
#undef COPY_PORTCTRS
			// Copy Previous Image Error Counters if Error Counters were not queried this sweep
#define COPY_PORTCTRS(cntr) \
	pImgPortCounters->cntr = pmportp->Image[imageIndexPrev].StlPortCounters.cntr;
#define COPY_VLPORTCTRS(cntr, vl) \
	pImgPortVLCounters[vl].cntr = pmportp->Image[imageIndexPrev].StlVLPortCounters[vl].cntr;
			COPY_PORTCTRS(PortRcvConstraintErrors);
			COPY_PORTCTRS(PortXmitDiscards);
			COPY_PORTCTRS(PortXmitConstraintErrors);
			COPY_PORTCTRS(PortRcvSwitchRelayErrors);
			COPY_PORTCTRS(PortRcvRemotePhysicalErrors);
			COPY_PORTCTRS(LocalLinkIntegrityErrors);
			COPY_PORTCTRS(PortRcvErrors);
			COPY_PORTCTRS(ExcessiveBufferOverruns);
			COPY_PORTCTRS(FMConfigErrors);
			COPY_PORTCTRS(LinkErrorRecovery);
			COPY_PORTCTRS(LinkDowned);
			COPY_PORTCTRS(UncorrectableErrors);
			if (pm_config.process_vl_counters) {
				for (i = 0; i < MAX_PM_VLS; i++) {
					COPY_VLPORTCTRS(PortVLXmitDiscards, i);
				}
			}
		}
#undef COPY_VLPORTCTRS
#undef COPY_PORTCTRS
	}

	// =================== tabulate results for Port =============
	if (unexpectedClear.AsReg32) {
		portImage->u.s.UnexpectedClear = 1;
		PmUnexpectedClear(pm, pmportp, imageIndex, unexpectedClear);
	}

	// Thresholds are calulated base upon MAX_UINT## / (ErrorClear/8)

	*counterSelect = 0;
	if (pm->clearCounterSelect.AsReg32) {
		CounterSelectMask_t select;

		select.AsReg32 = 0;
#define IF_EXCEED_CLEARTHRESHOLD(cntr) \
		do { if (IB_EXPECT_FALSE(pCounters->PortStatus->cntr \
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

		portImage->u.s.clearSent = (select.AsReg32 ? 1 : 0);
		*counterSelect |= select.AsReg32;
	}
	portImage->clearSelectMask.AsReg32 = *counterSelect;

	// Calulate Port Utilization
	UPDATE_MAX(portImage->SendMBps, (DeltaPortCounters.PortXmitData) / FLITS_PER_MB / pm->interval);
	UPDATE_MAX(portImage->SendKPps, (DeltaPortCounters.PortXmitPkts) / 1000 / pm->interval);

	// Calulate Port Errors from local counters
	portImage->Errors.Integrity += (uint32)(
		DeltaPortCounters.LocalLinkIntegrityErrors	* pm->integrityWeights.LocalLinkIntegrityErrors +
		DeltaPortCounters.PortRcvErrors				* pm->integrityWeights.PortRcvErrors +
		DeltaPortCounters.LinkErrorRecovery 		* pm->integrityWeights.LinkErrorRecovery +
		DeltaPortCounters.LinkDowned 				* pm->integrityWeights.LinkDowned +
		DeltaPortCounters.UncorrectableErrors 		* pm->integrityWeights.UncorrectableErrors +
		DeltaPortCounters.FMConfigErrors			* pm->integrityWeights.FMConfigErrors);

	portImage->Errors.Congestion += (uint32)(
		(DeltaPortCounters.PortXmitData ? DeltaPortCounters.PortXmitWait *
			pm->congestionWeights.PortXmitWait * 10 /
			(DeltaPortCounters.PortXmitData + DeltaPortCounters.PortXmitWait) : 0) +
		(DeltaPortCounters.PortXmitData ? DeltaPortCounters.PortXmitTimeCong *
			pm->congestionWeights.PortXmitTimeCong * 10 /
			(DeltaPortCounters.PortXmitData + DeltaPortCounters.PortXmitTimeCong) : 0) +
		(DeltaPortCounters.PortXmitPkts ? DeltaPortCounters.PortRcvBECN *
			pm->congestionWeights.PortRcvBECN * 10 *
			(pmportp->pmnodep->nodeType == STL_NODE_FI ? 1 : 0 ) /
			DeltaPortCounters.PortXmitPkts : 0) +
		(DeltaPortCounters.PortXmitPkts ? DeltaPortCounters.PortMarkFECN *
			pm->congestionWeights.PortMarkFECN * 10 /
			DeltaPortCounters.PortXmitPkts : 0) +
		(DeltaPortCounters.PortXmitDiscards ? DeltaPortCounters.SwPortCongestion *
			pm->congestionWeights.SwPortCongestion * 10 /
			DeltaPortCounters.PortXmitDiscards : 0) );

	// Bubble uses MAX between PortXmitWastedBW + PortXmitWaitData and neighbor's PortRcvBubble
	UPDATE_MAX(portImage->Errors.Bubble, (uint32)(DeltaPortCounters.PortXmitData ?
		(DeltaPortCounters.PortXmitWastedBW + DeltaPortCounters.PortXmitWaitData) * 10 /
			(DeltaPortCounters.PortXmitData + DeltaPortCounters.PortXmitWastedBW +
			DeltaPortCounters.PortXmitWaitData): 0) );

	portImage->Errors.Security += (uint32)(DeltaPortCounters.PortXmitConstraintErrors);

	// There is no neighbor counter associated with routing.
	portImage->Errors.Routing = (uint32)DeltaPortCounters.PortRcvSwitchRelayErrors;

	if (pm->flags & STL_PM_PROCESS_VL_COUNTERS) {
		// SmaCongestion uses congestion weigting on VL15 counters associated with congestion
		portImage->Errors.SmaCongestion += (uint32)(
			(DeltaVLCounters[15].PortVLXmitData ? DeltaVLCounters[15].PortVLXmitWait *
				pm->congestionWeights.PortXmitWait * 10 /
				(DeltaVLCounters[15].PortVLXmitData + DeltaVLCounters[15].PortVLXmitWait) : 0) +
			(DeltaVLCounters[15].PortVLXmitData ? DeltaVLCounters[15].PortVLXmitTimeCong *
				pm->congestionWeights.PortXmitTimeCong * 10 /
				(DeltaVLCounters[15].PortVLXmitData + DeltaVLCounters[15].PortVLXmitTimeCong) : 0) +
			(DeltaVLCounters[15].PortVLXmitPkts ? DeltaVLCounters[15].PortVLRcvBECN	*
				pm->congestionWeights.PortRcvBECN * 10 *
				(pmportp->pmnodep->nodeType == STL_NODE_FI ? 1 : 0 ) /
				DeltaVLCounters[15].PortVLXmitPkts : 0) +
			(DeltaVLCounters[15].PortVLXmitPkts ? DeltaVLCounters[15].PortVLMarkFECN *
				pm->congestionWeights.PortMarkFECN * 10 /
				DeltaVLCounters[15].PortVLXmitPkts : 0) +
			(DeltaVLCounters[15].PortVLXmitDiscards ? DeltaVLCounters[15].SwPortVLCongestion *
				pm->congestionWeights.SwPortCongestion * 10 /
				DeltaVLCounters[15].PortVLXmitDiscards : 0) );

		// Calculate VF/VL Utilization and Errors
		for (i = 0; i < portImage->numVFs; i++) {
			int j;
			for (j = 0; j < MAX_PM_VLS; j++) {
				if (portImage->vfvlmap[i].vl == j) {
					// Utilization
					UPDATE_MAX(portImage->VFSendMBps[i],
							   (uint32)(DeltaVLCounters[j].PortVLXmitData / FLITS_PER_MB / pm->interval));
					UPDATE_MAX(portImage->VFSendKPps[i],
							   (uint32)(DeltaVLCounters[j].PortVLXmitPkts / 1000 / pm->interval));

					// Errors
					//portImage->VFErrors[i].Integrity = 0;   // no contributing error counters

					portImage->VFErrors[i].Congestion += (uint32)(
						(DeltaVLCounters[j].PortVLXmitData ? DeltaVLCounters[j].PortVLXmitWait *
							pm->congestionWeights.PortXmitWait * 10 /
							(DeltaVLCounters[j].PortVLXmitData + DeltaVLCounters[j].PortVLXmitWait) : 0) +
						(DeltaVLCounters[j].PortVLXmitData ? DeltaVLCounters[j].PortVLXmitTimeCong *
							pm->congestionWeights.PortXmitTimeCong * 10 /
							(DeltaVLCounters[j].PortVLXmitData + DeltaVLCounters[j].PortVLXmitTimeCong) : 0) +
						(DeltaVLCounters[j].PortVLXmitPkts ? DeltaVLCounters[j].PortVLRcvBECN *
							pm->congestionWeights.PortRcvBECN * 10 *
							(pmportp->pmnodep->nodeType == STL_NODE_FI ? 1 : 0 ) /
							DeltaVLCounters[j].PortVLXmitPkts : 0) +
						(DeltaVLCounters[j].PortVLXmitPkts ? DeltaVLCounters[j].PortVLMarkFECN *
							pm->congestionWeights.PortMarkFECN * 10 /
							DeltaVLCounters[j].PortVLXmitPkts : 0) +
						(DeltaVLCounters[j].PortVLXmitDiscards ? DeltaVLCounters[j].SwPortVLCongestion *
							pm->congestionWeights.SwPortCongestion * 10 /
							DeltaVLCounters[j].PortVLXmitDiscards : 0) );

					if (j == 15) {
						// Use Port SmaCongestion only for VFs that include VL15
						UPDATE_MAX(portImage->VFErrors[i].SmaCongestion, portImage->Errors.SmaCongestion);
					}

					UPDATE_MAX(portImage->VFErrors[i].Bubble, (uint32)(DeltaVLCounters[j].PortVLXmitData ?
						(DeltaVLCounters[j].PortVLXmitWastedBW + DeltaVLCounters[j].PortVLXmitWaitData) * 10 /
							(DeltaVLCounters[j].PortVLXmitData + DeltaVLCounters[j].PortVLXmitWastedBW +
								DeltaVLCounters[j].PortVLXmitWaitData) : 0) );

					//portImage->VFErrors[i].Security = 0;    // no contributing error counters

					//portImage->VFErrors[i].Routing = 0;     // no contributing error counters
				}
			}
		}
	}

	// Calculate Neighbor's Utilization and Errors using receive side counters
	if (pmportp2) {
		PmPortImage_t *portImage2 = &pmportp2->Image[imageIndex];

		// Calulate Port Utilization
		UPDATE_MAX(portImage2->SendMBps, (uint32)(DeltaPortCounters.PortRcvData / FLITS_PER_MB / pm->interval));
		UPDATE_MAX(portImage2->SendKPps, (uint32)(DeltaPortCounters.PortRcvPkts / 1000 / pm->interval));

		portImage2->Errors.Integrity += (uint32)(
			DeltaPortCounters.ExcessiveBufferOverruns	* pm->integrityWeights.ExcessiveBufferOverruns);

		portImage2->Errors.Congestion += (uint32)(DeltaPortCounters.PortRcvPkts ?
			DeltaPortCounters.PortRcvFECN * pm->congestionWeights.PortRcvFECN * 10 /
				DeltaPortCounters.PortRcvPkts : 0);

		// Bubble uses MAX between PortXmitWastedBW + PortXmitWaitData and neighbor's PortRcvBubble
		UPDATE_MAX(portImage2->Errors.Bubble, (uint32)(DeltaPortCounters.PortRcvData ?
			DeltaPortCounters.PortRcvBubble * 10 /
				(DeltaPortCounters.PortRcvData + DeltaPortCounters.PortRcvBubble): 0) );

		//portImage2->Errors.Routing += 0;		// no contributing error counters

		portImage2->Errors.Security += (uint32)(DeltaPortCounters.PortRcvConstraintErrors);

		if (pm->flags & STL_PM_PROCESS_VL_COUNTERS) {
			// SmaCongestion uses congestion weigting on VL15 counters associated with congestion
			portImage2->Errors.SmaCongestion += (uint32)(DeltaVLCounters[15].PortVLRcvPkts ?
				DeltaVLCounters[15].PortVLRcvFECN * pm->congestionWeights.PortRcvFECN * 10 /
					DeltaVLCounters[15].PortVLRcvPkts : 0);

			// Calculate VF/VL Utilization and Errors
			for (i = 0; i < portImage2->numVFs; i++) {
				int j;
				for (j = 0; j < MAX_PM_VLS; j++) {
					if (portImage2->vfvlmap[i].vl == j) {
						// Utilization
						UPDATE_MAX(portImage2->VFSendMBps[i],
								   (uint32)(DeltaVLCounters[j].PortVLRcvData / FLITS_PER_MB / pm->interval) );
						UPDATE_MAX(portImage2->VFSendKPps[i],
								   (uint32)(DeltaVLCounters[j].PortVLRcvPkts / 1000 / pm->interval) );

						// Errors
						//portImage2->VFErrors[i].Integrity = 0;	// no contributing error counters

						portImage2->VFErrors[i].Congestion += (uint32)(DeltaVLCounters[j].PortVLRcvPkts ?
							DeltaVLCounters[j].PortVLRcvFECN * pm->congestionWeights.PortRcvFECN * 10 /
								DeltaVLCounters[j].PortVLRcvPkts : 0);
						if (j == 15) {
							// Use Port SmaCongestion only for VFs that include VL15
							UPDATE_MAX(portImage2->VFErrors[i].SmaCongestion, portImage2->Errors.SmaCongestion);
						}

						UPDATE_MAX(portImage2->VFErrors[i].Bubble, (uint32)(DeltaVLCounters[j].PortVLRcvData ?
							DeltaVLCounters[j].PortVLRcvBubble * 10 /
								(DeltaVLCounters[j].PortVLRcvData + DeltaVLCounters[j].PortVLRcvBubble): 0) );

						//portImage2->VFErrors[i].Security = 0;		// no contributing error counters

						//portImage2->VFErrors[i].Routing = 0;		// no contributing error counters

					}
				}
			}
		}

#define OVERFLOW_CHECK_MAX_PCT10(pct10, value) UPDATE_MAX(portImage2->Errors.pct10, (value > IB_UINT16_MAX ? IB_UINT16_MAX : (uint16)(value)))
		// Calculation of Pct10 counters for remote port
		uint32 RemoteMaxFlitsPinterval = (uint32)(s_StaticRateToMBps[portImage2->u.s.rate] * FLITS_PER_MB * pm->interval);	// MB/sec * Flits/MB * sec/interval = flits/interval
		uint32 RemoteSentFlitsPinterval = (uint32)DeltaPortCounters.PortRcvData;

		OVERFLOW_CHECK_MAX_PCT10(UtilizationPct10,
			(uint32)(RemoteSentFlitsPinterval * 10 / RemoteMaxFlitsPinterval) );

		OVERFLOW_CHECK_MAX_PCT10(BubbleInefficiencyPct10, (uint32)(portImage->Errors.UtilizationPct10 ?
			portImage->Errors.Bubble / (portImage->Errors.Bubble + 	// BubblePct10 / (BubblePct10 +
				portImage->Errors.UtilizationPct10) : 0) );			//   UtilizationPct10)

#undef OVERFLOW_CHECK_MAX_PCT10
	}	// End of 'if (pmportp2) {'

	// Calculation of Pct10 counters for local port
	uint32 MaxFlitsPinterval = (uint32)(s_StaticRateToMBps[portImage->u.s.rate] * FLITS_PER_MB * pm->interval);	// MB/sec * Flits/MB * sec/interval = flits/interval
	uint32 SentFlitsPinterval = (uint32)DeltaPortCounters.PortXmitData;
	uint32 AttemptedPktsPinterval = (uint32)(DeltaPortCounters.PortXmitPkts + DeltaPortCounters.PortXmitDiscards);

#define OVERFLOW_CHECK_SET_PCT10(pct10, value) portImage->Errors.pct10 = (value > IB_UINT16_MAX ? IB_UINT16_MAX : (uint16)(value))
#define OVERFLOW_CHECK_MAX_PCT10(pct10, value) UPDATE_MAX(portImage->Errors.pct10, (value > IB_UINT16_MAX ? IB_UINT16_MAX : (uint16)(value)))

	OVERFLOW_CHECK_MAX_PCT10(UtilizationPct10, (uint32)(SentFlitsPinterval * 10 / MaxFlitsPinterval) );

    OVERFLOW_CHECK_SET_PCT10(CongInefficiencyPct10,
		(uint32)(portImage->Errors.UtilizationPct10 * SentFlitsPinterval ?
			(DeltaPortCounters.PortXmitTimeCong * 10 /							// PortXmitTimeCongPct10
			(SentFlitsPinterval + DeltaPortCounters.PortXmitTimeCong)) /		//     ... /
				 ((DeltaPortCounters.PortXmitTimeCong * 10 /					// (PortXmitTimeCongPct10
				 (SentFlitsPinterval + DeltaPortCounters.PortXmitTimeCong)) +	//     ... +
					portImage->Errors.UtilizationPct10) : 0) );					//    UtilizationPct10)

    OVERFLOW_CHECK_SET_PCT10(WaitInefficiencyPct10,
		(uint32)(portImage->Errors.UtilizationPct10 * SentFlitsPinterval ?
			(DeltaPortCounters.PortXmitWait * 10 / 							// PortXmitWaitPct10
			(SentFlitsPinterval + DeltaPortCounters.PortXmitWait)) /		//     ... /
				((DeltaPortCounters.PortXmitWait * 10 / 					// (PortXmitWaitPct10
				(SentFlitsPinterval + DeltaPortCounters.PortXmitWait)) +	//     ... +
					portImage->Errors.UtilizationPct10) : 0) );				//    UtilizationPct10)

	OVERFLOW_CHECK_MAX_PCT10(BubbleInefficiencyPct10,
		(uint32)(portImage->Errors.UtilizationPct10 * SentFlitsPinterval ?
			portImage->Errors.Bubble / (portImage->Errors.Bubble +
				portImage->Errors.UtilizationPct10) : 0) );

    OVERFLOW_CHECK_SET_PCT10(DiscardsPct10, (uint32)(AttemptedPktsPinterval ?
		DeltaPortCounters.PortXmitDiscards * 10 / AttemptedPktsPinterval : 0) );

	OVERFLOW_CHECK_SET_PCT10(CongestionDiscardsPct10, (uint32)(DeltaPortCounters.PortXmitDiscards ?
		DeltaPortCounters.SwPortCongestion * 10 / DeltaPortCounters.PortXmitDiscards : 0) );

#undef OVERFLOW_CHECK_SET_PCT10
#undef OVERFLOW_CHECK_MAX_PCT10
	return (*counterSelect != 0);
} // End of PmTabulatePort
 
// build counter select to use when clearing counters
void PM_BuildClearCounterSelect(CounterSelectMask_t *select, boolean clearXfer, boolean clear64bit, boolean clear32bit, boolean clear8bit)
{
	// Set CounterSelect for use during Clear of counters.

	// Data Xfer Counters - Do not check for clear
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

FSTATUS PmClearPortRunningVFCounters(PmPort_t *pmportp, 
					STLVlCounterSelectMask select, char *vfName)
{
	PmCompositeVLCounters_t *pVLRunning;
	PmPortImage_t *portImage = &pmportp->Image[g_pmSweepData.SweepIndex];
	int vl = 0;
	int i;
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
	boolean useHiddenVF = !strcmp(HIDDEN_VL15_VF, vfName);

	pVLRunning = &pmportp->StlVLPortCountersTotal[0];

#define CLEAR_SELECTED(counter) \
	do { if (select.s.counter) pVLRunning[vl].counter = 0; } while (0)

	for (i = 0; i < MAX_VFABRICS; i++) {
		if ((useHiddenVF && !i) || strcmp(portImage->vfvlmap[i].vfName, vfName) == 0) {
			vl = useHiddenVF ? 15 : portImage->vfvlmap[i].vl;
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
		if (useHiddenVF) break;
	}
#undef CLEAR_SELECTED
	return status;
}

// Clear Running totals for a given Node.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals until we
// have a history feature.
// caller must have totalsLock held for write and imageLock held for read
FSTATUS PmClearNodeRunningVFCounters(PmNode_t *pmnodep,
					STLVlCounterSelectMask select, char *vfName)
{
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
	if (pmnodep->nodeType == STL_NODE_SW) {
		uint8_t i;

		for (i=0; i<=pmnodep->numPorts; ++i) {
			PmPort_t *pmportp = pmnodep->up.swPorts[i];
			if (pmportp)
				status |= PmClearPortRunningVFCounters(pmportp, select, vfName);
		}
		return status;
	} else {
		return PmClearPortRunningVFCounters(pmnodep->up.caPortp, select, vfName);
	}
}
