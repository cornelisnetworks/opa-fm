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

#ifndef _STL_PA_PRIV_H_
#define _STL_PA_PRIV_H_ (1) /* suppress duplicate loading of this file */

#include "iba/stl_pa_types.h"
#include "iba/stl_mad_types.h"

#ifdef __cplusplus
extern "C" {
#endif

static __inline void
BSWAP_STL_PA_IMAGE_ID(STL_PA_IMAGE_ID_DATA *pRecord)
{
#if CPU_LE
	pRecord->imageNumber						= ntoh64(pRecord->imageNumber);
	pRecord->imageOffset						= ntoh32(pRecord->imageOffset);
	pRecord->imageTime.absoluteTime				= ntoh32(pRecord->imageTime.absoluteTime);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_PM_GROUP_INFO(STL_PA_PM_GROUP_INFO_DATA *pRecord, boolean isRequest)
{
#if CPU_LE
	int i;
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);

	if (isRequest) return;

	pRecord->numInternalPorts				= ntoh32(pRecord->numInternalPorts);
	pRecord->numExternalPorts				= ntoh32(pRecord->numExternalPorts);

	pRecord->internalUtilStats.totalMBps	= ntoh64(pRecord->internalUtilStats.totalMBps);
	pRecord->internalUtilStats.totalKPps	= ntoh64(pRecord->internalUtilStats.totalKPps);
	pRecord->internalUtilStats.avgMBps		= ntoh32(pRecord->internalUtilStats.avgMBps);
	pRecord->internalUtilStats.minMBps		= ntoh32(pRecord->internalUtilStats.minMBps);
	pRecord->internalUtilStats.maxMBps		= ntoh32(pRecord->internalUtilStats.maxMBps);
	pRecord->internalUtilStats.numBWBuckets		= ntoh32(pRecord->internalUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->internalUtilStats.BWBuckets[i] = ntoh32(pRecord->internalUtilStats.BWBuckets[i]);
	pRecord->internalUtilStats.avgKPps		= ntoh32(pRecord->internalUtilStats.avgKPps);
	pRecord->internalUtilStats.minKPps		= ntoh32(pRecord->internalUtilStats.minKPps);
	pRecord->internalUtilStats.maxKPps		= ntoh32(pRecord->internalUtilStats.maxKPps);
	pRecord->internalUtilStats.pmaNoRespPorts = ntoh16(pRecord->internalUtilStats.pmaNoRespPorts);
	pRecord->internalUtilStats.topoIncompPorts = ntoh16(pRecord->internalUtilStats.topoIncompPorts);

	pRecord->sendUtilStats.totalMBps		= ntoh64(pRecord->sendUtilStats.totalMBps);
	pRecord->sendUtilStats.totalKPps		= ntoh64(pRecord->sendUtilStats.totalKPps);
	pRecord->sendUtilStats.avgMBps			= ntoh32(pRecord->sendUtilStats.avgMBps);
	pRecord->sendUtilStats.minMBps			= ntoh32(pRecord->sendUtilStats.minMBps);
	pRecord->sendUtilStats.maxMBps			= ntoh32(pRecord->sendUtilStats.maxMBps);
	pRecord->sendUtilStats.numBWBuckets		= ntoh32(pRecord->sendUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->sendUtilStats.BWBuckets[i] = ntoh32(pRecord->sendUtilStats.BWBuckets[i]);
	pRecord->sendUtilStats.avgKPps			= ntoh32(pRecord->sendUtilStats.avgKPps);
	pRecord->sendUtilStats.minKPps			= ntoh32(pRecord->sendUtilStats.minKPps);
	pRecord->sendUtilStats.maxKPps			= ntoh32(pRecord->sendUtilStats.maxKPps);
	pRecord->sendUtilStats.pmaNoRespPorts   = ntoh16(pRecord->sendUtilStats.pmaNoRespPorts);
	pRecord->sendUtilStats.topoIncompPorts  = ntoh16(pRecord->sendUtilStats.topoIncompPorts);

	pRecord->recvUtilStats.totalMBps		= ntoh64(pRecord->recvUtilStats.totalMBps);
	pRecord->recvUtilStats.totalKPps		= ntoh64(pRecord->recvUtilStats.totalKPps);
	pRecord->recvUtilStats.avgMBps			= ntoh32(pRecord->recvUtilStats.avgMBps);
	pRecord->recvUtilStats.minMBps			= ntoh32(pRecord->recvUtilStats.minMBps);
	pRecord->recvUtilStats.maxMBps			= ntoh32(pRecord->recvUtilStats.maxMBps);
	pRecord->recvUtilStats.numBWBuckets		= ntoh32(pRecord->recvUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->recvUtilStats.BWBuckets[i] = ntoh32(pRecord->recvUtilStats.BWBuckets[i]);
	pRecord->recvUtilStats.avgKPps			= ntoh32(pRecord->recvUtilStats.avgKPps);
	pRecord->recvUtilStats.minKPps			= ntoh32(pRecord->recvUtilStats.minKPps);
	pRecord->recvUtilStats.maxKPps			= ntoh32(pRecord->recvUtilStats.maxKPps);
	pRecord->recvUtilStats.pmaNoRespPorts   = ntoh16(pRecord->recvUtilStats.pmaNoRespPorts);
	pRecord->recvUtilStats.topoIncompPorts  = ntoh16(pRecord->recvUtilStats.topoIncompPorts);

	pRecord->internalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->internalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.congestion);
	pRecord->internalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->internalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->internalCategoryStats.categoryMaximums.bubble				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.bubble);
	pRecord->internalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.securityErrors);
	pRecord->internalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.routingErrors);

        pRecord->internalCategoryStats.categoryMaximums.utilizationPct10 = ntoh16(pRecord->internalCategoryStats.categoryMaximums.utilizationPct10);
        pRecord->internalCategoryStats.categoryMaximums.discardsPct10    = ntoh16(pRecord->internalCategoryStats.categoryMaximums.discardsPct10);

	for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
		pRecord->internalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].integrityErrors);
		pRecord->internalCategoryStats.ports[i].congestion			= ntoh32(pRecord->internalCategoryStats.ports[i].congestion);
		pRecord->internalCategoryStats.ports[i].smaCongestion		= ntoh32(pRecord->internalCategoryStats.ports[i].smaCongestion);
		pRecord->internalCategoryStats.ports[i].bubble				= ntoh32(pRecord->internalCategoryStats.ports[i].bubble);
		pRecord->internalCategoryStats.ports[i].securityErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].securityErrors);
		pRecord->internalCategoryStats.ports[i].routingErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].routingErrors);
	}

	pRecord->externalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->externalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.congestion);
	pRecord->externalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->externalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->externalCategoryStats.categoryMaximums.bubble				= ntoh32(pRecord->externalCategoryStats.categoryMaximums.bubble);
	pRecord->externalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->externalCategoryStats.categoryMaximums.securityErrors);
	pRecord->externalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->externalCategoryStats.categoryMaximums.routingErrors);

        pRecord->externalCategoryStats.categoryMaximums.utilizationPct10 = ntoh16(pRecord->externalCategoryStats.categoryMaximums.utilizationPct10);
        pRecord->externalCategoryStats.categoryMaximums.discardsPct10    = ntoh16(pRecord->externalCategoryStats.categoryMaximums.discardsPct10);

        for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
		pRecord->externalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].integrityErrors);
		pRecord->externalCategoryStats.ports[i].congestion			= ntoh32(pRecord->externalCategoryStats.ports[i].congestion);
		pRecord->externalCategoryStats.ports[i].smaCongestion		= ntoh32(pRecord->externalCategoryStats.ports[i].smaCongestion);
		pRecord->externalCategoryStats.ports[i].bubble				= ntoh32(pRecord->externalCategoryStats.ports[i].bubble);
		pRecord->externalCategoryStats.ports[i].securityErrors				= ntoh32(pRecord->externalCategoryStats.ports[i].securityErrors);
			pRecord->externalCategoryStats.ports[i].routingErrors			= ntoh32(pRecord->externalCategoryStats.ports[i].routingErrors);
	}
	pRecord->maxInternalMBps				= ntoh32(pRecord->maxInternalMBps);
	pRecord->maxExternalMBps				= ntoh32(pRecord->maxExternalMBps);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_LIST(STL_PA_GROUP_LIST *pRecord)
{
#if CPU_LE
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_CONFIG_REQ(STL_PA_PM_GROUP_CFG_REQ *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_CONFIG_RSP(STL_PA_PM_GROUP_CFG_RSP *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->nodeGUID							= ntoh64(pRecord->nodeGUID);
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_NODE_INFO_REQ(STL_PA_GROUP_NODEINFO_REQ *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->nodeGUID							= ntoh64(pRecord->nodeGUID);
	pRecord->nodeLID							= ntoh32(pRecord->nodeLID);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_NODE_INFO_RSP(STL_PA_GROUP_NODEINFO_RSP *pRecord)
{
#if CPU_LE
	int i;

	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->nodeGUID							= ntoh64(pRecord->nodeGUID);
	pRecord->nodeLID							= ntoh32(pRecord->nodeLID);
	for (i = 0; i < 4; i++)
		pRecord->portSelectMask[i]					= ntoh64(pRecord->portSelectMask[i]);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_LINK_INFO_REQ(STL_PA_GROUP_LINKINFO_REQ *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->lid								= ntoh32(pRecord->lid);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_GROUP_LINK_INFO_RSP(STL_PA_GROUP_LINKINFO_RSP *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->fromLID							= ntoh32(pRecord->fromLID);
	pRecord->toLID								= ntoh32(pRecord->toLID);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_PORT_COUNTERS(STL_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->flags								= ntoh32(pRecord->flags);
	pRecord->portXmitData						= ntoh64(pRecord->portXmitData);
	pRecord->portRcvData						= ntoh64(pRecord->portRcvData);
	pRecord->portXmitPkts						= ntoh64(pRecord->portXmitPkts);
	pRecord->portRcvPkts						= ntoh64(pRecord->portRcvPkts);
	pRecord->portMulticastXmitPkts				= ntoh64(pRecord->portMulticastXmitPkts);
	pRecord->portMulticastRcvPkts				= ntoh64(pRecord->portMulticastRcvPkts);
	pRecord->localLinkIntegrityErrors			= ntoh64(pRecord->localLinkIntegrityErrors);
	pRecord->fmConfigErrors						= ntoh64(pRecord->fmConfigErrors);
	pRecord->portRcvErrors						= ntoh64(pRecord->portRcvErrors);
	pRecord->excessiveBufferOverruns			= ntoh64(pRecord->excessiveBufferOverruns);
	pRecord->portRcvConstraintErrors			= ntoh64(pRecord->portRcvConstraintErrors);
	pRecord->portRcvSwitchRelayErrors			= ntoh64(pRecord->portRcvSwitchRelayErrors);
	pRecord->portXmitDiscards					= ntoh64(pRecord->portXmitDiscards);
	pRecord->portXmitConstraintErrors			= ntoh64(pRecord->portXmitConstraintErrors);
	pRecord->portRcvRemotePhysicalErrors		= ntoh64(pRecord->portRcvRemotePhysicalErrors);
	pRecord->swPortCongestion					= ntoh64(pRecord->swPortCongestion);
	pRecord->portXmitWait						= ntoh64(pRecord->portXmitWait);
	pRecord->portRcvFECN						= ntoh64(pRecord->portRcvFECN);
	pRecord->portRcvBECN						= ntoh64(pRecord->portRcvBECN);
	pRecord->portXmitTimeCong					= ntoh64(pRecord->portXmitTimeCong);
	pRecord->portXmitWastedBW					= ntoh64(pRecord->portXmitWastedBW);
	pRecord->portXmitWaitData					= ntoh64(pRecord->portXmitWaitData);
	pRecord->portRcvBubble						= ntoh64(pRecord->portRcvBubble);
	pRecord->portMarkFECN						= ntoh64(pRecord->portMarkFECN);
	pRecord->linkErrorRecovery					= ntoh32(pRecord->linkErrorRecovery);
	pRecord->linkDowned							= ntoh32(pRecord->linkDowned);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_CLR_PORT_COUNTERS(STL_CLR_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->NodeLid							= ntoh32(pRecord->NodeLid);
	pRecord->CounterSelectMask.AsReg32			= ntoh32(pRecord->CounterSelectMask.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(STL_CLR_ALL_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->CounterSelectMask.AsReg32			= ntoh32(pRecord->CounterSelectMask.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_PM_CFG(STL_PA_PM_CFG_DATA *pRecord)
{
#if CPU_LE
	pRecord->sweepInterval						= ntoh32(pRecord->sweepInterval);
	pRecord->maxClients							= ntoh32(pRecord->maxClients);
	pRecord->sizeHistory						= ntoh32(pRecord->sizeHistory);
	pRecord->sizeFreeze							= ntoh32(pRecord->sizeFreeze);
	pRecord->lease								= ntoh32(pRecord->lease);
	pRecord->pmFlags							= ntoh32(pRecord->pmFlags);
	pRecord->categoryThresholds.integrityErrors	= ntoh32(pRecord->categoryThresholds.integrityErrors);
	pRecord->categoryThresholds.congestion	= ntoh32(pRecord->categoryThresholds.congestion);
	pRecord->categoryThresholds.smaCongestion	= ntoh32(pRecord->categoryThresholds.smaCongestion);
	pRecord->categoryThresholds.bubble		= ntoh32(pRecord->categoryThresholds.bubble);
	pRecord->categoryThresholds.securityErrors		= ntoh32(pRecord->categoryThresholds.securityErrors);
	pRecord->categoryThresholds.routingErrors		= ntoh32(pRecord->categoryThresholds.routingErrors);
	pRecord->memoryFootprint					= ntoh64(pRecord->memoryFootprint);
	pRecord->maxAttempts						= ntoh32(pRecord->maxAttempts);
	pRecord->respTimeout						= ntoh32(pRecord->respTimeout);
	pRecord->minRespTimeout						= ntoh32(pRecord->minRespTimeout);
	pRecord->maxParallelNodes					= ntoh32(pRecord->maxParallelNodes);
	pRecord->pmaBatchSize						= ntoh32(pRecord->pmaBatchSize);
	// ErrorClear is uint8
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_FOCUS_PORTS_REQ(STL_FOCUS_PORTS_REQ *pRecord)
{
#if CPU_LE
	pRecord->select						= ntoh32(pRecord->select);
	pRecord->start						= ntoh32(pRecord->start);
	pRecord->range						= ntoh32(pRecord->range);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif
}


static __inline void
BSWAP_STL_PA_FOCUS_PORTS_MULTISELECT_REQ(STL_FOCUS_PORTS_MULTISELECT_REQ *pRecord)
{
#if CPU_LE
	int i;
	pRecord->start						= ntoh32(pRecord->start);
	pRecord->range						= ntoh32(pRecord->range);
	for (i = 0; i < MAX_NUM_FOCUS_PORT_TUPLES; i++) {
		pRecord->tuple[i].select			= ntoh32(pRecord->tuple[i].select);
		pRecord->tuple[i].argument			= ntoh64(pRecord->tuple[i].argument);
	}
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif
}


static __inline void
BSWAP_STL_PA_FOCUS_PORTS_RSP(STL_FOCUS_PORTS_RSP *pRecord)
{
#if CPU_LE
	pRecord->nodeLid					= ntoh32(pRecord->nodeLid);
	pRecord->value						= ntoh64(pRecord->value);
	pRecord->neighborLid				= ntoh32(pRecord->neighborLid);
	pRecord->neighborValue				= ntoh64(pRecord->neighborValue);
	pRecord->nodeGUID					= ntoh64(pRecord->nodeGUID);
	pRecord->neighborGuid				= ntoh64(pRecord->neighborGuid);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}


static __inline void
BSWAP_STL_PA_FOCUS_PORTS_MULTISELECT_RSP(STL_FOCUS_PORTS_MULTISELECT_RSP *pRecord)
{
#if CPU_LE
	int i;
	pRecord->nodeLid					= ntoh32(pRecord->nodeLid);
	pRecord->neighborLid					= ntoh32(pRecord->neighborLid);
	pRecord->nodeGUID					= ntoh64(pRecord->nodeGUID);
	pRecord->neighborGuid					= ntoh64(pRecord->neighborGuid);
	for (i = 0; i < MAX_NUM_FOCUS_PORT_TUPLES; i++) {
		pRecord->value[i]				= ntoh64(pRecord->value[i]);
		pRecord->neighborValue[i]			= ntoh64(pRecord->neighborValue[i]);
	}

	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}


static __inline void
BSWAP_STL_PA_IMAGE_INFO(STL_PA_IMAGE_INFO_DATA *pRecord)
{
#if CPU_LE
	int i;
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
	pRecord->sweepStart              = ntoh64(pRecord->sweepStart);
	pRecord->sweepDuration           = ntoh32(pRecord->sweepDuration);
	pRecord->numHFIPorts             = ntoh16(pRecord->numHFIPorts);
	pRecord->numSwitchNodes          = ntoh16(pRecord->numSwitchNodes);
	pRecord->numSwitchPorts          = ntoh32(pRecord->numSwitchPorts);
	pRecord->numLinks                = ntoh32(pRecord->numLinks);
	pRecord->numSMs                  = ntoh32(pRecord->numSMs);
	pRecord->numNoRespNodes          = ntoh32(pRecord->numNoRespNodes);
	pRecord->numNoRespPorts          = ntoh32(pRecord->numNoRespPorts);
	pRecord->numSkippedNodes         = ntoh32(pRecord->numSkippedNodes);
	pRecord->numSkippedPorts         = ntoh32(pRecord->numSkippedPorts);
	pRecord->numUnexpectedClearPorts = ntoh32(pRecord->numUnexpectedClearPorts);
	pRecord->imageInterval           = ntoh32(pRecord->imageInterval);
	for (i = 0; i < 2; i++) {
		pRecord->SMInfo[i].lid        = ntoh32(pRecord->SMInfo[i].lid);
		pRecord->SMInfo[i].smPortGuid = ntoh64(pRecord->SMInfo[i].smPortGuid);
	}
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_MOVE_FREEZE(STL_MOVE_FREEZE_DATA *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->oldFreezeImage);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->newFreezeImage);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_LIST(STL_PA_VF_LIST *pRecord)
{
#if CPU_LE
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_INFO(STL_PA_VF_INFO_DATA *pRecord, boolean isRequest)
{
#if CPU_LE
	int i;
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);

	if (isRequest) return;

	pRecord->numPorts						= ntoh32(pRecord->numPorts);

	pRecord->internalUtilStats.totalMBps	= ntoh64(pRecord->internalUtilStats.totalMBps);
	pRecord->internalUtilStats.totalKPps	= ntoh64(pRecord->internalUtilStats.totalKPps);
	pRecord->internalUtilStats.avgMBps		= ntoh32(pRecord->internalUtilStats.avgMBps);
	pRecord->internalUtilStats.minMBps		= ntoh32(pRecord->internalUtilStats.minMBps);
	pRecord->internalUtilStats.maxMBps		= ntoh32(pRecord->internalUtilStats.maxMBps);
	pRecord->internalUtilStats.numBWBuckets		= ntoh32(pRecord->internalUtilStats.numBWBuckets);
	for (i = 0; i < STL_PM_UTIL_BUCKETS; i++)
		pRecord->internalUtilStats.BWBuckets[i] = ntoh32(pRecord->internalUtilStats.BWBuckets[i]);
	pRecord->internalUtilStats.avgKPps		= ntoh32(pRecord->internalUtilStats.avgKPps);
	pRecord->internalUtilStats.minKPps		= ntoh32(pRecord->internalUtilStats.minKPps);
	pRecord->internalUtilStats.maxKPps		= ntoh32(pRecord->internalUtilStats.maxKPps);
	pRecord->internalUtilStats.pmaNoRespPorts = ntoh16(pRecord->internalUtilStats.pmaNoRespPorts);
	pRecord->internalUtilStats.topoIncompPorts = ntoh16(pRecord->internalUtilStats.topoIncompPorts);

	pRecord->internalCategoryStats.categoryMaximums.integrityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.integrityErrors);
	pRecord->internalCategoryStats.categoryMaximums.congestion			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.congestion);
	pRecord->internalCategoryStats.categoryMaximums.smaCongestion		= ntoh32(pRecord->internalCategoryStats.categoryMaximums.smaCongestion);
	pRecord->internalCategoryStats.categoryMaximums.bubble				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.bubble);
	pRecord->internalCategoryStats.categoryMaximums.securityErrors			= ntoh32(pRecord->internalCategoryStats.categoryMaximums.securityErrors);
	pRecord->internalCategoryStats.categoryMaximums.routingErrors				= ntoh32(pRecord->internalCategoryStats.categoryMaximums.routingErrors);

        pRecord->internalCategoryStats.categoryMaximums.utilizationPct10 = ntoh16(pRecord->internalCategoryStats.categoryMaximums.utilizationPct10);
        pRecord->internalCategoryStats.categoryMaximums.discardsPct10    = ntoh16(pRecord->internalCategoryStats.categoryMaximums.discardsPct10);

	for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
		pRecord->internalCategoryStats.ports[i].integrityErrors			= ntoh32(pRecord->internalCategoryStats.ports[i].integrityErrors);
		pRecord->internalCategoryStats.ports[i].congestion			= ntoh32(pRecord->internalCategoryStats.ports[i].congestion);
		pRecord->internalCategoryStats.ports[i].smaCongestion		= ntoh32(pRecord->internalCategoryStats.ports[i].smaCongestion);
		pRecord->internalCategoryStats.ports[i].bubble				= ntoh32(pRecord->internalCategoryStats.ports[i].bubble);
		pRecord->internalCategoryStats.ports[i].securityErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].securityErrors);
		pRecord->internalCategoryStats.ports[i].routingErrors				= ntoh32(pRecord->internalCategoryStats.ports[i].routingErrors);
	}

	pRecord->maxInternalMBps				= ntoh32(pRecord->maxInternalMBps);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_CFG_REQ(STL_PA_VF_CFG_REQ *pRecord)
{
#if CPU_LE
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_CFG_RSP(STL_PA_VF_CFG_RSP *pRecord)
{
#if CPU_LE
	pRecord->nodeGUID						= ntoh64(pRecord->nodeGUID);
	pRecord->nodeLid						= ntoh32(pRecord->nodeLid);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_PORT_COUNTERS(STL_PA_VF_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->flags								= ntoh32(pRecord->flags);
	pRecord->portVFXmitData						= ntoh64(pRecord->portVFXmitData);
	pRecord->portVFRcvData						= ntoh64(pRecord->portVFRcvData);
	pRecord->portVFXmitPkts						= ntoh64(pRecord->portVFXmitPkts);
	pRecord->portVFRcvPkts						= ntoh64(pRecord->portVFRcvPkts);
	pRecord->portVFXmitDiscards					= ntoh16(pRecord->portVFXmitDiscards);
	pRecord->swPortVFCongestion					= ntoh64(pRecord->swPortVFCongestion);
	pRecord->portVFXmitWait						= ntoh64(pRecord->portVFXmitWait);
	pRecord->portVFRcvFECN						= ntoh64(pRecord->portVFRcvFECN);
	pRecord->portVFRcvBECN						= ntoh64(pRecord->portVFRcvBECN);
	pRecord->portVFXmitTimeCong					= ntoh64(pRecord->portVFXmitTimeCong);
	pRecord->portVFXmitWastedBW					= ntoh64(pRecord->portVFXmitWastedBW);
	pRecord->portVFXmitWaitData					= ntoh64(pRecord->portVFXmitWaitData);
	pRecord->portVFRcvBubble					= ntoh64(pRecord->portVFRcvBubble);
	pRecord->portVFMarkFECN						= ntoh64(pRecord->portVFMarkFECN);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_CLEAR_VF_PORT_COUNTERS(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *pRecord)
{
#if CPU_LE
	pRecord->nodeLid							= ntoh32(pRecord->nodeLid);
	pRecord->vfCounterSelectMask.AsReg32		= ntoh32(pRecord->vfCounterSelectMask.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_PA_VF_FOCUS_PORTS_REQ(STL_PA_VF_FOCUS_PORTS_REQ *pRecord)
{
#if CPU_LE
	pRecord->select						= ntoh32(pRecord->select);
	pRecord->start						= ntoh32(pRecord->start);
	pRecord->range						= ntoh32(pRecord->range);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif
}

static __inline void
BSWAP_STL_PA_VF_FOCUS_PORTS_RSP(STL_PA_VF_FOCUS_PORTS_RSP *pRecord)
{
#if CPU_LE
	pRecord->nodeLid					= ntoh32(pRecord->nodeLid);
	pRecord->value						= ntoh64(pRecord->value);
	pRecord->neighborLid				= ntoh32(pRecord->neighborLid);
	pRecord->neighborValue				= ntoh64(pRecord->neighborValue);
	pRecord->nodeGUID					= ntoh64(pRecord->nodeGUID);
	pRecord->neighborGuid				= ntoh64(pRecord->neighborGuid);
	BSWAP_STL_PA_IMAGE_ID(&pRecord->imageId);
#endif /* CPU_LE */
}

/* Performance Analysis Response Structures */
typedef struct _STL_PA_GROUP_LIST_RESULTS  {
    uint32 NumGroupListRecords;                /* Number of PA Records returned */
    STL_PA_GROUP_LIST GroupListRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_LIST_RESULTS, *PSTL_PA_GROUP_LIST_RESULTS;

typedef struct _STL_PA_GROUP_INFO_RESULTS  {
    uint32 NumGroupInfoRecords;                /* Number of PA Records returned */
    STL_PA_PM_GROUP_INFO_DATA GroupInfoRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_INFO_RESULTS, *PSTL_PA_GROUP_INFO_RESULTS;

typedef struct _STL_PA_GROUP_CONFIG_RESULTS  {
    uint32 NumGroupConfigRecords;                /* Number of PA Records returned */
    STL_PA_PM_GROUP_CFG_RSP GroupConfigRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_CONFIG_RESULTS, *PSTL_PA_GROUP_CONFIG_RESULTS;

typedef struct _STL_PA_GROUP_NODEINFO_RESULTS  {
    uint32 NumGroupNodeInfoRecords;                /* Number of PA Records returned */
    STL_PA_GROUP_NODEINFO_RSP GroupNodeInfoRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_NODEINFO_RESULTS, *PSTL_PA_GROUP_NODEINFO_RESULTS;

typedef struct _STL_PA_GROUP_LINKINFO_RESULTS  {
    uint32 NumGroupLinkInfoRecords;                /* Number of PA Records returned */
    STL_PA_GROUP_LINKINFO_RSP GroupLinkInfoRecords[1];		/* list of PA records returned */
} STL_PA_GROUP_LINKINFO_RESULTS, *PSTL_PA_GROUP_LINKINFO_RESULTS;

typedef struct _STL_PA_PORT_COUNTERS_RESULTS  {
    uint32 NumPortCountersRecords;                /* Number of PA Records returned */
    STL_PORT_COUNTERS_DATA PortCountersRecords[1];		/* list of PA records returned */
} STL_PA_PORT_COUNTERS_RESULTS, *PSTL_PA_PORT_COUNTERS_RESULTS;

typedef struct _STL_PA_CLR_PORT_COUNTERS_RESULTS  {
    uint32 NumClrPortCountersRecords;                /* Number of PA Records returned */
    STL_CLR_PORT_COUNTERS_DATA ClrPortCountersRecords[1];		/* list of PA records returned */
} STL_PA_CLR_PORT_COUNTERS_RESULTS, *PSTL_PA_CLR_PORT_COUNTERS_RESULTS;

typedef struct _STL_PA_CLR_ALL_PORT_COUNTERS_RESULTS  {
    uint32 NumClrAllPortCountersRecords;                /* Number of PA Records returned */
    STL_CLR_ALL_PORT_COUNTERS_DATA ClrAllPortCountersRecords[1];		/* list of PA records returned */
} STL_PA_CLR_ALL_PORT_COUNTERS_RESULTS, *PSTL_PA_CLR_ALL_PORT_COUNTERS_RESULTS;

typedef struct _STL_PA_PM_CONFIG_RESULTS  {
    uint32 NumPmConfigRecords;                /* Number of PA Records returned */
    STL_PA_PM_CFG_DATA PmConfigRecords[1];		/* list of PA records returned */
} STL_PA_PM_CONFIG_RESULTS, *PSTL_PA_PM_CONFIG_RESULTS;

typedef struct _STL_PA_IMAGE_ID_RESULTS  {
    uint32 NumImageIDRecords;                /* Number of PA Records returned */
    STL_PA_IMAGE_ID_DATA ImageIDRecords[1];		/* list of PA records returned */
} STL_PA_IMAGE_ID_RESULTS, *PSTL_PA_IMAGE_ID_RESULTS;

typedef struct _STL_PA_FOCUS_PORTS_RESULTS  {
    uint32 NumFocusPortsRecords;                /* Number of PA Records returned */
    STL_FOCUS_PORTS_RSP FocusPortsRecords[1];		/* list of PA records returned */
} STL_PA_FOCUS_PORTS_RESULTS, *PSTL_PA_FOCUS_PORTS_RESULTS;

typedef struct _STL_PA_FOCUS_PORTS_MULTISELECT_RESULTS  {
    uint32 NumFocusPortsRecords;                /* Number of PA Records returned */
    STL_FOCUS_PORTS_MULTISELECT_RSP FocusPortsRecords[1];		/* list of PA records returned */
} STL_PA_FOCUS_PORTS_MULTISELECT_RESULTS, *PSTL_PA_FOCUS_PORTS_RESULTS_MULTISELECT;

typedef struct _STL_PA_IMAGE_INFO_RESULTS  {
    uint32 NumImageInfoRecords;                /* Number of PA Records returned */
    STL_PA_IMAGE_INFO_DATA ImageInfoRecords[1];		/* list of PA records returned */
} STL_PA_IMAGE_INFO_RESULTS, *PSTL_PA_IMAGE_INFO_RESULTS;

typedef struct _STL_MOVE_FREEZE_RESULTS  {
    uint32 NumMoveFreezeRecords;                /* Number of PA Records returned */
    STL_MOVE_FREEZE_DATA MoveFreezeRecords[1];		/* list of PA records returned */
} STL_MOVE_FREEZE_RESULTS, *PSTL_PA_MOVE_FREEZE_RESULTS;

typedef struct _STL_PA_VF_LIST_RESULTS  {
    uint32 NumVFListRecords;                /* Number of PA Records returned */
    STL_PA_VF_LIST VFListRecords[1];		/* list of PA records returned */
} STL_PA_VF_LIST_RESULTS, *PSTL_PA_VF_LIST_RESULTS;

typedef struct _STL_PA_VF_INFO_RESULTS  {
    uint32 NumVFInfoRecords;                /* Number of PA Records returned */
    STL_PA_VF_INFO_DATA VFInfoRecords[1];		/* list of PA records returned */
} STL_PA_VF_INFO_RESULTS, *PSTL_PA_VF_INFO_RESULTS;

typedef struct _STL_PA_VF_CONFIG_RESULTS  {
    uint32 NumVFConfigRecords;                /* Number of PA Records returned */
    STL_PA_VF_CFG_RSP VFConfigRecords[1];		/* list of PA records returned */
} STL_PA_VF_CONFIG_RESULTS, *PSTL_PA_VF_CONFIG_RESULTS;

typedef struct _STL_PA_VF_FOCUS_PORTS_RESULTS  {
    uint32 NumVFFocusPortsRecords;                /* Number of PA Records returned */
    STL_PA_VF_FOCUS_PORTS_RSP FocusPortsRecords[1];		/* list of PA records returned */
} STL_PA_VF_FOCUS_PORTS_RESULTS, *PSTL_PA_VF_FOCUS_PORTS_RESULTS;

typedef struct _STL_PA_GROUP_LIST2_RESULTS  {
    uint32 NumGroupList2Records;                /* Number of PA Records returned */
    STL_PA_GROUP_LIST2 GroupList2Records[1];		/* list of PA records returned */
} STL_PA_GROUP_LIST2_RESULTS, *PSTL_PA_GROUP_LIST2_RESULTS;

typedef struct _STL_PA_VF_LIST2_RESULTS  {
    uint32 NumVFList2Records;                /* Number of PA Records returned */
    STL_PA_VF_LIST2 VFList2Records[1];		/* list of PA records returned */
} STL_PA_VF_LIST2_RESULTS, *PSTL_PA_VF_LIST2_RESULTS;

/* PortXmitData and PortRcvData are in units of flits (8 bytes) */
#define FLITS_PER_MiB ((uint64)1024*(uint64)1024/(uint64)8)
#define FLITS_PER_MB ((uint64)1000*(uint64)1000/(uint64)8)

#ifdef __cplusplus
}
#endif

#endif /* _STL_PA_PRIV_H_ */
