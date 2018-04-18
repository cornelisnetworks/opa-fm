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

#include "pm_topology.h"
#include "pa_sth_old.h"

size_t PaSthOldCompositePortCountersToCurrent(PmCompositePortCounters_t *curr, uint8_t *buf)
{
	size_t len = 0;
	PmCompositePortCounters_old_t *old = (PmCompositePortCounters_old_t *)buf;

	len = sizeof(PmCompositePortCounters_old_t);
	memcpy(curr, old, len);

	return len;
}
size_t PaSthOldCompositePortToCurrent(PmCompositePort_t *curr, uint8_t *buf)
{
	int i;
	size_t len = 0;
	PmCompositePort_old_t *old = (PmCompositePort_old_t *)buf;

	curr->guid = old->guid;
	curr->u.AsReg32 = old->u.AsReg32;
	curr->neighborLid = old->neighborLid;
	curr->portNum = old->portNum;
	curr->neighborPort = old->neighborPort;
	len = 18;

	curr->InternalBitMask = old->InternalBitMask; /* Moved to fit larger numVFs */
	curr->numGroups = old->numGroups;
	memcpy(curr->groups, old->groups, PM_MAX_GROUPS_PER_PORT);
	len += 6;

	curr->numVFs = old->numVFs; /* Size change 8->32 bits */
	curr->vlSelectMask = old->vlSelectMask;
	len += 8;

	curr->clearSelectMask = old->clearSelectMask;
	/* reserved99 */
	len += 8;

	for (i = 0; i < 32; i++) {
		curr->compVfVlmap[i].vlmask = old->compVfVlmap[i].vlmask;
		curr->compVfVlmap[i].VF = old->compVfVlmap[i].VF;
		/* Expand unassigned: 0xFF to 0xFFFFFFFF */
		if (curr->compVfVlmap[i].VF == 0xFF) curr->compVfVlmap[i].VF = -1;
	}
	/* Array expanded: 32->1000 */
	for (; i < MAX_VFABRICS; i++) {
		/* set other VFs to unassigned */
		curr->compVfVlmap[i].VF = -1;
	}
	len += (32 * sizeof(PmCompositeVfvlmap_old_t));

	len += PaSthOldCompositePortCountersToCurrent(&curr->stlPortCounters, (uint8_t *)&old->stlPortCounters);

	memcpy(curr->stlVLPortCounters, old->stlVLPortCounters, sizeof(PmCompositeVLCounters_t) * 9);
	len += (sizeof(PmCompositeVLCounters_t) * 9);

	len += PaSthOldCompositePortCountersToCurrent(&curr->DeltaStlPortCounters, (uint8_t *)&old->DeltaStlPortCounters);

	memcpy(curr->DeltaStlVLPortCounters, old->DeltaStlVLPortCounters, sizeof(PmCompositeVLCounters_t) * 9);
	len += (sizeof(PmCompositeVLCounters_t) * 9);

	DEBUG_ASSERT(len == sizeof(PmCompositePort_old_t));

	return len;
}

size_t PaSthOldCompositeNodeToCurrent(PmCompositeNode_t *curr, uint8_t *buf)
{
	size_t len = 0;
	PmCompositeNode_old_t *old = (PmCompositeNode_old_t *)buf;

	curr->NodeGUID = old->guid;
	/* curr->SystemImageGUID = 0; TBD SysGuid Not used in PA */
	memcpy(curr->nodeDesc, old->nodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE);

	curr->lid = old->lid; /* Size change */
	curr->nodeType = old->nodeType;
	curr->numPorts = old->numPorts;


	len = sizeof(PmCompositeNode_old_t) - sizeof(PmCompositePort_old_t **);

	return len;
}

size_t PaSthOldCompositeImageToCurrent(PmCompositeImage_t *curr, uint8_t *buf)
{
	int i;
	size_t len = 0;
	PmCompositeImage_old_t *old = (PmCompositeImage_old_t *)buf;

	/* curr->header = old->header; Not Need, it is done earlier */

	curr->sweepStart = old->sweepStart;
	curr->sweepDuration = old->sweepDuration;
	/* uint8 reserved[2] */
	curr->HFIPorts = old->HFIPorts;
	len += 16;

	curr->switchNodes = old->switchNodes;
	/* uint16 reserved2 */
	curr->switchPorts = old->switchPorts;
	curr->numLinks = old->numLinks;
	curr->numSMs = old->numSMs;
	len += 16;

	curr->noRespNodes = old->noRespNodes;
	curr->noRespPorts = old->noRespPorts;
	curr->skippedNodes = old->skippedNodes;
	curr->skippedPorts = old->skippedPorts;
	len += 16;

	curr->unexpectedClearPorts = old->unexpectedClearPorts;
	curr->downgradedPorts = old->downgradedPorts;
	curr->numGroups = old->numGroups;
	curr->numVFs = old->numVFs;
	len += 16;

	curr->numVFsActive = old->numVFsActive;
	curr->maxLid = old->maxLid;
	curr->numPorts = old->numPorts;
	len += 12;

	for (i = 0; i < PM_HISTORY_MAX_SMS_PER_COMPOSITE; i++) {
		curr->SMs[i].smLid = old->SMs[i].smLid;
		curr->SMs[i].priority = old->SMs[i].priority;
		curr->SMs[i].state = old->SMs[i].state;
		len += sizeof(struct PmCompositeSmInfo_old);
	}

	/* uint32 reserved 3 */
	len += 4;

	curr->allPortsGroup = old->allPortsGroup;
	len += sizeof(PmCompositeGroup_t);

	memcpy(curr->groups, old->groups, sizeof(PmCompositeGroup_t) * PM_MAX_GROUPS);
	len += sizeof(PmCompositeGroup_t) * PM_MAX_GROUPS;

	for (i = 0; i < 32; i++) {
		curr->VFs[i] = old->VFs[i];
		len += sizeof(PmCompositeVF_t);
	}

	DEBUG_ASSERT(len == (sizeof(PmCompositeImage_old_t) - (sizeof(PmFileHeader_t) + sizeof(PmCompositeNode_old_t **)) ) );

	return len;
}
