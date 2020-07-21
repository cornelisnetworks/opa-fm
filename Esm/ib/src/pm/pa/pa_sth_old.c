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

#include "pm_topology.h"
#include "pa_sth_old.h"

/**
 * Handle changes to PmCompositeImage_t for v10 to current (v11)
 *
 * Changes:
 * 	VF Array Length 32->1000
 */
size_t PaSthOldCompositePortToCurrent(PmCompositePort_t *curr, uint8_t *buf)
{
	size_t len = 0, bytes;
	PmCompositePort_oldv10_t *old = (PmCompositePort_oldv10_t *)buf;
	int i;

	/* All Bytes up to VF array */
	bytes = offsetof(PmCompositePort_oldv10_t, compVfVlmap);
	memcpy((uint8_t *)curr, (uint8_t *)old, bytes);
	len += bytes;

	for (i = 0; i < MAX_VFABRICS_OLD; i++) {
		curr->compVfVlmap[i].vlmask = old->compVfVlmap[i].vlmask;
		curr->compVfVlmap[i].VF = old->compVfVlmap[i].VF;
		len += sizeof(PmCompositeVfvlmap_t);
	}
	for (; i < MAX_VFABRICS; i++) {
		/* set other VFs to unassigned */
		curr->compVfVlmap[i].VF = -1;
	}

	/* All bytes from stlPortCounters to the End */
	bytes = sizeof(PmCompositePort_oldv10_t) - offsetof(PmCompositePort_oldv10_t, stlPortCounters);
	memcpy((uint8_t *)curr, (uint8_t *)old, bytes);
	len += bytes;

	DEBUG_ASSERT(len == sizeof(PmCompositePort_oldv10_t));
	return len;
}

/**
 * Handle changes to PmCompositeNode_t for v10 to current (v11)
 *
 * Changes:
 */
size_t PaSthOldCompositeNodeToCurrent(PmCompositeNode_t *curr, uint8_t *buf)
{
	size_t len = sizeof(PmCompositeNode_oldv10_t);
	memcpy(curr, buf, len);

	return len;
}

/**
 * Handle changes to PmCompositeImage_t for v10 to current (v11)
 *
 * Changes:
 *  Removed 'All' PmPortGroup
 *  Shrank PmPortGroup and PmVF structs, removed unused fields
 *  Expanded VF Array Length 32->1000
 */
size_t PaSthOldCompositeImageToCurrent(PmCompositeImage_t *curr, uint8_t *buf)
{
	size_t len = 0, bytes;
	PmCompositeImage_oldv10_t *old = (PmCompositeImage_oldv10_t *)buf;
	int i;

	/* curr->header = old->header; Not Need, it is done earlier */

	/* Copy everything after header until 'All' PortGroup */
	bytes = offsetof(PmCompositeImage_oldv10_t, allPortsGroup) - sizeof(PmFileHeader_t);
	memcpy((uint8_t *)curr + sizeof(PmFileHeader_t), (uint8_t *)old + sizeof(PmFileHeader_t), bytes);

	/* Removed unused 'All' PortGroup from Image */
	len += sizeof(PmCompositeGroup_oldv10_t);
	/* Handle PortGroup struct change */
	for (i = 0; i < PM_MAX_GROUPS; i++) {
		StringCopy(curr->groups[i].name, old->groups[i].name, STL_PM_GROUPNAMELEN);
		len += sizeof(PmCompositeGroup_oldv10_t);
	}
	/* Handle VF array length and struct size change */
	for (i = 0; i < MAX_VFABRICS_OLD; i++) {
		StringCopy(curr->VFs[i].name, old->VFs[i].name, STL_PM_VFNAMELEN);
		curr->VFs[i].isActive = old->VFs[i].isActive;
		len += sizeof(PmCompositeVF_oldv10_t);
	}

	DEBUG_ASSERT(len == (sizeof(PmCompositeImage_oldv10_t) - sizeof(PmFileHeader_t)));
	return len;
}
