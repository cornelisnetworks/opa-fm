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

#include "pm_topology.h"

// These functions are all oriented toward PortImage.  We could alternatively
// supply a PmPort_t and a imageIndex.  Right now the implementation of
// these routines does not need imageIndex (a couple ASSERTs could benefit
// from it).

// adds a port to a group. used by PmAddExtPort and PmAddIntPort
void PmAddPortToGroupIndex(PmPortImage_t * portImage, uint32 grpIndex,
	PmGroup_t *groupp, boolean internal)
{
	portImage->Groups[grpIndex] = groupp;
	if (internal)
		portImage->InternalBitMask |= (1<<grpIndex);
	portImage->numGroups++;
}

static boolean PmIsPortImageInGroup(PmPortImage_t *portImage, PmGroup_t *groupp,
	boolean *isInternal)
{
    uint32 i;
    boolean isInGroup = FALSE;
    for (i = 0; i < portImage->numGroups; i++) {
        if (portImage->Groups[i] == groupp) {
            isInGroup = TRUE;
            break;
        }
    }
    if (isInternal && isInGroup) {
        *isInternal = (boolean)(portImage->InternalBitMask & (1<<i));
    }
    return isInGroup;
}

boolean PmIsPortInGroup(PmImage_t *pmimagep, PmPortImage_t *portImage,
	int groupIndex, boolean isAllGroup, boolean *isInternal)
{
	if (!portImage->u.s.active) return FALSE;
	if (isAllGroup) {
		if (isInternal) *isInternal = TRUE;
		return TRUE;
	}
	if (groupIndex == -1 || groupIndex > pmimagep->NumGroups ||
		groupIndex >= PM_MAX_GROUPS) return FALSE;

	return PmIsPortImageInGroup(portImage, &pmimagep->Groups[groupIndex], isInternal);
}

// adds a port to a group where the neighbor of the port WILL NOT be in
// the given group
void PmAddExtPortToGroupIndex(PmPortImage_t *portImage, uint32 grpIndex,
	PmGroup_t *groupp, uint32 imageIndex)
{
	PmAddPortToGroupIndex(portImage, grpIndex, groupp, FALSE);
}

// adds a port to a group where the neighbor of the port WILL be in
// the given group
// This DOES NOT add the neighbor.  Caller must do that separately.
void PmAddIntPortToGroupIndex(PmPortImage_t *portImage, uint32 grpIndex,
	PmGroup_t *groupp, uint32 imageIndex)
{
	PmAddPortToGroupIndex(portImage, grpIndex, groupp, TRUE);
}

