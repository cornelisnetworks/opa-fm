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

#include "sm_l.h"
#include "pm_l.h"
#include <iba/ibt.h>
#include "pm_topology.h"
#include <limits.h>

static void DisplayPmUtil(char *prefix, PmUtilStats_t *utilp)
{
	IB_LOG_INFO_FMT(__func__, "%sUtil: Max %6d Min %6d Avg %6d MB/s", prefix,
			utilp->MaxMBps,
			utilp->MinMBps,
			utilp->AvgMBps);
	IB_LOG_INFO_FMT(__func__, "%sUtil: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d", prefix,
			utilp->BwPorts[0],
			utilp->BwPorts[1],
			utilp->BwPorts[2],
			utilp->BwPorts[3],
			utilp->BwPorts[4],
			utilp->BwPorts[5],
			utilp->BwPorts[6],
			utilp->BwPorts[7],
			utilp->BwPorts[8],
			utilp->BwPorts[9]);
	IB_LOG_INFO_FMT(__func__, "%sPkts: Max %6d Min %6d Avg %6d KP/s", prefix,
			utilp->MaxKPps,
			utilp->MinKPps,
			utilp->AvgKPps);
}

static void DisplayPmErr(char *prefix, PmErrStats_t *errp)
{
	IB_LOG_INFO_FMT(__func__, "%sErr Integrity     Max %6d Buckets: %4d %4d %4d %4d %4d", prefix,
			errp->Max.Integrity,
			errp->Ports[0].Integrity,
			errp->Ports[1].Integrity,
			errp->Ports[2].Integrity,
			errp->Ports[3].Integrity,
			errp->Ports[4].Integrity);
	IB_LOG_INFO_FMT(__func__, "%sErr Congestion    Max %6d Buckets: %4d %4d %4d %4d %4d", prefix,
			errp->Max.Congestion,
			errp->Ports[0].Congestion,
			errp->Ports[1].Congestion,
			errp->Ports[2].Congestion,
			errp->Ports[3].Congestion,
			errp->Ports[4].Congestion);
	IB_LOG_INFO_FMT(__func__, "%sErr SmaCongestion Max %6d Buckets: %4d %4d %4d %4d %4d", prefix,
			errp->Max.SmaCongestion,
			errp->Ports[0].SmaCongestion,
			errp->Ports[1].SmaCongestion,
			errp->Ports[2].SmaCongestion,
			errp->Ports[3].SmaCongestion,
			errp->Ports[4].SmaCongestion);
	IB_LOG_INFO_FMT(__func__, "%sErr Bubble        Max %6d Buckets: %4d %4d %4d %4d %4d", prefix,
			errp->Max.Bubble,
			errp->Ports[0].Bubble,
			errp->Ports[1].Bubble,
			errp->Ports[2].Bubble,
			errp->Ports[3].Bubble,
			errp->Ports[4].Bubble);
	IB_LOG_INFO_FMT(__func__, "%sErr Security      Max %6d Buckets: %4d %4d %4d %4d %4d", prefix,
			errp->Max.Security,
			errp->Ports[0].Security,
			errp->Ports[1].Security,
			errp->Ports[2].Security,
			errp->Ports[3].Security,
			errp->Ports[4].Security);
	IB_LOG_INFO_FMT(__func__, "%sErr Routing       Max %6d Buckets: %4d %4d %4d %4d %4d", prefix,
			errp->Max.Routing,
			errp->Ports[0].Routing,
			errp->Ports[1].Routing,
			errp->Ports[2].Routing,
			errp->Ports[3].Routing,
			errp->Ports[4].Routing);
}

static void DisplayPmGroup(PmGroup_t *groupp, uint32 imageIndex)
{
	PmGroupImage_t *groupImage = &groupp->Image[imageIndex];

	IB_LOG_INFO0("------------------------------------------------------------------------------");
	IB_LOG_INFO_FMT(__func__, "%s (Ports: %u Int, %u Ext):", groupp->Name, groupImage->NumIntPorts, groupImage->NumExtPorts);
	if (groupImage->NumIntPorts && ! groupImage->NumExtPorts) {
		IB_LOG_INFO_FMT(__func__, "%sRate Min %u Max %u", "",
				groupImage->MinIntRate, groupImage->MaxIntRate);
		DisplayPmUtil("", &groupImage->IntUtil);
		DisplayPmErr("", &groupImage->IntErr);
	} else {
		if (groupImage->NumIntPorts) {
			IB_LOG_INFO_FMT(__func__, "Int Rate Min %u Max %u",
					groupImage->MinIntRate, groupImage->MaxIntRate);
			DisplayPmUtil("Int", &groupImage->IntUtil);
			DisplayPmErr("Int", &groupImage->IntErr);
		}
		if (groupImage->NumExtPorts) {
			IB_LOG_INFO_FMT(__func__, "Ext Rate Min %u Max %u",
					groupImage->MinExtRate, groupImage->MaxIntRate);
			DisplayPmUtil("Send", &groupImage->SendUtil);
			DisplayPmUtil("Recv", &groupImage->RecvUtil);
			DisplayPmErr("Ext", &groupImage->ExtErr);
		}
	}
}

// caller will have verified enough sweeps done to have data
// caller must hold imageLock
void DisplayPm(Pm_t *pm, uint32 imageIndex)
{
	int i;
	uint64_t size = 0;

	vs_pool_size(&pm_pool, &size);
	IB_LOG_INFO_FMT(__func__, "PM Sweep: %u Memory Used: %"PRIu64" MB (%"PRIu64" KB)",
				   				pm->NumSweeps, size / (1024*1024), size/1024);
	DisplayPmGroup(pm->AllPorts, imageIndex);
	for (i=0; i< pm->NumGroups; i++) {
		DisplayPmGroup(pm->Groups[i], imageIndex);
	}
}
