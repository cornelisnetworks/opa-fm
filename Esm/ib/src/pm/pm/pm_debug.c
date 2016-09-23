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

// caller will have verified enough sweeps done to have data
// caller must hold imageLock
void DisplayPm(Pm_t *pm)
{
	uint64 size = 0;

	vs_pool_size(&pm_pool, &size);
	if (size > (uint64)0x280000000ull) { // if greater than 10GB show in GB
		IB_LOG_INFO_FMT(__func__,
			"PM Sweep: %u Memory Used: %"PRIu64" GB (%"PRIu64" MB)",
			pm->NumSweeps, (uint64)(size/0x40000000ull), (uint64)(size/0x100000ull));
	} else {
		IB_LOG_INFO_FMT(__func__,
			"PM Sweep: %u Memory Used: %"PRIu64" MB (%"PRIu64" KB)",
			pm->NumSweeps, (uint64)(size/0x100000ull), (uint64)(size/0x400ull));
	}
}
