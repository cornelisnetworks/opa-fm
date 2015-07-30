/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#ifndef _RMPP_COUNTERS_H_
#define _RMPP_COUNTERS_H_ 1

#include "ispinlock.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "mai_g.h"


typedef enum _rmpp_peak_counters {
	rmppMaxContextsInUse,
	rmppMaxContextsFree,
	rmppMaxSweepTime,					// in ms

	rmppPeakCountersMax // Last value
} rmpp_peak_counters_t;

typedef struct _rmpp_counter {
	char * name;
	ATOMIC_UINT sinceLastSweep;
	ATOMIC_UINT lastSweep;
	ATOMIC_UINT total;
} rmpp_counter_t;

//
// These are the arrays that contain the counter values... They're definition
// is in rmpp_counters.c. If you add a counter to either list, be sure to
// add a description for it in rmpp_counters.c
//
// The rmppPeakCounters array is for counters that you'll want to use the
// SET_PEAK_COUNTER() function on.
//
//extern rmpp_counter_t rmppPeakCounters[rmppPeakCountersMax];


//
// This function compares and sets a value in the rmppPeakCounters array if
// it's greater that the current value.
//
static __inline__
void SET_RMPP_PEAK_COUNTER(rmpp_peak_counters_t counter, const uint32 value) {
/*--- DEBUG_CODE --- IMPLEMENT
	uint32 tmp;

	do {
		tmp = AtomicRead(&rmppPeakCounters[counter].sinceLastSweep);
	} while (  (value > tmp)
	        && ! AtomicCompareStore(&rmppPeakCounters[counter].sinceLastSweep,
	                                tmp, value));

	do {
		tmp = AtomicRead(&rmppPeakCounters[counter].total);
	} while (  (value > tmp)
	        && ! AtomicCompareStore(&rmppPeakCounters[counter].total,
	                                tmp, value));
*/
}

// Function prototypes
extern void pm_init_counters(void);
extern void pm_reset_counters(void);
extern void pm_print_counters_to_stream(FILE * out);

#ifndef __VXWORKS__
extern char * pm_print_counters_to_buf(void);
#endif

#endif	// _RMPP_COUNTERS_H_
