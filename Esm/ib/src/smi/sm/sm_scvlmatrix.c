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
#include "sm_l.h"
#include "sm_qos.h"

static const uint8_t sm_globalSCVLMatrix[SCVLMAP_BASE + 1][SCVLMAP_MAX_INDEX] = {
/* VL-8 */
[8]{ 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 15,
     0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7},
/* VL-7 */
[7]{ 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 15,
     0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 6, 15},
/* VL-6 */
[6]{ 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 15,
     0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 5, 15},
/* VL-5 */
[5]{ 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 15,
     0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 4, 15},
/* VL-4 */
[4]{ 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 15,
     0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 3, 15},
/* VL-3 */
[3]{ 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 15,
     0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 2, 15},
/* VL-2 */
[2]{ 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 15,
     0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 15},
/* VL-1 */
[1]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

void sm_fill_SCVLMap(Qos_t *qos)
{
	uint8_t vlsel = qos->numVLs;
	uint8_t i;
	bitset_t scvl_activevl_bitmap;

	if (vlsel > SCVLMAP_BASE) {
		vlsel = SCVLMAP_BASE;
	}

	if (vlsel < 1) {
		IB_LOG_ERROR_FMT(__func__, "index selection is invalid %d", vlsel);
		return;
	}

	if (SCVLMAP_MAX_INDEX != STL_MAX_SCS) {
		IB_LOG_ERROR_FMT(__func__, "scvlmap index max [%d] does not match max SCs [%d]", SCVLMAP_MAX_INDEX, STL_MAX_SCS);
		return;
	}

	if (!bitset_init(&sm_pool, &scvl_activevl_bitmap, SCVLMAP_MAX_INDEX)) {
		IB_LOG_ERROR_FMT(__func__, "cannot allocate scvl active VL bitmap");
		return;
	}

	for (i = 0; i < SCVLMAP_MAX_INDEX; i++) {
		/* mark active VLs */
		if (sm_globalSCVLMatrix[vlsel][i] != 15) {
			bitset_set(&scvl_activevl_bitmap, sm_globalSCVLMatrix[vlsel][i]);
		}
		qos->scvl.SCVLMap[i].VL = sm_globalSCVLMatrix[vlsel][i];
	}

	/* count actual data VLs in use (except SCs mapped to VL15) */
	qos->activeVLs = bitset_nset(&scvl_activevl_bitmap);
	bitset_free(&scvl_activevl_bitmap);

	return;
}

