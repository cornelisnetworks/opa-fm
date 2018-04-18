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

/* [ICS VERSION STRING: unknown] */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iba/ibt.h>
#include <stdarg.h>
#include "ibprint.h"

// TBD: now universally using 0x---:---- for GIDs
// saquery used to use 0x---:0x--- in a few places but
// mostly used 0x----:-----

void PrintGuid(PrintDest_t *dest, int indent, EUI64 Guid)
{
	PrintFunc(dest, "%*s0x%016"PRIx64"\n", indent, "", Guid);
}

void PrintLid(PrintDest_t *dest, int indent, IB_LID Lid)
{
	PrintFunc(dest, "%*s0x%04x\n", indent, "", Lid);
}

void PrintGid(PrintDest_t *dest, int indent, const IB_GID *pGid)
{
	PrintFunc(dest, "%*s0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",
				pGid->AsReg64s.H,
				pGid->AsReg64s.L);
}

void PrintLongMaskBits(PrintDest_t *dest, int indent, const char* prefix, const uint8 *bits, unsigned size) 
{
	char buf[80];
	int buflen;
	unsigned i;

#if 0
	// hex dump of field to aid debug
	buflen = sprintf(buf, "%s ", prefix);
	for (i=0; i<size/8; i++) {
			if (buflen > 60) {
				PrintFunc(dest, "%*s%s\n", indent, "", buf);
				buflen=sprintf(buf, "    ");	// indent list continuation
			}
			buflen +=sprintf(buf+buflen, "%02x", bits[i]);
	}
	PrintFunc(dest, "%*s%s\n", indent, "", buf);
#endif
	buflen = snprintf(buf, sizeof(buf), "%s", prefix);
	for (i=0; i<size; i++) {
		if (bits[LONG_MASK_BIT_INDEX(i, size)]&LONG_MASK_BIT_MASK(i)) {
			if (buflen > 60) {
				PrintFunc(dest, "%*s%s\n", indent, "", buf);
				buflen=sprintf(buf, "    ");	// indent list continuation
			}
			buflen +=sprintf(buf+buflen, " %3u", i);
		}
	}
	PrintFunc(dest, "%*s%s\n", indent, "", buf);
}

void PrintSeparator(PrintDest_t *dest)
{
	PrintFunc(dest, "-------------------------------------------------------------------------------\n");
}

// TBD - add a hex dump of a buffer, see idebugdump.c for example
// FormatChars in sa.c
// can just do the BYTES format
