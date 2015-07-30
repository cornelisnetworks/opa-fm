/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IMATH_H_
#define _IBA_PUBLIC_IMATH_H_

#include "iba/public/datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef MAX
#define MAX(x,y)	((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x,y)	((x) < (y) ? (x) : (y))
#endif

/* round up value to align, align must be a power of 2 */
#ifndef ROUNDUPP2
#define ROUNDUPP2(val, align)	\
	(((uintn)(val) + (uintn)(align) - 1) & (~((uintn)(align)-1)))
#endif
/* force to use 64 bits in 32bit box */
#ifndef ROUNDUP64P2
#define ROUNDUP64P2(val, align)   \
        (((uint64)(val) + (uint64)(align) - 1) & (~((uint64)(align)-1)))
#endif

/* round up value to align, align can be any value, less efficient than ROUNDUPP2 */
#ifndef ROUNDUP
#define ROUNDUP(val, align)	\
	((( ((uintn)(val)) + (uintn)(align) -1) / (align) ) * (align))
#endif

/* round down value to align, align must be a power of 2 */
#ifndef ROUNDDOWNP2
#define ROUNDDOWNP2(val, align)	\
	(((uintn)(val)) & (~((uintn)(align)-1)))
#endif
/* round down value to align, align can be any value, less efficient than ROUNDDOWNP2 */
#ifndef ROUNDDOWN
#define ROUNDDOWN(val, align)	\
	((( ((uintn)(val))) / (align) ) * (align))
#endif

/* convert bytes to Megabytes, rounding up */
#define BYTES_TO_MB(val) (((val) + ((1024*1024)-1))>> 20)
	

/* log2(x) truncated */
uint32 FloorLog2(uint64 x);

/* log2(x) rounded up if x is not a power of 2 */
uint32  CeilLog2(uint64 x);

static _inline uint32 NextPower2(uint64 x)
{
	return (1 << CeilLog2(x));
}

#if defined(__cplusplus)
}
#endif

#endif /* _IBA_PUBLIC_IMATH_H_ */
