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


/*
 * This provides defines __LITTLE_ENDIAN, __BIG_ENDIAN and __BYTE_ORDER
 */

#ifndef _IBA_PUBLIC_IBYTESWAP_OSD_H_
#define _IBA_PUBLIC_IBYTESWAP_OSD_H_

#include "iba/public/datatypes.h"
#include <endian.h>
#include <byteswap.h>

#if !defined(bswap_64) || (__PGIC__)
/* unfortunately when compiling with PGI C, the macro is not defined by
 * /usr/include/bits/byteswap.h and, in fact, newer versions declare a
 * prototype for bswap_64() but then never provide the function...
 */
#if __BYTE_ORDER == __LITTLE_ENDIAN
static uint64 bswap_64(uint64 x)
{
	union {
		uint64 __u64;
		uint32 __u32[2];
	} __in, __out;
	__in.__u64 = x;
	__out.__u32[0] = bswap_32(__in.__u32[1]);
	__out.__u32[1] = bswap_32(__in.__u32[0]);
	return __out.__u64;
}
#else
#define bswap_64(x) (x)
#endif	/* endian */
#endif	/* bswap_64 */

/*
 * Convert multi-byte words for the local machine architecture.
 * This is a byte swap on small-endian machines, but not on big-endian.
 */
#if defined(__LITTLE_ENDIAN) && (__BYTE_ORDER == __LITTLE_ENDIAN)
#define ntoh16(x)	bswap_16(x)
#define hton16(x)	bswap_16(x)

#define ntoh32(x)	bswap_32(x)
#define hton32(x)	bswap_32(x)

#define ntoh64(x)	bswap_64(x)
#define hton64(x)	bswap_64(x)
#else
#define ntoh16(x)	(x)
#define hton16(x)	(x)

#define ntoh32(x)	(x)
#define hton32(x)	(x)

#define ntoh64(x)	(x)
#define hton64(x)	(x)
#endif

#endif /* _IBA_PUBLIC_IBYTESWAP_H_ */
