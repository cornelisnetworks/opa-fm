/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IDEBUG_OSD_H_
#define _IBA_PUBLIC_IDEBUG_OSD_H_

#include "iba/public/datatypes.h"
#include <bits/wordsize.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>

#ifdef ___cplusplus
extern "C" {
#endif

extern void IbLogPrintf(uint32 level, const char* format, ...)
#if defined(__GNUC__)
				__attribute__((format(printf, 2, 3)))
#endif
				;
extern void PrintUDbg( char *Message, ... )
#if defined(__GNUC__)
				__attribute__((format(printf, 1, 2)))
#endif
				;
extern void PrintUMsg( char *Message, ... )
#if defined(__GNUC__)
				__attribute__((format(printf, 1, 2)))
#endif
				;

#if defined(IB_DEBUG) || defined(DBG)
#ifndef DbgOutRoutine
#define DbgOutRoutine PrintUDbg
extern void PrintUDbg(IN char *DebugMessage, IN ... );
#endif
#ifdef  __ia64
#define DbgBreakPoint() asm("   break 0")
#elif defined(ppc)
#define DbgBreakPoint() asm("   trap")
#else
#define DbgBreakPoint() asm("   int $3")
#endif
#else
#define DbgOut
#endif

#define MsgOut	PrintUMsg
#undef __PRI64_PREFIX
#if __WORDSIZE == 64
#define __PRI64_PREFIX	"l"
#define PRISZT "lu"
#elif __WORDSIZE == 32
#define __PRI64_PREFIX	"L"
#define PRISZT "u"
#else
#error "__WORDSIZE not 64 nor 32"
#endif

#undef __PRIN_PREFIX
#undef PRId64
#undef PRIo64
#undef PRIu64
#undef PRIx64
#undef PRIX64
#undef PRIdN 
#undef PRIoN 
#undef PRIuN 
#undef PRIxN
#define __PRIN_PREFIX	"l"

#define PRId64		__PRI64_PREFIX"d"
#define PRIo64		__PRI64_PREFIX"o"
#define PRIu64		__PRI64_PREFIX"u"
#define PRIx64		__PRI64_PREFIX"x"
#define PRIX64		__PRI64_PREFIX"X"
#define PRIdN		__PRIN_PREFIX"d"
#define PRIoN		__PRIN_PREFIX"o"
#define PRIuN		__PRIN_PREFIX"u"
#define PRIxN		__PRIN_PREFIX"x"

#ifdef ___cplusplus
}
#endif

#endif /* _IBA_PUBLIC_IDEBUG_OSD_H_ */
