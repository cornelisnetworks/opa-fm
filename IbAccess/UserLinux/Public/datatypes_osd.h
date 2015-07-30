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


#ifndef _IBA_PUBLIC_DATA_TYPES_OSD_H_
#define _IBA_PUBLIC_DATA_TYPES_OSD_H_

#include <inttypes.h>
#include <assert.h>
#include <pthread.h>
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

/* 
 * This file defines the data types specific to the Linux Operating System.
 * Most of the basic type definitions are picked up from <linux/types.h>
 */

#ifndef TRUE
#define TRUE  	1
#endif

#ifndef FALSE
#define FALSE  	0
#endif

#define IBA_API
#define _NATIVE_MEMORY

#if defined(__PGI) && defined(__X86_64__)
/* PGI C compiler's linux86-64/6.1/include/sys/types.h header does not define
 * this, oddly their linux86/6.1/include/sys/types.h header does
 */
typedef unsigned long u_int64_t;
#endif

#ifndef IB_STACK_OPENIB
typedef unsigned char 	boolean_t, boolean;
#else
typedef unsigned char 	boolean;
#endif
typedef uint8_t			uint8;	
typedef int8_t			int8;	
typedef uint16_t		uint16;
typedef int16_t			int16;
typedef uint32_t		uint32;
typedef int32_t			int32;
typedef int64_t			int64;
typedef uint64_t		uint64;
typedef long			intn; 	/* __WORDSIZE/pointer sized integer */
typedef unsigned long	uintn; 	/* __WORDSIZE/pointer sized integer */
typedef unsigned char 	uchar; 

/* Global state for all the objects that are created using the public
 * layer (Events, Timers, Threads etc.)
 */

typedef enum _ObjectState {
  Constructed = 0,
  Started,
  Destroyed
} ObjectState;

/* Copied from SLES9 compiler.h */
#if (__GNUC__ > 3) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1)

#ifndef inline
#define inline		__inline__
#endif /* inline */

#ifndef __inline__
#define __inline__	__inline__
#endif /* __inline__ */

#ifndef __inline
#define __inline	__inline__
#endif /* __inline */

#endif /* (__GNUC__ > 3) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1) */

/* Copied from SLES10 compiler.h */
#ifndef __always_inline
#define __always_inline	inline
#endif

#ifndef __inline

#if ! defined(__PATHSCALE__) && ! defined(__PGI) && (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1))
#define inline __attribute__((always_inline)) inline
#define __inline __attribute__((always_inline)) inline
#else
#define __inline inline
#endif /* GNUC */

#endif /* __inline */

#define _inline inline

#define CONTAINING_RECORD(address, type, field)	\
	((type *)((char *)(address) - (uintn)(&((type *)0)->field)))

extern void PanicOnAssert (char *exp, char *filename, int linenum, char *message);

#if defined(__PATHSCALE__) || (__GNUC__)
/* These are a few macros to aid compiler in branch prediction */
	/* tell compiler typical value for an expression */
#define IB_EXPECT(expr, expected) __builtin_expect((expr), (expected))
	/* tell compiler typical expectation for conditional expression */
#define IB_EXPECT_TRUE(cond) __builtin_expect(!!(cond), 1)
#define IB_EXPECT_FALSE(cond) __builtin_expect(!!(cond), 0)
#else
#define IB_EXPECT(expr, expected) (expr)
#define IB_EXPECT_TRUE(expr) (expr)
#define IB_EXPECT_FALSE(expr) (expr)
#endif

/* macro to tell compiler a variable may be unused and to suppress any related
 * warnings.  Useful to reduce clutter when code has ifdef's or debug code
 * this macro can be used as follows:
 *	int var IB_UNUSED;
 * it can be used for parameters and variables
 */
#define IB_UNUSED __attribute__((unused))

#undef DEBUG_ASSERT
#ifndef	ASSERT
extern void BackTrace(FILE *file);
#define	ASSERT(__exp__)	do { if (IB_EXPECT_FALSE(! (__exp__))) { BackTrace(stderr); assert(__exp__); } } while(0);
#endif	/* ASSERT */
#if defined(IB_DEBUG) || defined(DBG)
#define DEBUG_ASSERT(__exp__) ASSERT(__exp__)
#else
#define DEBUG_ASSERT(__exp__) (void)0
#endif

#define __cdecl

#ifdef __cplusplus
};
#endif

#endif /* _IBA_PUBLIC_DATA_TYPES_OSD_H_ */
