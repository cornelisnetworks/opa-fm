/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//---------------------------------------------------------------------//
//                                                                     //
// FILE NAME                                                           //
//    os_g.h							       //
//                                                                     //
// DESCRIPTION                                                         //
//    general global include file for OS specific files		       //
//                                                                     //
// DATA STRUCTURES                                                     //
//    None                                                             //
//                                                                     //
// FUNCTIONS                                                           //
//    None                                                             //
//                                                                     //
// DEPENDENCIES                                                        //
//    None                                                             //
//                                                                     //
//                                                                     //
//---------------------------------------------------------------------//

#ifndef	_OS_G_H_
#define	_OS_G_H_

//---------------------------------------------------------------------//
//                                                                     //
//      Linux defines.                                                 //
//                                                                     //
//---------------------------------------------------------------------//

#if	defined(__LINUX__)

//#ifdef CAL_IBACCESS
#include <sys/types.h>
//#endif

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/* Get a random number between two numbers.
 * a%b will return a value between 0 and b-1 (inclusive).
 * By doing rand()%(HIGH-LOW) we get a random index between HIGH and LOW
 * Add that value to LOW to get a random number between HIGH and LOW.
 */
#define GET_RANDOM(HIGH, LOW)   ((rand() % (HIGH-LOW))+LOW)

/* Macros for min/max.  */
#ifndef MIN
#define	MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define	MAX(a,b) (((a)>(b))?(a):(b))
#endif

// ASSERTs which are only tested in DEBUG builds
// x can be a function call or expensive expression, do not evaluate
// it in debug builds
#ifndef DEBUG_ASSERT
#if DEBUG
#define DEBUG_ASSERT(x)		ASSERT(x)
#else
#define DEBUG_ASSERT(x)		do {} while(0)
#endif
#endif

//---------------------------------------------------------------------//
//                                                                     //
//      FreeBSD defines.                                               //
//                                                                     //
//---------------------------------------------------------------------//

#elif	defined(__FreeBSD__)

//---------------------------------------------------------------------//
//                                                                     //
//      Windows 32 defines.                                            //
//                                                                     //
//---------------------------------------------------------------------//

#elif	defined(WIN32)

//---------------------------------------------------------------------//
//                                                                     //
//      __VXWORKS__ defines.                                           //
//                                                                     //
//---------------------------------------------------------------------//

#elif   defined(__VXWORKS__)
extern unsigned long SystemGetRandom(void);
/* Get a random number between two numbers.
 * a%b will return a value between 0 and b-1 (inclusive).
 * By doing rand()%(HIGH-LOW) we get a random index between HIGH and LOW
 * Add that value to LOW to get a random number between HIGH and LOW.
 */
#define GET_RANDOM(HIGH, LOW)   ((SystemGetRandom() % (HIGH-LOW))+LOW)

/* Macros for min/max.  */
#ifndef MIN
#define	MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define	MAX(a,b) (((a)>(b))?(a):(b))
#endif

//---------------------------------------------------------------------//
//                                                                     //
//      Unknown OS defines.                                            //
//                                                                     //
//---------------------------------------------------------------------//

#else

#endif

#endif	// _OS_G_H_

