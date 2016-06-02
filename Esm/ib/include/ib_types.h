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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//----------------------------------------------------------------------------//
//                                                                            //
// FILE NAME                                                                  //
//    ib_types.h                                                              //
//                                                                            //
// DESCRIPTION                                                                //
//    Basic VIEO and IB type definitions                                      //
//                                                                            //
// DATA STRUCTURES                                                            //
//    None								      //
//                                                                            //
// FUNCTIONS                                                                  //
//    None                                                                    //
//                                                                            //
// DEPENDENCIES                                                               //
//    None                                                                    //
//                                                                            //
//                                                                            //
// PJG  05/29/02    PR2176: Include keyword 'signed' in int8_t typedefs.      //
//----------------------------------------------------------------------------//

#ifndef	_IB_TYPES_H_
#define	_IB_TYPES_H_

//----------------------------------------------------------------------------//
//                                                                            //
//	Fundamental data types.  There should be a set for each type          //
//	of compiler/architecture that we need to use.                         //
//                                                                            //
//----------------------------------------------------------------------------//
typedef unsigned long unint;   /* define native unsigned integer (32 or 64 bit) */
typedef long nint;             /* define native integer (32 or 64 bit) */


//----------------------------------------------------------------------------//
//                                                                            //
//      Linux defines.                                                        //
//                                                                            //
//----------------------------------------------------------------------------//

#if	defined(__LINUX__)

#include <sys/types.h>				// int*_t defined here
#include <stdint.h>				// uint*_t defined here

//----------------------------------------------------------------------------//
//                                                                            //
//      FreeBSD defines.                                                      //
//                                                                            //
//----------------------------------------------------------------------------//

#elif	defined(__FreeBSD__)

#include <sys/inttypes.h>

//----------------------------------------------------------------------------//
//                                                                            //
//      Windows 32 defines.                                                   //
//                                                                            //
//----------------------------------------------------------------------------//

#elif	defined(WIN32)

typedef	unsigned char		uint8_t;
typedef	unsigned short		uint16_t;
typedef	unsigned int		uint32_t;
typedef unsigned __int64	uint64_t;

typedef	signed char     int8_t;
typedef	short			int16_t;
typedef	int			int32_t;
typedef	__int64			int64_t;


//----------------------------------------------------------------------------//
//                                                                            //
//      vxWorks defines.                                                          //
//                                                                            //
//----------------------------------------------------------------------------//

#elif   defined(__VXWORKS__)

#include <sys/types.h>
// VxWorks for ATOM defines uint64_t and int64_t



//----------------------------------------------------------------------------//
//                                                                            //
//      Unknown OS defines.                                                   //
//                                                                            //
//----------------------------------------------------------------------------//

#else

typedef	unsigned char		uint8_t;
typedef	unsigned short		uint16_t;
typedef	unsigned int		uint32_t;
typedef unsigned long long int	uint64_t;

typedef	signed char     int8_t;
typedef	short			int16_t;
typedef	int			int32_t;
typedef	long long int		int64_t;

#endif

//----------------------------------------------------------------------------//
//                                                                            //
//	Fundamental IBA types.  These are defined only in terms of the        //
//	types above.                                                          //
//                                                                            //
//----------------------------------------------------------------------------//

typedef	uint32_t		Lid_t;
typedef	uint64_t		Guid_t;
typedef	uint16_t		PKey_t;
typedef	uint16_t		QKey_t;
typedef	uint64_t		BKey_t;
typedef	uint64_t		MKey_t;
typedef	uint64_t		SAKey_t;
typedef	uint64_t		CCKey_t;
typedef	uint8_t			Gid_t[16];

//----------------------------------------------------------------------------//
//                                                                            //
//	Other fundamental types for the implementation.                       //
//                                                                            //
//----------------------------------------------------------------------------//

typedef nint			IBhandle_t;	// -1 => no handle (NULL)

#define	IB_NULLH		(-1)

#endif	/* _IB_TYPES_H_ */

