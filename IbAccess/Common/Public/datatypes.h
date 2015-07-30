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

#ifndef _IBA_PUBLIC_DATA_TYPES_H_
#define _IBA_PUBLIC_DATA_TYPES_H_


#include "iba/public/datatypes_osd.h"

/* TBD - use /usr/include/stdint.h??? */

#ifdef __cplusplus
extern "C" {
#endif

/* Minimum of signed integral types.  */
# define IB_INT8_MIN		(-128)
# define IB_INT16_MIN		(-32767-1)
# define IB_INT32_MIN		(-2147483647-1)
# define IB_INT64_MIN		(-9223372036854775807ll -1)
/* Maximum of signed integral types.  */
# define IB_INT8_MAX		(127)
# define IB_INT16_MAX		(32767)
# define IB_INT32_MAX		(2147483647)
# define IB_INT64_MAX		(9223372036854775807ll)
/* Maximum of unsigned integral types.  */
# define IB_UINT8_MAX		(255)
# define IB_UINT16_MAX		(65535)
# define IB_UINT32_MAX		(4294967295U)
# define IB_UINT64_MAX		(18446744073709551615ULL)

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((uintn) &((TYPE *)0)->MEMBER)
#endif

#ifdef __cplusplus
/* Currently the DTA fails to compile because of a bug
 * in 3.4.2-6.fc3 that ships with fc3 and el4 beta.
 * A workaround is possible, but I'm waiting to see what
 * redhats response is to bug 146385.
 */
#define PARENT_STRUCT(ADDRESS, TYPE, MEMBER) \
    ((TYPE *)((char *)(ADDRESS) - (uintn)(&(((TYPE*)0)->* (&TYPE::MEMBER)))))
#else
#define PARENT_STRUCT(ADDRESS, TYPE, MEMBER) \
    ((TYPE *)((char *)(ADDRESS) - offsetof(TYPE, MEMBER))) 
#endif

#ifndef		IN
#define		IN			/* Function input parameter */
#endif
#ifndef		OUT
#define		OUT			/* Function output parameter */
#endif
#ifndef		OPTIONAL
#define		OPTIONAL	/* Optional function parameter - NULL if not used */
#endif


/****************************************************************************
 *                  Function Returns And Completion Codes					*
 *																			*
 * The text for any addition to this enumerated type must be added to the 	*
 * string array defined in <statustext.h>.									*
 *																			*
 ****************************************************************************/

/*
 * The generic error codes are listed here. The lower 8 bits of FSTATUS
 * define the generic error codes. Modules using this can use the top
 * 24 bits to encode module specific error codes.
 */
typedef uint32 FSTATUS;

#define FSUCCESS					0x00
#define FERROR						0x01
#define FINVALID_STATE				0x02
#define FINVALID_OPERATION			0x03
#define FINVALID_SETTING			0x04
#define FINVALID_PARAMETER			0x05
#define FINSUFFICIENT_RESOURCES		0x06
#define FINSUFFICIENT_MEMORY		0x07
#define FCOMPLETED					0x08
#define FNOT_DONE					0x09
#define FPENDING					0x0A
#define FTIMEOUT					0x0B
#define FCANCELED					0x0C
#define FREJECT						0x0D
#define FOVERRUN					0x0E
#define FPROTECTION					0x0F
#define FNOT_FOUND					0x10
#define FUNAVAILABLE				0x11
#define FBUSY						0x12
#define FDISCONNECT					0x13
#define FDUPLICATE					0x14
#define FPOLL_NEEDED				0x15

#define FSTATUS_COUNT				0x16 /* should be the last value */

/****************************************************************************
 *                 		Debug Output Level									*
 ****************************************************************************/
typedef enum _FDEBUG_LEVEL
{
	FDEBUG_OFF,
	FDEBUG_ERROR,
	FDEBUG_TERSE,
	FDEBUG_NORMAL,
	FDEBUG_VERBOSE

} FDEBUG_LEVEL;

#ifdef __cplusplus
};
#endif


#endif /* _IBA_PUBLIC_DATA_TYPES_H_ */
