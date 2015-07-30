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


#ifndef _IBA_PUBLIC_ISYSCALLBACK_H_
#define _IBA_PUBLIC_ISYSCALLBACK_H_

#include "iba/public/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*SYSTEM_CALLBACK)( IN void *Key, IN void *Context );
#ifdef __cplusplus
};
#endif


#ifdef _NATIVE_SYSCALLBACK
/* Include platform specific system callback support. */
#include "iba/public/isyscallback_osd.h"

#else  /* Use this public version */
#include "iba/public/ilist.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _SYS_CALLBACK_ITEM
{
	SYSTEM_CALLBACK 	Callback;
	void 				*Key;
	void 				*Context;

	/* list maintenance */
	LIST_ITEM			ListItem;
} SYS_CALLBACK_ITEM;


/*
 * Return TRUE if initialization is successful.
 *        FALSE if initialization failed.
 */
boolean
SysCallbackInit(void);

void
SysCallbackDestroy(void);

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _NATIVE_SYSCALLBACK */

#ifdef __cplusplus
extern "C"
{
#endif

SYS_CALLBACK_ITEM *
SysCallbackGet( 
	IN void *Key );

void
SysCallbackPut( 
	IN SYS_CALLBACK_ITEM *pItem );

void
SysCallbackQueue( 
	IN SYS_CALLBACK_ITEM *pItem, 
	IN SYSTEM_CALLBACK Callback,
	IN void *Context, 
	IN boolean HighPriority );

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif	/* _IBA_PUBLIC_ISYSCALLBACK_H_ */
