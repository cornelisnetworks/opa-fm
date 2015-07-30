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


#ifndef _IBA_PUBLIC_ITHREAD_OSD_H_
#define _IBA_PUBLIC_ITHREAD_OSD_H_

#include "iba/public/datatypes.h"
#include "iba/public/ievent.h"
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_THREAD 0	/* set to 1 to enable DbgOut for thread */

/*---------------------------------------------------------
 * OS dependent data structure used by the thread object.
 * Users should not access these variables directly.
 *--------------------------------------------------------- */

typedef struct _THREAD_OSD
{
	uint32			tr_flags;
	char                    tr_name[THREAD_NAME_SIZE];
   	pthread_t               tr_id;
   	pthread_attr_t          tr_attr;
   	EVENT                   tr_RunningEvent;
   	ObjectState             tr_state;    
} THREAD_OSD;

/* for LINUX these are nice values */
typedef enum {
	THREAD_PRI_VERY_HIGH = -15,
	THREAD_PRI_HIGH = -10,
	THREAD_PRI_NORMAL = 0,
	THREAD_PRI_LOW = 10,
	THREAD_PRI_VERY_LOW = 15
} THREAD_PRI;

typedef struct _WORK_REQUEST_OSD 
{
   uint32 Foo;
} WORK_REQUEST_OSD;

#ifdef __cplusplus
};
#endif

#endif /* _IBA_PUBLIC_ITHREAD_OSD_H_ */
