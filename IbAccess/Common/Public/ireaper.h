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

#ifndef _IBA_PUBLIC_IREAPER_H_
#define _IBA_PUBLIC_IREAPER_H_

#include "iba/public/datatypes.h"
#include "iba/public/ilist.h"

/*
 * The Reaper helps to Destroy objects.  It allows a object to
 * schedule itself for later destruction.  As such it helps to avoid
 * the complex issue of an object needing to destroy itself within its
 * own callback, Such as when a timer callback needs to destroy the object
 * containing the timer.
 *
 * In general objects should mostly delete themselves before queueing to the
 * Reaper, this will avoid odd races or object existance issues.
 * The Reaper callback for the object should focus on destroying simpler
 * datatypes (such as timers) and deleting the object.
 * The object queued to the reaper should not be queued to any other lists
 *
 * While on the reaper queue, the LIST_ITEM->m_Object will be changed.
 * The callback must use PARENT_STRUCT to get from the pItem supplied
 * to the full object.
 *
 * Internally, the reaper uses the Syscallback mechanism to schedule its
 * callbacks.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*REAPER_CALLBACK)( IN LIST_ITEM *pItem );

boolean	
ReaperInit(void);

void	
ReaperDestroy(void);

void
ReaperQueue(
	IN LIST_ITEM *pItem,
	IN REAPER_CALLBACK Callback
	);

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IREAPER_H_ */
