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


#ifndef _IBA_PUBLIC_IEVENT_H_
#define _IBA_PUBLIC_IEVENT_H_

/* Indicates that waiting on an event should never timeout */
#define EVENT_NO_TIMEOUT	((int32)0xFFFFFFFF)

#include "iba/public/ievent_osd.h"


#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Function declarations.
 *
 * Initialize a event before its use.
 */
void
EventInitState( 
	IN EVENT *pEvent );

boolean
EventInit( 
	IN EVENT *pEvent );

/* Destroy the event when through using it. */
void
EventDestroy(
	IN EVENT *pEvent );

/* Signal an event. */
void
EventTrigger(
	IN EVENT *pEvent );

/* Signal all waiters of an event. */
void
EventBroadcast(
	IN EVENT *pEvent);

/* Clear an event from being signaled. */
void
EventClear( 

	IN EVENT *pEvent );

/* Wait on an event until it's signaled. */
FSTATUS
EventWaitOn( 
	IN EVENT *pEvent, 
	IN int32 wait_us );

FSTATUS
EventWaitOnInterruptible( 
	IN EVENT *pEvent, 
	IN int32 wait_micro );
		
#ifdef _INDIRECT_NATIVE_EVENT	/* set in ievent_osd.h */

/* Return a pointer to the native, platform specific event. */
void*
EventGetNative(
	IN EVENT *pEvent );

/* Signal an event, given a handle to the platform specific event. */
void
EventTriggerNative(
	IN void *pNativeEvent );

#else

/* Return a pointer to the native, platform specific event. */
static __inline void*
EventGetNative(
	IN EVENT *pEvent )
{
	return (void*)pEvent;
}


/* Signal an event, given a handle to the platform specific event. */
static __inline void
EventTriggerNative(
	IN void *pNativeEvent )
{
	EventTrigger( (EVENT*)pNativeEvent );
}

#endif	/* _INDIRECT_NATIVE_EVENT */



#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IEVENT_H_ */
