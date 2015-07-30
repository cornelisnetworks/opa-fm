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

#ifndef _IBA_PUBLIC_ITIMER_H_
#define _IBA_PUBLIC_ITIMER_H_

#include "iba/public/itimer_osd.h"

/* Timer Overview
 *
 * Timers provide a kernel level mechanism for scheduling a callback
 * in the future.
 *
 * TimerInit and TimerDestroy may only be used at base (not interrupt/tasklet)
 * level
 *
 * TimerStart and TimerStop may be used anywhere, including in the timer
 * callback itself
 *
 * TimerCallbackStopped should only be used in the callback itself.
 *
 * Timer callbacks should be assumed to be at interrupt level, it can vary
 * per OS.
 *
 * TimerStop will schedule the stop of the timer and make a best attempt
 * at stopping it.  However, it may not be completely successful.
 * If the callback is already starting, it may occur after TimerStop.
 * In which case the timer code will skip stale callbacks and not call the
 * consumer callback.
 * However if a callback is in progress and has already started to
 * execute the consumer callback function, TimerStop will not stop it nor
 * will TimerStop preempt nor wait for the callback to finish.
 * See the discussion of TimerCallbackStopped for a dicussion of the
 * recommended locking and usage model to prevent races and deadlocks.
 * (This behaviour is necessary such that TimerStop will not deadlock with
 * a caller who holds a consumer lock and is stopping the timer while the
 * callback has already started and is waiting for the consumer lock).
 *
 * TimerDestroy will wait for any callbacks to complete.  After the completion
 * of TimerDestroy the caller can safely assume no more callbacks will occur
 * nor are any still executing.
 *
 * If TimerDestroy is called while holding a lock which the callback might
 * also attempt to get, a deadlock can occur.  In which case the Reaper
 * should be used to destroy the object or the TimerDestroy should be done
 * outside of the lock if safe.  In simple cases, the sequence:
 *		get lock
 *		TimerStop
 *		release lock
 *		TimerDestroy
 * is safe if the callback is following the locking model recommended in
 * TimerCallbackStopped
 */


#ifdef __cplusplus
extern "C" {
#endif

/* Return the number of micro secs that have passed since the system was booted. */
uint64
GetTimeStamp( void );

/* Return the number of seconds that have passed since the system was booted. */
uint32
GetTimeStampSec( void );

#if defined(VXWORKS)
/*
 * Callback function invoked once the timer expires.
 */
typedef void (*TIMER_CALLBACK)( IN void *Context );


/*
 * Private data structure used by the spinlock object.
 * Users should not access these variables directly.
 */
typedef struct _TIMER_PRIV
{
	TIMER_OSD		m_Osd;
	TIMER_CALLBACK	m_TimerCallback;
	void			*m_Context;
} TIMER_PRIV;



/*
 * Timer object definition.
 */
typedef struct _TIMER
{
	/* No publically accessible members. */
	TIMER_PRIV	m_Private;
} TIMER;


/*
 * Function declarations.
 */
/* Initialize a timer before its use. */
void
TimerInitState(
	IN TIMER *pTimer );

boolean
TimerInit( 
	IN TIMER *pTimer, 
	IN TIMER_CALLBACK TimerCallback, 
	IN void *Context );

/* Destroy the timer object when done using it. */
void
TimerDestroy( 
	IN TIMER *pTimer );

/* Start the timer to alert at the specified time. */
boolean
TimerStart(	
	IN TIMER *pTimer, 
	IN uint32 time_ms );

/* Stop the timer from alerting. */
void TimerStop( 
	IN TIMER *pTimer );

/* used in a timer callback to determine if the callback is applicable
 * Mainly needed if timer callback gets a lock which is shared with code
 * outside the timer.  Helps to catch if while callback waited for lock
 * the toplevel code has gotten lock, stopped timer, and done other stuff.
 * TRUE - callback was stopped and should not do anything
 * FALSE - callback is real and should perform callback actions
 *
 * In general it is recommended that objects shared between timer callbacks
 * and other code should have a lock for the object or subsystem.
 * In which case the typical callback should be structured as:
 * callback(void *context)
 * {
 *     lock
 *     if (TimerCallbackStopped( timer))
 *        unlock
 *        return
 *     process callback
 *     unlock
 * }
 *
 * and other code using object should be structured as:
 *		lock
 *		do stuff
 *		TimerStop
 *		other stuff, possibly TimerStart
 *		unlock
 *
 */
boolean TimerCallbackStopped(TIMER *pTimer);

/* Indicates if timer has expired and callback has or will be run */
boolean TimerExpired(TIMER *pTimer);
#endif /* defined(VXWORKS) */

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ITIMER_H_ */
