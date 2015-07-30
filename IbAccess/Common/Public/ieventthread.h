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


#ifndef _IEVENT_THREAD_H_
#define _IEVENT_THREAD_H_

#include "iba/public/ithread.h"
#include "iba/public/ievent.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Event Thread object.  Thread will wait on an event, then invoke a callback.
 */
typedef struct _EVENT_THREAD
{
	/* Routine called by thread when the specified event occurs. */
	THREAD_CALLBACK	m_Callback;
	void			*m_Context;

	THREAD			m_Thread;
	EVENT			m_Event;
	boolean			m_Exit;
	boolean			m_Initialized;
	THREAD_PRI		m_Priority;

} EVENT_THREAD;

void	
EventThreadInitState( 
	IN EVENT_THREAD *pEventThread );

boolean 
EventThreadCreate( 
	IN EVENT_THREAD *pEventThread, 
	IN char* Name,
	IN THREAD_PRI Priority,
	IN THREAD_CALLBACK Callback, 
	IN void *Context );

/* Stop must be called at passive level.
 * stops thread and waits for it.  After this no more Callbacks will
 * occur, calls to EventThreadSignal will do nothing
 * intended for use during shutdown/destroy sequencing,
 * thread cannot be restarted without EventThreadDestroy being called first
 */
void	
EventThreadStop( 
	IN EVENT_THREAD *pEventThread );

/* Destroy must be called at passive level. */
void	
EventThreadDestroy( 
	IN EVENT_THREAD *pEventThread );

/* releases the thread for execution */
static _inline void	
EventThreadSignal( 
	IN EVENT_THREAD *pEventThread )
{
#ifdef IB_DEBUG
	ASSERT(pEventThread->m_Initialized);
#endif
	EventTrigger( &pEventThread->m_Event );
}

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IEVENT_THREAD_H_ */
