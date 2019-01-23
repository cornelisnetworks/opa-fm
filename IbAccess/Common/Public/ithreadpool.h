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


#ifndef _IBA_PUBLIC_ITHREAD_POOL_H_
#define _IBA_PUBLIC_ITHREAD_POOL_H_

#include "iba/public/ilist.h"
#include "iba/public/ithread.h"
#include "iba/public/ievent.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Thread Pool object.  Threads will wait on an event, then invoke a callback.
 */
typedef struct _THREAD_POOL
{
	/* Routine called by thread when the specified event occurs. */
	THREAD_CALLBACK	m_Callback;
	void			*m_Context;

	DLIST			m_List;
	EVENT			m_Event;
	EVENT			m_DestroyEvent;
	boolean			m_Exit;

	boolean			m_Initialized;

	ATOMIC_UINT		m_RunningCount;
	THREAD_PRI		m_Priority;

} THREAD_POOL;


void	
ThreadPoolInitState( 
	IN THREAD_POOL *pThreadPool );

boolean 
ThreadPoolCreate( 
	IN THREAD_POOL *pThreadPool, 
	IN uint32 ThreadCount, 
	IN char* Name,
	IN THREAD_PRI Priority,
	IN THREAD_CALLBACK Callback, 
	IN void *Context );

/* Destroy must be called in a premptable context. */
void	
ThreadPoolDestroy( 
	IN THREAD_POOL *pThreadPool );

void	
ThreadPoolSignal( 
	IN THREAD_POOL *pThreadPool );

void	
ThreadPoolBroadcast( 
	IN THREAD_POOL *pThreadPool );

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ITHREAD_POOL_H_ */
