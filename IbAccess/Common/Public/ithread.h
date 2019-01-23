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


#ifndef _IBA_PUBLIC_ITHREAD_H_
#define _IBA_PUBLIC_ITHREAD_H_

#define THREAD_NAME_SIZE 16		/* matches Linux current->comm[] size */

#include "iba/public/ithread_osd.h"
#include "iba/public/ilist.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Callback function invoked after the thread is created. */
typedef void (*THREAD_CALLBACK)( IN void *Context );

/* Thread object definition. */
typedef struct _THREAD
{
	/* Private data structure used by the thread object. */
	/* Users should not access these variables directly. */
	struct _ThreadPriv
	{
		THREAD_OSD			m_Osd;
		THREAD_CALLBACK		m_Callback;
		void				*m_Context;

	} m_Private;

} THREAD;



/* Create a new system thread.  */
void
ThreadInitState( 
	IN THREAD *pThread );

boolean
ThreadCreate( 
	IN THREAD *pThread, 
	IN char* Name,
	IN THREAD_CALLBACK Callback, 
	IN void *Context );

/* Destroy the thread when through using it. 
 * This should not be called by the thread itself to exiting. 
 */
void
ThreadDestroy( 
	IN THREAD *pThread );

/* Suspend the currently executing thread.  */
void
ThreadSuspend( 
	IN uint32 pause_ms );

/* Stall the currently executing thread.  */
void
ThreadStall(
	IN uint32 pause_us );

/* Return the number of processors in the system.  */
uint32
ProcCount( void );

/* Returns current CPU */
uint32
CurrentProc( void );

/* Returns TRUE if the calling thread is the specified thread object. */
boolean
IsCurrentThread(
	IN THREAD *pThread );

/* Sets priority for current thread */
void
ThreadSetPriority(
	IN THREAD_PRI pri);

/*Sets Name for the calling thread */
void ThreadSetName(
	IN char *name);
/* gets name of the calling thread set by ThreadSetName*/
char* ThreadGetName(void);

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ITHREAD_H_  */
