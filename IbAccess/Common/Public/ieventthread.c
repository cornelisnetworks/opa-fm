/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/

#include "datatypes.h"
#include "ieventthread.h"
#include "imemory.h"

//
// EventThread Implementation
//
static void 
EventThreadRoutine(
	IN void *Context )
{
	EVENT_THREAD *pEventThread = (EVENT_THREAD*)Context;

	ThreadSetPriority(pEventThread->m_Priority);

	// Continue looping until signalled to end.
	while( TRUE )
	{
		// Wait for the specified event to occur.
		// IbAccess created kernel threads have signals blocked
		// use interruptible wait so we don't contribute to loadavg
		EventWaitOnInterruptible( &pEventThread->m_Event, EVENT_NO_TIMEOUT );

		// See if we've been signalled to end execution.
		if( pEventThread->m_Exit )
		{
			break;
		}

		// The event has been signalled.  Invoke the callback.
		(*pEventThread->m_Callback)( pEventThread->m_Context );
	}
}


void 
EventThreadInitState( 
	IN EVENT_THREAD *pEventThread )
{
	MemoryClear( pEventThread, sizeof( EVENT_THREAD ) );
	EventInitState( &pEventThread->m_Event );
	ThreadInitState( &pEventThread->m_Thread );
}


//
// Starts the event thread
//
boolean 
EventThreadCreate( 
	IN EVENT_THREAD *pEventThread, 
    IN char* Name,
	IN THREAD_PRI Priority,
	IN THREAD_CALLBACK Callback,
	IN void *Context )
{
	ASSERT( Callback );
	
	EventThreadInitState( pEventThread );

	// Initialize the event that the thread waits on.
	if( !EventInit( &pEventThread->m_Event ) )
	{
		return FALSE;
	}

	pEventThread->m_Priority = Priority;
	pEventThread->m_Callback = Callback;
	pEventThread->m_Context = Context;

	// Start the thread.
	if( !ThreadCreate( &pEventThread->m_Thread, Name, EventThreadRoutine, pEventThread ) )
	{
		EventDestroy(&pEventThread->m_Event);
		return FALSE;
	}

	pEventThread->m_Initialized = TRUE;
	return TRUE;
}

// Destroys the thread, hence stopping it
void 
EventThreadStop( 
	IN EVENT_THREAD *pEventThread )
{
	if( !pEventThread->m_Initialized )
		return;

	// Indicate to thread that it needs to exit.
	pEventThread->m_Exit = TRUE;

	EventTrigger( &pEventThread->m_Event );

	// Stop the thread.  Note: it is safe to call ThreadDestroy multiple times
	ThreadDestroy( &pEventThread->m_Thread );
}

//
// Destroys the thread and event
//
void 
EventThreadDestroy( 
	IN EVENT_THREAD *pEventThread )
{
	if( !pEventThread->m_Initialized )
		return;

	EventThreadStop(pEventThread);

	EventDestroy( &pEventThread->m_Event );
	pEventThread->m_Initialized = FALSE;
}
