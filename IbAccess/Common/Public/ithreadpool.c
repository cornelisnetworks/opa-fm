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
#include "ithreadpool.h"
#include "ispinlock.h"
#include "imemory.h"
#include "idebug.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define THREAD_POOL_DEBUG 0

void 
ThreadPoolCleanup( 
	IN THREAD_POOL *pThreadPool );

#if defined(__cplusplus)
}
#endif

//
// ThreadPool Implementation
//
static void 
ThreadPoolRoutine(
	IN void *Context )
{
	THREAD_POOL *pThreadPool = (THREAD_POOL*)Context;

	ThreadSetPriority(pThreadPool->m_Priority);

	// Continue looping until signalled to end.
	while( TRUE )
	{
		// Wait for the specified event to occur.
		// IbAccess created kernel threads have signals blocked
		// use interruptible wait so we don't contribute to loadavg
		EventWaitOnInterruptible( &pThreadPool->m_Event, EVENT_NO_TIMEOUT );

		// See if we've been signalled to end execution.
		if( pThreadPool->m_Exit )
		{
			// Decrement the running count to notify the destroying thread
			// that the event was received and processed.
			AtomicDecrementVoid( &pThreadPool->m_RunningCount );
			EventTrigger( &pThreadPool->m_DestroyEvent );
			break;
		}

#if THREAD_POOL_DEBUG
		DbgOut("ThreadPool: calling callback: %p(%p)\n", pThreadPool->m_Callback, pThreadPool->m_Context);
#endif
		// The event has been signalled.  Invoke the callback.
		(*pThreadPool->m_Callback)( pThreadPool->m_Context );
	}
}


void 
ThreadPoolInitState( 
	IN THREAD_POOL *pThreadPool )
{
#if THREAD_POOL_DEBUG
	DbgOut("ThreadPoolInitState: %p\n", pThreadPool);
#endif
	MemoryClear( pThreadPool, sizeof( THREAD_POOL ) );
	EventInitState( &pThreadPool->m_Event );
	EventInitState( &pThreadPool->m_DestroyEvent );
	ListInitState( &pThreadPool->m_List );
}


//
// Starts the thread pool with the requested number of threads.
// If Count is set to zero, creates one thread per processor in the system.
//
boolean 
ThreadPoolCreate( 
	IN THREAD_POOL *pThreadPool, 
	IN uint32 Count, 
    IN char* Name,
	IN THREAD_PRI Priority,
	IN THREAD_CALLBACK Callback,
	IN void *Context )
{
	THREAD	*pThread;
	uint32	i;

#if THREAD_POOL_DEBUG
	DbgOut("ThreadPoolCreate: %p Count %d Name %s Callback %p Context %p\n", pThreadPool, Count, Name, Callback, Context);
#endif
	ASSERT( Callback );
	
	ThreadPoolInitState( pThreadPool );

	if( !Count )
		Count = ProcCount();

	if( !ListInit( &pThreadPool->m_List, Count) )
		return FALSE;

	// Initialize the event that the threads wait on.
	if( !EventInit( &pThreadPool->m_Event ) )
	{
		ListDestroy( &pThreadPool->m_List);
		return FALSE;
	}

	// Initialize the event used to destroy the threadpool.
	if( !EventInit( &pThreadPool->m_DestroyEvent ) )
	{
		ListDestroy( &pThreadPool->m_List );
		EventDestroy( &pThreadPool->m_Event );
		return FALSE;
	}

	pThreadPool->m_Callback = Callback;
	pThreadPool->m_Context = Context;
	pThreadPool->m_Priority = Priority;

	for( i = 0; i < Count; i++ )
	{
		char tname[THREAD_NAME_SIZE];

		// -6 allows for '\0', [] and 3 digits of count
		snprintf(tname, THREAD_NAME_SIZE, "%.*s[%d]",
			THREAD_NAME_SIZE-6, Name, i+1);

		// Create a new thread.
		if( !(pThread = (THREAD*)MemoryAllocate( sizeof( THREAD ), FALSE, 0)) )
		{
			ThreadPoolCleanup( pThreadPool );
			return FALSE;
		}
		ThreadInitState(pThread);

		// Add it to the list.  This is guaranteed to work.
		ListInsertHead( &pThreadPool->m_List, pThread );

		// Start the thread.
		if( !ThreadCreate( pThread, tname, ThreadPoolRoutine, pThreadPool ) )
		{
			ThreadPoolCleanup( pThreadPool );
			return FALSE;
		}

		// Increment the running count to insure that a destroying thread
		// will signal all the threads.
		AtomicIncrement(&pThreadPool->m_RunningCount);
	}
	pThreadPool->m_Initialized = TRUE;
	return TRUE;
}


//
// Destroys all threads that have been created.
//
void 
ThreadPoolDestroy( 
	IN THREAD_POOL *pThreadPool )
{
#if THREAD_POOL_DEBUG
	DbgOut("ThreadPoolDestroy: %p\n");
#endif
	if( !pThreadPool->m_Initialized )
		return;

	ThreadPoolCleanup( pThreadPool );
}


void 
ThreadPoolCleanup( 
	IN THREAD_POOL *pThreadPool )
{
	THREAD		*pThread;

#if THREAD_POOL_DEBUG
	DbgOut("ThreadPoolCleanup: %p\n");
#endif
	// Indicate to all threads that they need to exit.
	pThreadPool->m_Exit = TRUE;

	// Signal the threads until they have all exited.  Signalling
	// once for each thread is not guaranteed to work since two events 
	// could release only a single thread, depending on the rate at which
	// the events are set and how the thread scheduler processes notifications.
	while( AtomicRead(&pThreadPool->m_RunningCount) )
	{
		EventTrigger( &pThreadPool->m_Event );
		// Wait for the destroy event to occur, indicating that the thread
		// has exited.
		EventWaitOn( &pThreadPool->m_DestroyEvent, EVENT_NO_TIMEOUT );
	}

	// Stop each thread one at a time.  Note that this cannot be done in the 
	// above for loop because signal will wake up an unknown thread.
	while( NULL != (pThread = (THREAD*)ListRemoveHead( &pThreadPool->m_List )) )
	{
		ThreadDestroy( pThread );
		MemoryDeallocate( pThread );
	}

	EventDestroy( &pThreadPool->m_DestroyEvent );
	EventDestroy( &pThreadPool->m_Event );
	ListDestroy( &pThreadPool->m_List );
	pThreadPool->m_Initialized = FALSE;
}


//
// Releases one thread for execution.
//
void 
ThreadPoolSignal(
	IN THREAD_POOL *pThreadPool )
{
#if THREAD_POOL_DEBUG
	DbgOut("ThreadPoolSignal: %p\n");
#endif

	if (pThreadPool->m_Initialized) {
		EventTrigger( &pThreadPool->m_Event );
	}
}

#if !defined(__VXWORKS__)
//
// Releases all threads for execution.
//
void 
ThreadPoolBroadcast(
	IN THREAD_POOL *pThreadPool )
{
#if THREAD_POOL_DEBUG
	DbgOut("ThreadPoolBroadcast: %p\n");
#endif

	if (pThreadPool->m_Initialized) {
		EventBroadcast( &pThreadPool->m_Event );
	}
}
#endif
