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


#include "isyscallback.h"
#include "ithread.h"
#include "ithreadpool.h"
#include "ispinlock.h"
#include "imemory.h"
#include "idebug.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define SYS_CALLBACK_TAG 	MAKE_MEM_TAG(b,c,s,I)

#define SYS_CALLBACK_DEBUG 0

#define MIN_SYSCALLBACK_THREADS		(15)	// Low water mark
//#define MAX_SYSCALLBACK_THREADS		(10)	// High water mark
// BUGBUG TBD - implement growing thread pool based on demand and queue depths
#define GROW_SYSCALLBACK_COUNT		(1)		// Grow limit

typedef struct _SYS_CALLBACK_OBJ
{
	LOCKED_QUICK_LIST	LowPriQueue;
	LOCKED_QUICK_LIST	HighPriQueue;
	THREAD_POOL	ThreadPool;
	boolean 	Initialized;
	uint32		ThreadCount;

} SYS_CALLBACK_OBJ;

// Global sytem callback object.
static SYS_CALLBACK_OBJ	Me;

void _SysCallbackWrapper (IN void *Context);

#if defined(__cplusplus)
}
#endif

static
void
_SysCallbackCleanup (void)
{
#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackCleanup\n");
#endif

	// Destroy the thread pool. This blocks until all threads have exited.
	ThreadPoolDestroy (&Me.ThreadPool);

	// Cleanup any pending callbacks.
	if( Me.Initialized )
	{
		uint32		Count = 0;
		LIST_ITEM	*pListItem = NULL;

		// TBD if there is something left on our queue, perhaps we should
		// Invoke the user callback:
		//SYS_CALLBACK_ITEM *pItem = (SYS_CALLBACK_ITEM*)QListObj(pListItem);
		//ASSERT (pItem);
		//pItem->Callback (pItem->Key, pItem->Context);

		while ((pListItem = LQListRemoveHead (&Me.HighPriQueue)) != NULL)
		{
			SysCallbackPut( (SYS_CALLBACK_ITEM*)QListObj(pListItem) );
			Count++;
		}
		if (Count)
			DbgOut (" %d High Priority Items Destroyed \n", Count);

		Count = 0;
		while ((pListItem = LQListRemoveHead (&Me.LowPriQueue)) != NULL)
		{
			SysCallbackPut( (SYS_CALLBACK_ITEM*)QListObj(pListItem) );
			Count++;
		}

		if (Count)
			DbgOut (" %d Low Priority Items Destroyed \n", Count);
	}

	LQListDestroy (&Me.LowPriQueue);
	LQListDestroy (&Me.HighPriQueue);

	Me.Initialized = FALSE;
}

//
// Called once from the framework that initializes and loads
// the Public library
//

// Initialize the System callback module.
boolean
SysCallbackInit (void)
{
#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackInit\n");
#endif
	MemoryClear( &Me, sizeof(SYS_CALLBACK_OBJ) );

	LQListInitState (&Me.HighPriQueue);
	LQListInitState (&Me.LowPriQueue);
	ThreadPoolInitState (&Me.ThreadPool);

	Me.ThreadCount = MIN_SYSCALLBACK_THREADS;

	if (!LQListInit (&Me.HighPriQueue))
	{
		DbgOut ("Failed to initialize Syscallback High Pri Queue \n");
		goto fail;
	}

	if (!LQListInit (&Me.LowPriQueue))
	{
		DbgOut ("Failed to initialize Syscallback Low Pri Queue \n");
		goto faillow;
	}

	if( !ThreadPoolCreate (&Me.ThreadPool, Me.ThreadCount, "SysCallb",
						THREAD_PRI_HIGH, _SysCallbackWrapper, NULL) )
	{
		DbgOut ("Failed to initialize Syscallback Thread Pool \n");
		goto failthread;
	}

	Me.Initialized = TRUE;
	return TRUE;

failthread:
	LQListDestroy (&Me.LowPriQueue);
faillow:
	LQListDestroy (&Me.HighPriQueue);
fail:
	return FALSE;
}

void
SysCallbackDestroy (void)
{
#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackDestroy\n");
#endif
	if (Me.Initialized)
		_SysCallbackCleanup();
}


// Internal Function to manage SYS_CALLBACK_ITEMs
static SYS_CALLBACK_ITEM *
alloc_syscallback (void)
{
	SYS_CALLBACK_ITEM *pItem = NULL;

	pItem = (SYS_CALLBACK_ITEM*)MemoryAllocateAndClear (
									sizeof (SYS_CALLBACK_ITEM),
									FALSE, SYS_CALLBACK_TAG);
	if( pItem )
	{
		QListSetObj(&pItem->ListItem, pItem);
	}
	return pItem;
}

static void
free_syscallback (IN SYS_CALLBACK_ITEM *pItem)
{
	ASSERT (pItem);
	MemoryDeallocate (pItem);
	return;
}


SYS_CALLBACK_ITEM*
SysCallbackGet( IN void *Key )
{
	SYS_CALLBACK_ITEM *pItem = NULL;

#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackGet\n");
#endif
	ASSERT( Me.Initialized == TRUE );

	// Simple, return the next available one from the free pool
	pItem = alloc_syscallback ();

	if (!pItem)
		return NULL;

	pItem->Key = Key;

#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackGet returns: %p Key %p\n", pItem, Key);
#endif
	return pItem;
}

void
SysCallbackPut( IN SYS_CALLBACK_ITEM *pItem )
{
#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackPut: %p\n", pItem);
#endif
	ASSERT( pItem );
	ASSERT( Me.Initialized == TRUE );

	// Return it to the free pool
	free_syscallback (pItem);
}

void SysCallbackQueue (IN SYS_CALLBACK_ITEM *pItem,
					   IN SYSTEM_CALLBACK Callback, IN void *Context,
					   IN boolean HighPriority)
{
#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackQueue: %p: callback %p Context %p High=%d\n", pItem, Callback, Context, HighPriority);
#endif
	ASSERT( Me.Initialized == TRUE );

	pItem->Callback = Callback;
	pItem->Context = Context;

	// Queue the SYS_CALLBACK_ITEM at the tail of a queue (ie HIGH or LOW)
	// based on the Priority of this request 
	if (HighPriority)
	{
		LQListInsertTail (&Me.HighPriQueue, &pItem->ListItem);
	} else {
		LQListInsertTail (&Me.LowPriQueue, &pItem->ListItem);
	}

	// Signal one of the threads from the THREAD_POOL items
	ThreadPoolSignal (&Me.ThreadPool);
}


// This is our SysCallback Thread function.
// It looks at the high priority queue first, if there is some work to do
// pick it up and run it. Else move on to the low priority queue 
void
_SysCallbackWrapper (IN void *Context)
{
	LIST_ITEM 			*pListItem = NULL;

#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackWrapper: %p\n", Context);
#endif
	do
	{
#if SYS_CALLBACK_DEBUG
		DbgOut("SysCallbackWrapper: Check High Queue\n");
#endif
		// Process items from the High Priority Queue first.
		pListItem = LQListRemoveHead (&Me.HighPriQueue);

		// Check to see if there was anything on the high priority queue.
		if (!pListItem)
		{
#if SYS_CALLBACK_DEBUG
			DbgOut("SysCallbackWrapper: Check Low Queue\n");
#endif
			// Nothing on the high priority queue.  Check the low priority 
			// queue.
			pListItem = LQListRemoveHead (&Me.LowPriQueue);
		}

		if( pListItem )
		{
			// Invoke the user callback.
			SYS_CALLBACK_ITEM *pItem = (SYS_CALLBACK_ITEM*)QListObj(pListItem);
			ASSERT (pItem);
			// Invoke the user callback.
#if SYS_CALLBACK_DEBUG
			DbgOut("SysCallbackWrapper: process Item %p Key %p\n", pItem, pItem->Key);
#endif
			pItem->Callback (pItem->Key, pItem->Context);
		}
	} while( pListItem );
#if SYS_CALLBACK_DEBUG
	DbgOut("SysCallbackWrapper: Done\n");
#endif

	return;
}
