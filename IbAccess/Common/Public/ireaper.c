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


#include "ireaper.h"
#include "ilist.h"
#include "ievent.h"
#include "ispinlock.h"
#include "isyscallback.h"
#include "imemory.h"
#include "idebug.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define REAPER_TAG 	MAKE_MEM_TAG(p,e,r,I)
#define REAPER_THRESHOLD 20	// how many to queue before worth reaping

#define REAPER_DEBUG 0

typedef struct _REAPER_OBJ
{
	SPIN_LOCK			Lock;
	QUICK_LIST			Queue;
	SYS_CALLBACK_ITEM 	*SysCallItemp;
	ATOMIC_UINT			Scheduled;
	boolean 			Initialized;
} REAPER_OBJ;

// Global sytem reaper object.
static REAPER_OBJ	Me;

void ReaperDrain (void);
void ReaperCallback(IN void *Key, IN void *Context);

#if defined(__cplusplus)
}
#endif

// Initialize the reaper
// Called once from the framework that initializes and loads
// the Public library
boolean
ReaperInit (void)
{
#if REAPER_DEBUG
	DbgOut("ReaperInit\n");
#endif
	MemoryClear( &Me, sizeof(REAPER_OBJ) );

	QListInitState (&Me.Queue);
	SpinLockInitState (&Me.Lock);

	if (!QListInit (&Me.Queue))
	{
		DbgOut ("Failed to initialize Reaper Queue\n");
		goto failqueue;
	}

	if (!SpinLockInit (&Me.Lock))
	{
		DbgOut ("Failed to initialize Reaper Lock\n");
		goto faillock;
	}
	Me.SysCallItemp = SysCallbackGet( &Me );
	if (! Me.SysCallItemp)
	{
		DbgOut ("Failed to get Reaper Syscallback item\n");
		goto failsyscall;
	}

	Me.Initialized = TRUE;
	return TRUE;

failsyscall:
	SpinLockDestroy(&Me.Lock);
faillock:
	QListDestroy(&Me.Queue);
failqueue:
	return FALSE;
}

void
ReaperDestroy (void)
{
#if REAPER_DEBUG
	DbgOut("ReaperDestroy\n");
#endif
	if (! Me.Initialized)
		return;

	ReaperDrain();
	Me.Initialized = FALSE;

	SysCallbackPut(Me.SysCallItemp);
	SpinLockDestroy(&Me.Lock);
	QListDestroy(&Me.Queue);
}

void
ReaperDrain (void)
{
	LIST_ITEM	*pListItem = NULL;
#if REAPER_DEBUG
	DbgOut("ReaperDrain\n");
#endif

	SpinLockAcquire(&Me.Lock);
	while ((pListItem = QListRemoveHead (&Me.Queue)) != NULL)
	{
		SpinLockRelease(&Me.Lock);
		((REAPER_CALLBACK)(QListObj(pListItem)))(pListItem);
		SpinLockAcquire(&Me.Lock);
	}
	AtomicWrite(&Me.Scheduled, 0);
	SpinLockRelease(&Me.Lock);
}

void
ReaperQueue(
	IN LIST_ITEM *pItem,
	IN REAPER_CALLBACK Callback
	)
{
	uint32 count;

	QListSetObj(pItem, (void*)Callback);
	SpinLockAcquire(&Me.Lock);
	QListInsertTail(&Me.Queue, pItem);
	count = QListCount(&Me.Queue);
#if REAPER_DEBUG
	DbgOut("ReaperQueue: count=%u\n", count);
#endif
	SpinLockRelease(&Me.Lock);
	if (count >= REAPER_THRESHOLD && AtomicExchange(&Me.Scheduled, 1) == 0)
		SysCallbackQueue(Me.SysCallItemp, ReaperCallback, &Me, FALSE);
}


void ReaperCallback(IN void *Key, IN void *Context)
{
	ASSERT(Key == (void*)&Me);
	ASSERT(Context == (void*)&Me);

	ReaperDrain();
}
