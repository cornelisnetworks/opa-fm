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

#include "icmdthread.h"
#include "imemory.h"

#define CMD_THREAD_TAG		MAKE_MEM_TAG( i, c, t, h )

//
// CmdThread Implementation
//

static boolean CmdThreadDeQueue(CMD_THREAD* pCmdThread, void **pCmd)
{
	LIST_ITEM *pItem;

	SpinLockAcquire(&pCmdThread->m_QueueLock);
	pItem = QListRemoveHead(&pCmdThread->m_Queue);
	SpinLockRelease(&pCmdThread->m_QueueLock);
	if (pItem == NULL)
		return FALSE;
	*pCmd = (void*)QListObj(pItem);
	MemoryDeallocate(pItem);
	return TRUE;
}

// add a command to the end of the queue
boolean CmdThreadQueue(CMD_THREAD* pCmdThread, void *Cmd)
{
	LIST_ITEM *pItem;

	ASSERT( pCmdThread->m_Initialized );
	pItem = (LIST_ITEM*)MemoryAllocate2(sizeof(LIST_ITEM), IBA_MEM_FLAG_SHORT_DURATION, CMD_THREAD_TAG);
	if (pItem == NULL)
		return FALSE;
	QListSetObj(pItem, Cmd);
	SpinLockAcquire(&pCmdThread->m_QueueLock);
	QListInsertTail(&pCmdThread->m_Queue, pItem);
	SpinLockRelease(&pCmdThread->m_QueueLock);
	EventTrigger(&pCmdThread->m_Event);
	return TRUE;
}

// internal Thread for cmd queue processing
static void 
CmdThreadRoutine(
	IN void *Context )
{
	CMD_THREAD *pCmdThread = (CMD_THREAD*)Context;

	ThreadSetPriority(pCmdThread->m_Pri);

	// Continue looping until signalled to end.
	while( TRUE )
	{
		void* cmd;

		// Wait for the specified event to occur.
		// IbAccess created kernel threads have signals blocked
		// use interruptible wait so we don't contribute to loadavg
		EventWaitOnInterruptible( &pCmdThread->m_Event, EVENT_NO_TIMEOUT );

		// See if we've been signalled to end execution.
		if( pCmdThread->m_Exit )
		{
			break;
		}

		while (CmdThreadDeQueue(pCmdThread, &cmd)) {
			// A command is on the queue.  Invoke the callback.
			(*pCmdThread->m_Callback)( pCmdThread->m_Context, cmd );
		}
	}
}


void 
CmdThreadInitState( 
	IN CMD_THREAD *pCmdThread )
{
	MemoryClear( pCmdThread, sizeof( CMD_THREAD ) );
	EventInitState( &pCmdThread->m_Event );
	QListInitState( &pCmdThread->m_Queue );
	SpinLockInitState( &pCmdThread->m_QueueLock );
	ThreadInitState( &pCmdThread->m_Thread );
}


//
// Starts the thread
//
// Callback is called for each command processed, it should free the command
// as needed
// FreeCmd is only called on cleanup to free any commands remaining on the queue
// if cmd's do not need to be freed (eg. are not pointers), this can be NULL
boolean 
CmdThreadCreate( 
	IN CMD_THREAD *pCmdThread, 
	IN THREAD_PRI Pri,
    IN char* Name,
	IN CMD_THREAD_CALLBACK Callback,
	IN CMD_THREAD_CALLBACK FreeCmd,
	IN void *Context )
{
	ASSERT( Callback );
	ASSERT( ! pCmdThread->m_Initialized );
	
	CmdThreadInitState( pCmdThread );

	if( !SpinLockInit( &pCmdThread->m_QueueLock ) )
		goto fail_lock;
	if( !QListInit( &pCmdThread->m_Queue ) )
		goto fail_list;

	// Initialize the event that the threads wait on.
	if( !EventInit( &pCmdThread->m_Event ) )
		goto fail_event;

	pCmdThread->m_Callback = Callback;
	pCmdThread->m_FreeCmd = FreeCmd;
	pCmdThread->m_Context = Context;
	pCmdThread->m_Pri = Pri;

	// Start the thread.
	if( !ThreadCreate( &pCmdThread->m_Thread, Name, CmdThreadRoutine, pCmdThread ) )
		goto fail_thread;

	pCmdThread->m_Initialized = TRUE;
	return TRUE;

fail_thread:
	EventDestroy(&pCmdThread->m_Event);
fail_event:
	SpinLockDestroy(&pCmdThread->m_QueueLock);
fail_list:
	QListDestroy(&pCmdThread->m_Queue);
fail_lock:
	return FALSE;
}


// Destroys the thread, hence stopping it
void 
CmdThreadStop( 
	IN CMD_THREAD *pCmdThread )
{
	if( !pCmdThread->m_Initialized )
		return;

	// Indicate to thread that it needs to exit.
	pCmdThread->m_Exit = TRUE;

	EventTrigger( &pCmdThread->m_Event );

	// Stop the thread.  Note: it is safe to call ThreadDestroy multiple times
	ThreadDestroy( &pCmdThread->m_Thread );
}

//
// Destroys the threads and calls FreeCmd for each item on queue
//
void 
CmdThreadDestroy( 
	IN CMD_THREAD *pCmdThread )
{
	void *cmd;

	if( !pCmdThread->m_Initialized )
		return;

	CmdThreadStop(pCmdThread);

	// Free any remaining cmds on the Queue
	while (CmdThreadDeQueue(pCmdThread, &cmd)) {
		if (pCmdThread->m_FreeCmd != NULL) {
			(*pCmdThread->m_FreeCmd)( pCmdThread->m_Context, cmd );
		}
	}

	EventDestroy( &pCmdThread->m_Event );
	QListDestroy( &pCmdThread->m_Queue );
	SpinLockDestroy( &pCmdThread->m_QueueLock );
	pCmdThread->m_Initialized = FALSE;
}
