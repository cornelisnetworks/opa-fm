/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/


#include <stdio.h>
#include <unistd.h>
#include "datatypes.h"
#include "ithread.h"
#include "idebug.h"

//----------------------------------------------------------------
// Internal function to initialize the thread data structure. This
// function is typically called before a call to kernel_thread().
//----------------------------------------------------------------
static void 
__InitializeThreadData(IN THREAD *pThread, IN THREAD_CALLBACK Callback, 
                       IN void *Context)
{
  	pThread->m_Private.m_Osd.tr_flags 	= 0;
  	pThread->m_Private.m_Callback 		= Callback;
  	pThread->m_Private.m_Context 		= Context;
  	EventInit( &pThread->m_Private.m_Osd.tr_RunningEvent );
}

//
//----------------------------------------------------------------
// Internal function to run a new user mode thread.
// This function is always run as a result of creation a new user mode thread.
// Its main job is to synchronize the creation and running of the new thread.
//----------------------------------------------------------------
// 


static int
__UserThreadStartup(void * arg)
{
	THREAD *pThread = (THREAD *) arg;

	//
	// Trigger the thread running event.
	// The creating thread waits on this event.
	//
#if DEBUG_THREAD
	DbgOut ("__UserThreadStartup( %p )[\n",pThread );
#endif
	setpgrp();

	EventTrigger(&pThread->m_Private.m_Osd.tr_RunningEvent);

	if (pThread->m_Private.m_Callback)
	{
		pThread->m_Private.m_Callback(pThread->m_Private.m_Context);
	}

#if DEBUG_THREAD
	DbgOut("__UserThreadStartup( %p )]\n",pThread );
#endif

	return (0) ;
		
}
//----------------------------------------------------------------
//
// Indicate that there was an intent to use a thread 
//----------------------------------------------------------------
//
void 
ThreadInitState ( IN THREAD *pThread )
{
  	pThread->m_Private.m_Osd.tr_state = Constructed;
	EventInitState( &pThread->m_Private.m_Osd.tr_RunningEvent);
}

// 
//----------------------------------------------------------------
// Create a new thread. This thread will execute supplied "Callback"
// with the user supplied "Context"
//----------------------------------------------------------------
//
boolean
ThreadCreate( IN THREAD *pThread, IN char* Name, IN THREAD_CALLBACK Callback, IN void *Context )
{
  	int ret;

	if (pThread == NULL)
	{
		return FALSE;
	}

	//
  	// Initialize the thread structure 
	//
  	__InitializeThreadData(pThread, Callback, Context);

#if DEBUG_THREAD
	DbgOut("__ThreadCreate( %p )[\n",pThread );
#endif

	ret = pthread_create(&pThread->m_Private.m_Osd.tr_id,
				 NULL,
				(void *)__UserThreadStartup,
				(void *)pThread);

  	if (ret != 0)	// pthread_create returns a "0" for success
  	{
			return FALSE;
  	}
	
	// TBD - any way to set name of thread?

	//
  	// Wait for the "child" thread to indicate that it is up and running
	// It assumes that the spawned thread will signal Running Event when it starts up
	//
  	EventWaitOn( &pThread->m_Private.m_Osd.tr_RunningEvent, EVENT_NO_TIMEOUT);
	
  	pThread->m_Private.m_Osd.tr_state = Started;
	
#if DEBUG_THREAD
	DbgOut("__ThreadCreate( %p )]\n",pThread );
#endif

	return TRUE;
}

//
//----------------------------------------------------------------
// Destroy the thread.
// If this function is called after the callback function terminated causing
// the thread to terminate, this function returns immediately.
// If this function is called before the callback function returns, the
// caller will wait for the kill event forever.
//----------------------------------------------------------------
//
void
ThreadDestroy( IN THREAD *pThread )
{
  	if ( pThread->m_Private.m_Osd.tr_state != Started )
  	{
		// thread was never Started 
    		return;
  	}
	// If more than one threads are wating on a thread
	// the outcome may not be deterministic
	// Might want to try waiting on the thread kill event

#if DEBUG_THREAD
	DbgOut ("__ThreadDestroy( %p )[\n",pThread );
#endif
	pthread_join( pThread->m_Private.m_Osd.tr_id, NULL );
  	EventDestroy( &pThread->m_Private.m_Osd.tr_RunningEvent );

	// clear all the thread state data
  	pThread->m_Private.m_Osd.tr_state 	= Destroyed;
  	pThread->m_Private.m_Osd.tr_id 		= 0;
  	pThread->m_Private.m_Callback 		= NULL;
  	pThread->m_Private.m_Context 		= NULL;
#if DEBUG_THREAD
	DbgOut ("__ThreadDestroy( %p )]\n",pThread );
#endif
}

// 
//----------------------------------------------------------------
// Suspend a thread execution for "pause_ms" milliseconds
//----------------------------------------------------------------
//
void
ThreadSuspend( IN uint32 pause_ms )
{
	// Convert to micro seconds
	usleep(pause_ms * 1000);
	
}

//
//----------------------------------------------------------------
// Delay execution of thread for some number of microseconds.
// This uses Linuxs' udelay() function. Remember, suggested maximum
// value of microseconds is 1000, ie, 1 milliseconds.
//----------------------------------------------------------------
//
void
ThreadStall( IN uint32 pause_us )
{
	usleep(pause_us);
}


//----------------------------------------------------------------
// Number of processors on this system 
//----------------------------------------------------------------
uint32
ProcCount(void)
{
	//********************************
	//*********** FOR NOW!! **********
	//********************************
  return 0;
}

uint32
CurrentProc(void)
{
	return 0; 	// for now
}

//----------------------------------------------------------------
// set priority for current thread
//----------------------------------------------------------------
void ThreadSetPriority(IN THREAD_PRI pri)
{
	if (nice(pri) < 0) {
		MsgOut("Failed to set thread priority.\n");
	}
}
