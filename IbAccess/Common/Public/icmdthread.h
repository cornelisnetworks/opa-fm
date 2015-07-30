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


#ifndef _IBA_PUBLIC_ICMD_THREAD_H_
#define _IBA_PUBLIC_ICMD_THREAD_H_

#include "iba/public/ilist.h"
#include "iba/public/ithread.h"
#include "iba/public/ievent.h"
#include "iba/public/ispinlock.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*CMD_THREAD_CALLBACK)( IN void *Context, IN void* Cmd );

/*
 * Cmd Thread object.
 * Thread will wait until a command is placed on the queue, then invoke a
 * command processing callback
 * useful to perform command or packet processing in a thread context instead
 * of a callback/ISR context
 */
typedef struct _CMD_THREAD
{
	/* Routine called by thread when cmd found on queue */
	CMD_THREAD_CALLBACK	m_Callback;
	CMD_THREAD_CALLBACK	m_FreeCmd;	/* called on cleanup to free commands */
	void			*m_Context;
	THREAD_PRI 		m_Pri;

	QUICK_LIST		m_Queue;
	SPIN_LOCK		m_QueueLock;
	EVENT			m_Event;
	THREAD			m_Thread;

	boolean			m_Initialized;
	boolean			m_Exit;
} CMD_THREAD;

void	
CmdThreadInitState( 
	IN CMD_THREAD *pCmdThread );

boolean 
CmdThreadCreate( 
	IN CMD_THREAD *pCmdThread, 
	IN THREAD_PRI Pri,
	IN char* Name,
	IN CMD_THREAD_CALLBACK Callback, 
	IN CMD_THREAD_CALLBACK FreeCmd, 
	IN void *Context );

/* Stop must be called in a premptable context.
 * Stops thread and waits for it.  After this no more Callbacks will
 * occur, calls to CmdThreadQueue will add to queue and
 * wait for CmdThreadDestroy to clean the queue.
 * intended for use during shutdown/destroy sequencing,
 * thread cannot be restarted without CmdThreadDestroy being called first
 */
void	
CmdThreadStop( 
	IN CMD_THREAD *pCmdThread );

/* Destroy must be called at passive level. */
void	
CmdThreadDestroy( 
	IN CMD_THREAD *pCmdThread );

boolean	
CmdThreadQueue( 
	IN CMD_THREAD *pCmdThread,
	IN void* cmd);

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ICMD_THREAD_H_ */
