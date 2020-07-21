/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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


#include "datatypes.h"
#include "ievent.h"
#include "idebug.h"
//-----------------------------------------------------------
// Internal functions needed for events
//-----------------------------------------------------------


//-----------------------------------------------------------
// Declare that we intended to create an event
//-----------------------------------------------------------
void 
EventInitState (IN EVENT *pEvent)
{
   	pEvent->ev_state = Constructed;
}

//-----------------------------------------------------------
// Initialize an event 
//-----------------------------------------------------------
boolean
EventInit( IN EVENT *pEvent )
{
	pthread_cond_init(&pEvent->ev_condvar,NULL);
  	pthread_mutex_init( &pEvent->ev_mutex, NULL );
  	pEvent->ev_signaled = FALSE;
  	pEvent->ev_state = Started;	
  	return TRUE;
}

//-----------------------------------------------------------
// Destroy an event
//-----------------------------------------------------------
void
EventDestroy( IN EVENT *pEvent )
{
  	// Destroy only if the event was constructed 
  	if (pEvent->ev_state == Started)
  	{
   		pthread_cond_destroy( &pEvent->ev_condvar );
    	pthread_mutex_destroy( &pEvent->ev_mutex );
		pEvent->ev_signaled = FALSE;
		pEvent->ev_state = Destroyed;
  	}
}

//-----------------------------------------------------------
// Trigger or Signal an event
//-----------------------------------------------------------
void
EventTrigger( IN EVENT *pEvent )
{

	// Make sure that the event was Started
	ASSERT (pEvent->ev_state == Started);
	
 	pthread_mutex_lock( &pEvent->ev_mutex );
	
  	pEvent->ev_signaled = TRUE;
	pthread_cond_signal(&pEvent->ev_condvar );	

  	pthread_mutex_unlock( &pEvent->ev_mutex );
}

//-----------------------------------------------------------
// Wakes all waiters intest of just one.
//-----------------------------------------------------------
void
EventBroadcast( IN EVENT *pEvent )
{

	// Make sure that the event was Started
	ASSERT (pEvent->ev_state == Started);
	
 	pthread_mutex_lock( &pEvent->ev_mutex );
	
  	pEvent->ev_signaled = TRUE;
	pthread_cond_broadcast(&pEvent->ev_condvar );	

  	pthread_mutex_unlock( &pEvent->ev_mutex );
}

//-----------------------------------------------------------
// Clear an event, as though it was never triggered
//-----------------------------------------------------------
void
EventClear( IN EVENT *pEvent )
{
	
  	// Make sure that the event was Started 
  	ASSERT (pEvent->ev_state == Started);

  	pthread_mutex_lock( &pEvent->ev_mutex );

	pEvent->ev_signaled = FALSE;

  	pthread_mutex_unlock( &pEvent->ev_mutex );
}


//-----------------------------------------------------------
// Wait for an event - this event is NOT interruptible
//-----------------------------------------------------------
FSTATUS 
EventWaitOn( IN EVENT *pEvent, IN int32 wait_micro)
{
  	// Make sure that the event was started 
  	ASSERT (pEvent->ev_state == Started);

	//
	// Don't wait if the it has been signaled already!
	//
  	pthread_mutex_lock( &pEvent->ev_mutex );
    if (pEvent->ev_signaled == TRUE)
    {
		// autoclear
		pEvent->ev_signaled = FALSE;

		pthread_mutex_unlock( &pEvent->ev_mutex);
        return FSUCCESS;
    }

  	if (wait_micro == EVENT_NO_TIMEOUT)
	{
		wait_micro = 0;
		// Wait for condition variable ev_condvar to be signaled or broadcast.
		// Also need to implement interruptible/Non interruptible waits!!
		pthread_cond_wait(&pEvent->ev_condvar,&pEvent->ev_mutex);
	
		pEvent->ev_signaled = FALSE;
  		pthread_mutex_unlock( &pEvent->ev_mutex );
		return FSUCCESS;		
  	}
	else
	{
		FSTATUS status = FERROR;
		int retval;

		// Get the current time
		if (gettimeofday(&pEvent->ev_curtime, NULL) == 0)
		{
			(pEvent->ev_timeout).tv_sec = (pEvent->ev_curtime).tv_sec + (wait_micro/1000000);
			(pEvent->ev_timeout).tv_nsec = ((pEvent->ev_curtime).tv_usec + (wait_micro % 1000000)) * 1000;
			// handle carry from nsec to sec
			if ((pEvent->ev_timeout).tv_nsec > 1000000000) {
				(pEvent->ev_timeout).tv_sec++;
				(pEvent->ev_timeout).tv_nsec -= 1000000000;
			}
		
			retval = pthread_cond_timedwait(&pEvent->ev_condvar, &pEvent->ev_mutex, &pEvent->ev_timeout);
			if (retval == ETIMEDOUT)
			{
				status = FTIMEOUT;
			}
			else
			{
				// We got signalled
				status = FSUCCESS;
			}
		}
		pEvent->ev_signaled = FALSE;
  		pthread_mutex_unlock( &pEvent->ev_mutex );
		return status;		
	}
}

//-----------------------------------------------------------
// Wait for an event - this event is interruptible
//-----------------------------------------------------------
FSTATUS 
EventWaitOnInterruptible( IN EVENT *pEvent, IN int32 wait_micro)
{
	return EventWaitOn(pEvent, wait_micro);
}
