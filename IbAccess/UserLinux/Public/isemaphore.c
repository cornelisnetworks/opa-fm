/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "iba/public/isemaphore.h"
#include "iba/public/ithread.h"
#include "semaphore.h"
#include "errno.h"

///////////////////////////////////////////////////////////////////////////////
// SemaAcquire
// 
// Description:
//	Acquires a semaphore.  This call blocks for the specified interval if the
//	semaphore cannot be acquired immediately.  A wait of SEMA_NO_TIMEOUT
//	will never timeout.
// 
// Inputs:
//	pSema	- Pointer to a semaphore object
//	wait_us	- Time, in microseconds, to wait for the semaphore.
// 
// Outputs:
//	None
// 
// Returns:
//	FSUCCESS	- The samphore was successfully acquired.
//	FTIMEOUT	- The operation timed out.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
SemaAcquire( 
	IN SEMAPHORE* const	pSema, 
	IN const int32		wait_us )
{
	FSTATUS	Status;

	ASSERT( pSema );

	if( wait_us == SEMA_NO_TIMEOUT )
	{
		if ( 0 == sem_wait ( pSema ))
			Status = FSUCCESS;
		else
			Status = FERROR;
	} else if (wait_us == 0) {
		if (0 == sem_trywait(pSema))
			Status = FSUCCESS;
		else if (errno == EAGAIN)
			Status = FTIMEOUT;
		else
			Status = FERROR;
	} else {
		// user space semaphores don't have a timeout on wait
		uint32	ms = (wait_us+999)/1000;
		Status = FTIMEOUT;
		do
		{
			if (0 == sem_trywait( pSema ))
			{
				Status = FSUCCESS;
				break;
			} else if (errno != EAGAIN) {
				Status = FERROR;
				break;
			}
			ThreadSuspend(1);
		} while (ms--);
	}
	return Status;
}
