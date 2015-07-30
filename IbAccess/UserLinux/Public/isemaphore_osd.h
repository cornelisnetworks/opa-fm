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

#ifndef _IBA_PUBLIC_ISEMAPHORE_OSD_H_
#define _IBA_PUBLIC_ISEMAPHORE_OSD_H_

#include "iba/public/datatypes.h"
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef sem_t SEMAPHORE;


/****************************************************************************
 * SemaInitState
 * 
 * Description:
 *	Initializes the state of a semaphore.
 * 
 * Inputs:
 *	pSema - Pointer to a semaphore object.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void
SemaInitState( 
	IN SEMAPHORE* const pSema )
{
	/* No-op. */
}


/****************************************************************************
 * SemaInit
 * 
 * Description:
 *	Initializes the semaphore.  This function performs all initialization.
 * 
 * Inputs:
 *	pSema		- Pointer to a semaphore object
 *	MaxCount	- Maximum number of wait operations that the semaphore can 
 *				satisfy before blocking callers.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	FSUCCESS
 *	FERROR		- Initialization failed.
 * 
 ****************************************************************************/
static __inline FSTATUS
SemaInit( 
	IN SEMAPHORE* const	pSema,
	IN const int32		MaxCount )
{
	ASSERT( pSema );
	ASSERT( MaxCount );

	if (-1 == sem_init( pSema, 0, MaxCount ))
		return FERROR;
	else
		return FSUCCESS;
}


/****************************************************************************
 * SemaDestroy
 * 
 * Description:
 *	Destroys the semaphore.
 * 
 * Inputs:
 *	pSema	- Pointer to a semaphore object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void
SemaDestroy(
	IN SEMAPHORE* const pSema )
{
	ASSERT( pSema );
	sem_destroy(pSema);
}


/****************************************************************************
 * SemaAcquire
 * 
 * Description:
 *	Acquires a semaphore.  This call blocks for the specified interval if the
 *	semaphore cannot be acquired immediately.  A wait of SEMA_NO_TIMEOUT
 *	will never timeout.
 * 
 * Inputs:
 *	pSema	- Pointer to a semaphore object
 *	wait_us	- Time, in microseconds, to wait for the semaphore.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	FSUCCESS	- The samphore was successfully acquired.
 *	FTIMEOUT	- The operation timed out.
 * 
 ****************************************************************************/
FSTATUS
SemaAcquire( 
	IN SEMAPHORE* const	pSema, 
	IN const int32		wait_us );

/****************************************************************************
 * SemaRelease
 * 
 * Description:
 *	Releases a semaphore. This will free one blocking waiter, if any.
 * 
 * Inputs:
 *	pSema	- Pointer to a semaphore object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static __inline void
SemaRelease(
	IN SEMAPHORE* const pSema )
{
	sem_post(pSema);
}


#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ISEMAPHORE_OSD_H_ */
