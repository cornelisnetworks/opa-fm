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

#ifndef _IBA_PUBLIC_IMUTEX_OSD_H_
#define _IBA_PUBLIC_IMUTEX_OSD_H_

#include "iba/public/datatypes.h"
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef pthread_mutex_t		MUTEX;

/****************************************************************************
 * MutexInitState
 * 
 * Description:
 *	Initializes the state of a mutex.
 * 
 * Inputs:
 *	pMutex - Pointer to a mutex object.
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static _inline void
MutexInitState( 
	IN MUTEX* const pMutex )
{
	/* **************************************************** */
	/* ************ DO NOTHING FOR NOW !!! *************** */
	/* **************************************************** */
}


/****************************************************************************
 * MutexInit
 * 
 * Description:
 *	Initializes the mutex.  This function performs all initialization.
 * 
 * Inputs:
 *	pMutex		- Pointer to a mutex object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	FSUCCESS
 *	FERROR		- Initialization failed.
 * 
 ****************************************************************************/
static _inline FSTATUS
MutexInit( 
	IN MUTEX* const	pMutex)
{
	DEBUG_ASSERT(pMutex);

	/* Initialize with pthread_mutexattr_t = NULL */
  	pthread_mutex_init(pMutex,NULL);
  	return FSUCCESS;
}


/****************************************************************************
 * MutexDestroy
 * 
 * Description:
 *	Destroys the mutex.
 * 
 * Inputs:
 *	pMutex	- Pointer to a mutex object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static _inline void
MutexDestroy(
	IN MUTEX* const pMutex )
{
	DEBUG_ASSERT(pMutex);
  	pthread_mutex_destroy(pMutex);
}


/****************************************************************************
 * MutexAcquire
 * 
 * Description:
 *	Acquires a mutex.  This call blocks if the mutex cannot be
 *	acquired immediately.
 * 
 * Inputs:
 *	pMutex	- Pointer to a mutex object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static _inline void
MutexAcquire( 
	IN MUTEX* const	pMutex)
{
	ASSERT(pMutex);
  	pthread_mutex_lock(pMutex);
}


/****************************************************************************
 * MutexRelease
 * 
 * Description:
 *	Releases a mutex. This will free one blocking waiter, if any.
 * 
 * Inputs:
 *	pMutex	- Pointer to a mutex object
 * 
 * Outputs:
 *	None
 * 
 * Returns:
 *	None
 * 
 ****************************************************************************/
static _inline void
MutexRelease(
	IN MUTEX* const pMutex )
{
	ASSERT(pMutex);
  	pthread_mutex_unlock(pMutex);
}

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IMUTEX_OSD_H_ */
