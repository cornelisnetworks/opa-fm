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

#include "datatypes.h"
#include "ispinlock.h"		// Common/Public/

//-------------------------------------------------------------
// Initialize a spinlock before its use.
//-------------------------------------------------------------
void
SpinLockInitState( IN SPIN_LOCK *pSpinLock )  
{
	// ****************************************************
	// ************ DO NOTHING FOR NOW !!! ***************
	// ****************************************************
}

//-------------------------------------------------------------
// Initialize a spin lock 
//-------------------------------------------------------------
boolean
SpinLockInit( SPIN_LOCK *pSpinLock )
{
	ASSERT(pSpinLock);

	// Initialize with pthread_mutexattr_t = NULL

  	pthread_mutex_init(&pSpinLock->sp_lock,NULL);
  	return TRUE;
}

//-------------------------------------------------------------
// Destroy a spin lock 
//-------------------------------------------------------------
void
SpinLockDestroy( SPIN_LOCK *pSpinLock )
{
	ASSERT(pSpinLock);
  	pthread_mutex_destroy(&pSpinLock->sp_lock);
}

//-------------------------------------------------------------
// Acquire a spin lock 
//-------------------------------------------------------------
void
SpinLockAcquire( SPIN_LOCK *pSpinLock )
{
	ASSERT(pSpinLock);
  	pthread_mutex_lock(&pSpinLock->sp_lock);
}


//-------------------------------------------------------------
// Acquire a spin lock with less overhead 
//-------------------------------------------------------------
void
SpinLockFastAcquire( SPIN_LOCK *pSpinLock )
{
  SpinLockAcquire( pSpinLock );	// For Now
}


//-------------------------------------------------------------
// Release a spin lock 
//-------------------------------------------------------------
void
SpinLockRelease( SPIN_LOCK *pSpinLock )
{
	ASSERT(pSpinLock);
  	pthread_mutex_unlock(&pSpinLock->sp_lock);
}


//-------------------------------------------------------------
// Release a spin lock with less overhead
//-------------------------------------------------------------
void
SpinLockFastRelease( SPIN_LOCK *pSpinLock )
{
  SpinLockRelease( pSpinLock );	// For Now
}

//-------------------------------------------------------------
// Initialize a spinrwlock before its use.
//-------------------------------------------------------------
void
SpinRwLockInitState( IN SPIN_RW_LOCK *pSpinRwLock )  
{
	// ****************************************************
	// ************ DO NOTHING FOR NOW !!! ***************
	// ****************************************************
}

//-------------------------------------------------------------
// Initialize a spin rw lock 
//-------------------------------------------------------------
boolean
SpinRwLockInit( SPIN_RW_LOCK *pSpinRwLock )
{
	ASSERT(pSpinRwLock);

	// Initialize with pthread_mutexattr_t = NULL

#if 0
  	pthread_rwlock_init(&pSpinRwLock->rw_lock,NULL);
#else
  	pthread_mutex_init(&pSpinRwLock->rw_lock,NULL);
#endif
  	return TRUE;
}

//-------------------------------------------------------------
// Destroy a spin rw lock 
//-------------------------------------------------------------
void
SpinRwLockDestroy( SPIN_RW_LOCK *pSpinRwLock )
{
	ASSERT(pSpinRwLock);
#if 0
  	pthread_rwlock_destroy(&pSpinRwLock->rw_lock);
#else
  	pthread_mutex_destroy(&pSpinRwLock->rw_lock);
#endif
}

//-------------------------------------------------------------
// Acquire a spin rw lock 
//-------------------------------------------------------------
void
SpinRwLockAcquireRead( SPIN_RW_LOCK *pSpinRwLock )
{
	ASSERT(pSpinRwLock);
#if 0
  	pthread_rwlock_rdlock(&pSpinRwLock->rw_lock);
#else
  	pthread_mutex_lock(&pSpinRwLock->rw_lock);
#endif
}
void
SpinRwLockAcquireWrite( SPIN_RW_LOCK *pSpinRwLock )
{
	ASSERT(pSpinRwLock);
#if 0
  	pthread_rwlock_wrlock(&pSpinRwLock->rw_lock);
#else
  	pthread_mutex_lock(&pSpinRwLock->rw_lock);
#endif
}


//-------------------------------------------------------------
// Acquire a spin rw lock with less overhead 
//-------------------------------------------------------------
void
SpinRwLockFastAcquireRead( SPIN_RW_LOCK *pSpinRwLock )
{
  SpinRwLockAcquireRead( pSpinRwLock );	// For Now
}
void
SpinRwLockFastAcquireWrite( SPIN_RW_LOCK *pSpinRwLock )
{
  SpinRwLockAcquireWrite( pSpinRwLock );	// For Now
}


//-------------------------------------------------------------
// Release a spin rw lock 
//-------------------------------------------------------------
void
SpinRwLockReleaseRead( SPIN_RW_LOCK *pSpinRwLock )
{
	ASSERT(pSpinRwLock);
#if 0
  	pthread_rwlock_unlock(&pSpinRwLock->rw_lock);
#else
  	pthread_mutex_unlock(&pSpinRwLock->rw_lock);
#endif
}
void
SpinRwLockReleaseWrite( SPIN_RW_LOCK *pSpinRwLock )
{
	ASSERT(pSpinRwLock);
#if 0
  	pthread_rwlock_unlock(&pSpinRwLock->rw_lock);
#else
  	pthread_mutex_unlock(&pSpinRwLock->rw_lock);
#endif
}


//-------------------------------------------------------------
// Release a spin rw lock with less overhead
//-------------------------------------------------------------
void
SpinRwLockFastReleaseRead( SPIN_RW_LOCK *pSpinRwLock )
{
  SpinRwLockReleaseRead( pSpinRwLock );	// For Now
}
void
SpinRwLockFastReleaseWrite( SPIN_RW_LOCK *pSpinRwLock )
{
  SpinRwLockReleaseWrite( pSpinRwLock );	// For Now
}
