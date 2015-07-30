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


#ifndef _IBA_PUBLIC_ISPINLOCK_H_
#define _IBA_PUBLIC_ISPINLOCK_H_


#include "iba/public/ispinlock_osd.h"


#ifdef __cplusplus
extern "C"
{
#endif
/*
 * Function declarations.
 */

/*---------------------------------------------------------------------------
 * Simple spin locks.  These are callable in interrupts, callbacks
 * and processes.  The caller must not sleep/wait while holding the lock
 */
/* Initialize a spinlock before its use. */
void
SpinLockInitState( 
	IN SPIN_LOCK *pSpinLock );

boolean
SpinLockInit( 
	IN SPIN_LOCK *pSpinLock );

/* Destroy the spinlock when through using it. */
void
SpinLockDestroy( 
	IN SPIN_LOCK *pSpinLock );

/* Wait on and acquire a spinlock. */
void
SpinLockAcquire( 
	IN SPIN_LOCK *pSpinLock );

/* Wait on and acquire a spinlock.  User must be running at dispatch/interrupt. */
void
SpinLockFastAcquire( 
	IN SPIN_LOCK *pSpinLock );

/* Release the spinlock. */
void
SpinLockRelease(
	IN SPIN_LOCK *pSpinLock );

/* Release a spinlock acquired through the fast acquire. */
void
SpinLockFastRelease( 
	IN SPIN_LOCK *pSpinLock );

/*---------------------------------------------------------------------------
 * Read/Write spin locks.  These are callable in interrupts, callbacks
 * and processes.  The caller must not sleep/wait while holding the lock
 * Initialize a spinlock before its use.
 * These can be slightly more efficient than Simple Spin Locks.
 * locking for Read will allow multiple readers in at once but no writers.
 * locking for Write allows no one else in (readers nor writers).
 * A process must be consistent that its Acquire (Read/Write and Fast) matches
 * its Release.
 */
void
SpinRwLockInitState( 
	IN SPIN_RW_LOCK *pSpinRwLock );

boolean
SpinRwLockInit( 
	IN SPIN_RW_LOCK *pSpinRwLock );

/* Destroy the spinlock when through using it. */
void
SpinRwLockDestroy( 
	IN SPIN_RW_LOCK *pSpinRwLock );

/* Wait on and acquire a spinlock. */
void
SpinRwLockAcquireRead( 
	IN SPIN_RW_LOCK *pSpinRwLock );
void
SpinRwLockAcquireWrite( 
	IN SPIN_RW_LOCK *pSpinRwLock );


/* Wait on and acquire a spinlock.  User must be running at dispatch/interrupt. */
void
SpinRwLockFastAcquireRead( 
	IN SPIN_RW_LOCK *pSpinRwLock );
void
SpinRwLockFastAcquireWrite( 
	IN SPIN_RW_LOCK *pSpinRwLock );

/* Release the spinlock. */
void
SpinRwLockReleaseRead(
	IN SPIN_RW_LOCK *pSpinRwLock );
void
SpinRwLockReleaseWrite(
	IN SPIN_RW_LOCK *pSpinRwLock );

/* Release a spinlock acquired through the fast acquire. */
void
SpinRwLockFastReleaseRead( 
	IN SPIN_RW_LOCK *pSpinRwLock );
void
SpinRwLockFastReleaseWrite( 
	IN SPIN_RW_LOCK *pSpinRwLock );

/*---------------------------------------------------------------------------
 * Memory Barriers.
 *
 * Some CPU Architectures permit out of order access to memory.
 * For code within a spinlock this is not an issue because on such
 * CPUs, the spin lock code will implement a memory barrier.
 * However when accessing PCI memory (or CPU memory shared with hardware)
 * or when 2 Threads synchronize themselves without using spinlocks
 * (such as when ring buffers are used), Memory Barriers are needed to
 * force the ordering of memory access.
 *
 * IoBarriers protect memory shared with IO hardware.
 * CpuBarriers protect memory shared between threads/CPUs.
 *
 * A Read barrier ensures all Reads before the Barrier complete prior to
 * performing any Reads after the barrier
 *		for example:
 *			if (p->valid) {
 *				XBarrierRead();	// make sure reads below don't happen before
 *								// valid is set
 *				read rest of *p 
 *			}
 *
 * A Write barrier ensures all Writes before the Barrier complete prior to
 * performing any Writes after the barrier
 *		for example:
 *			put data at *p
 *			XBarrierWrite(); // make sure p written before we set valid
 *			p->valid = 1;
 *				OR
 *			put WQE on QP
 *			XBarrierWrite();	// make sure WQE written before set valid
 *			wqe->valid = 1;
 *			XBarrierWrite();	// make sure WQE marked as valid before ring
 *			*doorbell = RING;
 *
 * A Read/Write barrier insures all Reads and Writes before the Barrier complete
 * prior to performing any Reads or Writes after the barrier.
 *
 * These are interrupt callable.
 */
static _inline void IoBarrierRead(void);
static _inline void IoBarrierWrite(void);
static _inline void IoBarrierReadWrite(void);

static _inline void CpuBarrierRead(void);
static _inline void CpuBarrierWrite(void);
static _inline void CpuBarrierReadWrite(void);

/* This can help optimize cache operation by beginning the memory read before
 * the data is actually needed
 */
static _inline void CpuPrefetch(void *addr);

/*---------------------------------------------------------------------------
 * Atomic operations for reading, writing, incrementing and decrementing
 * ATOMIC_UINT is opaque system specific container for an atomically
 * accessible value.
 * This container will contain at least 24 bits of value, but may be larger than
 * 32 bits.
 * The API allows 32 bit values to be added, subtracted and exchanged with this
 * value.  If the value goes negative or exceeds 24 bits behaviour is undefined.
 *
 * functions declared in ispinlock_osd.h so they can be inline when appropriate
 */

/*uint32 
 *AtomicRead( 
 *	IN ATOMIC_UINT *pValue );
 */

/*void 
 *AtomicWrite( 
 *	IN ATOMIC_UINT *pValue, IN uint32 newValue );
 */

/*uint32 
 *AtomicExchange( 
 *	IN ATOMIC_UINT *pValue, IN uint32 newValue );
 */

/* atomically performs:
 * if (*pValue == oldValue) {
 *		*pValue = newValue; return TRUE;
 * } else
 *		return FALSE;
 *boolean 
 *AtomicCompareStore( 
 *	IN ATOMIC_UINT *pValue, IN uint32 oldValue, IN uint32 newValue );
 */

/*uint32 
 *AtomicIncrement( 
 *	IN ATOMIC_UINT *pValue );
 */

/*void 
 *AtomicIncrementVoid( 
 *	IN ATOMIC_UINT *pValue );
 */

/*uint32 
 *AtomicDecrement( 
 *	IN ATOMIC_UINT *pValue );
 */

/*void 
 *AtomicDecrementVoid( 
 *	IN ATOMIC_UINT *pValue );
 */

/*uint32 
 *AtomicAdd( 
 *	IN ATOMIC_UINT *pValue, uint32 add );
 */

/*void 
 *AtomicAddVoid( 
 *	IN ATOMIC_UINT *pValue, uint32 add );
 */

/*uint32 
 *AtomicSubtract( 
 *	IN ATOMIC_UINT *pValue, uint32 sub );
 */

/*void 
 *AtomicSubtractVoid( 
 *	IN ATOMIC_UINT *pValue, uint32 sub );
 */

#ifdef ATOMIC_TEST

#include "iba/public/idebug.h"

/* simple single threaded test to verify Atomics */
static __inline void
AtomicRunTest(void)
{
	ATOMIC_UINT a;

	MsgOut("Running Atomic Tests\n");
	AtomicWrite(&a, 5);
	if (AtomicRead(&a) != 5)
		{ MsgOut("AtomicRead Failed\n"); goto fail; }
	if (AtomicIncrement(&a) != 6)
		{ MsgOut("AtomicIncrement Failed\n"); goto fail; }
	AtomicIncrementVoid(&a);
	if (AtomicRead(&a) != 7)
		{ MsgOut("AtomicIncrementVoid Failed\n"); goto fail; }
	if (AtomicCompareStore(&a, 5, 7))
		{ MsgOut("AtomicCompareStore Failed\n"); goto fail; }
	if (! AtomicCompareStore(&a, 7, 0))
		{ MsgOut("AtomicCompareStore Failed\n"); goto fail; }
	if (AtomicExchange(&a, 1))
		{ MsgOut("AtomicExchange 1 Failed\n"); goto fail; }
	if (AtomicRead(&a) != 1)
		{ MsgOut("AtomicExchange 1 read Failed\n"); goto fail; }
	if (! AtomicExchange(&a, 0))
		{ MsgOut("AtomicExchange 0 Failed\n"); goto fail; }
	if (AtomicRead(&a) != 0)
		{ MsgOut("AtomicExchange 0 read Failed\n"); goto fail; }
	AtomicWrite(&a, 7);
	if (AtomicDecrement(&a) != 6)
		{ MsgOut("AtomicDecrement Failed\n"); goto fail; }
	AtomicDecrementVoid(&a);
	if (AtomicRead(&a) != 5)
		{ MsgOut("AtomicDecrementVoid Failed\n"); goto fail; }
	if (AtomicAdd(&a, 3) != 8)
		{ MsgOut("AtomicAdd Failed\n"); goto fail; }
	AtomicAddVoid(&a, 4);
	if (AtomicRead(&a) != 12)
		{ MsgOut("AtomicAddVoid Failed\n"); goto fail; }
	if (AtomicSubtract(&a, 3) != 9)
		{ MsgOut("AtomicSubtract Failed\n"); goto fail; }
	AtomicSubtractVoid(&a, 4);
	if (AtomicRead(&a) != 5)
		{ MsgOut("AtomicSubtractVoid Failed\n"); goto fail; }
	MsgOut("Atomic tests passed\n");
	return;
fail:
	MsgOut("Atomic tests failed\n");
	return;
}
#endif

#ifdef __cplusplus
}	/* extern "C" */

/* RAI object to help create and release spin locks */
class	AutoSpinLock
{
public:
	AutoSpinLock( SPIN_LOCK	*pLock) 
	{
		m_lock = pLock;
		SpinLockAcquire(m_lock);
	}

	~AutoSpinLock() 
	{
		SpinLockRelease(m_lock);
	}

private:
	SPIN_LOCK		*m_lock;
};

/* RAI objects to help create and release RW spin locks */
class	AutoRwSpinLockRead
{
public:
	AutoRwSpinLockRead( SPIN_RW_LOCK	*pLock) 
	{
		m_lock = pLock;
		SpinRwLockAcquireRead(m_lock);
	}

	~AutoRwSpinLockRead() 
	{
		SpinRwLockReleaseRead(m_lock);
	}

private:
	SPIN_RW_LOCK		*m_lock;
};

class	AutoRwSpinLockWrite
{
public:
	AutoRwSpinLockWrite( SPIN_RW_LOCK	*pLock) 
	{
		m_lock = pLock;
		SpinRwLockAcquireWrite(m_lock);
	}

	~AutoRwSpinLockWrite() 
	{
		SpinRwLockReleaseWrite(m_lock);
	}

private:
	SPIN_RW_LOCK		*m_lock;
};
#endif /* __cplusplus */

#endif /* _IBA_PUBLIC_ISPINLOCK_H_ */
