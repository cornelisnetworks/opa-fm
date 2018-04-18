/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef	_IBA_PUBLIC_ISPINLOCK_OSD_H
#define	_IBA_PUBLIC_ISPINLOCK_OSD_H

#include "iba/public/datatypes.h"
#include <pthread.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef	struct _SPIN_LOCK
{
	/* No publically accessible members.  */
	pthread_mutex_t		sp_lock;
} SPIN_LOCK;

typedef	struct _SPIN_RW_LOCK
{
	/* No publically accessible members.  */
	pthread_mutex_t		rw_lock;
} SPIN_RW_LOCK;

typedef uint32 ATOMIC_UINT;

static inline uint32 AtomicRead(const volatile ATOMIC_UINT *pValue) 
{
	return *pValue;
}

static inline void AtomicWrite(volatile ATOMIC_UINT *pValue,
							uint32 newValue)
{
	*pValue = newValue;
}

static inline ATOMIC_UINT AtomicExchange(volatile ATOMIC_UINT *pValue,
										 uint32 newValue)
{
	return __sync_lock_test_and_set(pValue,newValue);
}

static inline boolean AtomicCompareStore(volatile ATOMIC_UINT *pValue,
										 ATOMIC_UINT oldValue,
										 ATOMIC_UINT newValue)
{
	return __sync_bool_compare_and_swap(pValue, oldValue, newValue);
}

static inline ATOMIC_UINT AtomicAdd(ATOMIC_UINT *pValue,
									ATOMIC_UINT add) 
{
	return __sync_add_and_fetch(pValue, add);
}

static inline void AtomicAddVoid(ATOMIC_UINT *pValue, ATOMIC_UINT add) 
{
	(void)AtomicAdd(pValue, add);
}

static inline ATOMIC_UINT AtomicSubtract(ATOMIC_UINT *pValue,
										 ATOMIC_UINT sub) 
{
    return __sync_sub_and_fetch(pValue, sub);
}

static inline void AtomicSubtractVoid(ATOMIC_UINT *pValue, uint32 sub) 
{
	(void)AtomicSubtract(pValue, sub);
}

static inline ATOMIC_UINT AtomicIncrement(ATOMIC_UINT *pValue) 
{
	return AtomicAdd(pValue, 1);
}

static inline void AtomicIncrementVoid(ATOMIC_UINT *pValue) 
{
	AtomicAddVoid(pValue, 1);
}

static inline ATOMIC_UINT AtomicDecrement(ATOMIC_UINT *pValue)
{
	return AtomicSubtract(pValue, 1);
}

static inline void AtomicDecrementVoid(ATOMIC_UINT *pValue) 
{
	AtomicSubtractVoid(pValue, 1);
}

/* for user mode barriers we always use the SMP safe versions */
#if defined(__i386__) || defined(__i686__) || defined(__IA32__)
/* Some non intel clones support out of order store. be conservative  */
static _inline void IoBarrierRead(void)
		 	{ 	__asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory"); }
static _inline void IoBarrierReadWrite(void)
		 	{ 	__asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory"); }
static _inline void IoBarrierWrite(void)
		 	{ __asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory"); }

static _inline void CpuBarrierRead(void) {IoBarrierRead(); }
static _inline void CpuBarrierWrite(void) {IoBarrierWrite(); }
static _inline void CpuBarrierReadWrite(void) {IoBarrierReadWrite(); }

static _inline void CpuPrefetch(void *addr)
{
	asm volatile("prefetchnta (%0)" :: "r" (addr));
}
#elif defined(__x86_64__) || defined(__X64_64__)
static _inline void IoBarrierRead(void) { asm volatile("lfence":::"memory"); }
static _inline void IoBarrierWrite(void) { asm volatile("sfence":::"memory"); }
static _inline void IoBarrierReadWrite(void){asm volatile("mfence":::"memory");}

static _inline void CpuBarrierRead(void) {IoBarrierRead(); }
static _inline void CpuBarrierWrite(void) {IoBarrierWrite(); }
static _inline void CpuBarrierReadWrite(void) {IoBarrierReadWrite(); }

static _inline void CpuPrefetch(void *addr)
{
	asm volatile("prefetcht0 %0" :: "m" (*(unsigned long *)addr));
}
#elif defined(__ia64__) || defined(__IA64__)
static _inline void IoBarrierRead(void)
			{ __asm__ __volatile__ ("mf" ::: "memory"); }
static _inline void IoBarrierWrite(void)
			{ __asm__ __volatile__ ("mf" ::: "memory"); }
static _inline void IoBarrierReadWrite(void)
			{ __asm__ __volatile__ ("mf" ::: "memory"); }

static _inline void CpuBarrierRead(void) {IoBarrierRead(); }
static _inline void CpuBarrierWrite(void) {IoBarrierWrite(); }
static _inline void CpuBarrierReadWrite(void) {IoBarrierReadWrite(); }

static _inline void CpuPrefetch(void *addr)
{
	asm volatile("lfetch [%0]" :: "r" (addr));
}
#elif defined(__ppc__) || defined(__PPC__)
static _inline void IoBarrierRead(void)
			{ __asm__ __volatile__ ("eieio" : : : "memory"); }
static _inline void IoBarrierWrite(void)
			{ __asm__ __volatile__ ("eieio" : : : "memory"); }
static _inline void IoBarrierReadWrite(void)
			{ __asm__ __volatile__ ("eieio" : : : "memory"); }

static _inline void CpuBarrierRead(void)
			{ __asm__ __volatile__ ("isync" : : : "memory"); }
static _inline void CpuBarrierWrite(void)
			{ __asm__ __volatile__ ("eieio" : : : "memory"); }
static _inline void CpuBarrierReadWrite(void)
			{ __asm__ __volatile__ ("isync" : : : "memory"); }

static _inline void CpuPrefetch(void *addr)
{
	asm volatile("dcbt 0,%0" :: "r" (addr));
}
#else
#error "Unsupported CPU type"
#endif

#ifdef __cplusplus
};
#endif

#endif /* _IBA_PUBLIC_ISPINLOCK_OSD_H */
