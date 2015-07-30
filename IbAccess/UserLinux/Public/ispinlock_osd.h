/* BEGIN_ICS_COPYRIGHT2 ****************************************

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
	/*pthread_rwlock_t		rw_lock;	// only available for UNIX98 */
	pthread_mutex_t		rw_lock;
} SPIN_RW_LOCK;

#if defined(__IA64__) || defined(__ia64)

typedef uint32 ATOMIC_UINT;

static inline uint32 AtomicRead (IN const volatile ATOMIC_UINT *pValue) 
{
	return *pValue;
}

static inline void AtomicWrite (IN volatile ATOMIC_UINT *pValue,
									IN uint32 newValue)
{
	*pValue = newValue;
}

static inline uint32 AtomicExchange(IN volatile ATOMIC_UINT *pValue,
										uint32 newValue)
{
	uint64 result;

	__asm__ __volatile ("xchg4 %0=[%1],%2" : "=r" (result)
				    : "r" (pValue), "r" (newValue) : "memory");
	return (uint32)result;
}
#define InterlockedExchange AtomicExchange	/* depricated */

static inline boolean AtomicCompareStore(IN volatile ATOMIC_UINT *pValue,
						uint32 oldValue, uint32 newValue)
{
    uint64 o = oldValue;
	uint64 r;
    __asm__ __volatile__ ("mov ar.ccv=%0;;" :: "rO"(o));
    __asm__ __volatile__ ("cmpxchg4.acq %0=[%1],%2,ar.ccv"
                      : "=r"(r) : "r"(pValue), "r"(newValue) : "memory");
	return r == oldValue;
}

static inline uint32 AtomicAdd (IN ATOMIC_UINT *pValue, uint32 add) 
{
    uint32 oldValue, newValue;

    do {
        oldValue = *pValue;
        newValue = oldValue + add;
    } while (! AtomicCompareStore(pValue, oldValue, newValue));
    return newValue;
}

static inline void AtomicAddVoid (IN ATOMIC_UINT *pValue, uint32 add) 
{
	(void)AtomicAdd(pValue, add);
}

static inline uint32 AtomicSubtract (IN ATOMIC_UINT *pValue, uint32 sub) 
{
    uint32 oldValue, newValue;

    do {
        oldValue = *pValue;
        newValue = oldValue - sub;
    } while (! AtomicCompareStore(pValue, oldValue, newValue));
    return newValue;
}

static inline void AtomicSubtractVoid (IN ATOMIC_UINT *pValue, uint32 sub) 
{
	(void)AtomicSubtract(pValue, sub);
}

static inline uint32 AtomicIncrement (IN ATOMIC_UINT *pValue) 
{
	return AtomicAdd(pValue, 1);
}

static inline void AtomicIncrementVoid (IN ATOMIC_UINT *pValue) 
{
	AtomicAddVoid(pValue, 1);
}

static inline uint32 AtomicDecrement (IN ATOMIC_UINT *pValue)
{
	return AtomicSubtract(pValue, 1);
}

static inline void AtomicDecrementVoid (IN ATOMIC_UINT *pValue) 
{
	AtomicSubtractVoid(pValue, 1);
}

#elif defined(__IA32__) || defined(__X86_64__) || defined(__i386) || defined(__x86_64)
typedef uint32 ATOMIC_UINT;

static inline uint32 AtomicRead (IN const volatile ATOMIC_UINT *pValue) 
{
	/* these processors natively do 32 bits read atomically */
	return *pValue;
}

static inline void AtomicWrite (IN volatile ATOMIC_UINT *pValue, IN uint32 newValue)
{
	/* these processors natively do 32 bits writes atomically */
	*pValue = newValue;
}

static inline uint32 AtomicExchange(IN volatile ATOMIC_UINT *pValue, uint32 newValue)
{
	__asm__ __volatile__("xchgl %0,%1"
				:"=r" (newValue)
				:"m" (*pValue), "0" (newValue)
				:"memory");
	return newValue;
}

static inline uint32 AtomicCompareStore(IN volatile ATOMIC_UINT *pValue,
					uint32 oldValue, uint32 newValue)
{
		uint32 prev;

		__asm__ __volatile__("lock; cmpxchgl %1,%2"
				     : "=a"(prev)
				     : "q"(newValue), "m"(*pValue), "0"(oldValue)
				     : "memory");
		return prev == oldValue;
}

static inline uint32 AtomicAdd (IN ATOMIC_UINT *pValue, uint32 add) 
{
	uint32 oldValue, newValue;

	do {
		oldValue = AtomicRead(pValue);
		newValue = oldValue + add;
	} while (! AtomicCompareStore(pValue, oldValue, newValue));
	/* return post increment value */
	return newValue;
}

static inline void AtomicAddVoid (IN ATOMIC_UINT *pValue, uint32 add) 
{
	(void)AtomicAdd(pValue, add);
}

static inline uint32 AtomicSubtract (IN ATOMIC_UINT *pValue, uint32 sub) 
{
	uint32 oldValue, newValue;

	do {
		oldValue = AtomicRead(pValue);
		newValue = oldValue - sub;
	} while (! AtomicCompareStore(pValue, oldValue, newValue));
	/* return post increment value */
	return newValue;
}

static inline void AtomicSubtractVoid (IN ATOMIC_UINT *pValue, uint32 sub) 
{
	(void)AtomicSubtract(pValue, sub);
}

static inline uint32 AtomicIncrement (IN ATOMIC_UINT *pValue) 
{
	return AtomicAdd(pValue, 1);
}

static inline void AtomicIncrementVoid (IN volatile ATOMIC_UINT *pValue) 
{
	__asm__ __volatile__(
		"lock ;incl %0"
		: "=m" (*pValue)
		: "m" (*pValue));
}

static inline uint32 AtomicDecrement (IN ATOMIC_UINT *pValue) 
{
	return AtomicSubtract(pValue, 1);
}

static inline void AtomicDecrementVoid (IN volatile ATOMIC_UINT *pValue)
{
    __asm__ __volatile__(
        "lock ; decl %0"
        :"=m" (*pValue)
        :"m" (*pValue));	
}

#elif defined(__ppc__) || defined(__PPC__)
typedef uint32 ATOMIC_UINT;

static inline uint32 AtomicRead (IN const volatile ATOMIC_UINT *pValue) 
{
	return *pValue;	/* this is atomic on PPC */
}

static inline void AtomicWrite (IN volatile ATOMIC_UINT *pValue,
								IN uint32 newValue)
{
	*pValue = newValue;	/* this is atomic on PPC */
}

static inline uint32 AtomicAdd (IN volatile ATOMIC_UINT *pValue, uint32 add) 
{
	uint32 t;

	__asm__ __volatile__(
"1:	lwarx	%0,0,%2\n\
	add	%0,%1,%0\n\
	stwcx.	%0,0,%2\n\
	bne-	1b\n\
	isync"
	: "=&r" (t)
	: "r" (add), "r" (pValue)
	: "cc", "memory");

	return t;
}

static inline void AtomicAddVoid (IN ATOMIC_UINT *pValue, uint32 add) 
{
	(void)AtomicAdd(pValue, add);
}

static inline uint32 AtomicSubtract (IN volatile ATOMIC_UINT *pValue,
									uint32 sub)
{
	int t;

	__asm__ __volatile__(
"1:	lwarx	%0,0,%2\n\
	subf	%0,%1,%0\n\
	stwcx.	%0,0,%2\n\
	bne-	1b\n\
	isync"
	: "=&r" (t)
	: "r" (sub), "r" (pValue)
	: "cc", "memory");

	return (uint32)t;
}

static inline void AtomicSubtractVoid (IN ATOMIC_UINT *pValue, uint32 sub)
{
	(void)AtomicSubtract(pValue, sub);
}

static inline uint32 AtomicIncrement (IN ATOMIC_UINT *pValue) 
{
	return AtomicAdd(pValue, 1);
}

static inline void AtomicIncrementVoid (IN ATOMIC_UINT *pValue) 
{
	(void)AtomicAdd(pValue, 1);
}
static inline uint32 AtomicDecrement (IN ATOMIC_UINT *pValue)
{
	return AtomicSubtract(pValue, 1);
}

static inline void AtomicDecrementVoid (IN ATOMIC_UINT *pValue)
{
	(void)AtomicSubtract(pValue, 1);
}

static inline uint32 AtomicExchange(IN volatile ATOMIC_UINT *pValue,
									uint32 newValue)
{
	unsigned long oldValue;

	__asm__ __volatile__ ("\n\
1:	lwarx	%0,0,%2 \n\
	stwcx.	%3,0,%2 \n\
	bne-	1b"
	: "=&r" (oldValue), "=m" (*(volatile uint32 *)pValue)
	: "r" (pValue), "r" (newValue), "m" (*(volatile uint32 *)pValue)
	: "cc", "memory");

	return (uint32)oldValue;
}
#define InterlockedExchange AtomicExchange	/* depricated */

static inline boolean AtomicCompareStore(IN volatile ATOMIC_UINT *pValue,
						 uint32 oldValue, uint32 newValue)
{
	uint32 prev;

	__asm__ __volatile__ ("\n\
1:	lwarx	%0,0,%2 \n\
	cmpw	0,%0,%3 \n\
	bne	2f \n\
	stwcx.	%4,0,%2 \n\
	bne-	1b\n\
	sync\n\
2:"
	: "=&r" (prev), "=m" (*pValue)
	: "r" (pValue), "r" (oldValue), "r" (newValue), "m" (*pValue)
	: "cc", "memory");

	return prev == oldValue;
}

#else
#error "Unsupported processor type"
#endif

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
