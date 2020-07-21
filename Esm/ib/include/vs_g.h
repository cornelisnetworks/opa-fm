/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
 *
 * FILE NAME
 *    vs_g.h
 *
 * DESCRIPTION
 *    OS Abstraction Layer Header
 *
 * DATA STRUCTURES
 *
 * FUNCTIONS
 *      vs_time_get              Get OS independent uS since epoch
 *
 *
 * DEPENDENCIES
 *    ib_types.h
 *    ib_status.h
 *
 *
 * HISTORY
 *
 *    NAME DATE      REMARKS
 *    dkj  02/27/02  Initial creation of file.
 *    pjg  02/27/02  Moved vs_time_get to here.
 *    pjg  02/28/02  Move VEVENT_WAKE_* from cs_g.h
 *    sfw  03/05/02  Added initial Thread support
 *    dkj  03/05/02  Added vs_pool services
 *    dkj  03/06/02  Added additional vs_pool services
 *    sfw  03/07/02  Added comments for new functions implemented
 *    pjg  03/11/02  Increase OPAQUE_THREAD_ELEMENTS.
 *    dkj  03/12/02  Increase OPAQUE_POOL_ELEMENTS.
 *    dkj  03/14/02  Updated passing functions list
 *    dkj  03/18/02  Added vs_pool_page_size
 *    mgr  03/18/02  Added counting semaphore support.
 *    dkj  03/19/02  Updated events to use New locks
 *    sfw  03/20/02  Added flag to indicate  is prefixed to prototypes
 *    dkj  03/21/02  Updated vs_pool_create prototype
 *    dkj  03/22/02  More Pool prototype cleanup
 *    dkj  04/01/02  PR 1676. vs_pool* prototype per OS API 2.0g
 *    pjg  04/02/02  PR 1676. vs_thread_sleep prototype per OS API 2.0g
 *    dkj  04/02/02  PR 1676. vs_event* prototype per OS API 2.0g
 *    mgr  04/02/02  PR 1651. Added user_flag to Thread_t structure.
 *    dkj  04/04/02  PR 1676. vs_thread_create prototype per OS API 2.0g
 *    mgr  04/04/02  PR 1676. vs_lock_init prototype per OS API 2.0g
 ************************************************************************/

/************************************************************************
 * The following is a list of functions that successfully pass testing
 * and in which environments.
 *
 * vs_event_*() - Linux Kernel, Linux User
 * vs_time_get() - Linux Kernel, Linux User
 * vs_thread_*() - Linux Kernel, Linux User
 * vs_pool_create() - Linux Kernel, Linux User
 * vs_pool_delete() - Linux Kernel, Linux User
 * vs_pool_alloc() - Linux Kernel, Linux User
 * vs_pool_free() - Linux Kernel, Linux User
 * vs_pool_page_size() - Linux Kernel, Linux User, ATI
 ************************************************************************/


#ifndef __VS_G_
#define __VS_G_

#include <stl_types.h>
#include <ib_status.h>
#include <stdlib.h>
#include <string.h>

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#define VS_NAME_MAX (16U)

/*=== Locks ===*/
#define VLOCK_LOCKED  (0)		/* Initialize to locked		*/
#define VLOCK_FREE    (1)		/* Initialze lock to free	*/
#define VLOCK_SPIN    (3)       /* Initialize spin lock         */
#define VLOCK_THREAD  (4)       /* Initialize thread lock       */
#define VLOCK_RWTHREAD  (5)     /* Initialize rw thread lock       */

#define OPAQUE_LOCK_ELEMENTS              (18U)
#define VLOCK_THREADLOCK_MAGIC   (0x283CB160UL)
#define VLOCK_RWTHREADLOCK_MAGIC   (0x283CB185UL)
#define VLOCK_SPINLOCK_MAGIC     (0xAF22AE30UL)
typedef volatile struct
{
  uint32_t type;		/* type of lock (VLOCK_SPIN, VLOCK_RWTHREAD or VLOCK_THREAD) */
  uint32_t status;		/* status of lock (VLOCK_LOCKED or VLOCK_FREE) */
  uint32_t magic;		/* magic of lock (VLOCK_THREAD/SPINLOCK_MAGIC) */

  /* implementation private data */
  uint64_t opaque[OPAQUE_LOCK_ELEMENTS];
}
Lock_t;

extern Status_t vs_lock_init (Lock_t * handle,
                                  unsigned int state,
                                  unsigned int type);
extern Status_t vs_lock_delete (Lock_t * handle);
extern Status_t vs_lock (Lock_t * handle);		// for THREAD
extern Status_t vs_unlock (Lock_t * handle);	// for THREAD
extern Status_t vs_wrlock (Lock_t * handle);	// for RWTHREAD
extern Status_t vs_rdlock (Lock_t * handle);	// for RWTHREAD
extern Status_t vs_rwunlock (Lock_t * handle);	// for RWTHREAD
extern Status_t vs_spinlock (Lock_t * handle);	// for SPIN
extern Status_t vs_spinunlock (Lock_t * handle);	// for SPIN

/*=== Locks ===*/

/*=== Events ===*/

typedef void *Evt_handle_t;
typedef uint32_t Eventset_t;
#define VEVENT_NUM_EVENTS (sizeof(Eventset_t) << 3)
#define OPAQUE_EVENT_ELEMENTS (25U)
typedef struct
{
  unsigned char name[VS_NAME_MAX];
  Evt_handle_t event_handle;
  uint64_t opaque[OPAQUE_EVENT_ELEMENTS];
}
Event_t;
/*
** Creates a set of global events and associates a name with the
**   event set.
*/
extern Status_t vs_event_create (Event_t * ec, unsigned char name[],
				     Eventset_t state);
/*
** Deletes the specified event set.
*/
extern Status_t vs_event_delete (Event_t * ec);
/*
** Wait for any of specified events.
*/
extern Status_t vs_event_wait (Evt_handle_t handle, uint64_t timeout,
				   Eventset_t mask, Eventset_t * events);
/*
** Post a set of events.
*/
extern Status_t vs_event_post (Evt_handle_t handle, uint32_t option,
				   Eventset_t mask);
#define VEVENT_WAKE_ONE   (1)	/* Wake one thread waiting              */
#define VEVENT_WAKE_ALL   (2)	/* Wake all threads in vs_event_wait()  */
				// BEWARE, a WAKE_ALL before there are waiters will cause
				// the waiters to still wait for the next WAKE_ALL or WAKE_ONE

/*=== Events ===*/

/*=== Time ===*/

/*
 * vs_time_get
 *   Return a 64 bit number of uSecs since some epoch.  This function must
 * return a monotonically increasing sequence.  The actual granularity of
 * time is dependent on the OS/HW you are running on.
 *
 * INPUTS
 *      loc         Where to put the time value
 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 */
Status_t vs_time_get (uint64_t * loc);

#define	VTIMER_1S		            1000000ull
#define VTIMER_1_MILLISEC           1000ull
#define VTIMER_10_MILLISEC          10000ull
#define VTIMER_100_MILLISEC         100000ull
#define VTIMER_200_MILLISEC         200000ull
#define VTIMER_500_MILLISEC         500000ull
#define	VTIMER_ETERNITY		 0x0100000000000000ull
#define  TIMER_1S		           (1000000ull)
#define  TIMER_ETERNITY		(0x0100000000000000ull)

/*
 * vs_stdtime_get - get current time as a time_t, seconds since 1970
 */
Status_t vs_stdtime_get(time_t *address);
/*=== Time ===*/

/*=== Core Dumps ===*/
// Translate CoreDumpLimit string to a rlimit value
// returns 0 on success, -1 on error
// value argument is optional, if NULL just checks str is valid CoreDumpLimit
// str can be a byte count or "unlimited"
int vs_getCoreDumpLimit(const char* str, uint64_t* value);

#ifndef __VXWORKS__
/*
 *  vs_init_coredump_settings - setup coredump config for this process
 *  note this changes the current directory
 */
void vs_init_coredump_settings(const char* mgr, const char* limit, const char *dir);
#endif
/*=== Core Dumps ===*/

/*=== Threads ===*/

#define OPAQUE_THREAD_ELEMENTS (96U)	/* XXXXX for now */

typedef uint64_t Threadname_t;

typedef struct New_Thread
{
  unsigned char name[VS_NAME_MAX];
  uint32_t magic;
  unsigned int user_flag;
  int priority;

  struct Thread *next;	/* XXXXXX maybe will use these */
  struct Thread *prev;

  uint64_t opaque[OPAQUE_THREAD_ELEMENTS];
}
Thread_t;

/*
** Create a thread.
*/
extern Status_t
vs_thread_create (Thread_t * thr,
		      unsigned char * name,
		      void (*function) (uint32_t, uint8_t **),
		      uint32_t argc,
		      uint8_t * argv[],
                      size_t stack_size);

/*
** Create a thread and copy its argv vector.
*/
extern Status_t
vs_thread_create2(Thread_t * thr,
		      unsigned char * name,
		      void (*function) (uint32_t, uint8_t **),
		      uint32_t argc,
		      uint8_t * argv[],
                      size_t stack_size);

/*
** Query the threadname associated with the current thread
*/
extern Status_t vs_thread_name (Threadname_t * threadname);

/*
** Query the thread name string associated with the current thread
*/
extern const char *vs_thread_name_str (void);

/*
** Query the threadname group associated with the current
** thread
*/
extern Status_t
vs_thread_groupname (Threadname_t threadname,
			 Threadname_t * groupname);

/*
** Kill the specified thread
*/
extern Status_t vs_thread_kill (Thread_t * handle);

/*
** Exit the current thread, does not return on success.
*/
extern Status_t vs_thread_exit (Thread_t * handle);

/*
** Sleep for sleeptime microseconds.
*/
extern void vs_thread_sleep (uint64_t sleep_time);

/*
** Wait for the specified thread to exit.
*/
extern Status_t
vs_thread_join (Thread_t *handle, void **value_ptr);

/*
 ** set name for the calling thread
*/
extern int vs_thread_setname(char* name);

/*=== Threads ===*/

/*=== Pool Services ===*/
#define VMEM_PAGE	(0x00000008)	/* Return page aligned chunks	*/


#define DELETE_MARKER	0xCECE

#ifndef USE_POOL_LOCK
#define USE_POOL_LOCK 1
#endif

#define OPAQUE_POOL_ELEMENTS (8U)

typedef struct
{
  unsigned char name[VS_NAME_MAX];
  uint32_t options;
  uint32_t magic;
#if USE_POOL_LOCK
  Lock_t lock;
#endif /* USE_POOL_LOCK */
  size_t pagesize;
  uint64_t opaque[OPAQUE_POOL_ELEMENTS];
}
Pool_t;
/*
 * vs_pool_create
 *   Create a memory allocation pool.  Assign attributes to how memory is
 * allocated out of the pool.
 */
Status_t vs_pool_create (Pool_t * handle, uint32_t options,
			     unsigned char *name, void *address, size_t size);
/*
 * vs_pool_delete
 *   Release a pool.
 */
Status_t vs_pool_delete (Pool_t * handle);
/*
 * vs_pool_alloc
 *   Allocate a sub-chunk from an existing pool.  The region will be
 * allocated according to the options given when the pool was created.
 */
Status_t vs_pool_alloc (Pool_t * handle, size_t length, void **loc);
/*
 * vs_pool_free
 *   Release a chunk of memory allocated using vs_pool_alloc()
 */
Status_t vs_pool_free (Pool_t * handle, void *loc);
/*
 * vs_pool_size
 *   Return current size of pool (for dev debug)
 */
Status_t vs_pool_size(Pool_t * handle, uint64_t *numBytesAlloc);

/*
 * vs_pool_page_size
 *   Obtain the native memory page size in bytes.
 */
size_t vs_pool_page_size (void);
/*=== Pool Services ===*/

/*=== Basic Allocation ==*/

#define vs_malloc(LENGTH) malloc(LENGTH)
#define vs_free(ADDR) free(ADDR)
/*=== Basic Allocation ==*/

/*=== Semaphores ===*/
#define SEMAPHORE_MAGIC   (0xCB283037UL)
#ifdef __VXWORKS__
#include "semLib.h"
#else
#include <semaphore.h>
#endif

typedef volatile struct
{
  uint32_t magic;
#ifdef __VXWORKS__
  SEM_ID semId;
  int32_t count;
#else
sem_t osdSema;
#endif
}
Sema_t;

//
// MWHEINZ FIXME: Why are the cs_ functions prototyped here and not in cs_g.h?
// 
#define CS_SEMA_WAIT_FOREVER  -1
#define CS_SEMA_NO_WAIT        0
extern Status_t cs_sema_create (Sema_t * handle, uint32_t count);
extern Status_t cs_sema_delete (Sema_t * handle);
extern Status_t cs_vsema (Sema_t * handle);
extern Status_t cs_psema (Sema_t * handle);
extern Status_t cs_sema_getcount(Sema_t * handle, int *cnt);
/* wait on a semaphore for desired number of seconds */
extern Status_t cs_psema_wait (Sema_t * handle, int timeout);
/*=== Semaphores ===*/

/*=== String Operations ===*/
/* max/min values for uint*_t and int*_t */
#ifndef UINT8_MAX
#define UINT8_MAX       255
#endif

#ifndef UINT8_MIN
#define UINT8_MIN         0
#endif

#ifndef INT8_MAX
#define INT8_MAX        127
#endif

#ifndef INT8_MIN
#define INT8_MIN      (-128)
#endif

#ifndef UINT16_MAX
#define UINT16_MAX    65535
#endif

#ifndef UINT16_MIN
#define UINT16_MIN        0
#endif

#ifndef INT16_MAX
#define INT16_MAX     32767
#endif

#ifndef INT16_MIN
#define INT16_MIN   (-32768)
#endif

#ifndef UINT32_MAX
#define UINT32_MAX            4294967295U
#endif

#ifndef UINT32_MIN
#define UINT32_MIN                     0
#endif

#ifndef INT32_MAX
#define INT32_MAX             2147483647
#endif

#ifndef INT32_MIN
#define INT32_MIN         (-INT32_MAX -1)
#endif

#ifndef UINT64_MAX
#define UINT64_MAX  18446744073709551615ULL
#endif

#ifndef UINT64_MIN
#define UINT64_MIN                     0
#endif

#ifndef INT64_MAX
#define INT64_MAX     9223372036854775807LL
#endif

#ifndef INT64_MIN
#define INT64_MIN         (-INT64_MAX - 1LL)
#endif

extern uint8_t cs_strtoui8 (const char *nptr, uint8_t return_error);
extern uint16_t cs_strtoui16 (const char *nptr, uint16_t return_error);
extern uint32_t cs_strtoui32 (const char *nptr, uint32_t return_error);
extern uint64_t cs_strtoui64 (const char *nptr, uint64_t return_error);
extern int8_t cs_strtoi8 (const char *nptr, int8_t return_error);
extern int16_t cs_strtoi16 (const char *nptr, int16_t return_error);
extern int32_t cs_strtoi32 (const char *nptr, int32_t return_error);
extern int64_t cs_strtoi64 (const char *nptr, int64_t return_error);
extern Status_t	cs_parse_gid	(const char * str, Gid_t gid);

extern char *cs_getAidName(uint16_t, uint16_t);
extern char *cs_getMethodText(uint8_t);

extern int cs_snprintfcat(char ** buf, size_t * len, char * fmt, ...) __attribute__((format(printf,3,4)));
/*=== String Operations ===*/

/*=== Computational Utility Functions ===*/
extern uint32_t cs_numSwitches(uint32_t subnet_size);
extern uint32_t cs_numNodeRecords(uint32_t subnet_size);
extern uint32_t cs_numPortRecords(uint32_t subnet_size);
extern uint32_t cs_numLinkRecords(uint32_t subnet_size);
/*=== Computational Utility Functions ===*/
#endif
