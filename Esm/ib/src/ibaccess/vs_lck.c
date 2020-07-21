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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/***********************************************************************
* 
* FILE NAME
*      vs_lck.c
*
* DESCRIPTION
*      This file contains the Linux User implementation of the vs_lock
*      services.
*
* DATA STRUCTURES
*
* FUNCTIONS
*      vs_lock_init
*      vs_lock_delete
*      vs_lock
*      vs_unlock
*      vs_spinlock
*      vs_spinunlock
*
* DEPENDENCIES
*      vs_g.h
*      cs_g.h
*      cs_log.h
*
*
* HISTORY
*
* NAME      DATE        REMARKS
* MGR       03/01/02    Initial creation of file.
* MGR       03/13/02    Added spinlock recursion support.
* MGR       03/21/02    Added fix for bug, where only one thread waiting
*                       on the thread lock was woken when lock was deleted.
* MGR       03/26/02    Corrected lint errors.
* MGR       04/04/02    PR1676. Rev G, CS API required changes.
* MGR       04/10/02    PR1716.  Fixed spinlocks to block until freed by
*                       the owning thread.
* MGR       05/13/02    PR1812.  Clean-up lint errors/warnings.
*
***********************************************************************/
#ifdef BUILD_RHEL4
#define _POSIX_C_SOURCE 200112L
#endif
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include "ib_types.h"
#include "ib_status.h"
#include "vs_g.h"
#include "cs_g.h"
#include "cs_log.h"
#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID
/*
#include <vsdd.h>
*/
#include <sys/ioctl.h>
#include "iba/public/ispinlock.h"

/*
** Implementation private data structure 
*/
typedef struct
{
  uint8_t           name[VS_NAME_MAX];
  union {
    struct {                        // for THREAD
       pthread_mutex_t   mutex;
    } lock;
    struct {                        // for RWTHREAD
       pthread_rwlock_t  rwlock;
       ATOMIC_UINT reader_count;
    } rwlock;
    struct {                        // for recursive SPIN
       pthread_mutex_t   mutex;
       pthread_cond_t    cond;
       Threadname_t  owner_thread;
       uint32_t          spin_count;
    } spin;
  } u;
} Implpriv_Lock_t;


/**********************************************************************
*
* FUNCTION
*    vs_lock_init
*
* DESCRIPTION
*    Initialize a thread or spin lock.
*
* INPUTS
*    handle   - A pointer to a lock control block.
*    state    - The initial state of the lock, VLOCK_LOCKED or VLOCK_FREE.
*    type     - type of lock to create, VLOCK_SPIN, VLOCK_THREAD, VLOCK_RWTHREAD
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*   MGR     03/13/02    Added spinlock recursion support.
*   MGR     04/04/02    PR1676. Rev G, CS API required changes.
*
**********************************************************************/
Status_t
vs_lock_init (Lock_t *handle,
                  unsigned int state,
                  unsigned int type)
{
  Status_t          rc;
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_lock_init";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)state,
            (uint32_t)type,
        (uint32_t)0U);

  /*
  ** Validate parameters
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if ((state != VLOCK_LOCKED) && (state != VLOCK_FREE))
  {
    IB_LOG_ERROR ("invalid state:", state);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if ((type != VLOCK_SPIN) && (type != VLOCK_THREAD)
          && (type != VLOCK_RWTHREAD))
  {
    IB_LOG_ERROR ("invalid type:", type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Ensure opaque section of Lock_t is large enough to
  ** contain the Implementation private data structure.
  */
  if ((sizeof(Implpriv_Lock_t)) > (sizeof(handle->opaque)))
  {
    IB_LOG_ERROR("Implpriv_Lock_t too large:", (sizeof(Implpriv_Lock_t)));
    IB_FATAL_ERROR_NODUMP("Implpriv_Lock_t too large");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Do initial setup
  */
  // TBD - can we drop implpriv part for vs_lock and simply declare
  // an OS specific Lock_t
  memset ((void *)handle, 0, sizeof(Lock_t));
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  memcpy ((void *)implpriv->name, "VIEO_lock", 9);

  if (type == VLOCK_THREAD)
  {
    /* setup thread lock */
    if (pthread_mutex_init (&implpriv->u.lock.mutex, NULL) != 0)
    {
      IB_LOG_ERROR0 ("Unable to initialize lock");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

    handle->type = VLOCK_THREAD;
    handle->magic = VLOCK_THREADLOCK_MAGIC;

    if (state == VLOCK_LOCKED)
    {
      (void)pthread_mutex_lock (&implpriv->u.lock.mutex);
      handle->status = VLOCK_LOCKED;
    }
    else
    {
      handle->status = VLOCK_FREE;
    }
  }
  else if (type == VLOCK_RWTHREAD)
  {
    /* setup RW thread lock */
    if (pthread_rwlock_init (&implpriv->u.rwlock.rwlock, NULL) != 0)
    {
      IB_LOG_ERROR0 ("Unable to initialize lock");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }
    handle->status = VLOCK_FREE;
    AtomicWrite(&implpriv->u.rwlock.reader_count, 0);

    if (state == VLOCK_LOCKED)
    {
      pthread_rwlock_wrlock(&implpriv->u.rwlock.rwlock);
      handle->status = VLOCK_LOCKED;
    }

    handle->type = VLOCK_RWTHREAD;
    handle->magic = VLOCK_RWTHREADLOCK_MAGIC;
  }
  else
  {
    /* setup spinlock */
    if (pthread_mutex_init (&implpriv->u.spin.mutex, NULL) != 0)
    {
      IB_LOG_ERROR0 ("Unable to initialize lock");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

    (void)pthread_mutex_lock (&implpriv->u.spin.mutex);
    (void)pthread_cond_init (&implpriv->u.spin.cond, NULL);
    handle->status = VLOCK_FREE;

    /* initialize count and owner thread */
    implpriv->u.spin.spin_count = 0;
    implpriv->u.spin.owner_thread = (uint64_t)0ULL;

    if (state == VLOCK_LOCKED)
    {
      /* set the lock to show in-use */
      implpriv->u.spin.spin_count++;
      handle->status = VLOCK_LOCKED;

      /* set the thread name */
      rc = vs_thread_name (&implpriv->u.spin.owner_thread);
      if (rc != VSTATUS_OK)
      {
        IB_LOG_ERROR0 ("Unable to set owner_thread");
      }
      else
      {
        //IB_LOG_INFO ("owner_thread", (uint32_t)implpriv->owner_thread);
      }
    }
    //IB_LOG_INFO ("implpriv->spin_count =", implpriv->spin_count);

    handle->type = VLOCK_SPIN;
    handle->magic = VLOCK_SPINLOCK_MAGIC;
    (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
  }

  //IB_LOG_INFO ("handle->type =", handle->type);
  //IB_LOG_INFO ("handle->status =", handle->status);
  //IB_LOG_INFO ("handle->magic =", handle->magic);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_lock_delete
*
* DESCRIPTION
*    Delete a lock previously initialized with vs_lock_init().
*
* INPUTS
*    handle   - A pointer to the lock control block filled in by
*               the vs_lock_init() call.
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*
**********************************************************************/
Status_t
vs_lock_delete (Lock_t *handle)
{
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_lock_delete";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type != VLOCK_THREAD && handle->type != VLOCK_RWTHREAD &&
      handle->type != VLOCK_SPIN)
  {
    IB_LOG_INFINI_INFO ("Lock type is not thread lock or spinlock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_THREADLOCK_MAGIC &&
      handle->magic != VLOCK_RWTHREADLOCK_MAGIC &&
      handle->magic != VLOCK_SPINLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Delete/Invalidate the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  if (handle->type == VLOCK_RWTHREAD) {
    // getting the lock here is a bit of a hack to limit shutdown sequencing
    // problems
    vs_wrlock(handle);
    handle->type = 0xFFFFFFFF;
    handle->status = 0xFFFFFFFF;
    handle->magic = 0;
    // pthread_rwlock_destroy (&implpriv->u.rwlock.rwlock);	// TBD paranoia
  } else if (handle->type == VLOCK_THREAD) {
    (void)pthread_mutex_lock (&implpriv->u.lock.mutex);
    handle->type = 0xFFFFFFFF;
    handle->status = 0xFFFFFFFF;
    handle->magic = 0;
    (void)pthread_mutex_unlock (&implpriv->u.lock.mutex);
    /* pthread_mutex_destroy (&implpriv->u.lock.mutex); tmp */
  } else {    // SPIN
    (void)pthread_mutex_lock (&implpriv->u.spin.mutex);
    handle->type = 0xFFFFFFFF;
    handle->status = 0xFFFFFFFF;
    handle->magic = 0;

    /*
    ** Awaken all sleeping threads
    */
    //IB_LOG_INFO0 ("waking threads waiting on lock");
    (void)pthread_cond_broadcast (&implpriv->u.spin.cond);

    (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
    /* pthread_mutex_destroy (&implpriv->u.spin.mutex); tmp */
  }

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_lock
*
* DESCRIPTION
*    Acquire a thread lock.
*
* INPUTS
*    handle   - A pointer to the thread lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
* NOTES
*    * handle->status is only maintained for debug asserts, as we
*      rely directly on pthreads for locking state.
*    * vs_lock_delete can not call pthread_mutex_destroy, as the mutex
*      is intended to remain valid in order to handle racing locks and
*      deletes (themselves a sign that we need to cleanup shutdown
*      logic, as this case should be undefined).
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*   MGR     03/21/02    Added fix for bug, where only one thread waiting
*                       on the thread lock was woken when lock was deleted.
*
**********************************************************************/
Status_t
vs_lock (Lock_t *handle)
{
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_lock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type == VLOCK_RWTHREAD)
    return vs_wrlock(handle);    // safety valve in case wrong call used

  if (handle->type != VLOCK_THREAD)
  {
    IB_LOG_ERROR ("Lock type is not thread lock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_THREADLOCK_MAGIC)
  {
    IB_LOG_ERRORX("Lock does not exist magic:", handle->magic);
    IB_EXIT(function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Lock the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  (void)pthread_mutex_lock (&implpriv->u.lock.mutex);

  if (handle->magic != VLOCK_THREADLOCK_MAGIC)
  {
    (void)pthread_mutex_unlock(&implpriv->u.lock.mutex);
    IB_LOG_ERRORX("Lock deleted during acquisition attempt magic:", handle->magic);
    IB_EXIT(function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  DEBUG_ASSERT(handle->status == VLOCK_FREE);
  handle->status = VLOCK_LOCKED;

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_unlock
*
* DESCRIPTION
*    Release the thread lock associated with the handle.
*
* INPUTS
*    handle   - A pointer to a thread lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*
**********************************************************************/
Status_t
vs_unlock (Lock_t *handle)
{
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_unlock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type == VLOCK_RWTHREAD)
    return vs_rwunlock(handle);    // safety valve in case wrong call used

  if (handle->type != VLOCK_THREAD)
  {
    IB_LOG_ERROR ("Lock type is not thread lock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_THREADLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Unlock the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  DEBUG_ASSERT(handle->status == VLOCK_LOCKED);
  handle->status = VLOCK_FREE;
  (void)pthread_mutex_unlock (&implpriv->u.lock.mutex);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_wrlock
*
* DESCRIPTION
*    Acquire a rw thread lock.
*
* INPUTS
*    handle   - A pointer to the rw thread lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
* HISTORY
*
*   NAME    DATE        REMARKS
*
**********************************************************************/
Status_t
vs_wrlock (Lock_t *handle)
{
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_wrlock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type != VLOCK_RWTHREAD)
  {
    IB_LOG_ERROR ("Lock type is not rw thread lock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_RWTHREADLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Lock the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  pthread_rwlock_wrlock(&implpriv->u.rwlock.rwlock);
  handle->status = VLOCK_LOCKED;
  DEBUG_ASSERT(0 == AtomicRead(&implpriv->u.rwlock.reader_count));

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_rdlock
*
* DESCRIPTION
*    Acquire a rw thread lock.
*
* INPUTS
*    handle   - A pointer to the rw thread lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
* HISTORY
*
*   NAME    DATE        REMARKS
*
**********************************************************************/
Status_t
vs_rdlock (Lock_t *handle)
{
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_rdlock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type != VLOCK_RWTHREAD)
  {
    IB_LOG_ERROR ("Lock type is not rw thread lock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_RWTHREADLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Lock the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  pthread_rwlock_rdlock(&implpriv->u.rwlock.rwlock);
  AtomicIncrementVoid(&implpriv->u.rwlock.reader_count);
  DEBUG_ASSERT(handle->status != VLOCK_LOCKED);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_rwunlock
*
* DESCRIPTION
*    Release the rw thread lock associated with the handle.
*
* INPUTS
*    handle   - A pointer to a rw thread lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*
**********************************************************************/
Status_t
vs_rwunlock (Lock_t *handle)
{
  Implpriv_Lock_t   *implpriv;
  static const char function[] = "vs_rwunlock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type != VLOCK_RWTHREAD)
  {
    IB_LOG_ERROR ("Lock type is not rw thread lock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_RWTHREADLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Unlock the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  if (handle->status == VLOCK_FREE) {
     // we must have a rdlock
     DEBUG_ASSERT(0 != AtomicRead(&implpriv->u.rwlock.reader_count));
     AtomicDecrementVoid(&implpriv->u.rwlock.reader_count);
     (void)pthread_rwlock_unlock (&implpriv->u.rwlock.rwlock);
  } else {
     // we must have a wrlock
     DEBUG_ASSERT(0 == AtomicRead(&implpriv->u.rwlock.reader_count));
     handle->status = VLOCK_FREE;
     (void)pthread_rwlock_unlock (&implpriv->u.rwlock.rwlock);
  }

  //IB_LOG_INFO ("handle->status =", handle->status);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

// TBD - this is nice, supports recursive locks
// but its not implemented for vxWorks and is not used
// perhaps delete.
// Also thread_mutex allows a PTHREAD_MUTEX_RECURSIVE option
/**********************************************************************
*
* FUNCTION
*    vs_spinlock
*
* DESCRIPTION
*    Acquire a spin lock.
*
* INPUTS
*    handle   - A pointer to the spin lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*   MGR     03/13/02    Added recursion support.
*   MGR     04/10/02    PR1716.  Block until spinlock is freed by owner,
*                       either via vs_spinunlock or vs_lock_delete.
*
**********************************************************************/
Status_t
vs_spinlock (Lock_t *handle)
{
  uint32_t          wake_me_up = 0;
  Status_t          rc;
  Implpriv_Lock_t   *implpriv;
  Threadname_t  current_thread_name;
  static const char function[] = "vs_spinlock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type != VLOCK_SPIN)
  {
    IB_LOG_ERROR ("Lock type is not spinlock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_SPINLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  /*
  ** Lock the lock
  */
  implpriv = (Implpriv_Lock_t *)handle->opaque;
  (void)pthread_mutex_lock (&implpriv->u.spin.mutex);

  if (handle->status == VLOCK_LOCKED)
  {
    //IB_LOG_INFO ("lock is currently locked", handle->status);

    /*
    ** Get current thread name
    */
    rc = vs_thread_name (&current_thread_name);
    if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("vs_thread_name error rc:", rc);
      (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
      IB_EXIT (function, VSTATUS_NXIO);
      return VSTATUS_NXIO;
    }

    //IB_LOG_INFO ("thread owning lock", (uint32_t)implpriv->u.spin.owner_thread);
    //IB_LOG_INFO ("current thread", (uint32_t)current_thread_name);

    if (implpriv->u.spin.owner_thread == current_thread_name)
    {
      /* we are the owners of the lock, update counter */
      implpriv->u.spin.spin_count++;
      //IB_LOG_INFO ("current thread owns lock", (uint32_t)current_thread_name);
      //IB_LOG_INFO ("implpriv->spin_count =", implpriv->u.spin.spin_count);
      //IB_LOG_INFO ("handle->status", handle->status);

      (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
      IB_EXIT (function, VSTATUS_OK);
      return VSTATUS_OK;
    }
    else
    {
      /* wait until we are woken, either by vs_lock_delete or vs_spinunlock */
      while (handle->status == VLOCK_LOCKED)
      {
        //IB_LOG_INFO0 ("lock is locked and we are not owner; going to sleep");
        wake_me_up = (uint32_t)1U;
        (void)pthread_cond_wait (&implpriv->u.spin.cond, &implpriv->u.spin.mutex);
      }
    }
  }

  if (wake_me_up == (uint32_t)1U)
  {
    //IB_LOG_INFO0 ("thread has woken up");
  }

  if (handle->magic == VLOCK_SPINLOCK_MAGIC)
  {
    //IB_LOG_INFO ("lock has been freed, locking", handle->status);

    handle->status = VLOCK_LOCKED;
    implpriv->u.spin.spin_count = 1;
    //IB_LOG_INFO ("implpriv->spin_count =", implpriv->u.spin.spin_count);

    /* set the thread name */
    rc = vs_thread_name (&implpriv->u.spin.owner_thread);
    if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR0 ("Unable to set owner_thread");
    }
    else
    {
      //IB_LOG_INFO ("setting owner_thread", (uint32_t)implpriv->u.spin.owner_thread);
    }
  }
  else
  {
    (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
    IB_LOG_ERROR0 ("Lock no longer exists");
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  //IB_LOG_INFO ("handle->status", handle->status);
  (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_spinunlock
*
* DESCRIPTION
*    Release the spin lock associated with the handle.
*
* INPUTS
*    handle   - A pointer to the spin lock control object filled in
*               by vs_lock_init().
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/01/02    Initial creation of function.
*   MGR     03/13/02    Added recursion support.
*   MGR     04/10/02    PR1716.  Wake blocking thread after unwinding.
*
**********************************************************************/
Status_t
vs_spinunlock (Lock_t *handle)
{
  Status_t          rc;
  Implpriv_Lock_t   *implpriv;
  Threadname_t  current_thread_name;
  static const char function[] = "vs_spinunlock";

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == NULL)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->type != VLOCK_SPIN)
  {
    IB_LOG_ERROR ("Lock type is not spinlock type:", handle->type);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != VLOCK_SPINLOCK_MAGIC)
  {
    IB_LOG_ERRORX ("Lock does not exist magic:", handle->magic);
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  implpriv = (Implpriv_Lock_t *)handle->opaque;
  (void)pthread_mutex_lock (&implpriv->u.spin.mutex);

  if (implpriv->u.spin.spin_count < 1)
  {
    /* lock is already free */
    (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
    IB_LOG_ERROR0 ("Attempting to unlock already free lock");
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  /* we can only unlock if we are the owner */
  //IB_LOG_INFO0 ("lock is locked");
  rc = vs_thread_name (&current_thread_name);
  if (rc != VSTATUS_OK)
  {
    (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
    IB_LOG_ERRORRC("vs_thread_name error rc:", rc);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  //IB_LOG_INFO ("thread owning lock", (uint32_t)implpriv->u.spin.owner_thread);
  //IB_LOG_INFO ("current thread", (uint32_t)current_thread_name);

  if (implpriv->u.spin.owner_thread != current_thread_name)
  {
    (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);
    IB_LOG_ERRORX ("current thread does not own lock. owner:",
                  (uint32_t)implpriv->u.spin.owner_thread);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

  /* decrement counter */
  //IB_LOG_INFO ("current thread owns lock", (uint32_t)current_thread_name);
  implpriv->u.spin.spin_count--;
  //IB_LOG_INFO ("implpriv->spin_count =", implpriv->u.spin.spin_count);

  if (!implpriv->u.spin.spin_count)
  {
    /* lock has completely un-nested */
    //IB_LOG_INFO0 ("lock has completed un-nested");
    handle->status = VLOCK_FREE;
    implpriv->u.spin.owner_thread = (uint64_t)0ULL;

    /* awaken a sleeping thread */
    (void)pthread_cond_signal (&implpriv->u.spin.cond);
  }

  //IB_LOG_INFO ("handle->status =", handle->status);
  (void)pthread_mutex_unlock (&implpriv->u.spin.mutex);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}
