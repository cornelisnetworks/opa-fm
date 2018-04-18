/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

/***********************************************************************
* 
* FILE NAME
*      vs_lock_test.c
*
* DESCRIPTION
*      This file contains vs_lock common test routines.  These tests
*      exercise the vs_lock services.
*
* DATA STRUCTURES
*
* FUNCTIONS
*
* DEPENDENCIES
*
*
* HISTORY
*
* NAME      DATE        REMARKS
* MGR       02/26/02    Initial creation of file.
* PJG       03/07/02    Change all compares against NULL to compares
*                         against (type *) 0 due to ATI environment.
* PJG       03/08/02    Increase the maxtime value passed to dothread_lock
*                         to 20 seconds from 4 seconds.
* PJG       04/02/02    Update to version 2.0g of vs_thread_sleep.
* MGR       04/10/02    PR1716.  Added tests cases to test blocking
*                       of spinlock.
* MGR       04/11/02    Replaced returns in thread functions with
*                       vs_thread_exit.
* MGR       04/17/02    PR1728.  Added test case to test blocking of
*                       thread, when thread waiting on lock is killed.
*
***********************************************************************/
#include <cs_g.h>
#include <vs_g.h>

/**********************************************************************
* Local Macros
***********************************************************************/
#define DOATEST(func, pass, fail) ((func)() == VSTATUS_OK) ? pass++ : fail++

/**********************************************************************
* Defines
***********************************************************************/
#define HOLD_ONE_SECOND    ((uint64_t)1000000U)
#define MAX_WAIT_TIME      ((uint64_t)0000010U)
#define WAIT_CUSHION       ((uint64_t)1000000U)
#define NUM_TEST_THREADS         ((uint32_t)3U)
#define MAX_STACK_SIZE         ((uint32_t)4096)
#define VLOCK_INV_STATE        ((uint32_t)101U)
#define VLOCK_INV_TYPE         ((uint32_t)103U)

/**********************************************************************
* Variable Declarations
***********************************************************************/
static Lock_t     *Glock;
static Thread_t   Gthandle[NUM_TEST_THREADS];
static uint64_t       Gholdtime[NUM_TEST_THREADS];
static uint64_t       Gmintime[NUM_TEST_THREADS];
static uint64_t       Gmaxtime[NUM_TEST_THREADS];
static uint8_t        Gthreadcntr = 0;
static uint32_t       Gnxio_errs = 0;
static uint32_t       Gagain_errs = 0;
uint32_t              Gtiming_errs = 0;
uint32_t              Gspinlock_errs = 0;

static Status_t
dothread_lock (uint32_t thread_idx,
               Lock_t *lock,
               uint64_t holdtime,
               uint64_t mintime,
               uint64_t maxtime,
               void (*function) (uint32_t argc, uint8_t *argv[]))
{
  Status_t         rc;
  uint8_t          name[VS_NAME_MAX];
  static uint8_t   *argv[] = {(uint8_t*)"1", (uint8_t*)"2", (uint8_t*)"3"};

  /*
  ** Setup global variables
  */
  Glock = lock;
  Gholdtime[thread_idx] = holdtime;
  Gmintime[thread_idx] = mintime;
  Gmaxtime[thread_idx] = maxtime;

  /*
  ** Create/display the thread name, update thread counter
  */
  memcpy((void *)name, "VIEOLckThrd", 11);
  if (Gthreadcntr == 0)
  {
    name[11] = 0x30U;
    name[12] = 0x30U;
  }
  else
  {
    name[11] = ((uint8_t)Gthreadcntr / (uint8_t)10U) + 0x30;
    name[12] = ((uint8_t)Gthreadcntr % (uint8_t)10U) + 0x30;
  }
  name[13] = (uint8_t)0U;
  name[14] = (uint8_t)0U;
  IB_LOG_INFO0 (Log_StrDup(name));  /* debug */
  Gthreadcntr++;

  /*
  ** For testing purposes only, the thread index is passed
  ** to vs_thread_create in the argc parameter.
  */
  rc = vs_thread_create (&Gthandle[thread_idx],
                             name,
                             function,
                             thread_idx,
                             argv,
                             MAX_STACK_SIZE);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_thread_create failed error code", rc);
    return VSTATUS_BAD;
  }

  return VSTATUS_OK;
}

static void
lock_and_free (uint32_t argc,
               uint8_t *argv[])
{
  Status_t rc;
  uint64_t starttime = (uint64_t)0U;
  uint64_t stoptime = (uint64_t)0U;
  uint64_t deltatime = (uint64_t)0U;

  /* get time of the lock attempt */
  IB_LOG_INFO ("getting starttime", argc);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* attempt to obtain the lock */
  IB_LOG_INFO ("attempting to get lock", argc);  /* debug */
  rc = vs_lock (Glock);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_lock error", rc);

    if (rc == VSTATUS_NXIO)
    {
      Gnxio_errs++;
    }
    else if (rc == VSTATUS_AGAIN)
    {
      IB_LOG_ERROR ("thread was killed", argc);  /* debug */
      Gagain_errs++;
    }

    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* get time of the lock completion */
  IB_LOG_INFO ("getting stoptime", argc);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* determine if thread blocked */
  deltatime = stoptime - starttime;
/* +++debug */
IB_LOG_INFO ("thread index =", (uint32_t)argc);
IB_LOG_INFO ("deltatime =", (uint32_t)deltatime);
IB_LOG_INFO ("Gmaxtime =", (uint32_t)Gmaxtime[argc]);
IB_LOG_INFO ("Gmintime =", (uint32_t)Gmintime[argc]);
IB_LOG_INFO ("Gholdtime", (uint32_t)Gholdtime[argc]);
/* ---debug */
  if (deltatime > Gmaxtime[argc])
  {
    IB_LOG_INFO ("thread index", argc);
    IB_LOG_ERROR ("waittime too long delta", (uint32_t)deltatime);
    IB_LOG_ERROR ("waittime too long maxtime", (uint32_t)Gmaxtime[argc]);
    Gtiming_errs++;
    IB_LOG_INFO ("freeing lock", argc);
    (void)vs_unlock (Glock);
    (void)vs_thread_exit (&Gthandle[argc]);
  }
  else if (deltatime < Gmintime[argc])
  {
    IB_LOG_INFO ("thread index", argc);
    IB_LOG_ERROR ("waittime too short delta", (uint32_t)deltatime);
    IB_LOG_ERROR ("waittime too short mintime", (uint32_t)Gmintime[argc]);
    Gtiming_errs++;
    IB_LOG_INFO ("freeing lock", argc);
    (void)vs_unlock (Glock);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* hold the lock, if requested */
  if (Gholdtime[argc] != 0)
  {
    IB_LOG_INFO ("holding the lock", argc);  /* debug */
    vs_thread_sleep ((uint64_t)Gholdtime[argc]);
  }

  /* free the lock for the next thread */
  IB_LOG_INFO ("freeing the lock", argc);  /* debug */
  rc = vs_unlock (Glock);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_unlock error", rc);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  (void)vs_thread_exit (&Gthandle[argc]);
}

static void
spinlock_and_free (uint32_t argc,
                   uint8_t *argv[])
{
  Status_t rc;
  uint64_t starttime = (uint64_t)0U;
  uint64_t stoptime = (uint64_t)0U;
  uint64_t deltatime = (uint64_t)0U;

  /* get time of the lock attempt */
  IB_LOG_INFO ("getting starttime", argc);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* attempt to obtain the spinlock */
  IB_LOG_INFO ("attempting to get lock", argc);  /* debug */
  rc = vs_spinlock (Glock);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_spinlock error", rc);
    if (rc == VSTATUS_NXIO)
      Gnxio_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* get time of the lock completion */
  IB_LOG_INFO ("getting stoptime", argc);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* determine if thread blocked */
  deltatime = stoptime - starttime;
/* +++debug */
  IB_LOG_INFO ("thread index =", (uint32_t)argc);
  IB_LOG_INFO ("deltatime =", (uint32_t)deltatime);
  IB_LOG_INFO ("Gmaxtime =", (uint32_t)Gmaxtime[argc]);
  IB_LOG_INFO ("Gmintime =", (uint32_t)Gmintime[argc]);
  IB_LOG_INFO ("Gholdtime", (uint32_t)Gholdtime[argc]);
/* ---debug */
  if (deltatime > Gmaxtime[argc])
  {
    IB_LOG_INFO ("thread index", (uint32_t)argc);
    IB_LOG_ERROR ("waittime too long delta", (uint32_t)deltatime);
    IB_LOG_ERROR ("waittime too long maxtime", (uint32_t)Gmaxtime[argc]);
    Gtiming_errs++;
    IB_LOG_INFO ("freeing lock", argc);
    (void)vs_spinunlock (Glock);
    (void)vs_thread_exit (&Gthandle[argc]);
  }
  else if (deltatime < Gmintime[argc])
  {
    IB_LOG_INFO ("thread index", (uint32_t)argc);
    IB_LOG_ERROR ("waittime too short delta", (uint32_t)deltatime);
    IB_LOG_ERROR ("waittime too short mintime", (uint32_t)Gmintime[argc]);
    Gtiming_errs++;
    IB_LOG_INFO ("freeing lock", argc);
    (void)vs_spinunlock (Glock);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* hold the lock, if requested */
  if (Gholdtime[argc] != 0)
  {
    IB_LOG_INFO ("holding the lock", argc);  /* debug */
    vs_thread_sleep ((uint64_t)Gholdtime[argc]);
  }

  /* free the lock for the next thread */
  IB_LOG_INFO ("freeing the lock", argc);  /* debug */
  rc = vs_spinunlock (Glock);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_spinunlock error", rc);
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  (void)vs_thread_exit (&Gthandle[argc]);
}

static void
spinlock_owner (uint32_t argc,
                uint8_t *argv[])
{
  Status_t rc;
  uint64_t starttime = (uint64_t)0U;
  uint64_t stoptime = (uint64_t)0U;
  uint64_t deltatime = (uint64_t)0U;

  /* 1st attempt to obtain the spinlock */
  IB_LOG_INFO ("1st attempt to get lock", argc);  /* debug */
  rc = vs_spinlock (Glock);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_spinlock error", rc);
    Gspinlock_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* get time of the 2nd lock attempt */
  IB_LOG_INFO ("getting starttime", argc);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    Gtiming_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* 2nd attempt to obtain the spinlock */
  IB_LOG_INFO ("2nd attempt to get lock", argc);  /* debug */
  rc = vs_spinlock (Glock);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_spinlock error", rc);
    Gspinlock_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /* get time of the 2nd lock completion */
  IB_LOG_INFO ("getting stoptime", argc);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    Gtiming_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  /*
  ** call vs_spinunlock twice to unwind the lock
  */
  IB_LOG_INFO ("1st attempt to unlock spinlock", 0);  /* debug */
  (void)vs_spinunlock (Glock);
  IB_LOG_INFO ("2nd attempt to unlock spinlock", 0);  /* debug */
  (void)vs_spinunlock (Glock);

  /* determine if thread blocked */
  deltatime = stoptime - starttime;
/* +++debug */
  IB_LOG_INFO ("thread index =", (uint32_t)argc);
  IB_LOG_INFO ("deltatime =", (uint32_t)deltatime);
  IB_LOG_INFO ("Gmaxtime =", (uint32_t)Gmaxtime[argc]);
  IB_LOG_INFO ("Gmintime =", (uint32_t)Gmintime[argc]);
  IB_LOG_INFO ("Gholdtime", (uint32_t)Gholdtime[argc]);
/* ---debug */
  if (deltatime > Gmaxtime[argc])
  {
    IB_LOG_INFO ("thread index", (uint32_t)argc);
    IB_LOG_ERROR ("waittime too long delta", (uint32_t)deltatime);
    IB_LOG_ERROR ("waittime too long maxtime", (uint32_t)Gmaxtime[argc]);
    Gtiming_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }
  else if (deltatime < Gmintime[argc])
  {
    IB_LOG_INFO ("thread index", (uint32_t)argc);
    IB_LOG_ERROR ("waittime too short delta", (uint32_t)deltatime);
    IB_LOG_ERROR ("waittime too short mintime", (uint32_t)Gmintime[argc]);
    Gtiming_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  (void)vs_thread_exit (&Gthandle[argc]);
}

static void
spinunlock_unnest (uint32_t argc,
                   uint8_t *argv[])
{
  uint32_t i;
  Status_t rc;

  /* nest three times to obtain the spinlock */
  for (i = 0; i < 3; i++)
  {
    IB_LOG_INFO ("i =", i);  /* debug */
    IB_LOG_INFO ("attempt to get lock", argc);  /* debug */
    rc = vs_spinlock (Glock);
    if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_spinlock error", rc);
      Gspinlock_errs++;
      (void)vs_thread_exit (&Gthandle[argc]);
    }
  }

  /*
  ** verify it takes three unlocks to unwind the lock
  */
  for (i = 0; i < 3; i++)
  {
    IB_LOG_INFO ("i =", i);  /* debug */
    IB_LOG_INFO ("attempt to free lock", argc);  /* debug */
    rc = vs_spinunlock (Glock);
    if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_spinunlock error", rc);
      Gspinlock_errs++;
      (void)vs_thread_exit (&Gthandle[argc]);
    }
  }

  /*
  ** verify another attempt to unlock returns in error
  */
  IB_LOG_INFO ("attempt to unlock free lock", argc);  /* debug */
  rc = vs_spinunlock (Glock);
  if (rc != VSTATUS_NXIO)
  {
    IB_LOG_ERROR ("vs_spinunlock did not return expected return code", rc);
    Gspinlock_errs++;
    (void)vs_thread_exit (&Gthandle[argc]);
  }

  (void)vs_thread_exit (&Gthandle[argc]);
}

/*
** Sub-test Variations (vs_lock_init)
*/
static Status_t
vs_lock_init_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "vs_lock_init:1:1.a PASSED";
  static const char failed[] = "vs_lock_init:1:1.a FAILED";

  /* attempt to create a thread lock or spinlock with a NULL lock pointer */
  p = (void *) 0;
  rc = vs_lock_init ((Lock_t *)(p), VLOCK_FREE, VLOCK_THREAD);
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_init_2a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:2.a PASSED";
  static const char failed[] = "vs_lock_init:1:2.a FAILED";

  /* attempt to create a thread lock or spinlock with invalid initial state */
  rc = vs_lock_init (&lock, VLOCK_INV_STATE, VLOCK_SPIN);
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_init_3a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:3.a PASSED";
  static const char failed[] = "vs_lock_init:1:3.a FAILED";

  /* attempt to create a thread lock or spinlock with an invalid lock type */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_INV_TYPE);
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_init_4a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:4.a PASSED";
  static const char failed[] = "vs_lock_init:1:4.a FAILED";

  /* attempt to create a thread lock or spinlock with a valid state and type */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  /* delete the lock */
  (void)vs_lock_delete (&lock);

  return rc;
}

static Status_t
vs_lock_init_4b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time = (uint64_t)0U;
  uint64_t          hold_time = (uint64_t)0U;
  uint64_t          min_time = (uint64_t)0U;
  uint64_t          max_time = (uint64_t)1000U;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:4.b PASSED";
  static const char failed[] = "vs_lock_init:1:4.b FAILED";

  /* create a thread lock or spinlock in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /*
    ** verify that a sequence of threads will not block,
    ** when attempting to serially obtain/free the lock
    */
    for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
    {
      rc = dothread_lock (i,
                          &lock,
                          hold_time,
                          min_time,
                          max_time,
                          spinlock_and_free);
      if (rc != VSTATUS_OK)
      {
        break;
      }

      sleep_time = HOLD_ONE_SECOND;
      vs_thread_sleep (sleep_time);
    }

    if (rc == VSTATUS_OK)
    {
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
    else
    {
      rc = VSTATUS_BAD;
      IB_LOG_ERROR (failed, (uint32_t)0U);
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_init_4c (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time = (uint32_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:4.c PASSED";
  static const char failed[] = "vs_lock_init:1:4.c FAILED";

  /* initialize thread timing errors */
  Gtiming_errs = 0;

  /* create a thread lock in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* create sequence of threads that attempt to obtain/free the lock */
    for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
    {
      IB_LOG_INFO ("creating thread", i);  /* debug */
      if (i == 0U)
      {
        /* the first thread should obtain the lock immediately */
        rc = dothread_lock (i,
                            &lock,
                            HOLD_ONE_SECOND,
                            (uint64_t)0U,
                            (uint64_t)3000U,
                            lock_and_free);
      }
      else
      {
        /* the other threads should block until the lock is released */
        rc = dothread_lock (i,
                            &lock,
                            HOLD_ONE_SECOND,
                            ((HOLD_ONE_SECOND * (uint64_t)i) -
                             (WAIT_CUSHION * i)),
                            ((HOLD_ONE_SECOND * ((uint64_t)i+1)) +
                             (WAIT_CUSHION * i)),
                            lock_and_free);
      }

      if (rc != VSTATUS_OK)
      {
        break;
      }

      /* sleep here so that thread start in the proper order */
      sleep_time = (uint64_t)i;
      vs_thread_sleep (sleep_time);
    }

    if (rc == VSTATUS_OK)
    {
      /* wait until the lock is free, so that all threads are finished */
      IB_LOG_INFO ("waiting for threads to complete", 0);  /* debug */
      rc = vs_lock (&lock);
      IB_LOG_INFO ("obtained lock, threads have completed", 0);  /* debug */
      if (rc == VSTATUS_OK)
      {
        IB_LOG_INFO ("freeing lock", 0);  /* debug */
        (void)vs_unlock (&lock);
        if (Gtiming_errs == 0)
        {
          IB_LOG_INFO (passed, (uint32_t)0U);
        }
        else
        {
          IB_LOG_ERROR ("lock timing errors", Gtiming_errs);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }
      }
      else
      {
        IB_LOG_ERROR ("vs_lock failed", rc);
        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      rc = VSTATUS_BAD;
      IB_LOG_ERROR (failed, (uint32_t)0U);
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_init_5a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:5.a PASSED";
  static const char failed[] = "vs_lock_init:1:5.a FAILED";

  /* attempt to create a thread lock or spinlock initially locked */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  /* delete the lock */
  (void)vs_lock_delete (&lock);

  return rc;
}

static Status_t
vs_lock_init_5b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          hold_time = (uint64_t)0U;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_lock_init:1:5.b PASSED";
  static const char failed[] = "vs_lock_init:1:5.b FAILED";

  /* initialize thread timing errors */
  Gtiming_errs = 0;

  /* create a thread lock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /*
    ** verify that a sequence of threads will be suspended,
    ** while attempting to serially obtain/free the lock
    */
    for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
    {
      IB_LOG_INFO ("creating thread", i);
      rc = dothread_lock (i,
                          &lock,
                          HOLD_ONE_SECOND,
                          ((HOLD_ONE_SECOND * (uint32_t)15U) -
                           (WAIT_CUSHION * (i+1))),
                          ((HOLD_ONE_SECOND * ((uint32_t)16U + i)) + 
                           (WAIT_CUSHION * (i+1))),
                          lock_and_free);

      if (rc != VSTATUS_OK)
      {
        break;
      }

      sleep_time = (uint64_t)i;
      vs_thread_sleep (sleep_time);
    }

    if (rc == VSTATUS_OK)
    {
      /* hold the lock to make the other threads block */
      IB_LOG_INFO ("holding lock to make threads block", 0);  /* debug */
      hold_time = HOLD_ONE_SECOND * (uint32_t)15U;
      vs_thread_sleep (hold_time);

      /* free the lock */
      IB_LOG_INFO ("freeing lock for threads", 0);  /* debug */
      rc = vs_unlock (&lock);
      if (rc == VSTATUS_OK)
      {
        /* wait until the lock is free, so that all threads are finished */
        sleep_time = HOLD_ONE_SECOND * 15U;
        vs_thread_sleep (sleep_time);
        IB_LOG_INFO ("waiting for thread to complete", 0);  /* debug */
        rc = vs_lock (&lock);
        IB_LOG_INFO ("obtained lock, threads have completed", 0);  /* debug */
        if (rc == VSTATUS_OK)
        {
          (void)vs_unlock (&lock);
          if (Gtiming_errs == 0)
          {
            IB_LOG_INFO (passed, (uint32_t)0U);
          }
          else
          {
            IB_LOG_ERROR ("lock timing errors", Gtiming_errs);
            IB_LOG_ERROR (failed, (uint32_t)0U);
            rc = VSTATUS_BAD;
          }
        }
        else
        {
          IB_LOG_ERROR ("vs_lock failed", rc);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }

        /* delete the lock */
        (void)vs_lock_delete (&lock);
      }
      else
      {
        IB_LOG_ERROR ("vs_unlock error", rc);

        /* delete the lock */
        (void)vs_lock_delete (&lock);

        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      /* free the lock */
      rc = vs_unlock (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_unlock error", rc);

      /* delete the lock */
      rc = vs_lock_delete (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_lock_delete error", rc);

      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("vs_lock_init failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

void
test_lock_init_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("vs_lock_init:1 TEST STARTED", (uint32_t)0U);

  DOATEST (vs_lock_init_1a, total_passes, total_fails);
  DOATEST (vs_lock_init_2a, total_passes, total_fails);
  DOATEST (vs_lock_init_3a, total_passes, total_fails);
  DOATEST (vs_lock_init_4a, total_passes, total_fails);
  DOATEST (vs_lock_init_4b, total_passes, total_fails);
  DOATEST (vs_lock_init_4c, total_passes, total_fails);
  DOATEST (vs_lock_init_5a, total_passes, total_fails);
  DOATEST (vs_lock_init_5b, total_passes, total_fails);

  IB_LOG_INFO ("vs_lock_init:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_lock_init:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_lock_init:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (vs_lock)
*/
static Status_t
vs_lock_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "vs_lock:1:1.a PASSED";
  static const char failed[] = "vs_lock:1:1.a FAILED";

  /* attempt to lock a NULL thread lock pointer */
  p = (void *) 0;
  rc = vs_lock ((Lock_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_lock failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_2a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_lock:1:2.a PASSED";
  static const char failed[] = "vs_lock:1:2.a FAILED";

  /* create a thread lock initially in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* attempt to obtain the lock */
    rc = vs_lock (&lock);
    if (rc == VSTATUS_OK)
    {
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
    else
    {
      IB_LOG_ERROR ("vs_lock failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_lock failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_2b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          hold_time = (uint64_t)0U;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_lock:1:2.b PASSED";
  static const char failed[] = "vs_lock:1:2.b FAILED";

  /* initialize thread timing errors */
  Gtiming_errs = 0;

  /* create a thread lock initially in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* obtain the lock */
    rc = vs_lock (&lock);
    if (rc == VSTATUS_OK)
    {
      /*
      ** Create a sequence of threads that attempt to obtain/free
      ** the lock; verify that the threads cannot obtain the lock
      ** until it is freed.
      */
      for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
      {
        IB_LOG_INFO ("creating thread", i);  /* debug */
        rc = dothread_lock (i,
                            &lock,
                            HOLD_ONE_SECOND,
                            ((HOLD_ONE_SECOND * (uint32_t)15U) -
                             (WAIT_CUSHION * (i+1))),
                            ((HOLD_ONE_SECOND * ((uint32_t)16U + i)) +
                             (WAIT_CUSHION * (i+1))),
                            lock_and_free);

        if (rc != VSTATUS_OK)
        {
          break;
        }

        /* sleep so that the threads will start in the correct order */
        sleep_time = (uint64_t)i;
        vs_thread_sleep (sleep_time);
      }

      if (rc == VSTATUS_OK)
      {
        /* hold the lock to make the other threads block */
        IB_LOG_INFO ("holding lock to make threads block", 0);  /* debug */
        hold_time = HOLD_ONE_SECOND * (uint64_t)15U;
        vs_thread_sleep (hold_time);

        /* free the lock */
        IB_LOG_INFO ("freeing lock for threads", 0);  /* debug */
        rc = vs_unlock (&lock);
        if (rc == VSTATUS_OK)
        {
          /* wait until the lock is free, so that all threads are finished */
          sleep_time = HOLD_ONE_SECOND * (uint64_t)5U;
          vs_thread_sleep (sleep_time);
          IB_LOG_INFO ("waiting for thread to complete", 0);  /* debug */
          rc = vs_lock (&lock);
          IB_LOG_INFO ("obtained lock, threads have completed", 0);  /* debug */
          if (rc == VSTATUS_OK)
          {
            (void)vs_unlock (&lock);
            if (Gtiming_errs == 0)
            {
              IB_LOG_INFO (passed, (uint32_t)0U);
            }
            else
            {
              IB_LOG_ERROR ("lock timing errors", Gtiming_errs);
              IB_LOG_ERROR (failed, (uint32_t)0U);
              rc = VSTATUS_BAD;
            }
          }
          else
          {
            IB_LOG_ERROR ("vs_lock failed", rc);
            IB_LOG_ERROR (failed, (uint32_t)0U);
            rc = VSTATUS_BAD;
          }
        }
        else
        {
          IB_LOG_ERROR ("vs_unlock error", rc);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }
      }
      else
      {
        /* free the lock */
        rc = vs_unlock (&lock);
        if (rc != VSTATUS_OK)
          IB_LOG_ERROR ("vs_unlock error", rc);

        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      IB_LOG_ERROR ("vs_lock failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_lock failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

void
test_lock_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;
  uint32_t total_skipped = (uint32_t)0U;

  IB_LOG_INFO ("vs_lock:1 TEST STARTED", (uint32_t)0U);

  DOATEST (vs_lock_1a, total_passes, total_fails);
  DOATEST (vs_lock_2a, total_passes, total_fails);
  DOATEST (vs_lock_2b, total_passes, total_fails);
  IB_LOG_INFO ("vs_lock:1:3.a SKIPPED", (uint32_t)0U);
  total_skipped++;

  IB_LOG_INFO ("vs_lock:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_lock:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_lock:1 TOTAL SKIPPED", total_skipped);
  IB_LOG_INFO ("vs_lock:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (vs_unlock)
*/
static Status_t
vs_unlock_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "vs_unlock:1:1.a PASSED";
  static const char failed[] = "vs_unlock:1:1.a FAILED";

  /* attempt to unlock a NULL thread lock pointer */
  p = (void *) 0;
  rc = vs_unlock ((Lock_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_unlock failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_unlock failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_unlock_2a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_unlock:1:2.a PASSED";
  static const char failed[] = "vs_unlock:1:2.a FAILED";

  /* create a thread lock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* attempt to free the lock */
    rc = vs_unlock (&lock);
    if (rc == VSTATUS_OK)
    {
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
    else
    {
      IB_LOG_ERROR ("vs_unlock failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_unlock failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_unlock_2b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_unlock:1:2.b PASSED";
  static const char failed[] = "vs_unlock:1:2.b FAILED";

  /* initialize thread timing errors */
  Gtiming_errs = 0;

  /* create a thread lock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* free the lock */
    rc = vs_unlock (&lock);
    if (rc == VSTATUS_OK)
    {
      /* create sequence of threads that attempt to obtain/free the lock */
      for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
      {
        IB_LOG_INFO ("creating thread", i);  /* debug */
        if (i == 0U)
        {
          /* the first thread should obtain the lock immediately */
          rc = dothread_lock (i,
                              &lock,
                              HOLD_ONE_SECOND,
                              (uint64_t)0U,
                              ((uint64_t)3000U + WAIT_CUSHION),
                              lock_and_free);
        }
        else
        {
          /* the other threads should block until the lock is released */
          rc = dothread_lock (i,
                              &lock,
                              HOLD_ONE_SECOND,
                              ((HOLD_ONE_SECOND * ((uint64_t)i)) -
                               (WAIT_CUSHION * i)),
                              ((HOLD_ONE_SECOND * ((uint64_t)i+1)) +
                               (WAIT_CUSHION * i)),
                              lock_and_free);
        }

        if (rc != VSTATUS_OK)
        {
          break;
        }

        /* sleep here so that thread start in the proper order */
        sleep_time = (uint64_t)i;
        vs_thread_sleep (sleep_time);
      }

      if (rc == VSTATUS_OK)
      {
        /* wait until the lock is free, so that all threads are finished */
        IB_LOG_INFO ("waiting for threads to complete", 0);  /* debug */
        rc = vs_lock (&lock);
        IB_LOG_INFO ("obtained lock, threads have completed", 0);  /* debug */
        if (rc == VSTATUS_OK)
        {
          IB_LOG_INFO ("freeing lock", 0);  /* debug */
          (void)vs_unlock (&lock);
          if (Gtiming_errs == 0)
          {
            IB_LOG_INFO (passed, (uint32_t)0U);
          }
          else
          {
            IB_LOG_ERROR ("lock timing errors", Gtiming_errs);
            IB_LOG_ERROR (failed, (uint32_t)0U);
            rc = VSTATUS_BAD;
          }
        }
        else
        {
          IB_LOG_ERROR ("vs_lock failed", rc);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }
      }
      else
      {
        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      IB_LOG_ERROR ("vs_unlock failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_unlock failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

void
test_unlock_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("vs_unlock:1 TEST STARTED", (uint32_t)0U);

  DOATEST (vs_unlock_1a, total_passes, total_fails);
  DOATEST (vs_unlock_2a, total_passes, total_fails);
  DOATEST (vs_unlock_2b, total_passes, total_fails);

  IB_LOG_INFO ("vs_unlock:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_unlock:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_unlock:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (vs_lock_delete)
*/
static Status_t
vs_lock_delete_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "vs_lock_delete:1:1.a PASSED";
  static const char failed[] = "vs_lock_delete:1:1.a FAILED";

  /* attempt to delete a NULL lock pointer */
  p = (void *) 0;
  rc = vs_lock_delete ((Lock_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_delete failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_lock_delete failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_delete_2a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_lock_delete:1:2.a PASSED";
  static const char failed[] = "vs_lock_delete:1:2.a FAILED";

  /* create a thread lock or spinlock */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* attempt to delete the lock */
    rc = vs_lock_delete (&lock);
    if (rc == VSTATUS_OK)
    {
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
    else
    {
      IB_LOG_ERROR ("vs_lock_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_lock_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_delete_2b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_lock_delete:1:2.b PASSED";
  static const char failed[] = "vs_lock_delete:1:2.b FAILED";

  /* create a thread lock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_THREAD);
  if (rc == VSTATUS_OK)
  {
    /* clear invalid object error counter */
    Gnxio_errs = 0;

    /* create sequence of threads that attempt to obtain/free the lock */
    for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
    {
      IB_LOG_INFO ("creating thread", i);
      rc = dothread_lock (i,
                          &lock,
                          HOLD_ONE_SECOND,
                          (uint64_t)0U,
                          ((HOLD_ONE_SECOND * (uint32_t)10U) +
                           (WAIT_CUSHION * (i+1))),
                          lock_and_free);

      if (rc != VSTATUS_OK)
      {
        break;
      }

      sleep_time = HOLD_ONE_SECOND;
      vs_thread_sleep (sleep_time);
    }

    if (rc == VSTATUS_OK)
    {
      vs_thread_sleep (sleep_time);

      /* attempt to delete the lock, while we have it locked */
      IB_LOG_INFO ("attempting to delete lock", 0);
      rc = vs_lock_delete (&lock);
      if (rc == VSTATUS_OK)
      {
        /* give sleeping threads time to be woken */
        IB_LOG_INFO ("lock deleted, waiting for threads to be woken", 0);
        sleep_time = HOLD_ONE_SECOND * (uint64_t)10;
        vs_thread_sleep (sleep_time);

        /*
        ** verify other threads that are waiting for this lock
        ** are woken with a VSTATUS_NXIO return code status
        */
        if (Gnxio_errs == NUM_TEST_THREADS)
        {
          IB_LOG_INFO (passed, (uint32_t)0U);
        }
        else
        {
          IB_LOG_ERROR ("vs_lock_delete failed to wakeup sleeping threads", 
                        Gnxio_errs);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }
      }
      else
      {
        IB_LOG_ERROR ("vs_lock_delete failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_lock_delete failed; actual", rc);
        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      /* free the lock */
      rc = vs_unlock (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_unlock error", rc);

      /* delete the lock */
      rc = vs_lock_delete (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_unlock error", rc);

      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_lock_delete_2c (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_lock_delete:1:2.c PASSED";
  static const char failed[] = "vs_lock_delete:1:2.c FAILED";

  /* create a spin lock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /* clear invalid object error counter */
    Gnxio_errs = 0;

    /* create sequence of threads that attempt to obtain/free the lock */
    for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
    {
      IB_LOG_INFO ("creating thread", i);
      rc = dothread_lock (i,
                          &lock,
                          HOLD_ONE_SECOND,
                          (uint64_t)0U,
                          ((HOLD_ONE_SECOND * (uint32_t)10U) +
                           (WAIT_CUSHION * (i+1))),
                          spinlock_and_free);

      if (rc != VSTATUS_OK)
      {
        break;
      }

      sleep_time = HOLD_ONE_SECOND;
      vs_thread_sleep (sleep_time);
    }

    if (rc == VSTATUS_OK)
    {
      vs_thread_sleep (sleep_time);

      /* attempt to delete the lock, while we have it locked */
      IB_LOG_INFO ("attempting to delete lock", 0);
      rc = vs_lock_delete (&lock);
      if (rc == VSTATUS_OK)
      {
        /* give sleeping threads time to be woken */
        IB_LOG_INFO ("lock deleted, waiting for threads to be woken", 0);
        sleep_time = HOLD_ONE_SECOND * (uint64_t)10;
        vs_thread_sleep (sleep_time);

        /*
        ** verify other threads that are waiting for this lock
        ** are woken with a VSTATUS_NXIO return code status
        */
        if (Gnxio_errs == NUM_TEST_THREADS)
        {
          IB_LOG_INFO (passed, (uint32_t)0U);
        }
        else
        {
          IB_LOG_ERROR ("vs_lock_delete failed to wakeup sleeping threads", 
                        Gnxio_errs);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }
      }
      else
      {
        IB_LOG_ERROR ("vs_lock_delete failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_lock_delete failed; actual", rc);
        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      /* free the lock */
      rc = vs_spinunlock (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_spinunlock error", rc);

      /* delete the lock */
      rc = vs_lock_delete (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_lock_delete error", rc);

      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

void
test_lock_delete_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;
  uint32_t total_skipped = (uint32_t)0U;

  IB_LOG_INFO ("vs_lock_delete:1 TEST STARTED", (uint32_t)0U);

  DOATEST (vs_lock_delete_1a, total_passes, total_fails);
  DOATEST (vs_lock_delete_2a, total_passes, total_fails);
  DOATEST (vs_lock_delete_2b, total_passes, total_fails);
  DOATEST (vs_lock_delete_2c, total_passes, total_fails);

  IB_LOG_INFO ("vs_lock_delete:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_lock_delete:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_lock_delete:1 TOTAL SKIPPED", total_skipped);
  IB_LOG_INFO ("vs_lock_delete:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (vs_spinlock)
*/
static Status_t
vs_spinlock_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "vs_spinlock:1:1.a PASSED";
  static const char failed[] = "vs_spinlock:1:1.a FAILED";

  /* attempt to lock a NULL spinlock pointer */
  p = (void *) 0;
  rc = vs_spinlock ((Lock_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_spinlock failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_spinlock failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_spinlock_2a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_spinlock:1:2.a PASSED";
  static const char failed[] = "vs_spinlock:1:2.a FAILED";

  /* create a spinlock initially in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /* attempt to obtain the lock */
    IB_LOG_INFO ("attempting to obtain lock", 0);  /* debug */
    rc = vs_spinlock (&lock);
    if (rc == VSTATUS_OK)
    {
      IB_LOG_INFO ("freeing the lock", 0);  /* debug */
      rc = vs_spinunlock (&lock);
      if (rc == VSTATUS_OK)
      {
        IB_LOG_INFO (passed, (uint32_t)0U);
      }
      else
      {
	  IB_LOG_ERROR ("vs_spinunlock error", rc);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
      }
    }
    else
    {
      IB_LOG_ERROR ("vs_spinlock failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_spinlock failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    IB_LOG_INFO ("deleting lock", 0);  /* debug */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_spinlock_2b (void)
{
  Status_t          rc;
  uint64_t          hold_time = (uint64_t)0U;
  uint64_t          min_time = (uint64_t)0U;
  uint64_t          max_time = (uint64_t)1000U;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_spinlock:1:2.b PASSED";
  static const char failed[] = "vs_spinlock:1:2.b FAILED";

  /* initialize error counters */
  Gtiming_errs = 0;
  Gspinlock_errs = 0;

  /* create a spinlock initially in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /* create a thread which will make two calls to vs_spinlock */
    rc = dothread_lock (0,
                        &lock,
                        hold_time,
                        min_time,
                        max_time,
                        spinlock_owner);

    if (rc != VSTATUS_OK)
    {
      rc = VSTATUS_BAD;
      IB_LOG_ERROR (failed, (uint32_t)0U);
      return rc;
    }

    /* allow the thread to complete */
    sleep_time = HOLD_ONE_SECOND * (uint64_t)3;
    IB_LOG_INFO ("sleeping to allow threads to complete", 0);  /* debug */
    vs_thread_sleep (sleep_time);

    /* delete the spinlock */
    (void) vs_lock_delete (&lock);

    /* if the 2nd vs_spinlock call didn't complete immediately */
    if ((Gtiming_errs != 0) && (Gspinlock_errs != 0))
    {
      IB_LOG_ERROR ("timing errors", Gtiming_errs);
      IB_LOG_ERROR ("spinlock errors", Gspinlock_errs);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      return VSTATUS_BAD;
    }
    else
    {
      rc = VSTATUS_OK;
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_spinlock_2c (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          hold_time = (uint64_t)0U;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_spinlock:1:2.c PASSED";
  static const char failed[] = "vs_spinlock:1:2.c FAILED";

  /* initialize thread timing errors */
  Gtiming_errs = 0;

  /* create a spinlock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /*
    ** Create a sequence of threads that attempt to obtain/free
    ** the lock; verify that the threads cannot obtain the lock
    ** until it is freed.
    */
    for (i = (uint32_t)0U; i < NUM_TEST_THREADS; i++)
    {
      IB_LOG_INFO ("creating thread", i);  /* debug */
      rc = dothread_lock (i,
                          &lock,
                          HOLD_ONE_SECOND,
                          ((HOLD_ONE_SECOND * (uint32_t)15U) -
                           (WAIT_CUSHION * (i+1))),
                          ((HOLD_ONE_SECOND * ((uint32_t)16U + i)) +
                           (WAIT_CUSHION * (i+1))),
                          spinlock_and_free);

      if (rc != VSTATUS_OK)
      {
        break;
      }

      /* sleep so that the threads will start in the correct order */
      sleep_time = (uint64_t)i;
      vs_thread_sleep (sleep_time);
    }

    if (rc == VSTATUS_OK)
    {
      /* hold the lock to make the other threads block */
      IB_LOG_INFO ("holding lock to make threads block", 0);  /* debug */
      hold_time = HOLD_ONE_SECOND * (uint64_t)15U;
      vs_thread_sleep (hold_time);

      /* free the lock */
      IB_LOG_INFO ("freeing lock for threads", 0);  /* debug */
      rc = vs_spinunlock (&lock);
      if (rc == VSTATUS_OK)
      {
        /* wait until the lock is free, so that all threads are finished */
        sleep_time = HOLD_ONE_SECOND * (uint64_t)5U;
        vs_thread_sleep (sleep_time);
        IB_LOG_INFO ("waiting for thread to complete", 0);  /* debug */
        rc = vs_spinlock (&lock);
        IB_LOG_INFO ("obtained lock, threads have completed", 0);  /* debug */
        if (rc == VSTATUS_OK)
        {
          (void)vs_spinunlock (&lock);
          if (Gtiming_errs == 0)
          {
            IB_LOG_INFO (passed, (uint32_t)0U);
          }
          else
          {
            IB_LOG_ERROR ("lock timing errors", Gtiming_errs);
            IB_LOG_ERROR (failed, (uint32_t)0U);
            rc = VSTATUS_BAD;
          }
        }
        else
        {
          IB_LOG_ERROR ("vs_spinlock failed", rc);
          IB_LOG_ERROR (failed, (uint32_t)0U);
          rc = VSTATUS_BAD;
        }
      }
      else
      {
        IB_LOG_ERROR ("vs_spinunlock error", rc);
        IB_LOG_ERROR (failed, (uint32_t)0U);
        rc = VSTATUS_BAD;
      }
    }
    else
    {
      /* free the lock */
      rc = vs_spinunlock (&lock);
      if (rc != VSTATUS_OK)
        IB_LOG_ERROR ("vs_spinunlock error", rc);

      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

void
test_spinlock_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;
  uint32_t total_skipped = (uint32_t)0U;

  IB_LOG_INFO ("vs_spinlock:1 TEST STARTED", (uint32_t)0U);

  DOATEST (vs_spinlock_1a, total_passes, total_fails);
  DOATEST (vs_spinlock_2a, total_passes, total_fails);
  DOATEST (vs_spinlock_2b, total_passes, total_fails);
  DOATEST (vs_spinlock_2c, total_passes, total_fails);

  IB_LOG_INFO ("vs_spinlock:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_spinlock:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_spinlock:1 TOTAL SKIPPED", total_skipped);
  IB_LOG_INFO ("vs_spinlock:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (vs_spinunlock)
*/
static Status_t
vs_spinunlock_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "vs_spinunlock:1:1.a PASSED";
  static const char failed[] = "vs_spinunlock:1:1.a FAILED";

  /* attempt to unlock a NULL spinlock pointer */
  p = (void *) 0;
  rc = vs_spinunlock ((Lock_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("vs_spinunlock failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("vs_spinunlock failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_spinunlock_2a (void)
{
  Status_t          rc;
  Lock_t        lock;
  static const char passed[] = "vs_spinunlock:1:2.a PASSED";
  static const char failed[] = "vs_spinunlock:1:2.a FAILED";

  /* create a spin lock initially in the locked state */
  rc = vs_lock_init (&lock, VLOCK_LOCKED, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /* attempt to free the lock */
    IB_LOG_INFO ("attempting to free lock", 0);  /* debug */
    rc = vs_spinunlock (&lock);
    if (rc == VSTATUS_OK)
    {
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
    else
    {
      IB_LOG_ERROR ("vs_spinunlock failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_spinunlock failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      rc = VSTATUS_BAD;
    }

    /* delete the lock */
    IB_LOG_INFO ("attempting to delete the lock", 0);
    (void)vs_lock_delete (&lock);
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
vs_spinunlock_2b (void)
{
  Status_t          rc;
  uint64_t          hold_time = (uint64_t)0U;
  uint64_t          min_time = (uint64_t)0U;
  uint64_t          max_time = (uint64_t)1000U;
  uint64_t          sleep_time = (uint64_t)0U;
  Lock_t        lock;
  static const char passed[] = "vs_spinunlock:1:2.b PASSED";
  static const char failed[] = "vs_spinunlock:1:2.b FAILED";

  /* initialize error counters */
  Gspinlock_errs = 0;

  /* create a spinlock initially in the free state */
  rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_SPIN);
  if (rc == VSTATUS_OK)
  {
    /*
    ** create a thread which will make 3 calls to vs_spinlock and
    ** verify that it requires 3 calls to vs_spinunlock to unnest
    */
    rc = dothread_lock (0,
                        &lock,
                        hold_time,
                        min_time,
                        max_time,
                        spinunlock_unnest);

    if (rc != VSTATUS_OK)
    {
      rc = VSTATUS_BAD;
      IB_LOG_ERROR (failed, (uint32_t)0U);
      return rc;
    }

    /* allow the thread to complete */
    sleep_time = HOLD_ONE_SECOND * (uint64_t)3;
    IB_LOG_INFO ("sleeping to allow threads to complete", 0);  /* debug */
    vs_thread_sleep (sleep_time);

    /* delete the spinlock */
    (void)vs_lock_delete (&lock);

    /* if the 2nd vs_spinlock call didn't complete immediately */
    if (Gspinlock_errs != 0)
    {
      IB_LOG_ERROR ("spinlock errors", Gspinlock_errs);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      return VSTATUS_BAD;
    }
    else
    {
      rc = VSTATUS_OK;
      IB_LOG_INFO (passed, (uint32_t)0U);
    }
  }
  else
  {
    IB_LOG_ERROR ("vs_lock_init error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

void
test_spinunlock_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;
  uint32_t total_skipped = (uint32_t)0U;

  IB_LOG_INFO ("vs_spinunlock:1 TEST STARTED", (uint32_t)0U);

  DOATEST (vs_spinunlock_1a, total_passes, total_fails);
  DOATEST (vs_spinunlock_2a, total_passes, total_fails);
  DOATEST (vs_spinunlock_2b, total_passes, total_fails);

  IB_LOG_INFO ("vs_spinunlock:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_spinunlock:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_spinunlock:1 TOTAL SKIPPED", total_skipped);
  IB_LOG_INFO ("vs_spinunlock:1 TEST COMPLETE", (uint32_t)0U);

  return;
}
