/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

/***********************************************************************
* 
* FILE NAME
*      cs_sema_test.c
*
* DESCRIPTION
*      This file contains Counting Semaphore common test routines.
*      These tests exercise the Counting Semaphore services.
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
* MGR       03/14/02    Initial creation of file.
* MGR       03/18/02    Fixes for kernel support.
* MGR       03/26/02    Corrected lint errors.
* PJG       04/02/02    Update for vs_thread_sleep in OS Services 2.0g
* PJG       04/10/02    Replaced returns in thread functions with
*                         vs_thread_exit.
* PJG       04/11/02    Upped the value of MIN_SEMA time from 500uS to
*                         10mS.
* PJG       04/16/02    Change MIN_SEMA_TIME and LET_THRD_RUN_SLEEPTIME
*                         from octal to decimal constants.
* MGR       04/18/02    PR1728.  Added test case to test blocking of
*                       thread, when thread waiting on sema is killed.
* MGR       04/22/02    PR1742.  Added testcases to test blocking of
*                       cs_psema after cs_vsema.
* MGR       05/08/02    PR1812.  Clean-up lint errors/warnings.
* PJG       11/19/02    PR3096.  Change LET_THRD_RUN_SLEEP to 1010 from
*                         10, to cause reschedule to happen under Linux.
*
***********************************************************************/
#if defined (LINT)
#define __signed__ signed
#include <bits/sigset.h>
#endif
#include <cs_g.h>
#include <vs_g.h>

/**********************************************************************
* Local Macros
***********************************************************************/
#define DOATEST(func, pass, fail) ((func)() == VSTATUS_OK) ? pass++ : fail++

/**********************************************************************
* Defines
***********************************************************************/
#define SLEEP_ONE_SECOND        ((uint64_t)1000000U)
#define MAX_SLEEP_TIME          ((uint64_t)3000000U)
#define MIN_SEMA_TIME           ((uint64_t)  10000U)
#define LET_THRD_RUN_SLEEPTIME  ((uint64_t)   1010U)
#define NUM_TEST_THREADS              ((uint32_t)3U)
#define MAX_STACK_SIZE             ((uint32_t)4096U)
#define VLOCK_INV_STATE             ((uint32_t)101U)
#define VLOCK_INV_TYPE              ((uint32_t)103U)

/**********************************************************************
* Variable Declarations
***********************************************************************/
static Sema_t     *Gsema;
static Thread_t   Gthandle[NUM_TEST_THREADS];
static uint8_t        Gthreadcntr = 0;
static uint32_t       Gnxio_errs = 0;
static uint32_t       Gagain_errs = 0;
static uint32_t       Gthrd_cmp = 0;
uint32_t              Gthrd_errs = 0;
uint32_t              Gthrds_woken = 0;
uint32_t              Gthrd_decr_cnt = 0;

static Status_t
dothread_sema (uint32_t thread_idx,
               Sema_t *sema,
               void (*function) (uint32_t argc, uint8_t *argv[]))
{
  Status_t         rc;
  uint8_t          name[VS_NAME_MAX];
  static uint8_t   *argv[] = {(uint8_t*)"1", (uint8_t*)"2", (uint8_t*)"3"};

  /*
  ** Setup global variables
  */
  Gsema = sema;

  /*
  ** Create/display the thread name, update thread counter
  */
  memcpy((void *)name, "VIEOSemaThr", 11);
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
                             sizeof(int) * MAX_STACK_SIZE);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_thread_create error", rc);
    return VSTATUS_BAD;
  }

  return VSTATUS_OK;
}

static void
thrd_sema_decr_sleep_wake (uint32_t argc,
                           uint8_t *argv[])
{
  Status_t rc;
  uint64_t starttime = (uint64_t)0U;
  uint64_t stoptime = (uint64_t)0U;
  uint64_t deltatime = (uint64_t)0U;

  /* get start time of decrement */
  IB_LOG_INFO ("getting starttime", argc);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    Gthrd_errs++;
    return;
  }

  /* decrement the semaphore */
  IB_LOG_INFO ("decrementing semaphore", argc);  /* debug */
  rc = cs_psema (Gsema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_psema error", rc);

    if (rc == VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("semaphore has been deleted", argc);  /* debug */
      Gnxio_errs++;
    }
    else if (rc == VSTATUS_AGAIN)
    {
      IB_LOG_ERROR ("thread was killed", argc);  /* debug */
      Gagain_errs++;
    }
    else
    {
      IB_LOG_ERROR ("cs_psema error", argc);  /* debug */
      Gthrd_errs++;
    }

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  /* get stoptime of decrement */
  IB_LOG_INFO ("getting stoptime", argc);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    Gthrd_errs++;

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  /* verify thread slept and was awakened */
  deltatime = stoptime - starttime;
  if (deltatime < MAX_SLEEP_TIME)
  {
    IB_LOG_ERROR ("thread did not sleep long enough", deltatime);
    Gthrd_errs++;

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  IB_LOG_INFO ("thread did sleep and was awakened", argc);  /* debug */
  Gthrds_woken++;

  /*
  ** Exit.
  */
  (void) vs_thread_exit (&Gthandle[argc]);
}

static void
thrd_sema_repeat_decr (uint32_t argc,
                       uint8_t *argv[])
{
  Status_t rc;

  /*
  ** repeatedly attempt to decrement the
  ** semaphore, until it is deleted
  */
  while (1)
  {
    IB_LOG_INFO ("attempting to decrement semaphore", argc);
    rc = cs_psema (Gsema);
    if (rc == VSTATUS_OK)
    {
      Gthrd_decr_cnt++;
      IB_LOG_INFO ("successfully decremented semaphore", Gthrd_decr_cnt);
    }
    else
    {
      IB_LOG_INFO ("cannot decrement, semaphore has been deleted", rc);
      /*
      ** Exit.
      */
      (void) vs_thread_exit (&Gthandle[argc]);
      break;
    }
  }
}

static void
thrd_sema_decr_sleep_nxio (uint32_t argc,
                           uint8_t *argv[])
{
  Status_t rc;

  /* decrement the semaphore */
  IB_LOG_INFO ("decrementing semaphore", argc);  /* debug */
  rc = cs_psema (Gsema);
  if (rc == VSTATUS_NXIO)
  {
    IB_LOG_INFO ("Semaphore was deleted", argc);  /* debug */
    Gnxio_errs++;

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }
  else
  {
    IB_LOG_ERROR ("cs_psema did not return as expected", argc);
    IB_LOG_ERROR ("cs_psema error", rc);
    Gthrd_errs++;

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }
}

static void
thrd_sema_decr (uint32_t argc,
                uint8_t *argv[])
{
  Status_t rc;
  uint64_t starttime = (uint64_t)0U;
  uint64_t stoptime = (uint64_t)0U;

  /* get start time of decrement */
  IB_LOG_INFO ("getting starttime", argc);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    Gthrd_errs++;
    return;
  }

  /* decrement the semaphore */
  IB_LOG_INFO ("decrementing semaphore", argc);  /* debug */
  rc = cs_psema (Gsema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_psema error", rc);

    if (rc == VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("semaphore has been deleted", argc);  /* debug */
      Gnxio_errs++;
    }
    else if (rc == VSTATUS_AGAIN)
    {
      IB_LOG_ERROR ("thread was killed", argc);  /* debug */
      Gagain_errs++;
    }
    else
    {
      Gthrd_errs++;
    }

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  /* get stoptime of decrement */
  IB_LOG_INFO ("getting stoptime", argc);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    Gthrd_errs++;

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  /*
  ** Exit.
  */
  Gthrd_cmp++;
  IB_LOG_INFO ("decrement thread completed", argc);
  (void) vs_thread_exit (&Gthandle[argc]);
}

static void
thrd_sema_incr (uint32_t argc,
                uint8_t *argv[])
{
  Status_t rc;
  uint64_t starttime = (uint64_t)0U;
  uint64_t stoptime = (uint64_t)0U;

  /* get start time of increment */
  IB_LOG_INFO ("getting starttime", argc);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    Gthrd_errs++;
    return;
  }

  /* increment the semaphore */
  IB_LOG_INFO ("incrementing semaphore", argc);  /* debug */
  rc = cs_vsema (Gsema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_vsema error", rc);

    if (rc == VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("semaphore has been deleted", argc);  /* debug */
      Gnxio_errs++;
    }
    else if (rc == VSTATUS_AGAIN)
    {
      IB_LOG_ERROR ("thread was killed", argc);  /* debug */
      Gagain_errs++;
    }
    else
    {
      Gthrd_errs++;
    }

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  /* get stoptime of increment */
  IB_LOG_INFO ("getting stoptime", argc);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    Gthrd_errs++;

    /*
    ** Exit.
    */
    (void) vs_thread_exit (&Gthandle[argc]);
  }

  /*
  ** Exit.
  */
  Gthrd_cmp++;
  IB_LOG_INFO ("increment thread completed", argc);
  (void) vs_thread_exit (&Gthandle[argc]);
}


/*
** Sub-test Variations (cs_sema_create)
*/
static Status_t
cs_sema_create_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "cs_sema_create:1:1.a PASSED";
  static const char failed[] = "cs_sema_create:1:1.a FAILED";

  /* attempt to create a semaphore with a NULL semaphore pointer */
  p = (void *)0;
  rc = cs_sema_create ((Sema_t *)(p), 0);
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("cs_sema_create failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("cs_sema_create failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
cs_sema_create_2a (void)
{
  Status_t          rc;
  Sema_t        sema;
  static const char passed[] = "cs_sema_create:1:2.a PASSED";
  static const char failed[] = "cs_sema_create:1:2.a FAILED";

  /* attempt to create a semaphore with an initial count of 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_create failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* delete the semaphore */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_sema_create_2b (void)
{
  Status_t          rc;
  uint64_t          sleep_time;
  Sema_t        sema;
  static const char passed[] = "cs_sema_create:1:2.b PASSED";
  static const char failed[] = "cs_sema_create:1:2.b FAILED";

  /* initialize global variables */
  Gthrds_woken = 0;
  Gthrd_errs = 0;
  Gnxio_errs = 0;

  /* create a semaphore with an initial count of 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_create failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* 
  ** create a thread that will decrement the semaphore, sleep,
  ** and wake to resume execution when semaphore is incremented
  */
  IB_LOG_INFO ("creating thread", 0);  /* debug */
  rc = dothread_sema ((uint32_t)0U,
                      &sema,
                      thrd_sema_decr_sleep_wake);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow thread to run", 0);  /* debug */
  sleep_time = LET_THRD_RUN_SLEEPTIME;
  vs_thread_sleep (sleep_time);

  /*
  ** thread should be sleeping now, sleep a little
  ** longer so we can verify that the thread did sleep
  */
  IB_LOG_INFO ("sleeping to verify thread sleeps", 0);  /* debug */
  sleep_time = MAX_SLEEP_TIME;
  vs_thread_sleep (sleep_time);

  /* increment the semaphore to wake the thread */
  IB_LOG_INFO ("incrementing semaphore to wake thread", 0);  /* debug */
  rc = cs_vsema (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_vsema error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow thread to compete execution */
  IB_LOG_INFO ("sleeping to allow thread to complete", 0);  /* debug */
  sleep_time = SLEEP_ONE_SECOND;
  vs_thread_sleep (sleep_time);

  /* delete the semaphore */
  IB_LOG_INFO ("deleting semaphore", 0);  /* debug */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* verify the thread did sleep and then wake-up */
  if ((Gthrds_woken == 1) &&
      (Gthrd_errs == 0) &&
      (Gnxio_errs == 0))
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    if (Gthrds_woken != 1)
      IB_LOG_ERROR ("thread did not wake", Gthrds_woken);

    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_sema_create_2c (void)
{
  Status_t          rc;
  Sema_t        sema;
  static const char passed[] = "cs_sema_create:1:2.c PASSED";
  static const char failed[] = "cs_sema_create:1:2.c FAILED";

  /* attempt to create a semaphore with an initial count greater than 0 */
  rc = cs_sema_create (&sema, 10);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_create failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* delete the semaphore */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_sema_create_2d (void)
{
  Status_t          rc;
  uint32_t          sema_cnt;
  uint64_t          sleep_time;
  Sema_t        sema;
  static const char passed[] = "cs_sema_create:1:2.d PASSED";
  static const char failed[] = "cs_sema_create:1:2.d FAILED";

  /* initialize global variables */
  Gthrd_decr_cnt = 0;
  sema_cnt = 5;

  /* create a semaphore with an initial count greater than 0 */
  rc = cs_sema_create (&sema, sema_cnt);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_create failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* create a thread that repeatedly attempts to decrement the semaphore */
  IB_LOG_INFO ("creating thread", 0);
  rc = dothread_sema ((uint32_t)0U,
                      &sema,
                      thrd_sema_repeat_decr);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow thread to run", 0);  /* debug */
  sleep_time = SLEEP_ONE_SECOND;
  vs_thread_sleep (sleep_time);

  /*
  ** thread should be sleeping now, delete
  ** the semaphore to awaken the thread
  */
  IB_LOG_INFO ("deleting semaphore", 0);  /* debug */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow thread to complete execution */
  IB_LOG_INFO ("sleeping to allow thread to complete", 0);  /* debug */
  sleep_time = SLEEP_ONE_SECOND;
  vs_thread_sleep (sleep_time);

  /*
  ** verify the thread did successfully decrement
  ** the semaphore the correct number of times
  */
  if (Gthrd_decr_cnt == sema_cnt)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("semaphore was not decremented the correct number of times",
                  Gthrd_decr_cnt);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_sema_create_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_sema_create:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_sema_create_1a, total_passes, total_fails);
  DOATEST (cs_sema_create_2a, total_passes, total_fails);
  DOATEST (cs_sema_create_2b, total_passes, total_fails);
  DOATEST (cs_sema_create_2c, total_passes, total_fails);
  DOATEST (cs_sema_create_2d, total_passes, total_fails);

  IB_LOG_INFO ("cs_sema_create:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_sema_create:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_sema_create:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_sema_delete)
*/
static Status_t
cs_sema_delete_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "cs_sema_delete:1:1.a PASSED";
  static const char failed[] = "cs_sema_delete:1:1.a FAILED";

  /* attempt to delete a semaphore with a NULL semaphore pointer */
  p = (void *)0;
  rc = cs_sema_delete ((Sema_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("cs_sema_delete failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("cs_sema_delete failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
cs_sema_delete_2a (void)
{
  Status_t          rc;
  Sema_t        sema;
  static const char passed[] = "cs_sema_delete:1:2.a PASSED";
  static const char failed[] = "cs_sema_delete:1:2.a FAILED";

  /* create a semaphore with an initial count of 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* attempt to delete the semaphore with an initial count of 0 */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_delete failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_sema_delete_2b (void)
{
  Status_t          rc;
  Sema_t        sema;
  static const char passed[] = "cs_sema_delete:1:2.b PASSED";
  static const char failed[] = "cs_sema_delete:1:2.b FAILED";

  /* create a semaphore with an initial count greater than 0 */
  rc = cs_sema_create (&sema, 11);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* attempt to delete the semaphore with an initial count greater than 0 */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_delete failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_sema_delete_2c (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time;
  Sema_t        sema;
  static const char passed[] = "cs_sema_delete:1:2.c PASSED";
  static const char failed[] = "cs_sema_delete:1:2.c FAILED";

  /* initialize global variables */
  Gthrd_errs = 0;
  Gnxio_errs = 0;

  /* create a semaphore with an initial count of 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* create threads that will decrement the semaphore and sleep */
  for (i = 0; i < NUM_TEST_THREADS; i++)
  {
    IB_LOG_INFO ("creating thread", i);  /* debug */
    rc = dothread_sema (i,
                        &sema,
                        thrd_sema_decr_sleep_nxio);

    if (rc != VSTATUS_OK)
      break;

    /* sleep to allow thread to run */
    IB_LOG_INFO ("sleeping to allow thread to run", i);  /* debug */
    sleep_time = LET_THRD_RUN_SLEEPTIME;
    vs_thread_sleep (sleep_time);
  }

  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* threads should be sleeping now, delete the semaphore */
  IB_LOG_INFO ("deleting semaphore", 0);  /* debug */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_sema_delete failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow threads to compete execution */
  IB_LOG_INFO ("sleeping to allow threads to complete", 0);  /* debug */
  sleep_time = MAX_SLEEP_TIME;
  vs_thread_sleep (sleep_time);

  /* verify the threads did return a VSTATUS_NXIO return code */
  if ((Gnxio_errs == NUM_TEST_THREADS) && (Gthrd_errs == 0))
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    if (Gthrd_errs != 0)
      IB_LOG_ERROR ("thread errors", Gthrd_errs);

    if (Gnxio_errs != NUM_TEST_THREADS)
      IB_LOG_ERROR ("all threads did not return VSTATUS_NXIO", Gnxio_errs);

    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_sema_delete_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_sema_delete:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_sema_delete_1a, total_passes, total_fails);
  DOATEST (cs_sema_delete_2a, total_passes, total_fails);
  DOATEST (cs_sema_delete_2b, total_passes, total_fails);
  DOATEST (cs_sema_delete_2c, total_passes, total_fails);

  IB_LOG_INFO ("cs_sema_delete:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_sema_delete:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_sema_delete:1 TEST COMPLETE", (uint32_t)0U);

  return;
}

/*
** Sub-test Variations (cs_vsema)
*/
static Status_t
cs_vsema_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "cs_vsema:1:1.a PASSED";
  static const char failed[] = "cs_vsema:1:1.a FAILED";

  /* attempt to increment a semaphore with a NULL semaphore pointer */
  p = (void *)0;
  rc = cs_vsema ((Sema_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("cs_vsema failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("cs_vsema failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
cs_vsema_2a (void)
{
  Status_t          rc;
  uint64_t          starttime = (uint64_t)0U;
  uint64_t          stoptime = (uint64_t)0U;
  uint64_t          deltatime = (uint64_t)0U;
  Sema_t        sema;
  static const char passed[] = "cs_vsema:1:2.a PASSED";
  static const char failed[] = "cs_vsema:1:2.a FAILED";

  /* create a semaphore */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* get start time of increment attempt */
  IB_LOG_INFO ("getting starttime", 0);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* attempt to increment a semaphore */
  IB_LOG_INFO ("attempting to increment semaphore", 0);  /* debug */
  rc = cs_vsema (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_vsema failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_vsema failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  /* get stop time of increment attempt */
  IB_LOG_INFO ("getting stoptime", 0);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* delete the semaphore */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* verify call did not block */
  deltatime = stoptime - starttime;
  if (deltatime > MIN_SEMA_TIME)
  {
    IB_LOG_ERROR ("cs_vsema call blocked", deltatime);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_vsema_2b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time;
  Sema_t        sema;
  static const char passed[] = "cs_vsema:1:2.b PASSED";
  static const char failed[] = "cs_vsema:1:2.b FAILED";

  /* initialize global variables */
  Gthrds_woken = 0;
  Gthrd_errs = 0;
  Gnxio_errs = 0;

  /* create a semaphore with an initial count of 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* create threads that will decrement the semaphore and sleep */
  for (i = 0; i < NUM_TEST_THREADS; i++)
  {
    IB_LOG_INFO ("creating thread", i);  /* debug */
    rc = dothread_sema (i,
                        &sema,
                        thrd_sema_decr_sleep_wake);

    if (rc != VSTATUS_OK)
      break;

    /* sleep to allow thread to run */
    IB_LOG_INFO ("sleeping to allow thread to run", i);  /* debug */
    sleep_time = LET_THRD_RUN_SLEEPTIME;
    vs_thread_sleep (sleep_time);
  }

  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /*
  ** threads should be sleeping now; sleep so we can
  ** verify threads do sleep for at least this long
  */
  sleep_time = MAX_SLEEP_TIME;
  vs_thread_sleep (sleep_time);

  /*
  ** attempt to increment the semaphore;
  ** only one thread should have been woken
  */
  rc = cs_vsema (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_vsema failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_vsema failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow thread to compete execution */
  IB_LOG_INFO ("sleeping to allow thread to complete", 0);  /* debug */
  sleep_time = SLEEP_ONE_SECOND;
  vs_thread_sleep (sleep_time);

  /* delete the semaphore to awaken remaining threads */
  IB_LOG_INFO ("deleting semaphore", 0);  /* debug */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* sleep to allow remaining threads to complete execution */
  IB_LOG_INFO ("sleeping to allow threads to complete", 0);  /* debug */
  sleep_time = SLEEP_ONE_SECOND;
  vs_thread_sleep (sleep_time);

  /*
  ** verify the first thread did sleep and
  ** wake-up to complete execution, and the
  ** other threads returned VSTATUS_NXIO
  */
  if ((Gthrds_woken == 1) &&
      (Gthrd_errs == 0) &&
      (Gnxio_errs == (NUM_TEST_THREADS-1)))
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    if (Gthrds_woken != 1)
      IB_LOG_ERROR ("thread#1 did not wake", Gthrds_woken);

    if (Gnxio_errs != (NUM_TEST_THREADS-1))
      IB_LOG_ERROR ("threads did not return VSTATUS_NXIO", Gnxio_errs);

    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_vsema_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_vsema:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_vsema_1a, total_passes, total_fails);
  DOATEST (cs_vsema_2a, total_passes, total_fails);
  DOATEST (cs_vsema_2b, total_passes, total_fails);

  IB_LOG_INFO ("cs_vsema:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_vsema:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_vsema:1 TEST COMPLETE", (uint32_t)0U);

  return;
}

/*
** Sub-test Variations (cs_psema)
*/
static Status_t
cs_psema_1a (void)
{
  void              *p;
  Status_t          rc;
  static const char passed[] = "cs_psema:1:1.a PASSED";
  static const char failed[] = "cs_psema:1:1.a FAILED";

  /* attempt to decrement a semaphore with a NULL semaphore pointer */
  p = (void *)0;
  rc = cs_psema ((Sema_t *)(p));
  if (rc == VSTATUS_ILLPARM)
  {
    rc = VSTATUS_OK;
    IB_LOG_INFO (passed, (uint32_t)0U);
  }
  else
  {
    IB_LOG_ERROR ("cs_psema failed; expected", VSTATUS_ILLPARM);
    IB_LOG_ERROR ("cs_psema failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    rc = VSTATUS_BAD;
  }

  return rc;
}

static Status_t
cs_psema_2a (void)
{
  Status_t          rc;
  uint64_t          starttime = (uint64_t)0U;
  uint64_t          stoptime = (uint64_t)0U;
  uint64_t          deltatime = (uint64_t)0U;
  Sema_t        sema;
  static const char passed[] = "cs_psema:1:2.a PASSED";
  static const char failed[] = "cs_psema:1:2.a FAILED";

  /* attempt to create a semaphore with an initial count greater than 0 */
  rc = cs_sema_create (&sema, 1);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* get start time of decrement attempt */
  IB_LOG_INFO ("getting starttime", 0);  /* debug */
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get starttime error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* attempt to decrement the semaphore */
  rc = cs_psema (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_psema failed; expected", VSTATUS_OK);
    IB_LOG_ERROR ("cs_psema failed; actual", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
  }

  /* get stop time of decrement attempt */
  IB_LOG_INFO ("getting stoptime", 0);  /* debug */
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("vs_time_get stoptime error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* delete the semaphore */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* verify call did not block */
  deltatime = stoptime - starttime;
  if (deltatime > MIN_SEMA_TIME)
  {
    IB_LOG_ERROR ("cs_vsema call blocked", deltatime);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_psema_2b (void)
{
  Status_t          rc;
  uint32_t          i;
  uint64_t          sleep_time;
  Sema_t        sema;
  static const char passed[] = "cs_psema:1:2.b PASSED";
  static const char failed[] = "cs_psema:1:2.b FAILED";

  /* initialize global variables */
  Gthrds_woken = 0;
  Gthrd_errs = 0;
  Gnxio_errs = 0;

  /* create a semaphore with an initial count of 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* create threads that will decrement the semaphore and sleep */
  for (i = 0; i < NUM_TEST_THREADS; i++)
  {
    IB_LOG_INFO ("creating thread", i);  /* debug */
    rc = dothread_sema (i,
                        &sema,
                        thrd_sema_decr_sleep_wake);

    if (rc != VSTATUS_OK)
      break;

    /* sleep to allow thread to run */
    IB_LOG_INFO ("sleeping to allow thread to run", i);  /* debug */
    sleep_time = LET_THRD_RUN_SLEEPTIME;
    vs_thread_sleep (sleep_time);
  }

  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /*
  ** threads should be sleeping now; sleep so we can
  ** verify threads do sleep for at least this long
  */
  sleep_time = MAX_SLEEP_TIME;
  vs_thread_sleep (sleep_time);

  for (i = 0; i < NUM_TEST_THREADS; i++)
  {
    /*
    ** attempt to increment the semaphore;
    ** only one thread should have been woken
    */
    rc = cs_vsema (&sema);
    if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("cs_vsema error", rc);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      return VSTATUS_BAD;
    }

    /* sleep to allow thread to complete execution */
    IB_LOG_INFO ("sleeping to allow thread to complete", i);  /* debug */
    sleep_time = SLEEP_ONE_SECOND;
    vs_thread_sleep (sleep_time);

    /* verify only one thread was woken */
    if ((Gthrds_woken != (i+1)) ||
        (Gthrd_errs != 0) ||
        (Gnxio_errs != 0))
    {
      IB_LOG_ERROR ("thread was not woken", Gthrds_woken);
      IB_LOG_ERROR (failed, (uint32_t)0U);
      return VSTATUS_BAD;
    }
  }

  /* delete the semaphore */
  IB_LOG_INFO ("deleting semaphore", 0);  /* debug */
  rc = cs_sema_delete (&sema);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_delete error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  IB_LOG_INFO (passed, (uint32_t)0U);
  return VSTATUS_OK;
}

static Status_t
cs_psema_2c (void)
{
  Status_t          rc;
  uint32_t          i = 0;
  uint64_t          sleep_time = (uint64_t)0U;
  Sema_t        sema;
  static const char passed[] = "cs_psema:1:2.c PASSED";
  static const char failed[] = "cs_psema:1:2.c FAILED";

  /* initialize thread complete counter */
  Gthrd_cmp = 0;
  Gagain_errs = 0;

  /* create a semaphore with an initial count 0 */
  rc = cs_sema_create (&sema, 0);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* create a thread that will attempt to increment the sema */
  IB_LOG_INFO ("creating thread to increment sema", i);  /* debug */
  rc = dothread_sema (i,
                      &sema,
                      thrd_sema_incr);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
  i++;

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow incr thread to run", i);  /* debug */
  sleep_time = LET_THRD_RUN_SLEEPTIME;
  vs_thread_sleep (sleep_time);

  /* create a thread that will attempt to decrement the sema */
  IB_LOG_INFO ("creating thread to decrement sema", i);  /* debug */
  rc = dothread_sema (i,
                      &sema,
                      thrd_sema_decr);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
  i++;

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow decr thread to run", i);  /* debug */
  sleep_time = LET_THRD_RUN_SLEEPTIME;
  vs_thread_sleep (sleep_time);

  if ((Gthrd_cmp == 2) && (Gagain_errs == 0))
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_psema_2d (void)
{
  Status_t          rc;
  uint32_t          i = 0;
  uint64_t          sleep_time = (uint64_t)0U;
  Sema_t        sema;
  static const char passed[] = "cs_psema:1:2.d PASSED";
  static const char failed[] = "cs_psema:1:2.d FAILED";

  /* initialize thread complete counter */
  Gthrd_cmp = 0;
  Gagain_errs = 0;

  /* create a semaphore with an initial count of 1 */
  rc = cs_sema_create (&sema, 1);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR ("cs_sema_create error", rc);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }

  /* create a thread that will attempt to decrement the sema */
  IB_LOG_INFO ("creating thread to decrement sema", i);  /* debug */
  rc = dothread_sema (i,
                      &sema,
                      thrd_sema_decr);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
  i++;

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow decr thread to run", i);  /* debug */
  sleep_time = LET_THRD_RUN_SLEEPTIME;
  vs_thread_sleep (sleep_time);

  /* create a thread that will attempt to increment the sema */
  IB_LOG_INFO ("creating thread to increment sema", i);  /* debug */
  rc = dothread_sema (i,
                      &sema,
                      thrd_sema_incr);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
  i++;

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow incr thread to run", i);  /* debug */
  sleep_time = LET_THRD_RUN_SLEEPTIME;
  vs_thread_sleep (sleep_time);

  /* create a thread that will attempt to decrement the sema */
  IB_LOG_INFO ("creating thread to decrement sema", i);  /* debug */
  rc = dothread_sema (i,
                      &sema,
                      thrd_sema_decr);
  if (rc != VSTATUS_OK)
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
  i++;

  /* sleep to allow thread to run */
  IB_LOG_INFO ("sleeping to allow decr thread to run", i);  /* debug */
  sleep_time = LET_THRD_RUN_SLEEPTIME;
  vs_thread_sleep (sleep_time);

  if ((Gthrd_cmp == 3) && (Gagain_errs == 0))
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_psema_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;
  uint32_t total_skipped = (uint32_t)0U;

  IB_LOG_INFO ("cs_psema:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_psema_1a, total_passes, total_fails);
  DOATEST (cs_psema_2a, total_passes, total_fails);
  DOATEST (cs_psema_2b, total_passes, total_fails);
  DOATEST (cs_psema_2c, total_passes, total_fails);
  DOATEST (cs_psema_2d, total_passes, total_fails);
  IB_LOG_INFO ("cs_psema:1:3.a SKIPPED", (uint32_t)0U);
  total_skipped++;

  IB_LOG_INFO ("cs_psema:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_psema:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_psema:1 TOTAL SKIPPED", total_skipped);
  IB_LOG_INFO ("cs_psema:1 TEST COMPLETE", (uint32_t)0U);

  return;
}
