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
*      vs_threvent_test.c
*
* DESCRIPTION
*      This file contains vs_event common test routines. These tests
*      exercise the vs_event services between threads.
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
* DKJ       02/14/02    Initial creation of file.
* PJG       03/13/02    Give threads some stack if must_supply_stack is
*                         non-zero.
*                       Changes to support thread exits.
* DKJ       04/02/02    OS API 2.0g updates
* PJG       04/10/02    Sprinkle WAIT_FOR_LOGGING_TO_CATCHUP between
*                         tests.  Due to missed output under ATI.
*                       Allocate distinct stack space for each thread
*                         in vs_event_post_4a.
* PJG       04/10/02    Put in a thread exit in thread_delete_events 
***********************************************************************/
#include <vs_g.h>
#include <cs_g.h>

#define STACKSIZE (sizeof(int) * 2048)
//extern void *memset (void *, int, size_t);
#if 0
extern Status_t
vs_event_spacetest (Evt_handle_t handle, Eventset_t wait_mask,
			Eventset_t compare_mask,
			Eventset_t bad_mask, Eventset_t ok_mask,
			uint32_t option, uint64_t waittime, uint64_t mintime,
			uint64_t maxtime);
#endif
static uint64_t sleeptime;
#define DOATEST(func, pass, fail) ((func)() == VSTATUS_OK) ? pass++ : fail++
#define WAIT_FOR_LOGGING_TO_CATCHUP  sleeptime = (uint64_t) 1000000U; \
  (void) vs_thread_sleep (sleeptime)
#define WAIT_ONE_SECOND ((uint64_t) 1000000U)
#define WAIT_ONE_SECOND_MIN ((uint64_t) 750000U)
#define WAIT_ONE_SECOND_MAX ((uint64_t) 1250000U)
#define NOWAIT_MAX ((uint64_t) 250000U)
#define NUM_EVENTS ((uint32_t) 32U)
#define VEVENT_TST(e, eno) (((e) & (Eventset_t) (0x1U << (eno))) \
                             != (Eventset_t) 0x00)
#define VEVENT_CLR(e, eno) (e) = (e) & ~((Eventset_t) (0x01U << (eno)))
#define VEVENT_SET(e, eno) (e) = (e) | ((Eventset_t) (0x01U << (eno)))
#define VEVENT_CLRALL(e) (e) = (Eventset_t) 0x00U
#define VEVENT_SETALL(e) (e) = (Eventset_t) ~(0x00U)
static Event_t tevent;
static Eventset_t Gwait_mask;
static Eventset_t Gcompare_mask;
static Eventset_t GpostOK_mask;
static Eventset_t GpostBAD_mask;
static uint64_t Gwaittime;
static uint64_t Gmintime;
static uint64_t Gmaxtime;
static Eventset_t Await_mask;
static Eventset_t Acompare_mask;
static Eventset_t ApostOK_mask;
static Eventset_t ApostBAD_mask;
static uint64_t Awaittime;
static uint64_t Amintime;
static uint64_t Amaxtime;

static Status_t
dothread_events (Evt_handle_t ev, Eventset_t mask,
		 Eventset_t okmask, Eventset_t badmask,
		 uint64_t waittime, Eventset_t compare, uint64_t mintime,
		 uint64_t maxtime, uint64_t rendezvoustime,
		 void (*function) (uint32_t argc, uint8_t * argv[]),
		 size_t stack_size)
{
  Status_t rc;
  Thread_t thandle;
  unsigned char name[VS_NAME_MAX];
  Eventset_t events = (Eventset_t) 0U;


  GpostBAD_mask = badmask;
  GpostOK_mask = okmask;
  Gwait_mask = mask;
  Gcompare_mask = compare;
  Gwaittime = waittime;
  Gmintime = mintime;
  Gmaxtime = maxtime;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  rc =
    vs_thread_create (&thandle, name, function, 0, (void *) &thandle,
			  stack_size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_thread_create failed error code", rc);
      return VSTATUS_BAD;
    }
  rc = vs_event_wait (ev, rendezvoustime, badmask | okmask, &events);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_event_wait failed error code", rc);
      return VSTATUS_BAD;
    }
  if ((events & badmask) != (Eventset_t) 0U)
    {
      return VSTATUS_BAD;
    }
  if ((events & okmask) != (Eventset_t) 0U)
    {
      return VSTATUS_OK;
    }
  return VSTATUS_BAD;
}

static Status_t
timeout_events (Evt_handle_t ev, Eventset_t wmask, uint64_t mintime,
		uint64_t maxtime, uint64_t waittime)
{
  Eventset_t events;
  Status_t rc;
  uint64_t starttime = (uint64_t) 0U;
  uint64_t stoptime = (uint64_t) 0U;
  uint64_t deltatime = (uint64_t) 0U;

  events = (Eventset_t) 0U;
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_time_get starttime error", rc);
      return VSTATUS_BAD;
    }
  rc = vs_event_wait (ev, waittime, wmask, &events);
  if (rc != VSTATUS_TIMEOUT)
    {
      IB_LOG_ERROR ("vs_event_wait error: expected", VSTATUS_TIMEOUT);
      IB_LOG_ERROR ("vs_event_wait error: actual", rc);
      IB_LOG_ERROR ("vs_event_wait error: mask", wmask);
      IB_LOG_ERROR ("vs_event_wait error: events", events);
      return VSTATUS_BAD;
    }
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_time_get stoptime error", rc);
      return VSTATUS_BAD;
    }
  deltatime = stoptime - starttime;
  if (deltatime < mintime)
    {
      IB_LOG_ERROR ("waittime too short", (uint32_t) deltatime);
      return VSTATUS_BAD;
    }
  if (deltatime > maxtime)
    {
      IB_LOG_ERROR ("waittime too long", (uint32_t) deltatime);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;
}

static void
thread_timeout_events ( /*@unused@ */ uint32_t argc,
		       uint8_t * argv[])
{
  Status_t rc;
  rc = timeout_events (tevent.event_handle, Gwait_mask, Gmintime, Gmaxtime,
		       Gwaittime);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			   GpostOK_mask);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_post error code: ", rc);
	}
    }
  else
    {
      rc =
	vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			   GpostBAD_mask);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_post error code: ", rc);
	}
    }
  (void) vs_thread_exit ((Thread_t *) argv);
}

static Status_t
vs_event_create_2b (void)
{
  static const char passed[] = "vs_event_create:1:2.b PASSED";
  static const char failed[] = "vs_event_create:1:2.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX + 5];

  (void) memset (name, 0x08, (size_t) (VS_NAME_MAX + 5));

  rc = vs_event_create (&tevent, name, (Eventset_t) 0U);
  if (rc == VSTATUS_ILLPARM)
    {
      rc = VSTATUS_OK;
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_create_2a (void)
{
  static const char passed[] = "vs_event_create:1:2.a PASSED";
  static const char failed[] = "vs_event_create:1:2.a FAILED";
  Status_t rc;

  rc = vs_event_create (&tevent, 0, (Eventset_t) 0U);
  if (rc == VSTATUS_ILLPARM)
    {
      rc = VSTATUS_OK;
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_create_1a (void)
{
  static const char passed[] = "vs_event_create:1:1.a PASSED";
  static const char failed[] = "vs_event_create:1:1.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  rc = vs_event_create (0, name, (Eventset_t) 0U);
  if (rc == VSTATUS_ILLPARM)
    {
      rc = VSTATUS_OK;
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_create_3a (void)
{
  static const char passed[] = "vs_event_create:1:3.a PASSED";
  static const char failed[] = "vs_event_create:1:3.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  rc = vs_event_create (&tevent, name, (Eventset_t) 0U);
  if (rc == VSTATUS_OK)
    {
      rc = VSTATUS_OK;
      IB_LOG_INFO (passed, (uint32_t) 0U);
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_create_3b (void)
{
  static const char passed[] = "vs_event_create:1:3.b PASSED";
  static const char failed[] = "vs_event_create:1:3.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint32_t i;
  uint32_t current;
  uint32_t good;
  uint32_t bad;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  rc = vs_event_create (&tevent, name, (Eventset_t) 0U);
  if (rc == VSTATUS_OK)
    {
      /*
       ** Test clear from a different thread
       */
      for (i = (uint32_t) 0U; i < NUM_EVENTS; i++)
	{
	  current = (Eventset_t) (0x01U << i);
	  good =
	    (current ==
	     (Eventset_t) 0x01U) ? (Eventset_t) 0x04U : (Eventset_t) 0x01U;
	  bad =
	    (current ==
	     (Eventset_t) 0x02U) ? (Eventset_t) 0x08U : (Eventset_t) 0x02U;
	  rc =
	    dothread_events (tevent.event_handle, current, good, bad,
			     WAIT_ONE_SECOND, current, WAIT_ONE_SECOND_MIN,
			     WAIT_ONE_SECOND_MAX,
			     WAIT_ONE_SECOND * (uint64_t) 40U,
			     thread_timeout_events,
			     STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      break;
	    }
	}
      if (rc == VSTATUS_OK)
	{
	  rc = VSTATUS_OK;
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}


static Status_t
vs_event_create_3c (void)
{
  static const char passed[] = "vs_event_create:1:3.c PASSED";
  static const char failed[] = "vs_event_create:1:3.c FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  rc = vs_event_create (&tevent, name, (Eventset_t) 0U);
  if (rc == VSTATUS_OK)
    {
      /*
       ** Test clear from a different thread
       */
      rc =
	dothread_events (tevent.event_handle,
			 (Eventset_t) 0xFFFFFFFFU,
			 (Eventset_t) 0x01U, (Eventset_t) 0x02U,
			 WAIT_ONE_SECOND,
			 (Eventset_t) 0x00U,
			 WAIT_ONE_SECOND_MIN,
			 WAIT_ONE_SECOND_MAX,
			 WAIT_ONE_SECOND * (uint64_t) 2U,
			 thread_timeout_events,
			 STACKSIZE);
      if (rc == VSTATUS_OK)
	{
	  rc = VSTATUS_OK;
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_create_4a (void)
{
  static const char passed[] = "vs_event_create:1:4.a PASSED";
  static const char failed[] = "vs_event_create:1:4.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint32_t i;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  for (i = (uint32_t) 0U; i < NUM_EVENTS; i++)
    {
      rc = vs_event_create (&tevent, name, (Eventset_t) (0x01U << i));
      if (rc == VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
get_events (Evt_handle_t ev, Eventset_t waitmask,
	    Eventset_t comparemask, uint64_t mintime, uint64_t maxtime,
	    uint64_t waittime)
{
  Eventset_t events;
  Status_t rc;
  uint64_t starttime = (uint64_t) 0U;
  uint64_t stoptime = (uint64_t) 0U;
  uint64_t deltatime = (uint64_t) 0U;

  events = (Eventset_t) 0U;
  rc = vs_time_get (&starttime);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_time_get starttime error", rc);
      return VSTATUS_BAD;
    }
  rc = vs_event_wait (ev, waittime, waitmask, &events);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_event_wait error: expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_wait error: actual", rc);
      return VSTATUS_BAD;
    }
  rc = vs_time_get (&stoptime);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_time_get stoptime error", rc);
      return VSTATUS_BAD;
    }
  if ((comparemask ^ events) != (Eventset_t) 0U)
    {
      IB_LOG_ERROR ("vs_event_wait error: mask", comparemask);
      IB_LOG_ERROR ("vs_event_wait error: events", events);
      return VSTATUS_BAD;
    }
  deltatime = stoptime - starttime;
  if (deltatime > maxtime)
    {
      IB_LOG_ERROR ("waittime too long delta", (uint32_t) deltatime);
      IB_LOG_ERROR ("waittime too long maxtime", (uint32_t) maxtime);
      return VSTATUS_BAD;
    }
  if (deltatime < mintime)
    {
      IB_LOG_ERROR ("waittime too short delta", (uint32_t) deltatime);
      IB_LOG_ERROR ("waittime too short mintime", (uint32_t) mintime);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;
}

static void
thread_get_events ( /*@unused@ */ uint32_t argc,
		   uint8_t * argv[])
{
  Status_t rc;
  rc = get_events (tevent.event_handle, Gwait_mask, Gcompare_mask,
		   Gmintime, Gmaxtime, Gwaittime);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			   GpostOK_mask);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_post error code: ", rc);
	}
    }
  else
    {
      rc =
	vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			   GpostBAD_mask);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_post error code: ", rc);
	}
    }
  (void) vs_thread_exit ((Thread_t *) argv);
}

static void
threadA_get_events ( /*@unused@ */ uint32_t argc,	/*@unused@ */
		    uint8_t * argv[])
{
  Status_t rc;
  rc = get_events (tevent.event_handle, Await_mask, Acompare_mask,
		   Amintime, Amaxtime, Awaittime);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			   ApostOK_mask);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_post error code: ", rc);
	}
    }
  else
    {
      rc =
	vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			   ApostBAD_mask);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_post error code: ", rc);
	}
    }
  (void) vs_thread_exit ((Thread_t *) argv);
}

static Status_t
vs_event_create_4b (void)
{
  static const char passed[] = "vs_event_create:1:4.b PASSED";
  static const char failed[] = "vs_event_create:1:4.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint32_t i;
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  for (i = (uint32_t) 0U; i < NUM_EVENTS; i++)
    {
      current = (Eventset_t) (0x01U << i);
      good = (current == (Eventset_t) 0x01U) ? (Eventset_t) 0x04U :
	(Eventset_t) 0x01U;
      bad = (current == (Eventset_t) 0x02U) ? (Eventset_t) 0x08U :
	(Eventset_t) 0x02U;
      rc = vs_event_create (&tevent, name, current);
      if (rc == VSTATUS_OK)
	{
	  rc = dothread_events (tevent.event_handle,
				current,
				good,
				bad, WAIT_ONE_SECOND,
				current, (uint64_t) 0U,
				WAIT_ONE_SECOND_MIN,
				WAIT_ONE_SECOND * (uint64_t) 5U,
				thread_get_events,
				STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  (void) vs_event_delete (&tevent);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_4c (void)
{
  static const char passed[] = "vs_event_create:1:4.c PASSED";
  static const char failed[] = "vs_event_create:1:4.c FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint32_t i;
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  for (i = (uint32_t) 0U; i < NUM_EVENTS; i++)
    {
      current = (Eventset_t) (0x01U << i);
      good = (current == (Eventset_t) 0x01U) ? (Eventset_t) 0x04U :
	(Eventset_t) 0x01U;
      bad = (current == (Eventset_t) 0x02U) ? (Eventset_t) 0x08U :
	(Eventset_t) 0x02U;
      rc = vs_event_create (&tevent, name, current);
      if (rc == VSTATUS_OK)
	{
	  rc = dothread_events (tevent.event_handle,
				~current,
				good,
				bad, WAIT_ONE_SECOND,
				(uint32_t) 0U,
				WAIT_ONE_SECOND_MIN,
				WAIT_ONE_SECOND_MAX,
				WAIT_ONE_SECOND * (uint64_t) 2U,
				thread_timeout_events,
				STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  (void) vs_event_delete (&tevent);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_4d (void)
{
  static const char passed[] = "vs_event_create:1:4.d PASSED";
  static const char failed[] = "vs_event_create:1:4.d FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint32_t i;
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  for (i = (uint32_t) 0U; i < NUM_EVENTS; i++)
    {
      current = (Eventset_t) (0x01U << i);
      good = (current == (Eventset_t) 0x01U) ? (Eventset_t) 0x04U :
	(Eventset_t) 0x01U;
      bad = (current == (Eventset_t) 0x02U) ? (Eventset_t) 0x08U :
	(Eventset_t) 0x02U;
      rc = vs_event_create (&tevent, name, current);
      if (rc == VSTATUS_OK)
	{
	  rc = dothread_events (tevent.event_handle,
				(Eventset_t) 0xFFFFFFFFU,
				good,
				bad, WAIT_ONE_SECOND,
				current,
				(uint64_t) 0U,
				WAIT_ONE_SECOND_MIN,
				WAIT_ONE_SECOND, thread_get_events,
				STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  (void) vs_event_delete (&tevent);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_5a (void)
{
  static const char passed[] = "vs_event_create:1:5.a PASSED";
  static const char failed[] = "vs_event_create:1:5.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  rc = vs_event_create (&tevent, name, (Eventset_t) 0xFFFFFFFFU);
  if (rc == VSTATUS_OK)
    {
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_5b (void)
{
  static const char passed[] = "vs_event_create:1:5.b PASSED";
  static const char failed[] = "vs_event_create:1:5.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint32_t i;
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  for (i = (uint32_t) 0U; i < NUM_EVENTS; i++)
    {
      current = (Eventset_t) (0x01U << i);
      good = (current == (Eventset_t) 0x01U) ? (Eventset_t) 0x04U :
	(Eventset_t) 0x01U;
      bad = (current == (Eventset_t) 0x02U) ? (Eventset_t) 0x08U :
	(Eventset_t) 0x02U;
      created = (Eventset_t) 0xFFFFFFFFU ^ good ^ bad;
      rc = vs_event_create (&tevent, name, created);
      if (rc == VSTATUS_OK)
	{
	  rc = dothread_events (tevent.event_handle,
				current,
				good,
				bad, WAIT_ONE_SECOND,
				current, (uint64_t) 0U,
				WAIT_ONE_SECOND_MIN,
				WAIT_ONE_SECOND, thread_get_events,
				STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  (void) vs_event_delete (&tevent);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_5c (void)
{
  static const char passed[] = "vs_event_create:1:5.c PASSED";
  static const char failed[] = "vs_event_create:1:5.c FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  current = (Eventset_t) 0x80000001U;
  good = (Eventset_t) 0x04U;
  bad = (Eventset_t) 0x08U;
  created = (Eventset_t) 0xFFFFFFFFU ^ good ^ bad;
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_5d (void)
{
  static const char passed[] = "vs_event_create:1:5.d PASSED";
  static const char failed[] = "vs_event_create:1:5.d FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  good = (Eventset_t) 0x04U;
  bad = (Eventset_t) 0x08U;
  created = (Eventset_t) 0xFFFFFFFFU ^ good ^ bad;
  current = created;
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_5e (void)
{
  static const char passed[] = "vs_event_create:1:5.e PASSED";
  static const char failed[] = "vs_event_create:1:5.e FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  good = (Eventset_t) 0x04U;
  bad = (Eventset_t) 0x08U;
  created = (Eventset_t) 0xFFFFFFFFU ^ good ^ bad;
  current = (Eventset_t) 0x00018000U;
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_6a (void)
{
  static const char passed[] = "vs_event_create:1:6.a PASSED";
  static const char failed[] = "vs_event_create:1:6.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  rc = vs_event_create (&tevent, name, (Eventset_t) 0x80000001U);
  if (rc == VSTATUS_OK)
    {
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_6b (void)
{
  static const char passed[] = "vs_event_create:1:6.b PASSED";
  static const char failed[] = "vs_event_create:1:6.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  good = (Eventset_t) 0x04U;
  bad = (Eventset_t) 0x08U;
  created = (Eventset_t) 0x80000001U;
  current = (Eventset_t) 0x80000000U;
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_6c (void)
{
  static const char passed[] = "vs_event_create:1:6.c PASSED";
  static const char failed[] = "vs_event_create:1:6.c FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  good = (Eventset_t) 0x04U;
  bad = (Eventset_t) 0x08U;
  created = (Eventset_t) 0x80000001U;
  current = (Eventset_t) 0x00000001U;
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_create_6d (void)
{
  static const char passed[] = "vs_event_create:1:6.d PASSED";
  static const char failed[] = "vs_event_create:1:6.d FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t created;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  good = (Eventset_t) 0x04U;
  bad = (Eventset_t) 0x08U;
  created = (Eventset_t) 0x80000001U;
  current = (Eventset_t) 0x80000001U;
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

#if defined(__LINUX__)
static Status_t
vs_event_create_7a (void)
{
  static const char passed[] = "vs_event_create:1:7.a PASSED";
  static const char failed[] = "vs_event_create:1:7.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  uint32_t i;
  uint32_t j;
  Event_t cbs[33];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  VEVENT_CLRALL (current);
  for (i = (uint32_t) 0U; i < (uint32_t) 32U; i++)
    {
      (void) memset (&cbs[i], 0, sizeof (Event_t));
      rc = vs_event_create (&cbs[i], name, current);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_create failed; actual", rc);
	  for (j = (uint32_t) 0U; j < (uint32_t) i; j++)
	    {
	      (void) vs_event_delete (&cbs[j]);
	    }
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }
  rc = vs_event_create (&cbs[32], name, current);
  if (rc != VSTATUS_NODEV)
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_NODEV);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      for (j = (uint32_t) 0U; j < (uint32_t) 32U; j++)
	{
	  (void) vs_event_delete (&cbs[j]);
	}
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  for (j = (uint32_t) 0U; j < (uint32_t) 32U; j++)
    {
      (void) vs_event_delete (&cbs[j]);
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}
#endif
void
test_event_create_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_event_create:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_event_create_1a, total_passes, total_fails);
  DOATEST (vs_event_create_2a, total_passes, total_fails);
  DOATEST (vs_event_create_2b, total_passes, total_fails);
  DOATEST (vs_event_create_3a, total_passes, total_fails);
  DOATEST (vs_event_create_3b, total_passes, total_fails);
  DOATEST (vs_event_create_3c, total_passes, total_fails);
  DOATEST (vs_event_create_4a, total_passes, total_fails);
  DOATEST (vs_event_create_4b, total_passes, total_fails);
  DOATEST (vs_event_create_4c, total_passes, total_fails);
  DOATEST (vs_event_create_4d, total_passes, total_fails);
  DOATEST (vs_event_create_5a, total_passes, total_fails);
  DOATEST (vs_event_create_5b, total_passes, total_fails);
  DOATEST (vs_event_create_5c, total_passes, total_fails);
  DOATEST (vs_event_create_5d, total_passes, total_fails);
  DOATEST (vs_event_create_5e, total_passes, total_fails);
  DOATEST (vs_event_create_6a, total_passes, total_fails);
  DOATEST (vs_event_create_6b, total_passes, total_fails);
  DOATEST (vs_event_create_6c, total_passes, total_fails);
  DOATEST (vs_event_create_6d, total_passes, total_fails);
#if defined(__LINUX__)
  DOATEST (vs_event_create_7a, total_passes, total_fails);
#else
  IB_LOG_INFO ("vs_event_create:1:7.a SKIPPED", (uint32_t) 0U);
#endif
  IB_LOG_INFO ("vs_event_create:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_event_create:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_event_create:1 TEST COMPLETE", (uint32_t) 0U);
  return;
}

static Status_t
vs_event_wait_1a (void)
{
  static const char passed[] = "vs_event_wait:1:1.a PASSED";
  static const char failed[] = "vs_event_wait:1:1.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t events;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  rc = vs_event_create (&tevent, name, (Eventset_t) 0x00U);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_event_wait (0, WAIT_ONE_SECOND, (Eventset_t) 0xFFFFFFFFU,
			   &events);
      if (rc != VSTATUS_ILLPARM)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_ILLPARM);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_wait_1b (void)
{
  static const char passed[] = "vs_event_wait:1:1.b PASSED";
  static const char failed[] = "vs_event_wait:1:1.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Evt_handle_t ev;
  Eventset_t events;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  rc = vs_event_create (&tevent, name, (Eventset_t) 0x00U);
  if (rc == VSTATUS_OK)
    {
      ev = tevent.event_handle;
      (void) vs_event_delete (&tevent);
      rc =
	vs_event_wait (ev, WAIT_ONE_SECOND, (Eventset_t) 0xFFFFFFFFU,
			   &events);
      if (rc != VSTATUS_NXIO)
	{
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_NXIO);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_wait_2a (void)
{
  static const char passed[] = "vs_event_wait:1:2.a PASSED";
  static const char failed[] = "vs_event_wait:1:2.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t events;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  rc = vs_event_create (&tevent, name, (Eventset_t) 0x00U);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			   (Eventset_t) 0x00U, &events);
      if (rc != VSTATUS_ILLPARM)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_ILLPARM);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_wait_3a (void)
{
  static const char passed[] = "vs_event_wait:1:3.a PASSED";
  static const char failed[] = "vs_event_wait:1:3.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  rc = vs_event_create (&tevent, name, (Eventset_t) 0x00U);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			   (Eventset_t) 0x01U, 0);
      if (rc != VSTATUS_ILLPARM)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_ILLPARM);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_wait_4a (void)
{
  static const char passed[] = "vs_event_wait:1:4.a PASSED";
  static const char failed[] = "vs_event_wait:1:4.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint64_t delay;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  rc = vs_event_create (&tevent, name, (Eventset_t) 0U);
  if (rc == VSTATUS_OK)
    {
      /*
       ** Test clear from a different thread
       */
      delay = WAIT_ONE_SECOND >> 1;
      rc =
	dothread_events (tevent.event_handle,
			 (Eventset_t) 0xFFFFFFFFU,
			 (Eventset_t) 0x01U, (Eventset_t) 0x02U,
			 delay,
			 (Eventset_t) 0x00U,
			 delay - NOWAIT_MAX,
			 delay + NOWAIT_MAX,
			 WAIT_ONE_SECOND, thread_timeout_events,
			 STACKSIZE);
      if (rc == VSTATUS_OK)
	{
	  rc = VSTATUS_OK;
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_wait_4b (void)
{
  static const char passed[] = "vs_event_wait:1:4.b PASSED";
  static const char failed[] = "vs_event_wait:1:4.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  uint64_t delay;
  Eventset_t created;
  Eventset_t compare;
  Eventset_t wait;
  Eventset_t good;
  Eventset_t bad;


  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (created);
  VEVENT_CLRALL (compare);
  VEVENT_CLRALL (good);
  VEVENT_SET (good, 0U);
  VEVENT_CLRALL (bad);
  VEVENT_SET (bad, 1U);
  VEVENT_SETALL (wait);
  rc = vs_event_create (&tevent, name, created);
  if (rc == VSTATUS_OK)
    {
      /*
       ** Test clear from a different thread
       */
      delay = WAIT_ONE_SECOND * 4U;
      rc =
	dothread_events (tevent.event_handle,
			 wait,
			 good, bad,
			 delay,
			 compare,
			 delay - NOWAIT_MAX,
			 delay + NOWAIT_MAX,
			 delay + WAIT_ONE_SECOND, thread_timeout_events,
			 STACKSIZE);
      if (rc == VSTATUS_OK)
	{
	  rc = VSTATUS_OK;
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_wait_5a (void)
{
  static const char passed[] = "vs_event_wait:1:5.a PASSED";
  static const char failed[] = "vs_event_wait:1:5.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t events;
  uint32_t i;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  VEVENT_CLRALL (good);
  VEVENT_SET (good, 0x00U);
  VEVENT_CLRALL (bad);
  VEVENT_SET (bad, 0x01U);
  VEVENT_CLRALL (current);
  for (i = (uint32_t) 8U; i < (uint32_t) 23U; i++)
    {
      VEVENT_SET (current, i);
    }
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    current,
			    good,
			    bad, WAIT_ONE_SECOND,
			    current, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (events);
      rc =
	dothread_events (tevent.event_handle,
			 current,
			 good, bad,
			 WAIT_ONE_SECOND,
			 events,
			 WAIT_ONE_SECOND_MIN,
			 WAIT_ONE_SECOND_MAX,
			 WAIT_ONE_SECOND * (uint64_t) 2U,
			 thread_timeout_events,
			 STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_event_wait_5b (void)
{
  static const char passed[] = "vs_event_wait:1:5.b PASSED";
  static const char failed[] = "vs_event_wait:1:5.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t compare;
  Eventset_t good;
  Eventset_t bad;
  Eventset_t events;
  uint32_t i;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;

  VEVENT_CLRALL (good);
  VEVENT_SET (good, 0x00U);
  VEVENT_CLRALL (bad);
  VEVENT_SET (bad, 0x01U);
  VEVENT_CLRALL (current);
  for (i = (uint32_t) 8U; i < (uint32_t) 32U; i++)
    {
      VEVENT_SET (current, i);
    }
  VEVENT_CLRALL (compare);
  for (i = (uint32_t) 8U; i < (uint32_t) 24U; i++)
    {
      VEVENT_SET (compare, i);
    }
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      rc = dothread_events (tevent.event_handle,
			    compare,
			    good,
			    bad, WAIT_ONE_SECOND,
			    compare, (uint64_t) 0U,
			    WAIT_ONE_SECOND_MIN,
			    WAIT_ONE_SECOND, thread_get_events,
			    STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (events);
      for (i = (uint32_t) 24U; i < (uint32_t) 32U; i++)
	{
	  VEVENT_SET (events, i);
	}
      rc =
	dothread_events (tevent.event_handle,
			 current,
			 good, bad,
			 WAIT_ONE_SECOND,
			 events,
			 (uint64_t) 0U,
			 WAIT_ONE_SECOND_MIN,
			 WAIT_ONE_SECOND, thread_get_events,
			 STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}
void
test_event_wait_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_event_wait:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_event_wait_1a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_1b, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_2a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_3a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_4a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_4b, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_5a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_wait_5b, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_event_wait:1 TOTAL PASSED", total_passes);
WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_event_wait:1 TOTAL FAILED", total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_event_wait:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}

static Status_t
vs_event_post_1a (void)
{
  static const char passed[] = "vs_event_post:1:1.a PASSED";
  static const char failed[] = "vs_event_post:1:1.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_SET (current, 0U);
      rc = vs_event_post (0, VEVENT_WAKE_ONE, current);
      if (rc == VSTATUS_ILLPARM)
	{
	  rc = VSTATUS_OK;
	  (void) vs_event_delete (&tevent);
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_ILLPARM);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_post_2a (void)
{
  static const char passed[] = "vs_event_post:1:2.a PASSED";
  static const char failed[] = "vs_event_post:1:2.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_SET (current, 0U);
      rc = vs_event_post (tevent.event_handle, 7, current);
      if (rc == VSTATUS_ILLPARM)
	{
	  rc = VSTATUS_OK;
	  (void) vs_event_delete (&tevent);
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_ILLPARM);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_post_3a (void)
{
  static const char passed[] = "vs_event_post:1:3.a PASSED";
  static const char failed[] = "vs_event_post:1:3.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE, current);
      if (rc == VSTATUS_ILLPARM)
	{
	  rc = VSTATUS_OK;
	  (void) vs_event_delete (&tevent);
	  IB_LOG_INFO (passed, (uint32_t) 0U);
	}
      else
	{
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_ILLPARM);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  rc = VSTATUS_BAD;
	}
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_event_post_4a (void)
{
  static const char passed[] = "vs_event_post:1:4.a PASSED";
  static const char failed[] = "vs_event_post:1:4.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  uint32_t i;
#define NUM_THREADS (4U)
  Thread_t handle[NUM_THREADS];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 1U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 2U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;
      for (i = (uint32_t) 0U; i < (uint32_t) NUM_THREADS; i++)
	{
	  rc =
	    vs_thread_create (&handle[i], name,
				  thread_get_events, 0, (void *) &handle[i],
				  STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);	/* current gets both the OK and BAD events */
      VEVENT_SET (current, 2U);
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      for (i = (uint32_t) 0U; i < (uint32_t) NUM_THREADS; i++)
	{
	  rc =
	    vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			       Gwait_mask);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  VEVENT_CLRALL (events);
	  rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
				  current, &events);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  if (VEVENT_TST (events, 2U))
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }

	}
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      Gwait_mask, &events);
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post_4b (void)
{
  static const char passed[] = "vs_event_post:1:4.b PASSED";
  static const char failed[] = "vs_event_post:1:4.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle[2];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_SET (Gwait_mask, 1U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 1U);
      VEVENT_SET (Await_mask, 2U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 1U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 3U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 4U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle[0], name, thread_get_events,
			      0U, (void *) &handle[0],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      rc =
	vs_thread_create (&handle[1], name,
			      threadA_get_events, 0U, (void *) &handle[1],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_CLRALL (events);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post_5a (void)
{
  static const char passed[] = "vs_event_post:1:5.a PASSED";
  static const char failed[] = "vs_event_post:1:5.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle[2];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 0U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 0U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 5U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 6U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle[0], name, thread_get_events,
			      0U, (void *) &handle[0],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      rc =
	vs_thread_create (&handle[1], name,
			      threadA_get_events, 0U, (void *) &handle[1],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 5U);
      VEVENT_SET (current, 6U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 6U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_CLRALL (events);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post_5b (void)
{
  static const char passed[] = "vs_event_post:1:5.b PASSED";
  static const char failed[] = "vs_event_post:1:5.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle[2];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_SET (Gwait_mask, 1U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 1U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 1U);
      VEVENT_SET (Await_mask, 2U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 1U);
      VEVENT_SET (Acompare_mask, 2U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 5U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 6U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle[0], name, thread_get_events,
			      0U, (void *) &handle[0],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      rc =
	vs_thread_create (&handle[1], name,
			      threadA_get_events, 0U, (void *) &handle[1],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_SET (current, 2U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 5U);
      VEVENT_SET (current, 6U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 6U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post_5c (void)
{
  static const char passed[] = "vs_event_post:1:5.c PASSED";
  static const char failed[] = "vs_event_post:1:5.c FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle[2];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_SET (Gwait_mask, 1U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 1U);
      VEVENT_SET (Await_mask, 2U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 1U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 3U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 4U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle[0], name, thread_get_events,
			      0U, (void *) &handle[0],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      rc =
	vs_thread_create (&handle[1], name,
			      threadA_get_events, 0U, (void *) &handle[1],
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_CLRALL (events);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &events);
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}
void
test_event_post_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_event_post:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_event_post_1a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_2a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_3a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_4a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_4b, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_5a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_5b, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_post_5c, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_event_post:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_event_post:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_event_post:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}

static Status_t
vs_event_delete_1a (void)
{
  static const char passed[] = "vs_event_delete:1:1.a PASSED";
  static const char failed[] = "vs_event_delete:1:1.a FAILED";
  Status_t rc;

  rc = vs_event_delete (0);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_event_delete failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_event_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);

  return VSTATUS_OK;
}
static void
thread_delete_events ( /*@unused@ */ uint32_t argc,	/*@unused@ */
		      uint8_t * argv[])
{
  (void) vs_thread_sleep (WAIT_ONE_SECOND * (uint64_t) 2U);
  (void) vs_event_delete (&tevent);
  (void) vs_thread_exit ((Thread_t *) argv);
  return;
}

static Status_t
vs_event_delete_2a (void)
{
  static const char passed[] = "vs_event_delete:1:2.a PASSED";
  static const char failed[] = "vs_event_delete:1:2.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      rc =
	vs_thread_create (&handle, name, thread_delete_events,
			      0U, (void *) &handle,
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);
      rc =
	vs_event_wait (tevent.event_handle,
			   WAIT_ONE_SECOND * (uint64_t) 4U, current, &events);
      if (rc != VSTATUS_NXIO)
	{
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_NXIO);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_delete_3a (void)
{
  static const char passed[] = "vs_event_delete:1:3.a PASSED";
  static const char failed[] = "vs_event_delete:1:3.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      rc = vs_event_delete (&tevent);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_event_delete failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_delete failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}
void
test_event_delete_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_event_delete:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_event_delete_1a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_delete_2a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_event_delete_3a, total_passes, total_fails);
WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_event_delete:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_event_delete:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_event_delete:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}

static Status_t
vs_event_post2_1a (void)
{
  static const char passed[] = "vs_event_post:2:1.a PASSED";
  static const char failed[] = "vs_event_post:2:1.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  uint32_t i;
  Thread_t handle[NUM_THREADS];

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 1U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 2U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;
      /*
       * user space threads
       */
      for (i = (uint32_t) 0U; i < (uint32_t) NUM_THREADS; i++)
	{
	  rc =
	    vs_thread_create (&handle[i], name,
				  thread_get_events, 0U, (void *) &handle[i],
				  STACKSIZE);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	}

#if 0
      /*
       * kernel space threads
       */
      for (i = (uint32_t) 0U; i < (uint32_t) NUM_THREADS; i++)
	{
	  rc = vs_event_spacetest (tevent.event_handle, Gwait_mask,
				       Gcompare_mask, GpostBAD_mask,
				       GpostOK_mask, VEVENT_WAKE_ONE,
				       Gwaittime, Gmintime, Gmaxtime);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_event_spacetest failed; expected",
			    VSTATUS_OK);
	      IB_LOG_ERROR ("vs_event_spacetest failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	}
#endif

      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);	/* current gets both the OK and BAD events */
      VEVENT_SET (current, 2U);
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      for (i = (uint32_t) 0U; i < ((uint32_t) NUM_THREADS * (uint32_t) 2U);
	   i++)
	{
	  rc =
	    vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE,
			       Gwait_mask);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  VEVENT_CLRALL (events);
	  rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
				  current, &events);
	  if (rc != VSTATUS_OK)
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	  if (VEVENT_TST (events, 2U))
	    {
	      (void) vs_event_delete (&tevent);
	      IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }

	}
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      Gwait_mask, &events);
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post2_1b (void)
{
  static const char passed[] = "vs_event_post:2:1.b PASSED";
  static const char failed[] = "vs_event_post:2:1.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_SET (Gwait_mask, 1U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 1U);
      VEVENT_SET (Await_mask, 2U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 1U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 3U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 4U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle, name, thread_get_events,
			      0U, (void *) &handle,
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

#if 0
      rc = vs_event_spacetest (tevent.event_handle, Await_mask,
				   Acompare_mask,
				   ApostBAD_mask, ApostOK_mask,
				   VEVENT_WAKE_ONE, Awaittime, Amintime,
				   Amaxtime);

      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_spacetest failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_spacetest failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#endif
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ONE, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_CLRALL (events);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post2_2a (void)
{
  static const char passed[] = "vs_event_post:2:2.a PASSED";
  static const char failed[] = "vs_event_post:2:2.a FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 0U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 0U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 5U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 6U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle, name, thread_get_events,
			      0U, (void *) &handle,
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#if 0
      rc = vs_event_spacetest (tevent.event_handle, Await_mask,
				   Acompare_mask,
				   ApostBAD_mask, ApostOK_mask,
				   VEVENT_WAKE_ONE, Awaittime, Amintime,
				   Amaxtime);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_spacetest failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_spacetest failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#endif
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 5U);
      VEVENT_SET (current, 6U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 6U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_CLRALL (events);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post2_2b (void)
{
  static const char passed[] = "vs_event_post:2:2.b PASSED";
  static const char failed[] = "vs_event_post:2:2.b FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_SET (Gwait_mask, 1U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 1U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 1U);
      VEVENT_SET (Await_mask, 2U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 1U);
      VEVENT_SET (Acompare_mask, 2U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 5U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 6U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle, name, thread_get_events,
			      0U, (void *) &handle,
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#if 0
      rc = vs_event_spacetest (tevent.event_handle, Await_mask,
				   Acompare_mask,
				   ApostBAD_mask, ApostOK_mask,
				   VEVENT_WAKE_ONE, Awaittime, Amintime,
				   Amaxtime);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_spacetest failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_spacetest failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#endif
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_SET (current, 2U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 5U);
      VEVENT_SET (current, 6U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 6U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}

static Status_t
vs_event_post2_2c (void)
{
  static const char passed[] = "vs_event_post:2:2.c PASSED";
  static const char failed[] = "vs_event_post:2:2.c FAILED";
  Status_t rc;
  unsigned char name[VS_NAME_MAX];
  Eventset_t current;
  Eventset_t events;
  Thread_t handle;

  (void) memset (name, 0x08, (size_t) VS_NAME_MAX);
  name[VS_NAME_MAX - 1] = (unsigned char) 0x00U;
  VEVENT_CLRALL (current);
  rc = vs_event_create (&tevent, name, current);
  if (rc == VSTATUS_OK)
    {
      VEVENT_CLRALL (Gwait_mask);
      VEVENT_SET (Gwait_mask, 0U);
      VEVENT_SET (Gwait_mask, 1U);
      VEVENT_CLRALL (Gcompare_mask);
      VEVENT_SET (Gcompare_mask, 0U);
      VEVENT_CLRALL (GpostOK_mask);
      VEVENT_SET (GpostOK_mask, 3U);
      VEVENT_CLRALL (GpostBAD_mask);
      VEVENT_SET (GpostBAD_mask, 4U);
      Gwaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Gmintime = (uint64_t) 0U;
      Gmaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      VEVENT_CLRALL (Await_mask);
      VEVENT_SET (Await_mask, 1U);
      VEVENT_SET (Await_mask, 2U);
      VEVENT_CLRALL (Acompare_mask);
      VEVENT_SET (Acompare_mask, 1U);
      VEVENT_CLRALL (ApostOK_mask);
      VEVENT_SET (ApostOK_mask, 3U);
      VEVENT_CLRALL (ApostBAD_mask);
      VEVENT_SET (ApostBAD_mask, 4U);
      Awaittime = WAIT_ONE_SECOND * (uint64_t) 10U;
      Amintime = (uint64_t) 0U;
      Amaxtime = WAIT_ONE_SECOND * (uint64_t) 50U;

      rc =
	vs_thread_create (&handle, name, thread_get_events,
			      0U, (void *) &handle,
			      STACKSIZE);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_thread_create failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#if 0
      rc = vs_event_spacetest (tevent.event_handle, Await_mask,
				   Acompare_mask,
				   ApostBAD_mask, ApostOK_mask,
				   VEVENT_WAKE_ONE, Awaittime, Amintime,
				   Amaxtime);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_spacetest failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_spacetest failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
#endif
      (void) vs_thread_sleep (WAIT_ONE_SECOND);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 0U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);

      rc = vs_event_post (tevent.event_handle, VEVENT_WAKE_ALL, current);
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_post failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_post failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (events);
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 3U);
      VEVENT_SET (current, 4U);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_OK)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (VEVENT_TST (events, 4U))
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("bad status returned", (uint32_t) 0U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      VEVENT_CLRALL (current);
      VEVENT_SET (current, 1U);
      VEVENT_CLRALL (events);
      rc = vs_event_wait (tevent.event_handle, WAIT_ONE_SECOND,
			      current, &(events));
      if (rc != VSTATUS_TIMEOUT)
	{
	  (void) vs_event_delete (&tevent);
	  IB_LOG_ERROR ("vs_event_wait failed; expected", VSTATUS_TIMEOUT);
	  IB_LOG_ERROR ("vs_event_wait failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      (void) vs_event_delete (&tevent);
      IB_LOG_INFO (passed, (uint32_t) 0U);
    }
  else
    {
      IB_LOG_ERROR ("vs_event_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_event_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  return VSTATUS_OK;

}
void
test_event_post_2 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_event_post:2 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_event_post2_1a, total_passes, total_fails);
  DOATEST (vs_event_post2_1b, total_passes, total_fails);
  DOATEST (vs_event_post2_2a, total_passes, total_fails);
  DOATEST (vs_event_post2_2b, total_passes, total_fails);
  DOATEST (vs_event_post2_2c, total_passes, total_fails);
  IB_LOG_INFO ("vs_event_post:2 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_event_post:2 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_event_post:2 TEST COMPLETE", (uint32_t) 0U);

  return;
}
