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
*      vs_timeget_test.c
*
* DESCRIPTION
*      This file contains vs_time_get common test routines.
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
* DKJ       02/28/02    Initial creation of file.
***********************************************************************/
#include <ib_types.h>
#include <vs_g.h>
#include <cs_log.h>
#define DOATEST(func, pass, fail) ((func)() == VSTATUS_OK) ? pass++ : fail++
static Status_t
vs_time_get_1a (void)
{
  static const char passed[] = "vs_time_get:1:1.a PASSED";
  static const char failed[] = "vs_time_get:1:1.a FAILED";
  Status_t rc;

  rc = vs_time_get (0);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_time_get failed: expected: ", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_time_get failed: actual: ", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_time_get_2a (void)
{
  static const char passed[] = "vs_time_get:1:2.a PASSED";
  static const char failed[] = "vs_time_get:1:2.a FAILED";
  Status_t rc;
  uint64_t now = (uint64_t) 0U;
  uint64_t previous;
  uint32_t i;

  rc = vs_time_get (&now);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_time_get failed: expected: ", VSTATUS_OK);
      IB_LOG_ERROR ("vs_time_get failed: actual: ", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }
  for (i = (uint32_t) 0U; i < (uint32_t) 1000U; i++)
    {
      previous = now;
      rc = vs_time_get (&now);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_time_get failed: expected: ", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_time_get failed: actual: ", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      if (now < previous)
	{
	  IB_LOG_ERROR ("Decreasing time sequence: previous[L]",
			(uint32_t) previous);
	  IB_LOG_ERROR ("Decreasing time sequence: previous[H]",
			(uint32_t) (previous >> 32));
	  IB_LOG_ERROR ("Decreasing time sequence: now[L]", (uint32_t) now);
	  IB_LOG_ERROR ("Decreasing time sequence: now[H]",
			(uint32_t) (now >> 32));
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

void
test_time_get_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_time_get:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_time_get_1a, total_passes, total_fails);
  DOATEST (vs_time_get_2a, total_passes, total_fails);
  IB_LOG_INFO ("vs_time_get:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_time_get:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_time_get:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}
