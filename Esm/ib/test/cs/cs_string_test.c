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
*      cs_string_test.c
*
* DESCRIPTION
*      This file contains String Coversion common test routines.
*      These tests exercise the String Conversion services.
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
* MGR       05/01/02    PR1754.  Initial creation of file.
* MGR       05/07/02    PR1754.  Bug fixes during testing.
* MGR       05/08/02    PR1812.  Clean-up lint errors/warnings.
* PJG       05/28/02    PR2166.  Add to ATI build.
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
#define RET_ON_ERROR_INT8            0x0000000000000070
#define RET_ON_ERROR_UINT8           0x00000000000000F0
#define RET_ON_ERROR_INT16           0x0000000000007FF0
#define RET_ON_ERROR_UINT16          0x000000000000FFF0
#define RET_ON_ERROR_INT32           0x000000007FFFFFF0
#define RET_ON_ERROR_UINT32          0x00000000FFFFFFF0
#define RET_ON_ERROR_INT64           0x7FFFFFFFFFFFFFF0ll
#define RET_ON_ERROR_UINT64          0xFFFFFFFFFFFFFFF0ull

/**********************************************************************
* Variable Declarations
***********************************************************************/

/*
** Sub-test Variations (cs_strtoui8)
*/
static Status_t
cs_strtoui8_1a (void)
{
  char              *p;
  uint8_t           ui8Value;
  static const char passed[] = "cs_strtoui8:1:1.a PASSED";
  static const char failed[] = "cs_strtoui8:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == RET_ON_ERROR_UINT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)RET_ON_ERROR_UINT8);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_1b (void)
{
  char              *p;
  uint8_t           ui8Value;
  static const char passed[] = "cs_strtoui8:1:1.b PASSED";
  static const char failed[] = "cs_strtoui8:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xag";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == RET_ON_ERROR_UINT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)RET_ON_ERROR_UINT8);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_1c (void)
{
  char              *p;
  uint8_t           ui8Value;
  static const char passed[] = "cs_strtoui8:1:1.c PASSED";
  static const char failed[] = "cs_strtoui8:1:1.c FAILED";

  /* set pointer to point to a negative number */
  p = "-1";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == RET_ON_ERROR_UINT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)RET_ON_ERROR_UINT8);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_1d (void)
{
  char              *p;
  uint8_t           ui8Value;
  static const char passed[] = "cs_strtoui8:1:1.d PASSED";
  static const char failed[] = "cs_strtoui8:1:1.d FAILED";

  /* set pointer to point to a number greater than max uint8_t value */
  p = "256";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == RET_ON_ERROR_UINT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)RET_ON_ERROR_UINT8);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2a (void)
{
  char              *p;
  uint8_t           ui8Value = 0;
  static const char passed[] = "cs_strtoui8:1:2.a PASSED";
  static const char failed[] = "cs_strtoui8:1:2.a FAILED";

  /* set pointer to point to maximum valid decimal number */
  p = "255";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == UINT8_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)UINT8_MAX);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2b (void)
{
  char              *p;
  uint8_t           ui8Value = 0;
  static const char passed[] = "cs_strtoui8:1:2.b PASSED";
  static const char failed[] = "cs_strtoui8:1:2.b FAILED";

  /* set pointer to point to minimum valid decimal number */
  p = "0";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == UINT8_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)UINT8_MIN);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2c (void)
{
  uint8_t           ui8Value = 0;
  char     *p;
  static const char passed[] = "cs_strtoui8:1:2.c PASSED";
  static const char failed[] = "cs_strtoui8:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a leading + */
  p = "+237";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == 237)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)237U);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2d (void)
{
  uint8_t           ui8Value = 0;
  char     *p;
  static const char passed[] = "cs_strtoui8:1:2.d PASSED";
  static const char failed[] = "cs_strtoui8:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, containing spaces */
  p = "  130";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == 130)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)130U);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2e (void)
{
  char              *p;
  uint8_t           ui8Value = 0;
  static const char passed[] = "cs_strtoui8:1:2.e PASSED";
  static const char failed[] = "cs_strtoui8:1:2.e FAILED";

  /* set pointer to point to maximum valid hexadecimal number */
  p = "0xFF";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == UINT8_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)UINT8_MAX);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2f (void)
{
  char              *p;
  uint8_t           ui8Value = 0;
  static const char passed[] = "cs_strtoui8:1:2.f PASSED";
  static const char failed[] = "cs_strtoui8:1:2.f FAILED";

  /* set pointer to point to minimum valid hexdecimal number */
  p = "0x00";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == UINT8_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)UINT8_MIN);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui8_2g (void)
{
  char              *p;
  uint8_t           ui8Value = 0;
  static const char passed[] = "cs_strtoui8:1:2.g PASSED";
  static const char failed[] = "cs_strtoui8:1:2.g FAILED";

  /* set pointer to point to a valid hexadecimal number, with spaces */
  p = "     0x28";

  ui8Value = cs_strtoui8 (p, RET_ON_ERROR_UINT8);
  if (ui8Value == 0x28)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui8 failed; expected", (uint32_t)0x28);
    IB_LOG_ERROR ("cs_strtoui8 failed; actual", (uint32_t)ui8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoui8_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoui8:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoui8_1a, total_passes, total_fails);
  DOATEST (cs_strtoui8_1b, total_passes, total_fails);
  DOATEST (cs_strtoui8_1c, total_passes, total_fails);
  DOATEST (cs_strtoui8_1d, total_passes, total_fails);
  DOATEST (cs_strtoui8_2a, total_passes, total_fails);
  DOATEST (cs_strtoui8_2b, total_passes, total_fails);
  DOATEST (cs_strtoui8_2c, total_passes, total_fails);
  DOATEST (cs_strtoui8_2d, total_passes, total_fails);
  DOATEST (cs_strtoui8_2e, total_passes, total_fails);
  DOATEST (cs_strtoui8_2f, total_passes, total_fails);
  DOATEST (cs_strtoui8_2g, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoui8:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoui8:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoui8:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoui16)
*/
static Status_t
cs_strtoui16_1a (void)
{
  char              *p;
  uint16_t          ui16Value;
  static const char passed[] = "cs_strtoui16:1:1.a PASSED";
  static const char failed[] = "cs_strtoui16:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == RET_ON_ERROR_UINT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)RET_ON_ERROR_UINT16);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_1b (void)
{
  char              *p;
  uint16_t          ui16Value;
  static const char passed[] = "cs_strtoui16:1:1.b PASSED";
  static const char failed[] = "cs_strtoui16:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xabcg";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == RET_ON_ERROR_UINT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)RET_ON_ERROR_UINT16);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_1c (void)
{
  char              *p;
  uint16_t          ui16Value;
  static const char passed[] = "cs_strtoui16:1:1.c PASSED";
  static const char failed[] = "cs_strtoui16:1:1.c FAILED";

  /* set pointer to point to a negative number */
  p = "-1";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == RET_ON_ERROR_UINT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)RET_ON_ERROR_UINT16);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_1d (void)
{
  char              *p;
  uint16_t          ui16Value;
  static const char passed[] = "cs_strtoui16:1:1.d PASSED";
  static const char failed[] = "cs_strtoui16:1:1.d FAILED";

  /* set pointer to point to a number greater than max uint16_t value */
  p = "65536";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == RET_ON_ERROR_UINT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)RET_ON_ERROR_UINT16);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2a (void)
{
  char              *p;
  uint16_t          ui16Value = 0;
  static const char passed[] = "cs_strtoui16:1:2.a PASSED";
  static const char failed[] = "cs_strtoui16:1:2.a FAILED";

  /* set pointer to point to maximum valid decimal number */
  p = "65535";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == UINT16_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)UINT16_MAX);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2b (void)
{
  char              *p;
  uint16_t          ui16Value = 0;
  static const char passed[] = "cs_strtoui16:1:2.b PASSED";
  static const char failed[] = "cs_strtoui16:1:2.b FAILED";

  /* set pointer to point to minimum valid decimal number */
  p = "0";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == UINT16_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)UINT16_MIN);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2c (void)
{
  char              *p;
  uint16_t          ui16Value = 0;
  static const char passed[] = "cs_strtoui16:1:2.c PASSED";
  static const char failed[] = "cs_strtoui16:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a leading + */
  p = "+46735";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == 46735)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)46735U);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2d (void)
{
  uint16_t          ui16Value = 0;
  char     *p;
  static const char passed[] = "cs_strtoui16:1:2.d PASSED";
  static const char failed[] = "cs_strtoui16:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, containing spaces */
  p = "  59384";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == 59384)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)59384U);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2e (void)
{
  char              *p;
  uint16_t           ui16Value = 0;
  static const char passed[] = "cs_strtoui16:1:2.e PASSED";
  static const char failed[] = "cs_strtoui16:1:2.e FAILED";

  /* set pointer to point to maximum valid hexadecimal number */
  p = "0xFFFF";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == UINT16_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)UINT16_MAX);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2f (void)
{
  char              *p;
  uint16_t          ui16Value = 0;
  static const char passed[] = "cs_strtoui16:1:2.f PASSED";
  static const char failed[] = "cs_strtoui16:1:2.f FAILED";

  /* set pointer to point to minimum valid hexdecimal number */
  p = "0x00";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == UINT16_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)UINT16_MIN);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui16_2g (void)
{
  char              *p;
  uint16_t          ui16Value = 0;
  static const char passed[] = "cs_strtoui16:1:2.g PASSED";
  static const char failed[] = "cs_strtoui16:1:2.g FAILED";

  /* set pointer to point to a valid hexadecimal number, with spaces */
  p = "     0xaBCd";

  ui16Value = cs_strtoui16 (p, RET_ON_ERROR_UINT16);
  if (ui16Value == 0xABCD)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui16 failed; expected", (uint32_t)0xABCD);
    IB_LOG_ERROR ("cs_strtoui16 failed; actual", (uint32_t)ui16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoui16_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoui16:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoui16_1a, total_passes, total_fails);
  DOATEST (cs_strtoui16_1b, total_passes, total_fails);
  DOATEST (cs_strtoui16_1c, total_passes, total_fails);
  DOATEST (cs_strtoui16_1d, total_passes, total_fails);
  DOATEST (cs_strtoui16_2a, total_passes, total_fails);
  DOATEST (cs_strtoui16_2b, total_passes, total_fails);
  DOATEST (cs_strtoui16_2c, total_passes, total_fails);
  DOATEST (cs_strtoui16_2d, total_passes, total_fails);
  DOATEST (cs_strtoui16_2e, total_passes, total_fails);
  DOATEST (cs_strtoui16_2f, total_passes, total_fails);
  DOATEST (cs_strtoui16_2g, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoui16:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoui16:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoui16:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoui32)
*/
static Status_t
cs_strtoui32_1a (void)
{
  char              *p;
  uint32_t          ui32Value;
  static const char passed[] = "cs_strtoui32:1:1.a PASSED";
  static const char failed[] = "cs_strtoui32:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == RET_ON_ERROR_UINT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)RET_ON_ERROR_UINT32);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_1b (void)
{
  char              *p;
  uint32_t          ui32Value;
  static const char passed[] = "cs_strtoui32:1:1.b PASSED";
  static const char failed[] = "cs_strtoui32:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xabcdefg";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == RET_ON_ERROR_UINT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)RET_ON_ERROR_UINT32);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_1c (void)
{
  char              *p;
  uint32_t          ui32Value;
  static const char passed[] = "cs_strtoui32:1:1.c PASSED";
  static const char failed[] = "cs_strtoui32:1:1.c FAILED";

  /* set pointer to point to a negative number */
  p = "-1";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == RET_ON_ERROR_UINT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)RET_ON_ERROR_UINT32);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_1d (void)
{
  char              *p;
  uint32_t          ui32Value;
  static const char passed[] = "cs_strtoui32:1:1.d PASSED";
  static const char failed[] = "cs_strtoui32:1:1.d FAILED";

  /* set pointer to point to a number greater than max uint32_t value */
  p = "4294967296";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == RET_ON_ERROR_UINT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)RET_ON_ERROR_UINT32);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2a (void)
{
  char              *p;
  uint32_t          ui32Value = 0;
  static const char passed[] = "cs_strtoui32:1:2.a PASSED";
  static const char failed[] = "cs_strtoui32:1:2.a FAILED";

  /* set pointer to point to maximum valid decimal number */
  p = "4294967295";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == UINT32_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)UINT32_MAX);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2b (void)
{
  char              *p;
  uint32_t          ui32Value = 0;
  static const char passed[] = "cs_strtoui32:1:2.b PASSED";
  static const char failed[] = "cs_strtoui32:1:2.b FAILED";

  /* set pointer to point to minimum valid decimal number */
  p = "0";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == UINT32_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)UINT32_MIN);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2c (void)
{
  char              *p;
  uint32_t          ui32Value = 0;
  static const char passed[] = "cs_strtoui32:1:2.c PASSED";
  static const char failed[] = "cs_strtoui32:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a leading + */
  p = "+1234567890";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == 1234567890)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)1234567890U);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2d (void)
{
  char              *p;
  uint32_t          ui32Value;
  static const char passed[] = "cs_strtoui32:1:2.d PASSED";
  static const char failed[] = "cs_strtoui32:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, containing spaces */
  p = "  2468135790";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == 2468135790U)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)2468135790U);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2e (void)
{
  char              *p;
  uint32_t           ui32Value = 0;
  static const char passed[] = "cs_strtoui32:1:2.e PASSED";
  static const char failed[] = "cs_strtoui32:1:2.e FAILED";

  /* set pointer to point to maximum valid hexadecimal number */
  p = "0xFFFFFFFF";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == UINT32_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)UINT32_MAX);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2f (void)
{
  char              *p;
  uint32_t          ui32Value = 0;
  static const char passed[] = "cs_strtoui32:1:2.f PASSED";
  static const char failed[] = "cs_strtoui32:1:2.f FAILED";

  /* set pointer to point to minimum valid hexdecimal number */
  p = "0x00000000";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == UINT32_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)UINT32_MIN);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui32_2g (void)
{
  char              *p;
  uint32_t          ui32Value = 0;
  static const char passed[] = "cs_strtoui32:1:2.g PASSED";
  static const char failed[] = "cs_strtoui32:1:2.g FAILED";

  /* set pointer to point to a valid hexadecimal number, with spaces */
  p = "     0xaBCdEF05";

  ui32Value = cs_strtoui32 (p, RET_ON_ERROR_UINT32);
  if (ui32Value == 0xABCDEF05)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui32 failed; expected", (uint32_t)0xABCDEF05);
    IB_LOG_ERROR ("cs_strtoui32 failed; actual", (uint32_t)ui32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoui32_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoui32:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoui32_1a, total_passes, total_fails);
  DOATEST (cs_strtoui32_1b, total_passes, total_fails);
  DOATEST (cs_strtoui32_1c, total_passes, total_fails);
  DOATEST (cs_strtoui32_1d, total_passes, total_fails);
  DOATEST (cs_strtoui32_2a, total_passes, total_fails);
  DOATEST (cs_strtoui32_2b, total_passes, total_fails);
  DOATEST (cs_strtoui32_2c, total_passes, total_fails);
  DOATEST (cs_strtoui32_2d, total_passes, total_fails);
  DOATEST (cs_strtoui32_2e, total_passes, total_fails);
  DOATEST (cs_strtoui32_2f, total_passes, total_fails);
  DOATEST (cs_strtoui32_2g, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoui32:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoui32:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoui32:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoui64)
*/
static Status_t
cs_strtoui64_1a (void)
{
  char              *p;
  uint64_t          ui64Value;
  static const char passed[] = "cs_strtoui64:1:1.a PASSED";
  static const char failed[] = "cs_strtoui64:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == RET_ON_ERROR_UINT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_UINT64 & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_UINT64 & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));

    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_1b (void)
{
  char              *p;
  uint64_t          ui64Value;
  static const char passed[] = "cs_strtoui64:1:1.b PASSED";
  static const char failed[] = "cs_strtoui64:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xabcdef012345678g";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == RET_ON_ERROR_UINT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_UINT64 & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_UINT64 & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));

    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_1c (void)
{
  char              *p;
  uint64_t          ui64Value;
  static const char passed[] = "cs_strtoui64:1:1.c PASSED";
  static const char failed[] = "cs_strtoui64:1:1.c FAILED";

  /* set pointer to point to a negative number */
  p = "-1";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == RET_ON_ERROR_UINT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_UINT64 & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_UINT64 & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2a (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.a PASSED";
  static const char failed[] = "cs_strtoui64:1:2.a FAILED";

  /* set pointer to point to maximum valid decimal number */
  p = "18446744073709551615";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == UINT64_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((UINT64_MAX & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(UINT64_MAX & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2b (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.b PASSED";
  static const char failed[] = "cs_strtoui64:1:2.b FAILED";

  /* set pointer to point to minimum valid decimal number */
  p = "0";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == UINT64_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((UINT64_MAX & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(UINT64_MIN & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2c (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.c PASSED";
  static const char failed[] = "cs_strtoui64:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a leading + */
  p = "+12345678901234567890";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == 12345678901234567890ULL)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((12345678901234567890ULL & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(12345678901234567890ULL & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2d (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.d PASSED";
  static const char failed[] = "cs_strtoui64:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, containing spaces */
  p = "  01234567890123456789";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == 1234567890123456789ULL)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((1234567890123456789ULL & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(1234567890123456789ULL & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2e (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.e PASSED";
  static const char failed[] = "cs_strtoui64:1:2.e FAILED";

  /* set pointer to point to maximum valid hexadecimal number */
  p = "0xFFFFFFFFFFFFFFFF";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == UINT64_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((UINT64_MAX & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(UINT64_MAX & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2f (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.f PASSED";
  static const char failed[] = "cs_strtoui64:1:2.f FAILED";

  /* set pointer to point to minimum valid hexdecimal number */
  p = "0x0000000000000000";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == UINT64_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)", 0U);
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)", 0U);
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoui64_2g (void)
{
  char              *p;
  uint64_t          ui64Value = 0;
  static const char passed[] = "cs_strtoui64:1:2.g PASSED";
  static const char failed[] = "cs_strtoui64:1:2.g FAILED";

  /* set pointer to point to a valid hexadecimal number, with spaces */
  p = "     0xaBCdEF0123456789";

  ui64Value = cs_strtoui64 (p, RET_ON_ERROR_UINT64);
  if (ui64Value == 0xABCDEF0123456789ull)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Hi)",
                  (uint32_t)((0xABCDEF0123456789ull & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; expected (Lo)",
                  (uint32_t)(0xABCDEF0123456789ull & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Hi)",
                  (uint32_t)((ui64Value & 0xFFFFFFFF00000000ull) >> 32));
    IB_LOG_ERROR ("cs_strtoui64 failed; actual (Lo)",
                  (uint32_t)(ui64Value & 0x00000000FFFFFFFFull));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoui64_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoui64:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoui64_1a, total_passes, total_fails);
  DOATEST (cs_strtoui64_1b, total_passes, total_fails);
  DOATEST (cs_strtoui64_1c, total_passes, total_fails);
  DOATEST (cs_strtoui64_2a, total_passes, total_fails);
  DOATEST (cs_strtoui64_2b, total_passes, total_fails);
  DOATEST (cs_strtoui64_2c, total_passes, total_fails);
  DOATEST (cs_strtoui64_2d, total_passes, total_fails);
  DOATEST (cs_strtoui64_2e, total_passes, total_fails);
  DOATEST (cs_strtoui64_2f, total_passes, total_fails);
  DOATEST (cs_strtoui64_2g, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoui64:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoui64:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoui64:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoi8)
*/
static Status_t
cs_strtoi8_1a (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:1.a PASSED";
  static const char failed[] = "cs_strtoi8:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == RET_ON_ERROR_INT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)RET_ON_ERROR_INT8);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_1b (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:1.b PASSED";
  static const char failed[] = "cs_strtoi8:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xag";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == RET_ON_ERROR_INT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)RET_ON_ERROR_INT8);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_1c (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:1.c PASSED";
  static const char failed[] = "cs_strtoi8:1:1.c FAILED";

  /* set pointer to point to a number less than min int8_t value */
  p = "-129";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == RET_ON_ERROR_INT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)RET_ON_ERROR_INT8);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_1d (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:1.d PASSED";
  static const char failed[] = "cs_strtoi8:1:1.d FAILED";

  /* set pointer to point to a number greater than max int8_t value */
  p = "128";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == RET_ON_ERROR_INT8)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)RET_ON_ERROR_INT8);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_2a (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:2.a PASSED";
  static const char failed[] = "cs_strtoi8:1:2.a FAILED";

  /* set pointer to point to a number equal to max int8_t value */
  p = "127";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == INT8_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)INT8_MAX);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_2b (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:2.b PASSED";
  static const char failed[] = "cs_strtoi8:1:2.b FAILED";

  /* set pointer to point to a number equal to min int8_t value */
  p = "-128";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == INT8_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)INT8_MIN);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_2c (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:2.c PASSED";
  static const char failed[] = "cs_strtoi8:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a + */
  p = "+096";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == 96)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)96);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_2d (void)
{
  char              *p;
  int8_t            i8Value;
  static const char passed[] = "cs_strtoi8:1:2.d PASSED";
  static const char failed[] = "cs_strtoi8:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, with spaces */
  p = "   -30";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == -30)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)-30);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_2e (void)
{
  char              *p;
  int8_t            i8Value = 0;
  static const char passed[] = "cs_strtoi8:1:2.e PASSED";
  static const char failed[] = "cs_strtoi8:1:2.e FAILED";

  /* set pointer to point to a hexadecimal number equal to max int8_t value */
  p = "0x7F";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == INT8_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)INT8_MAX);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi8_2f (void)
{
  char              *p;
  int8_t            i8Value = 0;
  static const char passed[] = "cs_strtoi8:1:2.f PASSED";
  static const char failed[] = "cs_strtoi8:1:2.f FAILED";

  /* set pointer to point to a valid hexadecimal number with spaces */
  p = "   0x28";

  i8Value = cs_strtoi8 (p, RET_ON_ERROR_INT8);
  if (i8Value == 0x28)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi8 failed; expected", (uint32_t)0x28);
    IB_LOG_ERROR ("cs_strtoi8 failed; actual", (uint32_t)(int32_t)i8Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoi8_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoi8:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoi8_1a, total_passes, total_fails);
  DOATEST (cs_strtoi8_1b, total_passes, total_fails);
  DOATEST (cs_strtoi8_1c, total_passes, total_fails);
  DOATEST (cs_strtoi8_1d, total_passes, total_fails);
  DOATEST (cs_strtoi8_2a, total_passes, total_fails);
  DOATEST (cs_strtoi8_2b, total_passes, total_fails);
  DOATEST (cs_strtoi8_2c, total_passes, total_fails);
  DOATEST (cs_strtoi8_2d, total_passes, total_fails);
  DOATEST (cs_strtoi8_2e, total_passes, total_fails);
  DOATEST (cs_strtoi8_2f, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoi8:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoi8:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoi8:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoi16)
*/
static Status_t
cs_strtoi16_1a (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:1.a PASSED";
  static const char failed[] = "cs_strtoi16:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == RET_ON_ERROR_INT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)RET_ON_ERROR_INT16);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_1b (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:1.b PASSED";
  static const char failed[] = "cs_strtoi16:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xabcg";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == RET_ON_ERROR_INT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)RET_ON_ERROR_INT16);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_1c (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:1.c PASSED";
  static const char failed[] = "cs_strtoi16:1:1.c FAILED";

  /* set pointer to point to a number less than min int16_t value */
  p = "-32769";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == RET_ON_ERROR_INT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)RET_ON_ERROR_INT16);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_1d (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:1.d PASSED";
  static const char failed[] = "cs_strtoi16:1:1.d FAILED";

  /* set pointer to point to a number greater than max int16_t value */
  p = "32768";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == RET_ON_ERROR_INT16)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)RET_ON_ERROR_INT16);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_2a (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:2.a PASSED";
  static const char failed[] = "cs_strtoi16:1:2.a FAILED";

  /* set pointer to point to a number equal to max int16_t value */
  p = "32767";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == INT16_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)INT16_MAX);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_2b (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:2.b PASSED";
  static const char failed[] = "cs_strtoi16:1:2.b FAILED";

  /* set pointer to point to a number equal to min int16_t value */
  p = "-32768";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == INT16_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)INT16_MIN);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_2c (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:2.c PASSED";
  static const char failed[] = "cs_strtoi16:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a + */
  p = "+30287";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == 30287)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)30287);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_2d (void)
{
  char              *p;
  int16_t           i16Value;
  static const char passed[] = "cs_strtoi16:1:2.d PASSED";
  static const char failed[] = "cs_strtoi16:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, with spaces */
  p = "   -19654";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == -19654)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)-19654);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_2e (void)
{
  char              *p;
  int16_t           i16Value = 0;
  static const char passed[] = "cs_strtoi16:1:2.e PASSED";
  static const char failed[] = "cs_strtoi16:1:2.e FAILED";

  /* set pointer to point to a hexadecimal number equal to max int16_t value */
  p = "0x7FFF";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == INT16_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)INT16_MAX);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi16_2f (void)
{
  char              *p;
  int16_t           i16Value = 0;
  static const char passed[] = "cs_strtoi16:1:2.f PASSED";
  static const char failed[] = "cs_strtoi16:1:2.f FAILED";

  /* set pointer to point to a valid hexadecimal number with spaces */
  p = "   0x5BcD";

  i16Value = cs_strtoi16 (p, RET_ON_ERROR_INT16);
  if (i16Value == 0x5BCD)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi16 failed; expected", (uint32_t)0x5BCD);
    IB_LOG_ERROR ("cs_strtoi16 failed; actual", (uint32_t)(int32_t)i16Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoi16_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoi16:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoi16_1a, total_passes, total_fails);
  DOATEST (cs_strtoi16_1b, total_passes, total_fails);
  DOATEST (cs_strtoi16_1c, total_passes, total_fails);
  DOATEST (cs_strtoi16_1d, total_passes, total_fails);
  DOATEST (cs_strtoi16_2a, total_passes, total_fails);
  DOATEST (cs_strtoi16_2b, total_passes, total_fails);
  DOATEST (cs_strtoi16_2c, total_passes, total_fails);
  DOATEST (cs_strtoi16_2d, total_passes, total_fails);
  DOATEST (cs_strtoi16_2e, total_passes, total_fails);
  DOATEST (cs_strtoi16_2f, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoi16:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoi16:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoi16:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoi32)
*/
static Status_t
cs_strtoi32_1a (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:1.a PASSED";
  static const char failed[] = "cs_strtoi32:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == RET_ON_ERROR_INT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)RET_ON_ERROR_INT32);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_1b (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:1.b PASSED";
  static const char failed[] = "cs_strtoi32:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xaBCdefg0";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == RET_ON_ERROR_INT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)RET_ON_ERROR_INT32);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_1c (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:1.c PASSED";
  static const char failed[] = "cs_strtoi32:1:1.c FAILED";

  /* set pointer to point to a number less than min int32_t value */
  p = "-2147483649";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == RET_ON_ERROR_INT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)RET_ON_ERROR_INT32);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_1d (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:1.d PASSED";
  static const char failed[] = "cs_strtoi32:1:1.d FAILED";

  /* set pointer to point to a number greater than max int32_t value */
  p = "2147483648";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == RET_ON_ERROR_INT32)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)RET_ON_ERROR_INT32);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_2a (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:2.a PASSED";
  static const char failed[] = "cs_strtoi32:1:2.a FAILED";

  /* set pointer to point to a number equal to max int32_t value */
  p = "2147483647";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == INT32_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)INT32_MAX);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_2b (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:2.b PASSED";
  static const char failed[] = "cs_strtoi32:1:2.b FAILED";

  /* set pointer to point to a number equal to min int32_t value */
  p = "-2147483648";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == INT32_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)INT32_MIN);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_2c (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:2.c PASSED";
  static const char failed[] = "cs_strtoi32:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a + */
  p = "+1234567890";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == 1234567890)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)1234567890);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_2d (void)
{
  char              *p;
  int32_t           i32Value;
  static const char passed[] = "cs_strtoi32:1:2.d PASSED";
  static const char failed[] = "cs_strtoi32:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, with spaces */
  p = "   -1234567890";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == -1234567890)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)-1234567890);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_2e (void)
{
  char              *p;
  int32_t           i32Value = 0;
  static const char passed[] = "cs_strtoi32:1:2.e PASSED";
  static const char failed[] = "cs_strtoi32:1:2.e FAILED";

  /* set pointer to point to a hexadecimal number equal to max int32_t value */
  p = "0x7FFFFFFF";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == INT32_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)INT32_MAX);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi32_2f (void)
{
  char              *p;
  int32_t           i32Value = 0;
  static const char passed[] = "cs_strtoi32:1:2.f PASSED";
  static const char failed[] = "cs_strtoi32:1:2.f FAILED";

  /* set pointer to point to a valid hexadecimal number with spaces */
  p = "   0x6aBcDef7";

  i32Value = cs_strtoi32 (p, RET_ON_ERROR_INT32);
  if (i32Value == 0x6ABCDEF7)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi32 failed; expected", (uint32_t)0x6ABCDEF7);
    IB_LOG_ERROR ("cs_strtoi32 failed; actual", (uint32_t)i32Value);
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoi32_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoi32:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoi32_1a, total_passes, total_fails);
  DOATEST (cs_strtoi32_1b, total_passes, total_fails);
  DOATEST (cs_strtoi32_1c, total_passes, total_fails);
  DOATEST (cs_strtoi32_1d, total_passes, total_fails);
  DOATEST (cs_strtoi32_2a, total_passes, total_fails);
  DOATEST (cs_strtoi32_2b, total_passes, total_fails);
  DOATEST (cs_strtoi32_2c, total_passes, total_fails);
  DOATEST (cs_strtoi32_2d, total_passes, total_fails);
  DOATEST (cs_strtoi32_2e, total_passes, total_fails);
  DOATEST (cs_strtoi32_2f, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoi32:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoi32:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoi32:1 TEST COMPLETE", (uint32_t)0U);

  return;
}


/*
** Sub-test Variations (cs_strtoi64)
*/
static Status_t
cs_strtoi64_1a (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:1.a PASSED";
  static const char failed[] = "cs_strtoi64:1:1.a FAILED";

  /* set pointer to NULL */
  p = (void *)0;

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == RET_ON_ERROR_INT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_INT64 & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_INT64 & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_1b (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:1.b PASSED";
  static const char failed[] = "cs_strtoi64:1:1.b FAILED";

  /* set pointer to point to an invalid number */
  p = "0xabcdef012345678g";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == RET_ON_ERROR_INT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_INT64 & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_INT64 & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_1c (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:1.c PASSED";
  static const char failed[] = "cs_strtoi64:1:1.c FAILED";

  /* set pointer to point to a number less than min int64_t value */
  p = "-9223372036854775809";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == RET_ON_ERROR_INT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_INT64 & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_INT64 & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_1d (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:1.d PASSED";
  static const char failed[] = "cs_strtoi64:1:1.d FAILED";

  /* set pointer to point to a number greater than max int64_t value */
  p = "9223372036854775808";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == RET_ON_ERROR_INT64)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((RET_ON_ERROR_INT64 & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(RET_ON_ERROR_INT64 & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_2a (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:2.a PASSED";
  static const char failed[] = "cs_strtoi64:1:2.a FAILED";

  /* set pointer to point to a number equal to max int64_t value */
  p = "9223372036854775807";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == INT64_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((INT64_MAX & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(INT64_MAX & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_2b (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:2.b PASSED";
  static const char failed[] = "cs_strtoi64:1:2.b FAILED";

  /* set pointer to point to a number equal to min int64_t value */
  p = "-9223372036854775808";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == INT64_MIN)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((INT64_MIN & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(INT64_MIN & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_2c (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:2.c PASSED";
  static const char failed[] = "cs_strtoi64:1:2.c FAILED";

  /* set pointer to point to a valid decimal number, with a + */
  p = "+1234567890123456789";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == 1234567890123456789ll)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((1234567890123456789ll & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(1234567890123456789ll & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_2d (void)
{
  char              *p;
  int64_t           i64Value;
  static const char passed[] = "cs_strtoi64:1:2.d PASSED";
  static const char failed[] = "cs_strtoi64:1:2.d FAILED";

  /* set pointer to point to a valid decimal number, with spaces */
  p = "   -9876543210543210";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == -9876543210543210ll)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((-9876543210543210ll & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(-9876543210543210ll & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_2e (void)
{
  int64_t           i64Value = 0;
  char     *p;
  static const char passed[] = "cs_strtoi64:1:2.e PASSED";
  static const char failed[] = "cs_strtoi64:1:2.e FAILED";

  /* set pointer to point to a hexadecimal number equal to max int64_t value */
  p = "0x7FFFFFFFFFFFFFFF";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == INT64_MAX)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((INT64_MAX & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(INT64_MAX & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

static Status_t
cs_strtoi64_2f (void)
{
  char              *p;
  int64_t           i64Value = 0;
  static const char passed[] = "cs_strtoi64:1:2.f PASSED";
  static const char failed[] = "cs_strtoi64:1:2.f FAILED";

  /* set pointer to point to a valid hexadecimal number with spaces */
  p = "   0x0deadBEEFfedBEEF";

  i64Value = cs_strtoi64 (p, RET_ON_ERROR_INT64);
  if (i64Value == 0xDEADBEEFFEDBEEFll)
  {
    IB_LOG_INFO (passed, (uint32_t)0U);
    return VSTATUS_OK;
  }
  else
  {
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Hi)",
                  (uint32_t)((0xDEADBEEFFEDBEEFll & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; expected (Lo)",
                  (uint32_t)(0xDEADBEEFFEDBEEFll & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Hi)",
                  (uint32_t)((i64Value & 0xFFFFFFFF00000000ll) >> 32));
    IB_LOG_ERROR ("cs_strtoi64 failed; actual (Lo)",
                  (uint32_t)(i64Value & 0x00000000FFFFFFFFll));
    IB_LOG_ERROR (failed, (uint32_t)0U);
    return VSTATUS_BAD;
  }
}

void
test_cs_strtoi64_1 (void)
{
  uint32_t total_passes = (uint32_t)0U;
  uint32_t total_fails = (uint32_t)0U;

  IB_LOG_INFO ("cs_strtoi64:1 TEST STARTED", (uint32_t)0U);

  DOATEST (cs_strtoi64_1a, total_passes, total_fails);
  DOATEST (cs_strtoi64_1b, total_passes, total_fails);
  DOATEST (cs_strtoi64_1c, total_passes, total_fails);
  DOATEST (cs_strtoi64_1d, total_passes, total_fails);
  DOATEST (cs_strtoi64_2a, total_passes, total_fails);
  DOATEST (cs_strtoi64_2b, total_passes, total_fails);
  DOATEST (cs_strtoi64_2c, total_passes, total_fails);
  DOATEST (cs_strtoi64_2d, total_passes, total_fails);
  DOATEST (cs_strtoi64_2e, total_passes, total_fails);
  DOATEST (cs_strtoi64_2f, total_passes, total_fails);

  IB_LOG_INFO ("cs_strtoi64:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("cs_strtoi64:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("cs_strtoi64:1 TEST COMPLETE", (uint32_t)0U);

  return;
}
