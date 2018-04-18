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
*      vs_pool_test.c
*
* DESCRIPTION
*      This file contains vs_pool common test routines.
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
* DKJ       02/07/02    Initial creation of file.
* PJG       04/01/02    Changes for CAL API 2.0.
* DKJ       04/01/02    OS API 2.0g updates
* PJG       04/02/02    PR 1676. vs_thread_sleep prototype per OS API 2.0g
* PJG       04/10/02    Get page size from environment instead of using
*                         DEFAULT_PAGESIZE.
* PJG       04/12/02    Don't run vs_pool_create_6b on PPC Linux Kernel space.
***********************************************************************/
#include <cs_g.h>
static uint64_t sleeptime;
#define WAIT_FOR_LOGGING_TO_CATCHUP  sleeptime = (uint64_t) 1000000U; \
  (void) vs_thread_sleep (sleeptime)
#define DOATEST(func, pass, fail) ((func)() == VSTATUS_OK) ? pass++ : fail++
#define PAGE_ALIGNED_ADDR(addr, psize) (((uint64_t)(addr) & (psize - (uint64_t) 0x01U)) == (uint64_t) 0x00U)
#define DEFAULT_PAGESIZE ((size_t) 4096U)
static uint64_t block_memory[4096U];
static void *saved_addr[8192U];

//extern void* memset(void *, int, size_t);

static Status_t
vs_pool_page_size_1a (void)
{
  static const char passed[] = "vs_pool_page_size:1:1.a PASSED";
  static const char failed[] = "vs_pool_page_size:1:1.a FAILED";
  Status_t rc;
  size_t pagesize;

  pagesize = vs_pool_page_size ();
  if (pagesize != (size_t) 0U)
    {
      IB_LOG_INFO (passed, pagesize);
      rc = VSTATUS_OK;
    }
  else
    {
      IB_LOG_ERROR ("vs_pool_page_size returned zero", (uint32_t) 0U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_pool_page_size_1b (void)
{
  static const char passed[] = "vs_pool_page_size:1:1.b PASSED";
  static const char failed[] = "vs_pool_page_size:1:1.b FAILED";
  Status_t rc;
  size_t pagesize;
  uint32_t count;
  uint32_t i;

  pagesize = vs_pool_page_size ();
  for (count = (uint32_t) 0U, i = (uint32_t) 0U; i < (sizeof (pagesize) << 3);
       i++)
    {
      if ((pagesize & (((size_t) 0x01U) << i)) != (size_t) 0U)
	{
	  count++;
	}
    }
  if (count == (uint32_t) 0x01U)
    {
      IB_LOG_INFO (passed, pagesize);
      rc = VSTATUS_OK;
    }
  else
    {
      IB_LOG_ERROR ("Invalid page size: low", (uint32_t) pagesize);
      IB_LOG_ERROR ("Invalid page size: high", (uint32_t) (pagesize));
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }
  return rc;

}

static Status_t
vs_pool_page_size_1c (void)
{
  static const char passed[] = "vs_pool_page_size:1:1.c PASSED";
  static const char failed[] = "vs_pool_page_size:1:1.c FAILED";
  Status_t rc;
  size_t pagesize;

  pagesize = vs_pool_page_size ();
  if (pagesize > (size_t) 256U)
    {
      IB_LOG_INFO (passed, pagesize);
      rc = VSTATUS_OK;
    }
  else
    {
      IB_LOG_ERROR ("Invalid page size: low", (uint32_t) pagesize);
      IB_LOG_ERROR ("Invalid page size: high", (uint32_t) (pagesize));
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}
void
test_pool_page_size_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_pool_page_size:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_pool_page_size_1a, total_passes, total_fails);
  DOATEST (vs_pool_page_size_1b, total_passes, total_fails);
  DOATEST (vs_pool_page_size_1c, total_passes, total_fails);
  IB_LOG_INFO ("vs_pool_page_size:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_pool_page_size:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_pool_page_size:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}

static Status_t
vs_pool_create_1a (void)
{
  static const char passed[] = "vs_pool_create:1:1.a PASSED";
  static const char failed[] = "vs_pool_create:1:1.a FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();

  rc =
    vs_pool_create (0, (uint32_t) 0U,
			(unsigned char *) "tst_pool", 0, size);
  if (rc == VSTATUS_ILLPARM)
    {
      IB_LOG_INFO (passed, (uint32_t) 0U);
      rc = VSTATUS_OK;
    }
  else
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      rc = VSTATUS_BAD;
    }

  return rc;

}

static Status_t
vs_pool_create_2a (void)
{
  static const char passed[] = "vs_pool_create:1:2.a PASSED";
  static const char failed[] = "vs_pool_create:1:2.a FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  uint32_t i;
  static Pool_t pool;
  void *address;
  const uint32_t options[] = { (uint32_t) 0U,
    (uint32_t) VMEM_PAGE,
  };

  for (i = (uint32_t) 0U; i < sizeof (options) / sizeof (options[0]); i++)
    {
      (void) memset (&pool, 0, sizeof (Pool_t));
      address = 0;
      rc =
	vs_pool_create (&pool, options[i],
			    (unsigned char *) "tst_pool", address, size);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_create options: ", options[i]);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
      rc = vs_pool_alloc (&pool, size, &address);
	  if (rc != VSTATUS_OK)
	    {
	      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
	      IB_LOG_ERROR ("vs_pool_create options: ", options[i]);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      (void) vs_pool_delete (&pool);
	      return VSTATUS_BAD;
	    }
	  if ((options[i] & VMEM_PAGE) == VMEM_PAGE)
	    {
	      if (!PAGE_ALIGNED_ADDR (address, size))
		{
		  IB_LOG_ERROR ("allocated address not page aligned",
				(uint64_t) address);
		  IB_LOG_ERROR ("vs_pool_create options: ", options[i]);
		  IB_LOG_ERROR (failed, (uint32_t) 0U);
		  (void) vs_pool_delete (&pool);
		  return VSTATUS_BAD;
		}
	    }
	  rc = vs_pool_free (&pool, address);
	  if (rc != VSTATUS_OK)
	    {
	      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	      IB_LOG_ERROR ("vs_pool_free address", (uint64_t) address);
	      IB_LOG_ERROR ("vs_pool_create options: ", options[i]);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      (void) vs_pool_delete (&pool);
	      return VSTATUS_BAD;
	    }
	}
      rc = vs_pool_delete (&pool);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_pool_create_2b (void)
{
  static const char passed[] = "vs_pool_create:1:2.b PASSED";
  static const char failed[] = "vs_pool_create:1:2.b FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  uint32_t i;
  static Pool_t pool;
  void *address;
  uint32_t options;
  const uint32_t valid_options =
    (uint32_t) (VMEM_PAGE);

  for (i = (uint32_t) 0U; i < (uint32_t) 32U; i++)
    {
      (void) memset (&pool, 0, sizeof (Pool_t));
      address = 0;
      options = (uint32_t) 0x01U << i;
      if ((options & valid_options) == (uint32_t) 0U)
	{
	  rc =
	    vs_pool_create (&pool, options,
				(unsigned char *) "tst_pool", address, size);
	  if (rc != VSTATUS_ILLPARM)
	    {
	      IB_LOG_ERROR ("vs_pool_create failed; expected",
			    VSTATUS_ILLPARM);
	      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
	      IB_LOG_ERROR ("vs_pool_create options: ", options);
	      IB_LOG_ERROR (failed, (uint32_t) 0U);
	      return VSTATUS_BAD;
	    }
	}
    }

  (void) memset (&pool, 0, sizeof (Pool_t));
  address = 0;
  options = ~valid_options;
  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "tst_pool", address, size);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  (void) memset (&pool, 0, sizeof (Pool_t));
  address = 0;
  options = ~((uint32_t) 0x0U);
  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "tst_pool", address, size);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_pool_create_2c (void)
{
  static const char passed[] = "vs_pool_create:1:2.c PASSED";
  static const char failed[] = "vs_pool_create:1:2.c FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  static Pool_t pool;
  void *address;
  uint32_t options;
  size_t i;

  for (i = (size_t) 0x01U; i < size; i++)
    {
      (void) memset (&pool, 0, sizeof (Pool_t));
      address = 0;
      options = (uint32_t) VMEM_PAGE;
      rc =
	vs_pool_create (&pool, options,
			    (unsigned char *) "tst_pool", address, i);
      if (rc != VSTATUS_NOMEM)
	{
	  IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_NOMEM);
	  IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_create options: ", options);
	  IB_LOG_ERROR ("pool size", i);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_pool_create_2d (void)
{
  static const char passed[] = "vs_pool_create:1:2.d PASSED";
  static const char failed[] = "vs_pool_create:1:2.d FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  static Pool_t pool;
  uint32_t options;
  void *address;
  size_t i;

  for (i = (size_t) 0x01U; i < size; i++)
    {
      (void) memset (&pool, 0, sizeof (Pool_t));
      options = (uint32_t) VMEM_PAGE;
      address = &block_memory[0];
      rc =
	vs_pool_create (&pool, options,
			    (unsigned char *) "tst_pool", address, i);
      if (rc != VSTATUS_NOMEM)
	{
	  IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_NOMEM);
	  IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_create options: ", options);
	  IB_LOG_ERROR ("pool size", i);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;

}

static Status_t
vs_pool_create_2e (void)
{
  static const char passed[] = "vs_pool_create:1:2.e PASSED";
  static const char failed[] = "vs_pool_create:1:2.e FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  static Pool_t pool;
  uint32_t options;
  void *address;
  uint32_t i;

  if (PAGE_ALIGNED_ADDR (&block_memory[0], size))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  for (i = (size_t) 0x01U; i < (size_t) (size * (size_t) 0x02U); i++)
    {
      (void) memset (&pool, 0, sizeof (Pool_t));
      options = (uint32_t) VMEM_PAGE;
      rc =
	vs_pool_create (&pool, options,
			    (unsigned char *) "tst_pool", address, i);
      if (rc != VSTATUS_NOMEM)
	{
	  IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_NOMEM);
	  IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_create options: ", options);
	  IB_LOG_ERROR ("pool size", i);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_2f (void)
{
  static const char passed[] = "vs_pool_create:1:2.f PASSED";
  static const char failed[] = "vs_pool_create:1:2.f FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  static Pool_t pool;
  uint32_t options;
  void *address;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) VMEM_PAGE;
  address = 0;
  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "tst_pool", address, size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR ("pool size", (uint32_t) size);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, size, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR ("pool size", (uint32_t) size);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  if (!PAGE_ALIGNED_ADDR (address, size))
    {
      IB_LOG_ERROR ("allocated address not page aligned", (uint64_t) address);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_free (&pool, address);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_free address", (uint64_t) address);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_4a (void)
{
  static const char passed[] = "vs_pool_create:1:4.a PASSED";
  static const char failed[] = "vs_pool_create:1:4.a FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  static Pool_t pool;
  uint32_t options;
  void *address;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) VMEM_PAGE;
  address = 0;
  rc = vs_pool_create (&pool, options, 0, address, size);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR ("pool size", (uint32_t) size);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_4b (void)
{
  static const char passed[] = "vs_pool_create:1:4.b PASSED";
  static const char failed[] = "vs_pool_create:1:4.b FAILED";
  Status_t rc;
  size_t size = vs_pool_page_size();
  static Pool_t pool;
  uint32_t options;
  void *address;
  unsigned char name[2 * VS_NAME_MAX];

  (void) memset (&name[0], 18, sizeof (name));
  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) VMEM_PAGE;
  address = 0;
  rc = vs_pool_create (&pool, options, name, address, size);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR ("pool size", (uint32_t) size);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_5a (void)
{
  static const char passed[] = "vs_pool_create:1:5.a PASSED";
  static const char failed[] = "vs_pool_create:1:5.a FAILED";
  Status_t rc;
  static Pool_t pool;
  uint32_t options;
  void *address;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = 0;
  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			(size_t) 0x01U);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR ("pool size", (uint32_t) 0x01U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 0x01U, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR ("pool size", (uint32_t) 0x01U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_free address", (uint64_t) address);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_5b (void)
{
  static const char passed[] = "vs_pool_create:1:5.b PASSED";
  static const char failed[] = "vs_pool_create:1:5.b FAILED";
  Status_t rc;
  static Pool_t pool;
  uint32_t options;
  void *address;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = &block_memory[0];
  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			(size_t) 0x01U);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR ("pool size", (uint32_t) 0x01U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 0x01U, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR ("pool size", (uint32_t) 0x01U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  if (address != (void *) &block_memory[0])
    {
      IB_LOG_ERROR ("allocated address not within pool addresses",
		    (uint64_t) address);
      IB_LOG_ERROR ("pool memory address", (uint64_t) & block_memory[0]);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_free (&pool, address);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_free address", (uint64_t) address);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_6a (void)
{
  static const char passed[] = "vs_pool_create:1:6.a PASSED";
  static const char failed[] = "vs_pool_create:1:6.a FAILED";
  Status_t rc;
  static Pool_t pool;
  uint32_t options;
  void *address;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = 0;

  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			(size_t) 0x00U);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_6b (void)
{
  static const char passed[] = "vs_pool_create:1:6.b PASSED";
  static const char failed[] = "vs_pool_create:1:6.b FAILED";
  Status_t rc;
  static Pool_t pool;
  uint32_t options;
  void *address;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = 0;

  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			(size_t) 0xFFFFFFF0U);
  if (rc != VSTATUS_NOMEM)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_NOMEM);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_7a (void)
{
  static const char passed[] = "vs_pool_create:1:7.a PASSED";
  static const char failed[] = "vs_pool_create:1:7.a FAILED";
  Status_t rc;
  static Pool_t pool;
  uint32_t options;
  void *address;
  void *allocated[64U];
  uint32_t i;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = &block_memory[0];

  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			sizeof (block_memory));
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  (void) memset (&allocated[0], 0, sizeof (allocated));
  for (i = (uint32_t) 0U;
       i < (sizeof (allocated) / sizeof (allocated[0])); i++)
    {
      rc = vs_pool_alloc (&pool, (size_t) 128U, &allocated[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
	  IB_LOG_ERROR ("pool size", (uint32_t) 128U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  (void) vs_pool_delete (&pool);
	  return VSTATUS_BAD;
	}
    }

  for (i = (uint32_t) 0U;
       i < (sizeof (allocated) / sizeof (allocated[0])); i++)
    {
      rc = vs_pool_free (&pool, allocated[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address", (uint64_t) allocated[i]);
	  IB_LOG_ERROR ("vs_pool_create options: ", options);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  (void) vs_pool_delete (&pool);
	  return VSTATUS_BAD;
	}
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_create_7b (void)
{
  static const char passed[] = "vs_pool_create:1:7.b PASSED";
  static const char failed[] = "vs_pool_create:1:7.b FAILED";
  Status_t rc;
  static Pool_t pool;
  uint32_t options;
  void *address;
  void *allocated[64U];
  uint32_t i;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = 0;

  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			sizeof (block_memory));
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  (void) memset (&allocated[0], 0, sizeof (allocated));
  for (i = (uint32_t) 0U;
       i < (sizeof (allocated) / sizeof (allocated[0])); i++)
    {
      rc = vs_pool_alloc (&pool, (size_t) 128U, &allocated[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
	  IB_LOG_ERROR ("pool size", (uint32_t) 128U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  (void) vs_pool_delete (&pool);
	  return VSTATUS_BAD;
	}
    }

  for (i = (uint32_t) 0U;
       i < (sizeof (allocated) / sizeof (allocated[0])); i++)
    {
      rc = vs_pool_free (&pool, allocated[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address", (uint64_t) allocated[i]);
	  IB_LOG_ERROR ("vs_pool_create options: ", options);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  (void) vs_pool_delete (&pool);
	  return VSTATUS_BAD;
	}
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}
void
test_pool_create_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;
  IB_LOG_INFO ("vs_pool_create:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_pool_create_1a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_2a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_2b, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_2c, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_2d, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_2e, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_2f, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_4a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_4b, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_5a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_5b, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_6a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_6b, total_passes, total_fails);
  IB_LOG_INFO ("vs_pool_create:1:6.c SKIPPED", (uint32_t) 0U);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_7a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_create_7b, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_pool_create:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_pool_create:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_pool_create:1 TEST COMPLETE", (uint32_t) 0U);
  return;
}

static Status_t
vs_pool_alloc_1a (void)
{
  static const char passed[] = "vs_pool_alloc:1:1.a PASSED";
  static const char failed[] = "vs_pool_alloc:1:1.a FAILED";
  Status_t rc;
  void *address;

  rc = vs_pool_alloc (0, (size_t) 128U, &address);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_2a (void)
{
  static const char passed[] = "vs_pool_alloc:1:2.a PASSED";
  static const char failed[] = "vs_pool_alloc:1:2.a FAILED";
  Status_t rc;
  void *address;
  uint32_t options;
  Pool_t pool;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = &block_memory[0];

  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			sizeof (block_memory));
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 128U, 0);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_3a (void)
{
  static const char passed[] = "vs_pool_alloc:1:3.a PASSED";
  static const char failed[] = "vs_pool_alloc:1:3.a FAILED";
  Status_t rc;
  void *address;
  uint32_t options;
  Pool_t pool;

  (void) memset (&pool, 0, sizeof (Pool_t));
  options = (uint32_t) 0x00U;
  address = &block_memory[0];

  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", address,
			sizeof (block_memory));
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_create options: ", options);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 0U, &address);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4a (Pool_t * pool)
{
  static const char passed[] = "vs_pool_alloc:1:4.a PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.a FAILED";
  Status_t rc;
  void *address = 0;

  rc = vs_pool_alloc (pool, (size_t) 0x01U, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (address == 0)
    {
      IB_LOG_ERROR ("Invalid allocated address", (uint32_t) 0U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (!PAGE_ALIGNED_ADDR ( address, sizeof (uint64_t)))
    {
      IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
		    (uint64_t) address);
      (void) vs_pool_free (pool, address);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (((pool->options & VMEM_PAGE) == VMEM_PAGE) &&
      (!PAGE_ALIGNED_ADDR ( address, vs_pool_page_size())))
    {
      IB_LOG_ERROR ("Allocated buffer not page aligned", (uint64_t) address);
      (void) vs_pool_free (pool, address);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4b (Pool_t * pool, uint32_t options)
{
  static const char passed[] = "vs_pool_alloc:1:4.b PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.b FAILED";
  Status_t rc;
  uint32_t i;
  uint32_t j;
  uint32_t error_flag = (uint32_t) 0x0U;

  for (i = (uint32_t) 0x0U, rc = VSTATUS_OK; rc == VSTATUS_OK;)
    {
      saved_addr[i] = 0;
      rc = vs_pool_alloc (pool, (size_t) 0x01U, &saved_addr[i]);
      if (rc == VSTATUS_OK)
	{
	  i++;
	}
      if (i >= (sizeof (saved_addr) / sizeof (saved_addr[0])))
	{
	  break;
	}
    }

  if ((options & VMEM_PAGE) == VMEM_PAGE)
    {
      if (i < (uint32_t) 0x01U)
	{
	  error_flag++;
	}
    }
  else
    {
      if (i < (uint32_t) 0x02U)
	{
	  error_flag++;
	}
    }

  for (j = (uint32_t) 0U; j < i; j++)
    {
      if (!PAGE_ALIGNED_ADDR ( saved_addr[j], sizeof (uint64_t)))
	{
	  IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
			(uint64_t) saved_addr[j]);
	  error_flag++;
	}

      rc = vs_pool_free (pool, saved_addr[j]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) saved_addr[j]);
	  error_flag++;
	}
    }

  if (error_flag == (uint32_t) 0x00U)
    {
      IB_LOG_INFO (passed, (uint32_t) 0U);
      return VSTATUS_OK;
    }
  else
    {
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }
}

static Status_t
vs_pool_alloc_4c (Pool_t * pool)
{
  static const char passed[] = "vs_pool_alloc:1:4.c PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.c FAILED";
  Status_t rc;
  size_t pagesize = vs_pool_page_size();
  void *address = 0;

  rc = vs_pool_alloc (pool, (pagesize - (size_t) 0x01U), &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (address == 0)
    {
      IB_LOG_ERROR ("Invalid allocated address", (uint32_t) 0x00U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (!PAGE_ALIGNED_ADDR ( address, sizeof (uint64_t)))
    {
      IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
		    (uint64_t) address);
      (void) vs_pool_free (pool, address);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (((pool->options & VMEM_PAGE) == VMEM_PAGE) &&
      (!PAGE_ALIGNED_ADDR ( address, pagesize)))
    {
      IB_LOG_ERROR ("Allocated buffer not page aligned", (uint64_t) address);
      (void) vs_pool_free (pool, address);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4d (Pool_t * pool, Status_t compare)
{
  static const char passed[] = "vs_pool_alloc:1:4.d PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.d FAILED";
  Status_t rc;
  size_t pagesize = vs_pool_page_size();
  void *address = 0;

  rc = vs_pool_alloc (pool, pagesize, &address);
  if (rc != compare)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", compare);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      if (rc == VSTATUS_OK)
	{
	  (void) vs_pool_free (pool, address);
	}
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (rc == VSTATUS_OK)
    {
      if (address == 0)
	{
	  IB_LOG_ERROR ("Invalid allocated address", (uint32_t) 0x00U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (!PAGE_ALIGNED_ADDR ( address, sizeof (uint64_t)))
	{
	  IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (((pool->options & VMEM_PAGE) == VMEM_PAGE) &&
	  (!PAGE_ALIGNED_ADDR ( address, pagesize)))
	{
	  IB_LOG_ERROR ("Allocated buffer not page aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      rc = vs_pool_free (pool, address);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4e (Pool_t * pool, Status_t compare)
{
  static const char passed[] = "vs_pool_alloc:1:4.e PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.e FAILED";
  Status_t rc;
  size_t pagesize = vs_pool_page_size();
  void *address = 0;

  rc = vs_pool_alloc (pool, (pagesize + (size_t) 0x01U), &address);
  if (rc != compare)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", compare);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      if (rc == VSTATUS_OK)
	{
	  (void) vs_pool_free (pool, address);
	}
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (rc == VSTATUS_OK)
    {
      if (address == 0)
	{
	  IB_LOG_ERROR ("Invalid allocated address", (uint32_t) 0x00U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (!PAGE_ALIGNED_ADDR ( address, sizeof (uint64_t)))
	{
	  IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (((pool->options & VMEM_PAGE) == VMEM_PAGE) &&
	  (!PAGE_ALIGNED_ADDR ( address, pagesize)))
	{
	  IB_LOG_ERROR ("Allocated buffer not page aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      rc = vs_pool_free (pool, address);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4f (Pool_t * pool, size_t size, Status_t compare)
{
  static const char passed[] = "vs_pool_alloc:1:4.f PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.f FAILED";
  Status_t rc;
  size_t pagesize = vs_pool_page_size();
  void *address = 0;

  rc = vs_pool_alloc (pool, size, &address);
  if (rc != compare)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", compare);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      if (rc == VSTATUS_OK)
	{
	  (void) vs_pool_free (pool, address);
	}
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (rc == VSTATUS_OK)
    {
      if (address == 0)
	{
	  IB_LOG_ERROR ("Invalid allocated address", (uint32_t) 0x00U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (!PAGE_ALIGNED_ADDR ( address, sizeof (uint64_t)))
	{
	  IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (((pool->options & VMEM_PAGE) == VMEM_PAGE) &&
	  (!PAGE_ALIGNED_ADDR ( address, pagesize)))
	{
	  IB_LOG_ERROR ("Allocated buffer not page aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      rc = vs_pool_free (pool, address);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

    }
  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4g (Pool_t * pool, size_t size, Status_t compare)
{
  static const char passed[] = "vs_pool_alloc:1:4.g PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.g FAILED";
  Status_t rc;
  size_t pagesize = vs_pool_page_size();
  void *address = 0;

  rc = vs_pool_alloc (pool, size, &address);
  if (rc != compare)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", compare);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      if (rc == VSTATUS_OK)
	{
	  (void) vs_pool_free (pool, address);
	}
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (rc == VSTATUS_OK)
    {
      if (address == 0)
	{
	  IB_LOG_ERROR ("Invalid allocated address", (uint32_t) 0x00U);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (!PAGE_ALIGNED_ADDR ( address, sizeof (uint64_t)))
	{
	  IB_LOG_ERROR ("Allocated buffer not 64-bit aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      if (((pool->options & VMEM_PAGE) == VMEM_PAGE) &&
	  (!PAGE_ALIGNED_ADDR ( address, pagesize)))
	{
	  IB_LOG_ERROR ("Allocated buffer not page aligned",
			(uint64_t) address);
	  (void) vs_pool_free (pool, address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

      rc = vs_pool_free (pool, address);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_alloc_4h (Pool_t * pool, size_t size)
{
  static const char passed[] = "vs_pool_alloc:1:4.h PASSED";
  static const char failed[] = "vs_pool_alloc:1:4.h FAILED";
  Status_t rc;
  void *address = 0;

  rc = vs_pool_alloc (pool, size, &address);
  if (rc != VSTATUS_NOMEM)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_NOMEM);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}
static void
pool_alloc4_driver (void *start, size_t size,
		    uint32_t options, size_t pagesize, uint32_t * passes,
		    uint32_t * fails)
{
  Pool_t pool;
  Status_t rc;
  (void) memset (&pool, 0, sizeof (Pool_t));
  IB_LOG_INFO ("Starting vs_pool_alloc_4* tests", (uint32_t) 0U);
  IB_LOG_INFO ("vs_pool_create options: ", options);
  IB_LOG_INFO ("vs_pool_create size: ", size);
  IB_LOG_INFO ("vs_pool_create address: ", (uint64_t) start);
  IB_LOG_INFO ("vs_pool_create pagesize: ", (uint32_t) pagesize);
  rc =
    vs_pool_create (&pool, options,
			(unsigned char *) "test_pool", start, (size_t) size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      (*fails)++;
      return;
    }

  (vs_pool_alloc_4a (&pool) == VSTATUS_OK) ? (*passes)++ : (*fails)++;
  (vs_pool_alloc_4b (&pool, options) ==
   VSTATUS_OK) ? (*passes)++ : (*fails)++;
  (vs_pool_alloc_4c (&pool) == VSTATUS_OK) ? (*passes)++ : (*fails)++;
  if (size < pagesize)
    {
      (vs_pool_alloc_4d (&pool, VSTATUS_NOMEM) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  else
    {
      (vs_pool_alloc_4d (&pool, VSTATUS_OK) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  if (size < (pagesize + (size_t) 0x01U))
    {
      (vs_pool_alloc_4e (&pool, VSTATUS_NOMEM) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  else
    {
      if (((options & VMEM_PAGE) == VMEM_PAGE) &&
	  (!PAGE_ALIGNED_ADDR ( start, pagesize)))
	{
	  if ((size - pagesize) < (pagesize + (size_t) 0x01U))
	    {
	      (vs_pool_alloc_4e (&pool, VSTATUS_NOMEM) ==
	       VSTATUS_OK) ? (*passes)++ : (*fails)++;
	    }
	  else
	    {
	      (vs_pool_alloc_4e (&pool, VSTATUS_OK) ==
	       VSTATUS_OK) ? (*passes)++ : (*fails)++;
	    }
	}
      else
	{
	  (vs_pool_alloc_4e (&pool, VSTATUS_OK) ==
	   VSTATUS_OK) ? (*passes)++ : (*fails)++;
	}
    }
  if (((options & VMEM_PAGE) == VMEM_PAGE) &&
      (!PAGE_ALIGNED_ADDR ( start, pagesize)))
    {
      (vs_pool_alloc_4f (&pool, (size - (size_t) 0x01U), VSTATUS_NOMEM) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  else
    {
      (vs_pool_alloc_4f (&pool, (size - (size_t) 0x01U), VSTATUS_OK) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  if (((options & VMEM_PAGE) == VMEM_PAGE) &&
      (!PAGE_ALIGNED_ADDR ( start, pagesize)))
    {
      (vs_pool_alloc_4g (&pool, size, VSTATUS_NOMEM) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  else
    {
      (vs_pool_alloc_4g (&pool, size, VSTATUS_OK) ==
       VSTATUS_OK) ? (*passes)++ : (*fails)++;
    }
  (vs_pool_alloc_4h (&pool, size + (size_t) 0x01U) ==
   VSTATUS_OK) ? (*passes)++ : (*fails)++;

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      (*fails)++;
    }

  return;
}

static Status_t
vs_pool_alloc_5a (void)
{
  static const char passed[] = "vs_pool_alloc:1:5.a PASSED";
  static const char failed[] = "vs_pool_alloc:1:5.a FAILED";
  Status_t rc;
  Pool_t pool;
  void *address[3];
  uint32_t i;
  size_t pagesize = vs_pool_page_size();

  (void) memset (&pool, 0, sizeof (Pool_t));
  rc =
    vs_pool_create (&pool, VMEM_PAGE,
			(unsigned char *) "test_pool", &block_memory[0],
			sizeof (block_memory));
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0x00U);
      return VSTATUS_BAD;
    }

  (void) memset (address, 0, sizeof (address));
  for (i = (uint32_t) 0U; i < (sizeof (address) / sizeof (address[0])); i++)
    {
      rc = vs_pool_alloc (&pool, (size_t) 400U, &address[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  (void) vs_pool_delete (&pool);
	  return VSTATUS_BAD;
	}
      if (!PAGE_ALIGNED_ADDR ( address[i], pagesize))
	{
	  IB_LOG_ERROR ("Allocated buffer not page aligned",
			(uint64_t) address[i]);
	  (void) vs_pool_free (&pool, address[i]);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

    }

  for (i = (uint32_t) 0U; i < (sizeof (address) / sizeof (address[0])); i++)
    {
      rc = vs_pool_free (&pool, address[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address[i]);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  (void) memset (address, 0, sizeof (address));
  for (i = (uint32_t) 0U; i < (sizeof (address) / sizeof (address[0])); i++)
    {
      rc = vs_pool_alloc (&pool, (size_t) 3000U, &address[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  (void) vs_pool_delete (&pool);
	  return VSTATUS_BAD;
	}
      if (!PAGE_ALIGNED_ADDR ( address[i], pagesize))
	{
	  IB_LOG_ERROR ("Allocated buffer not page aligned",
			(uint64_t) address[i]);
	  (void) vs_pool_free (&pool, address[i]);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}

    }

  for (i = (uint32_t) 0U; i < (sizeof (address) / sizeof (address[0])); i++)
    {
      rc = vs_pool_free (&pool, address[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR ("vs_pool_free address: ", (uint64_t) address[i]);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0x00U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0x00U);
  return VSTATUS_OK;
}
void
test_pool_alloc_1 (void)
{
  size_t pagesize = vs_pool_page_size();
  size_t temp;
  void *address;
  uint32_t options;
  size_t size;
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;
  IB_LOG_INFO ("vs_pool_alloc:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_pool_alloc_1a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_alloc_2a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_alloc_3a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;

  /* SA = page aligned; size = pagesize -1; options = none */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~(pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = pagesize - (size_t) 0x01U;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = page aligned; size = pagesize -1; options = VMEM_PAGE */
  /* SA = not page aligned; size = pagesize -1; options = none */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = pagesize - (size_t) 0x01U;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = pagesize -1; options = VMEM_PAGE */
  /* SA = page aligned; size = pagesize; options = none */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~(pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = pagesize;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = page aligned; size = pagesize; options = VMEM_PAGE */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~(pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = pagesize;
  options = (uint32_t) VMEM_PAGE;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = pagesize; options = none */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = pagesize;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = pagesize; options = VMEM_PAGE */
  /* SA = page aligned; size = 2*pagesize -1; options = none */
  temp = ((uint64_t) (&block_memory[0]) +
	  (uint32_t) pagesize - (uint32_t) 0x01U) &
    ~((uint32_t) pagesize - (uint32_t) 0x01U);
  address = (void *) temp;
  size = (pagesize * (size_t) 0x02U) - (size_t) 0x01U;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = page aligned; size = 2*pagesize -1; options = VMEM_PAGE */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~((size_t) pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = (pagesize * (size_t) 0x02U) - (size_t) 0x01U;
  options = (uint32_t) VMEM_PAGE;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = 2*pagesize -1; options = none */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = (pagesize * (size_t) 0x02U) - (size_t) 0x01U;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = 2*pagesize -1; options = VMEM_PAGE */

  /* SA = page aligned; size = 2*pagesize; options = none */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~((size_t) pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = (pagesize * (size_t) 0x02U);
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = page aligned; size = 2*pagesize; options = VMEM_PAGE */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~((size_t) pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = (pagesize * (size_t) 0x02U);
  options = (uint32_t) VMEM_PAGE;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = 2*pagesize; options = none */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = (pagesize * (size_t) 0x02U);
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = 2*pagesize; options = VMEM_PAGE */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = (pagesize * (size_t) 0x02U);
  options = (uint32_t) VMEM_PAGE;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = page aligned; size = 2*pagesize +1; options = none */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~((size_t) pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = (pagesize * (size_t) 0x02U) + (size_t) 0x01U;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = page aligned; size = 2*pagesize +1; options = VMEM_PAGE */
  temp = ((size_t) (&block_memory[0]) +
	  pagesize - (size_t) 0x01U) & ~(pagesize - (size_t) 0x01U);
  address = (void *) temp;
  size = (pagesize * (size_t) 0x02U) + (size_t) 0x01U;
  options = (uint32_t) VMEM_PAGE;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = 2*pagesize +1; options = none */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = (pagesize * (size_t) 0x02U) + (size_t) 0x01U;
  options = (uint32_t) 0x00U;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);

  WAIT_FOR_LOGGING_TO_CATCHUP;
  /* SA = not page aligned; size = 2*pagesize +1; options = VMEM_PAGE */
  if (PAGE_ALIGNED_ADDR (&block_memory[0], pagesize))
    {
      address = &block_memory[1];
    }
  else
    {
      address = &block_memory[0];
    }
  size = (pagesize * (size_t) 0x02U) + (size_t) 0x01U;
  options = (uint32_t) VMEM_PAGE;
  pool_alloc4_driver (address, size, options, pagesize, &total_passes,
		      &total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_alloc_5a, total_passes, total_fails);
  IB_LOG_INFO ("vs_pool_alloc:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_pool_alloc:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_pool_alloc:1 TEST COMPLETE", (uint32_t) 0U);
  return;
}

static Status_t
vs_pool_free_1a (void)
{
  static const char passed[] = "vs_pool_free:1:1.a PASSED";
  static const char failed[] = "vs_pool_free:1:1.a FAILED";
  Status_t rc;
  uint32_t a;

  rc = vs_pool_free (0, &a);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_free_2a (void)
{
  static const char passed[] = "vs_pool_free:1:2.a PASSED";
  static const char failed[] = "vs_pool_free:1:2.a FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, 0);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_free_3a (void)
{
  static const char passed[] = "vs_pool_free:1:3.a PASSED";
  static const char failed[] = "vs_pool_free:1:3.a FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = DEFAULT_PAGESIZE;
  void *address[8];
  uint32_t i;

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  (void) memset (&address[0], 0, sizeof (address));
  for (i = (uint32_t) 0U; i < (sizeof (address) / sizeof (address[0])); i++)
    {
      rc = vs_pool_alloc (&pool, (size_t) 64U, &address[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }
  for (i = (uint32_t) 0U; i < (sizeof (address) / sizeof (address[0])); i++)
    {
      rc = vs_pool_free (&pool, address[i]);
      if (rc != VSTATUS_OK)
	{
	  IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
	  IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
	  IB_LOG_ERROR (failed, (uint32_t) 0U);
	  return VSTATUS_BAD;
	}
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_free_4a (void)
{
  static const char passed[] = "vs_pool_free:1:4.a PASSED";
  static const char failed[] = "vs_pool_free:1:4.a FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, &pool);
  if (rc != VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_NXIO);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_free_4b (void)
{
  static const char passed[] = "vs_pool_free:1:4.b PASSED";
  static const char failed[] = "vs_pool_free:1:4.b FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();
  void *address = 0;

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 64U, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_NXIO);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_free_4c (void)
{
  static const char passed[] = "vs_pool_free:1:4.c PASSED";
  static const char failed[] = "vs_pool_free:1:4.c FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();
  void *address;

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  address = &block_memory[0];

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_NXIO);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}
static Status_t
vs_pool_free_4d (void)
{
  static const char passed[] = "vs_pool_free:1:4.d PASSED";
  static const char failed[] = "vs_pool_free:1:4.d FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();
  void *address = 0;
  uint64_t * badaddress;

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 64U, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  if (address == 0)
    {
      IB_LOG_ERROR ("vs_pool_alloc returned null address", (uint32_t) 0U);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  (void) memset (address, 19, (size_t) 64U);
  badaddress = (uint64_t *) address;
  badaddress++;
  badaddress++;

  rc = vs_pool_free (&pool, badaddress);
  if (rc != VSTATUS_NXIO)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_NXIO);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_free (&pool, address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_free failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_free failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}
void
test_pool_free_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_pool_free:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_pool_free_1a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_free_2a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_free_3a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_free_4a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_free_4b, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_free_4c, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_free_4d, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_pool_free:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_pool_free:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_pool_free:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}

static Status_t
vs_pool_delete_1a (void)
{
  static const char passed[] = "vs_pool_delete:1:1.a PASSED";
  static const char failed[] = "vs_pool_delete:1:1.a FAILED";
  Status_t rc;

  rc = vs_pool_delete (0);
  if (rc != VSTATUS_ILLPARM)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_ILLPARM);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_delete_2a (void)
{
  static const char passed[] = "vs_pool_delete:1:2.a PASSED";
  static const char failed[] = "vs_pool_delete:1:2.a FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}

static Status_t
vs_pool_delete_3a (void)
{
  static const char passed[] = "vs_pool_delete:1:3.a PASSED";
  static const char failed[] = "vs_pool_delete:1:3.a FAILED";
  Status_t rc;
  Pool_t pool;
  size_t size = vs_pool_page_size();
  void *address = 0;

  rc =
    vs_pool_create (&pool, (uint32_t) 0U,
			(unsigned char *) "tst_pool", &block_memory[0], size);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_create failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_create failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  rc = vs_pool_alloc (&pool, (size_t) 0x64U, &address);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_alloc failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_alloc failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      (void) vs_pool_delete (&pool);
      return VSTATUS_BAD;
    }

  rc = vs_pool_delete (&pool);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERROR ("vs_pool_delete failed; expected", VSTATUS_OK);
      IB_LOG_ERROR ("vs_pool_delete failed; actual", rc);
      IB_LOG_ERROR (failed, (uint32_t) 0U);
      return VSTATUS_BAD;
    }

  IB_LOG_INFO (passed, (uint32_t) 0U);
  return VSTATUS_OK;
}
void
test_pool_delete_1 (void)
{
  uint32_t total_passes = (uint32_t) 0U;
  uint32_t total_fails = (uint32_t) 0U;

  IB_LOG_INFO ("vs_pool_delete:1 TEST STARTED", (uint32_t) 0U);
  DOATEST (vs_pool_delete_1a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_delete_2a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  DOATEST (vs_pool_delete_3a, total_passes, total_fails);
  WAIT_FOR_LOGGING_TO_CATCHUP;
  IB_LOG_INFO ("vs_pool_delete:1 TOTAL PASSED", total_passes);
  IB_LOG_INFO ("vs_pool_delete:1 TOTAL FAILED", total_fails);
  IB_LOG_INFO ("vs_pool_delete:1 TEST COMPLETE", (uint32_t) 0U);

  return;
}

#if 0
static Status_t
xlate_verify (size_t numentries, uint64_t entries[], size_t length,
	      size_t offset, size_t pagesize, void *baseaddr)
{
  size_t i;
  size_t temp = (size_t) 0U;
  size_t computed_length = (size_t) 0U;

  if (offset >= pagesize)
    {
      IB_LOG_ERROR (" offset value > pagesize: offset", (uint32_t) (offset));
      IB_LOG_ERROR (" offset value > pagesize: low", pagesize);
      return VSTATUS_BAD;
    }

  temp = (size_t) baseaddr & (pagesize - (size_t) 1U);
  if (offset != temp)
    {
      IB_LOG_ERROR ("Invalid offset value: high", (uint32_t) (offset));
      IB_LOG_ERROR ("Invalid offset value: low", (uint32_t) (offset));
      return VSTATUS_BAD;
    }

  for (i = (size_t) 0U; i < (size_t) numentries; i++)
    {
      computed_length += pagesize;
#if 0
      IB_LOG_INFO ("Entry index", i);
      IB_LOG_INFO ("Addr high", (uint32_t) (entries[i] >> 32));
      IB_LOG_INFO ("Addr low", (uint32_t) (entries[i]));
#endif
      if (!PAGE_ALIGNED_ADDR (entries[i], pagesize))
	{
	  IB_LOG_ERROR ("address not page aligned Addr high:",
			(uint32_t) (entries[i] >> 32));
	  IB_LOG_ERROR
	    ("address not page aligned Addr low:", (uint32_t) (entries[i]));
	  return VSTATUS_BAD;
	}
    }
  computed_length -= offset;
  if (computed_length >= length)
    {
      return VSTATUS_OK;
    }
  else
    {
      IB_LOG_ERROR ("xlate_verify computed length error - computed",
		    computed_length);
      IB_LOG_ERROR ("xlate_verify computed length error - expected", length);
      return VSTATUS_BAD;
    }
}
#endif
