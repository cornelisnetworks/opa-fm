/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/***********************************************************************
* 
* FILE NAME
*      vs_pool_common.c
*
* DESCRIPTION
*      This file contains vs_pool common routines.
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
* DKJ       03/06/02    Initial creation of file.
* DKJ       03/26/02    LINT cleanup
* DKJ       04/01/02    PR1676. OS API 2.0g updates
***********************************************************************/
#include <cs_g.h>
#include "cs_log.h"
#define function __FUNCTION__
#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID
extern Status_t vs_implpool_create (Pool_t *, uint32_t,
				    unsigned char *, void *, size_t);
extern Status_t vs_implpool_delete (Pool_t *);
extern Status_t vs_implpool_alloc (Pool_t *, size_t, void **);
extern Status_t vs_implpool_free (Pool_t *, void *);
extern Status_t vs_implpool_size (Pool_t *, uint64_t *);

#define VSPOOL_MAGIC ((uint32_t) 0xDCADFEEDU)
/**********************************************************************
*
* FUNCTION
*    vs_pool_create
*
* DESCRIPTION
*    Does fundamental common validation for vs_pool_create
*    parameters and invokes environment specific implmentation.
*
* INPUTS
*
* OUTPUTS
*      Status_t - On success VSTATUS_OK is returned, otherwise
*      the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   DKJ     03/07/02    Initial creation of function.
*   DKJ     03/21/02    Updated prototype
**********************************************************************/
Status_t
vs_pool_create (Pool_t * handle, uint32_t options, 
		    unsigned char * name, void *address, size_t size)
{
  Status_t rc;
  size_t namesize;
  const uint32_t valid_options = 0;

  IB_ENTER (function, (unint) handle, options, (unint) address,
	    (uint32_t) size);

  if (handle == 0)
    {
      IB_LOG_ERROR0 ("handle is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }
  (void) memset (handle, 0, sizeof (Pool_t));

#if USE_POOL_LOCK
  rc = vs_lock_init (&handle->lock, VLOCK_LOCKED, VLOCK_THREAD);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("vs_lock_init failed rc:", rc);
      IB_EXIT (function, VSTATUS_NOMEM);
      return VSTATUS_NOMEM;
    }
#endif /* USE_POOL_LOCK */

  if (name == 0)
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
      IB_LOG_ERROR0 ("name is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  for (namesize = (size_t) 0U; namesize < sizeof (handle->name); namesize++)
    {
      handle->name[namesize] = name[namesize];
      if (name[namesize] == (unsigned char) 0x00U)
	{
	  break;
	}
    }

  if (namesize >= sizeof (handle->name))
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
      IB_LOG_ERROR0 ("name doesn't contain a terminator");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if ((options & ~valid_options) != (uint32_t) 0x00U)
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
      IB_LOG_ERROR ("Invalid options specified: ", options);
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }
  handle->options = options;

  if (size == (size_t) 0x00U)
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
      IB_LOG_ERROR0 ("Pool size is zero");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

#ifdef __VXWORKS__
  /* cap size on VxWorks only */

  if (size > (size_t) 0xFFFFFFF0U)
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
      IB_LOG_ERROR ("Pool size is too big: high:", (uint32_t) (size));
      IB_LOG_ERROR ("Pool size is too big: low:", (uint32_t) (size));
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }
#endif

  handle->pagesize = vs_pool_page_size ();

  if ((options & VMEM_PAGE) == VMEM_PAGE)
    {
      if (size < handle->pagesize)
	{
#if USE_POOL_LOCK
	  (void) vs_unlock (&handle->lock);
	  (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
	  IB_LOG_ERROR ("size is less than pagesize:", size);
	  IB_EXIT (function, VSTATUS_NOMEM);
	  return VSTATUS_NOMEM;
	}
    }

  rc = vs_implpool_create (handle, options, name, address, size);
  if (rc == VSTATUS_OK)
    {
      handle->magic = VSPOOL_MAGIC;
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
#endif /* USE_POOL_LOCK */
    }
  else
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
    }

  IB_EXIT (function, rc);
  return rc;
}

/**********************************************************************
*
* FUNCTION
*    vs_pool_delete
*
* DESCRIPTION
*    Does fundamental common validation for vs_pool_delete
*    parameters and invokes environment specific implmentation.
*
* INPUTS
*
* OUTPUTS
*      Status_t - On success VSTATUS_OK is returned, otherwise
*      the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   DKJ     03/07/02    Initial creation of function.
**********************************************************************/
Status_t
vs_pool_delete (Pool_t * handle)
{
  Status_t rc;

  IB_ENTER (function, (unint) handle, (uint32_t) 0U, (uint32_t) 0U,
	    (uint32_t) 0U);

  if (handle == 0)
    {
      IB_LOG_ERROR0 ("handle is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (handle->magic != VSPOOL_MAGIC)
    {
      IB_LOG_ERRORX ("invalid handle magic:", handle->magic);
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

#if USE_POOL_LOCK
  rc = vs_lock (&handle->lock);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("vs_lock failed rc:", rc);
      IB_EXIT (function, VSTATUS_NXIO);
      return VSTATUS_NXIO;
    }
#endif /* USE_POOL_LOCK */

  rc = vs_implpool_delete (handle);
  if (rc == VSTATUS_OK)
    {
      handle->magic = ~VSPOOL_MAGIC;
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
      (void) vs_lock_delete (&handle->lock);
#endif /* USE_POOL_LOCK */
    }
  else
    {
#if USE_POOL_LOCK
      (void) vs_unlock (&handle->lock);
#endif /* USE_POOL_LOCK */
    }
  (void) memset (handle, 0, sizeof (Pool_t));

  IB_EXIT (function, rc);
  return rc;
}

/**********************************************************************
*
* FUNCTION
*    vs_pool_alloc
*
* DESCRIPTION
*    Does fundamental common validation for vs_pool_alloc
*    parameters and invokes environment specific implmentation.
*
* INPUTS
*
* OUTPUTS
*      Status_t - On success VSTATUS_OK is returned, otherwise
*      the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   DKJ     03/07/02    Initial creation of function.
**********************************************************************/
Status_t
vs_pool_alloc (Pool_t * handle, size_t length, void **loc)
{
  Status_t rc;

  IB_ENTER (function, (unint) handle, (uint32_t) length, (unint) loc,
	    (uint32_t) 0U);

  if (handle == 0)
    {
      IB_LOG_ERROR0 ("handle is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (handle->magic != VSPOOL_MAGIC)
    {
      IB_LOG_ERRORX ("invalid handle magic:", handle->magic);
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (loc == 0)
    {
      IB_LOG_ERROR0 ("loc is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (length == (size_t) 0x00U)
    {
      IB_LOG_ERROR0 ("length is zero");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

#if USE_POOL_LOCK
  rc = vs_lock (&handle->lock);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("vs_lock failed rc:", rc);
      IB_EXIT (function, VSTATUS_NXIO);
      return VSTATUS_NXIO;
    }
#endif /* USE_POOL_LOCK */

  rc = vs_implpool_alloc (handle, length, loc);

#if USE_POOL_LOCK
  (void) vs_unlock (&handle->lock);
#endif /* USE_POOL_LOCK */
  IB_EXIT (function, rc);
  return rc;
}

/**********************************************************************
*
* FUNCTION
*    vs_pool_free
*
* DESCRIPTION
*    Does fundamental common validation for vs_pool_free
*    parameters and invokes environment specific implmentation.
*
* INPUTS
*
* OUTPUTS
*      Status_t - On success VSTATUS_OK is returned, otherwise
*      the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   DKJ     03/07/02    Initial creation of function.
**********************************************************************/
Status_t
vs_pool_free (Pool_t * handle, void *loc)
{
  Status_t rc;

  IB_ENTER (function, (unint) handle, (unint) loc, (uint32_t) 0U,
	    (uint32_t) 0U);

  if (handle == 0)
    {
      IB_LOG_ERROR0 ("handle is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (handle->magic != VSPOOL_MAGIC)
    {
      IB_LOG_ERRORX ("invalid handle magic:", handle->magic);
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (loc == 0)
    {
      IB_LOG_ERROR0 ("loc is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

#if USE_POOL_LOCK
  rc = vs_lock (&handle->lock);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("vs_lock failed rc:", rc);
      IB_EXIT (function, VSTATUS_NXIO);
      return VSTATUS_NXIO;
    }
#endif /* USE_POOL_LOCK */

  rc = vs_implpool_free (handle, loc);

#if USE_POOL_LOCK
  (void) vs_unlock (&handle->lock);
#endif /* USE_POOL_LOCK */
  IB_EXIT (function, rc);
  return rc;
}

/**********************************************************************
*
* FUNCTION
*    vs_pool_size
*
* DESCRIPTION
*    Does fundamental common validation for vs_pool_size
*    parameters and invokes environment specific implmentation.
*
* INPUTS
*
* OUTPUTS
*      number of bytes allocated from pool.
**********************************************************************/
Status_t
vs_pool_size (Pool_t * handle, uint64_t *numBytesAlloc)
{
  Status_t rc;

  IB_ENTER (function, (unint) handle, 0, 0, 0);

  if (handle == 0)
    {
      IB_LOG_ERROR0 ("handle is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (handle->magic != VSPOOL_MAGIC)
    {
      IB_LOG_ERRORX ("invalid handle magic:", handle->magic);
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

  if (numBytesAlloc == 0)
    {
      IB_LOG_ERROR0 ("numBytesAlloc is null");
      IB_EXIT (function, VSTATUS_ILLPARM);
      return VSTATUS_ILLPARM;
    }

#if USE_POOL_LOCK
  rc = vs_lock (&handle->lock);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("vs_lock failed rc:", rc);
	  *numBytesAlloc = 0;
      IB_EXIT (function, VSTATUS_NXIO);
      return VSTATUS_NXIO;
    }
#endif /* USE_POOL_LOCK */

  rc = vs_implpool_size (handle, numBytesAlloc);

#if USE_POOL_LOCK
  (void) vs_unlock (&handle->lock);
#endif /* USE_POOL_LOCK */
  IB_EXIT (function, rc);
  return rc;
}
