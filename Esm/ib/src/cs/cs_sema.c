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
*      cs_sema.c
*
* DESCRIPTION
*      This file contains the implementation of the Counting Semaphores
*      services.
*
* DATA STRUCTURES
*
* FUNCTIONS
*      cs_sema_create
*      cs_sema_delete
*      cs_vsema
*      cs_psema
*
* DEPENDENCIES
*      cs_g.h
*
*
* HISTORY
*
* NAME      DATE        REMARKS
* MGR       03/14/02    Initial creation of file.
* MGR       03/26/02    Corrected lint errors.
* MGR       04/18/02    PR1728.  vs_psema should return VSTATUS_AGAIN
*                       error code, if thread has been killed.
* MGR       04/23/02    PR1742.  Added fix for blocking of cs_psema;
*                       cs_vsema should release lock if count goes to 1.
*
***********************************************************************/
#include "ib_types.h"
#include "ib_status.h"
#include "cs_g.h"
#include "vs_g.h"
#include "cs_log.h"
#define function __FUNCTION__
#ifdef __LINUX__
#include <semaphore.h>
#elif defined(__VXWORKS__) 
#include <drv/timer/timerDev.h>
#endif
#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID


/**********************************************************************
*
* FUNCTION
*    cs_sema_create
*
* DESCRIPTION
*    Initialize a counting semaphore object for use.
*
* INPUTS
*    handle   - A pointer to a semaphore object to be filled in by
*               the routine.
*    count    - The initial value for the semaphore.
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/14/02    Initial creation of function.
*
**********************************************************************/
Status_t
cs_sema_create (Sema_t *handle,
                    uint32_t count)
{
  Status_t          rc;

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)count,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == (Sema_t *) 0)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

#ifdef __VXWORKS__
  handle->magic = SEMAPHORE_MAGIC;
  handle->semId = semCCreate(SEM_Q_FIFO, count);
  handle->count = count;
  if (!handle->semId)
  {
      IB_LOG_ERROR0 ("failed to allocate semaphore");
      rc = VSTATUS_ILLPARM;
      IB_EXIT(function, rc);
      return rc;
  }
#else
  // LINUX USER SEMA
  handle->magic = SEMAPHORE_MAGIC;
  if (sem_init((sem_t *)&handle->osdSema, 0, 0) != 0)
  {
      IB_LOG_ERROR0 ("failed to allocate semaphore");
      rc = VSTATUS_ILLPARM;
      IB_EXIT(function, rc);
      return rc;
  }
#endif // vxworks

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    cs_sema_delete
*
* DESCRIPTION
*    Delete a counting semaphore object.
*
* INPUTS
*    handle   - A pointer to a semaphore object filled in by the
*               cs_sema_create() call.
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/14/02    Initial creation of function.
*
**********************************************************************/
Status_t
cs_sema_delete (Sema_t *handle)
{
  Status_t          rc;

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == (Sema_t *) 0)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != SEMAPHORE_MAGIC)
  {
    IB_LOG_ERRORX ("Invalid semaphore specified:", handle->magic);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

#ifdef __VXWORKS__
  if (!handle->semId)
  {
      IB_LOG_ERROR0 ("NULL semaphore");
      rc = VSTATUS_ILLPARM;
      IB_EXIT(function, rc);
      return rc;
  }
  if (ERROR == semDelete(handle->semId))
  {
      IB_LOG_ERROR0 ("failed to delete semaphore");
      rc = VSTATUS_ILLPARM;
      IB_EXIT(function, rc);
      return rc;
  }
  handle->semId = NULL;
  handle->count = 0;
#else
  // LINUX USER SEMA
  if (sem_destroy((sem_t *)&handle->osdSema) != 0) {
      IB_LOG_ERROR0 ("failed to delete semaphore");
      rc = VSTATUS_ILLPARM;
      IB_EXIT(function, rc);
      return rc;
  }
  memset((void *)&handle->osdSema, 0, sizeof(sem_t));
#endif

  IB_LOG_INFO ("semaphore deleted", (unint)handle);

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    cs_vsema
*
* DESCRIPTION
*    Increment a semaphore (post or give operation)
*
* INPUTS
*    handle   - A pointer to a semaphore object filled in by the
*               cs_sema_create() call.
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/14/02    Initial creation of function.
*   MGR     04/23/02    PR1742.  Added fix for blocking or cs_psema;
*                       cs_vsema should release lock if count goes to 1.
*
**********************************************************************/
Status_t
cs_vsema (Sema_t *handle)
{
  Status_t          rc;

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == (Sema_t *) 0)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != SEMAPHORE_MAGIC)
  {
    IB_LOG_ERRORX ("Invalid semaphore specified:", handle->magic);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

#ifdef __VXWORKS__
  if (ERROR == semGive(handle->semId))
  {
      IB_LOG_ERROR0 ("Failed to give semaphore");
      rc = VSTATUS_NXIO;
      IB_EXIT(function, rc);
      return rc;
  }
  handle->count++;
#else
  // LINUX USER SEMA
  if (sem_post((sem_t *)&handle->osdSema) != 0) {
      IB_LOG_ERROR0 ("Failed to give/post semaphore");
      rc = VSTATUS_NXIO;
      IB_EXIT(function, rc);
      return rc;
  }
#endif

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    cs_psema
*
* DESCRIPTION
*    Decrement a semaphore (wait or take operation)
*
* INPUTS
*    handle   - A pointer to a semaphore object filled in by the
*               cs_sema_create() call.
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/14/02    Initial creation of function.
*   MGR     04/18/02    PR1728.  vs_psema should return VSTATUS_AGAIN
*                       error code, if thread has been killed.
*
**********************************************************************/
Status_t
cs_psema_wait (Sema_t *handle, int timeout)
{
	Status_t          rc=VSTATUS_OK;

	IB_ENTER (function, (unint)handle, 0U, 0U, 0U);

  	/* Validate handle */
  	if (handle == (Sema_t *) 0)
  	{
    	IB_LOG_ERROR0 ("NULL handle pointer");
    	IB_EXIT (function, VSTATUS_ILLPARM);
    	return VSTATUS_ILLPARM;
  	}

  	if (handle->magic != SEMAPHORE_MAGIC)
  	{
    	IB_LOG_ERRORX ("Invalid semaphore specified:", handle->magic);
    	IB_EXIT (function, VSTATUS_NXIO);
    	return VSTATUS_NXIO;
  	}

  	if (timeout == CS_SEMA_WAIT_FOREVER) {
		return(cs_psema(handle));
  	} else {
#ifdef __VXWORKS__
  		if (ERROR == semTake(handle->semId, timeout*sysClkRateGet()))
  		{
      		IB_LOG_WARN0("Timed out trying to take semaphore");
      		rc = VSTATUS_NXIO;
  		} else {
            handle->count--;
        }
#elif defined (__LINUX__)
  		// LINUX USER SEMA
		int i=0, rc=-1;
		do {
  			rc = sem_trywait((sem_t *)&handle->osdSema);
			++i;
			(void)vs_thread_sleep(VTIMER_1S);
		} while (i<timeout && rc);
  		if (rc != 0) {
      		IB_LOG_WARN0("Timed out trying to take semaphore");
      		rc = VSTATUS_NXIO;
  		}
#endif
	}
  	IB_EXIT (function, rc);
    return rc;
}

/**********************************************************************
*
* FUNCTION
*    cs_psema
*
* DESCRIPTION
*    Decrement a semaphore (wait or take operation)
*
* INPUTS
*    handle   - A pointer to a semaphore object filled in by the
*               cs_sema_create() call.
*
* OUTPUTS
*    Status_t - On success VSTATUS_OK is returned, otherwise
*    the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     03/14/02    Initial creation of function.
*   MGR     04/18/02    PR1728.  vs_psema should return VSTATUS_AGAIN
*                       error code, if thread has been killed.
*
**********************************************************************/
Status_t
cs_psema (Sema_t *handle)
{
  Status_t          rc;

  IB_ENTER (function,
            (unint)handle,
            (uint32_t)0U,
            (uint32_t)0U,
            (uint32_t)0U);

  /*
  ** Validate handle
  */
  if (handle == (Sema_t *) 0)
  {
    IB_LOG_ERROR0 ("NULL handle pointer");
    IB_EXIT (function, VSTATUS_ILLPARM);
    return VSTATUS_ILLPARM;
  }

  if (handle->magic != SEMAPHORE_MAGIC)
  {
    IB_LOG_ERRORX ("Invalid semaphore specified:", handle->magic);
    IB_EXIT (function, VSTATUS_NXIO);
    return VSTATUS_NXIO;
  }

#ifdef __VXWORKS__
  if (ERROR == semTake(handle->semId, WAIT_FOREVER))
  {
      IB_LOG_ERROR0 ("Failed to take semaphore");
      rc = VSTATUS_NXIO;
      IB_EXIT(function, rc);
      return rc;
  }
  handle->count--;
#else
  // LINUX USER SEMA
  if (sem_wait((sem_t *)&handle->osdSema) != 0) {
      IB_LOG_ERROR0 ("Failed to take semaphore");
      rc = VSTATUS_NXIO;
      IB_EXIT(function, rc);
      return rc;
  }
#endif

  IB_EXIT (function, VSTATUS_OK);
  return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    cs_sema_getcount
*
* DESCRIPTION
*    return counting semaphore count
*
* INPUTS
*    handle   - A pointer to a semaphore object filled in by the
*               cs_sema_create() call.
*
* OUTPUTS
*    int - semaphore count
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   JMS     02/20/07    Initial creation of function.
*
**********************************************************************/
Status_t cs_sema_getcount(Sema_t *handle, int *semCount)
{
	Status_t rc = VSTATUS_OK;

	IB_ENTER (function, (unint)handle, 0U, 0U, 0U);

  	/* Validate handle */
  	if (handle == (Sema_t *) 0) {
    	IB_LOG_ERROR0 ("NULL handle pointer");
    	rc = VSTATUS_ILLPARM;
  	} else if (handle->magic != SEMAPHORE_MAGIC) {
    	IB_LOG_ERRORX ("Invalid semaphore specified:", handle->magic);
    	rc = VSTATUS_NXIO;
  	} else {
#ifdef __VXWORKS__
        *semCount = handle->count;
#elif defined (__LINUX__)
        if ((sem_getvalue((sem_t *)&handle->osdSema, semCount)) != 0)
        {
            IB_LOG_ERROR0 ("Failed to get semaphore count");
            rc = VSTATUS_NXIO;
        }
#endif
    }
    IB_EXIT (function, rc);
    return rc;
}
