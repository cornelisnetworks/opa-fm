/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/***********************************************************************
* 
* FILE NAME
*      vs_thr_common.c
*
* DESCRIPTION
*      This file contains the API entry and validation common among
*      all thread environments.  It will invoke the private
*      implementation specific version of the API.
*
*      General non-platform specific helpers are available as well.
*
* DATA STRUCTURES
*
* FUNCTIONS
*      vs_thread_create()
*
* DEPENDENCIES
*      vs_g.h
*
*
* HISTORY
*
* NAME      DATE        REMARKS
* SFW       03/05/02    Initial creation of file.
* PJG       04/01/02    PR 1676. vs_thread_sleep prototype per OS API 2.0g
***********************************************************************/
#include "ib_types.h"
#include "ib_status.h"
#include "vs_g.h"
#include "cs_g.h"
#include "cs_log.h"
#define function __FUNCTION__

#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID

/*
** Implmentation specific entry points.
*/
extern Status_t
impl_vs_thread_create (Thread_t *thr, 
                       unsigned char *name, 
                       void (*start_function)(uint32_t, uint8_t **),
                       uint32_t argc,
                       uint32_t argv_copy,
                       uint8_t  *argv[],
                       size_t stack_size);

extern Status_t
impl_vs_thread_name (Threadname_t *name);

extern const char*
impl_vs_thread_name_str (void);

extern Status_t
impl_vs_thread_groupname (Threadname_t name, 
                          Threadname_t *grpname);

extern Status_t
impl_vs_thread_exit (Thread_t *thr);

extern Status_t
impl_vs_thread_kill (Thread_t *thr);

extern void
impl_vs_thread_sleep (uint64_t sleep_time);

extern Status_t
impl_vs_thread_join(Thread_t *thr,
                          void **value_ptr);

/**********************************************************************
*
* FUNCTION
*    vs_real_thread_create       
*
* DESCRIPTION
*    Does the actual thread creation; called only by vs_thread_create()
*    and vs_thread_create2().
*
**********************************************************************/
static Status_t
vs_real_thread_create (Thread_t *thr, 
                       unsigned char *name, 
                       void (*start_function)(uint32_t, uint8_t **),
                       uint32_t argc,
                       uint32_t argv_copy,
                       uint8_t *argv[],
                       size_t stack_size)
{
    uint32_t          count;
    Status_t          status;


    IB_ENTER (function, (unint) thr, (unint) name, 
              (unint) start_function, (uint32_t) argc);

    /*
    ** Verify required pointer arguments.
    */
    if (thr == 0)
    {
        IB_LOG_ERROR0 ("Thread_t parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
  
    if (name == 0)
    {
        IB_LOG_ERROR0 ("name parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }

    if (start_function == 0)
    {
        IB_LOG_ERROR0 ("startup function parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
 
    /*
    ** Look for inconsistencies in argc and argv.
    */
    for (count = 0U; count < argc; count++)
    {
        if (!argv || !argv[count])
        {
            IB_LOG_ERROR0 ("argc and argv[] are inconsistent");
            IB_EXIT (function, VSTATUS_ILLPARM);
            return VSTATUS_ILLPARM;
        }
    }        

    if (stack_size == (size_t) 0U)
    {
        IB_LOG_ERROR0 ("stack_size parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }

    /*
    ** Zero the Thread control object and initialize.
    */
    memset(thr, 0, sizeof(Thread_t));

    /*
    ** Move in the name, validate name length is < VS_NAME_MAX.
    */
    for (count = 0U; count < VS_NAME_MAX; count++)
    {
        thr->name[count] = name[count];
        if (name[count] == (unsigned char) 0U)
            break; 
    }

    if (count >= VS_NAME_MAX)
    {
        IB_LOG_ERROR0("name too big");
        IB_EXIT(function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
 
    /*
    ** Invoke environment specific implmentation and return status.
    */ 
    status = impl_vs_thread_create (thr, name, start_function,
                                    argc, argv_copy, argv, 
				    stack_size);

    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_create       
*
* DESCRIPTION
*    Does fundamental common validation for vs_thread_create
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
*   SFW     03/05/02    Initial creation of function.
*   DKJ     04/04/02    PR1676. OS API 2.0g update
**********************************************************************/
Status_t
vs_thread_create (Thread_t *thr, 
                      unsigned char *name, 
                      void (*start_function)(uint32_t, uint8_t **),
                      uint32_t argc,
                      uint8_t *argv[],
                      size_t stack_size)
{
    Status_t		status;

    IB_ENTER (function, (unint) thr, (unint) name, 
              (unint) start_function, (uint32_t) argc);

    status = vs_real_thread_create(thr,
	    			   name,
				   start_function,
				   argc,
				   0,	/* do not copy argv */
				   argv,
				   stack_size);
    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_create2
*
* DESCRIPTION
*    Same as vs_thread_create() except that the argv vector
*    is copied.  This allows the caller to safely pass in an argv
*    created on the stack without worrying about it becoming
*    a dangling pointer.
*
* INPUTS
*    same as vs_thread_create
*      
* OUTPUTS
*    same as vs_thread_create
*
**********************************************************************/
Status_t
vs_thread_create2 (Thread_t *thr, 
                      unsigned char *name, 
                      void (*start_function)(uint32_t, uint8_t **),
                      uint32_t argc,
                      uint8_t *argv[],
                      size_t stack_size)
{
    Status_t		status;

    IB_ENTER (function, (unint) thr, (unint) name, 
              (unint) start_function, (uint32_t) argc);

    status = vs_real_thread_create(thr,
	    			   name,
				   start_function,
				   argc,
				   1,	/* copy argv */
				   argv,
				   stack_size);
    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_name       
*
* DESCRIPTION
*    Return the globally unique thread name opaque identifier for
*    the currently executing thread. When running in interrupt context, 
*    the identifier VTRHREAD_INTERRUPT is returned.
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
*   SFW     03/05/02    Initial creation of function.
**********************************************************************/
Status_t
vs_thread_name (Threadname_t *name) 
{
    Status_t     status;

    IB_ENTER (function, (unint) name, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Verify required pointer arguments.
    */
    if (name == 0)
    {
        IB_LOG_ERROR0 ("name parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }

    /*
    ** Invoke environment specific implmentation and return status.
    */ 
    status = impl_vs_thread_name(name);

    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_name_str
*
* DESCRIPTION
*    Return the globally unique thread name string for
*    the currently executing thread.
*
* INPUTS
*      
* OUTPUTS
*      thread name string
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   SFW     03/05/02    Initial creation of function.
**********************************************************************/
const char*
vs_thread_name_str (void)
{
    /*
    ** Invoke environment specific implmentation
    */ 
    return impl_vs_thread_name_str();
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_groupname       
*
* DESCRIPTION
*    Return the globaly unique opaque thread group id for the 
*    running user thread.
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
*   SFW     03/05/02    Initial creation of function.
**********************************************************************/
Status_t
vs_thread_groupname (Threadname_t name, Threadname_t *grpname) 
{
    Status_t     status;

    IB_ENTER (function, name, (unint) grpname, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Verify required pointer arguments.
    */
    if (grpname == 0)
    {
        IB_LOG_ERROR0 ("group name parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
  
    /*
    ** Invoke environment specific implmentation and return status.
    */ 
    status = impl_vs_thread_groupname(name, grpname);

    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_exit       
*
* DESCRIPTION
*    Exit the currently running thread.  
*
* INPUTS
*      
* OUTPUTS
*      Status_t - On success does not return, otherwise
*      the cause of the error.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   SFW     03/06/02    Initial creation of function.
**********************************************************************/
Status_t
vs_thread_exit (Thread_t *handle)
{
    Status_t     status;

    IB_ENTER (function, (unint) handle, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Verify required pointer arguments.
    */
    if (handle == 0)
    {
        IB_LOG_ERROR0 ("handle parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
  
    /*
    ** Invoke environment specific implmentation and return status.
    */ 
    status = impl_vs_thread_exit(handle);

    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_kill       
*
* DESCRIPTION
*    Kill the specified thread.
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
*   SFW     03/06/02    Initial creation of function.
**********************************************************************/
Status_t
vs_thread_kill (Thread_t *handle)
{
    Status_t     status;

    IB_ENTER (function, (unint) handle, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Verify required pointer arguments.
    */
    if (handle == 0)
    {
        IB_LOG_ERROR0 ("handle parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
  
    /*
    ** Invoke environment specific implmentation and return status.
    */ 
    status = impl_vs_thread_kill(handle);

    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_sleep      
*
* DESCRIPTION
*    Sleep for the specified number of microseconds.
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
*   SFW     03/06/02    Initial creation of function.
*   PJG     04/02/02    PR 1676. vs_thread_sleep prototype per OS API 2.0g
**********************************************************************/
void
vs_thread_sleep (uint64_t sleep_time)
{

    IB_ENTER (function, (uint32_t) sleep_time, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Invoke environment specific implmentation.
    */ 
    impl_vs_thread_sleep(sleep_time);

    IB_EXIT (function, VSTATUS_OK);
    return;
}

/**********************************************************************
*
* FUNCTION
*    vs_thread_join
*
* DESCRIPTION
*   Wait for the specified thread to exit.
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
*   SR     10/24/16    Initial creation of function.
**********************************************************************/
Status_t
vs_thread_join (Thread_t *handle, void **value_ptr)
{
    Status_t     status;

    IB_ENTER (function, (unint) handle, (unint) value_ptr, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Verify required pointer arguments.
    */
    if (handle == 0)
    {
        IB_LOG_ERROR0 ("handle parameter is null");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }

    /*
    ** Invoke environment specific implmentation and return status.
    */
    status = impl_vs_thread_join(handle, value_ptr);

    IB_EXIT (function, status);
    return status;
}
