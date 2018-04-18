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
*      vs_thr.c
*
* DESCRIPTION
*      This file contains the Linux user space implementation of the 
*      vs_thread services.
*
* DATA STRUCTURES
*
* FUNCTIONS
*
* DEPENDENCIES
*      vs_g.h
*
*
* HISTORY
*
* NAME      DATE        REMARKS
* SFW       03/08/02    Initial creation of file.
* SFW       03/10/02    Initial functions all passing testcases.
* MGR       04/19/02    Changed pthread_kill call to pthread_cancel.
***********************************************************************/
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#if 0
#include <fcntl.h>
#endif

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

/*
** Implementation private data structure for Linux Kernel Thread
*/
#define IMPLPRIV_THREAD_MAGIC ((uint32_t) 0x217B3135U)

typedef struct {
    uint32_t            magic;
    void                (*start_function)(uint32_t, uint8_t **);  
    pthread_t           thread;
    Threadname_t    create_pid;
    Threadname_t    create_ppid;
    uint32_t            argc;
    uint8_t             argv_copied;
    uint8_t             **argv;
    uint32_t            dead;
    pthread_cond_t	dead_cond;
    pthread_mutex_t	dead_mutex;
	pthread_attr_t	attr;
	char			name[64];
} Implpriv_Thread_t;

/*
** Local prototypes
*/
int
impl_vs_thread_start (void *);

//static __thread char *thread_name;
//
int name_key_init = 0;
pthread_key_t name_key;

void vs_thread_key_init(void)
{
	(void)pthread_key_create(&name_key, NULL);
	(void)pthread_setspecific(name_key, "main");
	name_key_init = 1;
}


/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_create       
*
* DESCRIPTION
*    Linux specific implementation of the vs_thread_create() function.
*    The supplied stack parameters are not used in the Linux user space
*    implementation.
*
*    This routine is called by the vs_thread_create() API.
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
*   SFW     03/10/02    Initial creation of function.
*   DKJ     04/04/02    PR1676. OS API 2.0g updates
**********************************************************************/
Status_t
impl_vs_thread_create (Thread_t *thr, 
                      unsigned char *name, 
                      void (*start_function)(uint32_t, uint8_t **),
                      uint32_t argc,
                      uint32_t argv_copy,
                      uint8_t  *argv[],
                      size_t stack_size)
{
    static const char function[] = "impl_vs_thread_create";
    Implpriv_Thread_t *impl;
    Status_t          status = VSTATUS_OK;
    int               pstatus;
    /*struct sched_param schedParams;*/

    IB_ENTER (function, (unint) thr, (unint) name, 
              (unint) start_function, (uint32_t) argc);

    /*
    ** This is a run-time check to ensure that the opaque section of 
    ** Thread_t is large enough to contain the Implementation private 
    ** data structure. If this test fails, adjust the OPAQUE_EVENT_SIZE_WORDS 
    ** define in vs_g.h and rebuild.
    */
    if (sizeof(Implpriv_Thread_t) > sizeof (thr->opaque))
    {
        IB_LOG_ERROR ("Implementation specific data to big:", 
                      sizeof(Implpriv_Thread_t));
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
	if (! name_key_init)
		vs_thread_key_init();

    /*
    ** Do the real Linux kernel specific stuff here
    */
    impl = (Implpriv_Thread_t *) & thr->opaque[0];

    impl->magic = IMPLPRIV_THREAD_MAGIC;
    impl->start_function = start_function;
	strncpy(impl->name, (const char*)name, sizeof(impl->name)-1);
	impl->name[sizeof(impl->name)-1] = '\0';

    /*
    ** If the argv vector is to be copied, malloc a single
    ** chunk of memory for it.  It will be freed by a cleanup
    ** handler (installed by impl_vs_thread_start) when
    ** the thread exits.
    */
    if (argv_copy)
    {
	int	i, size = 0;
	uint8_t	**new_argv;
	char	*p;

	for (i = 0; i < argc; ++i)
	{
	    size += (int)strlen((void *)argv[i])+1;
	}

	new_argv = (uint8_t **)malloc(argc*sizeof(void *) + size);
	if (new_argv == NULL)
	{
	    IB_LOG_ERROR0("Could not malloc memory for argv copy");
	    return VSTATUS_NOMEM;
	}

	p = (char *)(new_argv + argc);  /* Put env right after argv vector. */
	for (i = 0; i < argc; ++i)
	{
	    new_argv[i] = (void *)p;
	    cs_strlcpy(p, (void *)argv[i], size);
	    p += strlen((void *)argv[i])+1;
		size -= (strlen((void *)argv[i])+1);
	}
	impl->argv = new_argv;
	impl->argv_copied = 1;
    }
    else
    {
	impl->argv = argv;
	impl->argv_copied = 0;
    }
    impl->argc = argc;


    /*
    ** Initialize dead flag and associated condition variable and
    ** mutex.  These are used to ensure only one thread is ever
    ** allowed to kill the target thread.
    */
    impl->dead = 0;
    pthread_cond_init(&impl->dead_cond, NULL);
    pthread_mutex_init(&impl->dead_mutex, NULL);
    /* switch to default thread create with no attributes */
    #if 0
    pthread_attr_init(&impl->attr);
    pthread_attr_setstack(&impl->attr,stack,stack_size);
	if(thr->priority != 0)
	{
		schedParams.sched_priority = thr->priority;
		pthread_attr_setschedparam(&impl->attr,&schedParams);
        pthread_attr_setinheritsched (&impl->attr,PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&impl->attr, SCHED_RR);
	}

    /*
    ** Start the thread.
    */
    pstatus = pthread_create(&impl->thread, &impl->attr, 
                             (void * (*)(void *))impl_vs_thread_start, 
                             (void *) impl);
    #endif
    /* cannot send local impl pointer as argument, maybe gone before thread starts */
    pstatus = pthread_create(&impl->thread, NULL, 
                             (void *)impl_vs_thread_start, 
                             (void *)thr->opaque);
    if (pstatus)
    {
        status = (pstatus == EAGAIN) ? VSTATUS_OK : VSTATUS_BAD;
        IB_LOG_ERROR("Thread creation failed errno:", (uint32_t) pstatus);
        IB_LOG_ERRORRC("Thread creation rc:", status);
    }

    impl->create_pid = (Threadname_t) impl->thread;
    impl->create_ppid = (Threadname_t) (int64_t) getppid();

    IB_LOG_INFINI_INFOLX("new thread's name", (uint64_t) impl->create_pid); 
    IB_LOG_INFINI_INFOLX("new thread's pid ", (uint64_t) getpid()); 
    IB_LOG_INFINI_INFOLX("new thread's group", (uint64_t) impl->create_ppid); 
    IB_EXIT (function, status);
    return status;
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_name       
*
* DESCRIPTION
*    Linux user space specific implementation of the vs_thread_name().
*    This routine is called by the vs_thread_create() API.
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
*   SFW     03/10/02    Initial creation of function.
**********************************************************************/
Status_t
impl_vs_thread_name (Threadname_t *name) 
{
    static const char function[] = "impl_vs_thread_name";

    IB_ENTER (function, (unint) name, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Remember parms validated by API.
    ** Return the thread ID
    */
    //*name = (Threadname_t) (int64_t) getpid();
    *name = (Threadname_t) pthread_self();

    IB_EXIT (function, VSTATUS_OK);
    return VSTATUS_OK;
}

const char *impl_vs_thread_name_str (void)
{
	//return thread_name;
	if (! name_key_init) {
		return "main";
	} else {
		char* name = pthread_getspecific(name_key);
		return (name)?name:"<null>";
	}
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_groupname       
*
* DESCRIPTION
*    Linux kernel specific implementation of the vs_thread_groupname() 
*    API. This routine is called by the vs_thread_create() API.
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
*   SFW     03/10/02    Initial creation of function.
**********************************************************************/
Status_t
impl_vs_thread_groupname (Threadname_t name, Threadname_t *grpname) 
{
    static const char function[] = "impl_vs_thread_groupname";

    IB_ENTER (function, name, (unint) grpname, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Remember parms validated by API.
    ** Return the process pid
    */
    *grpname = (Threadname_t) (int64_t) getpid();

    IB_EXIT (function, VSTATUS_OK);
    return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_exit       
*
* DESCRIPTION
*    Linux user space specific implementation of the vs_thread_exit() 
*    API. This routine is called by the vs_thread_exit() API.
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
*   SFW     03/10/02    Initial creation of function.
**********************************************************************/
Status_t
impl_vs_thread_exit (Thread_t *handle) 
{
    static const char function[] = "impl_vs_thread_exit";

    IB_ENTER (function, (unint) handle, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** This will cause the thread's cleanup handler,
    ** impl_vs_thread_exit_handler(), to be called.
    ** That's where all thread cleanup is done.
    */
    pthread_exit(NULL);
   
    /*
    ** Should not return to here!
    */ 
    IB_EXIT (function, VSTATUS_BAD);
    return VSTATUS_BAD;
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_kill
*
* DESCRIPTION
*    Linux user space specific implementation of the vs_thread_kill() 
*    API. This routine is called by the vs_thread_kill() API.
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
*   SFW     03/08/02    Initial creation of function.
*   MGR     04/19/02    Changed pthread_kill call to pthread_cancel,
*                       since pthread_kill was terminating program.
**********************************************************************/
Status_t
impl_vs_thread_kill (Thread_t *handle) 
{
    static const char function[] = "impl_vs_thread_kill";
    Implpriv_Thread_t *impl;
    Status_t          status;
    int               pstatus;

    IB_ENTER (function, (unint) handle, (uint32_t) 0U, (uint32_t) 0U,
              (uint32_t) 0U);

    /*
    ** Remember parms validated by API.
    */

    /*
    ** This is a run-time check to ensure that the opaque section of 
    ** Thread_t is large enough to contain the Implementation private 
    ** data structure. If this test fails, adjust the OPAQUE_EVENT_SIZE_WORDS 
    ** define in vs_g.h and rebuild.
    */
    if (sizeof(Implpriv_Thread_t) > sizeof (handle->opaque))
    {
        IB_LOG_ERROR ("Implementation specific data to big:", 
                      sizeof(Implpriv_Thread_t));
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }

    /*
    ** Do the real Linux kernel specific stuff here
    */
    impl = (Implpriv_Thread_t *) & handle->opaque[0];

    /*
    ** Validate magic
    */
    if (impl->magic != IMPLPRIV_THREAD_MAGIC)
    {
        IB_LOG_ERRORX ("Bad thread control block:", impl->magic);
        IB_EXIT (function, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }

    /*
    ** If the thread is already dead or if another killer is
    ** killing the thread, don't bother.
    */
    pthread_mutex_lock(&impl->dead_mutex);
    if (!impl->dead) {
	IB_LOG_INFO("Kill Threadname", (uint32_t) impl->create_pid);
	impl->dead = 1;
	pstatus = pthread_cancel(impl->thread);
	if (pstatus)
	{
	    pthread_mutex_unlock(&impl->dead_mutex);
	    status = (pstatus == ESRCH) ? VSTATUS_ILLPARM : VSTATUS_BAD;
	    IB_LOG_ERROR("Thread killed failed errno:", (uint32_t) pstatus);
	    IB_LOG_ERRORRC("Thread killed failed rc:", status);
	    IB_EXIT(function, status);
	    return status;
	}
	pthread_cond_wait(&impl->dead_cond, &impl->dead_mutex);
    }
    else {
	pthread_mutex_unlock(&impl->dead_mutex);
    }

    IB_EXIT (function, VSTATUS_OK);
    return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_sleep
*
* DESCRIPTION
*    Linux user space specific implementation of the vs_thread_sleep() 
*    API. This routine is called by the vs_thread_sleep() API.
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
*   SFW     03/10/02    Initial creation of function.
*   PJG     04/02/02    PR 1676. vs_thread_sleep prototype per OS API 2.0g
**********************************************************************/
void
impl_vs_thread_sleep (uint64_t sleep_time) 
{
    static const char function[] = "impl_vs_thread_sleep";
    struct timespec   ts;
    struct timespec   ts_actual;
    long              micro_seconds;
    
    IB_ENTER (function, sleep_time, 0U,  0U, 0U);

    /*
    ** Remember parms validated by API.
    */
    ts.tv_sec       = (time_t) (sleep_time / 1000000ULL);
    micro_seconds   = (long) (sleep_time % 1000000ULL);
    ts.tv_nsec      = micro_seconds * 1000;
    (void) nanosleep( &ts, &ts_actual);
    
    IB_EXIT (function, VSTATUS_OK);
    return;
}

/**********************************************************************
*
* FUNCTION
*    argv_free
*
* DESCRIPTION
*    Cleanup function called when a thread is killed or exits normally.
*    I.e., any thread exit will funnel through here.
*
* INPUTS
*    Pointer to impl structure.
*      
* OUTPUTS
*    -
*
*
**********************************************************************/
static void
impl_vs_thread_exit_handler(void *arg)
{
    Implpriv_Thread_t	*thr_impl = (Implpriv_Thread_t *)arg;
    static const char	function[] = "impl_vs_thread_exit_handler";

    IB_ENTER (function, (unint) thr_impl, 0U, 0U, 0U);

    /*
    ** If argv was copied, free it now.
    */
    if (thr_impl->argv_copied) {
	IB_LOG_INFO("freeing argv", thr_impl->argv);
	free(thr_impl->argv);
    }

    /*
    ** Set our dead flag to 1, telling any would-be killers not
    ** to bother.  Also signal any waiting killers that we're now dead.
    */
    pthread_mutex_lock(&thr_impl->dead_mutex);
    thr_impl->dead = 1;
    pthread_cond_broadcast(&thr_impl->dead_cond);
    pthread_mutex_unlock(&thr_impl->dead_mutex);

    IB_EXIT (function, (uint32_t) 0U);
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_start
*
* DESCRIPTION
*    Linux user space specific helper to wrap the start a thread.
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
*   SFW     03/10/02    Initial creation of function.
**********************************************************************/
int
impl_vs_thread_start (void *impl)
{
    static const char function[] = "impl_vs_thread_start";
    Implpriv_Thread_t *thr_impl = (Implpriv_Thread_t *)impl;

    IB_ENTER (function, 0U, 0U, 0U, 0U);

    /*
    ** Only called by kernel create function.
    */
    if (!thr_impl)
    {
        IB_LOG_ERROR0("Internal logic error, no create data");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return -1; 
    }
	(void)pthread_setspecific(name_key, thr_impl->name);
	//thread_name = thr_impl->name;

    /*
    ** Enable thread cancellation and make it immediate -- this
    ** enables a vs_thread_kill() to kill the thread on-demand
    ** without waiting for the target thread to reach a
    ** cancellation point.
    */
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    /*
    ** Bracket the thread entry point with the installation/removal
    ** of a cleanup handler.  If the thread exits normally or is
    ** killed, the cleanup handler will be called.
    */
    pthread_cleanup_push(impl_vs_thread_exit_handler, thr_impl);

    thr_impl->start_function(thr_impl->argc, thr_impl->argv);
    pthread_exit(NULL);  /* Will call impl_vs_thread_exit_handler() */

    pthread_cleanup_pop(0);

    /*NOTREACHED*/
    return 0;
}

/**********************************************************************
*
* FUNCTION
*    impl_vs_thread_join
*
* DESCRIPTION
*    Linux user space specific implementation of the vs_thread_join()
*    API. This routine is called by the vs_thread_join() API.
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
impl_vs_thread_join (Thread_t *handle, void **value_ptr)
{
    static const char function[] = "impl_vs_thread_join";
    Status_t status;
    Implpriv_Thread_t *impl;

    IB_ENTER (function, (unint)handle, (unint) value_ptr, (uint32_t) 0U,
              (uint32_t) 0U);

    impl = (Implpriv_Thread_t *)handle->opaque;

    if (!impl)
    {
        IB_LOG_ERROR0("Internal logic error, no create data");
        IB_EXIT (function, VSTATUS_ILLPARM);
        return -1;
    }

    status = pthread_join(impl->thread, NULL);

    IB_EXIT (function, status);
    return status;
}
