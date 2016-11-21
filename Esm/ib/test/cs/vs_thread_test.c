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
*      vs_thread_test.c
*
* DESCRIPTION
*      This file contains vs_thread common test routines. These tests
*      exercise the vs_thread services.
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
* SFW       03/05/02    Initial creation of file.
* PJG       03/12/02    Call vs_thread_exit in each thread to keep
*                       ATI happy.
* PJG       04/02/02    PR1676. Update to OS API 2.0g.
* MGR       04/25/02    PR1651.  Corrected vs_thread_kill_2b().  In Linux
*                       Kernel, the sleeping thread will awaken when killed.
***********************************************************************/
#if defined (LINT)
#define __signed__ signed
#include <bits/sigset.h>
#endif
#include <stdio.h>
#include <ib_types.h>
#include  <ib_status.h>
#include <vs_g.h>
#include <cs_g.h>

/*
** Macros for consistency
*/
#define DOATEST(func, pass, fail) ((func)() == VSTATUS_OK) ? pass++ : fail++
#define HALF_SECOND ((uint64_t) 500000U)
#define MIN_HALF_SECOND ((uint64_t) 250000U)
#define MAX_HALF_SECOND ((uint64_t) 750000U)
#define ONE_SECOND ((uint64_t) 1000000U)
#define MIN_ONE_SECOND ((uint64_t) 750000U)
#define MAX_ONE_SECOND ((uint64_t) 1250000U)
#define TWO_SECONDS ((uint64_t) 2000000U)
#define MIN_TWO_SECONDS ((uint64_t) 1750000U)
#define MAX_TWO_SECONDS ((uint64_t) 2250000U)
#define FIVE_SECONDS ((uint64_t) 5000000U)
#define MIN_FIVE_SECONDS ((uint64_t) 4750000U)
#define MAX_FIVE_SECONDS ((uint64_t) 5250000U)
#define TEN_SECONDS ((uint64_t) 10000000U)
#define MIN_TEN_SECONDS ((uint64_t) 7500000U)
#define MAX_TEN_SECONDS ((uint64_t) 12500000U)
#define NOWAIT_MAX ((uint64_t) 250000U)

#define THR_TEST_NOT_INIT    0U
#define THR_TEST_PASSED      1U
#define THR_TEST_FAILED      2U
#define THR_SLEEPING         1U
#define THR_NOT_SLEEPING     2U

/*
** Thread values for testing
*/
#define MAX_THREADS      5
#define MAX_STACK_SIZE   4096
#define MAX_STACK_INTS   (4096/4)
#define MAX_ARGS         10
#define MAX_ARG_SIZE     30
#define NUM_JOIN_THREADS 2

typedef struct thread_info {
    Thread_t      thread;
    uint32_t          status;
    uint32_t          exit_status;
    uint32_t          status_value;
    uint32_t          sleep_status;
    uint64_t          sleep_time;
    uint64_t          act_sleep_time;
    Threadname_t  name;
    Threadname_t  grpname;
    size_t            stack_size;
    unsigned int      *stack;
    uint32_t          argc;
    uint8_t           *argv[MAX_ARGS];   /* allow to create weird errors */
    uint8_t           args[MAX_ARGS][MAX_ARG_SIZE];
} Thread_info_t; 

static Thread_info_t thr[MAX_THREADS];

#define CLEAR_THR_TEST_INFO(thr) \
        memset((void *)thr, 0, sizeof(thr));

/*
** Externs that will be specific to environment.
*/
extern void generic_sleep(uint64_t);

/*
** Generic string to uint32_t function
*/
static uint32_t my_atou(uint8_t *p)
{
    uint32_t   value = 0;

    while (p && *p)
    {
        value *= 10;
        value += *p - '0';
        p++;
    }
    return value;
}

/*
** Thread startup test function
*/
static void thr_test_startup(uint32_t argc, uint8_t *argv[])
{
    uint32_t    idx;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);

    /*
    ** Set status saying we were created.
    */
    thr[idx].status       = THR_TEST_PASSED;
    thr[idx].status_value = VSTATUS_OK;

    /*
    ** Exit.
    */
    (void) vs_thread_exit(&thr[idx].thread);
    return;
}

/*
** Thread startup test  no args function
*/
static void thr_test_noarg_startup(uint32_t argc, uint8_t *argv[])
{
    /*
    ** Set status saying we were created.
    */
    thr[0].status       = THR_TEST_PASSED;
    thr[0].status_value = VSTATUS_OK;

    /*
    ** Exit.
    */
    (void) vs_thread_exit(&thr[0].thread);
    return;
}

/*
** Thread name test
*/
static void thr_name_test(uint32_t argc, uint8_t *argv[])
{
    Status_t    status;
    uint32_t    idx;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);
    thr[idx].status =  THR_TEST_FAILED;

    /*
    ** Set status saying we were created.
    */

    /*
    ** Get my thread name
    */
    status = vs_thread_name(&thr[idx].name);

    thr[idx].status =  THR_TEST_PASSED;
    thr[idx].status_value = status;

    /*
    ** Do exit test too if argc > 1
    */
    if (argc > 1)
    {
        thr[idx].exit_status = THR_TEST_PASSED;
        status = vs_thread_exit(&thr[idx].thread);
        thr[idx].status      = status;
        thr[idx].exit_status = THR_TEST_FAILED;
    }
    else
    {
        (void) vs_thread_exit(&thr[idx].thread);
    }
    return;
}

/*
** Thread groupname test
*/
static void thr_groupname_test(uint32_t argc, uint8_t *argv[])
{
    Status_t    status;
    uint32_t    idx;
    Threadname_t  bad_name;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);

    thr[idx].status = THR_TEST_FAILED;
    
    /*
    ** Get my thread name
    */
    status = vs_thread_name(&thr[idx].name);
    if (status == VSTATUS_OK)
    {
        /*
        ** Get my thread group name, argc used to dictate
        ** special test cases.
        */
        switch (argc)
        {
            case 1 :
                status = vs_thread_groupname(thr[idx].name, 
                                                 &thr[idx].grpname);
                thr[idx].status = THR_TEST_PASSED;
                 break;

            case 2 :
                status = vs_thread_groupname(thr[idx].name, 
                                                 (Threadname_t *)0);
                thr[idx].status = THR_TEST_PASSED;
                 break;

            case 3 :
                bad_name = thr[idx].name + 1;
                status = vs_thread_groupname(bad_name, 
                                                 &thr[idx].grpname);
                thr[idx].status = THR_TEST_PASSED;
                break;

            default:
                status = VSTATUS_BAD;
        } 
    }

    thr[idx].status_value = status;

    /*
    ** Exit.
    */
    status = vs_thread_exit(&thr[idx].thread);
    return;
}

/*
** The following test the vs_thread_create() API.
*/
static Status_t
vs_thread_create_1a (void)
{
    static const char passed[] = "vs_thread_create:1:1.a PASSED";
    static const char failed[] = "vs_thread_create:1:1.a FAILED";
    static unsigned char name[] = "create1:1.a";
    Status_t status;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create ((Thread_t *) 0,
                                   name, 
                                   thr_test_startup,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }

    return status;
}

static Status_t
vs_thread_create_2a (void)
{
    static const char passed[] = "vs_thread_create:1:2.a PASSED";
    static const char failed[] = "vs_thread_create:1:2.a FAILED";
    Status_t status;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   (uint8_t *) 0, 
                                   thr_test_startup,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }

    return status;
}

static Status_t
vs_thread_create_2b (void)
{
    static const char passed[] = "vs_thread_create:1:2.b PASSED";
    static const char failed[] = "vs_thread_create:1:2.b FAILED";
    static unsigned char name[VS_NAME_MAX+5];
    Status_t       status;
    uint32_t       count;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    for (count = 0; count < VS_NAME_MAX; count++)
        name[count] = 'a';

    name[count++] = 'b';
    name[count++] = 'b';
    name[count] = (uint8_t) 0;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_test_startup,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_create_3a (void)
{
    static const char passed[] = "vs_thread_create:1:3.a PASSED";
    static const char failed[] = "vs_thread_create:1:3.a FAILED";
    static unsigned char name[] = "create_3a";
    Status_t       status;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   0,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_create_4a (void)
{
    static const char passed[] = "vs_thread_create:1:4.a PASSED";
    static const char failed[] = "vs_thread_create:1:4.a FAILED";
    static unsigned char name[] = "create_4a";
    Status_t       status;

    /*
    ** Create good parameters.
    */
    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_test_noarg_startup,
                                   0,
                                   0,
                                   thr[0].stack_size);                                         
    /* let the thread get going */
    generic_sleep((uint64_t)ONE_SECOND);

    if (status == VSTATUS_OK)
    {
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_create_5a (void)
{
    static const char passed[] = "vs_thread_create:1:5.a PASSED";
    static const char failed[] = "vs_thread_create:1:5.a FAILED";
    static unsigned char name[] = "create_5a";
    Status_t       status;

    /*
    ** Create good parameters.
    */

    thr[0].stack_size = MAX_STACK_SIZE;

    /*
    ** create a bad count arg combination
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argv[0] = thr[0].args[0];
    thr[0].argv[1] = (uint8_t *) 0;
    thr[0].argv[2] = (uint8_t *) 0;
    thr[0].argv[3] = (uint8_t *) 0;
    thr[0].argv[4] = (uint8_t *) 0;
    thr[0].argc = 5;
    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_test_startup,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_create_6a (void)
{
    static const char passed[] = "vs_thread_create:1:6.a PASSED";
    static const char failed[] = "vs_thread_create:1:6.a FAILED";
    static unsigned char name[] = "create_6a";
    Status_t       status;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack = (unsigned int *) 0;
    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   0,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}
static Status_t
vs_thread_create_7a (void)
{
    static const char passed[] = "vs_thread_create:1:7.a PASSED";
    static const char failed[] = "vs_thread_create:1:7.a FAILED";
    static unsigned char name[] = "create_7a";
    Status_t       status;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = 0;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   0,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}
static Status_t
vs_thread_create_8a (void)
{
    static const char passed[] = "vs_thread_create:1:8.a PASSED";
    static const char failed[] = "vs_thread_create:1:8.a FAILED";
    static unsigned char name[] = "create_8a";
    Status_t       status;

    thr[0].status = 0;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_test_startup,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status == VSTATUS_OK)
    {
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_create failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_create_8b (void)
{
    static const char passed[] = "vs_thread_create:1:8.b PASSED";
    static const char failed[] = "vs_thread_create:1:8.b FAILED";
    Status_t       status;

    /*
    ** make sure we wait for thread to get going.
    */
    generic_sleep((uint64_t)ONE_SECOND);
    
    if (thr[0].status == THR_TEST_PASSED)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_create failed; expected", THR_TEST_PASSED);
        IB_LOG_ERROR ("vs_thread_create failed; actual", thr[0].status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

void test_thread_create_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_create:1 TEST STARTED", (uint32_t) 0U);
    DOATEST (vs_thread_create_1a, total_passes, total_fails);
    DOATEST (vs_thread_create_2a, total_passes, total_fails);
    DOATEST (vs_thread_create_2b, total_passes, total_fails);
    DOATEST (vs_thread_create_3a, total_passes, total_fails);
    DOATEST (vs_thread_create_4a, total_passes, total_fails);
    DOATEST (vs_thread_create_5a, total_passes, total_fails);
#if defined(__LINUX__)
    DOATEST (vs_thread_create_6a, total_passes, total_fails);
    DOATEST (vs_thread_create_7a, total_passes, total_fails);
#endif
    DOATEST (vs_thread_create_8a, total_passes, total_fails);
    DOATEST (vs_thread_create_8b, total_passes, total_fails);
    IB_LOG_INFO ("vs_thread_create:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_create:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_create:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}

/*
** Thread null name test function
*/
static void thr_null_name_test(uint32_t argc, uint8_t *argv[])
{
    Status_t    status;
    uint32_t    idx;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);
    thr[idx].status = THR_TEST_FAILED;

    /*
    ** Get my thread name
    */
    status = vs_thread_name((Threadname_t *) 0);

    thr[idx].status = THR_TEST_PASSED;
    thr[idx].status_value = status;

    /*
    ** Exit.
    */
    (void) vs_thread_exit(&thr[idx].thread);
    return;
}

static Status_t
vs_thread_name_1a (void)
{
    static const char passed[] = "vs_thread_name:1:1.a PASSED";
    static const char failed[] = "vs_thread_name:1:1.a FAILED";
    static unsigned char name[] = "name_1a";
    Status_t       status;

    thr[0].status = 0;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_null_name_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].status == THR_TEST_PASSED && thr[0].status_value == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_name failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_name failed; actual", thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_name_2a (void)
{
    static const char passed[] = "vs_thread_name:1:2.a PASSED";
    static const char failed[] = "vs_thread_name:1:2.a FAILED";
    static unsigned char name[] = "name_2a";
    Status_t       status;

    thr[0].status = 0;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_name_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].status == THR_TEST_PASSED &&
        thr[0].status_value == VSTATUS_OK)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_name failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_name failed; actual", thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_name_2b (void)
{
    static const char passed[] = "vs_thread_name:1:2.b PASSED";
    static const char failed[] = "vs_thread_name:1:2.b FAILED";
    Status_t       status;

    if (thr[0].name != (Threadname_t) -1)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_name failed; expected", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; actual", (uint32_t)thr[0].name);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_name_3a (void)
{
    static const char passed[] = "vs_thread_name:1:2.a PASSED";
    static const char failed[] = "vs_thread_name:1:2.a FAILED";
    static unsigned char name[] = "name_3a";
    Status_t       status;
    int            count;
    int            count2;
    int            error;

    /*
    ** Start a bunch of threads, ignore errors will catch those later.
    */
    for (count = 0; count < MAX_THREADS; count++)
    {
        /*
        ** Create good parameters.
        */
        thr[count].status = 0;
        thr[count].name   = (Threadname_t) -1;
        sprintf((char *)thr[count].args[0], "%d", count);
        thr[count].argc = 1;
        thr[count].argv[0] = thr[count].args[0];

        thr[count].stack_size = MAX_STACK_SIZE;

        status = vs_thread_create (&thr[count].thread,
                                       name, 
                                       thr_name_test,
                                       thr[count].argc,
                                       thr[count].argv,
                                       thr[count].stack_size);                                         
        if (status != VSTATUS_OK)
        {
            IB_LOG_ERROR ("thread creation failed unexpectedly", status);
        }
    }

    /*
    ** Wait long enough for all threads to run
    */
    generic_sleep((uint64_t)ONE_SECOND);

    /*
    ** Validate all threads, quit on first error
    */
    for (count = 0, error = 0; !error && count < MAX_THREADS; count++)
    {
        if (thr[count].status != THR_TEST_PASSED ||
            thr[count].status_value  != VSTATUS_OK)
        {
            IB_LOG_ERROR ("thr.status", thr[count].status);
            IB_LOG_ERROR ("thr.status_value", thr[count].status_value);
            error = 1;
        }

        for (count2 = count+1; count2 < MAX_THREADS; count2++)
        {
           if (thr[count].name == thr[count2].name)
           {
               IB_LOG_ERROR ("Non-unique threadname", 
                             (uint32_t)thr[count].name);
               error = 2;
           }
        }
    }

    if (!error)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else if (error == 1)
    {
        IB_LOG_ERROR ("thread name not set", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; expected", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; actual", 0U);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    else
    {
        IB_LOG_ERROR ("thread name not unique", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; expected", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; actual", 0U);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

void test_thread_name_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_name:1 TEST STARTED", (uint32_t) 0U);
    DOATEST (vs_thread_name_1a, total_passes, total_fails);
    DOATEST (vs_thread_name_2a, total_passes, total_fails);
    DOATEST (vs_thread_name_2b, total_passes, total_fails);
    DOATEST (vs_thread_name_3a, total_passes, total_fails);
    IB_LOG_INFO ("vs_thread_name:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_name:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_name:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}

/*
** Thread null exit test function
*/
static void thr_null_exit_test(uint32_t argc, uint8_t *argv[])
{
    Status_t    status;
    uint32_t    idx;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);

    thr[idx].status = THR_TEST_PASSED;

    /*
    ** Exit.
    */
    thr[idx].exit_status = THR_TEST_FAILED;
    status = vs_thread_exit((Thread_t *)0);
    thr[idx].status_value = status;
    thr[idx].exit_status = THR_TEST_PASSED;

    /*
    ** Exit for real as well.
    */
    (void) vs_thread_exit(&thr[idx].thread);
}

static Status_t
vs_thread_exit_1a (void)
{
    static const char passed[] = "vs_thread_exit:1:1.a PASSED";
    static const char failed[] = "vs_thread_exit:1:1.a FAILED";
    static unsigned char name[] = "exit_1a";
    Status_t       status;

    thr[0].status = 0;
    thr[0].exit_status = THR_TEST_FAILED;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_null_exit_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].exit_status == THR_TEST_PASSED &&
        thr[0].status_value == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_exit failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_exit failed; actual", thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_exit_2a (void)
{
    static const char passed[] = "vs_thread_exit:1:2.a PASSED";
    static const char failed[] = "vs_thread_exit:1:2.a FAILED";
    static unsigned char name[] = "exit_2a";
    Status_t       status;

    thr[0].status = 0;
    thr[0].status_value = 0;
    thr[0].exit_status = THR_TEST_FAILED;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    /*
    ** Set argc == 2 to do exit test.
    */
    thr[0].argc = 2;
    thr[0].argv[1] = thr[0].args[0];

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_name_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].exit_status == THR_TEST_PASSED)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_exit failed; expected", THR_TEST_PASSED);
        IB_LOG_ERROR ("vs_thread_exit failed; actual", thr[0].exit_status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

void test_thread_exit_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_exit:1 TEST STARTED", (uint32_t) 0U);
    DOATEST (vs_thread_exit_1a, total_passes, total_fails);
    DOATEST (vs_thread_exit_2a, total_passes, total_fails);
    IB_LOG_INFO ("vs_thread_exit:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_exit:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_exit:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}


static Status_t
vs_thread_groupname_1a (void)
{
    static const char passed[] = "vs_thread_groupname:1:1.a PASSED";
    static const char failed[] = "vs_thread_groupname:1:1.a FAILED";
    static unsigned char name[] = "grouppname_1a";
    Status_t       status;

    thr[0].status       = THR_TEST_FAILED;
    thr[0].status_value = VSTATUS_BAD;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    /*
    ** Set argc == 2 for NULL Thread test variation.
    */
    thr[0].argc = 2;
    thr[0].argv[1] = thr[0].args[0];

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_groupname_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].status == THR_TEST_FAILED ||
        thr[0].status_value != VSTATUS_ILLPARM)
    {
        IB_LOG_ERROR ("vs_thread_groupname failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_groupname failed; actual", 
                      thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    else 
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    return status;
}

static Status_t
vs_thread_groupname_1b (void)
{
    static const char passed[] = "vs_thread_groupname:1:1.b PASSED";
    static const char failed[] = "vs_thread_groupname:1:1.b FAILED";
    static unsigned char name[] = "grouppname_1b";
    Status_t       status;

    thr[0].status       = THR_TEST_FAILED;
    thr[0].status_value = VSTATUS_BAD;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    /*
    ** Set argc == 3 for bad threadname test variation.
    */
    thr[0].argc = 3;
    thr[0].argc = 2;
    thr[0].argv[1] = thr[0].args[0];
    thr[0].argv[2] = thr[0].args[0];

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_groupname_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].status == THR_TEST_FAILED ||
        thr[0].status_value != VSTATUS_ILLPARM)
    {
        IB_LOG_ERROR ("vs_thread_groupname failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_groupname failed; actual", 
                      thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    else 
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    return status;
}

static Status_t
vs_thread_groupname_2a (void)
{
    static const char passed[] = "vs_thread_groupname:1:2.a PASSED";
    static const char failed[] = "vs_thread_groupname:1:2.a FAILED";
    static unsigned char name[] = "grouppname_2a";
    Status_t       status;

    thr[0].status       = THR_TEST_FAILED;
    thr[0].status_value = VSTATUS_BAD;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_groupname_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].status == THR_TEST_FAILED ||
        thr[0].status_value != VSTATUS_OK)
    {
        IB_LOG_ERROR ("vs_thread_groupname failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_groupname failed; actual", thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    else 
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    return status;
}

static Status_t
vs_thread_groupname_3a (void)
{
    static const char passed[] = "vs_thread_groupname:1:3.a PASSED";
    static const char failed[] = "vs_thread_groupname:1:3.a FAILED";
    static unsigned char name[] = "groupname_3a";
    Status_t       status;
    int            count;

    /*
    ** Start a bunch of threads, ignore errors will catch those later.
    */
    for (count = 0; count < MAX_THREADS; count++)
    {
        /*
        ** Create good parameters.
        */
        thr[count].status = THR_TEST_FAILED;
        thr[count].name   = (Threadname_t) -1;
        sprintf((char *)thr[count].args[0], "%d", count);
        thr[count].argc = 1;
        thr[count].argv[0] = thr[count].args[0];

        thr[count].stack_size = MAX_STACK_SIZE;

        status = vs_thread_create (&thr[count].thread,
                                       name, 
                                       thr_groupname_test,
                                       thr[count].argc,
                                       thr[count].argv,
                                       thr[count].stack_size);                                         
        if (status != VSTATUS_OK)
        {
            IB_LOG_ERROR ("thread creation failed unexpectedly", status);
        }
    }

    /*
    ** Wait long enough for all threads to run
    */
    generic_sleep((uint64_t)ONE_SECOND);

    /*
    ** Validate all threads, quit on first error
    */
    for (count = 0;  count < MAX_THREADS; count++)
    {
        if (thr[count].status != THR_TEST_PASSED ||
            thr[count].status_value  != VSTATUS_OK)
        {
            IB_LOG_ERROR ("thr.status", thr[count].status);
            IB_LOG_ERROR ("thr.status_value", thr[count].status_value);
            break;
        }
    }

    if (count >= MAX_THREADS)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_groupname failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_groupname failed; actual", thr[count].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_groupname_3b (void)
{
    static const char passed[] = "vs_thread_groupname:1:3.b PASSED";
    static const char failed[] = "vs_thread_groupname:1:3.b FAILED";
    Status_t       status;
    int            count;
    int            count2;
    int            error;

    /*
    ** Validate all threads belong to the same group
    */
    for (count = 0, error = 0; !error && count < MAX_THREADS; count++)
    {
        for (count2 = count+1; count2 < MAX_THREADS; count2++)
        {
           if (thr[count].grpname != thr[count2].grpname)
           {
               error = 2;
           }
        }
    }

    if (!error)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("inconsistent thread groupname among threads", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; expected", 0U);
        IB_LOG_ERROR ("vs_thread_name failed; actual", 0U);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

void test_thread_groupname_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_groupname:1 TEST STARTED", (uint32_t) 0U);
    DOATEST (vs_thread_groupname_1a, total_passes, total_fails);
    DOATEST (vs_thread_groupname_1b, total_passes, total_fails);
    DOATEST (vs_thread_groupname_2a, total_passes, total_fails);
    DOATEST (vs_thread_groupname_3a, total_passes, total_fails);
    DOATEST (vs_thread_groupname_3b, total_passes, total_fails);
    IB_LOG_INFO ("vs_thread_groupname:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_groupname:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_groupname:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}

/*
** Thread sleep test
*/
static void thr_sleep_test(uint32_t argc, uint8_t *argv[])
{
    Status_t    status = VSTATUS_OK;
    uint32_t    idx;
    uint64_t    sleep_time;
    uint64_t    start_time;
    uint64_t    stop_time;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);

    thr[idx].status = THR_TEST_FAILED;
    thr[idx].sleep_status = THR_SLEEPING;
    thr[idx].act_sleep_time   = 0;
    
     /*
     ** Sleep, use argc to dicate which sleep call to use for special
     ** test cases.
     */ 
    switch (argc)
    {
        case 1 :
            sleep_time = thr[idx].sleep_time;
            (void) vs_time_get(&start_time);
            vs_thread_sleep(sleep_time);
            (void) vs_time_get(&stop_time);
            thr[idx].status = THR_TEST_PASSED;
            thr[idx].sleep_status       = THR_NOT_SLEEPING;
            thr[idx].sleep_time         = sleep_time;
            thr[idx].act_sleep_time     = stop_time - start_time;
            break;

        case 2 :
            sleep_time = ONE_SECOND;
            (void) vs_time_get(&start_time);
            vs_thread_sleep((uint64_t) 0);
            (void) vs_time_get(&stop_time);
            thr[idx].status = THR_TEST_PASSED;
            thr[idx].sleep_status       = THR_NOT_SLEEPING;
            thr[idx].sleep_time         = sleep_time;
            thr[idx].act_sleep_time     = stop_time - start_time;
            break;

        default:
            status = VSTATUS_BAD;
            break;
    } 

    thr[idx].status_value = status;

    /*
    ** Exit.
    */
    status = vs_thread_exit(&thr[idx].thread);
    return;
}

#if 0
static Status_t
vs_thread_sleep_1a (void)
{
    static const char passed[] = "vs_thread_sleep:1:1.a PASSED";
    static const char failed[] = "vs_thread_sleep:1:1.a FAILED";
    static unsigned char name[] = "sleep_1a";
    Status_t       status;

    thr[0].status       = THR_TEST_FAILED;
    thr[0].sleep_status = THR_NOT_SLEEPING;
    thr[0].sleep_time   = ONE_SECOND;
    thr[0].status_value = VSTATUS_BAD;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    /*
    ** Set argc == 2 to indicate bad pointer test
    */
    thr[0].argc = 2;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_sleep_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    generic_sleep((uint64_t)ONE_SECOND);

    if (thr[0].status == THR_TEST_PASSED && thr[0].status_value == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_sleep failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_sleep failed; actual", thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}
#endif

static Status_t
vs_thread_sleep_main (void)
{
    Status_t       status;
    static unsigned char name[] = "sleep";

    thr[0].status       = THR_TEST_FAILED;
    thr[0].sleep_status = THR_NOT_SLEEPING;
    thr[0].status_value = VSTATUS_BAD;

    /*
    ** Create good parameters.
    */
    sprintf((char *)thr[0].args[0], "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name, 
                                   thr_sleep_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);                                         
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    return status;
}

static Status_t
vs_thread_sleep_2a (void)
{
    static const char passed[] = "vs_thread_sleep:1:2.a PASSED";
    static const char failed[] = "vs_thread_sleep:1:2.a FAILED";
    Status_t       status;

    thr[0].sleep_time   = ONE_SECOND;
    thr[0].act_sleep_time   = 0;

    (void) vs_thread_sleep_main();

    generic_sleep((uint64_t)TWO_SECONDS);

    if (thr[0].status == THR_TEST_PASSED && thr[0].status_value == VSTATUS_OK)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_sleep failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_sleep failed; actual", thr[0].status_value);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_sleep_3a (void)
{
    static const char passed[] = "vs_thread_sleep:1:3.a PASSED";
    static const char failed[] = "vs_thread_sleep:1:3.a FAILED";
    Status_t       status;

    thr[0].sleep_time   = ONE_SECOND;
    thr[0].act_sleep_time   = 0;

    (void) vs_thread_sleep_main();

    generic_sleep((uint64_t)FIVE_SECONDS);

    if (thr[0].act_sleep_time >= MIN_ONE_SECOND && 
        thr[0].act_sleep_time <= MAX_ONE_SECOND)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_sleep time error", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_sleep failed; expected", ONE_SECOND);
        IB_LOG_ERROR ("vs_thread_sleep failed; actual", (uint32_t)thr[0].act_sleep_time);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_sleep_3b (void)
{
    static const char passed[] = "vs_thread_sleep:1:3.b PASSED";
    static const char failed[] = "vs_thread_sleep:1:3.b FAILED";
    Status_t       status;

    thr[0].sleep_time   = TWO_SECONDS;
    thr[0].act_sleep_time   = 0;

    (void) vs_thread_sleep_main();

    generic_sleep((uint64_t)FIVE_SECONDS);

    if (thr[0].act_sleep_time >= MIN_TWO_SECONDS && 
        thr[0].act_sleep_time <= MAX_TWO_SECONDS)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_sleep time error", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_sleep failed; expected", TWO_SECONDS);
        IB_LOG_ERROR ("vs_thread_sleep failed; actual", (uint32_t)thr[0].act_sleep_time);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_sleep_3c (void)
{
    static const char passed[] = "vs_thread_sleep:1:3.c PASSED";
    static const char failed[] = "vs_thread_sleep:1:3.c FAILED";
    Status_t       status;

    thr[0].sleep_time   = FIVE_SECONDS;
    thr[0].act_sleep_time   = 0;

    (void) vs_thread_sleep_main();

    generic_sleep(FIVE_SECONDS + TWO_SECONDS);

    if (thr[0].act_sleep_time >= MIN_FIVE_SECONDS && 
        thr[0].act_sleep_time <= MAX_FIVE_SECONDS)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_sleep time error", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_sleep failed; expected", FIVE_SECONDS);
        IB_LOG_ERROR ("vs_thread_sleep failed; actual", (uint32_t)thr[0].act_sleep_time);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_sleep_3d (void)
{
    static const char passed[] = "vs_thread_sleep:1:3.d PASSED";
    static const char failed[] = "vs_thread_sleep:1:3.d FAILED";
    Status_t       status;

    thr[0].sleep_time   = HALF_SECOND;
    thr[0].act_sleep_time   = 0;

    (void) vs_thread_sleep_main();

    generic_sleep(FIVE_SECONDS + HALF_SECOND);

    if (thr[0].act_sleep_time >= MIN_HALF_SECOND && 
        thr[0].act_sleep_time <= MAX_HALF_SECOND)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_sleep time error", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_sleep failed; expected", HALF_SECOND);
        IB_LOG_ERROR ("vs_thread_sleep failed; actual", (uint32_t)thr[0].act_sleep_time);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

void test_thread_sleep_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_sleep:1 TEST STARTED", (uint32_t) 0U);
#if 0
    DOATEST (vs_thread_sleep_1a, total_passes, total_fails);
#endif
    DOATEST (vs_thread_sleep_2a, total_passes, total_fails);
    DOATEST (vs_thread_sleep_3a, total_passes, total_fails);
    DOATEST (vs_thread_sleep_3b, total_passes, total_fails);
    DOATEST (vs_thread_sleep_3c, total_passes, total_fails);
    DOATEST (vs_thread_sleep_3d, total_passes, total_fails);
#if 0
    DOATEST (vs_thread_sleep_4a, total_passes, total_fails);
    DOATEST (vs_thread_sleep_4b, total_passes, total_fails);
#endif
    IB_LOG_INFO ("vs_thread_sleep:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_sleep:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_sleep:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}

static Status_t
vs_thread_kill_1a (void)
{
    static const char passed[] = "vs_thread_kill:1:1.a PASSED";
    static const char failed[] = "vs_thread_kill:1:1.a FAILED";
    Status_t       status;

    status = vs_thread_kill((Thread_t *) 0);

    if (status == VSTATUS_ILLPARM)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_kill failed; expected", VSTATUS_ILLPARM);
        IB_LOG_ERROR ("vs_thread_kill failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_kill_2a (void)
{
    static const char passed[] = "vs_thread_kill:1:2.a PASSED";
    static const char failed[] = "vs_thread_kill:1:2.a FAILED";
    Status_t       status;

    thr[0].sleep_time   = FIVE_SECONDS;
    thr[0].act_sleep_time   = 0;

    (void) vs_thread_sleep_main();

    generic_sleep((uint64_t)TWO_SECONDS);

    status = vs_thread_kill(&thr[0].thread);

    if (status == VSTATUS_OK)
    {
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_kill failed; expected", VSTATUS_OK);
        IB_LOG_ERROR ("vs_thread_kill failed; actual", status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_kill_2b (void)
{
    static const char passed[] = "vs_thread_kill:1:2.b PASSED";
    static const char failed[] = "vs_thread_kill:1:2.b FAILED";
    Status_t       status;


    generic_sleep((uint64_t)TEN_SECONDS);

    if (thr[0].sleep_status == THR_SLEEPING)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else 
    {
        IB_LOG_ERROR ("vs_thread_kill failed; expected", THR_SLEEPING);
        IB_LOG_ERROR ("vs_thread_kill failed; actual", thr[0].sleep_status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

void test_thread_kill_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_kill:1 TEST STARTED", (uint32_t) 0U);
    DOATEST (vs_thread_kill_1a, total_passes, total_fails);
    DOATEST (vs_thread_kill_2a, total_passes, total_fails);
    DOATEST (vs_thread_kill_2b, total_passes, total_fails);
    IB_LOG_INFO ("vs_thread_kill:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_kill:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_kill:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}

/*
** Thread join test function
*/
static void
thr_join_test(uint32_t argc, uint8_t *argv[])
{
    uint32_t    idx;

    /* Argv[0] == thread test index */
    if (argc < 1)
    {
        IB_LOG_ERROR ("Internal test errror, no arguments", 0);
        return;
    }

    if (!argv[0])
    {
        IB_LOG_ERROR ("Internal test errror, thread value not passed index", 0);
        return;
    }
    idx = my_atou(argv[0]);

    thr[idx].exit_status = THR_TEST_PASSED;

    (void) vs_thread_exit(&thr[idx].thread);
}

static Status_t
vs_thread_join_1a (void)
{
    static const char passed[] = "vs_thread_join:1:1.a PASSED";
    static const char failed[] = "vs_thread_join:1:1.a FAILED";
    static unsigned char name[] = "join_1a";
    Status_t       status;

    thr[0].status = 0;
    thr[0].exit_status = THR_TEST_FAILED;
    thr[0].name   = (Threadname_t) -1;

    /*
    ** Create good parameters.
    */
    snprintf((char *)thr[0].args[0], sizeof(int), "%d", 0);
    thr[0].argc = 1;
    thr[0].argv[0] = thr[0].args[0];

    thr[0].stack_size = MAX_STACK_SIZE;

    status = vs_thread_create (&thr[0].thread,
                                   name,
                                   thr_join_test,
                                   thr[0].argc,
                                   thr[0].argv,
                                   thr[0].stack_size);
    if (status != VSTATUS_OK)
    {
        IB_LOG_ERROR ("thread creation failed unexpectedly", status);
    }

    // wait for the thread to complete
    status = vs_thread_join(&thr[0].thread, NULL);
    if (status)
    {
        thr[0].exit_status = THR_TEST_FAILED;
        IB_LOG_ERROR0("Failed to join compression thread");
    }

    if (thr[0].exit_status == THR_TEST_PASSED)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        IB_LOG_ERROR ("vs_thread_join failed; expected", THR_TEST_FAILED);
        IB_LOG_ERROR ("vs_thread_join failed; actual", thr[0].exit_status);
        IB_LOG_ERROR (failed, (uint32_t) 0U);
        status = VSTATUS_BAD;
    }
    return status;
}

static Status_t
vs_thread_join_2a (void)
{
    static const char passed[] = "vs_thread_join:1:2.a PASSED";
    static const char failed[] = "vs_thread_join:1:2.a FAILED";
    static unsigned char name[VS_NAME_MAX]= "";
    int index;
    Status_t status;
    int pass_counter = 0;

    /* create 2 threads to test thread_join */
    for (index = 0; index < NUM_JOIN_THREADS; index++)
    {
        thr[index].status = 0;
        thr[index].status_value = 0;
        thr[index].exit_status = THR_TEST_FAILED;
        thr[index].name   = (Threadname_t) -1;

        snprintf((char *)thr[index].args[0], sizeof(int), "%d", index);
        thr[index].argc = 1;
        thr[index].argv[index] = thr[index].args[0];

        thr[index].stack_size = MAX_STACK_SIZE;

        snprintf((char *)name, VS_NAME_MAX, "join_2a_%d", index);

        status = vs_thread_create (&thr[index].thread,
                                   name,
                                   thr_join_test,
                                   thr[index].argc,
                                   thr[index].argv,
                                   thr[index].stack_size);

        if (status != VSTATUS_OK)
        {
            IB_LOG_ERROR ("thread creation failed unexpectedly", status);
        }
    }

    // wait for all the threads to complete
    for (index = 0; index < NUM_JOIN_THREADS; index++)
    {
        status = vs_thread_join(&thr[index].thread, NULL);
        if (status)
        {
            thr[index].exit_status = THR_TEST_FAILED;
            IB_LOG_ERROR0("Failed to join compression thread");
        }

        if (thr[index].exit_status == THR_TEST_PASSED)
        {
            pass_counter++;

        }
    }

    if( pass_counter == NUM_JOIN_THREADS)
    {
        status = VSTATUS_OK;
        IB_LOG_INFO (passed, (uint32_t) 0U);
    }
    else
    {
        status = VSTATUS_BAD;
        IB_LOG_ERROR (failed, (uint32_t) 0U);
    }
    return status;
}

void test_thread_join_1(void)
{
    uint32_t total_passes = (uint32_t) 0U;
    uint32_t total_fails = (uint32_t) 0U;

    IB_LOG_INFO ("vs_thread_join:1 TEST STARTED", (uint32_t) 0U);
    DOATEST (vs_thread_join_1a, total_passes, total_fails);
    DOATEST (vs_thread_join_2a, total_passes, total_fails);
    IB_LOG_INFO ("vs_thread_join:1 TOTAL PASSED", total_passes);
    IB_LOG_INFO ("vs_thread_join:1 TOTAL FAILED", total_fails);
    IB_LOG_INFO ("vs_thread_join:1 TEST COMPLETE", (uint32_t) 0U);

    return;
}
