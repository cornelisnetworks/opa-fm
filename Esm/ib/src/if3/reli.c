/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************/
/*                                                                      */
/* FILE NAME                                                            */
/*    reli.c                                                            */
/*                                                                      */
/* DESCRIPTION                                                          */
/*    Code for Interface 3 multi-mad transport Library                  */
/*                                                                      */
/*                                                                      */
/* DEPENDENCIES                                                         */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/


#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "if3.h"
#include "if3_def.h"


#include <stdio.h>
#if defined( __LINUX__)
    #include <signal.h>
    #include <unistd.h>
    #include <pthread.h>
    #include <string.h>
#endif

#if defined( __VXWORKS__)
    #include <string.h>
#endif

Status_t if3_addr_swizzle(Mai_t *mad);

#define	Min(X,Y)		((X) < (Y)) ? (X) : (Y)

/* global debug RMPP flag for VFI */
uint32_t if3DebugRmpp=0;  // default to off for ESM




/*
 * FUNCTION
 *      MngrWaitHandle
 *
 * DESCRIPTION
 *      This function wait for message to be posted to any of  the handles 
 *      specified in argument. It returns the index of the lowest handle
 *      that has a message on it, if a message is received before timeout.
 *      This index is posted to the location the user passed in, pointed
 *      by the location pfirst.
 *
 * CALLED BY
 *
 * CALLS
 *
 *
 * INPUTS
 *      ha      Array of handles returned from mai_open
 *      count   Number of handles in array.
 *      timout  How long to wait. 0 means return immediately
 *      pfirst  Where to post the index handle with message on in 
 *      maip    Where to post the MAD recved on the handle
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_TIMEOUT
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      PAW     01/21/01        Initial entry
 */

#define WAIT_HANDLE_TO  (500000)  /* 0.5 ms */

Status_t MngrWaitHandle(IBhandle_t *ha, uint32_t count, 
                        uint64_t timeout, uint32_t *pfirst, Mai_t *maip)

{

    uint64_t          timenow;
    uint64_t          wakeup;
    uint32_t               rc,i;

    IB_ENTER(__func__, ha, count, timeout, 0);

    if (ha    == NULL || pfirst == NULL ||
        count <=0     || maip   == NULL) {
        rc = VSTATUS_ILLPARM;
        goto done;
    }


    /* Update the absolute timeout value to wait for.			*/

    if (timeout == MAI_RECV_NOWAIT) {
        wakeup  = 0;
    } else {
        (void)vs_time_get(&timenow);
        wakeup = timenow + timeout;
    }

    mai_wait_retry:  

    /* 
     * The handles are valid. Now Try and see if there is data
     * already waiting on any.
     */

    for (i=0;i<count;i++) {

        rc = mai_recv(ha[i],maip,MAI_RECV_NOWAIT);

        if (rc == VSTATUS_OK) {
            *pfirst = i;
            goto done;
        }

        if (rc != VSTATUS_TIMEOUT) {
            /* An error has occurred  */
            IB_LOG_ERROR("recv err on index ",i);
            goto done;
        }
    }



    /* 
     * If we get here then the handles did not have data on them. So let's 
     * proceed to the next stage of waiting.
     */

    /* See if we have reached (or exceeded timeout)	  	       */
    if (wakeup == 0) {
        rc =  VSTATUS_TIMEOUT;
        goto done;
    } else {
        (void) vs_time_get(&timenow);
        if (timenow > wakeup) {
            rc = VSTATUS_TIMEOUT;
            goto done;
        }
    }

    /* Increment waiters count                                            */

    rc = mai_recv(ha[0],maip, timeout);

    if (rc == VSTATUS_OK) {
        *pfirst = 0;
        goto done;
    }

    goto mai_wait_retry;     /* And try it again			*/

    done:
    IB_EXIT(__func__,rc);
    return(rc);
}




/*
 * if3_rec 
 *   perform recv operation on handle. It also monitors the 
 * callback function for new data and calls if need.
 *
 * INPUTS
 *   mi        - manager handle
 *   madp      - input mad
 *   timeout   - how long to wait
 *
 * RETURNS
 *    VSTATUS_OK          - the call was successful.
 *    VSTATUS_BAD         - the method is not understood.
 */



Status_t 
if3_recv(ManagerInfo_t *mi, Mai_t *madp, uint64_t timeout)
{
    IBhandle_t fda[2];
    uint32_t        rc;
    uint32_t        first;

    IB_ENTER(__func__,mi,madp,timeout,0);

    if (mi->cb_flag) {
        fda[0]=mi->cb_fd;
        fda[1]=mi->fds;

        wait:

        rc = MngrWaitHandle(fda,2,timeout,&first,madp);

        if (rc == VSTATUS_OK) {

            if (first==0) {
                /*we have a message on the call back*/
                mi->cb_func(mi->cb_fd,madp,mi->cb_ctx);
                goto wait;
            } else {
                IB_EXIT(__func__,rc);
                return rc;          
            }     
        }

        /*Not OK just return watever was passed in*/
        IB_EXIT(__func__,rc);
        return rc;    
    }

    /*no async.. just do a normal mai recv*/

    rc = mai_recv(mi->fds,madp,timeout);

    IB_EXIT(__func__,rc);
    return rc; 

}

int if3RmppDebugGet(void)
{
    return if3DebugRmpp;
}

void if3RmppDebugOn(void)
{
	IB_LOG_INFINI_INFO0("Turning ON RmppDebug");
    if3DebugRmpp = 1;
}

void if3RmppDebugOff(void)
{
	IB_LOG_INFINI_INFO0("Turning OFF RmppDebug");
    if3DebugRmpp = 0;
}

