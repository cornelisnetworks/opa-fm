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

/************************************************************************
* 
* FILE NAME
*   netblob.c
*
* DESCRIPTION
*   Libnet buffer handling
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   02/12/2000      Initial file for checkin to source control
* jrw   11/27/2001      Added prologues and changed malloc/free to 
*                       vs_pool_alloc/vs_pool_free
* jrw   12/04/2001      Changes from code review
*
***********************************************************************/

#include <stddef.h>
#include "libnet.h"
#include "vs_g.h"
#include "cs_log.h"
#undef  LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_FE_MOD_ID

extern Pool_t fe_pool;                /* Memory pool for FE    */

NetBlob *fe_net_new_blob(int len) {
    NetBlob     *blob   = NULL;
    uint32_t    rc      = NET_OK;

    IB_ENTER(__func__,len,0,0,0);

    /* Allocate memory for blob from the mem pool   */
    rc = vs_pool_alloc(&fe_pool,sizeof(NetBlob), (void *)&blob);

    if (rc != VSTATUS_OK) {
        IB_LOG_ERROR0("Unable to allocate memory for a fe_net_new_blob");
        IB_EXIT(__func__,rc);
        return(NULL);
    }

    if (len) {
        /* Allocate memory for the data from the mem pool   */
        rc = vs_pool_alloc(&fe_pool,len,(void *)&blob->data);
        
        if (rc != VSTATUS_OK) {
            /* Error allocating data, so free the entire blob   */
            vs_pool_free(&fe_pool,blob);
            IB_EXIT(__func__,rc);
            return(NULL);
        }
    }
    else {
        blob->data = NULL;
    }

    blob->len = len;
    blob->bytesLeft = len;
    blob->curPtr = blob->data;
    blob->next = NULL;

    IB_EXIT(__func__,rc);
    return blob;
}

void fe_net_free_blob(NetBlob *blob) {
    IB_ENTER(__func__,blob,0,0,0);

    /* If the data exists, free it  */
    if (blob->data) {
        /* Free the data    */
        vs_pool_free(&fe_pool,blob->data);
    }

    /* Free the blob    */
    vs_pool_free(&fe_pool,blob);

    IB_EXIT(__func__,0);
}

void fe_net_free_buf(char *buf) {
    IB_ENTER(__func__,buf,0,0,0);

    if (buf) {
        /* Free the buffer */
        vs_pool_free(&fe_pool,buf);
    }

    IB_EXIT(__func__,0);
}

int fe_net_adjust_blob_curptr(NetBlob *blob, int bytesSent) {
    int rc = NET_OK;
    IB_ENTER(__func__,blob,bytesSent,0,0);
    
    /* Do some error checking.  If anything fails here, we will disconnect
       the connection gracefully    */
    if ((!blob) || (!blob->curPtr) || (blob->bytesLeft < bytesSent)) {
        rc = NET_FAILED;
        IB_LOG_ERROR0("FATAL: error with the data buffer");
    }
    else {
        /* Error checking succeeded.  Adjust the ptr.   */
        blob->bytesLeft -= bytesSent;
        blob->curPtr += bytesSent;
    }

    IB_EXIT(__func__,rc);
    return rc;
}
