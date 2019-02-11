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
*   netqueue.c
*
* DESCRIPTION
*   Libnet queue handling
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   02/12/2000      Initial file for checkin to source control
* jrw   11/28/2001      Added prologues, comments, logging.  Removed 
*
***********************************************************************/

#include "libnet.h"
#include "cs_g.h"
#include "cs_log.h"
#undef  LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_FE_MOD_ID

void fe_net_init_queue(NetQueue *q) {
    IB_ENTER(__func__,q,0,0,0);

    q->head = q->tail = NULL;

    IB_EXIT(__func__,0);
}

void fe_net_enqueue_blob(NetQueue *q, NetBlob *blob) {
    IB_ENTER(__func__,q,blob,0,0);

    if (q->head == NULL) {
        q->head = blob;
        q->tail = blob;
        blob->next = NULL;
    }
    else {
        q->tail->next = blob;
        q->tail = blob;
        blob->next = NULL;
    }

    IB_EXIT(__func__,0);
}

NetBlob *fe_net_dequeue_blob(NetQueue *q) {
    NetBlob *retval = NULL;

    IB_ENTER(__func__,q,0,0,0);

    if (q->head) {
        retval = q->head;
        q->head = q->head->next;
    /* Note: q->tail won't be NULL once q->head advances to NULL, */
    /* but fe_net_enqueue_blob only tests q->head to determine emptiness */
    }

    IB_EXIT(__func__,0);
    return retval;
}

NetBlob *fe_net_peek_blob(NetQueue *q) {
    IB_ENTER(__func__,q,0,0,0);

    IB_EXIT(__func__,0);
    return q->head;
}

int fe_net_queue_empty(NetQueue *q) {
    IB_ENTER(__func__,q,0,0,0);

    IB_EXIT(__func__,0);
    return q->head == NULL;
}
