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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "omgt_oob_net.h"

void omgt_oob_init_queue(net_queue_t *q)
{
	q->head = q->tail = NULL;
}

void omgt_oob_enqueue_net_blob(net_queue_t *q, struct net_blob *blob)
{
	if (q->head == NULL) {
		q->head = blob;
		q->tail = blob;
		blob->next = NULL;
	} else {
		q->tail->next = blob;
		q->tail = blob;
		blob->next = NULL;
	}
}

struct net_blob *omgt_oob_dequeue_net_blob(net_queue_t *q)
{
	struct net_blob *retval = NULL;

	if (q->head) {
		retval = q->head;
		q->head = q->head->next;
		/* Note: q->tail won't be NULL once q->head advances to NULL, */
		/* but omgt_oob_enqueue_net_blob only tests q->head to determine emptiness */
	}

	return retval;
}

struct net_blob *omgt_oob_peek_net_blob(net_queue_t *q)
{
	return q->head;
}

int omgt_oob_queue_empty(net_queue_t *q)
{
	return q->head == NULL;
}
