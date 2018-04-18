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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <stddef.h>
#include <iba/ibt.h>
#include <iba/ipublic.h>
#include "omgt_oob_net.h"

/*
 * Return a new net_blob object with an allocated buffer of size len bytes.
 */
struct net_blob *omgt_oob_new_net_blob(size_t len)
{
	struct net_blob *blob = (struct net_blob *)malloc(sizeof(struct net_blob));
	if (blob == NULL) {
		return NULL;
	}
	if (len) {
		blob->data = malloc(len);
		if (blob->data == NULL) {
			free(blob);
			return NULL;
		}
	} else {
		blob->data = NULL;
	}
	blob->len = len;
	blob->bytes_left = len;
	blob->cur_ptr = blob->data;
	blob->next = NULL;
	return blob;
}

void omgt_oob_free_net_blob(struct net_blob *blob)
{
	free(blob->data);
	free(blob);
}

void omgt_oob_free_net_buf(char *buf)
{
	free(buf);
}

void omgt_oob_adjust_blob_cur_ptr(struct net_blob *blob, int bytes_sent)
{
	ASSERT(blob);
	ASSERT(blob->cur_ptr);
	ASSERT(blob->cur_ptr >= blob->data);
	ASSERT(blob->bytes_left <= blob->len);
	ASSERT(bytes_sent <= blob->bytes_left);

	blob->bytes_left -= bytes_sent;
	blob->cur_ptr += bytes_sent;
}
