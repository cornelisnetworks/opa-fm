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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _OMGT_OOB_NET_H_
#define _OMGT_OOB_NET_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include "ibyteswap.h"

#include "ib_utils_openib.h"
#include "omgt_oob_net_blob.h"
#include "omgt_oob_net_queue.h"

/**************************************************************************
* 		struct net_connection Structures
**************************************************************************/

/*
 * A net_connection encapsulates a logical network connection between
 * client and server.  More than one net_connection can exist between
 * two given machines.  Here we store the blobs queued up for send but
 * not yet sent, and the blobs received and ready to be processed.
 */
struct net_connection {
	int                   sock;
	net_queue_t           send_queue;
	net_queue_t           recv_queue;
	struct net_blob       *blob_in_progress;
	int                   err;     /* to indicate err on this connection */
	struct sockaddr_in    v4_addr; /* socket IPv4 address of peer */
	struct sockaddr_in6   v6_addr; /* socket IPv6 address of peer */
	int                   ipv6;    /* specify ipv6 or ipv4 as domain family */
};

/* Error defines */
#define INVALID_SOCKET      -1
#define SOCKET_ERROR        -1

/* Net connection functions */
FSTATUS omgt_oob_net_connect(struct omgt_port *port);
FSTATUS omgt_oob_net_disconnect(struct omgt_port *port);
void omgt_oob_net_process(struct omgt_port *port, int msec_to_wait, int blocking);
FSTATUS omgt_oob_net_send(struct omgt_port *port, char *data, int len);
void omgt_oob_net_get_next_message(struct net_connection *conn, uint8_t **data, int *len);

/* Connection functions */
FSTATUS omgt_oob_send_packet(struct omgt_port *port, uint8_t *buf, size_t len);
FSTATUS omgt_oob_receive_response(struct omgt_port *port, uint8_t **data, uint32_t *len);
FSTATUS omgt_oob_disconnect(struct omgt_port *port);

#endif /* _OMGT_OOB_NET_H_ */
