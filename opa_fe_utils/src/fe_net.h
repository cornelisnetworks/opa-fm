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

#ifndef _FE_NET_H_
#define _FE_NET_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include "ibyteswap.h"
#include "fe_net_blob.h"
#include "fe_net_queue.h"

/* External variables */
extern int g_verbose;
extern FILE *dbg_file;

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
	struct net_connection *prev;   /* to maintain doubly-linked list of connections */
	struct net_connection *next;
};

/* Error defines */
#define INVALID_ID          -1
#define NET_OK              0
#define NET_FAILED          1
#define NET_NO_ERROR        0
#define NET_NOT_INITIALIZED 1
#define NET_NOT_CONNECTED   2
#define NET_INTERNAL_ERR    3
#define NET_NO_MEMORY       4
#define NET_INVALID_HOST    5
#define NET_CANNOT_CONNECT  6
#define OK                  0
#define ERR                 1
#define INVALID_SOCKET      -1
#define SOCKET_ERROR        -1
#define WSAEWOULDBLOCK      EWOULDBLOCK

/* Debug defines */
#define FE_OOB_DBG_FILE_SYSLOG ((FILE *)-1)
void fe_oob_set_dbg(FILE *file); /* syslog(LOG_INFO|LOG_DEBUG, ... */

#ifndef NET_PRINT
#define NET_PRINT(format, args...) \
	do { \
		if (fe_oob_dbg_file) { \
			if (dbg_file == FE_OOB_DBG_FILE_SYSLOG) { \
				syslog(LOG_INFO, "osf_fe_utils: " format, ##args); \
			} else { \
				fflush(fe_oob_dbg_file); \
				fprintf(fe_oob_dbg_file, "osf_fe_utils: " format, ##args); \
			} \
		} \
	} while(0)
#endif

/* Net connection functions */
int fe_oob_net_connect(char *hostname, uint16_t port, struct net_connection **conn);
int fe_oob_net_disconnect(struct net_connection *conn);
struct net_connection *fe_oob_net_sleep(int msec_to_wait, int blocking);
struct net_connection *fe_oob_net_process(int msec_to_wait, int blocking);
int fe_oob_net_send(struct net_connection *conn, char *data, int len, int *err);
void fe_oob_net_get_next_message(struct net_connection *conn, uint8_t **data, int *len, int *err);
void fe_oob_add_to_connection_list(struct net_connection *conn);
void fe_oob_remove_from_connection_list(struct net_connection *conn);

#endif /* _FE_NET_H_ */
