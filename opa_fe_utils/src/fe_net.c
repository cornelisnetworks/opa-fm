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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include "imath.h"
#include "fe_net.h"
#include "fe_ssl.h"

#define CONNECTION_BACKLOG 10

extern int g_verbose;
extern char* g_sslParmsFile;
extern sslParametersData_t g_sslParmsData;

static struct net_connection *g_connections_;
static void * g_sslContext = NULL;
static void * g_sslSession;

#define SET_ERROR(x,y) if (x) { *(x)=(y); }
#define NET_MAGIC 0x31E0CC01
#define FE_IS_SSL_ENABLED()    (g_sslParmsFile && g_sslParmsData.sslSecurityEnabled)

static int fe_oob_read_from_socket(struct net_connection *conn);
static int fe_oob_write_to_socket(struct net_connection *conn);
static struct net_connection *fe_oob_new_connection();
static void fe_oob_handle_read_write_error(struct net_connection *conn);
static int fe_oob_print_addrinfo(char *hostname);

FILE *fe_oob_dbg_file = NULL;

/*
 * Set the debug file for NET_PRINT.
 */
void fe_oob_set_dbg(FILE *file)
{
	fe_oob_dbg_file = file;
}

/*
 * Establish a connection to the given host. hostname can either be
 * the name of a host or a dotted IP address.
 */
int fe_oob_net_connect(char *hostname, uint16_t port, struct net_connection **conn)
{
	struct sockaddr_in v4_addr;
	struct sockaddr_in6 v6_addr;
	struct in_addr ipv4addr;
	struct in6_addr ipv6addr;
	struct hostent *hp = NULL;
	int inaddr = 0;
	int ipv6 = 0;

	/*
	 * First resolve the hostname and fill in the sockaddr
	 */
	inaddr = inet_pton(AF_INET6, hostname, &ipv6addr);

	if ((ipv6 = (inaddr == 1) ? 1 : 0)) {
		memset((void *)&v6_addr, 0, sizeof(v6_addr));
		v6_addr.sin6_family = AF_INET6;
		v6_addr.sin6_port = htons((short)port);
		memcpy((void *)&v6_addr.sin6_addr, (void *)&ipv6addr, sizeof(ipv6addr));

		if (g_verbose) {
			fe_oob_print_addrinfo(hostname);
		}
	} else {
		memset((void *)&v4_addr, 0, sizeof(v4_addr));
		v4_addr.sin_family = AF_INET;
		v4_addr.sin_port = htons((short)port);
		inaddr = inet_pton(AF_INET, hostname, &ipv4addr);

		if (inaddr == 1) {
			memcpy((void *)&v4_addr.sin_addr, (void *)&ipv4addr, sizeof(ipv4addr));
		} else {
			hp = gethostbyname(hostname);
			if (hp == NULL) {
				fprintf(stderr, "Error, invalid hostname (%s).\n", hostname);
				return NET_INVALID_HOST;
			}
			memcpy((void *)&v4_addr.sin_addr, hp->h_addr, hp->h_length);
		}
	}

	/*
	 * Next, create a socket and attempt the connection
	 */
	*conn = fe_oob_new_connection();
	if (*conn == NULL) {
		fprintf(stderr, "Error, no memory for connection.\n");
		return NET_NO_MEMORY;
	}
	(*conn)->ipv6 = ipv6;
	(*conn)->sock = socket(((*conn)->ipv6) ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
	if ((*conn)->sock == INVALID_SOCKET) {
		fprintf(stderr, "Error, invalid socket.\n");
		free((void *)(*conn));
		return NET_INTERNAL_ERR;
	}

	if (connect((*conn)->sock,
				 ((*conn)->ipv6) ? (struct sockaddr *)&v6_addr : (struct sockaddr *)&v4_addr,
				 ((*conn)->ipv6) ? sizeof(v6_addr) : sizeof(v4_addr)) == SOCKET_ERROR) {
		fprintf(stderr, "Error, cannot connect to socket.\n");
		close((*conn)->sock);
		(*conn)->sock = INVALID_SOCKET;
		free((void *)(*conn));
		return NET_CANNOT_CONNECT;
	}

	if ((*conn)->ipv6)
		(*conn)->v6_addr = v6_addr;
	else
		(*conn)->v4_addr = v4_addr;

	NET_PRINT("Out-bound connection to %s port %d (conn #%d) established.\n",
			   hostname, port, (*conn)->sock);

    if (FE_IS_SSL_ENABLED()) {
        if (fe_ssl_init()) {
            fprintf(stderr, "Error, cannot initialize SSL/TLS\n");
            close((*conn)->sock);
            (*conn)->sock = INVALID_SOCKET;
            free((void *)(*conn));
            return NET_CANNOT_CONNECT;
        }

        // open SSL/TLS connection
        if (!(g_sslContext = fe_ssl_client_open(g_sslParmsData.sslSecurityDir, 
                                                g_sslParmsData.sslSecurityFFCertificate, 
                                                g_sslParmsData.sslSecurityFFPrivateKey, 
                                                g_sslParmsData.sslSecurityFFCaCertificate,
                                                g_sslParmsData.sslSecurityFFCertChainDepth,
                                                g_sslParmsData.sslSecurityFFDHParameters,
                                                g_sslParmsData.sslSecurityFFCaCRLEnabled,
                                                g_sslParmsData.sslSecurityFFCaCRL))) {
            fprintf(stderr, "Error, cannot open SSL/TLS connection\n"); 
            close((*conn)->sock); 
            (*conn)->sock = INVALID_SOCKET; 
            free((void *)(*conn)); 
            return NET_CANNOT_CONNECT;
        }

        // establish SSL/TLS session
        g_sslSession = fe_ssl_connect(g_sslContext, (*conn)->sock); 
        if (!g_sslSession) {
            fprintf(stderr, "Error, cannot establish SSL/TLS session\n");
            close((*conn)->sock);
            (*conn)->sock = INVALID_SOCKET;
            free((void *)(*conn));
            return NET_CANNOT_CONNECT;
        }
    }

	return NET_NO_ERROR;
}

int fe_oob_net_disconnect(struct net_connection *conn)
{
	struct net_blob *blob;
	int nr, ns;

	if (conn->sock == INVALID_SOCKET) {
		return NET_NOT_CONNECTED;
	}

	close(conn->sock);
	conn->sock = INVALID_SOCKET;

	/*
	 * Delete all enqueued blobs
	 */
	ns = 0;
	while (!fe_oob_queue_empty(&conn->send_queue)) {
		blob = fe_oob_dequeue_net_blob(&conn->send_queue);
		if (blob) fe_oob_free_net_blob(blob);
		++ns;
	}
	nr = 0;
	while (!fe_oob_queue_empty(&conn->recv_queue)) {
		blob = fe_oob_dequeue_net_blob(&conn->recv_queue);
		if (blob) fe_oob_free_net_blob(blob);
		++nr;
	}

	NET_PRINT("fe_oob_net_disconnect: closed connection %d, deleted %d send %d recv blobs\n",
			   conn->sock, ns, nr);

	return NET_NO_ERROR;
}

/*
 * Pack up the given data into a struct net_blob to be sent over the given
 * connection. Simply enqueue the blob here. fe_oob_net_sleep() will take
 * care of the actual sending.
 */
int fe_oob_net_send(struct net_connection *conn, char *data, int len, int *err)
{
	int magic;
	int tot_len;
	struct net_blob *blob;

	if (conn == NULL) {
		SET_ERROR(err, NET_NO_ERROR);
		return OK;
	}
	if (conn->sock == INVALID_SOCKET) {
		SET_ERROR(err, NET_NOT_CONNECTED);
		return ERR;
	}

	/*
	 * Copy the given data, prepended by a magic # and the tot msg len,
	 * into the blob.
	 */
	tot_len = len + 2 * sizeof(int);
	blob = fe_oob_new_net_blob(tot_len);
	if (blob == NULL || blob->data == NULL) {
		if (blob) fe_oob_free_net_blob(blob);
		SET_ERROR(err, NET_NO_MEMORY);
		return ERR;
	}
	magic = htonl(NET_MAGIC);
	memcpy((void *)blob->data, (void *)&magic, sizeof(int));
	tot_len = htonl(tot_len);
	memcpy((void *)(blob->data + sizeof(int)), (void *)&tot_len, sizeof(int));
	memcpy((void *)(blob->data + 2 * sizeof(int)), (void *)data, len);

	fe_oob_enqueue_net_blob(&conn->send_queue, blob);

	NET_PRINT("fe_oob_net_send: sent %d bytes at %p over conn %d\n", len, (void *)data, conn->sock);

	SET_ERROR(err, NET_NO_ERROR);
	return OK;
}

/*
 * Return the next message in the given connection's receive queue, NULL if none.
 */
void fe_oob_net_get_next_message(struct net_connection *conn, uint8_t **data, int *len, int *err)
{
	struct net_blob *blob;

	SET_ERROR(err, NET_NO_ERROR);

	if (conn == NULL) {
		if (data) {
			*data = NULL;
		}
		if (len) {
			*len = 0;
		}
		return;
	}

	blob = fe_oob_dequeue_net_blob(&conn->recv_queue);
	if (blob == NULL) {
		if (data) {
			*data = NULL;
		}
		if (len) {
			*len = 0;
		}
		return;
	} else {
		if (data) {
			*data = blob->data;
		}
		if (len) {
			*len = blob->len;
		}
		blob->data = NULL; /* so fe_oob_free_net_blob() won't free the data */
		fe_oob_free_net_blob(blob);
		return;
	}
}

struct net_connection *fe_oob_net_sleep(int msec_to_wait, int blocking)
{
	int n;
	int nfds;
	struct net_connection *conn;
	struct net_connection *retval = NULL;
	fd_set readfds;
	fd_set writefds;
	fd_set errorfds;
	struct timeval timeout;

	/*
	 * Do a select on the listen socket (to catch new connections),
	 * on all in-bound sockets, and on those out-bound sockets for
	 * which we have traffic enqueued. If blocking!=0, do the select
	 * even if there's nothing to listen for (so we will wait msec_to_wait
	 * always).
	 */
	FD_ZERO(&errorfds);
	FD_ZERO(&readfds);
	FD_ZERO(&writefds);
	nfds = 0;

	for (conn = g_connections_; conn; conn = conn->next) {
		FD_SET(conn->sock, &readfds);
		nfds = MAX(nfds, (int)conn->sock);
		if (!fe_oob_queue_empty(&conn->send_queue)) {
			nfds = MAX(nfds, (int)conn->sock);
			FD_SET(conn->sock, &writefds);
		}
	}
	if ((nfds == 0) && !blocking) {
		return NULL;
	}
	++nfds;

	if (msec_to_wait < 0) {
		msec_to_wait = 0;
	}
	timeout.tv_sec = msec_to_wait / 1000;
	timeout.tv_usec = msec_to_wait % 1000;
	n = select(nfds, &readfds, &writefds, &errorfds, &timeout);
	if (n == SOCKET_ERROR) {
		return NULL;
	}
	if (n == 0) {
		return NULL;
	}
	for (conn = g_connections_; conn; conn = conn->next) {
		if (FD_ISSET(conn->sock, &readfds)) {
			if (fe_oob_read_from_socket(conn) == ERR) {
				conn->err = 1;
				continue; /* don't bother trying to write */
			}
		}
		if (FD_ISSET(conn->sock, &writefds)) {
			if (fe_oob_write_to_socket(conn) == ERR) {
				conn->err = 1;
			}
		}
	}

	/*
	 * Handle errs here so we don't hassle with removing from
	 * conn list while iterating
	 */
	conn = g_connections_;
	while (conn) {
		if (conn->err) {
			fe_oob_handle_read_write_error(conn);
			conn = g_connections_; /* start over */
		} else {
			conn = conn->next;
		}
	}

	return retval;
}

static int fe_oob_read_from_socket(struct net_connection *conn)
{
	ssize_t bytes_read;
	struct net_blob *blob;

	/*
	 * If we're in the middle of a message, pick up where we left off.
	 * Otherwise, start reading a new message.
	 */
	if (conn->blob_in_progress == NULL) {
		blob = fe_oob_new_net_blob(0);
		if(blob){
			blob->data = NULL; /* this flags that we haven't read the msg size yet */
			blob->cur_ptr = (uint8_t *)blob->magic;
			blob->bytes_left = 2 * sizeof(int);
			conn->blob_in_progress = blob;
		} else {
			NET_PRINT("fe_oob_read_from_socket: Received NULL blob from socket.");
			return ERR;
		}
	} else {
		blob = conn->blob_in_progress;
	}

    if (FE_IS_SSL_ENABLED()) {
        bytes_read = fe_ssl_read(g_sslSession, blob->cur_ptr, blob->bytes_left);
    } else {
        bytes_read = recv(conn->sock, blob->cur_ptr, blob->bytes_left, 0);
    }

	if (bytes_read == 0) { /* graceful shutdown */
		NET_PRINT("fe_oob_read_from_socket: conn %d shut down gracefully\n", conn->sock);
		return ERR;
	} else if (bytes_read == SOCKET_ERROR) {
		NET_PRINT("fe_oob_read_from_socket: err %zd, %d over connection %d\n",
				   bytes_read, errno, conn->sock);
		return ERR;
	} else {
		if (bytes_read < blob->bytes_left) { /* still more to read */
			fe_oob_adjust_blob_cur_ptr(blob, bytes_read);
			NET_PRINT("fe_oob_read_from_socket: read %zu bytes over conn %d, %zu bytes to go\n",
					   bytes_read, conn->sock, blob->bytes_left);
			return OK;
		} else {
			if (blob->data == NULL) { /* NULL means we just finished reading msg size */
				/* if we didn't get the magic, DISCONNECT this connection */
				if (ntohl(blob->magic[0]) != NET_MAGIC) {
					fe_oob_handle_read_write_error(conn);
					fe_oob_free_net_blob(blob);
					return ERR;
				}

				blob->len = ntohl(blob->magic[1]) - 2 * sizeof(int);
				blob->data = malloc(blob->len);
				if (blob->data == NULL) {
					/* No memory! Bail out and disconnect, since we have to lose this msg */
					fe_oob_free_net_blob(blob);
					return ERR;
				}
				blob->cur_ptr = blob->data;
				blob->bytes_left = blob->len;
				NET_PRINT("fe_oob_read_from_socket: read %zd bytes over conn %d, start reading size %zu\n",
						   bytes_read, conn->sock, blob->len);
				return OK;
			} else { /* we just finished reading the user data -- enqueue blob */
				blob->bytes_left = 0;
				blob->cur_ptr = NULL;
				fe_oob_enqueue_net_blob(&conn->recv_queue, blob);
				conn->blob_in_progress = NULL;
				NET_PRINT("fe_oob_read_from_socket: read %zd bytes over conn %d, finish reading msg of size %zu\n",
						   bytes_read, conn->sock, blob->len);
				return OK;
			}
		}
	}
}

static int fe_oob_write_to_socket(struct net_connection *conn)
{
	ssize_t bytes_sent;
	struct net_blob *blob;

	if (fe_oob_queue_empty(&conn->send_queue)) {
		return OK;
	}

	blob = fe_oob_peek_net_blob(&conn->send_queue);

	/*
	 * #define TEST if you want to stress test message fragmentation.
	 * Leave undefined for release build.
	 */
#ifdef TEST
	xxx = blob->bytes_left / 2;
	if (xxx == 0) {
		xxx = blob->bytes_left;
	}
    if (FE_IS_SSL_ENABLED()) {
        bytes_sent = fe_ssl_write(g_sslSession, blob->cur_ptr, xxx);
    } else {
        bytes_sent = send(conn->sock, blob->cur_ptr, xxx, 0);
    }
#else
    if (FE_IS_SSL_ENABLED()) {
        bytes_sent = fe_ssl_write(g_sslSession, blob->cur_ptr, blob->bytes_left);
    } else {
        bytes_sent = send(conn->sock, blob->cur_ptr, blob->bytes_left, 0);
    }
#endif

	NET_PRINT("fe_oob_write_to_socket: wrote %zd bytes over conn %d\n", bytes_sent, conn->sock);

	if (bytes_sent == SOCKET_ERROR) {
		/*
		 * If we couldn't send because the send() would block, then just
		 * return.	We'll try again next time.
		 */
		if (errno == WSAEWOULDBLOCK) {
			return OK;
		} else {
			return ERR;
		}
	} else {
		/*
		 * If we sent the entire message, destroy it and go on to the next one
		 * in the queue. Otherwise, return; we'll continue were we left off
		 * next time.
		 */
		if (bytes_sent == blob->bytes_left) {
			blob = fe_oob_dequeue_net_blob(&conn->send_queue);
			if (blob) fe_oob_free_net_blob(blob);
			return OK;
		} else {
			fe_oob_adjust_blob_cur_ptr(blob, bytes_sent);
			return OK;
		}
	}
}

static struct net_connection *fe_oob_new_connection ()
/*
 * Return a new struct net_connection object.
 */
{
	struct net_connection *conn = malloc(sizeof(struct net_connection));
	if (conn == NULL) {
		return NULL;
	}

	conn->sock = INVALID_SOCKET;
	fe_oob_init_queue(&conn->send_queue);
	fe_oob_init_queue(&conn->recv_queue);
	conn->blob_in_progress = NULL;
	conn->err = 0;
	conn->prev = NULL;
	conn->next = NULL;
	return conn;
}

static void fe_oob_handle_read_write_error(struct net_connection *conn)
/*
 * A send or recv over a connection failed. Report an error and disconnect.
 */
{
	NET_PRINT("Read/write error over connection %d\n", conn->sock);

	fe_oob_net_disconnect(conn);
}

void fe_oob_add_to_connection_list(struct net_connection *conn)
{
	if (g_connections_ == NULL) {
		g_connections_ = conn;
		conn->prev = NULL;
		conn->next = NULL;
	} else {
		conn->next = g_connections_;
		conn->prev = NULL;
		g_connections_->prev = conn;
		g_connections_ = conn;
	}
}

void fe_oob_remove_from_connection_list(struct net_connection *conn)
{
	if (conn->next) {
		conn->next->prev = conn->prev;
	}
	if (conn->prev) {
		conn->prev->next = conn->next;
	}
	if (conn->prev == NULL) { /* conn is at head of list */
		g_connections_ = conn->next;
	}
}

/*-------------------------------------------------------------------------*
*
* FUNCTION
*		fe_oob_net_process
*
* DESCRIPTION
*		Put the connection to sleep and wait for the specified time to awake.
*		Listens for data to be sent and received and calls the appropriate
*		read/write socket function.
*
* INPUTS
*		msec_to_wait		Milliseconds to wait
*		blocking		specifies whether to block while sleeping
*
* OUTPUTS
*		struct net_connection	the new net connection or NULL if a connection
*						does not exist
*
*-------------------------------------------------------------------------*/
struct net_connection *fe_oob_net_process(int msec_to_wait, int blocking) {
	int n, nfds;
	struct net_connection *conn;
	struct net_connection *retval = NULL;
	fd_set readfds, writefds, errorfds;
	int queued_data = 0, inprogress_data = 0;
	struct timeval timeout;

	/*
	 * Do a select on the listen socket (to catch new connections),
	 * on all in-bound sockets, and on those out-bound sockets for
	 * which we have traffic enqueued.	If blocking!=0, do the select
	 * even if there's nothing to listen for (so we will wait msec_to_wait
	 * always).
	 */
	FD_ZERO(&errorfds);
	FD_ZERO(&readfds);
	FD_ZERO(&writefds);
	nfds = 0;

	for (conn = g_connections_; conn; conn = conn->next) {
		FD_SET(conn->sock, &readfds);
		nfds = MAX(nfds, (int)conn->sock);
		if (!fe_oob_queue_empty(&conn->send_queue)) {
			queued_data++;
			nfds = MAX(nfds, (int)conn->sock);
			FD_SET(conn->sock, &writefds);
		}
        if (conn->blob_in_progress) {
            inprogress_data++;
        }
	}
	if ((nfds == 0) && !blocking) {
		return(NULL);
	}
	++nfds;

	if (msec_to_wait < 0) {
		msec_to_wait = 0;
	}
	timeout.tv_sec = msec_to_wait / 1000;
	timeout.tv_usec = (msec_to_wait % 1000) * 1000;
	n = select(nfds, &readfds, &writefds, &errorfds, &timeout);

	if (n == SOCKET_ERROR) {
		return(NULL);
	}
	if (n == 0) {
        if (inprogress_data == 0) {
    		return(NULL);
        }
	}

	for (conn = g_connections_; conn; conn = conn->next) {
		if (FD_ISSET(conn->sock, &readfds) || conn->blob_in_progress) {
			if (fe_oob_read_from_socket(conn) == NET_FAILED) {
				conn->err = 1;
				continue; /* don't bother trying to write */
			}
		}
		if (FD_ISSET(conn->sock, &writefds) || !fe_oob_queue_empty(&conn->send_queue)) {
			if (fe_oob_write_to_socket(conn) == NET_FAILED) {
				conn->err = 1;
			}
		}
	}

	/*
	 * Handle errs here so we don't hassle with removing from
	 * conn list while iterating
	 */
	conn = g_connections_;
	while (conn) {
		if (conn->err) {
			fe_oob_handle_read_write_error(conn);
			conn = g_connections_; /* start over */
		} else {
			conn = conn->next;
		}
	}

	return retval;
}

static int fe_oob_print_addrinfo(char *hostname)
{
	int erc;
	struct addrinfo hints;
	struct addrinfo *res = NULL;
	struct addrinfo *ai;
	char host_bfr[NI_MAXHOST]; /* For use w/getnameinfo(3). */
	char serv_bfr[NI_MAXSERV]; /* For use w/getnameinfo(3). */

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = PF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags |= AI_CANONNAME;
	erc = getaddrinfo(hostname, NULL, &hints, &res);

	if (erc != 0 || !res) {
		fprintf(stderr, "Unable to get addressing information on IP address\n");
		return NET_INTERNAL_ERR;
	}

	for (ai = res; ai != NULL; ai = ai->ai_next) {
		/*
		** Display the current address info. Start with the protocol-
		** independent fields first.
		*/
		fprintf(stderr,
				"Setting up a socket connection based on the "
				"following address info:\n"
				"	ai_flags	 = 0x%02X\n"
				"	ai_family	 = %d (PF_INET = %d, PF_INET6 = %d)\n"
				"	ai_socktype	 = %d (SOCK_STREAM = %d, SOCK_DGRAM = %d)\n"
				"	ai_protocol	 = %d (IPPROTO_TCP = %d, IPPROTO_UDP = %d)\n"
				"	ai_addrlen	 = %d (sockaddr_in = %d, "
				"sockaddr_in6 = %d)\n",
				ai->ai_flags,
				ai->ai_family,
				PF_INET,
				PF_INET6,
				ai->ai_socktype,
				SOCK_STREAM,
				SOCK_DGRAM,
				ai->ai_protocol,
				IPPROTO_TCP,
				IPPROTO_UDP,
				ai->ai_addrlen,
				(int)sizeof(struct sockaddr_in),
				(int)sizeof(struct sockaddr_in6));

		/*
		** Now display the protocol-specific formatted socket address. Note
		** that the program is requesting that getnameinfo(3) convert the
		** host & service into numeric strings.
		*/
		getnameinfo(ai->ai_addr,
					ai->ai_addrlen,
					host_bfr,
					sizeof(host_bfr),
					serv_bfr,
					sizeof(serv_bfr),
					NI_NUMERICHOST | NI_NUMERICSERV);
		switch (ai->ai_family) {
		case PF_INET: /* IPv4 address record. */
			{
				struct sockaddr_in *p = (struct sockaddr_in *)ai->ai_addr;
				fprintf(stderr,
						"	ai_addr		 = sin_family:	 %d (AF_INET = %d, "
						"AF_INET6 = %d)\n"
						"				   sin_addr:	 %s\n"
						"				   sin_port:	 %s\n",
						p->sin_family,
						AF_INET,
						AF_INET6,
						host_bfr,
						serv_bfr);
				break;
			}
		case PF_INET6:
			{
				struct sockaddr_in6 *p = (struct sockaddr_in6 *)ai->ai_addr;
				fprintf(stderr,
						"	ai_addr		 = sin6_family:	  %d (AF_INET = %d, "
						"AF_INET6 = %d)\n"
						"				   sin6_addr:	  %s\n"
						"				   sin6_port:	  %s\n"
						"				   sin6_flowinfo: %d\n"
						"				   sin6_scope_id: %d\n",
						p->sin6_family,
						AF_INET,
						AF_INET6,
						host_bfr,
						serv_bfr,
						p->sin6_flowinfo,
						p->sin6_scope_id);
				break;
			}
		default:
			{
				fprintf(stderr,
						"Error, unknown protocol family (%d).\n",
						ai->ai_family);
				freeaddrinfo(res);
				break;
			}
		}
	}
	if (res)
		freeaddrinfo(res);
	return OK;
}
