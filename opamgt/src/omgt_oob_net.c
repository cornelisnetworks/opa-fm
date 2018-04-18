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

#define OPAMGT_PRIVATE 1

#include "ib_utils_openib.h"
#include "omgt_oob_net.h"
#include "omgt_oob_ssl.h"
#include "opamgt_dump_mad.h"

#define CONNECTION_BACKLOG 10

#define SET_ERROR(x,y) if (x) { *(x)=(y); }
#define NET_MAGIC 0x31E0CC01

static FSTATUS omgt_oob_read_from_socket(struct omgt_port *port, struct net_connection *conn);
static FSTATUS omgt_oob_write_to_socket(struct omgt_port *port, struct net_connection *conn);
static struct net_connection* omgt_oob_new_connection();
static FSTATUS omgt_oob_print_addrinfo(struct omgt_port *port, char *hostname, uint16_t conn_port);


/**
 * @brief Establish a connection to the given host.
 * @param port            port object with connection info
 * @return FSTATUS
 */
FSTATUS omgt_oob_net_connect(struct omgt_port *port, struct net_connection **cnn)
{
	struct net_connection *conn = NULL;
	struct sockaddr_in v4_addr;
	struct sockaddr_in6 v6_addr;
	struct in_addr ipv4addr;
	struct in6_addr ipv6addr;
	struct hostent *hp = NULL;
	int inaddr = 0;
	int ipv6 = 0;

	/* Set Timeout to default value if incorrect value */
	if (port->ms_timeout <= 0) {
		port->ms_timeout = OMGT_DEF_TIMEOUT_MS;
	}
	if (port->retry_count < 0) {
		port->retry_count = OMGT_DEF_RETRY_CNT;
	}
	/* First resolve the hostname and fill in the sockaddr */
	inaddr = inet_pton(AF_INET6, port->oob_input.host, &ipv6addr);

	if ((ipv6 = (inaddr == 1) ? 1 : 0)) {
		memset((void *)&v6_addr, 0, sizeof(v6_addr));
		v6_addr.sin6_family = AF_INET6;
		v6_addr.sin6_port = htons((short)port->oob_input.port);
		memcpy((void *)&v6_addr.sin6_addr, (void *)&ipv6addr, sizeof(ipv6addr));

		if (port->dbg_file) {
			(void)omgt_oob_print_addrinfo(port, port->oob_input.host, port->oob_input.port);
		}
	} else {
		memset((void *)&v4_addr, 0, sizeof(v4_addr));
		v4_addr.sin_family = AF_INET;
		v4_addr.sin_port = htons((short)port->oob_input.port);
		inaddr = inet_pton(AF_INET, port->oob_input.host, &ipv4addr);

		if (inaddr == 1) {
			memcpy((void *)&v4_addr.sin_addr, (void *)&ipv4addr, sizeof(ipv4addr));
		} else {
			hp = gethostbyname(port->oob_input.host);
			if (hp == NULL) {
				OMGT_OUTPUT_ERROR(port, "invalid hostname (%s).\n", port->oob_input.host);
				return FINVALID_PARAMETER;
			}
			memcpy((void *)&v4_addr.sin_addr, hp->h_addr, hp->h_length);
		}
	}

	/* Next, create a socket and attempt the connection */
	conn = omgt_oob_new_connection();
	if (conn == NULL) {
		OMGT_OUTPUT_ERROR(port, "no memory for connection.\n");
		return FINSUFFICIENT_MEMORY;
	}
	conn->ipv6 = ipv6;
	conn->sock = socket((conn->ipv6) ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
	if (conn->sock == INVALID_SOCKET) {
		OMGT_OUTPUT_ERROR(port, "invalid socket.\n");
		free(conn);
		return FINVALID_STATE;
	}

	if (connect(conn->sock,
			(conn->ipv6) ? (struct sockaddr *)&v6_addr : (struct sockaddr *)&v4_addr,
			(conn->ipv6) ? sizeof(v6_addr) : sizeof(v4_addr)) == SOCKET_ERROR) {

		OMGT_OUTPUT_ERROR(port, "cannot connect to socket.\n");
		goto bail;
	}

	if (conn->ipv6) {
		conn->v6_addr = v6_addr;
	} else {
		conn->v4_addr = v4_addr;
	}

	OMGT_DBGPRINT(port, "Out-bound connection to %s port %d (conn #%d) established.\n",
		port->oob_input.host, port->oob_input.port, conn->sock);

	// Should we setup SSL/TLS
	if (port->oob_input.ssl_params.enable) {
		port->is_ssl_enabled = TRUE;
		if (omgt_oob_ssl_init(port)) {
			OMGT_OUTPUT_ERROR(port, "cannot initialize SSL/TLS\n");
			goto bail;
		}

		if (port->ssl_context == NULL) {
			// open SSL/TLS connection
			if (!(port->ssl_context = omgt_oob_ssl_client_open(port,
						port->oob_input.ssl_params.directory,
						port->oob_input.ssl_params.certificate,
						port->oob_input.ssl_params.private_key,
						port->oob_input.ssl_params.ca_certificate,
						port->oob_input.ssl_params.cert_chain_depth,
						port->oob_input.ssl_params.dh_params,
						port->oob_input.ssl_params.ca_crl_enable,
						port->oob_input.ssl_params.ca_crl))) {
				OMGT_OUTPUT_ERROR(port, "cannot open SSL/TLS connection\n");
				goto bail;
			}
		}

		// establish SSL/TLS session
		conn->ssl_session = omgt_oob_ssl_connect(port, port->ssl_context, conn->sock);
		if (!conn->ssl_session) {
			OMGT_OUTPUT_ERROR(port, "cannot establish SSL/TLS session\n");
			goto bail; 
		}
	}

	*cnn = conn;
	return FSUCCESS;

bail:
	close(conn->sock);
	conn->sock = INVALID_SOCKET;
	free(conn);
	return FERROR;
}

FSTATUS omgt_oob_net_disconnect(struct omgt_port *port, struct net_connection *conn)
{
	struct net_blob *blob;
	int nr, ns;

	if (!conn || conn->sock == INVALID_SOCKET) {
		return FINVALID_PARAMETER;
	}

	close(conn->sock);
	conn->sock = INVALID_SOCKET;

	/*
	 * Delete all enqueued blobs
	 */
	ns = 0;
	while (!omgt_oob_queue_empty(&conn->send_queue)) {
		blob = omgt_oob_dequeue_net_blob(&conn->send_queue);
		if (blob) omgt_oob_free_net_blob(blob);
		++ns;
	}
	nr = 0;
	while (!omgt_oob_queue_empty(&conn->recv_queue)) {
		blob = omgt_oob_dequeue_net_blob(&conn->recv_queue);
		if (blob) omgt_oob_free_net_blob(blob);
		++nr;
	}

	OMGT_DBGPRINT(port, "closed connection %d, deleted %d send %d recv blobs\n",
		conn->sock, ns, nr);

	free(conn);

	return FSUCCESS;
}

/*
 * Pack up the given data into a struct net_blob to be sent over the given
 * connection. Simply enqueue the blob here. omgt_oob_net_sleep() will take
 * care of the actual sending.
 */
FSTATUS omgt_oob_net_send(struct omgt_port *port, uint8_t *data, int len)
{
	int magic;
	int tot_len;
	struct net_blob *blob;

	if (!port || !port->conn) {
		return FINVALID_PARAMETER;
	}
	if (port->conn->sock == INVALID_SOCKET) {
		return FINVALID_PARAMETER;
	}

	/*
	 * Copy the given data, prepended by a magic # and the tot msg len,
	 * into the blob.
	 */
	tot_len = len + 2 * sizeof(int);
	blob = omgt_oob_new_net_blob(tot_len);
	if (blob == NULL || blob->data == NULL) {
		if (blob) omgt_oob_free_net_blob(blob);
		return FINSUFFICIENT_MEMORY;
	}
	magic = htonl(NET_MAGIC);
	memcpy((void *)blob->data, (void *)&magic, sizeof(int));
	tot_len = htonl(tot_len);
	memcpy((void *)(blob->data + sizeof(int)), (void *)&tot_len, sizeof(int));
	memcpy((void *)(blob->data + 2 * sizeof(int)), (void *)data, len);

	if (port->dbg_file) {
		OMGT_DBGPRINT(port, ">>> sending: len %d pktsz %d\n", len, tot_len);
		omgt_dump_mad(port->dbg_file, data, len, "send mad\n");
	}

	omgt_oob_enqueue_net_blob(&port->conn->send_queue, blob);

	OMGT_DBGPRINT(port, "sent %d bytes at %p over conn %d\n", len, (void *)data, port->conn->sock);

	return FSUCCESS;
}

/*
 * Return the next message in the given connection's receive queue, NULL if none.
 */
void omgt_oob_net_get_next_message(struct net_connection *conn, uint8_t **data, int *len)
{
	struct net_blob *blob;

	if (conn == NULL) {
		if (data) {
			*data = NULL;
		}
		if (len) {
			*len = 0;
		}
		return;
	}

	blob = omgt_oob_dequeue_net_blob(&conn->recv_queue);
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
		blob->data = NULL; /* so omgt_oob_free_net_blob() won't free the data */
		omgt_oob_free_net_blob(blob);
		return;
	}
}

static FSTATUS omgt_oob_read_from_socket(struct omgt_port *port, struct net_connection *conn)
{
	ssize_t bytes_read;
	struct net_blob *blob;

	/*
	 * If we're in the middle of a message, pick up where we left off.
	 * Otherwise, start reading a new message.
	 */
	if (conn->blob_in_progress == NULL) {
		blob = omgt_oob_new_net_blob(0);
		if (blob) {
			blob->data = NULL; /* this flags that we haven't read the msg size yet */
			blob->cur_ptr = (uint8_t *)blob->magic;
			blob->bytes_left = 2 * sizeof(int);
			conn->blob_in_progress = blob;
		} else {
			OMGT_DBGPRINT(port, "Received NULL blob from socket.");
			return FERROR;
		}
	} else {
		blob = conn->blob_in_progress;
	}

	if (port->is_ssl_enabled && port->is_ssl_initialized) {
		bytes_read = omgt_oob_ssl_read(port, conn->ssl_session, blob->cur_ptr, blob->bytes_left);
	} else {
		bytes_read = recv(conn->sock, blob->cur_ptr, blob->bytes_left, 0);
	}

	if (bytes_read == 0) { /* graceful shutdown */
		OMGT_DBGPRINT(port, "conn %d shut down gracefully\n", conn->sock);
		return FERROR;
	} else if (bytes_read == SOCKET_ERROR) {
		OMGT_DBGPRINT(port, "err %zd, %d over connection %d\n", bytes_read, errno, conn->sock);
		return FERROR;
	} else {
		if (bytes_read < blob->bytes_left) { /* still more to read */
			omgt_oob_adjust_blob_cur_ptr(blob, bytes_read);
			OMGT_DBGPRINT(port, "read %zu bytes over conn %d, %zu bytes to go\n",
				bytes_read, conn->sock, blob->bytes_left);
			return FSUCCESS;
		} else {
			if (blob->data == NULL) { /* NULL means we just finished reading msg size */
				/* if we didn't get the magic, DISCONNECT this connection */
				if (ntohl(blob->magic[0]) != NET_MAGIC) {
					OMGT_OUTPUT_ERROR(port, "Read/write error over connection %d\n", conn->sock);
					omgt_oob_free_net_blob(blob);
					return FERROR;
				}

				blob->len = ntohl(blob->magic[1]) - 2 * sizeof(int);
				blob->data = malloc(blob->len);
				if (blob->data == NULL) {
					/* No memory! Bail out and disconnect, since we have to lose this msg */
					omgt_oob_free_net_blob(blob);
					return FERROR;
				}
				blob->cur_ptr = blob->data;
				blob->bytes_left = blob->len;
				OMGT_DBGPRINT(port, "read %zd bytes over conn %d, start reading size %zu\n",
					bytes_read, conn->sock, blob->len);
				return FSUCCESS;
			} else { /* we just finished reading the user data -- enqueue blob */
				blob->bytes_left = 0;
				blob->cur_ptr = NULL;
				omgt_oob_enqueue_net_blob(&conn->recv_queue, blob);
				conn->blob_in_progress = NULL;
				OMGT_DBGPRINT(port, "read %zd bytes over conn %d, finish reading msg of size %zu\n",
					bytes_read, conn->sock, blob->len);
				return FSUCCESS;
			}
		}
	}
}

static FSTATUS omgt_oob_write_to_socket(struct omgt_port *port, struct net_connection *conn)
{
	ssize_t bytes_sent;
	struct net_blob *blob;

	if (omgt_oob_queue_empty(&conn->send_queue)) {
		return FSUCCESS;
	}

	blob = omgt_oob_peek_net_blob(&conn->send_queue);

	/*
	 * #define TEST if you want to stress test message fragmentation.
	 * Leave undefined for release build.
	 */
#ifdef TEST
	xxx = blob->bytes_left / 2;
	if (xxx == 0) {
		xxx = blob->bytes_left;
	}
	if (port->is_ssl_enabled && port->is_ssl_initialized) {
		bytes_sent = omgt_oob_ssl_write(port, conn->ssl_session, blob->cur_ptr, xxx);
	} else {
		bytes_sent = send(conn->sock, blob->cur_ptr, xxx, 0);
	}
#else
	if (port->is_ssl_enabled && port->is_ssl_initialized) {
		bytes_sent = omgt_oob_ssl_write(port, conn->ssl_session, blob->cur_ptr, blob->bytes_left);
	} else {
		bytes_sent = send(conn->sock, blob->cur_ptr, blob->bytes_left, 0);
	}
#endif

	OMGT_DBGPRINT(port, "wrote %zd bytes over conn %d\n", bytes_sent, conn->sock);

	if (bytes_sent == SOCKET_ERROR) {
		/*
		 * If we couldn't send because the send() would block, then just
		 * return.	We'll try again next time.
		 */
		if (errno == EWOULDBLOCK) {
			return FSUCCESS;
		} else {
			return FERROR;
		}
	} else {
		/*
		 * If we sent the entire message, destroy it and go on to the next one
		 * in the queue. Otherwise, return; we'll continue were we left off
		 * next time.
		 */
		if (bytes_sent == blob->bytes_left) {
			blob = omgt_oob_dequeue_net_blob(&conn->send_queue);
			if (blob) omgt_oob_free_net_blob(blob);
			return FSUCCESS;
		} else {
			omgt_oob_adjust_blob_cur_ptr(blob, bytes_sent);
			return FSUCCESS;
		}
	}
}

static struct net_connection* omgt_oob_new_connection()
/*
 * Return a new struct net_connection object.
 */
{
	struct net_connection *conn = malloc(sizeof(struct net_connection));
	if (conn == NULL) {
		return NULL;
	}

	conn->sock = INVALID_SOCKET;
	omgt_oob_init_queue(&conn->send_queue);
	omgt_oob_init_queue(&conn->recv_queue);
	conn->blob_in_progress = NULL;
	conn->err = 0;
	return conn;
}

/**
 * @brief Listen on connection for data
 *
 * Put the connection to sleep and wait for the specified time to awake. Listens
 * for data to be sent and received and calls the appropriate read/write socket
 * function.
 *
 * @param port                port object 
 * @param conn                connection to listen on 
 * @param msec_to_wait        number of milliseconds to wait
 * @param blocking            specifies whether to block while sleeping
 *
 * @return None
 */
void omgt_oob_net_process(struct omgt_port *port, struct net_connection *conn, int msec_to_wait, int blocking)
{
	int n, nfds;
	fd_set readfds, writefds, errorfds;
	int queued_data = 0, inprogress_data = 0;
	struct timeval timeout = {0};

	/* Do nothing if no conn */
	if (!port || !conn)
		return;

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

	FD_SET(conn->sock, &readfds);
	nfds = MAX(nfds, (int)conn->sock);
	if (!omgt_oob_queue_empty(&conn->send_queue)) {
		queued_data++;
		nfds = MAX(nfds, (int)conn->sock);
		FD_SET(conn->sock, &writefds);
	}
	if (conn->blob_in_progress) {
		inprogress_data++;
	}

	if ((nfds == 0) && !blocking) {
		return;
	}
	++nfds;

	if (msec_to_wait > 0) {
		timeout.tv_sec = msec_to_wait / 1000;
		timeout.tv_usec = (msec_to_wait % 1000) * 1000;
	}
	n = select(nfds, &readfds, &writefds, &errorfds, (msec_to_wait < 0 ? NULL : &timeout));

	if (n == SOCKET_ERROR) {
		return;
	}
	if (n == 0 && inprogress_data == 0) {
		return;
	}

	if (FD_ISSET(conn->sock, &writefds) || !omgt_oob_queue_empty(&conn->send_queue)) {
		conn->err = omgt_oob_write_to_socket(port, conn);
	}
	if (conn->err == 0 &&
		(FD_ISSET(conn->sock, &readfds) || conn->blob_in_progress))
	{
		conn->err = omgt_oob_read_from_socket(port, conn);
	}

	if (conn->err) {
		OMGT_OUTPUT_ERROR(port, "Read/write error over connection %d\n", conn->sock);
	}

	return;
}

static FSTATUS omgt_oob_print_addrinfo(struct omgt_port *port, char *hostname, uint16_t conn_port)
{
	int erc;
	struct addrinfo hints;
	struct addrinfo *res = NULL;
	struct addrinfo *ai;
	char host_bfr[NI_MAXHOST];
	char serv_bfr[NI_MAXSERV];
	char port_bfr[NI_MAXSERV];

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = PF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags |= AI_CANONNAME;
	snprintf(port_bfr, NI_MAXSERV, "%u", conn_port);
	erc = getaddrinfo(hostname, port_bfr, &hints, &res);

	if (erc != 0 || !res) {
		OMGT_DBGPRINT(port, "Unable to get addressing information on IP address\n");
		return FERROR;
	}

	for (ai = res; ai != NULL; ai = ai->ai_next) {
		OMGT_DBGPRINT(port,
			"Setting up a socket connection based on the following address info:\n"
			" IP Version: %s\n IP Proto:   %s\n",
			(ai->ai_family == PF_INET ? "IPv4" : "IPv6"),
			(ai->ai_protocol == IPPROTO_TCP ? "TCP" : "UDP"));

		getnameinfo(ai->ai_addr, ai->ai_addrlen, host_bfr, sizeof(host_bfr), serv_bfr,
			sizeof(serv_bfr), NI_NUMERICHOST | NI_NUMERICSERV);

		OMGT_DBGPRINT(port,
			" IP Address: %s\n Port:       %s\n",
			host_bfr, serv_bfr);
	}
	if (res)
		freeaddrinfo(res);
	return FSUCCESS;
}
