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
*   net.c
*
* DESCRIPTION
*   Libnet protocol main routine
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   02/12/2000      Initial file for checkin to source control
* jrw   11/27/2001      Added prologues, comments, logging.  Removed 
*                       ASSERTS, replaced malloc and free with 
*                       vs_pool_alloc vs_pool_free. 
*
***********************************************************************/

#include "libnet.h"
#ifdef __VXWORKS__
#include <memLib.h>
#include <hostLib.h>
#else
#include <memory.h>
#endif
#include "ib_types.h"
#include "ib_status.h"
#include "vs_g.h"
#include "cs_log.h"
#include "fm_xml.h"
#include "if3.h"

#undef  LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_FE_MOD_ID

#define CONNECTION_BACKLOG 10
#define SET_ERROR(x,y)  if (x) { *(x)=(y); }
#define NET_MAGIC 0x31E0CC01

#ifndef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif

extern uint8_t fe_is_thread(void);
extern const struct in6_addr in6addr_any;
extern FEXmlConfig_t fe_config;

Pool_t fe_pool;                /* Memory pool for FE    */
static int G_initted_;
static SOCKET_t G_listenSock_ = INVALID_SOCKET;
static int G_numConnections_;
static int G_connectCount;
static NetConnection *G_connections_;
static void *G_sslContext_ = NULL;

/* Function prototypes  */
static void (*DisconnectCallBack) (NetConnection *) = NULL;
static int ReadFromSocket (NetConnection *);
static int WriteToSocket (NetConnection *);
static NetConnection *AcceptConnection (void);
static NetConnection *NewConnection (void);
static void HandleReadWriteError (NetConnection *);
static void AddToConnectionList (NetConnection *);
static void RemoveFromConnectionList (NetConnection *);
static void CloseSock (SOCKET_t);


int fe_net_init(int port, NetError *err, void (*Callback)(NetConnection *))
{
    struct sockaddr_in6 addr;
    int                 opt;
    int                 rc  = NET_OK;

#ifdef WINDOWS
    WORD wVersionRequested;
    WSADATA wsaData;
#endif

    IB_ENTER(__func__,port,0,0,0);

    if (G_initted_) {
        IB_EXIT(__func__,err);
        return(NET_OK);
    }

#ifdef WINDOWS
    wVersionRequested = MAKEWORD (2, 0);

    if (WSAStartup (wVersionRequested, &wsaData)) {
        SET_ERROR (err, NET_INTERNAL_ERR);
        IB_LOG_ERROR("init error err:", err);
        fprintf(stderr,"fe_net_init error%d\n", (int)*err);
        IB_EXIT(__func__,err);
        return(NET_FAILED);
    }
#endif

    /* Setup callback pointer   */
    if (Callback) {
        DisconnectCallBack = Callback;
    }
    else {
        DisconnectCallBack = NULL;
    }

    if (port) {
        /*
         * Setup IPv6 connection for client requests.
         */
        G_listenSock_ = SOCKET (AF_INET6, SOCK_STREAM, 0);
        if (G_listenSock_ == INVALID_SOCKET) {
            SET_ERROR (err, NET_INTERNAL_ERR);
            IB_LOG_ERROR("init error err:", (int)*err);
            fprintf(stderr,"fe_net_init error%d\n", (int)*err);
            IB_EXIT(__func__,err);
            return(NET_FAILED);
        }

        opt = 1;

        if (setsockopt (G_listenSock_, SOL_SOCKET, SO_REUSEADDR,
                         (char *) &opt, sizeof (int))) {
            SET_ERROR (err, NET_INTERNAL_ERR);
            CloseSock (G_listenSock_);
            G_listenSock_ = INVALID_SOCKET;
            IB_LOG_ERROR("setsockopt error err:", (int)*err);
            fprintf(stderr,"fe_net_init setsockopt error%d\n", (int)*err);
            IB_EXIT(__func__,err);
            return(NET_FAILED);
        }

        memset ((void *) &addr, 0, sizeof(addr));
        addr.sin6_family = AF_INET6;
        addr.sin6_flowinfo = 0;
        addr.sin6_port = htons ((short) port);
        addr.sin6_addr = in6addr_any;

        if (BIND (G_listenSock_, (struct sockaddr*)&addr, sizeof(addr))) {
            SET_ERROR (err, NET_INTERNAL_ERR);
            CloseSock (G_listenSock_);
            G_listenSock_ = INVALID_SOCKET;
            IB_LOG_ERROR("error binding socket err:", (int)*err);
            fprintf(stderr,"\nNetInit error binding socket %d\n", (int)*err);
            IB_EXIT(__func__,err);
            return(NET_FAILED);
        }

        if (LISTEN (G_listenSock_, CONNECTION_BACKLOG)) {
            SET_ERROR (err, NET_INTERNAL_ERR);
            CloseSock (G_listenSock_);
            G_listenSock_ = INVALID_SOCKET;
            fprintf(stderr,"\nNetInit error listening on socket %d\n", 
            (int)*err);
            IB_LOG_ERROR("error listening on socket err:", (int)*err);
            IB_EXIT(__func__,err);
            return(NET_FAILED);
        }

        if (fe_config.SslSecurityEnabled) {
            /*
             * if the FE is a thread of the Unified SM, then the SM will handle
             * initialization of the SSL/TLS network security interface. 
             */ 
            if (!fe_is_thread()) {
                if (if3_ssl_init(NULL)) {
                    SET_ERROR (err, NET_INTERNAL_ERR);
                    CloseSock (G_listenSock_);
                    G_listenSock_ = INVALID_SOCKET;
                    fprintf(stderr,"\nfe_net_init error setting SSL/TLS on socket %d\n", (int)*err);
                    IB_LOG_ERROR("error setting SSL/TLS on socket err:", (int)*err);
                    IB_EXIT(__func__,err);
                    return(NET_FAILED);
                }
            }

            /* get a context for the SSL/TLS connection */
            if (!(G_sslContext_ = if3_ssl_srvr_open(fe_config.SslSecurityDir, 
                                                    fe_config.SslSecurityFmCertificate, 
                                                    fe_config.SslSecurityFmPrivateKey, 
                                                    fe_config.SslSecurityFmCaCertificate, 
                                                    fe_config.SslSecurityFmCertChainDepth, 
                                                    fe_config.SslSecurityFmDHParameters,
                                                    fe_config.SslSecurityFmCaCRLEnabled,
                                                    fe_config.SslSecurityFmCaCRL))) {
                SET_ERROR(err, NET_INTERNAL_ERR); 
                CloseSock(G_listenSock_); 
                G_listenSock_ = INVALID_SOCKET; 
                fprintf(stderr, "\nfe_net_init error getting SSL/TLS context on socket %d\n", (int)*err); 
                IB_LOG_ERROR("error getting SSL/TLS context on socket err:", (int)*err); 
                IB_EXIT(__func__, err); 
                return (NET_FAILED);
            }
        }
    } else {
        G_listenSock_ = INVALID_SOCKET;
    }

    G_numConnections_ = 0;
    G_connections_ = NULL;
    G_initted_ = 1;

    IB_LOG_VERBOSE0("Net initialized");

    IB_EXIT(__func__,err);
    return rc;
}


int fe_net_disconnect(NetConnection *conn, NetError *err)
{
    NetBlob *blob;
    int     nr, ns;
    int     rc      = NET_OK;

    IB_ENTER(__func__,conn,0,0,0);
    if (conn->sock == INVALID_SOCKET) {
        SET_ERROR (err, NET_NOT_CONNECTED);
        IB_LOG_ERROR("disconnect error err:",(int)*err);
        IB_EXIT(__func__,err);
        return(NET_FAILED);
    }

    CloseSock (conn->sock);
    conn->sock = INVALID_SOCKET;

    /* Remove the connection from the list  */
    RemoveFromConnectionList (conn);

    /*
     * Delete all enqueued blobs
     */
    ns = 0;
    while (!fe_net_queue_empty (&conn->sendQueue)) {
        blob = fe_net_dequeue_blob (&conn->sendQueue);
        if (blob) fe_net_free_blob (blob);
        ++ns;
    }

    nr = 0;
    while (!fe_net_queue_empty (&conn->recvQueue)) {
        blob = fe_net_dequeue_blob (&conn->recvQueue);
        if (blob) fe_net_free_blob (blob);
        ++nr;
    }

    if (conn->blobInProgress != NULL) {
        fe_net_free_blob(conn->blobInProgress);
    }

    IB_LOG_VERBOSE("closed connection", conn->id);

    IB_EXIT(__func__,err);
    return rc;
}

int fe_net_send(NetConnection * conn, char *data, int len, NetError * err)
{
    int     rc      = NET_OK;
    int     magic;
    int     totLen;
    NetBlob *blob;

    IB_ENTER(__func__,conn,data,len,0);

    if (!G_initted_) {
        SET_ERROR (err, NET_NOT_INITIALIZED);
        IB_EXIT(__func__,err);
        IB_LOG_ERROR("send err:",(int)*err);
        return NET_FAILED;
    }

    if (conn == NULL) {
        SET_ERROR (err, NET_NO_ERROR);
        IB_LOG_ERROR("connection is NULL err:",(int)*err);
        IB_EXIT(__func__,err);
        return NET_FAILED;
    }

    if (conn->sock == INVALID_SOCKET) {
        SET_ERROR (err, NET_NOT_CONNECTED);
        IB_LOG_ERROR("socket is invalid err:",(int)*err);
        IB_EXIT(__func__,err);
        return NET_FAILED;
    }

    /*
     * Copy the given data, prepended by a magic # and the tot msg len,
     * into the blob.
     */
    totLen = len + 2 * sizeof (int);
    blob = fe_net_new_blob (totLen);
    if (blob == NULL || blob->data == NULL) {
        SET_ERROR (err, NET_NO_MEMORY);
        IB_LOG_ERROR("data is NULL err:",(int)*err);
        IB_EXIT(__func__,err);
        return NET_FAILED;
    }

    magic = ntoh32 (NET_MAGIC);
    memcpy ((void *) blob->data, (void *) &magic, sizeof (int));
    totLen = ntoh32 (totLen);
    memcpy ((void *) (blob->data + sizeof (int)), (void *) &totLen, 
        sizeof (int));
    memcpy ((void *) (blob->data + 2 * sizeof (int)), (void *) data, len);

    fe_net_enqueue_blob (&conn->sendQueue, blob);

    IB_LOG_VERBOSE("bytes sent ", len);
    IB_LOG_VERBOSE("Sent to connection ", conn->id);

    SET_ERROR (err, NET_NO_ERROR);

    IB_EXIT(__func__,err);
    return rc;
}

void fe_net_get_next_message(NetConnection * conn, char **data, int *len, NetError * err)
{
    NetBlob *blob;

    IB_ENTER(__func__,conn,data,len,0);

    SET_ERROR (err, NET_NO_ERROR);

    if (conn == NULL) {
        if (data) {
            *data = NULL;
        }
        if (len) {
            *len = 0;
        }
        IB_EXIT(__func__, err);
        return;
    }

    blob = fe_net_dequeue_blob (&conn->recvQueue);
    if (blob == NULL) {
        if (data) {
            *data = NULL;
        }
        if (len) {
            *len = 0;
        }
    }
    else {
        if (data) {
            *data = blob->data;
        }
        if (len) {
            *len = blob->len;
        }
        blob->data = NULL;    /* so fe_net_free_blob() won't free the data */
        fe_net_free_blob (blob);
    }

    IB_EXIT(__func__, err);
    return;
}

NetConnection* fe_net_process(int msecToWait, int blocking)
{
    int             n, nfds;
    NetConnection   *conn;
    NetConnection   *retval = NULL;
    FDSET_t          readfds, writefds, errorfds;
    int queuedData = 0, inprogressData=0;
    struct timeval  timeout;

    IB_ENTER(__func__,msecToWait,blocking,0,0);

    /*
     * Do a select on the listen socket (to catch new connections),
     * on all in-bound sockets, and on those out-bound sockets for
     * which we have traffic enqueued.  If blocking!=0, do the select
     * even if there's nothing to listen for (so we will wait msecToWait
     * always).
     */
    FD_ZERO (&errorfds);
    FD_ZERO (&readfds);
    FD_ZERO (&writefds);
    nfds = 0;

    if (G_listenSock_ != INVALID_SOCKET) {
        FDSET (G_listenSock_, &readfds);
        nfds = MAX (nfds, (int) G_listenSock_);
    }

    for (conn = G_connections_; conn; conn = conn->next) {
        FDSET (conn->sock, &readfds);
        nfds = MAX (nfds, (int) conn->sock);
        if (!fe_net_queue_empty (&conn->sendQueue)) {
            queuedData++;
            nfds = MAX (nfds, (int) conn->sock);
            FDSET (conn->sock, &writefds);
        }
        /*
         * The OpenSSL interface does not reliably support the return of
         * correct results from the standard socket select() routine, so a
         * flag must be used to indicate that outstanding data is on the
         * socket ready to be read. 
         */ 
        if (conn->blobInProgress) {
            inprogressData++;
        }
    }
    if ((nfds == 0) && !blocking) {
        IB_EXIT(__func__, retval);
        return(NULL);
    }
    ++nfds;

    if (msecToWait < 0) {
        msecToWait = 0;
    }
    timeout.tv_sec = msecToWait / 1000;
    timeout.tv_usec = (msecToWait % 1000) * 1000;
    n = SELECT (nfds, &readfds, &writefds, &errorfds, &timeout);
    if (n == SOCKET_ERROR) {
        IB_EXIT(__func__, retval);
        return(NULL);
    }
    if (n == 0 && !inprogressData) {
        IB_EXIT(__func__, retval);
        return(NULL);
    }
    if (G_listenSock_ != INVALID_SOCKET) {
        if (FD_ISSET (G_listenSock_, &readfds)) {
#if defined(__VXWORKS__)
			if (G_connections_ != NULL)
				vs_thread_sleep(VTIMER_1S / 100);
#endif
            retval = AcceptConnection();
        }
    }

    /*
     * The OpenSSL interface does not reliably support the return of
     * correct results from the standard socket fd_isset() routine, so a
     * flag must be used to indicate that outstanding inbound/outbound
     * data is ready to be processed on the socket. 
     */ 
    for (conn = G_connections_; conn; conn = conn->next) {
        if (FD_ISSET (conn->sock, &readfds) || conn->blobInProgress) {
            if (ReadFromSocket (conn) == NET_FAILED) {
                conn->err = 1;
                continue;     /* don't bother trying to write */
            }
        }
        if (FD_ISSET (conn->sock, &writefds) || !fe_net_queue_empty (&conn->sendQueue)) {
#ifdef RFR
#endif
            if (WriteToSocket (conn) == NET_FAILED) {
                conn->err = 1;
            }
        }
    }

    /*
     * Handle errs here so we don't hassle with removing from 
     * conn list while iterating
     */
    conn = G_connections_;
    while (conn) {
        if (conn->err) {
            HandleReadWriteError (conn);
            conn = G_connections_;    /* start over */
        }
        else {
            conn = conn->next;
        }
    }

    IB_EXIT(__func__, retval);
    return retval;
}

static int ReadFromSocket(NetConnection *conn) {
    int     bytesRead;
    NetBlob *blob;
    int     rc      = NET_OK;

    IB_ENTER(__func__,conn,0,0,0);

    /*
     * If we're in the middle of a message, pick up where we left off.
     * Otherwise, start reading a new message.
     */
    if (conn->blobInProgress == NULL) {
        blob = fe_net_new_blob (0);
        if (!blob) {
            IB_LOG_ERROR0("Failed to allocate memory.");
            return NET_NO_MEMORY;
        }
        blob->data = NULL;    /* we haven't read the msg size yet */
        blob->curPtr = (char *) blob->magic;
        blob->bytesLeft = 2 * sizeof (int);
        conn->blobInProgress = blob;
    }
    else {
        blob = conn->blobInProgress;
    }

    if (fe_config.SslSecurityEnabled) {
        bytesRead = if3_ssl_read(conn->sslSession, (uint8_t *)blob->curPtr, blob->bytesLeft);
    } else {
        bytesRead = RECV (conn->sock, blob->curPtr, blob->bytesLeft, 0);
    }

    if (bytesRead == 0) {               /* graceful shutdown */
        IB_LOG_VERBOSE("conn shutdown gracefully ", conn->id);
        IB_EXIT(__func__, rc);
        return(NET_FAILED);
    }
    else if (bytesRead == SOCKET_ERROR) {
        IB_LOG_VERBOSE("error ", errno);
        IB_LOG_VERBOSE("bytes Read", bytesRead);
        IB_LOG_VERBOSE("connection", conn->id);
        IB_EXIT(__func__, NET_FAILED);
        IB_LOG_ERROR0("socket error");
        return(NET_FAILED);
    }
    else {
        if (bytesRead < blob->bytesLeft) {           /* still more to read */
            rc = fe_net_adjust_blob_curptr (blob, bytesRead);

            if (rc) {
                /* Disconnect and callback FE */
                IB_LOG_ERROR0("Error with data stream");
                IB_EXIT(__func__, rc);
                return(NET_FAILED);
            }

            IB_LOG_VERBOSE("connection ", conn->id);
            IB_LOG_VERBOSE("bytes read ", bytesRead);
            IB_LOG_VERBOSE("bytes to go ", blob->bytesLeft);
            IB_EXIT(__func__, rc);
            return(NET_OK);
        }
        else {
            if (blob->data == NULL) {       /* NULL means we just finished
                         reading msg size. If we
                         didn't get the magic, 
                         DISCONNECT this connection */

                if (ntohl (blob->magic[0]) != NET_MAGIC) {
                    /* Disconnect and callback FE */
                    IB_LOG_ERROR("protocol is incorrect:", blob->magic[0]);
                    IB_EXIT(__func__, NET_FAILED);
                    return(NET_FAILED);
                }
                blob->len = ntohl (blob->magic[1]) - 2 * sizeof (int);

                if (blob->len < MIN_PACKET_SIZE || blob->len > sizeof(OOBPacket)) {
                    IB_LOG_WARN("packet size is incorrect len:", blob->len);
                    IB_EXIT(__func__, rc);
                    return(NET_FAILED);
                }

                rc = vs_pool_alloc(&fe_pool,blob->len,(void *)&blob->data);

                if (rc != VSTATUS_OK) {
                    /* No memory! Bail out and disconnect, since we have to 
                    lose this msg */
                    IB_LOG_ERROR0("Unable to allocate memory for blob");
                    IB_EXIT(__func__, rc);
                    return(NET_FAILED);
                }

                blob->curPtr = blob->data;
                blob->bytesLeft = blob->len;
                IB_LOG_VERBOSE("connection ", conn->id);
                IB_LOG_VERBOSE("bytes read ", bytesRead);
                IB_LOG_VERBOSE("Done reading size ", blob->len);
                IB_EXIT(__func__, rc);
                return(NET_OK);
            }
            else {  /* we just finished reading the user data - enqueue blob */

                /*  Check the blob to reassure ourselves that all data 
                    has been read; if not close the connection      */
                if (!blob->data) {
                    IB_LOG_ERROR("FATAL: blob->data is not empty:", (nint)blob->data);
                    IB_EXIT(__func__, NET_FAILED);
                    return(NET_FAILED);
                }

                if (blob->len <= 0) {
                    IB_LOG_ERROR("FATAL: blob->len:", blob->len);
                    IB_EXIT(__func__, NET_FAILED);
                    return(NET_FAILED);
                }

                if ((blob->curPtr + bytesRead) != (blob->data + blob->len)) {
                    IB_LOG_ERROR("FATAL: blob->curPtr + bytes read = ", (nint)blob->curPtr + bytesRead);
                    IB_LOG_ERROR("FATAL: blob->data + blob->len = ", (nint)blob->data + blob->len);
                    IB_EXIT(__func__, rc);
                    return(NET_FAILED);
                }

                blob->bytesLeft = 0;
                blob->curPtr = NULL;
                fe_net_enqueue_blob (&conn->recvQueue, blob);
                conn->blobInProgress = NULL;
                IB_LOG_VERBOSE("connection ", conn->id);
                IB_LOG_VERBOSE("bytes read ", bytesRead);
                IB_LOG_VERBOSE("Done reading message of size ", blob->len);
                IB_EXIT(__func__, rc);
                return(NET_OK);
            }
        }
    }

    IB_EXIT(__func__, rc);
    return rc;
}

static int WriteToSocket(NetConnection * conn) {
    int     bytesSent;
    int     rc          = NET_OK;
    NetBlob *blob;
#ifdef TEST_FRAG
    int xxx;
#endif

    IB_ENTER(__func__,conn->id,0,0,0);

    /* Check to see that the sendQueue is not 0.  If so, close the connection
       and continue on  */
    if (fe_net_queue_empty(&conn->sendQueue) != 0) {
        IB_LOG_ERRORX("Queue is empty", (nint)&conn->sendQueue);
        IB_EXIT(__func__, rc);
        return(NET_FAILED);
    }

    if (fe_net_queue_empty(&conn->sendQueue)) {
        IB_EXIT(__func__, rc);
        return(NET_OK);
    }

    blob = fe_net_peek_blob (&conn->sendQueue);

    /*
     * #define TEST if you want to stress test message fragmentation.
     * Leave undefined for release build.
     */
#ifdef TEST_FRAG
    xxx = blob->bytesLeft / 2;
    if (xxx == 0) {
        xxx = blob->bytesLeft;
    }

    if (fe_config.SslSecurityEnabled) {
        bytesSent = if3_ssl_write(conn->sslSession, (uint8_t *)blob->curPtr, xxx);
    } else {
        bytesSent = send (conn->sock, blob->curPtr, xxx, 0);
    }
#else
    if (fe_config.SslSecurityEnabled) {
        bytesSent = if3_ssl_write(conn->sslSession, (uint8_t *)blob->curPtr, blob->bytesLeft);
    } else {
        bytesSent = SEND (conn->sock, blob->curPtr, blob->bytesLeft, 0);
    }
#endif

    IB_LOG_VERBOSE("Connection ", conn->id);
    IB_LOG_VERBOSE("bytes sent", bytesSent);

    if (bytesSent == SOCKET_ERROR) {
        /*
         * If we couldn't send because the send() would block, then just
         * return.  We'll try again next time.
         */
   //rfr   wtf now?
        if (errno == WSAEWOULDBLOCK) {
            rc = NET_OK;
        }
        else {
            rc = NET_FAILED;
        }
    }
    else {
        /*
         * If we sent the entire message, destroy it and go on to the next one
         * in the queue.  Otherwise, return; we'll continue were we left off
         * next time.
         */
        if (bytesSent == blob->bytesLeft) {
            blob = fe_net_dequeue_blob (&conn->sendQueue);
            if (blob) fe_net_free_blob (blob);
        }
        else {
            rc = fe_net_adjust_blob_curptr (blob, bytesSent);

            if (rc) {
                /* Disconnect and callback FE */
                IB_LOG_ERROR0("Error with data stream");
                IB_EXIT(__func__, rc);
                return(NET_FAILED);
            }
        }
    }

    IB_EXIT(__func__, rc);
    return rc;

}

static NetConnection* AcceptConnection() {
    struct sockaddr_storage addr;
    int addrSize = sizeof(addr);
    SOCKET_t sock;
    NetConnection *conn;
    void * sslSession = NULL;
#ifndef VXWORKS 
    int optval;
    const int USER_TIMEOUT = 300000; //5 minutes in milliseconds
    socklen_t optlen;
#endif

    IB_ENTER(__func__,0,0,0,0);

    sock = ACCEPT (G_listenSock_, 
                  (struct sockaddr*) &addr, 
                   (void *)&addrSize);
 
    if (sock == INVALID_SOCKET) {
        IB_LOG_ERROR0("Invalid IPv6 socket");
        IB_EXIT(__func__, 0);
        return(NULL) ;
    }

	if (G_connectCount >= 15) { /* 3 socket connections per FV */
		CloseSock(sock);
		IB_LOG_ERROR("At least 5 Fabric Viewer sessions are already connected",0);
		return NULL;
	}
 
#ifndef VXWORKS 
    /*
 *     replacing TCP_USER_TIMEOUT with 18 due to RHEL issue:
 *        https://bugzilla.redhat.com/show_bug.cgi?id=1219891
 *     we can revisit this once RHEL 7.2 is minimum supported version
 *     TBD replace string 18 below with TCP_USER_TIMEOUT */
    if ( setsockopt(sock, SOL_TCP, 18, &USER_TIMEOUT, sizeof(USER_TIMEOUT)) < 0 ) {  
		CloseSock(sock);
        IB_LOG_ERROR0("Cannot setsockopt TCP_USER_TIMEOUT");
        IB_EXIT(__func__, 0);
        return(NULL) ;
    }
#endif

    if (fe_config.SslSecurityEnabled) {
        // establish a SSL/TLS session with the Fabric Viewer client
        sslSession = if3_ssl_accept(G_sslContext_, sock); 
        if (!sslSession) {
    		CloseSock(sock);
    		return NULL;
        }
    }

    conn = NewConnection ();
    if (conn == NULL) {
		CloseSock(sock);
        if (sslSession) (void)if3_ssl_sess_close(sslSession);
        IB_LOG_ERROR("Invalid IPv6 connection sock:", sock);
        IB_EXIT(__func__, 0);
        return(NULL) ;
    }

#ifndef VXWORKS 
    /* TBD replace 18 with TCP_USER_TIMEOUT */
    if ( getsockopt(sock, SOL_TCP, 18, &optval, &optlen) < 0 ) {
		CloseSock(sock);
        IB_LOG_ERROR0("Cannot getsockopt: TCP_USER_TIMEOUT");
        IB_EXIT(__func__, 0);
        return(NULL) ;
    }
    if (optval!=USER_TIMEOUT) IB_LOG_WARN0("Cannot set TCP_USER_TIMEOUT");
#endif

    conn->sock = sock;
    conn->sslSession = sslSession;
    conn->id = G_numConnections_++;
    conn->addr = addr;

    AddToConnectionList (conn);

    IB_LOG_VERBOSE("Accepted IPv6 Connection ", conn->id);

    IB_EXIT(__func__, 0);
    return conn;
}


static NetConnection *NewConnection()  {
    NetConnection   *conn   = NULL;
    int             rc      = NET_OK; 
    
    IB_ENTER(__func__,0,0,0,0);

    rc = vs_pool_alloc(&fe_pool,sizeof(NetConnection),(void *)&conn);

    if (rc != VSTATUS_OK) {
        IB_LOG_ERROR0("Unable to allocate memory for a NetConnetion");
    }
    else {
        memset(conn, 0, sizeof(NetConnection));
        conn->sock = INVALID_SOCKET;
        fe_net_init_queue (&conn->sendQueue);
        fe_net_init_queue (&conn->recvQueue);
        conn->blobInProgress = NULL;
        conn->id = INVALID_ID;
        conn->err = NET_OK;
    }

    IB_EXIT(__func__, rc);
    return conn;
}

void fe_net_free_connection(NetConnection *conn) {
    IB_ENTER(__func__,0,0,0,0);

    if (conn != NULL) {
        if (conn->sock != INVALID_SOCKET) {
            fe_net_disconnect (conn, NULL);
        }
        if (conn->fe_in_buff) {
            vs_pool_free(&fe_pool, (void *)conn->fe_in_buff);
        }
        vs_pool_free(&fe_pool, (void *)conn);
    }

    IB_EXIT(__func__,0);
}

static void HandleReadWriteError(NetConnection *conn) { 
    IB_ENTER(__func__,conn->id,0,0,0);
    IB_LOG_VERBOSE("Read/Write error over connection ", conn->id);

    fe_net_disconnect(conn, NULL);

    if (DisconnectCallBack) {
        DisconnectCallBack(conn);
    }

    IB_EXIT(__func__,0);
}

static void AddToConnectionList(NetConnection *conn) {
    IB_ENTER(__func__,conn->id,0,0,0);

    if (G_connections_ == NULL) {
        G_connections_ = conn;
        conn->prev = NULL;
        conn->next = NULL;
    }
    else {
        conn->next = G_connections_;
        conn->prev = NULL;
        G_connections_->prev = conn;
        G_connections_ = conn;
    }
	++G_connectCount;

    IB_EXIT(__func__,0);
}

static void RemoveFromConnectionList(NetConnection *conn) {
    IB_ENTER(__func__,conn->id,0,0,0);

    if (conn->next) {
        conn->next->prev = conn->prev;
    }
    if (conn->prev) {
        conn->prev->next = conn->next;
    }
    if (conn->prev == NULL) {               /* conn is at head of list */
        G_connections_ = conn->next;
    }
	--G_connectCount;

    IB_EXIT(__func__,0);
}


int fe_net_shutdown(void) {
    NetConnection *conn;
    IB_ENTER(__func__,0,0,0,0);

    if (G_listenSock_ != INVALID_SOCKET) {
        CloseSock(G_listenSock_);
    }

    for (conn = G_connections_; conn; conn = conn->next) {
        CloseSock(conn->sock);
    }

#ifdef __VXWORKS__
    G_numConnections_ = 0;
    G_connections_ = NULL;
    G_initted_ = 0;
	G_listenSock_ = INVALID_SOCKET;
#endif

    IB_EXIT(__func__,0);
    return NET_NO_ERROR;
}
    
static void CloseSock(SOCKET_t sock) {
    IB_ENTER(__func__,sock,0,0,0);

#ifdef WINDOWS
    closesocket (sock);
#elif defined(__LINUX__)
    close (sock);
#elif defined(__VXWORKS__)
	close(sock);
#endif

    IB_EXIT(__func__,0);
}

void fe_net_inet_ntop(NetConnection *conn, char *str, int slen)
{
    struct sockaddr_in6 *padr;

    if (!conn || !str)
        return;

    padr = (struct sockaddr_in6 *) &conn->addr;
    memset(str, 0, slen);
    inet_ntop (AF_INET6, &padr->sin6_addr, str, slen);
}

