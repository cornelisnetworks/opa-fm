/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
* 
* FILE NAME
*   net.h
*
* DESCRIPTION
*   Libnet protocol header file
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   02/12/2000      Initial file for checkin to source control
* jrw   11/26/2001      Code cleanup
* jrw   12/04/2001      Code changes from review 
*
***********************************************************************/

#ifndef LIBNET_H
#define LIBNET_H
/*
 * Everything here must compile under both Linux and Windows'95, so
 * we #define and typedef a lot of things in the following two .h files
 * accordingly.
 */
#ifdef __LINUX__
    #include <sys/types.h>
    #include "unixdefs.h"
    #include <netinet/tcp.h>
#elif __WINDOWS__
    #include "windowsdefs.h"
    #define WINDOWS
#endif

#ifdef __VXWORKS__

    #include <types/vxTypes.h>
    #include "vxworksdefs.h"

#endif

#include "iba/stl_mad_priv.h"
#include "iba/ib_generalServices.h"

#include "netblob.h"
#include "netqueue.h"
#include "cs_g.h"
#include "if3.h"

#if defined(__VXWORKS__) && !defined(INET6)
/*
 * RFC 2553: protocol-independent placeholder for socket addresses
 */

#define	_SS_MAXSIZE	128
#define	_SS_ALIGNSIZE	(sizeof(int64_t))
#define	_SS_PAD1SIZE	(_SS_ALIGNSIZE - sizeof(u_char) - sizeof(sa_family_t))
#define	_SS_PAD2SIZE	(_SS_MAXSIZE - sizeof(u_char) - sizeof(sa_family_t) - \
				_SS_PAD1SIZE - _SS_ALIGNSIZE)

#define	INET6_ADDRSTRLEN	46
#define	AF_INET6	        28

#define	IN6ADDR_ANY_INIT \
	{{{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }}}

typedef unsigned char u_int8_t;
typedef unsigned short int u_int16_t;
typedef unsigned int u_int32_t;
typedef unsigned long long u_int64_t;

typedef u_int8_t	__sa_family_t;
typedef __sa_family_t sa_family_t;


/*
 * Socket address structures for IPv6
 */
struct sockaddr_storage {
	u_char		ss_len;		/* address length */
	sa_family_t	ss_family;	/* address family */
	char		__ss_pad1[_SS_PAD1SIZE];
	int64_t		__ss_align;	/* force desired structure storage alignment */
	char		__ss_pad2[_SS_PAD2SIZE];
};

struct in6_addr {
	union {
		u_int8_t	__u6_addr8[16];
		u_int16_t	__u6_addr16[8];
		u_int32_t	__u6_addr32[4];
	} __u6_addr;			/* 128-bit IP6 address */
};

struct sockaddr_in6 {
	u_char	sin6_len;		/* length of this struct(sa_family_t)*/
	u_char	sin6_family;		/* AF_INET6 (sa_family_t) */
	u_int16_t	sin6_port;	/* Transport layer port # (in_port_t)*/
	u_int32_t	sin6_flowinfo;	/* IP6 flow information */
	struct	in6_addr	sin6_addr;	/* IP6 address */
	u_int32_t	sin6_scope_id;	/* intface scope id */
};
#endif

/*
 * A NetConnection encapsulates a logical network connection between
 * client and server.  More than one NetConnection can exist between
 * two given machines.  Here we store the blobs queued up for send but
 * not yet sent, and the blobs reeived and ready to be processed.
 */
struct NetConnection_t {
    SOCKET_t sock;
    NetQueue sendQueue;
    NetQueue recvQueue;
    NetBlob * blobInProgress;
    int id;                         /* session ID                           */
    int err;                        /* to indicate err on this connection   */
    struct sockaddr_storage addr;   /* IPv6 socket address of peer */
    struct NetConnection_t * prev;  /* maintain doubly-linked list of conns */
    struct NetConnection_t * next;
    uint8_t *fe_in_buff;            /* input buffer (cache) for connection  */
    uint32_t fe_nodes_len;          /* len of node data in cache            */
    void * sslSession;
};
typedef struct NetConnection_t NetConnection;

/**************************************************************************
* 		OOB PROTOCOL STRUCTURES
**************************************************************************/
typedef struct __OOBHeader {
    uint32_t        HeaderVersion;      /* Version of the FE protocol header */
    uint32_t        Length;             /* Length of the message data payload */
    uint32_t        Reserved[2];        /* Reserved */
} OOBHeader;

typedef struct _OOBPacket {
    OOBHeader Header;
    MAD_RMPP MadData;
} OOBPacket;

/*
 * Error defines
 */
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
typedef int NetError;

#define MIN_PACKET_SIZE (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER))

/*
 * Prototypes of externally callable Net functions
 */
/* yes, this is convoluted, but this has to compiler under cl AND gcc */
#ifdef WINDOWS
    #ifdef __cplusplus
        #define EXT extern "C"
    #else
        #define EXT
    #endif
#else
    #define EXT
#endif


EXT int fe_net_init(int port, NetError * err, void (*Callback)(NetConnection *));
EXT int fe_net_shutdown(void);
EXT NetConnection * fe_net_connect(char * hostname, int port, NetError * err);
EXT int fe_net_disconnect(NetConnection * conn, NetError * err);
EXT void fe_net_free_connection(NetConnection * conn);
EXT NetConnection * fe_net_process(int msecToWait, int blocking);
EXT int fe_net_send(NetConnection * conn, char * data, int len, NetError * err);
EXT void fe_net_get_next_message(NetConnection * conn, char ** data, int * len,
                            NetError * err);
void fe_net_inet_ntop(NetConnection *conn, char *str, int slen);
#endif
