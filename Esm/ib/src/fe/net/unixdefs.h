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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
* 
* FILE NAME
*   unixdefs.h
*
* DESCRIPTION
*   Defines to allow compilation under UNIX
*
* DATA STRUCTURES
*   X      
*
* FUNCTIONS
*   X
*
* DEPENDENCIES
*   X
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   02/12/2000      Initial file for checkin to source control
* jrw   12/04/2001      Changes from code review 
*
***********************************************************************/
#ifndef _UNIX_DEFS_
#define _UNIX_DEFS_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>

#ifdef __LINUX__
typedef int SOCKET_t;
#endif

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#ifndef INADDR_NONE
#define INADDR_NONE -1
#endif
#define WSAEWOULDBLOCK EWOULDBLOCK
#define __declspec(x)

/* defines for maintaining cross-platform compatibility */
/* Thes map the socket calls to UNIX/BSD type Calls */
/*struct defs */
#define SOCKET_ADDR_IN  sockaddr_in 
#define SIN_FAMILY      sin_family 
#define SIN_PORT        sin_port 
#define IN_ADDR         sin_addr 
#define S_ADDR          s_addr
#define HOSTENTRY       hostent 
#define FDSET_t         fd_set 
#define HADDR           h_addr
#define HLEN            h_length

/*constant defs */


/* function defs */
#define SETSOCKOPT          setsockopt 
#define SOCKET(a,b,c)       socket(a,b,c)       
#define CONNECT             connect
#define RECV                recv
#define SEND                send
#define ACCEPT              accept
#define LISTEN              listen
#define FDSET               FD_SET 
#define SELECT(a,b,c,d,e)   select(a,b,c,d,e)
#define BIND(a,b,c)         bind(a,b,c)


#define TRUE 1
#define FALSE 0

#endif
