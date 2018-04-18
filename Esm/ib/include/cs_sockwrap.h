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

/*******************************************************************************
 * @file cs_sockwrap.h
 *
 * @brief  
 * Header file contains declarations of socket creation routines for server
 * and client  AF_UNIX type sockets.
 * The routines are supposed to be used on both Unix and Vxworks but the
 * structure of the socket addresss strings passed to the routines differs
 * between the operating systems.
 * In UNIX the structure of the strings is supposed to be generally of the 
 * form "/var/tmp/<unique file name>" and in VXWORKS the string should
 * take the form "/comp/socket/0xNumber" where 0xNumber is a string
 * representation of a 16 bit number is hexadecimal form.
 *******************************************************************************
 */
#ifndef __CS_SOCKWRAP_H__

#define __CS_SOCKWRAP_H__

#include <sys/socket.h>

extern int cs_local_comm_init(const char *name);
extern int cs_local_comm_connect(const char *srvaddr, const char *claddr);
extern int cs_local_comm_accept(int listenfd);
extern int cs_udp_comm_connect(const struct sockaddr *srv_addr, 
			       socklen_t srv_len);
extern int cs_tcp_comm_connect(const struct sockaddr *srv_addr, 
			       socklen_t srv_len);
extern int cs_set_fd_non_block(int fd);

#endif /*__CS_SOCKWRAP_H__ */
