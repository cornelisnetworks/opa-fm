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

/*******************************************************************************
 * @file cs_sockwrap.c
 *
 * @brief
 * This file contains some socket creation routines for server and client
 * AF_UNIX type sockets.
 * The routines are supposed to be used on both Unix and Vxworks but the
 * structure of the socket addresss strings passed to the routines differs
 * between the operating systems.
 * In UNIX the structure of the strings is supposed to be generally of the
 * form "/var/tmp/<unique file name>" and in VXWORKS the string should
 * take the form "/comp/socket/0xNumber" where 0xNumber is a string
 * representation of a 16 bit number is hexadecimal form.
 *******************************************************************************
 */

#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include "cs_sockwrap.h"
#include <fcntl.h>
#include "cs_g.h"
#include <errno.h>
#include <string.h>

#define QLEN	10


#define	CLI_PERM	S_IRWXU			/* rwx for user only */

#define MAXWAIT  30


/*
 * @brief
 *
 * cs_local_comm_init --
 *
 * This function takes a string as an argument and creates a AF_UNIX
 * socket with the string supplied to it as the address in the socket
 * structure.
 * This routine is to be called by a program that needs a server socket.
 *
 * @param[in] name	pointer to unique socket address string
 *
 * @return *  socket descriptor on sucess or (-1) on failure
 *
 * NOTE: For VxWorks socket support, please refer to the WindRiver Network Stack
 * Programmer's Guide 6.9 (Section 5.3 - Working with Local Domain Sockets)
 *
 */
int
cs_local_comm_init(const char *name)
{
	int			fd;
	int			len;
	int			err;
	struct sockaddr_un	un;

	if (strlen(name) >= (sizeof(un.sun_path) )) {
		errno = ENAMETOOLONG;
		return(-1);
	}

	/* create a UNIX domain stream socket */
#ifdef __LINUX__
	if ((fd = socket(AF_UNIX, SOCK_SEQPACKET, 0)) < 0) {
#else
	if ((fd = socket(AF_LOCAL, SOCK_SEQPACKET, 0)) < 0) {
#endif
		return(fd);
	}

	unlink(name);	/* in case it already exists */

	/* fill in socket address structure */
	memset(&un, 0, sizeof(un));
	snprintf(un.sun_path, sizeof(un.sun_path), "%s", name);

#ifdef __LINUX__
	un.sun_family = AF_UNIX;
	len = offsetof(struct sockaddr_un, sun_path) + strlen(name);
#else
	un.sun_family = AF_LOCAL;
	len = sizeof(struct sockaddr_un);
	un.sun_len = len;
#endif

	/* bind the name */
	if (bind(fd, (struct sockaddr *)&un, len) < 0) {
		goto errout;
	}

	if (listen(fd, QLEN) < 0) {	/* tell kernel we're a server */
		goto errout;
	}
	return(fd);

errout:
	err = errno;
	close(fd);
	errno = err;
	return(-1);
}


/*
 * @brief
 *
 * cs_local_comm_accept --
 *
 * This function takes a socket descriptor as an argument and waits for
 * a client connectio to arrive on it. It accepts the new connection
 * checks for to see if the connection is via a socket and whether it
 * is writeable and returns with the new socket descriptor if everything
 * is successful
 *
 * @param[in] listenfd	socket to listen for a connection on
 *
 * @return    socket descriptor on sucess or (-1) on failure
 *
 */
int
cs_local_comm_accept(int listenfd)
{
	int				clifd;
	socklen_t			len;
	struct sockaddr_un		un;
	char				*name;
#ifdef __LINUX__
	int err;
	struct stat			statbuf;
#endif

	/* allocate enough space for longest name plus terminating null */
	if ((name = malloc(sizeof(un.sun_path) + 1)) == NULL)
		assert(name);
	len = sizeof(un);
	if ((clifd = accept(listenfd, (struct sockaddr *)&un, &len)) < 0) {
		free(name);
		return (clifd);
	}

	len -= offsetof(struct sockaddr_un, sun_path); /* len of pathname */
	StringCopy(name, un.sun_path, len);

#ifdef __LINUX__
	if (stat(name, &statbuf) < 0) {
		goto errout;
	}

	/* check path to make sure its a socket and writable */
	if (S_ISSOCK(statbuf.st_mode) == 0) {
		errno = ENOTSOCK;	/* not a socket */
		goto errout;
	}

	if ((statbuf.st_mode & (S_IRWXG | S_IRWXO)) ||
		(statbuf.st_mode & S_IRWXU) != S_IRWXU) {
		  errno = EPERM;
		  goto errout;
	}
#endif

	unlink(name);
	free(name);
	return(clifd);

#ifdef __LINUX__
errout:
	err = errno;
	close(clifd);
	free(name);
	errno = err;
	return(-1);
#endif
}




/*
 * @brief
 *
 * cs_local_comm_connect --
 *
 * This function takes two string arguments which
 * are AF_UNIX socket addresses creates a new socket
 * and connects to a specified server socket.
 *
 * @param[in] srvaddr   Server socket address string
 * @param[in] claddr    Client socket address string
 *
 * @return    socket descriptor on sucess or (-1) on failure
 *
 * NOTE: For VxWorks socket support, please refer to the WindRiver Network Stack
 * Programmer's Guide 6.9 (Section 5.3 - Working with Local Domain Sockets)
 *
 */
int
cs_local_comm_connect(const char *srvaddr, const char *claddr)
{
	int			fd, len, err;
	struct sockaddr_un	un, sun;
	int			do_unlink = 0;

	if ((strlen(srvaddr) >= (sizeof(un.sun_path)) ||
	     (strlen(claddr) >= sizeof(un.sun_path)))) {
		errno = ENAMETOOLONG;
		return(-1);
	}

	/* create a UNIX domain stream socket */
#ifdef __LINUX__
	if ((fd = socket(AF_UNIX, SOCK_SEQPACKET, 0)) < 0) {
#else
	if ((fd = socket(AF_LOCAL, SOCK_SEQPACKET, 0)) < 0) {
#endif
		return(-1);
	}

	/* fill socket address structure with our address */
	memset(&un, 0, sizeof(un));
	StringCopy(un.sun_path, claddr, sizeof(un.sun_path));

#ifdef __LINUX__
	un.sun_family = AF_UNIX;
	len = offsetof(struct sockaddr_un, sun_path) + strlen(un.sun_path);
	unlink(un.sun_path);		/* in case it already exists */
#else
	un.sun_family = AF_LOCAL;
	len = sizeof(struct sockaddr_un);
	un.sun_len = len;
#endif

	if (bind(fd, (struct sockaddr *)&un, len) < 0) {
		goto errout;
	}

#ifdef __LINUX__
	if (chmod(un.sun_path, CLI_PERM) < 0) {
		do_unlink = 1;
		goto errout;
	}
#endif

	/* fill socket address structure with server's address */
	memset(&sun, 0, sizeof(sun));
	StringCopy(sun.sun_path, srvaddr, sizeof(un.sun_path));

#ifdef __LINUX__
	sun.sun_family = AF_UNIX;
	len = offsetof(struct sockaddr_un, sun_path) + strlen(srvaddr);
#else
	sun.sun_family = AF_LOCAL;
	len = sizeof(struct sockaddr_un);
	sun.sun_len = len;
#endif
	if (connect(fd, (struct sockaddr *)&sun, len) < 0) {
		do_unlink = 1;
		goto errout;
	}
	return(fd);

errout:
	err = errno;
	close(fd);
	if (do_unlink)
		unlink(un.sun_path);
	errno = err;
	return(-1);
}

/**
 * @brief
 * cs_comm_connect --
 *
 * This function creates a socket with characteristics of the type requested
 * and then connects to the specified channel using that socket. An explicit
 * bind by the client is not done as the connect will bind a default address.
 * If the connection fails because of an unready or a busy server it retries
 * the connection in an exponential  form.
 *
 * @param[in]  domain   Socket domain
 * @param[in]  type     Socket type
 * @param[in]  protocol Usually 0
 * @param[in]  srv_addr     sockaddr structure of server
 * @param[in]  srv_len      len of server sockaddr
 *
 *
 * @return   socket fd is successful, -1 if not
 */
static int
cs_comm_connect(int domain, int type, int protocol,
		const struct sockaddr *srv_addr, socklen_t srv_len)
{
	int wait = 0;
	int fd = -1;

	if ((fd = socket(domain, type, protocol) <0)) {
	    return (-1);
	}

	while  (wait++ < MAXWAIT) {
		if (connect(fd, srv_addr, srv_len) == 0) {
			return (fd);
		}else {
			if (errno != ETIMEDOUT &&
			    errno != ECONNREFUSED) {
				goto exit_func;
			}
		}

		sleep(MAXWAIT - wait);
	}
exit_func:
	close(fd);
	return (-1);

}

/**
 * @brief
 * cs_tcp_comm_connect --
 *
 * This function initiates a tcp connect stream connection.
 * Used for demo purposes at this point.
 *
 * @param[in]  srv_addr     sockaddr structure of server
 * @param[in]  srv_len      len of server sockaddr
 *
 * @return   socket fd is successful, -1 if not
 */
int
cs_tcp_comm_connect(const struct sockaddr *srv_addr, socklen_t srv_len)
{
	return (cs_comm_connect(AF_INET, SOCK_STREAM, 0, srv_addr, srv_len));
}

/**
 * @brief
 * cs_udp_comm_connect --
 *
 * This function initiates a udp connect stream connection.
 * Used for demo purposes at this point.
 *
 * @param[in]  srv_addr     sockaddr structure of server
 * @param[in]  srv_len      len of server sockaddr
 *
 * @return   socket fd is successful, -1 if not
 */
int
cs_udp_comm_connect(const struct sockaddr *srv_addr, socklen_t srv_len)
{
	return (cs_comm_connect(AF_INET, SOCK_DGRAM, 0, srv_addr, srv_len));
}

/**
 * @brief
 *
 * cs_set_fd_non_block --
 *
 * This function sets the fd passed in to non-blocking.
 *
 * @params[in]  fd  file descriptor
 *
 * @return	1 on success, (-1) on failure
 *
 */
int
cs_set_fd_non_block(int fd)
{
	/* TBD on vxworks */
#ifdef __LINUX__
	int options;

	if ((options = fcntl(fd, F_GETFL)) < 0) {
		IB_LOG_ERROR_FMT(__func__, "fcntl get Error %s\n",
						strerror(errno));
		return (-1);
	}

	options |= O_NONBLOCK;

	if (fcntl(fd, F_SETFL, options) < 0) {
		IB_LOG_ERROR_FMT(__func__, "fcntl set Error %s\n",
						strerror(errno));
		return (-1);
	}
#endif

	return (1);
}


