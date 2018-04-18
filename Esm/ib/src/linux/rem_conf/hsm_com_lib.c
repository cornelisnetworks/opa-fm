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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "hsm_com_lib.h"

#define STALE 30 /* Number of seconds a conx request can hang around before we deem it stale */

int hsm_com_dbg = 1;

int unix_serv_listen(char *name) 
{
	int		fd,len;
	struct	sockaddr_un socketaddr;

	if((fd = socket(AF_UNIX,SOCK_STREAM,0)) < 0)
	{
		return(-1);
	}

	unlink(name);

	memset(&socketaddr,0,sizeof(socketaddr));

	socketaddr.sun_family = AF_UNIX;
	cs_strlcpy(socketaddr.sun_path,name, sizeof(socketaddr.sun_path));

	len = SUN_LEN(&socketaddr);

	if(bind(fd,(struct sockaddr*)&socketaddr,len) < 0)
	{
		return(-2);
	}

	if(listen(fd,5) < 0)
	{
		return(-3);
	}

	
	return fd;
}

int
unix_serv_accept(int listenfd, uid_t *uidptr)
{
	int					clifd, len;
	time_t				staletime;
	struct sockaddr_un	unix_addr;
	struct stat			statbuf;


	len = sizeof(unix_addr);

	if((clifd = accept(listenfd,(struct sockaddr *)&unix_addr,&len)) < 0)
	{
		return(-1);
	}

	len -= sizeof(unix_addr.sun_family);
	unix_addr.sun_path[len] = 0;

	if (stat(unix_addr.sun_path,&statbuf) < 0) 
	{
		return(-2);
	}

#ifdef S_ISSOCK
	if (S_ISSOCK(statbuf.st_mode) == 0) 
	{
		return(-3);
	}
#endif

	if ((statbuf.st_mode & (S_IRWXG | S_IRWXO)) ||
		(statbuf.st_mode & S_IRWXU) != S_IRWXU) 
	{
		return(-4);
	}

	staletime = time(NULL) - STALE;
	if (statbuf.st_atime < staletime ||
		statbuf.st_ctime < staletime ||
		statbuf.st_mtime < staletime) 
	{
		return(-5);
	}

	if(uidptr != NULL)
		*uidptr = statbuf.st_uid;

	unlink(unix_addr.sun_path);


	return clifd;

}



int
add_client(hsm_com_client_t *clist, int size, int serv_fd)
{
	int i;

	for(i=0;i<size;i++)
	{
		if(clist[i].fd < 0){
			if((clist[i].fd = unix_serv_accept(serv_fd, &clist[i].uid)) > 0){
				if(hsm_com_dbg)
					printf("Received new connection: fd: %d uid: %d\n",clist[i].fd,(int)clist[i].uid);
				return(clist[i].fd);
			}
		}
	}

	return(-1);
}



hsm_err_t
send_response(hsm_com_client_t *client, hsm_com_msg_t *msg)
{
	int  len;
	
	len = sizeof(hsm_com_common_t) + msg->common.payload_len;

	if(!msg)
	{
		return HSM_COM_RES_BAD;
	}

	if(client->state != HSM_COM_STATE_IN)
	{
		return HSM_COM_RES_CLI_DISC;
	}

	if(len > HSM_COM_MAX_DATA)
	{
		// Data exceeds our maximum packet size...
		return HSM_COM_RES_BAD;
	}

	if(write(client->fd, msg, len) != len)
	{
		return HSM_COM_RES_CLI_DISC;
	}

	return HSM_COM_RES_OK;

}




hsm_err_t
process_request(char *buf, int buflen, hsm_com_callback_fun cb, hsm_com_client_t *client)
{
	hsm_com_msg_t 		msg;
	hsm_com_con_data_t	*con_data;
	hsm_err_t			res = HSM_COM_RES_OK;


	if((buflen < sizeof(msg.common)) || (buflen > sizeof(msg))){
		return HSM_COM_RES_ERR_LEN;
	}

	memcpy(&msg,buf,sizeof(msg));

	if(msg.common.ver != HSM_COM_VER)
	{
		return HSM_COM_RES_ERR_VERSION;
	}

	switch(msg.common.cmd){
		case HSM_COM_CMD_CONN:
			if(msg.common.payload_len != (sizeof(*con_data) - sizeof(msg.common))){
				msg.common.resp_code = HSM_COM_RESP_INV_LEN;
			}

			con_data = (hsm_com_con_data_t*)&msg;

			if(con_data->key != HSM_COM_KEY)
			{
				// Drop client...
				return HSM_COM_RES_CLI_DISC;
			}
			else
			{
				client->state = HSM_COM_STATE_IN;
				msg.common.resp_code = HSM_COM_RESP_OK;
				msg.common.payload_len = 0;
				if((res = send_response(client, &msg)) != HSM_COM_RES_OK)
				{
					printf("Failed to send connect resp, code: %d\n", res);
				}
			}

			break;
		case HSM_COM_CMD_DISC:
			if(client->state != HSM_COM_STATE_IN){
				return HSM_COM_RES_CLI_DISC;
			}

			// Send the response.
			msg.common.resp_code = HSM_COM_RESP_OK;
			msg.common.payload_len = 0;

			if((res = send_response(client, &msg)) != HSM_COM_RES_OK)
			{
				printf("Failed to send disconnect resp, code: %d\n", res);
			}
			// Note: The client is supposed to be the one to close the socket once
			//		 it gets the response.

			break;
		case HSM_COM_CMD_PING:
			if(client->state != HSM_COM_STATE_IN){
				return HSM_COM_RES_CLI_DISC;
			}

			// Send the response.
			msg.common.resp_code = HSM_COM_RESP_OK;
			msg.common.payload_len = 0;
			if((res = send_response(client, &msg)) != HSM_COM_RES_OK)
			{
				printf("Failed to send ping resp, code: %d\n", res);
			}

			break;
		case HSM_COM_CMD_DATA:
			if(client->state != HSM_COM_STATE_IN){
				return HSM_COM_RES_CLI_DISC;
			}

			break;
	}


	return res;



}


int
unix_sck_run_server(char *path)
{
	char				buf[HSM_COM_MAX_DATA];
	int					i, n, maxfd, maxi, listenfd, clifd, nread;
	fd_set				rset, allset;
	hsm_com_client_t	c_list[HSM_COM_MAX_CLIENTS];

	FD_ZERO(&allset);

	for(i=0;i<HSM_COM_MAX_CLIENTS;i++){
		c_list[i].fd = -1;
	}

	if((listenfd = unix_serv_listen(path)) < 0)
	{
		printf("Error creating server socket\n");
		return(-1);
	}

	FD_SET(listenfd,&allset);

	maxfd = listenfd;
	maxi = -1;

	for( ; ; ) 
	{
		rset = allset;
		if ( (n = select(maxfd + 1,&rset,NULL,NULL,NULL)) < 0)
			printf("unix_sck_run_server: Select error: %d \n",n);

		if (FD_ISSET(listenfd, &rset)) 
		{
			// Accept a new connection
			if((clifd = add_client(&c_list[0], HSM_COM_MAX_CLIENTS, listenfd)) < 0){
				printf("Failed to add new client\n");
			}else{

				FD_SET(clifd,&allset);

				if(clifd > maxfd)
					maxfd = clifd;

				continue;
			}
		}

		for(i=0;i<HSM_COM_MAX_CLIENTS;i++)
		{
			if(c_list[i].fd < 0)
				continue;

			if(FD_ISSET(c_list[i].fd,&rset))
			{
				if ( (nread = read(c_list[i].fd, buf, HSM_COM_MAX_DATA)) < 0)
				{
									}
				else if(nread == 0)
				{
					FD_CLR(c_list[i].fd,&allset);
					close(c_list[i].fd);
					c_list[i].fd = -1;
					c_list[i].uid = 0;
				}
				else
				{
					printf("Got %d bytes\n",nread);
					process_request(buf, nread, NULL, &c_list[i]);
					//write(c_list[i].fd,buf,nread);
				}
			}

			
		}


	}




	return 0;
}

int
unix_sck_send_msg(int fd, char *snd_buf, int snd_len, char *rcv_buf, int rcv_len, int timeout)
{
	int					nread = 0;
	int					n;
	fd_set				rset;
	struct timeval		tm;

	write(fd,snd_buf,snd_len);

	tm.tv_sec = timeout;
	tm.tv_usec = 0;

	FD_ZERO(&rset);


	FD_SET(fd,&rset);

	if ( (n = select(fd + 1,&rset,NULL,NULL,&tm)) < 0)
		printf("unix_sck_send_msg: Select error: %d \n",n);

	if (FD_ISSET(fd, &rset)) 
	{
		if ( (nread = read(fd, rcv_buf, rcv_len)) < 0)
		{
			return -1;
		}
		else if(nread == 0)
		{
			return -1;
		}
		else
		{
			printf("Got %d bytes\n",nread);
		}
		
	}


	return nread;
}

hsm_err_t
unix_sck_send_conn(int fd, int timeout)
{
	int					nread,n;
	fd_set				rset;
	struct timeval		tm;
	hsm_com_con_data_t	msg;
	hsm_com_common_t	resp;

	memset(&msg,0,sizeof(msg));

	msg.header.cmd = HSM_COM_CMD_CONN;
	msg.header.trans_id = 1;
	msg.key = HSM_COM_KEY;
	msg.header.ver = HSM_COM_VER;

	write(fd,&msg,sizeof(msg));

	tm.tv_sec = timeout;
	tm.tv_usec = 0;

	FD_ZERO(&rset);


	FD_SET(fd,&rset);

	if ( (n = select(fd + 1,&rset,NULL,NULL,&tm)) < 0)
		printf("unix_sck_send_msg: Select error: %d \n",n);

	if (FD_ISSET(fd, &rset)) 
	{
		if ( (nread = read(fd, &resp, sizeof(resp))) < 0)
		{
			return HSM_COM_RES_TIMEOUT;
		}
		else if(nread == 0)
		{
			return HSM_COM_RES_RD_FAIL;
		}
		else
		{
			// Validate response is ok...
			if(nread == sizeof(resp)){
				if(resp.resp_code == HSM_COM_RESP_OK){
					return HSM_COM_RES_OK;
				}
			}else{
				printf("Num bytes is: %d, should be: %d\n",nread,sizeof(hsm_com_msg_t));
				return HSM_COM_RES_ERR_LEN;
			}
		}
		
	}


	return HSM_COM_RES_BAD;
}


hsm_err_t
unix_sck_send_disconnect(int fd, int timeout)
{
	int						nread,n;
	fd_set					rset;
	struct timeval			tm;
	hsm_com_discon_data_t	msg;
	hsm_com_discon_data_t	resp;

	memset(&msg,0,sizeof(msg));

	msg.header.cmd = HSM_COM_CMD_DISC;
	msg.header.trans_id = 1;
	msg.header.ver = HSM_COM_VER;

	write(fd,&msg,sizeof(msg));

	tm.tv_sec = timeout;
	tm.tv_usec = 0;

	FD_ZERO(&rset);


	FD_SET(fd,&rset);

	if ( (n = select(fd + 1,&rset,NULL,NULL,&tm)) < 0)
		printf("unix_sck_send_msg: Select error: %d \n",n);

	if (FD_ISSET(fd, &rset)) 
	{
		if ( (nread = read(fd, &resp, sizeof(resp))) < 0)
		{
			return HSM_COM_RES_TIMEOUT;
		}
		else if(nread == 0)
		{
			return HSM_COM_RES_RD_FAIL;
		}
		else
		{
			// Validate response is ok...
			if(nread == sizeof(resp)){
				if(resp.header.resp_code == HSM_COM_RESP_OK){
					return HSM_COM_RES_OK;
				}
			}else{
				printf("Num bytes is: %d, should be: %d\n",nread,sizeof(hsm_com_msg_t));
				return HSM_COM_RES_ERR_LEN;
			}
		}
		
	}


	return HSM_COM_RES_BAD;
}


hsm_err_t
unix_sck_send_ping(int fd, int timeout)
{
	int					nread,n;
	fd_set				rset;
	struct timeval		tm;
	hsm_com_ping_data_t	msg, resp;

	memset(&msg,0,sizeof(msg));

	msg.header.cmd = HSM_COM_CMD_PING;
	msg.header.trans_id = 1;
	msg.header.ver = HSM_COM_VER;

	write(fd,&msg,sizeof(msg));

	tm.tv_sec = timeout;
	tm.tv_usec = 0;

	FD_ZERO(&rset);


	FD_SET(fd,&rset);

	if ( (n = select(fd + 1,&rset,NULL,NULL,&tm)) < 0)
		printf("unix_sck_send_msg: Select error: %d \n",n);

	if (FD_ISSET(fd, &rset)) 
	{
		if ( (nread = read(fd, &resp, sizeof(resp))) < 0)
		{
			return HSM_COM_RES_TIMEOUT;
		}
		else if(nread == 0)
		{
			return HSM_COM_RES_RD_FAIL;
		}
		else
		{
			// Validate response is ok...
			if(nread == sizeof(resp)){
				if(resp.header.resp_code == HSM_COM_RESP_OK){
					return HSM_COM_RES_OK;
				}
			}else{
				printf("Num bytes is: %d, should be: %d\n",nread,sizeof(hsm_com_msg_t));
				return HSM_COM_RES_ERR_LEN;
			}
		}
		
	}


	return HSM_COM_RES_BAD;
}


int
unix_client_connect(const char *server_name, const char *client_name)
{
	int					fd, len;
	struct sockaddr_un	unix_addr;

	if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) 
	{
		return(-1);
	}

	memset(&unix_addr,0,sizeof(unix_addr));

	unix_addr.sun_family = AF_UNIX;
	
	if(strlen(client_name) >= sizeof(unix_addr.sun_path))
	{
		return(-2);
	}

	sprintf(unix_addr.sun_path,"%s",client_name);

	len = SUN_LEN(&unix_addr);

	unlink(unix_addr.sun_path);

	if(bind(fd, (struct sockaddr *)&unix_addr, len) < 0)
	{
		return(-3);
	}

	if(chmod(unix_addr.sun_path, S_IRWXU) < 0)
	{
		return(-4);
	}

	memset(&unix_addr,0,sizeof(unix_addr));

	unix_addr.sun_family = AF_UNIX;
	cs_strlcpy(unix_addr.sun_path,server_name, sizeof(unix_addr.sun_path));


	len = SUN_LEN(&unix_addr);

	if (connect(fd, (struct sockaddr *) &unix_addr, len) < 0) 
	{
		return(-5);
	}

	// Send connection data packet
	if(unix_sck_send_conn(fd, 2) != HSM_COM_RES_OK)
	{
		return(-6);
	}


	return fd;

}


hsm_err_t
unix_client_disconnect(int fd)
{
	// Send connection data packet
	if(unix_sck_send_disconnect(fd, 2) != HSM_COM_RES_OK)
	{
		return(-1);
	}

	close(fd);


	return HSM_COM_RES_OK;

}
