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

#ifdef __LINUX__
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#ifdef __VXWORKS__
#include  <sockLib.h>
#endif
#include <sys/un.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include "hsm_com_srvr_api.h"
#include "hsm_com_srvr_data.h"

int hsm_com_dbg = 0;

int unix_serv_listen(char *name) 
{
	int		fd,len;
	struct	sockaddr_un socketaddr;
	char *dname, *namecpy = strdup(name);

	//check strdup call completed successfully 
	if (!namecpy)
	{
		fd = -HSM_COM_NO_MEM;
		goto error;
	}
	
	unlink(name);
	
	//ensure socket directory exists
	dname = dirname(namecpy);
	if (mkdir(dname, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) < 0)
	{
		if (errno != EEXIST)
		{
			fd = -HSM_COM_PATH_ERR;
			goto error;
		}
	}

	if((fd = socket(AF_UNIX,SOCK_STREAM,0)) < 0)
	{
		fd = -HSM_COM_SOCK_ERR;
		goto error;
	}

	memset(&socketaddr,0,sizeof(socketaddr));

	socketaddr.sun_family = AF_UNIX;
	strcpy(socketaddr.sun_path,name);

	len = SUN_LEN(&socketaddr);

	if(bind(fd,(struct sockaddr*)&socketaddr,len) < 0)
	{
		close(fd);
		fd = -HSM_COM_BIND_ERR;
		goto error;
	}

	if(listen(fd,5) < 0)
	{
		close(fd);
		fd = -HSM_COM_LISTEN_ERR;
		goto error;
	}

	
error:
	if (namecpy) free(namecpy);
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

	if((clifd = accept(listenfd,(struct sockaddr *)&unix_addr,(socklen_t *)&len)) < 0)
	{
		return(-1);
	}

	len -= sizeof(unix_addr.sun_family);
	unix_addr.sun_path[len] = 0;

	if (stat(unix_addr.sun_path,&statbuf) < 0) 
	{
		close(clifd);
		return(-2);
	}

#ifdef S_ISSOCK
	if (S_ISSOCK(statbuf.st_mode) == 0) 
	{
		close(clifd);
		return(-3);
	}
#endif

	if ((statbuf.st_mode & (S_IRWXG | S_IRWXO)) ||
		(statbuf.st_mode & S_IRWXU) != S_IRWXU) 
	{
		close(clifd);
		return(-4);
	}

	staletime = time(NULL) - HSM_COM_STALE_CNX;
	if (statbuf.st_atime < staletime ||
		statbuf.st_ctime < staletime ||
		statbuf.st_mtime < staletime) 
	{
		close(clifd);
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
			if((clist[i].fd = unix_serv_accept(serv_fd, (void *)&clist[i].uid)) > 0){
				if(hsm_com_dbg)
					printf("Received new connection: fd: %d uid: %d\n",clist[i].fd,(int)clist[i].uid);
				return(clist[i].fd);
			}
		}
	}

	return(-1);
}



hsm_com_errno_t
send_response(hsm_com_client_t *client, hsm_com_msg_t *msg)
{
	int  len;
	
	if(!msg)
	{
		return HSM_COM_BAD;
	}

	len = sizeof(hsm_com_common_t) + msg->common.payload_len;

	if(client->state != HSM_COM_C_STATE_CT)
	{
		return HSM_COM_ERR_DISC;
	}
	/*
	if(len > HSM_COM_MAX_DATA)
	{
		// Data exceeds our maximum packet size...
		return HSM_COM_BAD;
	}
	*/

	if(hsm_com_dbg)
		printf("responding with %d bytes\n",len);

	if(write(client->fd, msg, len) != len)
	{
		return HSM_COM_ERR_DISC;
	}

	return HSM_COM_OK;

}

void 
dump_request(hsm_com_common_t *header)
{
	printf("DUMP:\n");
	printf("\tCMD: %lu\n",header->cmd);
	printf("\tLEN: %lu\n",header->payload_len);
	printf("\tRSP: %d\n",header->resp_code);
	printf("\tVER: %d\n",header->ver);
	printf("\tTID: %lu\n",header->trans_id);
	printf("\n\n");

}


hsm_com_errno_t
process_request(char *buf, int buflen, hsm_com_callback_func cb, hsm_com_client_t *client)
{
	hsm_com_msg_t 		msg;
	hsm_com_con_data_t	*con_data;
	hsm_com_errno_t		res = HSM_COM_OK;
	hsm_com_common_t	header;
	hsm_com_datagram_t	send_datagram;
	hsm_com_msg_t		*data_msg;


	if(buflen < sizeof(header)){
		return HSM_COM_ERR_LEN;
	}

	memcpy(&header,buf,sizeof(header));

	if(hsm_com_dbg)
		dump_request(&header);

	if(header.ver != HSM_COM_VER)
	{
		return HSM_COM_ERR_VERSION;
	}



	switch(header.cmd){
		case HSM_COM_CMD_CONN:
			if(hsm_com_dbg)
				printf("Processing connect command\n");
			con_data = (hsm_com_con_data_t*)buf;

			if(con_data->header.payload_len != (sizeof(con_data->key))){
				con_data->header.resp_code = HSM_COM_RESP_INV_LEN;
				con_data->header.payload_len = 0;
				// GOTO: Send response.
			}


			if(con_data->key != HSM_COM_KEY)
			{
				// Drop client...
				return HSM_COM_ERR_DISC;
			}
			else
			{
				client->state = HSM_COM_C_STATE_CT;
				con_data->header.resp_code = HSM_COM_RESP_OK;
				if((res = send_response(client, (hsm_com_msg_t*)con_data)) != HSM_COM_OK)
				{
					if(hsm_com_dbg)
						printf("Failed to send connect resp, code: %d\n", res);
				}
			}

			break;
		case HSM_COM_CMD_DISC:
			if(client->state != HSM_COM_C_STATE_CT){
				return HSM_COM_ERR_DISC;
			}

			// Send the response.
			msg.common.resp_code = HSM_COM_RESP_OK;
			msg.common.payload_len = 0;

			if((res = send_response(client, &msg)) != HSM_COM_OK)
			{
				if(hsm_com_dbg)
					printf("Failed to send disconnect resp, code: %d\n", res);
			}
			// Note: The client is supposed to be the one to close the socket once
			//		 it gets the response.

			break;
		case HSM_COM_CMD_PING:
			if(client->state != HSM_COM_C_STATE_CT){
				return HSM_COM_ERR_DISC;
			}

			// Send the response.
			msg.common.resp_code = HSM_COM_RESP_OK;
			msg.common.payload_len = 0;
			if((res = send_response(client, &msg)) != HSM_COM_OK)
			{
				if(hsm_com_dbg)
					printf("Failed to send ping resp, code: %d\n", res);
			}

			break;
		case HSM_COM_CMD_DATA:
			if(client->state != HSM_COM_C_STATE_CT){
				return HSM_COM_ERR_DISC;
			}
			data_msg = (hsm_com_msg_t*)buf;

			send_datagram.buf = &data_msg->data[0];
			send_datagram.data_len = data_msg->common.payload_len;
			// Note: Here we only allow the responses to be as big as what we received.
			send_datagram.buf_size = send_datagram.data_len;

			if(cb){
				if(cb(&send_datagram) == HSM_COM_OK){
					data_msg->common.payload_len = send_datagram.data_len;
					data_msg->common.resp_code = HSM_COM_RESP_OK;
					if((res = send_response(client, data_msg)) != HSM_COM_OK)
					{
						if(hsm_com_dbg)
							printf("Failed to send data resp, code: %d\n", res);
					}
				}
			}
				

			break;
	}


	return res;



}




hsm_com_errno_t
unix_sck_run_server(hsm_com_server_hdl_t *hdl)
{
	int					i, n, maxfd, listenfd, clifd, nread;
	fd_set				rset, allset;
	int					offset;
	hsm_com_errno_t		res;



	FD_ZERO(&allset);

	for(i=0;i<hdl->max_conx;i++){
		hdl->client_list[i].fd = -1;
	}

	if((listenfd = unix_serv_listen(hdl->path)) < 0)
	{
		if(hsm_com_dbg)
			printf("Error creating server socket\n");
		return(listenfd);
	}

	FD_SET(listenfd,&allset);

	maxfd = listenfd;

	for( ; ; ) 
	{
		rset = allset;
		if ( (n = select(maxfd + 1,&rset,NULL,NULL,NULL)) < 0)
			if(hsm_com_dbg)
				printf("unix_sck_run_server: Select error: %d \n",n);

		if (FD_ISSET(listenfd, &rset)) 
		{
			// Accept a new connection
			if((clifd = add_client(&hdl->client_list[0], HSM_COM_MAX_CLIENTS, listenfd)) < 0){
				if(hsm_com_dbg)
					printf("Failed to add new client\n");
			}else{

				FD_SET(clifd,&allset);

				if(clifd > maxfd)
					maxfd = clifd;

				continue;
			}
		}

		for(i=0;i<hdl->max_conx;i++)
		{
			if(hdl->client_list[i].fd < 0)
				continue;

			if(FD_ISSET(hdl->client_list[i].fd,&rset))
			{
				
				
				
				
				if ( (nread = unix_sck_read_data(hdl->client_list[i].fd,
												 &hdl->client_list[i].scr,
												 hdl->recv_buf, 
												 hdl->buf_len,
												 &offset)) == 0)
				{
					FD_CLR(hdl->client_list[i].fd,&allset);
					close(hdl->client_list[i].fd);
					hdl->client_list[i].fd = -1;
					hdl->client_list[i].uid = 0;
					hdl->client_list[i].scr.scratch_fill = 0;
				}
				else if(nread < 0)
				{
					if(hsm_com_dbg)
						printf("Skipping since we need more data\n");
				}
				else
				{
					if(hsm_com_dbg)
						printf("Got %d bytes (%d)\n",nread,offset);
					res = process_request(&hdl->recv_buf[offset], nread, 
										  hdl->callback, &hdl->client_list[i]);
					
					if(hsm_com_dbg)
						printf("Process result: %d\n",res);
					if(res == HSM_COM_ERR_DISC){
						// Disconnect the client...
						FD_CLR(hdl->client_list[i].fd,&allset);
						close(hdl->client_list[i].fd);
						hdl->client_list[i].fd = -1;
						hdl->client_list[i].uid = 0;
						hdl->client_list[i].scr.scratch_fill = 0;
					}
					
				}
				
			}

			
		}


	}




	return 0;
}
#endif

