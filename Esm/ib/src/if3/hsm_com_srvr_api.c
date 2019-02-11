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

#ifdef __LINUX__
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "hsm_com_srvr_api.h"
#include "hsm_com_srvr_data.h"
#include "cs_g.h"


   
   
hsm_com_errno_t
hcom_server_init
(
		OUT	p_hsm_com_server_hdl_t		*p_hdl,
	IN		char						*socket_path,
	IN		int							max_conx,		
	IN		int							max_data_len,
	IN		hsm_com_callback_func		callback
)
{
	hsm_com_server_hdl_t 	*hdl = NULL;
	int						buf_len;
	hsm_com_errno_t			res = HSM_COM_OK;
	int						i;

	if((strlen(socket_path) > (HSM_COM_SVR_MAX_PATH - 1)) ||
	   (strlen(socket_path) == 0)){
		res = HSM_COM_PATH_ERR;
		goto cleanup;
	}

	if((max_conx <= 0) || (max_conx > HSM_COM_MAX_CLIENTS)){
		res = HSM_COM_ERROR;
		goto cleanup;
	}
	
	// Allocate session structure
	if((hdl = calloc(1,sizeof(hsm_com_server_hdl_t))) == NULL){
		res = HSM_COM_NO_MEM;
		goto cleanup;
	}

	buf_len = (sizeof(hsm_com_con_data_t) > max_data_len) ? sizeof(hsm_com_con_data_t) : max_data_len;

	// Allocate buffers
	if((hdl->recv_buf = malloc(buf_len)) == NULL){
		res = HSM_COM_NO_MEM;
		goto cleanup;
	}

	if((hdl->send_buf = malloc(buf_len)) == NULL){
		res = HSM_COM_NO_MEM;
		goto cleanup;
	}

	if((hdl->client_list = calloc(max_conx, sizeof(hsm_com_client_t))) == NULL){
		res = HSM_COM_NO_MEM;
		goto cleanup;
	}

	hdl->max_conx = max_conx;
	hdl->conx_cnt = 0;

	for(i=0;i<max_conx;i++)
	{
		// Set up the scratch buffers for each client.
		if((hdl->client_list[i].scr.scratch = malloc(buf_len)) == NULL)
		{
			res = HSM_COM_NO_MEM;
			goto cleanup;
		}

		hdl->client_list[i].scr.scratch_len = buf_len;
		hdl->client_list[i].scr.scratch_fill = 0;
	}

	// Initialize session data
	hdl->buf_len = buf_len;
	StringCopy(hdl->path,socket_path, HSM_COM_SVR_MAX_PATH);
	hdl->callback = callback;


	hdl->server_state = HSM_COM_S_STATE_IN;

	*p_hdl = hdl;

	return res;


cleanup:
	if(hdl){
		if(hdl->recv_buf)
			free(hdl->recv_buf);
		if(hdl->send_buf)
			free(hdl->send_buf);
		if(hdl->client_list){
			for(i=0;i<hdl->max_conx;i++)
			{
				if(hdl->client_list[i].scr.scratch)
					free(hdl->client_list[i].scr.scratch);
			}
			free(hdl->client_list);
		}

		free(hdl);
	}

	return res;

}


hsm_com_errno_t
hcom_server_terminate
(
	IN p_hsm_com_server_hdl_t p_hdl		
)
{
	hsm_com_server_hdl_t 	*hdl = p_hdl;


	if(hdl){
		if(hdl->recv_buf)
			free(hdl->recv_buf);
		if(hdl->send_buf)
			free(hdl->send_buf);
		if(hdl->client_list)
			free(hdl->client_list);

		free(hdl);
	}

	return HSM_COM_OK;


}



hsm_com_errno_t
hcom_server_start
(
	IN p_hsm_com_server_hdl_t p_hdl		
)
{
	hsm_com_server_hdl_t 	*hdl = p_hdl;

	return unix_sck_run_server(hdl);

}


hsm_com_errno_t
hcom_server_stop
(
	IN p_hsm_com_server_hdl_t p_hdl		
)
{
	hsm_com_server_hdl_t 	*hdl = p_hdl;
	
	if(hdl->server_state == HSM_COM_S_STATE_RN)
	{
		hdl->server_state = HSM_COM_S_STATE_DN;
		if(hdl->server_fd)
		{
			// This should shut error the select and have the server pop out of its loop.
			close(hdl->server_fd);
		}else{
			hdl->server_state = HSM_COM_S_STATE_IN;
		}
	}
	else
	{
		return HSM_COM_ERROR;
	}

	return HSM_COM_OK;
}
#endif
