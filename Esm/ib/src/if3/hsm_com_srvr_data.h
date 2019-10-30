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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#ifndef HSM_COM_SRVR_DATA
#define HSM_COM_SRVR_DATA

#include "hsm_com_srvr_api.h"

#define HSM_COM_SVR_MAX_PATH 255
#define HSM_COM_MAX_CLIENTS	 32

#define HSM_COM_STALE_CNX	30 // (SECONDS) Disregard connection requests older then this

#define HSM_COM_VER 1 /* Note: client and server versions must match. This is always validated */
#define HSM_COM_KEY 0x00066A8239482938ull /* Used to verify that the client code was built with same lib */

typedef enum{
	HSM_COM_C_STATE_UN = 0, /* Client connected but has not sent a connect packet yet */
	HSM_COM_C_STATE_IN,	  /* Client initialized */
	HSM_COM_C_STATE_CT,	  /* Client connected */
	HSM_COM_C_STATE_DN	  /* Disconnect requested, but socket not yet closed */
}hsm_com_state_t;

typedef enum{
	HSM_COM_S_STATE_UN = 0, /* Server uninitialized */
	HSM_COM_S_STATE_IN,	  /* Server initialized (not running) */
	HSM_COM_S_STATE_RN,	  /* Running */
	HSM_COM_S_STATE_DN,	  /* Shutting down initiated */
	HSM_COM_S_STATE_ER	  /* Error state */
}hsm_com_srv_state_t;


#define HSM_COM_CMD_CONN	1
#define HSM_COM_CMD_DISC	2
#define HSM_COM_CMD_PING	3	
#define HSM_COM_CMD_DATA	4

#define HSM_COM_RESP_OK			1
#define HSM_COM_RESP_ERR		2
#define HSM_COM_RESP_INV_LEN	3

typedef struct hsm_com_scratch_s{
	char			*scratch;
	int				scratch_len;
	int				scratch_fill;
}hsm_com_scratch_t;


typedef struct hsm_com_client_s{
	int 				fd;
	int 				uid;
	hsm_com_state_t 	state;
	hsm_com_scratch_t	scr;
}hsm_com_client_t;


typedef struct _hsm_com_server_hdl{	
	char						path[HSM_COM_SVR_MAX_PATH];
	char						*recv_buf;
	char						*send_buf;
	int							buf_len;
	int							max_conx;
	int							conx_cnt;
	hsm_com_client_t			*client_list;
	hsm_com_srv_state_t			server_state;
	unsigned long long			server_pid;
	int							server_fd;
	hsm_com_callback_func		callback;
}hsm_com_server_hdl_t;


typedef struct hsm_com_client_strsvr_s{
	struct hsm_com_client_strsvr_s *next;
	hsm_com_server_hdl_t info;
}hsm_com_client_strsvr_t;


typedef struct _hsm_com_client_hdl{	
	char					s_path[HSM_COM_SVR_MAX_PATH];
	char					c_path[HSM_COM_SVR_MAX_PATH];
	char					*recv_buf;
	char					*send_buf;
	int						buf_len;
	int						client_state;
	int						client_fd;
	hsm_com_scratch_t		scr;
	hsm_com_client_strsvr_t	*stream_list;
	int						trans_id;
}hsm_com_client_hdl_t;

typedef struct hsm_com_common_s{
	unsigned char	ver;
	unsigned char	resp_code;
	unsigned long	cmd;
	unsigned long	trans_id;
	unsigned long	payload_len;
}hsm_com_common_t;

typedef struct hsm_com_con_data_s{
	hsm_com_common_t	header;
	unsigned long long	key;
}hsm_com_con_data_t;

typedef struct hsm_com_discon_data_s{
	hsm_com_common_t	header;
}hsm_com_discon_data_t;

typedef struct hsm_com_ping_data_s{
	hsm_com_common_t	header;
}hsm_com_ping_data_t;


typedef struct hsm_com_msg_s{
	hsm_com_common_t	common;
	char				data[1]; // First byte of the data section.
}hsm_com_msg_t;

hsm_com_errno_t
unix_sck_run_server(hsm_com_server_hdl_t *hdl);

int
unix_sck_read_data(int fd, hsm_com_scratch_t *scratch, char *buf, int buf_len, int *offset);

hsm_com_errno_t
unix_sck_send_ping(hsm_com_client_hdl_t *hdl, int timeout);

hsm_com_errno_t
unix_sck_send_data(hsm_com_client_hdl_t *hdl, int timeout, 
				   hsm_com_datagram_t *send, hsm_com_datagram_t *recv);


#endif
