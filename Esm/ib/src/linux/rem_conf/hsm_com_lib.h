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

#ifndef HSM_COM_LIB
#define HSM_COM_LIB

// Maximum number of connections to each server (normally this will be 1 or 2)
#define HSM_COM_MAX_CLIENTS 5
#define HSM_COM_MAX_DATA	1024

typedef enum{
	HSM_COM_RES_OK = 0,
	HSM_COM_RES_BAD,
	HSM_COM_RES_ERR_PARAM,
	HSM_COM_RES_ERR_VERSION,
	HSM_COM_RES_ERR_LEN,
	HSM_COM_RES_ERR_CMD,
	HSM_COM_RES_CLI_DISC,  /* Return code to tell server to disconnect the client socket */
	HSM_COM_RES_CON_FAILED,
	HSM_COM_RES_TIMEOUT,
	HSM_COM_RES_RD_FAIL
}hsm_err_t;

typedef enum{
	HSM_COM_STATE_UN = 0, /* Client connected but has not sent a connect packet yet */
	HSM_COM_STATE_IN,	  /* Client initialized */
	HSM_COM_STATE_DN	  /* Disconnect requested, but socket not yet closed */
}hsm_com_state_t;
	
 
typedef struct hsm_com_client_s{
	int 			fd;
	int 			uid;
	hsm_com_state_t state;
}hsm_com_client_t;

typedef int (*hsm_com_callback_fun)(char *buf, int buf_len, char *resp_buf, int *resp_len);

int 
unix_serv_listen(char *name);

int
unix_serv_accept(int listenfd, uid_t *uidptr);

int
unix_client_connect(const char *server_name, const char *client_name);

hsm_err_t
unix_client_disconnect(int sock_fd);

int
unix_sck_run_server(char *path);

int
unix_sck_send_msg(int fd, char *snd_buf, int snd_len, char *rcv_buf, int rcv_len, int timeout);

#define HSM_COM_CMD_CONN	1
#define HSM_COM_CMD_DISC	2
#define HSM_COM_CMD_PING	3	
#define HSM_COM_CMD_DATA	4

#define HSM_COM_RESP_OK			1
#define HSM_COM_RESP_ERR		2
#define HSM_COM_RESP_INV_LEN	3

#define HSM_COM_VER 1 /* Note: client and server versions must match. This is always validated */
#define HSM_COM_KEY 0x00066A8239482938ull /* Used to verify that the client code was built with same lib */

typedef struct hsm_com_common_s{
	unsigned char	ver;
	unsigned char	type;
	unsigned char	cmd;
	unsigned char	resp_code;
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
	char				data[HSM_COM_MAX_DATA];
}hsm_com_msg_t;



hsm_err_t
unix_sck_send_conn(int fd, int timeout);





#endif /*HSM_COM_LIB*/
