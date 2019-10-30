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

#ifndef HSM_COM_CLIENT_API
#define HSM_COM_CLIENT_API

#ifndef IN
#define IN  
#endif /* #ifndef IN */

#ifndef OUT
#define OUT
#endif /* #ifndef OUT */

#ifndef OPTIONAL
#define OPTIONAL
#endif /* #ifndef OPTIONAL */

#define HSM_COM_API_MAX_DATA	1024

			  
// C++ wrapper
#ifdef __cplusplus
extern "C"
{
#endif	/* __cplusplus */

// Error codes
typedef enum{
	HSM_COM_ERR_LEN,
	HSM_COM_ERR_VERSION,
	HSM_COM_ERR_DISC,
	HSM_COM_TEST = -1,
	HSM_COM_OK = 0,
	HSM_COM_ERROR = 1,
	HSM_COM_NO_RESOURCES = 2,
	HSM_COM_NO_MEM,
	HSM_COM_PATH_ERR,
	HSM_COM_BAD,
	HSM_COM_BIND_ERR,
	HSM_COM_SOCK_ERR,
	HSM_COM_CHMOD_ERR,
	HSM_COM_CONX_ERR,
	HSM_COM_SEND_ERR,
	HSM_COM_NOT_CONNECTED,
	HSM_COM_MAX_ERROR_NUM
}hsm_com_errno_t;


typedef enum{
	HSM_METH_GET = 1,
	HSM_METH_SET,
	HSM_METH_RSP,
}hsm_com_method_t;

// Session structures (opaque)
typedef struct _hsm_com_client_hdl	*p_hsm_com_client_hdl_t;
typedef struct _hsm_com_stream_hdl	*p_hsm_com_stream_hdl_t;

typedef struct hsm_com_datagram_s{
	int					data_len;
	int					buf_size;
	char				*buf;
}hsm_com_datagram_t;

// Server Data callback function
// Note: data is IN/OUT. 
typedef hsm_com_errno_t (*hsm_com_callback_func)(hsm_com_datagram_t *data);

// Server Error callback function
typedef void (*hsm_com_err_callback_func)(hsm_com_errno_t err);


hsm_com_errno_t
hcom_client_init
(
		OUT	p_hsm_com_client_hdl_t *p_hdl,
	IN		char					*server_path,
	IN		char					*client_path,
	IN		int						max_data_len
);

hsm_com_errno_t
hcom_client_connect
(
	IN p_hsm_com_client_hdl_t p_hdl		
);


hsm_com_errno_t
hcom_client_disconnect
(
	IN p_hsm_com_client_hdl_t p_hdl		
);



hsm_com_errno_t
hcom_client_send_ping
(
	IN	p_hsm_com_client_hdl_t	p_hdl,		
	IN	int						timeout_s
);



hsm_com_errno_t
hcom_client_send_data
(
	IN		p_hsm_com_client_hdl_t	p_hdl,
	IN		int						timeout_s,
	IN		hsm_com_datagram_t		*data,
		OUT	hsm_com_datagram_t		*res
);



hsm_com_errno_t
hcom_client_create_stream
(
		OUT	p_hsm_com_stream_hdl_t	*p_stream_hdl,
	IN		p_hsm_com_client_hdl_t	p_client_hdl,
	IN		char					*socket_path,
	IN		int						max_conx,		
	IN		int						max_data_len
);



hsm_com_errno_t
hcom_client_destroy_stream
(
	IN	p_hsm_com_stream_hdl_t	p_stream_hdl,
	IN	p_hsm_com_client_hdl_t	p_client_hdl
);



#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif /*HSM_COM_CLIENT_API*/

