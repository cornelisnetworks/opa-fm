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

#ifndef HSM_CONFIG_SRVR_DATA
#define	HSM_CONFIG_SRVR_DATA

#include "hsm_config_srvr_api.h"

#define HSM_FM_SCK_PREFIX	"/var/lib/opa-fm/"
#define HSM_FM_SCK_SM		"sm_"
#define HSM_FM_SCK_PM		"pm_"
#define HSM_FM_SCK_FE		"fe_"

typedef struct	_fm_config_conx_hdl{
	unsigned				instance;
	unsigned				conx_mask; // Mask to store which services are currently connected.
	p_hsm_com_client_hdl_t	sm_hdl;
	p_hsm_com_client_hdl_t	pm_hdl;
	p_hsm_com_client_hdl_t	bm_hdl;
	p_hsm_com_client_hdl_t	fe_hdl;
	uint32_t				index_len;
	fm_error_map_t			error_map;
}fm_config_conx_hdl;

typedef struct fm_config_datagram_header_s{
	fm_msg_ret_code_t	ret_code;
	unsigned long		action;
	unsigned long		data_id;
	unsigned long		data_len;
}fm_config_datagram_header_t;

typedef struct fm_config_status_header_s{
	fm_msg_ret_code_t	ret_code;
	uint32_t			run_status;
	uint32_t			uptime;
	uint32_t			master;
}fm_config_status_header_t;

typedef struct fm_config_datagram_s{
	fm_config_datagram_header_t	header;
	char						data[1];   // Storage for the first data byte.
}fm_config_datagram_t;

#define FM_CONFIG_INTERMEDIATE_SIZE	10000
#define FM_CONFIG_INTERMEDIATE_BUFF	10032

typedef struct fm_config_interation_data_s {
	int					index;
	int					offset;
	int					more;
	int					start;
	int 				done;
	int					largeLength;
	char				*largeBuffer;
	int					intermediateLength;
	char				intermediateBuffer[FM_CONFIG_INTERMEDIATE_BUFF];
}fm_config_interation_data_t;



#endif /* HSM_CONFIG_SRVR_DATA */
