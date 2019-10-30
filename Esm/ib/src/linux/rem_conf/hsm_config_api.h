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

#ifndef HSM_CONFIG_API
#define HSM_CONFIG_API

#include "hsm_com_api.h"

#ifndef IN
#define IN  
#endif /* #ifndef IN */

#ifndef OUT
#define OUT
#endif /* #ifndef OUT */

#ifndef OPTIONAL
#define OPTIONAL
#endif /* #ifndef OPTIONAL */



typedef struct _fm_config_conx_hdl	*p_fm_config_conx_hdlt;


typedef enum fm_mgr_type_s{
	FM_MGR_NONE = 0,
	FM_MGR_SM	= 0x0001,
	FM_MGR_PM	= 0x0002,
	FM_MGR_FE	= 0x0004,
	FM_MGR_BM	= 0x0008
}fm_mgr_type_t;

typedef enum{
	FM_CONF_ERR_LEN =  -4,
	FM_CONF_ERR_VERSION = -3,
	FM_CONF_ERR_DISC = -2,
	FM_CONF_TEST = -1,
	FM_CONF_OK = 0,
	FM_CONF_ERROR = 1,
	FM_CONF_NO_RESOURCES = 2,
	FM_CONF_NO_MEM,
	FM_CONF_PATH_ERR,
	FM_CONF_BAD,
	FM_CONF_BIND_ERR,
	FM_CONF_SOCK_ERR,
	FM_CONF_CHMOD_ERR,
	FM_CONF_CONX_ERR,
	FM_CONF_SEND_ERR,
	FM_CONF_INIT_ERR,
	FM_CONF_NO_RESP,
	FM_CONF_MAX_ERROR_NUM
}fm_mgr_config_errno_t;

typedef enum{
	FM_ACT_NONE	=	0,
	FM_ACT_GET,			// Get selected attributes
	FM_ACT_SET,			// Set appropriate attributes
	FM_ACT_RSP,			// Response
	FM_ACT_SUP_GET,		// Query which attributes are supported
	FM_ACT_SUP_SET		// Query which attributes are supported
}fm_mgr_action_t;


typedef enum{
	FM_RET_BAD_RET_LEN = -1,
	FM_RET_OK = 0,
	FM_RET_DT_NOT_SUPPORTED,	// Datatype is not supported
	FM_RET_ACT_NOT_SUPPORTED,	// Action is not supported for this datatype
	FM_RET_INVALID,				// Data is invalid.
	FM_RET_BAD_LEN,				// Data is an invalid length
	FM_RET_BUSY,				// Server busy, try again later.
	FM_RET_UNKNOWN_DT			// Data type is not recognized.
}fm_msg_ret_code_t;


typedef enum{
	FM_DT_NONE	=	0,
	FM_DT_COMMON,
	FM_DT_BM_CFG,
	FM_DT_PM_CFG,
	FM_DT_FE_CFG,
	FM_DT_SM_CFG,
	FM_DT_SM_PKEY,
	FM_DT_SM_MC
}fm_datatype_t;


#define CFG_COM_SEL_DEVICE		0x0001
#define CFG_COM_SEL_PORT		0x0002
#define CFG_COM_SEL_DEBUG		0x0004
#define CFG_COM_SEL_POOL_SIZE	0x0008
#define CFG_COM_SEL_NODAEMON	0x0010
#define CFG_COM_SEL_LOG_LEVEL	0x0020
#define CFG_COM_SEL_DBG_RMPP	0x0040
#define CFG_COM_SEL_LOG_FILTER	0x0080
#define CFG_COM_SEL_LOG_MASK	0x0100
#define CFG_COM_SEL_LOG_FILE	0x0200

#define CFG_COM_SEL_ALL 		0xFFFF

// Common query routines.
typedef struct fm_config_common_s{
	unsigned long	select_mask;
	unsigned long	device;
	unsigned long	port;
	int				debug;
	unsigned long 	pool_size;
	int				nodaemon;  // NOTE: READ-ONLY
	int				log_level; 
	int				debug_rmpp; 
	int				log_filter; 	// not used anymore
	int				log_mask; 		// not used anymore
	char			log_file[256]; 
}fm_config_common_t;

#define CFG_BM_SEL_BKEY			0x0001
#define CFG_BM_SEL_BKEY_LEASE	0x0002
#define CFG_BM_SEL_PRIORITY		0x0004
#define CFG_BM_SEL_TIMER		0x0008

#define CFG_BM_SEL_ALL 			0xFFFF

typedef struct bm_config_s{
	unsigned long		select_mask;
	unsigned char		bkey[8];
	unsigned long		bkey_lease;
	unsigned			priority;
	unsigned			timer;
}bm_config_t;


#define CFG_FE_SEL_LISTEN		0x0001
#define CFG_FE_SEL_LOGIN		0x0002
#define CFG_FE_SEL_PRIORITY		0x0004

#define CFG_FE_SEL_ALL 			0xFFFF

typedef struct fe_config_s{
	unsigned long		select_mask;
	unsigned listen;
	unsigned login;
	unsigned priority;
}fe_config_t;


#define CFG_PM_SEL_PRIORITY		0x0001
#define CFG_PM_SEL_TIMER		0x0002

#define CFG_PM_SEL_ALL 			0xFFFF


typedef struct pm_config_s{
	unsigned long		select_mask;
	unsigned long		priority;
	unsigned timer;
}pm_config_t;

#define CFG_SM_SEL_KEY				0x0001
#define CFG_SM_SEL_PRIORITY			0x0002
#define CFG_SM_SEL_TIMER			0x0004
#define CFG_SM_SEL_MAX_RETRY		0x0008
#define CFG_SM_SEL_RCV_WAIT_MSEC	0x0010
#define CFG_SM_SEL_SW_LFTIME		0x0020
#define CFG_SM_SEL_HOQ_LIFE			0x0040
#define CFG_SM_SEL_VL_STALL			0x0080
#define CFG_SM_SEL_SA_RESP_TIME		0x0100
#define CFG_SM_SEL_SA_PKT_LIFETIME	0x0200
#define CFG_SM_SEL_LID				0x0400
#define CFG_SM_SEL_LMC				0x0800
#define CFG_SM_SEL_PKEY_SUPPORT		0x1000
#define CFG_SM_SEL_MKEY				0x2000

#define CFG_SM_SEL_ALL 				0xFFFF


typedef struct sm_config_s{
	unsigned long		select_mask;
	unsigned char		key[8];
	unsigned long		priority;
	unsigned			timer;
	unsigned			max_retries;
	unsigned			rcv_wait_msec;
	unsigned			switch_lifetime;
	unsigned			hoq_life;
	unsigned			vl_stall;
	unsigned			sa_resp_time;
	unsigned			sa_packet_lifetime;
	unsigned			lid;
	unsigned			lmc;
	unsigned			pkey_support;
	unsigned char		mkey[8];
}sm_config_t;

// Note: Select mask here indicates the pkey index.
typedef struct sm_pkey_s{
	unsigned long		select_mask;
	unsigned long		pkey[32];
}sm_pkey_t;

#define CFG_SM_MC_SEL_CREATE	0x0001
#define CFG_SM_MC_SEL_PKEY		0x0002
#define CFG_SM_MC_SEL_MTU		0x0004
#define CFG_SM_MC_SEL_RATE		0x0008
#define CFG_SM_MC_SEL_SL		0x0010

#define CFG_SM_MC_SEL_ALL 		0xFFFF

typedef struct sm_mc_group_s{
	unsigned long		select_mask;
	unsigned			create;
	unsigned			pkey;
	unsigned			mtu;
	unsigned			rate;
	unsigned			sl;
}sm_mc_group_t;


#define CFG_FM_STATUS_SM			0x0001
#define CFG_FM_STATUS_PM			0x0002
#define CFG_FM_STATUS_BM			0x0004
#define CFG_FM_STATUS_FE			0x0008
#define CFG_FM_STATUS_UPTIME		0x0010
#define CFG_FM_STATUS_SM_MASTER		0x0020

#define CFG_FM_STATUS_SEL_ALL 		0xFFFF

typedef struct fm_cfg_status_s{
	unsigned long		select_mask;
	unsigned			sm_status;
	unsigned			pm_status;
	unsigned			bm_status;
	unsigned			fe_status;
	unsigned			sm_uptime;
	unsigned			sm_master;
}fm_cfg_status_t;

// init
fm_mgr_config_errno_t
fm_mgr_config_init
(
					OUT	p_fm_config_conx_hdlt		*p_hdl,
				IN		int							instance,
	OPTIONAL	IN		char						*rem_address,
	OPTIONAL	IN		char						*community
);


// connect
fm_mgr_config_errno_t
fm_mgr_config_connect
(
	IN		p_fm_config_conx_hdlt		p_hdl
);


fm_mgr_config_errno_t
fm_mgr_commong_cfg_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_type_t				mgr,
	IN		fm_mgr_action_t				action,
		OUT	fm_config_common_t			*info,
		OUT	fm_msg_ret_code_t			*ret_code
);


fm_mgr_config_errno_t
fm_mgr_bm_cfg_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	bm_config_t					*info,
		OUT	fm_msg_ret_code_t			*ret_code
);


fm_mgr_config_errno_t
fm_mgr_fe_cfg_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	fe_config_t					*info,
		OUT	fm_msg_ret_code_t			*ret_code
);

fm_mgr_config_errno_t
fm_mgr_pm_cfg_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	pm_config_t					*info,
		OUT	fm_msg_ret_code_t			*ret_code
);



fm_mgr_config_errno_t
fm_mgr_sm_cfg_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	sm_config_t					*info,
		OUT	fm_msg_ret_code_t			*ret_code
);

const char*
fm_mgr_get_error_str
(
	IN		fm_mgr_config_errno_t err
);

const char*
fm_mgr_get_resp_error_str
(
	IN		fm_msg_ret_code_t err
);



  

     




#endif
