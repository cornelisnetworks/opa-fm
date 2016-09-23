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

#ifndef HSM_CONFIG_SRVR_API
#define HSM_CONFIG_SRVR_API

#include "hsm_com_srvr_api.h"
#ifdef __LINUX__
#include <sys/types.h>
#include <stdint.h>
#else
#ifndef uint64_t
#define uint64_t unsigned long long
#endif
#endif

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
	FM_MGR_FE	= 0x0004
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
	FM_CONF_ALLOC_ERR,
	FM_CONF_MAX_ERROR_NUM
}fm_mgr_config_errno_t;

typedef enum{
	FM_ACT_NONE	=	0,
	FM_ACT_GET,			// Get selected attributes
	FM_ACT_SET,			// Set appropriate attributes
	FM_ACT_RSP,			// Response
	FM_ACT_SUP_GET,		// Query which attributes are supported
	FM_ACT_SUP_SET,		// Query which attributes are supported
	FM_ACT_GET_NEXT		// Get next logical row in table
}fm_mgr_action_t;


typedef enum{
	FM_RET_BAD_RET_LEN = -1,
	FM_RET_OK = 0,
	FM_RET_DT_NOT_SUPPORTED,	// Datatype is not supported
	FM_RET_ACT_NOT_SUPPORTED,	// Action is not supported for this datatype
	FM_RET_INVALID,				// Data is invalid.
	FM_RET_BAD_LEN,				// Data is an invalid length
	FM_RET_BUSY,				// Server busy, try again later.
	FM_RET_UNKNOWN_DT,			// Data type is not recognized.
	FM_RET_NOT_FOUND,			// Object not found
	FM_RET_NO_NEXT,				// No next entry in table
	FM_RET_NOT_MASTER,			// SM is not master and cannot perform requested operation
	FM_RET_NOSUCHOBJECT,
	FM_RET_NOSUCHINSTANCE,
	FM_RET_ENDOFMIBVIEW,
	FM_RET_ERR_NOERROR,
	FM_RET_ERR_TOOBIG,
	FM_RET_ERR_NOSUCHNAME,
	FM_RET_ERR_BADVALUE,
	FM_RET_ERR_READONLY,
	FM_RET_ERR_GENERR,
	FM_RET_ERR_NOACCESS,
	FM_RET_ERR_WRONGTYPE,
	FM_RET_ERR_WRONGLENGTH,
	FM_RET_ERR_WRONGENCODING,
	FM_RET_ERR_WRONGVALUE,
	FM_RET_ERR_NOCREATION,
	FM_RET_ERR_INCONSISTENTVALUE,
	FM_RET_ERR_RESOURCEUNAVAILABLE,
	FM_RET_ERR_COMMITFAILED,
	FM_RET_ERR_UNDOFAILED,
	FM_RET_ERR_AUTHORIZATIONERROR,
	FM_RET_ERR_NOTWRITABLE,
	FM_RET_END_OF_TABLE,
    FM_RET_INTERNAL_ERR,
    FM_RET_CONX_CLOSED,
	FM_RET_TIMEOUT
}fm_msg_ret_code_t;


typedef enum{
	FM_DT_NONE	=	0,
	FM_DT_COMMON,
	FM_DT_BM_CFG,
	FM_DT_PM_CFG,
	FM_DT_FE_CFG,
	FM_DT_SM_CFG,
	FM_DT_SM_PKEY,
	FM_DT_SM_MC,
	FM_DT_SM_STATUS,
	FM_DT_PM_STATUS,
	FM_DT_BM_STATUS,
	FM_DT_FE_STATUS,
	FM_DT_SM_NODE_INFO,
	FM_DT_SM_PORT_INFO,
	FM_DT_SM_SWITCH_INFO,
	FM_DT_SM_MCAST_GRP_INFO,
	FM_DT_SM_MCAST_REC_INFO,
	FM_DT_SM_SM_INFO,
	FM_DT_SM_LINK_INFO,
	FM_DT_SM_SERV_INFO,
	FM_DT_SM_GUID_INFO,
	FM_DT_LOG_LEVEL,
	FM_DT_DEBUG_TOGGLE,
	FM_DT_RMPP_DEBUG_TOGGLE,
	FM_DT_FORCE_SWEEP,
	FM_DT_SM_PERF_DEBUG_TOGGLE,
	FM_DT_SA_PERF_DEBUG_TOGGLE,
	FM_DT_SM_LOOP_TEST_FAST_MODE_START,
	FM_DT_SM_LOOP_TEST_START,
	FM_DT_SM_LOOP_TEST_STOP,
	FM_DT_SM_LOOP_TEST_FAST,
	FM_DT_SM_LOOP_TEST_INJECT_PACKETS,
	FM_DT_SM_LOOP_TEST_INJECT_ATNODE,
	FM_DT_SM_LOOP_TEST_INJECT_EACH_SWEEP,
	FM_DT_SM_LOOP_TEST_PATH_LEN,
	FM_DT_SM_LOOP_TEST_MIN_ISL_REDUNDANCY,
	FM_DT_SM_LOOP_TEST_SHOW_PATHS,
	FM_DT_SM_LOOP_TEST_SHOW_LFTS,
	FM_DT_SM_LOOP_TEST_SHOW_TOPO,
	FM_DT_SM_LOOP_TEST_SHOW_CONFIG,
	FM_DT_SM_RESTORE_PRIORITY,
	FM_DT_SM_GET_COUNTERS,
	FM_DT_SM_RESET_COUNTERS,
	FM_DT_SM_DUMP_STATE,
	FM_DT_BM_RESTORE_PRIORITY,
	FM_DT_PM_RESTORE_PRIORITY,
	FM_DT_SM_FORCE_REBALANCE_TOGGLE,
	FM_DT_PM_GET_COUNTERS,
	FM_DT_PM_RESET_COUNTERS,
	FM_DT_LOG_MODE,
	FM_DT_LOG_MASK,
	FM_DT_SM_BROADCAST_XML_CONFIG,
	FM_DT_SM_GET_ADAPTIVE_ROUTING,
	FM_DT_SM_SET_ADAPTIVE_ROUTING,
	FM_DT_SM_FORCE_ATTRIBUTE_REWRITE,
	FM_DT_SM_SKIP_ATTRIBUTE_WRITE,
	FM_DT_PAUSE_SWEEPS,
	FM_DT_RESUME_SWEEPS,
}fm_datatype_t;

typedef struct fm_error_map_s{
	int					err_set;
	fm_msg_ret_code_t	map[64]; 
}fm_error_map_t;


/* The current error map is copied to the pointer provided */
fm_mgr_config_errno_t
fm_mgr_config_get_error_map
(
	IN		p_fm_config_conx_hdlt	hdl,
		OUT	fm_error_map_t			*error_map
);

fm_mgr_config_errno_t
fm_mgr_config_clear_error_map
(
	IN		p_fm_config_conx_hdlt	hdl
);


fm_mgr_config_errno_t
fm_mgr_config_get_error_map_entry
(
	IN		p_fm_config_conx_hdlt	hdl,
	IN		uint64_t				mask,
		OUT	fm_mgr_config_errno_t	*error_code
);

fm_mgr_config_errno_t
fm_mgr_config_set_error_map_entry
(
	IN		p_fm_config_conx_hdlt	hdl,
	IN		uint64_t				mask,
	IN		fm_mgr_config_errno_t	error_code
);






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
	uint64_t		select_mask;
	int32_t			device;
	int32_t			port;
	int				debug;
	unsigned long 	pool_size;
	int				nodaemon;  // NOTE: READ-ONLY
	int				log_level; 
	int				debug_rmpp; 
	int				log_filter; 
	int				log_mask; 
	char			log_file[256]; 
}fm_config_common_t;

#define CFG_BM_SEL_BKEY			0x0001
#define CFG_BM_SEL_BKEY_LEASE	0x0002
#define CFG_BM_SEL_PRIORITY		0x0004
#define CFG_BM_SEL_TIMER		0x0008

#define CFG_BM_SEL_ALL 			0xFFFF

typedef struct bm_config_s{
	uint64_t			select_mask;
	unsigned char		bkey[8];
	int32_t				bkey_lease;
	unsigned			priority;
	unsigned			timer;
}bm_config_t;


#define CFG_FE_SEL_LISTEN		0x0001
#define CFG_FE_SEL_LOGIN		0x0002
#define CFG_FE_SEL_PRIORITY		0x0004

#define CFG_FE_SEL_ALL 			0xFFFF

typedef struct fe_config_s{
	uint64_t			select_mask;
	unsigned listen;
	unsigned login;
	unsigned priority;
}fe_config_t;


#define CFG_PM_SEL_PRIORITY		0x0001
#define CFG_PM_SEL_TIMER		0x0002

#define CFG_PM_SEL_ALL 			0xFFFF


typedef struct pm_config_s{
	uint64_t			select_mask;
	int32_t				priority;
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
	uint64_t			select_mask;
	unsigned char		key[8];
	int32_t				priority;
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
	uint64_t			select_mask;
	unsigned long		pkey[32];
}sm_pkey_t;

#define CFG_SM_MC_SEL_CREATE	0x0001
#define CFG_SM_MC_SEL_PKEY		0x0002
#define CFG_SM_MC_SEL_MTU		0x0004
#define CFG_SM_MC_SEL_RATE		0x0008
#define CFG_SM_MC_SEL_SL		0x0010

#define CFG_SM_MC_SEL_ALL 		0xFFFF

typedef struct sm_mc_group_s{
	uint64_t			select_mask;
	unsigned			create;
	unsigned			pkey;
	unsigned			mtu;
	unsigned			rate;
	unsigned			sl;
}sm_mc_group_t;


#define CFG_SM_STATUS_STATE			0x0001
#define CFG_SM_STATUS_UPTIME		0x0002
#define CFG_SM_STATUS_MASTER		0x0004

#define CFG_SM_STATUS_SEL_ALL 		0xFFFF

typedef struct fm_sm_status_s{
	uint64_t			select_mask;
	unsigned			status;
	int32_t				uptime;
	unsigned			master;
}fm_sm_status_t;

#define CFG_PM_STATUS_STATE			0x0001
#define CFG_PM_STATUS_UPTIME		0x0002
#define CFG_PM_STATUS_MASTER		0x0004

#define CFG_PM_STATUS_SEL_ALL 		0xFFFF

typedef struct fm_pm_status_s{
	uint64_t			select_mask;
	unsigned			status;
	unsigned long  		uptime;
	unsigned			master;
}fm_pm_status_t;

#define CFG_FE_STATUS_STATE			0x0001
#define CFG_FE_STATUS_UPTIME		0x0002
#define CFG_FE_STATUS_MASTER		0x0004

#define CFG_FE_STATUS_SEL_ALL 		0xFFFF

typedef struct fm_fe_status_s{
	uint64_t			select_mask;
	unsigned			status;
	unsigned long  		uptime;
	unsigned			master;
}fm_fe_status_t;

#define CFG_BM_STATUS_STATE			0x0001
#define CFG_BM_STATUS_UPTIME		0x0002
#define CFG_BM_STATUS_MASTER		0x0004

#define CFG_BM_STATUS_SEL_ALL 		0xFFFF

typedef struct fm_bm_status_s{
	uint64_t			select_mask;
	unsigned			status;
	unsigned long		uptime;
	unsigned			master;
}fm_bm_status_t;

typedef struct fm_sm_node_info_s{
	uint64_t		select_mask;
	unsigned char   ibSmNodeInfoSubnetPrefix[8];
	unsigned char   ibSmNodeInfoNodeGUID[8];
    unsigned long   ibSmNodeInfoBaseVersion;
    unsigned long   ibSmNodeInfoClassVersion;
    long            ibSmNodeInfoType;
    unsigned long   ibSmNodeInfoNumPorts;
    unsigned char   ibSmNodeInfoSystemImageGUID[8];
    unsigned long   ibSmNodeInfoPartitionCap;
    unsigned char   ibSmNodeInfoDeviceID[2];
    unsigned char   ibSmNodeInfoRevision[4];
    unsigned char   ibSmNodeInfoVendorID[3];
    char            ibSmNodeInfoDescription[256];
}fm_sm_node_info_t;


typedef struct fm_sm_port_info_s{
	uint64_t		select_mask;
    char            ibSmPortInfoSubnetPrefix[8];
    char            ibSmPortInfoNodeGUID[8];
    unsigned long   ibSmPortInfoLocalPortNum;
	char            ibSmPortInfoMKey[8];
    char            ibSmPortInfoGIDPrefix[8];
    unsigned long   ibSmPortInfoLID;
    unsigned long   ibSmPortInfoMasterSmLID;
    char            ibSmPortInfoCapMask[4];
    char            ibSmPortInfoDiagCode[2];
    unsigned long   ibSmPortInfoMKeyLeasePeriod;
    unsigned long   ibSmPortInfoLinkWidthEnabled;
    unsigned long   ibSmPortInfoLinkWidthSupported;
    unsigned long   ibSmPortInfoLinkWidthActive;
    unsigned long   ibSmPortInfoLinkSpeedSupported;
    unsigned long   ibSmPortInfoState;
    unsigned long   ibSmPortInfoPhyState;
    unsigned long   ibSmPortInfoLinkDownDefState;
    unsigned long   ibSmPortInfoMKeyProtBits;
    unsigned long   ibSmPortInfoLMC;
    unsigned long   ibSmPortInfoLinkSpeedActive;
    unsigned long   ibSmPortInfoLinkSpeedEnabled;
    long            ibSmPortInfoNeighborMTU;
    unsigned long   ibSmPortInfoMasterSmSL;
    unsigned long   ibSmPortInfoVLCap;
    unsigned long   ibSmPortInfoVLHighLimit;
    unsigned long   ibSmPortInfoVLArbHighCap;
    unsigned long   ibSmPortInfoVLArbLowCap;
    long            ibSmPortInfoMTUCap;
    unsigned long   ibSmPortInfoVLStallCount;
    unsigned long   ibSmPortInfoHOQLife;
    unsigned long   ibSmPortInfoOperVL;
    long            ibSmPortInfoInPartEnforce;
    long            ibSmPortInfoOutPartEnforce;
    long            ibSmPortInfoInFilterRawPktEnf;
    long            ibSmPortInfoOutFilterRawPktEnf;
    unsigned long   ibSmPortInfoMKeyViolation;
    unsigned long   ibSmPortInfoPKeyViolation;
    unsigned long   ibSmPortInfoQKeyViolation;
    unsigned long   ibSmPortInfoGUIDCap;
    unsigned long   ibSmPortInfoSubnetTimeout;
    unsigned long   ibSmPortInfoRespTime;
    unsigned long   ibSmPortInfoLocalPhyError;
    unsigned long   ibSmPortInfoOverrunError;
    char            ibSmPortInfoInitType;
    char            ibSmPortInfoInitTypeReply;
}fm_sm_port_info_t;

typedef struct fm_sm_switch_info_s{
	uint64_t		select_mask;
    char           	ibSmSwitchInfoSubnetPrefix[8];
    char            ibSmSwitchInfoNodeGUID[8];
    unsigned long   ibSmSwitchInfoLinearFdbCap;
    unsigned long   ibSmSwitchInfoRandomFdbCap;
    unsigned long   ibSmSwitchInfoMcastFdbCap;
    unsigned long   ibSmSwitchInfoLinearFdbTop;
    unsigned long   ibSmSwitchInfoDefaultPort;
    unsigned long   ibSmSwitchInfoDefPriMcastPort;
    unsigned long   ibSmSwitchInfoDefNonPriMcastPort;
    unsigned long   ibSmSwitchInfoLifeTimeValue;
    unsigned long   ibSmSwitchInfoPortStateChange;
    unsigned long   ibSmSwitchInfoLIDsPerPort;
    unsigned long   ibSmSwitchInfoPartitionEnfCap;
    long            ibSmSwitchInfoInEnfCap;
    long            ibSmSwitchInfoOutEnfCap;
    long            ibSmSwitchInfoInFilterRawPktCap;
    long            ibSmSwitchInfoOutFilterRawPktCap;
    long            ibSmSwitchInfoEnhanced0;
}fm_sm_switch_info_t;

typedef struct fm_sm_guid_info_s{
	uint64_t		select_mask;
    char            ibSmGUIDInfoSubnetPrefix[8];
    char            ibSmGUIDInfoNodeGUID[8];
    unsigned long   ibSmGUIDInfoPortNum;
    unsigned long   ibSmGUIDInfoBlockNum;
    char            ibSmGUIDInfoBlock[255];

}fm_sm_guid_info_t;

typedef struct fm_sm_link_info_s{
	uint64_t		select_mask;
    char            ibSmLinkSubnetPrefix[8];
    char            ibSmLinkFromNodeGUID[8];
    unsigned long   ibSmLinkFromPortNum;
    char            ibSmLinkToNodeGUID[8];
    unsigned long   ibSmLinkToPortNum;
}fm_sm_link_info_t;

typedef struct fm_sm_mcast_group_info_s{
	uint64_t		select_mask;
	unsigned char   ibSmMcastGroupSubnetPrefix[8];
	unsigned char   ibSmMcastGroupMGID[16];
	unsigned char   ibSmMcastGroupQKey[2];
	unsigned long   ibSmMcastGroupMLID;
	long            ibSmMcastGroupMTU;
	unsigned long   ibSmMcastGroupTClass;
	unsigned long   ibSmMcastGroupPKey;
	unsigned long   ibSmMcastGroupRate;
	unsigned long   ibSmMcastGroupPacketLifeTime;
	unsigned long   ibSmMcastGroupSL;
	unsigned char            ibSmMcastGroupFlowLabel[3];
	unsigned long   ibSmMcastGroupHopLimit;
	unsigned long   ibSmMcastGroupScope;
}fm_sm_mcast_group_info_t;

typedef struct fm_sm_mcast_member_info_s{
	uint64_t		select_mask;
    char            ibSmMcastMemberSubnetPrefix[8];
    char            ibSmMcastMemberMGID[16];
    long            ibSmMcastMemberVectorIndex;
    char            ibSmMcastMemberVector[255];
    long            ibSmMcastMemberVectorSize;
    long            ibSmMcastMemberVectorElementSize;
    unsigned long   ibSmMcastMemberLastChange;

}fm_sm_mcast_member_info_t;

typedef struct fm_sm_service_info_s{
	uint64_t		select_mask;
    unsigned char   ibSmServiceSubnetPrefix[8];
    unsigned char   ibSmServiceID[8];
    unsigned char   ibSmServiceGID[16];
    unsigned long   ibSmServicePKey;
    unsigned long   ibSmServiceLease;
    unsigned char   ibSmServiceKey[16];
    char            ibSmServiceName[256];
    unsigned char   ibSmServiceData[128];
}fm_sm_service_info_t;


fm_mgr_config_errno_t
fm_mgr_simple_query
(
	IN      p_fm_config_conx_hdlt       hdl,
	IN      fm_mgr_action_t             action,
	IN      fm_datatype_t               data_type_id,
	IN		fm_mgr_type_t				mgr,
	IN      int                         data_len,
		OUT void                        *data,
		OUT fm_msg_ret_code_t           *ret_code
);

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

fm_mgr_config_errno_t
fm_sm_status_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	fm_sm_status_t				*info,
		OUT	fm_msg_ret_code_t			*ret_code
);

fm_mgr_config_errno_t
fm_pm_status_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	fm_pm_status_t				*info,
		OUT	fm_msg_ret_code_t			*ret_code
);

fm_mgr_config_errno_t
fm_bm_status_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	fm_bm_status_t				*info,
		OUT	fm_msg_ret_code_t			*ret_code
);

fm_mgr_config_errno_t
fm_fe_status_query
(
	IN		p_fm_config_conx_hdlt		hdl,
	IN		fm_mgr_action_t				action,
		OUT	fm_fe_status_t				*info,
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
