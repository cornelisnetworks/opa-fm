/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_linux.c							     //
//									     //
// DESCRIPTION								     //
//    The OS dependent part of SM.  This version parses the command	     //
//    line and starts the OS independent part.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    main				main entry point		     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "if3.h"
#include "sm_l.h"
#include "sm_dbsync.h"
#include "sa_l.h"

#include "signal.h"
#include "hsm_config_srvr_api.h"
#include "hsm_config_srvr_data.h"
#include "ibyteswap.h"

/*
 *	Posix defines for argument cracking.
 */
extern	int		optind;
extern	int		opterr;
extern	int		optopt;
extern	char		*optarg;
extern  uint32_t sm_nodaemon;
extern  uint32_t xml_trace;
extern uint32_t            smDebugPerf;  // control SM/SA performance messages; default is off
extern uint32_t            saDebugRmpp;  // control SA RMPP INFO debug messages; default is off
extern IBhandle_t fdsa, fd_pm, pm_fd, dbsyncfd_if3;
extern Pool_t fe_pool, pm_pool;

/*
 *	Simulator definitions.
 */
time_t			sm_starttime = 0;

static SMThread_t				conf_server; // Runtime configuration server thread...
static p_hsm_com_server_hdl_t	conf_handle;

extern void* getSmXmlParserMemory(uint32_t size, char* info);
extern void freeSmXmlParserMemory(void *address, uint32_t size, char* info);
extern Status_t initSmXmlMemoryPool(void);
extern Status_t sm_parse_xml_config(void);
extern Status_t handleVfDgMemory(void);
extern Status_t sm_initialize_sm_pool(void);
extern void smLogLevelOverride(void);
extern size_t computeCompositeSize(void);

/*
 *	External SM independent routine.
 */
Status_t		sm_main(void);		   

extern Port_t *
sm_idb_find_port(uint64_t *guid, uint32_t *port, Port_t *pPort);

extern Port_t *
sm_idb_find_next_port(uint64_t *guid, uint32_t *port, Port_t *pPort);

extern Node_t *
sm_idb_find_node_id(uint32_t node_id, Node_t *pNode);

extern Node_t *
sm_idb_find_next_guid(uint64_t guid, Node_t *pNode);

extern Node_t *
sm_idb_find_guid(uint64_t guid, Node_t *pNode);

extern ServiceRecord_t* 
sm_idb_find_next_service_record(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, ServiceRecord_t *pSrp);

extern ServiceRecord_t* 
sm_idb_find_service_record(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, ServiceRecord_t *pSrp);
		 
extern McGroup_t*	
sm_idb_find_broadCast_group(IB_GID *pGid, McGroup_t *pGroup);

extern McGroup_t*	
sm_idb_find_next_broadCast_group(IB_GID *pGid, McGroup_t *pGroup);

extern McMember_t*	
sm_idb_find_broadCast_group_member(IB_GID *pGid, uint32_t *index, McMember_t *pMember);

extern McMember_t*	
sm_idb_find_next_broadCast_group_member(IB_GID *pGid, uint32_t *index, McMember_t *pMember);

extern Node_t *
sm_idb_find_switch(uint64_t guid, Node_t *pNode);

extern Node_t *
sm_idb_find_next_switch(uint64_t guid, Node_t *pNode);

extern void sm_setSweepRate(uint32_t rate);

extern fm_msg_ret_code_t sm_diag_ctrl(fm_config_datagram_t *msg);

fm_msg_ret_code_t
sm_conf_fill_common(fm_mgr_action_t action, fm_config_common_t *common)
{
	int reset_log_flag = 0;
	switch(action){
		case FM_ACT_GET:
			common->debug 		= sm_debug;
			common->debug_rmpp	= saDebugRmpp;
			common->device		= sm_config.hca;
			common->port		= sm_config.port;
			common->pool_size	= g_smPoolSize;
			common->log_filter	= 0; // sm_log_filter; no longer allowed
			common->log_level	= sm_log_level;
			common->log_mask	= 0; // sm_log_mask; no longer allowed
			common->nodaemon	= sm_nodaemon;
			strncpy(&common->log_file[0], sm_config.log_file, strlen(sm_config.log_file));
			
			return FM_RET_OK;
		case FM_ACT_SUP_GET:

			common->select_mask = CFG_COM_SEL_ALL;
			return FM_RET_OK;

		case FM_ACT_SUP_SET:

			common->select_mask = (CFG_COM_SEL_DEBUG | CFG_COM_SEL_LOG_LEVEL | CFG_COM_SEL_DBG_RMPP |
								   CFG_COM_SEL_LOG_FILTER | CFG_COM_SEL_LOG_MASK | CFG_COM_SEL_LOG_FILE);
			return FM_RET_OK;

		case FM_ACT_SET:
			if(common->select_mask & CFG_COM_SEL_DEVICE){
				//Ignore: Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_PORT){
				//Ignore:  Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_DEBUG){
				sm_debug = common->debug;
			}
			if(common->select_mask & CFG_COM_SEL_POOL_SIZE){
				//Ignore:  Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_NODAEMON){
				//Ignore:  Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_LOG_LEVEL){
				sm_log_level = common->log_level;
				reset_log_flag = 1;
			}
			if(common->select_mask & CFG_COM_SEL_DBG_RMPP){
				saDebugRmpp = common->debug_rmpp;
			}
#if 0	// filter and mask no longer allowed, ignored
			// if both are specified could use to set cs_log_set_mods_mask
			if(common->select_mask & CFG_COM_SEL_LOG_FILTER){
				sm_log_filter = common->log_filter;
				reset_log_flag = 1;
			}
			if(common->select_mask & CFG_COM_SEL_LOG_MASK){
				sm_log_mask = common->log_mask;
				reset_log_flag = 1;
			}
#endif
			if(common->select_mask & CFG_COM_SEL_LOG_FILE){
				strncpy(sm_config.log_file,&common->log_file[0],sizeof(sm_config.log_file)-1);
				sm_config.log_file[sizeof(sm_config.log_file)-1]=0;
				reset_log_flag = 1;
			}

			if(reset_log_flag) {
				cs_log_set_log_masks(sm_log_level, sm_config.syslog_mode, sm_log_masks);
				sm_init_log_setting();
			}

			return FM_RET_OK;

				

		default:

			return FM_RET_ACT_NOT_SUPPORTED;

			break;
	}

	return FM_RET_INVALID;


}


fm_msg_ret_code_t
sm_conf_fill_sm_conf(fm_mgr_action_t action, sm_config_t *config)
{
switch(action){
	case FM_ACT_GET:
			// FIXME: Byteswap the key....
			memcpy(&config->key[0], &sm_config.sm_key, sizeof(config->key));
			config->priority = sm_config.priority;
			config->timer = sm_config.timer;
			config->max_retries = sm_config.max_retries;
			config->rcv_wait_msec = sm_config.rcv_wait_msec;
			config->switch_lifetime = sm_config.switch_lifetime_n2;
			config->hoq_life = sm_config.hoqlife_n2;
			config->vl_stall = sm_config.vlstall;
			config->sa_resp_time = sm_config.sa_resp_time_n2;
			config->sa_packet_lifetime = sm_config.sa_packet_lifetime_n2;
			config->lid = sm_lid;
			config->lmc = sm_config.lmc;
			config->pkey_support = 1;
			
			return FM_RET_OK;
		case FM_ACT_SUP_GET:
			
			config->select_mask = CFG_SM_SEL_ALL;
			return FM_RET_OK;

		case FM_ACT_SUP_SET:

			config->select_mask = CFG_SM_SEL_ALL;
			return FM_RET_OK;

		case FM_ACT_SET:
			if(config->select_mask & CFG_SM_SEL_KEY){
				// FIXME: Byteswap the key....
				memcpy(&sm_config.sm_key, &config->key[0], sizeof(sm_config.sm_key));
			}
			if(config->select_mask & CFG_SM_SEL_PRIORITY){
				sm_smInfo.u.s.Priority = sm_config.priority = config->priority;
			}
			if(config->select_mask & CFG_SM_SEL_TIMER){
                /* PR 105316 - must convert time to microseconds */
                sm_setSweepRate((uint32_t)config->timer);
			}
			if(config->select_mask & CFG_SM_SEL_MAX_RETRY){
				sm_config.max_retries = config->max_retries;
			}
			if(config->select_mask & CFG_SM_SEL_RCV_WAIT_MSEC){
				sm_config.rcv_wait_msec = config->rcv_wait_msec;
			}
			if(config->select_mask & CFG_SM_SEL_SW_LFTIME){
				sm_config.switch_lifetime_n2 = config->switch_lifetime;
			}
			if(config->select_mask & CFG_SM_SEL_HOQ_LIFE){
				sm_config.hoqlife_n2 = config->hoq_life;
			}
			if(config->select_mask & CFG_SM_SEL_VL_STALL){
				sm_config.vlstall = config->vl_stall;
			}
			if(config->select_mask & CFG_SM_SEL_SA_RESP_TIME){
				sm_config.sa_resp_time_n2 = config->sa_resp_time;
			}
			if(config->select_mask & CFG_SM_SEL_SA_PKT_LIFETIME){
				sm_config.sa_packet_lifetime_n2 = config->sa_packet_lifetime;
			}
			if(config->select_mask & CFG_SM_SEL_LID){
				sm_lid = config->lid;
			}
			if(config->select_mask & CFG_SM_SEL_LMC){
				sm_config.lmc = config->lmc;
			}
			//if(config->select_mask & CFG_SM_SEL_PKEY_SUPPORT){
			//	sm_pkey_support = config->pkey_support;
			//}
			return FM_RET_OK;

		default:
			return FM_RET_ACT_NOT_SUPPORTED;
			break;
	}
	return FM_RET_INVALID;
}

fm_msg_ret_code_t
sm_conf_fill_sm_status(fm_mgr_action_t action, fm_sm_status_t *data)
{
	switch(action){
		case FM_ACT_GET:
			data->master = sm_util_get_state();
			data->status = 1;
			data->uptime = (unsigned long)difftime(time(NULL),sm_starttime);
			
			return FM_RET_OK;
		case FM_ACT_SUP_GET:
			
			data->select_mask = CFG_SM_SEL_ALL;
			return FM_RET_OK;

		case FM_ACT_SUP_SET:

			data->select_mask = 0;
			return FM_RET_OK;

		case FM_ACT_SET:

			data->select_mask = 0;
			return FM_RET_OK;

		default:

			return FM_RET_ACT_NOT_SUPPORTED;

			break;
	}

	return FM_RET_INVALID;
}

fm_msg_ret_code_t
sm_conf_fill_sm_pkey(void)
{
	return FM_RET_INVALID;
}

fm_msg_ret_code_t
sm_conf_fill_sm_mcast(void)
{
	return FM_RET_INVALID;
}

hsm_com_errno_t 
sm_conf_callback(hsm_com_datagram_t *data)
{
	fm_config_datagram_t *msg;

	if(data->data_len >= sizeof(msg->header)){
		msg = (fm_config_datagram_t*)data->buf;

		switch (msg->header.data_id) {
		case FM_DT_COMMON:
			if (msg->header.data_len != sizeof(fm_config_common_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = sm_conf_fill_common(msg->header.action,(fm_config_common_t*)&msg->data[0]);
			}
			break;
		case FM_DT_BM_CFG:
		case FM_DT_PM_CFG:
		case FM_DT_FE_CFG:
			msg->header.ret_code = FM_RET_DT_NOT_SUPPORTED;
			break;
		case FM_DT_SM_CFG:
			if (msg->header.data_len != sizeof(sm_config_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = sm_conf_fill_sm_conf(msg->header.action,(sm_config_t*)&msg->data[0]);
			}
			break;
		case FM_DT_SM_PKEY:
			msg->header.ret_code = FM_RET_DT_NOT_SUPPORTED;
			break;
		case FM_DT_SM_MC:
			msg->header.ret_code = FM_RET_DT_NOT_SUPPORTED;
			break;
		case FM_DT_SM_STATUS:
			if (msg->header.data_len != sizeof(fm_sm_status_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = sm_conf_fill_sm_status(msg->header.action,(fm_sm_status_t*)&msg->data[0]);
			}
			break;
		case FM_DT_SM_LOOP_TEST_START:
		case FM_DT_SM_LOOP_TEST_FAST_MODE_START:
		case FM_DT_SM_LOOP_TEST_STOP:
		case FM_DT_SM_LOOP_TEST_FAST:
		case FM_DT_SM_LOOP_TEST_INJECT_PACKETS:
		case FM_DT_SM_LOOP_TEST_INJECT_ATNODE:
		case FM_DT_SM_LOOP_TEST_INJECT_EACH_SWEEP:
		case FM_DT_SM_LOOP_TEST_PATH_LEN:
		case FM_DT_SM_LOOP_TEST_MIN_ISL_REDUNDANCY:
		case FM_DT_SM_LOOP_TEST_SHOW_PATHS:
		case FM_DT_SM_LOOP_TEST_SHOW_LFTS:
		case FM_DT_SM_LOOP_TEST_SHOW_TOPO:
		case FM_DT_SM_LOOP_TEST_SHOW_CONFIG:
		case FM_DT_FORCE_SWEEP:
		case FM_DT_SM_RESTORE_PRIORITY:
		case FM_DT_SM_GET_COUNTERS:
		case FM_DT_SM_RESET_COUNTERS:
		case FM_DT_SM_DUMP_STATE:
		case FM_DT_LOG_LEVEL:
		case FM_DT_LOG_MODE:
		case FM_DT_LOG_MASK:
		case FM_DT_SM_PERF_DEBUG_TOGGLE:
		case FM_DT_SA_PERF_DEBUG_TOGGLE:
		case FM_DT_RMPP_DEBUG_TOGGLE:
		case FM_DT_SM_FORCE_REBALANCE_TOGGLE:
		case FM_DT_SM_BROADCAST_XML_CONFIG:
		case FM_DT_SM_GET_ADAPTIVE_ROUTING:
		case FM_DT_SM_SET_ADAPTIVE_ROUTING:
        case FM_DT_SM_FORCE_ATTRIBUTE_REWRITE:
        case FM_DT_SM_SKIP_ATTRIBUTE_WRITE:
        case FM_DT_PAUSE_SWEEPS:
        case FM_DT_RESUME_SWEEPS:
			msg->header.ret_code = sm_diag_ctrl(msg);
			break;

		default:
			msg->header.ret_code = FM_RET_UNKNOWN_DT;
			break;
		}

		return HSM_COM_OK;
	}


	return HSM_COM_ERR_LEN;
}


/*                                                              
 * HFM remote control/config thread startup                                                                 
 */
void
sm_conf_server_run(uint32_t argc, uint8_t **argv){
	if(hcom_server_start(conf_handle) != HSM_COM_OK){
		if (errno != 0) {
			IB_LOG_ERROR_FMT("pm_conf_server_run", "Command server exited with error \"%s\" (%d)",
				strerror(errno), errno);
		} else {
			IB_LOG_ERROR0("Command server exited with error. No further information.");
		}
	}
}

/*                                                              
 * init and start HFM remote control/config thread                                                                 
 */
int
sm_conf_server_init(void){
	char server_path[64];
	int	status;

	memset(server_path,0,sizeof(server_path));

	// Start up the configuration server.
	conf_server.id = (void *)"RT-Conf server";
	conf_server.function = sm_conf_server_run;

	sprintf(server_path,"%s%s",HSM_FM_SCK_PREFIX,sm_env);

	if(hcom_server_init(&conf_handle,server_path,5,32768,&sm_conf_callback) != HSM_COM_OK){
		IB_LOG_ERROR0("Could not allocate server handle");
		return(-1);
	}

	status = sm_start_thread(&conf_server);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR("can't create remote configuration thread");
	}

	return 0;
}

/*
 * buffer output for several of the loop test commands that display a large
 * amount of data. This can be used for other large displays that utilize
 * the fm_diag interface for HSM
*/
void bufferLargeSmDiagOutputs(fm_config_interation_data_t *iterationData) {
    char        *location;
    int         len;

    if (iterationData->largeBuffer == NULL || iterationData->largeLength == 0) {
        iterationData->intermediateLength = 0;
        iterationData->more = 0;
        iterationData->done = 1;
        return;
    }

    location = iterationData->largeBuffer + iterationData->offset;
    len = iterationData->largeLength - iterationData->offset;
    if (len > FM_CONFIG_INTERMEDIATE_SIZE) {
        memcpy(iterationData->intermediateBuffer, location, FM_CONFIG_INTERMEDIATE_SIZE);
        iterationData->intermediateBuffer[FM_CONFIG_INTERMEDIATE_SIZE] = '\0';
        iterationData->intermediateLength = FM_CONFIG_INTERMEDIATE_SIZE + 1;
        iterationData->more = 1;
        iterationData->done = 0;
        iterationData->offset += FM_CONFIG_INTERMEDIATE_SIZE;
        return;
    }
    cs_strlcpy(iterationData->intermediateBuffer, location, FM_CONFIG_INTERMEDIATE_BUFF);
    iterationData->intermediateLength = strlen(location);
    iterationData->more = 0;
    iterationData->done = 1;
    return;
}

/*                                                              
 * sigterm signal handler setup                                                                
 */
void sm_linux_signal_handler(int a) {
	if (a == SIGHUP) {
		sm_control_reconfig();
	}
	else {
		sm_control_shutdown(NULL);
	}
}

//
//	IF3 RMPP Library API related Routines
//
static void if3_set_rmpp_cntx_pool_size_params(ManagerInfo_t *mi)
{
    // for now keep it simple and set the RMPP context pool sizes for
    // this thread to the RMPP context pool sizes of the SA
    mi->rmppDataLength = sa_data_length;
    mi->rmppMaxCntxt = sa_max_cntxt;
}

uint8_t if3_is_master(void) 
{ 
    return sm_isMaster(); 
}

Status_t if3_set_rmpp_minfo (ManagerInfo_t *mi)
{
    Status_t rc = VSTATUS_OK;
    // Default, RMPP filters required in order for thread to receive inbound
    // MAD requests
    mi->rmppCreateFilters = 1;

    switch (mi->mclass) {
    case MAD_CV_VENDOR_FE:
        mi->rmppMngrfd = &fdsa;
        mi->rmppPool = &fe_pool;
        // FE does not receive inbound MAD requests, so RMPP filters not required
        mi->rmppCreateFilters = 0;
        (void)if3_set_rmpp_cntx_pool_size_params(mi);
        break;        
    case MAD_CV_VFI_PM:
		{
			const char *tname = vs_thread_name_str();
			if (tname == NULL) {
            	IB_LOG_INFINI_INFO_FMT(__func__, 
					"Error unnamed thread");
				rc = VSTATUS_BAD;
			} else if (!strcmp(tname, "fe")) {
            	mi->rmppMngrfd = &fd_pm;
            	mi->rmppPool = &fe_pool;
                // FE does not receive inbound MAD requests, so RMPP filters not required
                mi->rmppCreateFilters = 0;
                (void)if3_set_rmpp_cntx_pool_size_params(mi);
        	} else if (!strcmp(tname, "pm")) {
            	mi->rmppMngrfd = &pm_fd;
            	mi->rmppPool = &pm_pool;
                (void)if3_set_rmpp_cntx_pool_size_params(mi);
        	} else {
            	IB_LOG_INFINI_INFO_FMT(__func__, 
					"Error unknown thread %s", tname);
                rc = VSTATUS_BAD;
        	}
		}
        break;
    case MAD_CV_VENDOR_DBSYNC:
        mi->rmppMngrfd = &dbsyncfd_if3; 
        mi->rmppPool = &sm_pool; 
        // SM DB Sync uses other filter mechanism to handle inbound MAD requests,
        // so RMPP filters not required
        mi->rmppCreateFilters = 0;
        mi->rmppDataLength = computeCompositeSize();
        mi->rmppMaxCntxt = sa_max_cntxt;
        break;
    default:
        rc = VSTATUS_ILLPARM;
        IB_LOG_INFINI_INFO_FMT(__func__, "MCLASS 0x%x not supported", mi->mclass);
        break;
    }
    return rc;
}

/*                                                              
 * SM main                                                                
 */
int
main(int argc, char *argv[]) {
	int		c;
	Status_t    status;
	char Opts[] = "Dnzie:d:p:k:r:t:x:y:s:l:C:X:";

	sm_starttime = time(NULL);

	/* Set the default environment name. */
	strcpy((void *)sm_env, "sm_0");

	// initialize XML memory pool here since we need it for XML parsing
	status = initSmXmlMemoryPool();
	if (status != VSTATUS_OK) {
		printf("exiting\n");
		exit(2);
	}

	// init callback function for XML parser so it can get pool memory
	initXmlPoolGetCallback(&getSmXmlParserMemory);
	initXmlPoolFreeCallback(&freeSmXmlParserMemory);

	// extract the instance number from the command line
	while ((c = getopt(argc, argv, Opts)) != -1) {
		switch (c) {
		case 'e':
			strncpy((void *)sm_env, optarg, 32);
			break;
		case 'X':
			sscanf(optarg, "%256s", sm_config_filename);
			break;
		case '?':
			IB_LOG_ERROR_FMT(__func__, "invalid command line parameter specified: %d", optopt);
			exit(1);
		default:
			break;
		}
	}

	// Parse the XML configuration
	status = sm_parse_xml_config();
	if (status != VSTATUS_OK) {
		printf("exiting\n");
		exit(1);
	}

	// Initialize SM memory pool so that we can allocate and copy sm_vfdg_config data
	// This parallels Esm_Init which also allocates the sm_pool early giving it the ability to
	// allocate memory that will be used for the sm_vfdg_config copy from the xml_config
	status = sm_initialize_sm_pool();
	if (status != VSTATUS_OK) {
		printf("sm_initialize_sm_pool not successful; exiting\n");
		exit(2);
	}

	// Allocate memory for and copy VF and DG information from xml config to sm_vfdg_config
	status = handleVfDgMemory();
	if (status != VSTATUS_OK) {
		printf("handleVfDgMemory not successful; exiting\n");
		exit(2);
	}

	// reset getopt()
	optind = 1;

	int overrideLogSetting = 0;

	/* Parse the command line.*/
	while ((c = getopt(argc, argv, Opts)) != -1) {
		switch (c) {
		case 'D':
			if (!xml_trace)
				sm_nodaemon = 0;
			break;
		case 'n':
			sm_nodaemon = 1;
			break;
		case 'd':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.hca);
			break;
		case 'p':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.port);
			break;
		case 'x':
			sscanf(optarg, "%u", (uint32_t *)&sm_lid);
			break;
		case 'k':
			sscanf(optarg, "0x%016"CS64"x", &sm_config.sm_key);
			break;
		case 't':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.timer);
			break;
		case 'r':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.priority);
			break;
		case 'y':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.lmc);
			break;
		case 'z':
			sm_config.debug = 1;
			break;
		case 'e':
			break;
		case 'l':
			sscanf(optarg, "%u", (uint32_t *)&sm_log_level);
			sm_log_level_override = 1;
			overrideLogSetting = 1;
			break;
		case 'C':
			sscanf(optarg, "%u", (uint32_t *)&sm_log_to_console);
			overrideLogSetting = 1;
			break;
		case 'X':
			break;
		}
	}

	if (overrideLogSetting)
		smLogLevelOverride();

#ifdef SIGTERM
    signal(SIGTERM, sm_linux_signal_handler);
#else
	#error SIGNTERM NOT INCLUDED
#endif
#ifdef SIGINT
    signal(SIGINT, sm_linux_signal_handler);
#endif
#ifdef SIGHUP
    signal(SIGHUP, sm_linux_signal_handler);
#endif
#ifdef SIGUSR1
    signal(SIGUSR1, sm_linux_signal_handler);
#endif
#ifdef SIGPIPE
    signal(SIGPIPE, SIG_IGN);   /* 'Inline' failure of wayward readers */
#endif

	/* Start up the independent SM code.*/
	(void)sm_main();
	printf("exiting\n");
	exit(0);
}
