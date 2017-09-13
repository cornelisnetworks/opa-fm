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

/************************************************************************
* 
* FILE NAME
*      fe_linux.c
*
* DESCRIPTION
*  Linux start file for fe 
* DATA STRUCTURES
* FUNCTIONS
*
* DEPENDENCIES
* HISTORY
*
*      NAME                 DATE        REMARKS
*
***********************************************************************/

#include <stdio.h>
#include <unistd.h>
#include "ib_types.h"
#include "sm_l.h"
#include "ib_mad.h"
#include "ib_status.h"
#include <fe_main.h>
#include "hsm_config_srvr_api.h"
#include "hsm_config_srvr_data.h"
#include <fm_xml.h>
#include "if3.h"
#include "cs_queue.h"
#include "mal_g.h"
#include "sm_dbsync.h"
#include "sa_l.h"


extern void fe_compute_pool_size(void);
extern int fe_main(void);
extern int atoi(const char *ptr);
extern void fe_process_config(FMXmlCompositeConfig_t *xml_config, uint32_t fe_instance);
extern void fe_init_globals(void);
extern uint8_t sm_isMaster(void);
extern void fe_init_log_setting(void);
extern void rmpp_sma_spoofing_check_set(uint32_t value);
extern FEXmlConfig_t fe_config;
extern SMXmlConfig_t sm_config;

// global vars
extern size_t g_fePoolSize;

extern IBhandle_t fdsa, fd_pm, pm_fd, dbsyncfd_if3;
extern Pool_t fe_pool, pm_pool;
extern uint32_t rmpp_sma_spoofing_check;
extern char msgbuf[256];

// extern's for getopt.h
extern	int		optind;
extern	int		opterr;
extern	int		optopt;
extern	char	*optarg;
extern  void    if3RmppDebugOn(void);
extern	void    if3RmppDebugOff(void);
extern	int		if3RmppDebugGet(void);

extern void fe_set_log_level(uint32_t log_level);
extern void fe_set_log_mode(uint32_t log_mode);
extern void fe_set_log_mask(const char* mod, uint32_t mask);
extern void mai_set_default_pkey(uint16_t pKey);

static Thread_t	conf_server;

static p_hsm_com_server_hdl_t	conf_handle;

fm_msg_ret_code_t
fe_conf_fill_common(fm_mgr_action_t action, fm_config_common_t *common)
{
	int reset_log_flag = 0;
	switch(action){
		case FM_ACT_GET:
			common->debug 		= 0;
			common->debug_rmpp	= if3RmppDebugGet();
			common->device		= fe_config.hca;
			common->port		= fe_config.port;
			common->pool_size	= g_fePoolSize;
			common->log_filter	= 0;	// fe_log_filter; no longer allowed
			common->log_level	= fe_config.log_level;
			common->log_mask	= 0;	// fe_log_mask; no longer allowed
			common->nodaemon	= fe_nodaemon;
			strncpy(common->log_file, fe_config.log_file, sizeof(fe_config.log_file)-1);
			common->log_file[sizeof(common->log_file)-1] = 0;
			
			return FM_RET_OK;
		case FM_ACT_SUP_GET:

			common->select_mask = (CFG_COM_SEL_ALL & ~CFG_COM_SEL_DEBUG);
			return FM_RET_OK;

		case FM_ACT_SUP_SET:

			common->select_mask = (CFG_COM_SEL_LOG_LEVEL | CFG_COM_SEL_DBG_RMPP | CFG_COM_SEL_LOG_FILTER |
								   CFG_COM_SEL_LOG_MASK | CFG_COM_SEL_LOG_FILE);
			return FM_RET_OK;

		case FM_ACT_SET:
			if(common->select_mask & CFG_COM_SEL_DEVICE){
				//Ignore Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_PORT){
				//Ignore:  Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_DEBUG){
				//Ignore: Not supported
			}
			if(common->select_mask & CFG_COM_SEL_POOL_SIZE){
				//Ignore:  Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_NODAEMON){
				//Ignore:  Requires restart of the SM.
			}
			if(common->select_mask & CFG_COM_SEL_LOG_LEVEL){
				fe_config.log_level = common->log_level;
				reset_log_flag = 1;
			}
			if(common->select_mask & CFG_COM_SEL_DBG_RMPP){
				if(common->debug_rmpp)
					if3RmppDebugOn();
				else
					if3RmppDebugOff();
			}
#if 0	// filter and mask no longer allowed, ignored
			// if both are specified could use to set cs_log_set_mods_mask
			if(common->select_mask & CFG_COM_SEL_LOG_FILTER){
				fe_log_filter = common->log_filter;
				reset_log_flag = 1;
			}
			if(common->select_mask & CFG_COM_SEL_LOG_MASK){
				fe_log_mask = common->log_mask;
				reset_log_flag = 1;
			}
#endif
			if(common->select_mask & CFG_COM_SEL_LOG_FILE){
				strncpy(fe_config.log_file, &common->log_file[0], sizeof(fe_config.log_file)-1);
				fe_config.log_file[sizeof(fe_config.log_file)-1] = 0;
				reset_log_flag = 1;
			}

			if(reset_log_flag) {
				cs_log_set_log_masks(fe_config.log_level, fe_config.syslog_mode, fe_log_masks);
				fe_init_log_setting();
			}

			return FM_RET_OK;

				

		default:

			return FM_RET_ACT_NOT_SUPPORTED;

			break;
	}

	return FM_RET_INVALID;


}

fm_msg_ret_code_t
fe_conf_fill_fe_conf(fm_mgr_action_t action, fe_config_t *config)
{
switch(action){
		case FM_ACT_GET:

			config->listen	= fe_config.listen;
			config->login	= fe_config.login;
			
			return FM_RET_OK;
		case FM_ACT_SUP_GET:
			
			config->select_mask = (CFG_FE_SEL_ALL & ~CFG_FE_SEL_PRIORITY);
			return FM_RET_OK;

		case FM_ACT_SUP_SET:

			config->select_mask = (CFG_FE_SEL_ALL & ~CFG_FE_SEL_PRIORITY);
			return FM_RET_OK;

		case FM_ACT_SET:
			if(config->select_mask & CFG_FE_SEL_LISTEN){
				fe_config.listen = config->listen;
			}
			if(config->select_mask & CFG_FE_SEL_LOGIN){
				fe_config.login = config->login;
			}
			if(config->select_mask & CFG_FE_SEL_PRIORITY){
				//Ignored
			}

			return FM_RET_OK;

				

		default:

			return FM_RET_ACT_NOT_SUPPORTED;

			break;
	}

	return FM_RET_INVALID;
}

hsm_com_errno_t 
fe_conf_callback(hsm_com_datagram_t *data)
{
	fm_config_datagram_t *msg;

	if(data->data_len >= sizeof(msg->header)){
		msg = (fm_config_datagram_t*)data->buf;

		switch (msg->header.data_id) {
		case FM_DT_COMMON:
			if (msg->header.data_len != sizeof(fm_config_common_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = fe_conf_fill_common(msg->header.action,(fm_config_common_t*)&msg->data[0]);
			}
			break;
		case FM_DT_FE_CFG:
			if (msg->header.data_len != sizeof(fe_config_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = fe_conf_fill_fe_conf(msg->header.action,(fe_config_t*)&msg->data[0]);
			}
			break;

		case FM_DT_LOG_LEVEL:
			fe_set_log_level(*(uint32_t *)&msg->data[0]);
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_LOG_MODE:
			fe_set_log_mode(*(uint32_t *)&msg->data[0]);
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_LOG_MASK:
			fe_set_log_mask((char*)&msg->data[sizeof(uint32_t)],
								*(uint32_t *)&msg->data[0]);
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_DEBUG_TOGGLE:
			if (feDebugGet()) {
				feDebugOff();
				IB_LOG_INFINI_INFO0("Turning OFF Debug for FE");
			} else {
				feDebugOn();
				IB_LOG_INFINI_INFO0("Turning ON Debug for FE");
			}
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_RMPP_DEBUG_TOGGLE:
			if (if3RmppDebugGet()) {
				IB_LOG_INFINI_INFO0("turning rmpp debug off for FE");
				if3RmppDebugOff();
			} else {
				IB_LOG_INFINI_INFO0("turning rmpp debug on for FE");
				if3RmppDebugOn();
			}
			msg->header.ret_code = FM_RET_OK;
			break;

		default:
			msg->header.ret_code = FM_RET_UNKNOWN_DT;
			break;
		}

		return HSM_COM_OK;
	}



	return HSM_COM_ERR_LEN;
}

void
fe_conf_server_run(uint32_t argc, uint8_t **argv){
	if(hcom_server_start(conf_handle) != HSM_COM_OK){
		IB_LOG_ERROR0("Server exited with error");
	}
}

int
fe_conf_server_init(void){
	char server_path[64];
	int	status;

	memset(server_path,0,sizeof(server_path));


	snprintf(server_path, sizeof(server_path), "%s%s",HSM_FM_SCK_PREFIX,fe_env_str);

	if(hcom_server_init(&conf_handle,server_path,5,1024,&fe_conf_callback) != HSM_COM_OK){
		IB_LOG_ERROR0("Could not allocate server handle");
		return(-1);
	}

	status = vs_thread_create (&conf_server, (void *)"RT-Conf (FE)", fe_conf_server_run,
							  0,NULL,256 * 1024);

	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR("can't create remote configuration thread");
	}

	return 0;
}

void fe_init_log_setting(void)
{
	// check log related configuration parameter settings, and display appropriate warning messages.
    // This is also checked during fe startup.
    if (!sm_isValidLogConfigSettings(VIEO_FE_MOD_ID, fe_config.log_level, fe_config.syslog_mode, fe_log_masks, fe_config.log_file, fe_config.syslog_facility)) {
        (void)sm_getLogConfigSettings(&fe_config.log_level, &fe_config.syslog_mode, fe_log_masks, fe_config.log_file, fe_config.syslog_facility);
    }
    // check core dump related configuration parameter settings 
    (void)sm_isValidCoreDumpConfigSettings(VIEO_FE_MOD_ID, fe_config.CoreDumpLimit, fe_config.CoreDumpDir); 
    // check SMA spoof checking related configuration parameter setting 
    (void)rmpp_sma_spoofing_check_set(sm_config.sma_spoofing_check);
}

void fe_set_log_level(uint32_t log_level)
{
	snprintf(msgbuf, sizeof(msgbuf), "Setting FE LogLevel to %u", (unsigned)log_level);
	fe_config.log_level = log_level;
	vs_log_output_message(msgbuf, FALSE);
	cs_log_set_log_masks(fe_config.log_level, fe_config.syslog_mode, fe_log_masks);
	fe_init_log_setting();
}

void fe_set_log_mode(uint32_t log_mode)
{
	snprintf(msgbuf, sizeof(msgbuf), "Setting FE LogMode to %u", (unsigned)log_mode);
	fe_config.syslog_mode = log_mode;
	vs_log_output_message(msgbuf, FALSE);
	cs_log_set_log_masks(fe_config.log_level, fe_config.syslog_mode, fe_log_masks);
	fe_init_log_setting();
}

void fe_set_log_mask(const char* mod, uint32_t mask)
{
	if (! cs_log_get_module_id(mod)) {
		snprintf(msgbuf, sizeof(msgbuf), "Requested setting FE LogMask for invalid subsystem: %s", mod);
		vs_log_output_message(msgbuf, FALSE);
	} else {
		snprintf(msgbuf, sizeof(msgbuf), "Setting FE %s_LogMask to 0x%x", mod, (unsigned)mask);
		vs_log_output_message(msgbuf, FALSE);
		cs_log_set_log_masks(fe_config.log_level, fe_config.syslog_mode, fe_log_masks);
		fe_init_log_setting();
	}
}

Status_t fe_main_init_port(void)
{
    return VSTATUS_OK;
}

Status_t fe_main_register_fe(int queue_size)
{
    Status_t status = VSTATUS_OK;

    status = ib_register_fe(queue_size, 1);
    if (status != VSTATUS_OK)
        Shutdown = 1;

    return status;
}

uint8_t fe_is_thread(void) 
{ 
    return 1; 
}

// This routine parses the FE configuration and initializes some of its global
// variables.
Status_t
fe_initialize_config(FMXmlCompositeConfig_t *xml_config, uint32_t fe_instance)
{
	IB_ENTER ("fe_initialize_config", 0, 0, 0, 0);

    //	Initialize the global variables that are used by FE and those that are common with VxWorks.
    //	This needs to be done here especially for the bm_is_running flag, because
    //	there will be a race for that flag, if the SM is stopped just after starting.
    //	Its possible that the signal handler runs before bm_main() and set the bm_is_running
    //	flag to 0. If the globals are initialized in bm_main(), it will overwrite the
    //	flag value that is set by bm_shutdown() and the BM would not shutdown cleanly.
    //
    //	For the same reason this also needs to be done before setting the signal handler
	fe_init_globals();

	// -e option information comes from sm's -e option
	// translate sm_env ("sm_#") to fe_env_str ("fm_#")
	// we use this to identify our instance and hence which Fm section of config
	memset (fe_env_str, 0, sizeof(fe_env_str));
	cs_strlcpy ((void *)fe_env_str, (void*)sm_env, sizeof(fe_env_str));
	fe_env_str[0]='f';

	// -X option information comes from sm's -X option
	cs_strlcpy(fe_config_filename, sm_config_filename, sizeof(fe_config_filename));
	
    //
    //	Get the environment variables before applying the command line overrides.
    //	If this fails, we still continue because we have default values that we can us.
	// sm_main has already called read_info_file()
    //
    fe_process_config(xml_config, fe_instance);

	return VSTATUS_OK;
}

// This requires the caller to have invoked fe_initialize_config prior to this
// call.
// It completes FE initialization and then starts the actual FE.
void
unified_sm_fe(uint32_t argc, uint8_t ** argv)
{
	fe_config.subnet_size = MAX(fe_config.subnet_size, MIN_SUPPORTED_ENDPORTS);
	mai_set_default_pkey(STL_DEFAULT_FM_PKEY);

    if (fe_config.debug_rmpp) if3RmppDebugOn();
    
	IB_LOG_INFO("Device: ", fe_config.hca);
	IB_LOG_INFO("Port: ", fe_config.port);

	fe_compute_pool_size();	// compute sizes for FE resources

    if (!fe_config.start) {
        IB_LOG_WARN0("FE not configured to start");
    } else {
        (void)sm_wait_ready(VIEO_FE_MOD_ID);
        (void)fe_main();
    }
}

