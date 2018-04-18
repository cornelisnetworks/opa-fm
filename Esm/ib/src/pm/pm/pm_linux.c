/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

/************************************************************************
* 
* FILE NAME
*      pma_main.c
*
* DESCRIPTION
*  Linux start file for PMA
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
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "pm_l.h"
#include "signal.h"
#include "hsm_config_srvr_api.h"
#include "hsm_config_srvr_data.h"
#include <fm_xml.h>
#include "sm_l.h"
#include "pm_counters.h"
#include "pm_topology.h"



extern void     pm_compute_pool_size(void);
extern void     pm_init_globals(void);
extern int      pm_main(void);
extern void     if3RmppDebugOn(void);
extern void     if3RmppDebugOff(void);
extern int 		if3RmppDebugGet(void);
extern void 	pm_init_log_setting(void);
extern void     pmApplyLogLevelChange(PMXmlConfig_t* new_pm_config);
extern Status_t pm_main_kill(void);
extern void 	mai_set_default_pkey(uint16_t pKey);
extern uint8_t sm_isReady(void);
extern Status_t pm_get_xml_config(void);

void pm_init_log_setting(void);

extern FMXmlConfig_t fm_config;
extern int          pm_interval;
extern int32_t      pm_nodaemon;

extern uint32_t		pm_log_level_override;
extern uint32_t		pm_log_masks[VIEO_LAST_MOD_ID+1];
extern uint32_t		pm_log_to_console;
extern  char		pm_config_filename[256];
extern uint8_t		pm_env_str[32];
extern uint32_t	    pm_conf_start;

extern size_t			g_pmPoolSize;

static Thread_t		conf_server; // Runtime configuration server thread...

static p_hsm_com_server_hdl_t	conf_handle;

fm_msg_ret_code_t
pm_conf_fill_common(fm_mgr_action_t action, fm_config_common_t *common)
{
	int reset_log_flag = 0;
	switch(action){
		case FM_ACT_GET:
			common->debug 		= 0;
			common->debug_rmpp	= if3RmppDebugGet();
			common->device		= pm_config.hca;
			common->port		= pm_config.port;
			common->pool_size	= g_pmPoolSize;
			common->log_filter	= 0; // pm_log_filter; no longer allowed
			common->log_level	= pm_config.log_level;
			common->log_mask	= 0; // pm_log_mask; no longer allowed
			common->nodaemon	= pm_nodaemon;
			strncpy(common->log_file,pm_config.log_file,strlen(pm_config.log_file));
			
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
				pm_config.log_level = common->log_level;
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
				pm_log_filter = common->log_filter;
				reset_log_flag = 1;
			}
			if(common->select_mask & CFG_COM_SEL_LOG_MASK){
				pm_log_mask = common->log_mask;
				reset_log_flag = 1;
			}
#endif
			if(common->select_mask & CFG_COM_SEL_LOG_FILE){
				strncpy(pm_config.log_file,common->log_file,sizeof(pm_config.log_file)-1);
				pm_config.log_file[sizeof(pm_config.log_file)-1]=0;
				reset_log_flag = 1;
			}

			if(reset_log_flag)
				pm_init_log_setting();

			return FM_RET_OK;

				

		default:

			return FM_RET_ACT_NOT_SUPPORTED;

			break;
	}

	return FM_RET_INVALID;


}


fm_msg_ret_code_t
pm_conf_fill_pm_conf(fm_mgr_action_t action, pm_config_t *config)
{
switch(action){
		case FM_ACT_GET:

			config->priority	= pm_config.priority;
			config->timer 		= pm_interval;
			
			return FM_RET_OK;
		case FM_ACT_SUP_GET:
			
			config->select_mask = CFG_PM_SEL_ALL;
			return FM_RET_OK;

		case FM_ACT_SUP_SET:

			config->select_mask = CFG_PM_SEL_ALL;
			return FM_RET_OK;

		case FM_ACT_SET:
			if(config->select_mask & CFG_PM_SEL_PRIORITY){
				pm_config.priority = config->priority;
			}
			if(config->select_mask & CFG_PM_SEL_TIMER){
				pm_interval = config->timer;
			}

			return FM_RET_OK;

				

		default:

			return FM_RET_ACT_NOT_SUPPORTED;

			break;
	}

	return FM_RET_INVALID;
}





hsm_com_errno_t 
pm_conf_callback(hsm_com_datagram_t *data)
{
	fm_config_datagram_t *msg;
	char * tmpData = NULL;
	int length = 0;

	printf("pm_conf_callback: LEN: %d, BUFLEN: %d \n",
		   data->data_len,data->buf_size);

	if(data->data_len >= sizeof(msg->header)){
		msg = (fm_config_datagram_t*)data->buf;

		switch (msg->header.data_id) {
		case FM_DT_COMMON:
			if (msg->header.data_len != sizeof(fm_config_common_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = pm_conf_fill_common(msg->header.action,(fm_config_common_t*)&msg->data[0]);
			}
			break;
		case FM_DT_PM_CFG:
			if (msg->header.data_len != sizeof(pm_config_t)) {
				msg->header.ret_code = FM_RET_BAD_LEN;
			} else {
				msg->header.ret_code = pm_conf_fill_pm_conf(msg->header.action,(pm_config_t*)&msg->data[0]);
			}
			break;

		case FM_DT_PM_RESTORE_PRIORITY:
		case FM_DT_LOG_LEVEL:
		case FM_DT_LOG_MODE:
		case FM_DT_LOG_MASK:
			msg->header.ret_code = FM_RET_UNKNOWN_DT;
			break;

		case FM_DT_DEBUG_TOGGLE:
			if (pmDebugGet()) {
				pmDebugOff();
				IB_LOG_INFINI_INFO0("Turning OFF Debug for PM");
			} else {
				pmDebugOn();
				IB_LOG_INFINI_INFO0("Turning ON Debug for PM");
			}
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_RMPP_DEBUG_TOGGLE:
			if (if3RmppDebugGet()) {
				IB_LOG_INFINI_INFO0("turning rmpp debug off for PM");
				if3RmppDebugOff();
			} else {
				IB_LOG_INFINI_INFO0("turning rmpp debug on for PM");
				if3RmppDebugOn();
			}
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_PM_GET_COUNTERS:
			tmpData = pm_print_counters_to_buf();

			printf("%s\n", tmpData);
			if (tmpData != NULL) {
				length = strlen(tmpData) + 1;

				memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
				msg->data[msg->header.data_len - 1] = '\0';
				/* PR 116307 - buffer returned from pm_print_counters_to_buf() is from
				 * sm_pool, so free the buffer from smpool.
				 */
				vs_pool_free(&sm_pool, tmpData);
			}
			msg->header.ret_code = FM_RET_OK;
			break;

		case FM_DT_PM_RESET_COUNTERS:
			pm_reset_counters();
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
pm_conf_server_run(uint32_t argc, uint8_t **argv){
	errno = 0;
	if(hcom_server_start(conf_handle) != HSM_COM_OK){
		if (errno != 0) {
			IB_LOG_ERROR_FMT("pm_conf_server_run", "Command server exited with error \"%s\" (%d)",
				strerror(errno), errno);
		} else {
			IB_LOG_ERROR0("Command server exited with error. No further information.");
		}
	}
}

int
pm_conf_server_init(void){
	char server_path[64];
	int	status;

	memset(server_path,0,sizeof(server_path));


	snprintf(server_path, sizeof(server_path), "%s%s", HSM_FM_SCK_PREFIX, pm_env_str);

	if(hcom_server_init(&conf_handle,server_path,5,32768,&pm_conf_callback) != HSM_COM_OK){
		IB_LOG_ERROR0("Could not allocate server handle");
		return(-1);
	}

	status =vs_thread_create (&conf_server, (void *)"RT-Conf (PM)", pm_conf_server_run,
							  0,NULL,256*1024);

	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create remote configuration thread");
	}

	return 0;
}

// This routine parses the PM configuration and initializes some of its global
// variables.
Status_t
pm_initialize_config(void)
{
	Status_t rc;

    //
    //	Set default values
    //	
	pm_interval          = 60;
	
    //	Initialize the other global variables that are used by PM and are common with VxWorks
    //	This needs to be done here especially for the shutdown flag, because
    //	there will be a race for that flag, if the SM is stopped just after starting.
    //	Its possible that the signal handler runs before pm_main() and set the shutdown
    //	flag to 0. If the globals are initialized in pm_main(), it will overwrite the
    //	flag value that is set by pm_shutdown() and the PM would not shutdown cleanly.
    //
    //	For the same reason this also needs to be done before setting the signal handler
	(void)pm_init_globals(); // unrelated to config

	// -e option information comes from sm's -e option
	// translate sm_env ("sm_#") to pm_env_str ("pm_#")
	// we use this to identify our instance and hence which Fm section of config
	memset (pm_env_str, 0, sizeof(pm_env_str));
	cs_strlcpy ((void *)pm_env_str, (void*)sm_env,sizeof(pm_env_str));
	pm_env_str[0]='p';

	// -X option information comes from sm's -X option
	cs_strlcpy(pm_config_filename, sm_config_filename, sizeof(pm_config_filename));
	
    //
    //	Get the environment variables before applying the command line overrides.
    //	If this fails, we still continue because we have default values that we can us.
	// sm_main has already called read_info_file()
    //
    if ((rc = pm_get_xml_config()) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "can't retrieve PM XML configuration");
        return rc;
    }

    if (pm_config.start) {
		// ensure that the primary PM configuration parameters match those of the SM (if PM enabled)
		if (!sm_isValidDeviceConfigSettings(VIEO_PM_MOD_ID, pm_config.hca, pm_config.port, pm_config.port_guid)) {
			(void)sm_getDeviceConfigSettings(&pm_config.hca, &pm_config.port, &pm_config.port_guid);
		}
		if (!sm_isValidLogConfigSettings(VIEO_PM_MOD_ID, pm_config.log_level, pm_config.syslog_mode, pm_log_masks, pm_config.log_file, pm_config.syslog_facility)) {
			(void)sm_getLogConfigSettings(&pm_config.log_level, &pm_config.syslog_mode, pm_log_masks,
										pm_config.log_file, pm_config.syslog_facility);
		}
		if (!sm_isValidMasterConfigSettings(VIEO_PM_MOD_ID, pm_config.priority, pm_config.elevated_priority)) {
			(void)sm_getMasterConfigSettings(&pm_config.priority, &pm_config.elevated_priority);
		}
	}

	return VSTATUS_OK;
}

// This requires the caller to have invoked pm_initialize_config prior to this
// call.
// It completes PM initialization and then starts the actual PM.
void
unified_sm_pm(uint32_t argc, uint8_t ** argv)
{
	mai_set_default_pkey(STL_DEFAULT_FM_PKEY);

    if (pm_config.debug_rmpp) if3RmppDebugOn();
    
	IB_LOG_INFO("Device: ", pm_config.hca);
	IB_LOG_INFO("Port: ", pm_config.port);

	pm_compute_pool_size();	// compute sizes for PM resources

    if (!pm_config.start) {
        IB_LOG_WARN0("PM not configured to start");
    } else {
        (void)sm_wait_ready(VIEO_PM_MOD_ID);
        (void)pm_main();
    }
}
