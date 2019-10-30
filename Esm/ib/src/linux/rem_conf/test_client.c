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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <signal.h>
#include "hsm_com_lib.h"
#include "hsm_com_api.h"
#include "hsm_config_api.h"

static int s_print_unsupported = 1;


/* Define a macro to print out the value or 'unsupported' */

#define HSM_P_VAR(FORMAT,NAME,VAL,MASK,SET_BIT) 			\
	do{														\
		if(MASK & SET_BIT){									\
			printf(FORMAT,NAME,VAL);						\
		}else if(s_print_unsupported){												\
			printf("\t%15s : %s\n",NAME,"---");		\
		}													\
	}while(0);												


void
usage(char *cmd)
{
	printf("%s instance\n",cmd);
}

void 
dunp_common_info(fm_config_common_t *common_data, char *mgr)
{
	printf("\n\nCommon Data for %s \n",mgr);
	HSM_P_VAR("\t%15s : %lu\n","DEVICE",common_data->device,common_data->select_mask,CFG_COM_SEL_DEVICE);
	HSM_P_VAR("\t%15s : %lu\n","PORT",common_data->port,common_data->select_mask,CFG_COM_SEL_PORT);
	HSM_P_VAR("\t%15s : %lu\n","POOL SIZE",common_data->pool_size,common_data->select_mask,CFG_COM_SEL_POOL_SIZE);
	HSM_P_VAR("\t%15s : %d\n","DEBUG",common_data->debug,common_data->select_mask,CFG_COM_SEL_DEBUG);
	HSM_P_VAR("\t%15s : %d\n","NODAEMON",common_data->nodaemon,common_data->select_mask,CFG_COM_SEL_NODAEMON);
	HSM_P_VAR("\t%15s : %s\n","LOG FILE",common_data->log_file,common_data->select_mask,CFG_COM_SEL_LOG_FILE);
	HSM_P_VAR("\t%15s : %d\n","LOG LEVEL",common_data->log_level,common_data->select_mask,CFG_COM_SEL_LOG_LEVEL);

}

void
dump_bm_config(bm_config_t *bm_config)
{
	unsigned long long bkey;

	memcpy(&bkey,bm_config->bkey,sizeof(bkey));
	printf("\n\nBM Configuration Data\n");
	HSM_P_VAR("\t%15s : %016llx\n","BKEY",bkey,bm_config->select_mask,CFG_BM_SEL_BKEY);
	HSM_P_VAR("\t%15s : %lu\n","BKEY LEASE",bm_config->bkey_lease,bm_config->select_mask,CFG_BM_SEL_BKEY_LEASE);
	HSM_P_VAR("\t%15s : %d\n","PRIORITY",bm_config->priority,bm_config->select_mask,CFG_BM_SEL_PRIORITY);
	HSM_P_VAR("\t%15s : %d\n","TIMER",bm_config->timer,bm_config->select_mask,CFG_BM_SEL_TIMER);

}

void
dump_pm_config(pm_config_t *pm_config)
{

	printf("\n\nPM Configuration Data\n");
	HSM_P_VAR("\t%15s : %lu\n","PRIORITY",pm_config->priority,pm_config->select_mask,CFG_PM_SEL_PRIORITY);
	HSM_P_VAR("\t%15s : %u\n","TIMER",pm_config->timer,pm_config->select_mask,CFG_PM_SEL_TIMER);

}

void
dump_fe_config(fe_config_t *fe_config)
{
	printf("\n\nFE Configuration Data\n");
	HSM_P_VAR("\t%15s : %lu\n","LISTEN",fe_config->listen,fe_config->select_mask,CFG_FE_SEL_LISTEN);
	HSM_P_VAR("\t%15s : %d\n","LOGIN",fe_config->login,fe_config->select_mask,CFG_FE_SEL_LOGIN);
	HSM_P_VAR("\t%15s : %d\n","PRIORITY",fe_config->priority,fe_config->select_mask,CFG_FE_SEL_PRIORITY);

}



void
dump_sm_config(sm_config_t *sm_config)
{
	unsigned long long key;

	memcpy(&key,sm_config->key,sizeof(key));
	printf("\n\nSM Configuration Data\n");
	HSM_P_VAR("\t%15s : %016llx\n","KEY",key,sm_config->select_mask,CFG_SM_SEL_KEY);
	HSM_P_VAR("\t%15s : %lu\n","PRIORITY",sm_config->priority,sm_config->select_mask,CFG_SM_SEL_PRIORITY);
	HSM_P_VAR("\t%15s : %d\n","TIMER",sm_config->timer,sm_config->select_mask,CFG_SM_SEL_TIMER);
	HSM_P_VAR("\t%15s : %d\n","MAX RETRIES",sm_config->max_retries,sm_config->select_mask,CFG_SM_SEL_MAX_RETRY);
	HSM_P_VAR("\t%15s : %d\n","RCV WAIT MSEC",sm_config->rcv_wait_msec,sm_config->select_mask,CFG_SM_SEL_RCV_WAIT_MSEC);
	HSM_P_VAR("\t%15s : %d\n","SWITCH LIFETIME",sm_config->switch_lifetime,sm_config->select_mask,CFG_SM_SEL_SW_LFTIME);
	HSM_P_VAR("\t%15s : %d\n","HOQ LIFE",sm_config->hoq_life,sm_config->select_mask,CFG_SM_SEL_HOQ_LIFE);
	HSM_P_VAR("\t%15s : %d\n","VL STALL",sm_config->vl_stall,sm_config->select_mask,CFG_SM_SEL_VL_STALL);
	HSM_P_VAR("\t%15s : %d\n","SA RESP TIME",sm_config->sa_resp_time,sm_config->select_mask,CFG_SM_SEL_SA_RESP_TIME);
	HSM_P_VAR("\t%15s : %d\n","SA PKT LIFETIME",sm_config->sa_packet_lifetime,sm_config->select_mask,CFG_SM_SEL_SA_PKT_LIFETIME);
	HSM_P_VAR("\t%15s : %d\n","LID",sm_config->lid,sm_config->select_mask,CFG_SM_SEL_LID);
	HSM_P_VAR("\t%15s : %d\n","LMC",sm_config->lmc,sm_config->select_mask,CFG_SM_SEL_LMC);
	HSM_P_VAR("\t%15s : %d\n","PKEY SUPPORT",sm_config->pkey_support,sm_config->select_mask,CFG_SM_SEL_PKEY_SUPPORT);


}


int
main(int argc, char *argv[])
{
	p_fm_config_conx_hdlt	hdl;
	int						instance;
	fm_config_common_t		common_data;
	sm_config_t 			sm_config;
	bm_config_t				bm_config;
	pm_config_t				pm_config;
	fe_config_t				fe_config;
	fm_mgr_config_errno_t	res;
	fm_msg_ret_code_t		ret_code;
	char					*rem_addr = NULL;
	char					*community = "public";

	memset(&common_data,0,sizeof(common_data));

	if(argc < 2){
		usage(argv[0]);
		return(-1);
	}

	instance = atol(argv[1]);

	if(argc >= 3)
		rem_addr = argv[2];

	if(argc >= 4)
		rem_addr = argv[3];

	printf("Connecting to instance: %d\n",instance);

	if((res = fm_mgr_config_init(&hdl,instance, rem_addr, community)) != FM_CONF_OK){

		printf("Failed to initialize the client handle: %d\n", res);
		return res;
	}

	if((res = fm_mgr_config_connect(hdl)) != FM_CONF_OK)
	{
		printf("Failed to connect: %d\n",res);
		return res;
	}

	// SM Data ----------------------------------
	memset(&common_data,0,sizeof(common_data));
	common_data.select_mask = CFG_COM_SEL_ALL;
	if((res = fm_mgr_commong_cfg_query(hdl,FM_MGR_SM,FM_ACT_GET,&common_data,&ret_code)) != FM_CONF_OK)
	{
		printf("Failed to retrieve data: \n\tError:(%d) %s \n\tRet code:(%d) %s\n",
			   res, fm_mgr_get_error_str(res),ret_code,fm_mgr_get_resp_error_str(ret_code));
		//return res;
	}else{
		dunp_common_info(&common_data,"SM");
	}

	memset(&sm_config,0,sizeof(sm_config));
	sm_config.select_mask = CFG_SM_SEL_ALL;
	if((res = fm_mgr_sm_cfg_query(hdl,FM_ACT_GET,&sm_config,&ret_code)) != FM_CONF_OK)
	{
		printf("Failed to retrieve data: \n\tError:(%d) %s \n\tRet code:(%d) %s\n",
			   res, fm_mgr_get_error_str(res),ret_code,fm_mgr_get_resp_error_str(ret_code));
		//return res;
	}else{
		dump_sm_config(&sm_config);
	}

	// PM Data ------------------------------------
	memset(&common_data,0,sizeof(common_data));
	common_data.select_mask = CFG_COM_SEL_ALL;
	if((res = fm_mgr_commong_cfg_query(hdl,FM_MGR_PM,FM_ACT_GET,&common_data,&ret_code)) != FM_CONF_OK)
	{
		printf("Failed to retrieve data: \n\tError:(%d) %s \n\tRet code:(%d) %s\n",
			   res, fm_mgr_get_error_str(res),ret_code,fm_mgr_get_resp_error_str(ret_code));
		//return res;
	}else{
		dunp_common_info(&common_data,"PM");
	}

	memset(&pm_config,0,sizeof(pm_config));
	pm_config.select_mask = CFG_PM_SEL_ALL;
	if((res = fm_mgr_pm_cfg_query(hdl,FM_ACT_GET,&pm_config,&ret_code)) != FM_CONF_OK)
	{
		printf("Failed to retrieve data: \n\tError:(%d) %s \n\tRet code:(%d) %s\n",
			   res, fm_mgr_get_error_str(res),ret_code,fm_mgr_get_resp_error_str(ret_code));
		//return res;
	}else{
		dump_pm_config(&pm_config);
	}

	// FE Data -------------------------------------
	memset(&common_data,0,sizeof(common_data));
	common_data.select_mask = CFG_COM_SEL_ALL;
	if((res = fm_mgr_commong_cfg_query(hdl,FM_MGR_FE,FM_ACT_GET,&common_data,&ret_code)) != FM_CONF_OK)
	{
		printf("Failed to retrieve data: \n\tError:(%d) %s \n\tRet code:(%d) %s\n",
			   res, fm_mgr_get_error_str(res),ret_code,fm_mgr_get_resp_error_str(ret_code));
		//return res;
	}else{
		dunp_common_info(&common_data,"FE");
	}

	memset(&fe_config,0,sizeof(fe_config));
	fe_config.select_mask = CFG_FE_SEL_ALL;
	if((res = fm_mgr_fe_cfg_query(hdl,FM_ACT_GET,&fe_config,&ret_code)) != FM_CONF_OK)
	{
		printf("Failed to retrieve data: \n\tError:(%d) %s \n\tRet code:(%d) %s\n",
			   res, fm_mgr_get_error_str(res),ret_code,fm_mgr_get_resp_error_str(ret_code));
		//return res;
	}else{
		dump_fe_config(&fe_config);
	}


	return 0;


}
