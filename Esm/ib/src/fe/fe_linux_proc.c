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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include <stdio.h>
#include <unistd.h>
#include "ib_types.h"
#include "sm_l.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "fe_main.h"
#include "hsm_config_srvr_api.h"
#include "hsm_config_srvr_data.h"
#include "fm_xml.h"
#include "if3.h"
#include "mal_g.h"
#include <opamgt_priv.h>


extern void fe_compute_pool_size(void);
extern int fe_main(void);
extern int atoi(const char *ptr);
extern void fe_parse_xml_config(void);
extern void fe_init_globals(void);
extern void rmpp_sma_spoofing_check_set(uint32_t value);
extern Pool_t fe_xml_pool, fe_pool;
extern IBhandle_t fdsa;
extern size_t g_fePoolSize;
extern char msgbuf[256];

extern struct omgt_port *fe_omgt_session;

// global vars
FEXmlConfig_t fe_config;
uint8_t *mai_host_name;
uint32_t mai_host_port=4999;

// extern's for getopt.h
extern	int optind;
extern	int opterr;
extern	int optopt;
extern	char *optarg;
extern  void if3RmppDebugOn(void);
extern	void if3RmppDebugOff(void);
extern	int	if3RmppDebugGet(void);
extern void fe_set_log_level(uint32_t log_level);
extern void fe_set_log_mode(uint32_t log_mode);
extern void fe_set_log_mask(const char* mod, uint32_t mask);
extern void fe_init_log_setting(void);
extern void mai_set_default_pkey(uint16_t pKey);

static uint8_t mai_host[256];
static Thread_t conf_server;
static p_hsm_com_server_hdl_t conf_handle;


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
			snprintf(common->log_file, sizeof(common->log_file),"%s", fe_config.log_file);
			
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

    if (if3_ssl_init(NULL)) {
		IB_LOG_ERROR0("Could not initialize SSL interface");
		return(-1);
    }

	snprintf(server_path, sizeof(server_path), "%s%s",HSM_FM_SCK_PREFIX,fe_env_str);

	if(hcom_server_init(&conf_handle,server_path,5,1024,&fe_conf_callback) != HSM_COM_OK){
		IB_LOG_ERROR0("Could not allocate server handle");
		return(-1);
	}

	status = vs_thread_create (&conf_server, (void *)"RT-Conf (FE)", fe_conf_server_run,
							  0,NULL,256 * 1024);

	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create remote configuration thread");
	}

	return 0;
}

void fe_init_log_setting(void){
	vs_log_control(VS_LOG_SETFACILITY, (void *)(unint)getFacility(fe_config.syslog_facility, /* test */ 0), (void *)0, (void *)0);
	vs_log_control(VS_LOG_SETMASK, fe_log_masks,(void*)(uintn)fe_config.log_level, (void *)(unint)fe_log_to_console);
	if(strlen(fe_config.name) > 0)
		vs_log_control(VS_LOG_SETSYSLOGNAME, (void *)fe_config.name, (void *)0, (void *)0);
	else
		vs_log_control(VS_LOG_SETSYSLOGNAME, (void *)"fm_fe", (void *)0, (void *)0);
	if(strlen(fe_config.log_file) > 0) {
		vs_log_control(VS_LOG_SETOUTPUTFILE, (void *)fe_config.log_file, (void *)0, (void *)0);
		if(fe_config.log_level > 0)
			omgt_set_err(fe_omgt_session, vs_log_get_logfile_fd());
		else
			omgt_set_err(fe_omgt_session, NULL);

		if(fe_config.log_level > 2)
			omgt_set_dbg(fe_omgt_session, vs_log_get_logfile_fd());
		else
			omgt_set_dbg(fe_omgt_session, NULL);
    } else {
		vs_log_control(VS_LOG_SETOUTPUTFILE, (void *)0, (void *)0, (void *)0);

		if(fe_config.log_level > 0)
			omgt_set_err(fe_omgt_session, OMGT_DBG_FILE_SYSLOG);
		else
			omgt_set_err(fe_omgt_session, NULL);

		if(fe_config.log_level > 2)
			omgt_set_dbg(fe_omgt_session, OMGT_DBG_FILE_SYSLOG);
		else
			omgt_set_dbg(fe_omgt_session, NULL);
    }

	vs_log_set_log_mode(fe_config.syslog_mode);

	vs_log_control(VS_LOG_STARTSYSLOG, (void *)0, (void *)0, (void *)0);

	vs_init_coredump_settings("FE", fe_config.CoreDumpLimit, fe_config.CoreDumpDir);

	// Now that FE is its own process, do not have access to the SM's anti-spoofing switch
	// Default to enabled.
    (void)rmpp_sma_spoofing_check_set(1);
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
		cs_log_set_log_mask(mod, mask, fe_log_masks);
		fe_init_log_setting();
	}
}

Status_t fe_main_init_port(void)
{
	Status_t status = VSTATUS_OK;

	FILE * tmp_log_file = strlen(fe_config.log_file) > 0 ? vs_log_get_logfile_fd() : OMGT_DBG_FILE_SYSLOG;
	struct omgt_params params = {.error_file = fe_config.log_level > 0 ? tmp_log_file : NULL,
		                         .debug_file = fe_config.log_level > 2 ? tmp_log_file : NULL};
	status = ib_init_devport(&fe_config.hca, &fe_config.port, (uint64_t *)&fe_config.port_guid, &params);

	if (status != VSTATUS_OK)
		Shutdown = 1;

	return status;
}

Status_t fe_main_register_fe(int queue_size)
{
    Status_t status = VSTATUS_OK;

    status = ib_register_fe(queue_size, 0);
    if (status != VSTATUS_OK)
        Shutdown = 1;

    return status;
}

uint8_t if3_is_master(void) 
{ 
    return 1; 
}

uint8_t fe_is_thread(void) 
{ 
    return 0; 
}

Status_t
if3_set_rmpp_minfo (ManagerInfo_t *mi)
{
    mi->rmppMngrfd = &fdsa;
    mi->rmppPool = &fe_pool;
    // FE does not receive inbound MAD requests, so RMPP filters not required
    mi->rmppCreateFilters = 0;
    // set the RMPP context pool sizes for the FE process, with the same techniques
    // used to define the RMPP context pool sizes for the SA.
    mi->rmppDataLength = 512 * cs_numPortRecords(fe_config.subnet_size);
    mi->rmppMaxCntxt = 2 * fe_config.subnet_size;
    return VSTATUS_OK;
}

int
smValidateGsiMadPKey(Mai_t *maip, uint8_t mgmntAllowedRequired, uint8_t antiSpoof)
{ 
    // Currently, the FE always uses the M_PKEY to communicate with the SA, PA,
    // and PM.  The response should be the M_PKEY.  Usage of the mgmntAllowedRequired
    // and antiSpoof is not applicable to the FE, because it does not have access to
    // the SA repository.
    return (maip->addrInfo.pkey == STL_DEFAULT_FM_PKEY) ? 1 : 0;
}

/*****************************************************************************
*
* FUNCTION
*       help
*
* DESCRIPTION
*       prints out SYNTAX error/help message
*
* INPUTS
*
* OUTPUTS
*
******************************************************************************/
static void help(uint8_t *progName) {
    IB_ENTER(__func__, 0, 0, 0, 0); 

    if ((strcmp((void *)progName, "fab_exsim")) == 0) {
        printf("fab_exsim - Fabric Executive Simulator Server\n");
        printf("Syntax: fab_exsim [ -d <ibdev> -m <port> -n <login> \n"); 
        printf("\t    -p <ibport> ] \n");
        printf("\t    -d   Specify iba device to open\n");
        printf("\t    -m   Specify port number to listen on\n");
        printf("\t    -n   Turn login off\n");
        printf("\t    -p   Specify iba port on the device\n");
    }
    else if ((strcmp((void *)progName, "fe")) == 0) {
        printf("fe - Fabric Executive Control Server\n");
        printf("Syntax: fe [ -d <ibdev> -e <env> -m <port> -n <login> -X <filename>\n"); 
        printf("\t    -p <ibport> ] \n");
        printf("\t    -d   Specify iba device to open\n");
        printf("\t    -e   Specify startup environment\n");
        printf("\t    -m   Specify port number to listen on\n");
        printf("\t    -n   Turn login off \n");
        printf("\t    -p   Specify iba port on the device\n");
        printf("\t    -X   config_filename\n");
    }
    else if ((strcmp((void *)progName, "fab_tsim")) == 0) {
        printf("fab_tsim - Fabric Executive Test Server\n");
        printf("Syntax: fab_tsim [ -n <login> -m <port> ] \n");
        printf("        -m	Specify port number to listen on\n");
        printf("        -n	Turn login off \n");
    }

    printf("            -h    prints this help message\n");

    IB_EXIT(__func__, 0); 
    exit(1);
}

static Status_t
initFeXmlMemoryPool(void) {

	uint32_t fe_xml_bytes;

	fe_xml_bytes = xml_compute_pool_size(/* one instance of fe */ 0);

	memset(&fe_xml_pool, 0, sizeof(fe_xml_pool));
	return vs_pool_create(&fe_xml_pool, 0, (void *)"FAB_EXEC_XML", NULL, fe_xml_bytes);
}

void*
getFeXmlParserMemory(uint32_t size, char* info) {
	void        *address;
	Status_t    status;

#ifdef XML_MEMORY
	printf("called getFeXmlParserMemory() size (%u) (%s) from fe_main.c\n", size, info);
#endif
	status = vs_pool_alloc(&fe_xml_pool, size, (void*)&address);
	if (status != VSTATUS_OK || !address)
		return NULL;
	return address;
}

void
freeFeXmlParserMemory(void *address, uint32_t size, char* info) {

#ifdef XML_MEMORY
	printf("called freeFeXmlParserMemory() size (%u) (%s) from fe_main.c\n", size, info);
#endif
	vs_pool_free(&fe_xml_pool, address);
}

int main(int argc, char *argv[]){
    char        *prog, *p;
    int32_t     op;
    int32_t	traceMask;
    Status_t    status;

    /*
     * Set default values
     */

    pm_lid          = 0;   
    dm_lid          = 0;    
    ConnDisconnect  = FALSE;
    Shutdown        = FALSE;

    /* Get the program name */
    prog = argv[0];
    p = prog;

    while (*p) {
        if (*p++ == '/') {
           prog = p;
        }
    }

	memset(mai_host,0,sizeof(mai_host)); /* Simulation host name          */
    mai_host_name = &mai_host[0];
    mai_host_name = NULL;

    /*
     * Get the instance argument from the command line.
     */

    // surpress getopt() error messages
    opterr = 0;

    snprintf(fe_env_str, sizeof(fe_env_str), "fe_0");

    while ((op = getopt (argc, argv, "De:E:hH:l:C:X:")) != -1) {
        switch (op) {
        case 'D':
			fe_nodaemon = 0;
			break;
        case 'e':
        case 'E':
            strncpy (fe_env_str, optarg, 20);
            break;
        case 'h':
        case 'H':
            help((void *)prog);
            exit(1);
		case 'l':
			sscanf(optarg, "%u", (uint32_t *)&fe_log_level_arg);
			fe_log_level_override = 1;
			break;
		case 'C':
			sscanf(optarg, "%u", (uint32_t *)&fe_log_to_console);
			break;
		case 'X':
			sscanf(optarg, "%255s", fe_config_filename);
			break;

        }
    }

	// initialize memory pool for FE XML parsing
	status = initFeXmlMemoryPool();
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("initFeXmlMemoryPool: fe_xml_pool vs_pool_create failure");
	}

	// init callback function for XML parser so it can get pool memory
	initXmlPoolGetCallback(&getFeXmlParserMemory);
	initXmlPoolFreeCallback(&freeFeXmlParserMemory);

	// Parse the XML configuration
	fe_parse_xml_config();

	fe_config.subnet_size = MAX(fe_config.subnet_size, MIN_SUPPORTED_ENDPORTS);
	mai_set_default_pkey(STL_DEFAULT_FM_PKEY);
	
    // enable rmpp debug messages if desired
    if (fe_config.debug_rmpp) if3RmppDebugOn();


    // reset getopt()
    optind = 1;

    while ((op = getopt(argc,argv,"d:m:M:n:N:p:P:")) != EOF) {
        switch (op) {
            case 'd':
                fe_config.hca = atoi(optarg);
                break;
            case 'm':
            case 'M':
                fe_config.listen = atoi(optarg);
                break;
            case 'n':
            case 'N':
                fe_config.login = 0;
                break;
            case 'p':
            case 'P':
                fe_config.port = atoi(optarg);
                break;
            case 't':
            case 'T':
                sscanf(optarg, "0x%08x", (uint32_t *)&traceMask);
		        IB_SET_LOG_MASK(traceMask);
                break;
        }
    }

	fe_compute_pool_size();	// compute sizes for FE resources

    fe_main();
    
    return 0;
}


