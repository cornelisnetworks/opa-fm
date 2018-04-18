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

/************************************************************************
* 
* FILE NAME
*   fe_main.c
*
* DESCRIPTION
*   Main control module for Fabric Executive manager
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   01/17/2000      Initial version for checkin to source control
* jrw   12/05/2001      Changes from code review 
*
***********************************************************************/

#include "fe_main.h"
#include "fe_mad.h"
#include "vs_g.h"
#include "if3.h"
#include "ib_sa.h"
#ifndef __VXWORKS__
#endif

#include <fm_xml.h>

#ifndef MAX
#define	MAX(a,b) (((a)>(b))?(a):(b))
#endif

#ifdef CAL_IBACCESS
#include "cal_ibaccess_g.h"
#endif

#include "mal_g.h"
#ifdef IB_STACK_OPENIB
#include "fe_trap_thread.h"
#include "opamgt_sa_priv.h"
#endif

#ifndef __VXWORKS__
#include <code_version.h>
#endif

#include "iba/ib_rmpp.h"

#define SA_CHECK_TIMEOUT    10000000        /* Timeout check SA in micr sec */

/* Declare Globals for feproc   */
size_t   		g_fePoolSize;
NetConnection   *dconn          = NULL;    /* Ptr to debug connection       */
NetConnection   *conn           = NULL;    /* Anchor ptr to conn structs    */
FE_ConnList     *clist          = NULL;    /* Pointer to connection list    */
IBhandle_t      fdsa,fd_pm,fd_dm;          /* Manager Handles               */
uint8_t         *g_fe_oob_send_buf;                 /* Ptr to net send buffer        */
STL_LID         pm_lid;                    /* Lid of PM                     */
STL_LID         dm_lid;                    /* Lid of DM                     */
STL_LID         ea_lid;                    /* Lid of EA                     */
int32_t         ConnDisconnect  = FALSE;   /* Disconnect connection flag    */
int32_t         Shutdown        = FALSE;   /* Orderly shutdown flag         */
int32_t         fe_nodaemon = 1;           /* daemonize or not              */
extern Pool_t   fe_pool;                   /* Mem pool for FE defined net.c */
uint8_t         *g_fe_recv_buf   = NULL;   /* FE IB data input buffer       */
extern uint32_t	fe_in_buff_size;
extern uint8_t  *fe_prts;
extern uint32_t	fe_prts_size;
extern uint8_t  *fe_lnks;
extern uint32_t	fe_lnks_size;

uint32_t		fe_log_level_override = 0;
uint32_t		fe_log_level_arg = 0;
uint32_t		fe_log_masks[VIEO_LAST_MOD_ID+1];
uint32_t		fe_log_to_console = 0;
char			fe_config_filename[256];
char			fe_env_str[32];

#ifndef __VXWORKS__
char msgbuf[256]={0};
#endif

#ifdef IB_STACK_OPENIB
struct omgt_port *fe_omgt_session;
#endif

extern Status_t fe_main_init_port(void);
extern Status_t fe_main_register_fe(int queue_size);

// XML configuration data structure
extern FEXmlConfig_t fe_config;
#ifdef __VXWORKS__
extern void if3RmppDebugOn(void);
#endif

// XML debug tracing
static uint32_t xml_trace = 0;

#ifndef __VXWORKS__
Pool_t   fe_xml_pool;               /* Mem pool for FE XML parsing defined net.c */
#endif


#ifdef __LINUX__
extern int fe_conf_server_init(void);
extern void fe_init_log_setting(void);
#endif

void fe_main_kill(void)
{
	Shutdown = TRUE;
    // give the FE thread a chance to shutdown
    vs_thread_sleep(VTIMER_1S * 2);
}

static int fe_init_failure(char *msg, int status)
{
	sysPrintf("%s (status %d)\n", msg, status);
	IB_LOG_ERROR_FMT(__func__, "%s (status %d)", msg, status);
	return status;
}

void fe_process_config(FMXmlCompositeConfig_t *xml_config, uint32_t fe_instance) {

#ifndef __VXWORKS__
    uint32_t modid;
#endif

    if (xml_config->xmlDebug.xml_fe_debug) {
        printf("###########fe_env %s fe_instance %u\n", fe_env_str, (unsigned int)fe_instance);
        xml_trace = 1;
    } else {
        xml_trace = 0;
    }

    if (fe_config.subnet_size > MAX_SUBNET_SIZE) {
        IB_LOG_INFO_FMT(__func__, "subnet size is being adjusted from %u to %u", fe_config.subnet_size, MAX_SUBNET_SIZE);
        fe_config.subnet_size = MAX_SUBNET_SIZE;
    }

#ifndef __VXWORKS__
    if (fe_log_level_override) {
        // command line override
        cs_log_set_log_masks(fe_log_level_arg, fe_config.syslog_mode, fe_log_masks);
    } else {
        for (modid = 0; modid <= VIEO_LAST_MOD_ID; ++modid)
            fe_log_masks[modid] = fe_config.log_masks[modid].value;
    }
#endif
    // display configured values if debug is turned on
    if (xml_trace) {
        feShowConfig(&fe_config);
    }

#ifdef __VXWORKS__
    if (fe_config.debug_rmpp)
        if3RmppDebugOn();
#endif
}

// Parse the XML configuration
void fe_parse_xml_config(void) {

#ifndef __VXWORKS__
	FMXmlCompositeConfig_t *xml_config;
#else
	extern FMXmlCompositeConfig_t *xml_config;
#endif
	uint32_t fe_instance;

    // The instance for now is the last character in the string fe_0 - fe_3 in the
    // fe_env_str variable. For now get it out of there so we have an integer instance.
    fe_instance = atoi((char*)&fe_env_str[3]);
    if (fe_instance >= MAX_INSTANCES) fe_instance = MAX_INSTANCES-1;

#ifndef __VXWORKS__
    // if we cannot parse correctly the default values as mentioned above should still work
    xml_config = parseFmConfig(fe_config_filename, IXML_PARSER_FLAG_NONE, fe_instance, /* full parse */ 0, /* embedded */ 0);
    if (!xml_config || !xml_config->fm_instance[fe_instance]) {
        IB_FATAL_ERROR_NODUMP("FE: unable to read configuration file");
    }

    // copy the configuration to local structures
    fe_config = xml_config->fm_instance[fe_instance]->fe_config;
#endif
	fe_process_config(xml_config, fe_instance);

	// if __VXWORKS__ we didn't allocate the xml_config, but we are responsible for releasing it
    releaseXmlConfig(xml_config, /* full */ 1);

#ifndef __VXWORKS__
    // for debugging XML do not deamonize
    if (xml_trace)
        fe_nodaemon = 1;
#endif
}

void fe_init_globals(void)
{
    //
    //	Initialize some variables before they're used.
    //
}

uint32_t fe_process_passthrough(uint8_t *netbuf, int buflen, FE_ConnList *clist) {
	OOBPacket* message; 				/* Pointer to the incoming packet */
	uint32_t rc; 						/* Return code */
	
	IB_ENTER(__func__, netbuf, clist, 0, 0);
	IB_LOG_INFO0("Passthrough packet received");

	rc = FE_SUCCESS;

	if(!clist) {
		rc = FE_FATAL;
		return rc;
	}

	/* If the buffer is empty, return */
	if(netbuf == NULL){
		rc = FE_NO_COMPLETE;
		return rc;
	}

	/* First do a simple sanity check on the OOB header */
	message= ((OOBPacket *) netbuf);

    BSWAP_OOB_HEADER(&(message->Header));
    if (message->Header.HeaderVersion != STL_BASE_VERSION || message->Header.Length > buflen) {
		return FE_NO_COMPLETE;
    }

	/* OOB header appears reasonable, so continue to process the message  */
    BSWAP_MAD_HEADER((MAD*) &(message->MadData));


	/* Process message type and route to the apropriate manager */
	switch(message->MadData.common.MgmtClass) {
	case MCLASS_SUBN_ADM: 			/* SA */
		rc = fe_sa_passthrough(netbuf, clist, fdsa);
		break;
	case MCLASS_VFI_PM: 			/* PA */
		rc = fe_pa_passthrough(netbuf, clist, fdsa);
		break;
	default:
		rc = FE_NO_COMPLETE;
		break;
	}

	IB_EXIT(__func__, rc);
	return(rc);
}

int fe_main()
{
    uint32_t        error; 
    FE_ConnList     *nlist      = NULL;     
    FE_ConnList     *tlist; 
    NetConnection   *newconn    = NULL; 
    char            *netbuf; 
    int             buflen; 
    uint64_t        timenow; 
    uint64_t        endtime; 
    int             sa_valid_con = FALSE; 
    int             rc; 
    uint32_t        conn_state   = CONN_UNCHANGED; 
    uint16_t        saHasMoved = 0; 
    
    IB_ENTER(__func__, 0, 0, 0, 0); 
    
#ifdef __VXWORKS__
    // read configuration data from XML
    fe_parse_xml_config(); 
#endif
    
    fdsa            = INVALID_HANDLE; 
    fd_pm           = INVALID_HANDLE; 
    fd_dm           = INVALID_HANDLE; 
    
    // setup the first timeout
    vs_time_get(&timenow); 
    endtime = timenow + SA_CHECK_TIMEOUT; 
    
#ifndef __VXWORKS__
    fe_init_log_setting(); 
#endif
    vs_log_output_message("Fabric Executive starting up.", TRUE); 
#ifndef __VXWORKS__
    vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID, NULL, NULL,
                    "FE: Version: %s", GetCodeVersion());
#endif
    
    
#ifndef __VXWORKS__
    if (!fe_nodaemon) {
        int	ret_value; 
        IB_LOG_INFO("Trying daemon, fe_nodaemon =", fe_nodaemon); 
        if ((ret_value = daemon(1, 0))) {
            int localerrno = errno; 
            IB_LOG_ERROR("daemon failed with return value of", ret_value); 
            IB_LOG_ERROR(strerror(localerrno), localerrno);
        }
    }
    
    fe_conf_server_init(); 
#endif
    
    if (fe_main_init_port() != VSTATUS_OK)
        IB_FATAL_ERROR_NODUMP("fe_main: Failed to bind to device; terminating; status"); 

#if defined(__LINUX__)
    // normally ib_init_devport or ib_init_guid would return portGuid,
    // hca and port.  However for unified SM
    // the SM calls it and updates sm_portGuid AFTER config for FE has been
    // checked.  technically we should keep one variable for config and one
    // for where we are running.  Now that we are past config check and SM
    // has initialized interface to port, we can use the port information
    // of the SM so that FE log messages are accurate.
	FILE * tmp_log_file = strlen(fe_config.log_file) > 0 ? vs_log_get_logfile_fd() : OMGT_DBG_FILE_SYSLOG;
	struct omgt_params params = {.error_file = fe_config.log_level > 0 ? tmp_log_file : NULL,
	                             .debug_file = fe_config.log_level > 2 ? tmp_log_file : NULL};
	if (omgt_open_port_by_guid(&fe_omgt_session, fe_config.port_guid, &params) != 0)
        IB_FATAL_ERROR("fe_main: Failed to init notice registration; terminating");
#elif defined(__VXWORKS__)
	// for CAL_IBACCESS, this call just returns values, does not open IbAccess
	if (VSTATUS_OK != ib_init_devport(&fe_config.hca, &fe_config.port, &fe_config.port_guid))
		IB_FATAL_ERROR_NODUMP("fe_main: Failed to bind to device; terminating");
#endif
    
    if (fe_main_register_fe(512) != VSTATUS_OK) {
        IB_FATAL_ERROR_NODUMP("fe_main: Failed to register management classes; terminating"); 
        Shutdown = 1;
    }
    
#if defined(IB_STACK_OPENIB)
    // spawn off a thread to deal with the synchronous notice API
    if (fe_trap_thread_create() != VSTATUS_OK) 
        IB_FATAL_ERROR_NODUMP("bm_main: Failed to launch trap thread; terminating; terminating"); 
#endif
    
    // initialize FE task
    if (!Shutdown && (error = fe_init())) {
        fe_init_failure("Fatal Error on fe_init, terminating", error); 
        Shutdown = 1;           // bail
    }

    // set flag to indicate that the connection to the SA is valid, in order
    // to avoid repeating registering with the SA unnessarily.
    if (fdsa != INVALID_HANDLE)
        sa_valid_con = TRUE; 
    
#ifndef __VXWORKS__
    if (xml_trace) 
        fprintf(stdout, "\nFE Initial Config Done\n"); 
#endif
    
    while (!Shutdown) {
        
        if ((newconn = fe_net_process(SLEEP_TIME, BLOCK))) {    /* check for new */
            // add new connection to list
            error = vs_pool_alloc(&fe_pool, sizeof(FE_ConnList), (void *)&nlist); 
            if (error != VSTATUS_OK) {
                IB_LOG_ERRORRC("Malloc for nlist rc:", error); 
                // disconnect connection and free the memory
                fe_net_disconnect(newconn, NULL); 
                fe_net_free_connection(newconn); 
                break;
            }
            
            // allocate input buffer associated with the connection
            error = vs_pool_alloc(&fe_pool, fe_in_buff_size, (void *)&newconn->fe_in_buff); 
            if (error != VSTATUS_OK) {
                IB_LOG_ERRORRC("Malloc for fe_in_buff rc:", error); 
                // deallocate connection list entry
                vs_pool_free(&fe_pool, nlist); 
                // disconnect connection and free the memory
                fe_net_disconnect(newconn, NULL); 
                fe_net_free_connection(newconn); 
                break;
            }
            
            nlist->prev = NULL; 
            nlist->next = NULL; 
            nlist->conn = newconn; 
            
            // check if login was disabled from the command line
            if (fe_config.login) {
                nlist->state = UNLOGGED;
            } else {
                nlist->state = ACTIVE;
            }
            if (!clist) {
                clist = nlist;
            } else {
                clist->prev = nlist; 
                nlist->next = clist; 
                clist = nlist;
            }
            
            IB_LOG_INFO0("New Connection accepted"); 
#ifdef DEBUG
            tlist = clist; 
            while (tlist) {
                IB_LOG_INFO("tlist->conn", tlist->conn); 
                tlist = tlist->next;
            }
#endif
        }
        
        
        //
        // check queue for new msgs
        tlist = clist; 
        while (tlist && !Shutdown) {
            fe_net_get_next_message(tlist->conn, &netbuf, &buflen, NULL); 
            if (netbuf) 
				if ((error = fe_process_passthrough((void *)netbuf, buflen, tlist))) {
                    if (error == FE_UNLOGGED) {
                        IB_LOG_ERROR0("Connection not logged in");
                    } else {
                        IB_LOG_ERROR("ProcCmd error:", error);
                    }
                }
            fe_net_free_buf(netbuf); 
            tlist = tlist->next;
        }
        
        // clean up any aborted conn
        if (ConnDisconnect) {
            tlist = clist; 
            while (tlist) {
                if (tlist->state == (BAD_CONN)) {
                    if (dconn == tlist->conn) 
                        dconn = NULL; 
                    
                    nlist = tlist; 
                    if (tlist->next) 
                        tlist->next->prev = tlist->prev; 
                    if (tlist->prev) 
                        tlist->prev->next = tlist->next; 
                    if (tlist->prev == NULL) 
                        clist = tlist->next; 
                    
                    fe_net_free_connection(tlist->conn); 
                    tlist = tlist->next; 
                    
                    vs_pool_free(&fe_pool, nlist);
                } else 
                    tlist = tlist->next;
            }
            ConnDisconnect = FALSE;
        }
        
        vs_time_get(&timenow);
        
        //
        // check on SA, and PM
        if (timenow > endtime && !Shutdown) {
            endtime = timenow + SA_CHECK_TIMEOUT; 
            saHasMoved = 0; 
            
            // see if SM/SA has moved on us
            if (fdsa != INVALID_HANDLE && (((rc = if3_check_sa(fdsa, TRUE, &saHasMoved)) != VSTATUS_OK) || ((rc = fe_if3_sa_check()) != VSTATUS_OK))) {
                // something wrong with lower layers, cleanup and go to open loop
                IB_LOG_INFINI_INFORC("Connection to SA lost, will try to reconnect later. rc:", rc); 
                if (sa_valid_con) {
                    conn_state = CONN_LOST;
                }
                sa_valid_con = FALSE; 
                fe_if3_unsubscribe_sa(FALSE);  // this will just delete the trap filters
                if3_close(fdsa);    // release management structure and related file descriptors
                fdsa = INVALID_HANDLE; 
                endtime = timenow + fe_config.manager_check_rate; 
                continue;
            } else if (saHasMoved) {
                // new SM/SA, update and PM/PA Management Info structures
                if (fd_pm != INVALID_HANDLE) {
                    IB_LOG_INFINI_INFO0("SM/SA have moved, closing connection to PM"); 
                    if3_close_mngr_cnx(fd_pm); 
                    fd_pm = INVALID_HANDLE;
                }

                
                // close the file descriptor used for sa comm
                sa_valid_con = FALSE; 
                fe_if3_unsubscribe_sa(FALSE);  // this will just delete the trap filters
                if3_close(fdsa);    // release management structure and related file descriptors
                fdsa = INVALID_HANDLE; 
                endtime = timenow + fe_config.manager_check_rate; 
                IB_LOG_INFINI_INFO0("SM/SA have moved, will resync if a few seconds"); 
                continue;
            // check on PM
            }

            if (fd_pm != INVALID_HANDLE && if3_cntrl_cmd_send(fd_pm, FE_MNGR_PROBE_CMD) != VSTATUS_OK){
                IB_LOG_INFINI_INFO0("connection to PM is lost, will try to reconnect later"); 
                if3_close_mngr_cnx(fd_pm); 
                fd_pm = INVALID_HANDLE; 
                endtime = timenow + fe_config.manager_check_rate; 
                continue;
            }

            
            if (Shutdown) 
                break; 

            // Attempt to reconnect to each of the managers if we don't have a valid connection
            if (!sa_valid_con) {
                fdsa = INVALID_HANDLE; 
                IB_LOG_VERBOSE("trying to re-register FE with SA", 0); 
                rc = if3_register_fe(fe_config.hca, fe_config.port, (void *)FE_SERVICE_NAME, 
                                     FE_SERVICE_ID, IF3_REGFORCE_PORT, &fdsa); 
                if (rc != VSTATUS_OK) {
                    IB_LOG_VERBOSE("Failed to reconnect to SA, will try later", rc); 
                    fdsa = INVALID_HANDLE;
					endtime = timenow + fe_config.manager_check_rate;
                } else {
					rc = fe_if3_subscribe_sa();
					if (rc != FSUCCESS) {
						IB_LOG_ERROR_FMT(__func__, "Failed to subscribe for traps in SA status: %u", rc);
						rc = FE_NO_RETRIEVE;
					}
                    sa_valid_con = TRUE; 
                    conn_state = CONN_ESTABLISH; 
                    IB_LOG_INFINI_INFO("Successful reconnect to SA", rc);
                }
            }
            if (fd_pm == INVALID_HANDLE) {
				IB_LOG_VERBOSE("trying to re-register FE with PM", 0);

                // reset connection to PM
				if(if3_sid_mngr_cnx(fe_config.hca, fe_config.port, (void *)STL_PM_SERVICE_NAME, STL_PM_SERVICE_ID, MAD_CV_VFI_PM, &fd_pm) == FSUCCESS){
                    IB_LOG_INFINI_INFO("connection to PM reinitialized, fd_pm=", fd_pm);
				} else {
					IB_LOG_VERBOSE("Failed to reconnect to PM, will try again later", 0);
					endtime = timenow + fe_config.manager_check_rate;
				}
            }

        }   // end time check
        
        // keep out of time check area to catch traps at full speed
        if (fdsa != INVALID_HANDLE && !Shutdown) {
            fe_unsolicited(clist, &conn_state);
        }
    }
    
    //
    // perform shutdown of the FE
    fe_shutdown(); 
    
#ifdef IB_STACK_OPENIB
    omgt_close_port(fe_omgt_session); 
#endif
    
#ifdef __VXWORKS__
    //
    // reset all the globals
    dconn = NULL;   // ptr to debug connection
    conn = NULL;    // anchor ptr to conn structs
    clist = NULL;   // pointer to connection list
    fdsa = INVALID_HANDLE; 
    fd_pm = INVALID_HANDLE; 
    fd_dm = INVALID_HANDLE; 
    g_fe_oob_send_buf = 0;   // ptr to net send buffer
    pm_lid = 0;        // Lid of PM
    dm_lid = 0;        // Lid of DM 
    ConnDisconnect = FALSE;   // disconnect connection flag
    Shutdown = FALSE;  // orderly shutdown flag
    
#endif
    IB_LOG_INFINI_INFO0("FE Task exiting OK."); 
    
    IB_EXIT(__func__, 0); 
    return (0);
}

void fe_callBack(NetConnection *bad_conn) {
    FE_ConnList *tlist;
    char addrstr[100] = {0};

    IB_ENTER(__func__, bad_conn, 0, 0, 0);

    (void) fe_net_inet_ntop(bad_conn, addrstr, sizeof(addrstr));
    IB_LOG_INFOX("Connection disconnected from", addrstr);

    /* Find the bad conn and mark   */
    tlist = clist;
    while (tlist) {
        if (tlist->conn == bad_conn) {
            ConnDisconnect = TRUE;   
            tlist->state = BAD_CONN;
            IB_EXIT(__func__, 0); 
            return;
        }
        tlist = tlist->next;
    }

    IB_EXIT(__func__, 0); 
    return;
}

void fe_compute_pool_size(void)
{
    uint32_t buf_recv_size, buf_oob_send_size;
     
    //
    // compute size of FE pool and other resources based on subnet_size
	fe_in_buff_size = cs_numNodeRecords(fe_config.subnet_size) * (sizeof(STL_NODE_RECORD) + Calculate_Padding(sizeof(STL_NODE_RECORD)));
	fe_prts_size    = cs_numPortRecords(fe_config.subnet_size) * (sizeof(STL_PORTINFO_RECORD) + Calculate_Padding(sizeof(STL_PORTINFO_RECORD)));
	fe_lnks_size    = cs_numLinkRecords(fe_config.subnet_size) * (sizeof(STL_LINK_RECORD) + Calculate_Padding(sizeof(STL_LINK_RECORD)));

    // compute size of the two primary buffers utilized by the FE that consume
    // an extensive amount of memory, for now keep it simple and make them the
    // same size.
    buf_recv_size = fe_in_buff_size + fe_prts_size + fe_lnks_size;      // receive inband buffer 
    buf_oob_send_size = fe_in_buff_size + fe_prts_size + fe_lnks_size;  // OOB send buffer

#ifdef __VXWORKS__
	g_fePoolSize = 4000 * 1024;  /* 4MB should be enough currently */
#else
	// this really doesn't matter since only VxWorks pools are limited in size
			// add an extra 10% to be safe
	g_fePoolSize = ((buf_recv_size + buf_oob_send_size)*11)/10;
	// in addition to in_buff, prts and lnks, the pool is used for:
	//		MFT for 1 switch (FV query - 1 per FV)
	//		LFT for 1 switch (FV query - 1 per FV))
	//		SLtoVL for 1 port (FV query - 1 per FV))
	//		List of all active FV groups (read or write by FV)
	//		FE_ConnList (1 per FV connection)
#endif
}

uint32_t fe_init(void)
{
    uint32_t        rc = SUCCESS;
    NetError        err;
	char buf[140];

    IB_ENTER(__func__, 0, 0, 0, 0); 

	sprintf(buf, "FE: Size Limits: EndNodePorts=%u Nodes=%u Ports=%u Links=%u",
				(unsigned)fe_config.subnet_size,
				(unsigned)cs_numNodeRecords(fe_config.subnet_size),
				(unsigned)cs_numPortRecords(fe_config.subnet_size),
				(unsigned)cs_numLinkRecords(fe_config.subnet_size));
	vs_log_output_message(buf, FALSE);

	sprintf(buf, "FE: Memory: Pool=%uK NodeBuf=%uK PrtsBuff=%uK, LinksBuf=%uK",
				(unsigned)(g_fePoolSize+1023)/1024, (unsigned)(fe_in_buff_size+1023)/1024,
				(unsigned)(fe_prts_size+1023)/1024, (unsigned)(fe_lnks_size+1023)/1024);
	vs_log_output_message(buf, FALSE);

    // create a memory pool to use for FE
    if ((rc = vs_pool_create(&fe_pool,0,(void *)"FAB_EXEC",NULL,g_fePoolSize)) != VSTATUS_OK) {
        IB_FATAL_ERROR_NODUMP("fe_init: fe_pool vs_pool_create failure");
		return 1;
	}

     // allocate the FE buffers from the fe_pool
    if ((rc = vs_pool_alloc(&fe_pool, STL_BUF_OOB_SEND_SIZE, (void *)&g_fe_oob_send_buf)) != VSTATUS_OK) {
        IB_FATAL_ERROR_NODUMP("fe_init: failed to allocate g_fe_oob_send_buf from fe_pool");
    } else if ((rc = vs_pool_alloc(&fe_pool, STL_BUF_RECV_SIZE, (void *)&g_fe_recv_buf)) != VSTATUS_OK) {
        IB_FATAL_ERROR_NODUMP("fe_init: failed to allocate g_fe_recv_buf from fe_pool");
    }
    
#ifndef __VXWORKS__
	sprintf(buf, "FE: Using: HFI %u Port %u PortGuid "FMT_U64,
			(unsigned)fe_config.hca + 1, (unsigned)fe_config.port, fe_config.port_guid);
	vs_log_output_message(buf, FALSE);
#endif

	// initialize IB management system
    if ((rc = fe_vieo_init(NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFORC("some errors during initialization in fe_vieo_init, will retry what failed later. rc:", rc);
		rc = SUCCESS;
	}

    if (rc == SUCCESS) {
        // initialize the network
        if (fe_net_init(fe_config.listen, &err, fe_callBack)) {
            rc = VSTATUS_BAD;
            IB_LOG_INFINI_INFORC("NetConnect err, could not setup listen port for FE. rc:", rc);
        }
    }

    IB_EXIT(__func__, rc); 
    return rc;
}

void feDebugOn(void) {
    fe_config.debug = 1;
}

void feDebugOff(void) {
    fe_config.debug = 0;
}

uint32_t feDebugGet(void)
{
	return fe_config.debug;
}

