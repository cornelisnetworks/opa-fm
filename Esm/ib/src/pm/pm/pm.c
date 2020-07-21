/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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
#include <ib_types.h>
#include <ib_mad.h>
#include <ib_status.h>
#include <cs_g.h>
#include <mai_g.h>
#include <ib_sa.h>
#include <ib_mad.h>
#include <if3.h>
#include "pm_l.h"
#include "pa_l.h"
#include "vs_g.h"
#include "vfi_g.h"
#include "pm_topology.h"
#include <fm_xml.h>

#ifdef CAL_IBACCESS
#include "cal_ibaccess_g.h"
#endif

#include "mal_g.h"
#include "sm_l.h"
#include "sm_dbsync.h"
#include "pm_topology.h"
#include "iba/stl_pa_priv.h"

#ifdef __LINUX__
#define static
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

extern int pm_conf_server_init(void);
int pm_conf_server_init(void);
extern void mai_umadt_read_kill(void);
#endif

#ifdef __VXWORKS__
#include <stdio.h>

extern STATUS esm_getPartitionActualSize(size_t *mbytes);
extern STATUS esm_getPartitionSize(size_t *mbytes);
#endif

// XML configuration data structure
extern FMXmlCompositeConfig_t *xml_config;
extern PMXmlConfig_t pm_config;
extern SMXmlConfig_t sm_config;
extern void if3RmppDebugOn(void);

// XML debug tracing
static uint32_t xml_trace;

// Instance of this PM
uint32_t		pm_instance;

extern Status_t pm_firstpass_comp(void);

uint32_t g_pmDebugPerf = 0;
int             pm_main(void);
/* PM Globals */
size_t				g_pmPoolSize;
IBhandle_t          pm_fd = -1;
IBhandle_t          hpma = 0;
static IBhandle_t   fh  = -1;
int			        pm_shutdown = 0;
int                 pm_interval;
// lease time for FreezeFrame, if idle more than this, discard Freeze
Pool_t              pm_pool;
uint8_t             mclass = MAD_CV_PERF;
uint8_t             vfi_mclass = MAD_CV_VFI_PM;
int                 master = 0;  /* Master PM */
int32_t             pm_nodaemon = 1;           /* daemonize or not */

uint32_t			pm_log_level_override = 0;
uint32_t			pm_log_masks[VIEO_LAST_MOD_ID+1];
uint32_t			pm_log_to_console = 0;
char				pm_config_filename[256];
uint8_t				pm_env_str[32];

static uint8_t  *gdata;
uint32_t 		pm_engineDebug = 0;

// XM config consistency checking
uint32_t    pm_overall_checksum = 0;
uint32_t    pm_consistency_checksum = 0;

uint32_t		pm_master_consistency_check_level = DEFAULT_CCC_LEVEL;
uint32_t		sm_master_consistency_check_level = DEFAULT_CCC_LEVEL;
int		        pm_inconsistency_posted = FALSE;

#define EXIT(a) exit (a)

extern Status_t vfi_GetPortGuid(ManagerInfo_t * fp, uint32_t gididx);

#define SID_ASCII_FORMAT "%02x%02x%02x%02x%02x%02x%02x%02x"
static IB_SERVICE_RECORD  pmServRer;
#ifndef __VXWORKS__
static char msgbuf[256];
#endif


#if 0
/*
 * Convert Service ID to ASCII
 */
static void
SidToAscii(uint8_t * string, uint64_t sid)
{
    uint64_t        temp;
    uint8_t        *cp = (uint8_t *) & temp;
    uint8_t         buff[17];

    temp = Endian64((sid));

    sprintf((buff), SID_ASCII_FORMAT,
            cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]);

    memcpy((string), buff, 16);

    return;
}
#endif

// Parse the XML configuration - note that with the unified SM that the PM configuration data was already read by SM
void pm_parse_xml_config(void) {

	// The instance for now is the last character in the string pm_0 - pm_3 in the
	// pm_env_str variable. For now get it out of there so we have an integer instance.
	pm_instance = atoi((char*)&pm_env_str[3]);

	if (xml_config->xmlDebug.xml_pm_debug) {
		printf("###########pm_env %s pm_instance %u\n", pm_env_str, (unsigned int)pm_instance);
		xml_trace = 1;
	} else {
		xml_trace = 0;
	}

	// clear inconsistency flag
	pm_inconsistency_posted = FALSE;

#ifndef __VXWORKS__
	// on VxWorks, these parameters come from SM, TBD - verify consistency
	if (pm_log_level_override) {
		// command line override
		cs_log_set_log_masks(pm_config.log_level, pm_config.syslog_mode, pm_log_masks);
	} else {
		uint32_t modid;
		for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid)
			pm_log_masks[modid] = pm_config.log_masks[modid].value;
	}
#endif
#ifdef __SIMULATOR__
	pm_interval = 900;
#else
	pm_interval = pm_config.timer;
#endif

	// adjust MAX Clients
	if (pm_config.max_clients > PM_MAX_MAX_CLIENTS) {
		IB_LOG_WARN_FMT( __func__, "MaxClients %u is too large, setting to %d", 
				pm_config.max_clients, PM_MAX_MAX_CLIENTS);
		pm_config.max_clients = PM_MAX_MAX_CLIENTS;
	}
	if (pm_config.max_clients < PM_MIN_MAX_CLIENTS) {
		IB_LOG_WARN_FMT(__func__, "MaxClients %u is too small, setting to %d", 
				pm_config.max_clients, PM_MIN_MAX_CLIENTS);
		pm_config.max_clients = PM_MIN_MAX_CLIENTS;
	}

	// adjust Freese Frame settings
	if (pm_config.freeze_frame_images < PM_MIN_FF_IMAGES) {
		IB_LOG_WARN_FMT( __func__, "FreezeFrameImages %u is too small, setting to %d", 
				pm_config.freeze_frame_images, PM_MIN_FF_IMAGES);
		pm_config.freeze_frame_images = PM_MIN_FF_IMAGES;
	}
	if (pm_config.freeze_frame_images > PM_MAX_FF_IMAGES) {
		IB_LOG_WARN_FMT( __func__, "FreezeFrameImages %u is too large, setting to %d", 
				pm_config.freeze_frame_images, PM_MAX_FF_IMAGES);
		pm_config.freeze_frame_images = PM_MAX_FF_IMAGES;
	}
	if (pm_config.freeze_frame_images < ((2 * pm_config.max_clients) - 1)) {
		IB_LOG_WARN_FMT( __func__, "FreezeFrameImages %u is not enough to satisfy maximum PA clients, setting to %u", 
				pm_config.freeze_frame_images, (int)((2 * pm_config.max_clients) - 1));
		pm_config.freeze_frame_images = (2 * pm_config.max_clients) - 1;
	}

	// adjust total images
	if (pm_config.total_images < PM_MIN_TOTAL_IMAGES) {
		IB_LOG_WARN_FMT( __func__, "TotalImages %u is too small, setting to %d", 
				pm_config.total_images, PM_MIN_TOTAL_IMAGES);
		pm_config.total_images = PM_MIN_TOTAL_IMAGES;
	}
	if (pm_config.total_images > PM_MAX_TOTAL_IMAGES) {
			IB_LOG_WARN_FMT( __func__, "TotalImages %u is too large, setting to %d", 
				pm_config.total_images, PM_MAX_TOTAL_IMAGES);
		pm_config.total_images = PM_MAX_TOTAL_IMAGES;
	}
	if (pm_config.total_images < pm_config.freeze_frame_images+2 ) {
		IB_LOG_WARN_FMT( __func__, "TotalImages %u is too small as compared to number of freeze frames, setting to %d", 
				pm_config.total_images, pm_config.freeze_frame_images+2);
		pm_config.total_images = pm_config.freeze_frame_images+2;
	}

	// TBD - simply use sm_numEndPorts for unified SM?
    if (pm_config.subnet_size > MAX_SUBNET_SIZE) {
		IB_LOG_INFO_FMT( __func__, "subnet size is being adjusted from %u to %u", pm_config.subnet_size, MAX_SUBNET_SIZE);
		pm_config.subnet_size = MAX_SUBNET_SIZE;
	}
	if (pm_config.subnet_size < MIN_SUPPORTED_ENDPORTS) {
		IB_LOG_WARN_FMT( __func__, "subnet size of %d is too small, setting to %d", 
				pm_config.subnet_size, MIN_SUPPORTED_ENDPORTS);
		pm_config.subnet_size = MIN_SUPPORTED_ENDPORTS;

	}

	if (xml_trace) {
		pmShowConfig(&pm_config);
	}

	if (xml_trace) {
		printf("XML - PM old overall_checksum %llu new overall_checksum %llu\n",
			(long long unsigned int)pm_overall_checksum, (long long unsigned int)pm_config.overall_checksum);
		printf("XML - PM old consistency_checksum %llu new consistency_checksum %llu\n",
			(long long unsigned int)pm_consistency_checksum, (long long unsigned int)pm_config.consistency_checksum);
	}
	pm_overall_checksum = pm_config.overall_checksum;
	pm_consistency_checksum = pm_config.consistency_checksum;

#ifdef __VXWORKS__
	if (pm_config.debug_rmpp) 
		if3RmppDebugOn();
#endif

#ifndef __VXWORKS__
	// for debugging XML do not deamonize
	if (xml_trace)
		pm_nodaemon = 1;
#endif
}

/*
 * Build Service Record
 */
static void
BuildRecord(IB_SERVICE_RECORD * srp, uint8_t * servName, uint64_t servID,
            uint16_t flags, uint32_t lease, uint8_t pmVersion, uint8_t pmState)
{

    srp->ServiceLease = lease;
    srp->ServiceData16[0] = flags;
    strncpy((void *)srp->ServiceName, (void *)servName, sizeof(srp->ServiceName));
    srp->RID.ServiceID = servID;
    //SidToAscii((srp->id), servID);

	// encode PM version and state into first bytes of data8
    srp->ServiceData8[0] = pmVersion;
    srp->ServiceData8[1] = pmState;

	// add checksums to first 3 words of data32 and the consistency check levels to 
	// the bytes 2 and 3 of data8 if we are configured to do so
	// This is now just informational. PM doesn't validate checksums.
	// That is left to the SM who will decide if we need to go inactive.
    	srp->ServiceData32[0] = pm_config.consistency_checksum;
    	srp->ServiceData32[1] = sm_config.consistency_checksum;

		srp->ServiceData32[2] = 0;

		srp->ServiceData8[2] = pm_config.config_consistency_check_level;
		srp->ServiceData8[3] = sm_config.config_consistency_check_level;
		// supply the current XM checksum version
		srp->ServiceData8[4] = FM_PROTOCOL_VERSION;

	srp->Reserved = 0;
}

/*
 * Used to select the master service record when more than one PM is
 * registered with the SA
 */
static int
select_master_compare(IB_SERVICE_RECORD *sr1, IB_SERVICE_RECORD *sr2)
{
    unsigned i;

    // compare priorities
    if (sr1->ServiceData16[0] < sr2->ServiceData16[0])
        return 1;
    else if (sr1->ServiceData16[0] > sr2->ServiceData16[0])
        return -1;

    // compare guids
    for (i = 8; i < 16; ++i) {
        if (sr1->RID.ServiceGID.Raw[i] < sr2->RID.ServiceGID.Raw[i])
            return -1;
        else if (sr1->RID.ServiceGID.Raw[i] > sr2->RID.ServiceGID.Raw[i])
            return 1;
    }

    return 0;
}

/******************************************************************
 *
 *
 *
 *******************************************************************/
#define MAX_PATH_RECORDS_EXPECTED 1     // we only want one record back
Status_t doRegisterPM(uint8_t masterPm)
{
    uint32_t rc;
    uint64_t lease;
    uint16_t flags;
    uint8_t pmState = (masterPm) ? STL_PM_MASTER : STL_PM_STANDBY;
    uint64_t pmSrvId = (masterPm) ? STL_PM_SERVICE_ID : STL_PM_SERVICE_ID_SEC;
    char *pmSrvNm = (masterPm) ? STL_PM_SERVICE_NAME : STL_PM_SERVICE_NAME_SEC;


    lease = 2 * pm_interval;
    // PM priority the same as the SM priority
	vs_lock(&smRecords.smLock);
    flags = pm_config.priority = sm_smInfo.u.s.Priority;
	vs_unlock(&smRecords.smLock);

    BuildRecord(&pmServRer, (void *)pmSrvNm, pmSrvId, flags, lease,
			    PmEngineRunning() ? STL_PM_VERSION : 0,
				PmEngineRunning() ? pmState : 0);
    rc = vfi_mngr_register(pm_fd, vfi_mclass, VFI_DEFAULT_GUID, &pmServRer,
                           VFI_SVRREC_GID | VFI_SVREC_NAME | VFI_SVRREC_ID, VFI_REGFORCE_FABRIC);

    if (rc != VSTATUS_OK) {
        char msg[100];

        snprintf(msg, sizeof(msg), "Registration as %s did not succeed rc:", (masterPm) ? "MASTER" : "STANDBY");
        msg[sizeof(msg)-1] = 0;
        IB_LOG_VERBOSERC(msg, rc);
        return rc;
    }

    master = (masterPm) ? 1 : 0;

    return rc;
}

Status_t
registerPM(void)
{
    int             count = MAX_PATH_RECORDS_EXPECTED;
    IB_PATH_RECORD  pathr[MAX_PATH_RECORDS_EXPECTED];
    uint16_t        flags=0,registered_priority=0;
    uint64_t        sguid, dguid;   
    uint64_t        lease;
    uint32_t        rc;

    IB_ENTER(__func__,master,0,0,0);

    /*
     * find master Performance Manager
     */
    flags = pm_config.priority;
    lease = 2 * pm_interval;

    memset((void *)&pmServRer, 0, sizeof(pmServRer));
    memset((void *)&pathr[0], 0, sizeof(pathr));
    BuildRecord(&pmServRer, (void *)STL_PM_SERVICE_NAME, STL_PM_SERVICE_ID, flags, lease, STL_PM_VERSION, STL_PM_MASTER);

    rc = vfi_mngr_find_cmp(pm_fd, vfi_mclass,
                           VFI_LMC_BASELID, &pmServRer,
                           VFI_SVREC_NAME | VFI_SVRREC_ID, &count,
                           pathr, select_master_compare);
    if (rc != VSTATUS_OK && rc != VSTATUS_NOT_FOUND) {
        IB_LOG_VERBOSERC("Failed while retrieving service record rc:", rc);
        IB_EXIT(__func__, rc);
        return rc;
    }

    if (count == 0 || rc == VSTATUS_NOT_FOUND) {
        // make the PM master, only if the SM is master. NOTE, if the SM is not
        // master,then we may be dealing with a legacy SM where the PM may or
        // may not be configured to run.
        rc = doRegisterPM(sm_isMaster());
        IB_LOG_VERBOSE0("Registered as master");
        IB_EXIT(__func__, rc);
        return rc;
    }

    IB_LOG_VERBOSE("vfi_mngr_find found existing PMs = ", count);

    /*
     * found a master, may be dealing with a legacy PM, so just in case compare
     * priority
     */
    registered_priority = pmServRer.ServiceData16[0];

	// if master SM then reset post
	if (sm_isMaster())
		pm_inconsistency_posted = FALSE;

    sguid = pm_config.port_guid;
	dguid = pmServRer.RID.ServiceGID.Type.Global.InterfaceID;

    if (IB_LOG_IS_INTERESTED(VS_LOG_VERBOSE)) {
        IB_LOG_VERBOSE("Our      PRI = ", pm_config.priority);
        IB_LOG_VERBOSE("Remote's PRI = ", registered_priority);
        IB_LOG_VERBOSELX("source      GUID ", sguid);
        IB_LOG_VERBOSELX("destination GUID ", dguid);
    }
    memset((void *)&pmServRer, 0, sizeof(pmServRer));

    if (dguid == 0) {
        IB_LOG_ERROR0("existing PM is invalid! SA query must have failed");
        registered_priority = 0xffff;
        dguid = 0xffffffffffffffffull;
    }

    if (sm_isMaster()) {
        IB_LOG_VERBOSE0("PM attempting to register as MASTER ");
        // may be dealing with a legacy PM, so just in case make this PM's
        // priority greater in order to avoid confusion.
        if ((sguid != dguid) && registered_priority > pm_config.priority)
            pm_config.priority = registered_priority + 1;
        rc = doRegisterPM(TRUE);
        IB_EXIT(__func__, rc);
    } else {
        IB_LOG_VERBOSE0("PM attempting to register as STANDBY");
        // may be dealing with a legacy PM, so just in case make this PM's
        // priority less in order to avoid confusion.
        if ((sguid != dguid) && registered_priority <= pm_config.priority)
            pm_config.priority = registered_priority - 1;
        rc = doRegisterPM(FALSE);
        IB_EXIT(__func__, rc);
    }
    return rc;
}

#ifndef __VXWORKS__
void pm_init_log_setting(void){
    // check log related configuration parameter settings, and display appropriate warning messages.
	// This is also checked during pm startup.
    if (!sm_isValidLogConfigSettings(VIEO_PM_MOD_ID, pm_config.log_level, pm_config.syslog_mode, pm_log_masks, pm_config.log_file, pm_config.syslog_facility)) {
        (void)sm_getLogConfigSettings(&pm_config.log_level, &pm_config.syslog_mode, pm_log_masks, pm_config.log_file, pm_config.syslog_facility);
    }
}

void
pmApplyLogLevelChange(PMXmlConfig_t* new_pm_config){
	pm_config.log_level = new_pm_config->log_level;
}


Status_t pm_get_xml_config(void)
{
   // Parse the XML configuration
   pm_parse_xml_config();

    return VSTATUS_OK;
}
#endif // __VXWORKS__

/* compute size of PM pool and other resources based on pm_config.subnet_size */
void
pm_compute_pool_size(void)
{
    size_t pmImagePoolSize;

    // calculate pool size for legacy support
#ifdef __VXWORKS__
    size_t mbytes1, mbytes2;

	// add a couple pages and 10% to allow for inefficiency in allocator
	g_pmPoolSize = ((PM_MAX_DATA_LEN + 2*PAGE_SIZE)*11)/10;
#else
	// doesn't really matter on linux, but parallel VxWorks algorithm
	// PAGE_SIZE not defined for linux, so just use 4096
	g_pmPoolSize = ((PM_MAX_DATA_LEN + 2*4096)*11)/10;
#endif

#if defined(__VXWORKS__) || defined(__LINUX__)
    // adjust pool size for PA support
    //pa_max_cntxt = 2 * pm_config.subnet_size;
	//pa_data_length = PA_SRV_MAX_RECORD_SZ * cs_numPortRecords(pm_config.subnet_size);
    //g_pmPoolSize += (((sizeof(pa_cntxt_t) * pa_max_cntxt)) + pa_data_length);
    pa_max_cntxt = pm_config.max_clients + 2;	// set to incr of max clients
	// assume 4K RMPP response per concurrent mgmt query
	pa_data_length = MAX((sizeof(STL_FOCUS_PORTS_RSP) * (cs_numPortRecords(pm_config.subnet_size) + 10)), 4096) * pa_max_cntxt;
    g_pmPoolSize += ((sizeof(pa_cntxt_t) + pa_data_length) * pa_max_cntxt);
#endif

	// PM Engine storage needs
	// Ideally -only applicable if Pm.SweepInterval != 0, but at this point
	// on ESM we have not yet read XML config file, so assume Pm enabled
	// also will use default Pm image counts
	pmImagePoolSize =
			// pmportp's, 1 per LID + 1 per Switch Port 1-N
		   	(sizeof(PmPort_t) + sizeof(PmPortImage_t)*(pm_config.total_images-1)) * cs_numPortRecords(pm_config.subnet_size)
			// pmnodep's, 1 per LID (1 per FI port, 1 per switch)
			+ (sizeof(PmNode_t) + sizeof(PmNodeImage_t)*(pm_config.total_images-1)) * cs_numNodeRecords(pm_config.subnet_size)
			// LidMap, 1 pointer per LID per image
			+ sizeof(PmPort_t*) * cs_numNodeRecords(pm_config.subnet_size)
					* pm_config.total_images
			// Pm.Groups and Pm.AllPorts
			+ (sizeof(PmGroup_t) + sizeof(PmGroupImage_t)* (pm_config.total_images-1)) * (PM_MAX_GROUPS+1)
			// Pm.Image
			+ sizeof(PmImage_t) * pm_config.total_images
			// Pm.history
			+ sizeof(uint32) * pm_config.total_images
			// Pm.freezeFrames
			+ sizeof(uint32) * pm_config.freeze_frame_images
			// Ports list pointers per switch node (all ports - FI ports)
		   	+ sizeof(PmPort_t*) * (cs_numPortRecords(pm_config.subnet_size) - pm_config.subnet_size)
			+ sizeof(PmDispatcherNode_t)*pm_config.MaxParallelNodes
			+ sizeof(PmDispatcherPort_t)*pm_config.MaxParallelNodes*pm_config.PmaBatchSize
			+ sizeof(cntxt_entry_t)*pm_config.MaxParallelNodes*pm_config.PmaBatchSize
			;

#ifdef __VXWORKS__
	// if the default ESM memory partition requirements are meant, then
	// keep it simple and just double image memory pool usage
    esm_getPartitionActualSize(&mbytes1);
    esm_getPartitionSize(&mbytes2);
    g_pmPoolSize += (pmImagePoolSize * ((mbytes1 >= mbytes2) ? 2 : 1));
#else
    g_pmPoolSize += pmImagePoolSize;
#endif

	if (pm_config.shortTermHistory.enable) {
		// PM Short Term History storage
		// Allocate space for all history records
		// check that imagesPerComposite isn't 0
		if (pm_config.shortTermHistory.imagesPerComposite == 0) 
			IB_FATAL_ERROR_NODUMP("Invalid Short Term History configuration, imagesPerComposite must not be 0");
		g_pmPoolSize +=
			sizeof(PmHistoryRecord_t)*((3600*pm_config.shortTermHistory.totalHistory)/(pm_config.shortTermHistory.imagesPerComposite*pm_config.sweep_interval))
			// also allocate space for the 'current' composite and the 'cached' composite
			+ 2 * (sizeof(PmCompositeImage_t) + sizeof(PmCompositeNode_t) * cs_numNodeRecords(pm_config.subnet_size) + sizeof(PmCompositePort_t) * cs_numPortRecords(pm_config.subnet_size))
			// allocate space for the 'loaded' image
			+ (sizeof(PmPort_t) + sizeof(PmPortImage_t)) * cs_numPortRecords(pm_config.subnet_size) 
			+ (sizeof(PmNode_t) + sizeof(PmNodeImage_t)) * cs_numNodeRecords(pm_config.subnet_size)
			+ sizeof(PmPort_t*) * cs_numNodeRecords(pm_config.subnet_size)
			+ (sizeof(PmGroup_t) + sizeof(PmGroupImage_t)) * (PM_MAX_GROUPS+1)
			+ (sizeof(PmVF_t) + sizeof(PmVFImage_t)) * MAX_VFABRICS
			+ sizeof(PmImage_t)
			+ sizeof(char)*PM_HISTORY_FILENAME_LEN*((3600*pm_config.shortTermHistory.totalHistory)/(pm_config.shortTermHistory.imagesPerComposite*pm_config.sweep_interval));
	}
}

void pm_init_globals(void)
{
       /* reset all file descriptors */
       pm_fd = -1;
       hpma = -1;
       fh = -1;
       pm_shutdown = 0;
}

/******************************************************************
 *
 *
 *
 *******************************************************************/
int
pm_main()
{
        int             rc=0, goOn=0;
        uint32_t        insize;
        Mai_t           mad;
        uint64_t        timenow;
        uint64_t        endtime, timeToCheckSa;
        ManagerInfo_t   *mi;
        uint16_t        newSm=0;
		int				isRegistered=0,cnt=0;
        Status_t        status;
        uint16_t        saHasMoved=0;

#ifdef __VXWORKS__
		// read configuration data from XML
		pm_parse_xml_config();
#endif

#ifndef __VXWORKS__
		pm_init_log_setting();
		// Verify PortGUID matches the SM
		pm_config.port_guid = sm_config.port_guid;
#endif
		vs_log_output_message("Performance Manager starting up.", TRUE);

#ifndef __VXWORKS__
    if (!pm_nodaemon) {
        int	ret_value;
        IB_LOG_INFO("Trying daemon, pm_nodaemon =", pm_nodaemon);
        if ((ret_value = daemon(1, 0))) {
            int localerrno = errno;
            IB_LOG_ERROR("daemon failed with return value of", ret_value);
            IB_LOG_ERROR(strerror(localerrno), localerrno);
        }
    }

	pm_conf_server_init();

#endif

	// check device related configuration parameter settings, and display
    // appropriate warning messages.  And always synch with the current device
    // settings of the SM. 
    (void)sm_getDeviceConfigSettings(&pm_config.hca, &pm_config.port, &pm_config.port_guid);

#if ! defined(IB_STACK_OPENIB)
	// normally ib_init_devport or ib_init_guid would return portGuid,
	// hca and port.  However for unified SM
	// the SM calls it and updates pm_config.port_guid AFTER config for PM has been
	// checked.  technically we should keep one variable for config and one
	// for where we are running.  Now that we are past config check and SM
	// has initialized interface to port, we can replicate the port information
	// so PM log messages are accurate.
	// for CAL_IBACCESS, this call just returns values, does not open IbAccess
	status = ib_init_devport(&pm_config.hca, &pm_config.port, &pm_config.port_guid);
	if (status != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("pm_main: Failed to bind to device; terminating");
#endif

    // register PM with the OFED stack
	if (VSTATUS_OK != (rc = ib_register_pm(512)) ) {
#if defined(IB_STACK_OPENIB)
        IB_LOG_ERRORRC("Failed to register management classes, rc:", rc);
        goto bail;
#else
		IB_FATAL_ERROR_NODUMP("Failed to register management classes; terminating");
#endif
       }
		{
			char buf[140];

			sprintf(buf, "Size Limits: EndNodePorts=%u",
							(unsigned)pm_config.subnet_size);
			vs_log_output_message(buf, FALSE);

			sprintf(buf, "Memory: Pool=%uK",
							(unsigned)(g_pmPoolSize+1023)/1024);
			vs_log_output_message(buf, FALSE);
#ifndef __VXWORKS__
			sprintf(buf, "Using: HFI %u Port %u PortGuid "FMT_U64,
					(unsigned)pm_config.hca+1, (unsigned)pm_config.port, pm_config.port_guid);
			vs_log_output_message(buf, FALSE);
#endif
		}

       /* Allocations */
       status = vs_pool_create(&pm_pool, 0, (void *)"pm_pool", NULL, g_pmPoolSize);
       if (status != VSTATUS_OK) {
           IB_LOG_ERRORRC("Failed to create PM pool rc:", status);
           return 1;
       }

       cs_log(_PM_TRACE(pm_config.debug), "PM",
              "Allocating PM global data area of size %d", PM_MAX_DATA_LEN);
       gdata = NULL;

       status = vs_pool_alloc(&pm_pool, PM_MAX_DATA_LEN, (void*)&gdata);
       if (status != VSTATUS_OK || !gdata) {
           IB_LOG_ERRORRC("Failed to allocate PM global data area rc:", status);
           return (int)status;
       }
	   
	   mai_set_num_end_ports(MIN(pm_config.subnet_size, MAI_MAX_QUEUED_DEFAULT));
       mai_init();
	   pm_init_counters();
       if ((rc = mai_open(MAI_GSI_QP, pm_config.hca, pm_config.port, &hpma)) != VSTATUS_OK) {
           IB_LOG_ERRORRC("mai_open failed rc:", rc);
       } else if ((rc = mai_filter_method(hpma,VFILTER_SHARE,MAI_TYPE_ANY,&fh,mclass,MAD_CM_GET_RESP)) != VSTATUS_OK) {
           IB_LOG_ERRORRC("can't create filter method rc:", rc);
       } else {
           /* OK to go on */
           goOn = 1;
       }
	   
#ifndef __VXWORKS__

	if (xml_trace)
		fprintf(stdout, "\nPM Initial Config Done\n");

#endif

       /*
        * Open connection to class manager
        * Stay here until connection made or shutdown
        */
       openIf3:
       if (pm_fd >= 0) {
           (void) if3_mngr_close_cnx_fe(pm_fd, FALSE);
           pm_fd = -1;
#if defined(__VXWORKS__) || defined(__LINUX__)
           // close PA related connections to PA Server
           (void) pa_mngr_close_cnx();
#endif
           // delay a little so we don't spam the log
           (void) vs_thread_sleep (VTIMER_1S*5);
       }

       while (!pm_shutdown && goOn && pm_fd < 0) {

#if defined(IB_STACK_OPENIB)
            if (!sm_isActive()) {
                IB_LOG_INFINI_INFO0("Waiting for SM to come out of NOTACTIVE state");
                (void)vs_thread_sleep(PM_CONN_INTERVAL/2);
                continue;
            }
#elif defined(__VXWORKS__)
           if (sm_isDeactivated()) {
               pm_shutdown = TRUE;
               IB_LOG_WARN_FMT( __func__, "Engine shutting down since SM is not active due to problems!");
               goto bail;
           }
#endif
           if ((rc = if3_mngr_open_cnx_fe(pm_config.hca, pm_config.port, vfi_mclass, &pm_fd)) == VSTATUS_OK) {
               while (!pm_shutdown && pm_fd > 0) {
                   if ( (if3_mngr_locate_minfo (pm_fd, &mi) == VSTATUS_OK) &&
                        (vfi_GetPortGuid(mi, VFI_DEFAULT_GUID) == VSTATUS_OK) ) {
                       break;
                   } else {
                       IB_LOG_INFINI_INFO0("Can't get port GUID from SA, will try later");
                      goto openIf3;
                   }
                   (void) vs_thread_sleep (PM_CONN_INTERVAL/2);
                   /* catch the inevitable smLid change */
                   if ((status = if3_check_sa(pm_fd, TRUE, (uint16_t*)&newSm)) != VSTATUS_OK) {
                      /* something wrong with lower layers, cleanup and go to open loop */
                      IB_LOG_INFO_FMT(__func__, "could not check local port info (status = %d)\n", status);
                      goto openIf3;
                   }
               }
           }  else {
               (void)vs_thread_sleep(PM_CONN_INTERVAL/2);
               if (pm_shutdown) goto bail;  // bail if operator dictates
               if (++cnt >= 6) {
                   IB_LOG_INFINI_INFORC("can't open connection for talking to FE, will try later. rc:", rc);
                   cnt = 0;
               }
               continue;
           }
       }
        
#if defined(__VXWORKS__) || defined(__LINUX__)
       /* initialize and open connection to PA Server for clients to access */
       if ((rc = pa_mngr_open_cnx()) != VSTATUS_OK)
           goto openIf3;
#endif

       /* force registration with SA now */
       (void)vs_time_get(&endtime);
       timeToCheckSa = endtime + (PM_CONN_INTERVAL);

	   if (goOn && ! PmEngineRunning()) {
           // start thread
           PmEngineStart();
       }

       while (!pm_shutdown && goOn)
       {

			// if SM is inactive then it was shut-down due to the config consistency problem - shutdown PM
			if (sm_isDeactivated()) {
				pm_shutdown = TRUE;
				IB_LOG_WARN_FMT( __func__, "Engine shutting down since SM is not active due to dbsync problems!");
				continue;
			}
           if (!sm_isActive() && !sm_isDeactivated()) {
               IB_LOG_INFO0("Waiting for SM to come out of initial NOTACTIVE state");
               (void)vs_thread_sleep(PM_CONN_INTERVAL/2);
               continue;
           }

          /* 
           * Register PM with SA 
           */
          (void)vs_time_get(&timenow);
          if (timeToCheckSa <= timenow) {
              saHasMoved = 0 ;
              if ((status = if3_check_sa(pm_fd, TRUE, &saHasMoved)) != VSTATUS_OK) {
                  /* something wrong with lower layers, cleanup and go to open loop */
                  IB_LOG_INFO_FMT (__func__, "could not check local port info (status = %d)\n", status);
                  goto openIf3;
              } else if (saHasMoved) {
                  /* new SM/SA, sweep now */
                  endtime = timenow;
              } 
              timeToCheckSa = timenow + PM_CONN_INTERVAL;
          }

          if (timenow >= endtime)
          {
              if ((rc = registerPM() != VSTATUS_OK)) {
                  IB_LOG_INFINI_INFORC("Unable to register with SA, will try later, rc:", rc);
                  isRegistered=0;
                  (void)vs_time_get(&timenow);
                  /* setup next time to try to register */
                  endtime = timenow + (5*VTIMER_1S);
              } else {
                  isRegistered=1;
                  (void)vs_time_get(&timenow);
                  /* setup time to renew registeration */
                  endtime = timenow + (pm_interval*1000000);
              }
          }

          rc = mai_recv(pm_fd, &mad, DEFAULT_INTERVAL_USEC);
          switch (rc) {
          
          case VSTATUS_TIMEOUT:
              break;
          
          case VSTATUS_MAI_INTERNAL:
          case VSTATUS_OK:
          {
		    uint16_t cmd;
#if defined(__VXWORKS__) || defined(__LINUX__)
            uint8_t processPaCmd;
            pa_cntxt_t *pa_cntxt;
#endif

		    // we have a MAD
            insize = (uint32_t) PM_MAX_DATA_LEN;
            if ((rc = if3_mngr_rcv_fe_data(pm_fd, &mad, gdata, &insize)) != VSTATUS_OK) continue;

            IB_LOG_INFO("Received cmd amod: ", mad.base.amod);
            IB_LOG_INFO("         cmd  aid: ", mad.base.aid);

#if defined(__VXWORKS__) || defined(__LINUX__)
            // check whether the request is a PA request
            if ((pa_cntxt = pa_mngr_get_cmd(&mad, &processPaCmd)) != NULL) {
                IB_LOG_INFO("Got PA command ", mad.base.method);
                if (processPaCmd)
                    pa_process_mad(&mad, pa_cntxt);
            }
#endif

            // check whether the request is a legacy FE request
            (void) if3_mngr_get_fe_cmd(&mad, &cmd);
            if (cmd == IF3_CONTROL_CMD) {
                switch (mad.base.method)
                {
                    case FM_CMD_SHUTDOWN:
                    {
                        IB_LOG_VERBOSE0("Got SHUTDOWN command");
                        pm_shutdown = TRUE;
                        (void)vfi_mngr_unregister(pm_fd, vfi_mclass, VFI_DEFAULT_GUID, &pmServRer,
                                                  VFI_SVRREC_GID | VFI_SVREC_NAME | VFI_SVRREC_ID);
                        isRegistered = 0;
                        goto bail;
                    }
                    break;
                    default:
                        IB_LOG_INFO("Got command ", mad.base.method);
                    break;
                } /* end switch on mad.base.method */
                continue;
            }
            if (!master)
            {
                /* Do not enter processing loop */
                continue;
            }

            cs_log(_PM_TRACE(pm_config.debug), "PM",
                   "Received cmd 0x%.8x  insize %d", cmd, insize);
            switch (cmd)
            {
			case STL_PA_ATTRID_GET_CLASSPORTINFO:
			case STL_PA_ATTRID_GET_GRP_CFG:
			case STL_PA_ATTRID_GET_GRP_INFO:
			case STL_PA_ATTRID_GET_GRP_LIST:
			case STL_PA_ATTRID_GET_PORT_CTRS:
			case STL_PA_ATTRID_CLR_PORT_CTRS:
			case STL_PA_ATTRID_CLR_ALL_PORT_CTRS:
			case STL_PA_ATTRID_GET_PM_CONFIG:
			case STL_PA_ATTRID_FREEZE_IMAGE:
			case STL_PA_ATTRID_RELEASE_IMAGE:
			case STL_PA_ATTRID_RENEW_IMAGE:
			case STL_PA_ATTRID_GET_FOCUS_PORTS:
			case STL_PA_ATTRID_GET_IMAGE_INFO:
			case STL_PA_ATTRID_MOVE_FREEZE_FRAME:
			case STL_PA_ATTRID_GET_VF_LIST:
			case STL_PA_ATTRID_GET_VF_INFO:
			case STL_PA_ATTRID_GET_VF_CONFIG:
			case STL_PA_ATTRID_GET_VF_PORT_CTRS:
			case STL_PA_ATTRID_CLR_VF_PORT_CTRS:
			case STL_PA_ATTRID_GET_VF_FOCUS_PORTS:
			case STL_PA_ATTRID_GET_FOCUS_PORTS_MULTISELECT:
			case STL_PA_ATTRID_GET_GRP_NODE_INFO:
			case STL_PA_ATTRID_GET_GRP_LINK_INFO:
			case STL_PA_ATTRID_GET_GRP_LIST2:
			case STL_PA_ATTRID_GET_VF_LIST2:
				break;

			default:
				IB_LOG_ERROR("Unknown command:", cmd);
			}  // end switch on CMD

		} // end case VSTATUS_OK
		break;
	    
        case VSTATUS_FILTER:
		{
		    IB_LOG_INFO("Filter take over notice received", rc);
		}
		break;
	    default:
		{
		    IB_LOG_ERRORRC("got bad status rc:", rc);
			if(pm_shutdown) goto bail;
		}
	    } // end switch on rc

      } // end while !pm_shutdown && goOn

      bail:

	   pm_shutdown = 1;	// just to be sure
	   PmEngineStop();

	  if (fh) (void) mai_filter_hdelete(hpma, fh);

#if defined(__VXWORKS__) || defined(__LINUX__)
      // PA related cleanup
      (void)pa_delete_filters();
      (void)pa_main_kill();
#endif

      if (hpma > 0) (void) mai_close(hpma);

      // deregister the pm if necessary
      if (isRegistered) {
          (void)vfi_mngr_unregister(pm_fd, vfi_mclass, VFI_DEFAULT_GUID, &pmServRer,
                                    VFI_SVRREC_GID | VFI_SVREC_NAME | VFI_SVRREC_ID);
      }

      if (pm_fd > 0) if3_mngr_close_cnx_fe(pm_fd, TRUE);

      pm_fd = -1;
      hpma = -1;
      fh = -1;
      pm_shutdown = 0;
#ifdef __LINUX__
#ifdef CAL_IBACCESS
      mai_umadt_read_kill();
      vs_thread_sleep(VTIMER_1S);
      cal_ibaccess_global_shutdown();
#endif
#endif
      vs_pool_delete(&pm_pool);	 

      IB_LOG_INFINI_INFO0("PM Task exiting OK.");
      return 0;
}

/*
 * send shutdown MAD to the local PM 
 */
Status_t
pm_main_kill(void){
    Status_t rc=0;
	PmEngineStop();
#ifdef __VXWORKS__
    IBhandle_t handle=0;

    if ((rc = if3_local_mngr_cnx(pm_config.hca, pm_config.port, MAD_CV_VFI_PM, &handle)) != VSTATUS_OK) {
        IB_LOG_VERBOSERC("failed to connect to local PM, rc:", rc);
    } else {
        if ((rc = if3_timeout_retry(handle, 333000ull, 3)) != VSTATUS_OK) {
            IB_LOG_VERBOSERC("failed to set timeout and retry count on handle, rc:", rc);
        } else {
            /* send Shutdown command */
            if ((rc = if3_cntrl_cmd_send(handle, FM_CMD_SHUTDOWN)) != VSTATUS_OK) {
                IB_LOG_VERBOSERC("failed to send SHUTDOWN command to local PM, rc:", rc);
            } else {
                IB_LOG_VERBOSE0("Sent SHUTDOWN command to PM thread");
            }
        }
        if3_close_mngr_cnx(handle);
    }
#endif    
    /* fail safe */
    pm_shutdown = TRUE;
    return rc;
}

void pmDebugOn(void) {
    pm_config.debug = 1;
}

void pmDebugOff(void) {
    pm_config.debug = 0;
}

uint32_t pmDebugGet(void)
{
	return pm_config.debug;
}
