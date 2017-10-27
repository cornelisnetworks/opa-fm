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
//    sm_main.c								     //
//									     //
// DESCRIPTION								     //
//    The main entry point to the OS independent part of SM.		     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sm_main				main entry point		     //
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
#include "sm_l.h"
#include "sm_qos.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "cs_csm_log.h"
#include "sm_jm.h"
#include "iba/public/imath.h"
#include "if3.h"

#ifndef __VXWORKS__
#include <opamgt_priv.h>
#include <code_version.h>
#endif

#ifdef CAL_IBACCESS
#include "cal_ibaccess_g.h"
#endif

#include "mal_g.h"

extern int sa_main(void);
extern void sa_main_kill(void);
extern void topology_main_kill(void);
extern void async_main_kill(void);
extern void topology_rcv_kill(void);
extern Status_t pm_main_kill(void);
extern void fe_main_kill(void);

#ifdef __VXWORKS__
#include "icsApi.h"
#include "icsBspUtil.h"
#else
extern int sm_conf_server_init(void);

extern Status_t pm_get_xml_config(void);
extern Status_t pm_initialize_config(void);
extern void pmApplyLogLevelChange(PMXmlConfig_t* new_pm_config);
extern void unified_sm_pm(uint32_t argc, uint8_t ** argv);

extern Status_t fe_initialize_config(FMXmlCompositeConfig_t *xml_config, uint32_t fe_instance);

extern void unified_sm_fe(uint32_t argc, uint8_t ** argv);

extern struct omgt_port *g_port_handle;

#ifdef CAL_IBACCESS
extern void mai_umadt_read_kill(void);
#endif


char hostName[64];
#endif

extern bool_t smCheckServiceId(int vf, uint64_t serviceId, VirtualFabrics_t *VirtualFabrics);

uint32_t    smFabricDiscoveryNeeded=0;
uint64_t	topology_sema_setTime=0;
uint64_t	topology_sema_runTime=0;
uint32_t   	sm_def_mc_group;
extern uint32_t            smDebugPerf;  // control SM performance messages; default is off
extern uint32_t            saDebugPerf;  // control SA performance messages; default is off
extern uint32_t            saDebugRmpp;  // control SA RMPP INFO debug messages; default is off
extern uint32_t            sm_debug;    // SM debug; default is off
extern uint32_t            saRmppCheckSum; // control checksum of SA RMPP responses; default is off;
extern uint8_t             smTerminateAfter; // Used for performance testing.
extern char*               smDumpCounters; // Used for performance testing.
extern uint8_t sa_dynamicPlt[];   // entry zero set to 1 indicates table in use

Pool_t		sm_pool;
Pool_t		sm_xml_pool;

Sema_t		sa_sema;
Lock_t		sa_lock;

LidMap_t	* lidmap = NULL;
cl_qmap_t	* sm_GuidToLidMap = NULL;

size_t	g_smPoolSize;

uint8_t		sm_env[32];
uint32_t    sa_max_cntxt;
uint32_t	sm_lid = 0;
uint32_t	sm_state = SM_STATE_NOTACTIVE;
uint32_t	sm_prevState = SM_STATE_NOTACTIVE;
int			sm_saw_another_sm = FALSE;	// did we ever see another SM in fabric
int			sm_hfi_direct_connect = FALSE;
uint64_t	sm_portguid;
uint8_t     sm_default_mkey_protect_level = 1;
uint8_t     sm_mkey_protect_level;
uint16_t    sm_mkey_lease_period;
uint64_t	sm_control_cmd;
uint64_t    sm_masterCheckInterval;
uint32_t    sm_trapThreshold = 0xffffffff;
// The number of traps required to validate a perceived trap rate.
// Or in other words, the sampling  window is sized so as to be large enough to collect
// this many traps before disabling a port.  Larger values increase the accuracy, but also increase
// the delay between a trap surge and a port disable.
uint32_t	sm_trapThreshold_minCount = 0xffffffff;
uint64_t    sm_trapThresholdWindow = 0;
uint32_t	sm_mcDosThreshold;
uint32_t	sm_mcDosAction;
uint64_t	sm_mcDosInterval;
uint32_t	sm_mcast_mlid_table_cap = 0;
uint16_t	sm_masterSmSl = 0;
uint16_t	sm_masterPmSl = 0;

bitset_t	sm_linkSLsInuse;
//			The % of bandwidth allocated to each SL. SLs can have multiple
//			SCs, so an SC's bandwidth is SL bandwidth / #  of SCs per SL.

uint32_t	sm_lid_lo = 1;
uint32_t	sm_lid_hi = UNICAST_LID_MAX;

uint32_t	sm_log_level = 1;
uint32_t	sm_log_level_override = 0;
uint32_t	sm_log_masks[VIEO_LAST_MOD_ID+1];
int			sm_log_to_console = 0;
char		sm_config_filename[256];

uint32_t    sm_nodaemon = 1;

uint32_t sm_lmc_0_freeLid_hint = 0;
uint32_t sm_lmc_e0_freeLid_hint = 0;
uint32_t sm_lmc_freeLid_hint = 0;


STL_SM_INFO	sm_smInfo;
uint32_t	sm_masterStartTime;
McGroup_t	*sm_McGroups=0;
uint32_t    sm_numMcGroups = 0;
uint32_t    sm_McGroups_Need_Prog = 0;
Lock_t      sm_McGroups_lock;

uint32_t	sm_useIdealMcSpanningTreeRoot = 1;
uint32_t	sm_mcSpanningTreeRoot_useLeastWorstCaseCost = 0;
uint32_t	sm_mcSpanningTreeRoot_useLeastTotalCost = 1;
/* Minimum cost improvement required to change the mc root switch */
#define		DEFAULT_MCROOT_COST_IMPROVEMENT_PERCENTAGE	50
uint32_t	sm_mcRootCostDeltaThreshold = DEFAULT_MCROOT_COST_IMPROVEMENT_PERCENTAGE;

boolean     sweepsPaused = 0;

int			sm_QosConfigChange = 0;

SmAdaptiveRouting_t sm_adaptiveRouting;

// XML configuration data structure
#ifdef __VXWORKS__
extern FMXmlCompositeConfig_t *xml_config;
static uint32_t    			xml_trace = 0;
extern SMXmlConfig_t 		sm_config;
extern FEXmlConfig_t 		fe_config;
extern PMXmlConfig_t 		pm_config;
extern SMDPLXmlConfig_t 	sm_dpl_config;
extern SMMcastConfig_t 		sm_mc_config;
extern SmMcastMlidShare_t 	sm_mls_config;
extern SMMcastDefGrpCfg_t	sm_mdg_config;
extern VFXmlConfig_t		vf_config;
extern DGXmlConfig_t		dg_config;
extern uint16_t 			numMcGroupClasses;
void sm_cleanGlobals(uint8_t);
#else
FMXmlCompositeConfig_t *xml_config = NULL;
SMXmlConfig_t 				sm_config;
FEXmlConfig_t 				fe_config;
PMXmlConfig_t 				pm_config;
SMDPLXmlConfig_t 			sm_dpl_config;
SMMcastConfig_t 			sm_mc_config;
SmMcastMlidShare_t 			sm_mls_config;
SMMcastDefGrpCfg_t 			sm_mdg_config;
VFXmlConfig_t				vf_config;
DGXmlConfig_t				dg_config;

uint32_t    				xml_trace = 0;

extern uint32_t pm_conf_start;
extern uint32_t bm_conf_start;

static int pm_running = 0;
#ifdef FE_THREAD_SUPPORT_ENABLED
static int fe_running = 0;
#endif
#endif

// Instance of this SM
uint32_t	sm_instance;

// SM Checksums
uint32_t	sm_overall_checksum;
uint32_t	sm_consistency_checksum;

// looptest control
uint32_t sm_looptest_disabled_ar = 0;

/*
 * Routing module instance used in the main thread. A copy of
 * this instance is used for the first sweep of the SM.
 */
static RoutingModule_t *sm_main_routingModule = NULL;

SmDorRouting_t smDorRouting;

// pointer to Virtual Fabric configuration info
VirtualFabrics_t *initialVfPtr  = NULL;
VirtualFabrics_t *updatedVirtualFabrics = NULL;

IBhandle_t	fd_sa;
IBhandle_t	fd_sa_writer;
IBhandle_t	fd_saTrap;
IBhandle_t	fd_async;
IBhandle_t	fd_async_request;
IBhandle_t  fd_sminfo;
IBhandle_t	fd_topology;
IBhandle_t	fd_atopology;
IBhandle_t	fd_loopTest;
IBhandle_t	fd_dbsync;

Sema_t		state_sema;
Sema_t		topology_sema;
Sema_t      topology_rcv_sema;          // topology receive thread ready semaphore
Lock_t		old_topology_lock;			// a RW Thread Lock
Lock_t		new_topology_lock;			// a Thread Lock
Lock_t		tid_lock;
Lock_t		handover_sent_lock;

#ifdef __LINUX__
Lock_t linux_shutdown_lock; //RW lock for shutdown. Taken by file I/O ops that shouldn't be killed mid operation.
#endif

/* flag to indicate if we have triggered a sweep for handover from async thread (sm_fsm.c) */
uint32_t triggered_handover=0;
/* flag to indicate if we have sent a handover from the topology thread (sm_topology.c) */
uint32_t handover_sent=0;


SMThread_t	*sm_threads;

static char msgbuf[256]={0};

/*
 * This function sets up a table of values and masks for
 * Multicast group GID's and the maximum number of groups
 * that may exist matching each value/mask pair before
 * the SM starts assigning groups matching the value/mask
 * pair the same multicast lid address.
 *
 * N.B. When we support pkeys, a new key must be added so
 * that we can support associated pkeys.
 */
void sm_init_mcast_mgid_mask_table(void)
{
	IB_GID mask;
	IB_GID value;
	uint16_t maximum;
	Status_t status = 0;
	int i = 0;

	/* Grab MCast Group stuff for each possible entry */
	for (i = 0; i < MAX_SUPPORTED_MCAST_GRP_CLASSES; ++i)
	{
		maximum = 0;

		/* grab the mask */
		if ((status = cs_parse_gid(sm_mls_config.mcastMlid[i].mcastGrpMGidLimitMaskConvert.value, mask.Raw)) != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Bad value for MLIDShare MGIDMask %d: %s", i,
				sm_mls_config.mcastMlid[i].mcastGrpMGidLimitMaskConvert.value);
			continue;
		}

		/* grab the value */
		if ((status = cs_parse_gid(sm_mls_config.mcastMlid[i].mcastGrpMGidLimitValueConvert.value, value.Raw)) != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Bad value for MLIDShare MGIDValue %d: %s", i,
				sm_mls_config.mcastMlid[i].mcastGrpMGidLimitValueConvert.value);
			continue;
		}

		// Gid_t helpers read in network byte order
		BSWAP_IB_GID(&mask);
		BSWAP_IB_GID(&value);

		/* grab the limit */
		maximum = 0xFFFF & sm_mls_config.mcastMlid[i].mcastGrpMGidLimitMax;
		if (maximum != 0)
		{
			if ((status = sm_multicast_add_group_class(mask, value, maximum)) != VSTATUS_OK)
				IB_LOG_ERROR_FMT(__func__, "Couldn't add mcast group class # %d to table", i);
		}
	}

	maximum = 0;
	if (sm_mc_config.mcast_mlid_table_cap > MAX_SIZE_MFT)
	{
		IB_LOG_ERROR_FMT(__func__, "Bad value for MLIDTableCap - %d... "
		       "Max allowed is %d, Falling back to default value of %d", sm_mc_config.mcast_mlid_table_cap, MAX_SIZE_MFT, DEFAULT_SW_MLID_TABLE_CAP);
		maximum = DEFAULT_SW_MLID_TABLE_CAP;
	} else
	{
		maximum = (uint16_t)(0xFFFF & sm_mc_config.mcast_mlid_table_cap);
	}
	sm_mcast_mlid_table_cap = maximum;

	IB_LOG_VERBOSE_FMT(__func__,
	       "Calling sm_multicast_set_default_group_class(0, 0, %d)",
	       maximum);

	if ((status = sm_multicast_set_default_group_class(maximum)) != VSTATUS_OK)
	{
		IB_LOG_ERROR_FMT(__func__, "Couldn't add default mcast group class, error = %d", status);
	}
}

/*
 * Read in the dynamic packet lifetime values from the config file
 */
void sm_init_plt_table(void){
	int i, rc=0;
    uint8_t  dplt[DYNAMIC_PACKET_LIFETIME_ARRAY_SIZE];
	char     pltStr[32];

	memset(pltStr,0,sizeof(pltStr));
    sprintf(pltStr,"dynamicPlt");

	if (sm_dpl_config.dp_lifetime[0] > 1) {
		IB_LOG_WARN0("SM: Invalid input specified for DynamicPacketLifetime Enable Must be between 0 or 1. Enabling.");
	}

    dplt[0] = (uint8_t)(sm_dpl_config.dp_lifetime[0]?1:0);
    if (dplt[0]) {
        /* dynamic packet lifetine is on, get the nine values */
        for(i = 1; i < DYNAMIC_PACKET_LIFETIME_ARRAY_SIZE; i++){
            sprintf(pltStr,"dynamicPlt_%.2d",i);
			/* Nota Bene: We don't check for dp_lifetime<DYNAMIC_PLT_MIN 
 			 * because DYNAMIC_PLT_MIN is zero, and dp_lifetime is unsigned. if
 			 * this is changed we will need to check. 
 			 */
            if (sm_dpl_config.dp_lifetime[i] > DYNAMIC_PLT_MAX){
                IB_LOG_WARN_FMT(__func__,
            		"SM: %s %u %s %s %s", "Invalid entry of", (unsigned)sm_dpl_config.dp_lifetime[i], "for", pltStr, "- Must be between 1-31.  Using defaults.");
                rc = 1;
                break;
            } else if (sm_dpl_config.dp_lifetime[i] > 0) {
                dplt[i] = (uint8_t)sm_dpl_config.dp_lifetime[i];
            } else if (i > 1) {
                /* use the previous entry for this index */
                dplt[i] = dplt[i-1];
            } else {
				IB_LOG_WARN0("SM: DynamicPacketLifetime Hops01 unspecified.  Using defaults.");
                /* just break out and use the hard coded defaults */
                rc = 1;
				break;
            }
        }
        if (rc == 0) {
            /* valid set of values entered for dynamic packet lifetime */
            memcpy(sa_dynamicPlt, dplt, sizeof(dplt));
        }
    } else {
        /* turning off dynamic packet lifetime */
        sa_dynamicPlt[0] = 0;
    }
    /* Output what we are using */        
    if (sa_dynamicPlt[0]) {
        sprintf(msgbuf, "SM: Using dynamic packet lifetime values %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d",
                sa_dynamicPlt[1], sa_dynamicPlt[2], sa_dynamicPlt[3], sa_dynamicPlt[4], sa_dynamicPlt[5], 
                sa_dynamicPlt[6], sa_dynamicPlt[7], sa_dynamicPlt[8], sa_dynamicPlt[9]);
        vs_log_output_message(msgbuf, FALSE);
    } else {
        sprintf(msgbuf, "SM: Dynamic packet lifetime is OFF, using saPacketLifetime contant %d", (unsigned int)sm_config.sa_packet_lifetime_n2);
        vs_log_output_message(msgbuf, FALSE);
    }
	return;
}

void sm_init_log_setting(void){
#ifndef __VXWORKS__
	vs_log_control(VS_LOG_SETFACILITY, (void *)(unint)getFacility(sm_config.syslog_facility, /* test */ 0), (void *)0, (void *)0);
#endif
	vs_log_control(VS_LOG_SETMASK, sm_log_masks,(void*)(uintn)sm_log_level, (void *)(unint)sm_log_to_console);
#ifndef __VXWORKS__
	if(strlen(sm_config.name) > 0)
		vs_log_control(VS_LOG_SETSYSLOGNAME, (void *)sm_config.name, (void *)0, (void *)0);
	else
		vs_log_control(VS_LOG_SETSYSLOGNAME, (void *)"fm_sm", (void *)0, (void *)0);
	if(strlen(sm_config.log_file) > 0)
	{
		vs_log_control(VS_LOG_SETOUTPUTFILE, (void *)sm_config.log_file, (void *)0, (void *)0);

		if(sm_log_level > 0)
			omgt_set_err(g_port_handle, vs_log_get_logfile_fd());
		else
			omgt_set_err(g_port_handle, NULL);

		if(sm_log_level > 2)
			omgt_set_dbg(g_port_handle, vs_log_get_logfile_fd());
		else
			omgt_set_dbg(g_port_handle, NULL);

	}
	else
	{
		vs_log_control(VS_LOG_SETOUTPUTFILE, (void *)0, (void *)0, (void *)0);

		if(sm_log_level > 0)
			omgt_set_err(g_port_handle, OMGT_DBG_FILE_SYSLOG);
		else
			omgt_set_err(g_port_handle, NULL);

		if(sm_log_level > 2)
			omgt_set_dbg(g_port_handle, OMGT_DBG_FILE_SYSLOG);
		else
			omgt_set_dbg(g_port_handle, NULL);

	}

#endif

	vs_log_set_log_mode(sm_config.syslog_mode);

#ifndef __VXWORKS__
	vs_log_control(VS_LOG_STARTSYSLOG, (void *)0, (void *)0, (void *)0);
#endif
}

void sm_set_log_level(uint32_t log_level)
{
	sm_log_level = log_level;
	snprintf(msgbuf, sizeof(msgbuf), "Setting SM LogLevel to %u", (unsigned)sm_log_level);
	vs_log_output_message(msgbuf, FALSE);
	cs_log_set_log_masks(sm_log_level, sm_config.syslog_mode, sm_log_masks);
	sm_init_log_setting();
}

uint32_t sm_get_log_level(void)
{
	return sm_log_level;
}

void sm_set_log_mode(uint32_t log_mode)
{
	sm_config.syslog_mode = log_mode;
	snprintf(msgbuf, sizeof(msgbuf), "Setting SM LogMode to %u", (unsigned)sm_config.syslog_mode);
	vs_log_output_message(msgbuf, FALSE);
	cs_log_set_log_masks(sm_log_level, sm_config.syslog_mode, sm_log_masks);
	sm_init_log_setting();
}

uint32_t sm_get_log_mode(void)
{
	return sm_config.syslog_mode;
}

void sm_set_log_mask(const char* mod, uint32_t mask)
{
	if (! cs_log_get_module_id(mod)) {
		snprintf(msgbuf, sizeof(msgbuf), "Requested setting SM LogMask for invalid subsystem: %s", mod);
		vs_log_output_message(msgbuf, FALSE);
	} else {
		snprintf(msgbuf, sizeof(msgbuf), "Setting SM %s_LogMask to 0x%x", mod, (unsigned)mask);
		vs_log_output_message(msgbuf, FALSE);
		cs_log_set_log_mask(mod, mask, sm_log_masks);
		sm_init_log_setting();
	}
}

int sm_valid_module(const char * mod)
{
	return 0 != cs_log_get_module_id(mod);
}

uint32_t sm_get_log_mask(const char* mod)
{
	return cs_log_get_log_mask(mod, sm_log_masks);
}

void sm_set_force_attribute_rewrite(uint32_t force_attr_rewrite){
	sm_config.forceAttributeRewrite = force_attr_rewrite;
	IB_LOG_INFINI_INFO_FMT(__func__, "Setting force attribute rewrite to %u", force_attr_rewrite);
	if (force_attr_rewrite) forceRebalanceNextSweep = 1;
}

void sm_set_skip_attribute_write(char * datap){
    uint32_t tmp;
    memcpy(&tmp, datap, sizeof(tmp));
	sm_config.skipAttributeWrite = tmp;
	IB_LOG_INFINI_INFO_FMT(__func__, "Setting skip attribute write to 0x%x", tmp);
}

void sm_free_vf_mem(void) {
	bitset_free(&sm_linkSLsInuse);
}

void sm_process_dor_info(VirtualFabrics_t *VirtualFabrics, SmDorRouting_t *dorCfg) {

	int d,p;
	smDorRouting = *dorCfg;
	int toroidal_count = 0;

	if (smDorRouting.dimensionCount == 0) {
		IB_LOG_ERROR_FMT(__func__,
						 "Routing algorithm configured as dor but no dimensions have been specified in the configuration.");
		IB_FATAL_ERROR_NODUMP("Please specify required dimensions in the configuration. Exiting");
	}

	if (smDorRouting.dimensionCount > SM_DOR_MAX_DIMENSIONS) {
		IB_LOG_ERROR_FMT(__func__, "Number of dimensions configured is %d but dor algorithm supports only %d dimensions.",
						smDorRouting.dimensionCount, SM_DOR_MAX_DIMENSIONS);
		IB_FATAL_ERROR_NODUMP("Please reconfigure with correct number of dimensions. Exiting");
	}

	if (smDorRouting.warn_threshold > SM_DOR_MAX_WARN_THRESHOLD) {
		IB_LOG_WARN_FMT(__func__,
			 "MeshTorusTopology WarnThreshold of %d is higher than max suported %d. Defaulting to %d.",
			 smDorRouting.warn_threshold, SM_DOR_MAX_WARN_THRESHOLD, SM_DOR_MAX_WARN_THRESHOLD);
		smDorRouting.warn_threshold = SM_DOR_MAX_WARN_THRESHOLD;
	}
	if (smDorRouting.debug) {
		IB_LOG_INFINI_INFO_FMT(__func__, "DorTopology %s NumberOfDimensions %d routingSCs %d",
				(smDorRouting.topology == DOR_MESH)?"mesh":((smDorRouting.topology == DOR_TORUS)? "torus":"partial torus"), 
				smDorRouting.dimensionCount, smDorRouting.routingSCs);

		for (d=0; d<smDorRouting.dimensionCount; d++) {
			if (smDorRouting.dimension[d].toroidal)
				toroidal_count++;

   	   		IB_LOG_INFINI_INFO_FMT(__func__, "DorDimension %d is %s", 
				d, smDorRouting.dimension[d].toroidal?"toroidal":"not toroidal");

			for (p=0; p<smDorRouting.dimension[d].portCount; p++) {
				IB_LOG_INFINI_INFO_FMT(__func__, "DorDimension %d PortPair %d,%d", d,
					smDorRouting.dimension[d].portPair[p].port1, smDorRouting.dimension[d].portPair[p].port2);
			}
		}
		IB_LOG_INFINI_INFO_FMT(__func__, "Total toroidal dimensions %d", toroidal_count);
	}
}

static Status_t
sm_resolve_pkeys_for_vfs(VirtualFabrics_t *VirtualFabrics)
{
	int				vf, vf2;
	VFDg_t*			mcastGrpp;
	VFAppMgid_t*	mgidp;

	if (!VirtualFabrics || (VirtualFabrics->number_of_vfs == 0))  {
		return VSTATUS_OK;
	}

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		VF_t *vfp = &VirtualFabrics->v_fabric_all[vf];

		if (vfp->security)
			VirtualFabrics->securityEnabled = 1;

		if (vfp->qos_enable)
			VirtualFabrics->qosEnabled = 1;

	} //end loop on all VFs
	// Prior code dealing with PKEYs
	for (vf=0; vf<VirtualFabrics->number_of_vfs && vf<MAX_VFABRICS; vf++) {

		// Check if pkey is defined.
		if ((VirtualFabrics->v_fabric[vf].pkey == UNDEFINED_PKEY) ||
			(PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) == INVALID_PKEY)) {
				IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name,
							__func__,
							"Found VF without assigned pkey. All VFs should have already assigned pkeys either manually or automatically");
						return VSTATUS_BAD;

		} else if (!VirtualFabrics->v_fabric[vf].security) {
			for (vf2=0; vf2<VirtualFabrics->number_of_vfs; vf2++) {
				if (vf==vf2) continue;
				if ((VirtualFabrics->v_fabric[vf].pkey == VirtualFabrics->v_fabric[vf2].pkey) && VirtualFabrics->v_fabric[vf2].security) {
					IB_LOG_INFINI_INFO_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
						"VFabric has security disabled. VFabric %s has same PKey with security enabled. Enabling security.",
 						VirtualFabrics->v_fabric[vf2].name);
					VirtualFabrics->v_fabric[vf].security = 1;
					break;
				}
			}
		}

		if (VirtualFabrics->v_fabric[vf].security) {
			VirtualFabrics->v_fabric[vf].pkey = PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey);
		} else {
			VirtualFabrics->v_fabric[vf].pkey = VirtualFabrics->v_fabric[vf].pkey | FULL_MEMBER;
		}
	
		if (VirtualFabrics->v_fabric[vf].apps.select_sa) {
			if (PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) != DEFAULT_PKEY) {
				IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
					"VFabric has application SA selected, bad PKey configured 0x%x, must use Mgmt PKey.",
					VirtualFabrics->v_fabric[vf].pkey);
			} else {
				sm_masterSmSl = VirtualFabrics->v_fabric[vf].base_sl;
			}
		}
		if (smCheckServiceId(vf, STL_PM_SERVICE_ID, VirtualFabrics)) {
			if (PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) != DEFAULT_PKEY) {
				IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
					"VFabric has application PA selected, bad PKey configured 0x%x, must use Mgmt PKey.",
					VirtualFabrics->v_fabric[vf].pkey);
			}
		}
		if (VirtualFabrics->v_fabric[vf].apps.select_pm) {
			if (PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) != DEFAULT_PKEY) {
				IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
					"VFabric has application PM selected, bad PKey configured 0x%x, must use Mgmt PKey.",
					VirtualFabrics->v_fabric[vf].pkey);
			} else sm_masterPmSl = VirtualFabrics->v_fabric[vf].base_sl;
		}
		for (mcastGrpp = VirtualFabrics->v_fabric[vf].default_group; mcastGrpp;
			mcastGrpp = mcastGrpp->next_default_group) {

			if (mcastGrpp->def_mc_create) {
				if (mcastGrpp->def_mc_pkey == UNDEFINED_PKEY) {
					mcastGrpp->def_mc_pkey = VirtualFabrics->v_fabric[vf].pkey;
				}

				if (PKEY_VALUE(mcastGrpp->def_mc_pkey) != PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey)) {
					IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
						"MulticastGroup configuration error, mismatch on pkey. Disabling Default Group");
					mcastGrpp->def_mc_create = 0;
					continue; // Not going to create Default Group, skip.
				}

				if (mcastGrpp->def_mc_rate_int >= UNDEFINED_XML8) {
					mcastGrpp->def_mc_rate_int = linkrate_gt(IB_STATIC_RATE_25G, VirtualFabrics->v_fabric[vf].max_rate_int) ?
						VirtualFabrics->v_fabric[vf].max_rate_int : IB_STATIC_RATE_25G;

				} else if (linkrate_gt(mcastGrpp->def_mc_rate_int, VirtualFabrics->v_fabric[vf].max_rate_int)) {
					mcastGrpp->def_mc_rate_int = linkrate_gt(IB_STATIC_RATE_25G, VirtualFabrics->v_fabric[vf].max_rate_int) ?
						VirtualFabrics->v_fabric[vf].max_rate_int : IB_STATIC_RATE_25G;

					IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
						"MulticastGroup configuration error, rate (%s) exceeds vFabric (%s), disabling Default Group",
						IbStaticRateToText(mcastGrpp->def_mc_rate_int),
						IbStaticRateToText(VirtualFabrics->v_fabric[vf].max_rate_int));
					mcastGrpp->def_mc_create = 0;
					continue; // Not going to create Default Group, skip.
				}

				if (mcastGrpp->def_mc_mtu_int >= UNDEFINED_XML8) {
					mcastGrpp->def_mc_mtu_int = MIN(IB_MTU_2048, VirtualFabrics->v_fabric[vf].max_mtu_int);

				} else if (mcastGrpp->def_mc_mtu_int > VirtualFabrics->v_fabric[vf].max_mtu_int) {
					IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
						"MulticastGroup configuration error, MTU (%s) exceeds vFabric (%s), disabling Default Group",
						IbMTUToText(mcastGrpp->def_mc_mtu_int), IbMTUToText(VirtualFabrics->v_fabric[vf].max_mtu_int));
					mcastGrpp->def_mc_create = 0;
					continue; // Not going to create Default Group, skip.
				}

				if (mcastGrpp->def_mc_sl == UNDEFINED_XML8) {
					mcastGrpp->def_mc_sl = VirtualFabrics->v_fabric[vf].mcast_sl;

				} else if (mcastGrpp->def_mc_sl != VirtualFabrics->v_fabric[vf].mcast_sl) {
					IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
						"MulticastGroup configuration error, SL must match SL %d (configured SL %d), disabling Default Group",
						VirtualFabrics->v_fabric[vf].mcast_sl, mcastGrpp->def_mc_sl);
					mcastGrpp->def_mc_create = 0;
					continue; // Not going to create Default Group, skip.
				}

				cl_map_item_t *cl_map_item;
				for_all_qmap_ptr(&mcastGrpp->mgidMap, cl_map_item, mgidp) {
					// Verify mgid has pkey inserted.
					smVerifyMcastPkey(mgidp->mgid, mcastGrpp->def_mc_pkey);
					if (smVFValidateMcDefaultGroup(vf, mgidp->mgid) != VSTATUS_OK) {
						IB_LOG_ERROR_FMT_VF(VirtualFabrics->v_fabric[vf].name, __func__,
							"MulticastGroup configuration error, MGID "FMT_GID" does not match app, disabling Default Group",
							mgidp->mgid[0], mgidp->mgid[1]);
						mcastGrpp->def_mc_create = 0;
						break; // for_all_qmap_ptr()
					}
				}
			}
		}
	}
	return VSTATUS_OK;
}

static Status_t
sm_assign_qos_params(VirtualFabrics_t *VirtualFabrics)
{
	DEBUG_ASSERT(sm_main_routingModule != NULL);

	/* Assign SLs */
	if (sm_main_routingModule->funcs.assign_sls)
		if (sm_main_routingModule->funcs.assign_sls(sm_main_routingModule, VirtualFabrics) != VSTATUS_OK)
			return VSTATUS_BAD;

	/* Update BW */
	if (sm_main_routingModule->funcs.update_bw)
		if (sm_main_routingModule->funcs.update_bw(sm_main_routingModule, VirtualFabrics) != VSTATUS_OK)
			return VSTATUS_BAD;

	/* Assign SCs to SLs */
	if (sm_main_routingModule->funcs.assign_scs_to_sls) {
		if (sm_main_routingModule->funcs.assign_scs_to_sls(sm_main_routingModule, VirtualFabrics) != VSTATUS_OK)
			return VSTATUS_BAD;
		sm_printf_vf_debug(VirtualFabrics);
	}
	return VSTATUS_OK;
}

static Status_t
sm_process_vf_info(VirtualFabrics_t *VirtualFabrics)
{
	if (!bitset_init(&sm_pool, &sm_linkSLsInuse, STL_MAX_SLS)) {
		IB_LOG_ERROR_FMT(__func__, "Out of memory.");
		return VSTATUS_NOMEM;
	}

    // In IB each default PKey entry was setup to be a full member.  In STL
    // the default PKey entry is a limited member.
    //
    //setPKey(0, DEFAULT_PKEY | FULL_MEMBER, 0);
    setPKey(0, STL_DEFAULT_APP_PKEY, 0);
    setPKey(1, STL_DEFAULT_PKEY, 0);

    if (!VirtualFabrics || (VirtualFabrics->number_of_vfs == 0))  {
        sm_masterSmSl = 0;
        return VSTATUS_OK;
    }

    if (sm_assign_qos_params(VirtualFabrics) != VSTATUS_OK)
        return VSTATUS_BAD;

    return sm_resolve_pkeys_for_vfs(VirtualFabrics);
}

static void print_error(const char *msg)
{
			IB_LOG_ERROR_FMT("sm_parse_xml_config", "%s", msg);
}

static void print_warning(const char *msg)
{
			IB_LOG_WARN_FMT("sm_parse_xml_config", "%s", msg);
}

// Parse the XML configuration
Status_t sm_parse_xml_config(void) {

	uint32_t 	modid;
	uint32_t	adaptiveRoutingDisable = 0;

#ifdef __VXWORKS__
	// if ESM then clean up the globals so we always read data correctly
	sm_cleanGlobals( /* stop */ 0);
#endif

	// The instance for now is the last character in the string sm_0 - sm_3 in the
	// sm_env variable. For now get it out of there so we have an integer instance.
	sm_instance = atoi((char*)&sm_env[3]);
	if (sm_instance >= MAX_INSTANCES) sm_instance = MAX_INSTANCES-1;

#ifndef __VXWORKS__
	// for now it's a fatal error if we can not parse correctly
	xml_config = parseFmConfig(sm_config_filename, IXML_PARSER_FLAG_NONE, sm_instance, /* full parse */ 0, /* embedded */ 0);
	if (!xml_config || !xml_config->fm_instance[sm_instance]) {
		IB_FATAL_ERROR_NODUMP("SM: Error encountered reading configuration file");
		return(VSTATUS_BAD);
	}
#endif // __VXWORKS__

	if (xml_config->xmlDebug.xml_sm_debug) {
		printf("###########sm_env %s sm_instance %u\n", sm_env, (unsigned int)sm_instance);
		xml_trace = 1;
	} else {
		xml_trace = 0;
	}

	// copy the configurations to local structures and adjust accordingly
#ifndef __VXWORKS__
	smCopyConfig(&sm_config,&xml_config->fm_instance[sm_instance]->sm_config);
	pm_config = xml_config->fm_instance[sm_instance]->pm_config;
	fe_config = xml_config->fm_instance[sm_instance]->fe_config;
	sm_dpl_config = xml_config->fm_instance[sm_instance]->sm_dpl_config;
	sm_mc_config = xml_config->fm_instance[sm_instance]->sm_mc_config;
	sm_mls_config = xml_config->fm_instance[sm_instance]->sm_mls_config;
	sm_mdg_config = xml_config->fm_instance[sm_instance]->sm_mdg_config;
#endif
	// use globals for debug since the extent into the MAI subsystem
	sm_debug = sm_config.debug;
	smDebugPerf = sm_config.sm_debug_perf;
	saDebugPerf = sm_config.sa_debug_perf;
	saDebugRmpp = sm_config.debug_rmpp;
	saRmppCheckSum = sm_config.sa_rmpp_checksum;
	smTerminateAfter = sm_config.terminateAfter;
	if (sm_config.dumpCounters[0] != 0) 
		smDumpCounters = sm_config.dumpCounters;

	/* The start parameter from the XML file is ignored for ESM */
#ifndef __VXWORKS__
	/* If start is disabled, the executable should exit in HSM */
	if (!sm_config.start) {
		IB_LOG_ERROR_FMT(__func__, "SM instance %u is not Enabled",(unsigned int)sm_instance );
		return(VSTATUS_ILLPARM);
        }
#endif

	// if LMC is zero, we will use this offset in the lid allocation logic 
	// value of 1 is the default behavior for LMC=0
	// setting this to 16 would allocate lids in multiples of 16, spreading 
	// the lid range sneaky way to make a small fabric have large lid values

	/* loopTest parameters */
	/* skip the looptest params if looptest was started manually on console */
	if (!esmLoopTestOn) {
		esmLoopTestOn = sm_config.loop_test_on;
		esmLoopTestNumPkts = sm_config.loop_test_packets;
		esmLoopTestFast = sm_config.loop_test_fast_mode;
 		if (esmLoopTestFast == 1) {
 			esmLoopTestInjectEachSweep = 0;
 			esmLoopTestPathLen = 4;
 		}
	}

	if (sm_log_level_override) {
		// command line override
		cs_log_set_log_masks(sm_log_level, sm_config.syslog_mode, sm_log_masks);
	} else {
		sm_log_level = sm_config.log_level;
		for (modid = 0; modid <= VIEO_LAST_MOD_ID; ++modid)
			sm_log_masks[modid] = sm_config.log_masks[modid].value;
	}
	sm_init_log_setting();
	vs_log_output_message("Subnet Manager starting up.", TRUE);
#ifndef __VXWORKS__
    vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID, NULL, NULL,
                    "SM: Version: %s", GetCodeVersion());
    //vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID, NULL, NULL,
    //                "SM: IntVersion: %s", GetCodeInternalVersion());
    //vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID, NULL, NULL,
    //                "SM: Brand: %s", GetCodeBrand());
#endif

    // Print out XML SM debug settings that could really break SM
    if ((sm_config.forceAttributeRewrite!=0) || (sm_config.skipAttributeWrite!=0)) {
        vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID, NULL, NULL,
                    "SM: (Debug Settings) ForceWrite %d, SkipWriteAttribute Bitmask: 0x%08x",
                     sm_config.forceAttributeRewrite, sm_config.skipAttributeWrite);
    }

#ifndef __VXWORKS__
	vs_init_coredump_settings("SM", sm_config.CoreDumpLimit, sm_config.CoreDumpDir);
#endif // __VXWORKS__

#ifndef __VXWORKS__
#endif

#ifndef __VXWORKS__ // not required for ESM
	// for debugging XML do not deamonize
	if (xml_trace)
		sm_nodaemon = 1;
#endif // __VXWORKS__

	// set global DG value
	sm_def_mc_group = sm_mdg_config.group[0].def_mc_create;

	// get the configuration for Virtual Fabrics
	initialVfPtr = renderVirtualFabricsConfig(sm_instance, xml_config, &sm_config, NULL);
	if (!initialVfPtr || !applyVirtualFabricRules(initialVfPtr, print_error, print_warning)) {
		IB_FATAL_ERROR_NODUMP("SM: Virtual Fabric configuration is invalid.");
		return(VSTATUS_BAD);
	}

	/* mcast table config */
	sm_init_mcast_mgid_mask_table();

	/* multicast spanning tree root selection parameters*/
	if (strncasecmp(sm_mc_config.mcroot_select_algorithm, "SMNeighbor", 32) == 0) {
		sm_useIdealMcSpanningTreeRoot = 0;
		sm_mcSpanningTreeRoot_useLeastTotalCost = 0;
		sm_mcSpanningTreeRoot_useLeastWorstCaseCost = 0;
	} else if (strncasecmp(sm_mc_config.mcroot_select_algorithm, "LeastTotalCost", 32) == 0) {
		sm_useIdealMcSpanningTreeRoot = 1;
		sm_mcSpanningTreeRoot_useLeastTotalCost = 1;
		sm_mcSpanningTreeRoot_useLeastWorstCaseCost = 0;
	}  else if (strncasecmp(sm_mc_config.mcroot_select_algorithm, "LeastWorstCaseCost", 32) == 0) {
		sm_useIdealMcSpanningTreeRoot = 1;
		sm_mcSpanningTreeRoot_useLeastWorstCaseCost = 1;
		sm_mcSpanningTreeRoot_useLeastTotalCost = 0;
	} else {
		if (sm_mc_config.mcroot_select_algorithm[0] != 0)
			IB_LOG_WARN_FMT(__func__,
			       "Invalid Multicast Root Selection algorithm ('%s'); defaulting to 'LeastTotalCost'",
			       sm_mc_config.mcroot_select_algorithm);
		sm_useIdealMcSpanningTreeRoot = 1;
		sm_mcSpanningTreeRoot_useLeastTotalCost = 1;
		sm_mcSpanningTreeRoot_useLeastWorstCaseCost = 0;
	}

	/* multicast spanning tree root update parameters */
	if (strlen(sm_mc_config.mcroot_min_cost_improvement) > 0) {
		char * percent;
		percent = strstr(sm_mc_config.mcroot_min_cost_improvement, "%");
		if (!percent) {
			IB_LOG_WARN_FMT(__func__,
			       "Multicast Root MinCostImprovement Parameter must be in percentage form - example 50%%."
					" Defaulting to %d%%", DEFAULT_MCROOT_COST_IMPROVEMENT_PERCENTAGE);
			sm_mcRootCostDeltaThreshold = DEFAULT_MCROOT_COST_IMPROVEMENT_PERCENTAGE;
		} else {
			*percent = 0;
			sm_mcRootCostDeltaThreshold = atoi(sm_mc_config.mcroot_min_cost_improvement);
		}
	} else {
		sm_mcRootCostDeltaThreshold = DEFAULT_MCROOT_COST_IMPROVEMENT_PERCENTAGE;
	}

	/* get the master ping interval params used by stanby SMs */
	sm_masterCheckInterval = (uint64_t)sm_config.master_ping_interval * VTIMER_1S;  /* put in in microsecs */
	if (sm_masterCheckInterval == 0)
		sm_masterCheckInterval = SM_CHECK_MASTER_INTERVAL * VTIMER_1S;

	/* check the trap threshold for auto-disabling ports throwing errors (0=off) */
	if (sm_config.trap_threshold != 0 &&
		(sm_config.trap_threshold < SM_TRAP_THRESHOLD_MIN || sm_config.trap_threshold > SM_TRAP_THRESHOLD_MAX)) {
		IB_LOG_WARN_FMT(__func__, "trap threshold of %d is out of range %d-%d, defaulting to %d",
			sm_config.trap_threshold, SM_TRAP_THRESHOLD_MIN, SM_TRAP_THRESHOLD_MAX, SM_TRAP_THRESHOLD_DEFAULT);
		sm_config.trap_threshold = SM_TRAP_THRESHOLD_DEFAULT;
	}

	if (sm_config.trap_threshold_min_count != 0 &&
		(sm_config.trap_threshold_min_count < SM_TRAP_THRESHOLD_COUNT_MIN)) {
		IB_LOG_WARN_FMT(__func__, "MinTrapThresholdCount of %d is invalid. Must be greater than or equal to %d. Defaulting to %d.",
			sm_config.trap_threshold_min_count, SM_TRAP_THRESHOLD_COUNT_MIN, SM_TRAP_THRESHOLD_COUNT_DEFAULT);
		sm_config.trap_threshold_min_count = SM_TRAP_THRESHOLD_COUNT_DEFAULT;
	}

	sm_setTrapThreshold(sm_config.trap_threshold, sm_config.trap_threshold_min_count);

	sm_mcDosThreshold = sm_config.mc_dos_threshold;
	if (sm_mcDosThreshold > SM_TRAP_THRESHOLD_MAX) {
		IB_LOG_WARN_FMT(__func__, "McDosThreshold of %d is invalid. Exceeds max of %d. Defaulting to max.",
						sm_mcDosThreshold, SM_TRAP_THRESHOLD_MAX);
		sm_mcDosThreshold = SM_TRAP_THRESHOLD_MAX;
	}
	sm_mcDosAction = sm_config.mc_dos_action;
	sm_mcDosInterval = sm_config.mc_dos_interval * VTIMER_1S;

#ifdef IB_STACK_OPENIB
	if (sm_config.debug_jm) {
		IB_LOG_INFINI_INFO0("SM: Enabling Job Management debug");
		ib_instrumentJmMads = 1;
	}
#endif

	// do we need to disable adaptive routing if loop test is on
	if (esmLoopTestOn && !esmLoopTestAllowAdaptiveRouting)
		adaptiveRoutingDisable = 1;
	else
		adaptiveRoutingDisable = 0;
	
	if (strncmp(sm_config.routing_algorithm, "dor", 32) == 0) {
		sm_process_dor_info(initialVfPtr, &sm_config.smDorRouting);
	}

	// Is Adaptive Routing Enabled
	if (sm_config.adaptiveRouting.enable && !adaptiveRoutingDisable) {
		sm_adaptiveRouting.enable = sm_config.adaptiveRouting.enable;
		sm_adaptiveRouting.debug = sm_config.adaptiveRouting.debug;
		sm_adaptiveRouting.algorithm = sm_config.adaptiveRouting.algorithm;
		sm_adaptiveRouting.lostRouteOnly = sm_config.adaptiveRouting.lostRouteOnly;
		sm_adaptiveRouting.arFrequency = sm_config.adaptiveRouting.arFrequency;
		sm_adaptiveRouting.threshold = sm_config.adaptiveRouting.threshold;

	} else {
		if (sm_config.adaptiveRouting.enable) {
			// Can be enabled when loop test stops
			sm_looptest_disabled_ar = 1;
			IB_LOG_WARN0("SM: Adaptive Routing disabled during loop test");
		}

		memset(&sm_adaptiveRouting, 0, sizeof(SmAdaptiveRouting_t));
	}

	if (xml_trace) {
		smShowConfig(&sm_config, &sm_dpl_config, &sm_mc_config, &sm_mls_config);
	}

	if (xml_trace) {
		printf("XML - SM old overall_checksum %llu new overall_checksum %llu\n",
			(long long unsigned int)sm_overall_checksum, (long long unsigned int)sm_config.overall_checksum);
		printf("XML - SM old consistency_checksum %llu new consistency_checksum %llu\n",
			(long long unsigned int)sm_consistency_checksum, (long long unsigned int)sm_config.consistency_checksum);
	}
	sm_overall_checksum = sm_config.overall_checksum;
	sm_consistency_checksum = sm_config.consistency_checksum;

	//loop on all DGs and convert all node descriptions
	int dgIdx;
	DGConfig_t *dgp;
	int numGroups = xml_config->fm_instance[sm_instance]->dg_config.number_of_dgs;
	for (dgIdx=0; dgIdx<numGroups; ++dgIdx) {

		dgp = xml_config->fm_instance[sm_instance]->dg_config.dg[dgIdx];

		if (dgp != NULL) {

			//loop on node descriptions
			if (dgp->number_of_node_descriptions > 0) {

				XmlNode_t* nodeDescPtr = dgp->node_description;
				RegExp_t*  regExprPtr = dgp->reg_expr;

				while ( (nodeDescPtr != NULL)&&(regExprPtr != NULL) ) {

					initializeRegexStruct(regExprPtr);					

					boolean isValid = FALSE;

					isValid = convertWildcardedString(nodeDescPtr->node, &regExprPtr->regexString[0], &regExprPtr->regexInfo);

					if (!isValid) {
						IB_LOG_WARN_FMT(__func__, "convertWildcardedString returned syntax invalid for node description: %s", nodeDescPtr->node);
					}
					else {

#ifdef __VXWORKS__
						RegexBracketParseInfo_t* regexInfoPtr = &regExprPtr->regexInfo;
						if (regexInfoPtr->numBracketRangesDefined > 0) {
							IB_LOG_WARN_FMT(__func__, "DG evaluation ignoring NodeDesc %s: VxWorks does not support bracket syntax in node descriptions", nodeDescPtr->node);
							isValid = FALSE;
						}
						else {
							if ((regExprPtr->regexpCompiled = regcomp(&regExprPtr->regexString[0])) == NULL) {
							IB_LOG_WARN_FMT(__func__, "Could not compile regular expression: %s", &regExprPtr->regexString[0]);
								isValid = FALSE;
						}
						else {
							isValid = TRUE;
						}
						}
#else
						//compile into regular expression if there aren't any syntax issues
						int regCompRetVal = regcomp(&regExprPtr->regexCompiled, &regExprPtr->regexString[0], REG_EXTENDED);

						if (regCompRetVal != 0) {
							char errorMsg[1000];
							regerror(regCompRetVal, &regExprPtr->regexCompiled, errorMsg, sizeof(errorMsg));
							IB_LOG_WARN_FMT(__func__, "Could not compile regular expression: %s: ErrorCode: %d Detail: %s", &regExprPtr->regexString[0], regCompRetVal, errorMsg);
							isValid = FALSE;
						}
					else {
							isValid = TRUE;
					}
#endif
					}

					//save isValid flag
					regExprPtr->isSyntaxValid = isValid;

					nodeDescPtr = nodeDescPtr->next;
					regExprPtr = regExprPtr->next;
				}
			}
		}
	}

	return VSTATUS_OK; 
}

void sm_compute_pool_size(void)
{
#ifdef __VXWORKS__
	// VxWorks computes smPoolSize in EsmInit.c based on memory available
#else
	// this doesn't really matter on linux
	g_smPoolSize = 0x10000000;
#endif
}
		
#ifndef __VXWORKS__
// initialize the memory pool for XML parsing
Status_t 
initSmXmlMemoryPool(void) {

	Status_t	status;
	uint32_t sm_xml_bytes;

	sm_xml_bytes = xml_compute_pool_size(/* one instance of sm */ 0);

    memset(&sm_xml_pool, 0, sizeof(sm_xml_pool));
	status = vs_pool_create(&sm_xml_pool, 0, (uint8_t *)"sm_xml_pool", NULL, sm_xml_bytes);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_main: can't create SM XML pool, ABORTING SM START");
        memset(&sm_xml_pool, 0, sizeof(sm_xml_pool));
        return status;
	}
	return VSTATUS_OK; 
}

// get memory for XML parser
void* 
getSmXmlParserMemory(uint32_t size, char* info) {
	void 		*address;
	Status_t    status;

#ifdef XML_MEMORY
	printf("called getSmXmlParserMemory() size (%u) (%s) from sm_main.c\n", size, info);
#endif
	status = vs_pool_alloc(&sm_xml_pool, size, (void*)&address);
	if (status != VSTATUS_OK || !address)
		return NULL;
	return address;
}

Status_t 
handleVfDgMemory(void) {

	Status_t status = VSTATUS_OK;

	//allocate memory for arrays inside vf_config, dg_config
	int idx;
	for (idx = 0; idx<xml_config->fm_instance[sm_instance]->vf_config.number_of_vfs; idx++) {
		status = vs_pool_alloc(&sm_pool, sizeof(VFConfig_t), (void*)&vf_config.vf[idx]);
		if (status != VSTATUS_OK || !vf_config.vf[idx]) {
			IB_LOG_ERROR0("can't malloc vf_config.vf");
			return status;
		}
	}

	for (idx = 0; idx<xml_config->fm_instance[sm_instance]->dg_config.number_of_dgs; idx++) {
		status = vs_pool_alloc(&sm_pool, sizeof(VFConfig_t), (void*)&dg_config.dg[idx]);
		if (status != VSTATUS_OK || !dg_config.dg[idx]) {
			IB_LOG_ERROR0("can't malloc dg_config.dg");
			return status;
		}
	}

	//copy xml_config memory to dg_config, and vf_config
	if (copyDgVfInfo(xml_config->fm_instance[sm_instance], &dg_config, &vf_config)) {
		IB_FATAL_ERROR_NODUMP("can't copy VF DG configuration");
		status = VSTATUS_NOMEM;
	}

	return status;
}

// free memory for XML parser
void 
freeSmXmlParserMemory(void *address, uint32_t size, char* info) {

#ifdef XML_MEMORY
	printf("called freeSmXmlParserMemory() size (%u) (%s) from sm_main.c\n", size, info);
#endif
	vs_pool_free(&sm_xml_pool, address);

}

Status_t
sm_initialize_sm_pool(void) {

	Status_t status = VSTATUS_OK;

	sm_compute_pool_size();

    memset(&sm_pool, 0, sizeof(sm_pool));
	status = vs_pool_create(&sm_pool, 0, (uint8_t *)"sm_pool", NULL, g_smPoolSize);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create SM pool, ABORTING SM START");
        memset(&sm_pool,0,sizeof(sm_pool));
        return status;
	}

	return status;
}

#endif // __VXWORKS__

void sm_test_logging_macros(void) {
#if 0
	// some tests of various log routines and macros
	IB_LOG_ERROR0("Test IB_LOG_ERROR0");
	IB_LOG_ERROR("Test IB_LOG_ERROR with 55 as value", 55);
	IB_LOG_ERRORX("Test IB_LOG_ERROR with 33 as value", 33);
	IB_LOG_ERRORLX("Test IB_LOG_ERROR with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_ERROR64("Test IB_LOG_ERROR with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_ERRORSTR("Test IB_LOG_ERROR with string as value", "string");
	IB_LOG_ERRORRC("Test IB_LOG_ERROR with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_WARN0("Test IB_LOG_WARN0");
	IB_LOG_WARN("Test IB_LOG_WARN with 55 as value", 55);
	IB_LOG_WARNX("Test IB_LOG_WARN with 33 as value", 33);
	IB_LOG_WARNLX("Test IB_LOG_WARN with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_WARN64("Test IB_LOG_WARN with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_WARNSTR("Test IB_LOG_WARN with string as value", "string");
	IB_LOG_WARNRC("Test IB_LOG_WARN with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_NOTICE0("Test IB_LOG_NOTICE0");
	IB_LOG_NOTICE("Test IB_LOG_NOTICE with 55 as value", 55);
	IB_LOG_NOTICEX("Test IB_LOG_NOTICE with 33 as value", 33);
	IB_LOG_NOTICELX("Test IB_LOG_NOTICE with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_NOTICE64("Test IB_LOG_NOTICE with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_NOTICESTR("Test IB_LOG_NOTICE with string as value", "string");
	IB_LOG_NOTICERC("Test IB_LOG_NOTICE with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_INFINI_INFO0("Test IB_LOG_INFINI_INFO0");
	IB_LOG_INFINI_INFO("Test IB_LOG_INFINI_INFO with 55 as value", 55);
	IB_LOG_INFINI_INFOX("Test IB_LOG_INFINI_INFO with 33 as value", 33);
	IB_LOG_INFINI_INFOLX("Test IB_LOG_INFINI_INFO with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_INFINI_INFO64("Test IB_LOG_INFINI_INFO with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_INFINI_INFOSTR("Test IB_LOG_INFINI_INFO with string as value", "string");
	IB_LOG_INFINI_INFORC("Test IB_LOG_INFINI_INFO with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_INFO0("Test IB_LOG_INFO0");
	IB_LOG_INFO("Test IB_LOG_INFO with 55 as value", 55);
	IB_LOG_INFOX("Test IB_LOG_INFO with 33 as value", 33);
	IB_LOG_INFOLX("Test IB_LOG_INFO with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_INFO64("Test IB_LOG_INFO with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_INFOSTR("Test IB_LOG_INFO with string as value", "string");
	IB_LOG_INFORC("Test IB_LOG_INFO with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_VERBOSE0("Test IB_LOG_VERBOSE0");
	IB_LOG_VERBOSE("Test IB_LOG_VERBOSE with 55 as value", 55);
	IB_LOG_VERBOSEX("Test IB_LOG_VERBOSE with 33 as value", 33);
	IB_LOG_VERBOSELX("Test IB_LOG_VERBOSE with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_VERBOSE64("Test IB_LOG_VERBOSE with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_VERBOSESTR("Test IB_LOG_VERBOSE with string as value", "string");
	IB_LOG_VERBOSERC("Test IB_LOG_VERBOSE with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_DATA("Test IB_LOG_DATA with str","Memory will be dumped in hex for easy reading", 45);
	IB_LOG_DEBUG1_0("Test IB_LOG_DEBUG1_0");
	IB_LOG_DEBUG1("Test IB_LOG_DEBUG1 with 55 as value", 55);
	IB_LOG_DEBUG1X("Test IB_LOG_DEBUG1 with 33 as value", 33);
	IB_LOG_DEBUG1LX("Test IB_LOG_DEBUG1 with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_DEBUG1_64("Test IB_LOG_DEBUG1 with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_DEBUG1STR("Test IB_LOG_DEBUG1 with string as value", "string");
	IB_LOG_DEBUG1RC("Test IB_LOG_DEBUG1 with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_DEBUG2_0("Test IB_LOG_DEBUG2_0");
	IB_LOG_DEBUG2("Test IB_LOG_DEBUG2 with 55 as value", 55);
	IB_LOG_DEBUG2X("Test IB_LOG_DEBUG2 with 33 as value", 33);
	IB_LOG_DEBUG2LX("Test IB_LOG_DEBUG2 with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_DEBUG2_64("Test IB_LOG_DEBUG2 with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_DEBUG2STR("Test IB_LOG_DEBUG2 with string as value", "string");
	IB_LOG_DEBUG2RC("Test IB_LOG_DEBUG2 with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_DEBUG3_0("Test IB_LOG_DEBUG3_0");
	IB_LOG_DEBUG3("Test IB_LOG_DEBUG3 with 55 as value", 55);
	IB_LOG_DEBUG3X("Test IB_LOG_DEBUG3 with 33 as value", 33);
	IB_LOG_DEBUG3LX("Test IB_LOG_DEBUG3 with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_DEBUG3_64("Test IB_LOG_DEBUG3 with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_DEBUG3STR("Test IB_LOG_DEBUG3 with string as value", "string");
	IB_LOG_DEBUG3RC("Test IB_LOG_DEBUG3 with rc as value", VSTATUS_TIMEOUT);
	IB_LOG_DEBUG4_0("Test IB_LOG_DEBUG4_0");
	IB_LOG_DEBUG4("Test IB_LOG_DEBUG4 with 55 as value", 55);
	IB_LOG_DEBUG4X("Test IB_LOG_DEBUG4 with 33 as value", 33);
	IB_LOG_DEBUG4LX("Test IB_LOG_DEBUG4 with 0x1234567890abcdef as value", 0x1234567890abcdefULL);
	IB_LOG_DEBUG4_64("Test IB_LOG_DEBUG4 with 1234567812345678 as value", 12345678123456789ULL);
	IB_LOG_DEBUG4STR("Test IB_LOG_DEBUG4 with string as value", "string");
	IB_LOG_DEBUG4RC("Test IB_LOG_DEBUG4 with rc as value", VSTATUS_TIMEOUT);
	IB_ENTER(__func__,4);
	IB_LOG_ARGS1(1);
	IB_LOG_ARGS2(1,2);
	IB_LOG_ARGS3(1,2,3);
	IB_LOG_ARGS4(1,2,3,4);
	IB_LOG_ARGS5(1,2,3,4,5);
	IB_EXIT0("Test IB_EXIT with no value");
	IB_EXIT(__func__, 55);
	cs_log(VS_LOG_ERROR, "test_func", "Test cs_log VS_LOG_ERROR with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_ERROR_FMT(__func__, "Test IB_LOG_ERROR_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_WARN, "test_func", "Test cs_log VS_LOG_WARN with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_WARN_FMT(__func__, "Test IB_LOG_WARN_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_NOTICE, "test_func", "Test cs_log VS_LOG_NOTICE with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_NOTICE_FMT(__func__, "Test IB_LOG_NOTICE_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_INFINI_INFO, "test_func", "Test cs_log VS_LOG_INFINI_INFO with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_INFINI_INFO_FMT(__func__, "Test IB_LOG_INFINI_INFO_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_INFO, "test_func", "Test cs_log VS_LOG_INFO with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_INFO_FMT(__func__, "Test IB_LOG_INFO_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_VERBOSE, "test_func", "Test cs_log VS_LOG_VERBOSE with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_VERBOSE_FMT(__func__, "Test IB_LOG_VERBOSE_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_DATA, "test_func", "Test cs_log VS_LOG_DATA with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_DEBUG1, "test_func", "Test cs_log VS_LOG_DEBUG1 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG1_FMT(__func__, "Test IB_LOG_DEBUG1_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_DEBUG2, "test_func", "Test cs_log VS_LOG_DEBUG2 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG2_FMT(__func__, "Test IB_LOG_DEBUG2_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_DEBUG3, "test_func", "Test cs_log VS_LOG_DEBUG3 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG3_FMT(__func__, "Test IB_LOG_DEBUG3_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_DEBUG4, "test_func", "Test cs_log VS_LOG_DEBUG4 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG4_FMT(__func__, "Test IB_LOG_DEBUG4_FMT with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_ENTER, "test_func", "Test cs_log VS_LOG_ENTER with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_ARGS, "test_func", "Test cs_log VS_LOG_ARGS with 55, 6 as values %u, %u", 55, 6);
	cs_log(VS_LOG_EXIT, "test_func", "Test cs_log VS_LOG_EXIT with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_ERROR, "test_vf", "test_func", "Test cs_log_vf VS_LOG_ERROR with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_ERROR_FMT_VF("test_vf","test_func", "Test IB_LOG_ERROR_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_WARN, "test_vf", "test_func", "Test cs_log_vf VS_LOG_WARN with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_WARN_FMT_VF("test_vf", "test_func", "Test IB_LOG_WARN_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_NOTICE, "test_vf", "test_func", "Test cs_log_vf VS_LOG_NOTICE with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_NOTICE_FMT_VF("test_vf", "test_func", "Test IB_LOG_NOTICE_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_INFINI_INFO, "test_vf", "test_func", "Test cs_log_vf VS_LOG_INFINI_INFO with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_INFINI_INFO_FMT_VF("test_vf", "test_func", "Test IB_LOG_INFINI_INFO_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_INFO, "test_vf", "test_func", "Test cs_log_vf VS_LOG_INFO with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_INFO_FMT_VF("test_vf", "test_func", "Test IB_LOG_INFO_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_VERBOSE, "test_vf", "test_func", "Test cs_log_vf VS_LOG_VERBOSE with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_VERBOSE_FMT_VF("test_vf", "test_func", "Test IB_LOG_VERBOSE_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_DATA, "test_vf", "test_func", "Test cs_log_vf VS_LOG_DATA with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_DEBUG1, "test_vf", "test_func", "Test cs_log_vf VS_LOG_DEBUG1 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG1_FMT_VF("test_vf", "test_func", "Test IB_LOG_DEBUG1_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_DEBUG2, "test_vf", "test_func", "Test cs_log_vf VS_LOG_DEBUG2 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG2_FMT_VF("test_vf", "test_func", "Test IB_LOG_DEBUG2_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_DEBUG3, "test_vf", "test_func", "Test cs_log_vf VS_LOG_DEBUG3 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG3_FMT_VF("test_vf", "test_func", "Test IB_LOG_DEBUG3_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_DEBUG4, "test_vf", "test_func", "Test cs_log_vf VS_LOG_DEBUG4 with 55, 6 as values %u, %u", 55, 6);
	IB_LOG_DEBUG4_FMT_VF("test_vf", "test_func", "Test IB_LOG_DEBUG4_FMT_VF with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_ENTER, "test_vf", "test_func", "Test cs_log_vf VS_LOG_ENTER with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_ARGS, "test_vf", "test_func", "Test cs_log_vf VS_LOG_ARGS with 55, 6 as values %u, %u", 55, 6);
	cs_log_vf(VS_LOG_EXIT, "test_vf", "test_func", "Test cs_log_vf VS_LOG_EXIT with 55, 6 as values %u, %u", 55, 6);
	smCsmLogMessage(CSM_SEV_ERROR, CSM_COND_SECURITY_ERROR, NULL, NULL, "Test CSM ERROR message with 55, 6 as values %u, %u", 55, 6);
	smCsmLogMessage(CSM_SEV_WARNING, CSM_COND_SECURITY_ERROR, NULL, NULL, "Test CSM WARNING message with 55, 6 as values %u, %u", 55, 6);
	smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SECURITY_ERROR, NULL, NULL, "Test CSM NOTICE message with 55, 6 as values %u, %u", 55, 6);
	smCsmLogMessage(CSM_SEV_INFO, CSM_COND_SECURITY_ERROR, NULL, NULL, "Test CSM INFO message with 55, 6 as values %u, %u", 55, 6);
	//IB_FATAL_ERROR("Test IB_LOG_FATAL");
	log_mask &= ~(VS_LOG_TRACE);	// reduce logging for rest of run
#endif
}

Status_t
sm_main(void) {
	Status_t	status;
#ifndef __VXWORKS__
	int         startPM=0;
#ifdef FE_THREAD_SUPPORT_ENABLED
	int         startFE=0;
#endif
#endif

	IB_ENTER(__func__, 0, 0, 0, 0);

//
//	Check for authorization and licenses.
//
	if (authorization_userexit() != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("authorization failed");
	}

	if (license_userexit() != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("license failed");
	}

    (void) sm_spanning_tree_resetGlobals();

//
//	Fetch the environment.
//
#ifndef __VXWORKS__

    // get PM related XML configuration parameters so we know if it should
	// be started
    status = pm_initialize_config();
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't retrieve PM XML configuration");
		return status;
    }
    startPM = pm_config.start;

    // get FE related XML configuration parameters so we know if it should
	// be started
    status = fe_initialize_config(xml_config, sm_instance);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't retrieve FE XML configuration");
		return status;
    }

#ifdef FE_THREAD_SUPPORT_ENABLED
    startFE = fe_config.start;
#endif

#else // __VXWORKS__
	// Parse the XML configuration
	status = sm_parse_xml_config();
	if (status != VSTATUS_OK) {
		return status;
	}

	if (copyDgVfInfo(xml_config->fm_instance[sm_instance], &dg_config, &vf_config)) {
		IB_FATAL_ERROR("can't copy VF DG configuration");
		return VSTATUS_NOMEM;
	}

#endif // __VXWORKS__

    sm_init_plt_table();

#ifndef __VXWORKS__
	// since the XML VirtualFabrics configuration has been rendered the memory
	// used for parsing XML and some of the common memory can be released
	releaseXmlConfig(xml_config, /* full */ 1);
	xml_config = NULL;
#endif

	sm_test_logging_macros();
    /* dbsync interval used internally in seconds */
    sm_config.db_sync_interval = ((sm_config.db_sync_interval > 60) ? (60*60) : (sm_config.db_sync_interval*60));
	sa_max_cntxt = 2 * sm_config.subnet_size;

	// the same value for embedded, but grows nicely for Host SM
	sa_data_length = 512 * cs_numPortRecords(sm_config.subnet_size);
	sa_max_path_records = sa_data_length / (sizeof(IB_PATH_RECORD) + Calculate_Padding(sizeof(IB_PATH_RECORD)));

    sm_mkey_protect_level = (sm_config.mkey) ? sm_default_mkey_protect_level : 0;
    sm_mkey_lease_period = (sm_config.mkey) ? sm_config.timer : 0;  /* this is in seconds */
	{
		char buf[200];

    	sprintf(buf, 
           "SM: GidPrefix="FMT_U64", Key="FMT_U64", "
           "MKey="FMT_U64" : protect_level=%d : lease=%d seconds, dbsync interval=%d seconds",
           sm_config.subnet_prefix, sm_config.sm_key, sm_config.mkey, sm_default_mkey_protect_level, 
		   sm_mkey_lease_period, (unsigned int)sm_config.db_sync_interval);
		vs_log_output_message(buf, FALSE);
	}
	sm_config.timer *= 1000000;
	sm_lid = sm_config.lid;

    /* 
     * if LMC is zero, we will use this offset in the lid allocation logic 
     * value of 1 is the default behavior for LMC=0
     * setting this to 16 would allocate lids in multiples of 16, spreading the lid range
     * sneaky way to make a small fabric have large lid values
     */
    if (sm_config.topo_lid_offset < 1 || sm_config.topo_lid_offset > 256) {
        sm_config.topo_lid_offset = 1;
    } else if (sm_config.topo_lid_offset > 1) {
        IB_LOG_INFINI_INFO("SM will allocate lids in increments of ", sm_config.topo_lid_offset);
    }

#ifndef __VXWORKS__

	if (!sm_nodaemon) {
		int	ret_value;
		IB_LOG_INFO("Trying daemon, sm_nodaemon =", sm_nodaemon);
		if ((ret_value = daemon(1, 0))) {
			int localerrno = errno;
			IB_LOG_ERROR("daemon failed with return value of", ret_value);
			IB_LOG_ERROR(strerror(localerrno), localerrno);
		}
	}

#endif

//
//	Initialize the pools.
//
	{
		char buf[140];

		sprintf(buf, "SM: Size Limits: EndNodePorts=%u Nodes=%u Ports=%u Links=%u",
				(unsigned)sm_config.subnet_size,
				(unsigned)cs_numNodeRecords(sm_config.subnet_size),
				(unsigned)cs_numPortRecords(sm_config.subnet_size),
				(unsigned)cs_numLinkRecords(sm_config.subnet_size));
		vs_log_output_message(buf, FALSE);

		sprintf(buf, "SM: Memory: Pool=%uK SA Resp=%uK",
				(unsigned)(g_smPoolSize+1023)/1024,
				(unsigned)(sa_data_length+1023)/1024);
		vs_log_output_message(buf, FALSE);
	}

	lidmap = NULL;
	status = vs_pool_alloc(&sm_pool, sizeof(LidMap_t) * (UNICAST_LID_MAX + 1), (void*)&lidmap);
	if (status != VSTATUS_OK || !lidmap) {
		status = VSTATUS_NOMEM;
		return status;
	}
	memset(lidmap, 0, sizeof(LidMap_t) * (UNICAST_LID_MAX + 1));

	status = vs_pool_alloc(&sm_pool, sizeof(cl_qmap_t), (void *)&sm_GuidToLidMap);
	if (status != VSTATUS_OK || !sm_GuidToLidMap) {
		IB_LOG_ERROR0("can't malloc GuidToLidMap");
		status = VSTATUS_NOMEM;
		return status;
	}
	cl_qmap_init(sm_GuidToLidMap, NULL);

	sm_lmc_e0_freeLid_hint = 1 << sm_config.lmc_e0;
	sm_lmc_freeLid_hint = 1 << sm_config.lmc;
	sm_lmc_0_freeLid_hint = 1;

	sm_threads = NULL;
	status = vs_pool_alloc(&sm_pool, sizeof(SMThread_t) * (SM_THREAD_MAX + 1), (void*)&sm_threads);
	if (status != VSTATUS_OK || !sm_threads) {
		status = VSTATUS_NOMEM;
		return status;
	}

	uniqueSpanningTrees = NULL;
	status = vs_pool_alloc(&sm_pool, sizeof(McSpanningTree_t *) * (STL_MTU_MAX * IB_STATIC_RATE_MAX), (void*)&uniqueSpanningTrees);
	if (status != VSTATUS_OK || !uniqueSpanningTrees) {
		IB_LOG_ERROR0("can't allocate uniqueSpanningTrees array from SM memory pool");
		status = VSTATUS_NOMEM;
		return status;
	}

	if ((status = sm_routing_init() != VSTATUS_OK))
		return status;

	if ((status = sm_routing_makeModule(sm_config.routing_algorithm,
		&sm_main_routingModule)) != VSTATUS_OK) {

		if (sm_main_routingModule)
			sm_routing_freeModule(&sm_main_routingModule);

		IB_LOG_ERROR_FMT(__func__,
			"Failed to instantiate routing module \"%s\".", sm_config.routing_algorithm);
		return status;
	}

	if (sm_main_routingModule == NULL) {
		IB_LOG_ERROR_FMT(__func__, "Failed to instantiate routing module");
		return VSTATUS_BAD;
	}

	status = sm_routing_makeCopy(&old_topology.routingModule,
		sm_main_routingModule);

	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to copy main thread routing module");
		return status;
	}

	if ((status = sm_process_vf_info(initialVfPtr)) != VSTATUS_OK)
		return status;

	//Set VirtualFabrics* in the topology_t structure
	old_topology.vfs_ptr = initialVfPtr;


//
//	Initialize the semaphores.
//
	if ((status = cs_sema_create(&topology_sema, 0)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize topology semaphore");
	}

	if ((status = cs_sema_create(&topology_rcv_sema, 0)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize topology receive semaphore");
	}

	if ((status = cs_sema_create(&sa_sema, 0)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize sa semaphore");
	}

    //
    //	Initialize the locks
    //
	status = vs_lock_init(&old_topology_lock, VLOCK_FREE, VLOCK_RWTHREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize old_topology lock");
	}

	status = vs_lock_init(&new_topology_lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize new_topology lock");
	}

	status = vs_lock_init(&tid_lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize tid lock");
	}

	status = vs_lock_init(&sa_lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize sa lock");
	}

	status = vs_lock_init(&handover_sent_lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize handover_sent lock");
	}

	triggered_handover= 0;
	handover_sent = 0;

	status = vs_lock_init(&sm_mcSpanningTreeRootGuidLock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize Multicast Spanning Tree Guid lock");
	}

	status = vs_lock_init(&sm_datelineSwitchGUIDLock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize Dateline Switch GUID lock");
	}

#ifdef __LINUX__
	status = vs_lock_init(&linux_shutdown_lock, VLOCK_FREE, VLOCK_RWTHREAD);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize shutdown lock");
	}
#endif

    //
    //	Initialize the MAI subsystem.
    //
	mai_set_num_end_ports( MIN(2*sm_config.subnet_size, MAI_MAX_QUEUED_DEFAULT));
	mai_init();

#ifndef __VXWORKS__
	FILE * tmp_log_file = strlen(sm_config.log_file) > 0 ? vs_log_get_logfile_fd() : NULL;
	struct omgt_params params = {.error_file = sm_log_level > 0 ? tmp_log_file : NULL,
	                             .debug_file = sm_log_level > 2 ? tmp_log_file : NULL};
	status = ib_init_devport(&sm_config.hca, &sm_config.port, &sm_config.port_guid, &params);
#else
	status = ib_init_devport(&sm_config.hca, &sm_config.port, &sm_config.port_guid);
#endif

	if (status != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("sm_main: Failed to bind to device; terminating");
	sm_portguid = sm_config.port_guid;
	if (ib_register_sm((int)sa_max_cntxt+32) != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("sm_main: Failed to register management classes; terminating");
#ifndef __VXWORKS__
	{
		char buf[140];

		sprintf(buf, "SM: Using: HFI %u Port %u PortGuid "FMT_U64,
				(unsigned)sm_config.hca+1, (unsigned)sm_config.port, sm_config.port_guid);
		vs_log_output_message(buf, FALSE);
	}
#endif

    //
    //	Open all of the MAI interfaces.
    //
	// used by the SA for new queries
	if ((status = mai_open(1, sm_config.hca, sm_config.port, &fd_sa)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_sa");
	}

	// used by the SA to handle RMPP responses and acks
	if ((status = mai_open(1, sm_config.hca, sm_config.port, &fd_sa_writer)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_sa_writer");
	}

	// used by the notice async context to handle SA reports (notices)
	if ((status = mai_open(1, sm_config.hca, sm_config.port, &fd_saTrap)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_saTrap");
	}

	// used by the async thread to catch traps and SMInfo requests
	if ((status = mai_open(0, sm_config.hca, sm_config.port, &fd_async)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_async");
	}

	// used by the SA (via async thread) to handle outbound queries (e.g. PortInfo in response to traps)
	if ((status = mai_open(0, sm_config.hca, sm_config.port, &fd_async_request)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_async_request");
	}

	// used by the fsm (via async thread) for SMInfo and PortInfo GETs
	if ((status = mai_open(0, sm_config.hca, sm_config.port, &fd_sminfo)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_sminfo");
	}

	// used for config consistency in the dbsync thread
	if ((status = mai_open(0, sm_config.hca, sm_config.port, &fd_dbsync)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_dbsync");
	}

	// used by the topology thread for sweep SMPs
	if ((status = mai_open(0, sm_config.hca, sm_config.port, &fd_topology)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_topology");
	}

	// used by the topology rcv thread for the async context
	// (async LFT, MFT, and GuidInfo)
	if ((status = mai_open(0, sm_config.hca, sm_config.port, &fd_atopology)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_atopology");
	}

	// used to transmit loop packets
	if ((status = mai_open(1, sm_config.hca, sm_config.port, &fd_loopTest)) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't open fd_loopTest");
	}

	IB_LOG_INFO("fd_sa", fd_sa);
	IB_LOG_INFO("fd_sa_writer", fd_sa_writer);
	IB_LOG_INFO("fd_async_request", fd_async_request);
	IB_LOG_INFO("fd_saTrap", fd_saTrap);
	IB_LOG_INFO("fd_async", fd_async);
	IB_LOG_INFO("fd_sminfo", fd_sminfo);
	IB_LOG_INFO("fd_dbsync", fd_dbsync);
	IB_LOG_INFO("fd_topology", fd_topology);
	IB_LOG_INFO("fd_atopology", fd_atopology);
	IB_LOG_INFO("fd_loopTest", fd_loopTest);

    //
    //	Create the SMInfo_t structure.
    //
	sm_state = sm_prevState = SM_STATE_NOTACTIVE;
	sm_smInfo.PortGUID = sm_portguid;
	sm_smInfo.SM_Key = sm_config.sm_key;
	sm_smInfo.ActCount = 0;
	sm_smInfo.u.s.Priority = sm_config.priority;
	sm_smInfo.u.s.InitialPriority = sm_config.priority;
	sm_smInfo.u.s.ElevatedPriority = sm_config.elevated_priority;
	sm_masterStartTime = 0;

#if defined __VXWORKS__
	smCsmSetLogSmDesc(Ics_GetIBDesc(), 0, sm_portguid);
#else
	gethostname(hostName, 64);
	smCsmSetLogSmDesc(hostName, sm_config.port, sm_portguid);
#endif

    //
    // Initialize the SA data structures
    //
    if (sa_main()) {
		IB_FATAL_ERROR_NODUMP("can't initialize SA data structures");
		return(VSTATUS_BAD);
	}

	//
	// Initialize async thread static data
	//
	(void)async_init();

    /*
     * Init SM dbsync thread data
    */
    (void)sm_dbsync_init();

#if 0
	// initialize job management
	status = sm_jm_init_job_table();
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("Failed to initialize Job Management data structures");
		return VSTATUS_BAD;
	}
#endif
   
    // Initialize the SSL/TLS network security interface
    if (sm_config.SslSecurityEnabled) 
        (void)if3_ssl_init(&sm_pool);

//
//	Start the SA reader thread.
//
#ifdef __VXWORKS__
	sm_threads[SM_THREAD_SA_READER].id = (uint8_t*)"esm_sar";
#else
	sm_threads[SM_THREAD_SA_READER].id = (void *)"sareader";
#endif
	sm_threads[SM_THREAD_SA_READER].function = sa_main_reader;

	status = sm_start_thread(&sm_threads[SM_THREAD_SA_READER]);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create SA reader thread");
		return(VSTATUS_BAD);
	}

    //
    //	Start the SA writer thread.
    //
#ifdef __VXWORKS__
	sm_threads[SM_THREAD_SA_WRITER].id = (uint8_t*)"esm_saw";
#else
	sm_threads[SM_THREAD_SA_WRITER].id = (void *)"sawriter";
#endif
	sm_threads[SM_THREAD_SA_WRITER].function = sa_main_writer;

	status = sm_start_thread(&sm_threads[SM_THREAD_SA_WRITER]);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create SA writer thread");
		return(VSTATUS_BAD);
	}

    //
    //	Start the TOPOLOGY thread.
    // 
#ifdef __VXWORKS__
	sm_threads[SM_THREAD_TOPOLOGY].id = (uint8_t*)"esm_top";
#else
	sm_threads[SM_THREAD_TOPOLOGY].id = (void *)"topology";
#endif
	sm_threads[SM_THREAD_TOPOLOGY].function = topology_main;

	status = sm_start_thread(&sm_threads[SM_THREAD_TOPOLOGY]);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create TOPOLOGY thread");
		return(VSTATUS_BAD);
	}

    //
    //	Start the ASYNC thread.
    //
#ifdef __VXWORKS__
	sm_threads[SM_THREAD_ASYNC].id = (uint8_t*)"esm_asy";
#else
	sm_threads[SM_THREAD_ASYNC].id = (void *)"async";
#endif
	sm_threads[SM_THREAD_ASYNC].function = async_main;

	status = sm_start_thread(&sm_threads[SM_THREAD_ASYNC]);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create ASYNC thread");
		return(VSTATUS_BAD);
	}

    //
    //	Start the SM topology discovery async receive thread.
    //
#ifdef __VXWORKS__
	sm_threads[SM_THREAD_TOP_RCV].id = (uint8_t*)"esm_rcv";
#else
	sm_threads[SM_THREAD_TOP_RCV].id = (void *)"topology rcv";
#endif
	sm_threads[SM_THREAD_TOP_RCV].function = topology_rcv;

	status = sm_start_thread(&sm_threads[SM_THREAD_TOP_RCV]);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create SM topology async receive thread");
		return(VSTATUS_BAD);
	}

    //
    //	Start the SM topology discovery async receive thread.
    //
#ifdef __VXWORKS__
	sm_threads[SM_THREAD_DBSYNC].id = (uint8_t*)"esm_dbs";
#else
	sm_threads[SM_THREAD_DBSYNC].id = (void *)"sm_dbsync";
#endif
	sm_threads[SM_THREAD_DBSYNC].function = sm_dbsync;

	status = sm_start_thread(&sm_threads[SM_THREAD_DBSYNC]);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't create SM db sync thread");
		return(VSTATUS_BAD);
	}

#ifdef __VXWORKS__
    // just exit
#else

    //	Start the PM thread.
    if (startPM) {
        pm_running = 1;
    	sm_threads[SM_THREAD_PM].id = (void *)"pm";
    	sm_threads[SM_THREAD_PM].function = unified_sm_pm;
    
    	status = sm_start_thread(&sm_threads[SM_THREAD_PM]);
    	if (status != VSTATUS_OK) {
    		IB_FATAL_ERROR_NODUMP("can't create Performance Manager thread");
    		return(VSTATUS_BAD);
    	}
    }

#ifdef FE_THREAD_SUPPORT_ENABLED
    //	Start the FE thread.
    if (startFE) {
        fe_running = 1;
    	sm_threads[SM_THREAD_FE].id = (void *)"fe";
    	sm_threads[SM_THREAD_FE].function = unified_sm_fe;
    
    	status = sm_start_thread(&sm_threads[SM_THREAD_FE]);
    	if (status != VSTATUS_OK) {
    		IB_FATAL_ERROR_NODUMP("can't create Fabric Executive thread");
    		return(VSTATUS_BAD);
    	}
    }
#endif

	sm_conf_server_init();

#ifndef __VXWORKS__
	if (xml_trace)
		fprintf(stdout, "\nSM Initial Config Done\n");
#endif

    //
    //	Just loop forever.
    //

	while (sm_control_cmd != SM_CONTROL_SHUTDOWN) {
		vs_thread_sleep(VTIMER_1S);

		if (sm_control_cmd == SM_CONTROL_RECONFIG) {

			// Handle reconfiguration request.
			// If one is already in progress,
			// let it complete before starting
			// a new one.
			smProcessReconfigureRequest();
			//reset the sm_control_cmd
			sm_control_cmd = 0;
		}
	}

#ifdef CAL_IBACCESS
    mai_umadt_read_kill();
    /* give readers time to exit else kernel panic */
	vs_thread_sleep(VTIMER_1S/2);
    cal_ibaccess_global_shutdown();
#endif
    /* Wait 1 second for our threads to die. */
	vs_thread_sleep(VTIMER_1S);

	/* Release VF config only after the threads are stopped as the
	 * threads might be using VF related data.
	 */
	VirtualFabrics_t *VirtualFabricsToRelease = old_topology.vfs_ptr;
	releaseVirtualFabricsConfig(VirtualFabricsToRelease);

	sm_routing_freeModule(&sm_main_routingModule);

	if (old_topology.routingModule)
		sm_routing_freeModule(&old_topology.routingModule);
#endif /* #ifdef __VXWORKS__ */

	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t sm_clearIsSM(void) {
#if defined(IB_STACK_OPENIB)
	return ib_disable_is_sm();
#elif defined(CAL_IBACCESS)
	Status_t	status;
	uint32_t	mask;
	
	/* we set/clear the isSm capability mask when we register/unregister with the Ism SM class */
	status = sm_get_CapabilityMask(fd_topology, sm_config.port, &mask);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't get isSM rc:", status);
		return status;
	}
	
	mask &= ~PI_CM_IS_SM;
	status = sm_set_CapabilityMask(fd_topology, sm_config.port, mask);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't clear isSM rc:", status);
	}

	return status;

#else
	return VSTATUS_NOSUPPORT;
#endif
}

void
sm_setPriority(uint32_t priority){
	/* Update the global and the value in the global sm_info structure */
	sm_smInfo.u.s.Priority = priority;
	sm_config.priority = priority;
}

uint32_t sm_get_smPerfDebug(void)
{
	return smDebugPerf;
}
 
void smPerfDebugOn(void)
{
    smDebugPerf = 1;
}

void smPerfDebugOff(void)
{
    smDebugPerf = 0;
}

void smPerfDebugToggle(void)
{
	if (smDebugPerf) {
		smDebugPerf = 0;
	} else {
		smDebugPerf = 1;
	}
}

void smForceRebalanceToggle(void)
{
	if (sm_config.force_rebalance) {
		IB_LOG_INFINI_INFO0("Turning OFF ForceRebalance");
		sm_config.force_rebalance = 0;
		forceRebalanceNextSweep = 0;
	} else {
		IB_LOG_INFINI_INFO0("Turning ON ForceRebalance");
		IB_LOG_INFINI_INFO0("will rebalance static routes on next sweep");
		sm_config.force_rebalance = 1;
		forceRebalanceNextSweep = 1;
	}
}

uint32_t sm_get_smAdaptiveRoutingConfigured(void) {
	return sm_config.adaptiveRouting.enable;
}

uint32_t sm_get_smAdaptiveRouting(void) {
	return sm_adaptiveRouting.enable;
}

void smAdaptiveRoutingToggle(uint32_t externalCmd)
{
	if (sm_adaptiveRouting.enable) {
		IB_LOG_INFINI_INFO0("Disabling Adaptive Routing");
		memset(&sm_adaptiveRouting, 0, sizeof(SmAdaptiveRouting_t));
		if (externalCmd) {
			sm_forceSweep("Disabling Adaptive Routing");
		}

	} else if (sm_config.adaptiveRouting.enable) {
		if (!esmLoopTestOn) {
			IB_LOG_INFINI_INFO0("Re-enabling Adaptive Routing");
			sm_adaptiveRouting.enable = sm_config.adaptiveRouting.enable;
			sm_adaptiveRouting.algorithm = sm_config.adaptiveRouting.algorithm;
			sm_adaptiveRouting.debug = sm_config.adaptiveRouting.debug;
			sm_adaptiveRouting.lostRouteOnly = sm_config.adaptiveRouting.lostRouteOnly;
			sm_adaptiveRouting.arFrequency = sm_config.adaptiveRouting.arFrequency;

			// Requires a forced rebalance to setup tables.
			forceRebalanceNextSweep = 1;
			if (externalCmd) {
				sm_forceSweep("Re-enabling Adaptive Routing");
			}
		} else {
			IB_LOG_WARN0("Cannot re-enable Adaptive Routing while Loop Test is running!");
		}

	} else if (externalCmd) {
		if (!esmLoopTestOn) {
			memset(&sm_adaptiveRouting, 0, sizeof(SmAdaptiveRouting_t));
			sm_adaptiveRouting.enable = 1;

			// Requires a forced rebalance to setup tables.
			forceRebalanceNextSweep = 1;
			sm_forceSweep("Enabling Adaptive Routing");
		} else {
			IB_LOG_WARN0("Cannot re-enable Adaptive Routing while Loop Test is running!");
		}

	}
}

void smSetAdaptiveRouting(uint32_t set) {
	if (set != sm_adaptiveRouting.enable) {
		smAdaptiveRoutingToggle(1);
	}
}

void smPauseResumeSweeps(boolean pauseSweeps) {

	sweepsPaused = pauseSweeps;

	if (sweepsPaused)
		IB_LOG_INFINI_INFO0("SM Sweeps Paused; Sweeps will not continue till a resumeSweeps command is received");
	else {
		IB_LOG_INFINI_INFO0("SM Sweeps Resuming");
		sm_forceSweep("SM Sweeps Resumed");
	}
}

void
smApplyLogLevelChange(FMXmlCompositeConfig_t *xml_config){
	uint32_t currentLogLevel=sm_config.log_level;
	uint32_t newLogLevel=xml_config->fm_instance[sm_instance]->sm_config.log_level;

#ifndef __VXWORKS__
	PMXmlConfig_t* new_pm_config=&xml_config->fm_instance[sm_instance]->pm_config;
#endif	
	if (currentLogLevel != newLogLevel) {
		sm_set_log_level(newLogLevel);
		sm_config.log_level = newLogLevel;

	}

#ifndef __VXWORKS__
	pmApplyLogLevelChange(new_pm_config);
#endif

}

static Status_t
smApplyVfChanges(FMXmlCompositeConfig_t *xml_config, VirtualFabrics_t *newVirtualFabrics){
	sm_QosConfigChange = 1;
	if (sm_assign_qos_params(newVirtualFabrics) != VSTATUS_OK)
		return VSTATUS_BAD;

	sm_resolve_pkeys_for_vfs(newVirtualFabrics);
	return VSTATUS_OK;
}

void
smLogLevelOverride(void){
	sm_set_log_level(sm_log_level);
}

void
smProcessReconfigureRequest(void){
	Status_t s = VSTATUS_OK;
	FMXmlCompositeConfig_t *new_xml_config;
	VirtualFabrics_t *oldVirtualFabrics = NULL;
	VirtualFabrics_t *newVirtualFabrics;
#ifdef __VXWORKS__
    uint32_t embedded = 1;
#else
    uint32_t embedded = 0;
#endif

	IB_ENTER(__func__, 0, 0, 0, 0);

	//Optimization:  Should not allocate memory and release memory every time we receive
	//a reconfigure request.  We should be able to do this once on startup and cleanup
	//to reduce the time a reconfigure request takes to perform.
	//uint32_t xml_bytes_needed;
	//xml_bytes_needed = xml_compute_pool_size(/* one instance of sm */ 0);
	//new_xml_config = (FMXmlCompositeConfig_t*)malloc(xml_bytes_needed);

    IB_LOG_INFINI_INFO0("SM: Processing reconfiguration request");

	// Kind of convoluted. If there is an updatedVirtualFabrics pointer, that is the
	// "previous" configuration. If not, then the previous configuration is in
	// the old_topology.
	(void)vs_lock(&new_topology_lock);
	if (updatedVirtualFabrics) oldVirtualFabrics = updatedVirtualFabrics;
	(void)vs_unlock(&new_topology_lock);

	new_xml_config = parseFmConfig(sm_config_filename, IXML_PARSER_FLAG_NONE, sm_instance, /* full parse */ 0, /* embedded */ embedded);
	if (!new_xml_config || !new_xml_config->fm_instance[sm_instance]) {
		IB_LOG_WARN0("SM: Error processing reconfigure request; parseFmConfig failed on new XML file");
	}
	else {
		//If overall checksum has not changed, no dynamic configuration updates to do
		boolean configChanged = FALSE;

		newVirtualFabrics = renderVirtualFabricsConfig(sm_instance, new_xml_config, &new_xml_config->fm_instance[sm_instance]->sm_config, NULL);
		if (newVirtualFabrics) {
			if (!applyVirtualFabricRules(newVirtualFabrics, print_error, print_warning)) {
				releaseVirtualFabricsConfig(newVirtualFabrics);
				newVirtualFabrics = NULL;
			}
		}
		if (newVirtualFabrics != NULL) {

			(void)vs_rdlock(&old_topology_lock);
			if (!oldVirtualFabrics) oldVirtualFabrics = old_topology.vfs_ptr;

			if ( (sm_config.overall_checksum != new_xml_config->fm_instance[sm_instance]->sm_config.overall_checksum) ||
				 (pm_config.overall_checksum != new_xml_config->fm_instance[sm_instance]->pm_config.overall_checksum) ||
				 (oldVirtualFabrics->overall_checksum != newVirtualFabrics->overall_checksum) ) {
				configChanged = TRUE;
			}

			if (configChanged) {
				//Verify that no disruptive checksums have changed

				if (!sm_config_valid(new_xml_config, newVirtualFabrics, oldVirtualFabrics) || !pm_config_valid(new_xml_config)) {
					IB_LOG_WARN0("SM: Failed processing reconfigure request; XML contains invalid changes; reconfiguration request being ignored");

					(void)vs_rwunlock(&old_topology_lock);
					releaseVirtualFabricsConfig(newVirtualFabrics);
				} else {

					uint32_t savedVfConsistencyChecksum;

					(void)vs_rwunlock(&old_topology_lock);

					smApplyLogLevelChange(new_xml_config);

					// Lock out any sweeps
					(void)vs_lock(&new_topology_lock);

					// If we have an updated Virtual Fabric, that means
					// no sweep has occurred since the last reconfigure
					// (possibly because we are a standby). Ditch the
					// previous reconfigure update, and switch to the
					// current value.
					if (updatedVirtualFabrics) {
						releaseVirtualFabricsConfig(updatedVirtualFabrics);
					}
					updatedVirtualFabrics = newVirtualFabrics;

					s = smApplyVfChanges(new_xml_config, updatedVirtualFabrics);
					if (s != VSTATUS_OK)
						IB_FATAL_ERROR_NODUMP("smApplyVfChanges() failed");

					savedVfConsistencyChecksum = updatedVirtualFabrics->consistency_checksum;

					//check if activating or deactivating a VF cause changes in mcgroups
					IB_LOG_INFINI_INFO0("SM: Checking changes in MC groups");
					sm_update_mc_groups( newVirtualFabrics, oldVirtualFabrics);

					// Allow sweeps to run
					(void)vs_unlock(&new_topology_lock);

					/* Update our consistency checksums */
    				sm_dbsync_checksums(savedVfConsistencyChecksum,
                       					new_xml_config->fm_instance[sm_instance]->sm_config.consistency_checksum,
                       					new_xml_config->fm_instance[sm_instance]->pm_config.consistency_checksum
                                        );
					sm_config.overall_checksum = new_xml_config->fm_instance[sm_instance]->sm_config.overall_checksum;
		 			pm_config.overall_checksum = new_xml_config->fm_instance[sm_instance]->pm_config.overall_checksum;

					if (sm_state == SM_STATE_MASTER) {
   						SmRecKeyp       smreckeyp;
   						SmRecp          smrecp;
   						CS_HashTableItr_t itr;
						Status_t status;

						/*
						 * Notify all standby SMs to reread their configuration.
						 */
   						/* lock out service record hash table */
   						if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
       						IB_LOG_ERRORRC("Can't lock SM Record table, rc:", status);
   						} else {
							if (cs_hashtable_count(smRecords.smMap) > 1) {
          						cs_hashtable_iterator(smRecords.smMap, &itr);
          						do {
               						smrecp = cs_hashtable_iterator_value(&itr);
                					smreckeyp = cs_hashtable_iterator_key(&itr);
                					if (smrecp->portguid == sm_smInfo.PortGUID) {
										/* Skip us */
										continue;
                					} else if (smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent <= SM_STATE_STANDBY) {
    									IB_LOG_INFINI_INFO_FMT(__func__,
                                           	"SM: Forwarding reconfiguration request to standby SM at Lid 0x%x, portGuid "FMT_U64,
                                   			smrecp->lid, *smreckeyp);
                            			(void) sm_dbsync_queueMsg(DBSYNC_TYPE_RECONFIG, DBSYNC_DATATYPE_NONE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
                					}
        						} while (cs_hashtable_iterator_advance(&itr));
							}
    						vs_unlock(&smRecords.smLock);
						}
						// After a reconfiguration, force a resweep
						sm_trigger_sweep(SM_SWEEP_REASON_RECONFIG);
					}
    				IB_LOG_INFINI_INFO0("SM: Reconfiguration completed successfully");
				}
			}
			else {
				(void)vs_rwunlock(&old_topology_lock);

    			IB_LOG_INFINI_INFO0("SM: No configuration changes to process; reconfiguration request being ignored");

				//if nothing changed, release the new VirtualFabrics*
				releaseVirtualFabricsConfig(newVirtualFabrics);
			}

		} //if newVirtualFabrics = NULL

	}

	//Optimize: release temp xml memory on sm cleanup, not every time we process
	//reconfigure request

	//Release the temp xml_config memory
	releaseXmlConfig(new_xml_config, /* full */ 1);
	new_xml_config = NULL;

	IB_EXIT(__func__, s);
}


void
sm_shutdown(void){
	(void)pm_main_kill();
    (void)fe_main_kill();
	sa_main_kill();
	topology_main_kill();
	async_main_kill();
    topology_rcv_kill();
    sm_dbsync_kill();
#if 0
	sm_jm_destroy_job_table();
#endif
	sm_clean_vfdg_memory();
	sm_free_vf_mem();
	sm_destroy_qos();

#ifdef __VXWORKS__
	/* Release VF config only after the threads are stopped as the
	 * threads might be using VF related data.
	 */
	VirtualFabrics_t *VirtualFabricsToRelease = old_topology.vfs_ptr;
	releaseVirtualFabricsConfig(VirtualFabricsToRelease);
	sm_routing_freeModule(&sm_main_routingModule);
	
	if (old_topology.routingModule)
		sm_routing_freeModule(&old_topology.routingModule);
#endif
}

void
sm_clean_vfdg_memory(void){

#ifndef __VXWORKS__
	// clear memory used for DG and VF config information
	int idx;
	for (idx = 0; idx<vf_config.number_of_vfs; idx++) {
		vs_pool_free(&sm_pool, (void*)vf_config.vf[idx]);
	}

	for (idx = 0; idx<dg_config.number_of_dgs; idx++) {
		vs_pool_free(&sm_pool, (void*)dg_config.dg[idx]);
	}

#else
	//VxWorks cleans this memory in Esm_Init.c
#endif

}

#ifdef __VXWORKS__
void
sm_cleanGlobals(uint8_t stop){

	if (stop) {
		sm_spanning_tree_resetGlobals();

		if (sm_pool.name[0] != 0) vs_pool_delete (&sm_pool);

    	memset(&sm_pool,0,sizeof(sm_pool));

		lidmap = NULL;

    	memset(sm_env,0,sizeof(sm_env));
	}

    sm_state = sm_prevState = SM_STATE_NOTACTIVE;
    sm_portguid = 0;
    sm_control_cmd = 0;
	numMcGroupClasses = 0;

	sm_useIdealMcSpanningTreeRoot = 1;
	sm_mcSpanningTreeRoot_useLeastWorstCaseCost = 0;
	sm_mcSpanningTreeRoot_useLeastTotalCost = 1;

	sm_mcSpanningTreeRootGuid = 0;
	sm_mcRootCostDeltaThreshold = DEFAULT_MCROOT_COST_IMPROVEMENT_PERCENTAGE;

	sm_datelineSwitchGUID = 0;

	sm_lmc_0_freeLid_hint = 0;
	sm_lmc_e0_freeLid_hint = 0;
	sm_lmc_freeLid_hint = 0;

	if (stop) {
		sm_lid_lo = 1;
		sm_lid_hi = UNICAST_LID_MAX;

		memset(&sm_smInfo,0,sizeof(sm_smInfo));
		sm_masterStartTime = 0;
		sm_McGroups = 0;
    	sm_numMcGroups = 0;
    	sm_McGroups_Need_Prog = 0;

    	memset(&fd_sa,0,sizeof(fd_sa));
		memset(&fd_saTrap,0,sizeof(fd_saTrap));
    	memset(&fd_async,0,sizeof(fd_async));
    	memset(&fd_sminfo,0,sizeof(fd_sminfo));
    	memset(&fd_dbsync,0,sizeof(fd_dbsync));
    	memset(&fd_topology,0,sizeof(fd_topology));
    	memset(&fd_atopology,0,sizeof(fd_atopology));
    	memset(&fd_loopTest,0,sizeof(fd_loopTest));

		sm_threads = NULL;

		esmLoopTestOn = 0;
		esmLoopTestAllowAdaptiveRouting = 0;
		esmLoopTestFast = 0;
		esmLoopTestInjectNode = -1;
		esmLoopTestNumPkts = 1;
		esmLoopTestPathLen = DEFAULT_LOOP_PATH_LENGTH;
		esmLoopTestMinISLRedundancy = 4;
		esmLoopTestTotalPktsInjected = 0;
		esmLoopTestInjectEachSweep = DEFAULT_LOOP_INJECT_EACH_SWEEP;
		esmLoopTestForceInject = 0;
	}

}

void smShowPacketCount()
{
	printf("SM's packet count (sm_smInfo.ActCount) is %d\n", (int)sm_smInfo.ActCount);
}

#endif
