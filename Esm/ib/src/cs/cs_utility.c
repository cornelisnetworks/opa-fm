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

//==============================================================================//
//										//
// FILE NAME									//
//    cs_utility.c								//
//										//
// DESCRIPTION									//
//    This file contains miscellaneous utility routiness.			//
//										//
// DATA STRUCTURES								//
//    None									//
//										//
// FUNCTIONS									//
//    bm_log			write out logging information			//
//										//
// DEPENDENCIES									//
//										//
// HISTORY									//
//										//
//    NAME	DATE  REMARKS							//
//     sjb  04/09/04  Initial creation of file.					//
//										//
//==============================================================================//


#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "cs_log.h"
#include "cs_queue.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_pa.h"
#include "sm_l.h"
#include "stl_sa.h"
#include "mai_g.h"
#include "sm_dbsync.h"
#include "if3.h"


uint32_t cs_log_masks[VIEO_LAST_MOD_ID+1] = {
		DEFAULT_LOG_MASK,	// VIEO_NONE_MOD_ID for vs_syslog_output_message
		0,					// VIEO_CS_MOD_ID   /* Library Modules */
		DEFAULT_LOG_MASK,	// VIEO_MAI_MOD_ID  
		DEFAULT_LOG_MASK,	// VIEO_CAL_MOD_ID 
		DEFAULT_LOG_MASK,	// VIEO_DRIVER_MOD_ID
		DEFAULT_LOG_MASK,	// VIEO_IF3_MOD_ID  
		DEFAULT_LOG_MASK,	// VIEO_SM_MOD_ID  /* Subnet Mgr */
		DEFAULT_LOG_MASK,	// VIEO_SA_MOD_ID  /* Subnet Administrator */
		DEFAULT_LOG_MASK,	// VIEO_PM_MOD_ID  /* Performance Mgr */
		DEFAULT_LOG_MASK,	// VIEO_PA_MOD_ID  /* Performance Administrator */
		DEFAULT_LOG_MASK,	// VIEO_BM_MOD_ID  /* Baseboard Mgr */
		DEFAULT_LOG_MASK,	// VIEO_FE_MOD_ID  /* Fabric Executive */
		DEFAULT_LOG_MASK,	// VIEO_APP_MOD_ID /* Generic VIEO mod id */
};

// module name in form of "name: ", useful for messages where name is optional
// for VIEO_NONE_MOD_ID, returns ""
const char *
cs_log_get_module_prefix(uint32_t modid)
{
	switch (modid) {
	case VIEO_NONE_MOD_ID:	return "";
	case VIEO_CS_MOD_ID:	return "CS: ";
	case VIEO_MAI_MOD_ID:	return "MAI: ";
	case VIEO_CAL_MOD_ID:	return "CAL: ";
	case VIEO_DRIVER_MOD_ID:return "DVR: ";
	case VIEO_IF3_MOD_ID:	return "IF3: ";
	case VIEO_SM_MOD_ID:	return "SM: ";
	case VIEO_SA_MOD_ID:	return "SA: ";
	case VIEO_PM_MOD_ID:	return "PM: ";
	case VIEO_PA_MOD_ID:	return "PA: ";
	case VIEO_BM_MOD_ID:	return "BM: ";
	case VIEO_FE_MOD_ID:	return "FE: ";
	case VIEO_APP_MOD_ID:	return "APP: ";
	default:				return "Unkwn: ";
	}
}

// module name in simple form, useful to put in middle of other messages
// names provided also match prefixes used in LogMask config file names
// for VIEO_NONE_MOD_ID, returns "NONE"
const char *
cs_log_get_module_name(uint32_t modid)
{
	switch (modid) {
	case VIEO_NONE_MOD_ID:	return "NONE";
	case VIEO_CS_MOD_ID:	return "CS";
	case VIEO_MAI_MOD_ID:	return "MAI";
	case VIEO_CAL_MOD_ID:	return "CAL";
	case VIEO_DRIVER_MOD_ID:return "DVR";
	case VIEO_IF3_MOD_ID:	return "IF3";
	case VIEO_SM_MOD_ID:	return "SM";
	case VIEO_SA_MOD_ID:	return "SA";
	case VIEO_PM_MOD_ID:	return "PM";
	case VIEO_PA_MOD_ID:	return "PA";
	case VIEO_BM_MOD_ID:	return "BM";
	case VIEO_FE_MOD_ID:	return "FE";
	case VIEO_APP_MOD_ID:	return "APP";
	default:				return "Unkwn";
	}
}

// convert module name to modid
// names provided also match prefixes used in LogMask config file names
uint32_t
cs_log_get_module_id(const char * mod)
{
	uint32_t modid;

	for (modid = 0 ; modid <= VIEO_LAST_MOD_ID; modid++) {
		if (modid == VIEO_NONE_MOD_ID)
			continue;
		if (0 == strcmp(cs_log_get_module_name(modid), mod))
			return modid;
	}
	return 0;	// VIEO_NONE_MOD_ID
}

const char* 
cs_log_get_sev_name(uint32_t sev)
{
    switch (sev) {
	case VS_LOG_NONE:		return "";
	case VS_LOG_FATAL:		return "FATAL";
	case VS_LOG_CSM_ERROR:	return "ERROR";
	case VS_LOG_CSM_WARN:	return "WARN ";
	case VS_LOG_CSM_NOTICE:	return "NOTIC";
	case VS_LOG_CSM_INFO:	return "INFO ";
	case VS_LOG_ERROR:		return "ERROR";
	case VS_LOG_WARN:		return "WARN ";
	case VS_LOG_NOTICE:		return "NOTIC";
	case VS_LOG_INFINI_INFO:return "PROGR";
	case VS_LOG_INFO:		return "INFO ";
	case VS_LOG_VERBOSE:	return "VBOSE";
	case VS_LOG_DATA:		return "DATA ";
	case VS_LOG_DEBUG1:		return "DBG1 ";
	case VS_LOG_DEBUG2:		return "DBG2 ";
	case VS_LOG_DEBUG3:		return "DBG3 ";
	case VS_LOG_DEBUG4:		return "DBG4 ";
	case VS_LOG_ENTER:		return "ENTER";
	case VS_LOG_ARGS:		return "ARGS ";
	case VS_LOG_EXIT:		return "EXIT ";
	default:				return "UNKWN";
	}
}

// translate level and mode to a sev_mask and a mod_mask of modules
// which sev_mask should be set for.  modules not in mod_mask
// should get a sev_mask of 0
static void
cs_log_translate_level(uint32_t level, int mode,
			   	uint32_t *mod_mask, uint32_t *sev_mask)
{
	switch(level)
	{
	case 5:	
		/* All of the modules on. */
		*mod_mask = VS_MOD_ALL;
		break;
	default:
		*mod_mask = (VS_MOD_ALL & ~(1<<VIEO_CS_MOD_ID));
		break;
	}

	*sev_mask = 0;
	if (mode & 1) {
		switch(level)
		{
		case 7:
			*sev_mask |= (VS_LOG_ARGS | VS_LOG_ENTER | VS_LOG_EXIT);
		case 6:
			*sev_mask |= (VS_LOG_DEBUG3 | VS_LOG_DEBUG4);
		case 5:
			*sev_mask |= (VS_LOG_DEBUG1 | VS_LOG_DEBUG2);
    	case 4:
			*sev_mask |= (VS_LOG_DATA | VS_LOG_VERBOSE );
		case 3:
			*sev_mask |= (VS_LOG_INFO | VS_LOG_ERROR | VS_LOG_WARN
					| VS_LOG_INFINI_INFO | VS_LOG_NOTICE);
    	case 2:
		case 1:
			*sev_mask |= (VS_LOG_CSM_ERROR | VS_LOG_CSM_WARN
				   	| VS_LOG_CSM_NOTICE | VS_LOG_CSM_INFO);
		default:
			*sev_mask |= VS_LOG_FATAL;
		}	
	} else {
		switch(level)
		{
		case 7:
			*sev_mask |= (VS_LOG_ARGS | VS_LOG_ENTER | VS_LOG_EXIT);
		case 6:
			*sev_mask |= (VS_LOG_DEBUG3 | VS_LOG_DEBUG4);
		case 5:
			*sev_mask |= (VS_LOG_DEBUG1 | VS_LOG_DEBUG2);
    	case 4:
			*sev_mask |= (VS_LOG_DATA | VS_LOG_VERBOSE );
		case 3:
			*sev_mask |= (VS_LOG_INFO );
    	case 2:
			*sev_mask |= (VS_LOG_INFINI_INFO | VS_LOG_NOTICE);
		case 1:
			*sev_mask |= (VS_LOG_CSM_ERROR | VS_LOG_CSM_WARN
				   	| VS_LOG_CSM_NOTICE | VS_LOG_CSM_INFO
				   	| VS_LOG_ERROR | VS_LOG_WARN );
		default:
			*sev_mask |= VS_LOG_FATAL;
		}	
	}
}

// fill in log_masks based on mod_mask and sev_mask, presently only used
// internally.  Use of this implies a limit of
// 32 module ids.  Probably enough for now
void
cs_log_set_mods_mask(uint32_t mod_mask, uint32_t sev_mask, uint32_t log_masks[VIEO_LAST_MOD_ID+1])
{
	uint32_t modid;

	for (modid = 0; modid<= VIEO_LAST_MOD_ID; modid++) {
		if (mod_mask & (1 << modid))
			log_masks[modid] = sev_mask;
		else
			log_masks[modid] = 0;
	}
}

// build log_masks based on level and mode
void
cs_log_set_log_masks(uint32_t level, int mode, uint32_t log_masks[VIEO_LAST_MOD_ID+1])
{
	uint32_t mod_mask;
	uint32_t sev_mask;

	cs_log_translate_level(level, mode, &mod_mask, &sev_mask);
	cs_log_set_mods_mask(mod_mask, sev_mask, log_masks);
}

// set a single mask based on module name
void
cs_log_set_log_mask(const char* mod, uint32_t mask, uint32_t log_masks[VIEO_LAST_MOD_ID+1])
{
	uint32_t modid = cs_log_get_module_id(mod);
	if (modid)
		log_masks[modid] = mask | VS_LOG_FATAL;
}

// get a single mask based on module name
uint32_t
cs_log_get_log_mask(const char* mod, uint32_t log_masks[VIEO_LAST_MOD_ID+1])
{
	uint32_t modid = cs_log_get_module_id(mod);
	if (modid)
		return log_masks[modid];
	else
		return 0;
}

//
//	Convert a status code to a string
//
char *
cs_convert_status (Status_t status) {
	switch (status) {
	case VSTATUS_OK: return "0: OK";
	case VSTATUS_BAD: return "1: Bad status";
	case VSTATUS_MISMATCH: return "2: Key mismatch";
	case VSTATUS_DROP: return "3: Drop this packet";
	case VSTATUS_FORWARD: return "4: Forward this packet";
	case VSTATUS_ILLPARM: return "5: Invalid parameter";
	case VSTATUS_NOMEM: return "6: Out of memory";
	case VSTATUS_TIMEOUT: return "7: Timeout";
	case VSTATUS_NOPRIV: return "8: Not enough privs";
	case VSTATUS_BUSY: return "9: Busy";
	case VSTATUS_NODEV: return "10: No object available";
	case VSTATUS_NXIO: return "11: Invalid object";
	case VSTATUS_PARTIAL_PRIVS: return "12: Some privs are avail";
	case VSTATUS_CONDITIONAL: return "13: Conditionally good";
	case VSTATUS_NOPORT: return "14: Port does not exist";
	case VSTATUS_INVALID_HANDL: return "15: invalid handle";
	case VSTATUS_INVALID_TYPE: return "16: invalid type (ib_attr)";
	case VSTATUS_INVALID_ATTR: return "17: invalid attribute (ib_attr)";
	case VSTATUS_INVALID_PROTO: return "18: invalid protocol (RC,RD,UC,UD,RAW)";
	case VSTATUS_INVALID_STATE: return "19: invalid CEP or QP state";
	case VSTATUS_NOT_FOUND: return "20: Not found";
	case VSTATUS_TOO_LARGE: return "21: Too large";
	case VSTATUS_CONNECT_FAILED: return "22: Connect failed";
	case VSTATUS_CONNECT_GONE: return "23: connection gone";
	case VSTATUS_NOHANDLE: return "24: Out of handles";
	case VSTATUS_NOCONNECT: return "25: Out of connections";
	case VSTATUS_EIO: return "26: IO error";
	case VSTATUS_KNOWN: return "27: Already known";
	case VSTATUS_NOT_MASTER: return "28: Not the Master SM";
	case VSTATUS_INVALID_MAD: return "29: invalid MAD";
	case VSTATUS_QFULL: return "30: Queue full";
	case VSTATUS_QEMPTY: return "31: Queue empty";
	case VSTATUS_AGAIN: return "32: Data not available";
	case VSTATUS_BAD_VERSION: return "33: Version mismatch";
	case VSTATUS_UNINIT: return "34: Not initialized";
	case VSTATUS_NOT_OWNER: return "35: Not resource owner";
	case VSTATUS_INVALID_MADT: return "36: Malformed Mai_t";
	case VSTATUS_INVALID_METHOD: return "37: MAD method invalid";
	case VSTATUS_INVALID_HOPCNT: return "38: HopCount not in [0..63]";
	case VSTATUS_MISSING_ADDRINFO: return "39: Mai_t has no addrInfo";
	case VSTATUS_INVALID_LID: return "40: Invalid LID";
	case VSTATUS_INVALID_ADDRINFO: return "41: addrInfo invalid in Mai_t";
	case VSTATUS_MISSING_QP: return "42: QP not in Mai_t";
	case VSTATUS_INVALID_QP: return "43: MAI QP not SMI(0) or GSI(1)";
	case VSTATUS_INVALID_MCLASS: return "48: Reserved mclass used";
	case VSTATUS_INVALID_QKEY: return "51: BTH qkey field for GSI invalid";
	case VSTATUS_INVALID_MADLEN: return "53: MAD datasize invalid/not determinable";
	case VSTATUS_NOSUPPORT: return "55: Function not supported";
	case VSTATUS_IGNORE: return "56: Ignorable condition";
	case VSTATUS_INUSE: return "57: Resource already in use";
	case VSTATUS_INVALID_PORT: return "58: Port invalid";
	case VSTATUS_ATOMICS_NOTSUP: return "59: Atomic operations not supported";
	case VSTATUS_INVALID_ACCESSCTL: return "60: Access definition invalid";
	case VSTATUS_INVALID_ADDR_HANDLE: return "61: Address handle invalid";
	case VSTATUS_INVALID_ADDR: return "62: Address invalid";
	case VSTATUS_INVALID_ARG: return "63: Arguments invalid";
	case VSTATUS_INVALID_HCA_HANDLE: return "64: CA handle invalid";
	case VSTATUS_INVALID_KEY_VALUE: return "65: Partition key invalid";
	case VSTATUS_INVALID_MEMADDR: return "66: Memory not accessible";
	case VSTATUS_INVALID_MEMSIZE: return "67: Memory region size not supported";
	case VSTATUS_INVALID_MIGSTATE: return "68: Migration state requested invalid";
	case VSTATUS_INVALID_NOTICE: return "69: Completion notice requested invalid";
	case VSTATUS_INVALID_OFFSET: return "70: Offset specified invalid";
	case VSTATUS_INVALID_PD: return "71: Protection domain invalid";
	case VSTATUS_INVALID_PKEY_IDX: return "72: Partition key index invalid";
	case VSTATUS_INVALID_RC_TIMER: return "73: Reliable connection timeout invalid";
	case VSTATUS_INVALID_RDD: return "74: Reliable datagram domain invalid";
	case VSTATUS_INVALID_REQUEST: return "75: Request is invalid for this hardware";
	case VSTATUS_INVALID_RNR_CNT: return "76: Receiver not ready count invalid";
	case VSTATUS_INVALID_RNR_TIMER: return "77: Receiver not ready timer invalid";
	case VSTATUS_INVALID_TRANSPORT: return "78: Transport type invalid or not supported";
	case VSTATUS_INVALID_WORKREQ: return "79: Work request invalid";
	case VSTATUS_MCAST_NOTSUP: return "80: Multicast not supported";
	case VSTATUS_MCGRP_QPEXCEEDED: return "81: Too many QPs in multicast group";
	case VSTATUS_MEMRGN_NOTSUP: return "82: Memory regions not supported";
	case VSTATUS_MEMWIN_NOTSUP: return "83: Memory windows not supported";
	case VSTATUS_NORESOURCE: return "84: Out of hardware resources";
	case VSTATUS_OPERATION_DENIED: return "85: Operation cannot be performed";
	case VSTATUS_QP_BUSY: return "86: Queue Pair is already opened";
	case VSTATUS_RDMA_NOTSUP: return "87: Remote DMA operations not supported";
	case VSTATUS_RD_NOTSUP: return "88: Reliable Datagram transport not supported";
	case VSTATUS_SG_TOOMANY: return "89: To many scatter/gather entries";
	case VSTATUS_WQ_RESIZE_NOTSUP: return "90: Work queue resizing not supported";
	case VSTATUS_WR_TOOMANY: return "91: Work queue size not supported";
	case VSTATUS_INVALID_NODE: return "92: Node index invalid";
	case VSTATUS_MAI_INTERNAL: return "93: MAI Internal message";
	case VSTATUS_MAD_OVERFLOW: return "94: MAD buffer overflow";
	case VSTATUS_SIGNAL: return "95: Signal received";
	case VSTATUS_FILTER: return "96: Filter take over notification received";
	case VSTATUS_EXPIRED: return "98: Time period has expired";
	case VSTATUS_EVENT_CONSUMED: return "99: Handler consumed event";
	case VSTATUS_INVALID_PATH: return "100: Path handle invalid";
	case VSTATUS_INVALID_GID_INDEX: return "101: GID index invalid";
	case VSTATUS_INVALID_DEVICE: return "102: Device ordinal out of range";
	case VSTATUS_INVALID_RESOP: return "103: resolver operation invalid";
	case VSTATUS_INVALID_RESCMD: return "104: resolver command invalid";
	case VSTATUS_INVALID_ITERATOR: return "105: resolver iterator invalid";
	case VSTATUS_INVALID_MAGIC: return "106:Incorrect magic number";
	case VSTATUS_BADPAGESIZE: return "107: invalid memory page size specified";
	case VSTATUS_ITERATOR_OUT_OF_DATE: return "119: data referred by case updated or deleted.";
	case VSTATUS_INSUFFICIENT_PERMISSION: return "120: client has insufficient privillages";
	case VSTATUS_INVALID_CQ_HANDLE: return "126: CQ handle is invalid";
	case VSTATUS_INVALID_FORMAT: return "127: Data format is invalid";
	case VSTATUS_REJECT: return "128: Request rejected";
	// a few extra just in case
	case 129: return "129: Unknown status code";
	case 130: return "130: Unknown status code";
	case 131: return "131: Unknown status code";
	case 132: return "132: Unknown status code";
	default: return "Unknown status code";
	}
}


//
//  return attribute ID text given the ID
//
char *cs_getAidName(uint16_t aidClass, uint16_t aid) {

#define CASE_AID(aid) case aid: return #aid
#define CASE_MCLASS_AID(aid) case MCLASS_ATTRIB_ID_##aid: return #aid
#define CASE_STL_MCLASS_AID(aid) case STL_MCLASS_ATTRIB_ID_##aid: return #aid
#define CASE_STL_SA_AID(aid) case STL_SA_ATTR_##aid: return #aid
#define CASE_STL_PM_AID(aid) case STL_PM_ATTRIB_ID_##aid: return #aid
#define CASE_MAD_CV(aid) case MAD_CV_##aid: return "Class " #aid
#define CASE_PA_AID(aid) case PA_ATTRID_##aid: return #aid
#define CASE_STL_PA_AID(aid) case STL_PA_ATTRID_##aid: return #aid

	// Check common attribute ids first
	switch (aid) {
	CASE_MCLASS_AID(RESERVED_0);
	CASE_MCLASS_AID(CLASS_PORT_INFO);
	CASE_MCLASS_AID(NOTICE);
	CASE_MCLASS_AID(INFORM_INFO);
	default:
		break;
	}

	switch (aidClass) {
    case MAD_CV_SUBN_LR:
	case MAD_CV_SUBN_DR: 
        switch (aid) {
		CASE_STL_MCLASS_AID(NODE_DESCRIPTION);
		CASE_STL_MCLASS_AID(NODE_INFO);
		CASE_STL_MCLASS_AID(SWITCH_INFO);
		CASE_STL_MCLASS_AID(PORT_INFO);
		CASE_STL_MCLASS_AID(PART_TABLE);
		CASE_STL_MCLASS_AID(SL_SC_MAPPING_TABLE);
		CASE_STL_MCLASS_AID(VL_ARBITRATION);
		CASE_STL_MCLASS_AID(LINEAR_FWD_TABLE);
		CASE_STL_MCLASS_AID(MCAST_FWD_TABLE);
		CASE_STL_MCLASS_AID(SM_INFO);
		CASE_STL_MCLASS_AID(LED_INFO);
		CASE_STL_MCLASS_AID(CABLE_INFO);
		CASE_STL_MCLASS_AID(AGGREGATE);
		CASE_STL_MCLASS_AID(SC_SC_MAPPING_TABLE);
		CASE_STL_MCLASS_AID(SC_SL_MAPPING_TABLE);
		CASE_STL_MCLASS_AID(SC_VLR_MAPPING_TABLE);
		CASE_STL_MCLASS_AID(SC_VLT_MAPPING_TABLE);
		CASE_STL_MCLASS_AID(SC_VLNT_MAPPING_TABLE);
		CASE_STL_MCLASS_AID(PORT_STATE_INFO);
		CASE_STL_MCLASS_AID(PORT_GROUP_FWD_TABLE);
		CASE_STL_MCLASS_AID(PORT_GROUP_TABLE);
		CASE_STL_MCLASS_AID(BUFFER_CONTROL_TABLE);
		CASE_STL_MCLASS_AID(CONGESTION_INFO);
		CASE_STL_MCLASS_AID(SWITCH_CONGESTION_LOG);
		CASE_STL_MCLASS_AID(SWITCH_CONGESTION_SETTING);
		CASE_STL_MCLASS_AID(SWITCH_PORT_CONGESTION_SETTING);
		CASE_STL_MCLASS_AID(HFI_CONGESTION_LOG);
		CASE_STL_MCLASS_AID(HFI_CONGESTION_SETTING);
		CASE_STL_MCLASS_AID(HFI_CONGESTION_CONTROL_TABLE);

		// IB AIDs that do not conflict with STL AIDs
		CASE_MCLASS_AID(GUID_INFO);
		CASE_MCLASS_AID(VENDOR_DIAG);
		CASE_MCLASS_AID(PORT_LFT);
		CASE_MCLASS_AID(PORT_GROUP);
		CASE_MCLASS_AID(AR_LIDMASK);
		CASE_MCLASS_AID(ICS_LED_INFO);
		CASE_MCLASS_AID(COLLECTIVE_NOTICE);
		CASE_MCLASS_AID(CMLIST);
		CASE_MCLASS_AID(CFT);

		default:
			break;
		}
    case MAD_CV_VENDOR_DBSYNC:
        switch (aid) {
    	CASE_AID(DBSYNC_AID_SYNC);
    	CASE_AID(DBSYNC_AID_GROUP);
    	CASE_AID(DBSYNC_AID_SERVICE);
    	CASE_AID(DBSYNC_AID_INFORM);
    	CASE_AID(DBSYNC_AID_MCROOT);
        default:
			break;
		}
    case MAD_CV_SUBN_ADM:
        switch (aid) {
        CASE_STL_SA_AID(CLASS_PORT_INFO);
        CASE_STL_SA_AID(NOTICE);
        CASE_STL_SA_AID(INFORM_INFO);
        CASE_STL_SA_AID(NODE_RECORD);
        CASE_STL_SA_AID(PORTINFO_RECORD);
        CASE_STL_SA_AID(SC_MAPTBL_RECORD);
        CASE_STL_SA_AID(SWITCHINFO_RECORD);
        CASE_STL_SA_AID(LINEAR_FWDTBL_RECORD);
        CASE_STL_SA_AID(MCAST_FWDTBL_RECORD);
        CASE_STL_SA_AID(SMINFO_RECORD);
        CASE_STL_SA_AID(LINK_SPD_WDTH_PAIRS_RECORD);
        CASE_STL_SA_AID(LINK_RECORD);
        CASE_STL_SA_AID(SERVICE_RECORD);
        CASE_STL_SA_AID(P_KEY_TABLE_RECORD);
        CASE_STL_SA_AID(PATH_RECORD);
        CASE_STL_SA_AID(VLARBTABLE_RECORD);
        CASE_STL_SA_AID(MCMEMBER_RECORD);
        CASE_STL_SA_AID(TRACE_RECORD);
        CASE_STL_SA_AID(MULTIPATH_GID_RECORD);
        CASE_STL_SA_AID(SERVICEASSOCIATION_RECORD);
        CASE_STL_SA_AID(INFORM_INFO_RECORD);
        CASE_STL_SA_AID(SC2SL_MAPTBL_RECORD);
        CASE_STL_SA_AID(SC2VL_NT_MAPTBL_RECORD);
        CASE_STL_SA_AID(SC2VL_T_MAPTBL_RECORD);
        CASE_STL_SA_AID(SC2VL_R_MAPTBL_RECORD);
        CASE_STL_SA_AID(PGROUP_FWDTBL_RECORD);
        CASE_STL_SA_AID(MULTIPATH_GUID_RECORD);
        CASE_STL_SA_AID(MULTIPATH_LID_RECORD);
        CASE_STL_SA_AID(CABLE_INFO_RECORD);
        CASE_STL_SA_AID(VF_INFO_RECORD);
        CASE_STL_SA_AID(PORTGROUP_TABLE_RECORD);
        CASE_STL_SA_AID(BUFF_CTRL_TAB_RECORD);
        CASE_STL_SA_AID(FABRICINFO_RECORD);
        CASE_STL_SA_AID(QUARANTINED_NODE_RECORD);
        CASE_STL_SA_AID(CONGESTION_INFO_RECORD);
        CASE_STL_SA_AID(SWITCH_CONG_RECORD);
        CASE_STL_SA_AID(SWITCH_PORT_CONG_RECORD);
        CASE_STL_SA_AID(HFI_CONG_RECORD);
        CASE_STL_SA_AID(HFI_CONG_CTRL_RECORD);
		default:
			break;
        }
	case MAD_CV_PERF:
		switch (aid) {
		CASE_STL_PM_AID(PORT_STATUS);
		CASE_STL_PM_AID(CLEAR_PORT_STATUS);
		CASE_STL_PM_AID(DATA_PORT_COUNTERS);
		CASE_STL_PM_AID(ERROR_PORT_COUNTERS);
		CASE_STL_PM_AID(ERROR_INFO);
		default:
			break;
		}
    case MAD_CV_VFI_PM:
        switch (aid) {
        CASE_PA_AID(GET_GRP_LIST);
        CASE_PA_AID(GET_GRP_INFO);
        CASE_PA_AID(GET_GRP_CFG); 
        CASE_PA_AID(GET_PORT_CTRS);
        CASE_PA_AID(CLR_PORT_CTRS);
        CASE_PA_AID(CLR_ALL_PORT_CTRS);
        CASE_PA_AID(GET_PM_CONFIG);
        CASE_PA_AID(FREEZE_IMAGE);
        CASE_PA_AID(RELEASE_IMAGE);
        CASE_PA_AID(RENEW_IMAGE);
        CASE_PA_AID(GET_FOCUS_PORTS);
        CASE_PA_AID(GET_IMAGE_INFO);
        CASE_PA_AID(MOVE_FREEZE_FRAME);
        CASE_STL_PA_AID(GET_VF_LIST);
        CASE_STL_PA_AID(GET_VF_INFO);
        CASE_STL_PA_AID(GET_VF_CONFIG);
        CASE_STL_PA_AID(GET_VF_PORT_CTRS);
        CASE_STL_PA_AID(CLR_VF_PORT_CTRS);
        CASE_STL_PA_AID(GET_VF_FOCUS_PORTS);
        default:
            break;
        }
	// Unhandled Classes
	CASE_MAD_CV(BM);
	CASE_MAD_CV(DEV_MGT);
	CASE_MAD_CV(COMM_MGT);
	CASE_MAD_CV(SNMP);
	CASE_MAD_CV(VENDOR_CM);
	CASE_MAD_CV(VENDOR_LOG);
	CASE_MAD_CV(VENDOR_1);
	CASE_MAD_CV(APP_0);
	CASE_MAD_CV(APP_1);
	CASE_MAD_CV(CC);
	CASE_MAD_CV(VENDOR_FE);
	CASE_MAD_CV(VFI_BM);
    default:
        break;
    }
#undef CASE_AID
#undef CASE_MCLASS_AID
#undef CASE_STL_MCLASS_AID
#undef CASE_STL_SA_AID
#undef CASE_STL_PM_AID
#undef CASE_MAD_CV

    return "UNKNOWN";
}

//
// return method text representation given the ID
//
char *cs_getMethodText(uint8_t method) {

#define CASE_MAD_CM(aid) case MAD_CM_##aid: return #aid
#define CASE_SA_CM(aid) case SA_CM_##aid: return #aid
#define CASE_FE_MNGR(aid) case FE_MNGR_##aid: return #aid
#define CASE_FE_CMD(aid) case FE_CMD_##aid: return #aid
#define CASE_FM_CMD(aid) case FM_CMD_##aid: return #aid
	switch (method) {
	CASE_MAD_CM(GET);
	CASE_MAD_CM(SET);
	CASE_MAD_CM(GET_RESP);
	CASE_MAD_CM(SEND);
	CASE_MAD_CM(TRAP);
	CASE_MAD_CM(REPORT);
	CASE_MAD_CM(REPORT_RESP);
	CASE_MAD_CM(TRAP_REPRESS);
	CASE_MAD_CM(VIEO_REQ);
	CASE_MAD_CM(VIEO_REP);

	CASE_SA_CM(GETTABLE);
	CASE_SA_CM(GETTABLE_RESP);
	CASE_SA_CM(GETTRACETABLE);
	CASE_SA_CM(GETMULTI);
	CASE_SA_CM(GETMULTI_RESP);
	CASE_SA_CM(DELETE);
	CASE_SA_CM(DELETE_RESP);

	CASE_FE_MNGR(PROBE_CMD);
	CASE_FE_MNGR(CLOSE_CMD);
	CASE_FE_CMD(RESP);

	CASE_FM_CMD(SHUTDOWN);
	default:
		break;
	}
#undef CASE_MAD_CM
#undef CASE_SA_CM
#undef CASE_FE_MNGR
#undef CASE_FE_CMD
#undef CASE_FM_CMD
	return "UNKNOWN";
}


/* Computational Utilty Functions */

// This is the smallest switch we will see in clusters moving forward.
// These computations will be slightly high for 36 port switches, but
// should be safe and not too far off.
// Fat Tree is the most hardware intensive topology, so these will be high
// for others.  However they could be a hair low for Scalable Unit fat trees
#define MIN_SWITCH_PORTS 24

// assuming a fat tree topology with switches of size MIN_SWITCH_PORTS
// compute number of switching tiers in a typical FBB fabric
static uint32_t cs_numTiers(uint32_t subnet_size)
{
	uint32_t tiers = 1;	// minimum we care about
	while (subnet_size > MIN_SWITCH_PORTS) {
		tiers++;
		subnet_size /= (MIN_SWITCH_PORTS/2);
	}
	return tiers;
}

// assuming a fat tree topology with switches of size MIN_SWITCH_PORTS
// compute number of switches in a typical FBB fabric
uint32_t cs_numSwitches(uint32_t subnet_size)
{
	// edge switches (and intermediate tiers)
	uint32_t edge_sw = (subnet_size + (MIN_SWITCH_PORTS/2)-1)/(MIN_SWITCH_PORTS/2);
	uint32_t tiers = cs_numTiers(subnet_size);

	// core/spine is half as many switches as edge
	return (edge_sw * (tiers-1)) + (edge_sw/2);
}

// assuming a fat tree topology with switches of size MIN_SWITCH_PORTS
// compute number of node records (1 per CA port, 1 per switch)
// in a typical FBB fabric
uint32_t cs_numNodeRecords(uint32_t subnet_size)
{
		return subnet_size + cs_numSwitches(subnet_size);
}

// assuming a fat tree topology with switches of size MIN_SWITCH_PORTS
// compute number of port records (1 per CA port, 1 per switch port)
// in a typical FBB fabric
// (this will be slightly high for larger switches since there will
// be fewer switches (less switch port 0), less tiers and potentially
// fewer ISLs (hence fewer switch ports).
uint32_t cs_numPortRecords(uint32_t subnet_size)
{
	// +1 is for port 0 of switch chips
	return subnet_size + cs_numSwitches(subnet_size) * (MIN_SWITCH_PORTS+1);
}

// assuming a fat tree topology with switches of size MIN_SWITCH_PORTS
// compute number of links records (ISL and CA to Switch)
// in a typical FBB fabric
uint32_t cs_numLinkRecords(uint32_t subnet_size)
{
    // standard switching tiers calculation is a hair too low, so increase
    // the number of switching tiers to accommodate the number of links records.
	return (cs_numTiers(subnet_size) + 2)*subnet_size;
}
