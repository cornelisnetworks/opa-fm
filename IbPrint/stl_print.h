/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef __STL_PRINT_H__
#define __STL_PRINT_H__

#include "iba/stl_mad_priv.h"
#include "iba/stl_sm_priv.h"
#include "iba/stl_sa_priv.h"
#include "iba/stl_sd.h"
#include "iba/stl_pm.h"
#include "ibprint.h"

#if defined (__cplusplus)
extern "C" {
#endif

// Note: The functions in ibprint.h should be used, unless there is a STL-specific diffference,
// in which case add a print here.  There are two groupings for these prints:
// First grouping are those used for debug/output primarily of SM/SMA interactions.
// Second grouping is for RECORD prints in support of customer-visible iba_* prints of SA info.

// Usage defines
#define CABLEINFO_DETAIL_ONELINE  0			// One-line output
#define CABLEINFO_DETAIL_SUMMARY  1			// Multi-line but abbreviated output
#define CABLEINFO_DETAIL_BRIEF    2			// Brief output
#define CABLEINFO_DETAIL_VERBOSE  3			// Verbose output
#define CABLEINFO_DETAIL_ALL      4			// All fields output
#define RCVERRORINFO1		  1
#define RCVERRORINFO4		  4
#define RCVERRORINFO12		  12
#define RCVERRORINFO13		  13


// verbose level of print content
int PrintStlVerboseSet(int level);

// Basic STL prints
void PrintStlLid(PrintDest_t *dest, int indent, STL_LID lid, int printLineByLine);

// General classPortInfo
void PrintStlClassPortInfo(PrintDest_t *dest, int indent, const STL_CLASS_PORT_INFO *pClassPortInfo, uint8 MgmtClass);

// SM/SMA
void PrintStlLedInfo(PrintDest_t *dest, int indent, const STL_LED_INFO *pLedInfo, EUI64 portGuid, int printLineByLine);
void PrintStlSmpHeader(PrintDest_t *dest, int indent, const STL_SMP *smp);
void PrintStlAggregateMember(PrintDest_t *dest, int indent, 
	const STL_AGGREGATE *aggr);
void PrintStlNodeInfo(PrintDest_t *dest, int indent, const STL_NODE_INFO *pNodeInfo, int printLineByLine);
void PrintStlNodeDesc(PrintDest_t *dest, int indent, const STL_NODE_DESCRIPTION *pStlNodeDesc, int printLineByLine);
void PrintStlPortInfo(PrintDest_t *dest, int indent, const STL_PORT_INFO *pPortInfo, EUI64 portGuid, int PrintLineByLine);

void PrintStlSwitchInfo(PrintDest_t *dest, int indent, const STL_SWITCH_INFO *pSwitchInfo, STL_LID lid, int PrintLineByLine);
void PrintStlPKeyTable(PrintDest_t *dest, int indent, const STL_PARTITION_TABLE *pPKeyTable, uint16 blockNum, int printLineByLine);
void PrintStlPortInfoSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, EUI64 portGuid, int printLineByLine);


void PrintStlPKeyTableSmp(PrintDest_t *dest, int indent, const STL_SMP *smp,
			  NODE_TYPE nodetype, boolean showHeading, boolean showBlock, int printLineByLine);
void PrintStlSLSCMap(PrintDest_t *dest, int indent, const char* prefix,
					const STL_SLSCMAP *pSLSCMap, boolean heading);
void PrintStlSLSCMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp,
					int printLineByLine);
void PrintStlSCSCMap(PrintDest_t *dest, int indent, const char* prefix,
					const STL_SCSCMAP *pSCSCMap, boolean heading);
void PrintStlSCSCMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp,
					boolean heading, int printLineByLine);
void PrintStlSCSLMap(PrintDest_t *dest, int indent, const char *prefix,
					const STL_SCSLMAP *pSCSLMap, boolean heading);
void PrintStlSCSLMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp,
					int printLineByLine);
void PrintStlSCVLxMap(PrintDest_t *dest, int indent, const char* prefix,
			const STL_SCVLMAP *pSCVLMap, boolean heading,
			uint16_t attribute, int printLineByLine);
void PrintStlSCVLxMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp,
			NODE_TYPE nodetype, boolean heading, int printLineByLine);
		/* NOTE: smp defines Attribute printed via AttributeID */
void PrintStlLinearFDB(PrintDest_t *dest, int indent, const STL_LINEAR_FORWARDING_TABLE *pLinearFDB, uint32 blockNum, int printLineByLine);


void PrintStlVLArbTable(PrintDest_t *dest, int indent, const STL_VLARB_TABLE *pVLArbTable, uint32 blockNum, int printLineByLine);
void PrintStlVLArbTableSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t nodeType, int printLineByLine);
void PrintStlMCastSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t pos, uint32_t startBlk, uint8_t count, int printLineByLine);
void PrintStlMCastFDB(PrintDest_t *dest, int indent, const STL_MULTICAST_FORWARDING_TABLE *pMCastFDB, uint32 blockNum, int printLineByLine);
void PrintStlInformInfo(PrintDest_t *dest, int indent, const STL_INFORM_INFO *pInformInfo);
void PrintStlCongestionInfo(PrintDest_t *dest, int indent, const STL_CONGESTION_INFO *pCongestionInfo, int printLineByLine);
void PrintStlSwitchCongestionSetting(PrintDest_t *dest, int indent, const STL_SWITCH_CONGESTION_SETTING *pSwCongestionSetting, int printLineByLine);
void PrintStlHfiCongestionSetting(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_SETTING *pHfiCongestionSetting, int printLineByLine);
void PrintStlSwitchPortCongestionSettingElement(PrintDest_t *dest, int indent, const STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT *pElement, uint8_t index, int printLineByLine);
void PrintStlSwitchPortCongestionSettingSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, int printLineByLine);
void PrintStlSwitchCongestionLog(PrintDest_t *dest, int indent, const STL_SWITCH_CONGESTION_LOG *pSwCongestionLog, int printLineByLine);
void PrintStlHfiCongestionLog(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_LOG *pHfiCongestionLog, int printLineByLine);
void PrintStlHfiCongestionControlTab(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_CONTROL_TABLE *pHfiCongestionControl, uint8_t cnt, uint8_t start, int printLineByLine);
void PrintStlBfrCtlTable(PrintDest_t *dest, int indent, const STL_BUFFER_CONTROL_TABLE *pBfrCtlTable, int printLineByLine);
void PrintStlBfrCtlTableSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t nodeType, int printLineByLine);
void PrintStlCableInfoLowPage(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint8_t detail, int printLineByLine);
void PrintStlCableInfoHighPage(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint8_t portType, uint8_t detail, int printLineByLine);
void PrintStlCableInfoHighPage0DD(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint8_t portType, uint8_t detail, int printLineByLine);
void PrintStlCableInfoDump(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint16_t addr, uint8_t len, int printLineByLine);
// len is real length of cable data
void PrintStlCableInfo(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint16_t addr, uint8_t len, uint8_t portType, uint8_t detail, int printLineByLine, boolean hexOnly);
void PrintStlCableInfoSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t portType, int printLineByLine);
void PrintStlPortGroupFDB(PrintDest_t *dest, int indent, const STL_PORT_GROUP_FORWARDING_TABLE *pPortGroupFDB, uint32 blockNum, int printLineByLine);
void PrintStlPortGroupTable(PrintDest_t *dest, int indent, const uint64_t *pPortGroup, uint32 blockNum, int position, int printLineByLine);
// SA/ iba_* customer-visible user commands
// SA one print function for each SA query OutputType
void PrintStlNodeRecord(PrintDest_t *dest, int indent, const STL_NODE_RECORD *pNodeRecord);
void PrintStlPortInfoRecord(PrintDest_t *dest, int indent, const STL_PORTINFO_RECORD *pPortInfoRecord);
void PrintStlPortStateInfo(PrintDest_t *dest, int indent, const STL_PORT_STATE_INFO *psip, uint16 portCount, uint16 startPort, int printLineByLine);
void PrintStlSwitchInfoRecord(PrintDest_t *dest, int indent, const STL_SWITCHINFO_RECORD *pSwitchInfoRecord);
void PrintStlPKeyTableRecord(PrintDest_t *dest, int indent, const STL_P_KEY_TABLE_RECORD *pPKeyTableRecord);
void PrintStlSCSCTableRecord(PrintDest_t *dest, int indent, int extended, const STL_SC_MAPPING_TABLE_RECORD *pSCSCMapRecord);
void PrintStlSLSCTableRecord(PrintDest_t *dest, int indent, const STL_SL2SC_MAPPING_TABLE_RECORD *pSCSLMapRecord);
void PrintStlSCSLTableRecord(PrintDest_t *dest, int indent, const STL_SC2SL_MAPPING_TABLE_RECORD *pSCSLMapRecord);
void PrintStlSCVLxTableRecord(PrintDest_t *dest, int indent, const STL_SC2VL_R_MAPPING_TABLE_RECORD *pSCVLxMapRecord, uint16_t attribute);
void PrintStlSMInfo(PrintDest_t *dest, int indent, const STL_SM_INFO *pSMInfo, STL_LID lid, int printLineByLine);
void PrintStlLinearFDBRecord(PrintDest_t *dest, int indent, const STL_LINEAR_FORWARDING_TABLE_RECORD *pLinearFDBRecord);
void PrintStlVLArbTableRecord(PrintDest_t *dest, int indent, const STL_VLARBTABLE_RECORD *pVLArbTableRecord);
void PrintStlMcMemberRecord(PrintDest_t *dest, int indent, const STL_MCMEMBER_RECORD *pMcMemberRecord);
void PrintStlMCastFDBRecord(PrintDest_t *dest, int indent, const STL_MULTICAST_FORWARDING_TABLE_RECORD *pMCastFDBRecord);
void PrintStlLinkRecord(PrintDest_t *dest, int indent, const STL_LINK_RECORD *pLinkRecord);
void PrintStlServiceRecord(PrintDest_t *dest, int indent, const STL_SERVICE_RECORD *pServiceRecord);
void PrintStlInformInfoRecord(PrintDest_t * dest, int indent, const STL_INFORM_INFO_RECORD * pInformInfoRecord);
void PrintStlVfInfoRecord_detail(PrintDest_t *dest, int indent, int detail, const STL_VFINFO_RECORD *pVfInfo, int showQueryParams);
void PrintStlVfInfoRecord(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo);
void PrintStlVfInfoRecordCSV(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo);
void PrintStlVfInfoRecordCSV2(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo);
void PrintStlTraceRecord(PrintDest_t *dest, int indent, const STL_TRACE_RECORD *pTraceRecord);
void PrintStlFabricInfoRecord(PrintDest_t *dest, int indent, const STL_FABRICINFO_RECORD *pFabricInfoRecord);
void PrintQuarantinedNodeRecord(PrintDest_t *dest, int indent, const STL_QUARANTINED_NODE_RECORD *pQuarantinedNodeRecord);
void PrintStlCongestionInfoRecord(PrintDest_t *dest, int indent, const STL_CONGESTION_INFO_RECORD *pCongestionInfo);
void PrintStlSwitchCongestionSettingRecord(PrintDest_t *dest, int indent, const STL_SWITCH_CONGESTION_SETTING_RECORD *pSwCongestionSetting);
void PrintStlSwitchPortCongestionSettingRecord(PrintDest_t *dest, int indent, const STL_SWITCH_PORT_CONGESTION_SETTING_RECORD *pSwPortCS);
void PrintStlHfiCongestionSettingRecord(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_SETTING_RECORD *pHfiCongestionSetting);
void PrintStlHfiCongestionControlTabRecord(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_CONTROL_TABLE_RECORD *pHfiCongestionControl);
void PrintBfrCtlTableRecord(PrintDest_t *dest, int indent, const STL_BUFFER_CONTROL_TABLE_RECORD *pBfrCtlTable);
void PrintStlCableInfoRecord(PrintDest_t *dest, int indent, const STL_CABLE_INFO_RECORD *pCableInfoRecord);
void PrintStlPortGroupTabRecord(PrintDest_t *dest, int indent, const STL_PORT_GROUP_TABLE_RECORD *pRecord);
void PrintStlPortGroupFwdTabRecord(PrintDest_t *dest, int indent, const STL_PORT_GROUP_FORWARDING_TABLE_RECORD *pRecord);
void PrintStlDeviceGroupMemberRecord(PrintDest_t *dest, int indent, const STL_DEVICE_GROUP_MEMBER_RECORD *pRecord);
void PrintStlDeviceGroupNameRecord(PrintDest_t *dest, int indent, const STL_DEVICE_GROUP_NAME_RECORD *pRecord, int record_no);
void PrintStlDeviceTreeMemberRecord(PrintDest_t *dest, int indent, const STL_DEVICE_TREE_MEMBER_RECORD *pRecord);
void PrintStlSwitchCostRecord(PrintDest_t *dest, int indent, const STL_SWITCH_COST_RECORD *pRecord);
// len is real length of cable data
void PrintStlPortSummary(PrintDest_t *dest, int indent, const char* portName, const STL_PORT_INFO *pPortInfo, EUI64 portGuid, uint16_t g_pkey, const uint8_t *cableInfoData, uint16_t addr, uint8_t len, const STL_PORT_STATUS_RSP *pPortStatusRsp, uint8_t detail, int printLineByLine);

// PM
void PrintStlPortStatusRsp(PrintDest_t *dest, int indent, const STL_PORT_STATUS_RSP *pStlPortStatusRsp);
void PrintStlPortStatusRspSummary(PrintDest_t *dest, int indent, const STL_PORT_STATUS_RSP *pStlPortStatusRsp, int printLineByLine);
void PrintStlClearPortStatus(PrintDest_t *dest, int indent, const STL_CLEAR_PORT_STATUS *pStlClearPortStatus);
void PrintStlDataPortCountersRsp(PrintDest_t *dest, int indent, const STL_DATA_PORT_COUNTERS_RSP *pStlDataPortCountersRsp);
void PrintStlErrorPortCountersRsp(PrintDest_t *dest, int indent, const STL_ERROR_PORT_COUNTERS_RSP *pStlErrorPortCountersRsp);
void PrintStlErrorInfoRsp(PrintDest_t *dest, int indent, const STL_ERROR_INFO_RSP *pStlErrorInfoRsp, int headerOnly);


// PA
void PrintStlPAGroupList(PrintDest_t *dest, int indent, const int numRecords, const STL_PA_GROUP_LIST *pGroupList);
void PrintStlPAGroupList2(PrintDest_t *dest, int indent, const int numRecords, const STL_PA_GROUP_LIST2 *pGroupList);
void PrintStlPAGroupInfo(PrintDest_t *dest, int indent, const STL_PA_PM_GROUP_INFO_DATA *pGroupInfo);
void PrintStlPAGroupConfig(PrintDest_t *dest, int indent, const char *groupName, const int numRecords, const STL_PA_PM_GROUP_CFG_RSP *pGroupConfig);
void PrintStlPAGroupNodeInfo(PrintDest_t *dest, int indent, const char *groupName, const int numRecords, const STL_PA_GROUP_NODEINFO_RSP *pGroupNodeInfo);
void PrintStlPAGroupLinkInfo(PrintDest_t *dest, int indent, const char *groupName, const int numRecords, const STL_PA_GROUP_LINKINFO_RSP *pGroupLinkInfo);
void PrintStlPAPortCounters(PrintDest_t *dest, int indent, const STL_PORT_COUNTERS_DATA *pPortCounters, const STL_LID nodeLid, const uint32 portNumber, const uint32 flags);
void PrintStlPMConfig(PrintDest_t *dest, int indent, const STL_PA_PM_CFG_DATA *pPMConfig);
void PrintStlPAImageId(PrintDest_t *dest, int indent, const STL_PA_IMAGE_ID_DATA *pPAImageID);
void PrintStlPAFocusPorts(PrintDest_t *dest, int indent, const char *groupName, const int numRecords, const uint32 select, const uint32 start, const uint32 range,
	const STL_FOCUS_PORTS_RSP *pFocusPorts);
void PrintStlPAFocusPortsMultiSelect(PrintDest_t *dest, int indent, const char *groupName, const int numRecords, const uint32 start, const uint32 range,
	const STL_FOCUS_PORTS_MULTISELECT_RSP *pFocusPorts, uint8 logical_operator, STL_FOCUS_PORT_TUPLE *tuple);
void PrintStlPAImageInfo(PrintDest_t *dest, int indent, const STL_PA_IMAGE_INFO_DATA *pImageInfo);
void PrintStlPAMoveFreeze(PrintDest_t *dest, int indent, const STL_MOVE_FREEZE_DATA *pMoveFreeze);
void PrintStlPAVFList(PrintDest_t *dest, int indent, const int numRecords, const STL_PA_VF_LIST *pVFList);
void PrintStlPAVFList2(PrintDest_t *dest, int indent, const int numRecords, const STL_PA_VF_LIST2 *pVFList);
void PrintStlPAVFInfo(PrintDest_t *dest, int indent, const STL_PA_VF_INFO_DATA *pVFInfo);
void PrintStlPAVFConfig(PrintDest_t *dest, int indent, const char *vfName, const int numRecords, const STL_PA_VF_CFG_RSP *pVFConfig);
void PrintStlPAVFPortCounters(PrintDest_t *dest, int indent, const STL_PA_VF_PORT_COUNTERS_DATA *pVFPortCounters, const STL_LID nodeLid, const uint32 portNumber, const uint32 flags);
void PrintStlPAVFFocusPorts(PrintDest_t *dest, int indent, const char *vfName, const int numRecords, const uint32 select, const uint32 start, const uint32 range,
	const STL_PA_VF_FOCUS_PORTS_RSP *pVFFocusPorts);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Print___WithDots for -g option printing fo iba_[sp]maquery
void PrintIntWithDots(PrintDest_t *dest, int indent, const char * name, uint64_t value);
void PrintIntWithDotsFull(PrintDest_t *dest, int indent, const char * name, uint64_t value);
void PrintIntWithDotsDec(PrintDest_t *dest, int indent, const char * name, uint64_t value);
void PrintStrWithDots(PrintDest_t *dest, int indent, const char * name, const char * value);

#if defined (__cplusplus)
}
#endif


static __inline const char*
FMConfigErrorInfoToText(uint8 errorInfo)
{
	switch (errorInfo) {
	case 0: return "BadHeadDist";
	case 1: return "BadTailDist";
	case 2: return "BadCtrlDist";
	case 3: return "BadCrdtAck";
	case 4: return "UnsupportedVLMarker";
	case 5: return "BadPreempt";
	case 6: return "BadControlFlit";
	case 7: return "ExceedMulticastLimit";
	case 8: return "BadMarkerDist";
	default:
		return "Unknown";
	}
}

static __inline const char*
PortRcvErrorInfoToText(uint8 errorInfo)
{
        switch (errorInfo)
        {
                case 0:
                        return "reserved";
                case 1:
                        return "BadPktLen";
                case 2:
                        return "PktLenTooLong";
                case 3:
                        return "PktLenTooShort";
                case 4:
                        return "BadSLID";
                case 5:
                        return "BadDLID";
                case 6:
                        return "BadL2";
                case 7:
                        return "BadSC";
                case 8:
                        return "reserved";
                case 9:
                        return "Headless";
                case 10:
                        return "reserved";
                case 11:
                        return "PreemptError";
                case 12:
                        return "PreemptVL15";
                case 13:
                        return "BadVLMarker";
                default:
                        return "Unknown"; 
        }
}

static __inline const char*
PortRcvSwitchRelayErrorInfoToText(uint8 errorInfo)
{
	switch (errorInfo) {
	case 0: return "BadDLIDRange";
	case 1: return "reserved";
	case 2: return "BadEgress";
	case 3: return "BadSC2SC";
	default:
		return "Unknown";
	}
}

static __inline const char*
PortRcvConstraintErrorInfoToText(uint8 errorInfo)
{
        switch (errorInfo)
        {
                case 0:
                        return "reserved";
                case 1:
                        return "pkeyViolation";
                case 2:
                        return "BadSLIDRange";
                case 3:
                        return "SWP0pkeyCheck";
                default:
                        return "Unknown"; 
        }
}
static __inline const char*
UncorrectableErrorInfoToText(uint8 errorInfo)
{
        switch (errorInfo)
        {
                case 0:
                        return "BadHead";
                case 1:
                        return "BadBody";
                case 2:
                        return "BadTail";
                case 3:
                        return "BadLFCmd";
                case 4:
                        return "InternalError";
                default:
                        return "Unknown"; 
        }
}


#endif /* __STL_PRINT_H__ */
