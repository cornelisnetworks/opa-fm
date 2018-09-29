/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iba/ibt.h>
#include <stdarg.h>
#include "ibprint.h"
#include "ib_sd_priv.h"
#include "stl_sd.h"
#include "stl_print.h"

// display the results from a query
// Note: csv is ignored for all but OutputTypeVfInfoRecord
void PrintQueryResultValue(PrintDest_t *dest, int indent, PrintDest_t *dbgDest,
			   QUERY_RESULT_TYPE OutputType, int csv,
			   const QUERY_RESULT_VALUES *pResult)
{
	uint32 i;

	if (dbgDest) {
		PrintFunc(dbgDest, "OutputType (0x%x): %s\n", OutputType, 
				  iba_sd_query_result_type_msg(OutputType));
		PrintFunc(dbgDest, "MadStatus 0x%x: %s\n", pResult->MadStatus,
				   				iba_sd_mad_status_msg(pResult->MadStatus));
		PrintFunc(dbgDest, "%d Bytes Returned\n", pResult->ResultDataSize);
	}
	switch ((int)OutputType)
	{
	case OutputTypeSystemImageGuid:
	case OutputTypeNodeGuid:
	case OutputTypePortGuid:
	case OutputTypeStlSystemImageGuid:
	case OutputTypeStlNodeGuid:
	case OutputTypeStlPortGuid:
		{
		GUID_RESULTS *p = (GUID_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumGuids; ++i)
		{
			PrintGuid(dest, indent, p->Guids[i]);
		}
		break;
		}
	case OutputTypeClassPortInfo:
		{
		IB_CLASS_PORT_INFO_RESULTS *p = (IB_CLASS_PORT_INFO_RESULTS*)pResult->QueryResult;
		
		/* There should never be more than 1 ClassPortInfo in the results. */
		if (p->NumClassPortInfo)
			PrintClassPortInfo2(dest, indent, p->ClassPortInfo,
				StlSaClassPortInfoCapMask, StlSaClassPortInfoCapMask2);
		break;
		}
	case OutputTypeStlClassPortInfo:
		{
		STL_CLASS_PORT_INFO_RESULT *p = (STL_CLASS_PORT_INFO_RESULT*)pResult->QueryResult;
		
		/* There should never be more than 1 ClassPortInfo in the results. */
		if (p->NumClassPortInfo)
			PrintStlClassPortInfo(dest, indent, &p->ClassPortInfo, MCLASS_SUBN_ADM);
		break;
		}
	case OutputTypeStlFabricInfoRecord:
		{
		STL_FABRICINFO_RECORD_RESULT *p = (STL_FABRICINFO_RECORD_RESULT*)pResult->QueryResult;
		
		/* There should never be more than 1 FaricInfoRecord in the results. */
		if (p->NumFabricInfoRecords)
			PrintStlFabricInfoRecord(dest, indent, &p->FabricInfoRecord);
		break;
		}
	case OutputTypeStlLid:
		{
		STL_LID_RESULTS *p = (STL_LID_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumLids; ++i)
		{
			PrintStlLid(dest, indent, p->Lids[i], 0);
		}
		break;
		}
	case OutputTypeGid:
		{
		GID_RESULTS *p = (GID_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumGids; ++i)
		{
			PrintGid(dest, indent, &p->Gids[i]);
		}
		break;
		}
	case OutputTypeNodeDesc:
		{
		NODEDESC_RESULTS *p = (NODEDESC_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumDescs; ++i)
		{
			PrintNodeDesc(dest, indent, &p->NodeDescs[i]);
		}
		break;
		}
	case OutputTypeStlNodeDesc:
		{
		STL_NODEDESC_RESULTS *p = (STL_NODEDESC_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumDescs; ++i)
		{
			PrintStlNodeDesc(dest, indent, &p->NodeDescs[i], 0);
		}
		break;
		}
	case OutputTypePathRecord:
		{
		PATH_RESULTS *p = (PATH_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumPathRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintPathRecord(dest, indent, &p->PathRecords[i]);
		}
		break;
		}
	case OutputTypeNodeRecord:
		{
		NODE_RECORD_RESULTS *p = (NODE_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumNodeRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintNodeRecord(dest, indent, &p->NodeRecords[i]);
		}
		break;
		}
	case OutputTypeStlNodeRecord:
		{
		STL_NODE_RECORD_RESULTS *p = (STL_NODE_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumNodeRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlNodeRecord(dest, indent, &p->NodeRecords[i]);
		}
		break;
		}
	case OutputTypePortInfoRecord:
		{
		PORTINFO_RECORD_RESULTS *p = (PORTINFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumPortInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintPortInfoRecord(dest, indent, &p->PortInfoRecords[i]);
		}
		break;
		}
	case OutputTypeStlPortInfoRecord:
		{
		STL_PORTINFO_RECORD_RESULTS *p = (STL_PORTINFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumPortInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlPortInfoRecord(dest, indent, &p->PortInfoRecords[i]);
		}
		break;
		}
	case OutputTypeSMInfoRecord:
		{
		SMINFO_RECORD_RESULTS *p = (SMINFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumSMInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintSMInfoRecord(dest, indent, &p->SMInfoRecords[i]);
		}
		break;
		}
	case OutputTypeStlSMInfoRecord:
		{
		STL_SMINFO_RECORD_RESULTS *p = (STL_SMINFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumSMInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlSMInfo(dest, indent, &(p->SMInfoRecords[i].SMInfo), 
							p->SMInfoRecords[i].RID.LID, 0);
		}
		break;
		}
	case OutputTypeLinkRecord:
		{
		LINK_RECORD_RESULTS *p = (LINK_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumLinkRecords; ++i)
		{
			PrintLinkRecord(dest, indent, &p->LinkRecords[i]);
		}
		break;
		}
	case OutputTypeStlLinkRecord:
		{
		STL_LINK_RECORD_RESULTS *p = (STL_LINK_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumLinkRecords; ++i)
		{
			PrintStlLinkRecord(dest, indent, &p->LinkRecords[i]);
		}
		break;
		}
	case OutputTypeServiceRecord:
		{
		SERVICE_RECORD_RESULTS *p = (SERVICE_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumServiceRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintServiceRecord(dest, indent, &p->ServiceRecords[i]);
		}
		break;
		}
	case OutputTypeMcMemberRecord:
		{
		MCMEMBER_RECORD_RESULTS *p = (MCMEMBER_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumMcMemberRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintMcMemberRecord(dest, indent, &p->McMemberRecords[i]);
		}
		break;
		}
	case OutputTypeInformInfoRecord:
		{
		INFORM_INFO_RECORD_RESULTS *p = (INFORM_INFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumInformInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintInformInfoRecord(dest, indent, &p->InformInfoRecords[i]);
		}
		break;
		}
	case OutputTypeStlInformInfoRecord:
		{
		STL_INFORM_INFO_RECORD_RESULTS *p = (STL_INFORM_INFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumInformInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlInformInfoRecord(dest, indent, &p->InformInfoRecords[i]);
		}
		break;
		}
	case OutputTypeStlTraceRecord:
		{
		STL_TRACE_RECORD_RESULTS *p = (STL_TRACE_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumTraceRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlTraceRecord(dest, indent, &p->TraceRecords[i]);
		}
		break;
		}
	case OutputTypeStlSCSCTableRecord:
		{
		STL_SC_MAPPING_TABLE_RECORD_RESULTS *p = (STL_SC_MAPPING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
        uint32_t lid = p->SCSCRecords[0].RID.LID;
		for (i=0; i<p->NumSCSCTableRecords; ++i)
		{
            if (lid!=p->SCSCRecords[i].RID.LID) {
                PrintSeparator(dest);
                lid=p->SCSCRecords[i].RID.LID;
            }
			PrintStlSCSCTableRecord(dest, indent, p->SCSCRecords[i].RID_Secondary, &p->SCSCRecords[i]);
        }
		break;
		}
	case OutputTypeStlSLSCTableRecord:
		{
		STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS *p = (STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
        uint32_t lid = p->SLSCRecords[0].RID.LID;
		for (i=0; i<p->NumSLSCTableRecords; ++i)
		{
            if (lid!=p->SLSCRecords[i].RID.LID) {
                PrintSeparator(dest);
                lid=p->SLSCRecords[i].RID.LID;
            }
			PrintStlSLSCTableRecord(dest, indent, &p->SLSCRecords[i]);
        }
		break;
		}
	case OutputTypeStlSCSLTableRecord:
		{
		STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS *p = (STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		uint32_t lid = p->SCSLRecords[0].RID.LID;
		for (i=0; i<p->NumSCSLTableRecords; i++) {
			if (lid!=p->SCSLRecords[i].RID.LID) {
				PrintSeparator(dest);
				lid=p->SCSLRecords[i].RID.LID;
			}
			PrintStlSCSLTableRecord(dest, indent, &p->SCSLRecords[i]);
		}
		break;
		}
	case OutputTypeStlSCVLtTableRecord:
		{
		STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS *p = (STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		uint32_t lid = p->SCVLtRecords[0].RID.LID;
		for (i=0; i<p->NumSCVLtTableRecords; i++) {
			if (lid!=p->SCVLtRecords[i].RID.LID) {
				PrintSeparator(dest);
				lid=p->SCVLtRecords[i].RID.LID;
			}
			PrintStlSCVLxTableRecord(dest, indent, &p->SCVLtRecords[i], STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE);
		}
		break;
		}
	case OutputTypeStlSCVLntTableRecord:
		{
		STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS *p = (STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		uint32_t lid=p->SCVLntRecords[0].RID.LID;
		for (i=0; i<p->NumSCVLntTableRecords; i++) {
			if (lid!=p->SCVLntRecords[i].RID.LID) {
				PrintSeparator(dest);
				lid=p->SCVLntRecords[i].RID.LID;
			}
			PrintStlSCVLxTableRecord(dest, indent, &p->SCVLntRecords[i], STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE);
		}
		break;
		}
	case OutputTypeStlSCVLrTableRecord:
		{
		STL_SC2PVL_R_MAPPING_TABLE_RECORD_RESULTS *p = (STL_SC2PVL_R_MAPPING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		uint32_t lid=p->SCVLrRecords[0].RID.LID;
		for (i=0; i<p->NumSCVLrTableRecords; i++) {
			if (lid!=p->SCVLrRecords[i].RID.LID) {
				PrintSeparator(dest);
				lid=p->SCVLrRecords[i].RID.LID;
			}
			PrintStlSCVLxTableRecord(dest, indent, &p->SCVLrRecords[i], STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE);
		}
		break;
		}
	case OutputTypeSwitchInfoRecord:
		{
		SWITCHINFO_RECORD_RESULTS *p = (SWITCHINFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumSwitchInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintSwitchInfoRecord(dest, indent, &p->SwitchInfoRecords[i]);
		}
		break;
		}
	case OutputTypeStlSwitchInfoRecord:
		{
		STL_SWITCHINFO_RECORD_RESULTS *p = (STL_SWITCHINFO_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumSwitchInfoRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlSwitchInfoRecord(dest, indent, &p->SwitchInfoRecords[i]);
		}
		break;
		}
	case OutputTypeStlLinearFDBRecord:
		{
		STL_LINEAR_FDB_RECORD_RESULTS *p = (STL_LINEAR_FDB_RECORD_RESULTS*)pResult->QueryResult;
		
		for (i=0; i<p->NumLinearFDBRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlLinearFDBRecord(dest, indent, &p->LinearFDBRecords[i]);
		}
		break;
		}
	case OutputTypeRandomFDBRecord:
		{
		RANDOM_FDB_RECORD_RESULTS *p = (RANDOM_FDB_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumRandomFDBRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintRandomFDBRecord(dest, indent, &p->RandomFDBRecords[i]);
		}
		break;
		}
	case OutputTypeStlMCastFDBRecord:
		{
		STL_MCAST_FDB_RECORD_RESULTS *p = (STL_MCAST_FDB_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumMCastFDBRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlMCastFDBRecord(dest, indent, &p->MCastFDBRecords[i]);
		}
		break;
		}
	case OutputTypeStlVLArbTableRecord:
		{
		STL_VLARBTABLE_RECORD_RESULTS *p = (STL_VLARBTABLE_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumVLArbTableRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlVLArbTableRecord(dest, indent, &p->VLArbTableRecords[i]);
		}
		break;
		}
	case OutputTypeStlPKeyTableRecord:
		{
		STL_PKEYTABLE_RECORD_RESULTS *p = (STL_PKEYTABLE_RECORD_RESULTS*)pResult->QueryResult;
		for (i=0; i<p->NumPKeyTableRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlPKeyTableRecord(dest, indent, &p->PKeyTableRecords[i]);
		}
		break;
		}
	case OutputTypeStlVfInfoRecord:
		{
		STL_VFINFO_RECORD_RESULTS *p = (STL_VFINFO_RECORD_RESULTS*)pResult->QueryResult;

		for (i=0; i<p->NumVfInfoRecords; ++i) {
			if (csv == 1) {
				PrintStlVfInfoRecordCSV(dest, indent, &p->VfInfoRecords[i]);
			} else if (csv == 2) {
				PrintStlVfInfoRecordCSV2(dest, indent, &p->VfInfoRecords[i]);
			} else {
				if (i) PrintSeparator(dest);
				PrintStlVfInfoRecord(dest, indent, &p->VfInfoRecords[i]);
			}
		}
		break;
		}
	case OutputTypeStlQuarantinedNodeRecord:
		{
		STL_QUARANTINED_NODE_RECORD_RESULTS *p = (STL_QUARANTINED_NODE_RECORD_RESULTS*)pResult->QueryResult;
		int hasPrinted = 0;
		// Loop through and print out spoofing violations first
		PrintSeparator(dest);
		PrintFunc(dest, "%*sNodes Quarantined Due To Spoofing:\n", indent, "");
		PrintSeparator(dest);
		for(i = 0; i < p->NumQuarantinedNodeRecords; ++i)
		{
			if (i && hasPrinted) PrintSeparator(dest);

			if(p->QuarantinedNodeRecords[i].quarantineReasons & STL_QUARANTINE_REASON_SPOOF_GENERIC) {
				PrintQuarantinedNodeRecord(dest, indent, &p->QuarantinedNodeRecords[i]);
				hasPrinted = 1;
			}
		}

		if(!hasPrinted) {
			PrintFunc(dest, "%*s    None.\n\n", indent, "");
		} else {
			hasPrinted = 0;
		}

		// Then loop through and print out all other violations
		PrintSeparator(dest);
		PrintFunc(dest, "%*sNodes Quarantined Due To Other Violations:\n", indent, "");
		PrintSeparator(dest);
		for(i = 0; i < p->NumQuarantinedNodeRecords; ++i)
		{
			if (i && hasPrinted) PrintSeparator(dest);
			if(!(p->QuarantinedNodeRecords[i].quarantineReasons & STL_QUARANTINE_REASON_SPOOF_GENERIC)) {
				PrintQuarantinedNodeRecord(dest, indent, &p->QuarantinedNodeRecords[i]);
				hasPrinted = 1;
			}
		}

		if(!hasPrinted) {
			PrintFunc(dest, "%*s    None.\n\n", indent, "");
		}

		break;
		}
	case OutputTypeStlCongInfoRecord:
		{
		STL_CONGESTION_INFO_RECORD_RESULTS *p = (STL_CONGESTION_INFO_RECORD_RESULTS*)pResult->QueryResult;
		for(i = 0; i < p->NumRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintStlCongestionInfoRecord(dest, indent, &p->Records[i]);
		}
		break;
		}
	case OutputTypeStlSwitchCongRecord:
		{
        STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS *p = (STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS*)pResult->QueryResult;
        for(i = 0; i < p->NumRecords; ++i)
        {
            if (i) PrintSeparator(dest);
            PrintStlSwitchCongestionSettingRecord(dest, indent, &p->Records[i]);
        }
		break;
		}
	case OutputTypeStlSwitchPortCongRecord:
		{
        STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS *p = (STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS*)pResult->QueryResult;
        for(i = 0; i < p->NumRecords; ++i)
        {
            if (i) PrintSeparator(dest);
            PrintStlSwitchPortCongestionSettingRecord(dest, indent, &p->Records[i]);
        }
		break;
		}
	case OutputTypeStlHFICongRecord:
		{
        STL_HFI_CONGESTION_SETTING_RECORD_RESULTS *p = (STL_HFI_CONGESTION_SETTING_RECORD_RESULTS*)pResult->QueryResult;
        for(i = 0; i < p->NumRecords; ++i)
        {
            if (i) PrintSeparator(dest);
            PrintStlHfiCongestionSettingRecord(dest, indent, &p->Records[i]);
        }
		break;
		}
	case OutputTypeStlHFICongCtrlRecord:
		{
        STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS *p = (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS*)pResult->QueryResult;
        for(i = 0; i < p->NumRecords; ++i)
        {
            if (i) PrintSeparator(dest);
            PrintStlHfiCongestionControlTabRecord(dest, indent, &p->Records[i]);
        }
		break;
		}
	case OutputTypeStlBufCtrlTabRecord:
		{
		STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS *p = (STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		for(i = 0; i < p->NumBufferControlRecords; ++i)
		{
			if (i) PrintSeparator(dest);
			PrintBfrCtlTableRecord(dest, indent, &p->BufferControlRecords[i]);
		}
		break;
		}
	case OutputTypeStlCableInfoRecord:
		{
		const uint32	max_records = 2;	// Max number of grouped records
											//  NOTE that CIR.Length is 7 bits (<= 127)
		STL_CABLE_INFO_RECORD_RESULTS *p = (STL_CABLE_INFO_RECORD_RESULTS*)pResult->QueryResult;
		STL_CABLE_INFO_RECORD *p_rec;
		uint32	ct_records = 0;			// Count of grouped records
		uint8	first_addr = 0;			// First CableInfo addr in bf_cable_info
		uint8	last_addr = 0;			// Last CableInfo addr in bf_cable_info
		uint32	prev_lid;
		uint8	prev_port;
		uint8	prev_addr;
		uint8	bf_cable_info[sizeof(STL_CABLE_INFO_RECORD) * max_records];
		for(i = 0; i < p->NumCableInfoRecords; ++i)
		{
			p_rec = &p->CableInfoRecords[i];
			// Skip unaligned or partial records
			if ( ( (p_rec->u1.s.Address != STL_CIB_STD_HIGH_PAGE_ADDR) &&
					(p_rec->u1.s.Address != STL_CIB_STD_HIGH_PAGE_ADDR + STL_CABLE_INFO_MAXLEN + 1) ) ||
					(p_rec->Length != STL_CABLE_INFO_MAXLEN) )
				continue;
			// Group multiple records for same LID and port into one
			if (! ct_records) {
				// First record for LID/Port
				memset(bf_cable_info, 0, sizeof(bf_cable_info));
				memcpy(bf_cable_info, p_rec, sizeof(STL_CABLE_INFO_RECORD));
				last_addr = 0;
			}
			else {
				if ((p_rec->LID == prev_lid) && (p_rec->Port == prev_port) &&
						(p_rec->u1.s.Address != prev_addr) && (ct_records < max_records)) {
					// Subsequent record for same LID/Port
					if (p_rec->u1.s.Address < first_addr) {
						memmove( ((STL_CABLE_INFO_RECORD *)bf_cable_info)->Data + first_addr - p_rec->u1.s.Address,
							((STL_CABLE_INFO_RECORD *)bf_cable_info)->Data,
							STL_CIR_DATA_SIZE * max_records - (first_addr - p_rec->u1.s.Address) );
						memcpy(bf_cable_info, p_rec, sizeof(STL_CABLE_INFO_RECORD));
					}
					else {
						memcpy( ((STL_CABLE_INFO_RECORD *)bf_cable_info)->Data + p_rec->u1.s.Address - first_addr,
							p_rec->Data, STL_CIR_DATA_SIZE );
					}
				}
				else {
					// Print grouped record
					if (i - ct_records) PrintSeparator(dest);
					PrintStlCableInfoRecord(dest, indent, (STL_CABLE_INFO_RECORD *)bf_cable_info);
					memset(bf_cable_info, 0, sizeof(bf_cable_info));
					memcpy(bf_cable_info, p_rec, sizeof(STL_CABLE_INFO_RECORD));
					last_addr = 0;
					ct_records = 0;
				}
			}
			prev_lid = p_rec->LID;
			prev_port = p_rec->Port;
			prev_addr = p_rec->u1.s.Address;
			if (prev_addr > last_addr)
				last_addr = prev_addr;
			ct_records += 1;
			first_addr = ((STL_CABLE_INFO_RECORD *)bf_cable_info)->u1.s.Address;
			((STL_CABLE_INFO_RECORD *)bf_cable_info)->Length = last_addr - first_addr + STL_CIR_DATA_SIZE - 1;
		}
		if (ct_records) {
			if (i - ct_records) PrintSeparator(dest);
			PrintStlCableInfoRecord(dest, indent, (STL_CABLE_INFO_RECORD *)bf_cable_info);
		}
		break;
		}
	case OutputTypeStlPortGroupRecord:
		{
		        STL_PORT_GROUP_TABLE_RECORD_RESULTS *p = (STL_PORT_GROUP_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		        for(i = 0; i < p->NumRecords; ++i)
		        {
		            if (i) PrintSeparator(dest);
		            PrintStlPortGroupTabRecord(dest, indent, &p->Records[i]);
		        }
			break;
		}
	case OutputTypeStlPortGroupFwdRecord:
		{
		        STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS *p = (STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS*)pResult->QueryResult;
		        for(i = 0; i < p->NumRecords; ++i)
		        {
		            if (i) PrintSeparator(dest);
		            PrintStlPortGroupFwdTabRecord(dest, indent, &p->Records[i]);
		        }
			break;
		}

	case OutputTypeStlDeviceGroupMemberRecord:
		{
			STL_DEVICE_GROUP_MEMBER_RECORD_RESULTS *p = (STL_DEVICE_GROUP_MEMBER_RECORD_RESULTS*)pResult->QueryResult;
			for (i=0; i < p->NumRecords; ++i)
			{
				PrintStlDeviceGroupMemberRecord(dest, indent, &p->Records[i]);
			}
			break;
		}

	case OutputTypeStlDeviceGroupNameRecord:
		{
			STL_DEVICE_GROUP_NAME_RECORD_RESULTS *p = (STL_DEVICE_GROUP_NAME_RECORD_RESULTS*)pResult->QueryResult;
			for (i=0; i < p->NumRecords; ++i)
			{
				PrintStlDeviceGroupNameRecord(dest, indent, &p->Records[i], i+1);
			}
			break;
		}

	case OutputTypeStlDeviceTreeMemberRecord:
		{
			STL_DEVICE_TREE_MEMBER_RECORD_RESULTS *p = (STL_DEVICE_TREE_MEMBER_RECORD_RESULTS*)pResult->QueryResult;
			for (i=0; i < p->NumRecords; ++i)
			{
				PrintStlDeviceTreeMemberRecord(dest, indent, &p->Records[i]);
			}
			break;
		}
	case OutputTypeStlSwitchCostRecord:
		{
			STL_SWITCH_COST_RECORD_RESULTS *p = (STL_SWITCH_COST_RECORD_RESULTS*)pResult->QueryResult;
			for(i = 0; i < p ->NumRecords; ++i)
			{
				if (i) PrintSeparator(dest);
				PrintStlSwitchCostRecord(dest, indent, &p->Records[i]);
			}
		}
		break;
	default:
		PrintFunc(dest, "Unsupported OutputType\n");
	}
}

// display the results from a query
// returns potential exit status:
// 	0 - success
// 	1 - query failed
// 	2 - query failed due to invalid query
// Note: csv is ignored for all but OutputTypeVfInfoRecord
int PrintQueryResult(PrintDest_t *dest, int indent, PrintDest_t *dbgDest,
			   	QUERY_RESULT_TYPE OutputType, int csv, FSTATUS status,
				QUERY_RESULT_VALUES *pResult)
{
	int ret = 0;

	if (! pResult || status == FINVALID_PARAMETER)
	{
		PrintFunc(dest, "%*sFailed: %s\n", indent, "", iba_fstatus_msg(status));
		// for FINVALID_PARAMETER, use special exit status
		// will trigger usage output by main
		ret = (status == FINVALID_PARAMETER)?2:1;
	} else if (pResult->Status != FSUCCESS || pResult->MadStatus!= MAD_STATUS_SUCCESS) {
		PrintFunc(dest, "%*sFailed: %s MadStatus 0x%x: %s\n", indent, "",
				iba_fstatus_msg(pResult->Status),
			   	pResult->MadStatus, iba_sd_mad_status_msg(pResult->MadStatus));
		ret = 1;
	} else if (pResult->ResultDataSize == 0) {
		PrintFunc(dest, "%*sNo Records Returned\n", indent, "");
	} else {
		PrintQueryResultValue(dest, indent, dbgDest, OutputType, csv, pResult);
	}
	return (ret);
}
