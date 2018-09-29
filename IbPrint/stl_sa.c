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
#include <ctype.h>
#include <time.h>
#include <iba/ibt.h>
#include <stdarg.h>
#include "stl_print.h"
#include <iba/stl_helper.h>
#include <iba/stl_sa_priv.h>

#define SIZE_TIME 256   // used in buffer to generate time string for output

void
PrintStlNodeRecord(PrintDest_t * dest, int indent,
				   const STL_NODE_RECORD * pNodeRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x Type: %s  Name: %.*s\n",
			  indent, "", pNodeRecord->RID.LID,
			  StlNodeTypeToText(pNodeRecord->NodeInfo.NodeType),
			  NODE_DESCRIPTION_ARRAY_SIZE,
			  pNodeRecord->NodeDesc.NodeString);
	PrintFunc(dest, "%*sPorts: %d PortNum: %d PartitionCap: %d\n",
			  indent, "", pNodeRecord->NodeInfo.NumPorts,
			  pNodeRecord->NodeInfo.u1.s.LocalPortNum,
			  pNodeRecord->NodeInfo.PartitionCap);
	PrintFunc(dest,
			  "%*sNodeGuid: 0x%016" PRIx64 " PortGuid: 0x%016" PRIx64 "\n",
			  indent, "", pNodeRecord->NodeInfo.NodeGUID,
			  pNodeRecord->NodeInfo.PortGUID);
	PrintFunc(dest, "%*sSystemImageGuid: 0x%016" PRIx64 "\n", indent, "",
			  pNodeRecord->NodeInfo.SystemImageGUID);
	PrintFunc(dest,
			  "%*sBaseVersion: %d SmaVersion: %d VendorID: 0x%x DeviceId: 0x%x Revision: 0x%x\n",
			  indent, "", pNodeRecord->NodeInfo.BaseVersion,
			  pNodeRecord->NodeInfo.ClassVersion,
			  pNodeRecord->NodeInfo.u1.s.VendorID,
			  pNodeRecord->NodeInfo.DeviceID,
			  pNodeRecord->NodeInfo.Revision);
}

void
PrintStlPortInfoRecord(PrintDest_t * dest, int indent,
					   const STL_PORTINFO_RECORD * pPortInfoRecord)
{
	const STL_PORT_INFO *pPortInfo = &pPortInfoRecord->PortInfo;
	char buffer[SIZE_TIME];      // Buffer for formatting time output
	struct tm *loctime;          // Time structure to convert into human readable time

	PrintFunc(dest,
			  "%*sPortLID:       0x%08x        PortNum:   0x%02x (%2u)\n",
			  indent, "", pPortInfoRecord->RID.EndPortLID,
			  (uint32) pPortInfoRecord->RID.PortNum,
			  (uint32) pPortInfoRecord->RID.PortNum);
	PrintStlPortInfo(dest, indent, pPortInfo, 0, 0);

	// Print out the linkdown reasons (in time order)
	int startIdx = STL_LINKDOWN_REASON_NEXT_INDEX((STL_LINKDOWN_REASON*)&pPortInfoRecord->LinkDownReasons[0]);
	if (pPortInfoRecord->LinkDownReasons[startIdx].Timestamp==0) {
		// Empty entries exist, reset the startIdx to 0.
		startIdx = 0;
	} else {
		if (++startIdx >= STL_NUM_LINKDOWN_REASONS) {
			startIdx = 0;
		}
	}
	// If no Linkdown reasons exist, print notice.
	if (pPortInfoRecord->LinkDownReasons[startIdx].Timestamp==0) {
		PrintFunc(dest, "%*sLinkDownErrorLog: None\n", indent,"");
	} else {
		int i = 0;
		for (; i < STL_NUM_LINKDOWN_REASONS; i++) {
			if (pPortInfoRecord->LinkDownReasons[startIdx].Timestamp==0) {
				break; // We are done.
			}
			uint8 ldr = pPortInfoRecord->LinkDownReasons[startIdx].LinkDownReason;
			uint8 nldr = pPortInfoRecord->LinkDownReasons[startIdx].NeighborLinkDownReason;
			time_t ts = (time_t) pPortInfoRecord->LinkDownReasons[startIdx].Timestamp;

			loctime=localtime(&ts);    // convert timestamp into time structure
                        if (loctime == NULL) {     // ensure we have a valid time back
			  strncpy(buffer, "N/A", SIZE_TIME);
			}
                        else {
			  strftime(buffer, SIZE_TIME, "%B %d, %I:%M:%S %p",loctime);   // generate time string
                        }
			PrintFunc(dest, "%*sLinkDownErrorLog: %2d (%20s) Time: %s \n",
				indent, "", ldr, StlLinkDownReasonToText(ldr), buffer);
			PrintFunc(dest, "%*sNeighborLinkDownErrorLog: %2d (%20s) Time: %s\n",
				indent, "", nldr, StlLinkDownReasonToText(nldr), buffer);
			if (++startIdx >= STL_NUM_LINKDOWN_REASONS) {
				startIdx = 0;
			}
		}
	}

}

void
PrintStlSwitchInfoRecord(PrintDest_t * dest, int indent,
						 const STL_SWITCHINFO_RECORD * pSwitchInfoRecord)
{
	PrintStlSwitchInfo(dest, indent, &pSwitchInfoRecord->SwitchInfoData,
					   pSwitchInfoRecord->RID.LID, 0);
}

void
PrintStlPKeyTableRecord(PrintDest_t * dest, int indent,
						const STL_P_KEY_TABLE_RECORD * pPKeyTableRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x PortNum: %2u BlockNum: %2u\n",
			  indent, "", pPKeyTableRecord->RID.LID,
			  pPKeyTableRecord->RID.PortNum,
			  pPKeyTableRecord->RID.Blocknum);
	PrintStlPKeyTable(dest, indent, &pPKeyTableRecord->PKeyTblData,
					  pPKeyTableRecord->RID.Blocknum, 0);
}

void
PrintStlSCSCTableRecord(PrintDest_t * dest, int indent, int extended,
						const STL_SC_MAPPING_TABLE_RECORD * pSCSCMapRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x InputPort: %3u OutputPort: %3u %s\n",
			  indent, "",
			  pSCSCMapRecord->RID.LID, pSCSCMapRecord->RID.InputPort,
			  pSCSCMapRecord->RID.OutputPort,
			  !extended ? "" : "(secondary)");

	PrintStlSCSCMap(dest, indent, "",
						(STL_SCSCMAP *) & pSCSCMapRecord->Map, TRUE);
}

void
PrintStlSLSCTableRecord(PrintDest_t * dest, int indent,
						const STL_SL2SC_MAPPING_TABLE_RECORD * pSLSCMapRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x\n",
			  indent, "", pSLSCMapRecord->RID.LID);
	PrintStlSLSCMap(dest, indent, "",
					(STL_SLSCMAP *) & pSLSCMapRecord->SLSCMap, TRUE);
}

void
PrintStlSCSLTableRecord(PrintDest_t * dest, int indent,
						const STL_SC2SL_MAPPING_TABLE_RECORD * pSCSLMapRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x\n",
			  indent, "", pSCSLMapRecord->RID.LID);
	PrintStlSCSLMap(dest, indent, "",
					(STL_SCSLMAP *) & pSCSLMapRecord->SCSLMap, TRUE);
}

void
PrintStlSCVLxTableRecord(PrintDest_t * dest, int indent,
						 const STL_SC2VL_R_MAPPING_TABLE_RECORD * pSCVLxMapRecord,
						 uint16_t attribute)
{
	PrintFunc(dest, "%*sLID: 0x%08x Port: %3u\n",
			  indent, "", pSCVLxMapRecord->RID.LID,
			  pSCVLxMapRecord->RID.Port);
	PrintStlSCVLxMap(dest, indent, "",
					(STL_SCVLMAP *) & pSCVLxMapRecord->SCVLMap, TRUE,
					 attribute, FALSE);
}

void
PrintStlLinearFDBRecord(PrintDest_t * dest, int indent,
						const STL_LINEAR_FORWARDING_TABLE_RECORD *
						pLinearFDBRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x BlockNum: %6u\n",
			  indent, "", pLinearFDBRecord->RID.LID,
			  pLinearFDBRecord->RID.BlockNum);
	PrintStlLinearFDB(dest, indent,
					  (STL_LINEAR_FORWARDING_TABLE *) pLinearFDBRecord->
					  LinearFdbData, pLinearFDBRecord->RID.BlockNum, 0);
}


void
PrintStlVLArbTableRecord(PrintDest_t * dest, int indent,
						 const STL_VLARBTABLE_RECORD * pVLArbTableRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x Port: %02u\n", indent, "",
			  pVLArbTableRecord->RID.LID,
			  pVLArbTableRecord->RID.OutputPortNum);
	PrintStlVLArbTable(dest, indent, &pVLArbTableRecord->VLArbTable,
					   pVLArbTableRecord->RID.BlockNum, 0);
}


void
PrintStlMcMemberRecord(PrintDest_t * dest, int indent,
					   const STL_MCMEMBER_RECORD * pMcMemberRecord)
{
	char buf[8];
	PrintFunc(dest, "%*sGID: 0x%016" PRIx64 ":0x%016" PRIx64 "\n",
			  indent, "",
			  pMcMemberRecord->RID.MGID.AsReg64s.H,
			  pMcMemberRecord->RID.MGID.AsReg64s.L);
	PrintFunc(dest,
			  "%*sPortGid: 0x%016" PRIx64 ":0x%016" PRIx64
			  " Membership: %s%s%s\n", indent, "",
			  pMcMemberRecord->RID.PortGID.Type.Global.SubnetPrefix,
			  pMcMemberRecord->RID.PortGID.Type.Global.InterfaceID,
			  pMcMemberRecord->JoinFullMember ? "Full " : "",
			  pMcMemberRecord->JoinNonMember ? "Non " : "",
			  pMcMemberRecord->JoinSendOnlyMember ? "Sendonly " : "");
	FormatTimeoutMult(buf, pMcMemberRecord->PktLifeTime);
	PrintFunc(dest,
			  "%*sMLID: 0x%08x PKey: 0x%04x Mtu: %5s Rate: %4s PktLifeTime: %s\n",
			  indent, "", pMcMemberRecord->MLID, pMcMemberRecord->P_Key,
			  IbMTUToText(pMcMemberRecord->Mtu),
			  StlStaticRateToText(pMcMemberRecord->Rate), buf);
	PrintFunc(dest,
			  "%*sQKey: 0x%08x SL: %2u HopLimit: 0x%02x  TClass:  0x%02x\n",
			  indent, "", pMcMemberRecord->Q_Key, pMcMemberRecord->SL,
			  pMcMemberRecord->HopLimit, pMcMemberRecord->TClass);
}

void
PrintStlMCastFDBRecord(PrintDest_t * dest, int indent,
					   const STL_MULTICAST_FORWARDING_TABLE_RECORD *
					   pMCastFDBRecord)
{
	PrintFunc(dest, "%*sLID: 0x%08x Position: %1u BlockNum: %7u\n",
			  indent, "", pMCastFDBRecord->RID.LID,
			  pMCastFDBRecord->RID.u1.s.Position,
			  pMCastFDBRecord->RID.u1.s.BlockNum);
	PrintStlMCastFDB(dest, indent, &pMCastFDBRecord->MftTable,
					 pMCastFDBRecord->RID.u1.s.BlockNum, 0);
}

void
PrintStlLinkRecord(PrintDest_t * dest, int indent,
				   const STL_LINK_RECORD * pLinkRecord)
{
	{

		PrintFunc(dest, "%*sLID: 0x%08x -> 0x%08x Port: %3u -> %3u\n",
			  indent, "",
			  pLinkRecord->RID.FromLID, pLinkRecord->ToLID,
			  pLinkRecord->RID.FromPort, pLinkRecord->ToPort);
	}

}


void
PrintStlInformInfoRecord(PrintDest_t * dest, int indent,
						 const STL_INFORM_INFO_RECORD * pInformInfoRecord)
{
	PrintFunc(dest,
			  "%*sSubLID: 0x%08x Enum: 0x%04x\n", indent, "",
			  pInformInfoRecord->RID.SubscriberLID,
			  pInformInfoRecord->RID.Enum);
	PrintStlInformInfo(dest, indent, &pInformInfoRecord->InformInfoData);
}

void PrintStlVfInfoRecord_detail(PrintDest_t *dest, int indent, int detail,
	const STL_VFINFO_RECORD *pVfInfo, int showQueryParams)
{
	char buf[8];
	char slStr[40];

	PrintFunc(dest,"%*svFabric Index: %d   Name: %s \n",
				indent, "",
				pVfInfo->vfIndex,
				pVfInfo->vfName);

	int remChars = sizeof(slStr);
	char *slStrPtr = slStr;
	int ret;

	ret = snprintf(slStrPtr, remChars, "%d", pVfInfo->s1.slBase);
	if (ret > 0 && ret <= remChars) {
		remChars -= ret;
		slStrPtr += ret;
	}


	if (pVfInfo->slMulticastSpecified) {
		ret = snprintf(slStrPtr, remChars, " McastSL: %d", pVfInfo->slMulticast);
		if (ret > 0 && ret <= remChars) {
			remChars -= ret;
			slStrPtr += ret;
		}
	}

	if (detail > 1) {
		if (showQueryParams != 0) {
			PrintFunc(dest,"%*sServiceId: 0x%016"PRIx64"  MGID: 0x%016"PRIx64":0x%016"PRIx64"\n",
							indent, "", pVfInfo->ServiceID,
							pVfInfo->MGID.AsReg64s.H,
							pVfInfo->MGID.AsReg64s.L);
		}

		// FormatTimeoutMult(buf, pVfInfo->s1.pktLifeTimeInc);
		snprintf(buf, 8, "%d", 1<<pVfInfo->s1.pktLifeTimeInc);

		PrintFunc(dest,"%*sPKey: 0x%x   SL: %s  Select: 0x%x%s %s%s  PktLifeTimeMult: %s \n",
					indent, "",
					pVfInfo->pKey,
					slStr,
					pVfInfo->s1.selectFlags,
					pVfInfo->s1.selectFlags? ":" : "",
					(pVfInfo->s1.selectFlags&STL_VFINFO_REC_SEL_PKEY_QUERY) ? "PKEY " : "",
					(pVfInfo->s1.selectFlags&STL_VFINFO_REC_SEL_SL_QUERY) ? "SL ": "",
					pVfInfo->s1.pktLifeSpecified? buf: "unspecified");

		if (pVfInfo->s1.mtuSpecified) {
			PrintFunc(dest,"%*sMaxMtu: %5s  ",
					indent, "",
					IbMTUToText(pVfInfo->s1.mtu));
		} else
			PrintFunc(dest,"%*sMaxMtu: unlimited  ", indent, "");


		PrintFunc(dest,"%*sMaxRate: %s   ", indent, "",
					pVfInfo->s1.rateSpecified ? IbStaticRateToText(pVfInfo->s1.rate) : "unlimited");

		PrintFunc(dest, "%*sOptions: 0x%02x%s %s%s%s\n", indent, "",
					pVfInfo->optionFlags,
					pVfInfo->optionFlags? ":" : "",
					(pVfInfo->optionFlags&STL_VFINFO_REC_OPT_SECURITY) ? "Security " : "",
					(pVfInfo->optionFlags&STL_VFINFO_REC_OPT_QOS) ? "QoS " : "",
					(pVfInfo->optionFlags&STL_VFINFO_REC_OPT_FLOW_DISABLE) ? "FlowCtrlDisable" : "");

		FormatTimeoutMult(buf, pVfInfo->hoqLife);

		if (pVfInfo->optionFlags&STL_VFINFO_REC_OPT_QOS) {
			if (pVfInfo->priority) {
				if (pVfInfo->bandwidthPercent) {
					PrintFunc(dest,"%*sQOS: Bandwidth: %3d%%  Priority: %s  PreemptionRank: %u  HoQLife: %s\n",
						indent, "", pVfInfo->bandwidthPercent, "high", pVfInfo->preemptionRank, buf);
				} else {
					PrintFunc(dest,"%*sQOS: HighPriority  PreemptionRank: %u  HoQLife: %s\n", indent, "", pVfInfo->preemptionRank, buf);
				}
			} else {
				PrintFunc(dest,"%*sQOS: Bandwidth: %3d%%  PreemptionRank: %u  HoQLife: %s\n",
					indent, "", pVfInfo->bandwidthPercent, pVfInfo->preemptionRank, buf);
			}
		} else {
			PrintFunc(dest,"%*sQOS: Disabled  PreemptionRank: %u  HoQLife: %s\n",
					indent, "", pVfInfo->preemptionRank, buf);
		}
	} else {
		PrintFunc(dest, "%*sPKey: 0x%x   SL: %s\n", indent, "", pVfInfo->pKey, slStr);
	}
}

void PrintStlVfInfoRecord(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo)
{
	PrintStlVfInfoRecord_detail(dest, indent, 255, pVfInfo, 1);
}

// implementation of single-line VFInfo output
// @param enumsAsText - when true, convert enumerated values to human-readable text, when false print enums as integer values
static void PrintStlVfInfoRecordCSV_impl(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo, boolean enumsAsText)
{
	char mcastSl[4] = "";

	if (pVfInfo->slMulticastSpecified)
		snprintf(mcastSl, sizeof(mcastSl), "%u", pVfInfo->slMulticast);

	if (enumsAsText) {
		PrintFunc(dest,"%*s%s:%d:0x%x:%d:%s:%s:0x%x:%s\n",

			indent, "",
			pVfInfo->vfName,
			pVfInfo->vfIndex,
			pVfInfo->pKey,
			pVfInfo->s1.slBase,
			(pVfInfo->s1.mtuSpecified)? IbMTUToText(pVfInfo->s1.mtu):"unlimited",
			pVfInfo->s1.rateSpecified ? IbStaticRateToText(pVfInfo->s1.rate) : "unlimited",
			pVfInfo->optionFlags,
			mcastSl);
	} else {
		PrintFunc(dest,"%*s%s:%d:0x%x:%d:%d:%d:0x%x:%s\n",
			indent, "",
			pVfInfo->vfName,
			pVfInfo->vfIndex,
			pVfInfo->pKey,
			pVfInfo->s1.slBase,
			(pVfInfo->s1.mtuSpecified)? pVfInfo->s1.mtu:0,
			pVfInfo->s1.rateSpecified ? pVfInfo->s1.rate:0,
			pVfInfo->optionFlags,
			mcastSl);
	}
}

// output VFINFO in a delimited format for easy parsing in shell scripts
void PrintStlVfInfoRecordCSV2(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo)
{
	PrintStlVfInfoRecordCSV_impl(dest, indent, pVfInfo, 0);
}

// output VFINFO in a delimited format for easy parsing in shell scripts
void PrintStlVfInfoRecordCSV(PrintDest_t *dest, int indent, const STL_VFINFO_RECORD *pVfInfo)
{
	PrintStlVfInfoRecordCSV_impl(dest, indent, pVfInfo, 1);
}


void PrintStlTraceRecord(PrintDest_t *dest, int indent, const STL_TRACE_RECORD *pTraceRecord)
{
	PrintFunc(dest, "%*sIDGeneration: 0x%04x\n",
			  indent, "", pTraceRecord->IDGeneration);
	PrintFunc(dest, "%*sNodeType: %s\n",
			  indent, "", StlNodeTypeToText(pTraceRecord->NodeType));
	PrintFunc(dest, "%*sNodeID: 0x%016"PRIx64" ChassisID: %016"PRIx64"\n",
			  indent, "", pTraceRecord->NodeID, pTraceRecord->ChassisID);
	PrintFunc(dest, "%*sEntryPortID: 0x%016"PRIx64" ExitPortID: %016"PRIx64"\n",
			  indent, "", pTraceRecord->EntryPortID, pTraceRecord->ExitPortID);
	PrintFunc(dest, "%*sEntryPort: 0x%02x ExitPort: 0x%02x\n",
			  indent, "", pTraceRecord->EntryPort, pTraceRecord->ExitPort);
}

void PrintStlFabricInfoRecord(PrintDest_t *dest, int indent, const STL_FABRICINFO_RECORD *pFabricInfoRecord)
{
	PrintFunc(dest, "%*sNumber of HFIs: %u\n",
			  indent, "", pFabricInfoRecord->NumHFIs);
	PrintFunc(dest, "%*sNumber of Switches: %u\n",
			  indent, "", pFabricInfoRecord->NumSwitches);
	PrintFunc(dest, "%*sNumber of Links: %u\n",
			  indent, "", pFabricInfoRecord->NumInternalHFILinks
			  				+ pFabricInfoRecord->NumExternalHFILinks
			  				+ pFabricInfoRecord->NumInternalISLs
			  				+ pFabricInfoRecord->NumExternalISLs);
	PrintFunc(dest, "%*sNumber of HFI Links: %-7u        (Internal: %u   External: %u)\n",
			  indent, "", pFabricInfoRecord->NumInternalHFILinks +
			  				pFabricInfoRecord->NumExternalHFILinks,
			  pFabricInfoRecord->NumInternalHFILinks,
			  pFabricInfoRecord->NumExternalHFILinks);
	PrintFunc(dest, "%*sNumber of ISLs: %-7u             (Internal: %u   External: %u)\n",
			  indent, "", pFabricInfoRecord->NumInternalISLs +
			  				pFabricInfoRecord->NumExternalISLs,
			  pFabricInfoRecord->NumInternalISLs,
			  pFabricInfoRecord->NumExternalISLs);
	PrintFunc(dest, "%*sNumber of Degraded Links: %-7u   (HFI Links: %u   ISLs: %u)\n",
			  indent, "", pFabricInfoRecord->NumDegradedHFILinks +
			  					pFabricInfoRecord->NumDegradedISLs,
			  pFabricInfoRecord->NumDegradedHFILinks,
			  pFabricInfoRecord->NumDegradedISLs);
	PrintFunc(dest, "%*sNumber of Omitted Links: %-7u    (HFI Links: %u   ISLs: %u)\n",
			  indent, "", pFabricInfoRecord->NumOmittedHFILinks +
			  					pFabricInfoRecord->NumOmittedISLs,
			  pFabricInfoRecord->NumOmittedHFILinks,
			  pFabricInfoRecord->NumOmittedISLs);
}

void PrintQuarantinedNodeRecord(PrintDest_t *dest, int indent, const STL_QUARANTINED_NODE_RECORD *pQuarantinedNodeRecord)
{
	int violStrLen = 256;
	char violationString[violStrLen];
	int previousViolation = 0;
	memset(violationString, 0, sizeof(violationString));

	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_SPOOF_GENERIC) {
		strncat(violationString, "NodeGUID/NodeType Spoofing", violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_DESC) {
		strncat(violationString, previousViolation ? ", NodeDesc Mismatch" : "NodeDesc Mismatch", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_GUID) {
		strncat(violationString, previousViolation ? ", NodeGUID Mismatch" : "NodeGUID Mismatch", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_PORT_GUID) {
		strncat(violationString, previousViolation ? ", PortGUID Mismatch" : "PortGUID Mismatch", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK) {
		strncat(violationString, previousViolation ? ", Undefined Link" : "Undefined Link", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_SMALL_MTU_SIZE) {
		strncat(violationString, previousViolation ? ", Small MTU Size" : "Small MTU Size", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_VL_COUNT) {
		strncat(violationString, previousViolation ? ", Incorrect VL Count" : "Incorrect VL Count", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_MAXLID) {
		strncat(violationString, previousViolation ? ", MaximumLID unsupportable" : "MaximumLID unsupportable", 
					violStrLen - strlen(violationString));
		previousViolation = 1;
	}

	PrintFunc(dest, "%*sConnected to Port %d of (LID: 0x%x, NodeGUID: 0x%016" PRIx64 ")\n", indent, "", pQuarantinedNodeRecord->trustedPortNum, pQuarantinedNodeRecord->trustedLid, pQuarantinedNodeRecord->trustedNodeGUID);
	PrintFunc(dest, "%*s    Offending Node Actual NodeGUID: 0x%016" PRIx64 "\n", indent, "", pQuarantinedNodeRecord->trustedNeighborNodeGUID);
	PrintFunc(dest, "%*s    Violation(s): %s\n", indent, "", violationString);

	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_DESC) {
		PrintFunc(dest, "%*s    Expected Node Description: %.*s\n", indent, "", STL_NODE_DESCRIPTION_ARRAY_SIZE, pQuarantinedNodeRecord->expectedNodeInfo.nodeDesc.NodeString);
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_GUID) {
		PrintFunc(dest, "%*s    Expected NodeGUID: 0x%016" PRIx64 "\n", indent, "", pQuarantinedNodeRecord->expectedNodeInfo.nodeGUID);
	}
	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_TOPO_PORT_GUID) {
		PrintFunc(dest, "%*s    Expected PortGUID: 0x%016" PRIx64 "\n", indent, "", pQuarantinedNodeRecord->expectedNodeInfo.portGUID);
	}

	if(pQuarantinedNodeRecord->quarantineReasons & STL_QUARANTINE_REASON_SPOOF_GENERIC) {
		PrintFunc(dest, "%*s    <SPOOFED NODE REPORTED INFORMATION (May be falsified)>\n", indent, "");
	} else {
		PrintFunc(dest, "%*s    Received node Information:\n", indent, "");
	}

	PrintFunc(dest, "%*s        Node Description: %.*s\n", indent, "", NODE_DESCRIPTION_ARRAY_SIZE, pQuarantinedNodeRecord->NodeDesc.NodeString);
	PrintFunc(dest, "%*s        Type: %s Ports: %d PortNum: %d PartitionCap: %d\n", indent, "", StlNodeTypeToText(pQuarantinedNodeRecord->NodeInfo.NodeType), pQuarantinedNodeRecord->NodeInfo.NumPorts, pQuarantinedNodeRecord->NodeInfo.u1.s.LocalPortNum, pQuarantinedNodeRecord->NodeInfo.PartitionCap);
	PrintFunc(dest, "%*s        NodeGUID: 0x%016" PRIx64 " PortGUID: 0x%016" PRIx64 "\n", indent, "", pQuarantinedNodeRecord->NodeInfo.NodeGUID, pQuarantinedNodeRecord->NodeInfo.PortGUID);
	PrintFunc(dest, "%*s        SystemImageGuid: 0x%016" PRIx64 " BaseVersion: %d SmaVersion: %d\n", indent, "", pQuarantinedNodeRecord->NodeInfo.SystemImageGUID, pQuarantinedNodeRecord->NodeInfo.BaseVersion, pQuarantinedNodeRecord->NodeInfo.ClassVersion);
	PrintFunc(dest, "%*s        VendorID: 0x%x DeviceId: 0x%x Revision: 0x%x\n", indent, "", pQuarantinedNodeRecord->NodeInfo.u1.s.VendorID, pQuarantinedNodeRecord->NodeInfo.DeviceID, pQuarantinedNodeRecord->NodeInfo.Revision);
}
 
void PrintStlCongestionInfoRecord(PrintDest_t *dest, int indent, const STL_CONGESTION_INFO_RECORD *pCongestionInfo) {
	PrintFunc(dest, "%*sLID %d\n", indent, "", pCongestionInfo->LID);
	PrintStlCongestionInfo(dest, indent, &pCongestionInfo->CongestionInfo, 0);
}

void PrintStlSwitchCongestionSettingRecord(PrintDest_t *dest, int indent, const STL_SWITCH_CONGESTION_SETTING_RECORD *pSwCongestionSetting) {
	PrintFunc(dest, "%*sLID %d\n", indent, "", pSwCongestionSetting->LID);
	PrintStlSwitchCongestionSetting(dest, indent, &pSwCongestionSetting->SwitchCongestionSetting, 0);
}

void PrintStlSwitchPortCongestionSettingRecord(PrintDest_t *dest, int indent, const STL_SWITCH_PORT_CONGESTION_SETTING_RECORD *pSwPortCS) {
	PrintFunc(dest, "%*sLID %d Port %d:\n", indent, "", pSwPortCS->RID.LID, pSwPortCS->RID.Port);
	PrintStlSwitchPortCongestionSettingElement(dest, indent, pSwPortCS->SwitchPortCongestionSetting.Elements, 0, 0);
}

void PrintStlHfiCongestionSettingRecord(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_SETTING_RECORD *pHfiCongestionSetting) {
	PrintFunc(dest, "%*sLID %d\n", indent, "", pHfiCongestionSetting->LID);
	PrintStlHfiCongestionSetting(dest, indent, &pHfiCongestionSetting->HFICongestionSetting, 0);
}

void PrintStlHfiCongestionControlTabRecord(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_CONTROL_TABLE_RECORD *pHfiCongestionControl) {
	PrintFunc(dest, "%*sLID %d BlockNum %d:\n", indent, "", pHfiCongestionControl->RID.LID, pHfiCongestionControl->RID.BlockNum);
	PrintStlHfiCongestionControlTab(dest, indent, &pHfiCongestionControl->HFICongestionControlTable, 1, 0, 0);
}

void PrintBfrCtlTableRecord(PrintDest_t *dest, int indent, const STL_BUFFER_CONTROL_TABLE_RECORD *pBfrCtlTable)
{
	PrintFunc(dest, "%*sLID %d Port %d:\n", indent, "", pBfrCtlTable->RID.LID, pBfrCtlTable->RID.Port);
	PrintStlBfrCtlTable(dest, indent, &pBfrCtlTable->BufferControlTable, 0);
}

void PrintStlCableInfoRecord(PrintDest_t *dest, int indent, const STL_CABLE_INFO_RECORD *pCableInfoRecord)
{
	PrintFunc(dest, "%*sLID %d Port %d:\nPortType: %s\n", indent, "", 
				pCableInfoRecord->LID, pCableInfoRecord->Port, StlPortTypeToText(pCableInfoRecord->u1.s.PortType));
	PrintStlCableInfo(dest, indent, pCableInfoRecord->Data, 
				pCableInfoRecord->u1.s.Address, pCableInfoRecord->Length+1, pCableInfoRecord->u1.s.PortType, CABLEINFO_DETAIL_ALL, 0, FALSE);
}

void PrintStlPortGroupTabRecord(PrintDest_t *dest, int indent, const STL_PORT_GROUP_TABLE_RECORD *pRecord)
{
	PrintFunc(dest, "%*sLID %d Block %d:\n", indent, "", pRecord->RID.LID, pRecord->RID.BlockNum);
	PrintStlPortGroupTable(dest, indent, pRecord->GroupBlock, pRecord->RID.BlockNum, 0, 0);
}

void PrintStlPortGroupFwdTabRecord(PrintDest_t *dest, int indent, const STL_PORT_GROUP_FORWARDING_TABLE_RECORD *pRecord)
{

	PrintFunc(dest, "%*sSwitch LID: 0x%08x BlockNum: %6u\n", indent, "", pRecord->RID.LID, pRecord->RID.u1.s.BlockNum);
	PrintStlPortGroupFDB(dest, indent+4, (STL_PORT_GROUP_FORWARDING_TABLE *) pRecord->PGFdbData, pRecord->RID.u1.s.BlockNum, 0);
}
void PrintStlDeviceGroupMemberRecord(PrintDest_t *dest, int indent, const STL_DEVICE_GROUP_MEMBER_RECORD *pRecord)
{
	PrintFunc(dest,"%*sDevice Group: %s LID: 0x%08x Port: %d PortGUID: 0x%016" PRIx64 " Node Description: %s\n", indent, "",
		pRecord->DeviceGroupName, pRecord->LID, pRecord->Port, pRecord->GUID, pRecord->NodeDescription.NodeString);
}

void PrintStlDeviceGroupNameRecord(PrintDest_t *dest, int indent, const STL_DEVICE_GROUP_NAME_RECORD *pRecord, int record_no)
{
	PrintFunc(dest,"%*sGroup %d: %s\n", indent, "",
		record_no, pRecord->DeviceGroupName);
}


void PrintStlDeviceTreeMemberRecord(PrintDest_t *dest, int indent, const STL_DEVICE_TREE_MEMBER_RECORD *pRecord)
{
	char buf_act[256];
	char buf_mode[256];
	int length = strlen("Ports :"); // The string from FormatStlPortMask()

	FormatStlPortMask(buf_act, pRecord->portMaskAct, MAX_STL_PORTS, sizeof(buf_act));
	FormatStlPortMask(buf_mode, pRecord->portMaskPortLinkMode, MAX_STL_PORTS, sizeof(buf_mode));

	PrintFunc(dest,"%*sNodeLID: 0x%08x NodeGUID: 0x%016" PRIx64 " NodeSystemImageGUID: 0x%016" PRIx64 " Type: [%s] Node Description: [%s] NumPorts: %d Active %s\n",
		indent, "", pRecord->LID, pRecord->GUID, pRecord->SystemImageGUID, StlNodeTypeToText(pRecord->NodeType), pRecord->NodeDescription.NodeString, pRecord->NumPorts,
		strlen(buf_act) > length ? buf_act : "ports: none");

}

void PrintStlSwitchCostRecord(PrintDest_t *dest, int indent, const STL_SWITCH_COST_RECORD *pRecord)
{
	int i;
	PrintFunc(dest, "%*sSource LID: 0x%08x\n", indent, "", pRecord->SLID);
	for(i = 0; (i < STL_SWITCH_COST_NUM_ENTRIES) && (pRecord->Cost[i].DLID != 0); ++i){
		PrintFunc(dest, "%*sLID: 0x%08x -> Cost: %u\n", indent+4, "", pRecord->Cost[i].DLID, pRecord->Cost[i].value);
	}
}

#define HEXTOCHAR(c) ((isgraph(c)||(c)==' ')?(c):'.')

static void FormatChars(char *buf, const uint8* data, uint32 len)
{
	uint32 j;
	int offset = 0;

	for (j=0; j<len; ++j)
		buf[offset++]=HEXTOCHAR(data[j]);
}

void PrintStlServiceRecord(PrintDest_t *dest, int indent, 
	const STL_SERVICE_RECORD *pServiceRecord)
{
	char buf[81];
	int offset;
	unsigned j;

	PrintFunc(dest, "%*sSID: 0x%016"PRIx64" GID: 0x%016"PRIx64":0x%016"PRIx64" PKey: 0x%04x\n",
				indent, "", pServiceRecord->RID.ServiceID,
				pServiceRecord->RID.ServiceGID.Type.Global.SubnetPrefix,
				pServiceRecord->RID.ServiceGID.Type.Global.InterfaceID,
				pServiceRecord->RID.ServiceP_Key);
	if (pServiceRecord->ServiceLease == SERVICE_LEASE_INFINITE)
		strcpy(buf, "Infinite");
	else
		// %6d not quite big enough for uint32, but will cover most non-infinite
		// practical values
		sprintf(buf, "%6d s", pServiceRecord->ServiceLease);
	PrintFunc(dest, "%*sLease: %s  Name: %s\n",
				indent, "", buf, pServiceRecord->ServiceName);
	// dump service data
	offset=sprintf(buf, "Data8: ");
	for (j=0; j<16; ++j)
		offset+=sprintf(&buf[offset], " %02x", pServiceRecord->ServiceData8[j]);
	offset+=sprintf(&buf[offset], " ");
	FormatChars(&buf[offset], pServiceRecord->ServiceData8, 16);
	PrintFunc(dest, "%*s%s\n", indent, "", buf);

	offset=sprintf(buf, "Data16:");
	for (j=0; j<8; ++j)
		offset+=sprintf(&buf[offset], " %04x", pServiceRecord->ServiceData16[j]);
	offset+=sprintf(&buf[offset], "         ");
	FormatChars(&buf[offset], (uint8*)pServiceRecord->ServiceData16, 16);
	PrintFunc(dest, "%*s%s\n", indent, "", buf);

	offset=sprintf(buf, "Data32:");
	for (j=0; j<4; ++j)
		offset+=sprintf(&buf[offset], " %08x", pServiceRecord->ServiceData32[j]);
	offset+=sprintf(&buf[offset], "             ");
	FormatChars(&buf[offset], (uint8*)pServiceRecord->ServiceData32, 16);
	PrintFunc(dest, "%*s%s\n", indent, "", buf);

	offset=sprintf(buf, "Data64:");
	for (j=0; j<2; ++j)
		offset+=sprintf(&buf[offset], " %016"PRIx64, pServiceRecord->ServiceData64[j]);
	offset+=sprintf(&buf[offset], "               ");
	FormatChars(&buf[offset], (uint8*)pServiceRecord->ServiceData64, 16);
	PrintFunc(dest, "%*s%s\n", indent, "", buf);
}
