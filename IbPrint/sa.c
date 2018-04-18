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

// Prints a path record with extended information. (Currently, just the SID.)
void PrintExtendedPathRecord(PrintDest_t *dest, int indent, const IB_PATH_RECORD *pPathRecord)
{
	PrintFunc(dest, "%*sSID:  0x%016"PRIx64"\n",
				indent, "",
				pPathRecord->ServiceID);

	PrintPathRecord(dest,indent,pPathRecord);
}

	char buf[8];
// one print function for each SA query OutputType
void PrintPathRecord(PrintDest_t *dest, int indent, const IB_PATH_RECORD *pPathRecord)
{
	char buf[8];
	PrintFunc(dest, "%*sSGID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",
				pPathRecord->SGID.Type.Global.SubnetPrefix,
				pPathRecord->SGID.Type.Global.InterfaceID);
	PrintFunc(dest, "%*sDGID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",
				pPathRecord->DGID.Type.Global.SubnetPrefix,
				pPathRecord->DGID.Type.Global.InterfaceID);
	PrintFunc(dest, "%*sSLID: 0x%04x DLID: 0x%04x Reversible: %s PKey: 0x%04x\n",
				indent, "", pPathRecord->SLID, pPathRecord->DLID,
				pPathRecord->Reversible?"Y":"N", pPathRecord->P_Key);
	if (IsGlobalRoute(pPathRecord)) {
		// Technically Raw is not part of GRH, but it too is not used
		PrintFunc(dest, "%*sRaw: %s FlowLabel: 0x%05x HopLimit: 0x%02x TClass: 0x%02x\n",
				indent, "", pPathRecord->u1.s.RawTraffic?"Y":"N",
				pPathRecord->u1.s.FlowLabel, pPathRecord->u1.s.HopLimit,
				pPathRecord->TClass);
	}
	FormatTimeoutMult(buf, pPathRecord->PktLifeTime);
	PrintFunc(dest, "%*sSL: %2u Mtu: %5s Rate: %4s PktLifeTime: %s Pref: %d\n",
				indent, "", pPathRecord->u2.s.SL, IbMTUToText(pPathRecord->Mtu),
				StlStaticRateToText(pPathRecord->Rate), buf,
				pPathRecord->Preference);
}

void PrintNodeRecord(PrintDest_t *dest, int indent, const IB_NODE_RECORD *pNodeRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x Type: %s  Name: %.*s\n",
				indent, "", pNodeRecord->RID.s.LID,
				StlNodeTypeToText(pNodeRecord->NodeInfoData.NodeType),
				NODE_DESCRIPTION_ARRAY_SIZE,
				pNodeRecord->NodeDescData.NodeString);
	PrintFunc(dest, "%*sPorts: %d PortNum: %d PartitionCap: %d\n",
				indent, "", pNodeRecord->NodeInfoData.NumPorts,
				pNodeRecord->NodeInfoData.u1.s.LocalPortNum,
				pNodeRecord->NodeInfoData.PartitionCap
				);
	PrintFunc(dest, "%*sNodeGuid: 0x%016"PRIx64" PortGuid: 0x%016"PRIx64"\n",
				indent, "",
				pNodeRecord->NodeInfoData.NodeGUID,
				pNodeRecord->NodeInfoData.PortGUID);
	PrintFunc(dest, "%*sSystemImageGuid: 0x%016"PRIx64"\n",
				indent, "", pNodeRecord->NodeInfoData.SystemImageGUID);
	PrintFunc(dest, "%*sBaseVersion: %d SmaVersion: %d VendorID: 0x%x DeviceId: 0x%x Revision: 0x%x\n",
				indent, "", pNodeRecord->NodeInfoData.BaseVersion,
				pNodeRecord->NodeInfoData.ClassVersion,
				pNodeRecord->NodeInfoData.u1.s.VendorID,
				pNodeRecord->NodeInfoData.DeviceID,
				pNodeRecord->NodeInfoData.Revision);
}

void PrintPortInfoRecord(PrintDest_t *dest, int indent, const IB_PORTINFO_RECORD *pPortInfoRecord)
{
	const PORT_INFO *pPortInfo = &pPortInfoRecord->PortInfoData;

	PrintFunc(dest, "%*sLID: 0x%04x PortNum: %d LocalPortNum: %d\n",
				indent, "", pPortInfoRecord->RID.s.EndPortLID,
				pPortInfoRecord->RID.s.PortNum, pPortInfo->LocalPortNum);
	PrintPortInfo(dest, indent, pPortInfo, 0, 0);
}

void PrintSMInfoRecord(PrintDest_t *dest, int indent, const IB_SMINFO_RECORD *pSMInfoRecord)
{
	PrintSMInfo(dest, indent, &pSMInfoRecord->SMInfoData,
				pSMInfoRecord->RID.s.LID);
}

void PrintLinkRecord(PrintDest_t *dest, int indent, const IB_LINK_RECORD *pLinkRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x -> 0x%04x Port: %3u -> %3u\n",
				indent, "",
				pLinkRecord->RID.FromLID, pLinkRecord->ToLID,
				pLinkRecord->RID.FromPort, pLinkRecord->ToPort);
}

void PrintTraceRecord(PrintDest_t *dest, int indent, const IB_TRACE_RECORD *pTraceRecord)
{
    PrintFunc(dest, "%*sGIDPrefix: 0x%016"PRIx64"\n",
           indent, "", pTraceRecord->GIDPrefix);
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

void PrintSwitchInfoRecord(PrintDest_t *dest, int indent, const IB_SWITCHINFO_RECORD *pSwitchInfoRecord)
{
	PrintSwitchInfo(dest, indent, &pSwitchInfoRecord->SwitchInfoData,
				pSwitchInfoRecord->RID.s.LID);
}

void PrintLinearFDBRecord(PrintDest_t *dest, int indent, const IB_LINEAR_FDB_RECORD *pLinearFDBRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x BlockNum: %5u\n",
				indent, "", pLinearFDBRecord->RID.s.LID,
				pLinearFDBRecord->RID.s.BlockNum);
	PrintLinearFDB(dest, indent, &pLinearFDBRecord->LinearFdbData,
				pLinearFDBRecord->RID.s.BlockNum);
}

void PrintRandomFDBRecord(PrintDest_t *dest, int indent, const IB_RANDOM_FDB_RECORD *pRandomFDBRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x BlockNum: %5u\n",
				indent, "", pRandomFDBRecord->RID.s.LID,
				pRandomFDBRecord->RID.s.BlockNum);
	PrintRandomFDB(dest, indent, &pRandomFDBRecord->RandomFdbData,
				pRandomFDBRecord->RID.s.BlockNum);
}

void PrintMCastFDBRecord(PrintDest_t *dest, int indent, const IB_MCAST_FDB_RECORD *pMCastFDBRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x Position: %2u BlockNum: %5u\n",
				indent, "", pMCastFDBRecord->RID.s.LID,
				pMCastFDBRecord->RID.s.Position,
				pMCastFDBRecord->RID.s.BlockNum);
	PrintMCastFDB(dest, indent, &pMCastFDBRecord->MCastFdbData,
				pMCastFDBRecord->RID.s.BlockNum);
}

void PrintVLArbTableRecord(PrintDest_t *dest, int indent, const IB_VLARBTABLE_RECORD *pVLArbTableRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x OutputPortNum: %2u BlockNum: %2u (%s)\n",
				indent, "",
				pVLArbTableRecord->RID.s.LID,
				pVLArbTableRecord->RID.s.OutputPortNum,
				pVLArbTableRecord->RID.s.BlockNum,
				pVLArbTableRecord->RID.s.BlockNum == 1?"Low Pri lower":
				pVLArbTableRecord->RID.s.BlockNum == 2?"Low Pri upper":
				pVLArbTableRecord->RID.s.BlockNum == 3?"High Pri lower":
				pVLArbTableRecord->RID.s.BlockNum == 4?"High Pri upper":
					"Unknown"
				);
	PrintVLArbTable(dest, indent, &pVLArbTableRecord->VLArbData,
				pVLArbTableRecord->RID.s.BlockNum);
}

void PrintPKeyTableRecord(PrintDest_t *dest, int indent, const IB_P_KEY_TABLE_RECORD *pPKeyTableRecord)
{
	PrintFunc(dest, "%*sLID: 0x%04x PortNum: %2u BlockNum: %2u\n",
				indent, "", pPKeyTableRecord->RID.s.LID,
				pPKeyTableRecord->RID.s.PortNum,
				pPKeyTableRecord->RID.s.BlockNum);
	PrintPKeyTable(dest, indent, &pPKeyTableRecord->PKeyTblData,
				pPKeyTableRecord->RID.s.BlockNum);
}

static char HexToChar(uint8 c)
{
	if (c >= ' ' && c <= '~')
		return (char)c;
	else
		return '.';
}

static void FormatChars(char *buf, const uint8* data, uint32 len)
{
	uint32 j;
	int offset = 0;

	for (j=0; j<len; ++j)
		offset += sprintf(&buf[offset], "%c", HexToChar(data[j]));
}

void PrintServiceRecord(PrintDest_t *dest, int indent, const IB_SERVICE_RECORD *pServiceRecord)
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

void PrintMcMemberRecord(PrintDest_t *dest, int indent, const IB_MCMEMBER_RECORD *pMcMemberRecord)
{
	char buf[8];
	PrintFunc(dest, "%*sGID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",
				pMcMemberRecord->RID.MGID.AsReg64s.H,
				pMcMemberRecord->RID.MGID.AsReg64s.L);
	PrintFunc(dest, "%*sPortGid: 0x%016"PRIx64":0x%016"PRIx64" Membership: %s%s%s\n",
				indent, "",
				pMcMemberRecord->RID.PortGID.Type.Global.SubnetPrefix,
				pMcMemberRecord->RID.PortGID.Type.Global.InterfaceID,
				pMcMemberRecord->JoinFullMember?"Full ":"",
				pMcMemberRecord->JoinNonMember?"Non ":"",
				pMcMemberRecord->JoinSendOnlyMember?"Sendonly ":"");
	FormatTimeoutMult(buf, pMcMemberRecord->PktLifeTime);
	PrintFunc(dest, "%*sMLID: 0x%08x PKey: 0x%04x Mtu: %5s Rate: %4s PktLifeTime: %s\n",
				indent, "",
				MCAST16_TO_MCAST32(pMcMemberRecord->MLID), pMcMemberRecord->P_Key,
				IbMTUToText(pMcMemberRecord->Mtu),
				IbStaticRateToText(pMcMemberRecord->Rate),
				buf);
	PrintFunc(dest, "%*sQKey: 0x%08x SL: %2u FlowLabel: 0x%05x  HopLimit: 0x%02x  TClass:  0x%02x\n",
				indent, "",
				pMcMemberRecord->Q_Key, pMcMemberRecord->u1.s.SL,
				pMcMemberRecord->u1.s.FlowLabel, pMcMemberRecord->u1.s.HopLimit,
				pMcMemberRecord->TClass);

}

void PrintInformInfo(PrintDest_t *dest, int indent, const IB_INFORM_INFO *pInformInfo)
{
	if (pInformInfo->IsGeneric)
	{
		char buf[8];
		PrintFunc(dest, "%*s%sSubcribe: Type: %8s Generic: Trap: %5d QPN: 0x%06x\n",
				indent, "",
				pInformInfo->Subscribe?"":"Un",
				IbNoticeTypeToText(pInformInfo->Type),
				pInformInfo->u.Generic.TrapNumber,
				pInformInfo->u.Generic.u2.s.QPNumber);
		FormatTimeoutMult(buf, pInformInfo->u.Generic.u2.s.RespTimeValue);
		PrintFunc(dest, "%*sRespTime: %s Producer: %s\n",
				indent, "", buf,
				StlNodeTypeToText(pInformInfo->u.Generic.u.s.ProducerType));
	} else {
		char buf[8];
		FormatTimeoutMult(buf, pInformInfo->u.Generic.u2.s.RespTimeValue);
		PrintFunc(dest, "%*s%sSubcribe: Type: %8s VendorId: 0x%04x DeviceId: 0x%04x QPN: 0x%06x\n",
				indent, "",
				pInformInfo->Subscribe?"":"Un",
				IbNoticeTypeToText(pInformInfo->Type),
				pInformInfo->u.Vendor.u.s.VendorID,
				pInformInfo->u.Vendor.DeviceID,
				pInformInfo->u.Vendor.u2.s.QPNumber);
		PrintFunc(dest, "%*sRespTime: %s\n",
				indent, "",
				buf);
	}
	PrintFunc(dest, "%*sGID: 0x%016"PRIx64":0x%016"PRIx64"  LIDs: 0x%04x - 0x%04x\n",
				indent, "",
				pInformInfo->GID.AsReg64s.H,
				pInformInfo->GID.AsReg64s.L,
				pInformInfo->LIDRangeBegin,
				pInformInfo->LIDRangeEnd);
}

void PrintInformInfoRecord(PrintDest_t *dest, int indent, const IB_INFORM_INFO_RECORD *pInformInfoRecord)
{
	PrintFunc(dest, "%*sSubGID: 0x%016"PRIx64":0x%016"PRIx64" Enum: 0x%04x\n",
				indent, "",
				pInformInfoRecord->RID.SubscriberGID.Type.Global.SubnetPrefix,
				pInformInfoRecord->RID.SubscriberGID.Type.Global.InterfaceID,
				pInformInfoRecord->RID.Enum);
	PrintInformInfo(dest, indent, &pInformInfoRecord->InformInfoData);
}
