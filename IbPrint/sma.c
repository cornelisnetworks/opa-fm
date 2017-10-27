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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iba/ibt.h>
#include <stdarg.h>
#include "ibprint.h"

void PrintNodeDesc(PrintDest_t *dest, int indent, const NODE_DESCRIPTION *pNodeDesc)
{
	PrintFunc(dest, "%*s%.*s\n",
			indent, "",
			NODE_DESCRIPTION_ARRAY_SIZE, pNodeDesc->NodeString);
}

void PrintNodeInfo(PrintDest_t *dest, int indent, const NODE_INFO *pNodeInfo)
{
	PrintFunc(dest, "%*sType: %s Ports: %d PortNum: %d PartitionCap: %d\n",
				indent, "",
				StlNodeTypeToText(pNodeInfo->NodeType), pNodeInfo->NumPorts,
				pNodeInfo->u1.s.LocalPortNum, pNodeInfo->PartitionCap);
	PrintFunc(dest, "%*sNodeGuid: 0x%016"PRIx64" PortGuid: 0x%016"PRIx64"\n",
				indent, "", pNodeInfo->NodeGUID, pNodeInfo->PortGUID);
	PrintFunc(dest, "%*sSystemImageGuid: 0x%016"PRIx64"\n",
				indent, "", pNodeInfo->SystemImageGUID);
	PrintFunc(dest, "%*sBaseVersion: %d SmaVersion: %d VendorID: 0x%x DeviceId: 0x%x Revision: 0x%x\n",
				indent, "", pNodeInfo->BaseVersion,
				pNodeInfo->ClassVersion, pNodeInfo->u1.s.VendorID,
				pNodeInfo->DeviceID, pNodeInfo->Revision);
}

// style=0 or 1
// TBD - key off portGuid!=0 to decide style instead?
void PrintPortInfo(PrintDest_t *dest, int indent, const PORT_INFO *pPortInfo, EUI64 portGuid, int style)
{
	char tempBuf[9];
	char tempBuf2[9];
	char cbuf[80];

	PrintFunc(dest, "%*sPortState: %-6s           PhysState: %-8s  DownDefault: %-5s\n",
		indent, "",
		IbPortStateToText(pPortInfo->Link.PortState),
		IbPortPhysStateToText(pPortInfo->Link.PortPhysicalState),
		IbPortDownDefaultToText(pPortInfo->Link.DownDefaultState));
	if (style == 0) {
		PrintFunc(dest, "%*sLID:    0x%04x              LMC: %u               Subnet: 0x%016"PRIx64"\n",
			indent, "",
			pPortInfo->LID, pPortInfo->s1.LMC, pPortInfo->SubnetPrefix);
	} else {
		PrintFunc(dest, "%*sLID:    0x%04x              LMC: %u\n",
			indent, "", pPortInfo->LID, pPortInfo->s1.LMC);
		if (portGuid) {
			PrintFunc(dest, "%*sSubnet: 0x%016"PRIx64"  GUID: 0x%016"PRIx64"  GUID Cap:  %4u\n",
				indent, "", pPortInfo->SubnetPrefix, portGuid, pPortInfo->GUIDCap);
		} else {
			PrintFunc(dest, "%*sSubnet: 0x%016"PRIx64"                            GUID Cap:  %4u\n",
				indent, "", pPortInfo->SubnetPrefix, pPortInfo->GUIDCap);
		}
	}
	FormatTimeoutMult(tempBuf, pPortInfo->Resp.TimeValue);
	FormatTimeoutMult(tempBuf2, pPortInfo->Subnet.Timeout);
	PrintFunc(dest, "%*sSMLID:  0x%04x   SMSL: %2u   RespTimeout: %s  SubnetTimeout: %s\n",
		indent, "",
		pPortInfo->MasterSMLID, pPortInfo->s2.MasterSMSL, tempBuf, tempBuf2);
	PrintFunc(dest, "%*sM_KEY:  0x%016"PRIx64"  Lease: %5u s          Protect: %s\n",
		indent, "", pPortInfo->M_Key, pPortInfo->M_KeyLeasePeriod,
		IbMKeyProtectToText(pPortInfo->s1.M_KeyProtectBits));
	PrintFunc(dest, "%*sMTU:       Active:   %5s  Supported:      %5s  VL Stall: %u\n",
		indent, "", IbMTUToText(pPortInfo->s2.NeighborMTU),
		IbMTUToText(pPortInfo->MTU.Cap), pPortInfo->XmitQ.VLStallCount);
	PrintFunc(dest, "%*sLinkWidth: Active:    %4s  Supported:    %7s  Enabled:    %7s\n",
		indent, "",
		IbLinkWidthToText(pPortInfo->LinkWidth.Active),
		IbLinkWidthToText(pPortInfo->LinkWidth.Supported),
		IbLinkWidthToText(pPortInfo->LinkWidth.Enabled));
	PrintFunc( dest, "%*sLinkSpeed: Active: %7s  Supported: %10s  Enabled: %10s\n",
		indent, "",
		IbLinkSpeedToText(PortDataGetLinkSpeedActiveCombined((PORT_INFO *)pPortInfo)),
		IbLinkSpeedToText(PortDataGetLinkSpeedSupportedCombined((PORT_INFO *)pPortInfo)), 
		IbLinkSpeedToText(PortDataGetLinkSpeedEnabledCombined((PORT_INFO *)pPortInfo)) );
	if (pPortInfo->XmitQ.HOQLife > IB_LIFETIME_MAX)
	{
		memcpy(tempBuf, "Infinite", 9);
	} else {
		FormatTimeoutMult(tempBuf, pPortInfo->XmitQ.HOQLife);
	}
	PrintFunc(dest, "%*sVLs:       Active:    %-2d  Supported:       %-2d  HOQLife: %s\n",
		indent, "",
		pPortInfo->s3.OperationalVL,
		pPortInfo->VL.s.Cap, tempBuf);
	PrintFunc(dest, "%*sVL Arb Cap:  High:    %4u        Low:       %4u  HiLimit: %4u\n",
		indent, "", pPortInfo->VL.ArbitrationHighCap,
		pPortInfo->VL.ArbitrationLowCap, pPortInfo->VL.HighLimit);
#if ! GUID_CAP_ALWAYS_ALONE
	if (style == 0)
#endif
		PrintFunc(dest, "%*sGUID Cap:  %4u\n", indent, "", pPortInfo->GUIDCap);

	FormatCapabilityMask(cbuf, pPortInfo->CapabilityMask);
	PrintFunc(dest, "%*sCapability 0x%08x: %s\n",
		indent, "",
		pPortInfo->CapabilityMask.AsReg32, cbuf);
	PrintFunc(dest, "%*sViolations: M_Key: %5u P_Key: %5u Q_Key: %5u\n",
		indent, "",
		pPortInfo->Violations.M_Key, pPortInfo->Violations.P_Key,
		pPortInfo->Violations.Q_Key);
	PrintFunc(dest, "%*sErrorLimits: Overrun: %2u LocalPhys: %2u  DiagCode: 0x%04x\n",
		indent, "",
		pPortInfo->Errors.Overrun, pPortInfo->Errors.LocalPhys,
		pPortInfo->DiagCode);
	PrintFunc(dest, "%*sP_Key Enforcement: In: %3s Out: %3s  FilterRaw: In: %3s Out: %3s\n",
		indent, "",
		pPortInfo->s3.PartitionEnforcementInbound?"On":"Off",
		pPortInfo->s3.PartitionEnforcementOutbound?"On":"Off",
		pPortInfo->s3.FilterRawInbound?"On":"Off",
		pPortInfo->s3.FilterRawOutbound?"On":"Off");
}

void PrintPortInfoSmp(PrintDest_t *dest, int indent, const SMP *smp, EUI64 portGuid)
{
	const PORT_INFO* pPortInfo = (PORT_INFO *)&(smp->SmpExt.DirectedRoute.SMPData);

	PrintFunc(dest, "%*sPort %u Info\n", indent, "",
				smp->common.AttributeModifier & 0xff);
	PrintPortInfo(dest, indent+4, pPortInfo, portGuid, 1);
}

void PrintSMInfo(PrintDest_t *dest, int indent, const SM_INFO *pSMInfo, IB_LID lid)
{
	if (lid)
		PrintFunc(dest, "%*sLID: 0x%04x PortGuid: 0x%016"PRIx64" State: %11s\n",
				indent, "", lid, pSMInfo->GUID,
				IbSMStateToText(pSMInfo->s.SMState));
	else
		PrintFunc(dest, "%*sPortGuid: 0x%016"PRIx64" State: %11s\n",
				indent, "", pSMInfo->GUID,
				IbSMStateToText(pSMInfo->s.SMState));
	PrintFunc(dest, "%*sSM_Key: 0x%016"PRIx64" Priority: %3d ActCount: 0x%08x\n",
				indent, "", pSMInfo->SM_Key,
				pSMInfo->s.Priority, pSMInfo->ActCount);
}

void PrintSwitchInfo(PrintDest_t *dest, int indent, const SWITCH_INFO *pSwitchInfo, IB_LID lid)
{
	char buf[8];

	if (lid)
		PrintFunc(dest, "%*sLID: 0x%04x\n", indent, "", lid);

	PrintFunc(dest, "%*sLinearFDBCap: %5u LinearFDBTop: %5u MCFDBCap: %5u MCFDBTop: %5u\n",
			indent, "",
			pSwitchInfo->LinearFDBCap, pSwitchInfo->LinearFDBTop,
			pSwitchInfo->MulticastFDBCap, pSwitchInfo->MulticastFDBTop);
	FormatTimeoutMult(buf, pSwitchInfo->u1.s.LifeTimeValue);
	PrintFunc(dest, "%*sRandomFDBCap: %5u LIDsPerPort: %5u LifeTime: %s\n",
				indent, "",
				pSwitchInfo->RandomFDBCap, pSwitchInfo->LIDsPerPort, buf);
	PrintFunc(dest, "%*sDefaultPort: %3u DefaultMCPrimaryPort: %3u DefaultMCNotPrimaryPort: %3u\n",
				indent, "",
				pSwitchInfo->DefaultPort,
				pSwitchInfo->DefaultMulticastPrimaryPort,
				pSwitchInfo->DefaultMulticastNotPrimaryPort);
	PrintFunc(dest, "%*sCapability: 0x%02x: %s%s%s%s%s%s PartEnfCap: %5u PortStateChange: %d\n",
				indent, "",
				pSwitchInfo->u2.AsReg8,
				pSwitchInfo->u2.s.InboundEnforcementCapable?"IE ": "",
				pSwitchInfo->u2.s.OutboundEnforcementCapable?"OE ": "",
				pSwitchInfo->u2.s.FilterRawPacketInboundCapable?"FI ": "",
				pSwitchInfo->u2.s.FilterRawPacketOutboundCapable?"FO ": "",
				pSwitchInfo->u2.s.EnhancedPort0?"E0 ": "",
				pSwitchInfo->u1.s.OptimizedSLVL?"SLVL ": "",
				pSwitchInfo->PartitionEnforcementCap,
				pSwitchInfo->u1.s.PortStateChange);
}

void PrintLinearFDB(PrintDest_t *dest, int indent, const FORWARDING_TABLE *pLinearFDB, uint16 blockNum)
{
	int i;
	IB_LID baselid = blockNum*LFT_BLOCK_SIZE;

	for (i=0; i<LFT_BLOCK_SIZE; ++i)
	{
		// LID is index into overall FDB table
		// 255 is invalid port, indicates an unsed table entry
		if (pLinearFDB->u.Linear.LftBlock[i] != 255)
			PrintFunc(dest, "%*s  LID 0x%04x -> Port %5u\n",
				indent, "", baselid + i,
				pLinearFDB->u.Linear.LftBlock[i]);
	}
}

void PrintRandomFDB(PrintDest_t *dest, int indent, const FORWARDING_TABLE *pRandomFDB, uint16 blockNum)
{
	int i;

	for (i=0; i<RFT_BLOCK_SIZE; ++i)
	{
		// Valid = 0 indicates unused table entry
		if (pRandomFDB->u.Random.RftBlock[i].s.Valid)
		{
			IB_LID lid = pRandomFDB->u.Random.RftBlock[i].LID;
			uint8 lmc = pRandomFDB->u.Random.RftBlock[i].s.LMC;
			if (lmc) {
				PrintFunc(dest, "%*s  LID 0x%04x-0x%04x -> Port %5u\n",
					indent, "", lid, lid + (1<<lmc)-1,
					pRandomFDB->u.Random.RftBlock[i].Port);
			} else {
				PrintFunc(dest, "%*s  LID 0x%04x -> Port %5u\n",
					indent, "", lid,
					pRandomFDB->u.Random.RftBlock[i].Port);
			}
		}
	}
}

boolean isMCastFDBEmpty(const FORWARDING_TABLE *pMCastFDB)
{
	int i;

	for (i=0; i<MFT_BLOCK_SIZE; ++i)
	{
		// PortMask = 0 indicates unused table entry
		if (pMCastFDB->u.Multicast.MftBlock[i])
			return FALSE;
	}
	return TRUE;
}

void PrintMCastFDB(PrintDest_t *dest, int indent, const FORWARDING_TABLE *pMCastFDB, uint16 blockNum)
{
	int i;
	IB_LID baselid = blockNum*MFT_BLOCK_SIZE+LID_MCAST_START;

	for (i=0; i<MFT_BLOCK_SIZE; ++i)
	{
		// PortMask = 0 indicates unused table entry
		if (pMCastFDB->u.Multicast.MftBlock[i])
		{
			PrintFunc(dest, "%*s  LID 0x%04x -> PortMask 0x%04x\n",
				indent, "", baselid+i,
				pMCastFDB->u.Multicast.MftBlock[i]);
		}
	}
}

void PrintMCastFDBSmp(PrintDest_t *dest, int indent, const SMP *smp)
{
	const FORWARDING_TABLE *pMCastFDB = (FORWARDING_TABLE *)&(smp->SmpExt.DirectedRoute.SMPData);

	PrintFunc(dest, "%*sPosition: %2u\n", indent, "",
				smp->common.AttributeModifier >> 28);

	PrintMCastFDB(dest, indent, pMCastFDB, smp->common.AttributeModifier & 0x1ff);
}

boolean isMCastFDBEmptyRows(const FORWARDING_TABLE *pMCastFDB, uint8 numPositions)
{
	int i;
	unsigned p;

	for (i=0; i<MFT_BLOCK_SIZE; ++i)
	{
		for (p=0; p<numPositions; ++p) {
			// PortMask = 0 indicates unused table entry
			if (pMCastFDB[p].u.Multicast.MftBlock[i])
				return FALSE;
		}
	}
	return TRUE;
}

// prints full rows in table
// pMcastFDB expected to be FORWARDING_TABLE[1+endPosition-startPosition] entry
void PrintMCastFDBRows(PrintDest_t *dest, int indent, const FORWARDING_TABLE *pMCastFDB, uint16 blockNum, uint8 startPosition, uint8 endPosition, boolean showHeading)
{
	int i;
	unsigned p;
	IB_LID baselid = blockNum*MFT_BLOCK_SIZE+LID_MCAST_START;

	if (showHeading)
		PrintFunc(dest, "%*sPositions: %2u - %2u\n", indent, "",
				startPosition, endPosition);
	for (i=0; i<MFT_BLOCK_SIZE; ++i)
	{
		// see if given MLID has a non-zero mask
		boolean empty = TRUE;
		for (p=startPosition; p<=endPosition; ++p) {
			// PortMask = 0 indicates unused table entry
			if (pMCastFDB[p-startPosition].u.Multicast.MftBlock[i])
			{
				empty = FALSE;
			}
		}
		if (! empty) {
			char buf[80];
			int offset;
			offset = sprintf(buf, "  LID 0x%04x -> PortMask 0x", baselid+i);
			// show the combined mask, MSb to LSb
			p=endPosition;
			do {
				offset += sprintf(&buf[offset], "%04x", pMCastFDB[p-startPosition].u.Multicast.MftBlock[i]);
			} while (p-- != startPosition);
			PrintFunc(dest, "%*s%s\n", indent, "", buf);

			// show the list of ports that Mask corresponds to
			offset = sprintf(buf, "                Ports:");
			for (p=startPosition; p<=endPosition; ++p) {
				PORTMASK mask = pMCastFDB[p-startPosition].u.Multicast.MftBlock[i];
				if (mask) {
					uint8 bit;
					for (bit=0; bit<MFT_BLOCK_WIDTH; ++bit) {
						if (mask & (1<<bit)) {
							if (offset > 70) {
								PrintFunc(dest, "%*s%s\n", indent, "", buf);
								offset = sprintf(buf, "                      ");
							}
							offset += sprintf(&buf[offset], " %3u", p*MFT_BLOCK_WIDTH+bit);
						}
					}
				}
			}
			PrintFunc(dest, "%*s%s\n", indent, "", buf);
		}
	}
}

void PrintVLArbTable(PrintDest_t *dest, int indent, const VLARBTABLE *pVLArbTable, uint8 blockNum)
{
	uint8 base_index = 0;
	int i;

	if (blockNum == 2 || blockNum == 4)
		base_index = IB_VLARBTABLE_SIZE;
	PrintFunc(dest, "%*s  Indx VL Weight\n", indent, "");
	for (i=0; i<IB_VLARBTABLE_SIZE; ++i)
	{
		// zero weight is a disabled entry
		if (pVLArbTable->ArbTable[i].Weight)
		{
			PrintFunc(dest, "%*s   %2d  %2u  %3u\n",
				indent, "", base_index+i,
				pVLArbTable->ArbTable[i].s.VL,
				pVLArbTable->ArbTable[i].Weight);
		}
	}
}

void PrintVLArbTableSmp(PrintDest_t *dest, int indent, const SMP *smp, NODE_TYPE nodetype)
{
	VLARBTABLE *pVLArbTable = (VLARBTABLE *)&(smp->SmpExt.DirectedRoute.SMPData);
	char buf[40] = "";
	uint32 block = smp->common.AttributeModifier >> 16;

	if (nodetype == STL_NODE_SW) {

		sprintf(buf, "OutputPort: %3u ", smp->common.AttributeModifier  & 0xffff);
	}
	PrintFunc(dest, "%*s%sBlockNum: %2u (%s)\n", indent, "",
					buf, block,
					block == 1?"Low Pri lower":
					block == 2?"Low Pri upper":
					block == 3?"High Pri lower":
					block == 4?"High Pri upper":
					"Unknown"
					);
	PrintVLArbTable(dest, indent, pVLArbTable, block);
}

void PrintPKeyTable(PrintDest_t *dest, int indent, const PARTITION_TABLE *pPKeyTable, uint16 blockNum)
{
	uint16 i;
	char buf[81];
	int offset=0;
	uint16 index = blockNum*PARTITION_TABLE_BLOCK_SIZE;

	// 8 entries per line
	for (i=0; i<PARTITION_TABLE_BLOCK_SIZE; ++i)
	{
		if (i%8 == 0) {
			if (i)
				PrintFunc(dest, "%*s%s\n", indent, "", buf);
			offset=sprintf(buf, "   %4u-%4u:", i+index, i+7+index);
		}
		offset+=sprintf(&buf[offset], "  0x%04x", pPKeyTable->PartitionTableBlock[i].AsInt16);
	}
	PrintFunc(dest, "%*s%s\n", indent, "", buf);
}

void PrintPKeyTableSmp(PrintDest_t *dest, int indent, const SMP *smp, NODE_TYPE nodetype, boolean showHeading, boolean showBlock)
{
	PARTITION_TABLE *pPKeyTable = (PARTITION_TABLE *)&(smp->SmpExt.DirectedRoute.SMPData);
	char buf[40] = "";
	uint32 block = smp->common.AttributeModifier & 0xffff;

	if (showHeading) {
		if (nodetype == STL_NODE_SW)
			sprintf(buf, "OutputPort: %3u ", smp->common.AttributeModifier >> 16);
		if (showBlock)
			PrintFunc(dest, "%*s%sBlockNum: %2u\n", indent, "", buf, block);
		else if (buf[0])
			PrintFunc(dest, "%*s%s\n", indent, "", buf);
	}
	PrintPKeyTable(dest, indent, pPKeyTable, block);
}

void PrintGuidInfo(PrintDest_t *dest, int indent, const GUID_INFO *pGuidInfo, uint8 blockNum)
{
	int i;
	char buf[81];
	int offset=0;
	uint16 index = blockNum*GUID_INFO_BLOCK_SIZE;

	// 2 entries per line
	for (i=0; i<GUID_INFO_BLOCK_SIZE; ++i)
	{
		if (i%2 == 0) {
			if (i)
				PrintFunc(dest, "%*s%s\n", indent, "", buf);
			offset=sprintf(buf, "   %4u-%4u:", i+index, i+1+index);
		}
		offset+=sprintf(&buf[offset], "  0x%016"PRIx64"", pGuidInfo->Gid[i]);
	}
	PrintFunc(dest, "%*s%s\n", indent, "", buf);
}

void PrintPortGroupTable(PrintDest_t *dest, int indent, const PORT_GROUP_TABLE *pPortGroup, uint16 tier, uint16 blockNum)
{
	int i;
	uint baseport = (blockNum & 0xff)*PGTABLE_LIST_COUNT;

	PrintFunc(dest, "%*s Adaptive Routing Port Group (Tier= %d):\n", indent, "", tier);
	for (i=0; i<PGTABLE_LIST_COUNT; ++i)
	{
		// zero is invalid port group id, indicates an unused table entry
		if (pPortGroup->PortGroup[i] != 0) {
			PrintFunc(dest, "%*s  Port %5u -> PortGroup %5u\n",
				indent, "", baseport + i,
				pPortGroup->PortGroup[i]);
		}
	}
}

void PrintAdaptiveRoutingLidmask(PrintDest_t *dest, int indent, const ADAPTIVE_ROUTING_LIDMASK *pLidmask, uint16 blockNum)
{
	char	string[256];
	char	str[32];
	int		i,j;
	uint8_t	lidmask;
 	IB_LID	curlid;
 	IB_LID  baselid = blockNum*(LIDMASK_LEN*8);
	int		firstdone = 0;
	int		first = 0;
	int		next = 0;

	string[0] = '\0';
	for (i=LIDMASK_LEN-1; i>=0; i--)
	{ 
		if (pLidmask->lidmask[i]) {
			lidmask = pLidmask->lidmask[i];
			for (j=0; j<8; j++) {
				if (lidmask & (1<<j)) {
					if (strlen(string) > 80) {
						strcat(string, ",");
						PrintFunc(dest, "%*s %s\n", indent, "", string);
						string[0] = '\0';
						firstdone = 0;
					}
					curlid = baselid+j;
					if (!first) {
						first = curlid;
					} else if (!next) {
						if (curlid - first > 1) {
							if (!firstdone) {
								sprintf(str, "0x%x", first);
								firstdone = 1;
							} else {
                                sprintf(str, ",0x%x", first);
							}
							strcat(string, str);
							first = curlid;
						} else {
							next = curlid;
						}
					} else {
						if (curlid - next > 1) {
							if (!firstdone) {
								sprintf(str, "0x%x-0x%x", first, next);
								firstdone = 1;
							} else {
								sprintf(str, ",0x%x-0x%x", first, next);
							}
							strcat(string, str);
							first = curlid;
							next = 0;
						} else {
							next = curlid;
						}
					}
				}
			}
		}
		baselid+=8;
	}

	if (first) {
		if (!firstdone) {
			sprintf(str, "0x%x", first);
			strcat(string, str);
		} else {
			sprintf(str, ",0x%x", first);
			strcat(string, str);
		}
		if (next) {
			sprintf(str, "-0x%x", next);
			strcat(string, str);
		}
	}
	if (strlen(string) > 0) {
		PrintFunc(dest, "%*s %s\n", indent, "", string);
	}
}

void PrintSmp(PrintDest_t *dest, int indent, const SMP *smp)
{
	PrintSmpHeader(dest, indent, smp);
	switch (smp->common.AttributeID) {
	case MCLASS_ATTRIB_ID_NODE_DESCRIPTION:
		PrintNodeDesc(dest, indent, (const NODE_DESCRIPTION *)&smp->SmpExt.DirectedRoute.SMPData);
		break;
	case MCLASS_ATTRIB_ID_NODE_INFO:
		PrintNodeInfo(dest, indent, (const NODE_INFO *)&smp->SmpExt.DirectedRoute.SMPData);
		break;
	case MCLASS_ATTRIB_ID_SWITCH_INFO:
		PrintSwitchInfo(dest, indent, (const SWITCH_INFO *)&smp->SmpExt.DirectedRoute.SMPData, 0);
		break;
	case MCLASS_ATTRIB_ID_GUID_INFO:
		PrintGuidInfo(dest, indent, (const GUID_INFO *)&smp->SmpExt.DirectedRoute.SMPData, smp->common.AttributeModifier);
		break;
	case MCLASS_ATTRIB_ID_PORT_INFO:
		PrintPortInfoSmp(dest, indent, smp, 0);
		break;
	case MCLASS_ATTRIB_ID_PART_TABLE:
		PrintPKeyTableSmp(dest, indent, smp,
			(smp->common.AttributeModifier >> 16)?STL_NODE_SW:0, TRUE, TRUE);
		break;
	case MCLASS_ATTRIB_ID_VL_ARBITRATION:
		PrintVLArbTableSmp(dest, indent, smp, 
			(smp->common.AttributeModifier &0xffff)?STL_NODE_SW:0);
		break;
	case MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE:
		PrintLinearFDB(dest, indent,
				(const FORWARDING_TABLE *)&smp->SmpExt.DirectedRoute.SMPData,
				smp->common.AttributeModifier & 0xffff);
		break;
	case MCLASS_ATTRIB_ID_RANDOM_FWD_TABLE:
		PrintRandomFDB(dest, indent,
				(const FORWARDING_TABLE *)&smp->SmpExt.DirectedRoute.SMPData,
				smp->common.AttributeModifier);
		break;
	case MCLASS_ATTRIB_ID_MCAST_FWD_TABLE:
		PrintMCastFDBSmp(dest, indent, smp);
		break;
	case MCLASS_ATTRIB_ID_SM_INFO:
		PrintSMInfo(dest, indent, (const SM_INFO *)&smp->SmpExt.DirectedRoute.SMPData, 0);
		break;
	case MCLASS_ATTRIB_ID_VENDOR_DIAG:
		// TBD Hex Dump
		break;
	case MCLASS_ATTRIB_ID_LED_INFO:
		// TBD Hex Dump
		break;
	case MCLASS_ATTRIB_ID_PORT_LFT:
		PrintLinearFDB(dest, indent,
				(const FORWARDING_TABLE *)&smp->SmpExt.DirectedRoute.SMPData,
				smp->common.AttributeModifier & 0xffff);
		break;
	case MCLASS_ATTRIB_ID_PORT_GROUP:
		PrintPortGroupTable(dest, indent, 
			(const PORT_GROUP_TABLE *)&smp->SmpExt.DirectedRoute.SMPData,
			smp->common.AttributeModifier >>16,
			smp->common.AttributeModifier & 0xffff);
		break;
	case MCLASS_ATTRIB_ID_AR_LIDMASK:
		PrintAdaptiveRoutingLidmask(dest, indent, 
			(const ADAPTIVE_ROUTING_LIDMASK *)&smp->SmpExt.DirectedRoute.SMPData,
			//smp->common.AttributeModifier >>17,
			smp->common.AttributeModifier & 0xffff);
		break;
	default:
		// TBD Hex Dump
		break;
	}
}
