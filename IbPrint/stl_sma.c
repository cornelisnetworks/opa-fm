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

#include <arpa/inet.h>

#include "stl_print.h"
#include <iba/stl_helper.h>

void PrintStlLid(PrintDest_t *dest, int indent, STL_LID lid, int printLineByLine)
{
    if (printLineByLine) 
        PrintIntWithDots(dest, indent, "Lid", lid);
    else
        PrintFunc(dest, "%*s0x%08x\n", indent, "", lid);
}

// PrintIntWithDots is a local function used when outputing structures in line-by-line format
void PrintIntWithDots(PrintDest_t *dest, int indent, const char * name, uint64_t value)
{
	//            0123456789012345678901234567890123456789012345678901234567890  
    char pad[] = ".............................................................";
    char dataFormat[] = "%*s%s%.*s : 0x%"PRIx64"\n";
	size_t maxDotColumn = 60;
#ifndef __VXWORKS__
	int padLen = maxDotColumn-strnlen((const char *)name, maxDotColumn);
#else
    size_t strlength = min(strlen((const char *)name), maxDotColumn);
	int padLen = maxDotColumn-strlength;
#endif
	PrintFunc(dest, (const char *)dataFormat, indent, "", name, padLen, pad, value);
}

// PrintIntWithDotsFull is a local function used when outputing structures in line-by-line format
void PrintIntWithDotsFull(PrintDest_t *dest, int indent, const char * name, uint64_t value)
{
	//            0123456789012345678901234567890123456789012345678901234567890  
    char pad[] = ".............................................................";
    char dataFormat[] = "%*s%s%.*s : 0x%016"PRIx64"\n"; //add padding for full width
	size_t maxDotColumn = 60;
#ifndef __VXWORKS__
	int padLen = maxDotColumn-strnlen((const char *)name, maxDotColumn);
#else
    size_t strlength = min(strlen((const char *)name), maxDotColumn);
	int padLen = maxDotColumn-strlength;
#endif
	PrintFunc(dest, (const char *)dataFormat, indent, "", name, padLen, pad, value);
}


// PrintIntWithDotsDec is a local function used when outputing structures in line-by-line format
void PrintIntWithDotsDec(PrintDest_t *dest, int indent, const char * name, uint64_t value)
{
	//            0123456789012345678901234567890123456789012345678901234567890  
    char pad[] = ".............................................................";
    char dataFormat[] = "%*s%s%.*s : %"PRIu64"\n";
	size_t maxDotColumn = 60;
#ifndef __VXWORKS__
	int padLen = maxDotColumn-strnlen((const char *)name, maxDotColumn);
#else
    size_t strlength = min(strlen((const char *)name), maxDotColumn);
	int padLen = maxDotColumn-strlength;
#endif
	PrintFunc(dest, (const char *)dataFormat, indent, "", name, padLen, pad, value);
}

// PrintStrWithDots is a local function used when outputing structures in line-by-line format
void PrintStrWithDots(PrintDest_t *dest, int indent, const char * name, const char * value)
{
	//            0123456789012345678901234567890123456789012345678901234567890  
    char pad[] = ".............................................................";
    char dataFormat[] = "%*s%s%.*s : %s\n";
	size_t maxDotColumn = 60;
#ifndef __VXWORKS__
	int padLen = maxDotColumn-strnlen((const char *)name, maxDotColumn);
#else
    size_t strlength = min(strlen((const char *)name), maxDotColumn);
	int padLen = maxDotColumn-strlength;
#endif
	PrintFunc(dest, (const char *)dataFormat, indent, "", name, padLen, pad, value);
}

void PrintStlNodeDesc(PrintDest_t *dest, int indent, const STL_NODE_DESCRIPTION *pStlNodeDesc, int printLineByLine)
{
    if (printLineByLine) {
        PrintStrWithDots(dest, indent, "NodeString", (const char *)pStlNodeDesc->NodeString);
    }
    else {
#ifndef __VXWORKS__
        size_t strlength = strnlen((const char *)pStlNodeDesc->NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);
#else
        size_t strlength = min(strlen((const char *)pStlNodeDesc->NodeString), STL_NODE_DESCRIPTION_ARRAY_SIZE);
#endif
        PrintFunc(dest, "%*s%.*s\n",
			  indent, "",
			  (int)strlength,
			  pStlNodeDesc->NodeString);
    }
}

void PrintStlNodeInfo(PrintDest_t *dest, int indent, const STL_NODE_INFO *pNodeInfo, int printLineByLine)
{
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "BaseVersion", pNodeInfo->BaseVersion);
        PrintIntWithDots(dest, indent, "ClassVersion", pNodeInfo->ClassVersion);
        PrintStrWithDots(dest, indent, "NodeType", StlNodeTypeToText(pNodeInfo->NodeType));
        PrintIntWithDots(dest, indent, "NumPorts", pNodeInfo->NumPorts);
        PrintIntWithDotsFull(dest, indent, "SystemImageGUID", pNodeInfo->SystemImageGUID);
        PrintIntWithDotsFull(dest, indent, "NodeGUID", pNodeInfo->NodeGUID);
        PrintIntWithDotsFull(dest, indent, "PortGUID", pNodeInfo->PortGUID);
        PrintIntWithDots(dest, indent, "PartitionCap", pNodeInfo->PartitionCap);
        PrintIntWithDots(dest, indent, "DeviceID", pNodeInfo->DeviceID);
        PrintIntWithDots(dest, indent, "Revision", pNodeInfo->Revision);
        PrintIntWithDots(dest, indent, "u1.AsReg32", pNodeInfo->u1.AsReg32);
        PrintIntWithDots(dest, indent, "u1.s.LocalPortNum", pNodeInfo->u1.s.LocalPortNum);
        PrintIntWithDots(dest, indent, "u1.s.VendorID", pNodeInfo->u1.s.VendorID);
    }
    else {
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
}

void PrintStlPortInfo(PrintDest_t *dest, int indent, const STL_PORT_INFO *pPortInfo, EUI64 portGuid, int printLineByLine)
{
	// Note: This output is used by a number of commands, including opaportinfo.
	// By design, there are two formats. The default format (printLineByLine = 0) puts the 
	// most useful information first, similar to what the IB command did.
	// In general, format 0 is patterned after the legacy PrintPortInfo used for IB.
	// The second format (printLineByLine = 1) dumps the record in a line-by-line format
	// more suitable for record content analysis. 
	if (printLineByLine) {
		PrintIntWithDots(dest, indent, "LID", (uint32)pPortInfo->LID);
		PrintIntWithDots(dest, indent, "FlowControlMask", (uint32)pPortInfo->FlowControlMask);

		if (pPortInfo->CapabilityMask3.s.VLSchedulingConfig == VL_SCHED_MODE_VLARB)
			PrintIntWithDots(dest, indent, "VL.PreemptCap", (uint32)pPortInfo->VL.PreemptCap);
		PrintIntWithDots(dest, indent, "VL.s2.Cap", (uint32)pPortInfo->VL.s2.Cap);
		if (pPortInfo->CapabilityMask3.s.VLSchedulingConfig == VL_SCHED_MODE_VLARB) {
			PrintIntWithDots(dest, indent, "VL.HighLimit", (uint32)pPortInfo->VL.HighLimit);
			PrintIntWithDots(dest, indent, "VL.PreemptingLimit", (uint32)pPortInfo->VL.PreemptingLimit);
			PrintIntWithDots(dest, indent, "VL.ArbitrationHighCap", (uint32)pPortInfo->VL.ArbitrationHighCap);
			PrintIntWithDots(dest, indent, "VL.ArbitrationLowCap", (uint32)pPortInfo->VL.ArbitrationLowCap);
		}
        // STL_PORT_STATES
		PrintIntWithDots(dest, indent, "PortStates.AsReg32", (uint32)pPortInfo->PortStates.AsReg32);
		PrintIntWithDots(dest, indent, "PortStates.s.IsSMConfigurationStarted", (uint32)pPortInfo->PortStates.s.IsSMConfigurationStarted);
		PrintIntWithDots(dest, indent, "PortStates.s.NeighborNormal", (uint32)pPortInfo->PortStates.s.NeighborNormal);
		PrintIntWithDots(dest, indent, "PortStates.s.OfflineDisabledReason", (uint32)pPortInfo->PortStates.s.OfflineDisabledReason);
		PrintIntWithDots(dest, indent, "PortStates.s.PortPhysicalState", (uint32)pPortInfo->PortStates.s.PortPhysicalState);
		PrintIntWithDots(dest, indent, "PortStates.s.PortState", (uint32)pPortInfo->PortStates.s.PortState);
		PrintIntWithDots(dest, indent, "PortStates.s.LEDEnabled", (uint32)pPortInfo->PortStates.s.LEDEnabled);
		PrintIntWithDots(dest, indent, "PortPhysConfig.AsReg8", pPortInfo->PortPhysConfig.AsReg8);
		PrintIntWithDots(dest, indent, "PortPhysConfig.s.PortType", pPortInfo->PortPhysConfig.s.PortType);
		PrintIntWithDots(dest, indent, "BundleNextPort", (uint32)pPortInfo->BundleNextPort);
		PrintIntWithDots(dest, indent, "BundleLane", (uint32)pPortInfo->BundleLane);
		PrintIntWithDots(dest, indent, "MultiCollectMask.CollectiveMask", (uint32)pPortInfo->MultiCollectMask.CollectiveMask);
		PrintIntWithDots(dest, indent, "MultiCollectMask.MulticastMask", (uint32)pPortInfo->MultiCollectMask.MulticastMask);
		PrintIntWithDots(dest, indent, "s1.M_KeyProtectBits", (uint32)pPortInfo->s1.M_KeyProtectBits);
		PrintIntWithDots(dest, indent, "s1.LMC", (uint32)pPortInfo->s1.LMC);
		PrintIntWithDots(dest, indent, "s2.MasterSMSL", (uint32)pPortInfo->s2.MasterSMSL);
		PrintIntWithDots(dest, indent, "s3.PartitionEnforcementInbound", (uint32)pPortInfo->s3.PartitionEnforcementInbound);
		PrintIntWithDots(dest, indent, "s3.PartitionEnforcementOutbound", (uint32)pPortInfo->s3.PartitionEnforcementOutbound);
		PrintIntWithDots(dest, indent, "s4.OperationalVL", (uint32)pPortInfo->s4.OperationalVL);
		PrintIntWithDots(dest, indent, "P_Keys.P_Key_8B", (uint32)pPortInfo->P_Keys.P_Key_8B);
		PrintIntWithDots(dest, indent, "P_Keys.P_Key_10B", (uint32)pPortInfo->P_Keys.P_Key_10B);
		PrintIntWithDots(dest, indent, "Violations.M_Key", (uint32)pPortInfo->Violations.M_Key);
		PrintIntWithDots(dest, indent, "Violations.P_Key", (uint32)pPortInfo->Violations.P_Key);
		PrintIntWithDots(dest, indent, "Violations.Q_Key", (uint32)pPortInfo->Violations.Q_Key);
		PrintIntWithDots(dest, indent, "SM_TrapQP.QueuePair", (uint32)pPortInfo->SM_TrapQP.s.QueuePair);
		PrintIntWithDots(dest, indent, "SA_QP.QueuePair", (uint32)pPortInfo->SA_QP.s.QueuePair);
		PrintIntWithDots(dest, indent, "NeighborPortNum", (uint32)pPortInfo->NeighborPortNum);
		PrintIntWithDots(dest, indent, "LinkDownReason", (uint32)pPortInfo->LinkDownReason);
		PrintIntWithDots(dest, indent, "NeighborLinkDownReason", (uint32)pPortInfo->NeighborLinkDownReason);
		PrintIntWithDots(dest, indent, "LinkInitReason", (uint32)pPortInfo->s3.LinkInitReason);
		PrintIntWithDots(dest, indent, "Subnet.ClientReregister", (uint32)pPortInfo->Subnet.ClientReregister);
		PrintIntWithDots(dest, indent, "Subnet.MulticastPKeyTrapSuppressionEnabled", (uint32)pPortInfo->Subnet.MulticastPKeyTrapSuppressionEnabled);
		PrintIntWithDots(dest, indent, "Subnet.Timeout", (uint32)pPortInfo->Subnet.Timeout);
		// STL_LINK_SPEED
		PrintIntWithDots(dest, indent, "LinkSpeed.Supported", (uint32)pPortInfo->LinkSpeed.Supported); 
		PrintIntWithDots(dest, indent, "LinkSpeed.Enabled", (uint32)pPortInfo->LinkSpeed.Enabled);
		PrintIntWithDots(dest, indent, "LinkSpeed.Active", (uint32)pPortInfo->LinkSpeed.Active);
		// STL_LINK_WIDTH
		PrintIntWithDots(dest, indent, "LinkWidth.Supported", (uint32)pPortInfo->LinkWidth.Supported);
		PrintIntWithDots(dest, indent, "LinkWidth.Enabled", (uint32)pPortInfo->LinkWidth.Enabled);
		PrintIntWithDots(dest, indent, "LinkWidth.Active", (uint32)pPortInfo->LinkWidth.Active);
        // STL_LINK_DOWNGRADE
		PrintIntWithDots(dest, indent, "LinkWidthDowngrade.Supported", (uint32)pPortInfo->LinkWidthDowngrade.Supported);
		PrintIntWithDots(dest, indent, "LinkWidthDowngrade.Enabled", (uint32)pPortInfo->LinkWidthDowngrade.Enabled);
		PrintIntWithDots(dest, indent, "LinkWidthDowngrade.TxActive", (uint32)pPortInfo->LinkWidthDowngrade.TxActive);
		PrintIntWithDots(dest, indent, "LinkWidthDowngrade.RxActive", (uint32)pPortInfo->LinkWidthDowngrade.RxActive);
		// STL_PORT_LINK_MODE
		PrintIntWithDots(dest, indent, "PortLinkMode.AsReg16", (uint32)pPortInfo->PortLinkMode.AsReg16);
		PrintIntWithDots(dest, indent, "PortLinkMode.s.Supported", (uint32)pPortInfo->PortLinkMode.s.Supported);
		PrintIntWithDots(dest, indent, "PortLinkMode.s.Enabled", (uint32)pPortInfo->PortLinkMode.s.Enabled);
		PrintIntWithDots(dest, indent, "PortLinkMode.s.Active", (uint32)pPortInfo->PortLinkMode.s.Active);
		// STL_PORT_LTP_CRC_MODE
		PrintIntWithDots(dest, indent, "PortLTPCRCMode.AsReg16", (uint32)pPortInfo->PortLTPCRCMode.AsReg16);
		PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Supported", (uint32)pPortInfo->PortLTPCRCMode.s.Supported);
		PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Enabled", (uint32)pPortInfo->PortLTPCRCMode.s.Enabled);
		PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Active", (uint32)pPortInfo->PortLTPCRCMode.s.Active);

		PrintIntWithDots(dest, indent, "PortMode.AsReg16", (uint32)pPortInfo->PortMode.AsReg16);
		PrintIntWithDots(dest, indent, "PortMode.s.IsActiveOptimizeEnabled", (uint32)pPortInfo->PortMode.s.IsActiveOptimizeEnabled);
		PrintIntWithDots(dest, indent, "PortMode.s.IsPassThroughEnabled", (uint32)pPortInfo->PortMode.s.IsPassThroughEnabled);
		PrintIntWithDots(dest, indent, "PortMode.s.IsVLMarkerEnabled", (uint32)pPortInfo->PortMode.s.IsVLMarkerEnabled);
		PrintIntWithDots(dest, indent, "PortMode.s.Is16BTrapQueryEnabled", (uint32)pPortInfo->PortMode.s.Is16BTrapQueryEnabled);
		// STL_PORT_PACKET_FORMAT
		PrintIntWithDots(dest, indent, "PortPacketFormats.Supported", (uint32)pPortInfo->PortPacketFormats.Supported);
		PrintIntWithDots(dest, indent, "PortPacketFormats.Enabled", (uint32)pPortInfo->PortPacketFormats.Enabled);
		PrintIntWithDots(dest, indent, "FlitControl.Interleave.AsReg16", (uint32)pPortInfo->FlitControl.Interleave.AsReg16);
		PrintIntWithDots(dest, indent, "FlitControl.Interleave.s.DistanceSupported", (uint32)pPortInfo->FlitControl.Interleave.s.DistanceSupported);
		PrintIntWithDots(dest, indent, "FlitControl.Interleave.s.DistanceEnabled", (uint32)pPortInfo->FlitControl.Interleave.s.DistanceEnabled);
		PrintIntWithDots(dest, indent, "FlitControl.Interleave.s.MaxNestLevelTxEnabled", (uint32)pPortInfo->FlitControl.Interleave.s.MaxNestLevelTxEnabled);
		PrintIntWithDots(dest, indent, "FlitControl.Interleave.s.MaxNestLevelRxSupported", (uint32)pPortInfo->FlitControl.Interleave.s.MaxNestLevelRxSupported);
		// Convert Flits to bytes and display in decimal
		PrintIntWithDotsDec(dest, indent, "FlitControl.Preemption.MinInitial", (uint32)(pPortInfo->FlitControl.Preemption.MinInitial * BYTES_PER_FLIT));
		// Convert Flits to bytes and display in decimal
		PrintIntWithDotsDec(dest, indent, "FlitControl.Preemption.MinTail", (uint32)(pPortInfo->FlitControl.Preemption.MinTail * BYTES_PER_FLIT));
		PrintIntWithDots(dest, indent, "FlitControl.Preemption.LargePktLimit", (uint32)pPortInfo->FlitControl.Preemption.LargePktLimit);
		PrintIntWithDots(dest, indent, "FlitControl.Preemption.SmallPktLimit", (uint32)pPortInfo->FlitControl.Preemption.SmallPktLimit);
		PrintIntWithDots(dest, indent, "FlitControl.Preemption.MaxSmallPktLimit", (uint32)pPortInfo->FlitControl.Preemption.MaxSmallPktLimit);
		PrintIntWithDots(dest, indent, "FlitControl.Preemption.PreemptionLimit", (uint32)pPortInfo->FlitControl.Preemption.PreemptionLimit);

		if (pPortInfo->CapabilityMask3.s.IsMAXLIDSupported) {
			PrintIntWithDots(dest, indent, "MaxLID", (uint32)pPortInfo->MaxLID);
		}

		PrintIntWithDots(dest, indent, "PortErrorAction.AsReg32", (uint32)pPortInfo->PortErrorAction.AsReg32);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.ExcessiveBufferOverrun", (uint32)pPortInfo->PortErrorAction.s.ExcessiveBufferOverrun);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorExceedMulticastLimit", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorExceedMulticastLimit);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorBadControlFlit", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorBadControlFlit);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorBadPreempt", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorBadPreempt);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorUnsupportedVLMarker", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorUnsupportedVLMarker);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorBadCrdtAck", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorBadCrdtAck);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorBadCtrlDist", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorBadCtrlDist);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorBadTailDist", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorBadTailDist);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.FmConfigErrorBadHeadDist", (uint32)pPortInfo->PortErrorAction.s.FmConfigErrorBadHeadDist);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadVLMarker", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadVLMarker);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorPreemptVL15", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorPreemptVL15);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorPreemptError", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorPreemptError);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadMidTail", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadMidTail);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorReserved", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorReserved);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadSC", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadSC);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadL2", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadL2);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadDLID", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadDLID);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadSLID", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadSLID);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorPktLenTooShort", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorPktLenTooShort);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorPktLenTooLong", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorPktLenTooLong);
		PrintIntWithDots(dest, indent, "PortErrorAction.s.RcvErrorBadPktLen", (uint32)pPortInfo->PortErrorAction.s.PortRcvErrorBadPktLen);
		PrintIntWithDots(dest, indent, "PassThroughControl.EgressPort", (uint32)pPortInfo->PassThroughControl.EgressPort);
		PrintIntWithDots(dest, indent, "PassThroughControl.DRControl", (uint32)pPortInfo->PassThroughControl.DRControl);
		PrintIntWithDots(dest, indent, "M_KeyLeasePeriod", (uint32)pPortInfo->M_KeyLeasePeriod);
		PrintIntWithDots(dest, indent, "BufferUnits.AsReg32", (uint32)pPortInfo->BufferUnits.AsReg32);
		PrintIntWithDots(dest, indent, "BufferUnits.s.VL15Init", (uint32)pPortInfo->BufferUnits.s.VL15Init);
		PrintIntWithDots(dest, indent, "BufferUnits.s.VL15CreditRate", (uint32)pPortInfo->BufferUnits.s.VL15CreditRate);
		PrintIntWithDots(dest, indent, "BufferUnits.s.CreditAck", (uint32)pPortInfo->BufferUnits.s.CreditAck);
		PrintIntWithDots(dest, indent, "BufferUnits.s.BufferAlloc", (uint32)pPortInfo->BufferUnits.s.BufferAlloc);
		PrintIntWithDots(dest, indent, "MasterSMLID", (uint32)pPortInfo->MasterSMLID);
		PrintIntWithDots(dest, indent, "M_Key", pPortInfo->M_Key);
		PrintIntWithDots(dest, indent, "SubnetPrefix", pPortInfo->SubnetPrefix);
        int i; 
        char NeighBorMTUbuf[64];
        for (i = 0; i < STL_MAX_VLS/2; i++) {
            snprintf(NeighBorMTUbuf, sizeof(NeighBorMTUbuf), "MTUActive[%02d].STL_VL_TO_MTU.AsReg8", i);
            PrintIntWithDots(dest, indent, NeighBorMTUbuf, (uint32)pPortInfo->NeighborMTU[i].AsReg8);
            snprintf(NeighBorMTUbuf, sizeof(NeighBorMTUbuf), "MTUActive[%02d].STL_VL_TO_MTU.s.VL0_to_MTU", i);
            PrintIntWithDots(dest, indent, NeighBorMTUbuf, (uint32)pPortInfo->NeighborMTU[i].s.VL0_to_MTU);
            snprintf(NeighBorMTUbuf, sizeof(NeighBorMTUbuf), "MTUActive[%02d].STL_VL_TO_MTU.s.VL1_to_MTU", i);
            PrintIntWithDots(dest, indent, NeighBorMTUbuf, (uint32)pPortInfo->NeighborMTU[i].s.VL1_to_MTU);
        }
        char XmitQbuf[64];
		for (i = 0; i < STL_MAX_VLS; i++) {
            snprintf(XmitQbuf, sizeof(XmitQbuf), "XmitQ[%02d].VLStallCount", i);
            PrintIntWithDots(dest, indent, XmitQbuf, (uint32)pPortInfo->XmitQ[i].VLStallCount);
            snprintf(XmitQbuf, sizeof(XmitQbuf), "XmitQ[%02d].HOQLife", i);
            PrintIntWithDots(dest, indent, XmitQbuf, (uint32)pPortInfo->XmitQ[i].HOQLife);
		}
		char tempBuf[64];
		PrintStrWithDots(dest, indent, "IPAddrIPV6.addr",
			inet_ntop(AF_INET6, pPortInfo->IPAddrIPV6.addr, tempBuf, sizeof(tempBuf)));
		PrintStrWithDots(dest, indent, "IPAddrIPV4.addr",
			inet_ntop(AF_INET, pPortInfo->IPAddrIPV4.addr, tempBuf, sizeof(tempBuf)));
        PrintIntWithDots(dest, indent, "NeighborNodeGUID", pPortInfo->NeighborNodeGUID);

		PrintIntWithDots(dest, indent, "CapabilityMask.AsReg32", (uint32)pPortInfo->CapabilityMask.AsReg32);
		PrintIntWithDots(dest, indent, "CapabilityMask.s.IsCapabilityMaskNoticeSupported", (uint32)pPortInfo->CapabilityMask.s.IsCapabilityMaskNoticeSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask.s.IsVendorClassSupported", (uint32)pPortInfo->CapabilityMask.s.IsVendorClassSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask.s.IsDeviceManagementSupported", (uint32)pPortInfo->CapabilityMask.s.IsDeviceManagementSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask.s.IsConnectionManagementSupported", (uint32)pPortInfo->CapabilityMask.s.IsConnectionManagementSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask.s.IsAutomaticMigrationSupported", (uint32)pPortInfo->CapabilityMask.s.IsAutomaticMigrationSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask.s.IsSM", (uint32)pPortInfo->CapabilityMask.s.IsSM);

		PrintIntWithDots(dest, indent, "CapabilityMask3.AsReg16", (uint32)pPortInfo->CapabilityMask3.AsReg16);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsMAXLIDSupported", (uint32)pPortInfo->CapabilityMask3.s.IsMAXLIDSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsSnoopSupported", (uint32)pPortInfo->CapabilityMask3.s.IsSnoopSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsAsyncSC2VLSupported", (uint32)pPortInfo->CapabilityMask3.s.IsAsyncSC2VLSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsAddrRangeConfigSupported", (uint32)pPortInfo->CapabilityMask3.s.IsAddrRangeConfigSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsPassThroughSupported", (uint32)pPortInfo->CapabilityMask3.s.IsPassThroughSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsSharedSpaceSupported", (uint32)pPortInfo->CapabilityMask3.s.IsSharedSpaceSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsSharedGroupSpaceSupported", (uint32)pPortInfo->CapabilityMask3.s.IsSharedGroupSpaceSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsVLMarkerSupported", (uint32)pPortInfo->CapabilityMask3.s.IsVLMarkerSupported);
		PrintIntWithDots(dest, indent, "CapabilityMask3.s.IsVLrSupported", (uint32)pPortInfo->CapabilityMask3.s.IsVLrSupported);
		PrintStrWithDots(dest, indent, "CapabilityMask3.s.VLSchedulingConfig", StlVLSchedulingConfigToText(pPortInfo->CapabilityMask3));


		PrintIntWithDots(dest, indent, "OverallBufferSpace", (uint32)pPortInfo->OverallBufferSpace);
		PrintIntWithDots(dest, indent, "DiagCode.AsReg16", (uint32)pPortInfo->DiagCode.AsReg16);
		PrintIntWithDots(dest, indent, "DiagCode.s.UniversalDiagCode", (uint32)pPortInfo->DiagCode.s.UniversalDiagCode);
		PrintIntWithDots(dest, indent, "DiagCode.s.VendorDiagCode", (uint32)pPortInfo->DiagCode.s.VendorDiagCode);
		PrintIntWithDots(dest, indent, "DiagCode.s.Chain", (uint32)pPortInfo->DiagCode.s.Chain);

		PrintIntWithDots(dest, indent, "ReplayDepth.BufferDepth",
			(uint32)((pPortInfo->ReplayDepthH.BufferDepthH << 8) | pPortInfo->ReplayDepth.BufferDepth));
		PrintIntWithDots(dest, indent, "ReplayDepth.WireDepth",
			(uint32)((pPortInfo->ReplayDepthH.WireDepthH << 8) | pPortInfo->ReplayDepth.WireDepth));
		PrintIntWithDots(dest, indent, "PortNeighborMode.MgmtAllowed", (uint32)pPortInfo->PortNeighborMode.MgmtAllowed);
		PrintIntWithDots(dest, indent, "PortNeighborMode.NeighborFWAuthenBypass", (uint32)pPortInfo->PortNeighborMode.NeighborFWAuthenBypass);
		PrintIntWithDots(dest, indent, "PortNeighborMode.NeighborNodeType (0=FI,1=SW,>1=Unknown)", (uint32)pPortInfo->PortNeighborMode.NeighborNodeType);
		PrintIntWithDots(dest, indent, "MTU.Cap", (uint32)pPortInfo->MTU.Cap);
		PrintIntWithDots(dest, indent, "Resp.TimeValue", (uint32)pPortInfo->Resp.TimeValue);
		PrintIntWithDots(dest, indent, "LocalPortNum", (uint32)pPortInfo->LocalPortNum);
	} else {
		// Patterned after the existing IB output
		char tempBuf[64];
		char tempBuf2[64];
		char tempBuf3[64];
		char tempBuf4[64];
		char cbuf[80];
		//int i;
		uint8_t ldr=0;

		if (portGuid) {
			PrintFunc(dest, "%*sSubnet:   %#018"PRIx64"       GUID: %#018"PRIx64"\n",
				indent, "", pPortInfo->SubnetPrefix, portGuid);
		} else {
			PrintFunc(dest, "%*sSubnet:   %#018"PRIx64"\n",
				indent, "", pPortInfo->SubnetPrefix);
		}

		if(pPortInfo->LinkDownReason != 0)
			ldr = pPortInfo->LinkDownReason; 
		else if(pPortInfo->NeighborLinkDownReason != 0)
			ldr = pPortInfo->NeighborLinkDownReason;

		if(pPortInfo->PortStates.s.PortState == IB_PORT_INIT
			&& pPortInfo->s3.LinkInitReason) {
			PrintFunc(dest, "%*sLocalPort:     %-3d               PortState:        %s (%s)\n",
				indent, "",
				pPortInfo->LocalPortNum,
				StlPortStateToText(pPortInfo->PortStates.s.PortState),
				StlLinkInitReasonToText(pPortInfo->s3.LinkInitReason));
		} 
		else if(pPortInfo->PortStates.s.PortState == IB_PORT_DOWN && ldr) {
			PrintFunc(dest, "%*sLocalPort:     %-3d               PortState:        %s (%s)\n",
				indent, "",
				pPortInfo->LocalPortNum,
				StlPortStateToText(pPortInfo->PortStates.s.PortState),
				StlLinkDownReasonToText(ldr));
		}
		else {
			PrintFunc(dest, "%*sLocalPort:     %-3d               PortState:        %s\n",
				indent, "",
				pPortInfo->LocalPortNum,
				StlPortStateToText(pPortInfo->PortStates.s.PortState));
		}
        PrintFunc(dest, "%*sPhysicalState: %-14s\n",
            indent, "",
			StlPortPhysStateToText(pPortInfo->PortStates.s.PortPhysicalState));
    	PrintFunc(dest, "%*sOfflineDisabledReason: %-14s\n",
    			indent, "",
    			StlPortOfflineDisabledReasonToText(pPortInfo->PortStates.s.OfflineDisabledReason));
		PrintFunc(dest, "%*sIsSMConfigurationStarted: %-5s  NeighborNormal: %s\n",
            indent, "",
			pPortInfo->PortStates.s.IsSMConfigurationStarted?"True":"False",
			pPortInfo->PortStates.s.NeighborNormal?"True":"False");
		if (pPortInfo->LID == 0xffffffff)
			PrintFunc(dest, "%*sBaseLID:       DontCare          SMLID:            0x%08x\n",
				indent, "", pPortInfo->MasterSMLID);
		else
			PrintFunc(dest, "%*sBaseLID:       0x%08x        SMLID:            0x%08x\n",
				indent, "", pPortInfo->LID, pPortInfo->MasterSMLID);
		PrintFunc(dest, "%*sLMC:           %-2u                SMSL:             %-2u\n",
			indent, "",
			pPortInfo->s1.LMC, pPortInfo->s2.MasterSMSL);
		FormatTimeoutMult(tempBuf, pPortInfo->Resp.TimeValue);
		FormatTimeoutMult(tempBuf2, pPortInfo->Subnet.Timeout);
        PrintFunc(dest,"%*sPortType: %-22s LimtRsp/Subnet:   %s,%s\n",
            indent, "",  StlPortTypeToText(pPortInfo->PortPhysConfig.s.PortType), tempBuf, tempBuf2);
		PrintFunc(dest, "%*sBundleNextPort:%-3u               BundleLane:       %-3u\n",
			indent, "",
			pPortInfo->BundleNextPort,
			pPortInfo->BundleLane);
		PrintFunc(dest, "%*sM_KEY:    0x%016"PRIx64"     Lease:   %5u s  Protect: %s\n",
			indent, "", pPortInfo->M_Key, pPortInfo->M_KeyLeasePeriod,
			IbMKeyProtectToText(pPortInfo->s1.M_KeyProtectBits));
#if 0
		PrintFunc(dest, "%*sMTU:       Active:    %4u  Supported:       %4u  VL Stall: %u\n",
			indent, "", GetBytesFromMtu(pPortInfo->s2.NeighborMTU),
			GetBytesFromMtu(pPortInfo->MTU.Cap), pPortInfo->XmitQ.VLStallCount);
#endif
		PrintFunc(dest, "%*sLinkWidth      Act: %-8s     En: %-13s Sup: %-13s\n",
			indent,"",
			StlLinkWidthToText(pPortInfo->LinkWidth.Active, tempBuf, sizeof(tempBuf)),
			StlLinkWidthToText(pPortInfo->LinkWidth.Enabled, tempBuf3, sizeof(tempBuf3)),
			StlLinkWidthToText(pPortInfo->LinkWidth.Supported, tempBuf2, sizeof(tempBuf2)));
		PrintFunc(dest, "%*sLinkWidthDnGrd ActTx: %-2.2s Rx: %-2.2s  En: %-13s Sup: %-13s\n",
			indent,"",
			StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.TxActive, tempBuf, sizeof(tempBuf)),
			StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.RxActive, tempBuf4, sizeof(tempBuf4)),
			StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.Enabled, tempBuf3, sizeof(tempBuf3)),
			StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.Supported, tempBuf2, sizeof(tempBuf2)));
		PrintFunc(dest, "%*sLinkSpeed      Act: %-8s     En: %-13s Sup: %-13s\n",
			indent,"",
			StlLinkSpeedToText(pPortInfo->LinkSpeed.Active, tempBuf, sizeof(tempBuf)),
			StlLinkSpeedToText(pPortInfo->LinkSpeed.Enabled, tempBuf3, sizeof(tempBuf3)),
			StlLinkSpeedToText(pPortInfo->LinkSpeed.Supported, tempBuf2, sizeof(tempBuf2)));

		PrintFunc(dest, "%*sPortLinkMode   Act: %-8s     En: %-13s Sup: %-13s\n", 
			indent, "",
			StlPortLinkModeToText(pPortInfo->PortLinkMode.s.Active, tempBuf, sizeof(tempBuf)), 
			StlPortLinkModeToText(pPortInfo->PortLinkMode.s.Enabled, tempBuf3, sizeof(tempBuf3)),
			StlPortLinkModeToText(pPortInfo->PortLinkMode.s.Supported, tempBuf2, sizeof(tempBuf2)));

		PrintFunc(dest, "%*sPortLTPCRCMode Act: %-8s     En: %-13s Sup: %-13s\n", 
			indent, "",
			StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Active, tempBuf, sizeof(tempBuf)), 
			StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Enabled, tempBuf3, sizeof(tempBuf3)),
			StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Supported, tempBuf2, sizeof(tempBuf2)));

		PrintFunc(dest, "%*sNeighborMode   MgmtAllowed: %3s  FWAuthBypass: %3s NeighborNodeType: %s\n",
			indent, "",
			pPortInfo->PortNeighborMode.MgmtAllowed?"Yes":"No",
			pPortInfo->PortNeighborMode.NeighborFWAuthenBypass?"On":"Off",
			OpaNeighborNodeTypeToText(pPortInfo->PortNeighborMode.NeighborNodeType));
		PrintFunc(dest, "%*sNeighborNodeGuid:   0x%016"PRIx64"   NeighborPortNum:   %-3d\n",
			indent, "", pPortInfo->NeighborNodeGUID, pPortInfo->NeighborPortNum);

		FormatStlPortPacketFormat(cbuf, pPortInfo->PortPacketFormats.Supported, sizeof(cbuf));
		PrintFunc(dest, "%*sPortPacketFormats   Sup: %s",
			indent, "", cbuf);
		FormatStlPortPacketFormat(cbuf, pPortInfo->PortPacketFormats.Enabled, sizeof(cbuf));
		PrintFunc(dest, "  En: %s\n", cbuf);

		FormatStlCapabilityMask(cbuf, pPortInfo->CapabilityMask); // same as IB
		PrintFunc(dest, "%*sCapability:    0x%08x: %s\n",
			indent, "",
			pPortInfo->CapabilityMask.AsReg32, cbuf);
		FormatStlCapabilityMask3(cbuf, pPortInfo->CapabilityMask3, sizeof(cbuf));
		PrintFunc(dest, "%*sCapability3:   0x%04x: %s\n",
			indent, "", pPortInfo->CapabilityMask3.AsReg16, cbuf);


		PrintFunc(dest, "%*sSM_TrapQP: 0x%x  SA_QP: 0x%x\n",
			indent, "",
			pPortInfo->SM_TrapQP.s.QueuePair, pPortInfo->SA_QP.s.QueuePair);
		PrintFunc(dest, "%*sIPAddr IPV6/IPAddr IPv4:  %s/%s\n",
			indent, "",
			inet_ntop(AF_INET6, pPortInfo->IPAddrIPV6.addr, tempBuf, sizeof(tempBuf)),
			inet_ntop(AF_INET, pPortInfo->IPAddrIPV4.addr, tempBuf2, sizeof(tempBuf2)));

		PrintFunc(dest, "%*sVLs Active:    %d+1\n",
			indent, "", pPortInfo->s4.OperationalVL);
		if (pPortInfo->CapabilityMask3.s.VLSchedulingConfig == VL_SCHED_MODE_VLARB) {
			PrintFunc(dest, "%*sVL: PreemptCap %d   Cap %d+1   HighLimit 0x%04x   PreemptLimit 0x%04x\n",
				indent, "", pPortInfo->VL.PreemptCap, pPortInfo->VL.s2.Cap,
					pPortInfo->VL.HighLimit, pPortInfo->VL.PreemptingLimit);
		} else
			PrintFunc(dest, "%*sVL: Cap %d+1\n", indent, "", pPortInfo->VL.s2.Cap);

		PrintFunc(dest, "%*sVLFlowControlDisabledMask: 0x%08x", indent, "",
			  pPortInfo->FlowControlMask);
		if (pPortInfo->CapabilityMask3.s.VLSchedulingConfig == VL_SCHED_MODE_VLARB) {
			PrintFunc(dest, "   ArbHighCap: %d  ArbLowCap: %d",
				  pPortInfo->VL.ArbitrationHighCap,
				  pPortInfo->VL.ArbitrationLowCap);
		}
		PrintFunc(dest, "\n");

		PrintFunc(dest, "%*sMulticastMask: 0x%x    CollectiveMask: 0x%x\n",
			indent, "", pPortInfo->MultiCollectMask.MulticastMask,
				pPortInfo->MultiCollectMask.CollectiveMask);

		PrintFunc(dest, "%*sP_Key Enforcement: In: %3s Out: %3s\n",
			indent, "",
			pPortInfo->s3.PartitionEnforcementInbound?"On":"Off",
			pPortInfo->s3.PartitionEnforcementOutbound?"On":"Off");

		PrintFunc(dest, "%*sP_Keys        P_Key_8B:  0x%04x   P_Key_10B: 0x%04x\n",
			indent, "", pPortInfo->P_Keys.P_Key_8B, pPortInfo->P_Keys.P_Key_10B);

		PrintFunc(dest, "%*sMulticastPKeyTrapSuppressionEnabled: %2u   ClientReregister %2u\n",
			indent, "",
			pPortInfo->Subnet.MulticastPKeyTrapSuppressionEnabled,
			pPortInfo->Subnet.ClientReregister);

		PrintFunc(dest, "%*sPortMode ActiveOptimize: %3s PassThru: %3s VLMarker: %3s 16BTrapQuery: %3s\n",
			indent, "",
			pPortInfo->PortMode.s.IsActiveOptimizeEnabled?"On":"Off",
            pPortInfo->PortMode.s.IsPassThroughEnabled?"On":"Off",
			pPortInfo->PortMode.s.IsVLMarkerEnabled?"On":"Off",
			pPortInfo->PortMode.s.Is16BTrapQueryEnabled?"On":"Off");

		// Not have to print, passthrough not implemented in gen1
		//PrintFunc(dest, "%*sPassThroughDRControl: %3s\n",
		//	indent, "", pPortInfo->PassThroughControl.DRControl?"On":"Off");
		PrintFunc(dest, "%*sFlitCtrlInterleave Distance Max: %2u  Enabled: %2u\n",
			indent, "",
			pPortInfo->FlitControl.Interleave.s.DistanceSupported,
			pPortInfo->FlitControl.Interleave.s.DistanceEnabled);
		PrintFunc(dest, "%*s  MaxNestLevelTxEnabled: %u  MaxNestLevelRxSupported: %u\n",
			indent, "",
			pPortInfo->FlitControl.Interleave.s.MaxNestLevelTxEnabled,
			pPortInfo->FlitControl.Interleave.s.MaxNestLevelRxSupported);
		// Convert Flits to bytes and display in decimal
		PrintFunc(dest, "%*sFlitCtrlPreemption MinInitial: %"PRIu64" MinTail: %"PRIu64" LargePktLim: 0x%02x\n",
			indent, "",
			(uint64_t)(pPortInfo->FlitControl.Preemption.MinInitial * BYTES_PER_FLIT),
			(uint64_t)(pPortInfo->FlitControl.Preemption.MinTail * BYTES_PER_FLIT),
			pPortInfo->FlitControl.Preemption.LargePktLimit);
		PrintFunc(dest, "%*s  SmallPktLimit: 0x%02x MaxSmallPktLimit 0x%02x PreemptionLimit: 0x%02x\n",
			indent, "",
			pPortInfo->FlitControl.Preemption.SmallPktLimit,
			pPortInfo->FlitControl.Preemption.MaxSmallPktLimit,
			pPortInfo->FlitControl.Preemption.PreemptionLimit);

		if (pPortInfo->CapabilityMask3.s.IsMAXLIDSupported)
			PrintFunc(dest, "%*sMaxLID: %u\n",
				indent, "", pPortInfo->MaxLID);

		FormatStlPortErrorAction(cbuf, pPortInfo, sizeof(cbuf));
		PrintFunc(dest, "%*sPortErrorActions: 0x%x: %s\n", indent, "", pPortInfo->PortErrorAction.AsReg32, cbuf);

		PrintFunc(dest, "%*sBufferUnits:VL15Init 0x%04x VL15CreditRate 0x%02x CreditAck 0x%x BufferAlloc 0x%x\n",
			indent, "",
			pPortInfo->BufferUnits.s.VL15Init,
			pPortInfo->BufferUnits.s.VL15CreditRate,
			pPortInfo->BufferUnits.s.CreditAck,
			pPortInfo->BufferUnits.s.BufferAlloc);
		PrintFunc(dest, "%*sMTU  Supported: (0x%x) %s bytes\n",
			indent, "", pPortInfo->MTU.Cap, IbMTUToText(pPortInfo->MTU.Cap));
		PrintFunc(dest, "%*sMTU  Active By VL:\n",indent,"");
		PrintFunc(dest, "%*s00:%5s 01:%5s 02:%5s 03:%5s 04:%5s 05:%5s 06:%5s 07:%5s\n",
			indent, "",
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 0)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 1)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 2)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 3)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 4)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 5)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 6)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 7)));
		PrintFunc(dest, "%*s08:%5s 09:%5s 10:%5s 11:%5s 12:%5s 13:%5s 14:%5s 15:%5s\n",
			indent, "",
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 8)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 9)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 10)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 11)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 12)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 13)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 14)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 15)));
		PrintFunc(dest, "%*s16:%5s 17:%5s 18:%5s 19:%5s 20:%5s 21:%5s 22:%5s 23:%5s\n",
			indent, "",
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 16)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 17)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 18)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 19)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 20)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 21)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 22)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 23)));
		PrintFunc(dest, "%*s24:%5s 25:%5s 26:%5s 27:%5s 28:%5s 29:%5s 30:%5s 31:%5s\n",
			indent, "",
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 24)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 25)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 26)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 27)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 28)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 29)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 30)),
			IbMTUToText(GET_STL_PORT_INFO_NeighborMTU(pPortInfo, 31)));

		FormatStlVLStalls(cbuf, pPortInfo, sizeof(cbuf));
		PrintFunc(dest, "%*sStallCnt/VL: %s\n", indent, "", cbuf);

		FormatStlVLHOQLife(cbuf, pPortInfo, 0, 8, sizeof(cbuf));
		PrintFunc(dest, "%*sHOQLife VL[00,07]: %s\n", indent, "", cbuf);        
		FormatStlVLHOQLife(cbuf, pPortInfo, 8, 16, sizeof(cbuf));              
		PrintFunc(dest, "%*sHOQLife VL[08,15]: %s\n", indent, "", cbuf);       
		FormatStlVLHOQLife(cbuf, pPortInfo, 16, 24, sizeof(cbuf));              
		PrintFunc(dest, "%*sHOQLife VL[16,23]: %s\n", indent, "", cbuf);       
		FormatStlVLHOQLife(cbuf, pPortInfo, 24, 32, sizeof(cbuf));              
		PrintFunc(dest, "%*sHOQLife VL[24,31]: %s\n", indent, "", cbuf);        

		// Other odds and ends?
		PrintFunc(dest, "%*sReplayDepth Buffer 0x%02x; Wire 0x%02x\n",
			indent, "", 
			(pPortInfo->ReplayDepthH.BufferDepthH << 8) | pPortInfo->ReplayDepth.BufferDepth,
			(pPortInfo->ReplayDepthH.WireDepthH << 8) | pPortInfo->ReplayDepth.WireDepth);
		PrintFunc(dest, "%*sDiagCode: 0x%04x    LedEnabled: %-3s\n", indent, "", \
			pPortInfo->DiagCode.AsReg16,
			pPortInfo->PortStates.s.LEDEnabled? "On" : "Off");
		PrintFunc(dest, "%*sLinkDownReason: %s    NeighborLinkDownReason: %s\n",
			indent, "", StlLinkDownReasonToText(pPortInfo->LinkDownReason),
			StlLinkDownReasonToText(pPortInfo->NeighborLinkDownReason));

		PrintFunc(dest, "%*sOverallBufferSpace: 0x%04x\n",
			indent, "", pPortInfo->OverallBufferSpace);

		// Not sure counters belong in this port structure
		PrintFunc(dest, "%*sViolations    M_Key: %-5u     P_Key: %-5u    Q_Key: %-5u\n",
			indent, "",
			pPortInfo->Violations.M_Key, pPortInfo->Violations.P_Key,
			pPortInfo->Violations.Q_Key);
	}

}

void PrintStlLedInfo(PrintDest_t *dest, int indent, const STL_LED_INFO *pLedInfo, EUI64 portGuid, int printLineByLine)
{

	PrintFunc(dest, "%*sLedInfo: %d \n", indent, "", pLedInfo->u.s.LedMask);
}

void PrintStlPortInfoSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, EUI64 portGuid, int printLineByLine)
{
	STL_PORT_INFO *pPortInfo = (STL_PORT_INFO *)stl_get_smp_data((STL_SMP *)smp);

	PrintFunc(dest, "%*sPort %u Info\n", indent, "",
				smp->common.AttributeModifier & 0xff);
	PrintStlPortInfo(dest, indent+3, pPortInfo, portGuid, printLineByLine);
}

void PrintStlPortStateInfo(PrintDest_t *dest, int indent, const STL_PORT_STATE_INFO *psip, uint16 portCount, uint16 startPort, int printLineByLine)
{
	uint16 i;
    char tempBuf[64];
    if (printLineByLine) {
        for (i = 0; i < portCount; i++) {
	             snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.AsReg32", i+startPort);
          	      PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.AsReg32);
		snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.s.LEDEnabled", i+startPort);
			PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.s.LEDEnabled);
            snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.s.IsSMConfigurationStarted", i+startPort);
            PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.s.IsSMConfigurationStarted);
            snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.s.NeighborNormal", i+startPort);
            PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.s.NeighborNormal);
            snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.s.OfflineDisabledReason", i+startPort);
            PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.s.OfflineDisabledReason);
            snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.s.PortPhysicalState", i+startPort);
            PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.s.PortPhysicalState);
            snprintf(tempBuf, sizeof(tempBuf), "Port[%d].PortStates.s.PortState", i+startPort);
            PrintIntWithDots(dest, indent, tempBuf, psip[i].PortStates.s.PortState);
			snprintf(tempBuf, sizeof(tempBuf), "Port[%d].LinkWidthDowngradeTxActive", i+startPort);
			PrintIntWithDots(dest, indent, tempBuf, psip[i].LinkWidthDowngradeTxActive);
			snprintf(tempBuf, sizeof(tempBuf), "Port[%d].LinkWidthDowngradeRxActive", i+startPort);
			PrintIntWithDots(dest, indent, tempBuf, psip[i].LinkWidthDowngradeRxActive);

        }
    }
    else {
	char tempBuf2[64];
        for (i=0; i<portCount; ++i) { 
    		PrintFunc(dest, "%*sPort %u: PortState: %-6s  PhysState: %-8s\n",
    			indent, "", startPort+i,
    			StlPortStateToText(psip[i].PortStates.s.PortState),
    			StlPortPhysStateToText(psip[i].PortStates.s.PortPhysicalState));

    		PrintFunc(dest, "%*sOfflineDisabledReason: %-14s  LedEnabled: %-3s\n",
				indent+10, "", 
    			StlPortOfflineDisabledReasonToText(psip[i].PortStates.s.OfflineDisabledReason),
				psip[i].PortStates.s.LEDEnabled ? "On" : "Off");

    		PrintFunc(dest, "%*sIsSMConfigurationStarted %-5s   NeighborNormal: %-5s\n",
    			indent+10, "",
    			psip[i].PortStates.s.IsSMConfigurationStarted?"True":"False",
    			psip[i].PortStates.s.NeighborNormal?"True":"False");
			PrintFunc(dest, "%*sLinkWidthDnGrd ActTx: %-2.2s Rx: %-2.2s\n",
				indent+10,"",
				StlLinkWidthToText(psip[i].LinkWidthDowngradeTxActive, tempBuf, sizeof(tempBuf)),
				StlLinkWidthToText(psip[i].LinkWidthDowngradeRxActive, tempBuf2, sizeof(tempBuf2)));	
		}
    }
}

void PrintStlPKeyTable(PrintDest_t *dest, int indent, const STL_PARTITION_TABLE *pPKeyTable, uint16 blockNum, int printLineByLine)
{
	uint16 i;
	char buf[81];
	int offset=0;
	uint16 index = blockNum*NUM_PKEY_ELEMENTS_BLOCK;
    if (printLineByLine) {
        for (i = 0; i < NUM_PKEY_ELEMENTS_BLOCK; i++) {
            snprintf(buf, sizeof(buf), "PartitionTableBlock[%d].AsReg16", i);
            PrintIntWithDots(dest, indent, buf, pPKeyTable->PartitionTableBlock[i].AsReg16);
            snprintf(buf, sizeof(buf), "PartitionTableBlock[%d].s.MembershipType", i);
            PrintIntWithDots(dest, indent, buf, pPKeyTable->PartitionTableBlock[i].s.MembershipType);
            snprintf(buf, sizeof(buf), "PartitionTableBlock[%d].s.P_KeyBase", i);
            PrintIntWithDots(dest, indent, buf, pPKeyTable->PartitionTableBlock[i].s.P_KeyBase);
        }
    }
    else {

    	// 8 entries per line
    	for (i=0; i<NUM_PKEY_ELEMENTS_BLOCK; ++i)
    	{
    		if (i%8 == 0) {
    			if (i)
    				PrintFunc(dest, "%*s%s\n", indent, "", buf);
    			offset=sprintf(buf, "   %4u-%4u:", i+index, i+7+index);
    		}
    		offset+=sprintf(&buf[offset], "  0x%04x", pPKeyTable->PartitionTableBlock[i].AsReg16);
    	}
    	PrintFunc(dest, "%*s%s\n", indent, "", buf);
    }
}

void PrintStlPKeyTableSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, NODE_TYPE nodetype, boolean showHeading, boolean showBlock, int printLineByLine)
{
	char buf[40] = "";
	uint32 sub;
	uint32 block = smp->common.AttributeModifier & PKEY_BLOCK_NUM_MASK;
	uint32 numBlocks = (smp->common.AttributeModifier >> 24) & 0xff;
    uint32 portNum = (smp->common.AttributeModifier >> 16) & 0xff;
	STL_PARTITION_TABLE *pPKeyTable;
    if (printLineByLine) {
        if (showHeading) {
            if (nodetype == STL_NODE_SW)
                PrintIntWithDots(dest, indent, "OutputPort", portNum);
            if (showBlock)
                PrintIntWithDots(dest, indent, "BlockNum", block);
        }
        pPKeyTable = (STL_PARTITION_TABLE *)stl_get_smp_data((STL_SMP *)smp);
        for (sub = 0; sub < numBlocks; sub++) {
            PrintStlPKeyTable(dest, indent, pPKeyTable + sub, sub+block, printLineByLine);
        }

    }
    else {
        if (showHeading) {
            if (nodetype == STL_NODE_SW)
                sprintf(buf, "OutputPort: %3u ", portNum);
            if (showBlock)
                PrintFunc(dest, "%*s%sBlockNum: %2u\n", indent, "", buf, block);
            else if (buf[0])
                PrintFunc(dest, "%*s%s\n", indent, "", buf);
        }

        pPKeyTable = (STL_PARTITION_TABLE *)stl_get_smp_data((STL_SMP *)smp);
        for (sub = 0; sub < numBlocks; sub++) {
            PrintStlPKeyTable(dest, indent, pPKeyTable + sub, sub+block, printLineByLine);
        }
    }
}

void PrintStlSwitchInfo(PrintDest_t *dest, int indent, const STL_SWITCH_INFO *pSwitchInfo, STL_LID lid, int printLineByLine)
{
	char buf[64];
    if (printLineByLine) {
        if (lid)
            PrintIntWithDots(dest, indent, "lid", lid);

		if (pSwitchInfo->RoutingMode.Enabled & STL_ROUTE_LINEAR)
			PrintIntWithDots(dest, indent, "LinearFDBCap", pSwitchInfo->LinearFDBCap);

		PrintIntWithDots(dest, indent, "MulticastFDBCap", pSwitchInfo->MulticastFDBCap);
		PrintIntWithDots(dest, indent, "LinearFDBTop", pSwitchInfo->LinearFDBTop);
        PrintIntWithDots(dest, indent, "MulticastFDBTop", pSwitchInfo->MulticastFDBTop);
    #if 0
        //Not Support in Gen 1.
        PrintIntWithDots(dest, indent, "CollectiveCap", pSwitchInfo->CollectiveCap);
        PrintIntWithDots(dest, indent, "CollectiveTop", pSwitchInfo->CollectiveTop);
    #endif
        PrintStrWithDots(dest, indent, "IPAddrIPV6",
			inet_ntop(AF_INET6, pSwitchInfo->IPAddrIPV6.addr, buf, sizeof(buf)));
        PrintStrWithDots(dest, indent, "IPAddrIPV4",
			inet_ntop(AF_INET, pSwitchInfo->IPAddrIPV4.addr, buf, sizeof(buf)));

        PrintIntWithDots(dest, indent, "u1.AsReg8", pSwitchInfo->u1.AsReg8);
        PrintIntWithDots(dest, indent, "u1.s.PortStateChange", pSwitchInfo->u1.s.PortStateChange);
        PrintIntWithDots(dest, indent, "u1.s.LifeTimeValue", pSwitchInfo->u1.s.LifeTimeValue);
        PrintIntWithDots(dest, indent, "PartitionEnforcementCap", pSwitchInfo->PartitionEnforcementCap);
        PrintIntWithDots(dest, indent, "PortGroupFDBCap", pSwitchInfo->PortGroupFDBCap);
        PrintIntWithDots(dest, indent, "PortGroupCap", pSwitchInfo->PortGroupCap);
        PrintIntWithDots(dest, indent, "PortGroupTop", pSwitchInfo->PortGroupTop);
        PrintIntWithDots(dest, indent, "RoutingMode.Supported", pSwitchInfo->RoutingMode.Supported);
        PrintIntWithDots(dest, indent, "RoutingMode.Enabled", pSwitchInfo->RoutingMode.Enabled);
        PrintIntWithDots(dest, indent, "u2.AsReg8", pSwitchInfo->u2.AsReg8);
        PrintIntWithDots(dest, indent, "u2.s.EnhancedPort0", pSwitchInfo->u2.s.EnhancedPort0);
        PrintIntWithDots(dest, indent, "MultiCollectMask.CollectiveMask", pSwitchInfo->MultiCollectMask.CollectiveMask);
        PrintIntWithDots(dest, indent, "MultiCollectMask.MulticastMask", pSwitchInfo->MultiCollectMask.MulticastMask);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.AsReg16", pSwitchInfo->AdaptiveRouting.AsReg16);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.s.Enable", pSwitchInfo->AdaptiveRouting.s.Enable);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.s.Pause", pSwitchInfo->AdaptiveRouting.s.Pause);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.s.Algorithm", pSwitchInfo->AdaptiveRouting.s.Algorithm);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.s.Frequency", pSwitchInfo->AdaptiveRouting.s.Frequency);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.s.LostRoutesOnly", pSwitchInfo->AdaptiveRouting.s.LostRoutesOnly);
        PrintIntWithDots(dest, indent, "AdaptiveRouting.s.Threshold", pSwitchInfo->AdaptiveRouting.s.Threshold);
        PrintIntWithDots(dest, indent, "CapabilityMask.AsReg16", pSwitchInfo->CapabilityMask.AsReg16);
        PrintIntWithDots(dest, indent, "CapabilityMask.s.IsAddrRangeConfigSupported", pSwitchInfo->CapabilityMask.s.IsAddrRangeConfigSupported);
        PrintIntWithDots(dest, indent, "CapabilityMask.s.IsExtendedSCSCSupported", pSwitchInfo->CapabilityMask.s.IsExtendedSCSCSupported);
        PrintIntWithDots(dest, indent, "CapabilityMask.s.IsAdaptiveRoutingSupported", pSwitchInfo->CapabilityMask.s.IsAdaptiveRoutingSupported);

#if 0
        //Not Support in Gen 1.
        PrintIntWithDots(dest, indent, "CapabilityMaskCollectives.AsReg16", pSwitchInfo->CapabilityMaskCollectives.AsReg16);
#endif
    }
    else {

		if (lid)
			PrintFunc(dest, "%*sLID: 0x%08x\n", indent, "", lid);

		if (pSwitchInfo->RoutingMode.Enabled & STL_ROUTE_LINEAR)
			PrintFunc(dest, "%*sLinearFDBCap: %5u LinearFDBTop: 0x%08x MCFDBCap: %5u MCFDBTop: 0x%08x\n",
					indent, "",
					pSwitchInfo->LinearFDBCap, pSwitchInfo->LinearFDBTop,
					pSwitchInfo->MulticastFDBCap, pSwitchInfo->MulticastFDBTop);
		else
			PrintFunc(dest, "%*sLinearFDBTop: 0x%08x MCFDBCap: %5u MCFDBTop: 0x%08x\n",
					indent, "",
					pSwitchInfo->LinearFDBTop, pSwitchInfo->MulticastFDBCap, pSwitchInfo->MulticastFDBTop);
    	PrintFunc(dest, "%*sCapability: 0x%02x: %s PartEnfCap: %5u PortStateChange: %d SwitchLifeTime: %d\n",
    				indent, "",
    				pSwitchInfo->u2.AsReg8,
    				pSwitchInfo->u2.s.EnhancedPort0?"E0 ": "",
    				pSwitchInfo->PartitionEnforcementCap,
    				pSwitchInfo->u1.s.PortStateChange, 
    				pSwitchInfo->u1.s.LifeTimeValue);
		PrintFunc(dest, "%*sPortGroupFDBCap: %5u PortGroupCap: %3u PortGroupTop: %3u\n",
					indent, "",
					pSwitchInfo->PortGroupFDBCap,
					pSwitchInfo->PortGroupCap, pSwitchInfo->PortGroupTop);
    	PrintFunc(dest, "%*sIPAddrIPV6:  %s\n",
    				indent, "", 
    				inet_ntop(AF_INET6, 
    						  pSwitchInfo->IPAddrIPV6.addr, 
    						  buf, sizeof(buf)));
    	PrintFunc(dest, "%*sIPAddrIPV4: %s\n",
    				indent, "", 
    				inet_ntop(AF_INET, 
    						  pSwitchInfo->IPAddrIPV4.addr, 
    						  buf, sizeof(buf)));
    	PrintFunc(dest, "%*sCollectiveCap: %5u CollectiveTop: %5u CollectiveMask: %1u MulticastMask: %1u\n",
    				indent, "", 
    			 	pSwitchInfo->CollectiveCap, pSwitchInfo->CollectiveTop,
    				pSwitchInfo->MultiCollectMask.CollectiveMask, 
    				pSwitchInfo->MultiCollectMask.MulticastMask); 
    	
    	PrintFunc(dest, "%*sRoutingMode: Supported: %s Enabled: %s\n",
    				indent, "", 
    				StlRoutingModeToText(pSwitchInfo->RoutingMode.Supported),
    				StlRoutingModeToText(pSwitchInfo->RoutingMode.Enabled));

    	PrintFunc(dest, "%*sAdaptiveRouting: Enable: %u Pause: %u Alg: %u Freq: %u LostRoutesOnly: %u Thresh: %u\n",
    				indent, "",
    				pSwitchInfo->AdaptiveRouting.s.Enable,
    				pSwitchInfo->AdaptiveRouting.s.Pause,
    				pSwitchInfo->AdaptiveRouting.s.Algorithm,
    				pSwitchInfo->AdaptiveRouting.s.Frequency,
    				pSwitchInfo->AdaptiveRouting.s.LostRoutesOnly,
					pSwitchInfo->AdaptiveRouting.s.Threshold);

    	PrintFunc(dest, "%*sCapabilityMask: IsAddrRangeConfigSupported: %u IsAdaptiveRoutingSupported: %u\n",
    				indent, "", 
    				pSwitchInfo->CapabilityMask.s.IsAddrRangeConfigSupported,
                    pSwitchInfo->CapabilityMask.s.IsAdaptiveRoutingSupported);

    	PrintFunc(dest, "%*sCapabilityMask: IsExtendedSCSCSupported: %u\n",
    				indent, "", 
                    pSwitchInfo->CapabilityMask.s.IsExtendedSCSCSupported);

#if 0
        //Not Support in Gen 1.
        PrintFunc(dest, "*sCapabilityMaskCollectives: Reserved: %u\n", indent, "", pSwitchInfo->CapabilityMaskCollectives.s.Reserved);
#endif

		PrintFunc(dest, "%*sMulticastMask: 0x%x    CollectiveMask: 0x%x\n",
					indent, "", pSwitchInfo->MultiCollectMask.MulticastMask,
					pSwitchInfo->MultiCollectMask.CollectiveMask);
    }
}

void PrintStlCongestionInfo(PrintDest_t *dest, int indent, const STL_CONGESTION_INFO *pCongestionInfo, int printLineByLine)
{
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "CongestionInfo", pCongestionInfo->CongestionInfo);
        PrintIntWithDots(dest, indent, "ControlTableCap", pCongestionInfo->ControlTableCap);
        PrintIntWithDots(dest, indent, "CongestionLogLength", pCongestionInfo->CongestionLogLength);
    }
    else {
    	PrintFunc(dest, "%*sCongestion Info: %s\n", indent, "",
    		pCongestionInfo->CongestionInfo == CC_CONGESTION_INFO_CREDIT_STARVATION ? "CS" : "No CS");
    	PrintFunc(dest, "%*sControl Table Capacity: %u\n", indent, "", pCongestionInfo->ControlTableCap);
    	PrintFunc(dest, "%*sCongestion Log Size: %u\n", indent, "", pCongestionInfo->CongestionLogLength);
    }
}

void PrintStlSwitchCongestionSetting(PrintDest_t *dest, int indent, const STL_SWITCH_CONGESTION_SETTING *pSwCongestionSetting, int printLineByLine)
{
	unsigned int i;
    char tempVal[128] = { '\0' };

    if (printLineByLine) {
		char *cursor;

        PrintIntWithDots(dest, indent, "ControlMap", pSwCongestionSetting->Control_Map);

		cursor = tempVal;
		i = 0;
		while (i < 32) {
			cursor += sprintf(cursor, "%02x", pSwCongestionSetting->Victim_Mask[i]);
			if ((++i % 8) == 0) *(cursor++) = ' ';
		}
        PrintStrWithDots(dest, indent, "Victim_Mask", tempVal);

		cursor = tempVal;
		i = 0;
		while (i < 32) {
			cursor += sprintf(cursor, "%02x", pSwCongestionSetting->Credit_Mask[i]);
			if ((++i % 8) == 0) *(cursor++) = ' ';
		}
        PrintStrWithDots(dest, indent, "Credit_Mask", tempVal);

        PrintIntWithDots(dest, indent, "Threshold", pSwCongestionSetting->Threshold);
        PrintIntWithDots(dest, indent, "Packet_Size", pSwCongestionSetting->Packet_Size);
        PrintIntWithDots(dest, indent, "CS_Threshold", pSwCongestionSetting->CS_Threshold);
        PrintIntWithDots(dest, indent, "CS_ReturnDelay", pSwCongestionSetting->CS_ReturnDelay);
        PrintIntWithDots(dest, indent, "Marking_Rate", pSwCongestionSetting->Marking_Rate);

    }
    else {
    	PrintFunc(dest, "%*sControl Map: 0x%x ( ", indent, "", pSwCongestionSetting->Control_Map);
    	if (pSwCongestionSetting->Control_Map & CC_SWITCH_CONTROL_MAP_VICTIM_VALID)
    		PrintFunc(dest, "Victim_Mask ");
    	if (pSwCongestionSetting->Control_Map & CC_SWITCH_CONTROL_MAP_CREDIT_VALID)
    		PrintFunc(dest, "Credit_Mask ");
    	if (pSwCongestionSetting->Control_Map & CC_SWITCH_CONTROL_MAP_CC_VALID)
    		PrintFunc(dest, "Threshold/Packet_Size ");
    	if (pSwCongestionSetting->Control_Map & CC_SWITCH_CONTROL_MAP_CS_VALID)
    		PrintFunc(dest, "CS_Threshold/CS_ReturnDelay ");
    	if (pSwCongestionSetting->Control_Map & CC_SWITCH_CONTROL_MAP_MARKING_VALID)
    		PrintFunc(dest, "Marking_Rate ");
    	PrintFunc(dest, ")\n");

    	PrintFunc(dest, "%*sPer Port Mask Values:\n", indent, "");
    	PrintFunc(dest, "%*sMask Type    |   Flagged Ports\n", indent+4, "");

    	PrintFunc(dest, "%*sVictim Mask  |   ", indent+4, "");
    	for (i = 0; i < 256; ++i)
    		if ((pSwCongestionSetting->Victim_Mask[31-(i/8)]>>(i % 8)) & 1) PrintFunc(dest, "%u ", i);
    	PrintFunc(dest, "\n");

    	PrintFunc(dest, "%*sCredit Mask  |   ", indent+4, "");
    	for (i = 0; i < 256; ++i)
    		if ((pSwCongestionSetting->Credit_Mask[31-(i/8)]>>(i % 8)) & 1) PrintFunc(dest, "%u ", i);
    	PrintFunc(dest, "\n");

    	PrintFunc(dest, "%*sThreshold: %u\n", indent, "", pSwCongestionSetting->Threshold);
    	PrintFunc(dest, "%*sPacket Size: %u\n", indent, "", pSwCongestionSetting->Packet_Size);
    	PrintFunc(dest, "%*sCS Threshold: %u\n", indent, "", pSwCongestionSetting->CS_Threshold);
    	PrintFunc(dest, "%*sCS ReturnDelay: %u\n", indent, "", pSwCongestionSetting->CS_ReturnDelay);
    	PrintFunc(dest, "%*sMarking Rate: %u\n", indent, "", pSwCongestionSetting->Marking_Rate);
    }
}

void PrintStlHfiCongestionSetting(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_SETTING *pHfiCongestionSetting, int printLineByLine)
{
	unsigned int i;
    char tempBuf[64];
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "Control_Map", pHfiCongestionSetting->Control_Map);
        PrintIntWithDots(dest, indent, "Port_Control", pHfiCongestionSetting->Port_Control);
        for (i = 0; i < STL_MAX_SLS; i++) {
            snprintf(tempBuf, sizeof(tempBuf), "HFICongestionEntryList[%d].CCTI_Increase", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionSetting->HFICongestionEntryList[i].CCTI_Increase);
            snprintf(tempBuf, sizeof(tempBuf), "HFICongestionEntryList[%d].CCTI_Timer", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionSetting->HFICongestionEntryList[i].CCTI_Timer);
            snprintf(tempBuf, sizeof(tempBuf), "HFICongestionEntryList[%d].TriggerThreshold", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionSetting->HFICongestionEntryList[i].TriggerThreshold);
            snprintf(tempBuf, sizeof(tempBuf), "HFICongestionEntryList[%d].CCTI_Min", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionSetting->HFICongestionEntryList[i].CCTI_Min);
        }
    }
    else {
    	PrintFunc(dest, "%*sCongestion Control Type: %s\n", indent, "", 
    		pHfiCongestionSetting->Port_Control == CC_HFI_CONGESTION_SETTING_SL_PORT ? "SL Based" : "QP Based");

    	PrintFunc(dest, "%*sEntries:\n", indent, "");
    	for (i=0; i < STL_MAX_SLS; ++i) {
    		if (pHfiCongestionSetting->Control_Map & (1<<i)) {
                PrintFunc(dest, "%*sSL: %u\n", indent+4, "", i);
                PrintFunc(dest, "%*sCCT Increase: %u\n", indent+8, "", pHfiCongestionSetting->HFICongestionEntryList[i].CCTI_Increase);
                PrintFunc(dest, "%*sCCT Timer: %u\n", indent+8, "", pHfiCongestionSetting->HFICongestionEntryList[i].CCTI_Timer);
                PrintFunc(dest, "%*sCCT Trigger Threshold: %u\n", indent+8, "", 
                    pHfiCongestionSetting->HFICongestionEntryList[i].TriggerThreshold);
                PrintFunc(dest, "%*sCCT Min: %u\n", indent+8, "", pHfiCongestionSetting->HFICongestionEntryList[i].CCTI_Min);
            }
        }
    }
}

void PrintStlSwitchPortCongestionSettingSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, int printLineByLine)
{
	const uint8_t start = smp->common.AttributeModifier & 0xff;
	const uint8_t count = (smp->common.AttributeModifier>>24) & 0xff;
	const STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT *element;
	uint8_t i;
	for (i = 0; i < count; ++i) {
		element = &((STL_SWITCH_PORT_CONGESTION_SETTING*)stl_get_smp_data((STL_SMP*)smp))->Elements[i];
		if (!printLineByLine)
			PrintFunc(dest, "%*sPort %u:\n", indent, "", i + start);
		PrintStlSwitchPortCongestionSettingElement(dest, indent + (printLineByLine ? 0 : 4), element,
													i + start, printLineByLine);
	}
}

void PrintStlSwitchPortCongestionSettingElement(PrintDest_t *dest, int indent, const STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT *pElement, uint8_t index, int printLineByLine)
{
    char tempBuf[64] = { '\0' };

    if (printLineByLine) {
 		snprintf(tempBuf, sizeof(tempBuf), "Elements[%d].Valid", index);
		PrintIntWithDots(dest, indent, tempBuf, pElement->Valid);
		snprintf(tempBuf, sizeof(tempBuf), "Elements[%d].Control_Type", index);
		PrintIntWithDots(dest, indent, tempBuf, pElement->Control_Type);
		snprintf(tempBuf, sizeof(tempBuf), "Elements[%d].Threshold", index);
		PrintIntWithDots(dest, indent, tempBuf, pElement->Threshold);
		snprintf(tempBuf, sizeof(tempBuf), "Elements[%d].Packet_Size", index);
		PrintIntWithDots(dest, indent, tempBuf, pElement->Packet_Size);
		snprintf(tempBuf, sizeof(tempBuf), "Elements[%d].Marking_Rate", index);
		PrintIntWithDots(dest, indent, tempBuf, pElement->Marking_Rate);
    } else {
    	PrintFunc(dest, "%*sValid: %s, Control Type: %s, Threshold: %u, Packet Size: %d, Marking Rate: %u\n",
    		indent, "",
    		pElement->Valid ? "Yes" : "No",
    		pElement->Control_Type ? "Credit Starvation" : "Congestion Control",
    		pElement->Threshold,
    		pElement->Packet_Size, pElement->Marking_Rate);
    }
}


void PrintStlSwitchCongestionLog(PrintDest_t *dest, int indent, const STL_SWITCH_CONGESTION_LOG *pSwCongestionLog, int printLineByLine)
{
	unsigned int i;
    char tempBuf[64];
    char tempVal[300];
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "LogType", pSwCongestionLog->LogType);
        PrintIntWithDots(dest, indent, "CongestionFlags", pSwCongestionLog->CongestionFlags);
        PrintIntWithDots(dest, indent, "LogEventsCounter", pSwCongestionLog->LogEventsCounter);
        PrintIntWithDots(dest, indent, "CurrentTimeStamp", pSwCongestionLog->CurrentTimeStamp);
        snprintf(tempVal, sizeof(tempVal), "%02x%02x%02x%02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x%02x%02x", 
                 pSwCongestionLog->PortMap[31], pSwCongestionLog->PortMap[30], pSwCongestionLog->PortMap[29], pSwCongestionLog->PortMap[28], 
                 pSwCongestionLog->PortMap[27], pSwCongestionLog->PortMap[26], pSwCongestionLog->PortMap[25], pSwCongestionLog->PortMap[24], 
                 pSwCongestionLog->PortMap[23], pSwCongestionLog->PortMap[22], pSwCongestionLog->PortMap[21], pSwCongestionLog->PortMap[20], 
                 pSwCongestionLog->PortMap[19], pSwCongestionLog->PortMap[18], pSwCongestionLog->PortMap[17], pSwCongestionLog->PortMap[16], 
                 pSwCongestionLog->PortMap[15], pSwCongestionLog->PortMap[14], pSwCongestionLog->PortMap[13], pSwCongestionLog->PortMap[12], 
                 pSwCongestionLog->PortMap[11], pSwCongestionLog->PortMap[10], pSwCongestionLog->PortMap[9],  pSwCongestionLog->PortMap[8], 
                 pSwCongestionLog->PortMap[7],  pSwCongestionLog->PortMap[6],  pSwCongestionLog->PortMap[5],  pSwCongestionLog->PortMap[4], 
                 pSwCongestionLog->PortMap[3],  pSwCongestionLog->PortMap[2],  pSwCongestionLog->PortMap[1],  pSwCongestionLog->PortMap[0]);
        PrintStrWithDots(dest, indent, "PortMap", tempVal);
        for (i = 0; i < MIN(STL_NUM_CONGESTION_LOG_ELEMENTS, pSwCongestionLog->LogEventsCounter); ++i) {
            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].SLID", i);
            PrintIntWithDots(dest, indent, tempBuf, pSwCongestionLog->CongestionEntryList[i].SLID);
            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].DLID", i);
            PrintIntWithDots(dest, indent, tempBuf, pSwCongestionLog->CongestionEntryList[i].DLID);
            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].SC", i);
            PrintIntWithDots(dest, indent, tempBuf, pSwCongestionLog->CongestionEntryList[i].SC);
            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].TimeStamp", i);
            PrintIntWithDots(dest, indent, tempBuf, pSwCongestionLog->CongestionEntryList[i].TimeStamp);
        }
    }
    else {
    	PrintFunc(dest, "%*sLogType: %s\nCongestion Flags: 0x%hhx\nLog Events: %hu\nCurrent Timestamp: %.3f uSec (0x%x)\n",
    		indent, "",
    		pSwCongestionLog->LogType == 1 ? "Switch" : "Invalid - Should always be Switch", 
    		pSwCongestionLog->CongestionFlags,
    		pSwCongestionLog->LogEventsCounter,
    		(float)pSwCongestionLog->CurrentTimeStamp * 1.024f, pSwCongestionLog->CurrentTimeStamp);
    	PrintFunc(dest, "%*sMarked Ports:\n", indent, "");
    	PrintFunc(dest, "%*s", indent+4, "");
    	for (i = 0; i < 256; ++i)
    		if ((pSwCongestionLog->PortMap[31-(i/8)]>>(i % 8)) & 1) PrintFunc(dest, "%u, ", i);
    	PrintFunc(dest, "\n");

    	PrintFunc(dest, "%*sCongestion Event Log:\n", indent, "");
    	// CongestionLog will contain at most the 20 most recent log events.
    	for (i = 0; i < MIN(STL_NUM_CONGESTION_LOG_ELEMENTS, pSwCongestionLog->LogEventsCounter); ++i) {
            const STL_SWITCH_CONGESTION_LOG_EVENT *cursor = &pSwCongestionLog->CongestionEntryList[i];
    		PrintFunc(dest, "%*sEvent #%u:\n", indent+4, "", i + 1);
    		PrintFunc(dest, "%*sSLID: %u\n", indent+8, "", cursor->SLID);
    		PrintFunc(dest, "%*sDLID: %u\n", indent+8, "", cursor->DLID);
    		PrintFunc(dest, "%*sSC: %u\n", indent+8, "", cursor->SC);
    		PrintFunc(dest, "%*sTimeStamp: %.3f uSec (0x%x)\n", indent+8, "", (float)cursor->TimeStamp * 1.024f, cursor->TimeStamp);
        }
	}
}


void PrintStlHfiCongestionLog(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_LOG *pHfiCongestionLog, int printLineByLine)
{
	unsigned int i;
    char tempBuf[64];
    char tempVal[16];
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "LogType", pHfiCongestionLog->LogType);
        PrintIntWithDots(dest, indent, "CongestionFlags", pHfiCongestionLog->CongestionFlags);
        PrintIntWithDots(dest, indent, "ThresholdEventCounter", pHfiCongestionLog->ThresholdEventCounter);
        PrintIntWithDots(dest, indent, "CurrentTimeStamp", pHfiCongestionLog->CurrentTimeStamp);
        snprintf(tempVal, sizeof(tempVal), "%02x%02x%02x%02x", 
                 pHfiCongestionLog->ThresholdCongestionEventMap[3],  pHfiCongestionLog->ThresholdCongestionEventMap[2],  
                 pHfiCongestionLog->ThresholdCongestionEventMap[1],  pHfiCongestionLog->ThresholdCongestionEventMap[0]);
        PrintStrWithDots(dest, indent, "ThresholdCongestionEventMap", tempVal);
        for (i = 0; i < MIN(STL_NUM_CONGESTION_LOG_ELEMENTS, pHfiCongestionLog->ThresholdEventCounter); ++i) {
            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].Local_QP_CN_Entry", i);
            snprintf(tempVal, sizeof(tempVal), "%02x%02x%02x", 
                     pHfiCongestionLog->CongestionEntryList[i].Local_QP_CN_Entry.AsReg8s[2],  
                     pHfiCongestionLog->CongestionEntryList[i].Local_QP_CN_Entry.AsReg8s[1],
                     pHfiCongestionLog->CongestionEntryList[i].Local_QP_CN_Entry.AsReg8s[0]);
            PrintStrWithDots(dest, indent, tempBuf, tempVal);

            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].Remote_QP_Number_CN_Entry", i);
            snprintf(tempVal, sizeof(tempVal), "%02x%02x%02x", 
                     pHfiCongestionLog->CongestionEntryList[i].Remote_QP_Number_CN_Entry.AsReg8s[2],  
                     pHfiCongestionLog->CongestionEntryList[i].Remote_QP_Number_CN_Entry.AsReg8s[1],
                     pHfiCongestionLog->CongestionEntryList[i].Remote_QP_Number_CN_Entry.AsReg8s[0]);
            PrintStrWithDots(dest, indent, tempBuf, tempVal);

            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].SL_CN_Entry", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionLog->CongestionEntryList[i].SL_CN_Entry);

            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].Service_Type_CN_Entry", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionLog->CongestionEntryList[i].Service_Type_CN_Entry);

            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].Remote_LID_CN_Entry", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionLog->CongestionEntryList[i].Remote_LID_CN_Entry);

            snprintf(tempBuf, sizeof(tempBuf), "CongestionEntryList[%d].TimeStamp_CN_Entry", i);
            PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionLog->CongestionEntryList[i].TimeStamp_CN_Entry);
        }
    }
    else {
    	PrintFunc(dest, "%*sLogType: %s\nCongestion Flags: 0x%hhx\nLog Events: %hu\nCurrent Timestamp: %.3f uSec (0x%x)\n",
    		indent, "",
    		pHfiCongestionLog->LogType == 2 ? "HFI" : "Invalid - Should always be HFI", 
    		pHfiCongestionLog->CongestionFlags,
    		pHfiCongestionLog->ThresholdEventCounter,
    		(float)pHfiCongestionLog->CurrentTimeStamp * 1.024f, pHfiCongestionLog->CurrentTimeStamp);
    	PrintFunc(dest, "%*sMarked SLs:\n", indent, "");
    	PrintFunc(dest, "%*s", indent+4, "");
    	for (i = 0; i < 32; ++i)
    		if ((pHfiCongestionLog->ThresholdCongestionEventMap[i/8]>>(i % 8)) & 1) PrintFunc(dest, "%u, ", i);
    	PrintFunc(dest, "\n"); 

    	PrintFunc(dest, "%*sCongestion Event Log:\n", indent, "");
    	for (i = 0; i < MIN(STL_NUM_CONGESTION_LOG_ELEMENTS, pHfiCongestionLog->ThresholdEventCounter); ++i) {
            const STL_HFI_CONGESTION_LOG_EVENT *cursor = &pHfiCongestionLog->CongestionEntryList[i];
    		const uint32_t localQP = cursor->Local_QP_CN_Entry.AsReg8s[2]<<16 |
    								 cursor->Local_QP_CN_Entry.AsReg8s[1]<<8 |
    								 cursor->Local_QP_CN_Entry.AsReg8s[0];
    		const uint32_t remoteQP = cursor->Remote_QP_Number_CN_Entry.AsReg8s[2]<<16 |
    								  cursor->Remote_QP_Number_CN_Entry.AsReg8s[1]<<8 |
    								  cursor->Remote_QP_Number_CN_Entry.AsReg8s[0];

    		PrintFunc(dest, "%*sEvent #%u:\n", indent+4, "", i + 1);
    		PrintFunc(dest, "%*sLocal QP CN: %u\n", indent+8, "", localQP);
    		PrintFunc(dest, "%*sRemote QP CN %u\n", indent+8, "", remoteQP);
    		PrintFunc(dest, "%*sLocal QP SL: %u\n", indent+8, "", cursor->SL_CN_Entry);
    		PrintFunc(dest, "%*sLocal QP Service Type: %s\n", indent+8, "", StlQPServiceTypeToText(cursor->Service_Type_CN_Entry));
    		PrintFunc(dest, "%*sRemote LID: %u\n", indent+8, "", cursor->Remote_LID_CN_Entry);
    		PrintFunc(dest, "%*sTimeStamp: %.3f uSec (0x%x)\n", indent+8, "", (float)cursor->TimeStamp_CN_Entry * 1.024f,
    						cursor->TimeStamp_CN_Entry);
    	}
    }
}

void PrintStlHfiCongestionControlTab(PrintDest_t *dest, int indent, const STL_HFI_CONGESTION_CONTROL_TABLE *pHfiCongestionControl, uint8_t cnt, uint8_t start, int printLineByLine)
{
    unsigned int block, i;
    char tempBuf[64];
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "CCTI_Limit", pHfiCongestionControl->CCTI_Limit);
		if (!pHfiCongestionControl->CCTI_Limit) {
			return;
		}
        for (block = 0; block < cnt; ++block) {
            for(i = 0; i < STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES; ++i){
                if (printLineByLine >= 2) {
                    snprintf(tempBuf, sizeof(tempBuf), "CCT_Block_List[%d].CCT_Entry_List[%d].AsReg16", block + start, i);
                    PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionControl->CCT_Block_List[block].CCT_Entry_List[i].AsReg16);
                }
                snprintf(tempBuf, sizeof(tempBuf), "CCT_Block_List[%d].CCT_Entry_List[%d].s.CCT_Shift", block + start, i);
                PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionControl->CCT_Block_List[block].CCT_Entry_List[i].s.CCT_Shift);
                snprintf(tempBuf, sizeof(tempBuf), "CCT_Block_List[%d].CCT_Entry_List[%d].s.CCT_Multiplier", block + start, i);
                PrintIntWithDots(dest, indent, tempBuf, pHfiCongestionControl->CCT_Block_List[block].CCT_Entry_List[i].s.CCT_Multiplier);
            }
        }
    } else {
    	PrintFunc(dest, "%*sMax CCTI: %u\n", indent, "", pHfiCongestionControl->CCTI_Limit);
		if (!pHfiCongestionControl->CCTI_Limit) {
			return;
		}
    	PrintFunc(dest, "%*sCCT Entries:\n", indent, "");
    	for (block = 0; block < cnt; ++block) {
            PrintFunc(dest, "%*sBlock %u:\n", indent+3, "", block + start);
            for (i = 0; i < STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES; ++i) {
                const STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY *cursor = &pHfiCongestionControl->CCT_Block_List[block].CCT_Entry_List[i];
                PrintFunc(dest, "%*sEntry %u: Shift %u, Multiplier: %u\n", indent+4, "", ((block + start) * STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES) + i, 
                    cursor->s.CCT_Shift, cursor->s.CCT_Multiplier);
            }
        }
    }
}

static void PrintStlSxSCMapHeading(PrintDest_t *dest, int indent, const char* prefix, const char* type)
{
	PrintFunc(dest, "%*s%s%s: 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31\n",
				indent, "", prefix, type);
}

void PrintStlSLSCMap(PrintDest_t *dest, int indent, const char* prefix, const STL_SLSCMAP *pSLSCMap, boolean heading)
{
	int i;
    if (heading)
        PrintStlSxSCMapHeading(dest, indent, prefix, "SL");
    PrintFunc(dest, "%*s%sSC: ", indent, "", prefix);

    for (i = 0; i < STL_MAX_SLS; i++)
        PrintFunc(dest, "%02u ", pSLSCMap->SLSCMap[i].SC);
    PrintFunc(dest, "\n");
}

void PrintStlSLSCMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, int printLineByLine)
{
	STL_SLSCMAP *pSLSCMap;
	int i;
    pSLSCMap = (STL_SLSCMAP *)stl_get_smp_data((STL_SMP *)smp);
    char tempBuf[64];

    if (printLineByLine) {
        for (i = 0; i < STL_MAX_SLS; i++) {
            snprintf(tempBuf, sizeof(tempBuf), "SLSCMap[%d].SC", i);
            PrintIntWithDots(dest, indent, tempBuf, pSLSCMap->SLSCMap[i].SC);
        }
    }
    else {
		PrintStlSxSCMapHeading(dest, indent, "", "SL");

    	PrintFunc(dest, "%*sSC: ", indent, "");

    	for (i = 0; i < STL_MAX_SLS; i++)
    		PrintFunc(dest, "%02u ", pSLSCMap->SLSCMap[i].SC);
    	PrintFunc(dest, "\n");
    }
}

void PrintStlSCSCMap(PrintDest_t *dest, int indent, const char* prefix, const STL_SCSCMAP *pSCSCMap, boolean heading)
{
	int i;
	if (heading)
		PrintStlSxSCMapHeading(dest, indent, prefix, "SC");
	PrintFunc(dest, "%*s%sSC':", indent, "", prefix);

	for (i = 0; i < STL_MAX_SCS; i++)
		PrintFunc(dest, "%02u ", pSCSCMap->SCSCMap[i].SC);

	PrintFunc(dest, "\n");
}

void PrintStlSCSCMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, boolean heading, int printLineByLine)
{
	int i;
	STL_SCSCMAP *pSCSCMap;
	const char *headingprefix = "";
    char tempBuf[64];
	uint32_t sub;
	uint32_t numBlocks = (smp->common.AttributeModifier >> 24) & 0xff;
    if (numBlocks==0) 
        numBlocks=1;

    pSCSCMap = (STL_SCSCMAP *)stl_get_smp_data((STL_SMP *)smp); 
	
    if (printLineByLine) {
        for (sub = 0; sub < numBlocks; sub++) {
            PrintIntWithDots(dest, indent, "blockNum", sub);
    		if (smp->common.AttributeModifier & 0x20000) {
                PrintStrWithDots(dest, indent, "Ingress Ports", "ALL");
            } else {
                PrintIntWithDots(dest, indent, "Ingress Port", (smp->common.AttributeModifier >> 8) & 0xff );
            }
            if (smp->common.AttributeModifier & 0x10000) {
                PrintStrWithDots(dest, indent, "Egress Ports", "ALL");
            } else {
                PrintIntWithDots(dest, indent, "Egress Port", (smp->common.AttributeModifier & 0xff) + sub);
            }
            for(i = 0; i < STL_MAX_SCS; i++) {
                snprintf(tempBuf, sizeof(tempBuf), "SCSCMap[%d].SC", i);
                PrintIntWithDots(dest, indent, tempBuf, pSCSCMap[sub].SCSCMap[i].SC);
            }
        }
    }
    else {
    	headingprefix = "InPort,OutPort|";
    	if (heading)
    		PrintStlSxSCMapHeading(dest, indent, headingprefix, "SC");

    	for (sub = 0; sub < numBlocks; sub++) {
    		char prefix[40] = "";
    		int offset;
    		if (smp->common.AttributeModifier & 0x20000) {
    			offset = sprintf(prefix, "  ALL ,");
    		} else {
    			offset = sprintf(prefix, "  %3u ,",
    					(smp->common.AttributeModifier >> 8) & 0xff);
    		}
    		if (smp->common.AttributeModifier & 0x10000) {
    			sprintf(&prefix[offset], "   ALL |");
    		} else {
    			sprintf(&prefix[offset], "   %3u |",
    				(smp->common.AttributeModifier & 0xff)
    					+ sub);
    		}
    		PrintStlSCSCMap(dest, indent, prefix, pSCSCMap + sub, FALSE);
    	}
    }
}

void PrintStlSCSLMap(PrintDest_t *dest, int indent, const char* prefix, const STL_SCSLMAP *pSCSLMap, boolean heading)
{
	int i;
	if (heading) {
		PrintStlSxSCMapHeading(dest, indent, "", "SC");

		PrintFunc(dest, "%*sSL: ", indent, "");
		for (i = 0; i< STL_MAX_SCS; i++) {
			PrintFunc(dest, "%02u ", pSCSLMap->SCSLMap[i].SL);
		}
		PrintFunc(dest, "\n");
	}
}

void PrintStlSCSLMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, int printLineByLine)
{
	STL_SCSLMAP *pSCSLMap;
	int i;
    pSCSLMap = (STL_SCSLMAP *)stl_get_smp_data((STL_SMP *)smp);
    char tempBuf[64];

    if (printLineByLine) {
        for (i = 0; i < STL_MAX_SCS; i++) {
            snprintf(tempBuf, sizeof(tempBuf), "SCSLMap[%d].SL", i);
            PrintIntWithDots(dest, indent, tempBuf, pSCSLMap->SCSLMap[i].SL);
        }
    }
    else {
		PrintStlSxSCMapHeading(dest, indent, "", "SC");

    	PrintFunc(dest, "%*sSL: ", indent, "");

    	for (i = 0; i < STL_MAX_SCS; i++)
    		PrintFunc(dest, "%02u ", pSCSLMap->SCSLMap[i].SL);
    	PrintFunc(dest, "\n");
    }
}

void PrintStlSCVLxMap(PrintDest_t *dest, int indent, const char* prefix,
			const STL_SCVLMAP *pSCVLMap, boolean heading,
			uint16_t attribute, int printLineByLine)
{
	int i;
    char tempBuf[64];
	const char* vlstr;
	switch (attribute) {
		case STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE:
			vlstr = " VLR";
			break;
		case STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE:
			vlstr = " VLT";
			break;
		case STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE:
			vlstr = "VLNT";
			break;
		default:
			vlstr = "UNKN";
	}
    if (printLineByLine) {
        PrintStrWithDots(dest, indent, "Type", vlstr);
        for (i = 0; i < STL_MAX_SCS; i++) {
            snprintf(tempBuf, sizeof(tempBuf), "SCVLMap[%d].VL", i);
            PrintIntWithDots(dest, indent, tempBuf, pSCVLMap->SCVLMap[i].VL);
        }
    }
    else {
    	if (heading)
			PrintStlSxSCMapHeading(dest, indent, "", "  SC");
    	PrintFunc(dest, "%*s%s%s: ", indent, "", prefix, vlstr);

    	for (i = 0; i < STL_MAX_SCS; i++)
    		PrintFunc(dest, "%02u ", pSCVLMap->SCVLMap[i].VL);
    	PrintFunc(dest, "\n");
    }
}

void PrintStlSCVLxMapSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, NODE_TYPE nodetype, boolean heading, int printLineByLine)
{
	STL_SCVLMAP *pSCVLMap;
	char prefix[40] = "";
	const char *headingprefix = "";
	uint32_t sub;
	uint32_t numBlocks = (smp->common.AttributeModifier >> 24) & 0xff;
	pSCVLMap = (STL_SCVLMAP *)stl_get_smp_data((STL_SMP *)smp);
	
	
    if (printLineByLine) {
        if (nodetype == STL_NODE_SW) {
    		if (smp->common.AttributeModifier & 0x100) {
                PrintStrWithDots(dest, indent, "Ports", "All");
            } 
            else {
                PrintIntWithDots(dest, indent, "Port", smp->common.AttributeModifier & 0xff);
            }
        }
        for (sub = 0; sub < numBlocks; sub++) {
    		PrintStlSCVLxMap(dest, indent, "", pSCVLMap + sub, FALSE,
    				smp->common.AttributeID, printLineByLine);
        }
    }
    else {
#if 0
        if (nodetype == STL_NODE_SW) {
    		int offset;

    		headingprefix = "StartPort,EndPort|";
    		if (smp->common.AttributeModifier & 0x100) {
    			offset = sprintf(prefix, "   ALL   ,");
    		} else {
    			offset = sprintf(prefix, "   %3u   ,", smp->common.AttributeModifier & 0xff);
    		}
    		sprintf(&prefix[offset], "   %3u |",
    				(smp->common.AttributeModifier & 0xff) + numBlocks - 1);
        }
#endif
        if (nodetype == STL_NODE_SW) {
            //            "Port ## |"
			headingprefix="         ";
        }
    	if (heading)
			PrintStlSxSCMapHeading(dest, indent, headingprefix, "  SC");
    	for (sub = 0; sub < numBlocks; sub++) {
            if (nodetype == STL_NODE_SW) {
                sprintf(prefix, "Port %2u |",
                        (smp->common.AttributeModifier & 0xff) + sub);
            }
    		PrintStlSCVLxMap(dest, indent, prefix, pSCVLMap + sub, FALSE,
    				smp->common.AttributeID, printLineByLine);
    	}
    }
}

void PrintStlSMInfo(PrintDest_t *dest, int indent, const STL_SM_INFO *pSMInfo, STL_LID lid, int printLineByLine)
{
    if (printLineByLine) {
        if (lid) {
            PrintIntWithDots(dest, indent, "LID", lid);
        }
        PrintIntWithDotsFull(dest, indent, "PortGUID", pSMInfo->PortGUID);
        PrintIntWithDots(dest, indent, "SM_Key", pSMInfo->SM_Key);
        PrintIntWithDots(dest, indent, "ActCount", pSMInfo->ActCount);
        PrintIntWithDots(dest, indent, "ElapsedTime", pSMInfo->ElapsedTime);
        PrintIntWithDots(dest, indent, "u.AsReg16", pSMInfo->u.AsReg16);
        PrintIntWithDots(dest, indent, "u.s.Priority", pSMInfo->u.s.Priority);
        PrintIntWithDots(dest, indent, "u.s.ElevatedPriority", pSMInfo->u.s.ElevatedPriority);
        PrintIntWithDots(dest, indent, "u.s.InitialPriority", pSMInfo->u.s.InitialPriority);
        PrintIntWithDots(dest, indent, "u.s.SMStateCurrent", pSMInfo->u.s.SMStateCurrent);
    }
    else {
    	if (lid)
    		PrintFunc(dest, "%*sLID: 0x%08x PortGuid: 0x%016"PRIx64" State: %s\n",
    				indent, "", lid, pSMInfo->PortGUID,
    				StlSMStateToText(pSMInfo->u.s.SMStateCurrent));
    	else
    		PrintFunc(dest, "%*sPortGuid: 0x%016"PRIx64" State: %s\n",
    				indent, "", pSMInfo->PortGUID,
    				StlSMStateToText(pSMInfo->u.s.SMStateCurrent));
    	PrintFunc(dest, "%*sPriority: %2d Initial Priority: %2d Elevated Priority: %2d\n",
    				indent, "", pSMInfo->u.s.Priority, pSMInfo->u.s.InitialPriority, pSMInfo->u.s.ElevatedPriority);
    	PrintFunc(dest, "%*sSM_Key: 0x%016"PRIx64" ActCount: 0x%08x\n",
    				indent, "", pSMInfo->SM_Key, pSMInfo->ActCount);
    	PrintFunc(dest, "%*sElapsed Time: %d seconds\n",
    				indent, "", pSMInfo->ElapsedTime);
    }
}

void PrintStlLinearFDB(PrintDest_t *dest, int indent, const STL_LINEAR_FORWARDING_TABLE *pLinearFDB, uint32 blockNum, int printLineByLine)
{
        int i;
        uint32 baselid = blockNum*MAX_LFT_ELEMENTS_BLOCK;
        char tempBuf[64];
        if (printLineByLine) {
            for (i=0; i< MAX_LFT_ELEMENTS_BLOCK; ++i) {
                if (pLinearFDB->LftBlock[i] != 0xFF) {
                    PrintIntWithDots(dest, indent, "LID", baselid+i);
                    snprintf(tempBuf, sizeof(tempBuf), "LftBlock[%d].Port", i);
                    PrintIntWithDots(dest, indent, tempBuf, pLinearFDB->LftBlock[i]);
                }
            }
        }
        else {
            for (i=0; i<MAX_LFT_ELEMENTS_BLOCK; ++i)
            {
                    // LID is index into overall FDB table
                    // 255 is invalid port, indicates an unused table entry
                    if (pLinearFDB->LftBlock[i] != 255)
                            PrintFunc(dest, "%*s  LID 0x%08x -> Port %5u\n",
                                    indent, "", baselid + i,
                                    pLinearFDB->LftBlock[i]);
            }
        }
}

void PrintStlVLArbTable(PrintDest_t *dest, int indent, const STL_VLARB_TABLE *pVLArbTable, uint32 section, int printLineByLine)
{
    int i, j;
    char buf[64];

    if (printLineByLine) {
        if (section == 0) PrintStrWithDots(dest, indent, "Table Type", "Low Priority");
    	else if (section == 1) PrintStrWithDots(dest, indent, "Table Type", "High Priority");
    	else if (section == 2) PrintStrWithDots(dest, indent, "Table Type", "Preemption Table");
    	else PrintStrWithDots(dest, indent, "Table Type", "Preemption Matrix");

        if (section < 3) {
            for (i = 0; i < VLARB_TABLE_LENGTH; i++) {
                snprintf(buf, sizeof(buf), "Elements[%d].s.VL", i);
                PrintIntWithDots(dest, indent, buf, pVLArbTable->Elements[i].s.VL);
                snprintf(buf, sizeof(buf), "Elements[%d].Weight", i);
                PrintIntWithDots(dest, indent, buf, pVLArbTable->Elements[i].Weight);
            }
        }
        else {
            for (i = 0; i < STL_MAX_VLS; i++) {
                snprintf(buf, sizeof(buf), "Matrix[%d]", i);
                PrintIntWithDots(dest, indent, buf, pVLArbTable->Matrix[i]);
            }

        }

    }
    else {
    	if (section == 0) PrintFunc(dest, "%*sTable Type: Low Priority\n", indent, "");
    	else if (section == 1) PrintFunc(dest, "%*sTable Type: High Priority\n", indent, "");
    	else if (section == 2) PrintFunc(dest, "%*sTable Type: Preemption Table\n", indent, "");
    	else PrintFunc(dest, "%*sTable Type: Preemption Matrix\n", indent, "");

    	if (section < 3)
    	{
    		PrintFunc(dest, "%*sIndx VL Weight\n", indent, "");
    		for (i = 0; i < VLARB_TABLE_LENGTH; ++i)
    		{
    			if (pVLArbTable->Elements[i].Weight == 0) continue;
    			PrintFunc(dest, "%*s%3d %2u %3u\n",
    				indent, "", i, pVLArbTable->Elements[i].s.VL,
    				pVLArbTable->Elements[i].Weight);
    		}		
    	}
    	else
    	{
    		PrintFunc(dest, "%*sVL | Preemptable VL's\n", indent, "");
    		for (i = 0; i < STL_MAX_VLS; ++i)
    		{
    			if ( pVLArbTable->Matrix[i] == 0)
    				continue;

    			PrintFunc(dest, "%*sVL%2u : ", indent, "", i);
    			for (j = 0; j < STL_MAX_VLS; ++j)
    				if ( pVLArbTable->Matrix[i] & 1<<j) PrintFunc(dest, "VL%2u ", j);
				PrintFunc(dest, "\n");
    		}
    	}
    }
}

void PrintStlVLArbTableSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t nodeType, int printLineByLine)
{
	STL_VLARB_TABLE *pVlArbTab = (STL_VLARB_TABLE *)stl_get_smp_data((STL_SMP *)smp);
	int numPorts = (smp->common.AttributeModifier >> 24) & 0xff;
	int section = (smp->common.AttributeModifier >> 16) & 0xff;
	int port = smp->common.AttributeModifier & 0xff;
	int i;
    if (printLineByLine) {
        for (i = 0; i < numPorts; i++) {
    	    BSWAP_STL_VLARB_TABLE(pVlArbTab, section);
            if (nodeType == STL_NODE_SW) {
	      PrintIntWithDots(dest, indent, "Port", port+i);
              PrintStlVLArbTable(dest, indent+4, pVlArbTab, section, printLineByLine);
            } else {
    	      PrintStlVLArbTable(dest, indent, pVlArbTab, section, printLineByLine);
            }
    	    pVlArbTab++;
        }
    }
    else {
    	for (i = 0; i < numPorts; i++) {
    		BSWAP_STL_VLARB_TABLE(pVlArbTab, section);
    		if (nodeType == STL_NODE_SW) {
		  PrintFunc(dest, "%*sPort: %d\n", indent, "", (int)port + i);
		  PrintStlVLArbTable(dest, indent+4, pVlArbTab, section, printLineByLine);
         	} else {
    		PrintStlVLArbTable(dest, indent, pVlArbTab, section, printLineByLine);
                }
    		pVlArbTab++;
        }
    }
}



void PrintStlMCastSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t pos, uint32_t startBlk, uint8_t count, int printLineByLine)
{
	uint8_t block;
	STL_MULTICAST_FORWARDING_TABLE *pMCastBlock;

	if (printLineByLine)
		PrintIntWithDots(dest, indent, "pos", pos);
	else {
		const uint8_t startPort = pos * STL_PORT_MASK_WIDTH;
		PrintFunc(dest, "Position %u (ports %u-%u)\n", pos, startPort, startPort + STL_PORT_MASK_WIDTH - 1);
	}

	for (block = 0, pMCastBlock = (STL_MULTICAST_FORWARDING_TABLE*)stl_get_smp_data((STL_SMP*)smp);
			block < count; ++block, ++pMCastBlock) {

		BSWAP_STL_MULTICAST_FORWARDING_TABLE(pMCastBlock);

		PrintStlMCastFDB(dest, indent + (printLineByLine ? 0 : 4), pMCastBlock, startBlk + block, printLineByLine);
	}
}

void PrintStlMCastFDB(PrintDest_t *dest, int indent, const STL_MULTICAST_FORWARDING_TABLE *pMCastFDB, uint32 blockNum, int printLineByLine)
{
	int i;
	STL_LID baselid = blockNum * STL_NUM_MFT_ELEMENTS_BLOCK + STL_LID_MULTICAST_BEGIN;
    char tempBuf[64];
    if (printLineByLine) {
        for (i=0; i< STL_NUM_MFT_ELEMENTS_BLOCK; ++i) {
            if (pMCastFDB->MftBlock[i]) {
                PrintIntWithDots(dest, indent, "LID", baselid+i);
                snprintf(tempBuf, sizeof(tempBuf), "MftBlock[%d].PortMask", i);
                PrintIntWithDots(dest, indent, tempBuf, pMCastFDB->MftBlock[i]);
            }
        }
    }
    else {
    	for (i=0; i <STL_NUM_MFT_ELEMENTS_BLOCK; ++i)
    	{
    		if (pMCastFDB->MftBlock[i])
    		{
    			PrintFunc(dest, "%*s LID 0x%08x -> PortMask 0x%016"PRIx64"\n",
    				indent, "", baselid+i, pMCastFDB->MftBlock[i]);
    		}
    	}
    }
}                                                                                  

void PrintStlInformInfo(PrintDest_t *dest, int indent, 
					 	const STL_INFORM_INFO *pInformInfo)
{
	if (pInformInfo->IsGeneric)
	{
		char buf[8];
		PrintFunc(dest, "%*s%sSubscribe: Type: %8s Generic: Trap: %5d "
				"QPN: 0x%06x\n",
				indent, "",
				pInformInfo->Subscribe?"":"Un",
				IbNoticeTypeToText(pInformInfo->Type),
				pInformInfo->u.Generic.TrapNumber,
				pInformInfo->u.Generic.u1.s.QPNumber);
		FormatTimeoutMult(buf, pInformInfo->u.Generic.u1.s.RespTimeValue);
		PrintFunc(dest, "%*sRespTime: %s Producer: %s\n",
				indent, "", buf,
				StlNodeTypeToText(pInformInfo->u.Generic.u2.s.ProducerType));
	} else {
		char buf[8];
		FormatTimeoutMult(buf, pInformInfo->u.Generic.u1.s.RespTimeValue);
		PrintFunc(dest, "%*s%sSubscribe: Type: %8s VendorId: 0x%04x DeviceId: 0x%04x QPN: 0x%06x\n",
				indent, "",
				pInformInfo->Subscribe?"":"Un",
				IbNoticeTypeToText(pInformInfo->Type),
				pInformInfo->u.Vendor.u2.s.VendorID,
				pInformInfo->u.Vendor.DeviceID,
				pInformInfo->u.Vendor.u1.s.QPNumber);
		PrintFunc(dest, "%*sRespTime: %s\n",
				indent, "",
				buf);
	}
	PrintFunc(dest, "%*sGID: 0x%016"PRIx64":0x%016"PRIx64
				"  LIDs: 0x%08x - 0x%08x\n",
				indent, "",
				pInformInfo->GID.AsReg64s.H,
				pInformInfo->GID.AsReg64s.L,
				pInformInfo->LIDRangeBegin,
				pInformInfo->LIDRangeEnd);
}

void PrintStlBfrCtlTable(PrintDest_t *dest, int indent, const STL_BUFFER_CONTROL_TABLE *pBfrCtlTable, int printLineByLine)
{
    int i = 0;
    char tempBuf[64];
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "TxOverallSharedLimit", pBfrCtlTable->TxOverallSharedLimit);
        for (i = 0; i < STL_MAX_VLS; i++) {
            snprintf(tempBuf, sizeof(tempBuf), "VL[%d].TxDedicatedLimit", i);
            PrintIntWithDots(dest, indent, tempBuf, pBfrCtlTable->VL[i].TxDedicatedLimit);
            snprintf(tempBuf, sizeof(tempBuf), "VL[%d].TxSharedLimit", i);
            PrintIntWithDots(dest, indent, tempBuf, pBfrCtlTable->VL[i].TxSharedLimit);
        }
    }
    else {
      PrintFunc(dest, "%*sGlobal Shared(AU): %d\n", indent, "", pBfrCtlTable->TxOverallSharedLimit);
        for (i = 0; i < STL_MAX_VLS; i++)
            PrintFunc(dest, "%*sVL%2u : D: %8d, S: %8d\n", indent, "", i, pBfrCtlTable->VL[i].TxDedicatedLimit, pBfrCtlTable->VL[i].TxSharedLimit);
    }
}

void PrintStlBfrCtlTableSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t nodeType, int printLineByLine)
{
	uint8_t * data = stl_get_smp_data((STL_SMP *)smp);
	STL_BUFFER_CONTROL_TABLE *table = (STL_BUFFER_CONTROL_TABLE *)(data);
	int numPorts = (smp->common.AttributeModifier >> 24) & 0xff;
	int port = smp->common.AttributeModifier & 0xff;
	int i = 0;
	if (printLineByLine) {
		for (i = 0; i < numPorts; i++) {
			BSWAP_STL_BUFFER_CONTROL_TABLE(table);
			PrintIntWithDots(dest, indent, "Port", (int)(port+i));
			PrintStlBfrCtlTable(dest, indent, table, printLineByLine);
			PrintSeparator(dest);
			// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
			data += STL_BFRCTRLTAB_PAD_SIZE;
			table = (STL_BUFFER_CONTROL_TABLE *)(data);
		}
	}
	else {
		for (i = 0; i < numPorts; i++) {
			BSWAP_STL_BUFFER_CONTROL_TABLE(table);
			PrintFunc(dest, "%*sPort: %2d: ", indent, "", (int)(port+i));
			PrintStlBfrCtlTable(dest, indent, table, printLineByLine);
			PrintSeparator(dest);
			// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
			data += STL_BFRCTRLTAB_PAD_SIZE;
			table = (STL_BUFFER_CONTROL_TABLE *)(data);
		}
	}
}

void PrintStlCableInfoLowPage(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint8_t detail, int printLineByLine)
{
	float temp;
	char tempBuf[129];
	char tempBuf2[129];
	char temperature[15]={0};
	char vcc[15]={0};
	boolean qsfp_dd = (cableInfoData[0] == STL_CIB_STD_QSFP_DD);
	
	if (qsfp_dd)
		temp = ((cableInfoData[14] << 8) + (cableInfoData[15]))/256.0;
	else
		temp = ((cableInfoData[22] << 8) + (cableInfoData[23]))/256.0;
	if (temp) {
		snprintf(temperature, sizeof(temperature), "%.2f C", temp);
	}
	else {
		snprintf(temperature, sizeof(temperature), "not indicated");
	}
	if (qsfp_dd)
		temp = ((cableInfoData[16] << 8) + (cableInfoData[17]))/10000.0;
	else
		temp = ((cableInfoData[26] << 8) + (cableInfoData[27]))/10000.0;
	if (temp) {
		snprintf(vcc, sizeof(vcc), "%.2f V", temp);
	}
	else {
		snprintf(vcc, sizeof(vcc), "not indicated");
	}

	// get connection type
	StlCableInfoDecodeConnType(cableInfoData[0], tempBuf2);

	// For lower page, print temperature and vcc
	switch (detail) {
		case CABLEINFO_DETAIL_ONELINE:
		case CABLEINFO_DETAIL_SUMMARY:
		case CABLEINFO_DETAIL_BRIEF:
		case CABLEINFO_DETAIL_VERBOSE:
			if (printLineByLine) {
				StringCopy(tempBuf, tempBuf2, strlen(tempBuf2));
				strncat(tempBuf2, ": Direct CableInfo", strlen(": Direct CableInfo") + 1);
				PrintStrWithDots(dest, indent, tempBuf2, "");
				PrintStrWithDots(dest, indent, "Temperature", temperature);
				PrintStrWithDots(dest, indent, "Supply Voltage", vcc);
				break;
			}
			else {
				memset(tempBuf, ' ', sizeof(tempBuf));
				snprintf(tempBuf, sizeof(tempBuf), "%s: %s  %s", tempBuf2, temperature, vcc);
				PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);
				break;
			}
		case CABLEINFO_DETAIL_ALL:
		default:
			if (printLineByLine) {
				StringCopy(tempBuf, tempBuf2, strlen(tempBuf2));
				strncat(tempBuf2, ": Direct CableInfo", strlen(": Direct CableInfo") + 1);
				PrintStrWithDots(dest, indent, tempBuf2, "");
				PrintStrWithDots(dest, indent, "Temperature", temperature);
				PrintStrWithDots(dest, indent, "Supply Voltage", vcc);
				break;
			}
			else {
				PrintFunc(dest, "%*s%s Interpreted CableInfo:\n", indent, "", tempBuf2);
				PrintFunc(dest, "%*sTemperature: %s\n", indent+4, "", temperature);
				PrintFunc(dest, "%*sSupply Voltage: %s\n", indent+4, "", vcc);
				break;
			}
		}
}

#define MAX_CABLE_LENGTH_STR_LEN 8		// ~2-3 digits (poss. decimal pt) plus 'm'

void PrintStlCableInfoHighPage(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint8_t portType, uint8_t detail, int printLineByLine)
{
	unsigned int i;
	boolean cableLenValid;			// Copper cable length valid
	char tempBuf[129];
	char tempStr[STL_CIB_STD_MAX_STRING + 1] = {'\0'};
	STL_CABLE_INFO_STD* cableInfo;
	CableTypeInfoType cableTypeInfo;
	cableInfo = (STL_CABLE_INFO_STD*)cableInfoData;
	
	StlCableInfoDecodeCableType(cableInfo->dev_tech.s.xmit_tech, cableInfo->connector, cableInfo->ident, &cableTypeInfo);
	cableLenValid = cableTypeInfo.cableLengthValid;

	// Output CableInfo fields per detail level
	switch (detail) {
	case CABLEINFO_DETAIL_ONELINE:
	case CABLEINFO_DETAIL_SUMMARY:
	case CABLEINFO_DETAIL_BRIEF:
	case CABLEINFO_DETAIL_VERBOSE:
		if (printLineByLine) {
			// Build ONELINE output line-by-line
			strncpy(tempBuf, cableTypeInfo.cableTypeShortDesc, strlen(cableTypeInfo.cableTypeShortDesc));
			PrintStrWithDots(dest, indent, "CableType", tempBuf);
			StlCableInfoOM4LengthToText(cableInfo->len_om4, cableLenValid, sizeof(tempBuf), tempBuf);
			PrintStrWithDots(dest, indent, "CableLength", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_name));
			PrintStrWithDots(dest, indent, "CableVendorName", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_pn));
			PrintStrWithDots(dest, indent, "CableVendorPN", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_rev));
			PrintStrWithDots(dest, indent, "CableVendorRev", tempBuf);
			if (detail == CABLEINFO_DETAIL_ONELINE)
				break;

			// Build line 2 of SUMMARY output line-by-line
			PrintStrWithDots( dest, indent, "CablePowerClass",
				StlCableInfoPowerClassToText(cableInfo->ext_ident.s.pwr_class_low, cableInfo->ext_ident.s.pwr_class_high) );
			memcpy(tempBuf, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_sn));
			PrintStrWithDots(dest, indent, "CableVendorSN", tempBuf);
			StlCableInfoDateCodeToText(cableInfo->date_code, tempBuf);
			PrintStrWithDots(dest, indent, "CableDateCode", tempBuf);

			// Build line 3 of SUMMARY output line-by-line
			sprintf(tempBuf, "0x%02X%02X%02X", cableInfo->vendor_oui[0], cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			PrintStrWithDots(dest, indent, "CableOUI", tempBuf);
			break;
		}
		else {
			// Build ONELINE output on one line (68 chars)
			memset(tempBuf, ' ', sizeof(tempBuf));
			i = 0;
			strcpy(&tempBuf[i], "      ");
			i = STL_CIB_LINE1_FIELD1;
			strncpy(&tempBuf[i], cableTypeInfo.cableTypeShortDesc, strlen(cableTypeInfo.cableTypeShortDesc));
			tempBuf[i + strlen(cableTypeInfo.cableTypeShortDesc)] = ',';
			tempBuf[i + 1 + strlen(cableTypeInfo.cableTypeShortDesc)] = ' ';
			i = STL_CIB_LINE1_FIELD2; 
			StlCableInfoOM4LengthToText(cableInfo->len_om4, cableLenValid, MAX_CABLE_LENGTH_STR_LEN, &tempBuf[i]);
			tempBuf[i + strlen(&tempBuf[i])] = ' ';
			i = STL_CIB_LINE1_FIELD3;
			memcpy(tempStr, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_name));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE1_FIELD4;
			strcpy(&tempBuf[i], "P/N ");
			i = STL_CIB_LINE1_FIELD5;
			memcpy(tempStr, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_pn));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE1_FIELD6;
			strcpy(&tempBuf[i], "Rev ");
			i = STL_CIB_LINE1_FIELD7;
			memcpy(tempStr, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_rev));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE1_FIELD8;
			tempBuf[i] = '\0';
			PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);
			if (detail == CABLEINFO_DETAIL_ONELINE)
				break;

			// Build line 2 of SUMMARY output on one line (68 chars)
			memset(tempBuf, ' ', sizeof(tempBuf));
			i = STL_CIB_LINE2_FIELD1;
			strcpy(&tempBuf[i], StlCableInfoPowerClassToText(cableInfo->ext_ident.s.pwr_class_low, cableInfo->ext_ident.s.pwr_class_high));
			tempBuf[i + strlen(&tempBuf[i])] = ' ';
			i = STL_CIB_LINE2_FIELD2;
			strcpy(&tempBuf[i], "S/N ");
			i = STL_CIB_LINE2_FIELD3;
			memcpy(tempStr, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_sn));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE2_FIELD4;
			strcpy(&tempBuf[i], "Mfg ");
			i = STL_CIB_LINE2_FIELD5;
			StlCableInfoDateCodeToText(cableInfo->date_code, &tempBuf[i]);
			PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);

			// Build line 3 of SUMMARY output on one line (68 chars)
			memset(tempBuf, ' ', sizeof(tempBuf));
			i = STL_CIB_LINE3_FIELD1;
			sprintf(&tempBuf[i], "OUI 0x%02X%02X%02X", cableInfo->vendor_oui[0], cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);
			break;
		}

	case CABLEINFO_DETAIL_ALL:
	default:
		if (printLineByLine) {
			if (portType != STL_PORT_TYPE_STANDARD) return;
			PrintStrWithDots(dest, indent, "QSFP Direct CableInfo", "");
			PrintIntWithDots(dest, indent, "Identifier", cableInfo->ident);
			PrintStrWithDots(dest, indent, "PowerClass",
				StlCableInfoPowerClassToText(cableInfo->ext_ident.s.pwr_class_low, cableInfo->ext_ident.s.pwr_class_high));
			PrintStrWithDots(dest, indent, "Connector", cableTypeInfo.connectorType);
			StlCableInfoBitRateToText(cableInfo->bit_rate_low, cableInfo->bit_rate_high, tempBuf);
			PrintStrWithDots(dest, indent, "NominalBR", tempBuf);
			StlCableInfoCableTypeToTextLong(cableInfo->dev_tech.s.xmit_tech, cableInfo->connector, tempBuf);
			PrintStrWithDots(dest, indent, "DeviceTech", tempBuf);
			memcpy(tempStr, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_name));
			PrintStrWithDots(dest, indent, "VendorName", tempStr);
			snprintf(tempStr, sizeof(tempStr), "0x%02x%02x%02x", cableInfo->vendor_oui[0],
				cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			PrintStrWithDots(dest, indent, "VendorOUI", tempStr);
			memcpy(tempStr, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_pn));
			PrintStrWithDots(dest, indent, "VendorPN", tempStr);
			memcpy(tempStr, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_rev));
			PrintStrWithDots(dest, indent, "VendorRev", tempStr);
			PrintIntWithDots(dest, indent, "CC_BASE", cableInfo->cc_base);
			memcpy(tempStr, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_sn));
			PrintStrWithDots(dest, indent, "VendorSN", tempStr);
			StlCableInfoDateCodeToText(cableInfo->date_code, tempBuf);
			PrintStrWithDots(dest, indent, "DateCode", tempBuf);
			PrintIntWithDots(dest, indent, "CC_EXT", cableInfo->cc_ext);
			break;
		}
		else {
			if (portType != STL_PORT_TYPE_STANDARD) return;
			PrintFunc(dest, "%*sQSFP Interpreted CableInfo:\n", indent, "");
			PrintFunc(dest, "%*sIdentifier: 0x%x\n", indent+4, "", cableInfo->ident);
			PrintFunc(dest, "%*sPowerClass: %s\n", indent+4, "", 
				StlCableInfoPowerClassToText(cableInfo->ext_ident.s.pwr_class_low, cableInfo->ext_ident.s.pwr_class_high));
			memcpy(tempBuf, cableTypeInfo.connectorType, sizeof(cableTypeInfo.connectorType));
			PrintFunc(dest, "%*sConnector: %s\n", indent+4, "", tempBuf);
			StlCableInfoBitRateToText(cableInfo->bit_rate_low, cableInfo->bit_rate_high, tempBuf);
			PrintFunc(dest, "%*sNominalBR: %s\n", indent+4, "", tempBuf);
			StlCableInfoCableTypeToTextLong(cableInfo->dev_tech.s.xmit_tech, cableInfo->connector, tempBuf);
			PrintFunc(dest, "%*sDeviceTech: %s\n", indent+4, "", tempBuf);
			memcpy(tempStr, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_name));
			PrintFunc(dest, "%*sVendorName: %s\n", indent+4, "", tempStr); 
			PrintFunc(dest, "%*sVendorOUI: 0x%02x%02x%02x\n", indent+4, "", cableInfo->vendor_oui[0],
				cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			memcpy(tempStr, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_pn));
			PrintFunc(dest, "%*sVendorPN: %s\n", indent+4, "", tempStr); 
			memcpy(tempStr, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_rev));
			PrintFunc(dest, "%*sVendorRev: %s\n", indent+4, "", tempStr); 
			PrintFunc(dest, "%*sCC_BASE: 0x%x\n", indent+4, "", cableInfo->cc_base);
			memcpy(tempStr, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_sn));
			PrintFunc(dest, "%*sVendorSN: %s\n", indent+4, "", tempStr);
			StlCableInfoDateCodeToText(cableInfo->date_code, tempBuf);
			PrintFunc(dest, "%*sDateCode: %s\n", indent+4, "", tempBuf);
			PrintFunc(dest, "%*sCC_EXT: 0x%x\n", indent+4, "", cableInfo->cc_ext);
			break;
		}

	}	// End of switch (detail)


}

// Print for upper page 0 for QSFP-DD
void PrintStlCableInfoHighPage0DD(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint8_t portType, uint8_t detail, int printLineByLine)
{
	unsigned int i;
	boolean cableLenValid;			// Copper cable length valid
	char tempBuf[129];
	char tempStr[STL_CIB_STD_MAX_STRING + 1] = {'\0'};
	STL_CABLE_INFO_UP0_DD* cableInfo;
	CableTypeInfoType cableTypeInfo;
	cableInfo = (STL_CABLE_INFO_UP0_DD*)cableInfoData;

	StlCableInfoDecodeCableType(cableInfo->cable_type, cableInfo->connector, cableInfo->ident, &cableTypeInfo);
	cableLenValid = cableTypeInfo.cableLengthValid;

	// Output CableInfo fields per detail level
	switch (detail) {
	case CABLEINFO_DETAIL_ONELINE:
	case CABLEINFO_DETAIL_SUMMARY:
	case CABLEINFO_DETAIL_BRIEF:
	case CABLEINFO_DETAIL_VERBOSE:
		if (printLineByLine) {
			// Build ONELINE output line-by-line
			strncpy(tempBuf, cableTypeInfo.cableTypeShortDesc, strlen(cableTypeInfo.cableTypeShortDesc));
			PrintStrWithDots(dest, indent, "CableType", tempBuf);
			StlCableInfoDDCableLengthToText(cableInfo->cableLengthEnc, cableLenValid, sizeof(tempBuf), tempBuf);
			PrintStrWithDots(dest, indent, "CableLength", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_name));
			PrintStrWithDots(dest, indent, "CableVendorName", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_pn));
			PrintStrWithDots(dest, indent, "CableVendorPN", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_rev));
			PrintStrWithDots(dest, indent, "CableVendorRev", tempBuf);
			if (detail == CABLEINFO_DETAIL_ONELINE)
				break;

			// Build line 2 of SUMMARY output line-by-line
			snprintf(tempBuf, 20, "%.2f W max", (float)cableInfo->powerMax / 4.0);
			PrintStrWithDots( dest, indent, "CableMaxPower", tempBuf);
			memcpy(tempBuf, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempBuf, sizeof(cableInfo->vendor_sn));
			PrintStrWithDots(dest, indent, "CableVendorSN", tempBuf);
			StlCableInfoDateCodeToText(cableInfo->date_code, tempBuf);
			PrintStrWithDots(dest, indent, "CableDateCode", tempBuf);

			// Build line 3 of SUMMARY output line-by-line
			sprintf(tempBuf, "0x%02X%02X%02X", cableInfo->vendor_oui[0], cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			PrintStrWithDots(dest, indent, "CableOUI", tempBuf);
			break;
		}
		else {
			// Build ONELINE output on one line (68 chars)
			memset(tempBuf, ' ', sizeof(tempBuf));
			i = 0;
			strcpy(&tempBuf[i], "      ");
			i = STL_CIB_LINE1_FIELD1;
			strncpy(&tempBuf[i], cableTypeInfo.cableTypeShortDesc, strlen(cableTypeInfo.cableTypeShortDesc));
			tempBuf[i + strlen(cableTypeInfo.cableTypeShortDesc)] = ',';
			tempBuf[i + 1 + strlen(cableTypeInfo.cableTypeShortDesc)] = ' ';
			i = STL_CIB_LINE1_FIELD2; 
			StlCableInfoDDCableLengthToText(cableInfo->cableLengthEnc, cableLenValid, sizeof(tempBuf), tempBuf);
			tempBuf[i + strlen(&tempBuf[i])] = ' ';
			i = STL_CIB_LINE1_FIELD3;
			memcpy(tempStr, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_name));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE1_FIELD4;
			strcpy(&tempBuf[i], "P/N ");
			i = STL_CIB_LINE1_FIELD5;
			memcpy(tempStr, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_pn));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE1_FIELD6;
			strcpy(&tempBuf[i], "Rev ");
			i = STL_CIB_LINE1_FIELD7;
			memcpy(tempStr, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_rev));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE1_FIELD8;
			tempBuf[i] = '\0';
			PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);
			if (detail == CABLEINFO_DETAIL_ONELINE)
				break;

			// Build line 2 of SUMMARY output on one line (68 chars)
			memset(tempBuf, ' ', sizeof(tempBuf));
			i = STL_CIB_LINE2_FIELD1;
			snprintf(&tempBuf[i], 20, "%.2f W max", (float)cableInfo->powerMax / 4.0);
			tempBuf[i + strlen(&tempBuf[i])] = ' ';
			i = STL_CIB_LINE2_FIELD2;
			strcpy(&tempBuf[i], "S/N ");
			i = STL_CIB_LINE2_FIELD3;
			memcpy(tempStr, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_sn));
			memcpy(&tempBuf[i], tempStr, strlen(tempStr));
			i = STL_CIB_LINE2_FIELD4;
			strcpy(&tempBuf[i], "Mfg ");
			i = STL_CIB_LINE2_FIELD5;
			StlCableInfoDateCodeToText(cableInfo->date_code, &tempBuf[i]);
			PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);

			// Build line 3 of SUMMARY output on one line (68 chars)
			memset(tempBuf, ' ', sizeof(tempBuf));
			i = STL_CIB_LINE3_FIELD1;
			sprintf(&tempBuf[i], "OUI 0x%02X%02X%02X", cableInfo->vendor_oui[0], cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			PrintFunc(dest, "%*s%s\n", indent, "", tempBuf);
			break;
		}

	case CABLEINFO_DETAIL_ALL:
	default:
		if (printLineByLine) {
			if (portType != STL_PORT_TYPE_STANDARD) return;
			PrintStrWithDots(dest, indent, "QSFP Direct CableInfo", "");
			PrintIntWithDots(dest, indent, "Identifier", cableInfo->ident);
			snprintf(tempBuf, 20, "%.2f W max", (float)cableInfo->powerMax / 4.0);
			PrintStrWithDots(dest, indent, "CableMaxPower", tempBuf);
			memcpy(tempBuf, cableTypeInfo.connectorType, sizeof(cableTypeInfo.connectorType));
			PrintStrWithDots(dest, indent, "Connector", tempBuf);
			snprintf(tempBuf, 4, "n/a");
			PrintStrWithDots(dest, indent, "NominalBR", tempBuf);
			StlCableInfoCableTypeToTextLong(cableInfo->cable_type, cableInfo->connector, tempBuf);
			PrintStrWithDots(dest, indent, "DeviceTech", tempBuf);
			memcpy(tempStr, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_name));
			PrintStrWithDots(dest, indent, "VendorName", tempStr);
			snprintf(tempStr, sizeof(tempStr), "0x%02x%02x%02x", cableInfo->vendor_oui[0],
				cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			PrintStrWithDots(dest, indent, "VendorOUI", tempStr);
			memcpy(tempStr, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_pn));
			PrintStrWithDots(dest, indent, "VendorPN", tempStr);
			memcpy(tempStr, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_rev));
			PrintStrWithDots(dest, indent, "VendorRev", tempStr);
			memcpy(tempStr, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_sn));
			PrintStrWithDots(dest, indent, "VendorSN", tempStr);
			StlCableInfoDateCodeToText(cableInfo->date_code, tempBuf);
			PrintStrWithDots(dest, indent, "DateCode", tempBuf);
			break;
		}
		else {
			if (portType != STL_PORT_TYPE_STANDARD) return;
			PrintFunc(dest, "%*sQSFP Interpreted CableInfo:\n", indent, "");
			PrintFunc(dest, "%*sIdentifier: 0x%x\n", indent+4, "", cableInfo->ident);
			snprintf(tempBuf, 20, "%.2f W max", (float)cableInfo->powerMax / 4.0);
			PrintFunc(dest, "%*sCableMaxPower: %s\n", indent+4, "", tempBuf);
			memcpy(tempBuf, cableTypeInfo.connectorType, sizeof(cableTypeInfo.connectorType));
			PrintFunc(dest, "%*sConnector: %s\n", indent+4, "", tempBuf);
			StlCableInfoCableTypeToTextLong(cableInfo->cable_type, cableInfo->connector, tempBuf);
			PrintFunc(dest, "%*sDeviceTech: %s\n", indent+4, "", tempBuf);
			memcpy(tempStr, cableInfo->vendor_name, sizeof(cableInfo->vendor_name));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_name));
			PrintFunc(dest, "%*sVendorName: %s\n", indent+4, "", tempStr); 
			PrintFunc(dest, "%*sVendorOUI: 0x%02x%02x%02x\n", indent+4, "", cableInfo->vendor_oui[0],
				cableInfo->vendor_oui[1], cableInfo->vendor_oui[2]);
			memcpy(tempStr, cableInfo->vendor_pn, sizeof(cableInfo->vendor_pn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_pn));
			PrintFunc(dest, "%*sVendorPN: %s\n", indent+4, "", tempStr); 
			memcpy(tempStr, cableInfo->vendor_rev, sizeof(cableInfo->vendor_rev));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_rev));
			PrintFunc(dest, "%*sVendorRev: %s\n", indent+4, "", tempStr); 
			memcpy(tempStr, cableInfo->vendor_sn, sizeof(cableInfo->vendor_sn));
			StlCableInfoTrimTrailingWS(tempStr, sizeof(cableInfo->vendor_sn));
			PrintFunc(dest, "%*sVendorSN: %s\n", indent+4, "", tempStr);
			StlCableInfoDateCodeToText(cableInfo->date_code, tempBuf);
			PrintFunc(dest, "%*sDateCode: %s\n", indent+4, "", tempBuf);
			break;
		}

	}	// End of switch (detail)

	return;
}

void PrintStlCableInfoDump(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint16_t addr, uint8_t len, int printLineByLine)
{
	unsigned int i, j;
	char tempBuf[129];
	char tempVal[64];
	if (printLineByLine) {
		PrintStrWithDots(dest, indent, "CableInfo Dump of Received Address and Data", "");
		for (i = 0; i * 16 < len; ++i) {
			int strOffset = 0;
			// Print 16 bytes per line
			snprintf(tempBuf, sizeof(tempBuf), "addr[%04X]", addr + i * 16);
			for (j = 0; j < MIN(16, len - i * 16); ++j)
				strOffset += snprintf( &tempVal[strOffset], 64-strOffset, "%s%02x",
					(j?",":""), cableInfoData[i * 16 + j] );
			PrintStrWithDots(dest, indent, tempBuf, tempVal);
		}
	}
	else {
		PrintFunc(dest, "%*sCableInfo Dump of Received Address and Data:\n", indent, "");
		for (i = 0; i * 16 < len; ++i) {
			// Print 16 bytes per line
			PrintFunc(dest, "%*s%8u:", indent, "", addr + i * 16);
			for (j = 0; j < MIN(16, len - i * 16); ++j)
				PrintFunc(dest, " %.2x", cableInfoData[i * 16 + j]);
			PrintFunc(dest, "\n");
		}
	}
}

//  Print CableInfo data at specified detail level and in specified format
//
//    len                 - Real length of cable data (must be <= 255)
//    detail              - Output detail level as CABLEINFO_DETAIL_xxx
//    printLineByLine = 1 - Print line-by-line format
void PrintStlCableInfo(PrintDest_t *dest, int indent, const uint8_t *cableInfoData, uint16_t addr, uint8_t len, uint8_t portType, uint8_t detail, int printLineByLine, boolean hexOnly)
{
	boolean qsfp_dd;

	if (cableInfoData)
		qsfp_dd = (cableInfoData[0] == STL_CIB_STD_QSFP_DD);
	else {
		fprintf(stderr, "PrintStlCableInfo: cableInfoData pointer is invalid\n");
		return;
	}

	if (hexOnly) {
		PrintStlCableInfoDump(dest, indent, cableInfoData, addr, len, printLineByLine);
	}
	else if (addr==STL_CIB_STD_LOW_PAGE_ADDR && len==STL_CIB_STD_LEN) {
		if (detail >= CABLEINFO_DETAIL_ALL) {
			PrintStlCableInfoDump(dest, indent, cableInfoData, addr, len, printLineByLine);
		}
		PrintStlCableInfoLowPage(dest, indent, cableInfoData, detail, printLineByLine);
	}
	else if (addr==STL_CIB_STD_HIGH_PAGE_ADDR && len==STL_CIB_STD_LEN){
		if (detail >= CABLEINFO_DETAIL_ALL) {
			PrintStlCableInfoDump(dest, indent, cableInfoData, addr, len, printLineByLine);
		}
		if (qsfp_dd)
			PrintStlCableInfoHighPage0DD(dest, indent, cableInfoData, portType, detail, printLineByLine);	
		else
			PrintStlCableInfoHighPage(dest, indent, cableInfoData, portType, detail, printLineByLine);	
	}
	else {
		PrintFunc(dest, "%*sOnly raw hex cable data available - Addr:%u Len:%u\n", indent, "", addr, len);
		PrintStlCableInfoDump(dest, indent, cableInfoData, addr, len, printLineByLine);
	}

}	// End of PrintStlCableInfo()

void PrintStlCableInfoSmp(PrintDest_t *dest, int indent, const STL_SMP *smp, uint8_t portType, int printLineByLine)
{
	STL_CABLE_INFO *pCableInfo = (STL_CABLE_INFO*)stl_get_smp_data((STL_SMP*)smp);
	const uint16_t addr = (smp->common.AttributeModifier>>19) & 0x0fff;
	const uint8_t len = (smp->common.AttributeModifier>>13) & 0x3f;
	
    if (printLineByLine) {
        PrintIntWithDots(dest, indent, "Starting Address", addr);
        PrintIntWithDots(dest, indent, "Length", len+1);
        PrintStrWithDots(dest, indent, "Port Type", StlPortTypeToText(portType));
    }
    else {
        PrintFunc(dest, "%*sStarting Address: 0x%x (%u)\nLength: %u\nPort Type: %s\n",
				indent, "",
				addr, addr, len+1, StlPortTypeToText(portType));
    }
	BSWAP_STL_CABLE_INFO(pCableInfo);
	PrintStlCableInfo(dest, indent, pCableInfo->Data, addr, len+1, portType, CABLEINFO_DETAIL_ALL, printLineByLine,FALSE);
}

void PrintStlPortGroupFDB(PrintDest_t *dest, int indent, const STL_PORT_GROUP_FORWARDING_TABLE *pPortGroupFDB, uint32 blockNum, int printLineByLine)
{
	int i;
	STL_LID baselid = blockNum*NUM_PGFT_ELEMENTS_BLOCK;
    char tempBuf[64];
    if (printLineByLine) {
    	for (i=0; i<NUM_PGFT_ELEMENTS_BLOCK; ++i) {
    		// LID is index into overall FDB table
    		// 255 is invalid port, indicates an unsed table entry
    		if (pPortGroupFDB->PgftBlock[i] != 255) {
                PrintIntWithDots(dest, indent, "LID", baselid +i);
                snprintf(tempBuf, sizeof(tempBuf), "PgftBlock[%i].Port", i);
                PrintIntWithDots(dest, indent, tempBuf, pPortGroupFDB->PgftBlock[i]);
            }
    	}
    }
    else {
    	for (i=0; i<NUM_PGFT_ELEMENTS_BLOCK; ++i) {
    		// LID is index into overall FDB table
    		// 255 is invalid port, indicates an unsed table entry
    		if (pPortGroupFDB->PgftBlock[i] != 255)
    			PrintFunc(dest, "%*s  LID 0x%08x -> PortGroup %3u\n",
    				indent, "", baselid + i,
    				pPortGroupFDB->PgftBlock[i]);
    	}
    }
}


void PrintStlPortGroupTable(PrintDest_t *dest, int indent, const uint64_t *pPortGroup, uint32 blockNum, int position, int printLineByLine)
{
	int i, j;
    position *= 64;  // Move to the right positional port number
    uint64 msk = 0;
    
    if (printLineByLine) {
    	for (i=0; i<NUM_PGT_ELEMENTS_BLOCK; ++i) {
            PrintIntWithDots(dest, indent, "PortGroup", (blockNum*NUM_PGT_ELEMENTS_BLOCK) + i);
            PrintIntWithDots(dest, indent, "PortMask", pPortGroup[i]);
        }
    }
    else {
    	for (i=0; i<NUM_PGT_ELEMENTS_BLOCK; ++i) {
            if (pPortGroup[i]==0) continue;
            PrintFunc(dest, "%*s  PG: 0x%04x   Egress:", indent, "", (blockNum*NUM_PGT_ELEMENTS_BLOCK) + i);
            for (msk = 1, j = 0;j < 64;msk<<= 1, j++)
                if (pPortGroup[i] & msk) PrintFunc(dest, "%d, ", j+position+1);
            PrintFunc(dest, "\n");
        }
    }
}

// len is real length of cable data
void PrintStlPortSummary(PrintDest_t *dest, int indent, const char* portName, const STL_PORT_INFO *pPortInfo, EUI64 portGuid, uint16_t pkey, const uint8_t *cableInfoData, uint16_t addr, uint8_t len, const STL_PORT_STATUS_RSP *pPortStatusRsp, uint8_t detail, int printLineByLine)
{
	// By design, there are two formats. The default format (printLineByLine = 0) puts the 
	// most useful information first
	// The second format (printLineByLine = 1) dumps the record in a line-by-line format
	// more suitable for record content analysis. 
	enum {
		SHOW_RATE_NONE,
		SHOW_RATE_EN, 		// show en/sup speed/widths, and LCRC en/sup
		SHOW_RATE_ACT 		// show act and en speed/widths, LCRC and mgmt
	} show_rate = SHOW_RATE_NONE;
	int show_mgmt = 0;		// show mgmt as part of SHOW_RATE_ACT
	int show_address = 0;	// show local addresses
	int show_cable = 0;		// show cable info if available
	int show_perf = 0; 		// show basic signal integrity errors and utilization
	uint8_t ldr=0;

	switch (pPortInfo->PortStates.s.PortState) {
	default:
	case IB_PORT_DOWN:
		show_rate = SHOW_RATE_EN;
		show_mgmt = 0;
		show_address = 0;
		show_perf = 0;

		if (pPortInfo->PortPhysConfig.s.PortType == STL_PORT_TYPE_STANDARD
			&& pPortInfo->PortStates.s.OfflineDisabledReason != STL_OFFDIS_REASON_LOCAL_MEDIA_NOT_INSTALLED
			&& pPortInfo->PortStates.s.OfflineDisabledReason != STL_OFFDIS_REASON_SWITCH_MGMT
			&& pPortInfo->PortStates.s.OfflineDisabledReason != STL_OFFDIS_REASON_SMA_DISABLED) {
			show_cable = 1;
		}
		break;
	case IB_PORT_INIT:
		show_rate = SHOW_RATE_ACT;
		if (pPortInfo->PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW
							// my neighbor is PRR
			&& pPortInfo->NeighborPortNum) // only switch port 0 has no neighbor
			show_mgmt = 1;	// pkey in Init reflects MgmtAllowed for links w/sw
		else
			show_mgmt = 0;	// omit for HFI to HFI link, and switch port 0
							// Mgmt is uncertain til armed
		show_address = 0;
		if (pPortInfo->PortPhysConfig.s.PortType == STL_PORT_TYPE_STANDARD)
			show_cable = 1;
		show_perf = 1;
		break;
	case IB_PORT_ARMED:
	case IB_PORT_ACTIVE:
		show_rate = SHOW_RATE_ACT;
		show_mgmt = 1;	// once Armed, pkey reflects MgmtAllowed or in Admin VF
		show_address = 1;
		if (pPortInfo->PortPhysConfig.s.PortType == STL_PORT_TYPE_STANDARD)
			show_cable = 1;
		show_perf = 1;
		break;
	}

	if(pPortInfo->LinkDownReason != 0)
		ldr = pPortInfo->LinkDownReason; 
	else if(pPortInfo->NeighborLinkDownReason != 0)
		ldr = pPortInfo->NeighborLinkDownReason;

	if (printLineByLine) {
		// this is for scripting, so its easier for calling scripts if we
		// always return all related fields from the summary
		PrintStrWithDots(dest, indent, "PortName", portName);
		if (portGuid)
			PrintIntWithDotsFull(dest, indent, "PortGUID", portGuid);
		if (show_address)
			PrintIntWithDots(dest, indent, "SubnetPrefix", pPortInfo->SubnetPrefix);
		if (pPortInfo->PortStates.s.PortState == IB_PORT_DOWN) {
			PrintIntWithDots(dest, indent, "PortStates.s.PortPhysicalState", (uint32)pPortInfo->PortStates.s.PortPhysicalState);
			if (ldr) {
				PrintIntWithDots(dest, indent, "LinkDownReason", (uint32)pPortInfo->LinkDownReason);
				PrintIntWithDots(dest, indent, "NeighborLinkDownReason", (uint32)pPortInfo->NeighborLinkDownReason);
			}

			if (pPortInfo->PortStates.s.PortPhysicalState == STL_PORT_PHYS_OFFLINE
				|| pPortInfo->PortStates.s.PortPhysicalState == IB_PORT_PHYS_DISABLED)
				PrintIntWithDots(dest, indent, "PortStates.s.OfflineDisabledReason", (uint32)pPortInfo->PortStates.s.OfflineDisabledReason);
		} else {
			PrintIntWithDots(dest, indent, "PortStates.s.PortState", (uint32)pPortInfo->PortStates.s.PortState);
			if (pPortInfo->PortStates.s.PortState == IB_PORT_INIT)
				PrintIntWithDots(dest, indent, "LinkInitReason", (uint32)pPortInfo->s3.LinkInitReason);
			if (pPortInfo->PortStates.s.PortState == IB_PORT_ARMED)
				PrintIntWithDots(dest, indent, "PortStates.s.NeighborNormal", (uint32)pPortInfo->PortStates.s.NeighborNormal);
		}

		switch (show_rate) {
		case SHOW_RATE_NONE:
			break;
		case SHOW_RATE_EN:
			PrintIntWithDots(dest, indent, "LinkSpeed.Enabled", (uint32)pPortInfo->LinkSpeed.Enabled);
			PrintIntWithDots(dest, indent, "LinkSpeed.Supported", (uint32)pPortInfo->LinkSpeed.Supported); 
			PrintIntWithDots(dest, indent, "LinkWidth.Enabled", (uint32)pPortInfo->LinkWidth.Enabled);
			PrintIntWithDots(dest, indent, "LinkWidth.Supported", (uint32)pPortInfo->LinkWidth.Supported);
			PrintIntWithDots(dest, indent, "LinkWidthDowngrade.Enabled", (uint32)pPortInfo->LinkWidthDowngrade.Enabled);
			PrintIntWithDots(dest, indent, "LinkWidthDowngrade.Supported", (uint32)pPortInfo->LinkWidthDowngrade.Supported);
			PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Enabled", (uint32)pPortInfo->PortLTPCRCMode.s.Enabled);
			PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Supported", (uint32)pPortInfo->PortLTPCRCMode.s.Supported);
			break;
		case SHOW_RATE_ACT:
			PrintIntWithDots(dest, indent, "LinkSpeed.Active", (uint32)pPortInfo->LinkSpeed.Active);
			PrintIntWithDots(dest, indent, "LinkSpeed.Enabled", (uint32)pPortInfo->LinkSpeed.Enabled);
			PrintIntWithDots(dest, indent, "LinkWidth.Active", (uint32)pPortInfo->LinkWidth.Active);
			PrintIntWithDots(dest, indent, "LinkWidth.Enabled", (uint32)pPortInfo->LinkWidth.Enabled);
			PrintIntWithDots(dest, indent, "LinkWidthDowngrade.Enabled", (uint32)pPortInfo->LinkWidthDowngrade.Enabled);
			PrintIntWithDots(dest, indent, "LinkWidthDowngrade.TxActive", (uint32)pPortInfo->LinkWidthDowngrade.TxActive);
			PrintIntWithDots(dest, indent, "LinkWidthDowngrade.RxActive", (uint32)pPortInfo->LinkWidthDowngrade.RxActive);
			PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Enabled", (uint32)pPortInfo->PortLTPCRCMode.s.Enabled);
			PrintIntWithDots(dest, indent, "PortLTPCRCMode.s.Active", (uint32)pPortInfo->PortLTPCRCMode.s.Active);
			if (show_mgmt)
				PrintStrWithDots(dest, indent, "Mgmt", (pkey == 0xffff)?"True":"False");
			break;
		}
		if (show_address) {
			PrintIntWithDots(dest, indent, "LID", (uint32)pPortInfo->LID);
			PrintIntWithDots(dest, indent, "LMC", (uint32)pPortInfo->s1.LMC);
			PrintIntWithDots(dest, indent, "MasterSMLID", (uint32)pPortInfo->MasterSMLID);
			PrintIntWithDots(dest, indent, "MasterSMSL", (uint32)pPortInfo->s2.MasterSMSL);
		}
	} else {
		char tempBuf[64];
		char tempBuf2[64];
		char tempBuf3[64];
		char tempBuf4[64];

		// portName can be up to 68 characters, but typically much smaller
		if (portGuid && show_address)
			PrintFunc(dest, "%*s%-34s PortGID:0x%016"PRIx64":%016"PRIx64"\n",
				indent, "", portName, pPortInfo->SubnetPrefix, portGuid);
		else if (portGuid)
			PrintFunc(dest, "%*s%-34s PortGUID:0x%016"PRIx64"\n",
				indent, "", portName, portGuid);
		else
			PrintFunc(dest, "%*s%-34s\n",
				indent, "", portName);

		indent += 3;

		switch (pPortInfo->PortStates.s.PortState) {
		default:
		case IB_PORT_DOWN:
			if (pPortInfo->PortStates.s.PortPhysicalState == STL_PORT_PHYS_OFFLINE
				|| pPortInfo->PortStates.s.PortPhysicalState == IB_PORT_PHYS_DISABLED) {
				if (ldr)
					PrintFunc(dest, "%*sPhysicalState: %-14s (%s) OfflineDisabledReason: %-14s\n",
						indent, "",
						StlPortPhysStateToText(pPortInfo->PortStates.s.PortPhysicalState),
						StlLinkDownReasonToText(ldr),
						StlPortOfflineDisabledReasonToText(pPortInfo->PortStates.s.OfflineDisabledReason));
				else
					PrintFunc(dest, "%*sPhysicalState: %-14s   OfflineDisabledReason: %-14s\n",
						indent, "",
						StlPortPhysStateToText(pPortInfo->PortStates.s.PortPhysicalState),
						StlPortOfflineDisabledReasonToText(pPortInfo->PortStates.s.OfflineDisabledReason));
			} else {
				if (ldr)
					PrintFunc(dest, "%*sPhysicalState: %-14s (%s)\n",
						indent, "",
						StlPortPhysStateToText(pPortInfo->PortStates.s.PortPhysicalState),
						StlLinkDownReasonToText(ldr));
				else
					PrintFunc(dest, "%*sPhysicalState: %-14s\n",
						indent, "",
						StlPortPhysStateToText(pPortInfo->PortStates.s.PortPhysicalState));
			}
			break;
		case IB_PORT_INIT:
			if (pPortInfo->s3.LinkInitReason)
				PrintFunc(dest, "%*sPortState:     %s (%s)\n",
					indent, "",
					StlPortStateToText(pPortInfo->PortStates.s.PortState),
					StlLinkInitReasonToText(pPortInfo->s3.LinkInitReason));
			else
				PrintFunc(dest, "%*sPortState:     %s\n",
					indent, "",
					StlPortStateToText(pPortInfo->PortStates.s.PortState));
			break;
		case IB_PORT_ARMED:
			PrintFunc(dest, "%*sPortState:     %s NeighborNormal: %s\n",
				indent, "",
				StlPortStateToText(pPortInfo->PortStates.s.PortState),
				pPortInfo->PortStates.s.NeighborNormal?"True":"False");
			break;
		case IB_PORT_ACTIVE:
			PrintFunc(dest, "%*sPortState:     %s\n",
				indent, "",
				StlPortStateToText(pPortInfo->PortStates.s.PortState));
			break;
		}
		switch (show_rate) {
		case SHOW_RATE_NONE:
			break;
		case SHOW_RATE_EN:
			// TBD - combine width and speed into one line
			// 16 char too long
			// remove "Link" = 8, Gb->g = 2, 12s->11s = 2
			PrintFunc(dest, "%*sLinkSpeed      En: %-12s Sup: %-12s\n",
				indent,"",
				StlLinkSpeedToText(pPortInfo->LinkSpeed.Enabled, tempBuf, sizeof(tempBuf2)),
				StlLinkSpeedToText(pPortInfo->LinkSpeed.Supported, tempBuf2, sizeof(tempBuf)));
			PrintFunc(dest, "%*sLinkWidth      En: %-12s Sup: %-12s\n",
				indent,"",
				StlLinkWidthToText(pPortInfo->LinkWidth.Enabled, tempBuf3, sizeof(tempBuf4)),
				StlLinkWidthToText(pPortInfo->LinkWidth.Supported, tempBuf4, sizeof(tempBuf3)));
			PrintFunc(dest, "%*sLCRC           En: %-8s     Sup: %-8s\n", 
				indent, "",
				StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Enabled, tempBuf, sizeof(tempBuf)), 
				StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Supported, tempBuf3, sizeof(tempBuf3)));
			break;
		case SHOW_RATE_ACT:
			// TBD - combine width and speed into one line
			PrintFunc(dest, "%*sLinkSpeed      Act: %-12s En: %-12s\n",
				indent,"",
				StlLinkSpeedToText(pPortInfo->LinkSpeed.Active, tempBuf, sizeof(tempBuf)),
				StlLinkSpeedToText(pPortInfo->LinkSpeed.Enabled, tempBuf2, sizeof(tempBuf3)));
			PrintFunc(dest, "%*sLinkWidth      Act: %-12s En: %-12s\n",
				indent,"",
				StlLinkWidthToText(pPortInfo->LinkWidth.Active, tempBuf3, sizeof(tempBuf)),
				StlLinkWidthToText(pPortInfo->LinkWidth.Enabled, tempBuf4, sizeof(tempBuf3)));
			PrintFunc(dest, "%*sLinkWidthDnGrd ActTx: %-2.2s Rx: %-2.2s  En: %-12s\n",
				indent,"",
				StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.TxActive, tempBuf, sizeof(tempBuf)),
				StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.RxActive, tempBuf4, sizeof(tempBuf4)),
				StlLinkWidthToText(pPortInfo->LinkWidthDowngrade.Enabled, tempBuf3, sizeof(tempBuf3)));
			if (show_mgmt) {
				PrintFunc(dest, "%*sLCRC           Act: %-8s     En: %-8s       Mgmt: %-5s\n", 
					indent, "",
					StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Active, tempBuf, sizeof(tempBuf)), 
					StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Enabled, tempBuf3, sizeof(tempBuf3)),
					(pkey == 0xffff)?"True":"False");
			} else {
				PrintFunc(dest, "%*sLCRC           Act: %-8s     En: %-8s\n", 
					indent, "",
					StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Active, tempBuf, sizeof(tempBuf)), 
					StlPortLtpCrcModeToText(pPortInfo->PortLTPCRCMode.s.Enabled, tempBuf3, sizeof(tempBuf3)));
			}
			break;
		}

		if (show_address) {
			PrintFunc(dest, "%*sLID: 0x%08x-0x%08x       SM LID: 0x%08x SL: %-2u\n",
				indent, "", pPortInfo->LID,
				pPortInfo->LID + (1<<pPortInfo->s1.LMC)-1,
				pPortInfo->MasterSMLID, pPortInfo->s2.MasterSMSL);
		}
	}
	if (show_cable && cableInfoData) {
		PrintStlCableInfo(dest, indent, cableInfoData, addr, len, pPortInfo->PortPhysConfig.s.PortType, detail, printLineByLine, FALSE);
	}
	if (show_perf && pPortStatusRsp) {
		PrintStlPortStatusRspSummary(dest, indent, pPortStatusRsp, printLineByLine);
	}
}

