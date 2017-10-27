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
//
// FILE NAME
//    sm_counters.c
//
// DESCRIPTION
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//
// DEPENDENCIES
//
//
//===========================================================================//

#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include "sm_l.h"
#include "sm_counters.h"
#ifdef __VXWORKS__
#include "UiUtil.h"
#endif

// time stamp for SM counters
time_t smCountersClearedTime = 0;

//
// Initializers for SM counters.
// If you added a counter, to the sm_counters_t enumeration, you should
// add a description here.
//
sm_counter_t smCounters[smCountersMax] = {
	// State Transitions
	[smCounterSmStateToDiscovering]     = { "SM State transition to DISCOVERY", 0, 0, 0 },
	[smCounterSmStateToMaster]          = { "SM State transition to MASTER", 0, 0, 0 },
	[smCounterSmStateToStandby]         = { "SM State transition to STANDBY", 0, 0, 0 },
	[smCounterSmStateToInactive]        = { "SM State transition to INACTIVE", 0, 0, 0 },

	// Basic Packet Info
	[smCounterSmPacketTransmits]        = { "Total transmitted SMA Packets", 0, 0, 0 },
	[smCounterDirectRoutePackets]       = { "Direct Routed SMA Packets", 0, 0, 0 },
	[smCounterLidRoutedPackets]         = { "LID Routed SMA Packets", 0, 0, 0 },
	[smCounterPacketRetransmits]        = { "SMA Query Retransmits", 0, 0, 0 },
	[smCounterSmTxRetriesExhausted]     = { "SMA Query Retransmits Exhausted", 0, 0, 0 },

	// SMA Method/Attributes
	[smCounterGetNotice]                = { "SM TX GET(Notice)", 0, 0, 0 },
	[smCounterSetNotice]                = { "SM TX SET(Notice)", 0, 0, 0 },
	
	// Traps Received
	[smCounterTrapsReceived]            = { "Total SM RX TRAP(Notice)", 0, 0, 0 },
	[smCounterTrapPortUp]				= { "SM RX TRAP(PortUp)", 0, 0, 0 },
	[smCounterTrapPortDown]				= { "SM RX TRAP(PortDown)", 0, 0, 0 },
	[smCounterTrapMcGrpCreate]			= { "SM RX TRAP(McGrpCreate)", 0, 0, 0 },
	[smCounterTrapMcGrpDel]				= { "SM RX TRAP(McGrpDelete)", 0, 0, 0 },
	[smCounterTrapUnPath]				= { "SM RX TRAP(UnPath)", 0, 0, 0 },
	[smCounterTrapRePath]				= { "SM RX TRAP(RePath)", 0, 0, 0 },
	[smCounterTrapPortStateChg]			= { "SM RX TRAP(PortStateChg)", 0, 0, 0 },
	[smCounterTrapLinkIntegrity]		= { "SM RX TRAP(LinkIntegrity)", 0, 0, 0 },
	[smCounterTrapBufOverrun]			= { "SM RX TRAP(BufferOverun)", 0, 0, 0 },
	[smCounterTrapFlowControl]			= { "SM RX TRAP(FlowControl)", 0, 0, 0 },
	[smCounterTrapLocalChg]				= { "SM RX TRAP(LocalChange)", 0, 0, 0 },
	[smCounterTrapSysImgChg]			= { "SM RX TRAP(SysImageChg)", 0, 0, 0 },
	[smCounterTrapBadMKey]				= { "SM RX TRAP(BadMKey)", 0, 0, 0 },
	[smCounterTrapBadPKey]				= { "SM RX TRAP(BadPkey)", 0, 0, 0 },
	[smCounterTrapBadQKey]				= { "SM RX TRAP(BadQkey)", 0, 0, 0 },
	[smCounterTrapBadPKeySwPort]		= { "SM RX TRAP(BadPkeySwPort)", 0, 0, 0 },
	[smCounterTrapLinkWidthDowngrade]	= { "SM RX TRAP(LinkWidthDowngrade)", 0, 0, 0 },
	[smCounterTrapCostMatrixChange]		= { "SM RX TRAP(CostMatrixChange)", 0, 0, 0 },

	[smCounterTrapsRepressed]           = { "SM TX TRAPREPRESS(Notice)", 0, 0, 0 },
	[smCounterGetNodeDescription]       = { "SM TX GET(NodeDescription)", 0, 0, 0 },
	[smCounterGetNodeInfo]              = { "SM TX GET(NodeInfo)", 0, 0, 0 },
	[smCounterGetSwitchInfo]            = { "SM TX GET(SwitchInfo)", 0, 0, 0 },
	[smCounterSetSwitchInfo]            = { "SM TX SET(SwitchInfo)", 0, 0, 0 },
	[smCounterGetPortInfo]              = { "SM TX GET(PortInfo)", 0, 0, 0 },
	[smCounterSetPortInfo]              = { "SM TX SET(PortInfo)", 0, 0, 0 },
	[smCounterGetPortStateInfo]			= { "SM TX GET(PortStateInfo)", 0, 0, 0 },
	[smCounterSetPortStateInfo]			= { "SM TX SET(PortStateInfo)", 0, 0, 0 },
	[smCounterGetPKeyTable]             = { "SM TX GET(PKeyTable)", 0, 0, 0 },
	[smCounterSetPKeyTable]             = { "SM TX SET(PKeyTable)", 0, 0, 0 },
	[smCounterGetSL2SCMappingTable]     = { "SM TX GET(SL2SCMappingTable)", 0, 0, 0 },
	[smCounterSetSL2SCMappingTable]     = { "SM TX SET(SL2SCMappingTable)", 0, 0, 0 },
	[smCounterGetSC2SLMappingTable]     = { "SM TX GET(SC2SLMappingTable)", 0, 0, 0 },
	[smCounterSetSC2SLMappingTable]     = { "SM TX SET(SC2SLMappingTable)", 0, 0, 0 },
	[smCounterGetSC2VLtMappingTable]     = { "SM TX GET(SC2VLtMappingTable)", 0, 0, 0 },
	[smCounterSetSC2VLtMappingTable]     = { "SM TX SET(SC2VLtMappingTable)", 0, 0, 0 },
	[smCounterGetSC2VLntMappingTable]     = { "SM TX GET(SC2VLntMappingTable)", 0, 0, 0 },
	[smCounterSetSC2VLntMappingTable]     = { "SM TX SET(SC2VLntMappingTable)", 0, 0, 0 },
	[smCounterGetSC2VLrMappingTable]     = { "SM TX GET(SC2VLrMappingTable)", 0, 0, 0 },
	[smCounterSetSC2VLrMappingTable]     = { "SM TX SET(SC2VLrMappingTable)", 0, 0, 0 },
	[smCounterGetSC2SCMappingTable]     = { "SM TX GET(SC2SCMappingTable)", 0, 0, 0 },
	[smCounterSetSC2SCMappingTable]     = { "SM TX SET(SC2SCMappingTable)", 0, 0, 0 },
	[smCounterGetVLArbitrationTable]    = { "SM TX GET(VLArbitrationTable)", 0, 0, 0 },
	[smCounterSetVLArbitrationTable]    = { "SM TX SET(VLArbitrationTable)", 0, 0, 0 },
	[smCounterGetLft]                   = { "SM TX GET(LFT)", 0, 0, 0 },
	[smCounterSetLft]                   = { "SM TX SET(LFT)", 0, 0, 0 },
	[smCounterGetMft]                   = { "SM TX GET(MFT)", 0, 0, 0 },
	[smCounterSetMft]                   = { "SM TX SET(MFT)", 0, 0, 0 },
	[smCounterGetAggregate]             = { "SM TX GET(AGGREGATE)", 0, 0, 0 },
	[smCounterSetAggregate]             = { "SM TX SET(AGGREGATE)", 0, 0, 0 },
	[smCounterGetLedInfo]               = { "SM TX GET(LedInfo)", 0, 0, 0 },
	[smCounterSetLedInfo]               = { "SM TX SET(LedInfo)", 0, 0, 0 },
	[smCounterGetCableInfo]             = { "SM TX GET(CableInfo)", 0, 0, 0 },
	[smCounterRxGetSmInfo]              = { "SM RX GET(SmInfo)", 0, 0, 0 },
	[smCounterGetSmInfo]                = { "SM TX GET(SmInfo)", 0, 0, 0 },
	[smCounterRxSetSmInfo]              = { "SM RX SET(SmInfo)", 0, 0, 0 },
	[smCounterSetSmInfo]                = { "SM TX SET(SmInfo)", 0, 0, 0 },
	[smCounterTxGetRespSmInfo]          = { "SM TX GETRESP(SmInfo)", 0, 0, 0 },
	[smCounterGetPG]                    = { "SM TX GET(PortGroup)", 0, 0, 0 },
	[smCounterSetPG]                    = { "SM TX SET(PortGroup)", 0, 0, 0 },
	[smCounterGetPGft]                  = { "SM TX GET(PGFT)", 0, 0, 0 },
	[smCounterSetPGft]                  = { "SM TX SET(PGFT)", 0, 0, 0 },
	[smCounterGetBufferControlTable]    = { "SM TX GET(BufferControlTable)", 0, 0, 0 },
	[smCounterSetBufferControlTable]    = { "SM TX SET(BufferControlTable)", 0, 0, 0 },
	[smCounterGetARLidMask]             = { "SM TX GET(ARLidMask)", 0, 0, 0 },
	[smCounterSetARLidMask]             = { "SM TX SET(ARLidMask)", 0, 0, 0 },

	// CCA SMA Counters
	[smCounterGetCongestionInfo]		= { "SM RX GET(CongestionInfo)", 0, 0, 0},
	[smCounterGetHfiCongestionSetting]	= { "SM RX GET(HfiCongestSetting)", 0, 0, 0},
	[smCounterSetHfiCongestionSetting]	= { "SM RX SET(HfiCongestSetting)", 0, 0, 0},
	[smCounterGetHfiCongestionControl]	= { "SM RX GET(HfiCongestControl)", 0, 0, 0},
	[smCounterSetHfiCongestionControl]	= { "SM RX SET(HfiCongestControl)", 0, 0, 0},
	[smCounterGetSwitchCongestionSetting]	= { "SM RX GET(SwCongestSetting)", 0, 0, 0},
	[smCounterSetSwitchCongestionSetting]	= { "SM RX SET(SwCongestSetting)", 0, 0, 0},
	[smCounterGetSwitchPortCongestionSetting]={ "SM RX GET(SwPortCongestionSetting", 0, 0, 0},

	[smCounterRxGetResp]                = { "SM RX GETRESP(*)", 0, 0, 0 },

	// SMA Response Status Codes
	[smCounterRxSmaMadStatusBusy]       = { "SM RX STATUS BUSY", 0, 0, 0 },
	[smCounterRxSmaMadStatusRedirect]   = { "SM RX STATUS REDIRECT", 0, 0, 0 },
	[smCounterRxSmaMadStatusBadClass]   = { "SM RX STATUS BADCLASS", 0, 0, 0 },
	[smCounterRxSmaMadStatusBadMethod]  = { "SM RX STATUS BADMETHOD", 0, 0, 0 },
	[smCounterRxSmaMadStatusBadAttr]    = { "SM RX STATUS BADMETHODATTR", 0, 0, 0 },
	[smCounterRxSmaMadStatusBadField]   = { "SM RX STATUS BADFIELD", 0, 0, 0 },
	[smCounterRxSmaMadStatusUnknown]    = { "SM RX STATUS UNKNOWN", 0, 0, 0 },

	// SA Queries
	[smCounterSaRxGetClassPortInfo]     = { "SA RX GET(ClassPortInfo)", 0, 0, 0 },
	[smCounterSaTxReportNotice]         = { "SA TX REPORT(Notice)", 0, 0, 0 },
	[smCounterSaRxSetInformInfo]        = { "SA RX SET(InformInfo)", 0, 0, 0 },
	[smCounterSaRxGetNodeRecord]        = { "SA RX GET(NodeRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblNodeRecord]     = { "SA RX GETTBL(NodeRecord)", 0, 0, 0 },
	[smCounterSaRxGetPortInfoRecord]    = { "SA RX GET(PortInfo)", 0, 0, 0 },
	[smCounterSaRxGetTblPortInfoRecord] = { "SA RX GETTBL(PortInfo)", 0, 0, 0 },

	[smCounterSaRxGetSc2ScMappingRecord]= { "SA RX GET(SC2SCMAP)", 0, 0, 0 },
	[smCounterSaRxGetTblSc2ScMappingRecord] = { "SA RX GETTBL(SC2SCMap)", 0, 0, 0 },
	[smCounterSaRxGetSl2ScMappingRecord]= { "SA RX GET(SL2SCMAP)", 0, 0, 0 },
	[smCounterSaRxGetTblSl2ScMappingRecord] = { "SA RX GETTBL(SL2SCMap)", 0, 0, 0 },
	[smCounterSaRxGetSc2SlMappingRecord]= { "SA RX GET(SC2SLMap)", 0, 0, 0 },
	[smCounterSaRxGetTblSc2SlMappingRecord]= { "SA RX GETTBL(SC2SLMap)", 0, 0, 0 },
	[smCounterSaRxGetSc2VltMappingRecord]= { "SA RX GET(SC2VLtMap)", 0, 0, 0 },
	[smCounterSaRxGetTblSc2VltMappingRecord]= { "SA RX GETTBL(SC2VLtMap)", 0, 0, 0 },
	[smCounterSaRxGetSc2VlntMappingRecord]= { "SA RX GET(SC2VLntMap)", 0, 0, 0 },
	[smCounterSaRxGetTblSc2VlntMappingRecord]= { "SA RX GETTBL(SC2VLntMap)", 0, 0, 0 },
	[smCounterSaRxGetSc2VlrMappingRecord]= { "SA RX GET(SC2VLrMap)", 0, 0, 0 },
	[smCounterSaRxGetTblSc2VlrMappingRecord]= { "SA RX GETTBL(SC2VLrMap)", 0, 0, 0 },

	[smCounterSaRxGetSwitchInfoRecord]  = { "SA RX GET(SwitchInfo)", 0, 0, 0 },
	[smCounterSaRxGetTblSwitchInfoRecord] = { "SA RX GETTBLE(SwitchInfo)", 0, 0, 0 },
	[smCounterSaRxGetLftRecord]         = { "SA RX GET(LFT)", 0, 0, 0 },
	[smCounterSaRxGetTblLftRecord]      = { "SA RX GETTBL(LFT)", 0, 0, 0 },
	[smCounterSaRxGetMftRecord]         = { "SA RX GET(MFT)", 0, 0, 0 },
	[smCounterSaRxGetTblMftRecord]      = { "SA RX GETTBL(MFT)", 0, 0, 0 },

	[smCounterSaRxGetVlArbTableRecord]  = { "SA RX GET(VLArbitration)", 0, 0, 0 },
	[smCounterSaRxGetTblVlArbTableRecord] = { "SA RX GETTBL(VLArbitration)", 0, 0, 0 },
	[smCounterSaRxGetSmInfoRecord]      = { "SA RX GET(SmInfo)", 0, 0, 0 },
	[smCounterSaRxGetTblSmInfoRecord]   = { "SA RX GETTBL(SmInfo)", 0, 0, 0 },
	[smCounterSaRxGetInformInfoRecord]  = { "SA RX GET(InformInfo)", 0, 0, 0 },
	[smCounterSaRxGetTblInformInfoRecord] = { "SA RX GETTBL(InformInfo)", 0, 0, 0 },

	[smCounterSaRxGetLinkRecord]        = { "SA RX GET(LinkRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblLinkRecord]     = { "SA RX GETTBL(LinkRecord)", 0, 0, 0 },
	[smCounterSaRxGetServiceRecord]     = { "SA RX GET(ServiceRecord)", 0, 0, 0 },
	[smCounterSaRxSetServiceRecord]     = { "SA RX SET(ServiceRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblServiceRecord]  = { "SA RX GETTBL(ServiceRecord)", 0, 0, 0 },
	[smCounterSaRxDeleteServiceRecord]  = { "SA RX DELETE(ServiceRecord)", 0, 0, 0 },

	[smCounterSaRxGetPKeyTableRecord]   = { "SA RX GET(PKeyRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblPKeyTableRecord]= { "SA RX GETTBL(PKeyRecord)", 0, 0, 0 },
	[smCounterSaRxGetPathRecord]        = { "SA RX GET(PathRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblPathRecord]     = { "SA RX GETTBL(PathRecord)", 0, 0, 0 },
	[smCounterSaRxGetMcMemberRecord]    = { "SA RX GET(McMemberRecord)", 0, 0, 0 },
	[smCounterSaRxSetMcMemberRecord]    = { "SA RX SET(McMemberRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblMcMemberRecord] = { "SA RX GETTBL(McMemberRecord)", 0, 0, 0 },
	[smCounterSaRxDeleteMcMemberRecord] = { "SA RX DELETE(McMemberRecord)", 0, 0, 0 },
	[smCounterSaRxGetTraceRecord]       = { "SA RX GETTRACE(TraceRecord)", 0, 0, 0 },
	[smCounterSaRxGetMultiPathRecord]   = { "SA RX GETMULTI(MultiPathRecord)", 0, 0, 0 },
	[smCounterSaRxGetVfRecord]			= { "SA RX GET(vFabricRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblVfRecord]		= { "SA RX GETTBL(vFabricRecord)", 0, 0, 0 },
	[smCounterSaRxGetFabricInfoRecord]			= { "SA RX GET(FabricInfoRecord)", 0, 0, 0 },

	[smCounterSaRxGetQuarantinedNodeRecord] = { "SA RX GET(QuarantinedNodeRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblQuarantinedNodeRecord] = { "SA RX GETTBL(QuarantinedNodeRecord)", 0, 0, 0 },
	[smCounterSaRxGetCongInfoRecord] = { "SA RX GET(CongInfoRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetTblCongInfoRecord] = { "SA RX GETTBL(CongInfoRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetSwitchCongRecord] = { "SA RX GET(SwitchCongRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetTblSwitchCongRecord] = { "SA RX GETTBL(SwitchCongRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetSwitchPortCongRecord] = { "SA RX GET(SwitchPortCongRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetTblSwitchPortCongRecord] = { "SA RX GETTBL(SwitchPortCongRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetHFICongRecord] = { "SA RX GET(HFICongRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetTblHFICongRecord] = { "SA RX GET(HFICongRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetHFICongCtrlRecord] = { "SA RX GET(HFICongCtrlRecord)", 0, 0, 0 }, 
	[smCounterSaRxGetTblHFICongCtrlRecord] = { "SA RX GET(HFICongCtrlRecord)", 0, 0, 0 }, 

	[smCounterSaRxGetBfrCtrlRecord]		= { "SA RX GET(BfrCtrlRecord)", 0, 0, 0 },
	[smCounterSaRxGetTblBfrCtrlRecord]	= { "SA RX GETTBL(BfrCtrlRecord)", 0, 0, 0 },
	[smCounterSaRxGetCableInfoRecord] 	= {"SA RX GET(CableInfo)", 0, 0, 0}, 
	[smCounterSaRxGetTblCableInfoRecord] ={"SA RX GETTBL(CableInfo)", 0, 0, 0},
	
    [smCounterSaRxGetPortGroupRecord] = { "SA RX GET(PortGroupRecord)", 0, 0, 0 },
    [smCounterSaRxGetTblPortGroupRecord] = { "SA RX GETTBL(PortGroupRecord)", 0, 0, 0 },
    [smCounterSaRxGetPortGroupFwdRecord] = { "SA RX GET(PortGroupFwdRecord)", 0, 0, 0 },
    [smCounterSaRxGetTblPortGroupFwdRecord] = { "SA RX GETTBL(PortGroupFwdRecord)", 0, 0, 0 },

	[smCounterSaRxReportResponse]       = { "SA RX REPORTRESP(Notice)", 0, 0, 0 },

	// Weird conditions
	[smCounterSaDuplicateRequests]      = { "SA RX DUPLICATE REQUESTS", 0, 0, 0 },
	[smCounterSaDeadRmppPacket]         = { "SA RX DEAD RMPP TRANSACTION PACKET", 0, 0, 0 },
	[smCounterSaDroppedRequests]        = { "SA DROPPED REQUESTS", 0, 0, 0 },
	[smCounterSaContextNotAvailable]    = { "SA NO AVAILABLE CONTEXTS", 0, 0, 0 },

	// GetMulti Request stuff
	[smCounterSaGetMultiNonRmpp]        = { "SA RX GETMULTI() Non-RMPP", 0, 0, 0 },
	[smCounterSaRxGetMultiInboundRmppAbort] = { "SA RX GETMULTI RMPP Abort", 0, 0, 0 },
	[smCounterSaTxGetMultiInboundRmppAbort] = { "SA TX GETMULTI RMPP Abort", 0, 0, 0 },
	[smCounterSaTxGetMultiAckRetries]   = { "SA TX GETMULTI Ack Retries", 0, 0, 0 },

	// SA RMPP Operations
	[smCounterSaRmppTxRetries]          = { "SA TX RMPP Retries", 0, 0, 0 },
	[smCounterRmppStatusStopNoresources]= { "SA TX RMPP STOP", 0, 0, 0 },
	[smCounterRmppStatusAbortInconsistentLastPayloadLength]
	                                    = { "SA TX RMMP ABORT(TIMEOUT)", 0, 0, 0 },
	[smCounterRmppStatusAbortInconsistentFirstSegnum]
	                                    = { "SA TX RMMP ABORT(BAD FIRST SEG)", 0, 0, 0 },
	[smCounterRmppStatusAbortBadType]   = { "SA TX RMMP ABORT(BAD TYPE)", 0, 0, 0 },
	[smCounterRmppStatusAbortNewWindowLastTooSmall]
	                                    = { "SA TX RMMP ABORT(WINDOW TOO SMALL)", 0, 0, 0 },
	[smCounterRmppStatusAbortSegnumTooBig]
	                                    = { "SA TX RMMP ABORT(SEGNUM TOO BIG)", 0, 0, 0 },
	[smCounterRmppStatusAbortUnsupportedVersion]
	                                    = { "SA TX RMMP ABORT(BAD VERSION)", 0, 0, 0 },
	[smCounterRmppStatusAbortTooManyRetries]
	                                    = { "SA TX RMMP ABORT(TOO MANY RETRIES)", 0, 0, 0 },
	[smCounterRmppStatusAbortUnspecified]
	                                    = { "SA TX RMMP ABORT(UNKNOWN)", 0, 0, 0 },

	// SA Request Response Stuff
	[smCountersSaTxRespMadStatusBusy]   = { "SA TX STATUS BUSY", 0, 0, 0 },
	[smCountersSaTxRespMadStatusRedirect] = { "SA TX STATUS REDIRECT", 0, 0, 0 },
	[smCountersSaTxRespMadStatusBadClass] = { "SA TX STATUS BADCLASS", 0, 0, 0 },
	[smCountersSaTxRespMadStatusBadMethod] = { "SA TX STATUS BADMETHOD", 0, 0, 0 },
	[smCountersSaTxRespMadStatusBadAttr]= { "SA TX STATUS BADMETHODATTR", 0, 0, 0 },
	[smCountersSaTxRespMadStatusBadField] = { "SA TX STATUS BADFIELD", 0, 0, 0 },
	[smCountersSaTxRespMadStatusSaNoResources] = { "SA TX STATUS NO RESOURCES", 0, 0, 0 },
	[smCountersSaTxRespMadStatusSaReqInvalid] = { "SA TX STATUS INVALID REQ", 0, 0, 0 },
	[smCountersSaTxRespMadStatusSaNoRecords] = { "SA TX STATUS NO RECORDS", 0, 0, 0 },
	[smCountersSaTxRespMadStatusSaTooManyRecs] = { "SA TX STATUS TOO MANY RECORDS", 0, 0, 0 },
	[smCountersSaTxRespMadStatusSaReqInvalidGid] = { "SA TX STATUS BAD GID", 0, 0, 0 },
	[smCountersSaTxRespMadStatusSaReqInsufficientComponents]
	                                    = { "SA TX STATUS TOO FEW FIELDS", 0, 0, 0 },
	[smCountersSaTxRespMadStatusUnknown]= { "SA TX STATUS UNKNOWN", 0, 0, 0 },

	// DB Sync Stuff
	[smCountersDbSyncSendCmd]           = { "DBSYNC Send Command", 0, 0, 0 },
	[smCountersDbSyncSendCmdFailure]    = { "DBSYNC Send Command Failure", 0, 0, 0 },
	[smCountersDbSyncRcvData]           = { "DBSYNC Receive Data", 0, 0, 0 },
	[smCountersDbSyncRcvDataFailure]    = { "DBSYNC Receive Data Failure", 0, 0, 0 },
	[smCountersDbSyncReply]             = { "DBSYNC Reply", 0, 0, 0 },
	[smCountersDbSyncReplyFailure]      = { "DBSYNC Reply Failure", 0, 0, 0 },

#if 0
	// Job Management (not really used?)
	[smCounterJmRequests]               = { "JM Requests", 0, 0, 0 },
	[smCounterJmErrors]                 = { "JM Errors", 0, 0, 0 },
	[smCounterJmReqClassPortInfo]       = { "JM Msg GetClassPortInfo", 0, 0, 0 },
	[smCounterJmReqCreateJob]           = { "JM Msg CreateJob", 0, 0, 0 },
	[smCounterJmReqSetUseMatrix]        = { "JM Msg SetUseMatrix", 0, 0, 0 },
	[smCounterJmReqPollReady]           = { "JM Msg PollReady", 0, 0, 0 },
	[smCounterJmReqCompleteJob]         = { "JM Msg CompleteJob", 0, 0, 0 },
	[smCounterJmReqGetGuidList]         = { "JM Msg GetGuidList", 0, 0, 0 },
	[smCounterJmReqGetSwitchMap]        = { "JM Msg GetSwitchMap", 0, 0, 0 },
	[smCounterJmReqGetCostMatrix]       = { "JM Msg GetCostMatrix", 0, 0, 0 },
	[smCounterJmReqGetUseMatrix]        = { "JM Msg GetUseMatrix", 0, 0, 0 },
	[smCounterJmReqGetJobs]             = { "JM Msg GetJobs", 0, 0, 0 },
#endif
};

sm_counter_t smPeakCounters[smPeakCountersMax] = {
	[smMaxSaContextsInUse]              = { "SA Maximum Contexts In Use", 0, 0, 0},
	[smMaxSaContextsFree]              = { "SA Maximum Contexts Free", 0, 0, 0},
	[smMaxSweepTime]                   = { "Maximum SM Sweep Time in ms", 0, 0, 0},
};

//
// Initialize SM counters
//
void sm_init_counters(void) {
	int i = 0;
	for (i = 0; i < smCountersMax; ++i) {
		AtomicWrite(&smCounters[i].sinceLastSweep, 0);
		AtomicWrite(&smCounters[i].lastSweep, 0);
		AtomicWrite(&smCounters[i].total, 0);
	}

	if (VSTATUS_OK != vs_stdtime_get(&smCountersClearedTime)) {
		smCountersClearedTime = 0;
	}
}

//
// Assigns values of .sinceLastSweep counters to .lastSweep and
// zero's the .sinceLastSweep counters
//
void sm_process_sweep_counters(void) {
	int i = 0;
	for (i = 0; i < smCountersMax; ++i) {
		// Save value
		AtomicWrite(&smCounters[i].lastSweep,
		            AtomicRead(&smCounters[i].sinceLastSweep));

		// Zero it out
		AtomicWrite(&smCounters[i].sinceLastSweep, 0);
	}

	for (i = 0; i < smPeakCountersMax; ++i) {
		AtomicWrite(&smPeakCounters[i].lastSweep,
		            AtomicRead(&smPeakCounters[i].sinceLastSweep));

		AtomicWrite(&smPeakCounters[i].sinceLastSweep, 0);
	}
}

//
// Resets the 'last sweep' and 'total' counters
//
void sm_reset_counters(void) {
	int i = 0;
	for (i = 0; i < smCountersMax; ++i) {
		// Zero it out
		AtomicWrite(&smCounters[i].lastSweep, 0);
		AtomicWrite(&smCounters[i].total, 0);
	}

	for (i = 0; i < smPeakCountersMax; ++i) {
		// Zero it out
		AtomicWrite(&smPeakCounters[i].lastSweep, 0);
		AtomicWrite(&smPeakCounters[i].total, 0);
	}

	if (VSTATUS_OK != vs_stdtime_get(&smCountersClearedTime)) {
		smCountersClearedTime = 0;
	}
}

#ifdef __VXWORKS__

//
// Like snprintf, but appends the message onto the end of buf
// this version written for ESM
// if no buffer is specified outputs to stdout
//
char * snprintfcat(char * buf, int * len, const char * format, ...)
{
    char * tmp = NULL;
    int n = 0;
    va_list ap;
    char temp[256];

	// if no buffer print out the contents
	if (!buf) {
		va_start(ap, format);
		vprintf(format, ap);
		va_end(ap);
		return NULL;
	}

    va_start(ap, format);
    vsprintf(temp, format, ap);
    va_end(ap);

    if ((n = strlen(temp)) >= (*len - 4)) {
        *len *= 2;
        if (vs_pool_alloc(&sm_pool, *len, (void*)&tmp) != VSTATUS_OK) {
            IB_FATAL_ERROR("snprintfcat: CAN'T ALLOCATE SPACE.");
            vs_pool_free(&sm_pool, buf);
            return NULL;
        }

        cs_strlcpy(tmp, buf, *len);
        vs_pool_free(&sm_pool, buf);
        buf = tmp;
    }
    strcat(buf, temp);
    return buf;
}


#else

//
// Like snprintf, but appends the message onto the end of buf
//
char * snprintfcat(char * buf, int * len, const char * format, ...)
{
	char * tmp = NULL, * writeLocation = NULL;
	int n = 0, bytesNeeded;
	va_list ap;

	// if no buffer print out the contents
	if (!buf) {
		va_start(ap, format);
		vprintf(format, ap);
		va_end(ap);
		return NULL;
	}

	if ((n = strlen(buf)) >= *len) {
		*len *= 2;
		if (vs_pool_alloc(&sm_pool, *len, (void*)&tmp) != VSTATUS_OK) {
			IB_FATAL_ERROR("sm_print_counters_to_buf: CAN'T ALLOCATE SPACE.");
			vs_pool_free(&sm_pool, buf);
			return NULL;
		}

		cs_strlcpy(tmp, buf, *len);
		vs_pool_free(&sm_pool, buf);
		buf = tmp;
	}

	writeLocation = buf + n;

	while (1) {
		va_start(ap, format);
		bytesNeeded = vsnprintf(writeLocation, *len - n, format, ap);
		va_end(ap);

		if (bytesNeeded > -1 && bytesNeeded < (*len - n))
			return buf;

		*len *= 2;
		if (vs_pool_alloc(&sm_pool, *len, (void*)&tmp) != VSTATUS_OK) {
			IB_FATAL_ERROR("sm_print_counters_to_buf: CAN'T ALLOCATE SPACE.");
			vs_pool_free(&sm_pool, buf);
			return NULL;
		}

		cs_strlcpy(tmp, buf, *len);
		vs_pool_free(&sm_pool, buf);
		buf = tmp;
		writeLocation = buf + n;
	}

	return NULL;
}
#endif

#ifndef __VXWORKS__

//
// prints counters to a dynamically allocate buffer - user is
// responsible for freeing it using vs_pool_free()
//
char * sm_print_counters_to_buf(void) {
	char * buf = NULL;
	int i =0;
	int len = 256;
	time_t currentTime;
	time_t elapsedTime;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR("sm_print_counters_to_buf: CAN'T ALLOCATE SPACE.");
		return NULL;
	}

	// get current time stamp
	if (VSTATUS_OK == vs_stdtime_get(&currentTime)) {
		elapsedTime = currentTime - smCountersClearedTime;
		snprintf(buf, len, "%35s: %u\n", "Seconds since last cleared", (unsigned int)elapsedTime);
	}

	// show cleared time stamp
	buf = snprintfcat(buf, &len, "%35s: %s", "Time last cleared", ctime(&smCountersClearedTime));

	buf = snprintfcat(buf, &len, "%35s: %10s %10s %10s\n",
		"COUNTER", "THIS SWEEP", "LAST SWEEP", "TOTAL");
	buf = snprintfcat(buf, &len, "------------------------------------ "
	                             "---------- "
	                             "---------- ----------\n");
	for (i = 0; i < smCountersMax; ++i) {
		buf = snprintfcat(buf, &len, "%35s: %10u %10u %10u\n",
		                  smCounters[i].name,
		                  AtomicRead(&smCounters[i].sinceLastSweep),
		                  AtomicRead(&smCounters[i].lastSweep),
		                  AtomicRead(&smCounters[i].total));
	}

	for (i = 0; i < smPeakCountersMax; ++i) {
		buf = snprintfcat(buf, &len, "%35s: %10u %10u %10u\n",
		                  smPeakCounters[i].name,
		                  AtomicRead(&smPeakCounters[i].sinceLastSweep),
		                  AtomicRead(&smPeakCounters[i].lastSweep),
		                  AtomicRead(&smPeakCounters[i].total));
	}

	return buf;
}
#endif

#ifdef __VXWORKS__
void sm_print_counters_to_stream(FILE * out) {
	int i = 0;
	time_t currentTime;
	time_t elapsedTime;
	char tbuf[30];

	if (topology_passcount < 1) {
		sysPrintf("\nSM is currently in the %s state, count = %d\n\n", sm_getStateText(sm_state), (int)sm_smInfo.ActCount);
		return;
	}

	// get current time stamp
	if (VSTATUS_OK == vs_stdtime_get(&currentTime)) {
		elapsedTime = currentTime - smCountersClearedTime;
		fprintf(out, "\n");
		fprintf(out, "%35s: %u\n", "Seconds since last cleared", (unsigned int)elapsedTime);
	}

	// show cleared time stamp
	UiUtil_GetLocalTimeString(smCountersClearedTime, tbuf, sizeof(tbuf));
	fprintf(out, "%35s: %s\n", "Time last cleared", tbuf);
	fprintf(out, "\n");

	fprintf(out, "%35s: %10s %10s %10s\n",
	        "COUNTER", "THIS SWEEP", "LAST SWEEP", "TOTAL");
	fprintf(out, "------------------------------------ "
	             "---------- "
	             "---------- ----------\n");
	for (i = 0; i < smCountersMax; ++i) {
		fprintf(out, "%35s: %10u %10u %10u\n",
		        smCounters[i].name,
		        AtomicRead(&smCounters[i].sinceLastSweep),
		        AtomicRead(&smCounters[i].lastSweep),
		        AtomicRead(&smCounters[i].total));
	}

	for (i = 0; i < smPeakCountersMax; ++i) {
		fprintf(out, "%35s: %10u %10u %10u\n",
		        smPeakCounters[i].name,
		        AtomicRead(&smPeakCounters[i].sinceLastSweep),
		        AtomicRead(&smPeakCounters[i].lastSweep),
		        AtomicRead(&smPeakCounters[i].total));
	}
}
#endif

