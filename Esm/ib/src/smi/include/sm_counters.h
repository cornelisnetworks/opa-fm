/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//
// FILE NAME
//    sm_counters.h
//
// DESCRIPTION
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    None
//
// DEPENDENCIES
//    ispinlock.h - for ATOMIC_UINT operations
//
//
//===========================================================================//


#ifndef _SM_COUNTERS_H_
#define _SM_COUNTERS_H_ 1

#include "ispinlock.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "mai_g.h"

//
// SM Counter Definitions
//
typedef enum _smCounters {
	// State transitions
	smCounterSmStateToDiscovering,
	smCounterSmStateToMaster,
	smCounterSmStateToStandby,
	smCounterSmStateToInactive,

	// Basic Packet Info
	smCounterSmPacketTransmits, 
	smCounterDirectRoutePackets, 
	smCounterLidRoutedPackets,
	smCounterPacketRetransmits,
	smCounterSmTxRetriesExhausted,

	// SMA Method/Attributes
	smCounterGetNotice, // Get(Notice)
	smCounterSetNotice, // Set(Notice)

	// Traps Received
	smCounterTrapsReceived, 	
	smCounterTrapPortUp,		// Trap 64
	smCounterTrapPortDown,		// Trap 65
	smCounterTrapMcGrpCreate,	// Trap 66
	smCounterTrapMcGrpDel,		// Trap 67
	smCounterTrapUnPath,		// Trap 68
	smCounterTrapRePath,		// Trap 69
	smCounterTrapPortStateChg,  // Trap 128
	smCounterTrapLinkIntegrity, // Trap 129
	smCounterTrapBufOverrun,	// Trap 130
	smCounterTrapFlowControl,	// Trap 131
	smCounterTrapLocalChg,		// Trap 144
	smCounterTrapSysImgChg,		// Trap 145
	smCounterTrapBadMKey,		// Trap 256
	smCounterTrapBadPKey,		// Trap 257
	smCounterTrapBadQKey,		// Trap 258
	smCounterTrapBadPKeySwPort,	// Trap 259
	smCounterTrapLinkWidthDowngrade, // Trap 2048
	smCounterTrapCostMatrixChange, // Trap 2049

	smCounterTrapsRepressed, 
	smCounterGetNodeDescription, // No set
	smCounterGetNodeInfo, // No set
	smCounterGetSwitchInfo,
	smCounterSetSwitchInfo,
	smCounterGetPortInfo,
	smCounterSetPortInfo,
	smCounterGetPortStateInfo,
	smCounterSetPortStateInfo,
	smCounterGetPKeyTable,
	smCounterSetPKeyTable,
	smCounterGetSL2SCMappingTable,
	smCounterSetSL2SCMappingTable,
	smCounterGetSC2SLMappingTable,
	smCounterSetSC2SLMappingTable,
	smCounterGetSC2VLtMappingTable,
	smCounterSetSC2VLtMappingTable,
	smCounterGetSC2VLntMappingTable,
	smCounterSetSC2VLntMappingTable,
	smCounterGetSC2VLrMappingTable,
	smCounterSetSC2VLrMappingTable,
	smCounterGetSC2SCMappingTable,
	smCounterSetSC2SCMappingTable,
	smCounterSetSC2SCMultiSet,
	smCounterGetVLArbitrationTable,
	smCounterSetVLArbitrationTable,
	smCounterGetLft,
	smCounterSetLft,
	smCounterGetMft,
	smCounterSetMft,
	smCounterGetAggregate,
	smCounterSetAggregate,
	smCounterGetLedInfo,
	smCounterSetLedInfo,
	smCounterGetCableInfo, // No Set
	smCounterRxGetSmInfo, // SmInfo is an oddball
	smCounterGetSmInfo,
	smCounterRxSetSmInfo,
	smCounterSetSmInfo,
	smCounterTxGetRespSmInfo,
	smCounterGetPG,
	smCounterSetPG,
	smCounterGetPGft,
	smCounterSetPGft,
	smCounterGetBufferControlTable,
	smCounterSetBufferControlTable,
	smCounterGetARLidMask,
	smCounterSetARLidMask,

	// CCA SMA Counters
	smCounterGetCongestionInfo,
	smCounterGetHfiCongestionSetting,
	smCounterSetHfiCongestionSetting,
	smCounterGetHfiCongestionControl,
	smCounterSetHfiCongestionControl,
	smCounterGetSwitchCongestionSetting,
	smCounterSetSwitchCongestionSetting,
	smCounterGetSwitchPortCongestionSetting,

	smCounterRxGetResp,

	// SMA Response Status codes
	smCounterRxSmaMadStatusBusy,
	smCounterRxSmaMadStatusRedirect,
	smCounterRxSmaMadStatusBadClass,
	smCounterRxSmaMadStatusBadMethod,
	smCounterRxSmaMadStatusBadAttr,
	smCounterRxSmaMadStatusBadField,
	smCounterRxSmaMadStatusUnknown,

	// SA Queries
	smCounterSaRxGetClassPortInfo,
	smCounterSaTxReportNotice,
	smCounterSaRxSetInformInfo,
	smCounterSaRxGetNodeRecord,
	smCounterSaRxGetTblNodeRecord,
	smCounterSaRxGetPortInfoRecord,
	smCounterSaRxGetTblPortInfoRecord,

	smCounterSaRxGetSc2ScMappingRecord,
	smCounterSaRxGetTblSc2ScMappingRecord,
	smCounterSaRxGetSl2ScMappingRecord,
	smCounterSaRxGetTblSl2ScMappingRecord,
	smCounterSaRxGetSc2SlMappingRecord,
	smCounterSaRxGetTblSc2SlMappingRecord,
	smCounterSaRxGetSc2VltMappingRecord,
	smCounterSaRxGetTblSc2VltMappingRecord,
	smCounterSaRxGetSc2VlntMappingRecord,
	smCounterSaRxGetTblSc2VlntMappingRecord,
	smCounterSaRxGetSc2VlrMappingRecord,
	smCounterSaRxGetTblSc2VlrMappingRecord,

	smCounterSaRxGetSwitchInfoRecord,
	smCounterSaRxGetTblSwitchInfoRecord,
	smCounterSaRxGetLftRecord,
	smCounterSaRxGetTblLftRecord,
	smCounterSaRxGetMftRecord,
	smCounterSaRxGetTblMftRecord,

	smCounterSaRxGetVlArbTableRecord,
	smCounterSaRxGetTblVlArbTableRecord,
	smCounterSaRxGetSmInfoRecord,
	smCounterSaRxGetTblSmInfoRecord,
	smCounterSaRxGetInformInfoRecord,
	smCounterSaRxGetTblInformInfoRecord,

	smCounterSaRxGetLinkRecord,
	smCounterSaRxGetTblLinkRecord,
	smCounterSaRxGetServiceRecord,
	smCounterSaRxSetServiceRecord,
	smCounterSaRxGetTblServiceRecord,
	smCounterSaRxDeleteServiceRecord,

	smCounterSaRxGetPKeyTableRecord,
	smCounterSaRxGetTblPKeyTableRecord,
	smCounterSaRxGetPathRecord,
	smCounterSaRxGetTblPathRecord,
	smCounterSaRxGetMcMemberRecord,
	smCounterSaRxSetMcMemberRecord,
	smCounterSaRxGetTblMcMemberRecord,
	smCounterSaRxDeleteMcMemberRecord,
	smCounterSaRxGetTraceRecord,
	smCounterSaRxGetMultiPathRecord,
	smCounterSaRxGetVfRecord,
	smCounterSaRxGetTblVfRecord,
	smCounterSaRxGetFabricInfoRecord,

	smCounterSaRxGetQuarantinedNodeRecord,
	smCounterSaRxGetTblQuarantinedNodeRecord,
    smCounterSaRxGetCongInfoRecord,
    smCounterSaRxGetTblCongInfoRecord,
    smCounterSaRxGetSwitchCongRecord,
    smCounterSaRxGetTblSwitchCongRecord,
    smCounterSaRxGetSwitchPortCongRecord,
    smCounterSaRxGetTblSwitchPortCongRecord,
    smCounterSaRxGetHFICongRecord,
    smCounterSaRxGetTblHFICongRecord,
    smCounterSaRxGetHFICongCtrlRecord,
    smCounterSaRxGetTblHFICongCtrlRecord,

    smCounterSaRxGetBfrCtrlRecord,
    smCounterSaRxGetTblBfrCtrlRecord,
	smCounterSaRxGetCableInfoRecord,
	smCounterSaRxGetTblCableInfoRecord,

	smCounterSaRxGetPortGroupRecord,
	smCounterSaRxGetTblPortGroupRecord,
	smCounterSaRxGetPortGroupFwdRecord,
	smCounterSaRxGetTblPortGroupFwdRecord,

	smCounterSaRxGetSwitchCostRecord,
	smCounterSaRxGetTblSwitchCostRecord,

	smCounterSaRxReportResponse,

	smCounterSaRxGetDgMemberRecord,
	smCounterSaRxGetTblDgMemberRecord,
	smCounterSaRxGetDgNameRecord,
	smCounterSaRxGetTblDgNameRecord,

	smCounterSaRxGetDtMemberRecord,
	smCounterSaRxGetTblDtMemberRecord,

	// Weird conditions
	smCounterSaDuplicateRequests,
	smCounterSaDeadRmppPacket,
	smCounterSaDroppedRequests,
	smCounterSaContextNotAvailable,

	// GetMulti Request stuff
	smCounterSaGetMultiNonRmpp,
	smCounterSaRxGetMultiInboundRmppAbort,
	smCounterSaTxGetMultiInboundRmppAbort,
	smCounterSaTxGetMultiAckRetries,

	// SA RMPP Operations
	smCounterSaRmppTxRetries,
	smCounterRmppStatusStopNoresources,
	smCounterRmppStatusAbortInconsistentLastPayloadLength,

	smCounterRmppStatusAbortInconsistentFirstSegnum,

	smCounterRmppStatusAbortBadType,
	smCounterRmppStatusAbortNewWindowLastTooSmall,

	smCounterRmppStatusAbortSegnumTooBig,

	smCounterRmppStatusAbortUnsupportedVersion,

	smCounterRmppStatusAbortTooManyRetries,

	smCounterRmppStatusAbortUnspecified,

	// SA Request Response Stuff
	smCountersSaTxRespMadStatusBusy,
	smCountersSaTxRespMadStatusRedirect,
	smCountersSaTxRespMadStatusBadClass,
	smCountersSaTxRespMadStatusBadMethod,
	smCountersSaTxRespMadStatusBadAttr,
	smCountersSaTxRespMadStatusBadField,
	smCountersSaTxRespMadStatusSaNoResources,
	smCountersSaTxRespMadStatusSaReqInvalid,
	smCountersSaTxRespMadStatusSaNoRecords,
	smCountersSaTxRespMadStatusSaTooManyRecs,
	smCountersSaTxRespMadStatusSaReqInvalidGid,
	smCountersSaTxRespMadStatusSaReqInsufficientComponents,

	smCountersSaTxRespMadStatusUnknown,

	// DB Sync Stuff
	smCountersDbSyncSendCmd,
	smCountersDbSyncSendCmdFailure,
	smCountersDbSyncRcvData,
	smCountersDbSyncRcvDataFailure,
	smCountersDbSyncReply,
	smCountersDbSyncReplyFailure,

#if 0
	// Job Management
	smCounterJmRequests,
	smCounterJmErrors,
	smCounterJmReqClassPortInfo,
	smCounterJmReqCreateJob,
	smCounterJmReqSetUseMatrix,
	smCounterJmReqPollReady,
	smCounterJmReqCompleteJob,
	smCounterJmReqGetGuidList,
	smCounterJmReqGetSwitchMap,
	smCounterJmReqGetCostMatrix,
	smCounterJmReqGetUseMatrix,
	smCounterJmReqGetJobs,
#endif

	smCountersMax  // Last value
} sm_counters_t;

typedef enum _sm_peak_counters {
	smMaxSaContextsInUse,
	smMaxSaContextsFree,
	smMaxSweepTime,			// in milliseconds

	smPeakCountersMax // Last value
} sm_peak_counters_t;

typedef struct _sm_counter {
	char * name;
	ATOMIC_UINT sinceLastSweep;
	ATOMIC_UINT lastSweep;
	ATOMIC_UINT total;
} sm_counter_t;

//
// These are the arrays that contain the counter values... They're definition
// is in sm_counters.c. If you add a counter to either list, be sure to
// add a description for it in sm_counters.c
//
// The smCounters array is used for strictly incrementing counters (i.e.
// one's that you'll use the INCREMENT_COUNTERS() function for. The
// smPeakCounters array is for counters that you'll want to use the
// SET_PEAK_COUNTER() function on.
//
extern sm_counter_t smCounters[smCountersMax];
extern sm_counter_t smPeakCounters[smPeakCountersMax];

//
// This function increments a counter in the smCounters array
//
static __inline__
void INCREMENT_COUNTER(sm_counters_t counter) {
	AtomicIncrementVoid(&smCounters[counter].sinceLastSweep);
	AtomicIncrementVoid(&smCounters[counter].total);
}

//
// This function compares and sets a value in the smPeakCounters array if
// it's greater that the current value.
//
static __inline__
void SET_PEAK_COUNTER(sm_peak_counters_t counter, const uint32 value) {
	uint32 tmp;

	do {
		tmp = AtomicRead(&smPeakCounters[counter].sinceLastSweep);
	} while (  (value > tmp)
	        && ! AtomicCompareStore(&smPeakCounters[counter].sinceLastSweep,
	                                tmp, value));

	do {
		tmp = AtomicRead(&smPeakCounters[counter].total);
	} while (  (value > tmp)
	        && ! AtomicCompareStore(&smPeakCounters[counter].total,
	                                tmp, value));
}

//
// Convenience function for processing a mad status return code.
//
static __inline__
void INCREMENT_MAD_STATUS_COUNTERS(Mai_t * mad) {
	if (mad->base.mclass == MAD_CV_SUBN_ADM) {
		switch (mad->base.status) {
		 case MAD_STATUS_OK:
			break;
		 case MAD_STATUS_BUSY:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusBusy);
			break;
		 case MAD_STATUS_REDIRECT:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusRedirect);
			break;
		 case MAD_STATUS_BAD_CLASS:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusBadClass);
			break;
		 case MAD_STATUS_BAD_METHOD:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusBadMethod);
			break;
		 case MAD_STATUS_BAD_ATTR:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusBadAttr);
			break;
		 case MAD_STATUS_BAD_FIELD:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusBadField);
			break;
		 case MAD_STATUS_SA_NO_RESOURCES:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusSaNoResources);
			break;
		 case MAD_STATUS_SA_REQ_INVALID:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusSaReqInvalid);
			break;
		 case MAD_STATUS_SA_NO_RECORDS:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusSaNoRecords);
			break;
		 case MAD_STATUS_SA_TOO_MANY_RECS:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusSaTooManyRecs);
			break;
		 case MAD_STATUS_SA_REQ_INVALID_GID:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusSaReqInvalidGid);
			break;
		 case MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusSaReqInsufficientComponents);
			break;
		 default:
			INCREMENT_COUNTER(smCountersSaTxRespMadStatusUnknown);
			break;
		}
	} else {  // SMA Status
		switch (mad->base.status & ~MAD_STATUS_D_BIT) {
		 case MAD_STATUS_OK:
			break;
		 case MAD_STATUS_BUSY:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusBusy);
			break;
		 case MAD_STATUS_REDIRECT:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusRedirect);
			break;
		 case MAD_STATUS_BAD_CLASS:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusBadClass);
			break;
		 case MAD_STATUS_BAD_METHOD:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusBadMethod);
			break;
		 case MAD_STATUS_BAD_ATTR:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusBadAttr);
			break;
		 case MAD_STATUS_BAD_FIELD:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusBadField);
			break;
		 default:
			INCREMENT_COUNTER(smCounterRxSmaMadStatusUnknown);
			break;
		}
	}
}

// Function prototypes
extern void sm_init_counters(void);
extern void sm_process_sweep_counters(void);
extern void sm_reset_counters(void);
extern void sm_print_counters_to_stream(FILE * out);

#ifndef __VXWORKS__
extern char * sm_print_counters_to_buf(void);
#endif

#endif	// _SM_COUNTERS_H_
