/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//
// FILE NAME
//    pm_counters.h
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


#ifndef _PM_COUNTERS_H_
#define _PM_COUNTERS_H_ 1

#include "ispinlock.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "mai_g.h"
#include "iba/stl_pa_priv.h"

//
// PM Counter Definitions
//
typedef enum _pmCounters {
	pmCounterPmSweeps,
	pmCounterPmNoRespPorts,
	pmCounterPmNoRespNodes,
	pmCounterPmUnexpectedClearPorts,

	// Total PM Class Packet Transmits
	pmCounterPmPacketTransmits,

	// PMA Retries/Failures
	pmCounterPacketRetransmits,
	pmCounterPmTxRetriesExhausted,

	// PMA Method/Attributes
	pmCounterGetClassPortInfo, // Get(ClassPortInfo)
	pmCounterGetPortStatus, // Get(PortStatus)
	pmCounterSetClearPortStatus, // Set(ClearPortStatus)
	pmCounterGetDataPortCounters, // Get(DataPortCounters)
	pmCounterGetErrorPortCounters, // Get(ErrorPortCounters)
	pmCounterGetErrorInfo, // Get(ErrorInfo)
	pmCounterSetErrorInfo, // Set(ErrorInfo)

	pmCounterRxGetResp,

	// PMA Response Status codes
	pmCounterRxPmaMadStatusBusy,
	pmCounterRxPmaMadStatusRedirect,
	pmCounterRxPmaMadStatusBadClass,
	pmCounterRxPmaMadStatusBadMethod,
	pmCounterRxPmaMadStatusBadAttr,
	pmCounterRxPmaMadStatusBadField,
	pmCounterRxPmaMadStatusUnknown,

	// PA Queries
	pmCounterPaRxGetClassPortInfo,
	pmCounterPaRxGetGrpList,
	pmCounterPaRxGetGrpInfo,
	pmCounterPaRxGetGrpCfg,
	pmCounterPaRxGetPortCtrs,
	pmCounterPaRxClrPortCtrs,
	pmCounterPaRxClrAllPortCtrs,
	pmCounterPaRxGetPmConfig,
	pmCounterPaRxFreezeImage,
	pmCounterPaRxReleaseImage,
	pmCounterPaRxRenewImage,
	pmCounterPaRxGetFocusPorts,
	pmCounterPaRxGetImageInfo,
	pmCounterPaRxMoveFreezeFrame,
	pmCounterPaRxGetVFList,
	pmCounterPaRxGetVFInfo,
	pmCounterPaRxGetVFCfg,
	pmCounterPaRxGetVFPortCtrs,
	pmCounterPaRxClrVFPortCtrs,
	pmCounterPaRxGetVFFocusPorts,
	pmCounterPaRxGetFocusPortsMultiSelect,
	pmCounterPaRxGetGrpNodeInfo,
	pmCounterPaRxGetGrpLinkInfo,

	// Weird conditions
	pmCounterPaDuplicateRequests,
	pmCounterPaDeadRmppPacket,
	pmCounterPaDroppedRequests,
	pmCounterPaContextNotAvailable,

	// PA RMPP Operations
	pmCounterPaRmppTxRetries,
	pmCounterRmppStatusStopNoresources,
	//pmCounterRmppStatusAbortTimeTooLong, // Not used anywhere
	pmCounterRmppStatusAbortInconsistentLastPayloadLength,
	pmCounterRmppStatusAbortInconsistentFirstSegnum,
	pmCounterRmppStatusAbortBadType,
	pmCounterRmppStatusAbortNewWindowLastTooSmall,
	pmCounterRmppStatusAbortSegnumTooBig,
	//pmCounterRmppStatusAbortIllegalStatus, // Not used anywhere
	pmCounterRmppStatusAbortUnsupportedVersion,
	pmCounterRmppStatusAbortTooManyRetries,
	pmCounterRmppStatusAbortUnspecified,

	// PA Request Response Stuff
	pmCountersPaTxRespMadStatusBusy,
	pmCountersPaTxRespMadStatusRedirect,
	pmCountersPaTxRespMadStatusBadClass,
	pmCountersPaTxRespMadStatusBadMethod,
	pmCountersPaTxRespMadStatusBadAttr,
	pmCountersPaTxRespMadStatusBadField,
	pmCountersPaTxRespMadStatusPaUnavailable,
	pmCountersPaTxRespMadStatusPaNoGroup,
	pmCountersPaTxRespMadStatusPaNoPort,
    pmCountersPaTxRespMadStatusStlPaNoVf,
    pmCountersPaTxRespMadStatusStlPaInvalidParam,
    pmCountersPaTxRespMadStatusStlPaNoImage,
	pmCountersPaTxRespMadStatusStlPaNoData,
	pmCountersPaTxRespMadStatusStlPaBadData,

	pmCountersPaTxRespMadStatusUnknown,

	pmCountersMax  // Last value
} pm_counters_t;

typedef enum _pm_peak_counters {
	pmMaxPaContextsInUse,
	pmMaxPaContextsFree,
	pmMaxSweepTime,					// in ms

	pmPeakCountersMax // Last value
} pm_peak_counters_t;

typedef struct _pm_counter {
	char * name;
	ATOMIC_UINT sinceLastSweep;
	ATOMIC_UINT lastSweep;
	ATOMIC_UINT total;
} pm_counter_t;

//
// These are the arrays that contain the counter values... They're definition
// is in pm_counters.c. If you add a counter to either list, be sure to
// add a description for it in pm_counters.c
//
// The pmCounters array is used for strictly incrementing counters (i.e.
// one's that you'll use the INCREMENT_PM_COUNTERS() function for. The
// pmPeakCounters array is for counters that you'll want to use the
// SET_PEAK_COUNTER() function on.
//
extern pm_counter_t pmCounters[pmCountersMax];
extern pm_counter_t pmPeakCounters[pmPeakCountersMax];

//
// This function increments a counter in the pmCounters array
//
static __inline__
void INCREMENT_PM_COUNTER(pm_counters_t counter) {
	AtomicIncrementVoid(&pmCounters[counter].sinceLastSweep);
	AtomicIncrementVoid(&pmCounters[counter].total);
}

//
// This function compares and sets a value in the pmPeakCounters array if
// it's greater that the current value.
//
static __inline__
void SET_PM_PEAK_COUNTER(pm_peak_counters_t counter, const uint32 value) {
	uint32 tmp;

	do {
		tmp = AtomicRead(&pmPeakCounters[counter].sinceLastSweep);
	} while (  (value > tmp)
	        && ! AtomicCompareStore(&pmPeakCounters[counter].sinceLastSweep,
	                                tmp, value));

	do {
		tmp = AtomicRead(&pmPeakCounters[counter].total);
	} while (  (value > tmp)
	        && ! AtomicCompareStore(&pmPeakCounters[counter].total,
	                                tmp, value));
}

//
// Convenience function for processing a mad status return code.
//
static __inline__
void INCREMENT_PM_MAD_STATUS_COUNTERS(Mai_t * mad) {
	if (mad->base.mclass == MAD_CV_VFI_PM) {
		switch (mad->base.status) {
		case MAD_STATUS_OK:
			break;
		case MAD_STATUS_BUSY:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusBusy); break;
		case MAD_STATUS_REDIRECT:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusRedirect); break;
		case MAD_STATUS_BAD_CLASS:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusBadClass); break;
		case MAD_STATUS_BAD_METHOD:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusBadMethod); break;
		case MAD_STATUS_BAD_ATTR:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusBadAttr); break;
		case MAD_STATUS_BAD_FIELD:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusBadField); break;
		case STL_MAD_STATUS_STL_PA_UNAVAILABLE:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusPaUnavailable); break;
		case STL_MAD_STATUS_STL_PA_NO_GROUP:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusPaNoGroup); break;
		case STL_MAD_STATUS_STL_PA_NO_PORT:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusPaNoPort); break;
		case STL_MAD_STATUS_STL_PA_NO_VF:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusStlPaNoVf); break;
		case STL_MAD_STATUS_STL_PA_INVALID_PARAMETER:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusStlPaInvalidParam); break;
		case STL_MAD_STATUS_STL_PA_NO_IMAGE:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusStlPaNoImage); break;
		case STL_MAD_STATUS_STL_PA_NO_DATA:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusStlPaNoData); break;
		case STL_MAD_STATUS_STL_PA_BAD_DATA:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusStlPaBadData); break;
		default:
			INCREMENT_PM_COUNTER(pmCountersPaTxRespMadStatusUnknown); break;
		}
	} else {  // PMA Status
		switch (mad->base.status & ~MAD_STATUS_D_BIT) {
		case MAD_STATUS_OK:
			break;
		case MAD_STATUS_BUSY:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusBusy); break;
		case MAD_STATUS_REDIRECT:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusRedirect); break;
		case MAD_STATUS_BAD_CLASS:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusBadClass); break;
		case MAD_STATUS_BAD_METHOD:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusBadMethod); break;
		case MAD_STATUS_BAD_ATTR:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusBadAttr); break;
		case MAD_STATUS_BAD_FIELD:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusBadField); break;
		default:
			INCREMENT_PM_COUNTER(pmCounterRxPmaMadStatusUnknown); break;
		}
	}
}

// Function prototypes
extern void pm_init_counters(void);
extern void pm_process_sweep_counters(void);
extern void pm_reset_counters(void);
extern void pm_print_counters_to_stream(FILE * out);

#ifndef __VXWORKS__
extern char * pm_print_counters_to_buf(void);
#endif

#endif	// _PM_COUNTERS_H_
