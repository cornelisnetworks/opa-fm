/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */
//===========================================================================//
//
// FILE NAME
//    pm_counters.c
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
#include "pm_l.h"
#include "pm_counters.h"
#ifdef __VXWORKS__
#include "UiUtil.h"
#endif 

// time stamp for PM counters
time_t pmCountersClearedTime = 0;

//
// Initializers for PM counters.
// If you added a counter, to the pm_counters_t enumeration, you should
// add a description here.
//
pm_counter_t pmCounters[pmCountersMax] = {
	[pmCounterPmSweeps]		        = { "PM Sweeps", 0, 0, 0 },
	[pmCounterPmNoRespPorts]		= { "Ports with No Response to PM query", 0, 0, 0 },
	[pmCounterPmNoRespNodes]		= { "Nodes with 1 or more NoResp Ports", 0, 0, 0 },
	[pmCounterPmUnexpectedClearPorts]	= { "Ports unexpectedly cleared", 0, 0, 0 },

	// Total PM Class Packet Transmits
	[pmCounterPmPacketTransmits]        = { "Total transmitted PMA Packets", 0, 0, 0 },

	// PMA Retries/Failures
	[pmCounterPacketRetransmits]        = { "PMA Query Retransmits", 0, 0, 0 },
	[pmCounterPmTxRetriesExhausted]     = { "PMA Query Retransmits Exhausted", 0, 0, 0 },

	// PMA Method/Attributes
	[pmCounterGetClassPortInfo]			= { "PM TX GET(ClassPortInfo)", 0, 0, 0 },
	[pmCounterGetPortStatus]			= { "PM TX GET(PortStatus)", 0, 0, 0 },
	[pmCounterSetClearPortStatus]		= { "PM TX SET(ClearPortStatus)", 0, 0, 0 },
	[pmCounterGetDataPortCounters]		= { "PM TX GET(DataPortCounters)", 0, 0, 0 },
	[pmCounterGetErrorPortCounters]		= { "PM TX GET(ErrorPortCounters)", 0, 0, 0 },
	[pmCounterGetErrorInfo]				= { "PM TX GET(ErrorInfo)", 0, 0, 0 },
	[pmCounterSetErrorInfo]				= { "PM TX SET(ErrorInfo)", 0, 0, 0 },

	[pmCounterRxGetResp]			= { "PM RX GETRESP(*)", 0, 0, 0 },

	// PMA Response Status codes
	[pmCounterRxPmaMadStatusBusy]       = { "PM RX STATUS BUSY", 0, 0, 0 },
	[pmCounterRxPmaMadStatusRedirect]   = { "PM RX STATUS REDIRECT", 0, 0, 0 },
	[pmCounterRxPmaMadStatusBadClass]   = { "PM RX STATUS BADCLASS", 0, 0, 0 },
	[pmCounterRxPmaMadStatusBadMethod]  = { "PM RX STATUS BADMETHOD", 0, 0, 0 },
	[pmCounterRxPmaMadStatusBadAttr]    = { "PM RX STATUS BADMETHODATTR", 0, 0, 0 },
	[pmCounterRxPmaMadStatusBadField]   = { "PM RX STATUS BADFIELD", 0, 0, 0 },
	[pmCounterRxPmaMadStatusUnknown]    = { "PM RX STATUS UNKNOWN", 0, 0, 0 },

	// PA Queries
	[pmCounterPaRxGetClassPortInfo]     = { "PA RX GET(ClassPortInfo)", 0, 0, 0 },
	[pmCounterPaRxGetGrpList]           = { "PA RX GET(GrpList)", 0, 0, 0 },
	[pmCounterPaRxGetGrpInfo]           = { "PA RX GET(GrpInfo)", 0, 0, 0 },
	[pmCounterPaRxGetGrpCfg]            = { "PA RX GET(GrpCfg)", 0, 0, 0 },
	[pmCounterPaRxGetPortCtrs]          = { "PA RX GET(PortCtrs)", 0, 0, 0 },
	[pmCounterPaRxClrPortCtrs]          = { "PA RX GET(ClrPortCtrs)", 0, 0, 0 },
	[pmCounterPaRxClrAllPortCtrs]       = { "PA RX GET(ClrAllPortCtrs)", 0, 0, 0 },
	[pmCounterPaRxGetPmConfig]          = { "PA RX GET(PmConfig)", 0, 0, 0 },
	[pmCounterPaRxFreezeImage]          = { "PA RX GET(FreezeImage)", 0, 0, 0 },
	[pmCounterPaRxReleaseImage]         = { "PA RX GET(ReleaseImage)", 0, 0, 0 },
	[pmCounterPaRxRenewImage]           = { "PA RX GET(RenewImage)", 0, 0, 0 },
	[pmCounterPaRxGetFocusPorts]        = { "PA RX GET(FocusPorts)", 0, 0, 0 },
	[pmCounterPaRxGetImageInfo]         = { "PA RX GET(ImageInfo)", 0, 0, 0 },
	[pmCounterPaRxMoveFreezeFrame]      = { "PA RX GET(MoveFreezeFrame)", 0, 0, 0 },
	[pmCounterPaRxGetVFList]            = { "PA RX GET(VFList)", 0, 0, 0 },
	[pmCounterPaRxGetVFInfo]            = { "PA RX GET(VFInfo)", 0, 0, 0 },
	[pmCounterPaRxGetVFCfg]             = { "PA RX GET(VFCfg)", 0, 0, 0 },
	[pmCounterPaRxGetVFPortCtrs]        = { "PA RX GET(VFPortCtrs)", 0, 0, 0 },
	[pmCounterPaRxClrVFPortCtrs]        = { "PA RX GET(ClrVFPortCtrs)", 0, 0, 0 },
	[pmCounterPaRxGetVFFocusPorts]      = { "PA RX GET(VFFocusPorts)", 0, 0, 0 },
	[pmCounterPaRxGetFocusPortsMultiSelect]
	                                   = { "PA RX GET(FocusPortsMultiSelect)", 0, 0, 0 },

	[pmCounterPaRxGetGrpNodeInfo]       = { "PA RX GET(GrpNodeInfo)", 0, 0, 0 },
	[pmCounterPaRxGetGrpLinkInfo]       = { "PA RX GET(GrpLinkInfo)", 0, 0, 0 },

	// Weird conditions
	[pmCounterPaDuplicateRequests]      = { "PA RX DUPLICATE REQUESTS", 0, 0, 0 },
	[pmCounterPaDeadRmppPacket]         = { "PA RX DEAD RMPP TRANSACTION PACKET", 0, 0, 0 },
	[pmCounterPaDroppedRequests]        = { "PA DROPPED REQUESTS", 0, 0, 0 },
	[pmCounterPaContextNotAvailable]    = { "PA NO AVAILABLE CONTEXTS", 0, 0, 0 },

	// PA RMPP Operations
	[pmCounterPaRmppTxRetries]          = { "PA TX RMPP Retries", 0, 0, 0 },
	[pmCounterRmppStatusStopNoresources]= { "PA TX RMPP STOP", 0, 0, 0 },
	// Not used anywhere
	//[pmCounterRmppStatusAbortTimeTooLong]
	//                                  = { "PA TX RMMP ABORT(TIMEOUT)", 0, 0, 0 },
	[pmCounterRmppStatusAbortInconsistentLastPayloadLength]
	                                    = { "PA TX RMMP ABORT(TIMEOUT)", 0, 0, 0 },
	[pmCounterRmppStatusAbortInconsistentFirstSegnum]
	                                    = { "PA TX RMMP ABORT(BAD FIRST SEG)", 0, 0, 0 },
	[pmCounterRmppStatusAbortBadType]   = { "PA TX RMMP ABORT(BAD TYPE)", 0, 0, 0 },
	[pmCounterRmppStatusAbortNewWindowLastTooSmall]
	                                    = { "PA TX RMMP ABORT(WINDOW TOO SMALL)", 0, 0, 0 },
	[pmCounterRmppStatusAbortSegnumTooBig]
	                                    = { "PA TX RMMP ABORT(SEGNUM TOO BIG)", 0, 0, 0 },
	// Not used anywhere
	//[pmCounterRmppStatusAbortIllegalStatus]
	//                                  = { "PA TX RMMP ABORT(TIMEOUT)", 0, 0, 0 },
	[pmCounterRmppStatusAbortUnsupportedVersion]
	                                    = { "PA TX RMMP ABORT(BAD VERSION)", 0, 0, 0 },
	[pmCounterRmppStatusAbortTooManyRetries]
	                                    = { "PA TX RMMP ABORT(TOO MANY RETRIES)", 0, 0, 0 },
	[pmCounterRmppStatusAbortUnspecified]
	                                    = { "PA TX RMMP ABORT(UNKNOWN)", 0, 0, 0 },
	// PA Request Response Stuff
	[pmCountersPaTxRespMadStatusBusy]   = { "PA TX STATUS BUSY", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusRedirect]
	                                    = { "PA TX STATUS REDIRECT", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusBadClass]
	                                    = { "PA TX STATUS BADCLASS", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusBadMethod]
	                                    = { "PA TX STATUS BADMETHOD", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusBadAttr]= { "PA TX STATUS BADMETHODATTR", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusBadField]
	                                    = { "PA TX STATUS BADFIELD", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusPaUnavailable]
	                                    = { "PA TX STATUS UNAVAILABLE", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusPaNoGroup]
	                                    = { "PA TX STATUS NO GROUP", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusPaNoPort]
	                                    = { "PA TX STATUS NO PORT", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusStlPaNoVf]
	                                    = { "PA TX STATUS NO VF", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusStlPaInvalidParam]
	                                    = { "PA TX STATUS INVALID PARAMETER", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusStlPaNoImage]
	                                    = { "PA TX STATUS NO IMAGE", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusStlPaNoData]
	                                    = { "PA TX STATUS NO DATA", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusStlPaBadData]
	                                    = { "PA TX STATUS BAD DATA", 0, 0, 0 },
	[pmCountersPaTxRespMadStatusUnknown]= { "PA TX STATUS UNKNOWN", 0, 0, 0 },

};

pm_counter_t pmPeakCounters[pmPeakCountersMax] = {
	[pmMaxPaContextsInUse]              = { "PA Maximum Contexts In Use", 0, 0, 0},
	[pmMaxPaContextsFree]              = { "PA Maximum Contexts Free", 0, 0, 0},
	[pmMaxSweepTime]                   = { "Maximum PM Sweep Time in ms", 0, 0, 0},
};

//
// Initialize SM counters
//
void pm_init_counters(void) {
	int i = 0;
	for (i = 0; i < pmCountersMax; ++i) {
		AtomicWrite(&pmCounters[i].sinceLastSweep, 0);
		AtomicWrite(&pmCounters[i].lastSweep, 0);
		AtomicWrite(&pmCounters[i].total, 0);
	}

	if (VSTATUS_OK != vs_stdtime_get(&pmCountersClearedTime)) {
		pmCountersClearedTime = 0;
	}
}

//
// Assigns values of .sinceLastSweep counters to .lastSweep and
// zero's the .sinceLastSweep counters
//
void pm_process_sweep_counters(void) {
	int i = 0;
	INCREMENT_PM_COUNTER(pmCounterPmSweeps);
	for (i = 0; i < pmCountersMax; ++i) {
		// Save value
		AtomicWrite(&pmCounters[i].lastSweep,
		            AtomicRead(&pmCounters[i].sinceLastSweep));

		// Zero it out
		AtomicWrite(&pmCounters[i].sinceLastSweep, 0);
	}

	for (i = 0; i < pmPeakCountersMax; ++i) {
		AtomicWrite(&pmPeakCounters[i].lastSweep,
		            AtomicRead(&pmPeakCounters[i].sinceLastSweep));

		AtomicWrite(&pmPeakCounters[i].sinceLastSweep, 0);
	}
}

//
// Resets the 'last sweep' and 'total' counters
//
void pm_reset_counters(void) {
	int i = 0;
	for (i = 0; i < pmCountersMax; ++i) {
		// Zero it out
		AtomicWrite(&pmCounters[i].lastSweep, 0);
		AtomicWrite(&pmCounters[i].total, 0);
	}

	for (i = 0; i < pmPeakCountersMax; ++i) {
		// Zero it out
		AtomicWrite(&pmPeakCounters[i].lastSweep, 0);
		AtomicWrite(&pmPeakCounters[i].total, 0);
	}

	if (VSTATUS_OK != vs_stdtime_get(&pmCountersClearedTime)) {
		pmCountersClearedTime = 0;
	}
}

#ifndef __VXWORKS__
//
// Like snprintf, but appends the message onto the end of buf
//
extern char * snprintfcat(char * buf, int * len, const char * format, ...);

//
// prints counters to a dynamically allocate buffer - user is
// responsible for freeing it using vs_pool_free()
//
char * pm_print_counters_to_buf(void) {
	char * buf = NULL;
	int i =0;
	int len = 256;
	time_t currentTime;
	time_t elapsedTime;

	/* PR 116307 - buf is an initial buffer to which the data is printed.
	 * If the size of the data to be printed increases, snprintfcat called
	 * below allocates and returns a new buffer and releases buf.
	 * But snprintfcat operates only on on sm_pool and assumes that the buffer
	 * passed to it was also allocated from sm_pool and hence it releases the
	 * passed in buffer from sm_pool. So allocate buf from sm_pool.
	 */
	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("pm_print_counters_to_buf: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

	if (! PmEngineRunning()) {
		snprintf(buf, len, "\nPM Engine is currently not running\n\n");
	}

	// get current time stamp
	if (VSTATUS_OK == vs_stdtime_get(&currentTime)) {
		elapsedTime = currentTime - pmCountersClearedTime;
		snprintf(buf, len, "%35s: %u\n", "Seconds since last cleared", (unsigned int)elapsedTime);
	}

	// show cleared time stamp
	buf = snprintfcat(buf, &len, "%35s: %s", "Time last cleared", ctime(&pmCountersClearedTime));

	buf = snprintfcat(buf, &len, "%35s: %10s %10s %10s\n",
	         "COUNTER", "THIS SWEEP", "LAST SWEEP", "TOTAL");
	buf = snprintfcat(buf, &len, "------------------------------------ "
	                             "---------- "
	                             "---------- ----------\n");
	for (i = 0; i < pmCountersMax; ++i) {
		buf = snprintfcat(buf, &len, "%35s: %10u %10u %10u\n",
		                  pmCounters[i].name,
		                  AtomicRead(&pmCounters[i].sinceLastSweep),
		                  AtomicRead(&pmCounters[i].lastSweep),
		                  AtomicRead(&pmCounters[i].total));
	}

	for (i = 0; i < pmPeakCountersMax; ++i) {
		buf = snprintfcat(buf, &len, "%35s: %10u %10u %10u\n",
		                  pmPeakCounters[i].name,
		                  AtomicRead(&pmPeakCounters[i].sinceLastSweep),
		                  AtomicRead(&pmPeakCounters[i].lastSweep),
		                  AtomicRead(&pmPeakCounters[i].total));
	}

	return buf;
}
#endif

#ifdef __VXWORKS__
void pm_print_counters_to_stream(FILE * out) {
	int i = 0;
	time_t currentTime;
	time_t elapsedTime;
	char tbuf[30];

	if (topology_passcount < 1) {
		sysPrintf("\nSM is currently in the %s state, count = %d\n\n", sm_getStateText(sm_state), (int)sm_smInfo.ActCount);
		return;
	}

	if (! PmEngineRunning()) {
		sysPrintf("\nPM Engine is currently not running\n\n");
	}

	// get current time stamp
	if (VSTATUS_OK == vs_stdtime_get(&currentTime)) {
		elapsedTime = currentTime - pmCountersClearedTime;
		fprintf(out, "\n");
		fprintf(out, "%35s: %u\n", "Seconds since last cleared", (unsigned int)elapsedTime);
	}

	// show cleared time stamp
	UiUtil_GetLocalTimeString(pmCountersClearedTime, tbuf, sizeof(tbuf));
	fprintf(out, "%35s: %s\n", "Time last cleared", tbuf); 
	fprintf(out, "\n");

	fprintf(out, "%35s: %10s %10s %10s\n",
	        "COUNTER", "THIS SWEEP", "LAST SWEEP", "TOTAL");
	fprintf(out, "------------------------------------ "
	             "---------- "
	             "---------- ----------\n");
	for (i = 0; i < pmCountersMax; ++i) {
		fprintf(out, "%35s: %10u %10u %10u\n",
		        pmCounters[i].name,
		        AtomicRead(&pmCounters[i].sinceLastSweep),
		        AtomicRead(&pmCounters[i].lastSweep),
		        AtomicRead(&pmCounters[i].total));
	}

	for (i = 0; i < pmPeakCountersMax; ++i) {
		fprintf(out, "%35s: %10u %10u %10u\n",
		        pmPeakCounters[i].name,
		        AtomicRead(&pmPeakCounters[i].sinceLastSweep),
		        AtomicRead(&pmPeakCounters[i].lastSweep),
		        AtomicRead(&pmPeakCounters[i].total));
	}
}
#endif

