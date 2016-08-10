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

void FormatPmClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	FormatClassPortInfoCapMask(buf, cmask);
	sprintf(buf+strlen(buf), "%s%s",
			(cmask & PM_CLASS_PORT_CAPMASK_SAMPLES_ONLY_SUPPORTED)?"SampleOnly ":"",
			(cmask & PM_CLASS_PORT_CAPMASK_PORT_COUNTERS_XMIT_WAIT_SUPPORTED)?"XmitWait ":"");
}

void FormatPmClassPortInfoCapMask2(char buf[80], uint32 cmask)
{
	FormatClassPortInfoCapMask2(buf, cmask);
	sprintf(buf+strlen(buf), "%s",
			(cmask & PM_CLASS_PORT_CAPMASK2_VENDOR_PORT_COUNTERS)?"VendCntr ":"");
}

void PrintPortCounters(PrintDest_t *dest, int indent, const PORT_COUNTERS *pPortCounters)
{
	PrintFunc(dest, "%*sPerformance: Transmit\n", indent, "");
	PrintFunc(dest, "%*s    Xmit Data             %10u MB (%u Flits)\n",
			indent, "",
			pPortCounters->PortXmitData/(uint32)FLITS_PER_MB,
			pPortCounters->PortXmitData);
	PrintFunc(dest, "%*s    Xmit Pkts             %10u\n",
			indent, "",
			pPortCounters->PortXmitPkts);
	PrintFunc(dest, "%*sPerformance: Receive\n",
			indent, "");
	PrintFunc(dest, "%*s    Rcv Data              %10u MB (%u Flits)\n",
			indent, "",
			pPortCounters->PortRcvData/(uint32)FLITS_PER_MB,
			pPortCounters->PortRcvData);
	PrintFunc(dest, "%*s    Rcv Pkts              %10u\n",
			indent, "",
			pPortCounters->PortRcvPkts);
	PrintFunc(dest, "%*sErrors:                            \n",
			indent, "");
	PrintFunc(dest, "%*s    Symbol Errors         %10u\n",
			indent, "",
	   		pPortCounters->SymbolErrorCounter);
	PrintFunc(dest, "%*s    Link Error Recovery   %10u\n",
			indent, "",
		   	pPortCounters->LinkErrorRecoveryCounter);
	PrintFunc(dest, "%*s    Link Downed           %10u\n",
			indent, "",
		   	pPortCounters->LinkDownedCounter);
	PrintFunc(dest, "%*s    Rcv Errors            %10u\n",
			indent, "",
	   		pPortCounters->PortRcvErrors);
	PrintFunc(dest, "%*s    Rcv Rmt Phys Err      %10u\n",
			indent, "",
	   		pPortCounters->PortRcvRemotePhysicalErrors);
	PrintFunc(dest, "%*s    Rcv Sw Relay Err      %10u\n",
			indent, "",
	   		pPortCounters->PortRcvSwitchRelayErrors);
	PrintFunc(dest, "%*s    Xmit Discards         %10u\n",
			indent, "",
	   		pPortCounters->PortXmitDiscards);
	PrintFunc(dest, "%*s    Xmit Constraint       %10u\n",
			indent, "",
	   		pPortCounters->PortXmitConstraintErrors);
	PrintFunc(dest, "%*s    Rcv Constraint        %10u\n",
			indent, "",
	   		pPortCounters->PortRcvConstraintErrors);
	PrintFunc(dest, "%*s    Local Link Integrity  %10u\n",
			indent, "",
	   		pPortCounters->LocalLinkIntegrityErrors);
	PrintFunc(dest, "%*s    Exc. Buffer Overrun   %10u\n",
			indent, "",
	   		pPortCounters->ExcessiveBufferOverrunErrors);
	PrintFunc(dest, "%*s    VL15 Dropped          %10u\n",
			indent, "",
		   	pPortCounters->VL15Dropped);
#if 0	// no way to clear counter, no use showing
	PrintFunc(dest, "%*s    Xmit Wait             %10u\n",
			indent, "",
		   	pPortCounters->PortXmitWait);
#endif
}
