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


#include "stl_print.h"
#include <iba/stl_pm.h>

void PrintStlPortStatusRsp(PrintDest_t *dest, int indent, const STL_PORT_STATUS_RSP *pStlPortStatusRsp)
{
	int i, j;

 	PrintFunc(dest, "%*sPort Number             %u\n",
		indent, "",
		pStlPortStatusRsp->PortNumber);
	PrintFunc(dest, "%*sVL Select Mask      0x%08x\n",
		indent+4, "",
		pStlPortStatusRsp->VLSelectMask);

	PrintFunc(dest, "%*sPerformance: Transmit\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Xmit Data             %20"PRIu64" MB (%"PRIu64" Flits)\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitData/FLITS_PER_MB,
		pStlPortStatusRsp->PortXmitData);
	PrintFunc(dest, "%*s    Xmit Pkts             %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitPkts);
	PrintFunc(dest, "%*s    MC Xmt Pkts           %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortMulticastXmitPkts);

	PrintFunc(dest, "%*sPerformance: Receive\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Rcv Data              %20"PRIu64" MB (%"PRIu64" Flits)\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvData/FLITS_PER_MB,
		pStlPortStatusRsp->PortRcvData);
	PrintFunc(dest, "%*s    Rcv Pkts              %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvPkts);
	PrintFunc(dest, "%*s    MC Rcv Pkts           %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortMulticastRcvPkts);

	PrintFunc(dest, "%*sErrors: Signal Integrity\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Link Qual Indicator   %20u (%s)\n",
		indent+4, "",
		pStlPortStatusRsp->lq.s.LinkQualityIndicator,
		StlLinkQualToText(pStlPortStatusRsp->lq.s.LinkQualityIndicator));
	PrintFunc(dest, "%*s    Uncorrectable Errors  %20u\n",	//8 bit
		indent+4, "",
		pStlPortStatusRsp->UncorrectableErrors);
	PrintFunc(dest, "%*s    Link Downed           %20u\n",	// 32 bit
		indent+4, "",
		pStlPortStatusRsp->LinkDowned);
	PrintFunc(dest, "%*s    Rcv Errors            %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvErrors);
	PrintFunc(dest, "%*s    Exc. Buffer Overrun   %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->ExcessiveBufferOverruns);
	PrintFunc(dest, "%*s    FM Config Errors      %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->FMConfigErrors);
	PrintFunc(dest, "%*s    Local Link Integ Err  %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->LocalLinkIntegrityErrors);
	PrintFunc(dest, "%*s    Link Error Recovery   %20u\n",	// 32 bit
		indent+4, "",
		pStlPortStatusRsp->LinkErrorRecovery);
	PrintFunc(dest, "%*s    Rcv Rmt Phys Err      %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvRemotePhysicalErrors);

	PrintFunc(dest, "%*sErrors: Security\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Xmit Constraint       %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitConstraintErrors);
	PrintFunc(dest, "%*s    Rcv Constraint        %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvConstraintErrors);

	PrintFunc(dest, "%*sErrors: Routing and Other Errors\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Rcv Sw Relay Err      %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvSwitchRelayErrors);
	PrintFunc(dest, "%*s    Xmit Discards         %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitDiscards);

	PrintFunc(dest, "%*sPerformance: Congestion\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Congestion Discards   %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->SwPortCongestion);
	PrintFunc(dest, "%*s    Rcv FECN              %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvFECN);
	PrintFunc(dest, "%*s    Rcv BECN              %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvBECN);
	PrintFunc(dest, "%*s    Mark FECN             %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortMarkFECN);
	PrintFunc(dest, "%*s    Xmit Time Congestion  %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitTimeCong);
	PrintFunc(dest, "%*s    Xmit Wait             %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitWait);

	PrintFunc(dest, "%*sPerformance: Bubbles\n",
		indent+4, "");
	PrintFunc(dest, "%*s    Xmit Wasted BW        %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitWastedBW);
	PrintFunc(dest, "%*s    Xmit Wait Data        %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortXmitWaitData);
	PrintFunc(dest, "%*s    Rcv Bubble            %20"PRIu64"\n",
		indent+4, "",
		pStlPortStatusRsp->PortRcvBubble);




	/* per_VL counters */
	for (i = 0, j = 0; i < STL_MAX_VLS; i++) {
		if ((pStlPortStatusRsp->VLSelectMask >> i) & (uint64)1) {
			PrintFunc(dest, "\n");
			PrintFunc(dest, "%*s    VL Number    %d\n",
				indent+4, "",
				i);
			PrintFunc(dest, "%*s    Performance: Transmit\n",
				indent+8, "");
			PrintFunc(dest, "%*s         Xmit Data             %20"PRIu64" MB (%"PRIu64" Flits)\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitData/FLITS_PER_MB,
				pStlPortStatusRsp->VLs[j].PortVLXmitData);
			PrintFunc(dest, "%*s         Xmit Pkts             %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitPkts);

			PrintFunc(dest, "%*s    Performance: Receive\n",
				indent+8, "");
			PrintFunc(dest, "%*s         Rcv Data              %20"PRIu64" MB (%"PRIu64" Flits)\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLRcvData/FLITS_PER_MB,
				pStlPortStatusRsp->VLs[j].PortVLRcvData);
			PrintFunc(dest, "%*s         Rcv Pkts              %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLRcvPkts);

			PrintFunc(dest, "%*s    Performance: Congestion\n",
				indent+8, "");
			PrintFunc(dest, "%*s         Congestion Discards   %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].SwPortVLCongestion);
			PrintFunc(dest, "%*s         Rcv FECN              %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLRcvFECN);
			PrintFunc(dest, "%*s         Rcv BECN              %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLRcvBECN);
			PrintFunc(dest, "%*s         Mark FECN             %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLMarkFECN);
			PrintFunc(dest, "%*s         Xmit Time Congestion  %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitTimeCong);
			PrintFunc(dest, "%*s         Xmit Wait             %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitWait);

			PrintFunc(dest, "%*s    Performance: Bubbles\n",
				indent+8, "");
			PrintFunc(dest, "%*s         Xmit Wasted BW        %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitWastedBW);
			PrintFunc(dest, "%*s         Xmit Wait Data        %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitWaitData);
			PrintFunc(dest, "%*s         Rcv Bubble            %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLRcvBubble);

			PrintFunc(dest, "%*s    Errors: Other\n",
				indent+8, "");
			PrintFunc(dest, "%*s         Xmit Discards         %20"PRIu64"\n",
				indent+8, "",
				pStlPortStatusRsp->VLs[j].PortVLXmitDiscards);

			j++;
		}
	}
	return;
}

void PrintStlPortStatusRspSummary(PrintDest_t *dest, int indent, const STL_PORT_STATUS_RSP *pStlPortStatusRsp, int printLineByLine)
{
    if (printLineByLine) {
		PrintIntWithDotsDec(dest, indent, "XmitDataMB", pStlPortStatusRsp->PortXmitData/FLITS_PER_MB);
		PrintIntWithDotsDec(dest, indent, "XmitPkts", pStlPortStatusRsp->PortXmitPkts);
		PrintIntWithDotsDec(dest, indent, "RcvDataMB", pStlPortStatusRsp->PortRcvData/FLITS_PER_MB);
		PrintIntWithDotsDec(dest, indent, "RcvPkts", pStlPortStatusRsp->PortRcvPkts);
		PrintIntWithDotsDec(dest, indent, "LinkQualityIndicator", pStlPortStatusRsp->lq.s.LinkQualityIndicator);
	} else {
		PrintFunc(dest, "%*sXmit Data: %18"PRIu64" MB Pkts: %20"PRIu64"\n",
			indent, "",
			pStlPortStatusRsp->PortXmitData/FLITS_PER_MB,
			pStlPortStatusRsp->PortXmitPkts);
		PrintFunc(dest, "%*sRecv Data: %18"PRIu64" MB Pkts: %20"PRIu64"\n",
			indent, "",
			pStlPortStatusRsp->PortRcvData/FLITS_PER_MB,
			pStlPortStatusRsp->PortRcvPkts);
		PrintFunc(dest, "%*sLink Quality: %u (%s)\n",
			indent, "",
			pStlPortStatusRsp->lq.s.LinkQualityIndicator,
			StlLinkQualToText(pStlPortStatusRsp->lq.s.LinkQualityIndicator));

	}
}

void PrintStlClearPortStatus(PrintDest_t *dest, int indent, const STL_CLEAR_PORT_STATUS *pStlClearPortStatus)
{
	char buf[128] = {0};

	if (pStlClearPortStatus->PortSelectMask[0]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlClearPortStatus->PortSelectMask[0],
			pStlClearPortStatus->PortSelectMask[1],
			pStlClearPortStatus->PortSelectMask[2],
			pStlClearPortStatus->PortSelectMask[3]);
	} else if (pStlClearPortStatus->PortSelectMask[1]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlClearPortStatus->PortSelectMask[1],
			pStlClearPortStatus->PortSelectMask[2],
			pStlClearPortStatus->PortSelectMask[3]);
	} else if (pStlClearPortStatus->PortSelectMask[2]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlClearPortStatus->PortSelectMask[2],
			pStlClearPortStatus->PortSelectMask[3]);
	} else {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64"\n",
			indent, "",
			pStlClearPortStatus->PortSelectMask[3]);
	}
	FormatStlCounterSelectMask(buf, pStlClearPortStatus->CounterSelectMask);
	PrintFunc(dest, "%*s Counter Sel Mask   0x%08x: %s\n",
		indent, "",
		pStlClearPortStatus->CounterSelectMask.AsReg32,
		buf);
}

void PrintStlDataPortCountersRsp(PrintDest_t *dest, int indent, const STL_DATA_PORT_COUNTERS_RSP *pStlDataPortCountersRsp)
{
	int i, j, ii, jj;
	uint64 portSelectMask;
	uint32 vlSelectMask;
	struct _port_dpctrs *port;

	if (pStlDataPortCountersRsp->PortSelectMask[0]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlDataPortCountersRsp->PortSelectMask[0],
			pStlDataPortCountersRsp->PortSelectMask[1],
			pStlDataPortCountersRsp->PortSelectMask[2],
			pStlDataPortCountersRsp->PortSelectMask[3]);
	} else if (pStlDataPortCountersRsp->PortSelectMask[1]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlDataPortCountersRsp->PortSelectMask[1],
			pStlDataPortCountersRsp->PortSelectMask[2],
			pStlDataPortCountersRsp->PortSelectMask[3]);
	} else if (pStlDataPortCountersRsp->PortSelectMask[2]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlDataPortCountersRsp->PortSelectMask[2],
			pStlDataPortCountersRsp->PortSelectMask[3]);
	} else {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64"\n",
			indent, "",
			pStlDataPortCountersRsp->PortSelectMask[3]);
	}
	PrintFunc(dest, "%*s VL Select Mask     0x%08x\n",
		indent, "",
		pStlDataPortCountersRsp->VLSelectMask);
	PrintFunc(dest, "%*s LocalLinkIntegrity Resolution %20u\n",
		indent, "",
		StlShiftToResolution(pStlDataPortCountersRsp->res.s.LocalLinkIntegrityResolution, RES_ADDER_LLI));
	PrintFunc(dest, "%*s LinkErrorRecovery Resolution  %20u\n",
		indent, "",
		StlShiftToResolution(pStlDataPortCountersRsp->res.s.LinkErrorRecoveryResolution, RES_ADDER_LER));


	/* Count the bits in the mask and process the ports in succession */
	/* Assume that even though port numbers may not be contiguous, that entries */
	/*   in the array are */
	port = (struct _port_dpctrs *)&(pStlDataPortCountersRsp->Port[0]);
	for (i = 3; i >= 0; i--) {
		portSelectMask = pStlDataPortCountersRsp->PortSelectMask[i];
		for (j = 0; portSelectMask && j < 64; j++, portSelectMask>>= (uint64)1) {
			if (portSelectMask & (uint64)1) {
				PrintFunc(dest, "%*s    Port Number     %u\n",
					indent, "",
					port->PortNumber);
				PrintFunc(dest, "%*s    Xmit Data             %20"PRIu64" MB (%"PRIu64" Flits)\n",
					indent+4, "",
					port->PortXmitData/FLITS_PER_MB,
					port->PortXmitData);
				PrintFunc(dest, "%*s    Rcv Data              %20"PRIu64" MB (%"PRIu64" Flits)\n",
					indent+4, "",
					port->PortRcvData/FLITS_PER_MB,
					port->PortRcvData);
				PrintFunc(dest, "%*s    Xmit Pkts             %20"PRIu64"\n",
					indent+4, "",
					port->PortXmitPkts);
				PrintFunc(dest, "%*s    Rcv Pkts              %20"PRIu64"\n",
					indent+4, "",
					port->PortRcvPkts);
				PrintFunc(dest, "%*s    MC Xmit Pkts          %20"PRIu64"\n",
					indent+4, "",
					port->PortMulticastXmitPkts);
				PrintFunc(dest, "%*s    MC Rcv Pkts           %20"PRIu64"\n",
					indent+4, "",
					port->PortMulticastRcvPkts);
				PrintFunc(dest, "%*s    Signal Integrity Errors: \n",
						indent, "");
				PrintFunc(dest, "%*s    Link Qual. Indicator  %20u (%s)\n",
					indent+4, "",
					port->lq.s.LinkQualityIndicator,
					StlLinkQualToText(port->lq.s.LinkQualityIndicator));
				PrintFunc(dest, "%*s    Congestion: \n",
						indent, "");
				PrintFunc(dest, "%*s    Congestion Discards   %20"PRIu64"\n",
					indent+4, "",
					port->SwPortCongestion);
				PrintFunc(dest, "%*s    Rcv FECN              %20"PRIu64"\n",
					indent+4, "",
					port->PortRcvFECN);
				PrintFunc(dest, "%*s    Rcv BECN              %20"PRIu64"\n",
					indent+4, "",
					port->PortRcvBECN);
				PrintFunc(dest, "%*s    Mark FECN             %20"PRIu64"\n",
					indent+4, "",
					port->PortMarkFECN);
				PrintFunc(dest, "%*s    Xmit Time Cong        %20"PRIu64"\n",
					indent+4, "",
					port->PortXmitTimeCong);
				PrintFunc(dest, "%*s    Xmit Wait             %20"PRIu64"\n",
					indent+4, "",
					port->PortXmitWait);
				PrintFunc(dest, "%*s    Bubbles: \n",
						indent, "");
				PrintFunc(dest, "%*s    Xmit Wasted BW        %20"PRIu64"\n",
					indent+4, "",
					port->PortXmitWastedBW);
				PrintFunc(dest, "%*s    Xmit Wait Data        %20"PRIu64"\n",
					indent+4, "",
					port->PortXmitWaitData);
				PrintFunc(dest, "%*s    Rcv Bubble            %20"PRIu64"\n",
					indent+4, "",
					port->PortRcvBubble);
				PrintFunc(dest, "%*s    Error Counter Summary %20"PRIu64"\n",
					indent+4, "",
					port->PortErrorCounterSummary);
				/* Count the bits in the mask and process the VLs in succession */
				/* Assume that even though VL numbers may not be contiguous, that entries */
				/*   in the array are */
				vlSelectMask = pStlDataPortCountersRsp->VLSelectMask;
				for (ii = 0, jj = 0; vlSelectMask && ii < STL_MAX_VLS; ii++, vlSelectMask >>= (uint32)1) {
					if (vlSelectMask & (uint32)1) {
						PrintFunc(dest, "%*s    VL Number     %d\n",
							indent+4, "",
							ii);
						PrintFunc(dest, "%*s         Xmit Data             %20"PRIu64" MB (%"PRIu64" Flits)\n",
							indent+4, "",
							port->VLs[jj].PortVLXmitData/FLITS_PER_MB,
							port->VLs[jj].PortVLXmitData);
						PrintFunc(dest, "%*s         Rcv Data              %20"PRIu64" MB (%"PRIu64" Flits)\n",
							indent+4, "",
							port->VLs[jj].PortVLRcvData/FLITS_PER_MB,
							port->VLs[jj].PortVLRcvData);
						PrintFunc(dest, "%*s         Xmit Pkts             %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLXmitPkts);
						PrintFunc(dest, "%*s         Rcv Pkts              %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLRcvPkts);
						PrintFunc(dest, "%*s         Congestion Discards   %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].SwPortVLCongestion);
						PrintFunc(dest, "%*s         Rcv FECN              %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLRcvFECN);
						PrintFunc(dest, "%*s         Rcv BECN              %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLRcvBECN);
						PrintFunc(dest, "%*s         Mark FECN             %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLMarkFECN);
						PrintFunc(dest, "%*s         Xmit Time Cong        %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLXmitTimeCong);
						PrintFunc(dest, "%*s         Xmit Wait             %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLXmitWait);
						PrintFunc(dest, "%*s         Xmit Wasted BW        %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLXmitWastedBW);
						PrintFunc(dest, "%*s         Xmit Wait Data        %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLXmitWaitData);
						PrintFunc(dest, "%*s         Rcv Bubble            %20"PRIu64"\n",
							indent+4, "",
							port->VLs[jj].PortVLRcvBubble);
						jj++;
					}
				}
				port = (struct _port_dpctrs *)&(port->VLs[jj]);
			}
		}
	}
}

void PrintStlErrorPortCountersRsp(PrintDest_t *dest, int indent, const STL_ERROR_PORT_COUNTERS_RSP *pStlErrorPortCountersRsp)
{
	int i, j, ii, p;
	uint64 portSelectMask;
	uint32 vlSelectMask;
	uint8  vlCount;
	struct _port_epctrs *ePort;

	if (pStlErrorPortCountersRsp->PortSelectMask[0]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlErrorPortCountersRsp->PortSelectMask[0],
			pStlErrorPortCountersRsp->PortSelectMask[1],
			pStlErrorPortCountersRsp->PortSelectMask[2],
			pStlErrorPortCountersRsp->PortSelectMask[3]); p = 0;
	} else if (pStlErrorPortCountersRsp->PortSelectMask[1]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlErrorPortCountersRsp->PortSelectMask[1],
			pStlErrorPortCountersRsp->PortSelectMask[2],
			pStlErrorPortCountersRsp->PortSelectMask[3]); p = 1;
	} else if (pStlErrorPortCountersRsp->PortSelectMask[2]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlErrorPortCountersRsp->PortSelectMask[2],
			pStlErrorPortCountersRsp->PortSelectMask[3]); p = 2;
	} else {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64"\n",
			indent, "",
			pStlErrorPortCountersRsp->PortSelectMask[3]); p = 3;
	}
	PrintFunc(dest, "%*s VL Select Mask     0x%08x\n",
		indent, "",
		pStlErrorPortCountersRsp->VLSelectMask);

	/* Count the bits in the mask and process the ports in succession */
	/* Assume that even though port numbers may not be contiguous, that entries */
	/*   in the array are */

	ePort = (struct _port_epctrs *)&pStlErrorPortCountersRsp->Port[0];
	for (i = 3; i >= p; i--) {
		portSelectMask = pStlErrorPortCountersRsp->PortSelectMask[i];
		for (j = 0; portSelectMask && j < 64; j++, portSelectMask >>= (uint64)1) {
			if (portSelectMask & (uint64)1) {
				PrintFunc(dest, "%*s   Port Number     %u\n",
						indent+1, "", ePort->PortNumber );
				PrintFunc(dest, "%*s    Signal Integrity Errors: \n",
						indent, "");
				PrintFunc(dest, "%*s   Uncorrectable Errors    %10u\n",
						indent+4, "", ePort->UncorrectableErrors);
				PrintFunc(dest, "%*s   Link Downed             %10u\n",
						indent+4, "", ePort->LinkDowned);
				PrintFunc(dest, "%*s   Rcv Errors              %10"PRIu64"\n",
						indent+4, "", ePort->PortRcvErrors);
				PrintFunc(dest, "%*s   Exc. Buffer Overrun     %10"PRIu64"\n",
						indent+4, "", ePort->ExcessiveBufferOverruns);
				PrintFunc(dest, "%*s   FM Config Errors        %10"PRIu64"\n",
						indent+4, "", ePort->FMConfigErrors);
				PrintFunc(dest, "%*s   Link Error Recovery     %10u\n",
						indent+4, "", ePort->LinkErrorRecovery);
				PrintFunc(dest, "%*s   Local Link Integrity    %10"PRIu64"\n",
						indent+4, "", ePort->LocalLinkIntegrityErrors);
				PrintFunc(dest, "%*s   Rcv Rmt Phys. Errors    %10"PRIu64"\n",
						indent+4, "", ePort->PortRcvRemotePhysicalErrors);
				PrintFunc(dest, "%*s    Security Errors: \n",
						indent, "");
				PrintFunc(dest, "%*s   Xmit Constraint Errors  %10"PRIu64"\n",
						indent+4, "", ePort->PortXmitConstraintErrors);
				PrintFunc(dest, "%*s   Rcv Constraint Errors   %10"PRIu64"\n",
						indent+4, "", ePort->PortRcvConstraintErrors);
				PrintFunc(dest, "%*s    Routing and Other Errors: \n",
						indent, "");
				PrintFunc(dest, "%*s   Rcv Switch Relay        %10"PRIu64"\n",
						indent+4, "", ePort->PortRcvSwitchRelayErrors);
				PrintFunc(dest, "%*s   Xmit Discards           %10"PRIu64"\n",
						indent+4, "", ePort->PortXmitDiscards);
				/* Count the bits in the mask and process the VLs in succession */
				/* Assume that even though VL numbers may not be contiguous, that entries */
				/*   in the array are */
				for (vlCount = 0, ii = 0, vlSelectMask = pStlErrorPortCountersRsp->VLSelectMask;
					  vlSelectMask && ii < 32; ii++, vlSelectMask >>= (uint32)1) {
					if (vlSelectMask & (uint32)1) {
						PrintFunc(dest, "%*s     VL Number     %d\n",
							indent+4, "", ii);
						PrintFunc(dest, "%*s     Xmit Discards         %10"PRIu64"\n",
							indent+8, "", ePort->VLs[vlCount++].PortVLXmitDiscards);
					}
				}
				ePort = (struct _port_epctrs *)&ePort->VLs[vlCount];
			}
		}
	}
	return;
}

/**
 * @param headerOnly when true, do not print per-port info
 */
void PrintStlErrorInfoRsp(PrintDest_t *dest, int indent, const STL_ERROR_INFO_RSP *pStlErrorInfoRsp,
  int headerOnly)
{
	int i, j, k;
	int ii;
	uint8 bits;
	uint8 *pChar;
	char *pBuf;
	char displayBuf[256];

	if (pStlErrorInfoRsp->PortSelectMask[0]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlErrorInfoRsp->PortSelectMask[0],
			pStlErrorInfoRsp->PortSelectMask[1],
			pStlErrorInfoRsp->PortSelectMask[2],
			pStlErrorInfoRsp->PortSelectMask[3]);
	} else if (pStlErrorInfoRsp->PortSelectMask[1]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlErrorInfoRsp->PortSelectMask[1],
			pStlErrorInfoRsp->PortSelectMask[2],
			pStlErrorInfoRsp->PortSelectMask[3]);
	} else if (pStlErrorInfoRsp->PortSelectMask[2]) {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64" %016"PRIx64"\n",
			indent, "",
			pStlErrorInfoRsp->PortSelectMask[2],
			pStlErrorInfoRsp->PortSelectMask[3]);
	} else {
		PrintFunc(dest, "%*s Port Select Mask   0x%016"PRIx64"\n",
			indent, "",
			pStlErrorInfoRsp->PortSelectMask[3]);
	}
	PrintFunc(dest, "%*s Error Select Mask  0x%08x\n",
		indent, "",
		pStlErrorInfoRsp->ErrorInfoSelectMask.AsReg32);

	if (headerOnly)
		return;

	for (i = 3; i >= 0; i--) {
		for (j = 0, k = 0; j < 64; j++) {
			if ((pStlErrorInfoRsp->PortSelectMask[i] >> j) & (uint64)1) {
				PrintFunc(dest, "%*s    Port Number     %u\n",
					indent, "",
					pStlErrorInfoRsp->Port[k].PortNumber);
				if (pStlErrorInfoRsp->Port[k].UncorrectableErrorInfo.s.Status) {
					PrintFunc(dest, "%*s    Uncorr Error Info     %s\n",
						indent+4, "",
						":");
					PrintFunc(dest, "%*s        Error Code        %s (%u)\n",
						indent+4, "",
						UncorrectableErrorInfoToText(pStlErrorInfoRsp->Port[k].UncorrectableErrorInfo.s.ErrorCode),
						pStlErrorInfoRsp->Port[k].UncorrectableErrorInfo.s.ErrorCode);
				} else {
					PrintFunc(dest, "%*s    Uncorr Error Info     %s\n",
						indent+4, "",
						"N/A");
				}
				if (pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.Status) {
					PrintFunc(dest, "%*s    Rcv Error Info        %s\n",
						indent+4, "",
						":");
					PrintFunc(dest, "%*s        Error Code        %s (%u)\n",
						indent+4, "",
						PortRcvErrorInfoToText(pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.ErrorCode),
						pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.ErrorCode);

					if ((pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.ErrorCode == RCVERRORINFO1) ||
						((pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.ErrorCode >= RCVERRORINFO4) &&
							(pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.ErrorCode <= RCVERRORINFO12))) {
						pBuf = displayBuf;
						for (ii = 0, pChar = (uint8 *)pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.ErrorInfo.EI1to12.PacketFlit1; ii < 8; ii++) {
							sprintf(pBuf, "0x%02x ", *pChar++);
							pBuf += 5;
						}
						bits = (uint8)pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.ErrorInfo.EI1to12.s.Flit1Bits;
						pChar = &bits;
						sprintf(pBuf, "%01x ", *pChar++);
						pBuf += 2;
						*pBuf = '\0';
						PrintFunc(dest, "%*s        Flit 1:           %s\n",
							indent+4, "",
							displayBuf);
						pBuf = displayBuf;
						for (ii = 0, pChar = (uint8 *)pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.ErrorInfo.EI1to12.PacketFlit2; ii < 8; ii++) {
							sprintf(pBuf, "0x%02x ", *pChar++);
							pBuf += 5;
						}
						bits = (uint8)pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.ErrorInfo.EI1to12.s.Flit2Bits;
						pChar = &bits;
						sprintf(pBuf, "%01x ", *pChar++);
						pBuf += 2;
						*pBuf = '\0';
						PrintFunc(dest, "%*s        Flit 2:           %s\n",
							indent+4, "",
							displayBuf);
					} else if (pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.s.ErrorCode == RCVERRORINFO13) {
						pBuf = displayBuf;
						for (ii = 0, pChar = (uint8 *)pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.ErrorInfo.EI13.PacketBytes; ii < 8; ii++) {
							sprintf(pBuf, "0x%02x ", *pChar++);
							pBuf += 5;
						}
						bits = (uint8)pStlErrorInfoRsp->Port[k].PortRcvErrorInfo.ErrorInfo.EI13.s.FlitBits;
						pChar = &bits;
						sprintf(pBuf, "%01x ", *pChar++);
						pBuf += 2;
						*pBuf = '\0';
						PrintFunc(dest, "%*s        VL Marker Flit:   %s\n",
							indent+4, "",
							displayBuf);
					} else {
						/* bad error code */
					}
				} else {
					PrintFunc(dest, "%*s    Rcv Error Info        %s\n",
						indent+4, "",
						"N/A");
				}
				if (pStlErrorInfoRsp->Port[k].ExcessiveBufferOverrunInfo.s.Status) {
					PrintFunc(dest, "%*s    Exc Buf Overrun Info  %s\n",
						indent+4, "",
						":");
					PrintFunc(dest, "%*s        SC                %u\n",
						indent+4, "",
						pStlErrorInfoRsp->Port[k].ExcessiveBufferOverrunInfo.s.SC);
				} else {
					PrintFunc(dest, "%*s    Exc Buf Overrun Info  %s\n",
						indent+4, "",
						"N/A");
				}
				if (pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.s.Status) {
					PrintFunc(dest, "%*s    FM Config Error Info  %s\n",
						indent+4, "",
						":");
					PrintFunc(dest, "%*s        Error Code        %s (%u)\n",
						indent+4, "",
						FMConfigErrorInfoToText(pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.s.ErrorCode),
						pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.s.ErrorCode);
					switch(pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.s.ErrorCode) {
					case 0:
					case 1:
					case 2:
					case 8:
						PrintFunc(dest, "%*s        Distance:     %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.ErrorInfo.EI0to2_8.Distance);
						break;
					case 3:
					case 4:
					case 5:
						PrintFunc(dest, "%*s        VL:           %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.ErrorInfo.EI3to5.VL);
						break;
					case 6:
						PrintFunc(dest, "%*s        Bad Flit Bits:    0x%010x\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.ErrorInfo.EI6.BadFlitBits);
						break;
					case 7:
						PrintFunc(dest, "%*s        SC:           %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].FMConfigErrorInfo.ErrorInfo.EI7.SC);
						break;
					default:
						/* bad error code */
						break;
					}
				} else {
					PrintFunc(dest, "%*s    FM Config Error Info  %s\n",
						indent+4, "",
						"N/A");
				}
				if (pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.s.Status) {
					PrintFunc(dest, "%*s    Rcv Sw Rel Info       %s\n",
						indent+4, "",
						":");
					PrintFunc(dest, "%*s        Error Code        %s (%u)\n",
						indent+4, "",
						PortRcvSwitchRelayErrorInfoToText(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.s.ErrorCode),
						pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.s.ErrorCode);
					switch (pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.s.ErrorCode) {
					case 0:
						PrintFunc(dest, "%*s        DLID:             0x%010x\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI0.DLID);
						PrintFunc(dest, "%*s        SLID:             0x%010x\n",
							indent+4, "",
							(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.SLID_23_16 << 16) |
							(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.SLID_15_0));
						PrintFunc(dest, "%*s        SC:               %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI0.SC);
						PrintFunc(dest, "%*s        RC:               %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.s.RC);
						break;
					case 2:
						PrintFunc(dest, "%*s        DLID:             0x%010x\n",
							indent+4, "",
							(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI2.DLID_23_16 << 16) |
							(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI2.DLID_15_0));
						PrintFunc(dest, "%*s        SLID:             0x%010x\n",
							indent+4, "",
							(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.SLID_23_16 << 16) |
							(pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.SLID_15_0));
						PrintFunc(dest, "%*s        Egress Port Num:  %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI2.EgressPortNum);
						PrintFunc(dest, "%*s        RC:               %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.s.RC);
						break;
					case 3:
						PrintFunc(dest, "%*s        Egress Port Num:  %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI3.EgressPortNum);
						PrintFunc(dest, "%*s        SC:               %u\n",
							indent+4, "",
							pStlErrorInfoRsp->Port[k].PortRcvSwitchRelayErrorInfo.ErrorInfo.EI3.SC);
						break;
					default:
						/* bad error code */
						break;
					}
				} else {
					PrintFunc(dest, "%*s    Rcv Sw Rel Info       %s\n",
						indent+4, "",
						"N/A");
				}
				if (pStlErrorInfoRsp->Port[k].PortXmitConstraintErrorInfo.s.Status) {
					PrintFunc(dest, "%*s    Xmit Constraint Info  %s\n",
						indent+4, "",
						":");
					PrintFunc(dest, "%*s        P_Key             0x%010x\n",
						indent+4, "",
						pStlErrorInfoRsp->Port[k].PortXmitConstraintErrorInfo.P_Key);
					PrintFunc(dest, "%*s        SLID              0x%010x\n",
						indent+4, "",
						pStlErrorInfoRsp->Port[k].PortXmitConstraintErrorInfo.SLID);
				} else {
					PrintFunc(dest, "%*s    Xmit Constraint Info  %s\n",
						indent+4, "",
						"N/A");
				}
				if (pStlErrorInfoRsp->Port[k].PortRcvConstraintErrorInfo.s.Status) {
					PrintFunc(dest, "%*s    Rcv Constraint Info   %s\n",
						indent+4, "",
						":");
					if(pStlErrorInfoRsp->Port[k].PortRcvConstraintErrorInfo.s.ErrorCode)
						PrintFunc(dest, "%*s        Error Code        %s (%u)\n",
							indent+4, "",
							PortRcvConstraintErrorInfoToText(
								pStlErrorInfoRsp->Port[k].PortRcvConstraintErrorInfo.s.ErrorCode),
							pStlErrorInfoRsp->Port[k].PortRcvConstraintErrorInfo.s.ErrorCode);
					PrintFunc(dest, "%*s        P_Key             0x%010x\n",
						indent+4, "",
						pStlErrorInfoRsp->Port[k].PortRcvConstraintErrorInfo.P_Key);
					PrintFunc(dest, "%*s        SLID              0x%010x\n",
						indent+4, "",
						pStlErrorInfoRsp->Port[k].PortRcvConstraintErrorInfo.SLID);
				} else {
					PrintFunc(dest, "%*s    Rcv Constraint Info   %s\n",
						indent+4, "",
						"N/A");
				}
				k++;
			}
		}
	}
	return;
}

