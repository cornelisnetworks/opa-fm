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

static const char* IbMgmtClassToText(uint8 class)
{
	switch (class) {
	case MCLASS_SM_LID_ROUTED: return "SMA";
	case MCLASS_SM_DIRECTED_ROUTE: return "SMA_DR";
	case MCLASS_SUBN_ADM: return "SA";
	case MCLASS_PERF: return "PMA";
	case MCLASS_BM: return "BMA";
	case MCLASS_DEV_MGT: return "DMA";
	case MCLASS_COMM_MGT: return "CM";
	case MCLASS_SNMP: return "SNMP";
	case MCLASS_CC: return "CCA";
	case MCLASS_VFI_PM: return "PM";
	default: return "Unknown";
	}
}

static const char* IbMadMethodToText(uint8 method)
{
	switch (method) {
	case MMTHD_GET: return "Get";
	case MMTHD_SET: return "Set";
	case MMTHD_SEND: return "Send";
	case MMTHD_TRAP: return "Trap";
	case MMTHD_REPORT: return "Report";
	case MMTHD_TRAP_REPRESS: return "TrapRepress";
	case MMTHD_GET_RESP: return "GetResp";
	case MMTHD_REPORT_RESP: return "ReportResp";
	default: return "Unknown";
	}
}

void PrintMadHeader(PrintDest_t *dest, int indent, const MAD_COMMON *hdr)
{
	MAD_STATUS mad_status;

	if (hdr->MgmtClass == MCLASS_SM_DIRECTED_ROUTE)
			mad_status.AsReg16 = hdr->u.DR.s.Status;
	else
			mad_status = hdr->u.NS.Status;
	PrintFunc(dest, "%*sBaseVersion: %d MgmtClass: 0x%02x (%6s) ClassVersion: %d\n",
				indent, "", hdr->BaseVersion, hdr->MgmtClass,
				IbMgmtClassToText(hdr->MgmtClass), hdr->ClassVersion);
	PrintFunc(dest, "%*sMethod: 0x%02x R: %u (%11s) AttribID: 0x%04x AttribMod: 0x%08x\n",
				indent, "", hdr->mr.s.Method, hdr->mr.s.R,
				IbMadMethodToText(hdr->mr.AsReg8),
				hdr->AttributeID, hdr->AttributeModifier);
	PrintFunc(dest, "%*sTID: 0x%016"PRIx64" Status: 0x%04x (%s)\n",
				indent, "", hdr->TransactionID, mad_status.AsReg16,
				iba_mad_status_msg(mad_status));
}

void PrintSmpHeader(PrintDest_t *dest, int indent, const SMP *smp)
{
	PrintMadHeader(dest, indent, &smp->common);
	PrintFunc(dest, "%*sM_KEY: 0x%016"PRIx64"\n", indent, "", smp->M_Key);
	if (smp->common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE) {
		uint16 offset;
		uint32 i;
		char buf[81];
		PrintFunc(dest, "%*sDR: D: %u Ptr: %3u Cnt: %3u DrSLID: 0x%04x DrDLID: 0x%04x\n",
				indent, "", smp->common.u.DR.s.D,
			   	smp->common.u.DR.HopPointer, smp->common.u.DR.HopCount,
				smp->SmpExt.DirectedRoute.DrSLID,
				smp->SmpExt.DirectedRoute.DrDLID);

		offset = sprintf(buf, "InitPath:");
		// entry 0 is not used
		for (i=1; i<= smp->common.u.DR.HopCount; i++) {
			if ((i-1)%16 == 0 && i>1) {
				PrintFunc(dest, "%*s%s\n", indent, "", buf);
				offset=sprintf(buf, "         ");
			}
			offset += sprintf(&buf[offset], " %3u", smp->SmpExt.DirectedRoute.InitPath[i]);
		}
		PrintFunc(dest, "%*s%s\n", indent, "", buf);

		if (smp->common.u.DR.s.D) {
			offset = sprintf(buf, "RetPath: ");
			// entry 0 is not used
			for (i=1; i<= smp->common.u.DR.HopCount; i++) {
				if ((i-1)%16 == 0 && i>1) {
					PrintFunc(dest, "%*s%s\n", indent, "", buf);
					offset=sprintf(buf, "         ");
				}
				offset += sprintf(&buf[offset], " %3u", smp->SmpExt.DirectedRoute.RetPath[i]);
			}
			PrintFunc(dest, "%*s%s\n", indent, "", buf);
		}
	}
}

void FormatClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	snprintf(buf, 80, "%s%s",
			(cmask & CLASS_PORT_CAPMASK_TRAP)?"Trap ":"",
			(cmask & CLASS_PORT_CAPMASK_NOTICE)?"Notice ":"");
}

void FormatClassPortInfoCapMask2(char buf[80], uint32 cmask)
{
	// no generic bits yet, only class specific
	buf[0] = '\0';
	//sprintf(buf, "%s%s",
	//		(cmask & CLASS_PORT_CAPMASK_TRAP)?"Trap ":"",
	//		(cmask & CLASS_PORT_CAPMASK_NOTICE)?"Notice ":"");
}

void PrintClassPortInfo2(PrintDest_t *dest, int indent, const IB_CLASS_PORT_INFO *pClassPortInfo, formatcapmask_func_t func, formatcapmask2_func_t func2)
{
	char tbuf[8];
	char cbuf[80];
	PrintFunc(dest, "%*sBaseVersion: %d ClassVersion: %d\n",
				indent, "", pClassPortInfo->BaseVersion,
				pClassPortInfo->ClassVersion);
	FormatTimeoutMult(tbuf, pClassPortInfo->u1.s.RespTimeValue);
	(*func)(cbuf, pClassPortInfo->CapMask);
	PrintFunc(dest, "%*sRespTime: %s Capability: 0x%04x: %s\n",
				indent, "", tbuf,
				pClassPortInfo->CapMask, cbuf);
	(*func2)(cbuf, pClassPortInfo->u1.s.CapMask2);
	PrintFunc(dest, "%*sCapability2: 0x%07x: %s\n",
				indent, "",
				pClassPortInfo->u1.s.CapMask2, cbuf);

	PrintFunc(dest, "%*sRedirect: LID: 0x%04x GID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",
				pClassPortInfo->RedirectLID,
				pClassPortInfo->RedirectGID.AsReg64s.H,
				pClassPortInfo->RedirectGID.AsReg64s.L);
	PrintFunc(dest, "%*s          QP: 0x%06x QKey: 0x%08x PKey: 0x%04x SL: %2u\n",
				indent, "",
				pClassPortInfo->u3.s.RedirectQP,
				pClassPortInfo->Redirect_Q_Key,
				pClassPortInfo->Redirect_P_Key,
				pClassPortInfo->u2.s.RedirectSL);
	PrintFunc(dest, "%*s          FlowLabel: 0x%05x TClass: 0x%02x\n",
				indent, "",
				pClassPortInfo->u2.s.RedirectFlowLabel,
				pClassPortInfo->u2.s.RedirectTClass);

	PrintFunc(dest, "%*sTrap: LID: 0x%04x GID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",
				pClassPortInfo->TrapLID,
				pClassPortInfo->TrapGID.AsReg64s.H,
				pClassPortInfo->TrapGID.AsReg64s.L);
	PrintFunc(dest, "%*s      QP: 0x%06x QKey: 0x%08x PKey: 0x%04x SL: %2u\n",
				indent, "",
				pClassPortInfo->u5.s.TrapQP,
				pClassPortInfo->Trap_Q_Key,
				pClassPortInfo->Trap_P_Key,
				pClassPortInfo->u4.s.TrapSL);
	PrintFunc(dest, "%*s      FlowLabel: 0x%05x HopLimit: 0x%02x TClass: 0x%02x\n",
				indent, "",
				pClassPortInfo->u4.s.TrapFlowLabel,
				pClassPortInfo->u5.s.TrapHopLimit,
				pClassPortInfo->u4.s.TrapTClass);
}

void PrintClassPortInfo(PrintDest_t *dest, int indent, const IB_CLASS_PORT_INFO *pClassPortInfo)
{
	PrintClassPortInfo2(dest, indent, pClassPortInfo, FormatClassPortInfoCapMask, FormatClassPortInfoCapMask2);
}

void PrintMad(PrintDest_t *dest, int indent, const MAD *pMad)
{
	switch (pMad->common.MgmtClass) {
	case MCLASS_SM_LID_ROUTED:
	case MCLASS_SM_DIRECTED_ROUTE:
		PrintSmp(dest, indent, (const SMP*)pMad);
		break;
	case MCLASS_SUBN_ADM:
	case MCLASS_PERF:
	case MCLASS_BM:
	case MCLASS_DEV_MGT:
	case MCLASS_COMM_MGT:
	case MCLASS_SNMP:
	case MCLASS_DEV_CONF_MGT:
	case MCLASS_DTA:
	case MCLASS_CC:
	case MCLASS_VFI_PM:
	default:
		PrintMadHeader(dest, indent, &pMad->common);
		// TBD hex dump of Data
		break;
	}
}

// TBD void PrintNotice(PrintDest_t *dest, int indent, const NOTICE *pNotice)
// TBD also trap 64-67 details
//
// TBD fix opamgt to dump mads using IbPrint
// add a raw hex dump to IbPrint
// add a MAD dump to IbPrint, decode mad header and output in correct format
// for all but SA and PA RMPP mads.
