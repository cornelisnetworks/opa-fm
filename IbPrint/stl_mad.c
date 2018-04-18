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

#include "ibprint.h"
#include "stl_print.h"

void PrintStlSmpHeader(PrintDest_t *dest, int indent, const STL_SMP *smp)
{
	PrintMadHeader(dest, indent, &smp->common);
	PrintFunc(dest, "%*sM_KEY: 0x%016"PRIx64"\n", indent, "", smp->M_Key);
	if (smp->common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE) {
		uint16 offset;
		uint32 i;
		char buf[128];
		PrintFunc(dest, "%*sDR: D: %u Ptr: %3u Cnt: %3u DrSLID: 0x%08x DrDLID: 0x%08x\n",
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

void PrintStlAggregateMember(PrintDest_t *dest, int indent, const STL_AGGREGATE *aggr)
{
	PrintFunc(dest, "%*sAttrID: 0x%04x\n", indent, "", aggr->AttributeID);
	PrintFunc(dest, "%*sError: 0x%02x, RequestLength: 0x%02x\n", indent+2, "", 
		aggr->Result.s.Error, aggr->Result.s.RequestLength);
	PrintFunc(dest, "%*sModifier: 0x%02x\n", indent+2, "", aggr->AttributeModifier);
}

void PrintStlClassPortInfo(PrintDest_t *dest, int indent, const STL_CLASS_PORT_INFO *pClassPortInfo, uint8 MgmtClass)
{
	char tbuf[8];
	char cbuf[80];
	char c2buf[80];
	PrintFunc(dest, "%*sBaseVersion: %d ClassVersion: %d\n",
				indent, "", pClassPortInfo->BaseVersion,
				pClassPortInfo->ClassVersion);
	FormatTimeoutMult(tbuf, pClassPortInfo->u1.s.RespTimeValue);
	switch (MgmtClass) {
	case MCLASS_SUBN_ADM: /* SA */
		StlSaClassPortInfoCapMask(cbuf, pClassPortInfo->CapMask);
		StlSaClassPortInfoCapMask2(c2buf, pClassPortInfo->u1.s.CapMask2);
		break;
	case MCLASS_PERF: /* PM */
		StlPmClassPortInfoCapMask(cbuf, pClassPortInfo->CapMask);
		StlPmClassPortInfoCapMask2(c2buf, pClassPortInfo->u1.s.CapMask2);
		break;
	case MCLASS_VFI_PM: /* PA */
		StlPaClassPortInfoCapMask(cbuf, pClassPortInfo->CapMask);
		StlPaClassPortInfoCapMask2(c2buf, pClassPortInfo->u1.s.CapMask2);
		break;
	default:
		StlCommonClassPortInfoCapMask(cbuf, pClassPortInfo->CapMask);
		StlCommonClassPortInfoCapMask2(c2buf, pClassPortInfo->u1.s.CapMask2);
		break;
	}
	PrintFunc(dest, "%*sRespTime: %s Capability: 0x%04x: %s\n",
				indent, "", tbuf,
				pClassPortInfo->CapMask, cbuf);
	PrintFunc(dest, "%*sCapability2: 0x%07x: %s\n",
				indent, "",
				pClassPortInfo->u1.s.CapMask2, c2buf);

	PrintFunc(dest, "%*sRedirect: LID: 0x%.*x GID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",(pClassPortInfo->RedirectLID <= IB_MAX_UCAST_LID ? 4 : 8),
				pClassPortInfo->RedirectLID,
				pClassPortInfo->RedirectGID.AsReg64s.H,
				pClassPortInfo->RedirectGID.AsReg64s.L);
	PrintFunc(dest, "%*s          QP: 0x%06x QKey: 0x%08x PKey: 0x%04x SL: %2u\n",
				indent, "",
				pClassPortInfo->u3.s.RedirectQP,
				pClassPortInfo->Redirect_Q_Key,
				pClassPortInfo->Redirect_P_Key,
				pClassPortInfo->u3.s.RedirectSL);
	PrintFunc(dest, "%*s          FlowLabel: 0x%05x TClass: 0x%02x\n",
				indent, "",
				pClassPortInfo->u2.s.RedirectFlowLabel,
				pClassPortInfo->u2.s.RedirectTClass);

	PrintFunc(dest, "%*sTrap: LID: 0x%.*x GID: 0x%016"PRIx64":0x%016"PRIx64"\n",
				indent, "",(pClassPortInfo->TrapLID <= IB_MAX_UCAST_LID ? 4:8),
				pClassPortInfo->TrapLID,
				pClassPortInfo->TrapGID.AsReg64s.H,
				pClassPortInfo->TrapGID.AsReg64s.L);
	PrintFunc(dest, "%*s      QP: 0x%06x QKey: 0x%08x PKey: 0x%04x SL: %2u\n",
				indent, "",
				pClassPortInfo->u5.s.TrapQP,
				pClassPortInfo->Trap_Q_Key,
				pClassPortInfo->Trap_P_Key,
				pClassPortInfo->u6.s.TrapSL);
	PrintFunc(dest, "%*s      FlowLabel: 0x%05x HopLimit: 0x%02x TClass: 0x%02x\n",
				indent, "",
				pClassPortInfo->u4.s.TrapFlowLabel,
				pClassPortInfo->u5.s.TrapHopLimit,
				pClassPortInfo->u4.s.TrapTClass);
}

