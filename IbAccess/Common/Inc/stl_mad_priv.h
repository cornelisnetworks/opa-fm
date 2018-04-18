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

#ifndef __IBA_STL_MAD_PRIV_H__
#define __IBA_STL_MAD_PRIV_H__

#define STL_MAD_BLOCK_SIZE			2048
#include "iba/stl_mad_types.h"
#include "iba/stl_types.h"
#include "iba/ib_types.h"
#include "iba/ib_mad.h"

#if defined (__cplusplus)
extern "C" {
#endif

/*
 * MAD Base Fields
 */
typedef MAD STL_MAD;

/*
 * Convenience functions for converting MAD packets from host to network byte
 * byte order (or vice-versa).
 */

static __inline
void
BSWAP_STL_CLASS_PORT_INFO(
	STL_CLASS_PORT_INFO	*Dest
	)
{
#if CPU_LE
	Dest->CapMask = ntoh16(Dest->CapMask);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	BSWAP_IB_GID(&Dest->RedirectGID);
	Dest->u2.AsReg32 = ntoh32(Dest->u2.AsReg32);
	Dest->RedirectLID = ntoh32(Dest->RedirectLID);
	Dest->u3.AsReg32 = ntoh32(Dest->u3.AsReg32);
	Dest->Redirect_Q_Key = ntoh32(Dest->Redirect_Q_Key);
	BSWAP_IB_GID(&Dest->TrapGID);
	Dest->u4.AsReg32 = ntoh32(Dest->u4.AsReg32);
	Dest->TrapLID = ntoh32(Dest->TrapLID);
	Dest->u5.AsReg32 = ntoh32(Dest->u5.AsReg32);
	Dest->Trap_Q_Key = ntoh32(Dest->Trap_Q_Key);
	Dest->Trap_P_Key = ntoh16(Dest->Trap_P_Key);
	Dest->Redirect_P_Key = ntoh16(Dest->Redirect_P_Key);
#endif
}



static __inline
void BSWAP_STL_NOTICE(STL_NOTICE * notice)
{
#if CPU_LE
	if (notice->Attributes.Generic.u.s.IsGeneric) {
		notice->Attributes.Generic.u.AsReg32 = ntoh32(notice->Attributes.Generic.u.AsReg32);
		notice->Attributes.Generic.TrapNumber = ntoh16(notice->Attributes.Generic.TrapNumber);
	}
	else {
		notice->Attributes.Vendor.u.AsReg32 = ntoh32(notice->Attributes.Vendor.u.AsReg32);
		notice->Attributes.Vendor.DeviceID = ntoh16(notice->Attributes.Vendor.DeviceID);
	}

	notice->Stats.AsReg16 = ntoh16(notice->Stats.AsReg16);
	notice->IssuerLID = ntoh32(notice->IssuerLID);

	BSWAP_IB_GID(&notice->IssuerGID);
#endif
}

static __inline
void BSWAPCOPY_STL_NOTICE(STL_NOTICE * src, STL_NOTICE * dst)
{
	memcpy(dst, src, sizeof(STL_NOTICE));
	(void)BSWAP_STL_NOTICE(dst);
}

static __inline 
void
BSWAP_STL_INFORM_INFO(STL_INFORM_INFO *dst)
{
#if CPU_LE
	BSWAP_IB_GID(&dst->GID);

	dst->LIDRangeBegin = ntoh32(dst->LIDRangeBegin);
	dst->LIDRangeEnd = ntoh32(dst->LIDRangeEnd);
	dst->Type = ntoh16(dst->Type);
	dst->u.Generic.TrapNumber = ntoh16(dst->u.Generic.TrapNumber);
	dst->u.Generic.u1.AsReg32 = ntoh32(dst->u.Generic.u1.AsReg32);
	dst->u.Generic.u2.AsReg32 = ntoh32(dst->u.Generic.u2.AsReg32);
#endif
}

static __inline 
void
BSWAPCOPY_STL_INFORM_INFO(STL_INFORM_INFO *src, STL_INFORM_INFO *dst)
{
#if CPU_LE
	memcpy(dst, src, sizeof(STL_INFORM_INFO));
	BSWAP_STL_INFORM_INFO(dst);
#endif
}

#if defined (__cplusplus)
};
#endif

#endif /* __IBA_STL_MAD_PRIV_H__ */

