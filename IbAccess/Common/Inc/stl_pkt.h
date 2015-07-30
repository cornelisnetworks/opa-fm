/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_STL_PKT_H_
#define _IBA_STL_PKT_H_	(1)

#include "iba/ib_pkt.h"

#if defined (__cplusplus)
extern "C" {
#endif

#include "iba/public/ipackon.h"

/*
 * STL packet headers
 */

/*  
 * 8B Header Format 
 */ 
typedef struct _STL_8B_HDR {
	STL_FIELDUNION6(u1, 32,
		LT:1,
		L2:2,
		F:1,
		RC:3,
		SC:5,
		DLID20:20);
	STL_FIELDUNION4(u2, 32,
		B:1,
		L4:4,
		Length:7,
		SLID20:20);
} STL_8B_HDR;

static __inline void BSWAP_STL_8B_HDR( STL_8B_HDR *hdr) {
#ifdef CPU_BE
	hdr->u1.AsReg32 = ntoh32(hdr->u1.AsReg32);
	hdr->u2.AsReg32 = ntoh32(hdr->u2.AsReg32);
#endif
}

/*  
 * 9B Header Format 
 */ 
typedef struct _STL_9B_HDR {
	STL_FIELDUNION5(u1, 16,
		LT:1,
		L2:2,
		Reserved_60_57:4,
		ServiceChannel:5,
		LinkVersion:4);
	STL_FIELDUNION3(u2, 8,
		ServiceLevel:4,
		Reserved_43_42:2,
		LNH:2);
	STL_LID_16 DLID16;
	STL_FIELDUNION2(u3, 16,
		Reserved_23_20:4,
		Length:12);
	STL_LID_16 SLID16;
} STL_9B_HDR;

static __inline void BSWAP_STL_9B_HDR( STL_9B_HDR *hdr) {
#ifdef CPU_LE
	hdr->u1.AsReg16 = ntoh16(hdr->u1.AsReg16);
	hdr->DLID16 = ntoh16(hdr->DLID16);
	hdr->u3.AsReg16 = ntoh16(hdr->u3.AsReg16);
	hdr->SLID16 = ntoh16(hdr->SLID16);
#endif
}

/*  
 * 10B Header Format 
 */ 
typedef struct _STL_10B_HDR {
	STL_FIELDUNION6(u1, 32,
		LT:1,
		L2:2,
		F:1,
		RC:3,
		SC:5,
		DLID20:20);
	STL_FIELDUNION4(u2, 32,
		B:1,
		L4:4,
		Length:7,
		SLID20:20);
	uint8 Entropy;
	STL_FIELDUNION2(u3, 8,
		Pkey:4,
		L4:4);

} STL_10B_HDR;

static __inline void BSWAP_STL_10B_HDR( STL_10B_HDR *hdr) {
#ifdef CPU_BE
	hdr->u1.AsReg32 = ntoh32(hdr->u1.AsReg32);
	hdr->u2.AsReg32 = ntoh32(hdr->u2.AsReg32);
#endif
}

/*  
 * 16B Header Format 
 */ 
typedef struct _STL_16B_HDR {
	STL_FIELDUNION6(u1, 32,
		LT:1,
		L2:2,
		F:1,
		RC:3,
		SC:5,
		DLID_19_0:20);
	STL_FIELDUNION3(u2, 32,
		B:1,
		Length:11,
		SLID_19_0:20);
	uint8 Reserved;
	uint8 Age;
	uint16 Entropy;
	uint16 Pkey;
	STL_FIELDUNION2(u3, 8,
		DLID_23_20:4,
		SLID_23_20:4);
	uint8 L4;
} STL_16B_HDR;

static __inline void BSWAP_STL_16B_HDR( STL_16B_HDR *hdr) {
#ifdef CPU_BE
	hdr->u1.AsReg32 = ntoh32(hdr->u1.AsReg32);
	hdr->u2.AsReg32 = ntoh32(hdr->u2.AsReg32);
	hdr->Entropy = ntoh16(hdr->Entropy);
	hdr->Pkey = ntoh16(hdr->Pkey);
#endif
}



#include "iba/public/ipackoff.h"

#if defined (__cplusplus)
};
#endif

#endif /* _IBA_STL_PKT_H_ */

