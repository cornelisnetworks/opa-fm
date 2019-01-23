/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_STL_TYPES_H_
#define _IBA_STL_TYPES_H_

#include "iba/ib_types.h"

/* Basic data types */

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16		STL_LID_16;			/* Can replace IB_LID */
typedef uint32		STL_LID_32;			/* Max LID size */
typedef uint32		STL_LID;			/* Max LID size */

#define STL_MAX_SLS			32			/* Max number of SLs */
#define STL_MAX_SCS			32			/* Max number of SCs */
#define STL_MAX_VLS			32			/* Max number of VLs */


#define MAX_STL_PORTS		64
#define MAX_STL2_VLS		9


typedef uint64		STL_PORTMASK;		/* Port mask element */

#define STL_MAX_PORTMASK				256/(sizeof(STL_PORTMASK)*8)	/* Max Ports in select */
#define STL_PORT_SELECTMASK_SIZE		(sizeof(STL_PORTMASK)*STL_MAX_PORTMASK)
#define STL_EXT_GID_INDEX	1		/* Index into GID table where extended LIDs are stored */

/* -------------------------------------------------------------------------- */
/* LID's */

#define STL_LID_UNICAST_BEGIN	0x00000001U
#define STL_LID_UNICAST_END		0xbfffffffU
#define STL_LID_MULTICAST_BEGIN	0xf0000000U
#define STL_LID_MULTICAST_END	0xf0003ffeU
#define STL_LID_MULTICAST_MASK	0x00003fffU
#define STL_LID_MCAST_OFFSET_MASK	STL_LID_MULTICAST_MASK 
#define STL_LID_RESERVED		0x00000000U
#define STL_LID_PERMISSIVE		0xffffffffU
#define PERMISSIVE_LID_24BITS   (STL_LID_PERMISSIVE & 0xffffff)

#define	MCAST32_TO_MCAST16(x) ((uint16_t)((x) & 0xffff) | 0xc000)
#define	MCAST24_TO_MCAST16(x) MCAST32_TO_MCAST16(x)
#define	MCAST20_TO_MCAST16(x) MCAST32_TO_MCAST16(x)

#define	MCAST16_TO_MCAST20(x) ((uint32_t)((x) & ~0xc000) | 0xf0000)
#define	MCAST16_TO_MCAST24(x) ((uint32_t)((x) & ~0xc000) | 0xf00000)
#define	MCAST16_TO_MCAST32(x) ((uint32_t)((x) & ~0xc000) | 0xf0000000)

/*
 * The IS_MCAST20,-24, and -32 macros below assume
 * MultiCollectMask.MulticastMask = 4
 */
#define	IS_MCAST16(x) (((STL_LID)(x) & 0xffffc000) == 0x0000c000)
#define	IS_MCAST20(x) (((STL_LID)(x) & 0xffff0000) == 0x000f0000)
#define	IS_MCAST24(x) (((STL_LID)(x) & 0xfff00000) == 0x00f00000)
#define	IS_MCAST32(x) (((STL_LID)(x) & 0xf0000000) == 0xf0000000)

#define	UCAST32_TO_UCAST16(x) (IB_LID)(x)
#define	UCAST16_TO_UCAST32(x) (STL_LID)(((x) == LID_PERMISSIVE) ? STL_LID_PERMISSIVE:(x))

static inline STL_LID
IB2STL_LID(IB_LID ib_lid) {
	if (IS_MCAST16(ib_lid)) return MCAST16_TO_MCAST32(ib_lid);
	else return UCAST16_TO_UCAST32(ib_lid);
}

static inline IB_LID
STL2IB_LID(STL_LID stl_lid) {
	if (IS_MCAST32(stl_lid)) return MCAST32_TO_MCAST16(stl_lid);
	else return UCAST32_TO_UCAST16(stl_lid);
}

#include "iba/public/ipackon.h"

/* STL IPV6 IP Address (128 bits) */
typedef struct {
	uint8   addr[16];
} PACK_SUFFIX STL_IPV6_IP_ADDR;

/* STL IPV4 IP Address (32 bits) */
typedef struct {
	uint8   addr[4];
} PACK_SUFFIX STL_IPV4_IP_ADDR;

typedef struct { IB_BITFIELD2( uint8,
	Reserved:	3,
	SL:			5 )
} STL_SL;

typedef struct { IB_BITFIELD2( uint8,
	Reserved:	3,
	SC:			5 )
} STL_SC;

typedef struct { IB_BITFIELD2( uint8,
	Reserved:	3,
	VL:			5 )
} STL_VL;

/* STL MTU values continue from IB_MTU */
#define STL_MTU_0			0
#define STL_MTU_8192		6
#define STL_MTU_10240		7
#define STL_MTU_MAX         STL_MTU_10240

/*
 * STL_FIELDUNIONx() macros are used to create bit-packed structures
 * suitable for network-to-host byte conversion. They are similar to
 * the older IB_BITFIELD() macros but automatically create the union
 * and struct wrappers that are usually manually added to the bitfields.
 *
 * The resulting structures are identical in use to the old IB structures.
 *
 * All macros take the form:
 *
 * STL_FIELDUNIONx(name, len, field1, field2, ..., fieldx);
 *
 * Where "name" is the name of the union, len is a bit length, either
 * 8, 16, 32 or 64, and the fields are the bit fields to be created.
 *
 * For example, this macro:
 *
 * STL_FIELDUNION3(q1, 32, QPN:24, Flag:1, Rsvd:7);
 *
 * Will expand into the following union:
 *
 * union {
 *	uint32		AsReg32;
 *	struct {
 *		uint32 QPN:24;
 *		uint32 Flag:1;
 *		uint32 Rsvd:7;
 *	} __attribute__((packed)) s;
 * } __attribute__((packed)) a1;
 *
 */ 
#define STL_UINT(len) uint##len
#define STL_ASREG(len) AsReg##len
#if CPU_BE
    #define STL_FIELDUNION2(name, len,field1,field2)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION3(name, len,field1,field2,field3)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION4(name, len,field1,field2,field3,field4)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION5(name, len,field1,field2,field3,field4,field5) \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION6(name, len,field1,field2,field3,field4,field5,field6)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION7(name, len,field1,field2,field3,field4,field5,field6,field7)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION8(name, len, field1,field2,field3,field4,field5,field6,field7,field8) \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION9(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field9; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION10(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
			STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field9; \
			STL_UINT(len) field10; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION11(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
			STL_UINT(len) field1; \
			STL_UINT(len) field2; \
			STL_UINT(len) field3; \
			STL_UINT(len) field4; \
			STL_UINT(len) field5; \
			STL_UINT(len) field6; \
			STL_UINT(len) field7; \
			STL_UINT(len) field8; \
			STL_UINT(len) field9; \
			STL_UINT(len) field10; \
			STL_UINT(len) field11; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION14(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field14; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION15(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15) \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field15; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION16(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field15; \
        		STL_UINT(len) field16; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION17(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field1; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field15; \
        		STL_UINT(len) field16; \
        		STL_UINT(len) field17; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
#else
    #define STL_FIELDUNION2(name, len,field1,field2)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION3(name, len,field1,field2,field3)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION4(name, len,field1,field2,field3,field4)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION5(name, len,field1,field2,field3,field4,field5) \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION6(name, len,field1,field2,field3,field4,field5,field6)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION7(name, len,field1,field2,field3,field4,field5,field6,field7)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION8(name, len, field1,field2,field3,field4,field5,field6,field7,field8) \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION9(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION10(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
			STL_UINT(len) field10; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION11(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11)  \
			union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
			STL_UINT(len) field11; \
			STL_UINT(len) field10; \
			STL_UINT(len) field9; \
			STL_UINT(len) field8; \
			STL_UINT(len) field7; \
			STL_UINT(len) field6; \
			STL_UINT(len) field5; \
			STL_UINT(len) field4; \
			STL_UINT(len) field3; \
			STL_UINT(len) field2; \
			STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION14(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION15(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15) \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field15; \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION16(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field16; \
        		STL_UINT(len) field15; \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
    #define STL_FIELDUNION17(name, len, field1,field2,field3,field4,field5,field6,field7,field8,field9,field10,field11,field12,field13,field14,field15,field16,field17)  \
		union { \
			STL_UINT(len) STL_ASREG(len); \
			struct { \
        		STL_UINT(len) field17; \
        		STL_UINT(len) field16; \
        		STL_UINT(len) field15; \
        		STL_UINT(len) field14; \
        		STL_UINT(len) field13; \
        		STL_UINT(len) field12; \
        		STL_UINT(len) field11; \
        		STL_UINT(len) field10; \
        		STL_UINT(len) field9; \
        		STL_UINT(len) field8; \
        		STL_UINT(len) field7; \
        		STL_UINT(len) field6; \
        		STL_UINT(len) field5; \
        		STL_UINT(len) field4; \
        		STL_UINT(len) field3; \
        		STL_UINT(len) field2; \
        		STL_UINT(len) field1; \
			} PACK_SUFFIX s; \
		} PACK_SUFFIX name
#endif

#include "iba/public/ipackoff.h"

#ifdef __cplusplus
};
#endif

#endif /* _IBA_STL_TYPES_H_ */
