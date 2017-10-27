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

#ifndef __IBA_STL_MAD_TYPES_H__
#define __IBA_STL_MAD_TYPES_H__

#include "iba/public/datatypes.h"
#include "iba/stl_types.h"
#include "iba/ib_mad.h"

#if defined (__cplusplus)
extern "C" {
#endif

#define STL_BASE_VERSION 						0x80

/*
 * STL equivalents of IBA_NODE_CHANNEL_ADAPTER and IBA_NODE_SWITCH.
 */
#define STL_NODE_FI 							1
#define STL_NODE_SW 							2

/*
 * The following MADs are generic across all management classes.
 */
#define STL_MCLASS_ATTRIB_ID_CLASS_PORT_INFO 	0x0001
#define STL_MCLASS_ATTRIB_ID_NOTICE 			0x0002
#define STL_MCLASS_ATTRIB_ID_INFORM_INFO		0x0003

/*
 * Common ClassPortInfo Capability Bits
 */
#define STL_CLASS_PORT_CAPMASK_TRAP             0x0001 /* Can generate traps */
#define STL_CLASS_PORT_CAPMASK_NOTICE           0x0002 /* Implements Get/Set Notice */
#define STL_CLASS_PORT_CAPMASK_CM2              0x0004 /* Implements Additional Class Specific Capabilities (CapMask2) */

#include "iba/public/ipackon.h"

/*
 * Class Port Info: 
 * 
 * This attribute is needed only for use in fabrics that need long LIDs or 
 * more than 16 SLs or VLs. The default should be to continue to use the IB 
 * version. While Storm Lake does not support redirect, this attribute 
 * continues to contain redirect fields such that it can be a canonical 
 * representation of both IB and STL devices. The assorted redirect fields 
 * shall be 0 when the response is supplied by an STL device.
 *
 * STL Differences:
 *		RedirectLID is now 32 bits.
 *		TrapLID is now 32 bits.
 *		Redirect_P_Key moved for qword alignment.
 *		Trap_P_Key moved for qword alignment.
 *		RedirectSL and TrapSL moved to handle longer lengths.
 */

typedef struct {
	uint8 		BaseVersion; 			// RO: Must be STL_BASE_VERSION 
	uint8 		ClassVersion;			// RO

	uint16		CapMask;				// RO

	STL_FIELDUNION2(u1, 32, 
				CapMask2:27,			// RO 
				RespTimeValue:5);		// RO

	/* 8 bytes */

	//Redirect attributes.
	IB_GID RedirectGID; 				// RO

	/* 24 bytes */

	STL_FIELDUNION3(u2, 32, 
				RedirectTClass:8, 		// RO
				Reserved:4, 			// Old SL. Must be zero.
				RedirectFlowLabel:20);	// RO
	
	uint32 RedirectLID; 				// Lengthened for STL. P_Key moved.

	/* 32 bytes */

	STL_FIELDUNION3(u3, 32, 
				RedirectSL:5, 			// RO: Moved from old location.
				Reserved2:3, 			// Must be zero.
				RedirectQP:24) ;		// RO

	uint32		Redirect_Q_Key; 		// RO

	/* 40 bytes */

	//Settable attributes to tell Class agent where to send traps.
	IB_GID	TrapGID;					// RW

	/* 56 bytes */

	STL_FIELDUNION3(u4, 32, 
				TrapTClass:8, 			// RW
				Reserved3:4, 			// Must be zero.
				TrapFlowLabel:20);		// RW

	uint32 TrapLID; 					// Lengthened for STL.

	/* 64 bytes */

	STL_FIELDUNION2(u5, 32, 
				TrapHopLimit:8, 		// RW
				TrapQP:24);				// RW

	uint32 Trap_Q_Key;					// RW

	/* 72 bytes */

	uint16 Trap_P_Key;					// Moved for STL qword alignment.
	uint16 Redirect_P_Key; 				// Moved for STL qword alignment.

	STL_FIELDUNION2(u6, 8,
				TrapSL:5,				// RW: Moved to accommodate larger size.
				Reserved4:3);			// Must be zero.
	
	uint8		Reserved5[3];			// Must be zero.

	/* 80 bytes */
	
} STL_CLASS_PORT_INFO;


/*
 * InformInfoRecord
 * 
 * This should only be used for fabrics that need long LIDs. Smaller 
 * fabrics should continue to use the IB version.
 *
 * STL Differences:
 *		LIDRangeBegin is now 32 bits.
 *		LIDRangeEnd is now 32 bits.
 *		Reserved removed after LIDRangeEnd and added after Type
 *		to qword/word align bit fields.
 */
typedef struct {
	IB_GID 		GID;					// RW	
			
	/* 16 bytes */

	uint32		LIDRangeBegin;			// RW
	uint32		LIDRangeEnd;			// RW

	/* 24 bytes */

	uint8		IsGeneric; 				// RW
	uint8		Subscribe; 				// RW
	
	uint16		Type; 					// RW

	uint16		Reserved1;				// Must be zero.

	/* 30 bytes */

	union {
		struct {
			uint16 TrapNumber;			// RW	
			/* 32 bytes */
			STL_FIELDUNION3(u1, 32, 
				QPNumber:24, 			// RW
				Reserved2:3, 			// Must be zero.
				RespTimeValue:5);		// RO
			STL_FIELDUNION2(u2, 32, 
				Reserved3:8, 			// Must be zero.
				ProducerType:24);		// RW
		} Generic;

		struct {
			uint16 DeviceID;			// RW
			/* 32 bytes */
			STL_FIELDUNION3(u1, 32, 
				QPNumber:24, 			// RW
				Reserved4:3, 			// Must be zero.
				RespTimeValue:5);		// RO
			STL_FIELDUNION2(u2, 32, 
				Reserved5:8, 			// Must be zero.
				VendorID:24);			// RW
		} Vendor;
	} u;
			
	/* 40 bytes */
} STL_INFORM_INFO;


/*
 * Notice 
 *
 * All STL fabrics should use the STL Notice structure when communicating with
 * STL devices and applications. When forwarding notices to IB applications,
 * the SM shall translate them into IB format, when IB equivalents exist.
 * 
 * STL Differences:
 *		IssuerLID is now 32 bits.
 *		Moved fields to maintain word alignment.
 *		Data and ClassTrapSpecificData combined into a single field.
 */
typedef struct {
	union {
		/* Generic Notice attributes */
		struct /*_GENERIC*/ {
			STL_FIELDUNION3(u, 32,
				IsGeneric:1, 			// RO
				Type:7, 				// RO
				ProducerType:24);		// RO
			uint16	TrapNumber;			// RO
		} PACK_SUFFIX Generic;

		/* Vendor specific Notice attributes */
		struct /*_VENDOR*/ {
			STL_FIELDUNION3(u, 32,
				IsGeneric:1, 			// RO
				Type:7, 				// RO
				VendorID:24);			// RO
			uint16	DeviceID;			// RO
		} PACK_SUFFIX Vendor;
	} PACK_SUFFIX Attributes;

	STL_FIELDUNION2(Stats, 16, 
				Toggle:1, 				// RW
				Count:15);				// RW

	/* 8 bytes */
	uint32 		IssuerLID; 				// RO: Extended for STL
	uint32		Reserved2;				// Added for qword alignment
	/* 16 bytes */
	IB_GID 		IssuerGID;				// RO
	/* 32 bytes */
	uint8 		Data[64]; 				// RO. 
	/* 96 bytes */
	uint8		ClassData[0];			// RO. Variable length.
} PACK_SUFFIX STL_NOTICE;

#include "iba/public/ipackoff.h"

static __inline void
StlCommonClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		snprintf(buf, 80, "%s%s%s",
			(cmask & STL_CLASS_PORT_CAPMASK_TRAP) ? "Trap " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_NOTICE) ? "Notice " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_CM2) ? "CapMask2 " : "");
	}
}

static __inline void
StlCommonClassPortInfoCapMask2(char buf[80], uint32 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		buf[0] = '\0';
	}
}

#if defined (__cplusplus)
}
#endif
#endif
