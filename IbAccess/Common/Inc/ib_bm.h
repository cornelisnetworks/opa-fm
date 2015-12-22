/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_BM_H_
#define _IBA_IB_BM_H_ (1) /* suppress duplicate loading of this file */

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"
#include "iba/ib_generalServices.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iba/public/ipackon.h"

/* Baseboard Management methods */
#define	BASEMGT_GET				0x01		/*	MMTHD_GET */
#define	BASEMGT_SET				0x02		/*	MMTHD_SET */
#define BASEMGT_SEND            0x03		/*  MMTHD_SEND */
#define BASEMGT_TRAP            0x05		/*  MMTHD_TRAP */
#define BASEMGT_REPORT			0x06		/*  MMTHD_REPORT */
#define BASEMGT_TRAP_REPRESS    0x07		/*  MMTHD_TRAP_REPRESS */
#define	BASEMGT_GET_RESP		0x81		/*	MMTHD_GET_RESP */
#define BASEMGT_REPORT_RESP		0x86		/*  MMTHD_REPORT_RESP */


/* Baseboard Management Attributes IDs 	    Attribute Modifier values */
#define	BM_ATTRIB_ID_CLASS_PORTINFO			0x0001	/* 0x0000_0000 */
#define	BM_ATTRIB_ID_NOTICE					0x0002	/* 0x0000_0000 */
#define	BM_ATTRIB_ID_BKEY_INFO				0x0010	/* 0x0000_0000 */

#define	BM_ATTRIB_ID_WRITE_VPD				0x0020	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_READ_VPD				0x0021	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_RESET_IBML				0x0022	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_SET_MODULE_PM_CONTROL	0x0023	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_GET_MODULE_PM_CONTROL	0x0024	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_SET_UNIT_PM_CONTROL	0x0025	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_GET_UNIT_PM_CONTROL	0x0026	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_SET_IOC_PM_CONTROL		0x0027	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_GET_IOC_PM_CONTROL		0x0028	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_SET_MODULE_STATE		0x0029	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_SET_MODULE_ATTENTION	0x002a	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_GET_MODULE_STATUS		0x002b	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_IB2IBML				0x002c	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_IB2CME					0x002d	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_IB2MME					0x002e	/* 0x0000_0000-0x0000_0001*/
#define	BM_ATTRIB_ID_OEM					0x002f	/* 0x0000_0000-0x0000_0001*/

    /* Baseboard Management Vendor ID */
#define BM_OEM_TRUESCALE_VENDOR_ID_BYTE_0      0x00
#define BM_OEM_TRUESCALE_VENDOR_ID_BYTE_1      0x06
#define BM_OEM_TRUESCALE_VENDOR_ID_BYTE_2      0x6A

    /* Definitions moved to ib_mad.h header file */
/*#define BM_ATTRIB_MOD_REQUEST		        0x0000 */
/*#define BM_ATTRIB_MOD_RESPONSE		    0x0001 */

/*
 * BMA Class Port Info
 */

	/* Class-specific Capability Mask bits */
#define BM_CLASS_PORT_CAPMASK_IS_IBML_SUPPORTED	0x0100
#define BM_CLASS_PORT_CAPMASK_IS_BKEY_NVRAM		0x0200

/*
 * BMA Notices/Traps (Data field for IB_NOTICE, see ib_mad.h)
 */

	/* Trap Numbers */
#define BMA_TRAP_BAD_BKEY		259		/* Any bad B_key */
#define BMA_TRAP_BM_TRAP		260		/* BMA Trap */

/* bad B_Key, <B_Key> from <RequestorLID/RequestorGID/QP> attempted <Method> */
/* with <AttributeID> and <AttributeModifier> */
typedef struct _BMA_TRAP_DATA_BAD_BKEY {
	uint16			RequestorLID;		/* LID which attempted access */
	uint8			Method;
	uint8			Reserved1;
	uint16			AttributeID;
	uint32			AttributeModifier;
	union {
		uint32		AsReg32;
	
		struct {
#if CPU_BE
			uint32		Reserved2:	 8;
			uint32		SenderQP:	24;
#else
			uint32		SenderQP:	24;
			uint32		Reserved2:	 8;
#endif
		} s;
	} u1;
	uint64			B_Key;				/* The 8-byte management key. */
	IB_GID			SenderGID;
} PACK_SUFFIX BMA_TRAP_DATA_BAD_BKEY;

/* BM Trap: <BMTrapDataLength> <BMTrapType> <BMTrapTypeModifier> <BMTrapData> */
	/* TrapType values in BMA_TRAP_DATA_BM_TRAP */
typedef enum _BM_TRAP_TYPE {
	BM_TRAP_TYPE_MME		= 0x00,
	BM_TRAP_TYPE_OEM_MME	= 0x01,
	BM_TRAP_TYPE_CME_RTR	= 0x02,
	BM_TRAP_TYPE_WRE		= 0x03,
	BM_TRAP_TYPE_CME		= 0x04,
	BM_TRAP_TYPE_OEM_CME	= 0x05,
	/* 0x05-0xff reserved */
} BM_TRAP_TYPE;

typedef struct _BMA_TRAP_DATA_BM_TRAP {
	uint8			DataLength;
	uint8			TrapType;
	uint8			TrapTypeModifier[3];
	uint8			TrapData[49];
} PACK_SUFFIX BMA_TRAP_DATA_BM_TRAP;

/*
 * BKeyInfo
 */
typedef struct _BKEY_INFO {
	uint64			B_Key;				/* The 8-byte management key. */
	union {
		uint16		AsReg16;
	
		struct {
#if CPU_BE
			uint16		B_KeyProtectBit:  1;
			uint16		Reserved1:		 15;
#else
			uint16		Reserved1:		 15;
			uint16		B_KeyProtectBit:  1;
#endif
		} s;
	} u1;
	uint16			B_KeyLeasePeriod;	/* in seconds, 0=infinite */
	uint16			B_KeyViolations;	/* read-only, 0xffff if not supported */
} PACK_SUFFIX BKEY_INFO;

/*
 * All other BM operations (all use BASEMGT_SEND Method)
 * Baseboard Management parameter size: Volume 2, Section 13.6.1.1, Table 135
 * The maximum length is 40 bytes in 1.2.1 spec. for the BMParameters field.
 * In order to support IPv6 address for the BM OEM CHASSIS IP request, this
 * field had to be extended; thereby, using some of the Reserve field, which
 * follows the BMParameters field.  The spec. indicates that this Reserve field
 * is used as a filler to produce the required 192 byte MAD Data Field. Content is
 * unspecified.
 */
typedef struct _BM_SEND {
	uint16		Sequence;				/* incremented on Requests */
	uint8		SourceDevice;			/* IBML_DEV_ID for source device */
	uint8		ParamCount;				/* number of bytes in Parameters field, 1-based */
	uint8		Parameters[64];			/* ParamCount command-specific parameters */
	uint8		Reserved[124];          /* Filler to produce the required 192 byte */
} PACK_SUFFIX BM_SEND;

typedef enum _BM_DEV_ID {
	BM_DEV_UNSPECIFIED	= 0x00,
	BM_DEV_MME			= 0xe0,
	BM_DEV_CME			= 0xe8,
	BM_DEV_BM			= 0xfe
} BM_DEV_ID;

typedef enum _BM_OEM_TRUESCALE_METHOD {
	BM_OEM_TRUESCALE_METHOD_GET = 0x0001,
    BM_OEM_TRUESCALE_METHOD_SET = 0x0002
} BM_OEM_TRUESCALE_METHOD;

typedef enum _BM_OEM_TRUESCALE_ATTRIBUTE {
	BM_OEM_TRUESCALE_ATTR_FT_STAT         = 0x0001,
	BM_OEM_TRUESCALE_ATTR_PS_STAT         = 0x0002,
	BM_OEM_TRUESCALE_ATTR_READ_VPD        = 0x0003,
	BM_OEM_TRUESCALE_ATTR_SLOT_STAT       = 0x0004,
    BM_OEM_TRUESCALE_ATTR_CHASSIS_IP      = 0x0005,
	BM_OEM_TRUESCALE_ATTR_EXT_SLOT_STAT   = 0x0006
} BM_OEM_TRUESCALE_ATTRIBUTE;

typedef struct _BM_OEM_TRUESCALE_CMD {
	uint8_t	  vendorId[3];  /* Vendor ID */
	uint8_t	  method;		/* Get/Set method identifier */
	uint16_t  attrId;	    /* Attribute identifier */
} BM_OEM_TRUESCALE_CMD;

/*
 * BM OEM chassis IP address structure
 */
#define BM_OEM_CHASSIS_IP_LEN   46      // INET6_ADDRSTRLEN

typedef	struct _BM_OEM_TRUESCALE_CHASSIS_IP {
	uint8_t chassisIP[BM_OEM_CHASSIS_IP_LEN];
} BM_OEM_TRUESCALE_CHASSIS_IP;



#define BM_REQ_GET_VPD_DEVICE(p)			((p)->Parameters[0])
#define BM_REQ_GET_VPD_NUM_BYTES(p)			(((uint16)(p)->Parameters[1] << 8) | ((uint16)(p)->Parameters[2]) )
#define BM_REQ_GET_VPD_OFFSET(p)			(((uint16)(p)->Parameters[3] << 8) | ((uint16)(p)->Parameters[4]) )
#define BM_REQ_WRITE_VPD_DATA_PTR(p)		(&(p)->Parameters[5])
#define BM_RSP_SET_STATUS(p, v)				((p)->Parameters[0] = (v))
#define BM_RSP_READ_VPD_DATA_PTR(p)			(&(p)->Parameters[1])
#define BM_RSP_MODULE_STATUS_DATA_PTR(p)	(&(p)->Parameters[1])

#define BM_REQ_GET_OEM_VENDOR_ID(p)			(((uint32)(p)->Parameters[0] << 16) | \
											 ((uint32)(p)->Parameters[1] << 8) | ((uint32)(p)->Parameters[2]) )
#define BM_REQ_GET_OEM_NUM_BYTES(p)			((p)->ParamCount - 3)
#define BM_REQ_OEM_DATA_PTR(p)				(&(p)->Parameters[3])
#define BM_RSP_SET_OEM_VENDOR_ID(p, v)		(((p)->Parameters[1] = (((v) >> 16))), \
											 ((p)->Parameters[2] = (((v) >> 8))), ((p)->Parameters[3] = (v)))
#define BM_RSP_SET_OEM_NUM_BYTES(p, n)		((p)->ParamCount = (n) + 4)
#define BM_RSP_OEM_DATA_PTR(p)				(&(p)->Parameters[4])
#define BM_RSP_GET_OEM_CHASSIS_IP(p, ipAddr) (memcpy((void *)ipAddr,(void *)&(p)->Parameters[7],(p)->ParamCount - 4))

#define BM_REQ_GET_OEM_VPD_SLOT_ID(p)		(p[3])
#define BM_REQ_GET_OEM_VPD_NUM_BYTES(p)		(((uint16)p[4] << 8) | ((uint16)p[5]) )
#define BM_REQ_GET_OEM_VPD_OFFSET(p)		(((uint16)p[6] << 8) | ((uint16)p[7]) )

#define BM_REQ_GET_OEM_SLOT_GRP_ID(p)		(p[3])

#define BM_WRITE_VPD_MAX_BYTES			35
#define BM_READ_VPD_MAX_BYTES			39
#define BM_GET_MODULE_STATUS_BYTES		6
#define BM_OEM_REQ_MAX_BYTES			37
#define BM_OEM_RSP_MAX_BYTES			192
#define BM_OEM_READ_VPD_MAX_BYTES		36
#define BM_OEM_SLOTS_STAT_MAX_CHASSIS   54 
#define BM_OEM_SLOTS_STAT_MAX_SLOTS     32

#define BM_STATUS_OK					0x00
#define BM_STATUS_ERROR					0x01
#define BM_STATUS_CME_BUSY				0x02
#define BM_STATUS_MME_BUSY				0x03
#define BM_STATUS_CMD_NOT_SUPPORTED		0x04
#define BM_STATUS_ILLEGAL_PARAM			0x05
#define BM_STATUS_WRITE_PROTECTED		0x40
#define BM_STATUS_NACK					0x41
#define BM_STATUS_BUS_ERROR				0x42
#define BM_STATUS_BUSY					0x43
#define BM_STATUS_INVALID_VPD			0x44
#define BM_STATUS_ILLEGAL_OFFSET		0x45
#define BM_STATUS_ILLEGAL_BYTE_COUNT	0x46

/* End of packed data structures */
#include "iba/public/ipackoff.h"


/* Byte Swap macros - convert between host and network byte order */

static __inline void
BSWAP_BMA_TRAP_DATA_BAD_BKEY (BMA_TRAP_DATA_BAD_BKEY *pDest)
{
#if CPU_LE
	pDest->RequestorLID = ntoh16(pDest->RequestorLID);
	pDest->AttributeID = ntoh16(pDest->AttributeID);
	pDest->AttributeModifier = ntoh32(pDest->AttributeModifier);
	pDest->u1.AsReg32 = ntoh32(pDest->u1.AsReg32);
	pDest->B_Key = ntoh64(pDest->B_Key);
	BSWAP_IB_GID(&pDest->SenderGID);
#endif
}

static __inline void
BSWAP_BMA_TRAP_DATA_BM_TRAP (BMA_TRAP_DATA_BM_TRAP *pDest)
{
	/* all 8-bit data, nothing to swap */
}

static __inline void
BSWAP_BKEY_INFO (BKEY_INFO *pDest)
{
#if CPU_LE
	pDest->B_Key = ntoh64(pDest->B_Key);
	pDest->u1.AsReg16 = ntoh16(pDest->u1.AsReg16);
	pDest->B_KeyLeasePeriod = ntoh16(pDest->B_KeyLeasePeriod);
	pDest->B_KeyViolations = ntoh16(pDest->B_KeyViolations);
#endif
}

static __inline void
BSWAP_BM_SEND (BM_SEND *pDest)
{
#if CPU_LE
	pDest->Sequence = ntoh16(pDest->Sequence);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_BM_H_ */
