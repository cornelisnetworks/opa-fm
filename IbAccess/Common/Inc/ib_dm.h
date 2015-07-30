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

#ifndef _IBA_IB_DM_H_
#define _IBA_IB_DM_H_ (1) /* suppress duplicate loading of this file */

#include "iba/public/datatypes.h"				/* Portable datatypes */
#include "iba/public/ibyteswap.h"
#include "iba/ib_generalServices.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * IBTA Device Management Class
 * --------------------------------------------------------------------------
 */

/* Device Management methods */
#define	DEVMGT_GET				MMTHD_GET
#define	DEVMGT_SET				MMTHD_SET
#define	DEVMGT_GET_RESP			MMTHD_GET_RESP
#define	DEVMGT_TRAP				MMTHD_TRAP
#define	DEVMGT_TRAP_REPRESS		MMTHD_TRAP_REPRESS
#define DEVMGT_REPORT			MMTHD_REPORT
#define DEVMGT_REPORT_RESP		MMTHD_REPORT_RESP


/* Device Management Attributes IDs */		 	/* Attribute Modifier values */
#define	DM_ATTRIB_ID_CLASS_PORTINFO		0x0001	/* 0x0000_0000 */
#define	DM_ATTRIB_ID_NOTICE				0x0002	/* 0x0000_0000 - 0xFFFF_FFFF */
#define	DM_ATTRIB_ID_IOUNIT_INFO		0x0010	/* 0x0000_0000 */
#define	DM_ATTRIB_ID_IOCONTROLLER_PROFILE 0x0011/* 0x0000_0001 - 0x0000_00FF */
										/*		(IOC slot#) */
#define	DM_ATTRIB_ID_SERVICE_ENTRIES	0x0012	/* 0x0001_0000 - 0x00FF_FFFF */
#define	DM_ATTRIB_ID_DIAGNOSTIC_TIMEOUT	0x0020	/* 0x0000_0001 - 0xFFFF_FFFF */
										/*		(time unit in millisec) */
#define	DM_ATTRIB_ID_PREPARE_TO_TEST	0x0021	/* 0x0000_0001 - 0xFFFF_FFFF */
#define	DM_ATTRIB_ID_TEST_DEVICE_ONCE	0x0022	/* 0x0000_0001 - 0xFFFF_FFFF */
#define	DM_ATTRIB_ID_TEST_DEVICE_LOOP	0x0023	/* 0x0000_0001 - 0xFFFF_FFFF */
#define	DM_ATTRIB_ID_DIAG_CODE			0x0024	/* 0x0000_0001 - 0xFFFF_FFFF */
/*		Reserved 0x0025 - 0xFEFF */
/*		Vendor Specific	0xFF00-0xFFFF */
#define	DM_ATTRIB_ID_VENDOR_MASK		0xff00	/* Vendor Specific Attrib IDs */

/* DM Class Specific MAD_STATUS (AsReg16) */
#define MAD_STATUS_DM_IOC_NOT_RESPONDING	0x0100
#define MAD_STATUS_DM_IOC_NO_SERVICE_ENTRIES	0x0200
	/* bits 10-14 reserved */
#define MAD_STATUS_DM_IOC_GENERAL_FAILURE	0x8000

/* A series of 4-bit nibbles with each representing an IOC slot in the IOU.
 * Each 4-bit nibble can take the following values.  0x2-0xE are reserved.
 */
#define IOC_NOT_INSTALLED	0x0
#define	IOC_INSTALLED		0x1
/* reserved values for		0x02-0xE */
#define	SLOT_NOT_EXISTED	0xF

/* The ControllerList nibble lists defined in IBA DM spec
 *	The high nibble has lower controller slot #
 *
 *	(start of 1024bits)	 0xXX XX XX XX XX ...
 *	slot #				   01 23 45 67 89 ...
 */

#define	SLOT_INFO(clVal,slot)	(((slot)-1)%2 ? ((clVal) & 0x0F) : \
											(((clVal) & 0xF0) >> 4))
/* This macro gives the nibble of the specified IOC # */
#define IOC_AT_SLOT(pIOUnitInfo,slot)		\
		SLOT_INFO( (pIOUnitInfo)->ControllerList[((slot)-1)/2], slot)

#define IOC_AT_SLOT_IS_PRESENT(pIOUnitInfo, slot) \
		(IOC_AT_SLOT(pIOUnitInfo, slot) == IOC_INSTALLED)


#define SET_IOC_AT_SLOT(pIOUnitInfo, slot, value) \
		((pIOUnitInfo)->ControllerList[((slot)-1)/2] = (((slot)-1)%2 ? \
			(((pIOUnitInfo)->ControllerList[((slot)-1)/2] & 0xF0) \
			 		| ((value) & 0xF)) \
			: (((pIOUnitInfo)->ControllerList[((slot)-1)/2] & 0x0F) \
					| ((value) << 4))))


/* --------------------------------------------------------------------------
 * DM Attribute Specific Data
 * Declared in wire format
 */
#include "iba/public/ipackon.h"

/* There is one IOUnitInfo per CA */
typedef struct __IOUnitInfo {
	uint16		Change_ID;			/* monotonically incremented with rollover */
									/*		by any change to ControllerList */
	uint8		MaxControllers;		/* # of slots in Controller List */

#if	CPU_BE
	uint8		Resv		:6;
	uint8		DiagDeviceId:1;		/* attr modifier in diag attrs is IOC Slot */
	uint8		OptionRom	:1;		/* 1 = present, 0 = absent */

#else			/* CPU_LE */
	uint8		OptionRom	:1;		/* 1 = present, 0 = absent */
	uint8		DiagDeviceId:1;		/* attr modifier in diag attrs is IOC Slot */
	uint8		Resv		:6;
#endif

	uint8		ControllerList[128];
} PACK_SUFFIX IOUnitInfo, *PIOUnitInfo;


/* There can be up to 255 IOCs per IOU.  IOCs are identified by a 1 based
 * slot number (1-255)
 */
#define DM_MAX_CONTROLLERS 255
#define IOC_IDSTRING_SIZE 64
typedef struct __IO_CONTROLLER_PROFILE {
	uint64		IocGUID;				/* USE EUI-64 GUID */

	union {
		uint32	AsUint32;
		struct {

#if	CPU_BE
			uint32		VendorId		:24;/* IO Controller vendor, IEEE */
			uint32		Resv1			:8;
#else	/* CPU_LE */
			uint32		Resv1			:8;
			uint32		VendorId		:24;/* IO Controller vendor, IEEE */
#endif

		} v;
	} ven;

#define	IocProfile_VendorId ven.v.VendorId

	uint32		DeviceId;				/* a number assigned by the vendor */
										/* It can be used by an OS to select */
										/* a device driver. */
	uint16		DeviceVersion;		    /* device version assigned by vendor */
	uint16		Resv2;

	union {
		uint32	AsUint32;
		struct {

#if	CPU_BE
			uint32	SubSystemVendorID	:24;/* ID of the enclosure vendor, per IEEE */
			uint32	Resv				:8;
#else		/*CPU_LE */
			uint32	Resv				:8;
			uint32	SubSystemVendorID	:24;/* ID of the enclosure vendor, per IEEE			 */
#endif

		} s;
	} sub;

#define	IocProfile_SubSystemVendorID	sub.s.SubSystemVendorID

	uint32		SubSystemID;			/* A number identifying the subsystem */
										/* where the controller resides */

	uint16		IOClass;				/* 0x0000-0xFFFE = Reserved pending */
										/* I/O class specification approval */
										/* 0xFFFF = vendor specific */

	uint16		IOSubClass;				/* 0x0000-0xFFFE = Reserved pending */
										/* I/O subclass specification approval */
										/* 0xFFFF = vendor specific */
										/* Must be 0xFFFF if IOClass = 0xFFFF */

	uint16		Protocol;				/* 0x0000-0xFFFE = Reserved pending */
										/* I/O subclass specification approval */
										/* 0xFFFF = vendor specific */
										/* Must be 0xFFFF if IOClass = 0xFFFF */

	uint16		ProtocolVer;			/* Protocol specific */
	
	uint16		Resv5;
	uint16		Resv6;

	uint16		SendMsgDepth;			/* Max depth of the send message */
										/* queue */

	uint8		Resv7;					/* RDMAReadDepth was 16 bits in 1.0a */
	uint8		RDMAReadDepth;			/* Max depth of the per-channel RDMA */
										/* read queue */

	uint32		SendMsgSize;			/* Max size of send messages in bytes */
	
	uint32		RDMASize;				/* Max size of outbound RDMA transfers */
										/* initiated by this IOC (in bytes) */

	union {
		uint8		CntlCapMask;
		struct	{

#if	CPU_BE
			uint8	AF		:1;			/*bit 7:Atomic Operations from IOCs */
			uint8	AT		:1;			/*		Atomic Operations to IOCs */
			uint8	WF		:1;			/*		RDMA Write Requests from IOCs */
			uint8	WT		:1;			/*		RDMA Write Requests to IOCs */
			uint8	RF		:1;			/*		RDMA Read Requests from IOCs */
			uint8	RT		:1;			/*		RDMA Read Requests to IOCs */
			uint8	SF		:1;			/*		Send Messages from IOCs */
			uint8	ST		:1;			/*bit 0:Send Messages to IOCs */

#else		/* CPU_LE */
			uint8	ST		:1;			/*bit 0:Send Messages to IOCs */
			uint8	SF		:1;			/*		Send Messages from IOCs */
			uint8	RT		:1;			/*		RDMA Read Requests to IOCs */
			uint8	RF		:1;			/*		RDMA Read Requests from IOCs */
			uint8	WT		:1;			/*		RDMA Write Requests to IOCs */
			uint8	WF		:1;			/*		RDMA Write Requests from IOCs */
			uint8	AT		:1;			/*		Atomic Operations to IOCs */
			uint8	AF		:1;			/*bit 7:Atomic Operations from IOCs */
#endif

		} ctlrCapMask;
	} ccm;

	uint8	Resv8;
	uint8		ServiceEntries;			/* Number of entries in the ServiceEntries table */
	uint8		Resv4[9];
	uint8		IDString[IOC_IDSTRING_SIZE];/* ASCII text string for identifying the */
										/* controller to operator */
} PACK_SUFFIX IOC_PROFILE, *PIOC_PROFILE;


/* there can be up to 255 service entries per IOC.
 * Services are identified by a 0 based service number 0-254
 * Note the IBTA spec is not clear if the 1st service is 0 or 1.  At present
 * all devices from SilverStorm, Mellanox and Engenio use 0 as the 1st service
 */

/* create attribute modifier for DM_ATTRIB_ID_SERVICE_ENTRIES to select
 * a range of service entries within an IOC (1 based IOC slot number)
 */
#define DM_ATTRIB_MODIFIER_SERVICE_ENTRIES(slot,first,last) \
			(((slot) << 16) | (first) | ((last) << 8))

/* extract parts of attribute modifier for a SERVICE_ENTRIES Attribute */
#define DM_ATTRIB_MODIFIER_SERVICE_ENTRIES_SLOT(modifier) \
			(((modifier) >> 16) & 0xff)
#define DM_ATTRIB_MODIFIER_SERVICE_ENTRIES_FIRST(modifier) \
			((modifier) & 0xff)
#define DM_ATTRIB_MODIFIER_SERVICE_ENTRIES_LAST(modifier) \
			(((modifier) >> 8) & 0xff)

#define IOC_SERVICE_NAME_SIZE 40
typedef struct __IOC_SERVICE {

	uchar		Name[IOC_SERVICE_NAME_SIZE];/* Service name string in text format */
	uint64		Id;						/* An identifier of the associated Service */

} PACK_SUFFIX IOC_SERVICE, *PIOC_SERVICE;

typedef struct __IOC_SERVICE_ENTRIES {

	IOC_SERVICE	ServiceEntry[4];

} PACK_SUFFIX DM_IOC_SERVICE_ENTRIES, *PDM_IOC_SERVICE_ENTRIES;


typedef struct __DIAGNOSTIC_TIMEOUT {
	uint32		MaxDiagTime;			/* Max time to finish the diagnotic operation */
										/* in milli-seconds */
} PACK_SUFFIX DIAGNOSTIC_TIMEOUT, *PDIAGNOSTIC_TIMEOUT;

typedef enum {
		DM_DIAG_CODE_OPERATIONAL = 0x0000
		/* others are vendor specific */
} DM_DIAG_CODE_VALUE;

typedef struct __DIAG_CODE {
	uint16		DiagCode;			/* a DM_DIAG_CODE_VALUE */
	/* additional vendor specific data permitted */
} PACK_SUFFIX DIAG_CODE, *PDIAG_CODE;

/* End of packed data structures */
#include "iba/public/ipackoff.h"


/* -------------------------------------------------------------------------- */
/* Byte swap between cpu and network formats */

static __inline void
BSWAP_DM_IOUNIT_INFO (PIOUnitInfo pIOUnitInfo)
{
#if CPU_LE
	pIOUnitInfo->Change_ID = ntoh16(pIOUnitInfo->Change_ID);
#endif
}

static __inline void
BSWAP_DM_IOC_PROFILE (PIOC_PROFILE pIOC_PROFILE)
{
#if CPU_LE
	pIOC_PROFILE->IocGUID		= ntoh64(pIOC_PROFILE->IocGUID);

	pIOC_PROFILE->ven.AsUint32	= ntoh32(pIOC_PROFILE->ven.AsUint32);
	pIOC_PROFILE->DeviceId		= ntoh32(pIOC_PROFILE->DeviceId);
	
	pIOC_PROFILE->DeviceVersion	= ntoh16(pIOC_PROFILE->DeviceVersion);
	/* 16-bit Resv 2: should be all 0. */
	pIOC_PROFILE->sub.AsUint32	= ntoh32(pIOC_PROFILE->sub.AsUint32);
	
	pIOC_PROFILE->SubSystemID	= ntoh32(pIOC_PROFILE->SubSystemID);
	pIOC_PROFILE->IOClass		= ntoh16(pIOC_PROFILE->IOClass);
	pIOC_PROFILE->IOSubClass	= ntoh16(pIOC_PROFILE->IOSubClass);

	pIOC_PROFILE->Protocol		= ntoh16(pIOC_PROFILE->Protocol);
	pIOC_PROFILE->ProtocolVer	= ntoh16(pIOC_PROFILE->ProtocolVer);
	
	pIOC_PROFILE->SendMsgDepth	= ntoh16(pIOC_PROFILE->SendMsgDepth);
	pIOC_PROFILE->RDMAReadDepth	= ntoh16(pIOC_PROFILE->RDMAReadDepth);
	pIOC_PROFILE->SendMsgSize	= ntoh32(pIOC_PROFILE->SendMsgSize);

	pIOC_PROFILE->RDMASize		= ntoh32(pIOC_PROFILE->RDMASize);
#endif
}

static __inline void
BSWAP_DM_IOC_SERVICE (PIOC_SERVICE pIOC_SERVICE)
{
#if CPU_LE
	pIOC_SERVICE->Id = ntoh64(pIOC_SERVICE->Id);
#endif
}

static __inline void
BSWAP_DM_DIAGNOSTIC_TIMEOUT (PDIAGNOSTIC_TIMEOUT pDIAGNOSTIC_TIMEOUT)
{
#if CPU_LE
	pDIAGNOSTIC_TIMEOUT->MaxDiagTime = 
		ntoh32(pDIAGNOSTIC_TIMEOUT->MaxDiagTime);
#endif
}

static __inline void
BSWAP_DM_DIAG_CODE (PDIAG_CODE pDIAG_CODE)
{
#if CPU_LE
	pDIAG_CODE->DiagCode = ntoh16(pDIAG_CODE->DiagCode);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_DM_H_ */
