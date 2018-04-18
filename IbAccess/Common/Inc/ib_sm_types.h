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

#ifndef __IBA_IB_SM_TYPES_H__
#define __IBA_IB_SM_TYPES_H__

#include "iba/public/datatypes.h"
#include "iba/stl_mad_types.h"
#include "iba/public/ibyteswap.h"

// CPU_LE code below can't get to uint typedef
#if defined(VXWORKS)
typedef unsigned int uint;
#endif

#if defined (__cplusplus)
extern "C" {
#endif

#include "iba/public/ipackon.h"

/*
 * Defines 
 */

#define IB_SM_CLASS_VERSION					1		/* Subnet Management */

/* SMP Attributes */
#define MCLASS_ATTRIB_ID_NODE_DESCRIPTION	0x0010	/* Node description string */
#define MCLASS_ATTRIB_ID_NODE_INFO			0x0011	/* Generic node data */
#define MCLASS_ATTRIB_ID_SWITCH_INFO		0x0012	/* Switch information */
/* 0x0013 reserved (was RouterInfo) */
#define MCLASS_ATTRIB_ID_GUID_INFO			0x0014	/* Assigned GIDs */
#define MCLASS_ATTRIB_ID_PORT_INFO			0x0015	/* Port Information */
#define MCLASS_ATTRIB_ID_PART_TABLE			0x0016	/* Partition Table */

#define MCLASS_ATTRIB_ID_SL_VL_MAPPING_TABLE	0x0017	/* Service level to  */
													/* Virtual Lane mapping info */
#define MCLASS_ATTRIB_ID_VL_ARBITRATION		0x0018	/* List of weights */
#define MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE	0x0019	/* Linear forwarding table */
#define MCLASS_ATTRIB_ID_RANDOM_FWD_TABLE	0x001A	/* Random forwarding  */
													/* database information */
#define MCLASS_ATTRIB_ID_MCAST_FWD_TABLE	0x001B	/* Multicast forwarding */
													/* database information */

#define MCLASS_ATTRIB_ID_SM_INFO			0x0020	/* Subnet Manager  */
													/* Information */
/* 0x0021 reserved - was RoutingInfo */
#define MCLASS_ATTRIB_ID_VENDOR_DIAG		0x0030	/* Vendor specific  */
													/* Diagnostic */
#define MCLASS_ATTRIB_ID_LED_INFO			0x0031	/* Turn on/off LED */

#define MCLASS_ATTRIB_ID_PORT_LFT			0xff19	/* Port Linear forwarding table */
#define MCLASS_ATTRIB_ID_PORT_GROUP			0xff21	/* Vendor Port Group Info */
#define MCLASS_ATTRIB_ID_AR_LIDMASK			0xff22	/* LID Mask for Adaptive Routing */

#define MCLASS_ATTRIB_ID_COLLECTIVE_NOTICE	0xff40	/* Vendor unique collective notice */
#define MCLASS_ATTRIB_ID_CMLIST				0xff41	/* Vendor unique collective multicast list */
#define MCLASS_ATTRIB_ID_CFT				0xff42	/* Vendor unique collective forwarding table */

/* SMP Attribute Modifiers */
#define MCLASS_ATTRIB_MOD_SUPPORTS_EXT_SPEEDS	0x80000000	/* SMSupportsExtendedLinkSpeeds */

typedef uint8 PORT; /* Convenience type. */

/*
 * NodeInfo
 */
typedef struct _NODE_INFO {

	uint8	BaseVersion;		/* Supported MAD Base Version. */
	uint8	ClassVersion;		/* Supported Subnet Management Class  */
								/* (SMP) Version. */
	uint8	NodeType;	
	uint8	NumPorts;			/* Number of link ports on this node. */

	/* note GUIDs on 4 byte boundary */
	uint64	SystemImageGUID;		

	uint64	NodeGUID;			/* GUID of the HCA, TCA, switch,  */
								/* or router itself. */
	uint64	PortGUID;			/* GUID of this end port itself.  */

	uint16	PartitionCap;		/* Number of entries in the Partition Table */
								/* for end ports. */
	uint16	DeviceID;			/* Device ID information as assigned by  */
								/* device manufacturer. */
	uint32	Revision;			/* Device revision, assigned by manufacturer. */

	union {
		uint32	AsReg32;

		struct {
#if CPU_BE
			uint32	LocalPortNum:	8;	/* The link port number this  */
			uint32	VendorID:		24;	/* Device vendor, per IEEE. */
#else
			uint32	VendorID:		24;	/* Device vendor, per IEEE. */
			uint32	LocalPortNum:	8;	/* The link port number this  */
										/* SMP came on in. */
#endif
		} s;
	} u1;

} PACK_SUFFIX NODE_INFO;


/*
 * NodeDescription
 */

#define NODE_DESCRIPTION_ARRAY_SIZE 64

typedef struct _NODE_DESCRIPTION {
	
	/* Node String is an array of UTF-8 character that */
	/* describes the node in text format */
	/* Note that this string is NOT NULL TERMINATED! */
	uint8	NodeString[NODE_DESCRIPTION_ARRAY_SIZE];

} PACK_SUFFIX NODE_DESCRIPTION;



/*
 * PortInfo
 */

#define	PORT_RESP_TIME_VALUE_MAX	8


/* Link width, indicated as follows:
 * values are additive for Supported and Enabled fields
 */
typedef enum _IB_LINK_WIDTH {
	IB_LINK_WIDTH_NOP	= 0,	/* no change, valid only for LinkWidthEnabled */
	IB_LINK_WIDTH_1X	= 1,
	IB_LINK_WIDTH_4X	= 2,
	IB_LINK_WIDTH_8X	= 4,
	IB_LINK_WIDTH_12X	= 8,
	IB_LINK_WIDTH_ALL_SUPPORTED	= 255 /* valid only for LinkWidthEnabled */
} IB_LINK_WIDTH;


/* Link speed, indicated as follows:
 * values are additive for Supported and Enabled fields
 * Link speed extended values (FDR/EDR) are also included; note however, that
 * extended values appear in PORT_INFO in LinkSpeedExt
 */
typedef enum _IB_LINK_SPEED {
	IB_LINK_SPEED_NOP		=	0,		/* no change, valid only for enabled */
	IB_LINK_SPEED_DEFAULT	=	1,		/* 2.5Gbps (SDR) */	/* depricated */
	IB_LINK_SPEED_2_5G		=	1,		/* 2.5Gbps (SDR) */	/* use this one */
	IB_LINK_SPEED_5G		=	2,		/* 5.0Gbps (DDR) */
	IB_LINK_SPEED_10G		=	4,		/* 10.0Gbps (QDR) */
	IB_LINK_SPEED_RESERVED	=	8,		/* Reserved */
	IB_LINK_SPEED_14G		=	16,		/* 14.0625 Gbps (FDR) */
	IB_LINK_SPEED_25G		=	32,		/* 25.78125 Gbps (EDR) */
	IB_LINK_SPEED_ALL_SUPPORTED	=	63	/* valid only for LinkSpeedEnabled */
} IB_LINK_SPEED;

/*  Port States */
#define	IB_PORT_NOP	PortStateNop		/* 0 - No state change */
#define	IB_PORT_DOWN	PortStateDown		/* 1 - Down  */
						/* includes failed links */
#define	IB_PORT_INIT	PortStateInit		/* 2 - Initialize */
#define	IB_PORT_ARMED	PortStateArmed		/* 3 - Armed */
#define	IB_PORT_ACTIVE	PortStateActive		/* 4 - Active */
#define	IB_PORT_STATE_MAX	PortStateMax	/* Maximum Valid Value */
						/* reserved 5-15 */

typedef enum _IB_PORT_PHYS_STATE {
	IB_PORT_PHYS_NOP		= 0,				/* 0 - No State change */
	IB_PORT_PHYS_SLEEP		= 1,				/* 1 - sleep */
	IB_PORT_PHYS_POLLING	= 2,				/* 2 - polling */
	IB_PORT_PHYS_DISABLED	= 3,				/* 3 - disabled */
	IB_PORT_PHYS_TRAINING	= 4,				/* 4 - port config training */
	IB_PORT_PHYS_LINKUP		= 5,				/* 5 - linkup */
	IB_PORT_PHYS_LINK_ERROR_RECOVERY = 6,		/* 6 - Link error recovery */
	IB_PORT_PHYS_MAX	 	= 6,				/* Maximum Valid Value */
	IB_PORT_PHYS_RESERVED	= 7					/* 7 - 15 - reserved, ignored */
} IB_PORT_PHYS_STATE;

/* IB_MTU is defined in ib_types.h */

#define IB_LIFETIME_MAX 19	/* max for HOQLife and LifeTimeValue, >19 -> Infinite */

/* Values for M_KeyProtectBits */
typedef enum _IB_MKEY_PROTECT {
	IB_MKEY_PROTECT_RO = 0,	/* Get allows any MKey, Set protected */
	IB_MKEY_PROTECT_HIDE = 1,	/* Get allows any MKey, Resp Hides, Set protected */
	IB_MKEY_PROTECT_SECURE = 2,	/* Get/Set Protected */
	IB_MKEY_PROTECT_SECURE2 = 3	/* Get/Set Protected */
} IB_MKEY_PROTECT;

/* Capability Mask A bit set to 1 for affirmation of supported capability
 * by a given port
 */
typedef union _CapabilityMask {
	uint32  AsReg32;
	struct { IB_BITFIELD31( uint32,
		CmReserved6:						1,  	/* shall be zero */
		IsMulticastFDBTopSupported: 		1,
		IsMulticastPKeyTrapSuppressionSupported:1,
		CmReserved5:						2,  	/* shall be zero */
		IsOtherLocalChangeNoticeSupported:  1,
		IsClientReregistrationSupported:	1,
		IsLinkRoundTripLatencySupported:	1,
		IsBootManagementSupported:  		1,
		IsCapabilityMaskNoticeSupported:	1,
		IsDRNoticeSupported:				1,
		IsVendorClassSupported: 			1,
		IsDeviceManagementSupported:		1,
		IsReinitSupported:  				1,
		IsSNMPTunnelingSupported:   		1,
		IsConnectionManagementSupported:	1,
		IsCapabilityMask2Supported: 		1,
		IsExtendedSpeedsSupported:  		1,
		IsCableInfoSupported:				1,
		IsPKeySwitchExternalPortTrapSupported:1,
		IsSystemImageGuidSupported: 		1,
		IsSMDisabled:   					1,
		IsLEDInfoSupported: 				1,
		IsPKeyNVRAM:						1,  	/* P_Key in NVRAM */
		IsMKeyNVRAM:						1,  	/* M_Key in NVRAM */
		IsSLMappingSupported:   			1,
		IsAutomaticMigrationSupported:  	1,
		CmReserved2:						1,
		IsTrapSupported:					1,
		IsNoticeSupported:  				1,
		IsSM:   							1,
		CmReserved1:						1 )
	} s; 
} IB_CAPABILITY_MASK;

typedef struct _PORT_INFO {

	uint64	M_Key;				/* The 8-byte management key. */
	uint64	SubnetPrefix;		/* Subnet prefix for this port.  */
								/* Set to default value of XXXXX if no */
								/* other subnet interaction. */
								/* GidPrefix ??? */
	uint16	LID;				/* The base LID of this port. */
	uint16	MasterSMLID;		/* The base LID of the master SM that is  */
								/* managing this port. */
	IB_CAPABILITY_MASK CapabilityMask;	/* port capabilities */
	uint16	DiagCode;			/* Diagnostic code, Refer Node Diagnostics */
	uint16	M_KeyLeasePeriod;	
	
	uint8	LocalPortNum;		/* The link port number this SMP came on in. */

	struct _LinkWidth {
		uint8	Enabled;		/* Enabled link width */
		uint8	Supported;		/* Supported link width */
		uint8	Active;			/* Currently active link width */
	} LinkWidth;


	struct {
#if CPU_BE      
		uint8	SpeedSupported:			4;	/* Supported link speed */
		uint8	PortState:				4;	/* Port State.  */
#else
		uint8	PortState:	       		4;	/* Port State.  */
		uint8	SpeedSupported:			4;	/* Supported link speed */
#endif
#if CPU_BE      
		uint8	PortPhysicalState:		4;
		uint8	DownDefaultState:		4;
#else
		uint8   DownDefaultState:		4;
		uint8	PortPhysicalState:		4;
#endif
	} Link;
	
	struct {
#if CPU_BE
		uint8	M_KeyProtectBits:	2;	/* see mgmt key usage */
		uint8	Reserved:			3;	/* reserved, shall be zero */
		uint8	LMC:				3;	/* LID mask for multipath support */
#else
		uint8	LMC:				3;	/* LID mask for multipath support */
		uint8	Reserved:			3;	/* reserved, shall be zero */
		uint8	M_KeyProtectBits:	2;	/* see mgmt key usage */
#endif
	} s1;

	struct _LinkSpeed {
#if CPU_BE
		uint8	Active:				4;	/* Currently active link speed */
		uint8	Enabled:			4;	/* Enabled link speed */
#else
		uint8	Enabled:			4;	/* Enabled link speed */
		uint8	Active:				4;	/* Currently active link speed */
#endif
	} LinkSpeed;

	struct {
#if CPU_BE
		uint8	NeighborMTU:		4;
		uint8	MasterSMSL:			4;
#else
		uint8	MasterSMSL:			4;	/* The adminstrative SL of the master  */
										/* SM that is managing this port. */
		uint8	NeighborMTU:		4;	/* MTU of neighbor endnode connected  */
										/* to this port */
#endif
	} s2;
	
	
	struct {
		struct {
#if CPU_BE
			uint8	Cap:			4;	/* Virtual Lanes supported on this port */
			uint8	InitType:		4;	/* IB_PORT_INIT_TYPE */
#else
			uint8	InitType:		4;	/* IB_PORT_INIT_TYPE */
			uint8	Cap:			4;	/* Virtual Lanes supported on this port */
#endif
		} s;

		uint8	HighLimit;			/* Limit of high priority component of  */
										/* VL Arbitration table */
		uint8	ArbitrationHighCap;		
		uint8	ArbitrationLowCap;
	} VL;

	struct {
#if CPU_BE
		uint8	InitTypeReply:	4;	/* IB_PORT_INIT_TYPE */
		uint8	Cap:			4;	/* Maximum MTU supported by this port. */
#else
		uint8	Cap:			4;	/* Maximum MTU supported by this port. */
		uint8	InitTypeReply:	4;	/* IB_PORT_INIT_TYPE */
#endif
	} MTU;

	struct {					/* transmitter Queueing Controls */
#if CPU_BE
		uint8	VLStallCount:	3;
		uint8	HOQLife:		5;
#else
		uint8	HOQLife:		5;	/* Applies to routers & switches only */
		uint8	VLStallCount:	3;	/* Applies to switches only.  */
#endif
	} XmitQ;
	
	struct {
#if CPU_BE
		uint8	OperationalVL:					4;	/* Virtual Lanes  */
		uint8	PartitionEnforcementInbound:	1;
		uint8	PartitionEnforcementOutbound:	1;
		uint8	FilterRawInbound:				1;
		uint8	FilterRawOutbound:				1;
#else
		uint8	FilterRawOutbound:				1;
		uint8	FilterRawInbound:				1;
		uint8	PartitionEnforcementOutbound:	1;
		uint8	PartitionEnforcementInbound:	1;
		uint8	OperationalVL:					4;	/* Virtual Lanes  */
													/* operational on this port */
#endif
	} s3;

	struct _Violations {
		uint16	M_Key;
		uint16	P_Key;
		uint16	Q_Key;
	} Violations;

	uint8	GUIDCap;	/* Number of GID entries supported in the  */
						/* GIDInfo attribute */


	struct {
#if CPU_BE
		uint8	ClientReregister:	1;
		uint8	Reserved:	2;
		uint8	Timeout:	5;
#else
		uint8	Timeout:	5;	/* Timer value used for subnet timeout  */
		uint8	Reserved:	2;
		uint8	ClientReregister:	1;
#endif
	} Subnet;


	struct {
#if CPU_BE
		uint8	Reserved:	3;
		uint8	TimeValue:	5;
#else
		uint8	TimeValue:	5;
		uint8	Reserved:		3;
#endif
			
	} Resp;

	struct	_ERRORS {
#if CPU_BE
		uint8	LocalPhys:	4;
		uint8	Overrun:	4;
#else
		uint8	Overrun:	4;
		uint8	LocalPhys:	4;
#endif
	} Errors;

	uint16	MaxCreditHint;		/* Max Credit Hint */

	struct _LATENCY {
#if CPU_BE
		uint32	Reserved:		8;
		uint32	LinkRoundTrip:	24;	/* LinkRoundTripLatency */
#else
		uint32	LinkRoundTrip:	24;
		uint32	Reserved:		8;
#endif
	} Latency;

	uint16	CapabilityMask2;	/* Capability Mask 2 */

	struct _LinkSpeedExt {
#if CPU_BE
		uint8	Active:			4;	/* Currently active link speed extended */
		uint8	Supported:		4;	/* Supported link speed extended */
#else
		uint8	Supported:		4;	/* Supported link speed extended */
		uint8	Active:			4;	/* Currently active link speed extended */
#endif
#if CPU_BE
		uint8	Reserved:		3;	/* Reserved */
		uint8	Enabled:		5;	/* Enabled link speed extended */
#else
		uint8	Enabled:		5;	/* Enabled link speed extended */
		uint8	Reserved:		3;	/* Reserved */
#endif
	} LinkSpeedExt;

} PACK_SUFFIX PORT_INFO;

#include "iba/public/ipackoff.h"

#if defined (__cplusplus)
};
#endif

#endif /* __IBA_IB_SM_TYPES_H__ */
