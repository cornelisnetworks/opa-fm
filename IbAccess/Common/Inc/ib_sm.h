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

#if defined(CHECK_HEADERS)

#ifndef __IBA_STL_SM_H__
#warning FIX ME!!! Your includes should use the stl_sm.h header and not the ib_sm.h header for STL builds
#endif

#endif

#ifndef __IBA_IB_SM_H__
#define __IBA_IB_SM_H__

#include "iba/public/datatypes.h"
#include "iba/vpi.h"
#include "iba/stl_mad.h"
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

#define MCLASS_ATTRIB_ID_ICS_LED_INFO		0xff31	/* Turn on/off/auto LED */

#define MCLASS_ATTRIB_ID_COLLECTIVE_NOTICE	0xff40	/* Vendor unique collective notice */
#define MCLASS_ATTRIB_ID_CMLIST				0xff41	/* Vendor unique collective multicast list */
#define MCLASS_ATTRIB_ID_CFT				0xff42	/* Vendor unique collective forwarding table */

/* SMP Attribute Modifiers */
#define MCLASS_ATTRIB_MOD_SUPPORTS_EXT_SPEEDS	0x80000000	/* SMSupportsExtendedLinkSpeeds */

/* *************************************************************************
 * SMPs
 */

/* SMP Fields (LID Routed/Directed Route) */
typedef struct _SMP {

	MAD_COMMON	common;
	uint64	M_Key;						/* A 64 bit key,  */
										/* employed for SM authentication */
	union {
		struct _LIDRouted
	{
			uint8	Reserved3[32];		/* For aligning the SMP data field with */
										/* the directed route SMP data field */
			uint8	SMPData[64];		/* 64 byte field of SMP data used to */
										/* contain the methods attribute */
			uint8	Reserved4[128];		/* Reserved. Shall be set to 0 */
		} LIDRouted;

		struct _DirectedRoute
		{
			uint16	DrSLID;				/* Directed route source LID */
			uint16	DrDLID;				/* Directed route destination LID */
			uint8	Reserved2[28];		/* For the purpose of aligning the Data */
										/* field on a 64 byte boundary */

			uint8	SMPData[64];		/* 64-byte field of SMP data used to  */
										/* contain the methods attribute */
			uint8	InitPath[64];		/* 64-byte field containing the initial */
										/* directed path */
			uint8	RetPath[64];		/* 64-byte field containing the  */
										/* returning directed path */
		} DirectedRoute;
	}SmpExt;

} PACK_SUFFIX SMP;

#if 0
#define SMP_SET_VERSION_INFO(smp, basever, mgmtcl, clver) {\
	MAD_SET_VERSION_INFO(smp, basever, mgmtcl, clver); \
}

#define SMP_GET_VERSION_INFO(smp, basever, mgmtcl, clver) {\
	MAD_GET_VERSION_INFO(smp, basever, mgmtcl, clver); \
}
#endif

#define SMP_GET_STATUS(smp, status) {\
	MAD_GET_STATUS(smp, status); \
}

#define SMP_GET_DR_STATUS(smp, status, d) { \
	(*status) = (smp)->common.u.DR.s.Status; \
	if( NULL != d){\
		(*d) = (smp)->common.u.DR.s.D; \
	}\
}

#define SMP_SET_HOP_POINTER(smp, hop_ptr) { \
	(smp)->common.u.DR.HopPointer = hop_ptr; \
}

#define SMP_GET_HOP_POINTER(smp, hop_ptr) { \
	(*hop_ptr) = (smp)->common.u.DR.HopPointer; \
}

#define SMP_SET_HOP_COUNT(smp, hop_count) { \
	(smp)->common.u.DR.HopCount = hop_count; \
}

#define SMP_GET_HOP_COUNT(smp, hop_count) { \
	(*hop_count) = (x)->common.u.DR.HopCount; \
}

#define SMP_SET_TRANSACTION_ID(smp, id) {\
	MAD_SET_TRANSACTION_ID(smp, id); \
}

#define SMP_GET_TRANSACTION_ID(smp, id) {\
	MAD_GET_TRANSACTION_ID(smp, id); \
}

#define SMP_SET_ATTRIB_ID(smp, id) {\
	MAD_SET_ATTRIB_ID(smp, id); \
}

#define SMP_GET_ATTRIB_ID(smp, id) {\
	MAD_GET_ATTRIB_ID(smp, id); \
}
	
#define SMP_SET_ATTRIB_MOD(smp, amod) {\
	MAD_SET_ATTRIB_MOD(smp, amod); \
}

#define SMP_GET_ATTRIB_MOD(smp, amod) {\
	MAD_GET_ATTRIB_MOD(smp, amod); \
}

#define SMP_SET_M_KEY(smp, mkey) {\
	(smp)->M_Key = mkey; \
}
	
#define SMP_GET_M_KEY(smp, mkey) {\
	(mkey) = (smp)->M_Key; \
}

#define SMP_LR_DATA		SmpExt.LIDRouted.SMPData

#define SMP_DR_DLID		SmpExt.DirectedRoute.DrDLID
#define SMP_DR_SLID		SmpExt.DirectedRoute.DrSLID
#define SMP_DR_DATA		SmpExt.DirectedRoute.SMPData
#define SMP_DR_IPath	SmpExt.DirectedRoute.InitPath
#define SMP_DR_RPath	SmpExt.DirectedRoute.RetPath


/* **************************************************************************
 * Attribute specific SMP Packet Formats
 */

/*
 * SMA Notices/Traps (Data field for IB_NOTICE, see ib_mad.h)
 */

	/* Trap Numbers */
#define SMA_TRAP_GID_NOW_IN_SERVICE		64		/* <GIDADDR> is now in service */
#define SMA_TRAP_GID_OUT_OF_SERVICE		65		/* <GIDADDR> is out of service */
#define SMA_TRAP_ADD_MULTICAST_GROUP	66		/* New multicast group with multicast address <GIDADDR> is now created */
#define SMA_TRAP_DEL_MULTICAST_GROUP	67		/* New multicast group with multicast address <GIDADDR> is now deleted */
#define SMA_TRAP_LINK_STATE 			128		/* Switch port link state has changed */
#define SMA_TRAP_LINK_INTEGRITY 		129		/* Any link integrity threshold reached */
#define SMA_TRAP_BUFFER_OVERRUN 		130		/* Any buffer overrun threshold reached */
#define SMA_TRAP_FLOW_WATCHDOG			131		/* Switch flow control update watchdog */
#define SMA_TRAP_CHG_CAPABILITY			144		/* capability mask changed */
#define SMA_TRAP_CHG_PLATGUID			145		/* platform (system) guid changed */
#define SMA_TRAP_BAD_MKEY				256		/* Any bad M_key */
#define SMA_TRAP_BAD_PKEY				257		/* Any bad P_key */
#define SMA_TRAP_BAD_QKEY				258		/* Any bad Q_key */
#define SMA_TRAP_SWITCH_BAD_PKEY		259		/* Switch bad P_key */

/* Link state of at least one port of switch at <ReportingLID> has changed */
typedef struct _SMA_TRAP_DATA_LINK_STATE {
	uint16			ReportingLID;			
} PACK_SUFFIX SMA_TRAP_DATA_LINK_STATE;

/* Local link integrity threshold reached at <ReportingLID><LocalPortNum> */
typedef struct _SMA_TRAP_DATA_LINK_INTEGRITY {
	uint16			Reserved0;
	uint16			ReportingLID;
	uint8			LocalPortNum;		/* port with error */
} PACK_SUFFIX SMA_TRAP_DATA_LINK_INTEGRITY;

/* Excessive Buffer Overrun threshold reached at <ReportingLID><LocalPortNum> */
typedef struct _SMA_TRAP_DATA_BUFFER_OVERRUN {
	uint16			Reserved0;
	uint16			ReportingLID;
	uint8			LocalPortNum;		/* port with error */
} PACK_SUFFIX SMA_TRAP_DATA_BUFFER_OVERRUN;

/* Flow Control Update Watchdog timer expired at <ReportingLID><LocalPortNum> */
typedef struct _SMA_TRAP_DATA_FLOW_WATCHDOG {
	uint16			Reserved0;
	uint16			ReportingLID;
	uint8			LocalPortNum;		/* port with timeout */
} PACK_SUFFIX SMA_TRAP_DATA_FLOW_WATCHDOG;

/* bad M_Key, <M_Key> from <RequestorLID> attempted <Method>
 * with <AttributeID> and < AttributeModifier>
 */
typedef struct _SMA_TRAP_DATA_BAD_MKEY {
	uint16			Reserved0;
	uint16			RequestorLID;		/* LID which attempted access */
	uint16			Reserved1;
	uint8			Method;
	uint8			Reserved2;
	uint16			AttributeID;
	uint32			AttributeModifier;
	uint64			M_Key;				/* The 8-byte management key. */
} PACK_SUFFIX SMA_TRAP_DATA_BAD_MKEY;

/* bad P_Key, <P_Key> from <SenderLID>/<SenderGID>/<SenderQP>
 * to <ReceiverLID>/<ReceiverGID>/<ReceiverQP> on <SL>
 */
typedef struct _SMA_TRAP_DATA_BAD_PKEY {
	uint16			Reserved0;
	uint16			SenderLID;			/* LID1 */
	uint16			ReceiverLID;		/* LID2 */
	uint16			Reserved;
	uint16			P_Key;
	union {
		uint32		AsReg32;
	
		struct {
#if CPU_BE
			uint32		SL:			 4;
			uint32		Reserved2:	 4;
			uint32		SenderQP:	24;
#else
			uint32		SenderQP:	24;
			uint32		Reserved2:	 4;
			uint32		SL:			 4;
#endif
		} s;
	} u1;
	union {
		uint32		AsReg32;
	
		struct {
#if CPU_BE
			uint32		Reserved3:	 8;
			uint32		ReceiverQP:	24;
#else
			uint32		ReceiverQP:	24;
			uint32		Reserved3:	 8;
#endif
		} s;
	} u2;
	IB_GID			SenderGID;
	IB_GID			ReceiverGID;
} PACK_SUFFIX SMA_TRAP_DATA_BAD_PKEY;

/* bad Q_Key, <Q_Key> from <SenderLID>/<SenderGID>/<SenderQP>
 * to <ReceiverLID>/<ReceiverGID>/<ReceiverQP> on <SL>
 */
typedef struct _SMA_TRAP_DATA_BAD_QKEY {
	uint16			Reserved0;
	uint16			SenderLID;
	uint16			ReceiverLID;
	uint32			Q_Key;
	union {
		uint32		AsReg32;
	
		struct {
#if CPU_BE
			uint32		SL:			 4;
			uint32		Reserved2:	 4;
			uint32		SenderQP:	24;
#else
			uint32		SenderQP:	24;
			uint32		Reserved2:	 4;
			uint32		SL:			 4;
#endif
		} s;
	} u1;
	union {
		uint32		AsReg32;
	
		struct {
#if CPU_BE
			uint32		Reserved3:	 8;
			uint32		ReceiverQP:	24;
#else
			uint32		ReceiverQP:	24;
			uint32		Reserved3:	 8;
#endif
		} s;
	} u2;
	IB_GID			SenderGID;
	IB_GID			ReceiverGID;
} PACK_SUFFIX SMA_TRAP_DATA_BAD_QKEY;


/*
 * NodeInfo
 */

#define OUI_TRUESCALE 0x00066a


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



/* VendorDiags
 * TBD
 */


/*
 * SwitchInfo
 */
typedef struct _SWITCH_INFO {
	uint16	LinearFDBCap;		/* Number of entries supported in the  */
								/* Linear Unicast Forwarding Database  */
	uint16	RandomFDBCap;		/* Number of entries supported in the  */
								/* Random Unicast Forwarding Database.  */
	uint16	MulticastFDBCap;	/* Number of entries supported in the  */
								/* Multicast Forwarding Database  */
	uint16	LinearFDBTop;		/* Indicates the top of the linear  */
								/* forwarding table. */
	uint8	DefaultPort;		/* forward to this port all unicast  */
								/* packets from other ports  */
								/* whose DLID does not exist in the random  */
								/* fwd table */
	uint8	DefaultMulticastPrimaryPort;	
	uint8	DefaultMulticastNotPrimaryPort;
	union {
		uint8	AsReg8;
		struct {
#if CPU_BE
			uint8	LifeTimeValue:	      5;
			uint8	PortStateChange:      1;
			uint8	OptimizedSLVL:	      2;
#else
			uint8	OptimizedSLVL:	2;	
			uint8	PortStateChange:1;	/* This bit is set to zero by a  */
										/* management write.  */
			uint8	LifeTimeValue:	5;	/* Sets the time a packet can live in  */
										/* the switch. See section XXX */
#endif
		} s;
	} u1;
	uint16	LIDsPerPort;			/* specifies no. of LID/LMC combinations  */
									/* that may be assigned  */
									/* to a given link port for switches that  */
									/* support the Random Forwarding database  */
									/* model */
	uint16	PartitionEnforcementCap;/* Specifies the number of entries in the  */
									/* partition enforcement table. */
	union {
		uint8	AsReg8;
		struct {
#if CPU_BE
			uint8	InboundEnforcementCapable:	1;
			uint8	OutboundEnforcementCapable:	1;
			uint8	FilterRawPacketInboundCapable:	1;
			uint8	FilterRawPacketOutboundCapable:	1;
			uint8	EnhancedPort0:	1;
			uint8	reserved:			3;
#else
			uint8	reserved:			3;
			uint8	EnhancedPort0:	1;
			uint8	FilterRawPacketOutboundCapable:	1;
			uint8	FilterRawPacketInboundCapable:	1;
			uint8	OutboundEnforcementCapable:		1;
			uint8	InboundEnforcementCapable:		1;
#endif /*CPU_BE */
		} s;
	} u2;

	uint8	Reserved2;

	uint16	MulticastFDBTop;	/* Indicates the top of the multicast  */
								/* forwarding table. */
} PACK_SUFFIX SWITCH_INFO;


/*
 * GuidInfo
 */

/* The GuidInfo Attribute allows the setting of GUIDs to an end port. */
#define GUID_INFO_BLOCK_SIZE 8
#define GUID_INFO_MAX_BLOCK_NUM 31

typedef struct _GUID_INFO {
	/* Gid 0 in block 0 is Read only invariant value of the BASE GUID */
	uint64 Gid[GUID_INFO_BLOCK_SIZE];
} PACK_SUFFIX GUID_INFO;


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


#define SMP_PORT_STATE(x)		(x)->Link.PortState
#define SMP_PORT_LMC(x)			(x)->s1.LMC
#define SMP_PORT_LID(x)			(x)->LID
#define SMP_PORT_SMLID(x)		(x)->MasterSMLID
#define SMP_PORT_SM(x)			(x)->CapabilityMask.s.IsSM
#define SMP_PORT_M_KEY_BITS(x)	(x)->s1.M_KeyProtectBits



/*
 * PartitionTable
 */

#define PARTITION_TABLE_BLOCK_SIZE 32
#define PARTITION_TABLE_MAX_BLOCK_NUM 2047

typedef union _P_KeyBlock {
	uint16	AsInt16;

	struct {
		uint16	P_KeyBase:		15;			/* Base value of the P_Key that  */
											/* the endnode will use to check  */
											/* against incoming packets */
		uint16	MembershipType:	1;			/* 0 = Limited type */
											/* 1 = Full type */
	} p;

} PACK_SUFFIX P_KEY_BLOCK;

typedef struct _PARTITION_TABLE {

	P_KEY_BLOCK	PartitionTableBlock[PARTITION_TABLE_BLOCK_SIZE];	/* List of P_Key Block elements */

} PACK_SUFFIX PARTITION_TABLE;



/*
 * ForwardingTable
 */

typedef uint8 PORT;				/* Port Block element */

typedef struct _LID_Port_Block {
	uint16	LID;				/* base LID */
	struct {
#if CPU_BE
		uint8	Valid:		1;	/* valid bit */
		uint8	LMC:		3;	/* the LMC of this LID */
		uint8	reserved:	4;	/* for alignment */
#else
		uint8	reserved:	4;	/* for alignment */
		uint8	LMC:		3;	/* the LMC of this LID */
		uint8	Valid:		1;	/* valid bit */
#endif
	} s;
	PORT	Port;				
} PACK_SUFFIX LID_PORT_BLOCK;

typedef uint16	PORTMASK;		/* port mask block element */

#define LFT_BLOCK_SIZE 64
#define RFT_BLOCK_SIZE 16
#define RFT_MAX_BLOCK_NUM 3071
#define MFT_BLOCK_SIZE 32
#define MFT_BLOCK_WIDTH 16	// width of portmask in bits
typedef struct _FORWARDING_TABLE {

	union {
	
		struct _Linear {
			PORT			LftBlock[64];
		} Linear;

		struct _Random {
			LID_PORT_BLOCK	RftBlock[16];
		} Random;

		struct _Multicast {
			PORTMASK		MftBlock[32];
		} Multicast;
	} u;

} PACK_SUFFIX FORWARDING_TABLE;



/*
 * SMInfo
 */

typedef enum _SM_STATE {
	SM_INACTIVE = 0,
	SM_DISCOVERING,
	SM_STANDBY,
	SM_MASTER,
	/* 4-255 - Reserved */
} SM_STATE ;

typedef struct _SM_INFO {
	uint64	GUID;				/* This SM's perception of the GUID  */
								/* of the master SM. */
	uint64	SM_Key;				/* Key of this SM. This is shown as 0 unless  */
								/* the requesting SM is proven to be the  */
								/* master, or the requester is otherwise */
								/* authenticated */
	uint32	ActCount;			/* Counter that increments each time the SM  */
								/* issues a SMP or performs other management  */
								/* activities. Used as a 'heartbeat' indicator  */
								/* by standby SMs. */
	struct {
#if CPU_BE
		uint8	Priority:	4;
		uint8	SMState:	4;
#else
		uint8	SMState:	4;	/* Enumerated value indicating this SM's state. */
								/* Enumerated as SM_STATE. */
		uint8	Priority:	4;	/* Administratively assigned priority for this  */
								/* SM. Can be reset by master SM. */
#endif
	} s;
		
} PACK_SUFFIX SM_INFO;

// SM Service Record
#define SM_SERVICE_ID           (0x1100d03c34822222ull)
#define SM_CAPABILITY_NONE		0x00000000
#define SM_CAPABILITY_VSWINFO	0x00000001

/*
 * VendorDiag
 */

typedef enum {
	DIAG_NODE_READY = 0,
	DIAG_SELF_TEST,
	DIAG_INIT,
	DIAG_SOFT_ERROR,
	DIAG_HARD_ERROR,
	DIAG_RESERVED
} IB_DIAG_CODE;

typedef struct _VendorDiag {

	union {
		uint16	NextIndex;		/* Next attribute Modifier to get to Diag Info. */
								/* Set to zero if this s last or only  */
								/* diag data. */
		struct {
#if CPU_BE
			uint16	UniversalDiagCode:		4;	
			uint16	VendorDiagCode:	11;
			uint16	Chain:	1;
#else /* CPU_LE */
			uint16	Chain:	1;
			uint16	VendorDiagCode:	11;
			uint16	UniversalDiagCode:		4;	
#endif
		} s;
	} u;

	uint8	DiagData[62];		/* Vendor specific diag info. Format undefined. */

} PACK_SUFFIX VendorDiag, VENDOR_DIAG;



/* LedInfo */
typedef union _LedInfo {
	uint32	AsReg32;
	struct {
#if CPU_BE
		uint32 LedMask:		1;		/* MSB of this mask is set to 1 for LED on, */
									/* and 0 for LED off. */
		uint32 Reserved:	31;
#else /* CPU_LE */
		uint32 Reserved:	31;
		uint32 LedMask:		1;		/* MSB of this mask is set to 1 for LED on, */
									/* and 0 for LED off. */
#endif
									/* Response packet shall indicate actual  */
									/* LED state.  */
									/* All other bits are set to zero or  */
									/* ignored on recv. */
	} s;

} PACK_SUFFIX LedInfo, LED_INFO;


/* IcsLedInfo
 * On ICS switches, the SMA sets the LED whenever any port is Active until
 * a SubnSet(LedInfo) is received.
 */
typedef union _IcsLedInfo {
	uint32	AsReg32;
	struct {
#if CPU_BE
		uint32 LedMask:		1;		/* MSB of this mask is set to 1 for LED on, */
									/* and 0 for LED off. */
		uint32 LedAuto:		1;		/* MSB of this mask is set to 1 for LED auto, */
									/* and 0 for LED default. */
		uint32 Reserved:	30;
#else /* CPU_LE */
		uint32 Reserved:	30;
		uint32 LedAuto:		1;		/* MSB of this mask is set to 1 for LED auto, */
									/* and 0 for LED default. */
		uint32 LedMask:		1;		/* MSB of this mask is set to 1 for LED on, */
									/* and 0 for LED off. */
#endif
									/* Response packet shall indicate actual  */
									/* LED state.  */
									/* All other bits are set to zero or  */
									/* ignored on recv. */
	} s;

} PACK_SUFFIX IcsLedInfo, ICS_LED_INFO;


/*
 * Define VL Arbitration table.
 */
#define IB_VLARBTABLE_SIZE 32

typedef struct _VLArbBlock{
	struct {
#if CPU_BE
		uint8			Reserved:	4;
		uint8			VL:			4;
#else
		uint8			VL:			4;
		uint8			Reserved:	4;
#endif /*CPU_BE */
	} s;

	uint8		Weight;
} PACK_SUFFIX VLArbBlock;

typedef struct _VLARBTABLE {
	VLArbBlock	ArbTable[IB_VLARBTABLE_SIZE];
} PACK_SUFFIX VLARBTABLE;

/* Parts of VL Arb Table (specified in upper 16 bits of Attribute Modifier) */
typedef enum _IB_VLARBTABLE_PART {
	IB_VLARBTABLE_PART_FIRST = 1,
	IB_VLARBTABLE_PART_LOW_LOW = 1,	/* lower 32 entries of low priority */
	IB_VLARBTABLE_PART_UPPER_LOW = 2,	/* upper 32 entries of low priority */
	IB_VLARBTABLE_PART_LOW_HIGH = 3,	/* lower 32 entries of high priority */
	IB_VLARBTABLE_PART_UPPER_HIGH = 4,	/* upper 32 entries of high priority */
	IB_VLARBTABLE_PART_LAST = 4
} IB_VLARBTABLE_PART;


#define PGTABLE_LIST_COUNT 64

typedef struct _PORT_GROUP_TABLE {
	uint8		PortGroup[PGTABLE_LIST_COUNT];		/* List of PortGroups indexed by port */
} PACK_SUFFIX PORT_GROUP_TABLE;

#define LIDMASK_LEN 64

typedef struct _ADAPTIVE_ROUTING_LIDMASK {
	uint8		lidmask[LIDMASK_LEN];           // lidmask block of 512 lids
} PACK_SUFFIX ADAPTIVE_ROUTING_LIDMASK;

/*
 * SM attributes for TrueScale Collectives feature.  See DN0310.
 */

typedef enum {
    CollReleasing = 1,
    CollStop = 2,
    CollResume = 3,
    CollFailed = 4,
    CollPublish = 5,
} VendorCollectiveNoticeStatus_t;

typedef struct _COLLECTIVE_NOTICE {
    uint8       collectiveStatus;           	    // collective notice status
    uint64      collectiveHash;             	    // collective hash ID
    uint64      nodeParent;                 	    // parent of the node
} PACK_SUFFIX COLLECTIVE_NOTICE;

#define VENDOR_CMLIST_NODE_COUNT       8   			// Nodes per block

typedef struct _COLLECTIVE_MEMBER_LIST {
    uint64      member[VENDOR_CMLIST_NODE_COUNT];   // list of members identified by GUID
} PACK_SUFFIX COLLECTIVE_MEMBER_LIST;

#define VENDOR_CFT_MASK_COUNT          32           // max of 32*8 entries (256 port max)

typedef struct _COLLECTIVE_FORWARDING_TABLE {
    uint8       parentPort;                        // Port ID to parent, zero indicates none
    uint8       children[VENDOR_CFT_MASK_COUNT];   // A bitmap of ports to children
} PACK_SUFFIX COLLECTIVE_FORWARDING_TABLE;


#include "iba/public/ipackoff.h"

/* Is the Smp given from a local requestor (eg. on the same host such as a
 * smp from IbAccess to its own local port or from a local SM)
 * allow HopPointer to be 0 for direct use of GetSetMad and
 * 1 for packets which passed through SMI prior to GetSetMad
 */
static __inline
boolean
SMP_IS_LOCAL(
	IN SMP			*Smp
	 )
{
	return (( MCLASS_SM_DIRECTED_ROUTE == Smp->common.MgmtClass ) &&
				( Smp->common.u.DR.HopPointer == 0
				  || Smp->common.u.DR.HopPointer == 1) &&
				( Smp->common.u.DR.HopCount == 0 ));
}

static __inline
void
SMP_SET_LOCAL(
	IN SMP			*Smp
	 )
{
	Smp->common.MgmtClass = MCLASS_SM_DIRECTED_ROUTE;
	Smp->common.u.DR.HopPointer = 0;
	Smp->common.u.DR.HopCount = 0;
	Smp->SmpExt.DirectedRoute.DrSLID = LID_PERMISSIVE;
	Smp->SmpExt.DirectedRoute.DrDLID = LID_PERMISSIVE;
}

static __inline
void
BSWAP_SMP_HEADER(
	SMP			*Dest
	 )
{
#if CPU_LE
	BSWAP_MAD_HEADER((MAD *)Dest);

	Dest->M_Key = ntoh64(Dest->M_Key);
	Dest->SmpExt.DirectedRoute.DrSLID = \
		ntoh16(Dest->SmpExt.DirectedRoute.DrSLID);
	Dest->SmpExt.DirectedRoute.DrDLID = \
		ntoh16(Dest->SmpExt.DirectedRoute.DrDLID);
#endif
}

static __inline
void
BSWAP_SMA_TRAP_DATA_LINK_STATE(
	SMA_TRAP_DATA_LINK_STATE			*Dest
	 )
{
#if CPU_LE
	Dest->ReportingLID = ntoh16(Dest->ReportingLID);
#endif
}

static __inline
void
BSWAP_SMA_TRAP_DATA_LINK_INTEGRITY(
	SMA_TRAP_DATA_LINK_INTEGRITY			*Dest
	 )
{
#if CPU_LE
	Dest->ReportingLID = ntoh16(Dest->ReportingLID);
#endif
}

static __inline
void
BSWAP_SMA_TRAP_DATA_BUFFER_OVERRUN(
	SMA_TRAP_DATA_BUFFER_OVERRUN			*Dest
	 )
{
#if CPU_LE
	Dest->ReportingLID = ntoh16(Dest->ReportingLID);
#endif
}

static __inline
void
BSWAP_SMA_TRAP_DATA_FLOW_WATCHDOG(
	SMA_TRAP_DATA_FLOW_WATCHDOG			*Dest
	 )
{
#if CPU_LE
	Dest->ReportingLID = ntoh16(Dest->ReportingLID);
#endif
}

static __inline
void
BSWAP_SMA_TRAP_DATA_BAD_MKEY(
	SMA_TRAP_DATA_BAD_MKEY			*Dest
	 )
{
#if CPU_LE
	Dest->RequestorLID = ntoh16(Dest->RequestorLID);
	Dest->AttributeID = ntoh16(Dest->AttributeID);
	Dest->AttributeModifier = ntoh32(Dest->AttributeModifier);
	Dest->M_Key = ntoh64(Dest->M_Key);
#endif
}

static __inline
void
BSWAP_SMA_TRAP_DATA_BAD_PKEY(
	SMA_TRAP_DATA_BAD_PKEY			*Dest
	 )
{
#if CPU_LE
	Dest->SenderLID = ntoh16(Dest->SenderLID);
	Dest->ReceiverLID = ntoh16(Dest->ReceiverLID);
	Dest->P_Key = ntoh16(Dest->P_Key);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	Dest->u2.AsReg32 = ntoh32(Dest->u2.AsReg32);
	BSWAP_IB_GID(&Dest->SenderGID);
	BSWAP_IB_GID(&Dest->ReceiverGID);
#endif
}

static __inline
void
BSWAPCOPY_SMA_TRAP_DATA_BAD_PKEY(
    SMA_TRAP_DATA_BAD_PKEY			*Src,
	SMA_TRAP_DATA_BAD_PKEY			*Dest
	 )
{
    memcpy(Dest, Src, sizeof(SMA_TRAP_DATA_BAD_PKEY));
    (void)BSWAP_SMA_TRAP_DATA_BAD_PKEY(Dest);
}

static __inline
void
BSWAP_SMA_TRAP_DATA_BAD_QKEY(
	SMA_TRAP_DATA_BAD_QKEY			*Dest
	 )
{
#if CPU_LE
	Dest->SenderLID = ntoh16(Dest->SenderLID);
	Dest->ReceiverLID = ntoh16(Dest->ReceiverLID);
	Dest->Q_Key = ntoh32(Dest->Q_Key);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	Dest->u2.AsReg32 = ntoh32(Dest->u2.AsReg32);
	BSWAP_IB_GID(&Dest->SenderGID);
	BSWAP_IB_GID(&Dest->ReceiverGID);
#endif
}

static __inline
void
BSWAP_NODE_DESCRIPTION(
	NODE_DESCRIPTION			*Dest
	)
{
	/* pure text field, nothing to swap */
}


static __inline
void
BSWAP_NODE_INFO(
	NODE_INFO	*Dest
	)
{
#if CPU_LE
	Dest->SystemImageGUID = ntoh64(Dest->SystemImageGUID );
	Dest->NodeGUID = ntoh64(Dest->NodeGUID);
	Dest->PortGUID = ntoh64(Dest->PortGUID);
	Dest->PartitionCap = ntoh16(Dest->PartitionCap);
	Dest->DeviceID = ntoh16(Dest->DeviceID);
	Dest->Revision = ntoh32(Dest->Revision);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
#endif
}


/* The extended flag should only be used when the PORT_INFO structure
 * used contains the full set of fields.  This is generally true for SMA
 * operations, but when operating against SA PortInfoRecords the
 * extra data may not be present
 */
static __inline
void
BSWAP_PORT_INFO(
	PORT_INFO			*Dest, int extended
	)
{
#if CPU_LE
	Dest->M_Key = ntoh64(Dest->M_Key);
	Dest->SubnetPrefix = ntoh64(Dest->SubnetPrefix);
	Dest->LID = ntoh16(Dest->LID);
	Dest->MasterSMLID = ntoh16(Dest->MasterSMLID);
	Dest->CapabilityMask.AsReg32 = ntoh32(Dest->CapabilityMask.AsReg32);
	Dest->DiagCode = ntoh16(Dest->DiagCode);
	Dest->M_KeyLeasePeriod = ntoh16(Dest->M_KeyLeasePeriod);
	Dest->Violations.M_Key = ntoh16(Dest->Violations.M_Key);
	Dest->Violations.P_Key = ntoh16(Dest->Violations.P_Key);
	Dest->Violations.Q_Key = ntoh16(Dest->Violations.Q_Key);
	Dest->MaxCreditHint = ntoh16(Dest->MaxCreditHint);
	Dest->Latency.LinkRoundTrip =
		((Dest->Latency.LinkRoundTrip & 0x0000FF) << 16) |
		(Dest->Latency.LinkRoundTrip & 0x00FF00) |
		((Dest->Latency.LinkRoundTrip & 0xFF0000) >> 16);
	if (extended)
		Dest->CapabilityMask2 = ntoh16(Dest->CapabilityMask2);
#endif
}


static __inline
void
BSWAP_PART_TABLE(
	PARTITION_TABLE		*Dest
	)
{
#if CPU_LE
	uint32			i;


	for (i=0; i<32; i++)
	{
		Dest->PartitionTableBlock[i].AsInt16 = \
			ntoh16(Dest->PartitionTableBlock[i].AsInt16);
	}
#endif
}


static __inline
void
BSWAP_SWITCH_INFO(
	SWITCH_INFO		*Dest
	)
{
#if CPU_LE
	Dest->LinearFDBCap = ntoh16(Dest->LinearFDBCap);
	Dest->RandomFDBCap = ntoh16(Dest->RandomFDBCap);
	Dest->MulticastFDBCap = ntoh16(Dest->MulticastFDBCap);
	Dest->LinearFDBTop = ntoh16(Dest->LinearFDBTop);
	Dest->LIDsPerPort = ntoh16(Dest->LIDsPerPort);
	Dest->PartitionEnforcementCap = ntoh16(Dest->PartitionEnforcementCap);
	Dest->MulticastFDBTop = ntoh16(Dest->MulticastFDBTop);
#endif
}

static __inline
void
BSWAP_GUID_INFO(
	GUID_INFO			*Dest
	)
{
#if CPU_LE
	int i;

	for (i=0; i<GUID_INFO_BLOCK_SIZE; ++i)
	{
		Dest->Gid[i] = ntoh64(Dest->Gid[i]);
	}
#endif
}

static __inline
void
BSWAP_VLARBTABLE(
	VLARBTABLE			*Dest
	)
{
	/* we laid this out in network byte order for intel compilers */
}

static __inline
void
BSWAP_LINEAR_FWD_TABLE(
    FORWARDING_TABLE            *Dest
    )
{
#if CPU_LE
	/* nothing to do, PORT is only 1 byte */
#endif
}

static __inline
void
BSWAP_RANDOM_FWD_TABLE(
    FORWARDING_TABLE            *Dest
    )
{
#if CPU_LE
#if defined(VXWORKS)
	uint32 i;
#else
	uint i;
#endif

	for (i=0; i< 16; ++i)
	{
		Dest->u.Random.RftBlock[i].LID = ntoh16(Dest->u.Random.RftBlock[i].LID);
	}
#endif
}

static __inline
void
BSWAP_MCAST_FWD_TABLE(
    FORWARDING_TABLE		*Dest
    )
{
#if CPU_LE
#if defined(VXWORKS)
	uint32 i;
#else
	uint i;
#endif

	for (i=0; i< 32; ++i)
	{
		Dest->u.Multicast.MftBlock[i] = ntoh16(Dest->u.Multicast.MftBlock[i]);
	}
#endif
}

static __inline
void
BSWAP_SM_INFO(
    SM_INFO            *Dest
    )
{
#if CPU_LE
    Dest->GUID =                 ntoh64(Dest->GUID);
    Dest->SM_Key =               ntoh64(Dest->SM_Key);
    Dest->ActCount =             ntoh32(Dest->ActCount);
#endif
}

static __inline
void
BSWAP_VENDOR_DIAG(
	VENDOR_DIAG			*Dest
	)
{
#if CPU_LE
	Dest->u.NextIndex = ntoh32(Dest->u.NextIndex);
#endif
}

static __inline
void
BSWAP_LED_INFO(
	LED_INFO			*Dest
	)
{
#if CPU_LE
	Dest->AsReg32 = ntoh32(Dest->AsReg32);
#endif
}

static __inline
void
BSWAP_ICS_LED_INFO(
	ICS_LED_INFO		*Dest
	)
{
#if CPU_LE
	Dest->AsReg32 = ntoh32(Dest->AsReg32);
#endif
}

static __inline
void
BSWAP_PORT_GROUP_TABLE(
	PORT_GROUP_TABLE	*Dest
	)
{
}

static __inline
void
BSWAP_ADAPTIVE_ROUTING_LIDMASK(
	ADAPTIVE_ROUTING_LIDMASK	*Dest
	)
{
}

static __inline
void
BSWAP_COLLECTIVE_NOTICE(
	COLLECTIVE_NOTICE 	*Dest
	)
{
#if CPU_LE
	Dest->collectiveHash = ntoh64(Dest->collectiveHash);
	Dest->nodeParent = ntoh64(Dest->nodeParent);
#endif
}
	
static __inline
void
BSWAP_COLLECTIVE_MEMBER_LIST(
	COLLECTIVE_MEMBER_LIST 	*Dest
	)
{
#if CPU_LE
#if defined(VXWORKS)
	uint32 i;
#else
	uint i;
#endif

	for (i=0; i< VENDOR_CMLIST_NODE_COUNT; ++i)
	{
		Dest->member[i] = ntoh64(Dest->member[i]);
	}
#endif
}
	
static __inline
void
BSWAP_COLLECTIVE_FORWARDING_TABLE(
	COLLECTIVE_FORWARDING_TABLE 	*Dest
	)
{
}
	

#if defined (__cplusplus)
};
#endif

#endif /* __IBA_IB_SM_H__ */
