/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

#ifndef __IBA_STL_SM_TYPES_H__
#define __IBA_STL_SM_TYPES_H__

#include "iba/stl_types.h"
#include "iba/ib_sm_types.h"

#if defined (__cplusplus)
extern "C" {
#endif

#include "iba/public/ipackon.h"

/*
 * Defines 
 */

#define STL_SM_CLASS_VERSION				0x80 	/* Subnet Management */

/* SMP Attributes */
#define STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION	0x0010  /* Node Description */
#define STL_MCLASS_ATTRIB_ID_NODE_INFO			0x0011  /* Node Information */
#define STL_MCLASS_ATTRIB_ID_SWITCH_INFO	   	0x0012  /* Switch Information */
#define STL_MCLASS_ATTRIB_ID_PORT_INFO 			0x0015  /* Port Information */
#define STL_MCLASS_ATTRIB_ID_PART_TABLE			0x0016  /* Partition Table */
#define STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE	0x0017  /* Service Level to */
															/* Service Channel Mapping Table */
#define STL_MCLASS_ATTRIB_ID_VL_ARBITRATION		0x0018  /* Lists of VL Arbitration Weights */
#define STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE	0x0019  /* Linear Forwarding Table */
// reserved										0x001A	/* (was) Random Forwarding Table */
#define STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE	0x001B  /* Multicast Forwarding Table */
#define STL_MCLASS_ATTRIB_ID_SM_INFO			0x0020  /* Subnet Manager Information */
#define STL_MCLASS_ATTRIB_ID_LED_INFO			0x0031  /* Turn on/off beaconing LED */
#define STL_MCLASS_ATTRIB_ID_CABLE_INFO			0x0032  /* Cable Information */

#define STL_MCLASS_ATTRIB_ID_AGGREGATE			0x0080  /* Aggregate */
#define STL_MCLASS_ATTRIB_ID_SC_SC_MAPPING_TABLE	0x0081  /* Service Channel to */
															/* Service Channel Mapping Table */
#define STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE	0x0082  /* Service Channel to */
															/* Service Level Mapping Table */
#define STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE	0x0083  /* Service Channel to Virtual */
															/* Lane Receive Mapping Table */
#define STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE	0x0084  /* Service Channel to Virtual */
															/* Lane Transmit Mapping Table */
#define STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE	0x0085  /* Service Channel to Virtual */
															/* Lane Credit Return Mapping Table */
#define STL_MCLASS_ATTRIB_ID_PORT_STATE_INFO		0x0087  /* Port State Information */
#define STL_MCLASS_ATTRIB_ID_PORT_GROUP_FWD_TABLE	0x0088  /* Adaptive Routing Port Group Forwarding Table */
#define STL_MCLASS_ATTRIB_ID_PORT_GROUP_TABLE		0x0089  /* Adaptive Routing Port Group Table */
#define STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE	0x008A  /* Buffer Control Table */
#define STL_MCLASS_ATTRIB_ID_CONGESTION_INFO		0x008B  /* Congestion Information */
#define STL_MCLASS_ATTRIB_ID_SWITCH_CONGESTION_LOG	0x008C  /* Congestion Log */
#define STL_MCLASS_ATTRIB_ID_SWITCH_CONGESTION_SETTING	0x008D	/* Switch Congestion Setting */
#define STL_MCLASS_ATTRIB_ID_SWITCH_PORT_CONGESTION_SETTING	0x008E	/* Switch Congestion Setting */
#define STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_LOG		0x008F	/* Congestion Log */
#define STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_SETTING	0x0090	/* HFI Congestion Setting */
#define STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_CONTROL_TABLE	0x0091	/* HFI Congestion Control Table */

#define STL_MCLASS_ATTRIB_ID_SC_SC_MULTI_SET        0x0094  /* Service Channel to Service */

/* SMP Attribute Modifiers */
#define STL_SM_CONF_START_ATTR_MOD 0x00000200  /* PortInfo & PortStateInfo Attr Mod Flag */


/* *************************************************************************
 * SMPs
 */

/* SMP Fields (LID Routed/Directed Route) */

#define STL_MAX_PAYLOAD_SMP_DR	1872	/* Max size of DR SMP payload */
#define STL_MAX_PAYLOAD_SMP_LR	2016	/* Max size of LR SMP payload */

#define STL_MIN_SMP_DR_MAD	(STL_MAD_BLOCK_SIZE - STL_MAX_PAYLOAD_SMP_DR)
#define STL_MIN_SMP_LR_MAD	(STL_MAD_BLOCK_SIZE - STL_MAX_PAYLOAD_SMP_LR)

typedef struct {
	MAD_COMMON  common;
	uint64  M_Key;  					/* A 64 bit key, */
										/* employed for SM authentication */
	union {
		struct _STL_LIDRouted {
			uint8   SMPData[STL_MAX_PAYLOAD_SMP_LR];	/* Up to 'MAX' byte field of SMP */
														/* data used to contain the */
														/* method's attribute */
		} LIDRouted;

		struct _STL_DirectedRoute {
			STL_LID DrSLID; 			/* Directed route source LID */
			STL_LID DrDLID; 			/* Directed route destination LID */
			uint8   InitPath[64];   	/* 64-byte field containing the initial */
										/* directed path */
			uint8   RetPath[64];		/* 64-byte field containing the */
										/* returning directed path */
			uint8   Reserved2[8];		/* For the purpose of aligning the Data */
										/* field on a 16-byte boundary */

			uint8   SMPData[STL_MAX_PAYLOAD_SMP_DR];	/* Up to 'MAX' byte field of SMP */
														/* data used to contain the */
														/* method's attribute */
		} DirectedRoute;
	}SmpExt;

} PACK_SUFFIX STL_SMP;


/* **************************************************************************
 * Attribute specific SMP Packet Formats
 *
 * In the attributes which follow member fields are marked (where appropriate)
 * with usage information as: 'AccessType/NodeType/IB-Only'.  AccessType
 * refers to how members can be accessed; NodeType refers to which node
 * types the member applies; IB-Only refers to members which have been
 * deprecated for STL use but retained to report information about IB nodes.
 *
 * AccessType: Read Write ('RW'), Read Only ('RO')
 *
 * NodeType: HFI ('H'), External Switch Port ('S'), Base Switch Port 0 ('P'),
 * Enhanced Switch Port 0 ('E').  NodeType is shown as a 4-character string
 * with each character representing (in positional order) a node type 'HSPE'.
 * If a node type is not applicable for a member, its character position is
 * shown as '-' (ex. 'H-PE').
 *
 * Power-On-Default values are shown as 'POD: xxx'
 * Link-Up-Default values are shown as 'LUD: xxx'
 * 
 * Member fields which require a link bounce for new values to become effective
 * are marked 'LinkBounce'
 */

/*
 * SMA Notices/Traps (Data field for IB_NOTICE)
 */

/* Trap Numbers */
#define STL_TRAP_GID_NOW_IN_SERVICE		0x40
#define STL_TRAP_GID_OUT_OF_SERVICE		0x41
#define STL_TRAP_ADD_MULTICAST_GROUP	0x42
#define STL_TRAP_DEL_MULTICAST_GROUP	0x43
#define STL_TRAP_LINK_PORT_CHANGE_STATE	0x80
#define STL_TRAP_LINK_INTEGRITY			0x81
#define STL_TRAP_BUFFER_OVERRUN			0x82
#define STL_TRAP_FLOW_WATCHDOG			0x83
#define STL_TRAP_CHANGE_CAPABILITY		0x90
#define STL_TRAP_CHANGE_SYSGUID			0x91
#define STL_TRAP_BAD_M_KEY				0x100
#define STL_TRAP_BAD_P_KEY				0x101
#define STL_TRAP_BAD_Q_KEY				0x102
#define STL_TRAP_SWITCH_BAD_PKEY		0x103
#define STL_SMA_TRAP_LINK_WIDTH			0x800
#define STL_TRAP_COST_MATRIX_CHANGE		0x801

typedef struct {
	IB_GID		Gid;
} PACK_SUFFIX STL_TRAP_GID;

#define STL_TRAP_GID_NOW_IN_SERVICE_DATA STL_TRAP_GID
#define STL_TRAP_GID_OUT_OF_SERVICE_DATA STL_TRAP_GID
#define STL_TRAP_GID_ADD_MULTICAST_GROUP_DATA STL_TRAP_GID
#define STL_TRAP_GID_DEL_MULTICAST_GROUP_DATA STL_TRAP_GID

typedef struct {
	STL_LID		Lid;
} PACK_SUFFIX STL_TRAP_PORT_CHANGE_STATE_DATA;

typedef struct {
	STL_LID		Lid;
	uint8		Port;
} PACK_SUFFIX STL_TRAP_LINK;
#define STL_TRAP_LINK_INTEGRITY_DATA STL_TRAP_LINK
#define STL_TRAP_BUFFER_OVERRUN_DATA STL_TRAP_LINK
#define STL_TRAP_FLOW_WATCHDOG_DATA STL_TRAP_LINK

typedef STL_FIELDUNION16(STL_CAPABILITY_MASK, 32,
		CmReserved6:						1,  	/* shall be zero */
		CmReserved24:						2,  	/* shall be zero */
		CmReserved5:						2,  	/* shall be zero */
		CmReserved23:  						4,		/* shall be zero */
		IsCapabilityMaskNoticeSupported:	1,
		CmReserved22:						1,		/* shall be zero */
		IsVendorClassSupported: 			1,
		IsDeviceManagementSupported:		1,
		CmReserved21:   					2,		/* shall be zero */
		IsConnectionManagementSupported:	1,
		CmReserved25: 					   10,		/* shall be zero */
		IsAutomaticMigrationSupported:		1,
		CmReserved2:						1,		/* shall be zero */
		CmReserved20:						2,
		IsSM:   							1,
		CmReserved1:						1 );	/* shall be zero */

/* Capability Mask 3 - a bit set to 1 for affirmation of supported capability
 * by a given port
 */
typedef STL_FIELDUNION15(STL_CAPABILITY_MASK3, 16,
		CmReserved0:					1,
		CmReserved1:					1,
		CmReserved2:					1,
		IsMAXLIDSupported:			1,		/* RO/H--- Does the HFI support the MAX */
											/* LID being configured */
		CmReserved3:			 	1,
		CmReserved4:				1,
		VLSchedulingConfig:			2,		/* RO/H-PE VL Arbitration */
											/* see STL_VL_SCHEDULING_MODE */
											/* Port 0 indicates whole switch */
		IsSnoopSupported: 			1,		/* RO/--PE Packet snoop */
											/* Port 0 indicates whole switch */
		IsAsyncSC2VLSupported:	 	1,		/* RO/H-PE Port 0 indicates whole switch */
		IsAddrRangeConfigSupported:	1,		/* RO/H-PE Can addr range for Multicast */
											/* and Collectives be configured */
											/* Port 0 indicates whole switch */
		IsPassThroughSupported: 	1,		/* RO/--PE Packet pass through */
											/* Port 0 indicates whole switch */
		IsSharedSpaceSupported: 	1,		/* RO/H-PE Shared Space */
											/* Port 0 indicates whole switch */
		IsSharedGroupSpaceSupported:1,		/* RO/H-PE Shared Space */
											/* Port 0 indicates whole switch */
		IsVLMarkerSupported: 		1,		/* RO/H-PE VL Marker */
											/* Port 0 indicates whole switch */
		IsVLrSupported: 			1 );	/* RO/H-PE SC->VL_r table */


typedef enum {
	VL_SCHED_MODE_VLARB			= 0,	/* VL Arbitration Tables */
	VL_SCHED_MODE_AUTOMATIC		= 2,	/* harcoded, not configurable */
	/* reserved 3 */
} STL_VL_SCHEDULING_MODE;

typedef struct {
	STL_LID					Lid;
	STL_CAPABILITY_MASK		CapabilityMask;
	uint16					Reserved;
	STL_CAPABILITY_MASK3	CapabilityMask3;
	STL_FIELDUNION5(u,16,
							Reserved:12,
							LinkWidthDowngradeEnabledChange:1,	
							LinkSpeedEnabledChange:1,
							LinkWidthEnabledChange:1,
							NodeDescriptionChange:1);

} PACK_SUFFIX STL_TRAP_CHANGE_CAPABILITY_DATA;

typedef struct {
	uint64		SystemImageGuid;
	STL_LID		Lid;
} PACK_SUFFIX STL_TRAP_SYSGUID_CHANGE_DATA;

typedef struct {
	STL_LID		Lid;
	STL_LID		DRSLid;
	/*	8 bytes */
	uint8		Method;
	STL_FIELDUNION3(u,8,
				DRNotice:1,
				DRPathTruncated:1,
				DRHopCount:6);
	uint16		AttributeID;
	/*	12 bytes */
	uint32		AttributeModifier;
	/*	16 bytes */
	uint64		MKey;
	/* 	24 bytes */
	uint8		DRReturnPath[30]; // We can make this longer....
	/*  54 bytes */
} PACK_SUFFIX STL_TRAP_BAD_M_KEY_DATA;

typedef struct {
	STL_LID		Lid1;
	STL_LID		Lid2;
	/*	8 bytes */
	uint32		Key;	// pkey or qkey
	STL_FIELDUNION2(u,8,
				SL:5,
				Reserved:3);
	uint8		Reserved[3];
	/*	16 bytes */
	IB_GID		Gid1;
	/*	32 bytes */
	IB_GID		Gid2;
	/*	48 bytes */
	STL_FIELDUNION2(qp1,32,
				Reserved:8,
				qp:24);
	/*	52 bytes */
	STL_FIELDUNION2(qp2,32,
				Reserved:8,
				qp:24);
	/*	56 bytes */
} PACK_SUFFIX STL_TRAP_BAD_KEY_DATA;
	
#define STL_TRAP_BAD_P_KEY_DATA STL_TRAP_BAD_KEY_DATA
#define STL_TRAP_BAD_Q_KEY_DATA STL_TRAP_BAD_KEY_DATA

typedef struct {
	STL_FIELDUNION9(u,16,
				Lid1:1, Lid2:1, PKey:1, SL:1,
				QP1:1, QP2:1, Gid1:1, Gid2:1,
				Reserved:8);
	uint16		PKey;
	/*	4 bytes */
	STL_LID		Lid1;
	STL_LID		Lid2;
	STL_FIELDUNION2(u2,8,
				SL:5,
				Reserved:3);
	uint8		Reserved[3];
	/*	16 bytes */
	IB_GID		Gid1;
	/*	32 bytes */
	IB_GID		Gid2;
	/*	48 bytes */
	STL_FIELDUNION2(qp1,32,
				qp:24,
				Reserved:8);
	/*	52 bytes */
	STL_FIELDUNION2(qp2,32,
				qp:24,
				Reserved:8);
	/*	56 bytes */
} PACK_SUFFIX STL_TRAP_SWITCH_BAD_PKEY_DATA;

/* LinkWidth of at least one port of switch at <ReportingLID> has changed */
typedef struct {
	STL_LID  ReportingLID;   		
} PACK_SUFFIX STL_SMA_TRAP_DATA_LINK_WIDTH;

/*
 * NodeInfo
 *
 * Attribute Modifier as: 0 (not used)
 */
typedef struct {

	uint8	BaseVersion;		/* RO Supported MAD Base Version */
	uint8	ClassVersion;		/* RO Supported Subnet Management Class */
								/* (SMP) Version */
	uint8	NodeType;	
	uint8	NumPorts;			/* RO Number of link ports on this node */

	uint32	Reserved;

	uint64	SystemImageGUID;		

	uint64	NodeGUID;			/* RO GUID of the HFI or switch */

	uint64	PortGUID;			/* RO GUID of this end port itself */

	uint16	PartitionCap;		/* RO Number of entries in the Partition Table */
								/* for end ports */
	uint16	DeviceID;			/* RO Device ID information as assigned by */
								/* device manufacturer */
	uint32	Revision;			/* RO Device revision, assigned by manufacturer */

	STL_FIELDUNION2(u1, 32, 
			LocalPortNum:	8,		/* RO The link port number this */
									/* SMP came on in */
			VendorID:		24);	/* RO Device vendor, per IEEE */

} PACK_SUFFIX STL_NODE_INFO;

/*
 * NodeDescription
 *
 * Attribute Modifier as: 0 (not used)
 */

#define STL_NODE_DESCRIPTION_ARRAY_SIZE 64

typedef struct {

    /* Node String is an array of UTF-8 character that */
    /* describes the node in text format */
    /* Note that this string MUST BE NULL TERMINATED! */
    uint8   NodeString[STL_NODE_DESCRIPTION_ARRAY_SIZE];	/* RO */

} PACK_SUFFIX STL_NODE_DESCRIPTION;


/*
 * SwitchInfo
 *
 * Attribute Modifier as: 0 (not used)
 */

/* STL Routing Modes */
#define STL_ROUTE_NOP			0	/* No change */
#define STL_ROUTE_LINEAR		1	/* Linear routing algorithm */



typedef	union {
	uint16	AsReg16;
	struct { IB_BITFIELD5( uint16,
		Reserved:						12,

		IsExtendedSCSCSupported:		1,	/* RO Extended SCSC */
		IsAddrRangeConfigSupported:		1,	/* Can addr range for Multicast */
											/* and Collectives be configured */
		Reserved2:						1,
		IsAdaptiveRoutingSupported:		1 )
	} s; 
} SWITCH_CAPABILITY_MASK;

typedef union {
	uint16  AsReg16;
	struct	{
		uint16  Reserved;
	} s;
} CAPABILITY_MASK_COLLECTIVES;

typedef struct {
	uint32  LinearFDBCap;	  	/* RO Number of entries supported in the */
								/* Linear Unicast Forwarding Database */
	uint32  PortGroupFDBCap;	/* RO Number of entries supported in the */
								/* Port Group Forwarding Database */
	uint32  MulticastFDBCap;	/* RO Number of entries supported in the */
								/*  Multicast Forwarding Database */
	STL_LID  LinearFDBTop;		/* RW Indicates the maximum DLID programmed */
								/*  in the routing tables */
								/* POD: 0 */
	STL_LID  MulticastFDBTop;	/* RW Indicates the top of the Multicast */
								/*  Forwarding Table */
								/* POD: 0 */
	uint32  CollectiveCap;   	/* RO Number of entries supported in the */
								/*  Collective Table */
								/* Reserved in Gen1 */
	STL_LID  CollectiveTop;		/* RW Indicates the top of the Collective Table */
								/* POD: 0 */
								/* Reserved in Gen1 */
	uint32  Reserved;

	STL_IPV6_IP_ADDR  IPAddrIPV6;	       /* RO IP Address - IPV6 */

	STL_IPV4_IP_ADDR  IPAddrIPV4;	       /* RO IP Address - IPV4 */

	uint32  Reserved26;
	uint32  Reserved27;

	uint32	Reserved28;
	uint8	Reserved21;
	uint8	Reserved22;
	uint8	Reserved23;

	union {
		uint8   AsReg8;
		struct { IB_BITFIELD3( uint8,
			LifeTimeValue:		5,	/* LifeTimeValue */
			PortStateChange:	1,	/* RW This bit is set to zero by a */
									/*  management write */
			Reserved20:		2 )	
		} s;
	} u1;

	uint16	Reserved24;
	uint16  PartitionEnforcementCap;	/* RO Specifies the number of entries in the */
										/*  partition enforcement table */
	uint8	PortGroupCap;			/* RO Specifies the maximum number of */
									/* entries in the port group table */
	uint8	PortGroupTop;			/* RW The current number of entries in */
									/* port group table. */

	struct {					/* RW (see STL_ROUTING_MODE) */
		uint8   Supported;		/* Supported routing mode */
		uint8   Enabled;		/* Enabled routing mode */
	} RoutingMode;

	union {
		uint8   AsReg8;
		struct { IB_BITFIELD6( uint8,
			Reserved20:		1,	
			Reserved21:		1,
			Reserved22:		1,
			Reserved23:		1,
			EnhancedPort0:	1,	
			Reserved:		3 )
		} s;
	} u2;

	struct { IB_BITFIELD3( uint8,	/* Multicast/Collectives masks */
		Reserved:			2,
		CollectiveMask:		3,	/* RW Num of additional upper 1s in */
								/* Collective address */
								/* POD: 1 */
								/* Reserved in Gen1 */
		MulticastMask:		3 )	/* RW Num of upper 1s in Multicast address */
								/* POD: 4 */
	} MultiCollectMask;

	STL_FIELDUNION8(AdaptiveRouting, 16,
		Enable: 			1,	/* RW Enable/Disable AR */
		Pause:				1,	/* RW Pause AR when true */
		Algorithm:			3,	/* RW 0 = Random, 1 = Greedy, */
								/* 2 = Random Greedy. */
		Frequency:			3,	/* RW 0-7. Value expands to 2^F*64ms. */
		LostRoutesOnly:		1,  /* RW. Indicates that AR should only be done */
								/* for failed links. */
		Threshold:			3,  /* CCA-level at which switch uses AR. */
		Reserved2:			1,
		Reserved:			3);

	SWITCH_CAPABILITY_MASK  CapabilityMask;		/* RO */

	CAPABILITY_MASK_COLLECTIVES  CapabilityMaskCollectives;	/* RW */
} PACK_SUFFIX STL_SWITCH_INFO;


/*
 * PortInfo
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 00SA PPPP PPPP
 *                        N:   Number of ports
 *                        S=1: Start of SM configuration
 *                        A=1: All ports starting at P (Set only)
 *                        P:   Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 */

/* STL_PORT_STATE values continue from IB_PORT_STATE */
/* reserved 5-6 */

/* STL_PORT_PHYS_STATE values continue from IB_PORT_PHYS_STATE */
/* reserved 7-8 */
#define STL_PORT_PHYS_OFFLINE		9		/* offline */
/* reserved 10 */
#define STL_PORT_PHYS_TEST			11		/* test */

/* Offline Disabled Reason, indicated as follows: */
#define STL_OFFDIS_REASON_NONE					0	/* Nop/No specified reason */
#define STL_OFFDIS_REASON_DISCONNECTED			1	/* not connected in design*/
#define STL_OFFDIS_REASON_LOCAL_MEDIA_NOT_INSTALLED	2	/* Module not installed
														 * in connector (QSFP,
													 	 * SiPh_x16, etc) */
#define STL_OFFDIS_REASON_NOT_INSTALLED			3	/* internal link not
													 * installed, neighbor FRU
													 * absent */
#define STL_OFFDIS_REASON_CHASSIS_CONFIG		4	/* Chassis mgmt forced
													 * offline due to incompat
													 * or absent neighbor FRU */
/* reserved 5 */
#define STL_OFFDIS_REASON_END_TO_END_NOT_INSTALLED	6	/* local module present
													 * but unable to detect
													 * end to optical link */
/* reserved 7 */
#define STL_OFFDIS_REASON_POWER_POLICY			8	/* enabling port would
													 * exceed power policy */
#define STL_OFFDIS_REASON_LINKSPEED_POLICY		9	/* enabled speed unable to
													 * be met due to persistent
													 * cause */
#define STL_OFFDIS_REASON_LINKWIDTH_POLICY		10	/* enabled width unable to
													 * be met due to persistent
													 * cause */
/* reserved 11 */
#define STL_OFFDIS_REASON_SWITCH_MGMT			12	/* user disabled via switch
													 * mangement interface
													 */
#define STL_OFFDIS_REASON_SMA_DISABLED			13	/* user disabled via SMA
													 * Set to phys port state
													 * disabled */
/* reserved 14 */
#define STL_OFFDIS_REASON_TRANSIENT				15	/* Transient offline as part
													 * of sync with neighbor
													 * phys port state machine*/

/* Link Init Reason, indicated as follows: */
#define STL_LINKINIT_REASON_NOP			 0	/* None on Get/no change on Set */
#define STL_LINKINIT_REASON_LINKUP		 1	/* link just came up */
/* values from 2-7 will not be altered by transistions from Polling to Linkup/Init */
/* these can represent persistent reasons why the SM is ignoring a link */
#define STL_LINKINIT_REASON_FLAPPING	 2	/* FM ignoring flapping port */
/* reserved 3-7 */
/* values from 8-15 will be altered by transistions from Polling to LinkUp/Init */
/* these can represent transient reasons why the SM is ignoring a link */
#define STL_LINKINIT_OUTSIDE_POLICY		 8	/* FM ignoring, width or speed outside FM configured policy */
#define STL_LINKINIT_QUARANTINED		 9	/* FM ignoring, quarantined for security */
#define STL_LINKINIT_INSUFIC_CAPABILITY	 10	/* FM ignoring, link has insufficient capabilities*/
											/* for FM configuration (eg. MTU too small etc) */
/* reserved 11-15 */


/* these correspond to locally initiated link bounce due to PortErrorAction */
/* See Section 9.8.3, “Error Counters”, Table 9-23 */
#define STL_LINKDOWN_REASON_NONE				0	/* No specified reason */
#define STL_LINKDOWN_REASON_RCV_ERROR_0			1
#define STL_LINKDOWN_REASON_BAD_PKT_LEN			2
#define STL_LINKDOWN_REASON_PKT_TOO_LONG		3
#define STL_LINKDOWN_REASON_PKT_TOO_SHORT		4
#define STL_LINKDOWN_REASON_BAD_SLID			5
#define STL_LINKDOWN_REASON_BAD_DLID			6
#define STL_LINKDOWN_REASON_BAD_L2				7
#define STL_LINKDOWN_REASON_BAD_SC				8
#define STL_LINKDOWN_REASON_RCV_ERROR_8			9
#define STL_LINKDOWN_REASON_BAD_MID_TAIL		10
#define STL_LINKDOWN_REASON_RCV_ERROR_10		11
#define STL_LINKDOWN_REASON_PREEMPT_ERROR		12
#define STL_LINKDOWN_REASON_PREEMPT_VL15		13
#define STL_LINKDOWN_REASON_BAD_VL_MARKER		14
#define STL_LINKDOWN_REASON_RCV_ERROR_14		15
#define STL_LINKDOWN_REASON_RCV_ERROR_15		16
#define STL_LINKDOWN_REASON_BAD_HEAD_DIST		17
#define STL_LINKDOWN_REASON_BAD_TAIL_DIST		18
#define STL_LINKDOWN_REASON_BAD_CTRL_DIST		19
#define STL_LINKDOWN_REASON_BAD_CREDIT_ACK		20
#define STL_LINKDOWN_REASON_UNSUPPORTED_VL_MARKER 21
#define STL_LINKDOWN_REASON_BAD_PREEMPT			22
#define STL_LINKDOWN_REASON_BAD_CONTROL_FLIT	23
#define STL_LINKDOWN_REASON_EXCEED_MULTICAST_LIMIT	24
#define STL_LINKDOWN_REASON_RCV_ERROR_24		25
#define STL_LINKDOWN_REASON_RCV_ERROR_25		26
#define STL_LINKDOWN_REASON_RCV_ERROR_26		27
#define STL_LINKDOWN_REASON_RCV_ERROR_27		28
#define STL_LINKDOWN_REASON_RCV_ERROR_28		29
#define STL_LINKDOWN_REASON_RCV_ERROR_29		30
#define STL_LINKDOWN_REASON_RCV_ERROR_30		31
#define STL_LINKDOWN_REASON_EXCESSIVE_BUFFER_OVERRUN 32
/* the next two correspond to locally initiated intentional link down */
#define STL_LINKDOWN_REASON_UNKNOWN				33
			/* code 33 is used for locally initiated link downs which don't */
			/* match any of these reason code */
/* reserved 34 */
#define STL_LINKDOWN_REASON_REBOOT				35 /* reboot or device reset */
#define STL_LINKDOWN_REASON_NEIGHBOR_UNKNOWN	36
			/* This indicates the link down was not locally initiated */
			/* but no LinkGoingDown idle flit was received */
			/* See Section 6.3.11.1.2, "PlannedDownInform Substate" */
/* reserved 37 - 38 */
/*These correspond to locally initiated intentional link down */
#define STL_LINKDOWN_REASON_FM_BOUNCE			39	/* FM initiated bounce */
													/* by transitioning from linkup to Polling */
#define STL_LINKDOWN_REASON_SPEED_POLICY		40 /* link outside speed policy */
#define STL_LINKDOWN_REASON_WIDTH_POLICY		41 /* link downgrade outside */
													/* LinkWidthDowngrade.Enabled policy */
/* reserved 42-47 */
/* these correspond to locally initiated link down via transition to Offline or Disabled */
/* See Section 6.6.2, “Offline/Disabled Reasons”, Table 6-38*/
/* All values in that section are provided for here, although in practice a few */
/* such as 49 (Disconnected) represent links which can never reach LinkUp and hence */
/* could not have a transition to LinkDown */
/* reserved 48 */
#define STL_LINKDOWN_REASON_DISCONNECTED		49
#define STL_LINKDOWN_REASON_LOCAL_MEDIA_NOT_INSTALLED 50
#define STL_LINKDOWN_REASON_NOT_INSTALLED		51
#define STL_LINKDOWN_REASON_CHASSIS_CONFIG		52
/* reserved 53 */
#define STL_LINKDOWN_REASON_END_TO_END_NOT_INSTALLED 54
/* reserved 55 */
#define STL_LINKDOWN_REASON_POWER_POLICY		56
#define STL_LINKDOWN_REASON_LINKSPEED_POLICY	57
#define STL_LINKDOWN_REASON_LINKWIDTH_POLICY	58
/* reserved 59 */
#define STL_LINKDOWN_REASON_SWITCH_MGMT			60
#define STL_LINKDOWN_REASON_SMA_DISABLED		61
/* reserved 62 */
#define STL_LINKDOWN_REASON_TRANSIENT			63
/* reserved 64-255 */


/* STL PORT TYPE values imply cable info availabilty and format */
#define STL_PORT_TYPE_UNKNOWN		0
#define STL_PORT_TYPE_DISCONNECTED		1	/* the port is not currently usable � CableInfo not available */
#define STL_PORT_TYPE_FIXED				2	/* A fixed backplane port in a director class switch � All STL ASICS */
#define STL_PORT_TYPE_VARIABLE			3	/* A backplane port in a blade system � possibly mixed configuration */
#define STL_PORT_TYPE_STANDARD			4	/* implies a SFF-8636 defined format for CableInfo (QSFP) */
#define STL_PORT_TYPE_SI_PHOTONICS		5	/* A silicon photonics module � 
											 * implies TBD defined format for CableInfo as defined by Intel SFO group */
/* 6 - 15 Reserved */

/* STL NEIGHBOR NODE TYPE indicate whether the neighbor is an HFI or 
 * a switch. */
#define STL_NEIGH_NODE_TYPE_HFI		0	/* Gen 1 HFIs are considered "untrusted" */
#define STL_NEIGH_NODE_TYPE_SW		1	/* Gen 1 Switches are considered "trusted" */
/* Values 2 & 3 are reserved in Gen1. */

typedef union {
	uint32  AsReg32;
	struct { IB_BITFIELD8( uint32,	/* Port states */
		Reserved:					9,
		LEDEnabled:					1, 	/* RO/HS-- Set to 1 if the port LED is active. */
		IsSMConfigurationStarted: 	1,  /* RO/HS-E - POD/LUD: 0 */
		NeighborNormal:				1,	/* RO/HS-- */
										/* POD/LUD: 0 */
		OfflineDisabledReason:		4,	/* RO/HS-E Reason for Offline (see STL_OFFDIS_REASON_XXX) */
		Reserved2:					8,	
		PortPhysicalState:			4,	/* RW/HS-E Port Physical State (see STL_PORT_PHYS_XXX) */
		PortState:					4 )	/* RW/HS-E Port State (see STL_PORT_XXX) */
	} s;
} STL_PORT_STATES;

typedef union {
	uint8   AsReg8;
	struct { IB_BITFIELD2( uint8,	/* RW/HS-E Neighbor MTU values per VL */
									/* LUD: 2048 MTU for STL VL15 */
				VL0_to_MTU:		4,
				VL1_to_MTU:		4 )
	} s;
} STL_VL_TO_MTU;

#define	STL_VL0_VL31		6

/* STL Link speed, continued from IB_LINK_SPEED and indicated as follows:
 * values are additive for Supported and Enabled fields
 */

#define STL_LINK_SPEED_NOP		 0		/* LinkSpeed.Enabled: no change */
										/* LinkSpeeed.Active: link is LinkDown*/
#define STL_LINK_SPEED_12_5G	 0x0001	/* 12.5 Gbps */
#define STL_LINK_SPEED_25G		 0x0002	/* 25.78125? Gbps (EDR) */


/* STL Link width, continued from IB_LINK_WIDTH and indicated as follows:
 * values are additive for Supported and Enabled fields
 */


#define		STL_LINK_WIDTH_NOP					 0	/* LinkWidth.Enabled: no changeon set (nop) */
													/* LinkWidth.Active: link is LinkDown*/
													/* LinkWidthDowngrade.Supported: unsupported */
													/* LinkWidthDowngrade.Enable: disabled */
													/* LinkWidthDowngrade.TxActive: link is LinkDown*/
													/* LinkWidthDowngrade.RxActive: link is LinkDown */
#define 	STL_LINK_WIDTH_1X					 0x0001
#define		STL_LINK_WIDTH_2X					 0x0002
#define		STL_LINK_WIDTH_3X					 0x0004
#define 	STL_LINK_WIDTH_4X					 0x0008

/* STL Port link mode, indicated as follows:
 * values are additive for Supported and Enabled fields
 */
#define STL_PORT_LINK_MODE_NOP	0		/* No change */
/* reserved 1 */
/* reserved 2 */
#define STL_PORT_LINK_MODE_STL	4		/* Port mode is STL */

/* STL Port link formats, indicated as follows:
 * values are additive for Supported and Enabled fields
 */
#define STL_PORT_PACKET_FORMAT_NOP	0		/* No change */
#define STL_PORT_PACKET_FORMAT_8B	1		/* Format 8B */
#define STL_PORT_PACKET_FORMAT_9B	2		/* Format 9B */
#define STL_PORT_PACKET_FORMAT_10B	4		/* Format 10B */
#define STL_PORT_PACKET_FORMAT_16B	8		/* Format 16B */

/* STL Port LTP CRC mode, indicated as follows:
 * values are additive for Supported and Enabled fields
 */
#define STL_PORT_LTP_CRC_MODE_NONE	0	/* No change */
#define STL_PORT_LTP_CRC_MODE_14	1	/* 14-bit LTP CRC mode (optional) */
#define STL_PORT_LTP_CRC_MODE_16	2	/* 16-bit LTP CRC mode */
#define STL_PORT_LTP_CRC_MODE_48	4	/* 48-bit LTP CRC mode (optional) */
#define STL_PORT_LTP_CRC_MODE_12_16_PER_LANE 8	/* 12/16-bit per lane LTP CRC mode */

/* STL Port Flit distance mode, indicated as follows:
 * values are additive for Supported and Enabled fields
 */
#define STL_PORT_FLIT_DISTANCE_MODE_NONE	0	/* No change */
#define STL_PORT_FLIT_DISTANCE_MODE_1		1	/* STL1 mode */
#define STL_PORT_FLIT_DISTANCE_MODE_2		2	/* STL2 mode */

/* STL VL Scheduling mode, indicated as follows:
 */
#define STL_VL_SCHED_MODE_VLARB		0	/* Gen1 VLARB */
#define STL_VL_SCHED_MODE_AUTOMATIC	2	/* hardcoded, not configurable */

/* STL Port Flit preemption limits of unlimited */
#define STL_PORT_PREEMPTION_LIMIT_NONE		255 /* Unlimited */

#define BYTES_PER_LTP 128

/* NOTE - first-pass ordering of PortInfo members:
 *   1  RW members before RO members;
 *   2  Roughly prioritize RW and RO sections;
 *   3  No separation of RO and RW members within sub-structures.
 *
 *   Attribute Modifier as NNNN NNNN 0000 0000 0000 000A PPPP PPPP
 *
 *   N = number of ports
 *   A = 1 - All ports starting at P
 *   P = port number
 */
typedef struct {
	STL_LID LID;					/* RW/HSPE H-PE: base LID of this node */
									/*               POD: 0 */
									/*         -S--: base LID of neighbor node */
									/*               POD/LUD: 0 */

	uint32  FlowControlMask;		/* RW/-S-- Flow control mask (1 bit per VL) */
									/* POD/LUD: flow control enabled all VLs except VL15 */

	struct {
		uint8	PreemptCap;		/* RO/HS-E size of Preempting VL Arbitration table */
						/* only used when VLSchedulingConfig */
						/* is VL_SCHED_MODE_VLARB, otherwise reserved */

		struct { IB_BITFIELD2( uint8,
			Reserved:		3,
			Cap:			5 )		/* RO/HS-E Virtual Lanes supported on this port */
		} s2;

		uint16  HighLimit;		/* RW/HS-E Limit of high priority component of */
						/*  VL Arbitration table */
						/* only used when VLSchedulingConfig */
						/* is VL_SCHED_MODE_VLARB, otherwise reserved */
						/* POD: 0 */
		uint16  PreemptingLimit;	/* RW/HS-E Limit of preempt component of */
						/*  VL Arbitration table */
						/* only used when VLSchedulingConfig */
						/* is VL_SCHED_MODE_VLARB, otherwise reserved */
						/* POD: 0 */

		union {
			uint8   ArbitrationHighCap;	/* RO/HS-E VL Arbitration table cap */
						/* only used when VLSchedulingConfig */
						/* is VL_SCHED_MODE_VLARB, otherwise reserved */
		};

		uint8   ArbitrationLowCap;	/* RO/HS-E VL Arbitration table cap */
						/* only used when VLSchedulingConfig */
						/* is VL_SCHED_MODE_VLARB, otherwise reserved */
	} VL;

	STL_PORT_STATES  PortStates;		/* Port states */

	STL_FIELDUNION2(PortPhysConfig,8,
			Reserved:4,				/* Reserved */
			PortType:4);            /* RO/HS-- PORT_TYPE */
						/* Switch port 0 shall report Fixed */
						/* reserved when PortLinkMode.Active is not STL */

	struct { IB_BITFIELD3( uint8,	/* Multicast/Collectives masks */
		Reserved:			2,
		CollectiveMask:		3,	/* RW/H--- Num of additional upper 1s in */
								/* Collective address */
								/* POD: 1 */
								/* Reserved in Gen1 */
		MulticastMask:		3 )	/* RW/H--- Num of upper 1s in Multicast address */
								/* POD: 4 */
	} MultiCollectMask;

	struct { IB_BITFIELD3( uint8,
		M_KeyProtectBits:   2,		/* RW/H-PE see mgmt key usage */
		Reserved:   		2,  	/* reserved, shall be zero */
		LMC:				4 )		/* RW/HSPE LID mask for multipath support */
									/*    H---: POD: 0 */
									/*    --PE: POD/LUD: 0 */
									/*    -S--: LID mask for Neighbor node */
									/*      POD/LUD: 0 */
	} s1;

	struct { IB_BITFIELD2( uint8,
		Reserved:			3,
		MasterSMSL:			5 )	/* RW/H-PE The adminstrative SL of the master */
								/* SM that is managing this port */
	} s2;

	struct { IB_BITFIELD5( uint8,
		LinkInitReason:					4,  /*RW/HSPE POD: 1, see STL_LINKINIT_REASON */
		PartitionEnforcementInbound:	1,	/* RW/-S-- */
											/* LUD: 1 neighbor is HFI, 0 else */
		PartitionEnforcementOutbound:   1,	/* RW/-S-- */
											/* LUD: 1 neighbor is HFI, 0 else */
		Reserved20:			1,
		Reserved21:			1 )	
	} s3;

	struct { IB_BITFIELD2( uint8,
		Reserved:			3,
		OperationalVL:		5 )			/* RW/HS-E Virtual Lanes operational this port */
	} s4;

	struct {						/* STL Partial P_Keys */
		uint16  P_Key_8B;			/* RW/HS-E Implicit 8B P_Key */
		uint16  P_Key_10B;			/* RW/HS-E Partial upper 10B P_Key */
									/*  (12 bits, lower 4 bits reserved) */
	} P_Keys;						/* POD/LUD: 0 */

	struct {
		uint16  M_Key;		/* RW/H-PE */
		uint16  P_Key;		/* RW/H-PE */
		uint16  Q_Key;		/* RW/H-PE */
	} Violations;			/* POD: 0 */

	STL_FIELDUNION2(SM_TrapQP, 32,
		Reserved: 	8,
		QueuePair:	24 );		/* RW/HS-E SM Trap QP. POD/LUD: 0 */

	STL_FIELDUNION2(SA_QP, 32,
		Reserved:	8,
		QueuePair:	24 );		/* RW/HS-E SA QP. POD/LUD: 1 */

	uint8   NeighborPortNum;	/* RO/HS-- Port number of neighbor node */

	uint8   LinkDownReason;		/* RW/HS-E Link Down Reason (see STL_LINKDOWN_REASON_XXX) */
								/* POD: 0 */

	uint8	NeighborLinkDownReason;/* RW/HS-E Neighbor Link Down Reason - STL_LINKDOWN_REASON */
								/* POD: 0 */

	struct { IB_BITFIELD3( uint8,
		ClientReregister:	1,	/* RW/H-PE POD/LUD: 0 */
		MulticastPKeyTrapSuppressionEnabled:2,	/* RW/H-PE */
		Timeout:			5 )	/* RW/H-PE Timer value used for subnet timeout */
	} Subnet;

	struct {					/* Link speed (see STL_LINK_SPEED_XXX) LinkBounce */
		uint16  Supported;  	/* RO/HS-E Supported link speed */
		uint16  Enabled;		/* RW/HS-E Enabled link speed POD: = supported */
		uint16  Active;			/* RO/HS-E Active link speed */
	} LinkSpeed;

	struct {					/* 9(12) of each 16 bits used (see STL_LINK_WIDTH_XXX) */
								/* LinkBounce */
		uint16  Supported;		/* RO/HS-E Supported link width */
		uint16  Enabled;		/* RW/HS-E Enabled link width POD: = supported */
		uint16  Active;			/* RO/HS-E link width negotiated by LNI*/
	} LinkWidth;

	struct {					/* Downgrade of link on error (see STL_LINK_WIDTH_XXX) */
		uint16  Supported;		/* RO/HS-E Supported downgraded link width */
		uint16  Enabled;		/* RW/HS-E Enabled link width downgrade */
								/* POD/LUD: = supported */
		uint16  TxActive;		/* RO/HS-E Currently active link width in tx dir */
		uint16	RxActive;		/* RO/HS-- Currently active link width in rx dir */
	} LinkWidthDowngrade;

	STL_FIELDUNION4(PortLinkMode,16, 	/* STL/Eth Port Link Modes */
									/* (see STL_PORT_LINK_MODE_XXX) */
		Reserved:	1,
		Supported:	5,					/* RO/HS-E Supported port link mode */
		Enabled:	5,					/* RW/HS-E Enabled port link mode POD: from FW INI */
		Active:		5 );				/* RO/HS-E Active port link mode */

	STL_FIELDUNION4(PortLTPCRCMode, 16,	/* STL Port LTP CRC Modes */
						/* (see STL_PORT_LTP_CRC_MODE_XXX) */
		Reserved:   4,
		Supported:	4,		/* RO/HS-E Supported port LTP mode */
						/* reserved when PortLinkMode.Active is not STL */
		Enabled:	4,		/* RW/HS-E Enabled port LTP mode POD: from FW INI */
						/* reserved when PortLinkMode.Active is not STL */
		Active:		4 );		/* RO/HS-E Active port LTP mode */
						/* reserved when PortLinkMode.Active is not STL */

	STL_FIELDUNION7(PortMode, 16, 		/* General port modes */
		Reserved:				9,
		IsActiveOptimizeEnabled:	1,	/* RW/HS-- Optimized Active handling */
										/* POD/LUD: 0 */
		IsPassThroughEnabled:	1,		/* RW/-S-- Pass-Through LUD: 0 */
		IsVLMarkerEnabled:		1,		/* RW/HS-- VL Marker LUD: 0 */
		Reserved2:				2,
		Is16BTrapQueryEnabled:	1,		/* RW/H-PE 16B Traps & SA/PA Queries (else 9B) */
										/* LUD: 0 */
		Reserved3:				1 );	/* RW/-S-- SMA Security Checking */
										/* LUD: 1 */

	struct {						/* Packet formats */
									/* (see STL_PORT_PACKET_FORMAT_XXX) */
		uint16  Supported;			/* RO/HSPE Supported formats */
		uint16  Enabled;			/* RW/HSPE Enabled formats */
	} PortPacketFormats;

	struct {						/* Flit control LinkBounce */
		union {
			uint16  AsReg16;
			struct { IB_BITFIELD5( uint16,	/* Flit interleaving */
				Reserved:			2,
				DistanceSupported:	2,	/* RO/HS-E Supported Flit distance mode */
										/* (see STL_PORT_FLIT_DISTANCE_MODE_XXX) */
				DistanceEnabled:	2,	/* RW/HS-E Enabled Flit distance mode */
										/* (see STL_PORT_FLIT_DISTANCE_MODE_XXX) */
										/* LUD: mode1 */
				MaxNestLevelTxEnabled:		5,	/* RW/HS-E Max nest level enabled Flit Tx */
												/* LUD: 0 */
				MaxNestLevelRxSupported:	5 )	/* RO/HS-E Max nest level supported Flit Rx */
			} s;
		} Interleave;

		struct Preemption_t {				/* Flit preemption */
			uint16  MinInitial;	/* RW/HS-E Min bytes before preemption Head Flit */
									/* Range 8 to 10240 bytes */
			uint16  MinTail;	/* RW/HS-E Min bytes before preemption Tail Flit */
									/* Range 8 to 10240 bytes */
			uint8   LargePktLimit;	/* RW/HS-E Size of packet that can be preempted */
									/* Packet Size >= 512+(512*LargePktLimit) */
									/* Packet Size Range >=512 to >=8192 bytes */
			uint8   SmallPktLimit;	/* RW/HS-E Size of packet that can preempt */
									/* Packet Size <= 32+(32*SmallPktLimit) */
									/* Packet Size Range <=32 to <=8192 bytes */
									/* MaxSmallPktLimit sets upper bound allowed */
			uint8   MaxSmallPktLimit;/* RO/HS-E Max value for SmallPktLimit */
									/* Packet Size <= 32+(32*MaxSmallPktLimit) */
									/* Packet Size Range <=32 to <=8192 bytes */
			uint8   PreemptionLimit;/* RW/HS-E Num bytes of preemption */
									/* limit = (256*PreemptionLimit) */
									/* Limit range 0 to 65024, 0xff=unlimited */
		} Preemption;

	} FlitControl;

	STL_LID  MaxLID;					/* RW/H---: POD: 0xBFFF */

	union _PortErrorAction {
		uint32  AsReg32;
		struct { IB_BITFIELD25( uint32,		/* RW/HS-E Port Error Action Mask */
											/* POD: 0 */
			ExcessiveBufferOverrun:			1,
			Reserved:						7,
			FmConfigErrorExceedMulticastLimit:	1,
			FmConfigErrorBadControlFlit:	1,
			FmConfigErrorBadPreempt:		1,
			FmConfigErrorUnsupportedVLMarker:	1,
			FmConfigErrorBadCrdtAck:		1,
			FmConfigErrorBadCtrlDist:		1,
			FmConfigErrorBadTailDist:		1,
			FmConfigErrorBadHeadDist:		1,
			Reserved2:						2,
			PortRcvErrorBadVLMarker:		1,
			PortRcvErrorPreemptVL15:		1,
			PortRcvErrorPreemptError:		1,
			Reserved3:						1,
			PortRcvErrorBadMidTail:			1,
			PortRcvErrorReserved:			1,
			PortRcvErrorBadSC:				1,
			PortRcvErrorBadL2:				1,
			PortRcvErrorBadDLID:			1,
			PortRcvErrorBadSLID:			1,
			PortRcvErrorPktLenTooShort:		1,
			PortRcvErrorPktLenTooLong:		1,
			PortRcvErrorBadPktLen:			1,
			Reserved4:						1 )
		} s;
	} PortErrorAction;

	struct {					/* Pass through mode control */
		uint8   EgressPort;		/* RW/-S-- Egress port: 0-disable pass through */
								/* LUD: 0 */

		IB_BITFIELD2( uint8,
		Reserved:	7,
		DRControl:	1 )			/* RW/-S-- DR: 0-normal process, 1-repeat on egress port */
								/* LUD: 0 */

	} PassThroughControl;

	uint16  M_KeyLeasePeriod;   /* RW/H-PE LUD: 0 */

	STL_FIELDUNION5(BufferUnits, 32, /* VL bfr & ack unit sizes (bytes) */
		Reserved:		9,
		VL15Init:		12,		/* RO/HS-E Initial VL15 units (N) */
		VL15CreditRate:	5,		/* RW/HS-E VL15 Credit rate (32*2^N) */
									/* LUD: if neighbor is STL HFI: 18, otherwise 0 */
		CreditAck:		3,		/* RO/HS-E Credit ack unit (BufferAlloc*2^N) */
		BufferAlloc:	3 );	/* RO/HS-E Buffer alloc unit (8*2^N) */

	uint16  Reserved14;
	uint8   BundleNextPort;		/* RO/HS-- next logical port in a bundled connector */
	uint8   BundleLane;			/* RO/HS-- first lane in connector associated with this port */

	STL_LID MasterSMLID;		/* RW/H-PE The base LID of the master SM that is */
								/* managing this port */
								/* POD/LUD: 0 */

	uint64  M_Key;  			/* RW/H-PE The 8-byte management key */
								/* POD/LUD: 0 */

	uint64  SubnetPrefix;   	/* RW/H-PE Subnet prefix for this port */
								/* Set to default value if no */
								/* other subnet interaction */
								/* POD: 0xf8000000:00000000 */

	STL_VL_TO_MTU  NeighborMTU[STL_MAX_VLS / 2];	/* RW/HS-E Neighbor MTU values per VL */
													/* VL15 LUD: 2048 STL mode */

	struct XmitQ_s { IB_BITFIELD2( uint8,	/* Transmitter Queueing Controls */
									/* per VL */
		VLStallCount:   3,			/* RW/-S-- Applies to switches only */
									/* LUD: 7 */
		HOQLife:		5 )			/* RW/-S-- Applies to routers & switches only */
									/* LUD: infinite */
	} XmitQ[STL_MAX_VLS];

/* END OF RW SECTION */

/* BEGINNING OF RO SECTION */
	STL_IPV6_IP_ADDR  IPAddrIPV6;	/* RO/H-PE IP Address - IPV6 */

	STL_IPV4_IP_ADDR  IPAddrIPV4;	/* RO/H-PE IP Address - IPV4 */

	uint32  Reserved26;
	uint32  Reserved27;

	uint32 Reserved28;

	uint64  NeighborNodeGUID;   /* RO/-S-E GUID of neighbor connected to this port */

	STL_CAPABILITY_MASK  CapabilityMask;	/* RO/H-PE Capability Mask */

	uint16  Reserved20;

	STL_CAPABILITY_MASK3  CapabilityMask3;	/* RO/H-PE Capability Mask 3 */

	uint32	Reserved23;

	uint16  OverallBufferSpace;		/* RO/HS-E Overall dedicated + shared space */

	struct {				/* most significant 8 bits of Replay depths */
		uint8   BufferDepthH;		/* RO/HS-- Replay buffer depth MSB */
						/* reserved when PortLinkMode.Active is not STL */
		uint8   WireDepthH;		/* RO/HS-- Replay wire depth MSB */
						/* reserved when PortLinkMode.Active is not STL */
	} ReplayDepthH;

	STL_FIELDUNION3(DiagCode, 16, 	/* RO/H-PE Diagnostic code, Refer Node Diagnostics */
		UniversalDiagCode:		4,
		VendorDiagCode:			11,
		Chain:			        1 );

	struct {				/* least significant 8 bits of Replay depths */
		uint8   BufferDepth;		/* RO/HS-E Replay buffer depth LSB */
						/* reserved when PortLinkMode.Active is not STL */
		uint8   WireDepth;		/* RO/HS-E Replay wire depth LSB */
						/* reserved when PortLinkMode.Active is not STL */
	} ReplayDepth;

	struct { IB_BITFIELD4( uint8,	/* RO/HS-E Port modes based on neighbor */
		Reserved:				4,
		MgmtAllowed:			1,	/* RO/H--- neighbor allows this node to be mgmt */
									/* Switch: mgmt is allowed for neighbor */
									/* EP0: mgmt is allowed for port */
		NeighborFWAuthenBypass:	1,	/* RO/-S-E 0=Authenticated, 1=Not Authenticated */
		NeighborNodeType:		2 )	/* RO/-S-E 0=HFI (not trusted), 1=Switch (trusted) */
	} PortNeighborMode;

	struct { IB_BITFIELD2( uint8,
		Reserved20:		4,			
		Cap:			4 )			/* RO/HS-E Max MTU supported by this port */
	} MTU;

	struct { IB_BITFIELD2( uint8,
		Reserved:	3,
		TimeValue:	5 )				/* RO/H-PE */
	} Resp;
	
	uint8   LocalPortNum;			/* RO/HSPE The link port number this SMP came on in */

	uint8   Reserved25;
	uint8	Reserved24;

} PACK_SUFFIX STL_PORT_INFO;


/*
 * PartitionTable
 *
 * Attribute Modifier as: NNNN NNNN PPPP PPPP 0000 0BBB BBBB BBBB
 *                        N:   Number of blocks
 *                        P:   Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 *                        B:   Block number
 *
 * The (max) PartitionTable is 2**16 entries (P_Keys) long, 16 bits wide.
 * Each PartitionTable block is 32 entries long, 16 bits wide.  The Partition
 * Table is a linear array of blocks[2**11].
 */

#define NUM_PKEY_ELEMENTS_BLOCK		(32)	/* Num elements per block; currently same as IB */
#define MAX_PKEY_BLOCK_NUM			0x7FF
#define PKEY_BLOCK_NUM_MASK			0x7FF
#define STL_DEFAULT_PKEY		    0x7FFF
#define STL_DEFAULT_APP_PKEY		0x8001
#define STL_DEFAULT_FM_PKEY         0xFFFF
#define STL_DEFAULT_CLIENT_PKEY		0x7FFF
#define STL_DEFAULT_APP_PKEY_IDX	0
#define STL_DEFAULT_CLIENT_PKEY_IDX	1
#define STL_DEFAULT_FM_PKEY_IDX     2
#define STL_MIN_PKEY_COUNT          3

typedef union {
	uint16  AsReg16;
	struct { IB_BITFIELD2( uint16,
		MembershipType:		1,				/* 0=Limited, 1=Full */
		P_KeyBase:			15 )			/* Base value of the P_Key that */
											/*  the endnode will use to check */
											/*  against incoming packets */
	} s;

} PACK_SUFFIX STL_PKEY_ELEMENT;

#define STL_NUM_PKEY_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_PARTITION_TABLE)))
#define STL_NUM_PKEY_BLOCKS_PER_LID_SMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / sizeof(STL_PARTITION_TABLE)))

typedef struct {

	STL_PKEY_ELEMENT PartitionTableBlock[NUM_PKEY_ELEMENTS_BLOCK];	/* RW List of P_Key Block elements */

} PACK_SUFFIX STL_PARTITION_TABLE;


/*
 * SL_TO_SC Mapping table
 *
 * Attribute Modifier as: 0 (not used)
 *
 * This Attribute is used only for HFIs and switch port 0
 *
 * Each block contains 32 bytes of SL->SC data, 1 byte per SL.
 */
typedef struct {
	STL_SC  SLSCMap[STL_MAX_SLS];	/* RW/HSPE */
									/*    H---: POD: SLn_to_SCn (1-to-1) */

} PACK_SUFFIX STL_SLSCMAP;

/*
 * SC_TO_SC Mapping table
 *
 * Attribute Modifier as: NNNN NNNN 0000 00AB IIII IIII EEEE EEEE
 *                        N:   Number of blocks (egress ports (B=0),
 *								 but if B=1 then ingress ports)
 *                        A=1: All ingress ports starting at I (Set only; excludes port 0)
 *                        B=1: All egress ports starting at E (Set only; excludes port 0)
 *                        I:   Ingress port number (0 - reserved, switches only)
 *                        E:   Egress port number (0 - reserved, switches only)
 *
 * This attribute is not applicable to HFIs.
 *
 * Each block contains 32 bytes of SC->SC data, 1 byte per SC.
 */
typedef struct {
	STL_SC  SCSCMap[STL_MAX_SCS];	/* RW/HSPE */
									/*    -SPE: POD: SCn_to_SCn (1-to-1) */

} PACK_SUFFIX STL_SCSCMAP;

#define STL_NUM_SCSC_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_SCSCMAP)))
#define STL_NUM_SCSC_BLOCKS_PER_LID_SMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / sizeof(STL_SCSCMAP)))

/*
 * SC_TO_SC MultiSet
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 0000 0000 0000
 *						N:	Number of Multi-Set Blocks
 *
 * This attribute is not applicable to HFIs.
 *
 * Each block contains 32 bytes of SC->SC data, 1 byte per SC and ingress/egress port masks
 */

typedef struct {
	STL_PORTMASK    IngressPortMask[STL_MAX_PORTMASK];
	STL_PORTMASK    EgressPortMask[STL_MAX_PORTMASK];
	STL_SCSCMAP		SCSCMap;				/* RW/HSPE */
											/*    -SPE: POD: SCn_to_SCn (1-to-1) */
} PACK_SUFFIX STL_SCSC_MULTISET;


#define STL_NUM_SCSC_MULTI_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_SCSC_MULTISET)))
#define STL_NUM_SCSC_MULTI_BLOCKS_PER_LRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / sizeof(STL_SCSC_MULTISET)))


/*
 * SC_TO_SL Mapping table
 *
 * Attribute Modifier as: 0 (not used)
 *
 * This Attribute is used only for HFIs and switch port 0
 *
 * Each block contains 32 bytes of SC->SL data, 1 byte per SC.
 */
typedef struct {					/* RW POD: SCn_to_SLn (1-to-1) */
	STL_SL  SCSLMap[STL_MAX_SCS];

} PACK_SUFFIX STL_SCSLMAP;

/*
 * SC_TO_VL Mapping table
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 000Y 000A PPPP PPPP
 *                        N:   Number of blocks (ports)
 *                        Y=1: Async update (Set of SC-to-VLt only with link state Armed or Active)
 *                        A=1: All ports starting at P (Set only)
 *                        P:   Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 *
 * Each block contains 32 bytes of SC->VL data, 1 byte per SC.
 *
 * NOTE: this attribute applies for all SC-to-VL tables (SC-to-VLr, SC-to-VLt,
 * SC-to-VLnt); Port is ingress port or egress port as applicable.
 */
typedef struct {
	STL_VL  SCVLMap[STL_MAX_SCS];   /* RW/HSPE POD: SCn_to_VLn (1-to-1) */

} PACK_SUFFIX STL_SCVLMAP;

#define STL_NUM_SCVL_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_SCVLMAP)))
#define STL_NUM_SCVL_BLOCKS_PER_LID_SMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / sizeof(STL_SCVLMAP))) 


/*
 * VL Arbitration Table (aka VL Arbitration Table 2)
 *
 * Attribute Modifier as: NNNN NNNN SSSS SSSS 0000 000A PPPP PPPP
 *                        N:   Number of ports
 *                        S:   Section of Table as:
 *                             0: Arbitration Low Elements
 *                             1: Arbitration High Elements
 *                             2: Preemption Elements
 *                             3: Preemption Matrix
 *                             4-255: Reserved
 *                        A=1: All ports starting at P (Set only)
 *                        P:   Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 */

#define STL_MAX_LOW_CAP             128 /* Also applies to High but not Preempt */
#define STL_MAX_PREEMPT_CAP         32  /* MAX Preempt table size is 32 */
#define STL_VLARB_LOW_ELEMENTS       0
#define STL_VLARB_HIGH_ELEMENTS      1
#define STL_VLARB_PREEMPT_ELEMENTS   2
#define STL_VLARB_PREEMPT_MATRIX     3
#define STL_VLARB_NUM_SECTIONS		 4

typedef struct {
	struct { IB_BITFIELD2( uint8,
		Reserved:		3,
		VL:				5 )		/* RW */
	} s;

	uint8   Weight;				/* RW */

} PACK_SUFFIX STL_VLARB_TABLE_ELEMENT;

#define VLARB_TABLE_LENGTH 128
typedef union {
	STL_VLARB_TABLE_ELEMENT  Elements[VLARB_TABLE_LENGTH]; /* RW */
	uint32                   Matrix[STL_MAX_VLS];	/* RW */
													/* POD: 0 */

} PACK_SUFFIX STL_VLARB_TABLE;

#define STL_NUM_VLARB_PORTS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_VLARB_TABLE)))
#define STL_NUM_VLARB_PORTS_PER_LID_SMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / sizeof(STL_VLARB_TABLE))) 


/*
 * Linear Forwarding Table (LFT)
 *
 * Attribute Modifier as: NNNN NNNN 0000 0ABB BBBB BBBB BBBB BBBB
 *                        N:   Number of blocks
 *                        A=1: All blocks starting at B (Set only)
 *                        B:   Block number
 *
 * The (max) LFT is 2**24 entries (LIDs) long (STL_LID_24), 8 bits wide.
 * Each LFT block is 64 entries long, 8 bits wide.  The LFT is a
 * linear array of blocks[2**18].
 */

#define MAX_LFT_ELEMENTS_BLOCK	(64)	/* Max elements per block; currently same as IB */
#define MAX_LFT_BLOCK_NUM	0x3FFFF

#define STL_NUM_LFT_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR/MAX_LFT_ELEMENTS_BLOCK))
typedef struct {
	PORT  LftBlock[MAX_LFT_ELEMENTS_BLOCK];

} PACK_SUFFIX STL_LINEAR_FORWARDING_TABLE;



/*
 * Multicast Forwarding Table (MFT)
 *
 * Attribute Modifier as: NNNN NNNN PP0A BBBB BBBB BBBB BBBB BBBB
 *                        N:   Number of blocks
 *                        P:   Position number
 *                        A=1: All blocks starting at B (Set only)
 *                        B:   Block number
 *
 * The (max) MFT is 2**23 entries (LIDs) long (STL_LID_24 / 2), 256 bits wide.
 * Each MFT block is 8 entries long, 64 bits wide.  The MFT is a
 * 2-dimensional array of blocks[2**20][4].
 */


#define STL_NUM_MFT_ELEMENTS_BLOCK	8	/* Num elements per block */
#define STL_NUM_MFT_POSITIONS_MASK	4	/* Num positions per 256-bit port mask */
#define STL_MAX_MFT_BLOCK_NUM		0xFFFFF
#define STL_PORT_MASK_WIDTH			64		/* Width of STL_PORTMASK in bits */
#define STL_MAX_PORTS				255

typedef struct {
	STL_PORTMASK  MftBlock[STL_NUM_MFT_ELEMENTS_BLOCK];

} PACK_SUFFIX STL_MULTICAST_FORWARDING_TABLE;

#define STL_NUM_MFT_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR/sizeof(STL_MULTICAST_FORWARDING_TABLE)))

/*
 * SMInfo
 *
 * Attribute Modifier as: 0 (not used)
 */
typedef struct {
	uint64  PortGUID;   		/* RO This SM's perception of the PortGUID */
								/* of the master SM */
	uint64  SM_Key; 			/* RO Key of this SM. This is shown as 0 unless */
								/* the requesting SM is proven to be the */
								/* master, or the requester is otherwise */
								/* authenticated */
	uint32  ActCount;   		/* RO Counter that increments each time the SM */
								/* issues a SMP or performs other management */
								/* activities. Used as a 'heartbeat' indicator */
								/* by standby SMs */
	uint32  ElapsedTime;   		/* RO Time (in seconds): time Master SM has been */
								/* Master, or time since Standby SM was last */
								/* updated by Master */
	union {
		uint16  AsReg16;
		struct { IB_BITFIELD4( uint16,
			Priority:			4, 	/* RO Administratively assigned priority for this */
									/* SM. Can be reset by master SM */
			ElevatedPriority:	4,	/* RO This SM's elevated priority */
			InitialPriority:	4,	/* RO This SM's initial priority */
			SMStateCurrent:		4 )	/* RO This SM's current state (see SM_STATE) */
		} s;
	} u;
		
} PACK_SUFFIX STL_SM_INFO;


/* 
 * LEDInfo
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 000A PPPP PPPP
 *                        N:   Number of ports
 *                        A=1: All ports starting at P (Set only)
 *                        P:   Port number (must be 0 for HFI)
 */

typedef struct {
	union { 
		uint32 AsReg32;
		struct { IB_BITFIELD2( uint32,
			LedMask: 	1,			/* RW POD: 0 */
			Reserved:	31 );
		} s;
	} u;
	uint32 Reserved2;

} PACK_SUFFIX STL_LED_INFO;

/*
 * CableInfo
 *
 * Attribute Modifier as: 0AAA AAAA AAAA ALLL LLL0 0000 PPPP PPPP
 *                        A: Starting address of cable data
 *                        L: Length (bytes) of cable data - 1
 *                           (L+1 bytes of data read)  
 *                        P: Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 *
 * NOTE: Cable Info is mapped onto a linear 4096-byte address space (0-4095).
 * Cable Info can only be read within 128-byte pages; that is, a single
 * read cannot cross a 128-byte (page) boundary.
 */
#define STL_CABLE_INFO_DATA_SIZE 	64
typedef struct {
	uint8   Data[STL_CABLE_INFO_DATA_SIZE];			/* RO Cable Info data (up to 64 bytes) */
} PACK_SUFFIX STL_CABLE_INFO;

#define STL_CABLE_INFO_PAGESZ 	128
#define STL_CABLE_INFO_MAXADDR 	4095
#define STL_CABLE_INFO_MAXLEN 	63

// Note: Even though the entire cable memory is available to be read,
//  tools only interpret one page of cable info memory which contains
//  relevant data.

// These are for PortType Standard, uses of these can assume START and END
// will be on and STL_CABLE_INFO_DATA_SIZE boundary
#define STL_CIB_STD_LOW_PAGE_ADDR		0
#define STL_CIB_STD_HIGH_PAGE_ADDR		128
#define STL_CIB_STD_END_ADDR		(STL_CIB_STD_HIGH_PAGE_ADDR+STL_CABLE_INFO_PAGESZ-1)
#define STL_CIB_STD_LEN				(STL_CABLE_INFO_PAGESZ)

#define STL_CIB_STD_MAX_STRING			16		// Max ASCII string in STD CableInfo field

// Byte 0 and byte 128: identifier (SFF-8024)
#define STL_CIB_STD_QSFP				0xC		// QSFP transceiver identifier value
#define STL_CIB_STD_QSFP_PLUS			0xD		// QSFP+ transceiver identifier value
#define STL_CIB_STD_QSFP_28				0x11	// QSFP28 transceiver identifier value
#define STL_CIB_STD_QSFP_DD				0x18	// QSFP-DD transceiver identifier value

// Byte 129: pwr_class_low, pwr_class_high
#define STL_CIB_STD_PWRLOW_1_5			0		// Pwr class low class 1 (1.5 W)
#define STL_CIB_STD_PWRLOW_2_0			1		// Pwr class low class 2 (2.0 W)
#define STL_CIB_STD_PWRLOW_2_5			2		// Pwr class low class 3 (2.5 W)
#define STL_CIB_STD_PWRLOW_3_5			3		// Pwr class low class 4 (3.5 W)
#define STL_CIB_STD_PWRHIGH_LEGACY		0		// Pwr class high legacy settings
#define STL_CIB_STD_PWRHIGH_4_0			1		// Pwr class high class 5 (4.0 W)
#define STL_CIB_STD_PWRHIGH_4_5			2		// Pwr class high class 6 (4.5 W)
#define STL_CIB_STD_PWRHIGH_5_0			3		// Pwr class high class 7 (5.0 W)

// Byte 130: connector
#define STL_CIB_STD_CONNECTOR_MPO1x12	0x0C	// Connector type is MPO 1x12
#define STL_CIB_STD_CONNECTOR_MPO2x16	0x0D	// Connector type is MPO 2x16
#define STL_CIB_STD_CONNECTOR_NO_SEP	0x23	// Connector type is non-separable
#define STL_CIB_STD_CONNECTOR_MXC2x16	0x24	// Connector type is MXC 2x16

// Byte 140: bit_rate_low
#define STL_CIB_STD_RATELOW_NONE		0		// Nominal bit rate low not specified 
#define STL_CIB_STD_RATELOW_EXCEED		0xFF	// Nominal bit rate low > 25.4 Gbps

// Byte 147: dev_tech.xmit_tech
#define STL_CIB_STD_TXTECH_850_VCSEL		0x0	// Tx tech 850 nm VCSEL
#define STL_CIB_STD_TXTECH_1310_VCSEL		0x1	// Tx tech 1310 nm VCSEL
#define STL_CIB_STD_TXTECH_1550_VCSEL		0x2	// Tx tech 1550 nm VCSEL
#define STL_CIB_STD_TXTECH_1310_FP			0x3	// Tx tech 1310 nm FP
#define STL_CIB_STD_TXTECH_1310_DFB			0x4	// Tx tech 1310 nm DFB
#define STL_CIB_STD_TXTECH_1550_DFB			0x5	// Tx tech 1550 nm DFB
#define STL_CIB_STD_TXTECH_1310_EML			0x6	// Tx tech 1310 nm EML
#define STL_CIB_STD_TXTECH_1550_EML			0x7	// Tx tech 1550 nm EML
#define STL_CIB_STD_TXTECH_OTHER			0x8	// Tx tech Other/Undefined
#define STL_CIB_STD_TXTECH_1490_DFB			0x9	// Tx tech 1490 nm DFB
#define STL_CIB_STD_TXTECH_CU_UNEQ			0xA	// Tx tech Cu unequalized
#define STL_CIB_STD_TXTECH_CU_PASSIVEQ		0xB	// Tx tech Cu passive equalized
#define STL_CIB_STD_TXTECH_CU_NFELIMACTEQ	0xC	// Tx tech Cu near & far end limiting active equalizers
#define STL_CIB_STD_TXTECH_CU_FELIMACTEQ	0xD	// Tx tech Cu far end limiting active equalizers
#define STL_CIB_STD_TXTECH_CU_NELIMACTEQ	0xE	// Tx tech Cu near end limiting active equalizers
#define STL_CIB_STD_TXTECH_CU_LINACTEQ		0xF	// Tx tech Cu linear active equalizers
#define STL_CIB_STD_TXTECH_MAX				0xF	// Tx tech max value

// Byte 250: opa_cert_cable
#define STL_CIB_STD_OPA_CERTIFIED_CABLE		0xAB	// OPA certified cable

// Byte 252: opa_cert_data_rate
#define STL_CIB_STD_OPACERTRATE_4X25G		0x02	// Certified data rate 4x25G

// The following structure represents STD CableInfo page 0 upper in memory.
// (based on SFF-8636 Rev 2-5)
typedef struct {
	// Page 0 upper, bytes 128-255
	uint8	ident;					// 128: Identifier
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD5( uint8,	// 129: Extended identifier:
			pwr_class_low:	2,			//		Power class low
			other:			2,			//		Other settings
			tx_cdr_supp:	1,			//		Tx CDR support
			rx_cdr_supp:	1,			//		Rx CDR support
			pwr_class_high:	2)			//		Power class low
		} s;
	} ext_ident;				
	uint8	connector;				// 130: Connector (see STL_CIB_CONNECTOR_TYPE_xxx)
	uint8	spec_comp[8];			// 131-138: Elec/optical compliance code
	uint8	encode;					// 139: Encoding algorithm
	uint8	bit_rate_low;			// 140: Nominal bit rate low (units 100 Mbps)
									//		(0xFF see bit_rate_high)
	uint8	ext_rate_comp;			// 141: Extended rate compliance code
	uint8	len_smf;				// 142: Link len SMF fiber (units km)
	uint8	len_om3;				// 143: Link len OM3 fiber (units 2m)
	uint8	len_om2;				// 144: Link len OM2 fiber (units 1m)
	uint8	len_om1;				// 145: Link len OM1 fiber (units 1m)
	uint8	len_om4;				// 146: Link len OM4 copper or fiber (units 1m or 2m)
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD2( uint8,	// 147: Device technology:
			xmit_tech:	4,				//		Transmitter technology
			other:		4)				//		Other settings
		} s;
	} dev_tech;				
	uint8	vendor_name[16];		// 148-163: Vendor name
	uint8	ext_mod;				// 164: Extended module code
	uint8	vendor_oui[3];			// 165-167: Vendor OUI
	uint8	vendor_pn[16];			// 168-183: Vendor part number
	uint8	vendor_rev[2];			// 184-185: Vendor revision
	uint8	wave_atten[2];			// 186-187: Wave length (value/20 nm) or
									//			copper attenuation (units dB)
	uint8	wave_tol[2];			// 188-189: Wave length tolerance (value/200 nm)
	uint8	max_case_temp;			// 190: Max case temperature (degrees C)
	uint8	cc_base;				// 191: Checksum addresses 128-190
	uint8	link_codes;				// 192: Link codes
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD5( uint8,   // 193: RxTx options: equalization & emphasis
			reserved:		4,			//		Reserved
			tx_inpeq_autadp_cap:	1,	//		Tx inp equal auto-adaptive capable
			tx_inpeq_fixpro_cap:	1,	//		Tx inp equal fixed-prog capable
			rx_outemp_fixpro_cap:	1,	//		Rx outp emphasis fixed-prog capable
			rx_outamp_fixpro_cap:	1)	//		Rx outp amplitude fixed-prog capable
		} s;
	} rxtx_opt_equemp;
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD8( uint8,   // 194: RxTx options: CDR, LOL, squelch
			tx_cdr_ctrl:	1,			//		Tx CDR On/Off ctrl implemented
			rx_cdr_ctrl:	1,			//		Rx CDR On/Off ctrl implemented
			tx_cdr_lol:		1,			//		Tx CDR loss of lock flag implemented
			rx_cdr_lol:		1,			//		Rx CDR loss of lock flag implemented
			rx_squel_dis:	1,			//		Rx squelch disable implemented
			rx_out_dis:		1,			//		Rx output disable implemented
			tx_squel_dis:	1,			//		Tx squelch disable implemented
			tx_squel:		1)			//		Tx squelch implemented
		} s;
	} rxtx_opt_cdrsquel;
	union {
		uint8	AsReg8;
		struct { IB_BITFIELD8( uint8,   // 195: MemTx options: pages 1 & 2, implementations
			page_2:				1,		//		Mem page 2 implemented
			page_1:				1,		//		Mem page 1 implemented
			rate_sel:			1,		//		Rate select implemented
			tx_dis:				1,		//		Tx disable implemented
			tx_fault:			1,		//		Tx fault signal implemented
			tx_squel_omapav:	1,		//		Tx squelch OMA/Pave
			tx_los:				1,		//		Tx loss of signal implemented
			reserved:			1)		//		Reserved
		} s;
	} memtx_opt_pagesquel;
	uint8	vendor_sn[16];			// 196-211: Vendor serial number
	uint8	date_code[8];			// 212-219: Vendor manufacture date code
	uint8	diag_mon_type;			// 220: Diagnostic monitoring type
	uint8	options_enh;			// 221: Enhanced options
	uint8	bit_rate_high;			// 222: Nominal bit rate high (units 250 Mbps)
									//		(see also bit_rate_low)
	uint8	cc_ext;					// 223: Checksum addresses 192-222
	uint8	vendor[26];				// 224-249: Vendor specific
	uint8	opa_cert_cable;			// 250: OPA certified cable (see STL_CIB_CERTIFIED_CABLE)
	uint8	vendor2;				// 251: Vendor specific
	uint8	opa_cert_data_rate;		// 252: OPA certified data rate
	uint8	vendor3[3];				// 253-255: Vendor specific
} PACK_SUFFIX STL_CABLE_INFO_STD;

// The following structure represents CableInfo page 0 upper (DD) in memory.
// (based on Rev 0.61 frozen memory map)
typedef struct {
	uint8	ident;					// 128: Identifier
	uint8	vendor_name[16];		// 129-144: Vendor name
	uint8	vendor_oui[3];			// 145-147: Vendor OUI
	uint8	vendor_pn[16];			// 148-163: Vendor part number
	uint8	vendor_rev[2];			// 164-165: Vendor revision
	uint8	vendor_sn[16];			// 166-181: Vendor serial number
	uint8	date_code[8];			// 182-189: Vendor manufacture date code
	uint8	reserved1[11];			// 190-200: Reserved
	uint8	powerMax;				// 201: Max power dissipation, in 0.25W increments
	uint8	cableLengthEnc;			// 202: Cable assembly length
	uint8	connector;				// 203: Connector (see STL_CIB_CONNECTOR_TYPE_xxx)
	uint8	reserved2[8];			// 204-211: Reserved
	uint8	cable_type;				// 212: Cable type (optics/passive/active Cu)
	uint8	reserved3[43];			// 213-255: Reserved
} PACK_SUFFIX STL_CABLE_INFO_UP0_DD;

/*
 * Aggregate
 *
 * Attribute Modifier as: 0000 0000 0000 0000 0000 0000 NNNN NNNN
 *                        N: Number of aggregated attributes
 *                            (1-MAX_AGGREGATE_ATTRIBUTES)
 *
 * A wrapper attribute, containing a sequence of AttributeID/AttributeModifier/Payload
 * segments.  Each segment begins on an 8-byte boundary and contains a payload
 * length specified by RequestLength (in 8-byte units).  RequestLength is
 * supplied by the Requester and specifies the amount of data in the request
 * and the response.  The offset from the beginning of one segment to the
 * next is determined by RequestLength.  If a request or response does not fit
 * within RequestLength then an error is set in Status; response data is undefined
 * in this situation.
 */
										/* Max number of aggregate attributes */
#define MAX_AGGREGATE_ATTRIBUTES  (STL_MAX_PAYLOAD_SMP_DR / 16)
#define STL_MAX_PAYLOAD_AGGREGATE 1016

typedef struct {
	uint16  AttributeID;
	union {
		uint16  AsReg16;
		struct { IB_BITFIELD3( uint16,
			Error:			1,	/* 1: Error (Invalid AttributeID/Modifier, */
								/*	   RequestLength, Attribute Data) */
			Reserved:		8,
			RequestLength:	7 )	/* Request length (8-byte units) */
		} s;

	} Result;

	uint32  AttributeModifier;

	uint8   Data[0];
} PACK_SUFFIX STL_AGGREGATE;

/*
 * Returns a pointer to the first member of an aggregate MAD.
 * Header must be in HOST byte order.
 */
#define STL_AGGREGATE_FIRST(smp) (STL_AGGREGATE*)((smp->common.MgmtClass == MCLASS_SM_LID_ROUTED)?(smp->SmpExt.LIDRouted.SMPData):(smp->SmpExt.DirectedRoute.SMPData))

/*
 * Given an STL_AGGREGATE member, returns the next member.
 * Member header must be in HOST byte order. Does not range check.
 */ 
#define STL_AGGREGATE_NEXT(pAggr) ((STL_AGGREGATE*)((uint8*)(pAggr)+((pAggr)->Result.s.RequestLength*8)+sizeof(STL_AGGREGATE)))

/*
 * PortStateInfo
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 00S0 PPPP PPPP
 *                        N: Number of ports
 *                        S=1: Start of SM configuration
 *                        P: Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 *
 * A list of port state information, one entry per port (starting at port P).
 */
typedef struct {
	STL_PORT_STATES  PortStates;		/* RW */
	uint16 LinkWidthDowngradeTxActive;  /* RO */
	uint16 LinkWidthDowngradeRxActive;	/* RO */

} PACK_SUFFIX STL_PORT_STATE_INFO;
#define STL_NUM_PORT_STATE_INFO_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / sizeof(STL_PORT_STATES)))
#define STL_NUM_PORT_STATE_INFO_BLOCKS_PER_LID_SMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / sizeof(STL_PORT_STATES)))


/*
 * Port Group Forwarding Table (PGFT)
 *
 * Attribute Modifier as: NNNN NNNN 0000 0ABB BBBB BBBB BBBB BBBB
 *                        N:   Number of blocks
 *                        A=1: All blocks starting at B (Set only)
 *                        B:   Block number
 *
 * The (max) PGFT is 2**24 entries (LIDs) long (STL_LID_24), 8 bits wide.
 * Each PGFT block is 64 entries long, 8 bits wide.  The PGFT is a
 * linear array of blocks[2**18].
 */

#define NUM_PGFT_ELEMENTS_BLOCK	0x40		/* Num elements per block */
#define DEFAULT_MAX_PGFT_BLOCK_NUM 0x80		/* Cap for alpha PRR is 128 blocks. */
#define DEFAULT_MAX_PGFT_LID ((DEFAULT_MAX_PGFT_BLOCK_NUM * NUM_PGFT_ELEMENTS_BLOCK) - 1)
#define STL_NUM_PGFT_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR/NUM_PGFT_ELEMENTS_BLOCK))

typedef struct {							/* RW POD: PGFTn=0xFF */
	PORT   PgftBlock[NUM_PGFT_ELEMENTS_BLOCK];

} PACK_SUFFIX STL_PORT_GROUP_FORWARDING_TABLE;


/*
 * Port Group Table (PGT)
 *
 * Attribute Modifier as: NNNN NNNN PP00 0000 0000 0000 00AB BBBB
 *                        N:   Number of blocks
 *                        P:   Position number
 *                        A=1: All blocks starting at B (Set only)
 *                        B:   Block number
 *
 * The PGT is 256 entries long, 256 bits wide.
 * Each PGT block is 8 entries long, 64 bits wide.  The PGT is a
 * 2-dimensional array of blocks[32][4].
 */

#define NUM_PGT_ELEMENTS_BLOCK	8		/* Num elements per block */
#define MAX_PGT_BLOCK_NUM	0x1F
#define MAX_PGT_ELEMENTS 256
#define STL_NUM_PGTABLE_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR/NUM_PGFT_ELEMENTS_BLOCK))

typedef struct {						/* RW POD: PGTn=0 */
	STL_PORTMASK  PgtBlock[NUM_PGT_ELEMENTS_BLOCK];

} PACK_SUFFIX STL_PORT_GROUP_TABLE;


/*
 * BufferControlTable
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 000A PPPP PPPP
 *                        N:   Number of ports
 *                        A=1: All ports starting at P (Set only)
 *                        P:   Port number (0 - management port, switches only;
 *								for HFIs P is ignored and the attribute is
 *								applicable only to the port that receives the packet)
 * Note:
 *   All receive values in buffer allocation units of this receiver
 *   All transmit values in buffer allocation units of neighbor receiver
 */
typedef struct {
	uint16  Reserved;
	uint16  TxOverallSharedLimit;			/* RW Overall shared limit */
		
	struct STL_BUFFER_CONTROL_TABLE_VL_s {	/* Per VL data */
		uint16  TxDedicatedLimit;			/* RW Dedicated limit */
		uint16  TxSharedLimit;				/* RW Shared limit */
	} VL[STL_MAX_VLS];
		
} PACK_SUFFIX STL_BUFFER_CONTROL_TABLE;

// NOTE: STL_BUFFER_CONTROL_TABLE is NOT 8 byte aligned. When doing multiblock queries, each block
// *MUST* be rounded to the nearest 8 byte boundary, or PRR will reject any SMA request for more than
// one block. In the future we may just want to pad out the structure, for now the following macros should
// be used when working with BCT blocks inside of SMAs.
#define STL_BFRCTRLTAB_PAD_SIZE ((sizeof(STL_BUFFER_CONTROL_TABLE)+7)&~0x7)
#define STL_NUM_BFRCTLTAB_BLOCKS_PER_DRSMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_DR / STL_BFRCTRLTAB_PAD_SIZE))
#define STL_NUM_BFRCTLTAB_BLOCKS_PER_LID_SMP ((uint8_t)(STL_MAX_PAYLOAD_SMP_LR / STL_BFRCTRLTAB_PAD_SIZE))

/*
 * Congestion Info
 *
 * Attribute Modifier as: 0 (not used)
 */

#define CC_CONGESTION_INFO_CREDIT_STARVATION 0x0001		/* Supports credit starvation (switch only) */

typedef struct {
	uint16	CongestionInfo;				/* RO Defined in IBTA V1.2.1 A10.4.3.3. */
	uint8	ControlTableCap;			/* RO Number of supported entries in */
										/* HFI Congestion Control Table */
	uint8	CongestionLogLength;		/* RO. Legal values are 0 <= N <= 96 */
} STL_CONGESTION_INFO;


/*
 * Switch Congestion Log
 *
 * Attribute Modifier as: 0 (not used)
 *
 * The Log is up to 96 entries long.  Each Log is 96 entries long.
 * If a request asks for more entries than the device supports, the
 * remainder should be zero-filled.
 */

#define STL_NUM_CONGESTION_LOG_ELEMENTS	96		/* Max num elements in log (SW and HFI) */

typedef struct {						/* POD: 0 */
	STL_LID	SLID;
	STL_LID	DLID;

	IB_BITFIELD2( uint8,
		SC:			5,
		Reserved:	3 );

	uint8	Reserved2;
	uint16	Reserved3;

	uint32	TimeStamp;

} STL_SWITCH_CONGESTION_LOG_EVENT;

typedef struct {						/* RO */
	uint8   LogType;					/* shall be 1 in response from a switch */
	uint8   CongestionFlags;
	uint16  LogEventsCounter;				/* POD: 0 */
	uint32  CurrentTimeStamp;				/* POD: 0 */
	uint8   PortMap[256/8];

	STL_SWITCH_CONGESTION_LOG_EVENT  CongestionEntryList[STL_NUM_CONGESTION_LOG_ELEMENTS];

} PACK_SUFFIX STL_SWITCH_CONGESTION_LOG;


/*
 * Switch Congestion Setting
 *
 * Attribute Modifier as: 0 (not used)
 */

/* bitfields for Control_Map */
#define CC_SWITCH_CONTROL_MAP_VICTIM_VALID		0x00000001		/* Victim_Mask */
#define CC_SWITCH_CONTROL_MAP_CREDIT_VALID		0x00000002		/* Credit_Mask */
#define CC_SWITCH_CONTROL_MAP_CC_VALID			0x00000004		/* Threshold & Packet_Size */
#define CC_SWITCH_CONTROL_MAP_CS_VALID			0x00000008		/* CS_threshold & CS_ReturnDelay */
#define CC_SWITCH_CONTROL_MAP_MARKING_VALID		0x00000010		/* Marking Rate */

typedef struct {						/* RW */
	uint32  Control_Map;				/* POD: 0 */
	uint8   Victim_Mask[256/8];			/* POD: 0 */
	uint8   Credit_Mask[256/8];			/* POD: 0 */

	IB_BITFIELD2( uint8,
		Threshold:	4,					/* POD: 0 */
		Reserved:	4 );

	uint8   Packet_Size;				/* in 64 byte units */
										/* POD: 0 */

	IB_BITFIELD2( uint8,
		CS_Threshold:	4,				/* POD: 0 */
		Reserved2:		4 );

	uint8   Reserved3;

	uint16  CS_ReturnDelay;				/* POD: 0 */
	uint16  Marking_Rate;				/* POD: 0 */

} PACK_SUFFIX STL_SWITCH_CONGESTION_SETTING;


/*
 * Switch Port Congestion Setting
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 0000 PPPP PPPP
 *                        N:   Number of blocks
 *                        P:   Port number
 */

// Values for Control_Type
#define SWITCH_PORT_CONGESTION_CONTROL_TYPE_CC			0
#define SWITCH_PORT_CONGESTION_CONTROL_TYPE_STARVATION	1

typedef struct {
	IB_BITFIELD4( uint8,			/* POD: 0 */
		Valid:			1,
		Control_Type:	1,
		Reserved:		2,
		Threshold:      4 );		/* 0-15 where 0 = no marking and       */
									/* 15 = most aggressive                */

	uint8 Packet_Size;				/* POD: 0. When Control_Type == 1      */
									/* this field is reserved. When Control*/
									/* _Type == 0, this field contains the */
									/* minimum size of a packet which may  */
									/* be marked with a FECN. Packets      */
									/* smaller than this size will not be  */
									/* marked. Packet_Size is specified in */
									/* credits.                            */
	uint16 Marking_Rate;			/* POD: 0. When Control_Type == 0 this */
									/* contains the port marking rate,     */
									/* defined as the minimum number of    */
									/* packets between marking eligible    */
									/* packets with a FECN.                */
} PACK_SUFFIX STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT;

typedef struct {					/* RW */

STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT  Elements[1];

} PACK_SUFFIX STL_SWITCH_PORT_CONGESTION_SETTING;


/*
 * HFI Congestion Log
 *
 * Attribute Modifier as: 0 (not used)
 *
 * The Log is up to 96 entries long.  Each Log is 96 entries long.
 * If a request asks for more entries than the device supports, the
 * remainder should be zero-filled.
 */
#define CC_RC_TYPE 0
#define CC_UC_TYPE 1
#define CC_RD_TYPE 2
#define CC_UD_TYPE 3

typedef struct {					/* POD: 0 */
	struct {
		uint8   AsReg8s[3];
	} Local_QP_CN_Entry;			/* 0->port threshold reached */

	struct {
		uint8   AsReg8s[3];
	} Remote_QP_Number_CN_Entry;	/* 0 for UD */

	IB_BITFIELD2( uint8,
		SL_CN_Entry:			5,
		Service_Type_CN_Entry:	3 );

	uint8   Reserved;

	STL_LID  Remote_LID_CN_Entry;	/* IBTA already used 32 bits for this */

	uint32  TimeStamp_CN_Entry;

} PACK_SUFFIX STL_HFI_CONGESTION_LOG_EVENT;

typedef struct {						/* RO */
	uint8   LogType;					/* shall be 0x2 in a response from an HFI */
	uint8   CongestionFlags;
	uint16  ThresholdEventCounter;		/* POD: 0 */
	uint32  CurrentTimeStamp;			/* POD: 0 */
	uint8   ThresholdCongestionEventMap[STL_MAX_SLS/8];	/* 1 bit per SL */

	STL_HFI_CONGESTION_LOG_EVENT  CongestionEntryList[STL_NUM_CONGESTION_LOG_ELEMENTS];

} PACK_SUFFIX STL_HFI_CONGESTION_LOG;

/*
 * HFI Congestion Setting
 *
 * Attribute Modifier as: 0 (not used)
 *
 * This attribute applicable to HFIs or Enhanced Switch Port 0.
 */

/* Port_Control field bit values */
#define CC_HFI_CONGESTION_SETTING_SL_PORT	0x0001		/* SL/Port control */

typedef struct {
	uint8   CCTI_Increase;				/* POD: 0 */
	uint8   Reserved;
	uint16  CCTI_Timer;					/* POD: 0 */
	uint8   TriggerThreshold;			/* POD: 0 */
	uint8   CCTI_Min;					/* POD: 0 */
} STL_HFI_CONGESTION_SETTING_ENTRY;

typedef struct {						/* RW */
	uint32  Control_Map;				/* POD: 0 */
	uint16  Port_Control;				/* POD: 0 */

	STL_HFI_CONGESTION_SETTING_ENTRY  HFICongestionEntryList[STL_MAX_SLS];

} PACK_SUFFIX STL_HFI_CONGESTION_SETTING;


/*
 * HFI Congestion Control Table
 *
 * Attribute Modifier as: NNNN NNNN 0000 0000 0000 0000 BBBB BBBB
 *                        N:   Number of blocks
 *                        B:   Block number
 *
 * The HFI Congestion Control Table is up to 2**14 entries long.  Each block
 * is 64 entries long.  Table is a linear array of blocks[2**8].
 */

#define STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES	64		/* Num elements per block */
#define STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCKS			256		/* Max num blocks per table */
#define STL_CONGESTION_CONTROL_ENTRY_MAX_VALUE				16384	/* 2**14 max multiplier value*/

typedef union {
	uint16  AsReg16;
	struct { IB_BITFIELD2( uint16,
		CCT_Shift:			2,			/* POD: 0 */
		CCT_Multiplier:		14 );		/* POD: 0 */
	} s;
} PACK_SUFFIX STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY;

typedef struct {
	STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY	CCT_Entry_List[STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES];
} PACK_SUFFIX STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK;

typedef struct {						/* RW */
	uint16  CCTI_Limit;					/* POD: 0 */
	STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK  CCT_Block_List[1]; /* 1 or more blocks */

} PACK_SUFFIX STL_HFI_CONGESTION_CONTROL_TABLE;

#define CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ	(sizeof(uint16))

/* this is conservative and considers the least payload */
#define CONGESTION_CONTROL_TABLE_BLOCKS_PER_MAD \
    ((MIN(STL_MAX_PAYLOAD_SMP_DR, STL_MAX_PAYLOAD_SMP_LR) - CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ) / sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK))
#define CONGESTION_CONTROL_TABLE_ENTRIES_PER_MAD \
		(CONGESTION_CONTROL_TABLE_BLOCKS_PER_MAD  * \
			STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES)
#define CONGESTION_CONTROL_IMPLEMENTATION_LIMIT 895

typedef struct {
   uint64  M_Key;  			/* A 64 bit key, */
   STL_LID DrSLID;            /* Directed route source LID */
   STL_LID DrDLID;            /* Directed route destination LID */
   uint8   InitPath[64];      /* 64-byte field containing the initial */
                              /* directed path */
   uint8   RetPath[64];       /* 64-byte field containing the */
                              /* returning directed path */
   uint8   Reserved2[8];      /* For the purpose of aligning the Data */
                              /* field on a 16-byte boundary */
   uint8   SMPData[STL_MAX_PAYLOAD_SMP_DR];  /* Up to 'MAX' byte field of SMP */
                                             /* data used to contain the */
                                             /* method's attribute */
} PACK_SUFFIX DRStlSmp_t; 
#define STL_SMP_DR_HDR_LEN      (sizeof(DRStlSmp_t) - STL_MAX_PAYLOAD_SMP_DR)

typedef struct {
   uint64  M_Key;  			/* A 64 bit key, */
   uint8   SMPData[STL_MAX_PAYLOAD_SMP_LR];  /* Up to 'MAX' byte field of SMP */
                                             /* data used to contain the */
                                             /* method's attribute */
   
} PACK_SUFFIX LRStlSmp_t; 
#define STL_SMP_LR_HDR_LEN      (sizeof(LRStlSmp_t) - STL_MAX_PAYLOAD_SMP_LR)

#include "iba/public/ipackoff.h"

#if defined (__cplusplus)
};
#endif

#endif /* __IBA_STL_SM_TYPES_H__ */
