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

#ifndef _IBA_IB_TYPES_H_
#define _IBA_IB_TYPES_H_

/* Basic Infiniband data types */

#define IB1_1	1	/* set to 1 to build IB 1.1 compliant version, 0 for 1.0a */

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _PCI_DEVICE;

typedef void		*IB_HANDLE;

typedef uint32		IB_L_KEY;
typedef uint32		IB_R_KEY;
typedef uint32		IB_Q_KEY;
typedef uint16		IB_P_KEY;
typedef uint16		IB_LID;
typedef	uint8		IB_PATHBITS;
#define IB_PATHBITS_MASK	0x7f	/* path bits are no more than 7 bits */
typedef uint8		IB_LMC;
typedef	uint8		IB_SL;
typedef	uint64		IB_VIRT_ADDR;

typedef	uint64		EUI64;

/* the well-known Q_Key for QP1. */
#define	QP1_WELL_KNOWN_Q_KEY				0x80010000U

/* the default P_Key */
#define DEFAULT_P_KEY						0xffffU

/* -------------------------------------------------------------------------- */
/* LID's */

#define	LID_RESERVED						0U		/* RESERVED;  */
#define	LID_PERMISSIVE						0xffffU	/* PROMISCUOUS;  */

#define LID_UCAST_START						0x0001U
#define LID_UCAST_END						0xbfffU

#define LID_MCAST_START						0xc000U
#define LID_MCAST_END						0xfffeU


/* -------------------------------------------------------------------------- */
/* GID's		 */
#define	IPV6_LINK_LOCAL_PREFIX				0x3faU	/* 1111111010b */
#define	IPV6_SITE_LOCAL_PREFIX				0x3fbU	/* 1111111011b */
#define IPV6_MULTICAST_PREFIX				0xffU	/* 11111111b */

/* Multicast Flags definitions */
#define IPV6_MCAST_FLAG_PERMANENT			0U
#define IPV6_MCAST_FLAG_TRANSIENT			1U

/* Multicast Address Scope definitions */
typedef enum {
	IPV6_RESERVED_0 = 0,
	IPV6_NODE_LOCAL,
	IPV6_LINK_LOCAL,
	IPV6_RESERVED_3,
	IPV6_RESERVED_4,
	IPV6_SITE_LOCAL,
	IPV6_RESERVED_6,
	IPV6_RESERVED_7,
	IPV6_ORG_LOCAL,
	IPV6_RESERVED_9,
	IPV6_RESERVED_10,
	IPV6_RESERVED_11,
	IPV6_RESERVED_12,
	IPV6_RESERVED_13,
	IPV6_GLOBAL,
	IPV6_RESERVED_15
} IPV6_MCAST_SCOPE;


/* Default GID subnet prefix.  Can be combined with the port GUID to form a */
/* GID usable on the local subnet. */
#define DEFAULT_SUBNET_PREFIX	0xFE80000000000000Ull

/* format fields for each format type */
#define IB_GID_FORMAT_LINK_LOCAL	0x3faU	/* 10 bits */
#define IB_GID_FORMAT_SITE_LOCAL	0x3fbU	/* 10 bits */
#define IB_GID_FORMAT_MCAST 		0xffU	/* 8 bits */
#define IB_GID_MCAST_FLAG_T			0x01U	/* if set, non-permanent MC GID */

	/* values for scope other than those below are reserved */
#define IB_GID_MCAST_SCOPE_LINK_LOCAL	0x2U
#define IB_GID_MCAST_SCOPE_SITE_LOCAL	0x5U
#define IB_GID_MCAST_SCOPE_ORG_LOCAL	0x8U
#define IB_GID_MCAST_SCOPE_GLOBAL		0xeU

#define IB_GID_MCAST_FORMAT_MASK_H		0xFF00000000000000Ull
#define IB_GID_ALL_CA_MCAST_H			0xFF02000000000000Ull
#define IB_GID_ALL_CA_MCAST_L			0x0000000000000001Ull

#define IB_QP_MCAST						0xffffffU	/* dest QP for multicast */

#include "iba/public/ipackon.h"

typedef union _IB_GID {
	uchar	Raw[16];
	struct {
#if CPU_BE
		uint32	HH;
		uint32	HL;
		uint32	LH;
		uint32	LL;
#else
		uint32	LL;
		uint32	LH;
		uint32	HL;
		uint32	HH;
#endif
	} AsReg32s;
	struct {
#if CPU_BE
		uint64	H;
		uint64	L;
#else
		uint64	L;
		uint64	H;
#endif
	} AsReg64s;
	union _IB_GID_TYPE {
		struct {
#if CPU_BE
			struct {
				uint64		FormatPrefix:	10;
				uint64		Reserved:		54;	/* Must be zero */
			} s;
			EUI64		InterfaceID;
#else
			EUI64		InterfaceID;
			struct {
				uint64		Reserved:		54;	/* Must be zero */
				uint64		FormatPrefix:	10;
			} s;
#endif
		} LinkLocal;
		
		struct {
#if CPU_BE
			struct {
				uint64		FormatPrefix:	10;
				uint64		Reserved:		38;	/* Must be zero */
				uint64		SubnetPrefix:	16;
			} s;
			EUI64		InterfaceID;
#else
			EUI64		InterfaceID;
			struct {
				uint64		SubnetPrefix:	16;
				uint64		Reserved:		38;	/* Must be zero */
				uint64		FormatPrefix:	10;
			} s;
#endif
		} SiteLocal;
		
		struct {
#if CPU_BE
			uint64		SubnetPrefix;
			EUI64		InterfaceID;
#else
			EUI64		InterfaceID;
			uint64		SubnetPrefix;
#endif
		} Global;
		
		struct {
#if CPU_BE
			struct {
				uint16		FormatPrefix:	8;
				uint16		Flags:		4;
				uint16		Scope:		4;
			} s;
			uchar			GroupId[14];
#else
			uchar			GroupId[14];
			struct {
				uint16		Scope:			4;
				uint16		Flags:			4;
				uint16		FormatPrefix:	8;
			} s;
#endif
		} Multicast;
	} Type;
} PACK_SUFFIX IB_GID;


/*
 * The IB Global Route header found in the first 40 bytes of the first data
 * segment on globally routed Unreliable Datagram messages.
 *
 * TBD - cleanup to use similar names to GlobalRouteInfo in vpi.h
 */

typedef struct _IB_GRH {
	union {
		uint32 AsReg32;

		struct _S_GRH {
#if CPU_LE
			uint32	FlowLabel:	20;	/* This field identifies sequences of  */
									/* packets requiring special handling. */
			uint32	TClass	:	8;	/* Traffic Class-This field is used by */
									/* IBA to communicate global service level */
			uint32	IPVer	:	4;	/* IP Version-This field indicates version */
									/* of the GRH */
#else
			uint32	IPVer	:	4;	/* IP Version-This field indicates version */
									/* of the GRH */
			uint32	TClass	:	8;	/* Traffic Class-This field is used by */
									/* IBA to communicate global service level */
			uint32	FlowLabel:	20;	/* This field identifies sequences of  */
									/* packets requiring special handling. */
#endif
		} s;
	} u1;

	uint16	PayLen;		/* Payload length -This field indicates the length of  */
						/* the packet in bytes following the GRH.  */
						/* This includes from the first byte after the end of */
						/* the GRH up to but not including either the VCRC  */
						/* or any padding to achieve 4 byte length.  */
						/* For raw packets with GRH, the padding is determined */
						/* from the lower two bits of this GRH:Payload length  */
						/* field. (For IBA packets it is determined from the  */
						/* pad field in the transport header.) Padding is */
						/* placed immediately before the VCRC field. */
						/* Note: GRH:PayLen is different from LRH:PkyLen. */
	uint8	NxtHdr;		/* Next Header-This field identifies the header  */
						/* following the GRH. This field is included for  */
						/* compatibility with IPV6 headers.  */
						/* It should indicate IBA transport. */
	uint8	HopLimit;	/* Hop Limit-This field sets a strict bound on  */
						/* the number of hops between subnets a packet  */
						/* can make before being discarded. This is */
						/* enforced only by routers. */
	IB_GID	SGID;		/* Source GID-This field identifies the  */
						/* Global Identifier (GID) for the port */
						/* which injected the packet into the network. */
	IB_GID	DGID;		/* Destination GID-This field identifies the GID  */
						/* for the port which will consume */
						/* the packet from the network. */
} PACK_SUFFIX IB_GRH;

#include "iba/public/ipackoff.h"

static __inline
void
BSWAP_IB_GID(
	IB_GID	*IbGid
	)
{
#if CPU_LE
	/* Swap all 128 bits */
	uint64 temp = IbGid->AsReg64s.L;

	IbGid->AsReg64s.L = ntoh64(IbGid->AsReg64s.H);
	IbGid->AsReg64s.H = ntoh64(temp);
#endif
}



static __inline
void
BSWAP_IB_GRH(
	IB_GRH			*Dest
	)
{	
#if CPU_LE
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	Dest->PayLen = ntoh16(Dest->PayLen);

	BSWAP_IB_GID(&Dest->SGID);
	BSWAP_IB_GID(&Dest->DGID);
#endif
} 

/* MTU of neighbor endnode connected to this port:
 * this enum is used for all SM and CM messages
 * it is also used in other datatypes as noted below
 */
typedef enum _IB_MTU {
	IB_MTU_256		= 1,
	IB_MTU_512		= 2,
	IB_MTU_1024		= 3,
	IB_MTU_2048		= 4,
	IB_MTU_4096		= 5,
    IB_MTU_MAX		= 5
	/* 0, 6 - 15 (or 63 in some packets): reserved */
} IB_MTU;


/* Static rate for link.
 * This enum is used in all SM, CM and ADDRESS_VECTOR APIs
 */
typedef enum _IB_STATIC_RATE {
	IB_STATIC_RATE_DONTCARE = 0,	/* allowed for SA query */
									/* for ADDRESS_VECTOR means local port rate */
	IB_STATIC_RATE_1GB = 1,		/* obsolete, now reserved */
	IB_STATIC_RATE_1X = 2,		/* depricated, use IB_STATIC_RATE_2_5G */
	IB_STATIC_RATE_2_5G = 2,	/* 2.5 Gb/sec (1X SDR) */
	IB_STATIC_RATE_MIN = 2,		/* lowest standard rate */
	IB_STATIC_RATE_4X = 3,		/* depricated, use IB_STATIC_RATE_10G */
	IB_STATIC_RATE_10G = 3,		/* 10.0 Gb/sec (4X SDR, 1X QDR) */
	IB_STATIC_RATE_12X = 4,		/* depricated, use IB_STATIC_RATE_30G */
	IB_STATIC_RATE_30G = 4,		/* 30.0 Gb/sec (12X SDR) */
	IB_STATIC_RATE_5G = 5,		/* 5.0 Gb/sec (1X DDR) */
	IB_STATIC_RATE_20G = 6,		/* 20.0 Gb/sec (4X DDR, 8X SDR) */
	IB_STATIC_RATE_40G = 7,		/* 40.0 Gb/sec (4X QDR, 8X DDR) */
	IB_STATIC_RATE_60G = 8,		/* 60.0 Gb/sec (12X DDR) */
	IB_STATIC_RATE_80G = 9,		/* 80.0 Gb/sec (8X QDR) */
	IB_STATIC_RATE_120G = 10,	/* 120.0 Gb/sec (12X QDR) */
	IB_STATIC_RATE_LAST_QDR = 10,	/* last QDR value, all FDR/EDR must be after this */
	IB_STATIC_RATE_14G = 11,	/* 14.0625 Gb/sec (1X FDR) */
	IB_STATIC_RATE_56G = 12,	/* 56.25 Gb/sec (4X FDR) */
	IB_STATIC_RATE_112G = 13,	/* 112.5 Gb/sec (8X FDR) */
	IB_STATIC_RATE_168G = 14,	/* 168.75 Gb/sec (12X FDR) */
	IB_STATIC_RATE_25G = 15,	/* 25.78125 Gb/sec (1X EDR) */
	IB_STATIC_RATE_100G = 16,	/* 103.125 Gb/sec (4X EDR) */
	IB_STATIC_RATE_200G = 17,	/* 206.25 Gb/sec (8X EDR) */
	IB_STATIC_RATE_300G = 18,	/* 309.375 Gb/sec (12X EDR) */
	IB_STATIC_RATE_MAX = 18,	/* highest standard rate */
	IB_STATIC_RATE_LAST = 18	/* last valid value */
	/* 19-63 reserved */
} IB_STATIC_RATE;

/* IB does not define rate for SMI/GSI packets and
 * IB does not have Static rate in UD LRH
 * so we can't be sure what rate of remote port is for SMI/GSI req/response
 * we use a constant value for SMI/GSI QPs, this uses full port speed
 * an alternative could be to use 1X speed to be conservative
 * (GSI should not be a performance path?)
 */
#define IB_STATIC_RATE_GSI IB_STATIC_RATE_DONTCARE
#define IB_STATIC_RATE_SMI IB_STATIC_RATE_DONTCARE

/*
 * Per Channel Adapter definitions and data types
 */

typedef enum {
	CAAtomicNone,	/* No Atomic operations supported. */
	CAAtomicLocal,	/* Atomicity guaranteed only between QPs on this CA. */
	CAAtomicGlobal	/* Atomicity guaranteed between CA and components that can */
					/* access the target memory. */
					/* of the atomic operation. */
} IB_CA_ATOMIC_LVL;

typedef enum {
	CAPathMigNone,	/* Channel Interface does not support path migration */
	CAPathMigAuto	/* Automatic Path Migration supported by CI */
} IB_CA_PATH_MIGRATION_LVL;


/*
 * Port Attributes
 */

/*
 * Port LID and LMC values for addressing within the Subnet
 */
typedef struct _IB_PORT_ADDRESS {
	IB_LID		BaseLID;	/* Base LID */
	IB_LMC		LMC;		/* LMC */
} IB_PORT_ADDRESS;

/*
 * Info needed to reach the Subnet Manager for a port
 */
typedef struct _IB_PORT_SM_ADDRESS {
	IB_LID		LID;		/* Base LID */
	IB_SL		ServiceLevel;
} IB_PORT_SM_ADDRESS;

/*
 * IB Port States (matches values for SMA PortInfo.PortState)
 */
typedef enum _IB_PORT_STATE {
	PortStateNop = 0,		/* Noop - only for SMA Set */
	PortStateDown = 1,		/* Down (includes failed links) */
	PortStateInit = 2,		/* Initialize */
	PortStateArmed = 3,		/* Armed */
	PortStateActive = 4,	/* Active */
	PortStateMax = 4		/* Maximum Valid Value */
} IB_PORT_STATE;

/* Port Capabilities (a subset of those in IB_PORT_INFO) */
typedef enum _IB_PORT_CAPABILITIES {
	PORT_IS_SM							= 0x00000001U,
	PORT_IS_SM_DISABLED					= 0x00000002U,
	PORT_IS_SNMP_TUNNELING_SUPPORTED	= 0x00000004U,
	PORT_IS_DEVICE_MGMT_SUPPORTED		= 0x00000008U,
	PORT_IS_VENDOR_CLASS_SUPPORTED		= 0x00000010U
} IB_PORT_CAPABILITIES;

/* Port initialization type requested from SM */
typedef enum _IB_PORT_INIT_TYPE {
	PORT_INIT_TYPE_NOLOAD				= 0x00000001U,
	PORT_INIT_TYPE_PRESERVE_CONTENT		= 0x00000002U,
	PORT_INIT_TYPE_PRESERVE_PRESENCE	= 0x00000004U,
	PORT_INIT_TYPE_DO_NOT_RESUSCITATE	= 0x00000008U
} IB_PORT_INIT_TYPE;

/* Port Link Width Active */
typedef enum _IB_PORT_WIDTH {
	PORT_WIDTH_1X =0,
	PORT_WIDTH_4X =1,
	PORT_WIDTH_12X =2,
	PORT_WIDTH_8X =3
} IB_PORT_WIDTH;

/* Port Link Speed Active */
typedef enum _IB_PORT_SPEED {
	PORT_SPEED_2_5G =0,
	PORT_SPEED_5G   =1,
	PORT_SPEED_10G  =2,
	PORT_SPEED_14G  =3,
	PORT_SPEED_25G  =4
} IB_PORT_SPEED;

/*
 * This is the per port attributes structure.
 * As there are two variable sized tables, pointers are used to reference into
 * the tables as well as the beginning of the next port's attribute structure.
 */
typedef struct _IB_PORT_ATTRIBUTES {
	struct _IB_PORT_ATTRIBUTES	*Next;	/* Pointer to the structure for the */
										/* next port.  */
										/* Null if this is last port in list. */
	IB_GID		*GIDTable;				/* Pointer to the GID table */
	IB_P_KEY	*PkeyTable;				/* Pointer to the Pkey table */
	EUI64		GUID;					/* Port GUID */
	uint32		VirtualLanes;			/* Number of data virtual lanes */
										/* supported (from VLCap incl. VL 15) */
										/* 2, 3, 5, 9 or 16 */
	IB_PORT_STATE	PortState;
	IB_PORT_CAPABILITIES	Capabilities;	/* optional capabilities */
	IB_PORT_INIT_TYPE	InitTypeReply;	/* initialization type requested */
	uint32		QkeyViolationCounter;	/* Invalid Qkey counter */
	uint16		PkeyViolationCounter;	/* Invalid Pkey counter */
	uint8		NumPkeys;				/* Number of Pkeys in the Pkey table */
	uint8		NumGIDs;				/* Number of GIDs in the GID table */
	IB_PORT_ADDRESS		Address;		/* Port addressing info */
	uint8		ActiveWidth:2;			/* IB_PORT_WIDTH enum */
	uint8		ActiveSpeed:3;			/* IB_PORT_SPEED enum */
	uint8		Reserved:3;
	IB_PORT_SM_ADDRESS	SMAddress;		/* SM info for the port */

	uint8	SubnetTimeout;		/* Specifies the maximum expected subnet  */
								/* propagation delay, which depends upon the  */
								/* configuration of the switches, to reach any  */
								/* port in the subnet and shall also be used to  */
								/* determine the maximum rate which SubnTraps()  */
								/* can be sent from this port.  */
								/* The duration of time is calculated based on  */
								/* (4.096 uS * 2^SubnetTimeOut) */

	uint8	RespTimeValue;		/* Specifies the expected maximum time between  */
								/* the port reception of a SMP and the  */
								/* transmission of the associated response.  */
								/* The duration of time is calculated based on  */
								/* (4.096 uS * 2^RespTimeValue).  */
								/* The default value shall be 8. */
	uint32	NeighborMTU;		/* Maximum Transfer Unit (packet size) */
								/* which has been configured between this */
								/* port and the neighbor port */
								/* represented in bytes (256, 512, ... 4096) */
	uint32	MaxMTU;				/* Maximum Transfer Unit supported */
								/* (packet size) from MTUCap */
								/* represented in bytes (256, 512, ... 4096) */
	uint64	MaxMessageLen;		/* Maximum message length */
	/* add new fields here to ensure binary compatibility with existing code */

	/* */
	/* The GID and Pkey tables are located here at the end of the port */
	/* attributes structure. The caller must allocate the space to hold the */
	/* tables at the end of this structure. The size indicated in */
	/* PortAttributesListSize will be used to determine the number of port */
	/* attributes structures that may be returned. */
	/* */
} IB_PORT_ATTRIBUTES;

/* Page size is 2K * 2^bit_position */
#define IB_MR_PAGESIZE_MIN_LOG2	11
#define IB_MR_PAGESIZE_MIN	2048
#define IB_MR_PAGESIZE_2K	0x000000001U
#define IB_MR_PAGESIZE_4K	0x000000002U
#define IB_MR_PAGESIZE_8K	0x000000004U
#define IB_MR_PAGESIZE_16K	0x000000008U
#define IB_MR_PAGESIZE_32K	0x000000010U
#define IB_MR_PAGESIZE_64K	0x000000020U
#define IB_MR_PAGESIZE_128K	0x000000040U
#define IB_MR_PAGESIZE_256K	0x000000080U
#define IB_MR_PAGESIZE_512K	0x000000100U
#define IB_MR_PAGESIZE_1M	0x000000200U
#define IB_MR_PAGESIZE_2M	0x000000400U
#define IB_MR_PAGESIZE_4M	0x000000800U
#define IB_MR_PAGESIZE_8M	0x000001000U
#define IB_MR_PAGESIZE_16M	0x000002000U
#define IB_MR_PAGESIZE_32M	0x000004000U
#define IB_MR_PAGESIZE_64M	0x000008000U
#define IB_MR_PAGESIZE_128M	0x000010000U
#define IB_MR_PAGESIZE_256M	0x000020000U
#define IB_MR_PAGESIZE_512M	0x000040000U
#define IB_MR_PAGESIZE_1G	0x000080000U
#define IB_MR_PAGESIZE_2G	0x000100000U
#define IB_MR_PAGESIZE_4G	0x000200000U
#define IB_MR_PAGESIZE_8G	0x000400000U
#define IB_MR_PAGESIZE_16G	0x000800000U
#define IB_MR_PAGESIZE_32G	0x001000000U
#define IB_MR_PAGESIZE_64G	0x002000000U
#define IB_MR_PAGESIZE_128G	0x004000000U
#define IB_MR_PAGESIZE_256G	0x008000000U
#define IB_MR_PAGESIZE_512G	0x010000000U
#define IB_MR_PAGESIZE_1T	0x020000000U
#define IB_MR_PAGESIZE_2T	0x040000000U
#define IB_MR_PAGESIZE_4T	0x080000000U

typedef uint32 IB_MR_PAGESIZES;

typedef enum _IB_CA_CAPABILITIES {
	CA_IS_SYSTEM_IMAGE_GUID_SUPPORTED		= 0x00000001U,
	CA_IS_INIT_TYPE_SUPPORTED				= 0x00000002U,	/* TBD ReInit? */
	CA_IS_PORT_ACTIVE_EVENT_SUPPORTED		= 0x00000004U,
	CA_IS_RNR_NAK_SUPPORTED					= 0x00000008U,
	CA_IS_AV_PORT_CHECK_SUPPORTED			= 0x00000010U,
	CA_IS_RC_PORT_CHANGE_SUPPORTED			= 0x00000020U,
	CA_IS_QP_MODIFY_NO_TRANSITION_SUPPORTED	= 0x00000040U,
	CA_IS_PKEY_VIOLATION_COUNTER_SUPPORTED	= 0x00000080U,
	CA_IS_QKEY_VIOLATION_COUNTER_SUPPORTED	= 0x00000100U,
	CA_IS_MODIFY_QDEPTH_SUPPORTED			= 0x00000200U,
	CA_IS_RAW_MULTICAST_SUPPORTED			= 0x00000400U,
	CA_IS_PORT_SHUTDOWN_SUPPORTED			= 0x00000800U,
    CA_IS_ENHANCED_SWITCH_PORT0             = 0x00001000U,
    CA_IS_UNSIGNALED_RECV_SUPPORTED         = 0x00002000U,
		/* AV not used by hardware after iba_post_send */
    CA_IS_AV_SOFT			         		= 0x00004000U
} IB_CA_CAPABILITIES;

/*
 * Channel Adapter Attributes
 */
typedef struct _IB_CA_ATTRIBUTES {
	EUI64		GUID;
	EUI64		SystemImageGuid;
	uint32		VendorId;
	uint16		DeviceId;
	uint16		DeviceRevision;
	uint32		QueuePairs;			/* Number of QPs */
	uint32		WorkReqs;			/* Max outstanding WRs permitted on any WQ */
	uint32		DSListDepth;		/* Max depth of scatter/gather list on a WR */
	uint32		RDDSListDepth;		/* Max depth of scatter list on an RD RQ WR */
	uint32		CompletionQueues;	/* Number of CQs */
	uint32		WorkCompletions;	/* Max number of WC/CQE per CQ */
	uint32		MemoryRegions;		/* Max number of Memory Regions */
	uint8		NodeType;
	uint8		Reserved10;
	uint16		Reserved11;
	uint64		MemRegionSize;		/* Max size of a Memory Region */
	uint32		MemoryWindows;		/* Max number of Memory Windows */
	uint32		ProtectionDomains;	/* Number of Protection Domains */
	uint32		RDDomains;			/* Number of Reliable Datagram Domains */
	IB_MR_PAGESIZES	MemRegionPageSizes;	/* The page sizes supported by the CA */
										/* memory management subsystem. */
	uint32		Ports;				/* Number of ports */
	uint32		Partitions;			/* Maximum number of partitions */
	uint32		MaxMTU;				/* smallest of MaxMtu values for */
									/* all ports on CA */
									/* represented in bytes (256, 512, ... 4096) */
	uint64		MaxMessageLen;		/* smallest of MaxMessageLen for all */
									/* ports on CA */
	IB_CA_CAPABILITIES	Capabilities;	/* optional capabilities */
	IB_CA_ATOMIC_LVL	AtomicityLevel;	/* Support level for atomic operations */
	uint8		LocalCaAckDelay;	/* time for local CA to respond to */
									/* an inbound transport layer ACK or NAK */
									/* in 4.096*2^n usec units */
	uint8		MaxQPInitiatorDepth;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per QP as a requestor/initiator */
	uint8		MaxEECInitiatorDepth;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per EEC as a requestor/initiator */
	uint8		MaxQPResponderResources;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per QP as a responder */
	uint8		MaxEECResponderResources;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per EEC as a responder */
	uint8		Reserved1;
	uint16		MaxRearmNumWC;		/* Maximum value for NumWC argument */
									/* in iba_rearm_n_cq.  If this value is 1 */
									/* then the CA does not support coallesing */
									/* completions and iba_rearm_n_cq is */
									/* functionally the same as */
									/* iba_rearm_cq(...,CQEventSelNextWC) */
	uint32		MaxCAResponderResources;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per CA as a responder */
	uint32		EEContexts;			/* Max number of EE contexts */
	uint32		IPv6QPs;			/* Max number of Raw IPv6 Datagram QPs */
	uint32		EthertypeQPs;		/* Max number of Raw Ethertype Datagram QPs */
	uint32		MCastGroups;		/* Max number of multicast groups */
	uint32		MCastQPs;			/* Max number of QPs which may be attached */
									/* to multicast groups. */
	uint32		MCastGroupQPs;		/* Max number of QPs per multicast group */
	uint32		AddressVectors;		/* Max number of AVs which may be created */
	IB_CA_PATH_MIGRATION_LVL	PathMigrationLevel;	/* Support level for */
											/* failover/migration mechanisms. */
	/* The PortAttributesList may be null. If not null the size in bytes of */
	/* the buffer pointed to by PortAttributesList must be given in */
	/* PortAttributeListSize. If PortAttributesList is null, the size in bytes */
	/* of the area needed to hold the attributes information for all ports is */
	/* returned. */
	uint32				PortAttributesListSize;
	IB_PORT_ATTRIBUTES	*PortAttributesList;
} IB_CA_ATTRIBUTES;

/*
 * Update of IB_CA_ATTRIBUTES. NOTE: The structure is identical to 
 * IB_CA_ATTRIBUTES, but adds new fields to the end.
 */
typedef struct _IB_CA_ATTRIBUTES2 {
	EUI64		GUID;
	EUI64		SystemImageGuid;
	uint32		VendorId;
	uint16		DeviceId;
	uint16		DeviceRevision;
	uint32		QueuePairs;			/* Number of QPs */
	uint32		WorkReqs;			/* Max outstanding WRs permitted on any WQ */
	uint32		DSListDepth;		/* Max depth of scatter/gather list on a WR */
	uint32		RDDSListDepth;		/* Max depth of scatter list on an RD RQ WR */
	uint32		CompletionQueues;	/* Number of CQs */
	uint32		WorkCompletions;	/* Max number of WC/CQE per CQ */
	uint32		MemoryRegions;		/* Max number of Memory Regions */
	uint8		NodeType;
	uint8		Reserved10;
	uint16		Reserved11;
	uint64		MemRegionSize;		/* Max size of a Memory Region */
	uint32		MemoryWindows;		/* Max number of Memory Windows */
	uint32		ProtectionDomains;	/* Number of Protection Domains */
	uint32		RDDomains;			/* Number of Reliable Datagram Domains */
	IB_MR_PAGESIZES	MemRegionPageSizes;	/* The page sizes supported by the CA */
										/* memory management subsystem. */
	uint32		Ports;				/* Number of ports */
	uint32		Partitions;			/* Maximum number of partitions */
	uint32		MaxMTU;				/* smallest of MaxMtu values for */
									/* all ports on CA */
									/* represented in bytes (256, 512, ... 4096) */
	uint64		MaxMessageLen;		/* smallest of MaxMessageLen for all */
									/* ports on CA */
	IB_CA_CAPABILITIES	Capabilities;	/* optional capabilities */
	IB_CA_ATOMIC_LVL	AtomicityLevel;	/* Support level for atomic operations */
	uint8		LocalCaAckDelay;	/* time for local CA to respond to */
									/* an inbound transport layer ACK or NAK */
									/* in 4.096*2^n usec units */
	uint8		MaxQPInitiatorDepth;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per QP as a requestor/initiator */
	uint8		MaxEECInitiatorDepth;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per EEC as a requestor/initiator */
	uint8		MaxQPResponderResources;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per QP as a responder */
	uint8		MaxEECResponderResources;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per EEC as a responder */
	uint8		Reserved1;
	uint16		MaxRearmNumWC;		/* Maximum value for NumWC argument */
									/* in iba_rearm_n_cq.  If this value is 1 */
									/* then the CA does not support coallesing */
									/* completions and iba_rearm_n_cq is */
									/* functionally the same as */
									/* iba_rearm_cq(...,CQEventSelNextWC) */
	uint32		MaxCAResponderResources;	/* Maximum number of RDMA reads */
									/* & Atomic Ops which can be outstanding */
									/* per CA as a responder */
	uint32		EEContexts;			/* Max number of EE contexts */
	uint32		IPv6QPs;			/* Max number of Raw IPv6 Datagram QPs */
	uint32		EthertypeQPs;		/* Max number of Raw Ethertype Datagram QPs */
	uint32		MCastGroups;		/* Max number of multicast groups */
	uint32		MCastQPs;			/* Max number of QPs which may be attached */
									/* to multicast groups. */
	uint32		MCastGroupQPs;		/* Max number of QPs per multicast group */
	uint32		AddressVectors;		/* Max number of AVs which may be created */
	IB_CA_PATH_MIGRATION_LVL	PathMigrationLevel;	/* Support level for */
											/* failover/migration mechanisms. */
	/* The PortAttributesList may be null. If not null the size in bytes of */
	/* the buffer pointed to by PortAttributesList must be given in */
	/* PortAttributeListSize. If PortAttributesList is null, the size in bytes */
	/* of the area needed to hold the attributes information for all ports is */
	/* returned. */
	uint32		PortAttributesListSize;
	IB_PORT_ATTRIBUTES	*PortAttributesList;

	/* The PciDevice is provided to aid DMA mapping by kernel mode ULPs
	 * Use of this pointer must be done with care.  The pointer cannot
	 * be used after the CA has been closed.  NULL is returned for user space.
	 */
#if defined(VXWORKS)
	struct _PCI_DEVICE*	PciDevice;
#else
	void*		PciDevice;
#endif

	/* Extra attributes: reserved for future expansion. */
	uint64		Reserved2;
	uint64		Reserved3;
	uint64		Reserved4;
	uint64		Reserved5;
	uint64		Reserved6;
	uint64		Reserved7;
	uint64		Reserved8;
	uint64		Reserved9;
} IB_CA_ATTRIBUTES2;

/* version of IbAccess stack itself, used to verify UVCA matches kernel stack
 * version
 */
typedef enum {
	IBT_VERSION_NONE = 0,
	IBT_VERSION_LATEST = 5
} IBT_VERSION;

#ifdef __cplusplus
};
#endif

#endif /* _IBA_IB_TYPES_H_ */
