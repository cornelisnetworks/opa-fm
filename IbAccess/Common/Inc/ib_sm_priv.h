/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2017, Intel Corporation

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

#ifndef __IBA_IB_SM_PRIV_H__
#define __IBA_IB_SM_PRIV_H__

#include "iba/ib_sm_types.h"

#if defined (__cplusplus)
extern "C" {
#endif

#include "iba/public/ipackon.h"

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

/* convert SM State to text */
static __inline const char*
IbSMStateToText(SM_STATE state)
{
	return (state == SM_INACTIVE? "Inactive":
			state == SM_DISCOVERING? "Discovering":
			state == SM_STANDBY? "Standby":
			state == SM_MASTER? "Master": "???");
}

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

#define PGTABLE_LIST_COUNT 64

typedef struct _PORT_GROUP_TABLE {
	uint8		PortGroup[PGTABLE_LIST_COUNT];		/* List of PortGroups indexed by port */
} PACK_SUFFIX PORT_GROUP_TABLE;

#define LIDMASK_LEN 64

typedef struct _ADAPTIVE_ROUTING_LIDMASK {
	uint8		lidmask[LIDMASK_LEN];           // lidmask block of 512 lids
} PACK_SUFFIX ADAPTIVE_ROUTING_LIDMASK;

#include "iba/public/ipackoff.h"

static __inline void
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

static __inline void
BSWAP_NODE_DESCRIPTION(
	NODE_DESCRIPTION			*Dest
	)
{
	/* pure text field, nothing to swap */
}


static __inline void
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
static __inline void
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


static __inline void
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


static __inline void
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

static __inline void
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

static __inline void
BSWAP_VLARBTABLE(
	VLARBTABLE			*Dest
	)
{
	/* we laid this out in network byte order for intel compilers */
}

static __inline void
BSWAP_LINEAR_FWD_TABLE(
    FORWARDING_TABLE            *Dest
    )
{
#if CPU_LE
	/* nothing to do, PORT is only 1 byte */
#endif
}

static __inline void
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

static __inline void
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

static __inline void
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

static __inline void
BSWAP_VENDOR_DIAG(
	VENDOR_DIAG			*Dest
	)
{
#if CPU_LE
	Dest->u.NextIndex = ntoh32(Dest->u.NextIndex);
#endif
}

static __inline void
BSWAP_PORT_GROUP_TABLE(
	PORT_GROUP_TABLE	*Dest
	)
{
}

static __inline void
BSWAP_ADAPTIVE_ROUTING_LIDMASK(
	ADAPTIVE_ROUTING_LIDMASK	*Dest
	)
{
}

#if defined (__cplusplus)
};
#endif

#endif /* __IBA_IB_SM_PRIV_H__ */
