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

#ifndef _IBA_IB_SA_RECORDS_PRIV_H_
#define _IBA_IB_SA_RECORDS_PRIV_H_

#include "iba/ib_sm_priv.h"
#include "iba/ib_sa_records.h"
#include "iba/stl_helper.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iba/public/ipackon.h"

static __inline void
BSWAP_IB_NODE_RECORD(
    IB_NODE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
    BSWAP_NODE_INFO(&Dest->NodeInfoData);
#endif /* CPU_LE */
}

static __inline void
BSWAP_IB_PORTINFO_RECORD(
    IB_PORTINFO_RECORD  *Dest, int extended
    )
{
#if CPU_LE

	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
    BSWAP_PORT_INFO(&Dest->PortInfoData, extended);
#endif
}

/* --------------------------------------------------------------------------
 * Switch Info Record - information about a switch in the fabric
 */

/* ComponentMask bits */
#define IB_SWITCHINFO_RECORD_COMP_LID								0x00000001
	/* reserved field												0x00000002*/
	/* switch info fields */
#define IB_SWITCHINFO_RECORD_COMP_LINEARFDBCAP						0x00000004
#define IB_SWITCHINFO_RECORD_COMP_RANDOMFDBCAP						0x00000008
#define IB_SWITCHINFO_RECORD_COMP_MULTICASTFDBCAP					0x00000010
#define IB_SWITCHINFO_RECORD_COMP_LINEARFDBTOP						0x00000020
#define IB_SWITCHINFO_RECORD_COMP_DEFAULTPORT						0x00000040
#define IB_SWITCHINFO_RECORD_COMP_DEFAULTMULTICASTPRIMARYPORT		0x00000080
#define IB_SWITCHINFO_RECORD_COMP_DEFAULTMULTICASTNOTPRIMARYPORT	0x00000100
#define IB_SWITCHINFO_RECORD_COMP_LIFETIMEVALUE						0x00000200
#define IB_SWITCHINFO_RECORD_COMP_PORTSTATECHANGE					0x00000400
	/* reserved field												0x00000800*/
#define IB_SWITCHINFO_RECORD_COMP_LIDSPERPORT						0x00001000
#define IB_SWITCHINFO_RECORD_COMP_PARTITIONENFORCEMENTCAP			0x00002000
#define IB_SWITCHINFO_RECORD_COMP_INBOUNDENFORCEMENTCAP				0x00004000
#define IB_SWITCHINFO_RECORD_COMP_OUTBOUNDENFORCEMENTCAP			0x00008000
#define IB_SWITCHINFO_RECORD_COMP_FILTERRAWINBOUNDCAP				0x00010000
#define IB_SWITCHINFO_RECORD_COMP_FILTERRAWOUTBOUNDCAP				0x00020000
#define IB_SWITCHINFO_RECORD_COMP_ENHANCEDPORT0						0x00040000
	/* reserved field												0x00080000*/

typedef struct _IB_SWITCHINFO_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Reserved:16;
#else
			uint32		Reserved:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	SWITCH_INFO		SwitchInfoData;
} PACK_SUFFIX IB_SWITCHINFO_RECORD;

static __inline void
BSWAP_IB_SWITCHINFO_RECORD(
    IB_SWITCHINFO_RECORD  *Dest
    )
{
#if CPU_LE

	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_SWITCH_INFO(&Dest->SwitchInfoData);
#endif
}

/* --------------------------------------------------------------------------
 * Linear Forwarding Table Record - linear forwarding table for a switch in the fabric
 */

/* ComponentMask bits */
#define IB_LINEARFDB_RECORD_COMP_LID						0x00000001
#define IB_LINEARFDB_RECORD_COMP_BLOCKNUM					0x00000002
	/* reserved field 										0x00000004 */
	/* linear forwarding table fields */
	/* Note insufficient bits in component mask to select */
	/* all 64 entries in record */

typedef struct _IB_LINEAR_FDB_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		BlockNum:16;
#else
			uint32		BlockNum:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	FORWARDING_TABLE	LinearFdbData;
} PACK_SUFFIX IB_LINEAR_FDB_RECORD;

static __inline void
BSWAP_IB_LINEAR_FDB_RECORD(
    IB_LINEAR_FDB_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_LINEAR_FWD_TABLE(&Dest->LinearFdbData);
#endif
}

/* --------------------------------------------------------------------------
 * Random Forwarding Table Record - random forwarding table for a switch in the fabric
 */

/* ComponentMask bits */
#define IB_RANDOMFDB_RECORD_COMP_LID						0x00000001
#define IB_RANDOMFDB_RECORD_COMP_BLOCKNUM					0x00000002
	/* reserved field 										0x00000004 */
	/* random forwarding table fields */
	/* Note insufficient bits in component mask to select */
	/* all 5 fields in all 16 entries in record */

typedef struct _IB_RANDOM_FDB_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		BlockNum:16;
#else
			uint32		BlockNum:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	FORWARDING_TABLE	RandomFdbData;
} PACK_SUFFIX IB_RANDOM_FDB_RECORD;

static __inline void
BSWAP_IB_RANDOM_FDB_RECORD(
    IB_RANDOM_FDB_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_RANDOM_FWD_TABLE(&Dest->RandomFdbData);
#endif
}

/* --------------------------------------------------------------------------
 * Multicast Forwarding Table Record - multicast forwarding table for a
 * switch in the fabric
 */

/* ComponentMask bits */
#define IB_MCASTFDB_RECORD_COMP_LID							0x00000001
#define IB_MCASTFDB_RECORD_COMP_POSITION					0x00000002
	/* reserved field 										0x00000004 */
#define IB_MCASTFDB_RECORD_COMP_BLOCKNUM					0x00000008
	/* reserved field 										0x00000010 */
	/* multicast forwarding table fields */
	/* limited value to select on these, so omitted defines */

typedef struct _IB_MCAST_FDB_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Position:4;
			uint32		Reserved0:3;
			uint32		BlockNum:9;
#else
			uint32		BlockNum:9;
			uint32		Reserved0:3;
			uint32		Position:4;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	FORWARDING_TABLE	MCastFdbData;
} PACK_SUFFIX IB_MCAST_FDB_RECORD;

static __inline void
BSWAP_IB_MCAST_FDB_RECORD(
    IB_MCAST_FDB_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_MCAST_FWD_TABLE(&Dest->MCastFdbData);
#endif
}

/* --------------------------------------------------------------------------
 * VL Arbitration Table Record - VL priority controls for a port on a node in
 * the fabric
 */

/* ComponentMask bits */
#define IB_VLARBTABLE_RECORD_COMP_LID						0x00000001
#define IB_VLARBTABLE_RECORD_COMP_OUTPUTPORTNUM				0x00000002
#define IB_VLARBTABLE_RECORD_COMP_BLOCKNUM					0x00000004
	/* reserved field 										0x00000008 */
	/* Note insufficient bits in component mask to select */
	/* all 3 fields in all 32 entries in record */

typedef struct _IB_VLARBTABLE_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		OutputPortNum:8;
			uint32		BlockNum:8;
#else
			uint32		BlockNum:8;
			uint32		OutputPortNum:8;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	VLARBTABLE		VLArbData;				/* VLArbitration attribute */
} PACK_SUFFIX IB_VLARBTABLE_RECORD;

static __inline void
BSWAP_IB_VLARBTABLE_RECORD(
    IB_VLARBTABLE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_VLARBTABLE(&Dest->VLArbData);
#endif
}

/* --------------------------------------------------------------------------
 * SM Info Record - basic information about an SM in the fabric
 */

/* ComponentMask bits */
#define IB_SMINFO_RECORD_COMP_LID						0x00000001
	/* reserved field									0x00000002 */
	/* SM Info fields */
#define IB_SMINFO_RECORD_COMP_GUID						0x00000004
#define IB_SMINFO_RECORD_COMP_SMKEY						0x00000008
#define IB_SMINFO_RECORD_COMP_ACTCOUNT					0x00000010
#define IB_SMINFO_RECORD_COMP_PRIORITY					0x00000020
#define IB_SMINFO_RECORD_COMP_SMSTATE					0x00000040

typedef struct _IB_SMINFO_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Reserved:16;
#else
			uint32		Reserved:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	SM_INFO			SMInfoData;				/* SMInfo attribute */
} PACK_SUFFIX IB_SMINFO_RECORD;

static __inline void
BSWAP_IB_SMINFO_RECORD(
    IB_SMINFO_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
    BSWAP_SM_INFO(&Dest->SMInfoData);
#endif
}

/* --------------------------------------------------------------------------
 * P_Key Table Record - P-Key configuration for a port on a node in the fabric
 */

/* ComponentMask bits */
#define IB_PKEYTABLE_RECORD_COMP_LID						0x00000001
#define IB_PKEYTABLE_RECORD_COMP_BLOCKNUM					0x00000002
#define IB_PKEYTABLE_RECORD_COMP_PORTNUM					0x00000004
	/* reserved field										0x00000008 */
	/* P Key Table fields */
	/* limited value to select on these, so omitted defines */

typedef struct _IB_P_KEY_TABLE_RECORD {
	union {
		struct {
			uint32	AsReg32;
			uint8	Byte;
		} PACK_SUFFIX s2;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		BlockNum:16;
#else
			uint32		BlockNum:16;
			uint32		LID:16;
#endif
			uint8		PortNum;
		} PACK_SUFFIX s;
	} PACK_SUFFIX RID;
	uint8		Reserved[3];
	PARTITION_TABLE	PKeyTblData;			/* PartitionTable for this port */
} PACK_SUFFIX IB_P_KEY_TABLE_RECORD;

static __inline void
BSWAP_IB_P_KEY_TABLE_RECORD(
    IB_P_KEY_TABLE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.s2.AsReg32 = ntoh32(Dest->RID.s2.AsReg32);
    BSWAP_PART_TABLE(&Dest->PKeyTblData);
#endif
}

static __inline void
BSWAP_IB_INFORM_INFO_RECORD(
    IB_INFORM_INFO_RECORD  *Dest
    )
{
#if CPU_LE
	BSWAP_IB_GID(&Dest->RID.SubscriberGID);
	Dest->RID.Enum = ntoh16(Dest->RID.Enum);
    BSWAP_INFORM_INFO(&Dest->InformInfoData);
#endif
}

/* --------------------------------------------------------------------------
 * Link Record - details about a link in the fabric
 */

/* ComponentMask bits */
#define IB_LINK_RECORD_COMP_FROMLID							0x00000001
#define IB_LINK_RECORD_COMP_FROMPORT						0x00000002
#define IB_LINK_RECORD_COMP_TOPORT							0x00000004
#define IB_LINK_RECORD_COMP_TOLID							0x00000008

typedef struct _IB_LINK_RECORD {
	struct {
		uint16			FromLID;			/* From this LID */
		uint8			FromPort;				/* From port number */
	} PACK_SUFFIX RID;
	uint8			ToPort;					/* To port number */
	uint16			ToLID;				/* To this LID */
} PACK_SUFFIX IB_LINK_RECORD;

static __inline void
BSWAP_IB_LINK_RECORD(
    IB_LINK_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.FromLID = ntoh16(Dest->RID.FromLID);
	Dest->ToLID = ntoh16(Dest->ToLID);
#endif /* CPU_LE */
}

static __inline void
BSWAP_IB_SERVICE_RECORD(
    IB_SERVICE_RECORD  *Dest
    )
{
#if CPU_LE
	uint8 i;

	Dest->RID.ServiceID = ntoh64(Dest->RID.ServiceID);
	BSWAP_IB_GID(&Dest->RID.ServiceGID);
	Dest->RID.ServiceP_Key = ntoh16(Dest->RID.ServiceP_Key);
	Dest->ServiceLease = ntoh32(Dest->ServiceLease);
	ntoh(&Dest->ServiceKey[0], &Dest->ServiceKey[0], 16);

	for (i=0; i<8; ++i)
	{
		Dest->ServiceData16[i] = ntoh16(Dest->ServiceData16[i]);
	}
	for (i=0; i<4; ++i)
	{
		Dest->ServiceData32[i] = ntoh32(Dest->ServiceData32[i]);
	}
	for (i=0; i<2; ++i)
	{
		Dest->ServiceData64[i] = ntoh64(Dest->ServiceData64[i]);
	}
#endif /* CPU_LE */
}

static __inline void
BSWAP_IB_MCMEMBER_RECORD(
    IB_MCMEMBER_RECORD  *Dest
    )
{
#if CPU_LE
	BSWAP_IB_GID(&Dest->RID.MGID);
	BSWAP_IB_GID(&Dest->RID.PortGID);
	Dest->Q_Key = ntoh32(Dest->Q_Key);
	Dest->MLID = ntoh16(Dest->MLID);
	Dest->P_Key = ntoh16(Dest->P_Key);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
#endif /* CPU_LE */
}

static __inline void
BSWAPCOPY_IB_MCMEMBER_RECORD(IB_MCMEMBER_RECORD *Src, IB_MCMEMBER_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(IB_MCMEMBER_RECORD));
	BSWAP_IB_MCMEMBER_RECORD(Dest);
}

static __inline void
BSWAP_IB_TRACE_RECORD(
    IB_TRACE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->GIDPrefix = ntoh64(Dest->GIDPrefix);
	Dest->IDGeneration = ntoh16(Dest->IDGeneration);
	Dest->NodeID = ntoh64(Dest->NodeID);
	Dest->ChassisID = ntoh64(Dest->ChassisID);
	Dest->EntryPortID = ntoh64(Dest->EntryPortID);
	Dest->ExitPortID = ntoh64(Dest->ExitPortID);
#endif
}

static __inline void
BSWAP_IB_MULTIPATH_RECORD(
    IB_MULTIPATH_RECORD  *Dest
    )
{
#if CPU_LE
	uint8 i;
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	Dest->P_Key = ntoh16(Dest->P_Key);
	Dest->u2.AsReg16 = ntoh16(Dest->u2.AsReg16);

	/* TBD - sanity check Counts */
	for (i=0; i<(Dest->SGIDCount + Dest->DGIDCount); ++i)
	{
		BSWAP_IB_GID(&Dest->GIDList[i]);
	}
#endif
}

static __inline void
BSWAP_IB_PATH_RECORD(
    IB_PATH_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->ServiceID = ntoh64(Dest->ServiceID);
    BSWAP_IB_GID(&Dest->DGID);
    BSWAP_IB_GID(&Dest->SGID);
 
    Dest->DLID =   ntoh16(Dest->DLID);
    Dest->SLID =   ntoh16(Dest->SLID);
	Dest->u1.AsReg32 =	ntoh32(Dest->u1.AsReg32);
    Dest->P_Key =  ntoh16(Dest->P_Key);
    Dest->u2.AsReg16 =  ntoh16(Dest->u2.AsReg16);
#endif /* CPU_LE */
}

static __inline void
BSWAPCOPY_IB_PATH_RECORD(
    IB_PATH_RECORD  *Src, IB_PATH_RECORD  *Dest
    )
{
    memcpy(Dest, Src, sizeof(IB_PATH_RECORD));
    BSWAP_IB_PATH_RECORD(Dest);
}

/* determine if the given PATH_RECORD represents a global route
 * (eg. where GRH is used to go through an IB Router)
 */
static __inline boolean
IsGlobalRoute( IN const IB_PATH_RECORD *pPathRecord)
{
	return (pPathRecord->u1.s.HopLimit > 1);
}

/* compute LocalAckTimeout from PktLifeTime
 * note PktLifetime is one directional on wire, while LocalAckTimeout is
 * total round trip including CA Ack Delay
 * for client REQ.AckTimeout, caAckDelay should be our local CA's AckDelay
 * for client QP, use REP.TargetAckDelay
 * for server QP, use REQ.AckTimeout directly (no need to call this)
 * pktLifeTime, caAckDelay and returned values are IB timeout multipliers
 */
static __inline uint8
ComputeAckTimeout(IN uint8 pktLifeTime, IN uint8 caAckDelay)
{
    /* return TimeoutTimeToMult(TimeoutMultToTimeInUsec(pktLifeTime)*2 */
	/* 				+ TimeoutMultToTimeInUsec(caAckDelay)); */
  	/*return MIN(pktLifeTime + 2, 0x1f); */
	uint8 ackTimeout;

	/* tests also handle if remote endpoint didn't set REP.TargetAckDelay */
	if (pktLifeTime+1 >= caAckDelay)
	{
 		/* since its a log2 value, +1 doubles the timeout */
 		/* Additional +1 is to account for remote CA Ack delay */
		ackTimeout = pktLifeTime+2;
	} else {
		/* since caAckDelay > 2*pktLifetime, +1 simply doubles ca Ack Delay */
		ackTimeout = caAckDelay+1;
	}
	/* limit value to 5 bits (approx 2.4 hours) */
	if (ackTimeout > 0x1f)
		ackTimeout = 0x1f;
	return ackTimeout;
}

#ifndef IB_STACK_OPENIB
/* helper function, converts a PATH_RECORD to an IB_ADDRESS_VECTOR to facilitate
 * connection establishment and UD traffic
 * PathMTU is an IB_MTU enum
 */
static __inline void
GetAVFromPath2( IN uint64 PortGuid,	/* only needed for UD AVs */
	IN const IB_PATH_RECORD *pPathRecord,
	OUT uint8* PathMTU OPTIONAL,
	OUT IB_ADDRESS_VECTOR* DestAv OPTIONAL )
{
	if (DestAv)
	{
		DestAv->PortGUID = PortGuid; /* only needed for UD AVs */
		DestAv->DestLID = pPathRecord->DLID;
		DestAv->PathBits = (uint8)(pPathRecord->SLID&IB_PATHBITS_MASK);
		DestAv->ServiceLevel = pPathRecord->u2.s.SL;
		DestAv->StaticRate = pPathRecord->Rate;

		/* Global route information. */
		DestAv->GlobalRoute = IsGlobalRoute(pPathRecord);
		DestAv->GlobalRouteInfo.DestGID = pPathRecord->DGID;
		DestAv->GlobalRouteInfo.FlowLabel = pPathRecord->u1.s.FlowLabel;
		DestAv->GlobalRouteInfo.HopLimit = (uint8)pPathRecord->u1.s.HopLimit;
		DestAv->GlobalRouteInfo.SrcGIDIndex = 0;	/* BUGBUG assume 0 */
		DestAv->GlobalRouteInfo.TrafficClass = pPathRecord->TClass;
	}

	/* Reliable connection information, N/A for UD */
	if (PathMTU)
		*PathMTU = pPathRecord->Mtu;
}

/* This function is depricated, will be dropped in 3.1 release,
 * use GetAVFromPath2 above
 */
static __inline void
GetAVFromPath( IN uint64 PortGuid,	/* only needed for UD AVs */
	IN const IB_PATH_RECORD *pPathRecord,
	OUT uint8* PathMTU OPTIONAL, OUT uint8* LocalAckTimeout OPTIONAL,
	OUT IB_ADDRESS_VECTOR* DestAv OPTIONAL )
{
	GetAVFromPath2(PortGuid, pPathRecord, PathMTU, DestAv);
	if (LocalAckTimeout)
		*LocalAckTimeout = ComputeAckTimeout(pPathRecord->PktLifeTime,
								TimeoutTimeToMult(8));	/* guess: 8 ms */
}

/* helper function, converts a MCMEMBER_RECORD to an IB_ADDRESS_VECTOR to
 * facilitate UD traffic
 */
static __inline void
GetAVFromMcMemberRecord(EUI64              PortGuid,
                        IB_MCMEMBER_RECORD *pMcMemberRecord,
                        IB_ADDRESS_VECTOR  *pDestAv )
{
	if (pDestAv != NULL)
	{
		pDestAv->PortGUID = pMcMemberRecord->RID.PortGID.Type.Global.InterfaceID;
		pDestAv->DestLID = pMcMemberRecord->MLID;
/*		pDestAv->PathBits = (uint8)pPathRecord->SLID; */
		pDestAv->PathBits = 0;
		pDestAv->ServiceLevel = pMcMemberRecord->u1.s.SL;
		pDestAv->StaticRate = pMcMemberRecord->Rate;

		/* Global route information. */
		pDestAv->GlobalRoute = TRUE;
		pDestAv->GlobalRouteInfo.DestGID = pMcMemberRecord->RID.MGID;
		pDestAv->GlobalRouteInfo.FlowLabel = pMcMemberRecord->u1.s.FlowLabel;
		pDestAv->GlobalRouteInfo.HopLimit = (uint8)pMcMemberRecord->u1.s.HopLimit;
		pDestAv->GlobalRouteInfo.SrcGIDIndex = 0;	/* BUGBUG assume 0 */
		pDestAv->GlobalRouteInfo.TrafficClass = pMcMemberRecord->TClass;
	}
}

#endif /* IB_STACK_OPENIB */

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_SA_RECORDS_PRIV_H_ */
