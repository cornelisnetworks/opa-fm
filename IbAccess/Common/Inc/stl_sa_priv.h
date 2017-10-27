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

#ifndef __STL_SA_PRIV_H__
#define __STL_SA_PRIV_H__

#include "iba/stl_mad_priv.h"
#include "iba/stl_sa_types.h"
#include "iba/ib_sa_records_priv.h"
#include "iba/stl_sm_priv.h"

#if defined (__cplusplus)
extern "C" {
#endif

// Replaces old Vieo macro that converts a single GID argument into two arguments
// that can be used in a print statement.
#define STLGIDPRINTARGS(GID) ntoh64((GID).Type.Global.SubnetPrefix), \
	ntoh64((GID).Type.Global.InterfaceID)

#define STLGIDPRINTARGS2(GID) ntoh64(alignArray64(&GID[0])), ntoh64(alignArray64(&GID[8]))

static __inline__ uint64_t
_alignArray64(uint8_t * array) { uint64_t rv; memcpy(&rv, array, sizeof(rv));return rv;}

#include "iba/public/ipackon.h"

/*
 * SA Header
 */
typedef	struct {
	uint8_t     rmppVersion;
	uint8_t     rmppType;
	union {
		struct {
			#if CPU_BE
				uint8_t  rmppRespTime : 5;
				uint8_t  rmppFlags : 3;
			#else
				uint8_t  rmppFlags : 3;
				uint8_t  rmppRespTime : 5;
			#endif
		} tf;
		uint8_t timeFlag;
	} u;
	uint8_t     rmppStatus;
	uint32_t    segNum;
	uint32_t    length;
	uint64_t	smKey;
	uint16_t    offset;
	uint16_t    rservd;
	uint64_t	mask;
} PACK_SUFFIX STL_SA_MAD_HEADER;

#define STL_SA_DATA_LEN	(STL_MAD_BLOCK_SIZE-sizeof(STL_SA_MAD_HEADER)-sizeof(MAD_COMMON))

typedef struct {
	STL_SA_MAD_HEADER	header;
	uint8_t		data[STL_SA_DATA_LEN];
} PACK_SUFFIX STL_SA_MAD;

static __inline void
BSWAPCOPY_STL_SA_MAD_HEADER(STL_SA_MAD_HEADER *src, STL_SA_MAD_HEADER *dst) 
{
	dst->rmppVersion = src->rmppVersion;
	dst->rmppType = src->rmppType;
	dst->rmppStatus = src->rmppStatus;
	dst->u.timeFlag = src->u.timeFlag;

	dst->segNum = ntoh32(src->segNum);
	dst->length = ntoh32(src->length);
	dst->smKey = ntoh64(src->smKey);
	dst->offset = ntoh16(src->offset);
	dst->rservd = ntoh16(src->rservd);
	dst->mask = ntoh64(src->mask);
}

static __inline void
BSWAPCOPY_STL_SA_MAD(STL_SA_MAD *src, STL_SA_MAD *dst, size_t len) 
{
	BSWAPCOPY_STL_SA_MAD_HEADER(&src->header,&dst->header);
	memcpy(&dst->data, &src->data, len);
}

static __inline void
BSWAP_STL_NODE_RECORD(STL_NODE_RECORD  *Dest)
{
    Dest->RID.LID = ntoh32(Dest->RID.LID);
    BSWAP_STL_NODE_INFO(&Dest->NodeInfo);
}

static __inline void
BSWAPCOPY_STL_NODE_RECORD(STL_NODE_RECORD *Src, STL_NODE_RECORD  *Dest)
{
    Dest->RID.LID = ntoh32(Src->RID.LID);
    BSWAPCOPY_STL_NODE_INFO(&Src->NodeInfo, &Dest->NodeInfo);
	memcpy(&(Dest->NodeDesc), &(Src->NodeDesc), sizeof(STL_NODE_DESCRIPTION));
}

static __inline void
BSWAPCOPY_IB_NODE_RECORD(IB_NODE_RECORD *Src, IB_NODE_RECORD  *Dest)
{
    Dest->RID.AsReg32 = ntoh32(Src->RID.AsReg32);
	memcpy(&Dest->NodeInfoData,&Src->NodeInfoData,sizeof(NODE_INFO));
    BSWAP_NODE_INFO(&Dest->NodeInfoData);
	memcpy(&(Dest->NodeDescData), &(Src->NodeDescData), sizeof(NODE_DESCRIPTION));
}

static __inline void
BSWAP_STL_PORTINFO_RECORD(STL_PORTINFO_RECORD  *Dest)
{
#if CPU_LE
	int i;

    Dest->RID.EndPortLID = ntoh32(Dest->RID.EndPortLID);
    BSWAP_STL_PORT_INFO(&Dest->PortInfo);

	for(i = 0; i < STL_NUM_LINKDOWN_REASONS; i++){
		Dest->LinkDownReasons[i].Timestamp = ntoh64(Dest->LinkDownReasons[i].Timestamp);
	}
#endif
}

static __inline void
BSWAP_STL_PARTITION_TABLE_RECORD(STL_P_KEY_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	Dest->RID.Blocknum = ntoh16(Dest->RID.Blocknum);
	BSWAP_STL_PARTITION_TABLE(&Dest->PKeyTblData);
}

static __inline void
BSWAPCOPY_STL_PARTITION_TABLE_RECORD(STL_P_KEY_TABLE_RECORD *Src, STL_P_KEY_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_P_KEY_TABLE_RECORD));
	BSWAP_STL_PARTITION_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_SC_MAPPING_TABLE_RECORD(STL_SC_MAPPING_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
}

static __inline void
BSWAPCOPY_STL_SC_MAPPING_TABLE_RECORD(STL_SC_MAPPING_TABLE_RECORD *Src, STL_SC_MAPPING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_SC_MAPPING_TABLE_RECORD));
	BSWAP_STL_SC_MAPPING_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_SL2SC_MAPPING_TABLE_RECORD(STL_SL2SC_MAPPING_TABLE_RECORD *Dest)
{
	Dest->RID.LID=ntoh32(Dest->RID.LID);
}

static __inline void
BSWAPCOPY_STL_SL2SC_MAPPING_TABLE_RECORD(STL_SL2SC_MAPPING_TABLE_RECORD *Src, STL_SL2SC_MAPPING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_SL2SC_MAPPING_TABLE_RECORD));
	BSWAP_STL_SL2SC_MAPPING_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_SC2SL_MAPPING_TABLE_RECORD(STL_SC2SL_MAPPING_TABLE_RECORD *Dest)
{
	Dest->RID.LID=ntoh32(Dest->RID.LID);
}

static __inline void
BSWAPCOPY_STL_SC2SL_MAPPING_TABLE_RECORD(STL_SC2SL_MAPPING_TABLE_RECORD *Src, STL_SC2SL_MAPPING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_SC2SL_MAPPING_TABLE_RECORD));
	BSWAP_STL_SC2SL_MAPPING_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_SC2VL_R_MAPPING_TABLE_RECORD(STL_SC2VL_R_MAPPING_TABLE_RECORD *Dest)
{
	Dest->RID.LID=ntoh32(Dest->RID.LID);
}

static __inline void
BSWAPCOPY_STL_SC2VL_R_MAPPING_TABLE_RECORD(STL_SC2VL_R_MAPPING_TABLE_RECORD *Src, STL_SC2VL_R_MAPPING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_SC2VL_R_MAPPING_TABLE_RECORD));
	BSWAP_STL_SC2VL_R_MAPPING_TABLE_RECORD(Dest);
}

static __inline void
BSWAPCOPY_STL_SWITCHINFO_RECORD(STL_SWITCHINFO_RECORD *Src, STL_SWITCHINFO_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Src->RID.LID);
	BSWAPCOPY_STL_SWITCH_INFO(&Src->SwitchInfoData, &Dest->SwitchInfoData);
}

static __inline void
BSWAP_STL_SWITCHINFO_RECORD(STL_SWITCHINFO_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	BSWAP_STL_SWITCH_INFO(&Dest->SwitchInfoData);
}

static __inline void
BSWAP_STL_LINEAR_FORWARDING_TABLE_RECORD(STL_LINEAR_FORWARDING_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	(*(uint32*)((char *)Dest + sizeof(uint32))) = ntoh32(*(uint32*)((char *)Dest + sizeof(uint32)));
	BSWAP_STL_LINEAR_FORWARDING_TABLE((STL_LINEAR_FORWARDING_TABLE *)Dest->LinearFdbData);
}

static __inline void
BSWAPCOPY_STL_LINEAR_FORWARDING_TABLE_RECORD(STL_LINEAR_FORWARDING_TABLE_RECORD *Src, STL_LINEAR_FORWARDING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD));
	BSWAP_STL_LINEAR_FORWARDING_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_MCFTB_RECORD(STL_MULTICAST_FORWARDING_TABLE_RECORD *Dest)
{
	int i;
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	Dest->RID.u1.AsReg32 = ntoh32(Dest->RID.u1.AsReg32);
	for (i = 0; i < STL_NUM_MFT_ELEMENTS_BLOCK; ++i) Dest->MftTable.MftBlock[i] = ntoh64(Dest->MftTable.MftBlock[i]);
}

static __inline void
BSWAPCOPY_STL_MCFTB_RECORD(STL_MULTICAST_FORWARDING_TABLE_RECORD *Src, STL_MULTICAST_FORWARDING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD));
	BSWAP_STL_MCFTB_RECORD(Dest);
}

static __inline void
BSWAP_STL_PORT_GROUP_TABLE_RECORD(STL_PORT_GROUP_TABLE_RECORD *Dest)
{
	int i;
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	for (i = 0; i < STL_PGTB_NUM_ENTRIES_PER_BLOCK; ++i) Dest->GroupBlock[i] = ntoh64(Dest->GroupBlock[i]);
}

static __inline void
BSWAPCOPY_STL_PORT_GROUP_TABLE_RECORD(STL_PORT_GROUP_TABLE_RECORD *Src, STL_PORT_GROUP_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_PORT_GROUP_TABLE_RECORD));
	BSWAP_STL_PORT_GROUP_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_PORT_GROUP_FORWARDING_TABLE_RECORD(STL_PORT_GROUP_FORWARDING_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	Dest->RID.u1.AsReg32 = ntoh32(Dest->RID.u1.AsReg32);
}

static __inline void
BSWAPCOPY_STL_PORT_GROUP_FORWARDING_TABLE_RECORD(STL_PORT_GROUP_FORWARDING_TABLE_RECORD *Src, STL_PORT_GROUP_FORWARDING_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_PORT_GROUP_FORWARDING_TABLE_RECORD));
	BSWAP_STL_PORT_GROUP_FORWARDING_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_VLARBTABLE_RECORD(STL_VLARBTABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	BSWAP_STL_VLARB_TABLE(&Dest->VLArbTable, Dest->RID.BlockNum);
}

static __inline void
BSWAPCOPY_STL_VLARBTABLE_RECORD(STL_VLARBTABLE_RECORD *Src, STL_VLARBTABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_VLARBTABLE_RECORD));
	BSWAP_STL_VLARBTABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_MCMEMBER_RECORD(STL_MCMEMBER_RECORD *Dest)
{
	uint64_t temp;
	
	temp = ntoh64(Dest->RID.MGID.AsReg64s.H);
	Dest->RID.MGID.AsReg64s.H = ntoh64(Dest->RID.MGID.AsReg64s.L);
	Dest->RID.MGID.AsReg64s.L = temp;
	
	temp = ntoh64(Dest->RID.PortGID.AsReg64s.H);
	Dest->RID.PortGID.AsReg64s.H = ntoh64(Dest->RID.PortGID.AsReg64s.L);
	Dest->RID.PortGID.AsReg64s.L = temp;

	Dest->Q_Key = ntoh32(Dest->Q_Key);
	Dest->P_Key = ntoh16(Dest->P_Key);
	*(uint32*)((uint8*)Dest + 44) = ntoh32( *(uint32*)( (uint8*)Dest + 44)  );
	Dest->MLID = ntoh32(Dest->MLID);
}

static __inline void
BSWAPCOPY_STL_MCMEMBER_RECORD(STL_MCMEMBER_RECORD *Src, STL_MCMEMBER_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_MCMEMBER_RECORD));
	BSWAP_STL_MCMEMBER_RECORD(Dest);
}

static __inline void
CONVERT_IB2STL_MCMEMBER_RECORD(IB_MCMEMBER_RECORD *Src, STL_MCMEMBER_RECORD *Dest)
{
	Dest->RID.MGID = Src->RID.MGID;
	Dest->RID.PortGID = Src->RID.PortGID;
	Dest->Q_Key = Src->Q_Key;
	Dest->MLID = Src->MLID;
	Dest->MtuSelector = Src->MtuSelector;
	Dest->Mtu = Src->Mtu;
	Dest->TClass = Src->TClass;
	Dest->P_Key = Src->P_Key;
	Dest->RateSelector = Src->RateSelector;
	Dest->Rate = Src->Rate;
	Dest->PktLifeTimeSelector = Src->PktLifeTimeSelector;
	Dest->PktLifeTime = Src->PktLifeTime;
	Dest->SL = Src->u1.s.SL;
	Dest->HopLimit = Src->u1.s.HopLimit;
	Dest->Scope = Src->Scope;
	Dest->JoinFullMember = Src->JoinFullMember;
	Dest->JoinSendOnlyMember = Src->JoinSendOnlyMember;
	Dest->JoinNonMember = Src->JoinNonMember;
	Dest->ProxyJoin = Src->ProxyJoin;
}

static __inline void
CONVERT_STL2IB_MCMEMBER_RECORD(STL_MCMEMBER_RECORD *Src, IB_MCMEMBER_RECORD *Dest)
{
	Dest->RID.MGID = Src->RID.MGID;
	Dest->RID.PortGID = Src->RID.PortGID;
	Dest->Q_Key = Src->Q_Key;
	Dest->MLID = Src->MLID;
	Dest->MtuSelector = Src->MtuSelector;
	Dest->Mtu = Src->Mtu;
	Dest->TClass = Src->TClass;
	Dest->P_Key = Src->P_Key;
	Dest->RateSelector = Src->RateSelector;
	Dest->Rate = Src->Rate;
	Dest->PktLifeTimeSelector = Src->PktLifeTimeSelector;
	Dest->PktLifeTime = Src->PktLifeTime;
	Dest->u1.s.SL = Src->SL;
	Dest->u1.s.FlowLabel = 0;
	Dest->u1.s.HopLimit = Src->HopLimit;
	Dest->Scope = Src->Scope;
	Dest->JoinFullMember = Src->JoinFullMember;
	Dest->JoinSendOnlyMember = Src->JoinSendOnlyMember;
	Dest->JoinNonMember = Src->JoinNonMember;
	Dest->ProxyJoin = Src->ProxyJoin;
	Dest->Reserved = 0;
	Dest->Reserved2 = 0;
	Dest->Reserved3[0] = 0;
	Dest->Reserved3[1] = 0;
}

static __inline uint64
REBUILD_COMPMSK_IB2STL_MCMEMBER_RECORD(uint64 mask)
{
	uint64 result = 0ull;
	uint64 pos;
	uint8 i;

	for (i = 0; i < 64; ++i)
	{
		pos = 1<<i;
		switch (pos)
		{
			case IB_MCMEMBER_RECORD_COMP_MLID:
				result |= STL_MCMEMBER_COMPONENTMASK_MLID;
				break;
			case IB_MCMEMBER_RECORD_COMP_JOINSTATE:
				result |= STL_MCMEMBER_COMPONENTMASK_JNSTATE;
				break;
			case IB_MCMEMBER_RECORD_COMP_PROXYJOIN:
				result |= STL_MCMEMBER_COMPONENTMASK_PROXYJN;
				break;
			default: // For component mask positions that are the same in IB & STL.
				result |= (mask & pos);
		}
	}
	
	return result;
}

typedef struct {
	STL_MCMEMBER_RECORD member;
	uint32_t	index;
	uint32_t	slid;
	uint8_t		proxy;
	uint8_t		state;
	uint64_t	nodeGuid;
	uint64_t	portGuid;
} PACK_SUFFIX STL_MCMEMBER_SYNCDB;

static __inline void
BSWAP_STL_MCMEMBER_SYNCDB(STL_MCMEMBER_SYNCDB* Dest)
{
	BSWAP_STL_MCMEMBER_RECORD(&Dest->member);
	Dest->index = ntoh32(Dest->index);
	Dest->slid = ntoh32(Dest->slid);
	Dest->nodeGuid = ntoh64(Dest->nodeGuid);
	Dest->portGuid = ntoh64(Dest->portGuid);
}

static __inline void
BSWAPCOPY_STL_MCMEMBER_SYNCDB(STL_MCMEMBER_SYNCDB* Src, STL_MCMEMBER_SYNCDB* Dest)
{
	memcpy(Dest, Src, sizeof(STL_MCMEMBER_SYNCDB));
	BSWAP_STL_MCMEMBER_SYNCDB(Dest);
}

static __inline void
BSWAP_STL_SMINFO_RECORD(STL_SMINFO_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	BSWAP_STL_SM_INFO(&Dest->SMInfo);
}

static __inline void
BSWAPCOPY_STL_SMINFO_RECORD(STL_SMINFO_RECORD *Src, STL_SMINFO_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Src->RID.LID);
	BSWAPCOPY_STL_SM_INFO(&Src->SMInfo, &Dest->SMInfo);
}

static __inline void
BSWAP_STL_INFORM_INFO_RECORD(STL_INFORM_INFO_RECORD *Dest)
{
	Dest->RID.SubscriberLID = ntoh32(Dest->RID.SubscriberLID);
	Dest->RID.Enum = ntoh16(Dest->RID.Enum);
	BSWAP_STL_INFORM_INFO(&Dest->InformInfoData);
}

static __inline void
BSWAPCOPY_STL_INFORM_INFO_RECORD(STL_INFORM_INFO_RECORD *Src, STL_INFORM_INFO_RECORD *Dest)
{
	Dest->RID.SubscriberLID = ntoh32(Src->RID.SubscriberLID);
	Dest->RID.Enum = ntoh16(Src->RID.Enum);
	BSWAPCOPY_STL_INFORM_INFO(&Src->InformInfoData, &Dest->InformInfoData);
}

static __inline void
BSWAP_STL_LINK_RECORD(STL_LINK_RECORD *Dest)
{
	Dest->RID.FromLID = ntoh32(Dest->RID.FromLID);
	Dest->ToLID = ntoh32(Dest->ToLID);
}

static __inline void
BSWAPCOPY_STL_LINK_RECORD(STL_LINK_RECORD *Src, STL_LINK_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_LINK_RECORD));
	BSWAP_STL_LINK_RECORD(Dest);
}

static __inline void
BSWAP_STL_SERVICE_RECORD( STL_SERVICE_RECORD  *Dest)
{
#if CPU_LE
    uint8 i;

    Dest->RID.ServiceID = ntoh64(Dest->RID.ServiceID);
    Dest->RID.ServiceLID = ntoh32(Dest->RID.ServiceLID);
    Dest->RID.ServiceP_Key = ntoh16(Dest->RID.ServiceP_Key);
    BSWAP_IB_GID(&Dest->RID.ServiceGID);

    Dest->ServiceLease = ntoh32(Dest->ServiceLease);
    ntoh(&Dest->ServiceKey[0], &Dest->ServiceKey[0], 16);

	Dest->Reserved = 0;

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
 
static __inline void BSWAPCOPY_STL_SERVICE_RECORD(STL_SERVICE_RECORD *Src, STL_SERVICE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_SERVICE_RECORD));
	BSWAP_STL_SERVICE_RECORD(Dest);
}

static __inline void
BSWAP_STL_TRACE_RECORD(STL_TRACE_RECORD  *Dest)
{
#if CPU_LE
	Dest->IDGeneration = ntoh16(Dest->IDGeneration);
	Dest->NodeID = ntoh64(Dest->NodeID);
	Dest->ChassisID = ntoh64(Dest->ChassisID);
	Dest->EntryPortID = ntoh64(Dest->EntryPortID);
	Dest->ExitPortID = ntoh64(Dest->ExitPortID);
#endif /* CPU_LE */
}
 
static __inline void
BSWAPCOPY_STL_TRACE_RECORD(STL_TRACE_RECORD *Src, STL_TRACE_RECORD *Dest)
{
#if CPU_LE
	memcpy(Dest,Src,sizeof(STL_TRACE_RECORD));
	BSWAP_STL_TRACE_RECORD(Dest);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_CABLE_INFO_RECORD(STL_CABLE_INFO_RECORD *Dest)
{
	Dest->LID = ntoh32(Dest->LID);
	Dest->u1.AsReg16 = ntoh16(Dest->u1.AsReg16);
}

static __inline void
BSWAPCOPY_STL_CABLE_INFO_RECORD(STL_CABLE_INFO_RECORD *Src, STL_CABLE_INFO_RECORD *Dest)
{
	*Dest = *Src;
	BSWAP_STL_CABLE_INFO_RECORD(Dest);
}

static __inline void
BSWAP_STL_VFINFO_RECORD(STL_VFINFO_RECORD *Dest)
{
	Dest->vfIndex = ntoh16(Dest->vfIndex);
	Dest->pKey = ntoh16(Dest->pKey);
	Dest->ServiceID = ntoh64(Dest->ServiceID);
	BSWAP_IB_GID(&Dest->MGID);
}

static __inline void
BSWAPCOPY_STL_VFINFO_RECORD(STL_VFINFO_RECORD *Src, STL_VFINFO_RECORD *Dest)
{
	*Dest = *Src;
	BSWAP_STL_VFINFO_RECORD(Dest);
	
}

static __inline void
BSWAP_STL_QUARANTINED_NODE_RECORD(STL_QUARANTINED_NODE_RECORD *Dest)
{
	Dest->trustedLid = ntoh32(Dest->trustedLid);
	Dest->trustedNodeGUID = ntoh64(Dest->trustedNodeGUID);
	Dest->trustedNeighborNodeGUID = ntoh64(Dest->trustedNeighborNodeGUID);
	Dest->quarantineReasons = ntoh32(Dest->quarantineReasons);
	Dest->expectedNodeInfo.nodeGUID = ntoh64(Dest->expectedNodeInfo.nodeGUID);
	Dest->expectedNodeInfo.portGUID = ntoh64(Dest->expectedNodeInfo.portGUID);
	BSWAP_STL_NODE_INFO(&Dest->NodeInfo);
}

static __inline void
BSWAPCOPY_STL_QUARANTINED_NODE_RECORD(STL_QUARANTINED_NODE_RECORD *Src, STL_QUARANTINED_NODE_RECORD *Dest)
{
	Dest->trustedLid = ntoh32(Src->trustedLid);
	Dest->trustedNodeGUID = ntoh64(Src->trustedNodeGUID);
	Dest->trustedPortNum = Src->trustedPortNum;
	Dest->trustedNeighborNodeGUID = ntoh64(Src->trustedNeighborNodeGUID);
	Dest->quarantineReasons = ntoh32(Src->quarantineReasons);
	Dest->expectedNodeInfo.nodeGUID = ntoh64(Src->expectedNodeInfo.nodeGUID);
	Dest->expectedNodeInfo.portGUID = ntoh64(Src->expectedNodeInfo.portGUID);
	BSWAPCOPY_STL_NODE_INFO(&Src->NodeInfo, &Dest->NodeInfo);
	memcpy(&(Dest->NodeDesc), &(Src->NodeDesc), sizeof(STL_NODE_DESCRIPTION));
	memcpy(&(Dest->expectedNodeInfo.nodeDesc), &(Src->expectedNodeInfo.nodeDesc), sizeof(STL_NODE_DESCRIPTION));
}

static __inline void
BSWAP_STL_CONGESTION_INFO_RECORD(STL_CONGESTION_INFO_RECORD *Dest)
{
	Dest->LID = ntoh32(Dest->LID);
	Dest->reserved = ntoh32(Dest->reserved);
	BSWAP_STL_CONGESTION_INFO(&Dest->CongestionInfo);
}

static __inline void
BSWAPCOPY_STL_CONGESTION_INFO_RECORD(STL_CONGESTION_INFO_RECORD *Src, STL_CONGESTION_INFO_RECORD *Dest)
{
	Dest->LID = ntoh32(Src->LID);
	Dest->reserved = ntoh32(Src->reserved);
	memcpy(&(Dest->CongestionInfo), &(Src->CongestionInfo), sizeof(STL_CONGESTION_INFO));
	BSWAP_STL_CONGESTION_INFO(&Dest->CongestionInfo);
}

static __inline void
BSWAP_STL_SWITCH_CONGESTION_SETTING_RECORD(STL_SWITCH_CONGESTION_SETTING_RECORD *Dest)
{
	Dest->LID = ntoh32(Dest->LID);
	Dest->reserved = ntoh32(Dest->reserved);
	BSWAP_STL_SWITCH_CONGESTION_SETTING(&Dest->SwitchCongestionSetting);
}
static __inline void
BSWAPCOPY_STL_SWITCH_CONGESTION_SETTING_RECORD(STL_SWITCH_CONGESTION_SETTING_RECORD *Src, STL_SWITCH_CONGESTION_SETTING_RECORD *Dest)
{
	Dest->LID = ntoh32(Src->LID);
	Dest->reserved = ntoh32(Src->reserved);
	memcpy(&(Dest->SwitchCongestionSetting), &(Src->SwitchCongestionSetting), sizeof(STL_SWITCH_CONGESTION_SETTING));
	BSWAP_STL_SWITCH_CONGESTION_SETTING(&Dest->SwitchCongestionSetting);
}

static __inline void
BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING_RECORD(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING(&Dest->SwitchPortCongestionSetting, 1);
}
static __inline void
BSWAPCOPY_STL_SWITCH_PORT_CONGESTION_SETTING_RECORD(STL_SWITCH_PORT_CONGESTION_SETTING_RECORD *Src, STL_SWITCH_PORT_CONGESTION_SETTING_RECORD *Dest)
{
	*Dest = *Src;
	BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING_RECORD(Dest);
}

static __inline void
BSWAP_STL_HFI_CONGESTION_SETTING_RECORD(STL_HFI_CONGESTION_SETTING_RECORD *Dest)
{
	Dest->LID = ntoh32(Dest->LID);
	Dest->reserved = ntoh32(Dest->reserved);
	BSWAP_STL_HFI_CONGESTION_SETTING(&Dest->HFICongestionSetting);
}
static __inline void
BSWAPCOPY_STL_HFI_CONGESTION_SETTING_RECORD(STL_HFI_CONGESTION_SETTING_RECORD *Src, STL_HFI_CONGESTION_SETTING_RECORD *Dest)
{
	Dest->LID = ntoh32(Src->LID);
	Dest->reserved = ntoh32(Src->reserved);
	memcpy(&(Dest->HFICongestionSetting), &(Src->HFICongestionSetting), sizeof(STL_HFI_CONGESTION_SETTING));
	BSWAP_STL_HFI_CONGESTION_SETTING(&Dest->HFICongestionSetting);
}

static __inline void
BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE_RECORD(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	Dest->RID.BlockNum = ntoh16(Dest->RID.BlockNum);
	Dest->reserved = ntoh16(Dest->reserved);
    BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE(&Dest->HFICongestionControlTable, 1); 
}
static __inline void
BSWAPCOPY_STL_HFI_CONGESTION_CONTROL_TABLE_RECORD(STL_HFI_CONGESTION_CONTROL_TABLE_RECORD *Src, STL_HFI_CONGESTION_CONTROL_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Src->RID.LID);
	Dest->RID.BlockNum = ntoh16(Src->RID.BlockNum);
	Dest->reserved = ntoh16(Src->reserved);
	memcpy(&(Dest->HFICongestionControlTable), &(Src->HFICongestionControlTable), sizeof(STL_HFI_CONGESTION_CONTROL_TABLE));
    BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE(&Dest->HFICongestionControlTable, 1); 
}

static __inline void
BSWAP_STL_BUFFER_CONTROL_TABLE_RECORD(STL_BUFFER_CONTROL_TABLE_RECORD *Dest)
{
	Dest->RID.LID = ntoh32(Dest->RID.LID);
	BSWAP_STL_BUFFER_CONTROL_TABLE(&Dest->BufferControlTable);
}

static __inline void
BSWAPCOPY_STL_BUFFER_CONTROL_TABLE_RECORD(STL_BUFFER_CONTROL_TABLE_RECORD *Src, STL_BUFFER_CONTROL_TABLE_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(STL_BUFFER_CONTROL_TABLE_RECORD));
	BSWAP_STL_BUFFER_CONTROL_TABLE_RECORD(Dest);
}

static __inline void
BSWAP_STL_FABRICINFO_RECORD(
    STL_FABRICINFO_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->NumHFIs = ntoh32(Dest->NumHFIs);
	Dest->NumSwitches = ntoh32(Dest->NumSwitches);
	Dest->NumInternalHFILinks = ntoh32(Dest->NumInternalHFILinks);
	Dest->NumExternalHFILinks = ntoh32(Dest->NumExternalHFILinks);
	Dest->NumInternalISLs = ntoh32(Dest->NumInternalISLs);
	Dest->NumExternalISLs = ntoh32(Dest->NumExternalISLs);
	Dest->NumDegradedHFILinks = ntoh32(Dest->NumDegradedHFILinks);
	Dest->NumDegradedISLs = ntoh32(Dest->NumDegradedISLs);
	Dest->NumOmittedHFILinks = ntoh32(Dest->NumOmittedHFILinks);
	Dest->NumOmittedISLs = ntoh32(Dest->NumOmittedISLs);
#endif /* CPU_LE */
}

static __inline void
BSWAP_STL_SWITCH_COST(STL_SWITCH_COST *cost)
{
	cost->DLID = ntoh32(cost->DLID);
	cost->value = ntoh16(cost->value);
}

static __inline void
BSWAP_STL_SWITCH_COST_RECORD(STL_SWITCH_COST_RECORD *record)
{
	int i;

	record->SLID = ntoh32(record->SLID);
	for (i = 0; i < STL_SWITCH_COST_NUM_ENTRIES; ++i) {
		BSWAP_STL_SWITCH_COST(record->Cost);
	}
}

//
//	OpaServiceRecord_t - This is the same as the original with a
//			      timestamp appended.
//
typedef	struct {
	STL_SERVICE_RECORD	serviceRecord;	
	uint64_t			expireTime;	
	uint8_t				pkeyDefined;// was the record created using pkey mask?
} OpaServiceRecord_t;
typedef OpaServiceRecord_t * OpaServiceRecordp; /* pointer to record */

#include "iba/public/ipackoff.h"

#if defined (__cplusplus)
}
#endif
#endif //__STL_SA_PRIV_H__

