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

#ifndef __IBA_STL_SM_PRIV_H__
#define __IBA_STL_SM_PRIV_H__

#include "iba/stl_types.h"
#include "iba/ib_sm_priv.h"
#include "iba/stl_sm_types.h"

#if defined (__cplusplus)
extern "C" {
#endif

static __inline void
BSWAP_STL_TRAP_CHANGE_CAPABILITY_DATA(STL_TRAP_CHANGE_CAPABILITY_DATA *Src)
{
#if CPU_LE
	Src->Lid = ntoh32(Src->Lid);
	Src->CapabilityMask.AsReg32 = ntoh32(Src->CapabilityMask.AsReg32);
	Src->CapabilityMask3.AsReg16 = ntoh16(Src->CapabilityMask3.AsReg16);


	Src->u.AsReg16 = ntoh16(Src->u.AsReg16);
#endif
}

static __inline void
BSWAPCOPY_STL_TRAP_CHANGE_CAPABILITY_DATA(STL_TRAP_CHANGE_CAPABILITY_DATA *Src,
	STL_TRAP_CHANGE_CAPABILITY_DATA *Dest)
{
	memcpy(Dest, Src, sizeof(STL_TRAP_CHANGE_CAPABILITY_DATA));
	BSWAP_STL_TRAP_CHANGE_CAPABILITY_DATA(Dest);
}

static __inline void
BSWAP_STL_TRAP_BAD_KEY_DATA(STL_TRAP_BAD_KEY_DATA *Src)
{
#if CPU_LE
	uint64_t temp;

	Src->Lid1 		= ntoh32(Src->Lid1);
	Src->Lid2 		= ntoh32(Src->Lid2);
	Src->Key 		= ntoh32(Src->Key);
	// u
	// Reserved[3]
	temp = ntoh64(Src->Gid1.AsReg64s.H);
	Src->Gid1.AsReg64s.H = ntoh64(Src->Gid1.AsReg64s.L);
	Src->Gid1.AsReg64s.L = temp;
	temp = ntoh64(Src->Gid2.AsReg64s.H);
	Src->Gid2.AsReg64s.H = ntoh64(Src->Gid2.AsReg64s.L);
	Src->Gid2.AsReg64s.L = temp;
	Src->qp1.AsReg32= ntoh32(Src->qp1.AsReg32);
	Src->qp2.AsReg32= ntoh32(Src->qp2.AsReg32);
#endif
}

static __inline void
BSWAPCOPY_STL_TRAP_BAD_KEY_DATA(STL_TRAP_BAD_KEY_DATA *Src, STL_TRAP_BAD_KEY_DATA *Dest)
{
	memcpy(Dest, Src, sizeof(STL_TRAP_BAD_KEY_DATA));
	BSWAP_STL_TRAP_BAD_KEY_DATA(Dest);
}

static __inline void
BSWAP_STL_AGGREGATE_HEADER(STL_AGGREGATE *Dest)
{
#if CPU_LE
	Dest->AttributeID = ntoh16(Dest->AttributeID);
	Dest->Result.AsReg16 = ntoh16(Dest->Result.AsReg16);
	Dest->AttributeModifier = ntoh32(Dest->AttributeModifier);
#endif
}

static __inline void
ZERO_RSVD_STL_AGGREGATE(STL_AGGREGATE *Dest)
{
	Dest->Result.s.Reserved = 0;
}

/**
  Zero & swap or swap & zero @c segCount segment headers in range [start, end) for network transport.
	Stops swapping response on first error segment.  When going from host to network order,
	stops before swapping segment header to network order if segment header error flag is set.

	Stops when either: next segment is past @c end; @c *segCount segments have been processed; or @c seg->Result.s.Error is not zero.

	@param segCount [in, out] As input, number of segments in <tt>[start, end)</tt>.  As output, number of segments processed.
	@param hton When true, swap host to network order, otherwise network to host.

	@return Pointer past the last good segment processed (also pointer to first bad segment in case of error).
*/
static __inline STL_AGGREGATE *
BSWAP_ZERO_AGGREGATE_HEADERS(STL_AGGREGATE * start, STL_AGGREGATE * end, int * segCount, boolean hton)
{
	int count = 0;
	STL_AGGREGATE * h = start;

	while (h < end && count < *segCount) {
		if (hton) {
			if (h->Result.s.Error)
				break;
			ZERO_RSVD_STL_AGGREGATE(h);
			STL_AGGREGATE * next = STL_AGGREGATE_NEXT(h);
			BSWAP_STL_AGGREGATE_HEADER(h);
			h = next;
		}
		else {
			BSWAP_STL_AGGREGATE_HEADER(h);
			ZERO_RSVD_STL_AGGREGATE(h);
			if (h->Result.s.Error)
				break;
			h = STL_AGGREGATE_NEXT(h);
		}
		++count;
	}

	*segCount = count;
	return h;
}

static __inline void
BSWAP_STL_DR_SMP(DRStlSmp_t *Dest)
{
#if CPU_LE
	Dest->M_Key  = ntoh64(Dest->M_Key);
	Dest->DrSLID = ntoh32(Dest->DrSLID);
	Dest->DrDLID = ntoh32(Dest->DrDLID);
#endif
}

static __inline void
BSWAPCOPY_STL_DR_SMP(DRStlSmp_t *Src, DRStlSmp_t *Dest)
{
    memcpy(Dest, Src, sizeof(DRStlSmp_t));
    (void)BSWAP_STL_DR_SMP(Dest);
}

#define	DRStlMad_Init(MAIP,METHOD,TID,AID,AMOD,MKEY,IPATH) {			\
	DRStlSmp_t	*drp;								\
										\
	(void)memset((void *)&(MAIP)->base, 0, sizeof((MAIP)->base));		\
										\
	(MAIP)->active |= (MAI_ACT_BASE | MAI_ACT_DATA);			\
	(MAIP)->base.bversion = STL_BASE_VERSION;					\
	(MAIP)->base.mclass = MAD_CV_SUBN_DR;					\
	(MAIP)->base.cversion = STL_SM_CLASS_VERSION;					\
	(MAIP)->base.method = METHOD;						\
	(MAIP)->base.status = 0;						\
	(MAIP)->base.hopPointer = 0;						\
	(MAIP)->base.hopCount = IPATH[0];					\
	(MAIP)->base.tid = TID;							\
	(MAIP)->base.aid = AID;							\
	(MAIP)->base.rsvd3 = 0;							\
	(MAIP)->base.amod = AMOD;						\
										\
	drp = (DRStlSmp_t *)(MAIP)->data;						\
	(void)memset((void *)drp, 0, sizeof(*drp));				\
										\
	drp->M_Key = MKEY;							\
	drp->DrSLID = STL_LID_PERMISSIVE;						\
	drp->DrDLID = STL_LID_PERMISSIVE;						\
										\
	(void)memcpy((void *)&drp->InitPath[1], (void *)&IPATH[1], IPATH[0]);	\
}

#define	LRDRStlMad_Init(MAIP,METHOD,TID,AID,AMOD,MKEY,IPATH,SLID) {			\
	DRStlSmp_t	*drp;								\
										\
	(void)memset((void *)&(MAIP)->base, 0, sizeof((MAIP)->base));		\
										\
	(MAIP)->active |= (MAI_ACT_BASE | MAI_ACT_DATA);			\
	(MAIP)->base.bversion = STL_BASE_VERSION;					\
	(MAIP)->base.mclass = MAD_CV_SUBN_DR;					\
	(MAIP)->base.cversion = STL_SM_CLASS_VERSION;					\
	(MAIP)->base.method = METHOD;						\
	(MAIP)->base.status = 0;						\
	(MAIP)->base.hopPointer = 0;						\
	(MAIP)->base.hopCount = IPATH[0];					\
	(MAIP)->base.tid = TID;							\
	(MAIP)->base.aid = AID;							\
	(MAIP)->base.rsvd3 = 0;							\
	(MAIP)->base.amod = AMOD;						\
										\
	drp = (DRStlSmp_t *)(MAIP)->data;						\
	(void)memset((void *)drp, 0, sizeof(*drp));				\
	drp->M_Key = MKEY;							\
	drp->DrSLID = SLID;						\
	drp->DrDLID = STL_LID_PERMISSIVE;						\
										\
	(void)memcpy((void *)&drp->InitPath[1], (void *)&IPATH[1], IPATH[0]);	\
}

#define	LRStlMad_Init(MAIP,MCLASS,METHOD,TID,AID,AMOD,MKEY) {			\
	LRStlSmp_t	*lrp;								\
										\
	(void)memset((void *)&(MAIP)->base, 0, sizeof((MAIP)->base));		\
										\
	(MAIP)->active |= (MAI_ACT_BASE | MAI_ACT_DATA);			\
	(MAIP)->base.bversion = STL_BASE_VERSION;					\
	(MAIP)->base.mclass = MCLASS;						\
	(MAIP)->base.cversion = MCLASS == MAD_CV_SUBN_ADM ? SA_MAD_CVERSION : STL_SM_CLASS_VERSION;   \
	(MAIP)->base.method = METHOD;						\
	(MAIP)->base.status = 0;						\
	(MAIP)->base.hopPointer = 0;						\
	(MAIP)->base.hopCount = 0;						\
	(MAIP)->base.tid = TID;							\
	(MAIP)->base.aid = AID;							\
	(MAIP)->base.rsvd3 = 0;							\
	(MAIP)->base.amod = AMOD;						\
										\
	lrp = (LRStlSmp_t *)(MAIP)->data;						\
	(void)memset((void *)lrp, 0, sizeof(*lrp));				\
										\
	lrp->M_Key = MKEY;							\
}

// USER ADVICE: Be careful with casting and using the macros below:
// There exist two versions of stl_get_smp_data:
//  1: Uses Mai_t* input parameter to be used with MAI API (FM only).
//  2: Uses STL_SMP* input parameter to be used with tools.
#define STL_GET_SMP_DATA(MAIP)  ((maip->base.mclass == MAD_CV_SUBN_DR) ?  \
       ((DRStlSmp_t *)(maip->data))->SMPData : ((LRStlSmp_t *)(maip->data))->SMPData)

#define STL_GET_MAI_KEY(MAIP)  ((maip->base.mclass == MAD_CV_SUBN_DR) ?  \
       &(((DRStlSmp_t *)(maip->data))->M_Key) : &(((LRStlSmp_t *)(maip->data))->M_Key))

static inline uint8_t *stl_get_smp_data(STL_SMP *smp)
{
	if (smp->common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE) {
		return smp->SmpExt.DirectedRoute.SMPData;
	}
	return smp->SmpExt.LIDRouted.SMPData;
}

static inline size_t stl_get_smp_data_size(STL_SMP *smp)
{
	if (smp->common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE) {
		return STL_MAX_PAYLOAD_SMP_DR;
	}
	return STL_MAX_PAYLOAD_SMP_LR;
}

static inline size_t stl_get_smp_header_size(STL_SMP * smp)
{
	return sizeof(STL_SMP) - stl_get_smp_data_size(smp);
}

static __inline uint8
GET_STL_PORT_INFO_NeighborMTU(const STL_PORT_INFO *PortInfo, uint8 vl) {
	if (vl & 0x01)
		return(PortInfo->NeighborMTU[(vl & 0x1F) / 2].s.VL1_to_MTU);
	else
		return(PortInfo->NeighborMTU[(vl & 0x1F) / 2].s.VL0_to_MTU);
}

static __inline void
PUT_STL_PORT_INFO_NeighborMTU(STL_PORT_INFO *PortInfo, uint8 vl, uint8 mtu) {
	if (vl & 0x01)
		PortInfo->NeighborMTU[(vl & 0x1F) / 2].s.VL1_to_MTU = mtu & 0x0F;
	else
		PortInfo->NeighborMTU[(vl & 0x1F) / 2].s.VL0_to_MTU = mtu & 0x0F;
}

/* Is the Smp given from a local requestor (eg. on the same host such as a
 * smp from IbAccess to its own local port or from a local SM)
 * allow HopPointer to be 0 for direct use of GetSetMad and
 * 1 for packets which passed through SMI prior to GetSetMad
 */
static __inline boolean
STL_SMP_IS_LOCAL(STL_SMP *Smp)
{
	return (( MCLASS_SM_DIRECTED_ROUTE == Smp->common.MgmtClass ) &&
				( Smp->common.u.DR.HopPointer == 0
				  || Smp->common.u.DR.HopPointer == 1) &&
				( Smp->common.u.DR.HopCount == 0 ));
}

static __inline void
STL_SMP_SET_LOCAL(STL_SMP *Smp)
{
	Smp->common.MgmtClass = MCLASS_SM_DIRECTED_ROUTE;
	Smp->common.u.DR.HopPointer = 0;
	Smp->common.u.DR.HopCount = 0;
	Smp->SmpExt.DirectedRoute.DrSLID = STL_LID_PERMISSIVE;
	Smp->SmpExt.DirectedRoute.DrDLID = STL_LID_PERMISSIVE;
}

static __inline void
BSWAP_STL_SMP_HEADER(STL_SMP *Dest)
{
#if CPU_LE
	BSWAP_MAD_HEADER((MAD *)Dest);

	Dest->M_Key = ntoh64(Dest->M_Key);
	if (Dest->common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE) {
		Dest->SmpExt.DirectedRoute.DrSLID = \
			ntoh32(Dest->SmpExt.DirectedRoute.DrSLID);
		Dest->SmpExt.DirectedRoute.DrDLID = \
			ntoh32(Dest->SmpExt.DirectedRoute.DrDLID);
	}
#endif
}

static __inline void
BSWAPCOPY_STL_SMP(STL_SMP *Src, STL_SMP *Dest)
{
    memcpy(Dest, Src, sizeof(STL_SMP));
    (void)BSWAP_STL_SMP_HEADER(Dest);
}

#define STL_BSWAP_SMP_HEADER  BSWAP_STL_SMP_HEADER	/* Temporary definition */

static __inline void
BSWAP_STL_TRAP_PORT_CHANGE_STATE_DATA(STL_TRAP_PORT_CHANGE_STATE_DATA * data)
{
#if CPU_LE
	data->Lid = ntoh32(data->Lid);
#endif
}

static __inline void
BSWAP_STL_SMA_TRAP_DATA_LINK_WIDTH(STL_SMA_TRAP_DATA_LINK_WIDTH *Dest)
{
#if CPU_LE
	Dest->ReportingLID = ntoh32(Dest->ReportingLID);
#endif
}


static __inline void
BSWAP_STL_NODE_DESCRIPTION(STL_NODE_DESCRIPTION *Dest)
{
#if CPU_LE
	/* pure text field, nothing to swap */
#endif
}


static __inline void
BSWAP_STL_NODE_INFO(STL_NODE_INFO *Dest)
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


/* NOTE: for this implementation the cost of checking for and handling
 * overlapping memory regions is assumed to be higher than warranted.
 */
static __inline void
BSWAPCOPY_STL_NODE_INFO(STL_NODE_INFO *Src, STL_NODE_INFO *Dest)
{
if (Src != Dest)
	(void)memcpy((void *)Dest, (void *)Src, sizeof(STL_NODE_INFO));
	BSWAP_STL_NODE_INFO(Dest);
}

static __inline void
BSWAP_STL_SWITCH_INFO(STL_SWITCH_INFO *Dest)
{
#if CPU_LE
	Dest->LinearFDBCap = ntoh32(Dest->LinearFDBCap);
	Dest->PortGroupFDBCap = ntoh32(Dest->PortGroupFDBCap);
	Dest->MulticastFDBCap = ntoh32(Dest->MulticastFDBCap);
	Dest->LinearFDBTop = ntoh32(Dest->LinearFDBTop);
	Dest->MulticastFDBTop = ntoh32(Dest->MulticastFDBTop);
	Dest->CollectiveCap = ntoh32(Dest->CollectiveCap);
	Dest->CollectiveTop = ntoh32(Dest->CollectiveTop);
#if !defined(PRODUCT_STL1)
#endif
	Dest->PartitionEnforcementCap = ntoh16(Dest->PartitionEnforcementCap);
	Dest->AdaptiveRouting.AsReg16 = ntoh16(Dest->AdaptiveRouting.AsReg16);
	Dest->CapabilityMask.AsReg16 = ntoh16(Dest->CapabilityMask.AsReg16);
	Dest->CapabilityMaskCollectives.AsReg16 = ntoh16(Dest->CapabilityMaskCollectives.AsReg16);
#endif
}

static __inline void
ZERO_RSVD_STL_SWITCH_INFO(STL_SWITCH_INFO * Dest)
{
	Dest->Reserved = 0;

	Dest->Reserved24 = 0;
	Dest->Reserved26 = 0;
	Dest->Reserved27 = 0;

	Dest->Reserved28 = 0;
	Dest->Reserved21 = 0;
	Dest->Reserved22 = 0;
	Dest->Reserved23 = 0;

	Dest->u1.s.Reserved20 = 0;

	Dest->u2.s.Reserved20 = 0;
	Dest->u2.s.Reserved21 = 0;
	Dest->u2.s.Reserved22 = 0;
	Dest->u2.s.Reserved23 = 0;
	Dest->u2.s.Reserved = 0;

	Dest->MultiCollectMask.Reserved = 0;
	Dest->AdaptiveRouting.s.Reserved = 0;
	Dest->CapabilityMask.s.Reserved = 0;
	Dest->CapabilityMaskCollectives.s.Reserved = 0;
}

static __inline void
BSWAPCOPY_STL_SWITCH_INFO(STL_SWITCH_INFO *Src, STL_SWITCH_INFO *Dest)
{
	if (Src != Dest) memcpy((void *)Dest, (void *)Src, sizeof(STL_SWITCH_INFO));
	BSWAP_STL_SWITCH_INFO(Dest);
}

static __inline void
BSWAP_STL_MKEY(uint64_t *mkey)
{
#if CPU_LE
    *mkey = ntoh64(*mkey);
#endif
}

static __inline void
BSWAPCOPY_STL_MKEY(uint64_t *Src, uint64_t *Dest)
{
	if (Src != Dest) memcpy((void *)Dest, (void *)Src, sizeof(uint64_t));
    BSWAP_STL_MKEY(Dest);
}

static __inline void
BSWAP_STL_PORT_INFO(STL_PORT_INFO *Dest)
{
#if CPU_LE
	Dest->LID = ntoh32(Dest->LID);
	Dest->FlowControlMask = ntoh32(Dest->FlowControlMask);
	Dest->VL.HighLimit = ntoh16(Dest->VL.HighLimit);
	Dest->VL.PreemptingLimit = ntoh16(Dest->VL.PreemptingLimit);
	Dest->PortStates.AsReg32 = ntoh32(Dest->PortStates.AsReg32);
	Dest->P_Keys.P_Key_8B = ntoh16(Dest->P_Keys.P_Key_8B);
	Dest->P_Keys.P_Key_10B = ntoh16(Dest->P_Keys.P_Key_10B);
	Dest->Violations.M_Key = ntoh16(Dest->Violations.M_Key);
	Dest->Violations.P_Key = ntoh16(Dest->Violations.P_Key);
	Dest->Violations.Q_Key = ntoh16(Dest->Violations.Q_Key);
	Dest->SM_TrapQP.AsReg32 = ntoh32(Dest->SM_TrapQP.AsReg32);
	Dest->SA_QP.AsReg32 = ntoh32(Dest->SA_QP.AsReg32);
	Dest->LinkSpeed.Supported = ntoh16(Dest->LinkSpeed.Supported);
	Dest->LinkSpeed.Enabled = ntoh16(Dest->LinkSpeed.Enabled);
	Dest->LinkSpeed.Active = ntoh16(Dest->LinkSpeed.Active);
	Dest->LinkWidth.Supported = ntoh16(Dest->LinkWidth.Supported);
	Dest->LinkWidth.Enabled = ntoh16(Dest->LinkWidth.Enabled);
	Dest->LinkWidth.Active = ntoh16(Dest->LinkWidth.Active);
	Dest->LinkWidthDowngrade.Supported = ntoh16(Dest->LinkWidthDowngrade.Supported);
	Dest->LinkWidthDowngrade.Enabled = ntoh16(Dest->LinkWidthDowngrade.Enabled);
	Dest->LinkWidthDowngrade.TxActive = ntoh16(Dest->LinkWidthDowngrade.TxActive);
	Dest->LinkWidthDowngrade.RxActive = ntoh16(Dest->LinkWidthDowngrade.RxActive);
	Dest->PortLinkMode.AsReg16 = ntoh16(Dest->PortLinkMode.AsReg16);
	Dest->PortLTPCRCMode.AsReg16 = ntoh16(Dest->PortLTPCRCMode.AsReg16);
	Dest->PortMode.AsReg16 = ntoh16(Dest->PortMode.AsReg16);
	Dest->PortPacketFormats.Supported = ntoh16(Dest->PortPacketFormats.Supported);
	Dest->PortPacketFormats.Enabled = ntoh16(Dest->PortPacketFormats.Enabled);
	Dest->FlitControl.Interleave.AsReg16 = ntoh16(Dest->FlitControl.Interleave.AsReg16);
	Dest->FlitControl.Preemption.MinInitial = ntoh16(Dest->FlitControl.Preemption.MinInitial);
	Dest->FlitControl.Preemption.MinTail = ntoh16(Dest->FlitControl.Preemption.MinTail);
	Dest->MaxLID = ntoh32(Dest->MaxLID);
	Dest->PortErrorAction.AsReg32 = ntoh32(Dest->PortErrorAction.AsReg32);
	Dest->M_KeyLeasePeriod = ntoh16(Dest->M_KeyLeasePeriod);

#if !defined(PRODUCT_STL1)
#endif


	Dest->BufferUnits.AsReg32 = ntoh32(Dest->BufferUnits.AsReg32);
	Dest->MasterSMLID = ntoh32(Dest->MasterSMLID);
	Dest->M_Key = ntoh64(Dest->M_Key);
	Dest->SubnetPrefix = ntoh64(Dest->SubnetPrefix);
	Dest->NeighborNodeGUID = ntoh64(Dest->NeighborNodeGUID);
	Dest->CapabilityMask.AsReg32 = ntoh32(Dest->CapabilityMask.AsReg32);
	Dest->CapabilityMask3.AsReg16 = ntoh16(Dest->CapabilityMask3.AsReg16);
	Dest->OverallBufferSpace = ntoh16(Dest->OverallBufferSpace);
	Dest->DiagCode.AsReg16 = ntoh16(Dest->DiagCode.AsReg16);
#endif
}

static __inline void
ZERO_RSVD_STL_PORT_INFO(STL_PORT_INFO * Dest)
{
	Dest->VL.s2.Reserved = 0;
	Dest->PortStates.s.Reserved = 0;
	Dest->PortPhysConfig.s.Reserved = 0;
	Dest->MultiCollectMask.Reserved = 0;
#if !defined(PRODUCT_STL1)
	Dest->s2.Reserved = 0;
#else
	Dest->s1.Reserved = 0;
#endif
	Dest->s3.Reserved20 = 0;
	Dest->s3.Reserved21 = 0;
	Dest->s4.Reserved = 0;
	Dest->SM_TrapQP.s.Reserved = 0;
	Dest->SA_QP.s.Reserved = 0;
	Dest->PortLinkMode.s.Reserved = 0;
	Dest->PortLTPCRCMode.s.Reserved = 0;
	Dest->PortMode.s.Reserved = 0;
	Dest->PortMode.s.Reserved2 = 0;
	Dest->PortMode.s.Reserved3 = 0;
	Dest->FlitControl.Interleave.s.Reserved = 0;
	Dest->PortErrorAction.s.Reserved = 0;
	Dest->PortErrorAction.s.Reserved2 = 0;
	Dest->PortErrorAction.s.Reserved3 = 0;
	Dest->PortErrorAction.s.Reserved4 = 0;
	Dest->PassThroughControl.Reserved = 0;
	Dest->BufferUnits.s.Reserved = 0;
#if !defined(PRODUCT_STL1)
#endif
	Dest->Reserved14 = 0;
	// Reserved fields in the RO section
#if !defined(PRODUCT_STL1)
	Dest->Reserved27 = 0;
	Dest->Reserved28 = 0;
#endif
	Dest->Reserved20 = 0;

	Dest->CapabilityMask.s.CmReserved6 = 0;
	Dest->CapabilityMask.s.CmReserved24 = 0;
	Dest->CapabilityMask.s.CmReserved5 = 0;
	Dest->CapabilityMask.s.CmReserved23 = 0;
	Dest->CapabilityMask.s.CmReserved22 = 0;
	Dest->CapabilityMask.s.CmReserved21 = 0;
	Dest->CapabilityMask.s.CmReserved25 = 0;
	Dest->CapabilityMask.s.CmReserved2 = 0;
	Dest->CapabilityMask.s.CmReserved20 = 0;
	Dest->CapabilityMask.s.CmReserved1 = 0;
	Dest->CapabilityMask3.s.CmReserved0 = 0;
	Dest->CapabilityMask3.s.CmReserved1 = 0;
	Dest->CapabilityMask3.s.CmReserved2 = 0;
	Dest->CapabilityMask3.s.CmReserved3 = 0;
	Dest->CapabilityMask3.s.CmReserved4 = 0;
	Dest->PortNeighborMode.Reserved = 0;
	Dest->MTU.Reserved20 = 0;
	Dest->Resp.Reserved = 0;
	Dest->Reserved24 = 0;
	Dest->Reserved25 = 0;

	Dest->Reserved23 = 0;
}

static __inline void
BSWAP_STL_PARTITION_TABLE(STL_PARTITION_TABLE *Dest)
{
#if CPU_LE
	uint32  i;

	for (i = 0; i < NUM_PKEY_ELEMENTS_BLOCK; i++)
	{
		Dest->PartitionTableBlock[i].AsReg16 = \
			ntoh16(Dest->PartitionTableBlock[i].AsReg16);
	}
#endif
}

static __inline void
BSWAPCOPY_STL_PARTITION_TABLE(STL_PARTITION_TABLE *Src, STL_PARTITION_TABLE *Dest)
{
	if (Src != Dest) memcpy(Dest, Src, sizeof(STL_PARTITION_TABLE));
	BSWAP_STL_PARTITION_TABLE(Dest);
}


static __inline void
BSWAP_STL_SLSCMAP(STL_SLSCMAP *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}

static __inline void
ZERO_RSVD_STL_SLSCMAP(STL_SLSCMAP * Dest)
{
	uint32 i;
	for (i = 0; i < sizeof(Dest->SLSCMap)/sizeof(Dest->SLSCMap[0]); ++i) {
		Dest->SLSCMap[i].Reserved = 0;
	}
}

static __inline void
BSWAP_STL_SCSCMAP(STL_SCSCMAP *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}


static __inline void
BSWAP_STL_SCSLMAP(STL_SCSLMAP *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}

static __inline void
ZERO_RSVD_STL_SCSLMAP(STL_SCSLMAP *Dest)
{
	uint32 i;
	for (i = 0; i < sizeof(Dest->SCSLMap)/sizeof(Dest->SCSLMap[0]); ++i) {
		Dest->SCSLMap[i].Reserved = 0;
	}
}

static __inline void
BSWAP_STL_SCVLMAP(STL_SCVLMAP *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}

static __inline void
ZERO_RSVD_STL_SCVLMAP(STL_SCVLMAP *Dest)
{
	uint32 i;
	for (i = 0; i < sizeof(Dest->SCVLMap)/sizeof(Dest->SCVLMap[0]); ++i) {
		Dest->SCVLMap[i].Reserved = 0;
	}
}
/* The section flag refers to which section of STL_VLARB_TABLE
 * is to be swapped (see STL_VLARB_TABLE).  Only the Preemption
 * Matrix section (STL_VLARB_PREEMPT_MATRIX) requires action.
 */
static __inline void
BSWAP_STL_VLARB_TABLE(STL_VLARB_TABLE *Dest, int section)
{
#if CPU_LE
	uint32  i;

	if (section == STL_VLARB_PREEMPT_MATRIX)
	{
		for (i = 0; i < STL_MAX_VLS; i++)
			Dest->Matrix[i] = ntoh32(Dest->Matrix[i]);
	}
#endif
}

static __inline void
ZERO_RSVD_STL_VLARB_TABLE(STL_VLARB_TABLE *Dest, uint8 section)
{
	if (section != STL_VLARB_PREEMPT_MATRIX) {
		uint32 i;
		for (i = 0; i < VLARB_TABLE_LENGTH; ++i)
			Dest->Elements[i].s.Reserved = 0;
	}
}

static __inline void
BSWAP_STL_PORT_SELECTMASK(STL_PORTMASK *Dest)
{
#if CPU_LE
	unsigned int i;

	for (i=0; i<STL_MAX_PORTMASK; i++) {
		Dest[i] = ntoh64(Dest[i]);
	}
#endif
}

static __inline void
BSWAP_STL_SCSC_MULTISET(STL_SCSC_MULTISET *Dest)
{
#if CPU_LE
	BSWAP_STL_PORT_SELECTMASK(Dest->IngressPortMask);
	BSWAP_STL_PORT_SELECTMASK(Dest->EgressPortMask);
#endif
}


static __inline void
BSWAP_STL_LINEAR_FORWARDING_TABLE(STL_LINEAR_FORWARDING_TABLE *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}


static __inline void
BSWAP_STL_MULTICAST_FORWARDING_TABLE(STL_MULTICAST_FORWARDING_TABLE *Dest)
{
#if CPU_LE
	uint32  i;

	for (i = 0; i < STL_NUM_MFT_ELEMENTS_BLOCK; i++)
	{
		Dest->MftBlock[i] = ntoh64(Dest->MftBlock[i]);
	}
#endif
}

static __inline void
BSWAP_STL_SM_INFO(STL_SM_INFO *Dest)
{
#if CPU_LE
    Dest->PortGUID = ntoh64(Dest->PortGUID);
    Dest->SM_Key = ntoh64(Dest->SM_Key);
    Dest->ActCount = ntoh32(Dest->ActCount);
    Dest->ElapsedTime = ntoh32(Dest->ElapsedTime);
    Dest->u.AsReg16 = ntoh16(Dest->u.AsReg16);
#endif
}

static __inline void
BSWAPCOPY_STL_SM_INFO(STL_SM_INFO *Src, STL_SM_INFO *Dest)
{
    memcpy(Dest, Src, sizeof(STL_SM_INFO));
	BSWAP_STL_SM_INFO(Dest);
}

static __inline const char*
StlSMStateToText(SM_STATE state)
{
	return ((state == SM_INACTIVE)? "Inactive":
			(state == SM_DISCOVERING)? "Discovering":
			(state == SM_STANDBY)? "Standby":
			(state == SM_MASTER)? "Master": "???");
}

static __inline void
BSWAP_STL_LED_INFO(STL_LED_INFO *Dest)
{
#if CPU_LE
    Dest->u.AsReg32 = ntoh32(Dest->u.AsReg32);
#endif
}

static __inline void
ZERO_RSVD_STL_LED_INFO(STL_LED_INFO *Dest) {

#if !defined(PRODUCT_STL1)
	Dest->u.s.Reserved = 0;
#endif
	Dest->Reserved2 = 0;

}


static __inline void
BSWAP_STL_CABLE_INFO(STL_CABLE_INFO *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}


static __inline void
BSWAP_STL_AGGREGATE(STL_AGGREGATE *Dest)
{
#if CPU_LE
    Dest->AttributeID = ntoh16(Dest->AttributeID);
    Dest->Result.AsReg16 = ntoh16(Dest->Result.AsReg16);
    Dest->AttributeModifier = ntoh32(Dest->AttributeModifier);
#endif
}


/* num_ports is the number of port entries in STL_PORT_STATE_INFO */
static __inline void
BSWAP_STL_PORT_STATE_INFO(STL_PORT_STATE_INFO *Dest, uint8 num_ports)
{
#if CPU_LE
	uint32  i;

	for (i = 0; i < num_ports; i++)
	{
		Dest[i].PortStates.AsReg32 = ntoh32(Dest[i].PortStates.AsReg32);
		Dest[i].LinkWidthDowngradeTxActive = ntoh16(Dest[i].LinkWidthDowngradeTxActive);
		Dest[i].LinkWidthDowngradeRxActive = ntoh16(Dest[i].LinkWidthDowngradeRxActive); 
	}
#endif
}


static __inline void
BSWAP_STL_PORT_GROUP_FORWARDING_TABLE(STL_PORT_GROUP_FORWARDING_TABLE *Dest)
{
#if CPU_LE
	/* nothing to swap */
#endif
}


static __inline void
BSWAP_STL_PORT_GROUP_TABLE(STL_PORT_GROUP_TABLE *Dest)
{
#if CPU_LE
	uint32  i;

	for (i = 0; i < NUM_PGT_ELEMENTS_BLOCK; i++)
	{
		Dest->PgtBlock[i] = ntoh64(Dest->PgtBlock[i]);
	}
#endif
}

static __inline void
BSWAP_STL_BUFFER_CONTROL_TABLE(STL_BUFFER_CONTROL_TABLE *Dest)
{
#if CPU_LE
	uint32  i;

    Dest->TxOverallSharedLimit = ntoh16(Dest->TxOverallSharedLimit);

	for (i = 0; i < STL_MAX_VLS; i++)
	{
		Dest->VL[i].TxDedicatedLimit = ntoh16(Dest->VL[i].TxDedicatedLimit);
		Dest->VL[i].TxSharedLimit = ntoh16(Dest->VL[i].TxSharedLimit);
	}
#endif
}

static __inline void
BSWAP_STL_CONGESTION_INFO(STL_CONGESTION_INFO *Dest)
{
#if CPU_LE
    Dest->CongestionInfo = ntoh16(Dest->CongestionInfo);
#endif
}


/* num_blocks is the number of blocks in STL_SWITCH_CONGESTION_LOG to swap */
static __inline void
BSWAP_STL_SWITCH_CONGESTION_LOG(STL_SWITCH_CONGESTION_LOG *Dest)
{
#if CPU_LE
	uint32  i;

	Dest->LogEventsCounter = ntoh16(Dest->LogEventsCounter);
	Dest->CurrentTimeStamp = ntoh32(Dest->CurrentTimeStamp);

	for (i = 0; i < STL_NUM_CONGESTION_LOG_ELEMENTS; i++)
	{
		Dest->CongestionEntryList[i].SLID = ntoh32(Dest->CongestionEntryList[i].SLID);
		Dest->CongestionEntryList[i].DLID = ntoh32(Dest->CongestionEntryList[i].DLID);
		Dest->CongestionEntryList[i].TimeStamp = ntoh32(Dest->CongestionEntryList[i].TimeStamp);
	}
#endif
}


static __inline void
BSWAP_STL_SWITCH_CONGESTION_SETTING(STL_SWITCH_CONGESTION_SETTING *Dest)
{
#if CPU_LE
    Dest->Control_Map = ntoh32(Dest->Control_Map);
    Dest->CS_ReturnDelay = ntoh16(Dest->CS_ReturnDelay);
    Dest->Marking_Rate = ntoh16(Dest->Marking_Rate);
#endif
}

static __inline void
ZERO_RSVD_STL_SWITCH_CONGESTION_SETTING(STL_SWITCH_CONGESTION_SETTING *Dest)
{
	Dest->Reserved = 0;
	Dest->Reserved2 = 0;
	Dest->Reserved3 = 0;
}

/* num_ports is the number of ports in STL_SWITCH_PORT_CONGESTION_SETTING to swap */
static __inline void
BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING(STL_SWITCH_PORT_CONGESTION_SETTING *Dest, uint8 num_ports)
{
#if CPU_LE
	uint32  i;

	for (i = 0; i < num_ports; i++)
	{
		Dest->Elements[i].Marking_Rate = ntoh16(Dest->Elements[i].Marking_Rate);
	}
#endif
}


/* num_blocks is the number of blocks in STL_HFI_CONGESTION_LOG to swap */
static __inline void
BSWAP_STL_HFI_CONGESTION_LOG(STL_HFI_CONGESTION_LOG *Dest)
{
#if CPU_LE
	uint32  i;

	Dest->ThresholdEventCounter = ntoh16(Dest->ThresholdEventCounter);
	Dest->CurrentTimeStamp = ntoh16(Dest->CurrentTimeStamp);

	for (i = 0; i < STL_NUM_CONGESTION_LOG_ELEMENTS; i++)
	{
		Dest->CongestionEntryList[i].Remote_LID_CN_Entry =
			ntoh32(Dest->CongestionEntryList[i].Remote_LID_CN_Entry);
		Dest->CongestionEntryList[i].TimeStamp_CN_Entry =
			ntoh32(Dest->CongestionEntryList[i].TimeStamp_CN_Entry);
	}
#endif
}


static __inline void
BSWAP_STL_HFI_CONGESTION_SETTING(STL_HFI_CONGESTION_SETTING *Dest)
{
#if CPU_LE
	uint32  i;

	Dest->Port_Control = ntoh16(Dest->Port_Control);
	Dest->Control_Map = ntoh32(Dest->Control_Map);

	for (i = 0; i < STL_MAX_SLS; i++)
	{
		Dest->HFICongestionEntryList[i].CCTI_Timer =
			ntoh16(Dest->HFICongestionEntryList[i].CCTI_Timer);
	}
#endif
}


/* num_blocks is the number of blocks in STL_HFI_CONGESTION_CONTROL_TABLE to swap */
static __inline void
BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE(STL_HFI_CONGESTION_CONTROL_TABLE *Dest, uint8 num_blocks)
{
#if CPU_LE
	uint32  i,j;

	Dest->CCTI_Limit = ntoh16(Dest->CCTI_Limit);

	for(i = 0; i < num_blocks; ++i) {
		for (j = 0; j < STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES; ++j) {
			Dest->CCT_Block_List[i].CCT_Entry_List[j].AsReg16 = 
				ntoh16(Dest->CCT_Block_List[i].CCT_Entry_List[j].AsReg16);
		}
	}
#endif
}


#if defined (__cplusplus)
};
#endif

#endif /* __IBA_STL_SM_PRIV_H__ */
