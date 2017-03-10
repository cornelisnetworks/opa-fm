/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//======================================================================//
//									     								//
// FILE NAME															//
//    sa_vFabricRecord.c						     					//
//									     								//
// DESCRIPTION								     						//
//    This file contains the routines to process the SA requests for	//
//    Virtual Fabric Information.										//
//									     								//
//======================================================================//


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "stl_print.h"

Status_t sa_VFabric_GetTable(Mai_t *, uint32_t *);

Status_t
sa_VFabricRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	records;
    uint16_t    attribOffset;

	IB_ENTER("sa_VFabricRecord", maip, sa_cntxt, 0, 0);

	records = 0;

//
//	Check the method.  If this is a template lookup, then call the regular
//	GetTable(*) template lookup routine.
//
	switch (maip->base.method) {
	case SA_CM_GET:
		INCREMENT_COUNTER(smCounterSaRxGetVfRecord);
		(void)sa_VFabric_GetTable(maip, &records);
		break;
	case SA_CM_GETTABLE:
		INCREMENT_COUNTER(smCounterSaRxGetTblVfRecord);
		(void)sa_VFabric_GetTable(maip, &records);
		break;
    default:
		maip->base.status = MAD_STATUS_BAD_METHOD;
        break;
	}

//
//	Determine reply status
//
	if (maip->base.status != MAD_STATUS_OK) {
		/* status already set */
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
        IB_LOG_WARN("sa_VFabricRecord: too many records for SA_CM_GET:", records);
        records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_VFINFO_RECORD) + Calculate_Padding(sizeof(STL_VFINFO_RECORD));
    /* setup attribute offset for possible RMPP transfer */
    sa_cntxt->attribLen = attribOffset;

	sa_cntxt_data(sa_cntxt, sa_data, records * attribOffset);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_VFabricRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_VFabric_Set(uint8_t *vfrp, uint8_t vf, STL_SA_MAD *samad, uint64_t serviceId, IB_GID mGid) {
	
	STL_VFINFO_RECORD vfRecord = {0};
	
	IB_ENTER("sa_VFabric_Set", vfrp, 0, 0, 0);

	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	vfRecord.vfIndex = VirtualFabrics->v_fabric[vf].index;
	vfRecord.pKey = VirtualFabrics->v_fabric[vf].pkey;
	strncpy((void *)vfRecord.vfName, (void *)VirtualFabrics->v_fabric[vf].name, STL_VFABRIC_NAME_LEN);
	vfRecord.ServiceID = serviceId;
	vfRecord.MGID = mGid;

	if (VirtualFabrics->v_fabric[vf].security) {
		vfRecord.s1.selectFlags = STL_VFINFO_REC_SEL_PKEY_QUERY;
	}

	if (VirtualFabrics->v_fabric[vf].qos_enable) {
		vfRecord.s1.selectFlags |= STL_VFINFO_REC_SEL_SL_QUERY;
	}

	vfRecord.s1.mtu = VirtualFabrics->v_fabric[vf].max_mtu_int;
	vfRecord.s1.mtuSpecified = VirtualFabrics->v_fabric[vf].max_mtu_specified;
	vfRecord.s1.rate = VirtualFabrics->v_fabric[vf].max_rate_int;
	vfRecord.s1.rateSpecified = VirtualFabrics->v_fabric[vf].max_rate_specified;
	vfRecord.s1.pktLifeTimeInc = VirtualFabrics->v_fabric[vf].pkt_lifetime_mult;
	vfRecord.s1.pktLifeSpecified = VirtualFabrics->v_fabric[vf].pkt_lifetime_specified;
	vfRecord.s1.sl = VirtualFabrics->v_fabric[vf].base_sl;
	vfRecord.bandwidthPercent = 0;
	vfRecord.priority = VirtualFabrics->v_fabric[vf].priority;
	vfRecord.routingSLs = 1;
	vfRecord.preemptionRank = VirtualFabrics->v_fabric[vf].preempt_rank;
	vfRecord.hoqLife = VirtualFabrics->v_fabric[vf].hoqlife_vf;
	vfRecord.optionFlags = 0;

	if (VirtualFabrics->v_fabric[vf].security) {
		vfRecord.optionFlags |= STL_VFINFO_REC_OPT_SECURITY;
	}

	if (VirtualFabrics->qosEnabled &&
		VirtualFabrics->v_fabric[vf].qos_enable) {
		vfRecord.optionFlags |= STL_VFINFO_REC_OPT_QOS;
		if (!VirtualFabrics->v_fabric[vf].priority)
			vfRecord.bandwidthPercent = VirtualFabrics->v_fabric[vf].percent_bandwidth;
	}

	if (VirtualFabrics->v_fabric[vf].flowControlDisable) {
		vfRecord.optionFlags |= STL_VFINFO_REC_OPT_FLOW_DISABLE;
	}

	*(STL_VFINFO_RECORD*)vfrp = vfRecord;

	IB_EXIT("sa_VFabric_Set", VSTATUS_OK);
	return(VSTATUS_OK);
}


Status_t
sa_VFabric_GetTable(Mai_t *maip, uint32_t *records) {
	uint8_t			*data;
	uint32_t		bytes;
	STL_SA_MAD			samad;
	Status_t		status;
	STL_VFINFO_RECORD vFabricRecord;
	int				vf;
	int				reqInFullDefault=0;	// requestor full member of Default PKey
	Node_t *reqNodep;
	Port_t *reqPortp;
	uint64_t serviceId = 0;	// only reported if in samad.header.mask
	IB_GID mGid = (IB_GID){.Raw = {0}}; // only reported if in samad.header.mask

	IB_ENTER("sa_VFabric_GetTable", maip, *records, 0, 0);

	*records = 0;
	data = sa_data;
	bytes = Calculate_Padding(sizeof(STL_VFINFO_RECORD));
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, sizeof(STL_VFINFO_RECORD));

    /*
     *	Create the template mask for the lookup.
     */
	if ((status = sa_create_template_mask(maip->base.aid, samad.header.mask)) != VSTATUS_OK) {
        IB_LOG_WARNRC("sa_VFabric_GetTable: failed to create template mask, rc:", status);
        IB_EXIT("sa_VFabric_GetTable", status);
		return(status);
	}

	maip->base.status = MAD_STATUS_OK;

	BSWAPCOPY_STL_VFINFO_RECORD((STL_VFINFO_RECORD*)samad.data, &vFabricRecord);
	
	vFabricRecord.rsvd1 = 0;
	vFabricRecord.s1.rsvd2 = 0;
	vFabricRecord.s1.rsvd3 = 0;
	vFabricRecord.s1.rsvd4 = 0;
	vFabricRecord.s1.rsvd5 = 0;
	vFabricRecord.rsvd6 = 0;
	memset(vFabricRecord.rsvd7, 0, sizeof(vFabricRecord.rsvd7));

	if (samad.header.mask & STL_VFINFO_REC_COMP_SERVICEID) {
		serviceId = vFabricRecord.ServiceID;
	}

	if (samad.header.mask & STL_VFINFO_REC_COMP_MGID) {
		mGid = vFabricRecord.MGID; // for response
	}

	(void)vs_rdlock(&old_topology_lock);

	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (!VirtualFabrics) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		goto reply_vFabric;
	}

	reqPortp = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, &reqNodep);
	if (!sm_valid_port(reqPortp) || reqPortp->state <= IB_PORT_DOWN) {
		if (saDebugPerf) {
			IB_LOG_INFINI_INFO_FMT("sa_VFabric_GetTable",
				"Request from node which is no longer valid, slid=0x%x", maip->addrInfo.slid);
		}
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		goto reply_vFabric;
	}

	if (smValidatePortPKey(DEFAULT_PKEY, reqPortp))
		reqInFullDefault=1;

	for (vf=0; vf < VirtualFabrics->number_of_vfs && vf < MAX_VFABRICS; vf++) {

		if ((samad.header.mask & STL_VFINFO_REC_COMP_PKEY) && 
			(PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) != PKEY_VALUE(vFabricRecord.pKey))) continue;

		if ((samad.header.mask & STL_VFINFO_REC_COMP_INDEX) &&
			(VirtualFabrics->v_fabric[vf].index != vFabricRecord.vfIndex)) continue;

		if (samad.header.mask & STL_VFINFO_REC_COMP_NAME) {
			if (strncmp((void*)VirtualFabrics->v_fabric[vf].name, 
				(void*)vFabricRecord.vfName, STL_VFABRIC_NAME_LEN) != 0) continue;
		}

		// TBD - when allow multiple SLs for DOR, adjust this query to check
		// if SL matches any of the SLs used by given VF
		if (samad.header.mask & STL_VFINFO_REC_COMP_SL) {
			if (VirtualFabrics->v_fabric[vf].base_sl != vFabricRecord.s1.sl) continue;
		}

		if (samad.header.mask & STL_VFINFO_REC_COMP_SERVICEID) {
			if (VSTATUS_OK != smVFValidateVfServiceId(vf, vFabricRecord.ServiceID) ) continue;
		}

		if (samad.header.mask & STL_VFINFO_REC_COMP_MGID) {
			if (VSTATUS_OK != smVFValidateVfMGid(vf, (uint64_t*)vFabricRecord.MGID.Raw) ) continue;
		}

		// If requestor is not a member of the VF and requestor is not
		// a full member of DEFAULT_PKEY (0x7fff), skip this VF 
		// convert VF pkey to a FULL PKey so we allow limited member reqPortp
		if (!reqInFullDefault
			&& !smValidatePortPKey(MAKE_PKEY(PKEY_TYPE_FULL, 
				VirtualFabrics->v_fabric[vf].pkey), reqPortp)) continue;
		
		if ((status = sa_check_len(data, sizeof(STL_VFINFO_RECORD), bytes)) != VSTATUS_OK) {
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
			IB_LOG_ERROR_FMT( "sa_VFabric_GetTable", "Reached size limit at %d records", *records);
			break;
		}

		// VFabric_Set doesn't use the samad paramater, so for the time being just type cast. (Should work either way).
		sa_VFabric_Set(data, vf, &samad, serviceId, mGid);
		BSWAP_STL_VFINFO_RECORD((STL_VFINFO_RECORD*)data);
	
		if (samad.header.mask) {
			(void)sa_template_test(samad.data, &data, sizeof(STL_VFINFO_RECORD), bytes, records);
		} else {
			sa_increment_and_pad(&data, sizeof(STL_VFINFO_RECORD), bytes, records);
		}
	
		// pkey not necessarily a unique identifier.
		if (samad.header.mask & STL_VFINFO_REC_COMP_INDEX) goto reply_vFabric;
		if (samad.header.mask & STL_VFINFO_REC_COMP_NAME) goto reply_vFabric;
	}

	if ((samad.header.mask & STL_VFINFO_REC_COMP_PKEY) && (*records == 0)) {
		if (saDebugPerf) {
       		IB_LOG_INFINI_INFO_FMT("sa_VFabric_GetTable",
				"No Virtual Fabric defined with PKey 0x%x", vFabricRecord.pKey);
		}
	}

	if ((samad.header.mask & STL_VFINFO_REC_COMP_SL) && (*records == 0)) {
		if (saDebugPerf) {
       		IB_LOG_INFINI_INFO_FMT("sa_VFabric_GetTable",
				"No Virtual Fabric defined with SL %u", vFabricRecord.s1.sl);
		}
	}

	if ((samad.header.mask & STL_VFINFO_REC_COMP_SERVICEID) && (*records == 0)) {
		if (saDebugPerf) {
       		IB_LOG_INFINI_INFO_FMT("sa_VFabric_GetTable",
				"No Virtual Fabric defined with ServiceId "FMT_U64, vFabricRecord.ServiceID);
		}
	}

	if ((samad.header.mask & STL_VFINFO_REC_COMP_MGID) && (*records == 0)) {
		if (saDebugPerf) {
       		IB_LOG_INFINI_INFO_FMT("sa_VFabric_GetTable",
				"No Virtual Fabric defined with MGID "FMT_GID,
		   		vFabricRecord.MGID.Type.Global.SubnetPrefix,
				vFabricRecord.MGID.Type.Global.InterfaceID);
		}
	}

	if ((samad.header.mask & STL_VFINFO_REC_COMP_INDEX) && (*records == 0)) {
		if (saDebugPerf) {
       		IB_LOG_INFINI_INFO_FMT("sa_VFabric_GetTable",
				"No Virtual Fabric defined with VF Index %d", vFabricRecord.vfIndex);
		}
	}

	if ((samad.header.mask & STL_VFINFO_REC_COMP_NAME) && (*records == 0)) {
		if (saDebugPerf) {
       		IB_LOG_INFINI_INFO_FMT( "sa_VFabric_GetTable",
				"No Virtual Fabric defined with VF Name %s", vFabricRecord.vfName);
		}
	}


reply_vFabric:
	(void)vs_rwunlock(&old_topology_lock);

	IB_EXIT("sa_VFabric_GetTable", status);
	return(status);
}

void showStlVFabrics(void) {
	STL_VFINFO_RECORD vFabricRecord;
	int vf;
	IB_GID mGid = (IB_GID){.Raw = {0}}; // only reported if in samad.header.mask
    PrintDest_t dest; 
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (!VirtualFabrics)
        return;

	memset(&vFabricRecord, 0, sizeof(vFabricRecord));
    PrintDestInitFile(&dest, stdout);
     
	(void)vs_rdlock(&old_topology_lock);

	for (vf=0; vf < VirtualFabrics->number_of_vfs && vf < MAX_VFABRICS; vf++) {
		sa_VFabric_Set((uint8_t *)&vFabricRecord, vf, NULL, 0, mGid);

        if (vf) PrintSeparator(&dest);
        PrintStlVfInfoRecord(&dest, 0, &vFabricRecord);
	}

	(void)vs_rwunlock(&old_topology_lock);
}
