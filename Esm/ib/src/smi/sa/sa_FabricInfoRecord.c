/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_FabricInfoRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the FabricInfoRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_FabricInfoRecord							     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//===========================================================================//


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"

Status_t
sa_FabricInfoRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	STL_FABRICINFO_RECORD		myFI = { 0 };
	uint16_t				attribOffset;
	Node_t *nodep;

	IB_ENTER("sa_FabricInfoRecord", maip, 0, 0, 0);

	// Check Method
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetFabricInfoRecord);
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_METHOD;
		IB_LOG_WARN_FMT(__func__, "invalid Method: %s (%u)",
			cs_getMethodText(maip->base.method), maip->base.method);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}
	// Check Base and Class Version
	if (maip->base.bversion == STL_BASE_VERSION && maip->base.cversion == STL_SA_CLASS_VERSION) {
		//sa_FabricInfoRecord_Get()
	} else {
		// Generate an error response and return.
		maip->base.status = MAD_STATUS_BAD_CLASS;
		IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
			maip->base.bversion, maip->base.cversion);
		(void)sa_send_reply(maip, sa_cntxt);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}

	// Lock the topology
	(void)vs_rdlock(&old_topology_lock);
	myFI.NumHFIs = old_topology.num_nodes - old_topology.num_sws;
	myFI.NumSwitches = old_topology.num_sws;
	for_all_nodes(&old_topology, nodep) {
		Port_t *portp;
		for_all_physical_ports(nodep, portp) {
			Node_t *neighborNodep = NULL;
			int isl = 0;
			int internal = 0;
			int degraded = 0;
			int omitted = 0;
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
				continue;

			if (portp->portData->neighborQuarantined) {
				// quarantined neighbors will just have the trusted port
				// in topology and QuaratinedNode_t will have only the
				// NodeInfo reported by the untrusted SMA, but no PortInfo
				neighborNodep = sm_find_quarantined_guid(&old_topology, portp->portData->portInfo.NeighborNodeGUID);
				// use only trusted information where possible
				if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
					&& portp->portData->portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW)
					isl = 1;
				// this could be a lie from the untrusted node but its the
				// best we can do
				if (neighborNodep && nodep->nodeInfo.SystemImageGUID ==
						neighborNodep->nodeInfo.SystemImageGUID)
					internal = 1;
				// for ports which are omitted, don't report degraded
				// In this case we also lack details for other side of link
				omitted = 1;
			} else {
				Port_t *neighborPortp;
				uint16 lwe; // best link width possible based on enabled
				uint16 lse; // best link speed possible based on enabled

				neighborPortp = sm_find_neighbor_node_and_port(&old_topology, portp, &neighborNodep);
				if (!neighborPortp || !neighborNodep)
					continue;
				// to avoid double counting, we only process when we have
				// nodep/portp as the lower NodeGuid and portNum of the link
				if (nodep->nodeInfo.NodeGUID > neighborNodep->nodeInfo.NodeGUID)
					continue;
				if (nodep->nodeInfo.NodeGUID == neighborNodep->nodeInfo.NodeGUID
					&& portp->index >= neighborPortp->index)
					continue;

				if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
					&& neighborNodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
					isl = 1;
				if (nodep->nodeInfo.SystemImageGUID ==
						neighborNodep->nodeInfo.SystemImageGUID)
					internal = 1;

				// Note Enabled checks could race if opaportconfig changes
				// width or speed but has not yet bounced the link
				//
				// Best width link could come up at
				lwe = StlBestLinkWidth(portp->portData->portInfo.LinkWidth.Enabled
					& neighborPortp->portData->portInfo.LinkWidth.Enabled);
				// Best speed link could come up at
				lse = StlBestLinkSpeed(portp->portData->portInfo.LinkSpeed.Enabled
					& neighborPortp->portData->portInfo.LinkSpeed.Enabled);

				// Active on both sides of link should match but
				// to be safe we check both sides of link
				if (portp->portData->portInfo.LinkWidthDowngrade.RxActive != lwe
					|| portp->portData->portInfo.LinkWidthDowngrade.TxActive != lwe
					|| neighborPortp->portData->portInfo.LinkWidthDowngrade.RxActive != lwe
					|| neighborPortp->portData->portInfo.LinkWidthDowngrade.TxActive != lwe
					|| portp->portData->portInfo.LinkSpeed.Active != lse
					|| neighborPortp->portData->portInfo.LinkSpeed.Active != lse) {
					degraded = 1;
				}

				if (portp->portData->portInfo.PortStates.s.PortState == IB_PORT_INIT) {
					omitted = 1;
					degraded = 0;   // don't report degraded for omitted ports
				}
			}
			if (isl) {
				if (internal)
					myFI.NumInternalISLs++;
				else
					myFI.NumExternalISLs++;
			} else {
				if (internal)
					myFI.NumInternalHFILinks++;
				else
					myFI.NumExternalHFILinks++;
			}
			if (degraded) {
				if (isl)
					myFI.NumDegradedISLs++;
				else
					myFI.NumDegradedHFILinks++;
			}
			if (omitted) {
				if (isl)
					myFI.NumOmittedISLs++;
				else
					myFI.NumOmittedHFILinks++;
			}
		} // for_all_physical_ports
	} // for_all_nodes
	(void)vs_rwunlock(&old_topology_lock);

	BSWAP_STL_FABRICINFO_RECORD(&myFI);

	attribOffset = sizeof(STL_FABRICINFO_RECORD) + Calculate_Padding(sizeof(STL_FABRICINFO_RECORD));

	sa_cntxt_data(sa_cntxt, &myFI, attribOffset);
	sa_cntxt->attribLen = attribOffset;
	maip->base.status = MAD_STATUS_OK;
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_FabricInfoRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}
