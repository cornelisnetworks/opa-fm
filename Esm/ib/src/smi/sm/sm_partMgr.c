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

//===========================================================================//
//																		     //
// FILE NAME															     //
//    sm_partMgr.c														     //
//																		     //
// DESCRIPTION															     //
//    This file contains the SM Partition Management routines.				 //
//									    									 //
//===========================================================================//

#include "sm_l.h"
#include "sa_l.h"

#ifdef IB_STACK_OPENIB
#include "mal_g.h"
#endif


static STL_PARTITION_TABLE defaultPKeyTable = { {{ STL_DEFAULT_APP_PKEY }, { STL_DEFAULT_PKEY }} };

static uint16_t defaultPKeys[PKEY_TABLE_LIST_COUNT] = { STL_DEFAULT_APP_PKEY, STL_DEFAULT_PKEY };

int sm_check_node_cache_valid(Node_t *);

#define SET_PORT_STL_PKEY(key1, key2) { \
			pkeyEntry1[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 = key1; \
			pkeyEntry2[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 = key1; \
			pkeyEntry1[STL_DEFAULT_FM_PKEY_IDX].AsReg16 = key2; \
			pkeyEntry2[STL_DEFAULT_FM_PKEY_IDX].AsReg16 = key2; \
}

//----------------------------------------------------------------------------//
static void sm_set_port_stl_mngnt_pkey(Node_t *nodep, Port_t *portp, uint16_t pkey,
									   STL_PKEY_ELEMENT *pkeyEntry1, STL_PKEY_ELEMENT *pkeyEntry2)
{

    // Section 15.4.2 Switch of the STL Volumne 1g1 Specification:
    // Port 0 is a special "internal port" inside a switch ASIC which is not externally accessible. This port is the
    // path to the firmware inside the switch and is where the switch management agents reside. Port 0 is
    // inside the fabric trust boundary, as such it can send packets with P_Key M, and is required to as part of
    // passing along a directed route SMA request. Port 0 is trusted not to abuse this capability.
    //
    // There is also a concept of "Enhanced Port 0" which is a port 0 which may have additional capabilities.
    // For example, the management card in a switch might be "Enhanced Port 0"on the switch ASIC, in
    // which case the management card can participate in more advanced protocols such as IPoIB, etc. Such
    // management card is still inside the trust boundary and is running more sophisticated switch firmware.
    if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
        portp->index == 0 &&
        !nodep->switchInfo.u2.s.EnhancedPort0) {
        // set pkeys for Switch Port 0
        SET_PORT_STL_PKEY(0, STL_DEFAULT_FM_PKEY);
    } else if (PKEY_TYPE(pkey) != PKEY_TYPE_FULL) {
		// set pkeys for non-managed node 
		SET_PORT_STL_PKEY(STL_DEFAULT_CLIENT_PKEY, 0);
	} else {
		// set pkeys for managed node
		if ((nodep->nodeInfo.NodeType == NI_TYPE_CA) ||
			(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && (portp->index == 0 && nodep->switchInfo.u2.s.EnhancedPort0))) {
			// HFIs and switches with enhancedPort0 enabled
			SET_PORT_STL_PKEY(STL_DEFAULT_CLIENT_PKEY, STL_DEFAULT_FM_PKEY);
			if ((nodep->index == 0) && (portp->index == sm_config.port)) {
				setPKey(STL_DEFAULT_CLIENT_PKEY_IDX, STL_DEFAULT_CLIENT_PKEY, 0);
				setPKey(STL_DEFAULT_FM_PKEY_IDX, STL_DEFAULT_FM_PKEY, 0);
			}
		} else {
			// switches connected to managed HFIs
			SET_PORT_STL_PKEY(0, STL_DEFAULT_FM_PKEY);
		}
	}
}

// sm_truncate_Pkeys()
// - Truncates the SMA and SA Pkey tables for the given capacities, while preserving managment
//   pkey entries.
//
//   Parameters:
//     pkeyCap - Capacity of pkey table on the SMA we will be generating for
//     pkeyCnt - Current size of SMA and SA PKey Tables
//     sma_pkey - Pointer to SMA PKey table we need to truncate
//     sa_pkey - Pointer to SA PKey table we need to truncate
//
static void
sm_truncate_Pkeys(Port_t *portp, int pkeyCap, int pkeyCnt, STL_PKEY_ELEMENT *sma_pkey, STL_PKEY_ELEMENT *sa_pkey)
{
	uint8_t pkeys;
	uint16_t pkey;
	boolean defaultDefined = FALSE;

	
	for (pkeys=0; pkeys<pkeyCap; pkeys++) {
		if (PKEY_VALUE(sma_pkey[pkeys].AsReg16) == DEFAULT_PKEY) {
			defaultDefined = TRUE;
		}
	}

	pkey = DEFAULT_PKEY;
	for (pkeys=pkeyCap; pkeys<pkeyCnt; pkeys++) {
		if (PKEY_VALUE(sma_pkey[pkeys].AsReg16) == DEFAULT_PKEY) {
			pkey = sma_pkey[pkeys].AsReg16;
		}
		bitset_clear(&portp->portData->pkey_idxs, pkeys);
		sma_pkey[pkeys].AsReg16 = 0;
		sa_pkey[pkeys].AsReg16 = 0;
	}

	if (!defaultDefined) {
		sma_pkey[pkeyCap-1].AsReg16 = pkey;
		sa_pkey[pkeyCap-1].AsReg16 = pkey;
	}
}

// sm_revise_vf_memberships()
// - Removes a port from VirtualFabrics it no longer belongs to based on its pkey table
//   contents. Will additionally remove the containing node's memberships to the VF if the
//   given port was the ONLY port of the node in said VF.
//
//   Parameters:
//     portData - Pointer to the port data structure of the port we're working on
//     pkeyTable - Pointer to pkey table we will use as basis for membership
//     pkeyCnt   - Size of the given pkey table
//     VirtualFabrics - Pointer to SM VirtualFabrics object.
static void
sm_revise_vf_memberships(PortData_t *portData, STL_PKEY_ELEMENT *pkeyTable, unsigned int pkeyCnt, VirtualFabrics_t *VirtualFabrics)
{
	Node_t *nodep = portData->nodePtr;
	unsigned int vf, i;

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;

		if (bitset_test(&portData->vfMember, vf)) {
			const uint16_t current_pkey = PKEY_VALUE(vf_config.vf[vf]->pkey);
			boolean removed = TRUE;

			for (i = 0; i < pkeyCnt; ++i) {
				if (PKEY_VALUE(pkeyTable[i].AsReg16) == current_pkey) {
					removed = FALSE;
					break;
				}
			}

			if (removed) {
				Port_t *current;
				boolean found = FALSE;

				bitset_clear(&portData->vfMember, vf);
				bitset_clear(&portData->fullPKeyMember, vf);

				// Check if we were the only port on this node with this VF membership.
				for_all_ports(nodep, current) {
					if (!sm_valid_port(current) || current->state <= IB_PORT_DOWN || current->portData == portData)
						continue;
					else if (bitset_test(&current->portData->vfMember, vf)) {
						found = TRUE;
						break;
					}
				}

				if (!found) {
					bitset_clear(&nodep->vfMember, vf);
					bitset_clear(&nodep->fullPKeyMember, vf);
				}
			}
		}
	}
}

static inline void
VfMapped(bitset_t *list, VirtualFabrics_t *VirtualFabrics, int vfIdx, uint16_t *vlHighLimit, uint8_t *enforcePkey)
{
	if (enforcePkey && VirtualFabrics->v_fabric_all[vfIdx].security)
		*enforcePkey = 1;
	if (VirtualFabrics->v_fabric_all[vfIdx].qos_enable &&
		VirtualFabrics->v_fabric_all[vfIdx].priority) {
		*vlHighLimit = *vlHighLimit + 1;
	}
	bitset_clear(list, vfIdx);
}

// sm_compute_portPkeys()
// - Computes the actual SMA PKey table and SA Pkey table for a given port.
//
//   Parameters:
//     nodep - Node we're computing PKey tables for
//	   portp - Port we're computing PKey tables for
//     linkedNodep - neighbor node
//     linkedPortp - neighbor port
//	   VirtualFabrics - Pointer to VirtualFabrics structure
//	   sma_pkey - Pointer to where we should write SMA PKey table
//	   sa_pkey - Pointer to where we should write SA PKey table
//	   vlHighLimit - Pointer to vlHighlimit **INCLUDED FOR OPTIMIZATION
//	   enforcePkey - Pointer to pkey enforcement flag **INCLUDED FOR OPTIMIZATION
//
// Returns:
//	   Limit of in-use entries in sma_pkey and sa_pkey.
//
// Note: vlHighLimit and enforcePkey have little to do with computing port Pkeys, but are
//       included here to prevent looping over VirtualFabric structures excessively.
//
static int
sm_compute_portPkeys(Node_t *nodep, Port_t *portp, Node_t *linkedNodep, Port_t *linkedPortp,
					VirtualFabrics_t *VirtualFabrics, STL_PKEY_ELEMENT *sma_pkey,
					STL_PKEY_ELEMENT *sa_pkey, uint16_t *vlHighLimit, uint8_t *enforcePkey)
{
	int pkIdx, vfIdx, vfLoop;
	uint16_t pk;
	boolean fullMgmt = FALSE;
	bitset_t *vfMember, *fullPKeyMember;
	bitset_t unmappedVfs;
	int cap; 		// Size of this PKey table
	int freePkeys;	// Number of PKeys slots we can use
	char *nodeName = NULL;
	Guid_t nodeGuid;
	uint8_t portNum;
	int missedVfs = 0;

	bitset_clear_all(&portp->portData->pkey_idxs);
	// Mark the management slots as in-use so we don't put anything else there
	bitset_set(&portp->portData->pkey_idxs,STL_DEFAULT_CLIENT_PKEY_IDX);
	bitset_set(&portp->portData->pkey_idxs,STL_DEFAULT_FM_PKEY_IDX);

	// Interswitch links only have management pkeys (and no enforcement)
	if (portp->portData->isIsl)
		goto management_pkeys;

	// If this is a switch port (not port 0), then
	// calculate PKeys based on the attached HFI VF membership
	// Don't use more PKeys then the minimum of this port and
	// the neighbor port (freePkeys). However, we can use any
	// Pkey index up to cap (cap is always less than or equal
	// to starting freePkeys)
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && !is_swport0(portp)) {
		cap = nodep->switchInfo.PartitionEnforcementCap;
		freePkeys = MIN(nodep->switchInfo.PartitionEnforcementCap,linkedNodep->nodeInfo.PartitionCap);
		vfMember = &linkedPortp->portData->vfMember;
		fullPKeyMember = &linkedPortp->portData->fullPKeyMember;
		nodeName = sm_nodeDescString(linkedNodep);
		nodeGuid = linkedNodep->nodeInfo.NodeGUID;
		portNum = linkedPortp->index;
	} else {
		if (is_swport0(portp)) {
			cap = freePkeys = nodep->nodeInfo.PartitionCap;
		} else if (linkedNodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			// Back-to-back HFIs
			cap = nodep->nodeInfo.PartitionCap;
			freePkeys = MIN(nodep->nodeInfo.PartitionCap,linkedNodep->nodeInfo.PartitionCap);
		} else {
			cap = nodep->nodeInfo.PartitionCap;
			freePkeys = MIN(nodep->nodeInfo.PartitionCap,linkedNodep->switchInfo.PartitionEnforcementCap);
		}
		vfMember = &portp->portData->vfMember;
		fullPKeyMember = &portp->portData->fullPKeyMember;
		nodeName = sm_nodeDescString(nodep);
		nodeGuid = nodep->nodeInfo.NodeGUID;
		portNum = portp->index;
		// We won't be updating enforcePkey if this is an HFI
		enforcePkey = NULL;
	}

	freePkeys -= 2;

	// Keep track of the VFs that we need to map PKeys for
	bitset_init(&sm_pool, &unmappedVfs, bitset_nbits(vfMember));
	bitset_copy(&unmappedVfs, vfMember);

	for (vfIdx = 0; (vfIdx = bitset_find_next_one(&unmappedVfs, vfIdx)) != -1; vfIdx++) {
		boolean full_member = FALSE;

		// Find the pk we want to add to the table
		pk = PKEY_VALUE(VirtualFabrics->v_fabric_all[vfIdx].pkey);

		// If we are a full member of this VF or if we are a limited member of a non-security VF
		if (bitset_test(fullPKeyMember,vfIdx) || PKEY_TYPE(VirtualFabrics->v_fabric_all[vfIdx].pkey))
			full_member = TRUE;

		// Management pkeys are handled special
		if (pk == DEFAULT_PKEY) {
			if (full_member) fullMgmt = TRUE;
			VfMapped(&unmappedVfs, VirtualFabrics, vfIdx, vlHighLimit, enforcePkey);
			continue;
		}

		/* Need to add pk to port. See if there is room */
		pkIdx = bitset_find_first_zero(&portp->portData->pkey_idxs);
		if (pkIdx == -1 || !freePkeys) {
			bitset_clear(&unmappedVfs, vfIdx);
			bitset_clear(vfMember, vfIdx);
			bitset_clear(fullPKeyMember, vfIdx);
			missedVfs++;
			continue;
		}

		VfMapped(&unmappedVfs, VirtualFabrics, vfIdx, vlHighLimit, enforcePkey);
		// See if any other VFs use the same pkey
		for (vfLoop = vfIdx+1; (vfLoop = bitset_find_next_one(&unmappedVfs, vfLoop)) != -1; vfLoop++) {
			if (pk == PKEY_VALUE(VirtualFabrics->v_fabric_all[vfLoop].pkey)) {
				// If we are a full member of this VF or if we are a limited member of a non-security VF
				if (bitset_test(fullPKeyMember,vfLoop) || PKEY_TYPE(VirtualFabrics->v_fabric_all[vfLoop].pkey))
					full_member = TRUE;
				// Processed this VF's pkey, don't have to process it later.
				VfMapped(&unmappedVfs, VirtualFabrics, vfLoop, vlHighLimit, enforcePkey);
			}
		}
		// Store the pkey. Update the in-use indexes.
		bitset_set(&portp->portData->pkey_idxs,pkIdx);
		if (full_member) pk |= FULL_MEMBER;
		sma_pkey[pkIdx].AsReg16 = pk;
		sa_pkey[pkIdx].AsReg16 = pk;
		freePkeys--;
	}

	if (missedVfs) {
		IB_LOG_WARN_FMT(__func__,
						"Node %s ["FMT_U64":%d] Unable to add node "
						"to %d VF%s (too many unique PKEYs defined in VFs), MAX of %d",
						nodeName, nodeGuid, portNum, missedVfs, (missedVfs>1)?"s":"", cap);
	}

	bitset_free(&unmappedVfs);

management_pkeys:
	// Handle management pkeys
	(void)sm_set_port_stl_mngnt_pkey(nodep, portp, fullMgmt ? STL_DEFAULT_FM_PKEY : STL_DEFAULT_CLIENT_PKEY, sma_pkey, sa_pkey);

	return bitset_find_last_one(&portp->portData->pkey_idxs) + 1;
}

Status_t
sm_set_portPkey(
	Topology_t *topop, Node_t *nodep, Port_t *portp, Node_t *linkedNodep, Port_t *linkedPortp,
	SmpAddr_t *addr, uint8_t *enforcePkey, uint16_t	*vlHighLimit)
{
	uint32_t    amod;
	Status_t	status=VSTATUS_OK;
	STL_PKEY_ELEMENT *otherPkey, *pkeyTable;
	int			pkeyCap=0;
	int			pkeyEntryCntr=0;
	static int	pkeyExceededAlarm=0;
	uint8_t		numBlocks=1;
	uint8_t		pkeyWriteCnt;

	/* Set up the pkey port table */
	pkeyTable = (STL_PKEY_ELEMENT*)topop->pad;
	otherPkey = &(((STL_PKEY_ELEMENT*)topop->pad)[SM_PKEYS]);
	memset(topop->pad, 0, 2 * sizeof(PKey_t) * SM_PKEYS);

	*vlHighLimit = 0;

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;

	pkeyEntryCntr = sm_compute_portPkeys(nodep, portp, linkedNodep, linkedPortp, VirtualFabrics, pkeyTable,
					otherPkey, vlHighLimit, enforcePkey);

	/* 
     * IB spec 10.9.2 C10-120 requirement - switch external ports won't 
     * have pkey tables if switchInfo->enforcementcap is zero, so don't ask
     */
	if (is_hfiport(portp) ||
		(portp->index == 0 && nodep->nodeInfo.PartitionCap) ||
		(portp->index > 0 && nodep->switchInfo.PartitionEnforcementCap))
	{
		/* for switch port0 and FIs, partitionCap is in nodep->nodeInfo.PartitionCap 
		 * Otherwise, for switch external ports, use switchInfo->partCap.
		 */
		if ( is_swport0(portp) || is_hfiport(portp) )
			pkeyCap = nodep->nodeInfo.PartitionCap;
		else
			pkeyCap = nodep->switchInfo.PartitionEnforcementCap;

		/* non-ISL (i.e. SW<->HFI, HFI<->HFI) links should not use more pkeys than their neighbor. */
		if (linkedPortp && !portp->portData->isIsl)
			pkeyCap = MIN(pkeyCap, linkedPortp->portData->nodePtr->nodeInfo.PartitionCap);

		if (pkeyCap < pkeyEntryCntr) {
			if (*enforcePkey) {
				*enforcePkey = 0;
				if (pkeyExceededAlarm++ < 20) {
					IB_LOG_ERROR_FMT(__func__,
           				"Switch %s ["FMT_U64":%d] partition cap = %d, pkeys configured for use = %d on remote FI port, no enforcement",
           				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkeyCap, pkeyEntryCntr);
				}
			} else if (pkeyExceededAlarm++ < 20) {
				IB_LOG_ERROR_FMT(__func__,
						"Node %s ["FMT_U64":%d] partition cap = %d, pkeys configured for use = %d, truncating pkey table, "
						"Virtual Fabric memberships may be affected!",
           				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkeyCap, pkeyEntryCntr);
			}

			sm_truncate_Pkeys(portp, pkeyCap, pkeyEntryCntr, pkeyTable, otherPkey);
			sm_revise_vf_memberships(portp->portData, pkeyTable, pkeyCap, VirtualFabrics);

		}
		IB_LOG_DEBUG1_FMT(__func__,
               "Node %s ["FMT_U64":%d] partition cap = %d, pkey = 0x%x", 
               sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkeyCap, otherPkey[0].AsReg16);

		if (!portp->portData->current.pkeys)
			return VSTATUS_BAD;

		/* Note: If node was previously non-responding break out */
		if (nodep->nonRespCount)
			return VSTATUS_OK;

		if (sm_config.forceAttributeRewrite || nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
			pkeyWriteCnt = pkeyCap;
		else
			pkeyWriteCnt = bitset_find_last_one(&portp->portData->pkey_idxs) + 1;

		numBlocks = (pkeyWriteCnt + NUM_PKEY_ELEMENTS_BLOCK - 1) / NUM_PKEY_ELEMENTS_BLOCK;
		amod = (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? portp->index << 16 : 0) | ((0xff & numBlocks)<<24);

		if (memcmp((void *)portp->portData->pPKey, pkeyTable, pkeyWriteCnt * sizeof(STL_PKEY_ELEMENT)) || sm_config.forceAttributeRewrite) {
			if (!portp->portData->isIsl || *enforcePkey) {
				status = SM_Set_PKeyTable(fd_topology, amod, addr, (STL_PARTITION_TABLE*)pkeyTable, sm_config.mkey);
				if (status != VSTATUS_OK) {
					IB_LOG_WARN_FMT(__func__,
						   "Failed to set Partition Table for node %s guid "FMT_U64" node index %d port index %d",
						   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
					return(status);
				}
#if defined(IB_STACK_OPENIB)
				if ((nodep->index == 0) && (portp->index == sm_config.port)) {
					IB_LOG_INFINI_INFO0("sm pkey table refresh");
					status = ib_refresh_devport();
					if (status != VSTATUS_OK) {
						IB_LOG_ERRORRC("cannot refresh sm pkeys rc:", status);
					}
				}
#endif
				// Overwrite the SA repository PKey value, as we just reprogrammed the one on the device.
				memcpy(portp->portData->pPKey, otherPkey, numBlocks * sizeof(STL_PARTITION_TABLE));
			}

			// For ISLs or ports w/o PKey enforcement, leave SA repository PKey values as they are.
			// Their value should reflect what's actually on the device from an earlier Get(), so they
			// should be PoD PKey values for switch ports.
	
		} else {
			IB_LOG_DEBUG1_FMT(__func__,
				   "Node %s ["FMT_U64"] Pkey table already setup",
				   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
		}
	}

	return(status);
}

Status_t
sm_set_local_port_pkey(STL_NODE_INFO *nodeInfop)
{
	Status_t	status=VSTATUS_OK;
    STL_PARTITION_TABLE pkeyTable = {{{0}}};
	uint8_t		path[64] = {0};
	uint8		needsSet = 0;
	SmpAddr_t	addr = SMP_ADDR_CREATE_DR(path);

	status = SM_Get_PKeyTable(fd_topology, 1<<24, &addr, &pkeyTable);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
						"Failed to get Partition Table for local node guid "FMT_U64" port index %d",
						nodeInfop->NodeGUID, sm_config.port);
		return(status);
	}
	if (pkeyTable.PartitionTableBlock[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 != STL_DEFAULT_CLIENT_PKEY) {
		pkeyTable.PartitionTableBlock[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 = STL_DEFAULT_CLIENT_PKEY;
		needsSet = 1;
	}
	if (pkeyTable.PartitionTableBlock[STL_DEFAULT_FM_PKEY_IDX].AsReg16 != STL_DEFAULT_FM_PKEY) {
		pkeyTable.PartitionTableBlock[STL_DEFAULT_FM_PKEY_IDX].AsReg16 = STL_DEFAULT_FM_PKEY;
		needsSet = 1;
	}

    IB_LOG_INFO_FMT(__func__,
           "Local node ["FMT_U64":%d] partition cap = %d, pkey = 0x%x", 
           nodeInfop->NodeGUID, sm_config.port, nodeInfop->PartitionCap, pkeyTable.PartitionTableBlock[0].AsReg16);
	if (!needsSet) {
        IB_LOG_INFINI_INFO0("sm pkey table already set");
		return(status);
	}
	status = SM_Set_PKeyTable(fd_topology, 1<<24, &addr, &pkeyTable, sm_config.mkey);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
			   "Failed to set Partition Table for local node guid "FMT_U64" port index %d",
			   nodeInfop->NodeGUID, sm_config.port);
		return(status);
	}
#if defined(IB_STACK_OPENIB)
	IB_LOG_INFINI_INFO0("sm pkey table refresh");
	status = ib_refresh_devport();
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("cannot refresh sm pkeys rc:", status);
	}
#endif

	return(status);
}

//---------------------------------------------------------------------------//

Status_t
setPKey(uint32_t index, uint16_t pkey, int forceSweep)
{
	if(index > PKEY_TABLE_LIST_COUNT-1){
		return(VSTATUS_ILLPARM);
	}

	defaultPKeys[index] = pkey;
	defaultPKeyTable.PartitionTableBlock[index].AsReg16 = pkey;

	if(forceSweep)
		sm_forceSweep("PKey Table Change");

	return(VSTATUS_OK);
}

uint16_t getPKey(uint8_t index) 
{
	if (index > PKEY_TABLE_LIST_COUNT - 1) {
		return 0;
	}
	return defaultPKeys[index];
}

// Verify SM knows about the pkey.
int checkPKey(uint16_t pkey)
{
	int i;

	for (i = 0; i < PKEY_TABLE_LIST_COUNT; ++i) {
		if (PKEY_VALUE(defaultPKeys[i]) == PKEY_VALUE(pkey)) {
			break;
		}
	}
	return (i < PKEY_TABLE_LIST_COUNT) ? i : -1;
}

uint16_t
smGetPortPkey(uint16_t pkey, Port_t* portp)
{
	int i;

	// Because the management pkey may occur twice in the pkey table
	// it requires special handling to avoid returning the limited
	// membership key when the full membership key is required.
	if (IB_EXPECT_FALSE(PKEY_VALUE(pkey) == STL_DEFAULT_CLIENT_PKEY)) {
		if (portp->portData->pPKey[STL_DEFAULT_FM_PKEY_IDX].AsReg16 
			== STL_DEFAULT_FM_PKEY) {
			return STL_DEFAULT_FM_PKEY;
		}
	}
	for (i = 0; i < PKEY_TABLE_LIST_COUNT; ++i) {
		if (PKEY_VALUE(portp->portData->pPKey[i].AsReg16) == PKEY_VALUE(pkey)) return portp->portData->pPKey[i].AsReg16;
	}
	return 0;
}

int
smCheckPortPKey(uint16_t pkey, Port_t* portp)
{
	int i;

	for (i = 0; i < PKEY_TABLE_LIST_COUNT; ++i) {
		if (PKEY_VALUE(portp->portData->pPKey[i].AsReg16) == PKEY_VALUE(pkey)) return 1;
	}
	return 0;
}

int
smValidatePortPKey(uint16_t pkey, Port_t* portp)
{
	int i;

	for (i = 0; i < PKEY_TABLE_LIST_COUNT; ++i) {
		if ((PKEY_VALUE(portp->portData->pPKey[i].AsReg16) == PKEY_VALUE(pkey)) &&
			((PKEY_TYPE(pkey) == PKEY_TYPE_FULL) ||
			 (PKEY_TYPE(portp->portData->pPKey[i].AsReg16) == PKEY_TYPE_FULL))) {
			return 1;
		}
	}
	return 0;
}

int
smValidateGsiMadPKey(Mai_t *maip, uint8_t mgmntAllowedRequired, uint8_t antiSpoof)
{ 
    int valid = 0; 
    Node_t *pNode; 
    Port_t *pPort; 

    // Anti-spoof checking disabled; just check the pkey in the mad packet for managment allowed.
    if (antiSpoof == 0) {
        if (mgmntAllowedRequired) {
            if (maip->addrInfo.pkey==STL_DEFAULT_FM_PKEY)
                valid = 1;    // validate PKEY for full member
        } else {
            if ((maip->addrInfo.pkey==STL_DEFAULT_FM_PKEY) || 
                (maip->addrInfo.pkey==STL_DEFAULT_CLIENT_PKEY))
                valid = 1;    
        }

        return valid;
    }


    // Antispoof is enabled...check the pkey as well as lookup the pkeys in the SM database
    // to make sure no one is lying about their pkey.
    (void)vs_rdlock(&old_topology_lock); 
    
    if ((pPort = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, &pNode)) != NULL) {
        if (sm_valid_port(pPort) && pPort->state > IB_PORT_DOWN) {
            if (mgmntAllowedRequired) {
                if (maip->addrInfo.pkey == STL_DEFAULT_FM_PKEY)
                    valid = sm_valid_port_mgmt_allowed_pkey(pPort);    // validate PKEY for full member
            } else {
                if (pNode->nodeInfo.NodeType == NI_TYPE_CA) {
                    if (maip->addrInfo.pkey == STL_DEFAULT_FM_PKEY)
                        valid = sm_valid_port_mgmt_allowed_pkey(pPort);    // validate PKEY for full member HFI
                    else if (maip->addrInfo.pkey == STL_DEFAULT_CLIENT_PKEY) {
                        // validate PKEY for limited member HFI
                        if (PKEY_VALUE(pPort->portData->pPKey[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16) == PKEY_VALUE(STL_DEFAULT_CLIENT_PKEY))
                            valid = 1; 
                    }
                } else if (pNode->nodeInfo.NodeType == NI_TYPE_SWITCH) {
                    // Only switch port 0 should be assigned a lid.
                    if (pPort->index == 0) {
                        if (pNode->switchInfo.u2.s.EnhancedPort0) {
                            // Enhanced Port 0 can use either limited or full pkey.
                            if (maip->addrInfo.pkey == STL_DEFAULT_FM_PKEY) {
                                    valid = sm_valid_port_mgmt_allowed_pkey(pPort);    // validate PKEY for full member on ESP0
                            } else if (maip->addrInfo.pkey == STL_DEFAULT_CLIENT_PKEY) {
                                if (PKEY_VALUE(pPort->portData->pPKey[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16) == PKEY_VALUE(STL_DEFAULT_CLIENT_PKEY))
                                    valid = 1; 
                            }
                        // Base port 0 must _only_ use full pkey.
                        } else if (maip->addrInfo.pkey == STL_DEFAULT_FM_PKEY)
							valid = sm_valid_port_mgmt_allowed_pkey(pPort);
                    } 
                }
            }
        } 
    }
    
    (void)vs_rwunlock(&old_topology_lock);
    
    return valid;
}

int
smValidatePortPKey2Way(uint16_t pkey, Port_t* port1p, Port_t* port2p)
{
	int i, j;

	if (port1p == port2p) return 1;

	// Special handling required for managment pkey.
	if (IB_EXPECT_FALSE(PKEY_VALUE(pkey) == STL_DEFAULT_CLIENT_PKEY)) {
		// A special case may exist where the FM PKEY is actually in index 1 instead of 2.
		if (port1p->portData->pPKey[STL_DEFAULT_FM_PKEY_IDX].AsReg16 == STL_DEFAULT_FM_PKEY ||
			port1p->portData->pPKey[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 == STL_DEFAULT_FM_PKEY)
			return smValidatePortPKey(STL_DEFAULT_FM_PKEY, port2p);
		else
			return smValidatePortPKey(STL_DEFAULT_CLIENT_PKEY, port2p);
	}
	for (i=0; i<PKEY_TABLE_LIST_COUNT; ++i) {
		if (PKEY_VALUE(port1p->portData->pPKey[i].AsReg16) == PKEY_VALUE(pkey)) {
			for (j=0; j<PKEY_TABLE_LIST_COUNT; ++j) {
				if (PKEY_VALUE(port2p->portData->pPKey[j].AsReg16) == PKEY_VALUE(pkey)) {
					if ((PKEY_TYPE(port1p->portData->pPKey[i].AsReg16) == PKEY_TYPE_FULL) ||
						(PKEY_TYPE(port2p->portData->pPKey[j].AsReg16) == PKEY_TYPE_FULL)) {
						return 1;
					}
					return 0;
				}
			}
			return 0;
		}
	}
	return 0;
}

uint16_t
smGetRequestPkeyIndex(uint8_t pkeyIndex,  STL_LID slid) {
	Port_t*		reqPortp;
	Port_t*		smPortp;
	uint16_t	pkey;
	
	if (pkeyIndex > PKEY_TABLE_LIST_COUNT-1) return INVALID_PKEY;

	reqPortp = sm_find_active_port_lid(&old_topology, slid);
	if (sm_valid_port(reqPortp) && (reqPortp->state > IB_PORT_DOWN)) {
		pkey = reqPortp->portData->pPKey[pkeyIndex].AsReg16;
		smPortp = sm_get_port(old_topology.node_head, sm_config.port);
		if (smValidatePortPKey2Way(pkey, reqPortp, smPortp)) {
			return pkey;
		}
	} 
	return INVALID_PKEY;
}

uint16_t
smGetCommonPKey(Port_t* port1, Port_t* port2) {

	uint16_t	pkey;
	uint8_t		pkeyEntry;
	int			defPkeyEntry=-1;

	for (pkeyEntry=0; pkeyEntry<PKEY_TABLE_LIST_COUNT; pkeyEntry++) {
		pkey = port1->portData->pPKey[pkeyEntry].AsReg16;

		if (PKEY_VALUE(pkey) == INVALID_PKEY) break;
	
		if (PKEY_VALUE(pkey) == PKEY_VALUE(STL_DEFAULT_FM_PKEY)) { 
			if (defPkeyEntry < 0 || PKEY_TYPE(pkey) == PKEY_TYPE_FULL)
				defPkeyEntry = pkeyEntry;
			continue;
		}

		if (smValidatePortPKey(pkey, port2)) return PKEY_VALUE(pkey);
	}

	// use default if defined
	if ((defPkeyEntry >= 0) &&
		smValidatePortPKey(port1->portData->pPKey[defPkeyEntry].AsReg16, port2)) {
		return PKEY_VALUE(port1->portData->pPKey[defPkeyEntry].AsReg16);
	}
	return INVALID_PKEY;
}

uint16_t getDefaultPKey(void)
{
	return defaultPKeys[STL_DEFAULT_FM_PKEY_IDX];
}


// is Service ID explictly in VF
// does not cover unmatchedServiceId option
bool_t
smCheckServiceId(int vf, uint64_t serviceId, VirtualFabrics_t *VirtualFabrics)
{
    cl_map_item_t   *cl_map_item;
	VFAppSid_t	*app;

    for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric_all[vf].apps.sidMap);
        cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric_all[vf].apps.sidMap);
        cl_map_item = cl_qmap_next(cl_map_item)) {
        app = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
		if ((!app->service_id_last &&
		 	(serviceId == app->service_id)) ||
			((app->service_id <= serviceId) &&
	     	(app->service_id_last >= serviceId)) ||
	    	((serviceId & app->service_id_mask) == 
		 	(app->service_id & app->service_id_mask))) {

			return TRUE;
		}
	}
	return FALSE;
}

// validate serviceId for VF.  Also covers the unmatchedSid option
Status_t
smVFValidateVfServiceId(int vf, uint64_t serviceId)
{
	int vf2;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (vf >= VirtualFabrics->number_of_vfs_all) return VSTATUS_BAD;

	// Check for service ID	
	if (smCheckServiceId(vf, serviceId, VirtualFabrics))
		return VSTATUS_OK;

	if (!VirtualFabrics->v_fabric_all[vf].apps.select_unmatched_sid)
		return VSTATUS_BAD;

	for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs_all && vf2 < MAX_VFABRICS; vf2++) {
		if (smCheckServiceId(vf2, serviceId, VirtualFabrics))
			// app found in another VF
			return VSTATUS_BAD;
	}
	return VSTATUS_OK;
}

// called in outer loop of path record query.  return a bitmask of VFs to be processed
// It should filter down those VFs based on pkey, SL, mtu and rate.
//
// Then main loop of PathRecord query would look like:
// for all srcport/dstport pairs to consider
//     call this function to filter down to a list of VFs (via a bitmask)
//     for each VF in bitmask
//         routing.select_path_lids(....,vf)
//             [lids returned could be controled by vf.SecondaryRouteOnly]

Status_t
smGetValidatedVFs(Port_t* srcport, Port_t* dstport, uint16_t pkey, uint8_t reqSL, uint8_t respSL,
					uint64_t serviceId, uint8_t checkServiceId, bitset_t* vfs) {
	int			vf;
	uint8_t		appFound=0;
	uint8_t		srvIdInVF=0;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	for (vf = 0; vf < VirtualFabrics->number_of_vfs_all && vf < MAX_VFABRICS; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;

		if (checkServiceId) {
			srvIdInVF = 0;
			// Check for service ID
			if (smCheckServiceId(vf, serviceId, VirtualFabrics)) {
				srvIdInVF = 1;
				if (!appFound) {
					// found actual match, clear any unmatched sid vfs
					bitset_clear_all(vfs);
					appFound=1;
				}
			}

			if (appFound && !srvIdInVF) continue;

			if (!appFound && !VirtualFabrics->v_fabric_all[vf].apps.select_unmatched_sid) continue;
		}

		// Is this the proper vf?
		if ((pkey != 0) && (PKEY_VALUE(pkey) != PKEY_VALUE(VirtualFabrics->v_fabric_all[vf].pkey))) continue;

		// Are these the proper sls?
		if ((reqSL < STL_MAX_SLS) && (reqSL != VirtualFabrics->v_fabric_all[vf].base_sl)) continue;
		if ((respSL < STL_MAX_SLS) && (respSL != VirtualFabrics->v_fabric_all[vf].resp_sl)) continue;

		if (sm_config.enforceVFPathRecs) {
			// One is full?
			if (srcport != dstport &&
				!bitset_test(&srcport->portData->fullPKeyMember, vf) &&
				!bitset_test(&dstport->portData->fullPKeyMember, vf)) continue;

			// Are both src and dst part of this VF?
			if (!bitset_test(&srcport->portData->vfMember, vf)) continue;
			if (!bitset_test(&dstport->portData->vfMember, vf)) continue;

			bitset_set(vfs, vf);
		}
		else {

			// If we are not checking that the src/dst ports are in the same VF,
			// then need to check that pkey shared between the source and destination ports
			int	vf2;
			Port_t*  tstport = 0;
			if (srcport == dstport) tstport = dstport;
			else if (bitset_test(&srcport->portData->fullPKeyMember, vf)) tstport = dstport;
			else if (bitset_test(&dstport->portData->fullPKeyMember, vf)) tstport = srcport;

			if (tstport == 0) continue; // Case where neither port is a full member of this VF

			uint16_t tstpkey = VirtualFabrics->v_fabric_all[vf].pkey;
			uint8_t tstSL = VirtualFabrics->v_fabric_all[vf].base_sl;
			uint8_t tstrspSL = VirtualFabrics->v_fabric_all[vf].resp_sl;

			for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs_all && vf2 < MAX_VFABRICS; vf2++) {
				if (VirtualFabrics->v_fabric_all[vf2].standby) continue;
				if (checkServiceId) {
					// Check for service ID
					if (smCheckServiceId(vf2, serviceId, VirtualFabrics)) {
						if (!appFound) {
							// Outer loop is using unmatched, exit this loop because this vf will be found later in outer loop.
							break;
						}
					} else if (appFound || !VirtualFabrics->v_fabric_all[vf2].apps.select_unmatched_sid) {
						// Two cases to skip:
						//    App is found and no service ID match on this vf, continue to next vf.
						//    Unmatched is in use and this vf is not using unmatched.
						continue;
					}
				}

				// To have a path between VFs, the VFs must share the same PKEY and Base SL
				// (AND Response SL if either VF requires Response SLs). Only need to check
				// base SL b/c rules for SL sharing requires consistent pairing among VFs
				if (PKEY_VALUE(tstpkey) != PKEY_VALUE(VirtualFabrics->v_fabric_all[vf2].pkey)) continue;
				if (tstSL != VirtualFabrics->v_fabric_all[vf2].base_sl) continue;
				if ((VirtualFabrics->v_fabric_all[vf].requires_resp_sl || VirtualFabrics->v_fabric_all[vf2].requires_resp_sl) &&
					tstrspSL != VirtualFabrics->v_fabric_all[vf2].resp_sl) continue;

				if (bitset_test(&tstport->portData->vfMember, vf2)) {
					bitset_set(vfs, vf2);
				}
			}
		}
	}

	return VSTATUS_OK;
}

int mgidMatch(uint64_t mgid[2], VFAppMgid_t * app) {

	if (app->mgid_last[0] != 0 || app->mgid_last[1] != 0) {
		if (mgid[0] < app->mgid[0]) return 0;

		if ((mgid[0] == app->mgid[0]) &&
			(mgid[1] < app->mgid[1])) return 0;


		if (mgid[0] > app->mgid_last[0]) return 0;

		if ((mgid[0] == app->mgid_last[0]) &&
			(mgid[1] > app->mgid_last[1])) return 0;

		return 1;

	} else if (((mgid[0] & app->mgid_mask[0]) == app->mgid[0]) &&
		   	   ((mgid[1] & app->mgid_mask[1]) == app->mgid[1])) {
		return  1;
	}

	return 0;
}

// is MGID explictly in VF
// does not cover unmatchedMGid option
static bool_t
smCheckMGid(VF_t* vfp, uint64_t mGid[2])
{
    cl_map_item_t   *cl_map_item;
	VFAppMgid_t 	*app;

	for_all_qmap_ptr(&vfp->apps.mgidMap, cl_map_item, app) {
		if (mgidMatch(mGid, app)) {
			return TRUE;
		}
	}
	return FALSE;
}

// validate mGid for VF.  Also covers the unmatchedMGid option
Status_t
smVFValidateVfMGid(VirtualFabrics_t *VirtualFabrics, int vf, uint64_t mGid[2])
{
	int vf2;

	if (vf >= MAX_VFABRICS) return VSTATUS_BAD;

	// Check for MGID	
	if (smCheckMGid(&VirtualFabrics->v_fabric_all[vf], mGid))
		return VSTATUS_OK;

	if (!VirtualFabrics->v_fabric_all[vf].apps.select_unmatched_mgid)
		return VSTATUS_BAD;

	for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs_all && vf2 < MAX_VFABRICS; vf2++) {
		if (VirtualFabrics->v_fabric_all[vf2].standby) continue;
		if (smCheckMGid(&VirtualFabrics->v_fabric_all[vf2], mGid))
			// mgid found in another VF
			return VSTATUS_BAD;
	}
	return VSTATUS_OK;
}

//
// This function is called from sa_McMemberRecord_Set. It essentially
// validates a McGroup Create request against the VF config in the SM by
// trying to find a VF that matches both the parameters in the McMember
// Record and to which the joiner port belongs. If a requestor port is
// specified, also verify that the requestor is a member of that VF.
//
// If a matching VF is found, search through the applications that might
// match the supplied MGid in the McMember record.
//
// Returns VSTATUS_OK if a matching VF is found.
//
// TODO: When multi-subnet support eventually gets implemented, we
// should think about how the VF concepts extend across subnets. This
// affects the flowLabel, tClass, hopLimit, and scope parameters in the
// request, and as such careful consideration needs to be paid to
// implementing matching rules for these parameters as well. This is the
// main reason that this function gets passed the entire McMember record,
// rather than a subset of the parameters.
//
Status_t
smVFValidateMcGrpCreateParams(Port_t * joiner, Port_t * requestor,
                              STL_MCMEMBER_RECORD * mcMemberRec, bitset_t * groupVf)
{
	VFAppMgid_t 	*app;
	Status_t		foundMatch = VSTATUS_BAD;
	bitset_t		unmatchedAll;
	int				vf;
	uint8_t			mgidFound=0;
	uint8_t			mgidInVF=0;

	uint64_t		mGid[2];
//  uint64_t		joinerGid[2];
//  uint64_t		requestorGid[2];
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	bitset_clear_all(groupVf);

	if (!VirtualFabrics || !VirtualFabrics->securityEnabled) {
		if (PKEY_VALUE(mcMemberRec->P_Key) == DEFAULT_PKEY) {
			return VSTATUS_OK;
		} else
			return VSTATUS_BAD;
	}

	// Get the MGid for the Create request
 	mGid[0] = mcMemberRec->RID.MGID.AsReg64s.H;
	mGid[1] = mcMemberRec->RID.MGID.AsReg64s.L;

	// If a requestor was supplied (as in a proxy group create request),
	// get it's port GID
	if (sm_valid_port(requestor)) {
//  	requestorGid[0] = requestor->portData->gidPrefix;
//  	requestorGid[1] = requestor->portData->guid;
	}

	// Get the GID for the port
	//
	// TODO: Loop over port's GUIDInfo Table entries instead of just the
	// port's primary Guid
	if (sm_valid_port(joiner)) {
//  	joinerGid[0] = joiner->portData->gidPrefix;
//  	joinerGid[1] = joiner->portData->guid;

	} else {
		return VSTATUS_BAD;
	}

	bitset_init(&sm_pool, &unmatchedAll, MAX_VFABRICS);

	// Loop over virtual fabrics
	for (vf = 0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		cl_map_item_t   *it;
		VF_t *vfp = &VirtualFabrics->v_fabric_all[vf];

		if (vfp->standby) continue;

		// Continue if virtual fabric parameters don't match those in the mcMemberRecord.
		mgidInVF = 0;

		for (it = cl_qmap_head(&vfp->apps.mgidMap);
			it != cl_qmap_end(&vfp->apps.mgidMap);
			it = cl_qmap_next(it)) {
			app = XML_QMAP_VOID_CAST cl_qmap_key(it);

			//printf("Request MGID: "FMT_GID"\n\tApp.mgid: "FMT_GID
			//        "\n\tApp.mgid_last: "FMT_GID"\n\tApp.mgid_mask: "FMT_GID"\n\n",
			//        mGid[0], mGid[1], app->mgid[0], app->mgid[1],
			//        app->mgid_last[0], app->mgid_last[1], app->mgid_mask[0], app->mgid_mask[1]);

			if (mgidMatch(mGid, app)) {
				mgidInVF = 1;
				mgidFound = 1;
				bitset_set(groupVf, vf);
				break;
			}

		}

		if (  // McGroup Pkey matches VF Pkey
		   (PKEY_VALUE(mcMemberRec->P_Key) != PKEY_VALUE(vfp->pkey))
		      // Check to see if the joiner/creator port is a member of this VF
		   || (!bitset_test(&joiner->portData->vfMember, vf))
		      // Check to see if the (optional) requestor port is a member of this VF
		   || (requestor != NULL && !bitset_test(&requestor->portData->vfMember, vf))) {
			continue;
		}

		if ((vfp->mcast_sl != UNDEFINED_XML8 && mcMemberRec->SL != vfp->mcast_sl) ||
			(vfp->mcast_sl == UNDEFINED_XML8 && mcMemberRec->SL != vfp->base_sl))
			continue;

		if (vfp->security &&
		   !bitset_test(&joiner->portData->fullPKeyMember, vf)) {
			// Joiner must be full member
			continue;
		}

		if (mcMemberRec->Mtu > vfp->max_mtu_int) continue;
		if (linkrate_gt(mcMemberRec->Rate, vfp->max_rate_int)) continue;

		if (mgidInVF) {
			foundMatch = VSTATUS_OK;
		}

		if (vfp->apps.select_unmatched_mgid) {
			bitset_set(&unmatchedAll, vf);
		}
	}

	if ((mgidFound == 0) && (foundMatch == VSTATUS_BAD) && unmatchedAll.nset_m) {
		foundMatch = VSTATUS_OK;
		bitset_copy(groupVf, &unmatchedAll);
	}

	bitset_free (&unmatchedAll);

	return foundMatch;
}

void
smVerifyMcastPkey(uint64_t* mGid, uint16_t pkey) {

	if (((mGid[0] & 0xffff0000) == 0) &&
		(((mGid[0] & 0xff00ffff00000000LL) == 0xff00401b00000000LL) ||
		 ((mGid[0] & 0xff00ffff00000000LL) == 0xff00601b00000000LL))) {
		mGid[0] |= ((uint64)0xffff0000 & ((uint64)MAKE_PKEY(1,pkey) << 16));
	}
}

// virtual fabric name given index
char* smGetVfName(uint16_t pKey) {
	char	*name = NULL;
	int		vf;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

    if (!VirtualFabrics ||
    	(VirtualFabrics->number_of_vfs_all == 0)) return NULL;

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;
		if (PKEY_VALUE(pKey) == PKEY_VALUE(VirtualFabrics->v_fabric_all[vf].pkey)) {
			if (!name) {
				name = VirtualFabrics->v_fabric_all[vf].name;
			} else  {
				// Which one is it?
				return NULL;
			}
		}
	}
    return name;
}   
 
void
smGetVfMaxMtu(Port_t *portp, Port_t *reqportp, STL_MCMEMBER_RECORD *mcmp, uint8_t *maxMtu, uint8_t *maxRate) {

	int			vf;
	uint64_t	mGid[2];
	VFAppMgid_t *app;
	uint8_t		mgidFound=0;
	uint8_t		unmatchedAllMtu=0;
	uint8_t		unmatchedAllRate=0;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	*maxMtu=0;
	*maxRate=0;

	if (!VirtualFabrics) {
		*maxMtu=0;
		*maxRate=0;
		return;
	}

	if (!sm_valid_port(portp)) return;
	if (!sm_valid_port(reqportp)) return;

 	mGid[0] = mcmp->RID.MGID.AsReg64s.H;
	mGid[1] = mcmp->RID.MGID.AsReg64s.L;

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
    	cl_map_item_t   *cl_map_item;
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;

		if (PKEY_VALUE(mcmp->P_Key) != PKEY_VALUE(VirtualFabrics->v_fabric_all[vf].pkey)) continue;
		if (!((mcmp->SL == VirtualFabrics->v_fabric_all[vf].base_sl) ||
			(mcmp->SL == VirtualFabrics->v_fabric_all[vf].resp_sl))) continue;
		if (!bitset_test(&portp->portData->vfMember, vf)) continue;
		if (!bitset_test(&reqportp->portData->vfMember, vf)) continue;

		if (VirtualFabrics->v_fabric_all[vf].security &&
		    !bitset_test(&portp->portData->fullPKeyMember, vf)) {
			// Create/joiner must be full member
			continue;
		}

    	for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric_all[vf].apps.mgidMap);
        	cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric_all[vf].apps.mgidMap);
        	cl_map_item = cl_qmap_next(cl_map_item)) {
        	app = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			if (mgidMatch(mGid, app)) {
				mgidFound = 1;
				*maxMtu =  MAX(*maxMtu, VirtualFabrics->v_fabric_all[vf].max_mtu_int);
				if (linkrate_lt(*maxRate, VirtualFabrics->v_fabric_all[vf].max_rate_int)) {
					*maxRate = VirtualFabrics->v_fabric_all[vf].max_rate_int;
				}
				break;
			}
		}

		if (mgidFound) continue;

		if (VirtualFabrics->v_fabric_all[vf].apps.select_unmatched_mgid) {
			unmatchedAllMtu = MAX(unmatchedAllMtu, VirtualFabrics->v_fabric_all[vf].max_mtu_int);
			if (linkrate_lt(unmatchedAllRate, VirtualFabrics->v_fabric_all[vf].max_rate_int)) {
				unmatchedAllRate = VirtualFabrics->v_fabric_all[vf].max_rate_int;
			}
		}
	}

	if (!mgidFound) {
		*maxMtu = unmatchedAllMtu;
		*maxRate = unmatchedAllRate;
	}
}

void
smProcessVFNodeMembers(Node_t *nodep, int vfIdx, char* memberName, uint8_t isFullMember) {

	int dgIdx = smGetDgIdx(memberName);

	if (dgIdx != -1) {

		//loop on all ports and set vfMember
		Port_t* portp=NULL;
		PortData_t* portDataPtr=NULL;
		boolean nodeHasMembers = FALSE;
		for_all_ports(nodep,portp) {
					
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
				continue;
			else {
				portDataPtr = portp->portData;

				if (isDgMember(dgIdx, portp->portData)) {
					//if dgMember, then set vfMember
					nodeHasMembers = TRUE;
					bitset_set(&portDataPtr->vfMember, vfIdx);

					if (isFullMember) {
						bitset_set(&portDataPtr->fullPKeyMember, vfIdx);
						bitset_set(&nodep->fullPKeyMember, vfIdx);
					}
				}
			}
		}

		if (nodeHasMembers)
			bitset_set(&nodep->vfMember, vfIdx);

	}
}

void
smSetupNodeVFs(Node_t *nodep) {

	int	vf;
	VFConfig_t* vfp;
	VirtualFabrics_t *VirtualFabrics = sm_topop->vfs_ptr;

	if (!VirtualFabrics) return;

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;
		// VfConfig index is the same as v_fabric_all
		vfp = vf_config.vf[vf];

		//loop on all DG members that are included and set vfMember based on already acquired knowledge

		//process all full members
		int isFullMember;
		int arrayIdx;
		for (arrayIdx=0; arrayIdx<vfp->number_of_full_members; arrayIdx++) {
			isFullMember = 1;
			smProcessVFNodeMembers(nodep, vf, vfp->full_member[arrayIdx].member, isFullMember);
		}

		//process all limited members
		for (arrayIdx=0; arrayIdx<vfp->number_of_limited_members; arrayIdx++) {

			if (!vfp->security) {
				// when security is off, all member are full
				isFullMember = 1;
			}
			else {
				isFullMember = 0;
			}

			smProcessVFNodeMembers(nodep, vf, vfp->limited_member[arrayIdx].member, isFullMember);
		}


	} //end loop on all VFs in VirtualFabrics*

}

boolean
smEvaluateNodeDG(Node_t* nodep, int dgIdxToEvaluate, PortRangeInfo_t* portInfo) {

	boolean isDgMember = FALSE;

	DGConfig_t* dgp = dg_config.dg[dgIdxToEvaluate];

	//check "Select" definition section
	if (dgp->select_all) {
		isDgMember = TRUE;
	}

	if (!isDgMember) {
		if ((dgp->select_self) && (nodep->index == 0)) {
			isDgMember = TRUE;
		}
	}

	//check "NodeType" definition section (ca, sw)
	if (!isDgMember) {
		if (((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) && (dgp->node_type_sw)) ||
			((nodep->nodeInfo.NodeType == NI_TYPE_CA) && (dgp->node_type_fi))) {
			isDgMember = TRUE;
		} 
	}

	//check "SystemImageGUID" definition section
	if (!isDgMember && (dgp->number_of_system_image_guids > 0)) {
		isDgMember = (cl_qmap_get(&dgp->system_image_guid, nodep->nodeInfo.SystemImageGUID) != 
			cl_qmap_end(&dgp->system_image_guid));
	}
	  
	//check "NodeGUID" definition section
	if (!isDgMember && (dgp->number_of_node_guids > 0)) {
		isDgMember = (cl_qmap_get(&dgp->node_guid, nodep->nodeInfo.NodeGUID) !=
			cl_qmap_end(&dgp->node_guid));
	}

	//check "NodeDesc" definition section
	if (!isDgMember) {

		boolean isFullMember = FALSE;

		if (dgp->number_of_node_descriptions > 0) {

			if (!isFullMember) {

				XmlNode_t* nodeDescPtr = dgp->node_description;
				RegExp_t* regExprPtr = dgp->reg_expr;

				while ( (nodeDescPtr != NULL)&&(regExprPtr != NULL) ) {

					if (regExprPtr->isSyntaxValid) {

						boolean isMatch = FALSE;
						RegexBracketParseInfo_t* regexInfoPtr = &regExprPtr->regexInfo;

#if defined(__VXWORKS__)
						int status = regexec(regExprPtr->regexpCompiled, sm_nodeDescString(nodep), TRUE);
						if (status)
							isMatch = TRUE;
						else
							isMatch = FALSE;
#else
						if (regexec(&regExprPtr->regexCompiled, sm_nodeDescString(nodep), MAX_BRACKETS_SUPPORTED, regExprPtr->groupArray, 0) == 0) {
							isMatch = TRUE;

							//evaluate all the bracket ranges defined
							int bracketIdx;
							for (bracketIdx=0; bracketIdx<regexInfoPtr->numBracketRangesDefined;bracketIdx++) {

								int bracketGroupNum = regexInfoPtr->bracketGroupNum[bracketIdx];

								//verify the string index of the group we are evaluting isn't -1
								if (regExprPtr->groupArray[bracketGroupNum].rm_so != (size_t)-1) {

									//Make a copy of the node name, which we will manipulate to extract the number out of using the regmatch_t struct fields
									char nodeName[strlen(sm_nodeDescString(nodep)) + 1];
									cs_strlcpy(nodeName, sm_nodeDescString(nodep), sizeof(nodeName));
									nodeName[regExprPtr->groupArray[bracketGroupNum].rm_eo] = 0;

									//Get the number to evalute by incrementing the node name to the position of the number
									char* numberToEval = nodeName + regExprPtr->groupArray[bracketGroupNum].rm_so;

									isMatch = isNumberInRange(numberToEval, regexInfoPtr, bracketIdx);

									if (!isMatch)
										break;
								}
								else {
									//Invalid group number for bracket evaluation
									isMatch = FALSE;
								}

							} //loop on all bracket ranges defined
						} //end if regexec matches check
#endif //if VxWorks check

							if (isMatch) {

								//set isDgMember to TRUE for all ports on this node
								isDgMember = TRUE;

								if (regexInfoPtr->portRangeDefined) {

									//set port number ranges
									int index = portInfo->numPortRanges;
									portInfo->port1[index] = regexInfoPtr->portNum1;
									portInfo->port2[index] = regexInfoPtr->portNum2;
									portInfo->numPortRanges++;
								}
								else {
									//full node match..erase previous port ranges and exit loop on all node descriptions
									regexInfoPtr->portRangeDefined = 0;
									portInfo->numPortRanges = 0;
									isFullMember = TRUE;
								}
							}

					} //end if isSyntaxValid check

					nodeDescPtr = nodeDescPtr->next;
					regExprPtr = regExprPtr->next;
				} //end while
			} // if check if full member
		} //end if num node descriptions < 0


	} //end "NodeDesc" check

	return isDgMember;
}


boolean
smEvaluatePortDG(Node_t* nodep, Port_t* portp, int dgIdxToEvaluate, bitset_t* dgMemberForPort, bitset_t* dgsEvaluated) {

	boolean isDgMember = FALSE;

	//Has dgIdxToEvaluate already been processed?
	if (!bitset_test(dgsEvaluated, dgIdxToEvaluate)) {

		DGConfig_t* dgp = dg_config.dg[dgIdxToEvaluate];

		//check select AllSWE0
		if (portp->index == 0) {
			if (dgp->select_swe0) {
				if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
					nodep->switchInfo.u2.s.EnhancedPort0) {
					isDgMember = TRUE;
				}
			}
		}

		//check select_hfi_direct_connect
		if (!isDgMember) {
			if (dgp->select_hfi_direct_connect) {
				if (sm_hfi_direct_connect != 0 && sm_topop->num_sws == 0) { // fabric sim check
					isDgMember = TRUE;
				}
			}
		}

		//check select_all_mgmt_allowed
		if (!isDgMember) {
			if (dgp->select_all_mgmt_allowed) {
				if (portp->portData->portInfo.PortNeighborMode.MgmtAllowed) {
					isDgMember = TRUE;
				} else if (sm_stl_appliance(portp->portData->portInfo.NeighborNodeGUID)) {
					// Allow to be considered mgmt allowed if neighbor is in SM appliance
					isDgMember = TRUE;
				}
			}
		}

		//check select AllTFIs 
		if (!isDgMember) {
			if (dgp->select_all_tfis) {
				if (nodep->nodeInfo.NodeType == NI_TYPE_CA &&
					((portp->portData->capmask & PI_CM_IS_DEVICE_MGT_SUPPORTED) != 0) ) {
					isDgMember = TRUE;
				}
			}
		}


		//check "PortGUID" definition section
		if (!isDgMember && (dgp->number_of_port_guids > 0)) {
			isDgMember = (cl_qmap_get(&dgp->port_guid, portp->portData->guid) != 
				cl_qmap_end(&dgp->port_guid));
		}

		if (isDgMember)
			bitset_set(dgMemberForPort, dgIdxToEvaluate);


		//evaluate all include groups
		if (!isDgMember) {
			//Verify groupName not in includes
			if (dgp->number_of_included_groups > 0) {

				XmlIncGroup_t *group = dgp->included_group;

				while (group != NULL) {

					//find dgIdxToEvaluate for group->name
					int includedDgIdx = smGetDgIdx(group->group);

					//skip evaluating this group if we can't find a valid dgIdxToEvaluate
					if ( (includedDgIdx != -1) && (includedDgIdx < dg_config.number_of_dgs) ) {

						//check if we have already evaluated this includedDgIdx

						//if we have not yet evaluated, need to evaluate on its own first (includedDgIdx is also passed in as the parentDgIdx)
						if (!bitset_test(dgsEvaluated, includedDgIdx)) {
							isDgMember = smEvaluatePortDG(nodep, portp, includedDgIdx, dgMemberForPort, dgsEvaluated);

							if (isDgMember) {
								bitset_set(dgMemberForPort, dgIdxToEvaluate);
								bitset_set(dgMemberForPort, includedDgIdx);
							}

						}
						else {
							//include group already evaluated, use previous knowledge
							if (bitset_test(dgMemberForPort, includedDgIdx))
								isDgMember = TRUE;
						}

					}

					//if we find the port is a member of DG, get out of loop
					if (isDgMember)
						break;

					group = group->next;
				}
			}
		}

	}
	else { 

		//use knowledge we have already acquired to set bits for this device group
		if (bitset_test(&portp->portData->dgMember, dgIdxToEvaluate))
			isDgMember = TRUE;
	}

	//Set bit that this dgIdx has been evaluated for this port
	bitset_set(dgsEvaluated, dgIdxToEvaluate);

	return isDgMember;
}

void
smSetupNodeDGs(Node_t *nodep) {

	int dgIdx;
	int numGroups = dg_config.number_of_dgs;
	Node_t* oldnp = NULL;
	int reuseDGs = 0;

	//Evaluate node against node specific criteria for each defined device group
	boolean isNodeMember = FALSE;
	bitset_clear_all(&nodep->dgMembership);

	//Save port ranges for dgMembers
	int numPorts = 2;
	int dgPortRanges[numGroups][MAX_NODE_DESC_ENTRIES][numPorts];

	PortRangeInfo_t portInfo;
	int numValidPortMatches[numGroups];

	// If we've already calculated the dgMembership bitset for this node, we
	// might be able to skip re-evaluating its dgMembership again.
	if (topology_passcount > 0) {
		Port_t* portp=NULL;

		// If the description hasn't changed, we might be able to reuse
		// the exsting bitset.
		oldnp = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
		if (oldnp &&
			(strncmp((const char *)oldnp->nodeDesc.NodeString,
			(const char *)nodep->nodeDesc.NodeString,
			sizeof(nodep->nodeDesc.NodeString))==0)) {

			// This node is a candidate for reusing dgMembership.
			reuseDGs = 1;

			// But... we still have to check every port of the node to see if its
			// capabilitymask or the port state has changed. In either case,
			// re-evaluate.
			for_all_ports(nodep,portp) {
				Port_t* oldpp = NULL;

				if (!sm_valid_port(portp)) { continue; }

				// if the port is new for this sweep, we should re-evaluate.
				oldpp = sm_get_port(oldnp,portp->index);
				if (!sm_valid_port(oldpp)) { reuseDGs = 0; break; }

				// if the port changed states this sweep, we should re-evaluate.
				if (oldpp->state != portp->state) { reuseDGs = 0; break; }

				// if the port changed capabilities, we should re-evaluate.
				if (oldpp->portData->capmask != portp->portData->capmask){ reuseDGs = 0; break; }
				if (oldpp->portData->capmask3 != portp->portData->capmask3){ reuseDGs = 0; break; }
			}

			if (reuseDGs) {
				bitset_copy(&nodep->dgMembership, &oldnp->dgMembership);
			}
		}
	}

	// For each node, evaluate the membership of that node.
	memset(numValidPortMatches, 0, sizeof(int)*numGroups);
	if (!reuseDGs) for (dgIdx = 0; dgIdx < numGroups; dgIdx++) {

		//clear num port ranges defined
		portInfo.numPortRanges = 0;

		// This may alter portInfo.numPortRanges.
		isNodeMember = smEvaluateNodeDG(nodep, dgIdx, &portInfo);

		if (isNodeMember) {
			bitset_set(&nodep->dgMembership, dgIdx);

			//reset number port matches
			numValidPortMatches[dgIdx] = 0;

			//Save all port ranges
			int portIdx;
			for (portIdx = 0; portIdx < portInfo.numPortRanges; portIdx++) {
				int validPortIdx = numValidPortMatches[dgIdx];
				dgPortRanges[dgIdx][validPortIdx][0] = portInfo.port1[portIdx];
				dgPortRanges[dgIdx][validPortIdx][1] = portInfo.port2[portIdx];
				numValidPortMatches[dgIdx]++;
			}
		}

	}

	//Evaluate port specific criteria (including evaluating include groups of each device group)
	Port_t* portp=NULL;
	Port_t* oldpp=NULL;
	for_all_ports(nodep,portp) {

		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
			// Skip over inactive ports.
			continue;
		}

		if (reuseDGs) {
			// We can copy this from the old topology.
			oldpp = sm_get_port(oldnp, portp->index);
			if (sm_valid_port(oldpp)) {
				bitset_copy(&portp->portData->dgMember, &oldpp->portData->dgMember);
				memcpy(portp->portData->dgMemberList, oldpp->portData->dgMemberList,
					sizeof(portp->portData->dgMemberList[0])*MAX_DEVGROUPS);
				// Skip further evaluation.
				continue;
			}
		}

		// We couldn't copy from the old topo, so do the work from scratch.

		//create bitset to track which DGs we've evaluated for this node
		bitset_t dgsEvaluated;
		bitset_init(&sm_pool, &dgsEvaluated, numGroups);

		bool_t isMember = FALSE;

		int numMemberships = 0;

		PortData_t *portDataPtr = portp->portData;

		// Fill in default value for the dgMember array
		// Nota Bene: This works because DEFAULT_DEVGROUP_ID is 0xffff.
		memset(portDataPtr->dgMemberList,0xff, sizeof(portDataPtr->dgMemberList[0])*MAX_DEVGROUPS);

		// loop on all device groups to determine which groups this port is a member of.
		for (dgIdx = 0; dgIdx < numGroups; dgIdx++) {

			if (bitset_test(&nodep->dgMembership, dgIdx)) {

				if (numValidPortMatches[dgIdx] > 0) {
					//loop on each defined port range
					int portIdx;
					for (portIdx = 0; portIdx < numValidPortMatches[dgIdx]; portIdx++) {
						//verify port index is within the range defined
						if ( (portp->index >= dgPortRanges[dgIdx][portIdx][0]) && (portp->index <= dgPortRanges[dgIdx][portIdx][1]) ) {
							bitset_set(&portp->portData->dgMember, dgIdx);
							bitset_set(&dgsEvaluated, dgIdx);
						}
					}
				} else {
					bitset_set(&portp->portData->dgMember, dgIdx);
					bitset_set(&dgsEvaluated, dgIdx);
				}
			}

			isMember = smEvaluatePortDG(nodep, portp, dgIdx, &portp->portData->dgMember, &dgsEvaluated);

			if (isMember) {
				bitset_set(&portp->portData->dgMember, dgIdx);
				// Update dgMember array
				if (numMemberships < MAX_DEVGROUPS) {
					portDataPtr->dgMemberList[numMemberships] = dgIdx;
					numMemberships++;
				} else {
					IB_LOG_WARN_FMT(__func__, "Node %s not added to device group %s - Max members exceeeded",
						sm_nodeDescString(nodep), dg_config.dg[dgIdx]->name);
				}
			}
		}
		bitset_free(&dgsEvaluated);
	}
}

void smLogVFs() {
	int					vf;
	VFAppSid_t*			sidp;
	VFAppMgid_t*		mgidp;
	VFDg_t*				mcastGrpp;
	Node_t*				nodep;
	Port_t*				portp;
	int 				i;
	char*				vfp;
	cl_qmap_t			*cl_map;
	cl_map_item_t 		*cl_map_item;
	VirtualFabrics_t *VirtualFabrics = sm_topop->vfs_ptr;

	if (!VirtualFabrics) {
		IB_LOG_INFINI_INFO0("No Virtual Fabric Enabled");
		return;
	}

	IB_LOG_INFINI_INFO_FMT(__func__, "Number of VFs %d (securityEnabled= %d. qosEnabled= %d)",
			VirtualFabrics->number_of_vfs_all, VirtualFabrics->securityEnabled, VirtualFabrics->qosEnabled);

	for (vf=0; vf < VirtualFabrics->number_of_vfs_all; vf++) {
		vfp = VirtualFabrics->v_fabric_all[vf].name;
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "Index= %d, pkey= 0x%x, security= %d standby= %d",
				VirtualFabrics->v_fabric_all[vf].index, VirtualFabrics->v_fabric_all[vf].pkey,
				VirtualFabrics->v_fabric_all[vf].security, VirtualFabrics->v_fabric_all[vf].standby);

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF max_mtu_int= %d  max_rate_int= %d  pkt_lifetime_inc= %d ",
				VirtualFabrics->v_fabric_all[vf].max_mtu_int, VirtualFabrics->v_fabric_all[vf].max_rate_int,
				VirtualFabrics->v_fabric_all[vf].pkt_lifetime_mult);
		
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "bandwidthPercent= %d  priority= %d",
				VirtualFabrics->v_fabric_all[vf].percent_bandwidth, VirtualFabrics->v_fabric_all[vf].priority);
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "preemptionRank= %d  hoqLife= %d",
				VirtualFabrics->v_fabric_all[vf].preempt_rank, VirtualFabrics->v_fabric_all[vf].hoqlife_vf);

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF APPs:");

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tselect_sa= %d, select_unmatched_sid= %d, select_unmatched_mgid= %d, select_pm= %d",
				VirtualFabrics->v_fabric_all[vf].apps.select_sa,
				VirtualFabrics->v_fabric_all[vf].apps.select_unmatched_sid,
				VirtualFabrics->v_fabric_all[vf].apps.select_unmatched_mgid,
				VirtualFabrics->v_fabric_all[vf].apps.select_pm);

    	for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric_all[vf].apps.sidMap);
        	cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric_all[vf].apps.sidMap);
        	cl_map_item = cl_qmap_next(cl_map_item)) {
        	sidp = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tservice_id "FMT_U64", service_id_last "FMT_U64" service_id_mask  "FMT_U64"",
						sidp->service_id, sidp->service_id_last, sidp->service_id_mask);
		}

    	for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric_all[vf].apps.mgidMap);
        	cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric_all[vf].apps.mgidMap);
        	cl_map_item = cl_qmap_next(cl_map_item)) {
        	mgidp = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tmgid " FMT_GID "", mgidp->mgid[0], mgidp->mgid[1]);
		}

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF Full Members:");
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tselect_all=%d, select_self= %d, select_swe0=%d, select_hfi_direct_connect=%d",
				VirtualFabrics->v_fabric_all[vf].full_members.select_all, VirtualFabrics->v_fabric_all[vf].full_members.select_self,
				VirtualFabrics->v_fabric_all[vf].full_members.select_swe0,
				VirtualFabrics->v_fabric_all[vf].full_members.select_hfi_direct_connect);
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnode_type_fi=%d, node_type_sw= %d,",
				VirtualFabrics->v_fabric_all[vf].full_members.node_type_fi, VirtualFabrics->v_fabric_all[vf].full_members.node_type_sw);

		cl_map = &VirtualFabrics->v_fabric_all[vf].full_members.sysGuidMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tsysGuid=  "FMT_U64"", cl_map_item->key);
		}
		cl_map = &VirtualFabrics->v_fabric_all[vf].full_members.nodeGuidMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeGuid=  "FMT_U64"", cl_map_item->key);
		}
		cl_map = &VirtualFabrics->v_fabric_all[vf].full_members.portGuidMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tportGuid=  "FMT_U64"", cl_map_item->key);
		}
		cl_map = &VirtualFabrics->v_fabric_all[vf].full_members.nodeDescMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", XML_QMAP_CHAR_CAST cl_map_item->key);
		}

/*
		for (nodeDescrp=VirtualFabrics->v_fabric_all[vf].full_members.node_descr; nodeDescrp; nodeDescrp=nodeDescrp->next_node_description) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", nodeDescrp->node_description);
		}
*/

		if (VirtualFabrics->v_fabric_all[vf].security) {	
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF Limited Members:");
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tselect_all= %d, select_self= %d, select_swe0= %d, select_hfi_direct_connect=%d",
				VirtualFabrics->v_fabric_all[vf].limited_members.select_all, VirtualFabrics->v_fabric_all[vf].limited_members.select_self,
				VirtualFabrics->v_fabric_all[vf].limited_members.select_swe0, 
				VirtualFabrics->v_fabric_all[vf].limited_members.select_hfi_direct_connect);

			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnode_type_fi= %d, node_type_sw= %d",
				VirtualFabrics->v_fabric_all[vf].limited_members.node_type_fi, VirtualFabrics->v_fabric_all[vf].limited_members.node_type_sw);
			cl_map = &VirtualFabrics->v_fabric_all[vf].limited_members.sysGuidMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tsysGuid=  "FMT_U64"", cl_map_item->key);
			}
			cl_map = &VirtualFabrics->v_fabric_all[vf].limited_members.nodeGuidMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeGuid=  "FMT_U64"", cl_map_item->key);
			}
			cl_map = &VirtualFabrics->v_fabric_all[vf].limited_members.portGuidMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tportGuid=  "FMT_U64"", cl_map_item->key);
			}
			cl_map = &VirtualFabrics->v_fabric_all[vf].limited_members.nodeDescMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", XML_QMAP_CHAR_CAST cl_map_item->key);
			}
/*
			for (nodeDescrp=VirtualFabrics->v_fabric_all[vf].limited_members.node_descr; nodeDescrp; nodeDescrp=nodeDescrp->next_node_description) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", nodeDescrp->node_description);
			}
*/
		}
		for (mcastGrpp = VirtualFabrics->v_fabric_all[vf].default_group; mcastGrpp; 
			 mcastGrpp = mcastGrpp->next_default_group) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "MC Default Group:");
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tmc_create=%d, mc_pkey= 0x%x, mc_mtu_int=%d, mc_rate_int=%d",
					mcastGrpp->def_mc_create, mcastGrpp->def_mc_pkey, mcastGrpp->def_mc_mtu_int, mcastGrpp->def_mc_rate_int);
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tmc_sl=0x%x, mc_qkey= 0x%x, mc_fl=%d, mc_tc=%d",
					mcastGrpp->def_mc_sl, mcastGrpp->def_mc_qkey, mcastGrpp->def_mc_fl, mcastGrpp->def_mc_tc);
	
			cl_map_item_t* cl_map_item;
			for_all_qmap_ptr(&mcastGrpp->mgidMap, cl_map_item, mgidp) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tmcmgid " FMT_GID "", mgidp->mgid[0], mgidp->mgid[1]);
			}
		}
	}

	IB_LOG_INFINI_INFO_FMT(__func__, "SM MasterSL is %d", sm_masterSmSl);

	if (sm_newTopology.num_nodes < 50) {
		for_all_nodes(&sm_newTopology, nodep) {
			if (bitset_nset(&nodep->vfMember)) {
				IB_LOG_INFINI_INFO_FMT(__func__, "Node %s", sm_nodeDescString(nodep));
				bitset_info_log(&nodep->vfMember, "member of VFs");
				bitset_info_log(&nodep->fullPKeyMember, "member is FULL");
			}
	
			for_all_ports(nodep,portp) {
				if (sm_valid_port(portp) &&
					(portp->state > IB_PORT_DOWN) &&
					(bitset_nset(&portp->portData->vfMember))) {
					IB_LOG_INFINI_INFO_FMT(__func__, "Node %s, port= %d", sm_nodeDescString(nodep), portp->index);
				bitset_info_log(&portp->portData->vfMember, "member of VFs");
					bitset_info_log(&portp->portData->fullPKeyMember, "member is FULL");
				}
			}
		}
	}

	for (i = 0; i < PKEY_TABLE_LIST_COUNT; ++i) {
		if (PKEY_VALUE(defaultPKeys[i]) == 0) {
			break;
		}
		IB_LOG_INFINI_INFO_FMT(__func__, "defaultPkeyEntry %d, pkey= 0x%x", i, defaultPKeys[i]);
	}
}
