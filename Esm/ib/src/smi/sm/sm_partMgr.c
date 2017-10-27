/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

static uint8_t isEntryZeroUseded=0; // FIXME: cjking - make a parameter to fucntion instead of global

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

static int sm_get_stl_pkey_index(int pkeyIndex)
{
	// skip STL management PKEY indices
	if (!isEntryZeroUseded) {
		pkeyIndex = 0;
		isEntryZeroUseded++;
	} else if (pkeyIndex == STL_DEFAULT_CLIENT_PKEY_IDX || pkeyIndex == STL_DEFAULT_FM_PKEY_IDX) {
		pkeyIndex = STL_DEFAULT_FM_PKEY_IDX + 1;
	}
	return (pkeyIndex);
}

Status_t
sm_set_portPkey(Topology_t *topop, Node_t *nodep, Port_t *portp,
				Node_t *linkedNodep, Port_t *linkedPortp,
				uint8_t *path, uint8_t *enforcePkey,
				uint16_t	*vlHighLimit, int use_lr_dr_mix)
{

	uint32_t    amod;
	Status_t	status=VSTATUS_OK;
    STL_PARTITION_TABLE pkeyTable, *pkp;
	uint8_t     pkeys, isl=0;
	int			pkeyCap=0;
	int			pkeyEntry=0, pkeyEntryCntr=0;
	static int	pkeyExceededAlarm=0;
	uint8_t		defaultDefined=0, numBlocks=1;
	int			vf;
	uint16_t	pkey=0, block=0;

	/* Set up the pkey port table */
	pkp = (STL_PARTITION_TABLE *)topop->pad;
	memset(pkp, 0, sizeof(*pkp));
	memset(portp->portData->pPKey, 0, sizeof(PKey_t) * SM_PKEYS);
	memset(&pkeyTable, 0, sizeof(pkeyTable));
	pkeyTable.PartitionTableBlock[STL_DEFAULT_APP_PKEY_IDX].AsReg16 = STL_DEFAULT_APP_PKEY;
	pkeyTable.PartitionTableBlock[STL_DEFAULT_CLIENT_PKEY_IDX].AsReg16 = STL_DEFAULT_CLIENT_PKEY;
    isEntryZeroUseded=0;

	*vlHighLimit = 0;

	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;
	if (!VirtualFabrics) {
		/*
		 * FIXME: This code never seems to get called, even if no virtual
		 * fabrics are defined.
		 */
		// Use default for all with limited membership
		portp->portData->pPKey[0] = defaultPKeyTable.PartitionTableBlock[0];
		pkeyTable = defaultPKeyTable;
		pkeyEntry = 1;
        pkeyEntryCntr = 1;
		defaultDefined = 1;

		IB_LOG_ERROR_FMT(__func__, "No Virtual Fabrics defined. "
			"Using Default Partition keys.");

	} else if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) && (portp->index > 0)) {
		// check the port at the other end.  If it is an FI port use that ports config
 		// to program the switch (unless this is sm).
		if (linkedNodep->nodeInfo.NodeType == NI_TYPE_CA) {

			for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
				pkey = 0;
				uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;
				if (bitset_test(&linkedPortp->portData->vfMember, vfIdx) ||
					bitset_test(&linkedPortp->portData->fullPKeyMember, vfIdx)) {
					pkey = VirtualFabrics->v_fabric[vf].pkey;
					if (bitset_test(&linkedPortp->portData->fullPKeyMember, vfIdx)) {
						pkey |= FULL_MEMBER;
					}
				}

				if (pkey) {
					if (VirtualFabrics->v_fabric[vf].security) {
						*enforcePkey = 1;
					}

					if (VirtualFabrics->v_fabric[vf].qos_enable &&
						VirtualFabrics->v_fabric[vf].priority) {
						*vlHighLimit = *vlHighLimit + 1;
					}

					//
					// handle STL management node related PKEYs
					if (PKEY_VALUE(pkey) == DEFAULT_PKEY) {
						if (!defaultDefined) {
							defaultDefined = 1;
							// if STL management PKEYs have not been defined, then
							// adjust entry; otherwise they have been accounted for 
							if (pkeyEntry <= STL_DEFAULT_CLIENT_PKEY_IDX) {
								pkeyEntry += 2;
                            }
                            // adjust pkey counter to reflect management PKEYs 
                            pkeyEntryCntr += 2;
						}

						(void)sm_set_port_stl_mngnt_pkey(nodep, portp, pkey, pkeyTable.PartitionTableBlock, portp->portData->pPKey);
						continue;
					}

					//
					// handle all other types of PKEYs
					for (pkeys=0; pkeys<pkeyEntry; pkeys++) {
						// skip STL management PKEYs
						if (pkeys == STL_DEFAULT_CLIENT_PKEY_IDX)
							pkeys = STL_DEFAULT_FM_PKEY_IDX + 1;

						if (PKEY_VALUE(portp->portData->pPKey[pkeys].AsReg16) == PKEY_VALUE(pkey)) {
							// VFs may share pkeys.  Change to full if needed.
							if ((PKEY_TYPE(portp->portData->pPKey[pkeys].AsReg16) != PKEY_TYPE_FULL) &&
								(PKEY_TYPE(pkey) == PKEY_TYPE_FULL)) {
								pkeyTable.PartitionTableBlock[pkeys].AsReg16 = pkey;
								portp->portData->pPKey[pkeys].AsReg16 = pkey;
							}
							pkey = 0;
							break;
						}
					}

					if (pkey) {
						pkeyEntry = sm_get_stl_pkey_index(pkeyEntry);
						if (pkeyEntry < SM_PKEYS) {
							pkeyTable.PartitionTableBlock[pkeyEntry].AsReg16=pkey;
							portp->portData->pPKey[pkeyEntry].AsReg16=pkey;
							pkeyEntry++;
                            pkeyEntryCntr++;
						} else {
							IB_LOG_ERROR_FMT(__func__,
								   "Node %s ["FMT_U64":%d] Pkey 0x%04x"
								   "PKEYEntries:%d (too many unique PKEYs defined in VFs", 
								   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkey, pkeyEntry);
							return (VSTATUS_BAD);
						}
					}

				}
			}
		} else if (linkedNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
            //
            // handle ISLs
            // for STL Gen1 pkey checking will not enforced; therefore, there is
            // no need to initialize the PKey Table of the physical port, just
            // let the PKey Table default to the power-on values of the switch.
            // be donein order to the programming of 
            isl = 1;
        }
		
	} else {
		for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
			pkey = 0;
			uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;
			if (bitset_test(&portp->portData->vfMember, vfIdx) ||
				bitset_test(&portp->portData->fullPKeyMember, vfIdx))  {
				pkey = VirtualFabrics->v_fabric[vf].pkey;
				if (bitset_test(&portp->portData->fullPKeyMember, vfIdx)) {
					pkey |= FULL_MEMBER;
				}
			}

			if (!pkey) continue;

			if (VirtualFabrics->v_fabric[vf].qos_enable &&
				VirtualFabrics->v_fabric[vf].priority) {
				*vlHighLimit = *vlHighLimit + 1;
			}

			//
			// handle STL management node related PKEYs
			if (PKEY_VALUE(pkey) == DEFAULT_PKEY) {
				if (!defaultDefined) {
					defaultDefined = 1;
					// if STL management PKEYs have not been defined, then
					// adjust entry; otherwise they have been accounted for 
					if (pkeyEntry <= STL_DEFAULT_CLIENT_PKEY_IDX) {
						pkeyEntry += 2;
                    }
                    // adjust pkey counter to reflect management PKEYs 
                    pkeyEntryCntr += 2;
				}

				(void)sm_set_port_stl_mngnt_pkey(nodep, portp, pkey, pkeyTable.PartitionTableBlock, portp->portData->pPKey);
				continue;
			}

			//
			// handle all other types of PKEYs
			for (pkeys=0; pkeys<pkeyEntry; pkeys++) {
				// skip STL management PKEYs
				if (pkeys == STL_DEFAULT_CLIENT_PKEY_IDX)
					pkeys = STL_DEFAULT_FM_PKEY_IDX + 1;

				if (PKEY_VALUE(portp->portData->pPKey[pkeys].AsReg16) == PKEY_VALUE(pkey)) {
					if ((PKEY_TYPE(portp->portData->pPKey[pkeys].AsReg16) != PKEY_TYPE_FULL) &&
						(PKEY_TYPE(pkey) == PKEY_TYPE_FULL)) {
						pkeyTable.PartitionTableBlock[pkeys].AsReg16 = pkey;
						portp->portData->pPKey[pkeys].AsReg16 = pkey;
					}
					pkey = 0;
					break;
				}
			}

			if (pkey) {
				pkeyEntry = sm_get_stl_pkey_index(pkeyEntry);
				if (pkeyEntry < SM_PKEYS) {
					pkeyTable.PartitionTableBlock[pkeyEntry].AsReg16=pkey;
					portp->portData->pPKey[pkeyEntry].AsReg16=pkey;
					pkeyEntry++;
                    pkeyEntryCntr++;
				} else {
					IB_LOG_ERROR_FMT(__func__,
						   "Node %s ["FMT_U64":%d] Pkey 0x%04x"
						   "PKEYEntries:%d (too many unique PKEYs defined in VFs", 
						   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkey, pkeyEntry);
					return (VSTATUS_BAD);
				}
			}
		}
	}
	
	if (!defaultDefined) {
		// no default defined
        // 
        // if no entries have been defined, then also initialize entry zero
        // used by applications
        if (!pkeyEntry) {
            pkeyEntry++;
            pkeyEntryCntr++;
            portp->portData->pPKey[STL_DEFAULT_APP_PKEY_IDX].AsReg16 = STL_DEFAULT_APP_PKEY;
        }

		// if STL management PKEYs have not been defined, then
		// adjust entry; otherwise they have been accounted for 
		if (pkeyEntry <= STL_DEFAULT_CLIENT_PKEY_IDX) {
			pkeyEntry += 2;
            pkeyEntryCntr += 2;
        }
		(void)sm_set_port_stl_mngnt_pkey(nodep, portp, STL_DEFAULT_FM_PKEY,
										 pkeyTable.PartitionTableBlock,
										 portp->portData->pPKey);
	} else if (pkeyEntry == 2) {
        // only the STL management PKEYs have been defined, no VF has been
        // defined for the default application entry, so add the default
        // application PKEY.
		pkeyEntry = sm_get_stl_pkey_index(pkeyEntry);
		if (pkeyEntry < SM_PKEYS) {
			pkeyTable.PartitionTableBlock[pkeyEntry].AsReg16=pkey;
			portp->portData->pPKey[pkeyEntry].AsReg16=pkey;
			pkeyEntry++;
			pkeyEntryCntr++;
		} 
    }

    // set total number of entries for the PKey Table within the SA repository.
	portp->portData->num_pkeys = pkeyEntryCntr;

	/* 
     * IB spec 10.9.2 C10-120 requirement - switch external ports won't 
     * have pkey tables if switchInfo->enforcementcap is zero, so don't ask
     */
	if (is_hfiport(portp) ||
		(is_swport0(portp) && nodep->nodeInfo.PartitionCap) ||
		(is_extswport(portp) && nodep->switchInfo.PartitionEnforcementCap))
	{
		amod = is_swport(portp) ? portp->index << 16 : 0;
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
           				"Node %s ["FMT_U64":%d] partition cap = %d, pkeys configured for use = %d, truncating pkey table", 
           				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkeyCap, pkeyEntryCntr);
			}
			defaultDefined = 0;
			for (pkeys=0; pkeys<pkeyCap; pkeys++) {
				if (PKEY_VALUE(pkeyTable.PartitionTableBlock[pkeys].AsReg16) == DEFAULT_PKEY) {
					defaultDefined = 1;
				}
			}
			pkey = DEFAULT_PKEY;
			for (pkeys=pkeyCap; pkeys<pkeyEntryCntr; pkeys++) {
				if (PKEY_VALUE(pkeyTable.PartitionTableBlock[pkeys].AsReg16) == DEFAULT_PKEY) {
					pkey = pkeyTable.PartitionTableBlock[pkeys].AsReg16;
				}
				pkeyTable.PartitionTableBlock[pkeys].AsReg16 = 0;
				portp->portData->pPKey[pkeys].AsReg16 = 0;
			}
			if (!defaultDefined) {
				pkeyTable.PartitionTableBlock[pkeyCap-1].AsReg16 = pkey;
				portp->portData->pPKey[pkeyCap-1].AsReg16 = pkey;
			}
			portp->portData->num_pkeys = pkeyCap;
		}
		IB_LOG_DEBUG1_FMT(__func__,
			"Node %s ["FMT_U64":%d] partition cap = %d, pkey = 0x%x",
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portp->index, pkeyCap, portp->portData->pPKey[0].AsReg16);

		for (pkeys = 0; pkeys < pkeyCap; pkeys += 32, ++amod) {
            uint32_t attrmod;
			Node_t *cache_nodep = NULL;
			Port_t *cache_portp = NULL;

			/* Note: If node was previously non-responding break out */
			if (nodep->nonRespCount) break;

            attrmod = amod;
            attrmod |= PKEY_BLOCK_NUM_MASK & block;
            attrmod |= (0xff & numBlocks) << 24;

			if(sm_find_cached_node_port(nodep, portp, &cache_nodep, &cache_portp) && sm_valid_port(cache_portp)) {
				pkp = (STL_PARTITION_TABLE*) cache_portp->portData->pPKey;
			} else {
				status = SM_Get_PKeyTable(fd_topology, attrmod, path, pkp, use_lr_dr_mix);
			}

			if (status != VSTATUS_OK) {
				if (sm_check_node_cache_valid(nodep)) break;
				
				IB_LOG_WARN_FMT(__func__,
					   "Failed to get Partition Table for node %s guid "FMT_U64" node index %d port index %d",
					   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
				return(status);
			}

            if (memcmp(pkp, &pkeyTable, sizeof(*pkp)) || sm_config.forceAttributeRewrite) { /* check to see if they are the same */
                if (isl && *enforcePkey == 0) {
                    // PKey enforcement has been disabled for ISLs, so synchronize
                    // the SA repository with the power-on default PKey Table
                    // values of the switch port.
                    memcpy(portp->portData->pPKey, pkp->PartitionTableBlock, sizeof(portp->portData->pPKey));
                } else {
                    memcpy(pkp, &pkeyTable, sizeof(*pkp));

                    status = SM_Set_PKeyTable(fd_topology, attrmod, path, pkp, sm_config.mkey, use_lr_dr_mix);
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
                }
            } else {
				IB_LOG_DEBUG1_FMT(__func__,
                       "Node %s ["FMT_U64"] Pkey table already setup",
                       sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
			}

            if (pkeys == 0) {	/* Only first table should be set until larger pkey table supported. */
                memset(&pkeyTable, 0, sizeof(pkeyTable));
            }
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

	status = SM_Get_PKeyTable(fd_topology, 1<<24, path, &pkeyTable, 0);
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
    status = SM_Set_PKeyTable(fd_topology, 1<<24, path, &pkeyTable, sm_config.mkey, 0);
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

    for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric[vf].apps.sidMap);
        cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric[vf].apps.sidMap);
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

	if (vf >= MAX_VFABRICS) return VSTATUS_BAD;

	// Check for service ID	
	if (smCheckServiceId(vf, serviceId, VirtualFabrics))
		return VSTATUS_OK;

	if (!VirtualFabrics->v_fabric[vf].apps.select_unmatched_sid)
		return VSTATUS_BAD;

	for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs && vf2 < MAX_VFABRICS; vf2++) {
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
smGetValidatedServiceIDVFs(Port_t* srcport, Port_t* dstport, uint16_t pkey, uint8_t reqSL, uint8_t respSL, uint64_t serviceId, bitset_t* vfs) {
	int			vf;
	uint8_t		appFound=0;
	uint8_t		srvIdInVF=0;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	for (vf = 0; vf < VirtualFabrics->number_of_vfs && vf < MAX_VFABRICS; vf++) {
		srvIdInVF = 0;
		uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;
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

		if (!appFound && !VirtualFabrics->v_fabric[vf].apps.select_unmatched_sid) continue;

		// Is this the proper vf?
		if ((pkey != 0) && (PKEY_VALUE(pkey) != PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey))) continue;

		// Are these the proper sls?
		if ((reqSL < STL_MAX_SLS) &&
			(reqSL != VirtualFabrics->v_fabric[vf].base_sl)) continue;
		if ((respSL < STL_MAX_SLS) &&
			(respSL != VirtualFabrics->v_fabric[vf].resp_sl)) continue;

		if (sm_config.enforceVFPathRecs) {
			// One is full?
			if (srcport != dstport &&
				!bitset_test(&srcport->portData->fullPKeyMember, vfIdx) &&
				!bitset_test(&dstport->portData->fullPKeyMember, vfIdx)) continue;

			// Are both src and dst part of this VF?
			if (!bitset_test(&srcport->portData->vfMember, vfIdx)) continue;
			if (!bitset_test(&dstport->portData->vfMember, vfIdx)) continue;

			bitset_set(vfs, vf);
		}
		else {

			// If we are not checking that the src/dst ports are in the same VF,
			// then need to check that pkey shared between the source and destination ports
			int	vf2;
			Port_t*  tstport = 0;
			if (srcport == dstport) tstport = dstport;
			else if (bitset_test(&srcport->portData->fullPKeyMember, vfIdx)) tstport = dstport;
			else if (bitset_test(&dstport->portData->fullPKeyMember, vfIdx)) tstport = srcport;

			if (tstport == 0) continue; // Case where neither port is a full member of this VF

			uint16_t tstpkey = VirtualFabrics->v_fabric[vf].pkey;
			uint8_t tstSL = VirtualFabrics->v_fabric[vf].base_sl;
			uint8_t tstrspSL = VirtualFabrics->v_fabric[vf].resp_sl;

			for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs && vf2 < MAX_VFABRICS; vf2++) {
				// Check for service ID
				if (smCheckServiceId(vf2, serviceId, VirtualFabrics)) {
					if (!appFound) {
						// Outer loop is using unmatched, exit this loop because this vf will be found later in outer loop.
						break;
					}
				} else if (appFound || !VirtualFabrics->v_fabric[vf2].apps.select_unmatched_sid) {
					// Two cases to skip:
					//    App is found and no service ID match on this vf, continue to next vf.
					//    Unmatched is in use and this vf is not using unmatched.
					continue;
				}

				// To have a path between VFs, the VFs must share the same PKEY and Base SL
				// (AND Response SL if either VF requires Response SLs). Only need to check
				// base SL b/c rules for SL sharing requires consistent pairing among VFs
				if (PKEY_VALUE(tstpkey) != PKEY_VALUE(VirtualFabrics->v_fabric[vf2].pkey)) continue;
				if (tstSL != VirtualFabrics->v_fabric[vf2].base_sl) continue;
				if ((VirtualFabrics->v_fabric[vf].requires_resp_sl || VirtualFabrics->v_fabric[vf2].requires_resp_sl) &&
					tstrspSL != VirtualFabrics->v_fabric[vf2].resp_sl) continue;

				vfIdx=VirtualFabrics->v_fabric[vf2].index;
				if (bitset_test(&tstport->portData->vfMember, vfIdx)) {
					bitset_set(vfs, vf2);
				}
			}
		}
	}

	return VSTATUS_OK;
}


Status_t
smGetValidatedVFs(Port_t* srcport, Port_t* dstport, uint16_t pkey, uint8_t reqSL, uint8_t respSL, bitset_t* vfs) {
	// TODO consolidate with smGetValidatedServiceIDVFs()
	int	vf, vfIdx;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	for (vf = 0; vf < VirtualFabrics->number_of_vfs; vf++) {
		// Is this the proper vf?
		if ((pkey != 0) && (PKEY_VALUE(pkey) != PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey))) continue;

		// Are these the proper sls?
		if ((reqSL < STL_MAX_SLS) &&
			(reqSL != VirtualFabrics->v_fabric[vf].base_sl)) continue;
		if ((respSL < STL_MAX_SLS) &&
			(respSL != VirtualFabrics->v_fabric[vf].resp_sl)) continue;
		vfIdx=VirtualFabrics->v_fabric[vf].index;

		if (sm_config.enforceVFPathRecs) {
			// One is full?
			if (
				(srcport != dstport) && !bitset_test(&srcport->portData->fullPKeyMember, vfIdx) &&
				!bitset_test(&dstport->portData->fullPKeyMember, vfIdx)) continue;

			// Are both src and dst part of this VF?
			if (!bitset_test(&srcport->portData->vfMember, vfIdx)) continue;
			if (!bitset_test(&dstport->portData->vfMember, vfIdx)) continue;

			bitset_set(vfs, vf);
		}
		else {

			// If we are not checking that the src/dst ports are in the same VF,
			// then need to check that pkey shared between the source and destination ports
			int	vf2;
			Port_t*  tstport = 0;
			if (bitset_test(&srcport->portData->fullPKeyMember, vfIdx)) tstport = dstport;
			else if (bitset_test(&dstport->portData->fullPKeyMember, vfIdx)) tstport = srcport;
			if (tstport == 0) {
				if (srcport == dstport)
					tstport = srcport; // Loopback case
				else 
					continue; // Case where neither port is a full member of this VF
			}

			uint16_t tstpkey = VirtualFabrics->v_fabric[vf].pkey;
			uint8_t tstSL = VirtualFabrics->v_fabric[vf].base_sl;

			for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs && vf2 < MAX_VFABRICS; vf2++) {
				// To have a path between VFs, the VFs must share the same PKEY AND Base SL
				// Only need to check base SL b/c rules for SL sharing requires consistent pairing among VFs
				if (PKEY_VALUE(tstpkey) != PKEY_VALUE(VirtualFabrics->v_fabric[vf2].pkey)) continue;
				if (tstSL != VirtualFabrics->v_fabric[vf2].base_sl) continue;

				vfIdx=VirtualFabrics->v_fabric[vf2].index;
				if (bitset_test(&tstport->portData->vfMember, vfIdx)) {
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
bool_t
smCheckMGid(int vf, uint64_t mGid[2])
{
    cl_map_item_t   *cl_map_item;
	VFAppMgid_t 	*app;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	for_all_qmap_ptr(&VirtualFabrics->v_fabric[vf].apps.mgidMap, cl_map_item, app) {
		if (mgidMatch(mGid, app)) {
			return TRUE;
		}
	}
	return FALSE;
}

// validate mGid for VF.  Also covers the unmatchedMGid option
Status_t
smVFValidateVfMGid(int vf, uint64_t mGid[2])
{
	int vf2;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (vf >= MAX_VFABRICS) return VSTATUS_BAD;

	// Check for MGID	
	if (smCheckMGid(vf, mGid))
		return VSTATUS_OK;

	if (!VirtualFabrics->v_fabric[vf].apps.select_unmatched_mgid)
		return VSTATUS_BAD;

	for (vf2 = 0; vf2 < VirtualFabrics->number_of_vfs && vf2 < MAX_VFABRICS; vf2++) {
		if (smCheckMGid(vf2, mGid))
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
	for (vf = 0; vf < VirtualFabrics->number_of_vfs; vf++) {
    	cl_map_item_t   *it;
		VF_t *vfp = &VirtualFabrics->v_fabric[vf];
		uint32 vfIdx=vfp->index;

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
		   || (!bitset_test(&joiner->portData->vfMember, vfIdx))
		      // Check to see if the (optional) requestor port is a member of this VF
		   || (requestor != NULL && !bitset_test(&requestor->portData->vfMember, vfIdx))) {
			continue;
		}

		if ((vfp->mcast_sl != UNDEFINED_XML8 && mcMemberRec->SL != vfp->mcast_sl) ||
			(vfp->mcast_sl == UNDEFINED_XML8 && mcMemberRec->SL != vfp->base_sl))
			continue;

		if (vfp->security &&
		   !bitset_test(&joiner->portData->fullPKeyMember, vfIdx)) {
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


Status_t
smVFValidateMcDefaultGroup(int vf, uint64_t* mGid) {
    cl_map_item_t   *cl_map_item;
	VFAppMgid_t	*app;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	if (vf < 0 || vf >= MAX_VFABRICS) return VSTATUS_BAD;

	if (!VirtualFabrics) return VSTATUS_OK;

	if (VirtualFabrics->v_fabric[vf].apps.select_unmatched_mgid) return VSTATUS_OK;


    for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric[vf].apps.mgidMap);
        cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric[vf].apps.mgidMap);
        cl_map_item = cl_qmap_next(cl_map_item)) {
        app = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
		if (mgidMatch(mGid, app)) {
			return VSTATUS_OK;
		}
	}

	return VSTATUS_BAD;
}

// virtual fabric name given index
char* smGetVfName(uint16_t pKey) {
	char	*name = NULL;
	int		vf;
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

    if (!VirtualFabrics ||
    	(VirtualFabrics->number_of_vfs == 0)) return NULL;

	for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
		if (PKEY_VALUE(pKey) == PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey)) {
			if (!name) {
				name = VirtualFabrics->v_fabric[vf].name;
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

	for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
    	cl_map_item_t   *cl_map_item;
		uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;

		if (PKEY_VALUE(mcmp->P_Key) != PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey)) continue;
		if (!((mcmp->SL == VirtualFabrics->v_fabric[vf].base_sl) ||
			(mcmp->SL == VirtualFabrics->v_fabric[vf].resp_sl))) continue;
		if (!bitset_test(&portp->portData->vfMember, vfIdx)) continue;
		if (!bitset_test(&reqportp->portData->vfMember, vfIdx)) continue;

		if (VirtualFabrics->v_fabric[vf].security &&
		    !bitset_test(&portp->portData->fullPKeyMember, vfIdx)) {
			// Create/joiner must be full member
			continue;
		}

    	for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric[vf].apps.mgidMap);
        	cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric[vf].apps.mgidMap);
        	cl_map_item = cl_qmap_next(cl_map_item)) {
        	app = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			if (mgidMatch(mGid, app)) {
				mgidFound = 1;
				*maxMtu =  MAX(*maxMtu, VirtualFabrics->v_fabric[vf].max_mtu_int);
				if (linkrate_lt(*maxRate, VirtualFabrics->v_fabric[vf].max_rate_int)) {
					*maxRate = VirtualFabrics->v_fabric[vf].max_rate_int;
				}
				break;
			}
		}

		if (mgidFound) continue;

		if (VirtualFabrics->v_fabric[vf].apps.select_unmatched_mgid) {
			unmatchedAllMtu = MAX(unmatchedAllMtu, VirtualFabrics->v_fabric[vf].max_mtu_int);
			if (linkrate_lt(unmatchedAllRate, VirtualFabrics->v_fabric[vf].max_rate_int)) {
				unmatchedAllRate = VirtualFabrics->v_fabric[vf].max_rate_int;
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

	for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {

		//VF_t index is the index into the vf_config array
		int vfIdx=VirtualFabrics->v_fabric[vf].index;
		vfp = vf_config.vf[vfIdx];

		//loop on all DG members that are included and set vfMember based on already acquired knowledge

		//process all full members
		int isFullMember;
		int arrayIdx;
		for (arrayIdx=0; arrayIdx<vfp->number_of_full_members; arrayIdx++) {
			isFullMember = 1;
			smProcessVFNodeMembers(nodep, vfIdx, vfp->full_member[arrayIdx].member, isFullMember);
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

			smProcessVFNodeMembers(nodep, vfIdx, vfp->limited_member[arrayIdx].member, isFullMember);
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

	//Evaluate node against node specific criteria for each defined device group
	boolean isNodeMember = FALSE;
	bitset_t dgNodeMemberBitset;
	bitset_init(&sm_pool, &dgNodeMemberBitset, MAX_VFABRIC_GROUPS);

	//Save port ranges for dgMembers
	int numPorts = 2;
	int dgPortRanges[numGroups][MAX_NODE_DESC_ENTRIES][numPorts];

	PortRangeInfo_t portInfo;
	int numValidPortMatches[numGroups];
	memset(numValidPortMatches, 0, sizeof(int)*numGroups);

	for (dgIdx = 0; dgIdx < numGroups; dgIdx++) {
		
		//clear num port ranges defined
		portInfo.numPortRanges = 0;

		isNodeMember = smEvaluateNodeDG(nodep, dgIdx, &portInfo);


		if (isNodeMember) {

			bitset_set(&dgNodeMemberBitset, dgIdx);
			
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
	for_all_ports(nodep,portp) {

		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;
		else {

			// Create bitmask to track DG membership for this port
			bitset_init(&sm_pool, &portp->portData->dgMember, MAX_VFABRIC_GROUPS);

			//create bitset to track which DGs we've evaluated for this node
			bitset_t dgsEvaluated;
			bitset_init(&sm_pool, &dgsEvaluated, numGroups);

			bool_t isMember = FALSE;

			// loop on all device groups to determine if the given port is a member each device group
			for (dgIdx = 0; dgIdx < numGroups; dgIdx++) {

				if (bitset_test(&dgNodeMemberBitset, dgIdx)) {

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

					}
					else {
						bitset_set(&portp->portData->dgMember, dgIdx);
						bitset_set(&dgsEvaluated, dgIdx);
					}

				}

				isMember = smEvaluatePortDG(nodep, portp, dgIdx, &portp->portData->dgMember, &dgsEvaluated);

				if (isMember)
					bitset_set(&portp->portData->dgMember, dgIdx);				
			}

            		bitset_free(&dgsEvaluated);

			// Update dgMember array
			PortData_t *portDataPtr = portp->portData;
			int numMemberships = 0;

			//loop for each device group
			for (dgIdx = 0; dgIdx < numGroups; dgIdx++) {				
				if (bitset_test(&portp->portData->dgMember, dgIdx)) {
					//determine if max number of DGs has been exceeded
					if (numMemberships < MAX_DEVGROUPS) {
						portDataPtr->dgMemberList[numMemberships] = dgIdx;
						numMemberships++;
					}
					else {
						IB_LOG_WARN_FMT(__func__, "Node %s not added to device group %s - Max members exceeeded", sm_nodeDescString(nodep), dg_config.dg[dgIdx]->name);
					}
				}
			}

			//Fill in default value for rest of the dgMember array
			while (numMemberships < MAX_DEVGROUPS) {
				portDataPtr->dgMemberList[numMemberships] = DEFAULT_DEVGROUP_ID;
				numMemberships++;
			}
		}
	}

    	bitset_free(&dgNodeMemberBitset);
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
			VirtualFabrics->number_of_vfs, VirtualFabrics->securityEnabled, VirtualFabrics->qosEnabled);

	for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
		vfp = VirtualFabrics->v_fabric[vf].name;
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "Index= %d, pkey= 0x%x, security= %d ",
				VirtualFabrics->v_fabric[vf].index,
				VirtualFabrics->v_fabric[vf].pkey, VirtualFabrics->v_fabric[vf].security);

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF max_mtu_int= %d  max_rate_int= %d  pkt_lifetime_inc= %d ",
				VirtualFabrics->v_fabric[vf].max_mtu_int, VirtualFabrics->v_fabric[vf].max_rate_int,
				VirtualFabrics->v_fabric[vf].pkt_lifetime_mult);
		
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "bandwidthPercent= %d  priority= %d",
				VirtualFabrics->v_fabric[vf].percent_bandwidth, VirtualFabrics->v_fabric[vf].priority);
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "preemptionRank= %d  hoqLife= %d",
				VirtualFabrics->v_fabric[vf].preempt_rank, VirtualFabrics->v_fabric[vf].hoqlife_vf);

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF APPs:");
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tselect_sa= %d, select_unmatched_sid= %d, select_unmatched_mgid= %d, select_pm= %d",
				VirtualFabrics->v_fabric[vf].apps.select_sa,
				VirtualFabrics->v_fabric[vf].apps.select_unmatched_sid,
				VirtualFabrics->v_fabric[vf].apps.select_unmatched_mgid,
				VirtualFabrics->v_fabric[vf].apps.select_pm);

    	for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric[vf].apps.sidMap);
        	cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric[vf].apps.sidMap);
        	cl_map_item = cl_qmap_next(cl_map_item)) {
        	sidp = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tservice_id "FMT_U64", service_id_last "FMT_U64" service_id_mask  "FMT_U64"",
						sidp->service_id, sidp->service_id_last, sidp->service_id_mask);
		}

    	for (cl_map_item = cl_qmap_head(&VirtualFabrics->v_fabric[vf].apps.mgidMap);
        	cl_map_item != cl_qmap_end(&VirtualFabrics->v_fabric[vf].apps.mgidMap);
        	cl_map_item = cl_qmap_next(cl_map_item)) {
        	mgidp = XML_QMAP_VOID_CAST cl_qmap_key(cl_map_item);
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tmgid " FMT_GID "", mgidp->mgid[0], mgidp->mgid[1]);
		}

		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF Full Members:");
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tselect_all=%d, select_self= %d, select_swe0=%d, select_hfi_direct_connect=%d",
				VirtualFabrics->v_fabric[vf].full_members.select_all, VirtualFabrics->v_fabric[vf].full_members.select_self,
				VirtualFabrics->v_fabric[vf].full_members.select_swe0,
				VirtualFabrics->v_fabric[vf].full_members.select_hfi_direct_connect);
		IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnode_type_fi=%d, node_type_sw= %d,",
				VirtualFabrics->v_fabric[vf].full_members.node_type_fi, VirtualFabrics->v_fabric[vf].full_members.node_type_sw);

		cl_map = &VirtualFabrics->v_fabric[vf].full_members.sysGuidMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tsysGuid=  "FMT_U64"", cl_map_item->key);
		}
		cl_map = &VirtualFabrics->v_fabric[vf].full_members.nodeGuidMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeGuid=  "FMT_U64"", cl_map_item->key);
		}
		cl_map = &VirtualFabrics->v_fabric[vf].full_members.portGuidMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tportGuid=  "FMT_U64"", cl_map_item->key);
		}
		cl_map = &VirtualFabrics->v_fabric[vf].full_members.nodeDescMap;
		for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", XML_QMAP_CHAR_CAST cl_map_item->key);
		}

/*
		for (nodeDescrp=VirtualFabrics->v_fabric[vf].full_members.node_descr; nodeDescrp; nodeDescrp=nodeDescrp->next_node_description) {
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", nodeDescrp->node_description);
		}
*/

		if (VirtualFabrics->v_fabric[vf].security) {	
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "VF Limited Members:");
			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tselect_all= %d, select_self= %d, select_swe0= %d, select_hfi_direct_connect=%d",
				VirtualFabrics->v_fabric[vf].limited_members.select_all, VirtualFabrics->v_fabric[vf].limited_members.select_self,
				VirtualFabrics->v_fabric[vf].limited_members.select_swe0, 
				VirtualFabrics->v_fabric[vf].limited_members.select_hfi_direct_connect);

			IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnode_type_fi= %d, node_type_sw= %d",
				VirtualFabrics->v_fabric[vf].limited_members.node_type_fi, VirtualFabrics->v_fabric[vf].limited_members.node_type_sw);
			cl_map = &VirtualFabrics->v_fabric[vf].limited_members.sysGuidMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tsysGuid=  "FMT_U64"", cl_map_item->key);
			}
			cl_map = &VirtualFabrics->v_fabric[vf].limited_members.nodeGuidMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeGuid=  "FMT_U64"", cl_map_item->key);
			}
			cl_map = &VirtualFabrics->v_fabric[vf].limited_members.portGuidMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tportGuid=  "FMT_U64"", cl_map_item->key);
			}
			cl_map = &VirtualFabrics->v_fabric[vf].limited_members.nodeDescMap;
			for (cl_map_item = cl_qmap_head(cl_map); cl_map_item != cl_qmap_end(cl_map); cl_map_item = cl_qmap_next(cl_map_item)) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", XML_QMAP_CHAR_CAST cl_map_item->key);
			}
/*
			for (nodeDescrp=VirtualFabrics->v_fabric[vf].limited_members.node_descr; nodeDescrp; nodeDescrp=nodeDescrp->next_node_description) {
				IB_LOG_INFINI_INFO_FMT_VF( vfp, "smLogVFs", "\tnodeDescription= %s", nodeDescrp->node_description);
			}
*/
		}
		for (mcastGrpp = VirtualFabrics->v_fabric[vf].default_group; mcastGrpp; 
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
