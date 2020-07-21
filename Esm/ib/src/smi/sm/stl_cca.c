/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <iba/stl_sm_priv.h>
#include "stl_cca.h"
#include "sm_parallelsweep.h"

Status_t stl_sm_cca_hfi_ref_acquire(HfiCongestionControlTableRefCount_t** congConrfCount, uint32_t tableSize) {
	Status_t status;
	if ((status = vs_pool_alloc(&sm_pool, tableSize, (void *)&(*congConrfCount))) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
				"No memory for HFI Congestion Control Table");
		return status;
	}
	else {
		memset(*congConrfCount, 0, tableSize);
		(*congConrfCount)->refCount++;
		return status;
	}
}

HfiCongestionControlTableRefCount_t* stl_sm_cca_hfi_ref_copyptr(HfiCongestionControlTableRefCount_t* congConrfCount) {
	if (congConrfCount) {
		congConrfCount->refCount++;
		return congConrfCount;
	}
	else
		return congConrfCount;
}

void stl_sm_cca_hfi_ref_release(HfiCongestionControlTableRefCount_t** congConRefCount) {
	if (*congConRefCount) {
		if(--(*congConRefCount)->refCount == 0)
			vs_pool_free(&sm_pool, (*congConRefCount));
			*congConRefCount = NULL;
	}
}

static Status_t build_cca_congestion_control_table(Node_t *nodep, Port_t *portp, HfiCongestionControlTableRefCount_t *congConRefCount,
int cap)
{
	const unsigned int mtu = GetBytesFromMtu(portp->portData->maxVlMtu);
	const double packet_xmit_time = (double)(mtu + 40) / (double)sm_GetBandwidth(&portp->portData->portInfo);
	uint32_t maxMultiplier;
	uint64_t maxIPG_shifted;
	unsigned int i, j, b, tmp;
	uint8_t shift;

	if (mtu == 0) {
		IB_LOG_WARN_FMT(__func__,
			"MTU for this port is invalid - NodeGUID "FMT_U64" [%s]",
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
		return VSTATUS_BAD;
	}


	// As defined in STL Spec Vol 1g1 17.3.12, Inter Packet Gap (IPG) is defined
	// as:
	//      IPG = packet_transmit_time>>shift_field * multiplier.
	// 
	// From this definition, we can derive required multiplier value to get the IPG
	// the user wants:
	//      multiplier = (IPG<<shift_field) / packet_transmit_time.
	//
	// We do this by trying the largest possible value of shift_field, reducing it as necessary
	// to get a valid multiplier value.
	// Note: IPG is in nanoseconds (10^-9 seconds).
	maxIPG_shifted = ((uint64_t)sm_config.congestion.ca.desired_max_delay)<<(shift = 3);
	maxMultiplier = ((double)maxIPG_shifted * 1.0E-9) / packet_xmit_time; //packet_xmit_time is in seconds.

	// multiplier can't occupy more than 14 bits.
	while (maxMultiplier & ~((uint32_t)0x3fff) && --shift) {
		maxMultiplier>>=1;
	}

	if (shift == 0) {
		maxMultiplier = 0x3fff;
		IB_LOG_ERROR_FMT(__func__,
			"DesiredMaxDelay of %u nS specified in config unrealizable! Using max possible value of %u nS - NodeGUID "FMT_U64" [%s]",
			sm_config.congestion.ca.desired_max_delay,
			(uint32_t)(packet_xmit_time * 1E9 * maxMultiplier), // See calculation above (assume shift = 0).
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
	}

	// CCTI_Limit is max valid index = num valid entries-1
	tmp = cap ? (unsigned)(cap * STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES - 1) : 0;
	congConRefCount->hfiCongCon.CCTI_Limit = MIN(sm_config.congestion.ca.limit, tmp);
	if (congConRefCount->hfiCongCon.CCTI_Limit != sm_config.congestion.ca.limit)
		IB_LOG_WARN_FMT(__func__,
			"SM Config has too large a CCT Index Limit: %d; max is %d for node: NodeGUID "FMT_U64" [%s]",
 				sm_config.congestion.ca.limit,
				cap*STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES-1,
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
	
	for (b = 0,i = 0; b <= cap; ++b) {
		STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK *block = &congConRefCount->hfiCongCon.CCT_Block_List[b];

		for(j = 0; i <= congConRefCount->hfiCongCon.CCTI_Limit && j < STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES; ++i, ++j){
			if (i == 0) continue;
			STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY *entry = &block->CCT_Entry_List[j];
			entry->s.CCT_Shift = shift;
			entry->s.CCT_Multiplier = i * ((double)maxMultiplier / (double)congConRefCount->hfiCongCon.CCTI_Limit);
		}
	}
	
	return VSTATUS_OK;
}

Status_t stl_sm_cca_configure_hfi(ParallelSweepContext_t *psc, SmMaiHandle_t *fd, Node_t *nodep)
{
	Status_t status;
	Port_t *portp;
	STL_HFI_CONGESTION_SETTING hfiCongSett;
	STL_LID dlid;

	// Can be called for ESP0. Return if Switch port 0 has no congestion table capacity.
	if(nodep->nodeInfo.NodeType==STL_NODE_SW) {
		if(!is_cc_supported_by_enhanceport0(nodep))
			return (VSTATUS_OK);
	}
	memset(&hfiCongSett, 0, sizeof(STL_HFI_CONGESTION_SETTING));
	if (sm_config.congestion.enable) {
		HfiCongestionControlTableRefCount_t *congConRefCount = NULL;
		uint8_t numBlocks = nodep->congestionInfo.ControlTableCap;
		const uint32_t tableSize = getCongrefTableSize(numBlocks);
		const uint8_t maxBlocks = (STL_MAD_PAYLOAD_SIZE - CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ) /
								 sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK); // Maximum number of blocks we can fit in a single MAD;
		unsigned int i;

		/* Build the congestion control table for each end port. */
		for_all_end_ports(nodep, portp) {

			if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN 
				) {
				continue;
			}
		
			if((status = stl_sm_cca_hfi_ref_acquire(&congConRefCount, tableSize)) != VSTATUS_OK)
				return status;

			build_cca_congestion_control_table(nodep, portp, congConRefCount, numBlocks);

			if (!portp->portData->congConRefCount || 0 != memcmp(portp->portData->congConRefCount, congConRefCount, tableSize)) { // if not same
				dlid = portp->portData->lid;
				// CCTI_Limit is max valid index = num valid entries-1
				numBlocks = MIN(numBlocks, (congConRefCount->hfiCongCon.CCTI_Limit + STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES) / STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES);
				for(i = 0; i < numBlocks;){
					uint8_t payloadBlocks; // How many blocks being sent in this MAD
					uint32_t amod = 0;

					payloadBlocks = numBlocks - i;  // Calculate how many blocks are left to be sent
					payloadBlocks = MIN(payloadBlocks, maxBlocks);  // Limit the number of blocks to be sent
					amod = payloadBlocks << 24 | i;
					SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
					psc_unlock(psc);
					status = SM_Set_HfiCongestionControl(fd, congConRefCount->hfiCongCon.CCTI_Limit,
							payloadBlocks, amod, &addr,
							&congConRefCount->hfiCongCon.CCT_Block_List[i], sm_config.mkey);
					psc_lock(psc);
					if(status != VSTATUS_OK)
						break;
					else
						i += payloadBlocks;
				}
				if (status != VSTATUS_OK) {
					IB_LOG_ERROR_FMT(__func__,
						"Failed to set HFI Congestion Control Table for NodeGUID "FMT_U64" [%s], portGUID "
						FMT_U64"; rc: %d, NumBlocks: %u", nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), 
						portp->portData->guid, status, numBlocks);
					stl_sm_cca_hfi_ref_release(&congConRefCount);
					return status;
				}
				stl_sm_cca_hfi_ref_release(&portp->portData->congConRefCount);
				portp->portData->congConRefCount = congConRefCount;
			} else
				stl_sm_cca_hfi_ref_release(&congConRefCount);
		}


		for (i = 0; i < STL_MAX_SLS; ++i) {
			STL_HFI_CONGESTION_SETTING_ENTRY *current = &hfiCongSett.HFICongestionEntryList[i];
			
			current->CCTI_Increase = sm_config.congestion.ca.increase;
			current->CCTI_Timer = sm_config.congestion.ca.timer;
			current->TriggerThreshold = sm_config.congestion.ca.threshold;
			current->CCTI_Min = sm_config.congestion.ca.min;
		}
	}
	

	hfiCongSett.Port_Control = sm_config.congestion.ca.sl_based ? 
									CC_HFI_CONGESTION_SETTING_SL_PORT : 0;
	//FM code always updates each SL settings, we have not implemented a way to
	//change settings on individual SLs, so Control_map bits always set. 
	hfiCongSett.Control_Map = 0xffffffff;
	
	portp = sm_get_node_end_port(nodep);
	if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN){
		return VSTATUS_BAD;
	}


	if (0 != memcmp(&nodep->hfiCongestionSetting, &hfiCongSett, sizeof(STL_HFI_CONGESTION_SETTING))) { // if not same
		dlid = portp->portData->lid;
		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
		psc_unlock(psc);
		status = SM_Set_HfiCongestionSetting(fd, 0, &addr, &hfiCongSett, sm_config.mkey);
		psc_lock(psc);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to set HFI Congestion Setting Table for NodeGUID "FMT_U64" [%s]; rc: %d",
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
			return status;
		}
	
		nodep->hfiCongestionSetting = hfiCongSett;
	}

	return VSTATUS_OK;
}


uint16 stl_sm_cca_resolve_marking_rate(Node_t *nodep, uint16 rate)
{
	// PRR limitation of 0-255 for Marking_Rate (HSD 291608)
	uint16 new_rate = MIN(255,rate);


	return new_rate;
}


Status_t stl_sm_cca_configure_sw(ParallelSweepContext_t *psc, SmMaiHandle_t *fd, Node_t *nodep)
{
	Status_t status=VSTATUS_OK;
	STL_SWITCH_CONGESTION_SETTING setting;
	STL_SWITCH_PORT_CONGESTION_SETTING * swPortSet;
	const uint8_t port_count = nodep->nodeInfo.NumPorts + 1; // index 0 reserved for SWP0.
	uint8_t i;
	STL_LID dlid;
	Port_t *portp;

	if (nodep->switchInfo.u2.s.EnhancedPort0) 
		stl_sm_cca_configure_hfi(psc, fd, nodep);

	memset(&setting, 0, sizeof(STL_SWITCH_CONGESTION_SETTING));
	if (sm_config.congestion.enable) {

		setting.Control_Map = CC_SWITCH_CONTROL_MAP_VICTIM_VALID
							| CC_SWITCH_CONTROL_MAP_CC_VALID
							| CC_SWITCH_CONTROL_MAP_MARKING_VALID;

		setting.Threshold      = sm_config.congestion.sw.threshold;
		setting.Packet_Size    = sm_config.congestion.sw.packet_size;
		setting.Marking_Rate   = stl_sm_cca_resolve_marking_rate(nodep, sm_config.congestion.sw.marking_rate);


		if(sm_config.congestion.sw.victim_marking_enable){

			i = port_count / 8;
			// Victim Mask is a 256 bitfield, stored most significant bit first.
			// (e.g. Port 0 = Victim_Mask[31] & 0x1)
			// Sets victim_marking_enable on all valid ports of the switch.
			setting.Victim_Mask[31 - i] = (1<<(port_count % 8)) - 1; 
			while (i > 0) setting.Victim_Mask[31 - --i] = 0xff;
			setting.Victim_Mask[31] &= 0xfe;			//As SWP0 doesn't support CC.
		}
		
	} else {
		// Disable CCA on this switch
		setting.Control_Map = CC_SWITCH_CONTROL_MAP_CC_VALID;
		setting.Threshold = 0;
	}
 
	portp = sm_get_port(nodep, 0);
	if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN){
		return VSTATUS_BAD;
	}

	dlid = portp->portData->lid;
	if (0 != memcmp(&nodep->swCongestionSetting, &setting, sizeof(STL_SWITCH_CONGESTION_SETTING))) { // if not same
		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
		psc_unlock(psc);
		status = SM_Set_SwitchCongestionSetting(fd, 0, &addr, &setting, sm_config.mkey);
		psc_lock(psc);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to set switch Congestion Setting for NodeGUID "FMT_U64" [%s]; rc: %d",
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
			return status;
		}
		nodep->swCongestionSetting = setting;
	}

	if (!sm_config.congestion.enable)
		return status;


	if (vs_pool_alloc(&sm_pool, sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT) * port_count, 
			(void *)&swPortSet) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate memory for switch port congestion setting.");
		return VSTATUS_NOMEM;
	}

	SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);
	psc_unlock(psc);
	status = SM_Get_SwitchPortCongestionSetting(fd, ((uint32_t)port_count)<<24, &addr, swPortSet);
	psc_lock(psc);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to get Switch Port Congestion Settings for NodeGUID "FMT_U64" [%s]; rc %d",
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
		(void)vs_pool_free(&sm_pool, (void *)swPortSet);
		return status;
	}

	for (i = 0; i < port_count; ++i) {
		Port_t* portp = sm_get_port(nodep, i);
		if (!sm_valid_port(portp))
			continue;
		portp->portData->swPortCongSet = swPortSet->Elements[i];
	}
	
	(void)vs_pool_free(&sm_pool, (void *)swPortSet);
	
	return VSTATUS_OK;
}

Status_t stl_sm_cca_copy_hfi_data(Node_t *nodep)
{
	Node_t		*oldnodep;
	Port_t		*portp, *oldportp;

	if (!nodep) {
		return (VSTATUS_OK);
	}
	// Can be called for ESP0. Return if Switch port 0 has no congestion table capacity.
	if ( (nodep->nodeInfo.NodeType==STL_NODE_SW) &&
			(!nodep->switchInfo.u2.s.EnhancedPort0 ||
			(nodep->congestionInfo.ControlTableCap == 0)) ) {
		return (VSTATUS_OK);
	}

	for_all_end_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN){
			return VSTATUS_BAD;
		}
		if ((oldnodep = lidmap[portp->portData->lid].oldNodep)) {
			memcpy((void *) &nodep->hfiCongestionSetting, &oldnodep->hfiCongestionSetting, sizeof(STL_HFI_CONGESTION_SETTING));
			oldportp = sm_get_port(oldnodep, portp->index);
			if (sm_valid_port(oldportp)
				) {
				// copy over hfiCongCon mem block pointer
				portp->portData->congConRefCount = stl_sm_cca_hfi_ref_copyptr(oldportp->portData->congConRefCount);
			}
			else {
				nodep->congConfigDone = 0;
			}
		}
		else {
			return VSTATUS_BAD;
		}
	}

	return VSTATUS_OK;
}

Status_t stl_sm_cca_copy_sw_data(Node_t *nodep)
{
	Node_t		*oldswp;
	Port_t		*portp, *oldportp;
	uint8_t		i, port_count;

	if (!nodep) {
		return (VSTATUS_OK);
	}
	if (nodep->switchInfo.u2.s.EnhancedPort0) {
		(void)stl_sm_cca_copy_hfi_data(nodep);
	}

	portp = sm_get_port(nodep,0);
	if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN){
		return VSTATUS_BAD;
	}
	else {
		if ((oldswp = lidmap[portp->portData->lid].oldNodep)) {
			if (nodep->initPorts.nset_m ||
					!bitset_equal(&nodep->activePorts, &oldswp->activePorts) ||
					oldswp->switchInfo.u1.s.PortStateChange) {
				nodep->congConfigDone = 0; // port count/state change detected; force reconfigure
				return VSTATUS_OK;
			}
			/* copy sw cca data over into new topology */
			memcpy((void *) &nodep->swCongestionSetting, &oldswp->swCongestionSetting, sizeof(STL_SWITCH_CONGESTION_SETTING));
			port_count = nodep->nodeInfo.NumPorts + 1; // index 0 reserved for SWP0.
			for (i = 0; i < port_count; ++i) {
				if (sm_valid_port((portp = sm_get_port(nodep, i))) &&
						sm_valid_port((oldportp = sm_get_port(oldswp, i)))) {
					memcpy((void *) &portp->portData->swPortCongSet, &oldportp->portData->swPortCongSet,
									sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT));
				}
			}
		}
	}
	return VSTATUS_OK;
}

/*
 *  copy the congestion control configure status to new topology
 */
Status_t sm_cong_config_copy() {
	Topology_t	*newtopop;
	Node_t		*nodep, *oldnodep;
	Port_t		*portp;

	newtopop = (Topology_t *)&sm_newTopology;

	for_all_nodes(newtopop, nodep) {
		portp = sm_get_node_end_port(nodep);
		if (sm_valid_port(portp)) {
			if ((oldnodep = lidmap[portp->portData->lid].oldNodep)) {
				/* copy congestion control configure status to new topology */
				nodep->congConfigDone = oldnodep->congConfigDone;
				nodep->congestionInfo = oldnodep->congestionInfo;

				if (nodep->congConfigDone) { // copy if config was done
					if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
						(void)stl_sm_cca_copy_sw_data(nodep);
					}
					else { // HFI
						(void)stl_sm_cca_copy_hfi_data(nodep);
					}
				}
			}
		}
	}
	return VSTATUS_OK;
}
