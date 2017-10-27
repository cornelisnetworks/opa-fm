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

#include <iba/stl_sm_priv.h>
#include "stl_cca.h"

static Status_t build_cca_congestion_control_table(Node_t *nodep, Port_t *portp, STL_HFI_CONGESTION_CONTROL_TABLE *hfiCongCon,
int cap)
{
	const unsigned int mtu = GetBytesFromMtu(portp->portData->maxVlMtu);
	const double packet_xmit_time = (double)(mtu + 40) / (double)sm_GetBandwidth(&portp->portData->portInfo);
	uint32_t maxMultiplier;
	uint64_t maxIPG_shifted;
	unsigned int i, j, b;
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
	hfiCongCon->CCTI_Limit = MIN(sm_config.congestion.ca.limit, cap * STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES - 1);
	if (hfiCongCon->CCTI_Limit != sm_config.congestion.ca.limit)
		IB_LOG_WARN_FMT(__func__,
			"SM Config has too large a CCT Index Limit: %d; max is %d for node: NodeGUID "FMT_U64" [%s]",
 				sm_config.congestion.ca.limit,
				cap*STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES-1,
				nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
	
	for (b = 0,i = 0; b <= cap; ++b) {
		STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK *block = &hfiCongCon->CCT_Block_List[b];
		
		for(j = 0; i <= hfiCongCon->CCTI_Limit && j < STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES; ++i, ++j){
			if (i == 0) continue;
			STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY *entry = &block->CCT_Entry_List[j];
			entry->s.CCT_Shift = shift;
			entry->s.CCT_Multiplier = i * ((double)maxMultiplier / (double)hfiCongCon->CCTI_Limit);
		}
	}
	
	return VSTATUS_OK;
}


Status_t stl_sm_cca_configure_hfi(Node_t *nodep)
{
	Status_t status;
	Port_t *portp;
	STL_HFI_CONGESTION_SETTING hfiCongSett = {0};
	uint32_t dlid;

	// Can be called for ESP0. Return if Switch port 0 has no congestion table capacity.
	if ( (nodep->nodeInfo.NodeType==STL_NODE_SW) &&
		  nodep->switchInfo.u2.s.EnhancedPort0      &&
		  (nodep->congestionInfo.ControlTableCap == 0) ) {
		return (VSTATUS_OK);
	}

	if (sm_config.congestion.enable) {
		STL_HFI_CONGESTION_CONTROL_TABLE *hfiCongCon = NULL;
		uint8_t numBlocks = nodep->congestionInfo.ControlTableCap;
		const size_t tableSize = CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ + sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK) * numBlocks;
		const uint8_t maxBlocks = (STL_MAD_PAYLOAD_SIZE - CONGESTION_CONTROL_TABLE_CCTILIMIT_SZ) /
								 sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK); // Maximum number of blocks we can fit in a single MAD;
		unsigned int i;

		/* Build the congestion control table for each end port. */
		for_all_end_ports(nodep, portp) {

			if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN) continue;
		
			if ((status = vs_pool_alloc(&sm_pool, tableSize, (void *)&hfiCongCon)) != VSTATUS_OK) {
				IB_LOG_ERROR_FMT(__func__,
					"No memory for HFI Congestion Control Table");
				return status;
			}
		
			memset(hfiCongCon, 0, tableSize);
		
			build_cca_congestion_control_table(nodep, portp, hfiCongCon, numBlocks);

			dlid = portp->portData->lid;
			// CCTI_Limit is max valid index = num valid entries-1
			numBlocks = MIN(numBlocks, (hfiCongCon->CCTI_Limit + STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES) / STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES);
			for(i = 0; i < numBlocks;){
				uint8_t payloadBlocks; // How many blocks being sent in this MAD
				uint32_t amod = 0;

				payloadBlocks = numBlocks - i;  // Calculate how many blocks are left to be sent
				payloadBlocks = MIN(payloadBlocks, maxBlocks);  // Limit the number of blocks to be sent
				amod = payloadBlocks << 24 | i;
				status = SM_Set_HfiCongestionControl_LR(fd_topology, hfiCongCon->CCTI_Limit,
													 payloadBlocks, amod, sm_lid, dlid, &hfiCongCon->CCT_Block_List[i], sm_config.mkey);
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
				vs_pool_free(&sm_pool, hfiCongCon);
				return status;
			}

			portp->portData->hfiCongCon = hfiCongCon;
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

	dlid = portp->portData->lid;
	status = SM_Set_HfiCongestionSetting_LR(fd_topology, 0, sm_lid, dlid, &hfiCongSett, sm_config.mkey);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to set HFI Congestion Setting Table for NodeGUID "FMT_U64" [%s]; rc: %d",
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
		return status;
	}
	
	nodep->hfiCongestionSetting = hfiCongSett;

	return VSTATUS_OK;
}

Status_t stl_sm_cca_configure_sw(Node_t *nodep)
{
	Status_t status;
	STL_SWITCH_CONGESTION_SETTING setting = { 0 };
	STL_SWITCH_PORT_CONGESTION_SETTING * swPortSet;
	const uint8_t port_count = nodep->nodeInfo.NumPorts + 1; // index 0 reserved for SWP0.
	uint8_t i;
	uint32_t dlid;
	Port_t *portp;

	if (nodep->switchInfo.u2.s.EnhancedPort0) 
		stl_sm_cca_configure_hfi(nodep);

	if (sm_config.congestion.enable) {

		setting.Control_Map = CC_SWITCH_CONTROL_MAP_VICTIM_VALID
							| CC_SWITCH_CONTROL_MAP_CREDIT_VALID
							| CC_SWITCH_CONTROL_MAP_CC_VALID
							| CC_SWITCH_CONTROL_MAP_CS_VALID
							| CC_SWITCH_CONTROL_MAP_MARKING_VALID;

		setting.Threshold     = sm_config.congestion.sw.threshold;
		setting.Packet_Size    = sm_config.congestion.sw.packet_size;
		setting.CS_Threshold   = sm_config.congestion.sw.cs_threshold;
		setting.CS_ReturnDelay = sm_config.congestion.sw.cs_return_delay;
		// PRR limitation of 0-255 for Marking_Rate (HSD 291608)
		setting.Marking_Rate   = MIN(255,sm_config.congestion.sw.marking_rate);

		if(sm_config.congestion.sw.victim_marking_enable){

			i = port_count / 8;
			// Victim Mask is a 256 bitfield, stored most significant bit first.
			// (e.g. Port 0 = Victim_Mask[31] & 0x1)
			// Sets victim_marking_enable on all valid ports of the switch.
			setting.Victim_Mask[31 - i] = (1<<(port_count % 8)) - 1; 
			while (i > 0) setting.Victim_Mask[31 - --i] = 0xff;
			if (!nodep->switchInfo.u2.s.EnhancedPort0) setting.Victim_Mask[31] &= 0xfe;
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
	status = SM_Set_SwitchCongestionSetting_LR(fd_topology, 0, sm_lid, dlid, &setting, sm_config.mkey);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to set switch Congestion Setting for NodeGUID "FMT_U64" [%s]; rc: %d",
			nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
		return status;
	}

	if (!sm_config.congestion.enable)
		return status;

	nodep->swCongestionSetting = setting;

	if (vs_pool_alloc(&sm_pool, sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT) * port_count, 
			(void *)&swPortSet) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate memory for switch port congestion setting.");
		return VSTATUS_NOMEM;
	}

	status = SM_Get_SwitchPortCongestionSetting_LR(fd_topology, ((uint32_t)port_count)<<24, sm_lid, dlid, swPortSet);
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
