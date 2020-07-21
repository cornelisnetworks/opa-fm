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

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_attribute.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the SM routines to fetch and manipulate the         //
//    SM attributes found in chapters 13 and 14.                             //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    None								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_status.h"
#include "ib_mad.h"
#include "ib_macros.h"
#include "ib_sa.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "iba/stl_sm_priv.h"


/**
	Implements sending and parsing both aggregate Set() and Get() operations.

	It's actually easier to handle the Set vs. Get stuff common
	than it is to make the 'how do you pack and unpack a request' stuff common.
*/
static Status_t
SM_Aggregate_impl(
	SmMaiHandle_t *fd, STL_AGGREGATE * inStart, STL_AGGREGATE * inEnd, size_t lastSegReqLen,
	SmpAddr_t * addr, uint8 method, uint64_t mkey, STL_AGGREGATE ** lastSeg, uint32_t * madStatus);

static boolean sm_valid_port_state(const STL_PORT_STATES * ps)
{
	return ((ps->s.PortPhysicalState > 0 && ps->s.PortPhysicalState <= STL_PORT_PHYS_TEST) &&
		(ps->s.PortState > 0 && ps->s.PortState <= IB_PORT_ACTIVE));
}

Status_t
SM_Get_NodeDesc(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_NODE_DESCRIPTION *ndp) {
   Status_t status;
   uint32_t bufferLength = sizeof(STL_NODE_DESCRIPTION); 
   uint8_t buffer[bufferLength];

   INCREMENT_COUNTER(smCounterGetNodeDescription);

   status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION, amod, addr, buffer, &bufferLength);
   if (status == VSTATUS_OK) {
      BSWAP_STL_NODE_DESCRIPTION((STL_NODE_DESCRIPTION*)buffer);
      memcpy(ndp, buffer, sizeof(STL_NODE_DESCRIPTION));

   }

   return status;
}

Status_t
SM_Get_NodeInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_NODE_INFO *nip) {
   Status_t status; 
   uint32_t bufferLength = sizeof(STL_NODE_INFO); 
   uint8_t buffer[bufferLength]; 
   
   INCREMENT_COUNTER(smCounterGetNodeInfo); 
   
   status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_NODE_INFO, amod, addr, buffer, &bufferLength);
   if (status == VSTATUS_OK) {
      BSWAP_STL_NODE_INFO((STL_NODE_INFO *)buffer); 

      // Fail out if node reports no ports.
      if(!((STL_NODE_INFO *)buffer)->NumPorts)
          return VSTATUS_REJECT;

      // Fail out if node reports no partition cap
      if (!((STL_NODE_INFO *)buffer)->PartitionCap)
          return VSTATUS_REJECT;

      memcpy(nip, buffer, sizeof(STL_NODE_INFO)); 
      
      if (sm_config.sm_debug_lid_assign) {
         if ((nip->NodeGUID & 0xffffffff00000000ULL) == 0) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Get(STL_NodeInfo) has Node Guid of "FMT_U64, 
                                   nip->NodeGUID);
         }
         if ((nip->PortGUID & 0xffffffff00000000ULL) == 0) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Get(STL_NodeInfo) has Port Guid of "FMT_U64, 
                                   nip->PortGUID);
         }
      }
      
      // Workaround for PR 113605
      if ((nip->NodeType == NI_TYPE_CA) && 
          (((nip->NodeGUID & 0xffffffff00000000ULL) == 0) || 
           ((nip->PortGUID & 0xffffffff00000000ULL) == 0))) {
         status = VSTATUS_BAD;
      }
   }

   return (status); 
}

Status_t
SM_Get_PortInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_INFO *pip) {
   Status_t status; 
   uint32_t bufferLength = sizeof(STL_PORT_INFO); 
   uint8_t buffer[bufferLength]; 
   
   INCREMENT_COUNTER(smCounterGetPortInfo);
   
   status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_INFO, amod, addr, buffer, &bufferLength);
   if (status == VSTATUS_OK) {
      (void)BSWAP_STL_PORT_INFO((STL_PORT_INFO *)buffer); 
      memcpy(pip, buffer, sizeof(STL_PORT_INFO));
      if (!sm_valid_port_state(&pip->PortStates)) return VSTATUS_BAD;
   }


   return (status); 
}

Status_t
SM_Get_PortStateInfo_Dispatch(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, Node_t *nodep,
	sm_dispatch_t *disp, cntxt_callback_t callback, void *cb_data)
{
	Status_t status;

	INCREMENT_COUNTER(smCounterGetPortStateInfo);

	status = sm_get_stl_attribute_async_dispatch(fd,
		STL_MCLASS_ATTRIB_ID_PORT_STATE_INFO, amod, addr,
		NULL, NULL, nodep, disp, callback, cb_data);
	
	return (status);
}

Status_t
SM_Get_PortStateInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_STATE_INFO *psip) {
	Status_t status;
	uint8_t portCount = (amod>>24 & 0xff);
	uint32_t bufferLength = sizeof(STL_PORT_STATE_INFO) * portCount;
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetPortStateInfo);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_STATE_INFO, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		(void)BSWAP_STL_PORT_STATE_INFO((STL_PORT_STATE_INFO *)buffer, portCount);
		memcpy(psip, buffer, sizeof(STL_PORT_STATE_INFO) * portCount); 

		uint8_t i;
		for (i = 0; i < portCount; ++i) {
			if (!sm_valid_port_state(&psip[i].PortStates)) return VSTATUS_BAD;
		}
	}

	return (status);
}

Status_t
SM_Get_SwitchInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_INFO *swp) {
	Status_t status;
	uint32_t bufferLength = sizeof(STL_SWITCH_INFO); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetSwitchInfo);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SWITCH_INFO, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		(void)BSWAP_STL_SWITCH_INFO((STL_SWITCH_INFO *)buffer);
		memcpy(swp, buffer, sizeof(STL_SWITCH_INFO));
	}

	return(status);
}

Status_t
SM_Get_SMInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SM_INFO *smip) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SM_INFO); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetSmInfo);

	/* Include our SMInfo in our request. The Master SM needs it. */
    memcpy(buffer, &sm_smInfo, sizeof(STL_SM_INFO));
	((STL_SM_INFO*)buffer)->SM_Key = 0;
    (void)BSWAP_STL_SM_INFO((STL_SM_INFO *)buffer);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SM_INFO, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SM_INFO((STL_SM_INFO *)buffer);
        memcpy(smip, buffer, sizeof(STL_SM_INFO));
	}

	return(status);
}

Status_t
SM_Get_VLArbitration(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_VLARB_TABLE *vlp) {
    Status_t status;
    uint32_t bufferLength = 0; 
    uint8_t buffer[STL_MAD_PAYLOAD_SIZE];
    uint8_t blkCount = (uint8_t)(amod >> 24);
    uint8_t section = (uint8_t)(amod >> 16);

    INCREMENT_COUNTER(smCounterGetVLArbitrationTable);
    bufferLength = sizeof(STL_VLARB_TABLE)*blkCount;

    status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_VL_ARBITRATION, amod, addr, buffer, &bufferLength);

    if (status == VSTATUS_OK) {
        uint8_t i;
        for (i = 0; i < blkCount; ++i) {
            BSWAP_STL_VLARB_TABLE(&((STL_VLARB_TABLE*)buffer)[i], section);
        }
        memcpy(vlp, buffer, blkCount*sizeof(STL_VLARB_TABLE));
    }

    return(status);
}



Status_t
SM_Get_SLSCMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SLSCMAP *slscp) {
	Status_t status;
	uint32_t bufferLength = sizeof(STL_SLSCMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetSL2SCMappingTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SLSCMAP((STL_SLSCMAP *)buffer);
        memcpy(slscp, buffer, sizeof(STL_SLSCMAP));
	}

	return(status);
}

Status_t
SM_Get_SCVLrMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCVLMAP *scvlp) {
	Status_t status;
	uint32_t bufferLength = sizeof(STL_SCVLMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetSC2VLrMappingTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SCVLMAP((STL_SCVLMAP *)buffer);
        memcpy(scvlp, buffer, sizeof(STL_SCVLMAP));
        ZERO_RSVD_STL_SCVLMAP(scvlp);
	}

	return(status);
}

Status_t
SM_Get_SCSLMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCSLMAP *scslp) {
	Status_t status;
	uint32_t bufferLength = sizeof(STL_SCSLMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetSC2SLMappingTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SCSLMAP((STL_SCSLMAP *)buffer);
        memcpy(scslp, buffer, sizeof(STL_SCSLMAP));
	}

	return(status);
}

Status_t
SM_Get_SCVLtMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCVLMAP *scvlp) {
	Status_t status;
	uint32_t bufferLength = 0;
	uint8_t buffer[STL_MAD_PAYLOAD_SIZE] = { 0 };

	uint8_t blkCount = (uint8_t)( amod >> 24);

	bufferLength = blkCount * sizeof(STL_SCVLMAP);
	INCREMENT_COUNTER(smCounterGetSC2VLtMappingTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		size_t cpyCount = MIN(blkCount*sizeof(STL_SCVLMAP), bufferLength);
		memcpy(scvlp, buffer, cpyCount);

		size_t end = cpyCount/sizeof(STL_SCVLMAP);
		uint8_t i;
		for (i = 0; i < end; ++i) {
			(void)BSWAP_STL_SCVLMAP(&scvlp[i]);
		}
	}

	return(status);
}

Status_t
SM_Get_SCVLntMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCVLMAP *scvlp) {
	Status_t status;
    uint32_t bufferLength = 0; 
	uint8_t buffer[STL_MAD_PAYLOAD_SIZE] = { 0 };

	uint8_t blkCount = (uint8_t)( amod >> 24);

	bufferLength = blkCount * sizeof(STL_SCVLMAP);
	INCREMENT_COUNTER(smCounterGetSC2VLntMappingTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		// This isn't really a good check 'cause neither arg is really the amount of mem available starting at scvlp
		size_t cpyCount = MIN(blkCount*sizeof(STL_SCVLMAP), bufferLength);
		memcpy(scvlp, buffer, cpyCount);

		size_t end = cpyCount/sizeof(STL_SCVLMAP);
		uint8_t i;
		for (i = 0; i < end; ++i) {
			(void)BSWAP_STL_SCVLMAP(&scvlp[i]);
		}
	}

	return(status);
}

Status_t
SM_Get_PKeyTable(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PARTITION_TABLE *pkp) {
	Status_t status;
	const uint8_t blkCnt = (amod>>24) & 0xff;
	uint32_t bufferLength = sizeof(STL_PARTITION_TABLE) * blkCnt;
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetPKeyTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PART_TABLE, amod, addr, buffer, &bufferLength);

	if (status == VSTATUS_OK) {
		uint8_t i;

		for (i = 0; i < blkCnt; ++i)
			(void)BSWAP_STL_PARTITION_TABLE( &(((STL_PARTITION_TABLE *)buffer)[i]) );

		memcpy(pkp, buffer, sizeof(STL_PARTITION_TABLE) * blkCnt);
	}

	return(status);
}

Status_t
SM_Get_BufferControlTable(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_BUFFER_CONTROL_TABLE pbct[])
{
	Status_t status;
	uint8_t portCount = (amod>>24 & 0xff);

	uint32_t bufferLength = STL_BFRCTRLTAB_PAD_SIZE * portCount;
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetBufferControlTable);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		uint32_t i;
		uint8_t * data = buffer;
		STL_BUFFER_CONTROL_TABLE *table = (STL_BUFFER_CONTROL_TABLE *)(data);
		for (i = 0; i < portCount; i++) {
			(void)BSWAP_STL_BUFFER_CONTROL_TABLE(table);
			table->Reserved = 0;
			memcpy(&pbct[i], table, sizeof(STL_BUFFER_CONTROL_TABLE));
			data += STL_BFRCTRLTAB_PAD_SIZE;
			table = (STL_BUFFER_CONTROL_TABLE *)(data);
		}
	}

	return (status);
}

Status_t
SM_Get_LedInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_LED_INFO *li)
{
	Status_t status;
	uint32_t bufferLength = sizeof(STL_LED_INFO);
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterGetLedInfo);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_LED_INFO, amod, addr, buffer, &bufferLength);

	if (status == VSTATUS_OK) {
		(void)BSWAP_STL_LED_INFO((STL_LED_INFO *)buffer);
		memcpy(li, buffer, bufferLength);
	}

	return (status);
}

Status_t
SM_Set_LedInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_LED_INFO *li, uint64_t mkey)
{
	Status_t status;
	uint32_t bufferLength = sizeof(STL_LED_INFO);
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetLedInfo);

	memcpy(buffer, li, sizeof(STL_LED_INFO));
	(void)BSWAP_STL_LED_INFO((STL_LED_INFO *)buffer);

	status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_LED_INFO, amod, addr, buffer, &bufferLength, mkey);

	if (status == VSTATUS_OK) {
		(void)BSWAP_STL_LED_INFO((STL_LED_INFO *)buffer);
		memcpy(li, buffer, bufferLength);
	}

	return (status);
}

// -------------------------------------------------------------------------- //

Status_t
SM_Set_PortInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t * addr, STL_PORT_INFO *pip, uint64_t mkey, uint32* madStatus) {
   Status_t status;
   uint32_t bufferLength=sizeof(STL_PORT_INFO); 
   uint8_t buffer[bufferLength];
   
   INCREMENT_COUNTER(smCounterSetPortInfo); 
   
   memcpy(buffer, pip, sizeof(STL_PORT_INFO)); 
   ZERO_RSVD_STL_PORT_INFO((STL_PORT_INFO *) buffer);
   (void)BSWAP_STL_PORT_INFO((STL_PORT_INFO *)buffer); 
   
   if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_PORTINFO) {
      status = VSTATUS_OK;
   } else {
      status = sm_set_stl_attribute_mad_status(fd, STL_MCLASS_ATTRIB_ID_PORT_INFO, amod, addr, buffer, &bufferLength, mkey, madStatus);
   }

   if (status == VSTATUS_OK || (status == VSTATUS_BAD && madStatus != NULL && *madStatus != 0)) {
      (void)BSWAP_STL_PORT_INFO((STL_PORT_INFO *)buffer); 
      memcpy(pip, buffer, bufferLength);
   }
   
   return (status); 
}
Status_t
SM_Set_PortInfo_Dispatch(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_INFO *pip,
	uint64_t mkey, Node_t *nodep, sm_dispatch_t *disp, cntxt_callback_t callback, void *cb_data)
{
	Status_t status;
	uint32_t bufferLength = sizeof(STL_PORT_INFO);
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetPortInfo);

	memcpy(buffer, pip, sizeof(STL_PORT_INFO));
	ZERO_RSVD_STL_PORT_INFO((STL_PORT_INFO *)buffer);
	(void)BSWAP_STL_PORT_INFO((STL_PORT_INFO *)buffer);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_PORTINFO) {
		status = VSTATUS_OK;
		if (callback) {
			// Build Fake MAI for Callback
			Mai_t mad = { 0 };
			mad.base.amod = amod;
			mad.base.mclass = (addr->path ? MCLASS_SM_DIRECTED_ROUTE : MCLASS_SM_LID_ROUTED);
			memcpy(stl_mai_get_smp_data(&mad), buffer, bufferLength);
			mad.datasize = bufferLength + stl_mai_get_smp_data_offset(&mad);
			// Call Callback
			callback(NULL, status, cb_data, &mad);
		}
	} else {
		status = sm_set_stl_attribute_async_dispatch(fd,
			STL_MCLASS_ATTRIB_ID_PORT_INFO, amod, addr,
			buffer, &bufferLength, mkey, nodep, disp,
			callback, cb_data);
	}

	return (status);
}

Status_t
SM_Set_PortStateInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_STATE_INFO *psip, uint64_t mkey) {
	Status_t status;
	uint32_t portCount = (0xff & (amod >> 24));
	uint32_t bufferLength = portCount * sizeof(STL_PORT_STATE_INFO);
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetPortStateInfo);

	memcpy(buffer, psip, bufferLength);
	(void)BSWAP_STL_PORT_STATE_INFO((STL_PORT_STATE_INFO *)buffer, portCount);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_PORTSTATEINFO) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_STATE_INFO, amod, addr, buffer, &bufferLength, mkey);
	}

	if(status == VSTATUS_OK) {
		(void)BSWAP_STL_PORT_STATE_INFO((STL_PORT_STATE_INFO *)buffer, portCount);
		memcpy(psip, buffer, sizeof(STL_PORT_STATE_INFO) * portCount); 

		uint8_t i;
		for (i = 0; i < portCount; ++i) {
			if (!sm_valid_port_state(&psip[i].PortStates)) return VSTATUS_BAD;
		}
	}

	return (status);
}

Status_t
SM_Set_SwitchInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_INFO *swp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SWITCH_INFO); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSwitchInfo);

    memcpy(buffer, swp, sizeof(STL_SWITCH_INFO));
	ZERO_RSVD_STL_SWITCH_INFO((STL_SWITCH_INFO*)buffer);
    (void)BSWAP_STL_SWITCH_INFO((STL_SWITCH_INFO *)buffer);

    if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_SWITCHINFO) {
        status = VSTATUS_OK;
    } else {
        status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SWITCH_INFO, amod, addr, buffer, &bufferLength, mkey);
    }
	if (status == VSTATUS_OK) {
		(void)BSWAP_STL_SWITCH_INFO((STL_SWITCH_INFO *)buffer);
		memcpy(swp, buffer, sizeof(STL_SWITCH_INFO));
	}

	return(status);
}

Status_t
SM_Set_SMInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SM_INFO *smip, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SM_INFO); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSmInfo);

    memcpy(buffer, smip, sizeof(STL_SM_INFO));
    (void)BSWAP_STL_SM_INFO((STL_SM_INFO *)buffer);

    if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_SMINFO) {
        status = VSTATUS_OK;
    } else {
        status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SM_INFO, amod, addr, buffer, &bufferLength, mkey);
    }
	if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SM_INFO((STL_SM_INFO *)buffer);
        memcpy(smip, buffer, sizeof(STL_SM_INFO));
	}

	return(status);
}


/**
	Does not implement multiblock requests.

	@param vlpSize the amount of data to copy back from the SMA into @c vlp.
*/
Status_t
SM_Set_VLArbitration(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_VLARB_TABLE *vlp, size_t vlpSize, uint64_t mkey) {
	Status_t status;
	uint32_t bufferLength = sizeof(STL_VLARB_TABLE); 
	uint8_t buffer[STL_MAD_PAYLOAD_SIZE] = { 0 };	// no need to zero, fills buffer
	uint8_t sec = (uint8_t)(amod >>16);
	uint8_t blkCount = (uint8_t)(amod >> 24);

	if (blkCount > 1)
		return VSTATUS_BAD;

	bufferLength = sizeof(STL_VLARB_TABLE) * blkCount;
	INCREMENT_COUNTER(smCounterSetVLArbitrationTable);

	memcpy(buffer, vlp, vlpSize);
	(void)BSWAP_STL_VLARB_TABLE((STL_VLARB_TABLE *)buffer, sec);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_VLARB) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_VL_ARBITRATION, amod, addr, buffer, &bufferLength, mkey);
	}
	if (status == VSTATUS_OK) {
		(void)BSWAP_STL_VLARB_TABLE((STL_VLARB_TABLE *)buffer, sec);
		memcpy(vlp, buffer, vlpSize);
	}

	return(status);
}



Status_t
SM_Set_SLSCMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SLSCMAP *slscp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SLSCMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSL2SCMappingTable);

    memcpy(buffer, slscp, sizeof(STL_SLSCMAP));
    ZERO_RSVD_STL_SLSCMAP((STL_SLSCMAP*)buffer);
    (void)BSWAP_STL_SLSCMAP((STL_SLSCMAP *)buffer);

    if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_MAPS) {
        status = VSTATUS_OK;
    } else {
        status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, amod, addr, buffer, &bufferLength, mkey);
    }
    if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SLSCMAP((STL_SLSCMAP *)buffer);
        memcpy(slscp, buffer, sizeof(STL_SLSCMAP));
    }

	return(status);
}

Status_t
SM_Set_SCVLrMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCVLMAP *scvlp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SCVLMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSC2VLrMappingTable);

    ZERO_RSVD_STL_SCVLMAP(scvlp);
    memcpy(buffer, scvlp, sizeof(STL_SCVLMAP));
    (void)BSWAP_STL_SCVLMAP((STL_SCVLMAP *)buffer);

    status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE, amod, addr, buffer, &bufferLength, mkey);

	return(status);
}

Status_t
SM_Set_SCSLMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCSLMAP *scslp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SCSLMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSC2SLMappingTable);

    memcpy(buffer, scslp, sizeof(STL_SCSLMAP));
    ZERO_RSVD_STL_SCSLMAP((STL_SCSLMAP*)buffer);
    (void)BSWAP_STL_SCSLMAP((STL_SCSLMAP *)buffer);

    if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_MAPS) {
        status = VSTATUS_OK;
    } else {
        status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, amod, addr, buffer, &bufferLength, mkey);
    }
    if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SCSLMAP((STL_SCSLMAP *)buffer);
        memcpy(scslp, buffer, sizeof(STL_SCSLMAP));
    }

	return(status);
}

Status_t
SM_Set_SCVLtMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCVLMAP *scvlp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SCVLMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSC2VLtMappingTable);

    memcpy(buffer, scvlp, sizeof(STL_SCVLMAP));
    ZERO_RSVD_STL_SCVLMAP((STL_SCVLMAP*)buffer);
    (void)BSWAP_STL_SCVLMAP((STL_SCVLMAP *)buffer);

    status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE, amod, addr, buffer, &bufferLength, mkey);
    if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SCVLMAP((STL_SCVLMAP *)buffer);
        memcpy(scvlp, buffer, sizeof(STL_SCVLMAP));
    }

	return(status);
}

Status_t
SM_Set_SCVLntMap(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCVLMAP *scvlp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_SCVLMAP); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetSC2VLntMappingTable);

    memcpy(buffer, scvlp, sizeof(STL_SCVLMAP));
    ZERO_RSVD_STL_SCVLMAP((STL_SCVLMAP*)buffer);
    (void)BSWAP_STL_SCVLMAP((STL_SCVLMAP *)buffer);

    status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE, amod, addr, buffer, &bufferLength, mkey);
    if (status == VSTATUS_OK) {
        (void)BSWAP_STL_SCVLMAP((STL_SCVLMAP *)buffer);
        memcpy(scvlp, buffer, sizeof(STL_SCVLMAP));
    }

	return(status);
}

Status_t
SM_Set_SCSC(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCSCMAP *scscp, uint64_t mkey) {
	Status_t status;
	uint8_t numBlocks = amod >> 24;
	uint32_t bufferLength = sizeof(STL_SCSCMAP)*numBlocks; 
	uint8_t buffer[bufferLength];
	int i;

	INCREMENT_COUNTER(smCounterSetSC2SCMappingTable);

	for (i=0; i<numBlocks; i++) {
		memcpy((void*)&((STL_SCSCMAP*)buffer)[i], scscp, sizeof(STL_SCSCMAP));
		(void)BSWAP_STL_SCSCMAP(&((STL_SCSCMAP*)buffer)[i]);
	}

	status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_SC_MAPPING_TABLE, amod, addr, buffer, &bufferLength, mkey);

	return(status);
}

Status_t
SM_Set_SCSCMultiSet(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SCSC_MULTISET *scscp, uint64_t mkey) {
	Status_t status;
	uint8_t numBlocks = amod >> 24;
    uint32_t bufferLength = sizeof(STL_SCSC_MULTISET)*numBlocks; 
	uint8_t buffer[bufferLength];
	int i;

	INCREMENT_COUNTER(smCounterSetSC2SCMultiSet);

    memcpy(buffer, scscp, sizeof(STL_SCSC_MULTISET) * numBlocks);
	for (i=0; i<numBlocks; i++) {
    	(void)BSWAP_STL_SCSC_MULTISET(&((STL_SCSC_MULTISET*)buffer)[i]);
	}

	status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SC_SC_MULTI_SET, amod, addr, buffer, &bufferLength, mkey);

	return(status);
}

Status_t
SM_Set_LFT(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_LINEAR_FORWARDING_TABLE *lftp, uint64_t mkey) {
	Status_t status;
    uint32_t bufferLength = sizeof(STL_LINEAR_FORWARDING_TABLE); 
	uint8_t buffer[bufferLength];

	INCREMENT_COUNTER(smCounterSetLft);

    memcpy(buffer, lftp, sizeof(STL_LINEAR_FORWARDING_TABLE));
    BSWAP_STL_LINEAR_FORWARDING_TABLE((STL_LINEAR_FORWARDING_TABLE *)buffer);

    if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_LFT)
        status = VSTATUS_OK;
    else
        status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE, amod, addr, buffer, &bufferLength, mkey);

    if (status == VSTATUS_OK) {
        BSWAP_STL_LINEAR_FORWARDING_TABLE((STL_LINEAR_FORWARDING_TABLE *)buffer);
        memcpy(lftp, buffer, sizeof(STL_LINEAR_FORWARDING_TABLE));
    }

	return(status);
}

Status_t
SM_Set_LFT_Dispatch(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr,
					   STL_LINEAR_FORWARDING_TABLE *lftp, uint16_t count, 
					   uint64_t mkey, Node_t *nodep, sm_dispatch_t *disp) {
	Status_t status;
	uint32_t bufferLength = sizeof(STL_LINEAR_FORWARDING_TABLE) * count;
	uint8_t buffer[STL_MAD_PAYLOAD_SIZE];	
	uint16_t i;

	if (bufferLength >= STL_MAD_PAYLOAD_SIZE) {
		IB_LOG_ERROR_FMT(__func__,
			"Count exceeds maximum MAD payload.");
		return VSTATUS_TOO_LARGE;
	}

	INCREMENT_COUNTER(smCounterSetLft);

	memcpy(buffer, lftp, bufferLength);
	for (i = 0; i < count; ++i)
		BSWAP_STL_LINEAR_FORWARDING_TABLE( &((STL_LINEAR_FORWARDING_TABLE *)buffer)[i] );

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_LFT) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute_async_dispatch(fd,
			STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE, amod, addr,
			buffer, &bufferLength, mkey, nodep, disp, NULL, NULL);
	}
	return(status);
}


Status_t
SM_Set_MFT_Dispatch(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_MULTICAST_FORWARDING_TABLE *mftp,
	uint64_t mkey, Node_t *nodep, sm_dispatch_t *disp)
{
	Status_t status;
	uint32_t bufferLength = sizeof(STL_MULTICAST_FORWARDING_TABLE);
	uint8_t buffer[STL_MAD_PAYLOAD_SIZE];

	INCREMENT_COUNTER(smCounterSetMft);

	memcpy(buffer, mftp, sizeof(STL_MULTICAST_FORWARDING_TABLE));
	BSWAP_STL_MULTICAST_FORWARDING_TABLE((STL_MULTICAST_FORWARDING_TABLE *)buffer);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_LFT) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute_async_dispatch(fd,
		STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE, amod, addr, buffer, &bufferLength, mkey, nodep, disp, NULL, NULL);
	}
	return (status);
}

Status_t
SM_Set_PKeyTable(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PARTITION_TABLE *pkp, uint64_t mkey) {
	Status_t status;
	const uint8_t blkCnt = (amod>>24) & 0xff;
    uint32_t bufferLength = sizeof(STL_PARTITION_TABLE) * blkCnt;
	uint8_t buffer[bufferLength];
	uint8_t i;

	INCREMENT_COUNTER(smCounterSetPKeyTable);

	memcpy(buffer, pkp, sizeof(STL_PARTITION_TABLE) * blkCnt);
	for (i = 0; i < blkCnt; ++i)
		(void)BSWAP_STL_PARTITION_TABLE( &(((STL_PARTITION_TABLE *)buffer)[i]) );

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_PKEY) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PART_TABLE,
			amod, addr, buffer, &bufferLength, mkey);
	}

	if (status == VSTATUS_OK) {
		for (i = 0; i < blkCnt; ++i)
			(void)BSWAP_STL_PARTITION_TABLE( &(((STL_PARTITION_TABLE *)buffer)[i]) );
		memcpy(pkp, buffer, sizeof(STL_PARTITION_TABLE) * blkCnt);
	}

	return(status);
}
Status_t
SM_Set_PKeyTable_Dispatch(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PARTITION_TABLE *pkp,
	uint64_t mkey, Node_t *nodep, sm_dispatch_t *disp, cntxt_callback_t callback, void *cb_data)
{

	Status_t status;
	const uint8_t blkCnt = (amod>>24) & 0xff;
    uint32_t bufferLength = sizeof(STL_PARTITION_TABLE) * blkCnt;
	uint8_t buffer[bufferLength];
	uint8_t i;

	INCREMENT_COUNTER(smCounterSetPKeyTable);

	memcpy(buffer, pkp, sizeof(STL_PARTITION_TABLE) * blkCnt);
	for (i = 0; i < blkCnt; ++i)
		(void)BSWAP_STL_PARTITION_TABLE( &(((STL_PARTITION_TABLE *)buffer)[i]) );

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_PKEY) {
		status = VSTATUS_OK;
		// Build Fake MAI for Callback
		Mai_t mad = { 0 };
		mad.base.amod = amod;
		mad.base.mclass = (addr->path ? MCLASS_SM_DIRECTED_ROUTE : MCLASS_SM_LID_ROUTED);
		memcpy(stl_mai_get_smp_data(&mad), buffer, bufferLength);
		mad.datasize = bufferLength + stl_mai_get_smp_data_offset(&mad);
		// Call Callback
		if (callback) callback(NULL, status, cb_data, &mad);
	} else {
		status = sm_set_stl_attribute_async_dispatch(fd,
			STL_MCLASS_ATTRIB_ID_PART_TABLE, amod, addr,
			buffer, &bufferLength, mkey, nodep, disp,
			callback, cb_data);
	}
	return(status);
}

// "blocks" specifies the # of entries in pgp[]. Makes no attempt to
// verify that "blocks" agrees with the value of "N" in amod.
Status_t
SM_Get_PortGroup(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_GROUP_TABLE pgp[], uint8_t blocks)
{
	uint8_t i;
	Status_t status;
	uint32_t length = blocks * sizeof(STL_PORT_GROUP_TABLE);

	assert(blocks >0 && blocks <= STL_NUM_PGTABLE_BLOCKS_PER_DRSMP);
	INCREMENT_COUNTER(smCounterGetPG);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_GROUP_TABLE, amod, addr, (uint8_t*)pgp, &length);

	if (status == VSTATUS_OK) {
		for(i = 0; i < blocks; i++) {
			BSWAP_STL_PORT_GROUP_TABLE(&pgp[i]);
		}
	}

	return (status);
}

// "blocks" specifies the # of entries in pgp[]. Makes no attempt to
// verify that "blocks" agrees with the value of "N" in amod.
Status_t
SM_Set_PortGroup(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr,
	STL_PORT_GROUP_TABLE *pgp, uint8_t blocks, uint64_t mkey) {
	Status_t	status;
	uint8_t		i;
	uint32_t	bufferLength = blocks * sizeof(STL_PORT_GROUP_TABLE);
	uint8_t		buffer[bufferLength];

	assert(blocks >0 && blocks <= STL_NUM_PGTABLE_BLOCKS_PER_DRSMP);

	memcpy(buffer, pgp, bufferLength);
	INCREMENT_COUNTER(smCounterSetPG);

	for (i=0;i<blocks;i++) {
		BSWAP_STL_PORT_GROUP_TABLE(&((STL_PORT_GROUP_TABLE*)buffer)[i]);
	}

	status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_GROUP_TABLE, 
		amod, addr, buffer, &bufferLength, mkey);

	if (status == VSTATUS_OK) {
		for (i=0;i<blocks;i++) {
			BSWAP_STL_PORT_GROUP_TABLE(&((STL_PORT_GROUP_TABLE*)buffer)[i]);
		}
		//@todo: this should compare request with response
		//@todo: if we only copy the response back on success, and success is defined as the request == the response, do we need to copy the response back?
	}

	return(status);
}

// "blocks" specifies the # of entries in pgp[]. Makes no attempt to
// verify that "blocks" agrees with the value of "N" in amod. pgp must fit
// in a single multi-block MAD.
Status_t
SM_Get_PortGroupFwdTable(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr,
	STL_PORT_GROUP_FORWARDING_TABLE *pgftp, uint8_t blocks) {
	Status_t	status;
	uint8_t		i;
	uint32_t	length = blocks * sizeof(STL_PORT_GROUP_FORWARDING_TABLE);

	assert(blocks >0 && blocks <= STL_NUM_PGFT_BLOCKS_PER_DRSMP);

	INCREMENT_COUNTER(smCounterGetPGft);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_GROUP_FWD_TABLE, 
		amod, addr, (uint8_t*)pgftp, &length);

	if (status == VSTATUS_OK) {
		for (i=0;i<blocks;i++) {
			BSWAP_STL_PORT_GROUP_FORWARDING_TABLE(&pgftp[i]);
		}
	}

	return(status);
}

// "blocks" specifies the # of entries in pgftp[]. Makes no attempt to
// verify that "blocks" agrees with the value of "N" in amod. pgftp must fit
// in a single multi-block MAD.
Status_t
SM_Set_PortGroupFwdTable(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_PORT_GROUP_FORWARDING_TABLE *pgftp,
	uint8_t blocks, uint64_t mkey)
{
	uint8_t		i;
	Status_t	status;
	uint32_t	length = blocks * sizeof(STL_PORT_GROUP_FORWARDING_TABLE);
	uint8_t		buffer[length];
	memcpy(buffer, pgftp, length);

	assert(blocks >0 && blocks <= STL_NUM_PGFT_BLOCKS_PER_DRSMP);

	INCREMENT_COUNTER(smCounterSetPGft);

	for (i=0;i<blocks;i++) {
		BSWAP_STL_PORT_GROUP_FORWARDING_TABLE(&((STL_PORT_GROUP_FORWARDING_TABLE*)buffer)[i]);
	}

	status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_PORT_GROUP_FWD_TABLE,
		amod, addr, buffer, &length, mkey);

	if (status == VSTATUS_OK) {
		for (i=0;i<blocks;i++) {
			BSWAP_STL_PORT_GROUP_FORWARDING_TABLE(&((STL_PORT_GROUP_FORWARDING_TABLE*)buffer)[i]);
			//@todo: compare request with response and return non-OK if not equal
		}
		//@todo: need to copy values back if everything's OK?
	}

	return(status);
}

Status_t
SM_Get_CongestionInfo(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_CONGESTION_INFO * congestionInfo) {
	uint32_t	bufferLength = sizeof(STL_CONGESTION_INFO);
	uint8_t		buffer[bufferLength];
	Status_t	status;

	INCREMENT_COUNTER(smCounterGetCongestionInfo);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_CONGESTION_INFO, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		BSWAP_STL_CONGESTION_INFO((STL_CONGESTION_INFO*)buffer);
		memcpy(congestionInfo, buffer, bufferLength);
	}

	return(status);
}

Status_t
SM_Get_HfiCongestionSetting(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_SETTING *hfics) {
	uint32_t	bufferLength = sizeof(STL_HFI_CONGESTION_SETTING);
	uint8_t		buffer[bufferLength];
	Status_t status;

	INCREMENT_COUNTER(smCounterGetHfiCongestionSetting);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_SETTING, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		BSWAP_STL_HFI_CONGESTION_SETTING((STL_HFI_CONGESTION_SETTING*)buffer);
		memcpy(hfics, buffer, bufferLength);
	}

	return(status);
}

Status_t SM_Set_HfiCongestionSetting(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_SETTING *hfics,
	uint64_t mkey)
{
	uint32_t bufferLength = sizeof(STL_HFI_CONGESTION_SETTING);
	uint8_t  buffer[bufferLength];
	Status_t status;

	INCREMENT_COUNTER(smCounterSetHfiCongestionSetting);

	memcpy(buffer, hfics, sizeof(STL_HFI_CONGESTION_SETTING));
	BSWAP_STL_HFI_CONGESTION_SETTING((STL_HFI_CONGESTION_SETTING *)buffer);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_CONG) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_SETTING, amod, addr, buffer, &bufferLength, mkey);
	}
	if (status == VSTATUS_OK) {
		BSWAP_STL_HFI_CONGESTION_SETTING((STL_HFI_CONGESTION_SETTING *)buffer);
		memcpy(hfics, buffer, sizeof(STL_HFI_CONGESTION_SETTING));
	}

	return (status);
}

Status_t
SM_Get_HfiCongestionControl(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_HFI_CONGESTION_CONTROL_TABLE *hficct) {
	const uint8_t numBlocks = (amod>>24) & 0xff;
	uint32_t	bufferLength = getCongTableSize(numBlocks);
	uint8_t		buffer[bufferLength];
	Status_t	status;

	INCREMENT_COUNTER(smCounterGetHfiCongestionControl);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_CONTROL_TABLE, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE((STL_HFI_CONGESTION_CONTROL_TABLE*)buffer, numBlocks);
		memcpy(hficct, buffer, bufferLength);
	}

	return(status);
}

Status_t
SM_Set_HfiCongestionControl(SmMaiHandle_t *fd, uint16 CCTI_Limit, const uint8_t numBlocks, uint32_t amod,
	SmpAddr_t *addr, STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK *hficct, uint64_t mkey)
{
	Status_t status = VSTATUS_OK;
	uint32_t bufferLength = MIN(STL_MAD_PAYLOAD_SIZE, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE) + ((numBlocks-1) * sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK)));
	uint8_t  buffer[bufferLength];

	STL_HFI_CONGESTION_CONTROL_TABLE *ptr = NULL;

	memset(buffer, 0, bufferLength);
	ptr = (STL_HFI_CONGESTION_CONTROL_TABLE *)buffer;
	INCREMENT_COUNTER(smCounterSetHfiCongestionControl);
	ptr->CCTI_Limit = CCTI_Limit;
	memcpy(ptr->CCT_Block_List, hficct, sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_BLOCK) * numBlocks);
	BSWAP_STL_HFI_CONGESTION_CONTROL_TABLE(ptr, numBlocks);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_CONG) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_CONTROL_TABLE, amod, addr, buffer, &bufferLength, mkey);
	}

	return (status);
}

Status_t
SM_Get_SwitchCongestionSetting(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_CONGESTION_SETTING *swcs) {
	uint32_t	bufferLength = sizeof(STL_SWITCH_CONGESTION_SETTING);
	uint8_t		buffer[bufferLength];
	Status_t status;

	INCREMENT_COUNTER(smCounterGetSwitchCongestionSetting);

	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SWITCH_CONGESTION_SETTING, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		BSWAP_STL_SWITCH_CONGESTION_SETTING((STL_SWITCH_CONGESTION_SETTING*)buffer);
		memcpy(swcs, buffer, bufferLength);
	}
	

	return(status);
}

Status_t
SM_Set_SwitchCongestionSetting(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr,
	STL_SWITCH_CONGESTION_SETTING *swcs, uint64_t mkey)
{
	uint32_t bufferLength = sizeof(STL_SWITCH_CONGESTION_SETTING);
	uint8_t  buffer[bufferLength];
	Status_t status;

	INCREMENT_COUNTER(smCounterSetSwitchCongestionSetting);

	memcpy(buffer, swcs, sizeof(STL_SWITCH_CONGESTION_SETTING));
	ZERO_RSVD_STL_SWITCH_CONGESTION_SETTING((STL_SWITCH_CONGESTION_SETTING*)buffer);
	BSWAP_STL_SWITCH_CONGESTION_SETTING((STL_SWITCH_CONGESTION_SETTING*)buffer);

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_CONG) {
		status = VSTATUS_OK;
	} else {
		status = sm_set_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SWITCH_CONGESTION_SETTING, amod, addr, buffer, &bufferLength, mkey);
	}
	if (status == VSTATUS_OK) {
		BSWAP_STL_SWITCH_CONGESTION_SETTING((STL_SWITCH_CONGESTION_SETTING*)buffer);
		memcpy(swcs, buffer, sizeof(STL_SWITCH_CONGESTION_SETTING));
	}

	return (status);
}


Status_t
SM_Get_SwitchPortCongestionSetting(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_SWITCH_PORT_CONGESTION_SETTING *swpcs) {
	const uint8_t count = (amod>>24) & 0xff;
	uint32_t	bufferLength = sizeof(STL_SWITCH_PORT_CONGESTION_SETTING_ELEMENT) * count;
	uint8_t		buffer[bufferLength];
	Status_t	status;

	INCREMENT_COUNTER(smCounterGetSwitchPortCongestionSetting);
	
	status = sm_get_stl_attribute(fd, STL_MCLASS_ATTRIB_ID_SWITCH_PORT_CONGESTION_SETTING, amod, addr, buffer, &bufferLength);
	if (status == VSTATUS_OK) {
		BSWAP_STL_SWITCH_PORT_CONGESTION_SETTING((STL_SWITCH_PORT_CONGESTION_SETTING*)buffer, count);
		memcpy(swpcs, buffer, bufferLength);
	}

	return(status);
}

Status_t
SM_Set_BufferControlTable(SmMaiHandle_t *fd, uint32_t amod, SmpAddr_t *addr, STL_BUFFER_CONTROL_TABLE pbct[], uint64_t mkey, uint32_t *madStatus)
{
	Status_t status;
	uint32_t i;
	uint8_t buffer[STL_MAD_PAYLOAD_SIZE];
	uint8_t portCount = (amod>>24 & 0xff);

	uint32_t bufferLength = STL_BFRCTRLTAB_PAD_SIZE * portCount;
	if (bufferLength > STL_MAD_PAYLOAD_SIZE)
		return (VSTATUS_ILLPARM);

	memset(buffer, 0, bufferLength);
	uint8_t * data = buffer;
	STL_BUFFER_CONTROL_TABLE *table = (STL_BUFFER_CONTROL_TABLE *)data;

	INCREMENT_COUNTER(smCounterSetBufferControlTable);

	if (madStatus)
		*madStatus = 0;

	for (i = 0; i < portCount; i++) {
		memcpy(table, &pbct[i], sizeof(STL_BUFFER_CONTROL_TABLE));
		table->Reserved = 0;
		(void)BSWAP_STL_BUFFER_CONTROL_TABLE(table);
		// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
		data += STL_BFRCTRLTAB_PAD_SIZE;
		table = (STL_BUFFER_CONTROL_TABLE *)(data);
	}

	if (sm_config.skipAttributeWrite & SM_SKIP_WRITE_BFRCTRL) {
		status = VSTATUS_OK;
	} else {
		// sm_set_stl_attribute_mad_status returns VSTATUS_BAD, if there is a madStatus error code.
		status = sm_set_stl_attribute_mad_status(fd, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE, amod, addr, buffer, &bufferLength, mkey, madStatus);
		// PR 125784: If buffer control table update is attempted for the same port back-to-back (say due to a SM retry following
		// lost response), FW can return BUSY only if the values are being changed otherwise, returns OK. Caller to treat BUSY as error and trigger resweep.
	}
	if (status == VSTATUS_OK) {
		data= buffer;
		table = (STL_BUFFER_CONTROL_TABLE *)(data);
		for (i = 0; i < portCount; i++) {
			(void)BSWAP_STL_BUFFER_CONTROL_TABLE(table);
			table->Reserved = 0;
			memcpy(&pbct[i], table, sizeof(STL_BUFFER_CONTROL_TABLE));
			// Handle the dissimilar sizes of Buffer Table and 8-byte pad alignment
			data += STL_BFRCTRLTAB_PAD_SIZE;
			table = (STL_BUFFER_CONTROL_TABLE *)(data);
		}
	}

	return (status);
}

/**
	Compose an aggregate requests for @c segCount 64-byte segments for the ports in the range <tt>[startPort, endPort]</tt>, each starting at segment @c startSeg.

	Performs necessary byte-swapping on the aggregate segment headers to put them in network order.

	@return pointer to the start of the next aggregate segment available (one past the end of the response space of the last segment added).
*/
static STL_AGGREGATE *
SM_ComposeCableInfoAggr(uint8_t * buffer, size_t bufferSize,
	uint8_t startPort, uint8_t endPort, uint8_t startSeg, uint8_t segCount)
{
	size_t reqMem = (sizeof(STL_AGGREGATE) + sizeof(STL_CABLE_INFO))* ((endPort - startPort + 1) * segCount);

	if (reqMem > bufferSize) {
		return NULL;
	}

	STL_AGGREGATE * aggr, * oldAggr;
	oldAggr = NULL;
	aggr = (STL_AGGREGATE*) buffer;

	int i;
	uint8_t portIdx;

	for (portIdx = startPort; portIdx <= endPort; ++portIdx) {
		for (i = 0; i < segCount; ++i) {
			uint16_t startAddr = (i + startSeg) * sizeof(STL_CABLE_INFO);
			uint32 amod = ((startAddr & 0x0FFF) << 19) | (((sizeof(STL_CABLE_INFO) - 1) & 0x3F) << 13) | portIdx;

			memset(aggr, 0, sizeof(STL_AGGREGATE));
			aggr->AttributeID = STL_MCLASS_ATTRIB_ID_CABLE_INFO;
			aggr->Result.s.RequestLength = (sizeof(STL_CABLE_INFO) + 7)/8;
			aggr->AttributeModifier = amod;

			oldAggr = aggr;
			aggr = STL_AGGREGATE_NEXT(aggr);
			BSWAP_STL_AGGREGATE_HEADER(oldAggr);
		}
	}

	return aggr;
}

Status_t
SM_Get_CableInfo(SmMaiHandle_t *fd, uint8_t portIdx, uint8_t startSeg, uint8_t segCount, SmpAddr_t *addr, STL_CABLE_INFO * ci, uint32_t * madStatus)
{
	Status_t status = VSTATUS_OK;
	uint32_t bufferLength = sizeof(STL_CABLE_INFO);

	if (madStatus)
		*madStatus = 0;

	uint8 buffer[STL_MAD_PAYLOAD_SIZE] = { 0 };

	if (sm_config.use_aggregates) {
		STL_AGGREGATE * aggr = SM_ComposeCableInfoAggr(buffer, sizeof(buffer),
			portIdx, portIdx, startSeg, segCount);

		if (!aggr) {
			return VSTATUS_TOO_LARGE;
		}

		bufferLength = ((uint8_t*)aggr) - buffer;

		INCREMENT_COUNTER(smCounterGetCableInfo);

		status = sm_send_stl_request( fd, MAD_CM_GET, STL_MCLASS_ATTRIB_ID_AGGREGATE,
			segCount, addr, buffer, &bufferLength, 0, madStatus);

		aggr = (STL_AGGREGATE*) buffer;
		BSWAP_STL_AGGREGATE_HEADER(aggr);

		uint32 i;
		for (i = 0; i < segCount && aggr->Result.s.Error == 0; ++i) {
			memcpy(&ci[i], aggr->Data, sizeof(STL_CABLE_INFO));
			BSWAP_STL_CABLE_INFO(&ci[i]);
			aggr = STL_AGGREGATE_NEXT(aggr);
			BSWAP_STL_AGGREGATE_HEADER(aggr);
		}
	} else {
		uint8_t i;
		for (i = 0; i < segCount && status == VSTATUS_OK; ++i) {
			uint16_t startAddr = (i + startSeg) * sizeof(STL_CABLE_INFO);
			uint32 amod = ((startAddr & 0x0FFF) << 19) | (((sizeof(STL_CABLE_INFO) - 1) & 0x3F) << 13) | portIdx;

			INCREMENT_COUNTER(smCounterGetCableInfo);

			status = sm_send_stl_request(fd, MAD_CM_GET, STL_MCLASS_ATTRIB_ID_CABLE_INFO,
				amod, addr, buffer, &bufferLength, 0, madStatus);

			if (status == VSTATUS_OK) {
				memcpy(&ci[i], buffer, sizeof(STL_CABLE_INFO));
				BSWAP_STL_CABLE_INFO(&ci[i]);
			}
		}
	}

	return status;
}



Status_t
SM_Get_Aggregate_DR(SmMaiHandle_t *fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	size_t lastSegReqLen, uint8_t *path, STL_AGGREGATE ** lastSeg, uint32_t * madStatus)
{
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
	return SM_Aggregate_impl(fd, start, end, lastSegReqLen, &addr, MAD_CM_GET, 0, lastSeg, madStatus);
}

Status_t
SM_Get_Aggregate_LR(SmMaiHandle_t *fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	size_t lastSegReqLen, STL_LID srcLid, STL_LID destLid, STL_AGGREGATE ** lastSeg,
	uint32_t * madStatus)
{
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(srcLid, destLid);
	return SM_Aggregate_impl(fd, start, end, lastSegReqLen, &addr, MAD_CM_GET, 0, lastSeg, madStatus);
}


Status_t
SM_Set_Aggregate_LR(SmMaiHandle_t *fd, STL_AGGREGATE * start, STL_AGGREGATE * end,
	STL_LID srcLid, STL_LID destLid, uint64_t mkey, STL_AGGREGATE ** lastSeg, uint32_t * madStatus)
{
	SmpAddr_t addr = SMP_ADDR_CREATE_LR(srcLid, destLid);
	return SM_Aggregate_impl(fd, start, end, 0, &addr, MAD_CM_SET, mkey, lastSeg, madStatus);
}

/**
	Utility function for sending and receiving LID-routed aggregate requests.  Takes requests from
	<tt>[inStart, inEnd)<\tt>, prepares and sends requests, and writes back results to same memory.  Sends
	as many MADs as necessary to complete all requests.

	Requests are stored in <tt>[inStart, inEnd)<\tt>.  @c inStart and @c inEnd must define a contiguous
	range in memory.

	@c STL_AGGREGATE segment headers contained in request data must be in host order.  Aggregate segment
	payloads should be in network order.  On response function will put response aggregate segment headers
	in host order.

	Processing stops on the first error, whether on sending or receiving and status is returned.  In the
	event of a MAD error, function will copy back up to and including the first failed segment.

	Function does not guard against mangled RequestLength in individual response segment headers but does
	guard against response segments exceeding bounds of original request segments.

	@param method Should be either MAD_CM_GET or MAD_CM_SET.  If MAD_CM_GET, for each MAD, the function will send up to the last segment header instead of last segment header+payload.
	@param lastSeg [out] points to last segment successfully processed in the range [@c inStart, @c inEnd] or NULL if no segments were processed.  <tt>*lastSeg != NULL && *lastSeg != inEnd</tt> indicates that some but not all segments were processed by the SMA; in this case, @c *lastSeg is a pointer to the first failed segment

	@param madStatus [out, optional] If provided, @c *madStatus will contain the full MAD status for the first operation to fail.

	@return
		@c VSTATUS_OK if <tt>inEnd == inStart</tt>; sets <tt>*lastSeg = inStart</tt>
		@c VSTATUS_ILLPARM if @c lastSeg is NULL
		@c VSTATUS_ILLPARM if @c method is something other than @c MAD_CM_GET or @c MAD_CM_SET
		@c VSTATUS_BAD if the segment lenght in a segment header results in a buffer pointer going past @c inEnd
		@c VSTATUS_BAD if no segments were added to a request
		@c VSTATUS_BAD if swapping from/to host order fails
		The return value of sm_send_stl_request_impl() for either the first error or the last successful send otherwise.
*/
static Status_t
SM_Aggregate_impl(
	SmMaiHandle_t *fd, STL_AGGREGATE * inStart, STL_AGGREGATE * inEnd, size_t lastSegReqLen,
	SmpAddr_t * addr, uint8 method, uint64_t mkey, STL_AGGREGATE ** lastSeg, uint32_t * madStatus)
{
	Status_t s = VSTATUS_OK;
	uint32_t lclMadStatus;
	const size_t maxPayload = addr->path ? STL_MAX_PAYLOAD_SMP_DR : STL_MAX_PAYLOAD_SMP_LR;

	if (!lastSeg) {
		return VSTATUS_ILLPARM;
	}
	*lastSeg = NULL;

	switch (method) {
		case MAD_CM_GET:
		case MAD_CM_SET:
			break;
		default:
			IB_LOG_ERROR_FMT(__func__, "Unsupported method : %d", method);
			return VSTATUS_ILLPARM;
	}

	if (!madStatus)
		madStatus = &lclMadStatus;
	*madStatus = MAD_STATUS_SUCCESS;

	if (inEnd == inStart) {
		*lastSeg = inStart;
		return VSTATUS_OK;
	}

	// next points to the end of the last segment to send/first segment after the current block
	STL_AGGREGATE * cur, * next;
	next = cur = inStart;

	while (cur < inEnd) {
		uint16 segCount = 0;

		uint16_t lastSegSize = 0;
		uint32_t reqLength = 0;

		while (next < inEnd && segCount < MAX_AGGREGATE_ATTRIBUTES) {
			if (((uint8_t*)STL_AGGREGATE_NEXT(next) - (uint8_t*)cur) > maxPayload)
				break;
			lastSegSize = next->Result.s.RequestLength * 8;
			next = STL_AGGREGATE_NEXT(next);
			reqLength = (uint32_t)((uint8_t*)next - (uint8_t*)cur);
			++segCount;
		}

		// next == inEnd is OK, next > inEnd is not
		if (next > inEnd) {
			IB_LOG_ERROR_FMT(__func__, "Next segment goes past buffer boundary");
			return VSTATUS_BAD;
		}

		if (cur == next || segCount == 0) {
			IB_LOG_ERROR_FMT(__func__, "No segments added");
			s = VSTATUS_BAD;
			break;
		}


		if (method == MAD_CM_GET) {
			// Get() requests need only be as long as the last segment header
			reqLength -= lastSegSize;
			// However, some attributes have required data in the request struct
			reqLength += lastSegReqLen;
			INCREMENT_COUNTER(smCounterGetAggregate);
		}
		else {
			INCREMENT_COUNTER(smCounterSetAggregate);
		}

		uint32_t respLength = (uint8*)next - (uint8*)cur;
		uint8 buffer[maxPayload];
		memcpy(buffer, cur, reqLength);


		int actCount = segCount;
		STL_AGGREGATE * bufEnd = (STL_AGGREGATE*)(buffer + respLength);

		BSWAP_ZERO_AGGREGATE_HEADERS((STL_AGGREGATE*)buffer, bufEnd, &actCount, 1);

		if (actCount != segCount) {
			IB_LOG_ERROR_FMT(__func__, "Failed to prep all request segments");
			s = VSTATUS_BAD;
			break;
		}

		s = sm_send_stl_request_impl(fd, method, STL_MCLASS_ATTRIB_ID_AGGREGATE,
			segCount, addr, reqLength, buffer, &respLength,
			WAIT_FOR_REPLY, mkey, NULL, NULL, madStatus);

		// sm_send_stl_request_impl() returns a non-OK status on MAD errors
		// So a mix of (OK,!OK)x(s,*madStatus) is an invalid or undefined state
		if ((s == VSTATUS_OK) ^ (*madStatus == MAD_STATUS_SUCCESS)) {
			IB_LOG_ERROR_FMT(__func__, "Unknown send/recv error");
			return VSTATUS_BAD;
		}

		//TODO should compare response and request segment lengths and attr IDs
		bufEnd = BSWAP_ZERO_AGGREGATE_HEADERS((STL_AGGREGATE*)buffer, bufEnd, &actCount, 0);

		if (((uint8*)bufEnd - (uint8*)buffer) > respLength) {
			return VSTATUS_BAD;
		}

		// If MAD status is not OK, only way we can continue to process is if we're
		// sure that the error bit had been set on one of the segment headers; otherwise,
		// we could be getting our request back at us
		if (*madStatus != MAD_STATUS_SUCCESS) {
			if (bufEnd < (STL_AGGREGATE*)((uint8*)buffer + respLength) && bufEnd->Result.s.Error) {
				// Error bit set; we're pretty sure that the SMA did *something* to the input
				break;
			}

			IB_LOG_ERROR_FMT(__func__, "SMA does not support aggregate operations.");
			*lastSeg = NULL;
			return VSTATUS_BAD;
		}

		// Only way 'bufEnd < buffer+respLength' is valid is if an error occurred on a specific segment
		if (((uint8*)bufEnd - (uint8*)buffer) < respLength) {
			if (!bufEnd->Result.s.Error) {
				return VSTATUS_BAD;
			}

			//TODO remove once above code compares request/response attr IDs & offsets
			// SMA should never modify RequestLength, but if it does, we have a problem
			const STL_AGGREGATE * reqHdr = &cur[bufEnd - (STL_AGGREGATE*)buffer];
			if (bufEnd->Result.s.RequestLength != reqHdr->Result.s.RequestLength) {
				IB_LOG_ERROR_FMT(__func__,
					"Request segment header length != response segment header length."
					"Request length : %d bytes; response length : %d bytes.",
					reqHdr->Result.s.RequestLength * 8, bufEnd->Result.s.RequestLength * 8);
				// 
				bufEnd->Result.s.RequestLength = reqHdr->Result.s.RequestLength;
			}

			// lastSeg -> start of first error segment
			*lastSeg = (STL_AGGREGATE*)((uint8*)cur + ((uint8*)bufEnd - (uint8*)buffer));

			// Copy back the data associated with the failed segment as well
			bufEnd = STL_AGGREGATE_NEXT(bufEnd);
		}
		else {
			*lastSeg = (STL_AGGREGATE*)((uint8*)cur + ((uint8*)bufEnd - (uint8*)buffer));
		}

		memcpy(cur, buffer, MIN(respLength, (uint8*)bufEnd - (uint8*)buffer));

		if (*madStatus != MAD_STATUS_SUCCESS) {
			IB_LOG_ERROR_FMT(__func__, "Bad MAD status; madStatus : 0x%02x", *madStatus);
			break;
		}

		if (actCount != segCount) {
			IB_LOG_ERROR_FMT(__func__,
				"Request/parsed response aggregate count mismatch; expected : %d; actual : %d",
				segCount, actCount);
			break;
		}

		cur = next;
	}

	return s;
}

