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

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_MultiPathRecord.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the MultiPathRecord type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_MultiPathRecord						     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "iba/ib_sa_records.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_l.h"
#include "sa_l.h"

extern		Pool_t		sm_pool;

Status_t	sa_PathRecord_Set(uint32_t*, uint32_t, Port_t*, uint32_t, Port_t*, uint32_t, PKey_t, uint64_t, uint8_t, uint8_t);
extern 		IB_GID nullGid;

static uint8_t serviceIdCheck;

Status_t
sa_MultiPathRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t		bytes=0;
	uint32_t		records;
	uint32_t		sGidIndex;
	uint32_t		dGidIndex;
	uint64_t		prefix;
	uint64_t		guid;
	STL_SA_MAD			samad;
	Port_t			*src_portp	= NULL;
	Port_t			*dst_portp	= NULL;
	IB_MULTIPATH_RECORD	*mprp;
	Status_t		status;
	PKey_t			pkey=0;
	Port_t*			reqPortp;
	Node_t*			reqNodep;
	uint64_t		serviceId=0;
	uint8_t			sl=0xff;
	IB_GID			*sGidList = NULL;
	IB_GID			*dGidList = NULL;

	IB_ENTER("sa_MultiPathRecord", maip, 0, 0, 0);

//
//	Assume failure.
//
	records = 0;

//
//	Lock the interface.
//
	(void)vs_rdlock(&old_topology_lock);

//
// Allocate enough memory to process the request.
//
	status = vs_pool_alloc(&sm_pool, 
		sizeof(IB_MULTIPATH_RECORD)+ 2*IB_MULTIPATH_RECORD_MAX_GIDCOUNT,
		(void**)&mprp);

	if (status != VSTATUS_OK) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
		IB_LOG_ERRORRC("sa_MultiPathRecord: can't allocate multipath record rc:", status);
		goto reply_MultiPathRecord;
	}

	(void)memset((void *)mprp, 0, 
		sizeof(IB_MULTIPATH_RECORD)+ 2*IB_MULTIPATH_RECORD_MAX_GIDCOUNT);

//
//	Check the basic assumptions.
//
	if (maip->base.method != SA_CM_GETMULTI) {
		goto reply_MultiPathRecord;
	}
	// incremented in sa_process_getmulti
	// INCREMENT_COUNTER(smCounterSaRxGetMultiPathRecord);

	// Note that we copy the entire payload. multipath queries are variable
	// length.
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, maip->datasize);
	memcpy(mprp, (void*)samad.data, maip->datasize-sizeof(samad.header));
	BSWAP_IB_MULTIPATH_RECORD(mprp);

//
//	Was NumbPath specified correctly?
//
	if ((samad.header.mask & IB_MULTIPATH_RECORD_COMP_NUMBPATH) && 
		mprp->NumbPath == 0) {
		IB_LOG_WARN0("sa_MultiPathRecord: NumPath is specified but is zero");
		goto reply_MultiPathRecord;
	}

//
//	Were at least one SGID and one DGID specified?
//
	if ((mprp->SGIDCount == 0) || (mprp->DGIDCount == 0)) {
		IB_LOG_WARN0("sa_MultiPathRecord: Source or Destination GID count is zero");
		goto reply_MultiPathRecord;
	}

//
//	Since we currently always use the same set of PKeys for all ports
//	We can check the validity of the requested PKey here.
//
	if (samad.header.mask & IB_MULTIPATH_RECORD_COMP_PKEY) {
		pkey = mprp->P_Key;
		if (saDebugPerf) IB_LOG_INFINI_INFOX("sa_MultiPathRecord: using PKey ", pkey);
	}

	if (samad.header.mask & IB_MULTIPATH_RECORD_COMP_SL) {
		sl = mprp->u2.s.SL;
		if (saDebugPerf) IB_LOG_INFINI_INFO("sa_MultiPathRecord: using SL ", sl);
	}

// Assume that if one part of a SID is provided, they are both present.
	if (samad.header.mask & IB_MULTIPATH_RECORD_COMP_SERVICEID8MSB ||
		samad.header.mask & IB_MULTIPATH_RECORD_COMP_SERVICEID56LSB) {
		serviceId = ((uint64_t)mprp->serviceID8msb << 56) |
			((uint64_t)mprp->serviceID56lsb[0] << 48) |
			((uint64_t)mprp->serviceID56lsb[1] << 40) |
			((uint64_t)mprp->serviceID56lsb[2] << 32) |
			((uint64_t)mprp->serviceID56lsb[3] << 24) |
			((uint64_t)mprp->serviceID56lsb[4] << 16) |
			((uint64_t)mprp->serviceID56lsb[5] << 8) |
			((uint64_t)mprp->serviceID56lsb[6] << 0);

		if (saDebugPerf) IB_LOG_INFINI_INFOLX("sa_MultiPathRecord: using ServiceId ", serviceId);
		serviceIdCheck = 1;
	} else {
		serviceIdCheck = 0;
	}

//
//	Now process the variable length GID lists
//
	sGidList = mprp->GIDList;
	dGidList = &mprp->GIDList[mprp->SGIDCount];

	for (sGidIndex = 0; sGidIndex < mprp->SGIDCount; sGidIndex++) {
	
	//
	//	Find the node/port for this SGID/SLID combination.
	//
		prefix = sGidList[sGidIndex].Type.Global.SubnetPrefix;
		if (prefix != sm_config.subnet_prefix) {
			IB_LOG_WARNLX("sa_MultiPathRecord: invalid sGid prefix:", prefix);
			continue;
		}
	
		guid = sGidList[sGidIndex].Type.Global.InterfaceID;
		if (guid == 0x0ull) {
			IB_LOG_WARN0("sa_MultiPathRecord: sGuid is zero");
			continue;
		}
	
		if ((src_portp = sm_find_active_port_guid(&old_topology, guid)) == NULL) {
			IB_LOG_WARNLX("sa_MultiPathRecord: Guid not found/active:", guid);
			continue;
		}
	
		if (src_portp->portData->gidPrefix != prefix || src_portp->portData->guid != guid) {
			// This implies an error in sm_find_active_port_guid().
			IB_LOG_WARN0("sa_MultiPathRecord: input Gid does not match");
			continue;
		}
	
		/* ensure that source port is at least in ARMED state */
		if (src_portp->state < IB_PORT_INIT) {
			/* PR#101615 - SM99% when host/line cards disappear in middle of sweep */
			IB_LOG_WARNLX("sa_MultiPathRecord: source port is DOWN; port GUID:", src_portp->portData->guid);
			continue;
		}
	
//
// IBTA 1.2 C15-0.1.21 - pairwise pkey check for request/src.
//
		reqPortp = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, &reqNodep);
		if (!sm_valid_port(reqPortp) ||
			reqPortp->state <= IB_PORT_DOWN ||
			(sa_Compare_Node_Port_PKeys(reqNodep, src_portp) != VSTATUS_OK)) {
			IB_LOG_WARN0("sa_MultiPathRecord: Failed pairwise PKey check for request/src");
			continue;
		}

		for (dGidIndex = 0; dGidIndex < mprp->DGIDCount; dGidIndex++) {
	
			//
			//	Find the destination port.
			//
			prefix = dGidList[dGidIndex].Type.Global.SubnetPrefix;
			if (prefix != sm_config.subnet_prefix) {
				IB_LOG_WARNLX("sa_MultiPathRecord: invalid sGid prefix:", prefix);
				continue;
			}
	
			guid = dGidList[dGidIndex].Type.Global.InterfaceID;
			if (guid == 0x0ull) {
				IB_LOG_WARN0("sa_MultiPathRecord: sGuid is zero");
				continue;
			}
	
			if ((dst_portp = sm_find_active_port_guid(&old_topology, guid)) == NULL) {
				IB_LOG_WARNLX("sa_MultiPathRecord: dGuid not found/active guid:", guid);
				continue;
			}
	
			if (dst_portp->portData->gidPrefix != prefix || dst_portp->portData->guid != guid) {
				// This implies an error in sm_find_active_port_guid().
				IB_LOG_WARN0("sa_MultiPathRecord: input destination Gid does not match");
				continue;
			}
			//
			//	Find the requested paths from the source port to the destination port.
			//
			if (dst_portp != NULL && src_portp != NULL) {
				/* PR#101615 - SM99% when host/line cards disappear in middle of sweep */
				if (dst_portp->state <= IB_PORT_INIT) {
					IB_LOG_WARNLX("sa_MultiPathRecord: destination port is DOWN; port GUID:", dst_portp->portData->guid);
					continue;
				}

				//
				// IBTA 1.2 C15-0.1.21 - pairwise pkey check for request/dst.
				//                       pairwise pkey check for src/dst (when pkey not specified).
				//
				if (sa_Compare_Node_Port_PKeys(reqNodep, dst_portp) != VSTATUS_OK) {
					if (saDebugPerf) {
        				IB_LOG_INFINI_INFO_FMT("sa_MultiPathRecord",
							"Failed pairwise PKey check for request from node "FMT_U64" for req/dst port "FMT_U64" ",
							reqNodep->nodeInfo.NodeGUID, dst_portp->portData->guid);
					}
					maip->base.status = MAD_STATUS_SA_REQ_INVALID;
					continue;
    			}
		
				if (pkey) {
					// Does requested pkey match?
					if (!smValidatePortPKey2Way(pkey, src_portp, dst_portp) && (src_portp != dst_portp)) {
						if (saDebugPerf) {
							IB_LOG_WARN_FMT("sa_MultiPathRecord",
				   				"Cannot find path to port LID 0x%x (port guid "FMT_U64") from port LID 0x%x (port guid "FMT_U64") with pkey 0x%x",
				   				dst_portp->portData->lid, dst_portp->portData->guid, src_portp->portData->lid, src_portp->portData->guid, pkey);
						}
						continue;
					}
		
				} else if (sa_Compare_PKeys(src_portp->portData->pPKey, dst_portp->portData->pPKey) != VSTATUS_OK) {
					// Do any pkeys match?
					if (saDebugPerf) {
            			IB_LOG_INFO_FMT("sa_MultiPathRecord",
                			"Cannot find path to port "FMT_U64" from port "FMT_U64": failing src/dst pkey 0x%x validation",
                			dst_portp->portData->guid, src_portp->portData->guid, pkey);
					}
					continue;
				}

				if (sa_PathRecord_Set(&records, records+mprp->NumbPath, src_portp, PERMISSIVE_LID,
						dst_portp, PERMISSIVE_LID,  pkey, serviceId, serviceIdCheck, sl) == VSTATUS_OK) {
					if (records >= sa_max_path_records) {
            			break;
        			}
				}
			}
		}
		if (records >= sa_max_path_records) {
           	break;
        }
	}

//
//	Determine reply status
//
reply_MultiPathRecord:
	(void)vs_rwunlock(&old_topology_lock);

	if (saDebugPerf) IB_LOG_INFINI_INFO("sa_MultiPathRecord: Num path records to return is ", records);
	bytes += sizeof(IB_PATH_RECORD);
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	}

	sa_cntxt->attribLen = bytes;

	(void)sa_cntxt_data(sa_cntxt, sa_data, records * bytes);
	(void)sa_send_reply(maip, sa_cntxt);

	if (mprp != (void *)NULL) {
		vs_pool_free(&sm_pool, (void*)mprp);
	}

	IB_EXIT("sa_MultiPathRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

