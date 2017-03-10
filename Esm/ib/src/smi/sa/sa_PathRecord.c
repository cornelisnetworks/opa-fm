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
//    sa_PathRecord.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the PathRecord type.					     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_PathRecord							     //
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
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "fm_xml.h"

Status_t	sa_PathRecord_Set(uint8_t*, uint32_t*, uint32_t, Port_t*, uint32_t, Port_t*, uint32_t, PKey_t, uint64_t, uint8_t, uint8_t);
void        sa_GroupPathRecord_Set(uint8_t*, uint32_t*, Port_t *, McGroup_t *);
Status_t	sa_PathRecord_Wildcard(uint8_t*, Port_t *, uint32_t, uint32_t *, PKey_t, uint8_t, Node_t*, uint64_t, uint8_t);
Status_t	sa_PathRecord_Interop(IB_PATH_RECORD *, uint64_t);
IB_GID		nullGid={.Raw={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

/************ support for dynamic update of switch config parms *************/
extern uint8_t sa_dynamicPlt[];

static uint16_t srcLids[128];
static uint16_t dstLids[128];

static uint8_t serviceIdCheck;

static Status_t
create_ib_mask(Mai_t *maip, IB_SA_MAD *samad) {
	// exclude special cases from generic mask comparison
	uint64 templateMask = samad->SaHdr.ComponentMask & (~(uint64)
		// ibta defines reversible == 0 to mean "need not be reversible" rather than "must not be reversible"
		( IB_PATH_RECORD_COMP_REVERSIBLE
		// query-only field that limits rather than matches the number of responses
		| IB_PATH_RECORD_COMP_NUMBPATH
		// selectable fields have special behavior rather than direct match
		| IB_PATH_RECORD_COMP_MTUSELECTOR
		| IB_PATH_RECORD_COMP_MTU
		| IB_PATH_RECORD_COMP_RATESELECTOR
		| IB_PATH_RECORD_COMP_RATE
		| IB_PATH_RECORD_COMP_PKTLIFESELECTOR
		| IB_PATH_RECORD_COMP_PKTLIFE
		// exclude fields that are manually checked as part of the code flow
		| IB_PATH_RECORD_COMP_SGID
		| IB_PATH_RECORD_COMP_DGID
		| IB_PATH_RECORD_COMP_SLID
		| IB_PATH_RECORD_COMP_DLID
		| IB_PATH_RECORD_COMP_PKEY
		| IB_PATH_RECORD_COMP_SERVICEID
		| IB_PATH_RECORD_COMP_SL // equivalent to STL's SL Base
		));

	return sa_create_template_mask(maip->base.aid, templateMask);
}

Status_t
sa_PathRecord(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint32_t	bytes;
	uint32_t	records;
	uint64_t	prefix=0,pf2=0;
	uint64_t	guid=0,sguid=0;
	IB_SA_MAD		samad;
	Port_t		*src_portp;
	Port_t		*dst_portp;
	IB_PATH_RECORD	*prp;
	IB_PATH_RECORD	pathRecord;
	PKey_t		pkey=0;
    McGroup_t   *mcastGroup=NULL;
    uint32_t    dstIsGroup=0;
	Port_t*		reqPortp;
	Node_t*		reqNodep;
	uint64_t	serviceId=0;
	uint8_t		sl=0xff;
	Status_t	status;

	// init to permissive lid to represent wildcarded lid
	uint32_t    slid = PERMISSIVE_LID, dlid = PERMISSIVE_LID;

	IB_ENTER("sa_PathRecord", maip, 0, 0, 0);

	if (maip->base.cversion != SA_MAD_CVERSION) {
		maip->base.status = MAD_STATUS_BAD_CLASS;
		(void) sa_send_reply(maip, sa_cntxt);
		IB_LOG_WARN("sa_PathRecord: invalid CLASS:",
					maip->base.cversion);
		IB_EXIT("sa_PathRecord", VSTATUS_OK);
		return (VSTATUS_OK);
	}

//
//	Assume failure.
//
	records = 0;

//
//	Lock the interface.
//
	(void)vs_rdlock(&old_topology_lock);
	bytes = 0;

//
//	Check the basic assumptions.
//
	if (maip->base.method != SA_CM_GET) {
		if (maip->base.method != SA_CM_GETTABLE) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_WARN("sa_PathRecord: bad method:", maip->base.method);
			goto reply_PathRecord;
		} else {
			INCREMENT_COUNTER(smCounterSaRxGetTblPathRecord);
		}
	} else {
		INCREMENT_COUNTER(smCounterSaRxGetPathRecord);
	}

	prp = &pathRecord;

//
//  Verify the size of the data received for the request
//
	if ( maip->datasize-sizeof(SA_MAD_HDR) < sizeof(IB_PATH_RECORD) ) {
		IB_LOG_ERROR_FMT(__func__,
			"invalid MAD length; size of IB_PATH_RECORD[%"PRISZT"], datasize[%d]",
			sizeof(IB_PATH_RECORD), (int)(maip->datasize-sizeof(SA_MAD_HDR)));
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		IB_EXIT("sa_PathRecord", MAD_STATUS_SA_REQ_INVALID);
		return (MAD_STATUS_SA_REQ_INVALID);
	}
	
	BSWAPCOPY_IB_SA_MAD((IB_SA_MAD*)maip->data, &samad, sizeof(IB_PATH_RECORD));
	memcpy(prp,samad.Data,sizeof(IB_PATH_RECORD));
	BSWAP_IB_PATH_RECORD(prp);

	if ((status = create_ib_mask(maip, &samad)) != VSTATUS_OK) {
		IB_LOG_WARNRC("sa_PathRecord: failed to create IB template mask, rc:", status);
		goto reply_PathRecord;
	}

//
//	JSY - this is a quick cut-through check for temporarily static fields.
//
	if (sa_PathRecord_Interop(prp, samad.SaHdr.ComponentMask) != VSTATUS_OK) {
		IB_LOG_WARNX("sa_PathRecord: mask failed interop:", samad.SaHdr.ComponentMask);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		goto reply_PathRecord;
	}

//
//	Check the validity of the requested PKey.
//
	if (samad.SaHdr.ComponentMask & PR_COMPONENTMASK_PKEY) {
		pkey = prp->P_Key;
	}

	if (samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SRV_ID) {
		serviceId = prp->ServiceID;
		serviceIdCheck = 1;
	} else {
		serviceIdCheck = 0;
	}

	if (samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SL) {
		sl = prp->u2.s.SL;
	}

//
//  GetTable only requirement - must have SGID and Numpath specified
//
	if (maip->base.method == SA_CM_GETTABLE) {
		/* validate the minimal set of components are specified */
		if (sm_config.queryValidation) {
			/* IBTA requires SGID and NumbPath on GetTable */
			if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_OK_SRC) != PR_COMPONENTMASK_OK_SRC) {
				maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
				IB_LOG_WARNX("sa_PathRecord: must specify a SGID and number of paths for GetTable:", samad.SaHdr.ComponentMask);
				goto reply_PathRecord;
			}
		} else {
			// For compatibility with OFED: Allow GETTABLE requests with a SLID
			// instead of a SGID.

			/* Were the SGID or SLID and NumbPath specified? */
			if (!(samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SGID) && !(samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SLID)) {
				maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
				IB_LOG_WARNX("sa_PathRecord: must specify a SLID or SGID for GetTable:", samad.SaHdr.ComponentMask);
				goto reply_PathRecord;
			} else if (!(samad.SaHdr.ComponentMask & PR_COMPONENTMASK_PATHS)) {
#if 0
				maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
				IB_LOG_WARNX("sa_PathRecord: must specify the number of paths for GetTable:", samad.SaHdr.ComponentMask);
				goto reply_PathRecord;
#else
				// OFED expects a default of 127
				prp->NumbPath = 127;
#endif
			}
		}

		/* now validate the components we have present */
		if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SGID) &&
			(memcmp(&prp->SGID, &nullGid, sizeof(IB_GID)) == 0)) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
			IB_LOG_WARN0("Invalid Query: Mask specifies a source GID but source GID is NULL.");
			goto reply_PathRecord;
		}
		if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SLID) &&
			(prp->SLID == 0)) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_WARN0("Invalid Query: Mask specifies a source LID but source LID is zero.");
			goto reply_PathRecord;
		}
		if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_PATHS) &&
	   		(prp->NumbPath == 0)) {
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			IB_LOG_WARN0("Invalid Query: Mask specifies numbPath, but numbPath = 0.");
			goto reply_PathRecord;
		}
	} else {
		/* GET can have either SLID or DGID */
		/* Spec states this should be ignored and a value of 1 should be used for get */
		prp->NumbPath = 1;
	}

//
// Find the requestor
//
	reqPortp = sm_find_node_and_port_lid(&old_topology, maip->addrInfo.slid, &reqNodep);
	if (!sm_valid_port(reqPortp) || reqPortp->state <= IB_PORT_DOWN) {
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		goto reply_PathRecord;
	}
//
//	Find the source port.
//
	if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SGID) != 0) {
		prefix = prp->SGID.Type.Global.SubnetPrefix;
		guid = prp->SGID.Type.Global.InterfaceID;
		if (prefix != sm_config.subnet_prefix) {
            IB_LOG_ERROR_FMT( "sa_PathRecord",
                   "gidprefix in Source Gid "FMT_GID" of PATH request from %s port "FMT_U64" LID 0x%x does not match SM's("FMT_U64")",
                   prefix, guid, sm_nodeDescString(reqNodep), reqPortp->portData->guid, maip->addrInfo.slid, sm_config.subnet_prefix);
            goto reply_PathRecord;
		}
        if (!guid) {
            IB_LOG_ERROR_FMT( "sa_PathRecord",
                   "NULL PORTGUID in Source Gid "FMT_GID" of PATH request from %s port "FMT_U64" LID 0x%x",
                   prefix, guid, sm_nodeDescString(reqNodep), reqPortp->portData->guid, maip->addrInfo.slid);
            maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
            goto reply_PathRecord;
        }
    	if ((src_portp = sm_find_active_port_guid(&old_topology, guid)) == NULL) {
			if (saDebugPerf) {
				IB_LOG_INFINI_INFOLX("sa_PathRecord: requested source GUID not found/active in current topology:", guid);
			}
			maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID_GID;
			goto reply_PathRecord;
		}
    	if (src_portp->portData->gidPrefix != prefix || src_portp->portData->guid != guid) {
            pf2 = ntoh64(*(uint64_t *)&src_portp->portData->gid[0]);
            sguid = ntoh64(*(uint64_t *)&src_portp->portData->gid[8]);
            IB_LOG_WARN_FMT( "sa_PathRecord",
                   "Source Gid "FMT_GID" in request from %s port "FMT_U64" LID 0x%x does not match found GID's "FMT_GID" "
                   "port guid of "FMT_U64,
                   prefix, guid, sm_nodeDescString(reqNodep), reqPortp->portData->guid, maip->addrInfo.slid, pf2, sguid, src_portp->portData->guid);
			maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
			goto reply_PathRecord;
		}
	} else if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SLID) != 0) {
		if ((src_portp = sm_find_active_port_lid(&old_topology, prp->SLID)) == NULL) {
			IB_LOG_INFINI_INFOX("sa_PathRecord: requested source Lid not found/active in current topology:", prp->SLID);
            maip->base.status = activateInProgress ? MAD_STATUS_BUSY : MAD_STATUS_SA_REQ_INVALID;
			goto reply_PathRecord;
		}
	} else {
		IB_LOG_INFINI_INFO0("sa_PathRecord: Invalid request, neither SGID nor SLID were specified");
		maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
		goto reply_PathRecord;
	}

	if (pkey) {
		if (smGetPortPkey(pkey, src_portp) == 0) {
        	IB_LOG_INFO_FMT("sa_PathRecord",
				"Failed pairwise PKey check (pkey= 0x%x) for request for paths from source port "FMT_U64,
				pkey, src_portp->portData->guid);
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			goto reply_PathRecord;
		}
	}

// if srclid is specified, store it.
	if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_SLID) != 0)
		slid = prp->SLID;

//
// IBTA 1.2 C15-0.1.21 - pairwise pkey check for request/src.
//
	if (sa_Compare_Node_Port_PKeys(reqNodep, src_portp) != VSTATUS_OK) {
        IB_LOG_INFO_FMT( "sa_PathRecord",
				"Failed pairwise PKey check for request from node "FMT_U64" for req/src port "FMT_U64" ",
				reqNodep->nodeInfo.NodeGUID, src_portp->portData->guid);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		goto reply_PathRecord;
	}

//
//	Check for Bus-walk operation.
//
	if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_NO_DST) == 0) {
		if (sa_PathRecord_Wildcard(samad.Data, src_portp, slid, &records, pkey, prp->NumbPath, reqNodep, serviceId, sl) != VSTATUS_OK) {
            maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
		}
		goto reply_PathRecord;
	}

//
//	Find the destination port.
//
	if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_DGID) != 0) {
		prefix = prp->DGID.Type.Global.SubnetPrefix;
		guid = prp->DGID.Type.Global.InterfaceID;
		if (prefix != sm_config.subnet_prefix) {
            IB_LOG_ERROR_FMT( "sa_PathRecord",
                   "gidprefix in Dest Gid "FMT_GID" of PATH request from %s port "FMT_U64" LID 0x%x does not match SM's("FMT_U64")",
                   prefix, guid, sm_nodeDescString(reqNodep), reqPortp->portData->guid, maip->addrInfo.slid, sm_config.subnet_prefix);
		}

        if (!guid) {
            IB_LOG_ERROR_FMT( "sa_PathRecord",
                   "NULL PORTGUID in Destination Gid "FMT_GID" of PATH request from %s port "FMT_U64" LID 0x%x",
                   prefix, guid, sm_nodeDescString(reqNodep), reqPortp->portData->guid, maip->addrInfo.slid);
            maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
            goto reply_PathRecord;
        }

		if ((dst_portp = sm_find_port_guid(&old_topology, guid)) == NULL) {
            (void)vs_lock(&sm_McGroups_lock);   /* lock out the multicast group table */
            if ((mcastGroup = sm_find_multicast_gid(prp->DGID)) == NULL) {
                (void)vs_unlock(&sm_McGroups_lock);
                maip->base.status = MAD_STATUS_SA_NO_RECORDS;
                if (saDebugPerf || !guid) 
                    IB_LOG_INFINI_INFOLX("sa_PathRecord: requested destination GUID not an active port nor a Multicast Group:", guid);
                goto reply_PathRecord;

            } else {
                if (saDebugPerf) {
					IB_LOG_INFINI_INFO_FMT("sa_PathRecord",
					   "Destination Multicast Group "FMT_GID" in request from %s port "FMT_U64" LID 0x%x found",
					   prefix, guid, sm_nodeDescString(reqNodep), reqPortp->portData->guid, maip->addrInfo.slid);
                }
                dstIsGroup = 1;
            }
		} else if (dst_portp->state < IB_PORT_ACTIVE) {
            if (saDebugPerf) IB_LOG_INFINI_INFOLX("sa_PathRecord: requested destination GUID not active in current topology:", guid);
            maip->base.status = MAD_STATUS_SA_NO_RECORDS;
            goto reply_PathRecord;
        }

	} else if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_DLID) != 0) {
		if ((dst_portp = sm_find_active_port_lid(&old_topology, prp->DLID)) == NULL) {
			if (saDebugPerf) IB_LOG_INFINI_INFOX("sa_PathRecord: requested destination Lid not found/active:", prp->DLID);
            maip->base.status = MAD_STATUS_SA_NO_RECORDS;
			goto reply_PathRecord;
		}

	} else {
		IB_LOG_INFINI_INFO0("sa_PathRecord: neither DGID nor DLID were specified");
		maip->base.status = MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS;
		goto reply_PathRecord;
	}
	
//
// if dlid is specified, store it
//
	if ((samad.SaHdr.ComponentMask & PR_COMPONENTMASK_DLID) != 0)
		dlid = prp->DLID;

//
//	Find the requested paths from the source port to the destination port.
//
	if (src_portp != NULL && dst_portp != NULL) {
		if (dst_portp->state <= IB_PORT_INIT) {
			/* PR#101615 - SM99% when host/line cards disappear in middle of sweep */
			if (saDebugPerf) {
				IB_LOG_INFINI_INFO_FMT("sa_PathRecord", "destination port is not in active state; port LID: 0x%x (port GUID "FMT_U64")",
                       dst_portp->portData->lid, dst_portp->portData->guid);
            }
			goto reply_PathRecord;
		}

		//
		// IBTA 1.2 C15-0.1.21 - pairwise pkey check for request/dst.
		//                       pairwise pkey check for src/dst (when pkey not specified).
		//
		if (sa_Compare_Node_Port_PKeys(reqNodep, dst_portp) != VSTATUS_OK) {
			if (saDebugPerf) {
        		IB_LOG_INFINI_INFO_FMT("sa_PathRecord",
					"Failed pairwise PKey check for request from node "FMT_U64" for req/dst port "FMT_U64" ",
					reqNodep->nodeInfo.NodeGUID, dst_portp->portData->guid);
			}
			maip->base.status = MAD_STATUS_SA_REQ_INVALID;
			goto reply_PathRecord;
    	}

		if (pkey) {
			// Does requested pkey match?
			if (!smCheckPortPKey(pkey, dst_portp) && (src_portp != dst_portp)) {
				if (saDebugPerf) {
					IB_LOG_WARN_FMT("sa_PathRecord",
				   		"Cannot find path to port LID 0x%x (port guid "FMT_U64") from port LID 0x%x (port guid "FMT_U64") with pkey 0x%x",
				   		dst_portp->portData->lid, dst_portp->portData->guid, src_portp->portData->lid, src_portp->portData->guid, pkey);
				}
				goto reply_PathRecord;
			}

		} else if (sa_Compare_PKeys(src_portp->portData->pPKey, dst_portp->portData->pPKey) != VSTATUS_OK) {
			// Do any pkeys match?
			if (saDebugPerf) {
            	IB_LOG_INFO_FMT("sa_PathRecord",
                	"Cannot find path to port "FMT_U64" from port "FMT_U64": failing src/dst pkey validation",
                	dst_portp->portData->guid, src_portp->portData->guid);
			}
			goto reply_PathRecord;
        }

		(void)sa_PathRecord_Set(samad.Data, &records, prp->NumbPath, src_portp, slid, dst_portp, dlid,
							pkey, serviceId, serviceIdCheck, sl);

		if (saDebugRmpp && (records == 0)) {
			IB_LOG_INFINI_INFO_FMT("sa_PathRecord",
				"Cannot find path to port LID 0x%x (port guid "FMT_U64") from port LID 0x%x (port guid "FMT_U64")",
				dst_portp->portData->lid, dst_portp->portData->guid, src_portp->portData->lid, src_portp->portData->guid);
		}
	
	} else if (src_portp != NULL && dstIsGroup) {
        // path request from node to multicast group
        // just make sure port is active and a member of the group
        IB_GID gid;
        memcpy(gid.Raw, src_portp->portData->gid, sizeof(IB_GID));
        BSWAP_IB_GID(&gid);
        if (sm_find_multicast_member(mcastGroup, gid) == NULL) {
            //maip->base.status = MAD_STATUS_SA_REQ_INVALID_GID;
            IB_LOG_ERROR_FMT( "sa_PathRecord",
                   "port LID 0x%x (port guid "FMT_U64") not a member of multicast group "FMT_GID,
                   src_portp->portData->lid, src_portp->portData->guid, prefix, guid);
            goto reply_PathRecord;
        }
        sa_GroupPathRecord_Set(samad.Data, &records, src_portp, mcastGroup);
    }

//
//	Determine reply status
//
reply_PathRecord:
	(void)vs_rwunlock(&old_topology_lock);
	if (dstIsGroup) {(void)vs_unlock(&sm_McGroups_lock);}  /*release the multicast group table if necessary */

//
//	Call the user exit to manipulate the records.
//
	(void)pathrecord_userexit(sa_data, &records);

	bytes += sizeof(IB_PATH_RECORD);

	if (maip->base.status != MAD_STATUS_OK) {
		/* status already set */
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == SA_CM_GET) && (records != 1)) {
		IB_LOG_WARN("sa_PathRecord: too many records for SA_CM_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	} else {
		maip->base.status = MAD_STATUS_OK;
	}
	/* setup attribute offset for possible RMPP transfer */
	sa_cntxt->attribLen = bytes;

	(void)sa_cntxt_data(sa_cntxt, sa_data, records * bytes);
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_PathRecord", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
sa_PathRecord_Wildcard(uint8_t *query, Port_t *src_portp, uint32_t slid, uint32_t *records, PKey_t pkey, uint8_t numPath,
						Node_t *reqNodep, uint64_t serviceId, uint8_t sl) {
	Node_t		*dst_nodep;
	Port_t		*dst_portp;
	uint32_t	pathCount;
	Status_t	status=VSTATUS_OK;

	IB_ENTER("sa_PathRecord_Wildcard", src_portp, records, pkey, numPath);

//
//	Loop over all of the nodes and find the paths.
//	
	for_all_nodes(&old_topology, dst_nodep) {
		for_all_end_ports(dst_nodep, dst_portp) {
			if (!sm_valid_port(dst_portp))  continue;
			if (dst_portp->state <= IB_PORT_DOWN) continue;			// RMW - should this also be <= IB_PORT_INIT (PR101615)?
			if (pkey && !smValidatePortPKey(pkey, dst_portp) && (src_portp != dst_portp)) continue;
			if (sa_Compare_Node_Port_PKeys(reqNodep, dst_portp) != VSTATUS_OK) continue;

            // The path count is specified for each SRCGID-DSTGID pair
            // So reset the local path count for each dst port.
			pathCount = (*records);

			if ((status = sa_PathRecord_Set(query, records, pathCount+numPath, src_portp, slid, dst_portp, PERMISSIVE_LID,
								pkey, serviceId, serviceIdCheck, sl)) == VSTATUS_OK) {
               	if ((*records) > sa_max_path_records) {
                 	*records = 0;
                   	IB_EXIT("sa_PathRecord_Wildcard", VSTATUS_BAD);
					return(VSTATUS_BAD);
               	}
				if (numPath && ((*records)-pathCount >= numPath)) break;

			} else if (status == VSTATUS_NOMEM) {
                IB_EXIT("sa_PathRecord_Wildcard", status);
				return(status);
			}
		}
	}

	IB_EXIT("sa_PathRecord_Wildcard", VSTATUS_OK);
	return(VSTATUS_OK);
}

void
sa_GroupPathRecord_Set(uint8_t *query, uint32_t *records, Port_t *src_portp, McGroup_t *group) {
	IB_PATH_RECORD	pathRecord;

	IB_ENTER("sa_GroupPathRecord_Set", src_portp, group, 0, 0);

    //
    //	Fill in the PathRecord.
    //
    //	NOTA BENE: You will notice that the GIDs are copied **after**
    //	the BSWAP call. This is because GIDs are stored in network byte
    //	order and should not be swapped.
    //
	(void)memset((void *)&pathRecord, 0, sizeof(IB_PATH_RECORD));
	(void)memcpy((void *)pathRecord.DGID.Raw, group->mGid.Raw, 16); // group gids are host order
	pathRecord.DLID = group->mLid;
	pathRecord.SLID = src_portp->portData->lid;
	pathRecord.u1.s.RawTraffic = 0;
	pathRecord.u1.s.FlowLabel = group->flowLabel;
	pathRecord.u1.s.HopLimit = group->hopLimit;
	pathRecord.TClass = 0;
    pathRecord.Reversible = 1;
	pathRecord.NumbPath = 0;
	pathRecord.P_Key = group->pKey;
	pathRecord.u2.s.SL = group->sl;
	pathRecord.MtuSelector = PR_EQ;
	pathRecord.Mtu = group->mtu;
	pathRecord.RateSelector = PR_EQ;
	pathRecord.Rate = group->rate;
	pathRecord.PktLifeTimeSelector = PR_EQ;
    pathRecord.PktLifeTime = group->life; 
	pathRecord.Preference = 0;
	BSWAP_IB_PATH_RECORD(&pathRecord);
	(void)memcpy((void *)pathRecord.SGID.Raw, src_portp->portData->gid, 16); // port gids are network order

	if (sa_template_test_noinc(query, (uint8_t *)&pathRecord, sizeof(IB_PATH_RECORD)) != VSTATUS_OK) {
		IB_EXIT(__func__, 0);
		return;
	}

	memcpy(sa_data,&pathRecord,sizeof(IB_PATH_RECORD));
	(*records)++;

	IB_EXIT(__func__, 0);
} // end sa_GroupPathRecord_Set


static void
sa_FillPathRecord (uint8_t *query, uint32_t* records, Port_t *src_portp, uint32_t slid, 
				Port_t *dst_portp, uint32_t dlid, PKey_t pkey,
				uint8_t mtu, uint8_t rate, uint8_t lifeMult,
				uint32_t hopCount, uint64_t serviceId, uint8_t sl) {

	IB_PATH_RECORD	pathRecord;
	uint8_t			*cp = sa_data;

    //
    //	Fill in the PathRecord.  This whole setup is an interop fix.
    //
	(void)memset((void *)&pathRecord, 0, sizeof(IB_PATH_RECORD));

	pathRecord.DLID = dlid;
	pathRecord.SLID = slid;
    pathRecord.ServiceID = serviceId;  
	pathRecord.u1.s.RawTraffic = 0;			// JSY - temp
	pathRecord.u1.s.FlowLabel = 0;		// JSY - temp
	pathRecord.u1.s.HopLimit = 0;		// JSY - temp
	pathRecord.TClass = 0;			// JSY - temp
    pathRecord.Reversible = 1;      // JMS - 1.1 compliance temp
	pathRecord.NumbPath = 0;			// JMS - 0 is OK since undefined in response
	pathRecord.P_Key = smGetPortPkey(pkey, src_portp);	// use membership of src
	pathRecord.MtuSelector = PR_EQ;		// JSY - interop fix
	pathRecord.RateSelector = PR_EQ;	// JSY - interop fix
	pathRecord.PktLifeTimeSelector = PR_EQ;	// JSY - interop fix
	pathRecord.Preference = 0;      // JMS - 1.1 compliance temp
	pathRecord.Mtu = mtu;
	pathRecord.Rate = rate;

	pathRecord.u2.s.SL = sl;

	if (hopCount == 0) {
		/* loopback paths get packetLifetime set to zero */
		pathRecord.PktLifeTime = 0; 
	} else {
		if (!sa_dynamicPlt[0]) {
			pathRecord.PktLifeTime = sm_config.sa_packet_lifetime_n2;		// DSA - VFx1, Lane 15
		} else {
			/* compute the PLT based on hopCount */
			if (saDebugRmpp) {
				IB_LOG_INFINI_INFO_FMT(__func__,
					"hopCount from SGID "FMT_GID", LID 0x%x to DGID "FMT_GID", LID 0x%x is %u",
					pathRecord.SGID.Type.Global.SubnetPrefix, pathRecord.SGID.Type.Global.InterfaceID, src_portp->portData->lid,
					pathRecord.DGID.Type.Global.SubnetPrefix, pathRecord.DGID.Type.Global.InterfaceID, dst_portp->portData->lid,
					hopCount);
            }
			if (hopCount < 8) {
				pathRecord.PktLifeTime = sa_dynamicPlt[hopCount];
			} else {
				pathRecord.PktLifeTime = sa_dynamicPlt[9];
			}
		}
	}

	pathRecord.PktLifeTime += lifeMult;

	IB_LOG_VERBOSE_FMT(NULL, "Path Record %u added to Response:  DLID 0x%x SLID 0x%x SL %u MTU %u Rate %u pkey 0x%x",
			*records, pathRecord.DLID, pathRecord.SLID,
			pathRecord.u2.s.SL, pathRecord.Mtu, pathRecord.Rate, pathRecord.P_Key);

	// NOTA BENE: We don't copy the GIDs till after we bswap the rest
	// of the path record. This is becasue GIDs are stored in network
	// byte order.
	BSWAP_IB_PATH_RECORD(&pathRecord);
	(void)memcpy((void *)&pathRecord.DGID, dst_portp->portData->gid, 16);
	(void)memcpy((void *)&pathRecord.SGID, src_portp->portData->gid, 16);

	if (sa_template_test_noinc(query, (uint8_t *)&pathRecord, sizeof(IB_PATH_RECORD)) != VSTATUS_OK)
		return;

	memcpy(cp + (*records)*sizeof(IB_PATH_RECORD),&pathRecord,sizeof(IB_PATH_RECORD));
	(*records)++;
}


Status_t
sa_PathRecord_Set(uint8_t *query, uint32_t* records, uint32_t numPath, Port_t *src_portp, uint32_t slid,
				Port_t *dst_portp, uint32_t dlid, PKey_t pkey, 
				uint64_t serviceId, uint8_t serviceIdChk, uint8_t sl) {

	uint8_t			mtu, vfMtu;
	uint8_t			rate, vfRate, rated;
	uint32_t		hopCount=1;
	uint8_t			lifeMult;
	int32_t			portno;
	Node_t			*next_nodep;
	Port_t			*next_portp;
	Node_t			*last_nodep;
	Port_t			*last_portp;
	int				vf, vf2;
	uint16_t		slid_iter, dlid_iter;
	lid_iterator_t	iter;
	uint8_t			srcLidLen, dstLidLen;
	bitset_t		vfs;
	Status_t		status=VSTATUS_OK;

	IB_ENTER("sa_PathRecord_Set", src_portp, dst_portp, pkey, 0);

//
//  Get all VFs containing the src/dst which match appropriate path data
//
	if (!bitset_init(&sm_pool, &vfs, MAX_VFABRICS)) {
		IB_LOG_WARN0("sa_PathRecord_Set: insufficient memory to process request");
		IB_EXIT("sa_PathRecord_Set", VSTATUS_NOMEM);
		return(VSTATUS_NOMEM);
	}

	if (serviceIdChk) {
		smGetValidatedServiceIDVFs(src_portp, dst_portp, pkey, sl, sl, serviceId, &vfs);
	} else {
		smGetValidatedVFs(src_portp, dst_portp, pkey, sl, sl, &vfs);
	}
	
	if (vfs.nset_m == 0) {
		goto done_PathRecordSet;
	}

//
//	Locate the node which house these ports.
//
	next_nodep = sm_find_port_node(&old_topology, src_portp);
	last_nodep = sm_find_port_node(&old_topology, dst_portp);
	if ((next_nodep == NULL) || (last_nodep == NULL)) {
		IB_LOG_WARN_FMT("sa_PathRecord_Set",
				"Cannot find nodes for to port "FMT_U64" from port "FMT_U64": failing req/dst pkey validation",
				dst_portp->portData->guid, src_portp->portData->guid);
		status = VSTATUS_BAD;
		goto done_PathRecordSet;
	}

	next_portp = src_portp;
	last_portp = dst_portp;
	if (src_portp == dst_portp) {
		mtu = src_portp->portData->maxVlMtu;
		rate = linkWidthToRate(src_portp->portData);
		hopCount = 0;

	} else if (IsDirectConnected(next_nodep, next_portp, last_nodep, last_portp)) {
		mtu = Min(src_portp->portData->maxVlMtu, dst_portp->portData->maxVlMtu);
		// rates are enums, we need to use linkrate_gt to see which is less.
		rate = linkWidthToRate(src_portp->portData);
		rated = linkWidthToRate(dst_portp->portData);
		if (linkrate_gt(rate,rated)) rate=rated;

		hopCount = 1;

	} else {
		if (next_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			next_nodep = sm_find_node(&old_topology, src_portp->nodeno);
			// PR#103535 - data corruption results in invalid topology
			if (next_nodep == NULL) {
				IB_LOG_WARN_FMT("sa_PathRecord_Set",
				   "Cannot find path to destination port "FMT_U64" from source port "FMT_U64"; INVALID TOPOLOGY, next_nodep is NULL", 
				   dst_portp->portData->guid, src_portp->portData->guid);
				status = VSTATUS_BAD;
				goto done_PathRecordSet;
			}
			next_portp = sm_get_port(next_nodep,0);
			mtu = src_portp->portData->maxVlMtu;
        	rate = linkWidthToRate(src_portp->portData);
		} else if (next_nodep->switchInfo.u2.s.EnhancedPort0) {
			mtu = src_portp->portData->maxVlMtu;
        	rate = linkWidthToRate(src_portp->portData);
		} else {
			mtu = IB_MTU_2048; 
			rate = IB_STATIC_RATE_2_5G;
		}

		if (last_nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
			last_nodep = sm_find_node(&old_topology, dst_portp->nodeno);
			// PR#103535 - data corruption results in invalid topology
			if (last_nodep == NULL) {
				IB_LOG_WARN_FMT("sa_PathRecord_Set",
					   "Cannot find path to destination port "FMT_U64" from source port "FMT_U64"; INVALID TOPOLOGY, last_nodep is NULL", 
					   dst_portp->portData->guid, src_portp->portData->guid);
				status = VSTATUS_BAD;
				goto done_PathRecordSet;
			}
			last_portp = sm_get_port(last_nodep,0);

			mtu = Min(mtu, dst_portp->portData->maxVlMtu);
			rated = linkWidthToRate(dst_portp->portData);
			if (linkrate_gt(rate,rated)) rate=rated;
		} else if (last_nodep->switchInfo.u2.s.EnhancedPort0) {
			mtu = Min(mtu, dst_portp->portData->maxVlMtu);
			rated = linkWidthToRate(dst_portp->portData);
			if (linkrate_gt(rate,rated)) rate=rated;
		} else {
			mtu = IB_MTU_2048; 
			rate = IB_STATIC_RATE_2_5G;
		}
	
		while (next_nodep != last_nodep && next_nodep != NULL) {
            /*
             * PR 106193 - the lft of the switch will be null if a secondary SM has taken ownership.
             * We need to improve discovery to not take ownership of nodes attached to ports that we do not own.
             */
			if (next_nodep->lft == NULL) {
				IB_LOG_WARN_FMT("sa_PathRecord_Set",
					   "INVALID TOPOLOGY, path to portGuid "FMT_U64" from portGuid "FMT_U64" goes through switch [%s] that belongs to another SM", 
					   dst_portp->portData->guid, src_portp->portData->guid, sm_nodeDescString(next_nodep));
				status = VSTATUS_BAD;
				goto done_PathRecordSet;
			}
			++hopCount;
			if (!sm_valid_port(last_portp) || last_portp->portData->lid > next_nodep->switchInfo.LinearFDBTop) {
                if (!sm_valid_port(last_portp)) {
					IB_LOG_ERROR_FMT(__func__,
						"Cannot find last port to port "FMT_U64" from port "FMT_U64": switch %s (guid "FMT_U64") LFTTop [0x%x]",
						dst_portp->portData->guid, src_portp->portData->guid, sm_nodeDescString(next_nodep),
						next_nodep->nodeInfo.NodeGUID, next_nodep->switchInfo.LinearFDBTop);
                } else {
                    /* PR#105641 - SAR crash */
					IB_LOG_WARN_FMT(__func__,
						"Cannot find path to port "FMT_U64" from port "FMT_U64": Lid [0x%x] is greater than switch %s (guid "FMT_U64") LFTTop [0x%x]",
						dst_portp->portData->guid, src_portp->portData->guid, last_portp->portData->lid,
						sm_nodeDescString(next_nodep), next_nodep->nodeInfo.NodeGUID, next_nodep->switchInfo.LinearFDBTop);
                }
				status = VSTATUS_BAD;
				goto done_PathRecordSet;
			}
			portno = next_nodep->lft[last_portp->portData->lid];
			if (portno == 255) {
				/* PR#101984 - no path from this node for given Lid */
				IB_LOG_WARN_FMT("sa_PathRecord_Set",
				"Cannot find path to node "FMT_U64" from node "FMT_U64": LFT entry for destination is 255 from switch %s (guid "FMT_U64")",
				dst_portp->portData->guid, src_portp->portData->guid, sm_nodeDescString(next_nodep), next_nodep->nodeInfo.NodeGUID);
				status = VSTATUS_BAD;
				goto done_PathRecordSet;
			}
			next_portp = sm_get_port(next_nodep,portno);
			if (!sm_valid_port(next_portp) || next_portp->state < IB_PORT_ACTIVE) {
				// PR#103535 - data corruption results in invalid topology
				IB_LOG_WARN_FMT("sa_PathRecord_Set",
					   "Cannot find path to destination port "FMT_U64" from source port "FMT_U64"; INVALID TOPOLOGY, next_portp[%d] state=%d (NOT ACTIVE=4)", 
					   dst_portp->portData->guid, src_portp->portData->guid, portno, (next_portp) ? next_portp->state : -1);
				status = VSTATUS_BAD;
				goto done_PathRecordSet;
			}
			next_nodep = sm_find_node(&old_topology, next_portp->nodeno);

			if ( (mtu = Min(mtu, next_portp->portData->maxVlMtu)) < IB_MTU_2048)
				break;
			rated = linkWidthToRate(next_portp->portData);
			if (linkrate_gt(rate,rated)) rate=rated;
		}
	}

	if (mtu < IB_MTU_2048) {
		IB_LOG_WARN_FMT(__func__, "Path to destination port "FMT_U64" from source port "FMT_U64" has invalid MTU < 2048. Offending node[port]: "
								"%s["FMT_U64"]", dst_portp->portData->guid, src_portp->portData->guid, (char*)next_nodep->nodeDesc.NodeString,
								next_portp->portData->guid);
		status = VSTATUS_BAD;
		goto done_PathRecordSet;
	}

	//
	// Get the path lids
	//
	(void)old_topology.routingModule->funcs.select_path_lids(&old_topology,
			src_portp, slid, dst_portp, dlid,
			srcLids, &srcLidLen, dstLids, &dstLidLen);

	// Check if valid
	if ((srcLidLen == 0) && (dstLidLen == 0)) {
		goto done_PathRecordSet;
	}

	//
	// Process VFs which specify a path
	//

	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	for (vf=bitset_find_first_one(&vfs); vf>=0;
		 vf=bitset_find_next_one(&vfs, vf+1)) {
			
		pkey = VirtualFabrics->v_fabric[vf].pkey;
		vfMtu = Min(mtu, VirtualFabrics->v_fabric[vf].max_mtu_int);
		vfRate = linkrate_gt(rate, VirtualFabrics->v_fabric[vf].max_rate_int) ?
						VirtualFabrics->v_fabric[vf].max_rate_int : rate;
		lifeMult = !VirtualFabrics->v_fabric[vf].pkt_lifetime_specified ? 0 :
						VirtualFabrics->v_fabric[vf].pkt_lifetime_mult;
		sl = VirtualFabrics->v_fabric[vf].base_sl;

		//
		// Check for other VFs sharing same pkey & sl (same path).
		// Adjust mtu/rate/pktLifetime accordingly.
		//
		for (vf2= bitset_find_next_one(&vfs, vf+1); vf2 >=0;
			 vf2= bitset_find_next_one(&vfs, vf2+1)) {

			if (PKEY_VALUE(pkey) != PKEY_VALUE(VirtualFabrics->v_fabric[vf2].pkey)) continue;

			if (sl != VirtualFabrics->v_fabric[vf2].base_sl) continue;

           	if (VirtualFabrics->v_fabric[vf2].max_mtu_int < vfMtu) {
				vfMtu = VirtualFabrics->v_fabric[vf2].max_mtu_int;
			}		
			if (linkrate_gt(vfRate, VirtualFabrics->v_fabric[vf2].max_rate_int)) {
				vfRate = VirtualFabrics->v_fabric[vf2].max_rate_int;
			}
			if (VirtualFabrics->v_fabric[vf2].pkt_lifetime_specified &&
				lifeMult < VirtualFabrics->v_fabric[vf2].pkt_lifetime_mult) {
				lifeMult = VirtualFabrics->v_fabric[vf2].pkt_lifetime_mult;
			}

			// Clear vf so dual path not reported.
			bitset_clear(&vfs, vf2);
		}

		//
		//	Iterate over lid pairs (multiple pairs for lmc > 0).
		//	
		if (slid == PERMISSIVE_LID && dlid == PERMISSIVE_LID) {
			for (lid_iterator_init(&iter, src_portp, srcLids[0], srcLidLen,
					dst_portp, dstLids[0], dstLidLen,
					sm_config.path_selection, &slid_iter, &dlid_iter);
				! lid_iterator_done(&iter);
				lid_iterator_next(&iter, &slid_iter, &dlid_iter)) {
	
				sa_FillPathRecord(query, records, src_portp, slid_iter, dst_portp, dlid_iter, 
								pkey, vfMtu, vfRate, lifeMult, hopCount, serviceId, sl);
				if ((*records) >= sa_max_path_records) {
            		IB_LOG_WARN("sa_PathRecord_Set: too many records:", (*records));
            		goto done_PathRecordSet;
        		}
				if (numPath && (*records) >= numPath) {
					goto done_PathRecordSet;
				}
			}
	
		} else if (slid == PERMISSIVE_LID) {
			for (lid_iterator_init1(&iter, dst_portp, dlid, dstLids[0], dstLidLen,
					src_portp, srcLids[0], srcLidLen,
					sm_config.path_selection, &slid_iter);
				! lid_iterator_done1(&iter);
				lid_iterator_next1(&iter, &slid_iter)) {
				sa_FillPathRecord(query, records, src_portp, slid_iter, dst_portp, dlid, 
								pkey, vfMtu, vfRate, lifeMult, hopCount, serviceId, sl);
				if ((*records) >= sa_max_path_records) {
            		IB_LOG_WARN("sa_PathRecord_Set: too many records:", (*records));
            		goto done_PathRecordSet;
        		}
				if (numPath && (*records) >= numPath) {
					goto done_PathRecordSet;
				}
			}
	
		} else if (dlid == PERMISSIVE_LID) {
			for (lid_iterator_init1(&iter, src_portp, slid, srcLids[0], srcLidLen,
					dst_portp, dstLids[0], dstLidLen,
					sm_config.path_selection, &dlid_iter);
				! lid_iterator_done1(&iter);
				lid_iterator_next1(&iter, &dlid_iter)) {
				sa_FillPathRecord(query, records, src_portp, slid, dst_portp, dlid_iter, 
								pkey, vfMtu, vfRate, lifeMult, hopCount, serviceId, sl);
				if ((*records) >= sa_max_path_records) {
            		IB_LOG_WARN("sa_PathRecord_Set: too many records:", (*records));
            		goto done_PathRecordSet;
        		}
				if (numPath && (*records) >= numPath) {
					goto done_PathRecordSet;
				}
			}
	
		} else {  // LID to LID
			sa_FillPathRecord(query, records, src_portp, slid, dst_portp, dlid,
						pkey, vfMtu, vfRate, lifeMult, hopCount, serviceId, sl);
			if ((*records) >= sa_max_path_records) {
            	IB_LOG_WARN("sa_PathRecord_Set: too many records:", (*records));
            	goto done_PathRecordSet;
        	}
			if (numPath && (*records) >= numPath) {
				goto done_PathRecordSet;
			}
		}
	}

done_PathRecordSet:
	bitset_free(&vfs);
	IB_EXIT("sa_PathRecord_Set", status);
	return status;
}

//
//	This whole routine is a quick short circuit for interoperability
//	until general MTU and RATES are available.
//
Status_t
sa_PathRecord_Interop(IB_PATH_RECORD *prp, uint64_t mask) {	// JSY - interop fix
	uint8_t		value;
	uint8_t		selector;

	IB_ENTER("sa_PathRecord_Interop", prp, &mask, 0, 0);

//
//	Does the MTU match what we have.
//
	if ((mask & PR_COMPONENTMASK_MTU_SEL) != 0ull) {
		selector = prp->MtuSelector;
		if (selector < PR_MAX && (mask & PR_COMPONENTMASK_MTU)) {
			value = prp->Mtu;
			if (value == 0 || value > STL_MTU_MAX) {
				IB_LOG_WARN("sa_PathRecord_Interop: invalid specified MTU value:", value);
				IB_EXIT("sa_PathRecord_Interop", VSTATUS_BAD);
				return(VSTATUS_BAD);
			} else if (value == STL_MTU_MAX) {
				if (selector == PR_GT) {
					IB_LOG_WARN("sa_PathRecord_Interop: invalid selector with 10K MTU value:", selector);
					IB_EXIT("sa_PathRecord_Interop", VSTATUS_BAD);
					return(VSTATUS_BAD);
				}
			} else if (value == IB_MTU_256) {
				if (selector < PR_LT) {
					IB_LOG_WARN("sa_PathRecord_Interop: invalid selector with 256 MTU value:", selector);
					IB_EXIT("sa_PathRecord_Interop", VSTATUS_BAD);
					return(VSTATUS_BAD);
				}
			}
		} /* MTU selector not max */
	}
//
//	Does the RATE match what we have.
//
	if ((mask & PR_COMPONENTMASK_RATE_SEL) != 0ull) {
		selector = prp->RateSelector;
		if (selector < PR_MAX && (mask & PR_COMPONENTMASK_RATE)) {
			value = prp->Rate;
			if (value < IB_STATIC_RATE_MIN || value > IB_STATIC_RATE_MAX) {
				IB_LOG_WARN("sa_PathRecord_Interop: invalid specified RATE value:", value);
				IB_EXIT("sa_PathRecord_Interop", VSTATUS_BAD);
				return(VSTATUS_BAD);
			} else if (value == IB_STATIC_RATE_MAX) {
				if (selector == PR_GT) {
					IB_LOG_WARN("sa_PathRecord_Interop: invalid selector with 30GPS rate value:", selector);
					IB_EXIT("sa_PathRecord_Interop", VSTATUS_BAD);
					return(VSTATUS_BAD);
				}
			} else if (value == IB_STATIC_RATE_MIN) {
				if (selector == PR_LT) {
					IB_LOG_WARN("sa_PathRecord_Interop: invalid selector with 2.5GPS rate value:", selector);
					IB_EXIT("sa_PathRecord_Interop", VSTATUS_BAD);
					return(VSTATUS_BAD);
				}
			}
		} /* rate selector not max */
	}

	IB_EXIT("sa_PathRecord_Interop", VSTATUS_OK);
	return(VSTATUS_OK);
}

