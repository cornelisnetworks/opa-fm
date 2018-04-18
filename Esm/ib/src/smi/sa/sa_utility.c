/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_utility.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the miscellaneous SA routines.  These include the   //
//    Multi-Mad routine and the access checking routines.		     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
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
#include "ib_macros.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sa_m.h"
#include "iba/stl_sa_priv.h"
#ifdef __VXWORKS__
#include "bspcommon/h/usrBootManager.h"
#include "BootCfgMgr.h"
#include "errnoLib.h"
#include "tms/common/miscLib.h"
#endif

#ifdef __VXWORKS__
uint32_t idbGetSmDefMcGrpPKey();
uint32_t idbGetSmDefMcGrpMtu();
uint32_t idbGetSmDefMcGrpQKey();
uint32_t idbGetSmDefMcGrpRate();
uint32_t idbGetSmDefMcGrpSl();
uint32_t idbGetSmDefMcGrpFlowLabel();
uint32_t idbGetSmDefMcGrpTClass();

uint32_t idbSetSmDefMcGrpPKey(uint32_t);
uint32_t idbSetSmDefMcGrpMtu(uint32_t);
uint32_t idbSetSmDefMcGrpQKey(uint32_t);
uint32_t idbSetSmDefMcGrpRate(uint32_t);
uint32_t idbSetSmDefMcGrpSl(uint32_t);
uint32_t idbSetSmDefMcGrpFlowLabel(uint32_t);
uint32_t idbSetSmDefMcGrpTClass(uint32_t);
#endif

extern	STL_CLASS_PORT_INFO	saClassPortInfo;
Status_t    sa_receive_getmulti(Mai_t *maip, sa_cntxt_t* sa_cntxt);
extern int createBroadcastGroup(uint16_t pkey, uint8_t mtu, uint8_t rate, uint8_t sl, uint32_t qkey, uint32_t fl, uint8_t tc);
extern Status_t createMCastGroup(uint64_t*, uint16_t, uint8_t, uint8_t, uint8_t, uint32_t, uint32_t, uint8_t);
extern Status_t	createMCastGroups(int, uint16_t, uint8_t, uint8_t, uint8_t, uint32_t, uint32_t, uint8_t); 

uint8_t		nullData[256];				// JSY - temp fix

char *sa_getMethodText(int method) {
    switch (method) {
    case (0x1):           // Get
        return "GET";
        break;
    case (0x2):           // Set
        return "SET";
        break;
    case (0x3):           // Send
        return "SEND";
        break;
    case (0x5):        // Trap
        return "TRAP";
        break;
    case (0x6):        // Report
        return "REPORT";
        break;
    case (0x7):        // TrapRepress
        return "TRAPREPRESS";
        break;
    case (0x12):        // SA_CM_GETTABLE
        return "GETTABLE";
        break;
    case (0x13):        // SA_CM_GETTRACETABLE
        return "GETTRACETABLE";
        break;
    case (0x14):        // SA_CM_GETMULTI
        return "GETMULTI";
        break;
    case (0x15):        // SA_CM_DELETE
        return "DELETE";
        break;
    case (0x81):        // GetResp
        return "GETRESP";
        break;
    case (0x86):        // ReportResp
        return "REPORTRESP";
        break;
    case (0x92):        // SA_CM_GETTABLE_RESP
        return "GETTABLERESP";
        break;
    case (0x94):        // SA_CM_GETMULTI_RESP
        return "GETMULTIRESP";
        break;
    case (0x95):        // SA_CM_DELETE_RESP
        return "DELETERESP";
        break;
    default:
        return "UNKNOWN METHOD";
        break;
    }
}

Status_t
sa_validate_mad(Mai_t *maip) {
    Status_t	rc = VSTATUS_OK;

	IB_ENTER("sa_validate_mad", maip, 0, 0, 0);

	if (maip->base.bversion != IB_BASE_VERSION &&
		maip->base.bversion != STL_BASE_VERSION) {
        IB_LOG_WARN_FMT( "sa_validate_mad",
               "Invalid SA Base Version %d received in %s[%s] request from LID [0x%x], TID ["FMT_U64"], ignoring request!",
               maip->base.bversion, sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
		rc = VSTATUS_BAD;
	} else if (maip->base.cversion != SA_MAD_CVERSION &&
			maip->base.cversion != STL_SA_CLASS_VERSION) {
            IB_LOG_WARN_FMT( "sa_validate_mad",
                   "Invalid SA Class Version %d received in %s[%s] request from LID [0x%x], TID ["FMT_U64"], ignoring request!",
                   maip->base.cversion, sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
		rc = VSTATUS_BAD;
    	} else if (maip->base.bversion == IB_BASE_VERSION &&
    		maip->base.cversion == STL_SA_CLASS_VERSION) {
        	IB_LOG_WARN_FMT( "sa_validate_mad",
               "Invalid SA Base Version %d - Class Version %d received in %s[%s] request from LID [0x%x], TID ["FMT_U64"], ignoring request!",
        		maip->base.bversion, maip->base.cversion, 
			sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), 
			maip->addrInfo.slid, maip->base.tid);
    		rc = VSTATUS_BAD;
    	} else {
           /*  Drop unsupported MADs */
    		switch (maip->base.method) {
    			case SA_CM_GET_RESP:
    			case SA_CM_GETTABLE_RESP:
        		case SA_CM_REPORT:
            			if (smDebugPerf || saDebugPerf) {
                			IB_LOG_INFINI_INFO_FMT( "sa_validate_mad",
                       			"Unsupported or invalid %s[%s] request from LID [0x%x], TID["FMT_U64"]", 
                       			sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
            			}
    				IB_EXIT("sa_validate_mad", VSTATUS_OK);
    				rc = VSTATUS_BAD;
            			break;
        		default:
            			break;
    		}
    	}

    	IB_EXIT("sa_validate_mad", rc);
	return(rc);
}

//
// Process only inflight rmpp requests
// Called by the sa_main_writer thread
//
Status_t
sa_process_inflight_rmpp_request(Mai_t *maip, sa_cntxt_t* sa_cntxt) {

	IB_ENTER("sa_process_inflight_rmpp_request", maip, sa_cntxt, 0, 0);
    //
    //	Process only inflight rmpp requests 
    //  ACKs of inprogress GETMULTI requests also come here
    //
    if (sa_cntxt->hashed  && !sa_cntxt->reqInProg) {
		sa_send_reply( maip, sa_cntxt );
	} else {
        IB_LOG_INFINI_INFO_FMT( "sa_process_inflight_rmpp_request",
               "SA_WRITER received %s[%s] RMPP packet from LID [0x%x] TID ["FMT_U64"] after transaction completion", 
               sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
    }
    IB_EXIT("sa_process_inflight_rmpp_request", VSTATUS_OK);
    return(VSTATUS_OK);
}


Status_t
sa_process_getmulti(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
    IB_ENTER("sa_process_getmulti", maip, sa_cntxt, 0, 0);

    /* Validate the MAD we received.  If it is not valid, just drop it. */
    if (sa_validate_mad(maip) != VSTATUS_OK) {
        IB_EXIT("sa_process_getmulti", VSTATUS_OK);
        return(VSTATUS_OK);
    }

    INCREMENT_COUNTER(smCounterSaRxGetMultiPathRecord);

    /* incoming multi-packet GETMULTI request */
    sa_receive_getmulti(maip, sa_cntxt);
    /* the request is fully received and ready to process when isDs is set and reqInProg=0 */
    if (sa_cntxt->isDS && !sa_cntxt->reqInProg) {
        /* release the context that we had reserved during the receive */
        sa_cntxt_release( sa_cntxt );
        /* get the request to processing routine */
        sa_cntxt->processFunc((Mai_t *)&sa_cntxt->mad, sa_cntxt);
        /* switch the mai_handle to writer thread's now that we are sending rmpp response */
        sa_cntxt->sendFd = fd_sa_writer;
    }

    IB_EXIT("sa_process_getmulti", VSTATUS_OK);
    return(VSTATUS_OK);
} /* end sa_process_getmulti */


Status_t
sa_process_mad(Mai_t *maip, sa_cntxt_t* sa_cntxt) {

    uint64_t startTime=0, endTime=0;

	IB_ENTER("sa_process_mad", maip, sa_cntxt, 0, 0);

    /* performance timestamp */
    if (saDebugPerf) (void) vs_time_get (&startTime);

    /*
     * Validate the MAD we received.  If it is not valid, just drop it.
     */
	if (sa_validate_mad(maip) != VSTATUS_OK) {
		IB_EXIT("sa_process_mad", VSTATUS_OK);
		return(VSTATUS_OK);
	}

    /* use sa reader mai handle for sending out 1st packet of responses */
    sa_cntxt->sendFd = fd_sa;

    /*
     * Since we have validated this MAD, we can now process it in an attribute specific way.
     */
	(void)sa_data_offset(maip->base.cversion, maip->base.aid);

#if 0
// make "#if 1" to assist in local SA debugging
        fprintf(stdout, "sa_process_mad: %s[%s] request from LID [0x%x], TID["FMT_U64"]\n", 
               sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
        fflush(stdout);
#endif
	switch (maip->base.aid) {
	case SA_CLASSPORTINFO:
		(void)sa_ClassPortInfo(maip, sa_cntxt);
		break;
	case SA_INFORMINFO:
		(void)sa_InformInfo(maip, sa_cntxt);
		break;
	case SA_INFORM_RECORD:
		(void)sa_InformRecord(maip, sa_cntxt);
		break;
	case SA_LFT_RECORD:
		(void)sa_LFTableRecord(maip, sa_cntxt);
		break;
	case SA_LINK_RECORD:
		(void)sa_LinkRecord(maip, sa_cntxt);
		break;
	case SA_MCMEMBER_RECORD:
		(void)sa_McMemberRecord(maip, sa_cntxt);
		break;
	case SA_MFT_RECORD:
		(void)sa_MFTableRecord(maip, sa_cntxt);
		break;
	case SA_NODE_RECORD:
		(void)sa_NodeRecord(maip, sa_cntxt);
		break;
	case SA_PARTITION_RECORD:
		(void)sa_PartitionRecord(maip, sa_cntxt);
		break;
	case SA_PATH_RECORD:
		(void)sa_PathRecord(maip, sa_cntxt);
		break;
	case SA_PORTINFO_RECORD:
		(void)sa_PortInfoRecord(maip, sa_cntxt);
		break;
	case SA_SERVICE_RECORD:
		(void)sa_ServiceRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_SC_MAPTBL_RECORD:
		(void)sa_SCSCTableRecord(maip, sa_cntxt);
		break;
    case STL_SA_ATTR_SL2SC_MAPTBL_RECORD:
         (void)sa_SLSCTableRecord(maip, sa_cntxt);
          break;
    case STL_SA_ATTR_SC2SL_MAPTBL_RECORD:
         (void)sa_SCSLTableRecord(maip, sa_cntxt);
          break;
	case STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD:
	case STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD:
	case STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD:
		(void)sa_SCVLTableRecord(maip, sa_cntxt);
		break;
	case SA_SMINFO_RECORD:
		(void)sa_SMInfoRecord(maip, sa_cntxt);
		break;
	case SA_SWITCH_RECORD:
		(void)sa_SwitchInfoRecord(maip, sa_cntxt);
		break;
	case SA_VLARBITRATION_RECORD:
		(void)sa_VLArbitrationRecord(maip, sa_cntxt);
		break;
    case SA_TRACE_RECORD:
		(void)sa_TraceRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_VF_INFO_RECORD:
    case SA_VFABRIC_RECORD:
		(void)sa_VFabricRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_FABRICINFO_RECORD:
		(void)sa_FabricInfoRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_QUARANTINED_NODE_RECORD:
		(void)sa_QuarantinedNodeRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_CONGESTION_INFO_RECORD:
		(void)sa_CongInfoRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_SWITCH_CONG_RECORD:
		(void)sa_SwitchCongRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_SWITCH_PORT_CONG_RECORD:
		(void)sa_SwitchPortCongRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_HFI_CONG_RECORD:
		(void)sa_HFICongRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_HFI_CONG_CTRL_RECORD:
		(void)sa_HFICongCtrlRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_BUFF_CTRL_TAB_RECORD:
		(void)sa_BufferControlTableRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_CABLE_INFO_RECORD:
		(void)sa_CableInfoRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_PORTGROUP_TABLE_RECORD:
		sa_PortGroupRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_PGROUP_FWDTBL_RECORD:
		sa_PortGroupFwdRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_DG_NAME_RECORD:
		(void)sa_DeviceGroupNameRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_DG_MEMBER_RECORD:
		(void)sa_DeviceGroupMemberRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_DT_MEMBER_RECORD:
		(void)sa_DeviceTreeMemberRecord(maip, sa_cntxt);
		break;
	case STL_SA_ATTR_SWITCH_COST_RECORD:
		(void)sa_SwitchCostRecord(maip, sa_cntxt);
		break;
    default:
        IB_LOG_INFINI_INFO_FMT( "sa_process_mad",
               "Unsupported or invalid %s[%s] request from LID [0x%x], TID["FMT_U64"]", 
               sa_getMethodText((int)maip->base.method), sa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		sa_cntxt_data( sa_cntxt, sa_data, 0 );
		(void)sa_send_reply(maip, sa_cntxt);
        IB_EXIT("sa_process_mad", VSTATUS_OK);
        return(VSTATUS_OK);
	}

    /* 
     * switch to sa writer mai handle for sending out remainder of rmpp responses
     * Mutipath request are set after complete receipt of requests in sa_process_getmulti
     */
    if (maip->base.aid != SA_MULTIPATH_RECORD) sa_cntxt->sendFd = fd_sa_writer;

    if (saDebugPerf) {
        /* use the time received from umadt as start time if available */
        startTime = (maip->intime) ? maip->intime : startTime;
        /* lids have been swapped, so use dlid here */
        (void) vs_time_get (&endTime);
        IB_LOG_INFINI_INFO_FMT( "sa_process_mad", 
               "%ld microseconds to process %s[%s] request from LID 0x%.8X, TID="FMT_U64,
               (long)(endTime - startTime), sa_getMethodText((int)sa_cntxt->method), 
               sa_getAidName(maip->base.aid), maip->addrInfo.dlid, maip->base.tid);
    }
	IB_EXIT("sa_process_mad", VSTATUS_OK);
	return(VSTATUS_OK);
}

// -------------------------------------------------------------------------- /

Status_t
sa_send_reply(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint8_t		method;
	STL_LID	lid;

	IB_ENTER("sa_send_reply", maip, sa_cntxt, 0, 0);

//
//	Setup the out-bound MAD.  We need to reverse the LRH addresses.
//

    // If the NoReplyIfBusy XML file option is set to 1 and the MAD request has caused
    // a MAD_STATUS_BUSY status to be generated, the return packet will be discarded.
    if (!(sm_config.NoReplyIfBusy &&  maip && (maip->base.status == MAD_STATUS_BUSY))) {
        if( maip )  {
			/* the fact the SA received the request implies its a member or */
			/* limited member of this pkey */
			/* Always use the best possible pkey for responses */
			/* In most cases only 1 pkey with the given low 15 bits will */
			/* be in our SMA's table and the lower layers will use that one */
			/* In some cases both 0x7fff and 0xffff will be in our table */
			/* and we will want to use the 0xffff pkey */
			maip->addrInfo.pkey |= FULL_MEMBER;

			lid = maip->addrInfo.slid;
            maip->addrInfo.slid = maip->addrInfo.dlid;
            maip->addrInfo.dlid = lid;
            if (maip->addrInfo.sl != sm_masterSmSl) {
                if (sm_config.sm_debug_vf) {
                    IB_LOG_INFINI_INFO_FMT("sa_send_reply",
                       "Bad SA SL (SL%d) in request from LID 0x%x, setting to SL%d in reply", maip->addrInfo.sl, lid, sm_masterSmSl);
                }
                maip->addrInfo.sl = sm_masterSmSl;
            }
            method = maip->base.method;
            if (sa_cntxt) sa_cntxt->method = method;
            if (method <= SA_CM_SET) {
                /* Get & Set -> GetResp */
                maip->base.method = SA_CM_GET_RESP;
            } else if (method == SA_CM_GETTRACETABLE) {
                maip->base.method = SA_CM_GETTABLE_RESP;
            } else {
                /* everybody else, OR in 0x80 to inbound; i.e., 14->94 */
                maip->base.method |= MAD_CM_REPLY;
            }
        } else {
            /* This is the timeout case */
            method = sa_cntxt->method;
        }


//
//  we need to always send rmpp responses to gettable/getmulti/gettracetable
//
        if (sa_cntxt && (method == SA_CM_GETTABLE || method == SA_CM_GETMULTI || method == SA_CM_GETTRACETABLE)) {
            //
            // As per Table 154, page 784 of spec, can only return this error in response to a GET/SET
            // for all other requests, we simply return a 1 segment rmpp response with header only 
            // additional spec references relating to handling of rmpp responses are (C15-0.1.19, C15-0.1.29)
            //
            if (maip && 
                (maip->base.method == SA_CM_GETTABLE_RESP || maip->base.method == SA_CM_GETMULTI_RESP) &&
                (maip->base.status == MAD_STATUS_SA_NO_RECORDS)) {
                maip->base.status = MAD_STATUS_SA_NO_ERROR;
            }
            if (sa_cntxt->len > sa_data_length) {
                sa_cntxt->len = 0;
                if (maip) {
                    IB_LOG_WARN_FMT("sa_send_reply", 
                       "sa_cntxt->len[%d] too large, returning no resources error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!",
                       sa_cntxt->len, sa_getMethodText((int)method), sa_getAidName((int)maip->base.aid), sa_cntxt->lid, sa_cntxt->tid);
                    maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
                } else {
                    IB_LOG_WARN_FMT("sa_send_reply", 
                       "sa_cntxt->len[%d] too large, returning no resources error to caller to %s request from LID [0x%x], TID ["FMT_U64"]!",
                       sa_cntxt->len, sa_getMethodText((int)method), sa_cntxt->lid, sa_cntxt->tid);
                }
            }
            (void)sa_send_multi(maip, sa_cntxt);
            IB_EXIT("sa_send_reply", VSTATUS_OK);
            return(VSTATUS_OK);
        }
        // Klocwork found an error that if maip is null, it will be dereferenced by the sa_send_single (and we'll crash)
        // Logically - by context in higher fn calls - this will never happen.
        // This fn (sa_send_reply) is called with a NULL pointer for an error recovery, and 
        //    such error recovery is only done for GET TABLE, GET MULTI, and would be handled in the clause above.
        // However - to keep klocwork happy and to add a level of robustnesss, check / log error if we get here
        //  with a NULL pointer. (as in a single SA GET ).
        if (maip) {
            (void)sa_send_single(maip, sa_cntxt); 
        } else {
            if (sa_cntxt) {
                IB_LOG_WARN_FMT("sa_send_reply", 
                   "%s request from LID [0x%x], TID ["FMT_U64"] has invalid mai context",
                   sa_getMethodText((int)method), sa_cntxt->lid, sa_cntxt->tid);
            } else {
                IB_LOG_WARN_FMT("sa_send_reply", 
                   "%s request has invalid mai context, sa_cntxt", 
                   sa_getMethodText((int)method));
            }
        }
    }
	IB_EXIT("sa_send_reply", VSTATUS_OK);
	return(VSTATUS_OK);
}


// -------------------------------------------------------------------------- /
// resend ACK for last segment that was acked in context
//--------------------------------------------------------------------------- /
Status_t sa_getMulti_resend_ack(sa_cntxt_t *sa_cntxt) {
    Mai_t       mad;
	STL_SA_MAD	samad;
    STL_LID    lid;
    Status_t    status=VSTATUS_OK;
    /* set the mai handle to use for sending - use reader handle fd_sa if no context */
    IBhandle_t  fd = (sa_cntxt->sendFd) ? sa_cntxt->sendFd : fd_sa;

    // get input mad from context
    memcpy((void *)&mad, (void *)&sa_cntxt->mad, sizeof(Mai_t));
    BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)sa_cntxt->mad.data, &samad, STL_SA_DATA_LEN);
    samad.header.rmppType = RMPP_TYPE_ACK;
    samad.header.length = sa_cntxt->last_ack;
    samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
    /* set the desired response time value for client to use */
    if (sa_cntxt->last_ack == 1)
        samad.header.u.tf.rmppRespTime = saClassPortInfo.u1.s.RespTimeValue;    // just do it once
    else
        samad.header.u.tf.rmppRespTime = 0x1f;     // none supplied
    samad.header.rmppStatus = 0;
    samad.header.segNum = sa_cntxt->last_ack;
    // reset segment expected
    if (sa_cntxt->segTotal == sa_cntxt->last_ack) {
        sa_cntxt->ES = sa_cntxt->last_ack;
    } else {
        sa_cntxt->ES = sa_cntxt->last_ack + 1;
    }
    samad.header.length = sa_cntxt->ES;
    ++sa_cntxt->retries;

    /* We are timing out, retry till retry count expires */
    if( sa_cntxt->retries > sm_config.max_retries ) {
        if (saDebugPerf || saDebugRmpp) {
            IB_LOG_WARN_FMT( "sa_getMulti_ack", 
                   "ABORT; Never received getMulti Direction Switch ACK from Lid [0x%X] for TID="FMT_U64,
               sa_cntxt->lid, sa_cntxt->tid);
        }
		INCREMENT_COUNTER(smCounterRmppStatusAbortTooManyRetries);
        samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
        samad.header.u.tf.rmppRespTime = 0;
        samad.header.rmppStatus = RMPP_STATUS_ABORT_TOO_MANY_RETRIES;
        samad.header.segNum = 0;
        samad.header.length = 0;
    } else {
		INCREMENT_COUNTER(smCounterSaTxGetMultiAckRetries);
        if (saDebugPerf || saDebugRmpp) {
            IB_LOG_WARN_FMT( "sa_getMulti_ack", "Timed out waiting for getMulti Direction Switch ACK from Lid[%d] for TID="FMT_U64", RETRYING COUNT [%d]",
                   sa_cntxt->lid, sa_cntxt->tid, sa_cntxt->retries);
        }
    }
    /* send the ACK */
    lid = mad.addrInfo.slid;
    mad.addrInfo.slid = mad.addrInfo.dlid;
    mad.addrInfo.dlid = lid;
    mad.base.method |= MAD_CM_REPLY;  // send as response
    BSWAPCOPY_STL_SA_MAD(&samad, (STL_SA_MAD*)mad.data, STL_SA_DATA_LEN);
    if ((status = mai_send(fd, &mad)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT( "sa_getMulti_ack", 
               "error from mai_send while sending ACK to LID [0x%x] for getMulti TID["FMT_U64"]",
                       sa_cntxt->lid, sa_cntxt->tid);
    } 
    
    IB_EXIT( "sa_getMulti_ack", status );
    return status;
}


/*
 * send ack packet
*/
static Status_t send_ack(Mai_t *maip, STL_SA_MAD *samad, sa_cntxt_t* sa_cntxt, uint16_t wsize) {
    Status_t    rc=VSTATUS_OK;
    STL_LID    lid;
    /* set the mai handle to use for sending - use reader handle fd_sa if no context */
    IBhandle_t  fd = (sa_cntxt->sendFd) ? sa_cntxt->sendFd : fd_sa;

    samad->header.rmppType = RMPP_TYPE_ACK;
    /* set NewWindowLast (next ACK) */
    if (samad->header.u.tf.rmppFlags & RMPP_FLAGS_LAST) {
        /* last segment, leave ack window alone */
        samad->header.length = samad->header.segNum;
    } else if (wsize == 1) {
        samad->header.length = samad->header.segNum + 1;
    } else {
        /* increment ack window */
        samad->header.length = Min(((samad->header.segNum == 1) ? wsize : (samad->header.segNum + wsize)), sa_cntxt->segTotal);
    }
    samad->header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
    if (saDebugRmpp) {
        IB_LOG_INFINI_INFO_FMT( "send_ack", 
               "ACK window[%d] reached for LID[0x%x] TID="FMT_U64", New ACK window[%d]",
               samad->header.segNum, (int)maip->addrInfo.slid, maip->base.tid, samad->header.length);
    }
    /* set the desired response time value for client to use */
    if (samad->header.segNum == 1)
        samad->header.u.tf.rmppRespTime = saClassPortInfo.u1.s.RespTimeValue;    // just do it once
    else
        samad->header.u.tf.rmppRespTime = 0x1f;     // none supplied

    /* flip the source and dest lids before sending */
    samad->header.rmppStatus = 0;
    lid = maip->addrInfo.slid;
    maip->addrInfo.slid = maip->addrInfo.dlid;
    maip->addrInfo.dlid = lid;
    maip->base.method |= MAD_CM_REPLY;  // send as response
    BSWAPCOPY_STL_SA_MAD(samad, (STL_SA_MAD*)maip->data, STL_SA_DATA_LEN);
    if ((rc = mai_send(fd, maip)) == VSTATUS_OK) {
        /* update lastSegAcked */
        sa_cntxt->last_ack = samad->header.segNum;
    }
    IB_EXIT("send_ack",rc);
    return rc;
} /* end send_ack */


// -------------------------------------------------------------------------- /
Status_t
sa_receive_getmulti(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	STL_SA_MAD	samad;
    uint16_t    bytesRcvd=0, badPayloadLen=0;
    uint16_t    sendStopAbort=0;    // 2=ABORT, 1=STOP
	uint16_t	wsize=1;            // ACK window of 1
    STL_LID    lid;
    uint32_t    len;
    Status_t    rc=VSTATUS_OK;
    /* set the mai handle to use for sending - use reader handle fd_sa if no context */
    IBhandle_t  fd = (sa_cntxt->sendFd) ? sa_cntxt->sendFd : fd_sa;

	IB_ENTER("sa_receive_getmulti", maip, sa_cntxt, 0, 0);

	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, STL_SA_DATA_LEN);

    /* check for possible non-rmpp GetMulti request */
    if ( samad.header.rmppType == RMPP_TYPE_NOT && sa_cntxt->hashed == 0) {
        /* single packet non RMPP request */
        sa_cntxt->method = maip->base.method;
        sa_cntxt->segTotal = 1;
        sa_cntxt->reqInProg = 0;
        memcpy( &sa_cntxt->mad, maip, sizeof( Mai_t ));
        sa_cntxt->reqDataLen = STL_SA_DATA_LEN;
        /* allocate the space for the request */
        rc = vs_pool_alloc( &sm_pool, sa_cntxt->reqDataLen, (void*)&sa_cntxt->reqData );
        if (!rc) {
            /* move STL_SA_DATA_LEN of data from first packet into buffer */
            memcpy(sa_cntxt->reqData, samad.data, STL_SA_DATA_LEN);
            /* get the request to processing routine */
            rc = sa_cntxt->processFunc((Mai_t *)&sa_cntxt->mad, sa_cntxt);
            if (saDebugRmpp) {
                IB_LOG_INFINI_INFO_FMT( "sa_receive_getmulti", 
                       "Non RMPP GetMulti from Lid[%d], TID="FMT_U64" processed",
                       (int)maip->addrInfo.slid, sa_cntxt->tid);
				INCREMENT_COUNTER(smCounterSaGetMultiNonRmpp);
            }
        } else {
            IB_LOG_ERRORX("sa_receive_getmulti: couldn't allocate resources for Non RMPP GetMulti request from LID:", sa_cntxt->lid);
        }
        IB_EXIT( "sa_receive_getmulti", rc );
        return rc ;
    } else {
        /*
         * Must be an RMPP request at this point, do the basic RMPP validation of header fields.
         */
        if (samad.header.rmppType == RMPP_TYPE_NOT) {
            // invalid RMPP type
            IB_LOG_WARN_FMT( "sa_receive_getmulti", "ABORTING - RMPP protocol error; type is NULL from Lid[%d] for TID="FMT_U64,
                   (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        } else if (samad.header.rmppVersion != RMPP_VERSION) {
            IB_LOG_WARN_FMT( "sa_receive_getmulti", "RMPP protocol error, received RMPP Version %d from LID[0x%x] for getMulti TID["FMT_U64"]",
                   (int)samad.header.rmppVersion, (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortUnsupportedVersion);
            samad.header.rmppStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION;
        } else if (!(samad.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            // invalid RMPP type
            IB_LOG_WARN_FMT( "sa_receive_getmulti", "RMPP protocol error, RMPPFlags.Active bit is NULL from LID[0x%x] for TID["FMT_U64"]",
                   (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        }
        if (samad.header.rmppStatus) {
            samad.header.rmppType = RMPP_TYPE_ABORT;
            sendStopAbort = 1;
            goto sendAbort;     //  just send the abort
        }
    }  
        
    /*
     * process the RMPP packet
     */
    if (samad.header.rmppType == RMPP_TYPE_STOP || samad.header.rmppType == RMPP_TYPE_ABORT) {
        /* got a STOP or ABORT */
        if (saDebugPerf || saDebugRmpp) {
            IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                   "Processing STOP OR ABORT with status code[0x%x] from LID[0x%x] for TID["FMT_U64"]",
               (int)samad.header.rmppStatus, (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterSaRxGetMultiInboundRmppAbort);
        }
        sa_cntxt_release( sa_cntxt );
        IB_EXIT( "sa_receive_getmulti", VSTATUS_OK );
        return VSTATUS_OK ;
    } else if (samad.header.rmppType == RMPP_TYPE_ACK) {
        // process ACK of segment zero from sender at end of request receipt indicating time to process request
        if (samad.header.segNum != 0) {
            // invalid ACK; must be segment zero
            IB_LOG_WARN_FMT( "sa_receive_getmulti", "ABORTING - Invalid segment number %d in ACK when expecting [0] from LID[0x%x] for TID["FMT_U64"]",
                   samad.header.segNum, (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortSegnumTooBig);
            samad.header.rmppType = RMPP_TYPE_ABORT;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG;
            sendStopAbort = 1;
        } else if (samad.header.length == 0) {
            // invalid window size
            IB_LOG_WARN_FMT( "sa_receive_getmulti", "ABORTING - A window size of zero was specified in ACK from LID[0x%x] for TID["FMT_U64"]",
                   (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortNewWindowLastTooSmall);
            samad.header.rmppType = RMPP_TYPE_ABORT;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL;
            sendStopAbort = 1;
        } else if (!sa_cntxt->reqInProg) {
            // ACK received before end of request
            IB_LOG_WARN_FMT( "sa_receive_getmulti", "ABORTING - ACK received before finished receiving request from LID[0x%x] for TID["FMT_U64"]",
                   (int)maip->addrInfo.slid, maip->base.tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortUnspecified);
            samad.header.rmppType = RMPP_TYPE_ABORT;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_UNSPECIFIED;
            sendStopAbort = 1;
        } else {
            /*
             * OK to process the request now
             * set DS bit, set Window Last to desired, and clear request in progress bit
             */
            sa_cntxt->isDS = 1;
            sa_cntxt->WL = samad.header.length;
            sa_cntxt->reqInProg = 0;
        }
    } else {
        // processing DATA packet
        if (saDebugRmpp) {
            IB_LOG_INFINI_INFO_FMT( "sa_receive_getmulti", 
                   "Processing RMPP GETMULTI request from Lid[%d], TID="FMT_U64,
                   (int)maip->addrInfo.slid, maip->base.tid);
        }
        if( sa_cntxt->hashed == 0 ) {
            /*
             * 	This is the start of request, init/save parameters for context entry
             */
            sa_cntxt->method = maip->base.method;
            sa_cntxt->WF = 1;               // window start/first
            sa_cntxt->WL = 1;               // window last/end - the one to ACK
            sa_cntxt->NS = 0;               // Next packet segment to send
            sa_cntxt->ES = 1;               // Expected segment number (Receiver only)
            sa_cntxt->last_ack = 0;         // last packet acked by receiver
            sa_cntxt->retries = 0;          // current retry count
            sa_cntxt->segTotal = 0;
            sa_cntxt->reqInProg = 1;
            sa_cntxt->sendFd = fd_sa;       // use reader mai handle for receiving request
            /* calculate packet and total transaction timeouts */
            sa_cntxt->RespTimeout = 4ull * ( (2*(1<<sm_config.sa_packet_lifetime_n2)) + (1<<saClassPortInfo.u1.s.RespTimeValue) );
            sa_cntxt->tTime = 0;            // for now just use max retries
            memcpy( &sa_cntxt->mad, maip, sizeof( Mai_t ));
            sa_cntxt_reserve( sa_cntxt );
            samad.header.rmppVersion = RMPP_VERSION;
            samad.header.u.tf.rmppRespTime = saClassPortInfo.u1.s.RespTimeValue; // use classPortInfo setting
            samad.header.offset = 0;
        } 
        /* see if this is the segment expected. */
        if (samad.header.segNum == sa_cntxt->ES) {
            if (samad.header.segNum == 1) {
                /* check to make sure that RMPPFlags.First is set */
                if (!(samad.header.u.tf.rmppFlags & RMPP_FLAGS_FIRST)) {
                    IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                           "ABORTING - invalid muli pkt first flag[%d] with segnum 1 from LID[0x%x] for TID["FMT_U64"]",
                           samad.header.u.tf.rmppFlags, (int)maip->addrInfo.slid, maip->base.tid);
					INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentFirstSegnum);
                    rc = RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM;
                    /*  Send an ABORT now */
                    samad.header.rmppType = RMPP_TYPE_ABORT;
                    samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM;
                    sendStopAbort = 1;
                } else {
                    /* extract the number of bytes in payload and calculate total segments */
                    len = samad.header.length;
                    if (len) {
                        // Handle condition where the length exceeds the data capacity.
                        if ((samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST) && (len > STL_SA_DATA_LEN)) {
                            /* send ABORT with status of inconsistent payloadLength */
                            IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                                   "ABORTING - First/Last segment received with length larger than payload capacity from LID[0x%x] for TID["FMT_U64"]",
                                   (int)maip->addrInfo.slid, maip->base.tid);
                            INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentLastPayloadLength);
                            rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
                            samad.header.rmppType = RMPP_TYPE_ABORT;
                            samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
                            sendStopAbort = 1;
                            goto sendAbort;
                        } else {
                            sa_cntxt->segTotal = (len + STL_SA_DATA_LEN + SA_HEADER_SIZE - 1)/ (STL_SA_DATA_LEN + SA_HEADER_SIZE);
                            sa_cntxt->reqDataLen = len - (sa_cntxt->segTotal * SA_HEADER_SIZE);
                            if (saDebugRmpp) {
                                IB_LOG_INFINI_INFO_FMT( "sa_receive_getmulti", 
                                       "GetMulti from LID[0x%x], TID="FMT_U64", %d total bytes, %d segments, %d data bytes",
                                       maip->addrInfo.slid, maip->base.tid, len, sa_cntxt->segTotal, sa_cntxt->reqDataLen);
                            }
                        }
                    } else {
                        if (samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST) {
                            /* send ABORT with status of inconsistent payloadLength */
                            IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                                   "ABORTING - First and Last segment received with no length from LID[0x%x] for TID["FMT_U64"]",
                                   (int)maip->addrInfo.slid, maip->base.tid);
							INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentLastPayloadLength);
                            rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
                            samad.header.rmppType = RMPP_TYPE_ABORT;
                            samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
                            sendStopAbort = 1;
                            goto sendAbort;
                        } else {
                            /* no payload length specified, use max 8184 for GetMulti */
                            sa_cntxt->reqDataLen = 8184;
                            sa_cntxt->segTotal = 0;
                            if (saDebugRmpp) {
                                IB_LOG_INFINI_INFO0("sa_receive_getmulti: PayloadLength is not specified, using 8184");
                            }
                        }
                    }

                    /* allocate the space for the request */
                    rc = vs_pool_alloc( &sm_pool, len, (void*)&sa_cntxt->reqData );
                    /* send STOP if unable to alloc resources */ 
                    if( rc != VSTATUS_OK ) {
                        sa_cntxt->reqData = NULL;
                        rc = RMPP_STATUS_STOP_NORESOURCES;
						INCREMENT_COUNTER(smCounterRmppStatusStopNoresources);
                        /*  Send a STOP now */
                        samad.header.rmppType = RMPP_TYPE_STOP;
                        samad.header.rmppStatus = RMPP_STATUS_STOP_NORESOURCES;
                        sendStopAbort = 1;
                    } else {
                        sa_cntxt->reqDataLen = len;
                        if (samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST) {
                            /* move len indicated of data from first/last packet into buffer */
                            memcpy(sa_cntxt->reqData + (samad.header.segNum - 1)*STL_SA_DATA_LEN, samad.data, MIN(len, STL_SA_DATA_LEN));
                        } else {
                            /* move STL_SA_DATA_LEN of data from first packet into buffer */
                            memcpy(sa_cntxt->reqData + (samad.header.segNum - 1)*STL_SA_DATA_LEN,
                                   samad.data, STL_SA_DATA_LEN);
                            /* increment next ES */
                            ++sa_cntxt->ES;
                        }
                    }
                } /* good first segment received */
            } else {
                /* handle middle/last segments */
                if ((samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST)) {
                    len = samad.header.length - SA_HEADER_SIZE;
                    /* We have last packet and the segment numbers line up. We are done */
                    if (saDebugRmpp) {
                        IB_LOG_INFINI_INFO_FMT( "sa_receive_getmulti", 
                               "GetMulti from LID[0x%x], TID="FMT_U64" has %d Total data bytes in last packet segment number %d",
                               maip->addrInfo.slid, maip->base.tid, len, samad.header.segNum);
                    }
                    /* abort if pass expected length and not at Last */
                    bytesRcvd = (samad.header.segNum - 1)*STL_SA_DATA_LEN + len;
                    if ((bytesRcvd > sa_cntxt->reqDataLen) || (len > STL_SA_DATA_LEN) ){
                        /* send ABORT with status of inconsistent payloadLength */
                        badPayloadLen = 1;
                    } else {
                        /* move len indicated of data from first/last packet into buffer */
                        memcpy(sa_cntxt->reqData + (samad.header.segNum - 1)*STL_SA_DATA_LEN, samad.data, len);
                    }
                } else {
                    /* abort if pass expected length and not at Last */
                    bytesRcvd = samad.header.segNum * STL_SA_DATA_LEN;
                    if (bytesRcvd > sa_cntxt->reqDataLen) {
                        /* send ABORT with status of inconsistent payloadLength */
                        badPayloadLen = 1;
                    } else {
                        /* move STL_SA_DATA_LEN of data from packet into buffer */
                        memcpy(sa_cntxt->reqData + (samad.header.segNum - 1)*STL_SA_DATA_LEN, samad.data, STL_SA_DATA_LEN);
                        /* increment next ES */
                        ++sa_cntxt->ES;
                    }
                } /* move received data into buffer */
                if (badPayloadLen) {
                    /* send ABORT with status of inconsistent payloadLength */
                    badPayloadLen = 0;
                    IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                           "ABORTING - data received [%d] inconsistent with payloadLength [%d] from LID[0x%x] for TID["FMT_U64"]",
                           bytesRcvd, sa_cntxt->reqDataLen, (int)maip->addrInfo.slid, maip->base.tid);
					INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentLastPayloadLength);
                    rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
                    samad.header.rmppType = RMPP_TYPE_ABORT;
                    samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
                    sendStopAbort = 1;
                }
            }  /* middle and last segments */

            /* ACK the first, last segments and window hits */
            if ( ((samad.header.segNum%wsize) == 0 || samad.header.segNum == 1 || samad.header.segNum == sa_cntxt->segTotal) && !sendStopAbort ) {
                if ((rc = send_ack(maip, &samad, sa_cntxt, wsize)) != VSTATUS_OK) {
                    IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                           "error %d while sending ACK to LID[0x%x] for TID["FMT_U64"], terminating transaction",
                           rc, (int)maip->addrInfo.dlid, maip->base.tid);
                    /* release the context, done with this RMPP xfer */
                    sa_cntxt_release( sa_cntxt );
                    IB_EXIT("sa_receive_getmulti",rc);
                    return rc;
                }
            }  /* ACK window reached */

        } else {
            /*
             * got segment number out of sequence
             * we can either not send ACK and sender should retransmit from WF to WL
             * or we send ack of (ES-1) as per figure 178 of receiver main flow diagram
             */
            if (saDebugPerf || saDebugRmpp) {
                IB_LOG_INFINI_INFO_FMT( "sa_receive_getmulti", 
                       "got seg[%d] when expecting seg[%d] from lID[0x%x] TID="FMT_U64,
                       samad.header.segNum, sa_cntxt->ES, (int)maip->addrInfo.slid, maip->base.tid);
            }
            /* resend ack of (ES-1) here */
            samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
            samad.header.segNum = sa_cntxt->ES - 1;
            if ((rc = send_ack(maip, &samad, sa_cntxt, wsize)) != VSTATUS_OK) {
                IB_LOG_WARN_FMT( "sa_receive_getmulti", 
                       "error %d while sending ACK to LID[0x%x] for TID["FMT_U64"]",
                       rc, (int)maip->addrInfo.dlid, maip->base.tid);
                /* release the context, done with this RMPP xfer */
                sa_cntxt_release( sa_cntxt );
                IB_EXIT("sa_receive_getmulti",rc);
                return rc;
            }
        }
    } /* good or bad RMPP packet */
    /*
     * Send Abort if desired
     */
    sendAbort:
    if (sendStopAbort) {
        /*
         *	Setup the out-bound MAD.  We need to reverse the LRH addresses.
         */
        lid = maip->addrInfo.slid;
        maip->addrInfo.slid = maip->addrInfo.dlid;
        maip->addrInfo.dlid = lid;
        maip->base.method |= MAD_CM_REPLY;  // send as response
        samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
        samad.header.u.timeFlag = 0;
        samad.header.segNum = 0;
        samad.header.length = 0;
        BSWAPCOPY_STL_SA_MAD(&samad, (STL_SA_MAD*)maip->data, STL_SA_DATA_LEN);

        if ((rc = mai_send(fd, maip)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT( "sa_receive_getmulti", 
                   "error from mai_send while sending ABORT to LID[0x%x] for TID["FMT_U64"]",
                   (int)maip->addrInfo.dlid, maip->base.tid);
        }
        /* release the context, done with this RMPP xfer */
        sa_cntxt_release( sa_cntxt );
    }

	IB_EXIT("sa_receive_getmulti", rc);
	return(rc);
}

// -------------------------------------------------------------------------- /

Status_t
sa_send_single(Mai_t *maip, sa_cntxt_t* sa_cntxt ) {
	Status_t	status;
    uint32_t    datalen = sizeof(SAMadh_t);
	STL_SA_MAD	samad;
    /* set the mai handle to use for sending - use reader handle fd_sa if no context */
    IBhandle_t  fd = (sa_cntxt && sa_cntxt->sendFd) ? sa_cntxt->sendFd : fd_sa;

	IB_ENTER("sa_send_single", maip, sa_cntxt, 0, 0);

//
//	Send the response MAD.
//
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, STL_SA_DATA_LEN);
    samad.header.rmppVersion = 0;
    samad.header.rmppType = 0;
    samad.header.u.timeFlag = 0;
    samad.header.rmppStatus = 0;
	samad.header.segNum = 0;
	samad.header.length = 0;

	(void)memset(samad.data, 0, sizeof(samad.data));
	if( ( sa_cntxt == NULL ) || ( sa_cntxt->data == NULL )) {
		if( maip->base.status == MAD_STATUS_OK )
			maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
	} else if (sa_cntxt->len != 0 ) {
		if( maip->base.status == MAD_STATUS_OK )
            samad.header.offset = sa_cntxt->attribLen / 8; // setup attribute offset for RMPP xfer
		(void)memcpy(samad.data, sa_cntxt->data, sa_cntxt->len);
        datalen += sa_cntxt->len;
	}

	INCREMENT_MAD_STATUS_COUNTERS(maip);

	BSWAPCOPY_STL_SA_MAD(&samad, (STL_SA_MAD*)maip->data, STL_SA_DATA_LEN);

	if (maip->base.bversion == IB_BASE_VERSION) {
#if 0
// make "#if 1" to assist in local SA debugging
            fprintf(stdout, "mai_send_single (ib base)\n");
#endif
		status = mai_send(fd, maip);
	} else {
		status = mai_stl_send(fd, maip, &datalen);
	}

	if (status != VSTATUS_OK) {
        IB_LOG_ERROR_FMT( "sa_send_single", 
               "can't send reply to %s request to LID[0x%x] for TID["FMT_U64"]",
               sa_getAidName(maip->base.aid), (int)maip->addrInfo.dlid, maip->base.tid);
		IB_EXIT("sa_send_single", VSTATUS_OK);
		return(VSTATUS_OK);
	}

	IB_EXIT("sa_send_single", VSTATUS_OK);
	return(VSTATUS_OK);
}

/*
 * Multi-paket RMPP protocol transfer
 */
Status_t
sa_send_multi(Mai_t *maip, sa_cntxt_t *sa_cntxt ) {
    int         i;
    int         wl=0;
    uint8_t     chkSum=0;
	uint32_t	dlen, datalen = sizeof(SAMadh_t);
	Status_t	status;
	STL_SA_MAD	samad;
	STL_SA_MAD	saresp;
    uint16_t    sendAbort=0;
    uint16_t    releaseContext=1;  /* release context here unless we are in resend mode */
    uint64_t    tnow, delta, ttemp;
    IBhandle_t  fd = (sa_cntxt->sendFd) ? sa_cntxt->sendFd : fd_sa;
	size_t      sa_data_size = IB_SA_DATA_LEN;

	IB_ENTER("sa_send_multi", maip, sa_cntxt, sa_cntxt->len , 0);

    /*
     * At this point, the maip->addrInfo.slid/dlid and maip->base.method have already 
     * been changed in sa_send_reply to reflect a response packet, so use sa_cntxt 
     * lid and method values
    */
    memset((void *)&samad, 0, sizeof(samad));
	saresp = samad;
	
    /* maip is NULL if Ack timeout */
	if( maip ) {
		BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER*)maip->data, 
									(STL_SA_MAD_HEADER*)&saresp);

		if (maip->base.bversion == STL_BASE_VERSION)
			sa_data_size = MIN(STL_SA_DATA_LEN, sa_cntxt->len);
	}

    /*
     * 	See if answer to the request coming in
     *  normal getTable requests are not hashed while getMulti's have isDS bit set
     */
    if( sa_cntxt->hashed == 0 || sa_cntxt->isDS ) {
        /* init/save parameters and reserve context */
		if( maip ) {
			INCREMENT_MAD_STATUS_COUNTERS(maip);
			memcpy( &sa_cntxt->mad, maip, sizeof( Mai_t ));
		}
        sa_cntxt->WF = 1;
        /* use value set in getMulti request ACK */
        if (!sa_cntxt->isDS) sa_cntxt->WL = 1;
        sa_cntxt->NS = sa_cntxt->WF;    // Next packet segment to send
        sa_cntxt->ES = 0;               // Expected segment number (Receiver only)
		sa_cntxt->last_ack = 0;         // last packet acked by receiver
		sa_cntxt->retries = 0;          // current retry count
		if ( sa_cntxt->len == 0 ) {
            sa_cntxt->segTotal = 1;
        } else if (sa_cntxt->len <= sa_data_length) {
            sa_cntxt->segTotal = (sa_data_size)?((sa_cntxt->len + sa_data_size - 1) / sa_data_size):1;
        } else {
            IB_LOG_WARN("sa_send_multi: NO RESOURCES--> sa_cntxt->len too large:", sa_cntxt->len);
			INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            sa_cntxt->len = 0;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
            sa_cntxt->segTotal = 1;
            sendAbort = 1;
        }
        /* calculate packet and total transaction timeouts C13-13.1.1 */
        sa_cntxt->RespTimeout = 4ull * (1<<20); // ~4.3 seconds
        sa_cntxt->tTime = 0;            // receiver only
        ttemp = sa_cntxt->mad.intime;   // save the time from original mad in context
        if (sa_cntxt->isDS) {
            sa_cntxt->mad.intime = ttemp;   // we want the time when getMulti request really started
            sa_cntxt->isDS = 0;
        }
        // we must return AID=SA_PATH_RECORD in response to getMultiPathRecord
        if (sa_cntxt->mad.base.aid == SA_MULTIPATH_RECORD) {
            sa_cntxt->mad.base.aid = SA_PATH_RECORD;
        }
        /* 8-bit cheksum of the rmpp response */
        sa_cntxt->chkSum = 0;
        if (saRmppCheckSum) {
            for (i=0; i<sa_cntxt->len; i++) {
                sa_cntxt->chkSum += sa_cntxt->data[i];
            }
        }
		sa_cntxt_reserve( sa_cntxt );
        samad.header.rmppVersion = RMPP_VERSION;
        samad.header.u.tf.rmppRespTime = 0x1F; // no time provided
        samad.header.offset = sa_cntxt->attribLen / 8; // setup attribute offset for RMPP xfer
        if (saDebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "Lid[0x%x] STARTING RMPP %s with TID="FMT_U64", CHKSUM[%d]",
                   (int)sa_cntxt->lid, sa_getMethodText((int)sa_cntxt->method), 
                   sa_cntxt->tid, sa_cntxt->chkSum);
        }
	} else if( maip ) {
        /* get original samad with appropriate offset */
        BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)sa_cntxt->mad.data, &samad, STL_SA_DATA_LEN);
        /*
         * Check the response and set the context parameters according to it   
         */
       if (saresp.header.rmppType == RMPP_TYPE_NOT && (saresp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            // invalid RMPP type
            IB_LOG_WARN_FMT(__func__,
                   "ABORTING - RMPP protocol error; RmppType is NULL in %s[%s] from Lid[%d] for TID="FMT_U64,
                   sa_getMethodText((int)sa_cntxt->method), sa_getAidName(maip->base.aid), (int)sa_cntxt->lid, sa_cntxt->tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            sendAbort = 1;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
       } else if (saresp.header.rmppVersion != RMPP_VERSION) {
            /* ABORT transaction with BadT status */
			INCREMENT_COUNTER(smCounterRmppStatusAbortUnsupportedVersion);
            sendAbort = 1;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION;
            IB_LOG_WARN_FMT(__func__,
                   "ABORTING - Unsupported Version %d in %s[%s] request from LID[0x%x], TID["FMT_U64"]",
                   saresp.header.rmppVersion, sa_getMethodText((int)sa_cntxt->method), sa_getAidName(maip->base.aid), (int)sa_cntxt->lid, sa_cntxt->tid);
       } else if (!(saresp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
           /* invalid RMPP type */
           IB_LOG_WARN_FMT(__func__,
                  "RMPP protocol error, RMPPFlags.Active bit is NULL in %s[%s] from LID[0x%x] for TID["FMT_U64"]",
                  sa_getMethodText((int)sa_cntxt->method), sa_getAidName(maip->base.aid), (int)sa_cntxt->lid, sa_cntxt->tid);
			INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
           samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
           sendAbort = 1;
       } else if (saresp.header.rmppType == RMPP_TYPE_ACK) {
            /* Got an ACK packet from receiver */
            if (saresp.header.segNum < sa_cntxt->WF) {
                /* silently discard the packet */
                if (saDebugPerf || saDebugRmpp) {
                	IB_LOG_INFINI_INFO_FMT(
                		"sa_send_multi",
					   	"LID[0x%x] sent ACK for seg %d which is less than Window First (%d) for TID: "FMT_U64, 
                        sa_cntxt->lid, (int)saresp.header.segNum, (int)sa_cntxt->WF,
                		sa_cntxt->tid);
                }
            } else if (saresp.header.segNum > sa_cntxt->WL) {
                /* ABORT the transaction with S2B status */
				INCREMENT_COUNTER(smCounterRmppStatusAbortSegnumTooBig);
                sendAbort = 1;
                samad.header.rmppStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG;
                IB_LOG_INFINI_INFO_FMT(
                	"sa_send_multi", "ABORT - LID[0x%x] sent invalid seg %d in ACK, should be <= than %d, ABORTING TID:" FMT_U64,
                    sa_cntxt->lid, (int)saresp.header.segNum, (int)sa_cntxt->WL,
                	sa_cntxt->tid);
            } else if (saresp.header.length < sa_cntxt->WL /*|| saresp.header.length > sa_cntxt->segTotal*/) { 
                /* length is NewWindowLast (NWL) in ACK packet */
                /* ABORT transaction with W2S status */
                sendAbort = 1;
                if (saresp.header.length < sa_cntxt->WL) {
					INCREMENT_COUNTER(smCounterRmppStatusAbortNewWindowLastTooSmall);
                    samad.header.rmppStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL;
                } else {
					INCREMENT_COUNTER(smCounterRmppStatusAbortUnspecified);
                    samad.header.rmppStatus = RMPP_STATUS_ABORT_UNSPECIFIED;
				}
                IB_LOG_INFINI_INFO_FMT(
                	"sa_send_multi", "ABORT - LID[0x%x] sent invalid NWL %d in ACK, should be >=%d and <=%d, ABORTING TID:"FMT_U64,
                    sa_cntxt->lid, (int)saresp.header.length, (int)sa_cntxt->WL,
				   	sa_cntxt->segTotal, sa_cntxt->tid);
            } else if (saresp.header.segNum >= sa_cntxt->last_ack) { 
				sa_cntxt->last_ack = saresp.header.segNum;
                sa_cntxt->retries = 0;  /* reset the retry count  after receipt of ack */
                /* is it ack of very last packet? */
                if (saresp.header.segNum == sa_cntxt->segTotal) {
                    /* we are done */
                    if (saDebugRmpp) {
                        IB_LOG_INFINI_INFO_FMT(__func__,
                               " Received seg %d ACK, %s[%s] transaction from LID[0x%x], TID["FMT_U64"] has completed",
                               saresp.header.segNum, sa_getMethodText((int)sa_cntxt->method), sa_getAidName(sa_cntxt->mad.base.aid),
                               sa_cntxt->lid, sa_cntxt->tid);
                    }
                } else {
                    /* update WF, WL, and NS and resume sends */
                    sa_cntxt->WF = sa_cntxt->last_ack + 1;
                    sa_cntxt->WL = (saresp.header.length > sa_cntxt->segTotal) ? sa_cntxt->segTotal : saresp.header.length;
                    /* see if new Response time needs to be calculated */
                    if (saresp.header.u.tf.rmppRespTime && saresp.header.u.tf.rmppRespTime != 0x1f) {
                        sa_cntxt->RespTimeout = 4ull * ( (2*(1<<sm_config.sa_packet_lifetime_n2)) + (1<<saresp.header.u.tf.rmppRespTime) );
                        if (saDebugRmpp) {
                            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "LID[0x%x] set RespTimeValue (%d usec) in ACK of seg %d for %s[%s], TID["FMT_U64"]",
                                   sa_cntxt->lid, (int)sa_cntxt->RespTimeout, saresp.header.segNum, 
                                   sa_getMethodText((int)sa_cntxt->method), sa_getAidName((int)sa_cntxt->mad.base.aid),
                                   sa_cntxt->tid);
                        }
                    }
                }
            }
		} else if (saresp.header.rmppType == RMPP_TYPE_STOP || saresp.header.rmppType == RMPP_TYPE_ABORT) {
            /* got a STOP or ABORT */
            if (saDebugPerf || saDebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
					"STOP/ABORT received for %s[%s] from LID[0x%x], status code = %x, for TID["FMT_U64"]",
					sa_getMethodText((int)sa_cntxt->method), sa_getAidName((int)maip->base.aid),
					sa_cntxt->lid, saresp.header.rmppStatus, sa_cntxt->tid);
            }
            sa_cntxt_release( sa_cntxt );
            IB_EXIT(__func__, VSTATUS_OK );
            return VSTATUS_OK ;
		} else {
            /* invalid RmppType received */
            IB_LOG_WARN_FMT(__func__,
                   "ABORT - Invalid rmppType %d received for %s[%s] from LID[0x%x] for TID["FMT_U64"]",
                   saresp.header.rmppType, sa_getMethodText((int)sa_cntxt->method), sa_getAidName((int)maip->base.aid),
                   sa_cntxt->lid, sa_cntxt->tid);
            // abort with badtype status
			INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            sendAbort = 1;
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
		}
	} else {
		/* We are timing out, retry till retry  count expires */
		/* get original samad from context with correct offset */
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)sa_cntxt->mad.data, &samad, STL_SA_DATA_LEN);
		++sa_cntxt->retries;
		if( sa_cntxt->retries > sm_config.max_retries ) {
			if (saDebugPerf || saDebugRmpp) {
				IB_LOG_INFINI_INFO_FMT(
					"sa_send_multi",
				   	"ABORT - MAX RETRIES EXHAUSTED; no ACK for seg %d of %s[%s] request from LID[0x%X], TID = "FMT_U64, 
				    (int)sa_cntxt->WL, sa_getMethodText((int)sa_cntxt->method),
				   	sa_getAidName(sa_cntxt->mad.base.aid), sa_cntxt->lid,
					sa_cntxt->tid);
			}
			/* ABORT transaction with too many retries status */
			INCREMENT_COUNTER(smCounterRmppStatusAbortTooManyRetries);
			sendAbort = 1;
			samad.header.rmppStatus = RMPP_STATUS_ABORT_TOO_MANY_RETRIES;
			/* let context_age do the releasing;  It already holds the lock */
			releaseContext=0;
		} else {
			INCREMENT_COUNTER(smCounterSaRmppTxRetries);
			sa_cntxt->NS = sa_cntxt->WF;            // reset Next packet segment to send
			if (saDebugRmpp) {
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Timed out waiting for ACK of seg %d of %s[%s] from LID[0x%x], TID["FMT_U64"], retry #%d",
				       (int)sa_cntxt->WL, sa_getMethodText((int)sa_cntxt->method), sa_getAidName((int)sa_cntxt->mad.base.aid),
				       sa_cntxt->lid, sa_cntxt->tid, sa_cntxt->retries);
			}
		}
	}

    /* see if we're done (last segment was acked) */
	if( sa_cntxt->last_ack == sa_cntxt->segTotal && !sendAbort) {
        if (saDebugPerf || saDebugRmpp) {
            vs_time_get (&tnow);
            delta = tnow-sa_cntxt->mad.intime;
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "%s[%s] RMPP [CHKSUM=%d] TRANSACTION from LID[0x%x], TID["FMT_U64"] has completed in %d.%.3d seconds (%"CS64"d usecs)",
                   sa_getMethodText((int)sa_cntxt->method), sa_getAidName(sa_cntxt->mad.base.aid), 
                   sa_cntxt->chkSum, sa_cntxt->lid, sa_cntxt->tid,
                   (int)(delta/1000000), (int)((delta - delta/1000000*1000000))/1000, delta);
        }
        /* validate that the 8-bit cheksum of the rmpp response is still the same as when we started */
        if (saRmppCheckSum) {
            chkSum = 0;
            for (i=0; i<sa_cntxt->len; i++) {
                chkSum += sa_cntxt->data[i];
            }
            if (chkSum != sa_cntxt->chkSum) {
                IB_LOG_ERROR_FMT(__func__,
                       "CHECKSUM FAILED [%d vs %d] for completeted %s[%s] RMPP TRANSACTION from LID[0x%x], TID["FMT_U64"]",
                       chkSum, sa_cntxt->chkSum, sa_getMethodText((int)sa_cntxt->method), sa_getAidName(sa_cntxt->mad.base.aid), 
                       sa_cntxt->lid, sa_cntxt->tid);
            }
        }
        if (releaseContext) sa_cntxt_release( sa_cntxt );
		IB_EXIT(__func__, VSTATUS_OK );
		return VSTATUS_OK ;
	}

    /* we must use the Mad_t that was saved in the context */
	maip = &sa_cntxt->mad ;
    /* 
     * send segments up till Window Last (WL) and wait for ACK 
     * Due to possible concurrency issue if reader is pre-empted while 
     * sending segement 1 and writer runs to process ACK of seg 1, use 
     * a local var for WL.
     * In the future we need to add individual context locking if we intend 
     * to go with a pool of SA threads.
     */
    wl = sa_cntxt->WL;
	while (sa_cntxt->NS <= wl && !sendAbort) {

        /*
         * calculate amount of data length to send in this segment and put in mad;
         * dlen=payloadLength in first packet, dlen=remainder in last packet
         */
		if( sa_cntxt->NS == sa_cntxt->segTotal ) {
			dlen = (sa_data_size)?(sa_cntxt->len % sa_data_size):0;
			dlen = (dlen)? dlen : sa_data_size ;
		} else 
			dlen = sa_data_size ;

        if (dlen > sa_data_length) {
            IB_LOG_WARN("sa_send_multi: dlen is too large dlen:", dlen);
        }
        // make sure there is data to send; could just be error case with no data
        if (sa_cntxt->len && sa_cntxt->data) {
            (void)memcpy(samad.data, sa_cntxt->data + ((sa_cntxt->NS - 1) * sa_data_size), dlen );
        }

        samad.header.rmppVersion = RMPP_VERSION;
        samad.header.rmppType = RMPP_TYPE_DATA;
        samad.header.rmppStatus = 0;
        samad.header.segNum = sa_cntxt->NS;
        if (sa_cntxt->NS == 1 && sa_cntxt->NS == sa_cntxt->segTotal) {
            /* 
             * first and last segment to transfer, set length to payload length
             * add 20 bytes of SA header to each segment for total payload 
             */
            samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST | RMPP_FLAGS_LAST;
			samad.header.length = sa_cntxt->len + (sa_cntxt->segTotal * SA_HEADER_SIZE);   
			//if (saDebugRmpp) IB_LOG_INFINI_INFO( "sa_send_multi: SA Transaction First and Last Frag len:", samad.header.length );
        } else if( sa_cntxt->NS == 1 ) {
            /* 
             * first segment to transfer, set length to payload length
             * add 20 bytes of SA header to each segment for total payload 
             */
			samad.header.length = sa_cntxt->len + (sa_cntxt->segTotal * SA_HEADER_SIZE);   
            samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST;
			//if (saDebugRmpp) IB_LOG_INFINI_INFO( "sa_send_multi: SA Transaction First Frag len:", samad.header.length );
		} else if( sa_cntxt->NS == sa_cntxt->segTotal ) {
            /* last segment to go; len=bytes remaining */
            samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_LAST;
			samad.header.length = dlen + SA_HEADER_SIZE;  // add the extra 20 bytes of SA header
			if( samad.header.length == 0 ) {
				samad.header.length = sa_data_size ;
			}
			//if (saDebugRmpp) IB_LOG_INFINI_INFO( "sa_send_multi: SA Transaction Last Frag len:", samad.header.length );
		}  else {
            /* middle segments */
            samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
			samad.header.length = 0;
			//if (saDebugRmpp) IB_LOG_INFINI_INFO( "sa_send_multi: SA Transaction Middle Frag len:", samad.header.length );
		}
        /* put samad back into Mad_t in the context */
        datalen += sa_data_size;
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)&samad, 
							(STL_SA_MAD*)maip->data,
							sa_data_size);

		if (saDebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(
            	"sa_send_multi",
			   	"sending fragment %d, len %d to LID[0x%x] for TID = "FMT_U64, 
                (int)samad.header.segNum, (int)samad.header.length, (int)sa_cntxt->lid,
            	sa_cntxt->tid);
        }
        /* increment NS */
        ++sa_cntxt->NS;
		if (maip->base.bversion == IB_BASE_VERSION) {
			status = mai_send(fd, maip);
		} else {
			status = mai_stl_send(fd, maip, &datalen);
		}

		if (status != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__,
                   "mai_send error [%d] while processing %s[%s] request from LID[0x%x], TID["FMT_U64"]",
                   status, sa_getMethodText((int)sa_cntxt->method), sa_getAidName(maip->base.aid), 
                   sa_cntxt->lid, sa_cntxt->tid);
            if (releaseContext) sa_cntxt_release( sa_cntxt );
			IB_EXIT(__func__, VSTATUS_OK );
			return VSTATUS_OK ;
		}
	}

	/*
     * Send Abort if desired
     */
    if (sendAbort) {
        samad.header.rmppVersion = RMPP_VERSION;
        samad.header.rmppType = RMPP_TYPE_ABORT;
        samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
        samad.header.u.tf.rmppRespTime = 0;
        samad.header.segNum = 0;
        samad.header.length = 0;
		BSWAPCOPY_STL_SA_MAD(&samad, (STL_SA_MAD*)maip->data, STL_SA_DATA_LEN);

		if ((status = mai_send(fd, maip)) != VSTATUS_OK)
            IB_LOG_ERROR_FMT(__func__,
				"error[%d] from mai_send while sending ABORT of %s[%s] request to LID[0x%x], TID["FMT_U64"]",
				status, sa_getMethodText((int)sa_cntxt->method), sa_getAidName(maip->base.aid),
				sa_cntxt->lid, maip->base.tid);
        /*
         * We are done with this RMPP xfer.  Release the context here if 
         * in flight transaction (maip not NULL) or let sa_cntxt_age do it 
         * for us because retries is > SA_MAX_RETRIES. 
         */
        if (releaseContext) sa_cntxt_release( sa_cntxt );
    }

	IB_EXIT("sa_send_multi", VSTATUS_OK);
	return(VSTATUS_OK);
}

//----------------------------------------------------------------------------//
//
// SA Caching methods
//
//----------------------------------------------------------------------------//

// SA caching initialization
//
Status_t
sa_cache_init(void)
{
	Status_t rc = VSTATUS_OK;
	
	IB_ENTER("sa_cache_init", 0, 0, 0, 0);
	
	// clear everything
	memset(&saCache, 0, sizeof(SACache_t));
	
	// init lock
	rc = vs_lock_init(&saCache.lock, VLOCK_FREE, VLOCK_THREAD);
	if (rc != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("can't initialize SA cache lock");
	}
	
	IB_EXIT("sa_cache_init", rc);
	return rc;
}

// Allocates a cache entry marked as transient, which is created an on-the-fly
// to hold aggregates of other cached data (such as an "all nodes" query, which
// can be built from the FI and Switch caches rather than being stored
// seperately).
//
Status_t
sa_cache_alloc_transient(SACacheEntry_t *caches[], int count, SACacheEntry_t **outCache)
{
	Status_t rc;
	int i, len, records;
	SACacheEntry_t *cp;
	
	IB_ENTER("sa_cache_alloc_transient", caches, count, 0, 0);
	
	*outCache = NULL;
	
	rc = vs_pool_alloc(&sm_pool, sizeof(SACacheEntry_t), (void*)&cp);
	if (rc != VSTATUS_OK) {
		IB_LOG_WARNRC("sa_cache_alloc_transient: failed to allocate memory for cache structure rc:", rc);
		IB_EXIT("sa_cache_alloc_transient", rc);
		return rc;
	} else {
		memset(cp, 0, sizeof(SACacheEntry_t));
	}
	
	len = records = 0;
	for (i = 0; i < count; i++) {
		if (caches[i] && caches[i]->valid) {
			len += caches[i]->len;
			records += caches[i]->records;
		}
	}
	
	if (len > 0) {
		rc = vs_pool_alloc(&sm_pool, len, (void*)&cp->data);
		if (rc != VSTATUS_OK) {
			vs_pool_free(&sm_pool, cp);
			IB_LOG_WARNRC("sa_cache_alloc_transient: failed to allocate memory for cache buffer rc:", rc);
			IB_EXIT("sa_cache_alloc_transient", rc);
			return rc;
		}
		for (i = 0; i < count; i++) {
			if (caches[i] && caches[i]->valid && caches[i]->len) {
				memcpy(cp->data + cp->len, caches[i]->data, caches[i]->len);
				cp->len += caches[i]->len;
			}
		}
	}
	
	cp->valid = 1;
	cp->transient = 1;
	cp->records = records;
	
	*outCache = cp;
	
	rc = VSTATUS_OK;
	IB_EXIT("sa_cache_alloc_transient", rc);
	return rc;
}

// Gets a cached SA query by its cache index (see #defines in header),
// and increments the cache element's reference count.
//
Status_t
sa_cache_get(int index, SACacheEntry_t **outCache)
{
	Status_t rc;
	SACacheEntry_t *cp;
	
	IB_ENTER("sa_cache_get", index, 0, 0, 0);
	
	*outCache = NULL;
	
	if (index < 0 || index >= SA_NUM_CACHES) {
		IB_LOG_WARN("sa_cache_get: invalid cache index:", index);
		rc = VSTATUS_BAD;
		IB_EXIT("sa_cache_get", rc);
		return rc;
	}
	
	cp = saCache.current[index];
	if (cp && cp->valid) {
		cp->refCount++;
		*outCache = cp;
	}
	
	rc = VSTATUS_OK;
	IB_EXIT("sa_cache_get", rc);
	return rc;
}

// Cleans entries out of the previous list.
//
void
sa_cache_clean(void)
{
	int count;
	SACacheEntry_t *cache, *next, *prev;
	
	IB_ENTER("sa_cache_clean", 0, 0, 0, 0);
	
	cache = saCache.previous;
	prev = NULL;
	count = 0;
	while (cache) {
		next = cache->next;
		if (cache->refCount == 0) {
			if (cache->data)
				(void)vs_pool_free(&sm_pool, cache->data);
			(void)vs_pool_free(&sm_pool, cache);
			if (prev) {
				prev->next = next;
			} else {
				saCache.previous = next;
			}
		} else {
			prev = cache;
			count++;
		}
		cache = next;
	}
	
	IB_LOG_VERBOSE("sa_cache_clean: elements remaining in SA cache history:", count);
	
	IB_EXIT("sa_cache_clean", 0);
}

// Release a cache structure, which is either a decref if it's a real cache
// element, or a free() if it's a transient element.
//
Status_t
sa_cache_release(SACacheEntry_t *cache)
{
	IB_ENTER("sa_cache_release", cache, 0, 0, 0);
	
	if (cache) {
		if (cache->transient) {
			// created on the fly, deallocate
			if (cache->data)
				(void)vs_pool_free(&sm_pool, cache->data);
			(void)vs_pool_free(&sm_pool, cache);
			cache = NULL;
		} else {
			// stored in cache; decref
			cache->refCount--;
		}
	}
	
	IB_EXIT("sa_cache_release", 0);
	return VSTATUS_OK;
}

// "free" function for an SA context containing cached data.  Simply passes the
// cache through to be released.
//
Status_t
sa_cache_cntxt_free(sa_cntxt_t *cntxt)
{
	Status_t rc;
	
	IB_ENTER("sa_cache_cntxt_free", cntxt, 0, 0, 0);
	
	rc = sa_cache_release(cntxt->cache);
	
	IB_EXIT("sa_cache_cntxt_free", rc);
	return rc;
}

#ifdef __VXWORKS__
// Utility method for displaying caching statistics from the shell.
//
void
sa_cache_print_stats(void)
{
	int i;
	SACacheEntry_t *curr;
	uint32_t bytesCurrent = 0, countPrevious = 0, bytesPrevious = 0;
	
	if (sm_state == SM_STATE_NOTACTIVE) {
		sysPrintf("SA caching statistics are unavailable; the SM is not active.\n");
		return;
	}
	
	(void)vs_lock(&saCache.lock);
	
	sysPrintf("Active SA caches:\n");
	for (i = 0; i < SA_NUM_CACHES; i++) {
		curr = saCache.current[i];
		if (curr) {
			sysPrintf("  %s [%p]: %ul records, %ul bytes, %ul references\n",
				curr->name, curr, curr->records, curr->len, curr->refCount);
			bytesCurrent += curr->len;
		} else {
			sysPrintf("  WARNING: Cache at index %u was not built.\n", i);
		}
	}
	
	if (saCache.previous) {
		sysPrintf("In-use cache elements pending deletion:\n");
		curr = saCache.previous;
		while (curr) {
			sysPrintf("  %s [%p]: %ul records, %ul bytes, %ul references\n",
				curr->name, curr, curr->records, curr->len, curr->refCount);
			countPrevious++;
			bytesPrevious += curr->len;
			curr = curr->next;
		}
	} else {
		sysPrintf("No in-use cache elements pending deletion.\n");
	}
	
	(void)vs_unlock(&saCache.lock);
	
	sysPrintf("Totals:\n");
	sysPrintf("  Active cache size: %ul bytes\n", bytesCurrent);
	sysPrintf("  In-use cache elements pending deletion: %ul (%ul bytes)\n",
		countPrevious, bytesPrevious);
}
#endif

//----------------------------------------------------------------------------//

Status_t
sa_Authenticate_Path(STL_LID srcLid, STL_LID dstLid) {
	int		i;
	int		j;
	STL_LID		topLid;
	Node_t		*nodep;
	Port_t		*portp;
	Status_t	status;
	Authenticator_t	srcAuth;
	Authenticator_t	dstAuth;
	Topology_t	*tp;

	IB_ENTER("sa_Authenticate_Path", srcLid, dstLid, 0, 0);

//
//	Assume failure.
//
	status = VSTATUS_BAD;

//
//	This routine must be called with the topology lock set.
//
	tp = &old_topology;
	if (tp->cost == NULL) {
        if (saDebugRmpp) IB_LOG_INFINI_INFO0("sa_Authenticate_Path: Topology cost array is NULL");
		IB_EXIT("sa_Authenticate_Path - cost = NULL", VSTATUS_BAD);
		return(VSTATUS_BAD);
	}

//
//	Initialize the stack variables.
//
	(void)memset((void *)&srcAuth, 0, sizeof(Authenticator_t));
	(void)memset((void *)&dstAuth, 0, sizeof(Authenticator_t));

//
//	Get the nodes of the src and dst.
//
	for_all_nodes(tp, nodep) {
		for_all_end_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
				continue;
			}

			topLid = portp->portData->lid + (1 << portp->portData->lmc) - 1;

			if ((portp->portData->lid <= srcLid) && (srcLid <= topLid)) {
				srcAuth.lid = srcLid;
				srcAuth.nodep = nodep;
				srcAuth.portp = portp;
			}

			if ((portp->portData->lid <= dstLid) && (dstLid <= topLid)) {
				dstAuth.lid = dstLid;
				dstAuth.nodep = nodep;
				dstAuth.portp = portp;
			}
		}
	}

//
//	If we haven't found the either one, then return an error.
//
	if ((dstAuth.nodep == NULL) || (srcAuth.nodep == NULL) ||
	    (dstAuth.portp == NULL) || (srcAuth.portp == NULL)) {
        if (saDebugRmpp) {
            if (srcAuth.nodep == NULL) {
                IB_LOG_INFINI_INFOX("sa_Authenticate_Path: source lid is not currently on fabric, LID ", srcLid);
            } else {
                IB_LOG_INFINI_INFOX("sa_Authenticate_Path: destination lid is not currently on fabric, LID ", dstLid);
            }
        }
		IB_EXIT("sa_Authenticate_Path - can't find nodes", VSTATUS_BAD);
		return(VSTATUS_BAD);
	}

//
//	We need to see if there is a path from the src to the dst.
//
    if (srcAuth.nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        i = srcAuth.nodep->swIdx;
    } else {
        /* it's an end node; need to find switch it's connected to */
        if ((nodep = sm_find_node(tp, srcAuth.portp->nodeno))) {
            /* it is the switch index in the switch list, not index in node list */
            i = nodep->swIdx;
        } else {
            IB_EXIT("sa_Authenticate_Path", VSTATUS_BAD);
            return(VSTATUS_BAD);
        }
    }
    if (dstAuth.nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
        j = dstAuth.nodep->swIdx;
    } else {
        /* it's an end node; need to find switch it's connected to */
        if ((nodep = sm_find_node(tp, dstAuth.portp->nodeno))) {
            /* it is the switch index in the switch list, not index in node list */
            j = nodep->swIdx;
        } else {
            IB_EXIT("sa_Authenticate_Path", VSTATUS_BAD);
            return(VSTATUS_BAD);
        }
    }

    if (tp->cost[Index(i,j)] == Cost_Infinity) {
        if (saDebugRmpp) IB_LOG_INFINI_INFO("sa_Authenticate_Path: no path from requester to LID ", dstAuth.lid);
    }

	IB_EXIT("sa_Authenticate_Path", status);
	return(status);
}

//------------------------------------------------------------------------------------------------//

Status_t
sa_Authenticate_Access(uint32_t type, STL_LID srcLid, STL_LID dstLid, STL_LID reqLid) {
	STL_LID		topLid;
	Node_t		*nodep;
	Port_t		*portp;
	Status_t	status;
	Authenticator_t	reqAuth;
	Authenticator_t	srcAuth;
	Authenticator_t	dstAuth;

	IB_ENTER("sa_Authenticate_Access", type, srcLid, dstLid, reqLid);

//
//	Assume bad status.
//
	status = VSTATUS_BAD;

//
//	Initialize the stack variables.
//
	(void)memset((void *)&reqAuth, 0, sizeof(Authenticator_t));
	(void)memset((void *)&srcAuth, 0, sizeof(Authenticator_t));
	(void)memset((void *)&dstAuth, 0, sizeof(Authenticator_t));

//
//	Get the nodes of the src, dst, and requester.
//
	for_all_nodes(&old_topology, nodep) {
		for_all_end_ports(nodep, portp) {
			if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
				continue;
			}

			topLid = portp->portData->lid + (1 << portp->portData->lmc) - 1;

			if ((portp->portData->lid <= reqLid) && (reqLid <= topLid)) {
				reqAuth.lid = reqLid;
				reqAuth.nodep = nodep;
				reqAuth.portp = portp;
			}

			if ((portp->portData->lid <= srcLid) && (srcLid <= topLid)) {
				srcAuth.lid = srcLid;
				srcAuth.nodep = nodep;
				srcAuth.portp = portp;
			}

			if ((portp->portData->lid <= dstLid) && (dstLid <= topLid)) {
				dstAuth.lid = dstLid;
				dstAuth.nodep = nodep;
				dstAuth.portp = portp;
			}
		}
	}

//
//	If we haven't found the requester, then return bad status.  If this
//	is a PathRecord authentication, we also have to have found the src
//	and dst.  For non-PathRecord, we just need to find the dst.
//
	if ((reqAuth.nodep == NULL) || (dstAuth.nodep == NULL)) {
		goto check_PathRecord_done;
	}

	if (type == SA_PATH_RECORD) {
		if (srcAuth.nodep == NULL) {
			goto check_PathRecord_done;
		}
	}

//
//	Do the PKey comparisons.  If this is NOT for a PathRecord, get the node
//	of the destination.  Use the PKeys from any port on the destination to
//	compare to any port on the requester.
//
	if (sa_Compare_Node_PKeys(reqAuth.nodep, dstAuth.nodep) != VSTATUS_OK) {
		goto check_PathRecord_done;
	}

	if ((type == SA_PATH_RECORD) &&
		((sa_Compare_Node_PKeys(reqAuth.nodep, srcAuth.nodep) != VSTATUS_OK) ||
		 (sa_Compare_Node_PKeys(srcAuth.nodep, dstAuth.nodep) != VSTATUS_OK))) {
		goto check_PathRecord_done;
	}

	status = VSTATUS_OK;

check_PathRecord_done:
	IB_EXIT("sa_Authenticate_Access", status);
	return(status);
}


Status_t
sa_Compare_PKeys(STL_PKEY_ELEMENT *key1, STL_PKEY_ELEMENT *key2) {
	int		i;
	int		j;

	IB_ENTER("sa_Compare_PKeys", key1, key2, 0, 0);

//
//	Loop over both arrays looking for at least one match.
//
	for (i = 0; (i < SM_PKEYS); i++) {
		for (j = 0; (j < SM_PKEYS); j++) {
			if ((PKEY_VALUE(key1[i].AsReg16) == PKEY_VALUE(key2[j].AsReg16)) &&
				((PKEY_TYPE(key1[i].AsReg16) == PKEY_TYPE_FULL) ||
				 (PKEY_TYPE(key2[j].AsReg16) == PKEY_TYPE_FULL))) {
				IB_EXIT("sa_Compare_PKeys", VSTATUS_OK);
				return(VSTATUS_OK);
			}
		}
	}

//
//	No match was found.  Return an error.
//
	IB_EXIT("sa_Compare_PKeys", VSTATUS_BAD);
	return(VSTATUS_BAD);
}


Status_t
sa_Compare_Port_PKeys(Port_t *port1, Port_t *port2) {
    STL_PKEY_ELEMENT *key1, *key2;
	int		i;
	int		j;

    if (!port1 || !port2)
        return VSTATUS_BAD;

	IB_ENTER(__func__, port1, port2, 0, 0);

    key1 = port1->portData->pPKey;
    key2 = port2->portData->pPKey;

    //
    //	If there are no PKeys, then return good.
    //
	if ((key1[0].AsReg16 == 0) && (key2[0].AsReg16 == 0)) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}

    //
    //	Loop over both arrays looking for at least one match.
    //
	for (i = 0; (i < SM_PKEYS) && (i <= bitset_find_last_one(&port1->portData->pkey_idxs)); i++) {
		for (j = 0; (j < SM_PKEYS) && (j <= bitset_find_last_one(&port2->portData->pkey_idxs)); j++) {
			if ((PKEY_VALUE(key1[i].AsReg16) == PKEY_VALUE(key2[j].AsReg16)) &&
				((PKEY_TYPE(key1[i].AsReg16) == PKEY_TYPE_FULL) ||
				 (PKEY_TYPE(key2[j].AsReg16) == PKEY_TYPE_FULL))) {
				IB_EXIT(__func__, VSTATUS_OK);
				return(VSTATUS_OK);
			}
		}
	}

    //
    //	No match was found.  Return an error.
    //
	IB_EXIT(__func__, VSTATUS_BAD);
	return(VSTATUS_BAD);
}

Status_t
sa_Compare_Node_PKeys(Node_t *node1p, Node_t *node2p) {
	Port_t		*port1p;
	Port_t		*port2p;

	IB_ENTER("sa_Compare_Node_PKeys", node1p, node2p, 0, 0);

	for_all_physical_ports(node1p, port1p) {
		if (!sm_valid_port(port1p) || port1p->state <= IB_PORT_DOWN) {
			continue;
		}

		for_all_physical_ports(node2p, port2p) {
			if (!sm_valid_port(port2p) || port2p->state <= IB_PORT_DOWN) {
				continue;
			}

			if (sa_Compare_PKeys(port1p->portData->pPKey, port2p->portData->pPKey) == VSTATUS_OK) {
				IB_EXIT("sa_Compare_Node_Pkeys", VSTATUS_OK);
				return(VSTATUS_OK);
			}
		}
	}

//
//	No match was found.  Return an error.
//
	IB_EXIT("sa_Compare_Node_Pkeys", VSTATUS_BAD);
	return(VSTATUS_BAD);
}

Status_t
sa_Compare_Node_Port_PKeys(Node_t *nodep, Port_t *srcportp) {
	Port_t		*portp;

	IB_ENTER("sa_Compare_Node_Port_PKeys", nodep, srcportp, 0, 0);
	
	if (!nodep || !srcportp) return VSTATUS_BAD;

	if (!sm_valid_port(srcportp) || srcportp->state <= IB_PORT_DOWN) return VSTATUS_BAD;
	
	if (srcportp->portData->nodePtr == nodep) return VSTATUS_OK;

	for_all_end_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)  continue;

		if (sa_Compare_PKeys(portp->portData->pPKey, srcportp->portData->pPKey) == VSTATUS_OK) {
			IB_EXIT("sa_Compare_Node_Port_PKeys", VSTATUS_OK);
			return(VSTATUS_OK);
		}
	}

//	No match was found.  Return an error.
//
	IB_EXIT("sa_Compare_Node_Port_PKeys", VSTATUS_BAD);
	return(VSTATUS_BAD);
}

// --------------------------------------------------------------------------- /

Status_t
sa_data_offset(uint16_t class, uint16_t type) {

	IB_ENTER("sa_data_offset", type, 0, 0, 0);

	template_type = type;
	template_fieldp = NULL;
//
//	Create the mask for the comparisons.
//
	switch (type) {
	case STL_SA_ATTR_NODE_RECORD:
	//case  SA_ATTR_NODE_RECORD: has same value....
		if (class == STL_SA_CLASS_VERSION) {
			template_length = sizeof(STL_NODE_RECORD);
			template_fieldp = StlNodeRecordFieldMask;
		} else {
			template_length = sizeof(IB_NODE_RECORD);
			template_fieldp = IbNodeRecordFieldMask;
		}
		break;
	case STL_SA_ATTR_PORTINFO_RECORD:
		if (class == STL_SA_CLASS_VERSION) {
    		template_length = sizeof(STL_PORTINFO_RECORD);
    		template_fieldp = StlPortInfoRecordFieldMask;
        } else {
            template_length = sizeof(IB_PORTINFO_RECORD);
            template_fieldp = IbPortInfoRecordFieldMask;
            }
		break;
	case STL_SA_ATTR_SWITCHINFO_RECORD:
		template_length = sizeof(STL_SWITCHINFO_RECORD);
		template_fieldp = StlSwitchInfoRecordFieldMask;
		break;
	case STL_SA_ATTR_LINEAR_FWDTBL_RECORD:
		template_length = sizeof(STL_LINEAR_FORWARDING_TABLE_RECORD);
		template_fieldp = StlLFTRecordFieldMask;
		break;
	case STL_SA_ATTR_MCAST_FWDTBL_RECORD:
		template_length = sizeof(STL_MULTICAST_FORWARDING_TABLE_RECORD);
		template_fieldp = StlMFTRecordFieldMask;
		break;
	case STL_SA_ATTR_SMINFO_RECORD:
		template_length = sizeof(STL_SMINFO_RECORD);
		template_fieldp = StlSMInfoRecordFieldMask;
		break;
	case STL_SA_ATTR_INFORM_INFO_RECORD:
		if (class == STL_SA_CLASS_VERSION) {
			template_length = sizeof(STL_INFORM_INFO_RECORD);
			template_fieldp = StlInformRecordFieldMask;
        } else {
			template_length = sizeof(IB_INFORM_INFO_RECORD);
			template_fieldp = IbInformRecordFieldMask;
		}
		break;
	case STL_SA_ATTR_LINK_RECORD:
		template_length = sizeof(STL_LINK_RECORD);
		template_fieldp = StlLinkRecordFieldMask;
		break;
	case STL_SA_ATTR_SERVICE_RECORD:
		if (class == STL_SA_CLASS_VERSION) {
			template_length = sizeof(STL_SERVICE_RECORD);
			template_fieldp = StlServiceRecordFieldMask;
		} else {
			template_length = sizeof(IB_SERVICE_RECORD);
			template_fieldp = IbServiceRecordFieldMask;
		}
		break;
	case STL_SA_ATTR_P_KEY_TABLE_RECORD:
		template_length = sizeof(STL_P_KEY_TABLE_RECORD);
		template_fieldp = StlPKeyTableFieldMask;
		break;
	case STL_SA_ATTR_VLARBTABLE_RECORD:
		template_length = sizeof(STL_VLARBTABLE_RECORD);
		template_fieldp = StlVLArbTableRecordFieldMask;
		break;
	case SA_PATH_RECORD:
			template_length = sizeof(IB_PATH_RECORD);
			template_fieldp = PathRecordFieldMask;
		break;
	case STL_SA_ATTR_MCMEMBER_RECORD:
		if (class == STL_SA_CLASS_VERSION) {
			template_length = sizeof(STL_MCMEMBER_RECORD);
			template_fieldp = StlMcMemberRecordFieldMask;
		} else {
			template_length = sizeof(IB_MCMEMBER_RECORD);
			template_fieldp = IbMcMemberRecordFieldMask;
		}
		break;
	case STL_SA_ATTR_TRACE_RECORD:
		template_length = sizeof(STL_TRACE_RECORD);
		template_fieldp = StlTraceRecordFieldMask;
		break;
	case STL_SA_ATTR_VF_INFO_RECORD:
		template_length = sizeof(STL_VFINFO_RECORD);
		template_fieldp = StlVfInfoRecordFieldMask;
		break;
	case STL_SA_ATTR_DG_MEMBER_RECORD:
		template_length = sizeof(STL_DEVICE_GROUP_MEMBER_RECORD);
		template_fieldp = StlDgMemberRecordFieldMask;
		break;
	case STL_SA_ATTR_DG_NAME_RECORD:
		template_length = sizeof(STL_DEVICE_GROUP_NAME_RECORD);
		template_fieldp = StlDgNameRecordFieldMask;
		break;
	case STL_SA_ATTR_DT_MEMBER_RECORD:
		template_length = sizeof(STL_DEVICE_TREE_MEMBER_RECORD);
		template_fieldp = StlDtMemberRecordFieldMask;
		break;
	default:
        //IB_LOG_ERROR_FMT( "sa_data_offset", "Unsupported SA attribute %x", type);
		IB_EXIT("sa_data_offset", VSTATUS_BAD);
		return(VSTATUS_BAD);
	}

	if (template_fieldp != NULL) {
		IB_EXIT("sa_data_offset", VSTATUS_OK);
		return(VSTATUS_OK);
	} else {
		IB_EXIT("sa_data_offset", VSTATUS_BAD);
		return(VSTATUS_BAD);
	}
}

char *sa_getAidName(uint16_t aid) {
	static char num[8] = "0x0000";
    static char *aidName[0x40]={	/* 0x00 to 0x3F */
		"0x00","CLASSPORTINFO","NOTICE","INFORMINFO","0x4","0x5","0x6","0x7",
		"0x08","0x09","0x0A","0x0B","0x0C","0x0D","0x0E","0x0F",
		"0x10","NODE","PORTINFO","SC MAP","SWITCH","LFT","0x16","MFT",
		"SMINFO","0x19","0x1A","0x1B","0x1C","0x1D","0x1E","0x1F",
		"LINK","0x21","0x22","0x23","0x24","0x25","0x26","0x27",
		"0x28","0x29","0x2A","0x2B","0x2C","0x2D","0x2E","0x2F",
		"0x30","SERVICE","0x32","PARTITION","0x34","PATH","VL ARB","0x37",
		"MCMEMBER","TRACE","MULTIPATH","SERVICEASSOC","0x3C","0x3D","0x3E","0x3F",
	};
    static char *aidNameHigh[0x18]={	/* 0x80 to 0x97 */
		"SL2SC","SC2SL","SC2VLnt","SC2VLt","SC2VLr","PGFWD","MULTIPATH_GUID","MULTIPATH_LID",
		"CABLEINFO","VFABRICINFO","PORTSTATEINFO","PGTBL","BUFCTRL","FABRICINFO","0x8E", "0x8F",
		"QUARANTINED","CONGINFO","SWCONFINFO","SWPORTCONG","HFICONG","HFICONGCTRL","0x96","0x97"
	};

	if (aid < 0x40)
		return(aidName[aid]);
	else if (aid >= 0x80 && aid <= 0x97)
		return(aidNameHigh[aid-0x80]);
	else if (aid == SA_INFORM_RECORD)	/* 0xF3 */
		return "INFORMINFORECORD";
	else {
		snprintf(num, sizeof(num), "0x%.2X", aid);
		return(num);
	}
}

uint8_t linkWidthToRate(PortData_t *portData) { 
    STL_PORT_INFO *portInfo = &portData->portInfo; 
    
	switch (portInfo->LinkSpeed.Active) {
		case STL_LINK_SPEED_12_5G:
			switch (portInfo->LinkWidth.Active) {
				case STL_LINK_WIDTH_1X: return IB_STATIC_RATE_14G; // STL_STATIC_RATE_12_5G;
				case STL_LINK_WIDTH_2X: return IB_STATIC_RATE_25G;
				case STL_LINK_WIDTH_3X: return IB_STATIC_RATE_40G; // STL_STATIC_RATE_37_5G;
				case STL_LINK_WIDTH_4X: return IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G
				default: return IB_STATIC_RATE_1GB; // Invalid!
    	}
		case STL_LINK_SPEED_25G:
			switch (portInfo->LinkWidth.Active) {
				case STL_LINK_WIDTH_1X: return IB_STATIC_RATE_25G;
				case STL_LINK_WIDTH_2X: return IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G;
				case STL_LINK_WIDTH_3X: return IB_STATIC_RATE_80G; // STL_STATIC_RATE_75G;
				case STL_LINK_WIDTH_4X: return IB_STATIC_RATE_100G;
				default: return IB_STATIC_RATE_1GB; // Invalid!
		}
		default: return IB_STATIC_RATE_1GB; // Invalid!
	}
}

void dumpGid(IB_GID gid) {
	int i;
	sysPrintf("0x");
	for (i = 0; i < 8; ++i) {
		sysPrintf("%.2X", gid.Raw[i]);
	}
	sysPrintf(":");
	for (;i < 16; ++i) {
		sysPrintf("%.2X", gid.Raw[i]);
	}
}

void dumpGuid(Guid_t guid) {
	int i;
	uint8_t * bytes = (uint8_t *)&guid;
	sysPrintf("0x");
	for (i = 0; i < 8; ++i) {
		sysPrintf("%.2X", bytes[i]);
	}
}

void dumpBytes(uint8_t * bytes, size_t num_bytes) {
	size_t i;
	for (i = 0; i < num_bytes; ++i) {
		sysPrintf("0x%.2X ", bytes[i]);
		if ((i + 1) % 16 == 0) {
			sysPrintf("\n");
		} else if ((i + 1) % 4 == 0) {
			sysPrintf(" ");
		}
	}
}

void dumpHWords(uint16_t * hwords, size_t num_hwords) {
	size_t i;
	for (i = 0; i < num_hwords; ++i) {
		sysPrintf("0x%.4X ", hwords[i]);
		if ((i + 1) % 8 == 0) {
			sysPrintf("\n");
		} else if ((i + 1) % 2 == 0) {
			sysPrintf(" ");
		}
	}
}

void dumpWords(uint32_t * words, size_t num_words) {
	size_t i;
	for (i = 0; i < num_words; ++i) {
		sysPrintf("0x%.4X ", (int)words[i]);
		if ((i + 1) % 4 == 0) {
			sysPrintf("\n");
		}
	}

}

void dumpGuids(Guid_t * guids, size_t num_guids) {
	size_t i;
	for (i = 0; i < num_guids; ++i)
	{
		dumpGuid(guids[i]);
		if ((i + 1) % 2 == 0) {
			sysPrintf("\n");
		} else {
			sysPrintf(" ");
		}
	}
}


int setDefBcGrp(uint16_t pKey, uint8_t	mtu, uint8_t rate, uint8_t sl, uint32_t qkey, uint32_t fl, uint8_t tc){
#ifdef __VXWORKS__
	//char 		bmVal[20];
	int rc = 0; // In icsCliSm.c CLI_OK == 0

	if (   (idbSetSmDefMcGrpPKey((uint32_t) pKey) != 1)
        || (idbSetSmDefMcGrpMtu((uint32_t) mtu) != 1)
        || (idbSetSmDefMcGrpRate((uint32_t) rate) != 1)
        || (idbSetSmDefMcGrpSl((uint32_t) sl) != 1)
        || (idbSetSmDefMcGrpQKey((uint32_t) qkey) != 1)
        || (idbSetSmDefMcGrpFlowLabel((uint32_t) fl) != 1)
        || (idbSetSmDefMcGrpTClass((uint32_t) tc) != 1) )
	{
		printf("Couldn't set broadcast group values (errno = %d)\n", errnoGet());
		rc = 3; // In icsCliSm.c CLI_ERR_ERR == 3
	}

	//memset(bmVal,0,sizeof(bmVal));
	//sprintf(bmVal,"%04x%02x%02x%02x%08x",pKey,mtu,rate,sl,(unsigned)qkey);
	//bmSetTagValue("sm_defmcgrp",bmVal);

	return rc;

#else
	return 0;
#endif
}

int clearDefBcGrp(void){
#ifdef __VXWORKS__
    (void)idbSetSmDefMcGrpPKey((uint32_t)0);
    (void)idbSetSmDefMcGrpMtu((uint32_t)0);
    (void)idbSetSmDefMcGrpRate((uint32_t)0);
    (void)idbSetSmDefMcGrpSl((uint32_t)0);
    (void)idbSetSmDefMcGrpQKey((uint32_t)0);
    (void)idbSetSmDefMcGrpFlowLabel((uint32_t)0);
    (void)idbSetSmDefMcGrpTClass((uint32_t)0);
	//bmSetTagValue("sm_defmcgrp",NULL);
	return 1;
#else
	return 0;
#endif
}
 
int getDefBcGrp(uint16_t *pKey, uint8_t	*mtu, uint8_t *rate, uint8_t *sl, uint32_t *qkey, uint32_t *fl, uint8_t *tc){
#ifdef __VXWORKS__
	/*
     * Get the values from IDB; not set if mtu or rate is 0
     */

    *pKey = (uint16_t)idbGetSmDefMcGrpPKey();
    *mtu = (uint8_t)idbGetSmDefMcGrpMtu();
    *rate = (uint8_t)idbGetSmDefMcGrpRate();
    *sl = (uint8_t)idbGetSmDefMcGrpSl();
    *qkey = (uint32_t)idbGetSmDefMcGrpQKey();
    *fl = (uint32_t)idbGetSmDefMcGrpFlowLabel();
    *tc = (uint8_t)idbGetSmDefMcGrpTClass();

	return idbGetMcGrpCreate();

#else

	if(!sm_def_mc_group){
		return 0;
	}

	*pKey	= (uint16_t)sm_def_mc_group;
	*rate	= (uint8_t)sm_mdg_config.group[0].def_mc_rate_int;
	*mtu	= (uint8_t)sm_mdg_config.group[0].def_mc_mtu_int; 
	*sl 	= (uint8_t)sm_mdg_config.group[0].def_mc_sl;
	*qkey	= (uint32_t)sm_mdg_config.group[0].def_mc_qkey;
    /* not currently used anywhere */
    *fl     = (uint32_t)sm_mdg_config.group[0].def_mc_fl;
    *tc     = (uint8_t)sm_mdg_config.group[0].def_mc_tc;

#endif
	/*
     * return default value if anything is zero
     */
    if (*pKey == 0) {
        *pKey = getDefaultPKey();
    }
	if (*mtu == 0) {
		*mtu = IB_MTU_2048;
	}

	if (*rate == 0) {
		*rate = IB_STATIC_RATE_200G;
	}
	return 1;
}


Status_t sa_SetDefBcGrp(void) {

	int				vf;
	VFDg_t*			mcastGrpp;
	VFAppMgid_t*	mgidp;
	cl_map_item_t* cl_map_item;

	(void)vs_rdlock(&old_topology_lock);
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;

	for (vf= 0; vf < VirtualFabrics->number_of_vfs_all && vf < MAX_VFABRICS; vf++) {
		if (VirtualFabrics->v_fabric_all[vf].standby) continue;
		for (mcastGrpp = VirtualFabrics->v_fabric_all[vf].default_group; mcastGrpp; 
			 mcastGrpp = mcastGrpp->next_default_group) {
			if (mcastGrpp->def_mc_create) {
				if (!mcastGrpp->mgidMapSize) {
					createMCastGroups(vf, mcastGrpp->def_mc_pkey, 
							mcastGrpp->def_mc_mtu_int, mcastGrpp->def_mc_rate_int,
							mcastGrpp->def_mc_sl, mcastGrpp->def_mc_qkey,
							mcastGrpp->def_mc_fl, mcastGrpp->def_mc_tc);
				} else {
					for_all_qmap_ptr(&mcastGrpp->mgidMap, cl_map_item, mgidp) {
						createMCastGroup(mgidp->mgid, mcastGrpp->def_mc_pkey,
								mcastGrpp->def_mc_mtu_int, mcastGrpp->def_mc_rate_int,
								mcastGrpp->def_mc_sl, mcastGrpp->def_mc_qkey,
								mcastGrpp->def_mc_fl, mcastGrpp->def_mc_tc);
					}
				}
			}
		}
	}
	(void)vs_rwunlock(&old_topology_lock);

	return(VSTATUS_OK);
}

// valid combos of start and end phase:
// 0	0	- just unique pairs (SLID1, DLID1, SLID2, DLID2, SLID3, DLID1,...)
// 0	1	- all combos, but put unique pairs first
// 4	4	- all combos, for all src, for all dst
// 6	6	- just unique pairs, limit to no more than 1 path per lid
// 				eg. if LMC not equal, some lids on one side never used
// 				can be useful to limit paths to switches where redundant paths
// 				not important because would only be redundant in 1 direction
void lid_iterator_init(lid_iterator_t *iter,
				Port_t *src_portp, STL_LID src_start_lid, STL_LID src_lid_len,
				Port_t *dst_portp, STL_LID dst_start_lid, STL_LID dst_lid_len,
				int path_mode,
			   	STL_LID *slid, STL_LID *dlid)
{
	*slid = iter->slid = iter->src_start = src_start_lid;
	iter->src_endp1 = iter->src_start + src_lid_len ;
	iter->src_lmc_mask = ~(0xffffffff<<src_portp->portData->lmc);

	*dlid = iter->dlid = iter->dst_start = dst_start_lid;
	iter->dst_endp1 = iter->dst_start + dst_lid_len;
	iter->dst_lmc_mask = ~(0xffffffff<<dst_portp->portData->lmc);

	switch (path_mode) {
	case PATH_MODE_PAIRWISE:
		iter->phase = 0;
		iter->end_phase = 0;
		break;
	case PATH_MODE_ORDERALL:
		iter->phase = 0;
		iter->end_phase = 1;
		break;
	case PATH_MODE_MINIMAL:
		iter->phase = 6;
		iter->end_phase = 6;
		break;
	case PATH_MODE_SRCDSTALL:
		iter->phase = 4;
		iter->end_phase = 4;
		break;
	}
}

int lid_iterator_done(lid_iterator_t *iter)
{
	return ((iter->slid >= iter->src_endp1 && iter->dlid >= iter->dst_endp1)
		|| iter->phase > iter->end_phase);
}

void lid_iterator_next(lid_iterator_t *iter, STL_LID *slid, STL_LID *dlid)
{
	if (iter->phase == 0) {
		// PATH_MODE_PAIRWISE or 1st part of PATH_MODE_ORDERALL
		// pairwise SLID1 DLID1, SLID2, DLID2, etc
		iter->slid++;
		iter->dlid++;
		// we keep going til both sides have used every lid at least once
		if (iter->slid < iter->src_endp1 || iter->dlid < iter->dst_endp1)
			goto done;
		iter->phase++;
		iter->slid = iter->src_start;
		iter->dlid = iter->dst_start;
	}
	if (iter->phase == 1) {
		// 2nd part of PATH_MODE_ORDERALL
		// ones covered not in phase 0 (SLID1, DLID2-N, SLID2, DLID1/3-N, etc

		do {
			do {
				// inner loop, incr dst
				iter->dlid++;
				if (iter->dlid >= iter->dst_endp1)
					break;
				if ((iter->slid & iter->src_lmc_mask & iter->dst_lmc_mask)
					!= (iter->dlid & iter->dst_lmc_mask & iter->src_lmc_mask))
					goto done;
			} while (1);
			iter->dlid = iter->dst_start;
			iter->slid++;
			if (iter->slid >= iter->src_endp1)
				break;
			if ((iter->slid & iter->src_lmc_mask & iter->dst_lmc_mask)
				!= (iter->dlid & iter->dst_lmc_mask & iter->src_lmc_mask))
				goto done;
		} while (1);
		iter->phase++;
		iter->slid = iter->src_start;
		iter->dlid = iter->dst_start;
		// done
	}
	if (iter->phase == 4) {
		// PATH_MODE_SRCDSTAL
		// inner loop, incr dst
		iter->dlid++;
		if (iter->dlid < iter->dst_endp1)
			goto done;
		iter->dlid = iter->dst_start;
		// outer loop, incr slid
		iter->slid++;
		if (iter->slid < iter->src_endp1)
			goto done;
		// end of all src/dst combos, trigger done
		iter->phase++;
		iter->slid = iter->src_start;
		iter->dlid = iter->dst_start;
		// done
	}
	if (iter->phase == 6) {
		// PATH_MODE_MINIMAL
		// pairwise SLID1 DLID1, SLID2, DLID2, etc
		// but unlike PATH_MODE_PAIRWISE stop when 1 side used all lids
		iter->slid++;
		iter->dlid++;
		if (iter->slid < iter->src_endp1 && iter->dlid < iter->dst_endp1)
			goto done;
		iter->phase++;
		iter->slid = iter->src_start;
		iter->dlid = iter->dst_start;
	}
	return;
done:
	*slid = (iter->src_start & (0xffffffff ^ iter->src_lmc_mask)) | (iter->slid & iter->src_lmc_mask);
	*dlid = (iter->dst_start & (0xffffffff ^ iter->dst_lmc_mask)) | (iter->dlid & iter->dst_lmc_mask);
	return;
}

// valid combos of start and end phase:
// 0	0	- just unique pairs (SLID1, DLID1, SLID2, DLID2, SLID3, DLID1,...)
// 0	1	- all combos, but put unique pairs first
// 4	4	- all combos, for all src, for all dst
// 6	6	- just unique pairs, limit to no more than 1 path per lid
// 				eg. if LMC not equal, some lids on one side never used
// 				can be useful to limit paths to switches where redundant paths
// 				not important because would only be redundant in 1 direction
// This handles 1 fixed lid, 1 GID or wildcard style queries.
// src is the fixed side, dst is the iterated side.
// doesn't really matter if src is source or dest and visa versa
void lid_iterator_init1(lid_iterator_t *iter,
				Port_t *src_portp, STL_LID slid, STL_LID src_lid_start, STL_LID src_lid_len,
				Port_t *dst_portp, STL_LID dst_lid_start, STL_LID dst_lid_len,
			   	int path_mode,
			   	STL_LID *dlid)
{
	STL_LID slid_offset;

	//iter->src_start = src_portp->portData->lid;
	iter->src_start = src_lid_start;
	iter->slid = slid;
	//iter->src_endp1 = iter->src_start + (1<<src_portp->portData->lmc);
	iter->src_endp1 = iter->src_start + src_lid_len;
	iter->src_lmc_mask = ~(0xffffffff<<src_portp->portData->lmc);
	slid_offset = slid & iter->src_lmc_mask;

	//*dlid = iter->dlid = iter->dst_start = dst_portp->portData->lid;
	*dlid = iter->dlid = iter->dst_start = dst_lid_start;
	//iter->dst_endp1 = iter->dst_start + (1<<dst_portp->portData->lmc);
	iter->dst_endp1 = iter->dst_start + dst_lid_len;
	iter->dst_lmc_mask = ~(0xffffffff<<dst_portp->portData->lmc);

	switch (path_mode) {
	case PATH_MODE_PAIRWISE:
		iter->phase = 0;
		iter->end_phase = 0;
		break;
	case PATH_MODE_ORDERALL:
		iter->phase = 0;
		iter->end_phase = 1;
		break;
	case PATH_MODE_MINIMAL:
		iter->phase = 6;
		iter->end_phase = 6;
		break;
	case PATH_MODE_SRCDSTALL:
		iter->phase = 4;
		iter->end_phase = 4;
		break;
	}

	switch (iter->phase) {
	case 0:
		*dlid = iter->dlid = iter->dst_start | (slid_offset & iter->dst_lmc_mask);
		break;
	case 4:
		// no adjustment, just start at 1st dlid; will do all
		break;
	case 6:
		// will be exactly 1 path
		*dlid = iter->dlid = iter->dst_start | (slid_offset & iter->dst_lmc_mask);
		break;
	}
}
	

int lid_iterator_done1(lid_iterator_t *iter)
{
	return ( iter->dlid >= iter->dst_endp1 || iter->phase > iter->end_phase);
}

void lid_iterator_next1(lid_iterator_t *iter, STL_LID *dlid)
{
	if (iter->phase == 0) {
		// pairwise SLID1 DLID1, SLID2, DLID2, etc
		iter->dlid += (iter->src_endp1 - iter->src_start);	// bump by 1<<lmc
		if (iter->dlid < iter->dst_endp1)
			goto done;
		iter->phase++;
		iter->dlid = iter->dst_start-1;	// compensate for dlid++ in phase 1
	}
	if (iter->phase == 1) {
		// ones covered not in phase 0 (SLID1, DLID2-N, SLID2, DLID1/3-N, etc

		do {
			// inner loop, incr dst
			iter->dlid++;
			if (iter->dlid >= iter->dst_endp1)
				break;
			if ((iter->slid & iter->src_lmc_mask & iter->dst_lmc_mask)
				!= (iter->dlid & iter->dst_lmc_mask & iter->src_lmc_mask))
				goto done;
		} while (1);
		iter->phase++;
		iter->dlid = iter->dst_start;
		// done
	}
	if (iter->phase == 4) {
		// ones covered not in phase 0 (SLID1, DLID2-N, SLID2, DLID1/3-N, etc

		// inner loop, incr dst
		iter->dlid++;
		if (iter->dlid < iter->dst_endp1)
			goto done;
		iter->phase++;
		iter->dlid = iter->dst_start;
		// done
	}
	if (iter->phase == 6) {
		// exactly 1 path
		iter->phase++;
		iter->dlid = iter->dst_start;
	}
	return;
done:
	*dlid = (iter->dst_start & ~iter->dst_lmc_mask) | (iter->dlid & iter->dst_lmc_mask);
	return;
}
