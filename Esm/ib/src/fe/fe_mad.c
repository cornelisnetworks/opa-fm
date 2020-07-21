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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/************************************************************************
* 
* FILE NAME
*       fe_mad.c
*
* DESCRIPTION
*       Fabric Executive MAD interface functions for collection of
*       information from the VIEO managers
* 
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   2/22/2000       Initial version for checkin to source control
* jrw   12/04/2001      Changes from code review    
*
***********************************************************************/

#include <stdlib.h>
#include "fe_main.h"
#include "fe_mad.h"
#include "fe.h"
#include "fe_trap_thread.h"
#include <fm_xml.h>
#include "if3.h"
#include "ib_sa.h"
#include "stl_pa_priv.h"

extern IBhandle_t  fdsa,fd_pm,fd_dm;
extern uint32_t fe_get_payload_length(uint8_t *netbuf);

extern IBhandle_t fd_p;
extern uint32_t g_fe_nodes_len;
extern FEXmlConfig_t fe_config;
extern uint8_t fe_paDebug;

#ifdef __VXWORKS__
extern int copyFile(char *src, char*dst, int compressFlag,long *fileSize);
#endif

#ifdef __VXWORKS__
#define FE_SMALL_BUF_SIZE (16*1024)
#else
#define FE_SMALL_BUF_SIZE (1024*1024) // more than we should ever need
#endif

#ifdef __LINUX__
extern void mai_umadt_read_kill(void);
extern void cal_ibaccess_global_shutdown(void);
#endif



uint32_t fe_vieo_init(uint8_t *logName)
{
    uint32_t rc = SUCCESS;
    static const char func[] = "fe_vieo_init"; 

    IB_ENTER(func, 0, 0, 0, 0); 

    fd_dm = INVALID_HANDLE;
    fdsa  = INVALID_HANDLE;
    
	// initialize MAI subsystem
	mai_set_num_end_ports( MIN(fe_config.subnet_size, MAI_MAX_QUEUED_DEFAULT) );
    mai_init(); 

    IB_LOG_INFO("Device = ", fe_config.hca);

    rc = if3_register_fe(fe_config.hca,fe_config.port,(void *)FE_SERVICE_NAME,FE_SERVICE_ID, IF3_REGFORCE_PORT,&fdsa);
    if (rc != VSTATUS_OK) {
        if (fe_config.debug) IB_LOG_INFINI_INFORC("Failed to register with IF3, will try later. rc:", rc);
    } else {
        rc = fe_if3_subscribe_sa();    
        if (rc != FSUCCESS) {
            IB_LOG_ERROR_FMT(__func__, "Failed to subscribe for traps in SA status: %u", rc);
        }

        // connect to Performance Manager
        if (pm_lid)
            rc = if3_lid_mngr_cnx(fe_config.hca,fe_config.port,MAD_CV_VFI_PM,pm_lid,&fd_pm);
        else
            rc = if3_sid_mngr_cnx(fe_config.hca,fe_config.port,(void *)STL_PM_SERVICE_NAME,STL_PM_SERVICE_ID,
                               MAD_CV_VFI_PM,&fd_pm);

        if (rc != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("Failed to open Performance Manager, will try later. rc:", rc);
            fd_pm = INVALID_HANDLE;
        }



#ifdef DEVICE_MANAGER   // not implemented yet
        // connect to Device Manager
        if (dm_lid)
            rc = if3_lid_mngr_cnx(fe_config.hca,fe_config.port,DM_IF3_MCLASS,dm_lid,&fd_dm);
        else
            rc = if3_sid_mngr_cnx(fe_config.hca,fe_config.port,DM_SERVICE_NAME,DM_SERVICE_ID,
                               DM_IF3_MCLASS,&fd_dm);

        if (rc != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("Failed to open Device Manager, will try later. rc:", rc);
            fd_dm = INVALID_HANDLE;
        }

#endif
    }

    IB_EXIT(func, 0); 
    return(rc);
}

void fe_shutdown(void)
{
    Status_t rc = SUCCESS; 
    NetError nerr; 
    
    IB_ENTER(__func__, 0, 0, 0, 0); 
    
    /* Close Performance Manager Connecttion */
    if (fd_pm != INVALID_HANDLE) {
        if3_close_mngr_cnx(fd_pm);
    } else {
        /* Complete termination of the RMPP connection. */
        (void)if3_close_mngr_rmpp_cnx(fd_pm);
    }
    
    /* Close Device Manager Connecttion */
    if (fd_dm != INVALID_HANDLE) {
        if3_close_mngr_cnx(fd_dm);
    }   
    
    /* Deregister FE Manager and Unsubscribe from SA    */
    if (fdsa != INVALID_HANDLE) {
        fe_if3_unsubscribe_sa(TRUE); 
        if3_deregister_fe(fdsa);
    } else {
        /* Complete termination of the RMPP connection. */
        (void)if3_close_mngr_rmpp_cnx(fdsa);
    }
    
    /* Shutdown the net and free the pool */
    nerr = fe_net_shutdown(); 
    if (nerr != NET_NO_ERROR) {
        IB_LOG_ERROR("Error shutting down libnet. nerr:", nerr);
    }
    
    rc = vs_pool_delete(&fe_pool);  
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("Error destroying fe memory pool. rc:", rc);
    }
    
    IB_EXIT(__func__, 0); 
    return;
}

uint32_t fe_send_packet(NetConnection *conn, uint8_t *buf, int32_t len)
{
    NetError    err;
    OOBPacket* packet; 		/* Pointer to out of band packet */
    uint32_t    rc = SUCCESS;

    IB_ENTER(__func__, buf, len, 0, 0);
	packet = (OOBPacket *) buf;
	packet->Header.Length = len - sizeof(OOBHeader);
	BSWAP_OOB_PACKET(packet);

    IB_LOG_INFO0("Sending packet");

    /* Do nothing if no conn */
    if (!conn) {
        IB_LOG_ERROR0("failed: no connection");
        return(FAILED); 
    }

    if (fe_net_send(conn,(void *)buf,len,&err)) {
        IB_LOG_ERROR("Error err:", err);
        rc = FAILED; 
    }

    IB_EXIT(__func__, rc);
     return(rc);
}

/***************************************************************************
*                      PASSTHROUGH HANDLERS                                *
***************************************************************************/

static uint32_t fe_passthrough_send_failure_response(uint8_t *netbuf, FE_ConnList *connList, IBhandle_t fd_sa, uint16 returnStatus)
{
	NetConnection *curConn; 					/* Pointer to the current connection */
	OOBPacket *ipacket; 						/* The incoming packet */
	OOBPacket *opacket; 						/* The out going packet */
	SA_MAD* saMad;								/* Pointer to the incoming mad data to forward */
	uint32_t rc = FE_SUCCESS;				    /* Whether or not the response to the FEC was successful */
    SA_MAD* returnMad; 							/* Outgoing mad to send */

	IB_ENTER(__func__, netbuf, connList, 0, 0);

	/* Sort out the incoming information */
	ipacket = (OOBPacket *) netbuf;
	saMad = (SA_MAD*) &(ipacket->MadData);
	curConn = connList->conn;

    /* Build the out of band packet and construct a response from the original request. */
    opacket = (OOBPacket *) g_fe_oob_send_buf;
    memset(opacket, 0, sizeof(OOBPacket));
    returnMad = (SA_MAD*) &(opacket->MadData);

    /* Fill the response mad's fields with the response we received */
    opacket->Header.HeaderVersion = STL_BASE_VERSION;
    MAD_SET_VERSION_INFO(returnMad, saMad->common.BaseVersion, saMad->common.MgmtClass, saMad->common.ClassVersion);
    MAD_SET_METHOD_TYPE(returnMad, saMad->common.mr.s.Method);
    MAD_SET_ATTRIB_ID(returnMad, saMad->common.AttributeID);
    returnMad->common.u.NS.Status.AsReg16 = returnStatus;
    returnMad->common.mr.s.R = 1;
    returnMad->common.TransactionID = saMad->common.TransactionID;
    returnMad->RmppHdr = saMad->RmppHdr;
    returnMad->SaHdr = saMad->SaHdr;

    /* Forward the response to the FEC*/
    if(fe_send_packet(curConn, g_fe_oob_send_buf, sizeof(OOBHeader) + IBA_SUBN_ADM_HDRSIZE)){
        IB_LOG_WARN_FMT(__func__, "Unable to send response packet");
        rc = FE_NO_COMPLETE;
    }

	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t fe_sa_passthrough(uint8_t *netbuf, FE_ConnList *connList, IBhandle_t fd_sa){
	NetConnection *curConn; 						/* Pointer to the current connection */
	OOBPacket *packet; 								/* The incoming packet */
	OOBHeader *messageHeader; 						/* Header on the incoming and outgoing message */
	SA_MAD* saMad;								 	/* Pointer to the incoming mad data to forward */
	int badRequest = FE_SUCCESS;					/* Whether or not the FEC was naughty with their request */

	IB_ENTER(__func__, netbuf, connList, 0, 0);

	/* Sort out the incoming information */
	packet = (OOBPacket *) netbuf;
	messageHeader = &(packet->Header);
	saMad = (SA_MAD*) &(packet->MadData);
	curConn = connList->conn;

	/* Validate the request, if we don't support it, mark it as bad and ignore it */
	switch(saMad->common.AttributeID){
	/* Attributes that only support SUBN_ADM_GET */
	case STL_SA_ATTR_CLASS_PORT_INFO:
		if(saMad->common.mr.s.Method != SUBN_ADM_GET){
			badRequest = FE_UNSUPPORTED;
		}
		break;
	
	/* Attributes that only support SUBN_ADM_SET */
	case STL_SA_ATTR_INFORM_INFO:
		if(saMad->common.mr.s.Method != SUBN_ADM_SET){
			badRequest = FE_UNSUPPORTED;
		}
		break;

	/* Attributes that only support SUBN_ADM_GETMULTI */
	case STL_SA_ATTR_MULTIPATH_GID_RECORD:
		if(saMad->common.mr.s.Method != SUBN_ADM_GETMULTI){
			badRequest = FE_UNSUPPORTED;
		}
		break;
	
	/* Attributes that only support SUBN_ADM_GETTRACETABLE */
	case STL_SA_ATTR_TRACE_RECORD:
		if(saMad->common.mr.s.Method != SUBN_ADM_GETTRACETABLE){
			badRequest = FE_UNSUPPORTED;
		}
		break;

	/* Attributes that only support SUBN_ADM_GET or SUBN_ADM_GETTABLE */
	case STL_SA_ATTR_NODE_RECORD:
	case STL_SA_ATTR_PORTINFO_RECORD:
	case STL_SA_ATTR_SC_MAPTBL_RECORD:
	case STL_SA_ATTR_SWITCHINFO_RECORD:
	case STL_SA_ATTR_LINEAR_FWDTBL_RECORD:
	case STL_SA_ATTR_MCAST_FWDTBL_RECORD:
	case STL_SA_ATTR_SMINFO_RECORD:
	case STL_SA_ATTR_LINK_RECORD:
	case STL_SA_ATTR_SERVICE_RECORD:
	case STL_SA_ATTR_P_KEY_TABLE_RECORD:
	case STL_SA_ATTR_PATH_RECORD:
	case STL_SA_ATTR_VLARBTABLE_RECORD:
	case STL_SA_ATTR_MCMEMBER_RECORD:
	case STL_SA_ATTR_SERVICEASSOCIATION_RECORD:
	case STL_SA_ATTR_INFORM_INFO_RECORD:
	case STL_SA_ATTR_SL2SC_MAPTBL_RECORD:
	case STL_SA_ATTR_SC2SL_MAPTBL_RECORD:
	case STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD:
	case STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD:
	case STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD:
	case STL_SA_ATTR_PGROUP_FWDTBL_RECORD:
	case STL_SA_ATTR_MULTIPATH_GUID_RECORD:
	case STL_SA_ATTR_MULTIPATH_LID_RECORD:
	case STL_SA_ATTR_CABLE_INFO_RECORD:
	case STL_SA_ATTR_VF_INFO_RECORD:
	case STL_SA_ATTR_PORT_STATE_INFO_RECORD:
	case STL_SA_ATTR_PORTGROUP_TABLE_RECORD:
	case STL_SA_ATTR_BUFF_CTRL_TAB_RECORD:
	case STL_SA_ATTR_FABRICINFO_RECORD:
	case STL_SA_ATTR_QUARANTINED_NODE_RECORD:
	case STL_SA_ATTR_CONGESTION_INFO_RECORD:
	case STL_SA_ATTR_SWITCH_CONG_RECORD:
	case STL_SA_ATTR_SWITCH_PORT_CONG_RECORD:
	case STL_SA_ATTR_HFI_CONG_RECORD:
	case STL_SA_ATTR_HFI_CONG_CTRL_RECORD:
	case STL_SA_ATTR_DG_NAME_RECORD:
	case STL_SA_ATTR_DG_MEMBER_RECORD:
	case STL_SA_ATTR_DT_MEMBER_RECORD:
	case STL_SA_ATTR_SWITCH_COST_RECORD:
		if(saMad->common.mr.s.Method != SUBN_ADM_GET && saMad->common.mr.s.Method != SUBN_ADM_GETTABLE){
			badRequest = FE_UNSUPPORTED;
		}
		break;
	default:
		IB_LOG_ERROR_FMT(__func__, "SA Attribute Unsupported: %02x", saMad->common.AttributeID);
		badRequest = FE_UNSUPPORTED;
		break;
	}

    /* If our request doesn't have enough data to hold an SA_MAD with no data, something is wrong with it */
    if(messageHeader->Length < (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER) + sizeof(SA_HDR))){
        IB_LOG_ERROR_FMT(__func__, "Malformed request - Size is too small: %d", messageHeader->Length);
        badRequest = FE_UNSUPPORTED;
        return badRequest;
    }

	/* Process the request */
	if(badRequest) {
        (void)fe_passthrough_send_failure_response(netbuf, connList, fd_sa, MAD_STATUS_SA_REQ_INVALID);
    } else {
		SA_MAD* returnMad; 							/* Outgoing mad to send */
		Mai_t returnMait; 							/* The Mai_t we are returned from the fabric query */
		uint8_t* returnBuffer = g_fe_recv_buf; 	    /* Buffer to fill with the fabric query results */
		uint32_t returnSize = STL_BUF_RECV_SIZE;	/* Size of the returned results */
		uint32_t returnStatus; 						/* Status of the fabric query */
        Status_t rc = VSTATUS_TIMEOUT;

		/* Send our mad and receive the response */
		if(fd_sa == INVALID_HANDLE ||
           (rc = if3_mngr_send_passthru_mad(fd_sa, saMad, messageHeader->Length, &returnMait, returnBuffer, &returnSize, &returnStatus, NULL, NULL)) != VSTATUS_OK){
            returnStatus = (rc == VSTATUS_TIMEOUT) ? STL_MAD_STATUS_STL_SA_UNAVAILABLE : MAD_STATUS_BUSY;
            if (returnStatus == STL_MAD_STATUS_STL_SA_UNAVAILABLE)
            	IB_LOG_ERROR_FMT(__func__, "Sending request to SA failed. Mad error code: (%x) SA Unavailable", returnStatus);
            else
            	IB_LOG_ERROR_FMT(__func__, "Sending request to SA failed. Mad error code: (%x) Busy", returnStatus);
            /* Mad was not sent, so use generic Mad status error */
            (void)fe_passthrough_send_failure_response(netbuf, connList, fd_sa, (uint16)returnStatus);
            return FE_NO_COMPLETE;
		}

		/* If the returnSize is greater than the size of our buffer, bail */
		if(returnSize > STL_BUF_OOB_SEND_SIZE){
			IB_LOG_ERROR_FMT(__func__, "Returned data is too large for network buffer. Size: %d bytes", returnSize);
            returnStatus = MAD_STATUS_SA_TOO_MANY_RECORDS;
			badRequest = FE_NO_COMPLETE;
			returnSize = 0;
		}

		/* Build the out of band packet and copy the response into it */
		/* There's no need to bswap the data, as that will be done by the FEC on the other side */
		packet = (OOBPacket *) g_fe_oob_send_buf;
		memset(packet, 0, sizeof(OOBPacket));
		returnMad = (SA_MAD*) &(packet->MadData);

		/* Fill the response mad's fields with the response we received */
		packet->Header.HeaderVersion = STL_BASE_VERSION;
		MAD_SET_VERSION_INFO(returnMad, returnMait.base.bversion, returnMait.base.mclass, returnMait.base.cversion);
		MAD_SET_METHOD_TYPE(returnMad, returnMait.base.method);
		MAD_SET_ATTRIB_ID(returnMad, returnMait.base.aid);
		returnMad->common.u.NS.Status.AsReg16 = returnStatus;
		returnMad->common.mr.s.R = 1;
		returnMad->common.TransactionID = saMad->common.TransactionID; /* We have to set this from our own mad, as we 
																			use different tids than the fabric */

		/* Copy the response data into our packet */
		returnMad->RmppHdr = *((RMPP_HEADER*)returnMait.data);
		returnMad->SaHdr = *((SA_HDR*)(returnMait.data + sizeof(RMPP_HEADER)));
		memcpy(&(returnMad->Data), returnBuffer, returnSize);

		/* Forward the response to the FEC*/
		if(fe_send_packet(curConn, g_fe_oob_send_buf, sizeof(OOBHeader) + IBA_SUBN_ADM_HDRSIZE + returnSize)){
			IB_LOG_WARN_FMT(__func__, "Unable to send response packet");
			badRequest = FE_NO_COMPLETE;
			return badRequest;
		}
	}

	IB_EXIT(__func__, badRequest);
	return badRequest;
}

uint32_t fe_pa_passthrough(uint8_t* netbuf, FE_ConnList* connList, IBhandle_t fd_sa){
	NetConnection *curConn; 						/* Pointer to the current connection */
	OOBPacket *packet; 								/* The incoming packet */
	OOBHeader *messageHeader; 						/* Header on the incoming and outgoing message */
	SA_MAD* saMad;								 	/* Pointer to the incoming mad data to forward */
	int badRequest = FE_SUCCESS;					/* Whether or not the FEC was naughty with their request */
	SA_MAD* returnMad; 								/* Outgoing mad to send */
	Mai_t returnMait; 								/* The Mai_t we are returned from the fabric query */
	uint8_t* returnBuffer = g_fe_recv_buf; 		    /* Buffer to fill with the fabric query results */
	uint32_t returnSize = STL_BUF_RECV_SIZE;		/* Size of the returned results */
	uint32_t returnStatus; 							/* Status of the fabric query */
	uint32_t dataLength;
    Status_t rc = VSTATUS_TIMEOUT;

	IB_ENTER(__func__, netbuf, connList, 0, 0);

	/* Sort out the incoming information */
	packet = (OOBPacket *) netbuf;
	messageHeader = &(packet->Header);
	saMad = (SA_MAD*) &(packet->MadData);
	curConn = connList->conn;
	dataLength = messageHeader->Length - IBA_SUBN_ADM_HDRSIZE;

	/* If our request doesn't have enough data to hold an SA_MAD with no data, something is wrong with it */
	if(messageHeader->Length < (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER) + sizeof(SA_HDR))){
		IB_LOG_ERROR_FMT(__func__, "Malformed request - Size is too small: %d", messageHeader->Length);
		badRequest = FE_UNSUPPORTED;
		return badRequest;
	}

	/* Send our mad and receive the response */
	if(fd_sa == INVALID_HANDLE ||
       (rc = if3_mngr_send_passthru_mad(fd_sa, saMad, dataLength, &returnMait, returnBuffer, &returnSize, &returnStatus, NULL, NULL)) != VSTATUS_OK){
        returnStatus = (rc == VSTATUS_TIMEOUT) ? STL_MAD_STATUS_STL_PA_UNAVAILABLE : MAD_STATUS_BUSY;
        if (returnStatus == STL_MAD_STATUS_STL_PA_UNAVAILABLE)
        	IB_LOG_ERROR_FMT(__func__, "Sending request to performance manager failed. Mad error code: (%x) PA Unavailable", returnStatus);
        else
        	IB_LOG_ERROR_FMT(__func__, "Sending request to performance manager failed. Mad error code: (%x) Busy", returnStatus);
        /* Mad was not sent, so use generic Mad status error */
        (void)fe_passthrough_send_failure_response(netbuf, connList, fd_sa, (uint16)returnStatus);
        return FE_NO_COMPLETE;
	}

    /* If the returnSize is greater than the size of our buffer, bail */
    if(returnSize > STL_BUF_OOB_SEND_SIZE){
        IB_LOG_ERROR_FMT(__func__, "Returned data is too large for network buffer. Size: %d bytes", returnSize);
        returnStatus = MAD_STATUS_SA_TOO_MANY_RECORDS;
        badRequest = FE_NO_COMPLETE;
        returnSize = 0;
    }

	/* Build the out of band packet and copy the response into it */
	/* There's no need to bswap the data, as that will be done by the FEC on the other side */
	packet = (OOBPacket *) g_fe_oob_send_buf;
	memset(packet, 0, sizeof(OOBPacket));
	returnMad = (SA_MAD*) &(packet->MadData);

	/* Fill the response mad's fields with the response we received */
	packet->Header.HeaderVersion = STL_BASE_VERSION;
	MAD_SET_VERSION_INFO(returnMad, returnMait.base.bversion, returnMait.base.mclass, returnMait.base.cversion);
	MAD_SET_METHOD_TYPE(returnMad, returnMait.base.method);
	MAD_SET_ATTRIB_ID(returnMad, returnMait.base.aid);
	returnMad->common.u.NS.Status.AsReg16 = returnStatus;
	returnMad->common.mr.s.R = 1;
	returnMad->common.TransactionID = saMad->common.TransactionID; /* We have to set this from our own mad, as we 
																			use different tids than the fabric */

	/* Copy the response data into our packet */
	returnMad->RmppHdr = *((RMPP_HEADER*)returnMait.data);
	returnMad->SaHdr = *((SA_HDR*)(returnMait.data + sizeof(RMPP_HEADER)));
	memcpy(&(returnMad->Data), returnBuffer, returnSize);

	/* Forward the response to the FEC*/
	if(fe_send_packet(curConn, g_fe_oob_send_buf, sizeof(OOBHeader) + IBA_SUBN_ADM_HDRSIZE + returnSize)){
		IB_LOG_WARN_FMT(__func__, "Unable to send response packet");
		badRequest = FE_NO_COMPLETE;
		return badRequest;
	}

	IB_EXIT(__func__, badRequest);
	return badRequest;
}

uint32_t fe_unsolicited(FE_ConnList *clist, uint32_t *conn_state) { 
    uint32_t i, rc = FE_SUCCESS; 
    uint32_t found = 0, count = 0; 
    int32_t len = 0;
    STL_NOTICE connStateTrap; 
    STL_NOTICE *noticep;
    FE_ConnList *tlist; 
    FE_Trap_t traps, *curr; 
#ifndef IB_STACK_OPENIB
    FE_Trap_t *temp;
#endif
    uint64_t tid; 
    ManagerInfo_t *mi; 
    SA_MAD *noticeMad; 
    OOBPacket *packet; 
    
    IB_ENTER(__func__, conn_state, 0, 0, 0); 
    tlist = clist; 
    
#ifdef IB_STACK_OPENIB
    // early sanity check on OFED to avoid locking the trap thread
    if (!fe_trap_thread_get_trapcount()
        && *conn_state != CONN_LOST && *conn_state != CONN_ESTABLISH) {
        IB_EXIT(__func__, rc); 
        return (rc);
    }
    
    // stop the trap thread and grab the traps.  due to the non-OFED semantics
    // of this function, we store the first trap on the stack
    {
        FE_Trap_t *ptraps; 
        fe_trap_thread_pause(); 
        fe_trap_thread_get_traplist(&ptraps, &found); 
        if (found) {
            memcpy(&traps, ptraps, sizeof(FE_Trap_t));
        }
    }
#else
    (void)fe_if3_get_traps(&traps, &found); 
#endif
    
    if (*conn_state == CONN_LOST) {
        count++;
    }
    
    if (*conn_state == CONN_ESTABLISH) {
        count++;
    }
    
    if (rc == FE_SUCCESS) {
        count = count + found;
    }
    
    if (count == 0) {
        /* no Notices to send */
#ifdef IB_STACK_OPENIB
        fe_trap_thread_resume(); 
#endif
        IB_EXIT(__func__, rc); 
        return (rc);
    }
    

    // find the handle stuff
    if ((rc = if3_mngr_locate_minfo(fdsa, &mi))) {
        IB_EXIT(__func__, rc); 
        return rc;
    }

    // allocate a tid
    ; 
    if ((rc = mai_alloc_tid(mi->fds, mi->mclass, &tid))) {
        IB_LOG_ERRORRC("unable to allocate tid rc:", rc);
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    //    
    // process the traps
    if (found) {
        curr = &traps;
        
        for (i = 0; i < found; i++, tid = mai_increment_tid(tid)) {
#ifndef IB_STACK_OPENIB
            temp = curr; 
#endif
            
            // build the out of band packet and copy the response into it.
            // there's no need to bswap the data, as that will be done by
            // the FEC on the other side
            packet = (OOBPacket *)g_fe_oob_send_buf; 
            memset(packet, 0, sizeof(OOBPacket)); 
            noticeMad = (SA_MAD *)&(packet->MadData); 
            
            // initialize SA MAD header fields    
            SA_MAD_SET_HEADER(noticeMad, SM_KEY, 0);             
            // initialize the commom mad header fields
            packet->Header.HeaderVersion = STL_BASE_VERSION; 
            MAD_SET_VERSION_INFO(noticeMad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
            MAD_SET_METHOD_TYPE(noticeMad, MAD_CM_REPORT); 
            MAD_SET_ATTRIB_ID(noticeMad, SA_NOTICE); 
            MAD_SET_TRANSACTION_ID(noticeMad, tid);             
            // initialize RMPP fields
            noticeMad->RmppHdr.RmppVersion = 1;
                         
            // copy the notice data into our packet and convert to network byte order.
            // Byte swapping to network byte order for the rest of the packet, will
            // be done by the fe_send_packet() routine.  
            (void)BSWAPCOPY_STL_NOTICE(&curr->notice, (STL_NOTICE *)noticeMad->Data);
            
            len = sizeof(OOBHeader) + IBA_SUBN_ADM_HDRSIZE + sizeof(STL_NOTICE);

            // send the notice to all FECs
            while (tlist) {
                if ((rc = fe_send_packet(tlist->conn, g_fe_oob_send_buf, len))) {
                    IB_LOG_ERROR("fe_send_packet fail in fe_unsolicited rc:", rc);
                }

                // the fe_send_packet() routine byte swaps the headers of the
                // OOB packet, so convert them back to host byte order before
                // sending packet again.
                BSWAP_OOB_PACKET(packet);
                
                tlist = tlist->next;
            }
            
            curr = curr->next;             
            // relegate memory management to trap thread functions on OFED
#ifndef IB_STACK_OPENIB
            if (i > 0) {
                vs_pool_free(&fe_pool, (void *)temp);
            }
#endif
        }
    }

    switch (*conn_state) {
    case CONN_LOST:
    case CONN_ESTABLISH:
        tlist = clist;
         
        // use 64/65 trap to notify the FEC that the connection to the SM/SA
        // has gone down/reestablished
        tid = mai_increment_tid(tid);

        // build the out of band packet and copy the response into it.
        // there's no need to bswap the data, as that will be done by
        // the FEC on the other side
        memset(&connStateTrap, 0, sizeof(connStateTrap));
        packet = (OOBPacket *)g_fe_oob_send_buf; 
        memset(packet, 0, sizeof(OOBPacket)); 
        noticeMad = (SA_MAD *)&(packet->MadData); 

        // initialize SA MAD header fields    
        SA_MAD_SET_HEADER(noticeMad, SM_KEY, 0);
                     
        // initialize the commom mad header fields
        packet->Header.HeaderVersion = STL_BASE_VERSION; 
        MAD_SET_VERSION_INFO(noticeMad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
        MAD_SET_METHOD_TYPE(noticeMad, MAD_CM_REPORT); 
        MAD_SET_ATTRIB_ID(noticeMad, SA_NOTICE); 
        MAD_SET_TRANSACTION_ID(noticeMad, tid);             
        // initialize RMPP fields
        noticeMad->RmppHdr.RmppVersion = 1;

        // initialize the notice fields
        connStateTrap.Attributes.Generic.u.s.IsGeneric = 1;
        connStateTrap.Attributes.Generic.u.s.Type = NOTICE_TYPE_INFO;
        connStateTrap.Attributes.Generic.u.s.ProducerType = NOTICE_PRODUCERTYPE_CLASSMANAGER;
        connStateTrap.Attributes.Generic.TrapNumber = (*conn_state == CONN_LOST) ? MAD_SMT_PORT_DOWN: MAD_SMT_PORT_UP;
        connStateTrap.IssuerLID	= mi->saLid;
        connStateTrap.Stats.s.Toggle = 0;
        connStateTrap.Stats.s.Count = 0;

        // copy the notice data into our packet and convert to network byte order.
        // Byte swapping to network byte order for the rest of the packet, will
        // be done by the fe_send_packet() routine.  
        (void)BSWAPCOPY_STL_NOTICE(&connStateTrap, (STL_NOTICE *)noticeMad->Data);

        // Now construct IssurerGID
        // 
        // Note, we don't copy the GIDs till after we bswap the rest of the notice.
        // This is done in order to remain consistant with the unique handling of
        // GIDs throughout the FM (GIDs are stored in network byte order).
        noticep = (STL_NOTICE *)noticeMad->Data;
		(void) memcpy((void *) &noticep->IssuerGID.Raw[0], (void *) &mi->gidPrefix, 8);
		(void) memcpy((void *) &noticep->IssuerGID.Raw[8], (void *) &mi->guid, 8);
		*(uint64_t *) &noticep->IssuerGID.Raw[0] = ntoh64(*(uint64_t *) &noticep->IssuerGID.Raw[0]);
		*(uint64_t *) &noticep->IssuerGID.Raw[8] = ntoh64(*(uint64_t *) &noticep->IssuerGID.Raw[8]);

        len = sizeof(OOBHeader) + IBA_SUBN_ADM_HDRSIZE + sizeof(STL_NOTICE);

        // send the notice to all FECs
        while (tlist) {
            if ((rc = fe_send_packet(tlist->conn, g_fe_oob_send_buf, len))) {
                IB_LOG_ERROR("fe_send_packet fail in fe_unsolicited rc:", rc);
            }
            // the fe_send_packet() routine byte swaps the headers of the
            // OOB packet, so convert them back to host byte order before
            // sending packet again.
            BSWAP_OOB_PACKET(packet);

            tlist = tlist->next;
        }
        break;

    default:
        break;
    }
    
    PCHK(rc, FAILED, "fe_unsolicited"); 
    
    *conn_state = CONN_UNCHANGED; 
    
#ifdef IB_STACK_OPENIB
    fe_trap_thread_free_traps(); 
    fe_trap_thread_resume(); 
#endif
    
    IB_EXIT(__func__, rc); 
    return (rc);
}

