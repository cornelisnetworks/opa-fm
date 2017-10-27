/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/***********************************************************************
*                                                                      *
* FILE NAME                                                            *
*    service.c                                                         *
*                                                                      *
* DESCRIPTION                                                          *
*    Code to register and delete and query SA for service associated   *
* with interface 3.                                                    *
*                                                                      *
*                                                                      *
*   PJG     06/17/02    PR2397.  Added a dummy #define of              *
*                                  Endian_ClassPortInfo for ATI builds *
*                                                                      *
************************************************************************/
#include <stdio.h>
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "mal_g.h"
#include "ib_sa.h"
#include "ib_status.h"
#include <if3.h>
#include <if3_def.h> 
#include "ib_macros.h"

#if defined( __LINUX__)
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#endif

#ifndef NULL
#define NULL (0)
#endif



/*=========================================================================
 * Local defines
 *=========================================================================
 */

#define	SA_KEY			0
#define	SM_KEY			0

#define	SVC_SA_GET_SERV_CMASK	(SR_COMPONENTMASK_NAME | SR_COMPONENTMASK_GID | SR_COMPONENTMASK_ID)
//#define	SVC_SA_GET_SERV_CMASK	0xe0  /* service name, service GID, serivce ID*/

#define	SVC_SA_DEL_SERV_CMASK	(SR_COMPONENTMASK_NAME | SR_COMPONENTMASK_GID | SR_COMPONENTMASK_ID)
//#define	SVC_SA_DEL_SERV_CMASK	0xe0  /* service name, service GID, serivce ID*/

#define	SVC_SA_GET_PATH_CMASK  (PR_COMPONENTMASK_SGID | PR_COMPONENTMASK_DGID | PR_COMPONENTMASK_PATHS)
//#define	SVC_SA_GET_PATH_CMASK   0x100c /* SGID, DGID, numPath                  */


#define MAX_RECORDS_SUPPORTED (8)            /* for querries                */
#define MAX_BUFFER (MAX_RECORDS_SUPPORTED*sizeof(ServiceRecord_t)) 


static void
BuildRecord(IB_SERVICE_RECORD	*srp, 
            uint8_t             *servName, 
            uint64_t            servID, 
            uint8_t             *inBuff, 
            ManagerInfo_t       *mi)
{
    srp->ServiceLease = 0xffffffff;                                             
    strncpy((void *)srp->ServiceName, (void *)servName, sizeof(srp->ServiceName) - 1);                              
    srp->RID.ServiceGID.Type.Global.SubnetPrefix = mi->gidPrefix; 
    srp->RID.ServiceGID.Type.Global.InterfaceID = mi->guid.guid[0]; 
    srp->RID.ServiceID = servID; 
	srp->Reserved = 0;
    /* servicep_key will be zero */
}
 
Status_t
if3_mngr_get_sa_classportInfo (ManagerInfo_t *fp)
{
	Status_t		status; 
    SA_MAD mad;
	uint8_t buffer[STL_SA_DATA_LEN];
	uint32_t madRc, bufferLength =STL_SA_DATA_LEN;

	IB_ENTER(__func__,fp,0,0,0); 
    
    memset ((void*)buffer, 0, sizeof(buffer)); 
    
    // initialize the commom mad header fields of the SA MAD
    memset ((void *)(&mad),0,sizeof(mad)); 
    
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GET); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_CLASS_PORT_INFO);

#if defined(IB_STACK_OPENIB)
	if (ib_refresh_devport() != VSTATUS_OK) {
		IB_LOG_WARN0 ("can't read local port pkeys");
	}
#endif
    
    // send service record to SA and wait for result
    if ( (status = if3_mngr_send_mad(fp->fdr, &mad, 0, buffer, &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
		IB_LOG_INFINI_INFORC("could not talk with SA (SM/SA may have moved) rc:", status);
		goto done;
	}

    if (madRc != 0) {
        status = VSTATUS_NOT_FOUND; 
        IB_LOG_INFINI_INFO(" Invalid status retrieving ClassPortInfo resp:", madRc); 
        goto done;
    }
    
    (void)BSWAP_IB_CLASS_PORT_INFO((IB_CLASS_PORT_INFO *) buffer);
    memcpy(&fp->cpi, buffer, sizeof(IB_CLASS_PORT_INFO));

    fp->cpi_valid = 1;

 done:
	IB_EXIT(__func__, status);
	return (status);
}

Status_t
if3_mngr_get_port_guid (ManagerInfo_t *fp)
{
	Status_t		status; 
    SA_MAD mad; 
	STL_NODE_RECORD nr, *pnr;
    uint8_t buffer[STL_SA_DATA_LEN];
	uint32_t madRc, bufferLength = STL_SA_DATA_LEN;

	IB_ENTER(__func__,fp,0,0, 0);

	memset ((void*)buffer, 0, sizeof(buffer));
    if (fp->guidIsValid) {
        IB_LOG_INFOLX("Guid already retrieved ", fp->guid.guid[0]); 
        status = VSTATUS_OK; 
        goto done;
    }
    
    // initialize the commom mad header fields of the SA MAD
    memset ((void*)(&mad), 0, sizeof(mad)); 
    
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GET); 
    MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, STL_SA_ATTR_NODE_RECORD);
     
    // setup lid to search on at SA
	pnr = (STL_NODE_RECORD *)mad.Data;
    pnr->RID.LID = fp->slid;
	pnr->Reserved = 0;

    // initialize SA MAD payload
	(void)BSWAP_STL_NODE_RECORD(pnr);

    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, STL_NODE_RECORD_COMP_LID); 

    // send service record to SA and wait for result
    if ((status = if3_mngr_send_mad(fp->fdr, 
                                    &mad, sizeof(STL_NODE_RECORD), 
                                    buffer, 
                                    &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFO_FMT(__func__, 
                               "Was not able to communicate with SA (SM/SA may have moved), rc:%d ", status); 
        goto done;
    }

    if (madRc != 0) {
        status = VSTATUS_NOT_FOUND; 
        IB_LOG_INFO("SA does not have Guid for port", madRc); 
        goto done;
    }
        
    // retrieve response data
    (void)BSWAPCOPY_STL_NODE_RECORD((STL_NODE_RECORD *)buffer, &nr);

	memcpy ((void *)&fp->guid, (void *)&nr.NodeInfo.PortGUID, sizeof(uint64_t));

	IB_LOG_INFOLX(__func__,fp->guid.guid[0]);

	fp->guidIsValid = 1;
		
 done:
	IB_EXIT(__func__, status);
	return (status);
}

Status_t
if3_mngr_reg_service (IBhandle_t fd, uint8_t *servName, uint64_t servID)
{
	Status_t status; 
    IB_SERVICE_RECORD sr; 
    SA_MAD mad;
	ManagerInfo_t *mi;
	uint8_t buffer[STL_SA_DATA_LEN];
	uint32_t madRc, bufferLength =STL_SA_DATA_LEN;

	IB_ENTER(__func__,fd, servName, servID,0); 
    
    // find the handle stuff
    status = if3_mngr_locate_minfo(fd, &mi);
    if (status) {
        IB_EXIT(__func__, status); 
        return status;        
    }  
    
    if (servName == NULL) {
        status = VSTATUS_ILLPARM; 
        IB_EXIT(__func__, status); 
        return status;        
    }
    
    memset ((void*)&sr, 0, sizeof(sr)); 
    memset ((void *)(&mad),0,sizeof(mad)); 
    
    // verify access to the SA by querying the the Port GUID of the SA
    status = if3_mngr_get_port_guid(mi);
    if (status != VSTATUS_OK) {
        IB_LOG_INFO("could not get GUID info from SA (SM/SA may have moved) ", status); 
        goto done;
    }
    
    // build service record
    BuildRecord(&sr,servName,servID, mad.Data,mi);
     
    // initialize SA MAD payload
    memcpy(mad.Data, &sr, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)mad.Data);
    
    // initialize SA MAD header fields, match on ID and GID    
    SA_MAD_SET_HEADER(&mad, SM_KEY, SVC_SA_DEL_SERV_CMASK); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_SET); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD); 
    
    // send service record to SA and wait for result
    if ((status = if3_mngr_send_mad(fd, &mad, sizeof(sr), 
                                    buffer, 
                                    &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFORC("could not talk with SA (SM/SA may have moved) rc:", status); 
        goto done;
    }
    
    
    if (madRc != 0) {
        IB_LOG_ERROR("failed at SA resp:", madRc); 
        status = VSTATUS_BAD;
    }
	
 done:
	IB_LOG_INFO(__func__, status);
	return (status);
}

Status_t
if3_mngr_del_service(IBhandle_t fd, uint8_t *servName, uint64_t servID, uint32_t mode)
{
	Status_t status; 
    IB_SERVICE_RECORD sr; 
    SA_MAD mad;
	uint8_t buffer[STL_SA_DATA_LEN];
	uint32_t madRc, bufferLength = STL_SA_DATA_LEN;
    uint32_t count,found=0;
    
    IB_ENTER(__func__,fd, servName, servID,mode); 
    
next:
    // find the service record 
    status = if3_mngr_query_service(fd, servName, servID, mode, &sr,&count); 
    if (status || count == 0) {
        if (found) {
            status = VSTATUS_OK; 
            IB_LOG_INFO("deleted service records", found); 
            IB_EXIT(__func__, status); 
        } else {
            status = VSTATUS_NOT_FOUND;		
            IB_EXIT(__func__, status); 
        }
        return status;        
    }  
    
    memset ((void*)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields, match on ID and GID    
    SA_MAD_SET_HEADER(&mad, SM_KEY, SVC_SA_DEL_SERV_CMASK); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_DELETE); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD); 
    
    // initialize SA MAD payload
    memcpy(mad.Data, &sr, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)mad.Data);

    if ((status = if3_mngr_send_mad(fd, &mad, sizeof(sr), 
                                    buffer, 
                                    &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFORC("could not talk with SA (SM/SA may have moved) rc:", status); 
        goto done;
    }
    
    if (madRc != 0) {
        IB_LOG_ERROR("SA did not find record resp:", madRc); 
        
        if (found) 
            status = VSTATUS_OK; 
        else 
            status = VSTATUS_NOT_FOUND;
    } else {
        if (count > 1) {
            found++; 
            goto next;
        }
    }
	
 done:
	IB_LOG_INFO(__func__, status);

	return (status);
}

Status_t
if3_mngr_query_service (IBhandle_t fd, 
		  uint8_t *servName, 
		  uint64_t servID,
		  uint32_t mode,
		  IB_SERVICE_RECORD *serviceFoundp,
		  uint32_t *count)
{
        Status_t		status; 
    IB_SERVICE_RECORD sr; 
    SA_MAD mad;
	ManagerInfo_t           *mi;
	uint8_t buffer[MAX_BUFFER];
	uint32_t madRc, bufferLength =MAX_BUFFER;

	IB_ENTER(__func__, servName, servID,serviceFoundp,count); 
    
    // find the handle stuff
    status = if3_mngr_locate_minfo(fd, &mi);
    if (status) {
        IB_EXIT(__func__, status); 
        return status;        
    }  

	if(servName       == NULL ||
	   serviceFoundp  == NULL ||
	   count          == NULL ||
	   (mode != IF3_PORT_SERVICE && mode != IF3_FABRIC_SERVICE )) {
	    status = VSTATUS_ILLPARM;
	    IB_EXIT(__func__,status); 
	    return status;        
	}

	*count = 0; 
    
    memset ((void*)(&sr),0,sizeof(sr));

	status = if3_mngr_get_port_guid(mi);
    if (status != VSTATUS_OK) {
        IB_LOG_INFINI_INFO("Error getting GUID rc:", status); 
        goto done;
    }
    
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields    
    if (mode == IF3_PORT_SERVICE) {
        IB_LOG_VERBOSE("Querying port service mode:", mode); 
        // get only services registered with this port
        SA_MAD_SET_HEADER(&mad, SM_KEY, SVC_SA_GET_SERV_CMASK);
    } else {
        IB_LOG_VERBOSE("Querying fabric service mode:", mode); 
        // get all services registered in the fabric
        SA_MAD_SET_HEADER(&mad, SM_KEY, SVC_SA_GET_SERV_CMASK);
    }
    
    // build service record
    BuildRecord(&sr,servName,servID, mad.Data,mi); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GETTABLE); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD); 
    
    // initialize SA MAD payload
    memcpy(mad.Data, &sr, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)mad.Data);
    
    // send request to SA and wait for result
    if ((status = if3_mngr_send_mad(fd, &mad, sizeof(sr), 
                                    buffer, 
                                    &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFORC("could not talk with SA (SM/SA may have moved) rc:", status); 
        goto done;
    }
    
    if (madRc != 0) {
        IB_LOG_INFO("No records found at SA, status=", madRc); 
        status = VSTATUS_NOT_FOUND; 
        goto done;
    }

	*count = (uint32_t)(bufferLength / (sizeof(sr))); 

    // retrieve response data
    memcpy(serviceFoundp, buffer, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)serviceFoundp);
	
	IB_LOG_INFO("Data length returned ", bufferLength);
	IB_LOG_INFO("Number of records    ",*(count));

 done:

	IB_EXIT(__func__, status);
	return (status);
}

Status_t
if3_mngr_query_srv_path(IBhandle_t fd, IB_SERVICE_RECORD *srp, STL_LID *lid , uint16_t *sl)
{ 
    Status_t status; 
    IB_PATH_RECORD pr; 
    SA_MAD mad; 
    uint8_t buffer[STL_SA_DATA_LEN]; 
    uint32_t madRc, bufferLength = STL_SA_DATA_LEN; 
    ManagerInfo_t *mi;
    
    IB_ENTER(__func__, fd, srp, lid, 0); 
    
    // find the handle stuff
    status = if3_mngr_locate_minfo(fd, &mi); 
    if (status) {
        IB_EXIT(__func__, status); 
        return status;        
    }  
    
    if (srp == NULL || lid == NULL  || sl == NULL) {
        status = VSTATUS_ILLPARM; 
        IB_EXIT(__func__, status); 
        return status;
    }
    
    memset((void *)(&pr), 0, sizeof(pr)); 
    
    //    
    // construct source GID.
    pr.SGID.Type.Global.SubnetPrefix = mi->gidPrefix; 
    pr.SGID.Type.Global.InterfaceID = mi->guid.guid[0]; 
    pr.DGID = srp->RID.ServiceGID; 
    
    // set the specified max number of path records to be returned from
    // the SA
    pr.NumbPath = 1; 
    
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, SVC_SA_GET_PATH_CMASK); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GET); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD); 
    
    // initialize SA MAD payload
    (void)BSWAPCOPY_IB_PATH_RECORD(&pr, (IB_PATH_RECORD *)mad.Data); 
    
    // send service record to SA and wait for result
    if ((status = if3_mngr_send_mad(fd, &mad, sizeof(IB_PATH_RECORD), 
                                    buffer, 
                                    &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFORC("could not talk with SA (SM/SA may have moved) rc:", status); 
        goto done;
    }
    
    if (madRc != 0) {
        IB_LOG_INFINI_INFOX("query to SA for server path failed with resp:", madRc); 
        status = VSTATUS_BAD;
    } else {
        // retrieve response data
        (void)BSWAPCOPY_IB_PATH_RECORD((IB_PATH_RECORD *)buffer, &pr); 
        
        *lid = pr.DLID; 
        *sl  = pr.u2.s.SL; 
        if (IB_LOG_IS_INTERESTED(VS_LOG_VERBOSE)) {
            IB_LOG_VERBOSE(" PathREC : ", madRc); 
            IB_LOG_VERBOSEX("    DLid : ", *lid); 
            IB_LOG_VERBOSEX("    SLid : ", pr.SLID); 
            IB_LOG_VERBOSE("      SL : ", *sl);
        }
        
    }
    
done:
    IB_LOG_INFO(__func__, status); 
    return (status);
}

