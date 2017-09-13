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

/************************************************************************
*                                                                      *
* FILE NAME                                                            *
*    vfi.c                                                             *
*                                                                      *
* DESCRIPTION                                                          *
*    Library calls for interface 3                                     *
*                                                                      *
*                                                                      *
*                                                                      *
*                                                                      *
************************************************************************/

#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "mal_g.h"
#include "ib_sa.h"
#include "ib_status.h"
#include <if3.h>
#include "ib_macros.h"
#include "if3_def.h"
#include "vfi_g.h"
#include "cs_log.h"

#if defined( __LINUX__)
    #include <stdio.h>
    #include <signal.h>
    #include <unistd.h>
    #include <pthread.h>
    #include <string.h>
#endif

#ifndef NULL
    #define NULL (0)
#endif

Status_t
vfi_MngrQueryService(ManagerInfo_t * mi, VFI_SERVICE_RECORD * service,
                     uint32_t mask, int *count, VfiSvcRecCmp_t cmp);

/*=========================================================================
 * Local defines
 *=========================================================================
 */

#define	SA_KEY			0

#define	SVC_SA_GETA_SERV_CMASK	(SR_COMPONENTMASK_NAME | SR_COMPONENTMASK_ID)
//#define	SVC_SA_GETA_SERV_CMASK	0xa0	/* service name, serivce ID */

#define	SVC_SA_GET_SERV_CMASK	(SR_COMPONENTMASK_NAME | SR_COMPONENTMASK_GID | SR_COMPONENTMASK_ID)
//#define	SVC_SA_GET_SERV_CMASK	0xe0	/* service name, service GID, serivce ID */

#define	SVC_SA_DELA_SERV_CMASK	(SR_COMPONENTMASK_NAME | SR_COMPONENTMASK_ID)
//#define	SVC_SA_DELA_SERV_CMASK	0xa0	/* service name, serivce ID */

#define	SVC_SA_DEL_SERV_CMASK	(SR_COMPONENTMASK_NAME | SR_COMPONENTMASK_GID | SR_COMPONENTMASK_ID)
//#define	SVC_SA_DEL_SERV_CMASK	0xe0	/* service name, service GID, serivce ID */

//#define INVALID_MASK_BITS ~(0xFE)

//#define	SVC_SA_GET_PATH_CMASK  (PR_COMPONENTMASK_SGID | PR_COMPONENTMASK_DGID | PR_COMPONENTMASK_PATHS)
#define	SVC_SA_GET_PATH_CMASK  0x000000000000100cull	/* SGID, DGID, numPath */

typedef struct {
    int             cnt;
    VFI_SERVICE_RECORD *sr;
    int             status;
	VfiSvcRecCmp_t  cmp;
	uint8_t         data[sizeof(VFI_SERVICE_RECORD)];
	uint8_t         dlen;
} cb_ctx_t;


static void
BuildRecordGID(VFI_SERVICE_RECORD * srp, ManagerInfo_t * mi)
{
	srp->RID.ServiceGID.Type.Global.SubnetPrefix = mi->gidPrefix;
	srp->RID.ServiceGID.Type.Global.InterfaceID = mi->guid.guid[(int)(mi->vfi_guid)];
	srp->Reserved = 0;
}

Status_t
vfi_GetPortGuid(ManagerInfo_t * fp, uint32_t gididx)
{
    Status_t status; 
    SA_MAD mad;
    STL_NODE_RECORD nr, *pnr;
    uint8_t buffer[10 * STL_SA_DATA_LEN];
    uint32_t madRc, bufferLength = (10 * STL_SA_DATA_LEN);

    IB_ENTER(__func__, fp, gididx, 0, 0);

	memset ((void*)(&buffer), 0, sizeof(buffer));
    if (gididx > VFI_MAX_PORT_GUID) {
        IB_LOG_ERROR_FMT(__func__, "Illegal guid param gididx %d", gididx);
        IB_EXIT(__func__, VSTATUS_ILLPARM);
        return VSTATUS_ILLPARM;
    }
    
    // initialize the commom mad header fields of the SA MAD
    memset((void *)(&mad), 0, sizeof(mad)); 
    
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
    
    // send request to SA and wait for result
    if ((status = if3_mngr_send_mad(fp->fdr, &mad, sizeof(STL_NODE_RECORD), 
                                    buffer, 
                                    &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_VERBOSE_FMT(__func__, 
                           "Was not able to communicate with SA (SM/SA may have moved), status=%d ", status); 
        goto done;
    }

    if (madRc != 0) {
        status = VSTATUS_NOT_FOUND;
        IB_LOG_INFINI_INFOX("SA does not have Guid for port, SM probably still sweeping, status=", madRc);
        goto done;
    }
    
    // retrieve response data
    (void)BSWAPCOPY_STL_NODE_RECORD((STL_NODE_RECORD *)buffer, &nr);

    memcpy((void *)&fp->guid.guid[gididx], (void *)&nr.NodeInfo.PortGUID, sizeof(uint64_t));
    fp->vfi_guid = (uint8_t) gididx;

    IB_LOG_VERBOSE_FMT(__func__, 
           "Guid at idx %d is 0x%"CS64"x", 
           gididx, (fp->guid.guid[gididx]));

    if (fp->guid.guid[gididx] == 0ll) {
        status = VSTATUS_NOT_FOUND; 
        IB_LOG_ERROR_FMT(__func__, "Invalid guid exist at index gididx %d", gididx);
        goto done;
    }

    fp->guidIsValid = 1;

    done:
    IB_EXIT(__func__, status);
    return(status);
}

Status_t
vfi_MngrDelService(ManagerInfo_t * mi, VFI_SERVICE_RECORD * service, uint32_t mask)
{
    Status_t        status; 
    SA_MAD mad;
    uint32_t        madRc;
    int             count=0;
    uint8_t buffer[STL_SA_DATA_LEN];
    uint32_t bufferLength = sizeof(buffer);

    IB_ENTER(__func__, mi, service, mask, 0);
     
    // query the SA for the specified service record
    status = vfi_MngrQueryService(mi, service, mask, &count, NULL);
    if (status || count == 0) {
        IB_LOG_ERROR_FMT(__func__, "service record not found, rc:%u count:%d", status, count);
        status = VSTATUS_NOT_FOUND; 
        IB_EXIT(__func__,status);
        return status;
    }
    
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, mask); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_DELETE); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD); 
    
    // initialize SA MAD payload
    memcpy(mad.Data, service, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)mad.Data); 
    
    if ((status = if3_mngr_send_mad(mi->fdr, &mad, sizeof(IB_SERVICE_RECORD), 
                               buffer,
                              &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFO_FMT(__func__,
               "Was not able to communicate with SA (SM/SA may have moved), rc:%d", status);
        goto done;
    }

    if (madRc != 0) {
        IB_LOG_INFINI_INFOX("SA did not find record, resp:", madRc);
        status = VSTATUS_NOT_FOUND;
    }

    done:
    IB_LOG_INFO(__func__, status);

    return(status);
}

static uint32_t
one_sr_callback(CBTxRxData_t *data, void *context)
{
    uint32_t recordSize = sizeof(VFI_SERVICE_RECORD);
	int ready, remaining, offset;
    cb_ctx_t *c = (cb_ctx_t *)context;
    VFI_SERVICE_RECORD sr;

    //
    // We use this method to avoid allocating large static buffers for
    // receiving service records from the SA. We only want one to satisfy
    // the fact that a matching record exist at the SA.

	switch (data->dir) {
	case CB_RX_DIR:
		// this is the case we're after
		break;
	case CB_TX_DIR:
        c->status = VSTATUS_BAD;
        IB_LOG_ERROR0("unexpected tx callback dir=CB_TX_DIR");
		return VSTATUS_DROP;
	case CB_TO_DIR:
        c->status = VSTATUS_TIMEOUT;
        IB_LOG_INFO("timeout", c);
        return VSTATUS_OK;
	default:
        c->status = VSTATUS_BAD;
        IB_LOG_ERROR("invalid callback dir:", (int)data->dir);
		return VSTATUS_DROP;
	}

    if (!data->data) {
        // found zero length payload response packet, or error condition occurred
        // during reception of the response packet 
		c->status = VSTATUS_BAD;
        return c->status;
    }

	// first segment?  strip off the first SR and store the rest
	if (c->cnt) {
        memcpy(c->sr, data->data, sizeof(IB_SERVICE_RECORD));
        (void)BSWAP_IB_SERVICE_RECORD(c->sr); 

		c->status = VSTATUS_OK;
		c->cnt--;
		c->dlen = data->dlen - recordSize;
		memcpy(c->data, data->data + recordSize, c->dlen);
		if (c->cmp == NULL) {
			// no comparison function?  then we always keep the
			// first, so we're done
			return VSTATUS_DROP;
        } else {
			return VSTATUS_OK;
        }
	}

	// pull off remaining SRs and compare with the first
	// keep the "lowest" according to the comparison function

	ready = c->dlen;
	remaining = data->dlen;
	offset = 0;

	while (remaining + ready >= recordSize) {
		memcpy(c->data + ready, data->data + offset, recordSize - ready);

        memcpy(&sr, c->data, sizeof(IB_SERVICE_RECORD));
        (void)BSWAP_IB_SERVICE_RECORD(&sr); 

		if (c->cmp(c->sr, &sr) > 0)
			memcpy(c->sr, &sr, sizeof(sr));

		offset += recordSize - ready;
		remaining -= recordSize - ready;
		ready = 0;
	}

	if (remaining)
		memcpy(c->data, data->data + offset, remaining);

	return VSTATUS_OK;
}

Status_t
vfi_MngrQueryService(ManagerInfo_t * mi, VFI_SERVICE_RECORD * service,
                     uint32_t mask, int *count, VfiSvcRecCmp_t cmp)

{
    Status_t        status; 
    SA_MAD mad;
    uint32_t bufferLength = sizeof(VFI_SERVICE_RECORD), madRc; 
    VFI_SERVICE_RECORD insr;
    cb_ctx_t        cb;
    IB_ENTER(__func__, mi, service, mask, 0);

    *count = 0; 
    
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, mask); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GETTABLE); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD); 
    
    // initialize callback context
    cb.cnt = 1;
    cb.status = VSTATUS_BAD;
    cb.sr = &insr;
    cb.cmp = cmp;
	cb.dlen = 0; 
    
    // initialize SA MAD payload
    memcpy(mad.Data, service, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)mad.Data);
    
    // send request to SA and wait for result
    if ((status = if3_mngr_send_mad(mi->fdr,
                                &mad, sizeof(IB_SERVICE_RECORD), 
                               NULL, 
                                &bufferLength, &madRc, one_sr_callback,
                                &cb)) != VSTATUS_OK) {
        IB_LOG_INFINI_INFOX("query to SA for service record failed (SM/SA may have moved), rc:", status);
        goto done;
    }

    if (madRc != 0) {
        IB_LOG_INFO("No records found at SA", madRc);
        status = VSTATUS_OK;
        goto done;
    }

    if (cb.status != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Callback protocol error %d", cb.status);
        status = cb.status;
        goto done;
    }

    *count = 1;

    IB_LOG_INFO0(Log_StrDup((const char*)insr.ServiceName));
    IB_LOG_INFO("insr.serviceId is ", insr.RID.ServiceID);

    memcpy((void *)(service),(void *)(&insr),sizeof(IB_SERVICE_RECORD));

    done:

    IB_EXIT(__func__, status);
    return(status);
}

Status_t
vfi_MngrQuerySrvPath(ManagerInfo_t * mi,
                     VFI_SERVICE_RECORD * srp, int *count,
                     IB_PATH_RECORD * pathRecordp)
{ 
    Status_t status; 
    SA_MAD mad; 
    uint32_t madRc, plen; 
    uint8_t buffer[STL_IBA_SUBN_ADM_DATASIZE]; 
    uint32_t bufferLength = sizeof(buffer);
    
    IB_ENTER(__func__, mi, srp, count, pathRecordp); 
    
    memset((void *)(pathRecordp), 0, sizeof(*pathRecordp)); 
    
    // construct source GID.
    pathRecordp->SGID.Type.Global.SubnetPrefix = mi->gidPrefix; 
    pathRecordp->SGID.Type.Global.InterfaceID = mi->guid.guid[0]; 
    pathRecordp->DGID = srp->RID.ServiceGID; 
    
    memset((void *)(&mad), 0, sizeof(mad));
     
    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, SVC_SA_GET_PATH_CMASK); 
    
    // set the specified max number of path records to be returned from
    // the SA
    pathRecordp->NumbPath = *count; 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_GETTABLE); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_PATH_RECORD); 
    
    // initialize SA MAD payload
    (void)BSWAPCOPY_IB_PATH_RECORD(pathRecordp, (IB_PATH_RECORD *)mad.Data); 
    *count = 0; 
    
    status = if3_mngr_send_mad(mi->fdr, 
                          &mad, sizeof(IB_PATH_RECORD), 
                          buffer, &bufferLength, 
                          &madRc, NULL, NULL); 
    
    if (status != VSTATUS_OK) {
        IB_LOG_VERBOSERC("query to SA for server path failed (SM/SA may have moved), rc:", status); 
        IB_EXIT(__func__, status); 
        return (status);
    }
    
    if (madRc != 0) {
        IB_LOG_VERBOSEX("server path query to SA returned with status=", madRc); 
        status = VSTATUS_BAD;
    } else {
        // determine how many records were returned 
        if (bufferLength % (sizeof(IB_PATH_RECORD))) {
            IB_LOG_ERROR
               ("length does not align with record size ", 
                bufferLength); 
            status = VSTATUS_BAD; 
            IB_EXIT(__func__, status); 
            return (status);
        }
        
        plen = bufferLength / (sizeof(IB_PATH_RECORD)); 
        
        *count = plen; 
        
        
        {
            uint16_t        lid, 
               sl; 
            
            for (plen = 0; plen < (uint32_t)(*count); plen++) {
                (void)BSWAPCOPY_IB_PATH_RECORD((IB_PATH_RECORD *)buffer + plen * (sizeof(IB_PATH_RECORD)), 
                                               pathRecordp + plen); 
                
                lid = ((pathRecordp + plen)->DLID); 
                sl  = ((pathRecordp + plen)->u2.s.SL); 
                if (IB_LOG_IS_INTERESTED(VS_LOG_VERBOSE)) {
                    IB_LOG_VERBOSE(" PathREC : ", plen); 
                    IB_LOG_VERBOSEX("    DLid : ", lid); 
                    IB_LOG_VERBOSEX("    SLid : ", 
                                    ((pathRecordp + 
                                      plen)->SLID)); 
                    IB_LOG_VERBOSE("      SL : ", sl);
                }
                
            }
        }
    }
    
    IB_EXIT(__func__, status); 
    return (status);
}

Status_t
vfi_mngr_register(IBhandle_t fd, uint8_t mclass, int gididx,
                  VFI_SERVICE_RECORD * service, uint64_t mask, int option)
{

    Status_t        status,
    rc; 
    VFI_SERVICE_RECORD servRec; 
    VFI_SERVICE_RECORD *serviceRecordp = service;
    ManagerInfo_t  *mi; 
    SA_MAD mad; 
    uint8_t buffer[STL_SA_DATA_LEN]; 
    uint32_t madRc, bufferLength = sizeof(buffer);
    int             found,cnt=0;
    uint64_t        guid=0;
    IB_ENTER(__func__, gididx, mask, 0, 0);

    if (service == NULL ||
        gididx < 0 || gididx > VFI_MAX_PORT_GUID || mask == 0 || 
        (option != VFI_REGFORCE_FABRIC && option != VFI_REGTRY_FABRIC)) {
        rc = VSTATUS_ILLPARM;
        IB_LOG_ERROR("illegal param", rc);
        IB_EXIT(__func__, rc);
        return rc;
    }
    
    // find the MAI handle related information    
    status = if3_mngr_locate_minfo(fd, &mi);

    if (status) {
        IB_LOG_ERROR("minfo not found", status);
        IB_EXIT(__func__, status);
        return status;
    }
    
    // verify access to the SA by querying the the Port GUID of the SA
    status = vfi_GetPortGuid(mi, gididx);
    if (status != VSTATUS_OK) {
        IB_EXIT(__func__, status);
        return(status);
    }

    delete_next:

    // build service record
    BuildRecordGID(serviceRecordp, mi); 
    
    // preserve service record that will be sent when registering
    if (cnt == 0) memcpy(&servRec,serviceRecordp,sizeof(servRec)); 
    
    // query for the existence of the service record with the SA 
    status = vfi_MngrQueryService(mi, serviceRecordp, mask, &found, NULL);

    if (status != VSTATUS_OK) {
        IB_EXIT(__func__, status);
        return(status);
    }

	guid = serviceRecordp->RID.ServiceGID.Type.Global.InterfaceID;

    if (found != 0) {
        if (option == VFI_REGTRY_FABRIC) {
            IB_LOG_VERBOSELX("Registration exist already, GUID ", guid);
            status = VSTATUS_BUSY;
            IB_EXIT(__func__, status);
            return(status);
        }
        
        // force the registration by deleting the existing service record 
        status = vfi_MngrDelService(mi, serviceRecordp, mask);
        if (status != VSTATUS_OK) {
            if (status == VSTATUS_NOT_FOUND) {
                IB_LOG_VERBOSELX("service was not found, GUID ", guid);
            } else {
                status = VSTATUS_OK;
                IB_LOG_VERBOSELX("Could not delete service record, GUID ", guid);
                IB_EXIT(__func__, status);
                return(status);
            }
        } else {
            cnt++;
            IB_LOG_INFO0("Successfully deleted service");
            if (found > 1) {
                // restore service record to initial state
                memcpy(serviceRecordp,&servRec,sizeof(servRec));     
                goto delete_next;
            }
        }
    }
        
    //
    // now proceed with the registration    
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, mask); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_SET); 
    MAD_SET_VERSION_INFO(&mad, IB_BASE_VERSION, MCLASS_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_SERVICE_RECORD); 
    
    // initialize SA MAD payload
    memcpy(mad.Data, &servRec, sizeof(IB_SERVICE_RECORD));
    (void)BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD *)mad.Data); 
    
    // send service record to SA and wait for result
    if ((status = if3_mngr_send_mad(mi->fdr, &mad, sizeof(servRec), 
                               buffer,
                              &bufferLength, &madRc, NULL, NULL)) != VSTATUS_OK) {
        IB_LOG_VERBOSERC("could not talk to SA (SM/SA may have moved) rc:", status);
        goto done;
    }

    if (madRc != 0) {
        IB_LOG_VERBOSELX("service record registration with SA returned status=", madRc);
        status = VSTATUS_BAD;
    }

    done:
    memcpy(service,&servRec,sizeof(IB_SERVICE_RECORD));
    IB_EXIT(__func__, status);
    return(status);

}

Status_t
vfi_mngr_unregister(IBhandle_t fd, uint8_t mclass, int gididx,
                    VFI_SERVICE_RECORD * service, uint64_t mask)
{
    Status_t        status,
    rc;
    VFI_SERVICE_RECORD *serviceRecordp = service;
    ManagerInfo_t  *mi;

    IB_ENTER(__func__, gididx, mask, 0, 0);

    if (service == NULL ||
        gididx < 0 || gididx > VFI_MAX_PORT_GUID || mask == 0 ) {
        rc = VSTATUS_ILLPARM;
        IB_LOG_ERROR("illegal param", rc);
        IB_EXIT(__func__, rc);
        return rc;
    }

    // find the MAI handle related information    
    status = if3_mngr_locate_minfo(fd, &mi);
    if (status) {
        IB_LOG_ERROR("minfo not found", status);
        IB_EXIT(__func__, status);
        return status;
    }


    status = vfi_GetPortGuid(mi, gididx);
    if (status != VSTATUS_OK) {
        IB_EXIT(__func__, status);
        return(status);
    }

    // build service record
    BuildRecordGID(serviceRecordp, mi);
    status = vfi_MngrDelService(mi, serviceRecordp, mask);
    if (status != VSTATUS_OK) {
        IB_LOG_INFINI_INFORC("Could not delete service rc:", status);
    }

    IB_EXIT(__func__, status);
    return(status);

}

Status_t
vfi_mngr_find(IBhandle_t fd, uint8_t mclass, int slmc,
              VFI_SERVICE_RECORD * service, uint64_t mask,
              int *count, IB_PATH_RECORD * pbuff)
{
    return vfi_mngr_find_cmp(fd, mclass, slmc, service,
                             mask, count, pbuff, NULL);
}

Status_t
vfi_mngr_find_cmp(IBhandle_t fd, uint8_t mclass, int slmc,
              VFI_SERVICE_RECORD * service, uint64_t mask,
              int *count, IB_PATH_RECORD * pbuff, VfiSvcRecCmp_t cmp)
{

    Status_t        status,
    rc;
    VFI_SERVICE_RECORD *serviceRecordp = service;
    ManagerInfo_t  *mi;
    int             found;

    IB_ENTER(__func__, slmc, mask, 0, 0);

    if (service == NULL || mask == 0 ||
        pbuff == NULL || count == NULL ||
        *count <= 0 || *count > VFI_MAX_SERVICE_PATH ||
        slmc < 0 || slmc > VFI_MAX_LMC) {
        rc = VSTATUS_ILLPARM;
        IB_LOG_ERROR("illegal param", rc);
        IB_EXIT(__func__, rc);
        return rc;
    }

    // find the MAI handle related information    
    status = if3_mngr_locate_minfo(fd, &mi);

    if (status) {
        IB_LOG_ERROR("minfo not found", status);
        IB_EXIT(__func__, status);
        return status;
    }

    // query for the existence of a port guid info record with the SA 
    status = vfi_GetPortGuid(mi, VFI_DEFAULT_GUID);
    if (status != VSTATUS_OK) {
        IB_EXIT(__func__, status);
        return(status);
    }

    // query for the existence of a service record with the SA 
    status = vfi_MngrQueryService(mi, serviceRecordp, mask, &found, cmp);
    if (status != VSTATUS_OK) {
        IB_EXIT(__func__, status);
        return(status);
    }

    if (found == 0) {
        status = VSTATUS_NOT_FOUND;
        IB_LOG_INFO("Registration not found", status);
        IB_EXIT(__func__, status);
        return(status);
    }

    // query for the service path records
    status = vfi_MngrQuerySrvPath(mi, serviceRecordp, count, pbuff);

    IB_EXIT(__func__, status);
    return(status);
}
