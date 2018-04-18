/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************/
/*                                                                      */
/* FILE NAME                                                            */
/*    if3.h                                                             */
/*                                                                      */
/* DESCRIPTION                                                          */
/*    Library calls for interface 3                                     */
/*                                                                      */
/*                                                                      */
/* DEPENDENCIES                                                         */
/*    ib_types.h                                                        */
/*                                                                      */
/*                                                                      */
/************************************************************************/


#ifndef __IF3___
#define __IF3___

#include <mai_g.h>
#include <iba/ib_generalServices.h>
#include <iba/stl_rmpp.h>

/*
 * Defines for service ID, service name and classes used by IF3
 * and managers that it speaks to.
 */


#define FE_SERVICE_ID       (0x1100d03c34834444ull)
#define FE_SERVICE_NAME     "Intel OmniPath Fabric Executive"

#define SM_SERVICE_ID       (0x1100d03c34822222ull)
#define SM_SERVICE_ID_SEC   (0x1100d03c34822222ull)
#define SM_SERVICE_NAME     "Primary Intel OmniPath Subnet Manager"
#define SM_SERVICE_NAME_SEC "Secondary Intel OmniPath Subnet Manager"


#define  IF3_REGFORCE_PORT      (1)      /* force the registration */
#define  IF3_REGFORCE_FABRIC    (2)      /* force the registration */
#define  IF3_REGTRY_PORT        (3)      /* only try to register   */
#define  IF3_REGTRY_FABRIC      (4)      /* only try to register   */
#define  IF3_CONTROL_CMD (0x0000)       /* by IF3 transport       */

// Intra manager reliable communication commands set in method field
#define FE_MNGR_PROBE_CMD       (0x21)
#define FE_MNGR_CLOSE_CMD       (0x22)
#define FE_CMD_RESP             (0x25 | MAD_CM_REPLY)
#define FM_CMD_SHUTDOWN         (0x40)


#define	SM_KEY			0

#define SA_MAD_SET_HEADER(mad, smKey, mask) {\
	(mad)->SaHdr.SmKey = smKey; \
    (mad)->SaHdr.ComponentMask = mask; \
}


#define	SA_SRP_NAME_LEN		124

/*
 * Function used to do call backs while manager is communicating over IF3.
 */
typedef void  (*AsyncCallBack_t)(IBhandle_t fd , Mai_t *mad, void *context);

typedef struct {
    uint8_t         flag;	/* flag indicating whether the entry is in use */

    STL_LID         slid;	/* this port lid */
    STL_LID         dlid;	/* the managers port lid */
    STL_LID         saLid;	/* where the sa lives */
    uint8_t         lmc;	/* lmc of port */
    uint64_t        SubnetTO;	/* Subnet Timeout */

    uint16_t        mclass;	/* manager's class */

    uint16_t        dev;	/* our device */
    uint16_t        port;	/* our port */

    uint8_t         sl;		/* Service Level */

    uint16_t        pkey;	/* the pkey to use */
    uint32_t        qkey;	/* the qkey; */
    uint8_t         qp;		/* destination qp */

    IBhandle_t      fdr;	/* handle used for recv */
    IBhandle_t      fds;	/* handle used for command sends. */

    uint8_t         cb_flag;	/* If call back set this is true. */
    AsyncCallBack_t cb_func;	/* callback function for high priority data 
				 * received while speaking to FE 
				 */
    IBhandle_t      cb_fd;	/* callback handle to listen on currently while 
				 * doing recieve from FE
				 */
    void           *cb_ctx;	/* context to pass to call back function */

    uint8_t         guidIsValid;	/* true if the guid is valid */
    int8_t          vfi_guid;	/* Guid being used by vfi  - set to -1 when not initialized */
    GuidInfo_t      guid;	/* Guid for port */
    uint64_t        gidPrefix;	/* GID prefix */
    STL_CLASS_PORT_INFO cpi;
    int             cpi_valid;	/* true if classPortInfo is valid */
    uint8_t         isRegistered;	/* true if Manager service is
					 * registered */
    uint8_t         servName[SA_SRP_NAME_LEN];	/* name of service */
    uint64_t        servID;	/* service ID regiseter */
    uint32_t        retries;
    uint64_t        timeout;
    uint8_t         rmppCreateFilters;
    Pool_t          *rmppPool;
    uint32_t        rmppDataLength;
    uint32_t        rmppMaxCntxt;
    IBhandle_t      *rmppMngrfd;
    IBhandle_t      rmppGetfd; 
    IBhandle_t      rmppGetTablefd; 
} ManagerInfo_t;

Status_t if3_timeout_retry(IBhandle_t fd, uint64_t timeout, uint32_t retry);
/**/
/* Data structure to pass back information to call back function*/
/**/

typedef struct
{
  uint32_t        msize;        /*the total message size  being sent/expected*/
  uint32_t        offset;       /*the byte offset of this payload relative to message*/
  uint8_t         *data;        /*pointer to the  data being passed */
  uint32_t        dlen;         /*amount of data being passed in  callback*/
  uint8_t         dir;          /*direction: CB_TX_DIR,CB_RX_DIR*/
}CBTxRxData_t;

/**/
/* Return values from CBTxRxFunc_t supported */
/* VSTATUS_OK   - continue to get data from SA*/
/* VSTATUS_DROP - terminate data reception from SA*/
/**/
typedef uint32_t  (*CBTxRxFunc_t)(CBTxRxData_t *arg, void* context);


Status_t if3_register_fe(uint32_t dev, uint32_t port, uint8_t *servName, uint64_t servID, uint32_t option, IBhandle_t *pfd);
Status_t if3_deregister_fe(IBhandle_t fd);
Status_t if3_lid_mngr_cnx(uint32_t dev,uint32_t port, uint8_t mclass, STL_LID lid, IBhandle_t *mhdl);
Status_t if3_sid_mngr_cnx(uint32_t dev,uint32_t port, uint8_t *servName, uint64_t servID, uint8_t mclass, IBhandle_t *mhdl);

Status_t if3_local_mngr_cnx (uint32_t dev, uint32_t port, uint8_t mclass, IBhandle_t * mhdl);
Status_t if3_close_mngr_cnx(IBhandle_t mhdl);
Status_t if3_close_mngr_rmpp_cnx (IBhandle_t mhdl);
void STL_BasicMadInit (Mai_t * madp, uint8_t mclass, uint8_t method, uint16_t aid, uint32_t amod, STL_LID slid, STL_LID dlid, uint8_t sl);
Status_t if3_mad_init(IBhandle_t fd, Mai_t *madp, uint8_t    mclass, uint8_t method, uint16_t   aid, uint32_t   amod, STL_LID dlid);
Status_t if3_cntrl_cmd_send(IBhandle_t fd, uint8_t cmd);
Status_t if3_open(uint32_t dev, uint32_t port, uint8_t mclass, IBhandle_t * pfd);
Status_t if3_close(IBhandle_t mhdl);
Status_t if3_check_sa(IBhandle_t fd, int refresh, uint16_t *hasMoved);
Status_t if3_set_dlid(IBhandle_t fd, STL_LID dlid);

Status_t if3_mngr_locate_minfo(IBhandle_t rcv, ManagerInfo_t ** pt);

Status_t if3_mngr_open_cnx_fe(uint32_t dev, uint32_t port, uint16_t mclass, IBhandle_t  *fhdl);
Status_t if3_mngr_close_cnx_fe(IBhandle_t  fhdl, uint8_t complete);
Status_t if3_mngr_rcv_fe_data(IBhandle_t fhdl, Mai_t *fimad, uint8_t *inbuff, uint32_t *inlen);
Status_t if3_mngr_get_fe_cmd(Mai_t *mad, uint16_t *cmd);
Status_t if3_mngr_send_mad(IBhandle_t fd, SA_MAD *psa, uint32_t dataLength, uint8_t *buffer, uint32_t *bufferLength, uint32_t *madRc, CBTxRxFunc_t cb, void *context);
Status_t if3_mngr_send_passthru_mad (IBhandle_t fd, SA_MAD *psa, uint32_t dataLength, Mai_t *maip, uint8_t *buffer, uint32_t *bufferLength, uint32_t *madRc, CBTxRxFunc_t cb, void *context);
Status_t if3_dbsync_cmd_from_mngr(IBhandle_t fd, Mai_t *maip, uint8_t *buffer, uint32_t *bufferLength, uint32_t *madRc, CBTxRxFunc_t cb, void *context);
Status_t if3_dbsync_reply_to_mngr(IBhandle_t fhdl, Mai_t *fmad, uint8_t *outbuff, uint32_t outlen, uint32_t resp_status);
Status_t if3_dbsync_cmd_to_mngr(IBhandle_t mhdl, uint16_t cmd, uint32_t mod, uint8_t *outbuff, uint32_t outlen, uint8_t *inbuff, uint32_t *inlen, uint32_t *resp_status);
Status_t if3_dbsync_close(IBhandle_t mhdl);

Status_t MngrWaitHandle(IBhandle_t *ha, uint32_t count, uint64_t timeout, uint32_t *pfirst, Mai_t *maip);

Status_t if3_ssl_init(Pool_t *pool);
void * if3_ssl_srvr_open(const char *dir, const char *fmCertificate, const char *fmPrivateKey, const char *fmCaCertificate, uint32_t fmCertChainDepth, const char *fmDHParameters, uint32_t fmCaCrlEnabled, const char *fmCaCrl);
void * if3_ssl_accept(void *context, int clientfd); 
int if3_ssl_read(void *session, uint8_t *buffer, int bufferLength);
int if3_ssl_write(void *session, uint8_t *buffer, int bufferLength);
void if3_ssl_conn_close(void *context);
void if3_ssl_sess_close(void *session);

Status_t vfi_GetPortGuid(ManagerInfo_t *fp, uint32_t gididx);


#endif
