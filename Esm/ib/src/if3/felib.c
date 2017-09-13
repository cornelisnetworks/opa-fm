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
*    Code for Interface 3 communications Library                       *
*                                                                      *
*                                                                      *
*                                                                      *
* DEPENDENCIES                                                         *
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
#include "iba/ib_pa.h"
#include "iba/stl_pa.h"
#include "ib_sm.h"
#include "iba/ib_rmpp.h"
#include <if3.h>
#include "if3_def.h"
#include "ib_macros.h"
#include "cs_log.h"

#if defined( __LINUX__)
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#endif

#if defined( __VXWORKS__)
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#endif

#include "iba/ib_rmpp.h"
#include "rmpp_l.h"

#ifndef NULL
#define NULL (0)
#endif

#define INVALID_HANDLE  (-1)
#define DRAIN_MAI_STALE(fd) drain_stale(fd)

extern uint8_t if3_is_master(void);
extern void if3_set_rmpp_minfo(ManagerInfo_t *mi);

static ManagerInfo_t maninfo[MAX_MANAGER];

int gIF3_IS_INTERNAL_MODE = 0;	/* Used in multi-mad test to determine internal loopback mode */
int gWINSIZE = 0;		/* Used in multi-mad test to determine window to use */
int fe_rmpp_usrid = -1; 

static Status_t if3_dbsync_send_mad(IBhandle_t fd, SA_MAD *psa, uint32_t dataLength,
			      uint8_t *buffer, uint32_t *bufferLength,
			      uint32_t *madRc, CBTxRxFunc_t cb, void *context);
static Status_t if3_dbsync_send_multi_mad (IBhandle_t fd, SA_MAD *psa,
                  uint8_t *dataBuffer, uint32_t dataLength,
                  uint8_t *buffer, uint32_t *bufferLength, uint32_t *madRc,
                  CBTxRxFunc_t cb, void *context);

static int initialized;
static Lock_t lock;

static char* if3_rmpp_get_method_text(int method) 
{ 
   switch (method) {
   case RMPP_CMD_GET:
      return "GET"; 
      break; 
   case RMPP_CMD_SET:
      return "SET"; 
      break; 
   case RMPP_CMD_GET_RESP:
      return "GET_RESP"; 
      break; 
   case RMPP_CMD_GETTABLE:
      return "GETTABLE"; 
      break; 
   case RMPP_CMD_GETTABLE_RESP:
      return "GETTABLE_RESP"; 
      break; 
   case RMPP_CMD_DELETE:
       return "DELETE"; 
       break;
   case RMPP_CMD_DELETE_RESP:
       return "DELETE_RESP"; 
       break;
   case RMPP_CMD_FE_SEND:
       return "FE_CMD_SEND"; 
       break;
   case RMPP_CMD_FE_RESP:
       return "FE_CMD_RESP"; 
       break;
   default:
      return "UNKNOWN METHOD"; 
      break;
   }
}

static Status_t if3_pre_process_request(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{ 
   Status_t rc = VSTATUS_OK;
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!maip) {
      IB_LOG_ERROR_FMT(__func__, "MAD is NULL!"); 
      return (VSTATUS_BAD);
   } else if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return (VSTATUS_BAD);
   }
   
   // process specific command request.    
   switch (maip->base.method) {
   case RMPP_CMD_GET:
   case RMPP_CMD_GET_RESP:
   case RMPP_CMD_GETTABLE:
   case RMPP_CMD_GETTABLE_RESP:
   case RMPP_CMD_GETTRACETABLE:
   case RMPP_CMD_SET:
   case RMPP_CMD_DELETE:
   case RMPP_CMD_FE_SEND:
      // validate attribute  
      switch (maip->base.aid) {
      default:
         //rc = VSTATUS_ILLPARM; 
         break;
      }
      break; 
      
   default:
      IB_LOG_ERROR_FMT(__func__, "Error, Got UNKNOWN instead of GET"); 
      rc = VSTATUS_ILLPARM; 
      break;
   }
   
   IB_EXIT(__func__, rc); 
   return (rc);
}

static Status_t if3_pre_process_response(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{ 
   Status_t rc = VSTATUS_OK; 
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!maip) {
      IB_LOG_ERROR_FMT(__func__, "MAD is NULL!"); 
      return (VSTATUS_BAD);
   } else if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return (VSTATUS_BAD);
   }
   
   // process specific command request response.
   switch (maip->base.method) {
   case RMPP_CMD_GET_RESP:
   case RMPP_CMD_GETTABLE_RESP:
   case RMPP_CMD_DELETE_RESP:
   case RMPP_CMD_FE_RESP:
      // validate response from manager 
      switch (maip->base.aid) {         
      default:
         //rc = VSTATUS_ILLPARM; 
         break;
      }
      break; 
      
   default:
       if (maip->base.mclass != MAD_CV_VENDOR_DBSYNC)
		   rc = VSTATUS_ILLPARM; 
	   else {
		   switch (maip->base.method) {
		   case RMPP_CMD_GET:
		   case RMPP_CMD_GETTABLE:
			   // validate response from manager 
			   switch (maip->base.aid) {
			   default:
				   //rc = VSTATUS_ILLPARM; 
				   break;
			   }
			   break; 
		   default:
			   rc = VSTATUS_ILLPARM; 
			   break;
		   }
       }

       if (rc)
		   IB_LOG_ERROR_FMT(__func__, "Error, Got UNKNOWN instead of GETTABLE RESP"); 
      break;
   }
   
   IB_EXIT(__func__, rc); 
   return (rc);
}

#define LOCK_MINFO()do{                         \
    if(((vs_lock(&lock    ))!= VSTATUS_OK)){    \
       IB_FATAL_ERROR("LOCK acquire fail ");    \
     }}while(0)

#define UNLOCK_MINFO()do{                    \
    if((vs_unlock(&lock))!= VSTATUS_OK){     \
       IB_FATAL_ERROR("LOCK free failed ");  \
     }}while(0)

#define  IF3_INIT() if(initialized == 0) (void) if3_init()

#if defined( __LINUX__)
static Threadname_t if3_init_pid;
#endif

extern uint32_t if3DebugRmpp;  // Rmpp info messages control

extern uint16_t	mai_get_default_pkey(void);

static int
if3_init (void)
{
	int     i, rc;

/* we call init on startup for vxworks */
#ifndef __VXWORKS__
	Threadname_t my_pid;
	/*
	 *   Check to see if we have been initialized yet.  Since there can be a race
	 *   condition when creating a lock, we can't do that here.  What we do instead
	 *   is have all 'N' contending processes write to a static variable and then go
	 *   to sleep.  After they wake up, only one will have the variable.  That
	 *   thread goes on.
	 *
	 *   If the initialized variable is zero, then there can be a thread in the
	 *   process of initialization.  In this case, we just wait here until the other
	 *   thread finishes.
	 */

	(void) vs_thread_name (&my_pid);

	if (initialized == 0) {
		if (if3_init_pid == 0ull) {
			if3_init_pid = my_pid;
			(void) vs_thread_sleep (VTIMER_1S);
		}

		i = 0;

		if (if3_init_pid != my_pid) {
			while (initialized == 0 && i < 30) {
				(void) vs_thread_sleep (VTIMER_1S);
				i++;
			}

			if (initialized == 0) {
				IB_LOG_ERROR0 ("failed to initialized");
			}
			return VSTATUS_BAD;
		}
	} else {
		IB_LOG_ERROR0 ("Already initialized");
		return VSTATUS_BAD;
	}
#endif
	
    /*
	 * Make sure we belong here 
	 */

	if (initialized) {
		IB_LOG_ERROR0 ("Already initialized");
		return VSTATUS_BAD;
	}

	memset ((void *) (&lock), 0, sizeof (Lock_t));

	rc = vs_lock_init (&lock, VLOCK_FREE, VLOCK_THREAD);
	if (rc) {
		IB_LOG_ERRORRC("vs_lock_init rc:", rc);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	for (i = 0; i < MAX_MANAGER; i++) {
		maninfo[i].flag = 0;
	}

	initialized = 1;

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;

}


#ifdef __SIMULATOR__
static	int	if3Retries;
static	int if3WaitInterval;
void	setIf3Timers(int retries, int timeout) {
	if3Retries = retries;
	if3WaitInterval = timeout*1000;
}
#endif


static Status_t
if3_mngr_alloc_minfo (ManagerInfo_t ** pt)
{
	uint32_t i;
	uint32_t rc = VSTATUS_OK;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, pt, 0, 0, 0);

	IF3_INIT ();
	/*
	 *  Take lock 
	 */
	
    LOCK_MINFO ();

	for (i = 0; i < MAX_MANAGER; i++) {

		if (maninfo[i].flag == 0) {
			mi = &maninfo[i];
			*pt = mi;
			memset (mi, 0, sizeof (ManagerInfo_t));
			mi->sl = FE_MANAGER_SL;
			mi->pkey = FE_MANAGER_PKEY;
			mi->qkey = FE_MANAGER_QKEY;
			mi->flag = 1;
			mi->qp = MAI_GSI_QP;
#ifdef __SIMULATOR__
			mi->retries = if3Retries;
			mi->timeout = if3WaitInterval;
#else
			mi->retries = MAX_RETRY;
			mi->timeout = RC_MAD_TIMEOUT;
#endif
            mi->vfi_guid = -1;
            mi->rmppGetfd = -1; 
            mi->rmppGetTablefd = -1;
			goto done;
		}
	}

	rc = VSTATUS_NORESOURCE;
	IB_LOG_ERROR0 ("out of resources");

    done:

	/*
	 * Release lock 
	 */
	UNLOCK_MINFO ();

	IB_EXIT(__func__, rc);
	return rc;
}

void
drain_stale (IBhandle_t fd)
{
	uint32_t i, rc;
	Mai_t mad;

	IB_ENTER(__func__, fd, 0, 0, 0);
	i = 0;
	while ((rc = mai_recv (fd, &mad, MAI_RECV_NOWAIT)) == VSTATUS_OK) {
		i++;
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFOX ("method:", mad.base.method);
            IB_LOG_INFINI_INFOX ("           : mclass ", mad.base.mclass);
            IB_LOG_INFINI_INFOX ("           : aid    ", mad.base.aid);
            IB_LOG_INFINI_INFOX ("           : amod   ", mad.base.amod);
            IB_LOG_INFINI_INFOLX ("           : tid    ", mad.base.tid);
        }
	}

	IB_EXIT(__func__, i);
	return;
}

Status_t
if3_timeout_retry (IBhandle_t fd, uint64_t timeout, uint32_t retry)
{
	uint32_t rc = VSTATUS_OK;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, fd, timeout, retry, 0);

	rc = if3_mngr_locate_minfo (fd, &mi);
	if (rc == VSTATUS_OK) {
		mi->retries = retry;
		mi->timeout = timeout;
		if (mi->timeout <= 0)
			mi->timeout = RC_MAD_TIMEOUT;
	}

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_mngr_locate_minfo (IBhandle_t rcv, ManagerInfo_t ** pt)
{
	uint32_t i;
	uint32_t rc = VSTATUS_OK;

	IB_ENTER(__func__, rcv, pt, 0, 0);

	for (i = 0; i < MAX_MANAGER; i++) {

		if (maninfo[i].flag && (maninfo[i].fdr == rcv || maninfo[i].fds == rcv)) {
			*pt = &maninfo[i];
			IB_EXIT(__func__, rc);
			return rc;
		}
	}

	rc = VSTATUS_ILLPARM;
	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_addr_swizzle (Mai_t * mad)
{

	Lid_t lid;
	/*
	 * swap the source and destination address
	 */
	lid = mad->addrInfo.slid;
	mad->addrInfo.slid = mad->addrInfo.dlid;
	mad->addrInfo.dlid = lid;

	/*
	 * TBD: PAW do GRH stuff
	 */
	return VSTATUS_OK;

}

Status_t
if3_mad_reply (IBhandle_t fd, Mai_t * mad)
{

	uint32_t rc;

	IB_ENTER(__func__, fd, mad, 0, 0);

    // set response bit
    mad->base.method |= MAD_CM_REPLY;
    // swap SLID and DLID
	(void) if3_addr_swizzle (mad);

	IB_LOG_VERBOSEX (" dlid ", mad->addrInfo.dlid);
	IB_LOG_VERBOSEX (" slid ", mad->addrInfo.slid);

	rc = mai_send (fd, mad);

	IB_EXIT(__func__, rc);
	return rc;
}

static Status_t
if3_mngr_get_pd (ManagerInfo_t * pt)
{

	uint32_t rc = VSTATUS_OK;
	STL_PORT_INFO *pi, po;

	IB_ENTER(__func__, pt, 0, 0, 0);

	/*
	 * Read the port info to determine data.
	 */
	rc = mai_get_stl_portinfo (pt->fdr, &po, pt->port);

	if (rc != VSTATUS_OK) {
		IB_LOG_WARNRC("can't read port info rc:", rc);
		goto bail;
	}


	pi = &po;

	if (pi->MasterSMLID == RESERVED_LID || pi->MasterSMLID == PERMISSIVE_LID) {
		IB_LOG_VERBOSEX ("Port not yet configured  smLid:", pi->MasterSMLID);

		rc = VSTATUS_NXIO;
		goto bail;
	}

	pt->slid = pi->LID;
	pt->saLid = pi->MasterSMLID;
	pt->sl = pi->s2.MasterSMSL;
	pt->lmc = (1 << (pi->s1.LMC)) - 1;

	pt->gidPrefix = pi->SubnetPrefix;
	pt->SubnetTO = (pi->Subnet.Timeout & (0x1F));

	if (pt->SubnetTO == 0) {
		pt->SubnetTO = DEFAULT_SUBNET_TIMEOUT;
	}
	pt->SubnetTO = ((1 << pt->SubnetTO) * 4ull);

	if (IB_LOG_IS_INTERESTED(VS_LOG_INFO)) {
	IB_LOG_INFOX("port   lid: ", (pt->slid));
	IB_LOG_INFOX("        port salid: ", (pt->saLid));
	IB_LOG_INFO ("        port  saSL: ", (pt->sl));
	IB_LOG_INFOLX("         gidPrefix: ", (pt->gidPrefix));
	IB_LOG_INFO ("         subnetTimeout: ", (pi->Subnet.Timeout));
	IB_LOG_INFO ("         SubnetTO: ", (pt->SubnetTO));
	}


      bail:
	IB_EXIT(__func__, rc);
	return rc;
}

static void
if3_mngr_mad_init (Mai_t * madp, ManagerInfo_t * fp)
{
	//SAMadh_t *sp;

	IB_ENTER(__func__, madp, fp, 0, 0);

	memset (madp, 0, sizeof (Mai_t));

	madp->type = MAI_TYPE_EXTERNAL;
	madp->base.mclass = fp->mclass;
	madp->base.bversion = MAD_BVERSION;
    if (madp->base.mclass == MAD_CV_SUBN_ADM) {
        madp->base.cversion = SA_MAD_CVERSION;
        IB_LOG_INFO("seting CVERSION for SA queries to 1.1 compliant seting ",
                           madp->base.cversion); 
    } else
        madp->base.cversion = MAD_CVERSION;

	/*
	 * Setup the outgoing MAD.
	 *   
	 * The QP is the determining factor here. If we opened QP0 then we must 
	 * talk using SMPs. Othewise we use a regular MAD.
	 * we are using A GSI QP.
	 */

	AddrInfo_Init (madp, fp->slid, fp->dlid, fp->sl, mai_get_default_pkey(), MAI_GSI_QP, fp->qp, GSI_WELLKNOWN_QKEY);

	madp->active = MAI_ACT_BASE | MAI_ACT_DATA |
		MAI_ACT_TYPE | MAI_ACT_ADDRINFO;

	if (fp->dlid == RESERVED_LID || fp->dlid == PERMISSIVE_LID ||
	    fp->slid == RESERVED_LID || fp->slid == PERMISSIVE_LID
	    /*
	     * || fp->slid == fp->dlid
	     */ ) {

		IB_LOG_VERBOSEX (" dlid ", fp->dlid);
		IB_LOG_VERBOSEX (" slid ", fp->slid);

		madp->type = MAI_TYPE_EXTERNAL;
		IB_LOG_INFO ("mad initialized to type INTERNAL", madp->type);
	}

	if (gIF3_IS_INTERNAL_MODE && fp->dlid == fp->slid){
		IB_LOG_ERROR ("NO Support mad initialized to type INTERNAL type:", madp->type);
		madp->type = MAI_TYPE_INTERNAL;

	}

	//sp = (SAMadh_t *) madp->data;
	//sp->window = Endian16 (gWINSIZE);

	IB_EXIT(__func__, 0);
}

static void
if3_mngr_stl_mad_init (Mai_t * madp, ManagerInfo_t * fp)
{
	//SAMadh_t *sp;

	IB_ENTER(__func__, madp, fp, 0, 0);

	memset (madp, 0, sizeof (Mai_t));

	madp->type = MAI_TYPE_EXTERNAL;
	madp->base.mclass = fp->mclass;
	madp->base.bversion = STL_BASE_VERSION;
	// For STL2 this may need to be class specific (see if3_mngr_mad_init)
	madp->base.cversion = STL_BASE_VERSION;

	/*
	 * Setup the outgoing MAD.
	 *   
	 * The QP is the determining factor here. If we opened QP0 then we must 
	 * talk using SMPs. Othewise we use a regular MAD.
	 * we are using A GSI QP.
	 */

	AddrInfo_Init (madp, fp->slid, fp->dlid, fp->sl, mai_get_default_pkey(), MAI_GSI_QP, fp->qp, GSI_WELLKNOWN_QKEY);

	madp->active = MAI_ACT_BASE | MAI_ACT_DATA |
		MAI_ACT_TYPE | MAI_ACT_ADDRINFO;

	if (fp->dlid == RESERVED_LID || fp->dlid == PERMISSIVE_LID ||
	    fp->slid == RESERVED_LID || fp->slid == PERMISSIVE_LID
	    /*
	     * || fp->slid == fp->dlid
	     */ ) {

		IB_LOG_VERBOSEX (" dlid ", fp->dlid);
		IB_LOG_VERBOSEX (" slid ", fp->slid);

		madp->type = MAI_TYPE_EXTERNAL;
		IB_LOG_INFO ("mad initialized to type INTERNAL", madp->type);
	}

	if (gIF3_IS_INTERNAL_MODE && fp->dlid == fp->slid){
		IB_LOG_ERROR ("NO Support mad initialized to type INTERNAL type:", madp->type);
		madp->type = MAI_TYPE_INTERNAL;

	}

	//sp = (SAMadh_t *) madp->data;
	//sp->window = Endian16 (gWINSIZE);

	IB_EXIT(__func__, 0);
}	// End of if3_mngr_stl_mad_init()

static void
if3_mngr_sa_mad_init (Mai_t * madp, ManagerInfo_t * fp, uint8_t mclass, uint8_t bversion, uint8_t cversion, uint64_t tid)
{
	IB_ENTER(__func__, madp, fp, 0, 0);

	memset (madp, 0, sizeof (Mai_t));

    // initialize MAD common fields
	madp->type = MAI_TYPE_EXTERNAL;
	madp->base.mclass = mclass;
	madp->base.bversion = bversion;
    madp->base.cversion = cversion;
	madp->base.tid = tid;

	// initialize address info
	AddrInfo_Init (madp, fp->slid, fp->dlid, fp->sl, mai_get_default_pkey(), MAI_GSI_QP, fp->qp, GSI_WELLKNOWN_QKEY);

    // initialize mask field to indicate to the MAI layer which data to process
	madp->active = MAI_ACT_BASE | MAI_ACT_DATA |
		MAI_ACT_TYPE | MAI_ACT_ADDRINFO;

	if (fp->dlid == RESERVED_LID || fp->dlid == PERMISSIVE_LID ||
	    fp->slid == RESERVED_LID || fp->slid == PERMISSIVE_LID ) {
		IB_LOG_VERBOSEX (" dlid ", fp->dlid);
		IB_LOG_VERBOSEX (" slid ", fp->slid);

		madp->type = MAI_TYPE_EXTERNAL;
		IB_LOG_INFO ("mad initialized to type INTERNAL", madp->type);
	}

	if (gIF3_IS_INTERNAL_MODE && fp->dlid == fp->slid){
		IB_LOG_ERROR ("No support mad initialized to type INTERNAL type:", madp->type);
		madp->type = MAI_TYPE_INTERNAL;

	}

	IB_EXIT(__func__, 0);
}

void
STL_BasicMadInit (Mai_t * madp,
			  uint8_t mclass, uint8_t method, uint16_t aid, uint32_t amod,
			  uint16_t slid, uint16_t dlid, uint8_t sl)
{
	ManagerInfo_t mi = { 0 };

	// supply only what is needed by if3_mngr_stl_mad_init above
	mi.slid = slid;
	mi.dlid = dlid;
	mi.sl = sl;
	mi.qp = MAI_GSI_QP;
	mi.mclass = mclass;

	if3_mngr_stl_mad_init(madp, &mi);

	madp->type = MAI_TYPE_EXTERNAL;
	madp->base.method = method;
	madp->base.aid = aid;
	madp->base.amod = amod;
}

Status_t
if3_mad_init (IBhandle_t fd, Mai_t * pmad,
	    uint8_t mclass, uint8_t method, uint16_t aid, uint32_t amod, uint16_t dlid)
{
	ManagerInfo_t *mi;
	uint32_t rc;

	IB_ENTER(__func__, fd, pmad, 0, 0);

	rc = if3_mngr_locate_minfo (fd, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	(void) if3_mngr_mad_init (pmad, mi);

	pmad->addrInfo.dlid = dlid;
	pmad->base.method = method;
	pmad->base.mclass = mclass;
	pmad->base.aid = aid;
	pmad->base.amod = amod;

	if (0 && pmad->addrInfo.dlid == pmad->addrInfo.slid) {
		pmad->type = MAI_TYPE_INTERNAL;
		IB_LOG_INFO ("mad set to type INTERNAL", pmad->type);
	} else {
		pmad->type = MAI_TYPE_EXTERNAL;
		IB_LOG_INFO ("mad set to type MAD", pmad->type);
	}

	IB_EXIT(__func__, rc);
	return rc;
}


Status_t
if3_register_fe(uint32_t dev, uint32_t port, 
                uint8_t *servName, uint64_t servID, uint32_t option, IBhandle_t *pfd)
{
    uint32_t rc; 
    ManagerInfo_t *mi; 
    
    IB_ENTER(__func__, dev, port, servID, option); 
    
    rc = if3_mngr_alloc_minfo(&mi); 
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    mi->port = port; 
    mi->dev = dev; 
    mi->mclass = MAD_CV_VENDOR_FE; 
    
    //    
    // open mai and query and other general incoming stuff
    rc = mai_open(MAI_GSI_QP, dev, port, &mi->fdr);     
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't open mai rc:", rc); 
        mi->flag = 0; 
        goto bail;
    }
    
    rc = mai_open(MAI_GSI_QP, dev, port, &mi->fds);     
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't open mai rc:", rc); 
        goto bail2;
    }
    
    // get port lid and sm lid
    rc = if3_mngr_get_pd(mi);    
    if (rc != VSTATUS_OK) {
        goto bail3;
    }
    
    // set value of the handle to be returned 
    *pfd = mi->fdr;
         
    // register the FE with the SA
    rc = if3_mngr_register_sa(mi->fdr, servName, servID, option);     
    if (rc) {
        *pfd = INVALID_HANDLE; 
        IB_LOG_INFINI_INFORC("can't register FE with the SA at this time rc:", rc); 
        goto bail3;
    }
    
    goto bail; 
    
bail3:
    (void)mai_close(mi->fds); 
    
bail2:
    (void)mai_close(mi->fdr); 
    mi->flag = 0; 
    
bail:    
    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_deregister_fe (IBhandle_t fd)
{
	uint32_t rc;
	ManagerInfo_t *mi;
	IB_ENTER(__func__, fd, 0, 0, 0);

	/*
	 * find the handle stuff
	 */

	rc = if3_mngr_locate_minfo (fd, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	if (mi->mclass != MAD_CV_VENDOR_FE) {
		rc = VSTATUS_ILLPARM;
		IB_LOG_ERROR ("class on handle is not FE_MCLASS class:", mi->mclass);
		goto done;
	}

	/*
	 * Now remove the registration
	 */

	rc = if3_mngr_deregister_sa (fd);

	if (rc != VSTATUS_OK && rc != VSTATUS_NOT_FOUND) { // not found means it was already removed

		/*
		 * There was a error speaking with SA. We will proceed to close
		 * the handle anyway.
		 */

		IB_LOG_INFINI_INFORC("deregistration with SA did not go through rc:", rc);
	}

	rc = VSTATUS_OK;

    /*
     * The FE is in the process of shutting down, so do a completed termination 
     * of the RMPP connection. 
     */
    (void)rmpp_mngr_close_cnx(mi, TRUE);

	(void) mai_close (mi->fds);
	(void) mai_close (mi->fdr);
	mi->flag = 0;

      done:

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_open (uint32_t dev, uint32_t port, uint8_t mclass, IBhandle_t * pfd)
{
	uint32_t rc;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, dev, port, mclass, 0);

	rc = if3_mngr_alloc_minfo (&mi);
	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	mi->port = port;
	mi->dev = dev;
	mi->mclass = mclass;

	/*
	 * Now open mai and querry and other general incoming stuff
	 */
	rc = mai_open (MAI_GSI_QP, dev, port, &mi->fdr);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't open mai rc:", rc);
        mi->flag = 0;
		goto bail;
	}

	rc = mai_open (MAI_GSI_QP, dev, port, &mi->fds);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't open mai rc:", rc);
		goto bail2;
	}

	/*
	 * Now get port lid and sm lid
	 */
	rc = if3_mngr_get_pd (mi);

	if (rc != VSTATUS_OK) {
		goto bail3;
	}

	/*
	 * return the received handle 
	 */

	*pfd = mi->fdr;
	goto bail;

      bail3:
	(void) mai_close (mi->fds);

      bail2:
	(void) mai_close (mi->fdr);
	mi->flag = 0;

      bail:

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_close (IBhandle_t mhdl)
{

	uint32_t rc;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, mhdl, 0, 0, 0);

	rc = if3_mngr_locate_minfo (mhdl, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	rc = VSTATUS_OK;

    /*
     * STL Manager has encountered a problem with its peer and is temporarily 
     * closing the connection in order to establish a new connection; so do 
     * a partial closure of the RMPP connection. 
     */
    (void)rmpp_mngr_close_cnx(mi, FALSE);

	(void) mai_close (mi->fds);
	(void) mai_close (mi->fdr);
	mi->flag = 0;

	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_dbsync_close (IBhandle_t mhdl)
{

	uint32_t rc;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, mhdl, 0, 0, 0);

	rc = if3_mngr_locate_minfo (mhdl, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	rc = VSTATUS_OK;

    /*
     * DBSync thread is terminating it connection to the remote SM; so do a
     * a complete closure of the RMPP connection. 
     */
    (void)rmpp_mngr_close_cnx(mi, TRUE);

	(void) mai_close (mi->fds);
	(void) mai_close (mi->fdr);
	mi->flag = 0;

	IB_EXIT(__func__, rc);
	return rc;

}

Status_t if3_set_dlid (IBhandle_t fd, Lid_t dlid)
{
	Status_t rc=VSTATUS_OK;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, fd, dlid, 0, 0);

	if ((rc = if3_mngr_locate_minfo (fd, &mi)) == VSTATUS_OK) {
        mi->dlid = dlid;
    }
    IB_EXIT(__func__, rc);
    return rc;
}

Status_t
if3_check_sa (IBhandle_t fd, int refresh, uint16_t *hasMoved)
{
	uint32_t rc=VSTATUS_OK;
	ManagerInfo_t *mi;
	STL_PORT_INFO pi;

	IB_ENTER(__func__, fd, refresh, hasMoved, 0);

    *hasMoved = 0;
	/*
	 * Read the port info to determine if SA has moved.
	 */
	if ((rc = if3_mngr_locate_minfo (fd, &mi)) != VSTATUS_OK) {
		IB_LOG_WARNRC("can't find management info structure rc:", rc);
		goto bail;
#ifdef __SIMULATOR__
	/* once we have an SA lid we're good; s20 sim can't handle all this traffic */
	} else if (mi->saLid > 0 && mi->saLid < 0xffff) {
		goto bail;
#endif
	} else if ((rc = mai_get_stl_portinfo (mi->fdr, &pi, mi->port)) != VSTATUS_OK) {
		IB_LOG_WARNRC("can't read local port info rc:", rc);
		goto bail;
	} else if (pi.PortStates.s.PortState != IB_PORT_ACTIVE) {
		IB_LOG_WARN ("Port not yet active state:", pi.PortStates.s.PortState);
		rc = VSTATUS_NXIO;
		goto bail;
	} else if (pi.MasterSMLID == RESERVED_LID || pi.MasterSMLID == PERMISSIVE_LID) {
		IB_LOG_WARNX ("Port not yet configured smLid:", pi.MasterSMLID);
		rc = VSTATUS_NXIO;
		goto bail;
	} else if (mi->saLid != pi.MasterSMLID || mi->slid != pi.LID || mi->sl != pi.s2.MasterSMSL) {
        /*
         * SM lid change or our own lid changing indicates that there is a new SM
         * there is a hole if a fabric merge results in a new SM with same 
         * lid as the old and our lid not changing. We will never know about it.
         * SM/SA has moved (or our lid has changed), register with new one 
         */
		if (mi->saLid != pi.MasterSMLID) {
            IB_LOG_WARNX ("SM/SA has moved to LID:", pi.MasterSMLID);
        } else if (mi->slid != pi.LID) {
            IB_LOG_WARNX ("This port's LID has changed to ", pi.LID);
		} else {
            IB_LOG_WARN ("The SM/SA SL has changed to ", pi.s2.MasterSMSL);
        }
        *hasMoved = 1;
        mi->slid = pi.LID;
        mi->saLid = pi.MasterSMLID;
        mi->sl = pi.s2.MasterSMSL;
        mi->lmc = (1 << (pi.s1.LMC)) - 1;
        mi->gidPrefix = pi.SubnetPrefix;
        mi->isRegistered = 0;
        mi->SubnetTO = (pi.Subnet.Timeout & (0x1F));
        if (mi->SubnetTO == 0) {
            mi->SubnetTO = DEFAULT_SUBNET_TIMEOUT;
        }
        mi->SubnetTO = ((1 << mi->SubnetTO) * 4ull);
    }
#if defined(IB_STACK_OPENIB)
	if ((rc = ib_refresh_devport()) != VSTATUS_OK) {
		IB_LOG_WARNRC("can't read local port pkeys rc:", rc);
		goto bail;
	}
#endif

    bail:
	IB_EXIT(__func__, rc);
	return (rc);
}  // end if3_check_sa

Status_t
if3_cntrl_cmd_send (IBhandle_t fd, uint8_t cmd)
{
	Mai_t mad;
	uint32_t rc, i;
	ManagerInfo_t *mi;
	IBhandle_t fh = 0;
	IBhandle_t fh1 = 0;
	Filter_t f, f1, *fp;

	IB_ENTER(__func__, fd, cmd, 0, 0);

	rc = if3_mngr_locate_minfo (fd, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
    return rc;
	}

	DRAIN_MAI_STALE (mi->fds);

	(void) if3_mngr_stl_mad_init (&mad, mi);
    mad.base.method = cmd;
	mai_alloc_tid(fd, mi->mclass, &mad.base.tid);

	IB_LOG_VERBOSE ("sending method ", mad.base.method);
	IB_LOG_VERBOSE ("sending cmd ", mad.base.aid);

	/*
	 * create the filter to get the response
	 */
    fp = &f;
	memset ((void *) (&f), 0, sizeof (f));

	fp->type = MAI_TYPE_ANY;
	fp->value.mclass = mi->mclass;
	fp->value.method = FE_CMD_RESP;
	fp->mask.mclass = MAI_FMASK_ALL;
	fp->mask.method = MAI_FMASK_ALL;
	fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK);
	fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter;

	rc = mai_filter_hcreate (mi->fds, fp, VFILTER_SHARE, &fh);
	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create filter mai rc:", rc);
		goto bail;
	}

	/*
	 * create a filter to catch errors specifically looking for a timeout contition
	 */
	fp = &f1;
	memset ((void *) (&f1), 0, sizeof (f1));

	fp->type = MAI_TYPE_ERROR;
	fp->value.mclass = mi->mclass;
	fp->value.method = FE_CMD_RESP;
	fp->mask.mclass = MAI_FMASK_ALL;
	fp->mask.method = MAI_FMASK_ALL;
	fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK);
	fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter;

	rc = mai_filter_hcreate (mi->fds, fp, VFILTER_SHARE, &fh1);
	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create timeout filter mai rc:", rc);
		goto bail;
	}

	/*
	 * We can do the transfer in one mad.. nothing tricky needed
	 */
	for (i = 0; i < mi->retries; i++) {

		rc = mai_send (mi->fdr, &mad);

		if (rc != VSTATUS_OK) {
			IB_LOG_WARNRC("mai_send failed rc:", rc);
			goto bail;
		}

#ifdef IB_STACK_OPENIB
		rc = if3_recv (mi, &mad, mi->timeout + VTIMER_1S);
#else
		rc = if3_recv (mi, &mad, mi->timeout);
#endif

		if (rc == VSTATUS_OK) {
			if (mad.type == MAI_TYPE_ERROR) {
				continue;
			}
		} else if (rc == VSTATUS_TIMEOUT) {
			continue;
		}

		if (rc != VSTATUS_OK && rc != VSTATUS_MAI_INTERNAL) {
			IB_LOG_WARNRC("mai_recev failed rc:", rc);
			goto bail;
		}

		rc = VSTATUS_OK;
		break;
	}

      bail:
	if (fh) 
		(void) mai_filter_hdelete (mi->fds, fh);
	if (fh1) 
		(void) mai_filter_hdelete (mi->fds, fh1);

	IB_EXIT(__func__, rc);
	return rc;

}

static Status_t
open_mngr_cnx(uint32_t dev, uint32_t port, 
              uint8_t *servName, uint64_t servID, 
              uint8_t mclass, uint16_t lid, uint32_t mode, IBhandle_t *mhdl)
{
    
    uint32_t rc = VSTATUS_OK; 
    ManagerInfo_t *mi; 
    
    IB_ENTER(__func__, dev, port, mclass, servID); 
    
    rc = if3_mngr_alloc_minfo(&mi); 
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    mi->port = port; 
    mi->dev = dev; 
    mi->mclass = mclass; 
    
    //
    // open mai and querry and other general incoming stuff
    rc = mai_open(MAI_GSI_QP, dev, port, &mi->fdr);     
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't open mai rc:", rc); 
        mi->flag = 0; 
        goto bail;
    }
    
    rc = mai_open(MAI_GSI_QP, dev, port, &mi->fds); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("can't open mai rc:", rc); 
        goto bail2;
    }
    
    //
    // get port lid and sm lid
    rc = if3_mngr_get_pd(mi); 
    if (rc != VSTATUS_OK) {
        goto bail3;
    }
    
    if (mode == 1) {
        // query SA for location of manager
        IB_SERVICE_RECORD sr; 
        uint32_t count; 
        uint16_t sl; 
        
        // query the SA for the location of the service
        *mhdl = mi->fdr; 
        
        rc = if3_mngr_query_service(mi->fdr, servName, servID, IF3_FABRIC_SERVICE, &sr, &count);         
        if (rc != VSTATUS_OK) {
            IB_LOG_INFORC("SA query failed rc:", rc); 
            goto bail3;
        }
        
        if (count != 1) {
            IB_LOG_VERBOSE("multiple services found count:", count);
        }
        
        //
        // get the service LID
        rc = if3_mngr_query_srv_path(mi->fdr, &sr, &mi->dlid, &sl);         
        if (rc != VSTATUS_OK) {
            IB_LOG_VERBOSERC("SA path query failed rc:", rc); 
            goto bail3;
        }
        
        if (mi->dlid == PERMISSIVE_LID || mi->dlid == RESERVED_LID) {
            IB_LOG_VERBOSEX(" Got  service lid", mi->dlid); 
            rc = VSTATUS_BAD; 
            goto bail3;
        } else {
            IB_LOG_INFOX(" Got  service lid", mi->dlid);
        }
        
    } else if (mode == 0) {
        
        // we have the lid of the manager location
        mi->dlid = (Lid_t)lid;
    } else {
        // manager is local
        mi->dlid = mi->slid;
    }
    
    // send a message to probe the manager 
    rc = if3_cntrl_cmd_send(mi->fdr, FE_MNGR_PROBE_CMD);     
    if (rc == VSTATUS_OK) {
        IB_LOG_INFORC("manager found .. returned rc:", rc);
    } else {
        rc = VSTATUS_NXIO; 
        IB_LOG_VERBOSEX(" Failure Sending CTRL message to manager at service lid", mi->dlid); 
        goto bail3;
    }
    
    *mhdl = mi->fdr; 
    
    goto bail; 
    
    //
    // we had an error .. fall through
    //
    
bail3:
    *mhdl = INVALID_HANDLE; 
    (void)mai_close(mi->fds); 
    
bail2:
    (void)mai_close(mi->fdr); 
    
    mi->flag = 0; 
bail:
    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_lid_mngr_cnx(uint32_t dev, uint32_t port, uint8_t mclass, uint16_t lid, IBhandle_t * mhdl)
{

	uint32_t rc;

	IB_ENTER(__func__, dev, port, mclass, lid);

	rc = open_mngr_cnx (dev, port, NULL, 0, mclass, lid, 0, mhdl);

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_sid_mngr_cnx(uint32_t dev, uint32_t port, 
                 uint8_t *servName, uint64_t servID, uint8_t mclass, IBhandle_t *mhdl)
{
    uint32_t rc; 
    
    IB_ENTER(__func__, dev, port, mclass, servID); 
    
    rc = open_mngr_cnx(dev, port, servName, servID, mclass, 0, 1, mhdl); 
    
    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_local_mngr_cnx (uint32_t dev, uint32_t port, uint8_t mclass, IBhandle_t * mhdl)
{
	uint32_t rc;

	IB_ENTER(__func__, dev, port, mclass, mhdl);

	rc = open_mngr_cnx (dev, port, NULL, 0, mclass, 0, 2, mhdl);

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_close_mngr_cnx (IBhandle_t mhdl)
{
	uint32_t rc;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, mhdl, 0, 0, 0);

	rc = if3_mngr_locate_minfo (mhdl, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	rc = if3_cntrl_cmd_send (mi->fdr, FE_MNGR_CLOSE_CMD);

	if (rc == VSTATUS_OK) {
		IB_LOG_INFO0 ("released minfo structure, manager connection close OK");
	} else {
		IB_LOG_INFORC("released minfo structure, manager close cmd sent rc:", rc);
	}

	rc = VSTATUS_OK;

    /*
     * STL Manager is in the process of shutting down and terminating its connection 
     * to the FE, so do a completed closure of the RMPP connection. 
     */
    (void)rmpp_mngr_close_cnx(mi, TRUE);

	(void) mai_close (mi->fds);
	(void) mai_close (mi->fdr);
	mi->flag = 0;

	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_close_mngr_rmpp_cnx (IBhandle_t mhdl)
{
	uint32_t rc;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, mhdl, 0, 0, 0);

	rc = if3_mngr_locate_minfo (mhdl, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

    /*
     * STL Manager is in the process of shutting down and terminating its connection 
     * to it's peer, so do a completed closure of the RMPP connection. 
     */
    (void)rmpp_mngr_close_cnx(mi, TRUE);

	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_dbsync_cmd_to_mngr (IBhandle_t mhdl, uint16_t cmd, uint32_t mod,
		uint8_t * outbuff, uint32_t outlen, uint8_t * inbuff,
		uint32_t * inlen, uint32_t * resp_status)
{ 
    uint32_t rc, datalen = 0, madRc = 0; 
    SA_MAD mad; 
    ManagerInfo_t *mi; 
    
    IB_ENTER(__func__, mhdl, cmd, mod, outbuff); 
    
    if ((!outbuff && outlen) || (resp_status == NULL) || (inbuff == NULL || inlen == NULL)) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR0("Illegal param"); 
        goto done;
    }
    
    // find the handle stuff
    if ((rc = if3_mngr_locate_minfo(mhdl, &mi))) {
        IB_EXIT(__func__, rc); 
        goto done;
    }
    
    // initialize the commom mad header fields of the SA MAD
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    MAD_SET_METHOD_TYPE(&mad, RMPP_CMD_GETTABLE); 
    MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, 
                         mi->mclass, STL_SA_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, cmd); 
    MAD_SET_ATTRIB_MOD(&mad, mod); 
    
    if (outbuff && outlen > sizeof(mad.Data)) {
        // send large DB Sync command to the remote SM and wait for result
        if ((rc = if3_dbsync_send_multi_mad(mhdl, &mad, outbuff, outlen, inbuff, inlen, &madRc, NULL, NULL)) != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("could not talk with remote SM (SM may have moved) rc:", rc); 
            goto done;
        }
    } else {
        // initialize SA MAD payload
        if (outbuff && outlen > 0) {
            datalen = MIN(outlen, sizeof(mad.Data)); 
            memcpy(mad.Data, outbuff, datalen);
        }

        // send DB Sync command to the remote SM and wait for result
        if ((rc = if3_dbsync_send_mad(mhdl, &mad, datalen, inbuff, inlen, &madRc, NULL, NULL)) != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("could not talk with remote SM (SM may have moved) rc:", rc); 
            goto done;
        }
    }
    
    if (madRc != 0) {
        rc = VSTATUS_NOT_FOUND; 
        IB_LOG_INFINI_INFO(" Invalid status retrieving dbsync request:", madRc); 
        goto done;
    }  
    
done:
    IB_EXIT(__func__, rc); 
    return rc;
    
}

Status_t
if3_mngr_open_cnx_fe (uint32_t dev, uint32_t port, uint16_t mclass, IBhandle_t * fhdl)
{

	uint32_t rc = VSTATUS_OK;
	ManagerInfo_t *mi;
	IBhandle_t fh;

	IB_ENTER(__func__, dev, port, mclass, fhdl);

	rc = if3_mngr_alloc_minfo (&mi);
	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	mi->port = port;
	mi->dev = dev;
	mi->mclass = mclass;

	/*
	 * Now open mai and querry and other general incoming stuff
	 */

	rc = mai_open (MAI_GSI_QP, dev, port, &mi->fdr);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't open mai rc:", rc);
        mi->flag = 0;
		goto bail;
	}

	rc = mai_open (MAI_GSI_QP, dev, port, &mi->fds);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't open mai rc:", rc);
		goto bail2;
	}

	/*
	 * Now create the filters that would allow us to recieve FE commands
	 * on the handle
	 */

	rc = mai_filter_method (mi->fdr, VFILTER_SHARE, MAI_TYPE_ANY,
				&fh, mclass, FE_MNGR_PROBE_CMD);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create filter mai rc:", rc);
		goto bail3;
	}

	rc = mai_filter_method (mi->fdr, VFILTER_SHARE, MAI_TYPE_ANY,
				&fh, mclass, FE_MNGR_CLOSE_CMD);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create filter mai rc:", rc);
		goto bail3;
	}

	rc = mai_filter_method (mi->fdr, VFILTER_SHARE, MAI_TYPE_ANY, &fh, mclass, FM_CMD_SHUTDOWN);

	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't create filter mai for FM_CMD_SHUTDOWN rc:", rc);
		goto bail3;
	}

	/*
	 * Now get port lid and sm lid
	 */

	rc = if3_mngr_get_pd (mi);

	if (rc != VSTATUS_OK) {
		goto bail3;
	}

	/*
	 * PAW: should we probe for FE or just wait for it contact us?
	 */

	/*
	 * We don't know the location of the FE so just set this to permissive
	 */

	mi->dlid = PERMISSIVE_LID;

	IB_LOG_INFO0 ("manager ready for FE");

	/*
	 * return the received handle 
	 */

	*fhdl = mi->fdr;
	goto bail;

      bail3:
	(void) mai_close (mi->fds);

      bail2:
	(void) mai_close (mi->fdr);

	mi->flag = 0;
      bail:
	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_mngr_close_cnx_fe (IBhandle_t fhdl, uint8_t complete)
{
	uint32_t rc = VSTATUS_OK;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, fhdl, 0, 0, 0);
	rc = if3_mngr_locate_minfo (fhdl, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

    /* Close the RMPP connection to the FE */
    /*
     * STL Manager has encountered a problem with the FE and is temporarily 
     * closing the connection in order to establish a new connection; or the 
     * manager may be terminating the connection; so do a partial/complete 
     * closure of the RMPP connection. 
     */
    (void)rmpp_mngr_close_cnx(mi, complete);

	(void) mai_close (mi->fds);
	(void) mai_close (mi->fdr);
	mi->flag = 0;

	IB_EXIT(__func__, rc);
	return rc;

}

static Status_t
if3_mngr_do_recv_fe_data (IBhandle_t fhdl, Mai_t * fimad, uint8_t * inbuff, uint32_t * inlen)
{

	uint32_t rc;
	ManagerInfo_t *mi;
	Mai_t m, *fmad;

	IB_ENTER(__func__, fhdl, fimad, inbuff, inlen);

	rc = if3_mngr_locate_minfo (fhdl, &mi);
	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	/*
	 * Copy the incoming mad ..so that we do not disturb the one passed in.
	 */
	fmad = &m;
	memcpy ((void *) fmad, (void *) fimad, sizeof (m));

	/*
	 * The first thing is to determine if this is a multimad or single mad 
	 * recv.
	 */
	switch (fmad->base.method) {
    case RMPP_CMD_GET:
    case RMPP_CMD_SET:
    case RMPP_CMD_GETTABLE:
		{
			STL_SA_MAD_HEADER sp;
			uint32_t len;

			/* figure out the data length */
			BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER*)(fmad->data),&sp);
            len = sp.length;

			/* unless first or last packet, ignore this check */
			if ((sp.u.tf.rmppFlags & RMPP_FLAGS_FIRST) || (sp.u.tf.rmppFlags & RMPP_FLAGS_LAST)) {
               if (len - SA_HEADER_SIZE > STL_SA_DATA_LEN) { 
					IB_LOG_ERROR("RMPP MAD protocol error - invalid length:", len);
					rc = VSTATUS_BAD;
				}
			}

            /*
             * skip the population of the input buffer, all processing of the
             * MAD packet is done later.
             */

			if (if3DebugRmpp)
                IB_LOG_INFINI_INFO("RMPP mad data received from client, class=\n", mi->mclass);

            rc = VSTATUS_OK;
		}
		break;
	case FE_MNGR_PROBE_CMD:
		{

			/*
			 * we have a probe command.. * now send the ACK
			 */
			IB_LOG_INFOX("probe command received from lid ", fmad->addrInfo.slid);
			if (inlen) {
				*inlen = 0;
			}
			fmad->base.method = FE_CMD_RESP;
			rc = if3_mad_reply (mi->fds, fmad);
			if (rc != VSTATUS_OK) {
				IB_LOG_ERRORRC("reply failed rc:", rc);
			}

			if (mi->dlid != fmad->addrInfo.slid) {
                // the dlid will contantly change if multiple FEs are running
                // this field is really worthless for PM/BM as currently implemented
				mi->dlid = fmad->addrInfo.slid;
			}
		}
		break;
	case FE_MNGR_CLOSE_CMD:
		{

			/*
			 * We have notice that the guy is going away
			 */
            IB_LOG_INFOX("FE MNGR CLOSE location call from LID: ", fmad->addrInfo.slid);
			if (inlen) {
				*inlen = 0;
			}

			fmad->base.method = FE_CMD_RESP;

			rc = if3_mad_reply (mi->fds, fmad);

			if (rc != VSTATUS_OK) {
				IB_LOG_INFORC("reply failed rc:", rc);

			}

			if (mi->dlid == fmad->addrInfo.slid) {
				/*
				 * set to this to ensusre all outstanding sync messages fail.
				 */
				mi->dlid = PERMISSIVE_LID;
            } 
        }
		break;
	case FM_CMD_SHUTDOWN:
		{
			/*
			 * shutdown request from management entity
			 */
			IB_LOG_INFOX("Manager shutdown command received from lid ", fmad->addrInfo.slid);
			if (inlen) *inlen = 0;
			fmad->base.method = FE_CMD_RESP;
			rc = if3_mad_reply (mi->fds, fmad);
			if (rc != VSTATUS_OK) {
				IB_LOG_INFORC("reply to shutdown command failed rc:", rc);

			}
        }
		break;
	default:
		{
			IB_LOG_ERROR ("Unknown method:", fmad->base.method);
			rc = VSTATUS_BAD;
		}
	}

	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_mngr_rcv_fe_data (IBhandle_t fhdl, Mai_t * fimad, uint8_t * inbuff, uint32_t * inlen)
{
	uint32_t rc;

	IB_ENTER(__func__, fhdl, fimad, inbuff, inlen);

	if ((inbuff == NULL || inlen == 0) || (fimad == NULL)) {
		rc = VSTATUS_ILLPARM;
		IB_LOG_ERROR0 ("Illegal param");
		goto done;
	}

	rc = if3_mngr_do_recv_fe_data (fhdl, fimad, inbuff, inlen);

      done:
	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_dbsync_reply_to_mngr(IBhandle_t fhdl, Mai_t * fmad,
	       uint8_t * outbuff, uint32_t outlen, uint32_t resp_status)
{

	uint32_t rc = VSTATUS_OK;
	ManagerInfo_t *mi; 
    rmpp_cntxt_t *rmpp_cntxt = NULL; 
    uint32_t attribOffset; 
    uint8_t processMad = 0; 
    int fd_rmpp_usrid = -1; 

	IB_ENTER(__func__, fmad, outbuff, 0, 0); 
    
    // find the handle stuff
    rc = if3_mngr_locate_minfo (fhdl, &mi);
	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}
    
    // setup RMPP related fields 
    (void)if3_set_rmpp_minfo(mi); 

    // set return status
    fmad->base.status = (uint16_t)resp_status;

    if (mi->rmppMngrfd == NULL) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR_FMT(__func__, "Handle not set"); 
        goto bail;
    }
    
    // get RMPP connection for the STL manager
    if ((fd_rmpp_usrid = rmpp_is_cnx_open(mi->rmppMngrfd)) == -1 || rmpp_is_cnx_partial_open(fd_rmpp_usrid)) {
        rc = VSTATUS_NOHANDLE; 
        IB_LOG_ERROR_FMT(__func__, "Connection not open"); 
        goto bail;
    }
    
    // get RMPP context to be associated with the MAD request 
    if ((rmpp_cntxt = rmpp_cntxt_get(fd_rmpp_usrid, fmad, &processMad)) == NULL || !processMad) {
        rc = VSTATUS_NOMEM; 
        IB_LOG_ERROR_FMT(__func__, "Unable to create context"); 
        goto bail;
    }
    
    // before forwarding the MAD request, perform any required pre-processing of
    // the RMPP context and MAD request    
    if (!(rc = rmpp_pre_process_request(fd_rmpp_usrid, fmad, rmpp_cntxt))) {
        attribOffset = outlen + Calculate_Padding(outlen); 
        // setup attribute offset for RMPP transfer
        rmpp_cntxt->attribLen = attribOffset; 
        
        // allocate RMPP request context structure
        rmpp_cntxt_data(fd_rmpp_usrid, rmpp_cntxt, outbuff, attribOffset); 
        
        // send DB Synch reply to the remote STL SM that issued the request
        if (VSTATUS_OK != (rc = rmpp_send_reply(fmad, rmpp_cntxt))) {
            IB_LOG_ERROR_FMT(__func__, "Failed to foward RMPP request to manager: rc %d", rc); 
        }
        // deallocate the context
        rmpp_cntxt_release(rmpp_cntxt);
    }
    
bail:
   // deallocate the context
   if (rmpp_cntxt) {
       rmpp_cntxt_full_release(rmpp_cntxt);
   }

    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_mngr_send_mad(IBhandle_t fd, SA_MAD *psa, uint32_t dataLength, uint8_t *buffer, 
                  uint32_t *bufferLength, uint32_t *madRc, CBTxRxFunc_t cb, void *context)
{ 
    
    uint32_t rc = VSTATUS_OK; 
    ManagerInfo_t *mi; 
    uint64_t tid; 
    IBhandle_t fh = 0; 
    Filter_t f, *fp; 
    Mai_t mad, imad; 
    uint64_t timeout; 
    rmpp_cntxt_t *fe_cntxt = NULL; 
    uint32_t attribOffset; 
    uint8_t processMad = 0; 
    int fd_rmpp_usrid = -1; 
    SA_MAD_HDR saHdr; 
    
    IB_ENTER(__func__, psa, dataLength, 0, 0); 
    
    // buffer and callback pointer parameters are optional    
    if (!psa || !bufferLength || !madRc) {
        IB_LOG_ERROR_FMT(__func__, "Missing required input parameter"); 
        return VSTATUS_ILLPARM; 
    }
    
	*madRc = MAD_STATUS_SA_NO_RECORDS;

    // find the handle stuff
    rc = if3_mngr_locate_minfo(fd, &mi); 
    
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // setup RMPP related fields 
    (void)if3_set_rmpp_minfo(mi); 
    
    // 
    // calculating Response Timeout Period
#ifdef __SIMULATOR__
    timeout = mi->timeout; 
#else
    if (mi->cpi.respTimeValue == 0) {
        timeout = RC_MAD_TIMEOUT; 
        if (if3DebugRmpp) 
            IB_LOG_INFINI_INFO("Using default RC_MAD_TIMEOUT = ", timeout);
    } else {
        timeout = ((1 << mi->cpi.respTimeValue) * 4ull) + mi->SubnetTO; 
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, "used RespTimeValue=%d, SubnetTO=%d to calculate timeout=%"CS64u, 
                                   (int)mi->cpi.respTimeValue, (int)mi->SubnetTO, timeout);
        }
    }
    mi->timeout = timeout; 
#endif
    
    // drain stale data from response handle
    DRAIN_MAI_STALE(mi->fds); 
    
    //
    // now setup the filter to receive the SA responses * Set up a filter
    // to get the response to our first * message.
    rc = mai_alloc_tid(mi->fds, mi->mclass, &tid); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("unable to allocate tid rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    //
    // the manager could live on my machine. I need to make sure that *
    // the filter looks at the method too. Otherwise as the *FI loops
    // the mad in the port we will recieve our own message * and beleive
    // it is the response from the manager
    fp = &f; 
    memset((void *)&f, 0, sizeof(f)); 
    
    fp->type = MAI_TYPE_ANY; 
    fp->value.mclass = psa->common.MgmtClass; 
    if (psa->common.mr.s.Method <= SA_CM_SET) {
        fp->value.method = SA_CM_GET_RESP;
    } else if (psa->common.mr.s.Method == SA_CM_GETTRACETABLE) {
        fp->value.method = SA_CM_GETTABLE_RESP;
    } else {
        // everybody else, OR in 0x80 to inbound; i.e., 14->94
        fp->value.method = (psa->common.mr.s.Method | MAD_CM_REPLY);
    }
    fp->value.tid = tid; 
    
    fp->mask.mclass = MAI_FMASK_ALL; 
    fp->mask.method = MAI_FMASK_ALL; 
    fp->mask.tid = MAI_FMASK_ALL; 
    
    fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK); 
    fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter; 
    
    // create the filter
    rc = mai_filter_hcreate(mi->fds, fp, VFILTER_SHARE, &fh); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("Unable to create filter rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // initialize common fields of the MAI MAD for a SA request
    (void)if3_mngr_sa_mad_init(&mad, mi, psa->common.MgmtClass, psa->common.BaseVersion, psa->common.ClassVersion, tid); 
    mad.addrInfo.dlid = mi->saLid; 
    if (0 && mad.addrInfo.dlid == mad.addrInfo.slid) {
        mad.type = MAI_TYPE_INTERNAL;
    } else {
        mad.type = MAI_TYPE_EXTERNAL;
    }
    
    // initialize SA request specific fields of the MAI MAD 
    mad.base.method = psa->common.mr.s.Method; 
    mad.base.status = psa->common.u.NS.Status.AsReg16; 
    mad.base.aid = psa->common.AttributeID; 
    mad.base.amod = psa->common.AttributeModifier; 
    
    saHdr.RmppHdr = psa->RmppHdr; 
    saHdr.SaHdr = psa->SaHdr; 
    
    (void)BSWAPCOPY_SA_HDR(&saHdr, (SA_MAD_HDR *)mad.data); 
    
    if (mi->rmppMngrfd == NULL) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR_FMT(__func__, "Handle not set"); 
        goto bail;
    }
    
    // initialize and open RMPP connection for the STL manager
    if ((fd_rmpp_usrid = rmpp_is_cnx_open(mi->rmppMngrfd)) == -1 || rmpp_is_cnx_partial_open(fd_rmpp_usrid)) {
        fd_rmpp_usrid = rmpp_mngr_open_cnx(
           mi->rmppMngrfd,   
           MAI_GSI_QP, 
           mi->dev, 
           mi->port, 
           mi->rmppPool, 
           mi->rmppDataLength, 
           mi->rmppMaxCntxt, 
           (mi->rmppCreateFilters) ? &mi->rmppGetfd : NULL, 
           (mi->rmppCreateFilters) ? &mi->rmppGetTablefd : NULL, 
           mi->mclass, 
           if3_rmpp_get_method_text, 
           cs_getAidName, 
           if3_pre_process_request, 
           if3_pre_process_response, 
           if3_is_master, 
           LOCAL_MOD_ID, 
           NULL
           ); 
        
        if (fd_rmpp_usrid == -1) {
            rc = VSTATUS_NOHANDLE; 
            IB_LOG_ERROR_FMT(__func__, "Unable to open connection"); 
            goto bail;
        }
    }
    
    // get RMPP context to be associated with the MAD request 
    if ((fe_cntxt = rmpp_cntxt_get(fd_rmpp_usrid, &mad, &processMad)) == NULL || !processMad) {
        rc = VSTATUS_NOMEM; 
        IB_LOG_ERROR_FMT(__func__, "Unable to create context"); 
        goto bail;
    }
    
    // before forwarding the MAD request, perform any required pre-processing of
    // the RMPP context and MAD request    
    if (!(rc = rmpp_pre_process_request(fd_rmpp_usrid, &mad, fe_cntxt))) {
        attribOffset = dataLength + Calculate_Padding(dataLength); 
        // setup attribute offset for RMPP transfer
        fe_cntxt->attribLen = attribOffset; 
        
        // allocate RMPP request context structure
        rmpp_cntxt_data(fd_rmpp_usrid, fe_cntxt, psa->Data/*outdata*/, attribOffset); 
        
        // forward FEC or STL manager single MAD request to STL manager
        if (VSTATUS_OK != (rc = rmpp_send_request(&mad, fe_cntxt))) {
            IB_LOG_ERROR_FMT(__func__, "Failed to foward RMPP request to manager: rc %d", rc);
        } else {
            // release context for single MAD request
            rmpp_cntxt_full_release(fe_cntxt);
            fe_cntxt = NULL;

            // wait for response from manager
            rc = rmpp_receive_response(fd_rmpp_usrid, mi, &imad, buffer, bufferLength, cb, context); 
            if (if3DebugRmpp) {
                if (rc == VSTATUS_OK) {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received OK, len %d", *bufferLength);
                } else {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received FAILED, rc %d", rc);
                }
            }
            
            // get MAD status from the header
            if (rc == VSTATUS_OK) {
                *madRc = imad.base.status; 
            }
            
            // set response status to no_records since rmpp returns zero length record with status OK 
            // in those cases
            if (!*madRc && !*bufferLength) {
                *madRc = MAD_STATUS_SA_NO_RECORDS;
            }
        }
    }
    
bail:
    if (fh) {
        if (mai_filter_hdelete(mi->fds, fh)) {
            IB_LOG_WARN0("filter delete failed");
        }
    }
    // deallocate the context
    if (fe_cntxt) {
        rmpp_cntxt_full_release(fe_cntxt); 
    }
    
    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_mngr_send_passthru_mad (IBhandle_t fd, SA_MAD *psa, uint32_t dataLength,
                            Mai_t *maip, uint8_t *buffer,
	   uint32_t *bufferLength, uint32_t *madRc, CBTxRxFunc_t cb, void *context)
{ 
    
    uint32_t rc = VSTATUS_OK; 
    ManagerInfo_t *mi; 
    uint64_t tid; 
    IBhandle_t fh = 0; 
    Filter_t f, *fp; 
    Mai_t mad, imad; 
    uint64_t timeout; 
    rmpp_cntxt_t *fe_cntxt = NULL; 
    uint32_t attribOffset; 
    uint8_t processMad = 0; 
    int fd_rmpp_usrid = -1; 
    SA_MAD_HDR saHdr;
    
    IB_ENTER(__func__, psa, dataLength, 0, 0); 
    
    // buffer and callback pointer parameters are optional    
    if (!psa || !maip || !bufferLength || !madRc) {
        IB_LOG_ERROR_FMT(__func__, "Missing required input parameter"); 
        return VSTATUS_ILLPARM; 
    }
    
    // find the handle stuff
    rc = if3_mngr_locate_minfo(fd, &mi); 
    
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // setup RMPP related fields 
    (void)if3_set_rmpp_minfo(mi); 
    
    // 
    // calculating Response Timeout Period
#ifdef __SIMULATOR__
    timeout = mi->timeout; 
#else
    if (mi->cpi.respTimeValue == 0) {
        timeout = RC_MAD_TIMEOUT; 
        if (if3DebugRmpp) 
            IB_LOG_INFINI_INFO("Using default RC_MAD_TIMEOUT = ", timeout);
    } else {
        timeout = ((1 << mi->cpi.respTimeValue) * 4ull) + mi->SubnetTO; 
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, "used RespTimeValue=%d, SubnetTO=%d to calculate timeout=%"CS64u, 
                                   (int)mi->cpi.respTimeValue, (int)mi->SubnetTO, timeout);
        }
    }
    mi->timeout = timeout; 
#endif
    
    // drain stale data from response handle
    DRAIN_MAI_STALE(mi->fds); 
    
    //
    // now setup the filter to receive the SA responses * Set up a filter
    // to get the response to our first * message.
    rc = mai_alloc_tid(mi->fds, mi->mclass, &tid); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("unable to allocate tid rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    //
    // the manager could live on my machine. I need to make sure that *
    // the filter looks at the method too. Otherwise as the *FI loops
    // the mad in the port we will recieve our own message * and beleive
    // it is the response from the manager
    fp = &f; 
    memset((void *)&f, 0, sizeof(f)); 
    
    fp->type = MAI_TYPE_ANY; 
    fp->value.mclass = psa->common.MgmtClass; 
    if (psa->common.mr.s.Method <= SA_CM_SET) {
        fp->value.method = SA_CM_GET_RESP;
    } else if (psa->common.mr.s.Method == SA_CM_GETTRACETABLE) {
        fp->value.method = SA_CM_GETTABLE_RESP;
    } else {
        // everybody else, OR in 0x80 to inbound; i.e., 14->94
        fp->value.method = (psa->common.mr.s.Method | MAD_CM_REPLY);
    }
    fp->value.tid = tid; 
    fp->mask.mclass = MAI_FMASK_ALL; 
    fp->mask.method = MAI_FMASK_ALL; 
    fp->mask.tid = MAI_FMASK_ALL; 
    fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK); 
	fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter;
    
    // create the filter
    rc = mai_filter_hcreate(mi->fds, fp, VFILTER_SHARE, &fh); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("Unable to create filter rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // initialize common fields of the MAI MAD for a SA request
    (void)if3_mngr_sa_mad_init(&mad, mi, psa->common.MgmtClass, psa->common.BaseVersion, psa->common.ClassVersion, tid); 
    mad.addrInfo.dlid = mi->saLid; 
    if (0 && mad.addrInfo.dlid == mad.addrInfo.slid) {
        mad.type = MAI_TYPE_INTERNAL;
    } else {
        mad.type = MAI_TYPE_EXTERNAL;
    }
    
    // initialize SA request specific fields of the MAI MAD 
    mad.base.method = psa->common.mr.s.Method; 
    mad.base.status = psa->common.u.NS.Status.AsReg16; 
    mad.base.aid = psa->common.AttributeID; 
    mad.base.amod = psa->common.AttributeModifier; 
    
    saHdr.RmppHdr = psa->RmppHdr; 
    saHdr.SaHdr = psa->SaHdr; 
    
	memcpy((SA_MAD_HDR *)mad.data, &saHdr, sizeof(SA_MAD_HDR)); 
    if (mi->rmppMngrfd == NULL) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR_FMT(__func__, "Handle not set"); 
        goto bail;
    }
    
    // initialize and open RMPP connection for the STL manager
    if ((fd_rmpp_usrid = rmpp_is_cnx_open(mi->rmppMngrfd)) == -1 || rmpp_is_cnx_partial_open(fd_rmpp_usrid)) {
        fd_rmpp_usrid = rmpp_mngr_open_cnx(
           mi->rmppMngrfd,   
           MAI_GSI_QP, 
           mi->dev, 
           mi->port, 
           mi->rmppPool, 
           mi->rmppDataLength, 
           mi->rmppMaxCntxt, 
           &mi->rmppGetfd, 
           &mi->rmppGetTablefd,
           mi->mclass, 
           if3_rmpp_get_method_text, 
           cs_getAidName, 
           if3_pre_process_request, 
           if3_pre_process_response, 
           if3_is_master, 
           LOCAL_MOD_ID, 
           NULL
           ); 
        
        if (fd_rmpp_usrid == -1) {
            rc = VSTATUS_NOHANDLE; 
            IB_LOG_ERROR_FMT(__func__, "Unable to open connection"); 
            goto bail;
        }
    }
    
    // get RMPP context to be associated with the MAD request 
    if ((fe_cntxt = rmpp_cntxt_get(fd_rmpp_usrid, &mad, &processMad)) == NULL || !processMad) {
        rc = VSTATUS_NOMEM; 
        IB_LOG_ERROR_FMT(__func__, "Unable to create context"); 
        goto bail;
    }
    
    // before forwarding the MAD request, perform any required pre-processing of
    // the RMPP context and MAD request    
    if (!(rc = rmpp_pre_process_request(fd_rmpp_usrid, &mad, fe_cntxt))) {
        attribOffset = dataLength + Calculate_Padding(dataLength); 
        // setup attribute offset for RMPP transfer
        fe_cntxt->attribLen = attribOffset; 
        
        // allocate RMPP request context structure
        rmpp_cntxt_data(fd_rmpp_usrid, fe_cntxt, psa->Data, attribOffset); 
        
        // forward FEC or STL manager single MAD request to STL manager
        if (VSTATUS_OK != (rc = rmpp_send_request(&mad, fe_cntxt))) {
            IB_LOG_ERROR_FMT(__func__, "Failed to foward RMPP request to manager: rc %d", rc); 
        } else {
            // release context for single MAD request
            rmpp_cntxt_full_release(fe_cntxt);

            // wait for response from manager
            rc = rmpp_receive_response(fd_rmpp_usrid, mi, &imad, buffer, bufferLength, cb, context); 
            if (if3DebugRmpp) {
                if (rc == VSTATUS_OK) {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received OK, len %d", *bufferLength);
                } else {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received FAILED, rc %d", rc);
                }
            }
            
            // get MAD
            memcpy(maip, &imad, sizeof(Mai_t));
                        
            // get MAD status from the header
            *madRc = imad.base.status; 
            
            // set response status to no_records since rmpp returns zero length record with status OK 
            // in those cases
            if (!*madRc && !*bufferLength) {
                *madRc = MAD_STATUS_SA_NO_RECORDS;
            }
        }
    }
    
bail:

    if (fh) {
        if (mai_filter_hdelete(mi->fds, fh)) {
            IB_LOG_WARN0("filter delete failed");
        }
    }
    // deallocate the context
    if (fe_cntxt) {
       rmpp_cntxt_full_release(fe_cntxt);
    }
    
    IB_EXIT(__func__, rc); 
    return rc;
}

static Status_t
if3_dbsync_send_mad(IBhandle_t fd, SA_MAD *psa, uint32_t dataLength, uint8_t *buffer, uint32_t *bufferLength, uint32_t *madRc, 
                    CBTxRxFunc_t cb, void *context)
{ 
    
    uint32_t rc = VSTATUS_OK; 
    ManagerInfo_t *mi; 
    uint64_t tid; 
    IBhandle_t fh = 0; 
    Filter_t f, *fp; 
    Mai_t mad, imad; 
    uint64_t timeout; 
    rmpp_cntxt_t *dbsync_cntxt = NULL; 
    uint32_t attribOffset; 
    uint8_t processMad = 0; 
    int fd_rmpp_usrid = -1; 
    SA_MAD_HDR saHdr; 
    STL_SA_MAD samad;
    
    IB_ENTER(__func__, psa, dataLength, 0, 0); 
    
    // buffer and callback pointer parameters are optional    
    if (!psa || !bufferLength || !madRc) {
        IB_LOG_ERROR_FMT(__func__, "Missing required input parameter"); 
        return VSTATUS_ILLPARM; 
    }

    // find the handle stuff
    rc = if3_mngr_locate_minfo(fd, &mi); 
    
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // setup RMPP related fields 
    (void)if3_set_rmpp_minfo(mi); 
    
    // 
    // calculating Response Timeout Period
#ifdef __SIMULATOR__
    timeout = mi->timeout; 
#else
    if (mi->cpi.respTimeValue == 0) {
        timeout = RC_MAD_TIMEOUT; 
        if (if3DebugRmpp) 
            IB_LOG_INFINI_INFO_FMT(__func__, "Using default RC_MAD_TIMEOUT = %"CS64u, timeout);
    } else {
        timeout = ((1 << mi->cpi.respTimeValue) * 4ull) + mi->SubnetTO; 
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, "used RespTimeValue=%d, SubnetTO=%d to calculate timeout=%"CS64u, 
                                   (int)mi->cpi.respTimeValue, (int)mi->SubnetTO, timeout);
        }
    }
    mi->timeout = timeout; 
#endif
    
    // drain stale data from response handles
    DRAIN_MAI_STALE(mi->fds); 
    DRAIN_MAI_STALE(mi->fdr);
    
    // now setup the filter to receive the SA responses.  Set up a filter
    // to get the response to our first * message.
    rc = mai_alloc_tid(mi->fds, mi->mclass, &tid); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("unable to allocate tid rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    //
    // the manager could live on my machine. I need to make sure that
    // the filter looks at the method too. Otherwise as the FI loops
    // the mad in the port we will recieve our own message and beleive
    // it is the response from the manager
    fp = &f; 
    memset((void *)&f, 0, sizeof(f)); 
    
    fp->type = MAI_TYPE_ANY; 
    fp->value.mclass = psa->common.MgmtClass; 
    if (psa->common.mr.s.Method <= SA_CM_SET) {
        fp->value.method = SA_CM_GET_RESP;
    } else if (psa->common.mr.s.Method == SA_CM_GETTRACETABLE) {
        fp->value.method = SA_CM_GETTABLE_RESP;
    } else {
        // everybody else, OR in 0x80 to inbound; i.e., 14->94
        fp->value.method = (psa->common.mr.s.Method | MAD_CM_REPLY);
    }
    fp->value.tid = tid; 
    fp->mask.mclass = MAI_FMASK_ALL; 
    fp->mask.method = MAI_FMASK_ALL; 
    fp->mask.tid = MAI_FMASK_ALL; 
    fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK); 
	fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter;
    
    // create the filter
    rc = mai_filter_hcreate(mi->fds, fp, VFILTER_SHARE, &fh); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("Unable to create filter rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // initialize common fields of the MAI MAD for a SA request
    (void)if3_mngr_sa_mad_init(&mad, mi, psa->common.MgmtClass, psa->common.BaseVersion, psa->common.ClassVersion, tid); 
    if (0 && mad.addrInfo.dlid == mad.addrInfo.slid) {
        mad.type = MAI_TYPE_INTERNAL;
    } else {
        mad.type = MAI_TYPE_EXTERNAL;
    }
    
    // initialize SA request specific fields of the MAI MAD 
    mad.base.method = psa->common.mr.s.Method; 
    mad.base.status = psa->common.u.NS.Status.AsReg16; 
    mad.base.aid = psa->common.AttributeID; 
    mad.base.amod = psa->common.AttributeModifier; 
    
    saHdr.RmppHdr = psa->RmppHdr; 
    saHdr.SaHdr = psa->SaHdr; 
    
    (void)BSWAPCOPY_SA_HDR(&saHdr, (SA_MAD_HDR *)mad.data);
    if (mi->rmppMngrfd == NULL) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR_FMT(__func__, "Handle not set"); 
        goto bail;
    }
    
    // initialize and open RMPP connection for the STL manager
    if ((fd_rmpp_usrid = rmpp_is_cnx_open(mi->rmppMngrfd)) == -1 || rmpp_is_cnx_partial_open(fd_rmpp_usrid)) {
        fd_rmpp_usrid = rmpp_mngr_open_cnx(
           mi->rmppMngrfd,   
           MAI_GSI_QP, 
           mi->dev, 
           mi->port, 
           mi->rmppPool, 
           mi->rmppDataLength, 
           mi->rmppMaxCntxt, 
           (mi->rmppCreateFilters) ? &mi->rmppGetfd : NULL, 
           (mi->rmppCreateFilters) ? &mi->rmppGetTablefd : NULL, 
           mi->mclass, 
           if3_rmpp_get_method_text, 
           cs_getAidName, 
           if3_pre_process_request, 
           if3_pre_process_response, 
           if3_is_master, 
           LOCAL_MOD_ID, 
           NULL
           ); 
        
        if (fd_rmpp_usrid == -1) {
            rc = VSTATUS_NOHANDLE; 
            IB_LOG_ERROR_FMT(__func__, "Unable to open connection"); 
            goto bail;
        }
    }
                //    
    // get RMPP context to be associated with the MAD request 
    if ((dbsync_cntxt = rmpp_cntxt_get(fd_rmpp_usrid, &mad, &processMad)) == NULL || !processMad) {
        rc = VSTATUS_NOMEM; 
        IB_LOG_ERROR_FMT(__func__, "Unable to create context"); 
        goto bail;
    }
    
    // before sending the MAD request, perform any required pre-processing of
    // the RMPP context and MAD request    
    if (!(rc = rmpp_pre_process_request(fd_rmpp_usrid, &mad, dbsync_cntxt))) {
        attribOffset = dataLength + Calculate_Padding(dataLength); 
        // setup attribute offset for RMPP transfer
        dbsync_cntxt->attribLen = attribOffset; 
        
        // allocate RMPP request context structure
        rmpp_cntxt_data(fd_rmpp_usrid, dbsync_cntxt, psa->Data, attribOffset); 
        
        // send DB Sync single MAD request to the remote STL SM
        if (VSTATUS_OK != (rc = rmpp_send_request(&mad, dbsync_cntxt))) {
            IB_LOG_ERROR_FMT(__func__, "Failed to send RMPP DB Sync request to remote SM: rc %d", rc); 
        } else while(1) {
			uint32_t tmpLength = *bufferLength;

            // release context for DB Sync single MAD request
            rmpp_cntxt_full_release(dbsync_cntxt);

            // wait for response from the remote STL SM
            rc = rmpp_receive_response(fd_rmpp_usrid, mi, &imad, buffer, &tmpLength, cb, context); 
            if (if3DebugRmpp) {
                if (rc == VSTATUS_OK) {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received OK, len %d", tmpLength);
                } else {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received FAILED, rc %d", rc);
                }
            }

            // no writer thread was implemented for DB Synch, like it was for the SA,
            // PM, PA, and FE.  The writer thread would normally handle the filtering
            // and processing of ACK responses.  DB synch main is the reader/writer thread,
            // so process the ACKs for the Slave here. 
            (void)BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)imad.data, &samad, STL_SA_DATA_LEN);
            if (!rc && samad.header.rmppType == RMPP_TYPE_ACK) {
                if (if3DebugRmpp)
                    IB_LOG_INFINI_INFO_FMT(__func__, "Got ACK packet");
                continue;
            }
            
			*bufferLength = tmpLength;
            // get MAD status from the header
            *madRc = imad.base.status; 
			break;
        } 
    }
    
bail:

    if (fh) {
        if (mai_filter_hdelete(mi->fds, fh)) {
            IB_LOG_WARN0("filter delete failed");
        }
    }
    // deallocate the context
    if (dbsync_cntxt) {
       rmpp_cntxt_full_release(dbsync_cntxt);
    }
    
    IB_EXIT(__func__, rc); 
    return rc;
}

static Status_t
if3_dbsync_send_multi_mad (IBhandle_t fd, SA_MAD *psa,
                           uint8_t *dataBuffer, uint32_t dataLength,
                           uint8_t *buffer, uint32_t *bufferLength, uint32_t *madRc,
                           CBTxRxFunc_t cb, void *context)
{ 
    
    uint32_t rc = VSTATUS_OK; 
    ManagerInfo_t *mi; 
    uint64_t tid; 
    IBhandle_t fh = 0; 
    Filter_t f, *fp; 
    Mai_t mad; 
    uint64_t timeout; 
    rmpp_cntxt_t *dbsync_cntxt = NULL; 
    uint32_t attribOffset; 
    uint8_t processMad = 0, omethod; 
    int fd_rmpp_usrid = -1; 
    SA_MAD_HDR saHdr; 
    STL_SA_MAD samad;
    
    IB_ENTER(__func__, psa, dataLength, 0, 0); 
    
    // buffer and callback pointer parameters are optional    
    if (!psa || !bufferLength || !madRc) {
        IB_LOG_ERROR_FMT(__func__, "Missing required input parameter"); 
        return VSTATUS_ILLPARM; 
    }

    // find the handle stuff
    rc = if3_mngr_locate_minfo(fd, &mi); 
    
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // setup RMPP related fields 
    (void)if3_set_rmpp_minfo(mi); 
    
    // 
    // calculating Response Timeout Period
#ifdef __SIMULATOR__
    timeout = mi->timeout; 
#else
    if (mi->cpi.respTimeValue == 0) {
        timeout = RC_MAD_TIMEOUT; 
        if (if3DebugRmpp) 
            IB_LOG_INFINI_INFO("Using default RC_MAD_TIMEOUT = ", timeout);
    } else {
        timeout = ((1 << mi->cpi.respTimeValue) * 4ull) + mi->SubnetTO; 
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, "used RespTimeValue=%d, SubnetTO=%d to calculate timeout=%"CS64u, 
                                   (int)mi->cpi.respTimeValue, (int)mi->SubnetTO, timeout);
        }
    }
    mi->timeout = timeout; 
#endif
    
    // drain stale data from response handles
    DRAIN_MAI_STALE(mi->fds); 
    DRAIN_MAI_STALE(mi->fdr);
    
    // now setup the filter to receive the SA responses.  Set up a filter
    // to get the response to our first * message.
    rc = mai_alloc_tid(mi->fds, mi->mclass, &tid); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("unable to allocate tid rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    //
    // the manager could live on my machine. I need to make sure that
    // the filter looks at the method too. Otherwise as the FI loops
    // the mad in the port we will recieve our own message and beleive
    // it is the response from the manager
    fp = &f; 
    memset((void *)&f, 0, sizeof(f)); 
    
    fp->type = MAI_TYPE_ANY; 
    fp->value.mclass = psa->common.MgmtClass; 
    if (psa->common.mr.s.Method <= SA_CM_SET) {
        fp->value.method = SA_CM_GET_RESP;
    } else if (psa->common.mr.s.Method == SA_CM_GETTRACETABLE) {
        fp->value.method = SA_CM_GETTABLE_RESP;
    } else {
        // everybody else, OR in 0x80 to inbound; i.e., 14->94
        fp->value.method = (psa->common.mr.s.Method | MAD_CM_REPLY);
    }
    fp->value.tid = tid; 
    fp->mask.mclass = MAI_FMASK_ALL; 
    fp->mask.method = MAI_FMASK_ALL; 
    fp->mask.tid = MAI_FMASK_ALL; 
    fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK); 
	fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter;
    
    // create the filter
    rc = mai_filter_hcreate(mi->fds, fp, VFILTER_SHARE, &fh); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("Unable to create filter rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // initialize common fields of the MAI MAD for a SA request
    (void)if3_mngr_sa_mad_init(&mad, mi, psa->common.MgmtClass, psa->common.BaseVersion, psa->common.ClassVersion, tid); 
    if (0 && mad.addrInfo.dlid == mad.addrInfo.slid) {
        mad.type = MAI_TYPE_INTERNAL;
    } else {
        mad.type = MAI_TYPE_EXTERNAL;
    }
    
    // initialize SA request specific fields of the MAI MAD 
    omethod = mad.base.method = psa->common.mr.s.Method; 
    mad.base.status = psa->common.u.NS.Status.AsReg16; 
    mad.base.aid = psa->common.AttributeID; 
    mad.base.amod = psa->common.AttributeModifier; 
    
    saHdr.RmppHdr = psa->RmppHdr; 
    saHdr.SaHdr = psa->SaHdr; 
    
    (void)BSWAPCOPY_SA_HDR(&saHdr, (SA_MAD_HDR *)mad.data);
    if (mi->rmppMngrfd == NULL) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR_FMT(__func__, "Handle not set"); 
        goto bail;
    }
    
    // initialize and open RMPP connection for the STL manager
    if ((fd_rmpp_usrid = rmpp_is_cnx_open(mi->rmppMngrfd)) == -1 || rmpp_is_cnx_partial_open(fd_rmpp_usrid)) {
        fd_rmpp_usrid = rmpp_mngr_open_cnx(
           mi->rmppMngrfd,   
           MAI_GSI_QP, 
           mi->dev, 
           mi->port, 
           mi->rmppPool, 
           mi->rmppDataLength, 
           mi->rmppMaxCntxt,
           (mi->rmppCreateFilters) ? &mi->rmppGetfd : NULL, 
           (mi->rmppCreateFilters) ? &mi->rmppGetTablefd : NULL, 
           mi->mclass, 
           if3_rmpp_get_method_text, 
           cs_getAidName, 
           if3_pre_process_request, 
           if3_pre_process_response, 
           if3_is_master, 
           LOCAL_MOD_ID, 
           NULL
           ); 
        
        if (fd_rmpp_usrid == -1) {
            rc = VSTATUS_NOHANDLE; 
            IB_LOG_ERROR_FMT(__func__, "Unable to open connection"); 
            goto bail;
        }
    }
                //    
    // get RMPP context to be associated with the MAD request 
    if ((dbsync_cntxt = rmpp_cntxt_get(fd_rmpp_usrid, &mad, &processMad)) == NULL || !processMad) {
        rc = VSTATUS_NOMEM; 
        IB_LOG_ERROR_FMT(__func__, "Unable to create context"); 
        goto bail;
    }

    // before sending the MAD request, perform any required pre-processing of
    // the RMPP context and MAD request    
    if (!(rc = rmpp_pre_process_request(fd_rmpp_usrid, &mad, dbsync_cntxt))) {
        attribOffset = dataLength + Calculate_Padding(dataLength); 
        // setup attribute offset for RMPP transfer
        dbsync_cntxt->attribLen = attribOffset; 
        
        // allocate RMPP request context structure
        rmpp_cntxt_data(fd_rmpp_usrid, dbsync_cntxt, dataBuffer, attribOffset); 
        uint32_t tmpLength;   
send:
        // send DB Sync request to the remote STL SM
        if (VSTATUS_OK != (rc = rmpp_send_request(&mad, dbsync_cntxt))) {
            IB_LOG_ERROR_FMT(__func__, "Failed to send RMPP DB Sync request to remote SM: rc %d", rc); 
        } else {
            tmpLength = *bufferLength;
            // wait for response from the remote STL SM
            rc = rmpp_receive_response(fd_rmpp_usrid, mi, &mad, buffer, &tmpLength, cb, context); 
            if (if3DebugRmpp) {
                if (rc == VSTATUS_OK) {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received OK, len %d", tmpLength);
                } else {
                    IB_LOG_INFINI_INFO_FMT(__func__, "Data received FAILED, rc %d", rc);
                }
            }

            // no writer thread was implemented for DB Synch, like it was for the SA,
            // PM, PA, and FE.  The writer thread would normally handle the filtering
            // and processing of ACK responses.  DB synch main is the reader/writer thread,
            // so process the ACKs for the Slave here. 
            (void)BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)mad.data, &samad, STL_SA_DATA_LEN);
            if (!rc && samad.header.rmppType == RMPP_TYPE_ACK) {
                if (if3DebugRmpp)
                    IB_LOG_INFINI_INFO_FMT(__func__, "Got ACK packet");
                // in RMPP the method field of an ACK response is modified to
                // indicate a response method ID, so restore the method of the
                // original DB Sync request 
                mad.base.method = omethod; 
                goto send;
            }
            
            *bufferLength = tmpLength;
            // get MAD status from the header
            *madRc = mad.base.status; 
        }
    }
    
bail:
    if (fh) {
        if (mai_filter_hdelete(mi->fds, fh)) {
            IB_LOG_WARN0("filter delete failed");
        }
    }

    // deallocate the context
    if (dbsync_cntxt) {
        rmpp_cntxt_full_release(dbsync_cntxt);
    }
    
    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_dbsync_cmd_from_mngr (IBhandle_t fd, Mai_t *maip, uint8_t *buffer, uint32_t *bufferLength,
                   uint32_t *madRc, CBTxRxFunc_t cb, void *context)
{ 
    
    uint32_t obufferLength, rc = VSTATUS_OK;
    ManagerInfo_t *mi; 
    Mai_t imad; 
    STL_SA_MAD samad; 
    IBhandle_t fh = 0; 
    Filter_t f, *fp; 
    int fd_rmpp_usrid = -1;
    
    IB_ENTER(__func__, buffer, bufferLength, 0, 0); 
    
    // buffer and callback pointer parameters are optional    
    if (!buffer || !bufferLength || !madRc) {
        IB_LOG_ERROR_FMT(__func__, "Missing required input parameter"); 
        return VSTATUS_ILLPARM; 
    }
    
    // retain original length of the receive buffer    
    obufferLength = *bufferLength;
        
	*madRc = MAD_STATUS_SA_NO_RECORDS;

    // find the handle stuff
    rc = if3_mngr_locate_minfo(fd, &mi); 
    
    if (rc) {
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // setup RMPP related fields 
    (void)if3_set_rmpp_minfo(mi); 
    
    // drain stale data from response handle
    DRAIN_MAI_STALE(mi->fds); 
    DRAIN_MAI_STALE(mi->fdr); 
    
    if (mi->rmppMngrfd == NULL) {
        rc = VSTATUS_ILLPARM; 
        IB_LOG_ERROR_FMT(__func__, "Handle not set"); 
        goto bail;
    }
    fp = &f; 
    memset((void *)&f, 0, sizeof(f)); 
    
    fp->type = MAI_TYPE_ANY; 
    fp->value.mclass = mi->mclass; 

    // initialize the filter
    fp->mask.mclass = MAI_FMASK_ALL; 
    fp->active = (MAI_ACT_TYPE | MAI_ACT_FMASK); 
	fp->mai_filter_check_packet = rmpp_gsi_check_packet_filter;
    
    // create the filter
    rc = mai_filter_hcreate(mi->fds, fp, VFILTER_SHARE, &fh); 
    if (rc != VSTATUS_OK) {
        IB_LOG_ERRORRC("Unable to create filter rc:", rc); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    // initialize and open RMPP connection for the STL manager
    if ((fd_rmpp_usrid = rmpp_is_cnx_open(mi->rmppMngrfd)) == -1 || rmpp_is_cnx_partial_open(fd_rmpp_usrid)) {
        fd_rmpp_usrid = rmpp_mngr_open_cnx(
           mi->rmppMngrfd,   
           MAI_GSI_QP, 
           mi->dev, 
           mi->port, 
           mi->rmppPool, 
           mi->rmppDataLength, 
           mi->rmppMaxCntxt, 
           (mi->rmppCreateFilters) ? &mi->rmppGetfd : NULL, 
           (mi->rmppCreateFilters) ? &mi->rmppGetTablefd : NULL, 
           mi->mclass, 
           if3_rmpp_get_method_text, 
           cs_getAidName, 
           if3_pre_process_request, 
           if3_pre_process_response, 
           if3_is_master, 
           LOCAL_MOD_ID, 
           NULL
           ); 
        
        if (fd_rmpp_usrid == -1) {
            rc = VSTATUS_NOHANDLE; 
            IB_LOG_ERROR_FMT(__func__, "Unable to open connection"); 
            goto bail;
        }
    }

recv:
    // wait for response from the remote STL Master SM
    rc = rmpp_receive_response(fd_rmpp_usrid, mi, &imad, buffer, bufferLength, cb, context); 
    if (if3DebugRmpp) {
        if (rc == VSTATUS_OK) {
            IB_LOG_INFINI_INFO_FMT(__func__, "Data received OK, len %d", *bufferLength);
        } else {
            IB_LOG_INFINI_INFO_FMT(__func__, "Data received FAILED, rc %d", rc);
        }
    }

    // get MAD
    memcpy(maip, &imad, sizeof(Mai_t)); 

    // get MAD status from the header
	if (rc == VSTATUS_OK ) {
		*madRc = imad.base.status; 
	}

    // set response status to no_records since rmpp returns zero length record with status OK 
    // in those cases
    if (!*madRc && !*bufferLength) {
        *madRc = MAD_STATUS_SA_NO_RECORDS;
    }

    // no reader thread was implemented for DB Synch, like it was for the SA,
    // PM, PA, and FE.  The reader thread would normally handle the filtering
    // and processing of ACK responses.  DB synch main is the reader/writer thread,
    // so process the ACKs for the Slave here. 
    (void)BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)imad.data, &samad, STL_SA_DATA_LEN);
    if (!rc && samad.header.rmppType == RMPP_TYPE_ACK) {
        // large data response in progress, so restore the original input
        // buffer length for on-going processing of the response.
        *bufferLength = obufferLength;
        if (if3DebugRmpp)
            IB_LOG_INFINI_INFO_FMT(__func__, "Got ACK packet");
        goto recv;
    }

bail:
    if (fh) {
        if (mai_filter_hdelete(mi->fds, fh)) {
            IB_LOG_WARN0("filter delete failed");
        }
    }
    
    IB_EXIT(__func__, rc); 
    return rc;
}

Status_t
if3_mngr_register_sa (IBhandle_t fd, uint8_t * servName, uint64_t servID, uint32_t option)
{

	uint32_t rc;
	ManagerInfo_t *mi;
	IB_SERVICE_RECORD serviceFound;
	uint32_t count;

	IB_ENTER(__func__, fd, servName, servID, option);

	/*
	 * find the handle stuff
	 */

	rc = if3_mngr_locate_minfo (fd, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	/*
	 * Drain stale data from response handle
	 */

	DRAIN_MAI_STALE (mi->fds);


	if (servName == NULL ||
	    (option != IF3_REGFORCE_FABRIC &&
	     option != IF3_REGFORCE_PORT &&
	     option != IF3_REGTRY_FABRIC && option != IF3_REGTRY_PORT)) {
		rc = VSTATUS_ILLPARM;
		IB_EXIT(__func__, rc);
		return rc;
	}

	if (mi->isRegistered && mi->servID != servID) {

		/*
		 * We are registering a new service on the handle. Note we * are 
		 * treating a handle as the end point of service. * We are not
		 * allowing more than one service to be registered * against the 
		 * same handle.
		 */

		IB_LOG_ERROR ("Handle already has service:", mi->servID);
		rc = VSTATUS_BUSY;
		goto done;
	}

	if (mi->cpi_valid == 0) {
		rc = if3_mngr_get_sa_classportInfo (mi);
		if (rc != VSTATUS_OK) {

			IB_EXIT(__func__, rc);
			return rc;

		}
	}



	/*
	 * First test if a service is already registered
	 */

	if (option == IF3_REGTRY_FABRIC || option == IF3_REGFORCE_FABRIC)
		rc = if3_mngr_query_service (fd, servName, servID, IF3_FABRIC_SERVICE,
				       &serviceFound, &count);
	else
		rc = if3_mngr_query_service (fd, servName, servID, IF3_PORT_SERVICE,
				       &serviceFound, &count);

	if (rc != VSTATUS_OK) {
		if (rc != VSTATUS_NOT_FOUND) {

			/*
			 * Something went wrong
			 */

			if (rc == VSTATUS_TOO_LARGE) {

				/*
				 * There wer too many entries to handle .. so we
				 * know * SA does have the records
				 */

				IB_LOG_ERROR0 ("too many service records at SA");
				rc = VSTATUS_BUSY;
			} else {
				IB_LOG_ERRORRC("error querrying SA rc:", rc);

			}
			goto done;
		}

	}

	if (count != 0 && (option == IF3_REGFORCE_FABRIC || option == IF3_REGFORCE_PORT)) {

		/*
		 * Delete all existing copy of the service at SA
		 */

		/*
		 * First query the then delete
		 */

		if (option == IF3_REGFORCE_FABRIC)
			rc = if3_mngr_del_service (fd, servName, servID, IF3_FABRIC_SERVICE);
		else
			rc = if3_mngr_del_service (fd, servName, servID, IF3_PORT_SERVICE);

		if (rc != VSTATUS_OK) {
			if (rc != VSTATUS_NOT_FOUND) {

				/*
				 * Something went wrong
				 */

				IB_LOG_ERRORRC("service delete failed at SA rc:", rc);
				goto done;
			}

		}
	}

	if (option == IF3_REGTRY_FABRIC || option == IF3_REGTRY_PORT) {

		/*
		 * Lets set how may records were found
		 */

		if (count) {
			IB_LOG_INFO ("service records already exist at  SA", count);
			rc = VSTATUS_BUSY;
			goto done;
		}
	}

	/*
	 * If we get here then we can proceed witht the registration * 
	 */

	rc = if3_mngr_reg_service (fd, servName, servID);

	if (rc != VSTATUS_OK) {

		IB_LOG_ERRORRC("Error registering service rc:", rc);
	} else {

		/*
		 * Service registered successfully. Keep the information around 
		 */

		(void) if3_mngr_locate_minfo (fd, &mi);

		strncpy ((void *)mi->servName, (void *)servName, SA_SRP_NAME_LEN);
		mi->servID = servID;
		mi->isRegistered = 1;
	}

      done:

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_mngr_deregister_sa (IBhandle_t fd)
{
	uint32_t rc;
	ManagerInfo_t *mi;

	IB_ENTER(__func__, fd, 0, 0, 0);

	/*
	 * find the handle stuff
	 */

	rc = if3_mngr_locate_minfo (fd, &mi);

	if (rc) {
		IB_EXIT(__func__, rc);
		return rc;
	}

	if (mi->isRegistered == 0) {
		rc = VSTATUS_ILLPARM;
		IB_LOG_INFINI_INFO ("no registration exist on handle", fd);
	} else {

		/*
		 * Drain stale data from response handle
		 */

		DRAIN_MAI_STALE (mi->fds);

		rc = if3_mngr_del_service (fd, mi->servName, mi->servID, IF3_PORT_SERVICE);

		if (rc == VSTATUS_OK) {
			mi->isRegistered = 0;
		} else if (rc != VSTATUS_NOT_FOUND) {
			IB_LOG_INFINI_INFORC("failed to deregister service rc:", rc);
		} else {
            // just tell caller OK.  Record already removed.
            rc = VSTATUS_OK;
        }
	}

	IB_EXIT(__func__, rc);
	return rc;
}

Status_t
if3_mngr_get_fe_cmd (Mai_t * mad, uint16_t * cmd)
{
	uint32_t rc = VSTATUS_OK;

	IB_ENTER(__func__, mad, cmd, 0, 0);

	if (mad == NULL || cmd == NULL) {
		rc = VSTATUS_ILLPARM;
		IB_EXIT(__func__, rc);
		return rc;
	}
	*cmd = mad->base.aid;
	IB_EXIT(__func__, rc);
	return rc;

}

Status_t
if3_mngr_get_fe_cmd_option (Mai_t * mad, uint32_t * cmd)
{
	uint32_t rc = VSTATUS_OK;

	IB_ENTER(__func__, mad, cmd, 0, 0);

	if (mad == NULL || cmd == NULL) {
		rc = VSTATUS_ILLPARM;
		IB_EXIT(__func__, rc);
		return rc;
	}
	*cmd = mad->base.amod;
	IB_EXIT(__func__, rc);
	return rc;
}

#ifdef __VXWORKS__
void if3_vxworks_init(int start)
{
    // initialization control for Esm_init
    if (start) {
        initialized = 0;
        if3_init();
    } else {
        /* SM being stopped */
        vs_lock_delete (&lock);
        initialized = 0;
    }
}
#endif
