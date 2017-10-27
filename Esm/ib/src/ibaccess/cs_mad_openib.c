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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include <cs_g.h>
#include <mai_g.h>
#include <mal_g.h>
#include <ib_macros.h>
#include <if3.h>
             
#include "opamgt_priv.h"
#include <ib_mad.h>
#include <ib_sa.h>
#include <iba/ib_pkt.h>
#include <iba/ib_mad.h>
#include <iba/ib_sm_priv.h>
#include <iba/public/statustext.h>
#include <iba/ib_rmpp.h>
#include <iba/stl_sm_priv.h>
#include <iba/stl_sa_priv.h>
#include <iba/stl_pm.h>
#include <iba/stl_pa_priv.h>

//==============================================================================

#define IB_ISSM_DEVICEPATH_MAXLEN 32

//==============================================================================

//==============================================================================

#define	IB_HANDLE2DEV(handle)		(( (handle) >> 8) & 0xFF)
#define	IB_HANDLE2PORT(handle)		(( (handle)     ) & 0xFF)

#define	IB_MAKEHANDLE(dev, port) \
	((((dev) & 0xff) << 8) | ((port) & 0xff))

#define	IB_FROMHANDLE(handle, dev, port) \
	dev  = IB_HANDLE2DEV(handle);        \
	port = IB_HANDLE2PORT(handle);

//==============================================================================

static uint8_t ib_truescale_oui[] = {OUI_TRUESCALE_0, OUI_TRUESCALE_1, OUI_TRUESCALE_2};

typedef struct
{
	FILE   *fp;
	Lock_t  lock;
} ib_issm_t;

static ib_issm_t ib_issm;

static struct omgt_class_args sm_class_args[] = {
	{ IB_BASE_VERSION, MAD_CV_SUBN_DR, 1, 
		.is_responding_client=1, .is_trap_client=1, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ IB_BASE_VERSION, MAD_CV_SUBN_LR, 1, 
		.is_responding_client=1, .is_trap_client=1, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0},
	{ STL_BASE_VERSION, MAD_CV_SUBN_DR, STL_SM_CLASS_VERSION, 
		.is_responding_client=1, .is_trap_client=1, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ STL_BASE_VERSION, MAD_CV_SUBN_LR, STL_SM_CLASS_VERSION, 
		.is_responding_client=1, .is_trap_client=1, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ STL_BASE_VERSION, MAD_CV_VENDOR_DBSYNC, STL_SA_CLASS_VERSION, 
		.oui=ib_truescale_oui, .use_methods=1, 
		.methods={ FE_MNGR_PROBE_CMD, FE_MNGR_CLOSE_CMD,
    	  FM_CMD_SHUTDOWN, SA_CM_GET, SA_CM_SET, SA_CM_GETTABLE }},
	{ STL_BASE_VERSION, MAD_CV_SUBN_ADM, STL_SA_CLASS_VERSION, 
		.oui=0, .use_methods=1, 
		.methods={ SA_CM_GET, SA_CM_SET, SA_CM_GETTABLE, 
				   SA_CM_GETTRACETABLE, SA_CM_GETMULTI, SA_CM_DELETE }},
	{ IB_BASE_VERSION, MAD_CV_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION, 
		.oui=0, .use_methods=1, 
		.methods={ SA_CM_GET, SA_CM_SET, SA_CM_GETTABLE, 
				   SA_CM_GETTRACETABLE, SA_CM_GETMULTI, SA_CM_DELETE }},
	{ 0 }
};

static struct omgt_class_args fe_class_args[] = {
	{ IB_BASE_VERSION, MAD_CV_VENDOR_FE, 1, 
		.kernel_rmpp=0, .oui=ib_truescale_oui, .use_methods=1, 
		.methods={ FE_MNGR_PROBE_CMD,
					FE_MNGR_CLOSE_CMD, FM_CMD_SHUTDOWN, RMPP_CMD_GET, RMPP_CMD_GETTABLE }},
	{ IB_BASE_VERSION, MAD_CV_VFI_PM, 1,
		.kernel_rmpp=0, .oui=ib_truescale_oui, .use_methods=1,
		.methods={ FE_CMD_RESP, RMPP_CMD_GET, RMPP_CMD_GETTABLE }},
};

static struct omgt_class_args fe_proc_class_args[] = {
	{ IB_BASE_VERSION, MAD_CV_VENDOR_FE, 1, 
		.kernel_rmpp=0, .oui=ib_truescale_oui, .use_methods=1, 
		.methods={ FE_MNGR_PROBE_CMD,
					FE_MNGR_CLOSE_CMD, FM_CMD_SHUTDOWN, RMPP_CMD_GET, RMPP_CMD_GETTABLE }},
	{ STL_BASE_VERSION, MAD_CV_VFI_PM, STL_PM_CLASS_VERSION, 
        .kernel_rmpp=0, .oui=ib_truescale_oui, .use_methods=1,
        .methods={ FE_CMD_RESP }},
	{ IB_BASE_VERSION, MAD_CV_SUBN_DR, 1, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ IB_BASE_VERSION, MAD_CV_SUBN_LR, 1, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0},
	{ STL_BASE_VERSION, MAD_CV_SUBN_DR, STL_SM_CLASS_VERSION, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ STL_BASE_VERSION, MAD_CV_SUBN_LR, STL_SM_CLASS_VERSION, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ STL_BASE_VERSION, MAD_CV_SUBN_ADM, STL_SA_CLASS_VERSION, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ IB_BASE_VERSION, MAD_CV_SUBN_ADM, IB_SUBN_ADM_CLASS_VERSION, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0 },
	{ 0 }
};

static struct omgt_class_args pm_class_args[] = {
	{ STL_BASE_VERSION, MAD_CV_VFI_PM, STL_PM_CLASS_VERSION, 
		.kernel_rmpp = 0, .oui=ib_truescale_oui, .use_methods=1, 
		.methods={ FE_MNGR_PROBE_CMD, FE_MNGR_CLOSE_CMD, FM_CMD_SHUTDOWN,
					STL_PA_CMD_GET, STL_PA_CMD_SET, STL_PA_CMD_GETTABLE }},
	{ STL_BASE_VERSION, MAD_CV_PERF, STL_PM_CLASS_VERSION, 
		.is_responding_client=0, .is_trap_client=0, .is_report_client=0, .kernel_rmpp=0, .oui=0, .use_methods=0},
	{ 0 }
};

int ib_instrumentJmMads = 0;

/** TODO: currently there is only one open to the opamgt later (translated into only one open file
 *  handle to the ib_umad driver) and shared by all the threads spawned in Esm/ib/src/ibaccess/smi/sm/sm_main.c,
 *  which is the reason behind the complicated filtering mechanism in the upper MAI layer. To streamline
 *  the operation, the field in struct mai_dc->hndl can be replaced with struct omgt_port * so that each
 *  thread that calls mai_open() can have a separate port open in opamgt (i.e., a separate file handle
 *  to the ib_umad driver): mai_open() -> mai_get_dc() -> ib_attach_sma(), the last function (ib_attach_sma)
 *  should call omgt_open_port_by_num(). Similarly, ib_detach_sma should call omgt_close_port() to close
 *  the corresponding port. In addition, all the sending/receiving function (ib_recv_sma, etc) should
 *  be modified to carrry the struct omgt_port * parameter install of the IBhandle parameter). Of course,
 *  the ib_init_devport() and ib_init_guid() should not be used any more or modified not to call the
 *  omgt_open_port_xx() function.
 *  Note: the xx_mai_to_wire() or xx_wire_to_mai function
 *  involve buffer copying, which is bad for performance.
 *  */
struct omgt_port *g_port_handle = NULL;

//==============================================================================

// We're hacking into MAI internals here.  In the case of local packets, we
// just call mai_mad_process to short-circuit the path.
extern int mai_mad_process(Mai_t *mad, int *filterMatch);

extern uint32_t smDebugPerf; 
extern uint32_t sm_debug;

//==============================================================================

static Status_t ib_mai_to_wire(Mai_t *mad, uint8_t *buf);
static Status_t stl_wire_to_mai(uint8_t *buf, Mai_t *mad, int len);
static void stl_mad_to_mai(MAD* mad, Mai_t *mai);
static Status_t stl_mai_to_wire(Mai_t *mad, uint8_t *buf, uint32_t *bufLen);

//==============================================================================
// ib_init_common
//==============================================================================
static Status_t ib_init_common(void)
{
	Status_t status;
	
	ib_issm.fp = NULL;
	status = vs_lock_init(&ib_issm.lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't initialize issm device lock; rc:", status);
		return status;
	}
	
	return VSTATUS_OK;
}

//==============================================================================
// ib_init_devport
//==============================================================================
Status_t
ib_init_devport(uint32_t *devp, uint32_t *portp, uint64_t *Guidp, struct omgt_params *session_params)
{
	OMGT_STATUS_T status;

	IB_ENTER(__func__, devp, portp, Guidp, 0);

	// Check if the port is opened or
	if (g_port_handle) {
		uint64_t port_guid = 0;
		int32_t hfi_num = -1;
		uint8_t port_num = 0;
		(void)omgt_port_get_port_guid(g_port_handle, &port_guid);
		(void)omgt_port_get_hfi_num(g_port_handle, &hfi_num);
		(void)omgt_port_get_hfi_port_num(g_port_handle, &port_num);
		IB_LOG_ERROR_FMT(__func__, "one port (%d/%d GUID 0x%.16"CS64"x) is already opened",
			hfi_num, port_num, port_guid);
		return VSTATUS_BUSY;
	}
	status = ib_init_common();
	if (status != VSTATUS_OK) {
		IB_EXIT(__func__, status);
		return status;
	}


	if (Guidp != NULL && *Guidp != 0ULL) {
		status = omgt_open_port_by_guid(&g_port_handle, *Guidp, session_params);
		if (status != FSUCCESS) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to bind to GUID 0x%.16"CS64"x; status: %u",
				*Guidp, status);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		// note that the opamgt interface uses 1-based devices
		if (devp != NULL) {
			(void)omgt_port_get_hfi_num(g_port_handle, (int32_t *)devp);
			(*devp)--;
		}
		if (portp != NULL) {
			uint8_t port_num = 0;
			(void)omgt_port_get_hfi_port_num(g_port_handle, &port_num);
			*portp = port_num;
		}
	} else if (devp != NULL && portp != NULL) {
		// note that the opamgt interface uses 1-based devices
		status = omgt_open_port_by_num(&g_port_handle, *devp + 1, *portp, session_params);
		if (status != FSUCCESS) {
			// PR 128290 - MWHEINZ - This is a bit of a hack. Some parts of
			// the stack use 0-based counting of ports and devices but other
			// parts use 1-based. We assume the caller of this function
			// is using zero-based counting, so we add 1 when we call
			// omgt_open_port_by_num(), but we also assume the user is
			// expecting a 1-based device number in their error messages.
			IB_LOG_ERROR_FMT(__func__,
				"Failed to bind to device %d, port %d; status: %u",
				*devp + 1, *portp, status);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
		if (Guidp != NULL) {
			(void)omgt_port_get_port_guid(g_port_handle, Guidp);
		}
	} else {
		IB_LOG_ERROR_FMT(__func__,
			"Neither Guid nor device and port were supplied");
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// ib_refresh_devport
//==============================================================================
Status_t
ib_refresh_devport(void)
{
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	if (omgt_mad_refresh_port_pkey(g_port_handle) < 0) {
		IB_LOG_ERROR_FMT(__func__,
		       "Failed to refresh UMAD pkeys");
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// ib_shutdown
//==============================================================================
Status_t ib_shutdown(void)
{
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	ib_disable_is_sm();
	omgt_close_port(g_port_handle);
	g_port_handle = NULL;
	
	IB_EXIT(__func__, 0);
	return VSTATUS_OK;
}

//==============================================================================
// ib_register_sm
//==============================================================================
Status_t
ib_register_sm(int queue_size)
{
	FSTATUS status;

	status = omgt_bind_classes(g_port_handle, sm_class_args);
	if (status != FSUCCESS) {
		IB_LOG_ERROR("Failed to register management classes;",
					 status);
		IB_EXIT(__func__, status);
		return status;
	}

	IB_EXIT(__func__, status);
	return status;
}

//==============================================================================
// ib_register_fe
//==============================================================================
Status_t
ib_register_fe(int queue_size, uint8_t thread)
{
	FSTATUS status;

	IB_ENTER(__func__, 0, 0, 0, 0);

	status = omgt_bind_classes(g_port_handle, (thread) ? fe_class_args : fe_proc_class_args);
	if (status != FSUCCESS) {
		IB_LOG_ERROR("Failed to register FE management classes;",
					 status);
		IB_EXIT(__func__, status);
		return status;
	}

	IB_EXIT(__func__, status);
	return status;
}


//==============================================================================
// ib_register_pm
//==============================================================================
Status_t
ib_register_pm(int queue_size)
{
	FSTATUS status;
	
	IB_ENTER(__func__, 0, 0, 0, 0);

	status = omgt_bind_classes(g_port_handle, pm_class_args);
	if (status != FSUCCESS) {
		IB_LOG_ERROR("Failed to register PM management classes;",
					 status);
		IB_EXIT(__func__, status);
		return status;
	}

	IB_EXIT(__func__, status);
	return status;
}


//==============================================================================
static void
dump_mad(uint8_t *buf, int len, char *prefix)
{
	int i;
	char s[1024], *s0 = s;
	memset(s, 0, sizeof(s));
	for (i = 0; i < len; ++i) {
		sprintf(s0, "%02x", *(buf + i)); s0 += 2;
		if (i % 4 == 3) { sprintf(s0, " "); s0 += 1; }
		if (i % 16 == 15) { IB_LOG_INFINI_INFO_FMT( __func__, "%s%s", prefix, s); s0 = s; memset(s, 0, sizeof(s)); }
	}
}

//==============================================================================
// ib_recv_sma
//==============================================================================
Status_t
ib_recv_sma(IBhandle_t handle, Mai_t *mai, uint64_t timeout)
{
   FSTATUS  status; 
   Status_t rc; 
   uint8_t  *buf = NULL; 
   size_t   len = 0; 
   int      dev, port; 
   int      issmi; 
   struct omgt_mad_addr	addr;
   
   IB_ENTER(__func__, handle, mai, timeout, 0); 
   
   IB_FROMHANDLE(handle, dev, port); 
   
   // wait for inbound mad, no timeout
   memset (&addr, 0, sizeof(addr));
   retry:
   do {
	  status = omgt_recv_mad_alloc(g_port_handle, &buf, &len, timeout, &addr);
   } 
   while (status == FNOT_DONE);
    
   // FSUCCESS - got a packet
   // FTIMEOUT - wait for response timed out, get header of our request
   // FREJECT - unexpected error processing a previous send or its response
   // 				get back at least header of our request
   if (status != FSUCCESS && status != FTIMEOUT && status != FREJECT) {
      // unexpected problem getting packets
      IB_LOG_ERRORSTR("Recieved SMA status:", iba_fstatus_msg(status)); 
      IB_EXIT(__func__, VSTATUS_BAD); 
      return VSTATUS_BAD;
   }

   if (len < sizeof(MAD_COMMON)) {
	   // if too small we can't deal with it and must discard it
	   if (buf) 
		   free(buf); 
	   IB_LOG_ERROR_FMT(__func__, "bad size: %lu status: %s", (unsigned long)len, iba_fstatus_msg(status));
	   goto retry; 
   }
#ifdef DEBUG
   else if (len > sizeof(MAD_COMMON) + STL_MAD_PAYLOAD_SIZE) {
	   // unexpected, we'll ignore the extra bytes of payload
	   IB_LOG_INFO_FMT(__func__, "large size: %lu status: %s", (unsigned long)len, iba_fstatus_msg(status));
   }
#endif
   
   // transfer STL packet to MAI structure   
   rc = stl_wire_to_mai(buf, mai, len); 
   free(buf); 
   
   if (status == FTIMEOUT || status == FREJECT) {
      // extern const char *iba_fstatus_msg(int);
      // printf("omgt_recv TYPE_ERROR: status=%s class=0x%x attr=0x%x method=0x%x\n",
      // 			iba_fstatus_msg(status),mai->base.mclass, mai->base.aid, mai->base.method);
      // this is a packet we tried to send but failed to send or
      // failed to get a response for
      mai->type = MAI_TYPE_ERROR; 
      
      // added info message as part of PR 115443 to show that the request timed out
      if (status == FTIMEOUT && sm_debug && smDebugPerf) {
         IB_LOG_INFINI_INFO_FMT(__func__, "FTIMEOUT of packet status=%s class=0x%x attr=0x%x method=0x%x TID=0x%.16"CS64"X\n", 
                                iba_fstatus_msg(status), mai->base.mclass, mai->base.aid, mai->base.method, mai->base.tid);
      }
   }
   
   if (rc != VSTATUS_OK) {
      IB_LOG_ERRORRC("Error converting MAD from wire format; rc:", rc); 
      IB_EXIT(__func__, rc); 
      return rc;
   }
   
   // SMP's are SMI, all else is GSI
   issmi = ((mai->base.mclass == MAD_CV_SUBN_LR || mai->base.mclass == MAD_CV_SUBN_DR)
            ? 1 : 0); 
   
   // fill in core MAI structure
   mai->dev     = dev; 
   mai->port    = port; 
   mai->qp      = issmi ? 0 : 1; 
   mai->active |=(MAI_ACT_DEV | MAI_ACT_PORT | MAI_ACT_QP); 
   
   // fill in packet header data
   mai->addrInfo.qkey  = addr.qkey; 
   mai->addrInfo.srcqp = addr.qpn;  
   mai->addrInfo.destqp = mai->qp; 
   mai->addrInfo.pkey   = addr.pkey; 
   mai->addrInfo.sl     = addr.sl; 

   if(mai->base.mclass == MAD_CV_SUBN_DR && ((DRStlSmp_t*)(mai->data))->DrSLID == STL_LID_PERMISSIVE && ((DRStlSmp_t*)(mai->data))->DrSLID == STL_LID_PERMISSIVE){
      mai->addrInfo.slid = LID_PERMISSIVE;
	  mai->addrInfo.dlid = LID_PERMISSIVE;
   } else {
      mai->addrInfo.slid = addr.lid;
	  mai->addrInfo.dlid = addr.lid;
   }
   
   /*
   if (qp == 1) 
       IB_LOG_WARN_FMT( __func__, "SL is %d (qp=%d, pkey=0x%x, slid= %d)", mai->addrInfo.sl, mai->qp, addr.pkey, mai->addrInfo.slid);
   */
   
   mai->active    |=MAI_ACT_ADDRINFO;
   
   if (ib_instrumentJmMads
       && mai->base.mclass == 0x03 && mai->base.aid == 0xffb2) {
      IB_LOG_INFINI_INFO_FMT(__func__, 
                             "Received: mclass 0x%02x method 0x%02x aid 0x%04x amod 0x%08x", 
                             mai->base.mclass, mai->base.method, mai->base.aid, mai->base.amod); 
      dump_mad(mai->data, (IB_SA_FULL_HEADER_SIZE + IB_SAMAD_DATA_COUNT), "  ");
   }
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return VSTATUS_OK; 
}

//==============================================================================
// ib_send_sma
//==============================================================================
Status_t
ib_send_sma(IBhandle_t handle, Mai_t * mai, uint64_t timeout)
{
	FSTATUS  status;
	Status_t rc;
	uint8_t  buf[sizeof(Mai_t)];
	int      filterMatch;
	int adjusted_timeout;
	struct omgt_mad_addr	addr;
	
	IB_ENTER(__func__, handle, mai, 0, 0);
	
	ASSERT(mai != NULL);
	
	if (mai->type == MAI_TYPE_INTERNAL) {
		IB_LOG_VERBOSE_FMT(__func__, "local delivery for MAD to 0x%04x on QP %d",
		       mai->addrInfo.dlid, mai->qp);
		// for local delivery
		(void)mai_mad_process(mai, &filterMatch);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}
	
	if (  ib_instrumentJmMads
	   && mai->base.mclass == 0x03 && mai->base.aid == 0xffb2) {
		IB_LOG_INFINI_INFO_FMT( __func__,
			"Sent: mclass 0x%02x method 0x%02x aid 0x%04x amod 0x%08x",
			mai->base.mclass, mai->base.method, mai->base.aid, mai->base.amod);
		dump_mad(mai->data, sizeof(mai->data), "  ");
	}

	rc = ib_mai_to_wire(mai, buf);
	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("Error converting MAD to wire format; rc:", rc);
		IB_EXIT(__func__, rc);
		return rc;
	}
	
	// only send a timeout that is 90% of what the caller thinks it should be.
	// this is to give OFED a moment to clear the MAD off the send list before
	// the caller retries.  without this, the caller's retry might be rejected
	// as a duplicate if it comes in too quickly
	// TBD - once all callers fixed, use timeout as given
	adjusted_timeout = (int)(timeout * 9 / 10 * VTIMER_1_MILLISEC / VTIMER_1S);
	if (adjusted_timeout <= 0 && timeout > 0)
		adjusted_timeout=1;
	memset(&addr, 0, sizeof(addr));
	addr.lid = mai->addrInfo.dlid;
	addr.qpn = mai->addrInfo.destqp;
	addr.qkey = mai->addrInfo.qkey;
	addr.pkey = mai->addrInfo.pkey;
	addr.sl = mai->addrInfo.sl;
	status = omgt_send_mad2(g_port_handle, (void*)buf, IB_MAX_MAD_DATA, &addr, adjusted_timeout, 0);
	if (status != FSUCCESS) {
		if (mai->addrInfo.srcqp != 1) {
			IB_LOG_INFO("Error sending packet via OPENIB interface; status:", status);
		} else {
			IB_LOG_INFO_FMT(__func__,
		       "Error sending packet via OPENIB interface; status: %u (sl= %d, pkey= 0x%x)", 
				status, mai->addrInfo.sl, mai->addrInfo.pkey);
		}
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// stl_send_sma
//==============================================================================
Status_t
stl_send_sma(IBhandle_t handle, Mai_t * mai, uint64_t timeout)
{
	FSTATUS  status;
	Status_t rc;
    uint32_t bufLen = 0;
	uint8_t  buf[sizeof(Mai_t)];
	int      filterMatch;
	int adjusted_timeout;
	struct omgt_mad_addr	addr;
	
	IB_ENTER(__func__, handle, mai, 0, 0);
	
	ASSERT(mai != NULL);
	
	if (mai->type == MAI_TYPE_INTERNAL) {
		IB_LOG_VERBOSE_FMT(__func__, "local delivery for MAD to 0x%04x on QP %d",
		       mai->addrInfo.dlid, mai->qp);
		// for local delivery
		(void)mai_mad_process(mai, &filterMatch);
		IB_EXIT(__func__, VSTATUS_OK);
		return VSTATUS_OK;
	}
	
	if (  ib_instrumentJmMads
	   && mai->base.mclass == 0x03 && mai->base.aid == 0xffb2) {
		IB_LOG_INFINI_INFO_FMT( __func__,
			"Sent: mclass 0x%02x method 0x%02x aid 0x%04x amod 0x%08x",
			mai->base.mclass, mai->base.method, mai->base.aid, mai->base.amod);
		dump_mad(mai->data, sizeof(mai->data), "  ");
	}

	rc = stl_mai_to_wire(mai, buf, &bufLen);
	if (rc != VSTATUS_OK) {
		IB_LOG_ERRORRC("converting MAD to wire format; rc:", rc);
		IB_EXIT(__func__, rc);
		return rc;
	}
	
	// only send a timeout that is 90% of what the caller thinks it should be.
	// this is to give OFED a moment to clear the MAD off the send list before
	// the caller retries.  without this, the caller's retry might be rejected
	// as a duplicate if it comes in too quickly
	// TBD - once all callers fixed, use timeout as given
	adjusted_timeout = (int)(timeout * 9 / 10 * VTIMER_1_MILLISEC / VTIMER_1S);
	if (adjusted_timeout <= 0 && timeout > 0)
		adjusted_timeout=1;

	memset(&addr, 0, sizeof(addr));
	addr.lid = mai->addrInfo.dlid;
	addr.qpn = mai->addrInfo.destqp;
	addr.qkey = mai->addrInfo.qkey;
	addr.pkey = mai->addrInfo.pkey;
	addr.sl = mai->addrInfo.sl;
	//IB_LOG_INFINI_INFO_FMT(__func__, "Sending MAD of size %d bytes", bufLen);
	status = omgt_send_mad2(g_port_handle, (void*)buf, bufLen, &addr, adjusted_timeout, 0);
	if (status != FSUCCESS) {
		if (mai->addrInfo.srcqp != 1) {
			IB_LOG_INFO("Error sending packet via OPENIB interface; status:", status);
		} else {
			IB_LOG_INFO_FMT(__func__,
		       "Error sending packet via OPENIB interface; status: %u (sl= %d, pkey= 0x%x)", 
				status, mai->addrInfo.sl, mai->addrInfo.pkey);
		}
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// ib_attach_sma
//==============================================================================
Status_t
ib_attach_sma(int16_t dev, int8_t port, uint32_t qpn,
              IBhandle_t *handlep, uint8_t *nodeTypep)
{
	FSTATUS status;
	uint8_t type;
	
	IB_ENTER(__func__, dev, port, qpn, 0);
	
	if (handlep != NULL)
		*handlep = IB_MAKEHANDLE(dev, port);
	
	if (nodeTypep != NULL)
	{
		status = omgt_port_get_node_type(g_port_handle, &type);
		if (status == FSUCCESS)
			*nodeTypep = type;
		else
		{
			IB_LOG_WARN("failed to get node type; status:", status);
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
	}
		
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// ib_detach_sma
//==============================================================================
Status_t
ib_detach_sma(IBhandle_t handle)
{
	return VSTATUS_OK;
}

//==============================================================================
// ib_init_sma
//==============================================================================
Status_t
ib_init_sma(uint32_t maxMadBuffers)
{
	return VSTATUS_OK;
}

//==============================================================================
// ib_term_sma
//==============================================================================
Status_t
ib_term_sma(void)
{
	return VSTATUS_OK;
}

//==============================================================================
// ib_shutdown_all
//==============================================================================
void
ib_shutdown_all(void)
{
}

//==============================================================================
// ib_enable_is_sm
//==============================================================================
Status_t ib_enable_is_sm(void)
{
	Status_t status;
	FSTATUS rc;
	char dev[IB_ISSM_DEVICEPATH_MAXLEN+1];
	
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	status = vs_lock(&ib_issm.lock);
	if (status != VSTATUS_OK) {
		vs_unlock(&ib_issm.lock);
		IB_LOG_WARNRC("failed to acquire issm lock; rc:", status);
		IB_EXIT(__func__, status);
		return status;
	}
	
	rc = omgt_get_issm_device(g_port_handle, dev, IB_ISSM_DEVICEPATH_MAXLEN);
	if (rc != FSUCCESS) {
		IB_LOG_WARN("failed to resolve ISSM device name; status:", status);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	
	// if already open, silently skip
	if (ib_issm.fp == NULL) {
		ib_issm.fp = fopen(dev, "rb");
		if (ib_issm.fp == NULL) {
			vs_unlock(&ib_issm.lock);
			IB_LOG_WARN0("failed to open issm device");
			IB_EXIT(__func__, VSTATUS_BAD);
			return VSTATUS_BAD;
		}
	}
	
	vs_unlock(&ib_issm.lock);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// ib_disable_is_sm
//==============================================================================
Status_t ib_disable_is_sm(void)
{
	Status_t status;
	IB_ENTER(__func__, 0, 0, 0, 0);
	
	status = vs_lock(&ib_issm.lock);
	if (status != VSTATUS_OK) {
		IB_LOG_WARNRC("failed to acquire issm lock; rc:", status);
		IB_EXIT(__func__, status);
		return status;
	}
	
	if (ib_issm.fp != NULL) {
		fclose(ib_issm.fp);
		ib_issm.fp = NULL;
	}
	
	vs_unlock(&ib_issm.lock);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// stl_mad_to_mai  
//==============================================================================
static void stl_mad_to_mai(MAD* mad, Mai_t *mai) 
{ 
   mai->base.bversion = mad->common.BaseVersion; 
   mai->base.mclass = mad->common.MgmtClass; 
   mai->base.cversion = mad->common.ClassVersion; 
   mai->base.method = mad->common.mr.AsReg8;    
   if (mad->common.MgmtClass == MCLASS_SM_DIRECTED_ROUTE) {
      mai->base.status = mad->common.u.DR.s.Status; 
      mai->base.hopCount = mad->common.u.DR.HopCount; 
      mai->base.hopPointer = mad->common.u.DR.HopPointer; 
   } else 
      mai->base.status = mad->common.u.NS.Status.AsReg16;
   mai->base.tid = mad->common.TransactionID; 
   mai->base.aid = mad->common.AttributeID; 
   mai->base.amod = mad->common.AttributeModifier;   
}

//==============================================================================
// ib_mai_to_wire  
//==============================================================================
static Status_t
ib_mai_to_wire(Mai_t *mad, uint8_t *buf)
{
	int       isDrouted;
	uint32_t  mask = 0;
	
	IB_ENTER(__func__, mad, buf, 0, 0);

	// Validate the obvious parameters 
	if (mad == NULL) {
		IB_LOG_ERROR0("Invalid parameter: 'mad'");
		IB_EXIT(__func__, VSTATUS_ILLPARM);
		return VSTATUS_ILLPARM;
	}

	// Make sure we have a MAD header 
	mask = MAI_ACT_TYPE;
	if ((mad->active & mask) != mask) {
		IB_LOG_ERROR0("MAI_ACT_TYPE not active!");
		IB_EXIT(__func__, VSTATUS_INVALID_MADT);
		return VSTATUS_INVALID_MADT;
	}

	// Validate the type of MAD 
	switch (mad->type) {
	case MAI_TYPE_EXTERNAL:
		IB_LOG_INFO("Type is MAD", mad->type);
		// MADs require these fields 
		mask = MAI_ACT_ADDRINFO;
		if ((mad->active & mask) != mask) {
			IB_LOG_ERROR("Invalid Mai_t:", mad->active);
			IB_EXIT(__func__, VSTATUS_INVALID_MAD);
			return VSTATUS_INVALID_MAD;
		}
		break;
	default:
		IB_LOG_ERROR("Invalid MAD type:", mad->type);
		IB_EXIT(__func__, VSTATUS_INVALID_TYPE);
		return VSTATUS_INVALID_TYPE;
	}

	// Figure out about Direct Routed or LID routed 
	isDrouted = 0;		/* Assume we are LID routed */
	if (mad->base.mclass == MAD_CV_SUBN_DR)
		isDrouted = 1;

	// Setup reasonable defaults
	// Check for 'Reserved' values in management class 
	if (   mad->base.mclass == 0    || mad->base.mclass == 2
	   || (mad->base.mclass >= 0x50 && mad->base.mclass <= 0x80)
	   ||  mad->base.mclass >= 0x82)
	{
		IB_LOG_ERROR("Reserved management class:",
					 mad->base.mclass);
		IB_EXIT(__func__, VSTATUS_INVALID_MCLASS);
		return VSTATUS_INVALID_MCLASS;
	}
	
	mad->base.bversion = MAD_BVERSION;
	mad->base.rsvd3    = 0;

	// copy MAD base header
	if (mad->active & MAI_ACT_BASE) {
		// Display the base header 
		if (IB_LOG_IS_INTERESTED(VS_LOG_INFO)) {
		IB_LOG_INFO("Base BaseVersion  ", mad->base.bversion);
		IB_LOG_INFO("     MgmtClass    ", mad->base.mclass);
		IB_LOG_INFO("     ClassVersion ", mad->base.cversion);
		IB_LOG_INFO("     Method       ", mad->base.method);
		IB_LOG_INFO("     Status       ", mad->base.status);
		IB_LOG_INFO("     HopPointer   ", mad->base.hopPointer);
		IB_LOG_INFO("     HopCount     ", mad->base.hopCount);
		IB_LOG_INFOLX("     TID          ", mad->base.tid & 0xffffffffu);
		IB_LOG_INFO("     AttributeID  ", mad->base.aid);
		IB_LOG_INFO("     AttributeMod ", mad->base.amod);
		IB_LOG_INFO_FMT(__func__,"FullTID: 0x%016"CS64"x", mad->base.tid);
		}

		if (isDrouted) {
			// clear status and ensure return bit is set if necessary
			mad->base.status = mad->base.method == MAD_CM_GET_RESP
			                   ? MAD_DR_RETURN : MAD_DR_INITIAL;
		}
		
		memcpy(buf, &mad->base, sizeof(mad->base));
		(void)BSWAP_MAD_HEADER((MAD *)buf); 
	}

	// Copy data to the end of the wire buffer 
	if (!(mad->active & MAI_ACT_DATA) && mad->datasize <= 0) {
		IB_LOG_ERROR("datasize size indeterminate:", mad->datasize);
		IB_EXIT(__func__, VSTATUS_INVALID_MADLEN);
		return VSTATUS_INVALID_MADLEN;
	} else {
		// we are constrained to send the number of bytes of the supported
		// payload at all times 
		if (mad->datasize && (mad->datasize != IB_MAD_PAYLOAD_SIZE)) {
			IB_LOG_ERROR("datasize out of range:", mad->datasize);
			IB_EXIT(__func__, VSTATUS_TOO_LARGE);
			return VSTATUS_TOO_LARGE;
		} else {
			// MAI_ACT_DATA not active; send the payload balance
			mad->datasize = IB_MAD_PAYLOAD_SIZE;
		}

		memcpy(buf + sizeof(MAD_COMMON), mad->data, mad->datasize);

		if (isDrouted) {
			// magic numbers: DR LIDs are 32 bytes into a DR SMP header and
			//                are 2 bytes long each
			*(uint16_t*)(buf + 32) = hton16(*(uint16_t*)(buf + 32));
			*(uint16_t*)(buf + 34) = hton16(*(uint16_t*)(buf + 34));
		}
	}

	// VENDOR MAD HACK:
	// We use SA MAD headers for vendor classes, but the new vendor range
	// requires a specific header with an OUI field 37 bytes into the MAD.
	// In the SA header, this would be in the middle of the SM Key.  Since
	// we're not using anything past the RMPP header for vendor traffic, we'll
	// just write the OUI directly into the MAD.
	if (  mad->base.mclass >= IBA_VENDOR_RANGE2_START
	   && mad->base.mclass <= IBA_VENDOR_RANGE2_END)
		memcpy(buf + 37, ib_truescale_oui, 3);
	// END HORRIBLE HORRIBLE HACK

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// stl_mai_to_wire  
//==============================================================================
static Status_t
stl_mai_to_wire(Mai_t *mad, uint8_t *buf, uint32_t *bufLen)
{
   int       isDrouted; 
   uint32_t  mask = 0; 
   
   IB_ENTER(__func__, mad, buf, 0, 0);
   
   // Validate the obvious parameters 
   if (mad == NULL) {
      IB_LOG_ERROR0("Invalid parameter: 'mad'"); 
      IB_EXIT(__func__, VSTATUS_ILLPARM); 
      return VSTATUS_ILLPARM;
   }
   
   // Make sure we have a MAD header 
   mask = MAI_ACT_TYPE; 
   if ((mad->active & mask) != mask) {
      IB_LOG_ERROR0("MAI_ACT_TYPE not active!"); 
      IB_EXIT(__func__, VSTATUS_INVALID_MADT); 
      return VSTATUS_INVALID_MADT;
   }
   
   // Validate the type of MAD 
   switch (mad->type) {
   case MAI_TYPE_EXTERNAL:
      IB_LOG_INFO("Type is MAD", mad->type); 
      // MADs require these fields 
      mask = MAI_ACT_ADDRINFO;
      if ((mad->active & mask) != mask) {
         IB_LOG_ERROR("Invalid Mai_t:", mad->active); 
         IB_EXIT(__func__, VSTATUS_INVALID_MAD); 
         return VSTATUS_INVALID_MAD;
      }
      break; 
   default:
      IB_LOG_ERROR("Invalid MAD type:", mad->type); 
      IB_EXIT(__func__, VSTATUS_INVALID_TYPE); 
      return VSTATUS_INVALID_TYPE;
   }
   
   // Figure out about Direct Routed or LID routed 
   isDrouted = 0;      /* Assume we are LID routed */
   if (mad->base.mclass == MAD_CV_SUBN_DR) 
      isDrouted = 1; 
   
   // Setup reasonable defaults
   // Check for 'Reserved' values in management class 
   if (mad->base.mclass == 0    || mad->base.mclass == 2
       || (mad->base.mclass >= 0x50 && mad->base.mclass <= 0x80)
       ||  mad->base.mclass >= 0x82) {
      IB_LOG_ERROR("Reserved management class:", 
                   mad->base.mclass); 
      IB_EXIT(__func__, VSTATUS_INVALID_MCLASS); 
      return VSTATUS_INVALID_MCLASS;
   }
   
   mad->base.bversion = STL_BASE_VERSION; 
   mad->base.rsvd3    = 0; 
   
   // copy MAD base header
   if (mad->active & MAI_ACT_BASE) {
      // Display the base header 
      if (IB_LOG_IS_INTERESTED(VS_LOG_INFO)) {
         IB_LOG_INFO("Base BaseVersion  ", mad->base.bversion); 
         IB_LOG_INFO("     MgmtClass    ", mad->base.mclass); 
         IB_LOG_INFO("     ClassVersion ", mad->base.cversion); 
         IB_LOG_INFO("     Method       ", mad->base.method); 
         IB_LOG_INFO("     Status       ", mad->base.status); 
         IB_LOG_INFO("     HopPointer   ", mad->base.hopPointer); 
         IB_LOG_INFO("     HopCount     ", mad->base.hopCount); 
         IB_LOG_INFOLX("     TID          ", mad->base.tid & 0xffffffffu); 
         IB_LOG_INFO("     AttributeID  ", mad->base.aid); 
         IB_LOG_INFO("     AttributeMod ", mad->base.amod); 
         IB_LOG_INFO_FMT(__func__, "FullTID: 0x%016"CS64"x", mad->base.tid);
      }
     
      if (isDrouted) {
         // clear status and ensure return bit is set if necessary
         mad->base.status = mad->base.method == MAD_CM_GET_RESP
            ? MAD_DR_RETURN : MAD_DR_INITIAL;
      }

      memcpy(buf, &mad->base, sizeof(mad->base));
      (void)BSWAP_MAD_HEADER((MAD *)buf); 
   }
   
   // Copy data to the end of the wire buffer 
   if (!(mad->active & MAI_ACT_DATA) && mad->datasize <= 0) {
       IB_LOG_ERROR("datasize size indeterminate:", mad->datasize); 
       IB_EXIT(__func__, VSTATUS_INVALID_MADLEN); 
       return VSTATUS_INVALID_MADLEN;
   } else {
       if (mad->datasize) {
           // assume the datasize field is valid
           if (mad->datasize > STL_MAD_PAYLOAD_SIZE) {
               IB_LOG_ERROR("datasize out of range:", mad->datasize); 
               IB_EXIT(__func__, VSTATUS_TOO_LARGE); 
               return VSTATUS_TOO_LARGE;
           }
       } else {
           // in order to handle requests from legacy code, check validity of
           // the datasize field.  if not valid then, set the datasize to max payload size;
           // otherwise datasize field is valid indicating a payload with no data.
           if (!(mad->active & MAI_ACT_DATASIZE)) 
               mad->datasize = STL_MAD_PAYLOAD_SIZE;
       }
       
       *bufLen = sizeof(mad->base) + mad->datasize;      
       memcpy(buf + sizeof(mad->base), mad->data, mad->datasize); 
       
       if (isDrouted) {
           STL_SMP *smp = (STL_SMP *)buf; 
           
#if CPU_LE
           smp->M_Key = ntoh64(smp->M_Key); 
           smp->SmpExt.DirectedRoute.DrSLID = ntoh32(smp->SmpExt.DirectedRoute.DrSLID); 
           smp->SmpExt.DirectedRoute.DrDLID = ntoh32(smp->SmpExt.DirectedRoute.DrDLID); 
#endif
       }
   }
   
   // VENDOR MAD HACK:
   // We use SA MAD headers for vendor classes, but the new vendor range
   // requires a specific header with an OUI field 37 bytes into the MAD.
   // In the SA header, this would be in the middle of the SM Key.  Since
   // we're not using anything past the RMPP header for vendor traffic, we'll
   // just write the OUI directly into the MAD.
   if (mad->base.mclass >= IBA_VENDOR_RANGE2_START
       && mad->base.mclass <= IBA_VENDOR_RANGE2_END) {
	 memcpy(buf + 37, ib_truescale_oui, 3); 
   }
   // END HORRIBLE HORRIBLE HACK
   
   IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

//==============================================================================
// stl_wire_to_mai  
//==============================================================================
static Status_t
stl_wire_to_mai(uint8_t *buf, Mai_t *mad, int len)
{ 
   IB_ENTER(__func__, 0, 0, 0, 0); 
   
   // Validate the obvious parameters 
   if (mad == NULL) {
      IB_LOG_ERROR0("NULL mad pointer passed"); 
      IB_EXIT(__func__, VSTATUS_ILLPARM); 
      return VSTATUS_ILLPARM;
   }
   if (buf == NULL) {
      IB_LOG_ERROR0("NULL buf pointer passed"); 
      IB_EXIT(__func__, VSTATUS_ILLPARM); 
      return VSTATUS_ILLPARM;
   }
   
   // clear out the active mask on the Mai_t struct 
   memset(mad, 0, sizeof(Mai_t)); 
   
   mad->type   = MAI_TYPE_EXTERNAL; // Assume type MAD until told otherwise 
   mad->active = MAI_ACT_TYPE; 
   
   (void)BSWAP_MAD_HEADER((MAD *)buf); 
   (void)stl_mad_to_mai((MAD *)buf, mad);

   // convert the data only if we can identify the header 
   if (mad->base.bversion != MAD_BVERSION && mad->base.bversion != STL_BASE_VERSION) {
      IB_LOG_ERROR("Unknown MAD version:", mad->base.bversion); 
      return VSTATUS_BAD;
   }
   
   // Check for 'Reserved' values in management class 
   if (mad->base.mclass == 0    || mad->base.mclass == 2
       || (mad->base.mclass >= 0x50 && mad->base.mclass <= 0x80)
       ||  mad->base.mclass >= 0x82) {
      IB_LOG_ERROR("Reserved management class:", 
                   mad->base.mclass); 
      return VSTATUS_BAD;
   }
   
   if (mad->base.mclass == MAD_CV_SUBN_DR
       && mad->base.hopPointer > IBA_MAX_PATHSIZE) {
      IB_LOG_ERROR("Unexpected hopPointer value:", 
                   mad->base.hopPointer); 
      return VSTATUS_BAD;
   }
   
   // We can safely say the MAD header is valid now.  
   mad->active |= MAI_ACT_BASE; 
   if (IB_LOG_IS_INTERESTED(VS_LOG_INFO)) {
      IB_LOG_INFO("Base BaseVersion  ", mad->base.bversion); 
      IB_LOG_INFO("    MgmtClass    ", mad->base.mclass); 
      IB_LOG_INFO("    ClassVersion ", mad->base.cversion); 
      IB_LOG_INFO("    Method       ", mad->base.method); 
      IB_LOG_INFO("    Status       ", mad->base.status); 
      IB_LOG_INFO("    HopPointer   ", mad->base.hopPointer); 
      IB_LOG_INFO("    HopCount     ", mad->base.hopCount); 
      IB_LOG_INFOLX("    TID          ", mad->base.tid); 
      IB_LOG_INFO("    AttributeID  ", mad->base.aid); 
      IB_LOG_INFO("    AttributeMod ", mad->base.amod); 
      IB_LOG_INFO_FMT(__func__, "FullTID: 0x%016"CS64"x", mad->base.tid);
   }
   
   // Copy the data, if no data leave it zeroed from memset above
   if (len > sizeof(MAD_COMMON))
      memcpy(mad->data, buf + sizeof(MAD_COMMON), len - sizeof(MAD_COMMON));
   mad->datasize = (mad->base.bversion == MAD_BVERSION) ? IB_MAD_PAYLOAD_SIZE : len; 
   
   (void)vs_time_get(&mad->intime); 
   
   mad->active |= MAI_ACT_DATA | MAI_ACT_TSTAMP | MAI_ACT_FMASK; 
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return VSTATUS_OK; 
}

